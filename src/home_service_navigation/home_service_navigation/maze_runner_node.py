#!/usr/bin/env python3
"""Ejecutor del Reto 4 (Laberinto): START -> FINISH de forma autonoma.

Responsabilidades
=================
1. Recoger el brazo a una pose segura antes de entrar al laberinto
   (en pasillos de 60 cm el MechArm desplegado es una colision segura).
2. Enviar el objetivo FINISH a Nav2 y vigilarlo hasta el final.
3. Detectar bloqueos ("el robot se queda colgado cerca de una pared") y
   aplicar una escalera de recuperacion SIN girar sobre si mismo:
       nivel 1 -> limpiar el costmap local
       nivel 2 -> limpiar ambos costmaps
       nivel 3 -> cancelar, limpiar y reenviar el objetivo
   Se evita deliberadamente la recuperacion 'spin' de Nav2: en este
   robot mecanum el giro es justo lo que descuadra la odometria y
   genera las paredes fantasma.
4. Publicar estado legible en /maze/status.

Arranque: automatico (``auto_start:=true``) o bajo demanda llamando al
servicio ``/maze/start`` (std_srvs/Trigger).
"""

import math
import threading
import time
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MazeRunner(Node):

    def __init__(self):
        super().__init__('maze_runner')

        self.cb_group = ReentrantCallbackGroup()

        # -----------------------------------------------------------------
        # Objetivo
        # -----------------------------------------------------------------
        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', -2.5)
        self.declare_parameter('goal_yaw_deg', 0.0)
        self.declare_parameter('goal_frame', 'map')

        # -----------------------------------------------------------------
        # Arranque
        # -----------------------------------------------------------------
        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay_sec', 5.0)

        # -----------------------------------------------------------------
        # Brazo
        # -----------------------------------------------------------------
        self.declare_parameter('tuck_arm', True)
        self.declare_parameter('tuck_pose', 'home')
        self.declare_parameter('tuck_timeout_sec', 8.0)

        # -----------------------------------------------------------------
        # Deteccion de bloqueo
        # -----------------------------------------------------------------
        # Si en 'stuck_window_sec' el robot no se ha desplazado mas de
        # 'stuck_min_progress_m', se considera bloqueado.
        self.declare_parameter('stuck_window_sec', 6.0)
        self.declare_parameter('stuck_min_progress_m', 0.05)
        self.declare_parameter('max_recoveries', 8)
        # Tiempo de gracia tras una recuperacion antes de volver a medir.
        self.declare_parameter('recovery_settle_sec', 3.0)

        # -----------------------------------------------------------------
        # Higiene de costmaps
        # -----------------------------------------------------------------
        # Limpieza periodica preventiva del costmap global.
        # 0.0 = desactivada (recomendado: el scan_sanitizer arregla la
        # causa raiz, esto es solo una red de seguridad).
        self.declare_parameter('periodic_clear_sec', 0.0)

        # -----------------------------------------------------------------
        # Nombres
        # -----------------------------------------------------------------
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter(
            'clear_local_service',
            '/local_costmap/clear_entirely_local_costmap',
        )
        self.declare_parameter(
            'clear_global_service',
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.declare_parameter('move_arm_action', '/mecharm/move_arm')

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = math.radians(
            float(self.get_parameter('goal_yaw_deg').value)
        )
        self.goal_frame = str(self.get_parameter('goal_frame').value)

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.start_delay = float(
            self.get_parameter('start_delay_sec').value
        )

        self.tuck_arm = bool(self.get_parameter('tuck_arm').value)
        self.tuck_pose = str(self.get_parameter('tuck_pose').value)
        self.tuck_timeout = float(
            self.get_parameter('tuck_timeout_sec').value
        )

        self.stuck_window = float(
            self.get_parameter('stuck_window_sec').value
        )
        self.stuck_min_progress = float(
            self.get_parameter('stuck_min_progress_m').value
        )
        self.max_recoveries = int(
            self.get_parameter('max_recoveries').value
        )
        self.recovery_settle = float(
            self.get_parameter('recovery_settle_sec').value
        )

        self.periodic_clear = float(
            self.get_parameter('periodic_clear_sec').value
        )

        # -----------------------------------------------------------------
        # Estado
        # -----------------------------------------------------------------
        self._pose_history = deque()
        self._history_lock = threading.Lock()
        self._last_recovery_time = 0.0
        self._recoveries = 0
        self._navigating = False
        self._start_event = threading.Event()
        self._distance_remaining = float('nan')

        # -----------------------------------------------------------------
        # ROS
        # -----------------------------------------------------------------
        self.status_pub = self.create_publisher(String, '/maze/status', 10)

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            qos_profile_sensor_data,
            callback_group=self.cb_group,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('nav_action').value),
            callback_group=self.cb_group,
        )

        self.arm_client = None
        if self.tuck_arm:
            move_arm_type = _load_move_arm()
            if move_arm_type is not None:
                self.arm_client = ActionClient(
                    self,
                    move_arm_type,
                    str(self.get_parameter('move_arm_action').value),
                    callback_group=self.cb_group,
                )

        self.clear_local = self.create_client(
            ClearEntireCostmap,
            str(self.get_parameter('clear_local_service').value),
            callback_group=self.cb_group,
        )
        self.clear_global = self.create_client(
            ClearEntireCostmap,
            str(self.get_parameter('clear_global_service').value),
            callback_group=self.cb_group,
        )

        self.create_service(
            Trigger,
            '/maze/start',
            self._start_service,
            callback_group=self.cb_group,
        )

        if self.periodic_clear > 0.0:
            self.create_timer(
                self.periodic_clear,
                self._periodic_clear_tick,
                callback_group=self.cb_group,
            )

        self.get_logger().info(
            f'maze_runner listo. FINISH = '
            f'({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'@ {math.degrees(self.goal_yaw):.0f} deg '
            f'en "{self.goal_frame}"'
        )
        if self.auto_start:
            self.get_logger().info(
                f'Arranque automatico en {self.start_delay:.0f} s. '
                f'Para arrancar a mano: auto_start:=false y luego '
                f'"ros2 service call /maze/start std_srvs/srv/Trigger"'
            )

    # =====================================================================
    # Estado / utilidades
    # =====================================================================

    def _status(self, text, level='info'):
        # NO usar getattr(self.get_logger(), level)(text): rclpy cachea
        # el estado de logging por punto de llamada (fichero+linea) para
        # soportar throttle/once/skip_first, y esa cache asocia una
        # UNICA severidad a esa linea. Con una sola linea sirviendo a
        # 'info', 'warn' y 'error' segun el 'level' de turno, la segunda
        # severidad distinta que llega revienta con
        # "ValueError: Logger severity cannot be changed between calls"
        # -> maze_runner_node moria en el primer _status(..., 'error').
        # Cada rama de abajo es su propio punto de llamada con severidad
        # fija: asi rclpy no se queja y encima queda mas explicito.
        logger = self.get_logger()
        if level == 'error':
            logger.error(text)
        elif level == 'warn':
            logger.warn(text)
        elif level == 'debug':
            logger.debug(text)
        else:
            logger.info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _odom_callback(self, msg):
        now = time.monotonic()
        position = msg.pose.pose.position
        with self._history_lock:
            self._pose_history.append((now, position.x, position.y))
            cutoff = now - max(self.stuck_window * 2.0, 10.0)
            while self._pose_history and self._pose_history[0][0] < cutoff:
                self._pose_history.popleft()

    def _progress_in_window(self):
        """Desplazamiento maximo observado en la ventana de vigilancia.

        Devuelve None si aun no hay historial suficiente.
        """
        now = time.monotonic()
        with self._history_lock:
            samples = [
                item
                for item in self._pose_history
                if item[0] >= now - self.stuck_window
            ]

        if len(samples) < 2:
            return None
        if samples[-1][0] - samples[0][0] < self.stuck_window * 0.8:
            return None

        _, x0, y0 = samples[0]
        return max(math.hypot(x - x0, y - y0) for _, x, y in samples)

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def _start_service(self, _request, response):
        self._start_event.set()
        response.success = True
        response.message = 'Recorrido del laberinto iniciado.'
        return response

    # =====================================================================
    # Limpieza de costmaps
    # =====================================================================

    def _call_clear(self, client, label):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'Servicio {label} no disponible; se omite la limpieza.'
            )
            return False

        future = client.call_async(ClearEntireCostmap.Request())
        if not self._wait_future(future, 5.0):
            self.get_logger().warn(f'Timeout limpiando {label}.')
            return False
        return True

    def _clear_local_costmap(self):
        return self._call_clear(self.clear_local, 'costmap local')

    def _clear_global_costmap(self):
        return self._call_clear(self.clear_global, 'costmap global')

    def _periodic_clear_tick(self):
        if not self._navigating:
            return
        self._clear_global_costmap()
        self.get_logger().debug('Limpieza periodica del costmap global.')

    # =====================================================================
    # Brazo
    # =====================================================================

    def _tuck_arm(self):
        if not self.tuck_arm:
            return

        if self.arm_client is None:
            self.get_logger().warn(
                'tuck_arm activo pero home_service_interfaces no esta '
                'disponible; se continua sin recoger el brazo.'
            )
            return

        self._status(
            f'Recogiendo el brazo a la pose "{self.tuck_pose}"...'
        )

        if not self.arm_client.wait_for_server(timeout_sec=self.tuck_timeout):
            self.get_logger().warn(
                'El driver del MechArm no responde; se continua sin '
                'recoger el brazo. ASEGURATE de que esta plegado.'
            )
            return

        goal_type = _load_move_arm()
        goal = goal_type.Goal()
        goal.pose_name = self.tuck_pose

        send_future = self.arm_client.send_goal_async(goal)
        if not self._wait_future(send_future, self.tuck_timeout):
            self.get_logger().warn('Timeout enviando la pose del brazo.')
            return

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('El brazo rechazo la pose de recogida.')
            return

        result_future = handle.get_result_async()
        if not self._wait_future(result_future, self.tuck_timeout + 15.0):
            self.get_logger().warn('Timeout esperando a que el brazo llegue.')
            return

        self._status('Brazo recogido.')

    # =====================================================================
    # Navegacion
    # =====================================================================

    def _build_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.goal_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        goal.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        return goal

    def _nav_feedback(self, feedback_msg):
        self._distance_remaining = float(
            feedback_msg.feedback.distance_remaining
        )

    def _send_goal(self):
        send_future = self.nav_client.send_goal_async(
            self._build_goal(), feedback_callback=self._nav_feedback
        )
        if not self._wait_future(send_future, 10.0):
            self._status('Timeout enviando el objetivo a Nav2.', 'error')
            return None

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self._status('Nav2 rechazo el objetivo.', 'error')
            return None

        self._status('Objetivo aceptado por Nav2. Navegando el laberinto.')
        return handle

    def _recover(self, handle):
        """Escalera de recuperacion. Devuelve el handle activo (o uno
        nuevo si hubo que reenviar el objetivo), o None si se agoto.
        """
        self._recoveries += 1
        level = ((self._recoveries - 1) % 3) + 1

        if self._recoveries > self.max_recoveries:
            self._status(
                f'Se agotaron las {self.max_recoveries} recuperaciones.',
                'error',
            )
            return None

        self._status(
            f'BLOQUEADO ({self._recoveries}/{self.max_recoveries}). '
            f'Recuperacion nivel {level}.',
            'warn',
        )

        if level == 1:
            self._clear_local_costmap()
        elif level == 2:
            self._clear_local_costmap()
            self._clear_global_costmap()
        else:
            self._status('Cancelando y reenviando el objetivo.', 'warn')
            cancel_future = handle.cancel_goal_async()
            self._wait_future(cancel_future, 5.0)
            self._clear_local_costmap()
            self._clear_global_costmap()
            time.sleep(1.0)
            handle = self._send_goal()
            if handle is None:
                return None

        self._last_recovery_time = time.monotonic()
        time.sleep(self.recovery_settle)
        return handle

    def run(self):
        # 1. Esperar el disparo de arranque.
        if self.auto_start:
            self._status(
                f'Arrancando en {self.start_delay:.0f} s...'
            )
            self._start_event.wait(timeout=self.start_delay)
        else:
            self._status('Esperando a /maze/start ...')
            while rclpy.ok() and not self._start_event.wait(timeout=0.5):
                pass

        if not rclpy.ok():
            return False

        # 2. Recoger el brazo.
        self._tuck_arm()

        # 3. Esperar a Nav2.
        self._status('Esperando al servidor de accion de Nav2...')
        while rclpy.ok():
            if self.nav_client.wait_for_server(timeout_sec=2.0):
                break
            self.get_logger().warn('Nav2 todavia no esta disponible...')

        if not rclpy.ok():
            return False

        # 4. Empezar con costmaps limpios: cualquier fantasma acumulado
        #    durante el arranque desaparece antes de planificar.
        self._clear_local_costmap()
        self._clear_global_costmap()

        handle = self._send_goal()
        if handle is None:
            return False

        self._navigating = True
        result_future = handle.get_result_async()
        self._last_recovery_time = time.monotonic()

        # 5. Vigilancia hasta el final.
        while rclpy.ok():
            if result_future.done():
                break

            time.sleep(0.5)

            since_recovery = time.monotonic() - self._last_recovery_time
            if since_recovery < max(self.stuck_window, self.recovery_settle):
                continue

            progress = self._progress_in_window()
            if progress is None:
                continue

            if progress >= self.stuck_min_progress:
                continue

            new_handle = self._recover(handle)
            if new_handle is None:
                self._navigating = False
                cancel_future = handle.cancel_goal_async()
                self._wait_future(cancel_future, 5.0)
                self._status('LABERINTO FALLIDO: bloqueo irrecuperable.',
                             'error')
                return False

            if new_handle is not handle:
                handle = new_handle
                result_future = handle.get_result_async()

        self._navigating = False

        if not rclpy.ok():
            return False

        response = result_future.result()
        if response is None:
            self._status('Nav2 no devolvio resultado.', 'error')
            return False

        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self._status(
                f'FINISH ALCANZADO. Recuperaciones usadas: '
                f'{self._recoveries}.'
            )
            return True

        self._status(
            f'LABERINTO FALLIDO: Nav2 termino con status='
            f'{response.status}.',
            'error',
        )
        return False


def _load_move_arm():
    """Importa MoveArm solo si esta disponible.

    Permite usar el laberinto sin el paquete del brazo compilado.
    """
    try:
        from home_service_interfaces.action import MoveArm
        return MoveArm
    except Exception:  # noqa: BLE001
        return None


def main(args=None):
    rclpy.init(args=args)
    node = MazeRunner()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
