#!/usr/bin/env python3
"""Driver ROS 2 del brazo MechArm 270 M5 del myAGV.

Expone dos acciones:

* ``/mecharm/move_arm``   (home_service_interfaces/action/MoveArm)
    Mueve el brazo a una pose con nombre, a angulos articulares o a
    coordenadas cartesianas.

* ``/mecharm/pick_place`` (home_service_interfaces/action/PickPlace)
    Primitiva de toma ('pick') o colocacion ('place') con el gripper.
    El posicionamiento de la base movil corre por cuenta de quien invoca.

El acceso al puerto serie de pymycobot NO es seguro entre hilos: todas
las llamadas se serializan con un cerrojo.
"""

import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_srvs.srv import SetBool

import yaml

from home_service_interfaces.action import MoveArm, PickPlace
from home_service_interfaces.srv import SetGripper

try:
    from pymycobot.mecharm270 import MechArm270
    _PYMYCOBOT_IMPORT_ERROR = None
except Exception as exc:  # noqa: BLE001  - entorno sin pymycobot
    MechArm270 = None
    _PYMYCOBOT_IMPORT_ERROR = exc


def clamp(value, low, high):
    return max(low, min(high, value))


class MechArmDriver(Node):

    def __init__(self):
        super().__init__("mecharm_driver_node")

        # -----------------------------------------------------------------
        # Parametros
        # -----------------------------------------------------------------
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("reconnect_period_sec", 3.0)

        self.declare_parameter("default_speed_percent", 40.0)
        self.declare_parameter("max_speed_percent", 60.0)
        self.declare_parameter("default_gripper_speed_percent", 50.0)

        self.declare_parameter("gripper_open_value", 100)
        self.declare_parameter("gripper_closed_value", 20)
        # pymycobot exige el tipo en algunas versiones aunque la API lo
        # documente como opcional. 1 = pinza adaptativa del MechArm.
        self.declare_parameter("gripper_type", 1)

        self.declare_parameter("move_timeout_sec", 20.0)
        self.declare_parameter("gripper_timeout_sec", 5.0)
        self.declare_parameter("settle_time_sec", 0.4)
        # Tiempo extra tras cerrar/abrir la pinza para que asiente.
        self.declare_parameter("gripper_settle_sec", 0.6)

        # --- Criterio de llegada (convergencia de posicion) ---
        # La repetibilidad del MechArm 270 es +-0.5 mm, pero la lectura
        # por serie tiene mas ruido: 2 grados / 6 mm son realistas.
        self.declare_parameter("angle_tolerance_deg", 2.0)
        self.declare_parameter("coord_tolerance_mm", 6.0)
        # Lecturas consecutivas dentro de tolerancia para dar por
        # alcanzado el objetivo.
        self.declare_parameter("arrival_stable_samples", 3)
        # Lecturas de posicion fallidas seguidas antes de dar el brazo
        # por perdido (get_angles/get_coords devuelven None a menudo).
        self.declare_parameter("max_read_failures", 25)
        # Si el error deja de reducirse durante este tiempo, se aborta:
        # obstruccion, limite articular o pose fuera del alcance.
        self.declare_parameter("stall_timeout_sec", 4.0)

        self.declare_parameter(
            "joint_limits_min",
            [-160.0, -85.0, -180.0, -160.0, -100.0, -180.0],
        )
        self.declare_parameter(
            "joint_limits_max",
            [160.0, 90.0, 45.0, 160.0, 100.0, 180.0],
        )

        self.declare_parameter("poses_file", "")
        self.declare_parameter("verify_grasp", False)

        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_states_rate_hz", 5.0)
        self.declare_parameter(
            "joint_names",
            [
                "joint1_to_base",
                "joint2_to_joint1",
                "joint3_to_joint2",
                "joint4_to_joint3",
                "joint5_to_joint4",
                "joint6_to_joint5",
            ],
        )

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.reconnect_period = float(
            self.get_parameter("reconnect_period_sec").value
        )

        self.default_speed = float(
            self.get_parameter("default_speed_percent").value
        )
        self.max_speed = float(
            self.get_parameter("max_speed_percent").value
        )
        self.default_gripper_speed = float(
            self.get_parameter("default_gripper_speed_percent").value
        )

        self.gripper_open_value = int(
            self.get_parameter("gripper_open_value").value
        )
        self.gripper_closed_value = int(
            self.get_parameter("gripper_closed_value").value
        )
        self.gripper_type = int(self.get_parameter("gripper_type").value)

        self.move_timeout = float(
            self.get_parameter("move_timeout_sec").value
        )
        self.gripper_timeout = float(
            self.get_parameter("gripper_timeout_sec").value
        )
        self.settle_time = float(
            self.get_parameter("settle_time_sec").value
        )
        self.gripper_settle = float(
            self.get_parameter("gripper_settle_sec").value
        )
        self.angle_tolerance = float(
            self.get_parameter("angle_tolerance_deg").value
        )
        self.coord_tolerance = float(
            self.get_parameter("coord_tolerance_mm").value
        )
        self.arrival_stable_samples = int(
            self.get_parameter("arrival_stable_samples").value
        )
        self.max_read_failures = int(
            self.get_parameter("max_read_failures").value
        )
        self.stall_timeout = float(
            self.get_parameter("stall_timeout_sec").value
        )

        self.joint_min = [
            float(v) for v in self.get_parameter("joint_limits_min").value
        ]
        self.joint_max = [
            float(v) for v in self.get_parameter("joint_limits_max").value
        ]

        self.verify_grasp = bool(self.get_parameter("verify_grasp").value)

        self.joint_names = [
            str(v) for v in self.get_parameter("joint_names").value
        ]

        # -----------------------------------------------------------------
        # Poses
        # -----------------------------------------------------------------
        self.poses = self._load_poses(
            str(self.get_parameter("poses_file").value)
        )

        # -----------------------------------------------------------------
        # Conexion con el brazo
        # -----------------------------------------------------------------
        self.mc = None
        self._serial_lock = threading.Lock()

        if MechArm270 is None:
            self.get_logger().error(
                "pymycobot no disponible "
                f"({_PYMYCOBOT_IMPORT_ERROR}). El nodo arranca pero las "
                "acciones devolveran ARM_FAULT hasta que se instale "
                "(pip3 install pymycobot)."
            )

        self._try_connect()

        self.reconnect_timer = self.create_timer(
            self.reconnect_period, self._reconnect_tick
        )

        # -----------------------------------------------------------------
        # joint_states de depuracion
        # -----------------------------------------------------------------
        if bool(self.get_parameter("publish_joint_states").value):
            rate = float(
                self.get_parameter("joint_states_rate_hz").value
            )
            rate = rate if rate > 0.0 else 5.0
            self.joint_state_pub = self.create_publisher(
                JointState, "/mecharm/joint_states", 10
            )
            self.create_timer(1.0 / rate, self._publish_joint_states)
        else:
            self.joint_state_pub = None

        # -----------------------------------------------------------------
        # Acciones
        # -----------------------------------------------------------------
        self.cb_group = ReentrantCallbackGroup()

        self.move_server = ActionServer(
            self,
            MoveArm,
            "/mecharm/move_arm",
            execute_callback=self._execute_move_arm,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self.cb_group,
        )

        self.pick_place_server = ActionServer(
            self,
            PickPlace,
            "/mecharm/pick_place",
            execute_callback=self._execute_pick_place,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self.cb_group,
        )

        # Un unico goal activo a la vez (el brazo es un recurso unico).
        self._busy_lock = threading.Lock()
        self._busy = False

        # -----------------------------------------------------------------
        # Servicios de prueba / calibracion
        # -----------------------------------------------------------------
        self.create_service(
            SetGripper,
            "/mecharm/set_gripper",
            self._srv_set_gripper,
            callback_group=self.cb_group,
        )

        self.create_service(
            SetBool,
            "/mecharm/free_move",
            self._srv_free_move,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "mecharm_driver_node listo. Acciones: /mecharm/move_arm, "
            "/mecharm/pick_place"
        )

    # =====================================================================
    # Poses
    # =====================================================================

    def _load_poses(self, poses_file):
        if not poses_file:
            try:
                from ament_index_python.packages import (
                    get_package_share_directory,
                )

                poses_file = os.path.join(
                    get_package_share_directory("myagv_mecharm_service"),
                    "config",
                    "poses.yaml",
                )
            except Exception:  # noqa: BLE001
                poses_file = ""

        poses_file = os.path.expanduser(poses_file)

        if not poses_file or not os.path.isfile(poses_file):
            self.get_logger().warn(
                f"poses.yaml no encontrado ({poses_file}); no habra "
                "poses con nombre."
            )
            return {}

        try:
            with open(poses_file, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}

            raw = data.get("poses", {})
            poses = {}
            for name, angles in raw.items():
                if not isinstance(angles, (list, tuple)) or len(angles) != 6:
                    self.get_logger().warn(
                        f"Pose '{name}' ignorada: se esperan 6 angulos."
                    )
                    continue
                poses[str(name)] = [float(a) for a in angles]

            self.get_logger().info(
                f"Poses cargadas: {sorted(poses.keys())}"
            )
            return poses

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error leyendo poses.yaml: {exc}")
            return {}

    # =====================================================================
    # Conexion
    # =====================================================================

    def _try_connect(self):
        if self.mc is not None or MechArm270 is None:
            return

        if not os.path.exists(self.port):
            self.get_logger().warn(
                f"Puerto {self.port} no existe todavia.",
                throttle_duration_sec=10.0,
            )
            return

        try:
            with self._serial_lock:
                mc = MechArm270(self.port, self.baud)
                try:
                    mc.power_on()
                except Exception:  # noqa: BLE001
                    pass
                try:
                    mc.set_fresh_mode(1)
                except Exception:  # noqa: BLE001
                    pass
                try:
                    mc.init_gripper()
                except Exception:  # noqa: BLE001
                    pass
            self.mc = mc
            self.get_logger().info(
                f"Conectado al MechArm 270 en {self.port}."
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"No se pudo conectar al MechArm ({exc}); "
                "reintentando."
            )
            self.mc = None

    def _reconnect_tick(self):
        if self.mc is None:
            self._try_connect()

    def _drop_connection(self, reason):
        self.get_logger().error(
            f"Conexion con el brazo perdida: {reason}"
        )
        self.mc = None

    # =====================================================================
    # Llamadas serializadas a pymycobot
    # =====================================================================

    def _arm(self, method_name, *args, **kwargs):
        """Llama a un metodo de pymycobot bajo cerrojo. Devuelve el
        resultado, o levanta RuntimeError si el brazo no esta disponible.
        """
        if self.mc is None:
            raise RuntimeError("brazo no conectado")

        method = getattr(self.mc, method_name, None)
        if method is None:
            raise RuntimeError(f"pymycobot no expone '{method_name}'")

        with self._serial_lock:
            return method(*args, **kwargs)

    def _read_angles(self, retries=10, delay=0.15):
        for _ in range(retries):
            try:
                angles = self._arm("get_angles")
            except RuntimeError:
                return None
            if isinstance(angles, (list, tuple)) and len(angles) == 6:
                return [float(a) for a in angles]
            time.sleep(delay)
        return None

    def _read_coords(self, retries=15, delay=0.2):
        for _ in range(retries):
            try:
                coords = self._arm("get_coords")
            except RuntimeError:
                return None
            if isinstance(coords, (list, tuple)) and len(coords) == 6:
                return [float(c) for c in coords]
            time.sleep(delay)
        return None

    def _wait_for_arrival(self, goal_handle, target, kind, timeout):
        """Espera a que el brazo LLEGUE al objetivo.

        Devuelve ('ok'|'timeout'|'canceled'|'fault').

        Por que no basta con is_moving():
          * is_moving() devuelve -1 de forma intermitente cuando la
            lectura por serie falla. Tratarlo como fallo aborta
            movimientos que en realidad iban bien.
          * is_moving() puede devolver 0 antes de que el brazo arranque,
            dando por terminado un movimiento que no ha empezado.

        Aqui el criterio principal es la CONVERGENCIA DE POSICION: se
        lee la posicion real y se compara con el objetivo. is_moving()
        se usa solo como senal secundaria. Las lecturas fallidas se
        toleran hasta 'max_read_failures' seguidas.
        """
        if kind == "angles":
            tolerance = self.angle_tolerance
            reader = self._read_angles
            components = 6
        else:
            tolerance = self.coord_tolerance
            reader = self._read_coords
            # Solo se comprueba X, Y, Z: la orientacion del efector
            # converge mas despacio y no condiciona el agarre.
            components = 3

        deadline = time.monotonic() + timeout
        # Margen para que el brazo arranque antes de evaluar nada.
        time.sleep(0.3)

        stable = 0
        read_failures = 0
        last_error = None
        last_progress_time = time.monotonic()

        while time.monotonic() < deadline:
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return "canceled"

            current = reader(retries=1, delay=0.0)

            if current is None:
                read_failures += 1
                if read_failures >= self.max_read_failures:
                    self.get_logger().error(
                        f"{read_failures} lecturas de posicion fallidas "
                        f"seguidas: se da el brazo por perdido."
                    )
                    return "fault"
                time.sleep(0.1)
                continue

            read_failures = 0

            error = max(
                abs(current[i] - target[i]) for i in range(components)
            )

            if error <= tolerance:
                stable += 1
                if stable >= self.arrival_stable_samples:
                    time.sleep(self.settle_time)
                    return "ok"
            else:
                stable = 0

            # Deteccion de atasco: si el error deja de reducirse durante
            # mucho tiempo, el brazo no va a llegar (obstruccion, limite
            # articular, objetivo fuera del alcance de 270 mm).
            if last_error is None or error < last_error - tolerance * 0.25:
                last_error = error
                last_progress_time = time.monotonic()
            elif time.monotonic() - last_progress_time > self.stall_timeout:
                self.get_logger().warn(
                    f"El brazo dejo de acercarse al objetivo "
                    f"(error {error:.2f}, tolerancia {tolerance:.2f}). "
                    f"Posible obstruccion o pose inalcanzable."
                )
                return "timeout"

            time.sleep(0.1)

        return "timeout"

    def _wait_gripper_idle(self, timeout):
        """Espera a que el gripper deje de moverse.

        is_gripper_moving() es poco fiable en muchas pinzas: si no da una
        respuesta clara se asume que termino tras el timeout. Nunca se
        devuelve 'fault' por esto, para no abortar un agarre correcto.
        """
        deadline = time.monotonic() + timeout
        time.sleep(0.2)
        while time.monotonic() < deadline:
            try:
                moving = self._arm("is_gripper_moving")
            except RuntimeError:
                return "ok"
            if moving == 0:
                break
            time.sleep(0.1)

        # Tiempo extra para que la pinza asiente sobre la pieza.
        time.sleep(self.gripper_settle)
        return "ok"

    # =====================================================================
    # Movimientos de alto nivel
    # =====================================================================

    def _resolve_speed(self, requested):
        speed = requested if requested and requested > 0.0 else self.default_speed
        return int(round(clamp(speed, 1.0, self.max_speed)))

    def _clamp_joints(self, angles):
        out = []
        for i, a in enumerate(angles):
            lo = self.joint_min[i] if i < len(self.joint_min) else -180.0
            hi = self.joint_max[i] if i < len(self.joint_max) else 180.0
            out.append(clamp(float(a), lo, hi))
        return out

    def _move_angles(self, goal_handle, angles, speed):
        angles = self._clamp_joints(angles)
        try:
            self._arm("send_angles", angles, speed)
        except RuntimeError as exc:
            self._drop_connection(str(exc))
            return "fault"
        return self._wait_for_arrival(
            goal_handle, angles, "angles", self.move_timeout
        )

    def _move_coords(self, goal_handle, coords, speed, mode):
        coords = [float(c) for c in coords]
        # pymycobot: mode 0 = angular (trayectoria libre),
        #            mode 1 = lineal (linea recta).
        try:
            self._arm("send_coords", coords, speed, int(mode))
        except RuntimeError as exc:
            self._drop_connection(str(exc))
            return "fault"
        return self._wait_for_arrival(
            goal_handle, coords, "coords", self.move_timeout
        )

    def _set_gripper(self, value, speed):
        """Mueve la pinza a una apertura 0..100 (0 cerrada, 100 abierta).

        Se usa set_gripper_value porque es inequivoco. Si esa via falla
        se recurre a set_gripper_state, cuyo flag es 0 = abrir y
        1 = cerrar (ojo: es al reves de lo que suele suponerse).
        """
        value = int(round(clamp(value, 0.0, 100.0)))
        speed = int(round(clamp(speed, 1.0, 100.0)))

        try:
            self._arm("set_gripper_value", value, speed, self.gripper_type)
        except RuntimeError as exc:
            self._drop_connection(str(exc))
            return "fault"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"set_gripper_value fallo ({exc}); se prueba "
                f"set_gripper_state."
            )
            try:
                flag = 0 if value >= 50 else 1
                self._arm("set_gripper_state", flag, speed, self.gripper_type)
            except Exception as exc2:  # noqa: BLE001
                self.get_logger().error(f"set_gripper_state fallo: {exc2}")
                return "fault"

        return self._wait_gripper_idle(self.gripper_timeout)

    # =====================================================================
    # Servicios de prueba / calibracion
    # =====================================================================

    def _srv_set_gripper(self, request, response):
        """Mueve la pinza directamente. Util para calibrar los valores
        de apertura/cierre antes de tocar una mision.
        """
        response.current_value = -1

        if self.mc is None:
            response.success = False
            response.message = "Brazo no conectado."
            return response

        if not self._acquire_busy():
            response.success = False
            response.message = "El brazo esta ocupado con otro objetivo."
            return response

        try:
            speed = (
                request.speed_percent
                if request.speed_percent and request.speed_percent > 0.0
                else self.default_gripper_speed
            )
            value = int(round(clamp(float(request.value), 0.0, 100.0)))

            outcome = self._set_gripper(value, speed)

            try:
                read = self._arm("get_gripper_value")
                if isinstance(read, (int, float)) and read >= 0:
                    response.current_value = int(read)
            except Exception:  # noqa: BLE001
                pass

            if outcome == "fault":
                response.success = False
                response.message = "Fallo al mover la pinza."
            else:
                response.success = True
                response.message = (
                    f"Pinza a {value} "
                    f"(lectura: {response.current_value})."
                )
            return response

        finally:
            self._release_busy()

    def _srv_free_move(self, request, response):
        """Libera (True) o vuelve a alimentar (False) los servos.

        Con los servos liberados el brazo se puede mover A MANO, que es
        como se ensenan las poses de poses.yaml.

        CUIDADO: al liberar, el brazo CAE por su propio peso. Sujetalo
        antes de llamar a este servicio.
        """
        if self.mc is None:
            response.success = False
            response.message = "Brazo no conectado."
            return response

        if not self._acquire_busy():
            response.success = False
            response.message = "El brazo esta ocupado con otro objetivo."
            return response

        try:
            if request.data:
                self.get_logger().warn(
                    "LIBERANDO SERVOS: sujeta el brazo, va a caer por su "
                    "propio peso."
                )
                self._arm("release_all_servos")
                response.success = True
                response.message = (
                    "Servos liberados: mueve el brazo a mano y lee la "
                    "pose en /mecharm/joint_states."
                )
            else:
                self._arm("power_on")
                response.success = True
                response.message = "Servos alimentados de nuevo."
            return response

        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"Error: {exc}"
            return response

        finally:
            self._release_busy()

    # =====================================================================
    # Goal / cancel comunes
    # =====================================================================

    def _accept_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _acquire_busy(self):
        with self._busy_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _release_busy(self):
        with self._busy_lock:
            self._busy = False

    # =====================================================================
    # Accion MoveArm
    # =====================================================================

    def _execute_move_arm(self, goal_handle):
        req = goal_handle.request

        result = MoveArm.Result()

        if not self._acquire_busy():
            result.success = False
            result.status = "ARM_FAULT"
            result.message = "El brazo ya esta ejecutando otro objetivo."
            goal_handle.abort()
            return result

        try:
            targets = [
                bool(req.pose_name),
                len(req.joint_angles) == 6,
                len(req.coords) == 6,
            ]
            if sum(targets) != 1:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "Rellena exactamente uno: pose_name, joint_angles(6) "
                    "o coords(6)."
                )
                goal_handle.abort()
                return result

            if self.mc is None:
                result.success = False
                result.status = "ARM_FAULT"
                result.message = "Brazo no conectado."
                goal_handle.abort()
                return result

            speed = self._resolve_speed(req.speed_percent)

            self._publish_move_feedback(goal_handle, "MOVING")

            if req.pose_name:
                if req.pose_name not in self.poses:
                    result.success = False
                    result.status = "INVALID_GOAL"
                    result.message = (
                        f"Pose desconocida: '{req.pose_name}'. "
                        f"Disponibles: {sorted(self.poses.keys())}"
                    )
                    goal_handle.abort()
                    return result
                outcome = self._move_angles(
                    goal_handle, self.poses[req.pose_name], speed
                )
            elif len(req.joint_angles) == 6:
                outcome = self._move_angles(
                    goal_handle, list(req.joint_angles), speed
                )
            else:
                outcome = self._move_coords(
                    goal_handle, list(req.coords), speed, req.move_mode
                )

            self._publish_move_feedback(goal_handle, "SETTLING")

            final_angles = self._read_angles() or []

            result.final_joint_angles = final_angles

            if outcome == "ok":
                result.success = True
                result.status = "OK"
                result.message = "Movimiento completado."
                goal_handle.succeed()
            elif outcome == "canceled":
                result.success = False
                result.status = "CANCELED"
                result.message = "Objetivo cancelado."
                goal_handle.canceled()
            elif outcome == "timeout":
                result.success = False
                result.status = "TIMEOUT"
                result.message = "Se agoto move_timeout_sec."
                goal_handle.abort()
            else:
                result.success = False
                result.status = "ARM_FAULT"
                result.message = "Fallo del brazo durante el movimiento."
                goal_handle.abort()

            return result

        finally:
            self._release_busy()

    def _publish_move_feedback(self, goal_handle, state):
        fb = MoveArm.Feedback()
        fb.state = state
        fb.current_joint_angles = self._read_angles() or []
        goal_handle.publish_feedback(fb)

    # =====================================================================
    # Accion PickPlace
    # =====================================================================

    def _execute_pick_place(self, goal_handle):
        req = goal_handle.request
        result = PickPlace.Result()

        if not self._acquire_busy():
            result.success = False
            result.status = "ARM_FAULT"
            result.message = "El brazo ya esta ejecutando otro objetivo."
            goal_handle.abort()
            return result

        try:
            operation = str(req.operation).strip().lower()
            if operation not in ("pick", "place"):
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = "operation debe ser 'pick' o 'place'."
                goal_handle.abort()
                return result

            use_pose = bool(req.target_pose_name)
            use_coords = len(req.target_coords) == 6
            if use_pose == use_coords:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "Rellena exactamente uno: target_pose_name o "
                    "target_coords(6)."
                )
                goal_handle.abort()
                return result

            if use_pose and req.target_pose_name not in self.poses:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    f"Pose desconocida: '{req.target_pose_name}'."
                )
                goal_handle.abort()
                return result

            if self.mc is None:
                result.success = False
                result.status = "ARM_FAULT"
                result.message = "Brazo no conectado."
                goal_handle.abort()
                return result

            speed = self._resolve_speed(req.speed_percent)
            gripper_speed = (
                req.gripper_speed_percent
                if req.gripper_speed_percent and req.gripper_speed_percent > 0.0
                else self.default_gripper_speed
            )

            open_value = (
                req.gripper_open_value
                if req.gripper_open_value > 0
                else self.gripper_open_value
            )
            closed_value = (
                req.gripper_closed_value
                if req.gripper_closed_value > 0
                else self.gripper_closed_value
            )

            approach_height = float(req.approach_height)

            if use_coords:
                outcome = self._pick_place_coords(
                    goal_handle,
                    operation,
                    list(req.target_coords),
                    approach_height,
                    speed,
                    gripper_speed,
                    open_value,
                    closed_value,
                    result,
                )
            else:
                outcome = self._pick_place_pose(
                    goal_handle,
                    operation,
                    self.poses[req.target_pose_name],
                    approach_height,
                    speed,
                    gripper_speed,
                    open_value,
                    closed_value,
                    result,
                )

            if outcome != "ok":
                return result  # status/message ya rellenados

            # Retirada opcional.
            if req.retreat_pose_name:
                if req.retreat_pose_name in self.poses:
                    self._feedback_pp(goal_handle, "RETREAT")
                    self._move_angles(
                        goal_handle,
                        self.poses[req.retreat_pose_name],
                        speed,
                    )
                else:
                    self.get_logger().warn(
                        f"retreat_pose_name '{req.retreat_pose_name}' "
                        "no existe; se omite."
                    )

            result.success = True
            result.status = "OK"
            result.message = f"{operation} completado."
            goal_handle.succeed()
            return result

        finally:
            self._release_busy()

    def _feedback_pp(self, goal_handle, state):
        fb = PickPlace.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    def _handle_outcome(self, outcome, goal_handle, result, phase):
        """Traduce el resultado de una fase a status. Devuelve True si
        se puede continuar.
        """
        if outcome == "ok":
            return True
        if outcome == "canceled":
            result.success = False
            result.status = "CANCELED"
            result.message = f"Cancelado en {phase}."
            goal_handle.canceled()
        elif outcome == "timeout":
            result.success = False
            result.status = "TIMEOUT"
            result.message = f"Timeout en {phase}."
            goal_handle.abort()
        else:
            result.success = False
            result.status = "ARM_FAULT"
            result.message = f"Fallo del brazo en {phase}."
            goal_handle.abort()
        return False

    def _pick_place_coords(
        self,
        goal_handle,
        operation,
        target,
        approach_height,
        speed,
        gripper_speed,
        open_value,
        closed_value,
        result,
    ):
        approach = list(target)
        approach[2] += approach_height

        if operation == "pick":
            if self._set_gripper(open_value, gripper_speed) == "fault":
                return self._fault(goal_handle, result, "abrir gripper")

        self._feedback_pp(goal_handle, "APPROACH")
        outcome = self._move_coords(goal_handle, approach, speed, 0)
        if not self._handle_outcome(outcome, goal_handle, result, "APPROACH"):
            return outcome

        self._feedback_pp(goal_handle, "DESCEND")
        outcome = self._move_coords(goal_handle, target, speed, 1)
        if not self._handle_outcome(outcome, goal_handle, result, "DESCEND"):
            return outcome

        self._feedback_pp(goal_handle, "GRIP")
        grip_value = closed_value if operation == "pick" else open_value
        if self._set_gripper(grip_value, gripper_speed) == "fault":
            return self._fault(goal_handle, result, "gripper")

        if operation == "pick" and self.verify_grasp:
            if not self._grasp_ok(closed_value):
                result.success = False
                result.status = "GRASP_FAILED"
                result.message = (
                    "El gripper cerro por completo: no se sujeto la pieza."
                )
                goal_handle.abort()
                return "grasp_failed"

        self._feedback_pp(goal_handle, "LIFT")
        outcome = self._move_coords(goal_handle, approach, speed, 1)
        if not self._handle_outcome(outcome, goal_handle, result, "LIFT"):
            return outcome

        return "ok"

    def _pick_place_pose(
        self,
        goal_handle,
        operation,
        pose_angles,
        approach_height,
        speed,
        gripper_speed,
        open_value,
        closed_value,
        result,
    ):
        if operation == "pick":
            if self._set_gripper(open_value, gripper_speed) == "fault":
                return self._fault(goal_handle, result, "abrir gripper")

        self._feedback_pp(goal_handle, "DESCEND")
        outcome = self._move_angles(goal_handle, pose_angles, speed)
        if not self._handle_outcome(outcome, goal_handle, result, "DESCEND"):
            return outcome

        self._feedback_pp(goal_handle, "GRIP")
        grip_value = closed_value if operation == "pick" else open_value
        if self._set_gripper(grip_value, gripper_speed) == "fault":
            return self._fault(goal_handle, result, "gripper")

        if operation == "pick" and self.verify_grasp:
            if not self._grasp_ok(closed_value):
                result.success = False
                result.status = "GRASP_FAILED"
                result.message = (
                    "El gripper cerro por completo: no se sujeto la pieza."
                )
                goal_handle.abort()
                return "grasp_failed"

        self._feedback_pp(goal_handle, "LIFT")
        coords = self._read_coords()
        if coords is None:
            return self._fault(goal_handle, result, "LIFT (get_coords)")
        coords[2] += approach_height
        outcome = self._move_coords(goal_handle, coords, speed, 1)
        if not self._handle_outcome(outcome, goal_handle, result, "LIFT"):
            return outcome

        return "ok"

    def _grasp_ok(self, closed_value):
        try:
            value = self._arm("get_gripper_value")
        except RuntimeError:
            return True  # sin lectura fiable, no penalizamos
        if not isinstance(value, (int, float)) or value < 0:
            return True
        return value > closed_value + 3

    def _fault(self, goal_handle, result, phase):
        result.success = False
        result.status = "ARM_FAULT"
        result.message = f"Fallo del brazo en {phase}."
        goal_handle.abort()
        return "fault"

    # =====================================================================
    # joint_states
    # =====================================================================

    def _publish_joint_states(self):
        if self.joint_state_pub is None or self.mc is None:
            return
        angles = self._read_angles(retries=1, delay=0.0)
        if angles is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [math.radians(a) for a in angles[: len(self.joint_names)]]
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MechArmDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
