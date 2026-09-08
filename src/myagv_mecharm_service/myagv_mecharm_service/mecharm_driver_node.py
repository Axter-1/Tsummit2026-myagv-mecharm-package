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

import inspect
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
        # 0 deja el ajuste actual del firmware. 150..980 activa el torque
        # solicitado para la pinza adaptativa.
        self.declare_parameter("gripper_torque", 0)
        self.declare_parameter("gripper_force_control", False)
        self.declare_parameter("gripper_protect_current", 0)
        # pymycobot exige el tipo en algunas versiones aunque la API lo
        # documente como opcional. 1 = pinza adaptativa del MechArm.
        self.declare_parameter("gripper_type", 1)
        self.gripper_torque = int(self.get_parameter("gripper_torque").value)
        self.gripper_force_control = bool(
            self.get_parameter("gripper_force_control").value
        )
        self.gripper_protect_current = int(
            self.get_parameter("gripper_protect_current").value
        )
        self._active_gripper_torque = None
        self._active_gripper_force_control = None
        self._active_gripper_protect_current = None

        self.declare_parameter("move_timeout_sec", 20.0)
        self.declare_parameter("gripper_timeout_sec", 5.0)
        self.declare_parameter("settle_time_sec", 0.4)
        # Tiempo extra tras cerrar/abrir la pinza para que asiente.
        self.declare_parameter("gripper_settle_sec", 0.6)
        self.declare_parameter("gripper_regrip_sec", 1.0)

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
        # Las poses articulares ensenadas pueden asentarse lentamente bajo
        # carga; mantienen el timeout total y solo amplian esta ventana.
        self.declare_parameter("taught_stall_timeout_sec", 12.0)
        # 0 = mandar cada pose ensenada directamente. Valores positivos
        # fragmentan el salto, solo para diagnostico de rutas dificiles.
        self.declare_parameter("taught_max_joint_step_deg", 0.0)

        self.declare_parameter(
            "joint_limits_min",
            [-160.0, -85.0, -180.0, -160.0, -100.0, -180.0],
        )
        self.declare_parameter(
            "joint_limits_max",
            [160.0, 90.0, 45.0, 160.0, 100.0, 180.0],
        )

        # Modo de movimiento del firmware.
        #   fresh_mode 1 = refresh (ejecuta siempre la ultima orden)
        #   fresh_mode 0 = cola/interpolado
        # En refresh mode, send_coords salta de rama de la IK; vision_mode
        # 1 lo limita (doc pymycobot: "limit the posture flipping of
        # send_coords in refresh mode"). -1 = no tocar el ajuste.
        self.declare_parameter("fresh_mode", 1)
        self.declare_parameter("vision_mode", 1)

        # False mantiene las coordenadas como coordenadas hasta el firmware:
        # es la via necesaria para poses ensenadas cuya rama articular
        # equivalente excede un limite. True queda disponible como respaldo
        # diagnostico y resuelve la IK localmente antes de mandar angulos.
        self.declare_parameter("coords_via_ik", False)
        # Compensa un sesgo medido entre send_coords y get_coords. Se suma
        # al comando, pero la llegada se valida contra la pose ensenada.
        self.declare_parameter("coords_command_z_offset_mm", 0.0)
        # Un segundo send_coords corrige la histéresis del firmware sin
        # relajar la tolerancia de llegada del contacto.
        self.declare_parameter("coords_retry_error_mm", 0.0)
        # Margen (mm) con el que la comprobacion FK de la IK da por buena
        # la solucion: angles_to_coords(sol) debe caer a esta distancia
        # del objetivo pedido.
        self.declare_parameter("ik_fk_check_mm", 15.0)

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
        self.gripper_regrip = float(
            self.get_parameter("gripper_regrip_sec").value
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
        self.taught_stall_timeout = float(
            self.get_parameter("taught_stall_timeout_sec").value
        )
        self.taught_max_joint_step = float(
            self.get_parameter("taught_max_joint_step_deg").value
        )

        self.joint_min = [
            float(v) for v in self.get_parameter("joint_limits_min").value
        ]
        self.joint_max = [
            float(v) for v in self.get_parameter("joint_limits_max").value
        ]

        self.fresh_mode = int(self.get_parameter("fresh_mode").value)
        self.vision_mode = int(self.get_parameter("vision_mode").value)
        self.coords_via_ik = bool(
            self.get_parameter("coords_via_ik").value
        )
        self.coords_command_z_offset = float(
            self.get_parameter("coords_command_z_offset_mm").value
        )
        self.coords_retry_error = float(
            self.get_parameter("coords_retry_error_mm").value
        )
        self.ik_fk_check_mm = float(
            self.get_parameter("ik_fk_check_mm").value
        )
        # Limites (min, max) leidos del firmware al conectar. Fuente
        # preferente para validar la IK; None hasta la primera lectura.
        self._firmware_limits = None
        # Motivo del ultimo fallo de IK, para el mensaje de INVALID_GOAL.
        self._last_ik_error = ""
        # Ultima comparacion objetivo/lectura para diagnosticar timeouts.
        self._last_motion_detail = ""

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
                    mc.clear_error_information()
                except Exception:  # noqa: BLE001
                    pass
                if self.fresh_mode >= 0:
                    try:
                        mc.set_fresh_mode(self.fresh_mode)
                    except Exception:  # noqa: BLE001
                        pass
                if self.vision_mode >= 0:
                    try:
                        mc.set_vision_mode(self.vision_mode)
                    except Exception:  # noqa: BLE001
                        pass
                try:
                    mc.init_gripper()
                except Exception:  # noqa: BLE001
                    pass
            self.mc = mc
            self.get_logger().info(
                f"Conectado al MechArm 270 en {self.port} "
                f"(fresh_mode={self.fresh_mode}, "
                f"vision_mode={self.vision_mode})."
            )
            self._log_gripper_status(mc)
            self._log_firmware_limits(mc)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"No se pudo conectar al MechArm ({exc}); "
                "reintentando."
            )
            self.mc = None

    def _log_firmware_limits(self, mc):
        """Vuelca los limites articulares que reporta el firmware.

        Sirve para zanjar la discrepancia entre el URDF (J2 +120) y el
        yaml (J2 +90): el numero del firmware manda.
        """
        try:
            mins, maxs = [], []
            for jid in range(1, 7):
                lo = mc.get_joint_min_angle(jid)
                hi = mc.get_joint_max_angle(jid)
                mins.append(float(lo))
                maxs.append(float(hi))
            if len(mins) == 6 and len(maxs) == 6:
                self._firmware_limits = (mins, maxs)
            self.get_logger().info(
                f"Limites del firmware  min={mins}  max={maxs}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"No se pudieron leer los limites del firmware: {exc}"
            )
        if self.coords_via_ik:
            self._check_ik_convention(mc)

    def _log_gripper_status(self, mc):
        """Lee capacidades del gripper sin moverlo."""
        values = []
        for label, method_name in (
            ("modo_torque", "is_torque_gripper"),
            ("torque_actual", "get_HTS_gripper_torque"),
            ("corriente_proteccion", "get_gripper_protect_current"),
        ):
            try:
                values.append(f"{label}={getattr(mc, method_name)()}")
            except Exception as exc:  # noqa: BLE001
                values.append(f"{label}=no_disponible({exc})")
        self.get_logger().info("Estado gripper: " + ", ".join(values))

    def _check_ik_convention(self, mc):
        """Comprueba SIN MOVER el brazo que solve_inv_kinematics usa el
        mismo convenio de pose que get_coords: angulos -> coords ->
        solve_inv_kinematics(coords, angulos) debe devolver algo parecido
        a los angulos de partida. Si no cierra, el convenio no cuadra.
        """
        try:
            with self._serial_lock:
                a0 = mc.get_angles()
                time.sleep(0.05)
                c0 = mc.get_coords()
            if not (isinstance(a0, (list, tuple)) and len(a0) == 6):
                self.get_logger().warn(
                    "Chequeo IK: get_angles no devolvio 6 valores."
                )
                return
            if not (isinstance(c0, (list, tuple)) and len(c0) == 6):
                self.get_logger().warn(
                    "Chequeo IK: get_coords no devolvio 6 valores."
                )
                return
            with self._serial_lock:
                a1 = mc.solve_inv_kinematics(
                    [float(x) for x in c0], [float(x) for x in a0]
                )
            if not (isinstance(a1, (list, tuple)) and len(a1) == 6):
                self.get_logger().warn(
                    f"Chequeo IK: solve_inv_kinematics devolvio {a1!r}."
                )
                return
            diff = max(abs(float(a1[i]) - float(a0[i])) for i in range(6))
            msg = (
                f"Chequeo IK ida y vuelta: partida={[round(x,1) for x in a0]} "
                f"-> IK={[round(x,1) for x in a1]}  dif_max={diff:.1f} deg"
            )
            if diff <= 5.0:
                self.get_logger().info(msg + "  -> convenio OK")
            else:
                self.get_logger().warn(
                    msg + "  -> NO CIERRA: revisar convenio de pose "
                    "antes de fiarse de la IK"
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Chequeo IK fallo: {exc}")

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
            try:
                return method(*args, **kwargs)
            except Exception as exc:  # noqa: BLE001 - errores de pyserial
                self._drop_connection(str(exc))
                raise RuntimeError(f"fallo serie en {method_name}: {exc}")

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

    def _joint_error(self, index, current, target):
        """Diferencia articular, respetando articulaciones de vuelta completa."""
        low = self.joint_min[index]
        high = self.joint_max[index]
        difference = float(current) - float(target)
        if high - low >= 359.0:
            return abs((difference + 180.0) % 360.0 - 180.0)
        return abs(difference)

    def _wait_for_arrival(
        self, goal_handle, target, kind, timeout, stall_timeout=None
    ):
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
        stall_limit = (
            self.stall_timeout if stall_timeout is None else stall_timeout
        )
        # Margen para que el brazo arranque antes de evaluar nada.
        time.sleep(0.3)

        stable = 0
        read_failures = 0
        last_error = None
        last_progress_time = time.monotonic()
        self._last_motion_detail = ""
        self._last_motion_error = None

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

            if kind == "angles":
                residual = [
                    self._joint_error(i, current[i], target[i])
                    for i in range(components)
                ]
                error = max(
                    residual
                )
            else:
                residual = [
                    abs(current[i] - target[i]) for i in range(components)
                ]
                error = max(
                    residual
                )
            self._last_motion_detail = (
                f"{kind}: objetivo={[round(v, 2) for v in target[:components]]}, "
                f"lectura={[round(v, 2) for v in current[:components]]}, "
                f"residuo={[round(v, 2) for v in residual]}, "
                f"error_max={error:.2f}"
            )
            self._last_motion_error = error

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
            elif time.monotonic() - last_progress_time > stall_limit:
                self.get_logger().warn(
                    f"El brazo dejo de acercarse al objetivo "
                    f"(error {error:.2f}, tolerancia {tolerance:.2f}). "
                    f"Posible obstruccion o pose inalcanzable. "
                    f"{self._last_motion_detail}"
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

    def _log_arm_errors(self, tag):
        """Vuelca el estado de error del firmware. Diagnostico: si el
        brazo ignora send_angles, aqui deberia salir el motivo.
        """
        try:
            with self._serial_lock:
                info = None
                for name in ("get_error_information", "read_next_error"):
                    m = getattr(self.mc, name, None)
                    if m is not None:
                        info = (name, m())
                        break
                servo = None
                sm = getattr(self.mc, "get_servo_status", None)
                if sm is not None:
                    servo = sm()
            self.get_logger().info(
                f"Estado brazo [{tag}]: error={info}  servo_status={servo}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"No se pudo leer el estado del brazo [{tag}]: {exc}"
            )

    def _move_angles(self, goal_handle, angles, speed):
        angles = self._clamp_joints(angles)
        current = self._read_angles(retries=2, delay=0.05)
        if current is not None:
            waypoint = list(angles)
            large_jumps = []
            for index, (actual, target) in enumerate(zip(current, angles)):
                if self._joint_error(index, actual, target) > 180.0:
                    waypoint[index] = (actual + target) / 2.0
                    large_jumps.append(index + 1)
            if large_jumps:
                home_pose = self.poses.get("home")
                safe_pose = self.poses.get("safe_navigation")
                is_home = (
                    home_pose is not None
                    and max(abs(a - b) for a, b in zip(angles, home_pose)) < 0.01
                )
                if is_home and safe_pose is not None:
                    waypoint = list(safe_pose)
                    route_name = "safe_navigation"
                else:
                    route_name = "waypoint intermedio"
                self.get_logger().info(
                    f"Ruta {route_name}: "
                    f"saltos articulares >180 grados en J{large_jumps}."
                )
                try:
                    self._arm("send_angles", waypoint, speed)
                except Exception as exc:  # noqa: BLE001 - valida pymycobot tambien
                    self._drop_connection(str(exc))
                    return "fault"
                outcome = self._wait_for_arrival(
                    goal_handle, waypoint, "angles", self.move_timeout
                )
                if outcome != "ok":
                    self._log_arm_errors("tras waypoint intermedio fallido")
                    return outcome
        return self._move_taught_angles(goal_handle, angles, speed)

    def _within_limits(self, angles):
        # Fuente preferente: lo leido del firmware. Si no, el parametro.
        if self._firmware_limits is not None:
            lo_all, hi_all = self._firmware_limits
        else:
            lo_all, hi_all = self.joint_min, self.joint_max
        for i, a in enumerate(angles):
            lo = lo_all[i] if i < len(lo_all) else -180.0
            hi = hi_all[i] if i < len(hi_all) else 180.0
            if a < lo - 1.0 or a > hi + 1.0:
                return False
        return True

    def _coords_to_angles(self, coords):
        """Resuelve la IK de 'coords' con solve_inv_kinematics sembrando
        con los angulos actuales. Devuelve una lista de 6 angulos valida
        o None si no hay solucion de confianza.
        """
        self._last_ik_error = ""
        seed = self._read_angles(retries=3, delay=0.1)
        if seed is None:
            self._last_ik_error = (
                "no se pudieron leer los angulos actuales para sembrar la IK"
            )
            self.get_logger().warn(f"IK: {self._last_ik_error}.")
            return None
        try:
            sol = self._arm("solve_inv_kinematics", list(coords), seed)
        except RuntimeError as exc:
            self._drop_connection(str(exc))
            return None
        if not isinstance(sol, (list, tuple)) or len(sol) != 6:
            self._last_ik_error = f"respuesta de IK no valida ({sol!r})"
            self.get_logger().warn(f"IK: {self._last_ik_error}.")
            return None
        sol = [float(a) for a in sol]
        if all(abs(a) < 1e-6 for a in sol):
            self._last_ik_error = "la IK no encontro solucion (solucion nula)"
            self.get_logger().warn(f"IK: {self._last_ik_error}.")
            return None
        if not self._within_limits(sol):
            self._last_ik_error = (
                f"la solucion de IK {[round(a, 1) for a in sol]} "
                "queda fuera de los limites del firmware"
            )
            self.get_logger().warn(f"IK: {self._last_ik_error}.")
            return None
        # Comprobacion FK: la solucion debe reproducir el objetivo.
        try:
            fk = self._arm("angles_to_coords", sol)
        except RuntimeError:
            fk = None
        if isinstance(fk, (list, tuple)) and len(fk) == 6:
            err = max(abs(float(fk[i]) - float(coords[i])) for i in range(3))
            if err > self.ik_fk_check_mm:
                self._last_ik_error = (
                    f"la solucion de IK no reproduce el objetivo "
                    f"(FK a {err:.1f} mm, tope {self.ik_fk_check_mm:.1f})"
                )
                self.get_logger().warn(f"IK: {self._last_ik_error}.")
                return None
            self.get_logger().info(
                f"IK: solucion {[round(a, 1) for a in sol]} "
                f"(FK a {err:.1f} mm del objetivo)."
            )
        else:
            self.get_logger().info(
                f"IK: solucion {[round(a, 1) for a in sol]} "
                f"(sin verificacion FK)."
            )
        return sol

    def _move_coords(self, goal_handle, coords, speed, mode):
        coords = [float(c) for c in coords]
        # pymycobot: mode 0 = angular (trayectoria libre),
        #            mode 1 = lineal (linea recta).
        if self.coords_via_ik:
            # send_coords deja la eleccion de rama al firmware y en esta
            # unidad falla. Resolvemos la IK aqui y mandamos angulos. Si
            # no hay solucion de confianza, el objetivo es INALCANZABLE:
            # se dice explicitamente, no se cae a send_coords.
            angles = self._coords_to_angles(coords)
            if angles is None:
                return "unreachable"
            return self._move_angles(goal_handle, angles, speed)
        command_coords = list(coords)
        command_coords[2] += self.coords_command_z_offset
        if self.coords_command_z_offset:
            self.get_logger().info(
                f"send_coords: compensacion Z "
                f"{self.coords_command_z_offset:+.1f} mm "
                f"({coords[2]:.1f} -> {command_coords[2]:.1f})"
            )
        try:
            self._arm("send_coords", command_coords, speed, int(mode))
        except RuntimeError as exc:
            self._drop_connection(str(exc))
            return "fault"
        outcome = self._wait_for_arrival(
            goal_handle, coords, "coords", self.move_timeout
        )
        if (
            outcome == "timeout"
            and self._last_motion_error is not None
            and 0.0 < self._last_motion_error <= self.coords_retry_error
        ):
            self.get_logger().warn(
                f"send_coords quedo a {self._last_motion_error:.2f} mm; "
                "se reintenta una vez sin relajar la tolerancia."
            )
            try:
                self._arm("send_coords", command_coords, speed, int(mode))
            except RuntimeError as exc:
                self._drop_connection(str(exc))
                return "fault"
            outcome = self._wait_for_arrival(
                goal_handle, coords, "coords", self.move_timeout
            )
        return outcome

    def _set_gripper(
        self,
        value,
        speed,
        torque=None,
        force_control=None,
        protect_current=None,
    ):
        """Mueve la pinza a una apertura 0..100 (0 cerrada, 100 abierta).

        Se usa set_gripper_value porque es inequivoco. Si esa via falla
        se recurre a set_gripper_state, cuyo flag es 0 = abrir y
        1 = cerrar (ojo: es al reves de lo que suele suponerse).
        """
        value = int(round(clamp(value, 0.0, 100.0)))
        speed = int(round(clamp(speed, 1.0, 100.0)))
        if torque is None:
            torque = (
                self._active_gripper_torque
                if self._active_gripper_torque is not None
                else self.gripper_torque
            )
        if force_control is None:
            force_control = (
                self._active_gripper_force_control
                if self._active_gripper_force_control is not None
                else self.gripper_force_control
            )
        if protect_current is None:
            protect_current = (
                self._active_gripper_protect_current
                if self._active_gripper_protect_current is not None
                else self.gripper_protect_current
            )
        torque = int(round(float(torque or 0)))
        if torque and not 150 <= torque <= 980:
            self.get_logger().error(
                "gripper_torque debe estar entre 150 y 980"
            )
            return "fault"
        protect_current = int(round(float(protect_current or 0)))
        if protect_current and not 1 <= protect_current <= 500:
            self.get_logger().error(
                "gripper_protect_current debe estar entre 1 y 500"
            )
            return "fault"

        try:
            gripper_value_method = getattr(self.mc, "set_gripper_value")
            supports_force_arg = "is_torque" in inspect.signature(
                gripper_value_method
            ).parameters
        except (AttributeError, TypeError, ValueError):
            supports_force_arg = False

        if (force_control or torque > 0) and not supports_force_arg:
            self.get_logger().error(
                "La version instalada de pymycobot no soporta control de "
                "fuerza en MechArm270 (falta is_torque en set_gripper_value)."
            )
            return "unsupported_force"

        if protect_current:
            try:
                self._arm("set_gripper_protect_current", protect_current)
                applied_current = self._arm("get_gripper_protect_current")
                self.get_logger().info(
                    f"Corriente de proteccion solicitada={protect_current}, "
                    f"lectura={applied_current}"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"No se pudo ajustar la corriente de proteccion a "
                    f"{protect_current}: {exc}"
                )
                return "unsupported_force"

        if torque:
            try:
                torque_result = self._arm("set_HTS_gripper_torque", torque)
                if torque_result == 0:
                    try:
                        current_torque = self._arm("get_HTS_gripper_torque")
                    except Exception:  # noqa: BLE001
                        current_torque = "no_disponible"
                    try:
                        current_protect = self._arm(
                            "get_gripper_protect_current"
                        )
                    except Exception:  # noqa: BLE001
                        current_protect = "no_disponible"
                    self.get_logger().error(
                        f"El firmware rechazo el torque del gripper: {torque}; "
                        f"torque_actual={current_torque}, "
                        f"corriente_proteccion={current_protect}"
                    )
                    return "fault"
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"No se pudo ajustar el torque del gripper a {torque}: {exc}. "
                    "Comprueba que el modelo/API soporte set_HTS_gripper_torque."
                )
                return "unsupported_force"

        # Un torque explicito implica modo de control de fuerza; de otro
        # modo el valor del torque se programa pero no participa en el
        # comando de cierre.
        is_torque = 1 if (force_control or torque > 0) else 0

        value_args = [value, speed, self.gripper_type]
        if supports_force_arg:
            value_args.append(is_torque)

        try:
            self._arm("set_gripper_value", *value_args)
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
                state_args = [flag, speed, self.gripper_type]
                if supports_force_arg:
                    state_args.append(is_torque)
                self._arm("set_gripper_state", *state_args)
            except Exception as exc2:  # noqa: BLE001
                self.get_logger().error(f"set_gripper_state fallo: {exc2}")
                return "fault"

        return self._wait_gripper_idle(self.gripper_timeout)

    def _close_gripper_with_regrip(
        self, goal_handle, result, value, speed, phase="gripper"
    ):
        """Cierra, deja asentarse la pieza y reaplica el cierre."""
        outcome = self._set_gripper(value, speed)
        if outcome != "ok":
            return self._fault(goal_handle, result, phase)

        if self.gripper_regrip > 0.0:
            time.sleep(self.gripper_regrip)
            outcome = self._set_gripper(value, speed)
            if outcome != "ok":
                return self._fault(goal_handle, result, f"{phase} (reapriete)")

        return "ok"

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

            outcome = self._set_gripper(
                value,
                speed,
                request.torque,
                request.force_control,
                request.protect_current,
            )

            try:
                read = self._arm("get_gripper_value")
                if isinstance(read, (int, float)) and read >= 0:
                    response.current_value = int(read)
            except Exception:  # noqa: BLE001
                pass

            if outcome != "ok":
                response.success = False
                response.message = (
                    "Control de fuerza no soportado por pymycobot/MechArm270."
                    if outcome == "unsupported_force"
                    else "Fallo al mover la pinza."
                )
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
                # En pymycobot, sin el 1 se conserva amortiguacion y una
                # articulacion cargada como J3 puede seguir pareciendo fija.
                self._arm("release_all_servos", 1)
                response.success = True
                response.message = (
                    "Servos liberados: mueve el brazo a mano y lee la "
                    "pose en /mecharm/joint_states."
                )
            else:
                self._arm("power_on")
                # Al enseñar una pose fuera de limite el firmware deja el
                # fallo enclavado. Tras devolver el brazo a un rango valido,
                # limpiarlo aqui permite retomar las acciones sin reiniciar.
                self._arm("clear_error_information")
                response.success = True
                response.message = "Servos alimentados y error limpiado."
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
            elif outcome == "unreachable":
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "Esta pose no es alcanzable: "
                    + (self._last_ik_error or "sin solucion de IK")
                )
                goal_handle.abort()
            elif outcome == "canceled":
                result.success = False
                result.status = "CANCELED"
                result.message = "Objetivo cancelado."
                goal_handle.canceled()
            elif outcome == "timeout":
                result.success = False
                result.status = "TIMEOUT"
                result.message = (
                    "Se agoto move_timeout_sec. "
                    + (self._last_motion_detail or "sin telemetria final")
                )
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
            use_coords = len(req.target_coords) in (6, 12)
            use_joints = len(req.target_joint_angles) == 6
            if sum((use_pose, use_coords, use_joints)) != 1:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "Rellena exactamente uno: target_pose_name o "
                    "target_coords(6 o 12) o target_joint_angles(6)."
                )
                goal_handle.abort()
                return result

            if len(req.approach_joint_waypoints) % 6 != 0:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "approach_joint_waypoints debe contener grupos de 6 angulos."
                )
                goal_handle.abort()
                return result

            if len(req.approach_coords_waypoints) % 6 != 0:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "approach_coords_waypoints debe contener grupos de 6 coordenadas."
                )
                goal_handle.abort()
                return result

            if use_coords and req.approach_joint_waypoints:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "Un objetivo cartesiano no puede incluir waypoints articulares."
                )
                goal_handle.abort()
                return result

            if use_joints and req.approach_coords_waypoints:
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    "Un objetivo articular no puede incluir waypoints cartesianos."
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

            if (
                req.initial_pose_name and
                req.initial_pose_name not in self.poses
            ):
                result.success = False
                result.status = "INVALID_GOAL"
                result.message = (
                    f"Pose inicial desconocida: '{req.initial_pose_name}'."
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

            self._active_gripper_torque = int(req.gripper_torque)
            self._active_gripper_force_control = bool(
                req.gripper_force_control
            )
            self._active_gripper_protect_current = int(
                req.gripper_protect_current
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

            if req.initial_pose_name:
                self._feedback_pp(goal_handle, "APPROACH")
                outcome = self._move_angles(
                    goal_handle,
                    self.poses[req.initial_pose_name],
                    speed,
                )
                if not self._handle_outcome(
                    outcome,
                    goal_handle,
                    result,
                    f"INITIAL {req.initial_pose_name}",
                ):
                    return result

            if use_coords:
                taught_pregrasp = (
                    list(req.target_coords[6:])
                    if len(req.target_coords) == 12 else None
                )
                outcome = self._pick_place_coords(
                    goal_handle,
                    operation,
                    list(req.target_coords[:6]),
                    approach_height,
                    speed,
                    gripper_speed,
                    open_value,
                    closed_value,
                    result,
                    taught_pregrasp,
                    [
                        list(req.approach_coords_waypoints[index:index + 6])
                        for index in range(
                            0, len(req.approach_coords_waypoints), 6
                        )
                    ],
                )
            elif use_joints:
                taught_waypoints = [
                    list(req.approach_joint_waypoints[index:index + 6])
                    for index in range(0, len(req.approach_joint_waypoints), 6)
                ]
                outcome = self._pick_place_taught_joints(
                    goal_handle,
                    operation,
                    list(req.target_joint_angles),
                    taught_waypoints,
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
            self._active_gripper_torque = None
            self._active_gripper_force_control = None
            self._active_gripper_protect_current = None
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
        if outcome == "unreachable":
            result.success = False
            result.status = "INVALID_GOAL"
            result.message = (
                f"Pose inalcanzable en {phase}: "
                + (self._last_ik_error or "sin solucion de IK")
            )
            goal_handle.abort()
            return False
        if outcome == "canceled":
            result.success = False
            result.status = "CANCELED"
            result.message = f"Cancelado en {phase}."
            goal_handle.canceled()
        elif outcome == "timeout":
            result.success = False
            result.status = "TIMEOUT"
            result.message = (
                f"Timeout en {phase}. "
                + (self._last_motion_detail or "sin telemetria final")
            )
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
        taught_pregrasp=None,
        taught_waypoints=None,
    ):
        # Un preagarre ensenado conserva el vector seguro real. Solo las
        # piezas sin calibracion explicita usan el respaldo target + Z.
        if taught_waypoints:
            approaches = [list(coords) for coords in taught_waypoints]
        elif taught_pregrasp is None:
            approach = list(target)
            approach[2] += approach_height
            approaches = [approach]
        else:
            approaches = [list(taught_pregrasp)]

        if operation == "pick":
            if self._set_gripper(open_value, gripper_speed) != "ok":
                return self._fault(goal_handle, result, "abrir gripper")

        self._feedback_pp(goal_handle, "APPROACH")
        for index, approach in enumerate(approaches, start=1):
            outcome = self._move_coords(goal_handle, approach, speed, 0)
            if not self._handle_outcome(
                outcome, goal_handle, result, f"APPROACH waypoint {index}"
            ):
                return outcome

        self._feedback_pp(goal_handle, "DESCEND")
        outcome = self._move_coords(goal_handle, target, speed, 1)
        if not self._handle_outcome(outcome, goal_handle, result, "DESCEND"):
            return outcome

        self._feedback_pp(goal_handle, "GRIP")
        grip_value = closed_value if operation == "pick" else open_value
        if operation == "pick":
            outcome = self._close_gripper_with_regrip(
                goal_handle, result, grip_value, gripper_speed
            )
        else:
            outcome = self._set_gripper(grip_value, gripper_speed)
        if outcome != "ok":
            return outcome

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
        for index, approach in enumerate(reversed(approaches), start=1):
            outcome = self._move_coords(goal_handle, approach, speed, 1)
            if not self._handle_outcome(
                outcome, goal_handle, result, f"LIFT waypoint {index}"
            ):
                return outcome

        return "ok"

    def _move_taught_angles(self, goal_handle, angles, speed):
        """Mueve a una pose que el operador ha ensenado y validado.

        No aplica _clamp_joints porque una pose ensenada debe rechazarse,
        no deformarse. Aun asi respeta los limites reales del firmware.
        """
        if len(angles) != 6:
            return "fault"
        if not self._within_limits(angles):
            self._last_ik_error = (
                f"pose articular ensenada fuera de limites: "
                f"{[round(float(a), 2) for a in angles]}"
            )
            self.get_logger().error(f"Ruta ensenada: {self._last_ik_error}.")
            return "unreachable"
        current = self._read_angles(retries=2, delay=0.05)
        steps = 1
        deltas = None
        if current is not None and self.taught_max_joint_step > 0.0:
            deltas = []
            for index, target in enumerate(angles):
                delta = float(target) - current[index]
                if self.joint_max[index] - self.joint_min[index] >= 359.0:
                    delta = (delta + 180.0) % 360.0 - 180.0
                deltas.append(delta)
            steps = max(
                1,
                int(math.ceil(
                    max(abs(delta) for delta in deltas)
                    / self.taught_max_joint_step
                )),
            )

        for step in range(1, steps + 1):
            if step == steps or current is None or deltas is None:
                waypoint = [float(angle) for angle in angles]
            else:
                fraction = step / steps
                waypoint = [
                    current[index] + delta * fraction
                    for index, delta in enumerate(deltas)
                ]
            try:
                self._arm("send_angles", waypoint, speed)
            except Exception as exc:  # noqa: BLE001 - valida pymycobot tambien
                self._drop_connection(str(exc))
                return "fault"
            outcome = self._wait_for_arrival(
                goal_handle,
                waypoint,
                "angles",
                self.move_timeout,
                self.taught_stall_timeout,
            )
            if outcome == "timeout" and steps == 1:
                # Tras un IK 32 el firmware puede quedarse a medio camino
                # aunque la pose final sea valida. Limpiar y reenviar una
                # vez la pose completa es preferible a aceptar un residuo.
                self.get_logger().warn(
                    "Pose ensenada sin llegada; se limpia el error y "
                    "se reintenta una vez el objetivo completo."
                )
                try:
                    self._arm("clear_error_information")
                    self._arm("send_angles", waypoint, speed)
                except Exception as exc:  # noqa: BLE001
                    self._drop_connection(str(exc))
                    return "fault"
                outcome = self._wait_for_arrival(
                    goal_handle,
                    waypoint,
                    "angles",
                    self.move_timeout,
                    self.taught_stall_timeout,
                )
            if outcome != "ok":
                self._log_arm_errors("tras pose articular ensenada fallida")
                return outcome
        return "ok"

    def _pick_place_taught_joints(
        self,
        goal_handle,
        operation,
        target,
        waypoints,
        speed,
        gripper_speed,
        open_value,
        closed_value,
        result,
    ):
        if operation == "pick":
            if self._set_gripper(open_value, gripper_speed) != "ok":
                return self._fault(goal_handle, result, "abrir gripper")

        self._feedback_pp(goal_handle, "APPROACH")
        for index, waypoint in enumerate(waypoints, start=1):
            outcome = self._move_taught_angles(goal_handle, waypoint, speed)
            if not self._handle_outcome(
                outcome, goal_handle, result, f"APPROACH waypoint {index}"
            ):
                return outcome

        self._feedback_pp(goal_handle, "DESCEND")
        outcome = self._move_taught_angles(goal_handle, target, speed)
        if not self._handle_outcome(outcome, goal_handle, result, "DESCEND"):
            return outcome

        self._feedback_pp(goal_handle, "GRIP")
        grip_value = closed_value if operation == "pick" else open_value
        if operation == "pick":
            outcome = self._close_gripper_with_regrip(
                goal_handle, result, grip_value, gripper_speed
            )
        else:
            outcome = self._set_gripper(grip_value, gripper_speed)
        if outcome != "ok":
            return outcome

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
        for index, waypoint in enumerate(reversed(waypoints), start=1):
            outcome = self._move_taught_angles(goal_handle, waypoint, speed)
            if not self._handle_outcome(
                outcome, goal_handle, result, f"LIFT waypoint {index}"
            ):
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
            if self._set_gripper(open_value, gripper_speed) != "ok":
                return self._fault(goal_handle, result, "abrir gripper")

        self._feedback_pp(goal_handle, "DESCEND")
        outcome = self._move_angles(goal_handle, pose_angles, speed)
        if not self._handle_outcome(outcome, goal_handle, result, "DESCEND"):
            return outcome

        self._feedback_pp(goal_handle, "GRIP")
        grip_value = closed_value if operation == "pick" else open_value
        if operation == "pick":
            outcome = self._close_gripper_with_regrip(
                goal_handle, result, grip_value, gripper_speed
            )
        else:
            outcome = self._set_gripper(grip_value, gripper_speed)
        if outcome != "ok":
            return outcome

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
