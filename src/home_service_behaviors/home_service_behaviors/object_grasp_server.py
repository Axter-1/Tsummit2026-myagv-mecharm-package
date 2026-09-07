#!/usr/bin/env python3
"""Modulo de TOMA de pieza para el T-SUMMIT Challenge.

QUE RESUELVE
============
Encadena las tres cosas que hacen falta para recoger una pieza y que
hasta ahora estaban sueltas:

    1. DETECTAR    el ArUco que identifica la pieza  (/aruco/detections)
    2. APROXIMAR   la base movil hasta el marcador   (/aruco_lidar_approach)
    3. TOMAR       la pieza con el brazo             (/mecharm/pick_place)

Lo que aporta respecto a llamar a las tres por separado es el PASO 2.5:
traducir "ArUco 1" a "esto es un engranaje, y un engranaje se coge
pinzando 32.8 mm de alma anular a 7.5 mm de la mesa, abriendo antes a
40 mm". Ese conocimiento vive en config/grasp_catalog.yaml, no aqui.

LOS TRES CASOS
==============
    engranaje  disco macizo Ø152.8 con agujero Ø49.2 -> pinza radial
               sobre el alma anular (32.8 mm de material).
    poste      tubo Ø20 sobre disco Ø110 -> pinza directa sobre el tubo,
               70 mm por encima de la mesa.
    rueda      aro Ø150 de pared 20 mm -> pinza en la cima del aro.

Ver grasp_catalog.yaml para la justificacion geometrica de cada uno.

INTERFAZ
========
Accion ``/grasp_object``, tipo home_service_interfaces/action/PickPlace
(se reutiliza el tipo existente para no anadir interfaces nuevas, que en
la Nano obligan a recompilar rosidl). Los campos se interpretan asi:

    operation         "pick" (unico soportado por ahora)
    target_pose_name  clave del catalogo ("engranaje"|"poste"|"rueda")
                      o "auto" -> se deduce del ArUco que se vea
    target_coords     [x, y, z] mm opcional. Si va vacio, se calcula a
                      partir del catalogo y de la pose de aproximacion.
    approach_height   <= 0 -> el del catalogo
    gripper_*         <= 0 -> los del catalogo
    retreat_pose_name "" -> el carry_pose del catalogo

SEGURIDAD
=========
Este nodo MUEVE LA BASE Y EL BRAZO. No se arranca solo: lo lanza
scripts/tsummit.sh, que exige ALLOW_MOTION=1. Ademas rechaza cualquier
objetivo fuera de max_reach_mm antes de mandar nada al brazo.
"""

import math
import os
import threading

import yaml

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory

from home_service_interfaces.action import ArucoApproach, MoveArm, PickPlace
from home_service_interfaces.msg import ArucoDetectionArray


def clamp(value, low, high):
    return max(low, min(high, value))


class GraspSpec:
    """Como se coge UNA pieza. Se construye desde el catalogo."""

    def __init__(self, key, raw, gripper_cfg, table_z_mm):
        self.key = key
        self.label = str(raw.get("label", key))
        self.approach = str(raw.get("approach", "top"))

        self.span_mm = float(raw["span_mm"])
        self.open_mm = float(raw.get("open_mm", self.span_mm + 8.0))
        self.close_mm = float(raw.get("close_mm", max(1.0, self.span_mm - 5.0)))

        self.grasp_z_mm = float(raw.get("grasp_z_mm", 0.0))
        self.approach_height_mm = float(raw.get("approach_height_mm", 60.0))
        self.lift_mm = float(raw.get("lift_mm", 60.0))
        self.offset_mm = [float(v) for v in raw.get("grasp_offset_mm", [0.0, 0.0])]
        self.wrist_deg = float(raw.get("wrist_deg", 0.0))
        self.carry_pose = str(raw.get("carry_pose", "carry"))
        self.require_calibrated_target = bool(
            raw.get("require_calibrated_target", False)
        )

        self.target_coords = None
        raw_target = raw.get("target_coords")

        self.table_z_mm = float(table_z_mm)

        stroke = float(gripper_cfg.get("stroke_mm", 45.0))
        self.stroke_mm = stroke
        self.min_close_mm = float(gripper_cfg.get("min_close_mm", 2.0))
        self.gripper_speed = float(gripper_cfg.get("speed_percent", 40.0))

        # Comprobaciones que atrapan una edicion mala del YAML antes de
        # que el brazo intente algo imposible.
        problems = []
        if raw_target is not None:
            if not isinstance(raw_target, (list, tuple)) or len(raw_target) != 6:
                problems.append("target_coords debe tener 6 valores [X,Y,Z,RX,RY,RZ]")
            else:
                try:
                    self.target_coords = [float(value) for value in raw_target]
                except (TypeError, ValueError):
                    problems.append("target_coords contiene un valor no numerico")
        if self.require_calibrated_target and self.target_coords is None:
            problems.append("requiere target_coords ensenadas antes de usar el brazo")
        if self.span_mm > stroke:
            problems.append(
                f"span_mm={self.span_mm:.1f} supera el recorrido de la "
                f"pinza ({stroke:.1f} mm): esta pieza NO cabe"
            )
        if self.open_mm > stroke:
            problems.append(
                f"open_mm={self.open_mm:.1f} > stroke {stroke:.1f}"
            )
        if self.close_mm >= self.span_mm:
            problems.append(
                f"close_mm={self.close_mm:.1f} >= span_mm={self.span_mm:.1f}: "
                "la pinza no llegaria a apretar"
            )
        if self.close_mm < self.min_close_mm:
            problems.append(
                f"close_mm={self.close_mm:.1f} < min_close_mm"
            )
        self.problems = problems

    # --- conversion mm <-> 0..100 del driver -------------------------

    def mm_to_value(self, mm):
        return int(round(clamp(100.0 * mm / self.stroke_mm, 0.0, 100.0)))

    @property
    def open_value(self):
        return self.mm_to_value(self.open_mm)

    @property
    def close_value(self):
        return self.mm_to_value(self.close_mm)

    @property
    def squeeze_mm(self):
        return self.span_mm - self.close_mm

    def absolute_grasp_z(self):
        """Z del punto de agarre en el frame del brazo."""
        return self.table_z_mm + self.grasp_z_mm

    def describe(self):
        return (
            f"{self.label}: pinza {self.span_mm:.1f} mm "
            f"(abre {self.open_mm:.0f}->{self.open_value}, "
            f"cierra {self.close_mm:.0f}->{self.close_value}, "
            f"aprieta {self.squeeze_mm:.1f} mm), "
            f"Z={self.absolute_grasp_z():.0f} mm, "
            f"aprox +{self.approach_height_mm:.0f} mm"
        )


class ObjectGraspServer(Node):

    def __init__(self):
        super().__init__("object_grasp_server")

        self.cb = ReentrantCallbackGroup()

        self.declare_parameter("catalog_file", "")
        self.declare_parameter("detections_topic", "/aruco/detections")
        self.declare_parameter("approach_action", "/aruco_lidar_approach")
        self.declare_parameter("pick_place_action", "/mecharm/pick_place")
        self.declare_parameter("move_arm_action", "/mecharm/move_arm")
        # Distancia a la que la aproximacion ArUco deja la base. Es la
        # que hace repetible el agarre: el brazo siempre encuentra la
        # pieza en el mismo sitio.
        # 0.30 y no 0.20: MEDIDO por el operador, con la base a 0.20 m
        # del plano la pinza no llega a rozar la superficie donde se
        # apoya la pieza. A 0.30 si.
        #
        # Ojo, esto destapa que _grasp_coords calcula X = parada * 1000,
        # o sea da por hecho que el origen del frame del brazo coincide
        # con lo que mide la parada del LiDAR. No coincide: si
        # coincidiera, acercarse mas nunca podria empeorar el alcance.
        # Falta medir ese offset y meterlo explicito.
        self.declare_parameter("approach_stop_distance", 0.30)

        # DESFASE ENTRE LO QUE MIDE LA PARADA Y LO QUE ALCANZA EL BRAZO
        #
        # _grasp_coords calculaba X = parada * 1000 a secas, dando por
        # hecho que el origen del frame del brazo coincide con el punto
        # desde el que se mide la parada del LiDAR. No coincide, y se
        # notaba: con parada 0.30 el radio salia 301 mm contra un
        # max_reach_mm de 250, o sea que el agarre se rechazaba justo a
        # la distancia a la que el operador comprobo que SI alcanza.
        #
        # MEDIDO: con la base a 300 mm de parada, la consola del brazo
        # leyo X = 149.8 mm con la pinza sobre la pieza. Diferencia
        # 150.2 mm.
        #
        # No se puede descomponer con una sola lectura -- parte es la
        # distancia de la base del brazo al borde delantero, y parte que
        # la pieza no esta en el mismo plano que el ArUco. Da igual: es
        # una constante de este montaje y basta con restarla.
        #
        # OJO: deja de valer si cambia la geometria pieza/marcador. Si
        # se recoloca el ArUco respecto a la pieza, hay que volver a
        # medir la X en la consola.
        self.declare_parameter("arm_x_offset_mm", 150.2)
        self.declare_parameter("approach_timeout_sec", 45.0)
        # Cuanto se espera a ver un ArUco en modo "auto".
        self.declare_parameter("detect_timeout_sec", 15.0)
        # Si es False el nodo hace todo menos mandar el pick al brazo:
        # sirve para ensayar deteccion + aproximacion sin tocar nada.
        self.declare_parameter("enable_arm", True)
        # Si es False no se mueve la base (la pieza ya esta delante).
        self.declare_parameter("enable_approach", True)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        # Distancia REAL a la que quedo la base tras la ultima
        # aproximacion, medida por LiDAR. None mientras no haya una.
        self.measured_stop_distance = None
        self.arm_x_offset_mm = float(
            self.get_parameter("arm_x_offset_mm").value
        )
        self.stop_distance = float(
            self.get_parameter("approach_stop_distance").value
        )
        self.approach_timeout = float(
            self.get_parameter("approach_timeout_sec").value
        )
        self.detect_timeout = float(
            self.get_parameter("detect_timeout_sec").value
        )
        self.enable_arm = bool(self.get_parameter("enable_arm").value)
        self.enable_approach = bool(self.get_parameter("enable_approach").value)

        self._load_catalog(str(self.get_parameter("catalog_file").value))

        # Ultimas detecciones vistas, para el modo "auto".
        self._lock = threading.Lock()
        self._last_detections = []

        self.create_subscription(
            ArucoDetectionArray,
            self.detections_topic,
            self._on_detections,
            qos_profile_sensor_data,
            callback_group=self.cb,
        )

        self.approach_client = ActionClient(
            self,
            ArucoApproach,
            str(self.get_parameter("approach_action").value),
            callback_group=self.cb,
        )
        self.pick_client = ActionClient(
            self,
            PickPlace,
            str(self.get_parameter("pick_place_action").value),
            callback_group=self.cb,
        )
        self.move_client = ActionClient(
            self,
            MoveArm,
            str(self.get_parameter("move_arm_action").value),
            callback_group=self.cb,
        )

        self.server = ActionServer(
            self,
            PickPlace,
            "/grasp_object",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self.cb,
        )

        self.get_logger().info(
            f"object_grasp_server listo. Accion: /grasp_object. "
            f"Piezas: {sorted(self.specs.keys())}. "
            f"brazo={'ON' if self.enable_arm else 'OFF (ensayo)'}, "
            f"base={'ON' if self.enable_approach else 'OFF'}"
        )
        for spec in self.specs.values():
            self.get_logger().info(f"  {spec.describe()}")

    # =================================================================
    # Catalogo
    # =================================================================

    def _load_catalog(self, path):
        if not path:
            try:
                path = os.path.join(
                    get_package_share_directory("home_service_behaviors"),
                    "config",
                    "grasp_catalog.yaml",
                )
            except Exception:  # noqa: BLE001
                path = ""

        if not path or not os.path.isfile(path):
            raise RuntimeError(
                f"No se encuentra grasp_catalog.yaml (probado: '{path}'). "
                "Sin catalogo no se puede agarrar nada."
            )

        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        cat = data.get("grasp_catalog", {})
        gripper = cat.get("gripper", {})
        self.table_z_mm = float(cat.get("table_z_mm", 0.0))
        self.max_reach_mm = float(cat.get("max_reach_mm", 250.0))
        # Suelo de alcance. El de arriba evita pedirle al brazo mas de lo
        # que da; este evita lo contrario, que con el offset del frame es
        # un riesgo real: la base llega con +-34 mm de dispersion, asi
        # que una parada corta de 0.24 deja X = 240 - 150.2 = 90 mm, o
        # sea el objetivo casi encima de la propia base del brazo.
        self.min_reach_mm = float(cat.get("min_reach_mm", 120.0))
        self.safe_navigation_pose = str(
            cat.get("safe_navigation_pose", "")
        ).strip()

        self.aruco_to_object = {
            int(k): str(v) for k, v in (cat.get("aruco_to_object", {}) or {}).items()
        }

        self.specs = {}
        for key, raw in (cat.get("objects", {}) or {}).items():
            spec = GraspSpec(str(key), raw, gripper, self.table_z_mm)
            for problem in spec.problems:
                self.get_logger().error(f"catalogo[{key}]: {problem}")
            self.specs[str(key)] = spec

        if not self.specs:
            raise RuntimeError("El catalogo no define ninguna pieza.")

        unknown = {
            marker: name
            for marker, name in self.aruco_to_object.items()
            if name not in self.specs
        }
        for marker, name in unknown.items():
            self.get_logger().warn(
                f"aruco_to_object: el ID {marker} apunta a '{name}', "
                "que no esta en 'objects'."
            )

        self.get_logger().info(f"Catalogo cargado de {path}")

    def _move_to_safe_pose(self, goal_handle):
        """Recoge el brazo antes de permitir que se mueva la base."""
        if not self.safe_navigation_pose:
            return False, (
                "safe_navigation_pose no esta configurada; ensena una pose "
                "recogida antes de mover la base."
            )

        self._feedback(goal_handle, "SAFE_POSE")
        goal = MoveArm.Goal()
        goal.pose_name = self.safe_navigation_pose
        ok, msg, _ = self._send_and_wait(
            self.move_client, goal, "mecharm/move_arm pose segura", 90.0
        )
        return ok, msg

    # =================================================================
    # Detecciones
    # =================================================================

    def _on_detections(self, msg):
        with self._lock:
            self._last_detections = list(msg.detections)

    def _visible_known_marker(self):
        """El ArUco visible mas cercano que este en el catalogo."""
        with self._lock:
            detections = list(self._last_detections)

        candidates = [
            d for d in detections
            if int(d.id) in self.aruco_to_object
            and self.aruco_to_object[int(d.id)] in self.specs
        ]
        if not candidates:
            return None
        # El mas cercano: si hay varias piezas en la mesa, la de delante.
        best = min(candidates, key=lambda d: d.distance_z or math.inf)
        return int(best.id)

    def _wait_for_marker(self, goal_handle, timeout_sec):
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout_sec
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return None
            marker = self._visible_known_marker()
            if marker is not None:
                return marker
            now = self.get_clock().now().nanoseconds * 1e-9
            if now > deadline:
                return None
            self._feedback(goal_handle, "APPROACH")
            threading_wait(0.2)
        return None

    # =================================================================
    # Utilidades de accion
    # =================================================================

    def _feedback(self, goal_handle, state):
        fb = PickPlace.Feedback()
        fb.state = state
        goal_handle.publish_feedback(fb)

    @staticmethod
    def _result(success, status, message):
        result = PickPlace.Result()
        result.success = success
        result.status = status
        result.message = message
        return result

    def _send_and_wait(self, client, goal, name, timeout_sec):
        """Manda un goal y bloquea. Devuelve (ok, msg, resultado).

        El resultado se devuelve entero a proposito: la aproximacion
        MIDE donde ha parado de verdad y ese numero no se puede tirar.
        Ver _execute, donde sustituye a la distancia nominal.
        """
        if not client.wait_for_server(timeout_sec=5.0):
            return False, f"{name} no disponible", None

        send_future = client.send_goal_async(goal)
        if not wait_future(self, send_future, timeout_sec):
            return False, f"{name}: timeout aceptando el goal", None

        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, f"{name}: goal rechazado", None

        result_future = handle.get_result_async()
        if not wait_future(self, result_future, timeout_sec):
            handle.cancel_goal_async()
            return False, f"{name}: timeout esperando el resultado", None

        wrapped = result_future.result()
        if wrapped is None:
            return False, f"{name}: sin resultado", None

        res = wrapped.result
        ok = bool(getattr(res, "success", False))
        msg = getattr(res, "message", "") or getattr(res, "status", "")
        return ok, f"{name}: {msg}", res

    # =================================================================
    # Ejecucion
    # =================================================================

    def _execute(self, goal_handle):
        req = goal_handle.request
        operation = (req.operation or "pick").strip().lower()

        if operation != "pick":
            goal_handle.abort()
            return self._result(
                False, "INVALID_GOAL",
                f"operation '{operation}' no soportada; solo 'pick'."
            )

        # --- 1. Que pieza es -----------------------------------------
        requested = (req.target_pose_name or "auto").strip().lower()
        marker_id = None

        if requested in ("", "auto"):
            self._feedback(goal_handle, "APPROACH")
            self.get_logger().info(
                f"Modo auto: buscando un ArUco conocido "
                f"({self.detect_timeout:.0f} s)..."
            )
            marker_id = self._wait_for_marker(goal_handle, self.detect_timeout)
            if marker_id is None:
                goal_handle.abort()
                return self._result(
                    False, "INVALID_GOAL",
                    "No se vio ningun ArUco del catalogo. "
                    f"IDs conocidos: {sorted(self.aruco_to_object)}"
                )
            key = self.aruco_to_object[marker_id]
        else:
            key = requested
            if key not in self.specs:
                goal_handle.abort()
                return self._result(
                    False, "INVALID_GOAL",
                    f"Pieza '{key}' desconocida. "
                    f"Disponibles: {sorted(self.specs)}"
                )
            # ID del marcador asociado a esa pieza (el primero que haya).
            for mid, name in sorted(self.aruco_to_object.items()):
                if name == key:
                    marker_id = mid
                    break

        spec = self.specs[key]
        if spec.problems:
            goal_handle.abort()
            return self._result(
                False, "INVALID_GOAL",
                f"El catalogo de '{key}' es invalido: {spec.problems[0]}"
            )

        self.get_logger().info(
            f"Pieza: {spec.label} (ArUco {marker_id}). {spec.describe()}"
        )

        # --- 2. Recoger el brazo antes de mover la base ---------------
        if self.enable_arm and self.enable_approach:
            ok, msg = self._move_to_safe_pose(goal_handle)
            self.get_logger().info(msg)
            if not ok:
                goal_handle.abort()
                return self._result(False, "GRASP_FAILED", msg)

        # --- 3. Aproximacion de la base ------------------------------
        if self.enable_approach and marker_id is not None:
            self._feedback(goal_handle, "APPROACH")
            approach = ArucoApproach.Goal()
            approach.target_id = int(marker_id)
            approach.stop_distance = self.stop_distance
            approach.timeout_sec = self.approach_timeout
            ok, msg, approach_res = self._send_and_wait(
                self.approach_client, approach,
                "aruco_lidar_approach", self.approach_timeout + 15.0,
            )
            self.get_logger().info(msg)
            if not ok:
                goal_handle.abort()
                return self._result(False, "GRASP_FAILED", msg)

            # DONDE PARO DE VERDAD, no donde se le pidio.
            #
            # La base no aterriza en stop_distance: medido en cuatro
            # corridas pidiendo 0.200, salieron 0.226, 0.197, 0.159 y
            # 0.227. Son 68 mm de dispersion, y la pinza del poste
            # tolera +-6 mm. Calcular el agarre sobre el valor NOMINAL
            # es dar por bueno un dato que sabemos que varia diez veces
            # mas que el margen de la pieza.
            #
            # final_distance lo mide el LiDAR contra el plano, que es
            # la misma regla que decide la llegada.
            medida = float(getattr(approach_res, "final_distance", 0.0))

            if medida > 0.0:
                self.measured_stop_distance = medida
                self.get_logger().info(
                    f"Aproximacion medida: {medida:.3f} m "
                    f"(nominal {self.stop_distance:.3f}, "
                    f"diferencia {(medida - self.stop_distance)*1000:+.0f} mm)"
                )
            else:
                self.measured_stop_distance = None
                self.get_logger().warn(
                    "La aproximacion no reporto distancia; se usara la "
                    "nominal y el agarre puede quedar descolocado."
                )
        else:
            self.get_logger().info("Aproximacion desactivada; se salta.")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._result(False, "CANCELED", "Cancelado por el usuario.")

        # --- 4. Objetivo de agarre -----------------------------------
        coords = self._grasp_coords(spec, req)
        # Alcance ESFERICO, no solo en planta: el MechArm 270 tiene 270 mm
        # de radio util contando la altura. Un agarre alto (la cima del
        # aro de la rueda) se sale por Z aunque en planta parezca corto,
        # y comprobarlo solo en XY dejaria pasar un objetivo imposible.
        reach = math.sqrt(
            coords[0] ** 2 + coords[1] ** 2 + coords[2] ** 2
        )
        if reach > self.max_reach_mm:
            goal_handle.abort()
            return self._result(
                False, "INVALID_GOAL",
                f"Objetivo a {reach:.0f} mm de la base del brazo "
                f"(X={coords[0]:.0f} Y={coords[1]:.0f} Z={coords[2]:.0f}), "
                f"por encima de max_reach_mm={self.max_reach_mm:.0f}. "
                "Baja grasp_z_mm o acerca la base."
            )

        if coords[0] <= 0.0 or reach < self.min_reach_mm:
            goal_handle.abort()
            return self._result(
                False, "INVALID_GOAL",
                f"Objetivo DEMASIADO CERCA: X={coords[0]:.0f} mm, radio "
                f"{reach:.0f}, por debajo de min_reach_mm="
                f"{self.min_reach_mm:.0f}. La base se ha quedado corta: "
                f"paro a {(self.measured_stop_distance or self.stop_distance)*1000:.0f} mm "
                f"y el offset del brazo son {self.arm_x_offset_mm:.0f}. "
                "Aleja la base y repite."
            )

        self.get_logger().info(
            f"Agarre en X={coords[0]:.0f} Y={coords[1]:.0f} "
            f"Z={coords[2]:.0f} mm (radio {reach:.0f} mm)"
        )

        if not self.enable_arm:
            goal_handle.succeed()
            return self._result(
                True, "OK",
                f"ENSAYO (enable_arm=false): {spec.label} identificada y "
                f"aproximada. Agarre calculado en {coords}."
            )

        # --- 5. Toma con el brazo ------------------------------------
        self._feedback(goal_handle, "DESCEND")
        pick = PickPlace.Goal()
        pick.operation = "pick"
        pick.target_coords = coords
        pick.approach_height = (
            req.approach_height if req.approach_height > 0.0
            else spec.approach_height_mm
        )
        pick.gripper_open_value = (
            req.gripper_open_value if req.gripper_open_value > 0
            else spec.open_value
        )
        pick.gripper_closed_value = (
            req.gripper_closed_value if req.gripper_closed_value > 0
            else spec.close_value
        )
        pick.gripper_speed_percent = (
            req.gripper_speed_percent if req.gripper_speed_percent > 0.0
            else spec.gripper_speed
        )
        pick.speed_percent = req.speed_percent
        pick.retreat_pose_name = (
            req.retreat_pose_name or spec.carry_pose
        )

        ok, msg, _ = self._send_and_wait(
            self.pick_client, pick, "mecharm/pick_place", 90.0
        )
        self.get_logger().info(msg)

        if not ok:
            goal_handle.abort()
            return self._result(False, "GRASP_FAILED", msg)

        self._feedback(goal_handle, "LIFT")
        goal_handle.succeed()
        return self._result(
            True, "OK",
            f"{spec.label} tomada (ArUco {marker_id}, "
            f"pinza {spec.close_mm:.0f} mm sobre {spec.span_mm:.1f} mm)."
        )

    def _grasp_coords(self, spec, req):
        """[X, Y, Z, RX, RY, RZ] del punto de agarre, en mm/grados.

        Si el goal trae target_coords se respeta (calibracion manual).
        Si no, se construye desde el catalogo: la aproximacion ArUco ha
        dejado la pieza a una distancia conocida delante del robot, asi
        que X es fijo y solo cambia la altura segun la pieza.
        """
        if len(req.target_coords) >= 3:
            base = list(req.target_coords[:3])
        elif spec.target_coords is not None:
            return list(spec.target_coords)
        else:
            # X delante del brazo a la distancia de parada, Y centrado.
            #
            # La MEDIDA manda sobre la nominal. La base no aterriza
            # donde se le pide: 68 mm de dispersion en cuatro corridas
            # contra los +-6 mm que tolera la pinza del poste. Usar el
            # valor pedido seria fiarse del unico numero que sabemos
            # que no se cumple.
            parada = (
                self.measured_stop_distance
                if self.measured_stop_distance is not None
                else self.stop_distance
            )
            base = [
                parada * 1000.0 - self.arm_x_offset_mm + spec.offset_mm[0],
                spec.offset_mm[1],
                spec.absolute_grasp_z(),
            ]

        if len(req.target_coords) >= 6:
            orientation = list(req.target_coords[3:6])
        else:
            # Muñeca apuntando hacia abajo (descenso vertical).
            orientation = [180.0, 0.0, spec.wrist_deg]

        return [float(v) for v in base + orientation]


def threading_wait(seconds):
    import time
    time.sleep(seconds)


def wait_future(node, future, timeout_sec):
    """Espera un future sin volver a hacer spin (el executor ya gira)."""
    import time
    deadline = time.time() + timeout_sec
    while not future.done():
        if time.time() > deadline:
            return False
        time.sleep(0.05)
    return True


def main(args=None):
    rclpy.init(args=args)
    node = ObjectGraspServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
