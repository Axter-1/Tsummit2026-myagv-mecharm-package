#!/usr/bin/env python3

import math
import os
import time

import yaml

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from home_service_interfaces.action import (
    ArucoApproach,
    MoveArm,
    PickPlace,
)


class MissionManager(Node):

    def __init__(self):

        super().__init__('home_service_mission_manager')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'mission_file',
            ''
        )

        mission_file = self.get_parameter(
            'mission_file'
        ).value

        if not mission_file:
            raise RuntimeError(
                'Parameter "mission_file" is empty.'
            )

        mission_file = os.path.expanduser(
            mission_file
        )

        if not os.path.isfile(mission_file):
            raise RuntimeError(
                f'Mission file does not exist: '
                f'{mission_file}'
            )

        # ---------------------------------------------------------
        # Action clients
        # ---------------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.aruco_client = ActionClient(
            self,
            ArucoApproach,
            '/aruco_lidar_approach'
        )

        self.move_arm_client = ActionClient(
            self,
            MoveArm,
            '/mecharm/move_arm'
        )

        self.pick_place_client = ActionClient(
            self,
            PickPlace,
            '/mecharm/pick_place'
        )

        # Camino CALIBRADO: object_grasp_server. Misma accion PickPlace,
        # otro servidor. La diferencia no es el mensaje, es lo que hay
        # detras: /mecharm/pick_place mueve el brazo a poses ensenadas
        # por nombre, mientras que /grasp_object resuelve la pieza
        # contra grasp_calibrations.yaml a la altura de plataforma
        # configurada, hace la aproximacion ArUco y verifica el agarre.
        # Los pasos 'grasp' de la mision van por aqui.
        self.grasp_client = ActionClient(
            self,
            PickPlace,
            '/grasp_object'
        )

        # Used only to avoid printing the same ArUco state at 20 Hz.
        self.last_aruco_state = None

        # ---------------------------------------------------------
        # Load YAML
        # ---------------------------------------------------------

        with open(
            mission_file,
            'r',
            encoding='utf-8'
        ) as file:

            data = yaml.safe_load(file)

        if data is None:
            raise RuntimeError(
                'Mission YAML is empty.'
            )

        self.mission = data.get(
            'mission',
            {}
        )

        if not self.mission:
            raise RuntimeError(
                'Mission YAML does not contain a "mission" section.'
            )

        # ---------------------------------------------------------
        # Mission configuration
        # ---------------------------------------------------------

        self.mission_name = (
            self.mission.get(
                'name',
                'unnamed_mission'
            )
        )

        self.steps = self.mission.get(
            'steps',
            []
        )

        self.stop_on_failure = bool(
            self.mission.get(
                'stop_on_failure',
                True
            )
        )

        self.repeat_count = int(
            self.mission.get(
                'repeat',
                1
            )
        )

        if not self.steps:
            raise RuntimeError(
                'Mission contains no steps.'
            )

        # Tipos de paso presentes en la mision. Solo se espera por los
        # servidores de accion que la mision realmente usa.
        self.step_types = {
            str(step.get('type', '')).strip().lower()
            for step in self.steps
        }

        # ---------------------------------------------------------
        # Posiciones guardadas junto al mapa
        #
        # Un paso 'navigate' puede pedir 'pose: start' en vez de x/y/yaw.
        # Las coordenadas salen de <mapa>.poses.yaml, que es el fichero
        # que escribe scripts/save_pose.py durante el mapeo.
        #
        # Se resuelve AQUI, al cargar la mision, y no en cada paso: si
        # falta una posicion es mejor enterarse antes de mover el robot
        # que a mitad de rutina con una pieza en la pinza.
        # ---------------------------------------------------------
        self.poses_file = self._resolve_poses_file(mission_file)
        self.poses = self._load_poses(self.poses_file)

        missing = sorted({
            str(step['pose'])
            for step in self.steps
            if str(step.get('type', '')).strip().lower() == 'navigate'
            and 'pose' in step
            and str(step['pose']) not in self.poses
        })

        if missing:
            raise RuntimeError(
                'Faltan posiciones en '
                f'{self.poses_file or "<ningun fichero de poses>"}: '
                f'{", ".join(missing)}. '
                'Guardalas durante el mapeo con: '
                './scripts/tsummit.sh save-pose <nombre>'
            )

        # ---------------------------------------------------------
        # Mission information
        # ---------------------------------------------------------

        self.get_logger().info(
            f'Mission loaded: '
            f'{self.mission_name}'
        )

        self.get_logger().info(
            f'Number of steps: '
            f'{len(self.steps)}'
        )

        if self.repeat_count == 0:

            self.get_logger().info(
                'Mission repetitions: INFINITE'
            )

        else:

            self.get_logger().info(
                f'Mission repetitions: '
                f'{self.repeat_count}'
            )

        self.get_logger().info(
            f'Stop on failure: '
            f'{self.stop_on_failure}'
        )

    # =============================================================
    # Posiciones guardadas junto al mapa
    # =============================================================

    def _resolve_poses_file(self, mission_file):
        """Ruta del <mapa>.poses.yaml que corresponde a esta mision.

        Tres formas, de mas explicita a mas comoda:

          poses_file: ruta                 la ruta tal cual
          map: <nombre>                    <maps_dir>/<nombre>.poses.yaml
          (nada)                           el mapa mas reciente de maps/

        La ultima es la que se usa a diario -- acabas de mapear y de
        guardar las poses -- pero se registra en el log cual se eligio:
        cargar en silencio las posiciones de otra pista manda al robot
        a coordenadas que no existen.
        """
        explicit = str(self.mission.get('poses_file', '')).strip()

        maps_dir = str(
            self.mission.get('maps_dir', '')
        ).strip() or self._default_maps_dir(mission_file)

        if explicit:
            return os.path.expanduser(explicit)

        map_name = str(self.mission.get('map', '')).strip()

        if map_name:
            return os.path.join(maps_dir, f'{map_name}.poses.yaml')

        if not os.path.isdir(maps_dir):
            return ''

        candidates = [
            f for f in os.listdir(maps_dir)
            if f.endswith('.poses.yaml')
        ]

        if not candidates:
            return ''

        candidates.sort(
            key=lambda f: os.path.getmtime(os.path.join(maps_dir, f)),
            reverse=True,
        )

        return os.path.join(maps_dir, candidates[0])

    @staticmethod
    def _default_maps_dir(mission_file):
        """`maps/` del workspace. /workspace dentro del contenedor."""
        if os.path.isdir('/workspace/maps'):
            return '/workspace/maps'

        # .../src/home_service_mission/config/x.yaml -> .../maps
        root = os.path.abspath(mission_file)

        for _ in range(5):
            root = os.path.dirname(root)
            candidate = os.path.join(root, 'maps')
            if os.path.isdir(candidate):
                return candidate

        return os.path.join(os.getcwd(), 'maps')

    def _load_poses(self, path):
        if not path:
            self.get_logger().warn(
                'Ninguna mision ha declarado mapa y no hay ningun '
                '<mapa>.poses.yaml: los pasos "navigate" tendran que '
                'traer x/y/yaw_deg a mano.'
            )
            return {}

        if not os.path.isfile(path):
            self.get_logger().warn(
                f'No existe {path}. Guarda las posiciones durante el '
                'mapeo con: ./scripts/tsummit.sh save-pose <nombre>'
            )
            return {}

        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}

        poses = data.get('poses', {}) or {}

        self.get_logger().info(
            f'Posiciones cargadas de {path} '
            f'(mapa "{data.get("map", "?")}"): '
            f'{", ".join(sorted(poses)) or "ninguna"}'
        )

        return poses

    def resolve_pose(self, step):
        """(x, y, yaw_rad, origen) de un paso de navegacion.

        Un paso con 'pose' manda sobre x/y/yaw_deg sueltos, y se admite
        un desplazamiento relativo opcional (dx, dy, dyaw_deg) para
        colocar variantes de una posicion guardada sin volver a
        conducir el robot hasta alli.
        """
        if 'pose' in step:

            name = str(step['pose'])
            saved = self.poses[name]

            x = float(saved['x']) + float(step.get('dx', 0.0))
            y = float(saved['y']) + float(step.get('dy', 0.0))
            yaw = math.radians(
                float(saved['yaw_deg']) + float(step.get('dyaw_deg', 0.0))
            )

            return x, y, yaw, f'pose guardada "{name}"'

        x = float(step['x'])
        y = float(step['y'])

        if 'yaw_deg' in step:
            yaw = math.radians(float(step['yaw_deg']))
        else:
            yaw = float(step.get('yaw', 0.0))

        return x, y, yaw, 'coordenadas del YAML'

    # =============================================================
    # Wait for servers
    # =============================================================

    def _wait_for_one_server(self, client, label):

        self.get_logger().info(
            f'Waiting for {label} action server...'
        )

        while rclpy.ok():

            if client.wait_for_server(timeout_sec=2.0):
                break

            self.get_logger().warn(
                f'Still waiting for {label}...'
            )

        self.get_logger().info(
            f'{label} action server available.'
        )

    def wait_for_action_servers(self):

        if 'navigate' in self.step_types:
            self._wait_for_one_server(
                self.nav_client,
                '/navigate_to_pose'
            )

        if 'aruco' in self.step_types:
            self._wait_for_one_server(
                self.aruco_client,
                '/aruco_lidar_approach'
            )

        if 'arm_pose' in self.step_types:
            self._wait_for_one_server(
                self.move_arm_client,
                '/mecharm/move_arm'
            )

        if self.step_types & {'pick', 'place'}:
            self._wait_for_one_server(
                self.pick_place_client,
                '/mecharm/pick_place'
            )

        if 'grasp' in self.step_types:
            self._wait_for_one_server(
                self.grasp_client,
                '/grasp_object'
            )

    # =============================================================
    # Navigation
    # =============================================================

    def execute_navigation(
        self,
        step
    ):

        name = step.get(
            'name',
            'navigate'
        )

        x, y, yaw, origen = self.resolve_pose(step)

        frame_id = step.get(
            'frame_id',
            'map'
        )

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = (
            frame_id
        )

        goal.pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0

        goal.pose.pose.orientation.z = (
            math.sin(
                yaw / 2.0
            )
        )

        goal.pose.pose.orientation.w = (
            math.cos(
                yaw / 2.0
            )
        )

        self.get_logger().info(
            '--------------------------------'
        )

        self.get_logger().info(
            f'NAVIGATION: {name}'
        )

        self.get_logger().info(
            f'Goal: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'yaw={math.degrees(yaw):.1f} deg '
            f'({origen})'
        )

        send_future = (
            self.nav_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = (
            send_future.result()
        )

        if goal_handle is None:

            self.get_logger().error(
                'Nav2 did not return '
                'a goal handle.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                'Nav2 rejected the goal.'
            )

            return False

        self.get_logger().info(
            'Nav2 goal accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result_response = (
            result_future.result()
        )

        if result_response is None:

            self.get_logger().error(
                'Nav2 returned no result.'
            )

            return False

        status = (
            result_response.status
        )

        if (
            status ==
            GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().info(
                f'Navigation completed: '
                f'{name}'
            )

            return True

        self.get_logger().error(
            f'Navigation failed. '
            f'Action status={status}'
        )

        return False

    # =============================================================
    # ArUco feedback
    # =============================================================

    def aruco_feedback_callback(
        self,
        feedback_msg
    ):

        feedback = (
            feedback_msg.feedback
        )

        state = feedback.state

        if (
            state !=
            self.last_aruco_state
        ):

            self.last_aruco_state = (
                state
            )

            self.get_logger().info(
                f'ArUco state → '
                f'{state}'
            )

        # Print useful distance only when available.
        if (
            feedback.distance >= 0.0
            and
            state in (
                'APPROACHING',
                'WAITING_LIDAR'
            )
        ):

            self.get_logger().debug(
                f'ArUco/LiDAR distance: '
                f'{feedback.distance:.3f} m'
            )

    # =============================================================
    # ArUco + LiDAR
    # =============================================================

    def execute_aruco(
        self,
        step
    ):

        name = step.get(
            'name',
            'aruco'
        )

        target_id = int(
            step['id']
        )

        stop_distance = float(
            step.get(
                'stop_distance',
                0.12
            )
        )

        timeout_sec = float(
            step.get(
                'timeout_sec',
                0.0
            )
        )

        goal = ArucoApproach.Goal()

        goal.target_id = target_id
        goal.stop_distance = stop_distance
        goal.timeout_sec = timeout_sec

        self.last_aruco_state = None

        self.get_logger().info(
            '--------------------------------'
        )

        self.get_logger().info(
            f'ARUCO: {name}'
        )

        self.get_logger().info(
            f'Target ID={target_id}, '
            f'stop_distance='
            f'{stop_distance:.3f} m'
        )

        send_future = (
            self.aruco_client.send_goal_async(
                goal,
                feedback_callback=(
                    self.aruco_feedback_callback
                )
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = (
            send_future.result()
        )

        if goal_handle is None:

            self.get_logger().error(
                'ArUco server returned '
                'no goal handle.'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                'ArUco goal rejected.'
            )

            return False

        self.get_logger().info(
            'ArUco goal accepted.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result_response = (
            result_future.result()
        )

        if result_response is None:

            self.get_logger().error(
                'ArUco server returned '
                'no result.'
            )

            return False

        result = (
            result_response.result
        )

        action_status = (
            result_response.status
        )

        if (
            action_status ==
            GoalStatus.STATUS_SUCCEEDED
            and
            result.success
        ):

            self.get_logger().info(
                f'ArUco reached: '
                f'ID={target_id}'
            )

            self.get_logger().info(
                f'Final distance: '
                f'{result.final_distance:.3f} m'
            )

            return True

        self.get_logger().error(
            f'ArUco approach failed: '
            f'status={result.status}, '
            f'message="{result.message}"'
        )

        return False

    # =============================================================
    # Brazo: helper generico de envio de goal
    # =============================================================

    def _send_arm_goal(
        self,
        client,
        goal,
        label
    ):
        """Envia un goal de accion y espera el resultado.

        Devuelve (ok: bool, result) donde result puede ser None.
        """

        send_future = client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                f'{label}: sin goal handle.'
            )
            return False, None

        if not goal_handle.accepted:
            self.get_logger().error(
                f'{label}: goal rechazado.'
            )
            return False, None

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        response = result_future.result()

        if response is None:
            self.get_logger().error(
                f'{label}: sin resultado.'
            )
            return False, None

        ok = (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and getattr(response.result, 'success', False)
        )

        return ok, response.result

    # =============================================================
    # Paso: arm_pose
    # =============================================================

    def execute_arm_pose(
        self,
        step
    ):

        name = step.get('name', 'arm_pose')

        goal = MoveArm.Goal()
        goal.pose_name = str(step.get('pose', ''))
        goal.joint_angles = [
            float(v) for v in step.get('joint_angles', [])
        ]
        goal.coords = [
            float(v) for v in step.get('coords', [])
        ]
        goal.move_mode = int(step.get('move_mode', 0))
        goal.speed_percent = float(step.get('speed_percent', 0.0))

        self.get_logger().info(
            '--------------------------------'
        )
        self.get_logger().info(
            f'ARM POSE: {name} '
            f'(pose="{goal.pose_name}")'
        )

        ok, result = self._send_arm_goal(
            self.move_arm_client,
            goal,
            f'arm_pose:{name}'
        )

        if ok:
            self.get_logger().info(
                f'Arm pose completada: {name}'
            )
            return True

        if result is not None:
            self.get_logger().error(
                f'Arm pose fallo: '
                f'status={result.status}, '
                f'message="{result.message}"'
            )

        return False

    # =============================================================
    # Paso: pick / place
    # =============================================================

    def execute_pick_place(
        self,
        step,
        operation
    ):

        name = step.get('name', operation)

        goal = PickPlace.Goal()
        goal.operation = operation
        goal.target_pose_name = str(step.get('target_pose', ''))
        goal.target_coords = [
            float(v) for v in step.get('target_coords', [])
        ]
        goal.approach_height = float(
            step.get('approach_height', 60.0)
        )
        goal.gripper_open_value = int(
            step.get('gripper_open_value', 0)
        )
        goal.gripper_closed_value = int(
            step.get('gripper_closed_value', 0)
        )
        goal.speed_percent = float(
            step.get('speed_percent', 0.0)
        )
        goal.gripper_speed_percent = float(
            step.get('gripper_speed_percent', 0.0)
        )
        goal.retreat_pose_name = str(
            step.get('retreat_pose', '')
        )

        self.get_logger().info(
            '--------------------------------'
        )
        self.get_logger().info(
            f'{operation.upper()}: {name}'
        )

        ok, result = self._send_arm_goal(
            self.pick_place_client,
            goal,
            f'{operation}:{name}'
        )

        if ok:
            self.get_logger().info(
                f'{operation} completado: {name}'
            )
            return True

        if result is not None:
            self.get_logger().error(
                f'{operation} fallo: '
                f'status={result.status}, '
                f'message="{result.message}"'
            )

        return False

    # =============================================================
    # Grasp calibrado (/grasp_object)
    # =============================================================

    def execute_grasp(self, step):
        """Toma o deja una pieza por el camino CALIBRADO.

        Diferencias con los pasos 'pick'/'place', que van a
        /mecharm/pick_place:

          - La pieza se nombra por su clave del catalogo ('rueda',
            'poste', ...) o 'auto' para que la identifique por ArUco.
            No hace falta una pose ensenada por nombre.
          - Las coordenadas del agarre salen de grasp_calibrations.yaml
            para la ALTURA DE PLATAFORMA con la que se arranco el
            servidor, no de poses.yaml.
          - El servidor hace la aproximacion ArUco y verifica el agarre;
            si se cerro al aire, lo dice en vez de seguir.

        Un 'place' aqui EXIGE calibracion propia de place para esa pieza
        y esa altura: el servidor no deduce el soltado del agarre. Si
        falta, aborta con INVALID_GOAL y esta rutina se para.
        """
        name = step.get('name', 'grasp')

        operation = str(
            step.get('action', step.get('operation', 'pick'))
        ).strip().lower()

        if operation not in ('pick', 'place'):
            self.get_logger().error(
                f'grasp "{name}": action debe ser pick o place, '
                f'no "{operation}".'
            )
            return False

        goal = PickPlace.Goal()
        goal.operation = operation

        # object_grasp_server usa target_pose_name como CLAVE DE PIEZA
        # del catalogo, no como pose del brazo. 'auto' = identificarla
        # por el ArUco que vea.
        goal.target_pose_name = str(
            step.get('object', step.get('piece', 'auto'))
        ).strip().lower()

        goal.approach_height = float(
            step.get('approach_height', 0.0)
        )
        goal.gripper_open_value = int(
            step.get('gripper_open_value', 0)
        )
        goal.gripper_closed_value = int(
            step.get('gripper_closed_value', 0)
        )
        goal.speed_percent = float(
            step.get('speed_percent', 0.0)
        )
        goal.gripper_speed_percent = float(
            step.get('gripper_speed_percent', 0.0)
        )
        goal.gripper_torque = int(
            step.get('gripper_torque', 0)
        )
        goal.gripper_force_control = bool(
            step.get('gripper_force_control', False)
        )
        goal.gripper_protect_current = int(
            step.get('gripper_protect_current', 0)
        )
        goal.retreat_pose_name = str(
            step.get('retreat_pose', '')
        )
        goal.initial_pose_name = str(
            step.get('initial_pose', '')
        )

        self.get_logger().info(
            '--------------------------------'
        )
        self.get_logger().info(
            f'GRASP/{operation.upper()}: {name} '
            f'(pieza "{goal.target_pose_name}")'
        )

        ok, result = self._send_arm_goal(
            self.grasp_client,
            goal,
            f'grasp:{operation}:{name}'
        )

        if ok:
            self.get_logger().info(
                f'grasp {operation} completado: {name}'
            )
            return True

        if result is not None:
            self.get_logger().error(
                f'grasp {operation} fallo: '
                f'status={result.status}, '
                f'message="{result.message}"'
            )

            if result.status == 'INVALID_GOAL' and operation == 'place':
                self.get_logger().error(
                    'Un INVALID_GOAL en place casi siempre es que falta '
                    'la calibracion de soltado. Calibrala con: '
                    './scripts/tsummit.sh calibrate-grasp '
                    f'{goal.target_pose_name} <altura_mm> --action place'
                )

        return False

    # =============================================================
    # Execute generic step
    # =============================================================

    def execute_step(
        self,
        step
    ):

        step_type = (
            str(
                step.get(
                    'type',
                    ''
                )
            )
            .strip()
            .lower()
        )

        if step_type == 'navigate':

            return self.execute_navigation(
                step
            )

        if step_type == 'aruco':

            return self.execute_aruco(
                step
            )

        if step_type == 'arm_pose':

            return self.execute_arm_pose(
                step
            )

        if step_type == 'pick':

            return self.execute_pick_place(
                step,
                'pick'
            )

        if step_type == 'place':

            return self.execute_pick_place(
                step,
                'place'
            )

        if step_type == 'grasp':

            return self.execute_grasp(
                step
            )

        self.get_logger().error(
            f'Unknown mission step type: '
            f'"{step_type}"'
        )

        return False

    # =============================================================
    # Mission
    # =============================================================

    def run_mission(self):

        self.wait_for_action_servers()

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            f'STARTING MISSION: '
            f'{self.mission_name}'
        )

        if self.repeat_count == 0:

            self.get_logger().info(
                'Repetitions: INFINITE'
            )

        else:

            self.get_logger().info(
                f'Repetitions: '
                f'{self.repeat_count}'
            )

        self.get_logger().info(
            '================================'
        )

        repetition = 0

        while rclpy.ok():

            # ---------------------------------------------------------
            # Stop when finite repetition count has been reached.
            #
            # repeat_count == 0 means infinite.
            # ---------------------------------------------------------

            if (
                self.repeat_count > 0
                and
                repetition >= self.repeat_count
            ):
                break

            repetition += 1

            if self.repeat_count == 0:

                loop_text = (
                    f'{repetition}/∞'
                )

            else:

                loop_text = (
                    f'{repetition}/'
                    f'{self.repeat_count}'
                )

            self.get_logger().info(
                '################################'
            )

            self.get_logger().info(
                f'MISSION LOOP '
                f'{loop_text}'
            )

            self.get_logger().info(
                '################################'
            )

            # =========================================================
            # Execute every step in this loop
            # =========================================================

            for index, step in enumerate(
                self.steps,
                start=1
            ):

                if not rclpy.ok():
                    return False

                name = step.get(
                    'name',
                    f'step_{index}'
                )

                retries = int(
                    step.get(
                        'retries',
                        0
                    )
                )

                success = False

                for attempt in range(
                    retries + 1
                ):

                    if not rclpy.ok():
                        return False

                    self.get_logger().info(
                        f'Loop {loop_text} '
                        f'- Step '
                        f'{index}/'
                        f'{len(self.steps)}: '
                        f'{name}'
                    )

                    if attempt > 0:

                        self.get_logger().warn(
                            f'Retry '
                            f'{attempt}/'
                            f'{retries}'
                        )

                    success = (
                        self.execute_step(
                            step
                        )
                    )

                    if success:
                        break

                    # ---------------------------------------------
                    # Small pause before retrying.
                    # ---------------------------------------------

                    if rclpy.ok():
                        time.sleep(1.0)

                # =====================================================
                # Step failed
                # =====================================================

                if not success:

                    self.get_logger().error(
                        f'Step failed: '
                        f'{name}'
                    )

                    # on_failure por paso:
                    #   'abort' | 'skip' | 'continue'
                    # Si no se indica, se usa el comportamiento global
                    # stop_on_failure (abort si True).
                    on_failure = str(
                        step.get('on_failure', '')
                    ).strip().lower()

                    if not on_failure:
                        on_failure = (
                            'abort'
                            if self.stop_on_failure
                            else 'continue'
                        )

                    if on_failure == 'abort':

                        self.get_logger().error(
                            'MISSION ABORTED'
                        )

                        return False

                    if on_failure == 'skip':

                        skip_message = str(
                            step.get(
                                'skip_message',
                                'Paso omitido'
                            )
                        )

                        self.get_logger().warn(skip_message)

                    else:

                        self.get_logger().warn(
                            'Continuing mission '
                            'despite failure.'
                        )

            # =========================================================
            # Whole loop completed
            # =========================================================

            self.get_logger().info(
                '--------------------------------'
            )

            self.get_logger().info(
                f'Mission loop '
                f'{loop_text} completed.'
            )

            self.get_logger().info(
                '--------------------------------'
            )

        # =============================================================
        # Finite mission completed
        # =============================================================

        if self.repeat_count > 0:

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                'ALL MISSION LOOPS COMPLETE'
            )

            self.get_logger().info(
                '================================'
            )

            return True

        return False


def main(args=None):

    rclpy.init(args=args)

    node = MissionManager()

    try:

        success = node.run_mission()

        if not success:
            node.get_logger().error(
                'Mission finished '
                'with failure.'
            )

    except KeyboardInterrupt:

        node.get_logger().warn(
            'Mission interrupted '
            'by user.'
        )

    except Exception as exc:

        node.get_logger().error(
            f'Mission exception: '
            f'{exc}'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
