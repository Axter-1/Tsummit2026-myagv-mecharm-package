#!/usr/bin/env bash
# =====================================================================
#  Retos 1 (Clasificacion) y 2 (Kitting). Lanzador de pruebas.
# ---------------------------------------------------------------------
#  Igual que run_maze.sh: se ejecuta en el sistema NATIVO de la Jetson
#  y habla con el contenedor Docker por 'docker exec'.
#
#  ORDEN DE ARRANQUE
#  =================
#    1)  START_BRINGUP=0 ./docker/run_jetson_robot.sh
#        (solo base + LiDAR; la camara, el brazo y el ArUco los levanta
#         este script para no duplicarlos)
#
#    2)  ./scripts/run_classification.sh run      # reto 1
#        ./scripts/run_classification.sh kitting  # reto 2
#
#  SUBCOMANDOS
#  ===========
#    run       Reto 1 completo (pila + mision)
#    kitting   Reto 2 completo (misma pila, mision con "Pieza Omitida")
#    stack     Solo la pila, SIN mision (para calibrar poses/coordenadas)
#    teach     Consola del brazo para ensenar poses (para el driver ROS)
#    gripper   Prueba rapida de la garra por servicio ROS: gripper <0-100>
#    arm       Mueve el brazo a una pose guardada:  arm <pose>
#    pose      Imprime la pose actual del robot en el mapa
#    aruco     Muestra los ArUco detectados en vivo
#    mission   Relanza solo el mission_manager:  mission [archivo.yaml]
#    stop      Detiene la pila de clasificacion
#    shell     Shell con el entorno ROS cargado
# =====================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${CONTAINER:-myagv-robot}"

MARKER_LENGTH="${MARKER_LENGTH:-0.08}"
CAMERA_SOURCE="${CAMERA_SOURCE:-nvargus}"
ARM_PORT="${ARM_PORT:-/dev/ttyACM0}"
BLIND_SECTORS="${BLIND_SECTORS:-[-50.0, 50.0]}"
MISSION_DELAY="${MISSION_DELAY:-15.0}"

DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
    DOCKER=(sudo docker)
fi

SOURCE_ENV='source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash'

exec_flags() {
    if [ -t 0 ] && [ -t 1 ]; then printf -- '-it'; else printf -- '-i'; fi
}

require_container() {
    if ! "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" \
         2>/dev/null | grep -q true; then
        cat >&2 <<EOF
ERROR: el contenedor '${CONTAINER}' no esta corriendo.

    START_BRINGUP=0 ./docker/run_jetson_robot.sh
EOF
        exit 1
    fi
}

in_container() {
    "${DOCKER[@]}" exec "$(exec_flags)" "${CONTAINER}" bash -lc "$1"
}

launch_stack() {
    local mission_file="$1"
    local run_mission="$2"
    require_container
    echo "[i] mision      : ${mission_file:-(ninguna)}"
    echo "[i] marker      : ${MARKER_LENGTH} m"
    echo "[i] camara      : ${CAMERA_SOURCE}"
    echo "[i] puerto brazo: ${ARM_PORT}"
    echo
    in_container "${SOURCE_ENV}; \
        ros2 launch home_service_bringup classification.launch.py \
            use_sim_time:=false \
            slam:=true \
            run_mission:=${run_mission} \
            mission:=${mission_file:-reto1_clasificacion.yaml} \
            mission_delay_sec:=${MISSION_DELAY} \
            marker_length:=${MARKER_LENGTH} \
            camera_source:=${CAMERA_SOURCE} \
            arm_port:=${ARM_PORT} \
            blind_sectors_deg:='${BLIND_SECTORS}'"
}

case "${1:-run}" in

    run)      launch_stack reto1_clasificacion.yaml true ;;
    kitting)  launch_stack reto2_kitting.yaml       true ;;
    stack)
        echo "[i] Pila SIN mision: usa 'pose', 'aruco', 'arm' y RViz"
        echo "    para calibrar coordenadas y poses."
        launch_stack "" false
        ;;

    teach)
        require_container
        echo "[i] Se detiene mecharm_driver_node para liberar ${ARM_PORT}."
        in_container "pkill -f mecharm_driver_node || true; sleep 1; \
            python3 /workspace/scripts/mecharm_console.py \
                --port ${ARM_PORT} \
                --poses-file /workspace/src/myagv_mecharm_service/config/poses.yaml"
        echo "[i] Recuerda recompilar para instalar las poses nuevas:"
        echo "    colcon build --packages-select myagv_mecharm_service"
        ;;

    gripper)
        require_container
        value="${2:-}"
        if [ -z "${value}" ]; then
            echo "uso: $0 gripper <0-100>   (0 = cerrada, 100 = abierta)" >&2
            exit 2
        fi
        in_container "${SOURCE_ENV}; \
            ros2 service call /mecharm/set_gripper \
                home_service_interfaces/srv/SetGripper \
                '{value: ${value}, speed_percent: 40.0}'"
        ;;

    arm)
        require_container
        pose="${2:-home}"
        in_container "${SOURCE_ENV}; \
            ros2 action send_goal /mecharm/move_arm \
                home_service_interfaces/action/MoveArm \
                '{pose_name: \"${pose}\", speed_percent: 25.0}' --feedback"
        ;;

    pose)
        require_container
        in_container "${SOURCE_ENV}; \
            ros2 run tf2_ros tf2_echo map base_footprint" || true
        ;;

    aruco)
        require_container
        in_container "${SOURCE_ENV}; \
            ros2 topic echo /aruco/detections --field detections"
        ;;

    mission)
        require_container
        file="${2:-reto1_clasificacion.yaml}"
        in_container "${SOURCE_ENV}; \
            ros2 launch home_service_mission mission.launch.py \
                use_sim_time:=false \
                mission_file:=/workspace/install/home_service_mission/share/home_service_mission/config/${file}"
        ;;

    stop)
        require_container
        in_container "pkill -f 'classification.launch.py' || true; \
                      pkill -f 'async_slam_toolbox_node' || true; \
                      pkill -f 'mission_manager' || true; \
                      pkill -f 'controller_server' || true; \
                      pkill -f 'planner_server' || true; \
                      pkill -f 'bt_navigator' || true; \
                      pkill -f 'behavior_server' || true; \
                      pkill -f 'velocity_smoother' || true; \
                      pkill -f 'smoother_server' || true; \
                      pkill -f 'lifecycle_manager' || true; \
                      pkill -f 'csi_camera_node' || true; \
                      pkill -f 'aruco_detector_node' || true; \
                      pkill -f 'aruco_lidar_approach' || true; \
                      pkill -f 'scan_sanitizer_node' || true; \
                      pkill -f 'mecharm_driver_node' || true; \
                      pkill -f 'twist_mux' || true" || true
        echo "[i] Pila de clasificacion detenida."
        ;;

    shell)
        require_container
        in_container "${SOURCE_ENV}; exec bash"
        ;;

    help|-h|--help)
        sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
        ;;

    *)
        echo "Subcomando desconocido: $1" >&2
        "${BASH_SOURCE[0]}" --help >&2
        exit 2
        ;;
esac
