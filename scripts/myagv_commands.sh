#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER="${CONTAINER:-myagv-robot}"
MAP="${MAP:-/workspace/maps/home_service_challenge_myagv.yaml}"
PARAMS="${PARAMS:-/workspace/install/home_service_bringup/share/home_service_bringup/config/nav2_real.yaml}"

DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
    DOCKER=(sudo docker)
fi

EXEC_FLAGS=(-i)
if [ -t 0 ] && [ -t 1 ]; then
    EXEC_FLAGS=(-it)
fi

docker_exec() {
    "${DOCKER[@]}" exec "${EXEC_FLAGS[@]}" "${CONTAINER}" bash -lc "$1"
}

require_container() {
    if ! "${DOCKER[@]}" inspect "${CONTAINER}" >/dev/null 2>&1; then
        printf 'ERROR: el contenedor %s no está ejecutándose.\n' "${CONTAINER}" >&2
        printf 'Inícialo con: ./docker/run_jetson_robot.sh\n' >&2
        exit 1
    fi
}

case "${1:-help}" in
    teleop)
        require_container
        if "${DOCKER[@]}" exec "${EXEC_FLAGS[@]}" "${CONTAINER}" bash -lc \
            'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 node list' \
            2>/dev/null | grep -qx '/bluetooth_gamepad_teleop'; then
            printf 'La teleoperación ya está activa en %s.\n' "${CONTAINER}"
        else
            docker_exec 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 launch myagv_teleop_joy bluetooth_gamepad_teleop.launch.py cmd_vel_topic:=/cmd_vel_joy'
        fi
        ;;
    nav2)
        require_container
        docker_exec "pkill -f '[b]luetooth_gamepad_teleop --ros-args' 2>/dev/null || true; source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; ros2 launch home_service_bringup nav2.launch.py map:=${MAP} params_file:=${PARAMS} autostart:=true"
        ;;
    rqt)
        require_container
        docker_exec 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; rqt'
        ;;
    inputs)
        require_container
        docker_exec "python3 /workspace/scripts/view_gamepad_inputs.py '${GAMEPAD_NAME:-TGZ Controller}'"
        ;;
    cmd_vel)
        require_container
        docker_exec 'source /opt/ros/humble/setup.bash; ros2 topic echo /cmd_vel --qos-reliability reliable --qos-durability volatile'
        ;;
    help|-h|--help)
        printf '%s\n' \
            'Uso: ./scripts/myagv_commands.sh {teleop|nav2|rqt|inputs|cmd_vel}' \
            '' \
            '  teleop  Abre la teleoperación Bluetooth (si no está activa).' \
            '  nav2    Inicia Nav2 autónomo con el mapa y parámetros reales.' \
            '  rqt     Abre RQt dentro del contenedor.' \
            '  inputs  Muestra eventos del mando en tiempo real.' \
            '  cmd_vel Muestra /cmd_vel con QoS explícito, sin depender del daemon.' \
            '' \
            'Variables opcionales: MAP=/workspace/... PARAMS=/workspace/... CONTAINER=nombre'
        ;;
    *)
        printf 'Comando desconocido: %s\n\n' "$1" >&2
        "${BASH_SOURCE[0]}" --help >&2
        exit 2
        ;;
esac
