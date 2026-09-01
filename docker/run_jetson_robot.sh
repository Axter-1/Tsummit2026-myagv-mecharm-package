#!/usr/bin/env bash
# =====================================================================
#  Lanza el contenedor del robot real en la Jetson Nano del myAGV.
#
#  Uso:
#     ./docker/run_jetson_robot.sh              # abre una shell
#     BUILD=1 ./docker/run_jetson_robot.sh      # fuerza recompilar
#     ./docker/run_jetson_robot.sh ros2 launch myagv_odometry myagv_active.launch.py
# =====================================================================
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE="${IMAGE:-myagv-home-service:jetson-robot}"
CONTAINER="${CONTAINER:-myagv-robot}"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker no está instalado en la Jetson."
    exit 1
fi

# --- Acceso a hardware -------------------------------------------------
# Placa base del myAGV, LiDAR YDLidar, mando Bluetooth, cámara.
# Ajusta las rutas /dev/tty* a las de tu robot (ver 'ls -l /dev/serial/by-id').
DEV_ARGS=()
for d in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyAMA0 /dev/video0; do
    [ -e "$d" ] && DEV_ARGS+=(--device "$d")
done
# Mando (evdev). Se monta el directorio entero porque el nº de eventX cambia.
[ -d /dev/input ] && DEV_ARGS+=(-v /dev/input:/dev/input:ro)

# --- X11 opcional (solo si vas a abrir RViz en la Jetson) ------------
X_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
    xhost +local:docker >/dev/null 2>&1 || true
    X_ARGS+=(-e DISPLAY="${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

docker run --rm -it \
    --name "${CONTAINER}" \
    --network host \
    --ipc host \
    -e BUILD="${BUILD:-0}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    --group-add dialout \
    --group-add video \
    --group-add input \
    -v "${ROOT}:/workspace:rw" \
    -v /run/udev:/run/udev:ro \
    -v /dev/bus/usb:/dev/bus/usb \
    "${DEV_ARGS[@]}" \
    "${X_ARGS[@]}" \
    "${IMAGE}" \
    "${@:-bash}"
