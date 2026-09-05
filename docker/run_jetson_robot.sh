#!/usr/bin/env bash
# =====================================================================
#  Lanza el contenedor del robot real en la Jetson Nano del myAGV.
#
#  Uso:
#     ./docker/run_jetson_robot.sh              # inicia el bringup en segundo plano
#     ./docker/run_jetson_robot.sh sleep infinity # contenedor persistente sin hardware activo
#     DETACH=0 ./docker/run_jetson_robot.sh     # ejecución interactiva
#     BUILD=1 ./docker/run_jetson_robot.sh      # fuerza recompilar
#     ./docker/run_jetson_robot.sh ros2 launch myagv_odometry myagv_active.launch.py
# =====================================================================
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "$#" -eq 0 ]; then
    set -- bash /workspace/docker/run_all_robot_nodes.sh
fi

IMAGE="${IMAGE:-myagv-home-service:jetson-robot}"
CONTAINER="${CONTAINER:-myagv-robot}"
DETACH="${DETACH:-1}"

DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
    DOCKER=(sudo docker)
fi

# Runtime nvidia: es lo que inyecta las librerias L4T listadas en
# /etc/nvidia-container-runtime/host-files-for-container.d/l4t.csv
# (libnvargus*, libgstnvarguscamerasrc.so, libnvbuf_utils...). Sin esto
# el plugin GStreamer de la camara CSI no existe dentro del contenedor
# y cv2.VideoCapture(..., CAP_GSTREAMER) se queda colgado sin fotogramas
# ni error. Se detecta en vez de forzar por si el host no lo tiene
# configurado (docker info no lista "nvidia" en Runtimes).
RUNTIME_ARGS=()
if "${DOCKER[@]}" info 2>/dev/null | grep -qw nvidia; then
    # NVIDIA_VISIBLE_DEVICES/DRIVER_CAPABILITIES: sin estas dos variables
    # el hook OCI de nvidia-container-runtime NO inyecta el CSV aunque
    # el runtime este activo (se queda en modo "legacy" sin hacer nada
    # en L4T). Con ellas monta las libs de l4t.csv (libnvargus*,
    # libgstnvarguscamerasrc.so, /usr/lib/aarch64-linux-gnu/tegra/...).
    RUNTIME_ARGS=(
        --runtime nvidia
        -e NVIDIA_VISIBLE_DEVICES=all
        -e NVIDIA_DRIVER_CAPABILITIES=all
    )
else
    echo "AVISO: runtime 'nvidia' no disponible en docker info; la" >&2
    echo "       camara CSI probablemente no funcionara en el contenedor." >&2
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker no está instalado en la Jetson."
    exit 1
fi

# --- Acceso a hardware -------------------------------------------------
# Placa base del myAGV, LiDAR YDLidar, mando Bluetooth, cámara.
# Ajusta las rutas /dev/tty* a las de tu robot (ver 'ls -l /dev/serial/by-id').
DEV_ARGS=()
for d in /dev/ttyS0 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyAMA0 \
         /dev/ttyTHS1 /dev/ttyTHS2 /dev/video0 \
         /dev/nvhost-* /dev/nvmap; do
    # nvhost-*/nvmap: los necesita nvarguscamerasrc para negociar el
    # canal VI/ISP con el demonio nvargus-daemon del host (ver mas abajo
    # el socket). Sin ellos cv2.VideoCapture(..., CAP_GSTREAMER) no da
    # ERROR: se queda colgado sin abrir ni fallar, y csi_camera_node
    # jamas publica un fotograma aunque el nodo parezca sano.
    [ -e "$d" ] && DEV_ARGS+=(--device "$d")
done
# Mando (evdev). El evento cambia al reconectar; se permite dinámicamente
# toda la clase evdev y se monta el directorio para no depender de eventX.
if [ -d /dev/input ]; then
    DEV_ARGS+=(-v /dev/input:/dev/input:ro)
    DEV_ARGS+=(--device-cgroup-rule='c 13:* rmw')
fi

# Camara CSI (nvarguscamerasrc). El pipeline habla con el demonio
# nvargus-daemon del HOST a traves de este socket Unix; sin montarlo,
# cv2.VideoCapture(..., cv2.CAP_GSTREAMER) abre "correctamente" pero
# nunca entrega un solo fotograma (el nodo no llega ni a loguear un
# error: se queda esperando la respuesta del daemon que no existe
# dentro del contenedor).
if [ -S /tmp/argus_socket ]; then
    DEV_ARGS+=(-v /tmp/argus_socket:/tmp/argus_socket)
else
    echo "AVISO: /tmp/argus_socket no existe en el host (nvargus-daemon" >&2
    echo "       parado?). La camara CSI no dara imagen dentro del contenedor." >&2
fi

# Todos los nodos del robot viven en este contenedor. CycloneDDS debe usar
# loopback siempre: Ethernet no es necesaria y una interfaz ausente no puede
# impedir que el mando publique a la base local.
DDS_ARGS=(
    -e 'CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
)

# --- X11 opcional (solo si vas a abrir RViz en la Jetson) ------------
X_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
    xhost +local:docker >/dev/null 2>&1 || true
    X_ARGS+=(-e DISPLAY="${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

if "${DOCKER[@]}" inspect "${CONTAINER}" >/dev/null 2>&1; then
    if "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" | grep -q true; then
        echo "El contenedor ${CONTAINER} ya está ejecutándose."
    else
        "${DOCKER[@]}" start "${CONTAINER}"
        echo "Contenedor ${CONTAINER} reanudado."
    fi
    exit 0
fi

RUN_MODE=(-d)
if [ "${DETACH}" = "0" ]; then
    RUN_MODE=(-it)
fi

"${DOCKER[@]}" run "${RUN_MODE[@]}" \
    "${RUNTIME_ARGS[@]}" \
    --restart unless-stopped \
    --name "${CONTAINER}" \
    --network host \
    --ipc host \
    --cap-add SYS_NICE \
    --ulimit rtprio=99 \
    -e BUILD="${BUILD:-0}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    "${DDS_ARGS[@]}" \
    --group-add dialout \
    --group-add video \
    --group-add input \
    -v "${ROOT}:/workspace:rw" \
    -v /run/udev:/run/udev:ro \
    -v /dev/bus/usb:/dev/bus/usb \
    "${DEV_ARGS[@]}" \
    "${X_ARGS[@]}" \
    --entrypoint /usr/local/bin/robot-entrypoint.sh \
    "${IMAGE}" \
    "${@:-bash}"

if [ "${DETACH}" = "1" ]; then
    echo "Contenedor ${CONTAINER} montado en segundo plano."
    echo "Pausar:    docker pause ${CONTAINER}"
    echo "Reanudar:  docker unpause ${CONTAINER}"
    echo "Estado:    docker ps -a --filter name=${CONTAINER}"
fi
