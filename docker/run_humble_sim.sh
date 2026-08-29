#!/usr/bin/env bash

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE="myagv-home-service:humble-sim"
CONTAINER="myagv-humble-sim"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker no está instalado."
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: DISPLAY no está definido."
    echo "Este script requiere una sesión gráfica X11/XWayland."
    exit 1
fi

# Permitir temporalmente a procesos locales de Docker usar X11.
xhost +local:docker >/dev/null 2>&1 || \
xhost +local:root >/dev/null 2>&1 || true

GPU_ARGS=()

# NVIDIA
if command -v nvidia-smi >/dev/null 2>&1; then
    if docker info 2>/dev/null | grep -qi nvidia; then
        echo "[GPU] NVIDIA detectada."
        GPU_ARGS+=(
            --gpus all
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=all
        )
    else
        echo "[GPU] NVIDIA detectada, pero Docker NVIDIA runtime no parece disponible."
        echo "[GPU] Se intentará ejecutar sin --gpus."
    fi

# Intel / AMD
elif [ -e /dev/dri ]; then
    echo "[GPU] /dev/dri detectado."
    GPU_ARGS+=(
        --device=/dev/dri:/dev/dri
    )

else
    echo "[GPU] No se detectó acceso directo a GPU."
    echo "[GPU] Gazebo podrá intentar renderizado por software."
fi

docker run \
    --rm \
    -it \
    --name "${CONTAINER}" \
    --network host \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${ROOT}:/repo:ro" \
    "${GPU_ARGS[@]}" \
    "${IMAGE}"
