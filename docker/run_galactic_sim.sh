#!/usr/bin/env bash

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE="myagv-home-service:galactic-sim"
CONTAINER="myagv-galactic-sim"

if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: DISPLAY no está definido."
    exit 1
fi

xhost +si:localuser:"$(whoami)" >/dev/null 2>&1 || true
xhost +local:docker >/dev/null 2>&1 || true

GPU_ARGS=()

# ------------------------------------------------------------
# NVIDIA
# ------------------------------------------------------------

if command -v nvidia-smi >/dev/null 2>&1; then

    if docker info 2>/dev/null | grep -qi nvidia; then

        echo "[GPU] NVIDIA runtime disponible."

        GPU_ARGS+=(
            --gpus all
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=all
        )

    elif [ -e /dev/dri ]; then

        echo "[GPU] NVIDIA detectada, usando /dev/dri."

        GPU_ARGS+=(
            --device=/dev/dri:/dev/dri
        )

    else

        echo "[GPU] NVIDIA detectada sin runtime Docker."
        echo "[GPU] Se usará renderizado software."

    fi

# ------------------------------------------------------------
# Intel / AMD
# ------------------------------------------------------------

elif [ -e /dev/dri ]; then

    echo "[GPU] Intel/AMD DRM detectado."

    GPU_ARGS+=(
        --device=/dev/dri:/dev/dri
    )

else

    echo "[GPU] Sin GPU directa."
    echo "[GPU] Se intentará llvmpipe."

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
