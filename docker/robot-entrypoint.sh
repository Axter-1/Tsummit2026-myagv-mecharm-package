#!/usr/bin/env bash
set -e

# El runtime nvidia (docker/run_jetson_robot.sh) monta las librerias L4T
# de /etc/nvidia-container-runtime/host-files-for-container.d/l4t.csv
# (libnvargus*, libgstnvarguscamerasrc.so, libnvrm*...) en
# /usr/lib/aarch64-linux-gnu/tegra JUSTO al crear el contenedor, DESPUES
# de que la imagen construyera su cache de ld.so. Dos problemas, dos
# arreglos:
#   1. esa subcarpeta NO esta en el ld.so.conf.d por defecto (solo lo
#      estan los directorios multiarch de nivel superior) -> hay que
#      declararla.
#   2. aunque este declarada, la cache de ldconfig sigue siendo la de
#      cuando se construyo la imagen -> hay que regenerarla.
# Sin esto, cualquier binario que dependa de esas libs (cv2 de la
# camara CSI incluido) falla con "cannot open shared object file"
# aunque el .so este ahi mismo, visible con 'ls'.
{
    [ -d /usr/lib/aarch64-linux-gnu/tegra ] && echo /usr/lib/aarch64-linux-gnu/tegra
    # tegra-egl: aqui vive libEGL_nvidia.so.0 (el ICD real de EGL/Argus).
    # Sin esta ruta, el dispatcher libglvnd no puede cargar el ICD nvidia
    # (aunque su .json en /usr/share/glvnd/egl_vendor.d/ si se monte) y
    # cae al ICD de Mesa/swrast (software, y encima incompleto en esta
    # imagen) -> Argus no consigue un EGLDisplay -> nvarguscamerasrc
    # "abre" pero jamas entrega un fotograma.
    [ -d /usr/lib/aarch64-linux-gnu/tegra-egl ] && echo /usr/lib/aarch64-linux-gnu/tegra-egl
} > /etc/ld.so.conf.d/nvidia-tegra.conf
ldconfig 2>/dev/null || true

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Restaurar dependencias externas (myagv_ros2, mycobot_ros2) si aún no están.
if [ -s /workspace/dependencies.repos ] && [ ! -d /workspace/src/elephant_myagv_ros2 ]; then
    echo "[entrypoint] vcs import de dependencias externas..."
    vcs import /workspace/src < /workspace/dependencies.repos || true
fi

# Compilar el workspace la primera vez (o si se fuerza con BUILD=1).
if [ "${BUILD:-0}" = "1" ] || [ ! -f /workspace/install/setup.bash ]; then
    echo "[entrypoint] colcon build (esto puede tardar en la Jetson)..."
    cd /workspace
    rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}" || true
    # La Nano tiene 4 GB: limitar la paralelización evita el OOM killer.
    MAKEFLAGS="-j2" colcon build --symlink-install \
        --parallel-workers 1 \
        --event-handlers console_direct+
fi

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"
