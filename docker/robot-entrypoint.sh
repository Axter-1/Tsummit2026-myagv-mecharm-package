#!/usr/bin/env bash
set -e

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
