#!/usr/bin/env bash
# Diagnostico no destructivo del runtime ROS del robot real.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${CONTAINER:-myagv-robot}"
DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
    DOCKER=(sudo docker)
fi

failed=0
warned=0

ok() { printf '  OK      %s\n' "$*"; }
fail() { printf '  FALTA   %s\n' "$*"; failed=1; }
warn() { printf '  AVISO   %s\n' "$*"; warned=1; }

inside() {
    "${DOCKER[@]}" exec "${CONTAINER}" bash -lc "$*"
}

printf '== Integridad del entorno del robot ==\n'

if [ -f "${ROOT}/dependencies.repos" ]; then
    ok 'dependencies.repos'
else
    fail 'dependencies.repos'
fi

if ! "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" \
    2>/dev/null | grep -qx true; then
    fail "contenedor ${CONTAINER} en ejecucion"
    exit 2
fi
ok "contenedor ${CONTAINER} en ejecucion"

for repo in elephant_myagv_ros2 elephant_mycobot_ros2; do
    if inside "test -d /workspace/src/${repo}/.git"; then
        ok "fuente externa ${repo}"
    else
        fail "fuente externa ${repo}"
    fi
done

if inside "git -c safe.directory=/workspace/src/elephant_myagv_ros2 -C /workspace/src/elephant_myagv_ros2 fsck --no-dangling >/dev/null"; then
    ok 'integridad git elephant_myagv_ros2'
else
    fail 'integridad git elephant_myagv_ros2'
fi

if inside "git -c safe.directory=/workspace/src/elephant_mycobot_ros2 -C /workspace/src/elephant_mycobot_ros2 fsck --no-dangling >/dev/null"; then
    ok 'integridad git elephant_mycobot_ros2'
else
    fail 'integridad git elephant_mycobot_ros2'
fi

printf '\n== Paquetes ROS instalados ==\n'
packages=(
    home_service_interfaces
    myagv_odometry
    ydlidar_ros2_driver
    myagv_description
    myagv_camera
    home_service_navigation
    home_service_perception
    myagv_mecharm_service
    home_service_behaviors
    home_service_bringup
)
for package in "${packages[@]}"; do
    if inside "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash 2>/dev/null; ros2 pkg prefix '${package}' >/dev/null 2>&1"; then
        ok "${package}"
    else
        fail "paquete ROS ${package} no instalado"
    fi
done

printf '\n== Artefactos criticos ==\n'
for path in \
    /workspace/src/elephant_myagv_ros2/ydlidar_ros2_driver/params/X2.yaml \
    /workspace/install/myagv_description/share/myagv_description/urdf/myAGV.urdf \
    /workspace/install/home_service_bringup/share/home_service_bringup/launch/robot.launch.py \
    /workspace/install/home_service_bringup/share/home_service_bringup/config/twist_mux_real.yaml \
    /workspace/install/home_service_behaviors/share/home_service_behaviors/config/grasp_calibrations.yaml; do
    if inside "test -r '${path}'"; then
        ok "${path#/workspace/}"
    else
        fail "archivo ${path#/workspace/}"
    fi
done

if inside "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; driver=\$(ros2 pkg prefix ydlidar_ros2_driver)/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node; test -x \"\${driver}\" && ! ldd \"\${driver}\" | grep -q 'not found'"; then
    ok 'dependencias dinamicas del driver YDLidar'
else
    fail 'dependencias dinamicas del driver YDLidar'
fi

printf '\n== Hardware disponible ==\n'
for device in /dev/ttyTHS1 /dev/ttyACM0; do
    if inside "test -e '${device}'"; then
        ok "${device}"
    else
        warn "${device} no esta montado en el contenedor"
    fi
done
if inside 'test -S /tmp/argus_socket'; then
    ok '/tmp/argus_socket (camara CSI)'
else
    warn '/tmp/argus_socket no esta disponible; la camara CSI no funcionara'
fi

if [ "${failed}" -ne 0 ]; then
    printf '\nRepara el install con:\n'
    printf '  ./scripts/tsummit.sh build myagv_odometry ydlidar_ros2_driver myagv_description myagv_camera home_service_navigation home_service_bringup\n'
    printf 'Despues repite: ./scripts/tsummit.sh doctor\n'
    exit 1
fi

if [ "${warned}" -ne 0 ]; then
    printf '\nIntegridad ROS correcta; revisa los avisos de hardware antes de mover el robot.\n'
else
    printf '\nEntorno listo para iniciar la pila del robot.\n'
fi
