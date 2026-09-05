#!/usr/bin/env bash
# =====================================================================
#  Bringup completo del robot REAL dentro del contenedor jetson-robot.
#
#  Lanza (cada uno en segundo plano; Ctrl+C los detiene todos):
#    * myagv_odometry            (base + /odom + /cmd_vel)
#    * ydlidar_ros2_driver       (/scan)
#    * home_service_bringup      (camara CSI, ArUco, brazo, twist_mux)
#    * teleop de mando           (opcional, -> /cmd_vel_joy)
#
#  Nav2 y la mision se lanzan aparte (ver scripts/myagv_commands.sh).
#
#  Variables de entorno:
#    START_TELEOP=1     lanzar el teleop de mando (por defecto NO: un
#                       teleop activo publicando 0 tiene prioridad en
#                       twist_mux y bloquearia Nav2/ArUco)
#    START_BRINGUP=0    no lanzar la capa de percepcion/brazo
#    CAMERA_SOURCE=v4l2 fuente de camara alternativa (por defecto nvargus)
#    ARM_PORT=/dev/ttyACM0
#    MARKER_LENGTH=0.08
# =====================================================================
set -eo pipefail

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /workspace/install/setup.bash

START_TELEOP="${START_TELEOP:-0}"
START_BRINGUP="${START_BRINGUP:-1}"
CAMERA_SOURCE="${CAMERA_SOURCE:-nvargus}"
ARM_PORT="${ARM_PORT:-/dev/ttyACM0}"
MARKER_LENGTH="${MARKER_LENGTH:-0.08}"

pids=()
cleanup() {
    trap - TERM INT EXIT
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup TERM INT EXIT

# La Jetson Nano (4 nucleos + escritorio del host) se satura al mapear y
# el hilo de lectura serie del LiDAR pierde tramas -> "Checksum error" ->
# scans corruptos -> el mapa SLAM se desfasa y las paredes se superponen.
# Solucion sin recrear el contenedor: aislar el laser y la odometria en
# el ultimo nucleo (taskset, no necesita capabilities) para que ninguna
# otra carga (slam, rviz) los interrumpa. Verificado: con el laser en un
# nucleo propio, 3 nucleos al 100% no anaden ni un Checksum error.
# Si ademas hay CAP_SYS_NICE (run_jetson_robot.sh la anade), tiempo real.
NCPU="$(nproc)"
SERIAL_CPU="$(( NCPU > 1 ? NCPU - 1 : 0 ))"
PRIO=(taskset -c "${SERIAL_CPU}")
if chrt -r 1 true >/dev/null 2>&1; then
    PRIO=(taskset -c "${SERIAL_CPU}" chrt -r 20)
    echo "[bringup] laser/odometria: nucleo ${SERIAL_CPU} aislado + tiempo real"
else
    echo "[bringup] laser/odometria: nucleo ${SERIAL_CPU} aislado (sin CAP_SYS_NICE)"
fi
# El resto del stack, fuera de ese nucleo.
OTHER_CPUS="0-$(( SERIAL_CPU > 1 ? SERIAL_CPU - 1 : 0 ))"
export SLAM_CPU_AFFINITY="${OTHER_CPUS}"

# --- Base + odometria -------------------------------------------------
"${PRIO[@]}" ros2 run myagv_odometry myagv_odometry_node &
pids+=("$!")

# --- LiDAR ----------------------------------------------------------
"${PRIO[@]}" ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
    params_file:=/workspace/src/elephant_myagv_ros2/ydlidar_ros2_driver/params/X2.yaml &
pids+=("$!")

# --- Percepcion + brazo + twist_mux -------------------------------
if [ "${START_BRINGUP}" != "0" ]; then
    ros2 launch home_service_bringup robot.launch.py \
        use_sim_time:=false \
        camera_source:="${CAMERA_SOURCE}" \
        marker_length:="${MARKER_LENGTH}" \
        arm_port:="${ARM_PORT}" &
    pids+=("$!")
fi

# --- Teleop de mando (opcional) -----------------------------------
if [ "${START_TELEOP}" != "0" ]; then
    ros2 launch myagv_teleop_joy bluetooth_gamepad_teleop.launch.py \
        cmd_vel_topic:=/cmd_vel_joy &
    pids+=("$!")
fi

wait "${pids[@]}"
