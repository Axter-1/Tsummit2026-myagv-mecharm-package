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

# --- Base + odometria -------------------------------------------------
ros2 run myagv_odometry myagv_odometry_node &
pids+=("$!")

# --- LiDAR ----------------------------------------------------------
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
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
