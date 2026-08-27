#!/usr/bin/env bash

# This file must be sourced:
#
#   source scripts/env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOME_SERVICE_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ERROR: ROS 2 ${ROS_DISTRO} not found at:"
    echo "  $ROS_SETUP"
    return 1 2>/dev/null || exit 1
fi

source "$ROS_SETUP"

if [ -f "${HOME_SERVICE_WS}/install/setup.bash" ]; then
    source "${HOME_SERVICE_WS}/install/setup.bash"
fi

# Keep Gazebo from scanning the complete ROS share directory.
export GAZEBO_MODEL_PATH="${HOME_SERVICE_WS}/gazebo_models:${HOME}/.gazebo/models:/usr/share/gazebo-11/models:/opt/ros/${ROS_DISTRO}/share/turtlebot3_manipulation_gazebo/models:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models"

echo "HOME_SERVICE_WS=${HOME_SERVICE_WS}"
echo "ROS_DISTRO=${ROS_DISTRO}"
