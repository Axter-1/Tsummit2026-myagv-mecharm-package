#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"

if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo
    echo "ERROR: ROS 2 ${ROS_DISTRO} is not installed."
    echo
    echo "Install ROS 2 ${ROS_DISTRO} before running bootstrap."
    exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo
echo "========================================"
echo "Installing development tools"
echo "========================================"

sudo apt update

sudo apt install -y \
    git \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions

echo
echo "========================================"
echo "Restoring external repositories"
echo "========================================"

mkdir -p "${ROOT}/src"

if [ -s "${ROOT}/dependencies.repos" ]; then

    vcs import \
        "${ROOT}/src" \
        < "${ROOT}/dependencies.repos"

fi

echo
echo "========================================"
echo "Initializing rosdep"
echo "========================================"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update

echo
echo "========================================"
echo "Installing ROS dependencies"
echo "========================================"

rosdep install \
    --from-paths "${ROOT}/src" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    -r \
    -y

echo
echo "========================================"
echo "Building workspace"
echo "========================================"

cd "${ROOT}"

colcon build \
    --symlink-install

echo
echo "========================================"
echo "Installation complete"
echo "========================================"
echo
echo "Run:"
echo
echo "  source ${ROOT}/scripts/env.sh"
echo
