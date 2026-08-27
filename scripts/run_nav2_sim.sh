#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${ROOT}/scripts/env.sh"

ros2 launch nav2_bringup bringup_launch.py \
    map:="${ROOT}/maps/home_service_challenge_myagv.yaml" \
    use_sim_time:=true \
    params_file:="${ROOT}/src/mobile_manipulator_sim/config/nav2_sim.yaml" \
    autostart:=true \
    use_composition:=False
