#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${ROOT}/scripts/env.sh"

ros2 launch \
    home_service_mission \
    mission.launch.py \
    use_sim_time:=true
