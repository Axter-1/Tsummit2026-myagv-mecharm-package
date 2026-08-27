#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${ROOT}/scripts/env.sh"

cd "${ROOT}"

colcon build \
    --symlink-install

source "${ROOT}/install/setup.bash"

echo
echo "Build complete."
