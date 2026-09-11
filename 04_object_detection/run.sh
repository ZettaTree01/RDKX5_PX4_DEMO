#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
exec ros2 launch "$SCRIPT_DIR/object_detection.launch.py" "$@"
