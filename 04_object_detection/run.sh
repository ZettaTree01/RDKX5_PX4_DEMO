#!/usr/bin/env bash
# 04 目标检测入口：加载 ROS 环境后启动 object_detection.launch.py（相机 + YOLO）。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
exec ros2 launch "$SCRIPT_DIR/object_detection.launch.py" "$@"
