#!/usr/bin/env bash
# 05 摄像头避障入口：加载 ROS 与飞行公共环境后启动 obstacle_avoidance.launch.py。
# 参数原样转给 launch（如 arm:=true、bench:=true、camera_source:=usb）。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/run_flight.sh
ros2 launch "$SCRIPT_DIR/obstacle_avoidance.launch.py" "$@"
