#!/usr/bin/env bash
# 02 台架位姿模拟入口：加载 ROS 与飞行公共环境后启动 bench_pose_sim.launch.py。
# 参数原样转给 launch（如 arm:=true、altitude:=0.1）。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/run_flight.sh
ros2 launch "$SCRIPT_DIR/bench_pose_sim.launch.py" "$@"
