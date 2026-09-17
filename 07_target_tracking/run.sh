#!/usr/bin/env bash
# 例程7 停机坪 H 标对准降落入口。
# 加载演示环境与飞控安全封装后，启动 target_tracking.launch.py。
# 其余 key:=value 透传给 launch（如 arm:=true）。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/run_flight.sh
ros2 launch "$SCRIPT_DIR/target_tracking.launch.py" "$@"
