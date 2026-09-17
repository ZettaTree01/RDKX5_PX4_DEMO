#!/usr/bin/env bash
# 例程11 编队飞行入口。
# 加载演示环境与飞控安全封装后，启动 formation_flight.launch.py。
# 参数示例：drone_id:=0 num_drones:=3 arm:=true。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/run_flight.sh
ros2 launch "$SCRIPT_DIR/formation_flight.launch.py" "$@"
