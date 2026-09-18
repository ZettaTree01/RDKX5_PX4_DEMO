#!/usr/bin/env bash
# 例程9：完整 C++ EGO-Planner（需先 setup_full_ego.sh）
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/run_flight.sh

EGO_WS="${EGO_WS:-$SCRIPT_DIR/ego_ws}"
if [ ! -f "$EGO_WS/install/setup.bash" ]; then
  echo "[09] 未找到完整 EGO 安装：$EGO_WS/install" >&2
  echo "     请先在板端执行：bash $SCRIPT_DIR/setup_full_ego.sh" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$EGO_WS/install/setup.bash"

if ! ros2 pkg prefix ego_planner >/dev/null 2>&1; then
  echo "[09] ego_planner 包不可用，请重新编译 setup_full_ego.sh" >&2
  exit 1
fi

WANT_STEREO=1
for a in "$@"; do
  case "$a" in
    source:=simulate|source:=orbbec|source:=realsense) WANT_STEREO=0 ;;
    start_stereo:=false|start_stereo:=False) WANT_STEREO=0 ;;
  esac
done
if [ "$WANT_STEREO" = "1" ]; then
  echo "[09] 确保 GS130W mipi dual（例程8 ensure）…"
  bash /app/zettatree_demo/08_depth_camera/ensure_mipi_bpu.sh
fi

echo "[09] 完整 C++ EGO-Planner + Stereonet；OpenCV 深彩|三维；默认开 RViz"
# Ctrl+C：停栈并强制上锁
_flight_run ros2 launch "$SCRIPT_DIR/ego_full.launch.py" \
  source:=stereonet start_stereo:=true rviz:=true \
  "$@"
