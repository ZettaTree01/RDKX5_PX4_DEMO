#!/usr/bin/env bash
# 例程10：目标跟随。planner:=ego（默认，完整 C++ EGO 避障跟随）
# 或 planner:=direct（直接位置跟随对照）。必须拆桨。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/run_flight.sh

# 从参数中识别 planner:=ego|direct（其余原样透传给 ros2 launch）
PLANNER=ego
for a in "$@"; do
  case "$a" in
    planner:=*) PLANNER="${a#planner:=}" ;;
  esac
done

WANT_STEREO=1
for a in "$@"; do
  case "$a" in
    source:=simulate|source:=orbbec|source:=realsense) WANT_STEREO=0 ;;
    start_stereo:=false|start_stereo:=False) WANT_STEREO=0 ;;
  esac
done
if [ "$WANT_STEREO" = "1" ]; then
  echo "[10] 确保 GS130W mipi dual（例程8 ensure）…"
  bash /app/zettatree_demo/08_depth_camera/ensure_mipi_bpu.sh
fi

if [ "${PLANNER,,}" = "ego" ]; then
  EGO_WS="${EGO_WS:-/app/zettatree_demo/09_depth_nav/ego_ws}"
  if [ ! -f "$EGO_WS/install/setup.bash" ]; then
    echo "[10] 未找到完整 EGO 安装：$EGO_WS/install" >&2
    echo "     请先执行：bash ${SCRIPT_DIR}/setup.sh" >&2
    exit 1
  fi
  # shellcheck disable=SC1091
  source "$EGO_WS/install/setup.bash"
    if ! ros2 pkg prefix ego_planner >/dev/null 2>&1; then
    echo "[10] ego_planner 包不可用，请重跑 setup.sh" >&2
    exit 1
  fi
  echo "[10] 完整 C++ EGO-Planner 动态目标跟随（/move_base_simple/goal）"
else
  echo "[10] 直接位置跟随（planner:=direct，无避障规划器对照）"
fi

# 用 _flight_run：Ctrl+C 立刻杀 launch 进程组，再 stop_nav + 上锁
_flight_run ros2 launch "$SCRIPT_DIR/target_follow.launch.py" "$@"
