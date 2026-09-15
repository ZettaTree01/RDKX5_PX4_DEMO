#!/bin/bash
# 例程9：默认跑完整 C++ EGO-Planner；可用 backend:=python 退回 Python A*
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

BACKEND=full
FILTERED=()
for a in "$@"; do
  case "$a" in
    backend:=python|backend:=py|ego:=python)
      BACKEND=python
      ;;
    backend:=full|ego:=full|ego:=true)
      BACKEND=full
      ;;
    *)
      FILTERED+=("$a")
      ;;
  esac
done

if [ "$BACKEND" = "python" ]; then
  echo "[09] 使用 Python 同构 EGO（A*）；完整 C++ 请省略 backend:=python"
  # shellcheck disable=SC1091
  source /app/zettatree_demo/_common/env.sh
  # shellcheck disable=SC1091
  source /app/zettatree_demo/_common/run_flight.sh
  WANT_STEREO=1
  for a in "${FILTERED[@]}"; do
    case "$a" in
      source:=simulate|source:=orbbec|source:=realsense) WANT_STEREO=0 ;;
      start_stereo:=false|start_stereo:=False) WANT_STEREO=0 ;;
    esac
  done
  if [ "$WANT_STEREO" = "1" ]; then
    bash /app/zettatree_demo/08_depth_camera/ensure_mipi_bpu.sh || true
  fi
  ros2 launch "$SCRIPT_DIR/depth_nav.launch.py" \
    source:=stereonet start_stereo:=true ego:=true rviz:=true \
    "${FILTERED[@]}"
  exit $?
fi

exec bash "$SCRIPT_DIR/run_ego_full.sh" "${FILTERED[@]}"
