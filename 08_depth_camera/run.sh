#!/usr/bin/env bash
# 例程8：GS130W 双目 → Depth → OpenCV【深彩 | 三维俯视】
# 默认不启 RViz
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh

if [ ! -w /userdata/.roslog ] 2>/dev/null; then
  echo sunrise | sudo -S mkdir -p /userdata/.roslog >/dev/null 2>&1 || true
  echo sunrise | sudo -S chown -R "$(id -un):$(id -gn)" /userdata/.roslog >/dev/null 2>&1 || true
  echo sunrise | sudo -S chmod 777 /userdata/.roslog >/dev/null 2>&1 || true
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-/userdata/.roslog}"
mkdir -p /tmp/zettatree_roslog "$ROS_LOG_DIR" 2>/dev/null || true

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[08] 警告: 未设置 DISPLAY。请在板端桌面终端运行，否则 OpenCV 无法弹窗。"
fi

echo "[08] 链路: 双目 → Depth → OpenCV(深彩|三维) + RViz(官方彩色点云)"
bash "$SCRIPT_DIR/ensure_mipi_bpu.sh"
ros2 launch "$SCRIPT_DIR/depth_camera.launch.py" \
  source:=stereonet start_mipi:=false start_stereonet:=true \
  show:=true rviz:=true map:=false \
  baseline_m:=0.07917 rotate_cw:=0 \
  min_range:=0.3 max_range:=5.0 snapshot:=none \
  "$@"
