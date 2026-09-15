#!/bin/bash
# 单独开 RViz：官方 hobot_stereonet 彩色点云（板端需 DISPLAY，或同网 PC）
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh

CFG="${SCRIPT_DIR}/depth_cloud.rviz"
if [ ! -f "$CFG" ]; then
  echo "missing $CFG"
  exit 1
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "无 DISPLAY：请在板端桌面终端运行，或在同网 PC："
  echo "  export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  rviz2 -d $CFG"
  echo "  # Fixed Frame=camera_link，Topic=/StereoNetNode/stereonet_pointcloud2"
  exit 1
fi

echo "[rviz] Fixed Frame=camera_link"
echo "[rviz] Topic=/StereoNetNode/stereonet_pointcloud2 (RGB8 / Flat Squares)"
echo "[rviz] 确认另一终端已运行: bash .../08_depth_camera/run.sh"
exec rviz2 -d "$CFG"
