#!/bin/bash
# 统一加载板端 ROS2 / TogetheROS 环境
# TROS setup.bash 会访问未定义变量；若调用方开了 set -u，先临时关闭。
_ament_nounset=0
case "$-" in *u*) _ament_nounset=1; set +u ;; esac
if [ -f /opt/tros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  [ "$_ament_nounset" = 1 ] && set -u
  echo "未找到 /opt/tros/humble 或 /opt/ros/humble，ROS2 例程无法运行" >&2
  return 1 2>/dev/null || exit 1
fi
[ "$_ament_nounset" = 1 ] && set -u
unset _ament_nounset

# 避免默认/被 stereonet 改写到 /userdata/.roslog（常无写权限）
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/zettatree_roslog}"
mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true
