#!/bin/bash
# 统一加载板端 ROS2 / TogetheROS 环境
if [ -f /opt/tros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "未找到 /opt/tros/humble 或 /opt/ros/humble，ROS2 例程无法运行" >&2
  return 1 2>/dev/null || exit 1
fi
