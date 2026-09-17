#!/usr/bin/env bash
# 当前终端立即进入 zettatree_demo 开发环境（需 source，不可直接 bash 执行后指望变量留存）。
# 顺序：TROS/ROS2 setup → 选择可用 RMW → ROS 日志目录 → 可选 mavros_ws overlay → 自检 ros2。
set +u
if [ -f /opt/tros/humble/setup.bash ]; then
  # 优先 TogetheROS（RDK 板端）
  source /opt/tros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "[env] 未找到 ROS2/TogetheROS Humble" >&2
  return 1 2>/dev/null || exit 1
fi
# 动态选择实际可用的 RMW；清除旧终端遗留的错误 RMW。
unset RMW_IMPLEMENTATION
if ldconfig -p 2>/dev/null | grep -q 'librmw_cyclonedds_cpp.so'; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
elif ldconfig -p 2>/dev/null | grep -q 'librmw_fastrtps_cpp.so'; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/zettatree_roslog}"
mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true
# 源码安装的 MAVROS overlay（apt 无 deb 时由 setup_mavros.sh 生成）
if [ -f /app/zettatree_demo/mavros_ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /app/zettatree_demo/mavros_ws/install/setup.bash
fi
if ros2 -h >/dev/null 2>&1; then
  echo "[env] READY: ROS2=${ROS_DISTRO:-humble}, RMW=${RMW_IMPLEMENTATION:-ROS default}"
else
  echo "[env] ERROR: ROS2 命令存在，但 RMW 动态库不可用。请先运行 00_env_check/run.sh --yes" >&2
  return 1 2>/dev/null || exit 1
fi
