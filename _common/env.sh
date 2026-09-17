#!/usr/bin/env bash
# 板端唯一 ROS 环境：地平线 TogetheROS Humble（/opt/tros/humble）。
# TROS setup 会 overlay 其依赖的 /opt/ros/humble，不要再单独 source 原生 ROS2。
# TROS/ament setup.bash 会读未定义变量；若调用方开了 set -u，先临时关闭。
_ament_nounset=0
case "$-" in *u*) _ament_nounset=1; set +u ;; esac

if [ -f /opt/tros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
else
  [ "$_ament_nounset" = 1 ] && set -u
  echo "未找到地平线 TogetheROS Humble：/opt/tros/humble/setup.bash" >&2
  echo "本例程只支持 TROS，不把单独的 /opt/ros/humble 当作一套 ROS2。" >&2
  unset _ament_nounset
  return 1 2>/dev/null || exit 1
fi

_ZT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd 2>/dev/null || true)"
_ZT_ROOT="${_ZT_ROOT:-/app/zettatree_demo}"

# 源码 overlay：apt 无 mavros 节点时由 00_env_check/setup_mavros.sh 生成
if [ -f "$_ZT_ROOT/mavros_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$_ZT_ROOT/mavros_ws/install/setup.bash"
fi
# 09/10 共用 C++ EGO-Planner
if [ -f "$_ZT_ROOT/09_depth_nav/ego_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$_ZT_ROOT/09_depth_nav/ego_ws/install/setup.bash"
fi

[ "$_ament_nounset" = 1 ] && set -u
unset _ament_nounset _ZT_ROOT

# 避免默认/被 stereonet 改写到 /userdata/.roslog（常无写权限）
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/zettatree_roslog}"
mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true

# 有显示且无 vs-drm 时使用软件 OpenGL
if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
  if [ ! -e /usr/lib/aarch64-linux-gnu/dri/vs-drm_dri.so ] \
    && [ ! -e /usr/lib/dri/vs-drm_dri.so ]; then
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
    export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
    export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
  fi
fi

# 动态选择实际存在的 RMW：先查 TROS/ROS 安装目录（ldconfig 常常扫不到 /opt）。
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  _zt_rmw=""
  for _r in rmw_cyclonedds_cpp rmw_fastrtps_cpp; do
    for _d in \
      /opt/tros/humble/lib \
      /opt/ros/humble/lib \
      /opt/ros/humble/lib/aarch64-linux-gnu \
      /usr/lib \
      /usr/lib/aarch64-linux-gnu
    do
      if [ -f "$_d/lib${_r}.so" ]; then
        _zt_rmw="$_r"
        break 2
      fi
    done
    if ldconfig -p 2>/dev/null | grep -q "lib${_r}.so"; then
      _zt_rmw="$_r"
      break
    fi
  done
  if [ -n "$_zt_rmw" ]; then
    export RMW_IMPLEMENTATION="$_zt_rmw"
  fi
  unset _zt_rmw _r _d
fi
