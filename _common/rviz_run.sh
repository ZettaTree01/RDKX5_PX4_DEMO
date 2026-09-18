#!/usr/bin/env bash
# 例程 8/9/10 共用：启动 rviz2。
#
# SSH 启动时通常没有 DISPLAY，但板端桌面在 :0（用户 sunrise）。
# 自动挂到本机 X11，并以桌面用户身份运行，避免 root 连 :0 时 Qt 段错误。
# 本机确实没有图形会话时退出 0，避免 launch 失败。
#
# 用法：bash rviz_run.sh /path/to/config.rviz
set -e
CFG="${1:-}"
if [ -z "$CFG" ] || [ ! -f "$CFG" ]; then
  echo "[rviz] 缺少配置文件: ${CFG:-<empty>}" >&2
  exit 1
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  if [ -S /tmp/.X11-unix/X0 ]; then
    export DISPLAY=:0
  elif [ -S /tmp/.X11-unix/X1 ]; then
    export DISPLAY=:1
  fi
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[rviz] 无本机图形会话，跳过。同网 PC 可："
  echo "  source /opt/ros/humble/setup.bash   # 或板端 TROS"
  echo "  export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}"
  echo "  rviz2 -d $CFG"
  exit 0
fi

_desktop_user=""
if getent passwd sunrise >/dev/null 2>&1; then
  _desktop_user=sunrise
fi

if [ -n "$_desktop_user" ] && [ -r "/home/${_desktop_user}/.Xauthority" ]; then
  export XAUTHORITY="/home/${_desktop_user}/.Xauthority"
fi

# root 连别人的 X 会话会 Qt SEGV；改以桌面用户启动，并显式传递显示环境
if [ "$(id -u)" = 0 ] && [ -n "$_desktop_user" ]; then
  _uid="$(id -u "$_desktop_user")"
  _self="$(readlink -f "$0")"
  _home="/home/${_desktop_user}"
  _xdg="/run/user/${_uid}"
  echo "[rviz] DISPLAY=$DISPLAY，以 ${_desktop_user} 启动 $CFG"
  _run_env=(
    env
    "DISPLAY=${DISPLAY}"
    "XAUTHORITY=${XAUTHORITY:-${_home}/.Xauthority}"
    "HOME=${_home}"
    "USER=${_desktop_user}"
    "LOGNAME=${_desktop_user}"
    "XDG_RUNTIME_DIR=${_xdg}"
    "DBUS_SESSION_BUS_ADDRESS=unix:path=${_xdg}/bus"
    "LC_ALL=C"
    "QT_X11_NO_MITSHM=1"
    "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}"
    "GALLIUM_DRIVER=${GALLIUM_DRIVER:-llvmpipe}"
    "MESA_LOADER_DRIVER_OVERRIDE=${MESA_LOADER_DRIVER_OVERRIDE:-swrast}"
    "ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/zettatree_roslog}"
    "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  )
  if [ -n "${RMW_IMPLEMENTATION:-}" ]; then
    _run_env+=("RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}")
  fi
  if command -v runuser >/dev/null 2>&1; then
    exec runuser -u "$_desktop_user" -- "${_run_env[@]}" /bin/bash "$_self" "$CFG"
  fi
  exec sudo -u "$_desktop_user" -- "${_run_env[@]}" /bin/bash "$_self" "$CFG"
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash 2>/dev/null || true
  # shellcheck disable=SC1091
  source /app/zettatree_demo/_common/env.sh 2>/dev/null || true
fi

export LC_ALL="${LC_ALL:-C}"
export QT_X11_NO_MITSHM=1
if [ -n "${XDG_RUNTIME_DIR:-}" ] && [ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]; then
  export DBUS_SESSION_BUS_ADDRESS="unix:path=${XDG_RUNTIME_DIR}/bus"
fi

# 板端 Xorg 声明 DRI 驱动 vs-drm，但系统没有 vs-drm_dri.so（已回退 swrast）。
# 客户端同样走软件 OpenGL，避免 Mesa 去 dlopen 不存在的 vs-drm。
if [ ! -e /usr/lib/aarch64-linux-gnu/dri/vs-drm_dri.so ] \
  && [ ! -e /usr/lib/dri/vs-drm_dri.so ]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
  export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-swrast}"
fi

echo "[rviz] rviz2 -d $CFG  DISPLAY=$DISPLAY user=$(id -un)"
set +e
rviz2 -d "$CFG"
_rc=$?
set -e
if [ "$_rc" -ne 0 ]; then
  echo "[rviz] rviz2 退出码 ${_rc}（板端无 vs-drm 时 Ogre 可能段错误）。同网 PC："
  echo "  export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}"
  echo "  rviz2 -d $CFG"
fi
exit "$_rc"
