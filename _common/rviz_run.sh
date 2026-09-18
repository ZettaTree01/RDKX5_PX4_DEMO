#!/usr/bin/env bash
# 例程 8/9/10 共用：有 DISPLAY 时启动 rviz2（与例程原来的实现一致）。
#
# SSH 通常没有 DISPLAY，板端桌面在 :0（用户 sunrise）。
# 自动挂到本机 X11，并以桌面用户运行，避免 root 连 :0 时 Qt 段错误。
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
  echo "  source /opt/tros/humble/setup.bash"
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

if [ "$(id -u)" = 0 ] && [ -n "$_desktop_user" ]; then
  _uid="$(id -u "$_desktop_user")"
  _self="$(readlink -f "$0")"
  _home="/home/${_desktop_user}"
  _xdg="/run/user/${_uid}"
  echo "[rviz] DISPLAY=$DISPLAY，以 ${_desktop_user} 启动 rviz2 -d $CFG"
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
    "QT_OPENGL=software"
    "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}"
    "GALLIUM_DRIVER=${GALLIUM_DRIVER:-llvmpipe}"
    "MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE:-3.3}"
    "MESA_GLSL_VERSION_OVERRIDE=${MESA_GLSL_VERSION_OVERRIDE:-330}"
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

_here="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${_here}/env.sh" 2>/dev/null || true
if ! command -v rviz2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash 2>/dev/null || true
fi

export LC_ALL="${LC_ALL:-C}"
export QT_X11_NO_MITSHM=1
export QT_OPENGL="${QT_OPENGL:-software}"
if [ -n "${XDG_RUNTIME_DIR:-}" ] && [ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]; then
  export DBUS_SESSION_BUS_ADDRESS="unix:path=${XDG_RUNTIME_DIR}/bus"
fi

# 例程原来的软件 OpenGL 回退（无 vs-drm 时）
if [ ! -e /usr/lib/aarch64-linux-gnu/dri/vs-drm_dri.so ] \
  && [ ! -e /usr/lib/dri/vs-drm_dri.so ]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
fi

if ! command -v rviz2 >/dev/null 2>&1; then
  echo "[rviz] 未找到 rviz2。同网 PC："
  echo "  rviz2 -d $CFG"
  exit 0
fi

echo "[rviz] rviz2 -d $CFG  DISPLAY=$DISPLAY user=$(id -un)"
# exec：launch / Ctrl+C 进程组信号直接打到 rviz2
exec rviz2 -d "$CFG"
