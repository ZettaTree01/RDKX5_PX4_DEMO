#!/usr/bin/env bash
# 有 DISPLAY 时启动 rviz2；无 vs-drm 时使用软件 OpenGL。
set -e
CFG="${1:-}"
if [ -z "$CFG" ] || [ ! -f "$CFG" ]; then
  echo "[rviz] 缺少配置文件: ${CFG:-<empty>}" >&2
  exit 1
fi
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "[rviz] 无 DISPLAY，跳过"
  exit 0
fi
if [ ! -e /usr/lib/aarch64-linux-gnu/dri/vs-drm_dri.so ] \
  && [ ! -e /usr/lib/dri/vs-drm_dri.so ]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
fi
exec rviz2 -d "$CFG"
