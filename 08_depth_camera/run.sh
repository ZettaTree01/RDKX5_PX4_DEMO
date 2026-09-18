#!/usr/bin/env bash
# 例程8 统一入口（GS130W Stereonet）
#
#   bash run.sh                         # 默认：MIPI + Stereonet + OpenCV + RViz2
#   bash run.sh rviz:=false             # 关闭 RViz2
#   bash run.sh views                   # 分窗左右目 / 深彩 / 深度
#   bash run.sh views --no-depth --no-visual
#   bash run.sh views start_stereo:=0   # 只要左右目，不拉 Stereonet
#   bash run.sh rviz                    # 仅开 RViz2（需另终端已跑默认模式）
#   bash run.sh source:=simulate start_stereonet:=false
#
# 其余 key:=value 透传给 depth_camera.launch.py。
# Ctrl+C 停止本例程（MIPI / Stereonet / OpenCV / RViz2；不接飞控、不上锁）。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/run_flight.sh

# 例程8不接飞控：Ctrl+C 只停视觉栈
export FLIGHT_DISARM=0

MODE=full
ARGS=()
STEREO_OVERRIDE=""
for a in "$@"; do
  case "$a" in
    views|show_views|mode:=views|mode:=show_views)
      MODE=views
      ;;
    rviz|mode:=rviz|rviz_only)
      # 单独「仅 RViz2」；若写作 rviz:=false/true 则留给 launch
      if [[ "$a" == rviz:=* ]]; then
        ARGS+=("$a")
      else
        MODE=rviz
      fi
      ;;
    start_stereo:=0|start_stereo:=false|start_stereo:=False|START_STEREO=0)
      START_STEREO=0
      STEREO_OVERRIDE="start_stereonet:=false"
      ;;
    start_stereo:=1|start_stereo:=true|start_stereo:=True|START_STEREO=1)
      START_STEREO=1
      STEREO_OVERRIDE="start_stereonet:=true"
      ;;
    *)
      ARGS+=("$a")
      ;;
  esac
done

_prepare_log() {
  local d="${ROS_LOG_DIR:-/tmp/zettatree_roslog}"
  mkdir -p "$d" /tmp/zettatree_roslog 2>/dev/null || true
  chmod 1777 /tmp/zettatree_roslog 2>/dev/null || true
  if [ ! -w "$d" ] 2>/dev/null; then
    d="/tmp/zettatree_roslog"
    mkdir -p "$d" 2>/dev/null || true
  fi
  if [ ! -w "$d" ] 2>/dev/null; then
    d="/tmp/zettatree_roslog_${UID:-0}"
    mkdir -p "$d" 2>/dev/null || true
  fi
  export ROS_LOG_DIR="$d"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  rm -f /tmp/mipi_cam_gs130w.log 2>/dev/null || true
  if [ -f /tmp/zettatree_stereonet_params.yaml ] && [ ! -w /tmp/zettatree_stereonet_params.yaml ]; then
    rm -f /tmp/zettatree_stereonet_params.yaml 2>/dev/null || true
  fi
}

_setup_gl() {
  if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
    if [ ! -e /usr/lib/aarch64-linux-gnu/dri/vs-drm_dri.so ] \
      && [ ! -e /usr/lib/dri/vs-drm_dri.so ]; then
      export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
      export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
      export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
      export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
    fi
  fi
}

_warn_display() {
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "[08] 未设置 DISPLAY：OpenCV 可能无窗；RViz2 会自动挂到本机桌面 :0。"
  fi
}

_wait_combine() {
  for _ in 1 2 3 4 5 6 7 8; do
    if ros2 topic list 2>/dev/null | grep -qx '/image_combine_raw'; then
      return 0
    fi
    sleep 0.5
  done
}

_stop_stale_08() {
  echo "[08] 停止已有 MIPI / Stereonet / OpenCV / RViz2 ..."
  timeout 8 bash /app/zettatree_demo/_common/stop_nav_stack.sh \
    >/dev/null 2>&1 || true
}

run_full() {
  _prepare_log
  _setup_gl
  _stop_stale_08
  _warn_display
  echo "[08] 模式=full：双目 → Depth → OpenCV(深彩|3D POINT) + RViz2"
  echo "[08] Ctrl+C 停止 MIPI / Stereonet / OpenCV / RViz2（本例程不上锁）"
  bash "$SCRIPT_DIR/ensure_mipi_bpu.sh"
  _wait_combine
  # 前台可中断（_flight_run）；MIPI 由 ensure 拉起，退出时 stop_nav 回收
  _flight_run ros2 launch "$SCRIPT_DIR/depth_camera.launch.py" \
    source:=stereonet start_mipi:=false start_stereonet:=true \
    show:=true rviz:=true map:=false panel_mode:=depth_cloud \
    baseline_m:=0.07917 rotate_cw:=0 \
    min_range:=0.3 max_range:=5.0 snapshot:=none \
    ${STEREO_OVERRIDE:+"$STEREO_OVERRIDE"} \
    "${ARGS[@]}"
}

run_views() {
  _prepare_log
  _warn_display
  echo "[08] 模式=views：分窗 LEFT / RIGHT / VISUAL / DEPTH（q 或 Ctrl+C 退出）"
  bash "$SCRIPT_DIR/ensure_mipi_bpu.sh"
  if ! ros2 topic list 2>/dev/null | grep -qx '/StereoNetNode/stereonet_visual'; then
    if [ "${START_STEREO:-1}" = "1" ]; then
      echo "[08] 拉起 Stereonet…"
      nohup bash "$SCRIPT_DIR/start_stereonet.sh" >/tmp/stereonet_show_views.log 2>&1 &
      for i in $(seq 1 20); do
        sleep 1
        if ros2 topic list 2>/dev/null | grep -qx '/StereoNetNode/stereonet_visual'; then
          echo "[08] Stereonet ready (${i}s)"
          break
        fi
      done
    fi
  fi
  _flight_run python3 "$SCRIPT_DIR/show_stereo_views.py" "${ARGS[@]}"
}

run_rviz_only() {
  _prepare_log
  _setup_gl
  CFG="${SCRIPT_DIR}/depth_cloud.rviz"
  if [ ! -f "$CFG" ]; then
    echo "[08] missing $CFG" >&2
    exit 1
  fi
  echo "[08] 模式=rviz：Fixed Frame=camera_link，rviz2 -d depth_cloud.rviz"
  echo "[08] Topic=/StereoNetNode/stereonet_pointcloud2（请另开: bash .../run.sh）"
  # 仅关本窗口，不要 stop_nav 把另一终端的 MIPI/Stereonet 一起杀掉
  export FLIGHT_STOP_STACK=0
  _flight_run bash "$SCRIPT_DIR/../_common/rviz_run.sh" "$CFG"
}

case "$MODE" in
  views) run_views ;;
  rviz)  run_rviz_only ;;
  *)     run_full ;;
esac
