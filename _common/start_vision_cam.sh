#!/usr/bin/env bash
# 视觉例程统一相机入口：默认 GS130W MIPI（ISP）→ /camera/image_raw，失败再 USB。
# YOLO / 巡航 / 避障走 BPU 量化模型时，应尽量吃 MIPI NV12，而不是 USB MJPEG 解码。
#
#   bash start_vision_cam.sh                 # auto：MIPI 优先
#   bash start_vision_cam.sh --source mipi
#   bash start_vision_cam.sh --source usb --device /dev/video0 --show
set -e
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh 2>/dev/null || true

COMMON="$(cd "$(dirname "$0")" && pwd)"
STEREO_DIR="$(cd "$COMMON/../08_depth_camera" && pwd)"
SOURCE="${CAMERA_SOURCE:-auto}"
DEVICE="${CAMERA_DEVICE:-/dev/video0}"
PASSTHRU=()

while [ $# -gt 0 ]; do
  case "$1" in
    --source)
      SOURCE="$2"; shift 2 ;;
    --source=*)
      SOURCE="${1#*=}"; shift ;;
    --device)
      DEVICE="$2"; shift 2 ;;
    --device=*)
      DEVICE="${1#*=}"; shift ;;
    camera_source:=*)
      SOURCE="${1#camera_source:=}"; shift ;;
    camera_device:=*)
      DEVICE="${1#camera_device:=}"; shift ;;
    *)
      PASSTHRU+=("$1"); shift ;;
  esac
done
SOURCE="$(echo "$SOURCE" | tr 'A-Z' 'a-z')"

_usb() {
  echo "[vision_cam] USB $DEVICE"
  exec python3 "$COMMON/camera_node.py" --device "$DEVICE" "${PASSTHRU[@]}"
}

_mipi() {
  echo "[vision_cam] GS130W MIPI → /camera/image_raw（BPU 视觉前置）"
  bash "$STEREO_DIR/ensure_mipi_bpu.sh"
  exec python3 "$COMMON/mipi_camera_bridge.py" "${PASSTHRU[@]}"
}

case "$SOURCE" in
  usb|v4l2|video)
    _usb
    ;;
  mipi|gs130w|stereo)
    _mipi
    ;;
  auto|*)
    if bash "$STEREO_DIR/ensure_mipi_bpu.sh"; then
      echo "[vision_cam] auto：MIPI 可用，走 BPU 视觉链路"
      exec python3 "$COMMON/mipi_camera_bridge.py" "${PASSTHRU[@]}"
    fi
    echo "[vision_cam] auto：MIPI 不可用，回退 USB $DEVICE"
    _usb
    ;;
esac
