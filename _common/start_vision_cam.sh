#!/usr/bin/env bash
# 视觉例程统一相机入口 → /camera/image_raw。
# 例程 03/04/05 显式 --source usb（普通 USB）；06/07 等可用 auto/mipi（GS130W 优先）。
#
#   bash start_vision_cam.sh --source usb --device /dev/video0 --show
#   bash start_vision_cam.sh --source mipi
#   bash start_vision_cam.sh                 # auto：MIPI 优先，失败再 USB
#
# 其余参数原样转给 camera_node.py / mipi_camera_bridge.py（如 --show）。
set -e
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh 2>/dev/null || true

COMMON="$(cd "$(dirname "$0")" && pwd)"
STEREO_DIR="$(cd "$COMMON/../08_depth_camera" && pwd)"
SOURCE="${CAMERA_SOURCE:-auto}"
DEVICE="${CAMERA_DEVICE:-/dev/video0}"
PASSTHRU=()

# 解析 --source / --device（含 launch 风格 camera_source:=）
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
  # USB V4L2 → camera_node.py → /camera/image_raw
  echo "[vision_cam] USB $DEVICE"
  exec python3 "$COMMON/camera_node.py" --device "$DEVICE" "${PASSTHRU[@]}"
}

_mipi() {
  # 确保 MIPI/BPU 就绪后，桥接左目到 /camera/image_raw
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
