#!/usr/bin/env bash
# RDK Stereo Camera GS130W（双 SC132GS）→ 按官方参数启动 mipi_cam dual
# 参考：
#   - https://github.com/D-Robotics/hobot_mipi_cam
#   - mipi_cam_dual_channel / 132gs launch
#   - Yahboom/D-Robotics GS130W 双目深度教程
# 官方推荐：640x352、lpwm、channel=2/0、rotation=90、dual_combine=2
set -e
# shellcheck disable=SC1091
source /opt/tros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

W="${MIPI_IMAGE_WIDTH:-640}"
H="${MIPI_IMAGE_HEIGHT:-352}"
FPS="${MIPI_FRAMERATE:-15.0}"
CH0="${MIPI_CHANNEL:-2}"
CH1="${MIPI_CHANNEL2:-0}"
LPWM="${MIPI_LPWM_ENABLE:-True}"
ROT="${MIPI_ROTATION:-90.0}"
CALIB="${MIPI_CALIB_FILE:-/opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml}"
NEED_RESTART=0

echo "[GS130W mipi] ${W}x${H} nv12 combine=2 ch=${CH0}/${CH1} lpwm=${LPWM} rot=${ROT}"
echo "[GS130W mipi] calib=${CALIB}"

if ! timeout 2 ros2 topic info /image_combine_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then
  echo "[GS130W mipi] /image_combine_raw 无发布者"
  NEED_RESTART=1
else
  META=$(timeout 3 ros2 topic echo /image_combine_raw --once 2>/dev/null || true)
  CW=$(echo "$META" | awk '/^width:/{print $2; exit}')
  CH=$(echo "$META" | awk '/^height:/{print $2; exit}')
  echo "[GS130W mipi] current combine ${CW}x${CH}"
  if [ -z "$CW" ] || [ -z "$CH" ]; then
    NEED_RESTART=1
  elif [ "$CW" != "$W" ] || [ "$CH" != "$((H * 2))" ]; then
    echo "[GS130W mipi] 尺寸不是官方 ${W}x$((H*2))，重启"
    NEED_RESTART=1
  else
    CMD=$(tr '\0' ' ' < /proc/"$(pgrep -f '/opt/tros/humble/lib/mipi_cam/mipi_cam' | head -1)"/cmdline 2>/dev/null || true)
    if ! echo "$CMD" | grep -qE -- "-p channel:=${CH0}( |$)"; then
      echo "[GS130W mipi] 通道不是官方 ${CH0}/${CH1}，重启"
      NEED_RESTART=1
    fi
  fi
fi

if [ "$NEED_RESTART" = "1" ]; then
  echo "[GS130W mipi] 重启 hobot_mipi_cam dual ..."
  pkill -f "/opt/tros/humble/lib/mipi_cam/mipi_cam" 2>/dev/null || true
  pkill -f "ros2 run mipi_cam mipi_cam" 2>/dev/null || true
  sleep 2
  CALIB_ARGS=()
  if [ -f "$CALIB" ]; then
    CALIB_ARGS=(-p "camera_calibration_file_path:=${CALIB}")
  fi
  nohup ros2 run mipi_cam mipi_cam --ros-args \
    -p device_mode:=dual \
    -p out_format:=nv12 \
    -p dual_combine:=2 \
    -p image_width:="$W" \
    -p image_height:="$H" \
    -p framerate:="$FPS" \
    -p channel:="$CH0" \
    -p channel2:="$CH1" \
    -p lpwm_enable:="$LPWM" \
    -p rotation:="$ROT" \
    -p gdc_enable:=True \
    -p frame_id:=camera_link \
    "${CALIB_ARGS[@]}" \
    --log-level warn \
    >/tmp/mipi_cam_gs130w.log 2>&1 &
  for i in $(seq 1 25); do
    sleep 1
    META=$(timeout 3 ros2 topic echo /image_combine_raw --once 2>/dev/null || true)
    CW=$(echo "$META" | awk '/^width:/{print $2; exit}')
    CHh=$(echo "$META" | awk '/^height:/{print $2; exit}')
    if [ -n "$CW" ] && [ -n "$CHh" ]; then
      echo "[GS130W mipi] ok combine ${CW}x${CHh} (${i}s)"
      exit 0
    fi
    echo "[GS130W mipi] waiting frames... ${i}/25"
  done
  echo "[GS130W mipi] 警告: 未见帧，见 /tmp/mipi_cam_gs130w.log" >&2
  tail -n 50 /tmp/mipi_cam_gs130w.log >&2 || true
  exit 1
fi

echo "[GS130W mipi] 已满足官方 dual 参数"
exit 0
