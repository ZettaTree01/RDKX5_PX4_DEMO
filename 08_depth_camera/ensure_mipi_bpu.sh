#!/usr/bin/env bash
# RDK Stereo Camera GS130W（双 SC132GS）→ 按官方参数启动 mipi_cam dual
# 参考：
#   - https://github.com/D-Robotics/hobot_mipi_cam
#   - mipi_cam_dual_channel / 132gs launch
#   - Yahboom/D-Robotics GS130W 双目深度教程
# 官方推荐：640x352、lpwm、channel=2/0、rotation=90、dual_combine=2
#
# 注意：不要用 `ros2 topic echo` 整包 Image 判活（NV12 大图易超时误报「未见帧」）。
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

# /tmp 有 sticky bit：别人留下的同名日志即使用户是 root，O_CREAT 也可能 Permission denied。
_mipi_log_init() {
  local d="${ROS_LOG_DIR:-/tmp/zettatree_roslog}"
  mkdir -p "$d" 2>/dev/null || d="/tmp"
  MIPI_LOG="${MIPI_LOG:-$d/mipi_cam_gs130w.log}"
  if ! ( : > "$MIPI_LOG" ) 2>/dev/null; then
    rm -f "$MIPI_LOG" 2>/dev/null || true
    MIPI_LOG="$d/mipi_cam_gs130w_${UID:-0}.log"
    if ! ( : > "$MIPI_LOG" ) 2>/dev/null; then
      MIPI_LOG="/dev/null"
    fi
  fi
}
_mipi_log_init

echo "[GS130W mipi] ${W}x${H} nv12 combine=2 ch=${CH0}/${CH1} lpwm=${LPWM} rot=${ROT}"
echo "[GS130W mipi] calib=${CALIB}"

_mipi_pids() {
  pgrep -f '/opt/tros/humble/lib/mipi_cam/mipi_cam' || true
}

# 跨 UID 的 FastDDS SHM 经常“有 Publisher、无数据”。MIPI 必须是当前用户的。
_mipi_owner_ok() {
  local pid ou
  pid=$(_mipi_pids | head -1)
  [ -n "$pid" ] || return 1
  ou=$(stat -c %u "/proc/$pid" 2>/dev/null || echo "")
  [ "$ou" = "${UID}" ] || [ "$ou" = "$(id -u)" ]
}

# 返回 0=有帧；打印 average rate 到 stdout
_mipi_has_frames() {
  local hz
  hz=$(timeout 8 ros2 topic hz /image_combine_raw --window 3 2>&1 || true)
  if echo "$hz" | grep -Eq 'average rate:[[:space:]]*[1-9]'; then
    echo "$hz" | grep -E 'average rate:' | head -1
    return 0
  fi
  return 1
}

_mipi_stop() {
  # 只杀 mipi 二进制；跨用户残留用 sudo -n（无密码 sudo 时才能清掉 root 调试进程）
  for p in $(_mipi_pids); do
    kill -TERM "$p" 2>/dev/null || sudo -n kill -TERM "$p" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5; do
    _mipi_pids | grep -q . || break
    sleep 1
  done
  for p in $(_mipi_pids); do
    kill -9 "$p" 2>/dev/null || sudo -n kill -9 "$p" 2>/dev/null || true
  done
  sleep 1
}

if ! _mipi_owner_ok; then
  echo "[GS130W mipi] 无本用户 mipi_cam（或属主不是 UID=${UID}），将按当前用户拉起"
  NEED_RESTART=1
elif ! ros2 topic list 2>/dev/null | grep -qx '/image_combine_raw'; then
  echo "[GS130W mipi] /image_combine_raw 无话题"
  NEED_RESTART=1
else
  if RATE=$(_mipi_has_frames); then
    echo "[GS130W mipi] live: $RATE"
    CMD=$(tr '\0' ' ' < /proc/"$(_mipi_pids | head -1)"/cmdline 2>/dev/null || true)
    if [ -n "$CMD" ] && ! echo "$CMD" | grep -qE -- "-p channel:=${CH0}( |$)"; then
      echo "[GS130W mipi] 通道不是官方 ${CH0}/${CH1}，重启"
      NEED_RESTART=1
    fi
  else
    echo "[GS130W mipi] 话题在但本用户未见稳定帧，重启"
    NEED_RESTART=1
  fi
fi

if [ "$NEED_RESTART" = "1" ]; then
  echo "[GS130W mipi] 重启 hobot_mipi_cam dual ..."
  _mipi_stop
  CALIB_ARGS=()
  if [ -f "$CALIB" ]; then
    CALIB_ARGS=(-p "camera_calibration_file_path:=${CALIB}")
  fi
  : >"$MIPI_LOG" 2>/dev/null || true
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
    >"$MIPI_LOG" 2>&1 &
  for i in $(seq 1 30); do
    sleep 1
    if ! pgrep -f '/opt/tros/humble/lib/mipi_cam/mipi_cam' >/dev/null 2>&1; then
      echo "[GS130W mipi] 进程已退出，见 $MIPI_LOG" >&2
      tail -n 50 "$MIPI_LOG" >&2 || true
      exit 1
    fi
    # 前几秒只等 init
    if [ "$i" -lt 5 ]; then
      echo "[GS130W mipi] waiting init... ${i}/30"
      continue
    fi
    if RATE=$(_mipi_has_frames); then
      echo "[GS130W mipi] ok ${RATE} (${i}s)"
      exit 0
    fi
    echo "[GS130W mipi] waiting frames... ${i}/30"
  done
  echo "[GS130W mipi] 警告: 未见帧，见 $MIPI_LOG" >&2
  tail -n 50 "$MIPI_LOG" >&2 || true
  # 传感器仍在线时给出提示；Aborted 多为杀进程时的已知析构问题
  if grep -q 'terminate called without an active exception' "$MIPI_LOG" 2>/dev/null; then
    echo "[GS130W mipi] 提示: Aborted 多为 mipi_cam 退出析构问题；可再跑一次本脚本" >&2
  fi
  exit 1
fi

echo "[GS130W mipi] 已满足官方 dual 参数"
exit 0
