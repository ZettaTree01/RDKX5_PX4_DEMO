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
source /opt/tros/humble/setup.bash

W="${MIPI_IMAGE_WIDTH:-640}"
H="${MIPI_IMAGE_HEIGHT:-352}"
FPS="${MIPI_FRAMERATE:-15.0}"
CH0="${MIPI_CHANNEL:-2}"
CH1="${MIPI_CHANNEL2:-0}"
LPWM="${MIPI_LPWM_ENABLE:-True}"
ROT="${MIPI_ROTATION:-90.0}"
CALIB="${MIPI_CALIB_FILE:-/opt/tros/humble/lib/mipi_cam/config/SC132gs_dual_calibration.yaml}"
NEED_RESTART=0

_mipi_log_init() {
  # 优先可写目录；按 UID 分文件，避免 root 预跑后 sunrise 写不进日志（变成 /dev/null）
  local d="${ROS_LOG_DIR:-/tmp/zettatree_roslog}"
  mkdir -p "$d" 2>/dev/null || true
  chmod a+rwxt "$d" 2>/dev/null || true
  MIPI_LOG="${MIPI_LOG:-$d/mipi_cam_gs130w_${UID:-0}.log}"
  if ! ( : > "$MIPI_LOG" ) 2>/dev/null; then
    MIPI_LOG="/tmp/mipi_cam_gs130w_${UID:-0}.log"
    if ! ( : > "$MIPI_LOG" ) 2>/dev/null; then
      MIPI_LOG="/tmp/mipi_cam_gs130w_$$.log"
      : > "$MIPI_LOG" 2>/dev/null || MIPI_LOG="/dev/null"
    fi
  fi
}
_mipi_log_init

echo "[GS130W mipi] ${W}x${H} nv12 combine=2 ch=${CH0}/${CH1} lpwm=${LPWM} rot=${ROT}"
echo "[GS130W mipi] calib=${CALIB}"

_mipi_pids() {
  pgrep -f '/opt/tros/humble/lib/mipi_cam/mipi_cam' || true
}

_mipi_launch_alive() {
  [ -n "${MIPI_RUN_PID:-}" ] && kill -0 "$MIPI_RUN_PID" 2>/dev/null
}

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
  local p left
  for p in $(_mipi_pids); do
    kill -TERM "$p" 2>/dev/null || sudo -n kill -TERM "$p" 2>/dev/null || true
  done
  # 勿对含本脚本路径的命令行用 pkill -f（会误杀 ensure / pkill 自身）
  for _ in 1 2 3 4 5; do
    _mipi_pids | grep -q . || break
    sleep 1
  done
  for p in $(_mipi_pids); do
    kill -9 "$p" 2>/dev/null || sudo -n kill -9 "$p" 2>/dev/null || true
  done
  sleep 1
  left=$(_mipi_pids)
  if [ -n "$left" ]; then
    echo "[GS130W mipi] ERROR: 无法结束占用相机的进程(pid: $left)。" >&2
    echo "  若此前用 root 跑过例程，请先执行: sudo bash /app/zettatree_demo/_common/stop_nav_stack.sh" >&2
    echo "  或: sudo kill -9 $left" >&2
    exit 1
  fi
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
  MIPI_RUN_PID=$!
  # ros2 run 拉起二进制常需数秒；前几秒只等进程，不误报「已退出」
  for i in $(seq 1 30); do
    sleep 1
    if pgrep -f '/opt/tros/humble/lib/mipi_cam/mipi_cam' >/dev/null 2>&1 \
      || _mipi_launch_alive; then
      :
    else
      # 前 3 秒允许尚未出现二进制；之后视为启动失败
      if [ "$i" -le 3 ]; then
        echo "[GS130W mipi] 初始化 ${i}/30"
        continue
      fi
      echo "[GS130W mipi] 进程已退出，见 $MIPI_LOG" >&2
      if [ "$MIPI_LOG" = "/dev/null" ]; then
        echo "[GS130W mipi] 日志目录不可写；请: sudo chmod 1777 /tmp/zettatree_roslog" >&2
      else
        tail -n 80 "$MIPI_LOG" >&2 || true
      fi
      exit 1
    fi
    if [ "$i" -lt 6 ]; then
      echo "[GS130W mipi] 初始化 ${i}/30"
      continue
    fi
    if RATE=$(_mipi_has_frames); then
      echo "[GS130W mipi] ok ${RATE} (${i}s)"
      exit 0
    fi
    echo "[GS130W mipi] 等待画面 ${i}/30"
  done
  echo "[GS130W mipi] 警告: 未见帧，见 $MIPI_LOG" >&2
  tail -n 80 "$MIPI_LOG" >&2 || true
  exit 1
fi

echo "[GS130W mipi] 已满足官方 dual 参数"
exit 0
