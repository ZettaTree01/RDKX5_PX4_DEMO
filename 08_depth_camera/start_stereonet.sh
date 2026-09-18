#!/usr/bin/env bash
# GS130W + hobot_stereonet（BPU DStereo V2.4 int16）
#
# 参数名必须与本机 TROS 节点声明的一致（用 ros2 param list /StereoNetNode 核对）：
#   base_line（不是 baseline）、postprocess（不是 post_version）、
#   pc_max_depth / height_min / height_max（不是 pointcloud_*）。
# 写错名字不会报错，只会静默用 C++ 默认值：base_line=0.1、postprocess=v1，
# 视差解码版本对不上，深度会整体缩小上千倍、点云挤成几厘米。
#
# need_rectify 固定 false：GS130W mipi dual 出图已由 GDC 按
# SC132gs_dual_calibration.yaml 校正；节点自带的 stereo.yaml 是 1280x640 的
# 另一款相机，拿它再校正一次会把画面 70% 推出视野变黑。
# CameraInfo 由 pub_stereo_caminfo.py 发布。
set -e
# shellcheck disable=SC1091
source /opt/tros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

_pick_dir() {
  local d
  for d in "$@"; do
    [ -n "$d" ] || continue
    mkdir -p "$d" 2>/dev/null || continue
    [ -w "$d" ] || continue
    printf '%s' "$d"
    return 0
  done
  return 1
}

if [ -z "${ROS_LOG_DIR:-}" ] || [ ! -w "${ROS_LOG_DIR}" ] 2>/dev/null; then
  ROS_LOG_DIR="$(_pick_dir /userdata/.roslog /tmp/zettatree_roslog \
    "/tmp/zettatree_roslog_${UID:-0}")" || ROS_LOG_DIR="/tmp/zettatree_roslog_${UID:-0}"
  mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true
  export ROS_LOG_DIR
fi

MODEL="${STEREO_MODEL:-}"
if [ -z "$MODEL" ]; then
  for p in \
    /opt/tros/humble/share/hobot_stereonet/config/DStereoV2.4_int16.bin \
    /opt/tros/humble/share/hobot_stereonet/model/DStereoV2.4_int16.bin \
    /opt/tros/humble/share/hobot_stereonet/model/x5/DStereoV2.4_int16.bin
  do
    if [ -f "$p" ]; then MODEL="$p"; break; fi
  done
fi
if [ -z "$MODEL" ] || [ ! -f "$MODEL" ]; then
  echo "[GS130W stereonet] ERROR: 找不到 DStereoV2.4_int16.bin" >&2
  echo "  请安装 tros-humble-hobot-stereonet 或设置 STEREO_MODEL=绝对路径" >&2
  exit 1
fi

FX="${CAMERA_FX:-328.379}"
FY="${CAMERA_FY:-328.379}"
CX="${CAMERA_CX:-320.0}"
CY="${CAMERA_CY:-176.0}"
BASELINE_M="${BASELINE_M:-0.07917}"
# 1=上下拼接（mipi dual_combine=2 → 640x704）；0=左右拼接
COMBINE_MODE="${STEREO_COMBINE_MODE:-1}"
RIGHT_INFO="${CAMERA_INFO_TOPIC:-/drone/stereo/right/camera_info}"
LEFT_INFO="${LEFT_CAMERA_INFO_TOPIC:-/drone/stereo/left/camera_info}"

# 仅当用户强制指定时才改；默认始终用 pub_stereo_caminfo 的话题
if [ -n "${CAMERA_INFO_TOPIC:-}" ]; then
  RIGHT_INFO="$CAMERA_INFO_TOPIC"
fi
if [ -n "${LEFT_CAMERA_INFO_TOPIC:-}" ]; then
  LEFT_INFO="$LEFT_CAMERA_INFO_TOPIC"
fi

# 节点会读相对路径 ./config/stereo.yaml
if [ -z "${STEREO_RUN_DIR:-}" ]; then
  STEREO_RUN_DIR="$(_pick_dir /userdata/stereonet_run \
    "${ROS_LOG_DIR}/stereonet_run" "/tmp/stereonet_run_${UID:-0}" \
    "${HOME:-/tmp}/.stereonet_run")" || STEREO_RUN_DIR="/tmp/stereonet_run_${UID:-0}"
fi
RUN_DIR="$STEREO_RUN_DIR"
mkdir -p "$RUN_DIR/config"
CALIB_ABS="/opt/tros/humble/share/hobot_stereonet/config/stereo.yaml"
if [ -f "$CALIB_ABS" ]; then
  cp -n "$CALIB_ABS" "$RUN_DIR/config/stereo.yaml" 2>/dev/null || \
    cp -f "$CALIB_ABS" "$RUN_DIR/config/stereo.yaml" 2>/dev/null || true
fi
cd "$RUN_DIR"

echo "[GS130W stereonet] check /image_combine_raw …"
ok_img=0
for _ in 1 2 3 4 5 6 7 8; do
  if ros2 topic list 2>/dev/null | grep -qx '/image_combine_raw'; then
    hz=$(timeout 5 ros2 topic hz /image_combine_raw --window 3 2>&1 || true)
    if echo "$hz" | grep -Eq 'average rate:[[:space:]]*[1-9]'; then
      ok_img=1
      echo "[GS130W stereonet] combine live: $(echo "$hz" | grep -E 'average rate:' | head -1)"
      break
    fi
    ok_img=1
    break
  fi
  sleep 0.5
done
if [ "$ok_img" != "1" ]; then
  echo "[GS130W stereonet] WARN: 尚无 /image_combine_raw，仍启动（请先 ensure_mipi_bpu.sh）" >&2
fi

# 确保 CameraInfo 有人发（launch 通常已拉起；单独跑本脚本时补上）
if ! pgrep -f 'pub_stereo_caminfo.py' >/dev/null 2>&1; then
  echo "[GS130W stereonet] start pub_stereo_caminfo → ${LEFT_INFO} / ${RIGHT_INFO}"
  nohup python3 "$SCRIPT_DIR/pub_stereo_caminfo.py" \
    --width 640 --height 352 \
    --fx "$FX" --fy "$FY" --cx "$CX" --cy "$CY" \
    --baseline "$BASELINE_M" \
    --left-topic "$LEFT_INFO" --right-topic "$RIGHT_INFO" \
    --rate 15.0 \
    >/tmp/stereo_caminfo.log 2>&1 &
  sleep 1
fi

# 本机 TROS（X5）将 render_type 声明为 integer：0=indoor，1=outdoor。
# 写成字符串 "indoor" 会直接 InvalidParameterTypeException 退出。
RENDER_TYPE="${RENDER_TYPE:-0}"
case "${RENDER_TYPE}" in
  indoor|Indoor|0) RENDER_YAML=0 ;;
  outdoor|Outdoor|1) RENDER_YAML=1 ;;
  distance|2) RENDER_YAML=2 ;;
  *) RENDER_YAML=0 ;;
esac
RENDER_PERF="${RENDER_PERF:-true}"

# 视差后处理版本必须与模型匹配，官方 launch 的对应关系：
#   DStereoV2.0=v2  V2.1=v2.1  V2.2=v2.2  V2.3(.1)=v2.3
#   V2.4_int16 / V2.4_int8=v2.3   V2.4_int16_320_256=v2.1
POSTPROCESS="${STEREO_POSTPROCESS:-}"
if [ -z "$POSTPROCESS" ]; then
  case "$(basename "$MODEL")" in
    DStereoV2.4_int16_320_256.bin) POSTPROCESS=v2.1 ;;
    DStereoV2.4_int16.bin|DStereoV2.4_int8.bin) POSTPROCESS=v2.3 ;;
    DStereoV2.3.bin|DStereoV2.3.1.bin) POSTPROCESS=v2.3 ;;
    DStereoV2.2.bin) POSTPROCESS=v2.2 ;;
    DStereoV2.1.bin) POSTPROCESS=v2.1 ;;
    *) POSTPROCESS=v2 ;;
  esac
fi

PARAMS="${STEREO_PARAMS_FILE:-}"
if [ -z "$PARAMS" ]; then
  PARAMS="${RUN_DIR}/stereonet_params.yaml"
fi
PARAMS_DIR="$(dirname "$PARAMS")"
if ! mkdir -p "$PARAMS_DIR" 2>/dev/null || ! (touch "$PARAMS" 2>/dev/null); then
  PARAMS="${ROS_LOG_DIR}/stereonet_params_${UID}.yaml"
  mkdir -p "$(dirname "$PARAMS")" 2>/dev/null || true
  touch "$PARAMS" 2>/dev/null || {
    echo "[GS130W stereonet] ERROR: 无法创建参数文件: $PARAMS" >&2
    exit 1
  }
fi

# 仅写入本机 TROS 已声明的参数名
cat > "$PARAMS" <<EOF
/**:
  ros__parameters:
    stereonet_model_file_path: "${MODEL}"
    stereo_image_topic: "/image_combine_raw"
    camera_info_topic: "${RIGHT_INFO}"
    stereo_calib_file_path: "${CALIB_ABS}"
    stereo_combine_mode: ${COMBINE_MODE}
    need_rectify: false
    load_rectify_param: false
    camera_fx: ${FX}
    camera_fy: ${FY}
    camera_cx: ${CX}
    camera_cy: ${CY}
    base_line: ${BASELINE_M}
    postprocess: "${POSTPROCESS}"
    max_disp: 192
    render_type: ${RENDER_YAML}
    render_perf: ${RENDER_PERF}
    render_need_filter: true
    render_max_depth: 10000
    visual_alpha: 3
    visual_beta: 0
    uncertainty_th: -0.09
    depth_need_filter: true
    pc_max_depth: 5.0
    height_min: -10.0
    height_max: 10.0
    # need_pcl_filter 关着时 leaf_size/stdv/KMean 的 C++ 默认值是未初始化脏值，
    # 显式写入官方默认，避免以后打开滤波踩坑
    need_pcl_filter: false
    leaf_size: 0.05
    stdv: 0.01
    KMean: 10
    use_usb_camera: false
    use_local_image: false
EOF

echo "[GS130W stereonet] model=$MODEL cwd=$RUN_DIR"
echo "[GS130W stereonet] fx=$FX fy=$FY cx=$CX cy=$CY baseline=${BASELINE_M}m combine_mode=$COMBINE_MODE"
echo "[GS130W stereonet] camera_info=$RIGHT_INFO need_rectify=false postprocess=$POSTPROCESS"
echo "[GS130W stereonet] render_type=$RENDER_YAML"
echo "[GS130W stereonet] params=$PARAMS"

extra=()
for a in "$@"; do
  case "$a" in
    *=*) extra+=(--ros-args -p "$a") ;;
    *) extra+=("$a") ;;
  esac
done

exec ros2 run hobot_stereonet stereonet_model_node --ros-args \
  --log-level warn \
  -r __node:=StereoNetNode \
  --params-file "$PARAMS" \
  "${extra[@]}"
