#!/usr/bin/env bash
# GS130W + hobot_stereonet（BPU DStereo V2.4 int16）
# DStereoV2.4：need_rectify=false，postprocess=v2.3，uncertainty_th=-0.09
# 参数名 base_line；CameraInfo 由 pub_stereo_caminfo.py 发布。
set -e
# shellcheck disable=SC1091
source /opt/tros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ ! -w /userdata/.roslog ] 2>/dev/null; then
  sudo -n mkdir -p /userdata/.roslog >/dev/null 2>&1 || true
  sudo -n chown -R "$(id -un):$(id -gn)" /userdata/.roslog >/dev/null 2>&1 || true
  sudo -n chmod 777 /userdata/.roslog >/dev/null 2>&1 || true
fi
export ROS_LOG_DIR=/userdata/.roslog
mkdir -p "$ROS_LOG_DIR"

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
RUN_DIR="${STEREO_RUN_DIR:-/userdata/stereonet_run}"
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

RENDER_TYPE="${RENDER_TYPE:-0}"
PC_STEP="${POINTCLOUD_DOWNSAMPLE_STEP:-2}"
RENDER_PERF="${RENDER_PERF:-true}"

PARAMS="${STEREO_PARAMS_FILE:-/tmp/zettatree_stereonet_params_${UID}.yaml}"
PARAMS_DIR="$(dirname "$PARAMS")"
if ! mkdir -p "$PARAMS_DIR" 2>/dev/null || ! (touch "$PARAMS" 2>/dev/null); then
  PARAMS="/tmp/zettatree_stereonet_params_${UID}.yaml"
  touch "$PARAMS" 2>/dev/null || {
    echo "[GS130W stereonet] ERROR: 无法创建参数文件: $PARAMS" >&2
    exit 1
  }
fi
if [ "${RENDER_TYPE_IS_STRING:-0}" = "1" ]; then
  RENDER_YAML="\"${RENDER_TYPE}\""
else
  case "$RENDER_TYPE" in
    ''|*[!0-9]*) RENDER_YAML="0" ;;
    *) RENDER_YAML="$RENDER_TYPE" ;;
  esac
fi

# DStereoV2.4 参数
cat > "$PARAMS" <<EOF
/**:
  ros__parameters:
    stereonet_model_file_path: "${MODEL}"
    stereo_image_topic: "/image_combine_raw"
    stereo_combine_mode: ${COMBINE_MODE}
    camera_info_topic: "${RIGHT_INFO}"
    need_rectify: false
    load_rectify_param: false
    stereo_calib_file_path: "${CALIB_ABS}"
    camera_fx: ${FX}
    camera_fy: ${FY}
    camera_cx: ${CX}
    camera_cy: ${CY}
    base_line: ${BASELINE_M}
    postprocess: "v2.3"
    render_type: ${RENDER_YAML}
    render_perf: ${RENDER_PERF}
    uncertainty_th: -0.09
    depth_need_filter: true
    pc_max_depth: 5.0
    height_min: -10.0
    height_max: 10.0
    leaf_size: 0.05
    pointcloud_downsample_step: ${PC_STEP}
EOF

echo "[GS130W stereonet] model=$MODEL cwd=$RUN_DIR"
echo "[GS130W stereonet] fx=$FX fy=$FY cx=$CX cy=$CY base_line=${BASELINE_M}m combine_mode=$COMBINE_MODE"
echo "[GS130W stereonet] camera_info=$RIGHT_INFO need_rectify=false postprocess=v2.3 uncertainty_th=-0.09"
echo "[GS130W stereonet] render_type=$RENDER_YAML leaf_size/downsample~ step env=$PC_STEP"
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
