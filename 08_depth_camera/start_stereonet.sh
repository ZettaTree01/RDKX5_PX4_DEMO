#!/usr/bin/env bash
# GS130W + hobot_stereonet（BPU DStereo V2.4 int16）
# 深彩：/StereoNetNode/stereonet_visual
# 点云：/StereoNetNode/stereonet_pointcloud2
# 参考官方：
#   ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int16.launch.py \
#     mipi_image_width:=640 mipi_image_height:=352 mipi_lpwm_enable:=True \
#     mipi_image_framerate:=30.0 mipi_rotation:=90.0 need_rectify:=False ...
# 标定：SC132gs_dual_calibration.yaml，基线约 0.0792 m（模组标称 80 mm）
set -e
# shellcheck disable=SC1091
source /opt/tros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

if [ ! -w /userdata/.roslog ] 2>/dev/null; then
  echo sunrise | sudo -S mkdir -p /userdata/.roslog >/dev/null 2>&1 || true
  echo sunrise | sudo -S chown -R "$(id -un):$(id -gn)" /userdata/.roslog >/dev/null 2>&1 || true
  echo sunrise | sudo -S chmod 777 /userdata/.roslog >/dev/null 2>&1 || true
fi
export ROS_LOG_DIR=/userdata/.roslog
mkdir -p "$ROS_LOG_DIR"

MODEL="${STEREO_MODEL:-/opt/tros/humble/share/hobot_stereonet/config/DStereoV2.4_int16.bin}"
# SC132gs 标定 fx≈fy（方像素）。切勿按高宽比分别缩放 fy，否则点云呈扇形失真。
# 1280→640 等比：fx=fy≈328.4；主点取 640x352 图像中心附近。
FX="${CAMERA_FX:-328.379}"
FY="${CAMERA_FY:-328.379}"
CX="${CAMERA_CX:-320.0}"
CY="${CAMERA_CY:-176.0}"
BASELINE_M="${BASELINE_M:-0.07917}"

# 优先用 mipi 自带 camera_info；没有数据时由 pub_stereo_caminfo 补
RIGHT_INFO="${CAMERA_INFO_TOPIC:-/drone/stereo/right/camera_info}"
LEFT_INFO="${LEFT_CAMERA_INFO_TOPIC:-/drone/stereo/left/camera_info}"
if timeout 2 ros2 topic echo /image_combine_raw/right/camera_info --once >/dev/null 2>&1; then
  RIGHT_INFO=/image_combine_raw/right/camera_info
  LEFT_INFO=/image_combine_raw/left/camera_info
  echo "[GS130W stereonet] use mipi camera_info topics"
fi

echo "[GS130W stereonet] model=$MODEL"
echo "[GS130W stereonet] fx=$FX fy=$FY cx=$CX cy=$CY bl=${BASELINE_M}m"
echo "[GS130W stereonet] pointcloud_downsample_step=1 (max for 640x352)"

exec ros2 launch hobot_stereonet stereonet_model.launch.py \
  stereonet_model_file_path:="$MODEL" \
  stereo_image_topic:=/image_combine_raw \
  camera_info_topic:="$RIGHT_INFO" \
  left_camera_info_topic:="$LEFT_INFO" \
  calib_method:=none \
  camera_fx:="$FX" \
  camera_fy:="$FY" \
  camera_cx:="$CX" \
  camera_cy:="$CY" \
  baseline:="$BASELINE_M" \
  stereonet_frame_id:=camera_link \
  publish_pcd_enabled:=True \
  publish_origin_enable:=True \
  publish_visual_enabled:=True \
  publish_rectify_bgr:=False \
  pointcloud_downsample_step:=1 \
  pointcloud_height_min:=-10.0 \
  pointcloud_height_max:=10.0 \
  pointcloud_depth_max:=5.0 \
  render_type:=indoor \
  render_perf:=False \
  uncertainty_th:=-0.10 \
  save_result_flag:=False \
  save_stereo_flag:=False \
  save_disp_flag:=False \
  save_depth_flag:=False \
  save_visual_flag:=False \
  save_pcd_flag:=False \
  save_origin_flag:=False \
  log_level:=warn \
  "$@"
