#!/usr/bin/env bash
# 03 摄像头例程入口：普通 USB 摄像头发图并弹窗/快照，发布 /camera/image_raw。
# 实际采集由 _common/start_vision_cam.sh（本例程固定 --source usb）。
# --device /dev/videoN 换设备；--no-show 只发话题不显示。
set -e
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# 例程 03/04/05 统一普通 USB；弹窗默认开，无显示环境自动回退快照
exec bash /app/zettatree_demo/_common/start_vision_cam.sh --source usb --show "$@"
