#!/usr/bin/env bash
set -e
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# 默认 GS130W MIPI（ISP）；无双目时回退 USB。--source usb 强制 VideoCapture。
# --show：弹窗；无显示环境自动回退快照。只发话题时追加 --no-show
exec bash /app/zettatree_demo/_common/start_vision_cam.sh --show "$@"
