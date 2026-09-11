#!/bin/bash
set -e
# shellcheck disable=SC1091
source /app/zettatree_demo/_common/env.sh
# --show：弹窗显示实时画面；无显示环境自动回退为快照输出
# 只发话题不显示画面时，追加 --no-show
exec python3 /app/zettatree_demo/_common/camera_node.py --show "$@"
