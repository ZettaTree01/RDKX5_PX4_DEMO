#!/usr/bin/env bash
# 板端官方 YOLOv8 图片推理：独立验证 BPU，不依赖 ROS/飞控。
# 调用 pydev_demo 官方样例，结果图写到 /tmp/zettatree_yolov8_result.jpg。
set -e
cd /app/pydev_demo/02_detection_sample/03_ultralytics_yolov8
python3 ultralytics_yolov8.py \
  --model-path /opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin \
  --test-img kite.jpg \
  --label-file coco_classes.names \
  --img-save-path /tmp/zettatree_yolov8_result.jpg \
  --score-thres 0.25
echo "结果图: /tmp/zettatree_yolov8_result.jpg"
