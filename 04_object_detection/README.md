# 04 目标检测

## 例程说明

文档 3.1。ROS 节点订阅 `/camera/image_raw`，加载板端量化 YOLO `.bin`，
做**端到端推理**并输出**推理画面**（检测框 + 类别 / 置信度 + 状态提示）。

推理链路与官方示例对齐
（`/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8`）：

```
BGR 帧 → letterbox 缩放到模型输入 → NV12(h*w*1.5) → hbm_runtime.run
      → 反量化 → 三个尺度(8/16/32) DFL 解码 → 拼接 → 按类 NMS
      → 坐标映射回原图 → 画框
```

推理链路实现在共享组件 `_common/yolo_detector.py`（`05_obstacle_avoidance`
同样使用它），本例程节点只负责订阅、画框与发布事件。

## 本节目标

- 在 ROS2 节点中订阅相机话题并完成视觉推理。
- 理解 640×640 NV12 输入格式（`h*w*1.5` 字节），以及像素框中心不等于三维坐标。
- 对照官方示例理解 DFL 解码与 NMS；更换模型时同步更新类别表与解码假设。
- 在板端以弹窗或快照方式查看推理画面。

> **以下命令为前台常驻，需另开终端做其它操作。**

```bash
bash /app/zettatree_demo/04_object_detection/run.sh
```

换摄像头 / 调阈值：

```bash
bash /app/zettatree_demo/04_object_detection/run.sh camera_device:=/dev/video1
bash /app/zettatree_demo/04_object_detection/run.sh score_thres:=0.4 nms_thres:=0.5
```

## 推理画面输出

launch 参数 `show` 默认 `true`，按当前环境自动选择输出方式：

- **有显示环境**（HDMI 显示器桌面终端或 `ssh -X`）：弹窗显示推理画面，
  检测到目标画绿框；窗口内按 `q` / `Esc` 只关闭画面，不退出节点；
- **纯 SSH 无显示环境**：自动回退为快照输出，每 5 秒把推理画面写到
  `/tmp/detection_snapshot.jpg`（含状态提示文字），开发机拉回查看：

```bash
scp sunrise@<X5_IP>:/tmp/detection_snapshot.jpg .
```

画面上的状态提示：

| 提示 | 含义 |
|---|---|
| `model not loaded (raw frame)` | 模型未加载（显示原图），查节点日志确认原因 |
| `no detections` | 模型已加载、推理正常，但当前画面按阈值没有目标 |
| `inference failed (see log)` | 推理报错，看节点日志（5 秒限流打印） |
| 绿框 + `类别 置信度` | 检测到目标，类别取自 `coco_classes.names` |

```bash
# 关闭画面输出（只发话题）
bash /app/zettatree_demo/04_object_detection/run.sh show:=false
```

## 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `camera_device` | `/dev/video0` | 摄像头设备 |
| `show` | `true` | 推理画面输出（弹窗/快照，无显示环境自动回退） |
| `score_thres` | `0.25` | 置信度阈值（概率域，官方默认 0.25） |
| `nms_thres` | `0.45` | NMS IoU 阈值（官方默认 0.45） |

> 类别为 **COCO 80 类**；类别表读取
> `/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8/coco_classes.names`，
> 读不到时回退显示 `cls<id>`。示例输出张量顺序假设为 `[cls,box] × 3`，
> 换非 YOLOv8 / 非 16 分箱模型需同步调整解码参数。

**独立验证（不依赖飞控与相机，可单独运行）— 官方 BPU YOLOv8 图片推理**
```bash
bash /app/zettatree_demo/04_object_detection/run_official_yolov8.sh
```

> 相机节点是共享组件 `_common/camera_node.py`（`03_camera_node` 是它的示例用法）；
> 画面输出逻辑在共享组件 `_common/frame_output.py`；
> YOLO 推理链路在共享组件 `_common/yolo_detector.py`。
