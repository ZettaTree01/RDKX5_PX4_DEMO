# 08 深度摄像头 — RDK Stereo Camera GS130W

## 实现思路

```
双目摄像头(GS130W)
        │
        ▼
     Depth          ← hobot_stereonet (BPU)
        │
        ├──────────────────────────┐
        ▼                          ▼
   OpenCV 双栏                RViz 官方彩色点云
   深彩 | 三维俯视            /StereoNetNode/stereonet_pointcloud2
   （内部抽稀累积）           Fixed Frame=camera_link，RGB8
```

## 一键启动

板端**桌面终端**：

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh
```

| 窗口 | 内容 |
|------|------|
| OpenCV 左 | 官方深彩 |
| OpenCV 右 | 三维俯视（体素累积） |
| RViz | 官方 XYZRGB 点云（Flat Squares / RGB8） |

Fixed Frame = **`camera_link`**（与 `stereonet_frame_id` 一致，对齐官方文档）。

```bash
# 不要 RViz
bash .../run.sh rviz:=false
```

## 模组

| 项目 | 值 |
|------|-----|
| 型号 | GS130W（双 SC132GS） |
| 基线 | **80 mm**（标定约 **79.2 mm**） |
| 分辨率 | **640×352** + LPWM + `ch=2/0` + `rot=90` |
| 内参 | **`fx=fy≈328.4`** |

## 验证

```bash
source /app/zettatree_demo/_common/env.sh
ros2 topic hz /StereoNetNode/stereonet_visual
ros2 topic hz /StereoNetNode/stereonet_pointcloud2
```
