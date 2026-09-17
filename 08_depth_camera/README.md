# 08 深度摄像头 — RDK Stereo Camera GS130W

文档章节：**4.3**。本例程**不接飞控**。

默认链路：GS130W MIPI 双目 → `hobot_stereonet`（BPU DStereo V2.4）→ OpenCV / RViz。

```text
GS130W → mipi_cam dual → hobot_stereonet
        │
        ├─ stereonet_visual       → /drone/depth/image_color
        ├─ stereonet_depth        → /drone/depth/image_raw
        └─ stereonet_pointcloud2  → RViz 主点云
                                      └→ 可选 /drone/depth/points（轻量抽稀）
```

## 启动

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh
```

| 命令 | 说明 |
|------|------|
| `bash run.sh` | MIPI → Stereonet → OpenCV + RViz |
| `bash run.sh rviz:=false` | 不启动 RViz |
| `bash run.sh views` | 分窗：LEFT / RIGHT / VISUAL / DEPTH |
| `bash run.sh rviz` | 仅开 RViz（需另终端已跑默认模式） |
| `bash run.sh source:=simulate start_stereonet:=false` | 无相机，验证软件 |
| `bash run.sh source:=orbbec` / `realsense` | USB 深度相机备选 |

| 窗口（默认模式） | 内容 |
|------|------|
| OpenCV 左 | 官方深彩 |
| OpenCV 右 | 当前帧俯视点云（`panel_mode:=depth` 可关掉） |
| RViz | `/StereoNetNode/stereonet_pointcloud2`，Fixed Frame=`camera_link` |

板端需 **桌面终端**（有 `DISPLAY`）才能弹 OpenCV / RViz。

## 目录结构

```
08_depth_camera/
├── run.sh                  # 启动入口
├── README.md
├── ensure_mipi_bpu.sh      # GS130W dual（09/10 复用）
├── pub_stereo_caminfo.py   # CameraInfo（09/10 复用）
├── start_stereonet.sh      # Stereonet 参数与启动（09/10 复用）
├── depth_camera.launch.py
├── depth_pointcloud.py     # OpenCV 深彩 / 俯视
├── show_stereo_views.py    # views 模式分窗
├── pointcloud_map.py       # 可选地图节点（默认关）
└── depth_cloud.rviz
```

## 模组与参数

| 项目 | 值 |
|------|-----|
| 型号 | GS130W（双 SC132GS） |
| 基线 | **80 mm**（标定约 **79.17 mm**） |
| 分辨率 | **640×352**，LPWM，`rotation=90`，`channel=2/0`，`dual_combine=2` |
| 内参 | **`fx=fy≈328.379`**，`cx=320`，`cy=176` |
| 模型 | `DStereoV2.4_int16.bin`，`post_version=auto`，`uncertainty_th=-0.09` |
| 右目 CameraInfo | `P[0,3]=+fx·B` |
| Stereonet | `calib_method=none`，`baseline`，`render_type=indoor` |

## 话题

| 话题 | 说明 |
|------|------|
| `/image_combine_raw` | MIPI 上下拼接 NV12（640×704） |
| `/drone/stereo/*/camera_info` | 左右目内参 |
| `/StereoNetNode/stereonet_depth` | 深度（mono16，mm） |
| `/StereoNetNode/stereonet_visual` | 官方深彩 |
| `/StereoNetNode/stereonet_pointcloud2` | 官方 XYZRGB 点云 |
| `/drone/depth/image_raw` | 原始深度转发 |
| `/drone/depth/image_color` | 官方深度伪彩 |
| `/drone/depth/points` | 轻量抽稀点云 |

```bash
source /app/zettatree_demo/_common/env.sh
ros2 topic hz /image_combine_raw
ros2 topic hz /StereoNetNode/stereonet_depth
ros2 topic hz /StereoNetNode/stereonet_pointcloud2
```

深度正常时 `stereonet_depth` 约 15 fps。大图请用 `ros2 topic hz` 判活，不要 `topic echo`。

## Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `source` | `stereonet` | `stereonet` / `simulate` / `orbbec` / `realsense` |
| `rviz` | `true` | 是否启动 RViz |
| `show` | `true` | OpenCV 窗口 |
| `start_stereonet` | `true` | 是否拉起 BPU Stereonet |
| `baseline_m` | `0.07917` | 基线（米） |
| `min_range` / `max_range` | `0.3` / `5.0` | 深度范围（米） |
| `map` | `false` | 无位姿时不要累积点云；需要时显式 `map:=true` |
| `tf_x/y/z`、`tf_roll/pitch/yaw` | `0` | `base_link` → `camera_link` 安装位姿 |

点云密度：环境变量 `POINTCLOUD_DOWNSAMPLE_STEP`（默认 **2**）。

安装位置示例：

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh \
  tf_x:=0.12 tf_y:=0.0 tf_z:=-0.08
```

## 依赖

- `tros-humble-hobot-stereonet`、`mipi_cam`、GS130W 扩展板
- 例程 9 / 10 复用本目录的 `ensure_mipi_bpu.sh`、`pub_stereo_caminfo.py`、`start_stereonet.sh`
