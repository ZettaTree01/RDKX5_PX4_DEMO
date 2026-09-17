# 08 深度摄像头 V2 — RDK Stereo Camera GS130W

文档章节：**4.3**。本例程**不接飞控**。

本 V2 在原例程基础上优化：**BPU 官方点云作为 RViz 主数据源；原始深度/深彩图标准化转发；默认关闭无位姿点云累积；增加 base_link→camera_link 静态 TF；限制 CPU 侧点云处理频率；去除脚本中的硬编码 sudo 密码。**

默认链路：

```text
GS130W → mipi_cam dual → hobot_stereonet (BPU DStereo V2.4)
        │
        ├─ stereonet_visual       → /drone/depth/image_color（官方深彩图）
        ├─ stereonet_depth        → /drone/depth/image_raw（原始深度）
        └─ stereonet_pointcloud2  → RViz 主点云 / 3D
                                      │
                                      └→ 可选 /drone/depth/points（轻量抽稀）
```

## 统一入口 `run.sh`

只保留一个启动脚本；子模式用首个参数区分，其余 `key:=value` 透传给 launch。

| 命令 | 说明 |
|------|------|
| `bash run.sh` | **默认**：ensure MIPI → CameraInfo → Stereonet → OpenCV + RViz |
| `bash run.sh rviz:=false` | 省 CPU，不要 RViz |
| `bash run.sh views` | 分窗：LEFT / RIGHT / VISUAL / DEPTH |
| `bash run.sh views --no-depth --no-visual` | 只要左右目（仍可拉 Stereonet） |
| `bash run.sh views start_stereo:=0 --no-depth --no-visual` | 只要左右目，不拉 Stereonet |
| `bash run.sh rviz` | 仅开 RViz（需另终端已跑默认模式） |
| `bash run.sh source:=simulate start_stereonet:=false` | 无相机，验证软件 |
| `bash run.sh source:=orbbec` / `realsense` | USB 深度相机备选 |

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh
```

| 窗口（默认模式） | 内容 |
|------|------|
| OpenCV 左 | 官方深彩 |
| OpenCV 右 | **3D POINT** 俯视（抽稀；panel_mode=depth 时不做转换） |
| RViz | `/StereoNetNode/stereonet_pointcloud2`，Fixed Frame=`camera_link`，Color=`RGB8` |

板端需 **桌面终端**（有 `DISPLAY`）才能弹 OpenCV / RViz。

## 目录结构

```
08_depth_camera/
├── run.sh                  # 唯一用户入口（full / views / rviz）
├── README.md
├── ensure_mipi_bpu.sh      # GS130W dual（09/10 复用）
├── pub_stereo_caminfo.py   # 有效 CameraInfo（09/10 复用）
├── start_stereonet.sh      # DStereoV2.4 参数 YAML（09/10 复用）
├── depth_camera.launch.py  # 编排 caminfo / Stereonet / OpenCV / RViz
├── depth_pointcloud.py     # OpenCV 深彩 | 3D POINT
├── show_stereo_views.py    # views 模式分窗
├── pointcloud_map.py       # 可选独立地图节点（默认关）
├── depth_cloud.rviz        # RViz 配置
└── flip_combine_nv12.py    # DEBUG：上下半幅对调（正式流程不启动）
```

## 模组与关键参数

| 项目 | 值 |
|------|-----|
| 型号 | GS130W（双 SC132GS） |
| 基线 | **80 mm**（标定约 **79.17 mm**） |
| 分辨率 | **640×352**，LPWM，`rotation=90`，`channel=2/0`，`dual_combine=2` |
| 内参 | **`fx=fy≈328.379`**，`cx=320`，`cy=176` |
| 模型 | `DStereoV2.4_int16.bin`，`postprocess=v2.3`，`uncertainty_th=-0.09` |
| 右目 CameraInfo | `P[0,3]=+fx·B`（勿为 0 或 `-fx·B`） |

## 话题

| 话题 | 说明 |
|------|------|
| `/image_combine_raw` | MIPI 上下拼接 NV12（640×704） |
| `/drone/stereo/*/camera_info` | 例程发布的有效内参 |
| `/StereoNetNode/stereonet_depth` | 深度（mono16，mm） |
| `/StereoNetNode/stereonet_visual` | 官方深彩 |
| `/StereoNetNode/stereonet_pointcloud2` | 官方 XYZRGB 点云 |
| `/drone/depth/image_raw` | 原始深度转发，保留原始编码/时间戳 |
| `/drone/depth/image_color` | 官方深度伪彩图 |
| `/drone/depth/points` | 轻量抽稀点云，仅供 OpenCV/可选地图 | 例程抽稀点云（俯视用） |

## 验证

```bash
source /app/zettatree_demo/_common/env.sh
ros2 topic hz /image_combine_raw
ros2 topic hz /StereoNetNode/stereonet_depth
ros2 topic hz /StereoNetNode/stereonet_visual
ros2 topic hz /StereoNetNode/stereonet_pointcloud2
```

- 判活用 **`ros2 topic hz`**，勿对大图 `topic echo`（易超时误报无帧）。
- 深度正常时 `stereonet_depth` 约 **15 fps**，非零像素应占绝大多数（此前「深度全 0」见下节）。

## Launch 常用参数（默认模式透传）

| 参数 | 默认 | 说明 |
|------|------|------|
| `source` | `stereonet` | `stereonet` / `simulate` / `orbbec` / `realsense` / `mipi_stereo` |
| `rviz` | `true` | 是否启动 RViz |
| `show` | `true` | OpenCV 窗口 |
| `start_stereonet` | `true` | 是否拉起 BPU Stereonet |
| `start_mipi` | `false` | 默认由 `ensure_mipi_bpu.sh` 拉起 |
| `baseline_m` | `0.07917` | 基线（米） |
| `min_range` / `max_range` | `0.3` / `5.0` | 深度范围（米） |

点云密度：环境变量 `POINTCLOUD_DOWNSAMPLE_STEP`（默认 **2**）。

## Stereonet 深度全 0（例程 9/10 同样适用）

正式脚本已固化下列项；若自改参数请勿回退：

1. **`postprocess:=v2.3`**（DStereoV2.4 官方要求）
2. **`uncertainty_th:=-0.09`**（须为负数）
3. 右目 **`P[0,3]=+fx·baseline`**（节点用 `P[0,3]/fx` 覆盖 `base_line`）
4. **`need_rectify:=false`** + 显式 `camera_fx/fy/cx/cy` + 参数名 **`base_line`**
5. MIPI **`dual_combine:=2`** + Stereonet **`stereo_combine_mode:=1`**
6. 停 mipi 只杀二进制路径，勿 `pkill -f mipi_cam`（会误杀启动脚本）

## 依赖

- `tros-humble-hobot-stereonet`、`mipi_cam`、GS130W 扩展板
- 例程 9 / 10 复用本目录的 `ensure_mipi_bpu.sh`、`pub_stereo_caminfo.py`、`start_stereonet.sh`

## V2 优化说明

1. **RViz 默认直接订 `/StereoNetNode/stereonet_pointcloud2`**，避免官方彩色点云经过 Python 再序列化，减少 CPU/内存拷贝。
2. **`/StereoNetNode/stereonet_depth` 原样转发到 `/drone/depth/image_raw`**，不修改编码和时间戳；深彩图转发到 `/drone/depth/image_color`。
3. 默认 `map:=false`，不再因为 Demo 方便而无视位姿累积点云；需要 Demo 地图时显式 `map:=true`。真正无人机建图应使用 TF/Pose 后再累积。
4. 增加 `base_link → camera_link` 静态 TF，默认零位姿；实际安装时使用 `tf_x/y/z/tf_roll/pitch/yaw`。
5. CPU 侧点云处理默认最高 4 Hz，仅服务 OpenCV 俯视/可选过滤点云；BPU 官方点云和 RViz 不受影响。
6. 移除脚本中的硬编码 sudo 密码，改为无密码 sudo 尝试；无权限时仅保留当前用户可写目录。

### 推荐启动

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh
```

只看深彩图并降低 CPU：

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh rviz:=false panel_mode:=depth publish_hz:=2
```

无人机安装位置示例（单位 m / rad）：

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh \
  tf_x:=0.12 tf_y:=0.0 tf_z:=-0.08 \
  tf_roll:=0 tf_pitch:=0 tf_yaw:=0
```

> 注意：`map:=true` 仍只是相机坐标系 Demo 累积，不等于 SLAM。后续接 PX4 时应使用视觉/里程计 Pose 将点云转换到 `map/odom` 后再建图。
