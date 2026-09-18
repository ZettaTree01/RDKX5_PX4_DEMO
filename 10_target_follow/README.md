# 10 目标跟随（行人 / 动态目标）

## 快速开始

```bash
cd /app/zettatree_demo/10_target_follow

# 首次使用：拉取/编译 EGO-Planner（已就绪则秒过）+ 深度相机检查
bash setup.sh

# 标准目标跟随（EGO 避障 + Stereonet + YOLO 行人）——必须拆桨
bash run.sh arm:=true

# 直接位置跟随（无 EGO 规划器的台架对照）
bash run.sh arm:=true planner:=direct

# 虚拟行人绕圈（无需真人/相机，验证完整控制链路）
bash run.sh arm:=true source:=simulate

# 无显示器时 HUD 定期写 JPEG 快照
bash run.sh arm:=true show:=false snapshot:=/tmp/tf10.jpg
```

## 例程说明

文档 4.5。在例程 9（Stereonet + EGO）上深化：**无人机跟随人的移动而移动**。

单独验证双目：

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh
# 或 bash .../08_depth_camera/run.sh views
```

技术栈对齐 [Fast-Planner](https://github.com/SnapDragonfly/Fast-Planner) / [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)：

```
YOLO 行人框（`/StereoNetNode/rectified_image`）→ 深度取 3D → 沿机头方向退 standoff 得跟随点
        ↓
 /move_base_simple/goal  →  EGO（避障 B 样条）→ traj_server → OFFBOARD
OpenCV：检测画面（行人框）| 三维俯视（N=初始机头）| 底部中文状态栏
RViz2：例程8同款（Fixed Frame=`camera_link` + 官方彩色点云）+ EGO 规划路径 + 跟随连线/目标点
```

| 层级 | 内容 |
|------|------|
| 感知 | Stereonet 深度 + BPU YOLO（COCO `person`，独立线程 10 Hz，目标丢失 1 s 记忆保持） |
| 规划 | 完整 C++ EGO `flight_type=MANUAL_TARGET`（建图参数同例程 9），目标点周期发布到 `/move_base_simple/goal`，依赖例程 9 的动态目标重规划补丁 |
| 对照 | `planner:=direct` 直接位置跟随（无需编 EGO） |
| OpenCV | 检测画面（行人框）+ 三维俯视（航迹 + EGO 规划路径；「人」/「跟」标记）+ 底部中文状态栏 |

室内默认 `bench:=true`。**必须拆桨**。

## 文件结构

```
10_target_follow/
├── README.md                 # 本文档
├── setup.sh                  # 环境配置（复用例程09的 EGO 构建）
├── run.sh                    # 统一运行脚本
├── target_follow.launch.py   # 启动文件（MAVROS/Stereonet/EGO/桥接）
├── target_follow.py          # YOLO + 深度 + 跟随点主程序
└── target_follow.rviz        # RViz 可视化配置
```

## 脚本说明

### setup.sh — 环境配置（板端执行一次）

EGO-Planner 的拉取/打补丁/编译全部**复用例程 09 的 `setup_full_ego.sh`**，两个例程共享同一个 `ego_ws`。若例程 9 已编译且动态目标补丁在位，本脚本直接跳过编译，只做深度相机就绪检查。

### run.sh — 统一运行脚本

自动完成：环境加载 → GS130W mipi 检查（用 Stereonet 时）→ `planner:=ego` 时 source EGO 工作空间 → `ros2 launch`。Ctrl+C / 异常退出自动经 UART 强制上锁（`_common/run_flight.sh`）。

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `planner` | `ego` | `ego`=完整 C++ EGO（`/move_base_simple/goal`）；`direct`=位置直跟 |
| `source` | `stereonet` | 深度源；`simulate`=虚拟行人绕圈 |
| `start_stereo` | `true` | 是否拉起例程 8 的 Stereonet |
| `arm` | `false` | `true` 才解锁（必须拆桨） |
| `bench` | `true` | 台架模式（起飞斜坡不看气压计） |
| `standoff` | `0.8` | 沿机头方向与行人保持的水平距离（米）；人偏右则右移以居中 |
| `follow_z` | `0.1` | 跟随高度（米） |
| `max_vel` | `0.02` | 跟随最大速度（m/s，室内拆桨慢速） |
| `safe_distance` | `1.2` | 安全层急停距离（米） |
| `stop_distance` | `0.45` | 绕行触发距离（米） |
| `min_score` | `0.25` | YOLO person 置信度阈值（与例程 04 同默认） |
| `show` | `true` | OpenCV HUD 窗口；无 DISPLAY 自动改写快照 |
| `snapshot` | 空 | 无显示器时的 JPEG 快照路径 |
| `rviz` | `true` | 开启 RViz2（点云 + 规划路径）。SSH 自动显示到本机桌面 `:0` |

## 话题说明

| 话题 | 说明 |
|------|------|
| `/move_base_simple/goal` | 跟随点 → EGO 动态目标（ego 模式，约 1.8 Hz） |
| `/drone/setpoint_position/local` | 位置设定点（direct 模式 / EGO poscmd 桥接） |
| `/position_cmd` | traj_server 轨迹命令（100 Hz） |
| `/drone/follow/target` | 行人 3D（world，z 已对齐） |
| `/drone/follow/goal` | 跟随点（world） |
| `/drone/follow/link` | 机体→跟随点→行人连线（RViz Marker） |
| `/optimal_list` / `/a_star_list` | EGO 规划 Marker |
| `/drone/nav/path_plan` | 规划路径（由 `/optimal_list` 转发） |
| `/drone/nav/path_history` | 已飞航迹（world） |
| `/grid_map/occupancy_inflate` | EGO 膨胀占据 |
| `/StereoNetNode/stereonet_pointcloud2` | 例程8同款彩色点云 |

## 注意事项

- **必须拆桨**：`arm:=true` 会切 OFFBOARD 并解锁，电机会转。
- **紧急停止**：Ctrl+C 经 UART 强制上锁，不依赖 MAVROS。
- 先完成例程 8（Stereonet）与例程 9（`ego_ws` 编译）。动态目标补丁由例程 9 的 `setup_full_ego.sh` 自动应用。

| 现象 | 处理 |
|------|------|
| `ego_planner` 包不可用 | `bash setup.sh` |
| 检测不到行人 | 人站在镜头前 0.5–5 m、光照充足；确认订阅 `/StereoNetNode/rectified_image`（本机不发 `origin_left_image`）；可把 `min_score:=0.2` |
| EGO 报 `the drone is in obstacle` / 无 `position_cmd` | 1) 机体前方约 1.5 m 内不要有椅子桌沿；2) 确认 `/odom_world` 在地图内（原点对齐后应接近 0,0,0）；3) 点云桥已滤 `min_depth=0.25` 与机体清空半径 |
| 目标短暂丢失就停 | 正常：约 1 s 记忆，超时后悬停并继续搜索 |
| HUD 无窗口 | 无 DISPLAY 时 OpenCV 写快照 `/tmp/target_follow_snapshot.jpg`。RViz2 显示在本机 HDMI；同网开发机可打开 `target_follow.rviz` |
| RViz 无彩色点云 | Fixed Frame 应为 `camera_link`；确认 stereonet_pointcloud2 已勾选 |
| 停止后仍有进程 | `bash /app/zettatree_demo/_common/stop_nav_stack.sh` |
| 有点云无规划线 | 需检出到行人并下发 goal；RViz 勾选 OptimalBspline / EgoPlan |

```bash
source /opt/tros/humble/setup.bash
source /app/zettatree_demo/09_depth_nav/ego_ws/install/setup.bash
ros2 topic hz /move_base_simple/goal
ros2 topic hz /position_cmd
ros2 topic echo /drone/follow/target
```

