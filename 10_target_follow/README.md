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
YOLO 行人框 → 深度取 3D → 计算 standoff 跟随点
        ↓
 /move_base_simple/goal  →  EGO（避障 B 样条）→ traj_server → OFFBOARD
OpenCV：检测画面（行人框）| 三维俯视（N=初始机头）| 底部中文状态栏
RViz：与例程 8 相同的官方彩色点云 + Depth Color + 目标/跟随点 + occ_viz + 规划 Marker
```

| 层级 | 内容 |
|------|------|
| 感知 | Stereonet 深度 + BPU YOLO（COCO `person`，限频 5 Hz，目标丢失 1 s 记忆保持） |
| 规划 | 完整 C++ EGO `flight_type=MANUAL_TARGET`（建图参数同例程 9），目标点周期发布到 `/move_base_simple/goal`，依赖例程 9 的动态目标重规划补丁 |
| 对照 | `planner:=direct` 直接位置跟随（无需编 EGO） |
| OpenCV | 检测画面（行人框）+ 三维俯视（航迹；机体→跟随点→目标连线；「人」/「跟」中文标记）+ 底部中文状态栏（阶段/速度/高度/目标距离/七扇区距离） |

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
| `standoff` | `0.8` | 跟随保持距离（米） |
| `follow_z` | `0.1` | 跟随高度（米） |
| `max_vel` | `0.02` | 跟随最大速度（m/s，室内拆桨慢速） |
| `safe_distance` | `1.2` | 安全层急停距离（米） |
| `stop_distance` | `0.45` | 绕行触发距离（米） |
| `min_score` | `0.25` | YOLO person 置信度阈值（与例程 04 同默认） |
| `show` | `true` | OpenCV HUD 窗口；无 DISPLAY 自动改写快照 |
| `snapshot` | 空 | 无显示器时的 JPEG 快照路径 |
| `rviz` | `true` | RViz2（官方彩色点云 + 目标/路径；`rviz:=false` 省 CPU） |

## 话题说明

| 话题 | 说明 |
|------|------|
| `/move_base_simple/goal` | 跟随点 → EGO 动态目标（ego 模式，约 1.8 Hz） |
| `/drone/setpoint_position/local` | 位置设定点（direct 模式 / EGO poscmd 桥接） |
| `/drone/follow/target` | 行人 3D 位置（RViz 可视） |
| `/drone/follow/goal` | 计算的跟随点（RViz 可视） |
| `/position_cmd` | traj_server 轨迹命令（100 Hz） |
| `/drone/nav/path_history` | 历史航迹 |
| `/grid_map/occupancy(_inflate)` | EGO 占据栅格 |
| `/StereoNetNode/stereonet_pointcloud2` | Stereonet 点云 |

## 注意事项

- **必须拆桨**：`arm:=true` 会切 OFFBOARD 并解锁，电机会转。
- **紧急停止**：Ctrl+C 经 UART 强制上锁，不依赖 MAVROS。
- 先完成例程 8（Stereonet）与例程 9（`ego_ws` 编译）。动态目标补丁由例程 9 的 `setup_full_ego.sh` 自动应用。

| 现象 | 处理 |
|------|------|
| `ego_planner` 包不可用 | `bash setup.sh` |
| 检测不到行人 | 距离 0.5–5 m、光照充足；可把 `min_score:=0.2` |
| 目标短暂丢失就停 | 正常：约 1 s 记忆，超时后悬停并继续搜索 |
| HUD 无窗口 | 无 DISPLAY 时写快照 `/tmp/target_follow_snapshot.jpg`，或传 `snapshot:=` |

```bash
source /opt/tros/humble/setup.bash
source /app/zettatree_demo/09_depth_nav/ego_ws/install/setup.bash
ros2 topic hz /move_base_simple/goal
ros2 topic hz /position_cmd
ros2 topic echo /drone/follow/target
```

