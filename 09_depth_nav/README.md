# 09 深度相机自主导航（完整 C++ EGO-Planner + 例程 8 GS130W）

## 例程说明

文档 **4.4**。默认使用上游完整栈 [ZJU ego-planner-swarm `ros2_version`](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)（B 样条优化 + `traj_server`），对接 Stereonet 点云与 OFFBOARD。

```
Stereonet 点云 → world 点云 → grid_map → ego_planner_node（A*+B样条）
                                              ↓
                                         traj_server
                                              ↓
                                    PositionCommand → offboard
OpenCV：官方深彩 | **3D POINT** 俯视 | 底部状态栏
RViz2：例程8同款官方彩色点云 + world 点云建模 + 膨胀占据 + EGO 规划 Marker
```

| 层级 | 内容 |
|------|------|
| OpenCV | 深彩 + **3D POINT** 俯视；底部中文状态栏 |
| RViz2 | 官方 `stereonet_pointcloud2`（RGB8）+ 深彩 + OccViz / grid_map / Marker |
| 控制 | `traj_server` 位姿设定点；深度安全层过近时速度覆盖 |

可选：`backend:=python` 退回板端 Python A* 同构实现（教学对照）。

深度相机侧复用例程 8 的 `ensure_mipi_bpu.sh` / `pub_stereo_caminfo.py` / `start_stereonet.sh`。单独验证双目可先：

```bash
bash /app/zettatree_demo/08_depth_camera/run.sh          # 或 views
```

## 板端编译（首次）

```bash
bash /app/zettatree_demo/09_depth_nav/setup_full_ego.sh
# 若曾失败（Duplicate package / 脏 ego_ws）：
EGO_FORCE_CLEAN=1 bash /app/zettatree_demo/09_depth_nav/setup_full_ego.sh
# 产物：$SCRIPT_DIR/ego_ws/install ，含 ego_planner_node / traj_server
```

建议 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（脚本已默认）。

## 话题（完整 EGO）

| 话题 | 说明 |
|------|------|
| `/StereoNetNode/stereonet_pointcloud2` | 官方彩色点云 |
| `/drone/ego/cloud_world` | 变换到 world 的建图点云 |
| `/odom_world` | MAVROS pose → Odometry |
| `/grid_map/occupancy` | EGO 占据点云 |
| `/grid_map/occupancy_inflate` | 膨胀占据 |
| `/optimal_list` / `/a_star_list` | 规划 Marker |
| `/planning/bspline` | B 样条轨迹 |
| `/position_cmd` | traj_server 位置指令 |
| `/drone/setpoint_position/local` | 转发给 OFFBOARD |
| `/traj_start_trigger` | 起飞后触发航点规划 |

室内默认 `bench:=true`。**必须拆桨**。

## 启动

```bash
# 首次编译
bash /app/zettatree_demo/09_depth_nav/setup_full_ego.sh

# 默认：完整 C++ EGO + Stereonet + OpenCV + RViz
bash /app/zettatree_demo/09_depth_nav/run.sh

# 监视解锁
bash /app/zettatree_demo/09_depth_nav/run.sh arm:=true

# 已跑例程8（不再拉 Stereonet）
bash /app/zettatree_demo/09_depth_nav/run.sh arm:=true start_stereo:=false

# 退回 Python A*（对照）
bash /app/zettatree_demo/09_depth_nav/run.sh backend:=python
```

> Stereonet 模式下 MIPI 未就绪则退出，避免无图启动规划。
>
> `Ctrl+C` 会经 `_common/run_flight.sh` 先停 MIPI/Stereonet/EGO/RViz/MAVROS，再限时强制上锁；若仍有残留可手动：
> `bash /app/zettatree_demo/_common/stop_nav_stack.sh`

## RViz2（完整 EGO）

| Display | Topic |
|---------|-------|
| Depth Color | `/StereoNetNode/stereonet_visual` |
| stereonet_pointcloud2 | `/StereoNetNode/stereonet_pointcloud2`（RGB8，与例程8相同，Fixed Frame=`camera_link`） |
| GridMapInflate | `/grid_map/occupancy_inflate`（橙色膨胀占据，**默认关闭**；建模看彩色点云即可，需要看规划障碍时再勾选） |
| OptimalBspline / AStarList | `/optimal_list`、`/a_star_list`（规划路线，frame=`world`） |
| PathHistory | `/drone/nav/path_history` |

Fixed Frame = **`camera_link`**（与例程 8 相同，官方彩色点云无需 TF 即可显示；规划 Marker 在 `world`，由 `pose_to_odom` 提供 `world→camera_link`）。SSH 启动也会把 RViz 挂到本机桌面 `:0`。
**不必**为建模接入橙色点云：规划建图在后台走 `/drone/ego/cloud_world`，与 RViz 是否显示 inflate 无关。

## 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| （默认） | full | 完整 C++ EGO |
| `backend:=python` | — | Python A* 同构 |
| `rviz` | `true` | **必须开启** RViz（官方彩色点云 + OccViz）。SSH 自动挂到本机桌面 `:0` |
| `source` | `stereonet` | 深度源 |
| `arm` | `false` | 解锁 |
| `max_vel` | `0.02` | 安全层速度上限 |
| `safe_distance` | `1.2` | 深度安全层 |
| `stop_distance` | `0.45` | 急停 |

## 文件

| 文件 | 作用 |
|------|------|
| `setup_full_ego.sh` | 拉取并编译完整 EGO |
| `ego_full.launch.py` / `run_ego_full.sh` | 完整 EGO 启动 |
| `bridges/*.py` | odom / 点云 / pos_cmd 桥 |
| `ego_full.rviz` | 完整 EGO 可视化 |
| `depth_nav.py` | OpenCV + 安全层（`control-mode=safety`） |
| `ego_planner_node.py` | 仅 `backend:=python` 时使用 |
| `_common/ego_local_planner.py` | Python A* 核心 |

## 依赖

- 例程 8 GS130W + Stereonet（`08_depth_camera/run.sh` 可单独验证）
- [ego-planner-swarm ros2_version](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)
- MAVROS + `offboard_manager.py`
- `libarmadillo-dev`、PCL、Eigen（见 setup 脚本）

台架注意：室内 MAVROS local 会带着气压绝对高度和上次飞行残留的 XY。`pose_to_odom` 在 `z_align:=true` 时把起飞位锁成 EGO 原点（`/drone/ego/origin_ref`），点云/轨迹按同一基准换系，否则飞机会落到 8 m 地图外，`getInflateOccupancy` 返回 -1 被当成障碍。建图只用 `/drone/ego/cloud_world`（已滤掉 0.25 m 内无效视差），不要把原始深度图接到 `grid_map/depth`。
