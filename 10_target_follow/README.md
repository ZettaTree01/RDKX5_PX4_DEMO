# 10 目标跟随（行人 / 动态目标）

## 🚀 快速开始

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

# 无显示器 SSH 调试：HUD 定期写 JPEG 快照
bash run.sh arm:=true show:=false snapshot:=/tmp/tf10.jpg
```

## 例程说明

文档 4.5。在例程 9（Stereonet + EGO）上深化：**无人机跟随人的移动而移动**。

Stereonet / MIPI 深度全 0 等排障见 [`08_depth_camera/README.md`](../08_depth_camera/README.md)。单独验证双目：

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
RViz：目标/跟随点 + occ_viz 占据 + 规划 Marker
```

| 层级 | 内容 |
|------|------|
| 感知 | Stereonet 深度 + BPU YOLO（COCO `person`，限频 5 Hz，目标丢失 1 s 记忆保持） |
| 规划 | 完整 C++ EGO `flight_type=MANUAL_TARGET`（建图参数同例程 9 实测值），目标点周期发布到 `/move_base_simple/goal`，依赖例程 9 的动态目标重规划补丁 |
| 对照 | `planner:=direct` 直接位置跟随（无需编 EGO） |
| OpenCV | 检测画面（行人框）+ 三维俯视（航迹；机体→跟随点→目标连线；「人」/「跟」中文标记）+ 底部中文状态栏（阶段/速度/高度/目标距离/七扇区距离） |

室内默认 `bench:=true`。**必须拆桨**。

## 📁 文件结构

```
10_target_follow/
├── README.md                 # 本文档
├── setup.sh                  # 环境配置（复用例程09的 EGO 构建）
├── run.sh                    # 统一运行脚本
├── target_follow.launch.py   # 启动文件（MAVROS/Stereonet/EGO/桥接）
├── target_follow.py          # YOLO + 深度 + 跟随点主程序
├── target_follow.rviz        # RViz 可视化配置
└── deploy_and_debug.py       # 开发机侧：自动部署 + 远程自检/台架测试
```

## 🔧 脚本说明

### setup.sh — 环境配置（板端执行一次）

EGO-Planner 的拉取/打补丁/编译全部**复用例程 09 的 `setup_full_ego.sh`**，两个例程共享同一个 `ego_ws`。若例程 9 已编译且动态目标补丁在位，本脚本直接跳过编译，只做深度相机就绪检查。

### run.sh — 统一运行脚本

自动完成：环境加载 → GS130W mipi 检查（用 Stereonet 时）→ `planner:=ego` 时 source EGO 工作空间 → `ros2 launch`。Ctrl+C / 异常退出自动经 UART 强制上锁（`_common/run_flight.sh`）。

### deploy_and_debug.py — 开发机侧自动部署与调试

从开发机（Windows/Linux）运行，paramiko SSH 到机载计算机：

```bash
python deploy_and_debug.py              # 同步例程10 → 远程自检（8 项）
python deploy_and_debug.py --bench      # 同步 + 台架实跑（arm:=false）
python deploy_and_debug.py --direct     # 台架实跑用 planner:=direct
python deploy_and_debug.py --check-only # 不上传，仅远程自检
```

自检项：Python 语法 / EGO 已编译 / 动态目标补丁在位 / YOLO 模型 / 例程 8 脚本 / ROS2 依赖 / ego_planner 包可用 / run.sh 可执行；台架实跑扫描 Traceback、进程崩溃、EGO executor abort 等致命错误。

## 💡 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `planner` | `ego` | `ego`=完整 C++ EGO（`/move_base_simple/goal`）；`direct`=位置直跟 |
| `source` | `stereonet` | 深度源；`simulate`=虚拟行人绕圈 |
| `start_stereo` | `true` | 是否拉起例程 8 的 Stereonet |
| `arm` | `false` | 是否解锁（台架强制解锁 21196） |
| `bench` | `true` | 台架模式（起飞斜坡不看气压计） |
| `standoff` | `0.8` | 跟随保持距离（米） |
| `follow_z` | `0.1` | 跟随高度（米） |
| `max_vel` | `0.02` | 跟随最大速度（m/s，室内拆桨慢速） |
| `safe_distance` | `1.2` | 安全层急停距离（米） |
| `stop_distance` | `0.45` | 绕行触发距离（米） |
| `min_score` | `0.25` | YOLO person 置信度阈值（与例程 04 同默认） |
| `show` | `true` | OpenCV HUD 窗口；无 DISPLAY 自动改写快照 |
| `snapshot` | 空 | headless 快照 JPEG 路径（`show:=false` 时取证用） |
| `rviz` | `false` | RViz（无稠密 stereonet 点云；默认关） |

## 🔌 话题说明

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

## ✅ 台架实测（2026-09-15，RDK X5 + PX4，拆桨）

| 验证项 | 结果 |
|--------|------|
| `source:=simulate planner:=ego arm:=true` 全链路 | 解锁→起飞→`traj_start_trigger`→z 对齐（z0）→EGO 持续重规划 **2744 次无 abort**（动态目标补丁生效） |
| `/move_base_simple/goal` 频率 | 实测 1.82 Hz（目标 0.55 s 周期发布） |
| `/position_cmd` 频率 | 实测 100.0 Hz |
| EGO→offboard 链路 | `EGO pos_cmd → offboard (x,y,z)` 连续输出，跟踪绕圈虚拟行人 |
| `source:=stereonet planner:=direct` | 解锁→起飞→Stereonet 15 fps（BPU 88%）→YOLO person 检测无失败、深度解码无错误 |
| HUD（`snapshot:=` 取证） | 992×472 双栏渲染：左检测画面、右三维俯视、底部中文状态栏 |
| 退出安全 | timeout/Ctrl+C 后 UART 强制上锁生效（`飞控已上锁`） |

## ⚠️ 重要注意事项

### 1. 安全要求
- **必须拆桨调试**：`arm:=true` 走台架强制解锁（21196），电机会转
- **紧急停止**：Ctrl+C 经 UART 强制上锁，不依赖 MAVROS 存活

### 2. 环境依赖
- **例程 8**：GS130W + Stereonet 已跑通
- **例程 9**：`ego_ws` 已编译且动态目标补丁在位（`setup.sh` 会检查）
- **EGO 补丁**：未打补丁时动态目标（本例程每 0.5 s 下发 goal）会触发上游
  `already been added to an executor` abort —— 补丁由例程 9 的
  `setup_full_ego.sh` 自动应用

### 3. 常见问题

| 现象 | 处理 |
|------|------|
| `ego_planner 包不可用` | `bash setup.sh`（或例程 9 `EGO_FORCE_CLEAN=1 bash setup_full_ego.sh`） |
| EGO 收到 goal 即崩（executor abort） | 动态目标补丁缺失，重跑例程 9 `setup_full_ego.sh` |
| 检测不到行人 | 距离 0.5–5 m 内光照充足；`min_score:=0.2` 放宽（默认 0.25） |
| 一直「搜索行人」但画面正常 | 已修复：`origin_left_image` 是 NV12 编码，须用 `image_msg_to_bgr` 解码（直接 `cv_bridge` 转 bgr8 会静默失败，YOLO 拿不到帧） |
| 目标短暂丢失就停 | 正常：1 s 记忆保持，超时回「搜索行人…」悬停 |
| HUD 无窗口（SSH） | 正常：无 DISPLAY 自动写快照 `/tmp/target_follow_snapshot.jpg`，或显式 `snapshot:=` |
| Stereonet 深度全 0 / 无深度 | 见例程 8 README：`postprocess=v2.3`、`uncertainty_th=-0.09`、右目 `P[0,3]=+fx·B`；先 `bash .../08_depth_camera/run.sh` 验证 |
| Stereonet 日志报 `top is not left image` | 偶发 mipi 帧序告警，深度仍以 15 fps 发布，不影响跟随 |

## 🔍 故障排除

```bash
# EGO 状态
source /opt/tros/humble/setup.bash
source /app/zettatree_demo/09_depth_nav/ego_ws/install/setup.bash
ros2 pkg prefix ego_planner
ros2 node info /ego_planner_node

# 动态目标补丁在位检查（应输出找到）
grep -c 'ZETTATREE: dynamic goal replan' \
  /app/zettatree_demo/09_depth_nav/ego_ws/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp

# 实时话题
ros2 topic hz /move_base_simple/goal     # ego 模式跟随中应约 2 Hz
ros2 topic hz /position_cmd              # 轨迹执行中应 100 Hz
ros2 topic echo /drone/follow/target     # 行人 3D 位置
```

## 🚀 开发机侧部署

例程目录下的 `deploy_and_debug.py` 从开发机一键完成（详见上文脚本说明）。
登录信息用环境变量传入，不要把板卡密码写进仓库：

```bash
export ONBOARD_HOST=<X5_IP>
export ONBOARD_USER=sunrise
export ONBOARD_PASS=<password>
python 10_target_follow/deploy_and_debug.py --check-only
```

Windows PowerShell：

```powershell
$env:ONBOARD_HOST="<X5_IP>"
$env:ONBOARD_USER="sunrise"
$env:ONBOARD_PASS="<password>"
python 10_target_follow/deploy_and_debug.py --check-only
```

其余例程把本仓库同步到机载 `/app/zettatree_demo` 即可（`rsync` / `scp` / `git clone`）。

