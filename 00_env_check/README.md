# 00 环境一键体检 / 修复器

**目标：执行成功后，直接进入 01~11 例程验证和开发阶段。**

```bash
cd /app/zettatree_demo/00_env_check
bash run.sh --yes
```

首次执行建议用 `--yes`，它会自动完成：

- Ubuntu 基础、编译、Python、OpenCV、GUI 依赖；
- ROS2 Humble / TogetheROS、RViz2、tf2、image_transport、PCL、CycloneDDS、`mavros_msgs`；
- **MAVROS 节点**：apt 有 `ros-humble-mavros` 则装；jammy/arm64 常无该 deb 时，自动运行 `setup_mavros.sh` 源码编译到 `mavros_ws/`；
- 在当前 apt 源中真实存在的 RDK/TogetheROS `mipi_cam` / `hobot_stereonet` / DNN 包；
- colcon / git / C++ EGO-Planner 编译环境；
- 09/10 共用的完整 C++ EGO-Planner（自动拉取、补丁、Release 编译）；
- 当前用户 `dialout` / `i2c` 权限组；
- `~/.bashrc` 中的 ROS2/TROS（及 mavros_ws overlay）环境；
- ROS2 Python import、MAVROS、RViz、GS130W mipi_cam、StereoNet；
- YOLO BPU 模型；
- 02~11 launch 的参数解析级启动检查。

## 模式

### 一键修复 + 完整安装

```bash
bash run.sh --yes
```

### 交互安装

```bash
bash run.sh
```

### 只检查

```bash
bash run.sh --check-only
```

### 暂时跳过 EGO

```bash
bash run.sh --yes --skip-ego
```

> `--skip-ego` 只适合先验证 01~08/11。09、10 的完整 C++ EGO 模式仍需后续运行 `setup_full_ego.sh`。

## 成功标准

最后出现：

```text
READY：软件/ROS/RDK 运行环境已准备完成，可以进入例程验证与开发阶段。
```

此时：

```bash
ros2 --version
rviz2 --version
ros2 pkg prefix mavros          # apt 或 source .../mavros_ws/install
ros2 pkg prefix mipi_cam
ros2 pkg prefix hobot_stereonet
ros2 pkg prefix ego_planner
```

若 MAVROS 仅源码安装，确认已：

```bash
source /opt/tros/humble/setup.bash
source /app/zettatree_demo/mavros_ws/install/setup.bash
ros2 pkg prefix mavros
```

均应可用。

源码编译注意：

- 编译前尽量停掉 Stereonet / mipi_cam 等重负载（同机 `-j2` 易 OOM 卡死）；默认 `MAVROS_JOBS=1`。
- `setup_mavros.sh` 会在系统 `ros-humble-mavlink` 尚无 `MAV_AUTOPILOT::FLIX` 时自动剥离相关代码，避免 `uas_stringify.cpp` 编译失败。

如果当前旧终端仍然提示 `ros2: command not found`，执行：

```bash
source /opt/tros/humble/setup.bash
```

或重新打开一个终端。脚本会把 ROS2/TROS 环境写入 `~/.bashrc`，新终端自动生效。当前终端若要立即生效，可执行 `source /app/zettatree_demo/00_env_check/activate.sh`。

## 硬件 FAIL 的含义

以下项目不能由软件脚本凭空创建：

- `/dev/ttyS2`：飞控 UART2；
- `/dev/i2c-5`：I2C 设备；
- `/dev/video*`：USB 摄像头；
- GS130W 实体相机；
- 飞控与实际 MAVLink 链路。

没有接硬件时，它们显示 FAIL 是正常的；**只要软件/RDK 项没有 BLOCKED，就可以进入对应的软件/仿真例程。**

## 安全

环境脚本只安装和验证软件，不会：

- 自动连接/控制飞控；
- 自动 ARM；
- 自动切 OFFBOARD；
- 自动启动电机。

首次验证飞行例程仍必须拆桨。
