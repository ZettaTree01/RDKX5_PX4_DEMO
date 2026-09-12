# 02：RDK X5 机载例程源码

面向 **多旋翼无人机 + PX4**：例程部署在 RDK X5 机载计算机 `/app/zettatree_demo`，飞行指令经 MAVROS 发给飞控。

ROS2 例程用 `run.sh`（会 `source` TogetheROS）。

每个例程一个目录，自带 `run.sh` / launch / 配置；被多个例程复用的运行时组件统一放在 `_common/`。

## 目录

| 目录 | 文档章节 | 说明 | 硬件依赖 |
|------|----------|------|----------|
| `_common` | 2.7 / 3.1 | 共享组件：MAVROS 插件清单、OFFBOARD 管理器、相机、YOLO、室内限速 | 无 |
| `00_env_check` | — | 环境自检 | 无 |
| `01_uart_serial` | 1.2.7 | 例程1：40PIN 针脚串口（机载↔飞控）读姿态 / 拆桨电机测试 | 飞控 TELEM + 杜邦线 |
| `02_bench_pose_sim` | 2.8 | 例程2：台架位姿模拟器（室内无 GPS、拆桨验证用） | 飞控，**拆桨** |
| `03_camera_node` | 3.1 | 例程3：USB 摄像头发布（弹窗 / 快照查看画面） | `/dev/video0` |
| `04_object_detection` | 3.1 | 例程4：检测节点 + 官方 YOLO（推理画面） | BPU 模型；相机可选 |
| `05_obstacle_avoidance` | 3.2 | 例程5：摄像头识别避障（单目估距） | 摄像头 + BPU + 飞控 |
| `06_autonomous_cruise` | 4.1 | 例程6：自主巡航拍照（巡航画面） | 飞控 + 摄像头 |
| `07_target_tracking` | 4.2 | 例程7：停机坪 H 标对准降落 | 摄像头 + 飞控 |
| `08_formation_flight` | 4.3 | 例程8：编队（一机一进程） | 多机 |

## 使用约定

1. 先跑环境自检：`python3 /app/zettatree_demo/00_env_check/env_check.py`
2. `01_uart_serial` 的针脚串口脚本可直接 `python3 xxx.py`（默认端口 `/dev/ttyS2`）
3. ROS2 例程用 `bash /app/zettatree_demo/<例程>/run.sh` 启动，
   参数以 `名称:=值` 透传，例如 `run.sh arm:=true`（室内默认高度 0.1 m）
4. 飞控通信统一走 **40PIN UART2 针脚串口** `/dev/ttyS2:57600`（首次使用先按下文使能 UART2）
5. 首次 OFFBOARD 验证必须拆桨
6. OFFBOARD 管理器在 `_common/offboard_manager.py`，往飞控发设定点只走它；
   任务节点只发 `/drone/setpoint_*`（`05`、`06` 等飞行例程就是这样用的）
7. 管理器只有显式传 `arm:=true` 才切 OFFBOARD 并解锁
8. 室内无 GPS 又必须解锁时，飞行例程默认 `bench:=true`（含台架位姿回灌）；也可单独跑 `02_bench_pose_sim`
9. **室内调试限速**：怠速慢转、按任务加速、最高 **300 r/min**；速度/高度为实飞 1/20。
   无指令时怠速，有避障/跟踪指令时能看出加速。实飞把
   `INDOOR_SPEED_SCALE` 改为 `1.0`，或 launch 显式传 `altitude:=2 max_vel:=0.5`
10. **安全**：必须拆桨调试。`Ctrl+C` / 例程退出后，`run.sh` 会经 UART 强制上锁停转；
    若电机仍转，手动执行：
    `python3 /app/zettatree_demo/_common/emergency_disarm.py`

## 接线与 UART2 使能（首次使用必读）

本套例程飞控链路走 **40PIN 针脚串口 UART2（`/dev/ttyS2`）**，实接三根线
（注意：**该组引脚与 X5 默认 UART1 的 PIN8/PIN10 不同**）：

| X5 40PIN 物理脚 | 功能 | 接到飞控 | 方向 |
|---|---|---|---|
| **PIN20** | GND | GND | 共地 |
| **PIN22** | UART2_RXD | 飞控主板 **TX** | ← 收 |
| **PIN15** | UART2_TXD | 飞控主板 **RX** | → 发 |

UART2 出厂在设备树里是 disabled，**首次使用必须使能并重启**：

```bash
# 1) 备份并修改启动设备树（x5_rdk_v2 对应 x5-rdk-v1p0.dtb，按实际板型选择）
sudo cp /boot/hobot/x5-rdk-v1p0.dtb /boot/hobot/x5-rdk-v1p0.dtb.bak-uart2
sudo fdtput -t s /boot/hobot/x5-rdk-v1p0.dtb /soc/a55_apb0/serial@34080000 status okay

# 2) 重启生效
sudo reboot

# 3) 重启后确认 ttyS2 已出现且可打开
ls -l /dev/ttyS2        # crw-rw---- 1 root dialout 4, 66 ... /dev/ttyS2
python3 /app/zettatree_demo/01_uart_serial/attitude_via_usb.py --duration 5
```

恢复出厂（回到 UART2 禁用）：

```bash
sudo cp /boot/hobot/x5-rdk-v1p0.dtb.bak-uart2 /boot/hobot/x5-rdk-v1p0.dtb && sudo reboot
```

> 板型/设备树对照（与 `srpi-config` 一致）：`x5_rdk_v1`→`x5-rdk.dtb`，
> `x5_rdk_v2`→`x5-rdk-v1p0.dtb`；MD 系列用 `x5-md-v0p2.dtb`/`x5-md-v1p2.dtb`。
> 不确定时用 `cat /sys/firmware/devicetree/base/model` 与各 dtb 的 `model` 属性比对。

飞控侧要求：所接 TELEM 口必须**已开启 MAVLink 实例**（`MAV_x_CONFIG` 指向该 TELEM），
波特率与 `fcu_url` 一致（本教程 57600）。PX4 默认在 TELEM2 输出 MAVLink；若接的是
TELEM1，需要先在 QGC 里把 `MAV_0_CONFIG` 改为对应 TELEM 端口。

## MAVROS 的启动方式

每个飞控例程的 launch 自己拉起 MAVROS，并统一传入 `_common/px4_pluginlists.yaml`
作为插件清单：

- **同一时刻只运行一个飞控例程**：一个串口只能被一路 MAVROS 占用；
- 不要把插件清单 `sudo cp` 到 `/opt/ros/humble/share/mavros/launch/`：
  那里是所有例程共用的安装目录。

调试后备：飞控的 USB 口也可用（`/dev/ttyACM0`，PX4 默认输出 MAVLink），
给例程传 `fcu_url:=/dev/ttyACM0:115200` 即可，例如：

```bash
bash /app/zettatree_demo/06_autonomous_cruise/run.sh fcu_url:=/dev/ttyACM0:115200 arm:=true
```

## 共享组件（`_common/`）

| 文件 | 文档章节 | 说明 | 示例用法 |
|---|---|---|---|
| `env.sh` | — | 板端 ROS2 / TogetheROS 环境 | 各 `run.sh` 自动 `source` |
| `run_flight.sh` | — | 飞行例程退出陷阱：Ctrl+C 后强制上锁 | `02/05/06/07/08` 的 `run.sh` |
| `emergency_disarm.py` | — | 经 UART 强制上锁（不依赖 MAVROS） | 退出陷阱 / 手动补救 |
| `px4_pluginlists.yaml` | — | 全体飞控例程的 MAVROS 插件清单基线 | 各例程 launch 显式传入 |
| `indoor.py` | — | 室内限速（速度 1/20；怠速→加速→最高 300 r/min） | 所有会转电机的例程 |
| `offboard_manager.py` | 2.7 | OFFBOARD 管理器：起飞、降落上锁、设定点仲裁 | `05`–`08`、`02` |
| `yolo_detector.py` | 3.1 / 3.2 | 共享 YOLO 推理：NV12 预处理 + DFL 解码 + NMS | `04_object_detection`、`05_obstacle_avoidance` |
| `helipad_h.py` | 4.2 | 停机坪 H 标识别（轮廓 + H 模板） | `07_target_tracking` |
| `camera_node.py` | 3.1 | USB 摄像头发布 `/camera/image_raw` | `03_camera_node` |
| `frame_output.py` | 3.1 | 共享画面输出器：弹窗 / 快照，无显示环境自动回退 | `camera_node`、`04/06/07` 任务节点 |
| `cn_hud.py` | — | 中文 HUD 叠字 | `05`/`07` 等画面输出 |
