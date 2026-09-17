# 05 摄像头识别避障

## 例程说明

文档 3.2。**普通 USB 摄像头**（默认 `/dev/video0`）发布 `/camera/image_raw`，
BPU YOLO 检测障碍并估算最近距离，按 **「越近退得越快」** 向 OFFBOARD 管理器发机体 FLU 反向速度。
本例程**不用深度相机 / GS130W**（深度链路见例程 8/9）。图像断流 0.5 秒后零速。

距离取较小值：小孔成像（类别高度 / 框高）与画面占比（框高占半屏 ≈ 0.8 m）。
反向速度与距离成比例：刚进入安全区约 20% 最大速度，贴脸时到 100%。
障碍在画面偏左则向右让、偏下则上升。检测画面保持干净（仅框与避障箭头）；
高度、阶段、距离、YOLO 状态集中在**底部中文状态栏**。室内暗场会先增强再推理。

室内无 GPS 时 launch **默认启用台架位姿模拟**（`bench:=true`），否则飞控拒绝解锁。
台架会强制解锁；先爬升拉转速，无障碍时悬停保持转速，靠近障碍再按距离加速。必须拆桨。
`Ctrl+C` 会强制上锁；电机还在转：`python3 /app/zettatree_demo/_common/emergency_disarm.py`。

## 为什么电机不转 / 不避障

1. 不传 `arm:=true` 是监视模式，**不会解锁，电机不会转**。
2. 室内无 GPS 必须 `bench:=true`（默认已开）。台架会**强制解锁**；先爬升拉转速，无避障时悬停保持转速，靠近障碍按距离加速。
3. 台架解锁后先爬升约 2 秒再悬停；实飞则等 `/drone/status/airborne` 为 true 后转发避障速度。

```bash
# 室内拆桨：台架位姿 + 低油门解锁 + 按距离后退
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh arm:=true
```

> 必须拆桨。解锁后电机会先拉高再落到悬停转速，靠近障碍会再加速，最高 **300 r/min**。

不解锁只看识别：

```bash
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh
```

实飞（有位置源，不要台架）：

```bash
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh \
  arm:=true bench:=false altitude:=2 max_vel:=0.5
```

## 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `fcu_url` | `/dev/ttyS2:57600` | X5↔飞控 40PIN UART2 串口及波特率 |
| `arm` | `false` | `true` 才切 OFFBOARD 并解锁（电机才会转） |
| `bench` | `true` | 室内台架位姿模拟 + 写 EKF 外部视觉参数 |
| `altitude` | `0.1` | 起飞高度（米）；室内默认实飞 2 m 的 1/20 |
| `camera_source` | `usb` | 普通 USB 摄像头；不要用深度相机 |
| `camera_device` | `/dev/video0` | USB 设备节点 |
| `show` | `true` | 避障画面输出（弹窗/快照） |
| `hfov` | `90.0` | USB 广角默认 90°（60° 会把近处目标估远） |
| `safe_distance` | `4.0` | 开始按比例后退的距离（米）；越近退得越快 |
| `max_vel` | `0.025` | 避障反向速度上限（m/s） |
| `score_thres` | `0.25` | 置信度阈值 |
| `nms_thres` | `0.45` | NMS IoU 阈值 |
| `infer_hz` | `10.0` | YOLO 推理频率（Hz） |

画面底部状态栏显示**相对开机高度**（室内气压计常漂到几十米，不再直接显示飞控绝对值）
与阶段（`等待解锁` / `起飞中` / `悬停`）、最近障碍距离、检测数与 YOLO 状态。
触发避障时显示位移方向，例如 `位移 后、左、上升`，画面上保留对应箭头。
室内欠曝时自动增强后再推理。纯 SSH 快照：`/tmp/avoid_snapshot.jpg`。
