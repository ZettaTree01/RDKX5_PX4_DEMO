# 06 自主巡航拍摄

## 例程说明

文档 4.1。**普通 USB 摄像头**（默认 `/dev/video0`，与例程 03–05/07 相同）
发布 `/camera/image_raw`。起飞稳定后沿起飞机头巡航 **0.05 m × 0.05 m** 方形（先向前、再向左；室内为实飞 1 m 的 1/20），
到点拍照，完成后通过 OFFBOARD 管理器请求 `AUTO.LAND`。飞行过程中持续输出
**巡航画面**（叠加航点进度）。本例程不用 GS130W / MIPI。

室内无 GPS 时 launch **默认启用台架位姿模拟**（`bench:=true`），否则飞控拒绝解锁。
解锁与 05/07/08 共用 `_common/offboard_manager.py`（强制解锁）：先爬升拉转速，
无航点时悬停保持转速，巡航时再按航点加速，最高 **600 r/min**。必须拆桨。

## 本节目标

- 完成「起飞 → 巡航 → 到点拍照 → 降落」任务编排。
- 理解任务节点与 OFFBOARD 管理器的分工（含 `AUTO.LAND` 请求）。
- 将相机采集与航点任务结合，形成可复用的作业流程。
- 通过巡航画面确认相机与航点进度。

> **以下命令为前台常驻，需另开终端做其它操作。**

```bash
# 室内拆桨：台架位姿 + 低油门解锁 + 巡航方形
bash /app/zettatree_demo/06_autonomous_cruise/run.sh arm:=true
```

> 必须拆桨。解锁后电机会先拉高再落到悬停转速，进入航点后会加速，最高 **600 r/min**。

不解锁只看画面：

```bash
bash /app/zettatree_demo/06_autonomous_cruise/run.sh
```

实飞（有位置源，不要台架）：

```bash
bash /app/zettatree_demo/06_autonomous_cruise/run.sh \
  arm:=true bench:=false altitude:=2
```

巡航节点会等 `/drone/status/airborne` 变为 `true` 后才规划航点。
台架解锁后管理器先爬升约 2 秒，再把 airborne 置为 true；悬停保持转速后开始巡航。

抓拍用普通 USB 摄像头（默认 `/dev/video0`）；换设备时：

```bash
bash /app/zettatree_demo/06_autonomous_cruise/run.sh arm:=true device:=/dev/video1
```

## 巡航画面输出

launch 参数 `show` 默认 `true`，按当前环境自动选择输出方式：

- **有显示环境**（HDMI 显示器桌面终端或 `ssh -X`）：弹窗显示巡航画面，
  叠加当前状态（相对高度、`等待解锁` / `起飞中` / `航点 2/5` 等）；窗口内按 `q` / `Esc` 只关闭画面，
  **不退出节点、不影响飞行**；
- **纯 SSH 无显示环境**：自动回退为快照输出，每 5 秒把巡航画面写到
  `/tmp/cruise_snapshot.jpg`，开发机拉回查看：

```bash
scp sunrise@<X5_IP>:/tmp/cruise_snapshot.jpg .
```

航点到位时另存抓拍原图 `/tmp/capture_<时间戳>.jpg`。

## 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `fcu_url` | `/dev/ttyS2:57600` | X5↔飞控 40PIN UART2 串口及波特率 |
| `arm` | `false` | `true` 才切 OFFBOARD 并解锁（电机才会转） |
| `bench` | `true` | 室内台架位姿模拟 + 写 EKF 外部视觉参数 |
| `altitude` | `0.1` | 起飞高度（米）；室内默认实飞 2 m 的 1/20 |
| `camera_source` | `usb` | 普通 USB 摄像头（本例程默认） |
| `device` | `/dev/video0` | USB 设备节点 |
| `show` | `true` | 巡航画面输出（弹窗/快照，无显示环境自动回退） |

## 仅运行本节点

MAVROS 与管理器已经由别的终端提供时，可以只跑巡航节点：

```bash
source /app/zettatree_demo/_common/env.sh
python3 /app/zettatree_demo/06_autonomous_cruise/autonomous_cruise.py --device auto
```

> 管理器脚本是共享组件 `_common/offboard_manager.py`，由本例程的 launch 直接拉起。
