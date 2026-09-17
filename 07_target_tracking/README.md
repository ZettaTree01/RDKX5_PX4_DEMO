# 07 停机坪 H 标对准降落

## 例程说明

文档 4.2。**普通 USB 摄像头**（默认 `/dev/video0`，与例程 03–06 相同）。
起飞完成后**悬停搜索**，识别直升机停机坪 **H 标**，把 H
对准画面中心并保持约 1 秒后，请求降落：台架下降后强制上锁停转；
实飞切 `AUTO.LAND`。本例程不用 GS130W / MIPI。

H 标不在 COCO YOLO 80 类里，本例程用轮廓 + H 模板相关、圆形 ROI 识别。
图像断流 0.5 秒后悬停、不降落。

室内无 GPS 时 launch **默认启用台架位姿模拟**（`bench:=true`），否则飞控拒绝解锁。
解锁参数与 05/06/08 共用 `_common/offboard_manager.py`（关磁罗盘、放宽 IMU
一致性、姿态设定点进 OFFBOARD 再强制解锁）。必须拆桨；上电后仍拒解锁时按下安全开关。
`Ctrl+C` 时 `run.sh` 会经 UART 再强制上锁；电机仍转时可手动执行：
`python3 /app/zettatree_demo/_common/emergency_disarm.py`。

摄像头朝下，或把 H 正对镜头：偏右则右移，偏下则后移；框偏小则下降，偏大则上升。

## 本节目标

- 完成「起飞悬停 → 识别 H → 对准 → 请求降落上锁」流程。
- 理解图像偏差到机体前后/左右/升降速度的映射。
- 在板端画面确认相对 H 的前/后/左/右/升/降指示。
- 未检测到 H 或图像断流时悬停，不自动降落。

> **以下命令为前台常驻，需另开终端做其它操作。**

```bash
# 室内拆桨：起飞悬停，对准 H 标后降落
bash /app/zettatree_demo/07_target_tracking/run.sh arm:=true
```

> 必须拆桨。不传 `arm:=true` 电机不会转。把打印的 H 标放到镜头前即可验证对准。

实飞（有位置源，摄像头朝下看停机坪，不要台架）：

```bash
bash /app/zettatree_demo/07_target_tracking/run.sh \
  arm:=true bench:=false altitude:=2
```

## 画面输出

launch 参数 `show` 默认 `true`：

- 左上角显示相对高度与阶段：`等待解锁` / `起飞中` / `悬停搜索` / `对准 H 标` / `已对准，降落`
- 识别到 H 时画框，并显示**相对 H 标**的位移：前/后/左/右/上升/下降（文字 + 箭头）
- 纯 SSH 快照：`/tmp/tracking_snapshot.jpg`

```bash
scp sunrise@<X5_IP>:/tmp/tracking_snapshot.jpg .
```

## 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `fcu_url` | `/dev/ttyS2:57600` | X5↔飞控 40PIN UART2 串口及波特率 |
| `arm` | `false` | `true` 才切 OFFBOARD 并解锁（电机才会转） |
| `bench` | `true` | 室内台架位姿模拟 + 写 EKF 外部视觉参数 |
| `altitude` | `0.1` | 起飞高度（米）；室内默认实飞 2 m 的 1/20 |
| `camera_source` | `usb` | 普通 USB 摄像头（本例程默认） |
| `camera_device` | `/dev/video0` | USB 设备节点 |
| `show` | `true` | 对准画面输出（弹窗/快照） |
| `max_vel` | `0.05` | 对准平移速度上限（m/s） |

## 仅运行本节点

```bash
source /app/zettatree_demo/_common/env.sh
python3 /app/zettatree_demo/07_target_tracking/target_tracking.py
```
