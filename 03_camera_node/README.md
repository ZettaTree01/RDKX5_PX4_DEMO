# 03 摄像头节点

## 例程说明

文档 3.1。发布 `/camera/image_raw`（bgr8，约 30fps），并直接输出摄像头画面。

节点脚本是共享组件 `_common/camera_node.py`，本目录是它的**示例用法**；
04/05/07 的 launch 也会直接拉起同一个节点（launch 拉起时传 `--no-show`，只发话题）。

## 本节目标

- 掌握 USB 摄像头采集与 ROS2 图像话题发布。
- 在板端以弹窗或快照方式查看画面。
- 理解图像话题 QoS（`BEST_EFFORT` / `RELIABLE`）差异，以及 `ros2 topic hz` 须显式指定的原因。
- 认识共享组件与示例用法的组织方式；例程 4/5/7 会复用同一相机入口。

## 画面输出

例程 **03/04/05** 统一使用**普通 USB 摄像头**（默认 `/dev/video0`），
经 `_common/camera_node.py` 发布 `/camera/image_raw`。不是 GS130W / MIPI。

`run.sh` 默认带 `--show`，按当前环境自动选择输出方式：

| 环境 | 行为 |
|---|---|
| 板端接 HDMI 显示器，在桌面终端运行；或开发机 `ssh -X` 登录 | **弹窗显示实时画面**，窗口内按 `q` / `Esc` 退出 |
| 纯 SSH，无显示环境 | 自动**回退为快照输出**：每 5 秒把最新一帧写到 `/tmp/camera_snapshot.jpg`，日志打印路径 |

```bash
# 默认：弹窗显示画面（无显示环境时自动改快照）
bash /app/zettatree_demo/03_camera_node/run.sh

# 指定摄像头设备
bash /app/zettatree_demo/03_camera_node/run.sh --device /dev/video1

# 只发话题、不输出画面
bash /app/zettatree_demo/03_camera_node/run.sh --no-show

# 主动指定快照输出（不弹窗，每 2 秒刷新一次）
bash /app/zettatree_demo/03_camera_node/run.sh --no-show \
  --snapshot /app/zettatree_demo/03_camera_node/snapshot.jpg --snapshot-period 2
```

快照写到板端后，在开发机拉回查看：

```bash
scp sunrise@<X5_IP>:/tmp/camera_snapshot.jpg .
```

## 话题验证

**另开终端验证出图**（图像是 `BEST_EFFORT` QoS，需显式指定）：

```bash
source /app/zettatree_demo/_common/env.sh
ros2 topic hz /camera/image_raw --qos-reliability best_effort
```

> MIPI CSI / GS130W（接口 5）不走本例程；深度与双目见例程 8/9/10。
