# -*- coding: utf-8 -*-
"""把 02 例程源码同步进 RDK_X5_AI_Tutorial.md 的对应 python 代码块。"""
import re
from pathlib import Path

DOC = Path(r"D:\gs-workspace\RDK_X5_AI_Tutorial.md")
DEMO = Path(r"D:\gs-workspace\02")

# 用代码块开头附近的独特字符串定位（出现在 ```python 之后）
BLOCKS = [
    # unique 须匹配教程里当前代码块中仍存在的字符串（同步前）
    ("PX4 OFFBOARD 管理器（文档 2.7）。",
     "_common/offboard_manager.py"),
    ("摄像头识别避障任务。文档 3.2。",
     "05_obstacle_avoidance/obstacle_avoidance.py"),
    ("自主巡航拍摄。文档 4.1。",
     "06_autonomous_cruise/autonomous_cruise.py"),
    ("停机坪 H 标对准降落。文档 4.2。",
     "07_target_tracking/target_tracking.py"),
    ("编队飞行。文档 4.3。",
     "08_formation_flight/formation_flight.py"),
    ("目标检测 ROS 节点。文档 3.1。",
     "04_object_detection/object_detection.py"),
]

PROSE = [
    ("--value 0.05 --duration 3 --i-am-sure",
     "--value 0.0025 --duration 3 --i-am-sure"),
    ("--value 0.005 --duration 3 --i-am-sure",
     "--value 0.0025 --duration 3 --i-am-sure"),
    ("# 拆桨 + 固定飞机后，全部电机同步转 3 秒（输出值 0.05）",
     "# 拆桨 + 固定飞机后，全部电机同步转 3 秒（室内默认输出值 0.0025，实飞测试 0.05 的 1/20）"),
    ("# 拆桨 + 固定飞机后，全部电机同步转 3 秒（室内默认输出值 0.005，实飞测试 0.05 的 1/10）",
     "# 拆桨 + 固定飞机后，全部电机同步转 3 秒（室内默认输出值 0.0025，实飞测试 0.05 的 1/20）"),
    ("`bash /app/zettatree_demo/06_autonomous_cruise/run.sh arm:=true altitude:=2`；",
     "`bash /app/zettatree_demo/06_autonomous_cruise/run.sh arm:=true`（室内默认高度 0.1 m）；"),
    ("（室内默认高度 0.2 m）",
     "（室内默认高度 0.1 m）"),
    ("bash /app/zettatree_demo/06_autonomous_cruise/run.sh arm:=true altitude:=2",
     "bash /app/zettatree_demo/06_autonomous_cruise/run.sh arm:=true"),
    ("# 不传 arm:=true 只监视（不解锁）；实飞时再加 arm:=true altitude:=2\n"
     "bash /app/zettatree_demo/<例程>/run.sh arm:=true altitude:=2",
     "# 不传 arm:=true 只监视（不解锁）；室内默认高度 0.1 m\n"
     "bash /app/zettatree_demo/<例程>/run.sh arm:=true\n"
     "# 实飞标称值：altitude:=2 max_vel:=0.5"),
    ("# 不传 arm:=true 只监视（不解锁）；室内默认高度 0.2 m\n"
     "bash /app/zettatree_demo/<例程>/run.sh arm:=true",
     "# 不传 arm:=true 只监视（不解锁）；室内默认高度 0.1 m\n"
     "bash /app/zettatree_demo/<例程>/run.sh arm:=true"),
    ("bash /app/zettatree_demo/07_target_tracking/run.sh arm:=true altitude:=2",
     "bash /app/zettatree_demo/07_target_tracking/run.sh arm:=true"),
    ("ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet \\\n"
     "  \"{param_id: 'EKF2_EV_CTRL', value: {integer: 15, real: 15.0}}\"\n"
     "ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet \\\n"
     "  \"{param_id: 'EKF2_EV_DELAY', value: {integer: 5, real: 5.0}}\"",
     "ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \\\n"
     "  \"{force_set: true, param_id: 'EKF2_EV_CTRL', value: {type: 2, integer_value: 15}}\"\n"
     "ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \\\n"
     "  \"{force_set: true, param_id: 'EKF2_EV_DELAY', value: {type: 3, double_value: 5.0}}\""),
    ("| `rate` | `50.0` | 位姿回灌频率（Hz） |",
     "| `rate` | `5.0` | 位姿回灌频率（Hz）；57600 UART 不宜再高 |"),
    ("### 前置：启用外部视觉融合\n\n"
     "飞控需启用外部视觉融合（只改 RAM，重启飞控即恢复默认）。\n"
     "等 launch 起来、MAVROS 连上飞控后，**另开一个终端**执行：",
     "本例程会给管理器传 `--bench`：自动写 `EKF2_EV_CTRL` / `COM_ARM_WO_GPS` /\n"
     "`EKF2_ABL_LIM` 等 RAM 参数，用姿态设定点切 OFFBOARD 后解锁（无遥控不能在\n"
     "STABILIZED 解锁）。一般不用再手写参数。若管理器日志里参数没写上，可另开终端："),
    ("| `altitude` | `2.0` | 起飞高度（米） |\n"
     "| `max_speed` | `2.0` | 模拟器跟随限速（m/s） |",
     "| `altitude` | `0.1` | 起飞高度（米）；室内默认实飞 2 m 的 1/20 |\n"
     "| `max_speed` | `0.1` | 模拟器跟随限速（m/s）；室内默认实飞 2 的 1/20 |"),
    ("| `altitude` | `0.2` | 起飞高度（米）；室内默认实飞 2 m 的 1/10 |\n"
     "| `max_speed` | `0.2` | 模拟器跟随限速（m/s）；室内默认实飞 2 的 1/10 |",
     "| `altitude` | `0.1` | 起飞高度（米）；室内默认实飞 2 m 的 1/20 |\n"
     "| `max_speed` | `0.1` | 模拟器跟随限速（m/s）；室内默认实飞 2 的 1/20 |"),
    ("起飞稳定后，以当前位置为原点飞行 1 m × 1 m 方形，到点拍照，完成后请求`AUTO.LAND`。",
     "起飞稳定后，以当前位置为原点飞行 0.05 m × 0.05 m 方形（室内为实飞 1 m 的 1/20），到点拍照，完成后请求`AUTO.LAND`。"),
    ("起飞稳定后，以当前位置为原点飞行 0.1 m × 0.1 m 方形（室内为实飞 1 m 的 1/10），到点拍照，完成后请求`AUTO.LAND`。",
     "起飞稳定后，以当前位置为原点飞行 0.05 m × 0.05 m 方形（室内为实飞 1 m 的 1/20），到点拍照，完成后请求`AUTO.LAND`。"),
]


def replace_block(text, unique, source_text):
    marker = unique
    idx = text.find(marker)
    if idx < 0:
        raise SystemExit("找不到代码块标记: " + unique[:40])
    fence = text.rfind("```python", 0, idx)
    if fence < 0:
        raise SystemExit("找不到 ```python: " + unique[:40])
    end = text.find("\n```", idx)
    if end < 0:
        raise SystemExit("找不到结束围栏: " + unique[:40])
    return text[:fence] + "```python\n" + source_text.rstrip() + "\n" + text[end:]


def main():
    text = DOC.read_text(encoding="utf-8")
    for unique, rel in BLOCKS:
        src = (DEMO / rel).read_text(encoding="utf-8")
        text = replace_block(text, unique, src)
        print("synced block", rel)

    indoor_note = """
> **室内调试限速**：怠速慢转、按任务加速、最高 **300 r/min**（约 2 秒从静止到最高速）。
> 速度/高度为实飞 1/20。把 `INDOOR_SPEED_SCALE` 改为 `1.0` 即恢复实飞。

"""
    needle = "无论哪个例程，启动方式都差不多，照着模板改参数就行。\n"
    if indoor_note.strip() not in text:
        if needle not in text:
            # 兼容旧措辞
            old_needle = "无论哪个例程，启动方式都遵循同一套模板，照着做即可。\n"
            if old_needle in text:
                text = text.replace(old_needle, needle, 1)
            else:
                raise SystemExit("找不到室内限速插入点")
        text = text.replace(needle, needle + "\n" + indoor_note, 1)
        print("inserted indoor note")

    for old, new in PROSE:
        if old not in text:
            print("prose skip (already changed?):", old[:50])
            continue
        text = text.replace(old, new)
        print("prose ok:", old[:50])

    # 例程5 启动说明
    old5 = """**启动步骤（一条命令，`run.sh` 会自动拉起 MAVROS + OFFBOARD 管理器 + 相机 + 避障节点）**：

```bash
# 不传 arm 只监视（不解锁）；首次必须拆桨
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh arm:=true

# 常用参数：换摄像头 / 校准视场角 / 调触发距离与阈值
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh camera_device:=/dev/video1
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh hfov:=70.0 safe_distance:=1.5
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh score_thres:=0.4 nms_thres:=0.5
```

启动参数：`fcu_url`（默认 `/dev/ttyS2:57600`）、`arm`（默认 `false`）、`altitude`
（默认 `2.0`）、`camera_device`（默认 `/dev/video0`）、`show`（默认 `true`）、
`hfov`（默认 `60.0` 度）、`safe_distance`（默认 `1.0` m）、`score_thres`（默认
`0.25`）、`nms_thres`（默认 `0.45`）。"""
    new5 = """**启动步骤（一条命令，`run.sh` 会自动拉起 MAVROS + OFFBOARD 管理器 + 相机 + 避障节点）**：

```bash
# 室内监视（不解锁），确认识别与速度话题
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh

# 室内拆桨后解锁（高度/速度已是实飞的 1/20）
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh arm:=true

# 常用参数：换摄像头 / 校准视场角 / 调触发距离与阈值
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh camera_device:=/dev/video1
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh hfov:=70.0 safe_distance:=1.5
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh score_thres:=0.4 nms_thres:=0.5

# 实飞标称值
bash /app/zettatree_demo/05_obstacle_avoidance/run.sh arm:=true altitude:=2 max_vel:=0.5
```

启动参数：`fcu_url`（默认 `/dev/ttyS2:57600`）、`arm`（默认 `false`）、`altitude`
（默认 `0.1`，室内为实飞 2 m 的 1/20）、`camera_device`（默认 `/dev/video0`）、
`show`（默认 `true`）、`hfov`（默认 `60.0` 度）、`safe_distance`（默认 `1.0` m）、
`max_vel`（默认 `0.025` m/s，室内为实飞 0.5 的 1/20）、`score_thres`（默认
`0.25`）、`nms_thres`（默认 `0.45`）、`infer_hz`（默认 `10` Hz，与图像帧率解耦）。"""
    if old5 in text:
        text = text.replace(old5, new5)
        print("updated 05 launch prose")
    else:
        print("05 launch prose not matched, check manually")

    old_overlay = (
        "`AVOIDING nearest x.x m yaw +x.xxrad` 或 `clear nearest x.x m`；纯 SSH 自动回退为快照输出，每 5 秒写")
    new_overlay = (
        "`AVOIDING x.x m yaw +x.xx vx=±x.xx vy=±x.xx` 或 `clear nearest x.x m`；"
        "纯 SSH 自动回退为快照输出，每 5 秒写")
    if old_overlay in text:
        text = text.replace(old_overlay, new_overlay)
        print("updated overlay text")

    dir_old = """├── 05_obstacle_avoidance        # 例程5：摄像头识别避障（单目估距）
└── 06_autonomous_cruise ... 08_formation_flight
```"""
    dir_new = """├── 05_obstacle_avoidance        # 例程5：摄像头识别避障（单目估距）
├── 06_autonomous_cruise ... 08_formation_flight
└── _common/                    # 含 indoor.py：室内调试把电机/速度压到 1/20
```"""
    if dir_old in text:
        text = text.replace(dir_old, dir_new)
        print("updated directory listing")

    DOC.write_text(text, encoding="utf-8")
    print("wrote", DOC)


if __name__ == "__main__":
    main()
