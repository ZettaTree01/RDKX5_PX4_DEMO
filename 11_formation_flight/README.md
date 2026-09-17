# 11 编队飞行

## 例程说明

文档 4.6。每架飞机运行一套 MAVROS、OFFBOARD 管理器和编队节点。
各机交换相对启动位置的位移，不直接共用不同飞控的 local 原点。
所有飞机需有一致的 ENU 朝向并处于同一 ROS Domain。

室内无 GPS 时 launch **默认启用台架位姿模拟**（`bench:=true`），否则飞控拒绝解锁。
解锁与 05/06/07 共用 `_common/offboard_manager.py`（强制解锁）：先爬升拉转速，
无队形指令时悬停保持转速，有位置指令再加速，最高 **300 r/min**。必须拆桨。

## 本节目标

- 确认多机前提：同一 ROS Domain、ENU 朝向一致、各机 `drone_id` 唯一。
- 理解为何交换「相对起飞点的位移」，而不可直接混用各飞控 local 原点。
- 掌握领队-僚机拓扑与期望队形的设定方法。

> **每架飞机各跑一条命令（各机处于同一 ROS Domain）；`drone_id` 每机不同，0 为领队。**

```bash
# 室内拆桨，每架飞机上执行；领队用 drone_id:=0，其余 1、2…
bash /app/zettatree_demo/11_formation_flight/run.sh arm:=true drone_id:=0 num_drones:=3
```

> 必须拆桨。不传 `arm:=true` 电机不会转。单机台架验证可用 `drone_id:=0 num_drones:=1`。

实飞（有位置源，不要台架）：

```bash
bash /app/zettatree_demo/11_formation_flight/run.sh \
  arm:=true bench:=false altitude:=2 drone_id:=0 num_drones:=3
```

编队节点会等 `/drone/status/airborne` 变为 `true` 后才开始规划。
台架解锁后管理器先爬升约 2 秒，再把 airborne 置为 true；悬停保持转速后进入队形。

## 启动参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `fcu_url` | `/dev/ttyS2:57600` | X5↔飞控 40PIN UART2 串口及波特率 |
| `arm` | `false` | `true` 才切 OFFBOARD 并解锁（电机才会转） |
| `bench` | `true` | 室内台架位姿模拟 + 写 EKF 外部视觉参数 |
| `altitude` | `0.1` | 起飞高度（米）；室内默认实飞 2 m 的 1/20 |
| `drone_id` | `0` | 本机编号，0 为领队（每机不同） |
| `num_drones` | `3` | 编队飞机数 |

## 仅运行本节点

```bash
source /app/zettatree_demo/_common/env.sh
python3 /app/zettatree_demo/11_formation_flight/formation_flight.py --drone-id 0 --num-drones 3
```
