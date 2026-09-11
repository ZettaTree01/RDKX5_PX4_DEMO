#!/bin/bash
# 飞行例程共用启动辅助：Ctrl+C / 异常退出后强制上锁停转。
#
# 用法：在 02/05/06/07/08 的 run.sh 里，source env.sh 之后：
#   source /app/zettatree_demo/_common/run_flight.sh
#   ros2 launch ...   # 注意不要用 exec，否则 trap 不会执行
#
# 陷阱在 EXIT / INT / TERM 时调用 emergency_disarm.py（直连 UART，
# 不依赖此时是否还活着 MAVROS）。先 sleep 片刻，让 launch 释放 /dev/ttyS2。

_DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_emergency_disarm() {
  # 等 MAVROS / 子进程释放串口后再上锁
  sleep 0.8
  python3 "$_DEMO_ROOT/_common/emergency_disarm.py" || true
}

trap _emergency_disarm EXIT INT TERM
