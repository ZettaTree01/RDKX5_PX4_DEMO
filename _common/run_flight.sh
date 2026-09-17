#!/usr/bin/env bash
# 飞行例程共用启动辅助：Ctrl+C / 异常退出后停栈并强制上锁。
#
# 用法：在 run.sh 里 source env.sh 之后：
#   source /app/zettatree_demo/_common/run_flight.sh
#   _flight_run ros2 launch ...   # 推荐：可中断；勿对 launch 用 exec
#   # 或仍可直接前台 ros2 launch（兼容旧写法，但 Ctrl+C 可能被 launch 吞掉）
#
# 可选：导出 FLIGHT_STOP_STACK=0 跳过视觉/导航杀进程（仅上锁）。
# 依赖：同目录 stop_nav_stack.sh、emergency_disarm.py。

_DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_FLIGHT_EXITING=0
_FLIGHT_CHILD_PID=0

_flight_kill_child() {
  # 立刻干掉 launch 进程组，避免 ros2 launch 优雅退出卡死
  local pid="${_FLIGHT_CHILD_PID:-0}"
  _FLIGHT_CHILD_PID=0
  if [ "$pid" = "0" ] || [ -z "$pid" ]; then
    return 0
  fi
  # setsid 启动时 pid 即进程组首领
  kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  sleep 0.4
  kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

_flight_exit_cleanup() {
  # 防止 EXIT+INT 重入把清理跑两遍
  local sig="${1:-EXIT}"
  if [ "$_FLIGHT_EXITING" = "1" ]; then
    return 0
  fi
  _FLIGHT_EXITING=1
  trap - EXIT INT TERM

  echo "[run_flight] 收到退出信号（$sig），清理进程…" >&2

  _flight_kill_child

  # 先停导航/深度/MAVROS，释放 /dev/ttyS2（限时，避免卡死）
  if [ "${FLIGHT_STOP_STACK:-1}" != "0" ]; then
    timeout 8 bash "$_DEMO_ROOT/_common/stop_nav_stack.sh" \
      >/tmp/zettatree_stop_nav.log 2>&1 || true
  else
    sleep 0.3
  fi

  # 紧急上锁：严格限时，串口被占时也不要拖住 shell
  timeout 6 python3 "$_DEMO_ROOT/_common/emergency_disarm.py" \
    --retries 2 >/tmp/zettatree_disarm.log 2>&1 || true

  echo "[run_flight] 清理结束" >&2
  # INT/TERM 时显式退出，避免 wait 继续挂住
  case "$sig" in
    INT) exit 130 ;;
    TERM) exit 143 ;;
  esac
}

# 以前台可中断方式跑命令（例程 09/10 必用）
_flight_run() {
  if command -v setsid >/dev/null 2>&1; then
    setsid "$@" &
  else
    "$@" &
  fi
  _FLIGHT_CHILD_PID=$!
  # wait 被信号打断时返回 >128；清理由 trap 负责
  wait "$_FLIGHT_CHILD_PID" || true
  local rc=$?
  _FLIGHT_CHILD_PID=0
  return "$rc"
}

# 注册退出陷阱：正常结束、Ctrl+C、kill 均走清理
trap '_flight_exit_cleanup EXIT' EXIT
trap '_flight_exit_cleanup INT' INT
trap '_flight_exit_cleanup TERM' TERM
