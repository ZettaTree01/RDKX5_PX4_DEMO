#!/usr/bin/env bash
# 当前终端立即进入 zettatree_demo 开发环境（需 source，不可直接 bash 执行后指望变量留存）。
# 唯一入口：地平线 TogetheROS Humble + mavros/ego overlay（见 _common/env.sh）。
_ZT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$_ZT_ROOT/_common/env.sh" || return 1 2>/dev/null || exit 1
if ros2 -h >/dev/null 2>&1; then
  echo "[env] READY: TROS=${ROS_DISTRO:-humble} RMW=${RMW_IMPLEMENTATION:-ROS default}"
else
  echo "[env] ERROR: TROS 已加载但 ros2 不可用。请先运行 00_env_check/run.sh --yes" >&2
  return 1 2>/dev/null || exit 1
fi
