#!/usr/bin/env bash
# 例程10：环境配置（板端执行一次）
# EGO-Planner 的拉取 / 补丁 / 编译全部复用例程 09 的 setup_full_ego.sh，
# 例程 10 与 09 共享同一个 ego_ws，装一次两边可用。
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

EGO_SETUP="/app/zettatree_demo/09_depth_nav/setup_full_ego.sh"
EGO_WS="${EGO_WS:-/app/zettatree_demo/09_depth_nav/ego_ws}"

if [ -f "$EGO_WS/install/setup.bash" ] \
   && grep -q "ZETTATREE: dynamic goal replan" \
      "$EGO_WS/src/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp" 2>/dev/null; then
    echo "[10] EGO 工作空间就绪（含动态目标补丁），跳过编译"
else
    echo "[10] 首次使用：拉取并编译完整 EGO-Planner（10-20 分钟）…"
    bash "$EGO_SETUP"
fi

# GS130W 深度相机 / BPU 就绪检查
echo "[10] 确保深度相机（例程8 ensure）…"
bash /app/zettatree_demo/08_depth_camera/ensure_mipi_bpu.sh

echo "[10] 目标跟随环境配置完成。运行：bash ${SCRIPT_DIR}/run.sh arm:=true"
