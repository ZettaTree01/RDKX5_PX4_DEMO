#!/usr/bin/env bash
# 在 RDK X5（Humble/TROS）拉取并编译完整 EGO-Planner（ZJU ego-planner-swarm ros2_version）
# 参考：https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version
set -eo pipefail
# 不要 set -u：TROS setup.bash 会读未定义变量（AMENT_TRACE_SETUP_FILES）

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
EGO_WS="${EGO_WS:-$SCRIPT_DIR/ego_ws}"
EGO_SRC="$EGO_WS/src/ego-planner-swarm"
EGO_REPO="${EGO_REPO:-https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git}"
EGO_BRANCH="${EGO_BRANCH:-ros2_version}"

if [ -f /opt/tros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/tros/humble/setup.bash
else
  echo "[ego] 未找到地平线 TogetheROS Humble：/opt/tros/humble/setup.bash" >&2
  exit 1
fi


# 编译 EGO 所需系统库与 ROS 包（缺省时 apt 失败不中断）
# 编译 EGO 所需系统库与 ROS 包（缺省时 apt 失败不中断）
echo "[ego] 安装依赖…"
sudo apt-get update -qq || true
sudo apt-get install -y \
  libarmadillo-dev \
  libeigen3-dev \
  libpcl-dev \
  ros-humble-pcl-ros \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rmw-cyclonedds-cpp \
  || true

# ---------- 清理错误布局（避免 Duplicate package names）----------
# 常见误操作：把仓库直接 clone 到 ego_ws 根，或同时存在 src/planner 与
# src/ego-planner-swarm/...，colcon 会报重复包名。
if [ "${EGO_FORCE_CLEAN:-0}" = "1" ]; then
  echo "[ego] EGO_FORCE_CLEAN=1，清空 $EGO_WS"
  rm -rf "$EGO_WS"
fi

mkdir -p "$EGO_WS/src"
# 去掉误放在 ego_ws 根下的上游树
for junk in planner uav_simulator pictures .vscode; do
  if [ -e "$EGO_WS/$junk" ]; then
    echo "[ego] 移除误放：$EGO_WS/$junk"
    rm -rf "$EGO_WS/$junk"
  fi
done
# 去掉重复的平铺/嵌套副本（保留唯一 clone：src/ego-planner-swarm）
for junk in \
  "$EGO_WS/src/planner" \
  "$EGO_WS/src/uav_simulator" \
  "$EGO_WS/src/bspline_opt" \
  "$EGO_WS/src/drone_detect" \
  "$EGO_WS/src/path_searching" \
  "$EGO_WS/src/plan_env" \
  "$EGO_WS/src/plan_manage" \
  "$EGO_WS/src/rosmsg_tcp_bridge" \
  "$EGO_WS/src/traj_utils" \
  "$EGO_WS/src/quadrotor_msgs"
do
  if [ -e "$junk" ] || [ -L "$junk" ]; then
    echo "[ego] 移除重复路径：$junk"
    rm -rf "$junk"
  fi
done
# ego_ws 根若误带 .git（整仓 clone 进了 ego_ws），去掉以免干扰
if [ -d "$EGO_WS/.git" ] && [ ! -f "$EGO_SRC/.git/config" ]; then
  echo "[ego] 移除误放的 $EGO_WS/.git"
  rm -rf "$EGO_WS/.git"
fi

if [ ! -f "$EGO_SRC/src/planner/plan_manage/package.xml" ] \
   && [ ! -f "$EGO_SRC/planner/plan_manage/package.xml" ]; then
  echo "[ego] clone $EGO_REPO ($EGO_BRANCH) → $EGO_SRC"
  rm -rf "$EGO_SRC"
  git clone --depth 1 -b "$EGO_BRANCH" "$EGO_REPO" "$EGO_SRC"
else
  echo "[ego] 源码已存在：$EGO_SRC"
fi

# 解析 planner / quadrotor_msgs 实际路径
if [ -d "$EGO_SRC/src/planner" ]; then
  PLANNER_DIR="$EGO_SRC/src/planner"
  QMSG="$EGO_SRC/src/uav_simulator/Utils/quadrotor_msgs"
elif [ -d "$EGO_SRC/planner" ]; then
  PLANNER_DIR="$EGO_SRC/planner"
  QMSG="$EGO_SRC/uav_simulator/Utils/quadrotor_msgs"
else
  echo "[ego] 找不到 planner 目录" >&2
  find "$EGO_SRC" -maxdepth 3 -type d -name plan_manage 2>/dev/null | head
  exit 1
fi

# 仅把需要的包平铺软链到 src/；并对 clone 根目录打 COLCON_IGNORE，
# 防止 colcon 再递归扫到嵌套副本 → Duplicate package names
link_pkg() {
  local src="$1"
  local name
  name="$(basename "$src")"
  ln -sfn "$src" "$EGO_WS/src/$name"
  echo "[ego] link $name"
}

for d in "$PLANNER_DIR"/*; do
  [ -d "$d" ] || continue
  link_pkg "$d"
done
if [ -d "$QMSG" ]; then
  link_pkg "$QMSG"
else
  echo "[ego] 警告：找不到 quadrotor_msgs：$QMSG" >&2
fi

# 忽略嵌套仓库，只认 src/ 下的软链
touch "$EGO_SRC/COLCON_IGNORE"
# 可选包：板端常缺依赖
for skip in drone_detect rosmsg_tcp_bridge; do
  if [ -e "$EGO_WS/src/$skip" ]; then
    touch "$EGO_WS/src/$skip/COLCON_IGNORE"
    echo "[ego] COLCON_IGNORE $skip"
  fi
done

# ---------- 应用本地补丁：动态目标重规划（例程 10 必需）----------
# 上游 ros2_version 在 planNextWaypoint() 的 waypoint 回调里调用
# rclcpp::spin_some(node_)，而该节点已在主 executor 中 → 抛
# "Node '/ego_planner_node' has already been added to an executor" → abort。
# 例程 10 目标跟随每 0.5 s 下发新 goal，必然命中，必须打此补丁。
PATCH_FILE="$SCRIPT_DIR/patches/ego_replan_fsm_dynamic_goal.patch"
FSM_FILE="$EGO_SRC/src/planner/plan_manage/src/ego_replan_fsm.cpp"
if [ -f "$PATCH_FILE" ] && [ -f "$FSM_FILE" ]; then
  if grep -q "ZETTATREE: dynamic goal replan" "$FSM_FILE"; then
    echo "[ego] 动态目标补丁已应用，跳过"
  else
    echo "[ego] 应用动态目标补丁…"
    ( cd "$EGO_SRC" && git apply -p1 --verbose "$PATCH_FILE" ) \
      || ( cd "$EGO_SRC" && patch -p1 --forward -i "$PATCH_FILE" ) \
      || echo "[ego] 警告：补丁应用失败，动态目标下 EGO 可能 abort" >&2
  fi
else
  echo "[ego] 提示：未找到补丁或源码（跳过）：$PATCH_FILE" >&2
fi

echo "[ego] colcon list（应无 Duplicate）："
cd "$EGO_WS"
colcon list || true

echo "[ego] colcon build（ego_planner + 依赖）…"
set +e
colcon build --symlink-install --packages-up-to ego_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
RC=$?
set -e
if [ $RC -ne 0 ]; then
  echo "[ego] 编译失败。请检查 ARM 依赖 / 日志：$EGO_WS/log" >&2
  echo "[ego] 可试：EGO_FORCE_CLEAN=1 bash $0" >&2
  exit $RC
fi

# shellcheck disable=SC1091
source "$EGO_WS/install/setup.bash"
echo "[ego] 验证："
ros2 pkg prefix ego_planner && ros2 pkg executables ego_planner | head
echo "[ego] 完成。运行："
echo "  bash $SCRIPT_DIR/run_ego_full.sh"
