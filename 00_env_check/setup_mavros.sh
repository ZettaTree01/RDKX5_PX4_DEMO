#!/usr/bin/env bash
# 在 RDK X5 / Ubuntu jammy arm64 上从源码安装 MAVROS（Humble）。
# 背景：packages.ros.org 当前常不提供 ros-humble-mavros / extras 的 arm64 deb，
# 仅有 mavros-msgs；例程依赖 mavros 节点，故在此 colcon 编译。
#
# 产物：/app/zettatree_demo/mavros_ws/install
# 布局：mavros_ws/src/mavros/{libmavconn,mavros,mavros_extras,...}
# 使用：source /opt/tros/humble/setup.bash && source .../mavros_ws/install/setup.bash
#
# 不用 set -u：ament/TROS setup.bash 会读可选未定义变量（如 AMENT_TRACE_SETUP_FILES）
set -eo pipefail

_source_setup() {
  # shellcheck disable=SC1090,SC1091
  set +u
  source "$1"
}

DEMO_ROOT="${DEMO_ROOT:-/app/zettatree_demo}"
WS="${MAVROS_WS:-$DEMO_ROOT/mavros_ws}"
ROS_SETUP="${ROS_SETUP:-}"
if [ -z "$ROS_SETUP" ]; then
  if [ -f /opt/tros/humble/setup.bash ]; then
    ROS_SETUP=/opt/tros/humble/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_SETUP=/opt/ros/humble/setup.bash
  else
    echo "[mavros] ERROR: 未找到 ROS2 Humble setup.bash" >&2
    exit 1
  fi
fi
_source_setup "$ROS_SETUP"

_have_mavros() {
  ros2 pkg prefix mavros >/dev/null 2>&1
}

if _have_mavros; then
  echo "[mavros] 已可用: $(ros2 pkg prefix mavros)"
  exit 0
fi

if [ -f "$WS/install/setup.bash" ]; then
  _source_setup "$WS/install/setup.bash"
  if _have_mavros; then
    echo "[mavros] 使用已有工作空间: $WS"
    exit 0
  fi
fi

# 纠正错误布局：曾把 mavros 仓库直接 clone 到 WS 根，同时又有 src/mavros，
# colcon 会报 Duplicate package names。
_fix_ws_layout() {
  mkdir -p "$WS/src"
  local d
  for d in libmavconn mavros mavros_extras mavros_examples mavros_msgs \
           test_mavros tools docs .devcontainer .git .github \
           AGENTS.md CONTRIBUTING.md README.md LICENSE.md LICENSE-BSD.txt \
           LICENSE-GPLv3.txt LICENSE-LGPLv3.txt mkdocs.yml SECURITY.md \
           dependencies.rosinstall .codespellrc .editorconfig .gitignore \
           .readthedocs.yaml; do
    if [ -e "$WS/$d" ]; then
      echo "[mavros] 移除错误布局: $WS/$d"
      rm -rf "$WS/$d"
    fi
  done
  for d in libmavconn mavros_extras mavros_examples mavros_msgs test_mavros; do
    if [ -d "$WS/src/$d" ] && [ -d "$WS/src/mavros/$d" ]; then
      echo "[mavros] 移除重复包: $WS/src/$d"
      rm -rf "$WS/src/$d"
    fi
  done
  if [ -d "$WS/src/mavros" ] && [ ! -f "$WS/src/mavros/mavros/package.xml" ]; then
    echo "[mavros] src/mavros 不是完整 monorepo，重建"
    rm -rf "$WS/src/mavros"
  fi
}

echo "[mavros] 从源码编译到 $WS …"
_fix_ws_layout

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends \
  git wget curl ca-certificates \
  python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-humble-mavros-msgs ros-humble-mavlink \
  geographiclib-tools libgeographic-dev \
  libeigen3-dev libxml2-dev libyaml-cpp-dev \
  libboost-system-dev libboost-dev \
  >/dev/null

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init 2>/dev/null || true
fi
rosdep update || true

mkdir -p "$WS/src"
cd "$WS/src"
if [ ! -d mavros/.git ] || [ ! -f mavros/mavros/package.xml ]; then
  rm -rf mavros
  echo "[mavros] git clone mavlink/mavros @ ros2 → src/mavros"
  git clone --depth 1 -b ros2 https://github.com/mavlink/mavros.git mavros
fi

# 系统已有 mavros_msgs / mavlink deb 时，去掉源码同名包，避免覆盖冲突
rm -rf mavros/mavros_msgs 2>/dev/null || true

_fix_ws_layout

# mavros ros2 tip 可能引用 MAV_AUTOPILOT::FLIX；jammy apt ros-humble-mavlink 常仍止于 REFLEX。
# 无该枚举时剥离 flix 模式解码，避免 uas_stringify.cpp 编译失败（PX4/APM 不受影响）。
_patch_mavlink_flix_compat() {
  local stringify="$WS/src/mavros/mavros/src/lib/uas_stringify.cpp"
  local hdr
  [ -f "$stringify" ] || return 0
  hdr="$(find /opt/ros /opt/tros -path '*/mavlink/v2.0/minimal/minimal.hpp' 2>/dev/null | head -1 || true)"
  if [ -n "$hdr" ] && grep -qE '\bFLIX\b' "$hdr" 2>/dev/null; then
    return 0
  fi
  if ! grep -q 'MAV_AUTOPILOT::FLIX' "$stringify" 2>/dev/null; then
    return 0
  fi
  echo "[mavros] 兼容补丁: 系统 mavlink 无 MAV_AUTOPILOT::FLIX，剥离相关代码"
  python3 - "$stringify" <<'PY'
import re, sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()
text2 = re.sub(
    r"\n// Flix modes\nstatic const cmode_map flix_cmode_map\{\{.*?\}\};\n",
    "\n",
    text,
    count=1,
    flags=re.S,
)
text2 = re.sub(
    r"\n  \} else if \(MAV_AUTOPILOT::FLIX == ap\) \{\n"
    r"    return str_mode_cmap\(flix_cmode_map, custom_mode\);\n",
    "\n",
    text2,
    count=1,
)
if text2 == text:
    sys.stderr.write("[mavros] ERROR: FLIX 兼容补丁未匹配到源码片段\n")
    sys.exit(1)
open(path, "w", encoding="utf-8", newline="\n").write(text2)
print(f"[mavros] patched {path}")
PY
}

_patch_mavlink_flix_compat

cd "$WS"
rm -rf build log 2>/dev/null || true
rosdep install --from-paths src --ignore-src -r -y || true

NPROC="$(nproc 2>/dev/null || echo 2)"
# 自适应并行：有 swap/内存时 -j3，紧张则降级。模板翻译单元大，-j4 易 OOM。
MEM_AVAIL_KB="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
if [ -n "${MAVROS_JOBS:-}" ]; then
  JOBS="$MAVROS_JOBS"
elif [ "${MEM_AVAIL_KB:-0}" -ge 4000000 ]; then
  JOBS=3
elif [ "${MEM_AVAIL_KB:-0}" -ge 2000000 ]; then
  JOBS=2
else
  JOBS=1
fi
if [ "$JOBS" -gt "$NPROC" ]; then JOBS="$NPROC"; fi
if [ "${MEM_AVAIL_KB:-0}" -gt 0 ] && [ "$MEM_AVAIL_KB" -lt 900000 ] && [ "$JOBS" -gt 1 ]; then
  echo "[mavros] MemAvailable=${MEM_AVAIL_KB}kB 偏低，降为 -j1"
  JOBS=1
fi
export MAKEFLAGS="-j${JOBS}"
echo "[mavros] colcon build -j${JOBS} （跳过测试，-O2）… MemAvailable=${MEM_AVAIL_KB:-?}kB"
colcon build --symlink-install \
  --base-paths src \
  --packages-up-to mavros mavros_extras \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG"

_source_setup "$WS/install/setup.bash"
if ! _have_mavros; then
  echo "[mavros] ERROR: 编译完成但仍找不到 mavros 包" >&2
  exit 1
fi

GEO_SCRIPT="$(ros2 pkg prefix mavros)/lib/mavros/install_geographiclib_datasets.sh"
if [ ! -x "$GEO_SCRIPT" ]; then
  GEO_SCRIPT="$(ros2 pkg prefix mavros)/share/mavros/scripts/install_geographiclib_datasets.sh"
fi
if [ -x "$GEO_SCRIPT" ]; then
  echo "[mavros] 安装 GeographicLib 数据集…"
  bash "$GEO_SCRIPT" || true
fi

MARKER="# >>> zettatree_demo mavros overlay >>>"
BASHRC="${HOME}/.bashrc"
if [ -f "$BASHRC" ] && ! grep -qF "$MARKER" "$BASHRC" 2>/dev/null; then
  cat >> "$BASHRC" <<EOF

${MARKER}
if [ -f "${WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
fi
# <<< zettatree_demo mavros overlay <<<
EOF
  echo "[mavros] 已写入 ~/.bashrc overlay"
fi

echo "[mavros] OK: $(ros2 pkg prefix mavros)"
echo "[mavros] 当前终端请执行: source $WS/install/setup.bash"
