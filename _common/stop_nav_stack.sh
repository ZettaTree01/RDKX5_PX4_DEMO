#!/usr/bin/env bash
# 停止例程 8/9/10 相关进程（MIPI / Stereonet / EGO / RViz / 桥接 / MAVROS）。
# 供 Ctrl+C / EXIT 陷阱调用；勿匹配过宽以免误杀 SSH。
# 流程：先 SIGTERM 宽匹配 → 短暂等待 → 对残留再 SIGKILL。
set +e

_term_then_kill() {
  # 向命令行匹配 ``pat`` 的进程发 SIGTERM（找不到则忽略）
  local pat="$1"
  pkill -TERM -f "$pat" 2>/dev/null || true
}

echo "[stop_nav] 停止导航/深度相关进程…"

# launch 入口
_term_then_kill '/app/zettatree_demo/09_depth_nav/ego_full.launch.py'
_term_then_kill '/app/zettatree_demo/09_depth_nav/depth_nav.launch.py'
_term_then_kill '/app/zettatree_demo/10_target_follow/target_follow.launch.py'
_term_then_kill '/app/zettatree_demo/08_depth_camera/depth_camera.launch.py'

# 任务与桥接
_term_then_kill '/app/zettatree_demo/09_depth_nav/depth_nav.py'
_term_then_kill '/app/zettatree_demo/10_target_follow/target_follow.py'
_term_then_kill '/app/zettatree_demo/09_depth_nav/bridges/'
_term_then_kill '/app/zettatree_demo/08_depth_camera/depth_pointcloud.py'
_term_then_kill '/app/zettatree_demo/08_depth_camera/show_stereo_views.py'
_term_then_kill '/app/zettatree_demo/08_depth_camera/pub_stereo_caminfo.py'
_term_then_kill '/app/zettatree_demo/08_depth_camera/start_stereonet.sh'
_term_then_kill '/app/zettatree_demo/_common/offboard_manager.py'
_term_then_kill '/app/zettatree_demo/02_bench_pose_sim/bench_pose_sim.py'

# EGO / Stereonet / RViz
_term_then_kill '/ego_planner/ego_planner_node'
_term_then_kill '/ego_planner/traj_server'
_term_then_kill 'hobot_stereonet/stereonet_model_node'
_term_then_kill 'stereonet_model_node'
_term_then_kill 'rviz2 -d /app/zettatree_demo/'

# MAVROS（释放 /dev/ttyS2，便于紧急上锁）
_term_then_kill '/lib/mavros/mavros_node'
_term_then_kill 'mavros_node'

# MIPI（ensure_mipi_bpu 以 nohup 拉起，不随 launch 退出）
_term_then_kill '/opt/tros/humble/lib/mipi_cam/mipi_cam'
_term_then_kill 'ros2 run mipi_cam mipi_cam'

sleep 0.8

# 仍存活则强杀
for pat in \
  'ego_full.launch.py' \
  'depth_nav.launch.py' \
  'target_follow.launch.py' \
  'depth_camera.launch.py' \
  'depth_nav.py' \
  'target_follow.py' \
  '/app/zettatree_demo/09_depth_nav/bridges/' \
  'ego_planner_node' \
  'traj_server' \
  'stereonet_model_node' \
  'rviz2 -d /app/zettatree_demo/' \
  'mavros_node' \
  '/opt/tros/humble/lib/mipi_cam/mipi_cam' \
  'ros2 run mipi_cam mipi_cam' \
  'offboard_manager.py' \
  'bench_pose_sim.py'
do
  pkill -9 -f "$pat" 2>/dev/null || true
done

echo "[stop_nav] 完成"
exit 0
