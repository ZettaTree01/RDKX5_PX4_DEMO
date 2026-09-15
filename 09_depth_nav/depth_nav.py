#!/usr/bin/env python3
"""基于深度的自主导航可视化 + 安全层。文档 4.4 / 例程 9。

默认对接例程 8：GS130W + hobot_stereonet（BPU）。
- 完整 C++ EGO：control-mode=safety（位姿由 traj_server 控；过近时速度覆盖）
- Python 对照：control-mode=follow（跟 /drone/ego/cmd_vel）
- OpenCV：官方 stereonet_visual | 俯视点云 + 轨迹（初始机头方向 = 屏幕上方 N）
- 状态栏：扇区距离/阶段/位移/速度/高度集中在画面底部，中文显示（PIL 渲染）
- 位移指示：机体 FLU 前/后/左/右/上升/下降
设定点只发给 OFFBOARD 管理器。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Path
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Header
from cv_bridge import CvBridge

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from depth_rgbd import (
    DEFAULT_CX, DEFAULT_CY, DEFAULT_FX, DEFAULT_FY,
    DepthSourceClock, depth_msg_to_meters, depth_to_points, image_msg_to_bgr,
    render_modeling_panel, simulate_depth_room)
from ego_depth_avoid import EgoAvoidConfig, compute_body_velocity
from frame_output import FrameOutput
from indoor import CRUISE_SIDE_M, NAV_VEL_MPS, RelAlt
from cn_hud import put_cn_lines

# Stereonet 发布端为 RELIABLE
_QOS_STEREO = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5)

# GS130W @ 640x352（与例程 8 一致）
GS_FX, GS_FY = 328.379, 328.379
GS_CX, GS_CY = 320.0, 176.0

TOPIC_PRESETS = {
    'stereonet': {
        'depth': '/StereoNetNode/stereonet_depth',
        'color': '/StereoNetNode/origin_left_image',
        'info': '/StereoNetNode/stereonet_depth/camera_info',
        'visual': '/StereoNetNode/stereonet_visual',
        'points': '/StereoNetNode/stereonet_pointcloud2',
    },
    'orbbec': {
        'depth': '/camera/depth/image_raw',
        'color': '/camera/color/image_raw',
        'info': '/camera/depth/camera_info',
    },
    'realsense': {
        'depth': '/camera/camera/depth/image_rect_raw',
        'color': '/camera/camera/color/image_raw',
        'info': '/camera/camera/depth/camera_info',
    },
}


def _ros_cloud_to_cam_xyz(msg: PointCloud2, max_points: int = 8000) -> np.ndarray:
    """官方点云为 ROS 相机系 (x前 y左 z上) → 光学系 (x右 y下 z前)，供俯视拼图。"""
    names = {f.name: f for f in msg.fields}
    if not all(k in names for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)
    off = {k: names[k].offset for k in ('x', 'y', 'z')}
    step = int(msg.point_step)
    n = int(msg.width) * max(1, int(msg.height))
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float32)
    stride = max(1, (n + max_points - 1) // max_points)
    idx = np.arange(0, n, stride, dtype=np.int32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    flat = buf[: n * step].reshape(n, step)
    xr = flat[idx, off['x']:off['x'] + 4].view(np.float32).reshape(-1)
    yr = flat[idx, off['y']:off['y'] + 4].view(np.float32).reshape(-1)
    zr = flat[idx, off['z']:off['z'] + 4].view(np.float32).reshape(-1)
    # ROS←相机：x=Z, y=-X, z=-Y  ⇒  X=-y, Y=-z, Z=x
    pts = np.empty((idx.size, 3), dtype=np.float32)
    pts[:, 0] = -yr
    pts[:, 1] = -zr
    pts[:, 2] = xr
    return pts


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _move_dirs(vx, vy, vz, max_vel):
    """按机体 FLU 速度拆成前/后/左/右/上升/下降。"""
    eps = max(1e-4, 0.12 * abs(float(max_vel)))
    dirs = []
    if vx > eps:
        dirs.append('前')
    elif vx < -eps:
        dirs.append('后')
    if vy > eps:
        dirs.append('左')
    elif vy < -eps:
        dirs.append('右')
    if vz > eps:
        dirs.append('上升')
    elif vz < -eps:
        dirs.append('下降')
    return dirs


def _draw_move_arrows(img, vx, vy, vz, max_vel):
    """右侧画位移箭头：上=升、下=降、左=左、右=右；底部双箭头=前/后。"""
    h, w = img.shape[:2]
    cx, cy = w - 78, h // 2 + 10
    reach = 42
    n = max(abs(max_vel), 1e-6)
    eps = 0.12 * n
    color_idle = (80, 80, 80)
    color_act = (0, 220, 255)

    def arm(dx, dy, active):
        color = color_act if active else color_idle
        thick = 3 if active else 1
        cv2.arrowedLine(
            img, (cx, cy), (cx + dx, cy + dy), color, thick, tipLength=0.35)

    arm(0, -reach, vz > eps)   # 上升
    arm(0, reach, vz < -eps)  # 下降
    arm(-reach, 0, vy > eps)  # 左
    arm(reach, 0, vy < -eps)  # 右
    moving = abs(vx) + abs(vy) + abs(vz) > eps
    cv2.circle(img, (cx, cy), 6, (0, 220, 255) if moving else (90, 90, 90), -1)

    bx, by = w // 2, h - 36
    if vx < -eps:
        cv2.arrowedLine(
            img, (bx, by - 28), (bx, by + 8), (0, 0, 255), 3, tipLength=0.4)
        cv2.arrowedLine(
            img, (bx, by - 44), (bx, by - 8), (0, 0, 255), 3, tipLength=0.4)
    elif vx > eps:
        cv2.arrowedLine(
            img, (bx, by + 8), (bx, by - 28), (0, 255, 0), 3, tipLength=0.4)
        cv2.arrowedLine(
            img, (bx, by - 8), (bx, by - 44), (0, 255, 0), 3, tipLength=0.4)


def _topic_or_none(value):
    if value is None:
        return None
    s = str(value).strip()
    if not s or s in ('__default__', 'none', 'None', 'null'):
        return None
    return s


class DepthNavNode(Node):
    def __init__(self, source='stereonet', depth_topic=None, color_topic=None,
                 info_topic=None, show=True, snapshot=None, snapshot_period=5.0,
                 max_vel=None, safe_distance=1.2, side_m=None,
                 stop_distance=0.45, min_range=0.3, max_range=5.0,
                 control_mode='follow'):
        super().__init__('depth_nav')
        self.bridge = CvBridge()
        self.source = source
        # follow=跟径/速度；safety=完整 EGO 控位，仅过近时速度覆盖；viz=只可视化
        self.control_mode = str(control_mode or 'follow').lower()
        self.max_vel = float(max_vel if max_vel is not None else NAV_VEL_MPS)
        self.safe_distance = float(safe_distance)
        self.stop_distance = float(stop_distance)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.side = float(side_m if side_m is not None else CRUISE_SIDE_M)
        if source == 'stereonet':
            self.fx, self.fy, self.cx, self.cy = GS_FX, GS_FY, GS_CX, GS_CY
        else:
            self.fx, self.fy = DEFAULT_FX, DEFAULT_FY
            self.cx, self.cy = DEFAULT_CX, DEFAULT_CY
        self.depth_m = None
        self.color_bgr = None
        self._sim = DepthSourceClock()

        self.airborne = False
        self.armed = False
        self.pose = None
        self.yaw = 0.0
        self._north_yaw = None  # 首个位姿锁定：初始机头方向 = 俯视图正上方 N
        self.home = None
        self.waypoints = None
        self.wp_idx = 0
        self.trail = []
        self._last_trail_t = 0.0
        self._rel_alt = RelAlt()
        self.rel_alt_m = 0.0
        self.phase = '等待起飞'
        self.front_depth = None
        self.left_depth = None
        self.right_depth = None
        self.clearance = None
        self.cmd = (0.0, 0.0, 0.0)  # 最新机体 FLU 速度：前/左/上
        self.move_label = '悬停'
        self._avoid_cfg = EgoAvoidConfig(
            depth_min=float(min_range),
            depth_max=float(max_range),
            skip_pixel=2,
            margin=2,
            safe_distance=float(safe_distance),
            stop_distance=float(stop_distance),
            max_vel=float(self.max_vel),
            enable_vertical=True,
        )
        self._viz_stride = 8 if source == 'simulate' else 14
        self._sim_color = None
        self._depth_ok = False
        self._wait_t0 = time.monotonic()
        self._last_visual = None
        self._last_visual_t = 0.0
        self._cloud_pts_cam = None
        self._last_cloud_t = 0.0
        self._ego_cmd = None
        self._ego_cmd_t = 0.0
        self.use_ego = True
        self._last_viz_draw_t = 0.0

        self.vel_pub = self.create_publisher(
            TwistStamped, '/drone/setpoint_velocity/body', 10)
        self.pos_pub = self.create_publisher(
            PoseStamped, '/drone/setpoint_position/local', 10)
        self.land_pub = self.create_publisher(Bool, '/drone/control/land', 10)
        self.path_pub = self.create_publisher(Path, '/drone/nav/path_history', 1)
        self.plan_pub = self.create_publisher(Path, '/drone/nav/path_plan', 1)
        self.ego_goal_pub = self.create_publisher(
            PoseStamped, '/drone/ego/goal', 1)

        self.create_subscription(
            Bool, '/drone/status/airborne',
            lambda m: setattr(self, 'airborne', m.data), 10)
        self.create_subscription(
            State, '/mavros/state',
            lambda m: setattr(self, 'armed', bool(m.armed)),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/drone/ego/cmd_vel', self._on_ego_cmd, 10)

        if source == 'simulate':
            self.create_timer(0.2, self._sim_depth)
            self._depth_ok = True
            self.get_logger().info('深度源=simulate（320x240 @5Hz）')
        elif source == 'stereonet':
            preset = TOPIC_PRESETS['stereonet']
            depth_topic = depth_topic or preset['depth']
            color_topic = color_topic or preset['color']
            info_topic = info_topic or preset['info']
            visual_topic = preset['visual']
            self.create_subscription(
                Image, depth_topic, self._on_depth, _QOS_STEREO)
            # 不用 origin_left / 官方点云：解码点云极吃 CPU，拖垮深彩 FPS
            self.create_subscription(
                Image, visual_topic, self._on_stereo_visual, _QOS_STEREO)
            self.create_subscription(
                CameraInfo, info_topic, self._on_info, _QOS_STEREO)
            self.create_timer(0.5, self._tick_wait_depth)
            self.get_logger().info(
                f'深度源=stereonet(BPU/GS130W) depth={depth_topic}')
            self.get_logger().info(
                f'可视化=官方深彩 {visual_topic} + 深度反投影俯视（省CPU）')
            self.get_logger().info(
                '需例程8链路：mipi dual + hobot_stereonet（可用 start_stereo:=true）')
        else:
            preset = TOPIC_PRESETS.get(source, TOPIC_PRESETS['orbbec'])
            depth_topic = depth_topic or preset['depth']
            color_topic = color_topic or preset['color']
            info_topic = info_topic or preset['info']
            self.create_subscription(
                Image, depth_topic, self._on_depth, qos_profile_sensor_data)
            self.create_subscription(
                Image, color_topic, self._on_color, qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
            self.create_timer(0.5, self._tick_wait_depth)
            self.get_logger().info(f'深度源={source} {depth_topic}')

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='depth nav (q/Esc)',
            fallback_path='/tmp/depth_nav_snapshot.jpg')

        self.create_timer(0.05, self._tick_control)
        self.create_timer(0.5, self._tick_viz)
        self.get_logger().info(
            f'深度导航 max_vel={self.max_vel:.3f} '
            f'safe={self.safe_distance:.2f} stop={self.stop_distance:.2f} '
            f'side={self.side:.3f}')

    def _tick_wait_depth(self):
        if self._depth_ok:
            return
        if time.monotonic() - self._wait_t0 < 2.0:
            return
        self.get_logger().warn(
            '尚无深度帧。stereonet 请先：'
            'bash /app/zettatree_demo/08_depth_camera/run.sh rviz:=false '
            '或本例程 start_stereo:=true',
            throttle_duration_sec=5.0)

    def _on_ego_cmd(self, msg: TwistStamped):
        """EGO 跟径建议速度（机体 FLU）。"""
        self._ego_cmd = (
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        )
        self._ego_cmd_t = time.monotonic()

    def _publish_ego_goal(self, xyz):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(xyz[0])
        ps.pose.position.y = float(xyz[1])
        ps.pose.position.z = float(xyz[2])
        ps.pose.orientation.w = 1.0
        self.ego_goal_pub.publish(ps)

    def _on_info(self, msg: CameraInfo):
        if msg.k[0] > 1.0:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])

    def _on_stereo_visual(self, msg: Image):
        """官方深彩（与例程 8 OpenCV 左栏一致）。限频解码减轻卡顿。"""
        now = time.monotonic()
        if now - self._last_visual_t < 0.2:
            return
        self._last_visual_t = now
        try:
            enc = (msg.encoding or '').lower()
            if enc in ('nv12', 'yuv420', 'yuv420p'):
                self._last_visual = image_msg_to_bgr(msg, self.bridge)
            else:
                self._last_visual = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            # 深彩同时作为 color，避免再订 origin_left
            self.color_bgr = self._last_visual
        except Exception as exc:
            self.get_logger().warn(
                f'官方深彩失败: {exc}', throttle_duration_sec=2.0)

    def _on_stereo_points(self, msg: PointCloud2):
        """官方彩色点云 → 光学系 xyz，供俯视拼图（RViz 仍直接订官方话题）。"""
        now = time.monotonic()
        if now - self._last_cloud_t < 0.35:
            return
        self._last_cloud_t = now
        try:
            self._cloud_pts_cam = _ros_cloud_to_cam_xyz(msg, max_points=2500)
        except Exception as exc:
            self.get_logger().warn(
                f'官方点云解析失败: {exc}', throttle_duration_sec=2.0)

    def _on_depth(self, msg: Image):
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_m = depth_msg_to_meters(np.asarray(arr), msg.encoding)
            self._depth_ok = True
        except Exception as exc:
            self.get_logger().warn(
                f'深度转换失败: {exc}', throttle_duration_sec=2.0)

    def _on_color(self, msg: Image):
        try:
            enc = (msg.encoding or '').lower()
            if enc in ('nv12', 'yuv420', 'yuv420p'):
                self.color_bgr = image_msg_to_bgr(msg, self.bridge)
            else:
                self.color_bgr = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _sim_depth(self):
        base = 2.4
        if self.pose and self.home and self.waypoints:
            base = 1.6 + 0.8 * abs(math.sin(self._sim.now() * 0.5))
        w, h = 320, 240
        self.fx = DEFAULT_FX * (w / 640.0)
        self.fy = DEFAULT_FY * (h / 480.0)
        self.cx = (w - 1) * 0.5
        self.cy = (h - 1) * 0.5
        self.depth_m = simulate_depth_room(
            width=w, height=h, t=self._sim.now(), wall_dist=base,
            fx=DEFAULT_FX, fy=DEFAULT_FY, noise_std=0.0)
        if self._sim_color is None:
            rgb = np.zeros((h, w, 3), np.uint8)
            rgb[:] = (35, 40, 48)
            cv2.putText(rgb, 'SIM NAV', (90, 125), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (150, 160, 180), 2)
            self._sim_color = rgb
        self.color_bgr = self._sim_color
        self._depth_ok = True

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.pose = (float(p.x), float(p.y), float(p.z))
        self.yaw = yaw_from_quat(msg.pose.orientation)
        if self._north_yaw is None:
            self._north_yaw = self.yaw
        self.rel_alt_m = self._rel_alt.update(p.z)
        now = time.monotonic()
        if self.airborne and now - self._last_trail_t > 0.2:
            self._last_trail_t = now
            if (not self.trail or
                    math.hypot(self.trail[-1][0] - p.x,
                               self.trail[-1][1] - p.y) > 0.02):
                self.trail.append((float(p.x), float(p.y)))
                if len(self.trail) > 2000:
                    self.trail = self.trail[-2000:]
                self._publish_history_path()

    def _publish_history_path(self):
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        z = self.pose[2] if self.pose else 0.0
        for x, y in self.trail:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _build_waypoints(self):
        x0, y0, z0 = self.home
        s = self.side
        self.waypoints = [
            (x0 + s, y0, z0),
            (x0 + s, y0 + s, z0),
            (x0, y0 + s, z0),
            (x0, y0, z0),
        ]
        plan = Path()
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = 'map'
        for x, y, z in self.waypoints:
            ps = PoseStamped()
            ps.header = plan.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            plan.poses.append(ps)
        self.plan_pub.publish(plan)

    def _band_median(self, depth_m: np.ndarray, x0, x1, y0, y1) -> float | None:
        """保留兼容；主路径已改用 ego_depth_avoid 多扇区。"""
        h, w = depth_m.shape[:2]
        xa, xb = int(w * x0), int(w * x1)
        ya, yb = int(h * y0), int(h * y1)
        roi = depth_m[ya:yb, xa:xb]
        vals = roi[(roi > self.min_range) & (roi < self.max_range)]
        if vals.size < 15:
            return None
        return float(np.median(vals))

    def _publish_vel(self, vx, vy, vz=0.0):
        self.cmd = (float(vx), float(vy), float(vz))
        dirs = _move_dirs(vx, vy, vz, self.max_vel)
        self.move_label = '、'.join(dirs) if dirs else '悬停'
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.vel_pub.publish(msg)

    def _apply_avoid(self, vx_b, vy_b):
        """Ego-Planner 风格局部自由空间反应式避障（教学轻量版）。

        参考：https://github.com/Kinang2/Ego-Planner-System
        机体 FLU：+x 前、+y 左、+z 上。
        """
        if self.depth_m is None:
            return vx_b, vy_b, 0.0, None
        self._avoid_cfg.safe_distance = self.safe_distance
        self._avoid_cfg.stop_distance = self.stop_distance
        self._avoid_cfg.max_vel = self.max_vel
        self._avoid_cfg.depth_min = self.min_range
        self._avoid_cfg.depth_max = self.max_range
        vx, vy, vz, sc, msg = compute_body_velocity(
            self.depth_m, vx_b, vy_b, self._avoid_cfg)
        self.clearance = sc
        self.left_depth = sc.left
        self.front_depth = sc.front
        self.right_depth = sc.right
        return vx, vy, vz, msg

    def _tick_control(self):
        if self.pose is None:
            self.phase = '等待位姿（MAVROS/台架）'
            self.cmd = (0.0, 0.0, 0.0)
            self.move_label = '悬停'
            return
        if not self.airborne:
            if not self.armed:
                self.phase = '监视/未解锁（需 arm:=true 才起飞）'
            else:
                self.phase = '已解锁，起飞斜坡中…'
            self.cmd = (0.0, 0.0, 0.0)
            self.move_label = '悬停'
            return
        if self.home is None:
            self.home = self.pose
            self._build_waypoints()
            self.wp_idx = 0
            self.phase = '导航中'
            self.get_logger().info(
                f'home={self.home} side={self.side:.3f} m '
                f'control={self.control_mode}')

        # 完整 C++ EGO：位姿跟径由 traj_server→offboard；本节点只可视化/安全覆盖
        if self.control_mode in ('viz', 'safety', 'full_ego'):
            vx_b = vy_b = vz_b = 0.0
            src_tag = '完整EGO'
            vx_b, vy_b, vz_s, avoid_msg = self._apply_avoid(vx_b, vy_b)
            if avoid_msg and self.control_mode != 'viz':
                vz_b = vz_s
                src_tag = '安全层'
                self._publish_vel(vx_b, vy_b, vz_b)
            elif self.control_mode == 'viz':
                self.cmd = (0.0, 0.0, 0.0)
                self.move_label = '悬停'
            self.phase = f'{avoid_msg or "完整 EGO 规划中"} | {src_tag}'
            return

        if self.wp_idx >= len(self.waypoints):
            self.phase = '完成，降落'
            self._publish_vel(0.0, 0.0, 0.0)
            land = Bool()
            land.data = True
            self.land_pub.publish(land)
            return

        tx, ty, tz = self.waypoints[self.wp_idx]
        self._publish_ego_goal((tx, ty, tz))
        x, y, _z = self.pose
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist < max(0.03, self.side * 0.15):
            self.wp_idx += 1
            self.get_logger().info(
                f'到达航点 {self.wp_idx}/{len(self.waypoints)}')
            return

        # 优先跟 Python EGO 规划路径速度；超时则直线朝向航点
        ego_fresh = (
            self.use_ego and self._ego_cmd is not None
            and (time.monotonic() - self._ego_cmd_t) < 0.5)
        if ego_fresh:
            vx_b, vy_b, vz_b = self._ego_cmd
            src_tag = 'EGO'
        else:
            yaw = self.yaw
            c, s = math.cos(yaw), math.sin(yaw)
            vx_w = dx / max(dist, 1e-3) * self.max_vel
            vy_w = dy / max(dist, 1e-3) * self.max_vel
            vx_b = c * vx_w + s * vy_w
            vy_b = -s * vx_w + c * vy_w
            vz_b = 0.0
            src_tag = '直飞'

        # 深度安全层：过近时覆盖 EGO（急停/绕行）
        vx_b, vy_b, vz_s, avoid_msg = self._apply_avoid(vx_b, vy_b)
        if avoid_msg:
            vz_b = vz_s
            src_tag = '安全层'
        speed = math.hypot(vx_b, vy_b)
        if speed > self.max_vel > 0:
            vx_b *= self.max_vel / speed
            vy_b *= self.max_vel / speed
        max_vz = abs(self.max_vel) * 0.5
        if abs(vz_b) > max_vz > 0:
            vz_b = math.copysign(max_vz, vz_b)
        self._publish_vel(vx_b, vy_b, vz_b)

        if avoid_msg:
            self.phase = f'{avoid_msg} | {src_tag}'
        else:
            self.phase = (
                f'航点 {self.wp_idx + 1}/{len(self.waypoints)} | {src_tag}')

    def _waiting_panel(self) -> np.ndarray:
        panel = np.zeros((360, 960, 3), np.uint8)
        panel[:] = (32, 34, 38)
        if self.source == 'simulate':
            lines = ['等待模拟深度…']
        elif self.source == 'stereonet':
            lines = [
                '等待 Stereonet（GS130W / 例程8）',
                'depth: /StereoNetNode/stereonet_depth',
                'visual/points: stereonet_visual + stereonet_pointcloud2',
                '一键：bash .../09_depth_nav/run.sh start_stereo:=true',
                '或另开：bash .../08_depth_camera/run.sh rviz:=false',
            ]
        else:
            lines = [
                f'等待深度图（source={self.source}）',
                '请先启动对应深度相机驱动',
                '或改用: source:=stereonet / source:=simulate',
            ]
        put_cn_lines(
            panel, [(t, (230, 230, 230)) for t in lines], origin=(24, 48))
        return panel

    def _tick_viz(self):
        if self.depth_m is None:
            if self.out.enabled():
                self.out.output(self._waiting_panel())
            return
        depth = self.depth_m
        color = self.color_bgr
        if color is not None and color.shape[:2] != depth.shape[:2]:
            color = cv2.resize(color, (depth.shape[1], depth.shape[0]))

        # stereonet：俯视用深度反投影（不再订官方点云，省大量 CPU）
        if (self.source == 'stereonet'
                and self._cloud_pts_cam is not None
                and len(self._cloud_pts_cam) > 0):
            pts = self._cloud_pts_cam
        else:
            pts, _ = depth_to_points(
                depth, self.fx, self.fy, self.cx, self.cy,
                stride=self._viz_stride, max_range=self.max_range,
                min_range=self.min_range, color_bgr=color)

        planned = None
        if self.waypoints:
            planned = [(p[0], p[1]) for p in self.waypoints[self.wp_idx:]]
            if self.pose:
                planned = [(self.pose[0], self.pose[1])] + planned
        panel = render_modeling_panel(
            depth, pts, color_bgr=color,
            trail_enu=self.trail,
            planned_enu=planned,
            pose_enu=self.pose,
            yaw=self.yaw,
            north_yaw=self._north_yaw,
            title='',  # 深彩画面不再叠标题；信息集中在底部状态栏
            panel_mode='depth_cloud')

        # 左栏换官方深彩（对齐例程 8）；扇区距离数值在底部状态栏，不再叠色框
        if panel.shape[1] > panel.shape[0]:
            h = panel.shape[0]
            left_w = panel.shape[1] - h
            try:
                if self._last_visual is not None:
                    viz = self._last_visual.copy()
                    if viz.shape[0] != h or viz.shape[1] != left_w:
                        viz = cv2.resize(viz, (left_w, h))
                    panel[:, :left_w] = viz
            except Exception:
                pass

        vx, vy, vz = self.cmd
        _draw_move_arrows(panel, vx, vy, vz, self.max_vel)

        # 底部状态栏：深彩图不再叠加文字，扇区距离/阶段/位移/速度/高度集中在此
        panel_h = panel.shape[0]  # 状态栏起始行（文字 y 需加此偏移）
        banner = np.zeros((120, panel.shape[1], 3), np.uint8)
        banner[:] = (28, 30, 36)
        cv2.line(banner, (0, 0), (banner.shape[1] - 1, 0),
                 (70, 72, 80), 1, cv2.LINE_AA)
        panel = np.vstack([panel, banner])

        def _sec_color(d):
            if d is None:
                return (150, 150, 155)
            if d <= self.stop_distance:
                return (90, 90, 255)
            if d < self.safe_distance:
                return (60, 165, 255)
            return (80, 220, 80)

        def _fmt(d):
            return f'{d:.1f}' if d is not None else '--'

        def _txt_w(text):
            # PIL 20px 字宽估算：CJK 全宽、ASCII 约半宽，用于状态栏排版
            return sum(20 if ord(ch) > 127 else 11.2 for ch in text)

        sc = self.clearance
        if sc is None:
            sectors = [('左', None), ('左前', None), ('前', None),
                       ('右前', None), ('右', None), ('上', None), ('下', None)]
        else:
            sectors = [('左', sc.left), ('左前', sc.front_left),
                       ('前', sc.front), ('右前', sc.front_right),
                       ('右', sc.right), ('上', sc.up), ('下', sc.down)]

        n_pts = len(pts) if pts is not None else 0
        y1, y2, y3, y4 = panel_h + 8, panel_h + 36, panel_h + 64, panel_h + 92
        cn_lines = [
            (f'阶段：{self.phase}    位移：{self.move_label}',
             (235, 235, 235), (14, y1)),
            (f'速度 ({vx:+.2f},{vy:+.2f},{vz:+.2f}) m/s    '
             f'高度 {self.rel_alt_m:.2f} m    点云 {n_pts}',
             (205, 205, 210), (14, y2)),
            ('距离(m)', (205, 205, 210), (14, y3)),
        ]
        # 水平扇区：左→右排列；红/橙/绿 = 急停/绕行/自由
        x = 14 + _txt_w('距离(m)') + 14
        for name, d in sectors[:5]:
            entry = f'{name} {_fmt(d)}'
            cn_lines.append((entry, _sec_color(d), (int(x), y3)))
            x += _txt_w(entry) + 14
        # 垂直扇区 + 深度源
        x = 14 + _txt_w('距离(m)') + 14
        for name, d in sectors[5:]:
            entry = f'{name} {_fmt(d)}'
            cn_lines.append((entry, _sec_color(d), (int(x), y4)))
            x += _txt_w(entry) + 14
        cn_lines.append((f'源 {self.source}', (150, 150, 155), (int(x), y4)))
        # 面板为 0.5s 定时刷新，整图一次 PIL 中文渲染不影响帧率
        put_cn_lines(panel, cn_lines, size=20)
        self.out.output(panel)


def main(args=None):
    parser = argparse.ArgumentParser(description='深度相机自主导航')
    parser.add_argument('--source', default='stereonet',
                        choices=['stereonet', 'simulate', 'orbbec', 'realsense'])
    parser.add_argument('--depth-topic', default='')
    parser.add_argument('--color-topic', default='')
    parser.add_argument('--info-topic', default='')
    parser.add_argument('--show', action='store_true', default=True)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--snapshot', default='')
    parser.add_argument('--snapshot-period', type=float, default=5.0)
    parser.add_argument('--max-vel', type=float, default=None)
    parser.add_argument('--safe-distance', type=float, default=1.2)
    parser.add_argument('--stop-distance', type=float, default=0.45)
    parser.add_argument('--side', type=float, default=None)
    parser.add_argument('--min-range', type=float, default=0.3)
    parser.add_argument('--max-range', type=float, default=5.0)
    parser.add_argument(
        '--control-mode', default='follow',
        choices=['follow', 'safety', 'viz', 'full_ego'],
        help='follow=跟径速度；safety/full_ego=完整EGO控位+安全覆盖；viz=仅显示')
    parsed, ros_args = parser.parse_known_args(args)
    show = parsed.show and not parsed.no_show

    rclpy.init(args=ros_args)
    node = DepthNavNode(
        source=parsed.source,
        depth_topic=_topic_or_none(parsed.depth_topic),
        color_topic=_topic_or_none(parsed.color_topic),
        info_topic=_topic_or_none(parsed.info_topic),
        show=show,
        snapshot=_topic_or_none(parsed.snapshot),
        snapshot_period=parsed.snapshot_period,
        max_vel=parsed.max_vel,
        safe_distance=parsed.safe_distance,
        stop_distance=parsed.stop_distance,
        side_m=parsed.side,
        min_range=parsed.min_range,
        max_range=parsed.max_range,
        control_mode=parsed.control_mode,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.out.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
