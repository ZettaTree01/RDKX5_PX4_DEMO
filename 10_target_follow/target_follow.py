#!/usr/bin/env python3
"""目标跟随。文档 4.5 / 例程 10。

在例程 9（深度导航 / EGO）上深化：YOLO 检测行人 → 深度取 3D →
保持 standoff 距离跟随（参考 Fast-Planner / EGO-Planner 动态目标）。

控制模式：
  - ego：发布 /move_base_simple/goal，由完整 C++ EGO 避障规划跟径
  - direct：直接发 /drone/setpoint_position/local（无规划器时的台架对照）

OpenCV：
  - 左：检测画面 + 行人框（数值在底部状态栏）
  - 右：三维俯视（初始机头 = N；航迹；机体→跟随点→目标；中文「人」「跟」）
  - 底部状态栏四行中文（PIL）
  - YOLO 限频 5 Hz；目标短暂丢失在 target_timeout 内记忆保持。

必须拆桨。
参考：
  https://github.com/SnapDragonfly/Fast-Planner
  https://github.com/ZJU-FAST-Lab/ego-planner
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
import traceback

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, Header
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from cn_hud import put_cn_lines
from depth_rgbd import (
    depth_msg_to_meters, depth_to_points, image_msg_to_bgr,
    render_modeling_panel)
from ego_depth_avoid import EgoAvoidConfig, compute_body_velocity
from frame_output import FrameOutput
from indoor import NAV_VEL_MPS, RelAlt
from yolo_detector import YoloDetector

# Stereonet 发布端为 RELIABLE
_QOS_STEREO = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5)

# pose_to_odom 锁定的 z 基准（latched）：EGO 世界系 z_ego = z_mavros - z0
_QOS_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

GS_FX, GS_FY = 328.379, 328.379
GS_CX, GS_CY = 320.0, 176.0
PERSON_CLS = 0  # COCO person


def _yaw_from_quat(q) -> float:
    """四元数 → 偏航角。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
        """画单条十字臂箭头（升/降/左/右）。"""
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


def _median_depth(depth_m: np.ndarray, x1, y1, x2, y2):
    """检测框内深度中位数（取下半躯干区域，过滤天空空洞）。"""
    h, w = depth_m.shape[:2]
    x1 = max(0, min(w - 1, int(x1)))
    x2 = max(0, min(w - 1, int(x2)))
    y1 = max(0, min(h - 1, int(y1)))
    y2 = max(0, min(h - 1, int(y2)))
    if x2 <= x1 + 2 or y2 <= y1 + 2:
        return None
    # 框下半部更接近躯干，减少头部天空空洞
    y_mid = y1 + (y2 - y1) // 3
    roi = depth_m[y_mid:y2, x1:x2]
    valid = roi[(roi > 0.25) & (roi < 8.0) & np.isfinite(roi)]
    if valid.size < 8:
        return None
    return float(np.median(valid))


def _pixel_to_cam_ros(u, v, z, fx, fy, cx, cy):
    """像素 + 深度(m) → ROS 相机系 (x前 y左 z上)。"""
    x_opt = (u - cx) * z / fx
    y_opt = (v - cy) * z / fy
    # optical → ROS camera_link
    return float(z), float(-x_opt), float(-y_opt)


def _cam_to_world(xb, yb, zb, pose_xyz, yaw):
    """ROS 相机/机体 FLU 点 → ENU world（绕 yaw 旋转后加平移）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy, oz = pose_xyz
    return (
        c * xb - s * yb + ox,
        s * xb + c * yb + oy,
        zb + oz,
    )


class TargetFollowNode(Node):
    """YOLO 行人 + 深度取 3D，按 standoff 发布 EGO 目标或直接位姿。"""

    def __init__(
            self, source='stereonet', planner='ego', show=True,
            snapshot=None, snapshot_period=5.0, max_vel=None,
            standoff=0.8, follow_z=0.1, goal_period=0.5,
            safe_distance=1.2, stop_distance=0.45,
            min_score=0.25, detect_period=0.2, target_timeout=1.0):
        """planner=ego|direct；standoff 为与行人保持的水平距离(m)。"""
        super().__init__('target_follow')
        self.bridge = CvBridge()
        self.source = source
        self.planner = str(planner or 'ego').lower()
        self.max_vel = float(max_vel if max_vel is not None else NAV_VEL_MPS)
        self.standoff = float(standoff)
        self.follow_z = float(follow_z)
        self.goal_period = float(goal_period)
        self.safe_distance = float(safe_distance)
        self.stop_distance = float(stop_distance)
        self.min_score = float(min_score)
        self.detect_period = max(0.05, float(detect_period))
        self.target_timeout = max(0.0, float(target_timeout))
        self.fx, self.fy, self.cx, self.cy = GS_FX, GS_FY, GS_CX, GS_CY

        self.depth_m = None
        self.color_bgr = None
        self.airborne = False
        self.armed = False
        self.pose = None
        self.yaw = 0.0
        self._north_yaw = None  # 首个位姿锁定：初始机头方向 = 俯视图正上方 N
        self._rel_alt = RelAlt()
        self.rel_alt_m = 0.0
        self.phase = '等待起飞'
        self.target_w = None  # (x,y,z) world
        self.goal_w = None
        self._last_goal_t = 0.0
        self._last_goal_pub = None
        self.trail = []
        self._last_trail_t = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.move_label = '悬停'
        self._avoid_cfg = EgoAvoidConfig(
            safe_distance=self.safe_distance,
            stop_distance=self.stop_distance,
            max_vel=self.max_vel,
            enable_vertical=True,
        )
        self.clearance = None
        self._avoid_msg = None
        self._avoid_vel = None
        self._sim_t0 = time.monotonic()
        self._sim_center = None
        self._detect_box = None
        self._last_visual = None
        self._last_visual_t = 0.0
        self._last_color_t = 0.0
        self._last_detect_t = 0.0
        self._last_target_t = 0.0
        self._viz_stride = 8 if source == 'simulate' else 14

        self.detector = YoloDetector(score_thres=self.min_score, log=self.get_logger())

        self.goal_ego_pub = self.create_publisher(
            PoseStamped, '/move_base_simple/goal', 5)
        self.pos_pub = self.create_publisher(
            PoseStamped, '/drone/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/drone/setpoint_velocity/body', 10)
        self.target_pub = self.create_publisher(
            PoseStamped, '/drone/follow/target', 5)
        self.follow_goal_pub = self.create_publisher(
            PoseStamped, '/drone/follow/goal', 5)
        self.path_pub = self.create_publisher(Path, '/drone/nav/path_history', 1)
        self.link_pub = self.create_publisher(Marker, '/drone/follow/link', 5)

        # EGO z 对齐基准：室内 MAVROS local z 是气压绝对高度（几十米），
        # EGO 地图 z 固定 [-0.5, 1.5]，需减 z0 回到地图内（桥已同基准变换）
        self._z0 = None
        self.create_subscription(
            Float64, '/drone/ego/z_ref', self._on_zref, _QOS_LATCHED)

        self.create_subscription(
            Bool, '/drone/status/airborne',
            lambda m: setattr(self, 'airborne', bool(m.data)), 10)
        self.create_subscription(
            State, '/mavros/state',
            lambda m: setattr(self, 'armed', bool(m.armed)),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_profile_sensor_data)

        if source == 'simulate':
            self.create_timer(0.1, self._sim_tick)
            self.get_logger().info('跟随源=simulate（虚拟行人绕圈，世界系）')
        else:
            self.create_subscription(
                Image, '/StereoNetNode/stereonet_visual',
                self._on_visual, _QOS_STEREO)
            self.create_subscription(
                Image, '/StereoNetNode/origin_left_image',
                self._on_color, _QOS_STEREO)
            self.create_subscription(
                Image, '/StereoNetNode/stereonet_depth',
                self._on_depth, _QOS_STEREO)
            self.get_logger().info(
                f'跟随源=stereonet + YOLO person（限频 {1.0 / self.detect_period:.0f} Hz，'
                f'记忆保持 {self.target_timeout:.1f} s）')

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='target follow (q/Esc)',
            fallback_path='/tmp/target_follow_snapshot.jpg')

        self.create_timer(0.05, self._tick_control)
        self.create_timer(0.5, self._tick_viz)
        self.get_logger().info(
            f'目标跟随 planner={self.planner} standoff={self.standoff:.2f} '
            f'max_vel={self.max_vel:.3f} yolo={self.detector.loaded}')

    def _on_pose(self, msg: PoseStamped):
        """缓存位姿；可选按 z_ref 对齐到 EGO 世界系。"""
        p = msg.pose.position
        self.pose = (float(p.x), float(p.y), float(p.z))
        self.yaw = _yaw_from_quat(msg.pose.orientation)
        if self._north_yaw is None:
            self._north_yaw = self.yaw
        self.rel_alt_m = self._rel_alt.update(self.pose[2])

    def _on_zref(self, msg: Float64):
        """pose_to_odom（z_align）锁定的 z 基准：EGO 世界系 z = z_mavros - z0。"""
        if self._z0 is None:
            self._z0 = float(msg.data)
            self.get_logger().info(
                f'z 对齐基准 z0={self._z0:.3f}（EGO 世界系 z = z_mavros - z0）')

    def _ego_z(self, z_mavros: float) -> float:
        """MAVROS local z → EGO/world 可视化高度。"""
        if self._z0 is None:
            return float(z_mavros)
        return float(z_mavros) - self._z0

    def _ego_xyz(self, xyz) -> tuple:
        """三元组高度分量做 z 对齐。"""
        return (float(xyz[0]), float(xyz[1]), self._ego_z(xyz[2]))

    def _on_depth(self, msg: Image):
        """深度图回调。"""
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_m = depth_msg_to_meters(np.asarray(arr), msg.encoding)
        except Exception:
            self.get_logger().warn('深度解码失败', throttle_duration_sec=3.0)

    def _on_color(self, msg: Image):
        """彩色/左目回调，供 YOLO。"""
        # 检测画面解码限频：YOLO 只按 detect_period 消费，无需全帧解码
        now = time.monotonic()
        if now - self._last_color_t < self.detect_period * 0.75:
            return
        self._last_color_t = now
        # origin_left 是 NV12（mipi dual 常用），cv_bridge 不认，须走
        try:
            self.color_bgr = image_msg_to_bgr(msg, self.bridge)
        except Exception as exc:
            self.get_logger().warn(
                f'检测画面解码失败: {exc}', throttle_duration_sec=5.0)

    def _on_visual(self, msg: Image):
        """Stereonet 深彩回调。"""
        # 官方深彩仅作显示底图兜底（检测画面优先 origin_left）
        now = time.monotonic()
        if now - self._last_visual_t < 0.2:
            return
        self._last_visual_t = now
        try:
            self._last_visual = image_msg_to_bgr(msg, self.bridge)
        except Exception:
            pass

    # ---- 模拟行人（世界系，验证链路）----

    def _sim_tick(self):
        """虚拟行人：以首个位姿前方约 1.2m 为中心，世界系缓慢绕圈。"""
        if self.pose is None:
            return
        if self._sim_center is None:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self._sim_center = (
                self.pose[0] + c * 1.2, self.pose[1] + s * 1.2)
        t = time.monotonic() - self._sim_t0
        cx, cy = self._sim_center
        r = 0.25
        self.target_w = (
            cx + r * math.sin(0.5 * t),
            cy + r * math.cos(0.5 * t),
            0.0)
        self._last_target_t = time.monotonic()

        # 假画面：按内参把行人投影到像素画模拟框
        h, w = 352, 640
        fwd = (math.cos(self.yaw) * (self.target_w[0] - self.pose[0])
               + math.sin(self.yaw) * (self.target_w[1] - self.pose[1]))
        left = (-math.sin(self.yaw) * (self.target_w[0] - self.pose[0])
                + math.cos(self.yaw) * (self.target_w[1] - self.pose[1]))
        frame = np.zeros((h, w, 3), np.uint8)
        frame[:] = (40, 42, 48)
        if fwd > 0.3:
            u = self.cx + self.fx * (-left) / fwd
            v = self.cy
            half_w = self.fx * 0.25 / fwd
            top = v - self.fy * 0.55 / fwd
            bottom = v + self.fy * 0.55 / fwd
            x1, y1 = int(u - half_w), int(top)
            x2, y2 = int(u + half_w), int(bottom)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (80, 200, 80), 2)
            put_cn_lines(frame, [('模拟行人', (80, 220, 80))],
                         origin=(x1, max(0, y1 - 26)), size=18)
            self._detect_box = (x1, y1, x2, y2, 0.99, PERSON_CLS)
        else:
            self._detect_box = None
        # 假深度 3m 背景：不触发安全层，扇区距离有数可显
        self.depth_m = np.full((h, w), 3.0, np.float32)
        self.color_bgr = frame

    # ---- 检测与目标 ----

    def _detect_person(self, frame):
        """YOLO 检人，返回最优行人框。"""
        if not self.detector.loaded:
            return None
        try:
            dets = self.detector.detect(frame)
        except Exception:
            self.get_logger().error(
                'YOLO 失败:\n' + traceback.format_exc(),
                throttle_duration_sec=5.0)
            return None
        best = None
        for x1, y1, x2, y2, score, cls_id in dets:
            if int(cls_id) != PERSON_CLS:
                continue
            if score < self.min_score:
                continue
            if best is None or score > best[4]:
                best = (x1, y1, x2, y2, score, cls_id)
        return best

    def _update_target_from_vision(self) -> bool:
        """更新行人 3D 目标。两次检测之间与短暂丢失时沿用记忆。"""
        now = time.monotonic()
        if self.source == 'simulate':
            return self.target_w is not None
        fresh = (self.target_w is not None
                 and now - self._last_target_t <= self.target_timeout)
        if now - self._last_detect_t < self.detect_period:
            return fresh
        self._last_detect_t = now
        if self.color_bgr is None or self.depth_m is None or self.pose is None:
            return False
        box = self._detect_person(self.color_bgr)
        self._detect_box = box
        if box is not None:
            x1, y1, x2, y2, score, _ = box
            u = 0.5 * (x1 + x2)
            v = 0.55 * y1 + 0.45 * y2
            z = _median_depth(self.depth_m, x1, y1, x2, y2)
            if z is not None:
                xb, yb, zb = _pixel_to_cam_ros(
                    u, v, z, self.fx, self.fy, self.cx, self.cy)
                self.target_w = _cam_to_world(xb, yb, zb, self.pose, self.yaw)
                self._last_target_t = now
                return True
        # 检测失败/深度无效：目标记忆短暂保持
        return fresh

    def _compute_follow_goal(self):
        """按 standoff 距离计算跟随点（机→人方向回退）。"""
        if self.target_w is None or self.pose is None:
            return None
        tx, ty, tz = self.target_w
        dx = tx - self.pose[0]
        dy = ty - self.pose[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            ux, uy = math.cos(self.yaw), math.sin(self.yaw)
        else:
            ux, uy = dx / dist, dy / dist
        # 停在行人与飞机连线上、距行人 standoff 处（Fast-Planner 式跟飞点）
        gx = tx - ux * self.standoff
        gy = ty - uy * self.standoff
        # gz 为 EGO 对齐世界系（z_ego = z_mavros - z0）：
        # follow_z>0 直接是相对高度；用目标 z 时需减基准换系
        if self.follow_z > 0:
            gz = self.follow_z
        else:
            gz = tz if self._z0 is None else tz - self._z0
        gz = max(0.05, gz)
        return (gx, gy, gz)

    # ---- 安全层与扇区距离 ----

    def _measure_clearance(self):
        """多扇区自由距离（底部状态栏 + 安全层共用，每周期一次）。"""
        self._avoid_msg = None
        self._avoid_vel = None
        if self.depth_m is None:
            self.clearance = None
            return
        self._avoid_cfg.safe_distance = self.safe_distance
        self._avoid_cfg.stop_distance = self.stop_distance
        self._avoid_cfg.max_vel = self.max_vel
        vx, vy, vz, sc, msg = compute_body_velocity(
            self.depth_m, 0.0, 0.0, self._avoid_cfg)
        self.clearance = sc
        if msg:
            self._avoid_msg = msg
            self._avoid_vel = (vx, vy, vz)

    def _apply_safety_override(self) -> bool:
        """过近时用机体速度覆盖 EGO/直跟（与例程 9 安全层一致）。"""
        if (self.depth_m is None or self.planner == 'direct'
                or not self._avoid_msg or self._avoid_vel is None):
            return False
        vx, vy, vz = self._avoid_vel
        self._publish_vel(vx, vy, vz)
        self.phase = f'安全层 {self._avoid_msg} | {self.planner}'
        return True

    # ---- 控制 ----

    def _publish_vel(self, vx, vy, vz=0.0):
        """发布机体速度给 OFFBOARD 管理器。"""
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

    def _pose_msg(self, xyz, frame='world') -> PoseStamped:
        """构造 PoseStamped。"""
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame
        ps.pose.position.x = float(xyz[0])
        ps.pose.position.y = float(xyz[1])
        ps.pose.position.z = float(xyz[2])
        # 朝向目标点水平朝向
        if self.target_w is not None:
            yaw = math.atan2(
                self.target_w[1] - xyz[1], self.target_w[0] - xyz[0])
        else:
            yaw = self.yaw
        ps.pose.orientation.z = math.sin(yaw * 0.5)
        ps.pose.orientation.w = math.cos(yaw * 0.5)
        return ps

    def _should_republish_goal(self, goal) -> bool:
        """目标变化足够大或超时才重发，减轻 EGO 抖动。"""
        now = time.monotonic()
        if now - self._last_goal_t < self.goal_period:
            if self._last_goal_pub is None:
                return True
            d = math.hypot(
                goal[0] - self._last_goal_pub[0],
                goal[1] - self._last_goal_pub[1])
            return d > max(0.12, self.standoff * 0.15)
        return True

    def _publish_goal(self, goal):
        """按 control_mode 发 EGO goal 或直接局部位姿。"""
        # EGO MANUAL_TARGET（z 已在 EGO 对齐系）
        ego = self._pose_msg(goal, 'world')
        self.goal_ego_pub.publish(ego)
        # RViz：与 EGO Marker 同用 world（Fixed Frame=camera_link 靠 TF）
        self.follow_goal_pub.publish(self._pose_msg(goal, 'world'))
        if self.target_w is not None:
            self.target_pub.publish(self._pose_msg(
                self._ego_xyz(self.target_w), 'world'))
        self._publish_follow_link(goal)
        if self.planner in ('direct', 'position'):
            # offboard 用 MAVROS local 系：把 EGO 对齐系 z 加回基准
            sp_goal = goal if self._z0 is None else (
                goal[0], goal[1], goal[2] + self._z0)
            self.pos_pub.publish(self._pose_msg(sp_goal, 'map'))
        self._last_goal_t = time.monotonic()
        self._last_goal_pub = goal

    def _publish_follow_link(self, goal):
        """机体 → 跟随点 → 行人：RViz 橙线，与 OpenCV 俯视一致。"""
        if self.pose is None:
            return
        mk = Marker()
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.header.frame_id = 'world'
        mk.ns = 'follow_link'
        mk.id = 0
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = 0.06
        mk.color.r = 1.0
        mk.color.g = 0.55
        mk.color.b = 0.1
        mk.color.a = 1.0
        mk.lifetime.sec = 1
        body = self._ego_xyz(self.pose)
        pts = [body, (float(goal[0]), float(goal[1]), float(goal[2]))]
        if self.target_w is not None:
            pts.append(self._ego_xyz(self.target_w))
        for xyz in pts:
            p = Point()
            p.x, p.y, p.z = xyz
            mk.points.append(p)
        self.link_pub.publish(mk)

    def _publish_history_path(self):
        """发布历史航迹 Path。"""
        path = Path()
        path.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id='world')
        for x, y, z in self.trail:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _tick_control(self):
        """控制周期：更新目标、安全覆盖、发布跟随指令。"""
        # 航迹：起飞后、水平位移超阈值才记录
        if (self.airborne and self.pose is not None
                and time.monotonic() - self._last_trail_t > 0.2):
            self._last_trail_t = time.monotonic()
            ez = self._ego_z(self.pose[2])
            if (not self.trail
                    or math.hypot(self.trail[-1][0] - self.pose[0],
                                  self.trail[-1][1] - self.pose[1]) > 0.02):
                self.trail.append((self.pose[0], self.pose[1], ez))
                if len(self.trail) > 2000:
                    self.trail = self.trail[-2000:]
                self._publish_history_path()

        if self.pose is None:
            self.phase = '等待位姿（MAVROS/台架）'
            self.cmd = (0.0, 0.0, 0.0)
            self.move_label = '悬停'
            return
        if not self.airborne:
            if self.armed:
                self.phase = '已解锁，起飞斜坡中…'
            else:
                self.phase = '监视/未解锁（需 arm:=true 才起飞）'
            self.cmd = (0.0, 0.0, 0.0)
            self.move_label = '悬停'
            return

        self._measure_clearance()

        has_tgt = self._update_target_from_vision()
        if not has_tgt:
            self.phase = '搜索行人…'
            self.move_label = '悬停'
            self.cmd = (0.0, 0.0, 0.0)
            return

        if self._apply_safety_override():
            return

        goal = self._compute_follow_goal()
        if goal is None:
            return
        self.goal_w = goal
        if self._should_republish_goal(goal):
            self._publish_goal(goal)

        # 位移 HUD：机体 FLU 朝跟随点的方向
        dx = goal[0] - self.pose[0]
        dy = goal[1] - self.pose[1]
        dist = math.hypot(dx, dy)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = c * dx + s * dy
        vy_b = -s * dx + c * dy
        self.cmd = (vx_b, vy_b, 0.0)
        dirs = []
        eps = max(0.03, 0.15 * self.standoff)
        if dist > eps:
            if vx_b > eps:
                dirs.append('前')
            elif vx_b < -eps:
                dirs.append('后')
            if vy_b > eps:
                dirs.append('左')
            elif vy_b < -eps:
                dirs.append('右')
        self.move_label = '、'.join(dirs) if dirs else '保持'
        td = math.hypot(
            self.target_w[0] - self.pose[0],
            self.target_w[1] - self.pose[1]) if self.target_w else 0.0
        stale = time.monotonic() - self._last_target_t > max(
            2.0 * self.detect_period, 0.4)
        self.phase = (
            f'跟随 | 目标 {td:.2f} m（保持 {self.standoff:.2f}）'
            f' | {self.planner}' + (' | 记忆保持' if stale else ''))

    # ---- 可视化 ----

    def _waiting_panel(self) -> np.ndarray:
        """等待视觉/深度时的提示面板。"""
        panel = np.zeros((472, 992, 3), np.uint8)
        panel[:] = (32, 34, 38)
        if self.source == 'simulate':
            lines = ['等待台架位姿…（bench_pose_sim / MAVROS）']
        else:
            lines = [
                '等待 Stereonet / 行人检测…',
                'depth: /StereoNetNode/stereonet_depth',
                '检测画面: /StereoNetNode/origin_left_image',
                '一键：bash .../10_target_follow/run.sh start_stereo:=true',
            ]
        put_cn_lines(
            panel, [(t, (230, 230, 230)) for t in lines], origin=(24, 48))
        return panel

    def _tick_viz(self):
        """OpenCV 双栏：检测画面 | 俯视跟随几何 + 中文状态栏。"""
        if not self.out.enabled():
            return
        # 左：检测画面（origin_left 优先，退官方深彩/模拟画面）
        left = None
        if self.color_bgr is not None:
            left = self.color_bgr.copy()
        elif self._last_visual is not None:
            left = self._last_visual.copy()
        if left is not None and self.depth_m is None:
            self.out.output(left)
            return
        if self.depth_m is None or left is None:
            self.out.output(self._waiting_panel())
            return

        # 行人框画在检测画面（画面干净：数值一律在底部状态栏）
        box = self._detect_box
        if box is not None:
            x1, y1, x2, y2, score, _ = box
            cv2.rectangle(
                left, (int(x1), int(y1)), (int(x2), int(y2)),
                (80, 220, 80), 2)
            cv2.putText(
                left, f'person {score:.2f}',
                (int(x1), max(16, int(y1) - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 220, 80), 1, cv2.LINE_AA)
        if left.shape[:2] != self.depth_m.shape[:2]:
            left = cv2.resize(
                left, (self.depth_m.shape[1], self.depth_m.shape[0]))

        # 右：三维俯视（N=初始机头；航迹黄线；机体→跟随点→目标橙线）
        pts, _ = depth_to_points(
            self.depth_m, self.fx, self.fy, self.cx, self.cy,
            stride=self._viz_stride, max_range=5.0, min_range=0.3)
        planned = None
        mk_target = mk_goal = None
        if self.pose is not None:
            if self.goal_w is not None:
                planned = [(self.pose[0], self.pose[1]),
                           (self.goal_w[0], self.goal_w[1])]
                if self.target_w is not None:
                    planned.append((self.target_w[0], self.target_w[1]))
                mk_goal = (self.goal_w[0], self.goal_w[1],
                           (60, 165, 255), 6, 'diamond')
            if self.target_w is not None:
                mk_target = (self.target_w[0], self.target_w[1],
                             (80, 220, 80), 7, 'circle')
        markers = [mk[:5] for mk in (mk_target, mk_goal) if mk is not None]
        top = render_modeling_panel(
            self.depth_m, pts, color_bgr=None,
            trail_enu=self.trail,
            planned_enu=planned,
            pose_enu=self.pose,
            yaw=self.yaw,
            north_yaw=self._north_yaw,
            title='', panel_mode='cloud', markers=markers)

        h = left.shape[0]
        top_r = cv2.resize(top, (h, h))
        panel = np.hstack([left, top_r])

        # 目标/跟随点中文标签（画在拼图上，与标记同步旋转）
        if markers and self._north_yaw is not None:
            rot = math.pi / 2.0 - self._north_yaw
            cm, sm = math.cos(rot), math.sin(rot)
            k = h / 480.0
            cn_labels = []
            for mk, txt in ((mk_target, '人'), (mk_goal, '跟')):
                if mk is None:
                    continue
                rx = cm * mk[0] - sm * mk[1]
                ry = sm * mk[0] + cm * mk[1]
                px = int((rx * (480.0 / 6.0) + 240.0) * k) + left.shape[1]
                py = int((240.0 - ry * (480.0 / 6.0)) * k)
                cn_labels.append((txt, mk[2], (px + 8, py - 14)))
            put_cn_lines(panel, cn_labels, size=16)

        vx, vy, vz = self.cmd
        _draw_move_arrows(panel, vx, vy, vz, self.max_vel)

        # 底部状态栏：检测画面不再叠加文字，数值集中在此分四行中文显示
        panel_h = panel.shape[0]
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

        td = (math.hypot(self.target_w[0] - self.pose[0],
                         self.target_w[1] - self.pose[1])
              if (self.target_w and self.pose) else None)
        gd = (math.hypot(self.goal_w[0] - self.pose[0],
                         self.goal_w[1] - self.pose[1])
              if (self.goal_w and self.pose) else None)
        n_pts = len(pts) if pts is not None else 0
        y1, y2, y3, y4 = panel_h + 8, panel_h + 36, panel_h + 64, panel_h + 92
        cn_lines = [
            (f'阶段：{self.phase}    位移：{self.move_label}',
             (235, 235, 235), (14, y1)),
            (f'速度 ({vx:+.2f},{vy:+.2f},{vz:+.2f}) m/s    '
             f'高度 {self.rel_alt_m:.2f} m    目标 {_fmt(td)} m    '
             f'点云 {n_pts}    yolo {"OK" if self.detector.loaded else "无"}',
             (205, 205, 210), (14, y2)),
            ('距离(m)', (205, 205, 210), (14, y3)),
        ]
        # 水平扇区：左→右排列；红/橙/绿 = 急停/绕行/自由
        x = 14 + _txt_w('距离(m)') + 14
        for name, d in sectors[:5]:
            entry = f'{name} {_fmt(d)}'
            cn_lines.append((entry, _sec_color(d), (int(x), y3)))
            x += _txt_w(entry) + 14
        # 垂直扇区 + 跟随点距离 + 源
        x = 14 + _txt_w('距离(m)') + 14
        for name, d in sectors[5:]:
            entry = f'{name} {_fmt(d)}'
            cn_lines.append((entry, _sec_color(d), (int(x), y4)))
            x += _txt_w(entry) + 14
        cn_lines.append((f'跟随点 {_fmt(gd)} m', (60, 165, 255), (int(x), y4)))
        x += _txt_w(f'跟随点 {0.0:.1f} m') + 14
        cn_lines.append((f'源 {self.source}', (150, 150, 155), (int(x), y4)))
        # 面板为 0.5s 定时刷新，整图一次 PIL 中文渲染不影响帧率
        put_cn_lines(panel, cn_lines, size=20)
        self.out.output(panel)


def main(args=None):
    """解析参数并 spin 目标跟随节点。"""
    parser = argparse.ArgumentParser(description='目标跟随（行人）')
    parser.add_argument('--source', default='stereonet',
                        choices=['stereonet', 'simulate'])
    parser.add_argument('--planner', default='ego',
                        choices=['ego', 'direct'],
                        help='ego=/move_base_simple/goal；direct=位置设定点')
    parser.add_argument('--show', action='store_true', default=True)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--snapshot', default='')
    parser.add_argument('--snapshot-period', type=float, default=5.0)
    parser.add_argument('--max-vel', type=float, default=None)
    parser.add_argument('--standoff', type=float, default=0.8,
                        help='与行人水平保持距离（m）')
    parser.add_argument('--follow-z', type=float, default=0.1,
                        help='跟随高度（m，室内台架默认 0.1）')
    parser.add_argument('--goal-period', type=float, default=0.5)
    parser.add_argument('--safe-distance', type=float, default=1.2)
    parser.add_argument('--stop-distance', type=float, default=0.45)
    parser.add_argument('--min-score', type=float, default=0.25)
    parser.add_argument('--detect-period', type=float, default=0.2,
                        help='YOLO 检测周期（s，默认 0.2 = 5 Hz）')
    parser.add_argument('--target-timeout', type=float, default=1.0,
                        help='目标丢失记忆保持时长（s）')
    parsed, ros_args = parser.parse_known_args(args)
    show = parsed.show and not parsed.no_show
    snap = parsed.snapshot.strip() or None

    rclpy.init(args=ros_args)
    node = TargetFollowNode(
        source=parsed.source,
        planner=parsed.planner,
        show=show,
        snapshot=snap,
        snapshot_period=parsed.snapshot_period,
        max_vel=parsed.max_vel,
        standoff=parsed.standoff,
        follow_z=parsed.follow_z,
        goal_period=parsed.goal_period,
        safe_distance=parsed.safe_distance,
        stop_distance=parsed.stop_distance,
        min_score=parsed.min_score,
        detect_period=parsed.detect_period,
        target_timeout=parsed.target_timeout,
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
