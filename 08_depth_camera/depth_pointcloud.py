#!/usr/bin/env python3
"""深度摄像头：深彩 + 三维俯视（OpenCV）。文档 4.3 / 例程 8。

链路：双目摄像头 → Depth(Stereonet BPU) → OpenCV【深彩 | 三维地图】
默认不启 RViz。RViz 直接订官方 /StereoNetNode/stereonet_pointcloud2（XYZRGB）。
内部仍抽稀发布 /drone/depth/points、/drone/map/points（俯视与可选地图节点用）。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from cv_bridge import CvBridge

_QOS_STEREO_DEPTH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5)

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from depth_rgbd import (
    DEFAULT_CX, DEFAULT_CY, DEFAULT_FX, DEFAULT_FY,
    DepthSourceClock, clean_depth_m, colorize_depth, depth_msg_to_meters,
    depth_to_points, image_msg_to_bgr, points_to_cloud2_xyz,
    render_modeling_panel, rotate_bgr, rotate_intrinsics_cw,
    simulate_depth_room, split_stereo_combine, stereo_to_depth_m,
    _project_top)
from frame_output import FrameOutput


TOPIC_PRESETS = {
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
    'mipi_stereo': {
        'combine': '/image_combine_raw',
        'info': '/image_combine_raw/left/camera_info',
    },
    'stereonet': {
        # 官方 hobot_stereonet 输出（由 mipi_cam dual 喂图）
        'depth': '/StereoNetNode/stereonet_depth',
        'visual': '/StereoNetNode/stereonet_visual',
        'points': '/StereoNetNode/stereonet_pointcloud2',
        'color': '/StereoNetNode/origin_left_image',
        'info': '/StereoNetNode/stereonet_depth/camera_info',
    },
}


def _topic_or_none(value):
    if value is None:
        return None
    s = str(value).strip()
    if not s or s in ('__default__', 'none', 'None', 'null', '__none__'):
        return None
    return s


class DepthPointCloudNode(Node):
    def __init__(self, source='simulate', depth_topic=None, color_topic=None,
                 info_topic=None, combine_topic=None, show=True, snapshot=None,
                 snapshot_period=5.0, stride=4, max_range=5.0,
                 baseline_m=0.07917, stereo_layout='tb', stereo_max_width=320,
                 rotate_cw=90, panel_mode='depth', stereo_matcher='bm',
                 stereo_period=1.0, min_range=0.4, opencv_fallback=False,
                 map_enable=False, publish_filtered_cloud=True, publish_hz=4.0):
        super().__init__('depth_pointcloud')
        self.bridge = CvBridge()
        self.source = source
        self.stride = max(1, int(stride))
        self.max_range = float(max_range)
        self.min_range = float(min_range)
        self.baseline_m = float(baseline_m)
        self.stereo_layout = stereo_layout
        self.stereo_max_width = int(stereo_max_width)
        self.rotate_cw = int(rotate_cw) % 360
        self.panel_mode = (panel_mode or 'depth').lower()
        self.stereo_matcher = (stereo_matcher or 'bm').lower()
        self.opencv_fallback = bool(opencv_fallback)
        self.map_enable = bool(map_enable)
        self.publish_filtered_cloud = bool(publish_filtered_cloud)
        self.publish_hz = max(0.5, float(publish_hz))
        self.fx = DEFAULT_FX
        self.fy = DEFAULT_FY
        self.cx = DEFAULT_CX
        self.cy = DEFAULT_CY
        self.depth_m = None
        self.color_bgr = None
        self._combine_bgr = None
        self._combine_stamp = 0.0
        self._last_proc = 0.0
        self._pub_min_dt = 1.0 / self.publish_hz
        self._sim = DepthSourceClock()
        self._busy = False
        self._depth_ema = None
        self._last_pts = None
        self._stereo_official = False
        self._stereo_depth_ok = False
        self._stereo_visual_ok = False
        self._stereo_points_ok = False
        # 官方 Stereonet 已按 mipi rotation/GDC 出图；stereonet 路径不做二次旋转
        self._need_rotate_depth = source in ('orbbec', 'realsense')

        self.cloud_pub = self.create_publisher(
            PointCloud2, '/drone/depth/points', _QOS_STEREO_DEPTH)
        self.depth_viz_pub = self.create_publisher(
            Image, '/drone/depth/image_color', _QOS_STEREO_DEPTH)
        self.depth_raw_pub = self.create_publisher(
            Image, '/drone/depth/image_raw', _QOS_STEREO_DEPTH)
        self.map_pub = self.create_publisher(
            PointCloud2, '/drone/map/points', _QOS_STEREO_DEPTH)
        self._last_visual = None
        self._last_depth_msg = None
        self._map_pts = np.zeros((0, 3), dtype=np.float32)
        self._map_voxel = 0.05
        self._map_max = 10000
        self._map_voxels = {}
        self._last_panel = 0.0

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='depth+map (q/Esc)',
            fallback_path=None,
            allow_file_fallback=False)

        if source == 'simulate':
            self.get_logger().info(
                '数据源=simulate。实拍请用 mipi_stereo 或 stereonet。')
            self.create_timer(0.2, self._tick_simulate)
            self.stride = max(self.stride, 6)
        elif source == 'mipi_stereo':
            preset = TOPIC_PRESETS['mipi_stereo']
            combine_topic = combine_topic or preset['combine']
            info_topic = info_topic or preset['info']
            # 轻量默认：不强制抬高用户传入的 stride
            if self.stereo_max_width <= 0:
                self.stereo_max_width = 320
            period = max(0.3, float(stereo_period))
            self.create_subscription(
                Image, combine_topic, self._on_combine, qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
            self.create_timer(period, self._tick_stereo)
            self.get_logger().info(
                f'数据源=mipi_stereo({self.stereo_matcher}) combine={combine_topic} '
                f'baseline={self.baseline_m:.3f}m rotate_cw={self.rotate_cw} '
                f'panel={self.panel_mode} max_w={self.stereo_max_width} '
                f'stride={self.stride} period={period:.1f}s [CPU OpenCV]')
            self.get_logger().warn(
                'CPU 视差较卡；推荐改用 BPU：bash .../run.sh（默认 stereonet）')
        elif source == 'stereonet':
            # OpenCV：深彩 | 三维俯视；不依赖 RViz
            preset = TOPIC_PRESETS['stereonet']
            visual_topic = preset['visual']
            points_topic = preset['points']
            self._stereo_official = True
            self.create_subscription(
                Image, preset['depth'], self._on_stereo_depth, _QOS_STEREO_DEPTH)
            self.create_subscription(
                Image, visual_topic, self._on_stereo_visual, _QOS_STEREO_DEPTH)
            self.create_subscription(
                PointCloud2, points_topic, self._on_stereo_points, _QOS_STEREO_DEPTH)
            self.create_timer(0.2, self._tick_stereo_official_wait)
            self._stereo_depth_ok = False
            self._stereo_visual_ok = False
            self._stereo_points_ok = False
            self._wait_t0 = time.monotonic()
            self.get_logger().info(
                f'数据源=stereonet(BPU) OpenCV=深彩|三维俯视 '
                f'visual={visual_topic} points={points_topic}')
            self.get_logger().info(
                '等待 /StereoNetNode/stereonet_visual 与 stereonet_pointcloud2')
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
            self.create_timer(0.1, self._tick_publish)
            self.get_logger().info(
                f'数据源={source} depth={depth_topic} color={color_topic}')

    def _on_stereo_depth(self, msg: Image):
        """官方原始深度（通常 mono16/mm）原样转发，保留原始时间戳。"""
        self._stereo_depth_ok = True
        self._last_depth_msg = msg
        try:
            self.depth_raw_pub.publish(msg)
        except Exception:
            pass

    def _on_stereo_visual(self, msg: Image):
        """官方深彩 → 左栏；转发 /drone/depth/image。"""
        self._stereo_visual_ok = True
        try:
            self.depth_viz_pub.publish(msg)
        except Exception:
            pass
        try:
            enc = (msg.encoding or '').lower()
            if enc in ('nv12', 'yuv420', 'yuv420p'):
                viz = image_msg_to_bgr(msg, self.bridge)
            else:
                viz = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_visual = viz
            self._show_depth_map_panel()
        except Exception as exc:
            self.get_logger().warn(
                f'官方深彩显示失败: {exc}', throttle_duration_sec=2.0)

    def _on_stereo_points(self, msg: PointCloud2):
        """官方彩色点云。RViz 直接订官方话题；仅在需要时做轻量处理。"""
        self._stereo_points_ok = True
        if not self.publish_filtered_cloud and not self.map_enable and not self.out.enabled():
            return
        now = time.monotonic()
        if now - getattr(self, '_last_pts_pub', 0.0) < self._pub_min_dt:
            return
        self._last_pts_pub = now
        try:
            # depth 模式只显示深彩图，不再为无用的 3D panel 做 PointCloud2→numpy 转换。
            need_xyz = self.map_enable or self.out.enabled() and self.panel_mode != 'depth'
            if not need_xyz:
                return
            out = self._downsample_cloud_xyz(msg, max_points=6000)
            if self.publish_filtered_cloud:
                self.cloud_pub.publish(out)
            pts = self._xyz_array_from_cloud(out)
            if self.map_enable and pts.size:
                self._update_map(pts)
                map_msg = points_to_cloud2_xyz(
                    self._map_pts, stamp=msg.header.stamp,
                    frame_id=msg.header.frame_id or 'camera_link')
                self.map_pub.publish(map_msg)
            if self.out.enabled():
                self._show_depth_map_panel()
            self.get_logger().info(
                f'official points {msg.width * max(1, msg.height)}->{out.width}, '
                f'map={self._map_pts.shape[0]}', throttle_duration_sec=3.0)
        except Exception as exc:
            self.get_logger().warn(
                f'点云/地图失败: {exc}', throttle_duration_sec=2.0)

    def _xyz_array_from_cloud(self, msg: PointCloud2) -> np.ndarray:
        if msg.width <= 0:
            return np.zeros((0, 3), dtype=np.float32)
        step = int(msg.point_step)
        n = int(msg.width)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        flat = buf[: n * step].reshape(n, step)
        # 本节点发出的是 xyz-only，offset 0/4/8
        pts = np.empty((n, 3), dtype=np.float32)
        pts[:, 0] = flat[:, 0:4].view(np.float32).reshape(-1)
        pts[:, 1] = flat[:, 4:8].view(np.float32).reshape(-1)
        pts[:, 2] = flat[:, 8:12].view(np.float32).reshape(-1)
        return pts

    def _update_map(self, pts: np.ndarray):
        z = pts[:, 2]
        r2 = pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2
        m = ((z > self.min_range)
             & (r2 <= self.max_range * self.max_range)
             & np.isfinite(r2))
        pts = pts[m]
        if pts.size == 0:
            return

        inv = 1.0 / self._map_voxel
        keys = np.floor(pts * inv).astype(np.int32)
        # 一个体素只保留一个代表点；避免每帧 vstack + unique 整张历史地图。
        uniq, idx = np.unique(keys, axis=0, return_index=True)
        for k, p in zip(uniq, pts[idx]):
            self._map_voxels[(int(k[0]), int(k[1]), int(k[2]))] = p

        if len(self._map_voxels) > self._map_max:
            # 地图超过上限时按距离保留最近体素，避免无限增长。
            keys_list = list(self._map_voxels.keys())
            vals = np.asarray([self._map_voxels[k] for k in keys_list], dtype=np.float32)
            d2 = np.sum(vals * vals, axis=1)
            keep = np.argpartition(d2, self._map_max - 1)[:self._map_max]
            self._map_voxels = {
                keys_list[int(i)]: vals[int(i)] for i in keep
            }

        if self._map_voxels:
            self._map_pts = np.asarray(
                list(self._map_voxels.values()), dtype=np.float32)
        else:
            self._map_pts = np.zeros((0, 3), dtype=np.float32)

    def _show_depth_map_panel(self):
        """OpenCV 双栏：深彩 | 三维俯视（相机系 x 右、z 前）。"""
        if not self.out.enabled():
            return
        now = time.monotonic()
        if now - self._last_panel < 0.12:
            return
        self._last_panel = now
        left = self._last_visual
        if left is None:
            return
        # 俯视：x→右, z→前（图像上）
        if self._map_pts.size:
            pts_plot = np.stack(
                [self._map_pts[:, 0], self._map_pts[:, 2], -self._map_pts[:, 1]],
                axis=1)
        else:
            pts_plot = np.zeros((0, 3), dtype=np.float32)
        right = _project_top(pts_plot, size=max(240, left.shape[0]), span=6.0)
        cv2.putText(right, f'MAP3D n={self._map_pts.shape[0]}', (8, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        h = min(left.shape[0], right.shape[0])
        left_r = cv2.resize(left, (int(left.shape[1] * h / left.shape[0]), h))
        right_r = cv2.resize(right, (h, h))
        panel = np.hstack([left_r, right_r])
        cv2.putText(panel, 'DEPTH | MAP3D', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240, 240, 240), 2, cv2.LINE_AA)
        self.out.output(panel)

    def _downsample_cloud_xyz(self, msg: PointCloud2, max_points=8000):
        """从官方 xyz(+rgb) 点云抽稀，只发 xyz。"""
        names = {f.name: f for f in msg.fields}
        if not all(k in names for k in ('x', 'y', 'z')):
            return msg
        off = {k: names[k].offset for k in ('x', 'y', 'z')}
        step = int(msg.point_step)
        n = int(msg.width) * max(1, int(msg.height))
        if n <= 0:
            return msg
        stride = max(1, (n + max_points - 1) // max_points)
        idx = np.arange(0, n, stride, dtype=np.int32)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        flat = buf[: n * step].reshape(n, step)
        pts = np.empty((idx.size, 3), dtype=np.float32)
        for i, ax in enumerate(('x', 'y', 'z')):
            o = off[ax]
            pts[:, i] = flat[idx, o:o + 4].view(np.float32).reshape(-1)
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(pts.shape[0])
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * out.width
        out.is_dense = True
        out.data = pts.tobytes()
        return out

    def _tick_stereo_official_wait(self):
        if self._stereo_depth_ok and self._stereo_visual_ok and self._stereo_points_ok:
            return
        miss = []
        if not self._stereo_depth_ok:
            miss.append('depth')
        if not self._stereo_visual_ok:
            miss.append('visual')
        if not self._stereo_points_ok:
            miss.append('pointcloud2')
        self._waiting(
            '等待 Stereonet 深度数据\n'
            + ' / '.join(miss))

    def _on_info(self, msg: CameraInfo):
        if msg.k[0] > 1.0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
        # 右目 P[0,3] 为 +fx*B，基线取绝对值。
        try:
            p3 = float(msg.p[3])
            if abs(p3) > 1.0 and self.fx > 1.0:
                bl = abs(p3) / self.fx
                if 0.02 < bl < 0.25:
                    self.baseline_m = bl
        except Exception:
            pass

    def _on_depth(self, msg: Image):
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_m = depth_msg_to_meters(np.asarray(arr), msg.encoding)
            # 深度一到立即出点云/画面，提高采集频率
            if self.source == 'stereonet':
                self._maybe_publish()
        except Exception as exc:
            self.get_logger().warn(f'深度图转换失败: {exc}', throttle_duration_sec=2.0)

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

    def _apply_rotate_pair(self, depth_m, color_bgr):
        """旋转深度/彩色，返回 (depth, color, fx, fy, cx, cy)，不反复改写 self 内参。"""
        if not self.rotate_cw:
            return (depth_m, color_bgr, self.fx, self.fy, self.cx, self.cy)
        h0, w0 = depth_m.shape[:2]
        depth_r = rotate_bgr(depth_m, self.rotate_cw)
        if color_bgr is not None:
            if color_bgr.shape[:2] != (h0, w0):
                color_bgr = cv2.resize(color_bgr, (w0, h0))
            color_r = rotate_bgr(color_bgr, self.rotate_cw)
        else:
            color_r = None
        fx, fy, cx, cy, _, _ = rotate_intrinsics_cw(
            self.fx, self.fy, self.cx, self.cy, w0, h0, self.rotate_cw)
        cx = (depth_r.shape[1] - 1) * 0.5
        cy = (depth_r.shape[0] - 1) * 0.5
        return depth_r, color_r, fx, fy, cx, cy

    def _maybe_publish(self):
        if self.depth_m is None:
            return
        now = time.monotonic()
        if now - self._last_proc < self._pub_min_dt:
            return
        if self._busy:
            return
        self._busy = True
        self._last_proc = now
        try:
            depth = self.depth_m
            color = self.color_bgr
            fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
            if self._need_rotate_depth and self.rotate_cw:
                depth, color, fx, fy, cx, cy = self._apply_rotate_pair(
                    depth, color)
            if color is not None and color.shape[:2] != depth.shape[:2]:
                color = cv2.resize(color, (depth.shape[1], depth.shape[0]))
            self._process(depth, color, fx=fx, fy=fy, cx=cx, cy=cy)
        finally:
            self._busy = False

    def _tick_publish(self):
        if self._stereo_official:
            return
        if self.depth_m is None:
            if self.source == 'stereonet':
                self._waiting('等待 Stereonet 深度图')
            return
        self._maybe_publish()

    def _on_combine(self, msg: Image):
        try:
            self._combine_bgr = image_msg_to_bgr(msg, self.bridge)
            self._combine_stamp = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(
                f'双目拼接图转换失败: {exc}', throttle_duration_sec=2.0)

    def _smooth_depth(self, depth_m):
        d = clean_depth_m(depth_m, self.min_range, self.max_range)
        if self._depth_ema is None or self._depth_ema.shape != d.shape:
            self._depth_ema = d
            return d
        a = 0.4
        prev = self._depth_ema
        both = (d > 0) & (prev > 0)
        self._depth_ema = np.where(
            both, a * d + (1.0 - a) * prev, np.where(d > 0, d, prev))
        return self._depth_ema

    def _depth_panel(self, depth_m, color_bgr):
        viz = colorize_depth(depth_m, self.max_range, self.min_range)
        if color_bgr is not None:
            if color_bgr.shape[:2] != viz.shape[:2]:
                color_bgr = cv2.resize(color_bgr, (viz.shape[1], viz.shape[0]))
            viz = cv2.addWeighted(color_bgr, 0.45, viz, 0.55, 0)
        valid = depth_m[(depth_m > self.min_range) & (depth_m <= self.max_range)]
        tag = 'BPU' if self.source == 'stereonet' else self.source
        cv2.putText(viz, f'depth [{tag}]', (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240, 240, 240), 2, cv2.LINE_AA)
        if valid.size:
            cv2.putText(
                viz,
                f'{float(np.percentile(valid, 5)):.2f}-{float(np.percentile(valid, 95)):.2f}m',
                (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1,
                cv2.LINE_AA)
        return viz

    def _process(self, depth_m, color_bgr, fx=None, fy=None, cx=None, cy=None):
        fx = self.fx if fx is None else fx
        fy = self.fy if fy is None else fy
        cx = self.cx if cx is None else cx
        cy = self.cy if cy is None else cy
        depth_m = self._smooth_depth(depth_m)
        color_for_pts = None if self.panel_mode in ('cloud', 'depth') else color_bgr
        pts, _cols = depth_to_points(
            depth_m, fx, fy, cx, cy,
            stride=self.stride,
            min_range=self.min_range,
            max_range=self.max_range,
            color_bgr=color_for_pts)
        if len(pts) < 80 and self._last_pts is not None:
            pts = self._last_pts
        else:
            self._last_pts = pts
        stamp = self.get_clock().now().to_msg()
        cloud = points_to_cloud2_xyz(pts, stamp=stamp)
        self.cloud_pub.publish(cloud)
        self.get_logger().info(
            f'published /drone/depth/points N={len(pts)}',
            throttle_duration_sec=2.0)

        viz = self._depth_panel(depth_m, color_bgr)
        if self.panel_mode in ('depth', 'depth_cloud', 'full'):
            try:
                self.depth_viz_pub.publish(
                    self.bridge.cv2_to_imgmsg(viz, encoding='bgr8'))
            except Exception:
                pass
        if self.out.enabled():
            if self.panel_mode == 'depth':
                panel = viz
            else:
                panel = render_modeling_panel(
                    depth_m, pts,
                    color_bgr=None if self.panel_mode == 'cloud' else color_bgr,
                    title=f'depth modeling [{self.source}]',
                    panel_mode=self.panel_mode)
            self.out.output(panel)

    def _waiting(self, text: str):
        if not self.out.enabled():
            self.get_logger().warn(text.replace('\n', ' | '),
                                   throttle_duration_sec=3.0)
            return
        panel = np.zeros((300, 720, 3), np.uint8)
        panel[:] = (32, 34, 38)
        for i, line in enumerate(text.split('\n')):
            cv2.putText(panel, line, (20, 50 + i * 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 2)
        self.out.output(panel)

    def _tick_simulate(self):
        w, h = 320, 240
        self.fx = DEFAULT_FX * (w / 640.0)
        self.fy = DEFAULT_FY * (h / 480.0)
        self.cx = (w - 1) * 0.5
        self.cy = (h - 1) * 0.5
        depth = simulate_depth_room(
            width=w, height=h, t=self._sim.now(),
            fx=DEFAULT_FX, fy=DEFAULT_FY, noise_std=0.0)
        if not hasattr(self, '_sim_rgb') or self._sim_rgb is None:
            rgb = np.zeros((h, w, 3), np.uint8)
            rgb[:] = (40, 40, 50)
            cv2.rectangle(rgb, (90, 60), (230, 200), (70, 70, 90), -1)
            cv2.putText(rgb, 'SIM RGB', (100, 140), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (160, 160, 180), 2)
            self._sim_rgb = rgb
        self._process(depth, self._sim_rgb)

    def _tick_stereo(self):
        if self._busy:
            return
        if self._combine_bgr is None:
            self._waiting('等待 MIPI 拼接图 /image_combine_raw')
            return
        if time.monotonic() - self._combine_stamp > 2.0:
            self._waiting('MIPI 画面中断')
            return
        self._busy = True
        try:
            left, right = split_stereo_combine(
                self._combine_bgr, layout=self.stereo_layout)
            h0, w0 = left.shape[:2]
            if self.fx < 10:
                self.fx = DEFAULT_FX * (w0 / 640.0)
                self.fy = DEFAULT_FY * (h0 / 480.0)
                self.cx = (w0 - 1) * 0.5
                self.cy = (h0 - 1) * 0.5
            if self.rotate_cw:
                left = rotate_bgr(left, self.rotate_cw)
                right = rotate_bgr(right, self.rotate_cw)
                self.fx, self.fy, self.cx, self.cy, _, _ = rotate_intrinsics_cw(
                    self.fx, self.fy, self.cx, self.cy, w0, h0, self.rotate_cw)
            depth, color, fx_s, scale = stereo_to_depth_m(
                left, right, fx=self.fx, baseline_m=self.baseline_m,
                max_width=self.stereo_max_width,
                matcher=self.stereo_matcher)
            self.fx = fx_s
            self.fy = self.fy * scale if scale != 1.0 else self.fy
            self.cx = (color.shape[1] - 1) * 0.5
            self.cy = (color.shape[0] - 1) * 0.5
            self.depth_m = depth
            self.color_bgr = color
            self._process(depth, color)
        except Exception as exc:
            self.get_logger().warn(
                f'双目点云失败: {exc}', throttle_duration_sec=2.0)
        finally:
            self._busy = False


def main(args=None):
    parser = argparse.ArgumentParser(description='深度相机点云建模')
    parser.add_argument('--source', default='stereonet',
                        choices=['simulate', 'orbbec', 'realsense',
                                 'mipi_stereo', 'stereonet'])
    parser.add_argument('--depth-topic', default='')
    parser.add_argument('--color-topic', default='')
    parser.add_argument('--info-topic', default='')
    parser.add_argument('--combine-topic', default='')
    parser.add_argument('--show', action='store_true', default=True)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--snapshot', default='none',
                        help='none=不存图，只实时窗口/话题流')
    parser.add_argument('--snapshot-period', type=float, default=2.0)
    parser.add_argument('--stride', type=int, default=2)
    parser.add_argument('--max-range', type=float, default=5.0)
    parser.add_argument('--min-range', type=float, default=0.4,
                        help='近距裁剪（米），去掉原点附近双目噪点')
    parser.add_argument('--baseline-m', type=float, default=0.07917,
                        help='MIPI 双目基线（米），按模组说明书微调')
    parser.add_argument('--stereo-layout', default='tb', choices=['tb', 'lr'])
    parser.add_argument('--stereo-max-width', type=int, default=320)
    parser.add_argument('--stereo-matcher', default='bm',
                        choices=['bm', 'sgbm'],
                        help='仅 mipi_stereo：bm 更快')
    parser.add_argument('--stereo-period', type=float, default=1.0,
                        help='双目处理周期（秒）')
    parser.add_argument('--panel-mode', default='depth',
                        choices=['depth', 'cloud', 'depth_cloud', 'full'],
                        help='depth=深度伪彩图示；depth_cloud=深度+俯视；full=三栏')
    parser.add_argument('--rotate-cw', type=int, default=0,
                        choices=[0, 90, 180, 270],
                        help='深度/彩色顺时针旋转；stereonet 官方链路请用 0')
    parser.add_argument('--opencv-fallback', action='store_true',
                        help='Stereonet 超时后回退 CPU 立体匹配')
    parser.add_argument('--map-enable', dest='map_enable', action='store_true',
                        help='启用点云累积地图；默认关闭，避免无位姿时错误累积')
    parser.add_argument('--no-map-enable', dest='map_enable', action='store_false')
    parser.set_defaults(map_enable=False)
    parser.add_argument('--no-filtered-cloud', action='store_true',
                        help='不发布 /drone/depth/points（RViz 推荐直接订官方点云）')
    parser.add_argument('--publish-hz', type=float, default=4.0,
                        help='处理/转发点云最高频率，默认 4Hz')
    parsed, ros_args = parser.parse_known_args(args)
    show = parsed.show and not parsed.no_show

    rclpy.init(args=ros_args)
    node = DepthPointCloudNode(
        source=parsed.source,
        depth_topic=_topic_or_none(parsed.depth_topic),
        color_topic=_topic_or_none(parsed.color_topic),
        info_topic=_topic_or_none(parsed.info_topic),
        combine_topic=_topic_or_none(parsed.combine_topic),
        show=show,
        snapshot=_topic_or_none(parsed.snapshot),
        snapshot_period=parsed.snapshot_period,
        stride=parsed.stride,
        max_range=parsed.max_range,
        min_range=parsed.min_range,
        baseline_m=parsed.baseline_m,
        stereo_layout=parsed.stereo_layout,
        stereo_max_width=parsed.stereo_max_width,
        rotate_cw=parsed.rotate_cw,
        panel_mode=parsed.panel_mode,
        stereo_matcher=parsed.stereo_matcher,
        stereo_period=parsed.stereo_period,
        opencv_fallback=parsed.opencv_fallback,
        map_enable=parsed.map_enable,
        publish_filtered_cloud=not parsed.no_filtered_cloud,
        publish_hz=parsed.publish_hz,
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
        # Ctrl+C / launch SIGINT 时 rclpy 可能已 shutdown，避免二次调用报错
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
