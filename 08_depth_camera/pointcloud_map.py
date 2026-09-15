#!/usr/bin/env python3
"""体素点云 / 局部三维地图（轻量，适合板端 RViz）。

PointCloud2 → 体素滤波(+可选累积) → /drone/map/points
默认稀疏：voxel=0.05m、上限约 1.2 万点、约 2Hz，避免 RViz 卡死。
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2)


def _xyz_from_cloud(msg: PointCloud2) -> np.ndarray:
    if msg.width == 0 or not msg.fields:
        return np.zeros((0, 3), dtype=np.float32)
    names = {f.name: f for f in msg.fields}
    if not all(k in names for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)
    off = {k: names[k].offset for k in ('x', 'y', 'z')}
    step = int(msg.point_step)
    n = int(msg.width) * max(1, int(msg.height))
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if buf.size < n * step:
        n = buf.size // max(1, step)
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float32)
    flat = buf[: n * step].reshape(n, step)
    pts = np.empty((n, 3), dtype=np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        o = off[ax]
        pts[:, i] = flat[:, o:o + 4].view(np.float32).reshape(-1)
    return pts


def _cloud_xyz(header: Header, pts: np.ndarray) -> PointCloud2:
    pts = np.ascontiguousarray(pts, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = pts.tobytes()
    return msg


def _voxel_unique(pts: np.ndarray, voxel: float) -> np.ndarray:
    """向量化体素去重，返回体素中心附近代表点。"""
    if pts.size == 0:
        return pts
    inv = 1.0 / voxel
    keys = np.floor(pts * inv).astype(np.int32)
    # 打包三维索引 → 1D，便于 np.unique
    # 偏移避免负坐标冲突
    keys = keys - keys.min(axis=0, keepdims=True)
    span = keys.max(axis=0) + 1
    flat = (keys[:, 0].astype(np.int64)
            + keys[:, 1].astype(np.int64) * int(span[0])
            + keys[:, 2].astype(np.int64) * int(span[0] * span[1]))
    _, idx = np.unique(flat, return_index=True)
    return pts[idx]


class PointCloudMapNode(Node):
    def __init__(self, in_topic, out_topic, voxel_size, max_points,
                 accumulate, max_range, min_range, frame_id, pub_hz):
        super().__init__('pointcloud_map')
        self.voxel = max(0.02, float(voxel_size))
        self.max_points = max(500, int(max_points))
        self.accumulate = bool(accumulate)
        self.max_range = float(max_range)
        self.min_range = float(min_range)
        self.frame_id = frame_id
        self._pub_dt = 1.0 / max(0.5, float(pub_hz))
        self._map_pts = np.zeros((0, 3), dtype=np.float32)
        self._last_pub = 0.0
        self._busy = False
        self.pub = self.create_publisher(PointCloud2, out_topic, _QOS)
        self.create_subscription(PointCloud2, in_topic, self._on_cloud, _QOS)
        self.get_logger().info(
            f'体素地图 in={in_topic} out={out_topic} '
            f'voxel={self.voxel:.3f}m max={self.max_points} '
            f'accumulate={self.accumulate} pub≈{1.0 / self._pub_dt:.1f}Hz '
            f'range=[{self.min_range:.2f},{self.max_range:.2f}]m')

    def _on_cloud(self, msg: PointCloud2):
        if self._busy:
            return
        now = time.monotonic()
        if now - self._last_pub < self._pub_dt:
            return
        self._busy = True
        try:
            pts = _xyz_from_cloud(msg)
            if pts.size == 0:
                return
            # 预下采样，加速后续体素
            if pts.shape[0] > 20000:
                pts = pts[:: max(1, pts.shape[0] // 20000)]
            z = pts[:, 2]
            r2 = pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2
            m = ((z > self.min_range)
                 & (r2 <= self.max_range * self.max_range)
                 & np.isfinite(r2))
            pts = pts[m]
            if pts.size == 0:
                return
            frame = _voxel_unique(pts, self.voxel)
            if self.accumulate and self._map_pts.size:
                merged = np.vstack((self._map_pts, frame))
                out = _voxel_unique(merged, self.voxel)
            else:
                out = frame
            if out.shape[0] > self.max_points:
                # 保留最近的点（按到原点距离排序截断，比随机更稳）
                d2 = out[:, 0] ** 2 + out[:, 1] ** 2 + out[:, 2] ** 2
                keep = np.argpartition(d2, self.max_points)[: self.max_points]
                out = out[keep]
            self._map_pts = out if self.accumulate else out
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.frame_id or (
                msg.header.frame_id or 'camera_link')
            self.pub.publish(_cloud_xyz(header, self._map_pts))
            self._last_pub = now
            self.get_logger().info(
                f'map voxels={self._map_pts.shape[0]} '
                f'(in={msg.width})',
                throttle_duration_sec=3.0)
        except Exception as exc:
            self.get_logger().warn(
                f'map failed: {exc}', throttle_duration_sec=2.0)
        finally:
            self._busy = False


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--in-topic', default='/drone/depth/points')
    p.add_argument('--out-topic', default='/drone/map/points')
    p.add_argument('--voxel-size', type=float, default=0.05)
    p.add_argument('--max-points', type=int, default=12000)
    p.add_argument('--accumulate', action='store_true', default=True)
    p.add_argument('--no-accumulate', action='store_false', dest='accumulate')
    p.add_argument('--max-range', type=float, default=5.0)
    p.add_argument('--min-range', type=float, default=0.3)
    p.add_argument('--frame-id', default='camera_link')
    p.add_argument('--pub-hz', type=float, default=2.0)
    args, _ = p.parse_known_args()

    rclpy.init()
    node = PointCloudMapNode(
        args.in_topic, args.out_topic, args.voxel_size, args.max_points,
        args.accumulate, args.max_range, args.min_range, args.frame_id,
        args.pub_hz)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
