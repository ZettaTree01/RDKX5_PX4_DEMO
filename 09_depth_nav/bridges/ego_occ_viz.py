#!/usr/bin/env python3
"""把 world 点云体素化后发给 RViz（/drone/ego/occ_viz）。

EGO 的 /grid_map/occupancy 在纯点云模式下常为空，本节点提供稳定可视化。
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2)


def _read_xyz(msg: PointCloud2, max_n: int = 12000) -> np.ndarray:
    names = {f.name: f for f in msg.fields}
    if not all(k in names for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), np.float32)
    off = {k: names[k].offset for k in ('x', 'y', 'z')}
    step = int(msg.point_step)
    n = int(msg.width) * max(1, int(msg.height))
    if n <= 0:
        return np.zeros((0, 3), np.float32)
    stride = max(1, (n + max_n - 1) // max_n)
    idx = np.arange(0, n, stride, dtype=np.int32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    flat = buf[: n * step].reshape(n, step)
    pts = np.empty((idx.size, 3), np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        o = off[ax]
        pts[:, i] = flat[idx, o:o + 4].view(np.float32).reshape(-1)
    return pts


def _to_cloud(header, pts: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(len(pts))
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    if len(pts) == 0:
        msg.data = b''
    else:
        msg.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
    return msg


class EgoOccViz(Node):
    def __init__(self):
        super().__init__('ego_occ_viz')
        self.declare_parameter('in_topic', '/drone/ego/cloud_world')
        self.declare_parameter('out_topic', '/drone/ego/occ_viz')
        self.declare_parameter('resolution', 0.15)
        self.declare_parameter('min_period', 0.25)
        self.declare_parameter('z_min', -0.4)
        self.declare_parameter('z_max', 1.6)
        inn = self.get_parameter('in_topic').value
        out = self.get_parameter('out_topic').value
        self.res = float(self.get_parameter('resolution').value)
        self.min_period = float(self.get_parameter('min_period').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self._last_t = 0.0
        self.pub = self.create_publisher(PointCloud2, out, _QOS)
        self.create_subscription(
            PointCloud2, inn, self._on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f'{inn} → voxel {out} res={self.res:.2f}m')

    def _on_cloud(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self._last_t < self.min_period:
            return
        self._last_t = now
        pts = _read_xyz(msg)
        if pts.size == 0:
            return
        zok = (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
        pts = pts[zok]
        if pts.size == 0:
            return
        # 体素中心
        keys = np.floor(pts / self.res).astype(np.int32)
        # 唯一体素
        flat = keys[:, 0].astype(np.int64) * 73856093 ^ keys[:, 1].astype(
            np.int64) * 19349663 ^ keys[:, 2].astype(np.int64) * 83492791
        _, uniq = np.unique(flat, return_index=True)
        keys = keys[uniq]
        if len(keys) > 6000:
            keys = keys[:: max(1, len(keys) // 6000)]
        centers = (keys.astype(np.float32) + 0.5) * self.res
        header = msg.header
        self.pub.publish(_to_cloud(header, centers))


def main():
    rclpy.init()
    node = EgoOccViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
