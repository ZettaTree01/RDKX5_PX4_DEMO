#!/usr/bin/env python3
"""把 Stereonet 相机系点云变换到 world，供 EGO grid_map/cloud 使用。

Stereonet: frame=camera_link，点为 ROS 相机系 (x前 y左 z上)。
台架下 map≈world≈机体起飞系；用 MAVROS pose 将点变到 world。

z_align=True（例程 10 EGO 用）：点云 world z 减去 pose_to_odom 广播的
z 基准（/drone/ego/z_ref），与 odom 同系，保证 grid_map（地图 z 固定
[-0.5, 1.5]）建图与碰撞检查一致。基准未广播前不发布点云。
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64


_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2)

_LATCHED = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)


def _yaw(q) -> float:
    """四元数 → 偏航角（ENU）。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _read_xyz(msg: PointCloud2, max_n: int = 20000) -> np.ndarray:
    """抽稀读取点云 xyz。"""
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
    """标准 XYZ32，保证 pcl::fromROSMsg 能解析（EGO grid_map 依赖）。"""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    n = int(len(pts))
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    if n == 0:
        msg.data = b''
    else:
        # 去掉 NaN，避免 PCL 整云丢弃
        clean = np.ascontiguousarray(pts, dtype=np.float32)
        ok = np.isfinite(clean).all(axis=1)
        clean = clean[ok]
        msg.width = int(len(clean))
        msg.row_step = 12 * msg.width
        msg.data = clean.tobytes()
    return msg


class CloudCamToWorld(Node):
    """相机系点云 × 机体位姿 → world，供 EGO grid_map。"""

    def __init__(self):
        """无 MAVROS 时用原点姿态，便于台架监视建图。"""
        super().__init__('cloud_cam_to_world')
        self.declare_parameter('in_topic', '/StereoNetNode/stereonet_pointcloud2')
        self.declare_parameter('out_topic', '/drone/ego/cloud_world')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('max_points', 8000)
        self.declare_parameter('min_period', 0.2)
        self.declare_parameter('z_align', False)
        # 无 MAVROS 位姿时用原点单位姿态，保证台架监视也能建 grid_map
        self.pose = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self._have_mavros_pose = False
        self._last_t = 0.0
        self.z_align = bool(self.get_parameter('z_align').value)
        self._z0 = None
        inn = self.get_parameter('in_topic').value
        out = self.get_parameter('out_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.max_points = int(self.get_parameter('max_points').value)
        self.min_period = float(self.get_parameter('min_period').value)
        self.pub = self.create_publisher(PointCloud2, out, _QOS)
        # Stereonet 点云多为 Best Effort，用 sensor QoS 才能稳定收到
        self.create_subscription(
            PointCloud2, inn, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, qos_profile_sensor_data)
        if self.z_align:
            self.create_subscription(
                Float64, '/drone/ego/z_ref', self._on_zref, _LATCHED)
        self.get_logger().info(
            f'{inn} + pose → {out} ({self.world_frame}) '
            f'max_pts={self.max_points} period>={self.min_period:.2f}s '
            f'(identity pose until mavros)'
            + (f' z_align=True' if self.z_align else ''))

    def _on_zref(self, msg: Float64):
        """接收 z 对齐基准。"""
        if self._z0 is None:
            self._z0 = float(msg.data)
            self.get_logger().info(f'z 对齐基准 z0={self._z0:.3f}')

    def _on_pose(self, msg: PoseStamped):
        """缓存最新局部位姿与偏航。"""
        p = msg.pose.position
        self.pose = (float(p.x), float(p.y), float(p.z))
        self.yaw = _yaw(msg.pose.orientation)
        self._have_mavros_pose = True

    def _on_cloud(self, msg: PointCloud2):
        """点云回调：限频、可选等 z0，再变换到 world 发布。"""
        now = time.monotonic()
        if now - self._last_t < self.min_period:
            return
        self._last_t = now
        if self.z_align and self._z0 is None:
            return  # 等 pose_to_odom 锁定并广播 z 基准，避免建脏图
        pts = _read_xyz(msg, max_n=self.max_points)
        if pts.size == 0:
            return
        # ROS 相机/机体 FLU：x前 y左 z上 → ENU world
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x_b, y_b, z_b = pts[:, 0], pts[:, 1], pts[:, 2]
        ox, oy, oz = self.pose
        if self.z_align:
            oz = oz - self._z0
        out = np.empty_like(pts)
        out[:, 0] = c * x_b - s * y_b + ox
        out[:, 1] = s * x_b + c * y_b + oy
        out[:, 2] = z_b + oz
        header = msg.header
        header.frame_id = self.world_frame
        self.pub.publish(_to_cloud(header, out))


def main():
    """启动 cloud_cam_to_world 桥接节点。"""
    rclpy.init()
    node = CloudCamToWorld()
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
