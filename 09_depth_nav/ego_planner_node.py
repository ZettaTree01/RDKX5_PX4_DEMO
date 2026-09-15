#!/usr/bin/env python3
"""例程9 EGO 局部规划桥接节点。

输入：Stereonet 点云 + MAVROS 位姿 + /drone/ego/goal
输出（供 RViz2，对齐 Ego-Planner-System 可视化习惯）：
  /drone/ego/occupancy          占据点云地图
  /drone/ego/occupancy_inflate  膨胀占据
  /drone/ego/a_star_path        A* 折线
  /drone/ego/optimal_path       平滑后局部路径（跟径用）
  /drone/ego/cmd_vel            机体 FLU 建议速度

深彩 / OpenCV 三维仍由 depth_nav 负责，本节点不改动。
参考：https://github.com/Kinang2/Ego-Planner-System
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from ego_local_planner import (
    EgoMapConfig, LocalOccupancyMap, astar_plan_xy, path_follow_body_vel)

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5)


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _xyz_from_cloud(msg: PointCloud2, max_points: int = 12000) -> np.ndarray:
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
    pts = np.empty((idx.size, 3), dtype=np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        o = off[ax]
        pts[:, i] = flat[idx, o:o + 4].view(np.float32).reshape(-1)
    return pts


def _cloud_xyz(header: Header, pts: np.ndarray) -> PointCloud2:
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
    msg.data = b'' if len(pts) == 0 else np.asarray(
        pts, dtype=np.float32).reshape(-1).tobytes()
    return msg


def _path_msg(header: Header, pts) -> Path:
    path = Path()
    path.header = header
    for x, y, z in pts:
        ps = PoseStamped()
        ps.header = header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def _ros_cam_to_map(pts_ros: np.ndarray, pose_xyz, yaw: float) -> np.ndarray:
    """Stereonet ROS 相机系点 (x前 y左 z上) + 机体位姿 → map/ENU。

    台架下 camera_link≈机体，map 与 local ENU 重合（例程 static TF）。
    """
    if pts_ros is None or len(pts_ros) == 0 or pose_xyz is None:
        return np.zeros((0, 3), dtype=np.float32)
    # body FLU ≈ ROS camera: x前 y左 z上
    c, s = math.cos(yaw), math.sin(yaw)
    x_b, y_b, z_b = pts_ros[:, 0], pts_ros[:, 1], pts_ros[:, 2]
    x_w = c * x_b - s * y_b + pose_xyz[0]
    y_w = s * x_b + c * y_b + pose_xyz[1]
    z_w = z_b + pose_xyz[2]
    return np.stack([x_w, y_w, z_w], axis=1).astype(np.float32)


def _smooth_path(path, win: int = 3):
    if len(path) < 3:
        return path
    out = [path[0]]
    for i in range(1, len(path) - 1):
        sl = path[max(0, i - win): min(len(path), i + win + 1)]
        xs = sum(p[0] for p in sl) / len(sl)
        ys = sum(p[1] for p in sl) / len(sl)
        zs = sum(p[2] for p in sl) / len(sl)
        out.append((xs, ys, zs))
    out.append(path[-1])
    return out


class EgoPlannerBridge(Node):
    def __init__(self, cloud_topic: str, max_vel: float = 0.02,
                 resolution: float = 0.15, inflation: float = 0.25):
        super().__init__('ego_planner_bridge')
        self.max_vel = float(max_vel)
        self.pose = None
        self.yaw = 0.0
        self.goal = None
        self._last_cloud_t = 0.0
        self._last_plan_t = 0.0
        self._path = []
        self._astar = []

        cfg = EgoMapConfig(
            resolution=float(resolution),
            inflation=float(inflation),
            local_range_xy=4.0,
            local_range_z=2.0,
        )
        self.occ = LocalOccupancyMap(cfg)

        self.pub_occ = self.create_publisher(
            PointCloud2, '/drone/ego/occupancy', 1)
        self.pub_inf = self.create_publisher(
            PointCloud2, '/drone/ego/occupancy_inflate', 1)
        self.pub_astar = self.create_publisher(Path, '/drone/ego/a_star_path', 1)
        self.pub_opt = self.create_publisher(Path, '/drone/ego/optimal_path', 1)
        self.pub_cmd = self.create_publisher(
            TwistStamped, '/drone/ego/cmd_vel', 10)

        self.create_subscription(
            PointCloud2, cloud_topic, self._on_cloud, _QOS)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/drone/ego/goal', self._on_goal, 10)

        self.create_timer(0.2, self._tick_plan)
        self.create_timer(0.05, self._tick_follow)
        self.get_logger().info(
            f'EGO bridge cloud={cloud_topic} res={resolution} '
            f'inf={inflation} max_vel={max_vel}')

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self.pose = (float(p.x), float(p.y), float(p.z))
        self.yaw = _yaw_from_quat(msg.pose.orientation)

    def _on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        self.goal = (float(p.x), float(p.y), float(p.z))

    def _on_cloud(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self._last_cloud_t < 0.15:
            return
        self._last_cloud_t = now
        if self.pose is None:
            return
        try:
            pts_ros = _xyz_from_cloud(msg, max_points=10000)
            pts_map = _ros_cam_to_map(pts_ros, self.pose, self.yaw)
            self.occ.integrate_points(pts_map, self.pose)
        except Exception as exc:
            self.get_logger().warn(
                f'点云建图失败: {exc}', throttle_duration_sec=2.0)

    def _tick_plan(self):
        if self.pose is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        occ = self.occ.occupied_centers(inflated=False)
        inf = self.occ.occupied_centers(inflated=True)
        self.pub_occ.publish(_cloud_xyz(header, occ))
        self.pub_inf.publish(_cloud_xyz(header, inf))

        goal = self.goal
        if goal is None:
            return
        now = time.monotonic()
        if now - self._last_plan_t < 0.35 and self._path:
            self.pub_astar.publish(_path_msg(header, self._astar))
            self.pub_opt.publish(_path_msg(header, self._path))
            return
        self._last_plan_t = now
        sx, sy, sz = self.pose
        gx, gy, gz = goal
        z_ref = 0.5 * (sz + gz)
        raw = astar_plan_xy(self.occ, (sx, sy), (gx, gy), z_ref)
        self._astar = raw
        self._path = _smooth_path(raw, win=2)
        self.pub_astar.publish(_path_msg(header, self._astar))
        self.pub_opt.publish(_path_msg(header, self._path))
        self.get_logger().info(
            f'EGO plan n={len(self._path)} occ={len(occ)} inf={len(inf)}',
            throttle_duration_sec=2.0)

    def _tick_follow(self):
        if self.pose is None or not self._path:
            return
        vx, vy, vz = path_follow_body_vel(
            self._path, self.pose, self.yaw, self.max_vel)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.pub_cmd.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='EGO local map + A* bridge')
    parser.add_argument(
        '--cloud-topic',
        default='/StereoNetNode/stereonet_pointcloud2')
    parser.add_argument('--max-vel', type=float, default=0.02)
    parser.add_argument('--resolution', type=float, default=0.15)
    parser.add_argument('--inflation', type=float, default=0.25)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = EgoPlannerBridge(
        cloud_topic=parsed.cloud_topic,
        max_vel=parsed.max_vel,
        resolution=parsed.resolution,
        inflation=parsed.inflation,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
