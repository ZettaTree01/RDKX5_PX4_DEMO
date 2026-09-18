#!/usr/bin/env python3
"""MAVROS local pose → nav_msgs/Odometry（EGO 需要 /odom_world）+ TF。

无位姿时仍发布 identity odom/TF，保证 RViz Fixed Frame=world 能显示
camera_link 点云，且 grid_map 不因 no odom 拒收。

z_align=True（例程 10 EGO 用）：室内 MAVROS local 会带着气压绝对高度
和上次飞行残留的 XY。EGO grid_map 固定在原点附近
（默认 XY∈[-4,4]、Z∈[-0.5,1.5]），出界时 getInflateOccupancy 返回 -1，
C++ 里 if(occ) 把 -1 当成障碍，规划必败。
以首个稳定位姿为原点 (x0,y0,z0)，EGO 世界系 = MAVROS - origin；
原点经 latched /drone/ego/origin_ref 与 /drone/ego/z_ref 广播，
cloud/goal/poscmd 各桥按同一基准变换。锁定前发 (0,0,0)，避免建脏图。
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

_LATCHED = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)

# 收到这么多次真实位姿后锁定 z 基准（10 Hz 下约 5 s，等 EKF/视觉对齐稳定）
Z_LOCK_FRAMES = 50


class PoseToOdom(Node):
    """MAVROS 位姿 → Odometry + TF；可选 z 对齐供 EGO grid_map。"""

    def __init__(self):
        """声明话题/坐标系参数；无位姿时定时发 identity。"""
        super().__init__('pose_to_odom')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('odom_topic', '/odom_world')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('also_camera_link', True)
        self.declare_parameter('fallback_hz', 10.0)
        self.declare_parameter('z_align', False)
        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.also_camera = bool(self.get_parameter('also_camera_link').value)
        self.z_align = bool(self.get_parameter('z_align').value)
        self._z_frames = 0
        self._origin = None  # (x0, y0, z0) MAVROS → EGO
        self._last = None
        self._have_pose = False
        self.pub = self.create_publisher(Odometry, odom_topic, 20)
        self.zref_pub = self.create_publisher(
            Float64, '/drone/ego/z_ref', _LATCHED)
        self.origin_pub = self.create_publisher(
            PoseStamped, '/drone/ego/origin_ref', _LATCHED)
        self.tf_br = TransformBroadcaster(self)
        self.create_subscription(
            PoseStamped, pose_topic, self._cb, qos_profile_sensor_data)
        hz = float(self.get_parameter('fallback_hz').value)
        self.create_timer(max(0.05, 1.0 / max(hz, 1.0)), self._fallback_tick)
        self.get_logger().info(
            f'{pose_topic} → {odom_topic} + TF（无位姿时发 identity）')

    def _publish(self, stamp, position, orientation, twist=None):
        """发布 Odometry，并广播 base_link（及可选 camera_link）TF。"""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation = orientation
        if twist is not None:
            odom.twist.twist.linear.x = float(twist[0])
            odom.twist.twist.linear.y = float(twist[1])
            odom.twist.twist.linear.z = float(twist[2])
        self.pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id
        tf.transform.translation.x = float(position[0])
        tf.transform.translation.y = float(position[1])
        tf.transform.translation.z = float(position[2])
        tf.transform.rotation = orientation
        self.tf_br.sendTransform(tf)
        if self.also_camera:
            cam = TransformStamped()
            cam.header = tf.header
            cam.child_frame_id = 'camera_link'
            cam.transform = tf.transform
            self.tf_br.sendTransform(cam)

    def _fallback_tick(self):
        """尚无真实位姿时发布原点 identity，保证 RViz/grid_map 可用。"""
        if self._have_pose:
            return
        stamp = self.get_clock().now().to_msg()
        q = Quaternion()
        q.w = 1.0
        self._publish(stamp, (0.0, 0.0, 0.0), q)

    def _cb(self, msg: PoseStamped):
        """位姿回调：差分估速，可选锁定 z0 后发对齐高度。"""
        self._have_pose = True
        p = msg.pose.position
        twist = None
        now = self.get_clock().now()
        if self._last is not None:
            dt = (now - self._last[0]).nanoseconds * 1e-9
            if dt > 1e-4:
                p0 = self._last[1]
                twist = (
                    (p.x - p0.x) / dt,
                    (p.y - p0.y) / dt,
                    (p.z - p0.z) / dt,
                )
        self._last = (now, p)
        x, y, z = float(p.x), float(p.y), float(p.z)
        if self.z_align:
            if self._origin is None:
                self._z_frames += 1
                if self._z_frames >= Z_LOCK_FRAMES:
                    self._origin = (x, y, z)
                    oref = PoseStamped()
                    oref.header.stamp = now.to_msg()
                    oref.header.frame_id = 'map'
                    oref.pose.position.x = x
                    oref.pose.position.y = y
                    oref.pose.position.z = z
                    oref.pose.orientation.w = 1.0
                    self.origin_pub.publish(oref)
                    ref = Float64()
                    ref.data = z
                    self.zref_pub.publish(ref)
                    self.get_logger().info(
                        f'EGO 原点已锁定 ({x:.3f},{y:.3f},{z:.3f})，'
                        f'EGO 世界系 = MAVROS local - origin')
                # 锁定前停在地图原点，避免残留 XY 直接出界
                x, y, z = 0.0, 0.0, 0.0
            else:
                x0, y0, z0 = self._origin
                x, y, z = x - x0, y - y0, z - z0
        # 用本机时钟，便于与 Stereonet depth 做 ApproximateTime 同步建图
        stamp = now.to_msg()
        self._publish(
            stamp,
            (x, y, z),
            msg.pose.orientation,
            twist,
        )


def main():
    """启动 pose_to_odom 桥接节点。"""
    rclpy.init()
    node = PoseToOdom()
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
