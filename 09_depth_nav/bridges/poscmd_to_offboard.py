#!/usr/bin/env python3
"""EGO traj_server 的 PositionCommand → 例程 Offboard 管理器。

优先发布 /drone/setpoint_position/local（与 offboard_manager 约定一致）。
若未编译到 quadrotor_msgs，则退化为订阅 geometry PoseStamped。

z_align=True（例程 10 EGO 用）：EGO 在对齐后的世界系规划（z_ego ≈ 0），
发布给 offboard 前把 z 加回 /drone/ego/z_ref 广播的基准 z0，回到
MAVROS local 系。基准未收到前丢弃 poscmd。
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool, Float64

try:
    from quadrotor_msgs.msg import PositionCommand
    _HAS_QMSG = True
except ImportError:
    PositionCommand = None
    _HAS_QMSG = False

_LATCHED = QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE)


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PoscmdToOffboard(Node):
    def __init__(self):
        super().__init__('poscmd_to_offboard')
        self.declare_parameter('cmd_topic', '/position_cmd')
        self.declare_parameter('out_topic', '/drone/setpoint_position/local')
        self.declare_parameter('trigger_topic', '/traj_start_trigger')
        self.declare_parameter('wait_airborne', True)
        self.declare_parameter('z_align', False)
        self.airborne = False
        self._sent_trigger = False
        self._last_log = 0.0
        self.z_align = bool(self.get_parameter('z_align').value)
        self._z0 = None
        out = self.get_parameter('out_topic').value
        cmd = self.get_parameter('cmd_topic').value
        self.pub = self.create_publisher(PoseStamped, out, 20)
        self.trig_pub = self.create_publisher(
            PoseStamped, self.get_parameter('trigger_topic').value, 5)
        self.create_subscription(
            Bool, '/drone/status/airborne',
            lambda m: setattr(self, 'airborne', bool(m.data)), 10)
        if self.z_align:
            self.create_subscription(
                Float64, '/drone/ego/z_ref', self._on_zref, _LATCHED)
        if _HAS_QMSG:
            self.create_subscription(PositionCommand, cmd, self._on_poscmd, 20)
            self.get_logger().info(
                f'PositionCommand {cmd} → {out}'
                + (' z_align=True' if self.z_align else ''))
        else:
            # 无消息包时：也可接 PoseStamped 目标
            self.create_subscription(PoseStamped, cmd, self._on_pose, 20)
            self.get_logger().warn(
                '未找到 quadrotor_msgs，按 PoseStamped 转发 ' + cmd)

        self.create_timer(1.0, self._maybe_trigger)

    def _on_zref(self, msg: Float64):
        if self._z0 is None:
            self._z0 = float(msg.data)
            self.get_logger().info(f'z 对齐基准 z0={self._z0:.3f}')

    def _maybe_trigger(self):
        if self._sent_trigger:
            return
        if self.get_parameter('wait_airborne').value and not self.airborne:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        self.trig_pub.publish(msg)
        self._sent_trigger = True
        self.get_logger().info('已发 /traj_start_trigger，启动 EGO 航点规划')

    def _on_poscmd(self, msg):
        if self.get_parameter('wait_airborne').value and not self.airborne:
            return
        if self.z_align and self._z0 is None:
            self.get_logger().warn(
                '等待 z 基准（/drone/ego/z_ref），暂不转发 poscmd',
                throttle_duration_sec=5.0)
            return
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.pose.position = msg.position
        if self.z_align:
            out.pose.position.z = msg.position.z + self._z0
        out.pose.orientation = _yaw_to_quat(float(msg.yaw))
        self.pub.publish(out)
        now = time.monotonic()
        if now - self._last_log > 2.0:
            self._last_log = now
            self.get_logger().info(
                f'EGO pos_cmd → offboard '
                f'({msg.position.x:.2f},{msg.position.y:.2f},{msg.position.z:.2f})')

    def _on_pose(self, msg: PoseStamped):
        if self.get_parameter('wait_airborne').value and not self.airborne:
            return
        if self.z_align and self._z0 is None:
            return
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'map'
        out.pose = msg.pose
        if self.z_align:
            out.pose.position.z = msg.pose.position.z + self._z0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PoscmdToOffboard()
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
