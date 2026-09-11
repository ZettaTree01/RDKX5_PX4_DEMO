#!/usr/bin/env python3
"""台架位姿模拟器：让室内无 GPS 的飞控也能拿到位置估计。文档 2.8。

订阅 OFFBOARD 管理器发给 MAVROS 的位置或速度设定点，按限速一阶跟随 / 积分，
再把结果当作外部视觉位姿回灌给飞控（/mavros/vision_pose/pose）。

飞控因此“看到”自己到达了目标，油门维持正常量级。若改为喂静止位姿，
起飞指令会因高度永远上不去而把油门积分顶满，导致电机满速空转。

仅用于拆桨台架：模拟出的位置只存在于飞控的 EKF 里，飞机并没有动。
一启动就回灌视觉（默认原点 0），与飞控本地点或设定点偏差过大时直接对齐，
避免室内气压计在几十米时从 0 慢慢爬、高度一直涨、又解不了锁。
"""
import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from indoor import BENCH_FOLLOW_MPS

SNAP_M = 1.0  # 与本地点/设定点差这么远就瞬移对齐，不爬几十米


class BenchPoseSim(Node):
    """把 OFFBOARD 设定点积分成「假视觉位姿」回灌飞控 EKF。

    - 有新鲜速度设定点：积分 ``pos += vel * dt``
    - 否则：一阶跟随位置设定点，步长受 ``max_speed`` 限制
    - 与本地点/目标偏差 > SNAP_M：瞬移对齐（避免从 0 爬气压几十米）
    """

    def __init__(self, max_speed=BENCH_FOLLOW_MPS, rate=50.0):
        super().__init__('bench_pose_sim')
        self.max_speed = max_speed
        self.dt = 1.0 / rate
        self.pos = [0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, 0.0]
        self.orientation = None
        self.vel = None
        self.vel_time = None
        self._snapped_local = False
        self.pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10)
        self.create_subscription(
            PoseStamped, '/mavros/setpoint_position/local',
            self._on_setpoint, 10)
        self.create_subscription(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel',
            self._on_velocity, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_local, qos_profile_sensor_data)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'台架模拟器：立即回灌视觉，偏差超过 {SNAP_M:.1f} m 时对齐，'
            f'限速 {max_speed} m/s')

    def _snap_to(self, xyz, reason):
        """偏差过大则瞬移，避免 EKF 长时间积分爬升。"""
        dist = math.sqrt(sum((xyz[i] - self.pos[i]) ** 2 for i in range(3)))
        if dist < SNAP_M:
            return False
        self.pos = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        self.target = list(self.pos)
        self.get_logger().info(
            f'{reason}，视觉对齐到 ({self.pos[0]:.2f},{self.pos[1]:.2f},'
            f'{self.pos[2]:.2f}) 偏差 {dist:.1f} m')
        return True

    def _on_local(self, msg):
        """首次收到飞控本地点时对齐一次（室内气压常已是几十米）。"""
        if self._snapped_local:
            return
        p = msg.pose.position
        q = msg.pose.orientation
        self.orientation = (q.x, q.y, q.z, q.w)
        self._snap_to((p.x, p.y, p.z), '收到飞控本地点')
        self._snapped_local = True

    def _on_setpoint(self, msg):
        p = msg.pose.position
        self.target = [p.x, p.y, p.z]
        q = msg.pose.orientation
        self.orientation = (q.x, q.y, q.z, q.w)
        self._snap_to(self.target, '设定点偏离过远')

    def _on_velocity(self, msg):
        self.vel = (
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.vel_time = self.get_clock().now()

    def _velocity_fresh(self):
        if self.vel is None or self.vel_time is None:
            return False
        return (self.get_clock().now() - self.vel_time).nanoseconds < 200_000_000

    def _tick(self):
        """发布 /mavros/vision_pose/pose，供 EKF 外部视觉融合。"""
        if self._velocity_fresh():
            for i in range(3):
                self.pos[i] += self.vel[i] * self.dt
            self.target = list(self.pos)
        else:
            step = self.max_speed * self.dt
            for i in range(3):
                delta = self.target[i] - self.pos[i]
                self.pos[i] += max(-step, min(step, delta))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        (msg.pose.position.x, msg.pose.position.y,
         msg.pose.position.z) = self.pos
        if self.orientation is not None:
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = self.orientation
        else:
            msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='台架位姿模拟器（拆桨、室内无位置源）')
    parser.add_argument(
        '--max-speed', type=float, default=BENCH_FOLLOW_MPS,
        help=f'跟随设定点的限速（m/s），室内调试默认 {BENCH_FOLLOW_MPS}')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='位姿发布频率，Hz（UART 57600 不宜超过 20）')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = BenchPoseSim(parsed.max_speed, parsed.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
