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
        """订阅设定点/本地点，定时发布 vision_pose。"""
        super().__init__('bench_pose_sim')
        self.max_speed = max_speed
        self.dt = 1.0 / rate
        self.pos = [0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, 0.0]
        # 固定航向：台架无磁，靠 EV yaw；勿跟飞控噪声姿态抖动
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.vel = None
        self.vel_time = None
        self._snapped_local = False
        # 气压/EKF 绝对高度常为几十~上千米；视觉统一归零到相对高度
        self._z0 = None
        # 回灌外部视觉位姿给飞控 EKF
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
            f'台架模拟器：立即回灌视觉（z 相对首帧归零，航向固定），偏差超过 '
            f'{SNAP_M:.1f} m 时对齐，限速 {max_speed} m/s')

    def _rel(self, x, y, z):
        """把飞控/设定点绝对坐标换成视觉相对坐标（z0 锁定前 z→0）。"""
        if self._z0 is None:
            return float(x), float(y), 0.0
        return float(x), float(y), float(z) - self._z0

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
        """首次收到飞控本地点：锁定 z0，XY 对齐，视觉 z 归零。"""
        p = msg.pose.position
        if self._z0 is None:
            self._z0 = float(p.z)
            self.get_logger().info(
                f'锁定视觉高度基准 z0={self._z0:.2f} m（之后视觉 z 为相对高度）')
        if self._snapped_local:
            return
        self._snap_to(self._rel(p.x, p.y, p.z), '收到飞控本地点')
        self._snapped_local = True

    def _on_setpoint(self, msg):
        """位置设定点更新：记录目标，偏差过大时瞬移对齐（姿态保持固定）。"""
        p = msg.pose.position
        self.target = list(self._rel(p.x, p.y, p.z))
        self._snap_to(self.target, '设定点偏离过远')

    def _on_velocity(self, msg):
        """速度设定点：记录线速度与时间戳，供积分分支使用。"""
        self.vel = (
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.vel_time = self.get_clock().now()

    def _velocity_fresh(self):
        """速度设定点是否在 200ms 内仍然有效。"""
        if self.vel is None or self.vel_time is None:
            return False
        return (self.get_clock().now() - self.vel_time).nanoseconds < 200_000_000

    def _tick(self):
        """发布 /mavros/vision_pose/pose，供 EKF 外部视觉融合。"""
        if self._velocity_fresh():
            # 有新鲜速度设定点：积分位移
            for i in range(3):
                self.pos[i] += self.vel[i] * self.dt
            self.target = list(self.pos)
        else:
            # 否则一阶跟随位置设定点，步长受 max_speed 限制
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
    """解析限速/频率参数并 spin 台架位姿模拟节点。"""
    parser = argparse.ArgumentParser(
        description='台架位姿模拟器（拆桨、室内无位置源）')
    parser.add_argument(
        '--max-speed', type=float, default=BENCH_FOLLOW_MPS,
        help=f'跟随设定点的限速（m/s），室内台架默认 {BENCH_FOLLOW_MPS}')
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
