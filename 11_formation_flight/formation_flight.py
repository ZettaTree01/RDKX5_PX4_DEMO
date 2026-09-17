#!/usr/bin/env python3
"""
编队飞行。文档 4.6 / 例程 11。

每架飞机一个进程、一套 MAVROS 和 OFFBOARD 管理器。
各机发布相对启动位置的位移，跟随机把领队位移叠加到自己的本地原点，
避免直接混用不同 PX4 的 local ENU 原点。所有飞机须使用一致的 ENU 朝向。

队形：3 机三角形，4 机菱形；偏移单位米，ENU。
室内台架默认把队形间距压到实飞值的 1/20（见 _common/indoor.py）。
室内无 GPS 时 launch 默认台架位姿模拟；arm:=true 后强制解锁，
先爬升拉转速再悬停，有队形位置指令时加速，最高 300 r/min。
"""
import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from indoor import FORMATION_SPAN_M


class FormationFlightNode(Node):
    """单机编队节点：广播本机相对起飞点位移，跟随领队叠加队形偏移。"""

    def __init__(self, drone_id=0, num_drones=3):
        super().__init__(f'formation_flight_{drone_id}')

        self.drone_id = drone_id
        self.num_drones = num_drones
        if not 0 <= drone_id < num_drones:
            raise ValueError(
                f'drone-id {drone_id} 越界：编队共 {num_drones} 架，'
                f'取值 0..{num_drones - 1}')

        self.position_pub = self.create_publisher(
            PoseStamped, '/drone/setpoint_position/local', 10)
        self.displacement_pub = self.create_publisher(
            PoseStamped, f'/drone_{drone_id}/displacement', 10)

        self.other_positions = {}
        for i in range(num_drones):
            if i != drone_id:
                self.create_subscription(
                    PoseStamped,
                    f'/drone_{i}/displacement',
                    lambda msg, idx=i: self.other_position_callback(idx, msg),
                    10)

        self.position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.position_callback, qos_profile_sensor_data)

        self.current_position = None
        self.local_origin = None
        self.airborne = False
        self.armed = False
        self.create_subscription(
            Bool, '/drone/status/airborne',
            lambda msg: setattr(self, 'airborne', msg.data), 10)
        self.create_subscription(
            State, '/mavros/state',
            lambda msg: setattr(self, 'armed', bool(msg.armed)), 10)
        self.create_subscription(
            State, '/mavros/state',
            lambda msg: setattr(self, 'armed', bool(msg.armed)),
            qos_profile_sensor_data)

        self.formation_offsets = self.calculate_formation_offsets()

        self.get_logger().info(f'编队飞行节点 {drone_id} 已启动')
        self.create_timer(0.05, self.control_loop)

    def calculate_formation_offsets(self):
        """相对领队的 ENU 偏移 (x, y, z)，单位米。"""
        offsets = []

        span = FORMATION_SPAN_M
        half = span * 0.5
        if self.num_drones == 3:
            offsets = [
                (0.0, 0.0, 0.0),
                (-span, -span, 0.0),
                (-span, span, 0.0),
            ]
        elif self.num_drones == 4:
            offsets = [
                (0.0, 0.0, 0.0),
                (-span, 0.0, 0.0),
                (-half, -span, 0.0),
                (-half, span, 0.0),
            ]
        else:
            offsets = [(0.0, float(i) * half, 0.0)
                       for i in range(self.num_drones)]

        return offsets

    def position_callback(self, msg):
        self.current_position = msg.pose.position
        self._maybe_origin()

    def _maybe_origin(self):
        if self.local_origin is not None or not self.airborne:
            return
        if self.current_position is None:
            return
        self.local_origin = (
            self.current_position.x,
            self.current_position.y,
            self.current_position.z)
        self.get_logger().info('已记录编队本地原点')

    def other_position_callback(self, drone_id, msg):
        self.other_positions[drone_id] = msg.pose.position

    def get_leader_position(self):
        if self.drone_id == 0:
            return (0.0, 0.0, 0.0)
        return self.other_positions.get(0)

    def calculate_formation_position(self):
        leader_pos = self.get_leader_position()
        if leader_pos is None:
            return None

        offset = self.formation_offsets[self.drone_id]

        if self.drone_id == 0:
            leader_dx, leader_dy, leader_dz = leader_pos
        else:
            leader_dx, leader_dy, leader_dz = (
                leader_pos.x, leader_pos.y, leader_pos.z)

        target_x = self.local_origin[0] + leader_dx + offset[0]
        target_y = self.local_origin[1] + leader_dy + offset[1]
        target_z = self.local_origin[2] + leader_dz + offset[2]

        return (target_x, target_y, target_z)

    def control_loop(self):
        self._maybe_origin()
        if self.current_position is None or self.local_origin is None:
            if not self.airborne:
                wait = '起飞中，随后进入队形' if self.armed else '等待解锁'
                self.get_logger().info(wait, throttle_duration_sec=5.0)
            return

        displacement = PoseStamped()
        displacement.header.stamp = self.get_clock().now().to_msg()
        displacement.header.frame_id = 'map'
        displacement.pose.position.x = (
            self.current_position.x - self.local_origin[0])
        displacement.pose.position.y = (
            self.current_position.y - self.local_origin[1])
        displacement.pose.position.z = (
            self.current_position.z - self.local_origin[2])
        displacement.pose.orientation.w = 1.0
        self.displacement_pub.publish(displacement)

        target = self.calculate_formation_position()
        if target is None:
            return

        target_x, target_y, target_z = target

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z
        msg.pose.orientation.w = 1.0

        self.position_pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='编队节点：一机一进程')
    parser.add_argument('--drone-id', type=int, default=0, help='本机编号，0 为领队')
    parser.add_argument('--num-drones', type=int, default=3, help='编队飞机数')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = FormationFlightNode(drone_id=parsed.drone_id, num_drones=parsed.num_drones)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
