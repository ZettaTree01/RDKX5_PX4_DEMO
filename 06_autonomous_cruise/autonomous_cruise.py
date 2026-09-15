#!/usr/bin/env python3
"""自主巡航拍摄。文档 4.1。

起飞后以当前位置为原点飞行方形航点，到点拍照，完成后请求降落。
位置目标仅发给 OFFBOARD 管理器；禁止在回调外使用 while + sleep，以免阻塞 spin。

室内 launch 默认启用台架；须指定 ``arm:=true`` 才会强制解锁，航点边长为实飞的 1/20。
``--show`` 默认开启；无显示器时回退为周期性快照。
"""
import argparse
import glob
import os
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool
import cv2
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from frame_output import FrameOutput
from indoor import CRUISE_SIDE_M, RelAlt
from cn_hud import put_cn_lines


def default_camera_device():
    """按序号依次试探，返回第一个能 read 到帧的 /dev/video*。

    全部试探失败时退化为 /dev/video0。
    """
    devs = sorted(glob.glob('/dev/video*'),
                  key=lambda p: int(re.sub(r'\D', '', p) or 0))
    if not devs:
        return '/dev/video0'
    for d in devs:
        cap = cv2.VideoCapture(d)
        try:
            if cap.isOpened():
                ok, _frame = cap.read()
                if ok:
                    return d
        finally:
            cap.release()
    return devs[0]


class AutonomousCruiseNode(Node):
    def __init__(self, device=None, show=True, snapshot=None,
                 snapshot_period=5.0):
        super().__init__('autonomous_cruise')

        self.position_pub = self.create_publisher(
            PoseStamped, '/drone/setpoint_position/local', 10)
        self.land_pub = self.create_publisher(
            Bool, '/drone/control/land', 10)
        self.airborne = False
        self.armed = False
        self.alt_z = None
        self._rel_alt = RelAlt()
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

        self.position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.position_callback, qos_profile_sensor_data)

        self.device = device or default_camera_device()
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            self.get_logger().warn(
                f'打不开摄像头 {self.device}，到达航点时跳过拍照')

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='cruise (q/Esc 退出)',
            fallback_path='/tmp/cruise_snapshot.jpg')

        self.current_position = None
        self.waypoints = None

        self.get_logger().info('自主巡航节点已启动')

        self.wp_idx = 0
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._preview)

    def position_callback(self, msg):
        self.current_position = msg.pose.position
        self.alt_z = self._rel_alt.update(msg.pose.position.z)
        self._maybe_plan()

    def _maybe_plan(self):
        if self.waypoints is not None or not self.airborne:
            return
        if self.current_position is None:
            return
        x, y, z = (
            self.current_position.x,
            self.current_position.y,
            self.current_position.z)
        self.waypoints = [
            (x, y, z), (x + CRUISE_SIDE_M, y, z),
            (x + CRUISE_SIDE_M, y + CRUISE_SIDE_M, z),
            (x, y + CRUISE_SIDE_M, z), (x, y, z)]
        self.get_logger().info(
            f'已规划巡航方形边长 {CRUISE_SIDE_M:.3f} m')

    def _publish_sp(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.position_pub.publish(msg)

    def capture_image(self):
        if not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        timestamp = int(time.time())
        path = f'/tmp/capture_{timestamp}.jpg'
        cv2.imwrite(path, frame)
        self.get_logger().info(f'图片已保存: {path}')

    def _preview(self):
        """周期输出巡航画面（与 _tick 同在单线程 executor，串行取帧安全）。"""
        if not self.cap.isOpened() or not self.out.enabled():
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        if self.waypoints is None:
            if self.airborne:
                state = '悬停，规划航点'
            elif self.armed:
                state = '起飞中'
            else:
                state = '等待解锁'
        elif self.wp_idx < len(self.waypoints):
            state = f'航点 {self.wp_idx + 1}/{len(self.waypoints)}'
        else:
            state = '已完成，降落'
        vis = frame.copy()
        alt = '高度 --' if self.alt_z is None else f'高度 {self.alt_z:.2f} m'
        put_cn_lines(vis, [(alt, (0, 255, 255)), (state, (0, 255, 255))],
                     origin=(8, 6), size=20)
        self.out.output(vis)

    def _tick(self):
        self._maybe_plan()
        if self.waypoints is None:
            return
        if self.wp_idx >= len(self.waypoints):
            self._publish_sp(*self.waypoints[-1])
            msg = Bool()
            msg.data = True
            self.land_pub.publish(msg)
            return
        x, y, z = self.waypoints[self.wp_idx]
        self._publish_sp(x, y, z)
        if self.current_position is None:
            return
        d2 = ((self.current_position.x - x) ** 2
              + (self.current_position.y - y) ** 2
              + (self.current_position.z - z) ** 2)
        arrive = max(0.02, CRUISE_SIDE_M * 0.3)
        if d2 < arrive * arrive:
            self.capture_image()
            self.wp_idx += 1

    def destroy_node(self):
        self.out.close()
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='自主巡航拍照任务')
    parser.add_argument('--device', default='auto',
                        help='摄像头设备；auto 表示自动探测能出图的 /dev/video*')
    # 必须显式 default=True：--show/--no-show 共用 dest，argparse 取
    # 第一个 action 的默认值，store_true 的默认值是 False
    parser.add_argument('--show', dest='show', action='store_true',
                        default=True,
                        help='输出巡航画面（默认输出）')
    parser.add_argument('--no-show', dest='show', action='store_false',
                        help='关闭巡航画面输出')
    parser.add_argument('--snapshot', default=None,
                        help='定期把巡航画面写到该 JPEG 路径')
    parser.add_argument('--snapshot-period', type=float, default=5.0,
                        help='快照间隔秒数，默认 5')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = AutonomousCruiseNode(
        None if parsed.device == 'auto' else parsed.device,
        parsed.show, parsed.snapshot, parsed.snapshot_period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 退出清理：launch 关停时会补发 SIGINT；rclpy 信号处理器可能已关闭 context
    try:
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
