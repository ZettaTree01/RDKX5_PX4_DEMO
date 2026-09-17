#!/usr/bin/env python3
"""GS130W MIPI 左目 → `/camera/image_raw`（bgr8）。

优先订 `/image_left_raw`；没有则拆 `/image_combine_raw` 上半幅（dual_combine=2）。
视觉例程 03/04/05/06/07 默认走这条链路，YOLO 才能吃到 ISP 出的 NV12 再上 BPU。
USB 摄像头仍用 camera_node.py。
"""
from __future__ import annotations

import argparse
import os
import sys

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from depth_rgbd import image_msg_to_bgr, split_stereo_combine
from frame_output import FrameOutput


class MipiCameraBridge(Node):
    def __init__(self, show=False, snapshot=None, snapshot_period=5.0):
        super().__init__('mipi_camera_bridge')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Image, '/camera/image_raw', qos_profile_sensor_data)
        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period, title='camera (q/Esc 退出)',
            fallback_path='/tmp/camera_snapshot.jpg')
        self._got_left = False
        self.create_subscription(
            Image, '/image_left_raw', self._on_left, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/image_combine_raw', self._on_combine, qos_profile_sensor_data)
        self.get_logger().info(
            'MIPI→/camera/image_raw：优先 /image_left_raw，否则拆 combine 上半幅')

    def _publish_bgr(self, bgr, stamp, frame_id='camera'):
        if bgr is None or bgr.size == 0:
            return
        msg = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id or 'camera'
        try:
            self.pub.publish(msg)
        except Exception:
            return
        self.out.output(bgr)

    def _on_left(self, msg: Image):
        self._got_left = True
        try:
            bgr = image_msg_to_bgr(msg, self.bridge)
        except Exception as exc:
            self.get_logger().warn(f'left 解码失败: {exc}', throttle_duration_sec=2.0)
            return
        self._publish_bgr(bgr, msg.header.stamp, msg.header.frame_id)

    def _on_combine(self, msg: Image):
        if self._got_left:
            return
        try:
            bgr = image_msg_to_bgr(msg, self.bridge)
            left, _right = split_stereo_combine(bgr, 'tb')
        except Exception as exc:
            self.get_logger().warn(
                f'combine 解码失败: {exc}', throttle_duration_sec=2.0)
            return
        self._publish_bgr(left, msg.header.stamp, msg.header.frame_id)


def main(args=None):
    parser = argparse.ArgumentParser(description='MIPI 左目转发 /camera/image_raw')
    parser.add_argument('--show', dest='show', action='store_true', default=False)
    parser.add_argument('--no-show', dest='show', action='store_false')
    parser.add_argument('--snapshot', default=None)
    parser.add_argument('--snapshot-period', type=float, default=5.0)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = MipiCameraBridge(
        parsed.show, parsed.snapshot, parsed.snapshot_period)
    try:
        while rclpy.ok() and not node.out.quit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    try:
        node.out.close()
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
