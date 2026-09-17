#!/usr/bin/env python3
"""持续发布 MIPI 双目 CameraInfo（mipi_cam 常只建话题不发数据）。

Stereonet 订阅 camera_info 使用 RELIABLE；本节点按 RELIABLE 发布有效内参。
"""
from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo


def _make_info(width, height, fx, fy, cx, cy, frame_id, baseline=0.0, is_right=False):
    """构造左右目 CameraInfo；右目 P[0,3]=+fx*B 供 Stereonet 取基线。"""
    msg = CameraInfo()
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [float(fx), 0.0, float(cx),
             0.0, float(fy), float(cy),
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    # Stereonet 用 P[0,3]/fx 作为基线，右目取 +fx*B。
    tx = float(fx) * float(baseline) if is_right else 0.0
    msg.p = [float(fx), 0.0, float(cx), tx,
             0.0, float(fy), float(cy), 0.0,
             0.0, 0.0, 1.0, 0.0]
    msg.header.frame_id = frame_id
    return msg


class StereoCamInfoNode(Node):
    """按固定频率 RELIABLE 发布左右目 CameraInfo。"""

    def __init__(self, width, height, fx, fy, cx, cy, baseline,
                 left_topic, right_topic, rate_hz):
        """内参与基线默认对齐 GS130W + Stereonet 640x352 输入。"""
        super().__init__('stereo_caminfo_pub')
        # Stereonet 需要 RELIABLE
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)
        self.pub_l = self.create_publisher(CameraInfo, left_topic, qos)
        self.pub_r = self.create_publisher(CameraInfo, right_topic, qos)
        self._left = _make_info(
            width, height, fx, fy, cx, cy, 'camera_link', baseline, False)
        self._right = _make_info(
            width, height, fx, fy, cx, cy, 'camera_link_right', baseline, True)
        period = 1.0 / max(1.0, float(rate_hz))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'publish CameraInfo(RELIABLE) {width}x{height} '
            f'fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} '
            f'baseline={baseline:.3f}m left={left_topic} right={right_topic}')

    def _tick(self):
        """刷新时间戳并发布左右 CameraInfo。"""
        stamp = self.get_clock().now().to_msg()
        self._left.header.stamp = stamp
        self._right.header.stamp = stamp
        self.pub_l.publish(self._left)
        self.pub_r.publish(self._right)


def main():
    """解析内参参数并 spin CameraInfo 发布节点。"""
    parser = argparse.ArgumentParser()
    # 默认对齐 Stereonet 模型输入 640x352（避免二次缩放）
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=352)
    # 方像素：fx=fy（错误缩小 fy 会导致点云扇形失真）
    parser.add_argument('--fx', type=float, default=328.379)
    parser.add_argument('--fy', type=float, default=328.379)
    parser.add_argument('--cx', type=float, default=320.0)
    parser.add_argument('--cy', type=float, default=176.0)
    parser.add_argument('--baseline', type=float, default=0.07917)
    parser.add_argument('--left-topic',
                        default='/drone/stereo/left/camera_info')
    parser.add_argument('--right-topic',
                        default='/drone/stereo/right/camera_info')
    parser.add_argument('--rate', type=float, default=10.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = StereoCamInfoNode(
        args.width, args.height, args.fx, args.fy, args.cx, args.cy,
        args.baseline, args.left_topic, args.right_topic, args.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
