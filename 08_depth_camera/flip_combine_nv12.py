#!/usr/bin/env python3
"""[DEBUG ONLY] Swap top/bottom halves of MIPI dual_combine NV12.

正式例程 08/09/10 不启动本节点。仅当确认 L/R 上下顺序反了、
需要临时对调半幅时手工运行。Stereonet 订阅为 RELIABLE，本节点
发布也必须 RELIABLE。
"""
from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def swap_tb_nv12(data: bytes, w: int, h: int) -> bytes:
    """h must be even; for 640x704 combine, each eye is h/2."""
    arr = np.frombuffer(data, dtype=np.uint8).copy()
    y_sz = w * h
    mid_y = (h // 2) * w
    y = arr[:y_sz]
    y2 = y.copy()
    y2[:mid_y] = y[mid_y:y_sz]
    y2[mid_y:y_sz] = y[:mid_y]
    arr[:y_sz] = y2
    uv = arr[y_sz:y_sz + w * h // 2]
    mid_uv = (h // 4) * w
    uv2 = uv.copy()
    uv2[:mid_uv] = uv[mid_uv:]
    uv2[mid_uv:] = uv[:mid_uv]
    arr[y_sz:y_sz + w * h // 2] = uv2
    return arr.tobytes()


class Flip(Node):
    def __init__(self):
        super().__init__('flip_combine_nv12')
        self.pub = self.create_publisher(Image, '/image_combine_flipped', _QOS)
        self.create_subscription(
            Image, '/image_combine_raw', self._on, qos_profile_sensor_data)
        self.get_logger().info(
            'flip TB NV12 /image_combine_raw → /image_combine_flipped (RELIABLE)')

    def _on(self, msg: Image):
        enc = (msg.encoding or '').lower()
        if enc != 'nv12':
            self.get_logger().warn(
                f'unexpected encoding {msg.encoding}', throttle_duration_sec=2.0)
            return
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = swap_tb_nv12(bytes(msg.data), msg.width, msg.height)
        self.pub.publish(out)


def main():
    rclpy.init()
    n = Flip()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
