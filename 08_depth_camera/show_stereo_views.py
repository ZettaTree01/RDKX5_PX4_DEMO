#!/usr/bin/env python3
"""GS130W 左右目分窗显示（可选 Stereonet 深彩 / 深度）。

用法（板端桌面终端）：
  bash /app/zettatree_demo/08_depth_camera/run.sh views
  bash .../run.sh views --no-depth --no-visual   # 仅左右目
  bash .../run.sh views start_stereo:=0 --no-depth --no-visual

按 q / Esc 退出。无 DISPLAY 时写 /tmp/stereo_views_*.jpg。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from depth_rgbd import (  # noqa: E402
    colorize_depth, depth_msg_to_meters, image_msg_to_bgr, split_stereo_combine)


def _label(img, text):
    """在画面左上角叠加英文标签。"""
    out = img.copy()
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 255), 2, cv2.LINE_AA)
    return out


class StereoViewsNode(Node):
    """订阅拼接图/官方左右目与 Stereonet 深彩/深度，分窗显示。"""

    def __init__(self, layout='tb', show_depth=True, show_visual=True):
        """layout=tb 表示上下拼接（左目在上）；可选关闭深度/深彩窗。"""
        super().__init__('stereo_views')
        self.bridge = CvBridge()
        self.layout = layout
        self.show_depth = show_depth
        self.show_visual = show_visual
        self.left = None
        self.right = None
        self.visual = None
        self.depth_viz = None
        self._last_t = 0.0
        self._has_display = bool(
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        self._snap_t = 0.0

        self.create_subscription(
            Image, '/image_combine_raw', self._on_combine, qos_profile_sensor_data)
        if show_visual:
            self.create_subscription(
                Image, '/StereoNetNode/stereonet_visual',
                self._on_visual, qos_profile_sensor_data)
        if show_depth:
            self.create_subscription(
                Image, '/StereoNetNode/stereonet_depth',
                self._on_depth, qos_profile_sensor_data)

        self.create_subscription(
            Image, '/StereoNetNode/origin_left_image',
            lambda m: self._on_side(m, 'left'), qos_profile_sensor_data)
        self.create_subscription(
            Image, '/StereoNetNode/origin_right_image',
            lambda m: self._on_side(m, 'right'), qos_profile_sensor_data)

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            '分窗：Left / Right'
            + (' / Visual' if show_visual else '')
            + (' / Depth' if show_depth else '')
            + '  （q/Esc 退出）')

    def _on_combine(self, msg: Image):
        """MIPI 拼接图回调：拆成左右目。"""
        try:
            bgr = image_msg_to_bgr(msg, self.bridge)
            left, right = split_stereo_combine(bgr, self.layout)
            self.left = left
            self.right = right
        except Exception as exc:
            self.get_logger().warn(f'combine 解码失败: {exc}',
                                   throttle_duration_sec=2.0)

    def _on_side(self, msg: Image, which: str):
        """官方 origin_left/right 单目回调（优先于拼接拆分）。"""
        try:
            bgr = image_msg_to_bgr(msg, self.bridge)
            if which == 'left':
                self.left = bgr
            else:
                self.right = bgr
        except Exception:
            pass

    def _on_visual(self, msg: Image):
        """Stereonet 深彩可视化回调。"""
        try:
            self.visual = image_msg_to_bgr(msg, self.bridge)
        except Exception as exc:
            self.get_logger().warn(f'visual 失败: {exc}',
                                   throttle_duration_sec=2.0)

    def _on_depth(self, msg: Image):
        """Stereonet 深度图回调：转米制并伪彩。"""
        try:
            arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth = depth_msg_to_meters(np.asarray(arr), msg.encoding)
            self.depth_viz = colorize_depth(depth)
        except Exception as exc:
            self.get_logger().warn(f'depth 失败: {exc}',
                                   throttle_duration_sec=2.0)

    def _tick(self):
        """定时刷新分窗；有 DISPLAY 用 imshow，否则写 /tmp 快照。"""
        now = time.monotonic()
        if now - self._last_t < 0.03:
            return
        self._last_t = now

        frames = []
        if self.left is not None:
            frames.append(('LEFT', _label(self.left, 'LEFT')))
        if self.right is not None:
            frames.append(('RIGHT', _label(self.right, 'RIGHT')))
        if self.show_visual and self.visual is not None:
            frames.append(('VISUAL', _label(self.visual, 'STEREO VISUAL')))
        if self.show_depth and self.depth_viz is not None:
            frames.append(('DEPTH', _label(self.depth_viz, 'DEPTH')))

        if not frames:
            return

        if self._has_display:
            for name, img in frames:
                cv2.imshow(f'stereo_{name}', img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                self.get_logger().info('退出')
                raise SystemExit(0)
        else:
            if now - self._snap_t < 2.0:
                return
            self._snap_t = now
            for name, img in frames:
                path = f'/tmp/stereo_views_{name.lower()}.jpg'
                cv2.imwrite(path, img)
            self.get_logger().info(
                '无 DISPLAY，已写 ' + ', '.join(
                    f'/tmp/stereo_views_{n.lower()}.jpg' for n, _ in frames),
                throttle_duration_sec=5.0)


def main():
    """解析参数并 spin 分窗显示节点。"""
    parser = argparse.ArgumentParser(description='GS130W 左右目分窗显示')
    parser.add_argument('--layout', default='tb', choices=['tb', 'lr'],
                        help='combine 拼接：tb=上下(左上) lr=左右(左左)')
    parser.add_argument('--no-depth', action='store_true')
    parser.add_argument('--no-visual', action='store_true')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = StereoViewsNode(
        layout=args.layout,
        show_depth=not args.no_depth,
        show_visual=not args.no_visual,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
