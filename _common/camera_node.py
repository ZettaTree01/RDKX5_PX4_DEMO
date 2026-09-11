#!/usr/bin/env python3
"""USB 摄像头发布 /camera/image_raw（bgr8）。文档 3.1。

共享组件，放在 _common/，由需要图像的各例程 launch 直接拉起；
03_camera_node 是它的示例用法。

OpenCV VideoCapture(0) 对应 /dev/video0，适合 USB 摄像头。
MIPI CSI（接口 5）不要用本节点硬开，请用 TROS mipi_cam。

约 15 fps，降低 USB 与 ROS 未压缩图像带宽，减轻画面卡顿。
Ctrl+C 时释放 VideoCapture，避免占用 /dev/video0。

画面输出（默认关闭；03_camera_node 的 run.sh 默认打开 --show）：
  --show     弹窗显示实时画面。需要显示环境：板端接 HDMI 显示器后
             在桌面终端运行，或从开发机 `ssh -X` 登录。
             窗口内按 q 或 Esc 退出。
  --snapshot 定期把最新一帧写成 JPEG（--snapshot-period 控制间隔）。
             --show 打开但当前环境没有显示时，会自动回退到快照输出
             （默认 /tmp/camera_snapshot.jpg），日志会打印快照路径。
弹窗 / 快照逻辑在共享组件 _common/frame_output.py。
"""
import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_output import FrameOutput

WINDOW_NAME = 'camera (q/Esc 退出)'


class CameraNode(Node):
    def __init__(self, device='/dev/video0', show=False,
                 snapshot=None, snapshot_period=5.0):
        super().__init__('camera_node')

        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', qos_profile_sensor_data)
        self.bridge = CvBridge()

        source = int(device) if str(device).isdigit() else device
        self.cap = cv2.VideoCapture(source)
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:
            pass
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if not self.cap.isOpened():
            self.get_logger().error(f'打不开摄像头 {device}')

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period, title=WINDOW_NAME,
            fallback_path='/tmp/camera_snapshot.jpg')

        self.timer = self.create_timer(1 / 15, self.publish_image)
        self.get_logger().info('摄像头节点已启动（15 fps，降低 USB/ROS 带宽）')

    def publish_image(self):
        if not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.image_pub.publish(msg)
        except Exception:
            # 进程被 timeout/Ctrl+C 打断时 publisher context 可能已失效
            return
        self.out.output(frame)

    def destroy_node(self):
        self.out.close()
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='USB 摄像头 ROS2 发布节点')
    parser.add_argument('--device', default='/dev/video0')
    # 被 04/05/07 launch 拉起时只发话题；03 的 run.sh 显式传 --show
    parser.add_argument('--show', dest='show', action='store_true',
                        default=False,
                        help='弹窗显示实时画面（需 HDMI 显示器或 X11 转发）')
    parser.add_argument('--no-show', dest='show', action='store_false',
                        help='不弹窗只发话题（被 04/05/07 launch 拉起时默认不弹窗）')
    parser.add_argument('--snapshot', default=None,
                        help='定期把最新一帧写到该 JPEG 路径')
    parser.add_argument('--snapshot-period', type=float, default=5.0,
                        help='快照间隔秒数，默认 5')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = CameraNode(parsed.device, parsed.show, parsed.snapshot,
                      parsed.snapshot_period)
    try:
        while rclpy.ok() and not node.out.quit:
            rclpy.spin_once(node, timeout_sec=0.1)
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
