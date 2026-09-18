#!/usr/bin/env python3
"""目标检测 ROS 节点。文档 3.1。

订阅 ``/camera/image_raw``，经 ``_common/yolo_detector.py`` 完成端到端推理
（NV12 → BPU → DFL → NMS，与官方
``/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8`` 对齐），
将结果发布至 ``/drone/detection_position``，并输出推理画面。

``--show`` / launch 参数 ``show:=`` 默认开启：有显示环境时弹窗（框 + 类别/置信度），
无显示环境时周期性写入快照（默认 ``/tmp/detection_snapshot.jpg``）。
模型未加载或推理失败时仍输出原图并叠加状态文字。窗口内按 q / Esc
仅关闭画面输出，节点继续运行。

像素框中心不等于 map 系三维坐标，不得写死 ``(0,0,2)`` 作为目标位姿。
类别为 COCO 80 类；更换模型时须同步调整解码假设。
"""
import argparse
import os
import sys
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from frame_output import FrameOutput
from yolo_detector import YoloDetector
from depth_rgbd import image_msg_to_bgr

FONT = cv2.FONT_HERSHEY_SIMPLEX


class DroneDetectionNode(Node):
    """订阅相机图 → BPU YOLO 推理 → 发布检测事件并可选可视化。"""

    def __init__(self, show=True, snapshot=None, snapshot_period=5.0,
                 score_thres=0.25, nms_thres=0.45):
        super().__init__('drone_detection')

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback,
            qos_profile_sensor_data)

        self.detection_pub = self.create_publisher(
            PoseStamped, '/drone/detection_position', 10)

        self.bridge = CvBridge()
        self.detector = YoloDetector(
            score_thres=score_thres, nms_thres=nms_thres,
            log=self.get_logger())
        self.detector.start_async(period=0.1)

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='detection (q/Esc 退出)',
            fallback_path='/tmp/detection_snapshot.jpg')

        self.get_logger().info('无人机检测节点已启动')

    def image_callback(self, msg):
        """图像回调：解码 → 推理 → 画框输出；有目标则发布检测 PoseStamped。"""
        try:
            frame = image_msg_to_bgr(msg, self.bridge)
        except Exception as exc:
            self.get_logger().warn(f'图像解码失败: {exc}', throttle_duration_sec=2.0)
            return
        if not self.detector.loaded:
            self._output(frame, [], 'model not loaded (raw frame)')
            return

        try:
            self.detector.submit_frame(frame)
            detections = self.detector.latest_detections()
            if detections is None:
                self._output(frame, [])
                return
        except Exception:
            self.get_logger().error(
                '推理失败:\n' + traceback.format_exc(),
                throttle_duration_sec=5.0)
            self._output(frame, [], 'inference failed (see log)')
            return

        note = None if detections else 'no detections'
        self._output(frame, detections, note)
        if detections:
            self.publish_detection(detections[0])

    def _output(self, frame, detections, note=None):
        """在副本上画检测框与状态文字，经 FrameOutput 弹窗/快照。"""
        if not self.out.enabled():
            return
        vis = frame.copy()
        for (x1, y1, x2, y2, score, cls_id) in detections:
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 0), 2)
            cv2.putText(vis, f'{self.detector.label(cls_id)} {score:.2f}',
                        (int(x1), max(12, int(y1) - 6)),
                        FONT, 0.55, (0, 255, 0), 2)
        if note:
            cv2.putText(vis, note, (8, 24), FONT, 0.55, (0, 0, 255), 2)
        self.out.output(vis)

    def publish_detection(self, detection):
        """发布「检测到目标」事件（不含写死三维坐标）。"""
        # 只发布"检测到目标"这一事件；像素框中心 ≠ map 系三维坐标，
        # 真实三维位置需配合深度/位姿估计，此处不写死坐标。
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.detection_pub.publish(msg)

    def destroy_node(self):
        """关闭画面输出后再销毁节点。"""
        self.out.close()
        super().destroy_node()


def main(args=None):
    """解析显示/阈值参数并 spin 检测节点。"""
    parser = argparse.ArgumentParser(description='目标检测 ROS 节点')
    # 必须显式 default=True：--show/--no-show 共用 dest，argparse 取
    # 第一个 action 的默认值，store_true 的默认值是 False
    parser.add_argument('--show', dest='show', action='store_true',
                        default=True,
                        help='输出推理画面（默认输出）')
    parser.add_argument('--no-show', dest='show', action='store_false',
                        help='关闭推理画面输出')
    parser.add_argument('--snapshot', default=None,
                        help='定期把推理画面写到该 JPEG 路径')
    parser.add_argument('--snapshot-period', type=float, default=5.0,
                        help='快照间隔秒数，默认 5')
    parser.add_argument('--score-thres', type=float, default=0.25,
                        help='置信度阈值（概率域），默认 0.25')
    parser.add_argument('--nms-thres', type=float, default=0.45,
                        help='NMS IoU 阈值，默认 0.45')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = DroneDetectionNode(parsed.show, parsed.snapshot,
                              parsed.snapshot_period, parsed.score_thres,
                              parsed.nms_thres)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 退出清理：launch 关停时会补发 SIGINT；rclpy 信号处理器可能已关闭 context
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
