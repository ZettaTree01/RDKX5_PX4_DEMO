#!/usr/bin/env python3
"""自主巡航拍摄。文档 4.1。

起飞后以当前位置为原点飞行方形航点，到点拍照，完成后请求降落。
位置目标仅发给 OFFBOARD 管理器；禁止在回调外使用 while + sleep，以免阻塞 spin。

室内 launch 默认启用台架；须指定 ``arm:=true`` 才会强制解锁，航点边长为实飞的 1/20。
``--show`` 默认开启；无显示器时回退为周期性快照。
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from frame_output import FrameOutput
from indoor import CRUISE_SIDE_M, RelAlt
from cn_hud import put_cn_lines
from depth_rgbd import image_msg_to_bgr
from yolo_detector import YoloDetector


class AutonomousCruiseNode(Node):
    """自主巡航节点：起飞后规划方形航点，到点拍照，完成后请求降落。"""

    def __init__(self, show=True, snapshot=None, snapshot_period=5.0,
                 infer_hz=5.0):
        """初始化发布/订阅、YOLO 检测与预览输出。

        Args:
            show: 是否输出巡航画面。
            snapshot: 定期快照 JPEG 路径；None 表示不强制写盘。
            snapshot_period: 快照间隔（秒）。
            infer_hz: BPU YOLO 推理频率上限。
        """
        super().__init__('autonomous_cruise')

        # 仅发给 OFFBOARD 管理器，不直接写 MAVROS 设定点
        self.position_pub = self.create_publisher(
            PoseStamped, '/drone/setpoint_position/local', 10)
        self.land_pub = self.create_publisher(
            Bool, '/drone/control/land', 10)
        self.airborne = False
        self.armed = False
        self.alt_z = None
        self._rel_alt = RelAlt()  # 相对开机高度，避免气压计绝对高度干扰 HUD
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

        self._bridge = None
        try:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        except Exception:
            pass
        self.frame = None
        self.dets = []
        self._last_infer = 0.0
        self._infer_dt = 1.0 / max(1.0, float(infer_hz))
        self.detector = YoloDetector(log=self.get_logger())
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='cruise (q/Esc 退出)',
            fallback_path='/tmp/cruise_snapshot.jpg')

        self.current_position = None
        self.waypoints = None  # 起飞后以当前位置为原点规划方形

        self.get_logger().info(
            f'自主巡航已启动（相机=/camera/image_raw，'
            f'BPU YOLO={"OK" if self.detector.loaded else "未加载"}）')

        self.wp_idx = 0
        self.create_timer(0.05, self._tick)      # 20 Hz 航点跟径
        self.create_timer(0.1, self._preview)    # 10 Hz 预览

    def position_callback(self, msg):
        """缓存局部位姿，并尝试在首次空中时规划航点。"""
        self.current_position = msg.pose.position
        self.alt_z = self._rel_alt.update(msg.pose.position.z)
        self._maybe_plan()

    def _maybe_plan(self):
        """空中且尚未规划时，以当前位置为起点生成闭合方形航点。"""
        if self.waypoints is not None or not self.airborne:
            return
        if self.current_position is None:
            return
        x, y, z = (
            self.current_position.x,
            self.current_position.y,
            self.current_position.z)
        # 方形四角 + 回到起点；边长取自室内缩放后的 CRUISE_SIDE_M
        self.waypoints = [
            (x, y, z), (x + CRUISE_SIDE_M, y, z),
            (x + CRUISE_SIDE_M, y + CRUISE_SIDE_M, z),
            (x, y + CRUISE_SIDE_M, z), (x, y, z)]
        self.get_logger().info(
            f'已规划巡航方形边长 {CRUISE_SIDE_M:.3f} m')

    def _publish_sp(self, x, y, z):
        """向 OFFBOARD 管理器发布局部位姿设定点。"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.position_pub.publish(msg)

    def _on_image(self, msg):
        """缓存最新相机帧；按 infer_hz 限频做 YOLO 检测。"""
        try:
            self.frame = image_msg_to_bgr(msg, self._bridge)
        except Exception as exc:
            self.get_logger().warn(f'相机解码失败: {exc}', throttle_duration_sec=2.0)
            return
        now = time.monotonic()
        if (self.detector.loaded and self.frame is not None
                and now - self._last_infer >= self._infer_dt):
            self._last_infer = now
            try:
                self.dets = self.detector.detect(self.frame)
            except Exception as exc:
                self.get_logger().warn(
                    f'BPU YOLO 失败: {exc}', throttle_duration_sec=5.0)

    def capture_image(self):
        """到达航点时把当前帧（含检测框）保存到 /tmp。"""
        frame = self.frame
        if frame is None:
            return
        timestamp = int(time.time())
        path = f'/tmp/capture_{timestamp}.jpg'
        vis = frame.copy()
        for x1, y1, x2, y2, score, cls_id in self.dets:
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)),
                          (80, 220, 80), 2)
        cv2.imwrite(path, vis)
        self.get_logger().info(f'图片已保存: {path}')

    def _preview(self):
        """周期输出巡航画面（与 _tick 同在单线程 executor）。"""
        if self.frame is None or not self.out.enabled():
            return
        vis = self.frame.copy()
        for x1, y1, x2, y2, score, cls_id in self.dets:
            p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(vis, p1, p2, (80, 220, 80), 2)
            label = self.detector.label(int(cls_id))
            cv2.putText(vis, f'{label} {score:.2f}', (p1[0], max(16, p1[1] - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 220, 80), 1,
                        cv2.LINE_AA)
        # 顶部中文状态：高度 / 航点阶段 / YOLO 是否可用
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
        alt = '高度 --' if self.alt_z is None else f'高度 {self.alt_z:.2f} m'
        bpu = 'BPU YOLO' if self.detector.loaded else 'YOLO 未加载'
        put_cn_lines(vis, [(alt, (0, 255, 255)), (state, (0, 255, 255)),
                           (bpu, (80, 220, 80))],
                     origin=(8, 6), size=20)
        self.out.output(vis)

    def _tick(self):
        """航点跟径：发布当前目标；到点则拍照并切下一航点；全部完成后请求降落。"""
        self._maybe_plan()
        if self.waypoints is None:
            return
        if self.wp_idx >= len(self.waypoints):
            # 保持最后一个航点设定点，同时请求管理器降落上锁
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
        # 到达半径随边长缩放，最小 2 cm（台架边长很短时仍可到点）
        arrive = max(0.02, CRUISE_SIDE_M * 0.3)
        if d2 < arrive * arrive:
            self.capture_image()
            self.wp_idx += 1

    def destroy_node(self):
        """关闭画面输出后销毁节点。"""
        self.out.close()
        super().destroy_node()


def main(args=None):
    """解析参数并 spin 自主巡航节点。"""
    parser = argparse.ArgumentParser(description='自主巡航拍照任务')
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
