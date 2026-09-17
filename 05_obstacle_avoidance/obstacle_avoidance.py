#!/usr/bin/env python3
"""
摄像头识别避障任务。文档 3.2。

本节点订阅 /camera/image_raw，用共享组件 _common/yolo_detector.py 检测障碍，
估算最近障碍距离，按「越近退得越快」向 OFFBOARD 管理器发布机体 FLU 反向速度。
不直接控制 MAVROS。图像超过 0.5 秒无数据时发布零速度。

距离取两种估法的较小值：
  1) 小孔成像：fx × 类别典型高度 / 框高；
  2) 画面占比：框高占画面 50% 时视为约 0.8 m（补偿 USB 广角估远）。
反向速度与距离成比例：刚进入安全区约 20% 最大速度，贴脸时到 100%。
画面始终显示当前高度；触发避障时显示位移方向（前/后/左/右/上升/下降）。

推理与图像回调解耦：回调只缓存最新帧，定时器按 infer-hz 推理。

室内调试：05 launch 默认拉起台架位姿模拟；arm:=true 后先爬升拉转速，
再悬停保持转速，靠近障碍按距离加速，最高 300 r/min。
画面高度为相对开机位置，避免室内气压计显示几十米。
"""
import argparse
import os
import sys
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from frame_output import FrameOutput
from indoor import AVOID_VEL_MPS, RelAlt
from yolo_detector import YoloDetector
from depth_rgbd import image_msg_to_bgr

FONT = cv2.FONT_HERSHEY_SIMPLEX
# 机体 FLU：+x 前、+y 左、+z 上
_CN_FONT_PATHS = (
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/arphic/uming.ttc',
    'C:/Windows/Fonts/msyh.ttc',
    'C:/Windows/Fonts/simhei.ttf',
)
_CN_FONT_CACHE = {}
_CN_ASCII = (
    ('上升', 'UP'), ('下降', 'DOWN'), ('升', 'U'), ('降', 'D'),
    ('高度', 'ALT'), ('位移', 'MOVE'), ('障碍', 'OBS'),
    ('安全', 'CLEAR'), ('无目标', 'NO TARGET'), ('悬停', 'HOLD'),
    ('等待解锁', 'WAIT ARM'), ('起飞中', 'TAKEOFF'),
    ('模型未加载', 'NO MODEL'), ('推理失败', 'INFER FAIL'),
    ('前', 'FWD'), ('后', 'BACK'), ('左', 'LEFT'), ('右', 'RIGHT'),
)

SAFE_DISTANCE = 4.0      # 米，室内从该距离开始按比例后退（越近越快）
MAX_VEL = AVOID_VEL_MPS  # m/s，室内调试已乘 INDOOR_SPEED_SCALE
IMAGE_TIMEOUT = 0.5      # 秒，图像断流超时归零
DEFAULT_HFOV = 90.0     # 度，常见 USB 广角；原 60° 会把近处椅子估成 2 m+
DEFAULT_INFER_HZ = 10.0
# 画面占比：框高占画面该比例时视为 FILL_DIST_M（室内广角经验）
FILL_FRAC_AT_REF = 0.50
FILL_DIST_M = 0.80

REAL_HEIGHTS = {
    'person': 1.70, 'bicycle': 1.10, 'car': 1.50, 'motorcycle': 1.10,
    'airplane': 4.00, 'bus': 3.20, 'train': 3.80, 'truck': 3.00,
    'boat': 1.50, 'dog': 0.50, 'horse': 1.60, 'cow': 1.50,
    'cat': 0.35, 'chair': 0.90, 'couch': 0.90, 'potted plant': 0.45,
    'bed': 0.60, 'bird': 0.25, 'bottle': 0.30, 'cup': 0.12,
    'backpack': 0.50, 'umbrella': 0.90,
    'tv': 0.50, 'laptop': 0.22, 'keyboard': 0.04, 'cell phone': 0.14,
    'book': 0.25, 'dining table': 0.75, 'refrigerator': 1.70,
    'microwave': 0.30, 'oven': 0.85, 'sink': 0.85, 'toilet': 0.80,
    'clock': 0.30, 'vase': 0.25, 'teddy bear': 0.30,
}
DEFAULT_HEIGHT = 0.5
REAL_WIDTHS = {
    'person': 0.50, 'chair': 0.50, 'couch': 1.60, 'dining table': 0.90,
    'tv': 0.80, 'laptop': 0.35, 'bottle': 0.08, 'cup': 0.08,
    'potted plant': 0.30, 'dog': 0.25, 'cat': 0.20, 'backpack': 0.30,
}
DEFAULT_WIDTH = 0.40


def _cn_font(size):
    key = int(size)
    if key in _CN_FONT_CACHE:
        return _CN_FONT_CACHE[key]
    try:
        from PIL import ImageFont
    except ImportError:
        _CN_FONT_CACHE[key] = None
        return None
    for path in _CN_FONT_PATHS:
        if not os.path.isfile(path):
            continue
        try:
            font = ImageFont.truetype(path, key)
            _CN_FONT_CACHE[key] = font
            return font
        except Exception:
            continue
    _CN_FONT_CACHE[key] = None
    return None


def _ascii_hud(text):
    out = text
    for cn, en in _CN_ASCII:
        out = out.replace(cn, en)
    return out


def _put_cn_lines(img, lines, origin=(10, 8), size=22):
    """lines: [(text, bgr), ...] 或 (text, bgr, (x, y))。优先中文字体。"""
    if not lines:
        return
    font = _cn_font(size)
    x0, y0 = origin
    positioned = []
    y = y0
    for item in lines:
        if len(item) == 3:
            text, color, xy = item
        else:
            text, color = item
            xy = (x0, y)
            y += size + 8
        positioned.append((text, color, xy))
    if font is None:
        for text, color, xy in positioned:
            cv2.putText(img, _ascii_hud(text), (xy[0], xy[1] + 18),
                        FONT, 0.62, color, 2)
        return
    from PIL import Image, ImageDraw
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil)
    for text, color, xy in positioned:
        rgb_c = (int(color[2]), int(color[1]), int(color[0]))
        x, y = xy
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            draw.text((x + dx, y + dy), text, font=font, fill=(0, 0, 0))
        draw.text(xy, text, font=font, fill=rgb_c)
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def _move_dirs(vx, vy, vz, max_vel):
    """按机体 FLU 速度拆成前/后/左/右/上升/下降。"""
    eps = max(1e-4, 0.12 * abs(float(max_vel)))
    dirs = []
    if vx > eps:
        dirs.append('前')
    elif vx < -eps:
        dirs.append('后')
    if vy > eps:
        dirs.append('左')
    elif vy < -eps:
        dirs.append('右')
    if vz > eps:
        dirs.append('上升')
    elif vz < -eps:
        dirs.append('下降')
    return dirs


def _draw_move_arrows(img, vx, vy, vz, max_vel):
    """右侧画位移箭头：画面上=升、下=降、左=左、右=右；后退画向下的双箭头。"""
    h, w = img.shape[:2]
    cx, cy = w - 78, h // 2 + 10
    reach = 42
    n = max(abs(max_vel), 1e-6)
    eps = 0.12 * n
    color_idle = (80, 80, 80)
    color_act = (0, 220, 255)

    def arm(dx, dy, active):
        color = color_act if active else color_idle
        thick = 3 if active else 1
        cv2.arrowedLine(
            img, (cx, cy), (cx + dx, cy + dy), color, thick, tipLength=0.35)

    arm(0, -reach, vz > eps)          # 上升
    arm(0, reach, vz < -eps)         # 下降
    arm(-reach, 0, vy > eps)         # 左
    arm(reach, 0, vy < -eps)         # 右
    moving = abs(vx) + abs(vy) + abs(vz) > eps
    cv2.circle(img, (cx, cy), 6, (0, 220, 255) if moving else (90, 90, 90), -1)

    bx, by = w // 2, h - 36
    if vx < -eps:
        cv2.arrowedLine(img, (bx, by - 28), (bx, by + 8), (0, 0, 255), 3, tipLength=0.4)
        cv2.arrowedLine(img, (bx, by - 44), (bx, by - 8), (0, 0, 255), 3, tipLength=0.4)
    elif vx > eps:
        cv2.arrowedLine(img, (bx, by + 8), (bx, by - 28), (0, 255, 0), 3, tipLength=0.4)
        cv2.arrowedLine(img, (bx, by - 8), (bx, by - 44), (0, 255, 0), 3, tipLength=0.4)


class ObstacleAvoidanceNode(Node):
    """YOLO 估距 → 机体 FLU 反向速度 → 交给 offboard_manager。"""
    def __init__(self, show=True, snapshot=None, snapshot_period=5.0,
                 hfov=DEFAULT_HFOV, safe_distance=SAFE_DISTANCE,
                 max_vel=MAX_VEL, score_thres=0.25, nms_thres=0.45,
                 infer_hz=DEFAULT_INFER_HZ):
        super().__init__('obstacle_avoidance')
        self.cmd = (0.0, 0.0, 0.0)
        self.hfov = float(hfov)
        self.safe_distance = float(safe_distance)
        self.max_vel = float(max_vel)
        self.last_image = None
        self.latest_frame = None
        self.airborne = False
        self.armed = False
        self.alt_z = None
        self._rel_alt = RelAlt()
        self.bridge = CvBridge()
        self._last_status = 0.0

        self.detector = YoloDetector(
            score_thres=score_thres, nms_thres=nms_thres,
            log=self.get_logger())

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/drone/status/airborne',
            lambda msg: setattr(self, 'airborne', msg.data), 10)
        self.create_subscription(
            State, '/mavros/state', self._on_state, 10)
        self.create_subscription(
            State, '/mavros/state', self._on_state, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_profile_sensor_data)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/drone/setpoint_velocity/body', 10)

        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='avoid (q/Esc 退出)',
            fallback_path='/tmp/avoid_snapshot.jpg')

        infer_hz = max(1.0, float(infer_hz))
        self.create_timer(1.0 / infer_hz, self._infer_tick)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'避障任务已启动，等待 /camera/image_raw '
            f'(safe_distance={self.safe_distance}m, hfov={self.hfov}°, '
            f'max_vel={self.max_vel}m/s, infer_hz={infer_hz})')

    def _on_state(self, msg):
        self.armed = bool(msg.armed)

    def _on_pose(self, msg):
        self.alt_z = self._rel_alt.update(msg.pose.position.z)

    def image_callback(self, msg):
        self.last_image = self.get_clock().now()
        try:
            self.latest_frame = image_msg_to_bgr(msg, self.bridge)
        except Exception as exc:
            self.get_logger().error(
                f'图像转换失败: {exc}', throttle_duration_sec=5.0)
            self.latest_frame = None

    def _avoid_speed(self, distance, yaw, pitch):
        """距离越近，反向速度越大。刚越界约 20% 上限，距离 0 时 100%。

        水平沿障碍方位后退；画面偏下则上升、偏上则下降。
        """
        if distance >= self.safe_distance:
            return (0.0, 0.0, 0.0)
        ratio = (self.safe_distance - distance) / self.safe_distance
        ratio = max(0.0, min(1.0, ratio))
        speed = self.max_vel * (0.2 + 0.8 * ratio)
        return (
            -speed * float(np.cos(yaw)),
            -speed * float(np.sin(yaw)),
            speed * float(np.sin(pitch)))

    def _infer_tick(self):
        frame = self.latest_frame
        if frame is None:
            return

        if not self.detector.loaded:
            self.cmd = (0.0, 0.0, 0.0)
            self._output(frame, [], None, None, None, '模型未加载')
            self._status('model not loaded')
            return

        detections = []
        try:
            detections = self.detector.detect(frame)
        except Exception:
            self.get_logger().error(
                '推理失败:\n' + traceback.format_exc(),
                throttle_duration_sec=5.0)
            self.cmd = (0.0, 0.0, 0.0)
            self._output(frame, [], None, None, None, '推理失败')
            self._status('inference failed')
            return

        nearest = self.nearest_obstacle(
            detections, frame.shape[1], frame.shape[0])
        if nearest is None:
            self.cmd = (0.0, 0.0, 0.0)
            self._output(frame, detections, None, None, None, '无目标')
            self._status(f'no detections n={len(detections)}')
            return

        distance, yaw, pitch, idx = nearest
        self.cmd = self._avoid_speed(distance, yaw, pitch)
        self._output(frame, detections, distance, idx, yaw, None)
        vx, vy, vz = self.cmd
        dirs = '、'.join(_move_dirs(vx, vy, vz, self.max_vel)) or '—'
        alt = '--' if self.alt_z is None else f'{self.alt_z:.2f}'
        if self.airborne:
            phase = '悬停'
        elif self.armed:
            phase = '起飞中'
        else:
            phase = '等待解锁'
        self._status(
            f'{phase} alt={alt}m n={len(detections)} nearest={distance:.2f}m '
            f'cmd=({vx:+.3f},{vy:+.3f},{vz:+.3f}) {dirs}')

    def nearest_obstacle(self, detections, frame_w, frame_h):
        """取估算距离最近的目标。返回 (距离 m, 方位 yaw, 俯仰 pitch, 索引) 或 None。"""
        cx = frame_w / 2.0
        cy = frame_h / 2.0
        fx = frame_w / (2.0 * np.tan(np.radians(self.hfov) / 2.0))
        fy = fx
        best = None
        for i, (x1, y1, x2, y2, _score, cls_id) in enumerate(detections):
            box_h = y2 - y1
            box_w = x2 - x1
            if box_h < 2.0 or box_w < 2.0:
                continue
            real_h = REAL_HEIGHTS.get(
                self.detector.label(cls_id), DEFAULT_HEIGHT)
            real_w = REAL_WIDTHS.get(
                self.detector.label(cls_id), DEFAULT_WIDTH)
            pinhole_h = fx * real_h / box_h
            pinhole_w = fx * real_w / box_w
            frac = box_h / max(float(frame_h), 1.0)
            fill = FILL_FRAC_AT_REF / max(frac, 0.02) * FILL_DIST_M
            distance = min(float(pinhole_h), float(pinhole_w), float(fill))
            yaw = float(np.arctan(((x1 + x2) / 2.0 - cx) / fx))
            pitch = float(np.arctan(((y1 + y2) / 2.0 - cy) / fy))
            if best is None or distance < best[0]:
                best = (float(distance), yaw, pitch, i)
        return best

    def _output(self, frame, detections, distance, idx, _yaw, note):
        if not self.out.enabled():
            return
        vis = frame.copy()
        for i, (x1, y1, x2, y2, score, cls_id) in enumerate(detections):
            is_trigger = (i == idx and distance is not None
                          and distance < self.safe_distance)
            color = (0, 0, 255) if is_trigger else (0, 255, 0)
            text = f'{self.detector.label(cls_id)} {score:.2f}'
            if i == idx and distance is not None:
                text += f' {distance:.1f}m'
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)),
                          color, 2)
            cv2.putText(vis, text, (int(x1), max(12, int(y1) - 6)),
                        FONT, 0.55, color, 2)

        h, w = vis.shape[:2]
        bar_h = 64
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, vis, 0.45, 0, vis)
        vis = np.ascontiguousarray(vis)

        alt_txt = ('高度 --' if self.alt_z is None
                   else f'高度 {self.alt_z:.2f} m')
        lines = [(alt_txt, (0, 255, 255))]
        avoiding = (distance is not None and distance < self.safe_distance)
        vx, vy, vz = self.cmd
        if not self.airborne:
            phase = '起飞中' if self.armed else '等待解锁'
            lines.append((phase, (0, 255, 255)))
        elif avoiding:
            dirs = _move_dirs(vx, vy, vz, self.max_vel)
            move_txt = '位移 ' + ('、'.join(dirs) if dirs else '悬停')
            lines.append((f'{move_txt}  障碍 {distance:.1f} m', (0, 0, 255)))
            _draw_move_arrows(vis, vx, vy, vz, self.max_vel)
            cx, cy = w - 78, h // 2 + 10
            tag = (0, 220, 255)
            lines.extend([
                ('升', tag, (cx - 10, cy - 64)),
                ('降', tag, (cx - 10, cy + 46)),
                ('左', tag, (cx - 64, cy - 12)),
                ('右', tag, (cx + 46, cy - 12)),
            ])
            if vx < -0.12 * self.max_vel:
                lines.append(('后', (0, 0, 255), (w // 2 - 12, h - 28)))
            elif vx > 0.12 * self.max_vel:
                lines.append(('前', (0, 255, 0), (w // 2 - 12, h - 28)))
        elif note:
            lines.append((note, (0, 0, 255)))
        elif distance is not None:
            lines.append((f'安全 {distance:.1f} m', (0, 255, 0)))
        _put_cn_lines(vis, lines, origin=(10, 6), size=22)
        self.out.output(vis)

    def _status(self, text):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_status < 1.0:
            return
        self._last_status = now
        self.get_logger().info(text)

    def _tick(self):
        cmd = self.cmd
        if (self.last_image is None
                or (self.get_clock().now() - self.last_image).nanoseconds
                > int(IMAGE_TIMEOUT * 1e9)):
            cmd = (0.0, 0.0, 0.0)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = cmd
        self.velocity_pub.publish(m)

    def destroy_node(self):
        self.out.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='摄像头识别避障 ROS 节点')
    parser.add_argument('--show', dest='show', action='store_true',
                        default=True,
                        help='输出避障画面（默认输出）')
    parser.add_argument('--no-show', dest='show', action='store_false',
                        help='关闭避障画面输出')
    parser.add_argument('--snapshot', default=None,
                        help='定期把避障画面写到该 JPEG 路径')
    parser.add_argument('--snapshot-period', type=float, default=5.0,
                        help='快照间隔秒数，默认 5')
    parser.add_argument('--hfov', type=float, default=DEFAULT_HFOV,
                        help=f'摄像头水平视场角（度），默认 {DEFAULT_HFOV}')
    parser.add_argument('--safe-distance', type=float, default=SAFE_DISTANCE,
                        help=f'触发避障的距离阈值（米），默认 {SAFE_DISTANCE}')
    parser.add_argument('--max-vel', type=float, default=MAX_VEL,
                        help=f'避障反向速度上限（m/s），默认 {MAX_VEL}（室内 1/20）')
    parser.add_argument('--score-thres', type=float, default=0.25,
                        help='置信度阈值（概率域），默认 0.25')
    parser.add_argument('--nms-thres', type=float, default=0.45,
                        help='NMS IoU 阈值，默认 0.45')
    parser.add_argument('--infer-hz', type=float, default=DEFAULT_INFER_HZ,
                        help=f'推理频率（Hz），默认 {DEFAULT_INFER_HZ}')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = ObstacleAvoidanceNode(
        parsed.show, parsed.snapshot, parsed.snapshot_period, parsed.hfov,
        parsed.safe_distance, parsed.max_vel, parsed.score_thres,
        parsed.nms_thres, parsed.infer_hz)
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
