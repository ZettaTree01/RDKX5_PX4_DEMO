#!/usr/bin/env python3
"""停机坪 H 标对准降落。文档 4.2。

起飞后先悬停找 H。认到了就往画面中心挪，对准并稳住大约 1 秒，再请求降落。
速度和降落请求只发给 OFFBOARD 管理器，别直接往 MAVROS 设定点里塞。

摄像头朝下（或把 H 正对镜头）时：
  H 偏右 → 往右；H 偏下 → 往后；框偏小 → 下降，偏大 → 上升。
  画面上会标相对 H 的前/后/左/右/升/降。

找不到 H、或者图像断了超过 0.5 秒：零速悬着，不降落。
已经请求过降落：本节点不再发速度，交给管理器下降上锁。

室内 launch 默认 ``bench:=true``；加 ``arm:=true`` 才会强制解锁，最高大约 300 r/min。
"""
import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from frame_output import FrameOutput
from helipad_h import detect_h_mark
from indoor import RelAlt, TRACK_VEL_MPS

FONT = cv2.FONT_HERSHEY_SIMPLEX
IMAGE_TIMEOUT = 0.5
ALIGN_ERR = 0.12
ALIGN_HOLD_SEC = 1.0
DETECT_PERIOD = 0.12
TARGET_FILL = 0.22   # H 框高占画面该比例视为合适高度
_CN_FONT_PATHS = (
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/arphic/uming.ttc',
    'C:/Windows/Fonts/msyh.ttc',
    'C:/Windows/Fonts/simhei.ttf',
)
_CN_FONT_CACHE = {}
_CN_ASCII = (
    ('等待起飞', 'WAIT TAKEOFF'), ('等待解锁', 'WAIT ARM'),
    ('起飞中', 'TAKING OFF'), ('悬停搜索', 'HOVER SEARCH'),
    ('对准 H 标', 'ALIGN H'), ('已对准，降落', 'ALIGNED LAND'),
    ('画面丢失', 'NO IMAGE'), ('未发现 H 标', 'NO H MARK'),
    ('高度', 'ALT'), ('位移', 'MOVE'), ('相对H', 'vs H'),
    ('上升', 'UP'), ('下降', 'DOWN'), ('升', 'U'), ('降', 'D'),
    ('前', 'FWD'), ('后', 'BACK'), ('左', 'LEFT'), ('右', 'RIGHT'),
    ('悬停', 'HOVER'), ('已居中', 'CENTERED'),
)


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
        x, yy = xy
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            draw.text((x + dx, yy + dy), text, font=font, fill=(0, 0, 0))
        draw.text(xy, text, font=font, fill=rgb_c)
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


def _move_dirs(vx, vy, vz, max_vel):
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


def _h_offset_dirs(err_u, err_v, err_z, thresh=0.08):
    """相对 H 标：机体应往哪边动才能把 H 拉到画面中心/合适大小。"""
    dirs = []
    if err_v < -thresh:
        dirs.append('前')
    elif err_v > thresh:
        dirs.append('后')
    if err_u < -thresh:
        dirs.append('左')
    elif err_u > thresh:
        dirs.append('右')
    if err_z > thresh:
        dirs.append('下降')
    elif err_z < -thresh:
        dirs.append('上升')
    return dirs


def _h_offset_cmd(err_u, err_v, err_z, thresh=0.08):
    """把画面偏差转成 ±1 速度，供箭头高亮（与 _h_offset_dirs 同一阈值）。"""
    vx = 0.0 if abs(err_v) < thresh else (-1.0 if err_v > 0 else 1.0)
    vy = 0.0 if abs(err_u) < thresh else (-1.0 if err_u > 0 else 1.0)
    vz = 0.0 if abs(err_z) < thresh else (-1.0 if err_z > 0 else 1.0)
    return vx, vy, vz


def _draw_move_arrows(img, vx, vy, vz, max_vel):
    h, w = img.shape[:2]
    cx, cy = w - 78, h // 2 + 18
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

    arm(0, -reach, vz > eps)
    arm(0, reach, vz < -eps)
    arm(-reach, 0, vy > eps)
    arm(reach, 0, vy < -eps)
    moving = abs(vx) + abs(vy) + abs(vz) > eps
    cv2.circle(img, (cx, cy), 6, (0, 220, 255) if moving else (90, 90, 90), -1)
    bx, by = w // 2, h - 36
    if vx < -eps:
        cv2.arrowedLine(img, (bx, by - 28), (bx, by + 8), (0, 0, 255), 3, tipLength=0.4)
        cv2.arrowedLine(img, (bx, by - 44), (bx, by - 8), (0, 0, 255), 3, tipLength=0.4)
    elif vx > eps:
        cv2.arrowedLine(img, (bx, by + 8), (bx, by - 28), (0, 255, 0), 3, tipLength=0.4)
        cv2.arrowedLine(img, (bx, by - 8), (bx, by - 44), (0, 255, 0), 3, tipLength=0.4)


class HelipadLandingNode(Node):
    """视觉对准节点：检测 H → 发机体速度 → 对准后请求降落上锁。"""

    def __init__(self, show=True, snapshot=None, snapshot_period=5.0,
                 max_vel=TRACK_VEL_MPS, align_err=ALIGN_ERR,
                 align_hold=ALIGN_HOLD_SEC):
        super().__init__('helipad_landing')
        self.cmd = (0.0, 0.0, 0.0)          # 最新机体 FLU 速度指令
        self.bridge = CvBridge()
        self.max_velocity = float(max_vel)
        self.align_err = float(align_err)   # 归一化像素误差门限（居中判据）
        self.align_hold = float(align_hold) # 连续对准满该秒数才请求降落
        self.kp = 0.85                      # 误差 → 速度比例增益
        self.airborne = False
        self.armed = False
        self.alt_z = None
        self._rel_alt = RelAlt()
        self.landing = False
        self.last_image = None
        self.latest_frame = None            # 回调只缓存；检测在独立线程
        self.mark = None
        self.phase = '等待解锁'
        self.align_since = None
        self.h_err = (0.0, 0.0, 0.0)        # (err_u, err_v, err_z) 归一化
        self._last_status = 0.0
        self._lock = threading.Lock()
        self._stop = False
        self._hud_key = None
        self._hud_bar = None

        self.create_subscription(
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
        self.land_pub = self.create_publisher(
            Bool, '/drone/control/land', 10)
        self.out = FrameOutput(
            self, show=show, snapshot=snapshot,
            snapshot_period=snapshot_period,
            title='helipad (q/Esc 退出)',
            fallback_path='/tmp/tracking_snapshot.jpg')
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._preview_tick)
        self._worker = threading.Thread(target=self._detect_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'H 标降落已启动：起飞后悬停，对准后降落 '
            f'(max_vel={self.max_velocity}m/s, align={self.align_err})')

    def _on_state(self, msg):
        self.armed = bool(msg.armed)

    def _on_pose(self, msg):
        self.alt_z = self._rel_alt.update(msg.pose.position.z)

    def _wait_phase(self):
        if self.airborne:
            return '悬停搜索'
        if self.armed:
            return '起飞中'
        return '等待解锁'

    def image_callback(self, msg):
        self.last_image = self.get_clock().now()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(
                f'图像转换失败: {exc}', throttle_duration_sec=5.0)
            return
        with self._lock:
            self.latest_frame = frame

    def _preview_tick(self):
        with self._lock:
            frame = None if self.latest_frame is None else self.latest_frame
        if frame is not None:
            self._output(frame)

    def _image_ok(self):
        if self.last_image is None:
            return False
        age = (self.get_clock().now() - self.last_image).nanoseconds
        return age <= int(IMAGE_TIMEOUT * 1e9)

    def _request_land(self):
        with self._lock:
            if self.landing:
                return
            self.landing = True
            self.cmd = (0.0, 0.0, 0.0)
            self.phase = '已对准，降落'
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)
        self.get_logger().warn('H 标已对准，请求降落上锁（下降后停转）')

    def _align_cmd(self, cx, cy, box_h, frame_w, frame_h):
        """像素偏差 → 机体 FLU 速度；返回 (cmd, aligned, err_u, err_v, err_z)。

        err_u/err_v：相对画面中心的归一化偏差（右/下为正）。
        err_z：目标框填充比相对 TARGET_FILL 的差（偏小=偏远=应下降）。
        """
        error_u = (cx - frame_w / 2.0) / max(frame_w / 2.0, 1.0)
        error_v = (cy - frame_h / 2.0) / max(frame_h / 2.0, 1.0)
        fill = box_h / max(float(frame_h), 1.0)
        error_z = TARGET_FILL - fill
        lim = self.max_velocity
        # 机体：+x 前、+y 左、+z 上。画面下偏 → 机体后移（-x）
        vx = max(-lim, min(lim, -self.kp * error_v * lim))
        vy = max(-lim, min(lim, -self.kp * error_u * lim))
        vz = max(-lim, min(lim, -self.kp * error_z * lim))
        if abs(error_u) < 0.06:
            vy = 0.0
        if abs(error_v) < 0.06:
            vx = 0.0
        if abs(error_z) < 0.04:
            vz = 0.0
        aligned = (abs(error_u) <= self.align_err
                   and abs(error_v) <= self.align_err)
        return (float(vx), float(vy), float(vz)), aligned, error_u, error_v, error_z

    def _detect_loop(self):
        while not self._stop:
            t0 = time.monotonic()
            with self._lock:
                frame = (None if self.latest_frame is None
                         else self.latest_frame)
                frame = None if frame is None else frame.copy()
            if frame is not None:
                try:
                    self._run_detect(frame)
                except Exception as exc:
                    self.get_logger().error(
                        f'H 标检测失败: {exc}', throttle_duration_sec=2.0)
            time.sleep(max(0.01, DETECT_PERIOD - (time.monotonic() - t0)))

    def _run_detect(self, frame):
        if self.landing:
            with self._lock:
                self.cmd = (0.0, 0.0, 0.0)
                self.phase = '已对准，降落'
            return
        if not self._image_ok():
            with self._lock:
                self.cmd = (0.0, 0.0, 0.0)
                self.mark = None
                self.h_err = (0.0, 0.0, 0.0)
                self.align_since = None
                self.phase = '画面丢失'
            return
        if not self.airborne:
            with self._lock:
                self.cmd = (0.0, 0.0, 0.0)
                self.mark = None
                self.h_err = (0.0, 0.0, 0.0)
                self.align_since = None
                self.phase = self._wait_phase()
            self._status('等待解锁/起飞后再悬停搜索 H 标')
            return

        mark = detect_h_mark(frame)
        if mark is None:
            with self._lock:
                self.cmd = (0.0, 0.0, 0.0)
                self.mark = None
                self.h_err = (0.0, 0.0, 0.0)
                self.align_since = None
                self.phase = '悬停搜索'
            self._status('悬停搜索：未发现 H 标，电机保持悬停转速')
            return

        x1, y1, x2, y2, score = mark
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        cmd, aligned, err_u, err_v, err_z = self._align_cmd(
            cx, cy, (y2 - y1), frame.shape[1], frame.shape[0])
        now = time.monotonic()
        with self._lock:
            self.mark = mark
            self.cmd = cmd
            self.h_err = (err_u, err_v, err_z)
            if aligned:
                if self.align_since is None:
                    self.align_since = now
                held = now - self.align_since
                self.phase = f'对准 H 标 {held:.1f}s'
                should_land = held >= self.align_hold
            else:
                self.align_since = None
                self.phase = '对准 H 标'
                should_land = False
        if should_land:
            self._request_land()
        offset = _h_offset_dirs(err_u, err_v, err_z)
        dirs = '、'.join(offset) or '已居中'
        self._status(
            f'{self.phase} score={score:.2f} 相对H={dirs} '
            f'cmd=({cmd[0]:+.3f},{cmd[1]:+.3f},{cmd[2]:+.3f})')

    def _output(self, frame):
        if not self.out.enabled():
            return
        vis = frame.copy()
        h, w = vis.shape[:2]
        with self._lock:
            mark = self.mark
            phase = self.phase
            landing = self.landing
            airborne = self.airborne
            alt_z = self.alt_z
            h_err = self.h_err
        cv2.line(vis, (w // 2 - 16, h // 2), (w // 2 + 16, h // 2),
                 (255, 255, 0), 1)
        cv2.line(vis, (w // 2, h // 2 - 16), (w // 2, h // 2 + 16),
                 (255, 255, 0), 1)
        if mark is not None:
            x1, y1, x2, y2, score = mark
            color = (0, 255, 0) if ('对准' in phase or landing) else (0, 165, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.circle(vis, (int(0.5 * (x1 + x2)), int(0.5 * (y1 + y2))),
                       4, color, -1)
            cv2.putText(vis, f'H {score:.2f}', (x1, max(16, y1 - 6)),
                        FONT, 0.55, color, 2)
        bar_h = 88
        cv2.rectangle(vis, (0, 0), (w, bar_h), (0, 0, 0), -1)
        alt_txt = '高度 --' if alt_z is None else f'高度 {alt_z:.2f} m'
        lines = [(alt_txt, (0, 255, 255)), (phase, (0, 255, 255))]
        if mark is None and not landing and airborne:
            lines[1] = ('悬停搜索  未发现 H 标', (0, 0, 255))
        extra = []
        if mark is not None:
            offset = _h_offset_dirs(*h_err)
            rel = '相对H ' + ('、'.join(offset) if offset else '已居中')
            lines.append((rel, (0, 0, 255)))
            dvx, dvy, dvz = _h_offset_cmd(*h_err)
            _draw_move_arrows(vis, dvx, dvy, dvz, 1.0)
            cx, cy = w - 78, h // 2 + 18
            tag = (0, 220, 255)
            extra = [
                ('升', tag, (cx - 10, cy - 64)),
                ('降', tag, (cx - 10, cy + 46)),
                ('左', tag, (cx - 64, cy - 12)),
                ('右', tag, (cx + 46, cy - 12)),
            ]
            _err_u, err_v, err_z = h_err
            if err_v > 0.08:
                extra.append(('后', (0, 0, 255), (w // 2 - 12, h - 28)))
            elif err_v < -0.08:
                extra.append(('前', (0, 255, 0), (w // 2 - 12, h - 28)))
            if err_z > 0.08:
                extra.append(('下降', (0, 0, 255), (cx - 18, cy + 68)))
            elif err_z < -0.08:
                extra.append(('上升', (0, 255, 0), (cx - 18, cy - 84)))
        elif airborne and not landing:
            lines.append(('位移 悬停', (0, 255, 255)))
        hud_key = (w, tuple((t, c) for t, c in lines))
        if hud_key != self._hud_key or self._hud_bar is None:
            bar = np.zeros((bar_h, w, 3), np.uint8)
            _put_cn_lines(bar, lines, origin=(10, 4), size=20)
            self._hud_bar = bar
            self._hud_key = hud_key
        vis[0:bar_h] = np.maximum(vis[0:bar_h], self._hud_bar)
        if extra:
            _put_cn_lines(vis, extra, size=18)
        self.out.output(vis)

    def _status(self, text):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_status < 1.0:
            return
        self._last_status = now
        self.get_logger().info(text)

    def _tick(self):
        with self._lock:
            cmd = self.cmd
            landing = self.landing
            airborne = self.airborne
        if landing:
            cmd = (0.0, 0.0, 0.0)
        elif not airborne:
            cmd = (0.0, 0.0, 0.0)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = cmd
        self.velocity_pub.publish(m)
        if landing:
            msg = Bool()
            msg.data = True
            self.land_pub.publish(msg)

    def destroy_node(self):
        self._stop = True
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)
        self.out.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='停机坪 H 标对准降落')
    parser.add_argument('--show', dest='show', action='store_true',
                        default=True, help='输出对准画面（默认输出）')
    parser.add_argument('--no-show', dest='show', action='store_false',
                        help='关闭画面输出')
    parser.add_argument('--snapshot', default=None,
                        help='定期把画面写到该 JPEG 路径')
    parser.add_argument('--snapshot-period', type=float, default=5.0,
                        help='快照间隔秒数，默认 5')
    parser.add_argument('--max-vel', type=float, default=TRACK_VEL_MPS,
                        help=f'对准速度上限（m/s），默认 {TRACK_VEL_MPS}')
    parser.add_argument('--align-err', type=float, default=ALIGN_ERR,
                        help=f'对准误差阈值（画面半宽比例），默认 {ALIGN_ERR}')
    parser.add_argument('--align-hold', type=float, default=ALIGN_HOLD_SEC,
                        help=f'对准保持秒数后降落，默认 {ALIGN_HOLD_SEC}')
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = HelipadLandingNode(
        parsed.show, parsed.snapshot, parsed.snapshot_period,
        parsed.max_vel, parsed.align_err, parsed.align_hold)
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
