#!/usr/bin/env python3
"""PX4 OFFBOARD 管理器（文档 2.7）。

各飞行例程的 launch 会拉起本节点。飞控侧设定点、解锁、切模式与写参数
均由此节点发出；任务节点不得并行向 MAVROS setpoint 话题发布，否则会抢占控制权。

任务节点仅通过以下话题与本节点交互：

  /drone/setpoint_position/local   本地 ENU 位置
  /drone/setpoint_velocity/body    机体 FLU 速度
  /drone/control/land              请求降落
  /drone/status/airborne           本节点对外发布「已起飞」状态

未指定 ``--arm`` 时为监视模式：持续发送「保持当前位置」设定点，不解锁。
指定 ``--bench`` 表示室内拆桨台架模式：写入一组 RAM 参数，以姿态设定点进入
OFFBOARD，再强制解锁（21196）。强制解锁仍会执行健康检查，``COM_ARM_IMU_*``
不得设为 0。

Ctrl+C 时本节点尽量请求上锁；``run.sh`` 退出时还会经串口强制上锁一次。

主流程（约 20 Hz）：写参数 → 预热设定点 → OFFBOARD 解锁 → 起飞/悬停 →
转发任务；任务超过 0.5 s 未更新则保持悬停。收到降落请求后，台架模式下下降后上锁，
实飞则切换 AUTO.LAND。
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandHome, CommandLong, SetMode
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool

try:
    from geographic_msgs.msg import GeoPointStamped
except ImportError:
    GeoPointStamped = None


# 板端 MAVROS 2.x 的 /mavros/param/set 实际是 ParamSetV2；旧板回退 ParamSet。
try:
    from mavros_msgs.srv import ParamSetV2 as ParamSetSrv
    _PARAM_SET_V2 = True
except ImportError:
    from mavros_msgs.msg import ParamValue
    from mavros_msgs.srv import ParamSet as ParamSetSrv
    _PARAM_SET_V2 = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from indoor import (
    ACC_DOWN, ACC_HOR, ACC_UP, BENCH_VEL_EPS, HOVER_THRUST, JERK_AUTO,
    LAND_SPEED, MAX_MOTOR_RPM, RAMP_SECONDS, TAKEOFF_ALT_M, THR_MAX,
    THR_MIN, TKO_SPEED, XY_VEL_MAX, Z_VEL_MAX)

# PX4 events::ID 的 FNV-1a 低 24 位；mavros 常把 EVENT <id> 打进 STATUSTEXT。
# 用此表把拒解锁原因翻译为可读说明，避免只看见数字。
_PX4_EVENT_NAMES = {
    133277: 'Arming denied: Resolve system health failures first',
    276785: 'Press safety button first',
    377561: 'No valid mission available',
    453929: 'No manual control input',
    79408: 'Home position not set',
    2481841: 'Gyro inconsistent between IMUs',
    3061044: 'Accel inconsistent between IMUs',
    3087815: 'No offboard signal',
    3613628: 'High Accelerometer Bias',
    5444856: 'Arming check (unknown detail; see QGC)',
    6801787: 'No GCS datalink',
    7662152: 'USB connected',
    9697819: 'Arming check (unknown detail; see QGC)',
    9634798: 'Heading estimate not stable',
    10011251: 'No valid global position estimate',
    10716939: 'Onboard control regained',
    11047904: 'arming check summary',
    13835193: 'No valid local position estimate',
    16642797: 'Heading estimate not stable',
    1914663: 'health summary',
}

try:
    from mavros_msgs.msg import AttitudeTarget
except ImportError:
    AttitudeTarget = None
try:
    from mavros_msgs.msg import ManualControl
except ImportError:
    ManualControl = None
try:
    from mavros_msgs.msg import ESCTelemetry
except ImportError:
    ESCTelemetry = None
try:
    from mavros_msgs.msg import ESCStatus
except ImportError:
    ESCStatus = None
try:
    from mavros_msgs.msg import OverrideRCIn
except ImportError:
    OverrideRCIn = None
try:
    from mavros_msgs.msg import HomePosition
except ImportError:
    HomePosition = None


class OffboardManager(Node):
    """飞控设定点的唯一出口：帮任务节点管解锁、起飞和降落。"""

    def __init__(self, altitude=TAKEOFF_ALT_M, arm_allowed=False, bench=False):
        super().__init__('offboard_manager')
        self.altitude = altitude
        self.arm_allowed = arm_allowed  # False=监视，不切模式/不解锁
        self.bench = bench              # True=室内台架（强制解锁 + 姿态预热）

        # 飞控与本地点
        self.state = State()
        self.pose = None                # (x,y,z, qx,qy,qz,qw) 本地 ENU
        self.home = None                # 开机/重定原点时的位置（本节点）
        self._px4_home_requested = False  # 已向飞控发过 DO_SET_HOME
        self._px4_home_ok = False         # 已收到 /mavros/home_position/home
        self._origin_requested = False
        self._home_cmd_t0 = None
        self.hold_target = None         # 无任务时锁定的目标点
        self.hold_orientation = None

        # 任务节点最近一次输入（超时见 _task_is_fresh）
        self.task_position = None
        self.task_velocity_body = None
        self.task_kind = None           # 'position' | 'velocity' | None
        self.task_time = None

        # 20 Hz 节拍计数与飞行阶段
        self.connected_ticks = 0
        self.setpoint_ticks = 0         # 已连续发布设定点的 tick 数（预热用）
        self.airborne = False           # 对外：是否已完成起飞斜坡/达到高度
        self.landing = False            # 已收到降落请求，不再解锁
        self._offboard_ticks = 0        # 连续处于 OFFBOARD 的 tick（再解锁）

        # 模式/解锁服务限流（避免 UART 被刷爆）
        self.mode_request_pending = False
        self.arm_request_pending = False
        self.last_mode_request = None
        self.last_arm_request = None

        # 台架参数队列：解锁相关写完才允许 arm；油门参数可稍后
        self.params_done = not bench
        self._arm_params_ready = not bench
        self._arm_ready_since = None if bench else self.get_clock().now()
        self._arm_param_names = set()
        self.param_pending = False
        self._param_queue = []
        self._param_sent_name = None
        self._param_sent_time = None

        # 最近飞控拒解锁相关 STATUSTEXT / EVENT（用于错误提示，勿写死 IMU）
        self._recent_fcu_fails = []

        self._logged_ignore_vel = False
        self._logged_hover = False
        self._arm_t0 = None             # 台架起飞斜坡起点（monotonic）
        self._hover_captured = False
        self._esc_max_rpm = 0           # 遥测到的最大电调转速，用于限速
        self._land_t0 = None
        self._logged_land_done = False
        self._disarm_request_pending = False
        self.last_disarm_request = None

        # ---- 订阅 / 发布 / 服务 ----
        self.create_subscription(
            State, '/mavros/state', self._on_state, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._on_pose,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/drone/setpoint_position/local',
            self._on_task_position, 10)
        self.create_subscription(
            TwistStamped, '/drone/setpoint_velocity/body',
            self._on_task_velocity, 10)
        self.create_subscription(
            Bool, '/drone/control/land', self._on_land, 10)
        self.position_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        # 台架解锁前用姿态设定点：OFFBOARD 预检不要求本地点（室内无 GPS 关键）
        self.attitude_pub = None
        if AttitudeTarget is not None:
            self.attitude_pub = self.create_publisher(
                AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        # 假遥控：满足「有摇杆」类检查；不要为此把 COM_RC_IN_MODE 写成 4
        self.manual_pub = None
        if ManualControl is not None:
            self.manual_pub = self.create_publisher(
                ManualControl, '/mavros/manual_control/send', 10)
        self.rc_override_pub = None
        if OverrideRCIn is not None:
            self.rc_override_pub = self.create_publisher(
                OverrideRCIn, '/mavros/rc/override', 10)
        self.home_set_pub = None
        if HomePosition is not None:
            self.home_set_pub = self.create_publisher(
                HomePosition, '/mavros/home_position/set', 10)
            self.create_subscription(
                HomePosition, '/mavros/home_position/home',
                self._on_home_position, 10)
        self.airborne_pub = self.create_publisher(
            Bool, '/drone/status/airborne', 10)
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')
        self.home_cli = self.create_client(CommandHome, '/mavros/cmd/set_home')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.param_cli = self.create_client(ParamSetSrv, '/mavros/param/set')
        self.gp_origin_pub = None
        if GeoPointStamped is not None:
            self.gp_origin_pub = self.create_publisher(
                GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)
            self.create_subscription(
                GeoPointStamped, '/mavros/global_position/gp_origin',
                self._on_gp_origin, 10)
        self._gp_origin_ok = False
        self.create_subscription(
            StatusText, '/mavros/statustext/recv', self._on_statustext,
            # mavros 用 RELIABLE；SensorDataQoS 是 BEST_EFFORT，会完全收不到
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST),
        )
        # EVENT 常只打到 mavros.sys 的 /rosout，不进 StatusText
        self.create_subscription(Log, '/rosout', self._on_rosout, 50)
        self._mavros_set_param_cli = self.create_client(
            SetParameters, '/mavros/set_parameters')
        self._thrust_scaling_fixed = False
        if ESCTelemetry is not None:
            self.create_subscription(
                ESCTelemetry, '/mavros/esc_telemetry', self._on_esc, 10)
        if ESCStatus is not None:
            self.create_subscription(
                ESCStatus, '/mavros/esc_status', self._on_esc,
                qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)          # 20 Hz 控制环
        self.create_timer(2.0, self._status_tick)    # 状态摘要日志
        if arm_allowed:
            self.get_logger().warn('已收到 --arm：定位有效并预热完成后将自动解锁')
        else:
            self.get_logger().info('监视模式：未传 --arm，不会切模式或解锁')
        if bench:
            self.get_logger().warn(
                f'台架：怠速 {THR_MIN:.3f} → 悬停 {HOVER_THRUST:.3f} → '
                f'最高 {THR_MAX:.3f}（{MAX_MOTOR_RPM} r/min），'
                f'加速约 {ACC_HOR:.3f} m/s²')

    def _on_state(self, msg):
        """缓存 /mavros/state（connected / armed / mode）。"""
        self.state = msg

    def _on_pose(self, msg):
        """更新本地点；首次或台架大幅跳变时重定 home / hold。"""
        p = msg.pose.position
        q = msg.pose.orientation
        self.pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if self.home is None:
            self.home = (p.x, p.y, p.z)
            self.hold_orientation = (q.x, q.y, q.z, q.w)
            # 台架解锁前先钉在当前高度，避免模拟器提前“飞”到起飞高度、解锁后看不出爬升。
            if self.bench:
                self.hold_target = (p.x, p.y, p.z)
            else:
                target_z = p.z + self.altitude if self.arm_allowed else p.z
                self.hold_target = (p.x, p.y, target_z)
        elif self.bench and abs(p.z - self.home[2]) > 2.0:
            # 视觉对齐瞬移或气压跳变：重定原点，避免相对高度乱飞
            self.home = (p.x, p.y, p.z)
            self.hold_orientation = (q.x, q.y, q.z, q.w)
            if not self.state.armed:
                self.hold_target = (p.x, p.y, p.z)
            self.get_logger().warn(
                f'台架本地点跳变到 z={p.z:.2f} m，已重定原点')

    def _on_task_position(self, msg):
        """任务节点：本地 ENU 位置目标。"""
        self.task_position = (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.task_kind = 'position'
        self.task_time = self.get_clock().now()

    def _on_task_velocity(self, msg):
        """任务节点：机体 FLU 速度（前/左/上）。"""
        self.task_velocity_body = (
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        self.task_kind = 'velocity'
        self.task_time = self.get_clock().now()

    def _on_land(self, msg):
        """任务节点请求降落：置位后由 _handle_landing 收尾上锁。"""
        if msg.data and not self.landing:
            self.landing = True
            self._land_t0 = None
            self._logged_land_done = False
            self.get_logger().warn('收到降落请求，开始降落并准备上锁停转')

    def _note_fcu_fail(self, reason):
        """记录最近拒解锁相关 FCU 原文，供强制解锁失败时打印。"""
        if not reason:
            return
        if reason in self._recent_fcu_fails:
            return
        self._recent_fcu_fails.append(reason)
        if len(self._recent_fcu_fails) > 8:
            self._recent_fcu_fails = self._recent_fcu_fails[-8:]

    def _on_rosout(self, msg):
        """从 mavros /rosout 抓 FCU EVENT（多数固件不发到 StatusText）。"""
        name = (msg.name or '')
        text = (msg.msg or '').strip()
        if not text or 'mavros' not in name:
            return
        if 'EVENT' not in text.upper() and 'FCU:' not in text:
            return
        # 去掉前缀「FCU: 」
        if text.upper().startswith('FCU:'):
            text = text[4:].strip()
        decoded = self._decode_fcu_event(text)
        if decoded:
            self._note_fcu_fail(decoded)
            self.get_logger().warn(f'FCU: {decoded}')
        elif 'EVENT' in text.upper():
            self._note_fcu_fail(text)
            self.get_logger().warn(f'FCU: {text}')

    def _fix_mavros_thrust_scaling(self):
        """yaml 里 thrust_scaling=1.0 仍可能未生效 → ignore_thrust；运行时强制写入。"""
        if self._thrust_scaling_fixed:
            return
        if not self._mavros_set_param_cli.service_is_ready():
            return
        # 只尝试一次，避免刷屏；失败则改用速度设定点预热
        self._thrust_scaling_fixed = True
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'setpoint_raw.thrust_scaling'
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)
        req.parameters = [p]
        fut = self._mavros_set_param_cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
                ok = all(r.successful for r in res.results)
                if ok:
                    self.get_logger().info(
                        '已设置 mavros setpoint_raw.thrust_scaling=1.0')
                else:
                    self.get_logger().warn(
                        'thrust_scaling 无法运行时写入（将用速度设定点预热）: '
                        + '; '.join(
                            r.reason for r in res.results if not r.successful))
            except Exception as exc:
                self.get_logger().warn(f'设置 thrust_scaling 异常: {exc}')

        fut.add_done_callback(_done)

    def _on_statustext(self, msg):
        """过滤飞控 STATUSTEXT；EVENT 数字尽量译成可读拒解锁原因。"""
        text = (msg.text or '').strip()
        if not text:
            return
        decoded = self._decode_fcu_event(text)
        if decoded:
            self._note_fcu_fail(decoded)
            self.get_logger().warn(f'FCU: {decoded}')
            return
        key = text.lower()
        if any(token in key for token in (
                'arm', 'preflight', 'fail', 'gps', 'ekf', 'offboard',
                'safety', 'switch', 'failsafe', 'denied', 'health',
                'check', 'rc ', 'radio', 'kill', 'usb', 'event',
                'bias', 'accel', 'compass', 'mag')):
            self._note_fcu_fail(text)
            self.get_logger().warn(f'FCU: {text}')
        elif text:
            self.get_logger().info(f'FCU: {text}', throttle_duration_sec=3.0)

    def _decode_fcu_event(self, text):
        """从「EVENT 133277 …」抽出 id，查 _PX4_EVENT_NAMES。"""
        marker = 'EVENT '
        idx = text.upper().find(marker)
        if idx < 0:
            return None
        tail = text[idx + len(marker):]
        digits = []
        for ch in tail:
            if ch.isdigit():
                digits.append(ch)
            elif digits:
                break
        if not digits:
            return None
        event_id = int(''.join(digits))
        name = _PX4_EVENT_NAMES.get(event_id)
        if name is None:
            return None
        return f'{text} → {name}'

    def _on_esc(self, msg):
        """记录电调最大转速，供 _rpm_scale 硬封顶 300 r/min。"""
        rpms = []
        for item in getattr(msg, 'esc_telemetry', []) or []:
            r = int(getattr(item, 'rpm', 0) or 0)
            if r > 0:
                rpms.append(r)
        for item in getattr(msg, 'esc_status', []) or []:
            r = int(getattr(item, 'rpm', 0) or 0)
            if r > 0:
                rpms.append(r)
        raw = getattr(msg, 'rpm', None)
        if raw:
            try:
                rpms.extend(int(r) for r in raw if r)
            except TypeError:
                pass
        if rpms:
            self._esc_max_rpm = max(rpms)

    def _rpm_scale(self):
        """遥测超速时整体缩小速度指令；无遥测则返回 1。"""
        if self._esc_max_rpm <= MAX_MOTOR_RPM:
            return 1.0
        self.get_logger().warn(
            f'电调 {self._esc_max_rpm} r/min 超过上限 {MAX_MOTOR_RPM}，回收油门',
            throttle_duration_sec=2.0)
        return MAX_MOTOR_RPM / float(self._esc_max_rpm)

    def _limit_body_vel(self, body_velocity):
        """按电调转速上限整体缩放机体速度指令。"""
        scale = self._rpm_scale()
        vx, vy, vz = body_velocity
        if scale >= 1.0:
            return (vx, vy, vz)
        return (vx * scale, vy * scale, vz * scale)

    def _vel_toward_position(self, target):
        """台架：位置误差转成室内限速的机体速度，电机有加速、不超过最高转速。"""
        px, py, pz = self.pose[:3]
        dx, dy, dz = target[0] - px, target[1] - py, target[2] - pz
        qx, qy, qz, qw = self.pose[3:]
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        bx = math.cos(yaw) * dx + math.sin(yaw) * dy
        by = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if (bx * bx + by * by + dz * dz) < 0.01 * 0.01:
            return (0.0, 0.0, 0.0)
        return (
            max(-XY_VEL_MAX, min(XY_VEL_MAX, bx)),
            max(-XY_VEL_MAX, min(XY_VEL_MAX, by)),
            max(-Z_VEL_MAX, min(Z_VEL_MAX, dz)))

    def _position_sp(self, target):
        """构造本地 ENU 位置设定点（姿态取 hold_orientation）。"""
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = target
        if self.hold_orientation is not None:
            (sp.pose.orientation.x, sp.pose.orientation.y,
             sp.pose.orientation.z, sp.pose.orientation.w) = self.hold_orientation
        return sp

    def _velocity_sp(self, body_velocity):
        """把机体 FLU 速度按当前偏航转换到 MAVROS 本地 ENU。"""
        vx, vy, vz = self._limit_body_vel(body_velocity)
        qx, qy, qz, qw = self.pose[3:]
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x = math.cos(yaw) * vx - math.sin(yaw) * vy
        msg.twist.linear.y = math.sin(yaw) * vx + math.cos(yaw) * vy
        msg.twist.linear.z = vz
        return msg

    def _attitude_sp(self, thrust):
        """姿态+油门设定点：OFFBOARD 预检不要求本地点/速度。"""
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.type_mask = 7  # ignore roll/pitch/yaw rates
        if self.pose is not None:
            msg.orientation.x = self.pose[3]
            msg.orientation.y = self.pose[4]
            msg.orientation.z = self.pose[5]
            msg.orientation.w = self.pose[6]
        else:
            msg.orientation.w = 1.0
        msg.thrust = float(max(0.0, min(1.0, thrust)))
        return msg

    def _publish_rc_keepalive(self):
        """注入中位摇杆，满足「有遥控输入」预检（不依赖真实遥控器）。

        mavros send_cb 把 float 原样写入 MANUAL_CONTROL（单位约 -1000..1000），
        不会再乘 1000；z=500 为油门中位。另发 RC override 作双保险。
        """
        if self.manual_pub is not None:
            msg = ManualControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 500.0
            msg.r = 0.0
            msg.buttons = 0
            self.manual_pub.publish(msg)
        if self.rc_override_pub is not None:
            ov = OverrideRCIn()
            ch = [1500] * 8 + [OverrideRCIn.CHAN_RELEASE] * 10
            ov.channels = ch
            self.rc_override_pub.publish(ov)

    def _on_gp_origin(self, msg):
        if not self._gp_origin_ok:
            self.get_logger().info('EKF gp_origin 已确认')
        self._gp_origin_ok = True

    def _on_home_position(self, msg):
        """飞控确认 home 已设置。"""
        if not self._px4_home_ok:
            self.get_logger().info('飞控 home 已确认 (/mavros/home_position/home)')
        self._px4_home_ok = True

    def _ensure_px4_home(self):
        """设 EKF 全局原点 + home，消 79408 / 无 global position。"""
        if self.pose is None:
            return
        now = self.get_clock().now()
        # 1) 经 mavros 话题设 GPS 原点（比 COMMAND_LONG 48 更稳）
        if self.gp_origin_pub is not None and not self._gp_origin_ok:
            gp = GeoPointStamped()
            gp.header.stamp = now.to_msg()
            gp.header.frame_id = 'map'
            gp.position.latitude = 47.397742
            gp.position.longitude = 8.545594
            gp.position.altitude = 488.0
            self.gp_origin_pub.publish(gp)
        # 2) COMMAND_LONG 备份
        if not self._origin_requested and self.cmd_cli.service_is_ready():
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 48
            req.confirmation = 0
            req.param5 = 47.397742
            req.param6 = 8.545594
            req.param7 = 488.0
            self._origin_requested = True
            self.cmd_cli.call_async(req)
            self.get_logger().info('已请求 SET_GPS_GLOBAL_ORIGIN（室内假原点）')
        if self.home_set_pub is not None:
            hp = HomePosition()
            hp.header.stamp = now.to_msg()
            hp.header.frame_id = 'map'
            hp.position.x = float(self.pose[0])
            hp.position.y = float(self.pose[1])
            hp.position.z = float(self.pose[2])
            hp.orientation.w = 1.0
            hp.geo.latitude = 47.397742
            hp.geo.longitude = 8.545594
            hp.geo.altitude = 488.0
            self.home_set_pub.publish(hp)
        if self._px4_home_ok:
            return
        if (self._home_cmd_t0 is not None
                and (now - self._home_cmd_t0).nanoseconds < 2_000_000_000):
            return
        self._home_cmd_t0 = now
        # 优先 /mavros/cmd/set_home
        if self.home_cli.service_is_ready():
            req = CommandHome.Request()
            req.current_gps = False
            req.yaw = 0.0
            req.latitude = 47.397742
            req.longitude = 8.545594
            req.altitude = 488.0
            fut = self.home_cli.call_async(req)

            def _done(f):
                try:
                    r = f.result()
                    if r.success:
                        self._px4_home_ok = True
                        self.get_logger().info('cmd/set_home 成功')
                    else:
                        self.get_logger().warn(
                            f'cmd/set_home 被拒 result={r.result}',
                            throttle_duration_sec=5.0)
                except Exception as exc:
                    self.get_logger().warn(f'cmd/set_home 异常: {exc}')

            fut.add_done_callback(_done)
            return
        if self.cmd_cli.service_is_ready():
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 179
            req.confirmation = 0
            req.param1 = 0.0
            req.param5 = 47.397742
            req.param6 = 8.545594
            req.param7 = 488.0
            self._px4_home_requested = True
            fut = self.cmd_cli.call_async(req)
            fut.add_done_callback(self._home_result)

    def _home_result(self, future):
        try:
            res = future.result()
            if res.success:
                self._px4_home_ok = True
                self.get_logger().info('DO_SET_HOME 成功')
            else:
                self.get_logger().warn(
                    f'DO_SET_HOME 被拒 result={res.result}',
                    throttle_duration_sec=5.0)
        except Exception as exc:
            self.get_logger().warn(f'DO_SET_HOME 调用失败: {exc}')

    def _task_is_fresh(self):
        """任务输入是否在 0.5 s 内更新过；过期则改悬停。"""
        if self.task_time is None:
            return False
        return (self.get_clock().now() - self.task_time).nanoseconds < 500_000_000

    def _vel_nonzero(self, body_velocity):
        """台架：速度幅值是否超过 ``BENCH_VEL_EPS``（否则当悬停）。"""
        vx, vy, vz = body_velocity
        return abs(vx) + abs(vy) + abs(vz) >= BENCH_VEL_EPS

    def _begin_bench_takeoff(self):
        """台架：解锁且 OFFBOARD 后启动一次时间斜坡起飞（不看气压高度）。"""
        if self._arm_t0 is not None:
            return
        self._arm_t0 = time.monotonic()
        self.get_logger().warn(
            f'台架起飞：约 {RAMP_SECONDS:.1f}s 拉升转速，不依赖气压计高度')

    def _bench_elapsed(self):
        """台架起飞斜坡已过秒数；未开始则 0。"""
        if self._arm_t0 is None:
            return 0.0
        return time.monotonic() - self._arm_t0

    def _bench_takeoff_done(self):
        """斜坡是否已满 ``RAMP_SECONDS``（视为「已起飞」）。"""
        return self._arm_t0 is not None and self._bench_elapsed() >= RAMP_SECONDS

    def _bench_takeoff_vel(self):
        """爬升速度从约 45% 拉到起飞速度，电机转速明显升高。"""
        frac = min(1.0, self._bench_elapsed() / max(RAMP_SECONDS, 1e-3))
        vz = TKO_SPEED * (0.45 + 0.55 * frac)
        return (0.0, 0.0, float(vz))

    def _active_bench_vel(self):
        """有明显速度/未到点的位置任务才回速度，否则 None → 位置悬停保转速。"""
        if not self._task_is_fresh():
            return None
        if self.task_kind == 'velocity' and self.task_velocity_body is not None:
            if self._vel_nonzero(self.task_velocity_body):
                return self.task_velocity_body
            return None
        if self.task_kind == 'position' and self.task_position is not None:
            body = self._vel_toward_position(self.task_position)
            if self._vel_nonzero(body):
                return body
            self.hold_target = self.task_position
            return None
        return None

    def _request_mode(self, mode):
        """切 PX4 模式；同一时刻只挂一个请求，且至少间隔 1 s。"""
        if self.mode_request_pending or not self.mode_cli.service_is_ready():
            return
        now = self.get_clock().now()
        if (self.last_mode_request is not None
                and (now - self.last_mode_request).nanoseconds < 1_000_000_000):
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_request_pending = True
        self.last_mode_request = now
        self.get_logger().info(f'请求切 {mode}')
        future = self.mode_cli.call_async(req)
        future.add_done_callback(lambda done: self._mode_result(done, mode))

    def _mode_result(self, future, mode):
        """切模式异步回调：失败时打 error。"""
        self.mode_request_pending = False
        try:
            if not future.result().mode_sent:
                self.get_logger().error(f'PX4 拒绝 {mode}')
        except Exception as exc:
            self.get_logger().error(f'{mode} 服务调用失败: {exc}')

    def _request_disarm(self):
        """上锁停转。台架用强制上锁(21196)；实飞走 /mavros/cmd/arming。"""
        if self._disarm_request_pending:
            return
        now = self.get_clock().now()
        if (self.last_disarm_request is not None
                and (now - self.last_disarm_request).nanoseconds < 1_000_000_000):
            return
        if self.bench:
            if not self.cmd_cli.service_is_ready():
                return
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
            req.confirmation = 0
            req.param1 = 0.0   # 0=上锁
            req.param2 = 21196.0
            self._disarm_request_pending = True
            self.last_disarm_request = now
            self.get_logger().warn('降落完成，正在强制上锁停转')
            future = self.cmd_cli.call_async(req)
            future.add_done_callback(self._disarm_long_result)
            return
        if not self.arm_cli.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = False
        self._disarm_request_pending = True
        self.last_disarm_request = now
        self.get_logger().warn('降落完成，正在请求上锁停转')
        future = self.arm_cli.call_async(req)
        future.add_done_callback(self._disarm_result)

    def _disarm_long_result(self, future):
        """台架强制上锁（CommandLong 400）异步回调。"""
        self._disarm_request_pending = False
        try:
            res = future.result()
            if res.success:
                self.get_logger().warn('已上锁，电机应已停转')
            else:
                self.get_logger().error(
                    f'强制上锁失败 result={res.result}，请手动上锁或断电')
        except Exception as exc:
            self.get_logger().error(f'强制上锁服务调用失败: {exc}')

    def _disarm_result(self, future):
        """实飞普通上锁（CommandBool）异步回调。"""
        self._disarm_request_pending = False
        try:
            if future.result().success:
                self.get_logger().warn('已上锁，电机应已停转')
            else:
                self.get_logger().error('上锁被拒绝，请手动上锁')
        except Exception as exc:
            self.get_logger().error(f'上锁服务调用失败: {exc}')

    def _handle_landing(self):
        """降落收尾：台架在 OFFBOARD 内下降后强制上锁；实飞切 AUTO.LAND。"""
        now = self.get_clock().now()
        if self._land_t0 is None:
            self._land_t0 = now
            self.get_logger().warn(
                '降落收尾开始：下降后上锁停转'
                if self.bench else '降落收尾开始：切换 AUTO.LAND')
        if not self.state.armed:
            if not self._logged_land_done:
                self.get_logger().warn('已上锁，降落完成，电机应已停转')
                self._logged_land_done = True
            return
        elapsed = (now - self._land_t0).nanoseconds * 1e-9
        if self.bench:
            # 室内 AUTO.LAND + 微油门几乎判不了「落地」；主动下降再强制上锁。
            if self.state.mode != 'OFFBOARD':
                self._request_mode('OFFBOARD')
            if elapsed < RAMP_SECONDS:
                self.velocity_pub.publish(
                    self._velocity_sp((0.0, 0.0, -float(LAND_SPEED))))
            elif elapsed < RAMP_SECONDS + 1.5:
                if self.attitude_pub is not None:
                    self.attitude_pub.publish(self._attitude_sp(THR_MIN))
                else:
                    self.velocity_pub.publish(
                        self._velocity_sp((0.0, 0.0, 0.0)))
            else:
                self._request_disarm()
            self.setpoint_ticks += 1
            return
        if self.state.mode != 'AUTO.LAND':
            self.position_pub.publish(self._position_sp(self.pose[:3]))
            self._request_mode('AUTO.LAND')
            self.setpoint_ticks += 1
        elif elapsed > 20.0:
            self.get_logger().warn(
                'AUTO.LAND 超时仍未上锁，改为请求上锁',
                throttle_duration_sec=5.0)
            self._request_disarm()

    # ------------------------------------------------------------------ 解锁
    def _request_arm(self):
        """请求解锁。降落中拒绝；台架强制解锁(21196)；实飞普通 arming。"""
        if self.landing or self.arm_request_pending:
            return
        now = self.get_clock().now()
        if (self.last_arm_request is not None
                and (now - self.last_arm_request).nanoseconds < 1_000_000_000):
            return
        if self.bench:
            if not self.cmd_cli.service_is_ready():
                self.get_logger().warn(
                    '强制解锁服务 /mavros/cmd/command 尚未就绪',
                    throttle_duration_sec=2.0)
                return
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
            req.confirmation = 0
            req.param1 = 1.0
            req.param2 = 21196.0  # PX4 force arm，绕过无 GPS 等预检（仅拆桨台架）
            self.arm_request_pending = True
            self.last_arm_request = now
            self.get_logger().warn('正在强制解锁（21196，必须已拆桨）')
            future = self.cmd_cli.call_async(req)
            future.add_done_callback(self._arm_long_result)
            return
        if not self.arm_cli.service_is_ready():
            self.get_logger().warn(
                '解锁服务 /mavros/cmd/arming 尚未就绪',
                throttle_duration_sec=2.0)
            return
        req = CommandBool.Request()
        req.value = True
        self.arm_request_pending = True
        self.last_arm_request = now
        self.get_logger().warn('正在请求解锁')
        future = self.arm_cli.call_async(req)
        future.add_done_callback(self._arm_result)

    def _arm_long_result(self, future):
        """台架强制解锁（21196）异步回调；失败时提示看 FCU 预检原文。"""
        self.arm_request_pending = False
        try:
            res = future.result()
            if res.success:
                self.get_logger().warn('已强制解锁，电机将按室内油门慢转')
            else:
                hints = '；'.join(self._recent_fcu_fails[-4:]) or '尚无 FCU 明细'
                self.get_logger().error(
                    f'PX4 拒绝强制解锁 result={res.result} '
                    f'（1=预检未过：机载端 21196 仍会跑健康检查）。'
                    f'近期 FCU：{hints}。请确认已按安全开关，拆桨后重试')
        except Exception as exc:
            self.get_logger().error(f'强制解锁服务调用失败: {exc}')

    def _arm_result(self, future):
        """实飞普通解锁异步回调。"""
        self.arm_request_pending = False
        try:
            if not future.result().success:
                self.get_logger().error('PX4 拒绝解锁，请查看 QGC Preflight Fail')
            else:
                self.get_logger().warn('已解锁，电机将按室内油门慢转')
        except Exception as exc:
            self.get_logger().error(f'解锁服务调用失败: {exc}')

    def _enqueue_bench_params(self):
        """组装台架参数队列：先解锁相关，再 MPC 油门/速度（可稍后失败跳过）。

        int → INTEGER，float → DOUBLE。机载端 21196 仍会跑预检，故必须先写：
        无 GPS、关磁、放宽 IMU 一致性、外部视觉；勿写 COM_RC_IN_MODE=4；
        勿把 COM_ARM_IMU_* 写成 0（阈值 0=任何不一致都拒解锁）。
        CBRK_IO_SAFETY 运行时写入后仍须按安全开关（或保存参数后重启飞控）。
        """
        arm_params = [
            # 最先写：MAVLink 摇杆（假遥控）；新固件 1=MAVLink only；勿用 4
            ('COM_RC_IN_MODE', 1),
            ('COM_ARM_WO_GPS', 1),
            ('CBRK_IO_SAFETY', 22027),
            ('COM_PREARM_MODE', 2),
            ('COM_RCL_EXCEPT', 4),
            # 台架常见拒解锁：无任务 / 无 GCS / 多 IMU 投票；尽量关掉硬依赖
            ('COM_ARM_MIS_REQ', 0),
            ('NAV_DLL_ACT', 0),
            ('COM_DLL_EXCEPT', 7),  # 忽略部分模式的数传丢失（含 offboard bit，视固件）
            ('SYS_HAS_GPS', 1),  # 允许假原点/global home；仍靠 EKF2_EV 视觉
            ('SYS_HAS_MAG', 0),
            ('EKF2_MAG_TYPE', 5),
            ('EKF2_MAG_CHECK', 0),
            ('COM_ARM_CHK_ESCS', 0),
            ('COM_ARM_MAG_STR', 0),
            ('COM_ARM_MAG_ANG', -1),
            # 阈值越大越松；写成 0 会让任何 IMU 不一致都拒解锁（NavModes::All）
            ('COM_ARM_IMU_ACC', 1.0),
            ('COM_ARM_IMU_GYR', 0.3),
            # 注意：不要把 CAL_ACC*_ID / CAL_GYRO*_ID 写成 0 来「屏蔽副 IMU」。
            # 槽位清零后飞控认为该 IMU 未校准，健康检查直接把 accel 判 Fail，
            # 整机进 ARMING_STATE_STANDBY_ERROR，连 21196 强制解锁也会被拒。
            ('EKF2_ABL_LIM', 2.0),
            # 15=位置+速度+yaw：台架视觉发固定航向，避免 Heading 不稳
            ('EKF2_EV_CTRL', 15),
            ('EKF2_EV_DELAY', 5.0),
            ('EKF2_HGT_REF', 3),
            ('CBRK_USB_CHK', 197848),
            ('COM_ARM_AUTH_REQ', 0),
            # 拆桨怠速达不到「已起飞」判定，默认 10s 会自动上锁；负数关闭起飞前超时。
            # 落地后仍要自动上锁停转：COM_DISARM_LAND 保持正数（秒）。
            ('COM_DISARM_PRFLT', -1.0),
            ('COM_DISARM_LAND', 2.0),
        ]
        rest = [
            ('MPC_THR_MIN', float(THR_MIN)),
            ('MPC_THR_HOVER', float(HOVER_THRUST)),
            ('MPC_THR_MAX', float(THR_MAX)),
            ('MPC_ACC_HOR', float(ACC_HOR)),
            ('MPC_ACC_HOR_MAX', float(ACC_HOR)),
            ('MPC_ACC_UP_MAX', float(ACC_UP)),
            ('MPC_ACC_DOWN_MAX', float(ACC_DOWN)),
            ('MPC_XY_VEL_MAX', float(XY_VEL_MAX)),
            ('MPC_Z_VEL_MAX_UP', float(Z_VEL_MAX)),
            ('MPC_Z_VEL_MAX_DN', float(Z_VEL_MAX)),
            ('MPC_TKO_SPEED', float(TKO_SPEED)),
            ('MPC_JERK_AUTO', float(JERK_AUTO)),
            ('MPC_LAND_SPEED', float(LAND_SPEED)),
        ]
        self._param_queue = arm_params + rest
        self._arm_param_names = {name for name, _v in arm_params}
        self.get_logger().info('台架参数队列已就绪：先写解锁参数，再写油门')

    def _make_param_request(self, name, value):
        """按 ParamSetV2 / ParamSet 组装写参请求（int→INTEGER，float→DOUBLE）。"""
        req = ParamSetSrv.Request()
        req.param_id = name
        if _PARAM_SET_V2:
            # MAVROS 2.x：/mavros/param/set 实际类型是 ParamSetV2。
            # force_set：UART 上参数表可能还没拉完，也要把解锁参数送出去。
            req.force_set = True
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = bool(value)
            elif isinstance(value, int):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = int(value)
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(value)
            req.value = pv
            return req
        if isinstance(value, int) and value != 0:
            req.value = ParamValue(integer=int(value), real=0.0)
        else:
            req.value = ParamValue(integer=0, real=float(value))
        return req

    def _mark_arm_params_if_done(self):
        """队列中已无解锁相关参数时，标记可进入解锁等待窗。"""
        pending = {item[0] for item in self._param_queue}
        if not (pending & self._arm_param_names):
            if not self._arm_params_ready:
                self._arm_params_ready = True
                self._arm_ready_since = self.get_clock().now()
                self._recent_fcu_fails = []
                self.get_logger().warn(
                    '解锁相关参数已处理完，等待 EKF 采用新偏置上限后再请求解锁')

    def _finish_params(self, reason):
        """结束写参流程并清空队列状态。"""
        self.params_done = True
        self._arm_params_ready = True
        if self._arm_ready_since is None:
            self._arm_ready_since = self.get_clock().now()
        self.param_pending = False
        self._param_queue = []
        self._param_sent_name = None
        self._param_sent_time = None
        self.get_logger().warn(reason)

    def _skip_current_param(self, reason):
        """跳过当前挂起参数（拒绝/超时），继续后续项。"""
        name = self._param_sent_name
        self.param_pending = False
        self._param_sent_name = None
        self._param_sent_time = None
        if self._param_queue and (name is None or self._param_queue[0][0] == name):
            skipped = self._param_queue.pop(0)[0]
            self.get_logger().error(f'{reason}: {skipped}')
        self._mark_arm_params_if_done()
        if not self._param_queue:
            self._finish_params('台架参数已写完，开始解锁')

    def _pump_params(self):
        """串行写下一个台架参数；UART 忙时一次只挂一个请求。"""
        if self.params_done or self.param_pending:
            return
        if not self.state.connected:
            return
        if not self.param_cli.service_is_ready():
            try:
                self.param_cli.wait_for_service(timeout_sec=0.2)
            except Exception:
                pass
        if not self.param_cli.service_is_ready():
            kind = 'ParamSetV2' if _PARAM_SET_V2 else 'ParamSet'
            self.get_logger().warn(
                f'参数服务 /mavros/param/set ({kind}) 尚未就绪',
                throttle_duration_sec=2.0)
            return
        if not self._param_queue:
            self._enqueue_bench_params()
        if not self._param_queue:
            self._finish_params('台架参数队列为空，继续解锁流程')
            return
        name, value = self._param_queue[0]
        req = self._make_param_request(name, value)
        self.param_pending = True
        self._param_sent_name = name
        self._param_sent_time = self.get_clock().now()
        self.get_logger().info(f'正在写参数 {name}')
        future = self.param_cli.call_async(req)
        future.add_done_callback(
            lambda done, n=name: self._param_result(done, n))

    def _param_result(self, future, name):
        """写参异步回调：成功则出队，失败则跳过该项。"""
        if self._param_sent_name != name:
            return
        ok = False
        try:
            ok = bool(future.result().success)
        except Exception as exc:
            self.get_logger().error(f'写参数 {name} 失败: {exc}')
        if ok:
            self.get_logger().info(f'已写参数 {name}')
            self.param_pending = False
            self._param_sent_name = None
            self._param_sent_time = None
            if self._param_queue and self._param_queue[0][0] == name:
                self._param_queue.pop(0)
            self._mark_arm_params_if_done()
            if not self._param_queue:
                self._finish_params('台架参数已写完，开始解锁')
        else:
            self._skip_current_param(f'飞控拒绝参数 {name}，跳过继续')

    def _tick(self):
        """20 Hz 主循环：发布 airborne、设定点，再推进参数/解锁（降落中跳过解锁）。"""
        if self.bench and not self.state.armed:
            self._arm_t0 = None
            self._logged_hover = False
            self._hover_captured = False
            self.airborne = False
        elif self.bench and self.state.armed and self.state.mode == 'OFFBOARD':
            self._begin_bench_takeoff()
            self.airborne = self._bench_takeoff_done()
        elif self.bench:
            self.airborne = False
        elif self.pose is not None and self.home is not None:
            self.airborne = (
                self.pose[2] >= self.home[2] + self.altitude * 0.95)
        airborne_msg = Bool()
        airborne_msg.data = self.airborne
        self.airborne_pub.publish(airborne_msg)

        if self.pose is not None and self.hold_target is not None:
            if self.landing:
                self._handle_landing()
            elif self.bench:
                # 解锁前：姿态设定点满足 OFFBOARD 信号；假遥控满足摇杆预检。
                # 不要依赖 COM_RC_IN_MODE=4（会把「无遥控」变成硬失败）。
                offboard = self.state.mode == 'OFFBOARD'
                if offboard:
                    self._offboard_ticks += 1
                else:
                    self._offboard_ticks = 0
                if not self.state.armed or not offboard:
                    self._fix_mavros_thrust_scaling()
                    self._ensure_px4_home()
                    self._publish_rc_keepalive()
                    # mavros setpoint_raw 在 thrust_scaling=NaN 时会丢弃非零 thrust，
                    # 导致飞控报 No offboard signal。预热一律用速度设定点。
                    self.velocity_pub.publish(
                        self._velocity_sp((0.0, 0.0, 0.0)))
                elif not self.airborne:
                    self.velocity_pub.publish(
                        self._velocity_sp(self._bench_takeoff_vel()))
                else:
                    if not self._hover_captured and self.pose is not None:
                        self.hold_target = self.pose[:3]
                        self._hover_captured = True
                    body = self._active_bench_vel()
                    if body is not None:
                        self.velocity_pub.publish(self._velocity_sp(body))
                    else:
                        self.velocity_pub.publish(
                            self._velocity_sp((0.0, 0.0, 0.0)))
                    if not self._logged_hover:
                        self.get_logger().info(
                            '台架悬停：保持转速，有速度/位置任务时再加速，'
                            f'最高 {MAX_MOTOR_RPM} r/min')
                        self._logged_hover = True
                self.setpoint_ticks += 1
            elif self.airborne and self._task_is_fresh():
                if self.task_kind == 'velocity':
                    self.velocity_pub.publish(
                        self._velocity_sp(self.task_velocity_body))
                    if not self._logged_ignore_vel:
                        self.get_logger().info('已起飞，开始转发避障速度')
                        self._logged_ignore_vel = True
                else:
                    self.position_pub.publish(
                        self._position_sp(self.task_position))
                self.setpoint_ticks += 1
            else:
                if (self.task_kind == 'velocity' and not self.airborne
                        and self._task_is_fresh()):
                    self.get_logger().info(
                        '已收到避障速度，等待起飞后再转发给飞控',
                        throttle_duration_sec=5.0)
                if (self.airborne and self.task_kind is not None
                        and not self._task_is_fresh()):
                    self.hold_target = self.pose[:3]
                    self.task_kind = None
                    self.get_logger().warn('任务设定点超时，切换到当前位置悬停')
                self.position_pub.publish(self._position_sp(self.hold_target))
                self.setpoint_ticks += 1
        else:
            self.setpoint_ticks = 0
            if self.landing:
                self._handle_landing()

        if self.landing:
            return

        if not self.state.connected:
            self.connected_ticks = 0
            return
        self.connected_ticks += 1
        if self.bench and not self.params_done:
            if (self.param_pending and self._param_sent_time is not None
                    and (self.get_clock().now()
                         - self._param_sent_time).nanoseconds > 2_500_000_000):
                self._skip_current_param('写参数超时（UART 无应答），跳过')
            if self.connected_ticks >= 20:
                self._pump_params()
            if (not self._arm_params_ready
                    and self.connected_ticks >= 400):
                self._arm_params_ready = True
                self._arm_ready_since = self.get_clock().now()
                self.get_logger().error(
                    '解锁参数未能及时写完，仍尝试解锁。看 FCU: 预检原文；'
                    '室内常见 Heading/Accel Bias——已写 EKF2_MAG_TYPE=5、'
                    'EKF2_ABL_LIM=2.0；若仍拒绝请按安全开关后重试')
        if not self.arm_allowed:
            return
        if self.pose is None or self.setpoint_ticks < 40:
            self.get_logger().info(
                '等待位姿与设定点预热后再解锁',
                throttle_duration_sec=5.0)
            return
        if self.bench and not self._arm_params_ready:
            self.get_logger().info(
                '等待写入无GPS/磁罗盘关闭/加速度计偏置参数后再解锁',
                throttle_duration_sec=5.0)
            return
        if (self.bench and self._arm_ready_since is not None
                and (self.get_clock().now()
                     - self._arm_ready_since).nanoseconds < 10_000_000_000):
            self.get_logger().info(
                '解锁参数已写入，等待 EKF 刷新偏航/home/原点（约 10s）',
                throttle_duration_sec=2.0)
            return
        # 台架：姿态设定点 + 假遥控 → OFFBOARD 稳定后再强制解锁。
        if not self.state.armed:
            if self.state.mode != 'OFFBOARD':
                self._request_mode('OFFBOARD')
                return
            if self.bench and self._offboard_ticks < 40:
                self.get_logger().info(
                    '已进入 OFFBOARD，等待设定点被飞控接受后再解锁',
                    throttle_duration_sec=2.0)
                return
            # home 未确认也继续尝试解锁（同时 _ensure_px4_home 每 2s 重试）
            self._request_arm()
            return
        if self.state.mode != 'OFFBOARD':
            self._request_mode('OFFBOARD')

    def _status_tick(self):
        """约 2 Hz 打印飞控模式/解锁/高度/参数与任务状态摘要。"""
        pose_z = None if self.pose is None else self.pose[2]
        z_rel = None if (self.home is None or self.pose is None) else (
            self.pose[2] - self.home[2])
        pending = self._param_sent_name or '-'
        self.get_logger().info(
            f'FCU mode={self.state.mode or "-"} armed={self.state.armed} '
            f'connected={self.state.connected} airborne={self.airborne} '
            f'z={pose_z} z_rel={z_rel} params_done={self.params_done} '
            f'param_pending={pending} sp_ticks={self.setpoint_ticks} '
            f'esc_rpm={self._esc_max_rpm} task={self.task_kind} '
            f'bench_takeoff={self._arm_t0 is not None}')


def main(args=None):
    """解析 --arm/--bench/--altitude；退出时若仍解锁则尽量请求上锁。"""
    parser = argparse.ArgumentParser(description='PX4 OFFBOARD 起飞与设定点管理器')
    parser.add_argument(
        '--altitude', type=float, default=TAKEOFF_ALT_M,
        help=f'起飞高度（米），室内台架默认 {TAKEOFF_ALT_M}（实飞 2 m 的 1/20）')
    parser.add_argument('--arm', action='store_true', help='允许切 OFFBOARD 并解锁')
    parser.add_argument('--no-arm', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--bench', action='store_true',
                        help='室内台架：写外部视觉参数并降低悬停油门')
    parser.add_argument('--no-bench', action='store_true', help=argparse.SUPPRESS)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = OffboardManager(
        parsed.altitude, parsed.arm and not parsed.no_arm,
        parsed.bench and not parsed.no_bench)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if getattr(node, 'state', None) is not None and node.state.armed:
                node.get_logger().warn('进程退出：请求上锁停转')
                node.landing = True
                node._request_disarm()
                # 给异步上锁一点时间；run.sh 退出陷阱还会经 UART 再上锁一次
                end = time.time() + 1.5
                while time.time() < end and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if not node.state.armed:
                        break
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
