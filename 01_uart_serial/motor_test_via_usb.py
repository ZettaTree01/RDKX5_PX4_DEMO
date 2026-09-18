#!/usr/bin/env python3
"""
机载计算机通过 40PIN 针脚串口让 PX4 飞控执行**电机测试**（拆桨后检查转向/怠速）。

链路:  RDK X5 40PIN UART2(/dev/ttyS2@57600) --MAVLink--> PX4 FMU
接线:  PIN20=GND | PIN22=RX(接飞控主板TX) | PIN15=TX(接飞控主板RX)
方式:  经 MAVLink shell(SERIAL_CONTROL) 执行 PX4 自带的 `actuator_test`。

  为什么不用 MAV_CMD_DO_MOTOR_TEST(209)？
  PX4 返回 UNSUPPORTED；MAV_CMD_ACTUATOR_TEST(310) 无响应。
  PX4 只暴露了 shell 里的 `actuator_test`：
      actuator_test set [-m <1..8>] [-s <1..8>] [-f <func>] -v <-1..1> [-t <秒>]
      actuator_test iterate-motors / iterate-servos

============================== 安全须知 ==============================
本脚本会让电机实际旋转，务必确认：
  1. 桨叶已全部拆除
  2. 飞机固定牢靠，周围无人、无遮挡物
  3. 供电充足（电池或台架电源）
默认必须显式加 --i-am-sure 才会执行。
命令带 -t 参数，超时后飞控自动停转；Ctrl+C 也会立刻发停止指令。
======================================================================

关于转速上限 600 r/min:
  PX4 的 actuator_test 只接受 **-1~1 的输出值**，没有直接设定 r/min 的接口。
  本脚本默认输出值已按室内上限封顶；若电调支持 DShot 遥测，实际 rpm 超过
  600 时会自动把输出值压下去。

用法:
  # 只探测 actuator_test 是否可用，不转电机
  python3 motor_test_via_usb.py --probe

  # 全部电机同步启动：怠速斜坡加速到 600 r/min 上限，约 6 秒
  python3 motor_test_via_usb.py --duration 6 --i-am-sure

  # 只转 1 号电机
  python3 motor_test_via_usb.py --motor 1 --duration 3 --i-am-sure

  # 依次点动全部电机（飞控自带流程，逐个启停）
  python3 motor_test_via_usb.py --iterate --i-am-sure

依赖: pyserial, pymavlink
"""
import argparse
import os
import sys
import time

import serial
from pymavlink.dialects.v20 import common as mavlink

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '_common'))
from indoor import MAX_MOTOR_RPM, MOTOR_TEST_VALUE, THR_MAX, THR_MIN

# ---- MAVLink ----
MSG_HEARTBEAT = mavlink.MAVLINK_MSG_ID_HEARTBEAT
MSG_STATUSTEXT = mavlink.MAVLINK_MSG_ID_STATUSTEXT
MSG_SERIAL_CONTROL = mavlink.MAVLINK_MSG_ID_SERIAL_CONTROL
MSG_ESC_STATUS = getattr(mavlink, 'MAVLINK_MSG_ID_ESC_STATUS', 290)
MSG_SERVO_OUTPUT_RAW = mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW  # 36
CMD_SET_MESSAGE_INTERVAL = 511
SHELL_DEV = 10
FLAG_RESPOND = 2
FLAG_EXCLUSIVE = 4
FLAG_BLOCKING = 8

MAX_SAFE_VALUE = THR_MAX    # 室内最高 600 r/min，不允许再加输出
MAX_SAFE_DURATION = 30.0


class Px4Link:
    """MAVLink 串口链路：心跳 + shell + 消息收集。"""

    def __init__(self, port, baud):
        """打开串口并创建 GCS 侧 MAVLink 解析器（sys=255）。"""
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.mav = mavlink.MAVLink(None, srcSystem=255, srcComponent=1)
        self.mav.robust_parsing = True
        self.sysid = self.compid = None
        self.armed = False
        self._scratch = []

    def close(self):
        """关闭串口。"""
        try:
            self.ser.close()
        except Exception:
            pass

    def _pump(self, seconds, ids):
        """读取若干秒串口数据，返回指定 msgid 的消息列表；顺带更新心跳状态。"""
        out, t0 = [], time.time()
        while time.time() - t0 < seconds:
            b = self.ser.read(self.ser.in_waiting or 1)
            if not b:
                continue
            for ch in b:
                m = self.mav.parse_char(bytes([ch]))
                if m is None:
                    continue
                if m.get_msgId() == MSG_HEARTBEAT:
                    self.sysid = m.get_srcSystem()
                    self.compid = m.get_srcComponent()
                    self.armed = bool(m.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not ids or m.get_msgId() in ids:
                    out.append(m)
        return out

    def wait_heartbeat(self, timeout=15.0):
        """阻塞等待首个 HEARTBEAT；超时返回 None。"""
        for m in self._pump(timeout, {MSG_HEARTBEAT}):
            return m
        return None

    def request_msg(self, msgid, hz):
        """请求某消息按 hz 下发；hz=0 恢复默认。"""
        interval = int(1_000_000 / hz) if hz > 0 else 0
        self.ser.write(self.mav.command_long_encode(
            self.sysid, self.compid, CMD_SET_MESSAGE_INTERVAL, 0,
            msgid, float(interval), 0, 0, 0, 0, 0).pack(self.mav))

    def servo_line(self, m):
        """把 SERVO_OUTPUT_RAW 八通道格式化成一行 PWM 文本。"""
        vals = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw,
                m.servo5_raw, m.servo6_raw, m.servo7_raw, m.servo8_raw]
        return ' '.join('%4d' % v if v else '   0' for v in vals)

    # ---- shell ----
    def shell_open(self):
        """打开 MAVLink shell（SERIAL_CONTROL），进入可交互状态。"""
        self._shell_send(b'', FLAG_RESPOND | FLAG_EXCLUSIVE | FLAG_BLOCKING)
        time.sleep(1.2)
        self._pump(1.0, {MSG_SERIAL_CONTROL})
        self._shell_send(b'\n')
        time.sleep(0.8)
        self._pump(1.0, {MSG_SERIAL_CONTROL})

    def _shell_send(self, data=b'', flags=FLAG_RESPOND):
        """经 SERIAL_CONTROL 向飞控 shell 设备写入最多 70 字节。"""
        d = bytes(data) + b'\0' * (70 - len(data))
        self.ser.write(self.mav.serial_control_encode(
            SHELL_DEV, flags, 0, 0, len(data), d).pack(self.mav))

    def shell_run(self, cmd, wait=2.5, read=3.5):
        """执行一条 shell 命令并返回输出文本。"""
        self._shell_send(cmd.encode() + b'\n')
        time.sleep(wait)
        return b''.join(bytes(m.data[:m.count]) for m in
                        self._pump(read, {MSG_SERIAL_CONTROL})).decode('utf-8', 'replace')

    def shell_close(self):
        """关闭 shell 会话（flags=0）。"""
        self._shell_send(b'', 0)

    def shell_write(self, cmd):
        """只下发 shell 命令、不等待回显（用于连续启动多个电机，尽量同步）。"""
        self._shell_send(cmd.encode() + b'\n')

    def stop_actuators(self, motors=None):
        """把电机输出归零。motors 为空时按 1~8 全部停一遍。"""
        targets = motors if motors else list(range(1, 9))
        try:
            for m in targets:
                self._shell_send(b'actuator_test set -m %d -v 0 -t 0\n' % m)
                time.sleep(0.05)
            self._pump(0.6, None)
        except Exception:
            pass


def main():
    """解析参数、安全校验、经 shell 执行 actuator_test，并监控 PWM/ESC。"""
    ap = argparse.ArgumentParser(
        description='PX4 电机测试（拆桨后使用）',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyS2', help='串口，本教程统一 40PIN UART2 /dev/ttyS2')
    ap.add_argument('--baud', type=int, default=57600, help='波特率，须与飞控 TELEM 端口一致(57600)')
    ap.add_argument('--motor', type=int, default=0,
                    help='电机编号 1~8；0 表示全部电机同步启动（默认）')
    ap.add_argument('--all', action='store_true', help='全部电机同步启动（等价 --motor 0）')
    ap.add_argument('--num-motors', type=int, default=0,
                    help='电机数量，0=从 SERVO_OUTPUT_RAW 自动识别')
    ap.add_argument('--value', type=float, default=MOTOR_TEST_VALUE,
                    help='输出值 -1~1（不是 rpm），默认 %.4f，且不超过 600 r/min 油门上限 %.3f'
                    % (MOTOR_TEST_VALUE, THR_MAX))
    ap.add_argument('--duration', type=float, default=6.0,
                    help='运行时长(秒)，默认 6（从怠速斜坡加速到上限）')
    ap.add_argument('--iterate', action='store_true',
                    help='用飞控自带的 iterate-motors 依次点动全部电机')
    ap.add_argument('--probe', action='store_true', help='只探测 actuator_test 是否可用')
    ap.add_argument('--i-am-sure', action='store_true',
                    help='确认已拆桨、飞机固定、周围安全（真正转动电机必须带）')
    ap.add_argument('--allow-high-value', action='store_true',
                    help='允许超过 %.2f 的输出值' % MAX_SAFE_VALUE)
    args = ap.parse_args()

    if args.probe:
        args.value, args.duration = 0.0, 0.0

    if not args.probe:
        if not args.i_am_sure:
            print('拒绝执行: 真正转动电机必须显式加 --i-am-sure')
            print('（请先拆桨、固定飞机、确认周围安全；只想探测可用性请加 --probe）')
            return 2
        if abs(args.value) > MAX_SAFE_VALUE and not args.allow_high_value:
            print('拒绝执行: 输出值 %.3f 超过安全上限 %.2f' % (args.value, MAX_SAFE_VALUE))
            print('如确有必要，额外加 --allow-high-value')
            return 2
        if args.duration > MAX_SAFE_DURATION:
            print('拒绝执行: 时长 %.1fs 超过上限 %.0fs' % (args.duration, MAX_SAFE_DURATION))
            return 2
        if args.iterate:
            print('注意: --iterate 由飞控自行依次点动全部电机')

    try:
        link = Px4Link(args.port, args.baud)
    except serial.SerialException as e:
        print('打开 %s 失败: %s' % (args.port, e))
        return 1

    print('已打开 %s，等待飞控心跳 ...' % args.port)
    if link.wait_heartbeat() is None:
        print('未收到心跳')
        link.close()
        return 1
    print('心跳 OK: sysid=%d compid=%d  解锁状态: %s'
          % (link.sysid, link.compid, '已解锁' if link.armed else '未解锁'))
    if link.armed and not args.probe:
        print('拒绝执行: 飞控已解锁，电机测试应在未解锁下进行')
        link.close()
        return 2

    # 订阅 SERVO_OUTPUT_RAW，用来证明电机输出端确实有变化
    link.request_msg(MSG_SERVO_OUTPUT_RAW, 10)
    link.request_msg(MSG_ESC_STATUS, 10)
    time.sleep(0.6)
    base = link._pump(1.5, {MSG_SERVO_OUTPUT_RAW})
    if base:
        print('静止时 PWM 输出: %s' % link.servo_line(base[-1]))
    else:
        print('(未收到 SERVO_OUTPUT_RAW)')

    # 依据 SERVO_OUTPUT_RAW 中非零通道数自动识别电机数量
    motors = []
    if not args.probe and not args.iterate:
        n = args.num_motors
        if n <= 0:
            if base:
                vals = [base[-1].servo1_raw, base[-1].servo2_raw,
                        base[-1].servo3_raw, base[-1].servo4_raw,
                        base[-1].servo5_raw, base[-1].servo6_raw,
                        base[-1].servo7_raw, base[-1].servo8_raw]
                n = sum(1 for v in vals if v and v > 0)
            else:
                n = 4
        n = max(1, min(8, n))
        motors = list(range(1, n + 1)) if (args.all or args.motor == 0) else [args.motor]
        print('目标电机: %s%s' % (' '.join(str(m) for m in motors),
                                   '（同步启动）' if len(motors) > 1 else ''))

    link.shell_open()

    if args.probe:
        out = link.shell_run('actuator_test', wait=1.5, read=2.5)
        print('--- actuator_test 可用性 ---')
        print(out.strip() if out.strip() else '(无输出，命令可能不存在)')
        link.request_msg(MSG_SERVO_OUTPUT_RAW, 0)
        link.shell_close()
        link.close()
        return 0 if 'set' in out else 1

    if args.iterate:
        cmds = ['actuator_test iterate-motors']
        print('=' * 62)
        print('连续下发 %d 条命令:' % len(cmds))
        for c in cmds:
            print('  %s' % c)
        print('确认已拆桨、飞机固定、周围安全。Ctrl+C 可立即停止。')
        print('=' * 62)
        watch = args.duration + 12.0
    else:
        peak = min(THR_MAX, abs(args.value))
        start = min(THR_MIN, peak)
        print('=' * 62)
        print('室内斜坡加速：怠速 %.3f → 最高 %.3f，时长 %.1fs，上限 %d r/min'
              % (start, peak, args.duration, MAX_MOTOR_RPM))
        print('确认已拆桨、飞机固定、周围安全。Ctrl+C 可立即停止。')
        print('=' * 62)
        watch = args.duration + 0.8
    try:
        if args.iterate:
            for c in cmds:
                link.shell_write(c)
                time.sleep(0.02)
            time.sleep(1.0)
            for m in link._pump(1.5, {MSG_SERIAL_CONTROL}):
                t = bytes(m.data[:m.count]).decode('utf-8', 'replace').strip()
                if t and 'actuator_test' in t:
                    print('[飞控] %s' % t.replace('\r', ' ')[:200])
            t0 = time.time()
            last_cut = 0.0
            last_send = 0.0
            held = False
            current_v = args.value
        else:
            current_v = start
            remain = args.duration + 0.5
            for mid in motors:
                link.shell_write(
                    'actuator_test set -m %d -v %.4f -t %.1f'
                    % (mid, start, remain))
                time.sleep(0.02)
            t0 = time.time()
            last_cut = 0.0
            last_send = 0.0
            held = False

        while time.time() - t0 < watch:
            t = time.time() - t0
            if not args.iterate and not held:
                frac = min(1.0, t / max(args.duration, 0.1))
                current_v = start + (peak - start) * frac
                if t - last_send >= 0.25:
                    last_send = t
                    remain = max(0.3, args.duration - t + 0.4)
                    for mid in motors:
                        link.shell_write(
                            'actuator_test set -m %d -v %.4f -t %.1f'
                            % (mid, current_v, remain))
            msgs = link._pump(0.2, {MSG_ESC_STATUS, MSG_STATUSTEXT,
                                       MSG_SERIAL_CONTROL, MSG_SERVO_OUTPUT_RAW})
            for m in msgs:
                if m.get_msgId() == MSG_SERVO_OUTPUT_RAW:
                    print('\r  输出 %.3f  PWM: %s   (%.1fs)      '
                          % (current_v, link.servo_line(m),
                             watch - (time.time() - t0)),
                          end='', flush=True)
                elif m.get_msgId() == MSG_ESC_STATUS:
                    try:
                        rpms = [r for r in m.rpm[:8] if r > 0]
                    except Exception:
                        rpms = []
                    if rpms:
                        peak_rpm = max(rpms)
                        print('\r  输出 %.3f  ESC: %s rpm   (%.1fs)      '
                              % (current_v, '  '.join('%d' % r for r in rpms),
                                 watch - (time.time() - t0)),
                              end='', flush=True)
                        if (peak_rpm > MAX_MOTOR_RPM and motors
                                and time.time() - last_cut > 0.4):
                            last_cut = time.time()
                            held = True
                            current_v = max(
                                THR_MIN,
                                min(THR_MAX,
                                    current_v * MAX_MOTOR_RPM / float(peak_rpm)))
                            remain = max(0.2, watch - (time.time() - t0))
                            print('\n  已到 %d r/min 上限，维持输出 %.4f'
                                  % (MAX_MOTOR_RPM, current_v))
                            for mid in motors:
                                link.shell_write(
                                    'actuator_test set -m %d -v %.4f -t %.1f'
                                    % (mid, current_v, remain))
                elif m.get_msgId() == MSG_STATUSTEXT:
                    txt = bytes(m.text).split(b'\0')[0].decode('utf-8', 'replace')
                    print('\n  [飞控] %s' % txt)
                elif m.get_msgId() == MSG_SERIAL_CONTROL:
                    ttxt = bytes(m.data[:m.count]).decode('utf-8', 'replace').strip()
                    if ttxt:
                        print('\n  [shell] %s' % ttxt[:200])
    except KeyboardInterrupt:
        print('\n收到中断，立即停止电机 ...')
    except serial.SerialException as e:
        print('\n串口异常: %s' % e)
    finally:
        link.stop_actuators(motors if motors else None)
        link.request_msg(MSG_SERVO_OUTPUT_RAW, 0)  # 恢复默认速率
        link.request_msg(MSG_ESC_STATUS, 0)
        link.shell_close()
        link.close()

    print('\n' + '=' * 62)
    print('已发送停止指令（-t 超时也会自动停转），串口已关闭。')
    print('未看到 ESC 转速 = 该电调不支持 DShot 遥测，只能靠油门上限 %.3f 限制转速。'
          % THR_MAX)
    print('室内最高转速限制 %d r/min。' % MAX_MOTOR_RPM)
    return 0


if __name__ == '__main__':
    sys.exit(main())
