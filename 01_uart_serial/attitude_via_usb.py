#!/usr/bin/env python3
"""
机载计算机(RDK X5)通过 40PIN 针脚串口从 PX4 飞控主板读取飞机姿态。

链路:  RDK X5 40PIN UART2(/dev/ttyS2@57600) --MAVLink--> PX4 FMU
接线:  PIN20=GND | PIN22=RX(接飞控主板TX) | PIN15=TX(接飞控主板RX)
       （UART2 与 X5 默认 UART1 的 PIN8/10 不同，需先在设备树中使能，
         见《RDK_X5_AI_Tutorial.md》接线/使能章节）

工作原理:
  1. 打开串口，用 MAVLink v2 解析字节流
  2. 等 HEARTBEAT 拿到飞控的 sysid/compid
  3. 用 MAV_CMD_SET_MESSAGE_INTERVAL(511) 请求 ATTITUDE(30) 按指定频率下发
  4. 循环打印 roll / pitch / yaw 与三轴角速度

安全性: 本脚本只订阅和打印，**不下发任何控制指令**，不会解锁或切换模式。
        退出时会把 ATTITUDE 的发送间隔恢复为 0（默认速率）。

用法:
  python3 /app/zettatree_demo/01_uart_serial/attitude_via_usb.py
  python3 /app/zettatree_demo/01_uart_serial/attitude_via_usb.py --rate 50 --duration 10

依赖: pyserial, pymavlink
"""
import argparse
import math
import sys
import time

import serial
from pymavlink.dialects.v20 import common as mavlink

# ---- MAVLink 消息 ID ----
MSG_HEARTBEAT = mavlink.MAVLINK_MSG_ID_HEARTBEAT        # 0
MSG_ATTITUDE = mavlink.MAVLINK_MSG_ID_ATTITUDE          # 30
MSG_STATUSTEXT = mavlink.MAVLINK_MSG_ID_STATUSTEXT      # 253
CMD_SET_MESSAGE_INTERVAL = 511

# MAV_AUTOPILOT / MAV_TYPE 常用取值，便于把数字翻译成文字
AUTOPILOT = {0: 'Generic', 3: 'ArduPilot', 4: 'OpenPilot', 12: 'PX4'}
MAV_TYPE = {0: 'Generic', 1: 'FixedWing', 2: 'Quadrotor', 4: 'Helicopter',
            6: 'Hexarotor', 7: 'Octorotor', 13: 'Hexarotor', 21: 'VTOL'}


def build_mavlink():
    """创建 MAVLink 解析器。robust_parsing 让坏帧被丢弃而不是抛异常。"""
    mav = mavlink.MAVLink(None, srcSystem=255, srcComponent=1)
    mav.robust_parsing = True
    return mav


def request_attitude(ser, mav, sysid, compid, hz):
    """请求飞控按 hz 频率下发 ATTITUDE。interval 为微秒，0 表示恢复默认。"""
    interval_us = int(1_000_000 / hz) if hz > 0 else 0
    msg = mav.command_long_encode(
        sysid, compid, CMD_SET_MESSAGE_INTERVAL, 0,
        MSG_ATTITUDE, interval_us, 0, 0, 0, 0, 0)
    ser.write(msg.pack(mav))


def fmt_signed(value, width=7, prec=2):
    """带正负号的定宽浮点格式化，便于姿态行对齐。"""
    return '%+*.*f' % (width, prec, value)


def main():
    """打开串口、等心跳、订阅 ATTITUDE 并打印；退出时恢复默认速率。"""
    ap = argparse.ArgumentParser(description='通过 40PIN 针脚串口(/dev/ttyS2)读取 PX4 飞控姿态')
    ap.add_argument('--port', default='/dev/ttyS2',
                    help='串口设备。本教程统一 40PIN UART2 /dev/ttyS2')
    ap.add_argument('--baud', type=int, default=57600,
                    help='波特率。ttyS2 须与飞控 TELEM 端口一致，本教程 57600')
    ap.add_argument('--rate', type=float, default=20.0,
                    help='请求 ATTITUDE 的频率(Hz)，默认 20')
    ap.add_argument('--duration', type=float, default=0,
                    help='运行时长(秒)，0 表示一直运行到 Ctrl+C')
    ap.add_argument('--quiet', action='store_true',
                    help='只输出汇总，不逐帧打印')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        print('打开 %s 失败: %s' % (args.port, e))
        print('检查: 飞控是否上电 / 接线(PIN20 GND, PIN22 RX←飞控TX, PIN15 TX→飞控RX)')
        print('      / UART2 是否已在设备树使能(见教程附录) / 当前用户是否在 dialout 组')
        return 1

    mav = build_mavlink()
    print('已打开 %s (波特率 %d)' % (args.port, args.baud))
    print('等待飞控心跳 ...')

    sysid = compid = None
    armed = False
    base_mode = 0
    last_att = None
    count = 0
    t_start = time.time()
    t_last_hb = time.time()
    t_first = None
    stop = False

    def on_sigint(sig, frm):
        nonlocal stop
        stop = True

    try:
        import signal
        signal.signal(signal.SIGINT, on_sigint)
    except Exception:
        pass

    try:
        while not stop:
            if args.duration and (time.time() - t_start) >= args.duration:
                break

            data = ser.read(ser.in_waiting or 1)
            if not data:
                # 长时间没心跳就提示，但不退出，方便热插拔
                if sysid is None and time.time() - t_last_hb > 10:
                    print('  仍未收到心跳，确认飞控已启动且该串口上有 MAVLink')
                    t_last_hb = time.time()
                continue

            for byte in data:
                msg = mav.parse_char(bytes([byte]))
                if msg is None:
                    continue
                mid = msg.get_msgId()

                if mid == MSG_HEARTBEAT:
                    t_last_hb = time.time()
                    if sysid is None:
                        sysid, compid = msg.get_srcSystem(), msg.get_srcComponent()
                        print('心跳 OK: sysid=%d compid=%d autopilot=%s type=%s'
                              % (sysid, compid,
                                 AUTOPILOT.get(msg.autopilot, msg.autopilot),
                                 MAV_TYPE.get(msg.type, msg.type)))
                        print('请求 ATTITUDE @ %.1f Hz ...' % args.rate)
                        request_attitude(ser, mav, sysid, compid, args.rate)
                    armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    base_mode = msg.custom_mode

                elif mid == MSG_ATTITUDE and sysid is not None:
                    count += 1
                    now = time.time()
                    if t_first is None:
                        t_first = now
                    last_att = msg
                    if not args.quiet:
                        roll = math.degrees(msg.roll)
                        pitch = math.degrees(msg.pitch)
                        yaw = math.degrees(msg.yaw)
                        hz = count / (now - t_first) if now > t_first else 0.0
                        print('\r[%6.2fs] roll %s°  pitch %s°  yaw %s°  | '
                              'p %s  q %s  r %s °/s | %5.1f Hz  '
                              % (now - t_start,
                                 fmt_signed(roll), fmt_signed(pitch), fmt_signed(yaw),
                                 fmt_signed(math.degrees(msg.rollspeed), 6),
                                 fmt_signed(math.degrees(msg.pitchspeed), 6),
                                 fmt_signed(math.degrees(msg.yawspeed), 6),
                                 hz), end='', flush=True)

                elif mid == MSG_STATUSTEXT and sysid is not None:
                    text = bytes(msg.text).split(b'\0')[0].decode('utf-8', 'replace')
                    sev = {0: 'EMERG', 1: 'ALERT', 2: 'CRIT', 3: 'ERR',
                           4: 'WARN', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'}.get(msg.severity, '?')
                    print('\n[飞控 %s] %s' % (sev, text))

    except serial.SerialException as e:
        print('\n串口异常: %s' % e)
        return 1
    except KeyboardInterrupt:
        stop = True
    finally:
        if sysid is not None:
            request_attitude(ser, mav, sysid, compid, 0)  # 恢复默认速率
        ser.close()

    elapsed = time.time() - t_start
    print('\n' + '=' * 56)
    if count == 0:
        print('未收到任何 ATTITUDE 数据（用时 %.1fs）' % elapsed)
        print('排查: 该串口上是否有 MAVLink 实例 / 波特率是否一致 / 接线是否通')
        return 1
    print('收到 ATTITUDE %d 帧，用时 %.1fs，平均 %.1f Hz' % (count, elapsed, count / max(elapsed, 1e-6)))
    if last_att is not None:
        print('最后一帧: roll %.2f°  pitch %.2f°  yaw %.2f°'
              % (math.degrees(last_att.roll), math.degrees(last_att.pitch),
                 math.degrees(last_att.yaw)))
    print('飞控状态: %s' % ('已解锁' if armed else '未解锁'))
    print('串口 %s 已关闭，ATTITUDE 速率已恢复默认' % args.port)
    return 0


if __name__ == '__main__':
    sys.exit(main())
