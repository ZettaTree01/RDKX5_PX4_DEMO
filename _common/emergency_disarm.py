#!/usr/bin/env python3
"""例程退出或 Ctrl+C 后，经串口强制上锁，防止电机继续转动。

``ros2 launch`` 被中断时，节点几乎同时退出，MAVROS 往往先于管理器退出，
ROS 服务来不及完成上锁，飞控可能仍处于 armed，电调会持续转动。

本脚本不依赖 ROS：直接打开 ``/dev/ttyS2``，发送
``MAV_CMD_COMPONENT_ARM_DISARM``（400，param1=0 上锁，param2=21196）。
由 ``run_flight.sh`` 在退出时调用，也可手动执行。

调用前应确保占用串口的 mavros 已释放（``run.sh`` 中已预留短暂等待）。
"""
from __future__ import annotations

import argparse
import sys
import time

import serial
from pymavlink.dialects.v20 import common as mavlink

DEFAULT_PORT = '/dev/ttyS2'
DEFAULT_BAUD = 57600
CMD_ARM_DISARM = 400
FORCE = 21196.0  # PX4 强制解锁/上锁用的魔法数


def _log(quiet: bool, msg: str) -> None:
    if not quiet:
        print(msg, flush=True)


def disarm(port: str, baud: int, quiet: bool = False, retries: int = 8) -> bool:
    """打开串口，等飞控心跳；若还解锁就连发强制上锁，再读心跳确认。

    返回 True 表示已经上锁（或本来就没解锁）；False 表示重试完还是不行。
    """
    mav = mavlink.MAVLink(None, srcSystem=255, srcComponent=1)
    mav.robust_parsing = True
    last_err = None
    retries = max(1, int(retries))
    for attempt in range(1, retries + 1):
        ser = None
        try:
            # 打开失败别卡死：短超时，便于 EXIT 陷阱尽快返回
            ser = serial.Serial(port, baud, timeout=0.2, write_timeout=0.5)
            sysid = compid = None
            armed = None
            # 等飞控心跳（跳过地面站 / 伴飞自己的 HEARTBEAT）
            t0 = time.time()
            while time.time() - t0 < 1.5:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                for ch in chunk:
                    msg = mav.parse_char(bytes([ch]))
                    if msg is None:
                        continue
                    if msg.get_msgId() != mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                        continue
                    if msg.type in (
                            mavlink.MAV_TYPE_GCS,
                            mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
                        continue
                    sysid = msg.get_srcSystem()
                    compid = msg.get_srcComponent()
                    armed = bool(
                        msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    break
                if sysid is not None:
                    break
            if sysid is None:
                last_err = '无飞控心跳'
                ser.close()
                time.sleep(0.4)
                continue
            if armed is False:
                _log(quiet, f'飞控已上锁（{port}），无需处理')
                ser.close()
                return True
            _log(quiet, f'检测到仍解锁，经 {port} 强制上锁…')
            for _ in range(5):
                pkt = mav.command_long_encode(
                    sysid, compid, CMD_ARM_DISARM, 0,
                    0.0, FORCE, 0, 0, 0, 0, 0)
                ser.write(pkt.pack(mav))
                time.sleep(0.15)
            t1 = time.time()
            while time.time() - t1 < 1.2:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                for ch in chunk:
                    msg = mav.parse_char(bytes([ch]))
                    if msg is None or msg.get_msgId() != mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                        continue
                    if msg.get_srcSystem() != sysid:
                        continue
                    if not (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        _log(quiet, '已上锁，电机应已停转')
                        ser.close()
                        return True
            last_err = '已发上锁指令，但心跳仍显示解锁'
            ser.close()
            time.sleep(0.3)
        except Exception as exc:
            # 多半是串口还被 mavros 占着，歇一会儿再试
            last_err = str(exc)
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            time.sleep(0.5)
            _log(quiet, f'上锁重试 {attempt}/{retries}: {last_err}')
    _log(quiet, f'紧急上锁失败: {last_err}')
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description='例程退出紧急上锁')
    ap.add_argument('--port', default=DEFAULT_PORT,
                    help='飞控串口，默认 40PIN UART2 /dev/ttyS2')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--quiet', action='store_true', help='少打日志')
    ap.add_argument('--retries', type=int, default=8,
                    help='串口忙/无心跳时的重试次数（EXIT 陷阱建议 3）')
    args = ap.parse_args()
    ok = disarm(args.port, args.baud, quiet=args.quiet, retries=args.retries)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
