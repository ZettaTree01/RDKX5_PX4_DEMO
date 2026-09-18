#!/usr/bin/env python3
"""向 mavros GCS 桥发送 MAVLink GCS HEARTBEAT + MANUAL_CONTROL。

配合 launch 里 gcs_url:=udp://0.0.0.0:14550@：本脚本连 127.0.0.1:14550，
经 mavros 转发到飞控。

- HEARTBEAT：消「No GCS datalink」(6801787)
- MANUAL_CONTROL：消「No manual control input」(453929)，配合 COM_RC_IN_MODE=1
"""
import argparse
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=14550)
    ap.add_argument('--rate', type=float, default=1.0,
                    help='GCS HEARTBEAT 频率 Hz')
    ap.add_argument('--manual-rate', type=float, default=10.0,
                    help='MANUAL_CONTROL 频率 Hz（摇杆保活）')
    ap.add_argument('--sysid', type=int, default=255)
    ap.add_argument('--target-system', type=int, default=1)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(
        f'udpout:{args.host}:{args.port}',
        source_system=args.sysid,
        source_component=190,
    )
    hb_period = 1.0 / max(args.rate, 0.1)
    man_period = 1.0 / max(args.manual_rate, 1.0)
    next_hb = time.monotonic()
    next_man = time.monotonic()
    print(
        f'[gcs_heartbeat] udpout {args.host}:{args.port} '
        f'hb={args.rate}Hz manual={args.manual_rate}Hz',
        flush=True,
    )
    while True:
        now = time.monotonic()
        if now >= next_hb:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            next_hb = now + hb_period
        if now >= next_man:
            # x/y/r: -1000..1000；z 油门 0..1000，500=中位
            conn.mav.manual_control_send(
                args.target_system,
                0, 0, 500, 0, 0,
            )
            next_man = now + man_period
        time.sleep(0.02)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
