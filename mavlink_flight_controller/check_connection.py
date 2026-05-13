#!/usr/bin/env python3
"""
pymavlink でドローンへの接続チェックのみ行うスクリプト。

受信した全ハートビートを表示し、PX4 オートパイロット（autopilot=12）を
見つけたら接続情報を表示して終了する。
DroneBridge 等の中継機器（system=0）は表示するがスキップする。
"""

import argparse
import sys
import time

from pymavlink import mavutil

MAV_AUTOPILOT_PX4 = 12


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MAVLink 接続の確認（全ハートビートを表示し、PX4 を探す）"
    )
    parser.add_argument(
        "--connection",
        default="udp:0.0.0.0:14550",
        help="MAVLink 接続（デフォルト: 0.0.0.0:14550 で待ち受け）",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        metavar="SEC",
        help="タイムアウト（秒）。0 で無制限",
    )
    args = parser.parse_args()

    print(f"接続中: {args.connection}")
    try:
        master = mavutil.mavlink_connection(args.connection)
    except Exception as e:
        print(f"接続失敗: {e}", file=sys.stderr)
        return 1

    deadline = (time.monotonic() + args.timeout) if args.timeout > 0 else None
    print("ハートビート待機...")

    found_px4 = False
    while True:
        remaining = None
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
        if msg is None:
            break

        src_sys = msg.get_srcSystem()
        src_comp = msg.get_srcComponent()
        autopilot = msg.autopilot
        mav_type = msg.type
        base_mode = msg.base_mode

        label = ""
        if src_sys == 0:
            label = " [中継機器?]"
        if autopilot == MAV_AUTOPILOT_PX4:
            label = " [PX4]"

        print(f"  system={src_sys} component={src_comp} "
              f"autopilot={autopilot} type={mav_type} "
              f"base_mode={base_mode}{label}")

        if autopilot == MAV_AUTOPILOT_PX4 and src_sys != 0 and not found_px4:
            found_px4 = True
            master.target_system = src_sys
            master.target_component = src_comp

            armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print()
            print(f"=== PX4 検出 ===")
            print(f"  system_id    = {src_sys}")
            print(f"  component_id = {src_comp}")
            print(f"  mav_type     = {mav_type}")
            print(f"  armed        = {armed}")
            break

    if not found_px4:
        print()
        print("PX4 オートパイロットが見つかりませんでした。")
        print("  → DroneBridge の WiFi に接続しているか確認してください")
        print("  → FC が PX4 でない場合は autopilot ID を確認してください")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
