#!/usr/bin/env python3
"""
pymavlink でドローンをアーミングし、数秒後にディスアームするスクリプト。

PX4 では MANUAL_CONTROL メッセージでスロットル位置を認識させる。
スレッドを使わず、メインループで MANUAL_CONTROL 送信と受信を交互に行う。
"""

import argparse
import sys
import time

from pymavlink import mavutil

MAV_AUTOPILOT_PX4 = 12

SEND_INTERVAL = 0.05  # MANUAL_CONTROL 送信間隔（秒）

# PX4 カスタムモード（main_mode << 16）
PX4_CUSTOM_MAIN_MODE = {
    "MANUAL":     1 << 16,
    "ALTCTL":     2 << 16,
    "POSCTL":     3 << 16,
    "AUTO":       4 << 16,
    "ACRO":       5 << 16,
    "OFFBOARD":   6 << 16,
    "STABILIZED": 7 << 16,
    "RATTITUDE":  8 << 16,
}

MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    7: "COMMAND_LONG_ONLY",
    8: "COMMAND_INT_ONLY",
}


def send_manual_control(master, z: int = 0) -> None:
    """MANUAL_CONTROL を 1 回送信する。z=0 がスロットルゼロ（推力なし）。"""
    master.mav.manual_control_send(
        master.target_system,
        0,    # x (pitch)
        0,    # y (roll)
        z,    # z (throttle): 0=推力なし, 1000=最大
        0,    # r (yaw)
        0,    # buttons
    )


def send_manual_control_loop(master, duration: float, z: int = 0) -> int:
    """指定時間、MANUAL_CONTROL を送り続ける。送信回数を返す。"""
    count = 0
    t0 = time.monotonic()
    while (time.monotonic() - t0) < duration:
        send_manual_control(master, z=z)
        count += 1
        time.sleep(SEND_INTERVAL)
    return count


def arm_with_polling(master, force: bool = False, throttle_z: int = 0,
                     timeout: float = 10.0) -> bool:
    """
    ARM コマンドを送信し、MANUAL_CONTROL を送り続けながら
    COMMAND_ACK と STATUSTEXT をポーリングする。
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,                          # param1: 1=arm
        21196 if force else 0,      # param2: 21196=force
        0, 0, 0, 0, 0,
    )

    deadline = time.monotonic() + timeout
    last_send = 0.0
    ack_result = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        if (now - last_send) >= SEND_INTERVAL:
            send_manual_control(master, z=throttle_z)
            last_send = now

        msg = master.recv_match(type=["COMMAND_ACK", "STATUSTEXT"],
                                blocking=False)
        if msg is None:
            time.sleep(0.01)
            continue

        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            print(f"  STATUSTEXT: [sev={msg.severity}] {msg.text}")
        elif mtype == "COMMAND_ACK":
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                result_name = MAV_RESULT_NAMES.get(msg.result,
                                                   f"UNKNOWN({msg.result})")
                print(f"  COMMAND_ACK: result={result_name}")
                if msg.result == 0:
                    return True
                ack_result = msg.result
                # ACK 失敗後、STATUSTEXT を拾うためにもう少し待つ
                deadline = min(deadline, time.monotonic() + 2.0)

    if ack_result is not None:
        return False
    print("  COMMAND_ACK タイムアウト")
    return False


def disarm_with_polling(master, force: bool = False, throttle_z: int = 0,
                        timeout: float = 5.0) -> bool:
    """DISARM コマンドを送信し、MANUAL_CONTROL を送り続けながら ACK を待つ。"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,                          # param1: 0=disarm
        21196 if force else 0,
        0, 0, 0, 0, 0,
    )
    deadline = time.monotonic() + timeout
    last_send = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if (now - last_send) >= SEND_INTERVAL:
            send_manual_control(master, z=throttle_z)
            last_send = now
        msg = master.recv_match(type="COMMAND_ACK", blocking=False)
        if msg is None:
            time.sleep(0.01)
            continue
        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            result_name = MAV_RESULT_NAMES.get(msg.result,
                                               f"UNKNOWN({msg.result})")
            print(f"  COMMAND_ACK: result={result_name}")
            return msg.result == 0
    print("  COMMAND_ACK タイムアウト")
    return False


def wait_for_px4(master, timeout: float = 30.0):
    """PX4 オートパイロットのハートビートを待つ。"""
    print("PX4 オートパイロットのハートビート待機...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = master.recv_match(type="HEARTBEAT", blocking=True,
                                timeout=min(remaining, 5.0))
        if msg is None:
            continue
        src_sys = msg.get_srcSystem()
        src_comp = msg.get_srcComponent()
        autopilot = msg.autopilot
        mav_type = msg.type
        print(f"  HEARTBEAT: system={src_sys} component={src_comp} "
              f"autopilot={autopilot} type={mav_type}")
        if autopilot == MAV_AUTOPILOT_PX4 and src_sys != 0:
            master.target_system = src_sys
            master.target_component = src_comp
            print(f"PX4 検出: system={src_sys} component={src_comp}")
            return True
        print("  → PX4 ではないのでスキップ...")
    return False


def set_mode(master, mode_name: str, throttle_z: int = 0) -> bool:
    """PX4 の飛行モードを設定する。MANUAL_CONTROL を送りながら ACK を待つ。"""
    custom_mode = PX4_CUSTOM_MAIN_MODE.get(mode_name.upper())
    if custom_mode is None:
        print(f"  不明なモード: {mode_name}")
        print(f"  有効なモード: {', '.join(PX4_CUSTOM_MAIN_MODE.keys())}")
        return False

    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    master.mav.set_mode_send(
        master.target_system,
        base_mode,
        custom_mode,
    )

    deadline = time.monotonic() + 5.0
    last_send = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if (now - last_send) >= SEND_INTERVAL:
            send_manual_control(master, z=throttle_z)
            last_send = now
        msg = master.recv_match(type="COMMAND_ACK", blocking=False)
        if msg is None:
            time.sleep(0.01)
            continue
        result_name = MAV_RESULT_NAMES.get(msg.result, f"UNKNOWN({msg.result})")
        print(f"  COMMAND_ACK: result={result_name}")
        return msg.result == 0
    print("  COMMAND_ACK タイムアウト")
    return False


def request_data_streams(master):
    """PX4 に全データストリームの送信をリクエストする。"""
    print("データストリームをリクエスト...")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4,  # 4 Hz
        1,  # start
    )


SENSOR_FLAGS = {
    0x01: "3D_GYRO",
    0x02: "3D_ACCEL",
    0x04: "3D_MAG",
    0x08: "ABS_PRESSURE",
    0x10: "DIFF_PRESSURE",
    0x20: "GPS",
    0x40: "OPTICAL_FLOW",
    0x80: "VISION_POSITION",
    0x100: "LASER_POSITION",
    0x200: "EXTERNAL_GROUND_TRUTH",
    0x400: "ANGULAR_RATE_CTRL",
    0x800: "ATTITUDE_STAB",
    0x1000: "YAW_POSITION",
    0x2000: "Z_ALT_CTRL",
    0x4000: "XY_POS_CTRL",
    0x8000: "MOTOR_OUTPUTS",
    0x10000: "RC_RECEIVER",
    0x20000: "3D_GYRO2",
    0x40000: "3D_ACCEL2",
    0x80000: "3D_MAG2",
    0x100000: "GEOFENCE",
    0x200000: "AHRS",
    0x400000: "TERRAIN",
    0x800000: "REVERSE_MOTOR",
    0x1000000: "LOGGING",
    0x2000000: "BATTERY",
    0x4000000: "PROXIMITY",
    0x8000000: "SATCOM",
    0x10000000: "PREARM_CHECK",
}


def decode_sensor_bits(value: int) -> list[str]:
    """センサービットマスクを名前のリストに変換する。"""
    names = []
    for bit, name in SENSOR_FLAGS.items():
        if value & bit:
            names.append(name)
    return names


def dump_all_messages(master, duration: float = 5.0):
    """指定時間、受信した全メッセージのタイプを表示する（診断用）。"""
    print(f"全メッセージを {duration} 秒間モニタ...")
    seen = {}
    sys_status_shown = False
    t0 = time.monotonic()
    while (time.monotonic() - t0) < duration:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mtype = msg.get_type()
        seen[mtype] = seen.get(mtype, 0) + 1
        if mtype == "STATUSTEXT":
            print(f"  STATUSTEXT: [sev={msg.severity}] {msg.text}")
        elif mtype == "SYS_STATUS" and not sys_status_shown:
            sys_status_shown = True
            present = msg.onboard_control_sensors_present
            enabled = msg.onboard_control_sensors_enabled
            health = msg.onboard_control_sensors_health

            print(f"\n  === SYS_STATUS センサー詳細 ===")
            print(f"  present = {present:#010x}")
            print(f"  enabled = {enabled:#010x}")
            print(f"  health  = {health:#010x}")

            unhealthy = enabled & ~health
            if unhealthy:
                print(f"\n  有効だが異常なセンサー (enabled & ~health):")
                for name in decode_sensor_bits(unhealthy):
                    print(f"    ✗ {name}")
            else:
                print(f"\n  有効なセンサーは全て正常")

            not_present = enabled & ~present
            if not_present:
                print(f"  有効だが未検出のセンサー:")
                for name in decode_sensor_bits(not_present):
                    print(f"    ? {name}")
            print()

    print("受信メッセージ種別:")
    for mtype, count in sorted(seen.items(), key=lambda x: -x[1]):
        print(f"  {mtype}: {count}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="MANUAL_CONTROL(throttle=0)→アーミング→待機→ディスアーム"
    )
    parser.add_argument(
        "--connection", default="udp:0.0.0.0:14550",
        help="MAVLink 接続（デフォルト: 0.0.0.0:14550 で待ち受け）",
    )
    parser.add_argument(
        "--wait", type=float, default=5.0, metavar="SEC",
        help="アーム後の待機時間（秒）",
    )
    parser.add_argument(
        "--throttle-prep", type=float, default=2.0, metavar="SEC",
        help="アーム前に MANUAL_CONTROL を送る準備時間（秒）",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="pre-arm チェックをバイパスして強制アーム（param2=21196）",
    )
    parser.add_argument(
        "--retry", type=int, default=3, metavar="N",
        help="アーム失敗時のリトライ回数",
    )
    parser.add_argument(
        "--throttle-z", type=int, default=0, metavar="VAL",
        help="MANUAL_CONTROL の z 値（0=推力なし [デフォルト], -1000=最小）",
    )
    parser.add_argument(
        "--diagnose", action="store_true",
        help="アーム試行前に全受信メッセージを表示して終了（デバッグ用）",
    )
    args = parser.parse_args()

    print(f"接続中: {args.connection}")
    master = mavutil.mavlink_connection(args.connection)

    if not wait_for_px4(master):
        print("PX4 が見つかりませんでした", file=sys.stderr)
        return 1

    request_data_streams(master)
    time.sleep(0.5)

    if args.diagnose:
        dump_all_messages(master, duration=8.0)
        return 0

    try:
        # 1. MANUAL_CONTROL でスロットルゼロを送り続ける
        z = args.throttle_z
        print(f"MANUAL_CONTROL (z={z}) を {args.throttle_prep} 秒間送信...")
        count = send_manual_control_loop(master, args.throttle_prep, z=z)
        print(f"  {count} 回送信完了")

        # 2. スタビライズモードに切替
        print("STABILIZEDモードに設定...")
        if not set_mode(master, "STABILIZED", throttle_z=z):
            print("モード設定失敗。続行を試みます...")

        # 3. アーミング（MANUAL_CONTROL を送り続けながら ACK を待つ）
        armed = False
        for attempt in range(1, args.retry + 1):
            print(f"アーミング... (試行 {attempt}/{args.retry})")
            if arm_with_polling(master, force=args.force, throttle_z=z):
                armed = True
                break
            print("  1秒後にリトライ...")
            send_manual_control_loop(master, 1.0, z=z)

        if not armed:
            print("アーミング失敗")
            return 1

        print("アーム完了")

        # 3. 待機（MANUAL_CONTROL を送り続ける）
        print(f"{args.wait} 秒待機...")
        send_manual_control_loop(master, args.wait, z=z)

    except KeyboardInterrupt:
        print("\n中断検出")

    finally:
        print("ディスアーム...")
        disarm_with_polling(master, force=args.force, throttle_z=z)
        print("完了")

    return 0


if __name__ == "__main__":
    sys.exit(main())
