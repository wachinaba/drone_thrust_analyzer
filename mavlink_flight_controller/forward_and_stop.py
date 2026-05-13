#!/usr/bin/env python3
"""
pymavlink でドローンを「アーム → 前進 → 後進で停止 → ディスアーム」するスクリプト。

MANUAL_CONTROL メッセージで制御する。
  y: roll (前進/後進)   — 正=前進, 負=後進  (-1000 ~ 1000)
  z: throttle           — 0=推力なし, 1000=最大  (-1000 ~ 1000)

安全のため:
  - 各フェーズの値と秒数をコマンドライン引数で調整可能
  - KeyboardInterrupt で即座にスロットルゼロ → ディスアーム
  - 後進フェーズの後にホバリング停止フェーズを挟む
"""

import argparse
import sys
import time

from pymavlink import mavutil

from log_downloader import download_latest_log

# ---------------------------------------------------------------------------
# 定数
# ---------------------------------------------------------------------------
MAV_AUTOPILOT_PX4 = 12
SEND_INTERVAL = 0.05

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


# ---------------------------------------------------------------------------
# 送受信ユーティリティ
# ---------------------------------------------------------------------------
def send_manual_control(master, x: int = 0, y: int = 0,
                        z: int = 0, r: int = 0) -> None:
    """MANUAL_CONTROL を 1 回送信する。"""
    master.mav.manual_control_send(
        master.target_system,
        x,    # pitch  (前進+/後進-)
        y,    # roll   (右+/左-)
        z,    # throttle
        r,    # yaw    (反時計+/時計-)
        0,    # buttons
    )


def clamp(val: int, lo: int = -1000, hi: int = 1000) -> int:
    return max(lo, min(hi, val))


def send_control_loop(master, duration: float,
                      x: int = 0, y: int = 0,
                      z: int = 0, r: int = 0,
                      trim: tuple[int, int, int, int] = (0, 0, 0, 0)) -> int:
    """指定時間、MANUAL_CONTROL を送り続ける。trim=(tx,ty,tz,tr) を加算する。"""
    tx, ty, tz, tr = trim
    count = 0
    t0 = time.monotonic()
    while (time.monotonic() - t0) < duration:
        send_manual_control(master,
                            x=clamp(x + tx), y=clamp(y + ty),
                            z=clamp(z + tz), r=clamp(r + tr))
        count += 1
        time.sleep(SEND_INTERVAL)
    return count


# ---------------------------------------------------------------------------
# 接続・モード・アーム
# ---------------------------------------------------------------------------
def wait_for_px4(master, timeout: float = 30.0) -> bool:
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


def request_data_streams(master):
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
    )


def set_mode(master, mode_name: str, z: int = 0) -> bool:
    custom_mode = PX4_CUSTOM_MAIN_MODE.get(mode_name.upper())
    if custom_mode is None:
        return False
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
    )
    deadline = time.monotonic() + 5.0
    last_send = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if (now - last_send) >= SEND_INTERVAL:
            send_manual_control(master, z=z)
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


def arm(master, force: bool = False, z: int = 0,
        timeout: float = 10.0) -> bool:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196 if force else 0, 0, 0, 0, 0, 0,
    )
    deadline = time.monotonic() + timeout
    last_send = 0.0
    ack_result = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if (now - last_send) >= SEND_INTERVAL:
            send_manual_control(master, z=z)
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
                deadline = min(deadline, time.monotonic() + 2.0)
    if ack_result is not None:
        return False
    print("  COMMAND_ACK タイムアウト")
    return False


def disarm(master, force: bool = False, z: int = 0,
           timeout: float = 5.0) -> bool:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196 if force else 0, 0, 0, 0, 0, 0,
    )
    deadline = time.monotonic() + timeout
    last_send = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if (now - last_send) >= SEND_INTERVAL:
            send_manual_control(master, z=z)
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


def reboot_vehicle(master, timeout: float = 5.0) -> bool:
    """MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN でオートパイロットを再起動する。"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,  # param1: 1 = reboot autopilot (MAV_REBOOT_SHUTDOWN_ACTION)
        0, 0, 0, 0, 0, 0,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=False)
        if msg is None:
            time.sleep(0.02)
            continue
        if msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            result_name = MAV_RESULT_NAMES.get(msg.result,
                                              f"UNKNOWN({msg.result})")
            print(f"  COMMAND_ACK (REBOOT): result={result_name}")
            return msg.result == 0
    print("  COMMAND_ACK タイムアウト（再起動は送信済みの可能性あり）")
    return False


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="アーム → 前進 → 後進停止 → ディスアーム",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--connection", default="udp:0.0.0.0:14550",
                        help="MAVLink 接続先")
    parser.add_argument("--force", action="store_true",
                        help="pre-arm チェックをバイパス")
    parser.add_argument("--retry", type=int, default=3,
                        help="アームのリトライ回数")

    g = parser.add_argument_group("飛行パラメータ（MANUAL_CONTROL -1000~1000）")
    g.add_argument("--throttle", type=int, default=100, metavar="Z",
                   help="飛行中のスロットル z 値")
    g.add_argument("--forward-x", type=int, default=0, metavar="X",
                   help="前進フェーズの x 値")
    g.add_argument("--forward-y", type=int, default=50, metavar="Y",
                   help="前進フェーズの y 値")
    g.add_argument("--reverse-x", type=int, default=0, metavar="X",
                   help="後進フェーズの x 値")
    g.add_argument("--reverse-y", type=int, default=-50, metavar="Y",
                   help="後進フェーズの y 値")

    g = parser.add_argument_group("トリム（アーム中の全フェーズに加算されるオフセット）")
    g.add_argument("--trim-x", type=int, default=0, metavar="X",
                   help="x 軸トリム")
    g.add_argument("--trim-y", type=int, default=0, metavar="Y",
                   help="y 軸トリム")
    g.add_argument("--trim-z", type=int, default=0, metavar="Z",
                   help="z 軸（スロットル）トリム")
    g.add_argument("--trim-r", type=int, default=0, metavar="R",
                   help="r 軸（ヨー）トリム")

    g = parser.add_argument_group("ログダウンロード")
    g.add_argument("--download-log", action=argparse.BooleanOptionalAction,
                   default=True,
                   help="ディスアーム後にフライトログをダウンロード")
    g.add_argument("--log-dir", default="./logs",
                   help="ログ保存先ディレクトリ")

    g = parser.add_argument_group("終了時")
    g.add_argument("--reboot", action=argparse.BooleanOptionalAction,
                   default=False,
                   help="ログ取得後・終了前にフライトコントローラーを再起動")

    g = parser.add_argument_group("タイミング（秒）")
    g.add_argument("--prep-time", type=float, default=2.0,
                   help="アーム前の MANUAL_CONTROL 送信時間")
    g.add_argument("--hover-time", type=float, default=2.0,
                   help="アーム直後のホバリング安定化時間")
    g.add_argument("--forward-time", type=float, default=3.0,
                   help="前進フェーズの時間")
    g.add_argument("--reverse-time", type=float, default=3.0,
                   help="後進（停止）フェーズの時間")
    g.add_argument("--settle-time", type=float, default=2.0,
                   help="後進後のホバリング安定化時間")
    args = parser.parse_args()

    throttle = args.throttle
    fwd_x = args.forward_x
    fwd_y = args.forward_y
    rev_x = args.reverse_x
    rev_y = args.reverse_y
    trim = (args.trim_x, args.trim_y, args.trim_z, args.trim_r)

    print(f"接続中: {args.connection}")
    master = mavutil.mavlink_connection(args.connection)

    if not wait_for_px4(master):
        print("PX4 が見つかりませんでした", file=sys.stderr)
        return 1

    request_data_streams(master)
    time.sleep(0.5)

    print()
    print("=== 飛行パラメータ ===")
    print(f"  throttle (z) = {throttle}")
    print(f"  前進 (x,y)   = ({fwd_x}, {fwd_y})  × {args.forward_time}s")
    print(f"  後進 (x,y)   = ({rev_x}, {rev_y})  × {args.reverse_time}s")
    if any(t != 0 for t in trim):
        print(f"  トリム (x,y,z,r) = {trim}")
    print()

    try:
        # 1. 準備: スロットルゼロで MANUAL_CONTROL を送信
        print(f"[準備] MANUAL_CONTROL (z=0) を {args.prep_time}s 送信...")
        send_control_loop(master, args.prep_time, z=0)

        # 2. STABILIZED モード
        print("[モード] STABILIZED に設定...")
        if not set_mode(master, "STABILIZED", z=0):
            print("  モード設定失敗。続行を試みます...")

        # 3. アーム
        armed = False
        for attempt in range(1, args.retry + 1):
            print(f"[アーム] 試行 {attempt}/{args.retry}")
            if arm(master, force=args.force, z=0):
                armed = True
                break
            print("  1秒後にリトライ...")
            send_control_loop(master, 1.0, z=0)

        if not armed:
            print("アーミング失敗")
            return 1
        print("アーム完了")

        # 4. ホバリング安定化
        print(f"[ホバー] throttle={throttle} で {args.hover_time}s 安定化...")
        send_control_loop(master, args.hover_time, z=throttle, trim=trim)

        # 5. 前進
        print(f"[前進] x={fwd_x}, y={fwd_y}, z={throttle} で {args.forward_time}s...")
        send_control_loop(master, args.forward_time, x=fwd_x, y=fwd_y, z=throttle, trim=trim)

        # 6. 後進（停止）
        print(f"[後進] x={rev_x}, y={rev_y}, z={throttle} で {args.reverse_time}s...")
        send_control_loop(master, args.reverse_time, x=rev_x, y=rev_y, z=throttle, trim=trim)

        # 7. ホバリングで安定
        print(f"[安定] z={throttle} で {args.settle_time}s...")
        send_control_loop(master, args.settle_time, z=throttle, trim=trim)

    except KeyboardInterrupt:
        print("\n[緊急] 中断検出 → スロットルゼロ送信中...")
        send_control_loop(master, 0.5, z=0)

    finally:
        # 8. スロットルゼロにしてディスアーム
        print("[停止] スロットルゼロ...")
        send_control_loop(master, 1.0, z=0)
        print("[ディスアーム]...")
        disarm(master, force=args.force, z=0)
        print("ディスアーム完了")

        # 9. フライトログダウンロード
        if args.download_log:
            try:
                download_latest_log(master, output_dir=args.log_dir)
            except Exception as exc:
                print(f"[ログ] ダウンロード中にエラー: {exc}", file=sys.stderr)

        if args.reboot:
            print("[再起動] フライトコントローラーへ再起動コマンドを送信中...")
            try:
                reboot_vehicle(master)
            except Exception as exc:
                print(f"[再起動] エラー: {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
