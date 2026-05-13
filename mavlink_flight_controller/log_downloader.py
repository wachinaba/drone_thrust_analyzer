#!/usr/bin/env python3
"""
MAVLink フライトログダウンローダー。

LOG_REQUEST_LIST / LOG_ENTRY / LOG_REQUEST_DATA / LOG_DATA プロトコルで
フライトコントローラーからログ (.ulg) をダウンロードする。

単体 CLI としても使用可能:
    python log_downloader.py --connection udp:0.0.0.0:14550 --output-dir ./logs
"""

import argparse
import os
import sys
import time
from datetime import datetime

from pymavlink import mavutil

LOG_DATA_PAYLOAD_LEN = 90
_RECV_TIMEOUT = 0.5
_RETRY_INTERVAL = 0.5


# ---------------------------------------------------------------------------
# ログ一覧
# ---------------------------------------------------------------------------
def get_latest_log_entry(master, timeout: float = 30.0):
    """ログ一覧を取得し、最新 (ID が最大) のエントリを返す。

    Returns:
        LOG_ENTRY メッセージ、またはログが無ければ None。
    """
    master.mav.log_request_list_send(
        master.target_system,
        master.target_component,
        0,       # start
        0xFFFF,  # end — 全ログ
    )

    latest = None
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = master.recv_match(
            type="LOG_ENTRY", blocking=True,
            timeout=min(_RECV_TIMEOUT, remaining),
        )
        if msg is None:
            continue

        print(f"  LOG_ENTRY: id={msg.id}  size={msg.size}  "
              f"num_logs={msg.num_logs}  last_log_num={msg.last_log_num}")

        if latest is None or msg.id > latest.id:
            latest = msg

        if msg.id == msg.last_log_num:
            break

    return latest


# ---------------------------------------------------------------------------
# ダウンロード
# ---------------------------------------------------------------------------
def download_log(master, log_id: int, log_size: int,
                 output_path: str, timeout: float = 300.0) -> int:
    """指定 ID のログをダウンロードしてファイルに保存する。

    タイムアウト時は受信済みオフセットから再リクエストを送る。

    Returns:
        受信バイト数。
    """
    buf = bytearray(log_size)
    next_ofs = 0

    master.mav.log_request_data_send(
        master.target_system,
        master.target_component,
        log_id,
        0,
        log_size,
    )

    last_activity = time.monotonic()
    last_progress_t = 0.0
    retries = 0
    max_retries = 20

    while next_ofs < log_size:
        if (time.monotonic() - last_activity) > timeout:
            print(f"  タイムアウト ({timeout}s)")
            break

        msg = master.recv_match(
            type="LOG_DATA", blocking=True,
            timeout=min(_RECV_TIMEOUT, timeout),
        )

        if msg is None:
            retries += 1
            if retries > max_retries:
                print(f"  リトライ上限到達 ({max_retries})")
                break
            print(f"  再リクエスト (ofs={next_ofs}, retry={retries}/{max_retries})")
            master.mav.log_request_data_send(
                master.target_system,
                master.target_component,
                log_id,
                next_ofs,
                log_size - next_ofs,
            )
            continue

        if msg.id != log_id:
            continue

        count = msg.count
        if count == 0:
            break

        ofs = msg.ofs
        end = min(ofs + count, log_size)
        buf[ofs:end] = msg.data[:end - ofs]

        if ofs <= next_ofs:
            next_ofs = max(next_ofs, ofs + count)

        retries = 0
        last_activity = time.monotonic()

        now = time.monotonic()
        if (now - last_progress_t) > 2.0 or next_ofs >= log_size:
            pct = next_ofs * 100.0 / log_size
            print(f"  ダウンロード進捗: {next_ofs}/{log_size} ({pct:.1f}%)")
            last_progress_t = now

    master.mav.log_request_end_send(
        master.target_system,
        master.target_component,
    )

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(bytes(buf[:log_size]))

    return next_ofs


# ---------------------------------------------------------------------------
# 高レベル API
# ---------------------------------------------------------------------------
def download_latest_log(master, output_dir: str = "./logs",
                        timeout_list: float = 30.0,
                        timeout_download: float = 300.0) -> str | None:
    """最新フライトログをダウンロードする。

    Returns:
        保存先パス。失敗時は None。
    """
    print("[ログ] ログ一覧を取得中...")
    entry = get_latest_log_entry(master, timeout=timeout_list)

    if entry is None:
        print("[ログ] ログが見つかりませんでした")
        return None

    log_size = entry.size
    log_id = entry.id
    print(f"[ログ] 最新ログ: id={log_id}, size={log_size} bytes")

    if log_size == 0:
        print("[ログ] ログサイズが 0 です。スキップ。")
        return None

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"log_{log_id:04d}_{timestamp}.ulg"
    output_path = os.path.join(output_dir, filename)

    print(f"[ログ] ダウンロード開始 → {output_path}")
    received = download_log(
        master, log_id, log_size, output_path,
        timeout=timeout_download,
    )

    if received >= log_size:
        print(f"[ログ] ダウンロード完了: {output_path} ({received} bytes)")
    elif received > 0:
        print(f"[ログ] 部分ダウンロード: {output_path} "
              f"({received}/{log_size} bytes)")
    else:
        print("[ログ] ダウンロード失敗")
        return None

    return output_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
_MAV_AUTOPILOT_PX4 = 12


def _wait_for_px4(master, timeout: float = 30.0) -> bool:
    print("PX4 オートパイロットのハートビート待機...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = master.recv_match(
            type="HEARTBEAT", blocking=True,
            timeout=min(remaining, 5.0),
        )
        if msg is None:
            continue
        if msg.autopilot == _MAV_AUTOPILOT_PX4 and msg.get_srcSystem() != 0:
            master.target_system = msg.get_srcSystem()
            master.target_component = msg.get_srcComponent()
            print(f"PX4 検出: system={master.target_system} "
                  f"component={master.target_component}")
            return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="PX4 フライトログを MAVLink 経由でダウンロード",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--connection", default="udp:0.0.0.0:14550",
                        help="MAVLink 接続先")
    parser.add_argument("--output-dir", default="./logs",
                        help="ログ保存先ディレクトリ")
    parser.add_argument("--timeout-list", type=float, default=30.0,
                        help="ログ一覧取得のタイムアウト (秒)")
    parser.add_argument("--timeout-download", type=float, default=300.0,
                        help="ログダウンロードのタイムアウト (秒)")
    args = parser.parse_args()

    print(f"接続中: {args.connection}")
    master = mavutil.mavlink_connection(args.connection)

    if not _wait_for_px4(master):
        print("PX4 が見つかりませんでした", file=sys.stderr)
        return 1

    result = download_latest_log(
        master,
        output_dir=args.output_dir,
        timeout_list=args.timeout_list,
        timeout_download=args.timeout_download,
    )
    return 0 if result else 1


if __name__ == "__main__":
    sys.exit(main())
