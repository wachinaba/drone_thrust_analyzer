import os
import sys
import re
import argparse
from pathlib import Path
from datetime import datetime
from typing import List, Optional, Tuple

import numpy as np
import pandas as pd


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="CSV 内の列を参照して前処理列を追加するツール（再帰探索で一括処理）。prop_spacing はファイル名から抽出"
    )
    parser.add_argument(
        "-d", "--directory",
        nargs="+",
        type=str,
        default=["."],
        help="検索するディレクトリ（複数指定可。デフォルト: 現在ディレクトリ）",
    )
    parser.add_argument(
        "-k", "--keywords",
        nargs="+",
        type=str,
        default=[],
        help="対象 CSV をファイル名でフィルタするキーワード（空なら全 CSV 対象）",
    )
    parser.add_argument(
        "-a", "--and-keywords",
        action="store_true",
        help="キーワードを AND 条件で適用（未指定時は OR 条件）",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="出力先ディレクトリ（未指定時は各入力 CSV と同じディレクトリ）",
    )
    parser.add_argument(
        "--tail-keyword",
        type=str,
        default="",
        help="出力ファイル末尾のキーワード（デフォルト: 空文字＝付与しない）",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="既存の normalized_moment 列があっても再計算して上書きする",
    )
    parser.add_argument(
        "--param-rename",
        action="append",
        default=[],
        help="ファイル名からのパラメータ抽出時に用いるキー名のリネーム規則 'old:new'（複数指定可）",
    )
    return parser.parse_args()


def find_csv_files(keywords: List[str], directories: List[str], and_keywords: bool) -> List[str]:
    files_set = set()
    for dir_path in directories:
        for p in Path(dir_path).rglob("*.csv"):
            files_set.add(str(p))
    files = sorted(files_set)
    if not keywords:
        return files
    if and_keywords:
        return [f for f in files if all(kw in os.path.basename(f) for kw in keywords)]
    return [f for f in files if any(kw in os.path.basename(f) for kw in keywords)]


def _build_output_path(input_path: str, output_dir: Optional[str], tail_keyword: str) -> Path:
    in_path = Path(input_path)
    stem = in_path.stem
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    name_parts = [stem]
    if tail_keyword:
        name_parts.append(tail_keyword)
    name_parts.append(timestamp)
    out_name = "_".join(name_parts) + ".csv"
    out_dir = Path(output_dir) if output_dir else in_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir / out_name


def _compute_normalized_moment(df: pd.DataFrame, prop_spacing_value: float) -> pd.Series:
    # 必須列の存在を前提に、ゼロ割を NaN で回避
    denom = df["force_z"] * (prop_spacing_value / 2.0)
    # ゼロを NaN に置換してから割り算
    denom_safe = denom.replace(0, np.nan)
    return df["torque_x_bias_corrected"] / denom_safe


def _parse_param_renames(rename_args):
    rename_map = {}
    if not rename_args:
        return rename_map
    for item in rename_args:
        if not isinstance(item, str) or ":" not in item:
            continue
        old, new = item.split(":", 1)
        old = old.strip()
        new = new.strip()
        if old and new:
            rename_map[old.lower()] = new
    return rename_map


def _coerce_value(val_str):
    if val_str is None:
        return None
    s = str(val_str)
    s = re.sub(r"\[[^\]]+\]", "", s)  # 単位表記を除去
    s = s.strip()
    if re.fullmatch(r"-?\d+", s):
        try:
            return int(s)
        except Exception:
            pass
    if re.fullmatch(r"-?\d+(?:\.\d+)?", s):
        try:
            return float(s)
        except Exception:
            pass
    return s.lower()


def _strip_trailing_keyword_timestamp(base_filename, tail_keywords):
    if not tail_keywords:
        return base_filename, None
    for kw in tail_keywords:
        if not kw:
            continue
        kw_esc = re.escape(str(kw))
        pattern = rf"^(?P<prefix>.*)_{kw_esc}_(?P<ts>\d{{8}}(?:[-_]?\d{{6}})?)\.csv$"
        m = re.match(pattern, base_filename, re.IGNORECASE)
        if m:
            prefix = m.group("prefix")
            ts = m.group("ts")
            return f"{prefix}.csv", ts
    return base_filename, None


def extract_parameters_generic(filename, param_rename_user=None, tail_keywords=None):
    base = os.path.basename(filename)
    base, tail_ts = _strip_trailing_keyword_timestamp(base, tail_keywords)
    pattern = r"(?P<key>[A-Za-z][A-Za-z0-9]*)=(?P<value>.*?)(?=_(?:[A-Za-z][A-Za-z0-9]*)=|\.csv$|$)"
    matches = re.finditer(pattern, base)

    default_rename = {
        "tilt": "tilt_angle",
        "fold": "fold_angle",
        "wheelbase": "prop_spacing",
        "keyword": "direction",
        "direction": "direction",
        "wallspacing": "wall_spacing",
        "flowdistance": "flow_distance",
    }
    user_map = _parse_param_renames(param_rename_user)
    rename_map = {**default_rename, **user_map}

    params = {}
    for m in matches:
        key = m.group("key")
        value = m.group("value")
        if not key:
            continue
        key_norm = key.lower()
        key_final = rename_map.get(key_norm, key_norm)
        params[key_final] = _coerce_value(value)

    if tail_ts is not None:
        params["file_timestamp"] = tail_ts

    return params if params else None


def process_one_csv(input_csv: str, overwrite: bool, rename_args: List[str], tail_keywords: List[str]) -> Tuple[Optional[pd.DataFrame], Optional[str]]:
    try:
        df = pd.read_csv(input_csv)
    except Exception as e:
        return None, f"ReadError: {e}"

    required_cols = ["torque_x_bias_corrected", "force_z"]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        return None, f"MissingColumns: {','.join(missing)}"

    # ファイル名から prop_spacing を取得
    filename = os.path.basename(input_csv)
    params = extract_parameters_generic(filename, rename_args, tail_keywords)
    if not params or "prop_spacing" not in params:
        return None, "MissingParams: prop_spacing"
    prop_spacing_value = params["prop_spacing"]
    try:
        prop_spacing_value = float(prop_spacing_value)
    except Exception:
        return None, "InvalidParam: prop_spacing_not_numeric"

    if "normalized_moment" in df.columns and not overwrite:
        # 既に存在し、上書き指定がなければスキップ
        return df, "AlreadyExists"

    try:
        df["normalized_moment"] = _compute_normalized_moment(df, prop_spacing_value)
    except Exception as e:
        return None, f"ComputeError: {e}"

    return df, None


def main():
    args = parse_arguments()

    csv_files = find_csv_files(args.keywords, args.directory, args.and_keywords)
    if not csv_files:
        print("CSV が見つかりませんでした。検索条件を確認してください。")
        sys.exit(1)

    print(f"対象ファイル数: {len(csv_files)}")

    processed = 0
    skipped_missing = []
    skipped_exists = []
    failed_compute = []
    failed_read = []
    skipped_params = []

    for f in csv_files:
        df, reason = process_one_csv(f, args.overwrite, getattr(args, "param_rename", []), getattr(args, "keywords", []))
        if df is None and reason:
            if reason.startswith("ReadError"):
                failed_read.append((f, reason))
            elif reason.startswith("MissingColumns"):
                skipped_missing.append((f, reason))
            elif reason.startswith("MissingParams") or reason.startswith("InvalidParam"):
                skipped_params.append((f, reason))
            elif reason.startswith("ComputeError"):
                failed_compute.append((f, reason))
            else:
                failed_compute.append((f, reason))
            continue

        if reason == "AlreadyExists":
            # 上書きしないため書き出しもスキップ
            skipped_exists.append((f, reason))
            continue

        out_path = _build_output_path(f, args.output_dir, args.tail_keyword)
        try:
            df.to_csv(out_path, index=False)
            processed += 1
            print(f"  出力: {out_path}")
        except Exception as e:
            failed_compute.append((f, f"WriteError: {e}"))

    print("\n処理結果:")
    print(f"  正常出力: {processed}")
    if skipped_exists:
        print(f"  スキップ(既存列あり・上書き無効): {len(skipped_exists)}")
        for f, reason in skipped_exists:
            print(f"    {f} | {reason}")
    if skipped_missing:
        print(f"  スキップ(必須列欠損): {len(skipped_missing)}")
        for f, reason in skipped_missing:
            print(f"    {f} | {reason}")
    if skipped_params:
        print(f"  スキップ(ファイル名パラメータ不足/不正): {len(skipped_params)}")
        for f, reason in skipped_params:
            print(f"    {f} | {reason}")
    if failed_read:
        print(f"  失敗(読込): {len(failed_read)}")
        for f, reason in failed_read:
            print(f"    {f} | {reason}")
    if failed_compute:
        print(f"  失敗(計算/書込): {len(failed_compute)}")
        for f, reason in failed_compute:
            print(f"    {f} | {reason}")


if __name__ == "__main__":
    main()


