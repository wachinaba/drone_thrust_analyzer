import argparse
import sys
from pathlib import Path
import pandas as pd
import numpy as np


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="旧形式の風速列(seven_segment_value_0..3)を新形式(front_in/front_out/rear_out/rear_in)へ変換するツール"
    )
    parser.add_argument(
        "-d", "--directory", nargs="+", type=str, default=["."],
        help="再帰探索するディレクトリ（複数指定可、デフォルト: 現在ディレクトリ）"
    )
    parser.add_argument(
        "--policy", type=str, choices=["auto", "normal", "reversed"], default="auto",
        help="列対応ポリシー: auto=ファイル名に'back'で反転, normal=通常対応, reversed=反転対応"
    )
    parser.add_argument(
        "--inplace", action="store_true",
        help="同じファイルに上書き保存（指定しない場合は *_named.csv を出力）"
    )
    parser.add_argument(
        "--drop-old", action="store_true",
        help="旧列 seven_segment_value_0..3 を削除する"
    )
    parser.add_argument(
        "--force", action="store_true",
        help="新列が既に存在しても再生成を試みる（旧列が無ければそのまま）"
    )
    parser.add_argument(
        "--suffix", type=str, default="_named",
        help="inplace でない場合の出力ファイル接尾辞（デフォルト: _named）"
    )
    return parser.parse_args()


def find_csv_files(directories):
    files_set = set()
    for d in directories:
        base = Path(d)
        if not base.exists():
            print(f"[WARN] ディレクトリが存在しません: {d}")
            continue
        for f in base.rglob("*.csv"):
            files_set.add(f.resolve())
    return sorted(files_set)


def determine_reversed(file_path: Path, policy: str) -> bool:
    if policy == "normal":
        return False
    if policy == "reversed":
        return True
    # auto
    return ("back" in str(file_path).lower())


def convert_dataframe(df: pd.DataFrame, reversed_mapping: bool, force: bool, drop_old: bool) -> tuple[pd.DataFrame, bool]:
    """
    df を変換し、(変換後DataFrame, 実際に変更が加わったか) を返す
    """
    named_cols = ["front_in", "front_out", "rear_out", "rear_in"]
    old_cols = [f"seven_segment_value_{i}" for i in range(4)]

    has_named_all = all(c in df.columns for c in named_cols)
    has_any_old = any(c in df.columns for c in old_cols)

    modified = False

    if has_named_all and not force:
        # すでに新列があるので何もしない
        return df, False

    # 旧列がない場合、forceでも再生成材料がないため何もしない
    if not has_any_old:
        return df, False

    # 旧列不足分は NaN で埋める
    for c in old_cols:
        if c not in df.columns:
            df[c] = np.nan

    if reversed_mapping:
        mapping = {
            "front_in": "seven_segment_value_3",
            "front_out": "seven_segment_value_2",
            "rear_out": "seven_segment_value_1",
            "rear_in": "seven_segment_value_0",
        }
    else:
        mapping = {
            "front_in": "seven_segment_value_0",
            "front_out": "seven_segment_value_1",
            "rear_out": "seven_segment_value_2",
            "rear_in": "seven_segment_value_3",
        }

    # 新列を生成（既存があっても上書きする想定：force時）
    for new_col, src_col in mapping.items():
        df[new_col] = df[src_col]
        modified = True

    if drop_old:
        to_drop = [c for c in old_cols if c in df.columns]
        if to_drop:
            df = df.drop(columns=to_drop)
            modified = True

    return df, modified


def process_file(file_path: Path, args) -> str:
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        return f"[SKIP] 読み込み失敗: {file_path} | 理由: {e}"

    reversed_flag = determine_reversed(file_path, args.policy)
    df_converted, changed = convert_dataframe(
        df=df, reversed_mapping=reversed_flag, force=args.force, drop_old=args.drop_old
    )

    if not changed:
        return f"[SKIP] 変更なし: {file_path}"

    try:
        if args.inplace:
            df_converted.to_csv(file_path, index=False)
            return f"[OK] 上書き保存: {file_path}"
        else:
            out_path = file_path.with_name(f"{file_path.stem}{args.suffix}{file_path.suffix}")
            df_converted.to_csv(out_path, index=False)
            return f"[OK] 変換出力: {out_path}"
    except Exception as e:
        return f"[FAIL] 書き込み失敗: {file_path} | 理由: {e}"


def main():
    args = parse_arguments()
    csv_files = find_csv_files(args.directory)
    if not csv_files:
        print("CSVファイルが見つかりませんでした。")
        sys.exit(1)

    print(f"対象CSVファイル数: {len(csv_files)}")
    results = []
    for f in csv_files:
        res = process_file(f, args)
        print(res)
        results.append(res)

    # 簡易サマリ
    ok = sum(r.startswith("[OK]") for r in results)
    skip = sum(r.startswith("[SKIP]") for r in results)
    fail = sum(r.startswith("[FAIL]") for r in results)
    print(f"\n処理結果: OK={ok}, SKIP={skip}, FAIL={fail}")


if __name__ == "__main__":
    main()


