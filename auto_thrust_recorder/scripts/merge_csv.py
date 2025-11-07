import os
import sys
import glob
import argparse
from datetime import datetime
from typing import List, Optional, Tuple

import pandas as pd
from pathlib import Path


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CSV ファイルを探索・結合して 1 ファイルにマージします"
    )
    parser.add_argument(
        "--files",
        nargs="+",
        type=str,
        default=None,
        help="明示的に結合するCSVファイルをスペース区切りで指定 (指定時は探索を無視)",
    )
    parser.add_argument(
        "-d",
        "--directory",
        type=str,
        default=".",
        help="CSV を検索するディレクトリ (default: .)",
    )
    parser.add_argument(
        "-k",
        "--keyword",
        type=str,
        default="",
        help="ファイル名に含まれるキーワード (default: 空=全CSV)",
    )
    parser.add_argument(
        "-r",
        "--recursive",
        action="store_true",
        help="再帰的に探索します",
    )
    parser.add_argument(
        "--columns-mode",
        choices=["union", "intersection"],
        default="union",
        help="列集合の扱い: union=全列, intersection=共通列のみ (default: union)",
    )
    parser.add_argument(
        "--add-source-col",
        type=str,
        default=None,
        help="元ファイル名を格納する列名 (例: source)。未指定なら追加しない",
    )
    parser.add_argument(
        "--dedup-by",
        type=str,
        default=None,
        help="指定列で重複行を削除。カンマ区切り (例: time,control)",
    )
    parser.add_argument(
        "--sort-by",
        type=str,
        default=None,
        help="結合後の並び替え列。カンマ区切り (例: distance,tilt_angle)",
    )
    parser.add_argument(
        "--encoding",
        type=str,
        default="utf-8",
        help="入出力の文字エンコーディング (default: utf-8)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="出力CSVファイルパス (default: merged_日付時刻.csv)",
    )
    return parser.parse_args()


def find_csv_files(directory: str, keyword: str, recursive: bool) -> List[str]:
    if recursive:
        pattern = os.path.join(directory, "**", "*.csv")
        candidates = glob.glob(pattern, recursive=True)
    else:
        pattern = os.path.join(directory, "*.csv")
        candidates = glob.glob(pattern)

    if keyword:
        candidates = [p for p in candidates if keyword in os.path.basename(p)]

    candidates.sort()
    return candidates


def try_read_csv(path: str, encoding: str) -> pd.DataFrame:
    tried: List[Tuple[str, Optional[str]]] = [
        (encoding, None),
        ("utf-8-sig", None),
        ("cp932", None),
        ("shift_jis", None),
    ]
    last_error: Optional[Exception] = None
    for enc, _ in tried:
        try:
            return pd.read_csv(path, encoding=enc)
        except Exception as e:
            last_error = e
            continue
    raise RuntimeError(f"CSV 読み込み失敗: {path} ({last_error})")


def merge_dataframes(
    file_paths: List[str],
    columns_mode: str,
    add_source_col: Optional[str],
    dedup_by: Optional[List[str]],
    sort_by: Optional[List[str]],
    encoding: str,
) -> pd.DataFrame:
    dataframes: List[pd.DataFrame] = []
    for p in file_paths:
        df = try_read_csv(p, encoding=encoding)
        if add_source_col:
            df[add_source_col] = os.path.basename(p)
        dataframes.append(df)

    if not dataframes:
        raise ValueError("結合対象のデータフレームが空です")

    if columns_mode == "intersection":
        common_cols = set(dataframes[0].columns)
        for df in dataframes[1:]:
            common_cols &= set(df.columns)
        common_order = [c for c in dataframes[0].columns if c in common_cols]
        dataframes = [df[common_order].copy() for df in dataframes]
        merged = pd.concat(dataframes, axis=0, ignore_index=True)
    else:
        merged = pd.concat(dataframes, axis=0, ignore_index=True, sort=True)

    if dedup_by:
        merged = merged.drop_duplicates(subset=dedup_by).reset_index(drop=True)

    if sort_by:
        missing = [c for c in sort_by if c not in merged.columns]
        if missing:
            raise ValueError(f"sort-by 指定列が存在しません: {missing}")
        merged = merged.sort_values(by=sort_by).reset_index(drop=True)

    return merged


def output_path(output: Optional[str]) -> str:
    if output:
        return output
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"merged_{ts}.csv"


def resolve_file_paths(files: List[str], base_directory: Optional[str]) -> Tuple[List[str], List[str]]:
    cwd = Path.cwd()
    script_dir = Path(__file__).resolve().parent
    base_dir = Path(base_directory).resolve() if base_directory else None

    resolved: List[str] = []
    missing: List[str] = []

    for p in files:
        raw = Path(p).expanduser()
        candidates = [
            (cwd / raw) if not raw.is_absolute() else raw,
        ]
        if base_dir is not None:
            candidates.append(base_dir / raw)
        candidates.append(script_dir / raw)

        chosen: Optional[Path] = None
        for c in candidates:
            # c には相対が混在しないよう作成しているが、念のため絶対化
            abs_c = c if c.is_absolute() else (cwd / c)
            if abs_c.is_file():
                chosen = abs_c.resolve()
                break

        if chosen is None:
            missing.append(p)
        else:
            resolved.append(str(chosen))

    return resolved, missing


def main() -> None:
    args = parse_arguments()
    
    if args.files:
        # 明示指定ファイルを順序維持で解決（CWD → --directory → スクリプトディレクトリ）
        files, missing = resolve_file_paths(args.files, args.directory)
        if missing:
            print(f"存在しない/解決できないファイルがあります: {missing}")
            sys.exit(1)
        print(f"指定ファイル数: {len(files)}")
        for f in files:
            print(f" - {f}")
    else:
        files = find_csv_files(args.directory, args.keyword, args.recursive)
        if not files:
            print("対象のCSVが見つかりませんでした。検索条件を見直してください。")
            sys.exit(1)
        print(f"検出ファイル数: {len(files)}")
        for f in files:
            print(f" - {f}")

    dedup_cols = args.dedup_by.split(",") if args.dedup_by else None
    sort_cols = args.sort_by.split(",") if args.sort_by else None

    try:
        merged = merge_dataframes(
            file_paths=files,
            columns_mode=args.columns_mode,
            add_source_col=args.add_source_col,
            dedup_by=dedup_cols,
            sort_by=sort_cols,
            encoding=args.encoding,
        )
    except Exception as e:
        print(f"マージ処理に失敗しました: {e}")
        sys.exit(2)

    out = output_path(args.output)
    try:
        merged.to_csv(out, index=False, encoding=args.encoding)
    except Exception:
        # 出力時エンコードで失敗したら UTF-8-SIG にフォールバック
        merged.to_csv(out, index=False, encoding="utf-8-sig")

    print(f"出力: {os.path.abspath(out)}")


if __name__ == "__main__":
    main()


