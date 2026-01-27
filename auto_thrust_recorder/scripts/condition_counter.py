#!/usr/bin/env python3
"""
csv_concat_4.py の出力CSVを読み取り、指定した条件列のユニーク数と
条件組合せ（ユニーク行）数をログするユーティリティ。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class RangeSpec:
    col: str
    min_val: Optional[float]
    max_val: Optional[float]


def _parse_fix_list(fixes: Sequence[str], df_columns: Sequence[str]) -> Dict[str, float]:
    """--fix col=value を {col: float(value)} に変換。CSVに無い列は警告して無視。"""
    col_set = set(df_columns)
    out: Dict[str, float] = {}
    for item in fixes or []:
        if not item or ("=" not in item):
            continue
        col, val = item.split("=", 1)
        col = col.strip()
        if col not in col_set:
            print(f"[warn] --fix '{col}' はCSVに存在しないため無視します。")
            continue
        try:
            out[col] = float(str(val).strip())
        except Exception:
            print(f"[warn] --fix '{col}={val}' の値が数値に変換できないため無視します。")
            continue
    return out


def _parse_ranges(ranges: Sequence[str], df_columns: Sequence[str]) -> List[RangeSpec]:
    """
    --range col=min:max もしくは --range col=min,max を RangeSpec に変換。
    片側省略（min: / :max / ,max / min,）を許容。
    """
    col_set = set(df_columns)
    out: List[RangeSpec] = []
    for item in ranges or []:
        if not item or ("=" not in item):
            continue
        col, expr = item.split("=", 1)
        col = col.strip()
        expr = str(expr).strip()
        if col not in col_set:
            print(f"[warn] --range '{col}' はCSVに存在しないため無視します。")
            continue
        if ":" in expr:
            a, b = expr.split(":", 1)
        elif "," in expr:
            a, b = expr.split(",", 1)
        else:
            print(f"[warn] --range '{col}={expr}' は 'min:max' または 'min,max' 形式で指定してください。無視します。")
            continue

        a = a.strip()
        b = b.strip()
        min_val = None if a == "" else float(a)
        max_val = None if b == "" else float(b)
        out.append(RangeSpec(col=col, min_val=min_val, max_val=max_val))
    return out


def _apply_filters(df: pd.DataFrame, *, fixes: Dict[str, float], ranges: List[RangeSpec]) -> pd.DataFrame:
    """fix/range を順に適用してフィルタした DataFrame を返す。"""
    out = df
    if out is None or out.empty:
        return out

    # fix
    for col, val in (fixes or {}).items():
        if col not in out.columns:
            continue
        s = pd.to_numeric(out[col], errors="coerce")
        m = np.isfinite(s.values) & np.isclose(s.values, float(val), atol=1e-9, rtol=0.0)
        before = len(out)
        out = out.loc[m].copy()
        after = len(out)
        print(f"[info] filter --fix: {col}=={val:g}  kept={after}/{before}")
        if out.empty:
            return out

    # range
    for r in ranges or []:
        if r.col not in out.columns:
            continue
        s = pd.to_numeric(out[r.col], errors="coerce")
        m = np.isfinite(s.values)
        if r.min_val is not None:
            m = m & (s.values >= float(r.min_val))
        if r.max_val is not None:
            m = m & (s.values <= float(r.max_val))
        before = len(out)
        out = out.loc[m].copy()
        after = len(out)
        a = "" if r.min_val is None else f"{float(r.min_val):g}"
        b = "" if r.max_val is None else f"{float(r.max_val):g}"
        print(f"[info] filter --range: {r.col} in [{a}, {b}]  kept={after}/{before}")
        if out.empty:
            return out

    return out


def _validate_condition_cols(df: pd.DataFrame, cols: Sequence[str]) -> None:
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise ValueError(f"条件列がCSVに存在しません: {missing}")


def _count_conditions(df: pd.DataFrame, condition_cols: Sequence[str]) -> Tuple[Dict[str, int], int]:
    """(列ごとのユニーク数, 条件組合せユニーク数) を返す。"""
    uniq_per_col: Dict[str, int] = {}
    for c in condition_cols:
        # 実験条件の列は NaN を条件として数えたくないことが多いので dropna=True
        uniq_per_col[c] = int(df[c].nunique(dropna=True))

    combo_unique = int(df.loc[:, list(condition_cols)].drop_duplicates().shape[0])
    return uniq_per_col, combo_unique


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="条件列のユニーク数・条件組合せ数をカウントしてログします")
    p.add_argument("--input", "-i", required=True, help="入力CSV（csv_concat_4.py の出力）")
    p.add_argument(
        "--condition-cols",
        nargs="+",
        required=True,
        help="条件列名（例: distance target_thrust tilt_angle fold_angle slant_angle wall_spacing）",
    )
    p.add_argument("--fix", action="append", default=[], help="固定条件 'col=value'（複数可、値は数値）")
    p.add_argument(
        "--range",
        dest="ranges",
        action="append",
        default=[],
        help="範囲条件 'col=min:max' または 'col=min,max'（複数可、片側省略可）",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"入力CSVが見つかりません: {input_path}")

    df = pd.read_csv(str(input_path))
    condition_cols = list(args.condition_cols)
    _validate_condition_cols(df, condition_cols)

    print(f"[info] input: {input_path}")
    print(f"[info] rows: {len(df)}")
    print(f"[info] condition_cols: {condition_cols}")

    fixes = _parse_fix_list(args.fix, df.columns)
    ranges = _parse_ranges(args.ranges, df.columns)

    df_f = _apply_filters(df, fixes=fixes, ranges=ranges)
    print(f"[info] rows_after_filter: {len(df_f)}")

    uniq_per_col, combo_unique = _count_conditions(df_f, condition_cols)
    print("[info] unique_per_column:")
    for c in condition_cols:
        print(f"  - {c}: {uniq_per_col.get(c, 0)}")
    print(f"[info] unique_condition_combinations: {combo_unique}")


if __name__ == "__main__":
    main()


