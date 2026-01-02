#!/usr/bin/env python3
"""
Read a CSV, multiply specified column groups by constant factors, and write a CSV.

Examples:
  python3 scale_csv_columns.py -i input.csv -o output.csv \\
    --scale "Fx,Fy,Fz:0.001" \\
    --scale "Mx,My,Mz:1000"

Notes:
  - If any specified column is missing, this script exits with an error.
  - If any specified column contains non-numeric values, this script exits with an error.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import List, Sequence, Tuple


@dataclass(frozen=True)
class ScaleSpec:
    columns: Tuple[str, ...]
    factor: float


def parse_scale_specs(raw_specs: Sequence[str]) -> List[ScaleSpec]:
    specs: List[ScaleSpec] = []
    for raw in raw_specs:
        if raw is None:
            continue
        s = raw.strip()
        if not s:
            raise ValueError("Empty --scale specification.")

        if ":" not in s:
            raise ValueError(
                f'Invalid --scale "{raw}". Expected format: "col1,col2:factor".'
            )

        cols_part, factor_part = s.rsplit(":", 1)
        cols = tuple(c.strip() for c in cols_part.split(",") if c.strip())
        if not cols:
            raise ValueError(
                f'Invalid --scale "{raw}". No columns found before ":".'
            )

        try:
            factor = float(factor_part.strip())
        except ValueError as e:
            raise ValueError(
                f'Invalid --scale "{raw}". Factor must be a number.'
            ) from e

        specs.append(ScaleSpec(columns=cols, factor=factor))

    if not specs:
        raise ValueError("At least one --scale must be provided.")
    return specs


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Multiply specified CSV columns by constant factors (multiple groups supported)."
    )
    p.add_argument(
        "-i",
        "--input",
        required=True,
        help="Input CSV path.",
    )
    p.add_argument(
        "-o",
        "--output",
        default=None,
        help='Output CSV path. If omitted, writes to stdout ("-").',
    )
    p.add_argument(
        "--scale",
        action="append",
        default=[],
        help='Scaling spec in the form "col1,col2:factor". Can be specified multiple times.',
    )
    p.add_argument(
        "--sep",
        default=",",
        help='CSV delimiter (default: ",").',
    )
    p.add_argument(
        "--encoding",
        default="utf-8",
        help='CSV encoding for both read/write (default: "utf-8").',
    )
    return p


def main(argv: Sequence[str]) -> int:
    args = build_argparser().parse_args(argv)

    try:
        specs = parse_scale_specs(args.scale)
    except ValueError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 2

    try:
        import pandas as pd
    except Exception as e:
        print(
            "[ERROR] pandas is required for this script. Please install it (e.g., pip install pandas).",
            file=sys.stderr,
        )
        print(f"[ERROR] Import error: {e}", file=sys.stderr)
        return 2

    try:
        df = pd.read_csv(args.input, sep=args.sep, encoding=args.encoding)
    except Exception as e:
        print(f"[ERROR] Failed to read CSV: {args.input}", file=sys.stderr)
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1

    # Validate columns
    requested_cols = sorted({c for spec in specs for c in spec.columns})
    missing = [c for c in requested_cols if c not in df.columns]
    if missing:
        print("[ERROR] Missing columns:", file=sys.stderr)
        for c in missing:
            print(f"  - {c}", file=sys.stderr)
        print("[ERROR] Available columns:", file=sys.stderr)
        for c in df.columns:
            print(f"  - {c}", file=sys.stderr)
        return 1

    # Apply scaling (force numeric conversion for safety)
    for spec in specs:
        for col in spec.columns:
            try:
                df[col] = pd.to_numeric(df[col], errors="raise") * spec.factor
            except Exception as e:
                print(
                    f'[ERROR] Failed to scale column "{col}" by factor {spec.factor}.',
                    file=sys.stderr,
                )
                print(f"[ERROR] {e}", file=sys.stderr)
                return 1

    # Write
    out_path = args.output
    if out_path in (None, "-", ""):
        try:
            df.to_csv(sys.stdout, index=False, sep=args.sep)
        except Exception as e:
            print("[ERROR] Failed to write CSV to stdout.", file=sys.stderr)
            print(f"[ERROR] {e}", file=sys.stderr)
            return 1
        return 0

    try:
        df.to_csv(out_path, index=False, sep=args.sep, encoding=args.encoding)
    except Exception as e:
        print(f"[ERROR] Failed to write CSV: {out_path}", file=sys.stderr)
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))


