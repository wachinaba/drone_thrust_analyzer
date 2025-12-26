#!/usr/bin/env python3
"""
postprocess_simple.py

各データフォルダで実行する前提の簡易 postprocess ランチャ。

実行する処理はこの2つだけ:
  1) csv_concat_4.py で raw CSV を結合して concat.csv を生成
  2) kernel_ridge_regression.py で concat.csv に対して KRR を実行

使い方（例）:
  cd /path/to/data_folder
  python3 /path/to/auto_thrust_recorder/scripts/postprocess_simple.py
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def _run(args: list[str], *, cwd: Path) -> int:
    print(f"[postprocess_simple] run: {' '.join(args)} (cwd={cwd})", flush=True)
    p = subprocess.run(args, cwd=str(cwd))
    if p.returncode != 0:
        print(f"[postprocess_simple] failed: rc={p.returncode}", file=sys.stderr, flush=True)
    return int(p.returncode)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run simple postprocess in a single data directory (concat(raw) + KRR).")
    p.add_argument("--python-bin", default="python3", help="python executable (default: python3)")
    p.add_argument(
        "--scripts-dir",
        default=None,
        help="directory containing csv_concat_4.py / kernel_ridge_regression.py (default: this script's directory)",
    )
    p.add_argument("--concat-output", default="concat.csv", help="output filename for concat step (default: concat.csv)")
    p.add_argument(
        "--krr-output",
        default="krr.png",
        help="--output passed to kernel_ridge_regression.py (default: krr.png). Note: the script writes *_results.png etc.",
    )
    # KRR defaults (A: minimal)
    p.add_argument("--krr-target", default="normalized_moment", help="KRR --target (default: normalized_moment)")
    p.add_argument(
        "--krr-target-expr",
        default="`torque_x` / `target_thrust` / `prop_spacing` * 100",
        help="KRR --target-expr (default: normalized moment expression)",
    )
    p.add_argument(
        "--krr-features",
        default="distance,tilt_angle,fold_angle,slant_angle,force_z,wall_spacing",
        help="KRR --features (comma-separated)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cwd = Path.cwd()
    scripts_dir = Path(args.scripts_dir).expanduser().resolve() if args.scripts_dir else Path(__file__).resolve().parent

    csv_concat = scripts_dir / "csv_concat_4.py"
    krr = scripts_dir / "kernel_ridge_regression.py"

    if not csv_concat.is_file():
        print(f"[postprocess_simple] not found: {csv_concat}", file=sys.stderr)
        return 2
    if not krr.is_file():
        print(f"[postprocess_simple] not found: {krr}", file=sys.stderr)
        return 2

    # 1) concat (keyword=raw)
    rc = _run(
        [
            str(args.python_bin),
            str(csv_concat),
            "-k",
            "raw",
            "-d",
            ".",
            "--output",
            str(args.concat_output),
            "--group-by-file-timestamp",
            "--dropna-mode",
            "none",
            "--default-column",
            "slant_angle=0",
            "--default-column-mode",
            "missing",
        ],
        cwd=cwd,
    )
    if rc != 0:
        return rc

    concat_path = cwd / str(args.concat_output)
    if not concat_path.is_file():
        print(f"[postprocess_simple] missing output: {concat_path}", file=sys.stderr)
        return 2

    # 2) KRR (minimal)
    rc = _run(
        [
            str(args.python_bin),
            str(krr),
            str(concat_path),
            "--output",
            str(args.krr_output),
            "--target",
            str(args.krr_target),
            "--target-expr",
            str(args.krr_target_expr),
            "--normalize",
            "--features",
            str(args.krr_features),
        ],
        cwd=cwd,
    )
    return rc


if __name__ == "__main__":
    raise SystemExit(main())


