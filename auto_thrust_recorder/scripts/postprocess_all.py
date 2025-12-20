#!/usr/bin/env python3
"""
postprocess_all.py

postprocess_all.bash と同等の処理を Python で実行するランチャ。

- root 直下のサブフォルダを走査して、必要な入力があるフォルダのみ対象
- 各サブフォルダで（必要なものだけ）並列実行:
    1) merge_front_back_bias.py（--no-biascorr でスキップ可）
    2) csv_concat_4.py (concat.csv)（--no-concat でスキップ可）
    3) add_calculated_columns_to_csv.py（--no-morph でスキップ可）
  - 3) は concat.csv が必要（無ければ失敗）
- 最後に root で:
    4) merge_csv.py (concat_merged.csv)
    5) kernel_ridge_regression.py (krr.png)
  を 1 回だけ実行
  - 4) は --no-merge でスキップ可
  - 5) は --no-krr でスキップ可（実行には concat_merged.csv が必要）

ログ:
  --log-dir を指定すると、各サブフォルダごとにログファイルへ stdout/stderr を保存。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Tuple


@dataclass(frozen=True)
class JobResult:
    dir_path: Path
    ok: bool
    returncode: int
    log_path: Optional[Path]
    error: Optional[str] = None


def _sanitize_name(p: Path) -> str:
    s = str(p)
    s = s.lstrip(os.sep)
    s = s.replace(os.sep, "__")
    s = s.replace(" ", "_")
    out = []
    for ch in s:
        if ch.isalnum() or ch in "._-":
            out.append(ch)
        else:
            out.append("_")
    return "".join(out) or "job"


def _run(
    args: List[str],
    *,
    cwd: Path,
    log_path: Optional[Path],
) -> Tuple[int, Optional[str]]:
    """
    Run subprocess. If log_path is provided, redirect stdout/stderr to it.
    Returns (returncode, error_message_if_failed).
    """
    try:
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("wb") as f:
                p = subprocess.run(args, cwd=str(cwd), stdout=f, stderr=subprocess.STDOUT)
        else:
            p = subprocess.run(args, cwd=str(cwd))
        if p.returncode != 0:
            return p.returncode, f"CommandFailed: rc={p.returncode} args={args}"
        return 0, None
    except FileNotFoundError as e:
        return 127, f"FileNotFound: {e}"
    except Exception as e:
        return 1, f"RunError: {e}"


def _has_raw_csv(dir_path: Path) -> bool:
    try:
        for p in dir_path.iterdir():
            if p.is_file() and p.name.endswith(".csv") and "raw" in p.name:
                return True
        return False
    except Exception:
        return False


def _has_concat_csv(dir_path: Path) -> bool:
    try:
        return (dir_path / "concat.csv").is_file()
    except Exception:
        return False


def _collect_subdirs(root: Path) -> List[Path]:
    subdirs = [p for p in root.iterdir() if p.is_dir()]
    subdirs.sort()
    return subdirs


def _run_one_dir(
    *,
    idx: int,
    total: int,
    dir_path: Path,
    scripts_dir: Path,
    python_bin: str,
    log_dir: Optional[Path],
    step_warmup: float,
    bias_scope: str,
    do_biascorr: bool,
    do_concat: bool,
    do_morph: bool,
    morph_cx: float,
    morph_cy: float,
    morph_rotor_radius_in: float,
) -> JobResult:
    prefix = f"[{idx}/{total}] "
    log_path = None
    if log_dir is not None:
        log_path = log_dir / f"{_sanitize_name(dir_path)}.log"
        print(f"{prefix}[postprocess] start: {dir_path} -> {log_path}", flush=True)
    else:
        print(f"{prefix}[postprocess] start: {dir_path}", flush=True)

    # 1) merge_front_back_bias.py (optional)
    if do_biascorr:
        corrected_dir = dir_path / "corrected"
        corrected_dir.mkdir(exist_ok=True)

        rc, err = _run(
            [
                python_bin,
                str(scripts_dir / "merge_front_back_bias.py"),
                "-k",
                "raw",
                "-d",
                ".",
                "--output-dir",
                "corrected/",
                "--dropna-mode",
                "none",
                "--step-warmup",
                str(step_warmup),
                "--bias-scope",
                bias_scope,
                "--param-rename",
                "dir:direction",
            ],
            cwd=dir_path,
            log_path=log_path,
        )
        if rc != 0:
            return JobResult(dir_path=dir_path, ok=False, returncode=rc, log_path=log_path, error=err)

    # 2) csv_concat_4.py -> concat.csv (optional)
    if do_concat:
        # If biascorr step is enabled, concat uses corrected/biascorr outputs.
        # Otherwise, concat uses raw CSVs under the directory.
        concat_keywords = "biascorr" if do_biascorr else "raw"
        concat_dir = "corrected/" if do_biascorr else "."

        rc, err = _run(
            [
                python_bin,
                str(scripts_dir / "csv_concat_4.py"),
                "-k",
                concat_keywords,
                "-d",
                concat_dir,
                "--output",
                "concat.csv",
                "--dropna-mode",
                "none",
                "--default-column",
                "slant_angle=0",
                "--default-column-mode",
                "missing",
            ],
            cwd=dir_path,
            log_path=log_path,
        )
        if rc != 0:
            return JobResult(dir_path=dir_path, ok=False, returncode=rc, log_path=log_path, error=err)

    # 3) add_calculated_columns_to_csv.py -> concat.csv (overwrite) (optional)
    if do_morph:
        concat_path = dir_path / "concat.csv"
        if not concat_path.is_file():
            return JobResult(
                dir_path=dir_path,
                ok=False,
                returncode=2,
                log_path=log_path,
                error="MissingInput: concat.csv not found (enable concat step or create concat.csv first)",
            )
        rc, err = _run(
            [
                python_bin,
                str(scripts_dir / "add_calculated_columns_to_csv.py"),
                "--input",
                "concat.csv",
                "--output",
                "concat.csv",
                "--cx",
                str(morph_cx),
                "--cy",
                str(morph_cy),
                "--rotor-radius-in",
                str(morph_rotor_radius_in),
            ],
            cwd=dir_path,
            log_path=log_path,
        )
        if rc != 0:
            return JobResult(dir_path=dir_path, ok=False, returncode=rc, log_path=log_path, error=err)

    print(f"{prefix}[postprocess] done : {dir_path}", flush=True)
    return JobResult(dir_path=dir_path, ok=True, returncode=0, log_path=log_path, error=None)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run postprocess over all subdirectories in parallel.")
    p.add_argument("-r", "--root", default=".", help="root directory (default: .)")
    p.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 1, help="parallel jobs (default: nproc)")
    p.add_argument("-l", "--log-dir", default=None, help="log output directory (optional)")
    p.add_argument("--python-bin", default="python3", help="python executable (default: python3)")

    # per-dir step parameters
    p.add_argument("--step-warmup", type=float, default=0.3, help="merge_front_back_bias.py --step-warmup (default: 0.3)")
    p.add_argument("--bias-scope", default="per-step", choices=["global", "per-step"], help="merge_front_back_bias.py --bias-scope")
    p.add_argument("--no-biascorr", action="store_true", help="skip merge_front_back_bias.py step (per-dir)")
    p.add_argument("--no-concat", action="store_true", help="skip csv_concat_4.py step (per-dir)")

    # morph/derived-columns step
    p.add_argument("--no-morph", action="store_true", help="disable add_calculated_columns_to_csv.py step")
    p.add_argument("--morph-cx", type=float, default=0.035, help="add_calculated_columns_to_csv.py --cx (default: 0.035)")
    p.add_argument("--morph-cy", type=float, default=0.035, help="add_calculated_columns_to_csv.py --cy (default: 0.035)")
    p.add_argument(
        "--morph-rotor-radius-in", type=float, default=3.5, help="add_calculated_columns_to_csv.py --rotor-radius-in (default: 3.5)"
    )

    # root-level steps
    p.add_argument("--no-krr", action="store_true", help="skip kernel_ridge_regression.py")
    p.add_argument("--no-merge", action="store_true", help="skip merge_csv.py (concat_merged.csv)")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    root = Path(args.root).expanduser().resolve()
    scripts_dir = Path(__file__).resolve().parent

    if not root.is_dir():
        print(f"RootNotFound: {root}", file=sys.stderr)
        return 2

    log_dir = Path(args.log_dir).expanduser().resolve() if args.log_dir else None
    if log_dir is not None:
        log_dir.mkdir(parents=True, exist_ok=True)

    do_biascorr = not bool(args.no_biascorr)
    do_concat = not bool(args.no_concat)
    do_morph = not bool(args.no_morph)

    do_any_per_dir = do_biascorr or do_concat or do_morph

    subdirs = _collect_subdirs(root)
    if do_any_per_dir:
        # Candidate detection depends on which inputs are required.
        # - If we only do morph (derived columns), we just need concat.csv.
        # - Otherwise, we need raw CSVs to generate corrected/concat outputs.
        if do_morph and not (do_biascorr or do_concat):
            candidates = [d for d in subdirs if _has_concat_csv(d)]
            if not candidates:
                print(f"No candidate subdirectories containing 'concat.csv' under: {root}", file=sys.stderr)
                return 1
        else:
            candidates = [d for d in subdirs if _has_raw_csv(d)]
            if not candidates:
                print(f"No candidate subdirectories containing '*raw*.csv' under: {root}", file=sys.stderr)
                return 1
    else:
        candidates = []

    total = len(candidates)
    if do_any_per_dir:
        print(f"[postprocess] candidates: {total} dirs (root={root})", flush=True)
    else:
        print(f"[postprocess] per-dir steps skipped (root={root})", flush=True)

    results: List[JobResult] = []
    failures: List[JobResult] = []

    if do_any_per_dir:
        # ThreadPool is fine because work is external subprocess
        with ThreadPoolExecutor(max_workers=max(1, int(args.jobs))) as ex:
            futs = []
            for i, d in enumerate(candidates, start=1):
                futs.append(
                    ex.submit(
                        _run_one_dir,
                        idx=i,
                        total=total,
                        dir_path=d,
                        scripts_dir=scripts_dir,
                        python_bin=str(args.python_bin),
                        log_dir=log_dir,
                        step_warmup=float(args.step_warmup),
                        bias_scope=str(args.bias_scope),
                        do_biascorr=do_biascorr,
                        do_concat=do_concat,
                        do_morph=do_morph,
                        morph_cx=float(args.morph_cx),
                        morph_cy=float(args.morph_cy),
                        morph_rotor_radius_in=float(args.morph_rotor_radius_in),
                    )
                )

            for fut in as_completed(futs):
                res = fut.result()
                results.append(res)
                if not res.ok:
                    failures.append(res)
                    msg = f"[postprocess] FAILED: {res.dir_path}"
                    if res.log_path:
                        msg += f" (log: {res.log_path})"
                    if res.error:
                        msg += f" :: {res.error}"
                    print(msg, file=sys.stderr, flush=True)

        ok_count = sum(1 for r in results if r.ok)
        print(f"[postprocess] per-dir done: ok={ok_count} fail={len(failures)}", flush=True)

    merged_path = root / "concat_merged.csv"
    merge_log = (log_dir / "merge_concat.log") if log_dir else None
    krr_log = (log_dir / "krr.log") if log_dir else None

    # root-level merge / krr
    if not args.no_merge:
        concat_files = sorted(root.rglob("concat.csv"))
        if not concat_files:
            print(f"No concat.csv found under: {root} (needed for merge step)", file=sys.stderr)
            return 1
        print(f"[postprocess] merging concat.csv -> {merged_path}", flush=True)
        rc, err = _run(
            [
                str(args.python_bin),
                str(scripts_dir / "merge_csv.py"),
                "--files",
                *[str(p) for p in concat_files],
                "--output",
                str(merged_path),
            ],
            cwd=root,
            log_path=merge_log,
        )
        if rc != 0:
            print(f"[postprocess] merge failed. {err}", file=sys.stderr)
            if merge_log:
                print(f"  log: {merge_log}", file=sys.stderr)
            return 2

    if not args.no_krr:
        if not merged_path.is_file():
            print(
                f"[postprocess] krr requires merged CSV but not found: {merged_path} "
                f"(run without --no-merge or create it beforehand)",
                file=sys.stderr,
            )
            return 2
        krr_out = root / "krr.png"
        print(f"[postprocess] training/plotting -> {krr_out}", flush=True)
        rc, err = _run(
            [
                str(args.python_bin),
                str(scripts_dir / "kernel_ridge_regression.py"),
                str(merged_path),
                "--output",
                str(krr_out),
                "--target",
                "normalized_moment",
                "--target-expr",
                "`torque_x` / `target_thrust` / `prop_spacing` * 100",
                "--do-grid-search",
                "--pairwise-heatmaps",
                "--std-heatmaps",
                "--plot-raw-fit",
                "--curve-x",
                "distance",
                "--row-group-by",
                "wall_spacing",
                "--col-group-by",
                "tilt_angle,fold_angle,slant_angle",
                "--normalize",
                "--features",
                "distance,tilt_angle,fold_angle,slant_angle,force_z,wall_spacing",
            ],
            cwd=root,
            log_path=krr_log,
        )
        if rc != 0:
            print(f"[postprocess] krr failed. {err}", file=sys.stderr)
            if krr_log:
                print(f"  log: {krr_log}", file=sys.stderr)
            return 2

    if failures:
        print("\n[postprocess] failures summary:", file=sys.stderr)
        for r in sorted(failures, key=lambda x: str(x.dir_path)):
            line = f" - {r.dir_path}"
            if r.log_path:
                line += f" (log: {r.log_path})"
            if r.error:
                line += f" :: {r.error}"
            print(line, file=sys.stderr)
        return 1

    print("[postprocess] all done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


