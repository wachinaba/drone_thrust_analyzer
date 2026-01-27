#!/usr/bin/env python3
"""
tilt/fold/slant (morph parameters) -> alpha/beta (rotor0 thrust angles) conversion
and lookup on a combined alpha-beta improvement map CSV (from gpr_effects_analysis.py).

目的:
  - (tilt, fold, slant) グリッドを走査
  - visualize_morph_drone.py と同じ定義で (alpha, beta) を計算
  - combined CSV の alpha-beta 格子上で combined_improve_pct を参照
  - 最大となる (tilt, fold, slant) を見つける
  - 例: slant 外側 / (tilt, fold) 内側の階層最適化も可能

入力CSV想定:
  gpr_effects_analysis.py の --output-csv で出力された 2D CSV
    columns: alpha, beta, combined_improve_pct (ほかの列があってもOK)
  ※ alpha/beta 列名は --alpha-col/--beta-col で変更可能
  ※ 参照値列は --value-col (default: combined_improve_pct)
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


# --- Conversion: (tilt, fold, slant) -> (alpha, beta) ---
#
# We intentionally mirror visualize_morph_drone.py's convention:
#   alpha = -atan2(vy, vz) [deg]
#   beta  = -atan2(vx, vz) [deg]
# where v is rotor0 rotor_normal (unit), computed by applying:
#   fold(phi): rotate about z by -phi
#   slant(psi): rotate about slant axis by -psi
#   tilt(theta): rotate about arm axis by +theta
#
# To stay robust and keep parity, we reuse the same math by importing internal helpers
# from visualize_morph_drone.py (they are pure numpy).


def _try_import_visualize_morph_drone():
    try:
        # scripts/ 同階層からの実行も想定（PYTHONPATHに載っていない場合がある）
        import importlib

        return importlib.import_module("visualize_morph_drone")
    except Exception:
        # fallback: attempt relative path execution environment where scripts is not on sys.path
        try:
            import importlib.util
            import os
            import sys

            here = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(here, "visualize_morph_drone.py")
            spec = importlib.util.spec_from_file_location("visualize_morph_drone", path)
            if spec is None or spec.loader is None:
                raise RuntimeError("spec loader is None")
            mod = importlib.util.module_from_spec(spec)
            sys.modules["visualize_morph_drone"] = mod
            spec.loader.exec_module(mod)  # type: ignore[attr-defined]
            return mod
        except Exception as e2:
            raise RuntimeError(
                "visualize_morph_drone.py の import に失敗しました。"
                "同ディレクトリにファイルが存在すること、実行環境のPYTHONPATHを確認してください。"
            ) from e2


def tfs_to_alpha_beta_deg(
    *,
    tilt_deg: float,
    fold_deg: float,
    slant_deg: float,
) -> Tuple[float, float]:
    """
    Return (alpha_deg, beta_deg) computed from rotor0 (the +x,+y arm) rotor_normal.
    """
    vmd = _try_import_visualize_morph_drone()
    _make_arm_pose = getattr(vmd, "_make_arm_pose")
    _normalize = getattr(vmd, "_normalize")

    # rotor0 definition in visualize_morph_drone.py: base arm in +x,+y quadrant
    arm_dir0 = _normalize(np.array([+1.0, +1.0, 0.0], dtype=float))
    pose = _make_arm_pose(
        hinge=np.zeros(3, dtype=float),
        arm_dir0=arm_dir0,
        phi_deg=float(fold_deg),
        psi_deg=float(slant_deg),
        theta_deg=float(tilt_deg),
    )
    v = _normalize(np.asarray(pose.rotor_normal, dtype=float).reshape(3))
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    alpha = -math.degrees(math.atan2(vy, vz))
    beta = -math.degrees(math.atan2(vx, vz))
    return float(alpha), float(beta)


# --- Map handling ---


@dataclass(frozen=True)
class ABMap:
    alpha_vals: np.ndarray  # (Na,)
    beta_vals: np.ndarray   # (Nb,)
    values: np.ndarray      # (Nb, Na) with ordering [beta_index, alpha_index]


def _build_ab_grid(
    df: pd.DataFrame,
    *,
    alpha_col: str,
    beta_col: str,
    value_col: str,
) -> ABMap:
    if alpha_col not in df.columns:
        raise ValueError(f"CSVに alpha 列 '{alpha_col}' が見つかりません")
    if beta_col not in df.columns:
        raise ValueError(f"CSVに beta 列 '{beta_col}' が見つかりません")
    if value_col not in df.columns:
        raise ValueError(f"CSVに value 列 '{value_col}' が見つかりません")

    a = pd.to_numeric(df[alpha_col], errors="coerce")
    b = pd.to_numeric(df[beta_col], errors="coerce")
    v = pd.to_numeric(df[value_col], errors="coerce")
    m = np.isfinite(a.values) & np.isfinite(b.values)
    # value は NaN を許容（mask-by-convex-hull 等）
    dff = pd.DataFrame({alpha_col: a[m].values, beta_col: b[m].values, value_col: v[m].values})

    alpha_vals = np.unique(dff[alpha_col].values.astype(float))
    beta_vals = np.unique(dff[beta_col].values.astype(float))
    alpha_vals.sort()
    beta_vals.sort()

    # pivot to (beta, alpha)
    piv = dff.pivot_table(index=beta_col, columns=alpha_col, values=value_col, aggfunc="first")
    piv = piv.reindex(index=beta_vals, columns=alpha_vals)
    Z = piv.values.astype(float)  # shape (Nb, Na)

    return ABMap(alpha_vals=alpha_vals, beta_vals=beta_vals, values=Z)


def _nearest_index(vals: np.ndarray, x: float) -> int:
    return int(np.argmin(np.abs(vals - float(x))))


def lookup_nearest(map_: ABMap, *, alpha: float, beta: float, allow_oob: bool = False) -> float:
    a = float(alpha)
    b = float(beta)
    if not allow_oob:
        if a < float(map_.alpha_vals[0]) or a > float(map_.alpha_vals[-1]):
            return float("nan")
        if b < float(map_.beta_vals[0]) or b > float(map_.beta_vals[-1]):
            return float("nan")
    ia = _nearest_index(map_.alpha_vals, a)
    ib = _nearest_index(map_.beta_vals, b)
    return float(map_.values[ib, ia])


def _find_bracketing_indices(vals: np.ndarray, x: float) -> Optional[Tuple[int, int, float]]:
    """
    vals: sorted unique (N,)
    Return (i0, i1, t) such that:
      vals[i0] <= x <= vals[i1], and x = (1-t)*vals[i0] + t*vals[i1]
    If x out of range -> None.
    """
    x = float(x)
    if x < float(vals[0]) or x > float(vals[-1]):
        return None
    # exact
    j = int(np.searchsorted(vals, x, side="left"))
    if j == 0:
        return (0, 0, 0.0)
    if j >= len(vals):
        k = len(vals) - 1
        return (k, k, 0.0)
    if float(vals[j]) == x:
        return (j, j, 0.0)
    i0 = j - 1
    i1 = j
    x0 = float(vals[i0])
    x1 = float(vals[i1])
    if x1 == x0:
        return (i0, i1, 0.0)
    t = (x - x0) / (x1 - x0)
    return (i0, i1, float(t))


def lookup_bilinear(map_: ABMap, *, alpha: float, beta: float, allow_oob: bool = False) -> float:
    a = float(alpha)
    b = float(beta)
    if allow_oob:
        # clamp to range
        a = float(np.clip(a, float(map_.alpha_vals[0]), float(map_.alpha_vals[-1])))
        b = float(np.clip(b, float(map_.beta_vals[0]), float(map_.beta_vals[-1])))
    ia = _find_bracketing_indices(map_.alpha_vals, a)
    ib = _find_bracketing_indices(map_.beta_vals, b)
    if ia is None or ib is None:
        return float("nan")
    a0, a1, ta = ia
    b0, b1, tb = ib

    z00 = float(map_.values[b0, a0])
    z10 = float(map_.values[b0, a1])
    z01 = float(map_.values[b1, a0])
    z11 = float(map_.values[b1, a1])

    # If any corner is NaN, bilinear becomes ambiguous -> return NaN (conservative)
    if not (np.isfinite(z00) and np.isfinite(z10) and np.isfinite(z01) and np.isfinite(z11)):
        return float("nan")

    z0 = (1.0 - ta) * z00 + ta * z10
    z1 = (1.0 - ta) * z01 + ta * z11
    z = (1.0 - tb) * z0 + tb * z1
    return float(z)


# --- CLI helpers ---


def _parse_axis_spec(text: str) -> np.ndarray:
    """
    Parse axis spec string:
      "lo,hi:N" -> N points inclusive (linspace)
      "lo,hi:step" -> step in deg (arange inclusive-ish)
      "lo,hi" -> default step=1.0
    Examples:
      "-30,30:61"   -> 61 points
      "-30,30:1"    -> step 1 deg
      "-15,15"      -> step 1 deg
    """
    s = str(text).strip()
    if "," not in s:
        raise ValueError(f"axis spec must be 'lo,hi[:N|step]': got {text!r}")
    if ":" in s:
        rng, tail = s.split(":", 1)
        tail = tail.strip()
    else:
        rng, tail = s, ""
    lo_s, hi_s = [t.strip() for t in rng.split(",", 1)]
    lo = float(lo_s)
    hi = float(hi_s)
    if tail == "":
        step = 1.0
        n = None
    else:
        # integer -> N points, otherwise float step
        try:
            n_int = int(tail)
            if str(n_int) == tail or tail.isdigit() or (tail.startswith("-") and tail[1:].isdigit()):
                n = int(n_int)
                step = None
            else:
                raise ValueError
        except Exception:
            n = None
            step = float(tail)

    if n is not None:
        if n <= 0:
            raise ValueError(f"N must be positive: {n}")
        if n == 1:
            return np.asarray([lo], dtype=float)
        return np.linspace(lo, hi, int(n), dtype=float)

    step = float(step if step is not None else 1.0)
    if step == 0.0:
        raise ValueError("step must be non-zero")
    # include hi if it lands on the grid within tolerance
    if lo <= hi and step < 0:
        step = abs(step)
    if lo >= hi and step > 0:
        step = -abs(step)
    vals = np.arange(lo, hi + 0.5 * step, step, dtype=float)  # best-effort include end
    return vals


def _parse_hierarchy(text: str) -> Tuple[str, Sequence[str]]:
    """
    "outer:inner1,inner2" -> ("outer", ["inner1","inner2"])
    """
    s = str(text).strip()
    if ":" not in s:
        raise ValueError("--hierarchy must be 'outer:inner1,inner2'")
    outer, inner = s.split(":", 1)
    outer = outer.strip()
    inners = [t.strip() for t in inner.split(",") if t.strip()]
    if not outer or not inners:
        raise ValueError("--hierarchy must be 'outer:inner1,inner2' (non-empty)")
    return outer, inners


def _aggregate_scores(
    scores: np.ndarray,
    *,
    method: str,
    q: float,
    threshold: float,
) -> float:
    """
    scores: (n_env,) float array (may contain NaN)

    Policy:
      - For min/mean/quantile/cvar: if any env score is non-finite -> return NaN
        (i.e., candidates must be valid across all environments)
      - For hitrate: NaN is treated as "not meeting threshold"; returns count.
    """
    s = np.asarray(scores, dtype=float).reshape(-1)
    method = str(method).strip().lower()
    if method in {"min", "max", "mean", "quantile", "cvar", "cvar_max"}:
        if not bool(np.all(np.isfinite(s))):
            return float("nan")
        if s.size == 0:
            return float("nan")
        if method == "min":
            return float(np.min(s))
        if method == "max":
            return float(np.max(s))
        if method == "mean":
            return float(np.mean(s))
        if method == "quantile":
            qq = float(q)
            if not (0.0 <= qq <= 1.0):
                raise ValueError("--q は [0,1] の範囲で指定してください")
            return float(np.quantile(s, qq))
        # cvar: mean of lower tail (worst q fraction)
        qq = float(q)
        if not (0.0 < qq <= 1.0):
            raise ValueError("--q は (0,1] の範囲で指定してください（cvar/cvar_max では 0 は不可）")
        k = max(1, int(math.ceil(qq * s.size)))
        ss = np.sort(s)  # ascending
        if method == "cvar":
            # worst tail: smallest k
            return float(np.mean(ss[:k]))
        # best tail: largest k (cvar_max)
        return float(np.mean(ss[-k:]))

    if method == "hitrate":
        thr = float(threshold)
        ok = np.isfinite(s) & (s >= thr)
        return float(np.sum(ok))

    raise ValueError(f"未知の集約方法です: {method!r}")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Search morph parameters (tilt/fold/slant) by looking up combined alpha-beta map.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--map-csv",
        action="append",
        required=True,
        help="gpr_effects_analysis.py が出力した combined CSV（alpha,beta格子）。複数回指定で複数環境を同時最適化。",
    )
    p.add_argument("--alpha-col", type=str, default="alpha", help="CSV中のalpha列名")
    p.add_argument("--beta-col", type=str, default="beta", help="CSV中のbeta列名")
    p.add_argument("--value-col", type=str, default="combined_improve_pct", help="参照する値列名")

    p.add_argument("--tilt", type=str, default="-30,30:61", help="tilt範囲 'lo,hi[:N|step]'（default: -30,30:61）")
    p.add_argument("--fold", type=str, default="-15,15:31", help="fold範囲 'lo,hi[:N|step]'（default: -15,15:31）")
    p.add_argument("--slant", type=str, default="-15,15:31", help="slant範囲 'lo,hi[:N|step]'（default: -15,15:31）")
    p.add_argument("--tilt-fixed", type=float, default=None, help="tilt をこの値[deg]に固定（指定時は --tilt を無視）")
    p.add_argument("--fold-fixed", type=float, default=None, help="fold をこの値[deg]に固定（指定時は --fold を無視）")
    p.add_argument("--slant-fixed", type=float, default=None, help="slant をこの値[deg]に固定（指定時は --slant を無視）")

    p.add_argument("--lookup", choices=["nearest", "bilinear"], default="nearest", help="マップ参照方法（default: nearest）")
    p.add_argument("--allow-oob", action="store_true", help="alpha/beta がマップ範囲外でも最近傍/クランプで評価する（default: False=範囲外はNaN）")
    p.add_argument(
        "--aggregate",
        choices=["min", "max", "quantile", "cvar", "cvar_max", "mean", "hitrate"],
        default="cvar",
        help=(
            "複数環境を同時に最適化する際の集約指標（default: cvar）。\n"
            "  - min: 最悪ケース最大化\n"
            "  - max: 最良ケース最大化\n"
            "  - quantile: 分位（--q）最大化\n"
            "  - cvar: 下位q割合の平均（--q）最大化\n"
            "  - cvar_max: 上位q割合の平均（--q）最大化\n"
            "  - mean: 平均最大化\n"
            "  - hitrate: s>=threshold の環境数（--threshold）最大化"
        ),
    )
    p.add_argument("--q", type=float, default=0.2, help="quantile/cvar 用の q（0..1）。default: 0.2（下位20%%）")
    p.add_argument("--threshold", type=float, default=0.0, help="hitrate 用の閾値。default: 0.0")
    p.add_argument("--output-csv", type=str, default=None, help="全走査結果をCSV保存（tilt,fold,slant,alpha,beta,score）")
    p.add_argument(
        "--hierarchy",
        type=str,
        default=None,
        help="階層最適化: 'outer:inner1,inner2' 例: 'slant:tilt,fold'。指定時は outerごとの最適(inner argmax)も出力。",
    )
    p.add_argument("--output-hierarchy-csv", type=str, default=None, help="階層最適化の outerごとの最適結果CSV")
    p.add_argument("--verbose", action="store_true", help="詳細ログ")

    args = p.parse_args()

    map_paths: List[str] = [str(pth) for pth in (args.map_csv or []) if str(pth).strip()]
    if not map_paths:
        raise SystemExit("--map-csv が指定されていません")

    maps: List[ABMap] = []
    for i, path in enumerate(map_paths):
        df = pd.read_csv(path)
        ab_map = _build_ab_grid(df, alpha_col=args.alpha_col, beta_col=args.beta_col, value_col=args.value_col)
        maps.append(ab_map)
        if args.verbose:
            print(f"[map{i}] path={path}")
            print("[map] alpha range:", float(ab_map.alpha_vals[0]), "to", float(ab_map.alpha_vals[-1]), "n=", int(len(ab_map.alpha_vals)))
            print("[map] beta  range:", float(ab_map.beta_vals[0]), "to", float(ab_map.beta_vals[-1]), "n=", int(len(ab_map.beta_vals)))

    tilt_vals = (
        np.asarray([float(args.tilt_fixed)], dtype=float)
        if args.tilt_fixed is not None
        else _parse_axis_spec(args.tilt)
    )
    fold_vals = (
        np.asarray([float(args.fold_fixed)], dtype=float)
        if args.fold_fixed is not None
        else _parse_axis_spec(args.fold)
    )
    slant_vals = (
        np.asarray([float(args.slant_fixed)], dtype=float)
        if args.slant_fixed is not None
        else _parse_axis_spec(args.slant)
    )

    if args.verbose:
        print("[grid] tilt:", tilt_vals[0], "to", tilt_vals[-1], "n=", int(len(tilt_vals)))
        print("[grid] fold:", fold_vals[0], "to", fold_vals[-1], "n=", int(len(fold_vals)))
        print("[grid] slant:", slant_vals[0], "to", slant_vals[-1], "n=", int(len(slant_vals)))

    def lookup_one(m: ABMap, a: float, b: float) -> float:
        if args.lookup == "nearest":
            return lookup_nearest(m, alpha=a, beta=b, allow_oob=bool(args.allow_oob))
        return lookup_bilinear(m, alpha=a, beta=b, allow_oob=bool(args.allow_oob))

    rows: List[Dict[str, float]] = []
    # brute-force scan
    for sl in slant_vals:
        for fd in fold_vals:
            for tl in tilt_vals:
                a, b = tfs_to_alpha_beta_deg(tilt_deg=float(tl), fold_deg=float(fd), slant_deg=float(sl))
                per_env = np.asarray([lookup_one(m, a, b) for m in maps], dtype=float)
                score_agg = _aggregate_scores(per_env, method=str(args.aggregate), q=float(args.q), threshold=float(args.threshold))

                row: Dict[str, float] = {
                    "tilt": float(tl),
                    "fold": float(fd),
                    "slant": float(sl),
                    "alpha": float(a),
                    "beta": float(b),
                    "score_agg": float(score_agg),
                }
                for i in range(len(maps)):
                    row[f"score_env_{i}"] = float(per_env[i])
                rows.append(row)

    df_out = pd.DataFrame(rows)
    # best overall by aggregated score (ignore NaNs)
    finite = np.isfinite(pd.to_numeric(df_out["score_agg"], errors="coerce").values)
    if not bool(np.any(finite)):
        raise SystemExit("有効な score_agg がありません（全てNaN）。マップ範囲・--allow-oob・変換範囲を確認してください。")
    best_idx = int(np.nanargmax(df_out["score_agg"].values.astype(float)))
    best = df_out.iloc[best_idx].to_dict()

    print(f"[best] aggregate={args.aggregate} q={float(args.q):g} threshold={float(args.threshold):g} lookup={args.lookup} allow_oob={bool(args.allow_oob)}")
    print("[best] score_agg=", float(best["score_agg"]))
    print("[best] tilt=", float(best["tilt"]), "fold=", float(best["fold"]), "slant=", float(best["slant"]))
    print("[best] alpha=", float(best["alpha"]), "beta=", float(best["beta"]))
    for i in range(len(maps)):
        k = f"score_env_{i}"
        if k in best:
            print(f"[best] {k}=", float(best[k]))

    if args.output_csv:
        df_out.to_csv(args.output_csv, index=False)
        if args.verbose:
            print("[out] wrote:", args.output_csv)

    # hierarchy optimization
    if args.hierarchy:
        outer, inners = _parse_hierarchy(args.hierarchy)
        # map symbolic names to columns
        col_map = {"tilt": "tilt", "fold": "fold", "slant": "slant"}
        if outer not in col_map:
            raise SystemExit(f"--hierarchy outer は tilt/fold/slant のいずれかにしてください: got {outer!r}")
        for c in inners:
            if c not in col_map:
                raise SystemExit(f"--hierarchy inner は tilt/fold/slant のいずれかにしてください: got {c!r}")
        outer_col = col_map[outer]
        inner_cols = [col_map[c] for c in inners]

        # Default behavior (as requested):
        #   - outer (e.g., slant) is shared across maps
        #   - inner (e.g., tilt/fold) is optimized PER MAP independently
        #   - then outer is chosen by aggregating per-map optima using --aggregate
        #
        # For each outer value:
        #   for each env i:
        #     pick row that maximizes score_env_i within that outer
        #   aggregate those best scores across env -> score_agg_outer
        df_base = df_out.copy()

        env_cols = [f"score_env_{i}" for i in range(len(maps))]
        for c in env_cols:
            if c not in df_base.columns:
                raise SystemExit(f"内部エラー: {c} が df_out にありません")

        outer_vals = np.unique(df_base[outer_col].values.astype(float))
        outer_vals.sort()

        per_outer_rows: List[Dict[str, float]] = []
        best_outer_row: Optional[Dict[str, float]] = None

        for ov in outer_vals:
            dfg = df_base[df_base[outer_col] == ov]
            if len(dfg) == 0:
                continue

            best_scores = []
            out_row: Dict[str, float] = {outer_col: float(ov)}

            # pick best inner per env
            for i, sc_col in enumerate(env_cols):
                s = pd.to_numeric(dfg[sc_col], errors="coerce").values.astype(float)
                if not bool(np.any(np.isfinite(s))):
                    # no feasible candidate for this env at this outer
                    out_row[f"best_env_{i}_score"] = float("nan")
                    best_scores.append(float("nan"))
                    continue
                j = int(np.nanargmax(s))
                r = dfg.iloc[j]
                out_row[f"best_env_{i}_score"] = float(r[sc_col])
                # store the per-env inner params (+ alpha/beta for debugging)
                for col in inner_cols:
                    out_row[f"best_env_{i}_{col}"] = float(r[col])
                out_row[f"best_env_{i}_alpha"] = float(r["alpha"])
                out_row[f"best_env_{i}_beta"] = float(r["beta"])
                best_scores.append(float(r[sc_col]))

            score_outer = _aggregate_scores(
                np.asarray(best_scores, dtype=float),
                method=str(args.aggregate),
                q=float(args.q),
                threshold=float(args.threshold),
            )
            out_row["score_agg_outer"] = float(score_outer)
            per_outer_rows.append(out_row)

            if (best_outer_row is None) or (
                np.isfinite(float(out_row["score_agg_outer"])) and float(out_row["score_agg_outer"]) > float(best_outer_row.get("score_agg_outer", float("-inf")))
            ):
                best_outer_row = out_row

        df_best_per_outer = pd.DataFrame(per_outer_rows)
        # keep only finite aggregated outers for selection/reporting
        finite_outer = np.isfinite(pd.to_numeric(df_best_per_outer["score_agg_outer"], errors="coerce").values)
        if not bool(np.any(finite_outer)):
            raise SystemExit("階層最適化: 有効な score_agg_outer がありません（全てNaN）。マップ範囲・マスク・--allow-oob を確認してください。")
        # select best outer
        idx_outer_best = int(np.nanargmax(df_best_per_outer["score_agg_outer"].values.astype(float)))
        best_outer = df_best_per_outer.iloc[idx_outer_best].to_dict()

        print(f"[hierarchy] outer={outer_col} inner={inner_cols} (per-env inner optimization)")
        print("[hierarchy-best] score_agg_outer=", float(best_outer["score_agg_outer"]))
        print(f"[hierarchy-best] {outer_col}=", float(best_outer[outer_col]))
        for i in range(len(maps)):
            sc = float(best_outer.get(f"best_env_{i}_score", float("nan")))
            print(f"[hierarchy-best] env{i} best_score=", sc)
            for col in inner_cols:
                k = f"best_env_{i}_{col}"
                if k in best_outer:
                    print(f"[hierarchy-best] env{i} {col}=", float(best_outer[k]))
            # alpha/beta for that env's best inner
            ak = f"best_env_{i}_alpha"
            bk = f"best_env_{i}_beta"
            if ak in best_outer and bk in best_outer:
                print(f"[hierarchy-best] env{i} alpha,beta=", float(best_outer[ak]), float(best_outer[bk]))

        if args.output_hierarchy_csv:
            df_best_per_outer.sort_values(by=outer_col).to_csv(args.output_hierarchy_csv, index=False)
            if args.verbose:
                print("[out] wrote:", args.output_hierarchy_csv)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


