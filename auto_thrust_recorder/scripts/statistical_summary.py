#!/usr/bin/env python3
"""
Compute n per condition, baseline vs optimal paired comparison (Wilcoxon, Cohen's d, 95% CI),
and improvement rate by force level. Outputs JSON + Markdown.
Usage:
  python statistical_summary.py CSV [--baseline-tilt 0 --baseline-slant 0] [--output-dir DIR]
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy import stats

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


def parse_args():
    p = argparse.ArgumentParser(description="Statistical summary: n, Wilcoxon, Cohen's d, 95% CI")
    p.add_argument("csv", help="Path to merged CSV (e.g. merged_1w_2w.csv)")
    p.add_argument("--condition-cols", type=str, default="distance,target_thrust,wall_spacing,tilt_angle,slant_angle,fold_angle",
                   help="Comma-separated columns that define a condition (use target_thrust for 2w)")
    p.add_argument("--metric", type=str, default="normalized_moment", help="Metric to compare (e.g. normalized_moment)")
    p.add_argument("--baseline-tilt", type=float, default=0.0)
    p.add_argument("--baseline-slant", type=float, default=0.0)
    p.add_argument("--baseline-fold", type=float, default=0.0)
    p.add_argument("--pose-cols", type=str, default="tilt_angle,slant_angle",
                   help="Columns that define pose (baseline vs optimal)")
    p.add_argument("--group-cols", type=str, default="distance,target_thrust,wall_spacing",
                   help="Columns that define a group for pairing (use target_thrust not force_z for 2w)")
    p.add_argument("--output-dir", type=str, default=None, help="Directory for statistical_summary.json and .md")
    p.add_argument("--force-bins", type=int, default=5, help="Number of force_z bins for improvement-by-thrust")
    return p.parse_args()


def cohens_d(x, y):
    """Paired Cohen's d: (mean(x)-mean(y)) / pooled_std of differences."""
    d = np.asarray(x, dtype=float) - np.asarray(y, dtype=float)
    n = len(d)
    if n < 2:
        return float("nan")
    mean_d = np.nanmean(d)
    std_d = np.nanstd(d, ddof=1)
    if std_d <= 0:
        return 0.0
    return float(mean_d / std_d)


def main():
    args = parse_args()
    condition_cols = [c.strip() for c in args.condition_cols.split(",")]
    pose_cols = [c.strip() for c in args.pose_cols.split(",")]
    group_cols = [c.strip() for c in args.group_cols.split(",")]

    df = pd.read_csv(args.csv)
    for c in condition_cols + [args.metric]:
        if c not in df.columns:
            raise SystemExit(f"Column not found: {c}")
    df = df[condition_cols + [args.metric]].dropna()

    # Ensure numeric
    for c in group_cols + pose_cols + [args.metric]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=group_cols + pose_cols + [args.metric])

    result = {"csv": args.csv, "metric": args.metric, "n_total_rows": int(len(df))}

    # --- 1) Condition counts (n per condition) ---
    combo = df[condition_cols].drop_duplicates()
    result["n_unique_conditions"] = int(len(combo))
    result["unique_per_column"] = {c: int(df[c].nunique()) for c in condition_cols}

    # n (sample size) per condition: count rows per (condition_cols)
    n_per_cond = df.groupby(condition_cols, dropna=False).size()
    result["n_per_condition_min"] = int(n_per_cond.min()) if len(n_per_cond) else 0
    result["n_per_condition_max"] = int(n_per_cond.max()) if len(n_per_cond) else 0
    result["n_per_condition_mean"] = float(n_per_cond.mean()) if len(n_per_cond) else 0.0

    # --- 2) Aggregate per (group_cols + pose_cols): mean(metric) and count
    agg_cols = group_cols + pose_cols
    if "fold_angle" in df.columns and "fold_angle" not in agg_cols:
        agg_cols = group_cols + pose_cols + ["fold_angle"]
    agg_df = df.groupby([c for c in agg_cols if c in df.columns], dropna=False).agg(
        mean_metric=(args.metric, "mean"),
        count=(args.metric, "size"),
    ).reset_index()

    # Use absolute value of metric for "magnitude" comparison if metric can be negative
    if agg_df["mean_metric"].min() < 0:
        agg_df["abs_metric"] = agg_df["mean_metric"].abs()
    else:
        agg_df["abs_metric"] = agg_df["mean_metric"]

    # Baseline: filter by baseline pose
    baseline_mask = True
    if "tilt_angle" in agg_df.columns:
        baseline_mask = baseline_mask & (np.isclose(agg_df["tilt_angle"], args.baseline_tilt))
    if "slant_angle" in agg_df.columns:
        baseline_mask = baseline_mask & (np.isclose(agg_df["slant_angle"], args.baseline_slant))
    if "fold_angle" in agg_df.columns:
        baseline_mask = baseline_mask & (np.isclose(agg_df["fold_angle"], args.baseline_fold))

    baseline_df = agg_df.loc[baseline_mask].copy()
    baseline_df = baseline_df.rename(columns={"abs_metric": "baseline_abs", "mean_metric": "baseline_mean", "count": "baseline_n"})
    baseline_df = baseline_df[[c for c in group_cols if c in baseline_df.columns] + ["baseline_abs", "baseline_mean", "baseline_n"]]

    # Optimal: per group, take the row where abs_metric is minimum
    grp_cols = [c for c in group_cols if c in agg_df.columns]
    idx_min = agg_df.groupby(grp_cols, dropna=False)["abs_metric"].idxmin()
    opt_rows = agg_df.loc[idx_min].copy()
    opt_rows = opt_rows.rename(columns={"abs_metric": "optimal_abs", "mean_metric": "optimal_mean", "count": "optimal_n"})
    opt_rows = opt_rows[[c for c in group_cols if c in opt_rows.columns] + ["optimal_abs", "optimal_mean", "optimal_n"]]

    merged = baseline_df.merge(
        opt_rows,
        on=[c for c in group_cols if c in baseline_df.columns and c in opt_rows.columns],
        how="inner",
    )
    if merged.empty:
        print("No paired baseline/optimal groups; check baseline filters and group columns.", file=sys.stderr)
        result["paired_n"] = 0
        result["wilcoxon"] = None
        result["cohens_d"] = None
        result["ci95_difference"] = None
    else:
        result["paired_n"] = int(len(merged))
        baseline_vals = merged["baseline_abs"].values
        optimal_vals = merged["optimal_abs"].values
        # Improvement: (baseline - optimal) / baseline when baseline > 0
        with np.errstate(divide="ignore", invalid="ignore"):
            improvement = np.where(baseline_vals > 1e-12, (baseline_vals - optimal_vals) / baseline_vals, 0.0)
        improvement = improvement[np.isfinite(improvement)]
        result["improvement_rate_mean"] = float(np.mean(improvement)) if len(improvement) else None
        result["improvement_rate_sd"] = float(np.std(improvement)) if len(improvement) else None

        # Wilcoxon signed-rank (paired)
        try:
            stat, pval = stats.wilcoxon(baseline_vals, optimal_vals, alternative="greater")
            result["wilcoxon"] = {"statistic": float(stat), "p_value": float(pval)}
        except Exception as e:
            result["wilcoxon"] = {"error": str(e)}

        result["cohens_d"] = cohens_d(baseline_vals, optimal_vals)
        diff = baseline_vals - optimal_vals
        n = len(diff)
        if n >= 2:
            se = np.std(diff, ddof=1) / np.sqrt(n)
            t_crit = stats.t.ppf(0.975, n - 1)
            result["ci95_difference"] = [float(np.mean(diff) - t_crit * se), float(np.mean(diff) + t_crit * se)]
            result["difference_mean"] = float(np.mean(diff))
        else:
            result["ci95_difference"] = None
            result["difference_mean"] = float(np.mean(diff)) if n else None

    # --- 3) Improvement by thrust bin (target_thrust or force_z) ---
    thrust_col = "target_thrust" if "target_thrust" in merged.columns else "force_z"
    if thrust_col in df.columns and not merged.empty and thrust_col in merged.columns:
        merged = merged.copy()
        merged["improvement"] = np.where(
            merged["baseline_abs"] > 1e-12,
            (merged["baseline_abs"] - merged["optimal_abs"]) / merged["baseline_abs"],
            np.nan,
        )
        n_bins = min(args.force_bins, len(merged), int(merged[thrust_col].nunique()))
        if n_bins >= 2:
            force_bins = pd.qcut(merged[thrust_col], q=n_bins, duplicates="drop")
            by_force = merged.groupby(force_bins, observed=True)["improvement"].agg(["mean", "std", "count"]).reset_index()
            bin_col = by_force.columns[0]  # first column is the bin
            by_force["force_bin"] = by_force[bin_col].astype(str)
            result["improvement_by_force"] = by_force[["force_bin", "mean", "std", "count"]].to_dict(orient="records")
        else:
            result["improvement_by_force"] = []

    # Output
    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
        json_path = os.path.join(args.output_dir, "statistical_summary.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        print(f"Wrote {json_path}")

        md_path = os.path.join(args.output_dir, "statistical_summary.md")
        with open(md_path, "w", encoding="utf-8") as f:
            f.write("# Statistical Summary\n\n")
            f.write(f"- CSV: {result['csv']}\n")
            f.write(f"- Metric: {result['metric']}\n")
            f.write(f"- Total rows: {result['n_total_rows']}\n")
            f.write(f"- Unique conditions: {result['n_unique_conditions']}\n")
            f.write(f"- n per condition: min={result['n_per_condition_min']}, max={result['n_per_condition_max']}, mean={result['n_per_condition_mean']:.1f}\n\n")
            f.write("## Baseline vs optimal (paired)\n\n")
            f.write(f"- Paired n: {result['paired_n']}\n")
            if result.get("wilcoxon"):
                f.write(f"- Wilcoxon (baseline > optimal): stat={result['wilcoxon'].get('statistic')}, p={result['wilcoxon'].get('p_value')}\n")
            f.write(f"- Cohen's d: {result.get('cohens_d')}\n")
            if result.get("ci95_difference"):
                f.write(f"- 95% CI (difference): {result['ci95_difference']}\n")
            if result.get("improvement_rate_mean") is not None:
                f.write(f"- Improvement rate (mean ± sd): {result['improvement_rate_mean']:.4f} ± {result.get('improvement_rate_sd', 0):.4f}\n")
            if result.get("improvement_by_force"):
                f.write("\n## Improvement by force bin\n\n")
                for row in result["improvement_by_force"]:
                    f.write(f"- {row['force_bin']}: mean={row['mean']:.4f}, std={row['std']:.4f}, n={row['count']}\n")
        print(f"Wrote {md_path}")

    print(json.dumps(result, indent=2, ensure_ascii=False))
    return result


if __name__ == "__main__":
    main()
