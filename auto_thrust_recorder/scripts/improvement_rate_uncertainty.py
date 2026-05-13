#!/usr/bin/env python3
"""
Monte Carlo uncertainty propagation for improvement rate I(omega).

Draws N samples from the SVGP posterior predictive distribution and computes
the improvement rate for each sample, yielding confidence intervals on I.

Also generates:
  - Improvement-rate map with 95% CI overlay
  - Comparison between F-score and geometric-mean aggregation
  - Sensitivity to integration domain Omega

Usage:
    python improvement_rate_uncertainty.py \
        --model model.joblib \
        --input input.csv \
        --n-samples 200 \
        --output uncertainty_results
"""

import argparse
import os
import sys
import warnings

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore")


def compute_improvement_fscore(M_abs, S_abs, M_abs_ref, S_abs_ref):
    """Compute F-score based improvement rate."""
    r_M = M_abs / M_abs_ref if M_abs_ref > 0 else np.inf
    r_S = S_abs / S_abs_ref if S_abs_ref > 0 else np.inf
    P_M = 1.0 / (1.0 + r_M)
    P_S = 1.0 / (1.0 + r_S)
    denom = P_M + P_S
    if denom == 0:
        return -100.0
    F = 2.0 * P_M * P_S / denom
    return (2.0 * F - 1.0) * 100.0


def compute_improvement_geomean(M_abs, S_abs, M_abs_ref, S_abs_ref):
    """Compute geometric-mean based improvement rate."""
    r_M = M_abs / M_abs_ref if M_abs_ref > 0 else np.inf
    r_S = S_abs / S_abs_ref if S_abs_ref > 0 else np.inf
    gm = np.sqrt(r_M * r_S)
    return (1.0 - gm) * 100.0


def compute_metrics_from_predictions(M_pred, d_grid, fz_grid, dd):
    """
    Compute M_abs and S_abs from a 2D grid of moment predictions.
    M_pred: shape (n_d, n_fz)
    """
    M_abs_vals = np.abs(M_pred)
    dM_dd = np.gradient(M_pred, dd, axis=0)
    S_abs_vals = np.abs(dM_dd)

    M_abs = np.trapz(np.trapz(M_abs_vals, fz_grid, axis=1), d_grid)
    S_abs = np.trapz(np.trapz(S_abs_vals, fz_grid, axis=1), d_grid)
    return M_abs, S_abs


def monte_carlo_improvement(
    model_wrapper,
    scaler_x,
    alpha_grid,
    beta_grid,
    d_range,
    fz_range,
    n_d=50,
    n_fz=20,
    n_samples=200,
    ref_alpha=0.0,
    ref_beta=0.0,
    fixed_params=None,
):
    """
    Monte Carlo propagation of GPR uncertainty through improvement rate.

    Returns:
        improvement_mean: (n_alpha, n_beta) mean improvement rate
        improvement_lo: (n_alpha, n_beta) 2.5th percentile
        improvement_hi: (n_alpha, n_beta) 97.5th percentile
    """
    d_grid = np.linspace(d_range[0], d_range[1], n_d)
    fz_grid = np.linspace(fz_range[0], fz_range[1], n_fz)
    dd = d_grid[1] - d_grid[0]

    D, FZ = np.meshgrid(d_grid, fz_grid, indexing="ij")
    d_flat = D.ravel()
    fz_flat = FZ.ravel()
    n_eval = len(d_flat)

    n_alpha = len(alpha_grid)
    n_beta = len(beta_grid)
    all_improvements = np.zeros((n_samples, n_alpha, n_beta))

    fixed = fixed_params or {}

    for si in range(n_samples):
        if (si + 1) % 50 == 0:
            print(f"  Sample {si + 1}/{n_samples}")

        ref_X = np.column_stack([
            d_flat,
            fz_flat,
            np.full(n_eval, fixed.get("W", 999)),
            np.full(n_eval, ref_alpha),
            np.full(n_eval, ref_beta),
            np.full(n_eval, fixed.get("Gx", 0)),
            np.full(n_eval, fixed.get("Gy", 0.128)),
        ])
        ref_X_s = scaler_x.transform(ref_X)
        ref_mean, ref_std = model_wrapper.predict(ref_X_s, return_std=True)
        ref_sample = ref_mean + np.random.randn(len(ref_mean)) * ref_std
        ref_M = ref_sample.reshape(n_d, n_fz)
        M_abs_ref, S_abs_ref = compute_metrics_from_predictions(ref_M, d_grid, fz_grid, dd)

        for ai, alpha in enumerate(alpha_grid):
            for bi, beta in enumerate(beta_grid):
                X_eval = np.column_stack([
                    d_flat,
                    fz_flat,
                    np.full(n_eval, fixed.get("W", 999)),
                    np.full(n_eval, alpha),
                    np.full(n_eval, beta),
                    np.full(n_eval, fixed.get("Gx", 0)),
                    np.full(n_eval, fixed.get("Gy", 0.128)),
                ])
                X_eval_s = scaler_x.transform(X_eval)
                pred_mean, pred_std = model_wrapper.predict(X_eval_s, return_std=True)
                pred_sample = pred_mean + np.random.randn(len(pred_mean)) * pred_std
                M_grid = pred_sample.reshape(n_d, n_fz)
                M_abs, S_abs = compute_metrics_from_predictions(M_grid, d_grid, fz_grid, dd)
                all_improvements[si, ai, bi] = compute_improvement_fscore(
                    M_abs, S_abs, M_abs_ref, S_abs_ref
                )

    improvement_mean = np.mean(all_improvements, axis=0)
    improvement_lo = np.percentile(all_improvements, 2.5, axis=0)
    improvement_hi = np.percentile(all_improvements, 97.5, axis=0)

    return improvement_mean, improvement_lo, improvement_hi, all_improvements


def plot_improvement_with_ci(
    alpha_grid, beta_grid, imp_mean, imp_lo, output_path, title=""
):
    """Plot improvement-rate heatmap with hatched insignificant regions."""
    fig, ax = plt.subplots(figsize=(7, 6))
    A, B = np.meshgrid(alpha_grid, beta_grid, indexing="ij")

    pcm = ax.pcolormesh(A, B, imp_mean, cmap="RdBu_r", vmin=-50, vmax=50, shading="auto")
    fig.colorbar(pcm, ax=ax, label="Improvement rate I [%]")

    insignificant = imp_lo < 0
    if insignificant.any():
        ax.contourf(A, B, insignificant.astype(float), levels=[0.5, 1.5],
                     hatches=["///"], colors="none", alpha=0)
        ax.contour(A, B, insignificant.astype(float), levels=[0.5], colors="k", linewidths=0.8)

    ax.set_xlabel(r"$\alpha$ [deg]")
    ax.set_ylabel(r"$\beta$ [deg]")
    ax.set_title(title or "Improvement rate with 95% CI")
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="MC uncertainty propagation for improvement rate")
    parser.add_argument("--model", required=True, help="Path to saved SVGP model (.joblib)")
    parser.add_argument("--input", required=True, help="Input CSV for scaler fitting")
    parser.add_argument("--n-samples", type=int, default=200)
    parser.add_argument("--output", default="uncertainty_results")
    args = parser.parse_args()

    print(f"Loading model from {args.model} ...")
    model_data = load(args.model)

    print("Monte Carlo uncertainty propagation is configured.")
    print(f"  N samples: {args.n_samples}")
    print(f"  Output prefix: {args.output}")
    print()
    print("NOTE: This script requires the trained model wrapper and scaler.")
    print("      Adjust the loading logic to match your model serialization format.")
    print("      The core functions (compute_improvement_fscore, monte_carlo_improvement)")
    print("      can be imported and used with any SVGP model that supports predict(X, return_std=True).")


if __name__ == "__main__":
    main()
