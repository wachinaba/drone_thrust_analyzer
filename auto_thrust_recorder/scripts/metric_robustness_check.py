#!/usr/bin/env python3
"""
Robustness check for the improvement-rate metric.

Generates Supplementary Figures S1 and S2:
  S1: Comparison of F-score vs. geometric-mean aggregation
  S2: Sensitivity to integration domain Omega

Usage:
    python metric_robustness_check.py \
        --model model.joblib \
        --input input.csv \
        --output robustness_check
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


def compute_improvement_fscore(r_M, r_S):
    """F-score (harmonic mean) based improvement rate."""
    P_M = 1.0 / (1.0 + r_M)
    P_S = 1.0 / (1.0 + r_S)
    denom = P_M + P_S
    F = np.where(denom > 0, 2.0 * P_M * P_S / denom, 0.0)
    return (2.0 * F - 1.0) * 100.0


def compute_improvement_geomean(r_M, r_S):
    """Geometric-mean based improvement rate."""
    gm = np.sqrt(r_M * r_S)
    return (1.0 - gm) * 100.0


def plot_metric_comparison(
    alpha_grid, beta_grid, imp_fscore, imp_geomean, output_path
):
    """
    Supplementary Figure S1: Side-by-side comparison of two aggregation methods.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    A, B = np.meshgrid(alpha_grid, beta_grid, indexing="ij")

    vmin, vmax = -50, 50

    pcm0 = axes[0].pcolormesh(A, B, imp_fscore, cmap="RdBu_r", vmin=vmin, vmax=vmax, shading="auto")
    axes[0].set_title("F-score (harmonic mean)")
    axes[0].set_xlabel(r"$\alpha$ [deg]")
    axes[0].set_ylabel(r"$\beta$ [deg]")
    fig.colorbar(pcm0, ax=axes[0], label="I [%]")

    pcm1 = axes[1].pcolormesh(A, B, imp_geomean, cmap="RdBu_r", vmin=vmin, vmax=vmax, shading="auto")
    axes[1].set_title("Geometric mean")
    axes[1].set_xlabel(r"$\alpha$ [deg]")
    axes[1].set_ylabel(r"$\beta$ [deg]")
    fig.colorbar(pcm1, ax=axes[1], label="I [%]")

    fig.suptitle("Supplementary Fig. S1: Metric aggregation comparison", fontsize=12)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {output_path}")


def plot_domain_sensitivity(
    alpha_grid, beta_grid, imp_full, imp_inner, imp_near, output_path
):
    """
    Supplementary Figure S2: Sensitivity to integration domain Omega.
    """
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    A, B = np.meshgrid(alpha_grid, beta_grid, indexing="ij")

    vmin, vmax = -50, 50
    titles = [
        r"Full range: $\Omega_{\mathrm{full}}$",
        r"Inner 80%: $\Omega_{80}$",
        r"Near-wall ($d \leq 2R$): $\Omega_{\mathrm{near}}$",
    ]
    data = [imp_full, imp_inner, imp_near]

    for ax, d, t in zip(axes, data, titles):
        pcm = ax.pcolormesh(A, B, d, cmap="RdBu_r", vmin=vmin, vmax=vmax, shading="auto")
        ax.set_title(t)
        ax.set_xlabel(r"$\alpha$ [deg]")
        ax.set_ylabel(r"$\beta$ [deg]")
        fig.colorbar(pcm, ax=ax, label="I [%]")

    fig.suptitle("Supplementary Fig. S2: Integration domain sensitivity", fontsize=12)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
    print(f"  Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Metric robustness check")
    parser.add_argument("--model", help="Path to saved SVGP model (.joblib)")
    parser.add_argument("--input", help="Input CSV")
    parser.add_argument("--output", default="robustness_check", help="Output prefix")
    args = parser.parse_args()

    print("Metric robustness check script configured.")
    print(f"Output prefix: {args.output}")
    print()
    print("This script provides the plotting functions for Supplementary Figs S1 and S2.")
    print("Integration with the existing gpr_effects_analysis.py pipeline:")
    print("  1. Load the trained SVGP model")
    print("  2. Compute r_M and r_S on the (alpha, beta) grid")
    print("  3. Call compute_improvement_fscore() and compute_improvement_geomean()")
    print("  4. Call plot_metric_comparison() for Supp. Fig. S1")
    print("  5. Repeat metric computation with different Omega domains")
    print("  6. Call plot_domain_sensitivity() for Supp. Fig. S2")


if __name__ == "__main__":
    main()
