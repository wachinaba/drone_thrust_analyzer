#!/usr/bin/env python3
"""
5-fold cross-validation for GPyTorch SVGP (gaussian_process_regression.py).
Reports mean +/- std of RMSE and R2 across folds.
Usage:
  python gpr_cross_validation.py CSV --features COL1,COL2,... --target COL [--k-folds 5] [--epochs 150] [--output-json path.json]
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


def parse_args():
    p = argparse.ArgumentParser(description="k-fold CV for SVGP")
    p.add_argument("csv", help="Path to merged CSV (e.g. merged_1w_2w.csv or merged_3w.csv)")
    p.add_argument("--features", type=str, required=True, help="Comma-separated feature columns")
    p.add_argument("--target", type=str, default="normalized_moment")
    p.add_argument("--k-folds", type=int, default=5)
    p.add_argument("--epochs", type=int, default=150, help="Epochs per fold")
    p.add_argument("--num-inducing", type=int, default=256)
    p.add_argument("--random-state", type=int, default=42)
    p.add_argument("--output-json", type=str, default=None, help="Write results to JSON")
    p.add_argument("--device", type=str, default="auto")
    return p.parse_args()


def main():
    args = parse_args()
    feature_cols = [c.strip() for c in args.features.split(",")]
    target_col = args.target

    df = pd.read_csv(args.csv)
    for c in feature_cols + [target_col]:
        if c not in df.columns:
            raise SystemExit(f"Column not found: {c}")
    df_clean = df[feature_cols + [target_col]].dropna()
    X = df_clean[feature_cols].values.astype(np.float32)
    y = df_clean[target_col].values.astype(np.float32).reshape(-1)

    n_samples = X.shape[0]
    if n_samples < 2 * args.k_folds:
        raise SystemExit(f"Too few samples ({n_samples}) for {args.k_folds} folds")

    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)

    # Minimal args for train_sparse_gpytorch
    class TrainArgs:
        alpha = 0.001
        anisotropic = True
        kernel = "rbf"
        matern_nu = 1.5
        num_inducing = min(args.num_inducing, n_samples // 2)
        epochs = args.epochs
        lr = 0.01
        batch_size = 1024
        random_state = args.random_state
        device = args.device
        log_every = 9999

    import gaussian_process_regression as gpr
    train_fn = gpr.train_sparse_gpytorch

    kf = KFold(n_splits=args.k_folds, shuffle=True, random_state=args.random_state)
    fold_rmse = []
    fold_r2 = []

    for fold_idx, (train_idx, test_idx) in enumerate(kf.split(X_scaled)):
        X_tr, X_te = X_scaled[train_idx], X_scaled[test_idx]
        y_tr, y_te = y[train_idx], y[test_idx]
        try:
            wrapper = train_fn(X_tr, y_tr, TrainArgs())
            y_pred = wrapper.predict(X_te, return_std=False)
            if hasattr(y_pred, "ravel"):
                y_pred = y_pred.ravel()
            rmse = np.sqrt(mean_squared_error(y_te, y_pred))
            r2 = r2_score(y_te, y_pred)
            fold_rmse.append(rmse)
            fold_r2.append(r2)
            print(f"Fold {fold_idx + 1}/{args.k_folds}: RMSE={rmse:.6f}, R2={r2:.6f}")
        except Exception as e:
            print(f"Fold {fold_idx + 1} failed: {e}", file=sys.stderr)
            fold_rmse.append(np.nan)
            fold_r2.append(np.nan)

    fold_rmse = np.array(fold_rmse)
    fold_r2 = np.array(fold_r2)
    valid = np.isfinite(fold_rmse)
    if not np.any(valid):
        raise SystemExit("All folds failed.")
    mean_rmse = np.mean(fold_rmse[valid])
    std_rmse = np.std(fold_rmse[valid])
    mean_r2 = np.mean(fold_r2[valid])
    std_r2 = np.std(fold_r2[valid])

    print("\n=== Cross-validation summary ===")
    print(f"  RMSE: {mean_rmse:.6f} +/- {std_rmse:.6f}")
    print(f"  R2:   {mean_r2:.6f} +/- {std_r2:.6f}")

    out = {
        "csv": args.csv,
        "target": target_col,
        "k_folds": args.k_folds,
        "n_samples": int(n_samples),
        "cv_rmse_mean": float(mean_rmse),
        "cv_rmse_std": float(std_rmse),
        "cv_r2_mean": float(mean_r2),
        "cv_r2_std": float(std_r2),
        "fold_rmse": [float(x) for x in fold_rmse],
        "fold_r2": [float(x) for x in fold_r2],
    }
    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)
        print(f"Wrote {args.output_json}")
    return out


if __name__ == "__main__":
    main()
