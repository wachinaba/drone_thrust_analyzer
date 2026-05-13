#!/usr/bin/env python3
"""
Compare Linear Regression, KRR (RBF), and SVGP on the same train/test split.
Outputs a CSV with columns: dataset, target, model, rmse, r2, mae.
Usage:
  python model_comparison.py CSV --features COL1,COL2,... --target COL [--output-csv path.csv]
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.kernel_ridge import KernelRidge
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


def parse_args():
    p = argparse.ArgumentParser(description="Compare Linear, KRR, SVGP on same split")
    p.add_argument("csv", help="Path to CSV")
    p.add_argument("--features", type=str, required=True)
    p.add_argument("--target", type=str, default="normalized_moment")
    p.add_argument("--test-size", type=float, default=0.2)
    p.add_argument("--random-state", type=int, default=42)
    p.add_argument("--output-csv", type=str, default=None)
    p.add_argument("--krr-alpha", type=float, default=0.01)
    p.add_argument("--krr-gamma", type=float, default=0.5)
    p.add_argument("--svgp-epochs", type=int, default=150)
    p.add_argument("--svgp-num-inducing", type=int, default=256)
    p.add_argument("--append", action="store_true", help="Append rows to existing --output-csv")
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

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=args.test_size, random_state=args.random_state
    )
    scaler = StandardScaler()
    X_train_s = scaler.fit_transform(X_train)
    X_test_s = scaler.transform(X_test)

    dataset_name = os.path.splitext(os.path.basename(args.csv))[0]
    rows = []

    def add_metrics(model_name, y_pred):
        y_pred = np.asarray(y_pred).ravel()
        rmse = np.sqrt(mean_squared_error(y_test, y_pred))
        r2 = r2_score(y_test, y_pred)
        mae = mean_absolute_error(y_test, y_pred)
        rows.append({
            "dataset": dataset_name,
            "target": target_col,
            "model": model_name,
            "rmse": rmse,
            "r2": r2,
            "mae": mae,
        })
        print(f"  {model_name}: RMSE={rmse:.6f}, R2={r2:.6f}, MAE={mae:.6f}")

    # Linear Regression
    print("Fitting Linear Regression...")
    lr = LinearRegression().fit(X_train_s, y_train)
    add_metrics("LinearRegression", lr.predict(X_test_s))

    # KRR (RBF)
    print("Fitting KRR (RBF)...")
    krr = KernelRidge(kernel="rbf", alpha=args.krr_alpha, gamma=args.krr_gamma)
    krr.fit(X_train_s, y_train)
    add_metrics("KRR_rbf", krr.predict(X_test_s))

    # SVGP
    print("Fitting SVGP...")
    import gaussian_process_regression as gpr

    class TrainArgs:
        alpha = 0.001
        anisotropic = True
        kernel = "rbf"
        matern_nu = 1.5
        num_inducing = min(args.svgp_num_inducing, len(X_train_s) // 2)
        epochs = args.svgp_epochs
        lr = 0.01
        batch_size = 1024
        random_state = args.random_state
        device = "auto"
        log_every = 9999

    wrapper = gpr.train_sparse_gpytorch(X_train_s, y_train, TrainArgs())
    y_pred_svgp = wrapper.predict(X_test_s, return_std=False)
    add_metrics("SVGP", y_pred_svgp)

    out_df = pd.DataFrame(rows)
    print("\n=== Summary ===")
    print(out_df.to_string(index=False))

    if args.output_csv:
        if args.append and os.path.isfile(args.output_csv):
            existing = pd.read_csv(args.output_csv, encoding="utf-8")
            out_df = pd.concat([existing, out_df], ignore_index=True)
        out_df.to_csv(args.output_csv, index=False, encoding="utf-8")
        print(f"Wrote {args.output_csv}")
    return out_df


if __name__ == "__main__":
    main()
