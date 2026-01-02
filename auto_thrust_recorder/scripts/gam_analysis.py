#!/usr/bin/env python3
"""
GAM (Generalized Additive Model) による平均/分散の解釈的解析

目的:
  - 目的変数（例: torque_x, force_y）を特徴量で説明（非線形はスプライン）
  - 同時に variance_*（例: variance_torque_x）も説明し、"ばらつき"に効く要因も把握
  - 各特徴量の重要度を「その項を落としたときの当てはまり劣化（ΔAIC/ΔpseudoR2）」で出力
  - 部分効果（partial dependence）のプロットを保存

注意:
  - 200Hzの生波形(400サンプル等)をそのまま独立観測として扱うと有意性が壊れます。
    本スクリプトは "条件×反復" 単位の集約済み行（target と variance_* が同一行）を想定します。
  - `--use-weights` は「目的変数が生波形の平均」で、かつ `variance_*` がその生波形分散のときにのみ
    近似的に妥当です（自己相関で有効サンプル数が下がる点は別途留保）。
"""

import argparse
import os
import sys
import warnings

import numpy as np
import pandas as pd

import matplotlib
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore")

try:
    from pygam import LinearGAM, s
except Exception as e:
    raise RuntimeError(
        "pygam を import できません。仮想環境を有効化して `pip install pygam` を実行してください。"
        f" (error: {e})"
    ) from e


def parse_tf(v):
    if isinstance(v, bool):
        return v
    if v is None:
        raise argparse.ArgumentTypeError("t/f を指定してください")
    s_ = str(v).strip().lower()
    if s_ in ["t", "true", "1", "yes", "y", "on"]:
        return True
    if s_ in ["f", "false", "0", "no", "n", "off"]:
        return False
    raise argparse.ArgumentTypeError(f"t/f を指定してください（入力='{v}'）")


def _ensure_outdir(path: str):
    if not path:
        return
    os.makedirs(path, exist_ok=True)


def _pick_target_thrust_column(df: pd.DataFrame, preferred: str | None):
    """
    プロジェクトの命名ゆれを吸収:
      - target_thrust が無ければ force_z をフォールバック
    """
    if preferred and preferred in df.columns:
        return preferred
    if "target_thrust" in df.columns:
        return "target_thrust"
    if "force_z" in df.columns:
        return "force_z"
    return preferred  # may be None; caller will validate


def _coerce_numeric(df: pd.DataFrame, cols: list[str]):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def _dropna_rows(df: pd.DataFrame, cols: list[str], label: str):
    before = len(df)
    out = df.dropna(subset=cols).reset_index(drop=True)
    after = len(out)
    dropped = before - after
    if dropped > 0:
        print(f"[{label}] dropna: {dropped} 行を除外（必要列={cols}）")
    if after == 0:
        raise ValueError(f"[{label}] 有効な行が 0 です。列名/欠損をご確認ください。")
    return out


def build_additive_gam(n_features: int, n_splines: int, spline_order: int, lam: float):
    """
    すべての特徴量に s() を当てた加法モデルを作る。
    """
    if n_features <= 0:
        raise ValueError("n_features must be > 0")
    terms = s(0, n_splines=n_splines, spline_order=spline_order)
    for i in range(1, n_features):
        terms = terms + s(i, n_splines=n_splines, spline_order=spline_order)
    # lam は各項共通の初期値（gridsearchしない場合の安定化用）
    return LinearGAM(terms, lam=float(lam))


def fit_gam(
    X: np.ndarray,
    y: np.ndarray,
    feature_names: list[str],
    *,
    n_splines: int,
    spline_order: int,
    lam: float,
    gridsearch: bool,
    weights: np.ndarray | None,
):
    gam = build_additive_gam(
        n_features=X.shape[1],
        n_splines=n_splines,
        spline_order=spline_order,
        lam=lam,
    )
    if gridsearch:
        # lam の探索：広すぎると時間が増えるので控えめに
        # ここは必要に応じて増やせます。
        lams = np.logspace(-3, 3, 9)
        gam.gridsearch(X, y, weights=weights, lam=lams, progress=False)
    else:
        gam.fit(X, y, weights=weights)
    stats = getattr(gam, "statistics_", {}) or {}
    pseudo_r2 = stats.get("pseudo_r2", {}).get("explained_deviance", None)
    aic = stats.get("AIC", None)
    print(f"[fit] n={len(y)}, pseudoR2(explained_deviance)={pseudo_r2}, AIC={aic}")
    print(f"[fit] lam={getattr(gam, 'lam', None)}")
    # summaryは出力が長いのでファイルに回す想定
    return gam


def _gam_metrics(gam: LinearGAM):
    st = getattr(gam, "statistics_", {}) or {}
    pseudo_r2 = st.get("pseudo_r2", {}).get("explained_deviance", np.nan)
    aic = st.get("AIC", np.nan)
    gcv = st.get("GCV", np.nan)
    return float(pseudo_r2) if pseudo_r2 is not None else np.nan, float(aic), float(gcv)


def compute_importance_drop_term(
    full_gam: LinearGAM,
    X: np.ndarray,
    y: np.ndarray,
    feature_names: list[str],
    *,
    n_splines: int,
    spline_order: int,
    weights: np.ndarray | None,
):
    """
    各特徴量を1つ落とした reduced model を作り、当てはまり劣化を計算。
    - reduced は full の lam を流用（term数が変わるので対応分だけ抜く）
    """
    full_p2, full_aic, full_gcv = _gam_metrics(full_gam)
    full_lam = np.array(getattr(full_gam, "lam", []), dtype=float).reshape(-1)

    rows = []
    for drop_idx, drop_name in enumerate(feature_names):
        keep_cols = [i for i in range(len(feature_names)) if i != drop_idx]
        Xr = X[:, keep_cols]
        # lam も対応する分だけ抜く（pygam は term毎に lam を持つ）
        lam_r = None
        if full_lam.size >= len(feature_names):
            lam_r = full_lam[keep_cols]
        # reduced fit
        gr = build_additive_gam(
            n_features=Xr.shape[1],
            n_splines=n_splines,
            spline_order=spline_order,
            lam=float(np.median(lam_r)) if lam_r is not None and lam_r.size > 0 else 0.6,
        )
        # できる限り full の lam を再現（term数が一致する場合のみ）
        if lam_r is not None and lam_r.size == Xr.shape[1]:
            try:
                gr.lam = lam_r
            except Exception:
                pass
        gr.fit(Xr, y, weights=weights)

        p2, aic, gcv = _gam_metrics(gr)
        rows.append(
            {
                "dropped_feature": drop_name,
                "full_pseudo_r2": full_p2,
                "reduced_pseudo_r2": p2,
                "delta_pseudo_r2": (full_p2 - p2) if np.isfinite(full_p2) and np.isfinite(p2) else np.nan,
                "full_aic": full_aic,
                "reduced_aic": aic,
                "delta_aic": (aic - full_aic) if np.isfinite(full_aic) and np.isfinite(aic) else np.nan,
                "full_gcv": full_gcv,
                "reduced_gcv": gcv,
                "delta_gcv": (gcv - full_gcv) if np.isfinite(full_gcv) and np.isfinite(gcv) else np.nan,
            }
        )

    df_imp = pd.DataFrame(rows)
    # 重要度としては delta_aic が大きいほど重要、delta_pseudo_r2 が大きいほど重要
    try:
        df_imp = df_imp.sort_values(by=["delta_aic", "delta_pseudo_r2"], ascending=[False, False]).reset_index(drop=True)
    except Exception:
        pass
    return df_imp


def plot_partial_effects(
    gam: LinearGAM,
    feature_names: list[str],
    *,
    outdir: str,
    prefix: str,
    title_prefix: str,
    xlabel_overrides: dict[str, str] | None = None,
):
    _ensure_outdir(outdir)
    xlabel_overrides = xlabel_overrides or {}
    for i, name in enumerate(feature_names):
        try:
            XX = gam.generate_X_grid(term=i)
            pdep = gam.partial_dependence(term=i, X=XX)
            ci = gam.partial_dependence(term=i, X=XX, width=0.95)
        except Exception as e:
            print(f"[plot] '{name}' の部分効果計算に失敗: {e}")
            continue

        xs = XX[:, i]
        plt.figure(figsize=(8, 5))
        plt.plot(xs, pdep, color="C0", lw=2.0)
        try:
            plt.fill_between(xs, ci[0], ci[1], color="C0", alpha=0.2)
        except Exception:
            pass
        plt.grid(True, alpha=0.3)
        plt.title(f"{title_prefix}: partial effect of {name}")
        plt.xlabel(xlabel_overrides.get(name, name))
        plt.ylabel("partial effect")
        out = os.path.join(outdir, f"{prefix}_{title_prefix}_effect_{name}.png")
        plt.tight_layout()
        plt.savefig(out, dpi=200, bbox_inches="tight")
        plt.close()


def main():
    parser = argparse.ArgumentParser(description="GAMで平均/分散を解析し、重要度と部分効果を出力します。")
    parser.add_argument("csv_file", help="入力CSVファイル")
    parser.add_argument("--target", type=str, default="torque_x", help="目的変数列（例: torque_x, force_y）")
    parser.add_argument(
        "--variance-col",
        type=str,
        default=None,
        help="分散列（例: variance_torque_x）。未指定なら variance_{target} を自動推定します。",
    )
    parser.add_argument(
        "--features",
        type=str,
        default="distance,target_thrust,wall_spacing,tilt_angle,fold_angle,slant_angle",
        help="使用する特徴量列（カンマ区切り）",
    )
    parser.add_argument("--outdir", type=str, default="gam_out", help="出力ディレクトリ（デフォルト: gam_out）")
    parser.add_argument("--prefix", type=str, default=None, help="出力ファイル接頭辞（未指定なら gam_{target}）")

    parser.add_argument("--gridsearch", type=parse_tf, default=True, metavar="{t,f}", help="lamを簡易探索（デフォルト: t）")
    parser.add_argument("--n-splines", type=int, default=20, help="各s()のスプライン数（デフォルト: 20）")
    parser.add_argument("--spline-order", type=int, default=3, help="スプライン次数（デフォルト: 3）")
    parser.add_argument("--lam", type=float, default=0.6, help="gridsearchしない場合の正則化強度（デフォルト: 0.6）")

    parser.add_argument(
        "--use-weights",
        type=parse_tf,
        default=False,
        metavar="{t,f}",
        help="平均モデルで重み付けを使う（デフォルト: f）。variance列から近似的にweights=1/(var/n_eff)を作成。",
    )
    parser.add_argument("--n-effective", type=float, default=400.0, help="有効サンプル数の近似（デフォルト: 400）")
    parser.add_argument("--weight-eps", type=float, default=1e-12, help="重み計算のeps（デフォルト: 1e-12）")

    parser.add_argument("--log-eps", type=float, default=1e-12, help="log(variance) のeps（デフォルト: 1e-12）")

    args = parser.parse_args()

    # headless / output前提
    if not os.environ.get("DISPLAY"):
        try:
            matplotlib.use("Agg", force=True)
        except Exception:
            pass

    csv_file = args.csv_file
    if not os.path.exists(csv_file):
        raise FileNotFoundError(f"CSVが見つかりません: {csv_file}")

    df = pd.read_csv(csv_file)
    if df.empty:
        raise ValueError("CSVが空です")

    target = str(args.target).strip()
    if not target:
        raise ValueError("--target が空です")

    # features
    features_raw = [c.strip() for c in str(args.features).split(",") if c.strip()]
    if not features_raw:
        raise ValueError("--features が空です")

    # 命名ゆれ吸収: target_thrust / force_z
    fixed_features = []
    for c in features_raw:
        if c in ["target_thrust", "force_z"]:
            fixed_features.append(_pick_target_thrust_column(df, c))
        else:
            fixed_features.append(c)
    # 重複除去しつつ順序維持
    seen = set()
    feature_cols = []
    for c in fixed_features:
        if c and (c not in seen):
            feature_cols.append(c)
            seen.add(c)

    # variance col
    variance_col = args.variance_col.strip() if args.variance_col else None
    if not variance_col:
        variance_col = f"variance_{target}"

    # validate columns
    missing = [c for c in ([target, variance_col] + feature_cols) if c not in df.columns]
    if missing:
        raise ValueError(
            "必要な列がCSVに存在しません: "
            f"{missing}\n"
            f"CSV columns: {list(df.columns)}"
        )

    # numeric conversion
    df = _coerce_numeric(df, [target, variance_col] + feature_cols)
    df = _dropna_rows(df, [target, variance_col] + feature_cols, label="preprocess")

    # matrices
    X = df[feature_cols].to_numpy(dtype=float)
    y_mean = df[target].to_numpy(dtype=float)
    v = df[variance_col].to_numpy(dtype=float)

    # weights (mean model only)
    weights = None
    if bool(args.use_weights):
        n_eff = float(args.n_effective)
        if not np.isfinite(n_eff) or n_eff <= 0:
            raise ValueError("--n-effective は正の数を指定してください")
        # var(mean) ≈ variance / n_eff
        var_mean = v / n_eff
        var_mean = np.maximum(var_mean, float(args.weight_eps))
        weights = 1.0 / var_mean
        # 影響が極端になりすぎないようにクリップ
        w_med = float(np.median(weights))
        if np.isfinite(w_med) and w_med > 0:
            weights = np.clip(weights, w_med / 1000.0, w_med * 1000.0)
        print(f"[weights] enabled: n_eff={n_eff}, median_w={float(np.median(weights)):.3g}")

    # variance response (log)
    log_eps = float(args.log_eps)
    if not np.isfinite(log_eps) or log_eps <= 0:
        raise ValueError("--log-eps は正の数を指定してください")
    y_var = np.log(np.maximum(v, log_eps))

    # output
    outdir = str(args.outdir).strip() if args.outdir else "gam_out"
    prefix = str(args.prefix).strip() if args.prefix else f"gam_{target}"
    _ensure_outdir(outdir)

    # model 1: mean
    print("\n=== [1/2] 平均モデル (target) のGAM ===")
    gam_mean = fit_gam(
        X,
        y_mean,
        feature_cols,
        n_splines=int(args.n_splines),
        spline_order=int(args.spline_order),
        lam=float(args.lam),
        gridsearch=bool(args.gridsearch),
        weights=weights,
    )

    # model 2: variance (log)
    print("\n=== [2/2] 分散モデル (log(variance)) のGAM ===")
    gam_var = fit_gam(
        X,
        y_var,
        feature_cols,
        n_splines=int(args.n_splines),
        spline_order=int(args.spline_order),
        lam=float(args.lam),
        gridsearch=bool(args.gridsearch),
        weights=None,
    )

    # save summaries
    summary_path = os.path.join(outdir, f"{prefix}_summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("=== GAM analysis ===\n")
        f.write(f"csv_file: {csv_file}\n")
        f.write(f"target: {target}\n")
        f.write(f"variance_col: {variance_col}\n")
        f.write(f"features: {feature_cols}\n")
        f.write(f"gridsearch: {bool(args.gridsearch)}\n")
        f.write(f"n_splines: {int(args.n_splines)}\n")
        f.write(f"spline_order: {int(args.spline_order)}\n")
        f.write(f"use_weights(mean): {bool(args.use_weights)}\n")
        if bool(args.use_weights):
            f.write(f"n_effective: {float(args.n_effective)}\n")
        f.write("\n--- mean model (target) ---\n")
        try:
            f.write(gam_mean.summary())
        except Exception:
            f.write(str(getattr(gam_mean, "statistics_", {})) + "\n")
        f.write("\n--- variance model (log variance) ---\n")
        try:
            f.write(gam_var.summary())
        except Exception:
            f.write(str(getattr(gam_var, "statistics_", {})) + "\n")
    print(f"[write] summary: {summary_path}")

    # importance
    print("\n=== 重要度（drop-one）計算: mean ===")
    imp_mean = compute_importance_drop_term(
        gam_mean,
        X,
        y_mean,
        feature_cols,
        n_splines=int(args.n_splines),
        spline_order=int(args.spline_order),
        weights=weights,
    )
    imp_mean_path = os.path.join(outdir, f"{prefix}_importance_mean.csv")
    imp_mean.to_csv(imp_mean_path, index=False)
    print(f"[write] importance mean: {imp_mean_path}")

    print("\n=== 重要度（drop-one）計算: variance ===")
    imp_var = compute_importance_drop_term(
        gam_var,
        X,
        y_var,
        feature_cols,
        n_splines=int(args.n_splines),
        spline_order=int(args.spline_order),
        weights=None,
    )
    imp_var_path = os.path.join(outdir, f"{prefix}_importance_variance.csv")
    imp_var.to_csv(imp_var_path, index=False)
    print(f"[write] importance variance: {imp_var_path}")

    # plots
    print("\n=== 部分効果プロット出力 ===")
    plot_partial_effects(
        gam_mean,
        feature_cols,
        outdir=outdir,
        prefix=prefix,
        title_prefix="mean",
    )
    plot_partial_effects(
        gam_var,
        feature_cols,
        outdir=outdir,
        prefix=prefix,
        title_prefix="variance",
    )
    print(f"[done] outdir: {outdir}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"エラー: {e}", file=sys.stderr)
        sys.exit(1)


