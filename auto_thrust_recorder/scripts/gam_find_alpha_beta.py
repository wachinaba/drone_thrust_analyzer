#!/usr/bin/env python3
"""
GAM で torque_x を学習し、固定条件（wall_spacing/target_thrust/distance など）下で
alpha, beta のみをスイープして「壁効果モーメント（torque_x）を抑制できる条件」を探索する。

主目的（ユーザ指定）:
  - torque_x を 0 に近づけたい
  - かつ torque_x < 0（壁に吸着方向）は避けたい（負を強く罰する or hard制約）
  - alpha, beta を動かす。他の条件は状況で固定される。

分散（variance_torque_x）:
  - `--use-variance t` のとき、log(variance_target) の GAM も学習し、標準偏差を score に加える。
    => 「平均だけ小さいが不安定」な条件を避けられる。

出力:
  - alpha/beta グリッドの全点CSV
  - topK候補CSV
  - 予測平均(mu)ヒートマップ（0等高線付き）
  - scoreヒートマップ
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
    os.makedirs(path, exist_ok=True)


def _coerce_numeric(df: pd.DataFrame, cols: list[str]):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def parse_fix_overrides(fixes):
    """
    --fix col=value を dict にする（float変換できるもののみ）
    """
    out = {}
    if not fixes:
        return out
    for item in fixes:
        if not item or ("=" not in item):
            continue
        col, val = item.split("=", 1)
        col = col.strip()
        try:
            out[col] = float(val)
        except Exception:
            continue
    return out


def build_additive_gam(n_features: int, n_splines: int, spline_order: int, lam: float | np.ndarray):
    if n_features <= 0:
        raise ValueError("n_features must be > 0")
    terms = s(0, n_splines=n_splines, spline_order=spline_order)
    for i in range(1, n_features):
        terms = terms + s(i, n_splines=n_splines, spline_order=spline_order)
    return LinearGAM(terms, lam=lam)


def fit_gam(
    X: np.ndarray,
    y: np.ndarray,
    *,
    n_splines: int,
    spline_order: int,
    lam: float,
    gridsearch: bool,
):
    if gridsearch:
        gam = build_additive_gam(X.shape[1], n_splines=n_splines, spline_order=spline_order, lam=float(lam))
        lams = np.logspace(-3, 3, 9)
        gam.gridsearch(X, y, lam=lams, progress=False)
        return gam
    # no gridsearch: fixed lam per term
    lam_vec = np.array([float(lam)] * X.shape[1], dtype=float)
    gam = build_additive_gam(X.shape[1], n_splines=n_splines, spline_order=spline_order, lam=lam_vec)
    gam.fit(X, y)
    return gam


def _predict_mu(gam: LinearGAM, X: np.ndarray):
    pred = gam.predict(X)
    return np.asarray(pred, dtype=float).reshape(-1)


def _predict_std_from_logvar(gam_logvar: LinearGAM, X: np.ndarray, log_eps: float):
    """
    log(variance) を予測 -> variance = exp(logvar) -> std = sqrt(variance)
    """
    lv = np.asarray(gam_logvar.predict(X), dtype=float).reshape(-1)
    var = np.exp(lv)
    var = np.maximum(var, float(log_eps))
    return np.sqrt(var)


def _filter_df_by_fixes(df: pd.DataFrame, fixes: dict[str, float], tol: float):
    """
    fixes の列で df を絞り込む（数値: |x-v|<=tol）。
    """
    if not fixes:
        return df
    out = df
    for col, val in fixes.items():
        if col not in out.columns:
            continue
        s = pd.to_numeric(out[col], errors="coerce")
        out = out[np.isfinite(s) & (np.abs(s - float(val)) <= float(tol))]
    return out


def _make_fixed_defaults(df: pd.DataFrame, feature_cols: list[str], fixes: dict[str, float], by_fixes_tol: float):
    """
    未指定の特徴量は中央値で固定（可能なら fixes で絞ったサブセットの中央値を使う）。
    """
    df_sub = _filter_df_by_fixes(df, fixes, tol=by_fixes_tol)
    base = df_sub if len(df_sub) > 50 else df  # 極端に減るなら全体に戻す
    fixed = {}
    for c in feature_cols:
        if c in fixes:
            fixed[c] = float(fixes[c])
        else:
            med = pd.to_numeric(base[c], errors="coerce").median()
            fixed[c] = float(med) if np.isfinite(med) else float(pd.to_numeric(df[c], errors="coerce").median())
    return fixed


def main():
    parser = argparse.ArgumentParser(description="GAMで torque_x を学習し、alpha/beta を探索して抑制条件を探します。")
    parser.add_argument("csv_file", help="入力CSVファイル")
    parser.add_argument("--target", type=str, default="torque_x", help="目的変数（デフォルト: torque_x）")
    parser.add_argument(
        "--variance-col",
        type=str,
        default=None,
        help="分散列（未指定なら variance_{target}）。use-variance=t のとき使用。",
    )
    parser.add_argument(
        "--features",
        type=str,
        default="distance,target_thrust,wall_spacing,alpha,beta,prop_spacing_x,prop_spacing_y",
        help="学習に使う特徴量（カンマ区切り）",
    )
    parser.add_argument("--alpha-col", type=str, default="alpha", help="探索するalpha列名（デフォルト: alpha）")
    parser.add_argument("--beta-col", type=str, default="beta", help="探索するbeta列名（デフォルト: beta）")
    parser.add_argument("--alpha-range", type=str, default="-40,40", help="alpha探索範囲 'a,b'（デフォルト:-40,40）")
    parser.add_argument("--beta-range", type=str, default="-40,40", help="beta探索範囲 'a,b'（デフォルト:-40,40）")
    parser.add_argument("--grid", type=int, default=81, help="alpha/betaの分割数（デフォルト:81）")

    parser.add_argument("--fix", action="append", default=None, help="固定条件 'col=value' を複数指定可")
    parser.add_argument(
        "--fix-tol",
        type=float,
        default=1e-3,
        help="固定条件で中央値を取る際のフィルタ許容誤差（デフォルト:1e-3）",
    )

    parser.add_argument("--outdir", type=str, default="gam_find_out", help="出力ディレクトリ")
    parser.add_argument("--prefix", type=str, default=None, help="出力接頭辞（未指定なら find_{target}）")

    # GAM params
    parser.add_argument("--gridsearch", type=parse_tf, default=False, metavar="{t,f}", help="lamを探索（デフォルト:f）")
    parser.add_argument("--n-splines", type=int, default=20, help="各s()のスプライン数（デフォルト:20）")
    parser.add_argument("--spline-order", type=int, default=3, help="スプライン次数（デフォルト:3）")
    parser.add_argument("--lam", type=float, default=0.001, help="gridsearchしない場合のlam（デフォルト:0.001）")

    # objective
    parser.add_argument("--neg-weight", type=float, default=10.0, help="負のmuへの罰係数（デフォルト:10）")
    parser.add_argument("--hard-nonnegative", type=parse_tf, default=False, metavar="{t,f}", help="mu<0 を候補から除外（デフォルト:f）")

    # variance on/off
    parser.add_argument("--use-variance", type=parse_tf, default=False, metavar="{t,f}", help="分散もスコアに入れる（デフォルト:f）")
    parser.add_argument("--lambda-std", type=float, default=0.0, help="stdへの重み（use-variance=t のとき、デフォルト:0）")
    parser.add_argument("--log-eps", type=float, default=1e-12, help="variance下限（デフォルト:1e-12）")

    parser.add_argument("--topk", type=int, default=30, help="上位候補の出力数（デフォルト:30）")
    args = parser.parse_args()

    # headless
    if not os.environ.get("DISPLAY"):
        try:
            matplotlib.use("Agg", force=True)
        except Exception:
            pass

    csv_file = args.csv_file
    if not os.path.exists(csv_file):
        raise FileNotFoundError(f"CSVが見つかりません: {csv_file}")

    target = str(args.target).strip()
    if not target:
        raise ValueError("--target が空です")

    feature_cols = [c.strip() for c in str(args.features).split(",") if c.strip()]
    if not feature_cols:
        raise ValueError("--features が空です")

    alpha_col = str(args.alpha_col).strip()
    beta_col = str(args.beta_col).strip()
    if alpha_col not in feature_cols or beta_col not in feature_cols:
        raise ValueError(f"alpha/beta列は features に含めてください: alpha_col={alpha_col}, beta_col={beta_col}, features={feature_cols}")

    variance_col = str(args.variance_col).strip() if args.variance_col else None
    if not variance_col:
        variance_col = f"variance_{target}"

    fixes = parse_fix_overrides(args.fix)

    # alpha/beta range
    def _parse_pair(s):
        t = str(s).strip()
        if "," not in t:
            raise ValueError(f"range は 'a,b' 形式です: {s}")
        a, b = t.split(",", 1)
        return float(a), float(b)

    a0, a1 = _parse_pair(args.alpha_range)
    b0, b1 = _parse_pair(args.beta_range)
    if not (np.isfinite(a0) and np.isfinite(a1) and np.isfinite(b0) and np.isfinite(b1)):
        raise ValueError("alpha/beta range に NaN/inf が含まれています")
    if int(args.grid) < 5:
        raise ValueError("--grid は 5以上を推奨します")

    outdir = str(args.outdir).strip() if args.outdir else "gam_find_out"
    prefix = str(args.prefix).strip() if args.prefix else f"find_{target}"
    _ensure_outdir(outdir)

    # load data
    need_cols = [target] + feature_cols
    if bool(args.use_variance):
        need_cols.append(variance_col)
    df = pd.read_csv(csv_file, usecols=lambda c: c in set(need_cols) or c in set([variance_col]))
    df = _coerce_numeric(df, list(set(need_cols)))
    df = df.dropna(subset=need_cols).reset_index(drop=True)
    if len(df) == 0:
        raise ValueError("有効な行がありません（必要列の欠損/列名を確認してください）")

    X = df[feature_cols].to_numpy(float)
    y = df[target].to_numpy(float)

    print(f"[data] rows={len(df)}, features={feature_cols}, target={target}")
    print(f"[fix] overrides={fixes}")

    # fit mean GAM
    print("[fit] mean GAM...")
    gam_mean = fit_gam(
        X, y,
        n_splines=int(args.n_splines),
        spline_order=int(args.spline_order),
        lam=float(args.lam),
        gridsearch=bool(args.gridsearch),
    )
    print(f"[fit] mean lam={getattr(gam_mean, 'lam', None)}")

    gam_logvar = None
    if bool(args.use_variance):
        if variance_col not in df.columns:
            raise ValueError(f"use-variance=t ですが variance列がありません: {variance_col}")
        yv = np.log(np.maximum(df[variance_col].to_numpy(float), float(args.log_eps)))
        print("[fit] log-variance GAM...")
        gam_logvar = fit_gam(
            X, yv,
            n_splines=int(args.n_splines),
            spline_order=int(args.spline_order),
            lam=float(args.lam),
            gridsearch=bool(args.gridsearch),
        )
        print(f"[fit] var lam={getattr(gam_logvar, 'lam', None)}")

    # build fixed values (medians for unspecified)
    fixed = _make_fixed_defaults(df, feature_cols, fixes, by_fixes_tol=float(args.fix_tol))
    # alpha/beta will be overwritten by grid; keep for record
    fixed_alpha0 = float(fixed.get(alpha_col, 0.0))
    fixed_beta0 = float(fixed.get(beta_col, 0.0))
    print(f"[fixed] (defaults) alpha={fixed_alpha0}, beta={fixed_beta0} (will be swept)")

    # grid
    alphas = np.linspace(a0, a1, int(args.grid))
    betas = np.linspace(b0, b1, int(args.grid))
    AA, BB = np.meshgrid(alphas, betas)
    n = AA.size

    Xg = np.zeros((n, len(feature_cols)), dtype=float)
    for j, c in enumerate(feature_cols):
        if c == alpha_col:
            Xg[:, j] = AA.reshape(-1)
        elif c == beta_col:
            Xg[:, j] = BB.reshape(-1)
        else:
            Xg[:, j] = float(fixed[c])

    mu = _predict_mu(gam_mean, Xg)
    std = None
    if gam_logvar is not None and bool(args.use_variance):
        std = _predict_std_from_logvar(gam_logvar, Xg, log_eps=float(args.log_eps))

    neg = np.maximum(0.0, -mu)
    score = np.abs(mu) + float(args.neg_weight) * neg
    if std is not None and bool(args.use_variance):
        score = score + float(args.lambda_std) * std

    # hard constraint
    if bool(args.hard_nonnegative):
        score = np.where(mu >= 0.0, score, np.inf)

    # outputs
    df_grid = pd.DataFrame(
        {
            alpha_col: AA.reshape(-1),
            beta_col: BB.reshape(-1),
            "mu": mu,
            "score": score,
        }
    )
    if std is not None:
        df_grid["std"] = std
    df_grid_path = os.path.join(outdir, f"{prefix}_alpha_beta_grid.csv")
    df_grid.to_csv(df_grid_path, index=False)
    print(f"[write] grid csv: {df_grid_path}")

    # topk
    topk = max(1, int(args.topk))
    df_top = df_grid.replace([np.inf, -np.inf], np.nan).dropna(subset=["score"])
    df_top = df_top.sort_values("score", ascending=True).head(topk).reset_index(drop=True)
    top_path = os.path.join(outdir, f"{prefix}_top{topk}.csv")
    df_top.to_csv(top_path, index=False)
    print(f"[write] topK csv: {top_path}")

    # heatmaps
    mu_img = mu.reshape(len(betas), len(alphas))
    score_img = score.reshape(len(betas), len(alphas))
    std_img = std.reshape(len(betas), len(alphas)) if std is not None else None

    def _imshow(Z, title, cbar_label, out_name, cmap="viridis", vmin=None, vmax=None):
        plt.figure(figsize=(10, 8))
        im = plt.imshow(
            Z,
            origin="lower",
            aspect="auto",
            extent=[a0, a1, b0, b1],
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
        )
        plt.xlabel(alpha_col)
        plt.ylabel(beta_col)
        plt.title(title)
        plt.colorbar(im, label=cbar_label)
        plt.grid(False)
        plt.tight_layout()
        outp = os.path.join(outdir, out_name)
        plt.savefig(outp, dpi=200, bbox_inches="tight")
        plt.close()
        return outp

    mu_out = _imshow(
        mu_img,
        title=f"mu({target}) vs {alpha_col},{beta_col} (fixed={{{', '.join([f'{k}={v:g}' for k,v in fixes.items()])}}})",
        cbar_label=f"mu({target})",
        out_name=f"{prefix}_mu_heatmap.png",
        cmap="coolwarm",
    )
    # overlay mu=0 contour on a separate figure (more visible)
    plt.figure(figsize=(10, 8))
    im = plt.imshow(mu_img, origin="lower", aspect="auto", extent=[a0, a1, b0, b1], cmap="coolwarm")
    try:
        cs = plt.contour(AA, BB, mu_img, levels=[0.0], colors="k", linewidths=2.0)
        plt.clabel(cs, fmt={0.0: "mu=0"}, inline=True)
    except Exception:
        pass
    plt.xlabel(alpha_col)
    plt.ylabel(beta_col)
    plt.title(f"mu({target}) with mu=0 contour")
    plt.colorbar(im, label=f"mu({target})")
    plt.tight_layout()
    mu0_out = os.path.join(outdir, f"{prefix}_mu_heatmap_with_zero_contour.png")
    plt.savefig(mu0_out, dpi=200, bbox_inches="tight")
    plt.close()

    score_out = _imshow(
        score_img,
        title=f"score vs {alpha_col},{beta_col} (neg_weight={float(args.neg_weight):g}, use_var={bool(args.use_variance)})",
        cbar_label="score (lower is better)",
        out_name=f"{prefix}_score_heatmap.png",
        cmap="magma",
    )

    if std_img is not None:
        _imshow(
            std_img,
            title=f"std({variance_col}) proxy vs {alpha_col},{beta_col}",
            cbar_label="pred std",
            out_name=f"{prefix}_std_heatmap.png",
            cmap="viridis",
        )

    # write meta
    meta_path = os.path.join(outdir, f"{prefix}_meta.txt")
    with open(meta_path, "w", encoding="utf-8") as f:
        f.write(f"csv_file: {csv_file}\n")
        f.write(f"target: {target}\n")
        f.write(f"variance_col: {variance_col}\n")
        f.write(f"features: {feature_cols}\n")
        f.write(f"alpha_col: {alpha_col}\n")
        f.write(f"beta_col: {beta_col}\n")
        f.write(f"fixes: {fixes}\n")
        f.write(f"fixed_defaults: {fixed}\n")
        f.write(f"alpha_range: {a0},{a1}\n")
        f.write(f"beta_range: {b0},{b1}\n")
        f.write(f"grid: {int(args.grid)}\n")
        f.write(f"gridsearch: {bool(args.gridsearch)}\n")
        f.write(f"n_splines: {int(args.n_splines)}\n")
        f.write(f"spline_order: {int(args.spline_order)}\n")
        f.write(f"lam: {float(args.lam)}\n")
        f.write(f"neg_weight: {float(args.neg_weight)}\n")
        f.write(f"hard_nonnegative: {bool(args.hard_nonnegative)}\n")
        f.write(f"use_variance: {bool(args.use_variance)}\n")
        f.write(f"lambda_std: {float(args.lambda_std)}\n")
        f.write(f"mu_heatmap: {mu_out}\n")
        f.write(f"mu0_heatmap: {mu0_out}\n")
        f.write(f"score_heatmap: {score_out}\n")
        f.write(f"grid_csv: {df_grid_path}\n")
        f.write(f"top_csv: {top_path}\n")

    print(f"[write] meta: {meta_path}")
    print(f"[done] open: {mu0_out}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"エラー: {e}", file=sys.stderr)
        sys.exit(1)


