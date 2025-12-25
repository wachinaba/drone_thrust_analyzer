#!/usr/bin/env python3
"""
GPyTorch(SVGP)で保存したGPRモデル（gaussian_process_regression.py の torch.save bundle）を読み込み、
matplotlib のスライダーで横軸以外の特徴量を固定値として変更しながら、
指定した横軸に対する平均関数（必要なら±2σ）をプロットするビューア。

使い方例:
  python3 gpr_gpytorch_matplotlib_viewer.py --model model.pt --csv data.csv --x-feature distance
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

try:
    import torch
    import gpytorch
except Exception:
    torch = None
    gpytorch = None

try:
    from sklearn.preprocessing import StandardScaler
except Exception:
    StandardScaler = None


def parse_tf(v):
    """
    argparse の type に渡す想定。
    受理: t/f, true/false, 1/0, yes/no, y/n（大小文字・空白は無視）
    """
    if isinstance(v, bool):
        return v
    if v is None:
        raise argparse.ArgumentTypeError("t/f を指定してください")
    s = str(v).strip().lower()
    if s in ["t", "true", "1", "yes", "y", "on"]:
        return True
    if s in ["f", "false", "0", "no", "n", "off"]:
        return False
    raise argparse.ArgumentTypeError(f"t/f を指定してください（入力='{v}'）")


def _select_torch_device(device_str: str):
    if torch is None:
        return None
    s = (device_str or "auto").strip().lower()
    if s == "cpu":
        return torch.device("cpu")
    if s == "cuda":
        return torch.device("cuda")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def _deserialize_scaler(obj):
    """
    gaussian_process_regression.py の _serialize_scaler で保存された dict を復元する。
    """
    if obj is None:
        return None
    if StandardScaler is None:
        return None
    if isinstance(obj, dict) and obj.get("type") == "StandardScaler":
        sc = StandardScaler(with_mean=bool(obj.get("with_mean", True)), with_std=bool(obj.get("with_std", True)))
        for k in ["mean_", "scale_", "var_"]:
            if obj.get(k, None) is not None:
                setattr(sc, k, np.array(obj[k], dtype=float))
        nfi = int(obj.get("n_features_in_", 0) or 0)
        if nfi > 0:
            sc.n_features_in_ = nfi
        sc.n_samples_seen_ = 1
        return sc
    if StandardScaler is not None and isinstance(obj, StandardScaler):
        return obj
    return None


@dataclass
class ModelBundle:
    bundle: Dict
    feature_columns: List[str]
    target_column: str
    scaler: Optional[object]


def load_gpytorch_bundle(path: str, trust: bool = False) -> ModelBundle:
    if torch is None:
        raise RuntimeError("torch が import できません。")
    if not os.path.exists(path):
        raise FileNotFoundError(f"モデルファイルが見つかりません: {path}")

    try:
        b = torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        b = torch.load(path, map_location="cpu")
    except Exception as e:
        msg = str(e)
        if ("Weights only load failed" in msg) or ("WeightsUnpickler" in msg):
            if not trust:
                raise RuntimeError(
                    "PyTorch の安全ロード(weights_only=True)で読み込めませんでした。"
                    "このモデルファイルを信頼できる場合は --trust-model t を付けて再実行してください。"
                ) from e
            try:
                b = torch.load(path, map_location="cpu", weights_only=False)
            except TypeError:
                b = torch.load(path, map_location="cpu")
        else:
            raise

    if not isinstance(b, dict) or b.get("backend") != "gpytorch":
        raise RuntimeError(f"gpytorch bundle ではありません: {path}")

    feats = b.get("feature_columns", None)
    if not feats or not isinstance(feats, (list, tuple)):
        raise RuntimeError("bundle に feature_columns がありません（または不正）。")
    feats = [str(x) for x in feats]

    tgt = str(b.get("target_column", "torque_x"))
    sc = _deserialize_scaler(b.get("scaler", None))
    return ModelBundle(bundle=b, feature_columns=feats, target_column=tgt, scaler=sc)


if gpytorch is not None and torch is not None:
    class _SVGPModel(gpytorch.models.ApproximateGP):
        def __init__(
            self,
            inducing_points,
            kernel_type="rbf",
            ard_num_dims=None,
            matern_nu=1.5,
            use_linear=False,
            length_scale_bounds=None,
        ):
            variational_distribution = gpytorch.variational.CholeskyVariationalDistribution(inducing_points.size(0))
            variational_strategy = gpytorch.variational.VariationalStrategy(
                self,
                inducing_points,
                variational_distribution,
                learn_inducing_locations=True,
            )
            super().__init__(variational_strategy)

            self.mean_module = gpytorch.means.ConstantMean()

            if kernel_type in ["rbf", "rbf_white", "rbf_linear"]:
                base_kernel = gpytorch.kernels.RBFKernel(ard_num_dims=ard_num_dims)
            elif kernel_type == "matern":
                base_kernel = gpytorch.kernels.MaternKernel(nu=float(matern_nu), ard_num_dims=ard_num_dims)
            else:
                base_kernel = gpytorch.kernels.RBFKernel(ard_num_dims=ard_num_dims)

            if length_scale_bounds is not None:
                try:
                    lo, hi = float(length_scale_bounds[0]), float(length_scale_bounds[1])
                    if (lo > 0) and (hi > lo):
                        base_kernel.register_constraint("raw_lengthscale", gpytorch.constraints.Interval(lo, hi))
                except Exception:
                    pass

            covar = gpytorch.kernels.ScaleKernel(base_kernel)
            if use_linear:
                covar = covar + gpytorch.kernels.LinearKernel()
            self.covar_module = covar

        def forward(self, x):
            mean_x = self.mean_module(x)
            covar_x = self.covar_module(x)
            return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)


    class GPyTorchSparseGPR:
        """
        SVGP を sklearn の predict に寄せた薄いラッパ。
        """

        def __init__(
            self,
            model,
            likelihood,
            device,
            y_mean=0.0,
            y_std=1.0,
            pred_batch_size=8192,
        ):
            self.model = model
            self.likelihood = likelihood
            self.device = device
            self.y_mean = float(y_mean)
            self.y_std = float(y_std) if float(y_std) != 0.0 else 1.0
            self.pred_batch_size = int(pred_batch_size) if pred_batch_size else 8192

        def _to_numpy(self, t):
            return t.detach().cpu().numpy()

        def predict(self, X, return_std=True):
            if X is None:
                raise ValueError("X is None")

            self.model.eval()
            self.likelihood.eval()

            X_np = np.asarray(X, dtype=np.float32)
            n = int(X_np.shape[0])
            bs = max(1, int(self.pred_batch_size))

            means = []
            stds = []
            with torch.no_grad(), gpytorch.settings.fast_pred_var():
                for i in range(0, n, bs):
                    xb = torch.from_numpy(X_np[i : i + bs]).to(self.device)
                    pred = self.likelihood(self.model(xb))
                    m = pred.mean * self.y_std + self.y_mean
                    means.append(self._to_numpy(m))
                    if return_std:
                        v = pred.variance.clamp_min(0.0)
                        s = torch.sqrt(v) * self.y_std
                        stds.append(self._to_numpy(s))

            y_mean = np.concatenate(means, axis=0)
            if return_std:
                y_std = np.concatenate(stds, axis=0) if stds else None
                return y_mean, y_std
            return y_mean


def build_gpytorch_wrapper_from_bundle(bundle: Dict, device_str: str = "auto") -> GPyTorchSparseGPR:
    if torch is None or gpytorch is None:
        raise RuntimeError("torch/gpytorch が import できません。")
    device = _select_torch_device(device_str)
    if device is None:
        device = torch.device("cpu")

    inducing = bundle.get("inducing_points", None)
    if inducing is None:
        raise RuntimeError("gpytorch bundle に inducing_points がありません。")
    inducing = inducing.to(device)

    feature_columns = bundle.get("feature_columns", None)
    n_features = len(feature_columns) if feature_columns else int(inducing.shape[1])

    ard = n_features if bool(bundle.get("anisotropic", False)) else None
    kernel = bundle.get("kernel", "rbf")
    use_linear = (kernel == "rbf_linear")

    model = _SVGPModel(
        inducing_points=inducing,
        kernel_type=kernel,
        ard_num_dims=ard,
        matern_nu=float(bundle.get("matern_nu", 1.5) or 1.5),
        use_linear=use_linear,
        length_scale_bounds=None,
    ).to(device)
    likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device)
    model.load_state_dict(bundle["model_state_dict"])
    likelihood.load_state_dict(bundle["likelihood_state_dict"])

    wrapper = GPyTorchSparseGPR(
        model=model,
        likelihood=likelihood,
        device=device,
        y_mean=float(bundle.get("y_mean", 0.0)),
        y_std=float(bundle.get("y_std", 1.0)),
        pred_batch_size=int(bundle.get("pred_batch_size", 8192) or 8192),
    )
    return wrapper


def load_csv_feature_stats(csv_path: str, feature_columns: List[str]) -> Tuple[pd.DataFrame, Dict[str, float], Dict[str, float], Dict[str, float]]:
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSVファイルが見つかりません: {csv_path}")

    df = pd.read_csv(csv_path)
    missing = [c for c in feature_columns if c not in df.columns]
    if missing:
        raise ValueError(f"CSVに必要な特徴量列がありません: {missing}")

    df_num = df.copy()
    for col in feature_columns:
        df_num[col] = pd.to_numeric(df_num[col], errors="coerce")

    df_clean = df_num.dropna(subset=feature_columns).reset_index(drop=True)
    if df_clean.empty:
        raise ValueError("CSVの前処理後に有効なデータがありません（特徴量がNaNだらけの可能性）。")

    mins = {c: float(df_clean[c].min()) for c in feature_columns}
    maxs = {c: float(df_clean[c].max()) for c in feature_columns}
    meds = {c: float(df_clean[c].median()) for c in feature_columns}
    return df_clean, mins, maxs, meds


def _safe_minmax(lo: float, hi: float, eps: float = 1e-6) -> Tuple[float, float]:
    lo = float(lo)
    hi = float(hi)
    if (not np.isfinite(lo)) or (not np.isfinite(hi)):
        return (-1.0, 1.0)
    if lo == hi:
        return (lo - eps, hi + eps)
    if lo > hi:
        return (hi, lo)
    return (lo, hi)


def main():
    p = argparse.ArgumentParser(description="GPyTorch GPR viewer (matplotlib slider)")
    p.add_argument("--model", required=True, help="gaussian_process_regression.py が保存した gpytorch モデル(.pt/.pth)")
    p.add_argument("--csv", required=True, help="スライダー範囲(min/max)算出用のCSV")
    p.add_argument("--x-feature", required=True, help="横軸にする特徴量名（学習特徴量のいずれか）")
    p.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto", help="推論デバイス（デフォルト: auto）")
    p.add_argument("--n-points", type=int, default=400, help="曲線の分解能（デフォルト: 400）")
    p.add_argument("--show-uncertainty", type=parse_tf, default=True, metavar="{t,f}", help="±2σ帯を表示（デフォルト: t）")
    p.add_argument("--trust-model", type=parse_tf, default=False, metavar="{t,f}", help="torch.load unsafe を許可（デフォルト: f）")
    p.add_argument("--x-min", type=float, default=None, help="横軸の下限（未指定ならCSVのmin）")
    p.add_argument("--x-max", type=float, default=None, help="横軸の上限（未指定ならCSVのmax）")
    args = p.parse_args()

    if torch is None or gpytorch is None:
        raise RuntimeError("このビューアは torch と gpytorch が必要です。インストールしてください。")

    mb = load_gpytorch_bundle(args.model, trust=bool(args.trust_model))
    feature_columns = mb.feature_columns

    x_feature = str(args.x_feature).strip()
    if x_feature not in feature_columns:
        raise ValueError(f"--x-feature '{x_feature}' がモデルの feature_columns に含まれていません: {feature_columns}")

    df_clean, mins, maxs, meds = load_csv_feature_stats(args.csv, feature_columns)

    # 横軸範囲
    x_lo = mins[x_feature] if args.x_min is None else float(args.x_min)
    x_hi = maxs[x_feature] if args.x_max is None else float(args.x_max)
    x_lo, x_hi = _safe_minmax(x_lo, x_hi, eps=1e-6)

    # モデル復元
    model = build_gpytorch_wrapper_from_bundle(mb.bundle, device_str=args.device)
    scaler = mb.scaler

    # スライダー対象（横軸以外）
    slider_features = [c for c in feature_columns if c != x_feature]

    # Figureサイズ（スライダー数に応じて縦を伸ばす）
    n_sl = len(slider_features)
    fig_h = max(6.5, 2.2 + 0.32 * n_sl)
    fig = plt.figure(figsize=(12.0, fig_h))

    # メインプロット領域
    ax = fig.add_axes([0.08, 0.08, 0.56, 0.86])
    ax.set_xlabel(x_feature)
    ax.set_ylabel(f"pred mean ({mb.target_column})")
    ax.grid(True, alpha=0.25)

    # スライダー領域（右側）
    # 上から下へ縦に並べる
    slider_axes = {}
    sliders: Dict[str, Slider] = {}
    top = 0.93
    bottom = 0.08
    if n_sl > 0:
        total_h = top - bottom
        h = total_h / n_sl
        for i, col in enumerate(slider_features):
            y0 = top - (i + 1) * h + 0.02 * h
            ax_sl = fig.add_axes([0.70, y0, 0.26, 0.55 * h])
            slider_axes[col] = ax_sl
            lo, hi = _safe_minmax(mins[col], maxs[col], eps=1e-6)
            init = float(meds.get(col, (lo + hi) / 2.0))
            if not np.isfinite(init):
                init = (lo + hi) / 2.0
            init = float(np.clip(init, lo, hi))
            sliders[col] = Slider(
                ax=ax_sl,
                label=col,
                valmin=lo,
                valmax=hi,
                valinit=init,
                valstep=None,
            )

    # リセットボタン（中央値へ）
    ax_btn = fig.add_axes([0.70, 0.02, 0.12, 0.04])
    btn = Button(ax_btn, "reset(median)")

    # 曲線の初期描画
    x_grid = np.linspace(x_lo, x_hi, int(max(10, args.n_points)))
    line_mean, = ax.plot(x_grid, np.zeros_like(x_grid), lw=2.2, color="C0", label="mean")
    band = None
    if bool(args.show_uncertainty):
        band = ax.fill_between(x_grid, np.zeros_like(x_grid), np.zeros_like(x_grid), color="C0", alpha=0.18, label="±2σ")
    ax.legend(frameon=True)

    def build_X_grid() -> np.ndarray:
        X = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
        # まず中央値で埋める
        for j, col in enumerate(feature_columns):
            X[:, j] = float(meds[col])
        # 横軸だけ置き換え
        X[:, feature_columns.index(x_feature)] = x_grid
        # スライダー値で置き換え
        for col, sl in sliders.items():
            X[:, feature_columns.index(col)] = float(sl.val)
        return X

    def predict_curve():
        X = build_X_grid()
        X_infer = X
        if scaler is not None:
            try:
                X_infer = scaler.transform(X)
            except Exception:
                X_infer = X
        if bool(args.show_uncertainty):
            y_mean, y_std = model.predict(X_infer, return_std=True)
            return np.asarray(y_mean).reshape(-1), (np.asarray(y_std).reshape(-1) if y_std is not None else None)
        y_mean = model.predict(X_infer, return_std=False)
        return np.asarray(y_mean).reshape(-1), None

    def redraw(_evt=None):
        nonlocal band
        y_mean, y_std = predict_curve()
        line_mean.set_ydata(y_mean)
        # y軸を自動調整（±2σも含める）
        y_lo = float(np.nanmin(y_mean))
        y_hi = float(np.nanmax(y_mean))
        if y_std is not None:
            y_lo = float(np.nanmin(y_mean - 2.0 * y_std))
            y_hi = float(np.nanmax(y_mean + 2.0 * y_std))
        if np.isfinite(y_lo) and np.isfinite(y_hi):
            if y_lo == y_hi:
                y_lo -= 1e-6
                y_hi += 1e-6
            ax.set_ylim(y_lo, y_hi)
        # 不確実性帯
        if bool(args.show_uncertainty):
            if band is not None:
                try:
                    band.remove()
                except Exception:
                    pass
            if y_std is not None:
                band = ax.fill_between(x_grid, y_mean - 2.0 * y_std, y_mean + 2.0 * y_std, color="C0", alpha=0.18, label="±2σ")
        fig.canvas.draw_idle()

    def on_reset(_evt=None):
        for col, sl in sliders.items():
            v = float(meds.get(col, sl.val))
            lo, hi = float(sl.valmin), float(sl.valmax)
            if not np.isfinite(v):
                v = (lo + hi) / 2.0
            v = float(np.clip(v, lo, hi))
            sl.set_val(v)
        redraw()

    # 初回描画
    redraw()

    # イベント登録
    for sl in sliders.values():
        sl.on_changed(redraw)
    btn.on_clicked(on_reset)

    title = f"GPyTorch GPR mean: x={x_feature}  (model target={mb.target_column})"
    fig.suptitle(title, fontsize=14)
    plt.show()


if __name__ == "__main__":
    main()





