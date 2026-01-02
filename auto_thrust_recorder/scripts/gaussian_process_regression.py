#!/usr/bin/env python3
"""
ガウス過程回帰によるtorque_xのモデル化スクリプト

このスクリプトは、distance, tilt_angle, fold_angle, force_z等の特徴量から
torque_xを予測するガウス過程回帰モデルを作成します。

使用方法:
    python gaussian_process_regression.py input.csv [options]

例:
    python gaussian_process_regression.py data.csv --output model_results.png
"""

import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import argparse
import os
import japanize_matplotlib
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, Matern, ConstantKernel, DotProduct
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split, cross_val_score, GridSearchCV
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
from sklearn.kernel_ridge import KernelRidge
import warnings
from joblib import dump, load
import sklearn
from tqdm.auto import tqdm
warnings.filterwarnings('ignore')

# argparse 用: t/f (true/false) パーサ
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

_FACET_DRONE_CACHE = {}

def _get_facet_drone_rgba_cached(phi_deg, psi_deg, theta_deg, force_2d=False, dpi=160):
    """
    facet 背景用: visualize_morph_drone のレンダリング結果をキャッシュして返す。
    """
    try:
        phi = float(phi_deg)
        psi = float(psi_deg)
        th = float(theta_deg)
    except Exception:
        phi, psi, th = 0.0, 0.0, 0.0

    # 角度は表示目的なので丸めてキャッシュキーを安定化
    key = (round(phi, 3), round(psi, 3), round(th, 3), bool(force_2d), int(dpi))
    if key in _FACET_DRONE_CACHE:
        return _FACET_DRONE_CACHE[key]

    try:
        # scripts 配下なので同ディレクトリから import 可能な想定
        from visualize_morph_drone import render_morphing_drone_rgba
    except Exception as e:
        raise RuntimeError(f"visualize_morph_drone.py を import できません: {e}") from e

    img = render_morphing_drone_rgba(
        phi_deg=phi,
        psi_deg=psi,
        theta_deg=th,
        force_2d=bool(force_2d),
        dpi=int(dpi),
        draw_y0_plane=False,
    )
    _FACET_DRONE_CACHE[key] = img
    return img

def _coerce_float_or_none(v):
    try:
        x = float(v)
        if np.isfinite(x):
            return x
    except Exception:
        pass
    return None

def _infer_angle_from_group_keys(row_group_by, rkey, col_group_by, ckey, colname_candidates):
    """
    row/col の group key から角度を探す（最優先）。
    """
    if row_group_by:
        for col, val in zip(row_group_by, rkey):
            if col in colname_candidates:
                x = _coerce_float_or_none(val)
                if x is not None:
                    return x
    if col_group_by:
        for col, val in zip(col_group_by, ckey):
            if col in colname_candidates:
                x = _coerce_float_or_none(val)
                if x is not None:
                    return x
    return None

def _infer_angle_from_df(df_cell, colname_candidates):
    """
    df の列から中央値で角度を推定する（キーに無い場合のフォールバック）。
    """
    if df_cell is None or len(df_cell) == 0:
        return None
    for col in colname_candidates:
        if col in df_cell.columns:
            try:
                v = float(df_cell[col].median())
                if np.isfinite(v):
                    return v
            except Exception:
                continue
    return None

def _infer_fold_from_hue(df_cell, hue_col, global_hue_values):
    """
    hue が fold であるケース向けに、セル内に存在する hue 値（なければ global）から代表 fold を選ぶ。
    """
    vals = []
    if (hue_col is not None) and (df_cell is not None) and (len(df_cell) > 0) and (hue_col in df_cell.columns):
        try:
            vals = list(pd.unique(df_cell[hue_col].dropna()))
        except Exception:
            vals = []
    if not vals:
        vals = list(global_hue_values or [])
    nums = []
    for v in vals:
        x = _coerce_float_or_none(v)
        if x is not None:
            nums.append(x)
    if not nums:
        return None
    nums = sorted(nums)
    return float(nums[len(nums)//2])

def _serialize_scaler(scaler):
    """
    torch.save の安全ロード(weights_only)で壊れないように、
    sklearn の scaler を「プリミティブ + numpy」へ落とす。
    """
    if scaler is None:
        return None
    # StandardScaler のみ対応（このスクリプトで使っているため）
    if isinstance(scaler, StandardScaler):
        payload = {
            "type": "StandardScaler",
            "with_mean": bool(getattr(scaler, "with_mean", True)),
            "with_std": bool(getattr(scaler, "with_std", True)),
            "mean_": getattr(scaler, "mean_", None),
            "scale_": getattr(scaler, "scale_", None),
            "var_": getattr(scaler, "var_", None),
            "n_features_in_": int(getattr(scaler, "n_features_in_", 0) or 0),
        }
        return payload
    # 未対応は諦めて None（安全優先）
    return None


def _deserialize_scaler(obj):
    """
    _serialize_scaler の逆変換。
    """
    if obj is None:
        return None
    if isinstance(obj, dict) and obj.get("type") == "StandardScaler":
        sc = StandardScaler(with_mean=bool(obj.get("with_mean", True)), with_std=bool(obj.get("with_std", True)))
        # 変換に必要な属性を復元
        for k in ["mean_", "scale_", "var_"]:
            if obj.get(k, None) is not None:
                setattr(sc, k, np.array(obj[k], dtype=float))
        nfi = int(obj.get("n_features_in_", 0) or 0)
        if nfi > 0:
            sc.n_features_in_ = nfi
        # sklearn が参照することがあるので最低限入れておく
        sc.n_samples_seen_ = 1
        return sc
    # 既に scaler オブジェクトとして渡ってきた場合（joblibロードなど）
    if isinstance(obj, StandardScaler):
        return obj
    return None

# --- Optional: Sparse GPR backend (GPyTorch) ---
# sklearn の GaussianProcessRegressor は "sparse/inducing point" 近似を提供しないため、
# `--backend gpytorch` が指定された場合のみ GPyTorch による Variational GP (SVGP) を使用する。
try:
    import torch
    import gpytorch
    from torch.utils.data import TensorDataset, DataLoader
except Exception:
    torch = None
    gpytorch = None
    TensorDataset = None
    DataLoader = None


def _select_torch_device(device_str: str):
    """
    device_str: 'auto' | 'cpu' | 'cuda'
    """
    if torch is None:
        return None
    s = (device_str or "auto").strip().lower()
    if s == "cpu":
        return torch.device("cpu")
    if s == "cuda":
        return torch.device("cuda")
    # auto
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


if gpytorch is not None and torch is not None:
    class _SVGPModel(gpytorch.models.ApproximateGP):
        """
        Inducing point variational GP (SVGP) for regression.
        """
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

            # length_scale bounds (optional)
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
        GPyTorch SVGP を sklearn の API に寄せた薄いラッパ。
        - predict(X, return_std=True) -> (mean, std) / mean
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
                    xb = torch.from_numpy(X_np[i:i+bs]).to(self.device)
                    pred = self.likelihood(self.model(xb))
                    m = pred.mean
                    # 正規化yを元スケールへ
                    m = m * self.y_std + self.y_mean
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

        def get_lengthscales(self, n_features):
            """
            ARD lengthscale を取り出す。取得できない場合は None。
            """
            try:
                if hasattr(self.model, "covar_module"):
                    cm = self.model.covar_module
                    # Additive kernelの可能性があるので base_kernel を探す
                    base = None
                    if hasattr(cm, "base_kernel"):
                        base = cm.base_kernel
                    else:
                        if hasattr(cm, "kernels"):
                            for k in cm.kernels:
                                if hasattr(k, "base_kernel"):
                                    base = k.base_kernel
                                    break
                                if hasattr(k, "lengthscale"):
                                    base = k
                                    break
                    if base is None:
                        return None
                    ls = base.lengthscale.detach().cpu().numpy().reshape(-1)
                    if ls.size == 1:
                        return np.array([float(ls[0])] * int(n_features), dtype=float)
                    return ls.astype(float)
            except Exception:
                return None
            return None
else:
    class GPyTorchSparseGPR:
        """
        gpytorch が無い環境でも import できるようにするためのスタブ。
        """
        pass


def train_sparse_gpytorch(
    X_train,
    y_train,
    args,
    length_scale_bounds=None,
):
    """
    SVGP (inducing points) で近似的に GPR を学習する。
    """
    if torch is None or gpytorch is None:
        raise RuntimeError("torch/gpytorch が import できません。--backend gpytorch を使うにはインストールが必要です。")

    device = _select_torch_device(getattr(args, "device", "auto"))
    if device is None:
        raise RuntimeError("torch device selection failed.")

    X_np = np.asarray(X_train, dtype=np.float32)
    y_np = np.asarray(y_train, dtype=np.float32).reshape(-1)
    n = int(X_np.shape[0])
    d = int(X_np.shape[1])

    # y を標準化（sklearn normalize_y 相当）
    y_mean = float(np.mean(y_np)) if n > 0 else 0.0
    y_std = float(np.std(y_np)) if n > 0 else 1.0
    if not np.isfinite(y_std) or y_std <= 0.0:
        y_std = 1.0
    y_norm = (y_np - y_mean) / y_std

    # inducing points
    m = int(getattr(args, "num_inducing", 512) or 512)
    m = max(4, min(m, n))
    rs = np.random.RandomState(int(getattr(args, "random_state", 42) or 42))
    idx = rs.choice(n, size=m, replace=False) if n > m else np.arange(n)
    inducing = torch.from_numpy(X_np[idx]).to(device)

    ard = d if bool(getattr(args, "anisotropic", False)) else None
    use_linear = (getattr(args, "kernel", "rbf") == "rbf_linear")
    model = _SVGPModel(
        inducing_points=inducing,
        kernel_type=getattr(args, "kernel", "rbf"),
        ard_num_dims=ard,
        matern_nu=float(getattr(args, "matern_nu", 1.5) or 1.5),
        use_linear=use_linear,
        length_scale_bounds=length_scale_bounds,
    ).to(device)

    likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device)
    # sklearn の alpha は「追加ノイズ分散」なので、ここでは初期値として likelihood.noise に反映する
    try:
        a = float(getattr(args, "alpha", 1e-6) or 1e-6)
        if np.isfinite(a) and a > 0:
            likelihood.noise = max(a, 1e-6)
    except Exception:
        pass

    model.train()
    likelihood.train()

    epochs = int(getattr(args, "epochs", 300) or 300)
    lr = float(getattr(args, "lr", 0.01) or 0.01)
    batch_size = int(getattr(args, "batch_size", 1024) or 1024)
    batch_size = max(1, batch_size)

    X_t = torch.from_numpy(X_np).to(device)
    y_t = torch.from_numpy(y_norm).to(device)
    loader = DataLoader(TensorDataset(X_t, y_t), batch_size=batch_size, shuffle=True, drop_last=False)

    optimizer = torch.optim.Adam(list(model.parameters()) + list(likelihood.parameters()), lr=lr)
    mll = gpytorch.mlls.VariationalELBO(likelihood, model, num_data=n)

    log_every = max(1, int(getattr(args, "log_every", 50) or 50))

    pbar_ep = tqdm(range(1, epochs + 1), desc="[gpytorch] training", unit="epoch")
    for ep in pbar_ep:
        total_loss = 0.0
        pbar_batch = tqdm(loader, desc=f"epoch {ep}/{epochs}", unit="batch", leave=False)
        for xb, yb in pbar_batch:
            optimizer.zero_grad(set_to_none=True)
            out = model(xb)
            loss = -mll(out, yb)
            loss.backward()
            optimizer.step()

            loss_v = float(loss.detach().cpu().item())
            total_loss += loss_v
            pbar_batch.set_postfix(loss=f"{loss_v:.4g}")

        avg = total_loss / max(1, len(loader))
        # epoch の postfix は毎回更新、従来の log_every 相当は明示 print
        pbar_ep.set_postfix(avg_loss=f"{avg:.6f}")
        if (ep == 1) or (ep % log_every == 0) or (ep == epochs):
            tqdm.write(f"[gpytorch] epoch {ep}/{epochs} - avg_loss {avg:.6f}")

    wrapper = GPyTorchSparseGPR(
        model=model,
        likelihood=likelihood,
        device=device,
        y_mean=y_mean,
        y_std=y_std,
        pred_batch_size=int(getattr(args, "pred_batch_size", 8192) or 8192),
    )
    return wrapper


def save_gpytorch_model(path, wrapper: GPyTorchSparseGPR, feature_columns, scaler, args):
    if torch is None:
        raise RuntimeError("torch is not available")
    model = wrapper.model
    likelihood = wrapper.likelihood
    inducing = None
    try:
        inducing = model.variational_strategy.inducing_points.detach().cpu()
    except Exception:
        inducing = None
    bundle = {
        "backend": "gpytorch",
        "model_state_dict": model.state_dict(),
        "likelihood_state_dict": likelihood.state_dict(),
        "inducing_points": inducing,
        "feature_columns": list(feature_columns) if feature_columns is not None else None,
        # torch.load(weights_only=True) で読める形にする
        "scaler": _serialize_scaler(scaler),
        "target_column": getattr(args, "target", "torque_x"),
        "kernel": getattr(args, "kernel", "rbf"),
        "anisotropic": bool(getattr(args, "anisotropic", False)),
        "matern_nu": float(getattr(args, "matern_nu", 1.5) or 1.5),
        "y_mean": float(wrapper.y_mean),
        "y_std": float(wrapper.y_std),
        "pred_batch_size": int(getattr(wrapper, "pred_batch_size", 8192)),
        "normalize": bool(getattr(args, "normalize", False)),
        "meta": {
            "script": "gaussian_process_regression.py",
            "note": "Saved with torch.save (pickle).",
        },
    }
    torch.save(bundle, path)


def load_model_any(path, trust=False):
    """
    既存(joblib)と gpytorch(torch.save)の両方を読む。
    Returns:
      (backend, model_or_wrapper, scaler, feature_columns, target_column)
    """
    # まず joblib を試す（sklearn互換）
    try:
        bundle = load(path)
        if isinstance(bundle, dict) and bundle.get("backend") == "gpytorch":
            # joblibにgpytorchが入っていた場合（レア）
            return ("gpytorch", bundle, bundle.get("scaler", None), bundle.get("feature_columns", None), bundle.get("target_column", None))
        if isinstance(bundle, dict) and "model" in bundle:
            return ("sklearn", bundle["model"], bundle.get("scaler", None), bundle.get("feature_columns", None), bundle.get("target_column", None))
        # モデル単体が保存されていた場合への後方互換
        return ("sklearn", bundle, None, None, None)
    except Exception:
        pass

    # torch.save を試す
    if torch is None:
        raise RuntimeError(f"モデル読み込み失敗: joblib/torch のどちらでも読めませんでした: {path}")
    # PyTorch 2.6+: weights_only の既定が True になったため、明示指定して挙動を安定化
    try:
        bundle = torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        # 古いtorchで weights_only が無い場合
        bundle = torch.load(path, map_location="cpu")
    except Exception as e:
        msg = str(e)
        # 旧フォーマットで sklearn オブジェクトを含む場合、weights_only=True が失敗する
        if ("Weights only load failed" in msg) or ("WeightsUnpickler" in msg):
            if not trust:
                raise RuntimeError(
                    "PyTorch の安全ロード(weights_only=True)で読み込めませんでした。"
                    "このモデルファイルを信頼できる場合は --trust-model t を付けて再実行してください。"
                ) from e
            # trust=True の場合のみ unsafe な unpickle を許可
            try:
                bundle = torch.load(path, map_location="cpu", weights_only=False)
            except TypeError:
                bundle = torch.load(path, map_location="cpu")
        else:
            raise
    if isinstance(bundle, dict) and bundle.get("backend") == "gpytorch":
        sc = _deserialize_scaler(bundle.get("scaler", None))
        return ("gpytorch", bundle, sc, bundle.get("feature_columns", None), bundle.get("target_column", None))
    raise RuntimeError(f"未知のモデル形式です: {path}")


def build_gpytorch_wrapper_from_bundle(bundle, device_str="auto"):
    """
    torch.save した dict から GPyTorchSparseGPR を復元する。
    """
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
    n_features = len(feature_columns) if feature_columns else inducing.shape[1]

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

def load_and_preprocess_data(csv_file, selected_features=None, max_variance_torque_x=None, target_column='torque_x', debug=False):
    """
    CSVファイルを読み込み、データの前処理を行う
    
    Args:
        csv_file: CSVファイルのパス
        selected_features: 使用する特徴量のリスト（Noneの場合はtorque_x以外の全列）
        max_variance_torque_x: 'variance_torque_x' による上限フィルタ（Noneで無効）
        target_column: 目的変数の列名
        debug: 前処理の詳細ログを出す
    
    Returns:
        df_clean: 前処理済みのデータフレーム
        feature_columns: 特徴量の列名リスト
    """
    # CSVファイルの存在確認
    if not os.path.exists(csv_file):
        raise FileNotFoundError(f"ファイル '{csv_file}' が見つかりません。")
    
    # CSVファイルの読み込み
    try:
        df = pd.read_csv(csv_file)
        print(f"CSVファイル '{csv_file}' を読み込みました。")
        print(f"データ行数: {len(df)}")
        print(f"利用可能な列: {list(df.columns)}")
    except Exception as e:
        raise Exception(f"CSVファイルの読み込み中にエラーが発生しました: {e}")
    
    # 必要な列の存在確認
    if target_column not in df.columns:
        raise ValueError(f"必要な列が見つかりません: ['{target_column}']")

    # 特徴量列の決定
    if selected_features is not None and len(selected_features) > 0:
        if target_column in selected_features:
            raise ValueError(f"'{target_column}' は目的変数のため特徴量に含められません。")
        missing = [c for c in selected_features if c not in df.columns]
        if missing:
            raise ValueError(f"指定された特徴量が見つかりません: {missing}")
        feature_columns = list(selected_features)
    else:
        # 既定はtorque_x以外の全列
        feature_columns = [c for c in df.columns if c != target_column]
    
    # 目的変数と特徴量の列を数値型に変換
    numeric_columns = [target_column] + feature_columns
    for col in numeric_columns:
        if col in df.columns:
            before_na = int(df[col].isna().sum())
            df[col] = pd.to_numeric(df[col], errors='coerce')
            after_na = int(df[col].isna().sum())
            new_na = after_na - before_na
            if debug and new_na > 0:
                print(f"[debug] 数値変換で NaN 増加: col='{col}', +{new_na} (before={before_na}, after={after_na})")
    
    # 高分散サンプル除外（指定があり、列が存在する場合）
    if max_variance_torque_x is not None and 'variance_torque_x' in df.columns:
        before = len(df)
        df = df[df['variance_torque_x'] <= max_variance_torque_x]
        after = len(df)
        print(f"variance_torque_x フィルタ: {before-after} 行を除外（閾値 {max_variance_torque_x}）")
    
    # dropna 前の状態を保持（デバッグ用）
    df_before_dropna = df

    # NaNを含む行を削除
    df_clean = df.dropna(subset=[target_column] + feature_columns)
    # 後段のインデックス対応を簡単にするため 0..N-1 に振り直す
    df_clean = df_clean.reset_index(drop=True)

    if debug:
        subset_cols = [target_column] + feature_columns
        dropped = len(df_before_dropna) - len(df_clean)
        print(f"[debug] dropna(subset={subset_cols}) により除外: {dropped} 行")

        # fold_angle の分布がどう変わったか（存在する場合）
        if 'fold_angle' in df_before_dropna.columns:
            try:
                before_vc = df_before_dropna['fold_angle'].value_counts(dropna=False)
                after_vc = df_clean['fold_angle'].value_counts(dropna=False)
                all_vals = sorted(set(before_vc.index.tolist()) | set(after_vc.index.tolist()))
                print("[debug] fold_angle: before_dropna vs after_dropna (kept / dropped)")
                for v in all_vals:
                    b = int(before_vc.get(v, 0))
                    a = int(after_vc.get(v, 0))
                    d = b - a
                    print(f"[debug]   fold_angle={v!s}: before={b}, after={a}, dropped={d}")
            except Exception as e:
                print(f"[debug] fold_angle before/after 集計に失敗: {e}")

            # fold_angle=5 にフォーカスして、欠損列の内訳を表示
            try:
                fa5 = df_before_dropna[df_before_dropna['fold_angle'] == 5]
                if len(fa5) > 0:
                    miss_counts = fa5[subset_cols].isna().sum().astype(int)
                    total = len(fa5)
                    print(f"[debug] fold_angle==5 rows in before_dropna: {total}")
                    # どの列が欠けているか（欠損がある列のみ）
                    miss_nonzero = miss_counts[miss_counts > 0]
                    if len(miss_nonzero) == 0:
                        print("[debug] fold_angle==5: subset_cols に欠損はありません（dropna原因ではない可能性）")
                    else:
                        print("[debug] fold_angle==5: 欠損列内訳（count / total）")
                        for col, cnt in miss_nonzero.items():
                            print(f"[debug]   {col}: {int(cnt)}/{total}")

                        # 代表サンプル（欠損がある行を最大5件）
                        bad = fa5[fa5[subset_cols].isna().any(axis=1)]
                        if len(bad) > 0:
                            show_n = min(5, len(bad))
                            print(f"[debug] fold_angle==5: 欠損行サンプル（先頭 {show_n} 件）")
                            # インデックスと欠損列名のみ表示
                            for i, row in bad.head(show_n).iterrows():
                                missing_cols = [c for c in subset_cols if pd.isna(row.get(c, np.nan))]
                                print(f"[debug]   idx={i}, missing={missing_cols}")
                else:
                    print("[debug] fold_angle==5 rows in before_dropna: 0（読み込み/変換/フィルタ時点で存在しません）")
            except Exception as e:
                print(f"[debug] fold_angle==5 欠損内訳の表示に失敗: {e}")
    
    if df_clean.empty:
        raise ValueError("有効なデータがありません。")
    
    print(f"前処理後のデータ行数: {len(df_clean)}")
    print(f"使用する特徴量: {feature_columns}")
    
    return df_clean, feature_columns

def _debug_print_distribution(df, cols, stage_name, topk=20):
    """
    指定列の分布（value_counts）と fold_angle=5 の件数などをログ出力する。
    """
    if df is None:
        return
    print(f"\n[debug] --- {stage_name} --- rows={len(df)}")
    for col in cols:
        if col not in df.columns:
            print(f"[debug] '{col}': (missing)")
            continue
        s = df[col]
        n_na = int(s.isna().sum())
        n_unique = int(s.nunique(dropna=True))
        print(f"[debug] '{col}': unique={n_unique}, na={n_na}, dtype={s.dtype}")

        try:
            vc = s.value_counts(dropna=False)
            if len(vc) <= topk:
                # value_counts は頻度順。混在型だと sort が例外になることがあるのでそのまま表示
                print(vc.to_string())
            else:
                print(vc.head(int(topk)).to_string())
                print(f"[debug] ... ({len(vc) - int(topk)} more)")
        except Exception as e:
            print(f"[debug] value_counts failed for '{col}': {e}")

        # fold_angle=5 が欲しいケースが多いので、固定で件数を出す
        if col == 'fold_angle':
            try:
                s_num = pd.to_numeric(s, errors='coerce')
                cnt5 = int((s_num == 5).sum())
                print(f"[debug] '{col}' count(value==5): {cnt5}")
            except Exception:
                pass

def create_gp_model(kernel_type='rbf', alpha=1e-6, n_restarts_optimizer=2, n_features=None, anisotropic=False, length_scale_bounds=None, matern_nu=1.5):
    """
    ガウス過程回帰モデルを作成する
    
    Args:
        kernel_type: カーネルの種類 ('rbf', 'matern', 'rbf_white')
        alpha: ノイズの分散
        n_restarts_optimizer: 最適化の再起動回数
    
    Returns:
        gp_model: ガウス過程回帰モデル
    """
    # boundsの設定
    ls_bounds = length_scale_bounds if length_scale_bounds is not None else (1e-5, 1e5)

    # length_scaleの形状
    if anisotropic and n_features is not None and n_features > 1:
        rbf = RBF(length_scale=[1.0] * n_features, length_scale_bounds=ls_bounds)
        matern = Matern(length_scale=[1.0] * n_features, nu=matern_nu, length_scale_bounds=ls_bounds)
    else:
        rbf = RBF(length_scale=1.0, length_scale_bounds=ls_bounds)
        matern = Matern(length_scale=1.0, nu=matern_nu, length_scale_bounds=ls_bounds)

    if kernel_type == 'rbf':
        kernel = ConstantKernel(1.0) * rbf
    elif kernel_type == 'matern':
        kernel = ConstantKernel(1.0) * matern
    elif kernel_type == 'rbf_white':
        kernel = ConstantKernel(1.0) * rbf + WhiteKernel(noise_level=1e-6)
    elif kernel_type == 'rbf_linear':
        # RBF + 線形項
        kernel = ConstantKernel(1.0) * rbf + DotProduct(sigma_0=1.0)
    else:
        kernel = ConstantKernel(1.0) * rbf
    
    gp_model = GaussianProcessRegressor(
        kernel=kernel,
        alpha=alpha,
        n_restarts_optimizer=n_restarts_optimizer,
        random_state=42,
        copy_X_train=False,
        normalize_y=True
    )
    
    return gp_model

def grid_search_gpr(X_train, y_train, base_args, n_features, ls_bounds_initial):
    """
    簡易グリッドサーチで alpha と length_scale_bounds を探索
    """
    cv = base_args.cv_folds if base_args.cv_folds and base_args.cv_folds > 0 else 3
    # alpha候補（現在値の周辺）
    a = max(base_args.alpha, 1e-6)
    candidate_alphas = sorted(set([a, a*3.0, max(a/3.0, 1e-6)]))
    # bounds候補
    if ls_bounds_initial is not None:
        lo, hi = ls_bounds_initial
        candidates_bounds = [
            (lo, hi),
            (max(lo*0.1, 1e-5), hi),
            (lo, min(hi*0.5, 1e5))
        ]
    else:
        candidates_bounds = [(1e-2, 1e3), (1e-1, 1e3), (1e-2, 1e2)]

    best = {
        'score': float('inf'),
        'alpha': None,
        'bounds': None
    }

    for alpha in candidate_alphas:
        for b in candidates_bounds:
            model = create_gp_model(
                kernel_type=base_args.kernel,
                alpha=alpha,
                n_restarts_optimizer=base_args.n_restarts,
                n_features=n_features,
                anisotropic=base_args.anisotropic,
                length_scale_bounds=b,
                matern_nu=base_args.matern_nu
            )
            scores = cross_val_score(
                model, X_train, y_train,
                cv=cv,
                scoring='neg_mean_squared_error',
                n_jobs=base_args.cv_jobs
            )
            mse = -scores.mean()
            if mse < best['score']:
                best = {'score': mse, 'alpha': alpha, 'bounds': b}

    # 最良パラメータで学習
    best_model = create_gp_model(
        kernel_type=base_args.kernel,
        alpha=best['alpha'],
        n_restarts_optimizer=base_args.n_restarts,
        n_features=n_features,
        anisotropic=base_args.anisotropic,
        length_scale_bounds=best['bounds'],
        matern_nu=base_args.matern_nu
    )
    best_model.fit(X_train, y_train)
    print(f"グリッドサーチ最良: alpha={best['alpha']}, bounds={best['bounds']}, CV MSE={best['score']:.6f}")
    return best_model

def evaluate_model(model, X_test, y_test, X_train=None, y_train=None, cv_folds=0, cv_jobs=1, return_std=True):
    """
    モデルの評価を行う
    
    Args:
        model: 訓練済みのモデル
        X_test: テストデータの特徴量
        y_test: テストデータの目的変数
        X_train: 訓練データの特徴量（クロスバリデーション用）
        y_train: 訓練データの目的変数（クロスバリデーション用）
    
    Returns:
        metrics: 評価指標の辞書
    """
    # 予測
    if return_std:
        y_pred, y_std = model.predict(X_test, return_std=True)
    else:
        y_pred = model.predict(X_test, return_std=False)
        y_std = None
    
    # 評価指標の計算
    mse = mean_squared_error(y_test, y_pred)
    rmse = np.sqrt(mse)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)
    
    metrics = {
        'mse': mse,
        'rmse': rmse,
        'mae': mae,
        'r2': r2,
        'y_pred': y_pred,
        'y_std': y_std
    }
    
    # クロスバリデーション（有効な場合）
    # GPyTorch backend は sklearn の estimator interface / clone を満たさないのでスキップする。
    if cv_folds and cv_folds > 0 and X_train is not None and y_train is not None:
        if hasattr(model, "fit") and isinstance(model, GaussianProcessRegressor):
            cv_scores = cross_val_score(
                model, X_train, y_train,
                cv=cv_folds,
                scoring='neg_mean_squared_error',
                n_jobs=cv_jobs
            )
            metrics['cv_rmse'] = np.sqrt(-cv_scores.mean())
            metrics['cv_std'] = np.sqrt(cv_scores.std())
        else:
            print("[evaluate_model] 注意: cv_folds が指定されましたが、この backend では cross_val_score をサポートしません。スキップします。")
    
    return metrics

def plot_results(y_test, y_pred, y_std, feature_columns, output_file=None):
    """
    結果の可視化を行う
    
    Args:
        y_test: 実際の値
        y_pred: 予測値
        y_std: 予測の標準偏差
        feature_columns: 特徴量の列名
        output_file: 出力ファイル名
    """
    # 目的変数名は呼び出し元から渡される想定（後方互換のため取得できない場合は既定名）
    target_column = getattr(plot_results, "_target_column", "torque_x")
    # フォントサイズの設定
    plt.rcParams.update({'font.size': 20})
    fig, axes = plt.subplots(2, 2, figsize=(30, 24))
    
    # 1. 予測値 vs 実際の値
    ax1 = axes[0, 0]
    ax1.scatter(y_test, y_pred, alpha=0.6, s=50)
    ax1.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], 'r--', lw=2)
    ax1.set_xlabel(f'実際の値 ({target_column})')
    ax1.set_ylabel(f'予測値 ({target_column})')
    ax1.set_title('予測値 vs 実際の値')
    ax1.grid(True, alpha=0.3)
    
    # 2. 残差プロット
    ax2 = axes[0, 1]
    residuals = y_test - y_pred
    ax2.scatter(y_pred, residuals, alpha=0.6, s=50)
    ax2.axhline(y=0, color='r', linestyle='--')
    ax2.set_xlabel(f'予測値 ({target_column})')
    ax2.set_ylabel('残差')
    ax2.set_title('残差プロット')
    ax2.grid(True, alpha=0.3)
    
    # 3. 予測の不確実性
    ax3 = axes[1, 0]
    if y_std is not None:
        ax3.scatter(y_pred, y_std, alpha=0.6, s=50)
        ax3.set_xlabel(f'予測値 ({target_column})')
        ax3.set_ylabel('予測の標準偏差')
        ax3.set_title('予測の不確実性')
        ax3.grid(True, alpha=0.3)
    else:
        ax3.text(0.5, 0.5, '不確実性計算なし', ha='center', va='center', fontsize=12)
        ax3.set_axis_off()
    
    # 4. 時系列プロット（インデックス順）
    ax4 = axes[1, 1]
    indices = range(len(y_test))
    ax4.plot(indices, y_test, 'o-', label='実際の値', alpha=0.7)
    ax4.plot(indices, y_pred, 's-', label='予測値', alpha=0.7)
    if y_std is not None:
        ax4.fill_between(indices, y_pred - 2*y_std, y_pred + 2*y_std, alpha=0.3, label='95%信頼区間')
    ax4.set_xlabel('データポイント')
    ax4.set_ylabel(target_column)
    ax4.set_title('時系列での比較')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"結果を '{output_file}' に保存しました。")
    else:
        plt.show()
    
    plt.close()

def plot_feature_importance(model, feature_columns, output_file=None):
    """
    特徴量の重要度を可視化する（カーネルパラメータから推定）
    
    Args:
        model: 訓練済みのガウス過程モデル
        feature_columns: 特徴量の列名
        output_file: 出力ファイル名
    """
    # フォントサイズの設定
    plt.rcParams.update({'font.size': 20})
    # カーネルパラメータから特徴量の重要度を推定
    length_scales = None
    # sklearn
    if hasattr(model, "kernel_") and hasattr(model.kernel_, "k1") and hasattr(model.kernel_.k1, "length_scale"):
        length_scales = model.kernel_.k1.length_scale
        if np.isscalar(length_scales):
            length_scales = [length_scales] * len(feature_columns)
        length_scales = np.array(length_scales, dtype=float)
    # gpytorch wrapper
    elif hasattr(model, "get_lengthscales"):
        try:
            ls = model.get_lengthscales(len(feature_columns))
            if ls is not None:
                length_scales = np.array(ls, dtype=float)
        except Exception:
            length_scales = None

    if length_scales is not None and len(length_scales) == len(feature_columns):
        importance = 1.0 / np.maximum(length_scales, 1e-12)
    else:
        # その他のカーネルの場合は均等に設定
        importance = np.ones(len(feature_columns), dtype=float)
    
    # 正規化
    importance = importance / np.sum(importance)
    
    # プロット
    plt.figure(figsize=(20, 12))
    bars = plt.bar(feature_columns, importance)
    plt.xlabel('特徴量')
    plt.ylabel('相対重要度')
    plt.title('特徴量の重要度（カーネルパラメータから推定）')
    plt.xticks(rotation=45)
    plt.grid(True, alpha=0.3)
    
    # バーの上に値を表示
    for bar, imp in zip(bars, importance):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                f'{imp:.3f}', ha='center', va='bottom')
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"特徴量重要度を '{output_file}' に保存しました。")
    else:
        plt.show()
    
    plt.close()

def parse_fixed_values(fixes, feature_columns, df_clean):
    fixed = {c: float(df_clean[c].median()) for c in feature_columns}
    if fixes:
        for item in fixes:
            if '=' not in item:
                continue
            col, val = item.split('=', 1)
            col = col.strip()
            if col in fixed:
                try:
                    fixed[col] = float(val)
                except Exception:
                    pass
    return fixed


def parse_fix_overrides(fixes, feature_columns):
    """
    `--fix col=value` の「明示指定されたものだけ」を辞書で返す。
    - feature_columns に存在する列のみ対象
    - value は float 変換できるもののみ対象（できない場合は無視）
    """
    overrides = {}
    if not fixes:
        return overrides
    for item in fixes:
        if not item or ('=' not in item):
            continue
        col, val = item.split('=', 1)
        col = col.strip()
        if col not in feature_columns:
            continue
        try:
            overrides[col] = float(val)
        except Exception:
            continue
    return overrides

def parse_ranges(ranges, feature_columns, df_clean):
    mins = {c: float(df_clean[c].min()) for c in feature_columns}
    maxs = {c: float(df_clean[c].max()) for c in feature_columns}
    if ranges:
        for item in ranges:
            if ':' not in item:
                continue
            col, span = item.split(':', 1)
            if ',' not in span:
                continue
            lo, hi = span.split(',', 1)
            col = col.strip()
            if col in mins:
                try:
                    mins[col] = float(lo)
                    maxs[col] = float(hi)
                except Exception:
                    pass
    return mins, maxs

def apply_facet_filters(df_clean, filters):
    """
    'col:min,max' の形式のフィルタ配列を df_clean に適用して返す。
    無効な指定はスキップする。
    """
    if not filters:
        return df_clean
    df_out = df_clean.copy()
    for item in filters:
        try:
            if ':' not in item:
                continue
            col, span = item.split(':', 1)
            if ',' not in span:
                continue
            lo, hi = span.split(',', 1)
            col = col.strip()
            lo = float(lo)
            hi = float(hi)
            if col not in df_out.columns:
                continue
            df_out = df_out[(df_out[col] >= lo) & (df_out[col] <= hi)]
        except Exception:
            continue
    return df_out

def generate_pairwise_heatmaps(
    model,
    feature_columns,
    df_clean,
    scaler,
    grid_size,
    fixes,
    ranges,
    output_prefix,
    include_std,
    overlay_raw=False,
    contour_lines=False,
    contour_levels=10,
    contour_color='k',
    contour_linewidth=0.8,
    contour_alpha=0.8,
    heatmap_filters=None,
):
    # 目的変数名は呼び出し元から渡される想定（関数属性に設定される）
    target_column = getattr(generate_pairwise_heatmaps, "_target_column", "torque_x")

    # 非表示軸のレンジ指定は「固定値のレンジ」ではなく、データの事前フィルタとして扱う
    # 例: --heatmap-filter wall_spacing:1.4,1.8
    df_hm = apply_facet_filters(df_clean, heatmap_filters)
    if df_hm is None or df_hm.empty:
        print("[pairwise-heatmaps] heatmap-filter により有効なデータがありません。スキップします。")
        return

    fixed_values = parse_fixed_values(fixes, feature_columns, df_hm)
    mins, maxs = parse_ranges(ranges, feature_columns, df_hm)

    for i in range(len(feature_columns)):
        for j in range(i + 1, len(feature_columns)):
            fi = feature_columns[i]
            fj = feature_columns[j]

            xi = np.linspace(mins[fi], maxs[fi], grid_size)
            xj = np.linspace(mins[fj], maxs[fj], grid_size)
            XI, XJ = np.meshgrid(xi, xj)

            # メッシュから特徴行列を構成
            X_grid = np.zeros((grid_size * grid_size, len(feature_columns)))
            for k, fk in enumerate(feature_columns):
                if fk == fi:
                    X_grid[:, k] = XI.ravel()
                elif fk == fj:
                    X_grid[:, k] = XJ.ravel()
                else:
                    X_grid[:, k] = fixed_values[fk]

            if scaler is not None:
                X_infer = scaler.transform(X_grid)
            else:
                X_infer = X_grid

            y_mean, y_std = model.predict(X_infer, return_std=True)
            Zm = y_mean.reshape(grid_size, grid_size)
            Zs = y_std.reshape(grid_size, grid_size)

            # 平均のヒートマップ（このペアの予測レンジに合わせてスケール）
            plt.figure(figsize=(12, 10))
            plt.rcParams.update({'font.size': 18})
            zmin = float(np.nanmin(Zm))
            zmax = float(np.nanmax(Zm))
            if not np.isfinite(zmin) or not np.isfinite(zmax):
                zmin, zmax = 0.0, 1.0
            if zmin == zmax:
                eps = 1e-6
                zmin -= eps
                zmax += eps
            im_mean = plt.imshow(
                Zm,
                origin='lower',
                aspect='auto',
                extent=[mins[fi], maxs[fi], mins[fj], maxs[fj]],
                cmap='viridis',
                vmin=zmin,
                vmax=zmax
            )
            # 等高線（予測平均のみに重畳）
            if contour_lines:
                try:
                    cs = plt.contour(
                        XI,
                        XJ,
                        Zm,
                        levels=int(contour_levels) if isinstance(contour_levels, (int, np.integer)) else contour_levels,
                        colors=contour_color,
                        linewidths=contour_linewidth,
                        alpha=contour_alpha,
                        zorder=2
                    )
                    try:
                        plt.clabel(cs, inline=True, fmt='%.2g', fontsize=12, colors=contour_color)
                    except Exception:
                        pass
                    # 0レベルの等高線を赤で強調
                    if zmin <= 0.0 <= zmax:
                        try:
                            cs0 = plt.contour(
                                XI,
                                XJ,
                                Zm,
                                levels=[0.0],
                                colors='r',
                                linewidths=max(contour_linewidth * 1.5, 1.5),
                                alpha=1.0,
                                zorder=4
                            )
                            try:
                                plt.clabel(cs0, inline=True, fmt='0', fontsize=12, colors='r')
                            except Exception:
                                pass
                        except Exception:
                            pass
                except Exception:
                    pass
            if overlay_raw:
                xi_raw = df_hm[fi].values
                xj_raw = df_hm[fj].values
                mask_raw = np.isfinite(xi_raw) & np.isfinite(xj_raw)
                plt.scatter(
                    xi_raw[mask_raw],
                    xj_raw[mask_raw],
                    marker='x',
                    color='white',
                    s=20,
                    alpha=0.5,
                    linewidths=0.7,
                    zorder=3
                )
            plt.colorbar(im_mean, label=f'予測平均 ({target_column})')
            plt.xlabel(fi)
            plt.ylabel(fj)
            plt.title(f'予測ヒートマップ: {fi} vs {fj}')
            if output_prefix:
                out = f"{output_prefix}_results_{fi}__{fj}.png"
                plt.savefig(out, dpi=200, bbox_inches='tight')
                print(f"ヒートマップを保存: {out}")
                plt.close()
            else:
                plt.show()

            if include_std:
                plt.figure(figsize=(12, 10))
                plt.rcParams.update({'font.size': 18})
                im_std = plt.imshow(Zs, origin='lower', aspect='auto',
                           extent=[mins[fi], maxs[fi], mins[fj], maxs[fj]],
                           cmap='magma')
                if overlay_raw:
                    xi_raw = df_hm[fi].values
                    xj_raw = df_hm[fj].values
                    mask_raw = np.isfinite(xi_raw) & np.isfinite(xj_raw)
                    plt.scatter(
                        xi_raw[mask_raw],
                        xj_raw[mask_raw],
                        marker='x',
                        color='white',
                        s=20,
                        alpha=0.5,
                        linewidths=0.7,
                        zorder=3
                    )
                plt.colorbar(im_std, label='予測標準偏差')
                plt.xlabel(fi)
                plt.ylabel(fj)
                plt.title(f'標準偏差ヒートマップ: {fi} vs {fj}')
                if output_prefix:
                    out = f"{output_prefix}_std_{fi}__{fj}.png"
                    plt.savefig(out, dpi=200, bbox_inches='tight')
                    print(f"標準偏差ヒートマップを保存: {out}")
                    plt.close()
                else:
                    plt.show()


def _parse_float_pair(s, name):
    if s is None:
        raise ValueError(f"{name} が未指定です")
    if isinstance(s, (tuple, list)) and len(s) == 2:
        return float(s[0]), float(s[1])
    t = str(s).strip()
    if "," not in t:
        raise ValueError(f"{name} は 'a,b' 形式で指定してください（入力='{s}'）")
    a, b = t.split(",", 1)
    return float(a), float(b)


def generate_facet_heatmap_1pair(
    model,
    feature_columns,
    df_clean,
    scaler,
    *,
    heatmap_x,
    heatmap_y,
    facet_by,
    facet_range,
    facet_step,
    facet_by2=None,
    facet_range2=None,
    facet_step2=None,
    grid_size=80,
    fixes=None,
    ranges=None,
    output_file=None,
    overlay_raw=False,
    contour_lines=False,
    contour_levels=10,
    contour_color="k",
    contour_linewidth=0.8,
    contour_alpha=0.8,
    heatmap_filters=None,
    std_heatmap=False,
):
    """
    1ペア(heatmap_x, heatmap_y)について、facet_by（+任意でfacet_by2）を区間分割してファセット描画する。
    - facet_by のみ: 1D（横方向にビンを並べる）
    - facet_by + facet_by2: 2D（行=facet_by2, 列=facet_by）
    - 各ビンで df を絞り、他特徴量はそのビンの中央値（+ --fix 上書き）で固定して予測
    - カラースケールは全セル共通（比較しやすい）
    """
    target_column = getattr(generate_facet_heatmap_1pair, "_target_column", "torque_x")

    if heatmap_x not in feature_columns:
        raise ValueError(f"--heatmap-x '{heatmap_x}' が学習特徴量に含まれていません: {feature_columns}")
    if heatmap_y not in feature_columns:
        raise ValueError(f"--heatmap-y '{heatmap_y}' が学習特徴量に含まれていません: {feature_columns}")
    if facet_by not in df_clean.columns:
        raise ValueError(f"--heatmap-facet-by '{facet_by}' がデータに存在しません。")
    use_2d = bool(facet_by2) or bool(facet_range2) or (facet_step2 is not None)
    if use_2d:
        if not facet_by2 or facet_by2 not in df_clean.columns:
            raise ValueError(f"--heatmap-facet-by2 '{facet_by2}' がデータに存在しません。")
        if facet_range2 is None:
            raise ValueError("--heatmap-facet-range2 は必須です（2Dファセット）。")
        if facet_step2 is None:
            raise ValueError("--heatmap-facet-step2 は必須です（2Dファセット）。")

    # 事前フィルタ（共通）
    df_base = apply_facet_filters(df_clean, heatmap_filters)
    if df_base is None or df_base.empty:
        print("[heatmap-facet] heatmap-filter により有効なデータがありません。スキップします。")
        return

    a, b = _parse_float_pair(facet_range, "--heatmap-facet-range")
    step = float(facet_step)
    if not np.isfinite(step) or step <= 0:
        raise ValueError("--heatmap-facet-step は正の数を指定してください。")

    # ビン境界（右端は含むように最後だけ少し広げる）
    edges = list(np.arange(a, b + 1e-12, step))
    if len(edges) < 2:
        raise ValueError("facet range/step からビンを作れません。range/step を見直してください。")
    bins_x = [(edges[i], edges[i + 1]) for i in range(len(edges) - 1)]

    bins_y = [(None, None)]
    if use_2d:
        a2, b2 = _parse_float_pair(facet_range2, "--heatmap-facet-range2")
        step2 = float(facet_step2)
        if not np.isfinite(step2) or step2 <= 0:
            raise ValueError("--heatmap-facet-step2 は正の数を指定してください。")
        edges2 = list(np.arange(a2, b2 + 1e-12, step2))
        if len(edges2) < 2:
            raise ValueError("facet2 range/step からビンを作れません。range2/step2 を見直してください。")
        bins_y = [(edges2[i], edges2[i + 1]) for i in range(len(edges2) - 1)]

    # 軸範囲（全ビン共通）: --range があればそれ、無ければ df_base の min/max
    mins_all, maxs_all = parse_ranges(ranges, feature_columns, df_base)
    x_min, x_max = float(mins_all[heatmap_x]), float(maxs_all[heatmap_x])
    y_min, y_max = float(mins_all[heatmap_y]), float(maxs_all[heatmap_y])
    if x_min == x_max:
        x_min -= 1e-6
        x_max += 1e-6
    if y_min == y_max:
        y_min -= 1e-6
        y_max += 1e-6

    xi = np.linspace(x_min, x_max, int(grid_size))
    yj = np.linspace(y_min, y_max, int(grid_size))
    XI, YJ = np.meshgrid(xi, yj)

    # 1pass: 各ビンの予測を計算して保持し、全体の zmin/zmax を決める
    panes = []  # list of dicts with keys: r, c, lo, hi, lo2, hi2, df, Zm, Zs
    zmins = []
    zmaxs = []

    for r_idx, (lo2, hi2) in enumerate(bins_y):
        for c_idx, (lo, hi) in enumerate(bins_x):
            # facet_by の絞り込み（左閉右開、最後のみ右端含む）
            if hi >= b - 1e-12:
                m1 = (df_base[facet_by] >= lo) & (df_base[facet_by] <= hi)
            else:
                m1 = (df_base[facet_by] >= lo) & (df_base[facet_by] < hi)

            # facet_by2 の絞り込み（2D時のみ）
            if use_2d:
                # right edge for facet2
                a2, b2 = _parse_float_pair(facet_range2, "--heatmap-facet-range2")
                if hi2 >= b2 - 1e-12:
                    m2 = (df_base[facet_by2] >= lo2) & (df_base[facet_by2] <= hi2)
                else:
                    m2 = (df_base[facet_by2] >= lo2) & (df_base[facet_by2] < hi2)
                df_bin = df_base[m1 & m2]
            else:
                df_bin = df_base[m1]

            if df_bin.empty:
                panes.append({"r": r_idx, "c": c_idx, "lo": lo, "hi": hi, "lo2": lo2, "hi2": hi2, "df": df_bin, "Zm": None, "Zs": None})
                continue

            fixed_values = parse_fixed_values(fixes, feature_columns, df_bin)
            # メッシュから特徴行列を構成
            X_grid = np.zeros((grid_size * grid_size, len(feature_columns)), dtype=float)
            for k, fk in enumerate(feature_columns):
                if fk == heatmap_x:
                    X_grid[:, k] = XI.ravel()
                elif fk == heatmap_y:
                    X_grid[:, k] = YJ.ravel()
                else:
                    X_grid[:, k] = fixed_values[fk]

            if scaler is not None:
                X_infer = scaler.transform(X_grid)
            else:
                X_infer = X_grid

            y_mean, y_std = model.predict(X_infer, return_std=True)
            Zm = y_mean.reshape(grid_size, grid_size)
            Zs = y_std.reshape(grid_size, grid_size) if y_std is not None else None
            panes.append({"r": r_idx, "c": c_idx, "lo": lo, "hi": hi, "lo2": lo2, "hi2": hi2, "df": df_bin, "Zm": Zm, "Zs": Zs})

            zmins.append(float(np.nanmin(Zm)))
            zmaxs.append(float(np.nanmax(Zm)))

    if not zmins or not zmaxs:
        print("[heatmap-facet] 全セルが空のためスキップします。")
        return

    zmin = float(np.nanmin(zmins))
    zmax = float(np.nanmax(zmaxs))
    if not np.isfinite(zmin) or not np.isfinite(zmax):
        zmin, zmax = 0.0, 1.0
    if zmin == zmax:
        zmin -= 1e-6
        zmax += 1e-6

    # subplot layout
    n_rows = len(bins_y)
    n_cols = len(bins_x)

    plt.rcParams.update({"font.size": 14})
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(4.2 * n_cols, 3.6 * n_rows), squeeze=False)

    last_im = None
    # panes を (r,c) で引けるように
    pane_map = {(p["r"], p["c"]): p for p in panes}
    for r in range(n_rows):
        for c in range(n_cols):
            ax = axes[r, c]
            pane = pane_map.get((r, c), None)
            if pane is None:
                ax.set_axis_off()
                continue

            lo = pane["lo"]
            hi = pane["hi"]
            lo2 = pane["lo2"]
            hi2 = pane["hi2"]
            df_bin = pane["df"]
            Zm = pane["Zm"]
            Zs = pane["Zs"]

            if use_2d:
                ax.set_title(f"{facet_by}: [{lo:g},{hi:g})\n{facet_by2}: [{lo2:g},{hi2:g})  n={len(df_bin)}")
            else:
                ax.set_title(f"{facet_by}: [{lo:g}, {hi:g})  n={len(df_bin)}")
            if Zm is None:
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_frame_on(False)
                continue

            Z_plot = Zs if bool(std_heatmap) else Zm
            cmap = "magma" if bool(std_heatmap) else "viridis"
            vmin = None
            vmax = None
            if not bool(std_heatmap):
                vmin, vmax = zmin, zmax

            last_im = ax.imshow(
                Z_plot,
                origin="lower",
                aspect="auto",
                extent=[x_min, x_max, y_min, y_max],
                cmap=cmap,
                vmin=vmin,
                vmax=vmax,
                zorder=1,
            )

            # contour (mean のみ)
            if contour_lines and (not bool(std_heatmap)):
                try:
                    cs = ax.contour(
                        XI,
                        YJ,
                        Zm,
                        levels=int(contour_levels) if isinstance(contour_levels, (int, np.integer)) else contour_levels,
                        colors=contour_color,
                        linewidths=contour_linewidth,
                        alpha=contour_alpha,
                        zorder=2,
                    )
                    try:
                        ax.clabel(cs, inline=True, fmt="%.2g", fontsize=10, colors=contour_color)
                    except Exception:
                        pass
                    if zmin <= 0.0 <= zmax:
                        try:
                            ax.contour(
                                XI,
                                YJ,
                                Zm,
                                levels=[0.0],
                                colors="r",
                                linewidths=max(contour_linewidth * 1.5, 1.5),
                                alpha=1.0,
                                zorder=3,
                            )
                        except Exception:
                            pass
                except Exception:
                    pass

            if overlay_raw:
                try:
                    xr = df_bin[heatmap_x].values
                    yr = df_bin[heatmap_y].values
                    m = np.isfinite(xr) & np.isfinite(yr)
                    ax.scatter(
                        xr[m],
                        yr[m],
                        marker="x",
                        color="white",
                        s=18,
                        alpha=0.55,
                        linewidths=0.7,
                        zorder=4,
                    )
                except Exception:
                    pass

            # labels (最小限)
            if r == n_rows - 1:
                ax.set_xlabel(heatmap_x)
            else:
                ax.set_xlabel("")
            if c == 0:
                ax.set_ylabel(heatmap_y)
            else:
                ax.set_ylabel("")

    if last_im is not None:
        label = "予測標準偏差" if bool(std_heatmap) else f"予測平均 ({target_column})"
        fig.colorbar(last_im, ax=axes.ravel().tolist(), label=label, shrink=0.95)

    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=200, bbox_inches="tight")
        print(f"ファセットヒートマップを保存: {output_file}")
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)

def plot_grouped_raw_and_fit_gpr(
    df_clean,
    feature_columns,
    model,
    scaler,
    group_by,
    curve_x=None,
    curve_points=200,
    ranges=None,
    output_file=None,
    show_uncertainty=True,
    hue=None,
    fixes=None,
    raw_alpha=0.5,
    fit_no_extrapolate=True,
):
    """
    指定した group_by 特徴量でデータをグループ化し、各グループで
    - 生の散布 (curve_x vs torque_x)
    - GPR のフィット曲線 (curve_x を掃引、他特徴量はグループの中央値固定)
    - show_uncertainty が True の場合は ±2σ の不確実性帯を表示
    を同一サブプロット上に描画する。
    """
    try:
        raw_alpha = float(raw_alpha)
    except Exception as e:
        raise ValueError(f"raw_alpha は float に変換可能な値を指定してください: {raw_alpha}") from e
    if (not np.isfinite(raw_alpha)) or (raw_alpha < 0.0) or (raw_alpha > 1.0):
        raise ValueError(f"raw_alpha は 0〜1 の範囲で指定してください: {raw_alpha}")
    if not group_by or len(group_by) == 0:
        print("[plot_grouped_raw_and_fit_gpr] group_by が指定されていないためスキップします。")
        return

    for col in group_by:
        if col not in df_clean.columns:
            raise ValueError(f"group_by 列 '{col}' がデータに存在しません。")

    # 目的変数名は呼び出し元から渡される想定（関数属性に設定される）
    target_column = getattr(plot_grouped_raw_and_fit_gpr, "_target_column", "torque_x")
    # x 軸の候補決定
    if curve_x is None:
        candidates = [c for c in feature_columns if c not in group_by]
        if not candidates:
            raise ValueError("curve_x を自動決定できません。group_by 以外の特徴量がありません。--curve-x を指定してください。")
        curve_x = candidates[0]
    if curve_x not in feature_columns:
        raise ValueError(f"curve_x '{curve_x}' は学習特徴量に含まれていません。feature_columns={feature_columns}")

    fit_no_extrapolate = bool(fit_no_extrapolate)

    # hue の検証（指定時）
    if hue is not None:
        hue = str(hue).strip()
        if not hue:
            hue = None
    if hue:
        if hue not in df_clean.columns:
            raise ValueError(f"hue '{hue}' がデータに存在しません。")
        if hue == curve_x:
            raise ValueError("hue と curve_x は同一にできません。別の列を指定してください。")
        if hue in group_by:
            raise ValueError("hue は group_by に含められません（同一プロット内の色分け用の列を指定してください）。")
        # hue は「グルーピング用」として扱う。文字列カテゴリ（数値化できない値を含む）は弾く。
        hue_raw = df_clean[hue]
        hue_num = pd.to_numeric(hue_raw, errors="coerce")
        bad = (~hue_raw.isna()) & (hue_num.isna())
        if bool(bad.any()):
            # 例示（最大5件）
            examples = hue_raw[bad].head(5).tolist()
            raise ValueError(
                f"hue '{hue}' は数値列のみ対応です（文字列カテゴリは不可）。"
                f" 数値化できない値の例: {examples}"
            )
        if int(hue_num.dropna().shape[0]) == 0:
            print(f"[plot_grouped_raw_and_fit_gpr] hue='{hue}' が指定されましたが、有効な数値がありません（全てNaN?）。hue無しで描画します。")
            hue = None

    fix_overrides = parse_fix_overrides(fixes, feature_columns)

    # hue の色割当（全部描画）
    if hue:
        hue_series = pd.to_numeric(df_clean[hue], errors="coerce")
        hue_values = list(pd.unique(hue_series.dropna()))
        try:
            hue_values = sorted(hue_values)
        except Exception:
            try:
                hue_values = sorted(hue_values, key=lambda v: str(v))
            except Exception:
                pass
        if len(hue_values) > 30:
            print(f"[plot_grouped_raw_and_fit_gpr] 注意: hue='{hue}' のユニーク値が {len(hue_values)} 個あります。指定通り全て描画します（凡例が大きくなる可能性）。")
        cmap = plt.get_cmap("tab20")
        hue_to_color = {v: cmap(i % cmap.N) for i, v in enumerate(hue_values)}
    else:
        hue_values = []
        hue_to_color = {}

    # レンジ決定
    if ranges is not None and isinstance(ranges, tuple) and len(ranges) == 2:
        mins, maxs = ranges
    else:
        mins = {c: float(df_clean[c].min()) for c in feature_columns}
        maxs = {c: float(df_clean[c].max()) for c in feature_columns}

    # グループを作成
    group_keys = df_clean[group_by].drop_duplicates()
    n_groups = len(group_keys)
    if n_groups == 0:
        print("[plot_grouped_raw_and_fit_gpr] グループが見つかりません。")
        return

    n_cols = min(4, n_groups)
    n_rows = int(np.ceil(n_groups / n_cols))
    plt.rcParams.update({'font.size': 18})
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(6.0 * n_cols, 5.0 * n_rows), squeeze=False)
    # ヘッダのクリッピングを避けるため余白を確保（行ヘッダをさらに左へ配置）
    fig.subplots_adjust(left=0.46, top=0.90, wspace=0.30, hspace=0.40)

    for idx, (_, gvals) in enumerate(group_keys.iterrows()):
        r = idx // n_cols
        c = idx % n_cols
        ax = axes[r, c]

        # グループ条件で抽出
        mask = np.ones(len(df_clean), dtype=bool)
        title_parts = []
        for col in group_by:
            val = gvals[col]
            mask &= (df_clean[col] == val)
            if isinstance(val, float):
                title_parts.append(f"{col}={val:g}")
            else:
                title_parts.append(f"{col}={val}")
        df_group = df_clean[mask]

        if df_group.empty:
            ax.set_axis_off()
            continue

        # 予測用グリッド（curve_x を掃引）
        # - fit_no_extrapolate=True: グループ実データの範囲内だけ
        # - False: 既存どおり --range(=mins/maxs) を優先（比較用に広げる用途）
        x_series_group = pd.to_numeric(df_group[curve_x], errors="coerce")
        if fit_no_extrapolate:
            x_min = float(np.nanmin(x_series_group.values))
            x_max = float(np.nanmax(x_series_group.values))
        else:
            x_min = mins.get(curve_x, float(np.nanmin(x_series_group.values)))
            x_max = maxs.get(curve_x, float(np.nanmax(x_series_group.values)))
        if (not np.isfinite(x_min)) or (not np.isfinite(x_max)) or (x_min == x_max):
            # データが無い/一点しかない場合は fit 曲線を描けない（rawのみ）
            x_grid_group = None
        else:
            x_grid_group = np.linspace(x_min, x_max, int(curve_points))

        if hue:
            # hue ごとに raw + fit（同色）を描画（A: ±2σ も hue ごとに描画）
            hue_num_g = pd.to_numeric(df_group[hue], errors="coerce")
            for hv in hue_values:
                df_h = df_group[hue_num_g == hv]
                if df_h.empty:
                    continue
                color = hue_to_color.get(hv, "C0")

                # raw
                if curve_x not in df_h.columns:
                    raise ValueError(f"x 軸列 '{curve_x}' がデータに存在しません。")
                x_raw = df_h[curve_x].values
                y_raw = df_h[target_column].values
                ax.scatter(x_raw, y_raw, alpha=raw_alpha, s=25, color=color)

                # fit: hue はグルーピング用。学習特徴量に含まれる場合のみ fcol==hue で hv に固定される。
                hv_f = float(hv)
                forced_cols = set(group_by) | {curve_x, hue}
                # hueごとの x_grid（補外禁止時は hueグループの範囲内だけ）
                x_series_h = pd.to_numeric(df_h[curve_x], errors="coerce")
                if fit_no_extrapolate:
                    xh_min = float(np.nanmin(x_series_h.values))
                    xh_max = float(np.nanmax(x_series_h.values))
                    if (not np.isfinite(xh_min)) or (not np.isfinite(xh_max)) or (xh_min == xh_max):
                        continue
                    x_grid = np.linspace(xh_min, xh_max, int(curve_points))
                else:
                    if x_grid_group is None:
                        continue
                    x_grid = x_grid_group
                X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
                for j, fcol in enumerate(feature_columns):
                    if fcol == curve_x:
                        X_grid[:, j] = x_grid
                    elif fcol == hue:
                        X_grid[:, j] = hv_f
                    elif fcol in group_by:
                        X_grid[:, j] = float(df_h[fcol].median())
                    elif (fcol in fix_overrides) and (fcol not in forced_cols):
                        X_grid[:, j] = float(fix_overrides[fcol])
                    else:
                        med = df_h[fcol].median()
                        if not np.isfinite(med):
                            med = df_group[fcol].median()
                        X_grid[:, j] = float(med)

                if scaler is not None:
                    try:
                        X_infer = scaler.transform(X_grid)
                    except Exception:
                        X_infer = X_grid
                else:
                    X_infer = X_grid

                if show_uncertainty:
                    y_mean, y_std = model.predict(X_infer, return_std=True)
                    ax.plot(x_grid, y_mean, color=color, lw=2.0, alpha=0.95)
                    ax.fill_between(x_grid, y_mean - 2.0*y_std, y_mean + 2.0*y_std, color=color, alpha=0.18)
                else:
                    y_mean = model.predict(X_infer, return_std=False)
                    ax.plot(x_grid, y_mean, color=color, lw=2.0, alpha=0.95)
        else:
            # 従来どおり 1色 raw + 1本 fit（±2σ）
            if curve_x not in df_group.columns:
                raise ValueError(f"x 軸列 '{curve_x}' がデータに存在しません。")
            x_raw = df_group[curve_x].values
            y_raw = df_group[target_column].values
            ax.scatter(x_raw, y_raw, alpha=raw_alpha, s=25, label='raw')

            forced_cols = set(group_by) | {curve_x}
            if x_grid_group is None:
                # rawのみ
                continue
            x_grid = x_grid_group
            X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
            for j, fcol in enumerate(feature_columns):
                if fcol == curve_x:
                    X_grid[:, j] = x_grid
                elif fcol in group_by:
                    X_grid[:, j] = float(df_group[fcol].median())
                elif (fcol in fix_overrides) and (fcol not in forced_cols):
                    X_grid[:, j] = float(fix_overrides[fcol])
                else:
                    X_grid[:, j] = float(df_group[fcol].median())

            if scaler is not None:
                try:
                    X_infer = scaler.transform(X_grid)
                except Exception:
                    X_infer = X_grid
            else:
                X_infer = X_grid

            if show_uncertainty:
                y_mean, y_std = model.predict(X_infer, return_std=True)
                ax.plot(x_grid, y_mean, color='C1', lw=2.0, label='GPR fit')
                ax.fill_between(x_grid, y_mean - 2.0*y_std, y_mean + 2.0*y_std, color='C1', alpha=0.2, label='±2σ')
            else:
                y_mean = model.predict(X_infer, return_std=False)
                ax.plot(x_grid, y_mean, color='C1', lw=2.0, label='GPR fit')

        ax.set_title(', '.join(title_parts))
        ax.set_xlabel(curve_x)
        ax.set_ylabel(target_column)
        ax.grid(True, alpha=0.3)
        if not hue:
            ax.legend(frameon=True, fontsize=10)

    # 余白のサブプロットを非表示
    for k in range(n_groups, n_rows * n_cols):
        r = k // n_cols
        c = k % n_cols
        axes[r, c].set_axis_off()

    # hue 指定時は、図全体で1つの凡例にまとめる
    if hue and hue_values:
        handles = []
        for hv in hue_values:
            color = hue_to_color.get(hv, "C0")
            if isinstance(hv, (float, np.floating)):
                label = f"{hue}={hv:g}"
            else:
                label = f"{hue}={hv}"
            handles.append(mlines.Line2D([], [], color=color, marker='o', linestyle='-', lw=2.0, markersize=6, label=label))
        fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=min(6, max(1, len(handles))), frameon=True, fontsize=10)
        plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    else:
        plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"グループ別 Raw vs GPR Fit を '{output_file}' に保存しました。")
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)

def plot_facet_raw_and_fit_gpr(
    df_clean,
    feature_columns,
    model,
    scaler,
    row_group_by,
    col_group_by,
    curve_x=None,
    curve_points=200,
    ranges=None,
    output_file=None,
    show_uncertainty=True,
    hue=None,
    fixes=None,
    facet_drone=False,
    facet_drone_alpha=0.12,
    facet_drone_force_2d=False,
    facet_drone_dpi=160,
    facet_drone_mode="background",  # 'background' | 'inset' | 'both'
    facet_drone_inset_loc="ur",      # 'ur'|'ul'|'lr'|'ll'
    facet_drone_inset_size=0.33,     # fraction of axes (0..1)
    facet_drone_inset_alpha=1.0,
    raw_alpha=0.5,
    fit_no_extrapolate=True,
):
    """
    行方向(row_group_byの組) × 列方向(col_group_byの組)のファセットで、
    各セルに Raw 散布と GPR フィット（±2σ）の曲線を描画する。
    指定順でソートして並べる。
    """
    try:
        raw_alpha = float(raw_alpha)
    except Exception as e:
        raise ValueError(f"raw_alpha は float に変換可能な値を指定してください: {raw_alpha}") from e
    if (not np.isfinite(raw_alpha)) or (raw_alpha < 0.0) or (raw_alpha > 1.0):
        raise ValueError(f"raw_alpha は 0〜1 の範囲で指定してください: {raw_alpha}")
    if not row_group_by and not col_group_by:
        print("[plot_facet_raw_and_fit_gpr] row/col が未指定のためスキップします。")
        return

    # 目的変数名は呼び出し元から渡される想定（関数属性に設定される）
    target_column = getattr(plot_facet_raw_and_fit_gpr, "_target_column", "torque_x")
    # x 軸の決定
    if curve_x is None:
        excluded = set((row_group_by or []) + (col_group_by or []))
        candidates = [c for c in feature_columns if c not in excluded]
        if not candidates:
            raise ValueError("curve_x を自動決定できません。--curve-x を指定してください。")
        curve_x = candidates[0]
    if curve_x not in feature_columns:
        raise ValueError(f"curve_x '{curve_x}' は学習特徴量に含まれていません。feature_columns={feature_columns}")

    fit_no_extrapolate = bool(fit_no_extrapolate)

    # hue の検証（指定時）
    if hue is not None:
        hue = str(hue).strip()
        if not hue:
            hue = None
    if hue:
        if hue not in df_clean.columns:
            raise ValueError(f"hue '{hue}' がデータに存在しません。")
        if hue == curve_x:
            raise ValueError("hue と curve_x は同一にできません。別の列を指定してください。")
        if (row_group_by and hue in row_group_by) or (col_group_by and hue in col_group_by):
            raise ValueError("hue は row_group_by / col_group_by に含められません（セル内の色分け用の列を指定してください）。")

        # hue は「グルーピング用」として扱う。文字列カテゴリ（数値化できない値を含む）は弾く。
        hue_raw = df_clean[hue]
        hue_series = pd.to_numeric(hue_raw, errors="coerce")
        bad = (~hue_raw.isna()) & (hue_series.isna())
        if bool(bad.any()):
            examples = hue_raw[bad].head(5).tolist()
            raise ValueError(
                f"hue '{hue}' は数値列のみ対応です（文字列カテゴリは不可）。"
                f" 数値化できない値の例: {examples}"
            )
        hue_values = list(pd.unique(hue_series.dropna()))
        try:
            hue_values = sorted(hue_values)
        except Exception:
            try:
                hue_values = sorted(hue_values, key=lambda v: str(v))
            except Exception:
                pass
        if len(hue_values) == 0:
            print(f"[plot_facet_raw_and_fit_gpr] hue='{hue}' が指定されましたが、有効な値がありません（全てNaN?）。hue無しで描画します。")
            hue = None
            hue_values = []
            hue_to_color = {}
        else:
            if len(hue_values) > 30:
                print(f"[plot_facet_raw_and_fit_gpr] 注意: hue='{hue}' のユニーク値が {len(hue_values)} 個あります。指定通り全て描画します（凡例が大きくなる可能性）。")
            cmap = plt.get_cmap("tab20")
            hue_to_color = {v: cmap(i % cmap.N) for i, v in enumerate(hue_values)}
    else:
        hue_values = []
        hue_to_color = {}

    fix_overrides = parse_fix_overrides(fixes, feature_columns)

    # レンジ（全ファセットで共有するグローバルな範囲）
    if ranges is not None and isinstance(ranges, tuple) and len(ranges) == 2:
        mins, maxs = ranges
    else:
        mins = {c: float(df_clean[c].min()) for c in feature_columns}
        maxs = {c: float(df_clean[c].max()) for c in feature_columns}

    # 全ファセットで共通の X 軸（curve_x）範囲を決定
    global_x_min = mins.get(curve_x, float(df_clean[curve_x].min()))
    global_x_max = maxs.get(curve_x, float(df_clean[curve_x].max()))
    if not np.isfinite(global_x_min) or not np.isfinite(global_x_max):
        global_x_min = float(df_clean[curve_x].min())
        global_x_max = float(df_clean[curve_x].max())
    if global_x_min == global_x_max:
        global_x_min -= 1e-6
        global_x_max += 1e-6

    # 全ファセットで共通の Y 軸（目的変数）範囲を決定
    global_y_min = float(df_clean[target_column].min())
    global_y_max = float(df_clean[target_column].max())
    if not np.isfinite(global_y_min) or not np.isfinite(global_y_max):
        global_y_min = -1.0
        global_y_max = 1.0
    if global_y_min == global_y_max:
        global_y_min -= 1e-6
        global_y_max += 1e-6

    # キー生成のヘルパ
    def make_keys(group_cols):
        if not group_cols:
            return [()]  # 単一グループ
        # 指定順でユニークな組を作る
        uniq = df_clean[group_cols].drop_duplicates()
        # ファセットの並びが毎回変わらないように、group_cols の順で安定ソートする
        try:
            uniq = uniq.sort_values(by=group_cols, kind="mergesort", na_position="last")
        except Exception:
            # 型が混在して比較できない場合などは文字列化してソートを試みる（それでもダメなら未ソート）
            try:
                uniq2 = uniq.copy()
                for c in group_cols:
                    uniq2[c] = uniq2[c].astype(str)
                uniq = uniq2.sort_values(by=group_cols, kind="mergesort", na_position="last")
            except Exception:
                pass
        keys = [tuple(row[c] for c in group_cols) for _, row in uniq.iterrows()]
        return keys

    row_keys = make_keys(row_group_by)
    col_keys = make_keys(col_group_by)

    n_rows = max(1, len(row_keys))
    n_cols = max(1, len(col_keys))

    plt.rcParams.update({'font.size': 18})
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(6.0 * n_cols, 5.0 * n_rows), squeeze=False)

    # 各ファセットを描画
    for r_idx, rkey in enumerate(row_keys):
        for c_idx, ckey in enumerate(col_keys):
            ax = axes[r_idx, c_idx]

            # 先にヘッダ（行/列）を設定しておく：データが空でも表示されるようにする
            if (c_idx == 0) and row_group_by:
                # 行ヘッダは横表示＋改行でコンパクト化
                row_title = '\n'.join(
                    [f"{col}={val:g}" if isinstance(val, float) else f"{col}={val}"
                     for col, val in zip(row_group_by, rkey)]
                )
                ax.set_ylabel('torque_x')
                ax.annotate(
                    row_title,
                    xy=(-0.40, 0.5),
                    xycoords='axes fraction',
                    rotation=0,
                    ha='right',
                    va='center',
                    fontsize=18,
                    linespacing=1.0,
                    annotation_clip=False
                )
            else:
                ax.set_ylabel('torque_x')

            if (r_idx == 0) and col_group_by:
                col_title = '\n'.join(
                    [f"{col}={val:g}" if isinstance(val, float) else f"{col}={val}"
                     for col, val in zip(col_group_by, ckey)]
                )
                ax.set_title(col_title, fontsize=20)
            else:
                ax.set_title('')

            # マスク作成
            mask = np.ones(len(df_clean), dtype=bool)
            title_parts = []
            if row_group_by:
                for col, val in zip(row_group_by, rkey):
                    mask &= (df_clean[col] == val)
                    title_parts.append(f"{col}={val:g}" if isinstance(val, float) else f"{col}={val}")
            if col_group_by:
                for col, val in zip(col_group_by, ckey):
                    mask &= (df_clean[col] == val)
                    title_parts.append(f"{col}={val:g}" if isinstance(val, float) else f"{col}={val}")

            df_cell = df_clean[mask]

            # --- Facet background: morph drone image (faint) ---
            # row/col/hue から tilt/fold/slant を読み取って背景に敷く
            if bool(facet_drone):
                if bool(facet_drone_force_2d):
                    raise RuntimeError("facet-drone-force-2d は禁止されています（フォールバック禁止のため）。")
                mode = str(facet_drone_mode or "background").strip().lower()
                if mode not in ["background", "inset", "both"]:
                    raise ValueError(f"facet_drone_mode が不正です: {facet_drone_mode} (background|inset|both)")
                # 候補列名（プロジェクト内の命名に寄せる）
                fold_cols = {"fold_angle", "fold", "phi", "phi_deg"}
                slant_cols = {"slant_angle", "slant", "psi", "psi_deg"}
                tilt_cols = {"tilt_angle", "tilt", "theta", "theta_deg"}

                # row/col のキーから取得（最優先）
                psi = _infer_angle_from_group_keys(row_group_by, rkey, col_group_by, ckey, slant_cols)
                th = _infer_angle_from_group_keys(row_group_by, rkey, col_group_by, ckey, tilt_cols)
                phi = _infer_angle_from_group_keys(row_group_by, rkey, col_group_by, ckey, fold_cols)

                # hue が fold 系なら、セル内の hue 値から fold を選ぶ（row/colに fold が無い場合）
                if phi is None and hue in fold_cols:
                    phi = _infer_fold_from_hue(df_cell, hue_col=hue, global_hue_values=hue_values)

                # df の中央値フォールバック
                if psi is None:
                    psi = _infer_angle_from_df(df_cell, slant_cols)
                if th is None:
                    th = _infer_angle_from_df(df_cell, tilt_cols)
                if phi is None:
                    phi = _infer_angle_from_df(df_cell, fold_cols)

                # 最終フォールバック（未指定）
                if psi is None:
                    psi = 0.0
                if th is None:
                    th = 0.0
                if phi is None:
                    phi = 0.0

                try:
                    img = _get_facet_drone_rgba_cached(
                        phi_deg=phi,
                        psi_deg=psi,
                        theta_deg=th,
                        force_2d=bool(facet_drone_force_2d),
                        dpi=int(facet_drone_dpi),
                    )
                    # 1) background
                    if mode in ["background", "both"]:
                        ax.imshow(
                            img,
                            extent=[global_x_min, global_x_max, global_y_min, global_y_max],
                            aspect="auto",
                            interpolation="bilinear",
                            alpha=float(facet_drone_alpha),
                            zorder=0,
                        )
                    # 2) inset
                    if mode in ["inset", "both"]:
                        loc = str(facet_drone_inset_loc or "ur").strip().lower()
                        s = float(facet_drone_inset_size)
                        s = max(0.05, min(0.95, s))
                        pad = 0.02
                        if loc == "ul":
                            x0, y0 = pad, 1.0 - pad - s
                        elif loc == "lr":
                            x0, y0 = 1.0 - pad - s, pad
                        elif loc == "ll":
                            x0, y0 = pad, pad
                        else:  # ur
                            x0, y0 = 1.0 - pad - s, 1.0 - pad - s
                        try:
                            axins = ax.inset_axes([x0, y0, s, s], transform=ax.transAxes, zorder=8)
                        except Exception:
                            # fallback: older matplotlib
                            axins = ax.inset_axes([x0, y0, s, s])
                        axins.imshow(img, interpolation="bilinear", alpha=float(facet_drone_inset_alpha))
                        axins.set_axis_off()
                except Exception as e:
                    # 背景描画に失敗しても本筋のプロットは続行
                    print(f"[facet-drone] 背景描画に失敗: {e}")

            if df_cell.empty:
                # 空セルでもヘッダを見せるため軸は消さず、グリッド/目盛りのみ最小化
                ax.grid(False)
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_xlabel(curve_x)
                # 以降のプロット処理はスキップ
                continue

            if hue:
                # hue ごとに raw + fit（同色）を描画（A: ±2σ も hue ごとに描画）
                for hv in hue_values:
                    hue_num_cell = pd.to_numeric(df_cell[hue], errors="coerce")
                    df_h = df_cell[hue_num_cell == hv]
                    if df_h.empty:
                        continue
                    color = hue_to_color.get(hv, "C0")

                    # raw
                    if curve_x not in df_h.columns:
                        raise ValueError(f"x 軸列 '{curve_x}' がデータに存在しません。")
                    x_raw = df_h[curve_x].values
                    y_raw = df_h[target_column].values
                    ax.scatter(x_raw, y_raw, alpha=raw_alpha, s=25, color=color)

                    # fit: hue はグルーピング用。学習特徴量に含まれる場合のみ fcol==hue で hv に固定される。
                    hv_f = float(hv)

                    # x_grid（補外禁止時は hueグループの実データ範囲内だけ。無効ならセル共通のglobal範囲）
                    if fit_no_extrapolate:
                        x_series_h = pd.to_numeric(df_h[curve_x], errors="coerce")
                        xh_min = float(np.nanmin(x_series_h.values))
                        xh_max = float(np.nanmax(x_series_h.values))
                        if (not np.isfinite(xh_min)) or (not np.isfinite(xh_max)) or (xh_min == xh_max):
                            # rawのみ
                            continue
                        x_grid = np.linspace(xh_min, xh_max, int(curve_points))
                    else:
                        x_grid = np.linspace(global_x_min, global_x_max, int(curve_points))

                    forced_cols = set((row_group_by or []) + (col_group_by or [])) | {curve_x, hue}
                    X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
                    for j, fcol in enumerate(feature_columns):
                        if fcol == curve_x:
                            X_grid[:, j] = x_grid
                        elif fcol == hue:
                            X_grid[:, j] = hv_f
                        elif (row_group_by and fcol in row_group_by):
                            val = rkey[row_group_by.index(fcol)] if fcol in row_group_by else float(df_h[fcol].median())
                            X_grid[:, j] = float(val)
                        elif (col_group_by and fcol in col_group_by):
                            val = ckey[col_group_by.index(fcol)] if fcol in col_group_by else float(df_h[fcol].median())
                            X_grid[:, j] = float(val)
                        elif (fcol in fix_overrides) and (fcol not in forced_cols):
                            X_grid[:, j] = float(fix_overrides[fcol])
                        else:
                            med = df_h[fcol].median()
                            if not np.isfinite(med):
                                med = df_cell[fcol].median()
                            X_grid[:, j] = float(med)

                    if scaler is not None:
                        try:
                            X_infer = scaler.transform(X_grid)
                        except Exception:
                            X_infer = X_grid
                    else:
                        X_infer = X_grid

                    if show_uncertainty:
                        y_mean, y_std = model.predict(X_infer, return_std=True)
                        ax.plot(x_grid, y_mean, color=color, lw=2.5, alpha=0.95)
                        ax.fill_between(x_grid, y_mean - 2.0*y_std, y_mean + 2.0*y_std, color=color, alpha=0.18)
                    else:
                        y_mean = model.predict(X_infer, return_std=False)
                        ax.plot(x_grid, y_mean, color=color, lw=2.5, alpha=0.95)
            else:
                # 従来どおり 1色 raw + 1本 fit（±2σ）
                if curve_x not in df_cell.columns:
                    raise ValueError(f"x 軸列 '{curve_x}' がデータに存在しません。")
                x_raw = df_cell[curve_x].values
                y_raw = df_cell[target_column].values
                ax.scatter(x_raw, y_raw, alpha=raw_alpha, s=25, label='raw')

                # x_grid（補外禁止時はセルの実データ範囲内だけ）
                if fit_no_extrapolate:
                    x_series_cell = pd.to_numeric(df_cell[curve_x], errors="coerce")
                    xc_min = float(np.nanmin(x_series_cell.values))
                    xc_max = float(np.nanmax(x_series_cell.values))
                    if (not np.isfinite(xc_min)) or (not np.isfinite(xc_max)) or (xc_min == xc_max):
                        # rawのみ
                        ax.set_xlim(global_x_min, global_x_max)
                        ax.set_ylim(global_y_min, global_y_max)
                        ax.set_xlabel(curve_x)
                        ax.grid(True, alpha=0.3)
                        ax.legend(frameon=True, fontsize=10)
                        continue
                    x_grid = np.linspace(xc_min, xc_max, int(curve_points))
                else:
                    x_grid = np.linspace(global_x_min, global_x_max, int(curve_points))

                forced_cols = set((row_group_by or []) + (col_group_by or [])) | {curve_x}
                X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
                for j, fcol in enumerate(feature_columns):
                    if fcol == curve_x:
                        X_grid[:, j] = x_grid
                    elif (row_group_by and fcol in row_group_by):
                        val = rkey[row_group_by.index(fcol)] if fcol in row_group_by else float(df_cell[fcol].median())
                        X_grid[:, j] = float(val)
                    elif (col_group_by and fcol in col_group_by):
                        val = ckey[col_group_by.index(fcol)] if fcol in col_group_by else float(df_cell[fcol].median())
                        X_grid[:, j] = float(val)
                    elif (fcol in fix_overrides) and (fcol not in forced_cols):
                        X_grid[:, j] = float(fix_overrides[fcol])
                    else:
                        X_grid[:, j] = float(df_cell[fcol].median())

                if scaler is not None:
                    try:
                        X_infer = scaler.transform(X_grid)
                    except Exception:
                        X_infer = X_grid
                else:
                    X_infer = X_grid

                if show_uncertainty:
                    y_mean, y_std = model.predict(X_infer, return_std=True)
                    ax.plot(x_grid, y_mean, color='red', lw=3.0, label='GPR fit')
                    ax.fill_between(x_grid, y_mean - 2.0*y_std, y_mean + 2.0*y_std, color='red', alpha=0.2, label='±2σ')
                else:
                    y_mean = model.predict(X_infer, return_std=False)
                    ax.plot(x_grid, y_mean, color='red', lw=3.0, label='GPR fit')

            # 全ファセットで共通の軸スケールを設定
            ax.set_xlim(global_x_min, global_x_max)
            ax.set_ylim(global_y_min, global_y_max)
            ax.set_xlabel(curve_x)
            ax.grid(True, alpha=0.3)
            if not hue:
                ax.legend(frameon=True, fontsize=10)

    # hue 指定時は、facet 全体で1つの凡例にまとめる
    if hue and hue_values:
        handles = []
        for hv in hue_values:
            color = hue_to_color.get(hv, "C0")
            if isinstance(hv, (float, np.floating)):
                label = f"{hue}={hv:g}"
            else:
                label = f"{hue}={hv}"
            handles.append(mlines.Line2D([], [], color=color, marker='o', linestyle='-', lw=2.0, markersize=6, label=label))
        fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=min(6, max(1, len(handles))), frameon=True, fontsize=10)
        plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    else:
        plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"ファセット Raw vs GPR Fit を '{output_file}' に保存しました。")
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='ガウス過程回帰によるtorque_xのモデル化')
    parser.add_argument('csv_file', help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', default='gpr.png', help='出力画像ファイルのパス（デフォルト: gpr.png）')
    parser.add_argument('--test-size', type=float, default=0.2, help='テストデータの割合（デフォルト: 0.2）')
    parser.add_argument('--alpha', type=float, default=1e-6, help='ノイズの分散（デフォルト: 1e-6）')
    parser.add_argument('--n-restarts', type=int, default=2, help='最適化の再起動回数（デフォルト: 2）')
    parser.add_argument('--random-state', type=int, default=42, help='乱数のシード（デフォルト: 42）')
    parser.add_argument('--normalize', type=parse_tf, default=True, metavar='{t,f}', help='特徴量の正規化を行う（デフォルト: t）')
    parser.add_argument('--cv-folds', type=int, default=0, help='クロスバリデーションの分割数（0で無効）')
    parser.add_argument('--cv-jobs', type=int, default=1, help='クロスバリデーションの並列数（デフォルト: 1）')
    parser.add_argument('--max-train-samples', type=int, default=None, help='学習に使用する最大サンプル数（指定時はランダム抽出）')
    parser.add_argument('--no-uncertainty', action='store_true', help='予測の不確実性（標準偏差）を計算しない')
    parser.add_argument('--features', type=str, default=None, help='使用する特徴量をカンマ区切りで指定（未指定ならtorque_x以外の全列）')
    parser.add_argument('--list-features', action='store_true', help='利用可能な特徴量候補を一覧表示して終了')
    parser.add_argument('--target', type=str, default='torque_x', help='目的変数の列名（デフォルト: torque_x）')
    parser.add_argument('--anisotropic', type=parse_tf, default=True, metavar='{t,f}', help='各次元で別のlength_scaleを学習する（RBF/Matern）（デフォルト: t）')
    parser.add_argument('--length-scale-bounds', type=str, default=None, help='length_scaleの下限,上限（例: 1e-2,1e3）')
    parser.add_argument('--matern-nu', type=float, default=1.5, help='Maternカーネルのnu（0.5,1.5,2.5など）')
    parser.add_argument('--kernel', choices=['rbf', 'matern', 'rbf_white', 'rbf_linear'], default='rbf',
                       help='カーネルの種類（デフォルト: rbf）')
    parser.add_argument('--do-grid-search', action='store_true', help='alpha と length_scale_bounds を簡易グリッドで探索')
    parser.add_argument('--max-variance-torque-x', type=float, default=None, help='variance_torque_x の上限で高分散サンプルを除外')
    # ペアワイズヒートマップ関連
    parser.add_argument('--pairwise-heatmaps', type=parse_tf, default=True, metavar='{t,f}', help='全特徴量ペアの予測ヒートマップを一括出力（デフォルト: t）')
    parser.add_argument('--grid', type=int, default=80, help='ヒートマップの格子数（デフォルト: 80）')
    parser.add_argument('--fix', action='append', default=None, help="非可視化軸の固定値 'col=value' を複数指定可（--pairwise-heatmaps と --plot-raw-fit の両方で使用）")
    parser.add_argument('--range', dest='ranges', action='append', default=None, help="各軸の範囲 'col:min,max' を複数指定可")
    parser.add_argument('--std-heatmaps', type=parse_tf, default=True, metavar='{t,f}', help='標準偏差のヒートマップも保存（デフォルト: t）')
    parser.add_argument('--overlay-raw', type=parse_tf, default=True, metavar='{t,f}', help='ヒートマップ上に生データ点 (×) を重ねて表示（デフォルト: t）')
    parser.add_argument('--contour-lines', type=parse_tf, default=True, metavar='{t,f}', help='予測平均ヒートマップに等高線を重ねて描画（デフォルト: t）')
    parser.add_argument('--contour-levels', type=int, default=10, help='等高線レベル数（デフォルト: 10）')
    parser.add_argument('--contour-color', type=str, default='k', help='等高線の色（例: k, w, #RRGGBB）')
    parser.add_argument('--contour-linewidth', type=float, default=0.8, help='等高線の線幅（デフォルト: 0.8）')
    parser.add_argument('--contour-alpha', type=float, default=0.8, help='等高線の透明度（デフォルト: 0.8）')
    # ペアワイズヒートマップ用の事前フィルタ（非表示軸のレンジ指定用途）
    parser.add_argument('--heatmap-filter', action='append', default=None,
                        help="ペアワイズヒートマップ用の事前フィルタ 'col:min,max' を複数指定可（非表示軸のレンジ指定用途）")
    # 1ペアのファセットヒートマップ（任意軸でビン分割）
    parser.add_argument('--heatmap-facet', type=parse_tf, default=False, metavar='{t,f}',
                        help="1ペアのヒートマップを facet_by で区間分割してサブプロット連結する（デフォルト: f）")
    parser.add_argument('--heatmap-x', type=str, default=None, help="facetヒートマップのX軸（特徴量名）")
    parser.add_argument('--heatmap-y', type=str, default=None, help="facetヒートマップのY軸（特徴量名）")
    parser.add_argument('--heatmap-facet-by', type=str, default=None, help="区間分割に使う列名（例: distance）")
    parser.add_argument('--heatmap-facet-range', type=str, default=None, help="区間の範囲 'a,b'（例: 0.0,5.0）")
    parser.add_argument('--heatmap-facet-step', type=float, default=1.0, help="区間の幅（デフォルト: 1.0）")
    parser.add_argument('--heatmap-facet-by2', type=str, default=None, help="(2D) 区間分割に使う第2列名（行方向）")
    parser.add_argument('--heatmap-facet-range2', type=str, default=None, help="(2D) 第2範囲 'a,b'")
    parser.add_argument('--heatmap-facet-step2', type=float, default=None, help="(2D) 第2幅")
    parser.add_argument('--heatmap-facet-output', type=str, default=None, help="facetヒートマップの出力パス（未指定なら --output 派生名）")
    # 生データとフィット曲線の比較
    parser.add_argument('--plot-raw-fit', type=parse_tf, default=True, metavar='{t,f}', help='グループごとに生データ散布とGPRフィット曲線（±2σ帯）を比較表示（デフォルト: t）')
    parser.add_argument('--group-by', type=str, default=None, help='グループ化に用いる列をカンマ区切りで指定（例: distance,tilt_angle）')
    parser.add_argument('--curve-x', type=str, default=None, help='フィット曲線の横軸にする特徴量（未指定なら group_by 以外の最初の特徴量）')
    parser.add_argument('--curve-points', type=int, default=200, help='フィット曲線の分解能')
    parser.add_argument('--groupfit-output', type=str, default=None, help='グループ別 Raw vs Fit 図の出力パス（未指定なら表示、--output 指定時は派生名を使用）')
    parser.add_argument('--row-group-by', type=str, default=None, help='行方向のファセットに用いる列（カンマ区切りの複数可、指定順でソート）')
    parser.add_argument('--col-group-by', type=str, default=None, help='列方向のファセットに用いる列（カンマ区切りの複数可、指定順でソート）')
    parser.add_argument('--facet-filter', action='append', default=None, help="ファセット用の事前フィルタ 'col:min,max' を複数指定可")
    parser.add_argument(
        '--hue',
        type=str,
        default=None,
        help=(
            "同一セル/同一グループ内で色分け（グルーピング）する列名。"
            "row/col-group-by と同様に、hue値ごとに raw と fit を分けて描画します。"
            "hue は学習特徴量に含まれていなくても指定可能ですが、数値列のみ対応（文字列カテゴリは不可）です。"
        ),
    )
    parser.add_argument('--raw-alpha', type=float, default=0.5, help='group fit の raw 散布点の透明度（0〜1、デフォルト: 0.5）')
    parser.add_argument(
        '--fit-no-extrapolate',
        type=parse_tf,
        default=True,
        metavar='{t,f}',
        help="fit曲線で curve_x の補外をしない（データが存在する範囲内だけ描く、デフォルト: t）",
    )
    # facet 背景に morph drone を敷く
    parser.add_argument('--facet-drone', type=parse_tf, default=False, metavar='{t,f}',
                        help="ファセット各セルの背景に morph drone 図を薄く敷く（デフォルト: f）")
    parser.add_argument('--facet-drone-alpha', type=float, default=0.12, help='facet-drone の透明度（デフォルト: 0.12）')
    parser.add_argument('--facet-drone-force-2d', type=parse_tf, default=False, metavar='{t,f}',
                        help="(非推奨) facet-drone を 2D 投影で描画。現在はフォールバック禁止のため t はエラー（デフォルト: f）")
    parser.add_argument('--facet-drone-dpi', type=int, default=160, help='facet-drone のレンダリングDPI（デフォルト: 160）')
    parser.add_argument('--facet-drone-mode', choices=['background', 'inset', 'both'], default='background',
                        help="facet-drone の描画モード（デフォルト: background）")
    parser.add_argument('--facet-drone-inset-loc', choices=['ur', 'ul', 'lr', 'll'], default='ur',
                        help="facet-drone inset の位置（ur/ul/lr/ll、デフォルト: ur）")
    parser.add_argument('--facet-drone-inset-size', type=float, default=0.33,
                        help="facet-drone inset のサイズ（軸に対する比率、デフォルト: 0.33）")
    parser.add_argument('--facet-drone-inset-alpha', type=float, default=1.0,
                        help="facet-drone inset の透明度（デフォルト: 1.0）")
    # KRR 初期化関連
    parser.add_argument('--init-from-krr', action='store_true', help='KRRのグリッドサーチで得たハイパーパラメータをGPRの初期値に利用')
    parser.add_argument('--krr-cv', type=int, default=5, help='KRR GridSearchCV の分割数（デフォルト: 5）')
    parser.add_argument('--krr-jobs', type=int, default=1, help='KRR GridSearchCV の並列数（デフォルト: 1）')
    parser.add_argument('--krr-alpha-grid', type=str, default='1e-3,1e-2,1e-1,1.0,10.0', help='KRRのalpha候補（カンマ区切り）')
    parser.add_argument('--krr-gamma-grid', type=str, default='1e-3,1e-2,1e-1,1.0,10.0', help='KRRのgamma候補（カンマ区切り）')
    parser.add_argument('--krr-bound-factor', type=float, default=10.0, help='KRR初期値の±倍率でGPRの探索範囲を制限（デフォルト: 10.0）')
    # モデル保存/読み込み
    parser.add_argument('--save-model', type=str, default=None, help='学習済みモデルの保存先パス（.joblib 推奨）')
    parser.add_argument('--load-model', type=str, default=None, help='保存済みモデルの読み込みパス')
    # Backend切替（デフォルトは既存互換の sklearn）
    parser.add_argument('--backend', choices=['sklearn', 'gpytorch'], default='sklearn',
                        help="学習/推論バックエンド（デフォルト: sklearn）。gpytorch は誘導点SVGP(sparse)を使用。")
    parser.add_argument('--trust-model', type=parse_tf, default=False, metavar='{t,f}',
                        help="--load-model 時に torch.load の unsafe な復元を許可する（デフォルト: f）。"
                             "PyTorch 2.6+ の weights_only 制限回避用。信頼できるモデルのみ t にしてください。")
    # gpytorch学習用
    parser.add_argument('--num-inducing', type=int, default=512, help='GPyTorch(SVGP)の誘導点数（デフォルト: 512）')
    parser.add_argument('--epochs', type=int, default=300, help='GPyTorch(SVGP)の学習エポック数（デフォルト: 300）')
    parser.add_argument('--lr', type=float, default=0.01, help='GPyTorch(SVGP)の学習率（デフォルト: 0.01）')
    parser.add_argument('--batch-size', type=int, default=1024, help='GPyTorch(SVGP)のミニバッチサイズ（デフォルト: 1024）')
    parser.add_argument('--pred-batch-size', type=int, default=8192, help='GPyTorch推論時のバッチサイズ（デフォルト: 8192）')
    parser.add_argument('--device', choices=['auto', 'cpu', 'cuda'], default='auto', help='GPyTorchのデバイス（デフォルト: auto）')
    parser.add_argument('--log-every', type=int, default=50, help='GPyTorch学習ログ出力間隔（エポック、デフォルト: 50）')
    # デバッグ
    parser.add_argument('--debug-splits', action='store_true', help='前処理/分割/サブサンプル各段階の分布ログを出力')
    parser.add_argument('--debug-columns', type=str, default=None, help="分布を見る列（カンマ区切り、未指定なら fold_angle があればそれ）")
    parser.add_argument('--debug-topk', type=int, default=20, help='value_counts の上位表示数（デフォルト: 20）')
    
    args = parser.parse_args()

    # raw scatter alpha validation (group fit)
    if (args.raw_alpha is None) or (not np.isfinite(args.raw_alpha)) or (args.raw_alpha < 0.0) or (args.raw_alpha > 1.0):
        raise ValueError(f"--raw-alpha は 0〜1 の範囲で指定してください: {args.raw_alpha}")
    
    try:
        # 特徴量一覧の表示のみ
        if args.list_features:
            print("=== 特徴量一覧 ===")
            if not os.path.exists(args.csv_file):
                raise FileNotFoundError(f"ファイル '{args.csv_file}' が見つかりません。")
            df_head = pd.read_csv(args.csv_file, nrows=5)
            candidates = [c for c in df_head.columns if c != args.target]
            print(f"利用可能な特徴量候補: {candidates}")
            print(f"選択中の目的変数: {args.target}")
            return 0

        # 描画バックエンドの自動切替（ヘッドレスや--output指定時）
        if (not os.environ.get('DISPLAY')) or args.output:
            try:
                matplotlib.use('Agg', force=True)
            except Exception:
                pass

        # 事前にモデルを読み込み（指定時）
        loaded_model = None
        loaded_scaler = None
        loaded_feature_columns = None
        loaded_target_column = None
        loaded_backend = None
        loaded_bundle = None
        if args.load_model:
            print(f"=== 保存済みモデルの読み込み: {args.load_model} ===")
            try:
                loaded_backend, loaded_obj, loaded_scaler, loaded_feature_columns, loaded_target_column = load_model_any(
                    args.load_model,
                    trust=bool(getattr(args, "trust_model", False)),
                )
                if loaded_backend == "sklearn":
                    loaded_model = loaded_obj
                else:
                    loaded_bundle = loaded_obj
                if loaded_feature_columns is not None:
                    print(f"保存モデルの特徴量: {loaded_feature_columns}")
                if loaded_target_column is not None:
                    print(f"保存モデルの目的変数: {loaded_target_column}")
                if loaded_backend is not None:
                    print(f"保存モデルの backend: {loaded_backend}")
            except Exception as e:
                raise RuntimeError(f"モデルの読み込みに失敗しました: {e}")

        # データの読み込みと前処理
        print("=== データの読み込みと前処理 ===")
        selected_features = None
        if args.features:
            selected_features = [c.strip() for c in args.features.split(',') if c.strip()]
        # 保存モデルの目的変数があれば優先（後方互換）
        if args.load_model and loaded_target_column:
            if args.target != loaded_target_column:
                print(f"注意: --load-model の保存時 target='{loaded_target_column}' を優先し、--target='{args.target}' を上書きします。")
            args.target = loaded_target_column
        # 読み込みモデルに特徴量が含まれている場合は最優先で使用
        if args.load_model and loaded_feature_columns is not None:
            if args.features:
                print("注意: --load-model で保存時の特徴量を使用するため、--features を上書きします。")
            selected_features = list(loaded_feature_columns)
        df_clean, feature_columns = load_and_preprocess_data(
            args.csv_file,
            selected_features,
            max_variance_torque_x=args.max_variance_torque_x,
            target_column=args.target,
            debug=args.debug_splits
        )

        # デバッグ対象列の決定
        debug_cols = []
        if args.debug_columns:
            debug_cols = [c.strip() for c in args.debug_columns.split(',') if c.strip()]
        else:
            if 'fold_angle' in df_clean.columns:
                debug_cols = ['fold_angle']

        if args.debug_splits and debug_cols:
            _debug_print_distribution(df_clean, debug_cols, stage_name="df_clean (after preprocess)", topk=args.debug_topk)
        
        # 特徴量と目的変数の準備
        X = df_clean[feature_columns].values
        y = df_clean[args.target].values
        
        print(f"特徴量の形状: {X.shape}")
        print(f"目的変数の形状: {y.shape}")
        
        # 特徴量の正規化
        scaler = None
        if args.load_model:
            scaler = loaded_scaler
            if scaler is not None:
                print("保存済みスケーラを用いて特徴量を変換します。")
                X = scaler.transform(X)
        else:
            # オプション/ KRR初期化時は強制
            force_normalize = args.normalize or args.init_from_krr
            if force_normalize:
                if args.init_from_krr and not args.normalize:
                    print("KRR初期化のため特徴量を標準化します（StandardScaler）。")
                else:
                    print("特徴量の正規化を実行...")
                scaler = StandardScaler()
                X = scaler.fit_transform(X)
        
        # 訓練データとテストデータの分割（デバッグ用にインデックスも保持）
        all_indices = np.arange(len(df_clean))
        train_idx, test_idx = train_test_split(
            all_indices, test_size=args.test_size, random_state=args.random_state
        )
        X_train, X_test = X[train_idx], X[test_idx]
        y_train, y_test = y[train_idx], y[test_idx]

        if args.debug_splits and debug_cols:
            _debug_print_distribution(df_clean.iloc[train_idx], debug_cols, stage_name="train (before subsampling)", topk=args.debug_topk)
            _debug_print_distribution(df_clean.iloc[test_idx], debug_cols, stage_name="test", topk=args.debug_topk)
        
        print(f"訓練データ数: {len(X_train)}")
        print(f"テストデータ数: {len(X_test)}")

        # 学習データのサブサンプリング（必要に応じて）
        if args.max_train_samples is not None and len(X_train) > args.max_train_samples:
            rs = np.random.RandomState(args.random_state)
            indices = rs.choice(len(X_train), size=args.max_train_samples, replace=False)
            if args.debug_splits and debug_cols:
                # train_idx に対応する df を 0.. に振り直して indices で参照できるようにする
                df_train_full = df_clean.iloc[train_idx].reset_index(drop=True)
                _debug_print_distribution(df_train_full.iloc[indices], debug_cols, stage_name="train (after subsampling)", topk=args.debug_topk)
            X_train = X_train[indices]
            y_train = y_train[indices]
            print(f"学習データをサブサンプリング: {len(indices)} サンプルを使用")
        
        # ガウス過程回帰モデルの作成/読み込みと訓練
        # length_scale boundsの解釈（sklearn/gpytorch共通で解釈だけする）
        ls_bounds = None
        if args.length_scale_bounds:
            try:
                lo, hi = [float(x) for x in args.length_scale_bounds.split(',')]
                ls_bounds = (lo, hi)
            except Exception:
                raise ValueError("--length-scale-bounds は '低,高' の形式で指定してください（例: 1e-2,1e3）")

        # load_model の場合 backend は保存モデルを優先
        if args.load_model and loaded_backend:
            if args.backend != loaded_backend:
                print(f"注意: 保存モデルの backend='{loaded_backend}' を優先し、--backend='{args.backend}' を上書きします。")
            args.backend = loaded_backend

        if args.load_model:
            if args.backend == "sklearn":
                gp_model = loaded_model
                if gp_model is None:
                    raise RuntimeError("--load-model が指定されましたが、モデルを取得できませんでした。")
                print(f"保存済みsklearnモデルを読み込みました。カーネル: {getattr(gp_model, 'kernel_', getattr(gp_model, 'kernel', None))}")
            else:
                if loaded_bundle is None:
                    raise RuntimeError("--load-model が指定されましたが、gpytorch bundle を取得できませんでした。")
                gp_model = build_gpytorch_wrapper_from_bundle(loaded_bundle, device_str=args.device)
                print("保存済みgpytorchモデルを読み込みました。")
        else:
            print(f"\n=== ガウス過程回帰モデルの訓練 ===")
            print(f"backend: {args.backend}")
            print(f"カーネル: {args.kernel}")

            if args.backend == "gpytorch":
                if args.init_from_krr:
                    print("注意: --backend gpytorch の場合、--init-from-krr は未対応のため無視します。")
                if args.do_grid_search:
                    print("注意: --backend gpytorch の場合、--do-grid-search は未対応のため無視します。")
                if args.n_restarts and args.n_restarts != 2:
                    print("注意: --backend gpytorch の場合、--n-restarts は未使用です。")
                gp_model = train_sparse_gpytorch(X_train, y_train, args=args, length_scale_bounds=ls_bounds)
            else:
                print(f"アルファ: {args.alpha}")
                print(f"最適化再起動回数: {args.n_restarts}")
                if args.init_from_krr:
                    if args.do_grid_search:
                        print("--init-from-krr が指定されたため、既存のGPR用グリッドサーチはスキップします。")

                    # 文字列グリッドを数値リストに変換
                    def _parse_float_grid(s):
                        vals = []
                        for t in s.split(','):
                            t = t.strip()
                            if not t:
                                continue
                            try:
                                vals.append(float(t))
                            except Exception:
                                pass
                        return vals

                    alpha_grid = _parse_float_grid(args.krr_alpha_grid)
                    gamma_grid = _parse_float_grid(args.krr_gamma_grid)
                    if not alpha_grid:
                        alpha_grid = [1e-3, 1e-2, 1e-1, 1.0, 10.0]
                    if not gamma_grid:
                        gamma_grid = [1e-3, 1e-2, 1e-1, 1.0, 10.0]

                    if args.kernel not in ['rbf', 'rbf_white', 'rbf_linear']:
                        print("警告: --kernel が RBFベース以外です。KRR初期化にはRBFを使用します。")

                    print("KRRのグリッドサーチを実行します...")
                    param_grid = {
                        'alpha': alpha_grid,
                        'gamma': gamma_grid
                    }
                    krr = KernelRidge(kernel='rbf')
                    gs = GridSearchCV(
                        krr,
                        param_grid=param_grid,
                        cv=max(2, args.krr_cv),
                        n_jobs=args.krr_jobs,
                        scoring='neg_mean_squared_error'
                    )
                    gs.fit(X_train, y_train.ravel())
                    best_alpha = float(gs.best_params_['alpha'])
                    best_gamma = float(gs.best_params_['gamma'])
                    print(f"KRR最良パラメータ: alpha={best_alpha}, gamma={best_gamma}")
                    try:
                        best_cv_rmse = float(np.sqrt(-gs.best_score_))
                        print(f"KRR CV 最良RMSE: {best_cv_rmse:.6f} (scoring=neg_mean_squared_error)")
                    except Exception:
                        pass

                    # KRR 最良モデルでテストデータに対する性能を表示
                    try:
                        krr_best = KernelRidge(kernel='rbf', alpha=best_alpha, gamma=best_gamma)
                        krr_best.fit(X_train, y_train.ravel())
                        y_pred_krr = krr_best.predict(X_test)
                        krr_mse = mean_squared_error(y_test, y_pred_krr)
                        krr_rmse = float(np.sqrt(krr_mse))
                        krr_mae = mean_absolute_error(y_test, y_pred_krr)
                        krr_r2 = r2_score(y_test, y_pred_krr)
                        print("KRR テスト評価:")
                        print(f"  RMSE: {krr_rmse:.6f}")
                        print(f"  MAE:  {krr_mae:.6f}")
                        print(f"  R²:   {krr_r2:.6f}")

                        # KRR結果のプロット（テストデータ）
                        if args.output:
                            base_name = os.path.splitext(args.output)[0]
                            krr_results_output = f"{base_name}_krr_results.png"
                        else:
                            krr_results_output = None
                        print("KRRの予測結果をプロットします...")
                        plot_results(y_test, y_pred_krr, None, feature_columns, krr_results_output)
                    except Exception:
                        pass

                    # KRR → GPR 初期値変換
                    initial_constant = float(np.var(y_train))
                    if best_gamma <= 0:
                        initial_length_scale = 1.0
                    else:
                        initial_length_scale = float(np.sqrt(1.0 / (2.0 * best_gamma)))
                    initial_noise = max(best_alpha, 1e-12)

                    # bounds（KRR初期値を中心に制限）
                    if initial_constant <= 1e-12:
                        initial_constant = 1.0

                    # 既定の広いbounds
                    base_const_bounds = (1e-8, 1e5)
                    base_ls_bounds = ls_bounds if ls_bounds is not None else (1e-5, 1e5)
                    base_noise_bounds = (1e-12, 1e2)

                    f = max(1.0, float(args.krr_bound_factor) if args.krr_bound_factor is not None else 10.0)

                    def around(val, factor, base):
                        lo = max(val / factor, base[0])
                        hi = min(val * factor, base[1])
                        if lo >= hi:  # フォールバックで僅かに広げる
                            mid = max(val, 1e-12)
                            lo = max(base[0], mid / (factor * 2.0))
                            hi = min(base[1], mid * (factor * 2.0))
                        return (lo, hi)

                    const_bounds = around(initial_constant, f, base_const_bounds)
                    ls_bounds_final = around(initial_length_scale, f, base_ls_bounds)
                    noise_bounds = around(initial_noise, f, base_noise_bounds)

                    if args.anisotropic and X_train.shape[1] > 1:
                        ls0 = [initial_length_scale] * X_train.shape[1]
                        rbf = RBF(length_scale=ls0, length_scale_bounds=ls_bounds_final)
                    else:
                        rbf = RBF(length_scale=initial_length_scale, length_scale_bounds=ls_bounds_final)

                    kernel = ConstantKernel(constant_value=initial_constant, constant_value_bounds=const_bounds) * rbf \
                             + WhiteKernel(noise_level=initial_noise, noise_level_bounds=noise_bounds)

                    print("KRR初期化から生成したGPR初期カーネル:")
                    print(kernel)
                    print("適用された探索範囲:")
                    print(f"  ConstantKernel bounds: {const_bounds}")
                    print(f"  RBF length_scale bounds: {ls_bounds_final}")
                    print(f"  WhiteKernel noise_level bounds: {noise_bounds}")

                    gp_model = GaussianProcessRegressor(
                        kernel=kernel,
                        alpha=0.0,
                        n_restarts_optimizer=args.n_restarts,
                        random_state=42,
                        copy_X_train=False,
                        normalize_y=True
                    )
                elif args.do_grid_search:
                    print("グリッドサーチを実行します...")
                    gp_model = grid_search_gpr(
                        X_train,
                        y_train,
                        base_args=args,
                        n_features=X_train.shape[1],
                        ls_bounds_initial=ls_bounds
                    )
                else:
                    gp_model = create_gp_model(
                        kernel_type=args.kernel,
                        alpha=args.alpha,
                        n_restarts_optimizer=args.n_restarts,
                        n_features=X_train.shape[1],
                        anisotropic=args.anisotropic,
                        length_scale_bounds=ls_bounds,
                        matern_nu=args.matern_nu
                    )

                # モデルの訓練
                print("モデルを訓練中...")
                gp_model.fit(X_train, y_train)
                # 最適化されたカーネルパラメータの表示
                print(f"最適化されたカーネル: {gp_model.kernel_}")
        
        # モデルの評価
        print(f"\n=== モデルの評価 ===")
        metrics = evaluate_model(
            gp_model,
            X_test,
            y_test,
            X_train if args.cv_folds > 0 else None,
            y_train if args.cv_folds > 0 else None,
            cv_folds=args.cv_folds,
            cv_jobs=args.cv_jobs,
            return_std=(not args.no_uncertainty)
        )
        
        print(f"テストデータでの評価:")
        print(f"  RMSE: {metrics['rmse']:.6f}")
        print(f"  MAE:  {metrics['mae']:.6f}")
        print(f"  R²:   {metrics['r2']:.6f}")
        
        if 'cv_rmse' in metrics:
            print(f"クロスバリデーション:")
            print(f"  CV RMSE: {metrics['cv_rmse']:.6f} ± {metrics['cv_std']:.6f}")
        
        # 結果の可視化
        print(f"\n=== 結果の可視化 ===")
        if args.output:
            base_name = os.path.splitext(args.output)[0]
            results_output = f"{base_name}_results.png"
            importance_output = f"{base_name}_importance.png"
        else:
            results_output = None
            importance_output = None
        
        # 動的に目的変数名を渡す（関数属性を利用）
        plot_results._target_column = args.target
        plot_results(y_test, metrics['y_pred'], metrics.get('y_std', None), 
                    feature_columns, results_output)
        plot_feature_importance(gp_model, feature_columns, importance_output)

        # ペアワイズヒートマップ
        if args.pairwise_heatmaps:
            print("\n=== ペアワイズヒートマップの生成 ===")
            output_prefix = None
            if args.output:
                output_prefix = os.path.splitext(args.output)[0]
            generate_pairwise_heatmaps._target_column = args.target
            generate_pairwise_heatmaps(
                gp_model,
                feature_columns,
                df_clean,
                scaler,
                grid_size=args.grid,
                fixes=args.fix,
                ranges=args.ranges,
                output_prefix=output_prefix,
                include_std=args.std_heatmaps,
                overlay_raw=args.overlay_raw,
                contour_lines=args.contour_lines,
                contour_levels=args.contour_levels,
                contour_color=args.contour_color,
                contour_linewidth=args.contour_linewidth,
                contour_alpha=args.contour_alpha,
                heatmap_filters=args.heatmap_filter,
            )

        # 1ペアのファセットヒートマップ（横連結）
        if bool(getattr(args, "heatmap_facet", False)):
            if not args.heatmap_x or not args.heatmap_y or not args.heatmap_facet_by or not args.heatmap_facet_range:
                raise ValueError("--heatmap-facet を使うには --heatmap-x/--heatmap-y/--heatmap-facet-by/--heatmap-facet-range が必要です。")
            hm_out = args.heatmap_facet_output
            if not hm_out and args.output:
                base_name = os.path.splitext(args.output)[0]
                if args.heatmap_facet_by2:
                    hm_out = f"{base_name}_facet_heatmap_{args.heatmap_x}__{args.heatmap_y}_by_{args.heatmap_facet_by}__{args.heatmap_facet_by2}.png"
                else:
                    hm_out = f"{base_name}_facet_heatmap_{args.heatmap_x}__{args.heatmap_y}_by_{args.heatmap_facet_by}.png"
            generate_facet_heatmap_1pair._target_column = args.target
            generate_facet_heatmap_1pair(
                gp_model,
                feature_columns,
                df_clean,
                scaler,
                heatmap_x=str(args.heatmap_x).strip(),
                heatmap_y=str(args.heatmap_y).strip(),
                facet_by=str(args.heatmap_facet_by).strip(),
                facet_range=str(args.heatmap_facet_range).strip(),
                facet_step=float(args.heatmap_facet_step),
                facet_by2=(str(args.heatmap_facet_by2).strip() if args.heatmap_facet_by2 else None),
                facet_range2=(str(args.heatmap_facet_range2).strip() if args.heatmap_facet_range2 else None),
                facet_step2=(float(args.heatmap_facet_step2) if args.heatmap_facet_step2 is not None else None),
                grid_size=int(args.grid),
                fixes=args.fix,
                ranges=args.ranges,
                output_file=hm_out,
                overlay_raw=bool(args.overlay_raw),
                contour_lines=bool(args.contour_lines),
                contour_levels=int(args.contour_levels),
                contour_color=str(args.contour_color),
                contour_linewidth=float(args.contour_linewidth),
                contour_alpha=float(args.contour_alpha),
                heatmap_filters=args.heatmap_filter,
                std_heatmap=False,
            )
            # std版（任意）：std-heatmaps が t の時だけ
            if bool(getattr(args, "std_heatmaps", False)):
                if hm_out:
                    base_name = os.path.splitext(hm_out)[0]
                    hm_out_std = f"{base_name}_std.png"
                else:
                    hm_out_std = None
                generate_facet_heatmap_1pair._target_column = args.target
                generate_facet_heatmap_1pair(
                    gp_model,
                    feature_columns,
                    df_clean,
                    scaler,
                    heatmap_x=str(args.heatmap_x).strip(),
                    heatmap_y=str(args.heatmap_y).strip(),
                    facet_by=str(args.heatmap_facet_by).strip(),
                    facet_range=str(args.heatmap_facet_range).strip(),
                    facet_step=float(args.heatmap_facet_step),
                    facet_by2=(str(args.heatmap_facet_by2).strip() if args.heatmap_facet_by2 else None),
                    facet_range2=(str(args.heatmap_facet_range2).strip() if args.heatmap_facet_range2 else None),
                    facet_step2=(float(args.heatmap_facet_step2) if args.heatmap_facet_step2 is not None else None),
                    grid_size=int(args.grid),
                    fixes=args.fix,
                    ranges=args.ranges,
                    output_file=hm_out_std,
                    overlay_raw=bool(args.overlay_raw),
                    contour_lines=False,
                    heatmap_filters=args.heatmap_filter,
                    std_heatmap=True,
                )
        
        # グループ別 生データ散布 + GPRフィット曲線（±2σ）
        if args.plot_raw_fit:
            print("\n=== グループ別 Raw vs GPR Fit の描画 ===")
            row_group_by = [c.strip() for c in args.row_group_by.split(',')] if args.row_group_by else None
            col_group_by = [c.strip() for c in args.col_group_by.split(',')] if args.col_group_by else None
            group_by = [c.strip() for c in args.group_by.split(',')] if (args.group_by and not row_group_by and not col_group_by) else None

            # 既存の --range 指定を流用
            mins, maxs = parse_ranges(args.ranges, feature_columns, df_clean)

            groupfit_output = args.groupfit_output
            if not groupfit_output and args.output:
                base_name = os.path.splitext(args.output)[0]
                groupfit_output = f"{base_name}_groupfit.png"

            try:
                if row_group_by or col_group_by:
                    # ファセット前フィルタ適用（任意）
                    df_for_facet = apply_facet_filters(df_clean, args.facet_filter)
                    plot_facet_raw_and_fit_gpr._target_column = args.target
                    plot_facet_raw_and_fit_gpr(
                        df_clean=df_for_facet,
                        feature_columns=feature_columns,
                        model=gp_model,
                        scaler=scaler,
                        row_group_by=row_group_by,
                        col_group_by=col_group_by,
                        curve_x=args.curve_x,
                        curve_points=args.curve_points,
                        ranges=(mins, maxs),
                        output_file=groupfit_output,
                        show_uncertainty=(not args.no_uncertainty),
                        hue=args.hue,
                        fixes=args.fix,
                        facet_drone=bool(getattr(args, "facet_drone", False)),
                        facet_drone_alpha=float(getattr(args, "facet_drone_alpha", 0.12)),
                        facet_drone_force_2d=bool(getattr(args, "facet_drone_force_2d", True)),
                        facet_drone_dpi=int(getattr(args, "facet_drone_dpi", 160)),
                        facet_drone_mode=str(getattr(args, "facet_drone_mode", "background")),
                        facet_drone_inset_loc=str(getattr(args, "facet_drone_inset_loc", "ur")),
                        facet_drone_inset_size=float(getattr(args, "facet_drone_inset_size", 0.33)),
                        facet_drone_inset_alpha=float(getattr(args, "facet_drone_inset_alpha", 1.0)),
                        raw_alpha=float(args.raw_alpha),
                        fit_no_extrapolate=bool(getattr(args, "fit_no_extrapolate", True)),
                    )
                elif group_by:
                    plot_grouped_raw_and_fit_gpr._target_column = args.target
                    plot_grouped_raw_and_fit_gpr(
                        df_clean=df_clean,
                        feature_columns=feature_columns,
                        model=gp_model,
                        scaler=scaler,
                        group_by=group_by,
                        curve_x=args.curve_x,
                        curve_points=args.curve_points,
                        ranges=(mins, maxs),
                        output_file=groupfit_output,
                        show_uncertainty=(not args.no_uncertainty),
                        hue=args.hue,
                        fixes=args.fix,
                        raw_alpha=float(args.raw_alpha),
                        fit_no_extrapolate=bool(getattr(args, "fit_no_extrapolate", True)),
                    )
                else:
                    print("(注意) --plot-raw-fit は指定されましたが、--group-by も --row-group-by/--col-group-by も未指定です。スキップします。")
            except Exception as e:
                print(f"グループ別 Raw vs GPR Fit の描画でエラー: {e}")
        
        # 統計情報の表示
        print(f"\n=== 統計情報 ===")
        print(f"使用した特徴量: {feature_columns}")
        print(f"データポイント数: {len(df_clean)}")
        print(f"{args.target}の範囲: {y.min():.6f} - {y.max():.6f}")
        print(f"{args.target}の平均: {y.mean():.6f}")
        print(f"{args.target}の標準偏差: {y.std():.6f}")
        
        # 予測の不確実性の統計
        if metrics.get('y_std', None) is not None:
            print(f"予測の不確実性:")
            print(f"  平均標準偏差: {metrics['y_std'].mean():.6f}")
            print(f"  最大標準偏差: {metrics['y_std'].max():.6f}")
            print(f"  最小標準偏差: {metrics['y_std'].min():.6f}")
        
        # モデルの保存
        if args.save_model:
            try:
                if args.backend == "gpytorch" and isinstance(gp_model, GPyTorchSparseGPR):
                    save_gpytorch_model(args.save_model, gp_model, feature_columns, scaler, args)
                    print(f"モデルを保存しました（gpytorch/torch.save）: {args.save_model}")
                else:
                    meta = {
                        'script': 'gaussian_process_regression.py',
                        'kernel_repr': str(getattr(gp_model, 'kernel_', getattr(gp_model, 'kernel', None))),
                        'normalize': scaler is not None,
                        'sklearn_version': getattr(sklearn, '__version__', 'unknown')
                    }
                    bundle_to_save = {
                        'model': gp_model,
                        'scaler': scaler,
                        'feature_columns': feature_columns,
                        'meta': meta,
                        'target_column': args.target
                    }
                    dump(bundle_to_save, args.save_model)
                    print(f"モデルを保存しました（joblib）: {args.save_model}")
            except Exception as e:
                print(f"モデルの保存に失敗しました: {e}")
        
        return 0
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        return 1

if __name__ == "__main__":
    exit(main())
