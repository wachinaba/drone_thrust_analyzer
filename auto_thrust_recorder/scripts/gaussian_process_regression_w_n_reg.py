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
import argparse
import os
from pathlib import Path
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
warnings.filterwarnings('ignore')

def load_and_preprocess_data(csv_file, selected_features=None, max_variance_torque_x=None, target_column='torque_x'):
    """
    CSVファイルを読み込み、データの前処理を行う
    
    Args:
        csv_file: CSVファイルのパス
        selected_features: 使用する特徴量のリスト（Noneの場合はtorque_x以外の全列）
        max_variance_torque_x: 'variance_torque_x' による上限フィルタ（Noneで無効）
        target_column: 目的変数の列名
    
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
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # 高分散サンプル除外（指定があり、列が存在する場合）
    if max_variance_torque_x is not None and 'variance_torque_x' in df.columns:
        before = len(df)
        df = df[df['variance_torque_x'] <= max_variance_torque_x]
        after = len(df)
        print(f"variance_torque_x フィルタ: {before-after} 行を除外（閾値 {max_variance_torque_x}）")
    
    # NaNを含む行を削除
    df_clean = df.dropna(subset=[target_column] + feature_columns)
    
    if df_clean.empty:
        raise ValueError("有効なデータがありません。")
    
    print(f"前処理後のデータ行数: {len(df_clean)}")
    print(f"使用する特徴量: {feature_columns}")
    
    return df_clean, feature_columns

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
    if cv_folds and cv_folds > 0 and X_train is not None and y_train is not None:
        cv_scores = cross_val_score(
            model, X_train, y_train,
            cv=cv_folds,
            scoring='neg_mean_squared_error',
            n_jobs=cv_jobs
        )
        metrics['cv_rmse'] = np.sqrt(-cv_scores.mean())
        metrics['cv_std'] = np.sqrt(cv_scores.std())
    
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
    if hasattr(model.kernel_, 'k1') and hasattr(model.kernel_.k1, 'length_scale'):
        # RBFカーネルの場合
        length_scales = model.kernel_.k1.length_scale
        if np.isscalar(length_scales):
            length_scales = [length_scales] * len(feature_columns)
        
        # 長さスケールの逆数が重要度の指標
        importance = 1.0 / np.array(length_scales)
    else:
        # その他のカーネルの場合は均等に設定
        importance = np.ones(len(feature_columns))
    
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

def generate_pairwise_heatmaps(model, feature_columns, df_clean, scaler, grid_size, fixes, ranges, output_prefix, include_std, overlay_raw=False, contour_lines=False, contour_levels=10, contour_color='k', contour_linewidth=0.8, contour_alpha=0.8):
    # 目的変数名は呼び出し元から渡される想定（関数属性に設定される）
    target_column = getattr(generate_pairwise_heatmaps, "_target_column", "torque_x")
    fixed_values = parse_fixed_values(fixes, feature_columns, df_clean)
    mins, maxs = parse_ranges(ranges, feature_columns, df_clean)

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
                xi_raw = df_clean[fi].values
                xj_raw = df_clean[fj].values
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
                    xi_raw = df_clean[fi].values
                    xj_raw = df_clean[fj].values
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


def generate_n_eff_heatmap(
    model,
    feature_columns,
    df_clean,
    scaler,
    distance_col: str,
    logforce_col: str,
    grid_size: int,
    fixes,
    ranges,
    output_path: str | None,
    delta_logF: float = 0.02,
):
    """
    GPR で学習した log|target| を用いて、局所的な有効指数
        n_eff(d, F) = d log|T| / d log F
    を、distance × log_force_z グリッド上で数値微分により推定し、
    ヒートマップとして保存/表示する。
    """
    if distance_col not in feature_columns:
        raise ValueError(f"n_eff 用 distance 列 '{distance_col}' が特徴量に含まれていません。--features を確認してください。")
    if logforce_col not in feature_columns:
        raise ValueError(
            f"n_eff 用 log(force) 列 '{logforce_col}' が特徴量に含まれていません。"
            " 例: --add-log-force-z force_z を指定し、--features に log_force_z を含めてください。"
        )

    # 固定値とレンジの取得（既存ヘルパを流用）
    fixed_values = parse_fixed_values(fixes, feature_columns, df_clean)
    mins, maxs = parse_ranges(ranges, feature_columns, df_clean)

    d_min = mins[distance_col]
    d_max = maxs[distance_col]
    lf_min = mins[logforce_col]
    lf_max = maxs[logforce_col]

    if not np.isfinite(d_min) or not np.isfinite(d_max):
        d_min = float(df_clean[distance_col].min())
        d_max = float(df_clean[distance_col].max())
    if not np.isfinite(lf_min) or not np.isfinite(lf_max):
        lf_min = float(df_clean[logforce_col].min())
        lf_max = float(df_clean[logforce_col].max())

    if d_min == d_max or lf_min == lf_max:
        raise ValueError("n_eff ヒートマップ用の distance / log_force 範囲がゼロ幅です。range 指定などを確認してください。")

    d_grid = np.linspace(d_min, d_max, grid_size)
    lf_grid = np.linspace(lf_min, lf_max, grid_size)
    D, LF = np.meshgrid(d_grid, lf_grid)

    # グリッド点数
    n_pts = grid_size * grid_size

    def _build_X(logF_offset: float) -> np.ndarray:
        Xg = np.zeros((n_pts, len(feature_columns)), dtype=float)
        for j, fcol in enumerate(feature_columns):
            if fcol == distance_col:
                Xg[:, j] = D.ravel()
            elif fcol == logforce_col:
                Xg[:, j] = LF.ravel() + logF_offset
            else:
                Xg[:, j] = fixed_values[fcol]
        if scaler is not None:
            try:
                return scaler.transform(Xg)
            except Exception:
                return Xg
        return Xg

    X_plus = _build_X(delta_logF)
    X_minus = _build_X(-delta_logF)

    # log|T| の予測値
    y_plus = model.predict(X_plus, return_std=False)
    y_minus = model.predict(X_minus, return_std=False)

    y_plus = y_plus.reshape(grid_size, grid_size)
    y_minus = y_minus.reshape(grid_size, grid_size)

    # 中心差分による d log|T| / d log F
    n_eff = (y_plus - y_minus) / (2.0 * float(delta_logF))

    # ヒートマップ描画
    plt.figure(figsize=(8, 6))
    plt.rcParams.update({'font.size': 16})
    zmin = float(np.nanpercentile(n_eff, 5))
    zmax = float(np.nanpercentile(n_eff, 95))
    if not np.isfinite(zmin) or not np.isfinite(zmax) or zmin == zmax:
        zmin = float(np.nanmin(n_eff))
        zmax = float(np.nanmax(n_eff))
        if not np.isfinite(zmin) or not np.isfinite(zmax) or zmin == zmax:
            zmin, zmax = -1.0, 1.0

    im = plt.imshow(
        n_eff,
        origin='lower',
        aspect='auto',
        extent=[d_min, d_max, lf_min, lf_max],
        cmap='coolwarm',
        vmin=zmin,
        vmax=zmax,
    )
    cbar = plt.colorbar(im, label='n_eff = d log|T| / d log F')
    cbar.ax.tick_params(labelsize=12)

    plt.xlabel(distance_col)
    plt.ylabel(logforce_col)
    plt.title("局所的な有効指数 n_eff(distance, log_force)")
    plt.tight_layout()

    if output_path:
        out_path = Path(output_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(str(out_path), dpi=200, bbox_inches='tight')
        print(f"n_eff ヒートマップを保存: {out_path}")
        plt.close()
    else:
        plt.show()
        plt.close()

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
    show_uncertainty=True
):
    """
    指定した group_by 特徴量でデータをグループ化し、各グループで
    - 生の散布 (curve_x vs torque_x)
    - GPR のフィット曲線 (curve_x を掃引、他特徴量はグループの中央値固定)
    - show_uncertainty が True の場合は ±2σ の不確実性帯を表示
    を同一サブプロット上に描画する。
    """
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

        # 生データの散布
        if curve_x not in df_group.columns:
            raise ValueError(f"x 軸列 '{curve_x}' がデータに存在しません。")
        x_raw = df_group[curve_x].values
        y_raw = df_group[target_column].values
        ax.scatter(x_raw, y_raw, alpha=0.5, s=25, label='raw')

        # 予測用グリッド（curve_x を掃引、他は中央値固定（group 内））
        x_min = mins.get(curve_x, float(df_group[curve_x].min()))
        x_max = maxs.get(curve_x, float(df_group[curve_x].max()))
        if not np.isfinite(x_min) or not np.isfinite(x_max):
            x_min = float(df_group[curve_x].min())
            x_max = float(df_group[curve_x].max())
        if x_min == x_max:
            x_min -= 1e-6
            x_max += 1e-6
        x_grid = np.linspace(x_min, x_max, int(curve_points))

        X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
        for j, fcol in enumerate(feature_columns):
            if fcol == curve_x:
                X_grid[:, j] = x_grid
            elif fcol in group_by:
                X_grid[:, j] = float(df_group[fcol].median())
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
        ax.legend(frameon=True, fontsize=10)

    # 余白のサブプロットを非表示
    for k in range(n_groups, n_rows * n_cols):
        r = k // n_cols
        c = k % n_cols
        axes[r, c].set_axis_off()

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
    show_uncertainty=True
):
    """
    行方向(row_group_byの組) × 列方向(col_group_byの組)のファセットで、
    各セルに Raw 散布と GPR フィット（±2σ）の曲線を描画する。
    指定順でソートして並べる。
    """
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

    # レンジ
    if ranges is not None and isinstance(ranges, tuple) and len(ranges) == 2:
        mins, maxs = ranges
    else:
        mins = {c: float(df_clean[c].min()) for c in feature_columns}
        maxs = {c: float(df_clean[c].max()) for c in feature_columns}

    # キー生成のヘルパ
    def make_keys(group_cols):
        if not group_cols:
            return [()]  # 単一グループ
        # 指定順でユニークな組を作る
        uniq = df_clean[group_cols].drop_duplicates()
        # 指定順に基づき、安定ソート
        # pandasのdrop_duplicatesは出現順を維持するため、さらに各列で安定ソート
        # ただしユーザーが指定した順を最優先にするため、そのままvaluesを使う
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
            if df_cell.empty:
                # 空セルでもヘッダを見せるため軸は消さず、グリッド/目盛りのみ最小化
                ax.grid(False)
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_xlabel(curve_x)
                # 以降のプロット処理はスキップ
                continue

            # 生データ散布
            if curve_x not in df_cell.columns:
                raise ValueError(f"x 軸列 '{curve_x}' がデータに存在しません。")
            x_raw = df_cell[curve_x].values
            y_raw = df_cell[target_column].values
            ax.scatter(x_raw, y_raw, alpha=0.5, s=25, label='raw')

            # 予測用グリッド
            x_min = mins.get(curve_x, float(df_cell[curve_x].min()))
            x_max = maxs.get(curve_x, float(df_cell[curve_x].max()))
            if not np.isfinite(x_min) or not np.isfinite(x_max):
                x_min = float(df_cell[curve_x].min())
                x_max = float(df_cell[curve_x].max())
            if x_min == x_max:
                x_min -= 1e-6
                x_max += 1e-6
            x_grid = np.linspace(x_min, x_max, int(curve_points))

            X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
            for j, fcol in enumerate(feature_columns):
                if fcol == curve_x:
                    X_grid[:, j] = x_grid
                elif (row_group_by and fcol in row_group_by):
                    # rkeyの該当値
                    val = rkey[row_group_by.index(fcol)] if fcol in row_group_by else float(df_cell[fcol].median())
                    X_grid[:, j] = float(val)
                elif (col_group_by and fcol in col_group_by):
                    val = ckey[col_group_by.index(fcol)] if fcol in col_group_by else float(df_cell[fcol].median())
                    X_grid[:, j] = float(val)
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

            ax.set_xlabel(curve_x)
            ax.grid(True, alpha=0.3)
            ax.legend(frameon=True, fontsize=10)

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
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は表示のみ）')
    parser.add_argument('--test-size', type=float, default=0.2, help='テストデータの割合（デフォルト: 0.2）')
    parser.add_argument('--alpha', type=float, default=1e-6, help='ノイズの分散（デフォルト: 1e-6）')
    parser.add_argument('--n-restarts', type=int, default=2, help='最適化の再起動回数（デフォルト: 2）')
    parser.add_argument('--random-state', type=int, default=42, help='乱数のシード（デフォルト: 42）')
    parser.add_argument('--normalize', action='store_true', help='特徴量の正規化を行う')
    parser.add_argument('--cv-folds', type=int, default=0, help='クロスバリデーションの分割数（0で無効）')
    parser.add_argument('--cv-jobs', type=int, default=1, help='クロスバリデーションの並列数（デフォルト: 1）')
    parser.add_argument('--max-train-samples', type=int, default=None, help='学習に使用する最大サンプル数（指定時はランダム抽出）')
    parser.add_argument('--no-uncertainty', action='store_true', help='予測の不確実性（標準偏差）を計算しない')
    parser.add_argument('--features', type=str, default=None, help='使用する特徴量をカンマ区切りで指定（未指定ならtorque_x以外の全列）')
    parser.add_argument('--list-features', action='store_true', help='利用可能な特徴量候補を一覧表示して終了')
    parser.add_argument('--target', type=str, default='torque_x', help='目的変数の列名（デフォルト: torque_x）')
    parser.add_argument(
        '--target-log-abs',
        action='store_true',
        help='目的変数を log(|target|) に変換して学習・評価する（torque_x が正負混在する場合などの指数解析用）'
    )
    parser.add_argument('--anisotropic', action='store_true', help='各次元で別のlength_scaleを学習する（RBF/Matern）')
    parser.add_argument('--length-scale-bounds', type=str, default=None, help='length_scaleの下限,上限（例: 1e-2,1e3）')
    parser.add_argument('--matern-nu', type=float, default=1.5, help='Maternカーネルのnu（0.5,1.5,2.5など）')
    parser.add_argument('--kernel', choices=['rbf', 'matern', 'rbf_white', 'rbf_linear'], default='rbf',
                       help='カーネルの種類（デフォルト: rbf）')
    parser.add_argument('--do-grid-search', action='store_true', help='alpha と length_scale_bounds を簡易グリッドで探索')
    parser.add_argument('--max-variance-torque-x', type=float, default=None, help='variance_torque_x の上限で高分散サンプルを除外')
    parser.add_argument(
        '--add-log-force-z',
        type=str,
        default=None,
        help='指定した列名（例: force_z）から log(|col|) を計算し、log_<col> として特徴量に追加する'
    )
    # ペアワイズヒートマップ関連
    parser.add_argument('--pairwise-heatmaps', action='store_true', help='全特徴量ペアの予測ヒートマップを一括出力')
    parser.add_argument('--grid', type=int, default=80, help='ヒートマップの格子数（デフォルト: 80）')
    parser.add_argument('--fix', action='append', default=None, help="非可視化軸の固定値 'col=value' を複数指定可")
    parser.add_argument('--range', dest='ranges', action='append', default=None, help="各軸の範囲 'col:min,max' を複数指定可")
    parser.add_argument('--std-heatmaps', action='store_true', help='標準偏差のヒートマップも保存')
    parser.add_argument(
        '--plot-n-eff',
        action='store_true',
        help='distance × log_force のグリッド上で局所的な有効指数 n_eff(d, F) のヒートマップを出力する'
    )
    parser.add_argument(
        '--n-eff-distance-col',
        type=str,
        default='distance',
        help='n_eff の distance 軸に用いる列名（デフォルト: distance）'
    )
    parser.add_argument(
        '--n-eff-logforce-col',
        type=str,
        default='log_force_z',
        help='n_eff の log(force) 軸に用いる列名（デフォルト: log_force_z）'
    )
    parser.add_argument('--overlay-raw', action='store_true', help='ヒートマップ上に生データ点 (×) を重ねて表示')
    parser.add_argument('--contour-lines', action='store_true', help='予測平均ヒートマップに等高線を重ねて描画')
    parser.add_argument('--contour-levels', type=int, default=10, help='等高線レベル数（デフォルト: 10）')
    parser.add_argument('--contour-color', type=str, default='k', help='等高線の色（例: k, w, #RRGGBB）')
    parser.add_argument('--contour-linewidth', type=float, default=0.8, help='等高線の線幅（デフォルト: 0.8）')
    parser.add_argument('--contour-alpha', type=float, default=0.8, help='等高線の透明度（デフォルト: 0.8）')
    # 生データとフィット曲線の比較
    parser.add_argument('--plot-raw-fit', action='store_true', help='グループごとに生データ散布とGPRフィット曲線（±2σ帯）を比較表示')
    parser.add_argument('--group-by', type=str, default=None, help='グループ化に用いる列をカンマ区切りで指定（例: distance,tilt_angle）')
    parser.add_argument('--curve-x', type=str, default=None, help='フィット曲線の横軸にする特徴量（未指定なら group_by 以外の最初の特徴量）')
    parser.add_argument('--curve-points', type=int, default=200, help='フィット曲線の分解能')
    parser.add_argument('--groupfit-output', type=str, default=None, help='グループ別 Raw vs Fit 図の出力パス（未指定なら表示、--output 指定時は派生名を使用）')
    parser.add_argument('--row-group-by', type=str, default=None, help='行方向のファセットに用いる列（カンマ区切りの複数可、指定順でソート）')
    parser.add_argument('--col-group-by', type=str, default=None, help='列方向のファセットに用いる列（カンマ区切りの複数可、指定順でソート）')
    parser.add_argument('--facet-filter', action='append', default=None, help="ファセット用の事前フィルタ 'col:min,max' を複数指定可")
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
    
    args = parser.parse_args()
    
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
        if args.load_model:
            print(f"=== 保存済みモデルの読み込み: {args.load_model} ===")
            try:
                bundle = load(args.load_model)
                if isinstance(bundle, dict) and 'model' in bundle:
                    loaded_model = bundle['model']
                    loaded_scaler = bundle.get('scaler', None)
                    loaded_feature_columns = bundle.get('feature_columns', None)
                else:
                    # モデル単体が保存されていた場合への後方互換
                    loaded_model = bundle
                if loaded_feature_columns is not None:
                    print(f"保存モデルの特徴量: {loaded_feature_columns}")
            except Exception as e:
                raise RuntimeError(f"モデルの読み込みに失敗しました: {e}")

        # データの読み込みと前処理
        print("=== データの読み込みと前処理 ===")
        selected_features = None
        # 通常モード: ユーザー指定の --features をそのまま使う
        if args.features:
            selected_features = [c.strip() for c in args.features.split(',') if c.strip()]
        # モデルロード時: 保存済みモデルの特徴量に対応する「元列」を使って読み込む
        if args.load_model and loaded_feature_columns is not None:
            base_features: set[str] = set()
            for fcol in loaded_feature_columns:
                if fcol.startswith("log_"):
                    base = fcol[4:]
                    base_features.add(base)
                else:
                    base_features.add(fcol)
            selected_features = sorted(base_features)
            if args.features:
                print("注意: --load-model が指定されたため、--features は無視されます。")
            print(f"保存モデルに合わせて特徴量列を読み込みます（元列）: {selected_features}")
        df_clean, feature_columns = load_and_preprocess_data(
            args.csv_file,
            selected_features,
            max_variance_torque_x=args.max_variance_torque_x,
            target_column=args.target
        )
        
        # 追加特徴量: log(|force_z|) などを自動生成（指定時）
        if args.add_log_force_z:
            src_col = args.add_log_force_z
            if src_col not in df_clean.columns:
                raise ValueError(f"--add-log-force-z で指定された列 '{src_col}' がデータに存在しません。")
            vals = df_clean[src_col].to_numpy(dtype=float)
            mask = np.isfinite(vals) & (np.abs(vals) > 0.0)
            dropped = int(len(vals) - mask.sum())
            if dropped > 0:
                print(f"[add_log_force_z] {src_col}=0 または NaN の {dropped} 行を除外します。")
            df_clean = df_clean.loc[mask].copy()
            new_col = f"log_{src_col}"
            df_clean[new_col] = np.log(np.abs(df_clean[src_col].to_numpy(dtype=float)))
            if new_col not in feature_columns:
                feature_columns.append(new_col)
            print(f"[add_log_force_z] 特徴量 '{new_col}' を追加しました。")

        # 保存モデルが log_* な特徴量を要求している場合、自動で再構成（ロード時）
        if args.load_model and loaded_feature_columns is not None:
            for fcol in loaded_feature_columns:
                if fcol not in df_clean.columns and fcol.startswith("log_"):
                    base = fcol[4:]
                    if base in df_clean.columns:
                        vals = df_clean[base].to_numpy(dtype=float)
                        mask = np.isfinite(vals) & (np.abs(vals) > 0.0)
                        dropped = int(len(vals) - mask.sum())
                        if dropped > 0:
                            print(f"[auto_log_feature] {base}=0 または NaN の {dropped} 行を除外します。")
                        df_clean = df_clean.loc[mask].copy()
                        df_clean[fcol] = np.log(np.abs(df_clean[base].to_numpy(dtype=float)))
                        print(f"[auto_log_feature] 保存モデルに合わせて特徴量 '{fcol}' を再構成しました（元列: '{base}'）。")
                    else:
                        raise ValueError(
                            f"保存モデルが特徴量 '{fcol}' を要求していますが、元となる列 '{base}' がCSVに存在しません。"
                        )
            # feature_columns を保存モデルに合わせて上書き
            missing = [c for c in loaded_feature_columns if c not in df_clean.columns]
            if missing:
                raise ValueError(f"保存モデルが要求する特徴量がデータに存在しません: {missing}")
            feature_columns = list(loaded_feature_columns)

        # 目的変数の設定（必要に応じて log(|target|) へ変換）
        target_label = args.target
        if args.target_log_abs:
            tvals = df_clean[args.target].to_numpy(dtype=float)
            mask_t = np.isfinite(tvals) & (np.abs(tvals) > 0.0)
            dropped_t = int(len(tvals) - mask_t.sum())
            if dropped_t > 0:
                print(f"[target_log_abs] target が 0 または NaN の {dropped_t} 行を除外します。")
            df_clean = df_clean.loc[mask_t].copy()
            df_clean[args.target] = np.log(np.abs(df_clean[args.target].to_numpy(dtype=float)))
            target_label = f"log|{args.target}|"
            print(f"[target_log_abs] 目的変数を log(|{args.target}|) に変換しました。")

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
        
        # 訓練データとテストデータの分割
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=args.test_size, random_state=args.random_state
        )
        
        print(f"訓練データ数: {len(X_train)}")
        print(f"テストデータ数: {len(X_test)}")

        # 学習データのサブサンプリング（必要に応じて）
        if args.max_train_samples is not None and len(X_train) > args.max_train_samples:
            rs = np.random.RandomState(args.random_state)
            indices = rs.choice(len(X_train), size=args.max_train_samples, replace=False)
            X_train = X_train[indices]
            y_train = y_train[indices]
            print(f"学習データをサブサンプリング: {len(indices)} サンプルを使用")
        
        # ガウス過程回帰モデルの作成/読み込みと訓練
        if args.load_model:
            gp_model = loaded_model
            if gp_model is None:
                raise RuntimeError("--load-model が指定されましたが、モデルを取得できませんでした。")
            print(f"保存済みモデルを読み込みました。最適化されたカーネル: {getattr(gp_model, 'kernel_', getattr(gp_model, 'kernel', None))}")
        else:
            print(f"\n=== ガウス過程回帰モデルの訓練 ===")
            print(f"カーネル: {args.kernel}")
            print(f"アルファ: {args.alpha}")
            print(f"最適化再起動回数: {args.n_restarts}")

            # length_scale boundsの解釈
            ls_bounds = None
            if args.length_scale_bounds:
                try:
                    lo, hi = [float(x) for x in args.length_scale_bounds.split(',')]
                    ls_bounds = (lo, hi)
                except Exception:
                    raise ValueError("--length-scale-bounds は '低,高' の形式で指定してください（例: 1e-2,1e3）")

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
                        span = max(base[1] - base[0], 1e-6)
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
        plot_results._target_column = target_label
        plot_results(y_test, metrics['y_pred'], metrics.get('y_std', None), 
                    feature_columns, results_output)
        plot_feature_importance(gp_model, feature_columns, importance_output)

        # ペアワイズヒートマップ
        if args.pairwise_heatmaps:
            print("\n=== ペアワイズヒートマップの生成 ===")
            output_prefix = None
            if args.output:
                output_prefix = os.path.splitext(args.output)[0]
            generate_pairwise_heatmaps._target_column = target_label
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
                contour_alpha=args.contour_alpha
            )

        # 局所的な有効指数 n_eff(d, F) のヒートマップ
        if args.plot_n_eff:
            print("\n=== 局所的な有効指数 n_eff(distance, log_force) ヒートマップの生成 ===")
            if args.output:
                base = os.path.splitext(args.output)[0]
                n_eff_out = f"{base}_n_eff_{args.n_eff_distance_col}__{args.n_eff_logforce_col}.png"
            else:
                n_eff_out = "n_eff_heatmap.png"
            generate_n_eff_heatmap(
                model=gp_model,
                feature_columns=feature_columns,
                df_clean=df_clean,
                scaler=scaler,
                distance_col=args.n_eff_distance_col,
                logforce_col=args.n_eff_logforce_col,
                grid_size=args.grid,
                fixes=args.fix,
                ranges=args.ranges,
                output_path=n_eff_out,
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
                    plot_facet_raw_and_fit_gpr._target_column = target_label
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
                        show_uncertainty=(not args.no_uncertainty)
                    )
                elif group_by:
                    plot_grouped_raw_and_fit_gpr._target_column = target_label
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
                        show_uncertainty=(not args.no_uncertainty)
                    )
                else:
                    print("(注意) --plot-raw-fit は指定されましたが、--group-by も --row-group-by/--col-group-by も未指定です。スキップします。")
            except Exception as e:
                print(f"グループ別 Raw vs GPR Fit の描画でエラー: {e}")
        
        # 統計情報の表示
        print(f"\n=== 統計情報 ===")
        print(f"使用した特徴量: {feature_columns}")
        print(f"データポイント数: {len(df_clean)}")
        print(f"{target_label}の範囲: {y.min():.6f} - {y.max():.6f}")
        print(f"{target_label}の平均: {y.mean():.6f}")
        print(f"{target_label}の標準偏差: {y.std():.6f}")
        
        # 予測の不確実性の統計
        if metrics.get('y_std', None) is not None:
            print(f"予測の不確実性:")
            print(f"  平均標準偏差: {metrics['y_std'].mean():.6f}")
            print(f"  最大標準偏差: {metrics['y_std'].max():.6f}")
            print(f"  最小標準偏差: {metrics['y_std'].min():.6f}")
        
        # モデルの保存
        if args.save_model:
            try:
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
                print(f"モデルを保存しました: {args.save_model}")
            except Exception as e:
                print(f"モデルの保存に失敗しました: {e}")
        
        return 0
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        return 1

if __name__ == "__main__":
    exit(main())
