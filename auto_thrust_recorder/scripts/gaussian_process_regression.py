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
import japanize_matplotlib
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, Matern, ConstantKernel, DotProduct
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
import warnings
warnings.filterwarnings('ignore')

def load_and_preprocess_data(csv_file, selected_features=None, max_variance_torque_x=None):
    """
    CSVファイルを読み込み、データの前処理を行う
    
    Args:
        csv_file: CSVファイルのパス
        selected_features: 使用する特徴量のリスト（Noneの場合はtorque_x以外の全列）
        max_variance_torque_x: 'variance_torque_x' による上限フィルタ（Noneで無効）
    
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
    if 'torque_x' not in df.columns:
        raise ValueError("必要な列が見つかりません: ['torque_x']")

    # 特徴量列の決定
    if selected_features is not None and len(selected_features) > 0:
        if 'torque_x' in selected_features:
            raise ValueError("'torque_x' は目的変数のため特徴量に含められません。")
        missing = [c for c in selected_features if c not in df.columns]
        if missing:
            raise ValueError(f"指定された特徴量が見つかりません: {missing}")
        feature_columns = list(selected_features)
    else:
        # 既定はtorque_x以外の全列
        feature_columns = [c for c in df.columns if c != 'torque_x']
    
    # 目的変数と特徴量の列を数値型に変換
    numeric_columns = ['torque_x'] + feature_columns
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
    df_clean = df.dropna(subset=['torque_x'] + feature_columns)
    
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
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # 1. 予測値 vs 実際の値
    ax1 = axes[0, 0]
    ax1.scatter(y_test, y_pred, alpha=0.6, s=50)
    ax1.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], 'r--', lw=2)
    ax1.set_xlabel('実際の値 (torque_x)')
    ax1.set_ylabel('予測値 (torque_x)')
    ax1.set_title('予測値 vs 実際の値')
    ax1.grid(True, alpha=0.3)
    
    # 2. 残差プロット
    ax2 = axes[0, 1]
    residuals = y_test - y_pred
    ax2.scatter(y_pred, residuals, alpha=0.6, s=50)
    ax2.axhline(y=0, color='r', linestyle='--')
    ax2.set_xlabel('予測値 (torque_x)')
    ax2.set_ylabel('残差')
    ax2.set_title('残差プロット')
    ax2.grid(True, alpha=0.3)
    
    # 3. 予測の不確実性
    ax3 = axes[1, 0]
    if y_std is not None:
        ax3.scatter(y_pred, y_std, alpha=0.6, s=50)
        ax3.set_xlabel('予測値 (torque_x)')
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
    ax4.set_ylabel('torque_x')
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
    plt.figure(figsize=(10, 6))
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

def generate_pairwise_heatmaps(model, feature_columns, df_clean, scaler, grid_size, fixes, ranges, output_prefix, include_std):
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

            # 平均のヒートマップ
            plt.figure(figsize=(6, 5))
            plt.imshow(Zm, origin='lower', aspect='auto',
                       extent=[mins[fi], maxs[fi], mins[fj], maxs[fj]],
                       cmap='viridis')
            plt.colorbar(label='予測平均 (torque_x)')
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
                plt.figure(figsize=(6, 5))
                plt.imshow(Zs, origin='lower', aspect='auto',
                           extent=[mins[fi], maxs[fi], mins[fj], maxs[fj]],
                           cmap='magma')
                plt.colorbar(label='予測標準偏差')
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
    parser.add_argument('--anisotropic', action='store_true', help='各次元で別のlength_scaleを学習する（RBF/Matern）')
    parser.add_argument('--length-scale-bounds', type=str, default=None, help='length_scaleの下限,上限（例: 1e-2,1e3）')
    parser.add_argument('--matern-nu', type=float, default=1.5, help='Maternカーネルのnu（0.5,1.5,2.5など）')
    parser.add_argument('--kernel', choices=['rbf', 'matern', 'rbf_white', 'rbf_linear'], default='rbf',
                       help='カーネルの種類（デフォルト: rbf）')
    parser.add_argument('--do-grid-search', action='store_true', help='alpha と length_scale_bounds を簡易グリッドで探索')
    parser.add_argument('--max-variance-torque-x', type=float, default=None, help='variance_torque_x の上限で高分散サンプルを除外')
    # ペアワイズヒートマップ関連
    parser.add_argument('--pairwise-heatmaps', action='store_true', help='全特徴量ペアの予測ヒートマップを一括出力')
    parser.add_argument('--grid', type=int, default=80, help='ヒートマップの格子数（デフォルト: 80）')
    parser.add_argument('--fix', action='append', default=None, help="非可視化軸の固定値 'col=value' を複数指定可")
    parser.add_argument('--range', dest='ranges', action='append', default=None, help="各軸の範囲 'col:min,max' を複数指定可")
    parser.add_argument('--std-heatmaps', action='store_true', help='標準偏差のヒートマップも保存')
    
    args = parser.parse_args()
    
    try:
        # 特徴量一覧の表示のみ
        if args.list_features:
            print("=== 特徴量一覧 ===")
            if not os.path.exists(args.csv_file):
                raise FileNotFoundError(f"ファイル '{args.csv_file}' が見つかりません。")
            df_head = pd.read_csv(args.csv_file, nrows=5)
            candidates = [c for c in df_head.columns if c != 'torque_x']
            print(f"利用可能な特徴量候補: {candidates}")
            return 0

        # 描画バックエンドの自動切替（ヘッドレスや--output指定時）
        if (not os.environ.get('DISPLAY')) or args.output:
            try:
                matplotlib.use('Agg', force=True)
            except Exception:
                pass

        # データの読み込みと前処理
        print("=== データの読み込みと前処理 ===")
        selected_features = None
        if args.features:
            selected_features = [c.strip() for c in args.features.split(',') if c.strip()]
        df_clean, feature_columns = load_and_preprocess_data(
            args.csv_file,
            selected_features,
            max_variance_torque_x=args.max_variance_torque_x
        )
        
        # 特徴量と目的変数の準備
        X = df_clean[feature_columns].values
        y = df_clean['torque_x'].values
        
        print(f"特徴量の形状: {X.shape}")
        print(f"目的変数の形状: {y.shape}")
        
        # 特徴量の正規化（オプション）
        scaler = None
        if args.normalize:
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
        
        # ガウス過程回帰モデルの作成と訓練
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

        if args.do_grid_search:
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
        
        plot_results(y_test, metrics['y_pred'], metrics.get('y_std', None), 
                    feature_columns, results_output)
        plot_feature_importance(gp_model, feature_columns, importance_output)

        # ペアワイズヒートマップ
        if args.pairwise_heatmaps:
            print("\n=== ペアワイズヒートマップの生成 ===")
            output_prefix = None
            if args.output:
                output_prefix = os.path.splitext(args.output)[0]
            generate_pairwise_heatmaps(
                gp_model,
                feature_columns,
                df_clean,
                scaler,
                grid_size=args.grid,
                fixes=args.fix,
                ranges=args.ranges,
                output_prefix=output_prefix,
                include_std=args.std_heatmaps
            )
        
        # 統計情報の表示
        print(f"\n=== 統計情報 ===")
        print(f"使用した特徴量: {feature_columns}")
        print(f"データポイント数: {len(df_clean)}")
        print(f"torque_xの範囲: {y.min():.6f} - {y.max():.6f}")
        print(f"torque_xの平均: {y.mean():.6f}")
        print(f"torque_xの標準偏差: {y.std():.6f}")
        
        # 予測の不確実性の統計
        if metrics.get('y_std', None) is not None:
            print(f"予測の不確実性:")
            print(f"  平均標準偏差: {metrics['y_std'].mean():.6f}")
            print(f"  最大標準偏差: {metrics['y_std'].max():.6f}")
            print(f"  最小標準偏差: {metrics['y_std'].min():.6f}")
        
        return 0
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        return 1

if __name__ == "__main__":
    exit(main())
