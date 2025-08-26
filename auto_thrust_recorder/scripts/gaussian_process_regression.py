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
import matplotlib.pyplot as plt
import argparse
import os
import japanize_matplotlib
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, Matern, ConstantKernel
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
import warnings
warnings.filterwarnings('ignore')

def load_and_preprocess_data(csv_file):
    """
    CSVファイルを読み込み、データの前処理を行う
    
    Args:
        csv_file: CSVファイルのパス
    
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
    
    # 基本的な必要な列
    basic_required_columns = ['torque_x', 'distance', 'tilt_angle']
    
    # オプションの特徴量列（存在する場合のみ使用）
    # target_thrust, sample_count, variance_torque_xは除外
    optional_feature_columns = ['wall_spacing', 'fold_angle', 'force_z']
    
    # 必要な列の存在確認
    missing_basic_columns = [col for col in basic_required_columns if col not in df.columns]
    if missing_basic_columns:
        raise ValueError(f"必要な列が見つかりません: {missing_basic_columns}")
    
    # 利用可能な特徴量列を特定
    available_feature_columns = []
    for col in optional_feature_columns:
        if col in df.columns:
            available_feature_columns.append(col)
    
    print(f"利用可能な特徴量: {available_feature_columns}")
    
    # 特徴量列のリストを作成
    feature_columns = ['distance', 'tilt_angle'] + available_feature_columns
    
    # 目的変数と特徴量の列を数値型に変換
    numeric_columns = ['torque_x'] + feature_columns
    for col in numeric_columns:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # NaNを含む行を削除
    df_clean = df.dropna(subset=['torque_x'] + feature_columns)
    
    if df_clean.empty:
        raise ValueError("有効なデータがありません。")
    
    print(f"前処理後のデータ行数: {len(df_clean)}")
    print(f"使用する特徴量: {feature_columns}")
    
    return df_clean, feature_columns

def create_gp_model(kernel_type='rbf', alpha=1e-6, n_restarts_optimizer=10):
    """
    ガウス過程回帰モデルを作成する
    
    Args:
        kernel_type: カーネルの種類 ('rbf', 'matern', 'rbf_white')
        alpha: ノイズの分散
        n_restarts_optimizer: 最適化の再起動回数
    
    Returns:
        gp_model: ガウス過程回帰モデル
    """
    if kernel_type == 'rbf':
        kernel = ConstantKernel(1.0) * RBF(1.0)
    elif kernel_type == 'matern':
        kernel = ConstantKernel(1.0) * Matern(1.0, nu=1.5)
    elif kernel_type == 'rbf_white':
        kernel = ConstantKernel(1.0) * RBF(1.0) + WhiteKernel(noise_level=1e-6)
    else:
        kernel = ConstantKernel(1.0) * RBF(1.0)
    
    gp_model = GaussianProcessRegressor(
        kernel=kernel,
        alpha=alpha,
        n_restarts_optimizer=n_restarts_optimizer,
        random_state=42
    )
    
    return gp_model

def evaluate_model(model, X_test, y_test, X_train=None, y_train=None):
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
    y_pred, y_std = model.predict(X_test, return_std=True)
    
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
    
    # クロスバリデーション（訓練データが提供された場合）
    if X_train is not None and y_train is not None:
        cv_scores = cross_val_score(model, X_train, y_train, cv=5, scoring='neg_mean_squared_error')
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
    ax3.scatter(y_pred, y_std, alpha=0.6, s=50)
    ax3.set_xlabel('予測値 (torque_x)')
    ax3.set_ylabel('予測の標準偏差')
    ax3.set_title('予測の不確実性')
    ax3.grid(True, alpha=0.3)
    
    # 4. 時系列プロット（インデックス順）
    ax4 = axes[1, 1]
    indices = range(len(y_test))
    ax4.plot(indices, y_test, 'o-', label='実際の値', alpha=0.7)
    ax4.plot(indices, y_pred, 's-', label='予測値', alpha=0.7)
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

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='ガウス過程回帰によるtorque_xのモデル化')
    parser.add_argument('csv_file', help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は表示のみ）')
    parser.add_argument('--test-size', type=float, default=0.2, help='テストデータの割合（デフォルト: 0.2）')
    parser.add_argument('--kernel', choices=['rbf', 'matern', 'rbf_white'], default='rbf',
                       help='カーネルの種類（デフォルト: rbf）')
    parser.add_argument('--alpha', type=float, default=1e-6, help='ノイズの分散（デフォルト: 1e-6）')
    parser.add_argument('--n-restarts', type=int, default=10, help='最適化の再起動回数（デフォルト: 10）')
    parser.add_argument('--random-state', type=int, default=42, help='乱数のシード（デフォルト: 42）')
    parser.add_argument('--normalize', action='store_true', help='特徴量の正規化を行う')
    
    args = parser.parse_args()
    
    try:
        # データの読み込みと前処理
        print("=== データの読み込みと前処理 ===")
        df_clean, feature_columns = load_and_preprocess_data(args.csv_file)
        
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
        
        # ガウス過程回帰モデルの作成と訓練
        print(f"\n=== ガウス過程回帰モデルの訓練 ===")
        print(f"カーネル: {args.kernel}")
        print(f"アルファ: {args.alpha}")
        print(f"最適化再起動回数: {args.n_restarts}")
        
        gp_model = create_gp_model(
            kernel_type=args.kernel,
            alpha=args.alpha,
            n_restarts_optimizer=args.n_restarts
        )
        
        # モデルの訓練
        print("モデルを訓練中...")
        gp_model.fit(X_train, y_train)
        
        # 最適化されたカーネルパラメータの表示
        print(f"最適化されたカーネル: {gp_model.kernel_}")
        
        # モデルの評価
        print(f"\n=== モデルの評価 ===")
        metrics = evaluate_model(gp_model, X_test, y_test, X_train, y_train)
        
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
        
        plot_results(y_test, metrics['y_pred'], metrics['y_std'], 
                    feature_columns, results_output)
        plot_feature_importance(gp_model, feature_columns, importance_output)
        
        # 統計情報の表示
        print(f"\n=== 統計情報 ===")
        print(f"使用した特徴量: {feature_columns}")
        print(f"データポイント数: {len(df_clean)}")
        print(f"torque_xの範囲: {y.min():.6f} - {y.max():.6f}")
        print(f"torque_xの平均: {y.mean():.6f}")
        print(f"torque_xの標準偏差: {y.std():.6f}")
        
        # 予測の不確実性の統計
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
