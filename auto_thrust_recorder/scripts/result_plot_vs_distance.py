#!/usr/bin/env python3
"""
Torque X Analysis Script
CSVファイルを読み込み、distanceでグループ化してtorque_xの平均と分散を計算し、
エラーバー付きの散布図でプロットします。
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

def parse_arguments():
    """コマンドライン引数を解析します。"""
    parser = argparse.ArgumentParser(
        description='CSVファイルを読み込み、torque_xの分析プロットを生成します。',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  python plot_torque_analysis.py data.csv
  python plot_torque_analysis.py data.csv --output torque_analysis.png
  python plot_torque_analysis.py data.csv --xlim 10 25 --ylim -0.2 0.2
  python plot_torque_analysis.py data.csv --no-show
  python plot_torque_analysis.py data.csv --thrust-filter 0.5 2.0
        """
    )
    
    parser.add_argument(
        'csv_file',
        help='分析対象のCSVファイルのパス'
    )
    
    parser.add_argument(
        '--output', '-o',
        default=None,
        help='出力画像ファイル名 (指定しない場合は保存しません)'
    )
    
    parser.add_argument(
        '--xlim',
        nargs=2,
        type=float,
        default=[0.0, 5.0],
        metavar=('MIN', 'MAX'),
        help='X軸の範囲 (デフォルト: 0.0 5.0)'
    )
    
    parser.add_argument(
        '--ylim',
        nargs=2,
        type=float,
        default=[-0.2, 0.2],
        metavar=('MIN', 'MAX'),
        help='Y軸の範囲 (デフォルト: -0.2 0.2)'
    )
    
    parser.add_argument(
        '--thrust-filter',
        nargs=2,
        type=float,
        default=None,
        metavar=('MIN', 'MAX'),
        help='target_thrustのフィルタ範囲 (指定しない場合はフィルタなし)'
    )
    
    parser.add_argument(
        '--figsize',
        nargs=2,
        type=int,
        default=[15, 10],
        metavar=('WIDTH', 'HEIGHT'),
        help='図のサイズ (デフォルト: 15 10)'
    )
    
    parser.add_argument(
        '--dpi',
        type=int,
        default=100,
        help='画像の解像度 (デフォルト: 300)'
    )
    
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='プロットを画面に表示しない'
    )
    
    return parser.parse_args()

def load_and_validate_csv(csv_file):
    """CSVファイルを読み込み、必要な列の存在を確認します。"""
    try:
        df = pd.read_csv(csv_file)
        print(f"CSVファイル '{csv_file}' を読み込みました。")
        print(f"データ形状: {df.shape}")
        print(f"列名: {list(df.columns)}")
        
        # 必要な列の存在確認
        required_columns = ['distance', 'torque_x', 'target_thrust']
        missing_columns = [col for col in required_columns if col not in df.columns]
        
        if missing_columns:
            print(f"エラー: 必要な列が見つかりません: {missing_columns}")
            return None
            
        return df
        
    except FileNotFoundError:
        print(f"エラー: ファイル '{csv_file}' が見つかりません。")
        return None
    except Exception as e:
        print(f"CSVファイルの読み込み中にエラーが発生しました: {e}")
        return None

def convert_numeric_columns(df):
    """数値型に変換可能な列を変換します。"""
    # 数値型に変換する列のリスト
    numeric_columns = [
        'target_thrust', 'sample_count', 'control', 'force_x', 'force_y', 'force_z',
        'torque_x', 'torque_y', 'torque_z', 'variance_force_x', 'variance_force_y',
        'variance_force_z', 'variance_torque_x', 'variance_torque_y', 'variance_torque_z',
        'distance', 'tilt_angle', 'fold_angle', 'prop_spacing', 'height'
    ]
    
    for col in numeric_columns:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    return df

def analyze_torque_data(df, thrust_filter=None):
    """distanceでグループ化してtorque_xの統計を計算します。"""
    # 必要な列のNaNを含む行を削除
    df_clean = df.dropna(subset=['distance', 'torque_x', 'target_thrust'])
    
    if df_clean.empty:
        print("警告: 有効なデータがありません。")
        return None
    
    # target_thrustフィルタを適用
    if thrust_filter is not None:
        min_thrust, max_thrust = thrust_filter
        df_clean = df_clean[(df_clean['target_thrust'] >= min_thrust) & 
                           (df_clean['target_thrust'] <= max_thrust)]
        print(f"target_thrustフィルタ適用: {min_thrust} <= target_thrust <= {max_thrust}")
        print(f"フィルタ適用後のデータ数: {len(df_clean)}")
        
        if df_clean.empty:
            print("警告: フィルタ適用後に有効なデータがありません。")
            return None
    
    # distanceでグループ化して統計を計算
    grouped_stats = df_clean.groupby('distance').agg({
        'torque_x': ['mean', 'std', 'count', 'var'],
        'target_thrust': ['mean', 'std']
    }).reset_index()
    
    # カラム名を整理
    grouped_stats.columns = [
        'distance', 'torque_x_mean', 'torque_x_std', 'torque_x_count', 'torque_x_var',
        'target_thrust_mean', 'target_thrust_std'
    ]
    
    print(f"分析結果:")
    print(f"  ユニークなdistance値: {len(grouped_stats)}")
    print(f"  データポイント数: {grouped_stats['torque_x_count'].sum()}")
    
    return grouped_stats

def create_plot(grouped_stats, args):
    """エラーバー付きの散布図を作成します。"""
    fig, ax = plt.subplots(figsize=args.figsize, dpi=args.dpi)
    
    # データポイントが1つ以上ある場合のみプロット
    valid_data = grouped_stats[grouped_stats['torque_x_count'] >= 1]
    
    if len(valid_data) == 0:
        ax.text(0.5, 0.5, "有効なデータがありません\n(各distanceで2つ以上のデータポイントが必要)",
                ha='center', va='center', fontsize=12, color='gray',
                transform=ax.transAxes)
        ax.set_title("Torque X Analysis - No Valid Data", fontsize=14)
        return fig
    
    # エラーバー付きの散布図
    ax.errorbar(valid_data['distance'], 
                valid_data['torque_x_mean'],
                yerr=valid_data['torque_x_std'],
                fmt='o',
                label='Torque X (mean ± std)',
                alpha=0.7,
                markersize=8,
                capsize=5,
                capthick=2,
                elinewidth=2,
                color='blue')
    
    # データポイントの数を表示
    for _, row in valid_data.iterrows():
        ax.annotate(f'n={int(row["torque_x_count"])}',
                   (row['distance'], row['torque_x_mean']),
                   xytext=(0, 10),
                   textcoords='offset points',
                   ha='center',
                   fontsize=10,
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
    
    # 近似直線の計算とプロット
    if len(valid_data) >= 2:
        try:
            # 重み付き最小二乗法（データポイント数の逆数を重みとして使用）
            weights = valid_data['torque_x_count'] / valid_data['torque_x_count'].sum()
            coeffs = np.polyfit(valid_data['distance'], valid_data['torque_x_mean'], 1, w=weights)
            poly_fn = np.poly1d(coeffs)
            
            x_fit = np.linspace(valid_data['distance'].min(), valid_data['distance'].max(), 100)
            y_fit = poly_fn(x_fit)
            
            ax.plot(x_fit, y_fit, 
                   color='red', 
                   linestyle='--', 
                   linewidth=2,
                   label=f'Fit line: y={coeffs[0]:.3f}x + {coeffs[1]:.3f}')
            
            # 重み付き決定係数（R²）の計算
            y_pred = poly_fn(valid_data['distance'])
            weighted_mean = np.average(valid_data['torque_x_mean'], weights=weights)
            r2 = 1 - (np.sum(weights * (valid_data['torque_x_mean'] - y_pred) ** 2) / 
                     np.sum(weights * (valid_data['torque_x_mean'] - weighted_mean) ** 2))
            
            ax.text(0.05, 0.95, 
                   f'R² = {r2:.3f}', 
                   transform=ax.transAxes,
                   fontsize=12,
                   verticalalignment='top',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
            
        except Exception as e:
            print(f"近似直線の計算に失敗しました: {e}")
    
    # 軸の設定
    ax.set_xlabel('Distance', fontsize=12)
    ax.set_ylabel('Torque X (Nm)', fontsize=12)
    
    # タイトルの設定（フィルタ情報を含む）
    title = 'Torque X vs Distance Analysis'
    if args.thrust_filter is not None:
        min_thrust, max_thrust = args.thrust_filter
        title += f' (Thrust: {min_thrust}-{max_thrust}N)'
    ax.set_title(title, fontsize=14, fontweight='bold')
    
    # 軸の範囲設定
    if args.xlim:
        ax.set_xlim(args.xlim)
    if args.ylim:
        ax.set_ylim(args.ylim)
    
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.legend(fontsize=11)
    
    # 統計情報を表示
    stats_text = f"Total data points: {grouped_stats['torque_x_count'].sum()}\n"
    stats_text += f"Unique distances: {len(grouped_stats)}\n"
    stats_text += f"Mean torque: {grouped_stats['torque_x_mean'].mean():.3f} ± {grouped_stats['torque_x_std'].mean():.3f} Nm"
    
    ax.text(0.02, 0.02, stats_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    return fig

def save_plot(fig, output_file):
    """プロットをファイルに保存します。"""
    try:
        fig.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"プロットを '{output_file}' に保存しました。")
        return True
    except Exception as e:
        print(f"プロットの保存中にエラーが発生しました: {e}")
        return False

def main():
    """メイン関数"""
    args = parse_arguments()
    
    # CSVファイルの存在確認
    if not os.path.exists(args.csv_file):
        print(f"エラー: ファイル '{args.csv_file}' が存在しません。")
        sys.exit(1)
    
    # CSVファイルの読み込み
    df = load_and_validate_csv(args.csv_file)
    if df is None:
        sys.exit(1)
    
    # 数値型への変換
    df = convert_numeric_columns(df)
    
    # データの分析
    grouped_stats = analyze_torque_data(df, args.thrust_filter)
    if grouped_stats is None:
        sys.exit(1)
    
    # プロットの作成
    fig = create_plot(grouped_stats, args)
    
    # プロットの保存
    if args.output:
        if not save_plot(fig, args.output):
            sys.exit(1)
    
    # 画面表示
    if not args.no_show:
        plt.show()
    
    print("分析が完了しました。")

if __name__ == "__main__":
    main() 