#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

def calculate_weighted_variance(values, weights):
    """
    重み付き分散を計算する関数
    values: 値の配列
    weights: 重みの配列（sample_count）
    """
    if len(values) == 0:
        return 0
    
    # 重み付き平均を計算
    weighted_mean = np.average(values, weights=weights)
    
    # 重み付き分散を計算
    weighted_variance = np.average((values - weighted_mean) ** 2, weights=weights)
    
    return weighted_variance

def calculate_normal_distribution_variance(sample_counts, variance_torque_x):
    """
    各測定が正規分布に従うとして、sample_countとvariance_torque_xを使った
    重み付き平均の分散を計算する関数
    
    Args:
        sample_counts: 各測定点のサンプル数
        variance_torque_x: 各測定点の分散
    
    Returns:
        weighted_mean_variance: 重み付き平均の分散
        effective_sample_size: 有効サンプルサイズ
    """
    if len(sample_counts) == 0:
        return 0, 0
    
    # 重み付き平均の分散を計算
    # V(weighted_mean) = Σ(w_i² × σ_i²) / (Σw_i)²
    weighted_mean_variance = np.sum((sample_counts**2) * variance_torque_x) / (np.sum(sample_counts))**2
    
    # 有効サンプルサイズを計算
    # neff = (Σw_i)² / Σ(w_i²)
    effective_sample_size = (np.sum(sample_counts))**2 / np.sum(sample_counts**2)
    
    return weighted_mean_variance, effective_sample_size

def calculate_combined_normal_variance(torque_x_values, variance_torque_x_values):
    """
    複数の正規分布を合成したときの分散を計算する関数
    N1(μ1, σ1^2), N2(μ2, σ2^2), ..., Ni の正規分布を合成
    
    Args:
        torque_x_values: 各正規分布の平均値（μ）
        variance_torque_x_values: 各正規分布の分散（σ^2）
    
    Returns:
        combined_variance: 合成された正規分布の分散
    """
    if len(torque_x_values) == 0:
        return 0
    
    # 各正規分布の平均を計算
    mean_of_means = np.mean(torque_x_values)
    
    # 合成分散の計算
    # 1. 各正規分布の分散の平均（測定誤差の分散）
    mean_variance = np.mean(variance_torque_x_values)
    
    # 2. 各正規分布の平均値の分散（グループ間の分散）
    variance_of_means = np.var(torque_x_values, ddof=1)  # 不偏分散
    
    # 3. 合成分散 = 測定誤差の分散 + グループ間の分散
    combined_variance = mean_variance + variance_of_means
    
    return combined_variance

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='CSVファイルからtorque_x vs distanceのプロットを作成')
    parser.add_argument('csv_file', help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は表示のみ）')
    parser.add_argument('--dpi', type=int, default=300, help='画像のDPI（デフォルト: 300）')
    
    args = parser.parse_args()
    
    # CSVファイルの存在確認
    if not os.path.exists(args.csv_file):
        print(f"エラー: ファイル '{args.csv_file}' が見つかりません。")
        return 1
    
    # CSVファイルの読み込み
    try:
        df = pd.read_csv(args.csv_file)
        print(f"CSVファイル '{args.csv_file}' を読み込みました。")
        print(f"データ行数: {len(df)}")
    except Exception as e:
        print(f"CSVファイルの読み込み中にエラーが発生しました: {e}")
        return 1
    
    # 必要な列の存在確認
    required_columns = ['torque_x', 'distance', 'tilt_angle', 'variance_torque_x', 'sample_count']
    missing_columns = [col for col in required_columns if col not in df.columns]
    
    if missing_columns:
        print(f"エラー: 必要な列が見つかりません: {missing_columns}")
        print(f"利用可能な列: {list(df.columns)}")
        return 1
    
    # 関連する列を数値型に変換
    cols_to_numeric = ['torque_x', 'distance', 'tilt_angle', 'variance_torque_x', 'sample_count']
    
    for col in cols_to_numeric:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # NaNを含む行を削除
    df_clean = df.dropna(subset=['torque_x', 'distance', 'tilt_angle', 'variance_torque_x', 'sample_count'])
    
    if df_clean.empty:
        print("エラー: 有効なデータがありません。")
        return 1
    
    print(f"有効なデータ行数: {len(df_clean)}")
    
    # tilt_angleとdistanceでグループ化
    grouped_data = df_clean.groupby(['tilt_angle', 'distance'])
    
    if grouped_data.ngroups == 0:
        print("指定されたキーに基づくグループが見つかりませんでした。")
        return 1
    
    print(f"グループ数: {grouped_data.ngroups}")
    
    # 各グループで統計量を計算
    results = []
    
    for (tilt_angle, distance), group_df in grouped_data:
        # 重み付き平均を計算
        weighted_mean = np.average(group_df['torque_x'], weights=group_df['sample_count'])
        
        # 統計的な分散を計算
        # 方法1: 各データポイントの分散を重み付き平均（測定誤差の分散）
        measurement_variance = np.average(group_df['variance_torque_x'], weights=group_df['sample_count'])
        
        # 方法2: グループ内のtorque_x値の重み付き分散（グループ間の分散）
        group_variance = np.average((group_df['torque_x'] - weighted_mean) ** 2, weights=group_df['sample_count'])
        
        # 方法3: 正規分布を仮定した重み付き平均の分散（新しい方法）
        normal_dist_variance, effective_sample_size = calculate_normal_distribution_variance(
            group_df['sample_count'].values, 
            group_df['variance_torque_x'].values
        )
        
        # 方法4: 複数の正規分布を合成した分散（サンプルサイズ無視）
        combined_normal_variance = calculate_combined_normal_variance(
            group_df['torque_x'].values,
            group_df['variance_torque_x'].values
        )
        
        # 総分散 = 測定誤差の分散 + グループ間の分散
        total_variance = measurement_variance + group_variance
        
        # サンプル数の合計
        total_samples = group_df['sample_count'].sum()
        
        results.append({
            'tilt_angle': tilt_angle,
            'distance': distance,
            'torque_x_mean': weighted_mean,
            'torque_x_variance': total_variance,
            'measurement_variance': measurement_variance,
            'group_variance': group_variance,
            'normal_dist_variance': normal_dist_variance,
            'combined_normal_variance': combined_normal_variance,
            'effective_sample_size': effective_sample_size,
            'total_samples': total_samples
        })
    
    # 結果をDataFrameに変換
    results_df = pd.DataFrame(results)
    
    # プロットの作成
    plt.figure(figsize=(12, 8))
    
    # 各tilt_angleで異なる色とマーカーを使用
    unique_tilt_angles = sorted(results_df['tilt_angle'].unique())
    colors = plt.cm.tab10(np.linspace(0, 1, len(unique_tilt_angles)))
    markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
    
    for i, tilt_angle in enumerate(unique_tilt_angles):
        tilt_data = results_df[results_df['tilt_angle'] == tilt_angle].sort_values('distance')
        
        if len(tilt_data) > 0:
            # 線とマーカーでプロット
            plt.plot(tilt_data['distance'], tilt_data['torque_x_mean'], 
                    marker=markers[i % len(markers)], 
                    color=colors[i], 
                    linewidth=2, 
                    markersize=8,
                    label=f'Tilt Angle: {tilt_angle}°')
            
            # エラーバー（標準偏差）を表示
            std_values = np.sqrt(tilt_data['combined_normal_variance'])
            plt.errorbar(tilt_data['distance'], tilt_data['torque_x_mean'],
                        yerr=std_values,
                        fmt='none',
                        color=colors[i],
                        alpha=0.5,
                        capsize=5)
    
    # グラフの設定
    plt.xlabel('Distance (m)', fontsize=18)
    plt.ylabel('Torque X (Nm)', fontsize=18)
    plt.title('Torque X vs Distance (Combined Normal Distribution Variance)', fontsize=18)
    plt.legend(loc='upper right', fontsize=18)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.ylim(-0.3, 0.3) 
    
    # 軸の目盛りラベルのフォントサイズも設定
    plt.xticks(fontsize=18)
    plt.yticks(fontsize=18)
    
    # 合成分散の範囲をテキストで表示
    combined_variance_min = results_df['combined_normal_variance'].min()
    combined_variance_max = results_df['combined_normal_variance'].max()
    combined_variance_text = f'Combined Variance Range:\n{combined_variance_min:.6f} - {combined_variance_max:.6f}'
    
    # テキストボックスをグラフの左上に配置
    plt.text(0.02, 0.98, combined_variance_text,
             transform=plt.gca().transAxes,
             verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
             fontsize=18)
    
    # レイアウトの調整
    plt.tight_layout()
    
    # 出力処理
    if args.output:
        plt.savefig(args.output, dpi=args.dpi, bbox_inches='tight')
        print(f"プロットを '{args.output}' に保存しました。")
    else:
        plt.show()
    
    # 統計情報の表示
    print("\n=== 統計情報 ===")
    print(f"データポイント数: {len(results_df)}")
    print(f"Tilt Angleの種類: {len(unique_tilt_angles)}")
    print(f"Distanceの範囲: {results_df['distance'].min():.3f} - {results_df['distance'].max():.3f}")
    print(f"Torque Xの範囲: {results_df['torque_x_mean'].min():.3f} - {results_df['torque_x_mean'].max():.3f}")
    print(f"総分散の範囲: {results_df['torque_x_variance'].min():.6f} - {results_df['torque_x_variance'].max():.6f}")
    print(f"測定誤差分散の範囲: {results_df['measurement_variance'].min():.6f} - {results_df['measurement_variance'].max():.6f}")
    print(f"グループ間分散の範囲: {results_df['group_variance'].min():.6f} - {results_df['group_variance'].max():.6f}")
    print(f"正規分布仮定分散の範囲: {results_df['normal_dist_variance'].min():.6f} - {results_df['normal_dist_variance'].max():.6f}")
    print(f"合成分散の範囲: {results_df['combined_normal_variance'].min():.6f} - {results_df['combined_normal_variance'].max():.6f}")
    print(f"合計サンプルサイズの範囲: {results_df['total_samples'].min():.0f} - {results_df['total_samples'].max():.0f}")
    print(f"有効サンプルサイズの範囲: {results_df['effective_sample_size'].min():.1f} - {results_df['effective_sample_size'].max():.1f}")
    print(f"全体の合計サンプル数: {results_df['total_samples'].sum():.0f}")
    print(f"全体の有効サンプルサイズ: {results_df['effective_sample_size'].sum():.1f}")
    
    return 0

if __name__ == "__main__":
    exit(main())
