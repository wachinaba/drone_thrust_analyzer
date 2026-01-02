#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import japanize_matplotlib

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

def calculate_difference_plot_data(results_df):
    """
    frontとback_reversedの差を計算する関数
    
    Args:
        results_df: 結果のDataFrame
    
    Returns:
        difference_df: 差のデータを含むDataFrame
    """
    difference_data = []
    
    # 各(tilt_angle, wall_spacing, distance)の組み合わせで差を計算
    for (tilt_angle, wall_spacing, distance), group in results_df.groupby(['tilt_angle', 'wall_spacing', 'distance']):
        front_data = group[group['keyword'] == 'front']
        back_reversed_data = group[group['keyword'] == 'back_reversed']
        
        if len(front_data) > 0 and len(back_reversed_data) > 0:
            # frontとback_reversedの差を計算
            torque_diff = front_data['torque_x_mean'].iloc[0] - back_reversed_data['torque_x_mean'].iloc[0]
            
            # 分散の合成（差の分散 = frontの分散 + back_reversedの分散）
            variance_diff = front_data['torque_x_variance'].iloc[0] + back_reversed_data['torque_x_variance'].iloc[0]
            
            difference_data.append({
                'tilt_angle': tilt_angle,
                'wall_spacing': wall_spacing,
                'distance': distance,
                'torque_diff': torque_diff,
                'torque_diff_variance': variance_diff,
                'front_torque': front_data['torque_x_mean'].iloc[0],
                'back_reversed_torque': back_reversed_data['torque_x_mean'].iloc[0],
                'front_variance': front_data['torque_x_variance'].iloc[0],
                'back_reversed_variance': back_reversed_data['torque_x_variance'].iloc[0]
            })
    
    return pd.DataFrame(difference_data)

def calculate_averaged_difference_plot_data(results_df):
    """
    バイアス成分を平均して推定する関数
    
    Args:
        results_df: 結果のDataFrame
    
    Returns:
        averaged_difference_df: 平均化された差のデータを含むDataFrame
    """
    # まず、各(tilt_angle, wall_spacing)の組み合わせでバイアスを推定
    bias_estimates = []
    
    for (tilt_angle, wall_spacing), group in results_df.groupby(['tilt_angle', 'wall_spacing']):
        # このグループ内のすべてのdistanceでのfrontとback_reversedの差を計算
        group_differences = []
        
        for distance in group['distance'].unique():
            distance_group = group[group['distance'] == distance]
            front_data = distance_group[distance_group['keyword'] == 'front']
            back_reversed_data = distance_group[distance_group['keyword'] == 'back_reversed']
            
            if len(front_data) > 0 and len(back_reversed_data) > 0:
                # 差を計算（バイアス成分の2倍）
                torque_diff = front_data['torque_x_mean'].iloc[0] - back_reversed_data['torque_x_mean'].iloc[0]
                variance_diff = front_data['torque_x_variance'].iloc[0] + back_reversed_data['torque_x_variance'].iloc[0]
                
                group_differences.append({
                    'distance': distance,
                    'torque_diff': torque_diff,
                    'torque_diff_variance': variance_diff
                })
        
        if len(group_differences) > 0:
            # 普通の平均でバイアスを推定
            differences_df = pd.DataFrame(group_differences)
            weighted_bias = np.mean(differences_df['torque_diff'])
            
            # バイアス推定の分散を計算
            bias_variance = np.var(differences_df['torque_diff'], ddof=1)
            
            bias_estimates.append({
                'tilt_angle': tilt_angle,
                'wall_spacing': wall_spacing,
                'estimated_bias': weighted_bias / 2.0,  # バイアス成分の推定値（差の半分）
                'bias_variance': bias_variance / 4.0,   # バイアス推定の分散
                'num_measurements': len(group_differences)
            })
    
    bias_df = pd.DataFrame(bias_estimates)
    
    # 次に、各distanceでの真の壁効果を計算
    true_wall_effect_data = []
    
    for (tilt_angle, wall_spacing, distance), group in results_df.groupby(['tilt_angle', 'wall_spacing', 'distance']):
        front_data = group[group['keyword'] == 'front']
        back_reversed_data = group[group['keyword'] == 'back_reversed']
        
        if len(front_data) > 0 and len(back_reversed_data) > 0:
            # 対応するバイアス推定値を取得
            bias_row = bias_df[(bias_df['tilt_angle'] == tilt_angle) & (bias_df['wall_spacing'] == wall_spacing)]
            
            if len(bias_row) > 0:
                estimated_bias = bias_row['estimated_bias'].iloc[0]
                bias_variance = bias_row['bias_variance'].iloc[0]
                
                # 真の壁効果を計算
                # 真の壁効果 = (front_corrected + back_reversed_corrected) / 2
                front_torque = front_data['torque_x_mean'].iloc[0]
                back_reversed_torque = back_reversed_data['torque_x_mean'].iloc[0]
                
                # 各測定値からバイアスを引く/足す
                # front: x_front = a + b → front_corrected = front - b = a
                # back_reversed: x_back_reversed = a - b → back_reversed_corrected = back_reversed + b = a
                front_corrected = front_torque - estimated_bias
                back_reversed_corrected = back_reversed_torque + estimated_bias
                
                # 真の壁効果 = (front_corrected + back_reversed_corrected) / 2
                true_wall_effect = (front_corrected + back_reversed_corrected) / 2.0
                
                # 分散の計算
                front_variance = front_data['torque_x_variance'].iloc[0]
                back_reversed_variance = back_reversed_data['torque_x_variance'].iloc[0]
                
                # 真の壁効果の分散 = (front分散 + back_reversed分散 + バイアス分散) / 4
                true_wall_effect_variance = (front_variance + back_reversed_variance + bias_variance) / 4.0
                
                true_wall_effect_data.append({
                    'tilt_angle': tilt_angle,
                    'wall_spacing': wall_spacing,
                    'distance': distance,
                    'true_wall_effect': true_wall_effect,
                    'true_wall_effect_variance': true_wall_effect_variance,
                    'front_torque': front_torque,
                    'back_reversed_torque': back_reversed_torque,
                    'front_corrected': front_corrected,
                    'back_reversed_corrected': back_reversed_corrected,
                    'estimated_bias': estimated_bias,
                    'bias_variance': bias_variance,
                    'front_variance': front_variance,
                    'back_reversed_variance': back_reversed_variance
                })
    
    # 真の壁効果の平均を計算
    averaged_wall_effect_data = []
    
    for (tilt_angle, wall_spacing), group in pd.DataFrame(true_wall_effect_data).groupby(['tilt_angle', 'wall_spacing']):
        if len(group) > 0:
            # 普通の平均で真の壁効果を平均化
            averaged_wall_effect = np.mean(group['true_wall_effect'])
            
            # 平均化された壁効果の分散を計算
            averaged_variance = np.var(group['true_wall_effect'], ddof=1)
            
            averaged_wall_effect_data.append({
                'tilt_angle': tilt_angle,
                'wall_spacing': wall_spacing,
                'averaged_wall_effect': averaged_wall_effect,
                'averaged_wall_effect_variance': averaged_variance,
                'num_measurements': len(group)
            })
    
    averaged_wall_effect_df = pd.DataFrame(averaged_wall_effect_data)
    
    return pd.DataFrame(true_wall_effect_data), bias_df, averaged_wall_effect_df

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='CSVファイルからtorque_x vs distanceのプロットを作成')
    parser.add_argument('csv_file', help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は表示のみ）')
    parser.add_argument('--dpi', type=int, default=300, help='画像のDPI（デフォルト: 300）')
    parser.add_argument('--show_difference', action='store_true', help='frontとback_reversedの差のプロットを表示')
    parser.add_argument('--show_corrected_difference', action='store_true', help='バイアス補正された差のプロットを表示')
    parser.add_argument('--show_bias', action='store_true', help='推定したバイアスのプロットを表示')
    
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
    
    # 必要な列の存在確認（keyword列を追加）
    required_columns = ['torque_x', 'distance', 'tilt_angle', 'wall_spacing', 'variance_torque_x', 'sample_count', 'keyword']
    missing_columns = [col for col in required_columns if col not in df.columns]
    
    if missing_columns:
        print(f"エラー: 必要な列が見つかりません: {missing_columns}")
        print(f"利用可能な列: {list(df.columns)}")
        return 1
    
    # 関連する列を数値型に変換
    cols_to_numeric = ['torque_x', 'distance', 'tilt_angle', 'wall_spacing', 'variance_torque_x', 'sample_count']
    
    for col in cols_to_numeric:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # NaNを含む行を削除
    df_clean = df.dropna(subset=['torque_x', 'distance', 'tilt_angle', 'wall_spacing', 'variance_torque_x', 'sample_count', 'keyword'])
    
    if df_clean.empty:
        print("エラー: 有効なデータがありません。")
        return 1
    
    print(f"有効なデータ行数: {len(df_clean)}")
    
    # keywordの種類を確認
    unique_keywords = df_clean['keyword'].unique()
    print(f"キーワードの種類: {unique_keywords}")
    
    # tilt_angle、wall_distance、keywordでグループ化
    grouped_data = df_clean.groupby(['tilt_angle', 'wall_spacing', 'keyword'])
    
    if grouped_data.ngroups == 0:
        print("指定されたキーに基づくグループが見つかりませんでした。")
        return 1
    
    print(f"グループ数: {grouped_data.ngroups}")
    
    # 各グループで統計量を計算
    results = []
    
    for (tilt_angle, wall_spacing, keyword), group_df in grouped_data:
        # 各distance値に対して個別に統計量を計算
        for distance in group_df['distance'].unique():
            # 特定のdistance値のデータを抽出
            distance_data = group_df[group_df['distance'] == distance]
            
            if len(distance_data) == 0:
                continue
                
            # 重み付き平均を計算
            weighted_mean = np.average(distance_data['torque_x'], weights=distance_data['sample_count'])
            
            # 統計的な分散を計算
            # 方法1: 各データポイントの分散を重み付き平均（測定誤差の分散）
            measurement_variance = np.average(distance_data['variance_torque_x'], weights=distance_data['sample_count'])
            
            # 方法2: グループ内のtorque_x値の重み付き分散（グループ間の分散）
            group_variance = np.average((distance_data['torque_x'] - weighted_mean) ** 2, weights=distance_data['sample_count'])
            
            # 方法3: 正規分布を仮定した重み付き平均の分散（新しい方法）
            normal_dist_variance, effective_sample_size = calculate_normal_distribution_variance(
                distance_data['sample_count'].values, 
                distance_data['variance_torque_x'].values
            )
            
            # 方法4: 複数の正規分布を合成した分散（サンプルサイズ無視）
            combined_normal_variance = calculate_combined_normal_variance(
                distance_data['torque_x'].values,
                distance_data['variance_torque_x'].values
            )
            
            # 総分散 = 測定誤差の分散 + グループ間の分散
            total_variance = measurement_variance + group_variance
            
            # サンプル数の合計
            total_samples = distance_data['sample_count'].sum()
            
            results.append({
                'tilt_angle': tilt_angle,
                'wall_spacing': wall_spacing,
                'keyword': keyword,
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
    
    # 差のプロットデータを計算
    difference_df = None
    true_wall_effect_df = None
    bias_df = None
    averaged_wall_effect_df = None
    
    if args.show_difference:
        difference_df = calculate_difference_plot_data(results_df)
        if len(difference_df) > 0:
            print(f"差のプロットデータ: {len(difference_df)} ポイント")
        else:
            print("警告: frontとback_reversedのペアが見つかりませんでした。")
    
    if args.show_corrected_difference:
        true_wall_effect_df, bias_df, averaged_wall_effect_df = calculate_averaged_difference_plot_data(results_df)
        if len(true_wall_effect_df) > 0:
            print(f"真の壁効果データ: {len(true_wall_effect_df)} ポイント")
            print(f"バイアス推定データ: {len(bias_df)} グループ")
            print(f"平均化された壁効果データ: {len(averaged_wall_effect_df)} グループ")
        else:
            print("警告: frontとback_reversedのペアが見つかりませんでした。")
    
    # プロットの作成
    if args.show_difference and args.show_corrected_difference and args.show_bias and len(difference_df) > 0 and len(true_wall_effect_df) > 0 and bias_df is not None:
        # 4つのサブプロットを作成
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 16))
        
        # 元のプロット（ax1）
        plot_original_data(results_df, ax1)
        
        # 差のプロット（ax2）
        plot_difference_data(difference_df, ax2)
        
        # 真の壁効果のプロット（ax3）
        plot_true_wall_effect_data(true_wall_effect_df, ax3)
        
        # バイアスのプロット（ax4）
        plot_bias_data(bias_df, ax4)
        
        plt.tight_layout()
    elif args.show_difference and args.show_corrected_difference and len(difference_df) > 0 and len(true_wall_effect_df) > 0:
        # 3つのサブプロットを作成
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))
        
        # 元のプロット（ax1）
        plot_original_data(results_df, ax1)
        
        # 差のプロット（ax2）
        plot_difference_data(difference_df, ax2)
        
        # 真の壁効果のプロット（ax3）
        plot_true_wall_effect_data(true_wall_effect_df, ax3)
        
        plt.tight_layout()
    elif args.show_difference and args.show_bias and len(difference_df) > 0 and bias_df is not None:
        # 3つのサブプロットを作成
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))
        
        # 元のプロット（ax1）
        plot_original_data(results_df, ax1)
        
        # 差のプロット（ax2）
        plot_difference_data(difference_df, ax2)
        
        # バイアスのプロット（ax3）
        plot_bias_data(bias_df, ax3)
        
        plt.tight_layout()
    elif args.show_corrected_difference and args.show_bias and len(true_wall_effect_df) > 0 and bias_df is not None:
        # 3つのサブプロットを作成
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))
        
        # 元のプロット（ax1）
        plot_original_data(results_df, ax1)
        
        # 真の壁効果のプロット（ax2）
        plot_true_wall_effect_data(true_wall_effect_df, ax2)
        
        # バイアスのプロット（ax3）
        plot_bias_data(bias_df, ax3)
        
        plt.tight_layout()
    elif args.show_difference and len(difference_df) > 0:
        # 2つのサブプロットを作成
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
        
        # 元のプロット（ax1）
        plot_original_data(results_df, ax1)
        
        # 差のプロット（ax2）
        plot_difference_data(difference_df, ax2)
        
        plt.tight_layout()
    elif args.show_corrected_difference and len(true_wall_effect_df) > 0:
        # 2つのサブプロットを作成
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
        
        # 元のプロット（ax1）
        plot_original_data(results_df, ax1)
        
        # 真の壁効果のプロット（ax2）
        plot_true_wall_effect_data(true_wall_effect_df, ax2)
        
        plt.tight_layout()
    elif args.show_bias and bias_df is not None:
        # 2つのサブプロットを作成
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
        
        # 元のプロット（ax1）
        plot_original_data(results_df, ax1)
        
        # バイアスのプロット（ax2）
        plot_bias_data(bias_df, ax2)
        
        plt.tight_layout()
    else:
        # 元のプロットのみ
        plt.figure(figsize=(12, 8))
        plot_original_data(results_df, plt.gca())
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
    print(f"Tilt Angleの種類: {len(results_df['tilt_angle'].unique())}")
    print(f"Wall Distanceの種類: {len(results_df['wall_spacing'].unique())}")
    print(f"Keywordの種類: {len(results_df['keyword'].unique())}")
    
    # (Tilt Angle, Wall Distance, Keyword)のペア数を計算
    unique_pairs = results_df[['tilt_angle', 'wall_spacing', 'keyword']].drop_duplicates()
    print(f"(Tilt Angle, Wall Distance, Keyword)のペア数: {len(unique_pairs)}")
    
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
    
    if args.show_difference and len(difference_df) > 0:
        print(f"\n=== 差の統計情報 ===")
        print(f"差のデータポイント数: {len(difference_df)}")
        print(f"差の範囲: {difference_df['torque_diff'].min():.3f} - {difference_df['torque_diff'].max():.3f}")
        print(f"差の分散の範囲: {difference_df['torque_diff_variance'].min():.6f} - {difference_df['torque_diff_variance'].max():.6f}")
    
    if args.show_corrected_difference and len(true_wall_effect_df) > 0:
        print(f"\n=== 真の壁効果の統計情報 ===")
        print(f"真の壁効果データポイント数: {len(true_wall_effect_df)}")
        print(f"真の壁効果の範囲: {true_wall_effect_df['true_wall_effect'].min():.3f} - {true_wall_effect_df['true_wall_effect'].max():.3f}")
        print(f"真の壁効果分散の範囲: {true_wall_effect_df['true_wall_effect_variance'].min():.6f} - {true_wall_effect_df['true_wall_effect_variance'].max():.6f}")
        
        if bias_df is not None:
            print(f"\n=== バイアス推定情報 ===")
            print(f"バイアス推定グループ数: {len(bias_df)}")
            print(f"推定バイアスの範囲: {bias_df['estimated_bias'].min():.3f} - {bias_df['estimated_bias'].max():.3f}")
            print(f"バイアス推定分散の範囲: {bias_df['bias_variance'].min():.6f} - {bias_df['bias_variance'].max():.6f}")
            print(f"測定数範囲: {bias_df['num_measurements'].min():.0f} - {bias_df['num_measurements'].max():.0f}")
        
        if averaged_wall_effect_df is not None:
            print(f"\n=== 平均化された壁効果情報 ===")
            print(f"平均化された壁効果グループ数: {len(averaged_wall_effect_df)}")
            print(f"平均化された壁効果の範囲: {averaged_wall_effect_df['averaged_wall_effect'].min():.3f} - {averaged_wall_effect_df['averaged_wall_effect'].max():.3f}")
            print(f"平均化された壁効果分散の範囲: {averaged_wall_effect_df['averaged_wall_effect_variance'].min():.6f} - {averaged_wall_effect_df['averaged_wall_effect_variance'].max():.6f}")
    
    if args.show_bias and bias_df is not None:
        print(f"\n=== バイアス推定情報 ===")
        print(f"バイアス推定グループ数: {len(bias_df)}")
        print(f"推定バイアスの範囲: {bias_df['estimated_bias'].min():.3f} - {bias_df['estimated_bias'].max():.3f}")
        print(f"バイアス推定分散の範囲: {bias_df['bias_variance'].min():.6f} - {bias_df['bias_variance'].max():.6f}")
        print(f"測定数範囲: {bias_df['num_measurements'].min():.0f} - {bias_df['num_measurements'].max():.0f}")
        
        # バイアスの統計情報
        print(f"バイアス平均: {bias_df['estimated_bias'].mean():.3f}")
        print(f"バイアス標準偏差: {bias_df['estimated_bias'].std():.3f}")
        print(f"バイアス変動係数: {(bias_df['estimated_bias'].std() / abs(bias_df['estimated_bias'].mean()) * 100):.1f}%")
    
    return 0

def plot_original_data(results_df, ax):
    """元のデータをプロットする関数"""
    # tilt_angleごとに色を割り当て、wall_spacingごとに線の種類を割り当て
    unique_tilt_angles = sorted(results_df['tilt_angle'].unique())
    unique_wall_spacings = sorted(results_df['wall_spacing'].unique())
    unique_keywords = sorted(results_df['keyword'].unique())
    
    # tilt_angleごとに色を割り当て
    colors = plt.cm.viridis(np.linspace(0, 0.8, len(unique_tilt_angles)))
    color_map = {angle: colors[i] for i, angle in enumerate(unique_tilt_angles)}
    
    # wall_spacingごとに線の種類を割り当て
    linestyles = ['-', '--', '-.', ':']
    linestyle_map = {spacing: linestyles[i % len(linestyles)] for i, spacing in enumerate(unique_wall_spacings)}
    
    # マーカーもwall_spacingごとに割り当て
    markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
    marker_map = {spacing: markers[i % len(markers)] for i, spacing in enumerate(unique_wall_spacings)}
    
    for tilt_angle in unique_tilt_angles:
        for wall_spacing in unique_wall_spacings:
            for keyword in unique_keywords:
                pair_data = results_df[(results_df['tilt_angle'] == tilt_angle) & 
                                      (results_df['wall_spacing'] == wall_spacing) &
                                      (results_df['keyword'] == keyword)].sort_values('distance')
                
                if len(pair_data) > 0:
                    # 線とマーカーでプロット
                    ax.plot(pair_data['distance'], -pair_data['torque_x_mean'], 
                            marker=marker_map[wall_spacing], 
                            color=color_map[tilt_angle], 
                            linestyle=linestyle_map[wall_spacing],
                            linewidth=2, 
                            markersize=8,
                            label=f'Tilt: {tilt_angle}°, Wall: {wall_spacing}, {keyword}')
    
    # グラフの設定
    ax.set_xlabel('壁との距離 [R]', fontsize=18)
    ax.set_ylabel('壁効果モーメント [Nm]', fontsize=18)
    ax.legend(loc='upper right', fontsize=14)
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_ylim(-0.1, 0.25) 
    
    # 軸の目盛りラベルのフォントサイズも設定
    ax.tick_params(axis='both', which='major', labelsize=18)
    ax.set_title('元のデータ', fontsize=20)

def plot_difference_data(difference_df, ax):
    """差のデータをプロットする関数"""
    # tilt_angleごとに色を割り当て、wall_spacingごとに線の種類を割り当て
    unique_tilt_angles = sorted(difference_df['tilt_angle'].unique())
    unique_wall_spacings = sorted(difference_df['wall_spacing'].unique())
    
    # tilt_angleごとに色を割り当て
    colors = plt.cm.viridis(np.linspace(0, 0.8, len(unique_tilt_angles)))
    color_map = {angle: colors[i] for i, angle in enumerate(unique_tilt_angles)}
    
    # wall_spacingごとに線の種類を割り当て
    linestyles = ['-', '--', '-.', ':']
    linestyle_map = {spacing: linestyles[i % len(linestyles)] for i, spacing in enumerate(unique_wall_spacings)}
    
    # マーカーもwall_spacingごとに割り当て
    markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
    marker_map = {spacing: markers[i % len(markers)] for i, spacing in enumerate(unique_wall_spacings)}
    
    for tilt_angle in unique_tilt_angles:
        for wall_spacing in unique_wall_spacings:
            pair_data = difference_df[(difference_df['tilt_angle'] == tilt_angle) & 
                                     (difference_df['wall_spacing'] == wall_spacing)].sort_values('distance')
            
            if len(pair_data) > 0:
                # 線とマーカーでプロット
                ax.plot(pair_data['distance'], -pair_data['torque_diff'], 
                        marker=marker_map[wall_spacing], 
                        color=color_map[tilt_angle], 
                        linestyle=linestyle_map[wall_spacing],
                        linewidth=2, 
                        markersize=8,
                        label=f'Tilt: {tilt_angle}°, Wall: {wall_spacing}')
    
    # グラフの設定
    ax.set_xlabel('壁との距離 [R]', fontsize=18)
    ax.set_ylabel('壁効果モーメント差 [Nm]', fontsize=18)
    ax.legend(loc='upper right', fontsize=14)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # 軸の目盛りラベルのフォントサイズも設定
    ax.tick_params(axis='both', which='major', labelsize=18)
    ax.set_title('Front - Back_Reversed の差', fontsize=20)

def plot_true_wall_effect_data(true_wall_effect_df, ax):
    """真の壁効果のデータをプロットする関数"""
    # tilt_angleごとに色を割り当て、wall_spacingごとに線の種類を割り当て
    unique_tilt_angles = sorted(true_wall_effect_df['tilt_angle'].unique())
    unique_wall_spacings = sorted(true_wall_effect_df['wall_spacing'].unique())
    
    # tilt_angleごとに色を割り当て
    colors = plt.cm.viridis(np.linspace(0, 0.8, len(unique_tilt_angles)))
    color_map = {angle: colors[i] for i, angle in enumerate(unique_tilt_angles)}
    
    # wall_spacingごとに線の種類を割り当て
    linestyles = ['-', '--', '-.', ':']
    linestyle_map = {spacing: linestyles[i % len(linestyles)] for i, spacing in enumerate(unique_wall_spacings)}
    
    # マーカーもwall_spacingごとに割り当て
    markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
    marker_map = {spacing: markers[i % len(markers)] for i, spacing in enumerate(unique_wall_spacings)}
    
    for tilt_angle in unique_tilt_angles:
        for wall_spacing in unique_wall_spacings:
            pair_data = true_wall_effect_df[(true_wall_effect_df['tilt_angle'] == tilt_angle) & 
                                           (true_wall_effect_df['wall_spacing'] == wall_spacing)].sort_values('distance')
            
            if len(pair_data) > 0:
                # 線とマーカーでプロット
                ax.plot(pair_data['distance'], -pair_data['true_wall_effect'], 
                        marker=marker_map[wall_spacing], 
                        color=color_map[tilt_angle], 
                        linestyle=linestyle_map[wall_spacing],
                        linewidth=2, 
                        markersize=8,
                        label=f'Tilt: {tilt_angle}°, Wall: {wall_spacing}')
    
    # グラフの設定
    ax.set_xlabel('壁との距離 [R]', fontsize=18)
    ax.set_ylabel('真の壁効果モーメント [Nm]', fontsize=18)
    ax.legend(loc='upper right', fontsize=14)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # 軸の目盛りラベルのフォントサイズも設定
    ax.tick_params(axis='both', which='major', labelsize=18)
    ax.set_title('真の壁効果（バイアス除去後）', fontsize=20)

def plot_bias_data(bias_df, ax):
    """バイアスのデータをプロットする関数"""
    # tilt_angleごとに色を割り当て、wall_spacingごとに線の種類を割り当て
    unique_tilt_angles = sorted(bias_df['tilt_angle'].unique())
    unique_wall_spacings = sorted(bias_df['wall_spacing'].unique())
    
    # tilt_angleごとに色を割り当て
    colors = plt.cm.viridis(np.linspace(0, 0.8, len(unique_tilt_angles)))
    color_map = {angle: colors[i] for i, angle in enumerate(unique_tilt_angles)}
    
    # wall_spacingごとに線の種類を割り当て
    linestyles = ['-', '--', '-.', ':']
    linestyle_map = {spacing: linestyles[i % len(linestyles)] for i, spacing in enumerate(unique_wall_spacings)}
    
    # マーカーもwall_spacingごとに割り当て
    markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
    marker_map = {spacing: markers[i % len(markers)] for i, spacing in enumerate(unique_wall_spacings)}
    
    for tilt_angle in unique_tilt_angles:
        for wall_spacing in unique_wall_spacings:
            pair_data = bias_df[(bias_df['tilt_angle'] == tilt_angle) & 
                                (bias_df['wall_spacing'] == wall_spacing)].sort_values('tilt_angle') # tilt_angleでソート
            
            if len(pair_data) > 0:
                # 線とマーカーでプロット
                ax.plot(pair_data['tilt_angle'], -pair_data['estimated_bias'], 
                        marker=marker_map[wall_spacing], 
                        color=color_map[tilt_angle], 
                        linestyle=linestyle_map[wall_spacing],
                        linewidth=2, 
                        markersize=8,
                        label=f'Tilt: {tilt_angle}°, Wall: {wall_spacing}')
    
    # グラフの設定
    ax.set_xlabel('Tilt Angle [°]', fontsize=18)
    ax.set_ylabel('推定バイアス [Nm]', fontsize=18)
    ax.legend(loc='upper right', fontsize=14)
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # 軸の目盛りラベルのフォントサイズも設定
    ax.tick_params(axis='both', which='major', labelsize=18)
    ax.set_title('推定バイアス', fontsize=20)

if __name__ == "__main__":
    exit(main())
