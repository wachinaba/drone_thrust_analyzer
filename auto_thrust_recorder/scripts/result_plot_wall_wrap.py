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

def linear_interpolate(x_values, y_values, x_target):
    """
    線形補間を行う関数
    x_values: 既知のx座標
    y_values: 既知のy座標
    x_target: 補間したいx座標
    """
    if len(x_values) == 0:
        return None
    
    # 補外の場合：一番外側の点の値を使用
    if x_target <= x_values.min():
        return y_values[x_values.argmin()]
    elif x_target >= x_values.max():
        return y_values[x_values.argmax()]
    
    # 線形補間
    for i in range(len(x_values) - 1):
        if x_values[i] <= x_target <= x_values[i + 1]:
            x1, x2 = x_values[i], x_values[i + 1]
            y1, y2 = y_values[i], y_values[i + 1]
            return y1 + (y2 - y1) * (x_target - x1) / (x2 - x1)
    
    return None

def create_mirrored_data(results_df, origin_distance, origin_torque_x):
    """
    単一壁のデータをX,Y軸反転して重ね合わせる関数
    """
    mirrored_results = []
    
    # 各tilt_angleについて処理
    for tilt_angle in results_df['tilt_angle'].unique():
        tilt_data = results_df[results_df['tilt_angle'] == tilt_angle].sort_values('distance')
        
        if len(tilt_data) == 0:
            continue
        
        # 元のデータの距離とトルク（origin_distanceを基準点としてオフセット）
        original_distances = tilt_data['distance'].values - origin_distance
        original_torques = tilt_data['torque_x_mean'].values
        original_variances = tilt_data['torque_x_variance'].values
        
        # 新しい距離範囲を生成（-origin_distance ~ origin_distance）
        new_distances = np.linspace(-origin_distance, origin_distance, 100)
        
        for new_dist in new_distances:
            # 右側の壁からの寄与（元のデータを補間）
            right_torque = linear_interpolate(original_distances, original_torques, new_dist)
            right_variance = linear_interpolate(original_distances, original_variances, new_dist)
            
            # 左側の壁からの寄与（X,Y軸反転）
            left_dist = -new_dist  # X軸反転
            left_torque = -linear_interpolate(original_distances, original_torques, left_dist)  # Y軸反転（torque_xも反転）
            left_variance = linear_interpolate(original_distances, original_variances, left_dist)
            
            if right_torque is not None and left_torque is not None:
                # 両側の寄与を重ね合わせ
                combined_torque = right_torque + left_torque - origin_torque_x
                # 分散は加算（独立な確率変数の和の分散）
                combined_variance = right_variance + left_variance
                
                mirrored_results.append({
                    'tilt_angle': tilt_angle,
                    'distance': new_dist,
                    'torque_x_mean': combined_torque,
                    'torque_x_variance': combined_variance,
                    'right_torque': right_torque,
                    'left_torque': left_torque,
                    'total_samples': 1  # 補間データなので1に設定
                })
    
    return pd.DataFrame(mirrored_results)

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='CSVファイルからtorque_x vs distanceのプロットを作成')
    parser.add_argument('csv_file', help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は表示のみ）')
    parser.add_argument('--dpi', type=int, default=300, help='画像のDPI（デフォルト: 300）')
    parser.add_argument('--origin_distance', type=float, help='重ね合わせの基準となる距離（指定すると両側壁シミュレーションを実行）')
    parser.add_argument('--origin_torque_x', type=float, default=0.0, help='トルクの基準点（デフォルト: 0.0）')
    
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
            'total_samples': total_samples
        })
    
    # 結果をDataFrameに変換
    results_df = pd.DataFrame(results)
    
    # 両側壁シミュレーションが指定された場合
    if args.origin_distance is not None:
        print(f"両側壁シミュレーションを実行します（origin_distance: {args.origin_distance}, origin_torque_x: {args.origin_torque_x}）")
        
        # 元のデータを保存
        original_results_df = results_df.copy()
        
        # 重ね合わせデータを作成
        mirrored_results_df = create_mirrored_data(results_df, args.origin_distance, args.origin_torque_x)
        print(f"重ね合わせ後のデータポイント数: {len(mirrored_results_df)}")
        
        # 3つのサブプロットを作成
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        
        # 各tilt_angleで異なる色とマーカーを使用
        unique_tilt_angles = sorted(results_df['tilt_angle'].unique())
        colors = plt.cm.tab10(np.linspace(0, 1, len(unique_tilt_angles)))
        markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
        
        # プロット1: 反転なし（元のデータ）
        for i, tilt_angle in enumerate(unique_tilt_angles):
            tilt_data = original_results_df[original_results_df['tilt_angle'] == tilt_angle].sort_values('distance')
            
            if len(tilt_data) > 0:
                # origin_distanceを基準点としてオフセット
                offset_distances = tilt_data['distance'].values - args.origin_distance
                
                axes[0].plot(offset_distances, tilt_data['torque_x_mean'], 
                           marker=markers[i % len(markers)], 
                           color=colors[i], 
                           linewidth=2, 
                           markersize=8,
                           label=f'Tilt Angle: {tilt_angle}°')
        
        axes[0].set_xlabel('Distance (m)', fontsize=12)
        axes[0].set_ylabel('Torque X (Nm)', fontsize=12)
        axes[0].set_title('Original Data (No Mirror)', fontsize=14)
        axes[0].legend()
        axes[0].grid(True, linestyle='--', alpha=0.7)
        axes[0].set_ylim(-0.5, 0.5)
        axes[0].set_xlim(-args.origin_distance, args.origin_distance)
        
        # プロット2: 反転あり（左側壁の寄与）
        for i, tilt_angle in enumerate(unique_tilt_angles):
            tilt_data = original_results_df[original_results_df['tilt_angle'] == tilt_angle].sort_values('distance')
            
            if len(tilt_data) > 0:
                # origin_distanceを基準点としてオフセットしてからX軸反転
                offset_distances = tilt_data['distance'].values - args.origin_distance
                mirrored_distances = -offset_distances  # X軸反転
                mirrored_torques = -tilt_data['torque_x_mean'].values  # Y軸反転（torque_xも反転）
                
                # 距離でソート
                sort_idx = np.argsort(mirrored_distances)
                mirrored_distances = mirrored_distances[sort_idx]
                mirrored_torques = mirrored_torques[sort_idx]
                
                axes[1].plot(mirrored_distances, mirrored_torques, 
                           marker=markers[i % len(markers)], 
                           color=colors[i], 
                           linewidth=2, 
                           markersize=8,
                           label=f'Tilt Angle: {tilt_angle}°')
        
        axes[1].set_xlabel('Distance (m)', fontsize=12)
        axes[1].set_ylabel('Torque X (Nm)', fontsize=12)
        axes[1].set_title('Mirrored Data (Left Wall)', fontsize=14)
        axes[1].legend()
        axes[1].grid(True, linestyle='--', alpha=0.7)
        axes[1].set_ylim(-0.5, 0.5)
        axes[1].set_xlim(-args.origin_distance, args.origin_distance)
        
        # プロット3: 重ね合わせ
        for i, tilt_angle in enumerate(unique_tilt_angles):
            tilt_data = mirrored_results_df[mirrored_results_df['tilt_angle'] == tilt_angle].sort_values('distance')
            
            if len(tilt_data) > 0:
                axes[2].plot(tilt_data['distance'], tilt_data['torque_x_mean'], 
                           marker=markers[i % len(markers)], 
                           color=colors[i], 
                           linewidth=2, 
                           markersize=8,
                           label=f'Tilt Angle: {tilt_angle}°')
        
        axes[2].set_xlabel('Distance (m)', fontsize=12)
        axes[2].set_ylabel('Torque X (Nm)', fontsize=12)
        axes[2].set_title(f'Combined Data (Dual Wall)', fontsize=14)
        axes[2].legend()
        axes[2].grid(True, linestyle='--', alpha=0.7)
        axes[2].set_ylim(-0.5, 0.5)
        axes[2].set_xlim(-args.origin_distance, args.origin_distance)
        
        # 全体のタイトル
        fig.suptitle(f'Dual Wall Simulation (Origin: {args.origin_distance}m)', fontsize=16)
        
        # レイアウトの調整
        plt.tight_layout()
        
        # 結果を更新
        results_df = mirrored_results_df
        
    else:
        # 通常のプロット（エラーバーなし）
        plt.figure(figsize=(12, 8))
        
        # 各tilt_angleで異なる色とマーカーを使用
        unique_tilt_angles = sorted(results_df['tilt_angle'].unique())
        colors = plt.cm.tab10(np.linspace(0, 1, len(unique_tilt_angles)))
        markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
        
        for i, tilt_angle in enumerate(unique_tilt_angles):
            tilt_data = results_df[results_df['tilt_angle'] == tilt_angle].sort_values('distance')
            
            if len(tilt_data) > 0:
                # 線とマーカーでプロット（エラーバーなし）
                plt.plot(tilt_data['distance'], tilt_data['torque_x_mean'], 
                        marker=markers[i % len(markers)], 
                        color=colors[i], 
                        linewidth=2, 
                        markersize=8,
                        label=f'Tilt Angle: {tilt_angle}°')
        
        # グラフの設定
        plt.xlabel('Distance (m)', fontsize=12)
        plt.ylabel('Torque X (Nm)', fontsize=12)
        plt.title('Torque X vs Distance (Grouped by Tilt Angle)', fontsize=14)
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.ylim(-0.5, 0.5)
        
        # レイアウトの調整
        plt.tight_layout()
    
    # 統計情報の表示
    print("\n=== 統計情報 ===")
    print(f"データポイント数: {len(results_df)}")
    print(f"Tilt Angleの種類: {len(unique_tilt_angles)}")
    
    if args.origin_distance is not None:
        print(f"両側壁シミュレーション: 有効")
        print(f"基準距離: {args.origin_distance}m")
        print(f"基準トルク: {args.origin_torque_x}Nm")
        print(f"プロット距離範囲: -{args.origin_distance} ~ {args.origin_distance}m")
    else:
        print(f"Distanceの範囲: {results_df['distance'].min():.3f} - {results_df['distance'].max():.3f}")
    
    print(f"Torque Xの範囲: {results_df['torque_x_mean'].min():.3f} - {results_df['torque_x_mean'].max():.3f}")
    
    if 'measurement_variance' in results_df.columns:
        print(f"総分散の範囲: {results_df['torque_x_variance'].min():.6f} - {results_df['torque_x_variance'].max():.6f}")
        print(f"測定誤差分散の範囲: {results_df['measurement_variance'].min():.6f} - {results_df['measurement_variance'].max():.6f}")
        print(f"グループ間分散の範囲: {results_df['group_variance'].min():.6f} - {results_df['group_variance'].max():.6f}")
    else:
        print(f"分散の範囲: {results_df['torque_x_variance'].min():.6f} - {results_df['torque_x_variance'].max():.6f}")
    
    # 出力処理
    if args.output:
        plt.savefig(args.output, dpi=args.dpi, bbox_inches='tight')
        print(f"プロットを '{args.output}' に保存しました。")
    else:
        plt.show()
    
    return 0

if __name__ == "__main__":
    exit(main())
