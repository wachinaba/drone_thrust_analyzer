#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import japanize_matplotlib
from matplotlib.colors import LinearSegmentedColormap

def create_custom_colormap():
    """
    マイナスが青、中心が黒、プラスが赤になるカスタムカラーマップを作成する関数
    
    Returns:
        custom_cmap: カスタムカラーマップ
    """
    # 色の定義（青→黒→赤）
    colors = ['#FF0000', '#BB7700', '#202020', '#3399EB', '#0000FF']
    n_bins = 256  # 色の段階数
    
    # LinearSegmentedColormapを作成
    custom_cmap = LinearSegmentedColormap.from_list('custom_blue_black_red', colors, N=n_bins)
    
    return custom_cmap

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

def create_heatmap_data(group_df):
    """
    グループデータからヒートマップ用のデータを作成する関数
    
    Args:
        group_df: グループ化されたデータフレーム
    
    Returns:
        heatmap_matrix: ヒートマップ用の行列
        distance_values: 距離の値のリスト
        target_thrust_values: target_thrustの値のリスト
    """
    # 距離とtarget_thrustの一意な値を取得
    distance_values = sorted(group_df['distance'].unique())
    target_thrust_values = sorted(group_df['target_thrust'].unique())
    
    # ヒートマップ用の行列を作成
    heatmap_matrix = np.full((len(target_thrust_values), len(distance_values)), np.nan)
    
    # 各target_thrustごとに平滑化と平行移動を計算
    for i, target_thrust in enumerate(target_thrust_values):
        # 特定のtarget_thrustのデータを抽出
        target_thrust_data = group_df[group_df['target_thrust'] == target_thrust]
        
        if len(target_thrust_data) > 0:
            # 各distance値に対してtorque_xの中央値を計算
            distance_torque_pairs = []
            for distance in distance_values:
                # 特定のtarget_thrustとdistanceの組み合わせのデータを抽出
                mask = (target_thrust_data['distance'] == distance)
                subset = target_thrust_data[mask]
                
                if len(subset) > 0:
                    # 中央値を計算（平行移動前）
                    torque_x_median = subset['torque_x'].median()
                    distance_torque_pairs.append((distance, torque_x_median))
            
            # 距離でソート
            distance_torque_pairs.sort(key=lambda x: x[0])
            
            if len(distance_torque_pairs) >= 3:
                # 距離とtorque_x値を分離
                sorted_distances = [pair[0] for pair in distance_torque_pairs]
                sorted_torques = [pair[1] for pair in distance_torque_pairs]
                
                # 平滑化を適用
                smoothed_distances, smoothed_torques = apply_moving_average(
                    np.array(sorted_distances), np.array(sorted_torques), window_size=5
                )
                
                # 平滑化後のデータで平行移動を計算
                if len(smoothed_distances) > 0:
                    # 最もdistanceが大きい点（平滑化後）を基準に平行移動
                    max_distance_idx = np.argmax(smoothed_distances)
                    offset = smoothed_torques[max_distance_idx]
                    
                    # 平行移動を適用
                    final_torques = smoothed_torques - offset
                    
                    # ヒートマップ行列に格納
                    for j, (distance, torque) in enumerate(zip(smoothed_distances, final_torques)):
                        distance_idx = distance_values.index(distance)
                        heatmap_matrix[i, distance_idx] = torque
    
    return heatmap_matrix, distance_values, target_thrust_values

def calculate_derivative(x_values, y_values):
    """
    数値微分を計算する関数
    
    Args:
        x_values: x座標の値（距離）
        y_values: y座標の値（torque_x）
    
    Returns:
        derivative_values: 微分値
    """
    if len(x_values) < 2:
        return np.array([])
    
    # 中央差分法で微分を計算
    derivative_values = np.zeros_like(y_values)
    
    for i in range(len(x_values)):
        if i == 0:
            # 前進差分
            derivative_values[i] = (y_values[1] - y_values[0]) / (x_values[1] - x_values[0])
        elif i == len(x_values) - 1:
            # 後退差分
            derivative_values[i] = (y_values[i] - y_values[i-1]) / (x_values[i] - x_values[i-1])
        else:
            # 中央差分
            derivative_values[i] = (y_values[i+1] - y_values[i-1]) / (x_values[i+1] - x_values[i-1])
    
    return derivative_values

def calculate_drone_body_distance(rotor_distance, drone_offset):
    """
    ロータと壁の距離からドローン本体と壁の距離を計算する関数
    
    Args:
        rotor_distance: ロータ端と壁の距離 [R]
        drone_offset: ロータ中心からドローン本体中心までの距離 [m]
    
    Returns:
        drone_body_distance: ドローン本体と壁の距離 [m]
    """
    return (rotor_distance + 1.0) * 0.0254 * 3.5 + drone_offset

def calculate_normalized_moment(torque_x, arm_length):
    """
    モーメントをアーム長で割ってNormalized Momentを計算する関数
    
    Args:
        torque_x: 壁効果モーメント [Nm]
        arm_length: ドローン中心からロータ中心までの距離 [m]
    
    Returns:
        normalized_moment: 正規化されたモーメント [N]（力の単位）
    """
    return torque_x / arm_length

def apply_moving_average(x_values, y_values, window_size=5):
    """
    移動平均を適用する関数
    
    Args:
        x_values: x座標の値
        y_values: y座標の値
        window_size: 窓の大きさ（デフォルト: 5）
    
    Returns:
        smoothed_x: 平滑化されたx座標
        smoothed_y: 平滑化されたy座標
    """
    if len(x_values) < window_size:
        return x_values, y_values
    
    # 移動平均を計算
    smoothed_y = np.convolve(y_values, np.ones(window_size)/window_size, mode='valid')
    
    # x座標も対応するように調整
    # 窓の中心に対応するx座標を計算
    half_window = window_size // 2
    smoothed_x = x_values[half_window:len(x_values) - half_window]
    
    # 窓サイズが奇数の場合の調整
    if window_size % 2 == 0:
        smoothed_x = x_values[half_window-1:len(x_values) - half_window]
    
    return smoothed_x, smoothed_y

def apply_kernel_smoothing(x_values, y_values, bandwidth=None, kernel='gaussian', sample_counts=None):
    """
    カーネル平滑化を適用する関数
    
    Args:
        x_values: x座標の値
        y_values: y座標の値
        bandwidth: バンド幅（Noneの場合は自動計算）
        kernel: カーネルタイプ ('gaussian', 'epanechnikov', 'uniform')
        sample_counts: 各点のサンプル数（Noneの場合は等重み）
    
    Returns:
        smoothed_x: 平滑化されたx座標
        smoothed_y: 平滑化されたy座標
    """
    if len(x_values) < 3:
        return x_values, y_values
    
    # バンド幅が指定されていない場合は自動計算
    if bandwidth is None:
        # 有効サンプルサイズを計算
        if sample_counts is not None and len(sample_counts) == len(x_values):
            # 重み付き有効サンプルサイズ
            total_samples = np.sum(sample_counts)
            # effective_n = (total_samples ** 2) / np.sum(sample_counts ** 2)
            effective_n = total_samples
        else:
            # 等重みの場合
            effective_n = len(x_values)

        print(f"effective_n: {effective_n}")
        
        # Silverman's rule of thumb（有効サンプルサイズを使用）
        bandwidth = 0.9 * min(np.std(x_values), np.percentile(np.abs(x_values - np.median(x_values)), 75) / 1.34) * effective_n ** (-1/5)
        bandwidth = max(bandwidth, (x_values.max() - x_values.min()) / 20)  # 最小値の制限
    
    # カーネル関数を定義
    def gaussian_kernel(u):
        return np.exp(-0.5 * u**2) / np.sqrt(2 * np.pi)
    
    def epanechnikov_kernel(u):
        return np.where(np.abs(u) <= 1, 0.75 * (1 - u**2), 0)
    
    def uniform_kernel(u):
        return np.where(np.abs(u) <= 1, 0.5, 0)
    
    # カーネル関数を選択
    if kernel == 'gaussian':
        kernel_func = gaussian_kernel
    elif kernel == 'epanechnikov':
        kernel_func = epanechnikov_kernel
    elif kernel == 'uniform':
        kernel_func = uniform_kernel
    else:
        kernel_func = gaussian_kernel
    
    # カーネル平滑化を計算
    smoothed_y = np.zeros_like(y_values)
    
    for i in range(len(x_values)):
        # 各点での重みを計算
        u = (x_values - x_values[i]) / bandwidth
        weights = kernel_func(u)
        
        # サンプル数を考慮した重み付け
        if sample_counts is not None and len(sample_counts) == len(x_values):
            # カーネル重みとサンプル数の両方を考慮
            total_weights = weights * sample_counts
        else:
            total_weights = weights
        
        # 重み付き平均を計算
        if np.sum(total_weights) > 0:
            smoothed_y[i] = np.sum(total_weights * y_values) / np.sum(total_weights)
        else:
            smoothed_y[i] = y_values[i]
    
    return x_values, smoothed_y

def create_derivative_heatmap_data(group_df):
    """
    グループデータから微分ヒートマップ用のデータを作成する関数
    
    Args:
        group_df: グループ化されたデータフレーム
    
    Returns:
        derivative_matrix: 微分ヒートマップ用の行列
        distance_values: 距離の値のリスト
        target_thrust_values: target_thrustの値のリスト
    """
    # 距離とtarget_thrustの一意な値を取得
    distance_values = sorted(group_df['distance'].unique())
    target_thrust_values = sorted(group_df['target_thrust'].unique())
    
    # 微分ヒートマップ用の行列を作成
    derivative_matrix = np.full((len(target_thrust_values), len(distance_values)), np.nan)
    
    # 各target_thrustごとに平滑化、平行移動、微分を計算
    for i, target_thrust in enumerate(target_thrust_values):
        # 特定のtarget_thrustのデータを抽出
        target_thrust_data = group_df[group_df['target_thrust'] == target_thrust]
        
        if len(target_thrust_data) > 0:
            # 各distance値に対してtorque_xの中央値を計算
            distance_torque_pairs = []
            for distance in distance_values:
                # 特定のtarget_thrustとdistanceの組み合わせのデータを抽出
                mask = (target_thrust_data['distance'] == distance)
                subset = target_thrust_data[mask]
                
                if len(subset) > 0:
                    # 中央値を計算（平行移動前）
                    torque_x_median = subset['torque_x'].median()
                    distance_torque_pairs.append((distance, torque_x_median))
            
            # 距離でソート
            distance_torque_pairs.sort(key=lambda x: x[0])
            
            if len(distance_torque_pairs) >= 3:
                # 距離とtorque_x値を分離
                sorted_distances = [pair[0] for pair in distance_torque_pairs]
                sorted_torques = [pair[1] for pair in distance_torque_pairs]
                
                # 平滑化を適用
                smoothed_distances, smoothed_torques = apply_moving_average(
                    np.array(sorted_distances), np.array(sorted_torques), window_size=5
                )
                
                # 平滑化後のデータで平行移動を計算
                if len(smoothed_distances) > 0:
                    # 最もdistanceが大きい点（平滑化後）を基準に平行移動
                    max_distance_idx = np.argmax(smoothed_distances)
                    offset = smoothed_torques[max_distance_idx]
                    
                    # 平行移動を適用
                    final_torques = smoothed_torques - offset
                    
                    # 微分を計算（平行移動後）
                    if len(smoothed_distances) >= 2:
                        derivative_values = calculate_derivative(smoothed_distances, final_torques)
                        
                        # 微分値を行列に格納
                        for j, (distance, derivative) in enumerate(zip(smoothed_distances, derivative_values)):
                            distance_idx = distance_values.index(distance)
                            derivative_matrix[i, distance_idx] = derivative
    
    return derivative_matrix, distance_values, target_thrust_values

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='CSVファイルからtorque_x vs distanceのプロットを作成')
    parser.add_argument('csv_file', help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は表示のみ）')
    parser.add_argument('--dpi', type=int, default=300, help='画像のDPI（デフォルト: 300）')
    parser.add_argument('--smoothing', choices=['moving_average', 'kernel'], default='moving_average', 
                       help='平滑化方法（デフォルト: moving_average）')
    parser.add_argument('--kernel', choices=['gaussian', 'epanechnikov', 'uniform'], default='gaussian',
                       help='カーネルタイプ（smoothing=kernelの場合、デフォルト: gaussian）')
    parser.add_argument('--bandwidth', type=float, help='カーネルのバンド幅（自動計算の場合は指定不要）')
    parser.add_argument('--drone-offset', type=float, default=0.0, 
                       help='ロータからドローン本体中心までの距離 [R]（デフォルト: 0.0）')
    parser.add_argument('--arm-length', type=float, default=0.0889, 
                       help='ドローン中心からロータ中心までの距離 [m]（デフォルト: 0.0889）')
    
    args = parser.parse_args()

    print(f"drone offset: {args.drone_offset}")
    print(f"arm length: {args.arm_length}" )

    
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
    required_columns = ['torque_x', 'distance', 'tilt_angle', 'wall_spacing', 'variance_torque_x', 'sample_count', 'target_thrust']
    missing_columns = [col for col in required_columns if col not in df.columns]
    
    if missing_columns:
        print(f"エラー: 必要な列が見つかりません: {missing_columns}")
        print(f"利用可能な列: {list(df.columns)}")
        return 1
    
    # 関連する列を数値型に変換
    cols_to_numeric = ['torque_x', 'distance', 'tilt_angle', 'wall_spacing', 'variance_torque_x', 'sample_count', 'target_thrust']
    
    for col in cols_to_numeric:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # NaNを含む行を削除
    df_clean = df.dropna(subset=['torque_x', 'distance', 'tilt_angle', 'wall_spacing', 'variance_torque_x', 'sample_count', 'target_thrust'])
    
    if df_clean.empty:
        print("エラー: 有効なデータがありません。")
        return 1
    
    print(f"有効なデータ行数: {len(df_clean)}")
    
    # tilt_angleとwall_distanceでグループ化
    grouped_data = df_clean.groupby(['tilt_angle', 'wall_spacing'])
    
    if grouped_data.ngroups == 0:
        print("指定されたキーに基づくグループが見つかりませんでした。")
        return 1
    
    print(f"グループ数: {grouped_data.ngroups}")
    
    # 各グループで統計量を計算
    results = []
    
    for (tilt_angle, wall_spacing), group_df in grouped_data:
        # 最もdistanceが大きい地点の測定値を0にするよう平行移動を計算
        max_distance = group_df['distance'].max()
        max_distance_data = group_df[group_df['distance'] == max_distance]
        
        if len(max_distance_data) > 0:
            # 最大distance地点の平均値を計算
            max_distance_mean = np.average(max_distance_data['torque_x'])
            # 平行移動量を計算（最大distance地点を0にする）
            offset = max_distance_mean
        else:
            offset = 0
        
        # 各distance値に対して個別に統計量を計算
        for distance in group_df['distance'].unique():
            # 特定のdistance値のデータを抽出
            distance_data = group_df[group_df['distance'] == distance]
            
            if len(distance_data) == 0:
                continue
                
            # 平行移動を適用したtorque_x値を計算
            torque_x_adjusted = distance_data['torque_x'] - offset
            
            # 平均を計算（平行移動後）
            weighted_mean = np.average(torque_x_adjusted)

            median = np.median(torque_x_adjusted)
            
            # 統計的な分散を計算（平行移動後）
            # 方法1: 各データポイントの分散を重み付き平均（測定誤差の分散）
            measurement_variance = np.average(distance_data['variance_torque_x'])
            
            # 方法2: グループ内のtorque_x値の重み付き分散（グループ間の分散）
            group_variance = np.average((torque_x_adjusted - weighted_mean) ** 2)
            
            # 方法3: 正規分布を仮定した重み付き平均の分散（新しい方法）
            normal_dist_variance, effective_sample_size = calculate_normal_distribution_variance(
                distance_data['sample_count'].values, 
                distance_data['variance_torque_x'].values
            )
            
            # 方法4: 複数の正規分布を合成した分散（サンプルサイズ無視）
            combined_normal_variance = calculate_combined_normal_variance(
                torque_x_adjusted.values,
                distance_data['variance_torque_x'].values
            )
            
            # 総分散 = 測定誤差の分散 + グループ間の分散
            total_variance = measurement_variance + group_variance
            
            # サンプル数の合計
            total_samples = distance_data['sample_count'].sum()
            
            results.append({
                'tilt_angle': tilt_angle,
                'wall_spacing': wall_spacing,
                'distance': distance,
                'torque_x_mean': weighted_mean,
                'torque_x_median': median,
                'torque_x_variance': total_variance,
                'measurement_variance': measurement_variance,
                'group_variance': group_variance,
                'normal_dist_variance': normal_dist_variance,
                'combined_normal_variance': combined_normal_variance,
                'effective_sample_size': effective_sample_size,
                'total_samples': total_samples,
                'offset': offset  # 平行移動量を記録
            })
    
    # 結果をDataFrameに変換
    results_df = pd.DataFrame(results)
    
    # プロットの作成
    fig, ax1 = plt.subplots(figsize=(10, 4.5))
    
    # 2つ目の横軸を作成（twiny）
    ax2 = ax1.twiny()
    
    # 2つ目の縦軸を作成（twinx）
    ax3 = ax1.twinx()
    
    # 表示するチルト角度を指定
    target_tilt_angles = [0, 15, 30]
    unique_wall_spacings = sorted(results_df['wall_spacing'].unique())
    
    # 実際のデータに存在するチルト角を取得
    actual_tilt_angles = sorted(results_df['tilt_angle'].unique())

    actual_tilt_angles = [-30, -15, 0, 15, 30]
    
    # カスタムカラーマップを作成
    custom_cmap = create_custom_colormap()
    
    # 色のマップを作成（実際のデータに存在する角度に対して）
    colors = custom_cmap(np.linspace(0, 1.0, len(actual_tilt_angles)))
    color_map = {angle: colors[i] for i, angle in enumerate(actual_tilt_angles)}
    
    # wall_spacingごとに線の種類を割り当て
    linestyles = ['-', '--']
    linestyle_map = {spacing: linestyles[i % len(linestyles)] for i, spacing in enumerate(unique_wall_spacings)}
    
    # マーカーもtilt_angleごとに割り当て（実際のデータに存在する角度に対して）
    markers = ['o', 's', '^', 'v', 'D', 'p', '*', 'h', 'H', '+']
    marker_map = {angle: markers[i % len(markers)] for i, angle in enumerate(actual_tilt_angles)}
    
    # 凡例用の要素を格納するリスト
    legend_elements = []
    
    # 線の種類の凡例要素を追加
    if len(unique_wall_spacings) > 1:
        for i, wall_spacing in enumerate(unique_wall_spacings):
            legend_line = plt.Line2D([], [], color='black', 
                                    linestyle=linestyles[i], 
                                    linewidth=2, 
                                    label=f'Wall spacing: {wall_spacing}m')
            legend_elements.append(legend_line)
    
    # チルト角度の凡例要素を追加（target_tilt_anglesの順序で）
    for i, tilt_angle in enumerate(target_tilt_angles):
        if tilt_angle in color_map:  # 実際のデータに存在する角度のみ
            legend_marker = plt.Line2D([], [], color=color_map[tilt_angle], 
                                      linestyle='-', 
                                      marker=marker_map[tilt_angle], 
                                      markersize=8,
                                      label=f'Tilt: {tilt_angle}°')
            legend_elements.append(legend_marker)
    
    for tilt_angle in target_tilt_angles:
        for wall_spacing in unique_wall_spacings:

            pair_data = results_df[(results_df['tilt_angle'] == tilt_angle) & 
                                  (results_df['wall_spacing'] == wall_spacing)].sort_values('distance')
            
            if len(pair_data) > 0:
                # 平滑化を適用
                x_values = pair_data['distance'].values
                y_values = -pair_data['torque_x_mean'].values
                
                if args.smoothing == 'kernel':
                    # サンプル数を取得
                    sample_counts = pair_data['total_samples'].values if 'total_samples' in pair_data.columns else None
                    smoothed_x, smoothed_y = apply_kernel_smoothing(x_values, y_values, 
                                                                   bandwidth=args.bandwidth, 
                                                                   kernel=args.kernel,
                                                                   sample_counts=sample_counts)
                else:
                    smoothed_x, smoothed_y = apply_moving_average(x_values, y_values, window_size=5)
                
                # 平滑化後のデータで平行移動を計算
                if len(smoothed_x) > 0:
                    # 最もdistanceが大きい点（平滑化後）を基準に平行移動
                    max_distance_idx = np.argmax(smoothed_x)
                    offset = smoothed_y[max_distance_idx]
                    
                    # 平行移動を適用
                    final_y = smoothed_y - offset
                    
                    # 元のモーメントをプロット（左の縦軸）
                    line1 = ax1.plot(smoothed_x, final_y, 
                                   color=color_map[tilt_angle], 
                                   linestyle=linestyle_map[wall_spacing],
                                   linewidth=2, alpha=1.0)
                    
                    # 真の値の位置にマーカーをプロット（元のモーメント）
                    ax1.scatter(pair_data['distance'], -pair_data['torque_x_mean'] - offset, 
                               marker=marker_map[tilt_angle], 
                               color=color_map[tilt_angle],
                               s=50, alpha=0.3, zorder=5)
                    
                    """
                    # Normalized Momentを計算してプロット（右の縦軸）
                    normalized_y = calculate_normalized_moment(final_y, args.arm_length)
                    line2 = ax3.plot(smoothed_x, normalized_y, 
                                   color=color_map[tilt_angle], 
                                   linestyle=linestyle_map[wall_spacing],
                                   linewidth=2, alpha=0.7)
                    
                    # 真の値の位置にマーカーをプロット（Normalized Moment）
                    normalized_true_y = calculate_normalized_moment(-pair_data['torque_x_mean'] - offset, args.arm_length)
                    ax3.scatter(pair_data['distance'], normalized_true_y, 
                               marker=marker_map[tilt_angle], 
                               color=color_map[tilt_angle],
                               s=30, alpha=0.5, zorder=5)
                    """
            
            # エラーバー（標準偏差）を表示
            """
            std_values = np.sqrt(pair_data['combined_normal_variance'])
            plt.errorbar(pair_data['distance'], -pair_data['torque_x_mean'],
                        yerr=std_values,
                        fmt='none',
                        color=colors[i],
                        alpha=0.5,
                        capsize=5)
            """
    
    # グラフの設定
    ax1.set_xlabel('Rotor End-Wall Distance [R]', fontsize=18)
    ax1.set_ylabel('Wall Effect Moment [Nm]', fontsize=18)
    ax1.legend(handles=legend_elements, loc='upper right', fontsize=14, ncol=2)
    ax1.grid(True, linestyle='--', alpha=0.7)
    
    # 軸の目盛りラベルのフォントサイズも設定
    ax1.tick_params(axis='both', which='major', labelsize=18)
    ax1.tick_params(axis='y')
    
    # 2つ目の横軸（ドローン本体と壁の距離）の設定
    # 現在のx軸の範囲を取得
    x_min, x_max = ax1.get_xlim()
    
    # ドローン本体と壁の距離の範囲を計算
    drone_body_x_min = calculate_drone_body_distance(x_min, args.drone_offset)
    drone_body_x_max = calculate_drone_body_distance(x_max, args.drone_offset)
    
    # 2つ目の横軸の範囲を設定
    print(f"x_min: {x_min}, x_max: {x_max}")
    print(f"drone_body_x_min: {drone_body_x_min}, drone_body_x_max: {drone_body_x_max}")

    ax2.set_xlim(drone_body_x_min, drone_body_x_max)
    ax2.set_xlabel('Drone Center-Wall Distance [m]', fontsize=18)
    ax2.tick_params(axis='x', which='major', labelsize=18)

    y_min, y_max = ax1.get_ylim()

    normalized_moment_min = calculate_normalized_moment(y_min, args.arm_length)
    normalized_moment_max = calculate_normalized_moment(y_max, args.arm_length)
    print(f"y_min: {y_min}, y_max: {y_max}")
    print(f"normalized_moment_min: {normalized_moment_min}, normalized_moment_max: {normalized_moment_max}")
    ax3.set_ylim(normalized_moment_min, normalized_moment_max)
    ax3.set_ylabel('Normalized Wall Effect [N]', fontsize=18)
    ax3.tick_params(axis='y', labelsize=18)
    
    # 2つ目の横軸の目盛りを設定（適切な間隔で）
    # drone_body_ticks = np.linspace(drone_body_x_min, drone_body_x_max, 6)
    # ax2.set_xticks(drone_body_ticks)
    # ax2.set_xticklabels([f'{tick:.3f}' for tick in drone_body_ticks])

    """    
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
    """
    
    # レイアウトの調整
    plt.tight_layout()
    
    # 出力処理
    if args.output:
        plt.savefig(args.output, dpi=args.dpi, bbox_inches='tight')
        print(f"プロットを '{args.output}' に保存しました。")
    else:
        plt.show()
    
    plt.close(fig)
    
    # 微分グラフの作成
    print("\n=== 微分グラフの作成 ===")
    
    # 微分データを計算
    derivative_results = []
    
    for (tilt_angle, wall_spacing), group_df in grouped_data:
        # 各distance値に対して個別に統計量を計算
        distance_torque_pairs = []
        
        for distance in sorted(group_df['distance'].unique()):
            # 特定のdistance値のデータを抽出
            distance_data = group_df[group_df['distance'] == distance]
            
            if len(distance_data) == 0:
                continue
                
            # 元のtorque_x値の平均を計算（平行移動前）
            weighted_mean = np.average(distance_data['torque_x'])
            
            distance_torque_pairs.append((distance, weighted_mean))
        
        # 距離でソート
        distance_torque_pairs.sort(key=lambda x: x[0])
        
        if len(distance_torque_pairs) >= 3:
            # 距離とtorque_x値を分離
            sorted_distances = [pair[0] for pair in distance_torque_pairs]
            sorted_torques = [pair[1] for pair in distance_torque_pairs]
            
            # 平滑化を適用
            if args.smoothing == 'kernel':
                # サンプル数を取得
                sample_counts = [group_df[group_df['distance'] == d]['total_samples'].sum() 
                               for d in sorted_distances]
                smoothed_x, smoothed_y = apply_kernel_smoothing(np.array(sorted_distances), 
                                                              np.array(sorted_torques), 
                                                              bandwidth=args.bandwidth, 
                                                              kernel=args.kernel,
                                                              sample_counts=sample_counts)
            else:
                smoothed_x, smoothed_y = apply_moving_average(np.array(sorted_distances), 
                                                            np.array(sorted_torques), 
                                                            window_size=5)
            
            # 平滑化後のデータで平行移動を計算
            if len(smoothed_x) > 0:
                # 最もdistanceが大きい点（平滑化後）を基準に平行移動
                max_distance_idx = np.argmax(smoothed_x)
                offset = smoothed_y[max_distance_idx]
                
                # 平行移動を適用
                final_torques = smoothed_y - offset
                
                # 平行移動後のデータで微分を計算
                if len(smoothed_x) >= 2:
                    derivative_values = calculate_derivative(smoothed_x, final_torques)
                    
                    # 微分結果を保存
                    for distance, derivative in zip(smoothed_x, derivative_values):
                        derivative_results.append({
                            'tilt_angle': tilt_angle,
                            'wall_spacing': wall_spacing,
                            'distance': distance,
                            'torque_x_derivative': derivative
                        })
    
    # 微分結果をDataFrameに変換
    derivative_df = pd.DataFrame(derivative_results)
    
    if len(derivative_df) > 0:
        # 微分グラフの作成
        fig_derivative, ax1_derivative = plt.subplots(figsize=(12, 8))
        
        # 2つ目の横軸を作成（twiny）
        ax2_derivative = ax1_derivative.twiny()
        
        # 2つ目の縦軸を作成（twinx）
        ax3_derivative = ax1_derivative.twinx()
        
        # 微分グラフ用の凡例要素を格納するリスト
        derivative_legend_elements = []
        
        # 線の種類の凡例要素を追加
        for i, wall_spacing in enumerate(unique_wall_spacings):
            legend_line = plt.Line2D([], [], color='black', 
                                    linestyle=linestyles[i], 
                                    linewidth=2, 
                                    label=f'Wall: {wall_spacing}m')
            derivative_legend_elements.append(legend_line)
        
        # チルト角度の凡例要素を追加（target_tilt_anglesの順序で）
        for i, tilt_angle in enumerate(target_tilt_angles):
            if tilt_angle in color_map:  # 実際のデータに存在する角度のみ
                legend_marker = plt.Line2D([], [], color=color_map[tilt_angle], 
                                          linestyle='-', 
                                          marker=marker_map[tilt_angle], 
                                          markersize=8,
                                          label=f'Tilt: {tilt_angle}°')
                derivative_legend_elements.append(legend_marker)
        
        for tilt_angle in target_tilt_angles:
            for wall_spacing in unique_wall_spacings:
                pair_data = derivative_df[(derivative_df['tilt_angle'] == tilt_angle) & 
                                        (derivative_df['wall_spacing'] == wall_spacing)].sort_values('distance')
                
                if len(pair_data) > 0:
                    # 平滑化を適用
                    x_values = pair_data['distance'].values
                    y_values = pair_data['torque_x_derivative'].values
                    
                    if args.smoothing == 'kernel':
                        # サンプル数を取得
                        sample_counts = pair_data['total_samples'].values if 'total_samples' in pair_data.columns else None
                        smoothed_x, smoothed_y = apply_kernel_smoothing(x_values, y_values, 
                                                                       bandwidth=args.bandwidth, 
                                                                       kernel=args.kernel,
                                                                       sample_counts=sample_counts)
                    else:
                        smoothed_x, smoothed_y = apply_moving_average(x_values, y_values, window_size=5)
                    
                    # 平滑化後のデータで平行移動を計算
                    if len(smoothed_x) > 0:
                        # 最もdistanceが大きい点（平滑化後）を基準に平行移動
                        max_distance_idx = np.argmax(smoothed_x)
                        offset = smoothed_y[max_distance_idx]
                        
                        # 平行移動を適用
                        final_y = smoothed_y - offset
                        
                        # 元の微分値をプロット（左の縦軸）
                        line1 = ax1_derivative.plot(smoothed_x, final_y, 
                                       color=color_map[tilt_angle], 
                                       linestyle=linestyle_map[wall_spacing],
                                       linewidth=2, alpha=0.7)
                        
                        # 平滑化されたデータの位置にマーカーをプロット（元の微分値）
                        ax1_derivative.scatter(smoothed_x, final_y, 
                                   marker=marker_map[tilt_angle], 
                                   color=color_map[tilt_angle],
                                   s=50, alpha=0.7, zorder=5)
                        
                        # Normalized微分値を計算してプロット（右の縦軸）
                        normalized_derivative_y = calculate_normalized_moment(final_y, args.arm_length)
                        line2 = ax3_derivative.plot(smoothed_x, normalized_derivative_y, 
                                       color=color_map[tilt_angle], 
                                       linestyle=linestyle_map[wall_spacing],
                                       linewidth=2, alpha=0.7)
                        
                        # 平滑化されたデータの位置にマーカーをプロット（Normalized微分値）
                        ax3_derivative.scatter(smoothed_x, normalized_derivative_y, 
                                   marker=marker_map[tilt_angle], 
                                   color=color_map[tilt_angle],
                                   s=30, alpha=0.5, zorder=5)
        
        # グラフの設定
        ax1_derivative.set_xlabel('Rotor-Wall Distance [R]', fontsize=18)
        ax1_derivative.set_ylabel('Wall Effect Moment Derivative [Nm/R]', fontsize=18, color='blue')
        ax1_derivative.legend(handles=derivative_legend_elements, loc='upper right', fontsize=14, ncol=2)
        ax1_derivative.grid(True, linestyle='--', alpha=0.7)
        
        # 軸の目盛りラベルのフォントサイズも設定
        ax1_derivative.tick_params(axis='both', which='major', labelsize=18)
        ax1_derivative.tick_params(axis='y', labelcolor='blue')
        
        # 2つ目の縦軸（Normalized微分値）の設定
        ax3_derivative.set_ylabel('Normalized Wall Effect Derivative [N/R]', fontsize=18, color='red')
        ax3_derivative.tick_params(axis='y', labelcolor='red', labelsize=18)
        
        # 2つ目の横軸（ドローン本体と壁の距離）の設定
        # 現在のx軸の範囲を取得
        x_min, x_max = ax1_derivative.get_xlim()
        
        # ドローン本体と壁の距離の範囲を計算
        drone_body_x_min = calculate_drone_body_distance(x_min, args.drone_offset)
        drone_body_x_max = calculate_drone_body_distance(x_max, args.drone_offset)
        
        # 2つ目の横軸の範囲を設定
        ax2_derivative.set_xlim(drone_body_x_min, drone_body_x_max)
        ax2_derivative.set_xlabel('Drone Body-Wall Distance [R]', fontsize=18)
        ax2_derivative.tick_params(axis='x', which='major', labelsize=18)
        
        # 2つ目の横軸の目盛りを設定（適切な間隔で）
        drone_body_ticks = np.linspace(drone_body_x_min, drone_body_x_max, 6)
        ax2_derivative.set_xticks(drone_body_ticks)
        ax2_derivative.set_xticklabels([f'{tick:.1f}' for tick in drone_body_ticks])
        
        # レイアウトの調整
        plt.tight_layout()
        
        # 微分グラフの出力処理
        if args.output:
            # ファイル名から拡張子を取得
            base_name = os.path.splitext(args.output)[0]
            derivative_output = f"{base_name}_derivative.png"
            plt.savefig(derivative_output, dpi=args.dpi, bbox_inches='tight')
            print(f"微分グラフを '{derivative_output}' に保存しました。")
        else:
            plt.show()
        
        plt.close(fig_derivative)
    
    # ヒートマップの作成
    print("\n=== ヒートマップの作成 ===")
    
    # グループ数を取得
    unique_groups = list(grouped_data.groups.keys())
    n_groups = len(unique_groups)
    
    if n_groups > 0:
        # 全グループのデータから色のスケールを決定
        all_heatmap_data = []
        for (tilt_angle, wall_spacing), group_df in grouped_data:
            heatmap_matrix, _, _ = create_heatmap_data(group_df)
            if not np.isnan(heatmap_matrix).all():
                # NaN値を除外してから追加
                valid_data = -heatmap_matrix.flatten()
                valid_data = valid_data[~np.isnan(valid_data)]
                all_heatmap_data.extend(valid_data)
        
        if all_heatmap_data:
            # NaN値を除外
            valid_data = [x for x in all_heatmap_data if not np.isnan(x)]
            
            if valid_data:
                # 色のスケールを決定（対称的に、0が白になるように）
                max_abs_value = max(abs(min(valid_data)), abs(max(valid_data)))
                vmin = -max_abs_value
                vmax = max_abs_value
                print(f"max_abs_value: {max_abs_value}")
                print(f"vmin: {vmin}")
                print(f"vmax: {vmax}")
            else:
                vmin = -0.1
                vmax = 0.1
                print("No valid data for color scale")
        else:
            vmin = -0.1
            vmax = 0.1
        
        # subplotの行数と列数を計算
        n_cols = min(3, n_groups)  # 最大3列
        n_rows = (n_groups + n_cols - 1) // n_cols  # 必要な行数を計算
        
        # ヒートマップ用のfigureを作成
        fig_heatmap, axes = plt.subplots(n_rows, n_cols, figsize=(5*n_cols, 4*n_rows))
        
        # 1次元配列の場合は2次元配列に変換
        if n_groups == 1:
            axes = np.array([axes])
        elif n_rows == 1:
            axes = axes.reshape(1, -1)
        elif n_cols == 1:
            axes = axes.reshape(-1, 1)
        
            # カスタムカラーマップを作成
    custom_cmap = create_custom_colormap()
    
    # 各グループに対してヒートマップを作成
    for idx, ((tilt_angle, wall_spacing), group_df) in enumerate(grouped_data):
        row = idx // n_cols
        col = idx % n_cols
        ax = axes[row, col]
        
        # ヒートマップデータを作成
        heatmap_matrix, distance_values, target_thrust_values = create_heatmap_data(group_df)
        
        if not np.isnan(heatmap_matrix).all():
            # ヒートマップをプロット（統一された色のスケールを使用）
            im = ax.imshow(-heatmap_matrix, cmap=custom_cmap, aspect='auto', 
                          extent=[min(distance_values), max(distance_values), 
                                 min(target_thrust_values), max(target_thrust_values)],
                          origin='lower', vmin=vmin, vmax=vmax)
            
            # カラーバーを追加
            cbar = plt.colorbar(im, ax=ax)
            cbar.set_label('壁効果モーメント [Nm]', fontsize=12)
            
            # 軸ラベルを設定
            ax.set_xlabel('壁との距離 [R]', fontsize=12)
            ax.set_ylabel('目標推力 [N]', fontsize=12)
            ax.set_title(f'Tilt: {tilt_angle}°, Wall: {wall_spacing}m', fontsize=14)
            
            # グリッドを追加
            ax.grid(True, alpha=0.3)
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', 
                   transform=ax.transAxes, fontsize=14)
            ax.set_title(f'Tilt: {tilt_angle}°, Wall: {wall_spacing}m', fontsize=14)
        
        # 使用されていないsubplotを非表示にする
        for idx in range(n_groups, n_rows * n_cols):
            row = idx // n_cols
            col = idx % n_cols
            axes[row, col].set_visible(False)
        
        # レイアウトの調整
        plt.tight_layout()
        
        # ヒートマップの出力処理
        if args.output:
            # ファイル名から拡張子を取得
            base_name = os.path.splitext(args.output)[0]
            heatmap_output = f"{base_name}_heatmap.png"
            plt.savefig(heatmap_output, dpi=args.dpi, bbox_inches='tight')
            print(f"ヒートマップを '{heatmap_output}' に保存しました。")
        else:
            plt.show()
        
        plt.close(fig_heatmap)
    
    # 微分ヒートマップの作成
    print("\n=== 微分ヒートマップの作成 ===")
    
    if n_groups > 0:
        # 全グループの微分データから色のスケールを決定
        all_derivative_data = []
        for (tilt_angle, wall_spacing), group_df in grouped_data:
            derivative_matrix, _, _ = create_derivative_heatmap_data(group_df)
            if not np.isnan(derivative_matrix).all():
                # NaN値を除外してから追加
                valid_data = derivative_matrix.flatten()
                valid_data = valid_data[~np.isnan(valid_data)]
                all_derivative_data.extend(valid_data)
        
        if all_derivative_data:
            # NaN値を除外
            valid_data = [x for x in all_derivative_data if not np.isnan(x)]
            
            if valid_data:
                # 色のスケールを決定（対称的に、0が白になるように）
                max_abs_value = max(abs(min(valid_data)), abs(max(valid_data)))
                vmin = -max_abs_value
                vmax = max_abs_value
                print(f"Derivative max_abs_value: {max_abs_value}")
                print(f"Derivative vmin: {vmin}")
                print(f"Derivative vmax: {vmax}")
            else:
                vmin = -0.1
                vmax = 0.1
                print("No valid derivative data for color scale")
        else:
            vmin = -0.1
            vmax = 0.1
        
        # 微分ヒートマップ用のfigureを作成
        fig_derivative_heatmap, axes = plt.subplots(n_rows, n_cols, figsize=(5*n_cols, 4*n_rows))
        
        # 1次元配列の場合は2次元配列に変換
        if n_groups == 1:
            axes = np.array([axes])
        elif n_rows == 1:
            axes = axes.reshape(1, -1)
        elif n_cols == 1:
            axes = axes.reshape(-1, 1)
        
        # 微分ヒートマップ用のカスタムカラーマップを作成
        derivative_custom_cmap = create_custom_colormap()
        
        # 各グループに対して微分ヒートマップを作成
        for idx, ((tilt_angle, wall_spacing), group_df) in enumerate(grouped_data):
            row = idx // n_cols
            col = idx % n_cols
            ax = axes[row, col]
            
            # 微分ヒートマップデータを作成
            derivative_matrix, distance_values, target_thrust_values = create_derivative_heatmap_data(group_df)
            
            if not np.isnan(derivative_matrix).all():
                # 微分ヒートマップをプロット（統一された色のスケールを使用）
                im = ax.imshow(derivative_matrix, cmap=derivative_custom_cmap, aspect='auto', 
                              extent=[min(distance_values), max(distance_values), 
                                     min(target_thrust_values), max(target_thrust_values)],
                              origin='lower', vmin=vmin, vmax=vmax)
                
                # カラーバーを追加
                cbar = plt.colorbar(im, ax=ax)
                cbar.set_label('壁効果モーメントの微分 [Nm/R]', fontsize=12)
                
                # 軸ラベルを設定
                ax.set_xlabel('壁との距離 [R]', fontsize=12)
                ax.set_ylabel('目標推力 [N]', fontsize=12)
                ax.set_title(f'Tilt: {tilt_angle}°, Wall: {wall_spacing}m (微分)', fontsize=14)
                
                # グリッドを追加
                ax.grid(True, alpha=0.3)
            else:
                ax.text(0.5, 0.5, 'No Data', ha='center', va='center', 
                       transform=ax.transAxes, fontsize=14)
                ax.set_title(f'Tilt: {tilt_angle}°, Wall: {wall_spacing}m (微分)', fontsize=14)
        
        # 使用されていないsubplotを非表示にする
        for idx in range(n_groups, n_rows * n_cols):
            row = idx // n_cols
            col = idx % n_cols
            axes[row, col].set_visible(False)
        
        # レイアウトの調整
        plt.tight_layout()
        
        # 微分ヒートマップの出力処理
        if args.output:
            # ファイル名から拡張子を取得
            base_name = os.path.splitext(args.output)[0]
            derivative_heatmap_output = f"{base_name}_derivative_heatmap.png"
            plt.savefig(derivative_heatmap_output, dpi=args.dpi, bbox_inches='tight')
            print(f"微分ヒートマップを '{derivative_heatmap_output}' に保存しました。")
        else:
            plt.show()
        
        plt.close(fig_derivative_heatmap)
    
    # 統計情報の表示
    print("\n=== 統計情報 ===")
    print(f"データポイント数: {len(results_df)}")
    print(f"Tilt Angleの種類: {len(results_df['tilt_angle'].unique())}")
    print(f"Wall Distanceの種類: {len(results_df['wall_spacing'].unique())}")
    
    # (Tilt Angle, Wall Distance)のペア数を計算
    unique_pairs = results_df[['tilt_angle', 'wall_spacing']].drop_duplicates()
    print(f"(Tilt Angle, Wall Distance)のペア数: {len(unique_pairs)}")
    
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
