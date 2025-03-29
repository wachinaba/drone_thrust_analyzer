import matplotlib.pyplot as plt
import pandas as pd
import glob
import re
import os
import argparse
import numpy as np
from scipy.stats import norm  # 正規分布用

def filter_row(force_z, control, force_z_min, force_z_max, control_min, control_max):
    filter_values = [values[1] for values in [
        (force_z_min, force_z >= force_z_min if force_z_min is not None else True), 
        (force_z_max, force_z <= force_z_max if force_z_max is not None else True), 
        (control_min, control >= control_min if control_min is not None else True), 
        (control_max, control <= control_max if control_max is not None else True)
        ]]
    if len(filter_values) == 0:
        return True
    else:
        return all(filter_values)

def create_scatter_plots(directory, resampling_ratio=1.0, keywords=["right"], force_z_min=None, force_z_max=None, control_min=None, control_max=None):
    """
    ガウス過程回帰を用いて、全データプロットのエラーバーを信頼区間として表示します。
    ヒストグラムも追加します。
    force_z_threshold で指定された値以上の force_z データのみを使用します。
    """
    all_data = []
    all_data_by_keyword = {keyword: [] for keyword in keywords}

    for keyword in keywords:
        for filepath in glob.glob(os.path.join(directory, f'*{keyword}_raw*.csv')):
            match = re.search(r'(\d+\.\d+)R_.*', os.path.basename(filepath))
            if match:
                distance = float(match.group(1))
                try:
                    df = pd.read_csv(filepath)
                    if 'force_z' in df.columns and 'torque_x' in df.columns:
                        for _, row in df.iterrows():
                            # force_z フィルタリング
                            if filter_row(row['force_z'], row['control'], force_z_min, force_z_max, control_min, control_max):
                                all_data.append([keyword, distance, row['force_z'], row['torque_x'], row['control']])
                                all_data_by_keyword[keyword].append([distance, row['force_z'], row['torque_x'], row['control']])
                    else:
                        print(f"Warning: 'force_z' or 'torque_x' not found in {filepath}")
                except pd.errors.EmptyDataError:
                    print(f"Warning: Empty file found: {filepath}")
                except Exception as e:
                    print(f"Error processing {filepath}: {e}")

    if not all_data:
        print("No valid data found.")
        return

    df_all = pd.DataFrame(all_data, columns=['keyword', 'distance', 'force_z', 'torque_x', 'control'])

    # リサンプリング (キーワードごと)
    df_resampled_by_keyword = {}
    for keyword in keywords:
        df_keyword = pd.DataFrame(all_data_by_keyword[keyword], columns=['distance', 'force_z', 'torque_x', 'control'])
        if 0.0 < resampling_ratio < 1.0:
            df_resampled = df_keyword.groupby('distance').apply(
                lambda x: x.sample(frac=resampling_ratio, random_state=42)
            ).reset_index(drop=True)
            print(f"Data for keyword '{keyword}' resampled to {resampling_ratio:.2f} of original size.")
        elif resampling_ratio == 1.0:
            df_resampled = df_keyword
            print(f"No resampling performed for keyword '{keyword}' (ratio=1.0).")
        else:
            print(f"Invalid resampling ratio for keyword '{keyword}'.  No resampling performed.")
            df_resampled = df_keyword
        df_resampled_by_keyword[keyword] = df_resampled

    # 全データのリサンプリング
    if 0.0 < resampling_ratio < 1.0:
        df_all_resampled = df_all.groupby(['keyword', 'distance']).apply(
            lambda x: x.sample(frac=resampling_ratio, random_state=42)
        ).reset_index(drop=True)
        print(f"All data resampled to {resampling_ratio:.2f} of original size.")
    elif resampling_ratio == 1.0:
        df_all_resampled = df_all
        print("No resampling performed for all data (ratio=1.0).")
    else:
        print("Invalid resampling ratio for all data. Must be 0.0 < ratio <= 1.0. No resampling performed.")
        df_all_resampled = df_all

    # torque_x / force_z を計算 (0除算対策)
    df_all_resampled['torque_x_div_force_z'] = df_all_resampled.apply(lambda row: row['torque_x'] / row['force_z'] if row['force_z'] != 0 else np.nan, axis=1)

    for keyword in keywords:
        df_resampled_by_keyword[keyword]['torque_x_div_force_z'] = df_resampled_by_keyword[keyword].apply(lambda row: row['torque_x'] / row['force_z'] if row['force_z'] != 0 else np.nan, axis=1)

    num_rows = len(keywords) + 1
    num_cols = 5 
    fig, axes = plt.subplots(num_rows, num_cols, figsize=(24, 4 * num_rows))  # figsizeも調整

    # キーワードごとのプロット
    for i, keyword in enumerate(keywords):
        df_resampled = df_resampled_by_keyword[keyword]

        # force_z vs torque_x
        axes[i, 0].scatter(df_resampled['torque_x'], df_resampled['force_z'], c=df_resampled['distance'], cmap='viridis')
        axes[i, 0].set_xlabel('Torque X')
        axes[i, 0].set_ylabel('Force Z')
        axes[i, 0].set_title(f'Force Z vs. Torque X ({keyword})')
        axes[i, 0].set_xlim(-0.4, 0.4)
        axes[i, 0].set_ylim(0, 22)

        # distance vs torque_x (エラーバー + 散布図)
        df_grouped = df_resampled.groupby('distance')['torque_x'].agg(['mean', 'std']).reset_index()
        axes[i, 1].errorbar(df_grouped['distance'], df_grouped['mean'], yerr=df_grouped['std'], fmt='o', capsize=5, label='Mean with Std Dev', color='red')
        axes[i, 1].scatter(df_resampled['distance'], df_resampled['torque_x'], c=df_resampled['distance'], cmap='viridis', alpha=0.5, label='Raw Data')
        axes[i, 1].set_xlabel('Distance (R)')
        axes[i, 1].set_ylabel('Torque X')
        axes[i, 1].set_title(f'Distance vs. Torque X ({keyword})')
        axes[i, 1].legend()
        axes[i, 1].set_xlim(0, 5.2)
        axes[i, 1].set_ylim(-0.4, 0.4)

        # distance vs (torque_x / force_z)
        df_grouped_ratio = df_resampled.groupby('distance')['torque_x_div_force_z'].agg(['mean', 'std']).reset_index()
        axes[i, 2].errorbar(df_grouped_ratio['distance'], df_grouped_ratio['mean'], yerr=df_grouped_ratio['std'], fmt='o', capsize=5, color='red', label='Mean with Std Dev')
        axes[i, 2].scatter(df_resampled['distance'], df_resampled['torque_x_div_force_z'], c=df_resampled['distance'], cmap='viridis', alpha=0.5, label='Raw Data')
        axes[i, 2].set_xlabel('Distance (R)')
        axes[i, 2].set_ylabel('Torque X / Force Z')
        axes[i, 2].set_title(f'Distance vs. Torque X / Force Z ({keyword})')
        axes[i, 2].legend()
        axes[i, 2].set_xlim(0, 5.2)
        axes[i, 2].set_ylim(-0.02, 0.02)

        # force_z のヒストグラム (追加)
        axes[i, 3].hist(df_resampled['force_z'], bins=60, alpha=0.7, label='Histogram')
        axes[i, 3].set_xlabel('Force Z')
        axes[i, 3].set_ylabel('Frequency')
        axes[i, 3].set_title(f'Force Z Histogram ({keyword})')

        # control vs force_z
        axes[i, 4].scatter(df_resampled['control'], df_resampled['force_z'], c=df_resampled['distance'], cmap='viridis')
        axes[i, 4].set_xlabel('Control')
        axes[i, 4].set_ylabel('Force Z')
        axes[i, 4].set_title(f'Control vs. Force Z ({keyword})')


    # 全データのプロット (最終行)
    cmap = plt.get_cmap('tab10')
    colors = [cmap(i) for i in range(len(keywords))]
    color_dict = dict(zip(keywords, colors))

    # force_z vs torque_x (全データ)
    for keyword, color in color_dict.items():
        df_subset = df_all_resampled[df_all_resampled['keyword'] == keyword]
        axes[num_rows - 1, 0].scatter(df_subset['torque_x'], df_subset['force_z'], color=color, label=keyword, alpha=0.6)

    axes[num_rows - 1, 0].set_xlabel('Torque X')
    axes[num_rows - 1, 0].set_ylabel('Force Z')
    axes[num_rows - 1, 0].set_title('Force Z vs. Torque X (All Data)')
    axes[num_rows - 1, 0].legend()
    axes[num_rows - 1, 0].set_xlim(-0.4, 0.4)
    axes[num_rows - 1, 0].set_ylim(0, 22)

    # distance vs torque_x (全データ, 標準偏差を折れ線で近似)
    for keyword, color in color_dict.items():
        df_subset = df_all_resampled[df_all_resampled['keyword'] == keyword]
        df_grouped = df_subset.groupby('distance')['torque_x'].agg(['mean', 'std']).reset_index()

        # 平均値と標準偏差の上下限をプロット
        axes[num_rows - 1, 1].plot(df_grouped['distance'], df_grouped['mean'], color=color, label=f'{keyword} (Mean)')
        axes[num_rows - 1, 1].plot(df_grouped['distance'], df_grouped['mean'] + df_grouped['std'], color=color, linestyle='--', alpha=0.7, label=f'{keyword} (Mean + Std)')
        axes[num_rows - 1, 1].plot(df_grouped['distance'], df_grouped['mean'] - df_grouped['std'], color=color, linestyle='--', alpha=0.7, label=f'{keyword} (Mean - Std)')
        axes[num_rows - 1, 1].scatter(df_subset['distance'], df_subset['torque_x'], color=color, alpha=0.4)


    axes[num_rows - 1, 1].set_xlabel('Distance (R)')
    axes[num_rows - 1, 1].set_ylabel('Torque X')
    axes[num_rows - 1, 1].set_title('Distance vs. Torque X (All Data with Std Dev Bounds)')
    axes[num_rows - 1, 1].legend()
    axes[num_rows - 1, 1].set_xlim(0, 5.2)
    axes[num_rows - 1, 1].set_ylim(-0.4, 0.4)

    # distance vs (torque_x / force_z) (全データ)
    for keyword, color in color_dict.items():
        df_subset = df_all_resampled[df_all_resampled['keyword'] == keyword]
        df_grouped_ratio = df_subset.groupby('distance')['torque_x_div_force_z'].agg(['mean', 'std']).reset_index()

        # 平均値と標準偏差の上下限をプロット
        axes[num_rows - 1, 2].plot(df_grouped_ratio['distance'], df_grouped_ratio['mean'], color=color, label=f'{keyword} (Mean)')
        axes[num_rows - 1, 2].plot(df_grouped_ratio['distance'], df_grouped_ratio['mean'] + df_grouped_ratio['std'], color=color, linestyle='--', alpha=0.7, label=f'{keyword} (Mean + Std)')
        axes[num_rows - 1, 2].plot(df_grouped_ratio['distance'], df_grouped_ratio['mean'] - df_grouped_ratio['std'], color=color, linestyle='--', alpha=0.7, label=f'{keyword} (Mean - Std)')

        axes[num_rows - 1, 2].scatter(df_subset['distance'], df_subset['torque_x_div_force_z'], color=color, alpha=0.4)

    axes[num_rows - 1, 2].set_xlabel('Distance (R)')
    axes[num_rows - 1, 2].set_ylabel('Torque X / Force Z')
    axes[num_rows - 1, 2].set_title('Distance vs. Torque X / Force Z (All Data with Std Dev Bounds)')
    axes[num_rows - 1, 2].legend()
    axes[num_rows - 1, 2].set_xlim(0, 5.2)
    axes[num_rows - 1, 2].set_ylim(-0.02, 0.02)



    # force_z のヒストグラム (全データ, 追加)
    for keyword, color in color_dict.items():
        df_subset = df_all_resampled[df_all_resampled['keyword'] == keyword]
        axes[num_rows - 1, 3].hist(df_subset['force_z'], bins=60, alpha=0.5, label=f'{keyword}', color=color)

    axes[num_rows - 1, 3].set_xlabel('Force Z')
    axes[num_rows - 1, 3].set_ylabel('Frequency')
    axes[num_rows - 1, 3].set_title('Force Z Histogram (All Data)')
    axes[num_rows - 1, 3].legend()



    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Create scatter plots from CSV files with resampling, error bars, and Gaussian Process Regression.')
    parser.add_argument('directory', type=str, help='Path to the directory containing CSV files.')
    parser.add_argument('--ratio', type=float, default=1.0,
                        help='Resampling ratio (0.0 < ratio <= 1.0). Default is 1.0 (no resampling).')
    parser.add_argument('--keywords', nargs='+', type=str, default=["right"],
                        help='Keywords to filter CSV files. Default is ["right"].')
    parser.add_argument('--force_z_min', type=float,
                        help='Minimum value of force_z to include. Default is 0.0.')  # force_z 閾値の引数を追加
    parser.add_argument('--force_z_max', type=float,
                        help='Maximum value of force_z to include. Default is 30.0.')  # force_z 閾値の引数を追加
    parser.add_argument('--control_min', type=float,
                        help='Minimum value of control to include. Default is 0.0.')  # control 閾値の引数を追加
    parser.add_argument('--control_max', type=float,
                        help='Maximum value of control to include. Default is 1.0.')  # control 閾値の引数を追加
    args = parser.parse_args()

    if not 0.0 < args.ratio <= 1.0:
        print("Error: Resampling ratio must be between 0.0 and 1.0.")
        exit(1)

    create_scatter_plots(args.directory, args.ratio, args.keywords, args.force_z_min, args.force_z_max, args.control_min, args.control_max) 