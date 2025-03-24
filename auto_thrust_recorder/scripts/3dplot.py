import matplotlib.pyplot as plt
import pandas as pd
import glob
import re
import os
import argparse
import numpy as np


def create_scatter_plots(directory, resampling_ratio=1.0, keyword="right"):
    """
    指定されたディレクトリ内のCSVファイルからデータを抽出し、リサンプリング後、2つの散布図を作成します。
    distance vs torque_x のグラフにはエラーバーと散布図を追加します。

    Args:
        directory (str): CSVファイルが格納されているディレクトリのパス。
        resampling_ratio (float): リサンプリング後のデータの割合 (0.0 < ratio <= 1.0)。
    """

    all_data = []
    for filepath in glob.glob(os.path.join(directory, f'*{keyword}_raw*.csv')):
        match = re.search(r'(\d+\.\d+)R_.*', os.path.basename(filepath))
        if match:
            distance = float(match.group(1))
            try:
                df = pd.read_csv(filepath)
                if 'force_z' in df.columns and 'torque_x' in df.columns:
                    for _, row in df.iterrows():
                        all_data.append([distance, row['force_z'], row['torque_x']])
                else:
                    print(f"Warning: 'force_z' or 'torque_x' not found in {filepath}")
            except pd.errors.EmptyDataError:
                print(f"Warning: Empty file found: {filepath}")
            except Exception as e:
                print(f"Error processing {filepath}: {e}")

    if not all_data:
        print("No valid data found.")
        return

    df_all = pd.DataFrame(all_data, columns=['distance', 'force_z', 'torque_x'])

    if 0.0 < resampling_ratio < 1.0:
        df_resampled = df_all.groupby('distance').apply(
            lambda x: x.sample(frac=resampling_ratio, random_state=42)
        ).reset_index(drop=True)
        print(f"Data resampled to {resampling_ratio:.2f} of original size.")
    elif resampling_ratio == 1.0:
        df_resampled = df_all
        print("No resampling performed (ratio=1.0).")
    else:
        print("Invalid resampling ratio.  Must be 0.0 < ratio <= 1.0.  No resampling performed.")
        df_resampled = df_all

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))

    # force_z vs torque_x (変更なし)
    axes[0].scatter(df_resampled['torque_x'], df_resampled['force_z'], c=df_resampled['distance'], cmap='viridis')
    axes[0].set_xlabel('Torque X')
    axes[0].set_ylabel('Force Z')
    axes[0].set_title('Force Z vs. Torque X')
    axes[0].set_xlim(-0.4, 0.4)
    axes[0].set_ylim(0, 22)

    # distance vs torque_x (エラーバー + 散布図)
    # 平均と標準偏差を計算
    df_grouped = df_resampled.groupby('distance')['torque_x'].agg(['mean', 'std']).reset_index()

    # エラーバーをプロット
    axes[1].errorbar(df_grouped['distance'], df_grouped['mean'], yerr=df_grouped['std'], fmt='o', capsize=5, label='Mean with Std Dev', color='red')

    # 散布図をプロット (エラーバーの後に追加)
    axes[1].scatter(df_resampled['distance'], df_resampled['torque_x'], c=df_resampled['distance'], cmap='viridis', alpha=0.5, label='Raw Data')


    axes[1].set_xlabel('Distance (R)')
    axes[1].set_ylabel('Torque X')  # y軸ラベルを修正
    axes[1].set_title('Distance vs. Torque X (with Error Bars and Scatter)') #タイトルを修正
    axes[1].legend() #凡例
    axes[1].set_xlim(0, 5.2)
    axes[1].set_ylim(-0.4, 0.4)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Create scatter plots from CSV files with resampling and error bars.')
    parser.add_argument('directory', type=str, help='Path to the directory containing CSV files.')
    parser.add_argument('--ratio', type=float, default=1.0,
                        help='Resampling ratio (0.0 < ratio <= 1.0). Default is 1.0 (no resampling).')
    parser.add_argument('--keyword', type=str, default="right",
                        help='Keyword to filter CSV files. Default is "right".')
    args = parser.parse_args()

    if not 0.0 < args.ratio <= 1.0:
        print("Error: Resampling ratio must be between 0.0 and 1.0.")
        exit(1)

    create_scatter_plots(args.directory, args.ratio, args.keyword)