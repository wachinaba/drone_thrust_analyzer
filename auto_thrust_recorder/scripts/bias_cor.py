import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import argparse
import sys
import re
from collections import defaultdict
import warnings

def find_csv_files(keyword, directory='.'):
    """
    指定されたキーワードを含むCSVファイルを検索します。

    Parameters:
    - keyword (str): ファイル名に含まれる必要があるキーワード。
    - directory (str): 検索するディレクトリ（デフォルトはカレントディレクトリ）。

    Returns:
    - list of str: 見つかったファイルのパスのリスト。
    """
    search_pattern = os.path.join(directory, f"*{keyword}*.csv")
    files = glob.glob(search_pattern)
    return files

def extract_prefix(filename):
    """
    ファイル名からプレフィックスを抽出します。

    例: '0.50d_0deg_25N_back_average_20240929_175842.csv' -> '0.50d_0deg_25N'

    Parameters:
    - filename (str): ファイル名。

    Returns:
    - str: 抽出されたプレフィックス。
    """
    # 正規表現を使用してプレフィックスを抽出
    #match = re.match(r'^([0-9.]+d_-?[0-9]+deg_[0-9]+N)_(front|back)_average_.*\.csv$', filename, re.IGNORECASE)
    match = re.match(r'^([0-9.]+d_-?[0-9]+deg)_(front|back)_average_.*\.csv$', filename, re.IGNORECASE)
    if match:
        return match.group(1)
    else:
        return None

def extract_initial_number(prefix):
    """
    プレフィックスから最初の数字を抽出します。

    例: '0.50d_0deg_25N' -> 0.50

    Parameters:
    - prefix (str): プレフィックス。

    Returns:
    - float: 抽出された最初の数字。
    """
    match = re.match(r'^([0-9.]+)d_.*$', prefix)
    if match:
        return float(match.group(1))
    else:
        return np.inf  # ソート時に最後になるように

def read_and_extract_data(file_path):
    """
    CSVファイルを読み込み、必要なカラムを抽出します。

    Parameters:
    - file_path (str): 読み込むCSVファイルのパス。

    Returns:
    - DataFrame: 'force_z', 'force_y', 'torque_x' のデータを含むDataFrame。
    """
    try:
        df = pd.read_csv(file_path)
        # 必要なカラムのみ抽出
        extracted_df = df[['force_z', 'force_y', 'torque_x']].dropna()
        return extracted_df
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return None

def rotate_data(df):
    """
    データをZ軸に対して180度回転させます。
    具体的には、force_y と torque_x を反転させます。

    Parameters:
    - df (DataFrame): 'force_z', 'force_y', 'torque_x' を含むDataFrame。

    Returns:
    - DataFrame: 回転後のデータを含むDataFrame。
    """
    df_rotated = df.copy()
    df_rotated['force_y'] = -df_rotated['force_y']
    df_rotated['torque_x'] = -df_rotated['torque_x']
    return df_rotated

def interpolate_data(df, z_common):
    """
    データを共通のforce_zに基づいて線形補間します。

    Parameters:
    - df (DataFrame): 'force_z', 'force_y', 'torque_x' を含むDataFrame。
    - z_common (numpy.ndarray): 補間後の共通のforce_z値。

    Returns:
    - dict: 補間された 'force_y' と 'torque_x' のデータ。
    """
    # ソート
    df_sorted = df.sort_values('force_z')
    z = df_sorted['force_z'].values
    force_y = df_sorted['force_y'].values
    torque_x = df_sorted['torque_x'].values

    # 補間範囲を超えるデータポイントを排除
    mask = (z >= z_common[0]) & (z <= z_common[-1])
    if not np.any(mask):
        warnings.warn("データが補間範囲内にありません。全てNaNになります。")
        return {'force_y': np.full_like(z_common, np.nan), 'torque_x': np.full_like(z_common, np.nan)}
    z = z[mask]
    force_y = force_y[mask]
    torque_x = torque_x[mask]

    if len(z) < 2:
        warnings.warn("補間に十分なデータポイントがありません。全てNaNになります。")
        return {'force_y': np.full_like(z_common, np.nan), 'torque_x': np.full_like(z_common, np.nan)}

    # 補間
    force_y_interp = np.interp(z_common, z, force_y)
    torque_x_interp = np.interp(z_common, z, torque_x)

    return {'force_y': force_y_interp, 'torque_x': torque_x_interp}

def parse_arguments():
    """
    コマンドライン引数を解析します。

    Returns:
    - Namespace: 解析された引数を含むNamespaceオブジェクト。
    """
    parser = argparse.ArgumentParser(description="CSVデータを処理し、プロットするアプリケーション")
    parser.add_argument(
        '-k', '--keyword',
        type=str,
        required=True,
        help="検索するファイル名に含まれるキーワード（例: 'average')"
    )
    parser.add_argument(
        '-d', '--directory',
        type=str,
        default='.',
        help="CSVファイルを検索するディレクトリ（デフォルト: カレントディレクトリ）"
    )
    parser.add_argument(
        '--z_min',
        type=float,
        default=10.0,
        help="補間するforce_zの最小値（デフォルト: 10.0）"
    )
    parser.add_argument(
        '--z_max',
        type=float,
        default=25.0,
        help="補間するforce_zの最大値（デフォルト: 25.0）"
    )
    parser.add_argument(
        '--z_step',
        type=float,
        default=0.1,
        help="補間するforce_zのステップサイズ（デフォルト: 0.1）"
    )
    parser.add_argument(
        '--force_y_lim',
        type=float,
        nargs=2,
        default=[-1.0, 1.0],
        help="force_yのプロット範囲（デフォルト: -1.0 1.0）"
    )
    parser.add_argument(
        '--torque_x_lim',
        type=float,
        nargs=2,
        default=[-0.5, 0.5],
        help="torque_xのプロット範囲（デフォルト: -0.5 0.5）"
    )
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help="プロットを保存するファイル名（指定しない場合は表示のみ）"
    )
    return parser.parse_args()

def main():
    # 1. コマンドライン引数の解析
    args = parse_arguments()

    # 2. ファイルの検索
    csv_files = find_csv_files(args.keyword, args.directory)

    if not csv_files:
        print(f"キーワード '{args.keyword}' を含むCSVファイルが見つかりません。")
        sys.exit(1)

    print(f"見つかったファイル数: {len(csv_files)}")

    # 3. ファイルをプレフィックスで分類
    grouped_files = defaultdict(list)
    for file in csv_files:
        filename = os.path.basename(file)
        prefix = extract_prefix(filename)
        if prefix:
            grouped_files[prefix].append(file)
        else:
            print(f"ファイル名 '{filename}' からプレフィックスを抽出できませんでした。スキップします。")

    if not grouped_files:
        print("有効なプレフィックスで分類されたファイルがありません。終了します。")
        sys.exit(1)

    print(f"分類されたプレフィックス数: {len(grouped_files)}")

    # 4. プレフィックスを最初の数字でソート
    sorted_prefixes = sorted(grouped_files.keys(), key=extract_initial_number)
    print(f"ソートされたプレフィックス順: {sorted_prefixes}")

    # 5. 補間用の共通のforce_z軸を定義
    z_min, z_max, z_step = args.z_min, args.z_max, args.z_step
    z_common = np.arange(z_min, z_max + z_step, z_step)

    # プロットの設定
    plt.figure(figsize=(12, 10))
    
    # グラデーションの色を設定
    # 赤から黒へのグラデーションカラーマップを作成
    colors = [(1, 0, 0), (0, 0, 0)]  # 赤 -> 黒
    n_bins = len(sorted_prefixes)
    red_to_black = LinearSegmentedColormap.from_list('RedBlack', colors, N=n_bins)

    num_groups = len(sorted_prefixes)
    colors = red_to_black(np.linspace(0, 1, num_groups))

    # サブプロットの設定
    ax1 = plt.subplot(2, 1, 1)
    ax2 = plt.subplot(2, 1, 2, sharex=ax1)

    # 6. 各グループの処理
    for idx, prefix in enumerate(sorted_prefixes):
        color = colors[idx]
        print(f"\nプレフィックス: {prefix}")
        files = grouped_files[prefix]
        all_force_y = []
        all_torque_x = []

        for file in files:
            print(f"  処理中のファイル: {file}")
            df = read_and_extract_data(file)
            if df is None or df.empty:
                print(f"  ファイル {file} の読み込みまたは抽出に失敗しました。スキップします。")
                continue

            # ファイル名に "front" または "back" が含まれているか確認
            filename = os.path.basename(file).lower()
            if 'front' in filename:
                # データをそのまま使用
                print(f"    '{file}' は 'front' を含むため、データをそのまま使用します。")
                df_processed = df
            elif 'back' in filename:
                # データをZ軸に対し180度回転
                print(f"    '{file}' は 'back' を含むため、データを180度回転させます。")
                df_processed = rotate_data(df)
            else:
                # "front" でも "back" でもない場合、処理をスキップ
                print(f"    '{file}' は 'front' も 'back' も含まないため、スキップします。")
                continue

            # force_zの範囲内にデータが存在するか確認
            min_z = df_processed['force_z'].min()
            max_z = df_processed['force_z'].max()
            if min_z > z_min or max_z < z_max:
                warnings.warn(
                    f"ファイル {file} はforce_zの範囲 {z_min}-{z_max} を完全にはカバーしていません。"
                )

            interpolated = interpolate_data(df_processed, z_common)
            if np.isnan(interpolated['force_y']).all() or np.isnan(interpolated['torque_x']).all():
                print(f"    ファイル {file} の補間に失敗しました。スキップします。")
                continue

            all_force_y.append(interpolated['force_y'])
            all_torque_x.append(interpolated['torque_x'])

        if not all_force_y or not all_torque_x:
            print(f"  プレフィックス '{prefix}' に対して有効なデータがありません。")
            continue

        # 平均の計算
        force_y_avg = np.nanmean(all_force_y, axis=0)
        torque_x_avg = np.nanmean(all_torque_x, axis=0)

        # プロットの追加
        ax1.plot(z_common, force_y_avg, label=prefix, color=color)
        ax2.plot(z_common, torque_x_avg, label=prefix, color=color)

    # 7. プロットの設定
    ax1.set_ylabel('force_y (N)')
    ax1.set_ylim(args.force_y_lim)
    ax1.set_title('average force_y, torque_x vs vertical force')
    ax1.grid(True)
    ax1.legend()

    ax2.set_ylabel('torque_x (Nm)')
    ax2.set_xlabel('vertical force (N)')
    ax2.set_ylim(args.torque_x_lim)
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()

    # プロットを保存するか表示するか
    if args.output:
        plt.savefig(args.output, dpi=300)
        print(f"\nプロットを '{args.output}' に保存しました。")
    else:
        plt.show()

if __name__ == "__main__":
    main()
