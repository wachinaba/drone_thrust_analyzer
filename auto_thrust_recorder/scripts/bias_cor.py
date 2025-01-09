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
from datetime import datetime

def find_csv_files(keyword, directory='.'): 
    # CSVファイル検索関数
    search_pattern = os.path.join(directory, f"*{keyword}*.csv")
    files = glob.glob(search_pattern)
    return files

def extract_parameters(filename):
    # ファイル名から距離と角度を抽出する関数
    match = re.match(r'^([0-9.]+)R_(-?[0-9]+)deg_([0-9.]+)R.*(front|back).*average_.*\.csv$', filename, re.IGNORECASE)
    if match:
        distance = float(match.group(1))
        tilt_angle = int(match.group(2))
        prop_spacing = float(match.group(3))
        return distance, tilt_angle, prop_spacing
    else:
        return None, None, None

def extract_initial_number(prefix):
    # プレフィックスから最初の数字を抽出する関数
    match = re.match(r'^([0-9.]+)d_.*$', prefix)
    if match:
        return float(match.group(1))
    else:
        return np.inf

def read_and_extract_data(file_path, force_z_options: dict[str, float] = {}):
    # CSVファイルを読み込み、必要なカラムを抽出する関数
    try:
        df = pd.read_csv(file_path)
        extracted_df = df[['force_z', 'force_y', 'torque_x']].dropna()
        if force_z_options:
            # オプションがあれば、その値にフィルタリング
            # op["min_force_z"] : 最小値
            # op["max_force_z"] : 最大値
            # op["step_force_z"] : ステップサイズ
            # minからmaxまでstep_force_zずつのリストを作成し, force_zを書き換える
            force_z_list = np.arange(force_z_options["min_force_z"], force_z_options["max_force_z"], force_z_options["step_force_z"])
            print(f"force_z_list: {force_z_list}")
            extracted_df["force_z"] = force_z_list
        return extracted_df
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return None

def rotate_data(df):
    # データをZ軸に対して180度回転させる関数
    df_rotated = df.copy()
    df_rotated['force_y'] = -df_rotated['force_y']
    df_rotated['torque_x'] = -df_rotated['torque_x']
    return df_rotated

def interpolate_data(df, z_common):
    # データを共通のforce_zに基づいて線形補間する関数
    df_sorted = df.sort_values('force_z')
    z = df_sorted['force_z'].values
    force_y = df_sorted['force_y'].values
    torque_x = df_sorted['torque_x'].values
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
    force_y_interp = np.interp(z_common, z, force_y)
    torque_x_interp = np.interp(z_common, z, torque_x)
    return {'force_y': force_y_interp, 'torque_x': torque_x_interp}

def parse_arguments():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description="CSVデータを処理し、プロットするアプリケーション")
    parser.add_argument('-k', '--keyword', type=str, required=True, help="検索するファイル名に含まれるキーワード（例: 'average')")
    parser.add_argument('-d', '--directory', type=str, default='.', help="CSVファイルを検索するディレクトリ（デフォルト: カレントディレクトリ）")
    parser.add_argument('--z_min', type=float, default=10.0, help="補間するforce_zの最小値（デフォルト: 10.0）")
    parser.add_argument('--z_max', type=float, default=25.0, help="補間するforce_zの最大値（デフォルト: 25.0）")
    parser.add_argument('--z_step', type=float, default=0.1, help="補間するforce_zのステップサイズ（デフォルト: 0.1）")
    parser.add_argument('--force_z_option_min', type=float, default=10.0, help="force_zの最小値（デフォルト: 10.0）")
    parser.add_argument('--force_z_option_max', type=float, default=25.0, help="force_zの最大値（デフォルト: 25.0）")
    parser.add_argument('--force_z_option_step', type=float, default=0.1, help="force_zのステップサイズ（デフォルト: 0.1）")
    parser.add_argument('--force_y_lim', type=float, nargs=2, default=[-1.0, 1.0], help="force_yのプロット範囲（デフォルト: -1.0 1.0）")
    parser.add_argument('--torque_x_lim', type=float, nargs=2, default=[-0.5, 0.5], help="torque_xのプロット範囲（デフォルト: -0.5 0.5）")
    parser.add_argument('--output', type=str, default=None, help="プロットを保存するファイル名（指定しない場合は表示のみ）")
    parser.add_argument('--export_csv', action='store_true', help="処理したデータをCSVにエクスポートする")
    parser.add_argument('--export_combined_csv', type=str, default=None, help="すべてのプレフィックスの処理結果を1つのCSVファイルにまとめてエクスポートするファイル名")
    return parser.parse_args()

def export_data_to_csv(prefix, z_common, force_y_avg, torque_x_avg, output_dir='.'): 
    # データをCSVファイルにエクスポートする関数
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filename = f"{prefix}_processed_{timestamp}.csv"
    output_path = os.path.join(output_dir, output_filename)
    
    export_df = pd.DataFrame({
        'force_z': z_common,
        'force_y_avg': force_y_avg,
        'torque_x_avg': torque_x_avg
    })
    
    export_df.to_csv(output_path, index=False)
    print(f"データをCSVファイルにエクスポートしました: {output_path}")

def main():
    # コマンドライン引数の解析
    args = parse_arguments()

    force_z_options = {
        "min_force_z": args.force_z_option_min,
        "max_force_z": args.force_z_option_max,
        "step_force_z": args.force_z_option_step
    }

    if None in force_z_options.values():
        print("force_z_optionsはすべての値を設定してください。")
        sys.exit(1)

    if force_z_options["min_force_z"] > force_z_options["max_force_z"]:
        print("min_force_zはmax_force_zより大きくなければなりません。")
        sys.exit(1)

    if force_z_options["step_force_z"] <= 0:
        print("step_force_zは正の値でなければなりません。")
        sys.exit(1)

    # ファイルの検索
    csv_files = find_csv_files(args.keyword, args.directory)
    if not csv_files:
        print(f"キーワード '{args.keyword}' を含むCSVファイルが見つかりません。")
        sys.exit(1)

    print(f"見つかったファイル数: {len(csv_files)}")

    # ファイルをプレフィックスで分類
    grouped_files = defaultdict(list)
    for file in csv_files:
        filename = os.path.basename(file)
        distance, tilt_angle, prop_spacing = extract_parameters(filename)
        if distance is not None and tilt_angle is not None and prop_spacing is not None:
            grouped_files[(distance, tilt_angle, prop_spacing)].append(file)
        else:
            print(f"ファイル名 '{filename}' からパラメータを抽出できませんでした。スキップします。")

    if not grouped_files:
        print("有効なパラメータで分類されたファイルがありません。終了します。")
        sys.exit(1)

    print(f"分類されたパラメータの数: {len(grouped_files)}")

    # パラメータでソート
    sorted_parameters = sorted(grouped_files.keys(), key=lambda x: (x[0], x[1], x[2]))
    print(f"ソートされたパラメータ順: {sorted_parameters}")

    # 補間用の共通のforce_z軸を定義
    z_min, z_max, z_step = args.z_min, args.z_max, args.z_step
    z_common = np.arange(z_min, z_max + z_step, z_step)

    # プロットの設定
    plt.figure(figsize=(12, 10))
    colors = [(1, 0, 0), (0, 0, 0)]  # 赤 -> 黒
    n_bins = len(sorted_parameters)
    red_to_black = LinearSegmentedColormap.from_list('RedBlack', colors, N=n_bins)
    num_groups = len(sorted_parameters)
    colors = red_to_black(np.linspace(0, 1, num_groups))

    # サブプロットの設定
    ax1 = plt.subplot(2, 1, 1)
    ax2 = plt.subplot(2, 1, 2, sharex=ax1)

    combined_data = []  # すべてのデータをまとめるリスト

    # 各グループの処理
    for idx, (distance, tilt_angle, prop_spacing) in enumerate(sorted_parameters):
        color = colors[idx]
        print(f"\nパラメータ: 距離={distance}, 角度={tilt_angle}, プロペラ間隔={prop_spacing}")
        files = grouped_files[(distance, tilt_angle, prop_spacing)]
        all_force_y = []
        all_torque_x = []

        for file in files:
            print(f"  処理中のファイル: {file}")
            df = read_and_extract_data(file, force_z_options)
            if df is None or df.empty:
                print(f"  ファイル {file} の読み込みまたは抽出に失敗しました。スキップします。")
                continue

            filename = os.path.basename(file).lower()
            if 'front' in filename:
                df_processed = df
            elif 'back' in filename:
                df_processed = rotate_data(df)
            else:
                print(f"    '{file}' は 'front' も 'back' も含まないため、スキップします。")
                continue

            min_z = df_processed['force_z'].min()
            max_z = df_processed['force_z'].max()
            if min_z > z_min or max_z < z_max:
                warnings.warn(f"ファイル {file} はforce_zの範囲 {z_min}-{z_max} を完全にはカバーしていません。")

            interpolated = interpolate_data(df_processed, z_common)
            if np.isnan(interpolated['force_y']).all() or np.isnan(interpolated['torque_x']).all():
                print(f"    ファイル {file} の補間に失敗しました。スキップします。")
                continue

            all_force_y.append(interpolated['force_y'])
            all_torque_x.append(interpolated['torque_x'])

        if not all_force_y or not all_torque_x:
            print(f"  パラメータ 距離={distance}, 角度={tilt_angle} に対して有効なデータがありません。")
            continue

        # 平均の計算
        force_y_avg = np.nanmean(all_force_y, axis=0)
        torque_x_avg = np.nanmean(all_torque_x, axis=0)

        # プロットの追加
        ax1.plot(z_common, force_y_avg, label=f'distance={distance}, angle={tilt_angle}, prop_spacing={prop_spacing}', color=color)
        ax2.plot(z_common, torque_x_avg, label=f'distance={distance}, angle={tilt_angle}, prop_spacing={prop_spacing}', color=color)

        # データのエクスポート
        if args.export_csv:
            prefix = f"{distance}d_{tilt_angle}deg_{prop_spacing}R"
            export_data_to_csv(prefix, z_common, force_y_avg, torque_x_avg, output_dir=args.directory)

        # 結合用データに追加
        data_dict = {
            'distance': [distance] * len(z_common),
            'tilt_angle': [tilt_angle] * len(z_common),
            'prop_spacing': [prop_spacing] * len(z_common),
            'force_z': z_common,
            'force_y': force_y_avg,
            'torque_x': torque_x_avg
        }
        combined_data.append(pd.DataFrame(data_dict))

    # すべてのパラメータのデータを1つのCSVにエクスポート
    if args.export_combined_csv and combined_data:
        combined_df = pd.concat(combined_data, ignore_index=True)
        combined_df = combined_df.sort_values(by=['distance', 'tilt_angle', 'prop_spacing', 'force_z']).reset_index(drop=True)
        combined_df.to_csv(args.export_combined_csv, index=False)
        print(f"すべてのパラメータのデータをまとめたCSVファイルをエクスポートしました: {args.export_combined_csv}")

    # プロットの設定
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