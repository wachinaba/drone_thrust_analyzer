import os
import glob
import numpy as np
import pandas as pd
import argparse
import sys
import re
from collections import defaultdict
from datetime import datetime

def find_csv_files(keyword, directory='.'): 
    # CSVファイル検索関数
    search_pattern = os.path.join(directory, f"*{keyword}*.csv")
    files = glob.glob(search_pattern)
    return files

def extract_parameters(filename):
    # ファイル名から距離と角度を抽出する関数
    match = re.match(r'^([0-9.]+)R_(-?[0-9]+)deg_([0-9.]+)R.*(front|back).*raw_.*\.csv$', filename, re.IGNORECASE)
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

def read_and_extract_data(file_path):
    # CSVファイルを読み込み、必要なカラムを抽出する関数
    try:
        df = pd.read_csv(file_path)
        extracted_df = df.dropna()

        print(extracted_df.head())
        if extracted_df.loc[0, 'control'] < 0.1:
            # check if the sensor bias is too high
            # calc norm of force, torque in the first row
            first_row = extracted_df.iloc[0].copy()
            force_norm = np.linalg.norm(first_row[['force_x', 'force_y', 'force_z']])
            torque_norm = np.linalg.norm(first_row[['torque_x', 'torque_y', 'torque_z']])
            if force_norm > 1.0 or torque_norm > 1.0:
                print(f"Sensor bias is too high: {force_norm}, {torque_norm}")
                # remove the bias with the first row
                extracted_df['force_x'] = extracted_df['force_x'] - first_row['force_x']
                extracted_df['force_y'] = extracted_df['force_y'] - first_row['force_y']
                extracted_df['force_z'] = extracted_df['force_z'] - first_row['force_z']
                extracted_df['torque_x'] = extracted_df['torque_x'] - first_row['torque_x']
                extracted_df['torque_y'] = extracted_df['torque_y'] - first_row['torque_y']
                extracted_df['torque_z'] = extracted_df['torque_z'] - first_row['torque_z']

        # sort by time
        extracted_df = extracted_df.sort_values('time').reset_index(drop=True)

        # add time elapsed
        extracted_df['time_elapsed'] = extracted_df['time'] - extracted_df['time'].iloc[0]

        # add time elapsed (step)
        extracted_df['control_prev'] = extracted_df['control'].shift(1)
        extracted_df['control_increase'] = extracted_df['control'] > extracted_df['control_prev']
        extracted_df['control_increase_step'] = extracted_df['control_increase'].cumsum()
        extracted_df['step_start_time'] = extracted_df.groupby('control_increase_step')['time_elapsed'].transform('first')
        extracted_df['step_elapsed_time'] = extracted_df['time_elapsed'] - extracted_df['step_start_time']
        
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

def parse_arguments():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description="CSVデータを処理し、結合するアプリケーション")
    parser.add_argument('-k', '--keyword', type=str, default='raw', help="検索するファイル名に含まれるキーワード（例: 'raw')")
    parser.add_argument('-d', '--directory', type=str, default='.', help="CSVファイルを検索するディレクトリ（デフォルト: カレントディレクトリ）")
    parser.add_argument('--output', type=str, required=True, help="すべてのプレフィックスの処理結果を1つのCSVファイルにまとめてエクスポートするファイル名")
    return parser.parse_args()

def export_data_to_csv(combined_df, output_file):
    # 結合されたデータをCSVファイルにエクスポートする関数
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if not output_file:
        output_filename = f"combined_processed_{timestamp}.csv"
    else:
        output_filename = output_file
    output_path = os.path.join('.', output_filename)
    
    combined_df.to_csv(output_path, index=False)
    print(f"データをCSVファイルにエクスポートしました: {output_path}")

def main():
    # コマンドライン引数の解析
    args = parse_arguments()

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
            print(f"ファイル '{filename}' からパラメータを抽出できませんでした。スキップします。")

    if not grouped_files:
        print("有効なパラメータで分類されたファイルがありません。終了します。")
        sys.exit(1)

    print(f"分類されたパラメータの数: {len(grouped_files)}")

    # パラメータでソート
    sorted_parameters = sorted(grouped_files.keys(), key=lambda x: (x[0], x[1], x[2]))
    print(f"ソートされたパラメータ順: {sorted_parameters}")

    thrust_coefs = {
        (0, 3.7): [117.9, 21.811, 0.5403],
        (15, 3.7): [113.8, 22.766, 0.6355],
        (30, 3.7): [97.806, 21.468, 0.5682],
        (0, 2.7): [119.92, 18.022, 0.5599],
        (15, 2.7): [117.5, 18.492, 0.5346],
        (30, 2.7): [91.475, 21.633, 0.4504],
    }

    # 結合用データを格納するリスト
    combined_data = []

    # 各グループの処理
    for (distance, tilt_angle, prop_spacing) in sorted_parameters:
        print(f"\nパラメータ: 距離={distance}, 角度={tilt_angle}, プロペラ間隔={prop_spacing}")
        files = grouped_files[(distance, tilt_angle, prop_spacing)]

        combined_data_group = []
        for file in files:
            print(f"  処理中のファイル: {file}")
            df = read_and_extract_data(file)
            if df is None or df.empty:
                print(f"  ファイル {file} の読み込みまたは抽出に失敗しました。スキップします。")
                continue

            filename = os.path.basename(file).lower()
            if 'front' in filename:
                df_processed = df.copy()
            elif 'back' in filename:
                df_processed = rotate_data(df)
            else:
                print(f"    '{file}' は 'front' も 'back' も含まないため、スキップします。")
                continue

            df_processed = df_processed[df_processed['step_elapsed_time'] > 0.5]
            
            # プレフィックスを追加
            df_processed.loc[:, 'target_thrust'] = (
                df_processed['control'] ** 2 * thrust_coefs[(tilt_angle, prop_spacing)][0] +
                df_processed['control'] * thrust_coefs[(tilt_angle, prop_spacing)][1] +
                thrust_coefs[(tilt_angle, prop_spacing)][2]
            )

            # 必要な列とパラメータを追加
            df_processed.loc[:, 'distance'] = distance
            df_processed.loc[:, 'tilt_angle'] = tilt_angle
            df_processed.loc[:, 'prop_spacing'] = prop_spacing

            for col in ['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']:
                df_processed[f"{col}_partial_variance"] = df_processed.groupby('target_thrust')[col].transform("var")

            combined_data_group.append(df_processed)

        if not combined_data_group:
            print(f"  パラメータグループ (距離={distance}, 角度={tilt_angle}, プロペラ間隔={prop_spacing}) に有効なデータがありません。")
            continue

        print(combined_data_group[0].head())

        # すべてのファイルからのデータを結合
        concatenated_group = pd.concat(combined_data_group, ignore_index=True)

        # target_thrustでグループ化して統計量を計算
        grouped_stats = concatenated_group.groupby('target_thrust').agg(
            sample_count=('target_thrust', 'count'),
            control=('control', 'mean'),
            force_x=('force_x', 'mean'),
            force_y=('force_y', 'mean'),
            force_z=('force_z', 'mean'),
            torque_x=('torque_x', 'mean'),
            torque_y=('torque_y', 'mean'),
            torque_z=('torque_z', 'mean'),
            variance_force_x=('force_x_partial_variance', 'mean'),
            variance_force_y=('force_y_partial_variance', 'mean'),
            variance_force_z=('force_z_partial_variance', 'mean'),
            variance_torque_x=('torque_x_partial_variance', 'mean'),
            variance_torque_y=('torque_y_partial_variance', 'mean'),
            variance_torque_z=('torque_z_partial_variance', 'mean')
        ).reset_index()

        # パラメータ情報を追加
        grouped_stats['distance'] = distance
        grouped_stats['tilt_angle'] = tilt_angle
        grouped_stats['prop_spacing'] = prop_spacing

        combined_data.append(grouped_stats)

    # すべてのパラメータのデータを1つのCSVにエクスポート
    if args.output and combined_data:
        combined_df = pd.concat(combined_data, ignore_index=True)
        combined_df = combined_df.sort_values(by=['distance', 'tilt_angle', 'prop_spacing', 'target_thrust']).reset_index(drop=True)
        export_data_to_csv(combined_df, args.output)
    else:
        print("結合されたデータがありません。エクスポートをスキップします。")

if __name__ == "__main__":
    main()