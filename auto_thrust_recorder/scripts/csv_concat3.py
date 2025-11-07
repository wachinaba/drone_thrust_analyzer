import os
import glob
import numpy as np
import pandas as pd
import argparse
import sys
import re
from collections import defaultdict
from datetime import datetime
from pathlib import Path

def find_csv_files(keywords, directory='.', and_keywords=False):
    """CSVファイルをキーワードに基づいて再帰的に検索する関数 (Pathlibを使用)。

    directory は文字列またはディレクトリのリストを受け付ける。
    複数ディレクトリが指定された場合は全てを走査し、重複は排除する。
    """
    directories = directory
    if not isinstance(directories, (list, tuple, set)):
        directories = [directories]

    files_set = set()
    for dir_path in directories:
        for keyword in keywords:
            search_pattern = f"*{keyword}*.csv"
            for file in Path(dir_path).rglob(search_pattern):
                files_set.add(str(file))  # Pathオブジェクトを文字列に変換

    files = sorted(files_set)
    if and_keywords:
        files = [file for file in files if all(keyword in file for keyword in keywords)]
    return files

def extract_parameters(filename):
    """ファイル名から距離、角度、キーワードを抽出する関数。
    キーワードは正規表現で使用されます。"""
    matcher = r"distance=(-?\d+\.?\d*)\[R\]_tilt=(-?\d+)\[deg\]_fold=(-?\d+)\[deg\]_wheelbase=(-?\d+\.?\d*)\[R\]_direction=([a-z_]+)_height=(-?\d+\.?\d*)\[mm\]_wallspacing=(-?\d+\.?\d*)\[m\]_flowdistance=(-?\d+\.?\d*)\[m\]_.*\.csv"
    print(matcher)
    match = re.match(matcher, filename, re.IGNORECASE)
    if match:
        distance = float(match.group(1))
        tilt_angle = int(match.group(2))
        fold_angle = int(match.group(3))
        prop_spacing = float(match.group(4))
        keyword = match.group(5)
        height = float(match.group(6))
        wall_spacing = float(match.group(7))
        flow_distance = float(match.group(8))
        return {
            'distance': distance,
            'tilt_angle': tilt_angle,
            'fold_angle': fold_angle,
            'prop_spacing': prop_spacing,
            'keyword': keyword,
            'height': height,
            'wall_spacing': wall_spacing,
            'flow_distance': flow_distance
        }
    else:
        return None

def read_and_extract_data(file_path):
    """CSVファイルを読み込み、必要なカラムを抽出する関数。

    戻り値: (DataFrame | None, エラー理由文字列 | None)
    """
    REQUIRED_COLUMNS = ['time', 'control', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']
    try:
        df = pd.read_csv(file_path)
    except UnicodeDecodeError as e:
        return None, f"UnicodeDecodeError: {e}"
    except pd.errors.ParserError as e:
        return None, f"ParserError: {e}"
    except Exception as e:
        return None, f"ReadError: {e}"

    try:
        extracted_df = df.dropna()
        if extracted_df.empty:
            return None, "EmptyDataAfterDropNA"

        missing = [c for c in REQUIRED_COLUMNS if c not in extracted_df.columns]
        if missing:
            return None, f"MissingColumns: {','.join(missing)}"

        # センサバイアス除去のための安全な先頭行参照
        if extracted_df.iloc[0]['control'] < 0.1:
            first_row = extracted_df.iloc[0].copy()
            force_norm = np.linalg.norm(first_row[['force_x', 'force_y', 'force_z']])
            torque_norm = np.linalg.norm(first_row[['torque_x', 'torque_y', 'torque_z']])
            if force_norm > 1.0 or torque_norm > 1.0:
                print(f"Sensor bias is too high: {force_norm}, {torque_norm}")
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

        return extracted_df, None
    except Exception as e:
        return None, f"ProcessError: {e}"

def parse_arguments():
    """コマンドライン引数の解析。"""
    parser = argparse.ArgumentParser(description="CSVデータを処理し、結合するアプリケーション")
    parser.add_argument('-k', '--keywords', nargs='+', type=str, default=['raw'], help="検索するファイル名に含まれるキーワードのリスト（例: 'raw', 'processed')")
    parser.add_argument('-d', '--directory', nargs='+', type=str, default=['.'], help="CSVファイルを検索するディレクトリ（複数指定可、デフォルト: 現在ディレクトリ）")
    parser.add_argument('-a', '--and_keywords', action='store_true', help="AND条件でファイルを検索する")
    parser.add_argument('--output', type=str, required=True, help="すべてのプレフィックスの処理結果を1つのCSVファイルにまとめてエクスポートするファイル名")
    parser.add_argument('-s', '--skip-seconds', type=float, default=0.0, help="各CSVの先頭から指定秒数をスキップして集計（time列を基準）")
    parser.add_argument('--step-warmup', type=float, default=0.5, help="各ステップ立ち上がり時の除外秒数（0で無効）")
    return parser.parse_args()

def export_data_to_csv(combined_df, output_file):
    """結合されたデータをCSVファイルにエクスポートする関数。"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if not output_file:
        output_filename = f"combined_processed_{timestamp}.csv"
    else:
        output_filename = output_file
    output_path = Path('.') / output_filename  # pathlibを使用

    combined_df.to_csv(output_path, index=False)
    print(f"データをCSVファイルにエクスポートしました: {output_path}")

def main():
    """メイン関数。"""
    args = parse_arguments()

    # ファイルの検索
    csv_files = find_csv_files(args.keywords, args.directory, args.and_keywords)
    if not csv_files:
        print(f"キーワード '{args.keywords}' を含むCSVファイルが見つかりません。")
        sys.exit(1)

    print(f"見つかったファイル数: {len(csv_files)}")

    # ファイルをプレフィックスで分類
    grouped_files = defaultdict(list)
    failed_param_files = []  # パラメータ抽出（ファイル名正規表現）に失敗
    failed_data_files = []   # データ読み込み/抽出に失敗（理由付き）
    for file in csv_files:
        filename = os.path.basename(file)
        params = extract_parameters(filename)
        if params:
            distance, tilt_angle, fold_angle, prop_spacing, height = params['distance'], params['tilt_angle'], params['fold_angle'], params['prop_spacing'], params['height']
            keyword = params['keyword']
            wall_spacing = params['wall_spacing']
            flow_distance = params['flow_distance']
            grouped_files[(distance, tilt_angle, fold_angle, prop_spacing, keyword, height, wall_spacing, flow_distance)].append(file)
        else:
            print(f"ファイル '{filename}' からパラメータを抽出できませんでした。スキップします。")
            failed_param_files.append(file)

    if not grouped_files:
        print("有効なパラメータで分類されたファイルがありません。終了します。")
        if failed_param_files:
            print(f"\n[失敗] パラメータ抽出: {len(failed_param_files)} 件")
            for f in failed_param_files:
                print(f"  {f}  | 理由: ParameterRegexMismatch")
        else:
            print("\nパラメータ抽出に失敗したファイルはありませんでした。")
        if failed_data_files:
            print(f"\n[失敗] データ読み込み/抽出: {len(failed_data_files)} 件")
            for f, reason in failed_data_files:
                print(f"  {f}  | 理由: {reason}")
        sys.exit(1)

    print(f"分類されたパラメータの数: {len(grouped_files)}")

    # パラメータでソート
    sorted_parameters = sorted(grouped_files.keys(), key=lambda x: (x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]))
    print(f"ソートされたパラメータ順: {sorted_parameters}")

    """
    "tilt0deg_fold15deg": [124.45, 17.182, 0.6627],
    "tilt8deg_fold15deg": [127.1, 15.612, 0.6906], #127.1x2 + 15.612x + 0.6906
    "tilt15deg_fold15deg": [107.09, 18.039, 0.5855], #107.09x2 + 18.039x + 0.5855
    "tilt30deg_fold15deg": [92.596, 17.961, 0.5213], #92.596x2 + 17.961x + 0.5213
    """

    thrust_coefs = {
        (0, 0, 3.7): [117.9, 21.811, 0.5403],
        (15, 0, 3.7): [113.8, 22.766, 0.6355],
        (30, 0, 3.7): [97.806, 21.468, 0.5682],
        (0, 0, 2.7): [119.92, 18.022, 0.5599],
        (15, 0, 2.7): [117.5, 18.492, 0.5346],
        (30, 0, 2.7): [91.475, 21.633, 0.4504],
        (0, 15, 2.7): [124.45, 17.182, 0.6627],
        (8, 15, 2.7): [127.1, 15.612, 0.6906],
        (15, 15, 2.7): [107.09, 18.039, 0.5855],
        (30, 15, 2.7): [92.596, 17.961, 0.5213],
    }

    # 結合用データを格納するリスト
    combined_data = []

    # 各グループの処理
    for (distance, tilt_angle, fold_angle, prop_spacing, keyword, height, wall_spacing, flow_distance) in sorted_parameters:
        print(f"\nパラメータ: 距離={distance}, チルト角={tilt_angle}, 折りたたみ角={fold_angle}, プロペラ間隔={prop_spacing}, キーワード={keyword}, 高さ={height}, 壁間隔={wall_spacing}, 流体距離={flow_distance}")
        files = grouped_files[(distance, tilt_angle, fold_angle, prop_spacing, keyword, height, wall_spacing, flow_distance)]

        combined_data_group = []
        for file in files:
            print(f"  処理中のファイル: {file}")
            df, err_reason = read_and_extract_data(file)
            if df is None or (hasattr(df, 'empty') and df.empty):
                print(f"  ファイル {file} の読み込みまたは抽出に失敗しました。スキップします。")
                failed_data_files.append((file, err_reason or "EmptyDataFrame"))
                continue

            try:
                df_processed = df.copy()

                # 任意列が存在しない場合は欠損列を作成して後段集計を安定化
                optional_cols = ['seven_segment_value_0', 'seven_segment_value_1', 'seven_segment_value_2', 'seven_segment_value_3']
                for col in optional_cols:
                    if col not in df_processed.columns:
                        df_processed[col] = np.nan

                # seven_segment に名前を付与（"back" を含む場合は front/rear を入れ替え）
                if 'back' in str(keyword).lower():
                    name_map = {
                        'front_in': 'seven_segment_value_3',
                        'front_out': 'seven_segment_value_2',
                        'rear_out': 'seven_segment_value_1',
                        'rear_in': 'seven_segment_value_0',
                    }
                else:
                    name_map = {
                        'front_in': 'seven_segment_value_0',
                        'front_out': 'seven_segment_value_1',
                        'rear_out': 'seven_segment_value_2',
                        'rear_in': 'seven_segment_value_3',
                    }
                for new_name, src_col in name_map.items():
                    df_processed[new_name] = df_processed[src_col]

                # 先頭スキップ（time列から計算した time_elapsed を使用）
                if hasattr(args, 'skip_seconds') and args.skip_seconds > 0:
                    df_processed = df_processed[df_processed['time_elapsed'] >= args.skip_seconds]

                if hasattr(args, 'step_warmup') and args.step_warmup > 0:
                    df_processed = df_processed[df_processed['step_elapsed_time'] > args.step_warmup]
                
                # プレフィックスを追加
                key = (tilt_angle, fold_angle, prop_spacing)
                if not key in thrust_coefs:
                    print(f"  thrust_coefsにキー {key} が存在しません。近いキーを探します。")
                    key_dist = float('inf')
                    for k in thrust_coefs.keys():
                        dist = np.linalg.norm(np.array(k) - np.array(key))
                        if dist < key_dist:
                            key_dist = dist
                            key = k
                    print(f"  近いキー: {key}")
                else:
                    print(f"  キー {key} が見つかりました。")

                coefs = thrust_coefs[key]
                df_processed.loc[:, 'target_thrust'] = (
                    df_processed['control'] ** 2 * coefs[0] +
                    df_processed['control'] * coefs[1] +
                    coefs[2]
                )
                
                # 必要な列とパラメータを追加
                df_processed.loc[:, 'distance'] = distance
                df_processed.loc[:, 'tilt_angle'] = tilt_angle
                df_processed.loc[:, 'fold_angle'] = fold_angle
                df_processed.loc[:, 'prop_spacing'] = prop_spacing
                df_processed.loc[:, 'keyword'] = keyword
                df_processed.loc[:, 'height'] = height
                df_processed.loc[:, 'wall_spacing'] = wall_spacing
                df_processed.loc[:, 'flow_distance'] = flow_distance
                for col in ['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z', 'seven_segment_value_0', 'seven_segment_value_1', 'seven_segment_value_2', 'seven_segment_value_3', 'front_in', 'front_out', 'rear_out', 'rear_in']:
                    df_processed[f"{col}_partial_variance"] = df_processed.groupby('target_thrust')[col].transform("var")

                combined_data_group.append(df_processed)
            except Exception as e:
                print(f"  ファイル {file} の処理で例外が発生しました。スキップします。理由: {e}")
                failed_data_files.append((file, f"ProcessError: {e}"))

        if not combined_data_group:
            print(f"  パラメータグループ (距離={distance}, 角度={tilt_angle}, 折曲={fold_angle}, プロペラ間隔={prop_spacing}, キーワード={keyword}, 壁間隔={wall_spacing}, 流体距離={flow_distance}) に有効なデータがありません。")
            continue

        print(combined_data_group[0].head())

        # すべてのファイルからのデータを結合
        concatenated_group = pd.concat(combined_data_group, ignore_index=True)

        # target_thrustでグループ化して統計量を計算
        grouped_stats = concatenated_group.groupby('target_thrust').agg(
            sample_count=('target_thrust', 'count'),
            control=('control', 'median'),
            force_x=('force_x', 'median'),
            force_y=('force_y', 'median'),
            force_z=('force_z', 'median'),
            torque_x=('torque_x', 'median'),
            torque_y=('torque_y', 'median'),
            torque_z=('torque_z', 'median'),
            seven_segment_value_0=('seven_segment_value_0', 'median'),
            seven_segment_value_1=('seven_segment_value_1', 'median'),
            seven_segment_value_2=('seven_segment_value_2', 'median'),
            seven_segment_value_3=('seven_segment_value_3', 'median'),
            front_in=('front_in', 'median'),
            front_out=('front_out', 'median'),
            rear_out=('rear_out', 'median'),
            rear_in=('rear_in', 'median'),
            variance_force_x=('force_x_partial_variance', 'median'),
            variance_force_y=('force_y_partial_variance', 'median'),
            variance_force_z=('force_z_partial_variance', 'median'),
            variance_torque_x=('torque_x_partial_variance', 'median'),
            variance_torque_y=('torque_y_partial_variance', 'median'),
            variance_torque_z=('torque_z_partial_variance', 'median'),
            variance_seven_segment_value_0=('seven_segment_value_0_partial_variance', 'median'),
            variance_seven_segment_value_1=('seven_segment_value_1_partial_variance', 'median'),
            variance_seven_segment_value_2=('seven_segment_value_2_partial_variance', 'median'),
            variance_seven_segment_value_3=('seven_segment_value_3_partial_variance', 'median'),
            variance_front_in=('front_in_partial_variance', 'median'),
            variance_front_out=('front_out_partial_variance', 'median'),
            variance_rear_out=('rear_out_partial_variance', 'median'),
            variance_rear_in=('rear_in_partial_variance', 'median')
        ).reset_index()

        # パラメータ情報を追加
        grouped_stats['distance'] = distance
        grouped_stats['tilt_angle'] = tilt_angle
        grouped_stats['fold_angle'] = fold_angle
        grouped_stats['prop_spacing'] = prop_spacing
        grouped_stats['keyword'] = keyword
        grouped_stats['height'] = height
        grouped_stats['wall_spacing'] = wall_spacing
        grouped_stats['flow_distance'] = flow_distance
        combined_data.append(grouped_stats)

    # すべてのパラメータのデータを1つのCSVにエクスポート
    if args.output and combined_data:
        combined_df = pd.concat(combined_data, ignore_index=True)
        combined_df = combined_df.sort_values(by=['distance', 'tilt_angle', 'fold_angle', 'prop_spacing', 'keyword', 'height', 'wall_spacing', 'flow_distance', 'target_thrust']).reset_index(drop=True)
        export_data_to_csv(combined_df, args.output)
    else:
        print("結合されたデータがありません。エクスポートをスキップします。")

    # 最後に、失敗したファイルの一覧を表示（原因付き）
    if failed_param_files:
        print(f"\n[失敗] パラメータ抽出: {len(failed_param_files)} 件")
        for f in failed_param_files:
            print(f"  {f}  | 理由: ParameterRegexMismatch")
    else:
        print("\nパラメータ抽出に失敗したファイルはありませんでした。")

    if failed_data_files:
        print(f"\n[失敗] データ読み込み/抽出: {len(failed_data_files)} 件")
        for f, reason in failed_data_files:
            print(f"  {f}  | 理由: {reason}")
    else:
        print("\nデータ読み込み/抽出に失敗したファイルはありませんでした。")

if __name__ == "__main__":
    main()