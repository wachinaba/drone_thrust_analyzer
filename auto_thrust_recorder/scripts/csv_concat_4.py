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

def _parse_param_renames(rename_args):
    """--param-rename で与えられた 'old:new' の配列を辞書に変換"""
    rename_map = {}
    if not rename_args:
        return rename_map
    for item in rename_args:
        if not isinstance(item, str) or ':' not in item:
            continue
        old, new = item.split(':', 1)
        old = old.strip()
        new = new.strip()
        if old and new:
            rename_map[old.lower()] = new
    return rename_map

def _coerce_value(val_str):
    """単位表記などを取り除き、数値に変換できれば int/float 化する。"""
    if val_str is None:
        return None
    s = str(val_str)
    s = re.sub(r'\[[^\]]+\]', '', s)  # 単位表記 [deg], [mm], [R] などを除去
    s = s.strip()
    # 数値判定（整数/浮動小数）
    if re.fullmatch(r'-?\d+', s):
        try:
            return int(s)
        except Exception:
            pass
    if re.fullmatch(r'-?\d+(?:\.\d+)?', s):
        try:
            return float(s)
        except Exception:
            pass
    return s.lower()

def _strip_trailing_keyword_timestamp(base_filename, tail_keywords):
    """末尾の '_<kw>_<timestamp>.csv' を取り除き、timestamp を返す。
    例: '..._raw_20251112-153045.csv' -> ('... .csv', '20251112-153045')
    """
    if not tail_keywords:
        return base_filename, None
    for kw in tail_keywords:
        if not kw:
            continue
        kw_esc = re.escape(str(kw))
        # タイムスタンプ: YYYYMMDD, または YYYYMMDD[ -_ ]HHMMSS
        pattern = rf'^(?P<prefix>.*)_{kw_esc}_(?P<ts>\d{{8}}(?:[-_]?\d{{6}})?)\.csv$'
        m = re.match(pattern, base_filename, re.IGNORECASE)
        if m:
            prefix = m.group('prefix')
            ts = m.group('ts')
            return f"{prefix}.csv", ts
    return base_filename, None

def extract_parameters_generic(filename, param_rename_user=None, tail_keywords=None):
    """ファイル名から 'key=value' を汎用に抽出する。
    値中の '_' を許容し、次の '_key=' または拡張子/終端までを値として取得。
    抽出後、デフォルト＋ユーザー指定のリネームを適用し、値は可能なら数値化。
    """
    # ベース名のみ対象
    base = os.path.basename(filename)
    # 末尾の '_<kw>_<timestamp>.csv' を除去してから解析
    base, tail_ts = _strip_trailing_keyword_timestamp(base, tail_keywords)
    # 'key=value' を非貪欲に抽出（値中の '_' を許容）
    pattern = r'(?P<key>[A-Za-z][A-Za-z0-9]*)=(?P<value>.*?)(?=_(?:[A-Za-z][A-Za-z0-9]*)=|\.csv$|$)'
    matches = re.finditer(pattern, base)

    # 既定の自動リネーム
    default_rename = {
        'tilt': 'tilt_angle',
        'fold': 'fold_angle',
        'wheelbase': 'prop_spacing',
        'direction': 'keyword',
        'wallspacing': 'wall_spacing',
        'flowdistance': 'flow_distance',
    }
    # ユーザー指定があれば上書き
    user_map = _parse_param_renames(param_rename_user)
    rename_map = {**default_rename, **user_map}

    params = {}
    for m in matches:
        key = m.group('key')
        value = m.group('value')
        if not key:
            continue
        key_norm = key.lower()
        key_final = rename_map.get(key_norm, key_norm)
        params[key_final] = _coerce_value(value)

    # 抽出対象からは外すが CSV には残すため timestamp を別キーで保持
    if tail_ts is not None:
        params['file_timestamp'] = tail_ts

    return params if params else None

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
    """後方互換ラッパ（新しい汎用抽出器を使用）"""
    return extract_parameters_generic(filename)

def read_and_extract_data(file_path, dropna_mode='any', dropna_subset=None):
    """CSVファイルを読み込み、必要なカラムを抽出する関数。

    dropna_mode: 'any'（デフォルト）, 'all', 'none'
    dropna_subset: dropna 対象列名のリスト（None の場合は全列）

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
        if dropna_mode == 'none':
            extracted_df = df.copy()
        else:
            dropna_kwargs = {'how': dropna_mode}
            if dropna_subset:
                if isinstance(dropna_subset, str):
                    dropna_subset = [dropna_subset]
                dropna_kwargs['subset'] = dropna_subset
            extracted_df = df.dropna(**dropna_kwargs)
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
    parser.add_argument('--agg', type=str, choices=['median', 'mean'], default='median', help="集計関数を選択（median/mean、デフォルト: median）")
    parser.add_argument('--iqr-filter', action='store_true', help="IQRによる外れ値除去を有効化（グループ単位）")
    parser.add_argument('--iqr-multiplier', type=float, default=1.5, help="IQRウィスカー係数（デフォルト: 1.5）")
    parser.add_argument('--iqr-columns', nargs='+', type=str, default=['force_x','force_y','force_z','torque_x','torque_y','torque_z'], help="IQR外れ値判定の対象カラム群")
    parser.add_argument('--iqr-mode', type=str, choices=['any', 'all'], default='any', help="外れ値結合規則（any=いずれか外れ値で除去 / all=全て外れ値で除去）")
    parser.add_argument('--dropna-mode', type=str, choices=['any', 'all', 'none'], default='any', help="dropnaのモード（any/all/none、デフォルト: any）")
    parser.add_argument('--dropna-subset', nargs='+', type=str, default=None, help="dropnaを適用する列名のリスト（指定しない場合は全列）")
    parser.add_argument('--param-rename', action='append', default=[], help="パラメータ名のリネーム規則 'old:new' を複数指定可")
    parser.add_argument('--group-keys', nargs='+', type=str, default=['distance', 'tilt_angle', 'fold_angle', 'prop_spacing', 'keyword', 'height', 'wall_spacing', 'flow_distance'], help="グループ化に使用するパラメータ名の並び")
    return parser.parse_args()

def apply_iqr_filter(df, columns, multiplier=1.5, mode='any'):
    """target_thrust ごとに IQR で外れ値を除去した DataFrame を返す。

    columns: 判定対象カラム名のリスト
    multiplier: ウィスカー係数（通常1.5）
    mode: 'any' はいずれか列が外れ値なら除去、'all' は全列が外れ値のときのみ除去
    """
    if df is None or len(df) == 0:
        return df

    # 存在する列かつ数値列に限定
    valid_columns = [c for c in columns if c in df.columns]
    if not valid_columns:
        return df

    def filter_group(g):
        if g.empty:
            return g
        masks = []
        for c in valid_columns:
            s = g[c]
            try:
                q1 = s.quantile(0.25)
                q3 = s.quantile(0.75)
            except Exception:
                masks.append(pd.Series([True] * len(g), index=g.index))
                continue
            iqr = q3 - q1
            if pd.isna(q1) or pd.isna(q3) or iqr == 0:
                masks.append(pd.Series([True] * len(g), index=g.index))
                continue
            lower = q1 - multiplier * iqr
            upper = q3 + multiplier * iqr
            masks.append((s >= lower) & (s <= upper))

        if not masks:
            return g

        inlier_mask = masks[0]
        if mode == 'any':
            # いずれか列が外れ値 -> 除去 => 全列が範囲内のみ残す（AND）
            for m in masks[1:]:
                inlier_mask = inlier_mask & m
        else:
            # 全列が外れ値 -> 除去 => いずれか列が範囲内なら残す（OR）
            for m in masks[1:]:
                inlier_mask = inlier_mask | m

        return g.loc[inlier_mask]

    return df.groupby('target_thrust', group_keys=False).apply(filter_group)

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
    file_params_map = {}
    group_keys = [k.lower() for k in getattr(args, 'group_keys', [])]
    for file in csv_files:
        filename = os.path.basename(file)
        params = extract_parameters_generic(
            filename,
            getattr(args, 'param_rename', []),
            getattr(args, 'keywords', [])
        )
        if params:
            # グループ化キーの存在チェック
            missing_keys = [k for k in group_keys if k not in params]
            if missing_keys:
                print(f"ファイル '{filename}' のパラメータに必要キーが不足しています。スキップします。不足: {missing_keys}")
                failed_param_files.append(file)
                continue
            group_key_tuple = tuple(params[k] for k in group_keys)
            grouped_files[group_key_tuple].append(file)
            file_params_map[file] = params
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
    sorted_parameters = sorted(grouped_files.keys())
    print(f"ソートされたパラメータ順（group-keys順）: {sorted_parameters}")

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
    for param_tuple in sorted_parameters:
        group_params = dict(zip(group_keys, param_tuple))
        print(f"\nパラメータグループ: {group_params}")
        files = grouped_files[param_tuple]

        combined_data_group = []
        for file in files:
            print(f"  処理中のファイル: {file}")
            df, err_reason = read_and_extract_data(
                file,
                dropna_mode=getattr(args, 'dropna_mode', 'any'),
                dropna_subset=getattr(args, 'dropna_subset', None)
            )
            if df is None or (hasattr(df, 'empty') and df.empty):
                print(f"  ファイル {file} の読み込みまたは抽出に失敗しました。スキップします。")
                failed_data_files.append((file, err_reason or "EmptyDataFrame"))
                continue

            try:
                df_processed = df.copy()
                file_params = file_params_map.get(file, {})

                # 風速の新形式のみを扱う（旧列からの自動変換は行わない）
                for col in ['front_in', 'front_out', 'rear_out', 'rear_in']:
                    if col not in df_processed.columns:
                        df_processed[col] = np.nan

                # 先頭スキップ（time列から計算した time_elapsed を使用）
                if hasattr(args, 'skip_seconds') and args.skip_seconds > 0:
                    df_processed = df_processed[df_processed['time_elapsed'] >= args.skip_seconds]

                if hasattr(args, 'step_warmup') and args.step_warmup > 0:
                    df_processed = df_processed[df_processed['step_elapsed_time'] > args.step_warmup]
                
                # プレフィックスを追加
                tilt_val = file_params.get('tilt_angle', group_params.get('tilt_angle'))
                fold_val = file_params.get('fold_angle', group_params.get('fold_angle'))
                prop_val = file_params.get('prop_spacing', group_params.get('prop_spacing'))
                key = (tilt_val, fold_val, prop_val)
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
                
                # 抽出したパラメータ列を付与（ファイルごと）
                for p_key, p_val in file_params.items():
                    df_processed.loc[:, p_key] = p_val
                # file_timestamp 列が無い場合も列を確保
                if 'file_timestamp' not in df_processed.columns:
                    df_processed['file_timestamp'] = np.nan

                for col in ['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z', 'front_in', 'front_out', 'rear_out', 'rear_in']:
                    df_processed[f"{col}_partial_variance"] = df_processed.groupby('target_thrust')[col].transform("var")

                combined_data_group.append(df_processed)
            except Exception as e:
                print(f"  ファイル {file} の処理で例外が発生しました。スキップします。理由: {e}")
                failed_data_files.append((file, f"ProcessError: {e}"))

        if not combined_data_group:
            print(f"  パラメータグループ {group_params} に有効なデータがありません。")
            continue

        print(combined_data_group[0].head())

        # すべてのファイルからのデータを結合
        concatenated_group = pd.concat(combined_data_group, ignore_index=True)

        # IQRフィルタ（有効な場合）: groupby前に target_thrust ごとで実施
        if hasattr(args, 'iqr_filter') and args.iqr_filter:
            concatenated_group = apply_iqr_filter(
                concatenated_group,
                columns=getattr(args, 'iqr_columns', ['force_x','force_y','force_z','torque_x','torque_y','torque_z']),
                multiplier=getattr(args, 'iqr_multiplier', 1.5),
                mode=getattr(args, 'iqr_mode', 'any')
            )

        # target_thrustでグループ化して統計量を計算（agg切替）
        agg_func = getattr(args, 'agg', 'median')
        grouped_stats = concatenated_group.groupby('target_thrust').agg(
            sample_count=('target_thrust', 'count'),
            control=('control', agg_func),
            force_x=('force_x', agg_func),
            force_y=('force_y', agg_func),
            force_z=('force_z', agg_func),
            torque_x=('torque_x', agg_func),
            torque_y=('torque_y', agg_func),
            torque_z=('torque_z', agg_func),
            front_in=('front_in', agg_func),
            front_out=('front_out', agg_func),
            rear_out=('rear_out', agg_func),
            rear_in=('rear_in', agg_func),
            file_timestamp=('file_timestamp', 'min'),
            variance_force_x=('force_x_partial_variance', agg_func),
            variance_force_y=('force_y_partial_variance', agg_func),
            variance_force_z=('force_z_partial_variance', agg_func),
            variance_torque_x=('torque_x_partial_variance', agg_func),
            variance_torque_y=('torque_y_partial_variance', agg_func),
            variance_torque_z=('torque_z_partial_variance', agg_func),
            variance_front_in=('front_in_partial_variance', agg_func),
            variance_front_out=('front_out_partial_variance', agg_func),
            variance_rear_out=('rear_out_partial_variance', agg_func),
            variance_rear_in=('rear_in_partial_variance', agg_func)
        ).reset_index()

        # パラメータ情報を追加
        for k, v in group_params.items():
            grouped_stats[k] = v
        combined_data.append(grouped_stats)

    # すべてのパラメータのデータを1つのCSVにエクスポート
    if args.output and combined_data:
        combined_df = pd.concat(combined_data, ignore_index=True)
        sort_columns = group_keys + ['target_thrust']
        combined_df = combined_df.sort_values(by=sort_columns).reset_index(drop=True)
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