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

def _iter_progress(iterable, *, total=None, desc=None, leave=True, progress: str = "auto"):
    """tqdm が利用可能なら進捗表示付きで iterable を返す。無ければそのまま返す。

    progress:
      - 'auto': stderr が TTY のときだけ tqdm（推奨）
      - 'on'  : 常に tqdm を試みる（tqdm が無ければ無効）
      - 'off' : tqdm を使わない
    """
    mode = (progress or "auto").lower()
    if mode == "off":
        return iterable
    if mode == "auto" and not sys.stderr.isatty():
        return iterable
    try:
        from tqdm.auto import tqdm  # type: ignore
    except Exception:
        return iterable
    return tqdm(iterable, total=total, desc=desc, leave=leave)

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

def _parse_column_defaults(default_args):
    """--default-column で与えられた 'col=value' を辞書に変換する。

    value は _coerce_value により int/float 化を試みる。
    'nan'/'none'/'null'（大文字小文字不問）は np.nan として扱う。
    """
    defaults = {}
    if not default_args:
        return defaults
    for item in default_args:
        if not isinstance(item, str) or '=' not in item:
            continue
        col, val = item.split('=', 1)
        col = col.strip()
        val = val.strip()
        if not col:
            continue
        if val.lower() in ('nan', 'none', 'null'):
            defaults[col] = np.nan
        else:
            defaults[col] = _coerce_value(val)
    return defaults

def _apply_column_defaults(df, defaults, mode='missing'):
    """DataFrame に列デフォルト値を適用する。

    - mode='missing': 列が存在しない場合のみ作成してデフォルトで埋める
    - mode='na':      列が存在する場合のみ NaN をデフォルトで埋める（列未存在は作らない）
    - mode='both':    列未存在なら作成、存在する列は NaN を埋める
    """
    if df is None or not hasattr(df, 'columns'):
        return df
    if not defaults:
        return df
    mode = (mode or 'missing').lower()
    for col, default_val in defaults.items():
        exists = col in df.columns
        if (not exists) and mode in ('missing', 'both'):
            df.loc[:, col] = default_val
        elif exists and mode in ('na', 'both'):
            try:
                df.loc[:, col] = df[col].fillna(default_val)
            except Exception:
                # dtype 不整合などで失敗したら安全にスキップ
                pass
    return df

def _strip_trailing_keyword_timestamp(base_filename, tail_keywords):
    """末尾の '_<kw>_...' セグメントを取り除き、末尾に存在するタイムスタンプを返す。
    ・キーワード例: 'biascorr' → '_biascorr_20251113-211756_20251114-105148.csv' を全て除去
    ・タイムスタンプ形式: YYYYMMDD または YYYYMMDD[ -_ ]HHMMSS（末尾に複数あれば最後を採用）
    例: '..._biascorr_20251113-211756_20251114-105148.csv' -> ('....csv', '20251114-105148')
    """
    if not tail_keywords:
        return base_filename, None
    for kw in tail_keywords:
        if not kw:
            continue
        kw_esc = re.escape(str(kw))
        # 末尾が '_<kw>' で始まる任意のセグメント + '.csv' にマッチ
        m = re.search(rf'_(?:{kw_esc})(?:_[^.]+)?\.csv$', base_filename, re.IGNORECASE)
        if m:
            prefix = base_filename[:m.start()]
            # '.csv' を除いた末尾部からタイムスタンプ候補を全抽出し、最後を採用
            suffix_no_ext = base_filename[m.start():-4]
            ts_candidates = re.findall(r'(\d{{8}}(?:[-_]\d{{6}})?)', suffix_no_ext)
            ts = ts_candidates[-1] if ts_candidates else None
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
        # abbreviations (default)
        'd': 'distance',
        'dir': 'direction',
        'f': 'fold_angle',
        'fdir': 'flow_direction',
        'fdst': 'flow_distance',
        'fhgt': 'fow_height',
        'h': 'height',
        'ps': 'prop_spacing',
        's': 'slant_angle',
        't': 'tilt_angle',
        'wb': 'wheelbase',
        'ws': 'wall_spacing',
        # aliases / backward-compat
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

def _detect_numeric_measurement_columns(df, exclude_columns=None):
    """プレフィックスに依存せず、数値として扱える測定列を自動検出する。
    除外対象（meta/補助列/パラメータ列など）は exclude_columns で指定。
    """
    if df is None or len(df) == 0:
        return []
    exclude = set(exclude_columns or [])
    # 常に除外する基本メタ列
    exclude.update({'time', 'time_elapsed', 'target_thrust', 'control', 'file_timestamp'})
    numeric_cols = []
    for col in df.columns:
        if col in exclude:
            continue
        name = str(col)
        # 補助列や派生列は除外
        if (
            name.endswith('_partial_variance') or
            name.endswith('_prev') or
            name.endswith('_step') or
            name.endswith('_start_time') or
            name.endswith('_elapsed_time') or
            'increase' in name
        ):
            continue
        coerced = pd.to_numeric(df[col], errors='coerce')
        if coerced.notna().sum() > 0:
            numeric_cols.append(col)
    return numeric_cols

def read_and_extract_data(file_path, dropna_mode='any', dropna_subset=None, sensor_bias=True):
    """CSVファイルを読み込み、必要なカラムを抽出する関数。

    dropna_mode: 'any'（デフォルト）, 'all', 'none'
    dropna_subset: dropna 対象列名のリスト（None の場合は全列）
    sensor_bias: True の場合、低制御時の先頭行を基準にセンサバイアス補正を試みる

    戻り値: (DataFrame | None, エラー理由文字列 | None)
    """
    REQUIRED_COLUMNS = ['time', 'control', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z', 'target_thrust']
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
            extracted_df = df.dropna(**dropna_kwargs).copy()
        if extracted_df.empty:
            return None, "EmptyDataAfterDropNA"

        missing = [c for c in REQUIRED_COLUMNS if c not in extracted_df.columns]
        if missing:
            return None, f"MissingColumns: {','.join(missing)}"

        # センサバイアス補正（オプション）
        if sensor_bias:
            # センサバイアス除去のための安全な先頭行参照
            if extracted_df.iloc[0]['control'] < 0.1:
                first_row = extracted_df.iloc[0].copy()
                force_norm = np.linalg.norm(first_row[['force_x', 'force_y', 'force_z']])
                torque_norm = np.linalg.norm(first_row[['torque_x', 'torque_y', 'torque_z']])
                if force_norm > 1.0 or torque_norm > 1.0:
                    print(f"Sensor bias is too high: {force_norm}, {torque_norm}")
                    extracted_df.loc[:, 'force_x'] = extracted_df['force_x'] - first_row['force_x']
                    extracted_df.loc[:, 'force_y'] = extracted_df['force_y'] - first_row['force_y']
                    extracted_df.loc[:, 'force_z'] = extracted_df['force_z'] - first_row['force_z']
                    extracted_df.loc[:, 'torque_x'] = extracted_df['torque_x'] - first_row['torque_x']
                    extracted_df.loc[:, 'torque_y'] = extracted_df['torque_y'] - first_row['torque_y']
                    extracted_df.loc[:, 'torque_z'] = extracted_df['torque_z'] - first_row['torque_z']

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
    parser.add_argument('--drop-zero-columns', nargs='+', type=str, default=None, help="指定した列で値が0のセルをNaNとして扱い、IQR判定および集計で無視する")
    parser.add_argument('--dropna-mode', type=str, choices=['any', 'all', 'none'], default='any', help="dropnaのモード（any/all/none、デフォルト: any）")
    parser.add_argument('--dropna-subset', nargs='+', type=str, default=None, help="dropnaを適用する列名のリスト（指定しない場合は全列）")
    parser.add_argument('--default-column', action='append', default=[],
                        help="列のデフォルト値を指定 'col=value'（複数指定可）。例: --default-column front_in=0 --default-column rear_in=0")
    parser.add_argument('--default-column-mode', type=str, choices=['missing', 'na', 'both'], default='missing',
                        help="デフォルト値の適用モード（missing=列が無い時だけ作成, na=既存列のNaNだけ埋める, both=両方）")
    parser.add_argument('--param-rename', action='append', default=[], help="パラメータ名のリネーム規則 'old:new' を複数指定可")
    parser.add_argument('--group-keys', nargs='+', type=str, default=['auto'], help="グループ化に使用するパラメータ名の並び（デフォルト: auto）")
    parser.add_argument('--sensor-bias', type=str, choices=['on', 'off'], default='on', help="センサバイアス補正を有効/無効化（デフォルト: on）")
    parser.add_argument(
        "--progress",
        type=str,
        choices=["auto", "on", "off"],
        default="auto",
        help="tqdm による進捗表示（auto=TTYのみ, on=常に, off=無効）。tqdm 未インストールなら自動で無効化。",
    )
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
    column_defaults = _parse_column_defaults(getattr(args, 'default_column', []))
    column_default_mode = getattr(args, 'default_column_mode', 'missing')

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
    is_auto_group_keys = (len(group_keys) == 1 and group_keys[0] == 'auto')
    param_keys_union = set()
    # 第1段階: 全ファイルからパラメータだけ収集（auto の場合はここで union を作る）
    for file in _iter_progress(
        csv_files,
        total=len(csv_files),
        desc="csv_concat_4: scan params",
        progress=getattr(args, "progress", "auto"),
    ):
        filename = os.path.basename(file)
        params = extract_parameters_generic(
            filename,
            getattr(args, 'param_rename', []),
            getattr(args, 'keywords', [])
        ) or {}
        file_params_map[file] = params
        if is_auto_group_keys:
            param_keys_union.update(params.keys())
    # auto の場合は union をグルーピングキーに採用（file_timestamp は除外）
    if is_auto_group_keys:
        if 'file_timestamp' in param_keys_union:
            param_keys_union.remove('file_timestamp')
        group_keys = sorted(param_keys_union)
    # 第2段階: グルーピングキーに基づいて分類（auto 以外は従来通り不足キーでスキップ）
    items = list(file_params_map.items())
    for file, params in _iter_progress(
        items,
        total=len(items),
        desc="csv_concat_4: group files",
        progress=getattr(args, "progress", "auto"),
    ):
        filename = os.path.basename(file)
        if not is_auto_group_keys:
            if not params:
                print(f"ファイル '{filename}' からパラメータを抽出できませんでした。スキップします。")
                failed_param_files.append(file)
                continue
            missing_keys = [k for k in group_keys if k not in params]
            if missing_keys:
                print(f"ファイル '{filename}' のパラメータに必要キーが不足しています。スキップします。不足: {missing_keys}")
                failed_param_files.append(file)
                continue
            group_key_tuple = tuple(params[k] for k in group_keys)
        else:
            # auto: 欠損は NaN で埋めてグループキー生成
            group_key_tuple = tuple(params.get(k, np.nan) for k in group_keys)
        grouped_files[group_key_tuple].append(file)

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
    

    # 結合用データを格納するリスト
    combined_data = []

    # 各グループの処理
    for param_tuple in _iter_progress(
        sorted_parameters,
        total=len(sorted_parameters),
        desc="csv_concat_4: process groups",
        progress=getattr(args, "progress", "auto"),
    ):
        group_params = dict(zip(group_keys, param_tuple))
        print(f"\nパラメータグループ: {group_params}")
        files = grouped_files[param_tuple]

        combined_data_group = []
        for file in files:
            print(f"  処理中のファイル: {file}")
            df, err_reason = read_and_extract_data(
                file,
                dropna_mode=getattr(args, 'dropna_mode', 'any'),
                dropna_subset=getattr(args, 'dropna_subset', None),
                sensor_bias=(getattr(args, 'sensor_bias', 'on') == 'on')
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

                # 任意列のデフォルト値を適用（列が無い場合の作成や NaN 埋め）
                if column_defaults:
                    df_processed = _apply_column_defaults(df_processed, column_defaults, mode=column_default_mode)

                # 先頭スキップ（time列から計算した time_elapsed を使用）
                if hasattr(args, 'skip_seconds') and args.skip_seconds > 0:
                    df_processed = df_processed[df_processed['time_elapsed'] >= args.skip_seconds]

                # ウォームアップ除去は target_thrust 切替ベースで後段で実施
                
                # CSV の target_thrust 列を使用（数値化し、NaN は除外）
                if 'target_thrust' not in df_processed.columns:
                    raise ValueError("target_thrust column not found in CSV")
                df_processed['target_thrust'] = pd.to_numeric(df_processed['target_thrust'], errors='coerce')
                df_processed = df_processed.dropna(subset=['target_thrust'])
                
                # target_thrust の切り替わりをステップと見做してウォームアップ除去
                if hasattr(args, 'step_warmup') and args.step_warmup > 0:
                    df_processed['target_thrust_prev'] = df_processed['target_thrust'].shift(1)
                    df_processed['thrust_change'] = df_processed['target_thrust'] != df_processed['target_thrust_prev']
                    df_processed['thrust_step'] = df_processed['thrust_change'].cumsum()
                    df_processed['thrust_step_start_time'] = df_processed.groupby('thrust_step')['time_elapsed'].transform('first')
                    df_processed['thrust_step_elapsed_time'] = df_processed['time_elapsed'] - df_processed['thrust_step_start_time']
                    df_processed = df_processed[df_processed['thrust_step_elapsed_time'] > args.step_warmup]
                    # 後続で使わない補助列は除去
                    df_processed = df_processed.drop(columns=['target_thrust_prev', 'thrust_change', 'thrust_step', 'thrust_step_start_time', 'thrust_step_elapsed_time'], errors='ignore')
                
                # ウォームアップ/target_thrust フィルタ後に空ならスキップ
                if df_processed.empty:
                    print(f"  ファイル {file} は target_thrust/ウォームアップ後にデータが空です。スキップします。")
                    failed_data_files.append((file, "EmptyAfterTargetThrustOrWarmup"))
                    continue
                
                # 測定列（存在列のみ）を自動検出し、数値化してから部分分散を計算
                exclude_for_detect = set(file_params.keys()) | {'file_timestamp'}
                numeric_meas_cols = _detect_numeric_measurement_columns(df_processed, exclude_for_detect)
                # 数値化（変換可能な列のみ上書き）
                for c in numeric_meas_cols:
                    df_processed[c] = pd.to_numeric(df_processed[c], errors='coerce')
                
                # 抽出したパラメータ列を付与（ファイルごと）
                for p_key, p_val in file_params.items():
                    df_processed.loc[:, p_key] = p_val
                # file_timestamp 列が無い場合も列を確保
                if 'file_timestamp' not in df_processed.columns:
                    df_processed['file_timestamp'] = np.nan
                
                # 動的に推定した数値列のみ、部分分散を計算
                for col in numeric_meas_cols:
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

        # 行ごとのゼロ判定: 指定列で値が0のセルを NaN に変換（パラメータグループ単位）
        drop_zero_columns = getattr(args, 'drop_zero_columns', None)
        if drop_zero_columns:
            for col in drop_zero_columns:
                if col in concatenated_group.columns:
                    # 数値化して 0 判定し、該当セルのみ NaN 扱いにする
                    col_numeric = pd.to_numeric(concatenated_group[col], errors='coerce')
                    zero_mask = col_numeric == 0
                    if zero_mask.any():
                        concatenated_group.loc[zero_mask, col] = np.nan
                        # 対応する部分分散列があれば、同じ行を NaN にしておく
                        var_col = f"{col}_partial_variance"
                        if var_col in concatenated_group.columns:
                            concatenated_group.loc[zero_mask, var_col] = np.nan

        # IQRフィルタ（有効な場合）: CSVから存在する数値列のみ対象にして実施
        if hasattr(args, 'iqr_filter') and args.iqr_filter:
            # 当該グループのパラメータ列（除外対象）を union で収集
            iqr_param_keys_union = set()
            for f in files:
                iqr_param_keys_union.update(file_params_map.get(f, {}).keys())
            iqr_exclude = set(group_keys) | {'target_thrust', 'control', 'file_timestamp'} | iqr_param_keys_union
            numeric_meas_cols = _detect_numeric_measurement_columns(concatenated_group, iqr_exclude)
            # 数値化（変換可能な列のみ上書き）
            for c in numeric_meas_cols:
                concatenated_group[c] = pd.to_numeric(concatenated_group[c], errors='coerce')
            # デフォルトか明示指定かで iqr 対象列を決定
            default_iqr_cols = ['force_x','force_y','force_z','torque_x','torque_y','torque_z']
            user_iqr_cols = getattr(args, 'iqr_columns', default_iqr_cols)
            if user_iqr_cols == default_iqr_cols:
                iqr_cols = list(numeric_meas_cols)
            else:
                iqr_cols = [c for c in user_iqr_cols if c in numeric_meas_cols]
            concatenated_group = apply_iqr_filter(
                concatenated_group,
                columns=iqr_cols,
                multiplier=getattr(args, 'iqr_multiplier', 1.5),
                mode=getattr(args, 'iqr_mode', 'any')
            )

        # target_thrustでグループ化して統計量を計算（agg切替）
        agg_func = getattr(args, 'agg', 'median')
        # 集計対象列も CSVの存在・数値判定により動的に構成（自動検出）
        # このグループで検出された全パラメータ列（ファイル名からの抽出列）を union で集約
        param_keys_union = set()
        for f in files:
            param_keys_union.update(file_params_map.get(f, {}).keys())
        agg_exclude = set(group_keys) | {'target_thrust', 'control', 'file_timestamp'} | param_keys_union
        numeric_meas_cols = _detect_numeric_measurement_columns(concatenated_group, agg_exclude)
        # 数値化（変換可能な列のみ上書き）
        for c in numeric_meas_cols:
            concatenated_group[c] = pd.to_numeric(concatenated_group[c], errors='coerce')
        # 出力に含めるパラメータ列（group_keys や集計済の特別列は除外）
        exclude_param_cols = set(group_keys) | {'target_thrust', 'control'}
        param_cols_for_output = [p for p in sorted(param_keys_union) if p not in exclude_param_cols and p in concatenated_group.columns]
        agg_dict = {
            'sample_count': ('target_thrust', 'count'),
            'control': ('control', agg_func),
            'file_timestamp': ('file_timestamp', 'min')
        }
        for c in numeric_meas_cols:
            agg_dict[c] = (c, agg_func)
            var_col = f"{c}_partial_variance"
            if var_col in concatenated_group.columns:
                agg_dict[f"variance_{c}"] = (var_col, agg_func)
        # すべてのパラメータ列を 'first' で出力（グループ内で一定と仮定）
        for pcol in param_cols_for_output:
            if pcol not in agg_dict:
                agg_dict[pcol] = (pcol, 'first')
        grouped_stats = concatenated_group.groupby('target_thrust').agg(**agg_dict).reset_index()

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