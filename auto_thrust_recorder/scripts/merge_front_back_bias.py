import os
import sys
import re
import argparse
from pathlib import Path
from collections import defaultdict
from datetime import datetime
from typing import Dict, List, Tuple, Optional

import numpy as np
import pandas as pd


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
    base = os.path.basename(filename)
    base, tail_ts = _strip_trailing_keyword_timestamp(base, tail_keywords)
    pattern = r'(?P<key>[A-Za-z][A-Za-z0-9]*)=(?P<value>.*?)(?=_(?:[A-Za-z][A-Za-z0-9]*)=|\.csv$|$)'
    matches = re.finditer(pattern, base)

    # 既定の自動リネーム
    default_rename = {
        'tilt': 'tilt_angle',
        'fold': 'fold_angle',
        'wheelbase': 'prop_spacing',
        # 'keyword' or 'direction' を 'direction' に正規化
        'keyword': 'direction',
        'direction': 'direction',
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

    if tail_ts is not None:
        params['file_timestamp'] = tail_ts

    return params if params else None


def find_csv_files(keywords, directory='.', and_keywords=False):
    """CSVファイルをキーワードに基づいて再帰的に検索する関数 (Pathlibを使用)。"""
    directories = directory
    if not isinstance(directories, (list, tuple, set)):
        directories = [directories]

    files_set = set()
    for dir_path in directories:
        for keyword in keywords:
            search_pattern = f"*{keyword}*.csv"
            for file in Path(dir_path).rglob(search_pattern):
                files_set.add(str(file))

    files = sorted(files_set)
    if and_keywords:
        files = [file for file in files if all(keyword in file for keyword in keywords)]
    return files


def read_and_extract_data(file_path, dropna_mode='any', dropna_subset=None):
    """CSVファイルを読み込み、必要なカラムを抽出し、時間系補助列を付与する。
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

        # sort by time
        extracted_df = extracted_df.sort_values('time').reset_index(drop=True)

        # add time elapsed
        extracted_df['time_elapsed'] = extracted_df['time'] - extracted_df['time'].iloc[0]

        # add step indices based on control increase
        extracted_df['control_prev'] = extracted_df['control'].shift(1)
        extracted_df['control_increase'] = extracted_df['control'] > extracted_df['control_prev']
        extracted_df['control_increase_step'] = extracted_df['control_increase'].cumsum()
        extracted_df['step_start_time'] = extracted_df.groupby('control_increase_step')['time_elapsed'].transform('first')
        extracted_df['step_elapsed_time'] = extracted_df['time_elapsed'] - extracted_df['step_start_time']

        return extracted_df, None
    except Exception as e:
        return None, f"ProcessError: {e}"


def parse_arguments():
    parser = argparse.ArgumentParser(description="front/back_reversed を結合してバイアス補正列を追加するツール")
    parser.add_argument('-k', '--keywords', nargs='+', type=str, default=['front', 'back_reversed'], help="検索するファイル名に含まれるキーワード（デフォルト: front/back_reversed）")
    parser.add_argument('-d', '--directory', nargs='+', type=str, default=['.'], help="CSVファイルを検索するディレクトリ（複数指定可）")
    parser.add_argument('-a', '--and_keywords', action='store_true', help="AND条件でファイルを検索する")
    parser.add_argument('--output-dir', type=str, default='.', help="グループ単位の出力ディレクトリ")
    parser.add_argument('--tail-keyword', type=str, default='biascorr', help="出力ファイル末尾のキーワード（例: biascorr）")
    parser.add_argument('--dropna-mode', type=str, choices=['any', 'all', 'none'], default='any', help="dropnaのモード（any/all/none）")
    parser.add_argument('--dropna-subset', nargs='+', type=str, default=None, help="dropna対象列（未指定は全列）")
    parser.add_argument('--skip-seconds', type=float, default=0.0, help="各CSVの先頭から指定秒数をスキップ（time_elapsed基準）")
    parser.add_argument('--step-warmup', type=float, default=0.5, help="各ステップ立ち上がり時の除外秒数（0で無効）")
    parser.add_argument('--bias-agg', type=str, choices=['median', 'mean'], default='median', help="バイアス推定に使う集計関数")
    parser.add_argument('--bias-scope', type=str, choices=['global', 'per-step'], default='global', help="バイアス推定のスコープ")
    parser.add_argument('--param-rename', action='append', default=[], help="パラメータ名のリネーム規則 'old:new' を複数指定可（ファイル名抽出用）")
    # group-keys は後方互換のため残すが、実際のグルーピングは「direction/timestamp を除いた全パラメータ一致」で行う
    parser.add_argument('--group-keys', nargs='+', type=str, default=['distance', 'tilt_angle', 'fold_angle', 'prop_spacing', 'height', 'wall_spacing', 'flow_distance'], help="後方互換用（実際のグルーピングでは使用しません）")
    # bias列の出力: デフォルトON、--no-emit-bias-columns で無効化
    emit_group = parser.add_mutually_exclusive_group()
    emit_group.add_argument('--emit-bias-columns', dest='emit_bias_columns', action='store_true', help="bias_* 列の出力を有効化（デフォルト）")
    emit_group.add_argument('--no-emit-bias-columns', dest='emit_bias_columns', action='store_false', help="bias_* 列の出力を無効化")
    parser.set_defaults(emit_bias_columns=True)
    parser.add_argument(
        "--progress",
        type=str,
        choices=["auto", "on", "off"],
        default="auto",
        help="tqdm による進捗表示（auto=TTYのみ, on=常に, off=無効）。tqdm 未インストールなら自動で無効化。",
    )
    return parser.parse_args()


TARGET_COLUMNS = ['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']


def _aggregate(series: pd.Series, mode: str) -> float:
    if mode == 'mean':
        return float(series.mean())
    # default median with numeric conversion safety
    try:
        return float(series.median())
    except Exception:
        return float(np.median(pd.to_numeric(series, errors='coerce')))


def _compute_bias_global(df_front: pd.DataFrame, df_back: pd.DataFrame, agg: str) -> Dict[str, float]:
    bias = {}
    for col in TARGET_COLUMNS:
        fv = _aggregate(df_front[col], agg)
        bv = _aggregate(df_back[col], agg)
        bias[col] = (fv - bv) / 2.0
    return bias


def _compute_bias_per_step(df_front: pd.DataFrame, df_back: pd.DataFrame, agg: str) -> Tuple[Dict[int, Dict[str, float]], Dict[str, float]]:
    """ステップ毎のバイアスとグローバルのフォールバックを返す。"""
    global_bias = _compute_bias_global(df_front, df_back, agg)
    step_to_bias: Dict[int, Dict[str, float]] = {}
    front_steps = set(df_front['control_increase_step'].unique())
    back_steps = set(df_back['control_increase_step'].unique())
    common_steps = sorted(front_steps & back_steps)
    for step in common_steps:
        fstep = df_front[df_front['control_increase_step'] == step]
        bstep = df_back[df_back['control_increase_step'] == step]
        step_bias = {}
        for col in TARGET_COLUMNS:
            fv = _aggregate(fstep[col], agg)
            bv = _aggregate(bstep[col], agg)
            step_bias[col] = (fv - bv) / 2.0
        step_to_bias[step] = step_bias
    return step_to_bias, global_bias


def _apply_bias_corrected_columns(df: pd.DataFrame, direction: str, bias_map: Dict[str, float],
                                  step_bias_map: Optional[Dict[int, Dict[str, float]]] = None,
                                  global_bias_fallback: Optional[Dict[str, float]] = None,
                                  emit_bias_columns: bool = True) -> pd.DataFrame:
    """補正列(*_bias_corrected)を追加して返す。direction に応じて加算/減算。"""
    is_front = direction.lower() == 'front'
    # per-step の場合、各行ごとにステップに応じたバイアスを選択
    if step_bias_map is not None:
        step_series = df.get('control_increase_step', pd.Series([-1] * len(df), index=df.index))
        out = df.copy()
        for col in TARGET_COLUMNS:
            corrected_vals = []
            bias_vals = []
            for idx, val in df[col].items():
                step_id = int(step_series.loc[idx]) if pd.notna(step_series.loc[idx]) else -1
                b = None
                if step_id in step_bias_map:
                    b = step_bias_map[step_id].get(col, 0.0)
                elif global_bias_fallback is not None:
                    b = global_bias_fallback.get(col, 0.0)
                else:
                    b = bias_map.get(col, 0.0)
                corrected_vals.append(val - b if is_front else val + b)
                bias_vals.append(b)
            out[f"{col}_bias_corrected"] = corrected_vals
            if emit_bias_columns:
                out[f"bias_{col}"] = bias_vals
        return out

    # global の場合
    out = df.copy()
    for col in TARGET_COLUMNS:
        b = bias_map.get(col, 0.0)
        out[f"{col}_bias_corrected"] = out[col] - b if is_front else out[col] + b
        if emit_bias_columns:
            out[f"bias_{col}"] = b
    return out


def main():
    args = parse_arguments()

    # 検索
    csv_files = find_csv_files(args.keywords, args.directory, args.and_keywords)
    if not csv_files:
        print(f"キーワード '{args.keywords}' を含むCSVファイルが見つかりません。")
        sys.exit(1)

    print(f"見つかったファイル数: {len(csv_files)}")

    # グルーピング: direction と file_timestamp を除いた「全パラメータ一致」でグループ化
    grouped_files: Dict[Tuple, Dict[str, List[str]]] = defaultdict(lambda: {'front': [], 'back_reversed': []})
    file_params_map: Dict[str, Dict] = {}
    failed_param_files: List[str] = []

    for file in csv_files:
        filename = os.path.basename(file)
        params = extract_parameters_generic(
            filename,
            getattr(args, 'param_rename', []),
            getattr(args, 'keywords', [])
        )
        if not params:
            print(f"ファイル '{filename}' からパラメータを抽出できませんでした。スキップ。")
            failed_param_files.append(file)
            continue
        direction = str(params.get('direction', '')).lower()
        if direction not in ('front', 'back_reversed'):
            print(f"ファイル '{filename}' の direction が不正/欠落です（得た値: '{direction}'）。スキップ。")
            failed_param_files.append(file)
            continue
        # direction, file_timestamp を除いた全パラメータを辞書としてソートしてタプル化
        group_items = tuple(sorted((k, v) for k, v in params.items() if k not in ('direction', 'file_timestamp')))
        group_tuple = group_items
        grouped_files[group_tuple][direction].append(file)
        file_params_map[file] = params

    if not grouped_files:
        print("有効なグループがありません。終了します。")
        sys.exit(1)

    # 出力準備（グループ単位のみ）
    failed_data_files: List[Tuple[str, str]] = []
    processed_groups = 0

    sorted_groups = sorted(grouped_files.items())
    for group_tuple, sides in _iter_progress(
        sorted_groups,
        total=len(sorted_groups),
        desc="merge_front_back_bias: groups",
        progress=getattr(args, "progress", "auto"),
    ):
        front_files = sides['front']
        back_files = sides['back_reversed']
        if not front_files or not back_files:
            print(f"グループ {group_tuple} は front/back のどちらかが欠けています。スキップ。")
            continue

        print(f"\n処理グループ: {group_tuple}")
        # front 読み込み
        front_frames = []
        for f in _iter_progress(
            front_files,
            total=len(front_files),
            desc="  front",
            leave=False,
            progress=getattr(args, "progress", "auto"),
        ):
            df, reason = read_and_extract_data(
                f,
                dropna_mode=getattr(args, 'dropna_mode', 'any'),
                dropna_subset=getattr(args, 'dropna_subset', None)
            )
            if df is None or (hasattr(df, 'empty') and df.empty):
                print(f"  frontファイル {f} の読み込み/抽出に失敗。スキップ。")
                failed_data_files.append((f, reason or "EmptyDataFrame"))
                continue
            # 先頭スキップとウォームアップ除外
            if hasattr(args, 'skip_seconds') and args.skip_seconds > 0:
                df = df[df['time_elapsed'] >= args.skip_seconds]
            if hasattr(args, 'step_warmup') and args.step_warmup > 0:
                df = df[df['step_elapsed_time'] > args.step_warmup]
            front_frames.append(df)

        # back 読み込み
        back_frames = []
        for f in _iter_progress(
            back_files,
            total=len(back_files),
            desc="  back",
            leave=False,
            progress=getattr(args, "progress", "auto"),
        ):
            df, reason = read_and_extract_data(
                f,
                dropna_mode=getattr(args, 'dropna_mode', 'any'),
                dropna_subset=getattr(args, 'dropna_subset', None)
            )
            if df is None or (hasattr(df, 'empty') and df.empty):
                print(f"  backファイル {f} の読み込み/抽出に失敗。スキップ。")
                failed_data_files.append((f, reason or "EmptyDataFrame"))
                continue
            if hasattr(args, 'skip_seconds') and args.skip_seconds > 0:
                df = df[df['time_elapsed'] >= args.skip_seconds]
            if hasattr(args, 'step_warmup') and args.step_warmup > 0:
                df = df[df['step_elapsed_time'] > args.step_warmup]
            back_frames.append(df)

        if not front_frames or not back_frames:
            print("  front/back の有効なデータが不足しています。グループをスキップ。")
            continue

        front_all = pd.concat(front_frames, ignore_index=True)
        back_all = pd.concat(back_frames, ignore_index=True)

        # バイアス推定
        if args.bias_scope == 'per-step':
            step_bias_map, global_bias = _compute_bias_per_step(front_all, back_all, args.bias_agg)
            bias_global = global_bias
        else:
            step_bias_map = None
            bias_global = _compute_bias_global(front_all, back_all, args.bias_agg)

        # 補正列の付与（元列は変更しない）
        front_corrected = _apply_bias_corrected_columns(front_all, 'front', bias_global, step_bias_map, bias_global, emit_bias_columns=args.emit_bias_columns)
        back_corrected = _apply_bias_corrected_columns(back_all, 'back_reversed', bias_global, step_bias_map, bias_global, emit_bias_columns=args.emit_bias_columns)
        merged = pd.concat([front_corrected, back_corrected], ignore_index=True)

        processed_groups += 1

        # 出力ファイル名: 代表として最初の front ファイル名を採用し、末尾だけ置換
        rep_file = os.path.basename(front_files[0])
        base, _ts = _strip_trailing_keyword_timestamp(rep_file, args.keywords)
        # base は '... .csv' 形式で返る想定なので '.csv' を外してから新しい尾部を付与
        if base.lower().endswith('.csv'):
            stem = base[:-4]
        else:
            stem = os.path.splitext(base)[0]
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        out_name = f"{stem}_{args.tail_keyword}_{stamp}.csv"
        out_dir = Path(args.output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / out_name
        merged.to_csv(out_path, index=False)
        print(f"  出力: {out_path}")

    if processed_groups == 0:
        print("\n有効な出力は生成されませんでした。")

    # 失敗情報の表示
    if failed_param_files:
        print(f"\n[失敗] パラメータ抽出: {len(failed_param_files)} 件")
        for f in failed_param_files:
            print(f"  {f}  | 理由: ParameterRegexMismatch/DirectionMissing")
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


