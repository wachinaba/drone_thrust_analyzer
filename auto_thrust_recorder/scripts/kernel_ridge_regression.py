#!/usr/bin/env python3
"""
カーネルリッジ回帰（KRR）による目的変数（デフォルト: torque_x）のモデル化スクリプト

使用例:
    python kernel_ridge_regression.py data.csv \
        --features distance,tilt_angle,force_z \
        --target torque_x \
        --kernel rbf --alpha 1e-2 --gamma 0.5 \
        --do-grid-search --cv-folds 5 --cv-jobs 4 \
        --normalize --output krr_results.png

派生ターゲットを式で指定する例（安全のため pandas.eval(engine='numexpr') 相当）:
    python kernel_ridge_regression.py data.csv \
        --target target_thrust \
        --target-expr "force_z / 9.80665" \
        --features distance,tilt_angle,force_z \
        --plot-raw-fit --row-group-by distance --col-group-by tilt_angle \
        --curve-x force_z \
        --normalize --output krr.png
"""

import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import argparse
import os
import japanize_matplotlib
import ast
import re
from sklearn.kernel_ridge import KernelRidge
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
from sklearn.inspection import permutation_importance
from sklearn.metrics.pairwise import rbf_kernel, linear_kernel, polynomial_kernel
import warnings
warnings.filterwarnings('ignore')


def _eval_expr_safe(df: pd.DataFrame, expr: str):
    """
    numexpr が無い環境向けの安全な式評価。
    - 列名（識別子）を df[col] として解決
    - 数値定数、+ - * / **、単項 +/-
    - いくつかの関数（abs, sqrt, log, log10, exp）
    - バッククォート列名: `col-name` をサポート（内部で置換）
    """
    expr = str(expr).strip()

    # バッククォートで囲われた列名を安全な仮名に置換
    # 例: `torque-x` -> __col0
    bt_map: dict[str, str] = {}
    def _repl(m):
        key = m.group(1)
        if key not in bt_map:
            bt_map[key] = f"__col{len(bt_map)}"
        return bt_map[key]
    expr2 = re.sub(r"`([^`]+)`", _repl, expr)

    # 評価環境
    env: dict[str, object] = {}
    for k, v in bt_map.items():
        if k not in df.columns:
            raise ValueError(f"式中の列 `{k}` がデータに存在しません。")
        env[v] = df[k]

    # 通常の識別子列名は AST の Name として解決する（存在チェックは後段）
    allowed_funcs = {
        "abs": np.abs,
        "sqrt": np.sqrt,
        "log": np.log,
        "log10": np.log10,
        "exp": np.exp,
    }
    env.update({"pi": float(np.pi), "e": float(np.e)})

    tree = ast.parse(expr2, mode="eval")

    class _SafeEval(ast.NodeVisitor):
        def visit(self, node):
            return super().visit(node)

        def visit_Expression(self, node: ast.Expression):
            return self.visit(node.body)

        def visit_Constant(self, node: ast.Constant):
            if isinstance(node.value, (int, float, bool)):
                return node.value
            raise ValueError("式では数値/真偽値以外の定数は使えません。")

        def visit_Name(self, node: ast.Name):
            name = node.id
            if name in env:
                return env[name]
            if name in allowed_funcs:
                return allowed_funcs[name]
            if name in df.columns:
                return df[name]
            raise ValueError(f"式中の識別子 '{name}' が解決できません（列名が存在しない？）。")

        def visit_UnaryOp(self, node: ast.UnaryOp):
            v = self.visit(node.operand)
            if isinstance(node.op, ast.UAdd):
                return +v
            if isinstance(node.op, ast.USub):
                return -v
            raise ValueError("許可されていない単項演算子です。")

        def visit_BinOp(self, node: ast.BinOp):
            l = self.visit(node.left)
            r = self.visit(node.right)
            if isinstance(node.op, ast.Add):
                return l + r
            if isinstance(node.op, ast.Sub):
                return l - r
            if isinstance(node.op, ast.Mult):
                return l * r
            if isinstance(node.op, ast.Div):
                return l / r
            if isinstance(node.op, ast.Pow):
                return l ** r
            raise ValueError("許可されていない二項演算子です。")

        def visit_Call(self, node: ast.Call):
            fn = self.visit(node.func)
            if fn not in allowed_funcs.values():
                raise ValueError("許可されていない関数呼び出しです。")
            if node.keywords:
                raise ValueError("キーワード引数は使えません。")
            args = [self.visit(a) for a in node.args]
            return fn(*args)

        # それ以外は全部禁止
        def generic_visit(self, node):
            raise ValueError(f"許可されていない構文です: {type(node).__name__}")

    return _SafeEval().visit(tree)


def _ensure_target_column(df: pd.DataFrame, target_column: str, target_expr: str | None):
    """
    target_column が df に存在しない場合、target_expr から生成して追加する。
    安全のため、生の eval は使わず pandas.eval(engine='numexpr') に限定する。
    """
    if target_column in df.columns:
        return df
    if not target_expr:
        return df

    expr = str(target_expr).strip()
    if not expr:
        return df
    if '=' in expr:
        raise ValueError(
            "target-expr に代入式は使えません。"
            "--target で列名を指定し、--target-expr には右辺の式だけを書いてください。"
            "例: --target my_target --target-expr \"force_z / 9.80665\""
        )

    # まず numexpr が使えるなら pandas.eval(engine='numexpr') を試す。
    # numexpr が未導入の場合は、安全な簡易評価へフォールバックする。
    try:
        y = df.eval(expr, engine='numexpr')
    except Exception as e:
        # "numexpr is not installed" / ImportError 系を含む幅広い例外をフォールバック対象にする
        try:
            y = _eval_expr_safe(df, expr)
            print("[target-expr] 注意: numexpr が利用できないため、簡易評価（安全制限あり）で式を計算しました。")
        except Exception as e2:
            raise ValueError(
                f"target-expr の評価に失敗しました: {e}\n"
                f"(fallback error) {e2}\n"
                f"target='{target_column}', expr='{expr}'\n"
                "列名に記号が含まれる場合はバッククォートで囲ってください（例: `torque-x`）。"
            )
    # df.eval は Series か ndarray/scalar になり得る
    df[target_column] = y
    return df


def load_and_preprocess_data(csv_file, selected_features=None, max_variance_torque_x=None, target_column: str = 'torque_x', target_expr: str | None = None):
    """
    CSVファイルを読み込み、データの前処理を行う
    
    Args:
        csv_file: CSVファイルのパス
        selected_features: 使用する特徴量のリスト（Noneの場合は目的変数以外の全列）
        max_variance_torque_x: 'variance_torque_x' の上限（Noneで無効）
        target_column: 目的変数の列名（デフォルト: torque_x）
    """
    if not os.path.exists(csv_file):
        raise FileNotFoundError(f"ファイル '{csv_file}' が見つかりません。")

    try:
        df = pd.read_csv(csv_file)
        print(f"CSVファイル '{csv_file}' を読み込みました。")
        print(f"データ行数: {len(df)}")
        print(f"利用可能な列: {list(df.columns)}")
    except Exception as e:
        raise Exception(f"CSVファイルの読み込み中にエラーが発生しました: {e}")

    # ターゲット列の生成（必要な場合）
    df = _ensure_target_column(df, target_column=target_column, target_expr=target_expr)

    if target_column not in df.columns:
        raise ValueError(f"必要な列が見つかりません: ['{target_column}']")

    if selected_features is not None and len(selected_features) > 0:
        if target_column in selected_features:
            raise ValueError(f"'{target_column}' は目的変数のため特徴量に含められません。")
        missing = [c for c in selected_features if c not in df.columns]
        if missing:
            raise ValueError(f"指定された特徴量が見つかりません: {missing}")
        feature_columns = list(selected_features)
    else:
        # 既定は 目的変数 以外の全列
        feature_columns = [c for c in df.columns if c != target_column]

    # 数値化
    numeric_columns = [target_column] + feature_columns
    for col in numeric_columns:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')

    # 高分散の除外（列があれば）
    if max_variance_torque_x is not None and 'variance_torque_x' in df.columns:
        before = len(df)
        df = df[df['variance_torque_x'] <= max_variance_torque_x]
        after = len(df)
        print(f"variance_torque_x フィルタ: {before - after} 行を除外（閾値 {max_variance_torque_x}）")

    # NaN除去
    df_clean = df.dropna(subset=[target_column] + feature_columns)
    if df_clean.empty:
        raise ValueError("有効なデータがありません。")

    print(f"前処理後のデータ行数: {len(df_clean)}")
    print(f"使用する特徴量: {feature_columns}")

    return df_clean, feature_columns


def make_rbf_linear_kernel(gamma: float):
    """RBF + 線形の和のカーネルを返す。"""
    def kernel(X, Y):
        return rbf_kernel(X, Y, gamma=gamma) + linear_kernel(X, Y)
    return kernel


def create_krr_model(kernel_type='rbf', alpha=1e-2, gamma=1.0, degree=3, coef0=1.0):
    if kernel_type == 'rbf':
        model = KernelRidge(alpha=alpha, kernel='rbf', gamma=gamma)
    elif kernel_type == 'linear':
        model = KernelRidge(alpha=alpha, kernel='linear')
    elif kernel_type == 'poly':
        model = KernelRidge(alpha=alpha, kernel='poly', gamma=gamma, degree=degree, coef0=coef0)
    elif kernel_type == 'rbf_linear':
        model = KernelRidge(alpha=alpha, kernel=make_rbf_linear_kernel(gamma))
    else:
        model = KernelRidge(alpha=alpha, kernel='rbf', gamma=gamma)
    return model


def evaluate_model(model, X_test, y_test, X_train=None, y_train=None, cv_folds=0, cv_jobs=1):
    y_pred = model.predict(X_test)

    mse = mean_squared_error(y_test, y_pred)
    rmse = np.sqrt(mse)
    mae = mean_absolute_error(y_test, y_pred)
    r2 = r2_score(y_test, y_pred)

    metrics = {
        'mse': mse,
        'rmse': rmse,
        'mae': mae,
        'r2': r2,
        'y_pred': y_pred,
    }

    if cv_folds and cv_folds > 0 and X_train is not None and y_train is not None:
        cv_scores = cross_val_score(model, X_train, y_train, cv=cv_folds, scoring='neg_mean_squared_error', n_jobs=cv_jobs)
        metrics['cv_rmse'] = np.sqrt(-cv_scores.mean())
        metrics['cv_std'] = np.sqrt(cv_scores.std())

    return metrics


def plot_results(y_test, y_pred, output_file=None, target_column: str = 'torque_x'):
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))

    # 1. 予測値 vs 実際の値
    ax1 = axes[0, 0]
    ax1.scatter(y_test, y_pred, alpha=0.6, s=50)
    ax1.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], 'r--', lw=2)
    ax1.set_xlabel(f'実際の値 ({target_column})')
    ax1.set_ylabel(f'予測値 ({target_column})')
    ax1.set_title('予測値 vs 実際の値')
    ax1.grid(True, alpha=0.3)

    # 2. 残差プロット
    ax2 = axes[0, 1]
    residuals = y_test - y_pred
    ax2.scatter(y_pred, residuals, alpha=0.6, s=50)
    ax2.axhline(y=0, color='r', linestyle='--')
    ax2.set_xlabel(f'予測値 ({target_column})')
    ax2.set_ylabel('残差')
    ax2.set_title('残差プロット')
    ax2.grid(True, alpha=0.3)

    # 3. 不確実性(未対応)
    ax3 = axes[1, 0]
    ax3.text(0.5, 0.5, '不確実性はKRRでは未対応', ha='center', va='center', fontsize=12)
    ax3.set_axis_off()

    # 4. 時系列プロット
    ax4 = axes[1, 1]
    indices = range(len(y_test))
    ax4.plot(indices, y_test, 'o-', label='実際の値', alpha=0.7)
    ax4.plot(indices, y_pred, 's-', label='予測値', alpha=0.7)
    ax4.set_xlabel('データポイント')
    ax4.set_ylabel(target_column)
    ax4.set_title('時系列での比較')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"結果を '{output_file}' に保存しました。")
    else:
        plt.show()
    plt.close()


def plot_grouped_raw_and_fit(df_clean, feature_columns, model, scaler, group_by, curve_x=None, curve_points=200, ranges=None, output_file=None, target_column: str = 'torque_x'):
    """
    指定した group_by 特徴量でデータをグループ化し、各グループで
    - 生の散布 (curve_x vs target_column)
    - KRR のフィット曲線 (curve_x を掃引、他特徴量はグループの中央値固定)
    を同一サブプロット上に描画する。

    Args:
        df_clean: 前処理済み DataFrame（目的変数 target_column を含む）
        feature_columns: 学習に使用した特徴量列
        model: 学習済み KRR モデル
        scaler: StandardScaler もしくは None
        group_by: グループ化に使用する列名のリスト（df_clean 上に存在）
        curve_x: フィット曲線で横軸にする特徴量（None の場合、group_by 以外の最初の特徴量）
        curve_points: 曲線の分解能
        ranges: (mins, maxs) を返す parse_ranges の結果タプル or None
        output_file: ファイルパス（None なら表示）
    """
    if not group_by or len(group_by) == 0:
        print("[plot_grouped_raw_and_fit] group_by が指定されていないためスキップします。")
        return

    for col in group_by:
        if col not in df_clean.columns:
            raise ValueError(f"group_by 列 '{col}' がデータに存在しません。")

    # x 軸の候補決定
    if curve_x is None:
        candidates = [c for c in feature_columns if c not in group_by]
        if not candidates:
            raise ValueError("curve_x を自動決定できません。group_by 以外の特徴量がありません。--curve-x を指定してください。")
        curve_x = candidates[0]
    if curve_x not in feature_columns:
        raise ValueError(f"curve_x '{curve_x}' は学習特徴量に含まれていません。feature_columns={feature_columns}")

    # レンジ決定
    if ranges is not None and isinstance(ranges, tuple) and len(ranges) == 2:
        mins, maxs = ranges
    else:
        mins = {c: float(df_clean[c].min()) for c in feature_columns}
        maxs = {c: float(df_clean[c].max()) for c in feature_columns}

    # グループを作成
    group_keys = df_clean[group_by].drop_duplicates()
    n_groups = len(group_keys)
    if n_groups == 0:
        print("[plot_grouped_raw_and_fit] グループが見つかりません。")
        return

    n_cols = min(4, n_groups)
    n_rows = int(np.ceil(n_groups / n_cols))
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5.0 * n_cols, 4.0 * n_rows), squeeze=False)

    for idx, (_, gvals) in enumerate(group_keys.iterrows()):
        r = idx // n_cols
        c = idx % n_cols
        ax = axes[r, c]

        # グループ条件で抽出
        mask = np.ones(len(df_clean), dtype=bool)
        title_parts = []
        for col in group_by:
            val = gvals[col]
            mask &= (df_clean[col] == val)
            # タイトル用に短く
            if isinstance(val, float):
                title_parts.append(f"{col}={val:g}")
            else:
                title_parts.append(f"{col}={val}")
        df_group = df_clean[mask]

        if df_group.empty:
            ax.set_axis_off()
            continue

        # 生データの散布
        if curve_x not in df_group.columns:
            raise ValueError(f"x 軸列 '{curve_x}' がデータに存在しません。")
        x_raw = df_group[curve_x].values
        y_raw = df_group[target_column].values
        ax.scatter(x_raw, y_raw, alpha=0.5, s=25, label='raw')

        # 予測用グリッド（curve_x を掃引、他は中央値固定（group 内））
        x_min = mins.get(curve_x, float(df_group[curve_x].min()))
        x_max = maxs.get(curve_x, float(df_group[curve_x].max()))
        if not np.isfinite(x_min) or not np.isfinite(x_max):
            x_min = float(df_group[curve_x].min())
            x_max = float(df_group[curve_x].max())
        if x_min == x_max:
            # 単一点の場合はわずかに広げる
            x_min -= 1e-6
            x_max += 1e-6
        x_grid = np.linspace(x_min, x_max, int(curve_points))

        X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
        for j, fcol in enumerate(feature_columns):
            if fcol == curve_x:
                X_grid[:, j] = x_grid
            elif fcol in group_by:
                # グループの代表値（同一値のはずだが保険で中央値）
                X_grid[:, j] = float(df_group[fcol].median())
            else:
                X_grid[:, j] = float(df_group[fcol].median())

        if scaler is not None:
            try:
                X_infer = scaler.transform(X_grid)
            except Exception:
                X_infer = X_grid
        else:
            X_infer = X_grid

        y_pred = model.predict(X_infer)
        ax.plot(x_grid, y_pred, color='C1', lw=2.0, label='KRR fit')

        ax.set_title(', '.join(title_parts))
        ax.set_xlabel(curve_x)
        ax.set_ylabel(target_column)
        ax.grid(True, alpha=0.3)
        ax.legend(frameon=True, fontsize=9)

    # 余白のサブプロットを非表示
    for k in range(n_groups, n_rows * n_cols):
        r = k // n_cols
        c = k % n_cols
        axes[r, c].set_axis_off()

    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"グループ別 Raw vs Fit を '{output_file}' に保存しました。")
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)


def apply_facet_filters(df: pd.DataFrame, facet_filters):
    """
    ファセット描画前の事前フィルタを適用する。

    facet_filters: ["col:min,max", ...] を想定（複数指定可）
    """
    if not facet_filters:
        return df

    df_out = df
    for item in facet_filters:
        if not item or (':' not in item) or (',' not in item):
            continue
        col, span = item.split(':', 1)
        col = col.strip()
        if col not in df_out.columns:
            print(f"[facet-filter] 警告: 列 '{col}' が存在しません。スキップ: {item}")
            continue
        lo_s, hi_s = span.split(',', 1)
        try:
            lo = float(lo_s)
            hi = float(hi_s)
        except Exception:
            print(f"[facet-filter] 警告: 範囲の数値化に失敗。スキップ: {item}")
            continue

        before = len(df_out)
        df_out = df_out[(df_out[col] >= lo) & (df_out[col] <= hi)]
        after = len(df_out)
        print(f"[facet-filter] {col}: [{lo}, {hi}] で {before-after} 行を除外")
    return df_out


def plot_facet_raw_and_fit_krr(
    df_clean,
    feature_columns,
    model,
    scaler,
    row_group_by,
    col_group_by,
    curve_x=None,
    curve_points=200,
    ranges=None,
    output_file=None,
):
    """
    行方向(row_group_byの組) × 列方向(col_group_byの組)のファセットで、
    各セルに Raw 散布と KRR フィット曲線を描画する。
    """
    if not row_group_by and not col_group_by:
        print("[plot_facet_raw_and_fit_krr] row/col が未指定のためスキップします。")
        return

    target_column = getattr(plot_facet_raw_and_fit_krr, "_target_column", "torque_x")

    # x 軸の決定
    if curve_x is None:
        excluded = set((row_group_by or []) + (col_group_by or []))
        candidates = [c for c in feature_columns if c not in excluded]
        if not candidates:
            raise ValueError("curve_x を自動決定できません。--curve-x を指定してください。")
        curve_x = candidates[0]
    if curve_x not in feature_columns:
        raise ValueError(f"curve_x '{curve_x}' は学習特徴量に含まれていません。feature_columns={feature_columns}")

    # レンジ（全ファセットで共有するグローバルな範囲）
    if ranges is not None and isinstance(ranges, tuple) and len(ranges) == 2:
        mins, maxs = ranges
    else:
        mins = {c: float(df_clean[c].min()) for c in feature_columns}
        maxs = {c: float(df_clean[c].max()) for c in feature_columns}

    # 全ファセットで共通の X / Y 範囲
    global_x_min = mins.get(curve_x, float(df_clean[curve_x].min()))
    global_x_max = maxs.get(curve_x, float(df_clean[curve_x].max()))
    if not np.isfinite(global_x_min) or not np.isfinite(global_x_max):
        global_x_min = float(df_clean[curve_x].min())
        global_x_max = float(df_clean[curve_x].max())
    if global_x_min == global_x_max:
        global_x_min -= 1e-6
        global_x_max += 1e-6

    global_y_min = float(df_clean[target_column].min())
    global_y_max = float(df_clean[target_column].max())
    if not np.isfinite(global_y_min) or not np.isfinite(global_y_max):
        global_y_min = -1.0
        global_y_max = 1.0
    if global_y_min == global_y_max:
        global_y_min -= 1e-6
        global_y_max += 1e-6

    def make_keys(group_cols):
        if not group_cols:
            return [()]
        uniq = df_clean[group_cols].drop_duplicates()
        keys = [tuple(row[c] for c in group_cols) for _, row in uniq.iterrows()]
        return keys

    row_keys = make_keys(row_group_by)
    col_keys = make_keys(col_group_by)

    n_rows = max(1, len(row_keys))
    n_cols = max(1, len(col_keys))

    plt.rcParams.update({'font.size': 18})
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(6.0 * n_cols, 5.0 * n_rows), squeeze=False)

    for r_idx, rkey in enumerate(row_keys):
        for c_idx, ckey in enumerate(col_keys):
            ax = axes[r_idx, c_idx]

            # 行ヘッダ（左側）/ 列ヘッダ（上側）
            if (c_idx == 0) and row_group_by:
                row_title = '\n'.join(
                    [f"{col}={val:g}" if isinstance(val, float) else f"{col}={val}"
                     for col, val in zip(row_group_by, rkey)]
                )
                ax.set_ylabel(target_column)
                ax.annotate(
                    row_title,
                    xy=(-0.40, 0.5),
                    xycoords='axes fraction',
                    rotation=0,
                    ha='right',
                    va='center',
                    fontsize=18,
                    linespacing=1.0,
                    annotation_clip=False
                )
            else:
                ax.set_ylabel(target_column)

            if (r_idx == 0) and col_group_by:
                col_title = '\n'.join(
                    [f"{col}={val:g}" if isinstance(val, float) else f"{col}={val}"
                     for col, val in zip(col_group_by, ckey)]
                )
                ax.set_title(col_title, fontsize=20)
            else:
                ax.set_title('')

            # マスク作成
            mask = np.ones(len(df_clean), dtype=bool)
            if row_group_by:
                for col, val in zip(row_group_by, rkey):
                    mask &= (df_clean[col] == val)
            if col_group_by:
                for col, val in zip(col_group_by, ckey):
                    mask &= (df_clean[col] == val)

            df_cell = df_clean[mask]
            if df_cell.empty:
                ax.grid(False)
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_xlabel(curve_x)
                continue

            # 生データ散布
            x_raw = df_cell[curve_x].values
            y_raw = df_cell[target_column].values
            ax.scatter(x_raw, y_raw, alpha=0.5, s=25, label='raw')

            # 予測用グリッド（全ファセット共通の X 範囲）
            x_grid = np.linspace(global_x_min, global_x_max, int(curve_points))
            X_grid = np.zeros((len(x_grid), len(feature_columns)), dtype=float)
            for j, fcol in enumerate(feature_columns):
                if fcol == curve_x:
                    X_grid[:, j] = x_grid
                elif (row_group_by and fcol in row_group_by):
                    val = rkey[row_group_by.index(fcol)] if fcol in row_group_by else float(df_cell[fcol].median())
                    X_grid[:, j] = float(val)
                elif (col_group_by and fcol in col_group_by):
                    val = ckey[col_group_by.index(fcol)] if fcol in col_group_by else float(df_cell[fcol].median())
                    X_grid[:, j] = float(val)
                else:
                    X_grid[:, j] = float(df_cell[fcol].median())

            if scaler is not None:
                try:
                    X_infer = scaler.transform(X_grid)
                except Exception:
                    X_infer = X_grid
            else:
                X_infer = X_grid

            y_pred = model.predict(X_infer)
            ax.plot(x_grid, y_pred, color='red', lw=3.0, label='KRR fit')

            ax.set_xlim(global_x_min, global_x_max)
            ax.set_ylim(global_y_min, global_y_max)
            ax.set_xlabel(curve_x)
            ax.grid(True, alpha=0.3)
            ax.legend(frameon=True, fontsize=10)

    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"ファセット Raw vs KRR Fit を '{output_file}' に保存しました。")
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)


def plot_permutation_importance(model, X_test, y_test, feature_columns, output_file=None):
    result = permutation_importance(model, X_test, y_test, n_repeats=10, random_state=42, n_jobs=1)
    importances = result.importances_mean
    importances = importances / np.sum(importances) if importances.sum() > 0 else importances

    plt.figure(figsize=(10, 6))
    bars = plt.bar(feature_columns, importances)
    plt.xlabel('特徴量')
    plt.ylabel('相対重要度（置換重要度）')
    plt.title('Permutation Importance')
    plt.xticks(rotation=45)
    plt.grid(True, alpha=0.3)
    for bar, imp in zip(bars, importances):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01, f'{imp:.3f}', ha='center', va='bottom')
    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"特徴量重要度を '{output_file}' に保存しました。")
    else:
        plt.show()
    plt.close()


def parse_fixed_values(fixes, feature_columns, df_clean):
    fixed = {c: float(df_clean[c].median()) for c in feature_columns}
    if fixes:
        for item in fixes:
            if '=' not in item:
                continue
            col, val = item.split('=', 1)
            col = col.strip()
            if col in fixed:
                try:
                    fixed[col] = float(val)
                except Exception:
                    pass
    return fixed


def parse_ranges(ranges, feature_columns, df_clean):
    mins = {c: float(df_clean[c].min()) for c in feature_columns}
    maxs = {c: float(df_clean[c].max()) for c in feature_columns}
    if ranges:
        for item in ranges:
            if ':' not in item:
                continue
            col, span = item.split(':', 1)
            if ',' not in span:
                continue
            lo, hi = span.split(',', 1)
            col = col.strip()
            if col in mins:
                try:
                    mins[col] = float(lo)
                    maxs[col] = float(hi)
                except Exception:
                    pass
    return mins, maxs


def generate_pairwise_heatmaps_krr(model, feature_columns, df_clean, scaler, grid_size, fixes, ranges, output_prefix):
    target_column = getattr(generate_pairwise_heatmaps_krr, "_target_column", "torque_x")
    fixed_values = parse_fixed_values(fixes, feature_columns, df_clean)
    mins, maxs = parse_ranges(ranges, feature_columns, df_clean)

    for i in range(len(feature_columns)):
        for j in range(i + 1, len(feature_columns)):
            fi = feature_columns[i]
            fj = feature_columns[j]

            xi = np.linspace(mins[fi], maxs[fi], grid_size)
            xj = np.linspace(mins[fj], maxs[fj], grid_size)
            XI, XJ = np.meshgrid(xi, xj)

            X_grid = np.zeros((grid_size * grid_size, len(feature_columns)))
            for k, fk in enumerate(feature_columns):
                if fk == fi:
                    X_grid[:, k] = XI.ravel()
                elif fk == fj:
                    X_grid[:, k] = XJ.ravel()
                else:
                    X_grid[:, k] = fixed_values[fk]

            if scaler is not None:
                X_infer = scaler.transform(X_grid)
            else:
                X_infer = X_grid

            y_pred = model.predict(X_infer)
            Z = y_pred.reshape(grid_size, grid_size)

            plt.figure(figsize=(6, 5))
            plt.imshow(Z, origin='lower', aspect='auto',
                       extent=[mins[fi], maxs[fi], mins[fj], maxs[fj]],
                       cmap='viridis')
            plt.colorbar(label=f'予測値 ({target_column})')
            plt.xlabel(fi)
            plt.ylabel(fj)
            plt.title(f'予測ヒートマップ(KRR): {target_column} / {fi} vs {fj}')
            if output_prefix:
                out = f"{output_prefix}_results_{fi}__{fj}.png"
                plt.savefig(out, dpi=200, bbox_inches='tight')
                print(f"ヒートマップを保存: {out}")
                plt.close()
            else:
                plt.show()


def grid_search_krr(X_train, y_train, base_args):
    """簡易グリッドで alpha と gamma（必要なら degree）を探索"""
    cv = base_args.cv_folds if base_args.cv_folds and base_args.cv_folds > 0 else 3

    a = max(base_args.alpha, 1e-6)
    g = max(base_args.gamma, 1e-6)
    candidate_alphas = sorted(set([a, a*3.0, max(a/3.0, 1e-6)]))
    candidate_gammas = sorted(set([g, g*3.0, max(g/3.0, 1e-6)]))
    candidate_degrees = [base_args.degree] if base_args.kernel != 'poly' else [base_args.degree, base_args.degree + 1]

    best = {'score': float('inf'), 'alpha': None, 'gamma': None, 'degree': None}

    for alpha in candidate_alphas:
        for gamma in (candidate_gammas if base_args.kernel in ['rbf', 'rbf_linear', 'poly'] else [None]):
            for deg in candidate_degrees:
                model = create_krr_model(
                    kernel_type=base_args.kernel,
                    alpha=alpha,
                    gamma=(gamma if gamma is not None else base_args.gamma),
                    degree=deg,
                    coef0=base_args.coef0
                )
                scores = cross_val_score(model, X_train, y_train, cv=cv, scoring='neg_mean_squared_error', n_jobs=base_args.cv_jobs)
                mse = -scores.mean()
                if mse < best['score']:
                    best = {'score': mse, 'alpha': alpha, 'gamma': gamma if gamma is not None else base_args.gamma, 'degree': deg}

    best_model = create_krr_model(
        kernel_type=base_args.kernel,
        alpha=best['alpha'],
        gamma=best['gamma'],
        degree=best['degree'],
        coef0=base_args.coef0
    )
    best_model.fit(X_train, y_train)
    print(f"グリッドサーチ最良: alpha={best['alpha']}, gamma={best['gamma']}, degree={best['degree']}, CV MSE={best['score']:.6f}")
    return best_model


def main():
    parser = argparse.ArgumentParser(description='カーネルリッジ回帰によるtorque_xのモデル化')
    parser.add_argument('csv_file', help='入力CSVファイルのパス')
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は表示のみ）')
    parser.add_argument('--target', type=str, default='torque_x', help='目的変数の列名（デフォルト: torque_x）')
    parser.add_argument('--target-expr', type=str, default=None,
                        help="目的変数を式から生成する（例: \"force_z / 9.80665\"）。"
                             "安全のため pandas.eval(engine='numexpr') で評価。代入式は不可。")
    parser.add_argument('--test-size', type=float, default=0.2, help='テストデータの割合（デフォルト: 0.2）')
    parser.add_argument('--kernel', choices=['rbf', 'linear', 'poly', 'rbf_linear'], default='rbf', help='カーネルの種類')
    parser.add_argument('--alpha', type=float, default=1e-2, help='リッジ正則化係数（デフォルト: 1e-2）')
    parser.add_argument('--gamma', type=float, default=1.0, help='RBF/Polyのgamma')
    parser.add_argument('--degree', type=int, default=3, help='Polyの次数')
    parser.add_argument('--coef0', type=float, default=1.0, help='Polyの係数coef0')
    parser.add_argument('--random-state', type=int, default=42, help='乱数のシード（デフォルト: 42）')
    parser.add_argument('--normalize', action='store_true', help='特徴量の正規化を行う')
    parser.add_argument('--cv-folds', type=int, default=0, help='クロスバリデーションの分割数（0で無効）')
    parser.add_argument('--cv-jobs', type=int, default=1, help='クロスバリデーションの並列数（デフォルト: 1）')
    parser.add_argument('--max-train-samples', type=int, default=None, help='学習に使用する最大サンプル数（指定時はランダム抽出）')
    parser.add_argument('--features', type=str, default=None, help='使用する特徴量をカンマ区切りで指定（未指定なら目的変数以外の全列）')
    parser.add_argument('--list-features', action='store_true', help='利用可能な特徴量候補を一覧表示して終了')
    parser.add_argument('--max-variance-torque-x', type=float, default=None, help='variance_torque_x の上限で高分散サンプルを除外')
    parser.add_argument('--do-grid-search', action='store_true', help='alpha/gamma(degree)の簡易グリッドサーチを実行')
    parser.add_argument('--perm-importance', action='store_true', help='Permutation Importanceを算出して出力')
    # ペアワイズヒートマップ
    parser.add_argument('--pairwise-heatmaps', action='store_true', help='全特徴量ペアの予測ヒートマップを一括出力')
    parser.add_argument('--grid', type=int, default=80, help='ヒートマップの格子数（デフォルト: 80）')
    parser.add_argument('--fix', action='append', default=None, help="非可視化軸の固定値 'col=value' を複数指定可")
    parser.add_argument('--range', dest='ranges', action='append', default=None, help="各軸の範囲 'col:min,max' を複数指定可")
    parser.add_argument('--std-heatmaps', action='store_true', help='(KRRでは無効) 標準偏差ヒートマップの要求は無視して警告表示')
    # 生データとフィット曲線の比較
    parser.add_argument('--plot-raw-fit', action='store_true', help='グループごとに生データ散布とKRRフィット曲線を比較表示')
    parser.add_argument('--group-by', type=str, default=None, help='グループ化に用いる列をカンマ区切りで指定（例: distance,tilt_angle）')
    parser.add_argument('--curve-x', type=str, default=None, help='フィット曲線の横軸にする特徴量（未指定なら group_by 以外の最初の特徴量）')
    parser.add_argument('--curve-points', type=int, default=200, help='フィット曲線の分解能')
    parser.add_argument('--groupfit-output', type=str, default=None, help='グループ別 Raw vs Fit 図の出力パス（未指定なら表示、--output 指定時は派生名を使用）')
    parser.add_argument('--row-group-by', type=str, default=None, help='行方向のファセットに用いる列（カンマ区切りの複数可）')
    parser.add_argument('--col-group-by', type=str, default=None, help='列方向のファセットに用いる列（カンマ区切りの複数可）')
    parser.add_argument('--facet-filter', action='append', default=None, help="ファセット用の事前フィルタ 'col:min,max' を複数指定可")

    args = parser.parse_args()

    try:
        # バックエンド切替（ヘッドレスや--output指定時）
        if (not os.environ.get('DISPLAY')) or args.output:
            try:
                matplotlib.use('Agg', force=True)
            except Exception:
                pass

        # 特徴量一覧だけ
        if args.list_features:
            print("=== 特徴量一覧 ===")
            if not os.path.exists(args.csv_file):
                raise FileNotFoundError(f"ファイル '{args.csv_file}' が見つかりません。")
            df_head = pd.read_csv(args.csv_file, nrows=5)
            candidates = [c for c in df_head.columns if c != args.target]
            print(f"利用可能な特徴量候補: {candidates}")
            print(f"選択中の目的変数: {args.target}")
            return 0

        # 読み込みと前処理
        print("=== データの読み込みと前処理 ===")
        selected_features = None
        if args.features:
            selected_features = [c.strip() for c in args.features.split(',') if c.strip()]
        df_clean, feature_columns = load_and_preprocess_data(
            args.csv_file,
            selected_features,
            max_variance_torque_x=args.max_variance_torque_x,
            target_column=args.target,
            target_expr=args.target_expr,
        )

        X = df_clean[feature_columns].values
        y = df_clean[args.target].values

        print(f"特徴量の形状: {X.shape}")
        print(f"目的変数の形状: {y.shape}")

        scaler = None
        if args.normalize:
            print("特徴量の正規化を実行...")
            scaler = StandardScaler()
            X = scaler.fit_transform(X)

        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=args.test_size, random_state=args.random_state
        )
        print(f"訓練データ数: {len(X_train)}")
        print(f"テストデータ数: {len(X_test)}")

        if args.max_train_samples is not None and len(X_train) > args.max_train_samples:
            rs = np.random.RandomState(args.random_state)
            indices = rs.choice(len(X_train), size=args.max_train_samples, replace=False)
            X_train = X_train[indices]
            y_train = y_train[indices]
            print(f"学習データをサブサンプリング: {len(indices)} サンプルを使用")

        # モデル作成・学習
        print(f"\n=== カーネルリッジ回帰モデルの訓練 ===")
        print(f"カーネル: {args.kernel}")
        print(f"アルファ: {args.alpha}")
        print(f"ガンマ: {args.gamma}")
        if args.kernel == 'poly':
            print(f"次数: {args.degree}, coef0: {args.coef0}")

        if args.do_grid_search:
            print("グリッドサーチを実行します...")
            model = grid_search_krr(X_train, y_train, args)
        else:
            model = create_krr_model(
                kernel_type=args.kernel,
                alpha=args.alpha,
                gamma=args.gamma,
                degree=args.degree,
                coef0=args.coef0
            )
            model.fit(X_train, y_train)

        print("モデルを評価中...")
        metrics = evaluate_model(
            model,
            X_test,
            y_test,
            X_train if args.cv_folds > 0 else None,
            y_train if args.cv_folds > 0 else None,
            cv_folds=args.cv_folds,
            cv_jobs=args.cv_jobs
        )

        print(f"テストデータでの評価:")
        print(f"  RMSE: {metrics['rmse']:.6f}")
        print(f"  MAE:  {metrics['mae']:.6f}")
        print(f"  R²:   {metrics['r2']:.6f}")
        if 'cv_rmse' in metrics:
            print(f"クロスバリデーション:")
            print(f"  CV RMSE: {metrics['cv_rmse']:.6f} ± {metrics['cv_std']:.6f}")

        # 可視化
        print(f"\n=== 結果の可視化 ===")
        results_output = None
        importance_output = None
        if args.output:
            base_name = os.path.splitext(args.output)[0]
            results_output = f"{base_name}_results.png"
            importance_output = f"{base_name}_importance.png"

        plot_results(y_test, metrics['y_pred'], results_output, target_column=args.target)

        if args.perm_importance:
            plot_permutation_importance(model, X_test, y_test, feature_columns, importance_output)

        # ペアワイズヒートマップ
        if args.pairwise_heatmaps:
            print("\n=== ペアワイズヒートマップの生成(KRR) ===")
            if args.std_heatmaps:
                print("(注意) KRRでは標準偏差を算出できないため、stdヒートマップは出力しません。")
            output_prefix = None
            if args.output:
                output_prefix = os.path.splitext(args.output)[0]
            generate_pairwise_heatmaps_krr._target_column = args.target
            generate_pairwise_heatmaps_krr(
                model,
                feature_columns,
                df_clean,
                scaler,
                grid_size=args.grid,
                fixes=args.fix,
                ranges=args.ranges,
                output_prefix=output_prefix
            )

        # 記述的統計
        print(f"\n=== 統計情報 ===")
        print(f"使用した特徴量: {feature_columns}")
        print(f"データポイント数: {len(df_clean)}")
        print(f"{args.target}の範囲: {y.min():.6f} - {y.max():.6f}")
        print(f"{args.target}の平均: {y.mean():.6f}")
        print(f"{args.target}の標準偏差: {y.std():.6f}")

        # 生データとフィット曲線の比較
        if args.plot_raw_fit:
            row_group_by = [c.strip() for c in args.row_group_by.split(',') if c.strip()] if args.row_group_by else None
            col_group_by = [c.strip() for c in args.col_group_by.split(',') if c.strip()] if args.col_group_by else None
            group_by = [c.strip() for c in args.group_by.split(',') if c.strip()] if (args.group_by and not row_group_by and not col_group_by) else None

            # レンジ（既存の --range 指定を流用）
            mins, maxs = parse_ranges(args.ranges, feature_columns, df_clean)

            groupfit_output = args.groupfit_output
            if not groupfit_output and args.output:
                base_name = os.path.splitext(args.output)[0]
                groupfit_output = f"{base_name}_groupfit.png"

            try:
                if row_group_by or col_group_by:
                    df_for_facet = apply_facet_filters(df_clean, args.facet_filter)
                    plot_facet_raw_and_fit_krr._target_column = args.target
                    plot_facet_raw_and_fit_krr(
                        df_clean=df_for_facet,
                        feature_columns=feature_columns,
                        model=model,
                        scaler=scaler,
                        row_group_by=row_group_by,
                        col_group_by=col_group_by,
                        curve_x=args.curve_x,
                        curve_points=args.curve_points,
                        ranges=(mins, maxs),
                        output_file=groupfit_output,
                    )
                elif group_by:
                    plot_grouped_raw_and_fit(
                        df_clean=df_clean,
                        feature_columns=feature_columns,
                        model=model,
                        scaler=scaler,
                        group_by=group_by,
                        curve_x=args.curve_x,
                        curve_points=args.curve_points,
                        ranges=(mins, maxs),
                        output_file=groupfit_output,
                        target_column=args.target,
                    )
                else:
                    print("(注意) --plot-raw-fit は指定されましたが、--group-by も --row-group-by/--col-group-by も未指定です。スキップします。")
            except Exception as e:
                print(f"グループ別 Raw vs Fit の描画でエラー: {e}")

        return 0

    except Exception as e:
        print(f"エラーが発生しました: {e}")
        return 1


if __name__ == "__main__":
    exit(main())


