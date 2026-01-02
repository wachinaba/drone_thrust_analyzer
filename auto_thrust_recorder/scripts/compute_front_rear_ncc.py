import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.lines import Line2D


REQUIRED_COLUMNS = [
    "distance",
    "tilt_angle",
    "wall_spacing",
    "flow_distance",
    "front_in",
    "front_out",
    "rear_in",
    "rear_out",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "front と rear の系列（in/out/out-in）について、flow_distance 軸で正規化相互相関 (NCC) を計算し、"
            "X軸オフセット dx（lag×grid_step）と最大NCCを求めます。"
        )
    )
    parser.add_argument("--input", required=True, type=str, help="csv_concat3.py が出力した結合CSVのパス")
    parser.add_argument("--output", type=str, default=None, help="結果CSVの保存先（未指定なら標準出力）")
    parser.add_argument("--min-points", type=int, default=4, help="補間前に必要な各系列の最小データ点数")
    parser.add_argument(
        "--series",
        type=str,
        default="in,out,out-in",
        help="対象系列（カンマ区切り: in,out,out-in から選択、デフォルトは全て）",
    )
    parser.add_argument("--plot-dir", type=str, default=None, help="各グループ・系列の重ね描き図の保存先ディレクトリ")
    parser.add_argument("--plot-format", type=str, default="png", help="図の保存フォーマット（png, pdf, svg など）")
    parser.add_argument("--plot-dpi", type=int, default=180, help="図の保存DPI")
    parser.add_argument("--facet-output", type=str, default=None, help="in/out/out-in をまとめたファセット図の保存先（1枚）")
    parser.add_argument("--facet-dpi", type=int, default=180, help="ファセット図の保存DPI（未指定時は180）")
    return parser.parse_args()


def validate_columns(df: pd.DataFrame):
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"CSV に必須列が不足しています: {', '.join(missing)}")


def load_csv(path: str) -> pd.DataFrame:
    try:
        df = pd.read_csv(path)
    except Exception as e:
        raise RuntimeError(f"CSV 読込に失敗しました: {e}")

    validate_columns(df)

    for col in REQUIRED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["distance", "tilt_angle", "wall_spacing", "flow_distance"]).copy()
    return df


def make_col_key(w, t):
    try:
        w_str = f"{float(w):g}"
    except Exception:
        w_str = str(w)
    try:
        t_str = f"{int(round(float(t)))}"
    except Exception:
        t_str = str(t)
    return f"w={w_str}, tilt={t_str}"


def aggregate_by_flow_distance(df: pd.DataFrame) -> pd.DataFrame:
    key_cols = ["distance", "tilt_angle", "wall_spacing", "flow_distance"]
    agg = (
        df.groupby(key_cols, as_index=False)[["front_in", "front_out", "rear_in", "rear_out"]]
        .mean()
    )
    agg["col_key"] = [make_col_key(w, t) for w, t in zip(agg["wall_spacing"], agg["tilt_angle"])]
    return agg


def build_series_pair(agg_group: pd.DataFrame, series: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # 戻り値: x_front, y_front, y_rear （x は flow_distance 正）
    df = agg_group.sort_values("flow_distance")
    x = df["flow_distance"].values.astype(float)

    if series == "in":
        y_front = df["front_in"].values.astype(float)
        y_rear = df["rear_in"].values.astype(float)
    elif series == "out":
        y_front = df["front_out"].values.astype(float)
        y_rear = df["rear_out"].values.astype(float)
    elif series == "out-in":
        y_front = (df["front_out"].values - df["front_in"].values).astype(float)
        y_rear = (df["rear_out"].values - df["rear_in"].values).astype(float)
    else:
        raise ValueError(f"未知の series: {series}")

    # 欠損除去（front/rear 片側でも NaN があれば除外）
    m = np.isfinite(x) & np.isfinite(y_front) & np.isfinite(y_rear)
    return x[m], y_front[m], y_rear[m]


def median_step(x: np.ndarray) -> float:
    if len(x) < 2:
        return np.nan
    diffs = np.diff(np.unique(x))
    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    if len(diffs) == 0:
        return np.nan
    return float(np.median(diffs))


def interpolate_to_common_grid(xf: np.ndarray, yf: np.ndarray, xr: np.ndarray, yr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    # 重なり範囲
    xmin = max(np.nanmin(xf), np.nanmin(xr))
    xmax = min(np.nanmax(xf), np.nanmax(xr))
    if not np.isfinite(xmin) or not np.isfinite(xmax) or xmax <= xmin:
        return np.array([]), np.array([]), np.array([]), np.nan

    step_f = median_step(xf)
    step_r = median_step(xr)
    steps = [s for s in [step_f, step_r] if np.isfinite(s) and s > 0]
    if not steps:
        return np.array([]), np.array([]), np.array([]), np.nan
    step = float(np.median(steps))

    # グリッド生成（端点を含む）
    n = int(np.floor((xmax - xmin) / step))
    if n < 1:
        return np.array([]), np.array([]), np.array([]), np.nan
    grid = xmin + step * np.arange(n + 1)

    yf_i = np.interp(grid, xf, yf)
    yr_i = np.interp(grid, xr, yr)
    return grid, yf_i, yr_i, step


def normalized_cross_correlation(a: np.ndarray, b: np.ndarray) -> tuple[int, float]:
    # a, b は同長の等間隔系列
    N = len(a)
    if N < 2:
        return 0, np.nan
    a = a.astype(float)
    b = b.astype(float)

    # 0-平均化
    a = a - np.mean(a)
    b = b - np.mean(b)

    # 標準偏差（ゼロ除算保護）
    std_a = np.std(a)
    std_b = np.std(b)
    if std_a == 0 or std_b == 0:
        return 0, np.nan

    # 全てのラグで NCC を評価（整数ラグ）
    best_lag = 0
    best_ncc = -np.inf
    for lag in range(-(N - 1), N):
        if lag < 0:
            a_seg = a[: N + lag]
            b_seg = b[-lag : N]
        elif lag > 0:
            a_seg = a[lag: N]
            b_seg = b[: N - lag]
        else:
            a_seg = a
            b_seg = b
        if len(a_seg) < 2:
            continue
        # 相関係数
        ncc = float(np.dot(a_seg, b_seg) / (len(a_seg) * std_a * std_b))
        if ncc > best_ncc:
            best_ncc = ncc
            best_lag = lag
    return best_lag, best_ncc


def compute_group_ncc(agg_group: pd.DataFrame, series_list: list[str], min_points: int) -> list[dict]:
    results = []
    for series in series_list:
        x, yf, yr = build_series_pair(agg_group, series)
        # 最低点数
        if len(x) < min_points:
            continue

        grid, yf_i, yr_i, step = interpolate_to_common_grid(x, yf, x, yr)
        if len(grid) < 2 or not np.isfinite(step):
            continue

        lag, ncc = normalized_cross_correlation(yf_i, yr_i)
        if not np.isfinite(ncc):
            continue

        dx = lag * step
        results.append({
            "series": series,
            "lag": int(lag),
            "grid_step": float(step),
            "dx": float(dx),
            "n_points_overlap": int(len(grid)),
            "ncc_max": float(ncc),
        })
    return results


def _format_val(v):
    try:
        if float(v).is_integer():
            return str(int(round(float(v))))
        return f"{float(v):g}"
    except Exception:
        return str(v)


def plot_group_overlay(agg_group: pd.DataFrame, series: str, dx: float, step: float, out_path: Path, title_suffix: str = "", dpi: int = 180):
    # データ再構築
    x, yf, yr = build_series_pair(agg_group, series)
    grid, yf_i, yr_i, step2 = interpolate_to_common_grid(x, yf, x, yr)
    if len(grid) < 2:
        return

    # 描画
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.plot(grid, yf_i, label="front", color="C0", marker="o")
    ax.plot(grid, yr_i, label="rear", color="C1", marker="o", linestyle="--", alpha=0.7)
    # シフト版（右へ +dx）
    ax.plot(grid + dx, yr_i, label=f"rear shifted by dx", color="C2", marker="o")

    ax.axhline(0, color="gray", linestyle="--", linewidth=1)
    ax.set_xlabel("flow_distance")
    ax.set_ylabel(series)

    if title_suffix:
        ax.set_title(title_suffix)
    ax.legend()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(str(out_path), dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def _aligned_series_with_lag(grid: np.ndarray, yf: np.ndarray, yr: np.ndarray, lag: int):
    # front (yf) と rear (yr) を整数ラグで同じ x に揃えて返す
    N = len(grid)
    if N < 2:
        return np.array([]), np.array([]), np.array([])
    if lag < 0:
        # rear を左に、front を右に詰める
        L = N + lag
        if L < 2:
            return np.array([]), np.array([]), np.array([])
        x_al = grid[:L]
        yf_al = yf[:L]
        yr_al = yr[-lag:]
    elif lag > 0:
        L = N - lag
        if L < 2:
            return np.array([]), np.array([]), np.array([])
        x_al = grid[lag:]
        yf_al = yf[lag:]
        yr_al = yr[:L]
    else:
        x_al = grid
        yf_al = yf
        yr_al = yr
    return x_al, yf_al, yr_al


def build_facet_long_df(agg: pd.DataFrame, result_rows: list[dict], series_list: list[str]) -> pd.DataFrame:
    # (distance, wall_spacing, tilt_angle, col_key, series) -> (dx, lag)
    key_to_dxlag = {}
    for r in result_rows:
        key = (r["distance"], r["wall_spacing"], r["tilt_angle"], r["col_key"], r["series"])
        key_to_dxlag[key] = (float(r["dx"]), int(r["lag"]))

    long_rows = []
    group_cols = ["distance", "wall_spacing", "tilt_angle", "col_key"]
    for keys, sub in agg.groupby(group_cols):
        for series in series_list:
            dxlag = key_to_dxlag.get((*keys, series))
            if dxlag is None:
                continue
            dx, lag = dxlag
            # front/rear を共通グリッドに補間
            x, yf, yr = build_series_pair(sub, series)
            grid, yf_i, yr_i, step = interpolate_to_common_grid(x, yf, x, yr)
            if len(grid) < 2:
                continue
            # 整数ラグで位置合わせ（x は共通）
            x_al, yf_al, yr_shift_al = _aligned_series_with_lag(grid, yf_i, yr_i, lag)
            if len(x_al) < 2:
                continue
            # front と rear_shifted をそれぞれ登録
            for xi, yi in zip(x_al, yf_al):
                long_rows.append({
                    "distance": keys[0],
                    "wall_spacing": keys[1],
                    "tilt_angle": keys[2],
                    "col_key": keys[3],
                    "flow_distance": float(xi),
                    "value": float(yi),
                    "series": series,
                    "line_kind": "front",
                })
            for xi, yi in zip(x_al, yr_shift_al):
                long_rows.append({
                    "distance": keys[0],
                    "wall_spacing": keys[1],
                    "tilt_angle": keys[2],
                    "col_key": keys[3],
                    "flow_distance": float(xi),
                    "value": float(yi),
                    "series": series,
                    "line_kind": "rear_shifted",
                })

    if not long_rows:
        return pd.DataFrame(columns=["distance", "wall_spacing", "tilt_angle", "col_key", "flow_distance", "value", "series", "line_kind"])
    df_long = pd.DataFrame(long_rows)
    df_long = df_long.sort_values(["distance", "wall_spacing", "tilt_angle", "series", "line_kind", "flow_distance"]).reset_index(drop=True)
    return df_long


def save_facet_figure(df_long: pd.DataFrame, out_path: Path, dpi: int = 180):
    if df_long.empty:
        print("ファセット図用のデータが空です。", file=sys.stderr)
        return
    sns.set_style("whitegrid")
    # 行=distance, 列=col_key, hue=series, style=line_kind
    row_order = sorted(df_long["distance"].dropna().unique().tolist())
    col_pairs = (
        df_long.drop_duplicates(subset=["wall_spacing", "tilt_angle"])[["wall_spacing", "tilt_angle", "col_key"]]
        .sort_values(["wall_spacing", "tilt_angle"])
    )
    col_order = col_pairs["col_key"].tolist()
    g = sns.FacetGrid(
        df_long,
        row="distance",
        col="col_key",
        hue="series",
        row_order=row_order,
        col_order=col_order,
        height=3.0,
        aspect=1.4,
        sharex=True,
        sharey=True,
        margin_titles=True,
        despine=False,
    )
    def _plot_kind(data, x, y, kind, **kwargs):
        sub = data[data["line_kind"] == kind]
        if len(sub) == 0:
            return
        ls = "-" if kind == "front" else "--"
        sns.lineplot(data=sub, x=x, y=y, hue="series", marker="o", linestyle=ls, legend=False)

    g.map_dataframe(_plot_kind, x="flow_distance", y="value", kind="front")
    g.map_dataframe(_plot_kind, x="flow_distance", y="value", kind="rear_shifted")
    g.add_legend(title="series")
    g.set_axis_labels("flow_distance", "value")
    # y=0 ライン
    for ax in g.axes.flat:
        ax.axhline(0, color="gray", linestyle="--", linewidth=1)
    # x 範囲を統一
    xmin = np.nanmin(df_long["flow_distance"].values)
    xmax = np.nanmax(df_long["flow_distance"].values)
    if np.isfinite(xmin) and np.isfinite(xmax):
        for ax in g.axes.flat:
            ax.set_xlim(xmin, xmax)
    # 追加の線種凡例（front / rear_shifted）を右下に追加
    front_line = Line2D([0], [0], color="black", linestyle="-", label="front")
    rear_line = Line2D([0], [0], color="black", linestyle="--", label="rear_shifted")
    # 最後の軸にのみ追加
    try:
        g.axes.flat[-1].legend(handles=[front_line, rear_line], title="kind", loc="lower right")
    except Exception:
        pass

    out_path.parent.mkdir(parents=True, exist_ok=True)
    g.savefig(str(out_path), dpi=dpi, bbox_inches="tight")


def main():
    args = parse_args()

    csv_path = Path(args.input)
    if not csv_path.exists():
        print(f"入力CSVが見つかりません: {csv_path}", file=sys.stderr)
        sys.exit(1)

    try:
        df = load_csv(str(csv_path))
    except Exception as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)

    if df.empty:
        print("入力CSVに有効なデータがありません。", file=sys.stderr)
        sys.exit(1)

    agg = aggregate_by_flow_distance(df)
    if agg.empty:
        print("集約後のデータが空です。", file=sys.stderr)
        sys.exit(1)

    series_list = [s.strip() for s in args.series.split(",") if s.strip()]
    valid = {"in", "out", "out-in"}
    for s in series_list:
        if s not in valid:
            print(f"不正な series: {s}. 許可: in,out,out-in", file=sys.stderr)
            sys.exit(1)

    rows = []
    group_cols = ["distance", "wall_spacing", "tilt_angle", "col_key"]
    for keys, sub in agg.groupby(group_cols):
        res = compute_group_ncc(sub, series_list, args.min_points)
        for r in res:
            rows.append({
                "distance": keys[0],
                "wall_spacing": keys[1],
                "tilt_angle": keys[2],
                "col_key": keys[3],
                **r,
            })

        # 図保存
        if args.plot_dir and res:
            for r in res:
                series = r["series"]
                dx = r["dx"]
                step = r["grid_step"]
                ncc = r["ncc_max"]
                d, w, t, ckey = keys
                sub_title = f"dist={_format_val(d)}, w={_format_val(w)}, tilt={_format_val(t)} | dx={dx:.3g}, NCC={ncc:.3f}"
                out_dir = Path(args.plot_dir) / series
                fname = f"dist-{_format_val(d)}__w-{_format_val(w)}__tilt-{_format_val(t)}.{args.plot_format}"
                out_path = out_dir / fname
                try:
                    plot_group_overlay(sub, series, dx=dx, step=step, out_path=out_path, title_suffix=sub_title, dpi=args.plot_dpi)
                except Exception as e:
                    print(f"図保存失敗: {out_path} ({e})", file=sys.stderr)

    result_df = pd.DataFrame(rows, columns=[
        "distance", "wall_spacing", "tilt_angle", "col_key",
        "series", "dx", "lag", "grid_step", "n_points_overlap", "ncc_max",
    ])

    # ファセット図の保存（要求がある場合）
    if args.facet_output:
        try:
            df_long = build_facet_long_df(agg, rows, series_list)
            out_path = Path(args.facet_output)
            save_facet_figure(df_long, out_path=out_path, dpi=args.facet_dpi)
            print(f"ファセット図を保存しました: {out_path}")
        except Exception as e:
            print(f"ファセット図の生成に失敗: {e}", file=sys.stderr)

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        result_df.to_csv(str(out_path), index=False)
        print(f"保存しました: {out_path}")
    else:
        # 標準出力
        result_df.to_csv(sys.stdout, index=False)


if __name__ == "__main__":
    main()


