import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


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
            "front - rear の差分を in / out / (out - in) の3系列で"
            " flow_distance を横軸としたファセットプロットに描画します。"
        )
    )
    parser.add_argument("--input", required=True, type=str, help="csv_concat3.py が出力した結合CSVのパス")
    parser.add_argument("--output", type=str, default=None, help="保存先画像パス（未指定なら画面表示）")
    parser.add_argument("--dpi", type=int, default=180, help="保存時のDPI")
    parser.add_argument("--style", type=str, default="whitegrid", help="seaborn style (default: whitegrid)")
    parser.add_argument("--palette", type=str, default="Set2", help="色パレット (in/out/out-in 用)")
    parser.add_argument("--moment", action="store_true", help="モーメントモード: y = (front-rear) × flow_distance を描画")
    parser.add_argument("--no-text", action="store_true", help="台形近似平均のテキスト注記を表示しない")
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

    # 数値化（失敗は NaN）
    for col in REQUIRED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # 解析対象のみ抽出
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


def compute_front_rear_differences(df: pd.DataFrame) -> pd.DataFrame:
    """front/rear の各値をキーごとに平均し、front - rear を算出。

    3系列:
      - in:  front_in - rear_in
      - out: front_out - rear_out
      - out-in: (front_out - front_in) - (rear_out - rear_in)
    """
    if df.empty:
        return df.iloc[0:0].copy()

    key_cols = ["distance", "tilt_angle", "wall_spacing", "flow_distance"]
    agg = (
        df.groupby(key_cols, as_index=False)[["front_in", "front_out", "rear_in", "rear_out"]]
        .mean()
        .dropna(subset=["front_in", "front_out", "rear_in", "rear_out"], how="any")
    )

    if agg.empty:
        return agg.iloc[0:0].copy()

    agg["in"] = agg["front_in"] - agg["rear_in"]
    agg["out"] = agg["front_out"] - agg["rear_out"]
    agg["out-in"] = (agg["front_out"] - agg["front_in"]) - (agg["rear_out"] - agg["rear_in"])  # = out - in

    agg["col_key"] = [make_col_key(w, t) for w, t in zip(agg["wall_spacing"], agg["tilt_angle"])]

    long_df = agg.melt(
        id_vars=key_cols + ["col_key"],
        value_vars=["in", "out", "out-in"],
        var_name="series",
        value_name="value",
    )

    # x 軸は flow_distance（正値）。並びを安定化
    long_df = long_df.sort_values(["distance", "tilt_angle", "wall_spacing", "series", "flow_distance"]).reset_index(drop=True)
    return long_df


def plot_front_rear_diff(long_df: pd.DataFrame, palette: str = "Set2", moment: bool = False, show_text: bool = True) -> sns.axisgrid.FacetGrid:
    sns.set_style("whitegrid")

    data = long_df.copy()
    if moment:
        data["y_val"] = data["value"] * data["flow_distance"]
        y_col = "y_val"
    else:
        y_col = "value"

    row_order = sorted(data["distance"].dropna().unique().tolist())
    series_order = ["in", "out", "out-in"]
    series_colors = sns.color_palette(palette, n_colors=len(series_order))
    series_color = {series_order[i]: series_colors[i] for i in range(len(series_order))}
    col_pairs = (
        data.drop_duplicates(subset=["wall_spacing", "tilt_angle"])[["wall_spacing", "tilt_angle", "col_key"]]
        .sort_values(["wall_spacing", "tilt_angle"])
    )
    col_order = col_pairs["col_key"].tolist()

    g = sns.FacetGrid(
        data,
        row="distance",
        col="col_key",
        hue="series",
        hue_order=series_order,
        row_order=row_order,
        col_order=col_order,
        sharex=True,
        sharey=True,
        margin_titles=True,
        despine=False,
        height=3.0,
        aspect=1.4,
        palette=palette,
    )

    g.map_dataframe(sns.lineplot, x="flow_distance", y=y_col, marker="o")
    g.add_legend(title="series")
    if moment:
        g.set_axis_labels("flow_distance", "moment = (front - rear) × flow_distance")
    else:
        g.set_axis_labels("flow_distance", "front - rear [m/s]")

    # y=0 の基準線
    for ax in g.axes.flat:
        ax.axhline(0, color="gray", linestyle="--", linewidth=1)

    # x 範囲を統一
    xmin = np.nanmin(data["flow_distance"].values) if len(data) else None
    xmax = np.nanmax(data["flow_distance"].values) if len(data) else None
    if xmin is not None and xmax is not None:
        for ax in g.axes.flat:
            ax.set_xlim(xmin, xmax)

    # 各ファセットに series ごとの台形近似平均をテキスト注記で表示
    if show_text:
        means = compute_trapz_mean_by_series(data, y_col=y_col, x_col="flow_distance")
        if not means.empty:
            row_index = {val: i for i, val in enumerate(row_order)}
            col_index = {val: j for j, val in enumerate(col_order)}
            # 系列の表示順
            for (d, ckey), sub in means.groupby(["distance", "col_key"]):
                if d not in row_index or ckey not in col_index:
                    continue
                ax = g.axes[row_index[d], col_index[ckey]]
                # 上部に系列ごとに段積みで表示
                y0 = 0.98
                dy = 0.08
                k = 0
                # series_order の順で並べる
                for s in series_order:
                    row = sub[sub["series"] == s]
                    if row.empty:
                        continue
                    mean_val = float(row["mean"].iloc[0]) if np.isfinite(row["mean"].iloc[0]) else None
                    if mean_val is None:
                        continue
                    txt = f"{s}: {mean_val:+.3f}"
                    ax.text(0.02, y0 - k * dy, txt, transform=ax.transAxes, ha="left", va="top", color=series_color.get(s, "black"))
                    k += 1

    return g


def compute_trapz_mean_by_series(df: pd.DataFrame, y_col: str, x_col: str = "flow_distance") -> pd.DataFrame:
    """各 (distance, col_key, series) ごとに台形近似平均を計算。"""
    if df.empty:
        return df.iloc[0:0].copy()
    results = []
    for keys, sub in df.groupby(["distance", "col_key", "series"]):
        sub = sub.dropna(subset=[x_col, y_col]).sort_values(x_col)
        if len(sub) < 2:
            continue
        x_min = float(sub[x_col].min())
        x_max = float(sub[x_col].max())
        width = x_max - x_min
        if not np.isfinite(width) or width == 0:
            continue
        mean_val = float(np.trapz(sub[y_col].values, sub[x_col].values) / width)
        results.append({
            "distance": keys[0],
            "col_key": keys[1],
            "series": keys[2],
            "mean": mean_val,
        })
    if not results:
        return df.iloc[0:0].copy()
    return pd.DataFrame(results)


def main():
    args = parse_args()
    sns.set(context="talk", style=args.style)

    csv_path = Path(args.input)
    if not csv_path.exists():
        print(f"入力CSVが見つかりません: {csv_path}", file=sys.stderr)
        sys.exit(1)

    df = load_csv(str(csv_path))
    if df.empty:
        print("入力CSVに有効なデータがありません。", file=sys.stderr)
        sys.exit(1)

    long_df = compute_front_rear_differences(df)
    if long_df.empty:
        print("front-rear 差分の計算結果が空です。", file=sys.stderr)
        sys.exit(1)

    g = plot_front_rear_diff(long_df, palette=args.palette, moment=args.moment, show_text=not args.no_text)

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        g.savefig(str(out_path), dpi=args.dpi, bbox_inches="tight")
        print(f"保存しました: {out_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()



