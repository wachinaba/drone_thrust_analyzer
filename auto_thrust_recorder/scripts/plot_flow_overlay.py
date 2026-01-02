import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# 既存ユーティリティを流用（直接実行とモジュール実行の双方で動作）
try:
    from .plot_flow_facet import (
        load_csv,
        reshape_and_aggregate,
        reshape_to_long,
        compute_sidewise_trapz_mean_inout,
    )
except Exception:
    THIS_DIR = Path(__file__).resolve().parent
    if str(THIS_DIR) not in sys.path:
        sys.path.insert(0, str(THIS_DIR))
    from plot_flow_facet import (  # type: ignore
        load_csv,
        reshape_and_aggregate,
        reshape_to_long,
        compute_sidewise_trapz_mean_inout,
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="flow_distance を横軸、front と rear を同一座標に重ねて描画します。"
    )
    parser.add_argument("--input", required=True, type=str, help="csv_concat3.py が出力した結合CSVのパス")
    parser.add_argument("--output", type=str, default=None, help="保存先画像パス（未指定なら画面表示）")
    parser.add_argument("--dpi", type=int, default=180, help="保存時のDPI")
    parser.add_argument("--style", type=str, default="whitegrid", help="seaborn style (default: whitegrid)")
    parser.add_argument(
        "--palette", type=str, default="Set1", help="色パレット (side 用; front/rear)"
    )
    parser.add_argument(
        "--markers", action="store_true", help="散布点をオーバーレイ（front/rear × in/out）"
    )
    parser.add_argument(
        "--moment",
        action="store_true",
        help="モーメントモード: y = wind_speed × position を描画（position = |x|）",
    )
    parser.add_argument(
        "--moment-signed",
        action="store_true",
        help="モーメントの position に符号付き x を使用（rear は負）",
    )
    parser.add_argument(
        "--split-by-flowdir",
        action="store_true",
        help="flowdir 列でグルーピングし、flowdir ごとに別グラフを生成",
    )
    return parser.parse_args()


def _compute_side_from_x(df: pd.DataFrame, x_col: str = "x") -> pd.Series:
    return np.where(df[x_col] < 0, "rear", "front")


def plot_overlay(agg: pd.DataFrame, raw_long: pd.DataFrame, palette: str = "Set1", show_markers: bool = False, moment: bool = False, moment_signed: bool = False) -> sns.axisgrid.FacetGrid:
    sns.set_style("whitegrid")

    # サイド列を追加（x の符号から判定）
    df = agg.copy()
    df["side"] = _compute_side_from_x(df, "x")
    # rear の x 反転をやめ、正方向で front/rear を重ねる
    df["x_pos"] = df["x"].abs()
    # y 値（速度 or モーメント）
    if moment:
        position = df["x"] if moment_signed else df["x_pos"]
        df["y_val"] = df["wind_speed"] * position
    else:
        df["y_val"] = df["wind_speed"]

    # 行=distance、列=(wall_spacing, tilt_angle)（plot_flow_facet と同順）
    row_order = sorted(df["distance"].dropna().unique().tolist())
    col_pairs = (
        df.drop_duplicates(subset=["wall_spacing", "tilt_angle"])[["wall_spacing", "tilt_angle", "col_key"]]
        .sort_values(["wall_spacing", "tilt_angle"])
    )
    col_order = col_pairs["col_key"].tolist()

    # side 色マップ
    side_order = ["front", "rear"]
    pal = sns.color_palette(palette, n_colors=len(side_order))
    side_color = {side_order[i]: pal[i] for i in range(len(side_order))}

    g = sns.FacetGrid(
        df,
        row="distance",
        col="col_key",
        hue="side",
        hue_order=side_order,
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

    # in/out は線種で区別。x は正方向の値で描画（凡例登録は抑止）
    g.map_dataframe(
        sns.lineplot,
        x="x_pos",
        y="y_val",
        style="io",
        style_order=["in", "out"],
        legend=False,
    )  # hue は FacetGrid が担当

    # 生データの散布点
    if show_markers and raw_long is not None and not raw_long.empty:
        # raw_long に side 列を用意
        rl = raw_long.copy()
        if "side" not in rl.columns:
            rl["side"] = _compute_side_from_x(rl, "x")
        rl["x_pos"] = rl["x"].abs()
        if moment:
            position_rl = rl["x"] if moment_signed else rl["x_pos"]
            rl["y_val"] = rl["wind_speed"] * position_rl
        else:
            rl["y_val"] = rl["wind_speed"]

        # Facet 位置インデックス
        row_index = {val: i for i, val in enumerate(row_order)}
        col_index = {val: j for j, val in enumerate(col_order)}

        for (d, ckey), sub in rl.groupby(["distance", "col_key"]):
            if d not in row_index or ckey not in col_index:
                continue
            ax = g.axes[row_index[d], col_index[ckey]]
            sns.scatterplot(
                data=sub,
                x="x_pos",
                y="y_val",
                hue="side",
                hue_order=side_order,
                palette=side_color,
                style="io",
                style_order=["in", "out"],
                s=16,
                alpha=1.0,
                edgecolor="none",
                legend=False,
                ax=ax,
                zorder=2,
            )

    g.add_legend(title="side")
    if moment:
        ylabel = "moment = wind_speed × position"
    else:
        ylabel = "wind speed [m/s]"
    g.set_axis_labels("flow_distance", ylabel)

    # x 範囲を原点対称に
    xmax = np.nanmax(df["x_pos"].values) if len(df) else None
    if xmax is not None:
        for ax in g.axes.flat:
            ax.set_xlim(0.0, xmax)

    # 平均矩形（io × side）。描画は正の x で行う
    try:
        value_col = "wind_speed" if not moment else "y_val"
        overlay_inout_mean_rectangles_positive(g, df if moment else agg, value_col=value_col, palette=palette, alpha=0.20)
    except Exception as e:
        print(f"平均長方形の描画に失敗: {e}", file=sys.stderr)

    return g


def overlay_inout_mean_rectangles_positive(g: sns.axisgrid.FacetGrid, df_source: pd.DataFrame, value_col: str = "wind_speed", palette: str = "Set2", alpha: float = 0.20):
    """台形近似平均を front/rear 別・in/out 別に計算し、正の x 軸上に矩形を描画する。

    value_col で台形近似平均に用いる列を指定（"wind_speed" や "y_val"）。
    df_source は少なくとも列 [distance, col_key, io, x, value_col, wall_spacing, tilt_angle] を含むこと。
    """
    avg_df = compute_sidewise_trapz_mean_generic(df_source, value_col=value_col, x_col="x")
    if avg_df.empty:
        return

    # Facet 配置
    row_order = sorted(df_source["distance"].dropna().unique().tolist())
    col_pairs = df_source.drop_duplicates(subset=["wall_spacing", "tilt_angle"])[["wall_spacing", "tilt_angle", "col_key"]].sort_values(["wall_spacing", "tilt_angle"])
    col_order = col_pairs["col_key"].tolist()

    row_index = {val: i for i, val in enumerate(row_order)}
    col_index = {val: j for j, val in enumerate(col_order)}

    # io ごとの色
    io_order = ["in", "out"]
    pal = sns.color_palette(palette, n_colors=len(io_order))
    io_color = {io_order[i]: pal[i] for i in range(len(io_order))}

    for _, r in avg_df.iterrows():
        d = r["distance"]
        ckey = r["col_key"]
        io = r["io"]
        x_min = float(r["x_min"]) if np.isfinite(r["x_min"]) else None
        x_max = float(r["x_max"]) if np.isfinite(r["x_max"]) else None
        mean_val = float(r["mean"]) if np.isfinite(r["mean"]) else None
        if d not in row_index or ckey not in col_index:
            continue
        if x_min is None or x_max is None or mean_val is None:
            continue

        # 正の x 軸上での左端/右端
        left = min(abs(x_min), abs(x_max))
        right = max(abs(x_min), abs(x_max))
        width = right - left
        if not np.isfinite(width) or width <= 0:
            continue

        i = row_index[d]
        j = col_index[ckey]
        ax = g.axes[i, j]

        y = min(0.0, mean_val)
        height = abs(mean_val)
        rect = plt.Rectangle((left, y), width, height, facecolor=io_color.get(io, "gray"), edgecolor=None, alpha=alpha)
        ax.add_patch(rect)


def compute_sidewise_trapz_mean_generic(df: pd.DataFrame, value_col: str, x_col: str = "x") -> pd.DataFrame:
    """各 (distance, col_key, io, side) 内で台形近似平均を計算（value_col を対象）。

    出力列: distance, col_key, io, side, x_min, x_max, mean
    """
    if df.empty:
        return df.iloc[0:0].copy()

    data = df.copy()
    if "side" not in data.columns:
        data["side"] = _compute_side_from_x(data, x_col)

    results = []
    group_keys = ["distance", "col_key", "io", "side"]
    for keys, sub in data.groupby(group_keys):
        sub = sub.dropna(subset=[x_col, value_col]).sort_values(x_col)
        if len(sub) < 2:
            continue
        x_min = float(sub[x_col].min())
        x_max = float(sub[x_col].max())
        width = x_max - x_min
        if not np.isfinite(width) or width == 0:
            continue
        mean_val = float(np.trapz(sub[value_col].values, sub[x_col].values) / width)
        results.append({
            "distance": keys[0],
            "col_key": keys[1],
            "io": keys[2],
            "side": keys[3],
            "x_min": x_min,
            "x_max": x_max,
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

    agg = reshape_and_aggregate(df)
    raw_long = reshape_to_long(df)
    if agg.empty:
        print("集約後のデータが空です。", file=sys.stderr)
        sys.exit(1)

    # flowdir で分割描画（元データ df を基準にフィルタ → 再集約）
    if args.split_by_flowdir:
        if "flowdir" in df.columns:
            flowdirs = [v for v in df["flowdir"].dropna().unique().tolist()]
            if flowdirs:
                for fd in flowdirs:
                    sub_df = df[df["flowdir"] == fd]
                    if sub_df.empty:
                        continue
                    sub_agg = reshape_and_aggregate(sub_df)
                    sub_raw = reshape_to_long(sub_df)
                    if sub_agg.empty:
                        continue
                    g = plot_overlay(
                        sub_agg,
                        sub_raw,
                        palette=args.palette,
                        show_markers=args.markers,
                        moment=args.moment or args.moment_signed,
                        moment_signed=args.moment_signed,
                    )
                    try:
                        g.fig.suptitle(f"flowdir = {fd}")
                    except Exception:
                        pass
                    if args.output:
                        out_path = Path(args.output)
                        out_path.parent.mkdir(parents=True, exist_ok=True)
                        base = out_path.stem
                        suffix = out_path.suffix or ".png"
                        out_name = f"{base}_flowdir-{str(fd).replace(' ', '_')}{suffix}"
                        out_file = out_path.with_name(out_name)
                        g.savefig(str(out_file), dpi=args.dpi, bbox_inches="tight")
                        print(f"保存しました: {out_file}")
                if not args.output:
                    plt.show()
                # flowdir 分割が完了したので終了
                return
        else:
            print("flowdir 列が見つからないため、分割せずに描画します。", file=sys.stderr)
    else:
        g = plot_overlay(
            agg,
            raw_long,
            palette=args.palette,
            show_markers=args.markers,
            moment=args.moment or args.moment_signed,
            moment_signed=args.moment_signed,
        )
        if args.output:
            out_path = Path(args.output)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            g.savefig(str(out_path), dpi=args.dpi, bbox_inches="tight")
            print(f"保存しました: {out_path}")
        else:
            plt.show()


if __name__ == "__main__":
    main()


