import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="flow_distance を横軸、風速(front/rear in/out)を縦軸にしたファセットプロットを作成します。"
    )
    parser.add_argument("--input", required=True, type=str, help="csv_concat3.py が出力した結合CSVのパス")
    parser.add_argument("--output", type=str, default=None, help="保存先画像パス（未指定なら画面表示）")
    parser.add_argument("--dpi", type=int, default=180, help="保存時のDPI")
    parser.add_argument("--style", type=str, default="whitegrid", help="seaborn style (default: whitegrid)")
    parser.add_argument(
        "--palette", type=str, default="Set2", help="色パレット (in/out 用)"
    )
    return parser.parse_args()


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
    for col in ["distance", "tilt_angle", "wall_spacing", "flow_distance", "front_in", "front_out", "rear_in", "rear_out"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # 解析対象のみ抽出（flow_distance が存在）
    df = df.dropna(subset=["distance", "tilt_angle", "wall_spacing", "flow_distance"]).copy()
    return df


def reshape_and_aggregate(df: pd.DataFrame) -> pd.DataFrame:
    # ワイド -> ロング
    long_df = df.melt(
        id_vars=["distance", "tilt_angle", "wall_spacing", "flow_distance"],
        value_vars=["front_in", "front_out", "rear_in", "rear_out"],
        var_name="probe",
        value_name="wind_speed",
    )

    # 風速を数値化し欠損除去
    long_df["wind_speed"] = pd.to_numeric(long_df["wind_speed"], errors="coerce")
    long_df = long_df.dropna(subset=["wind_speed"]).copy()

    # in/out, front/rear を抽出
    long_df["io"] = np.where(long_df["probe"].str.endswith("in"), "in", "out")
    long_df["side"] = np.where(long_df["probe"].str.startswith("front"), "front", "rear")

    # x 軸（rear は反転）
    long_df["x"] = np.where(long_df["side"] == "rear", -long_df["flow_distance"], long_df["flow_distance"])

    # 指定: tilt_angle と distance でグルーピングし平均
    # ただし in/out は別線、x も点の同一性に必要なのでキーに含める
    agg = (
        long_df.groupby(["distance", "tilt_angle", "wall_spacing", "x", "io"], as_index=False)["wind_speed"].mean()
    )

    # 描画順の安定化
    agg = agg.sort_values(["distance", "tilt_angle", "wall_spacing", "io", "x"]).reset_index(drop=True)

    # ファセット列キー: (w=wall_spacing, tilt=tilt_angle)
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

    agg["col_key"] = [make_col_key(w, t) for w, t in zip(agg["wall_spacing"], agg["tilt_angle"])]
    return agg


def plot_facet(agg: pd.DataFrame, palette: str = "Set2") -> sns.axisgrid.FacetGrid:
    sns.set_style("whitegrid")
    # 行=distance (昇順), 列=tilt_angle (昇順)
    row_order = sorted(agg["distance"].dropna().unique().tolist())
    # 列順は (wall_spacing, tilt_angle) の順で並べる
    col_pairs = (
        agg.drop_duplicates(subset=["wall_spacing", "tilt_angle"])[["wall_spacing", "tilt_angle", "col_key"]]
        .sort_values(["wall_spacing", "tilt_angle"])
    )
    col_order = col_pairs["col_key"].tolist()

    g = sns.FacetGrid(
        agg,
        row="distance",
        col="col_key",
        hue="io",
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

    g.map_dataframe(sns.lineplot, x="x", y="wind_speed", marker="o")
    g.add_legend(title="io")
    g.set_axis_labels("flow_distance (rear は負)", "wind speed [m/s]")

    # 軸の微調整: x を原点対称に近づける（各面の範囲を揃える）
    # ここでは全体の x 範囲を取得して統一
    xmin = np.nanmin(agg["x"].values) if len(agg) else None
    xmax = np.nanmax(agg["x"].values) if len(agg) else None
    if xmin is not None and xmax is not None:
        lim = max(abs(xmin), abs(xmax))
        for ax in g.axes.flat:
            ax.set_xlim(-lim, lim)

    return g


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
    if agg.empty:
        print("集約後のデータが空です。", file=sys.stderr)
        sys.exit(1)

    g = plot_facet(agg, palette=args.palette)

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        g.savefig(str(out_path), dpi=args.dpi, bbox_inches="tight")
        print(f"保存しました: {out_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()


