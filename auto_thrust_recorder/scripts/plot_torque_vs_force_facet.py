#!/usr/bin/env python3
"""
distance, force_z, torque_x, tilt_angle を含む CSV から、

- x 軸: force_z
- y 軸: torque_x / force_z
- 色: distance
- ファセット: tilt_angle ごとのサブプロット

の散布図を作成するスクリプト。

プロットは常に画面表示し、オプション指定時のみファイルに保存します。
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns


REQUIRED_COLUMNS = ["distance", "force_z", "torque_x", "tilt_angle"]


def parse_arguments() -> argparse.Namespace:
    """コマンドライン引数を解析する。"""
    parser = argparse.ArgumentParser(
        description=(
            "distance, force_z, torque_x, tilt_angle を含む CSV から、"
            "x=force_z, y=torque_x/force_z, 色=distance の散布図を "
            "tilt_angle ごとにファセット表示します。"
        )
    )
    parser.add_argument(
        "csv_file",
        help="入力 CSV ファイルのパス（distance, force_z, torque_x, tilt_angle 列を含むこと）",
    )
    parser.add_argument(
        "--distance-min",
        type=float,
        default=None,
        help="distance の下限（指定時は distance >= この値 のデータのみ使用）",
    )
    parser.add_argument(
        "--distance-max",
        type=float,
        default=None,
        help="distance の上限（指定時は distance <= この値 のデータのみ使用）",
    )
    parser.add_argument(
        "-o",
        "--save",
        dest="save",
        default=None,
        help="プロット画像の保存先パス（指定しない場合は保存しません）",
    )
    return parser.parse_args()


def _validate_columns(df: pd.DataFrame) -> None:
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"CSV に必須列が不足しています: {', '.join(missing)}")


def load_and_prepare_data(csv_path: Path) -> pd.DataFrame:
    """CSV を読み込み、必要列の数値化と torque_ratio 列の作成を行う。"""
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        raise RuntimeError(f"CSV 読込に失敗しました: {e}")

    _validate_columns(df)

    # 必要列のみ数値化
    for col in REQUIRED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # 必須列に NaN を含む行を除外
    df = df.dropna(subset=REQUIRED_COLUMNS).copy()
    if df.empty:
        return df

    # force_z = 0 は 0 除算を避けるため除外
    zero_force_mask = df["force_z"] == 0
    if zero_force_mask.any():
        dropped = int(zero_force_mask.sum())
        print(
            f"警告: force_z が 0 の行を {dropped} 件除外しました（torque_x/force_z の計算のため）。",
            file=sys.stderr,
        )
        df = df.loc[~zero_force_mask].copy()

    if df.empty:
        return df

    # 比 torque_x / force_z を計算
    df["torque_ratio"] = df["torque_x"] / df["force_z"]
    # 数値異常の除外（inf, NaN など）
    df["torque_ratio"] = pd.to_numeric(df["torque_ratio"], errors="coerce")
    df = df.dropna(subset=["torque_ratio"]).copy()
    return df


def create_facet_plot(df: pd.DataFrame) -> sns.axisgrid.FacetGrid:
    """tilt_angle ごとにファセットした散布図 (x=force_z, y=torque_x/force_z, 色=distance)。"""
    if df.empty:
        raise ValueError("プロット対象データが空です。")

    sns.set(context="talk", style="whitegrid")

    # tilt_angle の昇順で列順を固定
    tilt_order = sorted(df["tilt_angle"].dropna().unique().tolist())

    g = sns.relplot(
        data=df,
        x="force_z",
        y="torque_ratio",
        hue="distance",
        col="tilt_angle",
        col_order=tilt_order,
        kind="scatter",
        palette="viridis",
        height=4.0,
        aspect=1.1,
    )

    g.set_axis_labels("force_z [N]", "torque_x / force_z")
    g.fig.suptitle(
        "Torque ratio vs force_z (colored by distance, faceted by tilt_angle)",
        y=0.95,
    )
    # 右側にカラーバー用の余白、上下にタイトル／ラベル用の余白を確保
    g.fig.subplots_adjust(left=0.10, right=0.85, top=0.88, bottom=0.12)
    return g


def main() -> None:
    args = parse_arguments()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"エラー: 入力 CSV が見つかりません: {csv_path}", file=sys.stderr)
        sys.exit(1)

    try:
        df = load_and_prepare_data(csv_path)
    except Exception as e:
        print(f"エラー: {e}", file=sys.stderr)
        sys.exit(1)

    if df.empty:
        print("有効なデータがありません（必須列の欠損や force_z=0 の除外後に空になりました）。", file=sys.stderr)
        sys.exit(1)

    # distance 範囲によるフィルタ
    before_len = len(df)
    if args.distance_min is not None:
        df = df[df["distance"] >= args.distance_min].copy()
    if args.distance_max is not None:
        df = df[df["distance"] <= args.distance_max].copy()

    if len(df) == 0:
        print(
            "distance 範囲フィルタ適用後に有効なデータがありません。",
            file=sys.stderr,
        )
        sys.exit(1)
    elif len(df) < before_len:
        print(
            f"distance フィルタにより {before_len - len(df)} 行が除外されました（残り {len(df)} 行）。",
            file=sys.stderr,
        )

    try:
        g = create_facet_plot(df)
    except Exception as e:
        print(f"プロットの作成中にエラーが発生しました: {e}", file=sys.stderr)
        sys.exit(1)

    # オプション指定時のみ保存
    if args.save:
        out_path = Path(args.save)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            g.fig.savefig(str(out_path), dpi=150, bbox_inches="tight")
            print(f"プロットを保存しました: {out_path}")
        except Exception as e:
            print(f"警告: プロットの保存に失敗しました: {e}", file=sys.stderr)

    # 常に画面表示
    plt.show()


if __name__ == "__main__":
    main()


