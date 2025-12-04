import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "torque_x, distance, force_z を含む CSV に対して "
            "torque_x = f(distance) * force_z^n モデルをあてはめ、"
            "指数 n の推定とモデル妥当性の可視化を行うスクリプト。"
        )
    )
    parser.add_argument(
        "--csv",
        required=True,
        type=str,
        help="入力CSVファイルのパス",
    )
    parser.add_argument(
        "--torque-col",
        type=str,
        default="torque_x",
        help="トルク列名（デフォルト: torque_x）",
    )
    parser.add_argument(
        "--distance-col",
        type=str,
        default="distance",
        help="距離列名（デフォルト: distance）",
    )
    parser.add_argument(
        "--tilt-col",
        type=str,
        default=None,
        help=(
            "チルト角などの列名。指定した場合、この列でグルーピングして n を推定し、"
            "loglog ファセットプロットを作成します（例: tilt_angle）"
        ),
    )
    parser.add_argument(
        "--min-distance",
        type=float,
        default=None,
        help="距離の下限値。指定した場合、distance >= min_distance のデータのみを使用",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=None,
        help="距離の上限値。指定した場合、distance <= max_distance のデータのみを使用",
    )
    parser.add_argument(
        "--force-col",
        type=str,
        default="force_z",
        help="力列名（デフォルト: force_z）",
    )
    parser.add_argument(
        "--distance-degree",
        type=int,
        default=2,
        help=(
            "distance に対する多項式次数（0 以上の整数）。"
            "例: 0=定数, 1=一次, 2=二次（デフォルト: 2）"
        ),
    )
    parser.add_argument(
        "--min-force",
        type=float,
        default=1e-6,
        help="log(force_z) を取るために残す最小 force_z（デフォルト: 1e-6）",
    )
    parser.add_argument(
        "--min-abs-torque",
        type=float,
        default=1e-6,
        help="log(|torque_x|) を取るために残す最小 |torque_x|（デフォルト: 1e-6）",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="torque_model_plots",
        help="プロット画像の出力ディレクトリ（デフォルト: torque_model_plots）",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=180,
        help="保存時の DPI（デフォルト: 180）",
    )
    parser.add_argument(
        "--style",
        type=str,
        default="whitegrid",
        help="seaborn のスタイル名（デフォルト: whitegrid）",
    )
    return parser.parse_args()


def load_and_prepare_data(
    csv_path: Path,
    torque_col: str,
    distance_col: str,
    force_col: str,
    min_force: float,
    min_abs_torque: float,
    min_distance=None,
    max_distance=None,
) -> pd.DataFrame:
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        raise RuntimeError(f"CSV 読み込みに失敗しました: {e}")

    missing = [c for c in (torque_col, distance_col, force_col) if c not in df.columns]
    if missing:
        raise ValueError(f"必須列が不足しています: {', '.join(missing)}")

    df = df.copy()
    for c in (torque_col, distance_col, force_col):
        df[c] = pd.to_numeric(df[c], errors="coerce")

    df["abs_torque"] = df[torque_col].abs()
    df = df.replace([np.inf, -np.inf], np.nan)

    mask = (
        df[force_col].notna()
        & df[distance_col].notna()
        & df["abs_torque"].notna()
        & (df[force_col] > min_force)
        & (df["abs_torque"] > min_abs_torque)
    )

    if min_distance is not None:
        mask &= df[distance_col] >= float(min_distance)
    if max_distance is not None:
        mask &= df[distance_col] <= float(max_distance)
    df = df[mask].copy()

    if df.empty:
        raise ValueError(
            "有効なデータがありません。min_force や min_abs_torque の設定、"
            "または入力CSVの値を確認してください。"
        )

    df["log_force"] = np.log(df[force_col].astype(float))
    df["log_abs_torque"] = np.log(df["abs_torque"].astype(float))

    if not np.isfinite(df["log_force"]).any() or not np.isfinite(
        df["log_abs_torque"]
    ).any():
        raise ValueError("log(force_z) または log(|torque_x|) の有効な値がありません。")

    return df


def build_design_matrix(distance: np.ndarray, log_force: np.ndarray, degree: int) -> np.ndarray:
    if degree < 0:
        raise ValueError("distance-degree は 0 以上の整数で指定してください。")

    distance = np.asarray(distance, dtype=float)
    log_force = np.asarray(log_force, dtype=float)

    # [1, d, d^2, ..., d^degree, logF] のデザイン行列
    cols = [np.ones_like(distance)]
    for p in range(1, degree + 1):
        cols.append(distance ** p)
    cols.append(log_force)
    X = np.column_stack(cols)
    return X


def fit_log_model(
    distance: np.ndarray,
    log_force: np.ndarray,
    log_abs_torque: np.ndarray,
    degree: int,
) -> dict:
    X = build_design_matrix(distance, log_force, degree)
    y = np.asarray(log_abs_torque, dtype=float)

    if X.shape[0] <= X.shape[1]:
        raise ValueError(
            "サンプル数がパラメータ数以下です。"
            "distance-degree を小さくするか、データ数を増やしてください。"
        )

    beta, residuals, rank, s = np.linalg.lstsq(X, y, rcond=None)
    y_pred = X @ beta
    resid = y - y_pred

    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan

    n = float(beta[-1])

    # パラメータの標準誤差（特に n の不確かさ）を推定
    dof = X.shape[0] - X.shape[1]
    if dof > 0 and ss_res > 0:
        sigma2 = ss_res / dof
        try:
            xtx_inv = np.linalg.inv(X.T @ X)
        except np.linalg.LinAlgError:
            xtx_inv = np.linalg.pinv(X.T @ X)
        cov_beta = sigma2 * xtx_inv
        n_var = float(cov_beta[-1, -1])
        n_se = float(np.sqrt(n_var)) if n_var > 0 else np.nan
    else:
        n_se = np.nan

    return {
        "beta": beta,
        "n": n,
        "n_se": n_se,
        "r2": r2,
        "y_pred": y_pred,
        "residuals": resid,
        "X": X,
    }


def ensure_output_dir(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)


def plot_loglog_with_fit(
    df: pd.DataFrame,
    distance_col: str,
    result: dict,
    output_path: Path,
    dpi: int,
) -> None:
    plt.figure(figsize=(6, 5))
    sc = plt.scatter(
        df["log_force"],
        df["log_abs_torque"],
        c=df[distance_col],
        cmap="viridis",
        s=20,
        alpha=0.7,
    )
    cbar = plt.colorbar(sc)
    cbar.set_label(f"{distance_col}")

    beta = result["beta"]
    n = result["n"]
    degree = len(beta) - 1 - 1  # [1, d,...,d^deg, logF] -> deg = len(beta)-2

    # 代表距離（median）でのフィット直線を描く
    d0 = float(np.median(df[distance_col].to_numpy(dtype=float)))
    const_terms = [beta[0]]
    for p in range(1, degree + 1):
        const_terms.append(beta[p] * (d0 ** p))
    c0 = float(np.sum(const_terms))

    x_line = np.linspace(df["log_force"].min(), df["log_force"].max(), 100)
    y_line = c0 + n * x_line
    plt.plot(
        x_line,
        y_line,
        color="red",
        linestyle="--",
        linewidth=2,
        label=f"fit at {distance_col}={d0:.3g}",
    )
    plt.legend()

    plt.xlabel("log(force_z)")
    plt.ylabel("log(|torque_x|)")
    plt.title("log(|torque_x|) vs log(force_z)")
    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi)
    plt.close()


def plot_normalized_torque_vs_distance(
    df: pd.DataFrame,
    distance_col: str,
    force_col: str,
    n: float,
    output_path: Path,
    dpi: int,
) -> None:
    norm_torque = df["abs_torque"] / np.power(df[force_col].to_numpy(dtype=float), n)

    plt.figure(figsize=(6, 5))
    plt.scatter(
        df[distance_col],
        norm_torque,
        s=20,
        alpha=0.7,
    )
    plt.xlabel(distance_col)
    plt.ylabel(r"|torque_x| / force_z^n")
    plt.title("Normalized torque vs distance")
    plt.grid(True, linestyle=":", alpha=0.7)
    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi)
    plt.close()


def plot_residuals(
    df: pd.DataFrame,
    distance_col: str,
    force_col: str,
    residuals: np.ndarray,
    output_dir: Path,
    dpi: int,
) -> None:
    # 残差 vs distance
    plt.figure(figsize=(6, 4))
    plt.scatter(df[distance_col], residuals, s=20, alpha=0.7)
    plt.axhline(0.0, color="red", linestyle="--", linewidth=1)
    plt.xlabel(distance_col)
    plt.ylabel("residual (log(|torque_x|) - model)")
    plt.title("Residuals vs distance")
    plt.grid(True, linestyle=":", alpha=0.7)
    plt.tight_layout()
    plt.savefig(output_dir / "residuals_vs_distance.png", dpi=dpi)
    plt.close()

    # 残差 vs force_z
    plt.figure(figsize=(6, 4))
    plt.scatter(df[force_col], residuals, s=20, alpha=0.7)
    plt.axhline(0.0, color="red", linestyle="--", linewidth=1)
    plt.xlabel(force_col)
    plt.ylabel("residual (log(|torque_x|) - model)")
    plt.title("Residuals vs force_z")
    plt.grid(True, linestyle=":", alpha=0.7)
    plt.tight_layout()
    plt.savefig(output_dir / "residuals_vs_force_z.png", dpi=dpi)
    plt.close()


def plot_torque_sign_hist(
    df_raw: pd.DataFrame,
    torque_col: str,
    output_path: Path,
    dpi: int,
) -> None:
    vals = df_raw[torque_col].dropna()
    if vals.empty:
        return
    signs = np.sign(vals.to_numpy(dtype=float))
    labels = ["negative", "zero", "positive"]
    counts = [
        int(np.sum(signs < 0)),
        int(np.sum(signs == 0)),
        int(np.sum(signs > 0)),
    ]

    if sum(counts) == 0:
        return

    plt.figure(figsize=(4, 4))
    plt.bar(labels, counts)
    plt.ylabel("count")
    plt.title("Sign distribution of torque_x")
    plt.tight_layout()
    plt.savefig(output_path, dpi=dpi)
    plt.close()


def plot_loglog_facet_by_tilt(
    df: pd.DataFrame,
    distance_col: str,
    tilt_col: str,
    group_results: dict,
    output_path: Path,
    dpi: int,
) -> None:
    """tilt_col ごとにファセットし、各グループで推定した n に基づくフィット直線を描く。"""
    if tilt_col not in df.columns:
        raise ValueError(f"tilt 列 {tilt_col} が DataFrame に存在しません。")

    # 有効な tilt 値のみを対象にする
    tilt_vals = df[tilt_col].dropna().unique().tolist()
    if not tilt_vals:
        raise ValueError(f"{tilt_col} に有効な値がありません。")

    # Facet の列順を固定（昇順ソート）
    try:
        tilt_order = sorted(tilt_vals)
    except TypeError:
        # 数値と文字列が混ざるなどでソートできない場合は、そのまま
        tilt_order = tilt_vals

    sns.set(context="talk", style="whitegrid")

    g = sns.relplot(
        data=df,
        x="log_force",
        y="log_abs_torque",
        hue=distance_col,
        col=tilt_col,
        col_order=tilt_order,
        kind="scatter",
        palette="viridis",
        height=4.0,
        aspect=1.1,
    )

    # 各ファセットに、そのチルト角グループで推定したフィット直線を重ね描き
    axes = g.axes.flat if hasattr(g, "axes") else [g.ax]  # type: ignore[attr-defined]
    for idx, tilt_value in enumerate(tilt_order):
        if tilt_value not in group_results:
            continue
        if idx >= len(axes):
            break

        res = group_results[tilt_value]
        beta = res["beta"]
        n = res["n"]
        degree = len(beta) - 1 - 1  # [1, d,...,d^deg, logF] -> deg = len(beta)-2

        df_g = df[df[tilt_col] == tilt_value]
        if df_g.empty:
            continue

        d0 = float(np.median(df_g[distance_col].to_numpy(dtype=float)))
        const_terms = [beta[0]]
        for p in range(1, degree + 1):
            const_terms.append(beta[p] * (d0 ** p))
        c0 = float(np.sum(const_terms))

        x_line = np.linspace(df_g["log_force"].min(), df_g["log_force"].max(), 100)
        y_line = c0 + n * x_line

        ax = axes[idx]
        ax.plot(
            x_line,
            y_line,
            color="red",
            linestyle="--",
            linewidth=2,
            label=f"fit at {distance_col}={d0:.3g}",
        )
        ax.legend()

    g.set_axis_labels("log(force_z)", "log(|torque_x|)")
    g.fig.suptitle(
        f"log(|torque_x|) vs log(force_z) faceted by {tilt_col}", y=0.95
    )
    g.fig.subplots_adjust(left=0.10, right=0.95, top=0.88, bottom=0.12)
    g.fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(g.fig)


def main() -> None:
    args = parse_args()
    sns.set(context="talk", style=args.style)

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"入力CSVが見つかりません: {csv_path}", file=sys.stderr)
        sys.exit(1)

    if args.distance_degree < 0:
        print("distance-degree は 0 以上の整数で指定してください。", file=sys.stderr)
        sys.exit(1)

    output_dir = Path(args.output_dir)
    ensure_output_dir(output_dir)

    try:
        df_raw = pd.read_csv(csv_path)
        df = load_and_prepare_data(
            csv_path=csv_path,
            torque_col=args.torque_col,
            distance_col=args.distance_col,
            force_col=args.force_col,
            min_force=args.min_force,
            min_abs_torque=args.min_abs_torque,
            min_distance=args.min_distance,
            max_distance=args.max_distance,
        )

        print(f"有効サンプル数: {len(df)}")

        result = fit_log_model(
            distance=df[args.distance_col].to_numpy(dtype=float),
            log_force=df["log_force"].to_numpy(dtype=float),
            log_abs_torque=df["log_abs_torque"].to_numpy(dtype=float),
            degree=args.distance_degree,
        )

        n = result["n"]
        n_se = result["n_se"]
        r2 = result["r2"]

        if np.isnan(n_se):
            print(f"推定された n: {n:.6g}")
        else:
            print(f"推定された n: {n:.6g} ± {n_se:.3g}")
        print(f"決定係数 R^2: {r2:.4f}")

        # チルト角ごとにグルーピングして n を推定
        group_results: dict = {}
        if args.tilt_col is not None:
            if args.tilt_col not in df.columns:
                print(
                    f"警告: tilt 列 {args.tilt_col} が見つからないため、"
                    "チルト角ごとの解析はスキップします。",
                    file=sys.stderr,
                )
            else:
                for tilt_value, df_g in df.groupby(args.tilt_col):
                    df_g = df_g.copy()
                    # サンプル数チェックは fit_log_model 内でも行うが、
                    # ここでも極端に小さい場合はスキップしておく。
                    if len(df_g) < args.distance_degree + 2:
                        print(
                            f"警告: {args.tilt_col}={tilt_value} のグループは"
                            "サンプル数が少ないためスキップします。",
                            file=sys.stderr,
                        )
                        continue

                    try:
                        res_g = fit_log_model(
                            distance=df_g[args.distance_col].to_numpy(dtype=float),
                            log_force=df_g["log_force"].to_numpy(dtype=float),
                            log_abs_torque=df_g["log_abs_torque"].to_numpy(
                                dtype=float
                            ),
                            degree=args.distance_degree,
                        )
                    except Exception as e:
                        print(
                            f"警告: {args.tilt_col}={tilt_value} のグループで"
                            f"フィットに失敗しました: {e}",
                            file=sys.stderr,
                        )
                        continue

                    group_results[tilt_value] = res_g

                    n_g = res_g["n"]
                    n_se_g = res_g["n_se"]
                    r2_g = res_g["r2"]
                    n_samples = len(df_g)
                    if np.isnan(n_se_g):
                        print(
                            f"{args.tilt_col}={tilt_value}: "
                            f"n = {n_g:.6g}, R^2 = {r2_g:.4f} (N={n_samples})"
                        )
                    else:
                        print(
                            f"{args.tilt_col}={tilt_value}: "
                            f"n = {n_g:.6g} ± {n_se_g:.3g}, "
                            f"R^2 = {r2_g:.4f} (N={n_samples})"
                        )

        # プロット生成
        plot_loglog_with_fit(
            df=df,
            distance_col=args.distance_col,
            result=result,
            output_path=output_dir / "loglog_fit.png",
            dpi=args.dpi,
        )

        plot_normalized_torque_vs_distance(
            df=df,
            distance_col=args.distance_col,
            force_col=args.force_col,
            n=n,
            output_path=output_dir / "normalized_torque_vs_distance.png",
            dpi=args.dpi,
        )

        plot_residuals(
            df=df,
            distance_col=args.distance_col,
            force_col=args.force_col,
            residuals=result["residuals"],
            output_dir=output_dir,
            dpi=args.dpi,
        )

        # チルト角ごとのファセット loglog プロット
        if args.tilt_col is not None and group_results:
            try:
                facet_output = (
                    output_dir
                    / f"loglog_fit_facet_by_{args.tilt_col}.png"
                )
                plot_loglog_facet_by_tilt(
                    df=df,
                    distance_col=args.distance_col,
                    tilt_col=args.tilt_col,
                    group_results=group_results,
                    output_path=facet_output,
                    dpi=args.dpi,
                )
            except Exception as e:
                print(
                    f"警告: チルト角ファセットプロットの作成に失敗しました: {e}",
                    file=sys.stderr,
                )

        plot_torque_sign_hist(
            df_raw=df_raw,
            torque_col=args.torque_col,
            output_path=output_dir / "torque_sign_hist.png",
            dpi=args.dpi,
        )

        print(f"プロットを {output_dir} に保存しました。")

    except Exception as e:
        print(f"エラー: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()


