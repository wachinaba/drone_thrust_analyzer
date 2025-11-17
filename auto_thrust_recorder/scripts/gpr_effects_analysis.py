#!/usr/bin/env python3
import argparse
import os
from typing import Dict, Tuple

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from joblib import load

from auto_thrust_recorder.analysis.metrics import compute_metric_values
from auto_thrust_recorder.analysis.integration import build_grid, integrate_values
from auto_thrust_recorder.analysis.plotting import plot_1d, plot_2d, dump_csv


def parse_span(text: str) -> Tuple[float, float, int]:
    # format: min,max[:points]
    # examples: '0.5,9.0', '0.5,9.0:300'
    if ':' in text:
        rng, pts_str = text.split(':', 1)
        pts = int(pts_str) if pts_str.strip() else 0
    else:
        rng = text
        pts = 0
    if ',' not in rng:
        raise ValueError(f"span '{text}' must be 'min,max[:N]'")
    lo_str, hi_str = rng.split(',', 1)
    lo = float(lo_str); hi = float(hi_str)
    return lo, hi, pts


def main():
    p = argparse.ArgumentParser(description='GPR effects analysis (integrated metrics).')
    p.add_argument('csv_file', help='参照CSV（範囲推定に使用）')
    p.add_argument('--load-model', required=True, help='joblibで保存したモデルバンドル')
    p.add_argument('--metric', choices=['moment_abs', 'grad_abs'], default='moment_abs')
    p.add_argument('--grad-dims', type=str, default=None, help='grad_absで偏微分する軸（カンマ区切り）')
    p.add_argument('--grad-norm', choices=['l1', 'l2'], default='l2')
    p.add_argument('--fd-step', action='append', default=None, help="有限差分ステップ 'col:h' を複数指定可")
    p.add_argument('--integrate-over', action='append', required=True, help="積分軸と範囲 'col:min,max[:N]' を複数指定可")
    p.add_argument('--fix', action='append', default=None, help="固定値 'col=value' 複数可")
    p.add_argument('--viz-range', action='append', default=None, help="可視化軸の範囲 'col:min,max[:N]' 複数可")
    p.add_argument('--viz-points', type=int, default=100, help='可視化軸デフォルト分解能')
    p.add_argument('--output-eval', type=str, default=None)
    p.add_argument('--output-csv', type=str, default=None)
    p.add_argument('--cumulative-over', type=str, default=None, help="累積積分の軸と分割数 'col:splits'（同軸の [min, partial] を順次評価）")
    p.add_argument('--verbose', action='store_true', help='詳細ログを出力')

    args = p.parse_args()

    def vlog(*a, **k):
        if args.verbose:
            print(*a, **k)

    # backend
    if (not os.environ.get('DISPLAY')) or args.output_eval:
        try:
            matplotlib.use('Agg', force=True)
        except Exception:
            pass

    # load model bundle
    vlog('[load] model bundle:', args.load_model)
    bundle = load(args.load_model)
    if not isinstance(bundle, dict) or 'model' not in bundle:
        raise RuntimeError('ロードしたファイルはモデルバンドルではありません。')
    model = bundle['model']
    scaler = bundle.get('scaler', None)
    feature_names = bundle.get('feature_columns', None)
    if not feature_names:
        raise RuntimeError('feature_columns がモデルに含まれていません。')

    # read CSV for range reference
    vlog('[csv] read:', args.csv_file)
    df = pd.read_csv(args.csv_file)
    vlog('[csv] rows:', len(df))

    # parse fixes
    fixes: Dict[str, float] = {}
    if args.fix:
        for item in args.fix:
            if '=' not in item:
                continue
            c, v = item.split('=', 1)
            fixes[c.strip()] = float(v)

    # parse integrate spec
    integrate_spec = {}
    vlog('[integrate] specs:', args.integrate_over)
    for it in (args.integrate_over or []):
        if ':' not in it:
            continue
        col, span = it.split(':', 1)
        if ',' not in span:
            continue
        lo, hi, pts = parse_span(span)
        if pts <= 0:
            pts = 200
        integrate_spec[col.strip()] = (lo, hi, pts)

    # parse visualize spec (remaining non-fixed, non-integrated axes)
    viz_spec = {}
    if args.viz_range:
        vlog('[viz] ranges:', args.viz_range)
        for it in args.viz_range:
            if ':' not in it:
                continue
            col, span = it.split(':', 1)
            if ',' not in span:
                continue
            lo, hi, pts = parse_span(span)
            if pts <= 0:
                pts = args.viz_points
            viz_spec[col.strip()] = (lo, hi, pts)
    else:
        # default: choose one remaining axis if any, spanning observed range
        remaining = [c for c in feature_names if c not in fixes and c not in integrate_spec]
        if remaining:
            c = remaining[0]
            lo = float(df[c].min()); hi = float(df[c].max())
            viz_spec[c] = (lo, hi, args.viz_points)
            vlog('[viz] default axis:', c, 'range=', (lo, hi), 'points=', args.viz_points)

    def align_Z_to_axes(Z: np.ndarray, axes_names, viz_axes) -> np.ndarray:
        """
        integrate_values returns Z with axis order following feature_names filtered
        by presence in viz_axes. Here we transpose to match the plotting order axes_names.
        """
        vis_order = [c for c in feature_names if c in viz_axes]
        if list(axes_names) == vis_order:
            return Z
        perm = [vis_order.index(c) for c in axes_names]
        if Z.ndim != len(perm):
            return Z
        return np.transpose(Z, axes=perm)

    # finite difference steps (in data space)
    fd_steps = {}
    if args.fd_step:
        for it in args.fd_step:
            if ':' not in it:
                continue
            c, h = it.split(':', 1)
            fd_steps[c.strip()] = float(h)

    def compute_and_integrate(integrate_spec_local):
        vlog('[grid] build with integrate_spec:', integrate_spec_local)
        X, viz_axes, int_axes = build_grid(feature_names, integrate_spec_local, fixes, viz_spec)
        shape = (len(X), len(feature_names))
        vlog('[grid] X shape:', shape)
        vlog('[grid] visualize axes:', {k: len(v) for k, v in viz_axes.items()})
        vlog('[grid] integrate axes:', {k: len(v) for k, v in int_axes.items()})
        X_scaled = scaler.transform(X) if scaler is not None else X
        vlog('[scale] applied:', scaler is not None)
        grad_dims = [c.strip() for c in args.grad_dims.split(',')] if args.grad_dims else None
        vlog('[metric] type:', args.metric, 'grad_dims=', grad_dims, 'grad_norm=', args.grad_norm)
        values = compute_metric_values(
            model=model,
            X=X_scaled,
            metric=args.metric,
            feature_names=feature_names,
            grad_dims=grad_dims,
            grad_norm=args.grad_norm,
            fd_steps=fd_steps,
        )
        vlog('[metric] values: min=', float(np.min(values)), 'max=', float(np.max(values)), 'mean=', float(np.mean(values)))
        Z, _ = integrate_values(values, feature_names, viz_axes, int_axes)
        vlog('[integrate] result shape:', Z.shape)
        return Z, viz_axes, int_axes

    # cumulative option
    cumulative = None
    if args.cumulative_over:
        if ':' not in args.cumulative_over:
            raise ValueError("--cumulative-over は 'col:splits' 形式で指定してください")
        cum_col, cum_splits_s = args.cumulative_over.split(':', 1)
        cum_col = cum_col.strip(); cum_splits = int(cum_splits_s)
        if cum_col not in integrate_spec:
            raise ValueError(f"--cumulative-over の軸 '{cum_col}' は --integrate-over に含まれていません")
        if cum_splits <= 0:
            raise ValueError("--cumulative-over の分割数は正の整数で指定してください")
        cumulative = (cum_col, cum_splits)

    if cumulative is None:
        Z, viz_axes, int_axes = compute_and_integrate(integrate_spec)
        axes_names = list(viz_axes.keys())
        axes_vals = [viz_axes[k] for k in axes_names]
        integrate_desc_parts = [f"{col} [{arr[0]:.3g}, {arr[-1]:.3g}], N={len(arr)}" for col, arr in int_axes.items()]
        integrate_desc = "; ".join(integrate_desc_parts) if integrate_desc_parts else "(none)"
        title = f"metric={args.metric} | integrate over: {integrate_desc}"
        if len(axes_names) == 0:
            print(f"Integrate over: {integrate_desc}")
            print(f"integral value: {float(Z):.6f}")
            if args.output_csv:
                dump_csv([], [], np.array([Z]), args.output_csv)
        elif len(axes_names) == 1:
            plot_1d(axes_names[0], axes_vals[0], Z.reshape(-1), title, args.output_eval)
            if args.output_csv:
                dump_csv(axes_names, axes_vals, Z, args.output_csv)
        elif len(axes_names) == 2:
            Xv, Yv = axes_vals[0], axes_vals[1]
            Zm = align_Z_to_axes(Z, axes_names, viz_axes)
            plot_2d(axes_names[0], axes_names[1], Xv, Yv, Zm, title, args.output_eval)
            if args.output_csv:
                dump_csv(axes_names, axes_vals, Zm, args.output_csv)
        else:
            print(f"可視化軸が3以上のため図は出力しません。CSVで出力します。axes={axes_names}")
            if args.output_csv:
                dump_csv(axes_names, axes_vals, Z, args.output_csv)
    else:
        cum_col, splits = cumulative
        lo, hi, pts = integrate_spec[cum_col]
        if pts <= 0:
            pts = 200
        uppers = [lo + (hi - lo) * (k / splits) for k in range(1, splits + 1)]
        vlog('[cumulative] axis:', cum_col, 'range=', (lo, hi), 'splits=', splits, 'uppers=', uppers)

        results = []
        for k, up in enumerate(uppers, start=1):
            spec_k = dict(integrate_spec)
            pts_k = max(2, int(np.ceil(pts * (k / splits))))
            spec_k[cum_col] = (lo, up, pts_k)
            vlog('[cumulative] step', k, 'upper=', up, 'points=', pts_k)
            Zk, viz_axes_k, int_axes_k = compute_and_integrate(spec_k)
            results.append((up, Zk, viz_axes_k, int_axes_k))

        axes_names = list(results[0][2].keys())
        axes_vals = [results[0][2][k] for k in axes_names]
        integrate_desc = f"{cum_col} cumulative [{lo:.3g} -> upper], splits={splits}"

        if len(axes_names) == 0:
            print(f"Cumulative integrate over: {integrate_desc}")
            for up, Zk, _, _ in results:
                print(f"  upper={up:.6g}: value={float(Zk):.6f}")
            if args.output_csv:
                df_out = pd.DataFrame({'upper': [up for up, _, _, _ in results], 'value': [float(Zk) for _, Zk, _, _ in results]})
                df_out.to_csv(args.output_csv, index=False)
        elif len(axes_names) == 1:
            plt.figure(figsize=(10, 6))
            plt.rcParams.update({'font.size': 16})
            n = len(results)
            for i, (up, Zk, _, _) in enumerate(results):
                # magma で統一（序盤ほど明るく、終端は最も濃い）
                t = 1.0 if i == n - 1 else (i / max(1, n - 1))
                color = plt.cm.magma(1.0 - t)  # 反転
                lw = 3.5 if i == n - 1 else (1.5 + 1.0 * t)
                alpha = 1.0 if i == n - 1 else 0.6
                z = 5 if i == n - 1 else (1 + i)
                plt.plot(
                    axes_vals[0],
                    Zk.reshape(-1),
                    color=color,
                    lw=lw,
                    alpha=alpha,
                    zorder=z,
                    label=f"{cum_col} upper={up:.3g}"
                )
            plt.xlabel(axes_names[0])
            plt.ylabel('integral')
            plt.title(f"metric={args.metric} | {integrate_desc}")
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.tight_layout()
            if args.output_eval:
                plt.savefig(args.output_eval, dpi=300, bbox_inches='tight')
            else:
                plt.show()
            plt.close()
            if args.output_csv:
                df_out = pd.DataFrame({axes_names[0]: axes_vals[0]})
                for up, Zk, _, _ in results:
                    df_out[f"upper_{up:.6g}"] = Zk.reshape(-1)
                df_out.to_csv(args.output_csv, index=False)
        elif len(axes_names) == 2:
            base = args.output_eval or 'effects_cumulative.png'
            stem, ext = os.path.splitext(base)
            for (up, Zk, _, _) in results:
                Xv, Yv = axes_vals[0], axes_vals[1]
                Zm = align_Z_to_axes(Zk, axes_names, results[0][2])
                out_path = f"{stem}_upper_{up:.6g}{ext}"
                plot_2d(axes_names[0], axes_names[1], Xv, Yv, Zm, f"metric={args.metric} | {cum_col} upper={up:.3g}", out_path)
                if args.output_csv:
                    csv_path = f"{stem}_upper_{up:.6g}.csv"
                    dump_csv(axes_names, axes_vals, Zm, csv_path)
        else:
            print(f"可視化軸が3以上のため図は出力しません（累積モード）。CSVで出力します。axes={axes_names}")
            if args.output_csv:
                stem, _ = os.path.splitext(args.output_csv)
                for up, Zk, _, _ in results:
                    path = f"{stem}_upper_{up:.6g}.csv"
                    dump_csv(axes_names, axes_vals, Zk, path)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())


