#!/usr/bin/env python3
"""
gpytorch(SVGP)で学習済みの回帰モデル（gaussian_process_regression.py の --backend gpytorch --save-model 出力）
を読み込み、CSVの生データ(raw)散布とフィッティング曲線を描画する。

特徴:
- 単一プロットがデフォルト（--group-by 指定でグループ別サブプロットも可能）
- raw散布は --hue-raw で「学習特徴量に含まれない列」でも色分け可能
- fit曲線は --hue-fit が「学習特徴量に含まれる列」の場合のみ分割描画
  （含まれない場合は警告して無視: ユーザー指定 A）
- visualize_morph_drone.py の 3Dドローン図を背景/インセットに重畳可能

使用例:
  python plot_gpytorch_fit_curve_with_raw.py data.csv --load-model model.pt --curve-x distance --output out.png
  python plot_gpytorch_fit_curve_with_raw.py data.csv --load-model model.pt --curve-x distance --hue-raw wall_spacing --output out.png
  python plot_gpytorch_fit_curve_with_raw.py data.csv --load-model model.pt --curve-x distance --hue-fit fold_angle --output out.png
"""

from __future__ import annotations

import argparse
import os
import warnings
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.colors as mcolors
import matplotlib.cm as cm

warnings.filterwarnings("ignore")


def _import_gpr_utils():
    """
    gaussian_process_regression.py から必要なユーティリティを import する。
    scripts/ 配下での単体実行を想定し、相対 import は使わない。
    """
    try:
        import gaussian_process_regression as gpr  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "gaussian_process_regression.py を import できません。"
            "このスクリプトは auto_thrust_recorder/scripts 配下で実行してください。"
            f" reason={e}"
        ) from e

    required = [
        "parse_tf",
        "load_model_any",
        "build_gpytorch_wrapper_from_bundle",
        "load_and_preprocess_data",
        "parse_fixed_values",
        "parse_fix_overrides",
        "apply_facet_filters",
        "_get_facet_drone_rgba_cached",
        "GPyTorchSparseGPR",
    ]
    for name in required:
        if not hasattr(gpr, name):
            raise RuntimeError(f"gaussian_process_regression.py に必要な関数/クラスが見つかりません: {name}")
    return gpr


def _is_numeric_series(s: pd.Series) -> bool:
    try:
        ss = pd.to_numeric(s.dropna(), errors="coerce")
        if len(ss) == 0:
            return False
        return bool(np.isfinite(ss).mean() > 0.8)
    except Exception:
        return False


def _stable_sorted_unique(values: List[Any]) -> List[Any]:
    # 数値っぽいものは数値でソート、だめなら文字列でソート、最後は元順
    if not values:
        return []
    try:
        vals_num = [float(v) for v in values]
        order = np.argsort(vals_num).tolist()
        return [values[i] for i in order]
    except Exception:
        pass
    try:
        return sorted(values, key=lambda v: str(v))
    except Exception:
        return values


def _get_drone_angles_from_df(
    df: pd.DataFrame,
    *,
    phi_override: Optional[float] = None,
    psi_override: Optional[float] = None,
    theta_override: Optional[float] = None,
) -> Tuple[float, float, float]:
    """
    df の列（fold/slant/tilt）中央値からドローン角を推定する。
    override があれば優先。
    """
    phi = phi_override
    psi = psi_override
    th = theta_override

    if phi is None and "fold_angle" in df.columns:
        try:
            phi = float(pd.to_numeric(df["fold_angle"], errors="coerce").median())
        except Exception:
            phi = None
    if psi is None and "slant_angle" in df.columns:
        try:
            psi = float(pd.to_numeric(df["slant_angle"], errors="coerce").median())
        except Exception:
            psi = None
    if th is None and "tilt_angle" in df.columns:
        try:
            th = float(pd.to_numeric(df["tilt_angle"], errors="coerce").median())
        except Exception:
            th = None

    # 最終フォールバック
    if phi is None:
        phi = 0.0
    if psi is None:
        psi = 0.0
    if th is None:
        th = 0.0
    return float(phi), float(psi), float(th)

def _parse_float_pair_opt(s: Optional[str], *, name: str) -> Optional[Tuple[float, float]]:
    """
    'a,b' 形式を (a,b) に変換。None/空は None。
    """
    if s is None:
        return None
    t = str(s).strip()
    if not t:
        return None
    if "," not in t:
        raise ValueError(f"{name} は 'a,b' 形式で指定してください（入力='{s}'）")
    a, b = t.split(",", 1)
    return float(a), float(b)

def _parse_fix_for_csv_filter(
    fixes: Optional[List[str]],
    df_columns: List[str],
) -> Dict[str, float]:
    """
    `--fix col=value` をパースし、CSVに存在する列であればフィルタ用辞書に含める。
    学習特徴量かどうかは問わない。CSVにも存在しない列は警告を出す。
    """
    overrides: Dict[str, float] = {}
    if not fixes:
        return overrides
    col_set = set(df_columns)
    for item in fixes:
        if not item or ("=" not in item):
            continue
        col, val = item.split("=", 1)
        col = col.strip()
        if col not in col_set:
            print(f"[warn] --fix '{col}' はCSVに存在しないため無視します。")
            continue
        try:
            overrides[col] = float(val)
        except Exception:
            print(f"[warn] --fix '{col}={val}' の値が数値に変換できないため無視します。")
            continue
    return overrides


def _apply_fix_filters_to_df(
    df: pd.DataFrame,
    *,
    fix_overrides: Dict[str, float],
    exclude_cols: Optional[List[str]] = None,
    atol: float = 1e-9,
    rtol: float = 0.0,
) -> pd.DataFrame:
    """
    `--fix col=value` の指定を rawデータ側のフィルタとして適用する。
    - 学習特徴量に含まれる列であっても、CSVに列があればフィルタ対象にする
    - exclude_cols に含まれる列はフィルタしない
    - 数値列のみ想定（fix_overrides が float なので）
    """
    if df is None or df.empty:
        return df
    if not fix_overrides:
        return df
    exc = set(exclude_cols or [])
    out = df
    for col, val in fix_overrides.items():
        if col in exc:
            continue
        if col not in out.columns:
            continue
        s = pd.to_numeric(out[col], errors="coerce")
        m = np.isfinite(s.values) & np.isclose(s.values, float(val), atol=float(atol), rtol=float(rtol))
        before = len(out)
        out = out.loc[m].copy()
        after = len(out)
        print(f"[info] raw filter by --fix: {col}=={val:g}  kept={after}/{before}")
        if out.empty:
            break
    return out


def _overlay_drone_image(
    ax: plt.Axes,
    gpr_mod,
    *,
    df_for_angles: pd.DataFrame,
    mode: str,
    alpha: float,
    inset_loc: str,
    inset_size: float,
    inset_alpha: float,
    dpi: int,
    phi_override: Optional[float] = None,
    psi_override: Optional[float] = None,
    theta_override: Optional[float] = None,
) -> None:
    """
    visualize_morph_drone のレンダリング画像を ax に重畳する。
    mode: background|inset|both
    """
    mode = str(mode or "background").strip().lower()
    if mode not in ["background", "inset", "both"]:
        raise ValueError(f"--drone-mode が不正です: {mode}")

    phi, psi, th = _get_drone_angles_from_df(
        df_for_angles, phi_override=phi_override, psi_override=psi_override, theta_override=theta_override
    )

    # 3D only / 2Dフォールバック禁止のため force_2d=False 固定
    img = gpr_mod._get_facet_drone_rgba_cached(phi_deg=phi, psi_deg=psi, theta_deg=th, force_2d=False, dpi=int(dpi))

    # 描画範囲は現在の軸レンジに合わせる（after plot）
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()

    if mode in ["background", "both"]:
        ax.imshow(
            img,
            extent=[x0, x1, y0, y1],
            aspect="auto",
            interpolation="bilinear",
            alpha=float(alpha),
            zorder=0,
        )

    if mode in ["inset", "both"]:
        loc = str(inset_loc or "ur").strip().lower()
        s = max(0.05, min(0.95, float(inset_size)))
        pad = 0.02
        if loc == "ul":
            ix, iy = pad, 1.0 - pad - s
        elif loc == "lr":
            ix, iy = 1.0 - pad - s, pad
        elif loc == "ll":
            ix, iy = pad, pad
        else:  # ur
            ix, iy = 1.0 - pad - s, 1.0 - pad - s

        try:
            axins = ax.inset_axes([ix, iy, s, s], transform=ax.transAxes, zorder=8)
        except Exception:
            axins = ax.inset_axes([ix, iy, s, s])
        axins.imshow(img, interpolation="bilinear", alpha=float(inset_alpha))
        axins.set_axis_off()


def plot_single_raw_and_fit(
    *,
    df: pd.DataFrame,
    feature_columns: List[str],
    target_column: str,
    model: Any,
    scaler: Any,
    curve_x: str,
    curve_points: int,
    fixes: Optional[List[str]],
    hue_raw: Optional[str],
    hue_fit: Optional[str],
    hue_raw_cmap: str,
    hue_raw_range: Optional[Tuple[float, float]],
    hue_fit_cmap: str,
    hue_fit_range: Optional[Tuple[float, float]],
    x_domain_per_hue_raw: bool,
    show_uncertainty: bool,
    ylim: Optional[Tuple[float, float]],
    raw_alpha: float,
    fit_extrema: bool,
    fit_extrema_vline: bool,
    fit_extrema_marker: bool,
    fit_extrema_reverse_colors: bool,
    xlabel: Optional[str],
    ylabel: Optional[str],
    colorbar_label: Optional[str],
    figsize: Tuple[float, float],
    title: Optional[str],
    drone: bool,
    drone_mode: str,
    drone_alpha: float,
    drone_inset_loc: str,
    drone_inset_size: float,
    drone_inset_alpha: float,
    drone_dpi: int,
    drone_phi: Optional[float],
    drone_psi: Optional[float],
    drone_theta: Optional[float],
    output: Optional[str],
    gpr_mod,
) -> None:
    """
    単一プロット: raw散布 + fit曲線。
    hue_raw: raw散布の色分け（学習特徴量でなくても可）
    hue_fit: 学習特徴量に含まれる場合のみ、fit曲線を分割（含まれない場合は警告して無視）
    fit_extrema: fit曲線の最大/最小を（全曲線での全体として）表示する
    """
    if curve_x not in feature_columns:
        raise ValueError(f"--curve-x '{curve_x}' は学習特徴量に含まれていません: {feature_columns}")
    if target_column not in df.columns:
        raise ValueError(f"target列 '{target_column}' がCSVに存在しません。")

    # xレンジ
    x_series = pd.to_numeric(df[curve_x], errors="coerce")
    x_min = float(np.nanmin(x_series.values))
    x_max = float(np.nanmax(x_series.values))
    if not np.isfinite(x_min) or not np.isfinite(x_max):
        raise ValueError(f"curve_x '{curve_x}' の範囲が取得できません。")
    if x_min == x_max:
        x_min -= 1e-6
        x_max += 1e-6
    x_grid = np.linspace(x_min, x_max, int(curve_points))

    # rawデータを --fix でフィルタ（CSVに存在する列全てが対象）
    # ただし curve_x は掃引軸なので除外する
    csv_fix_overrides = _parse_fix_for_csv_filter(fixes, list(df.columns))
    if csv_fix_overrides:
        df = _apply_fix_filters_to_df(
            df,
            fix_overrides=csv_fix_overrides,
            exclude_cols=[curve_x],
            atol=float(getattr(plot_single_raw_and_fit, "_fix_filter_atol", 1e-9)),
            rtol=float(getattr(plot_single_raw_and_fit, "_fix_filter_rtol", 0.0)),
        )
        if df is None or df.empty:
            raise RuntimeError("rawデータが --fix フィルタで空になりました。--fix の指定を見直してください。")

    # 固定値はフィルタ後のデータの中央値を使用
    fixed_values = {c: float(pd.to_numeric(df[c], errors="coerce").median()) for c in feature_columns}

    # 学習特徴量に対する --fix 指定で上書き
    fix_overrides = gpr_mod.parse_fix_overrides(fixes, feature_columns)
    for k, v in fix_overrides.items():
        if k in fixed_values and k != curve_x:
            fixed_values[k] = float(v)

    # fit曲線に使う hue（学習特徴量に含まれる場合のみ）
    hue_fit_col = None
    if hue_fit:
        if hue_fit in feature_columns:
            hue_fit_col = hue_fit
        else:
            print(f"[warn] --hue-fit '{hue_fit}' は学習特徴量に無いため無視します（rawのみ色分けは可能です）。")

    # raw散布の hue
    hue_raw_col = str(hue_raw).strip() if hue_raw else None
    if hue_raw_col:
        if hue_raw_col not in df.columns:
            print(f"[warn] --hue-raw '{hue_raw_col}' がCSVに無いため無視します。")
            hue_raw_col = None

    plt.rcParams.update({"font.size": 16})
    fig, ax = plt.subplots(1, 1, figsize=figsize)

    # --- fit extrema tracking (global across all drawn fit curves) ---
    extrema = {
        "max": {"y": None, "x": None, "hue": None},
        "min": {"y": None, "x": None, "hue": None},
    }

    def _update_extrema(y_arr: np.ndarray, *, x_arr: np.ndarray, hue_value: Optional[float]) -> None:
        if y_arr is None:
            return
        yy = np.asarray(y_arr).reshape(-1)
        xx = np.asarray(x_arr).reshape(-1)
        if yy.size == 0:
            return
        if xx.size != yy.size:
            return
        m = np.isfinite(yy)
        if not np.any(m):
            return
        yy2 = yy.copy()
        yy2[~m] = np.nan
        try:
            imax = int(np.nanargmax(yy2))
            imin = int(np.nanargmin(yy2))
        except Exception:
            return
        y_max = float(yy2[imax])
        y_min = float(yy2[imin])
        try:
            x_max = float(xx[imax])
            x_min = float(xx[imin])
        except Exception:
            return

        if (extrema["max"]["y"] is None) or (y_max > float(extrema["max"]["y"])):
            extrema["max"]["y"] = y_max
            extrema["max"]["x"] = x_max
            extrema["max"]["hue"] = hue_value
        if (extrema["min"]["y"] is None) or (y_min < float(extrema["min"]["y"])):
            extrema["min"]["y"] = y_min
            extrema["min"]["x"] = x_min
            extrema["min"]["hue"] = hue_value

    # raw散布
    x_raw = pd.to_numeric(df[curve_x], errors="coerce").values
    y_raw = pd.to_numeric(df[target_column], errors="coerce").values
    mxy = np.isfinite(x_raw) & np.isfinite(y_raw)

    # hue-raw の「色決定」と「グループ値」を fit 側でも使えるように保持する
    hue_raw_is_num = False
    hue_raw_num = None  # numeric series (float) if hue_raw_is_num
    hue_raw_color_of = None  # callable: v(float) -> rgba
    hue_raw_group_values_num: List[float] = []

    raw_handles = []
    if hue_raw_col:
        s = df[hue_raw_col]
        if _is_numeric_series(s):
            hue_raw_is_num = True
            hue_raw_num = pd.to_numeric(s, errors="coerce")
            cvals = pd.to_numeric(s, errors="coerce").values
            cmap_name = str(hue_raw_cmap or "viridis")
            cmap = plt.get_cmap(cmap_name)
            if hue_raw_range is not None:
                vmin, vmax = float(hue_raw_range[0]), float(hue_raw_range[1])
            else:
                vmin = float(np.nanmin(cvals))
                vmax = float(np.nanmax(cvals))
            if (not np.isfinite(vmin)) or (not np.isfinite(vmax)) or (vmin == vmax):
                vmin, vmax = 0.0, 1.0
            norm = mcolors.Normalize(vmin=vmin, vmax=vmax)
            hue_raw_color_of = lambda vv: cmap(norm(float(vv)))
            try:
                hue_raw_group_values_num = [float(v) for v in pd.unique(hue_raw_num.dropna())]
                hue_raw_group_values_num = [float(v) for v in _stable_sorted_unique(hue_raw_group_values_num)]
            except Exception:
                hue_raw_group_values_num = []
            ax.scatter(
                x_raw[mxy],
                y_raw[mxy],
                c=cvals[mxy],
                s=22,
                alpha=raw_alpha,
                cmap=cmap,
                norm=norm,
                zorder=3,
            )
            # カラーバーは ScalarMappable から作成（scatter の alpha の影響を受けないように）
            sm = cm.ScalarMappable(norm=norm, cmap=cmap)
            sm.set_array([])
            cb = fig.colorbar(sm, ax=ax, shrink=0.95)
            cb.set_label(colorbar_label if colorbar_label else hue_raw_col)
        else:
            hue_vals = list(pd.unique(s.dropna()))
            hue_vals = _stable_sorted_unique(hue_vals)
            cmap_name = str(hue_raw_cmap or "tab20")
            cmap = plt.get_cmap(cmap_name)
            # カテゴリ数に応じて等間隔サンプリング
            n = max(1, len(hue_vals))
            color_map = {v: cmap(0.5 if n == 1 else (i / (n - 1))) for i, v in enumerate(hue_vals)}
            if hue_raw_range is not None:
                print(f"[warn] --hue-raw-range はカテゴリhueでは無視されます: hue-raw='{hue_raw_col}'")
            for v in hue_vals:
                mv = (s.values == v) & mxy
                if not np.any(mv):
                    continue
                ax.scatter(x_raw[mv], y_raw[mv], s=22, alpha=raw_alpha, color=color_map[v], zorder=3)
                raw_handles.append(
                    mlines.Line2D([], [], color=color_map[v], marker="o", linestyle="None", markersize=6, label=f"{hue_raw_col}={v}")
                )
    else:
        ax.scatter(x_raw[mxy], y_raw[mxy], s=22, alpha=raw_alpha, color="C0", label="raw", zorder=3)

    # fit曲線（1本 or hue-fitごと）
    def _predict_for(
        hv: Optional[float] = None,
        fixed: Optional[Dict[str, float]] = None,
        *,
        x_arr: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        fixed_local = fixed if fixed is not None else fixed_values
        x_use = np.asarray(x_arr if x_arr is not None else x_grid).reshape(-1)
        X_grid = np.zeros((len(x_use), len(feature_columns)), dtype=float)
        for j, fcol in enumerate(feature_columns):
            if fcol == curve_x:
                X_grid[:, j] = x_use
            elif (hue_fit_col is not None) and (fcol == hue_fit_col) and (hv is not None):
                X_grid[:, j] = float(hv)
            else:
                X_grid[:, j] = float(fixed_local[fcol])

        if scaler is not None:
            try:
                X_infer = scaler.transform(X_grid)
            except Exception:
                X_infer = X_grid
        else:
            X_infer = X_grid

        if show_uncertainty:
            y_mean, y_std = model.predict(X_infer, return_std=True)
            return np.asarray(y_mean).reshape(-1), np.asarray(y_std).reshape(-1) if y_std is not None else None
        y_mean = model.predict(X_infer, return_std=False)
        return np.asarray(y_mean).reshape(-1), None

    fit_handles = []
    if hue_fit_col is not None:
        # --- Special mode (requested):
        # If both hue-raw and hue-fit are given AND hue-raw is numeric,
        # then override hue-fit value using hue-raw value (per hue-raw category).
        # Example: hue-raw=target_thrust (10/15/20/25), hue-fit=force_z -> force_z fixed to 10/15/20/25.
        if hue_raw_col and hue_raw_is_num and (hue_raw_num is not None) and (hue_raw_color_of is not None):
            if hue_fit_col == curve_x:
                raise ValueError(
                    f"--hue-fit '{hue_fit_col}' は --curve-x と同一です。"
                    "curve-x は掃引するため、hue-fit 固定（hue-raw上書き）モードは使えません。"
                )
            if (hue_fit_range is not None) or (str(hue_fit_cmap or "").strip() not in ["", "viridis"]):
                print("[warn] hue-raw→hue-fit 上書きモードでは --hue-fit-cmap/--hue-fit-range は無視されます（rawの色に揃えます）。")

            # 既に raw 側で colorbar が出ていれば十分だが、fit で使う列名も注釈したいので軽くログ
            print(f"[info] hue-raw '{hue_raw_col}' の値で hue-fit '{hue_fit_col}' を上書きして fit 曲線を描画します。")

            # hue-raw の各値ごとに、他特徴量をそのグループ中央値（+fix）にして予測
            for hv_raw in hue_raw_group_values_num:
                # グループ抽出（数値で一致）
                mask = (hue_raw_num.values == float(hv_raw))
                df_g = df.loc[mask]
                if df_g is None or len(df_g) == 0:
                    continue

                # グループごとに curve_x の定義域を分ける（任意）
                if bool(x_domain_per_hue_raw):
                    x_g_series = pd.to_numeric(df_g[curve_x], errors="coerce")
                    x_g_min = float(np.nanmin(x_g_series.values))
                    x_g_max = float(np.nanmax(x_g_series.values))
                    if (not np.isfinite(x_g_min)) or (not np.isfinite(x_g_max)):
                        continue
                    if x_g_min == x_g_max:
                        x_g_min -= 1e-6
                        x_g_max += 1e-6
                    x_grid_g = np.linspace(x_g_min, x_g_max, int(curve_points))
                else:
                    x_grid_g = x_grid

                # グループ中央値で固定値を作る（学習特徴量のみ）
                fixed_g = {}
                for c in feature_columns:
                    med = pd.to_numeric(df_g[c], errors="coerce").median()
                    if not np.isfinite(med):
                        med = pd.to_numeric(df[c], errors="coerce").median()
                    fixed_g[c] = float(med) if np.isfinite(med) else float(fixed_values[c])

                # fix overrides を適用（curve-x と hue-fit 自体は forced）
                forced_cols = {curve_x, hue_fit_col}
                for k, v in fix_overrides.items():
                    if (k in fixed_g) and (k not in forced_cols):
                        fixed_g[k] = float(v)

                color = hue_raw_color_of(float(hv_raw))
                y_mean, y_std = _predict_for(float(hv_raw), fixed=fixed_g, x_arr=x_grid_g)
                ax.plot(x_grid_g, y_mean, color=color, lw=2.6, alpha=0.95, zorder=4)
                _update_extrema(np.asarray(y_mean), x_arr=np.asarray(x_grid_g), hue_value=float(hv_raw))
                if show_uncertainty and (y_std is not None):
                    ax.fill_between(
                        x_grid_g,
                        y_mean - 2.0 * y_std,
                        y_mean + 2.0 * y_std,
                        color=color,
                        alpha=0.18,
                        zorder=2,
                    )
            # このモードでは fit_handles は追加しない（凡例肥大化防止）
        else:
            if bool(x_domain_per_hue_raw):
                print("[warn] --x-domain-per-hue-raw は hue-raw→hue-fit 上書きモードのときのみ有効です（現在の条件では無視します）。")
            # --- fallback hue-fit mode (original behavior) ---
            # hue-fit は学習特徴量として数値の想定。カテゴリっぽい場合も一応対応する。
            hv_raw_series = df[hue_fit_col]
            is_num = _is_numeric_series(hv_raw_series)
            cmap_name = str(hue_fit_cmap or "viridis")
            cmap = plt.get_cmap(cmap_name)

            if is_num:
                hv_series = pd.to_numeric(hv_raw_series, errors="coerce")
                hv_values = [v for v in pd.unique(hv_series.dropna())]
                hv_values = _stable_sorted_unique(hv_values)
                if len(hv_values) > 30:
                    print(f"[warn] --hue-fit '{hue_fit_col}' のユニーク値が {len(hv_values)} 個あります。指定通り全て描画します。")

                if hue_fit_range is not None:
                    vmin, vmax = float(hue_fit_range[0]), float(hue_fit_range[1])
                else:
                    vmin = float(np.nanmin(hv_series.values))
                    vmax = float(np.nanmax(hv_series.values))
                if (not np.isfinite(vmin)) or (not np.isfinite(vmax)) or (vmin == vmax):
                    vmin, vmax = 0.0, 1.0
                norm = mcolors.Normalize(vmin=vmin, vmax=vmax)

                for hv in hv_values:
                    try:
                        hv_f = float(hv)
                    except Exception:
                        continue
                    color = cmap(norm(hv_f))
                    y_mean, y_std = _predict_for(hv_f, fixed=None, x_arr=x_grid)
                    ax.plot(x_grid, y_mean, color=color, lw=2.6, alpha=0.95, zorder=4)
                    _update_extrema(np.asarray(y_mean), x_arr=np.asarray(x_grid), hue_value=float(hv_f))
                    if show_uncertainty and (y_std is not None):
                        ax.fill_between(
                            x_grid, y_mean - 2.0 * y_std, y_mean + 2.0 * y_std, color=color, alpha=0.18, zorder=2
                        )
                # colorbar for fit mapping
                sm = cm.ScalarMappable(norm=norm, cmap=cmap)
                sm.set_array([])
                cb = fig.colorbar(sm, ax=ax, shrink=0.95)
                cb.set_label(f"{hue_fit_col} (fit)")
            else:
                hv_values = list(pd.unique(hv_raw_series.dropna()))
                hv_values = _stable_sorted_unique(hv_values)
                n = max(1, len(hv_values))
                if hue_fit_range is not None:
                    print(f"[warn] --hue-fit-range はカテゴリhueでは無視されます: hue-fit='{hue_fit_col}'")
                for i, hv in enumerate(hv_values):
                    try:
                        hv_f = float(hv)
                    except Exception:
                        # カテゴリのまま学習特徴量に入っているケースは想定しないのでスキップ
                        continue
                    color = cmap(0.5 if n == 1 else (i / (n - 1)))
                    y_mean, y_std = _predict_for(hv_f, fixed=None, x_arr=x_grid)
                    ax.plot(x_grid, y_mean, color=color, lw=2.6, alpha=0.95, zorder=4)
                    _update_extrema(np.asarray(y_mean), x_arr=np.asarray(x_grid), hue_value=float(hv_f))
                    if show_uncertainty and (y_std is not None):
                        ax.fill_between(
                            x_grid, y_mean - 2.0 * y_std, y_mean + 2.0 * y_std, color=color, alpha=0.18, zorder=2
                        )
                    fit_handles.append(mlines.Line2D([], [], color=color, linestyle="-", lw=2.6, label=f"fit: {hue_fit_col}={hv}"))
            # end fallback hue-fit mode
    else:
        y_mean, y_std = _predict_for(None, fixed=None, x_arr=x_grid)
        ax.plot(x_grid, y_mean, color="red", lw=3.0, label="GPR fit", zorder=4)
        _update_extrema(np.asarray(y_mean), x_arr=np.asarray(x_grid), hue_value=None)
        if show_uncertainty and (y_std is not None):
            ax.fill_between(x_grid, y_mean - 2.0 * y_std, y_mean + 2.0 * y_std, color="red", alpha=0.20, label="±2σ", zorder=2)

    ax.set_xlabel(xlabel if xlabel else curve_x)
    ax.set_ylabel(ylabel if ylabel else target_column)
    ax.grid(True, alpha=0.3)
    if ylim is not None:
        ax.set_ylim(ylim[0], ylim[1])
    if title:
        ax.set_title(title)

    # fit extrema overlay (optional) - after ylim/title so placement uses final axes range
    if bool(fit_extrema) and (extrema["max"]["y"] is not None) and (extrema["min"]["y"] is not None):
        # Keep plot range stable
        ax.set_xlim(float(x_min), float(x_max))

        def _fmt_num(v: float) -> str:
            try:
                return f"{float(v):g}"
            except Exception:
                return str(v)

        def _label(which: str) -> str:
            yv = float(extrema[which]["y"])
            hv = extrema[which]["hue"]
            base = f"{which}={_fmt_num(yv)}"
            if (hue_fit_col is not None) and (hv is not None):
                return f"{base} @{hue_fit_col}={_fmt_num(float(hv))}"
            return base

        def _smart_annotate(*, x: float, y: float, text: str, color: str) -> None:
            # Decide annotation direction based on relative position within current axes limits
            x0, x1 = ax.get_xlim()
            y0, y1 = ax.get_ylim()
            xr = float(x1 - x0) if np.isfinite(x1 - x0) and (x1 != x0) else 1.0
            yr = float(y1 - y0) if np.isfinite(y1 - y0) and (y1 != y0) else 1.0
            fx = float((x - x0) / xr)
            fy = float((y - y0) / yr)

            # If point is on the right, place text to the left; if on the top, place text to the bottom.
            place_left = bool(fx > 0.75)
            place_down = bool(fy > 0.75)

            dx = -10 if place_left else 10
            dy = -12 if place_down else 12
            ha = "right" if place_left else "left"
            va = "top" if place_down else "bottom"

            bbox_kw = dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.75)
            ann = ax.annotate(
                text,
                xy=(x, y),
                xytext=(dx, dy),
                textcoords="offset points",
                ha=ha,
                va=va,
                fontsize=10,
                color=color,
                bbox=bbox_kw,
                zorder=7,
                annotation_clip=True,
            )
            try:
                ann.set_clip_on(True)
            except Exception:
                pass

        # Styling
        c_max = "tab:red"
        c_min = "tab:blue"
        if bool(fit_extrema_reverse_colors):
            c_max, c_min = c_min, c_max
        ls = (0, (4, 3))  # dashed
        lw = 1.6
        a = 0.75

        x_max_e = float(extrema["max"]["x"])
        y_max_e = float(extrema["max"]["y"])
        x_min_e = float(extrema["min"]["x"])
        y_min_e = float(extrema["min"]["y"])

        # Horizontal lines
        ax.axhline(y_max_e, color=c_max, linestyle=ls, lw=lw, alpha=a, zorder=3, label="_nolegend_")
        ax.axhline(y_min_e, color=c_min, linestyle=ls, lw=lw, alpha=a, zorder=3, label="_nolegend_")

        # Vertical lines (optional)
        if bool(fit_extrema_vline):
            ax.axvline(x_max_e, color=c_max, linestyle=ls, lw=lw, alpha=a, zorder=3, label="_nolegend_")
            ax.axvline(x_min_e, color=c_min, linestyle=ls, lw=lw, alpha=a, zorder=3, label="_nolegend_")

        # Markers (optional)
        if bool(fit_extrema_marker):
            ax.plot([x_max_e], [y_max_e], marker="o", color=c_max, markersize=6.5, zorder=6, label="_nolegend_")
            ax.plot([x_min_e], [y_min_e], marker="o", color=c_min, markersize=6.5, zorder=6, label="_nolegend_")

        # Annotations near the extrema points (auto direction to avoid going out of axes)
        _smart_annotate(x=x_max_e, y=y_max_e, text=_label("max"), color=c_max)
        _smart_annotate(x=x_min_e, y=y_min_e, text=_label("min"), color=c_min)

    # ドローン図（プロット後に重畳）
    if bool(drone):
        try:
            _overlay_drone_image(
                ax,
                gpr_mod,
                df_for_angles=df,
                mode=drone_mode,
                alpha=drone_alpha,
                inset_loc=drone_inset_loc,
                inset_size=drone_inset_size,
                inset_alpha=drone_inset_alpha,
                dpi=drone_dpi,
                phi_override=drone_phi,
                psi_override=drone_psi,
                theta_override=drone_theta,
            )
        except Exception as e:
            print(f"[warn] ドローン図の重畳に失敗: {e}")

    # 凡例（rawのカテゴリ + fitの分割 をまとめる）
    handles = []
    labels = []
    # Matplotlibが拾った凡例（raw単色やfit単色）も残す
    h0, l0 = ax.get_legend_handles_labels()
    for h, l in zip(h0, l0):
        handles.append(h)
        labels.append(l)
    # rawカテゴリ凡例
    for h in raw_handles:
        handles.append(h)
        labels.append(h.get_label())
    # fitカテゴリ凡例
    for h in fit_handles:
        handles.append(h)
        labels.append(h.get_label())

    if handles:
        # ラベル重複を除去（順序維持）
        seen = set()
        uniq_h = []
        uniq_l = []
        for h, l in zip(handles, labels):
            if l in seen:
                continue
            seen.add(l)
            uniq_h.append(h)
            uniq_l.append(l)
        ax.legend(handles=uniq_h, labels=uniq_l, fontsize=10, frameon=True, loc="best")

    plt.tight_layout()
    if output:
        plt.savefig(output, dpi=300, bbox_inches="tight")
        print(f"保存: {output}")
        plt.close(fig)
    else:
        plt.show()
        plt.close(fig)


def main() -> int:
    gpr = _import_gpr_utils()

    parser = argparse.ArgumentParser(description="gpytorchモデルのフィット曲線とCSV生データを描画（3Dドローン図重畳可）")
    parser.add_argument("csv_file", help="入力CSV")
    parser.add_argument("--load-model", required=True, help="gpytorchモデル（torch.save）")
    parser.add_argument(
        "--trust-model",
        type=gpr.parse_tf,
        default=False,
        metavar="{t,f}",
        help=(
            "torch.load の unsafe な復元を許可する（デフォルト: f）。"
            "PyTorch 2.6+ の weights_only 制限回避用。信頼できるモデルのみ t にしてください。"
        ),
    )
    parser.add_argument("--output", "-o", default=None, help="出力画像パス（未指定なら表示）")
    parser.add_argument("--target", default="torque_x", help="目的変数列名（保存モデルにあればそちらを優先）")
    parser.add_argument("--curve-x", default=None, help="横軸にする特徴量（未指定なら先頭特徴量）")
    parser.add_argument("--curve-points", type=int, default=300, help="フィット曲線の分解能（デフォルト: 300）")
    parser.add_argument("--fix", action="append", default=None, help="非表示軸の固定値 'col=value' を複数指定可")
    parser.add_argument("--fix-filter-atol", type=float, default=1e-9, help="rawを--fixで絞る際の atol（デフォルト: 1e-9）")
    parser.add_argument("--fix-filter-rtol", type=float, default=0.0, help="rawを--fixで絞る際の rtol（デフォルト: 0.0）")
    parser.add_argument("--data-filter", action="append", default=None, help="事前フィルタ 'col:min,max' を複数指定可")
    parser.add_argument("--no-uncertainty", action="store_true", help="±2σ帯を描かない")
    parser.add_argument("--ylim", default=None, help="縦軸の範囲 'min,max'（未指定なら自動）")
    parser.add_argument("--raw-alpha", type=float, default=0.65, help="rawプロット点の透明度（デフォルト: 0.65）")
    parser.add_argument("--no-title", action="store_true", help="タイトルを表示しない")
    parser.add_argument("--xlabel", default=None, help="横軸ラベル（未指定なら curve-x 列名）")
    parser.add_argument("--ylabel", default=None, help="縦軸ラベル（未指定なら target 列名）")
    parser.add_argument("--colorbar-label", default=None, help="カラーバーのラベル（未指定なら hue-raw 列名）")
    parser.add_argument("--figsize", default="10.5,7.5", help="図のサイズ '幅,高さ'（インチ、デフォルト: 10.5,7.5）")

    # 色分け
    parser.add_argument("--hue-raw", default=None, help="raw散布の色分け列（学習特徴量でなくても可）")
    parser.add_argument("--hue-fit", default=None, help="fit曲線の分割列（学習特徴量に含まれる場合のみ有効）")
    parser.add_argument("--hue-raw-cmap", default="viridis", help="hue-raw に使うカラーマップ名（数値/カテゴリ共通）")
    parser.add_argument("--hue-raw-range", default=None, help="hue-raw が数値の場合の値域 'min,max'（未指定ならデータから自動）")
    parser.add_argument("--hue-fit-cmap", default="viridis", help="hue-fit（数値）に使うカラーマップ名")
    parser.add_argument("--hue-fit-range", default=None, help="hue-fit が数値の場合の値域 'min,max'（未指定ならデータから自動）")
    parser.add_argument(
        "--x-domain-per-hue-raw",
        type=gpr.parse_tf,
        default=False,
        metavar="{t,f}",
        help="hue-raw グループごとに curve-x の定義域（min/max）を分けて fit 曲線を描画（デフォルト: f、hue-raw→hue-fit 上書きモードで有効）",
    )

    # fit extrema overlay
    parser.add_argument("--fit-extrema", type=gpr.parse_tf, default=False, metavar="{t,f}", help="fitの最大/最小を点線と注釈で表示（デフォルト: f）")
    parser.add_argument(
        "--fit-extrema-vline",
        type=gpr.parse_tf,
        default=False,
        metavar="{t,f}",
        help="fitの最大/最小位置の縦線も表示（--fit-extrema=t のとき有効、デフォルト: f）",
    )
    parser.add_argument(
        "--fit-extrema-marker",
        type=gpr.parse_tf,
        default=False,
        metavar="{t,f}",
        help="fitの最大/最小点にマーカーも表示（--fit-extrema=t のとき有効、デフォルト: f）",
    )
    parser.add_argument(
        "--fit-extrema-reverse-colors",
        type=gpr.parse_tf,
        default=False,
        metavar="{t,f}",
        help="fitの最大/最小表示の色（max=赤/min=青）を入れ替える（--fit-extrema=t のとき有効、デフォルト: f）",
    )

    # ドローン図
    parser.add_argument("--drone", type=gpr.parse_tf, default=True, metavar="{t,f}", help="3Dドローン図を重畳（デフォルト: t）")
    parser.add_argument("--drone-mode", choices=["background", "inset", "both"], default="background", help="ドローン図の描画モード")
    parser.add_argument("--drone-alpha", type=float, default=0.12, help="背景ドローン図の透明度")
    parser.add_argument("--drone-inset-loc", choices=["ur", "ul", "lr", "ll"], default="ur", help="インセット位置")
    parser.add_argument("--drone-inset-size", type=float, default=0.33, help="インセットサイズ（軸比）")
    parser.add_argument("--drone-inset-alpha", type=float, default=1.0, help="インセット透明度")
    parser.add_argument("--drone-dpi", type=int, default=160, help="ドローン図レンダリングDPI")
    parser.add_argument("--drone-phi", type=float, default=None, help="fold(phi)角を上書き（deg）")
    parser.add_argument("--drone-psi", type=float, default=None, help="slant(psi)角を上書き（deg）")
    parser.add_argument("--drone-theta", type=float, default=None, help="tilt(theta)角を上書き（deg）")

    # 追加: group-by（必要時のみ。単一プロットが主のため最小限）
    parser.add_argument("--group-by", default=None, help="グループ別サブプロットにする列（カンマ区切り）")
    parser.add_argument("--title", default=None, help="図タイトル（未指定なら自動）")

    args = parser.parse_args()

    # ヘッドレス対応
    if (not os.environ.get("DISPLAY")) or args.output:
        try:
            matplotlib.use("Agg", force=True)
        except Exception:
            pass

    # モデル読み込み
    backend, bundle, scaler, feature_columns, saved_target = gpr.load_model_any(
        args.load_model, trust=bool(getattr(args, "trust_model", False))
    )
    if backend != "gpytorch":
        raise RuntimeError(f"このスクリプトは gpytorch 保存モデル専用です。backend={backend}")
    wrapper = gpr.build_gpytorch_wrapper_from_bundle(bundle, device_str="auto")

    if saved_target:
        if args.target != saved_target:
            print(f"注意: 保存モデルの target='{saved_target}' を優先し、--target='{args.target}' を上書きします。")
        target_column = str(saved_target)
    else:
        target_column = str(args.target)

    if feature_columns is None:
        raise RuntimeError("保存モデルに feature_columns がありません。gaussian_process_regression.py の保存形式を確認してください。")
    feature_columns = list(feature_columns)

    # CSV読み込み・前処理（特徴量は保存モデルのものを固定で使用）
    df_clean, _ = gpr.load_and_preprocess_data(
        args.csv_file, selected_features=feature_columns, max_variance_torque_x=None, target_column=target_column, debug=False
    )

    # フィルタ適用
    df_plot = gpr.apply_facet_filters(df_clean, args.data_filter)
    if df_plot is None or df_plot.empty:
        raise RuntimeError("data-filter により有効なデータがありません。")

    # curve_x 決定
    curve_x = args.curve_x
    if not curve_x:
        if not feature_columns:
            raise RuntimeError("特徴量が空です。")
        curve_x = feature_columns[0]
    curve_x = str(curve_x).strip()

    # group-by（簡易対応：指定時は gaussian_process_regression.py の既存関数を使う）
    if args.group_by:
        group_by = [c.strip() for c in str(args.group_by).split(",") if c.strip()]
        if not group_by:
            raise RuntimeError("--group-by が空です。")
        # hue-fit は学習特徴量のみ対応。hue-raw はここでは対応しない（単一プロット主のため）。
        # 必要なら後で拡張できるが、追加機能になるのでここでは入れない。
        hue_fit = str(args.hue_fit).strip() if args.hue_fit else None
        if hue_fit and hue_fit not in feature_columns:
            print(f"[warn] --hue-fit '{hue_fit}' は学習特徴量に無いため無視します。")
            hue_fit = None
        gpr.plot_grouped_raw_and_fit_gpr._target_column = target_column
        gpr.plot_grouped_raw_and_fit_gpr(
            df_clean=df_plot,
            feature_columns=feature_columns,
            model=wrapper,
            scaler=scaler,
            group_by=group_by,
            curve_x=curve_x,
            curve_points=int(args.curve_points),
            ranges=None,
            output_file=args.output,
            show_uncertainty=(not bool(args.no_uncertainty)),
            hue=hue_fit,
            fixes=args.fix,
        )
        return 0

    # 単一プロット
    if args.no_title:
        title = None
    elif args.title:
        title = args.title
    else:
        title = f"raw + gpytorch fit ({target_column})"
        if args.hue_raw:
            title += f"  hue_raw={args.hue_raw}"
        if args.hue_fit:
            title += f"  hue_fit={args.hue_fit}"

    # rawの--fixフィルタ許容誤差を plot_single_raw_and_fit へ渡す（関数属性で簡易に受け渡し）
    plot_single_raw_and_fit._fix_filter_atol = float(getattr(args, "fix_filter_atol", 1e-9))
    plot_single_raw_and_fit._fix_filter_rtol = float(getattr(args, "fix_filter_rtol", 0.0))

    plot_single_raw_and_fit(
        df=df_plot,
        feature_columns=feature_columns,
        target_column=target_column,
        model=wrapper,
        scaler=scaler,
        curve_x=curve_x,
        curve_points=int(args.curve_points),
        fixes=args.fix,
        hue_raw=(str(args.hue_raw).strip() if args.hue_raw else None),
        hue_fit=(str(args.hue_fit).strip() if args.hue_fit else None),
        hue_raw_cmap=str(args.hue_raw_cmap),
        hue_raw_range=_parse_float_pair_opt(args.hue_raw_range, name="--hue-raw-range"),
        hue_fit_cmap=str(args.hue_fit_cmap),
        hue_fit_range=_parse_float_pair_opt(args.hue_fit_range, name="--hue-fit-range"),
        x_domain_per_hue_raw=bool(getattr(args, "x_domain_per_hue_raw", False)),
        show_uncertainty=(not bool(args.no_uncertainty)),
        ylim=_parse_float_pair_opt(args.ylim, name="--ylim"),
        raw_alpha=float(args.raw_alpha),
        fit_extrema=bool(getattr(args, "fit_extrema", False)),
        fit_extrema_vline=bool(getattr(args, "fit_extrema_vline", False)),
        fit_extrema_marker=bool(getattr(args, "fit_extrema_marker", False)),
        fit_extrema_reverse_colors=bool(getattr(args, "fit_extrema_reverse_colors", False)),
        xlabel=args.xlabel,
        ylabel=args.ylabel,
        colorbar_label=args.colorbar_label,
        figsize=_parse_float_pair_opt(args.figsize, name="--figsize") or (10.5, 7.5),
        title=title,
        drone=bool(args.drone),
        drone_mode=str(args.drone_mode),
        drone_alpha=float(args.drone_alpha),
        drone_inset_loc=str(args.drone_inset_loc),
        drone_inset_size=float(args.drone_inset_size),
        drone_inset_alpha=float(args.drone_inset_alpha),
        drone_dpi=int(args.drone_dpi),
        drone_phi=args.drone_phi,
        drone_psi=args.drone_psi,
        drone_theta=args.drone_theta,
        output=(str(args.output) if args.output else None),
        gpr_mod=gpr,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


