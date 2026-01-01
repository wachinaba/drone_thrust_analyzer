from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd


ZERO_EPS = 1e-9
DIFF_RATIO_IQR_K = 1.5


def _robust_max_iqr(vals: pd.Series) -> float:
    """IQR で外れ値影響を抑えたロバスト最大値（正の値のみ）を返す。"""
    v = pd.to_numeric(vals, errors="coerce")
    v = v[np.isfinite(v)]
    v = v[v > ZERO_EPS]
    if v.empty:
        return np.nan
    q1 = v.quantile(0.25)
    q3 = v.quantile(0.75)
    iqr = q3 - q1
    if np.isfinite(iqr) and iqr > 0:
        thresh = q3 + DIFF_RATIO_IQR_K * iqr
        v = v[v <= thresh]
        if v.empty:
            return np.nan
    return float(v.max())


def _safe_div(num: pd.Series | np.ndarray, den: pd.Series | np.ndarray) -> np.ndarray:
    num = np.asarray(num, dtype=float)
    den = np.asarray(den, dtype=float)
    out = np.full_like(num, np.nan, dtype=float)
    ok = np.isfinite(num) & np.isfinite(den) & (np.abs(den) > ZERO_EPS)
    out[ok] = num[ok] / den[ok]
    return out


@dataclass(frozen=True)
class CriteriaConfig:
    prop_center_x_m: float = 0.125
    wall_open_by_xrel_sign: bool = True


CRITERIA_FIELDS = [
    "ux",
    "ux_norm",
    "ux_norm_y",
    "uy",
    "uy_norm",
    "uy_norm_y",
    "u",
    "u_norm",
    "u_norm_y",
]


def add_normalized_fields(df_long: pd.DataFrame) -> pd.DataFrame:
    """df_long に ux_norm/ux_norm_y, uy_norm/uy_norm_y, u_norm/u_norm_y を追加して返す。

    正規化は facet 内（row_facet,col_facet）かつ io, side ごとに行う：
      - *_norm: group=(row_facet,col_facet,io,side)
      - *_norm_y: group=(row_facet,col_facet,io,y)  ※ side を跨いだロバスト最大で割る
    分母はロバスト最大値（ux/uy は abs を最大化し、符号は保持）を用いる。
    """
    df = df_long.copy()
    required = ["row_facet", "col_facet", "io", "side", "y", "ux", "uy", "u_mag"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"criteria: df_long に必要な列が不足しています: {', '.join(missing)}")

    base = ["row_facet", "col_facet", "io", "side"]
    base_y = ["row_facet", "col_facet", "io", "y"]

    ux_abs = df["ux"].abs()
    uy_abs = df["uy"].abs()
    u = df["u_mag"]

    ux_max = df.groupby(base, dropna=False).apply(lambda g: _robust_max_iqr(g["ux"].abs())).rename("ux_max")
    uy_max = df.groupby(base, dropna=False).apply(lambda g: _robust_max_iqr(g["uy"].abs())).rename("uy_max")
    u_max = df.groupby(base, dropna=False)["u_mag"].transform(_robust_max_iqr)

    # map back for ux/uy (groupby.apply gives MultiIndex series)
    ux_max_df = ux_max.reset_index()
    uy_max_df = uy_max.reset_index()
    df = df.merge(ux_max_df, on=base, how="left")
    df = df.merge(uy_max_df, on=base, how="left")
    df["u_max"] = u_max

    df["ux_norm"] = _safe_div(df["ux"], df["ux_max"])
    df["uy_norm"] = _safe_div(df["uy"], df["uy_max"])
    df["u_norm"] = _safe_div(u, df["u_max"])

    # per-y normalization
    ux_max_y = (
        df.groupby(base_y, dropna=False)
        .apply(lambda g: _robust_max_iqr(g["ux"].abs()))
        .rename("ux_max_y")
        .reset_index()
    )
    uy_max_y = (
        df.groupby(base_y, dropna=False)
        .apply(lambda g: _robust_max_iqr(g["uy"].abs()))
        .rename("uy_max_y")
        .reset_index()
    )
    u_max_y = df.groupby(base_y, dropna=False)["u_mag"].transform(_robust_max_iqr)

    df = df.merge(ux_max_y, on=base_y, how="left")
    df = df.merge(uy_max_y, on=base_y, how="left")
    df["u_max_y"] = u_max_y

    df["ux_norm_y"] = _safe_div(df["ux"], df["ux_max_y"])
    df["uy_norm_y"] = _safe_div(df["uy"], df["uy_max_y"])
    df["u_norm_y"] = _safe_div(u, df["u_max_y"])

    # cleanup helper cols
    df = df.drop(columns=["ux_max", "uy_max", "u_max", "ux_max_y", "uy_max_y", "u_max_y"], errors="ignore")
    return df


def compute_criteria_table(
    df_long: pd.DataFrame,
    criteria_field: str,
    cfg: CriteriaConfig | None = None,
    *,
    by_y: bool = False,
) -> pd.DataFrame:
    """criteria.tex に基づく指標を facet × io × side ごとに集計する。

    - 非対称指数 I_asym と流束重心 X_center は criteria_field の値を用いる
    - 偏角 theta_def は定義の都合上 ux/uy から計算（criteria_field に依存しない）
    - Wall/Open は機体中心基準 x の符号で分ける（x>0 を Wall, x<0 を Open）
    """
    if cfg is None:
        cfg = CriteriaConfig()

    if criteria_field not in CRITERIA_FIELDS:
        raise ValueError(f"criteria_field が不正です: {criteria_field} (choices={CRITERIA_FIELDS})")

    df = add_normalized_fields(df_long)

    if criteria_field == "ux":
        scalar = df["ux"]
    elif criteria_field == "ux_norm":
        scalar = df["ux_norm"]
    elif criteria_field == "ux_norm_y":
        scalar = df["ux_norm_y"]
    elif criteria_field == "uy":
        scalar = df["uy"]
    elif criteria_field == "uy_norm":
        scalar = df["uy_norm"]
    elif criteria_field == "uy_norm_y":
        scalar = df["uy_norm_y"]
    elif criteria_field == "u":
        scalar = df["u_mag"]
    elif criteria_field == "u_norm":
        scalar = df["u_norm"]
    elif criteria_field == "u_norm_y":
        scalar = df["u_norm_y"]
    else:
        raise AssertionError("unreachable")

    df = df.copy()
    df["_scalar"] = pd.to_numeric(scalar, errors="coerce")

    if "x" not in df.columns:
        raise ValueError("criteria: df_long に 'x' 列がありません（座標計算後の df_long が必要です）。")

    # x [mm] -> [m] （プロペラ中心 x_c の扱いは無効化：機体中心基準の x をそのまま使う）
    x_m = pd.to_numeric(df["x"], errors="coerce") / 1000.0
    side = df["side"].astype(str)
    df["x_rel_m"] = x_m

    # Wall/Open masks
    df["_is_wall"] = df["x_rel_m"] > 0
    df["_is_open"] = df["x_rel_m"] < 0

    # theta_def (per side): outward-positive horizontal, downward-positive vertical
    # outward: front:+ux, rear:-ux
    ux = pd.to_numeric(df["ux"], errors="coerce")
    uy = pd.to_numeric(df["uy"], errors="coerce")
    df["_v_h"] = np.where(side == "front", ux, -ux)
    df["_v_v"] = -uy  # downward positive

    group = ["row_facet", "col_facet", "io", "side"]
    if by_y:
        group = group + ["y"]

    def _agg(g: pd.DataFrame) -> pd.Series:
        s = pd.to_numeric(g["_scalar"], errors="coerce")
        xrel = pd.to_numeric(g["x_rel_m"], errors="coerce")
        wall = g["_is_wall"].to_numpy(dtype=bool)
        open_ = g["_is_open"].to_numpy(dtype=bool)

        # means for asymmetry
        mean_total = float(np.nanmean(s))
        mean_wall = float(np.nanmean(s[wall])) if np.any(wall) else np.nan
        mean_open = float(np.nanmean(s[open_])) if np.any(open_) else np.nan
        denom = abs(mean_total)
        if np.isfinite(mean_wall) and np.isfinite(mean_open) and np.isfinite(denom) and denom > ZERO_EPS:
            i_asym = abs(mean_wall - mean_open) / denom
        else:
            i_asym = np.nan

        # flow center (signed weight: criteria.tex の定義 w をそのまま使う)
        w = s.to_numpy(dtype=float)
        xr = xrel.to_numpy(dtype=float)
        ok = np.isfinite(w) & np.isfinite(xr)
        w = w[ok]
        xr = xr[ok]
        sw = float(np.sum(w)) if w.size else 0.0
        if w.size > 0 and np.isfinite(sw) and abs(sw) > ZERO_EPS:
            x_center = float(np.sum(xr * w) / sw)
        else:
            x_center = np.nan

        # deflection angle
        vh = pd.to_numeric(g["_v_h"], errors="coerce").to_numpy(dtype=float)
        vv = pd.to_numeric(g["_v_v"], errors="coerce").to_numpy(dtype=float)
        vh_mean = float(np.nanmean(vh))
        vv_mean = float(np.nanmean(vv))
        if np.isfinite(vh_mean) and np.isfinite(vv_mean) and abs(vv_mean) > ZERO_EPS:
            theta_deg = float(np.degrees(np.arctan2(vh_mean, vv_mean)))
        else:
            theta_deg = np.nan

        return pd.Series(
            {
                "criteria_field": g["criteria_field"].iloc[0] if "criteria_field" in g.columns else np.nan,
                "I_asym": i_asym,
                "X_center_m": x_center,
                "theta_def_deg": theta_deg,
                "mean_total": mean_total,
                "mean_wall": mean_wall,
                "mean_open": mean_open,
                "n_total": int(np.isfinite(s).sum()),
                "n_wall": int(np.isfinite(s[wall]).sum()) if np.any(wall) else 0,
                "n_open": int(np.isfinite(s[open_]).sum()) if np.any(open_) else 0,
            }
        )

    df["criteria_field"] = criteria_field
    out = df.groupby(group, dropna=False).apply(_agg).reset_index()
    # helpful unit conversion for display
    out["X_center_mm"] = out["X_center_m"] * 1000.0
    return out


