#!/usr/bin/env python3
import argparse
import os
import warnings
from typing import Dict, Tuple, Optional, Sequence, Any

# Matplotlib 環境によっては Axes3D が読めず警告が出ることがあるが、本スクリプトの機能には不要なので抑制する
warnings.filterwarnings(
    "ignore",
    message=r"Unable to import Axes3D\..*",
    category=UserWarning,
    module=r"matplotlib\..*",
)

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from joblib import load
from tqdm import tqdm

from auto_thrust_recorder.analysis.metrics import compute_metric_values
from auto_thrust_recorder.analysis.integration import build_grid, integrate_values
from auto_thrust_recorder.analysis.plotting import plot_1d, plot_2d, dump_csv


def _connected_components_2d(mask: np.ndarray, *, connectivity: int = 8) -> Sequence[np.ndarray]:
    """
    2D bool mask の連結成分を抽出する（SciPy不要）。
    Returns: list of (N,2) int array [iy, ix]
    """
    m = np.asarray(mask, dtype=bool)
    if m.ndim != 2:
        return []
    ny, nx = m.shape
    visited = np.zeros_like(m, dtype=bool)
    comps: list[np.ndarray] = []
    if connectivity == 4:
        neigh = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    else:
        neigh = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    for iy in range(ny):
        for ix in range(nx):
            if (not m[iy, ix]) or visited[iy, ix]:
                continue
            stack = [(iy, ix)]
            visited[iy, ix] = True
            coords: list[tuple[int, int]] = []
            while stack:
                cy, cx = stack.pop()
                coords.append((cy, cx))
                for dy, dx in neigh:
                    nyy = cy + dy
                    nxx = cx + dx
                    if (0 <= nyy < ny) and (0 <= nxx < nx) and m[nyy, nxx] and (not visited[nyy, nxx]):
                        visited[nyy, nxx] = True
                        stack.append((nyy, nxx))
            comps.append(np.asarray(coords, dtype=int))
    return comps


def _optima_2d_from_combined(
    combined_improve_pct: np.ndarray,
    x_vals: np.ndarray,
    y_vals: np.ndarray,
    *,
    top_pct: float = 5.0,
    delta: Optional[float] = None,
    connectivity: int = 8,
    max_islands: int = 10,
) -> Dict[str, Any]:
    """
    combined_improve_pct(大きいほど良い) の2Dグリッドから、
    準最適集合(top-pct or vmax-delta)を抽出し、連結成分ごとに代表点を返す。
    代表点は島内点の (median(x), median(y)) に最も近い点（medoid的）。
    """
    V = np.asarray(combined_improve_pct, dtype=float)
    if V.ndim != 2:
        raise ValueError("combined_improve_pct must be 2D")
    finite = np.isfinite(V)
    if not bool(np.any(finite)):
        return {"mask": None, "threshold": None, "vmax": None, "islands": [], "rep_points": None, "labels": []}

    vals = V[finite]
    vmax = float(np.max(vals))

    if delta is not None:
        thr = vmax - float(delta)
        method_desc = f"delta={float(delta):g}"
    else:
        p = float(top_pct)
        if not (0.0 < p <= 100.0):
            raise ValueError("--optima-top-pct は (0, 100] の範囲で指定してください")
        q = 100.0 - p
        thr = float(np.percentile(vals, q))
        method_desc = f"top_pct={p:g}"

    mask = finite & (V >= thr)
    comps = _connected_components_2d(mask, connectivity=int(connectivity))

    Xv = np.asarray(x_vals, dtype=float).reshape(-1)
    Yv = np.asarray(y_vals, dtype=float).reshape(-1)
    if V.shape != (len(Yv), len(Xv)):
        # (ny,nx) を想定
        raise ValueError(f"shape mismatch: combined={V.shape}, x={len(Xv)}, y={len(Yv)}")

    # step sizes for rough area estimate
    dx = float(np.median(np.abs(np.diff(Xv)))) if len(Xv) >= 2 else 0.0
    dy = float(np.median(np.abs(np.diff(Yv)))) if len(Yv) >= 2 else 0.0
    cell_area = (dx * dy) if (dx > 0.0 and dy > 0.0) else None

    islands: list[Dict[str, Any]] = []
    for comp in comps:
        iys = comp[:, 0]
        ixs = comp[:, 1]
        xs = Xv[ixs]
        ys = Yv[iys]
        # island center (for stable tie-break)
        cx = float(np.median(xs))
        cy = float(np.median(ys))

        island_vals = np.asarray(V[iys, ixs], dtype=float)
        island_max = float(np.nanmax(island_vals))
        island_med = float(np.nanmedian(island_vals))

        # representative point = argmax within island (ties -> nearest to median center)
        max_mask = np.isfinite(island_vals) & (island_vals == island_max)
        cand = np.where(max_mask)[0]
        if cand.size == 0:
            # fallback: nearest to center among all cells (should be rare)
            d2 = (xs - cx) ** 2 + (ys - cy) ** 2
            k = int(np.argmin(d2))
        elif cand.size == 1:
            k = int(cand[0])
        else:
            d2c = (xs[cand] - cx) ** 2 + (ys[cand] - cy) ** 2
            k = int(cand[int(np.argmin(d2c))])

        rep_ix = int(ixs[k])
        rep_iy = int(iys[k])
        rep_x = float(Xv[rep_ix])
        rep_y = float(Yv[rep_iy])
        rep_v = float(island_max)
        n_cells = int(comp.shape[0])
        area = (float(n_cells) * float(cell_area)) if (cell_area is not None) else float(n_cells)
        islands.append(
            dict(
                center_median_x=cx,
                center_median_y=cy,
                rep_x=rep_x,
                rep_y=rep_y,
                rep_value=rep_v,
                island_max=island_max,
                island_median=island_med,
                n_cells=n_cells,
                area=area,
            )
        )

    # rank (multi-solution): max value desc, then size desc
    islands.sort(key=lambda d: (-float(d["island_max"]), -int(d["n_cells"])))
    if max_islands is not None and int(max_islands) > 0:
        islands = islands[: int(max_islands)]

    rep_points = None
    labels: list[str] = []
    if islands:
        rep_points = np.asarray([[d["rep_x"], d["rep_y"]] for d in islands], dtype=float)
        labels = [str(i + 1) for i in range(len(islands))]

    return {
        "mask": mask,
        "threshold": float(thr),
        "vmax": float(vmax),
        "method": method_desc,
        "islands": islands,
        "rep_points": rep_points,
        "labels": labels,
    }


def _pretty_axis_symbol(name: str) -> str:
    s = str(name or "").strip()
    low = s.lower()
    if "alpha" in low or low in {"a", "α"}:
        return "α"
    if "beta" in low or low in {"b", "β"}:
        return "β"
    return s if s else "x"


def _format_peak_list_text(
    islands: Sequence[Dict[str, Any]],
    *,
    x_label: str,
    y_label: str,
    max_items: int = 10,
) -> str:
    """
    例:
      Top Improve
      1: 12.3 % (at α=0.0, β=5.0)
      ...
    """
    if not islands:
        return ""
    xl = _pretty_axis_symbol(x_label)
    yl = _pretty_axis_symbol(y_label)
    lines = ["Top Improve"]
    n = min(int(max_items) if max_items else len(islands), len(islands))
    for i in range(n):
        d = islands[i]
        v = float(d.get("rep_value", float("nan")))
        x = float(d.get("rep_x", float("nan")))
        y = float(d.get("rep_y", float("nan")))
        lines.append(f"{i+1}: {v:.3g} % (at {xl}={x:.3g}, {yl}={y:.3g})")
    return "\n".join(lines)

def _convex_hull_2d(points_xy: np.ndarray) -> Optional[np.ndarray]:
    """
    2D点群 (N,2) の凸包を monotone chain で計算する（SciPy不要）。
    Returns: hull vertices (M,2) in CCW order, without repeating the first point.
             点が足りない場合は None。
    """
    pts = np.asarray(points_xy, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        return None
    # drop non-finite
    m = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1])
    pts = pts[m]
    if pts.shape[0] < 3:
        return None
    # unique points
    pts = np.unique(pts, axis=0)
    if pts.shape[0] < 3:
        return None
    # sort by x then y
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b) -> float:
        return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))

    lower: list[np.ndarray] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)

    upper: list[np.ndarray] = []
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)

    hull = np.vstack([np.asarray(lower), np.asarray(upper[1:-1])])
    if hull.shape[0] < 3:
        return None
    return hull


def _mask_by_polygon_2d(Z: np.ndarray, x_vals: np.ndarray, y_vals: np.ndarray, poly_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Z (ny,nx) を polygon 外で NaN にする。
    Returns: (Z_masked, inside_mask[ny,nx])
    """
    Z0 = np.asarray(Z, dtype=float)
    Xv = np.asarray(x_vals, dtype=float).reshape(-1)
    Yv = np.asarray(y_vals, dtype=float).reshape(-1)
    if Z0.shape != (len(Yv), len(Xv)):
        raise ValueError(f"shape mismatch: Z={Z0.shape}, x={len(Xv)}, y={len(Yv)}")
    poly = np.asarray(poly_xy, dtype=float)
    if poly.ndim != 2 or poly.shape[1] != 2 or poly.shape[0] < 3:
        return Z0, np.ones_like(Z0, dtype=bool)

    # build grid points (N,2)
    Xg, Yg = np.meshgrid(Xv, Yv)
    pts = np.stack([Xg.reshape(-1), Yg.reshape(-1)], axis=1)
    path = _make_closed_path(poly)
    if path is None:
        return Z0, np.ones_like(Z0, dtype=bool)
    inside = path.contains_points(pts)
    inside_mask = inside.reshape(Z0.shape)
    Zm = Z0.copy()
    Zm[~inside_mask] = np.nan
    return Zm, inside_mask


def _make_closed_path(poly_xy: np.ndarray):
    """
    Build a properly closed matplotlib Path for polygon containment tests.

    NOTE:
      Path(poly, closed=True) だけだと、contains_point(s) が閉路として扱わないケースがあるため、
      CLOSEPOLY コードで明示的に閉じる。
    """
    from matplotlib.path import Path  # local import

    poly = np.asarray(poly_xy, dtype=float)
    if poly.ndim != 2 or poly.shape[1] != 2 or poly.shape[0] < 3:
        return None
    # codes + CLOSEPOLY require one extra vertex (ignored for CLOSEPOLY)
    verts = np.vstack([poly, poly[0]])
    codes = [Path.MOVETO] + [Path.LINETO] * (poly.shape[0] - 1) + [Path.CLOSEPOLY]
    return Path(verts, codes)


def _bbox_str_xy(points_xy: Optional[np.ndarray]) -> str:
    """
    points_xy: (N,2) array
    Return bbox string for debug.
    """
    if points_xy is None:
        return "(none)"
    pts = np.asarray(points_xy, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] == 0:
        return f"(invalid shape={getattr(pts, 'shape', None)})"
    m = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1])
    if not bool(np.any(m)):
        return "(no finite points)"
    x = pts[m, 0]
    y = pts[m, 1]
    return f"x[{float(np.min(x)):.6g}, {float(np.max(x)):.6g}] y[{float(np.min(y)):.6g}, {float(np.max(y)):.6g}] n={int(np.sum(m))}"


def _hull_none_reason(points_xy: Optional[np.ndarray]) -> str:
    """
    Explain why convex hull could not be formed (best-effort).
    """
    if points_xy is None:
        return "pts_hull is None (missing columns or no points after filtering)"
    pts = np.asarray(points_xy, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        return f"invalid pts_hull shape={getattr(pts, 'shape', None)}"
    if pts.shape[0] < 3:
        return f"too few points: n={pts.shape[0]}"
    m = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1])
    nf = int(np.sum(m))
    if nf < 3:
        return f"too few finite points: n_finite={nf}/{pts.shape[0]}"
    uniq = np.unique(pts[m], axis=0)
    nu = int(uniq.shape[0])
    if nu < 3:
        return f"too few unique finite points: n_unique={nu} (duplicates?)"
    return "degenerate hull (likely collinear points) or hull computation returned <3 vertices"


def _unique_sorted_points_xy(points_xy: Optional[np.ndarray]) -> np.ndarray:
    """
    points_xy: (N,2) -> unique finite points (M,2), sorted by x then y.
    """
    if points_xy is None:
        return np.zeros((0, 2), dtype=float)
    pts = np.asarray(points_xy, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=float)
    m = np.isfinite(pts[:, 0]) & np.isfinite(pts[:, 1])
    pts = pts[m]
    if pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=float)
    pts = np.unique(pts, axis=0)
    if pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=float)
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]  # sort by x then y
    return pts


def _print_points_block(prefix: str, title: str, pts_xy: np.ndarray) -> None:
    """
    Print all points to stdout, one per line: '  x,y'
    """
    pts = np.asarray(pts_xy, dtype=float)
    n = int(pts.shape[0]) if (pts.ndim == 2 and pts.shape[1] == 2) else 0
    print(f"{prefix} {title} (n={n})")
    if n == 0:
        return
    for i in range(n):
        print(f"{prefix}   {pts[i, 0]:.10g},{pts[i, 1]:.10g}")


def parse_normalize_ref(text: str) -> Dict[str, float]:
    """
    Parse normalize reference specification.
    format: 'col1=val1,col2=val2' or 'col=val' (for 1D)
    examples: 'prop_spacing_x=0.3,prop_spacing_y=0.3', 'distance=1.0'
    """
    result = {}
    for item in text.split(','):
        item = item.strip()
        if '=' not in item:
            raise ValueError(f"normalize-ref の各項目は 'col=value' 形式で指定してください: '{item}'")
        col, val = item.split('=', 1)
        result[col.strip()] = float(val.strip())
    return result


def find_nearest_index(arr: np.ndarray, value: float) -> int:
    """Find index of nearest value in array."""
    return int(np.argmin(np.abs(arr - value)))


def normalize_by_ref_1d(
    Z: np.ndarray,
    axis_vals: np.ndarray,
    axis_name: str,
    ref_spec: Dict[str, float],
) -> Tuple[np.ndarray, float, Dict[str, float]]:
    """
    Normalize 1D array by reference point value.
    Returns: (normalized_Z, ref_value, actual_ref_coords)
    """
    if axis_name not in ref_spec:
        raise ValueError(f"--normalize-ref に軸 '{axis_name}' が指定されていません")
    ref_val = ref_spec[axis_name]
    idx = find_nearest_index(axis_vals, ref_val)
    actual_val = float(axis_vals[idx])
    ref_z = float(Z.flat[idx])
    if ref_z == 0.0:
        raise ValueError(f"基準点の値が0です。正規化できません。(ref={actual_val}, Z={ref_z})")
    return Z / ref_z, ref_z, {axis_name: actual_val}


def normalize_by_ref_2d(
    Z: np.ndarray,
    x_vals: np.ndarray,
    y_vals: np.ndarray,
    x_name: str,
    y_name: str,
    ref_spec: Dict[str, float],
) -> Tuple[np.ndarray, float, Dict[str, float]]:
    """
    Normalize 2D array by reference point value.
    Z.shape == (len(y_vals), len(x_vals)) を想定（行=Y, 列=X）
    Returns: (normalized_Z, ref_value, actual_ref_coords)
    """
    if x_name not in ref_spec:
        raise ValueError(f"--normalize-ref に軸 '{x_name}' が指定されていません")
    if y_name not in ref_spec:
        raise ValueError(f"--normalize-ref に軸 '{y_name}' が指定されていません")
    ref_x = ref_spec[x_name]
    ref_y = ref_spec[y_name]
    ix = find_nearest_index(x_vals, ref_x)
    iy = find_nearest_index(y_vals, ref_y)
    actual_x = float(x_vals[ix])
    actual_y = float(y_vals[iy])
    ref_z = float(Z[iy, ix])
    if ref_z == 0.0:
        raise ValueError(f"基準点の値が0です。正規化できません。(ref=({actual_x}, {actual_y}), Z={ref_z})")
    return Z / ref_z, ref_z, {x_name: actual_x, y_name: actual_y}


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


def parse_span_auto(text: str, col: str, df: pd.DataFrame) -> Tuple[float, float, int]:
    """
    format: min,max[:points]
      - min/max は float もしくは 'auto'
      - 例: 'auto,9.0', '0.5,auto:300', 'auto,auto:200'
    auto の場合、df[col] の観測 min/max（数値変換後）を使用する。
    """
    # split points
    if ':' in text:
        rng, pts_str = text.split(':', 1)
        pts = int(pts_str) if pts_str.strip() else 0
    else:
        rng = text
        pts = 0
    if ',' not in rng:
        raise ValueError(f"span '{text}' must be 'min,max[:N]' (min/max may be 'auto')")

    if col not in df.columns:
        raise ValueError(f"column '{col}' not found in CSV (needed for auto range)")
    s = pd.to_numeric(df[col], errors="coerce")
    s = s[np.isfinite(s.values)]
    if len(s) == 0:
        raise ValueError(f"column '{col}' has no finite numeric values; cannot use auto range")
    observed_min = float(s.min())
    observed_max = float(s.max())

    lo_tok, hi_tok = rng.split(',', 1)
    lo_tok = lo_tok.strip()
    hi_tok = hi_tok.strip()

    def resolve(tok: str, which: str) -> float:
        if tok.lower() == "auto":
            return observed_min if which == "lo" else observed_max
        return float(tok)

    lo = resolve(lo_tok, "lo")
    hi = resolve(hi_tok, "hi")
    return lo, hi, pts


def _parse_metrics_list(text: Optional[str], *, fallback: str) -> Sequence[str]:
    """
    Parse comma-separated metric names.
    If text is falsy -> [fallback]
    """
    if not text:
        return [fallback]
    items = [t.strip() for t in str(text).split(",") if t.strip()]
    return items if items else [fallback]


def _parse_weights(text: Optional[str]) -> Dict[str, float]:
    """
    format: 'moment_abs=1,grad_abs=1' (also accepts moment/grad aliases)
    Returns canonical keys: {'moment_abs': w1, 'grad_abs': w2}
    """
    if not text:
        return {"moment_abs": 1.0, "grad_abs": 1.0}
    raw: Dict[str, float] = {}
    for item in str(text).split(","):
        item = item.strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError(f"--weights の各項目は 'name=value' 形式で指定してください: '{item}'")
        k, v = item.split("=", 1)
        raw[k.strip().lower()] = float(v.strip())

    def pick(keys, default=None):
        for kk in keys:
            if kk in raw:
                return raw[kk]
        return default

    w_m = pick(["moment_abs", "moment", "m"], 1.0)
    w_g = pick(["grad_abs", "grad", "g"], 1.0)
    return {"moment_abs": float(w_m), "grad_abs": float(w_g)}


def _safe_log_ratio(r: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    rr = np.asarray(r, dtype=float)
    rr = np.clip(rr, eps, np.inf)
    return np.log(rr)


def _improve_pct_from_log_ratio(score_log_ratio: np.ndarray) -> np.ndarray:
    """
    score_log_ratio = log(gm_ratio) where gm_ratio is a (weighted) geometric-mean ratio (1.0 at ref).
    Return "normal" improvement percentage: (1 - gm_ratio) * 100.
      - gm_ratio < 1 => positive improvement
      - gm_ratio > 1 => negative (worse)
    """
    s = np.asarray(score_log_ratio, dtype=float)
    gm_ratio = np.exp(s)
    return (1.0 - gm_ratio) * 100.0


class _SimpleStandardScaler:
    """
    torch.save(weights_only=True) で読める dict 形式の StandardScaler を復元するための簡易実装。
    sklearn が無い環境でも最低限 transform できるようにする。
    """
    def __init__(self, payload: dict):
        self.with_mean = bool(payload.get("with_mean", True))
        self.with_std = bool(payload.get("with_std", True))
        mean_ = payload.get("mean_", None)
        scale_ = payload.get("scale_", None)
        self.mean_ = None if mean_ is None else np.asarray(mean_, dtype=float)
        self.scale_ = None if scale_ is None else np.asarray(scale_, dtype=float)

    def transform(self, X):
        X = np.asarray(X, dtype=float)
        Y = X
        if self.with_mean and (self.mean_ is not None):
            Y = Y - self.mean_
        if self.with_std and (self.scale_ is not None):
            s = np.where(self.scale_ == 0.0, 1.0, self.scale_)
            Y = Y / s
        return Y


def _deserialize_scaler(obj):
    # joblib で sklearn オブジェクトがそのまま入っている場合は transform を持つのでそのまま使う
    if obj is None:
        return None
    if hasattr(obj, "transform"):
        return obj
    if isinstance(obj, dict) and obj.get("type") == "StandardScaler":
        return _SimpleStandardScaler(obj)
    return None


def _select_torch_device(device_str: str):
    try:
        import torch  # type: ignore
    except Exception:
        return None
    s = (device_str or "auto").strip().lower()
    if s == "cpu":
        return torch.device("cpu")
    if s == "cuda":
        return torch.device("cuda")
    # auto
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def _try_import_torch_gpytorch():
    try:
        import torch  # type: ignore
        import gpytorch  # type: ignore
        return torch, gpytorch
    except Exception:
        return None, None


def _load_torch_bundle(path: str, trust: bool):
    torch, _ = _try_import_torch_gpytorch()
    if torch is None:
        raise RuntimeError("torch が import できません。gpytorchモデルを読むには torch/gpytorch が必要です。")
    # PyTorch 2.6+: weights_only の既定が True になったため、明示指定して挙動を安定化
    try:
        bundle = torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        bundle = torch.load(path, map_location="cpu")
    except Exception as e:
        msg = str(e)
        if ("Weights only load failed" in msg) or ("WeightsUnpickler" in msg):
            if not trust:
                raise RuntimeError(
                    "PyTorch の安全ロード(weights_only=True)で読み込めませんでした。"
                    "このモデルファイルを信頼できる場合は --trust-model t を付けて再実行してください。"
                ) from e
            try:
                bundle = torch.load(path, map_location="cpu", weights_only=False)
            except TypeError:
                bundle = torch.load(path, map_location="cpu")
        else:
            raise
    return bundle


def load_model_any(path: str, trust: bool = False):
    """
    joblib(sklearn) と torch.save(gpytorch) の両方を読む。
    Returns:
      (backend, model_or_bundle, scaler, feature_columns)
    """
    # まず joblib を試す（sklearn互換）
    try:
        bundle = load(path)
        if isinstance(bundle, dict) and bundle.get("backend") == "gpytorch":
            # joblibにgpytorch dictが入っていた場合（レア）
            sc = _deserialize_scaler(bundle.get("scaler", None))
            return ("gpytorch", bundle, sc, bundle.get("feature_columns", None))
        if isinstance(bundle, dict) and "model" in bundle:
            sc = _deserialize_scaler(bundle.get("scaler", None))
            return ("sklearn", bundle["model"], sc, bundle.get("feature_columns", None))
        # モデル単体が保存されていた場合への後方互換
        return ("sklearn", bundle, None, None)
    except Exception:
        pass

    # torch.save を試す
    bundle = _load_torch_bundle(path, trust=trust)
    if isinstance(bundle, dict) and bundle.get("backend") == "gpytorch":
        sc = _deserialize_scaler(bundle.get("scaler", None))
        return ("gpytorch", bundle, sc, bundle.get("feature_columns", None))
    raise RuntimeError(f"未知のモデル形式です: {path}")


class _ScaledPredictor:
    """
    compute_metric_values は X をそのまま有限差分して model.predict(X) を呼ぶので、
    X は「元の特徴量空間」で渡し、predict 内側で scaler.transform を適用する。
    """
    def __init__(self, model, scaler=None):
        self.model = model
        self.scaler = scaler

    def predict(self, X, return_std=True):
        X_in = np.asarray(X, dtype=float)
        if self.scaler is not None:
            X_in = self.scaler.transform(X_in)
        return self.model.predict(X_in, return_std=return_std)


def build_gpytorch_wrapper_from_bundle(bundle: dict, device_str: str = "auto"):
    """
    torch.save した dict から gpytorch SVGP を復元し、predict API を提供するラッパを返す。
    """
    torch, gpytorch = _try_import_torch_gpytorch()
    if torch is None or gpytorch is None:
        raise RuntimeError("torch/gpytorch が import できません。gpytorchモデルを読むにはインストールが必要です。")

    device = _select_torch_device(device_str)
    if device is None:
        device = torch.device("cpu")

    inducing = bundle.get("inducing_points", None)
    if inducing is None:
        raise RuntimeError("gpytorch bundle に inducing_points がありません。")
    inducing = inducing.to(device)

    feature_columns = bundle.get("feature_columns", None)
    n_features = len(feature_columns) if feature_columns else int(inducing.shape[1])

    ard = n_features if bool(bundle.get("anisotropic", False)) else None
    kernel = bundle.get("kernel", "rbf")
    use_linear = (kernel == "rbf_linear")
    matern_nu = float(bundle.get("matern_nu", 1.5) or 1.5)

    class _SVGPModel(gpytorch.models.ApproximateGP):
        def __init__(
            self,
            inducing_points,
            kernel_type="rbf",
            ard_num_dims=None,
            matern_nu=1.5,
            use_linear=False,
        ):
            variational_distribution = gpytorch.variational.CholeskyVariationalDistribution(inducing_points.size(0))
            variational_strategy = gpytorch.variational.VariationalStrategy(
                self,
                inducing_points,
                variational_distribution,
                learn_inducing_locations=True,
            )
            super().__init__(variational_strategy)

            self.mean_module = gpytorch.means.ConstantMean()
            if (kernel_type or "rbf") == "matern":
                base_kernel = gpytorch.kernels.MaternKernel(nu=float(matern_nu), ard_num_dims=ard_num_dims)
            else:
                # rbf / rbf_linear などは rbf として扱う
                base_kernel = gpytorch.kernels.RBFKernel(ard_num_dims=ard_num_dims)
            covar = gpytorch.kernels.ScaleKernel(base_kernel)
            if use_linear:
                covar = covar + gpytorch.kernels.LinearKernel()
            self.covar_module = covar

        def forward(self, x):
            mean_x = self.mean_module(x)
            covar_x = self.covar_module(x)
            return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

    class GPyTorchSparseGPR:
        """
        GPyTorch SVGP を sklearn の API に寄せた薄いラッパ。
        - predict(X, return_std=True) -> (mean, std) / mean
        """
        def __init__(
            self,
            model,
            likelihood,
            device,
            y_mean=0.0,
            y_std=1.0,
            pred_batch_size=8192,
        ):
            self.model = model
            self.likelihood = likelihood
            self.device = device
            self.y_mean = float(y_mean)
            self.y_std = float(y_std) if float(y_std) != 0.0 else 1.0
            self.pred_batch_size = int(pred_batch_size) if pred_batch_size else 8192

        def _to_numpy(self, t):
            return t.detach().cpu().numpy()

        def predict(self, X, return_std=True):
            if X is None:
                raise ValueError("X is None")
            self.model.eval()
            self.likelihood.eval()

            X_np = np.asarray(X, dtype=np.float32)
            n = int(X_np.shape[0])
            bs = max(1, int(self.pred_batch_size))

            means = []
            stds = []
            with torch.no_grad(), gpytorch.settings.fast_pred_var():
                for i in tqdm(range(0, n, bs), desc="GPyTorch predict", leave=False, disable=(n <= bs)):
                    xb = torch.from_numpy(X_np[i:i+bs]).to(self.device)
                    pred = self.likelihood(self.model(xb))
                    m = pred.mean
                    # 正規化yを元スケールへ
                    m = m * self.y_std + self.y_mean
                    means.append(self._to_numpy(m))
                    if return_std:
                        v = pred.variance.clamp_min(0.0)
                        s = torch.sqrt(v) * self.y_std
                        stds.append(self._to_numpy(s))

            y_mean = np.concatenate(means, axis=0)
            if return_std:
                y_std = np.concatenate(stds, axis=0) if stds else None
                return y_mean, y_std
            return y_mean

    model = _SVGPModel(
        inducing_points=inducing,
        kernel_type=kernel,
        ard_num_dims=ard,
        matern_nu=matern_nu,
        use_linear=use_linear,
    ).to(device)
    likelihood = gpytorch.likelihoods.GaussianLikelihood().to(device)
    model.load_state_dict(bundle["model_state_dict"])
    likelihood.load_state_dict(bundle["likelihood_state_dict"])

    wrapper = GPyTorchSparseGPR(
        model=model,
        likelihood=likelihood,
        device=device,
        y_mean=float(bundle.get("y_mean", 0.0)),
        y_std=float(bundle.get("y_std", 1.0)),
        pred_batch_size=int(bundle.get("pred_batch_size", 8192) or 8192),
    )
    return wrapper


def main():
    p = argparse.ArgumentParser(
        description='GPR effects analysis (integrated metrics).',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "例:\n"
            "  # 単一メトリクス（従来）\n"
            "  gpr_effects_analysis.py data.csv --load-model model.joblib \\\n"
            "    --integrate-over prop_spacing_x:auto,auto:200 --integrate-over prop_spacing_y:auto,auto:200 \\\n"
            "    --viz-range distance:auto,auto:150 \\\n"
            "    --normalize-ref distance=1.0 --normalize-as-change-rate \\\n"
            "    --metric moment_abs --output-eval out.png --output-csv out.csv\n"
            "\n"
            "  # moment_abs と grad_abs を同時に出力 + 合成(logsum)（小さいほど良い前提で改善率[%]を表示）\n"
            "  gpr_effects_analysis.py data.csv --load-model model.joblib \\\n"
            "    --integrate-over prop_spacing_x:auto,auto:200 --integrate-over prop_spacing_y:auto,auto:200 \\\n"
            "    --viz-range distance:auto,auto:150 \\\n"
            "    --normalize-ref distance=1.0 --normalize-as-change-rate \\\n"
            "    --metrics moment_abs,grad_abs --combine logsum --weights moment_abs=1,grad_abs=1 \\\n"
            "    --output-eval out.png --output-csv out.csv\n"
        ),
    )
    p.add_argument('csv_file', help='参照CSV（範囲推定に使用）')
    p.add_argument('--load-model', required=True, help='モデル（joblib保存 or torch.save(gpytorch)）')
    p.add_argument('--device', choices=['auto', 'cpu', 'cuda'], default='auto', help='gpytorchモデルの推論デバイス')
    p.add_argument('--trust-model', type=str, default='f', help="torch.load の安全ロード失敗時のみ使用。信頼できるモデルなら 't'。")
    p.add_argument('--metric', choices=['moment_abs', 'grad_abs'], default='moment_abs', help='単一メトリクス（後方互換）')
    p.add_argument('--metrics', type=str, default=None, help="複数メトリクスを同時評価（カンマ区切り）。例: 'moment_abs,grad_abs'。指定時は --metric より優先。")
    p.add_argument('--grad-dims', type=str, default=None, help='grad_absで偏微分する軸（カンマ区切り）')
    p.add_argument('--grad-norm', choices=['l1', 'l2'], default='l2')
    p.add_argument('--fd-step', action='append', default=None, help="有限差分ステップ 'col:h' を複数指定可")
    p.add_argument('--integrate-over', action='append', required=True, help="積分軸と範囲 'col:min,max[:N]' を複数指定可（min/maxに 'auto' 可）")
    p.add_argument('--fix', action='append', default=None, help="固定値 'col=value' 複数可")
    p.add_argument('--overlay-raw', action='store_true', help='2Dヒートマップに raw data 点（--fixで絞り込み）を重ね描きする')
    p.add_argument('--overlay-raw-all', action='store_true', help='2Dヒートマップに raw data 点（--fix無視で全点）を重ね描きする')
    p.add_argument(
        '--mask-by-convex-hull',
        nargs='?',
        const='all',
        default=None,
        choices=['all', 'fix'],
        help="2Dヒートマップをrawデータ点の凸包でマスクする。'all'は全点、'fix'は--fix適用後の点。凸包外はNaN。",
    )
    p.add_argument('--fix-tol', type=float, default=1e-3, help='--fix の一致判定許容誤差（abs(x - value) <= tol）')
    p.add_argument('--viz-range', action='append', default=None, help="可視化軸の範囲 'col:min,max[:N]' 複数可（min/maxに 'auto' 可）")
    p.add_argument('--viz-points', type=int, default=100, help='可視化軸デフォルト分解能')
    p.add_argument('--output-eval', type=str, default=None)
    p.add_argument('--output-csv', type=str, default=None)
    p.add_argument('--cumulative-over', type=str, default=None, help="累積積分の軸と分割数 'col:splits'（同軸の [min, partial] を順次評価）")
    p.add_argument('--normalize-ref', type=str, default=None, help="正規化基準点 'col1=val1,col2=val2'（その点の値で全体を割る）")
    p.add_argument('--normalize-ref-per-step', action='store_true', help="cumulative モードで各ステップごとに独立に正規化する（--normalize-ref と併用）")
    p.add_argument('--normalize-as-change-rate', action='store_true', help="正規化を増減率で表示（基準点=0, +0.5=50%%増, -0.5=50%%減）")
    p.add_argument('--combine', choices=['none', 'logsum'], default='none', help="--metrics 使用時の合成方法。logsum: w_m*log(r_m)+w_g*log(r_g)（重み付き幾何平均）")
    p.add_argument('--weights', type=str, default=None, help="--combine logsum の重み。例: 'moment_abs=1,grad_abs=1'（moment/grad/m/g も可）")
    p.add_argument('--heatmap-range', type=str, default=None, help="ヒートマップの値範囲 'min,max'（normalize後の単位で指定）")
    p.add_argument('--colormap', type=str, default='viridis', help="カラーマップ名（例: viridis, magma, plasma, inferno, cividis）")
    p.add_argument('--transparent', type=str, default='f', help="画像出力(--output-eval)を透過背景で保存する場合は 't'。")
    p.add_argument('--no-title', action='store_true', help='プロットのタイトルを表示しない')
    p.add_argument('--no-colorbar', action='store_true', help='2Dプロットのカラーバーを表示しない')
    p.add_argument('--xlabel', type=str, default=None, help='X軸ラベルを上書き（未指定なら列名）')
    p.add_argument('--ylabel', type=str, default=None, help='Y軸ラベルを上書き（未指定なら列名）')
    p.add_argument('--colorbar-label', type=str, default=None, help='2Dカラーバー表記を上書き（未指定なら自動）')
    p.add_argument('--verbose', action='store_true', help='詳細ログを出力')

    # near-optimal set / multiple optima extraction (combined 2D)
    p.add_argument('--optima', action='store_true', help='combined(2D)の準最適集合(top-pct)から複数最適条件を抽出し、図に重ね描きする')
    # argparse の help は '%' を内部でフォーマットするので '%%' にエスケープが必要
    p.add_argument('--optima-top-pct', type=float, default=5.0, help='準最適集合の上位割合[%%]（default: 5.0）')
    p.add_argument('--optima-delta', type=float, default=None, help='maxからの許容幅[%%]（指定時は --optima-top-pct より優先）')
    p.add_argument('--optima-connectivity', type=int, choices=[4, 8], default=8, help='島の連結判定（4 or 8）')
    p.add_argument('--optima-max-islands', type=int, default=10, help='出力する島（最適条件）の最大数')
    p.add_argument('--output-optima-csv', type=str, default=None, help='抽出した複数最適条件のCSV出力先（combined 2D時）')

    args = p.parse_args()

    def vlog(*a, **k):
        if args.verbose:
            print(*a, **k)

    def maybe_title(s: str) -> str:
        return "" if bool(args.no_title) else (s or "")

    transparent = str(getattr(args, "transparent", "f") or "f").strip().lower() in {"t", "true", "1", "yes", "y"}

    # backend
    if (not os.environ.get('DISPLAY')) or args.output_eval:
        try:
            matplotlib.use('Agg', force=True)
        except Exception:
            pass

    # load model bundle
    vlog('[load] model bundle:', args.load_model)
    trust = str(args.trust_model or "f").strip().lower() in {"t", "true", "1", "yes", "y"}
    backend, loaded_obj, scaler, feature_names = load_model_any(args.load_model, trust=trust)
    vlog('[load] backend:', backend)
    if backend == "gpytorch":
        if not isinstance(loaded_obj, dict):
            raise RuntimeError("gpytorch モデルの形式が不正です（dictを期待）。")
        model = build_gpytorch_wrapper_from_bundle(loaded_obj, device_str=args.device)
        if not feature_names:
            feature_names = loaded_obj.get("feature_columns", None)
    else:
        model = loaded_obj
    if not feature_names:
        raise RuntimeError('feature_columns がモデルに含まれていません。')
    predictor = _ScaledPredictor(model=model, scaler=scaler)

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

    def select_raw_points_2d(x_col: str, y_col: str, *, ignore_fix: bool = False) -> Optional[np.ndarray]:
        """
        raw data を 2D散布点 (N,2) として返す。
        - ignore_fix=False の場合: --fix 条件で絞り込み
        - ignore_fix=True  の場合: --fix を無視して全点
        """
        if x_col not in df.columns or y_col not in df.columns:
            return None
        dff = df
        if not bool(ignore_fix):
            tol = float(args.fix_tol) if args.fix_tol is not None else 0.0
            for c, v in fixes.items():
                if c not in dff.columns:
                    continue
                # 数値なら abs 判定、ダメなら文字列一致
                try:
                    s = pd.to_numeric(dff[c], errors="coerce")
                    mask = np.isfinite(s.values) & (np.abs(s.values - float(v)) <= tol)
                except Exception:
                    mask = (dff[c].astype(str) == str(v)).values
                dff = dff.loc[mask]
                if len(dff) == 0:
                    break
        if len(dff) == 0:
            return None
        # 可視化軸の2列だけを返す（NaN除去）
        xs = pd.to_numeric(dff[x_col], errors="coerce")
        ys = pd.to_numeric(dff[y_col], errors="coerce")
        m = np.isfinite(xs.values) & np.isfinite(ys.values)
        pts = np.stack([xs.values[m], ys.values[m]], axis=1) if np.any(m) else None
        if pts is None or pts.shape[0] == 0:
            return None
        return pts.astype(float)

    # parse integrate spec
    integrate_spec = {}
    vlog('[integrate] specs:', args.integrate_over)
    for it in (args.integrate_over or []):
        if ':' not in it:
            continue
        col, span = it.split(':', 1)
        if ',' not in span:
            continue
        lo, hi, pts = parse_span_auto(span, col.strip(), df)
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
            lo, hi, pts = parse_span_auto(span, col.strip(), df)
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

    def compute_and_integrate(integrate_spec_local, metric_name: str):
        vlog('[grid] build with integrate_spec:', integrate_spec_local)
        X, viz_axes, int_axes = build_grid(feature_names, integrate_spec_local, fixes, viz_spec)
        shape = (len(X), len(feature_names))
        vlog('[grid] X shape:', shape)
        vlog('[grid] visualize axes:', {k: len(v) for k, v in viz_axes.items()})
        vlog('[grid] integrate axes:', {k: len(v) for k, v in int_axes.items()})
        # 重要: 勾配(有限差分)は「元の特徴量空間」で取るので、ここではスケーリングしない。
        # predict 内側で scaler.transform を適用する（_ScaledPredictor）。
        vlog('[scale] applied inside predict:', scaler is not None)
        grad_dims = [c.strip() for c in args.grad_dims.split(',')] if args.grad_dims else None
        vlog('[metric] type:', metric_name, 'grad_dims=', grad_dims, 'grad_norm=', args.grad_norm)
        values = compute_metric_values(
            model=predictor,
            X=X,
            metric=metric_name,
            feature_names=feature_names,
            grad_dims=grad_dims,
            grad_norm=args.grad_norm,
            fd_steps=fd_steps,
        )
        vlog('[metric] values: min=', float(np.min(values)), 'max=', float(np.max(values)), 'mean=', float(np.mean(values)))
        Z, _ = integrate_values(values, feature_names, viz_axes, int_axes)
        vlog('[integrate] result shape:', Z.shape)
        return Z, viz_axes, int_axes

    def _base_out_paths(base_path: Optional[str], suffix: str) -> Optional[str]:
        if not base_path:
            return None
        stem, ext = os.path.splitext(base_path)
        return f"{stem}_{suffix}{ext}"

    def _optima_csv_path(base_path: Optional[str], suffix: str) -> Optional[str]:
        if not base_path:
            return None
        stem, ext = os.path.splitext(base_path)
        if not ext:
            ext = ".csv"
        return f"{stem}_{suffix}{ext}"

    def _grid_dataframe(axes_names: Sequence[str], axes_vals: Sequence[np.ndarray]) -> pd.DataFrame:
        if len(axes_names) == 0:
            return pd.DataFrame({})
        if len(axes_names) == 1:
            return pd.DataFrame({axes_names[0]: axes_vals[0]})
        if len(axes_names) == 2:
            Xg, Yg = np.meshgrid(axes_vals[0], axes_vals[1])
            return pd.DataFrame({axes_names[0]: Xg.reshape(-1), axes_names[1]: Yg.reshape(-1)})
        grid_axes = np.meshgrid(*axes_vals, indexing='xy')
        data = {axes_names[i]: grid_axes[i].reshape(-1) for i in range(len(axes_names))}
        return pd.DataFrame(data)

    def _append_value_column(df_base: pd.DataFrame, col_name: str, grid: np.ndarray) -> pd.DataFrame:
        df = df_base.copy()
        df[col_name] = np.asarray(grid).reshape(-1)
        return df

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
        metrics = _parse_metrics_list(args.metrics, fallback=args.metric)
        is_multi = (args.metrics is not None) and (len(metrics) >= 2)
        weights = _parse_weights(args.weights)
        w_m, w_g = float(weights.get("moment_abs", 1.0)), float(weights.get("grad_abs", 1.0))
        w_sum = (w_m + w_g) if (w_m + w_g) != 0.0 else 1.0

        Z_map: Dict[str, np.ndarray] = {}
        viz_axes = None
        int_axes = None
        for m in metrics:
            Zm, viz_axes_m, int_axes_m = compute_and_integrate(integrate_spec, m)
            Z_map[m] = Zm
            viz_axes = viz_axes_m if viz_axes is None else viz_axes
            int_axes = int_axes_m if int_axes is None else int_axes

        assert viz_axes is not None and int_axes is not None
        axes_names = list(viz_axes.keys())
        axes_vals = [viz_axes[k] for k in axes_names]
        integrate_desc_parts = [f"{col} [{arr[0]:.3g}, {arr[-1]:.3g}], N={len(arr)}" for col, arr in int_axes.items()]
        integrate_desc = "; ".join(integrate_desc_parts) if integrate_desc_parts else "(none)"

        if is_multi and (args.normalize_ref is None) and (args.normalize_as_change_rate or args.combine == "logsum"):
            raise ValueError("--metrics で増減率/合成(logsum)を扱う場合は --normalize-ref が必須です（比率が必要）。")

        # helper: build ratio & display grid for each metric
        def build_display(metric_name: str):
            Z = Z_map[metric_name]
            title_base = f"metric={metric_name} | integrate over: {integrate_desc}"
            if len(axes_names) == 0:
                return float(Z), title_base, None, None, None, None, None
            if len(axes_names) == 1:
                Z_1d = Z.reshape(-1)
                ylabel = "integral"
                ratio = None
                ref_z = None
                actual_ref = None
                title = title_base
                if args.normalize_ref:
                    ref_spec = parse_normalize_ref(args.normalize_ref)
                    ratio, ref_z, actual_ref = normalize_by_ref_1d(Z_1d, axes_vals[0], axes_names[0], ref_spec)
                    ref_desc = ", ".join(f"{k}={v:.4g}" for k, v in actual_ref.items())
                    if args.normalize_as_change_rate:
                        disp = (ratio - 1.0) * 100.0
                        ylabel = "change rate [%]"
                        title = f"{title_base}\n(change rate [%] from ref: {ref_desc}, value={ref_z:.4g})"
                    else:
                        disp = ratio
                        ylabel = "ratio"
                        title = f"{title_base}\n(normalized by ref: {ref_desc}, value={ref_z:.4g})"
                else:
                    disp = Z_1d
                return disp, title, ylabel, ratio, ref_z, actual_ref, None
            if len(axes_names) == 2:
                Xv, Yv = axes_vals[0], axes_vals[1]
                Zm0 = align_Z_to_axes(Z, axes_names, viz_axes)
                Z_plot0 = Zm0.T  # (Y, X)
                ylabel = None
                ratio = None
                ref_z = None
                actual_ref = None
                title = title_base
                cbl = "integral"
                if args.normalize_ref:
                    ref_spec = parse_normalize_ref(args.normalize_ref)
                    ratio, ref_z, actual_ref = normalize_by_ref_2d(Z_plot0, Xv, Yv, axes_names[0], axes_names[1], ref_spec)
                    ref_desc = ", ".join(f"{k}={v:.4g}" for k, v in actual_ref.items())
                    if args.normalize_as_change_rate:
                        disp = (ratio - 1.0) * 100.0
                        cbl = "change rate [%]"
                        title = f"{title_base}\n(change rate [%] from ref: {ref_desc}, value={ref_z:.4g})"
                    else:
                        disp = ratio
                        cbl = "ratio"
                        title = f"{title_base}\n(normalized by ref: {ref_desc}, value={ref_z:.4g})"
                else:
                    disp = Z_plot0
                return disp, title, cbl, ratio, ref_z, actual_ref, (Xv, Yv)
            # >=3D
            return Z, title_base, None, None, None, None, None

        # output selection
        overlay = None
        if len(axes_names) == 2:
            if args.overlay_raw_all:
                overlay = select_raw_points_2d(axes_names[0], axes_names[1], ignore_fix=True)
            elif args.overlay_raw:
                overlay = select_raw_points_2d(axes_names[0], axes_names[1], ignore_fix=False)

        # convex-hull mask (2D only): outside hull -> NaN
        hull_inside_mask = None
        hull_desc = None
        if len(axes_names) == 2 and args.mask_by_convex_hull:
            src = str(args.mask_by_convex_hull).strip().lower()
            ignore_fix = (src == "all")
            pts_hull = select_raw_points_2d(axes_names[0], axes_names[1], ignore_fix=ignore_fix)
            hull = _convex_hull_2d(pts_hull) if pts_hull is not None else None

            # debug: always show hull inputs when verbose
            if args.verbose:
                n_pts = 0 if pts_hull is None else int(np.asarray(pts_hull).shape[0])
                print("[hull] axes:", axes_names[0], axes_names[1], "| src=", src, "| ignore_fix=", bool(ignore_fix))
                print("[hull] pts_hull bbox:", _bbox_str_xy(pts_hull))
                print("[hull] pts_hull n:", n_pts)
                # raw points unique list (all)
                pts_u = _unique_sorted_points_xy(pts_hull)
                _print_points_block("[hull]", "raw unique points (x,y)", pts_u)

            if hull is not None:
                # build inside-mask for the current grid
                Xv0 = np.asarray(axes_vals[0], dtype=float)
                Yv0 = np.asarray(axes_vals[1], dtype=float)
                _dummy = np.zeros((len(Yv0), len(Xv0)), dtype=float)
                _, hull_inside_mask = _mask_by_polygon_2d(_dummy, Xv0, Yv0, hull)
                hull_desc = f"convex_hull(src={src}, n_pts={int(pts_hull.shape[0])})"

                # debug: sanity checks (only when verbose)
                if args.verbose:
                    try:
                        path = _make_closed_path(np.asarray(hull, dtype=float))
                        inside_pts = path.contains_points(np.asarray(pts_hull, dtype=float)) if path is not None else None
                        if inside_pts is None:
                            raise RuntimeError("failed to build closed Path for hull")
                        inside_pts = np.asarray(inside_pts, dtype=bool)
                        n_in = int(np.sum(inside_pts))
                        n_all = int(len(inside_pts))
                        print("[hull] axes:", axes_names[0], axes_names[1], "| src=", src)
                        print("[hull] pts_hull bbox:", _bbox_str_xy(pts_hull))
                        print("[hull] hull vertices:", int(np.asarray(hull).shape[0]), "bbox:", _bbox_str_xy(hull))
                        # hull vertices list (all)
                        _print_points_block("[hull]", "hull vertices (x,y)", np.asarray(hull, dtype=float))
                        print(
                            "[hull] pts_hull inside hull:",
                            f"{n_in}/{n_all}",
                            f"({(100.0 * n_in / max(1, n_all)):.3g}%)",
                        )
                        grid_bbox = _bbox_str_xy(np.stack([Xv0.reshape(-1), Yv0.reshape(-1)], axis=1))
                        print("[hull] grid centers bbox:", grid_bbox, "grid_shape=", (len(Yv0), len(Xv0)))
                        if hull_inside_mask is not None:
                            n_grid_in = int(np.sum(hull_inside_mask))
                            n_grid = int(hull_inside_mask.size)
                            print(
                                "[hull] grid inside hull:",
                                f"{n_grid_in}/{n_grid}",
                                f"({(100.0 * n_grid_in / max(1, n_grid)):.3g}%)",
                            )
                    except Exception as e:
                        print("[hull] debug failed:", repr(e))
            else:
                if args.verbose:
                    print("[hull] hull=None reason:", _hull_none_reason(pts_hull))

        def apply_hull_nan(arr: Any) -> np.ndarray:
            a = np.asarray(arr, dtype=float)
            if hull_inside_mask is None:
                return a
            out = a.copy()
            out[~hull_inside_mask] = np.nan
            return out

        # heatmap range (shared)
        hm_vmin, hm_vmax = None, None
        if args.heatmap_range:
            parts = args.heatmap_range.split(',')
            if len(parts) == 2:
                hm_vmin = float(parts[0].strip())
                hm_vmax = float(parts[1].strip())

        # build CSV base
        df_base = _grid_dataframe(axes_names, axes_vals)

        # single metric path (keeps behavior, but uses improved ylabel)
        if not is_multi:
            m = metrics[0]
            disp, title, ylab_or_cbl, _, _, _, xy = build_display(m)
            if len(axes_names) == 0:
                print(f"Integrate over: {integrate_desc}")
                print(f"integral value: {float(disp):.6f}")
                if args.output_csv:
                    pd.DataFrame({"value": [float(disp)]}).to_csv(args.output_csv, index=False)
            elif len(axes_names) == 1:
                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab = args.ylabel if args.ylabel is not None else str(ylab_or_cbl)
                plot_1d(xlab, axes_vals[0], np.asarray(disp), maybe_title(title), args.output_eval, ylabel=ylab, transparent=transparent)
                if args.output_csv:
                    dump_csv(axes_names, axes_vals, np.asarray(disp).reshape(Z_map[m].shape), args.output_csv)
            elif len(axes_names) == 2:
                Xv, Yv = xy
                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab = args.ylabel if args.ylabel is not None else axes_names[1]
                cbl = args.colorbar_label if args.colorbar_label is not None else str(ylab_or_cbl)
                disp_plot = apply_hull_nan(disp)
                title_eff = title if not hull_desc else f"{title}\n(mask: {hull_desc})"
                plot_2d(
                    xlab, ylab, Xv, Yv, disp_plot,
                    maybe_title(title_eff), args.output_eval,
                    overlay_points=overlay, vmin=hm_vmin, vmax=hm_vmax, cmap=args.colormap,
                    colorbar_label=cbl,
                    show_colorbar=(not bool(args.no_colorbar)),
                    transparent=transparent,
                )
                if args.output_csv:
                    dump_csv(axes_names, axes_vals, disp_plot, args.output_csv)
            else:
                print(f"可視化軸が3以上のため図は出力しません。CSVで出力します。axes={axes_names}")
                if args.output_csv:
                    dump_csv(axes_names, axes_vals, Z_map[m], args.output_csv)
            return 0

        # multi-metric path
        # Prepare displays & ratios
        displays: Dict[str, Any] = {}
        ratios: Dict[str, Any] = {}
        meta: Dict[str, Any] = {}
        for m in metrics:
            disp, title, ylab_or_cbl, ratio, ref_z, actual_ref, xy = build_display(m)
            displays[m] = (disp, title, ylab_or_cbl, xy)
            ratios[m] = ratio
            meta[m] = {"ref_z": ref_z, "actual_ref": actual_ref}

        # combined
        combined = None
        combined_title = None
        combined_label = None
        if args.combine == "logsum":
            if ("moment_abs" not in ratios) or ("grad_abs" not in ratios):
                raise ValueError("--combine logsum は moment_abs と grad_abs の両方が必要です（--metrics に含めてください）。")
            r_m = ratios["moment_abs"]
            r_g = ratios["grad_abs"]
            if r_m is None or r_g is None:
                raise ValueError("--combine logsum は --normalize-ref が必須です（比率が必要）。")
            score = (w_m * _safe_log_ratio(r_m) + w_g * _safe_log_ratio(r_g)) / w_sum
            # 通常の改善率[%] = (1 - 幾何平均比率) * 100（小さいほど良い前提で、改善ほどプラス）
            improve_pct = _improve_pct_from_log_ratio(score)
            combined = improve_pct
            combined_label = "improve [%] (1 - geom-mean ratio)"
            combined_title = f"combined=logsum (weights: moment_abs={w_m:g}, grad_abs={w_g:g}) | integrate over: {integrate_desc}"

        # plot & csv
        # 0D
        if len(axes_names) == 0:
            print(f"Integrate over: {integrate_desc}")
            for m in metrics:
                print(f"{m}: integral value: {float(Z_map[m]):.6f}")
            if combined is not None:
                print(f"combined(logsum) improve[%]: {float(combined):.6f}")
            if args.output_csv:
                out = {m: [float(Z_map[m])] for m in metrics}
                if combined is not None:
                    out["combined_improve_pct"] = [float(combined)]
                pd.DataFrame(out).to_csv(args.output_csv, index=False)
            return 0

        # 1D
        if len(axes_names) == 1:
            for m in metrics:
                disp, title, ylab, _xy = displays[m]
                out_path = _base_out_paths(args.output_eval, m)
                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab_eff = args.ylabel if args.ylabel is not None else str(ylab)
                plot_1d(xlab, axes_vals[0], np.asarray(disp), maybe_title(title), out_path, ylabel=ylab_eff, transparent=transparent)
            if combined is not None:
                out_path = _base_out_paths(args.output_eval, "combined")
                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab_eff = args.ylabel if args.ylabel is not None else str(combined_label)
                plot_1d(xlab, axes_vals[0], np.asarray(combined), maybe_title(combined_title), out_path, ylabel=ylab_eff, transparent=transparent)
            if args.output_csv:
                df = df_base
                # store change rate if requested else ratio if normalized else raw
                for m in metrics:
                    disp, _title, _ylab, _xy = displays[m]
                    col = f"{m}_{'change_pct' if args.normalize_as_change_rate else ('ratio' if args.normalize_ref else 'value')}"
                    df = _append_value_column(df, col, np.asarray(disp))
                if combined is not None:
                    df = _append_value_column(df, "combined_improve_pct", np.asarray(combined))
                df.to_csv(args.output_csv, index=False)
            return 0

        # 2D
        if len(axes_names) == 2:
            for m in metrics:
                disp, title, cbl, xy = displays[m]
                out_path = _base_out_paths(args.output_eval, m)
                Xv, Yv = xy
                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab = args.ylabel if args.ylabel is not None else axes_names[1]
                cbl_eff = args.colorbar_label if args.colorbar_label is not None else str(cbl)
                disp_plot = apply_hull_nan(disp)
                title_eff = title if not hull_desc else f"{title}\n(mask: {hull_desc})"
                plot_2d(
                    xlab, ylab, Xv, Yv, disp_plot,
                    maybe_title(title_eff), out_path,
                    overlay_points=overlay, vmin=hm_vmin, vmax=hm_vmax, cmap=args.colormap,
                    colorbar_label=cbl_eff,
                    show_colorbar=(not bool(args.no_colorbar)),
                    transparent=transparent,
                )
            if combined is not None:
                # combined: improve[%] は 0 を中心に見たいので vmin/vmax を±対称にする
                out_path = _base_out_paths(args.output_eval, "combined")
                Xv, Yv = axes_vals[0], axes_vals[1]
                if (hm_vmin is not None) and (hm_vmax is not None):
                    # 手動指定がある場合は必ず優先（複数回実行で色の意味を揃えるため）
                    vmin_c, vmax_c = hm_vmin, hm_vmax
                else:
                    c = np.asarray(combined, dtype=float)
                    mabs = float(np.nanmax(np.abs(c))) if np.isfinite(c).any() else 0.0
                    vmin_c = -mabs if mabs > 0.0 else None
                    vmax_c = +mabs if mabs > 0.0 else None
                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab = args.ylabel if args.ylabel is not None else axes_names[1]
                cbl_eff = args.colorbar_label if args.colorbar_label is not None else str(combined_label)

                # optima extraction (near-optimal set) for combined 2D
                opt = None
                if bool(args.optima):
                    opt = _optima_2d_from_combined(
                        apply_hull_nan(combined),
                        np.asarray(Xv, dtype=float),
                        np.asarray(Yv, dtype=float),
                        top_pct=float(args.optima_top_pct),
                        delta=(None if args.optima_delta is None else float(args.optima_delta)),
                        connectivity=int(args.optima_connectivity),
                        max_islands=int(args.optima_max_islands),
                    )
                    islands = opt.get("islands", []) if isinstance(opt, dict) else []
                    if islands:
                        print(f"[optima] combined 2D: method={opt.get('method')} threshold={opt.get('threshold'):.6g} vmax={opt.get('vmax'):.6g} islands={len(islands)}")
                        for i, d in enumerate(islands, start=1):
                            print(
                                f"  #{i}: rep=({float(d['rep_x']):.6g}, {float(d['rep_y']):.6g}) "
                                f"value={float(d['rep_value']):.6g} island_max={float(d['island_max']):.6g} n_cells={int(d['n_cells'])}"
                            )
                        if args.output_optima_csv:
                            df_opt = pd.DataFrame(
                                [
                                    dict(
                                        rank=i,
                                        rep_x=float(d["rep_x"]),
                                        rep_y=float(d["rep_y"]),
                                        rep_value=float(d["rep_value"]),
                                        island_max=float(d["island_max"]),
                                        island_median=float(d["island_median"]),
                                        n_cells=int(d["n_cells"]),
                                        area=float(d["area"]),
                                        center_median_x=float(d["center_median_x"]),
                                        center_median_y=float(d["center_median_y"]),
                                        threshold=float(opt.get("threshold")),
                                        vmax=float(opt.get("vmax")),
                                        method=str(opt.get("method")),
                                    )
                                    for i, d in enumerate(islands, start=1)
                                ]
                            )
                            df_opt.to_csv(args.output_optima_csv, index=False)

                combined_plot = apply_hull_nan(combined)
                title_eff = combined_title if not hull_desc else f"{combined_title}\n(mask: {hull_desc})"
                corner_text = ""
                if opt and isinstance(opt, dict) and opt.get("islands"):
                    corner_text = _format_peak_list_text(
                        opt.get("islands", []),
                        x_label=xlab,
                        y_label=ylab,
                        max_items=int(args.optima_max_islands),
                    )
                # combined は従来 custom_improve 固定だったが、--colormap が明示された場合はそれを優先する
                cmap_combined = args.colormap if str(args.colormap) != "viridis" else "custom_improve"
                plot_2d(
                    xlab, ylab, Xv, Yv, combined_plot,
                    maybe_title(title_eff), out_path,
                    overlay_points=overlay,
                    contour_mask=(None if (not opt or opt.get("mask") is None) else opt.get("mask")),
                    mark_points=(None if (not opt or opt.get("rep_points") is None) else opt.get("rep_points")),
                    mark_labels=(None if (not opt) else opt.get("labels")),
                    corner_text=corner_text,
                    vmin=vmin_c, vmax=vmax_c, cmap=cmap_combined,
                    colorbar_label=cbl_eff,
                    show_colorbar=(not bool(args.no_colorbar)),
                    transparent=transparent,
                )
            if args.output_csv:
                df = df_base
                for m in metrics:
                    disp, _title, _cbl, _xy = displays[m]
                    disp_out = apply_hull_nan(disp)
                    col = f"{m}_{'change_pct' if args.normalize_as_change_rate else ('ratio' if args.normalize_ref else 'value')}"
                    df = _append_value_column(df, col, disp_out)
                if combined is not None:
                    df = _append_value_column(df, "combined_improve_pct", apply_hull_nan(combined))
                df.to_csv(args.output_csv, index=False)
            return 0

        # >=3D
        print(f"可視化軸が3以上のため図は出力しません。CSVで出力します。axes={axes_names}")
        if args.output_csv:
            df = df_base
            for m in metrics:
                df = _append_value_column(df, f"{m}_value", np.asarray(Z_map[m]))
            df.to_csv(args.output_csv, index=False)
    else:
        cum_col, splits = cumulative
        lo, hi, pts = integrate_spec[cum_col]
        if pts <= 0:
            pts = 200
        uppers = [lo + (hi - lo) * (k / splits) for k in range(1, splits + 1)]
        vlog('[cumulative] axis:', cum_col, 'range=', (lo, hi), 'splits=', splits, 'uppers=', uppers)

        metrics = _parse_metrics_list(args.metrics, fallback=args.metric)
        is_multi = (args.metrics is not None) and (len(metrics) >= 2)
        weights = _parse_weights(args.weights)
        w_m, w_g = float(weights.get("moment_abs", 1.0)), float(weights.get("grad_abs", 1.0))
        w_sum = (w_m + w_g) if (w_m + w_g) != 0.0 else 1.0

        if is_multi and (args.normalize_ref is None) and (args.normalize_as_change_rate or args.combine == "logsum"):
            raise ValueError("--metrics で増減率/合成(logsum)を扱う場合は --normalize-ref が必須です（比率が必要）。")

        results = []  # [(upper, {metric: Zk}, viz_axes_k, int_axes_k)]
        for k, up in enumerate(tqdm(uppers, desc="Cumulative integration", leave=True), start=1):
            spec_k = dict(integrate_spec)
            pts_k = max(2, int(np.ceil(pts * (k / splits))))
            spec_k[cum_col] = (lo, up, pts_k)
            vlog('[cumulative] step', k, 'upper=', up, 'points=', pts_k)
            Zk_map: Dict[str, np.ndarray] = {}
            viz_axes_k = None
            int_axes_k = None
            for m in metrics:
                Zk, viz_axes_m, int_axes_m = compute_and_integrate(spec_k, m)
                Zk_map[m] = Zk
                viz_axes_k = viz_axes_m if viz_axes_k is None else viz_axes_k
                int_axes_k = int_axes_m if int_axes_k is None else int_axes_k
            assert viz_axes_k is not None and int_axes_k is not None
            results.append((up, Zk_map, viz_axes_k, int_axes_k))

        axes_names = list(results[0][2].keys())
        axes_vals = [results[0][2][k] for k in axes_names]
        integrate_desc = f"{cum_col} cumulative [{lo:.3g} -> upper], splits={splits}"

        if len(axes_names) == 0:
            print(f"Cumulative integrate over: {integrate_desc}")
            for up, Zk_map, _, _ in results:
                for m in metrics:
                    print(f"  upper={up:.6g}: {m} value={float(Zk_map[m]):.6f}")
            if args.output_csv:
                df_out = pd.DataFrame({'upper': [up for up, _, _, _ in results]})
                for m in metrics:
                    df_out[m] = [float(Zk_map[m]) for _, Zk_map, _, _ in results]
                if is_multi and args.combine == "logsum":
                    # combined improve[%] from ratios (requires normalize_ref)
                    pass  # handled in 1D/2D where ratio exists
                df_out.to_csv(args.output_csv, index=False)
        elif len(axes_names) == 1:
            n = len(results)
            per_step_normalize = args.normalize_ref_per_step

            def compute_common_ref(metric_name: str):
                if not args.normalize_ref or per_step_normalize:
                    return None, None
                ref_spec = parse_normalize_ref(args.normalize_ref)
                # 最終結果から基準値（比率の分母）を取る
                _, ref_z_common, actual_ref_common = normalize_by_ref_1d(
                    results[-1][1][metric_name].reshape(-1), axes_vals[0], axes_names[0], ref_spec
                )
                return ref_z_common, actual_ref_common

            # build per-metric figures
            for metric_name in metrics:
                ref_z_common, actual_ref_common = compute_common_ref(metric_name)
                plt.figure(figsize=(10, 6))
                plt.rcParams.update({'font.size': 16})
                for i, (up, Zk_map, _, _) in enumerate(results):
                    Z_1d = Zk_map[metric_name].reshape(-1)
                    if ref_z_common is not None:
                        ratio = Z_1d / ref_z_common
                    elif args.normalize_ref and per_step_normalize:
                        ref_spec = parse_normalize_ref(args.normalize_ref)
                        ratio, ref_z_step, _actual_ref_step = normalize_by_ref_1d(
                            Z_1d, axes_vals[0], axes_names[0], ref_spec
                        )
                    else:
                        ratio = None
                    if ratio is None:
                        disp = Z_1d
                        ylabel = "integral"
                    else:
                        if args.normalize_as_change_rate:
                            disp = (ratio - 1.0) * 100.0
                            ylabel = "change rate [%]"
                        else:
                            disp = ratio
                            ylabel = "ratio"

                    t = 1.0 if i == n - 1 else (i / max(1, n - 1))
                    color = plt.cm.magma(1.0 - t)
                    lw = 3.5 if i == n - 1 else (1.5 + 1.0 * t)
                    alpha = 1.0 if i == n - 1 else 0.6
                    z = 5 if i == n - 1 else (1 + i)
                    plt.plot(
                        axes_vals[0], disp,
                        color=color, lw=lw, alpha=alpha, zorder=z,
                        label=f"{cum_col} upper={up:.3g}",
                    )

                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab = args.ylabel if args.ylabel is not None else ylabel
                plt.xlabel(xlab)
                plt.ylabel(ylab)
                title_1d = f"metric={metric_name} | {integrate_desc}"
                if actual_ref_common is not None:
                    ref_desc = ", ".join(f"{k}={v:.4g}" for k, v in actual_ref_common.items())
                    title_1d = f"{title_1d}\n(common ref: {ref_desc}, value={ref_z_common:.4g})"
                if not bool(args.no_title):
                    plt.title(title_1d)
                plt.grid(True, alpha=0.3)
                plt.legend()
                plt.tight_layout()
                out_path = _base_out_paths(args.output_eval, metric_name) if args.output_eval else None
                if out_path:
                    plt.savefig(out_path, dpi=300, bbox_inches='tight', transparent=bool(transparent))
                else:
                    plt.show()
                plt.close()

            # combined (logsum) 1D cumulative: produce a final-figure per step overlay
            if is_multi and args.combine == "logsum":
                # need ratios for both metrics
                ref_m, _ = compute_common_ref("moment_abs")
                ref_g, _ = compute_common_ref("grad_abs")
                plt.figure(figsize=(10, 6))
                plt.rcParams.update({'font.size': 16})
                for i, (up, Zk_map, _, _) in enumerate(results):
                    Zm = Zk_map["moment_abs"].reshape(-1)
                    Zg = Zk_map["grad_abs"].reshape(-1)
                    if (ref_m is not None) and (ref_g is not None):
                        r_m = Zm / ref_m
                        r_g = Zg / ref_g
                    else:
                        ref_spec = parse_normalize_ref(args.normalize_ref)
                        r_m, _, _ = normalize_by_ref_1d(Zm, axes_vals[0], axes_names[0], ref_spec)
                        r_g, _, _ = normalize_by_ref_1d(Zg, axes_vals[0], axes_names[0], ref_spec)
                    score = (w_m * _safe_log_ratio(r_m) + w_g * _safe_log_ratio(r_g)) / w_sum
                    improve_pct = _improve_pct_from_log_ratio(score)
                    t = 1.0 if i == n - 1 else (i / max(1, n - 1))
                    color = plt.cm.magma(1.0 - t)
                    lw = 3.5 if i == n - 1 else (1.5 + 1.0 * t)
                    alpha = 1.0 if i == n - 1 else 0.6
                    z = 5 if i == n - 1 else (1 + i)
                    plt.plot(
                        axes_vals[0], improve_pct,
                        color=color, lw=lw, alpha=alpha, zorder=z,
                        label=f"{cum_col} upper={up:.3g}",
                    )
                xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                ylab = args.ylabel if args.ylabel is not None else "improve [%] (1 - geom-mean ratio)"
                plt.xlabel(xlab)
                plt.ylabel(ylab)
                title_1d = f"combined=logsum (moment_abs={w_m:g}, grad_abs={w_g:g}) | {integrate_desc}"
                if not bool(args.no_title):
                    plt.title(title_1d)
                plt.grid(True, alpha=0.3)
                plt.legend()
                plt.tight_layout()
                out_path = _base_out_paths(args.output_eval, "combined") if args.output_eval else None
                if out_path:
                    plt.savefig(out_path, dpi=300, bbox_inches='tight', transparent=bool(transparent))
                else:
                    plt.show()
                plt.close()

            if args.output_csv:
                df_out = pd.DataFrame({axes_names[0]: axes_vals[0]})
                for metric_name in metrics:
                    # store each upper as separate column like previous behavior, but per metric
                    ref_z_common, _ = compute_common_ref(metric_name)
                    for up, Zk_map, _, _ in results:
                        Z_1d = Zk_map[metric_name].reshape(-1)
                        if ref_z_common is not None:
                            ratio = Z_1d / ref_z_common
                        elif args.normalize_ref and per_step_normalize:
                            ref_spec = parse_normalize_ref(args.normalize_ref)
                            ratio, _, _ = normalize_by_ref_1d(Z_1d, axes_vals[0], axes_names[0], ref_spec)
                        else:
                            ratio = None
                        if ratio is None:
                            disp = Z_1d
                            col_suffix = "value"
                        else:
                            if args.normalize_as_change_rate:
                                disp = (ratio - 1.0) * 100.0
                                col_suffix = "change_pct"
                            else:
                                disp = ratio
                                col_suffix = "ratio"
                        df_out[f"{metric_name}_{col_suffix}_upper_{up:.6g}"] = disp

                if is_multi and args.combine == "logsum":
                    # combined columns per upper
                    ref_m, _ = compute_common_ref("moment_abs")
                    ref_g, _ = compute_common_ref("grad_abs")
                    for up, Zk_map, _, _ in results:
                        Zm = Zk_map["moment_abs"].reshape(-1)
                        Zg = Zk_map["grad_abs"].reshape(-1)
                        if (ref_m is not None) and (ref_g is not None):
                            r_m = Zm / ref_m
                            r_g = Zg / ref_g
                        else:
                            ref_spec = parse_normalize_ref(args.normalize_ref)
                            r_m, _, _ = normalize_by_ref_1d(Zm, axes_vals[0], axes_names[0], ref_spec)
                            r_g, _, _ = normalize_by_ref_1d(Zg, axes_vals[0], axes_names[0], ref_spec)
                        score = (w_m * _safe_log_ratio(r_m) + w_g * _safe_log_ratio(r_g)) / w_sum
                        df_out[f"combined_improve_pct_upper_{up:.6g}"] = _improve_pct_from_log_ratio(score)
                df_out.to_csv(args.output_csv, index=False)
        elif len(axes_names) == 2:
            base = args.output_eval or 'effects_cumulative.png'
            stem, ext = os.path.splitext(base)
            Xv, Yv = axes_vals[0], axes_vals[1]
            per_step_normalize = args.normalize_ref_per_step

            def compute_common_ref_2d(metric_name: str):
                if not args.normalize_ref or per_step_normalize:
                    return None, None
                ref_spec = parse_normalize_ref(args.normalize_ref)
                Z_last = results[-1][1][metric_name]
                Zm_last = align_Z_to_axes(Z_last, axes_names, results[0][2])
                Z_plot_last = Zm_last.T
                _, ref_z_common, actual_ref_common = normalize_by_ref_2d(
                    Z_plot_last, Xv, Yv, axes_names[0], axes_names[1], ref_spec
                )
                return ref_z_common, actual_ref_common

            overlay = None
            if args.overlay_raw_all:
                overlay = select_raw_points_2d(axes_names[0], axes_names[1], ignore_fix=True)
            elif args.overlay_raw:
                overlay = select_raw_points_2d(axes_names[0], axes_names[1], ignore_fix=False)

            hm_vmin, hm_vmax = None, None
            if args.heatmap_range:
                parts = args.heatmap_range.split(',')
                if len(parts) == 2:
                    hm_vmin = float(parts[0].strip()); hm_vmax = float(parts[1].strip())

            # convex-hull mask (2D cumulative): outside hull -> NaN
            hull_inside_mask = None
            hull_desc = None
            if args.mask_by_convex_hull:
                src = str(args.mask_by_convex_hull).strip().lower()
                ignore_fix = (src == "all")
                pts_hull = select_raw_points_2d(axes_names[0], axes_names[1], ignore_fix=ignore_fix)
                hull = _convex_hull_2d(pts_hull) if pts_hull is not None else None

                # debug: always show hull inputs when verbose (cumulative branch)
                if args.verbose:
                    n_pts = 0 if pts_hull is None else int(np.asarray(pts_hull).shape[0])
                    print("[hull] (cumulative) axes:", axes_names[0], axes_names[1], "| src=", src, "| ignore_fix=", bool(ignore_fix))
                    print("[hull] (cumulative) pts_hull bbox:", _bbox_str_xy(pts_hull))
                    print("[hull] (cumulative) pts_hull n:", n_pts)
                    # raw points unique list (all)
                    pts_u = _unique_sorted_points_xy(pts_hull)
                    _print_points_block("[hull] (cumulative)", "raw unique points (x,y)", pts_u)

                if hull is not None:
                    _dummy = np.zeros((len(Yv), len(Xv)), dtype=float)
                    _, hull_inside_mask = _mask_by_polygon_2d(_dummy, Xv, Yv, hull)
                    hull_desc = f"convex_hull(src={src}, n_pts={int(pts_hull.shape[0])})"
                    if args.verbose:
                        try:
                            path = _make_closed_path(np.asarray(hull, dtype=float))
                            inside_pts = path.contains_points(np.asarray(pts_hull, dtype=float)) if path is not None else None
                            if inside_pts is None:
                                raise RuntimeError("failed to build closed Path for hull")
                            inside_pts = np.asarray(inside_pts, dtype=bool)
                            n_in = int(np.sum(inside_pts))
                            n_all = int(len(inside_pts))
                            print("[hull] (cumulative) hull vertices:", int(np.asarray(hull).shape[0]), "bbox:", _bbox_str_xy(hull))
                            # hull vertices list (all)
                            _print_points_block("[hull] (cumulative)", "hull vertices (x,y)", np.asarray(hull, dtype=float))
                            print(
                                "[hull] (cumulative) pts_hull inside hull:",
                                f"{n_in}/{n_all}",
                                f"({(100.0 * n_in / max(1, n_all)):.3g}%)",
                            )
                            if hull_inside_mask is not None:
                                n_grid_in = int(np.sum(hull_inside_mask))
                                n_grid = int(hull_inside_mask.size)
                                print(
                                    "[hull] (cumulative) grid inside hull:",
                                    f"{n_grid_in}/{n_grid}",
                                    f"({(100.0 * n_grid_in / max(1, n_grid)):.3g}%)",
                                )
                        except Exception as e:
                            print("[hull] (cumulative) debug failed:", repr(e))
                else:
                    if args.verbose:
                        print("[hull] (cumulative) hull=None reason:", _hull_none_reason(pts_hull))

            def apply_hull_nan(arr: Any) -> np.ndarray:
                a = np.asarray(arr, dtype=float)
                if hull_inside_mask is None:
                    return a
                out = a.copy()
                out[~hull_inside_mask] = np.nan
                return out

            for metric_name in metrics:
                ref_z_common, actual_ref_common = compute_common_ref_2d(metric_name)
                for i, (up, Zk_map, _, _) in enumerate(results):
                    Zk = Zk_map[metric_name]
                    Zm = align_Z_to_axes(Zk, axes_names, results[0][2])
                    Z_plot = Zm.T
                    title_2d = f"metric={metric_name} | {cum_col} upper={up:.3g}"
                    cbl = "integral"
                    if ref_z_common is not None:
                        ratio = Z_plot / ref_z_common
                        ref_desc = ", ".join(f"{k}={v:.4g}" for k, v in actual_ref_common.items())
                        if args.normalize_as_change_rate:
                            disp = (ratio - 1.0) * 100.0
                            title_2d = f"{title_2d}\n(change rate [%] from ref: {ref_desc}, value={ref_z_common:.4g})"
                            cbl = "change rate [%]"
                        else:
                            disp = ratio
                            title_2d = f"{title_2d}\n(normalized by ref: {ref_desc}, value={ref_z_common:.4g})"
                            cbl = "ratio"
                    elif args.normalize_ref and per_step_normalize:
                        ref_spec = parse_normalize_ref(args.normalize_ref)
                        ratio, ref_z_step, actual_ref_step = normalize_by_ref_2d(
                            Z_plot, Xv, Yv, axes_names[0], axes_names[1], ref_spec
                        )
                        ref_desc = ", ".join(f"{k}={v:.4g}" for k, v in actual_ref_step.items())
                        if args.normalize_as_change_rate:
                            disp = (ratio - 1.0) * 100.0
                            title_2d = f"{title_2d}\n(change rate [%] per-step from ref: {ref_desc}, value={ref_z_step:.4g})"
                            cbl = "change rate [%]"
                        else:
                            disp = ratio
                            title_2d = f"{title_2d}\n(normalized per-step by ref: {ref_desc}, value={ref_z_step:.4g})"
                            cbl = "ratio"
                    else:
                        disp = Z_plot
                    out_path = f"{stem}_{metric_name}_upper_{up:.6g}{ext}"
                    xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                    ylab = args.ylabel if args.ylabel is not None else axes_names[1]
                    cbl_eff = args.colorbar_label if args.colorbar_label is not None else cbl
                    disp_plot = apply_hull_nan(disp)
                    title_eff = title_2d if not hull_desc else f"{title_2d}\n(mask: {hull_desc})"
                    plot_2d(
                        xlab, ylab, Xv, Yv, disp_plot,
                        maybe_title(title_eff), out_path,
                        overlay_points=overlay, vmin=hm_vmin, vmax=hm_vmax, cmap=args.colormap,
                        colorbar_label=cbl_eff,
                        show_colorbar=(not bool(args.no_colorbar)),
                        transparent=transparent,
                    )
                    if args.output_csv:
                        csv_path = f"{stem}_{metric_name}_upper_{up:.6g}.csv"
                        dump_csv(axes_names, axes_vals, disp_plot, csv_path)

            if is_multi and args.combine == "logsum":
                ref_m, _ = compute_common_ref_2d("moment_abs")
                ref_g, _ = compute_common_ref_2d("grad_abs")
                for up, Zk_map, _, _ in results:
                    Zm = align_Z_to_axes(Zk_map["moment_abs"], axes_names, results[0][2]).T
                    Zg = align_Z_to_axes(Zk_map["grad_abs"], axes_names, results[0][2]).T
                    if (ref_m is not None) and (ref_g is not None):
                        r_m = Zm / ref_m
                        r_g = Zg / ref_g
                    else:
                        ref_spec = parse_normalize_ref(args.normalize_ref)
                        r_m, _, _ = normalize_by_ref_2d(Zm, Xv, Yv, axes_names[0], axes_names[1], ref_spec)
                        r_g, _, _ = normalize_by_ref_2d(Zg, Xv, Yv, axes_names[0], axes_names[1], ref_spec)
                    score = (w_m * _safe_log_ratio(r_m) + w_g * _safe_log_ratio(r_g)) / w_sum
                    improve_pct = _improve_pct_from_log_ratio(score)
                    improve_plot = apply_hull_nan(improve_pct)
                    # CSV export for combined (logsum) in cumulative 2D
                    if args.output_csv:
                        csv_path = f"{stem}_combined_upper_{up:.6g}.csv"
                        dump_csv(axes_names, axes_vals, improve_plot, csv_path)
                    if (hm_vmin is not None) and (hm_vmax is not None):
                        vmin_c, vmax_c = hm_vmin, hm_vmax
                    else:
                        c = np.asarray(improve_plot, dtype=float)
                        mabs = float(np.nanmax(np.abs(c))) if np.isfinite(c).any() else 0.0
                        vmin_c = -mabs if mabs > 0.0 else None
                        vmax_c = +mabs if mabs > 0.0 else None
                    out_path = f"{stem}_combined_upper_{up:.6g}{ext}"
                    title_2d = f"combined=logsum (moment_abs={w_m:g}, grad_abs={w_g:g}) | {cum_col} upper={up:.3g}"
                    xlab = args.xlabel if args.xlabel is not None else axes_names[0]
                    ylab = args.ylabel if args.ylabel is not None else axes_names[1]
                    cbl_eff = args.colorbar_label if args.colorbar_label is not None else "improve [%] (1 - geom-mean ratio)"

                    # optima extraction per upper (combined 2D cumulative)
                    opt = None
                    if bool(args.optima):
                        opt = _optima_2d_from_combined(
                            improve_plot,
                            np.asarray(Xv, dtype=float),
                            np.asarray(Yv, dtype=float),
                            top_pct=float(args.optima_top_pct),
                            delta=(None if args.optima_delta is None else float(args.optima_delta)),
                            connectivity=int(args.optima_connectivity),
                            max_islands=int(args.optima_max_islands),
                        )
                        islands = opt.get("islands", []) if isinstance(opt, dict) else []
                        if islands:
                            print(f"[optima] cumulative combined 2D: upper={up:.6g} method={opt.get('method')} threshold={opt.get('threshold'):.6g} vmax={opt.get('vmax'):.6g} islands={len(islands)}")
                            for i, d in enumerate(islands, start=1):
                                print(
                                    f"  #{i}: rep=({float(d['rep_x']):.6g}, {float(d['rep_y']):.6g}) "
                                    f"value={float(d['rep_value']):.6g} island_max={float(d['island_max']):.6g} n_cells={int(d['n_cells'])}"
                                )
                            if args.output_optima_csv:
                                csv_path = _optima_csv_path(args.output_optima_csv, f"upper_{up:.6g}")
                                if csv_path:
                                    df_opt = pd.DataFrame(
                                        [
                                            dict(
                                                upper=float(up),
                                                rank=i,
                                                rep_x=float(d["rep_x"]),
                                                rep_y=float(d["rep_y"]),
                                                rep_value=float(d["rep_value"]),
                                                island_max=float(d["island_max"]),
                                                island_median=float(d["island_median"]),
                                                n_cells=int(d["n_cells"]),
                                                area=float(d["area"]),
                                                center_median_x=float(d["center_median_x"]),
                                                center_median_y=float(d["center_median_y"]),
                                                threshold=float(opt.get("threshold")),
                                                vmax=float(opt.get("vmax")),
                                                method=str(opt.get("method")),
                                            )
                                            for i, d in enumerate(islands, start=1)
                                        ]
                                    )
                                    df_opt.to_csv(csv_path, index=False)

                    corner_text = ""
                    if opt and isinstance(opt, dict) and opt.get("islands"):
                        corner_text = _format_peak_list_text(
                            opt.get("islands", []),
                            x_label=xlab,
                            y_label=ylab,
                            max_items=int(args.optima_max_islands),
                        )
                    title_eff = title_2d if not hull_desc else f"{title_2d}\n(mask: {hull_desc})"
                    # combined は従来 custom_improve 固定だったが、--colormap が明示された場合はそれを優先する
                    cmap_combined = args.colormap if str(args.colormap) != "viridis" else "custom_improve"
                    plot_2d(
                        xlab, ylab, Xv, Yv, improve_plot,
                        maybe_title(title_eff), out_path,
                        overlay_points=overlay,
                        contour_mask=(None if (not opt or opt.get("mask") is None) else opt.get("mask")),
                        mark_points=(None if (not opt or opt.get("rep_points") is None) else opt.get("rep_points")),
                        mark_labels=(None if (not opt) else opt.get("labels")),
                        corner_text=corner_text,
                        vmin=vmin_c, vmax=vmax_c, cmap=cmap_combined,
                        colorbar_label=cbl_eff,
                        show_colorbar=(not bool(args.no_colorbar)),
                        transparent=transparent,
                    )
        else:
            print(f"可視化軸が3以上のため図は出力しません（累積モード）。CSVで出力します。axes={axes_names}")
            if args.output_csv:
                stem, _ = os.path.splitext(args.output_csv)
                for up, Zk_map, _, _ in results:
                    for m in metrics:
                        path = f"{stem}_{m}_upper_{up:.6g}.csv"
                        dump_csv(axes_names, axes_vals, Zk_map[m], path)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())


