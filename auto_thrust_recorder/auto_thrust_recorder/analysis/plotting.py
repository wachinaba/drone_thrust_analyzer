from __future__ import annotations

from typing import Dict, Sequence, Tuple, Optional, Any
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import pandas as pd


def get_custom_rdbu_cmap():
    """
    カスタムカラーマップ: -100%〜+300% の非対称RdBu
    - -100% (-1.0 in change rate, or -100 in %) が濃い青
    - 0% が白
    - +300% (+3.0 in change rate, or +300 in %) が濃い赤
    
    使用時は vmin=-100, vmax=300 (％単位) で指定することを想定
    """
    # -100〜+300 の範囲で 0 が 1/4 (0.25) の位置
    # 青(濃) -> 白 -> 赤(濃)
    colors = [
        (0.0, (0.0196, 0.188, 0.380)),    # 濃い青 at -100%
        (0.25, (1.0, 1.0, 1.0)),           # 白 at 0%
        (1.0, (0.404, 0.0, 0.122)),        # 濃い赤 at +300%
    ]
    cmap = mcolors.LinearSegmentedColormap.from_list("custom_rdbu", colors)
    return cmap


def get_custom_improve_cmap():
    """
    改善率[%] 用の発散カラーマップ（悪化=赤 / 0=白 / 改善=緑）。
    0 がレンジ中央に来る（vmin=-A, vmax=+A のように対称）ことを前提に設計。
    """
    colors = [
        (0.0, (0.404, 0.0, 0.122)),  # red (bad)  at vmin
        (0.5, (1.0, 1.0, 1.0)),      # white      at 0
        (1.0, (0.0, 0.39, 0.0)),     # green(good)at vmax
    ]
    cmap = mcolors.LinearSegmentedColormap.from_list("custom_improve", colors)
    return cmap


def get_custom_rwg_cmap():
    """
    -100〜+100 の発散カラーマップ（-100=赤 / 0=白 / +100=緑）。
    0 がレンジ中央に来る（vmin=-100, vmax=+100 のように対称）ことを前提に設計。
    """
    colors = [
        (0.0, (0.404, 0.0, 0.122)),  # red (at vmin)
        (0.5, (1.0, 1.0, 1.0)),      # white (at 0)
        (1.0, (0.0, 0.39, 0.0)),     # green (at vmax)
    ]
    cmap = mcolors.LinearSegmentedColormap.from_list("custom_rwg", colors)
    return cmap


def get_custom_log2_rwg_cmap():
    """
    log2オッズ用の非対称発散カラーマップ（-2=赤 / 0=白 / +10=緑）。
    使用時は vmin=-2, vmax=+10 を指定することを想定。

    -2〜+10 の範囲で 0 が 1/6 (≈0.1667) の位置に来る。
    """
    # vmin=-2, vmax=+10 のとき 0 の位置
    zero_pos = (0.0 - (-2.0)) / (10.0 - (-2.0))  # 2/12
    colors = [
        (0.0, (0.404, 0.0, 0.122)),       # red   at -2
        (float(zero_pos), (1.0, 1.0, 1.0)),  # white at 0
        (1.0, (0.0, 0.39, 0.0)),          # green at +10
    ]
    cmap = mcolors.LinearSegmentedColormap.from_list("custom_log2_rwg", colors)
    return cmap


# カスタムカラーマップを登録
plt.colormaps.register(cmap=get_custom_rdbu_cmap(), name="custom_rdbu")
plt.colormaps.register(cmap=get_custom_improve_cmap(), name="custom_improve")
plt.colormaps.register(cmap=get_custom_rwg_cmap(), name="custom_rwg")
plt.colormaps.register(cmap=get_custom_log2_rwg_cmap(), name="custom_log2_rwg")


def plot_1d(
    x_name: str,
    x: np.ndarray,
    y: np.ndarray,
    title: str = "",
    output: str | None = None,
    ylabel: str = "integral",
    transparent: bool = False,
):
    plt.figure(figsize=(10, 6))
    plt.rcParams.update({'font.size': 16})
    plt.plot(x, y, color='red', lw=3.0)
    plt.xlabel(x_name)
    plt.ylabel(ylabel)
    if title:
        plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    if output:
        plt.savefig(output, dpi=300, bbox_inches='tight', transparent=bool(transparent))
    else:
        plt.show()
    plt.close()


def plot_2d(
    x_name: str,
    y_name: str,
    X: np.ndarray,
    Y: np.ndarray,
    Z: np.ndarray,
    title: str = "",
    output: str | None = None,
    overlay_points: np.ndarray | None = None,
    overlay_style: dict | None = None,
    contour_mask: np.ndarray | None = None,
    contour_style: dict | None = None,
    mark_points: np.ndarray | None = None,
    mark_style: dict | None = None,
    mark_labels: Sequence[str] | None = None,
    mark_label_style: dict | None = None,
    corner_text: str | None = None,
    corner_text_style: dict | None = None,
    vmin: float | None = None,
    vmax: float | None = None,
    cmap: str = "viridis",
    colorbar_label: str = "integral",
    show_colorbar: bool = True,
    transparent: bool = False,
):
    plt.figure(figsize=(10, 8))
    plt.rcParams.update({'font.size': 16})
    ax = plt.gca()

    def _edges_from_centers(v: np.ndarray) -> np.ndarray:
        vv = np.asarray(v, dtype=float).reshape(-1)
        if vv.size == 1:
            # arbitrary unit width
            return np.asarray([vv[0] - 0.5, vv[0] + 0.5], dtype=float)
        dv = np.diff(vv)
        edges = np.empty(vv.size + 1, dtype=float)
        edges[1:-1] = (vv[:-1] + vv[1:]) * 0.5
        edges[0] = vv[0] - dv[0] * 0.5
        edges[-1] = vv[-1] + dv[-1] * 0.5
        return edges

    Xc = np.asarray(X, dtype=float).reshape(-1)
    Yc = np.asarray(Y, dtype=float).reshape(-1)
    Xe = _edges_from_centers(Xc)
    Ye = _edges_from_centers(Yc)

    # If spacing is non-uniform, imshow (equal pixel widths) will look shifted.
    # Use pcolormesh for correct cell geometry.
    def _is_uniform_spacing(v: np.ndarray, rtol: float = 1e-2) -> bool:
        vv = np.asarray(v, dtype=float).reshape(-1)
        if vv.size <= 2:
            return True
        dv = np.diff(vv)
        m = float(np.mean(np.abs(dv)))
        if m == 0.0:
            return True
        return bool(np.allclose(dv, dv[0], rtol=rtol, atol=1e-12))

    Z0 = np.asarray(Z)
    if _is_uniform_spacing(Xc) and _is_uniform_spacing(Yc):
        # Align extent to cell edges (prevents half-pixel visual offsets vs contour).
        im = ax.imshow(
            Z0,
            origin='lower',
            aspect='auto',
            extent=[Xe[0], Xe[-1], Ye[0], Ye[-1]],
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
        )
    else:
        # Correct rendering for non-uniform grids.
        im = ax.pcolormesh(Xe, Ye, Z0, cmap=cmap, vmin=vmin, vmax=vmax, shading='auto')

    if bool(show_colorbar):
        plt.colorbar(im, ax=ax, label=colorbar_label)

    # Optional contour overlay (e.g., near-optimal region boundary)
    if contour_mask is not None:
        m = np.asarray(contour_mask)
        if m.shape == np.asarray(Z).shape:
            Xg, Yg = np.meshgrid(np.asarray(X), np.asarray(Y))
            style: Dict[str, Any] = {
                "levels": [0.5],
                "colors": "white",
                "linewidths": 1.8,
                "alpha": 0.9,
            }
            if contour_style:
                style.update(contour_style)
            ax.contour(Xg, Yg, m.astype(float), **style)

    if overlay_points is not None:
        pts = np.asarray(overlay_points)
        if pts.ndim == 2 and pts.shape[1] == 2 and pts.shape[0] > 0:
            style = {
                "s": 18,
                "c": "white",
                "alpha": 0.65,
                "edgecolors": "black",
                "linewidths": 0.4,
            }
            if overlay_style:
                style.update(overlay_style)
            ax.scatter(pts[:, 0], pts[:, 1], **style)

    # Optional marker points (e.g., representative optima)
    if mark_points is not None:
        mp = np.asarray(mark_points)
        if mp.ndim == 2 and mp.shape[1] == 2 and mp.shape[0] > 0:
            style = {
                "s": 90,
                "c": "#ffea00",  # yellow
                "alpha": 0.95,
                "edgecolors": "black",
                "linewidths": 0.9,
                "marker": "o",
                "zorder": 6,
            }
            if mark_style:
                style.update(mark_style)
            ax.scatter(mp[:, 0], mp[:, 1], **style)

            if mark_labels is not None:
                lbl_style = {
                    "fontsize": 10,
                    "color": "black",
                    "ha": "left",
                    "va": "bottom",
                    "bbox": dict(boxstyle="round,pad=0.2", fc="white", ec="black", alpha=0.75, lw=0.5),
                    "zorder": 7,
                }
                if mark_label_style:
                    lbl_style.update(mark_label_style)
                for i in range(min(len(mark_labels), mp.shape[0])):
                    ax.text(float(mp[i, 0]), float(mp[i, 1]), str(mark_labels[i]), **lbl_style)

    # Optional corner text (bottom-left, in axes fraction)
    if corner_text:
        style: Dict[str, Any] = {
            "fontsize": 10,
            "color": "black",
            "ha": "left",
            "va": "bottom",
            "transform": ax.transAxes,
            "bbox": dict(boxstyle="round,pad=0.25", fc="white", ec="black", alpha=0.75, lw=0.5),
            "zorder": 10,
        }
        if corner_text_style:
            style.update(corner_text_style)
        ax.text(0.02, 0.02, str(corner_text), **style)
    plt.xlabel(x_name)
    plt.ylabel(y_name)
    if title:
        plt.title(title)
    plt.tight_layout()
    if output:
        plt.savefig(output, dpi=300, bbox_inches='tight', transparent=bool(transparent))
    else:
        plt.show()
    plt.close()


def dump_csv(axes_names: Sequence[str], axes_values: Sequence[np.ndarray], values_grid: np.ndarray, path: str):
    if len(axes_names) == 1:
        df = pd.DataFrame({axes_names[0]: axes_values[0], 'value': values_grid.reshape(-1)})
    elif len(axes_names) == 2:
        X, Y = np.meshgrid(axes_values[0], axes_values[1])
        df = pd.DataFrame({axes_names[0]: X.reshape(-1), axes_names[1]: Y.reshape(-1), 'value': values_grid.reshape(-1)})
    else:
        # flatten with multi-index columns
        grid_axes = np.meshgrid(*axes_values, indexing='xy')
        data = {axes_names[i]: grid_axes[i].reshape(-1) for i in range(len(axes_names))}
        data['value'] = values_grid.reshape(-1)
        df = pd.DataFrame(data)
    df.to_csv(path, index=False)


