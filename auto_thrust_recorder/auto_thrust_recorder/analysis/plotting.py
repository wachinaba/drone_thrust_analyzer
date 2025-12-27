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


# カスタムカラーマップを登録
plt.colormaps.register(cmap=get_custom_rdbu_cmap(), name="custom_rdbu")
plt.colormaps.register(cmap=get_custom_improve_cmap(), name="custom_improve")


def plot_1d(
    x_name: str,
    x: np.ndarray,
    y: np.ndarray,
    title: str = "",
    output: str | None = None,
    ylabel: str = "integral",
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
        plt.savefig(output, dpi=300, bbox_inches='tight')
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
    vmin: float | None = None,
    vmax: float | None = None,
    cmap: str = "viridis",
    colorbar_label: str = "integral",
    show_colorbar: bool = True,
):
    plt.figure(figsize=(10, 8))
    plt.rcParams.update({'font.size': 16})
    ax = plt.gca()
    ax.imshow(Z, origin='lower', aspect='auto', extent=[X.min(), X.max(), Y.min(), Y.max()], cmap=cmap, vmin=vmin, vmax=vmax)
    if bool(show_colorbar):
        plt.colorbar(label=colorbar_label)

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
    plt.xlabel(x_name)
    plt.ylabel(y_name)
    if title:
        plt.title(title)
    plt.tight_layout()
    if output:
        plt.savefig(output, dpi=300, bbox_inches='tight')
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


