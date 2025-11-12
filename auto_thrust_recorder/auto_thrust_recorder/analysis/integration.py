from __future__ import annotations

from typing import Dict, List, Sequence, Tuple
import numpy as np


def make_linspace(min_val: float, max_val: float, points: int) -> np.ndarray:
    if points <= 1:
        return np.array([(min_val + max_val) * 0.5], dtype=float)
    return np.linspace(min_val, max_val, int(points))


def trapezoid_weights(n: int, span: float) -> np.ndarray:
    if n <= 1:
        return np.array([span], dtype=float)
    w = np.ones(n, dtype=float)
    w[0] = 0.5; w[-1] = 0.5
    return w * (span / (n - 1))


def build_grid(
    feature_names: Sequence[str],
    integrate_spec: Dict[str, Tuple[float, float, int]],
    fix_values: Dict[str, float],
    visualize_spec: Dict[str, Tuple[float, float, int]],
) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    """
    Return X grid (N x D), along with index arrays for visualization and integration axes.
    visualize_spec/integrate_spec: {col: (min, max, points)}
    """
    axes_vals = {}
    for col, (a, b, p) in visualize_spec.items():
        axes_vals[col] = make_linspace(a, b, p)
    for col, (a, b, p) in integrate_spec.items():
        axes_vals[col] = make_linspace(a, b, p)

    shape = [len(axes_vals[c]) if c in axes_vals else 1 for c in feature_names]
    total = int(np.prod(shape))
    X = np.zeros((total, len(feature_names)), dtype=float)

    # meshgrid style index
    grids = []
    for c in feature_names:
        if c in axes_vals:
            grids.append(axes_vals[c])
        else:
            grids.append(np.array([fix_values.get(c, 0.0)], dtype=float))
    # Use 'ij' indexing so that axis order matches input order (feature_names)
    MG = np.meshgrid(*grids, indexing='ij')
    for j, c in enumerate(feature_names):
        X[:, j] = MG[j].reshape(-1)

    return X, {k: axes_vals[k] for k in visualize_spec}, {k: axes_vals[k] for k in integrate_spec}


def integrate_values(
    values: np.ndarray,
    feature_names: Sequence[str],
    visualize_axes: Dict[str, np.ndarray],
    integrate_axes: Dict[str, np.ndarray],
) -> Tuple[np.ndarray, Tuple[int, ...]]:
    """
    Fold integral dimensions by trapezoidal rule, return values reshaped over visualize axes.
    """
    # compute weights per integrate axis
    weights = {}
    for col, arr in integrate_axes.items():
        n = len(arr)
        span = float(arr[-1] - arr[0])
        weights[col] = trapezoid_weights(n, span)

    # reshape to full grid (axis order == feature_names)
    shape = [
        (len(visualize_axes[c]) if c in visualize_axes else (len(integrate_axes[c]) if c in integrate_axes else 1))
        for c in feature_names
    ]
    V = values.reshape(shape)

    # fold integrate dims in reverse axis order to avoid index shifts
    for idx in reversed(range(len(feature_names))):
        c = feature_names[idx]
        if c in integrate_axes:
            w = weights[c]
            V = np.tensordot(V, w, axes=(idx, 0))

    # resulting shape is visualize dims only
    vis_shape = tuple(len(visualize_axes[c]) for c in feature_names if c in visualize_axes)
    return V.reshape(vis_shape), vis_shape


