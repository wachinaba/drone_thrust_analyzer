from __future__ import annotations

from typing import Iterable, List, Optional, Sequence
import numpy as np
from tqdm import tqdm


def compute_metric_values(
    model,
    X: np.ndarray,
    metric: str,
    feature_names: Sequence[str],
    grad_dims: Optional[Sequence[str]] = None,
    grad_norm: str = "l2",
    fd_steps: Optional[dict] = None,
) -> np.ndarray:
    """
    Evaluate metric on points X.

    metric:
      - 'moment_abs': |f|
      - 'grad_abs'  : ||grad f|| (dims selectable, norm l1/l2)
    fd_steps: optional per-dimension finite difference step in data space.
    """
    if metric not in {"moment_abs", "grad_abs"}:
        raise ValueError(f"Unsupported metric: {metric}")

    if metric == "moment_abs":
        y = model.predict(X, return_std=False)
        return np.abs(np.asarray(y).ravel())

    # grad_abs
    if not grad_dims or len(grad_dims) == 0:
        # default: use all features
        grad_dims = list(feature_names)
    dim_indices = [feature_names.index(d) for d in grad_dims]

    # choose steps
    if fd_steps is None:
        fd_steps = {}

    grads_sq = np.zeros((X.shape[0], len(dim_indices)), dtype=float)
    grad_dim_names = [feature_names[j] for j in dim_indices]
    for k, j in enumerate(tqdm(dim_indices, desc="Computing gradients", leave=False)):
        h = fd_steps.get(feature_names[j], None)
        if h is None:
            # heuristic: relative step 1e-3 of data range or 1e-3 of std
            col = X[:, j]
            rng = float(np.nanmax(col) - np.nanmin(col))
            h = max(rng * 1e-3, 1e-6)

        Xp = X.copy(); Xp[:, j] += h
        Xm = X.copy(); Xm[:, j] -= h
        yp = model.predict(Xp, return_std=False).ravel()
        ym = model.predict(Xm, return_std=False).ravel()
        d = (yp - ym) / (2.0 * h)
        grads_sq[:, k] = d * d

    if grad_norm == "l1":
        val = np.sqrt(grads_sq)  # |d|
        return np.sum(val, axis=1)
    else:
        # l2
        return np.sqrt(np.sum(grads_sq, axis=1))


