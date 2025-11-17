from __future__ import annotations

from typing import Dict, Sequence, Tuple
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def plot_1d(x_name: str, x: np.ndarray, y: np.ndarray, title: str = "", output: str | None = None):
    plt.figure(figsize=(10, 6))
    plt.rcParams.update({'font.size': 16})
    plt.plot(x, y, color='red', lw=3.0)
    plt.xlabel(x_name)
    plt.ylabel('integral')
    if title:
        plt.title(title)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    if output:
        plt.savefig(output, dpi=300, bbox_inches='tight')
    else:
        plt.show()
    plt.close()


def plot_2d(x_name: str, y_name: str, X: np.ndarray, Y: np.ndarray, Z: np.ndarray, title: str = "", output: str | None = None):
    plt.figure(figsize=(10, 8))
    plt.rcParams.update({'font.size': 16})
    plt.imshow(Z, origin='lower', aspect='auto', extent=[X.min(), X.max(), Y.min(), Y.max()], cmap='viridis')
    plt.colorbar(label='integral')
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


