"""Top-down picture of the map and the path. Display only; not a score."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection


def write_topdown(
    path: Path,
    map_xyz: np.ndarray | None,
    traj_xyz: np.ndarray | None,
    traj_t: np.ndarray | None,
    start_xyz: np.ndarray | None,
    end_xyz: np.ndarray | None,
    title: str,
) -> None:
    map_xy = _xy(map_xyz)
    path_xy = _xy(traj_xyz)
    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    xmin, xmax, ymin, ymax = _bounds(map_xy, path_xy)
    if map_xy is not None and len(map_xy):
        counts, _, _ = np.histogram2d(
            map_xy[:, 0],
            map_xy[:, 1],
            bins=360,
            range=[[xmin, xmax], [ymin, ymax]],
        )
        ax.imshow(
            np.log1p(counts.T),
            origin="lower",
            extent=[xmin, xmax, ymin, ymax],
            cmap="gray_r",
            aspect="equal",
            interpolation="nearest",
            zorder=0,
        )
    else:
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal")
        ax.set_facecolor("white")

    if path_xy is not None and len(path_xy) >= 2 and traj_t is not None and len(traj_t) == len(path_xy):
        segments = np.stack([path_xy[:-1], path_xy[1:]], axis=1)
        times = np.asarray(traj_t, dtype=np.float64)
        collection = LineCollection(segments, cmap="plasma", norm=plt.Normalize(times[0], times[-1]))
        collection.set_array(times[:-1])
        collection.set_linewidth(1.6)
        collection.set_zorder(3)
        ax.add_collection(collection)
        bar = fig.colorbar(collection, ax=ax, fraction=0.046, pad=0.04)
        bar.set_label("time (s)")
    elif path_xy is not None and len(path_xy) == 1:
        ax.scatter(path_xy[0, 0], path_xy[0, 1], c="#6a3d9a", s=20, zorder=3)

    if start_xyz is not None:
        ax.scatter(
            [float(start_xyz[0])],
            [float(start_xyz[1])],
            s=70,
            c="#1b8a3e",
            marker="o",
            zorder=5,
            label="start mean",
            edgecolors="white",
            linewidths=0.7,
        )
    if end_xyz is not None:
        ax.scatter(
            [float(end_xyz[0])],
            [float(end_xyz[1])],
            s=70,
            c="#c4272f",
            marker="s",
            zorder=5,
            label="end mean",
            edgecolors="white",
            linewidths=0.7,
        )
    if start_xyz is not None or end_xyz is not None:
        ax.legend(loc="upper right", framealpha=0.9)

    if (map_xy is None or len(map_xy) == 0) and (path_xy is None or len(path_xy) == 0):
        ax.text(0.5, 0.5, "nothing to draw", transform=ax.transAxes, ha="center", va="center")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)


def _xy(xyz: np.ndarray | None) -> np.ndarray | None:
    if xyz is None:
        return None
    points = np.asarray(xyz, dtype=np.float64)
    if points.size == 0:
        return points.reshape(0, 3)[:, :2]
    if points.ndim != 2 or points.shape[1] < 2:
        return None
    keep = np.isfinite(points[:, 0]) & np.isfinite(points[:, 1])
    return points[keep, :2]


def _bounds(map_xy: np.ndarray | None, path_xy: np.ndarray | None):
    pieces = []
    if map_xy is not None and len(map_xy):
        pieces.append(np.percentile(map_xy, 0.5, axis=0))
        pieces.append(np.percentile(map_xy, 99.5, axis=0))
    if path_xy is not None and len(path_xy):
        pieces.append(path_xy.min(axis=0))
        pieces.append(path_xy.max(axis=0))
    if not pieces:
        return -1.0, 1.0, -1.0, 1.0
    stack = np.vstack(pieces)
    low = stack.min(axis=0)
    high = stack.max(axis=0)
    span = np.maximum(high - low, 1e-3)
    pad = np.maximum(span * 0.05, 0.25)
    return (
        float(low[0] - pad[0]),
        float(high[0] + pad[0]),
        float(low[1] - pad[1]),
        float(high[1] + pad[1]),
    )
