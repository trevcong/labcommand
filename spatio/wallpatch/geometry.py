"""Oriented patch boxes and a robust plane fit.

World up is +Z. FAST-LIO's map frame is the IMU frame at the first sample,
and its gravity vector is initialized opposite the mean specific force
(default mean acceleration (0, 0, -1), so +Z is up when the rig starts level).
Both historical maps are tightest on Z in the central percentiles.

half_extent_m is (along the wall, across the wall, vertical) in the axes below.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from spatio.wallpatch.errors import WallpatchError
from spatio.wallpatch.numbers import parse_positive_metres, parse_real

WORLD_UP = np.array([0.0, 0.0, 1.0], dtype=np.float64)


def project(points, axis):
    """Coordinate of each point along one axis.

    Written out by component. On this Mac, numpy's matmul warns and can drop
    a point for a length-3 projection.
    """
    pts = np.asarray(points, dtype=np.float64)
    ax = np.asarray(axis, dtype=np.float64)
    return pts[:, 0] * ax[0] + pts[:, 1] * ax[1] + pts[:, 2] * ax[2]


def gram3(centered):
    """3x3 Gram matrix of an (N, 3) cloud, without matmul."""
    c0 = centered[:, 0]
    c1 = centered[:, 1]
    c2 = centered[:, 2]
    return np.array(
        [
            [np.dot(c0, c0), np.dot(c0, c1), np.dot(c0, c2)],
            [np.dot(c1, c0), np.dot(c1, c1), np.dot(c1, c2)],
            [np.dot(c2, c0), np.dot(c2, c1), np.dot(c2, c2)],
        ],
        dtype=np.float64,
    )

# Trimming stops if it would discard half the cropped points. A balanced
# doubled wall must stay intact so the second sheet is still visible.
_MIN_KEEP_FRACTION = 0.5
_MAD_K = 3.0
_MAD_TO_SIGMA = 1.4826
_MAX_EIGEN_RATIO = 0.5


def box_axes(normal_hint):
    # type: (np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]
    """Return unit (along, across, vertical), right-handed.

    `across` follows the hint. `along` is horizontal (world-up × across).
    `vertical` completes the frame and equals world up when the hint is horizontal.
    """
    hint = np.asarray(normal_hint, dtype=np.float64)
    norm = float(np.linalg.norm(hint))
    if norm < 1e-8 or not np.isfinite(norm):
        raise WallpatchError("normal_hint is degenerate")
    across = hint / norm
    side = np.cross(WORLD_UP, across)
    side_norm = float(np.linalg.norm(side))
    if side_norm < 1e-6:
        # The hint is vertical. The patch is not a wall; pick any horizontal axis.
        side = np.cross(np.array([1.0, 0.0, 0.0]), across)
        side_norm = float(np.linalg.norm(side))
        if side_norm < 1e-6:
            side = np.cross(np.array([0.0, 1.0, 0.0]), across)
            side_norm = float(np.linalg.norm(side))
    along = side / side_norm
    vertical = np.cross(across, along)
    vertical_norm = float(np.linalg.norm(vertical))
    if vertical_norm < 1e-8:
        raise WallpatchError("could not build a patch frame from normal_hint")
    vertical = vertical / vertical_norm
    return along, across, vertical


def parse_box(box):
    """Return (center, hint, half) or raise WallpatchError."""
    if not isinstance(box, dict):
        raise WallpatchError("box_map_frame must be an object")
    try:
        center = _vec3(box["center_m"], "center_m", positive=False)
        hint = _vec3(box["normal_hint"], "normal_hint", positive=False)
        half = _vec3(box["half_extent_m"], "half_extent_m", positive=True)
    except KeyError:
        raise WallpatchError(
            "box_map_frame needs center_m, normal_hint, and half_extent_m"
        )
    # Validates the hint and raises if it is degenerate.
    box_axes(hint)
    return center, hint, half


def _vec3(raw, name, positive):
    """Three metres, through the shared number rule."""
    if isinstance(raw, (str, bytes, bytearray)) or not hasattr(raw, "__len__") or len(raw) != 3:
        raise WallpatchError("box vectors must have length 3")
    parser = parse_positive_metres if positive else parse_real
    return np.asarray([parser(item, name) for item in raw], dtype=np.float64)


def crop_mask(points, center, normal_hint, half_extent):
    """True for points inside the closed oriented box. No downsampling."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0,), dtype=bool)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise WallpatchError("points must have shape (N, 3)")
    along, across, vertical = box_axes(normal_hint)
    delta = pts - np.asarray(center, dtype=np.float64)
    coord_along = project(delta, along)
    coord_across = project(delta, across)
    coord_vertical = project(delta, vertical)
    half = np.asarray(half_extent, dtype=np.float64)
    return (
        (np.abs(coord_along) <= half[0])
        & (np.abs(coord_across) <= half[1])
        & (np.abs(coord_vertical) <= half[2])
    )


def crop_points(points, center, normal_hint, half_extent):
    mask = crop_mask(points, center, normal_hint, half_extent)
    return np.asarray(points, dtype=np.float64)[mask]


def angle_deg(vector_a, vector_b):
    """Angle in degrees between two vectors, in [0, 180]."""
    a = np.asarray(vector_a, dtype=np.float64)
    b = np.asarray(vector_b, dtype=np.float64)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    cosine = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def fit_plane(points, normal_hint):
    # type: (np.ndarray, np.ndarray) -> Optional[dict]
    """PCA plane with iterative 3×MAD trimming.

    Returns None when the cloud is not a plane. Residuals for the report are
    computed by the caller on every cropped point, not on the inliers only.
    The normal is flipped so it points the same way as normal_hint.
    """
    pts = np.asarray(points, dtype=np.float64)
    hint = np.asarray(normal_hint, dtype=np.float64)
    hint = hint / np.linalg.norm(hint)
    if len(pts) < 3:
        return None
    chosen = pts
    floor_keep = max(3, int(_MIN_KEEP_FRACTION * len(pts)))
    for _ in range(5):
        fitted = _pca_normal(chosen, hint)
        if fitted is None:
            return None
        centroid, normal, _eigvals = fitted
        signed = project(chosen - centroid, normal)
        median = float(np.median(signed))
        mad = float(np.median(np.abs(signed - median)))
        scale = _MAD_TO_SIGMA * mad
        if scale < 1e-9:
            break
        keep = np.abs(signed - median) <= _MAD_K * scale
        n_keep = int(np.count_nonzero(keep))
        if n_keep < floor_keep or n_keep == len(chosen):
            break
        chosen = chosen[keep]
    fitted = _pca_normal(chosen, hint)
    if fitted is None:
        return None
    centroid, normal, eigvals = fitted
    if float(eigvals[1]) < 1e-10:
        return None
    ratio = float(eigvals[0] / eigvals[1])
    if ratio > _MAX_EIGEN_RATIO:
        return None
    signed = project(chosen - centroid, normal)
    rms = float(np.sqrt(np.mean(signed * signed)))
    return {
        "point": centroid,
        "normal": normal,
        "eigvals": eigvals,
        "eigenvalue_ratio": ratio,
        "n_inliers": int(len(chosen)),
        "rms_inlier_m": rms,
    }


def _pca_normal(points, hint):
    """Smallest-eigenvector normal, flipped toward hint. None if it is undefined."""
    if len(points) < 3:
        return None
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = gram3(centered) / float(max(len(points) - 1, 1))
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = np.asarray(eigvecs[:, 0], dtype=np.float64).copy()
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12 or not np.isfinite(normal).all():
        return None
    normal /= norm
    if float(np.dot(normal, hint)) < 0.0:
        normal = -normal
    return centroid, normal, np.asarray(eigvals, dtype=np.float64)
