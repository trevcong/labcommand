"""Two-surface test on signed point-to-plane distances.

The distances have no visit id behind them. A positive result means two
parallel surfaces sit inside the patch, not that a particular revisit drifted.
"""

from __future__ import annotations

import numpy as np

# 2 mm bins, as required. Peak merging is 6 bins so sample wobble on one
# surface does not look like a second wall.
BIN_WIDTH_M = 0.002
PEAK_MERGE_M = 0.012
MIN_PEAK_MASS = 0.20
VALLEY_HEIGHT_RATIO = 0.60
# 6 cm at σ = 5 mm clears 40 mm cleanly. 3 cm at the same noise falls in the
# band below and is reported undetermined, never as a single wall.
CONFIDENT_SEPARATION_M = 0.040
UNCERTAIN_SEPARATION_M = 0.020

RULE_TEXT = (
    "Signed distances to the robust plane are histogrammed in 2 mm bins and "
    "smoothed with a 3-bin [1, 2, 1] kernel. Local maxima closer than 12 mm "
    "count as one peak. The two tallest peaks are split at their midpoint. "
    "The split is kept when each side holds at least 20% of the points and "
    "the valley between the peaks is at most 60% of the shorter peak. "
    "separation_m is the difference of the mean signed distances of the two "
    "sides. At least 40 mm is doubled=true. At least 20 mm and under 40 mm "
    "is doubled=undetermined. Otherwise doubled=false and separation_m is "
    "null, because there is no second surface centre. This is not a "
    "visit-to-visit measurement."
)


def analyze_distances(distances):
    """Histogram, residual summaries, and the doubled-wall decision.

    `distances` must be the signed distances of every cropped point.
    """
    values = np.asarray(distances, dtype=np.float64).reshape(-1)
    if values.size == 0:
        raise ValueError("analyze_distances needs at least one distance")
    absolute = np.abs(values)
    p95 = float(np.percentile(absolute, 95))
    low, high = np.percentile(values, [2.5, 97.5])
    counts, edges = _histogram(values)
    decision = _decide(values, counts, edges)
    return {
        "p95_abs_m": p95,
        "p2_5_m": float(low),
        "p97_5_m": float(high),
        "thickness_m": float(high - low),
        "bin_width_m": BIN_WIDTH_M,
        "n": int(values.size),
        "doubled": decision["doubled"],
        "separation_m": decision["separation_m"],
        "mark_m": decision["mark_m"],
        "hist_counts": counts,
        "hist_edges": edges,
    }


def _histogram(values):
    bin_w = BIN_WIDTH_M
    i0 = int(np.floor(float(values.min()) / bin_w + 1e-9))
    i1 = int(np.ceil(float(values.max()) / bin_w - 1e-9))
    if i1 <= i0:
        i1 = i0 + 1
    lo = i0 * bin_w
    hi = i1 * bin_w
    n_bins = i1 - i0
    counts, edges = np.histogram(values, bins=n_bins, range=(lo, hi))
    return counts, edges


def _smooth(counts):
    kernel = np.array([1.0, 2.0, 1.0], dtype=np.float64)
    kernel /= kernel.sum()
    return np.convolve(counts.astype(np.float64), kernel, mode="same")


def _local_peaks(smooth):
    """Centres of strict local maxima, including the middle of a flat top."""
    peaks = []
    index = 0
    n_bins = len(smooth)
    while index < n_bins:
        end = index
        while end + 1 < n_bins and smooth[end + 1] == smooth[index]:
            end += 1
        left = smooth[index - 1] if index > 0 else -np.inf
        right = smooth[end + 1] if end + 1 < n_bins else -np.inf
        if smooth[index] > left and smooth[index] >= right and smooth[index] > 0.0:
            peaks.append((index + end) // 2)
        index = end + 1
    return peaks


def _decide(values, counts, edges):
    centers = 0.5 * (edges[:-1] + edges[1:])
    smooth = _smooth(counts)
    peaks = _local_peaks(smooth)
    order = sorted(peaks, key=lambda idx: float(smooth[idx]), reverse=True)
    kept = []
    for idx in order:
        if all(abs(float(centers[idx] - centers[other])) >= PEAK_MERGE_M for other in kept):
            kept.append(idx)
    if not kept:
        return {"doubled": "false", "separation_m": None, "mark_m": []}
    if len(kept) == 1:
        return {
            "doubled": "false",
            "separation_m": None,
            "mark_m": [float(centers[kept[0]])],
        }
    tall = sorted(kept, key=lambda idx: float(smooth[idx]), reverse=True)[:2]
    tall = sorted(tall, key=lambda idx: float(centers[idx]))
    left_i, right_i = tall
    lo, hi = (left_i, right_i) if left_i < right_i else (right_i, left_i)
    valley = float(smooth[lo : hi + 1].min())
    shorter = min(float(smooth[left_i]), float(smooth[right_i]))
    valley_ok = valley <= VALLEY_HEIGHT_RATIO * shorter
    midpoint = 0.5 * (float(centers[left_i]) + float(centers[right_i]))
    left = values[values <= midpoint]
    right = values[values > midpoint]
    n_values = float(len(values))
    mass_left = len(left) / n_values
    mass_right = len(right) / n_values
    mass_ok = mass_left >= MIN_PEAK_MASS and mass_right >= MIN_PEAK_MASS
    if len(left) == 0 or len(right) == 0:
        separation = 0.0
    else:
        separation = abs(float(left.mean()) - float(right.mean()))
    marks = [float(left.mean()) if len(left) else float(centers[left_i]),
             float(right.mean()) if len(right) else float(centers[right_i])]
    if valley_ok and mass_ok and separation >= CONFIDENT_SEPARATION_M:
        return {"doubled": "true", "separation_m": separation, "mark_m": marks}
    if valley_ok and mass_ok and separation >= UNCERTAIN_SEPARATION_M:
        return {
            "doubled": "undetermined",
            "separation_m": separation,
            "mark_m": marks,
        }
    # One dominant surface. Mark its peak only.
    main = max(tall, key=lambda idx: float(smooth[idx]))
    return {
        "doubled": "false",
        "separation_m": None,
        "mark_m": [float(centers[main])],
    }
