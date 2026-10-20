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
    "smoothed with a 3-bin [1, 2, 1] kernel. The smoothed histogram has the "
    "same length as the raw one. Local maxima closer than 12 mm count as one "
    "peak. Each pair of remaining peaks is split at its midpoint. A split is "
    "kept when each side holds at least 20% of the points and the valley "
    "between the peaks is at most 60% of the shorter peak. A kept split of "
    "at least 40 mm is preferred. Ties go to the taller shorter peak, then "
    "the larger separation. separation_m is the difference of the mean signed "
    "distances of the two sides. At least 40 mm is doubled=true. At least 20 mm "
    "and under 40 mm is doubled=undetermined. If no split is kept, "
    "doubled=false and separation_m is null, because there is no second "
    "surface centre. This is not a visit-to-visit measurement. "
    "The same rule is run on distances along the fitted plane and along "
    "normal_hint. A confident split the fitted plane misses is kept when "
    "normal_hint still shows it, so a small rotation of the pooled fit cannot "
    "erase it. A confident split the hint misses is kept when the fitted plane "
    "still shows it. If both confident splits fall on opposite sides of the "
    "separation gate, or one axis reports a candidate the other does not "
    "confirm, the patch is not evaluable. A false result on both axes is a "
    "single surface."
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
    """3-bin [1, 2, 1] smooth. The result has one value per histogram bin.

    np.convolve(..., mode="same") returns max(len(counts), 3). A one-bin
    plane then looks up a centre that does not exist.
    """
    values = np.asarray(counts, dtype=np.float64).reshape(-1)
    kernel = np.array([1.0, 2.0, 1.0], dtype=np.float64)
    kernel /= kernel.sum()
    if values.size == 0:
        return values
    full = np.convolve(values, kernel, mode="full")
    start = (kernel.size - 1) // 2
    return full[start : start + values.size]


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
    # The two tallest peaks can both sit on one sheet. Try every remaining pair.
    best = None
    for i in range(len(kept)):
        for j in range(i + 1, len(kept)):
            verdict = _evaluate_pair(values, smooth, centers, kept[i], kept[j])
            if verdict is None:
                continue
            if best is None or _prefer_pair(verdict, best):
                best = verdict
    if best is not None:
        return {
            "doubled": best["doubled"],
            "separation_m": best["separation_m"],
            "mark_m": best["mark_m"],
        }
    main = max(kept, key=lambda idx: float(smooth[idx]))
    return {
        "doubled": "false",
        "separation_m": None,
        "mark_m": [float(centers[main])],
    }


def _evaluate_pair(values, smooth, centers, index_a, index_b):
    """A kept split, or None when this pair is not two surfaces."""
    if centers[index_a] <= centers[index_b]:
        left_i, right_i = index_a, index_b
    else:
        left_i, right_i = index_b, index_a
    height_left = float(smooth[left_i])
    height_right = float(smooth[right_i])
    shorter = min(height_left, height_right)
    if shorter <= 0.0:
        return None
    valley = float(smooth[left_i : right_i + 1].min())
    if valley > VALLEY_HEIGHT_RATIO * shorter:
        return None
    midpoint = 0.5 * (float(centers[left_i]) + float(centers[right_i]))
    left = values[values <= midpoint]
    right = values[values > midpoint]
    if len(left) == 0 or len(right) == 0:
        return None
    n_values = float(len(values))
    if (len(left) / n_values) < MIN_PEAK_MASS or (len(right) / n_values) < MIN_PEAK_MASS:
        return None
    separation = abs(float(left.mean()) - float(right.mean()))
    if separation < UNCERTAIN_SEPARATION_M:
        return None
    doubled = "true" if separation >= CONFIDENT_SEPARATION_M else "undetermined"
    return {
        "doubled": doubled,
        "separation_m": separation,
        "mark_m": [float(left.mean()), float(right.mean())],
        "shorter": shorter,
    }


def reconcile_axes(fit_stats, hint_stats, separation_gate_m):
    """One doubled-wall decision from the plane normal and the hint.

    Partial overlap shears the pooled cloud. The fitted normal then tilts a
    few degrees, and distances along that normal can merge two parallel
    sheets into one peak. Distances along normal_hint still separate those
    sheets. A confident split on either axis is kept. Any other disagreement
    is not a score: the caller leaves the gates unevaluated.

    Returns kind "measured" with the stats that support doubled and
    separation_m, or kind "not_evaluable" with a reason. Point-to-plane
    residuals stay with the caller.
    """
    fit_d = fit_stats["doubled"]
    hint_d = hint_stats["doubled"]
    fit_s = fit_stats["separation_m"]
    hint_s = hint_stats["separation_m"]

    if fit_d == "false" and hint_d == "false":
        return _measured(fit_stats)
    if fit_d == "undetermined" and hint_d == "undetermined":
        return _measured(fit_stats)
    if fit_d == "true" and hint_d == "true":
        if fit_s is None or hint_s is None:
            return _refused(_disagree(fit_d, hint_d))
        fit_fails = float(fit_s) > float(separation_gate_m)
        hint_fails = float(hint_s) > float(separation_gate_m)
        if fit_fails != hint_fails:
            return _refused(
                "fitted plane separation %.4f m and normal_hint separation %.4f m "
                "are on opposite sides of the %.2f m separation gate; "
                "the patch is not evaluable"
                % (fit_s, hint_s, separation_gate_m)
            )
        # Both sides of the gate agree. Keep the plane-normal gap. The hint
        # cannot shrink it under the gate, because that case was refused above.
        return _measured(fit_stats)
    if fit_d == "false" and hint_d == "true":
        return _measured(hint_stats)
    if fit_d == "true" and hint_d == "false":
        return _measured(fit_stats)
    # A confident split that already fails the gate is still reported.
    # A confident split that would pass, next to an unconfirmed candidate,
    # is not a pass.
    if fit_d == "true" and hint_d == "undetermined" and fit_s is not None:
        if float(fit_s) > float(separation_gate_m):
            return _measured(fit_stats)
    if hint_d == "true" and fit_d == "undetermined" and hint_s is not None:
        if float(hint_s) > float(separation_gate_m):
            return _measured(hint_stats)
    return _refused(_disagree(fit_d, hint_d))


def _measured(stats):
    return {"kind": "measured", "stats": stats, "reason": None}


def _refused(reason):
    return {"kind": "not_evaluable", "stats": None, "reason": reason}


def _disagree(fit_d, hint_d):
    return (
        "fitted plane reports doubled=%s and normal_hint reports doubled=%s; "
        "the axes disagree, so the patch is not evaluable" % (fit_d, hint_d)
    )


def _prefer_pair(candidate, incumbent):
    """True when candidate replaces incumbent.

    A split of at least 40 mm wins over a shorter kept split. Otherwise the
    taller shorter peak wins, then the larger separation.
    """
    candidate_true = candidate["doubled"] == "true"
    incumbent_true = incumbent["doubled"] == "true"
    if candidate_true != incumbent_true:
        return candidate_true
    if candidate["shorter"] != incumbent["shorter"]:
        return candidate["shorter"] > incumbent["shorter"]
    return candidate["separation_m"] > incumbent["separation_m"]
