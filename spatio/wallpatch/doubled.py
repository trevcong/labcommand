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
# Split positions for the stepped-band pass guard, as a share of the points.
SPLIT_STEP = 0.01
SPLIT_TURN_DEG = 15.0

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
    "normal_hint. A confident split that only one axis shows is kept as a "
    "failure when its gap is above the separation gate; at or under the gate "
    "the patch is not evaluable, because a gap along an axis a few degrees "
    "off the sheets reads short. If both confident splits fall on opposite "
    "sides of the separation gate, or one axis reports a candidate the other "
    "does not confirm, the patch is not evaluable. A false result on both "
    "axes is a single surface. A patch that would pass both gates is still "
    "not evaluable when the narrowest band holding 80% of the signed "
    "distances to the fitted plane is at least the separation gate wide, "
    "since two surfaces that far apart, each holding at least 20% of the "
    "points, could sit inside it. The same holds after the patch is split in "
    "two, at every 1% of the points with at least 20% on each side and in "
    "split directions every 15 deg across the wall, and the tilt a step "
    "between the parts would cause is taken out of the distances. That "
    "catches two offset sheets that tilt the fitted plane. Results that fail "
    "a gate are kept."
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
        "core_band_m": core_band(values),
        "bin_width_m": BIN_WIDTH_M,
        "n": int(values.size),
        "doubled": decision["doubled"],
        "separation_m": decision["separation_m"],
        "mark_m": decision["mark_m"],
        "hist_counts": counts,
        "hist_edges": edges,
    }


def core_band(values):
    """Width of the narrowest interval holding 1 - MIN_PEAK_MASS of the values.

    Any surface with at least MIN_PEAK_MASS of the points, the share the
    split rule needs, has points inside this interval. Far clutter under
    that share falls outside it, unlike the p2.5 - p97.5 thickness.
    """
    ordered = np.sort(np.asarray(values, dtype=np.float64).reshape(-1))
    n_values = len(ordered)
    keep = int(np.ceil((1.0 - MIN_PEAK_MASS) * n_values - 1e-9))
    keep = min(max(keep, 1), n_values)
    widths = ordered[keep - 1 :] - ordered[: n_values - keep + 1]
    return float(widths.min())


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
    # A split one axis sees and the other misses is measured along an axis
    # that may be a few degrees off the sheets, which shrinks the gap. It can
    # fail the gate. It cannot pass it.
    if fit_d == "false" and hint_d == "true":
        if hint_s is not None and float(hint_s) > float(separation_gate_m):
            return _measured(hint_stats)
        return _refused(_one_axis(fit_d, hint_d, hint_s, separation_gate_m))
    if fit_d == "true" and hint_d == "false":
        if fit_s is not None and float(fit_s) > float(separation_gate_m):
            return _measured(fit_stats)
        return _refused(_one_axis(fit_d, hint_d, fit_s, separation_gate_m))
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


def _one_axis(fit_d, hint_d, separation, separation_gate_m):
    shown = "null" if separation is None else "%.4f m" % separation
    return (
        "fitted plane reports doubled=%s and normal_hint reports doubled=%s; "
        "the one-axis separation %s is not above the %.2f m separation gate, "
        "so the patch is not evaluable" % (fit_d, hint_d, shown, separation_gate_m)
    )


def thick_band_reason(core_band_m, separation_gate_m):
    """Why a would-be pass is refused, or None when the band is thin enough.

    core_band_m is the narrowest interval holding 80% of every signed
    distance to the fitted plane. A band at least as wide as the separation
    gate can hold two surfaces that far apart, whatever the histogram along
    either axis shows. A pooled fit tilted by partly overlapping sheets, or
    a hint a few degrees off, both look like one wide surface. Not a pass.
    """
    if float(core_band_m) < float(separation_gate_m):
        return None
    return (
        "narrowest band holding %d%% of the distances to the fitted plane is "
        "%.4f m wide, at least the %.2f m separation gate; two surfaces that far "
        "apart could sit inside it, so the patch is not evaluable"
        % (
            int(round(100 * (1.0 - MIN_PEAK_MASS))),
            core_band_m,
            separation_gate_m,
        )
    )


def stepped_band(signed, coords):
    """Widest 80% band left after taking out the tilt a step would cause.

    Two sheets that are offset along the normal and only partly overlap tilt
    the pooled fit. The tilt soaks up the offset, so the band around the
    pooled plane stays thin. For each in-plane coordinate in coords, the
    patch is split at every SPLIT_STEP of the points, keeping MIN_PEAK_MASS
    on each side. That is the same floor the split rule uses. The model
    signed = a + b * u + c * [past the split] is fitted, and the tilt b * u
    is removed. On one flat surface b stays near zero and the band does not
    change. Two sheets split near their join give back the tilt that the
    offset caused. The band then shows the offset.
    """
    values = np.asarray(signed, dtype=np.float64).reshape(-1)
    n_values = len(values)
    lo = int(np.ceil(MIN_PEAK_MASS * n_values - 1e-9))
    widest = core_band(values)
    if lo < 1 or n_values - lo < lo:
        return widest
    for coord in coords:
        u_all = np.asarray(coord, dtype=np.float64).reshape(-1)
        order = np.argsort(u_all, kind="stable")
        u = u_all[order]
        r = values[order]
        cum_u = np.concatenate([[0.0], np.cumsum(u)])
        cum_r = np.concatenate([[0.0], np.cumsum(r)])
        sum_uu = float(np.dot(u, u))
        sum_ur = float(np.dot(u, r))
        splits = np.unique(
            np.round(
                np.arange(MIN_PEAK_MASS, 1.0 - MIN_PEAK_MASS + 1e-9, SPLIT_STEP) * n_values
            ).astype(int)
        )
        splits = splits[(splits >= lo) & (splits <= n_values - lo)]
        for k in splits:
            n_after = float(n_values - k)
            u_after = cum_u[-1] - cum_u[k]
            normal_eq = np.array(
                [
                    [n_values, cum_u[-1], n_after],
                    [cum_u[-1], sum_uu, u_after],
                    [n_after, u_after, n_after],
                ]
            )
            rhs = np.array([cum_r[-1], sum_ur, cum_r[-1] - cum_r[k]])
            try:
                _a, slope, _c = np.linalg.solve(normal_eq, rhs)
            except np.linalg.LinAlgError:
                continue
            widest = max(widest, core_band(values - slope * u_all))
    return float(widest)


def stepped_band_reason(stepped_band_m, separation_gate_m):
    """Why a would-be pass is refused, or None when the band stays thin."""
    if float(stepped_band_m) < float(separation_gate_m):
        return None
    return (
        "with the patch split in two and the tilt a step between the parts "
        "would cause taken out, the narrowest band holding %d%% of the "
        "distances is %.4f m wide, at least the %.2f m separation gate; two "
        "offset surfaces that far apart could sit in the patch, so the patch "
        "is not evaluable"
        % (
            int(round(100 * (1.0 - MIN_PEAK_MASS))),
            stepped_band_m,
            separation_gate_m,
        )
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
