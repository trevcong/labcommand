"""Timestamp statistics for bag inspection. Stdlib only; all times in seconds.

Reports facts (offsets, gaps, coverage). Never corrects timestamps.
"""

from __future__ import annotations

import bisect
import math
import statistics
from typing import Dict, List, Optional, Sequence, Tuple


def _q(values: Sequence[float], frac: float) -> float:
    s = sorted(values)
    return s[min(len(s) - 1, max(0, int(round(frac * (len(s) - 1)))))]


def stream_stats(header_t: Sequence[float], receipt_t: Sequence[float],
                 gap_factor: float = 3.0) -> Dict:
    """Header-vs-receipt offset and inter-message timing for one topic.

    ``receipt_t`` is the bag (recorder receive) time, ``header_t`` the
    message header stamp, both in message order.
    """
    if len(header_t) != len(receipt_t):
        raise ValueError("header and receipt lists differ in length")
    n = len(header_t)
    out: Dict = {"count": n}
    if n == 0:
        return out
    offs = [r - h for h, r in zip(header_t, receipt_t)]
    out.update({
        "header_first": header_t[0],
        "header_last": header_t[-1],
        "receipt_first": receipt_t[0],
        "receipt_last": receipt_t[-1],
        "receipt_minus_header_s": {
            "min": min(offs), "median": statistics.median(offs), "max": max(offs),
        },
    })
    if n < 2:
        return out
    dts = [b - a for a, b in zip(header_t, header_t[1:])]
    med = statistics.median(dts)
    gap_limit = gap_factor * med if med > 0 else math.inf
    out.update({
        "header_dt_median_s": med,
        "header_dt_p99_s": _q(dts, 0.99),
        "header_dt_max_s": max(dts),
        "header_rate_hz": (1.0 / med) if med > 0 else None,
        "header_backward_steps": sum(1 for d in dts if d < 0),
        "header_duplicate_steps": sum(1 for d in dts if d == 0),
        "header_gaps_over_%gx_median" % gap_factor: sum(1 for d in dts if d > gap_limit),
        "header_out_of_order_vs_receipt": sum(
            1 for i in range(1, n) if header_t[i] < header_t[i - 1] and receipt_t[i] >= receipt_t[i - 1]
        ),
    })
    return out


def imu_coverage(scans: Sequence[Tuple[float, float]], imu_t: Sequence[float]) -> Dict:
    """For each lidar scan [start, end], is it bracketed by IMU samples?

    FAST-LIO needs an IMU sample at or after the scan end before it will
    process the scan, and uses samples inside the scan to undistort it.
    ``imu_t`` must be sorted ascending (checked).
    """
    if any(b < a for a, b in zip(imu_t, imu_t[1:])):
        raise ValueError("imu_t must be sorted ascending")
    out: Dict = {"scans": len(scans), "imu_samples": len(imu_t)}
    if not scans or not imu_t:
        out["covered"] = 0
        return out
    no_before = no_after = covered = 0
    worst_gap = 0.0
    worst_gap_scan: Optional[int] = None
    spans: List[float] = []
    for i, (s, e) in enumerate(scans):
        spans.append(e - s)
        lo = bisect.bisect_right(imu_t, s) - 1   # last sample <= start
        hi = bisect.bisect_left(imu_t, e)        # first sample >= end
        if lo < 0:
            no_before += 1
        if hi >= len(imu_t):
            no_after += 1
        if lo >= 0 and hi < len(imu_t):
            covered += 1
            seg = imu_t[lo:hi + 1]
            g = max((b - a for a, b in zip(seg, seg[1:])), default=0.0)
            if g > worst_gap:
                worst_gap, worst_gap_scan = g, i
    out.update({
        "covered": covered,
        "scans_without_imu_before_start": no_before,
        "scans_without_imu_after_end": no_after,
        "worst_imu_gap_inside_scan_s": worst_gap,
        "worst_imu_gap_scan_index": worst_gap_scan,
        "scan_span_s": {"min": min(spans), "median": statistics.median(spans), "max": max(spans)},
        "lidar_first_minus_imu_first_s": scans[0][0] - imu_t[0],
        "lidar_last_end_minus_imu_last_s": scans[-1][1] - imu_t[-1],
    })
    return out


def accel_units_guess(acc_norms: Sequence[float]) -> Dict:
    """Median |acc| while presumably near-static. ~9.8 => m/s^2, ~1.0 => g.

    This is a guess from magnitude only and is labelled as such.
    """
    if not acc_norms:
        return {"median_acc_norm": None, "guess": "no samples"}
    m = statistics.median(acc_norms)
    if 8.8 < m < 10.8:
        g = "m/s^2 (magnitude ~9.8)"
    elif 0.9 < m < 1.1:
        g = "g (magnitude ~1.0)"
    else:
        g = "unclear"
    return {"median_acc_norm": m, "guess": g}
