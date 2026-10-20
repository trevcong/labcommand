"""Pure-numpy IMU characterization.

The caller's arrays are never written. Reductions allocate their own outputs.

Method
------
Stationary window (1.0 s, step 1.0 s, at least 20 samples spanning >= 0.8 s):

- std(|a|) / median(|a|) <= 0.02. This is scale-free, so it does not assume
  g versus m/s^2. Two percent of gravity is about 0.2 m/s^2.
- std(|ω|) <= 0.05 rad/s and median(|ω|) <= 0.20 rad/s. The median gate
  rejects a steady turn, which has a small standard deviation and is not
  stationary. When the gyro-unit verdict is deg/s, those two numbers are
  multiplied by 180/π so the same physical gate is applied in the message
  unit. When the gyro unit is undetermined, the gate is left unscaled, so
  a deg/s stream is not treated as stationary and the accel unit is not
  guessed.

Accel unit is the median |a| over samples inside stationary windows:
0.9–1.1 → g, 9.3–10.3 → m/s^2, otherwise undetermined.

Gravity. A stationary accelerometer measures specific force, the support
force, which points opposite gravity (up). The reported direction is the
mean acceleration vector, normalized. Dominant +z within 15° means the
published +z axis points up ("sensor appears upright"). Dominant -z within
15° means that axis points down ("sensor appears inverted"). Any other
case is "sensor appears tilted N°" with N the angle from +z.
FAST-LIO uses the same sign: gravity is initialized to -mean_acc/|mean_acc| * 9.81.

Gyro unit. On the same 1 s grid, compare trapezoidal ∑|ω|Δt with the
geodesic rotation angle of /Odometry over the odometry samples that fall
in the window. Magnitudes only. A window is used when the odometry span
covers at least half the window, the angle is between 0.15 and 2.5 rad
(provisional; below π, so the quaternion short-arc can be the whole turn),
the sum of the short-arc steps between consecutive odometry poses is also
at most 2.5 rad (a wrapped revolution cannot hide in the endpoint angle).
That sum is the turn only when no longer angle (the short-arc plus whole
turns) fits a unit band as closely as the short-arc does. When a longer
angle fits as well or better, the poses do not show which turn happened
and the window is refused. |∫ω dt| / ∫|ω| dt >= 0.70 (the body
rate does not reverse), and the IMU samples inside that odometry interval
actually cover it. Coverage means those samples span at least 80% of the
interval and no step, including either uncovered end, is longer than the
provisional 1 s gap. Median ratio 0.75–1.25 → rad/s. The same band times
180/π → deg/s (57.3 is that constant rounded). Otherwise undetermined.
Nothing is inferred from a noise floor. /Odometry was produced by FAST-LIO
using this IMU, so the ratio is support, not proof.

Clock epochs. Samples are ordered by header time only inside one clock
run. A sustained rewind is a separate epoch whether or not its time range
overlaps an earlier run, and it is not sorted onto that run. Finite headers
are split before non-finite vectors or quaternions are removed, so a NaN
measurement cannot erase the rewind its stamp shows. Epochs are paired
with odometry in read order. One sample out of order stays on one timeline.
An epoch whose measurements are all non-finite is kept, so a later run
does not slide onto the wrong odometry.

Bias and noise are the per-axis mean and the sample standard deviation
(ddof 1) on the stationary samples, in the message's native unit.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

SCHEMA = "spatio.imucheck/1"

WINDOW_S = 1.0
WINDOW_STEP_S = 1.0
MIN_SAMPLES = 20
MIN_SPAN_FRACTION = 0.8

# Scale-free accel quietness. See the module docstring.
ACCEL_REL_STD_MAX = 0.02
# Native angular unit. Sized for a resting rad/s IMU.
GYRO_MAG_STD_MAX = 0.05
GYRO_MAG_MEDIAN_MAX = 0.20

ACCEL_G_LO = 0.9
ACCEL_G_HI = 1.1
ACCEL_MPS2_LO = 9.3
ACCEL_MPS2_HI = 10.3

# Fraction either side of 1 and of 180/π. Wide enough for lidar updates
# and a small unremoved bias, narrow enough that 2 or 30 cannot pass.
GYRO_RATIO_TOL = 0.25
DEG_PER_RAD = 180.0 / math.pi  # 57.2957..., the task's 57.3

# Provisional (approved for now, revisit after real data). 2.5 rad is below
# π, so a short-arc is the whole turn only when the path stays under this cap.
# The cap applies to the endpoint and to the sum of intermediate steps.
MIN_MOTION_ANGLE_RAD = 0.15
MAX_MOTION_ANGLE_RAD = 2.5
MIN_COHERENCE = 0.70
MIN_ODOM_SPAN_FRACTION = 0.50

# Delegate-approved gap. Anything longer is not a measurement of that turn.
# provisional (approved for now, revisit after real data)
PROVISIONAL_LABEL = "provisional (approved for now, revisit after real data)"
MAX_IMU_GAP_S = 1.0
# Local provisional gate, same 0.80 fraction as MIN_SPAN_FRACTION. The 1 s
# gap is wider than the hole that produced a false rad/s verdict (0.01 s of
# gyro against 0.61 s of rotation), so the span of the samples inside the
# odometry interval has to cover this fraction of that interval.
MIN_IMU_COVERAGE_FRACTION = 0.80
# User-typed durations. One parser rejects non-finite, non-positive, and
# values past this. Unix header stamps are not parsed here; they are larger
# than this and are still valid sample times.
ABSURD_DURATION_S = 1.0e6

UPRIGHT_MAX_DEG = 15.0

CAVEAT = (
    "Odometry is FAST-LIO output that itself used this IMU, so a ratio "
    "near 1 or near 57.3 is supporting evidence, not proof of the gyro unit."
)

_AXIS_NAMES = ("x", "y", "z")


def characterize(
    t_s,
    accel,
    gyro,
    frame_ids=None,
    orientation_covariance=None,
    angular_velocity_covariance=None,
    linear_acceleration_covariance=None,
    odom_t=None,
    odom_quat=None,
    topic="",
):
    """Build the spatio.imucheck/1 report. Does not modify the inputs.

    t_s, accel (N, 3), gyro (N, 3) are the Imu header time and vectors.
    odom_t and odom_quat (M, 4) xyzw are optional. Covariance arguments
    are (N, 9) row-major arrays, or None.
    """
    t = np.asarray(t_s, dtype=np.float64)
    acc = np.asarray(accel, dtype=np.float64)
    gyr = np.asarray(gyro, dtype=np.float64)
    if t.ndim != 1:
        raise ValueError("t_s must be one-dimensional")
    if acc.ndim != 2 or acc.shape[1] != 3:
        raise ValueError("accel must have shape (N, 3)")
    if gyr.ndim != 2 or gyr.shape[1] != 3:
        raise ValueError("gyro must have shape (N, 3)")
    if not (len(t) == len(acc) == len(gyr)):
        raise ValueError("t_s, accel, and gyro must have the same length")

    n = int(len(t))
    # Header rate, intervals, and backward steps use the stamp only. A NaN
    # vector must not halve the rate or hide a backward stamp. Physics
    # still requires a finite header and finite vectors.
    header_finite = np.isfinite(t) if n else np.zeros(0, dtype=bool)
    vector_finite = header_finite
    if n:
        vector_finite = (
            header_finite
            & np.all(np.isfinite(acc), axis=1)
            & np.all(np.isfinite(gyr), axis=1)
        )
    n_dropped = int(n - int(np.count_nonzero(vector_finite)))

    rate, intervals, n_backward = _rate_and_intervals(t, header_finite)

    if n:
        # Finite headers decide the epochs. Filtering NaN vectors first
        # drops the stamps that show a rewind.
        epochs = _epoch_series(t, acc, gyr)
    else:
        epochs = [(
            np.zeros(0, dtype=np.float64),
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 3), dtype=np.float64),
        )]

    odom_epochs = _odom_epochs(odom_t, odom_quat)
    # Gyro unit does not use the stationary mask, so it can set the unit of
    # the stationary gyro gate without a circular argument.
    gyro_unit = _gyro_from_epochs(epochs, odom_epochs)
    gyro_gate_scale = DEG_PER_RAD if gyro_unit["verdict"] == "deg/s" else 1.0
    windows, diag, stat_acc, stat_gyr = _stationary_across(
        epochs,
        gyro_std_max=GYRO_MAG_STD_MAX * gyro_gate_scale,
        gyro_median_max=GYRO_MAG_MEDIAN_MAX * gyro_gate_scale,
    )
    if n and np.any(header_finite):
        header_times = t[header_finite]
        diag["t_first"] = float(np.min(header_times))
        diag["t_last"] = float(np.max(header_times))
    else:
        diag["t_first"] = None
        diag["t_last"] = None
    stat_mask = np.ones(len(stat_acc), dtype=bool)
    accel_norm_stats, accel_unit = _accel_unit(stat_acc, stat_mask)
    gravity = _gravity(stat_acc, stat_mask)
    gyro_bias, gyro_noise, accel_noise = _bias_and_noise(stat_acc, stat_gyr, stat_mask)

    covariance = {
        "orientation": _classify_covariance(orientation_covariance),
        "angular_velocity": _classify_covariance(angular_velocity_covariance),
        "linear_acceleration": _classify_covariance(linear_acceleration_covariance),
        "sensor_msgs_note": (
            "sensor_msgs/Imu says a covariance of all zeros means unknown, "
            "and a first element of -1 means that estimate is unavailable. "
            "all_zero here is the published pattern, not a measured variance."
        ),
    }

    frames: List[str] = []
    if frame_ids:
        for item in frame_ids:
            text = str(item)
            if text not in frames:
                frames.append(text)

    findings = _findings(
        n=n,
        n_dropped=n_dropped,
        n_backward=n_backward,
        rate=rate,
        intervals=intervals,
        windows=windows,
        diag=diag,
        accel_unit=accel_unit,
        accel_norm_stats=accel_norm_stats,
        gravity=gravity,
        gyro_unit=gyro_unit,
        gyro_bias=gyro_bias,
        gyro_noise=gyro_noise,
        accel_noise=accel_noise,
        covariance=covariance,
    )

    return {
        "schema": SCHEMA,
        "topic": topic,
        "frame_ids": frames,
        "count": n,
        "rate_hz": rate,
        "interval_s": intervals,
        "stationary_windows": [{"t0_s": float(a), "t1_s": float(b)} for a, b in windows],
        "accel_norm_stationary": accel_norm_stats,
        "accel_unit": accel_unit,
        "gravity": gravity,
        "gyro_unit": gyro_unit,
        "gyro_bias": gyro_bias,
        "gyro_noise_std": gyro_noise,
        "accel_noise_std": accel_noise,
        "covariance_published": covariance,
        "findings": findings,
    }


def _rate_and_intervals(t, finite):
    empty = {"p5": None, "p50": None, "p95": None, "max": None}
    if t.size == 0 or not np.any(finite):
        return None, empty, 0
    raw = t[finite]
    if raw.size >= 2:
        read_dt = np.diff(raw)
        n_backward = int(np.count_nonzero(read_dt < 0))
    else:
        n_backward = 0
    ordered = np.sort(raw)
    if ordered.size < 2:
        return None, empty, n_backward
    span = float(ordered[-1] - ordered[0])
    rate = None
    if span > 0.0:
        rate = float((ordered.size - 1) / span)
    dt = np.diff(ordered)
    finite_dt = dt[np.isfinite(dt)]
    if finite_dt.size == 0:
        return rate, empty, n_backward
    return (
        rate,
        {
            "p5": float(np.percentile(finite_dt, 5)),
            "p50": float(np.percentile(finite_dt, 50)),
            "p95": float(np.percentile(finite_dt, 95)),
            "max": float(np.max(finite_dt)),
        },
        n_backward,
    )


def _window_bounds(t_srt):
    """Yield (t0, t1, i0, i1, long_enough) on a 1 s grid.

    An empty window jumps to the next sample. A bad stamp years away from
    the rest must not make this loop once per second across the gap.
    """
    if t_srt.size == 0:
        return
    last = float(t_srt[-1])
    start = float(t_srt[0])
    # A corrupt stamp years away must not walk one second at a time forever.
    # 200_000 steps covers more than two days of 1 s windows.
    guard = 0
    limit = max(int(t_srt.size) + 2, 200000)
    while start <= last and guard < limit:
        guard += 1
        stop = start + WINDOW_S
        i0 = int(np.searchsorted(t_srt, start, side="left"))
        i1 = int(np.searchsorted(t_srt, stop, side="left"))
        if i1 <= i0:
            if i0 >= t_srt.size:
                break
            nxt = float(t_srt[i0])
            if nxt <= start:
                nxt = start + WINDOW_STEP_S
            start = nxt
            continue
        span = float(t_srt[i1 - 1] - t_srt[i0])
        long_enough = (i1 - i0) >= MIN_SAMPLES and span >= MIN_SPAN_FRACTION * WINDOW_S
        yield start, stop, i0, i1, long_enough
        start += WINDOW_STEP_S


def _stationary_windows(t_srt, acc_srt, gyr_srt, gyro_std_max=GYRO_MAG_STD_MAX, gyro_median_max=GYRO_MAG_MEDIAN_MAX):
    """Return merged (t0, t1) pairs and a small diagnostic dict."""
    passed: List[Tuple[float, float]] = []
    gyro_stds: List[float] = []
    accel_rels: List[float] = []
    n_windows = 0
    n_fail_accel = 0
    n_fail_gyro_std = 0
    n_fail_gyro_median = 0
    n_too_short = 0

    gmag = np.linalg.norm(gyr_srt, axis=1) if len(gyr_srt) else np.zeros(0)
    anorm = np.linalg.norm(acc_srt, axis=1) if len(acc_srt) else np.zeros(0)

    for start, stop, i0, i1, long_enough in _window_bounds(t_srt):
        if not long_enough:
            n_too_short += 1
            continue
        n_windows += 1
        gm = gmag[i0:i1]
        am = anorm[i0:i1]
        g_std = float(np.std(gm))
        g_med = float(np.median(gm))
        a_med = float(np.median(am))
        a_std = float(np.std(am))
        if a_med > 1e-6:
            a_rel = a_std / a_med
        else:
            a_rel = float("inf")
        gyro_stds.append(g_std)
        accel_rels.append(a_rel)
        accel_ok = a_rel <= ACCEL_REL_STD_MAX
        gyro_std_ok = g_std <= gyro_std_max
        gyro_med_ok = g_med <= gyro_median_max
        if accel_ok and gyro_std_ok and gyro_med_ok:
            passed.append((start, stop))
        else:
            if not accel_ok:
                n_fail_accel += 1
            if not gyro_std_ok:
                n_fail_gyro_std += 1
            if not gyro_med_ok:
                n_fail_gyro_median += 1

    merged = _merge(passed)
    diag = {
        "n_windows": n_windows,
        "n_pass": len(passed),
        "n_fail_accel": n_fail_accel,
        "n_fail_gyro_std": n_fail_gyro_std,
        "n_fail_gyro_median": n_fail_gyro_median,
        "n_too_short": n_too_short,
        "gyro_std_max": float(gyro_std_max),
        "gyro_median_max": float(gyro_median_max),
        "gyro_mag_std_p10": _percentile(gyro_stds, 10),
        "gyro_mag_std_p50": _percentile(gyro_stds, 50),
        "accel_rel_std_p10": _percentile(accel_rels, 10),
        "accel_rel_std_p50": _percentile(accel_rels, 50),
        "_gyro_stds": gyro_stds,
        "_accel_rels": accel_rels,
    }
    return merged, diag


def _percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def _combine_stationary_diags(diags):
    """Sum window counts and recompute percentiles on the pooled windows."""
    keys = (
        "n_windows",
        "n_pass",
        "n_fail_accel",
        "n_fail_gyro_std",
        "n_fail_gyro_median",
        "n_too_short",
    )
    gyro_stds: List[float] = []
    accel_rels: List[float] = []
    combined = {key: 0 for key in keys}
    for diag in diags:
        for key in keys:
            combined[key] += diag[key]
        gyro_stds.extend(diag["_gyro_stds"])
        accel_rels.extend(diag["_accel_rels"])
    first = diags[0]
    combined["gyro_std_max"] = first["gyro_std_max"]
    combined["gyro_median_max"] = first["gyro_median_max"]
    combined["gyro_mag_std_p10"] = _percentile(gyro_stds, 10)
    combined["gyro_mag_std_p50"] = _percentile(gyro_stds, 50)
    combined["accel_rel_std_p10"] = _percentile(accel_rels, 10)
    combined["accel_rel_std_p50"] = _percentile(accel_rels, 50)
    return combined


def _stationary_across(epochs, gyro_std_max, gyro_median_max):
    """Stationary windows per clock epoch. Epochs are not merged."""
    windows: List[Tuple[float, float]] = []
    diags = []
    acc_parts = []
    gyr_parts = []
    for t_srt, acc_srt, gyr_srt in epochs:
        epoch_windows, diag = _stationary_windows(
            t_srt,
            acc_srt,
            gyr_srt,
            gyro_std_max=gyro_std_max,
            gyro_median_max=gyro_median_max,
        )
        windows.extend(epoch_windows)
        diags.append(diag)
        mask = _mask_windows(t_srt, epoch_windows)
        if np.any(mask):
            acc_parts.append(acc_srt[mask])
            gyr_parts.append(gyr_srt[mask])
    if acc_parts:
        stat_acc = np.concatenate(acc_parts, axis=0)
        stat_gyr = np.concatenate(gyr_parts, axis=0)
    else:
        stat_acc = np.zeros((0, 3), dtype=np.float64)
        stat_gyr = np.zeros((0, 3), dtype=np.float64)
    return windows, _combine_stationary_diags(diags), stat_acc, stat_gyr


def _merge(windows: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    merged: List[Tuple[float, float]] = []
    for t0, t1 in windows:
        if merged and t0 <= merged[-1][1] + 1e-9:
            prev0, prev1 = merged[-1]
            merged[-1] = (prev0, max(prev1, t1))
        else:
            merged.append((t0, t1))
    return merged


def _mask_windows(t_srt, windows: Sequence[Tuple[float, float]]):
    mask = np.zeros(len(t_srt), dtype=bool)
    for t0, t1 in windows:
        i0 = int(np.searchsorted(t_srt, t0, side="left"))
        i1 = int(np.searchsorted(t_srt, t1, side="left"))
        mask[i0:i1] = True
    return mask


def _accel_unit(acc_srt, mask):
    empty = {"median": None, "p5": None, "p95": None}
    if not np.any(mask):
        return empty, {
            "verdict": "undetermined",
            "evidence": "No stationary window. Accel unit was not guessed.",
        }
    norms = np.linalg.norm(acc_srt[mask], axis=1)
    median = float(np.median(norms))
    stats = {
        "median": median,
        "p5": float(np.percentile(norms, 5)),
        "p95": float(np.percentile(norms, 95)),
    }
    if ACCEL_G_LO <= median <= ACCEL_G_HI:
        verdict = "g"
        band = "0.9–1.1"
    elif ACCEL_MPS2_LO <= median <= ACCEL_MPS2_HI:
        verdict = "m/s^2"
        band = "9.3–10.3"
    else:
        verdict = "undetermined"
        band = None
    if band is None:
        evidence = (
            "Median stationary accel norm is {0:.6g}, outside 0.9–1.1 and "
            "outside 9.3–10.3.".format(median)
        )
    else:
        evidence = (
            "Median stationary accel norm is {0:.6g}, inside {1}.".format(median, band)
        )
    return stats, {"verdict": verdict, "evidence": evidence}


def _gravity(acc_srt, mask):
    unknown = {
        "mean_dir_sensor": None,
        "dominant_axis": None,
        "angle_from_axis_deg": None,
        "mount_sentence": "undetermined",
    }
    if not np.any(mask):
        return unknown
    mean = np.mean(acc_srt[mask], axis=0)
    norm = float(np.linalg.norm(mean))
    if not math.isfinite(norm) or norm < 1e-8:
        return unknown
    direction = mean / norm
    # Ties prefer z, then y, then x.
    order = (2, 1, 0)
    idx = order[0]
    for candidate in order[1:]:
        if abs(float(direction[candidate])) > abs(float(direction[idx])):
            idx = candidate
    sign = 1.0 if float(direction[idx]) >= 0.0 else -1.0
    axis_name = ("+" if sign > 0.0 else "-") + _AXIS_NAMES[idx]
    axis_vec = np.zeros(3, dtype=np.float64)
    axis_vec[idx] = sign
    cosine = float(np.clip(np.dot(direction, axis_vec), -1.0, 1.0))
    angle = math.degrees(math.acos(cosine))
    if axis_name == "+z" and angle <= UPRIGHT_MAX_DEG:
        sentence = "sensor appears upright"
    elif axis_name == "-z" and angle <= UPRIGHT_MAX_DEG:
        sentence = "sensor appears inverted"
    else:
        tilt = math.degrees(math.acos(float(np.clip(direction[2], -1.0, 1.0))))
        sentence = "sensor appears tilted {0:.1f}°".format(tilt)
    return {
        "mean_dir_sensor": [float(direction[0]), float(direction[1]), float(direction[2])],
        "dominant_axis": axis_name,
        "angle_from_axis_deg": float(angle),
        "mount_sentence": sentence,
    }


def _bias_and_noise(acc_srt, gyr_srt, mask):
    if int(np.count_nonzero(mask)) < 2:
        return None, None, None
    acc = acc_srt[mask]
    gyr = gyr_srt[mask]
    bias = [float(v) for v in np.mean(gyr, axis=0)]
    gyro_noise = [float(v) for v in np.std(gyr, axis=0, ddof=1)]
    accel_noise = [float(v) for v in np.std(acc, axis=0, ddof=1)]
    return bias, gyro_noise, accel_noise


def _segment_bounds(t):
    """Read-order [start, end) runs of one clock.

    A stamp that moves backward starts a new run, except when the next
    stamp continues the time already reached (one sample out of order)
    or the backward stamp is the last sample. Equal stamps stay together.
    """
    n = int(len(t))
    if n == 0:
        return []
    bounds = []
    start = 0
    i = 1
    while i < n:
        if t[i] < t[i - 1]:
            if i + 1 >= n or t[i + 1] >= t[i - 1]:
                i += 1
                continue
            bounds.append((start, i))
            start = i
        i += 1
    bounds.append((start, n))
    return bounds


def _empty_columns(columns):
    empties = []
    for col in columns:
        arr = np.asarray(col)
        if arr.ndim == 1:
            empties.append(np.zeros(0, dtype=np.float64))
        else:
            empties.append(np.zeros((0,) + arr.shape[1:], dtype=np.float64))
    return empties


def _epochs_keeping_resets(times, columns, row_ok):
    """Split finite headers into clock epochs, then apply `row_ok`.

    Non-finite headers are not stamps, so they are removed before the
    split. `row_ok` drops non-finite measurements inside an epoch. Doing
    it the other way erases a rewind whose only stamps are on those rows.
    An epoch that then has no rows is kept so later epochs stay paired.
    Each returned epoch is `(times, [columns...])`, sorted by time.
    """
    times = np.asarray(times, dtype=np.float64)
    columns = [np.asarray(col) for col in columns]
    row_ok = np.asarray(row_ok, dtype=bool)
    if times.size:
        header_ok = np.isfinite(times)
        if not np.all(header_ok):
            times = times[header_ok]
            columns = [col[header_ok] for col in columns]
            row_ok = row_ok[header_ok]
    n = int(times.shape[0])
    if n == 0:
        return []
    bounds = _segment_bounds(times)
    if len(bounds) <= 1:
        bounds = [(0, n)]
    epochs = []
    for start, stop in bounds:
        keep = row_ok[start:stop]
        kept_t = times[start:stop][keep]
        if kept_t.size == 0:
            epochs.append((np.zeros(0, dtype=np.float64), _empty_columns(columns)))
            continue
        kept = [col[start:stop][keep] for col in columns]
        order = np.argsort(kept_t, kind="mergesort")
        epochs.append((kept_t[order], [col[order] for col in kept]))
    return epochs


def _epoch_series(t, acc, gyr):
    """Sorted (t, accel, gyro) epochs.

    A sustained rewind is its own epoch even when the time ranges do not
    overlap. Sorting a disjoint reset onto the earlier run lets one window
    bridge an odometry origin change. Headers are split before NaN vectors
    are removed.
    """
    empty = (
        np.zeros(0, dtype=np.float64),
        np.zeros((0, 3), dtype=np.float64),
        np.zeros((0, 3), dtype=np.float64),
    )
    if len(t) == 0:
        return [empty]
    row_ok = np.all(np.isfinite(acc), axis=1) & np.all(np.isfinite(gyr), axis=1)
    raw = _epochs_keeping_resets(t, (acc, gyr), row_ok)
    if not raw:
        return [empty]
    return [(kept_t, cols[0], cols[1]) for kept_t, cols in raw]


def _odom_epochs(odom_t, odom_quat):
    """Sorted (t, xyzw) epochs, or None when there is no usable odometry.

    Same sustained-rewind rule as the IMU series. Finite headers are split
    before non-finite quaternions are removed. A disjoint reset stays its
    own epoch. An epoch with no finite quaternion is kept when another
    epoch still has poses, so pairing does not slide.
    """
    if odom_t is None or odom_quat is None:
        return None
    ot = np.asarray(odom_t, dtype=np.float64)
    oq = np.asarray(odom_quat, dtype=np.float64)
    if ot.ndim != 1:
        raise ValueError("odom_t must be one-dimensional")
    if oq.ndim != 2 or oq.shape[1] != 4:
        raise ValueError("odom_quat must have shape (M, 4) xyzw")
    if len(ot) != len(oq):
        raise ValueError("odom_t and odom_quat must have the same length")
    if len(ot) == 0:
        return None
    row_ok = np.all(np.isfinite(oq), axis=1)
    raw = _epochs_keeping_resets(ot, (oq,), row_ok)
    if not raw or all(len(kept_t) == 0 for kept_t, _cols in raw):
        return None
    return [(kept_t, cols[0]) for kept_t, cols in raw]


def _rotation_angle_rad(q_a, q_b) -> Optional[float]:
    """Geodesic angle in radians between two xyzw quaternions."""
    a = np.asarray(q_a, dtype=np.float64)
    b = np.asarray(q_b, dtype=np.float64)
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na == 0.0 or nb == 0.0 or not math.isfinite(na) or not math.isfinite(nb):
        return None
    a = a / na
    b = b / nb
    x1, y1, z1, w1 = -float(a[0]), -float(a[1]), -float(a[2]), float(a[3])
    x2, y2, z2, w2 = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
    y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
    z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1
    w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
    vnorm = math.sqrt(x * x + y * y + z * z)
    return 2.0 * math.atan2(vnorm, abs(w))


def _accumulated_turn_rad(quats) -> Optional[float]:
    """Sum of short-arc steps between consecutive xyzw poses.

    The endpoint short-arc hides whole revolutions. The sum does not,
    when each step is the whole turn. A step whose gyro path also matches
    a longer angle is not that turn; the caller refuses the window.
    A sum above the provisional 2.5 rad cap is not a usable turn.
    """
    total = 0.0
    count = len(quats)
    for index in range(count - 1):
        step = _rotation_angle_rad(quats[index], quats[index + 1])
        if step is None:
            return None
        total += step
    return total


def parse_positive_seconds(value, name):
    """Accept one user-typed duration in seconds.

    This is the only check for those numbers. It rejects text that is not
    a number, and any value that is not finite, not strictly positive, or
    larger than ABSURD_DURATION_S.
    """
    if isinstance(value, str):
        text = value.strip()
        try:
            number = float(text)
        except ValueError as exc:
            raise ValueError(
                "{0} must be a positive finite number of seconds".format(name)
            ) from exc
    else:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "{0} must be a positive finite number of seconds".format(name)
            ) from exc
    if (
        not math.isfinite(number)
        or number <= 0.0
        or number > ABSURD_DURATION_S
    ):
        raise ValueError(
            "{0} must be a positive finite number of seconds at most {1:g}".format(
                name, ABSURD_DURATION_S
            )
        )
    return number


def _imu_slice(t_s, t_a, t_b):
    i0 = int(np.searchsorted(t_s, t_a, side="left"))
    i1 = int(np.searchsorted(t_s, t_b, side="right"))
    return i0, i1


def _imu_interval_problem(t_s, t_a, t_b):
    """Why the IMU samples do not support a ratio over [t_a, t_b].

    Returns None when they do. 'gap' means a step or an uncovered end is
    longer than the provisional 1 s gap. 'coverage' means the samples that
    are inside the interval span less than MIN_IMU_COVERAGE_FRACTION of it,
    or the times themselves are not usable. One function owns both gates.
    """
    if not math.isfinite(t_a) or not math.isfinite(t_b):
        return "coverage"
    duration = float(t_b - t_a)
    if not math.isfinite(duration) or duration <= 0.0:
        return "coverage"
    i0, i1 = _imu_slice(t_s, t_a, t_b)
    if i1 - i0 < 2:
        return "coverage"
    ts = t_s[i0:i1]
    if not np.all(np.isfinite(ts)):
        return "coverage"
    gaps = [float(ts[0] - t_a), float(t_b - ts[-1])]
    gaps.extend(float(v) for v in np.diff(ts))
    for gap in gaps:
        if not math.isfinite(gap) or gap < -1e-9:
            return "coverage"
        if gap > MAX_IMU_GAP_S:
            return "gap"
    covered = float(ts[-1] - ts[0])
    if covered < MIN_IMU_COVERAGE_FRACTION * duration:
        return "coverage"
    return None


def _integrate(t_s, gyro, t_a, t_b):
    """Trapezoidal ∫|ω| dt and ∫ω dt for samples inside [t_a, t_b].

    Returns None when those samples do not cover the interval. A short
    burst at one end must not be compared with the whole rotation.
    """
    if _imu_interval_problem(t_s, t_a, t_b) is not None:
        return None
    i0, i1 = _imu_slice(t_s, t_a, t_b)
    ts = t_s[i0:i1]
    g = gyro[i0:i1]
    dt = np.diff(ts)
    positive = dt > 0.0
    if not np.any(positive):
        return None
    mag = np.linalg.norm(g, axis=1)
    path_steps = 0.5 * (mag[:-1] + mag[1:]) * dt
    vec_steps = 0.5 * (g[:-1] + g[1:]) * dt[:, None]
    path = float(np.sum(path_steps[positive]))
    vec = np.sum(vec_steps[positive], axis=0)
    if not math.isfinite(path) or path <= 0.0:
        return None
    return path, vec


def _message_unit_path(t_s, gyro, t_a, t_b):
    """∫|ω| dt in message units, or None when the interval is not covered.

    A covered interval with no motion returns 0. That step cannot hide a
    revolution. `_integrate` returns None in that case because a ratio
    needs a positive path.
    """
    if _imu_interval_problem(t_s, t_a, t_b) is not None:
        return None
    integrated = _integrate(t_s, gyro, t_a, t_b)
    if integrated is None:
        return 0.0
    return float(integrated[0])


def _band_center_distance(ratio):
    """|log(ratio / band center)| for a ratio that sits in a unit band.

    None when the ratio is outside both bands. The centers are 1 and
    180/π. Closeness uses that log distance, not a new cutoff.
    """
    if not math.isfinite(ratio) or ratio <= 0.0:
        return None
    lo = 1.0 - GYRO_RATIO_TOL
    hi = 1.0 + GYRO_RATIO_TOL
    best = None
    if lo <= ratio <= hi:
        best = abs(math.log(ratio))
    if (DEG_PER_RAD * lo) <= ratio <= (DEG_PER_RAD * hi):
        dist = abs(math.log(ratio / DEG_PER_RAD))
        if best is None or dist < best:
            best = dist
    return best


def _step_can_hide_a_revolution(path, theta):
    """True when a longer angle fits a unit band as well as the short-arc.

    The longer angles are the short-arc plus whole turns, either way.
    They use the same rad/s and deg/s bands as the verdict. A longer fit
    competes only when it is at least as close to its band center as the
    short-arc is. A short-arc that is the closer fit is the turn.
    """
    if not math.isfinite(path) or not math.isfinite(theta) or theta < 0.0:
        return True
    if path <= 0.0:
        return False
    if theta > 0.0:
        short_dist = _band_center_distance(path / theta)
    else:
        short_dist = None
    # Largest angle still inside the rad/s band. The deg/s band only
    # matches smaller angles, so it cannot extend this limit.
    max_angle = path / (1.0 - GYRO_RATIO_TOL)
    two_pi = 2.0 * math.pi
    k = 1
    while k < 10000:
        base = two_pi * k
        if base - theta > max_angle:
            return False
        for angle in (base - theta, base + theta):
            if angle <= theta or angle > max_angle:
                continue
            dist = _band_center_distance(path / angle)
            if dist is None:
                continue
            if short_dist is None or dist <= short_dist:
                return True
        k += 1
    return True


def _window_steps_hide_revolution(t_s, gyro, odom_t, odom_q):
    """Whether the short-arc sum is not the turn.

    True: some step's gyro path matches a longer angle, so a revolution
    between those poses is not observable. False: every covered step is
    only the short-arc. None: a step is not covered and none of the covered
    steps hides a revolution; the window coverage check owns that.
    """
    deferred = False
    last = len(odom_t) - 1
    for index in range(last):
        theta = _rotation_angle_rad(odom_q[index], odom_q[index + 1])
        if theta is None:
            return True
        path = _message_unit_path(
            t_s, gyro, float(odom_t[index]), float(odom_t[index + 1])
        )
        if path is None:
            deferred = True
            continue
        if _step_can_hide_a_revolution(path, theta):
            return True
    if deferred:
        return None
    return False


def _empty_gyro_stats():
    return {
        "max_angle": 0.0,
        "n_with_pair": 0,
        "n_angle_in_range": 0,
        "n_coverage_fail": 0,
        "n_gap_fail": 0,
        "n_coherence_fail": 0,
        "n_wrapped": 0,
        "n_ambiguous": 0,
    }


def _add_gyro_stats(total, part):
    for key, value in part.items():
        if key == "max_angle":
            if value > total[key]:
                total[key] = value
        else:
            total[key] += value


def _collect_gyro_windows(t_srt, gyr_srt, odom_t, odom_q):
    """Ratio samples for one IMU epoch against one odometry epoch."""
    ratios: List[float] = []
    stats = _empty_gyro_stats()
    for start, stop, _i0, _i1, long_enough in _window_bounds(t_srt):
        if not long_enough:
            continue
        j0 = int(np.searchsorted(odom_t, start, side="left"))
        j1 = int(np.searchsorted(odom_t, stop, side="left"))
        if j1 - j0 < 2:
            continue
        t_a = float(odom_t[j0])
        t_b = float(odom_t[j1 - 1])
        if (t_b - t_a) < MIN_ODOM_SPAN_FRACTION * WINDOW_S:
            continue
        angle = _rotation_angle_rad(odom_q[j0], odom_q[j1 - 1])
        if angle is None:
            continue
        accumulated = _accumulated_turn_rad(odom_q[j0:j1])
        if accumulated is None:
            continue
        stats["n_with_pair"] += 1
        # A sparse pair can hide whole turns inside one short-arc. Refuse
        # the window when a longer angle fits a unit band as closely as
        # the short-arc does.
        hidden = _window_steps_hide_revolution(
            t_srt, gyr_srt, odom_t[j0:j1], odom_q[j0:j1]
        )
        if hidden is True:
            stats["n_ambiguous"] += 1
            continue
        # The path length is the rotation the cap is about. The endpoint
        # short-arc is only the denominator once that path is in range.
        if accumulated > stats["max_angle"]:
            stats["max_angle"] = accumulated
        if accumulated > MAX_MOTION_ANGLE_RAD:
            stats["n_wrapped"] += 1
            continue
        if angle < MIN_MOTION_ANGLE_RAD or angle > MAX_MOTION_ANGLE_RAD:
            continue
        stats["n_angle_in_range"] += 1
        reason = _imu_interval_problem(t_srt, t_a, t_b)
        if reason == "gap":
            stats["n_gap_fail"] += 1
            continue
        if reason is not None:
            stats["n_coverage_fail"] += 1
            continue
        integrated = _integrate(t_srt, gyr_srt, t_a, t_b)
        if integrated is None:
            continue
        path, vec = integrated
        coherence = float(np.linalg.norm(vec) / path)
        if coherence < MIN_COHERENCE:
            stats["n_coherence_fail"] += 1
            continue
        ratios.append(path / angle)
    return ratios, stats


def _ratios_to_verdict(ratios, stats, prefix):
    base = {
        "verdict": "undetermined",
        "ratio": None,
        "windows_used": 0,
        "evidence": "",
        "caveat": CAVEAT,
    }
    if not ratios:
        if stats["n_with_pair"] == 0:
            detail = "No 1 s window contained two usable odometry poses."
        else:
            detail = (
                "No motion window met the gates (odometry rotation between "
                "{0:.2f} and {1:.1f} rad, including the sum of intermediate "
                "odometry steps, coherence >= {2:.2f}, odometry span "
                ">= {3:.0%} of the window, IMU samples spanning >= {4:.0%} of "
                "the odometry interval, no gap above {5:g} s). These limits "
                "are {6}. Windows with a pose pair: {7}. Of those, {8} "
                "had a rotation in range, {9} failed because intermediate "
                "steps exceeded the rotation limit, {10} failed because a "
                "turn step was not observable, {11} failed IMU coverage, "
                "{12} failed the gap limit, and {13} failed coherence. Largest "
                "1 s odometry rotation was {14:.4f} rad.".format(
                    MIN_MOTION_ANGLE_RAD,
                    MAX_MOTION_ANGLE_RAD,
                    MIN_COHERENCE,
                    MIN_ODOM_SPAN_FRACTION,
                    MIN_IMU_COVERAGE_FRACTION,
                    MAX_IMU_GAP_S,
                    PROVISIONAL_LABEL,
                    stats["n_with_pair"],
                    stats["n_angle_in_range"],
                    stats["n_wrapped"],
                    stats["n_ambiguous"],
                    stats["n_coverage_fail"],
                    stats["n_gap_fail"],
                    stats["n_coherence_fail"],
                    stats["max_angle"],
                )
            )
        base["evidence"] = (
            prefix
            + detail
            + " Gyro unit is undetermined; it was not guessed from the noise level."
        )
        return base

    arr = np.asarray(ratios, dtype=np.float64)
    median = float(np.median(arr))
    lo = 1.0 - GYRO_RATIO_TOL
    hi = 1.0 + GYRO_RATIO_TOL
    deg_lo = DEG_PER_RAD * lo
    deg_hi = DEG_PER_RAD * hi
    if lo <= median <= hi:
        verdict = "rad/s"
        band = "{0:.2f}–{1:.2f}".format(lo, hi)
    elif deg_lo <= median <= deg_hi:
        verdict = "deg/s"
        band = "{0:.2f}–{1:.2f}".format(deg_lo, deg_hi)
    else:
        verdict = "undetermined"
        band = None
    evidence = (
        "Median over {0} motion windows of (trapezoidal sum |gyro| dt) / "
        "(odometry rotation angle) is {1:.6g}. Window ratios ranged from "
        "{2:.6g} to {3:.6g}. rad/s band is {4:.2f}–{5:.2f}; deg/s band is "
        "{6:.2f}–{7:.2f} (57.3 is 180/π rounded; center is {8:.4f}).".format(
            int(arr.size),
            median,
            float(np.min(arr)),
            float(np.max(arr)),
            lo,
            hi,
            deg_lo,
            deg_hi,
            DEG_PER_RAD,
        )
    )
    if band is None:
        evidence += " The median is outside both bands, so the unit is undetermined."
    else:
        evidence += " The median is inside the {0} band.".format(band)
    return {
        "verdict": verdict,
        "ratio": median,
        "windows_used": int(arr.size),
        "evidence": prefix + evidence,
        "caveat": CAVEAT,
    }


def _gyro_from_epochs(imu_epochs, odom_epochs):
    """Gyro unit across clock epochs.

    Sustained rewinds are paired in read order, whether or not the ranges
    overlap. They are not matched by the shared timestamp numbers. A count
    mismatch is ambiguous.
    """
    base = {
        "verdict": "undetermined",
        "ratio": None,
        "windows_used": 0,
        "evidence": "",
        "caveat": CAVEAT,
    }
    if odom_epochs is None:
        base["evidence"] = (
            "No odometry was supplied. Gyro unit is undetermined; "
            "it was not guessed from the noise level."
        )
        return base
    n_imu = len(imu_epochs)
    n_odom = len(odom_epochs)
    if n_imu != n_odom:
        base["evidence"] = (
            "Sustained clock rewinds could not be paired ({0} IMU epochs, "
            "{1} odometry epochs). They were not placed on one timeline. "
            "Gyro unit is undetermined; it was not guessed from the noise level."
        ).format(n_imu, n_odom)
        return base
    prefix = ""
    if n_imu > 1:
        prefix = (
            "Sustained clock rewinds were kept separate ({0} IMU, {1} odometry). "
        ).format(n_imu, n_odom)
    ratios: List[float] = []
    stats = _empty_gyro_stats()
    for (_t_srt, _acc_srt, gyr_srt), (odom_t, odom_q) in zip(imu_epochs, odom_epochs):
        part_ratios, part_stats = _collect_gyro_windows(
            _t_srt, gyr_srt, odom_t, odom_q
        )
        ratios.extend(part_ratios)
        _add_gyro_stats(stats, part_stats)
    return _ratios_to_verdict(ratios, stats, prefix)


def _classify_covariance(values):
    if values is None:
        return {"pattern": "not_provided", "message_count": 0, "n_unique_rows": 0, "example": None}
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return {"pattern": "not_provided", "message_count": 0, "n_unique_rows": 0, "example": None}
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] != 9:
        return {
            "pattern": "unexpected_shape",
            "message_count": int(arr.shape[0]),
            "n_unique_rows": None,
            "example": None,
        }
    zero = bool(np.all(np.abs(arr) <= 1e-12))
    all_neg = bool(np.all(np.abs(arr + 1.0) <= 1e-12))
    first_neg = bool(np.all(np.abs(arr[:, 0] + 1.0) <= 1e-12))
    uniq = np.unique(np.round(arr, 6), axis=0)
    n_unique = int(uniq.shape[0])
    if zero:
        pattern = "all_zero"
    elif all_neg:
        pattern = "all_-1"
    elif first_neg and n_unique == 1:
        pattern = "first_element_-1"
    elif n_unique == 1:
        pattern = "constant_other"
    else:
        pattern = "mixed"
    example = [float(v) for v in arr[0]]
    return {
        "pattern": pattern,
        "message_count": int(arr.shape[0]),
        "n_unique_rows": n_unique,
        "example": example,
    }


def _fmt(value, digits=4):
    if value is None:
        return "n/a"
    return "{0:.{1}g}".format(value, digits)


def _findings(
    n,
    n_dropped,
    n_backward,
    rate,
    intervals,
    windows,
    diag,
    accel_unit,
    accel_norm_stats,
    gravity,
    gyro_unit,
    gyro_bias,
    gyro_noise,
    accel_noise,
    covariance,
):
    lines: List[str] = []
    stationary_s = 0.0
    for t0, t1 in windows:
        stationary_s += float(t1 - t0)
    lines.append(
        "Read {0} IMU messages. Header stamps run from {1} s to {2} s. "
        "Header-span rate is {3} Hz. "
        "Interval p5/p50/p95/max seconds: {4} / {5} / {6} / {7}. "
        "{8} consecutive header steps decreased in read order.".format(
            n,
            _fmt(diag.get("t_first"), 12),
            _fmt(diag.get("t_last"), 12),
            _fmt(rate, 6),
            _fmt(intervals.get("p5"), 6),
            _fmt(intervals.get("p50"), 6),
            _fmt(intervals.get("p95"), 6),
            _fmt(intervals.get("max"), 6),
            n_backward,
        )
    )
    if n_dropped:
        lines.append(
            "{0} messages were left out of the physics because a header time "
            "or a vector component was not finite.".format(n_dropped)
        )
    lines.append(
        "Stationary gate: accel relative std <= {0}, gyro magnitude std <= {1}, "
        "gyro magnitude median <= {2}, window {3:.0f} s. "
        "{4} windows evaluated, {5} passed, {6} failed the accel gate, "
        "{7} failed the gyro-std gate, {8} failed the gyro-median gate, "
        "{9} were too short. Gyro-magnitude-std p10/p50 = {10} / {11}. "
        "Accel relative-std p10/p50 = {12} / {13}. "
        "Merged stationary time is {14:.1f} s in {15} ranges.".format(
            ACCEL_REL_STD_MAX,
            diag["gyro_std_max"],
            diag["gyro_median_max"],
            WINDOW_S,
            diag["n_windows"],
            diag["n_pass"],
            diag["n_fail_accel"],
            diag["n_fail_gyro_std"],
            diag["n_fail_gyro_median"],
            diag["n_too_short"],
            _fmt(diag["gyro_mag_std_p10"], 4),
            _fmt(diag["gyro_mag_std_p50"], 4),
            _fmt(diag["accel_rel_std_p10"], 4),
            _fmt(diag["accel_rel_std_p50"], 4),
            stationary_s,
            len(windows),
        )
    )
    lines.append(
        "Accel unit: {0}. {1}".format(accel_unit["verdict"], accel_unit["evidence"])
    )
    lines.append(
        "Gravity: dominant axis {0}, angle from that axis {1}°. {2}. "
        "A stationary accelerometer reads specific force, opposite gravity, so +z means the published +z axis points up.".format(
            gravity["dominant_axis"] if gravity["dominant_axis"] is not None else "undetermined",
            _fmt(gravity["angle_from_axis_deg"], 4),
            gravity["mount_sentence"],
        )
    )
    lines.append(
        "Gyro unit: {0}. {1} {2}".format(
            gyro_unit["verdict"], gyro_unit["evidence"], gyro_unit["caveat"]
        )
    )
    if gyro_bias is None:
        lines.append("Gyro bias and noise were not computed: no stationary samples.")
    else:
        lines.append(
            "Stationary gyro mean (native message units, verdict {0}) is "
            "[{1:.6g}, {2:.6g}, {3:.6g}]. Gyro sample std is "
            "[{4:.6g}, {5:.6g}, {6:.6g}]. Accel sample std is "
            "[{7:.6g}, {8:.6g}, {9:.6g}].".format(
                gyro_unit["verdict"],
                gyro_bias[0],
                gyro_bias[1],
                gyro_bias[2],
                gyro_noise[0],
                gyro_noise[1],
                gyro_noise[2],
                accel_noise[0],
                accel_noise[1],
                accel_noise[2],
            )
        )
    cov_bits = []
    for name in ("orientation", "angular_velocity", "linear_acceleration"):
        item = covariance[name]
        cov_bits.append("{0}={1}".format(name, item.get("pattern")))
    lines.append(
        "Published covariance: {0}. {1}".format(
            ", ".join(cov_bits), covariance["sensor_msgs_note"]
        )
    )
    # accel_norm_stats is already described via accel_unit. Keep the argument
    # so a future sentence can quote p5/p95 without another pass.
    _ = accel_norm_stats
    return lines
