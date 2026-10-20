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
(below π, so the quaternion short-arc is the whole turn), and
|∫ω dt| / ∫|ω| dt >= 0.70 (the body rate does not reverse). Median ratio
0.75–1.25 → rad/s. The same band times 180/π → deg/s (57.3 is that
constant rounded). Otherwise undetermined. Nothing is inferred from a
noise floor. /Odometry was produced by FAST-LIO using this IMU, so the
ratio is support, not proof.

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

MIN_MOTION_ANGLE_RAD = 0.15
MAX_MOTION_ANGLE_RAD = 2.5
MIN_COHERENCE = 0.70
MIN_ODOM_SPAN_FRACTION = 0.50

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
    finite = np.isfinite(t)
    if n:
        finite = finite & np.all(np.isfinite(acc), axis=1) & np.all(np.isfinite(gyr), axis=1)
    n_dropped = int(n - int(np.count_nonzero(finite)))

    rate, intervals, n_backward = _rate_and_intervals(t, finite)

    if n and np.any(finite):
        order = np.argsort(t[finite], kind="mergesort")
        t_srt = t[finite][order]
        acc_srt = acc[finite][order]
        gyr_srt = gyr[finite][order]
    else:
        t_srt = np.zeros(0, dtype=np.float64)
        acc_srt = np.zeros((0, 3), dtype=np.float64)
        gyr_srt = np.zeros((0, 3), dtype=np.float64)

    odom_time, odom_q = _as_odom(odom_t, odom_quat)
    # Gyro unit does not use the stationary mask, so it can set the unit of
    # the stationary gyro gate without a circular argument.
    gyro_unit = _gyro_unit(t_srt, gyr_srt, odom_time, odom_q)
    gyro_gate_scale = DEG_PER_RAD if gyro_unit["verdict"] == "deg/s" else 1.0
    windows, diag = _stationary_windows(
        t_srt,
        acc_srt,
        gyr_srt,
        gyro_std_max=GYRO_MAG_STD_MAX * gyro_gate_scale,
        gyro_median_max=GYRO_MAG_MEDIAN_MAX * gyro_gate_scale,
    )
    diag["t_first"] = float(t_srt[0]) if t_srt.size else None
    diag["t_last"] = float(t_srt[-1]) if t_srt.size else None
    stat_mask = _mask_windows(t_srt, windows)

    accel_norm_stats, accel_unit = _accel_unit(acc_srt, stat_mask)
    gravity = _gravity(acc_srt, stat_mask)
    gyro_bias, gyro_noise, accel_noise = _bias_and_noise(acc_srt, gyr_srt, stat_mask)

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

    def _pct(values, q):
        if not values:
            return None
        return float(np.percentile(np.asarray(values, dtype=np.float64), q))

    diag = {
        "n_windows": n_windows,
        "n_pass": len(passed),
        "n_fail_accel": n_fail_accel,
        "n_fail_gyro_std": n_fail_gyro_std,
        "n_fail_gyro_median": n_fail_gyro_median,
        "n_too_short": n_too_short,
        "gyro_std_max": float(gyro_std_max),
        "gyro_median_max": float(gyro_median_max),
        "gyro_mag_std_p10": _pct(gyro_stds, 10),
        "gyro_mag_std_p50": _pct(gyro_stds, 50),
        "accel_rel_std_p10": _pct(accel_rels, 10),
        "accel_rel_std_p50": _pct(accel_rels, 50),
    }
    return merged, diag


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


def _as_odom(odom_t, odom_quat):
    if odom_t is None or odom_quat is None:
        return None, None
    ot = np.asarray(odom_t, dtype=np.float64)
    oq = np.asarray(odom_quat, dtype=np.float64)
    if ot.ndim != 1:
        raise ValueError("odom_t must be one-dimensional")
    if oq.ndim != 2 or oq.shape[1] != 4:
        raise ValueError("odom_quat must have shape (M, 4) xyzw")
    if len(ot) != len(oq):
        raise ValueError("odom_t and odom_quat must have the same length")
    if len(ot) == 0:
        return None, None
    finite = np.isfinite(ot) & np.all(np.isfinite(oq), axis=1)
    if not np.any(finite):
        return None, None
    order = np.argsort(ot[finite], kind="mergesort")
    return ot[finite][order], oq[finite][order]


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


def _integrate(t_s, gyro, t_a, t_b):
    """Trapezoidal ∫|ω| dt and ∫ω dt for samples inside [t_a, t_b]."""
    i0 = int(np.searchsorted(t_s, t_a, side="left"))
    i1 = int(np.searchsorted(t_s, t_b, side="right"))
    if i1 - i0 < 2:
        return None
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


def _gyro_unit(t_srt, gyr_srt, odom_t, odom_q):
    base = {
        "verdict": "undetermined",
        "ratio": None,
        "windows_used": 0,
        "evidence": "",
        "caveat": CAVEAT,
    }
    if odom_t is None or odom_q is None:
        base["evidence"] = (
            "No odometry was supplied. Gyro unit is undetermined; "
            "it was not guessed from the noise level."
        )
        return base
    ratios: List[float] = []
    max_angle = 0.0
    n_with_pair = 0
    n_angle_in_range = 0
    n_coherence_fail = 0
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
        n_with_pair += 1
        if angle > max_angle:
            max_angle = angle
        if angle < MIN_MOTION_ANGLE_RAD or angle > MAX_MOTION_ANGLE_RAD:
            continue
        n_angle_in_range += 1
        integrated = _integrate(t_srt, gyr_srt, t_a, t_b)
        if integrated is None:
            continue
        path, vec = integrated
        coherence = float(np.linalg.norm(vec) / path)
        if coherence < MIN_COHERENCE:
            n_coherence_fail += 1
            continue
        ratios.append(path / angle)

    if not ratios:
        if n_with_pair == 0:
            detail = "No 1 s window contained two usable odometry poses."
        else:
            detail = (
                "No motion window met the gates (odometry rotation between "
                "{0:.2f} and {1:.1f} rad, coherence >= {2:.2f}, odometry span "
                ">= {3:.0%} of the window). Windows with a pose pair: {4}. "
                "Of those, {5} had a rotation in range and {6} of those failed "
                "coherence. Largest 1 s odometry rotation was {7:.4f} rad.".format(
                    MIN_MOTION_ANGLE_RAD,
                    MAX_MOTION_ANGLE_RAD,
                    MIN_COHERENCE,
                    MIN_ODOM_SPAN_FRACTION,
                    n_with_pair,
                    n_angle_in_range,
                    n_coherence_fail,
                    max_angle,
                )
            )
        base["evidence"] = (
            detail + " Gyro unit is undetermined; it was not guessed from the noise level."
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
        "evidence": evidence,
        "caveat": CAVEAT,
    }


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
