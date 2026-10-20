"""Write the JSON report and a plain-English markdown page."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

import numpy as np

from spatio.imucheck.analyze import (
    ACCEL_G_HI,
    ACCEL_G_LO,
    ACCEL_MPS2_HI,
    ACCEL_MPS2_LO,
    ACCEL_REL_STD_MAX,
    GYRO_MAG_MEDIAN_MAX,
    GYRO_MAG_STD_MAX,
    GYRO_RATIO_TOL,
    MAX_IMU_GAP_S,
    MAX_MOTION_ANGLE_RAD,
    MIN_IMU_COVERAGE_FRACTION,
    PROVISIONAL_LABEL,
    WINDOW_S,
)
from spatio.imucheck.fastlio import quotes_as_text


def topic_slug(topic: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "_", topic).strip("_")
    return slug or "topic"


def output_paths(out_dir, topic):
    directory = Path(out_dir)
    stem = "imucheck_{0}".format(topic_slug(topic))
    return directory / (stem + ".json"), directory / (stem + ".md")


def write_reports(report, out_dir, bag_path, library_versions, quotes):
    directory = Path(out_dir)
    directory.mkdir(parents=True, exist_ok=True)
    json_path, md_path = output_paths(directory, report.get("topic", ""))
    payload = _to_builtin(report)
    json_path.write_text(
        json.dumps(payload, indent=2, allow_nan=False, sort_keys=False) + "\n",
        encoding="utf-8",
    )
    md_path.write_text(
        _markdown(payload, bag_path, library_versions, quotes),
        encoding="utf-8",
    )
    return json_path, md_path


def _to_builtin(obj):
    if isinstance(obj, dict):
        return {str(key): _to_builtin(value) for key, value in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_builtin(value) for value in obj]
    if isinstance(obj, np.ndarray):
        return _to_builtin(obj.tolist())
    if isinstance(obj, np.floating):
        value = float(obj)
        if not math.isfinite(value):
            return None
        return value
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, float):
        if not math.isfinite(obj):
            return None
        return obj
    return obj


def _markdown(report, bag_path, library_versions, quotes):
    accel = report["accel_unit"]
    gravity = report["gravity"]
    gyro = report["gyro_unit"]
    norms = report["accel_norm_stationary"]
    interval = report["interval_s"]
    lines = [
        "# IMU check: {0}".format(report.get("topic") or "(no topic)"),
        "",
        "Bag: `{0}`".format(bag_path),
        "",
        "Reader: {0}.".format(library_versions),
        "",
        "Schema `{0}`. Messages: {1}. Frame ids: {2}.".format(
            report["schema"],
            report["count"],
            ", ".join(report["frame_ids"]) if report["frame_ids"] else "(none)",
        ),
        "",
        "## Rate",
        "",
        "Header-span rate: {0} Hz.".format(_num(report["rate_hz"])),
        "",
        "Sorted header intervals, seconds: p5 {0}, p50 {1}, p95 {2}, max {3}.".format(
            _num(interval.get("p5")),
            _num(interval.get("p50")),
            _num(interval.get("p95")),
            _num(interval.get("max")),
        ),
        "",
        "## Stationary windows",
        "",
        "A window is {0:.0f} s. It is stationary when the accel-norm relative std is "
        "<= {1}, the gyro-magnitude std is <= {2} rad/s, and the gyro-magnitude median is "
        "<= {3} rad/s. When the gyro verdict is deg/s those two angular gates are "
        "multiplied by 180/π. When the gyro unit is undetermined they are left unscaled, "
        "so a deg/s stream is not called stationary. A steady turn fails the median gate.".format(
            WINDOW_S, ACCEL_REL_STD_MAX, GYRO_MAG_STD_MAX, GYRO_MAG_MEDIAN_MAX
        ),
        "",
        "Ranges ({0}):".format(len(report["stationary_windows"])),
        "",
    ]
    shown = report["stationary_windows"][:12]
    if not shown:
        lines.append("- none")
    for window in shown:
        lines.append("- {0:.3f} s to {1:.3f} s".format(window["t0_s"], window["t1_s"]))
    if len(report["stationary_windows"]) > len(shown):
        lines.append(
            "- … {0} more ranges in the JSON".format(
                len(report["stationary_windows"]) - len(shown)
            )
        )
    lines.extend(
        [
            "",
            "## Accel unit",
            "",
            "Verdict: **{0}**.".format(accel["verdict"]),
            "",
            accel["evidence"],
            "",
            "Stationary |a| median {0}, p5 {1}, p95 {2}.".format(
                _num(norms.get("median")),
                _num(norms.get("p5")),
                _num(norms.get("p95")),
            ),
            "",
            "Bands: g is {0}–{1}, m/s^2 is {2}–{3}. Outside both is undetermined.".format(
                ACCEL_G_LO, ACCEL_G_HI, ACCEL_MPS2_LO, ACCEL_MPS2_HI
            ),
            "",
            "## Gravity and mount",
            "",
            "A stationary accelerometer measures specific force, the support force, "
            "which points opposite gravity (up). Dominant +z within 15° means the "
            "published +z axis points up. Dominant -z within 15° means it points down. "
            "This is the published frame, not a claim about the metal housing.",
            "",
            "Mean direction: {0}.".format(gravity["mean_dir_sensor"]),
            "",
            "Dominant axis: {0}. Angle from that axis: {1}°.".format(
                gravity["dominant_axis"], _num(gravity["angle_from_axis_deg"])
            ),
            "",
            gravity["mount_sentence"] + ".",
            "",
            "## Gyro unit",
            "",
            "Verdict: **{0}**.".format(gyro["verdict"]),
            "",
            "Median ratio: {0}. Windows used: {1}.".format(
                _num(gyro["ratio"]), gyro["windows_used"]
            ),
            "",
            gyro["evidence"],
            "",
            gyro["caveat"],
            "",
            "The ratio band is ±{0:.0%} around 1 (rad/s) and around 180/π (deg/s). "
            "The raw sum |gyro| dt is the verdict. Bias is not removed first. "
            "A window is not used when the IMU samples inside the odometry "
            "interval span less than {1:.0%} of it, when a gap in that "
            "interval is longer than {2:g} s, or when the sum of intermediate "
            "odometry steps exceeds {3:g} rad. Overlapping clock epochs are "
            "kept separate and are not sorted onto one timeline. {4}.".format(
                GYRO_RATIO_TOL,
                MIN_IMU_COVERAGE_FRACTION,
                MAX_IMU_GAP_S,
                MAX_MOTION_ANGLE_RAD,
                PROVISIONAL_LABEL,
            ),
            "",
            "## Bias and noise",
            "",
            "Gyro bias (native units): {0}.".format(report["gyro_bias"]),
            "",
            "Gyro sample std: {0}.".format(report["gyro_noise_std"]),
            "",
            "Accel sample std: {0}.".format(report["accel_noise_std"]),
            "",
            "## Covariance",
            "",
        ]
    )
    cov = report["covariance_published"]
    for name in ("orientation", "angular_velocity", "linear_acceleration"):
        item = cov[name]
        lines.append(
            "- {0}: {1} (unique rows {2})".format(
                name, item.get("pattern"), item.get("n_unique_rows")
            )
        )
    lines.extend(
        [
            "",
            cov.get("sensor_msgs_note", ""),
            "",
            "## Findings",
            "",
        ]
    )
    for sentence in report["findings"]:
        lines.append("- {0}".format(sentence))
    lines.extend(
        [
            "",
            "## FAST-LIO normalization",
            "",
            "Static read of the copied source. Line numbers are from the files on this machine.",
            "",
            "```",
            quotes_as_text(quotes),
            "```",
            "",
            "FAST-LIO divides acceleration by the mean norm from initialization and "
            "multiplies by G_m_s2 (9.81), so it accepts either g or m/s^2 as long as "
            "the init window is near one gravity. Angular velocity is not converted. "
            "It is subtracted from the gyro bias and fed to the SO3 rate, which is "
            "radians. The Livox comments say g and rad/s; those are comments. The "
            "verdict above is from the bag.",
            "",
        ]
    )
    return "\n".join(lines)


def _num(value):
    if value is None:
        return "n/a"
    return "{0:.6g}".format(value)
