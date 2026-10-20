"""Command line for the walk checker."""

from __future__ import annotations

import argparse
import sys

from spatio.walkcheck.defaults import (
    MAX_ENDPOINT_SPEED_M_S,
    MAX_ENDPOINT_SPREAD_M,
    MAX_GAP_S,
    MIN_EXCURSION_M,
    MIN_MAP_POINTS,
    MIN_PATH_LENGTH_M,
    STATIONARY_WINDOW_S,
)
from spatio.walkcheck.evaluate import SessionDirError, evaluate_session
from spatio.walkcheck.trajectory import ThresholdError
from spatio.walkcheck.holds import (
    MIN_HOLD_S, MAX_HOLD_SPEED_M_S, MAX_ANGULAR_SPEED_DEG_S, SIGNIFICANT_MOTION_M,
    SIGNIFICANT_ROTATION_DEG, MAX_IMU_GAP_S, MAX_ACCEL_DEVIATION_M_S2,
    GRAVITY_M_S2,
)

_EXIT = {"PASS": 0, "FAIL": 1, "NOT-EVALUABLE": 2}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m spatio.walkcheck",
        description=(
            "Score a session folder's trajectory.tum and map.pcd. "
            "Writes walkcheck.json, walkcheck.md, and topdown.png into that folder. "
            "A missing or unreadable trajectory or map, or an undersized map, is NOT-EVALUABLE. "
            "The wall gate does not change the exit code."
        ),
    )
    parser.add_argument("session_dir", help="Session directory with trajectory.tum and map.pcd")
    parser.add_argument(
        "--min-map-points",
        type=int,
        default=MIN_MAP_POINTS,
        help=f"PROVISIONAL: minimum finite XYZ map points (positive integer; default {MIN_MAP_POINTS})",
    )
    parser.add_argument(
        "--stationary-window",
        type=float,
        default=STATIONARY_WINDOW_S,
        help="Seconds averaged inside the first/last detected holds (default 5)",
    )
    parser.add_argument(
        "--max-gap",
        type=float,
        default=MAX_GAP_S,
        help="A larger timestamp step makes the run NOT-EVALUABLE (default 1 second)",
    )
    parser.add_argument(
        "--min-path-length",
        type=float,
        default=MIN_PATH_LENGTH_M,
        help="Minimum path length in metres, or the run fails as stationary (default 2)",
    )
    parser.add_argument(
        "--min-excursion",
        type=float,
        default=MIN_EXCURSION_M,
        help="Minimum farthest distance from the start mean in metres (default 0.5)",
    )
    parser.add_argument(
        "--max-endpoint-spread",
        type=float,
        default=MAX_ENDPOINT_SPREAD_M,
        help=(
            "Largest distance, in metres, from a sample to its window mean. "
            "Over this, the walk is NOT-EVALUABLE (default 0.05)"
        ),
    )
    parser.add_argument(
        "--max-endpoint-speed",
        type=float,
        default=MAX_ENDPOINT_SPEED_M_S,
        help=(
            "Largest trajectory step speed for hold detection, in metres per second. "
            "Over this, the walk is NOT-EVALUABLE (default 0.10)"
        ),
    )
    parser.add_argument("--imu-file", help="Optional CSV in trajectory clock: t,gx,gy,gz,ax,ay,az; rad/s and m/s² including gravity. Defaults to session/imu.csv if present.")
    for option, default, help_text in (
        ("max-hold-speed", MAX_HOLD_SPEED_M_S, "Maximum trajectory step speed for holds, m/s"),
        ("min-hold", MIN_HOLD_S, "Minimum sustained hold duration, seconds"),
        ("max-angular-speed", MAX_ANGULAR_SPEED_DEG_S, "Maximum pose/IMU angular speed, deg/s"),
        ("significant-motion", SIGNIFICANT_MOTION_M, "Motion flag threshold, metres"),
        ("significant-rotation", SIGNIFICANT_ROTATION_DEG, "Rotation flag and hold excursion limit, degrees"),
        ("max-imu-gap", MAX_IMU_GAP_S, "Maximum IMU timestamp gap, seconds"),
        ("max-accel-deviation", MAX_ACCEL_DEVIATION_M_S2, "Maximum acceleration magnitude deviation from gravity, m/s²"),
        ("gravity", GRAVITY_M_S2, "Expected gravity magnitude, m/s²"),
    ):
        parser.add_argument("--" + option, type=float, default=default,
                            help=f"PROVISIONAL: {help_text} (default {default:g})")
    args = parser.parse_args(argv)
    try:
        report = evaluate_session(
            args.session_dir,
            min_hold_s=args.min_hold,
            max_hold_speed_m_s=args.max_hold_speed,
            max_angular_speed_deg_s=args.max_angular_speed,
            significant_motion_m=args.significant_motion,
            significant_rotation_deg=args.significant_rotation,
            max_imu_gap_s=args.max_imu_gap,
            max_accel_deviation_m_s2=args.max_accel_deviation,
            gravity_m_s2=args.gravity,
            imu_file=args.imu_file,
            stationary_window_s=args.stationary_window,
            max_gap_s=args.max_gap,
            min_path_length_m=args.min_path_length,
            min_excursion_m=args.min_excursion,
            min_map_points=args.min_map_points,
            max_endpoint_spread_m=args.max_endpoint_spread,
            max_endpoint_speed_m_s=args.max_endpoint_speed,
        )
    except ThresholdError as exc:
        parser.error(str(exc))
    except SessionDirError as exc:
        print(str(exc), file=sys.stderr)
        return 3
    except OSError as exc:
        print(str(exc), file=sys.stderr)
        return 3
    print(_summary(report))
    return _EXIT[report["result"]]


def _summary(report: dict) -> str:
    returned = report.get("return_to_start")
    reasons = report.get("reasons") or []
    reason_text = ""
    if reasons:
        reason_text = "  reasons: " + ",".join(reasons)
    holds = report.get("holds") or {}
    if holds.get("flags"):
        reason_text += "  flags: " + ",".join(holds["flags"])
    walls = report.get("walls") or {}
    wall_text = ""
    if walls.get("status") == "OK":
        wall_text = (
            f"  walls p95 {walls['p95_point_to_plane_cm']:.2f} cm"
            f" (provisional gate {walls['proposed_gate_result']})"
        )
    if not returned:
        return f"{report['result']}{reason_text}{wall_text}"
    return (
        f"{report['result']}  3D {returned['error_3d_cm']:.2f} cm"
        f"  horizontal {returned['error_horizontal_cm']:.2f} cm"
        f"  vertical {returned['error_vertical_cm']:.2f} cm"
        f"  heading {returned['heading_error_deg']:.2f} deg"
        f"{reason_text}{wall_text}"
    )
