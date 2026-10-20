"""Command line for the walk checker."""

from __future__ import annotations

import argparse
import sys

from spatio.walkcheck.defaults import (
    MAX_ENDPOINT_SPEED_M_S,
    MAX_ENDPOINT_SPREAD_M,
    MAX_GAP_S,
    MIN_EXCURSION_M,
    MIN_PATH_LENGTH_M,
    STATIONARY_WINDOW_S,
)
from spatio.walkcheck.evaluate import SessionDirError, evaluate_session

_EXIT = {"PASS": 0, "FAIL": 1, "NOT-EVALUABLE": 2}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m spatio.walkcheck",
        description=(
            "Score a session folder's trajectory.tum and map.pcd. "
            "Writes walkcheck.json, walkcheck.md, and topdown.png into that folder. "
            "A missing or unreadable trajectory or map is NOT-EVALUABLE. "
            "The wall gate does not change the exit code."
        ),
    )
    parser.add_argument("session_dir", help="Session directory with trajectory.tum and map.pcd")
    parser.add_argument(
        "--stationary-window",
        type=float,
        default=STATIONARY_WINDOW_S,
        help="Seconds averaged at the start and at the end (default 5)",
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
            "Largest mean speed inside an endpoint window, in metres per second. "
            "Over this, the walk is NOT-EVALUABLE (default 0.10)"
        ),
    )
    args = parser.parse_args(argv)
    if args.stationary_window <= 0 or args.max_gap <= 0:
        parser.error("stationary window and max gap must be positive")
    if args.min_path_length < 0 or args.min_excursion < 0:
        parser.error("motion minimums cannot be negative")
    if args.max_endpoint_spread < 0 or args.max_endpoint_speed < 0:
        parser.error("endpoint spread and speed limits cannot be negative")
    try:
        report = evaluate_session(
            args.session_dir,
            stationary_window_s=args.stationary_window,
            max_gap_s=args.max_gap,
            min_path_length_m=args.min_path_length,
            min_excursion_m=args.min_excursion,
            max_endpoint_spread_m=args.max_endpoint_spread,
            max_endpoint_speed_m_s=args.max_endpoint_speed,
        )
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
