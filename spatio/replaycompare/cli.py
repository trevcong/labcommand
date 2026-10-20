"""Command line for replay-vs-historical comparison."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

from spatio.replaycompare.compare import compare_sessions
from spatio.replaycompare.report import write_outputs


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="spatio.replaycompare",
        description=(
            "Compare two Spatio session folders. "
            "The headline is the raw-frame trajectory difference. "
            "SE(3)-aligned ATE is labelled diagnostic only."
        ),
    )
    parser.add_argument("session_a", type=Path, help="Session folder A")
    parser.add_argument("session_b", type=Path, help="Session folder B")
    parser.add_argument("--out", type=Path, required=True, help="Directory for the report")
    parser.add_argument("--max-dt", type=float, default=0.05, help="Association window in seconds")
    parser.add_argument("--voxel", type=float, default=0.05, help="Map voxel size in metres")
    parser.add_argument(
        "--time-offset",
        type=float,
        default=None,
        help="Seconds added to session B timestamps. Omitted means no shift.",
    )
    args = parser.parse_args(argv)
    if args.max_dt < 0:
        parser.error("--max-dt must be >= 0")
    if args.voxel <= 0:
        parser.error("--voxel must be > 0")
    try:
        result, series = compare_sessions(
            args.session_a,
            args.session_b,
            max_dt=args.max_dt,
            voxel=args.voxel,
            time_offset=args.time_offset,
        )
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    write_outputs(args.out, result, series)
    position = result["trajectory"]["raw"]["position_m"]
    if position is None:
        print(f"status={result['status']} out={args.out}")
    else:
        print(
            f"status={result['status']} "
            f"raw_position_mean_m={position['mean']:.6f} "
            f"out={args.out}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
