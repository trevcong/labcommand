"""Command line for suggest and measure."""

from __future__ import annotations

import argparse
import os
import sys

from spatio.wallpatch.errors import WallpatchError
from spatio.wallpatch.measure import measure_map
from spatio.wallpatch.suggest import suggest_map


def main(argv=None):
    parser = _parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        code = exc.code
        if code is None:
            return 0
        return int(code)
    try:
        if args.command == "suggest":
            suggest_map(args.map, args.out, n=args.n, voxel_m=args.voxel_m, verbose=True)
            return 0
        if args.command == "measure":
            measure_map(
                _resolve_map(args.target),
                args.patches,
                args.out,
                accuracy_label=args.accuracy_label,
                verbose=True,
            )
            return 0
    except WallpatchError as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 1
    except OSError as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 1
    sys.stderr.write("error: unknown command\n")
    return 2


def _parser():
    parser = argparse.ArgumentParser(prog="spatio.wallpatch")
    sub = parser.add_subparsers(dest="command", required=True)

    suggest = sub.add_parser("suggest", help="propose vertical wall patches")
    suggest.add_argument("map", help="path to map.pcd")
    suggest.add_argument("--n", type=int, default=6, help="how many patches to write")
    suggest.add_argument("--out", required=True, help="patches.json path")
    suggest.add_argument(
        "--voxel-m",
        type=float,
        default=0.08,
        help="voxel size for the search only; measure does not downsample",
    )

    measure = sub.add_parser("measure", help="measure named wall patches")
    measure.add_argument("target", help="session directory or map.pcd path")
    measure.add_argument("--patches", required=True, help="patches.json path")
    measure.add_argument("--out", required=True, help="directory for the report and figures")
    measure.add_argument(
        "--accuracy-label",
        default=None,
        help="copied into the report; this tool never sets physical accuracy to true",
    )
    return parser


def _resolve_map(target):
    if os.path.isdir(target):
        candidate = os.path.join(target, "map.pcd")
        if not os.path.isfile(candidate):
            raise WallpatchError("no map.pcd in %s" % target)
        return candidate
    if not os.path.isfile(target):
        raise WallpatchError("map not found: %s" % target)
    return target
