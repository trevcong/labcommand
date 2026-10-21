"""Command line for explain and compare.

Exit 0: PASS (or adapter conversion completed).
Exit 1: an input was rejected.
Exit 2: an input or the output path was unreadable.
Exit 3: FAIL.
Exit 4: INCOMPLETE.
Exit 5: NOT_EVALUABLE.
Exit 6: REPLAY_ONLY.
"""

from __future__ import annotations

import argparse
import os
import sys

from spatio.manifest.jsonio import write_json_atomic, write_text_atomic
from spatio.series.report import build_series, default_series_id, explain_text, render_markdown
from spatio.series.validate import InputRejected, Unreadable, load_walk_result
from spatio.thresholds import WALL_GATE_M, WALL_GATE_STATE, wall_gate

VERDICT_EXIT = {"PASS": 0, "FAIL": 3, "INCOMPLETE": 4,
                "NOT_EVALUABLE": 5, "REPLAY_ONLY": 6}


def main(argv=None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "explain":
            return _explain(args)
        if args.command == "compare":
            return _compare(args)
        if args.command == "adapt":
            return _adapt(args)
    except Unreadable as exc:
        print("unreadable: %s" % exc, file=sys.stderr)
        return 2
    except InputRejected as exc:
        print("rejected: %s" % exc, file=sys.stderr)
        return 1
    print("unreadable: unknown command", file=sys.stderr)
    return 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m spatio.series",
        description="Plain-English walk reports and the three-run series verdict.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    explain = sub.add_parser("explain", help="Summarize one walk_result.json")
    explain.add_argument("walk_result", help="Path to one walk_result.json")
    explain.add_argument("--out", help="Also write the summary to this file")

    compare = sub.add_parser("compare", help="Compare walk results and write a series verdict")
    compare.add_argument(
        "walk_results",
        nargs="+",
        help="walk_result.json files; sorted by recorded_at, regardless of argument order",
    )
    compare.add_argument("--out", required=True, help="Directory for series.md and series.json")
    compare.add_argument("--series-id", help="Series name. Default uses the first and last run id")
    adapt = sub.add_parser("adapt", help="Convert walkcheck.json using explicit session metadata")
    adapt.add_argument("walkcheck", help="spatio.walkcheck.v1 JSON report")
    adapt.add_argument("--metadata", required=True, help="Session metadata JSON (see series README)")
    adapt.add_argument("--out", required=True, help="Output walk_result.json")
    adapt.add_argument("--wallpatch", help="Measured wallpatch.json report")
    adapt.add_argument("--patches", help="Source patches.json (retained as a provenance hash)")
    adapt.add_argument("--spec", help="Frozen spec naming the required preselected patches")
    adapt.add_argument("--patch-origin", choices=("preselected", "auto_suggested"), default="auto_suggested")
    adapt.add_argument("--wall-gate-m", type=wall_gate, default=WALL_GATE_M)
    adapt.add_argument("--wall-gate-state", choices=("approved", "proposed"), default=WALL_GATE_STATE)
    return parser


def _explain(args: argparse.Namespace) -> int:
    walk = load_walk_result(args.walk_result)
    text = explain_text(walk)
    if args.out:
        try:
            write_text_atomic(args.out, text)
        except OSError as exc:
            print("unreadable: %s: %s" % (args.out, exc), file=sys.stderr)
            return 2
    sys.stdout.write(text)
    return VERDICT_EXIT[walk.status]


def _compare(args: argparse.Namespace) -> int:
    walks = [load_walk_result(path) for path in args.walk_results]
    series_id = args.series_id or default_series_id(walks)
    report = build_series(walks, series_id)
    markdown = render_markdown(report)
    try:
        os.makedirs(args.out, exist_ok=True)
        if not os.path.isdir(args.out):
            print("unreadable: %s: not a directory" % args.out, file=sys.stderr)
            return 2
        md_path = os.path.join(args.out, "series.md")
        json_path = os.path.join(args.out, "series.json")
        write_text_atomic(md_path, markdown)
        write_json_atomic(json_path, report)
    except OSError as exc:
        print("unreadable: %s: %s" % (args.out, exc), file=sys.stderr)
        return 2
    print("%s %s" % (report["verdict"], os.path.join(args.out, "series.md")))
    return VERDICT_EXIT[report["verdict"]]


def _adapt(args: argparse.Namespace) -> int:
    from spatio.series.adapter import adapt_walkcheck, read_json

    result = adapt_walkcheck(
        read_json(args.walkcheck), read_json(args.metadata), args.walkcheck,
        wallpatch=read_json(args.wallpatch) if args.wallpatch else None,
        patches_file=args.patches, spec_path=args.spec, patch_origin=args.patch_origin,
        gate_m=args.wall_gate_m, gate_state=args.wall_gate_state,
    )
    try:
        write_json_atomic(args.out, result)
    except OSError as exc:
        raise Unreadable(args.out, str(exc)) from exc
    print("%s %s" % (result["status"], args.out))
    return 0
