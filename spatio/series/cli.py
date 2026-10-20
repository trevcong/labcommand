"""Command line for explain and compare.

Exit 0: the report was written (any verdict).
Exit 1: an input was rejected.
Exit 2: an input or the output path was unreadable.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from spatio.series.report import build_series, default_series_id, explain_text, render_markdown
from spatio.series.validate import InputRejected, Unreadable, load_walk_result


def main(argv=None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "explain":
            return _explain(args)
        if args.command == "compare":
            return _compare(args)
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
        help="walk_result.json files, in attempt order",
    )
    compare.add_argument("--out", required=True, help="Directory for series.md and series.json")
    compare.add_argument("--series-id", help="Series name. Default uses the first and last run id")
    return parser


def _explain(args: argparse.Namespace) -> int:
    walk = load_walk_result(args.walk_result)
    text = explain_text(walk)
    if args.out:
        try:
            with open(args.out, "w", encoding="utf-8") as handle:
                handle.write(text)
        except OSError as exc:
            print("unreadable: %s: %s" % (args.out, exc), file=sys.stderr)
            return 2
    sys.stdout.write(text)
    return 0


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
        with open(md_path, "w", encoding="utf-8") as handle:
            handle.write(markdown)
        with open(json_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, ensure_ascii=False)
            handle.write("\n")
    except OSError as exc:
        print("unreadable: %s: %s" % (args.out, exc), file=sys.stderr)
        return 2
    print("%s %s" % (report["verdict"], os.path.join(args.out, "series.md")))
    return 0
