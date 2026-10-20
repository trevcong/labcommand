"""Command line for the milestone-1 experiment spec.

Exit 0 means no errors. Warnings still exit 0. Exit 1 means the command
refused the input.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from spatio.expspec.freeze import FreezeError, freeze_spec
from spatio.expspec.io_util import read_json_object, write_bytes_atomic
from spatio.expspec.issues import errors_of
from spatio.expspec.record import RecordError, parse_residual_arg, record_physical_reference
from spatio.expspec.render import RenderError, render_markdown
from spatio.expspec.runsheet import RunsheetError, write_runsheet
from spatio.expspec.template import build_template
from spatio.expspec.validate import validate


def _load(path):
    try:
        spec, _raw = read_json_object(path)
    except (OSError, ValueError) as exc:
        print("ERROR: {0}: {1}".format(path, exc), file=sys.stderr)
        return None
    return spec


def _print_issues(issues):
    for item in issues:
        print(item.format(), file=sys.stderr)


def _cmd_new(args):
    text = json.dumps(build_template(), indent=2, ensure_ascii=False) + "\n"
    write_bytes_atomic(Path(args.out), text.encode("utf-8"))
    print(args.out)
    return 0


def _cmd_validate(args):
    spec = _load(args.spec)
    if spec is None:
        return 1
    issues = validate(spec)
    _print_issues(issues)
    if errors_of(issues):
        return 1
    print("ok")
    return 0


def _cmd_render(args):
    spec = _load(args.spec)
    if spec is None:
        return 1
    try:
        text = render_markdown(spec)
    except (KeyError, RenderError, TypeError) as exc:
        print("ERROR: {0}".format(exc), file=sys.stderr)
        return 1
    write_bytes_atomic(Path(args.out), text.encode("utf-8"))
    print(args.out)
    return 0


def _cmd_freeze(args):
    spec = _load(args.spec)
    if spec is None:
        return 1
    try:
        digest, issues = freeze_spec(spec, args.out)
    except FreezeError as exc:
        _print_issues(exc.issues)
        return 1
    _print_issues(issues)
    print(digest)
    return 0


def _cmd_runsheet(args):
    try:
        _text, spec = write_runsheet(args.spec, args.runs, args.out)
    except RunsheetError as exc:
        print("ERROR: {0}".format(exc), file=sys.stderr)
        return 1
    required = spec["acceptance"]["runs_required"]
    if args.runs < required:
        print(
            "WARNING: sheet has {0} runs; the spec requires {1}".format(args.runs, required),
            file=sys.stderr,
        )
    print(args.out)
    return 0


def _cmd_record(args):
    try:
        start = parse_residual_arg(args.start_residual_m, "start_placement_residual_m")
        end = parse_residual_arg(args.end_residual_m, "end_placement_residual_m")
        result = record_physical_reference(
            spec_path=args.spec,
            run_id=args.run_id,
            start_residual_m=start,
            end_residual_m=end,
            heading=args.heading,
            operator=args.operator,
            method=args.method,
            notes=args.notes,
            out_path=args.out,
        )
    except RecordError as exc:
        print("ERROR: {0}".format(exc), file=sys.stderr)
        return 1
    print("placement_within_tolerance: {0}".format("true" if result.within_tolerance else "false"))
    for reason in result.reasons:
        print("placement_not_within_tolerance_because: {0}".format(reason))
    print(args.out)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(prog="python3 -m spatio.expspec")
    sub = parser.add_subparsers(dest="command")
    sub.required = True

    new_parser = sub.add_parser("new", help="write a spec template")
    new_parser.add_argument("--out", required=True)
    new_parser.set_defaults(func=_cmd_new)

    validate_parser = sub.add_parser("validate", help="check a spec")
    validate_parser.add_argument("spec")
    validate_parser.set_defaults(func=_cmd_validate)

    render_parser = sub.add_parser("render", help="render the one-page spec")
    render_parser.add_argument("spec")
    render_parser.add_argument("--out", required=True)
    render_parser.set_defaults(func=_cmd_render)

    freeze_parser = sub.add_parser("freeze", help="write canonical JSON and its sha256")
    freeze_parser.add_argument("spec")
    freeze_parser.add_argument("--out", required=True)
    freeze_parser.set_defaults(func=_cmd_freeze)

    sheet_parser = sub.add_parser("runsheet", help="print the operator sheet for N walks")
    sheet_parser.add_argument("spec")
    sheet_parser.add_argument("--runs", type=int, required=True)
    sheet_parser.add_argument("--out", required=True)
    sheet_parser.set_defaults(func=_cmd_runsheet)

    record_parser = sub.add_parser("record", help="write physical_reference.json")
    record_parser.add_argument("--spec", required=True)
    record_parser.add_argument("--run-id", required=True)
    record_parser.add_argument("--start-residual-m", required=True)
    record_parser.add_argument("--end-residual-m", required=True)
    record_parser.add_argument("--heading", required=True)
    record_parser.add_argument("--operator", required=True)
    record_parser.add_argument("--method", required=True)
    record_parser.add_argument("--notes", default="")
    record_parser.add_argument("--out", required=True)
    record_parser.set_defaults(func=_cmd_record)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)
