"""Command line for `python3 -m spatio.manifest`."""

from __future__ import annotations

import argparse
import json
import sys

from spatio.manifest.build import build_manifest
from spatio.manifest.findings import ManifestBuildError, ManifestUnreadable
from spatio.manifest.validate import validate_session


def _parser():
    parser = argparse.ArgumentParser(prog="spatio.manifest")
    subparsers = parser.add_subparsers(dest="command", required=True)

    build = subparsers.add_parser("build", help="write manifest.json")
    build.add_argument("session_dir")
    build.add_argument("--run-id", required=True)
    build.add_argument("--source-recording", required=True)
    build.add_argument("--sensor", required=True)
    build.add_argument("--fastlio-revision", required=True)
    build.add_argument("--config-file", required=True)
    build.add_argument("--image", required=True)
    build.add_argument("--notes", required=True)
    build.add_argument("--physical-reference", default=None)
    build.add_argument("--hash-bag", action="store_true")
    build.add_argument("--force", action="store_true")

    validate = subparsers.add_parser("validate", help="check session folders")
    validate.add_argument("session_dirs", nargs="+")
    validate.add_argument("--json", action="store_true")
    return parser


def _cmd_build(args):
    try:
        manifest = build_manifest(
            args.session_dir,
            args.run_id,
            args.source_recording,
            args.sensor,
            args.fastlio_revision,
            args.config_file,
            args.image,
            args.notes,
            physical_reference=args.physical_reference,
            hash_bag=args.hash_bag,
            force=args.force,
        )
    except ManifestUnreadable as exc:
        print("ERROR IO %s" % exc, file=sys.stderr)
        return 2
    except ManifestBuildError as exc:
        print("ERROR build %s" % exc, file=sys.stderr)
        return 1
    except OSError as exc:
        print("ERROR IO %s" % exc, file=sys.stderr)
        return 2
    print("wrote %s/manifest.json" % str(args.session_dir).rstrip("/"))
    print("config_sha256 %s" % manifest["config_sha256"])
    print("physical_accuracy_evaluated %s" % manifest["physical_accuracy_evaluated"])
    return 0


def _cmd_validate(args):
    rows = []
    saw_unreadable = False
    saw_error = False
    for session in args.session_dirs:
        try:
            findings = validate_session(session)
        except ManifestUnreadable as exc:
            saw_unreadable = True
            rows.append((session, "ERROR", "IO", str(exc)))
            continue
        for finding in findings:
            if finding.severity == "ERROR":
                saw_error = True
            rows.append((session, finding.severity, finding.rule, finding.message))
    if args.json:
        payload = [
            {
                "severity": severity,
                "rule": rule,
                "message": message,
                "session": str(session),
            }
            for session, severity, rule, message in rows
        ]
        json.dump(payload, sys.stdout, indent=2)
        sys.stdout.write("\n")
    else:
        for session, severity, rule, message in rows:
            print("%s %s %s: %s" % (severity, rule, session, message))
    if saw_unreadable:
        return 2
    if saw_error:
        return 1
    return 0


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    if args.command == "build":
        return _cmd_build(args)
    if args.command == "validate":
        return _cmd_validate(args)
    parser.error("unknown command")
    return 2
