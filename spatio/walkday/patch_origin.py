"""Classify a supplied patch file before process.sh uses it as preselection."""

import argparse
import json
import sys
from pathlib import Path

from spatio.manifest.jsonio import write_json_atomic


def classify(document, spec_path=None):
    patches = document.get("patches") if isinstance(document, dict) else None
    if not isinstance(patches, list) or not patches or any(not isinstance(p, dict) for p in patches):
        raise ValueError("patches must be a non-empty list of objects")
    if any(p.get("origin") not in (None, "preselected", "auto_suggested") for p in patches):
        raise ValueError("unknown patch origin")
    if any(p.get("origin") == "auto_suggested" for p in patches):
        return "auto_suggested"
    if not spec_path:
        raise ValueError("preselected patches require --spec with matching wall_patches")
    spec = json.loads(Path(spec_path).read_text(encoding="utf-8"))
    required = spec.get("wall_patches") if isinstance(spec, dict) else None

    def names(items):
        if not isinstance(items, list) or not items:
            raise ValueError("spec and patches must have non-empty named patch lists")
        out = [p.get("name") if isinstance(p, dict) else None for p in items]
        if any(not isinstance(n, str) or not n.strip() for n in out):
            raise ValueError("every patch must have a non-blank name")
        out = [n.strip() for n in out]
        if len(set(out)) != len(out):
            raise ValueError("duplicate patch names cannot establish preselection")
        return set(out)

    if names(patches) != names(required):
        raise ValueError("patch names do not match the frozen spec's wall_patches")
    return "preselected"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("patches")
    parser.add_argument("--spec")
    args = parser.parse_args(argv)
    try:
        document = json.loads(Path(args.patches).read_text(encoding="utf-8"))
        origin = classify(document, args.spec)
        if origin == "preselected":
            for patch in document["patches"]:
                patch["origin"] = origin
            write_json_atomic(args.patches, document)
        print(origin)
        return 0
    except (OSError, ValueError) as exc:
        print(f"patch origin: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
