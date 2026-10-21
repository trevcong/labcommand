"""Classify a supplied patch file before process.sh uses it as preselection."""

import argparse
import json
import sys
from pathlib import Path

from spatio.manifest.jsonio import write_json_atomic
from spatio.wall_scoring import _same_box
from spatio.wallpatch.pcd import sha256_file


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


def bind_spec_boxes(document, spec_path, map_path):
    """Give a sha-less synthetic preselected file this run's map sha256.

    Allowed only when the frozen spec fixes every patch box and each supplied
    box is the spec's box: those boxes were chosen before any run, so they hold
    on every synthetic run's map. The caller must establish that the recording
    is synthetic. A file that names a map_sha256 keeps it.
    """
    if document.get("map_sha256") is not None:
        return
    spec = json.loads(Path(spec_path).read_text(encoding="utf-8"))
    boxes = {p["name"].strip(): p.get("box_map_frame") for p in spec["wall_patches"]}
    for patch in document["patches"]:
        box = boxes[patch["name"].strip()]
        if box is None or not _same_box(box, patch.get("box_map_frame")):
            raise ValueError(f"patch {patch['name']}: no map_sha256, and its box is not "
                             "the frozen spec's fixed box")
    document["map_sha256"] = sha256_file(map_path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("patches")
    parser.add_argument("--spec")
    parser.add_argument("--map", help="this run's map.pcd: binds synthetic sha-less spec boxes")
    parser.add_argument("--synthetic", action="store_true",
                        help="recording is marked SYNTHETIC.json; allow binding sha-less spec boxes")
    args = parser.parse_args(argv)
    try:
        document = json.loads(Path(args.patches).read_text(encoding="utf-8"))
        origin = classify(document, args.spec)
        if origin == "preselected":
            if document.get("map_sha256") is None and not args.synthetic:
                raise ValueError("preselected patches require map_sha256 for real recordings")
            if args.map:
                bind_spec_boxes(document, args.spec, args.map)
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
