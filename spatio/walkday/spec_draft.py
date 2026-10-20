"""Draft a g5 experiment spec with the builder-side fields filled from files.

    python3 -m spatio.walkday.spec_draft --config CFG --fastlio-revision REV --out DRAFT.json
        [--fill FIELDS.json]

Fills config_file, config_sha256 (hashed from CFG), fastlio_revision and
frames.lidar_to_imu (quoted from CFG's extrinsic). Everything Trevor decides
stays TODO(Trevor) unless --fill supplies it (a JSON object merged on top;
used for synthetic dry-run specs). Then: edit, `spatio.expspec validate`,
`spatio.expspec freeze`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys

from spatio.expspec.template import build_template


def _merge(base, extra):
    for k, v in extra.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v


def draft(config_path, revision, fill=None):
    with open(config_path, "rb") as f:
        raw = f.read()
    text = raw.decode("utf-8", errors="replace")
    t = re.search(r"extrinsic_T:\s*\[([^\]]*)\]", text)
    r = re.search(r"extrinsic_R:\s*\[([^\]]*)\]", text, re.S)
    est = re.search(r"extrinsic_est_en:\s*(\w+)", text)
    spec = build_template()
    spec["config_file"] = config_path
    spec["config_sha256"] = hashlib.sha256(raw).hexdigest()
    spec["fastlio_revision"] = revision
    spec["frames"]["lidar_to_imu"] = (
        "from the config: extrinsic_T [%s], extrinsic_R [%s], extrinsic_est_en %s"
        % (" ".join(t.group(1).split()) if t else "?", " ".join(r.group(1).split()) if r else "?",
           est.group(1) if est else "?"))
    if fill:
        _merge(spec, fill)
    return spec


def main(argv=None):
    ap = argparse.ArgumentParser(prog="spatio.walkday.spec_draft", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--fastlio-revision", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fill")
    a = ap.parse_args(argv)
    fill = None
    if a.fill:
        with open(a.fill) as f:
            fill = json.load(f)
    spec = draft(a.config, a.fastlio_revision, fill)
    with open(a.out, "w") as f:
        json.dump(spec, f, indent=2)
        f.write("\n")
    print(a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
