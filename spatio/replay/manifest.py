#!/usr/bin/env python3
"""Write manifest.json for a Spatio session folder (shared file contract).

The manifest holds exactly the contract keys. Run details that don't belong
in the contract go to logs/run_params.json (written by replay.sh).

    manifest.py SESSION_DIR --run-id ID --source-recording PATH --sensor S
        --fastlio-revision REV --config-file PATH --image ID --notes TEXT
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import os
import sys
from pathlib import Path

# replay.sh invokes this file directly, including from outside the repo root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from spatio.manifest.jsonio import write_json_atomic

CONTRACT_KEYS = (
    "run_id", "source_recording", "sensor", "fastlio_revision", "config_file",
    "config_sha256", "image", "created_at", "notes", "physical_accuracy_evaluated",
)
HISTORICAL_LABEL = "replay demonstrated; physical accuracy not evaluated"


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def build_manifest(run_id, source_recording, sensor, fastlio_revision,
                   config_file, config_sha256, image, notes, created_at=None):
    """Contract manifest. Historical recordings never claim physical accuracy."""
    if not run_id or "/" in run_id:
        raise ValueError(f"bad run_id {run_id!r}")
    if HISTORICAL_LABEL not in notes:
        notes = f"{HISTORICAL_LABEL}. {notes}".strip()
    m = {
        "run_id": run_id,
        "source_recording": source_recording,
        "sensor": sensor,
        "fastlio_revision": fastlio_revision,
        "config_file": config_file,
        "config_sha256": config_sha256,
        "image": image,
        "created_at": created_at or _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "notes": notes,
        "physical_accuracy_evaluated": False,
    }
    assert tuple(m) == CONTRACT_KEYS
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session_dir")
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--source-recording", required=True)
    ap.add_argument("--sensor", required=True)
    ap.add_argument("--fastlio-revision", required=True)
    ap.add_argument("--config-file", required=True,
                    help="config path; hashed if it exists, else config_sha256 is null")
    ap.add_argument("--image", required=True)
    ap.add_argument("--notes", default="")
    args = ap.parse_args()

    cfg_sha = sha256_file(args.config_file) if os.path.isfile(args.config_file) else None
    m = build_manifest(args.run_id, args.source_recording, args.sensor,
                       args.fastlio_revision, args.config_file, cfg_sha,
                       args.image, args.notes)
    write_json_atomic(os.path.join(args.session_dir, "manifest.json"), m)
    print(f"wrote {args.session_dir}/manifest.json")


if __name__ == "__main__":
    main()
