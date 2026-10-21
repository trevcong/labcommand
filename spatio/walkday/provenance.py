"""Walk-day provenance: the effective settings identity and the configs a run used.

    python3 -m spatio.walkday.provenance settings NAME=VALUE ...
        check the walk-day settings and print their identity (sha256);
        exit 3 if a setting is missing, empty or unknown
    python3 -m spatio.walkday.provenance write OUT --rec REC --config CFG
        [--driver-config DCFG] NAME=VALUE ...
        validate the driver config and write OUT (logs/provenance.json);
        exit 2 if the driver config cannot belong to this recording, 3 on bad input

The settings identity is the sha256 of the canonical JSON of the values
process.sh actually used (after sourcing the settings file), so comment edits
do not change it and "0.10" equals "0.1". A changed value starts a new series
(enforced by spatio.walkday.seriesgate). spatio.walkday.result checks the
identity against the values the walk check and the scoring really used, and
recomputes it when one differs (a re-scored run is then not stage 1's identity).

The driver config is the sensor driver's own config, upstream of FAST-LIO (for
the JT128: the Hesai driver's jt128_post.yaml). Replay does not read it: the bag
already holds the driver's (or an adapter's) output. It is recorded and checked
for provenance only: readable text, not a FAST-LIO config, not the FAST-LIO
config file itself, and at least one ROS topic it declares is in the bag.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys

from spatio.manifest.jsonio import write_json_atomic
from spatio.replay.recording import RecordingError, read_metadata

SCHEMA = "spatio.walkday.provenance/1"
# Every walkday.conf value process.sh passes to a stage. All are part of the
# identity; the review's minimum was hold limits, endpoint limits, playback rate
# and map voxel size.
SETTINGS_KEYS = (
    "STATIONARY_WINDOW_S", "MAX_GAP_S", "MIN_PATH_LENGTH_M", "MIN_EXCURSION_M",
    "MAX_ENDPOINT_SPREAD_M", "MAX_ENDPOINT_SPEED_M_S", "MAX_HOLD_SPEED_M_S", "MIN_HOLD_S",
    "WALL_GATE_M", "WALL_GATE_STATE", "SUGGEST_PATCHES", "BAGTIMING_SAMPLE_EVERY",
    "MAP_VOXEL_M", "PLAYBACK_RATE",
)
# Settings spatio.walkday.result can check after the fact, against the walk
# checker's own record (walkcheck.json "parameters", stage 6). WALL_GATE_M and
# WALL_GATE_STATE are checked against result.py's own arguments. SUGGEST_PATCHES,
# BAGTIMING_SAMPLE_EVERY, MAP_VOXEL_M and PLAYBACK_RATE leave no record result.py
# reads, so stage 1's values stand for them.
WALKCHECK_PARAMS = {
    "STATIONARY_WINDOW_S": "stationary_window_s", "MAX_GAP_S": "max_gap_s",
    "MIN_PATH_LENGTH_M": "min_path_length_m", "MIN_EXCURSION_M": "min_excursion_m",
    "MAX_ENDPOINT_SPREAD_M": "max_endpoint_spread_m", "MAX_ENDPOINT_SPEED_M_S": "max_endpoint_speed_m_s",
    "MAX_HOLD_SPEED_M_S": "max_hold_speed_m_s", "MIN_HOLD_S": "min_hold_s",
}
DRIVER_NOTE = ("not read by the FAST-LIO replay (the bag already holds the driver's or an adapter's "
               "output); recorded for provenance and series consistency only")
_TOPIC = re.compile(r"""^\s*[A-Za-z0-9_]*topic[A-Za-z0-9_]*\s*:\s*["']?(/[^"'\s#]+)""", re.M)


class ProvenanceError(Exception):
    pass


def _value(text):
    try:
        v = float(text)
    except ValueError:
        return text
    return v if math.isfinite(v) else text


def settings_identity(pairs):
    """{"values": {...}, "sha256": hex} from NAME=VALUE strings or a dict."""
    raw = dict(p.split("=", 1) for p in pairs) if not isinstance(pairs, dict) else dict(pairs)
    unknown = sorted(set(raw) - set(SETTINGS_KEYS))
    missing = [k for k in SETTINGS_KEYS if str(raw.get(k, "")).strip() == ""]
    if unknown:
        raise ProvenanceError("unknown walk-day settings: " + ", ".join(unknown))
    if missing:
        raise ProvenanceError("walk-day settings missing or empty: " + ", ".join(missing))
    values = {k: _value(str(raw[k]).strip()) for k in SETTINGS_KEYS}
    return {"values": values, "sha256": _sha_json(values)}


def _sha_json(values):
    canon = json.dumps(values, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canon.encode("utf-8")).hexdigest()


def check_identity(identity):
    """True if identity is a settings identity whose sha256 matches its values."""
    if not isinstance(identity, dict) or not isinstance(identity.get("values"), dict):
        return False
    try:
        return settings_identity(identity["values"]) == identity
    except (ProvenanceError, AttributeError, ValueError, TypeError):
        return False


def effective_identity(identity, used):
    """Stage 1's identity with the values a later stage actually used.

    used: {NAME: (value, source)}. Returns (identity, changes): stage 1's own
    identity when every value matches, else a recomputed one, with one
    {setting, stage1, used, source} per changed value."""
    values, changes = dict(identity["values"]), []
    for key, (value, source) in used.items():
        v = _value(str(value).strip())
        if v != values[key]:
            changes.append({"setting": key, "stage1": values[key], "used": v, "source": source})
            values[key] = v
    return (identity, []) if not changes else (settings_identity(values), changes)


def _sha_file(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def driver_config(path, fastlio_config, rec):
    try:
        with open(path, "rb") as f:
            raw = f.read()
        text = raw.decode("utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise ProvenanceError(f"driver config {path} is not readable text: {exc}")
    if not text.strip():
        raise ProvenanceError(f"driver config {path} is empty")
    sha = hashlib.sha256(raw).hexdigest()
    if sha == _sha_file(fastlio_config):
        raise ProvenanceError(f"driver config {path} is the FAST-LIO config itself ({fastlio_config})")
    if re.search(r"^\s*lid_topic\s*:", text, re.M):
        raise ProvenanceError(f"driver config {path} has lid_topic: it is a FAST-LIO config, "
                              "not the sensor driver's config")
    declared = sorted(set(_TOPIC.findall(text)))
    if not declared:
        raise ProvenanceError(f"driver config {path} declares no ROS topic, so it cannot be tied to this recording")
    try:
        topics = read_metadata(rec)["topics"]
    except RecordingError as exc:
        raise ProvenanceError(f"cannot read the bag's topics: {exc}")
    in_bag = [t for t in declared if t in topics]
    if not in_bag:
        raise ProvenanceError(f"none of the driver config's topics ({', '.join(declared)}) is in the bag "
                              f"({', '.join(sorted(topics))}); it does not belong to this recording")
    return {"file": os.path.abspath(path), "sha256": sha, "topics_declared": declared,
            "topics_in_bag": in_bag, "note": DRIVER_NOTE}


def build(rec, config, settings_pairs, driver=None):
    return {
        "schema": SCHEMA,
        "settings": settings_identity(settings_pairs),
        "fastlio_config": {"file": os.path.abspath(config), "sha256": _sha_file(config)},
        "driver_config": None if driver is None else driver_config(driver, config, rec),
    }


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if not argv or argv[0] not in ("settings", "write"):
        print(__doc__, file=sys.stderr)
        return 3
    # One parser per command: parse_intermixed_args (NAME=VALUE after the
    # options) does not support subparsers.
    ap = argparse.ArgumentParser(prog="spatio.walkday.provenance " + argv[0])
    if argv[0] == "write":
        ap.add_argument("out")
        ap.add_argument("--rec", required=True)
        ap.add_argument("--config", required=True)
        ap.add_argument("--driver-config")
    ap.add_argument("pairs", nargs="*")
    a = ap.parse_intermixed_args(argv[1:])
    a.cmd = argv[0]
    bad = [p for p in a.pairs if "=" not in p]
    if bad:
        print(f"provenance: expected NAME=VALUE, got {bad}", file=sys.stderr)
        return 3
    try:
        if a.cmd == "settings":
            print(settings_identity(a.pairs)["sha256"])
            return 0
        identity = settings_identity(a.pairs)   # a settings problem is a setup error (3)
    except ProvenanceError as exc:
        print(f"provenance: {exc}", file=sys.stderr)
        return 3
    try:
        data = build(a.rec, a.config, identity["values"], a.driver_config)
    except (ProvenanceError, OSError) as exc:
        print(f"provenance: {exc}", file=sys.stderr)
        return 2
    write_json_atomic(a.out, data)
    print(f"walk-day settings sha256 {data['settings']['sha256']}")
    print(f"FAST-LIO config {data['fastlio_config']['file']} sha256 {data['fastlio_config']['sha256']}")
    d = data["driver_config"]
    print("driver config: none given" if d is None else
          f"driver config {d['file']} sha256 {d['sha256']} (topics in the bag: {', '.join(d['topics_in_bag'])}; {d['note']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
