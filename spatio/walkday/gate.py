"""Input gate: decide from x1's bagtiming.json whether a recording can be scored.

    python3 -m spatio.walkday.gate BAGTIMING_JSON --lidar T --imu T --out GATE_JSON

Exit 0 = evaluable (warnings allowed), 2 = NOT-EVALUABLE, 3 = unusable input.

Blocking rules (generic; nothing is fitted to a particular recording):
  B1 the lidar and IMU topics named by the FAST-LIO config are in the bag
     and have messages
  B2 no x1 finding of severity "error" (x1 raises CLOCK_RESET as an error)
  B3 no backward jump, reset or zero stamp in lidar or IMU header time
  B4 the inspection covered the whole bag (no PARTIAL_INSPECTION)
  B5 at least one lidar scan has per-point time bounds that x1 can check
     against the IMU stream (otherwise IMU coverage of scans is unknowable)

Reported, not blocking (their effect on the trajectory is gated later by the
walk checker's gap/continuity rules): header gaps, scans the IMU does not fully
cover, unknown clock domain (x1 can never prove it from a bag alone),
receipt-time gaps, storage-order notes.
"""

from __future__ import annotations

import argparse
import json
import sys

from spatio.manifest.jsonio import write_json_atomic

SCHEMA = "spatio.walkday.input_gate/1"


def evaluate(bt: dict, lidar: str, imu: str) -> dict:
    blocking, warnings = [], []
    topics = {t.get("name"): t for t in bt.get("topics", [])}
    for role, name in (("lidar", lidar), ("imu", imu)):
        t = topics.get(name)
        if t is None:
            blocking.append(f"B1 {role} topic {name} not inspected / not in the bag")
            continue
        if not t.get("count"):
            blocking.append(f"B1 {role} topic {name} has no messages")
            continue
        h = t.get("header") or {}
        for key in ("backward_jumps", "zero_stamps"):
            if h.get(key):
                blocking.append(f"B3 {name} header time: {h[key]} {key}")
        if h.get("resets"):
            blocking.append(f"B3 {name} header time: {len(h['resets'])} resets")
        if h.get("gap_count"):
            warnings.append(f"{name} header time: {h['gap_count']} gaps, largest {h.get('max_gap_s')} s")
    for f in bt.get("findings", []):
        if f.get("severity") == "error":
            blocking.append(f"B2 x1 error {f.get('code')}: {f.get('text')}")
        elif f.get("code") == "PARTIAL_INSPECTION":
            blocking.append(f"B4 {f.get('text')}")
        elif f.get("code") in ("IMU_COVERAGE", "UNKNOWN_CLOCK_DOMAIN", "POINT_TIME_DISCONTINUITY",
                               "UNKNOWN_SCAN_BOUNDS", "NO_POINT_TIMING", "POINT_TIMING_SAMPLED"):
            if lidar in f.get("text", "") or imu in f.get("text", "") or f.get("code") == "POINT_TIMING_SAMPLED":
                warnings.append(f"{f.get('code')}: {f.get('text')}")
    cover = None
    for c in bt.get("imu_coverage", []):
        if c.get("lidar_topic") == lidar and c.get("imu_topic") == imu:
            cover = c
    if cover is None:
        if lidar in topics and imu in topics:
            blocking.append(f"B5 x1 reported no IMU coverage entry for {lidar} vs {imu}")
    elif not cover.get("evaluable_scans"):
        blocking.append(f"B5 none of {cover.get('scans')} scans on {lidar} has checkable point "
                        f"time bounds; IMU coverage of scans is unknown")
    return {
        "schema": SCHEMA,
        "evaluable": not blocking,
        "lidar_topic": lidar,
        "imu_topic": imu,
        "blocking": blocking,
        "warnings": warnings,
        "imu_coverage": cover,
        "rules": __doc__.split("Blocking rules", 1)[1].strip(),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="spatio.walkday.gate")
    ap.add_argument("bagtiming_json")
    ap.add_argument("--lidar", required=True)
    ap.add_argument("--imu", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    try:
        with open(args.bagtiming_json, encoding="utf-8") as f:
            bt = json.load(f)
    except (OSError, ValueError) as exc:
        print(f"gate: cannot read {args.bagtiming_json}: {exc}", file=sys.stderr)
        return 3
    if not isinstance(bt, dict) or bt.get("schema") != "spatio.bagtiming/1":
        print("gate: not a spatio.bagtiming/1 report", file=sys.stderr)
        return 3
    out = evaluate(bt, args.lidar, args.imu)
    write_json_atomic(args.out, out)
    for line in out["blocking"]:
        print(f"INPUT BLOCKED: {line}", file=sys.stderr)
    for line in out["warnings"]:
        print(f"input warning: {line}")
    return 0 if out["evaluable"] else 2


if __name__ == "__main__":
    sys.exit(main())
