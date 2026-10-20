"""F7a/F8a presentation of existing evidence; never computes a new clearance."""

import argparse
import copy
import html
import json
import math
from pathlib import Path

from .classify import validate_min_free_rays
from .model import header, load_json, number, parse_zones, text, vector
from . import provenance as prov


SCHEMA = "spatio.clearance_report.v1"
STATUSES = ("PASS", "FAIL", "NOT_MEASURED", "INDETERMINATE")
SETTINGS = ("voxel_size_m", "grid_origin", "uncertainty_m", "uncertainty_status",
            "min_free_rays", "minimum_evidence_status")


def json_text(value):
    return json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True,
                      allow_nan=False) + "\n"


def _count(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _time_range(value, name):
    if value is None:
        return
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{name} must be null or [start, end]")
    if number(value[0], name) > number(value[1], name):
        raise ValueError(f"{name} must be ordered")


def build_report(results, zones_data, walk_id, supplied=None):
    """Join result records to their map geometry; reject inconsistent summaries.

    supplied: sha256 of the current recording, extrinsics, trajectory and scans
    (None = not given). A changed trajectory, extrinsics or zone geometry is a
    hard error; see provenance.check."""
    walk_id = text(walk_id, "walk_id")
    # The classifier requires a nonempty scene. Presentation also supports a
    # walk with no declared zones, without weakening that classifier contract.
    frame = header(zones_data, "zones")
    definitions = parse_zones(zones_data)[1] if zones_data["zones"] else []
    if not isinstance(results, dict):
        raise ValueError("results must be an object")
    try:
        json_text(results)  # Reject nonfinite numbers, including extension fields.
        if type(results["schema_version"]) is not int or results["schema_version"] != 1:
            raise ValueError("results schema_version must be integer 1")
        if results["frame_id"] != frame:
            raise ValueError("results and zones frame_id must match")
        if results["physical_accuracy_evaluated"] is not False:
            raise ValueError("this output stage requires physical_accuracy_evaluated: false")
        text(results["claim"], "claim")
        if number(results["voxel_size_m"], "voxel_size_m") <= 0:
            raise ValueError("voxel_size_m must be positive")
        vector(results["grid_origin"], "grid_origin")
        if number(results["uncertainty_m"], "uncertainty_m") < 0:
            raise ValueError("uncertainty_m must be nonnegative")
        validate_min_free_rays(results["min_free_rays"])
        for key in ("uncertainty_status", "minimum_evidence_status"):
            text(results[key], key)
        scan_count = _count(results["scan_count"], "scan_count")
        _time_range(results["scan_time_range"], "scan_time_range")
        if (scan_count == 0) != (results["scan_time_range"] is None):
            raise ValueError("scan_count and scan_time_range disagree")
        if not isinstance(results["zones"], list):
            raise ValueError("results zones must be a list")
        records = {}
        for record in results["zones"]:
            if not isinstance(record, dict):
                raise ValueError("each zone result must be an object")
            zid = text(record["id"], "zone id")
            if zid in records:
                raise ValueError(f"duplicate result zone id: {zid}")
            records[zid] = record
        if set(records) != {z.id for z in definitions}:
            raise ValueError("result and definition zone IDs must match exactly")
        zones = []
        for definition in sorted(definitions, key=lambda z: z.id):
            record = records[definition.id]
            for key in ("shape", "rule_id", "provision"):
                if record[key] != getattr(definition, key):
                    raise ValueError(f"{definition.id}: {key} differs from zone definition")
            if number(record["required_depth_m"], "required_depth_m") != definition.depth:
                raise ValueError(f"{definition.id}: required depth differs from zone definition")
            if record["status"] not in STATUSES:
                raise ValueError("unknown zone status")
            text(record["reason"], "reason")
            for key in ("min_clearance_m", "occupied_voxel_clearance_lower_bound_m",
                        "decision_margin_m"):
                if record[key] is not None:
                    number(record[key], key)
            for key in ("uncertainty_m", "min_free_rays"):
                if type(record[key]) is bool or record[key] != results[key]:
                    raise ValueError(f"zone {key} differs from result settings")
            counts = {key: _count(record[key], key) for key in (
                "required_cells", "free_only_cells", "occupied_cells", "unknown_cells",
                "insufficient_evidence_cells")}
            required = counts["required_cells"]
            if required < 1 or required != sum(counts[k] for k in (
                    "free_only_cells", "occupied_cells", "unknown_cells")):
                raise ValueError("zone evidence counts disagree")
            if not counts["unknown_cells"] <= counts["insufficient_evidence_cells"] <= (
                    required - counts["occupied_cells"]):
                raise ValueError("insufficient evidence count disagrees")
            if not isinstance(record["conflicting_cells"], list):
                raise ValueError("conflicting_cells must be a list")
            if len(record["conflicting_cells"]) > counts["occupied_cells"]:
                raise ValueError("conflict count exceeds occupancy")
            coverage = number(record["free_coverage"], "free_coverage")
            if coverage != counts["free_only_cells"] / required:
                raise ValueError("free coverage disagrees with evidence counts")
            _time_range(record["evidence_time_range"], "evidence_time_range")
            times = record["evidence_time_range"]
            if times is not None and (scan_count == 0 or
                    times[0] < results["scan_time_range"][0] or
                    times[1] > results["scan_time_range"][1]):
                raise ValueError("zone evidence time lies outside scan range")
            measured = record["min_clearance_m"]
            lower = record["occupied_voxel_clearance_lower_bound_m"]
            margin = record["decision_margin_m"]
            uncertainty = record["uncertainty_m"]
            # Match the classifier's positive-uncertainty boundary tolerance.
            pass_margin = uncertainty + (1e-12 if uncertainty > 0 else 0)
            if record["status"] == "PASS" and (
                    counts["free_only_cells"] != required or
                    counts["insufficient_evidence_cells"] != 0 or
                    record["conflicting_cells"] or times is None or scan_count == 0 or
                    lower is not None or
                    (margin is not None and margin <= pass_margin)):
                raise ValueError(f"{definition.id}: PASS lacks full free evidence")
            if not counts["occupied_cells"]:
                expected_margin = None if measured is None else measured - definition.depth
                if lower is not None or margin != expected_margin:
                    raise ValueError(f"{definition.id}: clearance and decision margin disagree")
            else:
                # Occupied decisions use in-volume hits or conservative voxel
                # bounds, not necessarily the expanded-footprint measured hit.
                # Only bounds are recoverable from this summary.
                if (lower is None or margin is None or
                        not 0 <= lower <= definition.depth or
                        not lower - definition.depth <= margin <= 0 or
                        (measured is None and margin != lower - definition.depth)):
                    raise ValueError(f"{definition.id}: occupied clearance and decision margin disagree")
            if record["zone_geometry_sha256"] != prov.zone_geometry_sha256(frame, definition):
                raise ValueError(f"{definition.id}: zone geometry changed since clearance was "
                                 "computed; recompute clearance")
            if record["provenance_limitations"] != results["provenance"]["limitations"]:
                raise ValueError(f"{definition.id}: provenance limitations differ from the result's")
            # Divide before summing to avoid overflowing large finite bounds.
            position = (definition.low / 2 + definition.high / 2).tolist()
            if not all(math.isfinite(x) for x in position):
                raise ValueError("zone centre must be finite")
            zones.append({"map_position_m": position,
                          "evidence": copy.deepcopy(record)})
        reuse_limitations = prov.check(results["provenance"], supplied or {})
    except KeyError as error:
        raise ValueError(f"missing result field: {error.args[0]}") from error
    counts = {status: sum(z["evidence"]["status"] == status for z in zones)
              for status in STATUSES}
    return {
        "schema": SCHEMA, "walk_id": walk_id, "frame_id": frame,
        "claim": results["claim"], "physical_accuracy_evaluated": False,
        "settings": {key: copy.deepcopy(results[key]) for key in SETTINGS},
        "scan_count": scan_count, "scan_time_range": copy.deepcopy(results["scan_time_range"]),
        "provenance": copy.deepcopy(results["provenance"]),
        "reuse_limitations": reuse_limitations,
        "position_reference": "Centre of required volume; not an obstacle or viewing pose.",
        "counts": counts, "zone_count": len(zones), "zones": zones,
        "failures": [z for z in zones if z["evidence"]["status"] == "FAIL"],
        "rechecks": [dict(number=i, **z) for i, z in enumerate(
            (z for z in zones if z["evidence"]["status"] in (
                "NOT_MEASURED", "INDETERMINATE")), start=1)],
    }


def _safe(value):
    # Keep supplied text literal, single-line and safe inside Markdown tables.
    escaped = html.escape(str(value), quote=False)
    for character in "\\`*_{}[]()#+-.!|":
        escaped = escaped.replace(character, "\\" + character)
    return " ".join(escaped.split())


def _value(value):
    return "not measured" if value is None else str(value)


def markdown_report(result):
    settings = result["settings"]
    lines = [
        "# One-walk clearance exception report", "",
        f"Walk: {_safe(result['walk_id'])} · Frame: {_safe(result['frame_id'])}",
        _safe(result["claim"]),
        "Physical accuracy not evaluated. Supplied rules/settings are not an approval.",
        f"Settings: voxel={settings['voxel_size_m']} m; origin={settings['grid_origin']}; "
        f"uncertainty=±{settings['uncertainty_m']} m ({_safe(settings['uncertainty_status'])}); "
        f"minimum free rays/cell={settings['min_free_rays']} "
        f"({_safe(settings['minimum_evidence_status'])}).",
        f"Scans: {result['scan_count']}; scan time range (input seconds): {result['scan_time_range']}.",
        "Counts: " + "; ".join(f"{s.replace('_', ' ')}={result['counts'][s]}" for s in STATUSES) + ".",
        "Map positions (x, y, z), metres: required-volume centres, not obstacle/viewing poses.",
    ]
    p = result["provenance"]
    lines += [
        f"Inputs (sha256): recording {p['recording_sha256']}; extrinsics {p['extrinsics_sha256']}; "
        f"trajectory {p['trajectory_sha256']}; scans {p['scans_sha256']}.",
        f"Time basis: {_safe(p['time_basis'])}.",
    ]
    limits = [(key, prov.LIMITATIONS[key]) for key in p["limitations"]] + [
        (key, prov.REUSE_LIMITATIONS[key]) for key in result["reuse_limitations"]]
    if limits:
        lines.append("Provenance limitations (apply to every zone, including PASS):")
        lines += [f"- {_safe(key)}: {_safe(detail)}" for key, detail in limits]
    lines += ["", "## FAIL exceptions", ""]
    if result["failures"]:
        lines += [
            "| Zone / rule | Observed min hit / required min (m) | Occupied voxel lower bound (m) | Map position | Evidence time | Reason / supplied provision |",
            "|---|---|---|---|---|---|",
        ]
        for item in result["failures"]:
            z = item["evidence"]
            lines.append(
                f"| {_safe(z['id'])} / {_safe(z['rule_id'])} | "
                f"{_value(z['min_clearance_m'])} / {z['required_depth_m']} | "
                f"{_value(z['occupied_voxel_clearance_lower_bound_m'])} | "
                f"{item['map_position_m']} | {z['evidence_time_range']} | "
                f"{_safe(z['reason'])}; {_safe(z['provision'])} |")
    else:
        lines.append("None. This does not mean all zones passed.")
    lines += ["", "## Numbered recheck list", ""]
    for item in result["rechecks"]:
        z = item["evidence"]
        lines.append(
            f"{item['number']}. {_safe(z['id'])} / {_safe(z['rule_id'])} — "
            f"{z['status'].replace('_', ' ')} at {item['map_position_m']}: "
            f"{_safe(z['reason'])}. Evidence time: {z['evidence_time_range']}.")
    if not result["rechecks"]:
        lines.append("None.")
    if result["zone_count"] == 0:
        lines += ["", "No zones supplied; no clearance conclusions."]
    lines += ["", "Recheck order is by zone ID, not a route. Null time means no evidence. "
              "A voxel bound is not a measured hit distance; full evidence is retained in JSON."]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True, help="existing clearance report.json")
    parser.add_argument("--zones", type=Path, required=True, help="matching zone definitions")
    parser.add_argument("--walk-id", required=True)
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    parser.add_argument("--trajectory", type=Path,
                        help="current trajectory; required when the results are bound to one")
    parser.add_argument("--extrinsics", type=Path,
                        help="current extrinsics; required when the results are bound to them")
    parser.add_argument("--recording", type=Path, help="recording to recheck (optional)")
    parser.add_argument("--scans", type=Path, help="scans to recheck (optional)")
    args = parser.parse_args(argv)
    try:
        supplied = {name: None if path is None else prov.sha256_path(path) for name, path in (
            ("recording", args.recording), ("extrinsics", args.extrinsics),
            ("trajectory", args.trajectory))}
        supplied["scans"] = None if args.scans is None else prov.canonical_sha256(
            load_json(args.scans))
        result = build_report(load_json(args.results), load_json(args.zones), args.walk_id,
                              supplied)
        encoded, rendered = json_text(result), markdown_report(result)
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output / "report.json").write_bytes(encoded.encode("utf-8"))
        (args.output / "report.md").write_bytes(rendered.encode("utf-8"))
    except (ValueError, TypeError, OSError, OverflowError) as error:
        parser.error(str(error))
    print(rendered, end="")
    return 0
