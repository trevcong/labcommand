"""Explicit walkcheck-to-series conversion; no rescoring or inferred metadata."""

from __future__ import annotations

import json
import math
import os

from spatio.series.validate import (
    InputRejected, Unreadable, REPLAY_LABEL, WALK_SCHEMA, parse_walk_result,
)
from spatio.wall_scoring import WallScoringError, patch_provenance, score_walls, wall_verdict
from spatio.walkcheck.values import measurement_values


def read_json(path: str):
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, UnicodeError, ValueError) as exc:
        raise Unreadable(path, str(exc)) from exc


def adapt_walkcheck(report: dict, metadata: dict, source_path: str, *,
                    wallpatch=None, patch_origin="auto_suggested", spec_path=None,
                    patches_file=None, gate_m=0.05, gate_state="proposed") -> dict:
    """Map v1 fields, then apply the existing series validation rules.

    Metadata supplies the recording time, revision, configuration hash and
    physical reference that walkcheck does not carry. It must describe this run.
    The original producer verdict is retained separately. Wall acceptance uses
    exactly the same evidence and preselection checks as walk-day.
    """
    def reject(message):
        raise InputRejected(source_path, message)

    try:
        report = measurement_values(report)
    except ValueError as exc:
        reject(str(exc))

    if not isinstance(report, dict) or report.get("schema") != "spatio.walkcheck.v1":
        reject("adapter requires spatio.walkcheck.v1")
    if not isinstance(metadata, dict):
        reject("metadata must be an object")
    for key in ("run_id", "recorded_at", "fastlio_revision", "config_sha256",
                "physical_accuracy_evaluated", "physical_reference"):
        if key not in metadata:
            reject("metadata is missing %s" % key)

    def block(name, nullable=False):
        value = report.get(name)
        if name in report and value is None and nullable:
            return {}
        if not isinstance(value, dict):
            reject("%s must be an object%s" % (name, " or null" if nullable else ""))
        return value

    def number(value, name, scale=1.0):
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            reject("%s must be a finite number or null" % name)
        return value * scale

    producer_status = report.get("result")
    if producer_status not in ("PASS", "FAIL", "NOT-EVALUABLE"):
        reject("walkcheck result must be PASS, FAIL, or NOT-EVALUABLE")
    reasons = report.get("reasons")
    if not isinstance(reasons, list) or any(not isinstance(r, str) or not r.strip() for r in reasons):
        reject("walkcheck reasons must be a list of non-blank strings")
    reasons = list(reasons)
    if producer_status != "PASS" and not reasons:
        reject("walkcheck non-PASS requires reasons")

    manifest = block("manifest")
    if manifest.get("run_id") is not None and manifest["run_id"] != metadata["run_id"]:
        reject("metadata run_id disagrees with walkcheck manifest")
    physical = metadata["physical_accuracy_evaluated"]
    embedded_physical = manifest.get("physical_accuracy_evaluated")
    if (embedded_physical is not None and embedded_physical is not physical) or (
        physical is True and embedded_physical is not True
    ):
        reject("metadata physical_accuracy_evaluated disagrees with walkcheck manifest")

    parameters = block("parameters")
    returned = block("return_to_start", nullable=True)
    timestamps = block("timestamps", nullable=True)
    motion = block("motion", nullable=True)
    block("walls")
    drift = block("drift")
    backward = timestamps.get("backward_steps")
    if backward is not None and (
        isinstance(backward, bool) or not isinstance(backward, int) or backward < 0
    ):
        reject("timestamps.backward_steps must be a nonnegative integer or null")

    try:
        provenance = patch_provenance(
            patch_origin, spec_path, metadata["physical_reference"], wallpatch, patches_file,
            gate_m=gate_m,
        )
        p95, doubled, failures, notes, _, warnings = score_walls(
            report, wallpatch, gate_m, provenance,
            map_path=os.path.join(os.path.dirname(source_path), "map.pcd"),
        )
    except WallScoringError as exc:
        reject(str(exc))
    status, reasons = wall_verdict(producer_status, reasons, doubled, failures, notes)

    result = {
        "schema": WALK_SCHEMA,
        "run_id": metadata["run_id"],
        "recorded_at": metadata["recorded_at"],
        "status": status,
        "reasons": reasons,
        "physical_accuracy_evaluated": physical,
        "physical_reference": metadata["physical_reference"],
        "fastlio_revision": metadata["fastlio_revision"],
        "config_sha256": metadata["config_sha256"],
        "duration_s": timestamps.get("duration_s"),
        "path_length_m": motion.get("path_length_m"),
        "return_error": {
            "d3_m": returned.get("error_3d_m"),
            "horizontal_m": number(returned.get("error_horizontal_cm"), "error_horizontal_cm", 0.01),
            "vertical_m": number(returned.get("error_vertical_cm"), "error_vertical_cm", 0.01),
            "heading_deg": returned.get("heading_error_deg"),
            "threshold_m": parameters.get("return_pass_m"),
            "start_window_s": returned.get("start_window_s"),
            "end_window_s": returned.get("end_window_s"),
        },
        "drift_pct_of_path": drift.get("return_error_pct_of_path"),
        "validity": {
            "timestamps_monotonic": None if backward is None else backward == 0,
            # Conservative clock discontinuity count, not a pose-reset detector.
            "resets": backward,
            "max_gap_s": timestamps.get("max_dt_s"),
            "moving": motion.get("enough_motion"),
        },
        "walls": {
            "p95_point_to_plane_m": p95,
            "doubled_wall_suspected": doubled,
            "gate_m": gate_m,
            "gate_state": gate_state,
        },
        "wall_patches": provenance,
        "wall_warnings": warnings,
        "walkcheck_source": source_path,
        "walkcheck_result": producer_status,
    }
    if physical is False:
        result["accuracy_label"] = REPLAY_LABEL

    if producer_status == "NOT-EVALUABLE":
        for key in ("d3_m", "horizontal_m", "vertical_m"):
            result["return_error"][key] = None
        result["drift_pct_of_path"] = None

    # Keep the series validation contract for non-wall missing evidence too.
    required = (result["return_error"]["d3_m"], result["return_error"]["threshold_m"],
                result["validity"]["timestamps_monotonic"], backward,
                result["validity"]["moving"])
    if any(value is None for value in required) and result["status"] != "NOT_EVALUABLE":
        result["status"] = "NOT_EVALUABLE"
        reasons.append("series: required return or validity evidence is missing")

    walk = parse_walk_result(result, source_path)
    if walk.recorded_at is None:
        reject("metadata recorded_at is required")
    return result
