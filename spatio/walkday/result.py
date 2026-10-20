"""Per-run walk-day result: walk_result.json (input to spatio.series) plus a
plain-English walkday.md / walkday.json for Trevor.

    python3 -m spatio.walkday.result SESSION [--wall-gate-m 0.05]
        [--wall-gate-state approved|proposed] [--patch-source TEXT]
    python3 -m spatio.walkday.result SESSION --stopped-at STAGE --reason TEXT

The first form reads SESSION/walkcheck.json, SESSION/wallpatch/wallpatch.json,
SESSION/manifest.json, SESSION/physical_reference.json (only when the manifest
says physical accuracy was evaluated), SESSION/logs/input_gate.json, the
recording's bag/metadata.yaml (its starting_time becomes recorded_at, which
spatio.series uses to order runs; never the processing time) and, for a
synthetic recording, its SYNTHETIC.json ground truth. The second form records
a run that stopped before scoring (NOT-EVALUABLE; no walk_result.json).

Exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 bad or contradictory input.

Run status (stricter than spatio.series W1, never looser):
  PASS only if the walk checker says PASS (return error < 0.10 m with parked,
  continuous, moving endpoints; no end-onto-start alignment), no measured wall
  patch is doubled, the whole-map 95th-percentile point-to-plane residual is
  within the wall gate, and every measured patch passes its p95 and separation
  gates. Doubled walls are decided on the patches, as the workplan defines
  them ("revisit wall separation <= 5 cm on the two preselected patches").
  The walk checker's whole-map doubled-wall flag is shown as a WARNING, not
  used: since fd3cf19 it has no upper separation cap and reports the two faces
  of any solid object (a 0.5 m pillar reads as a 50 cm doubled wall), so it
  cannot tell a ghost wall from furniture. Walkcheck itself never let walls
  change its verdict; requiring that flag was this module's own extra rule.
  NOT_EVALUABLE if the walk checker says NOT-EVALUABLE (its return error is then
  withheld from walk_result.json and kept in walkday.json), or the wall evidence
  is incomplete (a patch undetermined/not measurable, or no patch measured).
  FAIL otherwise.

Acceptance provenance (extra walk_result.json keys; spatio.series ignores them,
spatio.walkday.seriesgate enforces them):
  wall_patches: origin ("preselected" only with process.sh --patches), the
    frozen spec's patch names (required; the spec must be the one the physical
    reference names) and which of them were verified: exactly one patch of that
    name, MEASURED, not doubled, both gates passed, and the spec's box when the
    spec fixes one. A run counts toward a series only with origin preselected
    and every one of at least two required patches verified.
  walkday_settings: the effective settings identity. Stage 1's (logs/provenance.json)
    when the walk check (walkcheck.json parameters) and this scoring
    (--wall-gate-m, --wall-gate-state) used its values; recomputed from the
    values used when one differs, so a re-scored run starts a new series.
    walkday.json/md list each changed setting. A walkcheck.json that does not
    record its parameters cannot be checked (exit 3).
  driver_config_sha256: the sensor driver config (process.sh --driver-config).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from datetime import datetime, timezone

from spatio.replay.recording import RecordingError, recorded_at
from spatio.walkday.provenance import WALKCHECK_PARAMS, check_identity, effective_identity
from spatio.series.validate import (
    PLACEMENT_TOLERANCE_M,
    REPLAY_LABEL,
    RETURN_THRESHOLD_M,
    InputRejected,
    parse_walk_result,
)

WALK_SCHEMA = "spatio.walk_result/1"
REPORT_SCHEMA = "spatio.walkday.report/1"
PROVISIONAL = "provisional (approved for now, revisit after real data)"
PATCH_ORIGINS = ("preselected", "auto_suggested")
# Workplan step 1/6: "two preselected patches". The milestone's rule, not a
# property of any recording (expspec's MIN_PATCHES is the same number).
REQUIRED_PATCHES = 2


class ResultError(Exception):
    pass


def _load(path, required=True):
    if not os.path.isfile(path):
        if required:
            raise ResultError(f"missing {path}")
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError) as exc:
        raise ResultError(f"cannot read {path}: {exc}")
    if not isinstance(data, dict):
        raise ResultError(f"{path} is not a JSON object")
    return data


def _num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def _cm_to_m(v):
    v = _num(v)
    return None if v is None else round(v / 100.0, 9)


def wall_evidence(walkcheck: dict, wallpatch, gate_m: float):
    """Doubled walls from the measured patches; whole-map p95 as a gate; the
    whole-map doubled flag as a warning only (see module docstring).

    Returns (p95_m, doubled_suspected True/False/None, failures, notes, patches, warnings).
    """
    failures, notes, patches, warnings = [], [], [], []
    whole = walkcheck.get("walls") or {}
    p95 = None
    doubled = []
    if whole.get("status") == "OK":
        p95 = _cm_to_m(whole.get("p95_point_to_plane_cm"))
        if p95 is None:
            doubled.append(None)
            notes.append("whole-map p95 missing")
        elif p95 > gate_m:
            failures.append(f"whole-map wall p95 {p95 * 100:.1f} cm > gate {gate_m * 100:g} cm")
        if whole.get("doubled_wall") is True:
            warnings.append(
                "whole-map check suspects a doubled wall (revisit separation "
                f"{whole.get('revisit_separation_cm')} cm, largest {whole.get('max_revisit_separation_cm')} cm). "
                "It cannot tell a ghost wall from two faces of a real object (pillar, crate, thin wall): "
                "inspect topdown.png. Not used for PASS/FAIL; the patches decide.")
    else:
        doubled.append(None)
        notes.append(f"whole-map wall check not evaluable: {whole.get('reason')}")
    if wallpatch is None:
        doubled.append(None)
        notes.append("no wall-patch report")
    else:
        for p in wallpatch.get("patches", []):
            item = {k: p.get(k) for k in ("name", "status", "doubled", "separation_m",
                                          "gate_p95_pass", "gate_separation_pass", "reasons")}
            item["p95_abs_m"] = (p.get("residuals") or {}).get("p95_abs_m")
            patches.append(item)
            if p.get("status") != "MEASURED":
                doubled.append(None)
                notes.append(f"patch {p.get('name')} not measurable: {p.get('reasons')}")
                continue
            d = {"true": True, "false": False}.get(p.get("doubled"))
            doubled.append(d)
            if d is None:
                notes.append(f"patch {p.get('name')}: doubled wall undetermined "
                             f"(split {p.get('separation_m')} m)")
            if d is True:
                failures.append(f"patch {p.get('name')}: doubled wall, separation {p.get('separation_m')} m")
            if p.get("gate_p95_pass") is False:
                failures.append(f"patch {p.get('name')}: p95 {item['p95_abs_m']} m over gate")
            if p.get("gate_separation_pass") is False:
                failures.append(f"patch {p.get('name')}: separation {p.get('separation_m')} m over gate")
        if not patches:
            doubled.append(None)
            notes.append("wall-patch report has no patches")
    if any(d is True for d in doubled):
        suspected = True
    elif any(d is None for d in doubled):
        suspected = None
    else:
        suspected = False
    return p95, suspected, failures, notes, patches, warnings


def patch_provenance(origin, spec_path, raw_reference, wallpatch, patches_file):
    """Which of the frozen spec's preselected patches this run verified."""
    if origin not in PATCH_ORIGINS:
        raise ResultError(f"patch origin must be one of {PATCH_ORIGINS}, got {origin!r}")
    out = {"origin": origin, "spec_sha256": None, "required": [], "verified": [], "problems": [],
           "patches_sha256": _sha_file(patches_file) if os.path.isfile(patches_file) else None}
    if spec_path is None:
        out["problems"].append("no frozen spec given, so there are no preselected patch names to check")
        return out
    try:
        with open(spec_path, "rb") as f:
            raw = f.read()
        spec = json.loads(raw)
    except (OSError, ValueError) as exc:
        raise ResultError(f"cannot read spec {spec_path}: {exc}")
    if not isinstance(spec, dict):
        raise ResultError(f"spec {spec_path} is not a JSON object")
    sha = hashlib.sha256(raw).hexdigest()
    if raw_reference is not None and raw_reference.get("spec_sha256") != sha:
        raise ResultError(f"spec {spec_path} (sha256 {sha}) is not the spec the physical reference names "
                          f"({raw_reference.get('spec_sha256')})")
    out["spec_sha256"] = sha
    wanted = []
    for p in spec.get("wall_patches") or []:
        name = p.get("name") if isinstance(p, dict) else None
        if not isinstance(name, str) or not name.strip():
            out["problems"].append("the spec has a wall patch without a name")
            continue
        wanted.append((name.strip(), p.get("box_map_frame")))
    out["required"] = [n for n, _ in wanted]
    measured = (wallpatch or {}).get("patches") or []
    for name, box in wanted:
        hits = [p for p in measured if isinstance(p.get("name"), str) and p["name"].strip() == name]
        if len(hits) != 1:
            out["problems"].append(f"patch {name}: {len(hits)} measured patches have this name, need exactly 1")
            continue
        p, why = hits[0], []
        if p.get("status") != "MEASURED":
            why.append(f"status {p.get('status')}")
        if p.get("doubled") != "false":
            why.append(f"doubled={p.get('doubled')}")
        if p.get("gate_p95_pass") is not True:
            why.append(f"p95 gate {p.get('gate_p95_pass')}")
        if p.get("gate_separation_pass") is not True:
            why.append(f"separation gate {p.get('gate_separation_pass')}")
        if box is not None and not _same_box(box, p.get("box_map_frame")):
            why.append("box differs from the frozen spec's box")
        if why:
            out["problems"].append(f"patch {name}: " + ", ".join(why))
        else:
            out["verified"].append(name)
    return out


def acceptance_blockers(wall_patches):
    """Why a run's wall patches cannot count toward acceptance. Empty = they can.

    Re-derived from the lists, so a hand-edited summary cannot make a run count."""
    wp = wall_patches
    if not isinstance(wp, dict) or not isinstance(wp.get("required"), list) \
            or not isinstance(wp.get("verified"), list):
        return ["no wall-patch provenance in walk_result.json"]
    why = []
    if wp.get("origin") != "preselected":
        why.append(f"wall patches were {wp.get('origin')}, not the spec's preselected patches")
    if wp.get("spec_sha256") is None:
        why.append("no frozen spec named the preselected patches")
    required = set(wp["required"])
    if len(required) < REQUIRED_PATCHES:
        why.append(f"{len(required)} preselected patches named; {REQUIRED_PATCHES} are required")
    missing = [n for n in wp["required"] if n not in wp["verified"]]
    if missing:
        why.append("preselected patches not verified: " + ", ".join(missing))
    return why


def _same_box(want, got, tol=1e-9):
    def unit(v):
        n = math.sqrt(sum(float(x) ** 2 for x in v))
        return [float(x) / n for x in v]
    try:
        pairs = [(want["center_m"], got["center_m"]), (want["half_extent_m"], got["half_extent_m"]),
                 (unit(want["normal_hint"]), unit(got["normal_hint"]))]
        return all(len(a) == len(b) == 3 and all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))
                   for a, b in pairs)
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return False


def _sha_file(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _provenance(session, manifest=None):
    """logs/provenance.json (process.sh stage 1), checked; None when absent."""
    prov = _load(os.path.join(session, "logs", "provenance.json"), required=False)
    if prov is None:
        return None
    if not check_identity(prov.get("settings")):
        raise ResultError("logs/provenance.json: settings sha256 does not match its values")
    want = (prov.get("fastlio_config") or {}).get("sha256")
    if manifest is not None and want != manifest.get("config_sha256"):
        raise ResultError(f"logs/provenance.json FAST-LIO config sha256 {want} != manifest "
                          f"config_sha256 {manifest.get('config_sha256')}")
    return prov


def _settings_used(prov, walkcheck, gate_m, gate_state):
    """Stage 1's settings identity checked against what this run used: the walk
    checker's recorded parameters and this scoring's arguments.

    Returns (identity, changes); (None, []) without logs/provenance.json."""
    if prov is None:
        return None, []
    params = walkcheck.get("parameters")
    used = {}
    for key, name in WALKCHECK_PARAMS.items():
        v = _num(params.get(name)) if isinstance(params, dict) else None
        if v is None:
            raise ResultError(f"walkcheck.json does not record parameters.{name}, so the walk-day "
                              f"settings ({key}) cannot be checked against the walk check that ran")
        used[key] = (v, f"walkcheck.json parameters.{name}")
    used["WALL_GATE_M"] = (gate_m, "--wall-gate-m")
    used["WALL_GATE_STATE"] = (gate_state, "--wall-gate-state")
    return effective_identity(prov["settings"], used)


def build(session: str, gate_m: float, gate_state: str, patch_source: str,
          patch_origin: str = "auto_suggested", spec_path=None):
    wc = _load(os.path.join(session, "walkcheck.json"))
    manifest = _load(os.path.join(session, "manifest.json"))
    wp = _load(os.path.join(session, "wallpatch", "wallpatch.json"), required=False)
    gate = _load(os.path.join(session, "logs", "input_gate.json"), required=False)
    prov = _provenance(session, manifest)
    run_id = os.path.basename(os.path.normpath(session))
    if manifest.get("run_id") != run_id:
        raise ResultError(f"manifest run_id {manifest.get('run_id')!r} != folder {run_id!r}")
    evaluated = manifest.get("physical_accuracy_evaluated") is True
    source = manifest.get("source_recording")
    if not isinstance(source, str) or not source:
        raise ResultError("manifest has no source_recording, so the recording start time is unknown")
    try:
        started = recorded_at(source)
    except RecordingError as exc:
        raise ResultError(f"recording start time: {exc}")
    synthetic = _synthetic_truth(source)
    if synthetic is not None and evaluated:
        raise ResultError("synthetic recording but manifest claims physical accuracy was evaluated")
    reference = None
    placement = None
    ref = None
    if evaluated:
        ref = _load(os.path.join(session, "physical_reference.json"))
        reference = {k: ref.get(k) for k in ("start_placement_residual_m", "end_placement_residual_m",
                                             "heading_check", "placement_tolerance_m")}
        placement = _placement(reference)
    elif os.path.isfile(os.path.join(session, "logs", "physical_reference.not_applied.json")):
        na = _load(os.path.join(session, "logs", "physical_reference.not_applied.json"))
        placement = _placement({k: na.get(k) for k in ("start_placement_residual_m", "end_placement_residual_m",
                                                       "heading_check", "placement_tolerance_m")})
        placement["applied"] = False
    wall_patches = patch_provenance(patch_origin, spec_path, ref, wp,
                                    os.path.join(session, "wallpatch", "patches.json"))
    driver = None if prov is None else prov.get("driver_config")

    rts = wc.get("return_to_start") or {}
    ts = wc.get("timestamps") or {}
    motion = wc.get("motion") or {}
    wc_result = wc.get("result")
    if wc_result not in ("PASS", "FAIL", "NOT-EVALUABLE"):
        raise ResultError(f"walkcheck result {wc_result!r}")
    settings, settings_changed = _settings_used(prov, wc, gate_m, gate_state)
    p95, doubled, wall_fail, wall_notes, patches, wall_warn = wall_evidence(wc, wp, gate_m)

    d3 = _num(rts.get("error_3d_m"))
    horiz = _cm_to_m(rts.get("error_horizontal_cm"))
    vert = _cm_to_m(rts.get("error_vertical_cm"))
    raw_return = {"d3_m": d3, "horizontal_m": horiz, "vertical_m": vert,
                  "heading_deg": _num(rts.get("heading_error_deg"))}
    reasons = list(wc.get("reasons") or [])
    if wc_result == "NOT-EVALUABLE":
        status = "NOT_EVALUABLE"
        d3 = horiz = vert = None   # an unparked/unreadable endpoint is not a measurement
        reasons = reasons or ["walkcheck NOT-EVALUABLE"]
    elif doubled is None:
        status = "NOT_EVALUABLE"
        reasons = reasons + ["wall evidence incomplete: " + "; ".join(wall_notes)]
    else:
        reasons = reasons + wall_fail
        status = "PASS" if (wc_result == "PASS" and not wall_fail and doubled is False) else "FAIL"
        if status == "FAIL" and not reasons:
            reasons = ["walkcheck FAIL"]
    backward = ts.get("backward_steps")
    walk = {
        "schema": WALK_SCHEMA,
        "run_id": run_id,
        "recorded_at": started,
        "status": status,
        "reasons": [str(r) for r in reasons],
        "physical_accuracy_evaluated": evaluated,
        "accuracy_label": None if evaluated else REPLAY_LABEL,
        "fastlio_revision": manifest.get("fastlio_revision"),
        "config_sha256": manifest.get("config_sha256"),
        "duration_s": _num(ts.get("duration_s")),
        "path_length_m": _num(motion.get("path_length_m")),
        "return_error": {
            "d3_m": d3, "horizontal_m": horiz, "vertical_m": vert,
            "heading_deg": raw_return["heading_deg"],
            "threshold_m": RETURN_THRESHOLD_M,
            "start_window_s": rts.get("start_window_s"),
            "end_window_s": rts.get("end_window_s"),
        },
        "drift_pct_of_path": None if d3 is None else _num((wc.get("drift") or {}).get("return_error_pct_of_path")),
        "validity": {
            "timestamps_monotonic": None if backward is None else backward == 0,
            "resets": backward,
            "max_gap_s": _num(ts.get("max_dt_s")),
            "moving": motion.get("enough_motion") if isinstance(motion.get("enough_motion"), bool) else None,
        },
        "walls": {"p95_point_to_plane_m": p95, "doubled_wall_suspected": doubled,
                  "gate_m": gate_m, "gate_state": gate_state},
        "physical_reference": reference,
        "wall_patches": wall_patches,
        "walkday_settings": settings,
        "driver_config_sha256": None if driver is None else driver.get("sha256"),
    }
    try:
        parse_walk_result(walk, os.path.join(session, "walk_result.json"))
    except InputRejected as exc:
        raise ResultError(f"walk_result would be rejected by spatio.series: {exc.message}")

    report = {
        "schema": REPORT_SCHEMA,
        "run_id": run_id,
        "status": status,
        "reasons": walk["reasons"],
        "recorded_at": started,
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "synthetic": synthetic is not None,
        "accuracy_label": walk["accuracy_label"],
        "physical_accuracy_evaluated": evaluated,
        "placement": placement,
        "walkcheck_result": wc_result,
        "return_error_measured": raw_return,
        "return_threshold_m": RETURN_THRESHOLD_M,
        "path_length_m": walk["path_length_m"],
        "duration_s": walk["duration_s"],
        "drift_pct_of_path": _num((wc.get("drift") or {}).get("return_error_pct_of_path")),
        "endpoints": wc.get("endpoints"),
        "timestamps": {k: ts.get(k) for k in ("max_dt_s", "backward_steps", "gap_steps", "fault")},
        "walls": {"whole_map_p95_m": p95, "doubled_wall_suspected": doubled, "gate_m": gate_m,
                  "gate_label": PROVISIONAL, "patch_source": patch_source, "patches": patches,
                  "failures": wall_fail, "notes": wall_notes, "warnings": wall_warn,
                  "whole_map_doubled_flag": (wc.get("walls") or {}).get("doubled_wall")},
        "input_gate": None if gate is None else {k: gate.get(k) for k in ("evaluable", "blocking", "warnings")},
        "manifest": {k: manifest.get(k) for k in ("sensor", "fastlio_revision", "config_file",
                                                  "config_sha256", "image", "source_recording")},
        "wall_patches": wall_patches,
        "acceptance_blockers": acceptance_blockers(wall_patches),
        "provenance": _prov_summary(prov, settings, settings_changed),
        "synthetic_truth": synthetic,
        "raw_start_end": _startend(session),
    }
    if synthetic is not None and raw_return["d3_m"] is not None:
        report["synthetic_truth"]["estimated_minus_true_return_m"] = round(
            raw_return["d3_m"] - synthetic["ground_truth"]["true_return_error_3d_m"], 6)
    return walk, report


def _startend(session):
    """Raw-scan start->end registration (startend.py). Information only."""
    data = _load(os.path.join(session, "startend", "startend.json"), required=False)
    if data is None:
        return None
    keep = {k: data.get(k) for k in ("computed", "reason", "note", "registration", "fastlio_relative",
                                     "fastlio_minus_registration", "winner_seed")}
    return keep


def _placement(ref):
    tol = ref.get("placement_tolerance_m")
    lim = min(tol, PLACEMENT_TOLERANCE_M) if _num(tol) is not None else PLACEMENT_TOLERANCE_M
    why = []
    for k in ("start_placement_residual_m", "end_placement_residual_m"):
        v = _num(ref.get(k))
        if v is None:
            why.append(f"{k} not measured")
        elif v > lim:
            why.append(f"{k} {v * 100:.2f} cm > {lim * 100:g} cm")
    if ref.get("heading_check") != "ok":
        why.append(f"heading check is {ref.get('heading_check')}")
    return dict(ref, within_tolerance=not why, problems=why, applied=True)


def _synthetic_truth(source):
    if not isinstance(source, str):
        return None
    path = os.path.join(source, "SYNTHETIC.json")
    if not os.path.isfile(path):
        return None
    data = _load(path)
    return {"marker": path, "statement": data.get("statement"), "ground_truth": data.get("ground_truth")}


def _prov_summary(prov, settings=None, changed=()):
    if prov is None:
        return None
    d = prov.get("driver_config")
    return {"settings_sha256": (settings or prov["settings"])["sha256"],
            "stage1_settings_sha256": prov["settings"]["sha256"],
            "settings_changed": list(changed),
            "fastlio_config": prov.get("fastlio_config"),
            "driver_config": None if d is None else {k: d.get(k) for k in ("file", "sha256", "topics_in_bag", "note")}}


def stopped(session: str, stage: str, reason: str):
    gate = _load(os.path.join(session, "logs", "input_gate.json"), required=False)
    try:
        prov = _prov_summary(_provenance(session))
    except ResultError as exc:
        prov, reason = None, f"{reason}; also {exc}"
    return {
        "schema": REPORT_SCHEMA,
        "run_id": os.path.basename(os.path.normpath(session)),
        "status": "NOT_EVALUABLE",
        "reasons": [f"stopped at {stage}: {reason}"],
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "stopped_at": stage,
        "input_gate": None if gate is None else {k: gate.get(k) for k in ("evaluable", "blocking", "warnings")},
        "provenance": prov,
    }


def _prov_lines(prov):
    if not prov:
        return ["- walk-day settings / driver config: not recorded (no logs/provenance.json)"]
    changed = prov.get("settings_changed") or []
    if not changed:
        L = [f"- walk-day settings sha256 {prov['settings_sha256']} (logs/provenance.json; a change starts a new series)"]
    else:
        L = [f"- walk-day settings sha256 {prov['settings_sha256']}: NOT stage 1's {prov['stage1_settings_sha256']} "
             "(logs/provenance.json, manifest notes). This run was checked or scored with other settings, "
             "so it starts a new series:"]
        L += [f"  - {c['setting']}: stage 1 {c['stage1']}, used {c['used']} ({c['source']})" for c in changed]
    fc = prov.get("fastlio_config") or {}
    L.append(f"- FAST-LIO config {fc.get('file')} sha256 {fc.get('sha256')}")
    d = prov.get("driver_config")
    L.append("- driver config: none given" if d is None else
             f"- driver config {d['file']} sha256 {d['sha256']} (topics in the bag: "
             f"{', '.join(d['topics_in_bag'])}; {d['note']})")
    return L


def _f(v, scale=1.0, unit="", nd=1):
    return "n/a" if v is None else f"{v * scale:.{nd}f}{unit}"


def render(report: dict) -> str:
    st = report["status"].replace("_", "-")
    L = [f"# Walk-day result: {report['run_id']}", "", f"**{st}**", ""]
    if report.get("synthetic"):
        L += ["> SYNTHETIC recording (generated, not a physical walk). It can never count as "
              "physical evidence.", ""]
    if report.get("stopped_at"):
        L += [f"The run stopped at **{report['stopped_at']}** and was not scored.", ""]
        for r in report["reasons"]:
            L.append(f"- {r}")
        gate = report.get("input_gate")
        if gate:
            L += ["", "Input check:"] + [f"- BLOCKED: {b}" for b in gate.get("blocking") or []]
            L += [f"- warning: {w}" for w in gate.get("warnings") or []]
        if report.get("provenance"):
            L += ["", "Provenance:"] + _prov_lines(report["provenance"])
        return "\n".join(L) + "\n"
    if report["reasons"]:
        L.append("Why: " + "; ".join(report["reasons"]))
        L.append("")
    r = report["return_error_measured"]
    L += ["## Return to start (estimator, no alignment)", "",
          f"- 3D: **{_f(r['d3_m'], 100, ' cm')}** (must be under {report['return_threshold_m'] * 100:g} cm)",
          f"- horizontal {_f(r['horizontal_m'], 100, ' cm')}, vertical {_f(r['vertical_m'], 100, ' cm')}, "
          f"heading {_f(r['heading_deg'], 1, ' deg')}",
          f"- path {_f(report['path_length_m'], 1, ' m')} in {_f(report['duration_s'], 1, ' s')}; "
          f"drift {_f(report['drift_pct_of_path'], 1, ' % of path', 2)}",
          f"- walk checker: {report['walkcheck_result']} (details: walkcheck.md, topdown.png)"]
    if report["walkcheck_result"] == "NOT-EVALUABLE":
        L.append("- The number above is shown for diagnosis only; it is not a valid measurement.")
    truth = report.get("synthetic_truth")
    if truth:
        gt = truth["ground_truth"]
        L.append(f"- synthetic ground truth: true return {gt['true_return_error_3d_m'] * 100:.1f} cm; "
                 f"estimate minus truth {_f(truth.get('estimated_minus_true_return_m'), 100, ' cm')}")
    se = report.get("raw_start_end")
    L += ["", "## Raw-scan start to end (information only, not scored)", ""]
    if not se:
        L.append("- not run")
    elif not se.get("computed"):
        L.append(f"- not computed: {se.get('reason')}")
    else:
        r, fl = se["registration"], se["fastlio_relative"]
        L.append(f"- raw scans say the sensor ended {r['translation_m'] * 100:.1f} cm "
                 f"(horizontal {r['xy_m'] * 100:.1f} cm) and {r['yaw_deg']:+.2f} deg from its start pose; "
                 f"fit {r['inlier_frac_5cm'] * 100:.0f}% of points within 5 cm")
        L.append(f"- FAST-LIO's own start-to-end pose: {fl['translation_m'] * 100:.1f} cm, {fl['yaw_deg']:+.2f} deg "
                 f"(difference {se['fastlio_minus_registration']['translation_m'] * 100:.1f} cm)")
        L.append("- read the fit first: it is the share of end-hold points lying within 5 cm of a start-hold "
                 "surface after registration. Near 100% means both holds see the same scene; a few percent "
                 "means they did not match and the distance above means nothing. No cutoff is applied.")
    w = report["walls"]
    L += ["", "## Walls", "",
          f"- whole map p95 point-to-plane: {_f(w['whole_map_p95_m'], 100, ' cm')} "
          f"(gate {w['gate_m'] * 100:g} cm, {w['gate_label']})",
          f"- doubled wall suspected: {w['doubled_wall_suspected']}",
          f"- patches: {w['patch_source']}"]
    for p in w["patches"]:
        L.append(f"  - {p['name']}: {p['status']}, doubled={p['doubled']}, "
                 f"p95 {_f(p['p95_abs_m'], 100, ' cm')}, separation {_f(p['separation_m'], 100, ' cm')}")
    for x in w["failures"]:
        L.append(f"- FAIL: {x}")
    for x in w.get("warnings") or []:
        L.append(f"- WARNING: {x}")
        L.append("  (The Trail tab shows the walk checker's own wall gate, so it can say FAIL here "
                 "while this report decides on the patches.)")
    for x in w["notes"]:
        L.append(f"- note: {x}")
    blockers = report["acceptance_blockers"]
    L.append("- preselected patches verified: "
             + (", ".join(report["wall_patches"]["verified"]) or "none")
             + f" of {', '.join(report['wall_patches']['required']) or 'none named'}")
    L.append("- these patches can count toward the series: "
             + ("yes" if not blockers else "NO: " + "; ".join(blockers)))
    for x in report["wall_patches"]["problems"]:
        L.append(f"  - {x}")
    L += ["", "## Physical reference", ""]
    pl = report.get("placement")
    if pl is None:
        L.append(f"- none recorded. {report['accuracy_label']}.")
    else:
        L.append(f"- start residual {_f(pl.get('start_placement_residual_m'), 100, ' cm', 2)}, "
                 f"end residual {_f(pl.get('end_placement_residual_m'), 100, ' cm', 2)}, "
                 f"heading {pl.get('heading_check')}, tolerance {_f(pl.get('placement_tolerance_m'), 100, ' cm', 2)}")
        L.append("- within tolerance: " + ("yes" if pl["within_tolerance"] else "NO: " + "; ".join(pl["problems"])))
        if not pl.get("applied"):
            L.append("- recorded for the dry run only; NOT applied to the manifest (synthetic recording)")
    if not report["physical_accuracy_evaluated"]:
        L.append(f"- label: {report['accuracy_label']}")
    gate = report.get("input_gate")
    if gate:
        L += ["", "## Input check (x1 bag timing)", ""]
        L += [f"- BLOCKED: {b}" for b in gate.get("blocking") or []] or ["- no blocking problems"]
        L += [f"- warning: {x}" for x in gate.get("warnings") or []]
    m = report["manifest"]
    L += ["", "## Provenance", "",
          f"- recording: {m['source_recording']}",
          f"- recorded at {report['recorded_at']} (bag starting_time: recorder clock, first message received)",
          f"- sensor: {m['sensor']}",
          f"- FAST-LIO {m['fastlio_revision']}, config {m['config_file']} (sha256 {m['config_sha256']})",
          f"- image: {m['image']}"]
    L += _prov_lines(report.get("provenance")) + [""]
    return "\n".join(L) + "\n"


def _write(session, name, data):
    with open(os.path.join(session, name), "w", encoding="utf-8") as f:
        if isinstance(data, str):
            f.write(data)
        else:
            json.dump(data, f, indent=2)
            f.write("\n")


EXIT = {"PASS": 0, "FAIL": 1, "NOT_EVALUABLE": 2}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="spatio.walkday.result", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--wall-gate-m", type=float, default=0.05)
    ap.add_argument("--wall-gate-state", choices=("approved", "proposed"), default="approved")
    ap.add_argument("--patch-source", default="auto-suggested from this map, NOT Trevor's preselected patches")
    ap.add_argument("--patch-origin", choices=PATCH_ORIGINS, default="auto_suggested",
                    help="preselected only when the patches file holds the spec's preselected patches")
    ap.add_argument("--spec", help="the frozen spec (its wall_patches are the preselected patches)")
    ap.add_argument("--stopped-at")
    ap.add_argument("--reason", default="")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.session):
        print(f"result: no session folder {args.session}", file=sys.stderr)
        return 3
    if not (math.isfinite(args.wall_gate_m) and args.wall_gate_m > 0):
        print("result: --wall-gate-m must be a positive number", file=sys.stderr)
        return 3
    try:
        if args.stopped_at:
            report = stopped(args.session, args.stopped_at, args.reason)
        else:
            walk, report = build(args.session, args.wall_gate_m, args.wall_gate_state, args.patch_source,
                                 args.patch_origin, args.spec)
            _write(args.session, "walk_result.json", walk)
    except ResultError as exc:
        print(f"result: {exc}", file=sys.stderr)
        return 3
    _write(args.session, "walkday.json", report)
    _write(args.session, "walkday.md", render(report))
    print(f"{report['run_id']}: {report['status'].replace('_', '-')}"
          + (f" ({'; '.join(report['reasons'])})" if report["reasons"] else ""))
    return EXIT[report["status"]]


if __name__ == "__main__":
    sys.exit(main())
