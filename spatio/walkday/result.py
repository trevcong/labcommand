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
Either form first moves an earlier walk_result.json aside (walk_result.json.retired;
deleted again when a new one is written). So after exit 3 or a stopped run the
session has no walk_result.json and series.sh refuses it; walkday.md/json then
say why (stopped at result: rejected ...).

Run status (stricter than spatio.series W1, never looser):
  NOT_EVALUABLE first unless logs/input_gate.json is gate.py's report saying
  evaluable with no blocker (input_gate_blockers). A missing, blocked (e.g.
  B2 CLOCK_RESET) or contradictory gate withholds the return error, so the
  run can never count. An unreadable gate file is bad input (exit 3).
  Otherwise status uses spatio.wall_scoring, also used by spatio.series adapt.
  PASS requires a passing trajectory, complete verified preselected patches,
  whole-map and patch gates, and no whole-map or patch doubled-wall suspicion.
  Zero verified patches and auto-suggested patches are NOT_EVALUABLE. The
  whole-map flag can mistake solid-object faces for doubled walls; it remains
  conservatively decisive in both paths, with that limitation shown as a warning.
  A wall-patch report whose map_sha256 is not this run's map.pcd is bad input (exit 3).

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
import json
import math
import os
import sys
from datetime import datetime, timezone

from spatio.wall_scoring import (
    PATCH_ORIGINS, WallScoringError as ResultError, acceptance_blockers,
    patch_provenance, score_walls, wall_verdict,
)
from spatio.replay.recording import RecordingError, recorded_at
from spatio.walkday.gate import SCHEMA as GATE_SCHEMA
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


def input_gate_blockers(gate):
    """Why logs/input_gate.json is not a clean pass. Empty = it is.

    A clean pass is gate.py's own report saying evaluable with no blocker. A
    missing report is not evidence: scoring never assumes stage 2 passed."""
    if gate is None:
        return ["no input gate (logs/input_gate.json missing), so the input was never checked"]
    if gate.get("schema") != GATE_SCHEMA:
        return [f"logs/input_gate.json is not a {GATE_SCHEMA} report"]
    blocking = gate.get("blocking")
    if not isinstance(blocking, list):
        return ["logs/input_gate.json has no blocking list"]
    why = [f"input gate: {b}" for b in blocking]
    if gate.get("evaluable") is not True:
        why = why or ["input gate: evaluable is %s" % gate.get("evaluable")]
    elif why:
        why.insert(0, "input gate says evaluable but lists blockers")
    return why


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
                                    os.path.join(session, "wallpatch", "patches.json"), gate_m=gate_m)
    driver = None if prov is None else prov.get("driver_config")

    rts = wc.get("return_to_start") or {}
    ts = wc.get("timestamps") or {}
    motion = wc.get("motion") or {}
    wc_result = wc.get("result")
    if wc_result not in ("PASS", "FAIL", "NOT-EVALUABLE"):
        raise ResultError(f"walkcheck result {wc_result!r}")
    settings, settings_changed = _settings_used(prov, wc, gate_m, gate_state)
    p95, doubled, wall_fail, wall_notes, patches, wall_warn = score_walls(
        wc, wp, gate_m, wall_patches, map_path=os.path.join(session, "map.pcd"),
    )

    d3 = _num(rts.get("error_3d_m"))
    horiz = _cm_to_m(rts.get("error_horizontal_cm"))
    vert = _cm_to_m(rts.get("error_vertical_cm"))
    raw_return = {"d3_m": d3, "horizontal_m": horiz, "vertical_m": vert,
                  "heading_deg": _num(rts.get("heading_error_deg"))}
    status, reasons = wall_verdict(wc_result, wc.get("reasons") or [], doubled, wall_fail, wall_notes)
    gate_why = input_gate_blockers(gate)
    if gate_why:
        status = "NOT_EVALUABLE"
        d3 = horiz = vert = None   # blocked or unchecked input: the return error is not a measurement
        reasons = gate_why + reasons
    elif wc_result == "NOT-EVALUABLE":
        d3 = horiz = vert = None   # an unparked/unreadable endpoint is not a measurement
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
        "input_gate_blockers": gate_why,
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
    try:
        gate = _load(os.path.join(session, "logs", "input_gate.json"), required=False)
    except ResultError as exc:          # a stop report must always be written
        gate, reason = None, f"{reason}; also {exc}"
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
    if report["walkcheck_result"] == "NOT-EVALUABLE" or report.get("input_gate_blockers"):
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
    L += ["", "## Input check (x1 bag timing)", ""]
    L += [f"- NOT PASSED: {b}" for b in report.get("input_gate_blockers") or []] or ["- no blocking problems"]
    L += [f"- warning: {x}" for x in (gate or {}).get("warnings") or []]
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
RETIRED = "walk_result.json.retired"


def _retire(session):
    """Move an earlier walk_result.json aside before scoring again. Only a
    successful build writes a new one, so a rejected re-score (exit 3) or a
    stopped re-run can never leave an earlier result for series.sh to count.
    A mark inside the file would not do: spatio.series ignores extra keys."""
    path = os.path.join(session, "walk_result.json")
    if os.path.exists(path):
        os.replace(path, os.path.join(session, RETIRED))


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
    _retire(args.session)
    try:
        if not (math.isfinite(args.wall_gate_m) and args.wall_gate_m > 0):
            raise ResultError("--wall-gate-m must be a positive number")
        if args.stopped_at:
            report = stopped(args.session, args.stopped_at, args.reason)
        else:
            walk, report = build(args.session, args.wall_gate_m, args.wall_gate_state, args.patch_source,
                                 args.patch_origin, args.spec)
            _write(args.session, "walk_result.json", walk)
            if os.path.exists(os.path.join(args.session, RETIRED)):
                os.remove(os.path.join(args.session, RETIRED))
    except ResultError as exc:
        print(f"result: {exc}", file=sys.stderr)
        # Replace any earlier walkday.md/json too: it may still say PASS.
        report = stopped(args.session, "result", f"rejected, no walk_result.json: {exc}")
        _write(args.session, "walkday.json", report)
        _write(args.session, "walkday.md", render(report))
        return 3
    _write(args.session, "walkday.json", report)
    _write(args.session, "walkday.md", render(report))
    print(f"{report['run_id']}: {report['status'].replace('_', '-')}"
          + (f" ({'; '.join(report['reasons'])})" if report["reasons"] else ""))
    return EXIT[report["status"]]


if __name__ == "__main__":
    sys.exit(main())
