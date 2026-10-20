"""Per-run walk-day result: walk_result.json (input to spatio.series) plus a
plain-English walkday.md / walkday.json for Trevor.

    python3 -m spatio.walkday.result SESSION [--wall-gate-m 0.05]
        [--wall-gate-state approved|proposed] [--patch-source TEXT]
    python3 -m spatio.walkday.result SESSION --stopped-at STAGE --reason TEXT

The first form reads SESSION/walkcheck.json, SESSION/wallpatch/wallpatch.json,
SESSION/manifest.json, SESSION/physical_reference.json (only when the manifest
says physical accuracy was evaluated), SESSION/logs/input_gate.json and, for a
synthetic recording, its SYNTHETIC.json ground truth. The second form records
a run that stopped before scoring (NOT-EVALUABLE; no walk_result.json).

Exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 bad or contradictory input.

Run status (stricter than spatio.series W1, never looser):
  PASS only if the walk checker says PASS (return error < 0.10 m with parked,
  continuous, moving endpoints; no end-onto-start alignment), no doubled wall
  is suspected (whole-map check and every measured patch), the whole-map
  95th-percentile point-to-plane residual is within the wall gate, and every
  measured patch passes its p95 and separation gates.
  NOT_EVALUABLE if the walk checker says NOT-EVALUABLE (its return error is then
  withheld from walk_result.json and kept in walkday.json), or the wall evidence
  is incomplete (a patch undetermined/not measurable, or no patch measured).
  FAIL otherwise.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime, timezone

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
    """Combine the whole-map check with the measured patches.

    Returns (p95_m, doubled_suspected True/False/None, failures, notes, patches).
    """
    failures, notes, patches = [], [], []
    whole = walkcheck.get("walls") or {}
    p95 = None
    doubled = []
    if whole.get("status") == "OK":
        p95 = _cm_to_m(whole.get("p95_point_to_plane_cm"))
        doubled.append(whole.get("doubled_wall") if isinstance(whole.get("doubled_wall"), bool) else None)
        if p95 is None:
            doubled.append(None)
        elif p95 > gate_m:
            failures.append(f"whole-map wall p95 {p95 * 100:.1f} cm > gate {gate_m * 100:g} cm")
        if whole.get("doubled_wall") is True:
            failures.append("whole-map check found a doubled wall "
                            f"(largest revisit separation {whole.get('max_revisit_separation_cm')} cm)")
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
    return p95, suspected, failures, notes, patches


def build(session: str, gate_m: float, gate_state: str, patch_source: str):
    wc = _load(os.path.join(session, "walkcheck.json"))
    manifest = _load(os.path.join(session, "manifest.json"))
    wp = _load(os.path.join(session, "wallpatch", "wallpatch.json"), required=False)
    gate = _load(os.path.join(session, "logs", "input_gate.json"), required=False)
    run_id = os.path.basename(os.path.normpath(session))
    if manifest.get("run_id") != run_id:
        raise ResultError(f"manifest run_id {manifest.get('run_id')!r} != folder {run_id!r}")
    evaluated = manifest.get("physical_accuracy_evaluated") is True
    synthetic = _synthetic_truth(manifest.get("source_recording"))
    if synthetic is not None and evaluated:
        raise ResultError("synthetic recording but manifest claims physical accuracy was evaluated")
    reference = None
    placement = None
    if evaluated:
        ref = _load(os.path.join(session, "physical_reference.json"))
        reference = {k: ref.get(k) for k in ("start_placement_residual_m", "end_placement_residual_m",
                                             "heading_check", "placement_tolerance_m")}
        placement = _placement(reference)
    elif os.path.isfile(os.path.join(session, "logs", "physical_reference.not_applied.json")):
        ref = _load(os.path.join(session, "logs", "physical_reference.not_applied.json"))
        placement = _placement({k: ref.get(k) for k in ("start_placement_residual_m", "end_placement_residual_m",
                                                        "heading_check", "placement_tolerance_m")})
        placement["applied"] = False

    rts = wc.get("return_to_start") or {}
    ts = wc.get("timestamps") or {}
    motion = wc.get("motion") or {}
    wc_result = wc.get("result")
    if wc_result not in ("PASS", "FAIL", "NOT-EVALUABLE"):
        raise ResultError(f"walkcheck result {wc_result!r}")
    p95, doubled, wall_fail, wall_notes, patches = wall_evidence(wc, wp, gate_m)

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
                  "failures": wall_fail, "notes": wall_notes},
        "input_gate": None if gate is None else {k: gate.get(k) for k in ("evaluable", "blocking", "warnings")},
        "manifest": {k: manifest.get(k) for k in ("sensor", "fastlio_revision", "config_file",
                                                  "config_sha256", "image", "source_recording")},
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


def stopped(session: str, stage: str, reason: str):
    gate = _load(os.path.join(session, "logs", "input_gate.json"), required=False)
    return {
        "schema": REPORT_SCHEMA,
        "run_id": os.path.basename(os.path.normpath(session)),
        "status": "NOT_EVALUABLE",
        "reasons": [f"stopped at {stage}: {reason}"],
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "stopped_at": stage,
        "input_gate": None if gate is None else {k: gate.get(k) for k in ("evaluable", "blocking", "warnings")},
    }


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
    for x in w["notes"]:
        L.append(f"- note: {x}")
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
          f"- recording: {m['source_recording']}", f"- sensor: {m['sensor']}",
          f"- FAST-LIO {m['fastlio_revision']}, config {m['config_file']} (sha256 {m['config_sha256']})",
          f"- image: {m['image']}", ""]
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
            walk, report = build(args.session, args.wall_gate_m, args.wall_gate_state, args.patch_source)
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
