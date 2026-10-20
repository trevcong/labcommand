"""Shared wall acceptance for walk-day and series adapt.

Evidence comes only from the supplied reports and explicit preselection spec.
The whole-map doubled-wall flag is a warning only: it cannot tell a ghost wall
from two faces of a solid object. Doubled walls are decided on the patches.
"""

from __future__ import annotations

import hashlib
import json
import math
import os

PATCH_ORIGINS = ("preselected", "auto_suggested")
REQUIRED_PATCHES = 2
# Provisional (Trevor, 2026-10-20): guard band under the 5 cm wall gate. On a
# patch that would otherwise pass, p95 and both 80% bands must be at most this.
# Above it and up to the gate the patch is not checkable (never a pass); p95
# or separation over the gate still fails. Revisit after real walks.
PASS_MAX_M = 0.040
PASS_MAX_LABEL = "provisional guard band (Trevor 2026-10-20, revisit after real walks)"


class WallScoringError(Exception):
    pass


def _num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def _cm_to_m(v):
    v = _num(v)
    return None if v is None else round(v / 100.0, 9)


def _patches(wallpatch):
    if wallpatch is None:
        return []
    if not isinstance(wallpatch, dict):
        raise WallScoringError("wallpatch report must be an object")
    patches = wallpatch.get("patches", [])
    if not isinstance(patches, list) or any(not isinstance(p, dict) for p in patches):
        raise WallScoringError("wallpatch patches must be a list of objects")
    return patches


def _tool_gate(wallpatch, key):
    gates = wallpatch.get("gates") if isinstance(wallpatch, dict) else None
    return _num(gates.get(key)) if isinstance(gates, dict) else None


def effective_gate_check(p, wallpatch, gate_m):
    """Re-check one MEASURED patch against the stricter of gate_m and the patch tool's gate.

    The patch tool's pass flags were computed at its own gate, so a stricter
    requested gate must be applied to the measured numbers here. A would-be
    pass also needs p95 and both bands at most the pass limit (PASS_MAX_M or
    a stricter gate); between that and the gate it is not evaluable.
    Returns ([(flag_key, failure)], not_evaluable_reasons).
    """
    name = p.get("name")
    failures, unknown = [], []
    residuals = p.get("residuals") if isinstance(p.get("residuals"), dict) else {}
    p95_gate = min(g for g in (gate_m, _tool_gate(wallpatch, "p95_point_to_plane_max_m")) if g is not None)
    sep_gate = min(g for g in (gate_m, _tool_gate(wallpatch, "revisit_separation_max_m")) if g is not None)
    p95 = _num(residuals.get("p95_abs_m"))
    if p95 is None:
        unknown.append(f"patch {name}: measured p95 missing")
    elif p95 > p95_gate:
        failures.append(("gate_p95_pass",
                         f"patch {name}: p95 {p95 * 100:.2f} cm > effective gate {p95_gate * 100:g} cm"))
    if p.get("doubled") == "true":
        sep = _num(p.get("separation_m"))
        if sep is not None and sep > sep_gate:
            failures.append(("gate_separation_pass",
                             f"patch {name}: separation {sep * 100:.2f} cm > effective gate {sep_gate * 100:g} cm"))
    elif p.get("doubled") == "false" and not failures \
            and p.get("gate_p95_pass") is not False and p.get("gate_separation_pass") is not False:
        # Same fail-safe as the patch tool, on would-be passes only (the tool
        # leaves stepped_band_m null otherwise): a band at least as wide as the
        # gate could hold two surfaces that far apart, so "not doubled" is unproven.
        # The pass limit is the guard band's lower edge: never looser than either gate.
        limit = min(g for g in (PASS_MAX_M, _tool_gate(wallpatch, "pass_max_m"), p95_gate, sep_gate)
                    if g is not None)
        if p95 is not None and p95 > limit:
            unknown.append(f"patch {name}: p95 {p95 * 100:.2f} cm > pass limit {limit * 100:g} cm "
                           f"and within the gate, not checkable")
        for key in ("core_band_m", "stepped_band_m"):
            band = _num(residuals.get(key))
            if band is None:
                unknown.append(f"patch {name}: {key} missing, single surface unproven at the effective gate")
            elif band >= sep_gate:
                unknown.append(f"patch {name}: {key} {band * 100:.2f} cm >= effective gate "
                               f"{sep_gate * 100:g} cm, single surface unproven")
            elif band > limit:
                unknown.append(f"patch {name}: {key} {band * 100:.2f} cm > pass limit {limit * 100:g} cm "
                               f"and under the gate, not checkable")
    return failures, unknown


def wall_evidence(walkcheck: dict, wallpatch, gate_m: float):
    """Measure the whole-map p95 gate and the patch gates without accepting provenance.

    The whole-map doubled-wall flag only adds a warning (Trevor, 2026-10-20).
    Returns (p95_m, doubled_suspected True/False/None, failures, notes, patches, warnings).
    """
    failures, notes, patches, warnings = [], [], [], []
    whole = walkcheck.get("walls") or {}
    if not isinstance(whole, dict):
        raise WallScoringError("whole-map walls must be an object")
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
                "inspect topdown.png. Warning only: doubled walls are decided on the scoring wall patches.")
        elif not isinstance(whole.get("doubled_wall"), bool):
            warnings.append("whole-map doubled-wall flag missing (warning only)")
    else:
        doubled.append(None)
        notes.append(f"whole-map wall check not evaluable: {whole.get('reason')}")
    if wallpatch is None:
        doubled.append(None)
        notes.append("no wall-patch report")
    else:
        for p in _patches(wallpatch):
            item = {k: p.get(k) for k in ("name", "status", "doubled", "separation_m",
                                          "gate_p95_pass", "gate_separation_pass", "reasons")}
            residuals = p.get("residuals") or {}
            if not isinstance(residuals, dict):
                raise WallScoringError("patch residuals must be an object or null")
            item["p95_abs_m"] = residuals.get("p95_abs_m")
            patches.append(item)
            if p.get("status") != "MEASURED":
                doubled.append(None)
                notes.append(f"patch {p.get('name')} not measurable: {p.get('reasons')}")
                continue
            d = True if p.get("doubled") == "true" else False if p.get("doubled") == "false" else None
            doubled.append(d)
            if d is None:
                notes.append(f"patch {p.get('name')}: doubled wall undetermined "
                             f"(split {p.get('separation_m')} m)")
            if d is True:
                failures.append(f"patch {p.get('name')}: doubled wall, separation {p.get('separation_m')} m")
            strict, unknown = effective_gate_check(p, wallpatch, gate_m)
            failures.extend(msg for _, msg in strict)
            if unknown:
                notes.extend(unknown)
                doubled.append(None)
            caught = {key for key, _ in strict}
            if p.get("gate_p95_pass") is False and "gate_p95_pass" not in caught:
                failures.append(f"patch {p.get('name')}: p95 {item['p95_abs_m']} m over gate")
            if p.get("gate_separation_pass") is False and "gate_separation_pass" not in caught:
                failures.append(f"patch {p.get('name')}: separation {p.get('separation_m')} m over gate")
            for key in ("gate_p95_pass", "gate_separation_pass"):
                if not isinstance(p.get(key), bool):
                    notes.append(f"patch {p.get('name')}: {key} missing")
                    doubled.append(None)
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


def patch_provenance(origin, spec_path, raw_reference, wallpatch, patches_file, *, gate_m):
    """Which of the frozen spec's preselected patches this run verified.

    gate_m is the requested wall gate; verification uses the stricter of it and
    the patch tool's own gate."""
    _check_gate(gate_m)
    if origin not in PATCH_ORIGINS:
        raise WallScoringError(f"patch origin must be one of {PATCH_ORIGINS}, got {origin!r}")
    measured = _patches(wallpatch)
    if raw_reference is not None and not isinstance(raw_reference, dict):
        raise WallScoringError("physical_reference must be an object or null")
    out = {"origin": origin, "spec_sha256": None, "required": [], "verified": [], "problems": [],
           "patches_sha256": _sha_file(patches_file) if patches_file and os.path.isfile(patches_file) else None}
    if spec_path is None:
        out["problems"].append("no frozen spec given, so there are no preselected patch names to check")
        return out
    try:
        with open(spec_path, "rb") as f:
            raw = f.read()
        spec = json.loads(raw)
    except (OSError, ValueError) as exc:
        raise WallScoringError(f"cannot read spec {spec_path}: {exc}")
    if not isinstance(spec, dict):
        raise WallScoringError(f"spec {spec_path} is not a JSON object")
    sha = hashlib.sha256(raw).hexdigest()
    if raw_reference is not None and raw_reference.get("spec_sha256") != sha:
        raise WallScoringError(f"spec {spec_path} (sha256 {sha}) is not the spec the physical reference names "
                          f"({raw_reference.get('spec_sha256')})")
    out["spec_sha256"] = sha
    wanted = []
    required = spec.get("wall_patches", [])
    if not isinstance(required, list):
        raise WallScoringError("spec wall_patches must be a list")
    for p in required:
        name = p.get("name") if isinstance(p, dict) else None
        if not isinstance(name, str) or not name.strip():
            out["problems"].append("the spec has a wall patch without a name")
            continue
        wanted.append((name.strip(), p.get("box_map_frame")))
    out["required"] = [n for n, _ in wanted]
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
        if p.get("status") == "MEASURED":
            strict, unknown = effective_gate_check(p, wallpatch, gate_m)
            why.extend(msg.removeprefix(f"patch {p.get('name')}: ") for _, msg in strict)
            why.extend(msg.removeprefix(f"patch {p.get('name')}: ") for msg in unknown)
        if box is not None and not _same_box(box, p.get("box_map_frame")):
            why.append("box differs from the frozen spec's box")
        if why:
            out["problems"].append(f"patch {name}: " + ", ".join(why))
        elif origin == "preselected":
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
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _check_gate(gate_m):
    if _num(gate_m) is None or gate_m <= 0:
        raise WallScoringError("wall gate must be a positive finite number")


def _check_map(wallpatch, map_path):
    """Bind supplied patch measurements to the run's actual map bytes."""
    if wallpatch is None:
        return
    _patches(wallpatch)
    expected = wallpatch.get("map_sha256")
    if not isinstance(expected, str) or len(expected) != 64 or any(
        c not in "0123456789abcdefABCDEF" for c in expected
    ):
        raise WallScoringError("wallpatch map_sha256 must be 64 hex characters")
    try:
        actual = _sha_file(map_path)
    except OSError as exc:
        raise WallScoringError(f"cannot read run map {map_path}: {exc}") from exc
    if expected.lower() != actual:
        raise WallScoringError(
            f"wallpatch map_sha256 {expected} does not match run map {map_path} (sha256 {actual})"
        )


def score_walls(walkcheck, wallpatch, gate_m, provenance, *, map_path):
    """Return the same evidence tuple to both converters.

    verified retains the existing meaning: a named, uniquely measured,
    spec-matching patch with both gates passed and no doubled wall. Without
    any such preselected patch the wall verdict is NOT_EVALUABLE, even if
    diagnostic failures exist. Supplied patch reports must hash-match map_path.
    With verified evidence, known failures remain
    FAIL; otherwise every required patch must be verified before PASS.
    """
    _check_gate(gate_m)
    _check_map(wallpatch, map_path)
    p95, doubled, failures, notes, patches, warnings = wall_evidence(walkcheck, wallpatch, gate_m)
    blockers = acceptance_blockers(provenance)
    verified = provenance.get("verified", []) if isinstance(provenance, dict) else []
    preselected = isinstance(provenance, dict) and provenance.get("origin") == "preselected"
    if not preselected or not verified:
        doubled = None
        notes.extend(blockers)
        notes.append("zero verified preselected wall patches")
    elif doubled is None or notes or (blockers and not failures):
        doubled = None
        notes.extend(blockers)
    return p95, doubled, failures, notes, patches, warnings


def wall_verdict(producer_status, reasons, doubled, failures, notes):
    """Combine trajectory and wall evidence without upgrading a producer failure."""
    reasons = list(reasons)
    if producer_status == "NOT-EVALUABLE":
        return "NOT_EVALUABLE", reasons or ["walkcheck NOT-EVALUABLE"]
    if doubled is None:
        return "NOT_EVALUABLE", reasons + failures + ["wall evidence incomplete: " + "; ".join(notes)]
    if producer_status == "FAIL" or failures or doubled is True:
        return "FAIL", reasons + failures or ["walkcheck FAIL"]
    return "PASS", reasons
