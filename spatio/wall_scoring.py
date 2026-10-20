"""Shared wall acceptance for walk-day and series adapt.

Evidence comes only from the supplied reports and explicit preselection spec.
Whole-map doubled flags remain decisive, even for possible solid-object faces.
"""

from __future__ import annotations

import hashlib
import json
import math
import os

PATCH_ORIGINS = ("preselected", "auto_suggested")
REQUIRED_PATCHES = 2


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


def wall_evidence(walkcheck: dict, wallpatch, gate_m: float):
    """Measure both whole-map and patch gates without accepting provenance.

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
        d = whole.get("doubled_wall")
        doubled.append(d if isinstance(d, bool) else None)
        if not isinstance(d, bool):
            notes.append("whole-map doubled-wall evidence missing")
        if p95 is None:
            doubled.append(None)
            notes.append("whole-map p95 missing")
        elif p95 > gate_m:
            failures.append(f"whole-map wall p95 {p95 * 100:.1f} cm > gate {gate_m * 100:g} cm")
        if whole.get("doubled_wall") is True:
            failures.append("whole-map check suspects a doubled wall")
            warnings.append(
                "whole-map check suspects a doubled wall (revisit separation "
                f"{whole.get('revisit_separation_cm')} cm, largest {whole.get('max_revisit_separation_cm')} cm). "
                "It cannot tell a ghost wall from two faces of a real object (pillar, crate, thin wall): "
                "inspect topdown.png. Conservatively treated as FAIL by the shared wall rule.")
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
            if p.get("gate_p95_pass") is False:
                failures.append(f"patch {p.get('name')}: p95 {item['p95_abs_m']} m over gate")
            if p.get("gate_separation_pass") is False:
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


def patch_provenance(origin, spec_path, raw_reference, wallpatch, patches_file):
    """Which of the frozen spec's preselected patches this run verified."""
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
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def score_walls(walkcheck, wallpatch, gate_m, provenance):
    """Return the same evidence tuple to both converters.

    verified retains the existing meaning: a named, uniquely measured,
    spec-matching patch with both gates passed and no doubled wall. Without
    any such preselected patch the wall verdict is NOT_EVALUABLE, even if
    diagnostic failures exist. With verified evidence, known failures remain
    FAIL; otherwise every required patch must be verified before PASS.
    """
    if _num(gate_m) is None or gate_m <= 0:
        raise WallScoringError("wall gate must be a positive finite number")
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
