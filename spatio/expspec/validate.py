"""Refuse a vague or incomplete experiment spec.

Errors block freeze. Warnings do not. A warning is only raised when one of
the four workplan thresholds differs from the proposed value.
"""

from __future__ import annotations

import math
import re

from spatio.expspec.constants import (
    ACCEPTANCE_KEYS,
    ACCEPTANCE_STATES,
    BOX_KEYS,
    FRAME_KEYS,
    MIN_PATCHES,
    MIN_RUNS,
    MIN_STATIONARY_S,
    MIN_STOPS,
    OPTIONAL_TOP_LEVEL_KEYS,
    PATCH_KEYS,
    PROPOSED_THRESHOLDS,
    REQUIRED_FORBIDDEN,
    ROUTE_KEYS,
    SPEC_SCHEMA,
    STATIONARY_KEYS,
    TOP_LEVEL_KEYS,
)
from spatio.expspec.issues import Issue

_SPEC_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_THRESHOLD_KEYS = tuple(PROPOSED_THRESHOLDS.keys())


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _finite(value):
    return _is_number(value) and math.isfinite(float(value))


def _near(left, right):
    return abs(float(left) - float(right)) <= 1e-9


def _reject_todos(value, path, issues):
    if isinstance(value, str):
        if "TODO(" in value:
            issues.append(Issue("error", path, "TODO placeholder is not allowed"))
        return
    if isinstance(value, dict):
        for key in value:
            child = "{0}.{1}".format(path, key) if path else str(key)
            _reject_todos(value[key], child, issues)
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _reject_todos(item, "{0}[{1}]".format(path, index), issues)


def _missing(path, issues):
    issues.append(Issue("error", path, "missing key"))


def _expect_object(parent, key, path, issues):
    full = "{0}.{1}".format(path, key) if path else key
    if key not in parent:
        _missing(full, issues)
        return None
    value = parent[key]
    if not isinstance(value, dict):
        issues.append(Issue("error", full, "must be an object"))
        return None
    return value


def _require_text(parent, key, path, issues):
    full = "{0}.{1}".format(path, key) if path else key
    if key not in parent:
        _missing(full, issues)
        return
    value = parent[key]
    if not isinstance(value, str) or value.strip() == "":
        issues.append(Issue("error", full, "must be a non-empty string"))


def _unknown_keys(parent, known, path, issues):
    for key in parent:
        if key not in known:
            full = "{0}.{1}".format(path, key) if path else str(key)
            issues.append(Issue("warning", full, "unknown key"))


def _check_number_list(value, path, issues, count, label):
    if not isinstance(value, list) or len(value) != count:
        issues.append(Issue("error", path, "{0} must have {1} numbers".format(label, count)))
        return None
    numbers = []
    for index, item in enumerate(value):
        if not _finite(item):
            issues.append(Issue("error", "{0}[{1}]".format(path, index), "must be a finite number"))
            return None
        numbers.append(float(item))
    return numbers


def _check_box(box, path, issues):
    if box is None:
        return
    if not isinstance(box, dict):
        issues.append(Issue("error", path, "must be an object or null"))
        return
    _unknown_keys(box, BOX_KEYS, path, issues)
    for key in BOX_KEYS:
        if key not in box:
            _missing("{0}.{1}".format(path, key), issues)
    if "center_m" in box:
        # Any finite position is allowed, including zero and negative.
        _check_number_list(
            box.get("center_m"), "{0}.center_m".format(path), issues, 3, "center_m"
        )
    hint = _check_number_list(
        box.get("normal_hint"), "{0}.normal_hint".format(path), issues, 3, "normal_hint"
    ) if "normal_hint" in box else None
    half = _check_number_list(
        box.get("half_extent_m"), "{0}.half_extent_m".format(path), issues, 3, "half_extent_m"
    ) if "half_extent_m" in box else None
    if hint is not None:
        magnitude_sq = hint[0] * hint[0] + hint[1] * hint[1] + hint[2] * hint[2]
        if magnitude_sq <= 1e-24:
            issues.append(Issue("error", "{0}.normal_hint".format(path), "zero normal_hint"))
    if half is not None:
        for index, component in enumerate(half):
            if component <= 0:
                issues.append(Issue(
                    "error",
                    "{0}.half_extent_m[{1}]".format(path, index),
                    "half extent must be positive",
                ))


def validate(spec):
    """Return a list of Issue. Empty means the spec can be frozen."""
    issues = []
    if not isinstance(spec, dict):
        issues.append(Issue("error", "", "spec must be a JSON object"))
        return issues

    _reject_todos(spec, "", issues)
    _unknown_keys(spec, set(TOP_LEVEL_KEYS) | set(OPTIONAL_TOP_LEVEL_KEYS), "", issues)

    for key in TOP_LEVEL_KEYS:
        if key not in spec:
            _missing(key, issues)

    if "notes" in spec and not isinstance(spec["notes"], str):
        issues.append(Issue("error", "notes", "must be a string"))

    if "schema" in spec and spec["schema"] != SPEC_SCHEMA:
        issues.append(Issue("error", "schema", "must be {0}".format(SPEC_SCHEMA)))

    if "spec_id" in spec:
        spec_id = spec["spec_id"]
        if not isinstance(spec_id, str) or _SPEC_ID.match(spec_id) is None:
            issues.append(Issue(
                "error",
                "spec_id",
                "must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}",
            ))

    for key in ("sensor", "fastlio_revision", "config_file", "reference_point", "start_marks"):
        if key in spec:
            _require_text(spec, key, "", issues)

    if "config_sha256" in spec:
        digest = spec["config_sha256"]
        if not isinstance(digest, str) or _SHA256.match(digest) is None:
            issues.append(Issue("error", "config_sha256", "must be 64 lowercase hex characters"))

    route = _expect_object(spec, "route", "", issues) if "route" in spec else None
    if route is not None:
        _unknown_keys(route, ROUTE_KEYS, "route", issues)
        for key in ROUTE_KEYS:
            if key not in route:
                _missing("route.{0}".format(key), issues)
        if "description" in route:
            _require_text(route, "description", "route", issues)
        if "duration_target_s" in route:
            pair = _check_number_list(
                route["duration_target_s"],
                "route.duration_target_s",
                issues,
                2,
                "duration_target_s",
            )
            if pair is not None and (pair[0] <= 0 or pair[1] <= 0 or pair[0] > pair[1]):
                issues.append(Issue(
                    "error",
                    "route.duration_target_s",
                    "must be two positive numbers, low then high",
                ))
        if "intermediate_stops" in route:
            stops = route["intermediate_stops"]
            if not isinstance(stops, list):
                issues.append(Issue("error", "route.intermediate_stops", "must be a list"))
            else:
                if len(stops) < MIN_STOPS:
                    issues.append(Issue(
                        "error",
                        "route.intermediate_stops",
                        "need at least {0} intermediate stops".format(MIN_STOPS),
                    ))
                for index, stop in enumerate(stops):
                    if not isinstance(stop, str) or stop.strip() == "":
                        issues.append(Issue(
                            "error",
                            "route.intermediate_stops[{0}]".format(index),
                            "must be a non-empty string",
                        ))

    frames = _expect_object(spec, "frames", "", issues) if "frames" in spec else None
    if frames is not None:
        _unknown_keys(frames, FRAME_KEYS, "frames", issues)
        for key in FRAME_KEYS:
            if key not in frames:
                _missing("frames.{0}".format(key), issues)
            else:
                _require_text(frames, key, "frames", issues)

    if "wall_patches" in spec:
        patches = spec["wall_patches"]
        if not isinstance(patches, list):
            issues.append(Issue("error", "wall_patches", "must be a list"))
        else:
            if len(patches) < MIN_PATCHES:
                issues.append(Issue(
                    "error",
                    "wall_patches",
                    "need at least {0} wall patches".format(MIN_PATCHES),
                ))
            seen = set()
            for index, patch in enumerate(patches):
                path = "wall_patches[{0}]".format(index)
                if not isinstance(patch, dict):
                    issues.append(Issue("error", path, "must be an object"))
                    continue
                _unknown_keys(patch, PATCH_KEYS, path, issues)
                for key in PATCH_KEYS:
                    if key not in patch:
                        _missing("{0}.{1}".format(path, key), issues)
                if "name" in patch:
                    _require_text(patch, "name", path, issues)
                    if isinstance(patch["name"], str):
                        label = patch["name"].strip()
                        if label in seen:
                            issues.append(Issue("error", "{0}.name".format(path), "duplicate patch name"))
                        seen.add(label)
                if "description" in patch:
                    _require_text(patch, "description", path, issues)
                if "box_map_frame" in patch:
                    _check_box(patch["box_map_frame"], "{0}.box_map_frame".format(path), issues)

    windows = _expect_object(spec, "stationary_windows_s", "", issues) if "stationary_windows_s" in spec else None
    if windows is not None:
        _unknown_keys(windows, STATIONARY_KEYS, "stationary_windows_s", issues)
        for key in STATIONARY_KEYS:
            full = "stationary_windows_s.{0}".format(key)
            if key not in windows:
                _missing(full, issues)
                continue
            value = windows[key]
            if not _finite(value) or float(value) < MIN_STATIONARY_S:
                issues.append(Issue(
                    "error",
                    full,
                    "stationary window must be at least {0:g} s".format(MIN_STATIONARY_S),
                ))

    acceptance = _expect_object(spec, "acceptance", "", issues) if "acceptance" in spec else None
    if acceptance is not None:
        _unknown_keys(acceptance, ACCEPTANCE_KEYS, "acceptance", issues)
        for key in ACCEPTANCE_KEYS:
            if key not in acceptance:
                _missing("acceptance.{0}".format(key), issues)
        for key in _THRESHOLD_KEYS:
            if key not in acceptance:
                continue
            value = acceptance[key]
            full = "acceptance.{0}".format(key)
            if not _finite(value) or float(value) <= 0:
                issues.append(Issue("error", full, "threshold must be a positive finite number"))
                continue
            proposed = PROPOSED_THRESHOLDS[key]
            if not _near(value, proposed):
                issues.append(Issue(
                    "warning",
                    full,
                    "{0} differs from the workplan proposed value {1}".format(value, proposed),
                ))
        if "runs_required" in acceptance:
            runs = acceptance["runs_required"]
            if isinstance(runs, bool) or not isinstance(runs, int) or runs < MIN_RUNS:
                issues.append(Issue(
                    "error",
                    "acceptance.runs_required",
                    "must be an integer >= {0}".format(MIN_RUNS),
                ))
        if "state" in acceptance:
            state = acceptance["state"]
            if state not in ACCEPTANCE_STATES:
                issues.append(Issue(
                    "error",
                    "acceptance.state",
                    "must be proposed or approved_by_trevor",
                ))

    if "forbidden" in spec:
        forbidden = spec["forbidden"]
        if not isinstance(forbidden, list):
            issues.append(Issue("error", "forbidden", "must be a list"))
        else:
            cleaned = []
            for index, item in enumerate(forbidden):
                if not isinstance(item, str):
                    issues.append(Issue("error", "forbidden[{0}]".format(index), "must be a string"))
                    continue
                cleaned.append(item.strip())
            for phrase in REQUIRED_FORBIDDEN:
                if phrase not in cleaned:
                    issues.append(Issue(
                        "error",
                        "forbidden",
                        "missing required phrase: {0}".format(phrase),
                    ))

    return issues
