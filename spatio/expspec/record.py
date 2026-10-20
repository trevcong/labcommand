"""Write physical_reference.json. This file is what a later manifest may cite.

The command does not set physical_accuracy_evaluated. A null residual, a
heading check other than ok, or a residual above the frozen tolerance is
recorded and flagged not within tolerance. Consumers must not treat that
as a pass.
"""

from __future__ import annotations

import math
import re
from datetime import datetime
from pathlib import Path

from spatio.expspec.canonical import canonical_bytes
from spatio.expspec.constants import HEADING_VALUES, REFERENCE_KEYS, REFERENCE_SCHEMA
from spatio.expspec.freeze import FrozenSpecError, load_frozen
from spatio.expspec.io_util import write_bytes_atomic

_MEASURED_AT = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}[+-]\d{2}:\d{2}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class RecordError(Exception):
    pass


class RecordResult(object):
    def __init__(self, document, within_tolerance, reasons):
        self.document = document
        self.within_tolerance = within_tolerance
        self.reasons = list(reasons)


def parse_residual_arg(text, flag):
    """Parse a CLI residual. The token null means not measured."""
    if text is None:
        raise RecordError("{0} is required (use null if not measured)".format(flag))
    token = str(text).strip()
    if token == "null":
        return None
    if token == "":
        raise RecordError("{0} is required (use null if not measured)".format(flag))
    try:
        value = float(token)
    except ValueError:
        raise RecordError("{0} must be a number or null".format(flag))
    return _check_residual(value, flag)


def _check_residual(value, flag):
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RecordError("{0} must be a number >= 0 or null".format(flag))
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise RecordError("{0} must be finite and >= 0, or null".format(flag))
    if number == 0:
        return 0.0
    return number


def placement_within_tolerance(document):
    """P2, plus a residual above the frozen tolerance.

    Null is not zero. heading_check other than ok is not a pass.
    """
    reasons = []
    start = document.get("start_placement_residual_m")
    end = document.get("end_placement_residual_m")
    heading = document.get("heading_check")
    tolerance = document.get("placement_tolerance_m")
    if start is None:
        reasons.append("start_placement_residual_m is null")
    if end is None:
        reasons.append("end_placement_residual_m is null")
    if heading != "ok":
        reasons.append("heading_check is {0}, not ok".format(heading))
    if start is not None and end is not None and isinstance(tolerance, (int, float)):
        if start > tolerance:
            reasons.append("start residual is above placement_tolerance_m")
        if end > tolerance:
            reasons.append("end residual is above placement_tolerance_m")
    return (len(reasons) == 0), reasons


def _now_stamp():
    return datetime.now().astimezone().replace(microsecond=0).isoformat()


def _near(left, right):
    return abs(float(left) - float(right)) <= 1e-9


def validate_physical_reference(document, spec, spec_sha256):
    """Return a list of error strings. Empty means the document matches S3."""
    problems = []
    if not isinstance(document, dict):
        return ["physical reference must be an object"]
    for key in REFERENCE_KEYS:
        if key not in document:
            problems.append("missing key: {0}".format(key))
    for key in document:
        if key not in REFERENCE_KEYS:
            problems.append("unknown key: {0}".format(key))
    if document.get("schema") != REFERENCE_SCHEMA:
        problems.append("schema must be {0}".format(REFERENCE_SCHEMA))
    run_id = document.get("run_id")
    if not isinstance(run_id, str) or run_id.strip() == "" or "/" in run_id or "\\" in run_id:
        problems.append("run_id must be a non-empty folder name")
    if document.get("spec_id") != spec.get("spec_id"):
        problems.append("spec_id does not match the frozen spec")
    digest = document.get("spec_sha256")
    if not isinstance(digest, str) or _SHA256.match(digest) is None or digest != spec_sha256:
        problems.append("spec_sha256 does not match the frozen spec file")
    operator = document.get("operator")
    if not isinstance(operator, str) or operator.strip() == "":
        problems.append("operator must be a non-empty string")
    measured_at = document.get("measured_at")
    if not isinstance(measured_at, str) or _MEASURED_AT.match(measured_at) is None:
        problems.append("measured_at must be ISO 8601 with a numeric offset")
    for flag in ("start_placement_residual_m", "end_placement_residual_m"):
        value = document.get(flag)
        if value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            problems.append("{0} must be a number or null".format(flag))
            continue
        if not math.isfinite(float(value)) or float(value) < 0:
            problems.append("{0} must be finite and >= 0, or null".format(flag))
    if document.get("heading_check") not in HEADING_VALUES:
        problems.append("heading_check must be ok, off, or not_checked")
    tolerance = document.get("placement_tolerance_m")
    spec_tolerance = spec.get("acceptance", {}).get("placement_tolerance_m")
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)):
        problems.append("placement_tolerance_m must be a positive number")
    elif not math.isfinite(float(tolerance)) or float(tolerance) <= 0:
        problems.append("placement_tolerance_m must be a positive finite number")
    elif not _near(tolerance, spec_tolerance):
        problems.append("placement_tolerance_m must match the frozen spec")
    method = document.get("method")
    if not isinstance(method, str) or method.strip() == "":
        problems.append("method must be a non-empty string")
    if not isinstance(document.get("notes"), str):
        problems.append("notes must be a string")
    return problems


def record_physical_reference(
    spec_path,
    run_id,
    start_residual_m,
    end_residual_m,
    heading,
    operator,
    method,
    notes,
    out_path,
    measured_at=None,
):
    """Validate and write S3. Raise RecordError and write nothing on refusal."""
    try:
        spec, digest = load_frozen(spec_path)
    except FrozenSpecError as exc:
        raise RecordError(str(exc))

    start = _check_residual(start_residual_m, "start_placement_residual_m")
    end = _check_residual(end_residual_m, "end_placement_residual_m")
    if heading not in HEADING_VALUES:
        raise RecordError("heading must be ok, off, or not_checked")
    if not isinstance(operator, str) or operator.strip() == "":
        raise RecordError("operator must be a non-empty string")
    if not isinstance(method, str) or method.strip() == "":
        raise RecordError("method must be a non-empty string")
    if notes is None:
        notes = ""
    if not isinstance(notes, str):
        raise RecordError("notes must be a string")
    if not isinstance(run_id, str) or run_id.strip() == "" or "/" in run_id or "\\" in run_id:
        raise RecordError("run_id must be a non-empty folder name")
    if measured_at is None:
        measured_at = _now_stamp()
    if _MEASURED_AT.match(measured_at) is None:
        raise RecordError("measured_at must be ISO 8601 with a numeric offset")

    document = {
        "schema": REFERENCE_SCHEMA,
        "run_id": run_id,
        "spec_id": spec["spec_id"],
        "spec_sha256": digest,
        "operator": operator,
        "measured_at": measured_at,
        "start_placement_residual_m": start,
        "end_placement_residual_m": end,
        "heading_check": heading,
        "placement_tolerance_m": spec["acceptance"]["placement_tolerance_m"],
        "method": method,
        "notes": notes,
    }
    problems = validate_physical_reference(document, spec, digest)
    if problems:
        raise RecordError("; ".join(problems))
    within, reasons = placement_within_tolerance(document)
    write_bytes_atomic(Path(out_path), canonical_bytes(document))
    return RecordResult(document, within, reasons)
