"""S3 physical_reference.json checks.

P2 is a consumer rule: a null residual or a heading other than ``ok`` means
the placement is not within tolerance. That is a warning, not an error.
The numbers were measured; they are not a pass.
"""

from __future__ import annotations

import math

from spatio.manifest.checks import (
    is_finite_number,
    is_sha256_hex,
    parse_iso8601_with_offset,
)
from spatio.manifest.findings import error, warning


PHYSICAL_SCHEMA = "spatio.physical_reference/1"
# 1 cm. Approved as provisional by Trevor's delegate. A different positive
# value warns (P3) and is not rejected.
PROPOSED_TOLERANCE_M = 0.01
HEADING_VALUES = ("ok", "off", "not_checked")

_STRING_FIELDS = (
    "schema",
    "run_id",
    "spec_id",
    "operator",
    "method",
    "notes",
)

REQUIRED_FIELDS = _STRING_FIELDS + (
    "spec_sha256",
    "measured_at",
    "start_placement_residual_m",
    "end_placement_residual_m",
    "heading_check",
    "placement_tolerance_m",
)

_KNOWN_FIELDS = frozenset(REQUIRED_FIELDS)


def placement_within_tolerance(data):
    """True only when both residuals are measured and inside the tolerance.

    A null residual, a bad heading, or a residual above the stated tolerance
    all return False. Callers must not treat False as a pass.
    """
    if not isinstance(data, dict):
        return False
    start = data.get("start_placement_residual_m", None)
    end = data.get("end_placement_residual_m", None)
    heading = data.get("heading_check", None)
    tolerance = data.get("placement_tolerance_m", None)
    if start is None or end is None:
        return False
    if not is_finite_number(start) or not is_finite_number(end):
        return False
    if not is_finite_number(tolerance) or tolerance <= 0:
        return False
    if start < 0 or end < 0:
        return False
    if heading != "ok":
        return False
    return start <= tolerance and end <= tolerance


def _residual_findings(data, field):
    if field not in data:
        return []
    value = data[field]
    if value is None:
        return []
    if not is_finite_number(value) or value < 0:
        return [
            error(
                "P1",
                "%s must be a finite number >= 0, or null if it was not measured"
                % field,
            )
        ]
    return []


def validate_physical_reference(data, expected_run_id=None):
    """Return findings for one S3 object. ``data`` is already a dict."""
    findings = []
    if not isinstance(data, dict):
        return [error("S3", "physical_reference.json must be a JSON object")]

    for field in REQUIRED_FIELDS:
        if field not in data:
            findings.append(error("S3", "missing required key: %s" % field))

    for field in _STRING_FIELDS:
        if field in data and not isinstance(data[field], str):
            findings.append(error("S3", "%s must be a string" % field))

    for key in sorted(data):
        if key not in _KNOWN_FIELDS:
            findings.append(
                warning("S3", "unknown key in physical_reference.json: %s" % key)
            )

    schema = data.get("schema")
    if isinstance(schema, str) and schema != PHYSICAL_SCHEMA:
        findings.append(
            error(
                "S3",
                "schema must be %s" % PHYSICAL_SCHEMA,
            )
        )

    spec_sha = data.get("spec_sha256")
    if "spec_sha256" in data and not is_sha256_hex(spec_sha):
        findings.append(
            error("S3", "spec_sha256 must be 64 lowercase hexadecimal characters")
        )

    measured_at = data.get("measured_at")
    if isinstance(measured_at, str) and parse_iso8601_with_offset(measured_at) is None:
        findings.append(
            error("S3", "measured_at must be ISO 8601 with a timezone offset")
        )
    elif "measured_at" in data and not isinstance(measured_at, str):
        findings.append(error("S3", "measured_at must be a string"))

    heading = data.get("heading_check")
    if "heading_check" in data and heading not in HEADING_VALUES:
        findings.append(
            error(
                "S3",
                "heading_check must be one of: %s" % ", ".join(HEADING_VALUES),
            )
        )

    findings.extend(_residual_findings(data, "start_placement_residual_m"))
    findings.extend(_residual_findings(data, "end_placement_residual_m"))

    if "placement_tolerance_m" in data:
        tolerance = data["placement_tolerance_m"]
        if not is_finite_number(tolerance) or tolerance <= 0:
            findings.append(
                error(
                    "S3",
                    "placement_tolerance_m must be a finite number > 0",
                )
            )

    run_id = data.get("run_id")
    if (
        expected_run_id is not None
        and isinstance(run_id, str)
        and run_id != expected_run_id
    ):
        findings.append(
            error(
                "M4",
                "physical_reference.json run_id %r does not match session folder %r"
                % (run_id, expected_run_id),
            )
        )

    structural_errors = [
        item for item in findings if item.severity == "ERROR" and item.rule in ("S3", "P1", "M4")
    ]
    if not structural_errors:
        if not placement_within_tolerance(data):
            findings.append(
                warning(
                    "P2",
                    "placement is not within tolerance; do not treat it as a pass",
                )
            )
        tolerance = data.get("placement_tolerance_m")
        if is_finite_number(tolerance) and not math.isclose(
            tolerance, PROPOSED_TOLERANCE_M, rel_tol=0.0, abs_tol=1e-9
        ):
            findings.append(
                warning(
                    "P3",
                    "placement_tolerance_m is %s; the approved provisional "
                    "tolerance is 0.01 m" % tolerance,
                )
            )
    return findings
