"""Explicit missing measurements at the walkcheck JSON boundary.

Scoring and the existing walk_result/1 contract still use None. A missing
measurement is serialized with its reason, while optional metadata stays null.
"""

import math


MEASUREMENT_BLOCKS = {
    "timestamps", "return_to_start", "endpoints", "holds", "motion", "drift",
    "intermediate_stops", "walls",
}
OPTIONAL_METADATA = {"reason", "detail", "fault", "source", "first_gap", "first_backward"}


def explicit_measurements(report):
    """Tag unavailable measurements without changing finite values or verdicts."""
    def encode(value, path, reason):
        if value is None:
            return {"status": "not_measured", "reason": f"{path}: {reason}"}
        if isinstance(value, dict):
            reason = value.get("reason") or reason
            return {k: v if k in OPTIONAL_METADATA else encode(v, f"{path}.{k}", reason)
                    for k, v in value.items()}
        if isinstance(value, list):
            return [encode(v, f"{path}[{i}]", reason) for i, v in enumerate(value)]
        return value

    reason = report.get("detail") or "; ".join(report.get("reasons") or [])
    return {k: encode(v, k, reason or "no measurement available from this check")
            if k in MEASUREMENT_BLOCKS else v for k, v in report.items()}


def measurement_values(value):
    """Read explicit not_measured values and legacy nulls; reject invalid numbers.

    Decode before scoring so a truthy status object cannot become evidence.
    """
    if isinstance(value, dict):
        if value.get("status") == "not_measured":
            if set(value) != {"status", "reason"} or not isinstance(value["reason"], str) \
                    or not value["reason"].strip():
                raise ValueError("not_measured requires a non-blank reason and no measured value")
            return None
        return {k: measurement_values(v) for k, v in value.items()}
    if isinstance(value, list):
        return [measurement_values(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("non-finite measured value")
    return value
