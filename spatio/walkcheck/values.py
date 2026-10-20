"""Explicit missing measurements at the walkcheck JSON boundary.

Scoring and the existing walk_result/1 contract still use None. A missing
measurement is serialized with its reason, while optional metadata stays null.
"""

import math


MEASUREMENT_BLOCKS = {
    "timestamps", "return_to_start", "endpoints", "holds", "motion", "drift",
    "intermediate_stops", "walls",
}
# Only these nullable fields represent computations that could not be made.
# Other nulls (errors, event/hold selectors, optional metadata) retain their
# meaning. Keep this tied to the producer schema rather than tagging every null.
NULLABLE_MEASUREMENTS = {
    "timestamps": {"t_start", "t_end", "duration_s", "median_dt_s", "max_dt_s"},
    "motion": {"return_error_pct_of_path"},
    "drift": {"path_length_m", "return_error_pct_of_path"},
    "intermediate_stops.stops[]": {"max_mean_separation_cm"},
    "walls": {
        "bounds_min_m", "bounds_max_m", "p95_point_to_plane_cm",
        "median_point_to_plane_cm", "headline_phase_origin_m", "doubled_wall",
        "revisit_separation_cm", "max_revisit_separation_cm", "closest_parallel_cm",
    },
    "walls.phases[]": {"p95_cm"},
}
WALL_ABSENCE_RESULTS = {
    "revisit_separation_cm", "max_revisit_separation_cm", "closest_parallel_cm",
}


def explicit_measurements(report):
    """Tag unavailable measurements without changing finite values or verdicts."""
    def encode(value, path, reason, unavailable=False):
        if value is None:
            return {"status": "not_measured", "reason": f"{path}: {reason}"} if unavailable else None
        if isinstance(value, dict):
            reason = value.get("reason") or reason
            fields = NULLABLE_MEASUREMENTS.get(path, set())
            if path == "walls" and value.get("status") == "OK":
                # The search ran: null separation means no matching pair was
                # found. An unevaluable wall has not run that search.
                fields = fields - WALL_ABSENCE_RESULTS
            return {k: encode(v, f"{path}.{k}", reason, k in fields)
                    for k, v in value.items()}
        if isinstance(value, list):
            return [encode(v, f"{path}[]", reason) for v in value]
        return value

    reason = report.get("detail") or "; ".join(report.get("reasons") or [])
    return {k: encode(v, k, reason or "no measurement available from this check", True)
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
