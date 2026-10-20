"""Load one walk_result.json and reject claims the numbers do not support.

Schema S4, rules W1–W4. This module does not score a trajectory. It only
checks the document the producer already wrote.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, List, Optional, Sequence

# Milestone gates. A looser value in the file must not create a pass.
# The 0.10 m return gate is the workplan rule.
# The 1 cm placement tolerance and the 5 cm wall gate are provisional:
# Trevor's delegate approved the numbers, and they are not a final freeze.
# The wall gate is a label only. W1 does not accept or reject on wall p95.
RETURN_THRESHOLD_M = 0.10
PLACEMENT_TOLERANCE_M = 0.01
WALL_GATE_M = 0.05
# A measured horizontal or vertical component must fit inside d3.
# When both components exist, d3 must also match their root sum square.
# That 1 mm allowance is consistency only. It does not move the 0.10 m gate.
COMPONENT_TOLERANCE_M = 0.001
REPLAY_LABEL = "replay demonstrated; physical accuracy not evaluated"
WALK_SCHEMA = "spatio.walk_result/1"
_HEX64 = re.compile(r"^[0-9a-f]{64}$")
_METRE_MATCH = 1e-9
# Times are absolute epoch seconds (~1.8e9 s), where float64 resolves only
# ~0.24 us and producers round each stamp to 1 us. A duration equal to a
# window span can therefore differ from it by a few us; 10 us absorbs that and
# is far below any real timing claim.
_SECOND_MATCH = 1e-5

_TOP_KEYS = (
    "schema",
    "run_id",
    "status",
    "reasons",
    "physical_accuracy_evaluated",
    "fastlio_revision",
    "config_sha256",
    "duration_s",
    "path_length_m",
    "return_error",
    "drift_pct_of_path",
    "validity",
    "walls",
    "physical_reference",
)
_RETURN_KEYS = (
    "d3_m",
    "horizontal_m",
    "vertical_m",
    "heading_deg",
    "threshold_m",
    "start_window_s",
    "end_window_s",
)
_VALIDITY_KEYS = ("timestamps_monotonic", "resets", "max_gap_s", "moving")
_WALL_KEYS = (
    "p95_point_to_plane_m",
    "doubled_wall_suspected",
    "gate_m",
    "gate_state",
)
_PLACEMENT_KEYS = (
    "start_placement_residual_m",
    "end_placement_residual_m",
    "heading_check",
    "placement_tolerance_m",
)


class SeriesError(Exception):
    """A problem with one input file. The message names the file."""

    def __init__(self, path: str, message: str) -> None:
        self.path = path
        self.message = message
        super().__init__("%s: %s" % (path, message))


class InputRejected(SeriesError):
    """The file was readable, but it breaks S4 or contradicts its numbers."""


class Unreadable(SeriesError):
    """The file could not be read, or it is not JSON."""


@dataclass
class ReturnError:
    d3_m: Optional[float]
    horizontal_m: Optional[float]
    vertical_m: Optional[float]
    heading_deg: Optional[float]
    threshold_m: Optional[float]
    start_window_s: Optional[List[float]]
    end_window_s: Optional[List[float]]


@dataclass
class Validity:
    timestamps_monotonic: Optional[bool]
    resets: Optional[int]
    max_gap_s: Optional[float]
    moving: Optional[bool]


@dataclass
class Walls:
    p95_point_to_plane_m: Optional[float]
    doubled_wall_suspected: Optional[bool]
    gate_m: float
    gate_state: str


@dataclass
class PhysicalReference:
    start_placement_residual_m: Optional[float]
    end_placement_residual_m: Optional[float]
    heading_check: str
    placement_tolerance_m: float


@dataclass
class WalkResult:
    source_path: str
    run_id: str
    status: str
    reasons: List[str]
    physical_accuracy_evaluated: bool
    accuracy_label: Optional[str]
    fastlio_revision: str
    config_sha256: str
    duration_s: Optional[float]
    path_length_m: Optional[float]
    return_error: ReturnError
    drift_pct_of_path: Optional[float]
    validity: Validity
    walls: Walls
    physical_reference: Optional[PhysicalReference]
    recorded_at: Optional[datetime] = None

    def w1_supports_pass(self) -> bool:
        """True when the milestone numbers themselves allow status PASS.

        Uses the fixed 0.10 m gate on the 3D return, on each measured
        component, and on the combined length of the two components. The
        1 mm consistency tolerance is not part of that comparison. A wider
        threshold from the file is not used.
        """
        error = self.return_error
        validity = self.validity
        walls = self.walls
        if (
            error.d3_m is None
            or error.threshold_m is None
            or validity.timestamps_monotonic is None
            or validity.resets is None
            or validity.moving is None
            or walls.doubled_wall_suspected is None
        ):
            return False
        return (
            not _return_gate_failures(error)
            and _same_metre(error.threshold_m, RETURN_THRESHOLD_M)
            and validity.timestamps_monotonic is True
            and validity.resets == 0
            and validity.moving is True
            and walls.doubled_wall_suspected is False
        )


def load_walk_result(path: str) -> WalkResult:
    """Read one UTF-8 JSON file. Unreadable files raise Unreadable."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = handle.read()
    except UnicodeDecodeError as exc:
        raise Unreadable(path, "file is not UTF-8") from exc
    except OSError as exc:
        raise Unreadable(path, _os_message(exc)) from exc
    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise Unreadable(path, "invalid JSON: %s" % exc) from exc
    return parse_walk_result(data, path)


def parse_walk_result(data: Any, source_path: str) -> WalkResult:
    """Validate an already-parsed JSON value. Reject bad claims."""
    if not isinstance(data, dict):
        raise InputRejected(source_path, "top-level JSON value must be an object")

    errors: List[str] = []
    _require_keys(data, _TOP_KEYS, "walk_result", errors)
    recorded_at = _recorded_at(data.get("recorded_at"), errors)

    schema = data.get("schema")
    if schema != WALK_SCHEMA:
        errors.append('schema must be "%s"' % WALK_SCHEMA)

    run_id = _plain_id(data.get("run_id"), "run_id", errors)
    status = data.get("status")
    if status not in ("PASS", "FAIL", "NOT_EVALUABLE"):
        errors.append("status must be PASS, FAIL, or NOT_EVALUABLE")
        status = None

    reasons = _reasons(data.get("reasons"), errors)
    evaluated = data.get("physical_accuracy_evaluated")
    if not isinstance(evaluated, bool):
        errors.append("physical_accuracy_evaluated must be true or false")
        evaluated = None

    accuracy_label = _optional_text(data, "accuracy_label", errors)
    revision = _plain_id(data.get("fastlio_revision"), "fastlio_revision", errors)
    config = data.get("config_sha256")
    if not isinstance(config, str) or _HEX64.match(config) is None:
        errors.append("config_sha256 must be 64 lowercase hex characters")
        config = None

    duration = _optional_number(
        data.get("duration_s") if "duration_s" in data else _MISSING,
        "duration_s",
        errors,
        minimum=0.0,
    )
    path_length = _optional_number(
        data.get("path_length_m") if "path_length_m" in data else _MISSING,
        "path_length_m",
        errors,
        minimum=0.0,
    )
    drift = _optional_number(
        data.get("drift_pct_of_path") if "drift_pct_of_path" in data else _MISSING,
        "drift_pct_of_path",
        errors,
        minimum=0.0,
    )

    return_error = _parse_return(data.get("return_error"), errors)
    validity = _parse_validity(data.get("validity"), errors)
    walls = _parse_walls(data.get("walls"), errors)
    reference = _parse_reference(
        data.get("physical_reference") if "physical_reference" in data else _MISSING,
        evaluated,
        errors,
    )
    _check_w3_w4(evaluated, accuracy_label, data.get("physical_reference") if "physical_reference" in data else _MISSING, errors)

    # One rule for numbers that cannot be true together. A minimum of 0 on
    # duration_s and path_length_m still allows a measured zero; this is
    # what rejects a zero that contradicts the rest of the file.
    _check_absurd_numbers(duration, path_length, validity, return_error, errors)
    if return_error is not None:
        _check_threshold(return_error, errors)

    if (
        status is not None
        and reasons is not None
        and return_error is not None
        and validity is not None
        and walls is not None
    ):
        _check_status(status, return_error, validity, walls, errors)
        if status != "PASS" and not _usable_reasons(reasons):
            errors.append("status %s requires a non-empty reasons list" % status)

    if errors:
        raise InputRejected(source_path, "; ".join(errors))
    if (
        run_id is None
        or status is None
        or reasons is None
        or evaluated is None
        or revision is None
        or config is None
        or return_error is None
        or validity is None
        or walls is None
    ):
        raise InputRejected(source_path, "walk_result is incomplete")
    return WalkResult(
        source_path=source_path,
        run_id=run_id,
        status=status,
        reasons=reasons,
        physical_accuracy_evaluated=evaluated,
        accuracy_label=accuracy_label,
        fastlio_revision=revision,
        config_sha256=config,
        duration_s=duration,
        path_length_m=path_length,
        return_error=return_error,
        drift_pct_of_path=drift,
        validity=validity,
        walls=walls,
        physical_reference=reference,
        recorded_at=recorded_at,
    )


def _recorded_at(value: Any, errors: List[str]) -> Optional[datetime]:
    if value is None:
        return None  # Legacy explain remains usable; compare requires chronology.
    try:
        if not isinstance(value, str):
            raise ValueError()
        stamp = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if stamp.tzinfo is None:
            raise ValueError()
        return stamp.astimezone(timezone.utc)
    except (ValueError, OverflowError):
        errors.append("recorded_at must be an ISO 8601 timestamp with a timezone")
        return None


class _Missing(object):
    pass


_MISSING = _Missing()


def _os_message(exc: OSError) -> str:
    if isinstance(exc, FileNotFoundError):
        return "file not found"
    text = str(exc).strip()
    return text or exc.__class__.__name__


def _require_keys(obj: dict, keys: Sequence[str], where: str, errors: List[str]) -> None:
    for key in keys:
        if key not in obj:
            errors.append("%s is missing %s" % (where, key))


def _plain_id(value: Any, name: str, errors: List[str]) -> Optional[str]:
    if not isinstance(value, str) or not value.strip():
        errors.append("%s must be a non-empty string" % name)
        return None
    if "\n" in value or "\r" in value or "|" in value:
        errors.append("%s must not contain a newline or a vertical bar" % name)
        return None
    return value


def _optional_text(data: dict, key: str, errors: List[str]) -> Optional[str]:
    if key not in data or data[key] is None:
        return None
    value = data[key]
    if not isinstance(value, str):
        errors.append("%s must be a string or null" % key)
        return None
    return value


def _reasons(value: Any, errors: List[str]) -> Optional[List[str]]:
    if not isinstance(value, list):
        errors.append("reasons must be a list of strings")
        return None
    cleaned: List[str] = []
    for item in value:
        if not isinstance(item, str) or not item.strip():
            errors.append("reasons must be non-blank strings")
            return None
        cleaned.append(item.strip())
    return cleaned


def _usable_reasons(reasons: Sequence[str]) -> bool:
    return any(item.strip() for item in reasons)


def _optional_number(
    value: Any,
    name: str,
    errors: List[str],
    minimum: Optional[float] = None,
    allow_negative: bool = False,
) -> Optional[float]:
    if isinstance(value, _Missing):
        return None
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append("%s must be a number or null" % name)
        return None
    number = float(value)
    if not math.isfinite(number):
        errors.append("%s must be finite" % name)
        return None
    if not allow_negative and minimum is not None and number < minimum:
        errors.append("%s must be >= %s" % (name, minimum))
        return None
    return number


def _optional_bool(value: Any, name: str, errors: List[str]) -> Optional[bool]:
    if value is None:
        return None
    if not isinstance(value, bool):
        errors.append("%s must be true, false, or null" % name)
        return None
    return value


def _optional_window(value: Any, name: str, errors: List[str]) -> Optional[List[float]]:
    # One window only. The start/end pair is checked in _check_absurd_numbers.
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 2:
        errors.append("%s must be [start_s, end_s] or null" % name)
        return None
    start = _optional_number(value[0], "%s start" % name, errors)
    end = _optional_number(value[1], "%s end" % name, errors)
    if start is None or end is None:
        errors.append("%s endpoints must be numbers" % name)
        return None
    if end < start:
        errors.append("%s end is before start" % name)
        return None
    return [start, end]


def _parse_return(value: Any, errors: List[str]) -> Optional[ReturnError]:
    if not isinstance(value, dict):
        errors.append("return_error must be an object")
        return None
    _require_keys(value, _RETURN_KEYS, "return_error", errors)
    return ReturnError(
        d3_m=_optional_number(value.get("d3_m", _MISSING), "return_error.d3_m", errors, minimum=0.0),
        horizontal_m=_optional_number(
            value.get("horizontal_m", _MISSING),
            "return_error.horizontal_m",
            errors,
            minimum=0.0,
        ),
        vertical_m=_optional_number(
            value.get("vertical_m", _MISSING),
            "return_error.vertical_m",
            errors,
            minimum=0.0,
        ),
        heading_deg=_optional_number(
            value.get("heading_deg", _MISSING),
            "return_error.heading_deg",
            errors,
            allow_negative=True,
        ),
        threshold_m=_optional_number(
            value.get("threshold_m", _MISSING),
            "return_error.threshold_m",
            errors,
            minimum=0.0,
        ),
        start_window_s=_optional_window(
            value.get("start_window_s", _MISSING) if "start_window_s" in value else _MISSING,
            "return_error.start_window_s",
            errors,
        )
        if "start_window_s" in value
        else None,
        end_window_s=_optional_window(
            value.get("end_window_s", _MISSING) if "end_window_s" in value else _MISSING,
            "return_error.end_window_s",
            errors,
        )
        if "end_window_s" in value
        else None,
    )


def _parse_validity(value: Any, errors: List[str]) -> Optional[Validity]:
    if not isinstance(value, dict):
        errors.append("validity must be an object")
        return None
    _require_keys(value, _VALIDITY_KEYS, "validity", errors)
    resets = _optional_resets(value.get("resets", _MISSING), errors)
    return Validity(
        timestamps_monotonic=_optional_bool(
            value.get("timestamps_monotonic", _MISSING) if "timestamps_monotonic" in value else _MISSING,
            "validity.timestamps_monotonic",
            errors,
        )
        if "timestamps_monotonic" in value
        else None,
        resets=resets,
        max_gap_s=_optional_number(
            value.get("max_gap_s", _MISSING) if "max_gap_s" in value else _MISSING,
            "validity.max_gap_s",
            errors,
            minimum=0.0,
        )
        if "max_gap_s" in value
        else None,
        moving=_optional_bool(
            value.get("moving", _MISSING) if "moving" in value else _MISSING,
            "validity.moving",
            errors,
        )
        if "moving" in value
        else None,
    )


def _optional_resets(value: Any, errors: List[str]) -> Optional[int]:
    if isinstance(value, _Missing) or value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append("validity.resets must be a whole number or null")
        return None
    if isinstance(value, float) and not value.is_integer():
        errors.append("validity.resets must be a whole number or null")
        return None
    if not math.isfinite(float(value)):
        errors.append("validity.resets must be finite")
        return None
    resets = int(value)
    if resets < 0:
        errors.append("validity.resets must be >= 0")
        return None
    return resets


def _parse_walls(value: Any, errors: List[str]) -> Optional[Walls]:
    if not isinstance(value, dict):
        errors.append("walls must be an object")
        return None
    _require_keys(value, _WALL_KEYS, "walls", errors)
    if "gate_m" not in value or value.get("gate_m") is None:
        if "gate_m" in value:
            errors.append("walls.gate_m must be a number > 0")
        gate_number = None
    else:
        gate_number = _optional_number(value["gate_m"], "walls.gate_m", errors, minimum=0.0)
        if gate_number is not None and gate_number <= 0.0:
            errors.append("walls.gate_m must be > 0")
            gate_number = None
    gate_state = value.get("gate_state")
    if gate_state not in ("proposed", "approved"):
        errors.append('walls.gate_state must be "proposed" or "approved"')
        gate_state = None
    if gate_number is None or gate_state is None:
        return None
    return Walls(
        p95_point_to_plane_m=_optional_number(
            value.get("p95_point_to_plane_m", _MISSING) if "p95_point_to_plane_m" in value else _MISSING,
            "walls.p95_point_to_plane_m",
            errors,
            minimum=0.0,
        )
        if "p95_point_to_plane_m" in value
        else None,
        doubled_wall_suspected=_optional_bool(
            value.get("doubled_wall_suspected", _MISSING) if "doubled_wall_suspected" in value else _MISSING,
            "walls.doubled_wall_suspected",
            errors,
        )
        if "doubled_wall_suspected" in value
        else None,
        gate_m=gate_number,
        gate_state=gate_state,
    )


def _parse_reference(
    value: Any,
    evaluated: Optional[bool],
    errors: List[str],
) -> Optional[PhysicalReference]:
    if isinstance(value, _Missing):
        return None
    if value is None:
        return None
    if not isinstance(value, dict):
        errors.append("physical_reference must be an object or null")
        return None
    # W3 is reported separately. Still parse a present object so other errors show up
    # when the run claims physical accuracy was evaluated.
    if evaluated is False:
        return None
    _require_keys(value, _PLACEMENT_KEYS, "physical_reference", errors)
    start = _optional_number(
        value.get("start_placement_residual_m", _MISSING) if "start_placement_residual_m" in value else _MISSING,
        "physical_reference.start_placement_residual_m",
        errors,
        minimum=0.0,
    )
    end = _optional_number(
        value.get("end_placement_residual_m", _MISSING) if "end_placement_residual_m" in value else _MISSING,
        "physical_reference.end_placement_residual_m",
        errors,
        minimum=0.0,
    )
    heading = value.get("heading_check")
    if heading not in ("ok", "off", "not_checked"):
        errors.append('physical_reference.heading_check must be "ok", "off", or "not_checked"')
        heading = None
    tolerance = _optional_number(
        value.get("placement_tolerance_m", _MISSING) if "placement_tolerance_m" in value else _MISSING,
        "physical_reference.placement_tolerance_m",
        errors,
        minimum=0.0,
    )
    if tolerance is None or heading is None:
        return None
    if not _same_metre(tolerance, PLACEMENT_TOLERANCE_M):
        errors.append(
            "physical_reference.placement_tolerance_m must be 0.01 "
            "(provisional placement tolerance); the file says %s" % tolerance
        )
        return None
    return PhysicalReference(
        start_placement_residual_m=start,
        end_placement_residual_m=end,
        heading_check=heading,
        placement_tolerance_m=tolerance,
    )


def _check_w3_w4(
    evaluated: Optional[bool],
    accuracy_label: Optional[str],
    reference_raw: Any,
    errors: List[str],
) -> None:
    if evaluated is None or isinstance(reference_raw, _Missing):
        return
    if evaluated is False:
        if accuracy_label != REPLAY_LABEL:
            errors.append(
                'accuracy_label must be "%s" when physical_accuracy_evaluated is false'
                % REPLAY_LABEL
            )
        if reference_raw is not None:
            errors.append(
                "physical_reference must be null when physical_accuracy_evaluated is false"
            )
        return
    if reference_raw is None:
        errors.append(
            "physical_reference is required when physical_accuracy_evaluated is true"
        )


def _check_absurd_numbers(
    duration: Optional[float],
    path_length: Optional[float],
    validity: Optional[Validity],
    error: Optional[ReturnError],
    errors: List[str],
) -> None:
    """Reject numeric claims that cannot describe one return.

    This is the one parser rule for an absurd combination. It does not add a
    minimum path, a minimum duration, a minimum gap, or a window length.
    Five checks:

    - each measured return component must fit inside d3, and both together
      must match it;
    - validity.moving true cannot sit beside a path length of exactly 0;
    - when both endpoint windows are present, the end window must start
      after the start window ends. A shared instant counts as overlap;
    - a duration of exactly 0 cannot sit beside a moving claim, a positive
      path length, or endpoint windows that span a positive time;
    - a measured duration cannot be shorter than one endpoint window, and
      cannot be shorter than the span of both. Each present window is
      checked on its own. A duration equal to that length still fits. A
      null duration is not measured. A null window is skipped, not read
      as zero.
    """
    if error is not None:
        _check_components(error, errors)
        _check_endpoint_windows(error, errors)
    _check_zero_path(path_length, validity, errors)
    _check_zero_duration(duration, path_length, validity, error, errors)
    _check_duration_covers_span(duration, error, errors)


def _check_zero_path(
    path_length: Optional[float],
    validity: Optional[Validity],
    errors: List[str],
) -> None:
    if path_length is None or validity is None or validity.moving is not True:
        return
    if path_length == 0.0:
        errors.append(
            "path_length_m is 0 while validity.moving is true; "
            "a moving run cannot have a zero-length path"
        )


def _check_zero_duration(
    duration: Optional[float],
    path_length: Optional[float],
    validity: Optional[Validity],
    error: Optional[ReturnError],
    errors: List[str],
) -> None:
    """Reject a measured duration of exactly 0 that contradicts the run.

    Called only from _check_absurd_numbers. Exact 0 is the cutoff for this
    check. A positive duration that is shorter than one endpoint window,
    or shorter than the span of both, is rejected by
    _check_duration_covers_span. Neither check is a minimum duration. A
    null duration is not measured.
    """
    if duration is None or duration != 0.0:
        return
    claims = []
    if validity is not None and validity.moving is True:
        claims.append("validity.moving is true")
    if path_length is not None and path_length > 0.0:
        claims.append("path_length_m is %s" % path_length)
    span = _endpoint_span_s(error)
    if span is not None and span > 0.0:
        claims.append("the endpoint windows span %.6f s" % span)
    if not claims:
        return
    errors.append(
        "duration_s is 0 while %s; a measured duration of 0 cannot sit beside that claim"
        % " and ".join(claims)
    )


def _check_duration_covers_span(
    duration: Optional[float],
    error: Optional[ReturnError],
    errors: List[str],
) -> None:
    """Reject a measured duration shorter than a window or their paired span.

    Called only from _check_absurd_numbers. Each present window is checked
    on its own: its length is the end of that window minus its start. When
    both windows are present, the paired span is also checked: the end of
    the end window minus the start of the start window. Both comparisons
    are lengths, so the clock does not have to start at zero. A duration
    equal to a length still fits. A null window is skipped. A null duration
    is not measured. Zero duration must also cover each present window,
    even when the other window is null. This is not a minimum duration.
    """
    if duration is None or error is None:
        return
    _check_one_window_duration(
        duration, error.start_window_s, "return_error.start_window_s", errors
    )
    _check_one_window_duration(
        duration, error.end_window_s, "return_error.end_window_s", errors
    )
    span = _endpoint_span_s(error)
    if span is None or span <= 0.0:
        return
    # _SECOND_MATCH absorbs epoch-second rounding so a duration equal to the span loads.
    if span - duration > _SECOND_MATCH:
        errors.append(
            "duration_s is %s while the endpoint windows span %.6f s; "
            "a run cannot be shorter than the time its endpoint windows cover"
            % (duration, span)
        )


def _check_one_window_duration(
    duration: float,
    window: Optional[List[float]],
    name: str,
    errors: List[str],
) -> None:
    if window is None:
        return
    length = window[1] - window[0]
    # _SECOND_MATCH absorbs epoch-second rounding so a duration equal to the window loads.
    if length - duration > _SECOND_MATCH:
        errors.append(
            "duration_s is %s while %s covers %.6f s; "
            "a run cannot be shorter than one of its endpoint windows"
            % (duration, name, length)
        )


def _endpoint_span_s(error: Optional[ReturnError]) -> Optional[float]:
    if error is None or error.start_window_s is None or error.end_window_s is None:
        return None
    return error.end_window_s[1] - error.start_window_s[0]


def _check_endpoint_windows(error: ReturnError, errors: List[str]) -> None:
    start = error.start_window_s
    end = error.end_window_s
    if start is None or end is None:
        return
    if end[0] > start[1]:
        return
    if end[1] < start[0]:
        relation = "are reversed"
    else:
        relation = "overlap"
    errors.append(
        "return_error.end_window_s %s and return_error.start_window_s %s %s; "
        "the end window must start after the start window ends"
        % (end, start, relation)
    )


def _check_components(error: ReturnError, errors: List[str]) -> None:
    """Reject a component that cannot fit inside the reported 3D return.

    Called only from _check_absurd_numbers. A null axis does not skip the
    check: each measured component is checked on its own against d3. When
    both components are present they must also match d3.
    """
    if error.d3_m is None:
        return
    _check_one_component(error.d3_m, error.horizontal_m, "horizontal_m", errors)
    _check_one_component(error.d3_m, error.vertical_m, "vertical_m", errors)
    if error.horizontal_m is None or error.vertical_m is None:
        return
    expected = math.sqrt(
        error.horizontal_m * error.horizontal_m + error.vertical_m * error.vertical_m
    )
    if abs(error.d3_m - expected) > COMPONENT_TOLERANCE_M:
        errors.append(
            "return_error.d3_m %s does not match sqrt(horizontal_m^2 + vertical_m^2) = %.6f "
            "(tolerance 0.001 m)" % (error.d3_m, expected)
        )


def _check_one_component(
    d3_m: float,
    component: Optional[float],
    name: str,
    errors: List[str],
) -> None:
    if component is None:
        return
    # COMPONENT_TOLERANCE_M is 1 mm. _METRE_MATCH absorbs binary rounding so
    # a component of exactly d3 + 0.001 m is still inside the allowance.
    if component - d3_m > COMPONENT_TOLERANCE_M + _METRE_MATCH:
        errors.append(
            "return_error.%s %s exceeds return_error.d3_m %s "
            "(a measured component cannot exceed the 3D return; tolerance 0.001 m)"
            % (name, component, d3_m)
        )


_GATE_NOTE = "strict return gate; the 0.001 m consistency tolerance does not apply"


def _return_gate_failures(error: ReturnError) -> List[str]:
    """Lengths that are not strictly under the 0.10 m return gate.

    The 1 mm consistency tolerance is not applied. A null component is not
    measured, so it is skipped. The combined length is checked only when
    both components are present, and a null is never read as zero.
    """
    bits = []
    if error.d3_m is None or not error.d3_m < RETURN_THRESHOLD_M:
        bits.append(
            "return_error.d3_m %s is not < %s" % (error.d3_m, RETURN_THRESHOLD_M)
        )
    for name, component in (
        ("horizontal_m", error.horizontal_m),
        ("vertical_m", error.vertical_m),
    ):
        if component is not None and not component < RETURN_THRESHOLD_M:
            bits.append(
                "return_error.%s %s is not < %s (%s)"
                % (name, component, RETURN_THRESHOLD_M, _GATE_NOTE)
            )
    if error.horizontal_m is not None and error.vertical_m is not None:
        norm = math.sqrt(
            error.horizontal_m * error.horizontal_m
            + error.vertical_m * error.vertical_m
        )
        if not norm < RETURN_THRESHOLD_M:
            bits.append(
                "sqrt(horizontal_m^2 + vertical_m^2) = %.6f is not < %s (%s)"
                % (norm, RETURN_THRESHOLD_M, _GATE_NOTE)
            )
    return bits


def _check_threshold(error: ReturnError, errors: List[str]) -> None:
    if error.threshold_m is None:
        return
    if not _same_metre(error.threshold_m, RETURN_THRESHOLD_M):
        errors.append(
            "return_error.threshold_m must be 0.10 (milestone return-error gate); "
            "the file says %s" % error.threshold_m
        )


def _check_status(
    status: str,
    error: ReturnError,
    validity: Validity,
    walls: Walls,
    errors: List[str],
) -> None:
    """Reject a status the numbers contradict.

    A null needed field forces NOT_EVALUABLE (W2). A PASS is legal only when
    W1 holds. FAIL is allowed when the numbers would still allow PASS: W1 does
    not force PASS, and this tool does not upgrade status. NOT_EVALUABLE is
    rejected when every needed field is present, because the run is then evaluable.
    """
    missing = _missing_needed(error, validity, walls)
    if missing:
        if status != "NOT_EVALUABLE":
            errors.append(
                "status %s contradicts numbers: %s is null, which forces NOT_EVALUABLE"
                % (status, ", ".join(missing))
            )
        return
    if status == "NOT_EVALUABLE":
        errors.append(
            "status NOT_EVALUABLE contradicts numbers: the fields needed for a pass "
            "are all present, so the run is evaluable"
        )
        return
    if status != "PASS":
        return
    bits = _pass_failures(error, validity, walls)
    if bits:
        errors.append("status PASS contradicts numbers: " + "; ".join(bits))


def _missing_needed(error: ReturnError, validity: Validity, walls: Walls) -> List[str]:
    missing = []
    if error.d3_m is None:
        missing.append("return_error.d3_m")
    if error.threshold_m is None:
        missing.append("return_error.threshold_m")
    if validity.timestamps_monotonic is None:
        missing.append("validity.timestamps_monotonic")
    if validity.resets is None:
        missing.append("validity.resets")
    if validity.moving is None:
        missing.append("validity.moving")
    if walls.doubled_wall_suspected is None:
        missing.append("walls.doubled_wall_suspected")
    return missing


def _pass_failures(error: ReturnError, validity: Validity, walls: Walls) -> List[str]:
    bits = []
    if error.threshold_m is None or not _same_metre(error.threshold_m, RETURN_THRESHOLD_M):
        bits.append("return_error.threshold_m is not 0.10")
    else:
        bits.extend(_return_gate_failures(error))
    if validity.timestamps_monotonic is not True:
        bits.append("validity.timestamps_monotonic is %s" % validity.timestamps_monotonic)
    if validity.resets != 0:
        bits.append("validity.resets is %s" % validity.resets)
    if validity.moving is not True:
        bits.append("validity.moving is %s" % validity.moving)
    if walls.doubled_wall_suspected is not False:
        bits.append("walls.doubled_wall_suspected is %s" % walls.doubled_wall_suspected)
    return bits


def _same_metre(value: float, expected: float) -> bool:
    return abs(value - expected) <= _METRE_MATCH
