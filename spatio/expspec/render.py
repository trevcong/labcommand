"""Render one page. The frozen JSON is the source of truth; this is the page Trevor reads."""

from __future__ import annotations

import math

from spatio.expspec.constants import (
    APPROVED_BANNER,
    MAX_RENDER_LINES,
    PROPOSED_BANNER,
    RETURN_ERROR_SENTENCE,
)
from spatio.expspec.numbers import format_quantity


class RenderError(Exception):
    pass

# Wide enough that the return-error sentence stays on one line (tests look for
# that exact sentence). Long frame descriptions still wrap inside the 70-line cap.
_WRAP_WIDTH = 100


def _one_line(value):
    return " ".join(str(value).split())


def _labelled(label, value, rest):
    """Join a field value to the renderer's explanation without a double period."""
    text = _one_line(value).rstrip()
    if text.endswith("."):
        text = text[:-1].rstrip()
    return "{0}: {1}. {2}".format(label, text, rest)


def _metres(value):
    """Print a gate with enough digits that a different gate stays different."""
    if isinstance(value, bool):
        return _one_line(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return _one_line(value)
    if not math.isfinite(number):
        return _one_line(value)
    return format_quantity(number)


def _wrap_line(line):
    if line == "" or line.startswith("#") or len(line) <= _WRAP_WIDTH:
        return [line]
    if RETURN_ERROR_SENTENCE in line or line in (PROPOSED_BANNER, APPROVED_BANNER):
        return [line]
    indent = "  " if line.startswith("- ") else ""
    words = line.split()
    wrapped = []
    current = ""
    for word in words:
        candidate = word if current == "" else current + " " + word
        if current and len(candidate) > _WRAP_WIDTH:
            wrapped.append(current)
            current = indent + word
        else:
            current = candidate
    if current:
        wrapped.append(current)
    return wrapped or [line]


def _box_text(box):
    if box is None:
        return "box: null (not chosen yet; valid only for the session it is later chosen on)"
    return "box center_m {0}, normal_hint {1}, half_extent_m {2}".format(
        box.get("center_m"),
        box.get("normal_hint"),
        box.get("half_extent_m"),
    )


def render_markdown(spec):
    """Return the one-page spec as UTF-8 text with \\n line endings.

    Free text is flattened to one line so a newline inside a field cannot
    quietly turn the page into two. More than 70 lines is an error.
    """
    acceptance = spec["acceptance"]
    frames = spec["frames"]
    route = spec["route"]
    windows = spec["stationary_windows_s"]
    state = acceptance.get("state")
    lines = []
    lines.append("# Spatio milestone 1 experiment spec: {0}".format(spec.get("spec_id", "")))
    lines.append("")
    if state == "approved_by_trevor":
        lines.append(APPROVED_BANNER)
    else:
        lines.append(PROPOSED_BANNER)
    lines.append("")
    lines.append("Sensor: {0}. FAST-LIO revision: {1}.".format(
        _one_line(spec.get("sensor", "")),
        _one_line(spec.get("fastlio_revision", "")),
    ))
    lines.append("Config file: {0}.".format(_one_line(spec.get("config_file", ""))))
    lines.append("Config sha256: {0}.".format(_one_line(spec.get("config_sha256", ""))))
    notes = spec.get("notes") or ""
    if isinstance(notes, str) and notes.strip():
        lines.append("Notes: {0}".format(_one_line(notes)))
    lines.append("")
    lines.append("## Route")
    lines.append(_one_line(route.get("description", "")))
    duration = route.get("duration_target_s") or [None, None]
    lines.append("Duration target: {0}–{1} s.".format(duration[0], duration[1]))
    stops = route.get("intermediate_stops") or []
    lines.append("Intermediate stops: {0}.".format("; ".join(_one_line(item) for item in stops)))
    lines.append("")
    lines.append("## Reference point and start marks")
    lines.append("Reference point: {0}.".format(_one_line(spec.get("reference_point", ""))))
    lines.append("Start marks: {0}.".format(_one_line(spec.get("start_marks", ""))))
    lines.append(
        "The tape measures that reference point against the marks. "
        "The tape residual is not the estimator return error."
    )
    lines.append("")
    lines.append("## Frames")
    lines.append(_labelled(
        "map",
        frames.get("map", ""),
        "This is the FAST-LIO world frame for this run only. "
        "The origin is this run's start. A coordinate or a wall-patch box from another run "
        "is not valid here. Milestone 1 does not register runs together.",
    ))
    lines.append(_labelled(
        "body",
        frames.get("body", ""),
        "This is the rigid body whose pose is scored. "
        "Return error is the 3D displacement of this frame between the two stationary windows. "
        "It is not a point-cloud centroid.",
    ))
    lines.append(_labelled(
        "lidar_to_imu",
        frames.get("lidar_to_imu", ""),
        "This is the factory lidar-to-IMU extrinsic only. "
        "It is not the cart mount and it is not the floor-mark measurement. "
        "Do not edit it to shrink the endpoint error.",
    ))
    lines.append(_labelled(
        "cart_mount",
        frames.get("cart_mount", ""),
        "This is the rigid mount of the sensor on the cart. "
        "Keep it separate from the factory extrinsic. A display transform is not a calibration.",
    ))
    lines.append("")
    lines.append("## Wall patches")
    lines.append(
        "A patch box is in this session's map frame. It does not transfer to the next run. "
        "null means the box is not chosen yet."
    )
    for patch in spec.get("wall_patches") or []:
        lines.append("- {0}: {1}. {2}.".format(
            _one_line(patch.get("name", "")),
            _one_line(patch.get("description", "")),
            _box_text(patch.get("box_map_frame")),
        ))
    lines.append("")
    lines.append("## Stationary windows and return error")
    lines.append(
        "Start window: the first {0} s of the run. End window: the last {1} s of the run. "
        "Both lengths are fixed in this spec before the walk. "
        "Scoring requires disjoint windows, so both lengths fit in the shortest run. "
        "The 5 s minimum is provisional.".format(
            windows.get("start"), windows.get("end")
        )
    )
    lines.append(RETURN_ERROR_SENTENCE)
    lines.append(
        "Use the pose nearest the middle of each window. Do not slide either window after seeing the error. "
        "Also report the horizontal component, the vertical component, and the heading change in degrees. "
        "The metre pass uses the 3D distance only."
    )
    lines.append("")
    lines.append("## Acceptance")
    lines.append(
        "Return error threshold: {0} m in 3D. The pass requires the distance to be below this value.".format(
            _metres(acceptance.get("return_error_threshold_m"))
        )
    )
    lines.append(
        "Placement tolerance: {0} m. A residual equal to the tolerance is within it. A residual above it is not. "
        "A null residual is not measured, is not zero, and is not within tolerance. "
        "heading_check must be ok. off or not_checked is not within tolerance. "
        "This tolerance is provisional.".format(
            _metres(acceptance.get("placement_tolerance_m"))
        )
    )
    lines.append(
        "Wall p95 gate: {0} m. Revisit separation gate: {1} m. Both are provisional.".format(
            _metres(acceptance.get("wall_p95_gate_m")),
            _metres(acceptance.get("revisit_separation_gate_m")),
        )
    )
    lines.append(
        "Runs required: {0}. State: {1}.".format(
            acceptance.get("runs_required"), acceptance.get("state")
        )
    )
    lines.append("")
    lines.append("## Forbidden")
    for item in spec.get("forbidden") or []:
        lines.append("- {0}".format(_one_line(item)))
    lines.append("")
    lines.append(
        "This page does not set physical_accuracy_evaluated. That flag may become true only when a "
        "valid physical_reference.json for the same run exists. Consumers must not treat a null residual "
        "or a heading check other than ok as a pass."
    )
    wrapped = []
    for line in lines:
        wrapped.extend(_wrap_line(line))
    lines = wrapped
    if len(lines) > MAX_RENDER_LINES:
        raise RenderError(
            "rendered spec is {0} lines; the one-page limit is {1}".format(len(lines), MAX_RENDER_LINES)
        )
    return "\n".join(lines) + "\n"
