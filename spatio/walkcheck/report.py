"""JSON and Markdown writers. Numbers are rounded only after the verdict."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from spatio.walkcheck.defaults import HISTORICAL_LABEL, PROPOSED_WALL_LABEL


def plain(obj):
    """Convert numpy values to JSON values. Bool is checked before int."""
    if isinstance(obj, dict):
        return {str(key): plain(value) for key, value in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [plain(value) for value in obj]
    if isinstance(obj, np.ndarray):
        return plain(obj.tolist())
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        value = float(obj)
        if not math.isfinite(value):
            return None
        return round(value, 6)
    if isinstance(obj, int):
        return obj
    if obj is None or isinstance(obj, str):
        return obj
    return str(obj)


def write_json(path: Path, report: dict) -> None:
    path.write_text(json.dumps(plain(report), indent=2) + "\n", encoding="utf-8")


def render_markdown(report: dict) -> str:
    result = report["result"]
    lines = [
        f"# Walk check: {result}",
        "",
        f"Session: `{report['session_dir']}`",
        f"Scored: {report['created_at']}",
        "",
        "The end of the path was not moved onto the start. "
        "The return error is the gap between the mean position of a fixed "
        "start window and the mean position of a fixed end window.",
        "",
    ]
    if HISTORICAL_LABEL in report.get("notes", []):
        lines.append(HISTORICAL_LABEL)
        lines.append("")
    if report["reasons"]:
        lines.append("Reasons: " + ", ".join(report["reasons"]) + ".")
        lines.append("")

    returned = report.get("return_to_start")
    lines.append("## Return to start")
    lines.append("")
    if not returned:
        lines.append("The return error was not scored. " + (report.get("detail") or "The trajectory cannot be trusted."))
        lines.append("")
    else:
        start = returned["start_window_s"]
        end = returned["end_window_s"]
        sxyz = returned["start_mean_xyz_m"]
        exyz = returned["end_mean_xyz_m"]
        lines.append(
            f"Start window: {_sec(start[0])} s to {_sec(start[1])} s "
            f"({returned['start_samples']} poses), "
            f"mean position ({_m(sxyz[0])}, {_m(sxyz[1])}, {_m(sxyz[2])}) m."
        )
        lines.append(
            f"End window: {_sec(end[0])} s to {_sec(end[1])} s "
            f"({returned['end_samples']} poses), "
            f"mean position ({_m(exyz[0])}, {_m(exyz[1])}, {_m(exyz[2])}) m."
        )
        lines.append("")
        lines.append(f"- 3D error: {_cm(returned['error_3d_cm'])} cm")
        lines.append(f"- Horizontal (XY): {_cm(returned['error_horizontal_cm'])} cm")
        lines.append(f"- Vertical (|Z|): {_cm(returned['error_vertical_cm'])} cm")
        lines.append(f"- Heading error (yaw about Z): {_deg(returned['heading_error_deg'])} deg")
        lines.append(f"- Full attitude error: {_deg(returned['attitude_error_deg'])} deg")
        inside = "inside" if returned["within_threshold"] else "not inside"
        lines.append(f"- 3D threshold: below 10 cm. This measurement is {inside} that threshold.")
        lines.append("")
        if result == "FAIL" and returned["within_threshold"]:
            lines.append(
                "The 3D gap is under 10 cm. The result is still FAIL because the path "
                "did not move enough to count as a walk."
            )
            lines.append("")

    motion = report.get("motion")
    lines.append("## Motion")
    lines.append("")
    if not motion:
        lines.append("Path length was not used. The timestamps or the windows are not trustworthy.")
        lines.append("")
    else:
        share = motion["return_error_pct_of_path"]
        share_text = "n/a" if share is None else f"{share:.2f}%"
        lines.append(f"- Path length: {motion['path_length_m']:.2f} m")
        lines.append(f"- Farthest point from the start mean: {motion['excursion_m']:.2f} m")
        lines.append(f"- Return error as a share of the path: {share_text}")
        met = "meets" if motion["enough_motion"] else "does not meet"
        lines.append(
            f"- Motion rule: path at least {motion['min_path_length_m']:.1f} m and "
            f"excursion at least {motion['min_excursion_m']:.1f} m. This run {met} it."
        )
        lines.append("")

    stops = report.get("intermediate_stops") or {}
    if not stops.get("provided"):
        lines.append("No intermediate stops were provided.")
        lines.append("")
    else:
        lines.append(f"Intermediate stops from {stops.get('source')}:")
        lines.append("")
        for stop in stops.get("stops") or []:
            spread = stop["max_mean_separation_cm"]
            spread_text = "not compared" if spread is None else f"{_cm(spread)} cm between visit means"
            lines.append(
                f"- {stop['name']}: {stop['visits_used']} of {stop['visits']} visits used, {spread_text}."
            )
            for note in stop.get("notes") or []:
                lines.append(f"  - {note}")
        lines.append("")
        lines.append("Stop separation does not change the walk result.")
        lines.append("")

    stamps = report.get("timestamps")
    lines.append("## Timestamps")
    lines.append("")
    if not stamps or stamps.get("t_start") is None:
        lines.append("No timestamps were read.")
        lines.append("")
    else:
        lines.append(f"- Poses: {report['trajectory']['pose_count']}")
        duration = stamps.get("duration_s")
        lines.append(
            "- Duration: " + ("n/a" if duration is None else f"{duration:.3f} s")
        )
        if stamps.get("median_dt_s") is not None:
            lines.append(f"- Median step: {stamps['median_dt_s']:.3f} s")
            lines.append(f"- Largest step: {stamps['max_dt_s']:.3f} s")
        if stamps.get("backward_steps"):
            lines.append(
                f"- Backwards or repeated stamps: {stamps['backward_steps']}. "
                f"{stamps.get('fault') or ''}"
            )
        if stamps.get("gap_steps"):
            lines.append(
                f"- Steps over {report['parameters']['max_gap_s']:.3f} s: {stamps['gap_steps']}. "
                f"{stamps.get('fault') or ''}"
            )
        if not stamps.get("backward_steps") and not stamps.get("gap_steps") and stamps.get("median_dt_s") is not None:
            lines.append(
                f"- No backwards stamps and no step over {report['parameters']['max_gap_s']:.3f} s."
            )
        lines.append("")

    walls = report["walls"]
    lines.append("## Walls")
    lines.append("")
    if walls["status"] != "OK":
        lines.append(f"Wall score was not computed ({walls.get('reason') or 'unavailable'}).")
    else:
        lines.append(
            f"95th-percentile point-to-plane residual: {_cm(walls['p95_point_to_plane_cm'])} cm "
            f"(worse of two {walls['voxel_m']:.2f} m voxel grids)."
        )
        doubled = "yes" if walls["doubled_wall"] else "no"
        lines.append(f"Doubled-wall indicator: {doubled}.")
        if walls.get("revisit_separation_cm") is not None:
            lines.append(
                f"Separations over 5 cm: smallest {_cm(walls['revisit_separation_cm'])} cm, "
                f"largest {_cm(walls['max_revisit_separation_cm'])} cm, "
                f"{walls['parallel_pair_count']} detections across the two grids. "
                "A detection is one cell with two sheets, or one pair of neighboring cells. "
                "It is not a count of walls."
            )
        lines.append(
            f"Proposed gate: residual ≤ {walls['proposed_gate_cm']:.0f} cm and no doubled wall "
            f"over 5 cm. Result: {walls['proposed_gate_result']}."
        )
    lines.append(
        f"This gate is {PROPOSED_WALL_LABEL}. It does not change the walk result."
    )
    if walls.get("finite_point_count"):
        low = walls["bounds_min_m"]
        high = walls["bounds_max_m"]
        lines.append(
            f"Map points: {walls['finite_point_count']} finite of {walls['point_count']}. "
            f"Bounds x [{_m(low[0])}, {_m(high[0])}], "
            f"y [{_m(low[1])}, {_m(high[1])}], "
            f"z [{_m(low[2])}, {_m(high[2])}] m."
        )
    lines.append("")
    lines.append("## Notes")
    lines.append("")
    for note in report.get("notes") or []:
        lines.append(f"- {note}")
    lines.append("")
    return "\n".join(lines)


def write_markdown(path: Path, report: dict) -> None:
    path.write_text(render_markdown(report), encoding="utf-8")


def _sec(value: float) -> str:
    return f"{float(value):.3f}"


def _m(value: float) -> str:
    return f"{float(value):.3f}"


def _cm(value: float) -> str:
    return f"{float(value):.2f}"


def _deg(value: float) -> str:
    return f"{float(value):.2f}"
