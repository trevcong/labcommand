"""Write compare.json, compare.md, and a raw top-down overlay."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Union

import numpy as np

from spatio.replaycompare.compare import DIAGNOSTIC_LABEL

PathLike = Union[str, Path]


def write_outputs(directory: PathLike, result: dict, series: dict) -> None:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    payload = _jsonable(result)
    text = json.dumps(payload, indent=2, allow_nan=False) + "\n"
    (directory / "compare.json").write_text(text, encoding="utf-8")
    (directory / "compare.md").write_text(format_markdown(payload), encoding="utf-8")
    _write_plot(directory / "overlay.png", series)


def format_markdown(result: dict) -> str:
    """Markdown whose first numerical claim is the raw trajectory difference."""
    lines = []
    lines.append(
        f"# Replay comparison: {result['a']['run_id']} vs {result['b']['run_id']}"
    )
    lines.append("")
    lines.append(f"**Status:** {result['status']}")
    lines.append("")
    offset = result["time_offset_s"]
    if offset is None:
        lines.append("**Time offset:** none (no automatic time shift).")
    else:
        lines.append(
            "**Time offset:** explicit "
            f"{float(offset):.6g} s added to session B timestamps."
        )
    lines.append("")
    if result["reasons"]:
        lines.append("Reasons:")
        lines.append("")
        for reason in result["reasons"]:
            lines.append(f"- {reason}")
        lines.append("")

    raw = result["trajectory"]["raw"]
    position = raw["position_m"]
    lines.append("## Headline (raw frame, no alignment)")
    lines.append("")
    lines.append(
        "Primary numbers. Nothing was aligned. "
        "This is a diagnostic of replay agreement, not a physical acceptance result."
    )
    lines.append("")
    if position is None:
        association = result["trajectory"]["association"]
        lines.append(
            "**NOT_EVALUABLE.** No raw position or rotation difference was computed."
        )
        lines.append("")
        lines.append(
            f"Session A time range (s): {_format_range(association['a_range_s'])}"
        )
        lines.append(
            f"Session B time range (s): {_format_range(association['b_range_s'])}"
        )
        lines.append("")
    else:
        rotation = raw["rotation_deg"]
        lines.append(
            "Position difference (m): "
            f"mean {position['mean']:.6f}, median {position['median']:.6f}, "
            f"p95 {position['p95']:.6f}, max {position['max']:.6f}, "
            f"final {position['final']:.6f}."
        )
        lines.append("")
        lines.append(
            "Rotation difference (deg): "
            f"median {rotation['median']:.6f}, p95 {rotation['p95']:.6f}, "
            f"max {rotation['max']:.6f}."
        )
        lines.append("")

    lines.append("## Each trajectory")
    lines.append("")
    lines.append(_pose_line("A", raw["first_pose"]["a"]))
    lines.append(_pose_line("B", raw["first_pose"]["b"]))
    lines.append(
        "Start-to-end distance (m): "
        f"A {_format_number(raw['start_to_end_m']['a'])}, "
        f"B {_format_number(raw['start_to_end_m']['b'])}."
    )
    lines.append(
        "Path length (m): "
        f"A {_format_number(raw['path_length_m']['a'])}, "
        f"B {_format_number(raw['path_length_m']['b'])}."
    )
    lines.append("")

    association = result["trajectory"]["association"]
    lines.append("## Association")
    lines.append("")
    lines.append(
        f"Matched {association['matched']}, "
        f"unmatched A {association['unmatched_a']}, "
        f"unmatched B {association['unmatched_b']}."
    )
    lines.append(
        f"Overlap duration (s): {association['overlap_duration_s']:.6f}. "
        f"Range gap (s): {_format_number(association['range_gap_s'])}. "
        f"max-dt (s): {association['max_dt_s']:.6f}."
    )
    lines.append(
        f"Session A: {association['a_pose_count']} poses, "
        f"duration {_format_number(association['a_duration_s'])} s, "
        f"rate {_format_number(association['a_rate_hz'])} Hz, "
        f"range {_format_range(association['a_range_s'])}."
    )
    lines.append(
        f"Session B: {association['b_pose_count']} poses, "
        f"duration {_format_number(association['b_duration_s'])} s, "
        f"rate {_format_number(association['b_rate_hz'])} Hz, "
        f"range {_format_range(association['b_range_s'])}."
    )
    if result["time_offset_s"] is not None:
        lines.append(
            "Session B range after the explicit offset (s): "
            f"{_format_range(association['b_range_compared_s'])}."
        )
    lines.append("")

    diagnostic = result["trajectory"]["diagnostic_aligned"]
    lines.append(f"## {DIAGNOSTIC_LABEL}")
    lines.append("")
    if diagnostic.get("computed"):
        lines.append(
            f"ATE RMSE (m): {diagnostic['ate_rmse_m']:.6f}. "
            f"{DIAGNOSTIC_LABEL}. Not the headline."
        )
        lines.append("")
        lines.append(
            "Umeyama SE(3), scale fixed at 1, aligns A onto B. "
            f"Translation (m): {_format_vector(diagnostic['translation_m'])}. "
            f"Rotation (deg): {diagnostic['rotation_deg']:.6f}."
        )
        lines.append("")
    else:
        lines.append(diagnostic.get("reason", "Not computed."))
        lines.append("")
        lines.append("No ATE number was computed.")
        lines.append("")

    lines.append("## Map nearest neighbours")
    lines.append("")
    map_report = result["map"]
    if map_report is None:
        lines.append("Map distances were not computed.")
        lines.append("")
    else:
        lines.append(
            f"Voxel {map_report['voxel_m']:.3f} m, centroid per voxel, "
            "used only for this comparison."
        )
        lines.append("")
        lines.append(
            f"Points A: {map_report['a_points_before']} before, "
            f"{map_report['a_points_after']} after. "
            f"Points B: {map_report['b_points_before']} before, "
            f"{map_report['b_points_after']} after."
        )
        lines.append(_distance_line("A to B", map_report["a_to_b"]))
        lines.append(_distance_line("B to A", map_report["b_to_a"]))
        lines.append(
            "Centroid offset, B minus A, before downsampling (m): "
            f"{_format_vector(map_report['centroid_offset_m'])} "
            f"(norm {map_report['centroid_offset_norm_m']:.6f})."
        )
        lines.append(
            "Bounding box A (m): "
            f"{_format_vector(map_report['bbox_a']['min'])} to "
            f"{_format_vector(map_report['bbox_a']['max'])}."
        )
        lines.append(
            "Bounding box B (m): "
            f"{_format_vector(map_report['bbox_b']['min'])} to "
            f"{_format_vector(map_report['bbox_b']['max'])}."
        )
        lines.append("")
    return "\n".join(lines)


def _write_plot(path: Path, series: dict) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(8, 8))
    try:
        colors = ("#1f4e79", "#b85c38")
        for color, side in zip(colors, ("a", "b")):
            track = series[side]
            x = np.asarray(track["x"], dtype=np.float64)
            y = np.asarray(track["y"], dtype=np.float64)
            label = f"{track['run_id']} ({side.upper()})"
            if len(x) >= 2:
                axis.plot(x, y, color=color, linewidth=1.5, label=label)
            elif len(x) == 1:
                axis.plot(x, y, color=color, marker="o", label=label)
            else:
                axis.plot([], [], color=color, label=f"{label} (no poses)")
                continue
            axis.scatter(
                [x[0]],
                [y[0]],
                s=46,
                marker="o",
                color=color,
                zorder=3,
                label=f"{track['run_id']} start",
            )
            axis.scatter(
                [x[-1]],
                [y[-1]],
                s=46,
                marker="s",
                color=color,
                zorder=3,
                label=f"{track['run_id']} end",
            )
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_title("Top-down trajectories (raw frames, no alignment)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best", fontsize=8)
        figure.tight_layout()
        figure.savefig(path, dpi=120)
    finally:
        plt.close(figure)


def _pose_line(label: str, pose: dict) -> str:
    if pose is None:
        return f"First pose {label}: none."
    return (
        f"First pose {label}: t {pose['t']:.6f} s, "
        f"xyz ({pose['x']:.6f}, {pose['y']:.6f}, {pose['z']:.6f}) m, "
        f"quat xyzw ({pose['qx']:.6f}, {pose['qy']:.6f}, {pose['qz']:.6f}, {pose['qw']:.6f})."
    )


def _distance_line(label: str, stats: dict) -> str:
    return (
        f"{label}: p50 {stats['p50_m']:.6f} m, p95 {stats['p95_m']:.6f} m, "
        f"p99 {stats['p99_m']:.6f} m, "
        f"fraction within 0.05 m {stats['fraction_within_0_05m']:.6f}, "
        f"fraction within 0.10 m {stats['fraction_within_0_10m']:.6f}."
    )


def _format_range(span) -> str:
    if span is None:
        return "n/a"
    return f"[{float(span[0]):.6f}, {float(span[1]):.6f}]"


def _format_number(value) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.6f}"


def _format_vector(values) -> str:
    return "(" + ", ".join(f"{float(value):.6f}" for value in values) + ")"


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.floating):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("non-finite number in report")
        return number
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("non-finite number in report")
        return value
    return value
