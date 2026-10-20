"""Score one session folder and write the three report files."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from spatio.walkcheck.defaults import (
    HISTORICAL_LABEL,
    MAX_GAP_S,
    MIN_EXCURSION_M,
    MIN_PATH_LENGTH_M,
    RETURN_PASS_M,
    SCHEMA,
    STATIONARY_WINDOW_S,
)
from spatio.walkcheck.pcd import PcdError, read_pcd_xyz
from spatio.walkcheck.plot import write_topdown
from spatio.walkcheck.report import plain, render_markdown, write_json
from spatio.walkcheck.trajectory import load_tum, score_stops, score_trajectory
from spatio.walkcheck.walls import wall_sharpness

ASSUMPTIONS = [
    "Horizontal is the XY distance and vertical is the absolute Z difference. "
    "This assumes the map frame's Z axis is up, which is the usual FAST-LIO "
    "gravity-aligned frame. It is an assumption, not a measurement from this file. "
    "The 3D error does not use that assumption.",
    "Heading is the yaw difference about Z. It is reported and it is not part of the pass rule.",
    "A path that does not move enough fails even when the start and end positions match.",
    "The wall number is a geometric estimate on vertical planar patches. "
    "Floors, ceilings, and clutter are left out. Two real parallel surfaces "
    "look the same as a doubled wall from drift.",
]


class SessionDirError(FileNotFoundError):
    """The session path is not a directory."""


def evaluate_session(
    session_dir: Path | str,
    *,
    stationary_window_s: float = STATIONARY_WINDOW_S,
    max_gap_s: float = MAX_GAP_S,
    min_path_length_m: float = MIN_PATH_LENGTH_M,
    min_excursion_m: float = MIN_EXCURSION_M,
    write: bool = True,
) -> dict:
    session = Path(session_dir)
    if not session.is_dir():
        raise SessionDirError(f"session directory not found: {session}")

    manifest, manifest_error = _read_json(session / "manifest.json")
    stops_list, stops_source, stops_error = _load_stops(session, manifest)
    tum_path = session / "trajectory.tum"
    map_path = session / "map.pcd"

    if tum_path.is_file():
        loaded = load_tum(tum_path)
    else:
        loaded = {
            "ok": False,
            "reason": "missing_trajectory",
            "detail": "trajectory.tum is missing",
            "comments": [],
            "t": np.zeros(0),
            "xyz": np.zeros((0, 3)),
            "q": np.zeros((0, 4)),
        }
    scored = score_trajectory(
        loaded,
        window_s=stationary_window_s,
        max_gap_s=max_gap_s,
        min_path_length_m=min_path_length_m,
        min_excursion_m=min_excursion_m,
    )

    map_xyz = None
    if not map_path.is_file():
        walls = wall_sharpness(np.zeros((0, 3)))
        walls["reason"] = "missing_map"
        walls["status"] = "NOT-EVALUABLE"
        walls["proposed_gate_result"] = "NOT-EVALUABLE"
    else:
        try:
            map_xyz = read_pcd_xyz(map_path)
            walls = wall_sharpness(map_xyz)
        except (OSError, PcdError, ValueError) as exc:
            map_xyz = None
            walls = wall_sharpness(np.zeros((0, 3)))
            walls["reason"] = f"unreadable_map: {exc}"
            walls["status"] = "NOT-EVALUABLE"
            walls["proposed_gate_result"] = "NOT-EVALUABLE"

    if scored["evaluated"]:
        reasons = list(scored["result_reasons"])
        result = "FAIL" if reasons else "PASS"
    else:
        reasons = list(scored["result_reasons"])
        result = "NOT-EVALUABLE"

    stop_report = {
        "provided": bool(stops_list) or stops_error is not None,
        "source": stops_source,
        "stops": [],
        "error": stops_error,
    }
    if stops_list and scored["evaluated"]:
        stop_report["stops"] = score_stops(loaded["t"], loaded["xyz"], stops_list)
    elif stops_list:
        stop_report["stops"] = [
            {
                "name": str(item.get("name") or "unnamed") if isinstance(item, dict) else "unnamed",
                "visits": 0,
                "visits_used": 0,
                "max_mean_separation_cm": None,
                "notes": ["not scored because the trajectory is not evaluable"],
            }
            for item in stops_list
        ]

    physical_flag = None
    manifest_notes = None
    run_id = None
    sensor = None
    if isinstance(manifest, dict):
        physical_flag = manifest.get("physical_accuracy_evaluated")
        manifest_notes = manifest.get("notes")
        run_id = manifest.get("run_id")
        sensor = manifest.get("sensor")

    notes = list(ASSUMPTIONS)
    if physical_flag is not True:
        notes.insert(0, HISTORICAL_LABEL)
    else:
        notes.append(
            "The manifest says physical accuracy was evaluated. "
            "This tool did not re-check the floor measurement."
        )
    if any("SYNTHETIC" in comment.upper() for comment in scored["comments"]):
        notes.append("The trajectory file says it is synthetic.")
    if manifest_notes:
        notes.append(f"Manifest notes: {manifest_notes}")
    if manifest_error:
        notes.append(f"manifest.json could not be read: {manifest_error}")
    if stops_error:
        notes.append(f"Intermediate stops could not be read: {stops_error}")

    returned = scored["return_to_start"]
    title = result
    if returned:
        title = f"{result}   return {returned['error_3d_cm']:.1f} cm"
    start_xyz = None if not returned else returned["start_mean_xyz_m"]
    end_xyz = None if not returned else returned["end_mean_xyz_m"]
    traj_xyz = loaded["xyz"] if loaded.get("ok") else None
    traj_t = loaded["t"] if loaded.get("ok") else None

    report = {
        "schema": SCHEMA,
        "result": result,
        "reasons": reasons,
        "detail": scored.get("detail"),
        "session_dir": str(session.resolve()),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "parameters": {
            "stationary_window_s": stationary_window_s,
            "max_gap_s": max_gap_s,
            "return_pass_m": RETURN_PASS_M,
            "return_comparison": "strictly_below",
            "min_path_length_m": min_path_length_m,
            "min_excursion_m": min_excursion_m,
            "min_window_samples": 3,
            "vertical_axis": "z",
            "alignment": "none",
            "wall_voxel_m": walls.get("voxel_m"),
            "wall_gate_cm": walls.get("proposed_gate_cm"),
            "wall_gate_label": walls.get("proposed_gate_label"),
        },
        "inputs": {
            "trajectory": "trajectory.tum",
            "trajectory_present": tum_path.is_file(),
            "map": "map.pcd",
            "map_present": map_path.is_file(),
            "manifest_present": (session / "manifest.json").is_file(),
            "stops_present": (session / "stops.json").is_file(),
        },
        "manifest": {
            "present": isinstance(manifest, dict),
            "run_id": run_id,
            "sensor": sensor,
            "physical_accuracy_evaluated": physical_flag if isinstance(physical_flag, bool) else None,
            "notes": manifest_notes,
        },
        "trajectory": {
            "pose_count": scored["pose_count"],
            "comments": scored["comments"][:8],
        },
        "timestamps": scored["timestamps"],
        "return_to_start": returned,
        "motion": scored["motion"],
        "drift": {
            "path_length_m": None if not scored["motion"] else scored["motion"]["path_length_m"],
            "return_error_pct_of_path": None
            if not scored["motion"]
            else scored["motion"]["return_error_pct_of_path"],
        },
        "intermediate_stops": stop_report,
        "walls": walls,
        "outputs": {
            "json": "walkcheck.json",
            "markdown": "walkcheck.md",
            "topdown": "topdown.png",
        },
        "notes": notes,
    }
    report = plain(report)
    if write:
        write_json(session / "walkcheck.json", report)
        (session / "walkcheck.md").write_text(render_markdown(report), encoding="utf-8")
        write_topdown(
            session / "topdown.png",
            map_xyz,
            None if traj_xyz is None else np.asarray(traj_xyz),
            None if traj_t is None else np.asarray(traj_t),
            None if start_xyz is None else np.asarray(start_xyz, dtype=np.float64),
            None if end_xyz is None else np.asarray(end_xyz, dtype=np.float64),
            title,
        )
    return report


def _read_json(path: Path):
    if not path.is_file():
        return None, None
    try:
        return json.loads(path.read_text(encoding="utf-8")), None
    except (OSError, json.JSONDecodeError) as exc:
        return None, str(exc)


def _load_stops(session: Path, manifest):
    stops_path = session / "stops.json"
    if stops_path.is_file():
        payload, error = _read_json(stops_path)
        if error:
            return [], "stops.json", error
        return _stop_list(payload), "stops.json", None
    if isinstance(manifest, dict) and "intermediate_stops" in manifest:
        return _stop_list(manifest.get("intermediate_stops")), "manifest.json", None
    return [], None, None


def _stop_list(payload) -> list:
    if isinstance(payload, dict):
        payload = payload.get("stops") or []
    if not isinstance(payload, list):
        return []
    return payload
