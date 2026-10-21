"""Offline parked-push-return diagnostics; never launches a replay.

Run ``python3 -m spatio.preflight --help``. Parked intervals are operator labels
in absolute trajectory/header seconds, not intervals inferred from the holds.
This is a diagnostic packet, not physical accuracy or milestone acceptance.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import json
import shlex
import sys

import numpy as np

from spatio.bagtiming.core import analyze, get, header_ns
from spatio.bagtiming.reader import BagReader, resolve_bags
from spatio.io.pcd import read_pcd
from spatio.manifest.jsonio import write_json_atomic
from spatio.walkcheck import defaults, holds
from spatio.walkcheck.report import plain
from spatio.walkcheck.trajectory import load_tum, score_trajectory

SETTINGS = Path(__file__).parent / "walkday" / "walkday.conf"
SETTING_ARGS = {
    "STATIONARY_WINDOW_S": "window_s", "MAX_GAP_S": "max_gap_s",
    "MIN_PATH_LENGTH_M": "min_path_length_m", "MIN_EXCURSION_M": "min_excursion_m",
    "MAX_ENDPOINT_SPREAD_M": "max_endpoint_spread_m",
    "MAX_ENDPOINT_SPEED_M_S": "max_endpoint_speed_m_s",
    "MAX_HOLD_SPEED_M_S": "max_hold_speed_m_s", "MIN_HOLD_S": "min_hold_s",
}


def read_settings(path, hold_speed=None):
    """Read numeric assignments only. Never source/execute a shell config."""
    options = {}
    for line in Path(path).read_text().splitlines():
        tokens = shlex.split(line, comments=True)
        if not tokens:
            continue
        key, sep, value = tokens[0].partition("=")
        if key not in SETTING_ARGS:
            continue
        if not sep or len(tokens) != 1 or key in options:
            raise ValueError(f"invalid or duplicate setting: {key}")
        options[key] = float(value)
    missing = SETTING_ARGS.keys() - options.keys()
    if missing:
        raise ValueError("missing settings: " + ", ".join(sorted(missing)))
    kwargs = {SETTING_ARGS[k]: v for k, v in options.items()}
    if hold_speed is not None:
        kwargs["max_hold_speed_m_s"] = hold_speed
    # Reuse the scorer's threshold validation even when replay is absent.
    score_trajectory({"ok": False}, **kwargs)
    return kwargs


def unavailable(reason):
    return {"status": "not_measured", "reason": reason}


def spread(xyz):
    radius = np.linalg.norm(xyz - xyz.mean(axis=0), axis=1)
    return {"samples": len(xyz), "mean_xyz_m": xyz.mean(axis=0),
            "rms_radius_m": float(np.sqrt(np.mean(radius ** 2))),
            "p95_radius_m": float(np.percentile(radius, 95)),
            "max_radius_m": float(radius.max())}


def scan_points(msg, kind):
    if kind == "livox_ros_driver2/msg/CustomMsg":
        for point in get(msg, "points", []):
            yield [get(point, axis, float("nan")) for axis in "xyz"]


def scan_centroid(msg, kind):
    xyz = np.asarray(list(scan_points(msg, kind)), dtype=float).reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz.mean(axis=0) if len(xyz) else None


def replay_result(session, recording, trajectory_error):
    """Inspect existing replay.sh artifacts, preserving failures and absence."""
    result = {"status": "not_available", "executed_by_preflight": False,
              "session": str(session), "exit_code": None}
    try:
        result["exit_code"] = int((session / "logs/container_exit_code").read_text())
    except FileNotFoundError:
        result["reason"] = "saved replay exit status missing; artifacts alone do not prove success"
        return result
    except (OSError, ValueError) as exc:
        result.update(status="invalid", reason=str(exc))
        return result
    if result["exit_code"] != 0:
        result.update(status="failed", reason="saved replay returned nonzero")
        return result
    try:
        manifest_path = session / "logs/replay_manifest.json"
        if not manifest_path.exists():
            manifest_path = session / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        params = json.loads((session / "logs/run_params.json").read_text())
        if not isinstance(manifest, dict) or not isinstance(params, dict):
            raise ValueError("manifest and run_params must be JSON objects")
        source = manifest.get("source_recording")
        if not isinstance(source, str) or not Path(source).is_absolute():
            raise ValueError("manifest source_recording must be an absolute local path")
        if resolve_bags(source) != resolve_bags(recording):
            raise ValueError("replay source_recording does not match the inspected bag files")
        if type(params.get("container_exit")) is not int or params["container_exit"] != 0:
            raise ValueError("run_params container_exit disagrees with saved exit status")
        duration = params.get("duration_s", "unknown")
        if (type(duration) not in (str, int, float)
                or isinstance(duration, float) and not math.isfinite(duration)):
            raise ValueError("run_params duration_s must be a string or finite number")
        if trajectory_error:
            raise ValueError(trajectory_error)
        columns = read_pcd(str(session / "map.pcd"))
        xyz = np.column_stack([columns[a] for a in "xyz"])
        if not len(xyz) or not np.isfinite(xyz).all():
            raise ValueError("replay map is empty or contains non-finite points")
        result["map_points"] = len(xyz)
        result["mode"] = params.get("mode")
        result["requested_duration_s"] = duration
        if params.get("mode") == "replay":
            result["status"] = "succeeded"
        elif params.get("mode") == "historical":
            result.update(status="historical", reason="historical extraction is not a new replay")
        else:
            raise ValueError("unknown replay mode")
    except (OSError, ValueError, KeyError) as exc:
        result.update(status="invalid", reason=str(exc))
    return result


def score(recording, session, parked, *, lidar_topic, settings=SETTINGS, hold_speed=None):
    if (len(parked) != 2 or any(not math.isfinite(v) for pair in parked for v in pair)
            or any(a >= b for a, b in parked) or parked[0][1] >= parked[1][0]):
        raise ValueError("supply two finite, ordered, disjoint parked intervals START END")
    kwargs = read_settings(settings, hold_speed)
    session = Path(session).resolve()
    reader = BagReader(recording)
    centroids, scan_times, missing_xyz, frames = [], [], 0, set()

    def records():
        nonlocal missing_xyz
        for topic, receipt, msg in reader.records():
            if topic == lidar_topic:
                stamp = header_ns(msg)
                kind = reader.topic_info[topic]["type"]
                frames.add(str(get(get(msg, "header"), "frame_id", "")))
                if stamp is None:
                    missing_xyz += 1
                elif any(start <= stamp / 1e9 <= end for start, end in parked):
                    center = scan_centroid(msg, kind)
                    if center is None:
                        missing_xyz += 1
                    else:
                        scan_times.append(stamp / 1e9)
                        centroids.append(center)
                else:
                    # Preserve the whole-bag missing-data count, but stop at
                    # the first usable point outside the labeled windows.
                    if not any(all(math.isfinite(v) for v in point)
                               for point in scan_points(msg, kind)):
                        missing_xyz += 1
            yield topic, receipt, msg

    timing = analyze(records(), reader.topic_info)
    loaded = load_tum(session / "trajectory.tum")
    error = loaded.get("detail") if not loaded["ok"] else None
    if loaded["ok"]:
        t, xyz, q = loaded["t"], loaded["xyz"], loaded["q"]
        norms = np.linalg.norm(q, axis=1)
        if not all(np.isfinite(a).all() for a in (t, xyz, q)):
            error = "non-finite trajectory"
        elif len(t) < defaults.MIN_POSES or np.any(np.diff(t) <= 0):
            error = "too few poses or non-increasing trajectory timestamps"
        elif np.any((norms < .5) | (norms > 2)):
            error = "invalid trajectory quaternion"
    trajectory = score_trajectory(loaded, **kwargs) if not error else None
    detected = (trajectory or {}).get("holds")
    segments = []
    for name, (start, end) in zip(("start", "end"), parked):
        segment = {"name": name, "t_start_s": start, "t_end_s": end}
        segment["pose_jitter"] = unavailable(error or "parked interval lacks trajectory coverage")
        segment["hold"] = unavailable(error or "parked interval lacks trajectory coverage")
        if not error:
            mask = (t >= start) & (t <= end)
            covered = (t[0] <= start and t[-1] >= end
                       and np.count_nonzero(mask) >= defaults.MIN_WINDOW_SAMPLES)
            if covered:
                covered = (t[mask][0] - start <= kwargs["max_gap_s"]
                           and end - t[mask][-1] <= kwargs["max_gap_s"])
            if covered:
                segment["hold"] = unavailable("hold detection unavailable")
                times, points, rotations = t[mask], xyz[mask], q[mask]
                if np.any(np.diff(times) > kwargs["max_gap_s"]):
                    segment["pose_jitter"] = unavailable("trajectory gap inside parked interval")
                else:
                    rotations = rotations / np.linalg.norm(rotations, axis=1)[:, None]
                    speeds = np.linalg.norm(np.diff(points, axis=0), axis=1) / np.diff(times)
                    segment["pose_jitter"] = dict(spread(points), status="measured",
                        first_to_last_m=float(np.linalg.norm(points[-1] - points[0])),
                        step_speed_p95_m_s=float(np.percentile(speeds, 95)),
                        step_speed_max_m_s=float(speeds.max()),
                        max_rotation_from_first_deg=float(holds.angular_distance_deg(
                            rotations, rotations[0]).max()))
                if detected is not None:
                    overlap = [max(0., min(end, h["t_end"]) - max(start, h["t_start"]))
                               for h in detected["intervals"]]
                    duration = max(overlap, default=0.)
                    segment["hold"] = {"status": "evaluated", "longest_overlap_s": duration,
                        "fires": duration >= max(kwargs["min_hold_s"], kwargs["window_s"])}
        selection = [c for stamp, c in zip(scan_times, centroids) if start <= stamp <= end]
        segment["scan_centroid_spread"] = (
            dict(spread(np.asarray(selection)), status="measured")
            if len(selection) >= 2 and len(frames) == 1 else
            unavailable("fewer than two CustomMsg scans with XYZ/header, or changing lidar frame"))
        segments.append(segment)
    result = {
        "schema": "spatio.preflight/1", "physical_accuracy_evaluated": False,
        "recording": str(Path(recording).resolve()), "lidar_topic": lidar_topic,
        "settings": {"path": str(Path(settings).resolve()), "trajectory_options": kwargs,
            "effective_hold_speed_m_s": min(kwargs["max_hold_speed_m_s"], kwargs["max_endpoint_speed_m_s"]),
            "hold_speed_source": "CLI override" if hold_speed is not None else "settings file",
            "bare_walkcheck_hold_speed_m_s": holds.MAX_HOLD_SPEED_M_S,
            "approval": "hold and endpoint settings remain provisional; no values changed"},
        "timing": timing, "replay": replay_result(session, recording, error),
        "trajectory_reasons": (trajectory or {}).get("result_reasons", [error]),
        "detected_holds": detected or unavailable(error or "trajectory validation prevented hold detection"),
        "parked": segments, "scans_without_xyz_or_header": missing_xyz,
        "limitations": [
            "Operator labels assert parked intervals and a shared trajectory/lidar header clock; no clock correction applied.",
            "Pose spread is estimator motion; raw scan-centroid spread includes scene sampling, not point accuracy.",
            "Scan-centroid spread requires decoded Livox CustomMsg XYZ; other types and timing-only decoder fallback are not measured.",
            "Hold detection uses trajectory only (no IMU quietness veto); timing still reports recorded IMU topics.",
            "Replay status is saved evidence with source-path matching, not a new execution or content-hash verification.",
            "No readiness or physical-accuracy verdict: no new jitter thresholds or placement evidence.",
        ],
    }
    if lidar_topic not in reader.topic_info:
        result["limitations"].append("Selected Mid-360 topic was not observed: " + lidar_topic)
    # Keep bagtiming's sub-microsecond offsets intact; pose presentation uses
    # walkcheck's existing conversion/rounding helper.
    output = plain(result)
    output["timing"] = timing
    return output


def summary(report):
    lines = ["Preflight diagnostics (physical accuracy not evaluated)",
             "Replay: " + report["replay"]["status"] + "; " + report["replay"].get("reason", "saved artifacts checked"),
             f"Hold speed used: {report['settings']['effective_hold_speed_m_s']:g} m/s "
             f"({report['settings']['hold_speed_source']}; bare walkcheck default "
             f"{report['settings']['bare_walkcheck_hold_speed_m_s']:g} m/s)"]
    for topic in report["timing"]["topics"]:
        r, h = topic["receipt"], topic["header"]
        lines.append(f"{topic['name']} [{topic['type']}]: n={topic['count']}; "
            f"receipt/header Hz={r['rate_hz']}/{h['rate_hz']}, "
            f"max gap s={r['max_gap_s']}/{h['max_gap_s']}, "
            f"backwards={r['backward_jumps']}/{h['backward_jumps']}, "
            f"duplicates={r['duplicate_intervals']}/{h['duplicate_intervals']}; "
            f"header-receipt median={topic['header_minus_receipt_s']['median']} s")
    for segment in report["parked"]:
        pose, points, hold = (segment[k] for k in ("pose_jitter", "scan_centroid_spread", "hold"))
        lines.append(f"{segment['name']} parked: pose max radius="
            f"{pose.get('max_radius_m', pose.get('reason'))} m; centroid max radius="
            f"{points.get('max_radius_m', points.get('reason'))} m; "
            f"hold={hold.get('fires', hold.get('reason'))}")
    lines.append("Scan-centroid spread includes scene sampling; intervals/clock are operator supplied. See JSON for limitations.")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", help="local MCAP file or recording/bag directory")
    parser.add_argument("--session", required=True, help="existing replay session (never executed)")
    parser.add_argument("--lidar-topic", required=True, help="operator-identified Mid-360 topic")
    parser.add_argument("--parked", nargs=2, type=float, action="append", required=True,
                        metavar=("START", "END"), help="repeat twice; absolute trajectory/header seconds")
    parser.add_argument("--settings", type=Path, default=SETTINGS)
    parser.add_argument("--max-hold-speed", type=float, help="explicit override in m/s; reported in JSON")
    parser.add_argument("--json", required=True, type=Path, help="output JSON path (parent must exist)")
    args = parser.parse_args(argv)
    try:
        report = score(args.recording, args.session, args.parked, lidar_topic=args.lidar_topic,
                       settings=args.settings, hold_speed=args.max_hold_speed)
        write_json_atomic(args.json, report)
    except Exception as exc:
        # MCAP/decoder failures can use exception types outside ValueError.
        # Keep invalid input on the documented CLI exit path, as bagtiming does.
        print(f"preflight: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(summary(report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
