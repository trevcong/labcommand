"""Read-only, ROS-free JT128 bag characterization; units are never inferred as facts."""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict, deque
import hashlib
from importlib.metadata import version
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

VENDOR = Path(__file__).parent / "vendor/jt128_slam_adapter.py"
EXPECTED_FIELDS = [
    {"name": name, "offset": offset, "datatype": dtype, "count": 1}
    for name, offset, dtype in [("x", 0, 7), ("y", 4, 7), ("z", 8, 7),
                                ("intensity", 12, 7), ("time", 16, 7), ("ring", 20, 4)]
]
TOPICS = ("/slam/points", "/lidar_imu", "/slam/imu")


def distribution(values):
    a = np.asarray(values, dtype=np.float64)
    good = a[np.isfinite(a)]
    out = {"count": int(a.size), "nonfinite": int(a.size - good.size)}
    if good.size:
        out.update(zip(("min", "p05", "median", "p95", "max"),
                       map(float, np.percentile(good, [0, 5, 50, 95, 100]))))
        out["mean"] = float(good.mean())
    return out


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def vector(v):
    return [float(v.x), float(v.y), float(v.z)]


def pair_imu(raw, adapted, tolerance_ns=1_000_000):
    """One-to-one, globally exact-first, then greedy nearest; never reuse a raw sample."""
    by_stamp = defaultdict(deque)
    for i, row in enumerate(raw):
        by_stamp[row[0]].append(i)
    pairs, pending, used = [], [], set()
    for j, row in enumerate(adapted):
        queue = by_stamp[row[0]]
        if queue:
            i = queue.popleft()
            pairs.append((i, j, "exact"))
            used.add(i)
        else:
            pending.append(j)
    available = sorted((r[0], i) for i, r in enumerate(raw) if i not in used)
    for j in sorted(pending, key=lambda k: (adapted[k][0], k)):
        t = adapted[j][0]
        k = bisect_left(available, (t, -1))
        candidates = [n for n in (k - 1, k) if 0 <= n < len(available)]
        if not candidates:
            continue
        n = min(candidates, key=lambda n: (abs(available[n][0] - t), available[n]))
        if abs(available[n][0] - t) <= tolerance_ns:
            _, i = available.pop(n)
            pairs.append((i, j, "nearest"))
            used.add(i)
    return pairs


def axis_analysis(raw_values, adapted_values, expected_scale):
    if not raw_values:
        return {"pairs": 0, "axes": {}, "axis_mapping": "not determinable: no pairs"}
    a, b = np.asarray(raw_values), np.asarray(adapted_values)
    axes = {}
    for k, axis in enumerate("xyz"):
        valid = np.isfinite(a[:, k]) & np.isfinite(b[:, k])
        nonzero = valid & (a[:, k] != 0)
        residual = b[:, k] - a[:, k] * expected_scale
        axes[axis] = {"ratio_slam_over_raw": distribution(b[nonzero, k] / a[nonzero, k]),
                      "zero_raw_count": int(np.sum(a[:, k] == 0)),
                      "nonfinite_pair_count": int(np.sum(~valid)),
                      "expected_scale": expected_scale,
                      "max_abs_expected_residual": float(np.max(np.abs(residual[valid]))) if valid.any() else None}
    finite = np.all(np.isfinite(a) & np.isfinite(b), axis=1)
    candidates = []
    if finite.any():
        for perm in itertools.permutations(range(3)):
            for signs in itertools.product((-1, 1), repeat=3):
                err = a[finite][:, perm] * signs * expected_scale - b[finite]
                candidates.append((float(np.sqrt(np.mean(err * err))), perm, signs))
    candidates.sort()
    return {"pairs": len(a), "axes": axes,
            "axis_mapping": {"best_rmse": candidates[0][0],
                             "raw_axis_for_slam_xyz": ["xyz"[k] for k in candidates[0][1]],
                             "signs": list(candidates[0][2]),
                             "runner_up_rmse": candidates[1][0],
                             "unique_best": candidates[1][0] > candidates[0][0]} if candidates else "no finite pairs"}


def imu_report(raw, adapted):
    pairs = pair_imu(raw, adapted)
    used_raw, used_adapted = {p[0] for p in pairs}, {p[1] for p in pairs}
    def unmatched(rows, used, other):
        stamps = [r[0] for i, r in enumerate(rows) if i not in used]
        lo, hi = (min(r[0] for r in other), max(r[0] for r in other)) if other else (None, None)
        return {"count": len(stamps), "stamp_ns": stamps,
                "before_other_range": sum(t < lo for t in stamps) if other else None,
                "after_other_range": sum(t > hi for t in stamps) if other else None,
                "inside_other_range": sum(lo <= t <= hi for t in stamps) if other else None}
    return {"raw_count": len(raw), "adapted_count": len(adapted),
            "count_difference_adapted_minus_raw": len(adapted) - len(raw),
            "matched": len(pairs), "exact": sum(p[2] == "exact" for p in pairs),
            "nearest_within_1ms": sum(p[2] == "nearest" for p in pairs),
            "raw_match_rate": len(pairs) / len(raw) if raw else None,
            "adapted_match_rate": len(pairs) / len(adapted) if adapted else None,
            "raw_duplicate_stamps": len(raw) - len({r[0] for r in raw}),
            "adapted_duplicate_stamps": len(adapted) - len({r[0] for r in adapted}),
            "raw_unmatched": unmatched(raw, used_raw, adapted),
            "adapted_unmatched": unmatched(adapted, used_adapted, raw),
            "stamp_delta_slam_minus_raw_ns": distribution([adapted[j][0] - raw[i][0] for i, j, _ in pairs]),
            "matched_frame_pairs": dict(Counter(raw[i][3] + " -> " + adapted[j][3] for i, j, _ in pairs)),
            "gyro": axis_analysis([raw[i][1] for i, _, _ in pairs], [adapted[j][1] for _, j, _ in pairs], math.pi / 180),
            "accel": axis_analysis([raw[i][2] for i, _, _ in pairs], [adapted[j][2] for _, j, _ in pairs], 1.0),
            "count_explanation": "Matched pairs are one-to-one. Unmatched and duplicate stamps locate the count imbalance; recorder loss, startup/shutdown subscription timing, or a different publisher cannot be distinguished from this bag alone."}


def cloud_arrays(msg):
    fields = [{k: getattr(f, k) for k in ("name", "offset", "datatype", "count")} for f in msg.fields]
    by_name = {f["name"]: f for f in fields}
    types = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    data = bytes(msg.data)
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step or len(data) < msg.row_step * msg.height:
        raise ValueError("invalid PointCloud2 dimensions or truncated payload")
    arrays = {}
    for name in ("time", "ring"):
        if name not in by_name or by_name[name]["count"] != 1:
            raise ValueError("missing or nonscalar PointCloud2 field " + name)
        f = by_name[name]
        dtype = np.dtype((">" if msg.is_bigendian else "<") + types[f["datatype"]])
        if f["offset"] < 0 or f["offset"] + dtype.itemsize > msg.point_step:
            raise ValueError("PointCloud2 field outside point_step: " + name)
        arrays[name] = (np.ndarray((msg.height, msg.width), dtype=dtype, buffer=data,
                                  offset=f["offset"], strides=(msg.row_step, msg.point_step)).reshape(-1)
                        if msg.height * msg.width else np.array([], dtype=dtype))
    return fields, arrays


def verdicts(points, imu, raw_points_present=False):
    rows = []
    def add(prop, status, evidence):
        rows.append({"property": prop, "verdict": status, "evidence": evidence})
    consistent = "consistent with vendored source"
    unknown = "not determinable from this bag"
    inconsistent = "inconsistent (evidence)"
    layouts = points["layouts"]
    ok = bool(layouts) and all(x["fields"] == EXPECTED_FIELDS and x["point_step"] == 24 and
                               not x["is_bigendian"] and x["is_dense"] for x in layouts)
    add("field layout", (consistent if ok else inconsistent) if layouts else unknown,
        "{} layout(s); source declares x/y/z/intensity/time float32 at 0/4/8/12/16, ring uint16 at 20, step 24, dense=true, little endian.".format(len(layouts)))
    scan = points["scans"]
    contradiction = any(s["first_time_s"] != 0 or (s["time_min_s"] is not None and s["time_min_s"] < 0) for s in scan if s["points"])
    add("time origin", inconsistent if contradiction else unknown,
        "{}/{} nonempty scans start at zero; source uses first kept point, not header. Raw input timestamp/scan start absent; zero alone cannot prove origin.".format(points["starts_at_zero_count"], points["nonempty_scan_count"]))
    add("time unit", unknown, "Values span {} (nominal seconds under source interpretation); magnitude alone cannot establish units or firing-time semantics without raw points.".format(points["scan_duration_s"]))
    add("stride", unknown, "No raw-to-adapted point mapping is analyzed. Raw /lidar_points {}. Synthetic stride 4 keeps indices 0,4,8,...; recorded point count alone cannot prove stride.".format("is present" if raw_points_present else "is absent"))
    add("ring handling", unknown, "Observed {} distinct rings, range {}. Source copies selected ring uint16; raw input is needed to prove preservation or identify dropped rings.".format(len(points["ring_histogram"]), points["ring_range"]))
    for prop, key in (("gyro conversion", "gyro"), ("accel handling", "accel")):
        axes = imu[key]["axes"]
        finite = axes and all(x["max_abs_expected_residual"] is not None and x["nonfinite_pair_count"] == 0 for x in axes.values())
        ok = finite and all(x["max_abs_expected_residual"] <= 1e-12 for x in axes.values())
        add(prop, (consistent if ok else inconsistent) if axes else unknown,
            "{} matched pairs; per-axis ratios/residuals {}. Physical input units remain unverified; expected scale {}.".format(imu["matched"], axes, math.pi / 180 if key == "gyro" else 1))
    add("header stamp source", unknown,
        "IMU: {} exact matches, {} nearest, delta ns {}; source copies header. Points: header-minus-receipt seconds {}; raw cloud header absent. Receipt time is not acquisition/firing time.".format(imu["exact"], imu["nearest_within_1ms"], imu["stamp_delta_slam_minus_raw_ns"], points["header_minus_receipt_s"]))
    return rows


def inspect_bag(recording, max_seconds=None):
    recording = Path(recording)
    files = [recording] if recording.is_file() else sorted((recording / "bag").glob("*.mcap"))
    if not files and recording.is_dir():
        files = sorted(recording.glob("*.mcap"))
    if not files:
        raise ValueError("no MCAP files found in recording or recording/bag")
    if max_seconds is not None and (not math.isfinite(max_seconds) or max_seconds <= 0):
        raise ValueError("max-seconds must be finite and positive")
    inventory, starts, sources = Counter(), [], []
    for path in files:
        with path.open("rb") as stream:
            summary = make_reader(stream).get_summary()
            if summary is None or summary.statistics is None:
                raise ValueError("MCAP summary/statistics required for bounded receipt-time window")
            starts.append(summary.statistics.message_start_time)
            for cid, count in summary.statistics.channel_message_counts.items():
                inventory[summary.channels[cid].topic] += count
        sources.append({"path": str(path.resolve()), "size_bytes": path.stat().st_size})
    start = min(starts)
    end = start + int(max_seconds * 1e9) if max_seconds is not None else None
    raw, adapted, scans, layouts = [], [], [], {}
    rings, frames, counts = Counter(), Counter(), Counter()
    for path in files:
        for record in read_ros2_messages(path, topics=TOPICS, start_time=start, end_time=end, log_time_order=False):
            msg, topic = record.ros_msg, record.channel.topic
            counts[topic] += 1
            if topic != "/slam/points":
                row = (stamp_ns(msg), vector(msg.angular_velocity), vector(msg.linear_acceleration), msg.header.frame_id)
                (raw if topic == "/lidar_imu" else adapted).append(row)
                continue
            fields, arrays = cloud_arrays(msg)
            layout = {"fields": fields, "point_step": msg.point_step, "is_dense": msg.is_dense,
                      "is_bigendian": msg.is_bigendian}
            key = json.dumps(layout, sort_keys=True)
            if key not in layouts:
                layouts[key] = dict(layout, messages=0)
            layouts[key]["messages"] += 1
            t, r = arrays["time"], arrays["ring"]
            unique, number = np.unique(r, return_counts=True)
            rings.update({str(int(v)): int(n) for v, n in zip(unique, number)})
            finite = bool(np.isfinite(t).all())
            def value(v):
                return float(v) if np.isfinite(v) else None
            scans.append({"header_stamp_ns": stamp_ns(msg), "receipt_ns": record.log_time_ns,
                          "frame_id": msg.header.frame_id, "points": int(t.size),
                          "first_time_s": value(t[0]) if t.size else None,
                          "last_time_s": value(t[-1]) if t.size else None,
                          "time_min_s": value(t.min()) if t.size else None,
                          "time_max_s": value(t.max()) if t.size else None,
                          "last_minus_first_s": value(float(t[-1]) - float(t[0])) if t.size else None,
                          "range_duration_s": value(float(t.max()) - float(t.min())) if t.size else None,
                          "nonfinite_times": int(np.sum(~np.isfinite(t))),
                          "nondecreasing": bool(t.size and finite and np.all(t[1:] >= t[:-1])),
                          "height": msg.height, "row_step": msg.row_step})
            frames[msg.header.frame_id] += 1
    nonempty = [s for s in scans if s["points"]]
    zeros = sum(s["first_time_s"] == 0 for s in nonempty)
    monotonic = sum(s["nondecreasing"] for s in nonempty)
    both = sum(s["first_time_s"] == 0 and s["nondecreasing"] for s in nonempty)
    points = {"messages": len(scans), "nonempty_scan_count": len(nonempty), "layouts": list(layouts.values()),
              "points_per_message": distribution([s["points"] for s in scans]),
              "scan_duration_s": distribution([s["last_minus_first_s"] for s in nonempty]),
              "time_range_duration_s": distribution([s["range_duration_s"] for s in nonempty]),
              "starts_at_zero_count": zeros, "nondecreasing_count": monotonic,
              "starts_at_zero_fraction": zeros / len(nonempty) if nonempty else None,
              "nondecreasing_fraction": monotonic / len(nonempty) if nonempty else None,
              "starts_zero_and_nondecreasing_fraction": both / len(nonempty) if nonempty else None,
              "ring_histogram": dict(sorted(rings.items(), key=lambda x: int(x[0]))),
              "ring_range": [min(map(int, rings)), max(map(int, rings))] if rings else None,
              "all_128_rings_present": set(rings) == set(map(str, range(128))),
              "frame_ids": dict(frames),
              "header_minus_receipt_s": distribution([(s["header_stamp_ns"] - s["receipt_ns"]) / 1e9 for s in scans]),
              "header_step_s": distribution(np.diff(sorted(s["header_stamp_ns"] for s in scans)) / 1e9),
              "scans": scans}
    imu = imu_report(raw, adapted)
    return {"vendor_sha256": hashlib.sha256(VENDOR.read_bytes()).hexdigest(),
            "provenance": "not verified to be the version that produced the 20260826T221936Z bag",
            "reader": {"api": "mcap_ros2.reader.read_ros2_messages(log_time_order=False)",
                       "mcap": version("mcap"), "mcap_ros2": version("mcap-ros2-support"),
                       "python": sys.version, "ros_runtime": False},
            "sources": sources, "inventory_whole_bag": dict(inventory), "decoded_counts": dict(counts),
            "scope": {"max_seconds": max_seconds, "receipt_start_ns": start, "receipt_end_exclusive_ns": end,
                      "full_pass": max_seconds is None},
            "physical_accuracy_evaluated": False,
            "evidence_boundary": "Historical recording inspection only; no replay executed here; physical accuracy not evaluated. No raw-to-adapted point mapping or firing-time claim is established.",
            "points": points, "imu": imu,
            "verdicts": verdicts(points, imu, "/lidar_points" in inventory)}


def markdown(report):
    p, i = report["points"], report["imu"]
    lines = ["# JT128 adapter characterization", "", report["provenance"], "",
             report["evidence_boundary"], "", "Reader: `{}`.".format(report["reader"]),
             "Scope: `{}`.".format(report["scope"]), "", "## Consistency verdicts", "",
             "| Property | Verdict | Evidence |", "|---|---|---|"]
    for v in report["verdicts"]:
        lines.append("| {property} | {verdict} | {evidence} |".format(**{k: str(x).replace("|", "\\|").replace("\n", " ") for k, x in v.items()}))
    lines += ["", "## Point measurements", "", "Per-scan minima, maxima, durations, stamps and monotonicity are in adaptercheck.json.", ""]
    for key, value in p.items():
        if key != "scans":
            lines.append("- {}: `{}`".format(key, json.dumps(value, sort_keys=True)))
    lines += ["", "## IMU measurements", "", i["count_explanation"], ""]
    for key, value in i.items():
        if key == "count_explanation":
            continue
        if key.endswith("unmatched"):
            value = {k: v for k, v in value.items() if k != "stamp_ns"}
        lines.append("- {}: `{}`".format(key, json.dumps(value, sort_keys=True)))
    lines += ["", "The ratios test numerical conversion, not the physical calibration or raw units. Scan duration is last minus first relative time; the range duration is reported separately. Fractions use nonempty scans. Receipt timestamps are recorder times, never labelled firing times.", ""]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--max-seconds", type=float)
    args = parser.parse_args(argv)
    try:
        report = inspect_bag(args.recording, args.max_seconds)
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "adaptercheck.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        (args.out / "adaptercheck.md").write_text(markdown(report))
    except (OSError, ValueError) as e:
        parser.exit(2, "bagcheck: {}\n".format(e))
    print("Wrote {}: {} clouds, {} raw IMU, {} adapted IMU".format(args.out, report["points"]["messages"], report["imu"]["raw_count"], report["imu"]["adapted_count"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
