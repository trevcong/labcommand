"""Measure named wall patches. No downsampling before the residual stats.

separation_m is the gap between two parallel surfaces inside the patch.
map.pcd has no per-point time or visit id, so the gap is not a revisit.
"""

from __future__ import annotations

import datetime
import os

import numpy as np

from spatio.wallpatch.doubled import (
    CONFIDENT_SEPARATION_M,
    MIN_PEAK_MASS,
    PEAK_MERGE_M,
    RULE_TEXT,
    UNCERTAIN_SEPARATION_M,
    VALLEY_HEIGHT_RATIO,
    analyze_distances,
    reconcile_axes,
)
from spatio.wallpatch.errors import MapShaMismatch, WallpatchError
from spatio.wallpatch.figures import write_note_figure, write_patch_figure
from spatio.wallpatch.geometry import (
    MAX_HINT_ANGLE_DEG,
    angle_deg,
    box_axes,
    crop_mask,
    fit_plane,
    parse_box,
    project,
)
from spatio.wallpatch.pcd import iter_xyz, sha256_file
from spatio.wallpatch.util import write_json

# Provisional: approved for now, revisit after a taped walk.
# The 0.05 m numbers are the workplan gates. They did not change.
P95_GATE_M = 0.05
SEPARATION_GATE_M = 0.05
GATE_LABEL = "provisional (approved for now, revisit after real data)"
MIN_POINTS = 200
REVISIT_NOTE = (
    "map.pcd has no per-point time or visit id. "
    "separation_m is the distance between two parallel surfaces found inside the patch. "
    "It cannot be attributed to a specific visit or revisit."
)
REPORT_SCHEMA = "spatio.wallpatch_report/1"
PATCH_SCHEMA = "spatio.wall_patches/1"


def measure_map(map_path, patches_path, out_dir, accuracy_label=None, verbose=False):
    """Crop each box, fit a plane, and write wallpatch.json, wallpatch.md, and PNGs.

    Refuses a patch file whose map_sha256 is not the sha256 of this map file.
    """
    map_path = os.path.abspath(map_path)
    patches_path = os.path.abspath(patches_path)
    out_dir = os.path.abspath(out_dir)
    if not os.path.isfile(map_path):
        raise WallpatchError("map not found: %s" % map_path)
    if not os.path.isfile(patches_path):
        raise WallpatchError("patches file not found: %s" % patches_path)
    document = _load_patches(patches_path)
    actual = sha256_file(map_path)
    expected = document["map_sha256"].lower()
    if actual != expected:
        raise MapShaMismatch(expected, actual)

    specs = []
    names = []
    invalid = []
    for index, patch in enumerate(document["patches"]):
        name, spec, reason = _parse_patch(patch, index)
        names.append(name)
        specs.append(spec)
        invalid.append(reason)
    cropped, n_map = _crop_all(map_path, specs)

    measured = []
    for name, points, spec, reason in zip(names, cropped, specs, invalid):
        measured.append(_measure_one(name, points, spec, reason))

    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    used_names = set()
    for patch in measured:
        filename = _figure_name(patch["name"], used_names)
        patch["figure"] = filename
        figure_path = os.path.join(out_dir, filename)
        _write_figure(figure_path, patch)
        patch.pop("_plot", None)

    report = {
        "schema": REPORT_SCHEMA,
        "map_path": map_path,
        "map_sha256": actual,
        "n_points_in_map": int(n_map),
        "patches_file": patches_path,
        "created_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "revisit_note": REVISIT_NOTE,
        "accuracy_label": accuracy_label if accuracy_label else None,
        "physical_accuracy_evaluated": False if accuracy_label else None,
        "downsampled_before_measure": False,
        "gates": {
            "p95_point_to_plane_max_m": P95_GATE_M,
            "revisit_separation_max_m": SEPARATION_GATE_M,
            "label": GATE_LABEL,
        },
        "doubled_wall_rule": {
            "text": RULE_TEXT,
            "bin_width_m": 0.002,
            "min_mass_fraction": MIN_PEAK_MASS,
            "valley_to_peak_max": VALLEY_HEIGHT_RATIO,
            "peak_merge_m": PEAK_MERGE_M,
            "confident_separation_m": CONFIDENT_SEPARATION_M,
            "undetermined_separation_m": UNCERTAIN_SEPARATION_M,
        },
        "patches": measured,
    }
    write_json(os.path.join(out_dir, "wallpatch.json"), report)
    with open(os.path.join(out_dir, "wallpatch.md"), "w", encoding="utf-8") as handle:
        handle.write(_markdown(report))
    if verbose:
        _print_summary(report, out_dir)
    return report


def _load_patches(path):
    import json

    try:
        with open(path, "r", encoding="utf-8") as handle:
            document = json.load(handle)
    except json.JSONDecodeError as exc:
        raise WallpatchError("patches file is not valid JSON: %s" % exc)
    if not isinstance(document, dict):
        raise WallpatchError("patches file must be a JSON object")
    if document.get("schema") != PATCH_SCHEMA:
        raise WallpatchError("patches file schema must be %s" % PATCH_SCHEMA)
    sha = document.get("map_sha256")
    if not isinstance(sha, str) or len(sha) != 64:
        raise WallpatchError("patches file map_sha256 must be 64 hex characters")
    try:
        int(sha, 16)
    except ValueError:
        raise WallpatchError("patches file map_sha256 must be hexadecimal")
    patches = document.get("patches")
    if not isinstance(patches, list) or len(patches) == 0:
        raise WallpatchError("patches file has no patches")
    return document


def _parse_patch(patch, index):
    if not isinstance(patch, dict):
        return "patch-%d" % (index + 1), None, "patch entry is not an object"
    name = patch.get("name")
    if not isinstance(name, str) or not name.strip():
        name = "patch-%d" % (index + 1)
    else:
        name = name.strip()
    box = patch.get("box_map_frame", None)
    if box is None:
        return name, None, "box not chosen"
    try:
        spec = parse_box(box)
    except WallpatchError as exc:
        return name, None, "invalid box: %s" % exc
    return name, spec, None


def _crop_all(map_path, specs):
    """One pass over the map. Each array is every finite point inside that box."""
    buckets = [[] for _ in specs]
    n_map = 0
    for pts in iter_xyz(map_path):
        n_map += int(pts.shape[0])
        finite = np.isfinite(pts).all(axis=1)
        good = pts[finite]
        if good.shape[0] == 0:
            continue
        for index, spec in enumerate(specs):
            if spec is None:
                continue
            mask = crop_mask(good, spec[0], spec[1], spec[2])
            if np.any(mask):
                buckets[index].append(good[mask])
    arrays = []
    for parts in buckets:
        if parts:
            arrays.append(np.concatenate(parts, axis=0))
        else:
            arrays.append(np.zeros((0, 3), dtype=np.float64))
    return arrays, n_map


def _measure_one(name, points, spec, invalid_reason):
    n_cropped = int(len(points))
    if invalid_reason is not None:
        return _blank(name, n_cropped, invalid_reason, spec)
    if n_cropped == 0:
        return _blank(name, 0, "empty box", spec)
    if n_cropped < MIN_POINTS:
        return _blank(
            name,
            n_cropped,
            "fewer than 200 points (%d)" % n_cropped,
            spec,
        )
    _center, hint, _half = spec
    fitted = fit_plane(points, hint)
    if fitted is None:
        return _blank(name, n_cropped, "plane fit failed", spec)
    if not fitted["orientation_supported"]:
        # A normal past 45 degrees from the hint is the thin axis of the
        # patch, not the wall. Scoring it reports the wrong residual and
        # can pass a doubled wall. Leave both gates unevaluated.
        return _blank(
            name,
            n_cropped,
            "fitted plane is %.2f deg from normal_hint "
            "(limit %.0f deg); wall orientation is not supported"
            % (fitted["angle_from_hint_deg"], MAX_HINT_ANGLE_DEG),
            spec,
        )
    normal = fitted["normal"]
    plane_point = fitted["point"]
    # Residuals use every cropped point. The inlier set is only the fit support.
    # Separation is not taken from that normal alone. A few degrees of pooled
    # tilt, from sheets that only partly overlap, merges the two peaks.
    signed = project(points - plane_point, normal)
    fit_stats = analyze_distances(signed)
    hint_unit = np.asarray(hint, dtype=np.float64)
    hint_unit = hint_unit / np.linalg.norm(hint_unit)
    hint_signed = project(points - points.mean(axis=0), hint_unit)
    hint_stats = analyze_distances(hint_signed)
    if int(fit_stats["n"]) != n_cropped or int(hint_stats["n"]) != n_cropped:
        raise WallpatchError("internal error: measured count != cropped count")
    decision = reconcile_axes(fit_stats, hint_stats, SEPARATION_GATE_M)
    if decision["kind"] != "measured":
        return _blank(name, n_cropped, decision["reason"], spec)
    stats = decision["stats"]
    doubled = stats["doubled"]
    separation = stats["separation_m"]
    separation_axis = "normal_hint" if stats is hint_stats else "fitted_plane"
    # p95 stays the point-to-plane residual. The hint axis is only the gap.
    p95_pass = bool(fit_stats["p95_abs_m"] <= P95_GATE_M)
    if doubled == "false":
        sep_pass = True
    elif doubled == "true":
        sep_pass = bool(separation is not None and separation <= SEPARATION_GATE_M)
    else:
        sep_pass = None
    across, vertical, mask = _slice_frame(points, plane_point, normal)
    if separation_axis == "normal_hint":
        # The histogram is the hint gap. The scatter uses that same axis.
        across = hint_signed
    return {
        "name": name,
        "status": "MEASURED",
        "reasons": [],
        "n_cropped": n_cropped,
        "n_measured": n_cropped,
        "downsampled_before_measure": False,
        "box_map_frame": _box_dict(spec),
        "plane": {
            "normal": _vec(normal),
            "point_m": _vec(plane_point),
            "angle_from_hint_deg": angle_deg(normal, hint),
            "n_inliers": int(fitted["n_inliers"]),
            "rms_inlier_m": float(fitted["rms_inlier_m"]),
            "eigenvalue_ratio": float(fitted["eigenvalue_ratio"]),
        },
        "residuals": {
            "p95_abs_m": fit_stats["p95_abs_m"],
            "thickness_m": fit_stats["thickness_m"],
            "p2_5_m": fit_stats["p2_5_m"],
            "p97_5_m": fit_stats["p97_5_m"],
            "bin_width_m": fit_stats["bin_width_m"],
        },
        "doubled": doubled,
        "separation_m": separation,
        "separation_axis": separation_axis,
        "gate_p95_pass": p95_pass,
        "gate_separation_pass": sep_pass,
        "figure": None,
        "_plot": {
            "hist_counts": stats["hist_counts"],
            "hist_edges": stats["hist_edges"],
            "mark_m": stats["mark_m"],
            "across": across,
            "vertical": vertical,
            "mask": mask,
        },
    }


def _blank(name, n_cropped, reason, spec):
    return {
        "name": name,
        "status": "NOT_EVALUABLE",
        "reasons": [reason],
        "n_cropped": int(n_cropped),
        "n_measured": None,
        "downsampled_before_measure": False,
        "box_map_frame": _box_dict(spec) if spec is not None else None,
        "plane": None,
        "residuals": None,
        "doubled": None,
        "separation_m": None,
        "separation_axis": None,
        "gate_p95_pass": None,
        "gate_separation_pass": None,
        "figure": None,
        "_plot": None,
    }


def _slice_frame(points, plane_point, normal):
    """Across (signed distance) and vertical coordinates, plus a thin along-wall slab."""
    along_axis, _across_axis, vertical_axis = box_axes(normal)
    delta = points - plane_point
    along = project(delta, along_axis)
    vertical = project(delta, vertical_axis)
    signed = project(delta, normal)
    half = 0.05
    mask = np.abs(along) <= half
    if int(np.count_nonzero(mask)) < 40:
        mask = np.abs(along) <= 0.15
    if int(np.count_nonzero(mask)) < 40:
        mask = np.ones(len(points), dtype=bool)
    return signed, vertical, mask


def _box_dict(spec):
    center, hint, half = spec
    unit = hint / np.linalg.norm(hint)
    return {
        "center_m": _vec(center),
        "normal_hint": _vec(unit),
        "half_extent_m": _vec(half),
    }


def _vec(value):
    return [float(item) for item in np.asarray(value, dtype=np.float64).reshape(-1)]


def _figure_name(name, used):
    chars = []
    for ch in str(name).strip():
        if ch.isalnum() or ch in "-_":
            chars.append(ch)
        else:
            chars.append("_")
    base = "".join(chars).strip("_") or "unnamed"
    while "__" in base:
        base = base.replace("__", "_")
    filename = "patch_%s.png" % base
    suffix = 2
    while filename in used:
        filename = "patch_%s_%d.png" % (base, suffix)
        suffix += 1
    used.add(filename)
    return filename


def _write_figure(path, patch):
    plot = patch.get("_plot")
    if plot is None:
        reason = "; ".join(patch["reasons"]) if patch["reasons"] else patch["status"]
        write_note_figure(path, patch["name"], patch["status"], reason)
        return
    write_patch_figure(
        path,
        patch["name"],
        patch["status"],
        plot["hist_counts"],
        plot["hist_edges"],
        plot["mark_m"],
        plot["across"],
        plot["vertical"],
        plot["mask"],
    )


def _flag(value):
    if value is True:
        return "pass"
    if value is False:
        return "fail"
    return "not evaluable"


def _metres(value):
    if value is None:
        return "null"
    return "%.4f m (%.2f cm)" % (value, value * 100.0)


def _markdown(report):
    gates = report["gates"]
    lines = [
        "# Wall patch report",
        "",
        report["revisit_note"],
        "",
        'Gates (labelled "%s"): 95th-percentile point-to-plane residual '
        "<= %.2f m, and separation between the two surfaces <= %.2f m."
        % (
            gates["label"],
            gates["p95_point_to_plane_max_m"],
            gates["revisit_separation_max_m"],
        ),
        "",
        "Doubled-wall rule: %s" % report["doubled_wall_rule"]["text"],
        "",
        "Map: `%s`" % report["map_path"],
        "",
        "Map sha256: `%s`" % report["map_sha256"],
        "",
        "Points in the map: %d" % report["n_points_in_map"],
        "",
        "Measured before cosmetic downsampling: yes. "
        "The plane is fit on inliers. p95, thickness, and the doubled-wall "
        "test use every cropped point.",
        "",
    ]
    if report.get("accuracy_label"):
        lines.append("Accuracy label: %s" % report["accuracy_label"])
        lines.append("")
        lines.append("physical_accuracy_evaluated: false")
        lines.append("")
    for patch in report["patches"]:
        lines.append("## %s" % patch["name"])
        lines.append("")
        lines.append("- Status: %s" % patch["status"])
        if patch["reasons"]:
            lines.append("- Reasons: %s" % "; ".join(patch["reasons"]))
        lines.append("- Points cropped: %s" % patch["n_cropped"])
        lines.append("- Points measured: %s" % patch["n_measured"])
        plane = patch["plane"]
        if plane is not None:
            lines.append("- Fitted normal: %s" % plane["normal"])
            lines.append(
                "- Angle from normal_hint: %.2f deg" % plane["angle_from_hint_deg"]
            )
            lines.append("- Inliers used for the fit: %d" % plane["n_inliers"])
        residuals = patch["residuals"]
        if residuals is not None:
            lines.append("- p95 |residual|: %s" % _metres(residuals["p95_abs_m"]))
            lines.append(
                "- Thickness (p97.5 - p2.5): %s" % _metres(residuals["thickness_m"])
            )
        lines.append("- Doubled: %s" % patch["doubled"])
        lines.append("- Separation: %s" % _metres(patch["separation_m"]))
        axis = patch["separation_axis"]
        lines.append("- Separation axis: %s" % (axis if axis else "null"))
        lines.append("- Gate p95: %s" % _flag(patch["gate_p95_pass"]))
        lines.append("- Gate separation: %s" % _flag(patch["gate_separation_pass"]))
        lines.append("- Figure: `%s`" % patch["figure"])
        lines.append("")
    return "\n".join(lines)


def _print_summary(report, out_dir):
    print("map_sha256 %s" % report["map_sha256"])
    for patch in report["patches"]:
        residuals = patch["residuals"] or {}
        print(
            "%s %s n=%s p95=%s doubled=%s separation=%s"
            % (
                patch["name"],
                patch["status"],
                patch["n_measured"],
                residuals.get("p95_abs_m"),
                patch["doubled"],
                patch["separation_m"],
            )
        )
    print("wrote %s" % out_dir)
