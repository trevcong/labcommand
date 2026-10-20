import argparse
import json
from pathlib import Path
import sys

from .classify import classify
from .grid import EvidenceGrid
from .model import load_json, number, parse_scans, parse_zones
from .synthetic import scene


def report(zones_data, scans_data, voxel_size, grid_origin, uncertainty_m):
    uncertainty_m = number(uncertainty_m, "uncertainty_m")
    if uncertainty_m < 0:
        raise ValueError("uncertainty_m must be nonnegative")
    frame, zones = parse_zones(zones_data)
    scan_frame, scans = parse_scans(scans_data)
    if frame != scan_frame:
        raise ValueError("zone and scan frame_id must match")
    grid = EvidenceGrid(voxel_size, grid_origin)
    for scan in scans:
        grid.add_scan(scan)
    return {
        "schema_version": 1, "frame_id": frame,
        "claim": "Provisional voxel evidence only; does not certify compliance.",
        "physical_accuracy_evaluated": False,
        "voxel_size_m": grid.size, "grid_origin": grid.origin.tolist(),
        "uncertainty_m": uncertainty_m, "uncertainty_status": "PROVISIONAL",
        "scan_count": len(scans),
        "scan_time_range": [scans[0].timestamp, scans[-1].timestamp] if scans else None,
        "observed_cells": len(grid.cells),
        "conflicting_cells": [list(i) for i, c in sorted(grid.cells.items())
                              if c.free_times and c.occupied_times],
        "zones": [classify(z, grid, uncertainty_m) for z in zones],
    }


def markdown(result):
    def safe(value):
        return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")

    lines = ["# Clearance evidence", "", result["claim"],
             "Physical accuracy not evaluated. Rule provisions are supplied inputs.",
             f"Voxel size: {result['voxel_size_m']:g} m. "
             f"Provisional uncertainty: ±{result['uncertainty_m']:g} m.",
             f"Scans: {result['scan_count']}; times: {result['scan_time_range']}.",
             f"Conflicting cells (whole grid): {len(result['conflicting_cells'])}.", "",
             "| Zone / rule | Status | Min hit clearance (m) | Free / required | Occupied / unknown / conflicts |",
             "|---|---|---:|---:|---:|"]
    for z in result["zones"]:
        clearance = "not measured" if z["min_clearance_m"] is None else f"{z['min_clearance_m']:.6g}"
        lines.append(f"| {safe(z['id'])} / {safe(z['rule_id'])} | {z['status']} | {clearance} | "
                     f"{z['free_only_cells']} / {z['required_cells']} | "
                     f"{z['occupied_cells']} / {z['unknown_cells']} / {len(z['conflicting_cells'])} |")
    lines += ["", "Supplied provisions:"]
    lines += [f"- {safe(z['id'])}: {safe(z['provision'])}" for z in result["zones"]]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--synthetic", action="store_true", help="use EXAMPLE, NOT APPROVED scene")
    parser.add_argument("--zones", type=Path)
    parser.add_argument("--scans", type=Path)
    parser.add_argument("--voxel-size", type=float, required=True, help="metres")
    parser.add_argument("--grid-origin", type=float, nargs=3, default=[0, 0, 0])
    parser.add_argument("--uncertainty-m", type=float, required=True, help="provisional, not calibrated")
    parser.add_argument("--output", type=Path, required=True, help="new directory for JSON and Markdown")
    args = parser.parse_args(argv)
    if args.synthetic == bool(args.zones or args.scans) or (not args.synthetic and not (args.zones and args.scans)):
        parser.error("choose --synthetic OR both --zones and --scans")
    try:
        zones, scans = scene() if args.synthetic else (load_json(args.zones), load_json(args.scans))
        result = report(zones, scans, args.voxel_size, args.grid_origin, args.uncertainty_m)
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output / "report.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        (args.output / "report.md").write_text(markdown(result))
        if args.synthetic:
            (args.output / "zones.json").write_text(json.dumps(zones, indent=2) + "\n")
            (args.output / "scans.json").write_text(json.dumps(scans, indent=2) + "\n")
    except (ValueError, OSError, OverflowError) as error:
        parser.error(str(error))
    print(markdown(result), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
