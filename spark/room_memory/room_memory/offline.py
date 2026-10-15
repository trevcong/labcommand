"""Offline runner: ``python -m room_memory.offline [flags] <before.bin> <after.bin>``.

By default it prints what ``swift run change-detect --evidence --json`` prints on the same
inputs: the canonical v2 record plus one newline, byte for byte. Every ``change-detect``
flag is accepted (``--evidence`` and ``--json`` are then no-ops); ``--no-evidence`` and
``--text`` select the Swift defaults instead. Inputs are CDR ``sensor_msgs/PointCloud2``
blobs (``.bin``).
"""

import math
import pathlib
import sys
import time
from fractions import Fraction

import numpy as np

from .core import ChangeDetectionError, Config, PointCloudError, compare, decode_pointcloud2, to_json

USAGE = """\
python -m room_memory.offline [--voxel-size m] [--max-range m] [--min-points n]
                              [--tolerance n] [--min-voxels n] [--min-extent n]
                              [--max-move m] [--no-pair] [--evidence | --no-evidence]
                              [--origin x,y,z]
                              [--ray-stride n] [--unobserved] [--json | --text]
                              <before.bin> <after.bin>
evidence and JSON are on by default (change-detect --evidence --json)
"""


class UsageError(Exception):
    """Unknown flag, missing value, or wrong number of paths."""


def parse_f32(text):
    """Parses a decimal to the nearest binary32 (ties to even), as Swift ``Float(text)`` does.

    Rounding through binary64 first could land one ulp off on a decimal near a binary32
    midpoint; this compares the exact rational value against both neighbors instead.

    Raises:
        ValueError: When ``text`` is not a number.
    """
    value = float(text)
    with np.errstate(over="ignore"):
        nearest = np.float32(value)
    if not math.isfinite(value) or not np.isfinite(nearest):
        return nearest
    exact = Fraction(text.strip())
    best = nearest
    for neighbor in (np.nextafter(nearest, np.float32(-np.inf)), np.nextafter(nearest, np.float32(np.inf))):
        if not np.isfinite(neighbor):
            continue
        gap = abs(Fraction(float(neighbor)) - exact)
        best_gap = abs(Fraction(float(best)) - exact)
        if gap < best_gap or (gap == best_gap and int(neighbor.view(np.uint32)) & 1 == 0):
            best = neighbor
    return best


def parse_args(args, evidence=False, emit_json=False):
    """Parses ``change-detect`` arguments into ``(config, emit_json, before, after)``.

    ``evidence`` and ``emit_json`` are the values before any flag; the defaults are those of
    ``change-detect``, and ``main`` turns both on.

    Raises:
        UsageError: On an unknown flag, a missing or malformed value, or not exactly two paths.
    """
    fields = {"evidence_of_absence": evidence}
    paths = []
    index = 0

    def value():
        nonlocal index
        if index + 1 >= len(args) or args[index + 1].startswith("-"):
            raise UsageError(arg)
        index += 1
        return args[index]

    def number(parse):
        try:
            return parse(value())
        except ValueError:
            raise UsageError(arg) from None

    while index < len(args):
        arg = args[index]
        if arg == "--voxel-size":
            fields["voxel_size"] = float(number(parse_f32))
        elif arg == "--max-range":
            fields["max_range"] = float(number(parse_f32))
        elif arg == "--min-points":
            fields["min_points"] = number(int)
        elif arg == "--tolerance":
            fields["tolerance"] = number(int)
        elif arg == "--min-voxels":
            fields["min_voxels"] = number(int)
        elif arg == "--min-extent":
            fields["min_extent"] = number(int)
        elif arg == "--max-move":
            fields["max_move"] = float(number(parse_f32))
        elif arg == "--no-pair":
            fields["pair_moves"] = False
        elif arg == "--evidence":
            fields["evidence_of_absence"] = True
        elif arg == "--no-evidence":
            fields["evidence_of_absence"] = False
        elif arg == "--origin":
            if index + 1 >= len(args) or args[index + 1].startswith("--"):
                raise UsageError(arg)
            index += 1
            parts = args[index].split(",")
            try:
                if len(parts) != 3:
                    raise ValueError(args[index])
                fields["origin"] = tuple(float(parse_f32(part)) for part in parts)
            except ValueError:
                raise UsageError(arg) from None
        elif arg == "--ray-stride":
            fields["ray_stride"] = number(int)
        elif arg == "--unobserved":
            fields["report_unobserved"] = True
        elif arg == "--json":
            emit_json = True
        elif arg == "--text":
            emit_json = False
        elif arg == "--":
            paths.extend(args[index + 1:])
            index = len(args)
        elif arg.startswith("-"):
            raise UsageError(arg)
        else:
            paths.append(arg)
        index += 1
    if len(paths) != 2:
        raise UsageError("expected two paths")
    return Config(**fields), emit_json, paths[0], paths[1]


def _load(path):
    suffix = pathlib.Path(path).suffix.lower()
    if suffix != ".bin":
        raise PointCloudError(f"expected a .bin file, got '{suffix or path}'")
    return decode_pointcloud2(pathlib.Path(path).read_bytes()).positions


def _point(values):
    return ",".join(f"{float(value):.3f}" for value in values)


def _bounds(low, high):
    return f"[{_point(low)} .. {_point(high)}]"


def _text(config, before_path, after_path, before_points, after_points, before, after, report, elapsed_ms):
    lines = [
        f"before: {pathlib.Path(before_path).name}  points={len(before_points)}  "
        f"voxels={len(before.occupied)}  dropped={before.dropped_points}",
        f"after:  {pathlib.Path(after_path).name}  points={len(after_points)}  "
        f"voxels={len(after.occupied)}  dropped={after.dropped_points}",
        f"voxel_size_m={float(config.voxel_size_f32):.3f}  tolerance={config.tolerance}  "
        f"min_points={config.min_points}  min_voxels={config.min_voxels}  min_extent={config.min_extent}  "
        f"pair_moves={'true' if config.pair_moves else 'false'}",
    ]
    counts = f"appeared_voxels={report.appeared_voxels}  disappeared_voxels={report.disappeared_voxels}"
    if config.evidence_of_absence:
        counts += f"  unknown_voxels={report.unknown_voxels}"
    lines += [counts, f"elapsed_ms={elapsed_ms:.1f}", f"regions={len(report.regions)}"]
    for index, region in enumerate(report.regions):
        line = (
            f"region {index} {region.kind} voxels={region.voxel_count} volume_m3={float(region.volume):.6f} "
            f"centroid={_point(region.centroid)} bounds={_bounds(region.bounds_min, region.bounds_max)}"
        )
        if region.kind == "moved":
            line += (
                f" from_voxels={region.from_voxel_count} from_centroid={_point(region.from_centroid)}"
                f" from_bounds={_bounds(region.from_bounds_min, region.from_bounds_max)}"
            )
        lines.append(line)
    return "\n".join(lines) + "\n"


def main(argv=None):
    """Runs the CLI. Returns the process exit code: 0 on a successful compare, 1 otherwise."""
    args = sys.argv[1:] if argv is None else argv
    try:
        config, emit_json, before_path, after_path = parse_args(args, evidence=True, emit_json=True)
    except UsageError:
        sys.stderr.write(USAGE)
        return 1
    try:
        before_points = _load(before_path)
        after_points = _load(after_path)
        start = time.perf_counter()
        before, after, report = compare(before_points, after_points, config)
        elapsed_ms = (time.perf_counter() - start) * 1000
    except (OSError, PointCloudError, ChangeDetectionError) as error:
        sys.stderr.write(f"room_memory.offline: {error}\n")
        return 1
    if emit_json:
        sys.stdout.write(to_json(report, before, after, config) + "\n")
    else:
        sys.stdout.write(_text(config, before_path, after_path, before_points, after_points, before, after, report, elapsed_ms))
    return 0


if __name__ == "__main__":
    sys.exit(main())
