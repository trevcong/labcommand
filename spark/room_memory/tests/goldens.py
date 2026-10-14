"""Scenes whose expected JSON was produced by the Swift ``change-detect`` CLI.

Each case is a builder returning ``(before, after)`` positions and the CLI flags. The
expected output lives in ``fixtures/swift/<name>.json``; ``make_swift_goldens.py`` writes
the scenes as PointCloud2 ``.bin`` files and regenerates those files with the Swift CLI.
Phase 0 cases read the committed synthetic fixtures of the Phase 0 branch.
"""

import json
import os
import pathlib

import numpy as np

import scenes

HERE = pathlib.Path(__file__).resolve().parent
SWIFT_DIR = HERE / "fixtures" / "swift"
PHASE0_DIR = pathlib.Path(
    os.environ.get("ROOM_MEMORY_PHASE0_FIXTURES", "~/Developer/labcommand-memory/Tests/Fixtures/room/synthetic")
).expanduser()

EVIDENCE = ["--evidence", "--json"]
# Column and single-cell fixtures are one cell thick; the extent gate is the extent scenes' subject.
THIN = ["--min-extent", "1"]
UNIT = ["--voxel-size", "1", "--origin", "0.5,0.5,0.5", "--evidence", "--tolerance", "0", "--min-voxels", "1",
        "--no-pair", "--unobserved", "--json", *THIN]


def _box_pair(before_center, after_center):
    room = scenes.room_shell()
    before = room if before_center is None else np.concatenate([room, scenes.box_shell(before_center)])
    after = room if after_center is None else np.concatenate([room, scenes.box_shell(after_center)])
    return before, after


def _moved_pair():
    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30))
    return np.concatenate([room, box]), np.concatenate([room, scenes.shifted(box, 1)])


def _jitter_pair():
    cloud = np.concatenate([scenes.room_shell(), scenes.box_shell((1.0, 0.5, 0.30))])
    return scenes.jittered(cloud, 0x9E3779B97F4A7C15), scenes.jittered(cloud, 0xD1B54A32D192ED03)


def _speck_pair():
    room = scenes.room_shell()
    speck = np.array([(-0.8, 1.8, 0.413), (-0.8, 1.8, 0.463)], dtype=np.float32)
    return room, np.concatenate([room, scenes.box_shell((1.0, 0.5, 0.30)), speck])


def _clear_line_doubled():
    before = scenes.column(list(range(20, 28)) + [40])
    after = scenes.column([40])
    return np.concatenate([before, before]), np.concatenate([after, after])


def _cells(*cells):
    return np.array([scenes.cell_point(*cell) for cell in cells], dtype=np.float32).reshape(-1, 3)


def _board_pair():
    before, after, _, _ = scenes.board_scene()
    return before, after


def _sheet_pair():
    """The Mid-360 static blip: 7 voxels in a 5 x 3 x 1 box, one cell thick."""
    return np.zeros((0, 3), dtype=np.float32), _cells(
        (0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0), (4, 0, 0), (1, 1, 0), (2, 2, 0)
    )


def _small_box_pair(before_center, after_center):
    room = scenes.room_shell()
    before = room if before_center is None else np.concatenate([room, scenes.box_shell(before_center, 0.10)])
    after = room if after_center is None else np.concatenate([room, scenes.box_shell(after_center, 0.10)])
    return before, after


def _small_moved_pair():
    room = scenes.room_shell()
    box = scenes.box_shell((0.0, 0.5, 0.30), 0.10)
    return np.concatenate([room, box]), np.concatenate([room, scenes.shifted(box, 1)])


def _unit_center(i, j, k):
    return (np.float32(i) + np.float32(0.5), np.float32(j) + np.float32(0.5), np.float32(k) + np.float32(0.5))


def boundary_pair(far):
    """Voxel size 1, sensor cell (0, 0, 0), return on the x and y faces at (16, -15, 0.5).

    The tie walks the diagonal i = -j. Unclamped it overshoots to (16, -16, 0) and on to the
    Manhattan cap; (15, 0, 0), (16, -1, 0), (16, -16, 0) and (21, -21, 0) are the cells that
    overshoot reaches or aliases, and they must stay unknown. ``far`` adds a shared far
    return so the Swift side uses its open-addressed walker instead of the bit grid.
    """
    endpoint = (np.float32(16), np.float32(-15), np.float32(0.5))
    before = [_unit_center(i, -i, 0) for i in range(16)]
    before += [_unit_center(15, 0, 0), _unit_center(16, -1, 0), _unit_center(16, -16, 0), _unit_center(21, -21, 0)]
    before.append(endpoint)
    after = [endpoint]
    if far:
        before.append(_unit_center(700, 700, 700))
        after.append(_unit_center(700, 700, 700))
    return np.array(before, dtype=np.float32), np.array(after, dtype=np.float32)


def _min_key_pair():
    after = [scenes.cell_point(i, 0, 0) for i in range(6)] + [scenes.cell_point(1, j, 0) for j in range(20, 26)]
    return np.zeros((0, 3), dtype=np.float32), np.array(after, dtype=np.float32)


SCENES = {
    "indoor-moved": (lambda: (scenes.indoor(False), scenes.indoor(True)), EVIDENCE),
    "box-added": (lambda: _box_pair(None, (1.0, 0.5, 0.30)), EVIDENCE),
    "box-removed": (lambda: _box_pair((1.0, 0.5, 0.30), None), EVIDENCE),
    "box-moved": (_moved_pair, EVIDENCE),
    "box-moved-v1": (_moved_pair, ["--json"]),
    "box-moved-unpaired-v1": (_moved_pair, ["--no-pair", "--json"]),
    "jitter": (_jitter_pair, EVIDENCE),
    "speck-min-voxels-1": (_speck_pair, ["--min-voxels", "1", "--evidence", "--json", *THIN]),
    "speck-min-voxels-1-extent-2": (_speck_pair, ["--min-voxels", "1", "--evidence", "--json"]),
    "sheet": (_sheet_pair, EVIDENCE),
    "sheet-min-extent-1": (_sheet_pair, ["--evidence", "--json", *THIN]),
    "shell-10cm-added": (lambda: _small_box_pair(None, (1.0, 0.5, 0.30)), EVIDENCE),
    "shell-10cm-removed": (lambda: _small_box_pair((1.0, 0.5, 0.30), None), EVIDENCE),
    "shell-10cm-moved": (_small_moved_pair, EVIDENCE),
    "boundary-endpoint": (lambda: boundary_pair(False), UNIT),
    "boundary-endpoint-far": (lambda: boundary_pair(True), UNIT),
    "board-occluded": (_board_pair, ["--evidence", "--unobserved", "--json"]),
    "occluded-column": (
        lambda: (scenes.column(list(range(8, 17)) + list(range(20, 28))), scenes.column([16])),
        ["--evidence", "--tolerance", "0", "--unobserved", "--json", *THIN],
    ),
    "clear-line-min-points-2": (
        _clear_line_doubled, ["--evidence", "--tolerance", "0", "--min-points", "2", "--json", *THIN],
    ),
    "diagonal-tie": (
        lambda: (_cells((0, 0, 0), (1, 1, 1), (2, 2, 2), (1, 0, 0), (3, 3, 3)), _cells((3, 3, 3))),
        ["--evidence", "--tolerance", "0", "--min-voxels", "1", "--unobserved", "--origin", "0.025,0.025,0.025", "--json",
         *THIN],
    ),
    "slide-toward": (lambda: (scenes.column([20]), scenes.column([19])), ["--evidence", "--min-voxels", "1", "--json"]),
    "slide-away": (lambda: (scenes.column([20]), scenes.column([21])), ["--evidence", "--min-voxels", "1", "--json"]),
    "wide-grid": (
        lambda: (_cells((10, 10, 10), (700, 700, 700)), _cells((700, 700, 700))),
        ["--evidence", "--tolerance", "0", "--min-voxels", "1", "--json", *THIN],
    ),
    "distance-tie": (
        lambda: (_cells((-20, 0, 0), (20, 0, 0)), _cells((0, 0, 0))),
        ["--tolerance", "0", "--min-voxels", "1", "--json", *THIN],
    ),
    "min-key-order": (_min_key_pair, ["--tolerance", "0", "--min-voxels", "1", "--no-pair", "--json", *THIN]),
    "long-ray-stride-16": (scenes.long_ray, ["--evidence", "--max-range", "15", "--ray-stride", "16", "--json"]),
    "long-ray-stride-64": (scenes.long_ray, ["--evidence", "--max-range", "15", "--ray-stride", "64", "--json"]),
}


def phase0_pairs():
    """``(name, before_file, after_file)`` for every Phase 0 synthetic pair, walkthrough expanded."""
    manifest = json.loads((PHASE0_DIR / "MANIFEST.json").read_text())
    pairs = []
    for pair in manifest["pairs"]:
        if "after_sequence" in pair:
            for index, after in enumerate(pair["after_sequence"]):
                pairs.append((f"phase0-{pair['name']}-{index}", pair["before"], after))
        else:
            pairs.append((f"phase0-{pair['name']}", pair["before"], pair["after"]))
    return pairs
