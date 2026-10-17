"""Rhythm log v1: minute bins, ``.rhy`` bytes, the writer, and ``RhythmMinute`` CDR.

The header + 60-window hex below is the shared pin (the C++ CTest uses the same bytes).
It is little endian. ``crc32(b"RHY1")`` is ``0x046E226A``.
"""

import ast
import dataclasses
import datetime
import errno
import math
import os
import pathlib
import re
import struct
import time
import zlib

import numpy as np
import pytest

import scenes
from room_memory.core.change_detection import occupancy_map, pack, splitmix64
from room_memory.memory import MemoryConfig, RoomMemory, reference_path
from room_memory.messages import CDRError, Invalid, RhythmMinute, Time
from room_memory.rhythm import (
    ENVELOPE_BYTES,
    FLAG_DISCONTINUITY,
    FLAG_LOST,
    FLAG_OVERFLOW,
    FLAG_PARTIAL,
    FLAG_UNAVAILABLE,
    MAX_RECORD_BYTES,
    QUEUE_SLOTS,
    RECOVERY_BYTES,
    REGION_SLOTS,
    Rhythm,
    RhythmAccumulator,
    _Table,
    encode_header,
    header_identity,
    history_depth,
    parse_record,
    rhythm_active,
    scan_records,
    validate_rhythm,
    voxels_per_cell,
    windows_per_period,
)

START = Time(1_700_000_000, 0)
GENERATION = 7
# Minute 60, epoch 4, room lab, Mid-360, dims 2, 10 voxels/cell, 0.05 m voxels, 0.50 m cells,
# period 60. Fifteen windows of 10 voxels in cell (0, 0) and 45 windows of one voxel in cell
# (1, 0): hit 15, mean 3, and hit 45, mean 1. pose_ok 0. No flags.
FILE_HEX = (
    "524859310100240001020a00cdcc4c3d0000003f3c00000003006c6162000000f427ca96"
    "3c0000003c0000000000000004000000000000003c003c000000000000000000000000000000"
    "02000000000000000f0003000100000000002d000100dcdc9597"
)
# Same minute on /room_memory/rhythm: seq 1, sensor 1, the two regions. LE then BE.
CDR_LE = (
    "00010000040000006c616200010000000000000004000000000000000100000000000000"
    "3c000000000000003c003c00000000000000000000000000000002000200000000000100"
    "02000000000000000200000000000000020000000f002d000200000003000100"
)
CDR_BE = (
    "00000000000000046c616200010000000000000000000000000000040000000000000001"
    "0000003c00000000003c003c000000000000000000000000000000020000000200000001"
    "0000000200000000000000020000000000000002000f002d0000000200030001"
)


def stamp(second):
    return Time(START.sec + second, 0)


def config(directory, **changes):
    values = dict(
        sensors=("mid360",), state_dir=None, voxel_size=0.05, window_s=1.0, room_id="lab",
        rhythm_enable=True, rhythm_publish=False, rhythm_dir=str(directory), rhythm_dims=2,
        rhythm_cell_m=0.5, rhythm_period_s=60, rhythm_keep_days=0, rhythm_max_bytes=0,
        rhythm_max_file_bytes=0, rhythm_fsync_s=0,
    )
    values.update(changes)
    return MemoryConfig(**values)


@pytest.fixture
def rhythm(tmp_path):
    opened = []

    def make(path=None, **changes):
        store = changes.pop("store", None)
        publish = changes.pop("publish", None)
        would_block = changes.pop("publish_would_block", None)
        directory = path if path is not None else tmp_path / f"r{len(opened)}"
        generation = changes.pop("publisher_generation", None)
        obj = Rhythm(
            config(directory, **changes), store=store, publish=publish, publish_would_block=would_block,
            publisher_generation=generation,
        )
        opened.append(obj)
        return obj

    yield make
    for obj in opened:
        obj.close()


def key(*index):
    return np.array([pack(*index)], dtype=np.int64)


def load_rhy(path):
    data = path.read_bytes()
    header = header_identity(data)
    assert header is not None
    records = []
    offset = header["header_bytes"]
    while offset + 4 <= len(data):
        body = struct.unpack_from("<I", data, offset)[0]
        end = offset + 4 + body
        if body < 40 or end > len(data):
            break
        record = parse_record(data[offset:end])
        if record is None:
            break
        records.append(record)
        offset = end
    return header, records, data, offset


def rhy_files(directory):
    root = directory / "lab" / "rhythm"
    return sorted(path for path in root.iterdir() if path.is_file()) if root.exists() else []


def utc_day(offset_days):
    day = datetime.datetime.now(datetime.timezone.utc).date() + datetime.timedelta(days=offset_days)
    return day.strftime("%Y%m%d")


def this_minute():
    now = int(time.time())
    return now - now % 60


class ScriptStore:
    """One write action per call, then real ``os.write``. Truncate can fail once."""

    def __init__(self, writes=()):
        self.writes = list(writes)
        self.fsyncs = 0
        self.truncations = []
        self.truncate_error = None

    def write(self, fd, data):
        if self.writes:
            return self.writes.pop(0)(fd, data)
        return os.write(fd, data)

    def fsync(self, fd):
        self.fsyncs += 1
        os.fsync(fd)

    def truncate(self, fd, length):
        if self.truncate_error is not None:
            error = self.truncate_error
            self.truncate_error = None
            raise error
        os.ftruncate(fd, length)
        self.truncations.append(length)


def short_write(count):
    def write(fd, data):
        os.write(fd, data[:count])
        return count
    return write


def full_write(fd, data):
    return os.write(fd, data)


# MARK: config and grid


def test_crc32_of_the_magic_is_the_pinned_value():
    assert zlib.crc32(b"RHY1") & 0xFFFFFFFF == 0x046E226A


def test_float32_ratio_period_and_file_cap_are_rejected_without_clamping():
    assert voxels_per_cell(0.5, 0.05) == 10
    assert voxels_per_cell(0.05, 0.05) == 1
    assert voxels_per_cell(3.2, 0.05) == 64
    with pytest.raises(ValueError, match="1...64"):
        voxels_per_cell(0.5, 0.03)
    with pytest.raises(ValueError, match="1...64"):
        voxels_per_cell(3.25, 0.05)
    assert windows_per_period(60, 1) == 60
    assert windows_per_period(3600, 1) == 3600
    assert windows_per_period(2, 0.5) == 4
    with pytest.raises(ValueError):
        windows_per_period(0, 1)
    with pytest.raises(ValueError):
        windows_per_period(65536, 1)
    with pytest.raises(ValueError):
        windows_per_period(60, 7)
    with pytest.raises(ValueError):
        windows_per_period(3601, 1)

    good = MemoryConfig(state_dir=None, rhythm_max_bytes=0, rhythm_max_file_bytes=0)
    validate_rhythm(good)
    validate_rhythm(MemoryConfig(state_dir=None, rhythm_max_bytes=65536, rhythm_max_file_bytes=65536))
    with pytest.raises(ValueError, match="rhythm_max_file_bytes"):
        validate_rhythm(MemoryConfig(state_dir=None, rhythm_max_bytes=65536, rhythm_max_file_bytes=65537))
    with pytest.raises(ValueError, match="65536"):
        validate_rhythm(MemoryConfig(state_dir=None, rhythm_max_file_bytes=1000))
    with pytest.raises(ValueError, match="rhythm_dims"):
        validate_rhythm(MemoryConfig(state_dir=None, rhythm_dims=1))
    with pytest.raises(ValueError, match="rhythm_enable"):
        validate_rhythm(MemoryConfig(state_dir=None, rhythm_enable=1))
    with pytest.raises(ValueError):
        RoomMemory(MemoryConfig(state_dir=None, rhythm_cell_m=0.03), START, GENERATION)


def test_node_declares_rhythm_dir_and_the_cpp_logger_is_built():
    """The startup KeyError and the unlinked C++ logger. Every MemoryConfig field is a parameter,
    and an empty window can index ``rhythm_dir`` because that key was declared."""
    package = pathlib.Path(__file__).resolve().parents[1]
    tree = ast.parse((package / "room_memory" / "node.py").read_text())
    node = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "RoomMemoryNode")
    init = next(item for item in node.body if isinstance(item, ast.FunctionDef) and item.name == "__init__")
    params = next(
        stmt.value for stmt in init.body
        if isinstance(stmt, ast.Assign) and isinstance(stmt.targets[0], ast.Name) and stmt.targets[0].id == "params"
    )
    keys = [ast.literal_eval(key) for key in params.keys]
    fields = [field.name for field in dataclasses.fields(MemoryConfig)]
    assert set(keys) == set(fields)
    assert "rhythm_dir" in keys
    source = (package / "room_memory" / "node.py").read_text()
    assert 'values["rhythm_dir"]' in source
    defaults = MemoryConfig()
    values = {name: getattr(defaults, name) for name in keys}
    values["sensors"] = tuple(values["sensors"])
    values["max_range"] = values["max_range"] if values["max_range"] > 0 else None
    values["state_dir"] = values["state_dir"] or None
    values["rhythm_dir"] = values["rhythm_dir"] or ""
    assert MemoryConfig(**values) == defaults

    cpp = package.parent / "room_memory_cpp"
    declared = set(re.findall(
        r'declare_parameter<.+?>\(\s*"([^"]+)"',
        (cpp / "node" / "room_memory_node.cpp").read_text(),
    ))
    assert declared == set(fields)
    node_cpp = (cpp / "node" / "room_memory_node.cpp").read_text()
    assert "enqueue_rhythm" in node_cpp and "memory_->close()" in node_cpp
    library = (cpp / "CMakeLists.txt").read_text()
    assert "src/rhythm.cpp" in library and "Threads::Threads" in library
    tests = (cpp / "tests" / "CMakeLists.txt").read_text()
    assert tests.count("add_test(NAME rhythm COMMAND test_rhythm)") == 1
    assert "test_rhythm" not in tests.split("foreach(suite")[1].split(")", 1)[0]
    assert "add_test(NAME interface COMMAND test_interface)" in tests
    msgs = (package.parent.parent / "ros" / "labcommand_msgs" / "CMakeLists.txt").read_text()
    assert "msg/RhythmMinute.msg" in msgs
    assert "struct RhythmMinute" in (cpp / "include" / "room_memory" / "messages.hpp").read_text()
    assert "std::make_unique<Rhythm>" in (cpp / "src" / "memory.cpp").read_text()


def test_an_empty_window_returns_and_a_directory_records_it(tmp_path):
    """``close_window`` used to raise AttributeError before it could return, because ``_rhythm``
    was never initialized. With the logger on, the same empty window is one unavailable minute.
    A state directory alone does not turn the logger on."""
    quiet = RoomMemory(MemoryConfig(state_dir=None, sensors=("mid360",)), START, GENERATION)
    assert quiet._rhythm is None and quiet.close_window(Time(60, 0)) == []
    parked = RoomMemory(MemoryConfig(state_dir=str(tmp_path), sensors=("mid360",)), START, GENERATION)
    assert parked._rhythm is None

    memory = RoomMemory(MemoryConfig(
        sensors=("mid360",), state_dir=str(tmp_path), max_range=None, rhythm_enable=True,
        rhythm_keep_days=0, rhythm_fsync_s=0,
    ), START, GENERATION)
    try:
        assert memory._rhythm is not None
        assert memory.close_window(Time(60, 0)) == []
        memory.enqueue_rhythm()
    finally:
        memory.close()
    path = tmp_path / "lab" / "rhythm" / "rhythm-mid360-19700101.rhy"
    data = path.read_bytes()
    identity = header_identity(data)
    assert identity["sensor"] == 1 and identity["room_id"] == "lab"
    offset, minute = scan_records(data)
    record = parse_record(data[identity["header_bytes"]:offset])
    assert minute == 60 and record["unavailable"] == 1 and record["closed"] == 0
    assert record["with_points"] == 0 and record["regions"] == () and record["flags"] & FLAG_UNAVAILABLE


def test_rhythm_history_depth_is_one_minute_per_sensor():
    """Depth 1 kept only the newest message. Both lidars share ``/room_memory/rhythm``."""
    assert history_depth(1) == 1 and history_depth(2) == 2
    with pytest.raises(ValueError, match="history depth"):
        history_depth(0)
    with pytest.raises(ValueError):
        history_depth(True)
    package = pathlib.Path(__file__).resolve().parents[1]
    node = (package / "room_memory" / "node.py").read_text()
    assert "latched(history_depth(len(self.config.sensors)))" in node
    cpp = (package.parent / "room_memory_cpp" / "node" / "room_memory_node.cpp").read_text()
    assert "latched_qos(room_memory::history_depth(config.sensors.size()))" in cpp


def test_defaults_are_off_and_do_not_publish(tmp_path):
    defaults = MemoryConfig()
    assert defaults.rhythm_enable is False and defaults.rhythm_publish is False
    assert defaults.rhythm_cell_m == 0.5 and defaults.rhythm_dims == 2 and defaults.rhythm_period_s == 60
    assert defaults.rhythm_dir == "" and defaults.rhythm_keep_days == 90
    assert defaults.rhythm_max_bytes == 536870912 and defaults.rhythm_max_file_bytes == 33554432
    assert defaults.rhythm_fsync_s == 600
    assert rhythm_active(MemoryConfig(state_dir=None)) is False
    assert rhythm_active(MemoryConfig(state_dir=str(tmp_path))) is False
    assert rhythm_active(MemoryConfig(state_dir=None, rhythm_publish=True)) is False
    assert rhythm_active(MemoryConfig(state_dir=None, rhythm_enable=True, rhythm_publish=True)) is True
    assert rhythm_active(MemoryConfig(state_dir=str(tmp_path), rhythm_enable=False)) is False
    quiet = RoomMemory(MemoryConfig(state_dir=None), START, GENERATION)
    disabled = RoomMemory(MemoryConfig(state_dir=str(tmp_path), rhythm_enable=False), START, GENERATION)
    assert quiet._rhythm is None and disabled._rhythm is None
    publish_only = RoomMemory(MemoryConfig(
        state_dir=None, rhythm_enable=True, rhythm_publish=True, rhythm_keep_days=0,
    ), START, GENERATION)
    try:
        assert publish_only._rhythm is not None and publish_only._rhythm.writer.directory is None
    finally:
        publish_only.close()


def test_floor_pins_dims_and_the_mean_pin():
    def cell(index, dims=2, k=0):
        acc = RhythmAccumulator(["mid360"], 10, dims, 60, 60)
        acc.add("mid360", 60, key(index, 0, k), 1)
        return parse_record(acc.flush()[0].data)["regions"]

    assert cell(9) == ((0, 0, 0, 1, 1),)
    assert cell(10) == ((1, 0, 0, 1, 1),)
    assert cell(-1) == ((-1, 0, 0, 1, 1),)
    assert cell(-10) == ((-1, 0, 0, 1, 1),)
    assert cell(-11) == ((-2, 0, 0, 1, 1),)

    both = np.array([pack(0, 0, 0), pack(0, 0, 10)], dtype=np.int64)
    flat = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    flat.add("mid360", 60, both, 1)
    assert parse_record(flat.flush()[0].data)["regions"] == ((0, 0, 0, 1, 2),)
    deep = RhythmAccumulator(["mid360"], 10, 3, 60, 60)
    deep.add("mid360", 60, both, 1)
    assert parse_record(deep.flush()[0].data)["regions"] == ((0, 0, 0, 1, 1), (0, 0, 1, 1, 1))


def _slot_of(table, ix, iy, iz):
    mask = (table.used == 1) & (table.ix == ix) & (table.iy == iy) & (table.iz == iz)
    found = np.flatnonzero(mask)
    assert len(found) == 1
    return int(found[0])


def test_a_coarse_cell_split_by_floor_counts_once_on_insert_and_on_the_cache():
    """Sorted voxels do not stay grouped after floor division.

    With 10 voxels per cell, ``(0, 0, 0), (0, 10, 0), (1, 0, 0)`` become A, B, A.
    An adjacent run inserts A twice on the first window. The cached slot list then
    repeats A, and that vectorized update does not accumulate a repeated index.
    Negatives use the same split, and one index outside int16 stays clipped.
    """
    voxels = [
        (0, 0, 0), (0, 10, 0), (1, 0, 0), (1, 1, 0), (1, 2, 0),
        (-10, 0, 0), (-10, 10, 0), (-1, 0, 0),
        (327680, 0, 0),
    ]
    keys = np.array([pack(*voxel) for voxel in reversed(voxels)], dtype=np.int64)
    expected = ((-1, 0, 0, 2), (-1, 1, 0, 1), (0, 0, 0, 4), (0, 1, 0, 1))

    for dims in (2, 3):
        acc = RhythmAccumulator(["mid360"], 10, dims, 60, 60)
        clipped, ix, iy, iz, counts = acc._cells(keys)
        assert clipped == 1
        assert [
            (int(ix[n]), int(iy[n]), int(iz[n]), int(counts[n])) for n in range(len(ix))
        ] == list(expected)

        acc.add("mid360", 60, keys, 1)
        table = acc._sensors["mid360"]["table"]
        assert table._cached == len(expected)
        assert len(np.unique(table._cslot[:table._cached])) == table._cached
        for cell_ix, cell_iy, cell_iz, count in expected:
            slot = _slot_of(table, cell_ix, cell_iy, cell_iz)
            assert int(table.hits[slot]) == 1 and int(table.sums[slot]) == count

        def refuse_probe(*_args):
            raise AssertionError("cached window probed the table")

        table._slot = refuse_probe
        acc.add("mid360", 61, keys, 1)
        for cell_ix, cell_iy, cell_iz, count in expected:
            slot = _slot_of(table, cell_ix, cell_iy, cell_iz)
            assert int(table.hits[slot]) == 2 and int(table.sums[slot]) == count * 2
        record = parse_record(acc.flush()[0].data)
        assert record["with_points"] == 2 and record["clipped"] == 2
        assert record["regions"] == tuple(
            (cell_ix, cell_iy, cell_iz, 2, count) for cell_ix, cell_iy, cell_iz, count in expected
        )

    # k is the fast-changing lane, so these three keys are already sorted. Dims 3 still
    # sees A, B, A. Dims 2 stores iz 0 and the three voxels are one cell.
    k_split = np.array([pack(0, 0, 0), pack(0, 0, 10), pack(0, 1, 0)], dtype=np.int64)
    flat = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    _, ix, iy, iz, counts = flat._cells(k_split)
    assert [(int(ix[n]), int(iy[n]), int(iz[n]), int(counts[n])) for n in range(len(ix))] == [(0, 0, 0, 3)]
    deep = RhythmAccumulator(["mid360"], 10, 3, 60, 60)
    _, ix, iy, iz, counts = deep._cells(k_split)
    assert [
        (int(ix[n]), int(iy[n]), int(iz[n]), int(counts[n])) for n in range(len(ix))
    ] == [(0, 0, 0, 2), (0, 0, 1, 1)]


def test_room_shell_minute_counts_each_cell_once_and_keeps_every_voxel():
    """18,724 occupied voxels in 64 cells. Identical windows keep one hit each.

    The summed means stay the occupied-voxel count, which is what a published
    ``RhythmMinute`` checks against ``windows_with_points``.
    """
    keys = occupancy_map(scenes.room_shell(), MemoryConfig(state_dir=None).change_config()).occupied
    assert len(keys) == 18724
    acc = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    acc.add("mid360", 60, keys, 1)
    table = acc._sensors["mid360"]["table"]
    used = table.used == 1
    assert table._cached == 64 and int(used.sum()) == 64
    assert len(np.unique(table._cslot[:table._cached])) == 64
    assert int(table.hits[used].sum()) == 64
    assert int(table.sums[used].sum()) == 18724

    def refuse_probe(*_args):
        raise AssertionError("cached window probed the table")

    table._slot = refuse_probe
    for sec in range(61, 120):
        acc.add("mid360", sec, keys, 1)
    used = table.used == 1
    assert int(table.hits[used].sum()) == 64 * 60
    assert int(table.sums[used].sum()) == 18724 * 60
    record = parse_record(acc.flush()[0].data)
    assert (record["closed"], record["with_points"], record["unavailable"]) == (60, 60, 0)
    assert record["clipped"] == 0 and record["regions_dropped"] == 0 and record["flags"] == 0
    assert len(record["regions"]) == 64
    assert all(hits == 60 for _ix, _iy, _iz, hits, _mean in record["regions"])
    assert sum(mean for _ix, _iy, _iz, _hits, mean in record["regions"]) == 18724
    regions = record["regions"]
    message = RhythmMinute(
        "lab", 1, record["map_epoch"], 1, Time(record["minute"], 0),
        record["closed"], record["with_points"], record["unavailable"],
        record["clipped"], record["regions_dropped"], record["pose_ok"], record["flags"],
        tuple(region[0] for region in regions),
        tuple(region[1] for region in regions),
        tuple(region[2] for region in regions),
        tuple(region[3] for region in regions),
        tuple(region[4] for region in regions),
    )
    assert RhythmMinute.from_cdr(message.to_cdr()) == message


def test_clipped_indices_are_dropped_and_zeros_are_omitted():
    acc = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    keys = np.array([pack(0, 0, 0), pack(327680, 0, 0), pack(-327690, 0, 0)], dtype=np.int64)
    acc.add("mid360", 60, keys, 1)
    record = parse_record(acc.flush()[0].data)
    assert record["clipped"] == 2
    assert record["regions"] == ((0, 0, 0, 1, 1),)

    table = _Table(4)
    table.used[0] = 1
    table.count = 1
    table.ix[0] = 4
    table.hits[0] = 0
    table.sums[0] = np.uint64(80)
    assert table.regions(10) == ()
    table.hits[0] = 1
    table.sums[0] = np.uint64((1 << 63) + 10)
    assert table.regions(1) == ((4, 0, 0, 1, 65535),)


def test_empty_is_not_unavailable_and_a_building_skip_stores_no_regions():
    empty = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    empty.add_empty("mid360", 60, 1)
    empty.add_empty("mid360", 61, 1)
    record = parse_record(empty.flush()[0].data)
    assert (record["closed"], record["with_points"], record["unavailable"], record["regions"]) == (2, 0, 0, ())
    assert record["pose_ok"] == 0
    assert record["flags"] & FLAG_PARTIAL and not record["flags"] & FLAG_UNAVAILABLE

    skipped = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    skipped.note_unavailable("mid360", 60, 1)
    record = parse_record(skipped.flush()[0].data)
    assert (record["closed"], record["with_points"], record["unavailable"], record["regions"]) == (0, 0, 1, ())
    assert record["flags"] & FLAG_UNAVAILABLE and record["pose_ok"] == 0


def test_pose_ok_requires_every_point_window_and_this_engine_stores_zero(tmp_path):
    def pose(tracking_residual):
        acc = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
        for sec, residual in tracking_residual:
            if residual is None:
                acc.add_empty("mid360", sec, 1)
            else:
                acc.add("mid360", sec, key(0, 0, 0), 1, tracking=True, residual=residual, learn_max_offset_m=0.2)
        return parse_record(acc.flush()[0].data)["pose_ok"]

    assert pose((sec, 0.1) for sec in range(60)) == 1
    assert pose([(60, 0.1), (61, 0.5)]) == 0
    assert pose([(60, 0.0), (61, None), (62, 0.0)]) == 1
    assert pose([(60, None)]) == 0

    memory = RoomMemory(config(tmp_path, reference_windows=1, max_range=None), START, GENERATION)
    try:
        memory.add_points("mid360", scenes.room_shell())
        memory.close_window(stamp(0))
        memory.add_points("mid360", scenes.room_shell())
        memory.close_window(stamp(1))
        record = parse_record(memory._rhythm.accumulator.flush()[0].data)
        assert memory.state.name == "TRACKING" and record["pose_ok"] == 0 and record["with_points"] > 0
    finally:
        memory.close()


def test_sixty_sixtyone_fiftynine_stay_one_fragment_and_gaps_write_no_filler():
    acc = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    for sec in (60, 61, 59):
        acc.add("mid360", sec, key(0, 0, 0), 1)
    done = acc.flush()
    assert len(done) == 1
    record = parse_record(done[0].data)
    assert record["minute"] == 60 and record["closed"] == 3
    assert not record["flags"] & FLAG_DISCONTINUITY

    edge = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    edge.add("mid360", 60, key(0, 0, 0), 1)
    edge.add("mid360", 58, key(0, 0, 0), 1)
    assert edge.finished == []
    edge.add("mid360", 57, key(0, 0, 0), 1)
    assert parse_record(edge.finished[0].data)["minute"] == 60
    earlier = [parse_record(fragment.data) for fragment in edge.flush()]
    assert [item["minute"] for item in earlier] == [60, 0]
    assert earlier[1]["flags"] & FLAG_DISCONTINUITY

    gap = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    gap.add("mid360", 60, key(0, 0, 0), 1)
    gap.add("mid360", 240, key(0, 0, 0), 1)
    minutes = [parse_record(fragment.data) for fragment in gap.flush()]
    assert [item["minute"] for item in minutes] == [60, 240]
    assert minutes[1]["flags"] & FLAG_DISCONTINUITY

    back = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    back.add("mid360", 120, key(0, 0, 0), 1)
    back.add("mid360", 0, key(0, 0, 0), 1)
    stepped = [parse_record(fragment.data) for fragment in back.flush()]
    assert [item["minute"] for item in stepped] == [120, 0]
    assert stepped[1]["flags"] & FLAG_DISCONTINUITY

    skipped = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    skipped.add("mid360", -1, key(0, 0, 0), 1)
    assert skipped.flush() == []


def test_the_first_miss_sets_overflow_and_stops_the_window():
    full = RhythmAccumulator(["mid360"], 1, 2, 60, 60)
    full.add("mid360", 10, pack(np.arange(1, 4097), np.zeros(4096, dtype=np.int64), np.zeros(4096, dtype=np.int64)), 1)
    full.add("mid360", 10, pack(np.array([0, 1]), np.zeros(2, dtype=np.int64), np.zeros(2, dtype=np.int64)), 1)
    record = parse_record(full.flush()[0].data)
    assert record["flags"] & FLAG_OVERFLOW and not record["flags"] & FLAG_LOST
    assert record["regions_dropped"] == 2
    stored = {region[0]: region[3] for region in record["regions"]}
    assert 0 not in stored and stored[1] == 1 and len(record["regions"]) == 4096

    wide = RhythmAccumulator(["mid360"], 1, 2, 60, 60)
    wide.add("mid360", 10, pack(np.arange(4100), np.zeros(4100, dtype=np.int64), np.zeros(4100, dtype=np.int64)), 1)
    record = parse_record(wide.flush()[0].data)
    assert len(record["regions"]) == 4096 and record["regions_dropped"] == 4
    assert record["flags"] & FLAG_OVERFLOW and record["regions"][0][0] == 0 and record["regions"][-1][0] == 4095


def test_slots_are_fixed_and_the_hash_uses_uint16_lanes():
    acc = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    table = acc._sensors["mid360"]["table"]
    arrays = tuple(id(getattr(table, name)) for name in ("used", "ix", "iy", "iz", "hits", "sums"))
    for sec in range(4):
        acc.add("mid360", sec, pack(np.arange(80), np.zeros(80, dtype=np.int64), np.zeros(80, dtype=np.int64)), 1)
    assert table.slots == REGION_SLOTS == 4096
    assert arrays == tuple(id(getattr(table, name)) for name in ("used", "ix", "iy", "iz", "hits", "sums"))

    probe = _Table()
    packed = (np.int64(-1 & 0xFFFF) << np.int64(32)) | (np.int64(2) << np.int64(16)) | np.int64(3)
    start = int(np.asarray(splitmix64(packed)).reshape(-1)[0]) & probe.mask
    probe.insert_window(
        np.array([-1], dtype=np.int16), np.array([2], dtype=np.int16), np.array([3], dtype=np.int16),
        np.array([1], dtype=np.int64),
    )
    assert int(probe.used[start]) == 1 and int(probe.ix[start]) == -1
    shuffled = np.array([pack(n, -n, n % 3) for n in range(40)], dtype=np.int64)
    ordered = RhythmAccumulator(["mid360"], 10, 3, 60, 60)
    mixed = RhythmAccumulator(["mid360"], 10, 3, 60, 60)
    ordered.add("mid360", 60, np.sort(shuffled), 1)
    mixed.add("mid360", 60, shuffled[::-1].copy(), 1)
    assert parse_record(ordered.flush()[0].data)["regions"] == parse_record(mixed.flush()[0].data)["regions"]


def test_a_failed_seal_keeps_the_open_bin_for_discard(monkeypatch):
    import room_memory.rhythm as rhythm_module

    acc = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    acc.add("mid360", 60, key(0, 0, 0), 5)
    monkeypatch.setattr(rhythm_module, "encode_record", lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("encode")))
    with pytest.raises(RuntimeError, match="encode"):
        acc.add("mid360", 120, key(0, 0, 0), 5)
    assert acc._sensors["mid360"]["bin"].minute == 60
    acc.discard("mid360", 120, 9)
    assert acc._sensors["mid360"]["loss"] == (60, 5)
    assert acc._sensors["mid360"]["bin"] is None and acc.hook_errors == 1


def test_shared_header_and_sixty_window_hex(rhythm, tmp_path):
    acc = RhythmAccumulator(["mid360"], 10, 2, 60, 60)
    for n in range(60):
        if n < 15:
            keys = pack(np.arange(10), np.zeros(10, dtype=np.int64), np.full(10, 99, dtype=np.int64))
        else:
            keys = key(10, 0, 5)
        acc.add("mid360", 60 + n, keys, 4)
    fragment = acc.flush()
    assert len(fragment) == 1
    record = parse_record(fragment[0].data)
    assert record["minute"] == 60 and record["map_epoch"] == 4
    assert (record["closed"], record["with_points"], record["unavailable"]) == (60, 60, 0)
    assert record["pose_ok"] == 0 and record["flags"] == 0 and record["pad"] == 0
    assert record["regions"] == ((0, 0, 0, 15, 3), (1, 0, 0, 45, 1))
    blob = encode_header(1, 2, 10, np.float32(0.05), np.float32(0.5), 60, "lab") + fragment[0].data
    assert blob.hex() == FILE_HEX

    log = rhythm(path=tmp_path / "golden")
    for n in range(60):
        if n < 15:
            keys = pack(np.arange(10), np.zeros(10, dtype=np.int64), np.full(10, 99, dtype=np.int64))
        else:
            keys = key(10, 0, 5)
        log.add("mid360", 60 + n, keys, 4)
    log.close()
    written = (tmp_path / "golden" / "lab" / "rhythm" / "rhythm-mid360-19700101.rhy").read_bytes()
    assert written.hex() == FILE_HEX


# MARK: CDR


def minute_message(sensor=1):
    return RhythmMinute(
        "lab", sensor, 4, 1, Time(60, 0), 60, 60, 0, 0, 0, 0, 0,
        (0, 1), (0, 0), (0, 0), (15, 45), (3, 1),
    )


def test_rhythm_minute_golden_round_trips_and_rejects_other_sensors():
    message = minute_message(1)
    assert message.to_cdr().hex() == CDR_LE
    assert RhythmMinute.from_cdr(bytes.fromhex(CDR_LE)) == message
    assert RhythmMinute.from_cdr(bytes.fromhex(CDR_BE)) == message
    twin = minute_message(2)
    assert twin.to_cdr() != message.to_cdr()
    assert RhythmMinute.from_cdr(twin.to_cdr()) == twin
    assert twin.to_cdr()[12] == 2 and message.to_cdr()[12] == 1 and bytes.fromhex(CDR_BE)[12] == 1
    for sensor in (0, 3, 255):
        bad = bytearray(message.to_cdr())
        bad[12] = sensor
        with pytest.raises(Invalid):
            RhythmMinute.from_cdr(bytes(bad))
        with pytest.raises(Invalid):
            RhythmMinute(
                "lab", sensor, 4, 1, Time(60, 0), 0, 0, 0, 0, 0, 0, 0, (), (), (), (), (),
            ).to_cdr()
    data = message.to_cdr()
    for length in range(len(data)):
        with pytest.raises(CDRError):
            RhythmMinute.from_cdr(data[:length])
    with pytest.raises(Invalid, match="increasing"):
        RhythmMinute(
            "lab", 1, 4, 1, Time(60, 0), 60, 60, 0, 0, 0, 0, 0,
            (1, 0), (0, 0), (0, 0), (1, 1), (1, 1),
        ).to_cdr()
    with pytest.raises(Invalid):
        RhythmMinute(
            "lab", 1, 4, 1, Time(60, 0), 0, 0, 0, 0, 0, 0, 0,
            (0,), (0,), (0,), (1,), (1,),
        ).to_cdr()
    stamped = RhythmMinute(
        "lab", 1, 4, 1, Time(60, 0), 60, 60, 0, 0, 0, 0, 0,
        (0, 1), (0, 0), (0, 0), (15, 45), (3, 1), GENERATION,
    )
    stamped_bytes = stamped.to_cdr()
    assert stamped_bytes.startswith(message.to_cdr())
    assert RhythmMinute.from_cdr(stamped_bytes).publisher_generation == GENERATION
    assert RhythmMinute.from_cdr(message.to_cdr()).publisher_generation is None
    with pytest.raises(Invalid):
        RhythmMinute(
            "lab", 1, 4, 1, Time(60, 0), 60, 60, 0, 0, 0, 0, 0,
            (0, 1), (0, 0), (0, 0), (15, 45), (3, 1), 0,
        ).to_cdr()


def test_a_loss_marker_message_carries_its_sensor():
    marker = RhythmMinute(
        "lab", 2, 4, 7, Time(60, 0), 0, 0, 0, 0, 0, 0, FLAG_DISCONTINUITY | FLAG_LOST,
        (), (), (), (), (),
    )
    restored = RhythmMinute.from_cdr(marker.to_cdr())
    assert restored == marker and restored.sensor == 2 and restored.ix == ()


# MARK: writer


def test_full_queue_drops_the_record_and_writes_a_loss_marker(rhythm, tmp_path):
    log = rhythm(path=tmp_path / "queue", rhythm_period_s=1, window_s=1.0)
    log.writer.gate.clear()
    time.sleep(0.3)
    for sec in range(9):
        log.add("mid360", sec, key(0, 0, 0), 3)
    log.enqueue()
    assert log.writer._count == QUEUE_SLOTS == 8
    assert all(len(slot) == MAX_RECORD_BYTES == 41004 for slot in log.writer._slots)
    # Seal the open minute into ``finished``. ``flush`` would take that record and drop it.
    log.accumulator._seal("mid360")
    log.enqueue()
    assert log.writer.queue_dropped >= 1
    assert log.accumulator._sensors["mid360"]["loss"][0] == 8
    log.writer.gate.set()
    log.writer._wake.set()
    assert log.writer.wait_idle(2)
    log.add("mid360", 30, key(0, 0, 0), 3)
    log.add("mid360", 31, key(1, 0, 0), 3)
    log.enqueue()
    assert log.writer.wait_idle(2)
    _, records, _, end = load_rhy(tmp_path / "queue" / "lab" / "rhythm" / "rhythm-mid360-19700101.rhy")
    assert end == (tmp_path / "queue" / "lab" / "rhythm" / "rhythm-mid360-19700101.rhy").stat().st_size
    marker = [record for record in records if record["minute"] == 8]
    assert len(marker) == 1 and marker[0]["regions"] == () and marker[0]["closed"] == 0
    assert marker[0]["flags"] == FLAG_DISCONTINUITY | FLAG_LOST
    assert [record["minute"] for record in records if record["flags"] & FLAG_LOST] == [8]


def test_short_write_retry_truncates_before_the_marker(rhythm, tmp_path):
    store = ScriptStore([full_write, short_write(4), short_write(4), full_write])
    directory = tmp_path / "short"
    log = rhythm(path=directory, store=store)
    minute = this_minute()
    log.add("mid360", minute + 10, key(0, 0, 0), 6)
    log.close()
    path = directory / "lab" / "rhythm" / f"rhythm-mid360-{utc_day(0)}.rhy"
    header, records, data, end = load_rhy(path)
    assert end == len(data) and len(records) == 1
    assert store.truncations == [header["header_bytes"], header["header_bytes"]]
    assert records[0]["minute"] == minute and records[0]["regions"] == ()
    assert records[0]["flags"] == FLAG_DISCONTINUITY | FLAG_LOST
    assert records[0]["map_epoch"] == 6 and records[0]["closed"] == 0
    assert log.writer.io_errors >= 1
    assert len(rhy_files(directory)) == 1


def test_failed_truncate_keeps_the_prefix_and_the_next_start_sets_bit_1(rhythm, tmp_path):
    store = ScriptStore([full_write, short_write(4)])
    store.truncate_error = OSError(errno.EIO, "truncate")
    directory = tmp_path / "tear"
    log = rhythm(path=directory, store=store)
    minute = this_minute()
    log.add("mid360", minute + 10, key(0, 0, 0), 6)
    log.close()
    path = directory / "lab" / "rhythm" / f"rhythm-mid360-{utc_day(0)}.rhy"
    assert path.stat().st_size > header_identity(path.read_bytes())["header_bytes"]
    assert len(rhy_files(directory)) == 1
    assert not list(path.parent.glob("rhythm-mid360-*.rhy.bad-*"))
    restarted = rhythm(path=directory)
    restarted.add("mid360", minute + 20, key(3, 0, 0), 6)
    restarted.close()
    assert len(rhy_files(directory)) == 1
    _, records, data, end = load_rhy(path)
    assert end == len(data) and len(records) == 1
    assert records[0]["flags"] & FLAG_DISCONTINUITY
    assert records[0]["regions"][0][0] == 0  # voxel 3*10? vpc 10, key(3,0,0) is voxel 3, cell 0
    assert not records[0]["flags"] & FLAG_LOST


def test_restart_recovery_supersede_bad_magic_and_oversized(rhythm, tmp_path):
    minute = this_minute()
    day = utc_day(0)
    directory = tmp_path / "recover"
    first = rhythm(path=directory)
    first.add("mid360", minute + 5, key(0, 0, 0), 2)
    first.add("mid360", minute + 6, key(0, 0, 0), 2)
    first.close()
    path = directory / "lab" / "rhythm" / f"rhythm-mid360-{day}.rhy"
    original = path.read_bytes()
    path.write_bytes(original + b"\xff\xff")
    second = rhythm(path=directory)
    second.add("mid360", minute + 7, key(10, 0, 0), 2)
    second.close()
    _, records, data, end = load_rhy(path)
    assert end == len(data) and len(records) == 2
    assert records[0]["minute"] == records[1]["minute"] == minute
    assert records[0]["closed"] == 2 and records[1]["closed"] == 1
    assert records[1]["flags"] & FLAG_DISCONTINUITY
    assert records[1]["regions"][0][3] == 1

    mismatch = tmp_path / "mismatch"
    planted = mismatch / "lab" / "rhythm"
    planted.mkdir(parents=True)
    foreign = encode_header(1, 2, 10, np.float32(0.05), np.float32(0.5), 30, "lab")
    (planted / f"rhythm-mid360-{day}.rhy").write_bytes(foreign)
    third = rhythm(path=mismatch)
    third.add("mid360", minute + 5, key(0, 0, 0), 1)
    third.close()
    names = sorted(path.name for path in rhy_files(mismatch))
    assert any(".superseded-" in name for name in names)
    _, records, _, _ = load_rhy(planted / f"rhythm-mid360-{day}.rhy")
    assert records[0]["flags"] & FLAG_DISCONTINUITY

    bad = tmp_path / "bad"
    bad_dir = bad / "lab" / "rhythm"
    bad_dir.mkdir(parents=True)
    (bad_dir / f"rhythm-mid360-{day}.rhy").write_bytes(b"NOPE" + b"\x00" * 32)
    fourth = rhythm(path=bad)
    fourth.add("mid360", minute + 5, key(0, 0, 0), 1)
    fourth.close()
    assert any(".bad-" in path.name for path in rhy_files(bad))
    _, records, _, _ = load_rhy(bad_dir / f"rhythm-mid360-{day}.rhy")
    assert records[0]["flags"] & FLAG_DISCONTINUITY

    huge = tmp_path / "huge"
    huge_dir = huge / "lab" / "rhythm"
    huge_dir.mkdir(parents=True)
    huge_path = huge_dir / f"rhythm-mid360-{day}.rhy"
    fd = os.open(huge_path, os.O_CREAT | os.O_RDWR, 0o644)
    os.truncate(fd, RECOVERY_BYTES + 1)
    os.close(fd)
    fifth = rhythm(path=huge)
    fifth.add("mid360", minute + 5, key(0, 0, 0), 1)
    fifth.close()
    quarantined = [path for path in rhy_files(huge) if ".superseded-" in path.name]
    assert len(quarantined) == 1 and quarantined[0].stat().st_size == RECOVERY_BYTES + 1
    _, records, _, _ = load_rhy(huge_dir / f"rhythm-mid360-{day}.rhy")
    assert records[0]["flags"] & FLAG_DISCONTINUITY


def test_rotation_stays_inside_the_byte_cap_and_spares_the_open_file(rhythm, tmp_path):
    directory = tmp_path / "rotate"
    log = rhythm(
        path=directory, rhythm_max_bytes=65536, rhythm_max_file_bytes=65536, rhythm_period_s=60,
    )
    wide = pack(np.arange(4096), np.zeros(4096, dtype=np.int64), np.zeros(4096, dtype=np.int64))
    for minute in (1_700_000_040, 1_700_000_100, 1_700_000_160):
        log.add("mid360", minute, wide, 1)
    log.close()
    files = rhy_files(directory)
    assert files
    assert sum(path.stat().st_size for path in files) <= 65536
    _, records, _, _ = load_rhy([path for path in files if path.suffix == ".rhy"][-1])
    assert records[-1]["minute"] == 1_700_000_160
    assert all(".rhy" in path.name for path in files)

    budget = tmp_path / "budget"
    root = budget / "lab" / "rhythm"
    root.mkdir(parents=True)
    from room_memory.rhythm import RhythmWriter, _OpenSegment

    # Not today's name, so startup recovery does not quarantine this planted append file.
    open_path = root / "rhythm-mid360-20200101.rhy"
    open_path.write_bytes(b"open")
    quarantine = root / "rhythm-mid360-20261015.rhy.superseded-1"
    quarantine.write_bytes(b"q" * 100)
    closed = root / "rhythm-jt128-20261015.rhy"
    closed.write_bytes(b"c" * 100)
    os.utime(quarantine, (1, 1))
    writer = RhythmWriter(config(budget, rhythm_max_bytes=154, rhythm_keep_days=0), root)
    writer._open["mid360"] = _OpenSegment(open_path, -1, "20200101", 1, None)
    assert writer._fits(50, prune=True) is True
    assert not quarantine.exists() and closed.exists() and open_path.exists()
    assert writer._fits(10_000, prune=True) is False
    assert open_path.exists()


def test_age_prune_runs_at_startup_and_once_per_utc_day(tmp_path):
    directory = tmp_path / "lab" / "rhythm"
    directory.mkdir(parents=True)
    old = directory / f"rhythm-mid360-{utc_day(-91)}.rhy"
    kept = directory / f"rhythm-jt128-{utc_day(-90)}.rhy"
    today = directory / f"rhythm-mid360-{utc_day(0)}.rhy"
    old.write_bytes(b"old")
    kept.write_bytes(b"kept")
    # A valid header, so recovery leaves today's append file in place for the age check.
    today.write_bytes(encode_header(1, 2, 10, np.float32(0.05), np.float32(0.5), 60, "lab"))
    from room_memory.rhythm import RhythmWriter

    writer = RhythmWriter(config(tmp_path, rhythm_keep_days=90), directory)
    assert not old.exists() and kept.exists() and today.exists()
    late = directory / f"rhythm-mid360-{utc_day(-91)}-2.rhy"
    late.write_bytes(b"late")
    writer._maybe_age_prune()
    assert late.exists()
    writer._last_age_day = None
    writer._maybe_age_prune()
    assert not late.exists() and kept.exists()


def test_a_blocking_store_keeps_the_callback_under_50ms(rhythm, tmp_path):
    class BlockingStore:
        def __init__(self):
            self.started = __import__("threading").Event()
            self._slept = False

        def write(self, fd, data):
            # A one-region record is 54 bytes. The lab header is 36. Block on the record only.
            if len(data) >= 44 and not self._slept:
                self._slept = True
                self.started.set()
                time.sleep(0.5)
            return os.write(fd, data)

        def fsync(self, fd):
            os.fsync(fd)

        def truncate(self, fd, length):
            os.ftruncate(fd, length)

    store = BlockingStore()
    log = rhythm(path=tmp_path / "block", store=store)
    log.add("mid360", 60, key(0, 0, 0), 1)
    log.add("mid360", 120, key(0, 0, 0), 1)
    log.enqueue()
    assert store.started.wait(2)
    samples = []
    for _ in range(10):
        started = time.perf_counter()
        log.writer.try_enqueue("mid360", encode_header(1, 2, 10, 0.05, 0.5, 60, "lab")[:44])
        samples.append(time.perf_counter() - started)
    assert max(samples) < 0.05

    memory = RoomMemory(config(tmp_path / "callback", reference_windows=1, max_range=None), START, GENERATION)
    try:
        blocker = BlockingStore()
        memory._rhythm.writer.store = blocker
        memory._rhythm.add("mid360", 60, key(0, 0, 0), 0)
        memory._rhythm.add("mid360", 120, key(0, 0, 0), 0)
        memory.enqueue_rhythm()
        assert blocker.started.wait(2)
        started = time.perf_counter()
        memory.add_points("mid360", np.zeros((8, 3), dtype=np.float32))
        memory.close_window(stamp(3))
        memory.enqueue_rhythm()
        assert time.perf_counter() - started < 0.05
    finally:
        memory.close()


def test_publish_drops_when_it_would_block_and_markers_carry_sensor(rhythm, tmp_path):
    published = []
    directory = tmp_path / "pub"
    log = rhythm(
        path=directory, rhythm_publish=True, publish=published.append, sensors=("mid360", "jt128"),
        publisher_generation=GENERATION,
    )
    log.add("mid360", 60, key(0, 0, 0), 4)
    log.add("jt128", 60, key(10, 0, 0), 4)
    log.close()
    messages = [RhythmMinute.from_cdr(blob) for blob in published]
    assert sorted(message.sensor for message in messages) == [1, 2]
    assert {message.room_id for message in messages} == {"lab"}
    assert {message.publisher_generation for message in messages} == {GENERATION}
    jt = directory / "lab" / "rhythm" / "rhythm-jt128-19700101.rhy"
    assert header_identity(jt.read_bytes())["sensor"] == 2

    blocked = []
    quiet = rhythm(
        path=tmp_path / "drop", rhythm_publish=True, publish=blocked.append, publish_would_block=lambda: True,
    )
    quiet.add("mid360", 60, key(0, 0, 0), 1)
    quiet.close()
    assert blocked == [] and quiet.writer.publish_dropped >= 1
    assert rhy_files(tmp_path / "drop")


def test_startup_disk_error_logs_once_and_detection_still_starts(tmp_path):
    from room_memory.rhythm import RhythmWriter

    blocked = tmp_path / "not-a-directory"
    blocked.write_text("x")
    logs = []
    writer = RhythmWriter(config(tmp_path), blocked / "lab" / "rhythm", log_fn=logs.append)
    assert writer.io_errors >= 1 and len(logs) == 1
    writer._log_once("again")
    assert len(logs) == 1
    memory = RoomMemory(
        config(blocked, rhythm_keep_days=0),
        START, GENERATION,
    )
    try:
        assert memory._rhythm is not None
        memory.add_points("mid360", np.zeros((4, 3), dtype=np.float32))
        assert memory.close_window(stamp(0)) == []
    finally:
        memory.close()


def test_shutdown_fsyncs_once_and_does_not_prune(rhythm, tmp_path):
    store = ScriptStore()
    directory = tmp_path / "sync"
    log = rhythm(path=directory, store=store, rhythm_fsync_s=600, rhythm_keep_days=1)
    log.add("mid360", this_minute() + 5, key(0, 0, 0), 1)
    log.close()
    assert store.fsyncs == 2
    late = directory / "lab" / "rhythm" / f"rhythm-mid360-{utc_day(-5)}.rhy"
    # Planted after close, so shutdown did not prune it. A new writer prunes at startup.
    late.parent.mkdir(parents=True, exist_ok=True)
    late.write_bytes(b"late")
    assert late.exists()


def test_building_skip_does_not_run_a_second_occupancy(tmp_path, monkeypatch):
    import room_memory.memory as memory_module

    calls = []
    real = memory_module.occupancy_map

    def counted(*args, **kwargs):
        calls.append(1)
        return real(*args, **kwargs)

    monkeypatch.setattr(memory_module, "occupancy_map", counted)
    memory = RoomMemory(
        config(tmp_path, sensors=("mid360", "jt128"), reference_windows=1, max_range=None,
               occupied_min_hits=1, occupied_min_ratio=0, free_min_seen=1),
        START, GENERATION,
    )
    try:
        room = scenes.room_shell()
        memory.add_points("mid360", room)
        assert memory.close_window(stamp(0)) == []
        assert len(calls) == 1 and memory.state.name == "BUILDING"
        memory.add_points("mid360", room)
        memory.add_points("jt128", room)
        assert memory.close_window(stamp(1)) == []
        assert len(calls) == 2 and memory.state.name == "TRACKING"
        mid = memory._rhythm.accumulator._sensors["mid360"]["bin"]
        jt = memory._rhythm.accumulator._sensors["jt128"]["bin"]
        assert mid.unavailable == 1 and mid.with_points == 1
        assert jt.unavailable == 1 and jt.with_points == 1
    finally:
        memory.close()


def test_a_throw_discards_only_that_sensor(tmp_path, monkeypatch):
    memory = RoomMemory(
        config(tmp_path, sensors=("mid360", "jt128"), reference_windows=1, max_range=None,
               occupied_min_hits=1, occupied_min_ratio=0, free_min_seen=1,
               candidate_windows=1, expire_windows=1, fresh_min_voxels=1, fresh_min_fraction=0,
               activity_radius_m=0, min_voxels=1, min_extent=1),
        START, GENERATION,
    )
    try:
        room = scenes.room_shell()
        box = np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])
        memory.add_points("mid360", room)
        memory.add_points("jt128", room)
        memory.close_window(stamp(0))
        real = _Table.insert_window

        def explode(self, ix, iy, iz, counts):
            if self is memory._rhythm.accumulator._sensors["mid360"]["table"]:
                raise RuntimeError("insert failed")
            return real(self, ix, iy, iz, counts)

        monkeypatch.setattr(_Table, "insert_window", explode)
        memory.add_points("mid360", box)
        memory.add_points("jt128", box)
        events = memory.close_window(stamp(1))
        assert any(event.sensors == 1 for event in events)
        assert any(event.sensors == 2 for event in events)
        assert memory._rhythm.accumulator._sensors["mid360"]["bin"] is None
        assert memory._rhythm.accumulator._sensors["mid360"]["loss"] is not None
        # The reference window and this window both had points, so the bin holds two.
        assert memory._rhythm.accumulator._sensors["jt128"]["bin"].with_points == 2
        memory.close()
        mid_path = tmp_path / "lab" / "rhythm" / f"rhythm-mid360-{time.strftime('%Y%m%d', time.gmtime(START.sec))}.rhy"
        jt_path = tmp_path / "lab" / "rhythm" / f"rhythm-jt128-{time.strftime('%Y%m%d', time.gmtime(START.sec))}.rhy"
        _, mid_records, _, _ = load_rhy(mid_path)
        _, jt_records, _, _ = load_rhy(jt_path)
        assert any(record["flags"] & FLAG_LOST for record in mid_records)
        assert jt_records and jt_records[-1]["with_points"] >= 1 and not jt_records[-1]["flags"] & FLAG_LOST
    finally:
        memory.close()


def test_rhythm_on_and_off_match_event_map_and_status_bytes(tmp_path):
    def drive(path, enable):
        memory = RoomMemory(config(
            path / "rhythm", state_dir=str(path / "state"), rhythm_enable=enable,
            rhythm_dir=str(path / "rhythm") if enable else "", reference_windows=1, max_range=None,
        ), START, GENERATION)
        try:
            room = scenes.room_shell()
            changed = np.concatenate([room, scenes.box_shell((0.3, 1.2, 0.3))])
            memory.add_points("mid360", room)
            first = memory.close_window(stamp(0))
            memory.add_points("mid360", changed)
            second = memory.close_window(stamp(1))
            memory.enqueue_rhythm()
            status = memory.status(stamp(2), 1.0)
            maps = [memory.map_update(stamp(0)), memory.map_update(stamp(2))]
            reference = reference_path(path / "state", "lab", "mid360").read_bytes()
            events = [event.to_cdr() for event in first + second]
            return events, maps, status.to_cdr(), reference
        finally:
            memory.close()

    assert drive(tmp_path / "on", True) == drive(tmp_path / "off", False)


def _percent(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def _close_samples(path, enable, points, count):
    """Time steady windows. The period is one hour so a minute seal is not in the sample."""
    memory = RoomMemory(config(
        path, state_dir=None, rhythm_enable=enable, rhythm_dir="" if not enable else str(path),
        rhythm_publish=False, rhythm_period_s=3600, reference_windows=1, max_range=None,
        sensors=("mid360",),
    ), START, GENERATION)
    try:
        memory.add_points("mid360", points)
        memory.close_window(stamp(0))
        for warm in range(3):
            memory.add_points("mid360", points)
            memory.close_window(stamp(1 + warm))
            memory.enqueue_rhythm()
        walls, cpus = [], []
        for index in range(count):
            memory.add_points("mid360", points)
            started = time.perf_counter()
            cpu = time.process_time()
            memory.close_window(stamp(10 + index))
            memory.enqueue_rhythm()
            walls.append((time.perf_counter() - started) * 1000)
            cpus.append((time.process_time() - cpu) * 1000)
        return walls, cpus
    finally:
        memory.close()


def _assert_within_allowance(root, points, trials=5):
    """Enabled median wall, p99 wall, and p99 CPU against the disabled median wall.

    A scheduler stall is larger than the allowance and shows up in the disabled run too.
    The first quiet trial is enough. An overrun that is actually the logger fails every trial.
    """
    seen = []
    for trial in range(trials):
        off_wall, off_cpu = _close_samples(root / f"off{trial}", False, points, 100)
        on_wall, on_cpu = _close_samples(root / f"on{trial}", True, points, 100)
        off_med = _percent(off_wall, 0.5)
        allowance = max(1.0, 0.10 * off_med)
        deltas = (
            _percent(on_wall, 0.5) - off_med,
            _percent(on_wall, 0.99) - off_med,
            _percent(on_cpu, 0.99) - _percent(off_cpu, 0.5),
        )
        seen.append((tuple(round(delta, 3) for delta in deltas), round(allowance, 3)))
        if all(delta <= allowance for delta in deltas):
            return
    median, tail, cpu = seen[-1][0]
    allowance = seen[-1][1]
    assert median <= allowance and tail <= allowance and cpu <= allowance, seen


def test_enabled_callback_stays_within_the_allowance(tmp_path):
    _assert_within_allowance(tmp_path, scenes.room_shell())


@pytest.mark.skipif(
    os.environ.get("ROOM_MEMORY_RHYTHM_BUDGET") != "1",
    reason="set ROOM_MEMORY_RHYTHM_BUDGET=1 to check the allowance across a rotation",
)
def test_allowance_holds_across_a_rotation(tmp_path, rhythm):
    _assert_within_allowance(tmp_path / "time", scenes.room_shell())

    directory = tmp_path / "cap"
    log = rhythm(path=directory, rhythm_max_bytes=65536, rhythm_max_file_bytes=65536)
    wide = pack(np.arange(4096), np.zeros(4096, dtype=np.int64), np.zeros(4096, dtype=np.int64))
    for minute in (1_700_000_040, 1_700_000_100, 1_700_000_160):
        log.add("mid360", minute, wide, 1)
        log.enqueue()
        assert log.writer.wait_idle(2)
    log.close()
    files = rhy_files(directory)
    assert sum(path.stat().st_size for path in files) <= 65536
    assert log.rhythm_budget_dropped == 0
