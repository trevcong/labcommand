"""``RigPose`` against an independent ROS 2 CDR implementation: rosbags, with the type built from
``RigPose.msg``. The hand goldens decode to their values and re-serialize byte for byte, the
Python codec and rosbags encode the same values to the same bytes, and a rosbag2 written and read
back by rosbags carries them unchanged. Skipped without rosbags (the gate's venv has none)."""

import inspect
import itertools
import math
import pathlib
from dataclasses import replace

import numpy as np
import pytest

pytest.importorskip("rosbags", reason="rosbags cross-checks the RigPose CDR (pip install rosbags)")

from rosbags.rosbag2 import Reader, Writer  # noqa: E402
from rosbags.typesys import Stores, get_types_from_msg, get_typestore  # noqa: E402

from room_memory import messages as m  # noqa: E402
from test_messages import G, hexfile, rig_pose  # noqa: E402

MSG = pathlib.Path(__file__).resolve().parents[3] / "ros" / "labcommand_msgs" / "msg" / "RigPose.msg"
TYPE = "labcommand_msgs/msg/RigPose"
TOPIC = "/room_memory/rig"
GENERATION = "uint64 publisher_generation"
# builtin_interfaces/Time, the one type RigPose uses, is the same in every ROS 2 store; older
# rosbags have no Jazzy one.
STORE = getattr(Stores, "ROS2_JAZZY", Stores.ROS2_HUMBLE)
# rosbags 0.10 and later require the rosbag2 format version (8 is Humble's and Jazzy's); 0.9 writes
# version 8 and takes no such argument.
WRITER_ARGS = {"version": 8} if "version" in inspect.signature(Writer).parameters else {}


def typestore(v2):
    """A store with RigPose from the .msg; the legacy layout is the same text without its tail."""
    text = MSG.read_text()
    assert GENERATION in text
    if not v2:
        text = "\n".join(line for line in text.splitlines() if not line.startswith(GENERATION))
    store = get_typestore(STORE)
    store.register(get_types_from_msg(text, TYPE))
    return store


STORES = {True: typestore(True), False: typestore(False)}


def nan_none(value):
    return None if math.isnan(value) else float(value)


def from_rosbags(msg, v2):
    """The rosbags message as the codec's value: NaN is None, a NaN x 7 pose is None."""
    def time(t):
        return m.Time(int(t.sec), int(t.nanosec))

    def pose(values):
        return None if all(math.isnan(v) for v in values) else tuple(float(v) for v in values)

    def pair(values):
        return tuple(nan_none(v) for v in values)

    return m.RigPose(
        m.Header(msg.room_id, int(msg.map_epoch), int(msg.seq), time(msg.stamp)), m.RigOutcome(msg.outcome),
        int(msg.causes), int(msg.sensors), tuple(int(c) for c in msg.channel), int(msg.frame_epoch),
        int(msg.n_realigned), pose(msg.mid360_pose), pose(msg.jt128_pose), pair(msg.delta_m), pair(msg.delta_deg),
        pair(msg.inlier_frac), pair(msg.rms_m), pair(msg.observe), nan_none(msg.tilt_deg), bool(msg.imu_ok),
        time(msg.moved_at), int(msg.publisher_generation) if v2 else None,
    )


def to_rosbags(value):
    """The codec's value as a rosbags message, built field by field (not from the codec's bytes)."""
    v2 = value.publisher_generation is not None
    store = STORES[v2]
    time = store.types["builtin_interfaces/msg/Time"]

    def f32(values, n):
        return np.array([math.nan] * n if values is None else [math.nan if v is None else v for v in values],
                        dtype=np.float32)

    fields = dict(
        room_id=value.header.room_id, map_epoch=value.header.map_epoch, seq=value.header.seq,
        stamp=time(sec=value.header.stamp.sec, nanosec=value.header.stamp.nanosec), outcome=int(value.outcome),
        causes=value.causes, sensors=value.sensors, channel=np.array([int(c) for c in value.channel], dtype=np.uint8),
        frame_epoch=value.frame_epoch, n_realigned=value.n_realigned, mid360_pose=f32(value.mid360_pose, 7),
        jt128_pose=f32(value.jt128_pose, 7), delta_m=f32(value.delta_m, 2), delta_deg=f32(value.delta_deg, 2),
        inlier_frac=f32(value.inlier_frac, 2), rms_m=f32(value.rms_m, 2), observe=f32(value.observe, 2),
        tilt_deg=math.nan if value.tilt_deg is None else value.tilt_deg, imu_ok=value.imu_ok,
        moved_at=time(sec=value.moved_at.sec, nanosec=value.moved_at.nanosec),
    )
    if v2:
        fields["publisher_generation"] = value.publisher_generation
    return store.types[TYPE](**fields)


GOLDENS = {
    "rig_pose_le.hex": rig_pose,
    "rig_pose_v2_le.hex": lambda: replace(rig_pose(), publisher_generation=G),
}


def variants():
    """Room ids of every length mod 8 (the string sets the alignment of all that follows), each
    layout, and the missing values: no Mid-360 pose, no increments, no checks, no IMU."""
    base = rig_pose()
    missing = dict(mid360_pose=None, delta_m=(None, None), delta_deg=(None, None), inlier_frac=(None, None),
                   rms_m=(None, None), observe=(None, None), tilt_deg=None, imu_ok=False,
                   outcome=m.RigOutcome.MOVED, channel=(m.RigChannel.UNAVAILABLE, m.RigChannel.MOVED))
    for length, generation, absent in itertools.product(range(1, 9), (None, G, 1), (False, True)):
        value = replace(base, header=replace(base.header, room_id="lab-room"[:length]),
                        publisher_generation=generation)
        yield replace(value, **missing) if absent else value


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_rosbags_decodes_the_golden_and_reserializes_it(name):
    v2 = "_v2_" in name
    golden = hexfile(name)
    msg = STORES[v2].deserialize_cdr(golden, TYPE)
    assert from_rosbags(msg, v2) == GOLDENS[name]() == m.RigPose.from_cdr(golden)
    if v2:
        assert msg.publisher_generation == G
    assert bytes(STORES[v2].serialize_cdr(msg, TYPE)) == golden


@pytest.mark.parametrize("value", list(variants()), ids=lambda v: f"{v.header.room_id}-{v.publisher_generation}"
                         f"-{'missing' if v.tilt_deg is None else 'full'}")
def test_codec_and_rosbags_agree(value):
    v2 = value.publisher_generation is not None
    ours = value.to_cdr()
    theirs = bytes(STORES[v2].serialize_cdr(to_rosbags(value), TYPE))
    assert ours == theirs
    assert from_rosbags(STORES[v2].deserialize_cdr(ours, TYPE), v2) == value == m.RigPose.from_cdr(theirs)


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_rosbag2_round_trip(tmp_path, name):
    """Written to a rosbag2 by rosbags and read back: the same bytes, decoded alike by both."""
    v2 = "_v2_" in name
    golden = hexfile(name)
    path = tmp_path / "bag"
    with Writer(path, **WRITER_ARGS) as writer:
        connection = writer.add_connection(TOPIC, TYPE, typestore=STORES[v2])
        writer.write(connection, 1_760_400_010_250_000_000, golden)
    with Reader(path) as reader:
        assert [(c.topic, c.msgtype) for c in reader.connections] == [(TOPIC, TYPE)]
        read = [(timestamp, bytes(raw)) for _, timestamp, raw in reader.messages()]
    assert read == [(1_760_400_010_250_000_000, golden)]
    assert from_rosbags(STORES[v2].deserialize_cdr(read[0][1], TYPE), v2) == m.RigPose.from_cdr(read[0][1])
