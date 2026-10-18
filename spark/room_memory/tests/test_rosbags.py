"""``RigPose`` and the D51 name-object types against an independent ROS 2 CDR implementation:
rosbags, with the types built from the ``.msg`` files and both halves of ``NameObject.srv``.
The hand goldens decode to their values and re-serialize byte for byte, the Python codec and
rosbags encode the same values to the same bytes, and a rosbag2 written and read back by rosbags
carries them unchanged. Skipped without rosbags (the gate's venv has none)."""

import inspect
import itertools
import math
import pathlib
from dataclasses import replace

import numpy as np
import pytest

pytest.importorskip(
    "rosbags", reason="rosbags cross-checks the RigPose and D51 name-object CDR (pip install rosbags)",
)

from rosbags.rosbag2 import Reader, Writer  # noqa: E402
from rosbags.typesys import Stores, get_types_from_msg, get_typestore  # noqa: E402

from room_memory import messages as m  # noqa: E402
from test_messages import (  # noqa: E402
    G, hexfile, name_object_request, name_object_response, object_registry, rig_pose,
)

MSG = pathlib.Path(__file__).resolve().parents[3] / "ros" / "labcommand_msgs" / "msg" / "RigPose.msg"
TYPE = "labcommand_msgs/msg/RigPose"
TOPIC = "/room_memory/rig"
OBJECTS_TYPE = "labcommand_msgs/msg/ObjectRegistry"
REQUEST_TYPE = "labcommand_msgs/msg/NameObjectRequest"
RESPONSE_TYPE = "labcommand_msgs/msg/NameObjectResponse"
OBJECTS_TOPIC = "/room_memory/objects"
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


# Built on first use so a broken D51 definition does not fail the RigPose tests at import.
_D51_STORE = None
D51_MESSAGES = ("ObjectLink", "RecoveredAccept", "NameableSeed", "RegistryObject", "ObjectRegistry")


def d51_store():
    """The five D51 messages and both ``NameObject`` halves, registered in one store.

    Rosbags parses ``.msg`` text. The service is split on its first ``---`` and registered under
    ``labcommand_msgs/msg/``: a name whose parent is ``srv`` gains an extra ``msg`` segment.
    These types are v2-only, so nothing strips ``publisher_generation``. Nested types have to
    share the store; ``ObjectRegistry`` alone raises ``KeyError`` at deserialize.
    """
    global _D51_STORE
    if _D51_STORE is not None:
        return _D51_STORE
    package = MSG.parents[1]
    types = {}
    for name in D51_MESSAGES:
        text = (package / "msg" / f"{name}.msg").read_text()
        types.update(get_types_from_msg(text, f"labcommand_msgs/msg/{name}"))
    request, response = (package / "srv" / "NameObject.srv").read_text().split("---", 1)
    types.update(get_types_from_msg(request, REQUEST_TYPE))
    types.update(get_types_from_msg(response, RESPONSE_TYPE))
    store = get_typestore(STORE)
    store.register(types)
    _D51_STORE = store
    return store


def wire_f32(values, count):
    return np.array([math.nan] * count if values is None else values, dtype=np.float32)


def finite_box(values):
    """All-NaN geometry is None. A partial NaN is not a value this file checks."""
    return None if all(math.isnan(float(item)) for item in values) else tuple(float(item) for item in values)


def header_of(msg):
    stamp = msg.stamp
    return m.Header(msg.room_id, int(msg.map_epoch), int(msg.seq), m.Time(int(stamp.sec), int(stamp.nanosec)))


def stamp_of(store, stamp):
    return store.types["builtin_interfaces/msg/Time"](sec=stamp.sec, nanosec=stamp.nanosec)


def registry_from(msg):
    def link(item):
        return m.ObjectLink(
            int(item.event_generation), int(item.event_id), m.EventKind(int(item.kind)), bool(item.recovered),
        )

    def obj(item):
        return m.RegistryObject(
            int(item.object_id), int(item.revision), item.name, m.Baseline(int(item.baseline)),
            m.Association(int(item.association)), int(item.association_generation), int(item.association_event_id),
            int(item.sensor), int(item.frame_epoch), finite_box(item.centroid), finite_box(item.aabb),
            tuple(link(entry) for entry in item.links),
        )

    def seed(item):
        return m.NameableSeed(
            int(item.event_generation), int(item.event_id), int(item.sensor),
            tuple(float(value) for value in item.centroid), tuple(float(value) for value in item.aabb),
            int(item.voxel_count),
        )

    def receipt(item):
        return m.RecoveredAccept(int(item.event_generation), int(item.event_id), m.EventKind(int(item.kind)))

    return m.ObjectRegistry(
        header_of(msg), int(msg.snapshot_seq), tuple(obj(item) for item in msg.objects),
        tuple(seed(item) for item in msg.seeds), tuple(msg.recent_action_ids),
        tuple(receipt(item) for item in msg.recovered_accepts), int(msg.publisher_generation),
    )


def request_from(msg):
    return m.NameObjectRequest(
        header_of(msg), int(msg.object_id), int(msg.expected_revision), int(msg.event_generation),
        int(msg.event_id), msg.name, msg.operator_id, msg.action_id,
    )


def response_from(msg):
    return m.NameObjectResponse(
        m.NameOutcome(int(msg.outcome)), int(msg.object_id), int(msg.revision), bool(msg.repeat_differs),
    )


def from_d51(msg):
    """The rosbags message as the codec's value."""
    store = d51_store()
    return {
        store.types[OBJECTS_TYPE]: registry_from,
        store.types[REQUEST_TYPE]: request_from,
        store.types[RESPONSE_TYPE]: response_from,
    }[type(msg)](msg)


def registry_to(store, value):
    def link(item):
        return store.types["labcommand_msgs/msg/ObjectLink"](
            event_generation=item.event_generation, event_id=item.event_id, kind=int(item.kind),
            recovered=item.recovered,
        )

    def obj(item):
        return store.types["labcommand_msgs/msg/RegistryObject"](
            object_id=item.object_id, revision=item.revision, name=item.name, baseline=int(item.baseline),
            association=int(item.association), association_generation=item.association_generation,
            association_event_id=item.association_event_id, sensor=item.sensor, frame_epoch=item.frame_epoch,
            centroid=wire_f32(item.centroid, 3), aabb=wire_f32(item.aabb, 6),
            links=[link(entry) for entry in item.links],
        )

    def seed(item):
        return store.types["labcommand_msgs/msg/NameableSeed"](
            event_generation=item.event_generation, event_id=item.event_id, sensor=item.sensor,
            centroid=wire_f32(item.centroid, 3), aabb=wire_f32(item.aabb, 6), voxel_count=item.voxel_count,
        )

    def receipt(item):
        return store.types["labcommand_msgs/msg/RecoveredAccept"](
            event_generation=item.event_generation, event_id=item.event_id, kind=int(item.kind),
        )

    header = value.header
    return store.types[OBJECTS_TYPE](
        room_id=header.room_id, map_epoch=header.map_epoch, seq=header.seq, stamp=stamp_of(store, header.stamp),
        snapshot_seq=value.snapshot_seq, objects=[obj(item) for item in value.objects],
        seeds=[seed(item) for item in value.seeds], recent_action_ids=list(value.recent_action_ids),
        recovered_accepts=[receipt(item) for item in value.recovered_accepts],
        publisher_generation=value.publisher_generation,
    )


def request_to(store, value):
    header = value.header
    return store.types[REQUEST_TYPE](
        room_id=header.room_id, map_epoch=header.map_epoch, seq=header.seq, stamp=stamp_of(store, header.stamp),
        object_id=value.object_id, expected_revision=value.expected_revision,
        event_generation=value.event_generation, event_id=value.event_id, name=value.name,
        operator_id=value.operator_id, action_id=value.action_id,
    )


def response_to(store, value):
    return store.types[RESPONSE_TYPE](
        outcome=int(value.outcome), object_id=value.object_id, revision=value.revision,
        repeat_differs=value.repeat_differs,
    )


def to_d51(value):
    """The codec's value as a rosbags message, built field by field (not from the codec's bytes)."""
    store = d51_store()
    return {
        m.ObjectRegistry: registry_to,
        m.NameObjectRequest: request_to,
        m.NameObjectResponse: response_to,
    }[type(value)](store, value)


def d51_typename(value):
    return {
        m.ObjectRegistry: OBJECTS_TYPE,
        m.NameObjectRequest: REQUEST_TYPE,
        m.NameObjectResponse: RESPONSE_TYPE,
    }[type(value)]


D51_GOLDENS = {
    "object_registry_le.hex": (m.ObjectRegistry, object_registry, OBJECTS_TYPE),
    "object_registry_be.hex": (m.ObjectRegistry, object_registry, OBJECTS_TYPE),
    "name_object_le.hex": (m.NameObjectRequest, name_object_request, REQUEST_TYPE),
    "name_object_be.hex": (m.NameObjectRequest, name_object_request, REQUEST_TYPE),
    "name_object_response_le.hex": (m.NameObjectResponse, name_object_response, RESPONSE_TYPE),
    "name_object_response_be.hex": (m.NameObjectResponse, name_object_response, RESPONSE_TYPE),
}


def d51_cases():
    """Room ids, object names, and action ids of every length mod 8, plus the odd shapes.

    The room id shifts the header. ``RegistryObject.name`` is followed by two ``uint8``s and then
    ``uint64``s, and ``recent_action_ids`` is followed by ``RecoveredAccept`` ``uint64``s, so those
    two strings are swept on their own. The sweeps are not a product: each extra byte walks the
    fields after that string through every alignment.
    """
    registry = object_registry()
    request = name_object_request()
    obj = registry.objects[0]
    response = name_object_response()
    for length in range(1, 9):
        room = "lab-room"[:length]
        yield f"empty-{room}", replace(
            registry, header=replace(registry.header, room_id=room), snapshot_seq=0, objects=(), seeds=(),
            recent_action_ids=(), recovered_accepts=(),
        )
        yield f"create-{room}", replace(request, header=replace(request.header, room_id=room))
    yield "registry", registry
    yield "gone", replace(
        registry, objects=(replace(obj, baseline=m.Baseline.GONE, frame_epoch=0, centroid=None, aabb=None),),
    )
    yield "rename", replace(request, object_id=7, expected_revision=2, event_generation=0, event_id=0)
    yield "repeat-same", response
    yield "repeat-differs", replace(response, repeat_differs=True)
    for length in range(1, 9):
        yield f"name-{length}", replace(registry, objects=(replace(obj, name="n" * length),))
        yield f"action-{length}", replace(registry, recent_action_ids=("a" * length,))


D51_CASES = list(d51_cases())


@pytest.mark.parametrize("name", sorted(D51_GOLDENS))
def test_rosbags_decodes_the_d51_golden_and_reserializes_it(name):
    kind, build, typename = D51_GOLDENS[name]
    golden = hexfile(name)
    store = d51_store()
    msg = store.deserialize_cdr(golden, typename)
    assert from_d51(msg) == build() == kind.from_cdr(golden)
    little = not name.endswith("_be.hex")
    assert bytes(store.serialize_cdr(msg, typename, little_endian=little)) == golden


@pytest.mark.parametrize("value", [value for _, value in D51_CASES], ids=[label for label, _ in D51_CASES])
def test_d51_codec_and_rosbags_agree(value):
    store = d51_store()
    typename = d51_typename(value)
    ours = value.to_cdr()
    theirs = bytes(store.serialize_cdr(to_d51(value), typename))
    assert ours == theirs
    assert from_d51(store.deserialize_cdr(ours, typename)) == value == type(value).from_cdr(theirs)


def test_object_registry_rosbag2_round_trip(tmp_path):
    """``NameObject`` is a service. Only the registry topic is written into a bag."""
    golden = hexfile("object_registry_le.hex")
    path = tmp_path / "bag"
    store = d51_store()
    with Writer(path, **WRITER_ARGS) as writer:
        connection = writer.add_connection(OBJECTS_TOPIC, OBJECTS_TYPE, typestore=store)
        writer.write(connection, 1_760_400_010_250_000_000, golden)
    with Reader(path) as reader:
        assert [(c.topic, c.msgtype) for c in reader.connections] == [(OBJECTS_TOPIC, OBJECTS_TYPE)]
        read = [(timestamp, bytes(raw)) for _, timestamp, raw in reader.messages()]
    assert read == [(1_760_400_010_250_000_000, golden)]
    assert from_d51(store.deserialize_cdr(read[0][1], OBJECTS_TYPE)) == m.ObjectRegistry.from_cdr(read[0][1])
