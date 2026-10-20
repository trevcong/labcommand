"""``RigPose`` and the D51 messages (``ObjectRegistry`` with ``RegistryObject``, ``ObjectLink``,
``NameableSeed`` and ``RecoveredAccept``, and both halves of ``NameObject``) against an independent
ROS 2 CDR implementation: rosbags, with the types built from the ``.msg`` and ``.srv`` files. The
hand goldens decode to their values and re-serialize byte for byte, a golden with any byte flipped
fails that check, the Python codec and rosbags encode the same values to the same bytes, and a
rosbag2 written and read back by rosbags carries the goldens unchanged. Skipped without rosbags
(the gate's venv has none); ``scripts/check-rosbags.sh`` fails instead of skipping (issue #73)."""

import inspect
import itertools
import math
import pathlib
from collections import namedtuple
from dataclasses import replace

import numpy as np
import pytest

pytest.importorskip("rosbags", reason="rosbags cross-checks the RigPose and D51 CDR (pip install rosbags)")

from rosbags.rosbag2 import Reader, Writer  # noqa: E402
from rosbags.typesys import Stores, get_types_from_msg, get_typestore  # noqa: E402

from room_memory import messages as m  # noqa: E402
from test_messages import (  # noqa: E402
    G, hexfile, name_object_request, name_object_response, object_registry, rig_pose,
)

ROS = pathlib.Path(__file__).resolve().parents[3] / "ros" / "labcommand_msgs"
MSG = ROS / "msg" / "RigPose.msg"
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




D51_MSGS = ("ObjectLink", "RecoveredAccept", "NameableSeed", "RegistryObject", "ObjectRegistry")


def d51_typestore():
    """A store with the D51 types from their .msg files and the two halves of ``NameObject.srv``,
    and the name rosbags gives each. rosbags files the halves under ``labcommand_msgs/srv/msg/``,
    so those names are the ones it returns."""
    store = get_typestore(STORE)
    names = {}
    request, response = (ROS / "srv" / "NameObject.srv").read_text().split("\n---\n")
    sources = [((ROS / "msg" / f"{name}.msg").read_text(), name, f"labcommand_msgs/msg/{name}") for name in D51_MSGS]
    sources += [(request, "NameObjectRequest", "labcommand_msgs/srv/NameObject_Request"),
                (response, "NameObjectResponse", "labcommand_msgs/srv/NameObject_Response")]
    types = {}
    for text, name, typename in sources:
        found = get_types_from_msg(text, typename)
        (names[name],) = found
        types.update(found)
    store.register(types)
    return store, names


D51, D51_NAMES = d51_typestore()


def d51(name, /, **fields):
    return D51.types[D51_NAMES[name]](**fields)


def header_from(msg):
    return m.Header(msg.room_id, int(msg.map_epoch), int(msg.seq), m.Time(int(msg.stamp.sec), int(msg.stamp.nanosec)))


def header_to(header):
    stamp = D51.types["builtin_interfaces/msg/Time"](sec=header.stamp.sec, nanosec=header.stamp.nanosec)
    return dict(room_id=header.room_id, map_epoch=header.map_epoch, seq=header.seq, stamp=stamp)


def floats(values):
    return tuple(float(v) for v in values)


def optional_floats(values):
    return None if all(math.isnan(v) for v in values) else floats(values)


def f32(values, n):
    return np.array([math.nan] * n if values is None else values, dtype=np.float32)


def registry_from_rosbags(msg):
    """The rosbags ``ObjectRegistry`` as the codec's value: NaN geometry is None."""
    def link(item):
        return m.ObjectLink(int(item.event_generation), int(item.event_id), m.EventKind(item.kind),
                            bool(item.recovered))

    def obj(item):
        return m.RegistryObject(
            int(item.object_id), int(item.revision), item.name, m.Baseline(item.baseline),
            m.Association(item.association), int(item.association_generation), int(item.association_event_id),
            int(item.sensor), int(item.frame_epoch), optional_floats(item.centroid), optional_floats(item.aabb),
            tuple(link(entry) for entry in item.links),
        )

    def seed(item):
        return m.NameableSeed(int(item.event_generation), int(item.event_id), int(item.sensor), floats(item.centroid),
                              floats(item.aabb), int(item.voxel_count))

    def receipt(item):
        return m.RecoveredAccept(int(item.event_generation), int(item.event_id), m.EventKind(item.kind))

    return m.ObjectRegistry(
        header_from(msg), int(msg.snapshot_seq), tuple(obj(item) for item in msg.objects),
        tuple(seed(item) for item in msg.seeds), tuple(msg.recent_action_ids),
        tuple(receipt(item) for item in msg.recovered_accepts), int(msg.publisher_generation),
    )


def registry_to_rosbags(value):
    """The codec's ``ObjectRegistry`` as a rosbags message, built field by field."""
    def obj(item):
        links = [d51("ObjectLink", event_generation=link.event_generation, event_id=link.event_id, kind=int(link.kind),
                     recovered=link.recovered) for link in item.links]
        return d51("RegistryObject", object_id=item.object_id, revision=item.revision, name=item.name,
                   baseline=int(item.baseline), association=int(item.association),
                   association_generation=item.association_generation, association_event_id=item.association_event_id,
                   sensor=item.sensor, frame_epoch=item.frame_epoch, centroid=f32(item.centroid, 3),
                   aabb=f32(item.aabb, 6), links=links)

    seeds = [d51("NameableSeed", event_generation=seed.event_generation, event_id=seed.event_id, sensor=seed.sensor,
                 centroid=f32(seed.centroid, 3), aabb=f32(seed.aabb, 6), voxel_count=seed.voxel_count)
             for seed in value.seeds]
    receipts = [d51("RecoveredAccept", event_generation=receipt.event_generation, event_id=receipt.event_id,
                    kind=int(receipt.kind)) for receipt in value.recovered_accepts]
    return d51("ObjectRegistry", **header_to(value.header), snapshot_seq=value.snapshot_seq,
               objects=[obj(item) for item in value.objects], seeds=seeds,
               recent_action_ids=list(value.recent_action_ids), recovered_accepts=receipts,
               publisher_generation=value.publisher_generation)


def request_from_rosbags(msg):
    return m.NameObjectRequest(header_from(msg), int(msg.object_id), int(msg.expected_revision),
                               int(msg.event_generation), int(msg.event_id), msg.name, msg.operator_id, msg.action_id)


def request_to_rosbags(value):
    return d51("NameObjectRequest", **header_to(value.header), object_id=value.object_id,
               expected_revision=value.expected_revision, event_generation=value.event_generation,
               event_id=value.event_id, name=value.name, operator_id=value.operator_id, action_id=value.action_id)


def response_from_rosbags(msg):
    return m.NameObjectResponse(m.NameOutcome(msg.outcome), int(msg.object_id), int(msg.revision),
                                bool(msg.repeat_differs))


def response_to_rosbags(value):
    return d51("NameObjectResponse", outcome=int(value.outcome), object_id=value.object_id, revision=value.revision,
               repeat_differs=value.repeat_differs)


def d51_variants():
    """``(id, value)``: each D51 type varied inside the one message that carries it, strings of
    every length mod 8 (a string sets the alignment of all that follows), and empty sequences."""
    base = object_registry()
    (obj,), (seed,), (receipt,) = base.objects, base.seeds, base.recovered_accepts
    (link,) = obj.links
    for length in range(1, 9):
        yield f"ObjectRegistry-room-{length}", replace(base, header=replace(base.header, room_id="lab-room"[:length]))
        yield f"ObjectRegistry-actions-{length}", replace(base, recent_action_ids=("act-1", "abcdefgh"[:length]))
        yield f"RegistryObject-name-{length}", replace(base, objects=(replace(obj, name="Scope ca"[:length]),))
    yield "ObjectRegistry-empty", replace(base, objects=(), seeds=(), recent_action_ids=(), recovered_accepts=())
    yield "RegistryObject-utf8-name", replace(base, objects=(replace(obj, name="Kühlschrank ✓"),))
    gone = replace(obj, baseline=m.Baseline.GONE, association=m.Association.NONE, association_generation=0,
                   association_event_id=0, frame_epoch=0, centroid=None, aabb=None, links=())
    yield "RegistryObject-gone", replace(base, objects=(gone,))
    detached = replace(obj, object_id=8, revision=1, baseline=m.Baseline.DETACHED,
                       association=m.Association.UNRESOLVED, association_generation=1, sensor=m.SENSOR_JT128)
    yield "RegistryObject-several", replace(base, objects=(obj, detached, replace(gone, object_id=9),
                                                           replace(obj, object_id=10, baseline=m.Baseline.RETIRED)))
    yield "ObjectLink-none", replace(base, objects=(replace(obj, links=()),))
    links = (link, m.ObjectLink(G, 12, m.EventKind.APPEARED, False),
             m.ObjectLink(1, 13, m.EventKind.DISAPPEARED, False))
    yield "ObjectLink-several", replace(base, objects=(replace(obj, links=links),))
    yield "NameableSeed-none", replace(base, seeds=())
    jt128 = m.NameableSeed(1, 14, m.SENSOR_JT128, (-0.5, 0.25, 1.0), (-0.75, 0.0, 0.875, -0.25, 0.5, 1.125), 1)
    yield "NameableSeed-several", replace(base, seeds=(seed, jt128))
    yield "RecoveredAccept-none", replace(base, recovered_accepts=())
    yield "RecoveredAccept-every-kind", replace(
        base, recovered_accepts=tuple(m.RecoveredAccept(G, 20 + kind, kind) for kind in m.EventKind))
    request = name_object_request()
    for length in range(1, 9):
        yield f"NameObjectRequest-room-{length}", replace(request, header=replace(request.header,
                                                                                  room_id="lab-room"[:length]))
        yield f"NameObjectRequest-name-{length}", replace(
            request, name="Scope ca"[:length], operator_id="operator"[:length], action_id="act-1234"[:length])
    yield "NameObjectRequest-rename", replace(request, object_id=7, expected_revision=2, event_generation=0, event_id=0)
    yield "NameObjectRequest-attach", replace(request, object_id=7, expected_revision=2)
    for outcome, differs in itertools.product(m.NameOutcome, (False, True)):
        yield f"NameObjectResponse-{outcome.name}-{differs}", m.NameObjectResponse(outcome, 7, 1, differs)


TO_ROSBAGS = {
    m.ObjectRegistry: ("ObjectRegistry", registry_to_rosbags, registry_from_rosbags),
    m.NameObjectRequest: ("NameObjectRequest", request_to_rosbags, request_from_rosbags),
    m.NameObjectResponse: ("NameObjectResponse", response_to_rosbags, response_from_rosbags),
}

# store, rosbags type, rosbag2 topic, rosbags message -> codec value, codec class, the golden's value
Golden = namedtuple("Golden", "store type topic from_rosbags codec value")
GOLDENS = {
    "rig_pose_le.hex": Golden(STORES[False], TYPE, TOPIC, lambda msg: from_rosbags(msg, False), m.RigPose, rig_pose),
    "rig_pose_v2_le.hex": Golden(STORES[True], TYPE, TOPIC, lambda msg: from_rosbags(msg, True), m.RigPose,
                                 lambda: replace(rig_pose(), publisher_generation=G)),
}
for _order in ("le", "be"):
    GOLDENS.update({
        f"object_registry_{_order}.hex": Golden(D51, D51_NAMES["ObjectRegistry"], "/room_memory/objects",
                                                registry_from_rosbags, m.ObjectRegistry, object_registry),
        f"name_object_{_order}.hex": Golden(D51, D51_NAMES["NameObjectRequest"], "/room_memory/objects/name",
                                            request_from_rosbags, m.NameObjectRequest, name_object_request),
        f"name_object_response_{_order}.hex": Golden(D51, D51_NAMES["NameObjectResponse"], "/room_memory/objects/name",
                                                     response_from_rosbags, m.NameObjectResponse, name_object_response),
    })


def check_golden(name, data):
    """``data`` stands for the golden ``name``: rosbags decodes it to the golden's value, as the codec
    does, re-serializes it byte for byte in the golden's byte order, and in little endian gives the
    codec's encoding of that value (which also pins NaN payloads the value cannot see)."""
    golden = GOLDENS[name]
    msg = golden.store.deserialize_cdr(data, golden.type)
    value = golden.value()
    assert golden.from_rosbags(msg) == value == golden.codec.from_cdr(data)
    little = name.endswith("_le.hex")
    assert bytes(golden.store.serialize_cdr(msg, golden.type, little_endian=little)) == data
    assert bytes(golden.store.serialize_cdr(msg, golden.type)) == value.to_cdr()


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_rosbags_decodes_the_golden_and_reserializes_it(name):
    golden = hexfile(name)
    check_golden(name, golden)
    if name == "rig_pose_v2_le.hex":
        assert STORES[True].deserialize_cdr(golden, TYPE).publisher_generation == G


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_a_golden_with_any_byte_flipped_fails_the_check(name):
    """Every byte of the golden, flipped in memory three ways (low bit, high bit, all bits), fails
    ``check_golden``: by an assertion or by a decode error."""
    golden = hexfile(name)
    check_golden(name, golden)
    survivors = []
    for index, flip in itertools.product(range(len(golden)), (0x01, 0x80, 0xFF)):
        corrupt = bytearray(golden)
        corrupt[index] ^= flip
        try:
            check_golden(name, bytes(corrupt))
        except Exception:  # noqa: BLE001 - any failure is the check catching the corruption
            continue
        survivors.append((index, flip))
    assert survivors == []


@pytest.mark.parametrize("value", list(variants()), ids=lambda v: f"{v.header.room_id}-{v.publisher_generation}"
                         f"-{'missing' if v.tilt_deg is None else 'full'}")
def test_codec_and_rosbags_agree(value):
    v2 = value.publisher_generation is not None
    ours = value.to_cdr()
    theirs = bytes(STORES[v2].serialize_cdr(to_rosbags(value), TYPE))
    assert ours == theirs
    assert from_rosbags(STORES[v2].deserialize_cdr(ours, TYPE), v2) == value == m.RigPose.from_cdr(theirs)


D51_VARIANTS = list(d51_variants())


@pytest.mark.parametrize("value", [value for _, value in D51_VARIANTS], ids=[case for case, _ in D51_VARIANTS])
def test_d51_codec_and_rosbags_agree(value):
    name, to_rosbags_d51, from_rosbags_d51 = TO_ROSBAGS[type(value)]
    ours = value.to_cdr()
    theirs = bytes(D51.serialize_cdr(to_rosbags_d51(value), D51_NAMES[name]))
    assert ours == theirs
    assert from_rosbags_d51(D51.deserialize_cdr(ours, D51_NAMES[name])) == value == type(value).from_cdr(theirs)


def test_d51_variants_cover_every_d51_type():
    covered = {case.split("-", 1)[0] for case, _ in D51_VARIANTS}
    assert covered == set(D51_MSGS) | {"NameObjectRequest", "NameObjectResponse"}
    assert len({case for case, _ in D51_VARIANTS}) == len(D51_VARIANTS)


@pytest.mark.parametrize("name", sorted(GOLDENS))
def test_rosbag2_round_trip(tmp_path, name):
    """Written to a rosbag2 by rosbags and read back: the same bytes, decoded alike by both."""
    golden = GOLDENS[name]
    data = hexfile(name)
    path = tmp_path / "bag"
    with Writer(path, **WRITER_ARGS) as writer:
        connection = writer.add_connection(golden.topic, golden.type, typestore=golden.store)
        writer.write(connection, 1_760_400_010_250_000_000, data)
    with Reader(path) as reader:
        assert [(c.topic, c.msgtype) for c in reader.connections] == [(golden.topic, golden.type)]
        read = [(timestamp, bytes(raw)) for _, timestamp, raw in reader.messages()]
    assert read == [(1_760_400_010_250_000_000, data)]
    raw = read[0][1]
    assert golden.from_rosbags(golden.store.deserialize_cdr(raw, golden.type)) == golden.codec.from_cdr(raw)
