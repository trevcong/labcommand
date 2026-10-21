"""Memory-v2 core evidence store (job V1). See docs/MEMORY-V2-DESIGN.md.

Covers §2.1 (transactions, known_at), §2.2 (retractions, transitive invalidation),
§2.3 (trajectory and map revisions), §2.5 rule 2 and §2.6 (immutability, purge
tombstones), under the §5 decisions. Status words (CURRENT, UNKNOWN, ...) and the
absence rules are V3, so these tests assert on ``evidence_for`` and
``compare_positions`` instead.
"""

from __future__ import annotations

import hashlib
import math
import sqlite3
from contextlib import contextmanager

import pytest

from spatio.memory.evidence import (
    MOVED_SIGMA_K,
    RETENTION_S,
    SCHEMA_VERSION,
    EvidenceStore,
    ValidationError,
    canonical_json,
    comparison_sigma,
    is_moved,
)

T0 = 1.76e9  # sensor time, as v1 DESIGN §9
W0 = 1.80e9  # wall time
NOW = T0 + 10
OPERATOR = "operator:trevor"
MODEL = "model:detector@1"
X_BOX = (1.9, -0.1, 0.0, 2.1, 0.1, 1.0)

TABLES = (
    "transactions",
    "sessions",
    "trajectory_revisions",
    "map_revisions",
    "observations",
    "interpretations",
    "retractions",
    "look_again_requests",
    "look_again_closures",
    "purged_observations",
    "purged_associations",
)


class Clock:
    """Injected wall clock. Inside ``reads_forbidden()`` any call fails the test."""

    def __init__(self, t: float = W0) -> None:
        self.t = t
        self._forbidden = False

    def __call__(self) -> float:
        assert not self._forbidden, "a query read the wall clock"
        return self.t

    @contextmanager
    def reads_forbidden(self):
        self._forbidden = True
        try:
            yield
        finally:
            self._forbidden = False


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def store(tmp_path, clock):
    s = EvidenceStore(tmp_path / "evidence.sqlite", wallclock=clock)
    yield s
    s.close()


def sha(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def add_session(store, sid, t_start=T0, t_end=T0 + 200):
    store.add_session(sid, t_start, t_end, source_recording_sha256=sha("raw-" + sid))


def add_traj(store, sid, revision, parent=None):
    return store.add_trajectory_revision(
        sid,
        revision,
        tum_sha256=sha(f"tum-{sid}-{revision}"),
        parent_revision_id=parent,
        method="fastlio" if revision == 1 else "loop_closure",
        method_version="1",
        params_sha256=sha("params"),
        reason="walk output" if revision == 1 else "loop closure",
    )


def add_map(store, map_id, revision, traj_id):
    return store.add_map_revision(
        map_id,
        revision,
        map_pcd_sha256=sha(f"pcd-{map_id}-{revision}"),
        trajectory_revision_id=traj_id,
        built_by="ingest",
        method="accumulate",
        method_version="1",
        params_sha256=sha("params"),
    )


def seed(store, sid="S1", t_start=T0, t_end=T0 + 200):
    """Session ``sid`` with trajectory revision 1 and map revision 1 built from it."""
    add_session(store, sid, t_start, t_end)
    t1 = add_traj(store, sid, 1)
    r1 = add_map(store, "M-" + sid, 1, t1)
    return t1, r1


def observe(store, sid, acquired_at, *, map_revision_id=None, object_id="X", label="chair",
            sensor_kind="lidar", aabb=X_BOX, sigma_m=0.03, **extra):
    """One valid detection plus a model interpretation, in the caller's transaction."""
    obs = store.add_observation(
        sid,
        kind="detection",
        sensor_kind=sensor_kind,
        geometry_frame="body" if map_revision_id is None else "map",
        map_revision_id=map_revision_id,
        acquired_at=acquired_at,
        aabb=aabb,
        sigma_m=sigma_m,
        measurement_confidence=0.9,
        **extra,
    )
    if object_id is not None:
        store.add_interpretation(obs, object_id, label, 0.9, source_kind="model", writer=MODEL)
    return obs


def detect(store, sid, acquired_at, **kw):
    with store.transaction("ingest", "detect"):
        return observe(store, sid, acquired_at, **kw)


def support(store, object_id, now=NOW, known_at=None):
    return [s["observation_id"] for s in store.evidence_for(object_id, now, known_at)["support"]]


def counts(store):
    conn = store.connection
    return {t: conn.execute(f"SELECT count(*) FROM {t}").fetchone()[0] for t in TABLES}


def box_at(x):
    return (x - 0.1, -0.1, 0.0, x + 0.1, 0.1, 1.0)


def interpretation_ids(store, observation_id):
    rows = store.connection.execute(
        "SELECT interpretation_id FROM interpretations WHERE observation_id = ? ORDER BY 1",
        (observation_id,),
    )
    return [r[0] for r in rows]


# 1 ---------------------------------------------------------------------------


def test_transactions_monotonic_wall_floor_and_known_at_wall(store, clock):
    assert store.connection.execute("PRAGMA user_version").fetchone()[0] == SCHEMA_VERSION == 2
    clock.t = W0 + 100
    add_session(store, "S1")
    clock.t = W0 + 50  # the wall clock steps back
    add_session(store, "S2")
    clock.t = W0 + 200
    add_session(store, "S3")
    rows = store.connection.execute(
        "SELECT txn_id, recorded_at, writer, operation, knowledge_basis FROM transactions ORDER BY txn_id"
    ).fetchall()
    assert [r[0] for r in rows] == [1, 2, 3]
    assert [r[1] for r in rows] == [W0 + 100, W0 + 100, W0 + 200]
    assert {r[4] for r in rows} == {"live"}
    assert [r[0] for r in store.connection.execute(
        "SELECT recorded_txn FROM sessions ORDER BY session_id")] == [1, 2, 3]

    with clock.reads_forbidden():
        assert store.latest_txn() == 3
        assert store.known_at_wall(W0 + 99) is None
        assert store.known_at_wall(W0 + 100) == 2
        assert store.known_at_wall(W0 + 150) == 2
        assert store.known_at_wall(W0 + 1e6) == 3
        assert store.evidence_for("X", NOW)["known_at"] == 3
        # A known_at in the future could change answer later, so it is refused.
        with pytest.raises(ValidationError):
            store.evidence_for("X", NOW, known_at=4)

    # A failed write leaves no transaction behind.
    before = counts(store)
    with pytest.raises(ValidationError):
        add_session(store, "S1")
    with pytest.raises(RuntimeError):
        with store.transaction("ingest", "aborted"):
            add_session(store, "S4")
            raise RuntimeError("abort")
    assert counts(store) == before
    with pytest.raises(ValidationError):
        with store.transaction("ingest", "x", knowledge_basis="guessed"):
            pass
    with store.transaction("ingest", "nest"):
        with pytest.raises(ValidationError):
            with store.transaction("ingest", "inner"):
                pass
    assert store.latest_txn() == 4


# 2 ---------------------------------------------------------------------------


def test_late_observation_visible_only_from_its_txn(store, clock):
    _, r1 = seed(store)
    d = detect(store, "S1", T0, map_revision_id=r1)
    k1 = store.latest_txn()
    with clock.reads_forbidden():
        before = canonical_json(store.evidence_for("X", NOW, k1))
        assert support(store, "X", known_at=k1) == [d]

    # Acquired after D, ingested later.
    e = detect(store, "S1", T0 + 5, map_revision_id=r1)
    k2 = store.latest_txn()
    with clock.reads_forbidden():
        assert canonical_json(store.evidence_for("X", NOW, k1)) == before
        at_k2 = store.evidence_for("X", NOW, k2)
        assert [s["observation_id"] for s in at_k2["support"]] == [e, d]
        assert at_k2["frame"] == r1
        assert at_k2["known_at"] == k2
        latest = store.evidence_for("X", NOW)
        assert canonical_json(latest) == canonical_json(at_k2)
        # The acquisition axis still filters on now.
        assert support(store, "X", now=T0 + 1, known_at=k2) == [d]
        assert support(store, "X", now=T0 - 1, known_at=k2) == []


# 3 ---------------------------------------------------------------------------


def test_rows_immutable_outside_purge(store, clock):
    clock.t = T0
    t1, r1 = seed(store)
    cam = detect(store, "S1", T0, sensor_kind="camera", map_revision_id=r1)
    lidar = detect(store, "S1", T0 + 1, map_revision_id=r1)
    keep_cam = detect(store, "S1", T0 + 100, sensor_kind="camera", map_revision_id=r1)
    store.retract("interpretation", interpretation_ids(store, lidar)[0], "wrong", OPERATOR)
    clock.t = T0 + 2
    req = store.request_look_again("X", writer=OPERATOR)
    fulfil = detect(store, "S1", T0 + 3, map_revision_id=r1)
    store.fulfil_look_again(req, fulfil, writer=OPERATOR)
    clock.t = T0 + 50 + RETENTION_S
    assert store.purge_expired(T0 + 50 + RETENTION_S) == [cam]

    conn = store.connection
    assert all(n > 0 for n in counts(store).values()), counts(store)
    before = counts(store)
    for table in TABLES:
        column = conn.execute(f"PRAGMA table_info({table})").fetchall()[1][1]
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute(f"UPDATE {table} SET {column} = {column}")
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute(f"DELETE FROM {table}")
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute("UPDATE retractions SET reason = NULL")
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute("DELETE FROM observations WHERE observation_id = ?", (keep_cam,))

    # Even with the purge guard held, lidar rows cannot be deleted and a
    # retraction can only have its reason blanked.
    conn.execute("BEGIN")
    try:
        conn.execute("INSERT INTO purge_guard (id) VALUES (1)")
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute("DELETE FROM interpretations WHERE observation_id = ?", (lidar,))
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute("DELETE FROM observations WHERE observation_id = ?", (lidar,))
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute("UPDATE retractions SET reason = NULL, retracted_by = 'someone'")
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute("UPDATE retractions SET reason = 'rewritten'")
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute("DELETE FROM retractions")
        with pytest.raises(sqlite3.DatabaseError):
            conn.execute("UPDATE observations SET acquired_at = acquired_at + 1")
    finally:
        conn.execute("ROLLBACK")
    assert counts(store) == before
    assert conn.execute("SELECT count(*) FROM purge_guard").fetchone()[0] == 0


# 4 ---------------------------------------------------------------------------


def test_correction_is_one_transaction(store, clock):
    _, r1 = seed(store)
    d = detect(store, "S1", T0, map_revision_id=r1, label="chair")
    k1 = store.latest_txn()
    (chair,) = interpretation_ids(store, d)

    # A correction that fails part-way writes nothing.
    before = counts(store)
    with pytest.raises(RuntimeError):
        with store.transaction(OPERATOR, "correct_label"):
            store.retract("interpretation", chair, "is a stool", retracted_by=OPERATOR)
            raise RuntimeError("abort")
    assert counts(store) == before

    with store.transaction(OPERATOR, "correct_label") as k2:
        store.retract("interpretation", chair, "is a stool", retracted_by=OPERATOR)
        stool = store.add_interpretation(d, "X", "stool", 1.0, source_kind="operator", writer=OPERATOR)
    assert k2 == k1 + 1 == store.latest_txn()
    conn = store.connection
    assert conn.execute("SELECT recorded_txn FROM retractions").fetchone()[0] == k2
    assert conn.execute(
        "SELECT recorded_txn FROM interpretations WHERE interpretation_id = ?", (stool,)
    ).fetchone()[0] == k2

    with clock.reads_forbidden():
        for k in range(k1, k2 + 1):
            assert len(store.active_interpretations(d, k)) == 1, k
        (at_k1,) = store.active_interpretations(d, k1)
        (at_k2,) = store.active_interpretations(d, k2)
        assert (at_k1["label"], at_k1["source_kind"]) == ("chair", "model")
        assert (at_k2["label"], at_k2["source_kind"]) == ("stool", "operator")
        assert support(store, "X", known_at=k1) == support(store, "X", known_at=k2) == [d]


# 5 ---------------------------------------------------------------------------


def test_retraction_rules(store):
    t1, r1 = seed(store)
    d = detect(store, "S1", T0, map_revision_id=r1)
    (interp,) = interpretation_ids(store, d)
    before = counts(store)
    latest = store.latest_txn()

    # Operator-only for every target kind (§5 decision 4; model self-retraction is deferred).
    for kind, target in (
        ("interpretation", interp),
        ("observation", d),
        ("trajectory_revision", t1),
        ("session", "S1"),
    ):
        with pytest.raises(ValidationError):
            store.retract(kind, target, "bad", retracted_by=MODEL)
    with pytest.raises(ValidationError):
        with store.transaction(MODEL, "retract"):
            store.retract("observation", d, "bad", retracted_by=OPERATOR)
    with pytest.raises(ValidationError):
        store.retract("observation", 9999, "no such row", retracted_by=OPERATOR)
    with pytest.raises(ValidationError):
        store.retract("map_revision", r1, "not a retractable kind", retracted_by=OPERATOR)
    with pytest.raises(ValidationError):
        store.retract("interpretation", interp, "", retracted_by=OPERATOR)
    assert counts(store) == before
    assert store.latest_txn() == latest

    rid = store.retract("interpretation", interp, "wrong object", retracted_by=OPERATOR)
    with pytest.raises(ValidationError):
        store.retract("interpretation", interp, "again", retracted_by=OPERATOR)
    with pytest.raises(ValidationError):
        store.retract("retraction", rid, "undo", retracted_by=OPERATOR)

    # A row that is already invalid through its session can still be retracted itself.
    store.retract("session", "S1", "wrong building", retracted_by=OPERATOR)
    store.retract("observation", d, "sensor fault", retracted_by=OPERATOR)
    rows = store.connection.execute(
        "SELECT target_kind, target_id, retracted_by FROM retractions ORDER BY retraction_id"
    ).fetchall()
    assert [tuple(r) for r in rows] == [
        ("interpretation", str(interp), OPERATOR),
        ("session", "S1", OPERATOR),
        ("observation", str(d), OPERATOR),
    ]


# 6 ---------------------------------------------------------------------------


def test_retracted_session_invalidates_derived_without_cascade_rows(store, clock):
    t1, r1 = seed(store)
    t2 = add_traj(store, "S1", 2, parent=t1)
    r2 = add_map(store, "M-S1", 2, t2)
    m = detect(store, "S1", T0, map_revision_id=r2)
    b = detect(store, "S1", T0 + 5, object_id="B")
    _, q1 = seed(store, "S2")
    o = detect(store, "S2", T0 + 1, map_revision_id=q1, object_id="Y")
    (im,) = interpretation_ids(store, m)
    (ib,) = interpretation_ids(store, b)
    k_before = store.latest_txn()
    with clock.reads_forbidden():
        saved = canonical_json(store.evidence_for("X", NOW, k_before))
        assert support(store, "X", known_at=k_before) == [m]
        assert support(store, "B", known_at=k_before) == [b]
        assert store.compare_positions(m, o, k_before).reason == "different_frames"

    store.retract("session", "S1", "clock offset", retracted_by=OPERATOR)
    k = store.latest_txn()
    rows = store.connection.execute("SELECT target_kind, target_id FROM retractions").fetchall()
    assert [tuple(r) for r in rows] == [("session", "S1")]

    derived = (
        ("session", "S1"),
        ("trajectory_revision", t1),
        ("trajectory_revision", t2),
        ("map_revision", r1),
        ("map_revision", r2),
        ("observation", m),
        ("observation", b),
        ("interpretation", im),
        ("interpretation", ib),
    )
    with clock.reads_forbidden():
        for kind, target in derived:
            assert store.is_invalid(kind, target, k), (kind, target)
            assert not store.is_invalid(kind, target, k_before), (kind, target)
        assert not store.is_invalid("session", "S2", k)
        assert not store.is_invalid("observation", o, k)
        assert support(store, "X", known_at=k) == []
        assert support(store, "B", known_at=k) == []
        assert support(store, "Y", known_at=k) == [o]
        cmp = store.compare_positions(m, o, k)
        assert (cmp.comparable, cmp.reason, cmp.distance_m) == (False, "source_retracted", None)
        assert canonical_json(store.evidence_for("X", NOW, k_before)) == saved


# 7 ---------------------------------------------------------------------------


def test_retracted_trajectory_revision_is_transitive(store, clock):
    t1, r1 = seed(store)
    m1 = detect(store, "S1", T0, map_revision_id=r1)
    t2 = add_traj(store, "S1", 2, parent=t1)
    r2 = add_map(store, "M-S1", 2, t2)
    m2 = detect(store, "S1", T0 + 5, map_revision_id=r2)
    with clock.reads_forbidden():
        assert store.active_trajectory_revision("S1") == t2
        assert support(store, "X") == [m2]

    # A retracted revision falls back to its parent.
    store.retract("trajectory_revision", t2, "bad loop closure", retracted_by=OPERATOR)
    ka = store.latest_txn()
    with clock.reads_forbidden():
        assert store.active_trajectory_revision("S1", ka) == t1
        assert store.is_frame_current(r1, ka)
        for kind, target in (("trajectory_revision", t2), ("map_revision", r2), ("observation", m2)):
            assert store.is_invalid(kind, target, ka)
        assert not store.is_invalid("observation", m1, ka)
        assert support(store, "X", known_at=ka) == [m1]

    # A rebuilt root (no parent), then a second child of T1.
    t3 = add_traj(store, "S1", 3, parent=None)
    r3 = add_map(store, "M-S1", 3, t3)
    m3 = detect(store, "S1", T0 + 6, map_revision_id=r3)
    t4 = add_traj(store, "S1", 4, parent=t1)
    r4 = add_map(store, "M-S1", 4, t4)
    m4 = detect(store, "S1", T0 + 7, map_revision_id=r4)
    kb = store.latest_txn()
    with clock.reads_forbidden():
        assert store.active_trajectory_revision("S1", kb) == t4
        assert support(store, "X", known_at=kb) == [m4]
        saved = canonical_json(store.evidence_for("X", NOW, kb))

    # Retracting T1 takes its child T4, T4's map revision and the map-frame
    # observation bound to it. The rebuilt root survives.
    store.retract("trajectory_revision", t1, "wrong timing", retracted_by=OPERATOR)
    kc = store.latest_txn()
    with clock.reads_forbidden():
        for kind, target in (
            ("trajectory_revision", t1),
            ("trajectory_revision", t4),
            ("map_revision", r1),
            ("map_revision", r4),
            ("observation", m1),
            ("observation", m4),
        ):
            assert store.is_invalid(kind, target, kc), (kind, target)
            assert not store.is_invalid(kind, target, kb), (kind, target)
        for kind, target in (("trajectory_revision", t3), ("map_revision", r3), ("observation", m3)):
            assert not store.is_invalid(kind, target, kc), (kind, target)
        assert store.active_trajectory_revision("S1", kc) == t3
        ev = store.evidence_for("X", NOW, kc)
        assert [s["observation_id"] for s in ev["support"]] == [m3]
        assert ev["frame"] == r3
        assert canonical_json(store.evidence_for("X", NOW, kb)) == saved
    rows = store.connection.execute("SELECT target_kind, target_id FROM retractions").fetchall()
    assert sorted(tuple(r) for r in rows) == [
        ("trajectory_revision", str(t1)),
        ("trajectory_revision", str(t2)),
    ]

    # Body observations have no position once their session has no valid revision.
    u1, q1 = seed(store, "S2")
    bb = detect(store, "S2", T0 + 1, object_id="Y")
    bb2 = detect(store, "S2", T0 + 2, object_id="Y")
    mm = detect(store, "S2", T0 + 3, map_revision_id=q1, object_id="Y")
    kd = store.latest_txn()
    with clock.reads_forbidden():
        assert support(store, "Y", known_at=kd) == [mm, bb2, bb]
    store.retract("trajectory_revision", u1, "corrupt", retracted_by=OPERATOR)
    ke = store.latest_txn()
    with clock.reads_forbidden():
        assert store.active_trajectory_revision("S2", ke) is None
        assert not store.is_invalid("observation", bb, ke)  # the row is valid; it has no position
        for other in (bb2, mm):
            cmp = store.compare_positions(bb, other, ke)
            assert (cmp.comparable, cmp.reason, cmp.distance_m) == (False, "source_retracted", None)
        assert support(store, "Y", known_at=ke) == []


# 8 ---------------------------------------------------------------------------


def test_trajectory_correction_supersedes_not_invalidates(store, clock):
    t1, r1 = seed(store)
    a = detect(store, "S1", T0, map_revision_id=r1)
    b = detect(store, "S1", T0 + 5, map_revision_id=r1)
    k1 = store.latest_txn()
    hashes = {o: store.row_sha256("observations", o) for o in (a, b)}
    with clock.reads_forbidden():
        saved = canonical_json(store.evidence_for("X", NOW, k1))
        assert store.compare_positions(a, b, k1).comparable

    t2 = add_traj(store, "S1", 2, parent=t1)
    k2 = store.latest_txn()
    with clock.reads_forbidden():
        assert store.active_trajectory_revision("S1", k2) == t2
        assert store.is_frame_current(r1, k1)
        assert not store.is_frame_current(r1, k2)
        for kind, target in (("map_revision", r1), ("observation", a), ("observation", b)):
            assert not store.is_invalid(kind, target, k2)
        assert support(store, "X", known_at=k2) == []
        cmp = store.compare_positions(a, b, k2)
        assert (cmp.comparable, cmp.reason, cmp.distance_m) == (False, "superseded_revision", None)
        assert canonical_json(store.evidence_for("X", NOW, k1)) == saved
    assert store.connection.execute("SELECT count(*) FROM retractions").fetchone()[0] == 0

    r2 = add_map(store, "M-S1", 2, t2)
    k3 = store.latest_txn()
    with clock.reads_forbidden():
        assert store.is_frame_current(r2, k3)
        assert support(store, "X", known_at=k3) == []
    assert {o: store.row_sha256("observations", o) for o in (a, b)} == hashes


# 9 ---------------------------------------------------------------------------


def test_compare_positions_rule2_reasons(store, clock):
    t1, r1 = seed(store)
    a = detect(store, "S1", T0, map_revision_id=r1, aabb=box_at(2.00))
    c = detect(store, "S1", T0 + 100, map_revision_id=r1, aabb=box_at(2.30))
    b = detect(store, "S1", T0 + 50, object_id="B")  # body frame
    _, q1 = seed(store, "S2")
    e = detect(store, "S2", T0 + 1, map_revision_id=q1)
    k1 = store.latest_txn()

    with clock.reads_forbidden():
        cmp = store.compare_positions(a, c, k1)
        assert cmp.comparable and cmp.reason is None
        assert cmp.frame == r1
        assert cmp.distance_m == pytest.approx(0.30)
        assert cmp.sigma_m == pytest.approx(0.0424, abs=1e-4)
        assert cmp.revisions_used == {"map_revisions": [r1], "trajectory_revisions": [t1]}
        assert is_moved(cmp.distance_m, cmp.sigma_m)
        other = store.compare_positions(a, e, k1)
        assert (other.comparable, other.reason, other.distance_m) == (False, "different_frames", None)
        # Same current frame, but a body position needs pose derivation (out of V1 scope).
        with pytest.raises(NotImplementedError):
            store.compare_positions(b, a, k1)

    t2 = add_traj(store, "S1", 2, parent=t1)
    k2 = store.latest_txn()
    with clock.reads_forbidden():
        assert store.compare_positions(b, a, k2).reason == "no_map_revision"
        assert store.compare_positions(a, c, k2).reason == "superseded_revision"

    add_map(store, "M-S1", 2, t2)
    k3 = store.latest_txn()
    with clock.reads_forbidden():
        cmp = store.compare_positions(b, a, k3)
        assert (cmp.comparable, cmp.reason, cmp.distance_m) == (False, "superseded_revision", None)

    store.retract("observation", c, "sensor fault", retracted_by=OPERATOR)
    k4 = store.latest_txn()
    with clock.reads_forbidden():
        assert store.compare_positions(a, c, k4).reason == "source_retracted"
        assert store.compare_positions(a, c, k1).comparable

    # An observation not yet recorded at known_at is refused, not guessed.
    later = detect(store, "S1", T0 + 120, map_revision_id=r1)
    with pytest.raises(ValidationError):
        store.compare_positions(a, later, k1)


# 10 --------------------------------------------------------------------------


def test_sigma_pose_adds_zero_and_moved_k3():
    assert MOVED_SIGMA_K == 3
    sigma = comparison_sigma(0.03, 0.03)
    # Exactly the two measurement sigmas in quadrature: no pose term (§5 decision 3).
    assert sigma == math.sqrt(0.03 ** 2 + 0.03 ** 2)
    assert sigma == pytest.approx(0.042, abs=5e-4)
    assert comparison_sigma(0.03, 0.0) == 0.03
    assert is_moved(0.30, sigma)
    assert not is_moved(0.10, sigma)
    assert not is_moved(3 * sigma, sigma)  # strictly greater than K sigma


# 11 --------------------------------------------------------------------------


def test_body_frame_absence_refused(store):
    _, r1 = seed(store)
    before = counts(store)
    with pytest.raises(ValidationError):
        store.add_observation(
            "S1",
            kind="absence",
            sensor_kind="lidar",
            geometry_frame="body",
            acquired_at=T0 + 5,
            aabb=X_BOX,
            sigma_m=0.03,
            measurement_confidence=0.9,
            coverage=1.0,
        )
    with pytest.raises(ValidationError):  # a map-frame absence needs its coverage
        store.add_observation(
            "S1",
            kind="absence",
            sensor_kind="lidar",
            geometry_frame="map",
            map_revision_id=r1,
            acquired_at=T0 + 5,
            aabb=X_BOX,
            sigma_m=0.03,
            measurement_confidence=0.9,
        )
    assert counts(store) == before
    store.add_observation(
        "S1",
        kind="absence",
        sensor_kind="lidar",
        geometry_frame="map",
        map_revision_id=r1,
        acquired_at=T0 + 5,
        aabb=X_BOX,
        sigma_m=0.03,
        measurement_confidence=0.9,
        coverage=1.0,
    )
    assert counts(store)["observations"] == before["observations"] + 1


# 12 --------------------------------------------------------------------------


def test_purge_tombstones_and_blanks_reason(store, clock):
    clock.t = T0  # wall time alongside sensor time, so the look-again rule can be met
    _, r1 = seed(store)
    d = detect(store, "S1", T0, sensor_kind="camera", map_revision_id=r1)
    (i1,) = interpretation_ids(store, d)
    i2 = store.add_interpretation(d, "X", "chair", 1.0, source_kind="operator", writer=OPERATOR)
    store.retract("interpretation", i1, "photo shows Alice at her desk", retracted_by=OPERATOR)
    lidar = detect(store, "S1", T0 + 1, map_revision_id=r1)
    (il,) = interpretation_ids(store, lidar)
    store.retract("interpretation", il, "duplicate", retracted_by=OPERATOR)
    clock.t = T0 + 20
    req = store.request_look_again("X", writer=OPERATOR)
    f = detect(store, "S1", T0 + 30, sensor_kind="camera", map_revision_id=r1)
    (i_f,) = interpretation_ids(store, f)
    store.fulfil_look_again(req, f, writer=OPERATOR)

    conn = store.connection
    expected_hash = {o: store.row_sha256("observations", o) for o in (d, f)}
    recorded = {
        o: conn.execute("SELECT recorded_txn FROM observations WHERE observation_id = ?", (o,)).fetchone()[0]
        for o in (d, f)
    }
    interp_txn = {
        i: conn.execute("SELECT recorded_txn FROM interpretations WHERE interpretation_id = ?", (i,)).fetchone()[0]
        for i in (i1, i2, i_f)
    }
    retraction_before = [tuple(r) for r in conn.execute("SELECT * FROM retractions ORDER BY retraction_id")]
    assert conn.execute("SELECT expires_at FROM observations WHERE observation_id = ?", (d,)).fetchone()[0] == T0 + RETENTION_S
    assert conn.execute("SELECT expires_at FROM observations WHERE observation_id = ?", (lidar,)).fetchone()[0] is None

    clock.t = T0 + RETENTION_S
    assert store.purge_expired(T0 + RETENTION_S - 1) == []
    clock.t = T0 + RETENTION_S + 60
    assert store.purge_expired(T0 + RETENTION_S + 60) == [d, f]
    purge_txn = store.latest_txn()
    assert conn.execute("SELECT operation FROM transactions WHERE txn_id = ?", (purge_txn,)).fetchone()[0] == "purge_expired"

    assert [r[0] for r in conn.execute("SELECT observation_id FROM observations")] == [lidar]
    assert [r[0] for r in conn.execute("SELECT interpretation_id FROM interpretations")] == [il]
    assert conn.execute("SELECT count(*) FROM purge_guard").fetchone()[0] == 0

    obs_rows = conn.execute("SELECT * FROM purged_observations ORDER BY observation_id").fetchall()
    assert [tuple(r) for r in obs_rows] == [
        (d, "detection", "S1", "camera", T0, recorded[d], purge_txn, expected_hash[d]),
        (f, "detection", "S1", "camera", T0 + 30, recorded[f], purge_txn, expected_hash[f]),
    ]
    assoc_rows = conn.execute("SELECT * FROM purged_associations ORDER BY interpretation_id").fetchall()
    assert [tuple(r) for r in assoc_rows] == [
        (i1, d, "X", interp_txn[i1], purge_txn),
        (i2, d, "X", interp_txn[i2], purge_txn),
        (i_f, f, "X", interp_txn[i_f], purge_txn),
    ]
    # Exactly the §2.6 columns: no label, confidence, thumbnail or geometry.
    assert [c[1] for c in conn.execute("PRAGMA table_info(purged_observations)")] == [
        "observation_id", "kind", "session_id", "sensor_kind", "acquired_at",
        "recorded_txn", "purged_txn", "row_sha256",
    ]
    assert [c[1] for c in conn.execute("PRAGMA table_info(purged_associations)")] == [
        "interpretation_id", "observation_id", "object_id", "recorded_txn", "purged_txn",
    ]

    # The purged target's retraction keeps every column but reason; the other is untouched.
    after = [tuple(r) for r in conn.execute("SELECT * FROM retractions ORDER BY retraction_id")]
    columns = [c[1] for c in conn.execute("PRAGMA table_info(retractions)")]
    reason = columns.index("reason")
    assert after[0][reason] is None
    assert after[0][:reason] + after[0][reason + 1:] == retraction_before[0][:reason] + retraction_before[0][reason + 1:]
    assert after[1] == retraction_before[1]
    assert after[1][reason] == "duplicate"

    # The closure keeps the purged observation's id (§5 decision 5).
    closure = conn.execute(
        "SELECT outcome, fulfilled_by_observation_id FROM look_again_closures WHERE request_id = ?", (req,)
    ).fetchone()
    assert tuple(closure) == ("fulfilled", f)


# 13 --------------------------------------------------------------------------


def test_purge_incomplete_rule(store, clock):
    _, r1 = seed(store)
    d = detect(store, "S1", T0, sensor_kind="camera", map_revision_id=r1)
    k1 = store.latest_txn()
    (ix,) = interpretation_ids(store, d)
    with store.transaction(OPERATOR, "correct_identity") as k2:
        store.retract("interpretation", ix, "it is Y", retracted_by=OPERATOR)
        store.add_interpretation(d, "Y", "stool", 1.0, source_kind="operator", writer=OPERATOR)
    with clock.reads_forbidden():  # control before the purge
        assert support(store, "X", known_at=k1) == [d]
        assert support(store, "Y", known_at=k2) == [d]
        assert store.evidence_for("X", NOW, k1)["complete"] is True

    assert store.purge_expired(T0 + RETENTION_S + 1) == [d]
    k3 = store.latest_txn()

    def flag(obj, k, now=NOW):
        ev = store.evidence_for(obj, now, k)
        return ev["support"], ev["complete"], ev["purged_ids"]

    with clock.reads_forbidden():
        assert flag("X", k1) == ([], False, [d])
        assert flag("X", k2) == ([], True, [])
        assert flag("X", k3) == ([], True, [])
        assert flag("Y", k1) == ([], True, [])
        assert flag("Y", k2) == ([], False, [d])
        assert flag("Y", k3) == ([], False, [d])
        assert flag("X", k1, now=T0 - 1) == ([], True, [])  # D was not yet acquired

    # An observation retracted before its purge is not listed from the retraction on.
    e = detect(store, "S1", T0 + 5, sensor_kind="camera", map_revision_id=r1)
    k4 = store.latest_txn()
    store.retract("observation", e, "sensor fault", retracted_by=OPERATOR)
    k5 = store.latest_txn()
    assert store.purge_expired(T0 + 5 + RETENTION_S + 1) == [e]
    k6 = store.latest_txn()
    with clock.reads_forbidden():
        assert flag("X", k4) == ([], False, [e])
        assert flag("X", k5) == ([], True, [])
        assert flag("X", k6) == ([], True, [])

    # So is one whose session is retracted after its purge.
    _, q1 = seed(store, "S2")
    g = detect(store, "S2", T0 + 6, sensor_kind="camera", map_revision_id=q1)
    assert store.purge_expired(T0 + 6 + RETENTION_S + 1) == [g]
    k7 = store.latest_txn()
    store.retract("session", "S2", "wrong building", retracted_by=OPERATOR)
    k8 = store.latest_txn()
    with clock.reads_forbidden():
        assert flag("X", k7) == ([], False, [g])
        assert flag("X", k8) == ([], True, [])


# 14 --------------------------------------------------------------------------


def test_look_again_append_only(store, clock):
    clock.t = T0
    _, r1 = seed(store)
    detect(store, "S1", T0, map_revision_id=r1)
    clock.t = T0 + 20
    a = store.request_look_again("X", writer=OPERATOR)
    k_a = store.latest_txn()
    clock.t = T0 + 30
    b = store.request_look_again("X", writer=OPERATOR)
    k3 = store.latest_txn()

    # Acquired before A was created, ingested after it: does not fulfil A (§2.1).
    late = detect(store, "S1", T0 + 10, map_revision_id=r1)
    before = counts(store)
    with pytest.raises(ValidationError):
        store.fulfil_look_again(a, late, writer=OPERATOR)
    with pytest.raises(ValidationError):
        store.fulfil_look_again(a, 9999, writer=OPERATOR)
    assert counts(store) == before

    f = detect(store, "S1", T0 + 50, sensor_kind="camera", map_revision_id=r1)
    k4 = store.latest_txn()
    store.fulfil_look_again(a, f, writer=OPERATOR)
    k5 = store.latest_txn()
    store.cancel_look_again(b, writer=OPERATOR)
    k6 = store.latest_txn()

    with clock.reads_forbidden():
        assert store.open_look_again("X", k_a) == [a]
        assert store.open_look_again("X", k3) == [a, b]
        assert store.open_look_again("X", k4) == [a, b]
        assert store.open_look_again("X", k5) == [b]
        assert store.open_look_again("X", k6) == []
        assert store.open_look_again("Y", k6) == []

    before = counts(store)
    with pytest.raises(ValidationError):
        store.fulfil_look_again(a, f, writer=OPERATOR)
    with pytest.raises(ValidationError):
        store.cancel_look_again(a, writer=OPERATOR)
    with pytest.raises(ValidationError):
        store.fulfil_look_again(b, f, writer=OPERATOR)
    assert counts(store) == before
    conn = store.connection
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute("UPDATE look_again_requests SET object_id = 'Y'")
    with pytest.raises(sqlite3.DatabaseError):
        conn.execute("UPDATE look_again_closures SET fulfilled_by_observation_id = NULL")

    # Retracting the fulfilling observation does not reopen the request.
    store.retract("observation", f, "wrong session", retracted_by=OPERATOR)
    k7 = store.latest_txn()
    with clock.reads_forbidden():
        assert store.open_look_again("X", k7) == []
    assert tuple(conn.execute(
        "SELECT outcome, fulfilled_by_observation_id FROM look_again_closures WHERE request_id = ?", (a,)
    ).fetchone()) == ("fulfilled", f)

    # An observation must be valid at the fulfilling transaction.
    c = store.request_look_again("X", writer=OPERATOR)
    clock.t = T0 + 40
    g = detect(store, "S1", T0 + 60, map_revision_id=r1)
    store.retract("observation", g, "sensor fault", retracted_by=OPERATOR)
    with pytest.raises(ValidationError):
        store.fulfil_look_again(c, g, writer=OPERATOR)
    assert store.open_look_again("X") == [c]


# 15 --------------------------------------------------------------------------


def seed_migrated_session(store):
    """The post-migration shape of one v1 session (§2.9), via the public API.

    Named deviation: v1 is not on main, so this stands in for the migration. S,
    T1, R1 and the map-frame detections D and E of X bound to R1 are written in
    one transaction flagged ``knowledge_basis='migrated'`` (KM).
    """
    with store.transaction("migration:v1", "migrate_v1", knowledge_basis="migrated") as km:
        add_session(store, "S")
        t1 = add_traj(store, "S", 1)
        r1 = add_map(store, "M-S", 1, t1)
        d = observe(store, "S", T0, map_revision_id=r1)
        e = observe(store, "S", T0 + 5, map_revision_id=r1)
    return km, t1, r1, d, e


def test_V2_6c_migrated_session_first_trajectory_correction(store, clock):
    """V2-6c in its approved scope (§5 decision 6, as written in #101).

    The status words need V3, so CURRENT is asserted as "newest support" and
    UNKNOWN as "no support" in ``evidence_for``.
    """
    km, t1, r1, d, e = seed_migrated_session(store)
    conn = store.connection
    assert conn.execute("SELECT knowledge_basis FROM transactions WHERE txn_id = ?", (km,)).fetchone()[0] == "migrated"
    hashes = {o: store.row_sha256("observations", o) for o in (d, e)}

    # At KM: X is supported by E (newest) in R1; D and E are comparable in R1.
    with clock.reads_forbidden():
        h = store.evidence_for("X", NOW, km)
        assert [s["observation_id"] for s in h["support"]] == [e, d]
        assert h["frame"] == r1
        h_json = canonical_json(h)
        cmp = store.compare_positions(d, e, km)
        assert cmp.comparable and cmp.frame == r1 and cmp.reason is None

    # K2: S's first new trajectory revision, no map built from it yet.
    t2 = add_traj(store, "S", 2, parent=t1)
    k2 = store.latest_txn()
    with clock.reads_forbidden():
        assert not store.is_frame_current(r1, k2)
        ev = store.evidence_for("X", NOW, k2)
        assert ev["support"] == [] and ev["frame"] is None
        cmp = store.compare_positions(d, e, k2)
        assert (cmp.comparable, cmp.reason, cmp.distance_m) == (False, "superseded_revision", None)
        assert canonical_json(store.evidence_for("X", NOW, km)) == h_json

    # K2 retracted nothing and changed no evidence row.
    assert conn.execute("SELECT count(*) FROM retractions").fetchone()[0] == 0
    assert {o: store.row_sha256("observations", o) for o in (d, e)} == hashes

    # K3: R2 built from T2. D and E stay bound to R1 and support nothing.
    r2 = add_map(store, "M-S", 2, t2)
    k3 = store.latest_txn()
    assert [r[0] for r in conn.execute(
        "SELECT map_revision_id FROM observations WHERE observation_id IN (?, ?)", (d, e))] == [r1, r1]
    with clock.reads_forbidden():
        assert store.is_frame_current(r2, k3)
        assert support(store, "X", known_at=k3) == []

    # K4: D2 re-extracted from R2, acquired at t0 + 5, derived from S's raw recording.
    d2 = detect(
        store,
        "S",
        T0 + 5,
        map_revision_id=r2,
        derived_from="sha256:" + sha("raw-S"),
        method="reextract_clusters",
        method_version="1",
        params_sha256=sha("params"),
    )
    k4 = store.latest_txn()
    with clock.reads_forbidden():
        for k in (k4, None):
            ev = store.evidence_for("X", NOW, k)
            assert [s["observation_id"] for s in ev["support"]] == [d2]
            assert ev["frame"] == r2
        assert canonical_json(store.evidence_for("X", NOW, km)) == h_json
    assert conn.execute("SELECT count(*) FROM retractions").fetchone()[0] == 0
    assert {o: store.row_sha256("observations", o) for o in (d, e)} == hashes
