"""Offline acceptance: real MCAP decoding plus synthetic parked/push/return."""
import json
from pathlib import Path
import shutil
import subprocess
import sys

from mcap_ros2.writer import Writer
import numpy as np
import pytest

from spatio import preflight
from spatio.bagtiming.tests.test_bagtiming import (
    CUSTOM, CUSTOM_SCHEMA, IMU, IMU_SCHEMA, NS, custom, imu,
)


@pytest.fixture
def packet(tmp_path):
    bag = tmp_path / "recording.mcap"
    session = tmp_path / "replay"
    (session / "logs").mkdir(parents=True)
    t = 1000 + np.arange(401) / 10
    x = np.interp(t, [1000, 1010, 1020, 1030, 1040], [0, 0, 2, 0, 0])
    # 6 cm/s per-step parked jitter: caught by the shipped 0.10 speed,
    # rejected by bare walkcheck's 0.02. No thresholds are tuned here.
    x += np.where(np.arange(len(t)) % 2, .003, -.003)
    data = np.column_stack([t, x, np.zeros((len(t), 5)), np.ones(len(t))])
    np.savetxt(session / "trajectory.tum", data, fmt="%.9f")
    fixture = Path(__file__).parents[1] / "io/fixtures/loop_5cm/map.pcd"
    shutil.copyfile(fixture, session / "map.pcd")
    (session / "manifest.json").write_text(json.dumps({"source_recording": str(bag)}))
    (session / "logs/run_params.json").write_text(json.dumps({"mode": "replay", "container_exit": 0}))
    (session / "logs/container_exit_code").write_text("0\n")
    with bag.open("wb") as f:
        writer = Writer(f)
        lidar_schema = writer.register_msgdef(CUSTOM, CUSTOM_SCHEMA)
        imu_schema = writer.register_msgdef(IMU, IMU_SCHEMA)
        for i, stamp in enumerate(t):
            ns = round(stamp * NS)
            msg = custom(ns)
            for point in msg["points"]:
                point["x"] = 1 + (.004 if i % 2 else -.004)
            writer.write_message("/livox/lidar", lidar_schema, msg, log_time=ns + 2_000_000)
            writer.write_message("/livox/imu", imu_schema, imu(ns), log_time=ns + 3_000_000)
        writer.finish()
    return bag, session, [(1000., 1010.), (1030., 1040.)]


def run(packet, **kwargs):
    return preflight.score(*packet, lidar_topic="/livox/lidar", **kwargs)


def test_parked_push_return_packet(packet):
    report = run(packet)
    assert report["replay"]["status"] == "succeeded"
    assert report["replay"]["executed_by_preflight"] is False
    assert report["physical_accuracy_evaluated"] is False
    assert report["settings"]["effective_hold_speed_m_s"] == .10
    assert report["settings"]["bare_walkcheck_hold_speed_m_s"] == .02
    assert report["detected_holds"]["imu_used"] is False
    assert len(report["detected_holds"]["intervals"]) == 2
    for segment in report["parked"]:
        assert segment["hold"]["fires"] is True
        assert segment["pose_jitter"]["samples"] == 101
        assert segment["pose_jitter"]["step_speed_p95_m_s"] == pytest.approx(.06)
        assert segment["pose_jitter"]["max_radius_m"] == pytest.approx(.00303, abs=1e-6)
        assert segment["scan_centroid_spread"]["max_radius_m"] == pytest.approx(.00404, abs=1e-6)
    lidar, imu_topic = report["timing"]["topics"]
    assert lidar["header"]["rate_hz"] == 10
    assert lidar["header_minus_receipt_s"]["median"] == -.002
    assert imu_topic["header_minus_receipt_s"]["median"] == -.003
    assert "Hold speed used: 0.1 m/s" in preflight.summary(report)
    json.dumps(report, allow_nan=False)


def test_failed_holds_still_measure_jitter(packet):
    report = run(packet, hold_speed=.02)
    assert report["settings"]["effective_hold_speed_m_s"] == .02
    assert report["settings"]["hold_speed_source"] == "CLI override"
    for segment in report["parked"]:
        assert segment["hold"]["fires"] is False
        assert segment["pose_jitter"]["status"] == "measured"


def test_settings_and_effective_speed_cap(packet, tmp_path):
    settings = tmp_path / "settings.conf"
    settings.write_text(preflight.SETTINGS.read_text().replace("MAX_HOLD_SPEED_M_S=0.10", "MAX_HOLD_SPEED_M_S=0.02"))
    assert not run(packet, settings=settings)["parked"][0]["hold"]["fires"]
    assert run(packet, hold_speed=.5)["settings"]["effective_hold_speed_m_s"] == .1


def test_walkday_preserved_replay_manifest(packet):
    _, session, _ = packet
    (session / "manifest.json").rename(session / "logs/replay_manifest.json")
    (session / "manifest.json").write_text('{"schema": "rebuilt walkday manifest"}')
    assert run(packet)["replay"]["status"] == "succeeded"


def test_timing_only_fallback_cannot_invent_point_spread(packet, monkeypatch):
    original = preflight.BagReader.records

    def timing_only(reader):
        for topic, receipt, msg in original(reader):
            if topic == "/livox/lidar":
                for point in msg.points:
                    del point.x
                    del point.y
                    del point.z
            yield topic, receipt, msg

    monkeypatch.setattr(preflight.BagReader, "records", timing_only)
    report = run(packet)
    assert report["scans_without_xyz_or_header"] == 401
    assert report["parked"][0]["scan_centroid_spread"]["status"] == "not_measured"
    assert report["parked"][0]["pose_jitter"]["status"] == "measured"


def test_timing_faults_preserve_storage_order(packet):
    bag, _, _ = packet
    with bag.open("wb") as f:
        writer = Writer(f)
        schema = writer.register_msgdef(IMU, IMU_SCHEMA)
        for stamp in [1000, 1001, 1002, 1008, 1008, 999]:
            writer.write_message("/livox/imu", schema, imu(stamp * NS), log_time=(stamp + 1) * NS)
        writer.finish()
    report = run(packet)
    topic = report["timing"]["topics"][0]
    for basis in ("header", "receipt"):
        assert topic[basis]["gap_count"] == 1
        assert topic[basis]["max_gap_s"] == 6
        assert topic[basis]["backward_jumps"] == 1
        assert topic[basis]["duplicate_intervals"] == 1
        assert topic[basis]["rate_hz"] is None
    assert topic["header_minus_receipt_s"]["median"] == -1
    assert report["parked"][0]["scan_centroid_spread"]["status"] == "not_measured"
    assert "not observed" in report["limitations"][-1]


def test_submicrosecond_offsets_are_not_rounded_away(packet):
    bag, _, _ = packet
    with bag.open("wb") as f:
        writer = Writer(f)
        schema = writer.register_msgdef(IMU, IMU_SCHEMA)
        for stamp in (1000 * NS, 1001 * NS):
            writer.write_message("/imu", schema, imu(stamp), log_time=stamp + 1)
        writer.finish()
    assert run(packet)["timing"]["topics"][0]["header_minus_receipt_s"]["median"] == -1e-9


@pytest.mark.parametrize("case,status", [
    ("failed", "failed"), ("missing", "not_available"), ("map", "invalid"),
    ("mismatch", "invalid"), ("historical", "historical"), ("conflict", "invalid"),
])
def test_replay_evidence(packet, case, status):
    bag, session, _ = packet
    if case == "failed":
        (session / "logs/container_exit_code").write_text("4")
    elif case == "missing":
        (session / "logs/container_exit_code").unlink()
    elif case == "map":
        (session / "map.pcd").write_text("broken")
    elif case == "mismatch":
        other = bag.with_name("other.mcap")
        shutil.copyfile(bag, other)
        (session / "manifest.json").write_text(json.dumps({"source_recording": str(other)}))
    else:
        (session / "logs/run_params.json").write_text(json.dumps({
            "mode": "historical" if case == "historical" else "replay",
            "container_exit": 0 if case == "historical" else 1}))
    assert run(packet)["replay"]["status"] == status


@pytest.mark.parametrize("case", ["reset", "nan", "missing", "gap"])
def test_invalid_trajectory_is_not_a_hold(packet, case):
    _, session, _ = packet
    path = session / "trajectory.tum"
    data = np.loadtxt(path)
    if case == "missing":
        path.unlink()
    else:
        if case == "reset":
            data[5, 0] = data[4, 0]
        elif case == "nan":
            data[5, 1] = float("nan")
        else:
            data = np.delete(data, np.arange(20, 40), axis=0)
        np.savetxt(path, data)
    report = run(packet)
    assert report["parked"][0]["pose_jitter"]["status"] == "not_measured"
    assert report["parked"][0]["hold"]["status"] == "not_measured"
    json.dumps(report, allow_nan=False)


def test_short_or_outside_labels_do_not_claim_holds(packet):
    bag, session, _ = packet
    report = run((bag, session, [(990, 999), (1031, 1033)]))
    assert report["parked"][0]["pose_jitter"]["status"] == "not_measured"
    assert report["parked"][1]["hold"]["fires"] is False


@pytest.mark.parametrize("parked", [[], [(0, 2)], [(0, 2), (1, 3)], [(0, 2), (3, float("nan"))]])
def test_invalid_labels(packet, parked):
    with pytest.raises(ValueError, match="parked intervals"):
        run((packet[0], packet[1], parked))


@pytest.mark.parametrize("speed", [float("nan"), float("inf"), -1])
def test_invalid_settings(packet, speed):
    with pytest.raises(ValueError):
        run(packet, hold_speed=speed)


def test_cli_summary_and_strict_json(packet, tmp_path):
    bag, session, _ = packet
    output = tmp_path / "preflight.json"
    result = subprocess.run([sys.executable, "-m", "spatio.preflight", str(bag),
        "--session", str(session), "--lidar-topic", "/livox/lidar",
        "--parked", "1000", "1010", "--parked", "1030", "1040", "--json", str(output)],
        stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert "Replay: succeeded" in result.stdout
    assert "hold=True" in result.stdout
    assert json.loads(output.read_text())["schema"] == "spatio.preflight/1"


@pytest.mark.parametrize("damage", ["truncated", "bad_magic"])
def test_cli_corrupt_mcap_exits_2(packet, tmp_path, damage):
    bag, session, _ = packet
    data = bag.read_bytes()
    bag.write_bytes(data[:len(data) // 2] if damage == "truncated" else b"corrupt!" + data[8:])
    output = tmp_path / "preflight.json"
    result = subprocess.run([sys.executable, "-m", "spatio.preflight", str(bag),
        "--session", str(session), "--lidar-topic", "/livox/lidar",
        "--parked", "1000", "1010", "--parked", "1030", "1040", "--json", str(output)],
        stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=30)
    assert result.returncode == 2, result.stderr
    assert "preflight:" in result.stderr
    assert "Traceback" not in result.stderr
    assert not output.exists()


@pytest.mark.parametrize("edge", ["start", "end"])
@pytest.mark.parametrize("gap,measured", [(1., True), (1.1, False)])
def test_parked_interval_edge_coverage(packet, edge, gap, measured):
    bag, session, _ = packet
    # Whole-trajectory coverage and interior spacing are valid. Only the
    # distance from the label boundary to its first/last pose varies.
    path = session / "trajectory.tum"
    data = np.loadtxt(path)
    if edge == "start":
        data = data[(data[:, 0] < 1001) | (data[:, 0] >= 1001 + gap)]
    else:
        data = data[(data[:, 0] <= 1009 - gap) | (data[:, 0] > 1009)]
    np.savetxt(path, data)
    segment = run((bag, session, [(1001., 1009.), (1030., 1040.)]))["parked"][0]
    assert segment["pose_jitter"]["status"] == ("measured" if measured else "not_measured")
    if not measured:
        assert "coverage" in segment["pose_jitter"]["reason"]
        assert segment["hold"] == preflight.unavailable("parked interval lacks trajectory coverage")


@pytest.mark.parametrize("constant", ["NaN", "Infinity", "-Infinity", "1e999"])
def test_nonfinite_replay_duration_is_invalid_evidence(packet, constant):
    _, session, _ = packet
    (session / "logs/run_params.json").write_text(
        '{"mode": "replay", "container_exit": 0, "duration_s": ' + constant + '}')
    report = run(packet)
    assert report["replay"]["status"] == "invalid"
    assert "duration_s" in report["replay"]["reason"]
    assert report["parked"][0]["pose_jitter"]["status"] == "measured"
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize("duration", ["all", "30", 30., 0])
def test_replay_duration_preserved(packet, duration):
    _, session, _ = packet
    (session / "logs/run_params.json").write_text(json.dumps({
        "mode": "replay", "container_exit": 0, "duration_s": duration}))
    replay = run(packet)["replay"]
    assert replay["status"] == "succeeded"
    assert replay["requested_duration_s"] == duration


def test_outside_labels_report_missing_coverage(packet):
    bag, session, _ = packet
    report = run((bag, session, [(990., 999.), (1041., 1050.)]))
    assert report["detected_holds"]["intervals"]
    for segment in report["parked"]:
        assert segment["hold"] == preflight.unavailable("parked interval lacks trajectory coverage")


@pytest.mark.parametrize("fault", ["none", "missing_xyz", "frame"])
def test_centroids_only_use_parked_scans_but_diagnostics_cover_whole_bag(packet, monkeypatch, fault):
    original_records = preflight.BagReader.records
    original_centroid = preflight.scan_centroid
    calls = []

    def records(reader):
        for topic, receipt, msg in original_records(reader):
            if topic == "/livox/lidar" and preflight.header_ns(msg) == 1020 * NS:
                if fault == "missing_xyz":
                    for point in msg.points:
                        point.x = float("nan")
                elif fault == "frame":
                    msg.header.frame_id = "different_frame"
            yield topic, receipt, msg

    def centroid(msg, kind):
        calls.append(preflight.header_ns(msg) / NS)
        return original_centroid(msg, kind)

    monkeypatch.setattr(preflight.BagReader, "records", records)
    monkeypatch.setattr(preflight, "scan_centroid", centroid)
    report = run(packet)
    assert len(calls) == 202
    assert all(1000 <= t <= 1010 or 1030 <= t <= 1040 for t in calls)
    assert report["timing"]["topics"][0]["count"] == 401
    assert report["scans_without_xyz_or_header"] == (1 if fault == "missing_xyz" else 0)
    for segment in report["parked"]:
        assert segment["scan_centroid_spread"]["status"] == (
            "not_measured" if fault == "frame" else "measured")
