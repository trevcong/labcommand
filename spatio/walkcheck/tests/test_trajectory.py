import pytest

from spatio.walkcheck.defaults import HISTORICAL_LABEL
from spatio.walkcheck.evaluate import evaluate_session
from spatio.walkcheck.synthetic import (
    hold_and_square,
    insert_time_gap,
    stationary_rows,
    write_session,
    write_tum,
    yaw_quat,
)
from spatio.walkcheck.trajectory import ThresholdError, parse_tum, score_trajectory


# A readable cloud so trajectory tests are not tripped by the missing-map rule.
_TINY_MAP = [[0.0, 0.0, 0.0], [0.2, 0.0, 0.0], [0.0, 0.2, 0.1]]


def _score(tmp_path, rows, **kwargs):
    comments = kwargs.pop("comments", ())
    manifest = kwargs.pop("manifest", None)
    stops = kwargs.pop("stops", None)
    xyz = kwargs.pop("xyz", None)
    if xyz is None:
        xyz = _TINY_MAP
    dest = tmp_path / "session"
    write_session(dest, rows, xyz=xyz, manifest=manifest, stops=stops, comments=comments)
    return evaluate_session(dest, **kwargs)


def _polyline(waypoints, rate=10.0):
    quat = yaw_quat(0.0)
    rows = []
    for (t0, p0), (t1, p1) in zip(waypoints, waypoints[1:]):
        steps = int(round((t1 - t0) * rate))
        for i in range(steps):
            frac = i / steps
            xyz = tuple(a + (b - a) * frac for a, b in zip(p0, p1))
            rows.append((t0 + (t1 - t0) * frac, *xyz, *quat))
    t1, p1 = waypoints[-1]
    rows.append((t1, *p1, *quat))
    return rows


def _out_and_back(end_from, end_to, end_t0=30.0, end_t1=35.0):
    """Stationary start, a square, then a straight 5 s end window."""
    return _polyline(
        [
            (0.0, (0.0, 0.0, 0.0)),
            (6.0, (0.0, 0.0, 0.0)),
            (12.0, (3.0, 0.0, 0.0)),
            (18.0, (3.0, 3.0, 0.0)),
            (24.0, (0.0, 3.0, 0.0)),
            (end_t0, end_from),
            (end_t1, end_to),
        ]
    )


def test_five_cm_passes(tmp_path):
    report = _score(tmp_path, hold_and_square(offset=(0.05, 0.0, 0.0)))
    assert report["result"] == "PASS"
    assert report["reasons"] == []
    returned = report["return_to_start"]
    assert returned["error_3d_cm"] == pytest.approx(5.0, abs=0.05)
    assert returned["within_threshold"] is True
    assert returned["alignment"] == "none"


def test_fifteen_cm_fails(tmp_path):
    report = _score(tmp_path, hold_and_square(offset=(0.15, 0.0, 0.0)))
    assert report["result"] == "FAIL"
    assert report["reasons"] == ["return_error"]
    assert report["return_to_start"]["error_3d_cm"] == pytest.approx(15.0, abs=0.05)
    assert report["return_to_start"]["within_threshold"] is False


def test_exactly_ten_cm_fails(tmp_path):
    report = _score(tmp_path, hold_and_square(offset=(0.10, 0.0, 0.0)))
    returned = report["return_to_start"]
    assert returned["error_3d_cm"] == pytest.approx(10.0, abs=0.05)
    assert returned["within_threshold"] is False
    assert report["result"] == "FAIL"
    assert "return_error" in report["reasons"]


def test_vertical_and_horizontal_components(tmp_path):
    vertical = _score(tmp_path / "z", hold_and_square(offset=(0.0, 0.0, 0.04)))
    returned = vertical["return_to_start"]
    assert returned["error_vertical_cm"] == pytest.approx(4.0, abs=0.05)
    assert returned["error_horizontal_cm"] == pytest.approx(0.0, abs=0.05)
    assert returned["error_3d_cm"] == pytest.approx(4.0, abs=0.05)
    assert vertical["result"] == "PASS"

    horizontal = _score(tmp_path / "xy", hold_and_square(offset=(0.03, 0.04, 0.0)))
    returned = horizontal["return_to_start"]
    assert returned["error_horizontal_cm"] == pytest.approx(5.0, abs=0.05)
    assert returned["error_vertical_cm"] == pytest.approx(0.0, abs=0.05)
    assert returned["error_3d_cm"] == pytest.approx(5.0, abs=0.05)


def test_window_mean_not_first_and_last_pose(tmp_path):
    quat = yaw_quat(0.0)
    samples = [
        (0, 0.0),
        (1, 0.0),
        (2, 0.0),
        (3, 0.02),
        (4, 0.02),
        (5, 0.02),
        (6, 1.0),
        (7, 2.0),
        (8, 3.0),
        (9, 2.0),
        (10, 1.0),
        (11, 0.0),
        (12, 0.0),
        (13, 0.0),
        (14, 0.0),
        (15, 0.06),
        (16, 0.06),
        (17, 0.06),
        (18, 0.06),
        (19, 0.06),
        (20, 0.06),
    ]
    rows = [(float(t), x, 0.0, 0.0, *quat) for t, x in samples]
    report = _score(tmp_path, rows)
    returned = report["return_to_start"]
    # First pose to last pose is 6 cm. The window means differ by 5 cm.
    assert returned["error_3d_cm"] == pytest.approx(5.0, abs=0.05)
    assert returned["error_3d_cm"] != pytest.approx(6.0, abs=0.02)
    assert returned["start_mean_xyz_m"][0] == pytest.approx(0.01, abs=1e-6)
    assert returned["end_mean_xyz_m"][0] == pytest.approx(0.06, abs=1e-6)
    assert report["result"] == "PASS"


def test_offset_is_not_aligned_away(tmp_path):
    report = _score(
        tmp_path,
        hold_and_square(offset=(0.15, 0.0, 0.0), end_yaw_deg=40.0),
    )
    returned = report["return_to_start"]
    assert returned["alignment"] == "none"
    assert returned["error_3d_cm"] == pytest.approx(15.0, abs=0.05)
    assert returned["delta_xyz_m"][0] == pytest.approx(0.15, abs=1e-6)
    assert returned["heading_error_deg"] == pytest.approx(40.0, abs=0.05)
    assert report["result"] == "FAIL"
    assert report["reasons"] == ["return_error"]


def test_heading_is_reported_and_does_not_fail_a_closed_walk(tmp_path):
    report = _score(tmp_path, hold_and_square(offset=(0.0, 0.0, 0.0), end_yaw_deg=90.0))
    returned = report["return_to_start"]
    assert report["result"] == "PASS"
    assert returned["error_3d_cm"] == pytest.approx(0.0, abs=0.05)
    assert returned["heading_error_deg"] == pytest.approx(90.0, abs=0.05)
    assert returned["attitude_error_deg"] == pytest.approx(90.0, abs=0.05)


def test_stationary_trajectory_is_not_pass(tmp_path):
    report = _score(tmp_path, stationary_rows())
    assert report["result"] != "PASS"
    assert report["result"] == "FAIL"
    assert "stationary_trajectory" in report["reasons"]
    assert report["return_to_start"]["within_threshold"] is True
    assert report["return_to_start"]["error_3d_cm"] == pytest.approx(0.0, abs=0.05)
    assert report["motion"]["enough_motion"] is False


def test_timestamp_reset_is_not_evaluable(tmp_path):
    rows = hold_and_square()
    broken = list(rows)
    # Pull one middle stamp behind the previous one, without opening a 1 s gap.
    index = 120
    broken[index] = (broken[index - 1][0] - 0.05, *broken[index][1:])
    report = _score(tmp_path, broken)
    assert report["result"] == "NOT-EVALUABLE"
    assert "non_increasing_timestamps" in report["reasons"]
    assert report["return_to_start"] is None
    assert "PASS" != report["result"]


def test_timestamp_gap_is_not_evaluable(tmp_path):
    rows = insert_time_gap(hold_and_square(), after_t=10.0, extra_s=2.0)
    report = _score(tmp_path, rows)
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["timestamp_gap"]
    assert report["return_to_start"] is None


def test_nonfinite_gap_limit_cannot_pass_a_gap(tmp_path):
    """Codex P2. `--max-gap nan` used to score a 10.1 s step as PASS.

    Comparisons with NaN are false, so the gap gate never fired. The same
    hole exists for an infinite gap limit, and for an infinite endpoint
    spread on a window that travels from -30 cm to +30 cm.
    """
    rows = insert_time_gap(hold_and_square(offset=(0.05, 0.0, 0.0)), after_t=10.0, extra_s=10.1)
    blocked = _score(tmp_path / "blocked", rows)
    assert blocked["result"] == "NOT-EVALUABLE"
    assert blocked["reasons"] == ["timestamp_gap"]
    tum_path = tmp_path / "trajectory.tum"
    write_tum(tum_path, rows)
    loaded = parse_tum(tum_path.read_text(encoding="utf-8"))
    in_memory = score_trajectory(loaded)
    assert in_memory["evaluated"] is False
    assert "timestamp_gap" in in_memory["result_reasons"]
    for bad in (float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ThresholdError):
            score_trajectory(loaded, max_gap_s=bad)
        with pytest.raises(ThresholdError):
            _score(tmp_path / "api", rows, max_gap_s=bad)
    moving = _out_and_back((-0.30, 0.0, 0.0), (0.30, 0.0, 0.0))
    assert _score(tmp_path / "moving", moving)["result"] == "NOT-EVALUABLE"
    with pytest.raises(ThresholdError):
        _score(tmp_path / "spread", moving, max_endpoint_spread_m=float("inf"))
    with pytest.raises(ThresholdError):
        _score(tmp_path / "speed", moving, max_endpoint_speed_m_s=float("nan"))


def test_gap_just_over_the_threshold_can_be_relaxed(tmp_path):
    rows = insert_time_gap(hold_and_square(offset=(0.05, 0.0, 0.0)), after_t=10.0, extra_s=1.5)
    blocked = _score(tmp_path / "default", rows)
    assert blocked["result"] == "NOT-EVALUABLE"
    allowed = _score(tmp_path / "wide", rows, max_gap_s=3.0)
    assert allowed["result"] == "PASS"


def test_comments_and_blank_lines_are_ignored(tmp_path):
    dest = tmp_path / "session"
    rows = hold_and_square(offset=(0.05, 0.0, 0.0))
    write_session(dest, rows, xyz=_TINY_MAP, comments=("# a comment",))
    text = (dest / "trajectory.tum").read_text(encoding="utf-8")
    lines = text.splitlines()
    lines.insert(3, "")
    lines.insert(4, "# still a comment")
    (dest / "trajectory.tum").write_text("\n".join(lines) + "\n", encoding="utf-8")
    report = evaluate_session(dest)
    assert report["result"] == "PASS"


def test_malformed_line_is_not_evaluable(tmp_path):
    dest = tmp_path / "session"
    write_session(dest, hold_and_square(offset=(0.05, 0.0, 0.0)), xyz=_TINY_MAP)
    text = (dest / "trajectory.tum").read_text(encoding="utf-8")
    (dest / "trajectory.tum").write_text(text + "1 2 3\n", encoding="utf-8")
    report = evaluate_session(dest)
    assert report["result"] == "NOT-EVALUABLE"
    assert "malformed_trajectory" in report["reasons"]


def test_bad_quaternion_is_not_evaluable(tmp_path):
    rows = hold_and_square(offset=(0.05, 0.0, 0.0))
    broken = list(rows)
    broken[10] = (*broken[10][:4], 0.0, 0.0, 0.0, 0.0)
    report = _score(tmp_path, broken)
    assert report["result"] == "NOT-EVALUABLE"
    assert "bad_quaternion" in report["reasons"]


def test_trajectory_shorter_than_two_windows_is_not_evaluable(tmp_path):
    rows = hold_and_square(duration=4.0, hold=0.5, side=3.0)
    report = _score(tmp_path, rows)
    assert report["result"] == "NOT-EVALUABLE"
    assert "trajectory_too_short" in report["reasons"]


def test_intermediate_stop_is_sanity_only(tmp_path):
    quat = yaw_quat(0.0)
    samples = []
    for t in range(0, 7):
        samples.append((t, 0.0, 0.0, 0.0))
    for t, x in ((7, 1.0), (8, 2.0), (9, 3.0), (10, 4.0)):
        samples.append((t, x, 0.0, 0.0))
    for t in range(11, 16):
        samples.append((t, 4.0, 0.0, 0.0))
    for t, x in ((16, 4.05), (17, 4.10), (18, 4.15), (19, 4.20)):
        samples.append((t, x, 0.0, 0.0))
    for t in range(20, 25):
        samples.append((t, 4.2, 0.0, 0.0))
    for t, x in ((25, 3.0), (26, 2.0), (27, 1.0), (28, 0.0), (29, 0.0), (30, 0.0), (31, 0.0), (32, 0.0)):
        samples.append((t, x, 0.0, 0.0))
    for t in range(33, 41):
        samples.append((t, 0.04, 0.0, 0.0))
    rows = [(float(t), x, y, z, *quat) for t, x, y, z in samples]
    stops = {
        "stops": [
            {
                "name": "east-mark",
                "visits": [{"t0": 11, "t1": 15}, {"t0": 20, "t1": 24}],
            }
        ]
    }
    report = _score(tmp_path, rows, stops=stops)
    assert report["result"] == "PASS"
    stop = report["intermediate_stops"]["stops"][0]
    assert stop["name"] == "east-mark"
    assert stop["max_mean_separation_cm"] == pytest.approx(20.0, abs=0.05)
    assert report["return_to_start"]["error_3d_cm"] == pytest.approx(4.0, abs=0.05)


def test_cancelling_end_window_is_not_evaluable(tmp_path):
    """Codex P1. The end window travels from -30 cm to +30 cm.

    Its mean matches the start, and the last pose is +30 cm away. The old
    checker returned PASS. A moving window is not a parked endpoint.
    """
    report = _score(tmp_path, _out_and_back((-0.30, 0.0, 0.0), (0.30, 0.0, 0.0)))
    assert report["result"] == "NOT-EVALUABLE"
    assert report["result"] != "PASS"
    assert report["reasons"] == ["endpoint_not_stationary"]
    assert report["return_to_start"] is None
    ends = report["endpoints"]
    assert ends["start_spread_cm"] == pytest.approx(0.0, abs=0.05)
    assert ends["end_mean_xyz_m"][0] == pytest.approx(0.0, abs=1e-6)
    assert ends["end_last_xyz_m"][0] == pytest.approx(0.30, abs=1e-6)
    assert ends["end_spread_cm"] == pytest.approx(30.0, abs=0.05)
    assert ends["end_speed_cm_s"] == pytest.approx(12.0, abs=0.1)
    assert ends["stationary"] is False


def test_slow_drift_over_the_spread_limit_is_not_evaluable(tmp_path):
    """8 cm of drift at 3.2 cm/s. Speed is under the limit. Spread is not.

    The window mean is the start, so dropping the spread check would pass.
    """
    report = _score(tmp_path, _out_and_back((-0.08, 0.0, 0.0), (0.08, 0.0, 0.0)))
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["endpoint_not_stationary"]
    ends = report["endpoints"]
    assert ends["end_spread_cm"] == pytest.approx(8.0, abs=0.05)
    assert ends["end_spread_cm"] > ends["spread_limit_cm"]
    assert ends["end_speed_cm_s"] < ends["speed_limit_cm_s"]
    assert ends["end_mean_xyz_m"][0] == pytest.approx(0.0, abs=1e-6)
    assert ends["spread_limit_cm"] == pytest.approx(5.0, abs=1e-6)


def test_fast_jitter_inside_the_spread_limit_is_not_evaluable(tmp_path):
    """±4 cm at 10 Hz stays inside 5 cm and is far faster than 10 cm/s.

    Dropping the speed check would pass, because the mean is the start.
    """
    quat = yaw_quat(0.0)
    rows = _polyline(
        [
            (0.0, (0.0, 0.0, 0.0)),
            (6.0, (0.0, 0.0, 0.0)),
            (12.0, (3.0, 0.0, 0.0)),
            (18.0, (3.0, 3.0, 0.0)),
            (24.0, (0.0, 3.0, 0.0)),
            (30.0, (0.04, 0.0, 0.0)),
        ]
    )
    rows = rows[:-1]
    for i in range(51):
        x = 0.04 if i % 2 == 0 else -0.04
        rows.append((30.0 + i * 0.1, x, 0.0, 0.0, *quat))
    report = _score(tmp_path, rows)
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["endpoint_not_stationary"]
    ends = report["endpoints"]
    assert ends["end_spread_cm"] < ends["spread_limit_cm"]
    assert ends["end_speed_cm_s"] > ends["speed_limit_cm_s"]
    assert ends["speed_limit_cm_s"] == pytest.approx(10.0, abs=1e-6)
    assert ends["end_mean_xyz_m"][0] == pytest.approx(0.0, abs=1e-3)


def test_historical_label_follows_the_manifest_flag(tmp_path):
    rows = hold_and_square(offset=(0.05, 0.0, 0.0))
    unmarked = _score(
        tmp_path / "unmarked",
        rows,
        manifest={"physical_accuracy_evaluated": False, "run_id": "hist"},
    )
    assert HISTORICAL_LABEL in unmarked["notes"]
    marked = _score(
        tmp_path / "marked",
        rows,
        manifest={"physical_accuracy_evaluated": True, "run_id": "live", "notes": "floor marks checked"},
    )
    assert HISTORICAL_LABEL not in marked["notes"]
    assert any("did not re-check" in note for note in marked["notes"])
