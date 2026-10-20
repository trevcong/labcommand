import pytest

from spatio.walkcheck.defaults import HISTORICAL_LABEL
from spatio.walkcheck.evaluate import evaluate_session
from spatio.walkcheck.synthetic import (
    hold_and_square,
    insert_time_gap,
    stationary_rows,
    write_session,
    yaw_quat,
)


def _score(tmp_path, rows, **kwargs):
    comments = kwargs.pop("comments", ())
    manifest = kwargs.pop("manifest", None)
    stops = kwargs.pop("stops", None)
    xyz = kwargs.pop("xyz", None)
    dest = tmp_path / "session"
    write_session(dest, rows, xyz=xyz, manifest=manifest, stops=stops, comments=comments)
    return evaluate_session(dest, **kwargs)


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


def test_gap_just_over_the_threshold_can_be_relaxed(tmp_path):
    rows = insert_time_gap(hold_and_square(offset=(0.05, 0.0, 0.0)), after_t=10.0, extra_s=1.5)
    blocked = _score(tmp_path / "default", rows)
    assert blocked["result"] == "NOT-EVALUABLE"
    allowed = _score(tmp_path / "wide", rows, max_gap_s=3.0)
    assert allowed["result"] == "PASS"


def test_comments_and_blank_lines_are_ignored(tmp_path):
    dest = tmp_path / "session"
    rows = hold_and_square(offset=(0.05, 0.0, 0.0))
    write_session(dest, rows, comments=("# a comment",))
    text = (dest / "trajectory.tum").read_text(encoding="utf-8")
    lines = text.splitlines()
    lines.insert(3, "")
    lines.insert(4, "# still a comment")
    (dest / "trajectory.tum").write_text("\n".join(lines) + "\n", encoding="utf-8")
    report = evaluate_session(dest)
    assert report["result"] == "PASS"


def test_malformed_line_is_not_evaluable(tmp_path):
    dest = tmp_path / "session"
    write_session(dest, hold_and_square(offset=(0.05, 0.0, 0.0)))
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
