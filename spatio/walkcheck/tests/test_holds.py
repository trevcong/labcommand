"""New-walk regressions. Every trajectory and IMU stream here is synthetic."""

import json

import numpy as np
import pytest

from spatio.walkcheck.cli import main
from spatio.walkcheck.evaluate import evaluate_session
from spatio.walkcheck.holds import load_imu
from spatio.walkcheck.synthetic import hold_and_square, write_session, yaw_quat
from spatio.walkcheck.trajectory import ThresholdError, score_trajectory


def score(rows, **kwargs):
    data = np.asarray(rows, dtype=float)
    return score_trajectory(
        {"ok": True, "t": data[:, 0], "xyz": data[:, 1:4], "q": data[:, 4:8]},
        **kwargs,
    )


def line(points, rate=10, origin=0):
    rows = []
    for (a, x), (b, y) in zip(points, points[1:]):
        for stamp in np.linspace(a, b, int(round((b - a) * rate)), endpoint=False):
            rows.append((origin + stamp, x + (y - x) * (stamp - a) / (b - a),
                         0, 0, *yaw_quat(0)))
    b, y = points[-1]
    rows.append((origin + b, y, 0, 0, *yaw_quat(0)))
    return rows


def imu_file(tmp_path, *, rate=20, duration=40):
    times = np.arange(int(duration * rate) + 1) / rate
    data = np.zeros((len(times), 7))
    data[:, 0] = times
    data[:, 6] = 9.80665
    path = tmp_path / "imu.csv"
    save_imu(path, data)
    return path, data


def save_imu(path, data):
    np.savetxt(path, data, delimiter=",", header="t,gx,gy,gz,ax,ay,az", comments="")


@pytest.mark.parametrize("rate,origin,hold", [(10, 0, 10), (7, 1200, 12), (20, 1700000000, 9)])
def test_first_last_holds_are_detected_on_different_new_walks(rate, origin, hold):
    rows = hold_and_square(rate=rate, hold=hold, duration=55)
    rows = [(r[0] + origin, *r[1:]) for r in rows]
    result = score(rows)
    assert result["evaluated"]
    assert result["result_reasons"] == []
    holds = result["holds"]
    assert len(holds["intervals"]) == 2
    assert holds["start"]["t_start"] == origin
    assert holds["start"]["duration_s"] >= hold - 1 / rate
    assert holds["end"]["t_end"] == origin + 55
    assert holds["end"]["duration_s"] >= hold - 1 / rate
    assert result["return_to_start"]["error_3d_cm"] == pytest.approx(5)
    assert not holds["flags"]


@pytest.mark.parametrize("hold", [0, 5, 7.9])
def test_short_or_missing_holds_never_score(hold):
    result = score(hold_and_square(hold=hold))
    assert not result["evaluated"]
    assert result["return_to_start"] is None
    assert "endpoint_not_stationary" in result["result_reasons"]


def test_minimum_duration_is_inclusive_and_overridable():
    assert score(hold_and_square(hold=8))["evaluated"]
    rows = hold_and_square(hold=6)
    assert not score(rows)["evaluated"]
    assert score(rows, min_hold_s=6)["evaluated"]
    assert not score(rows, min_hold_s=6, window_s=8)["evaluated"]


@pytest.mark.parametrize("remove_start", [False, True])
def test_one_hold_cannot_supply_both_endpoints(remove_start):
    rows = hold_and_square()
    rows = [r for r in rows if (r[0] > 10 if remove_start else r[0] < 30)]
    result = score(rows)
    assert not result["evaluated"]
    assert len(result["holds"]["intervals"]) == 1


def test_reposition_then_lap_does_not_redefine_start():
    # Long first hold, short move, longer pause, lap, last hold.
    rows = line([(0, 0), (25, 0), (32, .6), (82, .6),
                 (100, 4), (120, -3), (169, .65), (182, .65)])
    result = score(rows)
    assert result["evaluated"]
    holds = result["holds"]
    assert len(holds["intervals"]) == 3
    assert holds["start"]["t_start"] == 0
    assert holds["start"]["t_end"] == pytest.approx(25, abs=.11)
    assert holds["end"]["t_start"] == pytest.approx(169, abs=.11)
    assert holds["flags"] == ["possible_repositioning"]
    # Re-zeroing at the 0.6 m pause would claim 5 cm. We retain 65 cm.
    assert result["return_to_start"]["error_3d_cm"] == pytest.approx(65)
    assert result["result_reasons"] == ["return_error"]


@pytest.mark.parametrize("before", [True, False])
def test_significant_motion_outside_holds_blocks_a_pass(before):
    rows = line([(0, -1), (4, 0), (14, 0), (20, 3), (26, .05), (36, .05)])
    if not before:
        rows = [(36 - r[0], *r[1:]) for r in reversed(rows)]
    result = score(rows)
    assert not result["evaluated"]
    flag = "motion_before_start_hold" if before else "motion_after_end_hold"
    assert flag in result["result_reasons"]
    assert flag in result["holds"]["flags"]


def test_small_initial_settling_does_not_replace_first_hold():
    rows = line([(0, -.05), (.2, 0), (10.2, 0), (18, 3), (26, .05), (36, .05)])
    result = score(rows)
    assert result["evaluated"]
    assert not result["result_reasons"]
    assert result["holds"]["start"]["t_start"] == pytest.approx(.2)
    assert result["return_to_start"]["error_3d_cm"] == pytest.approx(5)
    assert not score(rows, significant_motion_m=.01)["evaluated"]


def test_rotation_in_place_is_not_a_hold():
    rows = hold_and_square()
    rows = [(r[0], *r[1:4], *yaw_quat(r[0] * 20 if r[0] <= 10 else 0)) for r in rows]
    result = score(rows)
    assert not result["evaluated"]
    assert result["holds"]["start"]["t_start"] >= 30


def test_quaternion_sign_does_not_break_holds():
    rows = hold_and_square()
    rows = [(*r[:4], *(np.array(r[4:]) * (-1 if i % 2 else 1))) for i, r in enumerate(rows)]
    assert score(rows)["evaluated"]


def test_slow_accumulated_drift_is_not_chopped_into_convenient_holds():
    rows = line([(0, 0), (10, 0), (20, 3), (30, -.3), (50, .3)])
    result = score(rows)
    assert not result["evaluated"]
    assert len(result["holds"]["intervals"]) == 1


def test_no_hold_across_a_timestamp_gap():
    rows = hold_and_square(hold=10)
    rows = [r for r in rows if not 3 < r[0] < 7]
    result = score(rows)
    assert not result["evaluated"]
    assert "timestamp_gap" in result["result_reasons"]


@pytest.mark.parametrize("origin", [0, 1700000000])
def test_optional_imu_confirms_holds(tmp_path, origin):
    # 10 Hz is exactly the default gap limit, including on an epoch clock.
    path, data = imu_file(tmp_path, rate=10)
    data[:, 0] += origin
    save_imu(path, data)
    rows = [(r[0] + origin, *r[1:]) for r in hold_and_square()]
    result = score(rows, imu=load_imu(path))
    assert result["evaluated"]
    assert result["holds"]["imu_used"]
    assert len(result["holds"]["intervals"]) == 2


@pytest.mark.parametrize("fault", ["gyro", "acceleration", "gap", "coverage", "spike"])
def test_imu_disagreement_or_missing_coverage_vetoes_hold(tmp_path, fault):
    path, data = imu_file(tmp_path)
    if fault == "gyro":
        data[:, 3] = .2
    elif fault == "acceleration":
        data[:, 6] += 1
    elif fault == "gap":
        data = data[~((data[:, 0] > 3) & (data[:, 0] < 7))]
    elif fault == "coverage":
        data[:, 0] += 100
    else:
        # At higher rate than poses, between trajectory samples.
        data[101, 1] = 1
    save_imu(path, data)
    result = score(hold_and_square(), imu=load_imu(path))
    assert not result["evaluated"]
    assert result["return_to_start"] is None


@pytest.mark.parametrize("text", [
    "bad header\n", "t,gx,gy,gz,ax,ay,az\n0,0,0,0,0,0,9.8\n",
    "t,gx,gy,gz,ax,ay,az\n0,0,0,0,0,0,9.8\n0,0,0,0,0,0,9.8\n",
    "t,gx,gy,gz,ax,ay,az\n0,nan,0,0,0,0,9.8\n1,0,0,0,0,0,9.8\n",
])
def test_bad_imu_is_not_silently_ignored(tmp_path, text):
    path = tmp_path / "imu.csv"
    path.write_text(text)
    result = score(hold_and_square(), imu=load_imu(path))
    assert not result["evaluated"]
    assert result["result_reasons"] == ["invalid_imu"]


@pytest.mark.parametrize("parameter", [
    "min_hold_s", "max_hold_speed_m_s", "max_angular_speed_deg_s", "significant_motion_m",
    "significant_rotation_deg", "max_imu_gap_s", "max_accel_deviation_m_s2", "gravity_m_s2",
])
def test_new_thresholds_must_be_finite_nonnegative(parameter):
    for bad in (float("nan"), float("inf"), -1, True):
        with pytest.raises(ThresholdError):
            score(hold_and_square(), **{parameter: bad})


def test_cli_and_reports_expose_detection_and_override(tmp_path):
    write_session(tmp_path, hold_and_square(hold=6), xyz=[[0, 0, 0]])
    assert main([str(tmp_path)]) == 2
    assert main([str(tmp_path), "--min-hold", "6"]) == 0
    saved = json.loads((tmp_path / "walkcheck.json").read_text())
    assert saved["parameters"]["min_hold_s"] == 6
    assert saved["holds"]["start"]["duration_s"] == 6
    text = (tmp_path / "walkcheck.md").read_text()
    assert "Start hold: 0.000–6.000 s; duration 6.000 s" in text
    assert "PROVISIONAL" in text
    with pytest.raises(SystemExit) as caught:
        main([str(tmp_path), "--min-hold", "nan"])
    assert caught.value.code == 2


def test_cli_threads_all_detection_limits(tmp_path):
    write_session(tmp_path, hold_and_square(), xyz=[[0, 0, 0]])
    options = {
        "--min-hold": ("min_hold_s", 9),
        "--max-hold-speed": ("max_hold_speed_m_s", .03),
        "--max-angular-speed": ("max_angular_speed_deg_s", 2),
        "--significant-motion": ("significant_motion_m", .15),
        "--significant-rotation": ("significant_rotation_deg", 8),
        "--max-imu-gap": ("max_imu_gap_s", .2),
        "--max-accel-deviation": ("max_accel_deviation_m_s2", .3),
        "--gravity": ("gravity_m_s2", 9.8),
    }
    path, _ = imu_file(tmp_path)
    args = [str(tmp_path), "--imu-file", str(path)]
    for flag, (_, value) in options.items():
        args.extend([flag, str(value)])
    assert main(args) == 0
    report = json.loads((tmp_path / "walkcheck.json").read_text())
    assert report["holds"]["imu_used"]
    for name, value in options.values():
        assert report["parameters"][name] == value


def test_motion_before_holds_does_not_satisfy_route_gate():
    # Sub-threshold excursions accumulate >2 m of motion before the hold.
    points = [(i * .2, .09 if i % 2 else -.09) for i in range(30)]
    points += [(6, 0), (16, 0), (18, .15), (20, .05), (30, .05)]
    result = score(line(points), min_excursion_m=0)
    assert result["evaluated"]
    assert result["result_reasons"] == ["stationary_trajectory"]
    assert result["motion"]["path_length_m"] == pytest.approx(.25)


def test_session_uses_optional_imu_and_explicit_missing_file_fails(tmp_path):
    write_session(tmp_path, hold_and_square(), xyz=[[0, 0, 0]])
    path, data = imu_file(tmp_path)
    assert evaluate_session(tmp_path, write=False)["holds"]["imu_used"]
    data[:, 3] = 1
    save_imu(path, data)
    assert evaluate_session(tmp_path, write=False)["result"] == "NOT-EVALUABLE"
    report = evaluate_session(tmp_path, imu_file=tmp_path / "missing.csv", write=False)
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["invalid_imu"]
