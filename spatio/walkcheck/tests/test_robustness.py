"""Synthetic walk-day cases; expected closure comes from explicit inputs.

These are tests of observed trajectory scoring, not physical accuracy or
featureless-scene performance. A small map meeting the finite-point minimum isolates this gate;
wall quality is deliberately not claimed.
"""

import json
import math

import numpy as np
import pytest

from spatio.walkcheck.cli import main
from spatio.walkcheck.evaluate import evaluate_session
from spatio.walkcheck.synthetic import (
    insert_time_gap,
    jitter_timestamps,
    sharp_wall,
    walking_loop,
    write_session,
)


SPEEDS = [(speed,) * 4 for speed in (0.2, 0.5, 0.9, 1.5)]
SPEEDS += [(0.2, 1.5, 0.4, 1.1)]


def score(tmp_path, rows, **kwargs):
    write_session(tmp_path, rows, xyz=sharp_wall(count=32))
    return evaluate_session(tmp_path, write=False, **kwargs)


@pytest.mark.parametrize("speeds", SPEEDS)
@pytest.mark.parametrize("offset", [
    (0, 0, 0), (.05, 0, 0), (.099, 0, 0), (.10, 0, 0),
    (.101, 0, 0), (.15, 0, 0), (.08, .08, 0), (0, 0, -.15),
])
def test_known_closure_across_speeds(tmp_path, speeds, offset):
    truth = math.dist((0, 0, 0), offset)
    report = score(tmp_path, walking_loop(speeds=speeds, offset=offset))
    assert report["result"] == ("PASS" if truth < .10 else "FAIL")
    assert report["reasons"] == ([] if truth < .10 else ["return_error"])
    assert report["return_to_start"]["error_3d_m"] == pytest.approx(truth, abs=2e-6)
    assert report["return_to_start"]["alignment"] == "none"
    assert len(report["holds"]["intervals"]) == 2


@pytest.mark.parametrize("side,rate,origin", [(1.3, 7, 143.25), (4.7, 23, 1_800_000_000)])
@pytest.mark.parametrize("speeds", SPEEDS)
def test_clean_new_geometry_rate_and_epoch(tmp_path, side, rate, origin, speeds):
    report = score(tmp_path, walking_loop(side=side, rate=rate, origin=origin,
                                         speeds=speeds, offset=(.03, -.04, 0),
                                         holds=(11, 13)))
    assert report["result"] == "PASS"
    assert report["return_to_start"]["error_3d_cm"] == pytest.approx(5, abs=.001)
    assert report["holds"]["start"]["t_start"] == origin
    assert report["holds"]["end"]["duration_s"] == pytest.approx(13, abs=.001)


@pytest.mark.parametrize("speeds", SPEEDS)
@pytest.mark.parametrize("stops", [(2, 0, 4), (9, 12, 10)])
@pytest.mark.parametrize("closure", [.05, .15])
def test_mid_loop_stops_do_not_replace_endpoints(tmp_path, speeds, stops, closure):
    report = score(tmp_path, walking_loop(speeds=speeds, stops=stops,
                                         offset=(closure, 0, 0)))
    assert report["result"] == ("PASS" if closure < .10 else "FAIL")
    assert report["return_to_start"]["error_3d_m"] == pytest.approx(closure, abs=2e-6)
    assert report["holds"]["start"]["t_start"] == 0
    assert len(report["holds"]["intervals"]) == (5 if min(stops) >= 8 else 2)
    if min(stops) >= 8:
        assert report["holds"]["flags"] == ["possible_repositioning"]


@pytest.mark.parametrize("speeds", SPEEDS)
@pytest.mark.parametrize("reposition", [(-.4, 0, 0), (0, .6, .2)])
def test_reposition_before_start_hold_never_passes(tmp_path, speeds, reposition):
    report = score(tmp_path, walking_loop(speeds=speeds, reposition=reposition,
                                         offset=(.05, 0, 0), stops=(0, 12, 0)))
    assert report["result"] == "NOT-EVALUABLE"
    assert "motion_before_start_hold" in report["reasons"]
    assert report["return_to_start"] is None
    assert report["holds"]["start"]["t_start"] == pytest.approx(
        math.dist(reposition, (0, 0, 0)) / speeds[0], abs=.0001)


@pytest.mark.parametrize("holds", [(0, 10), (10, 0), (0, 0), (1, 10),
                                  (10, 1), (7.5, 10), (10, 7.5), (7.5, 7.5)])
@pytest.mark.parametrize("stops", [(0, 0, 0), (11, 0, 12)])
def test_missing_or_short_endpoint_holds_never_pass(tmp_path, holds, stops):
    report = score(tmp_path, walking_loop(holds=holds, stops=stops))
    assert report["result"] == "NOT-EVALUABLE"
    assert report["return_to_start"] is None
    assert report["reasons"]


@pytest.mark.parametrize("seed", [3, 29, 101])
@pytest.mark.parametrize("speeds", SPEEDS)
@pytest.mark.parametrize("closure", [.05, .15])
def test_timestamp_jitter_preserves_verdict(tmp_path, seed, speeds, closure):
    rows = walking_loop(speeds=speeds, rate=17, origin=1_800_000_000,
                        offset=(closure, 0, 0), stops=(0, 11, 0))
    jittered = jitter_timestamps(rows, amplitude_s=.01, seed=seed)
    assert np.all(np.diff(np.asarray(jittered)[:, 0]) > 0)
    assert jittered == jitter_timestamps(rows, amplitude_s=.01, seed=seed)
    report = score(tmp_path, jittered)
    assert report["result"] == ("PASS" if closure < .10 else "FAIL")
    assert report["return_to_start"]["error_3d_m"] == pytest.approx(closure, abs=2e-6)


@pytest.mark.parametrize("location", ["start_hold", "route", "end_hold"])
@pytest.mark.parametrize("mode", ["shift_clock", "drop_samples"])
def test_timestamp_gaps_never_pass(tmp_path, location, mode):
    rows = walking_loop(speeds=(.5,) * 4, offset=(.05, 0, 0))
    at = {"start_hold": 3, "route": 17, "end_hold": rows[-1][0] - 7}[location]
    if mode == "shift_clock":
        rows = insert_time_gap(rows, after_t=at, extra_s=1.2)
    else:
        rows = [row for row in rows if not at < row[0] < at + 1.3]
    report = score(tmp_path, rows)
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["timestamp_gap"]
    assert report["return_to_start"] is None


def test_jitter_that_reverses_time_is_not_repaired(tmp_path):
    rows = jitter_timestamps(walking_loop(), amplitude_s=.2, seed=29)
    assert np.any(np.diff(np.asarray(rows)[:, 0]) <= 0)
    report = score(tmp_path, rows)
    assert report["result"] == "NOT-EVALUABLE"
    assert "non_increasing_timestamps" in report["reasons"]


@pytest.mark.parametrize("start,end,expected", [(0, 90, 90), (170, 190, 20),
                                              (-170, -190, -20), (15, -120, -135)])
def test_heading_only_error_is_reported_separately(tmp_path, start, end, expected):
    report = score(tmp_path, walking_loop(start_yaw_deg=start, end_yaw_deg=end))
    assert report["result"] == "PASS"
    returned = report["return_to_start"]
    assert returned["error_3d_m"] == pytest.approx(0, abs=1e-6)
    assert returned["heading_error_deg"] == pytest.approx(expected, abs=.001)
    assert returned["attitude_error_deg"] == pytest.approx(abs(expected), abs=.001)


@pytest.mark.parametrize("speeds", SPEEDS)
@pytest.mark.parametrize("stretch", [(.1, .3), (.55, .9)])
@pytest.mark.parametrize("drift", [(.05, 0, 0), (.15, 0, 0), (.08, -.08, 0), (0, 0, -.2)])
def test_featureless_stretch_proxy_preserves_injected_drift(tmp_path, speeds, stretch, drift):
    rows = walking_loop(speeds=speeds, drift=drift, drift_stretch=stretch,
                        stops=(0, 11, 0))
    # Independent known endpoint displacement, including vertical/diagonal drift.
    truth = math.dist(drift, (0, 0, 0))
    assert np.asarray(rows)[-1, 1:4] == pytest.approx(drift)
    report = score(tmp_path, rows)
    assert report["result"] == ("PASS" if truth < .10 else "FAIL")
    assert report["return_to_start"]["error_3d_m"] == pytest.approx(truth, abs=2e-6)
    assert report["return_to_start"]["alignment"] == "none"


def test_generator_has_requested_speeds_and_continuous_drift():
    rows = np.asarray(walking_loop(side=2, speeds=(.2, .5, 1, 1.5),
                                   holds=(0, 0), rate=13))
    velocities = np.linalg.norm(np.diff(rows[:, 1:4], axis=0), axis=1) / np.diff(rows[:, 0])
    assert set(np.round(velocities, 8)) == {.2, .5, 1, 1.5}
    assert np.max(np.diff(rows[:, 0])) <= 1 / 13 + 1e-12
    base = np.asarray(walking_loop(speeds=(.5,) * 4))
    faulty = np.asarray(walking_loop(speeds=(.5,) * 4, drift=(.15, 0, 0)))
    injected = faulty[:, 1:4] - base[:, 1:4]
    assert np.all(np.diff(injected[:, 0]) >= -1e-12)
    assert np.max(np.diff(injected[:, 0])) < .003
    assert injected[base[:, 0] <= 10] == pytest.approx(np.zeros_like(injected[base[:, 0] <= 10]))
    assert injected[-1] == pytest.approx([.15, 0, 0])


def test_stricter_provisional_cli_limits_apply_to_a_new_walk(tmp_path):
    rows = walking_loop(side=1.7, rate=7, holds=(10, 10), origin=123.4)
    write_session(tmp_path, rows, xyz=sharp_wall(count=32))
    assert main([str(tmp_path)]) == 0
    # Tighten, never relax, the hold limit on this independent synthetic walk.
    assert main([str(tmp_path), "--min-hold", "12"]) == 2
    report = json.loads((tmp_path / "walkcheck.json").read_text())
    assert report["parameters"]["min_hold_s"] == 12
    assert report["result"] == "NOT-EVALUABLE"
