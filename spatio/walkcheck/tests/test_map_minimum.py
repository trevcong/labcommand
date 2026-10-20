"""Map input sufficiency is independent of trajectory and wall quality."""

import json

import numpy as np
import pytest

from spatio.walkcheck.cli import main
from spatio.walkcheck.evaluate import evaluate_session
from spatio.walkcheck.synthetic import (
    walking_loop, write_pcd_ascii, write_pcd_binary, write_session,
)
from spatio.walkcheck.trajectory import ThresholdError


def make_session(path, count, writer=write_pcd_ascii, closure=.05):
    # Different geometry/rate/epoch from the committed sample; no real recording.
    rows = walking_loop(side=1.7, rate=7, origin=123.4,
                        holds=(11, 13), offset=(closure, 0, 0))
    write_session(path, rows)
    xyz = np.column_stack((np.arange(count), np.zeros(count), np.zeros(count)))
    writer(path / "map.pcd", xyz)


@pytest.mark.parametrize("writer", [write_pcd_ascii, write_pcd_binary])
@pytest.mark.parametrize("count", [0, 1, 3, 24])
@pytest.mark.parametrize("closure", [.05, .15])
def test_insufficient_map_is_not_evaluable(tmp_path, count, writer, closure):
    make_session(tmp_path, count, writer, closure)
    report = evaluate_session(tmp_path, write=False)
    assert report["result"] == "NOT-EVALUABLE"
    assert "too_few_map_points" in report["reasons"]
    assert report["walls"]["finite_point_count"] == count
    assert report["parameters"]["min_map_points"] == 25
    assert report["return_to_start"]["error_3d_cm"] == pytest.approx(closure * 100)
    assert f"{count} finite XYZ points" in report["detail"]


@pytest.mark.parametrize("count", [25, 32])
def test_sufficient_map_without_wall_patches_keeps_trajectory_verdict(tmp_path, count):
    make_session(tmp_path, count)
    report = evaluate_session(tmp_path, write=False)
    assert report["result"] == "PASS"
    assert report["reasons"] == []
    assert report["walls"]["reason"] == "no_vertical_planar_patches"


@pytest.mark.parametrize("writer", [write_pcd_ascii, write_pcd_binary])
@pytest.mark.parametrize("finite_count", [0, 24, 25])
def test_only_finite_xyz_points_count(tmp_path, writer, finite_count):
    make_session(tmp_path, 30, writer)
    xyz = np.column_stack((np.arange(30), np.zeros(30), np.zeros(30))).astype(float)
    xyz[finite_count:, 1] = np.nan
    xyz[-1, 2] = np.inf
    writer(tmp_path / "map.pcd", xyz)
    report = evaluate_session(tmp_path, write=False)
    assert report["walls"]["point_count"] == 30
    assert report["walls"]["finite_point_count"] == finite_count
    assert report["result"] == ("PASS" if finite_count >= 25 else "NOT-EVALUABLE")


def test_cli_override_and_reports(tmp_path):
    make_session(tmp_path, 32)
    assert main([str(tmp_path)]) == 0
    assert main([str(tmp_path), "--min-map-points", "33"]) == 2
    report = json.loads((tmp_path / "walkcheck.json").read_text())
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["too_few_map_points"]
    assert report["parameters"]["min_map_points"] == 33
    text = (tmp_path / "walkcheck.md").read_text()
    assert text.startswith("# Walk check: NOT-EVALUABLE")
    assert "32 finite XYZ points" in text
    assert "33" in text
    assert "PROVISIONAL" in text
    assert (tmp_path / "topdown.png").read_bytes().startswith(b"\x89PNG")
    assert main([str(tmp_path), "--min-map-points", "32"]) == 0


@pytest.mark.parametrize("count", [0, 3])
def test_cli_rejects_empty_and_tiny_maps(tmp_path, count):
    make_session(tmp_path, count)
    assert main([str(tmp_path)]) == 2
    assert json.loads((tmp_path / "walkcheck.json").read_text())["result"] == "NOT-EVALUABLE"


def test_empty_map_cannot_pass_lowest_allowed_minimum(tmp_path):
    make_session(tmp_path, 0)
    report = evaluate_session(tmp_path, min_map_points=1, write=False)
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["too_few_map_points"]


@pytest.mark.parametrize("bad", [0, -1, 1.5, True, float("nan"), float("inf"), "25", None])
def test_api_rejects_invalid_map_minimum(tmp_path, bad):
    with pytest.raises(ThresholdError, match="min_map_points"):
        evaluate_session(tmp_path, min_map_points=bad)
    assert not (tmp_path / "walkcheck.json").exists()


@pytest.mark.parametrize("bad", ["0", "-1", "1.5", "nan", "inf"])
def test_cli_rejects_invalid_map_minimum(tmp_path, bad):
    with pytest.raises(SystemExit) as caught:
        main([str(tmp_path), "--min-map-points", bad])
    assert caught.value.code == 2
    assert not (tmp_path / "walkcheck.json").exists()
