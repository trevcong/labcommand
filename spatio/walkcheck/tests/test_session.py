import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from spatio.walkcheck.cli import main
from spatio.walkcheck.defaults import HISTORICAL_LABEL
from spatio.walkcheck.evaluate import evaluate_session
from spatio.walkcheck.synthetic import (
    doubled_wall,
    hold_and_square,
    insert_time_gap,
    sharp_wall,
    write_pcd_ascii,
    write_session,
)

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "sample_session"
ROOT = Path(__file__).resolve().parents[3]


def test_writes_the_three_report_files(tmp_path):
    dest = tmp_path / "session"
    write_session(dest, hold_and_square(offset=(0.05, 0.0, 0.0)), xyz=sharp_wall(count=1500, seed=3))
    report = evaluate_session(dest)
    assert report["result"] == "PASS"
    assert (dest / "walkcheck.json").is_file()
    assert (dest / "walkcheck.md").is_file()
    png = dest / "topdown.png"
    assert png.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    saved = json.loads((dest / "walkcheck.json").read_text(encoding="utf-8"))
    assert saved["schema"] == "spatio.walkcheck.v1"
    assert saved["result"] == "PASS"
    text = (dest / "walkcheck.md").read_text(encoding="utf-8")
    assert text.startswith("# Walk check: PASS")
    assert "was not moved onto the start" in text


def test_wall_gate_does_not_change_the_walk_verdict(tmp_path):
    closed = tmp_path / "closed"
    write_session(closed, hold_and_square(offset=(0.05, 0.0, 0.0)), xyz=doubled_wall())
    closed_report = evaluate_session(closed)
    assert closed_report["result"] == "PASS"
    assert closed_report["walls"]["proposed_gate_result"] == "FAIL"
    assert closed_report["walls"]["doubled_wall"] is True

    open_walk = tmp_path / "open"
    write_session(open_walk, hold_and_square(offset=(0.15, 0.0, 0.0)), xyz=sharp_wall())
    open_report = evaluate_session(open_walk)
    assert open_report["result"] == "FAIL"
    assert open_report["walls"]["proposed_gate_result"] == "PASS"


def test_missing_map_still_scores_the_trajectory(tmp_path):
    dest = tmp_path / "session"
    write_session(dest, hold_and_square(offset=(0.05, 0.0, 0.0)))
    report = evaluate_session(dest)
    assert report["result"] == "PASS"
    assert report["walls"]["status"] == "NOT-EVALUABLE"
    assert report["walls"]["reason"] == "missing_map"
    assert (dest / "topdown.png").is_file()


def test_missing_trajectory(tmp_path):
    dest = tmp_path / "session"
    dest.mkdir()
    write_pcd_ascii(dest / "map.pcd", sharp_wall(count=200, seed=4))
    report = evaluate_session(dest)
    assert report["result"] == "NOT-EVALUABLE"
    assert report["reasons"] == ["missing_trajectory"]
    assert (dest / "walkcheck.md").is_file()


def test_not_evaluable_still_writes_a_png(tmp_path):
    dest = tmp_path / "session"
    rows = insert_time_gap(hold_and_square(), after_t=10.0, extra_s=2.0)
    write_session(dest, rows, xyz=sharp_wall(count=400, seed=5))
    report = evaluate_session(dest)
    assert report["result"] == "NOT-EVALUABLE"
    assert (dest / "topdown.png").read_bytes().startswith(b"\x89PNG")


def test_fixture_sample_session(tmp_path):
    dest = tmp_path / "sample"
    shutil.copytree(FIXTURE, dest)
    report = evaluate_session(dest)
    assert report["result"] == "PASS"
    assert report["return_to_start"]["error_3d_cm"] == pytest.approx(5.0, abs=0.05)
    assert HISTORICAL_LABEL in report["notes"]
    assert any("synthetic" in note.lower() for note in report["notes"])
    text = (dest / "walkcheck.md").read_text(encoding="utf-8")
    assert HISTORICAL_LABEL in text


def test_cli_exit_codes(tmp_path):
    passing = tmp_path / "pass"
    failing = tmp_path / "fail"
    broken = tmp_path / "broken"
    write_session(passing, hold_and_square(offset=(0.05, 0.0, 0.0)))
    write_session(failing, hold_and_square(offset=(0.15, 0.0, 0.0)))
    write_session(broken, insert_time_gap(hold_and_square(), after_t=10.0, extra_s=2.0))

    assert main([str(passing)]) == 0
    assert main([str(failing)]) == 1
    assert main([str(broken)]) == 2
    assert main([str(tmp_path / "missing")]) == 3

    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    env["MPLBACKEND"] = "Agg"
    completed = subprocess.run(
        [sys.executable, "-m", "spatio.walkcheck", str(passing), "--stationary-window", "5"],
        cwd=str(ROOT),
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0
    assert completed.stdout.splitlines()[-1].startswith("PASS")


def test_compressed_map_does_not_crash(tmp_path):
    dest = tmp_path / "session"
    write_session(dest, hold_and_square(offset=(0.05, 0.0, 0.0)))
    (dest / "map.pcd").write_bytes(
        b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        b"WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\nxxxx"
    )
    report = evaluate_session(dest)
    assert report["result"] == "PASS"
    assert report["walls"]["status"] == "NOT-EVALUABLE"
    assert "binary_compressed" in report["walls"]["reason"]
