"""Q104/#91: stricter-only settings, recorded gates and approval defaults."""

import inspect
import json
import math
import shutil
from pathlib import Path

import pytest

from spatio import thresholds
from spatio.series import adapter, cli as series_cli
from spatio.walkcheck.cli import main as walkcheck_main
from spatio.walkday import result
from spatio.walkday.tests.test_process_origins import process_harness  # noqa: F401
from spatio.walkday.tests.test_result import make_session, quiet, run
from spatio.walkday.tests.test_thresholds import CONF, _run
from spatio.wallpatch.cli import main as wallpatch_main


@pytest.fixture(scope="module")
def scored_source(tmp_path_factory):
    session = make_session(tmp_path_factory.mktemp("strict-gates"), "run")
    assert run(session) == 0
    return session


@pytest.fixture
def scored_session(scored_source, tmp_path):
    return Path(shutil.copytree(scored_source, tmp_path / "run"))


@pytest.mark.parametrize("gate", [0.05, 0.03, math.nextafter(0.05, 0)])
def test_ceiling_and_stricter_gates_are_allowed(scored_session, gate):
    assert thresholds.wall_gate(str(gate)) == gate
    assert run(scored_session, "--wall-gate-m", str(gate)) == 0
    assert json.loads((scored_session / "walk_result.json").read_text())["walls"]["gate_m"] == gate


@pytest.mark.parametrize("gate", ["0.08", "0.05000000000000001", "1"])
def test_all_stage_clis_reject_looser_gates(scored_session, tmp_path, gate):
    with pytest.raises(SystemExit) as exc:
        quiet(walkcheck_main, [str(scored_session), "--wall-gate-m", gate])
    assert exc.value.code == 2
    out = tmp_path / "patch-output"
    assert quiet(wallpatch_main, ["measure", str(scored_session), "--patches",
                                 str(scored_session / "wallpatch" / "patches.json"),
                                 "--out", str(out), "--wall-gate-m", gate]) == 2
    assert not out.exists()
    adapted = tmp_path / "adapted.json"
    with pytest.raises(SystemExit) as exc:
        quiet(series_cli.main, ["adapt", str(scored_session / "walkcheck.json"),
                               "--metadata", "unused.json", "--out", str(adapted),
                               "--wall-gate-m", gate])
    assert exc.value.code == 2
    assert not adapted.exists()
    assert run(scored_session, "--wall-gate-m", gate) == 2
    assert not (scored_session / "walk_result.json").exists()
    assert (scored_session / result.RETIRED).exists()
    report = json.loads((scored_session / "walkday.json").read_text())
    assert report["stopped_at"] == "result"
    assert "stricter-only" in " ".join(report["reasons"])


@pytest.mark.parametrize("gate", ["0.08", "0.05000000000000001", "nan", "inf", "0", "-0.01"])
def test_process_rejects_bad_settings_before_any_stage(process_harness, tmp_path, gate):
    conf = tmp_path / "bad.conf"
    conf.write_text(CONF.read_text().replace("\nWALL_GATE_M=0.05 ", f"\nWALL_GATE_M={gate} "))
    proc, session = _run(process_harness, conf)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert "WALL_GATE_M" in proc.stderr
    assert not session.exists()


RECORDED = [
    ("walkcheck.json", "parameters", "wall_gate_m", 1),
    ("walkcheck.json", "parameters", "wall_gate_cm", 100),
    ("walkcheck.json", "walls", "proposed_gate_cm", 100),
    ("wallpatch/wallpatch.json", "gates", "p95_point_to_plane_max_m", 1),
    ("wallpatch/wallpatch.json", "gates", "revisit_separation_max_m", 1),
    ("wallpatch/wallpatch.json", "doubled_wall_rule", "pass_needs_band_under_m", 1),
]


@pytest.mark.parametrize("rel,block,key,scale", RECORDED)
@pytest.mark.parametrize("bad", ["missing", None, True, "0.05", 0, -1, 0.08, 0.03])
def test_result_rejects_missing_invalid_or_inconsistent_stage_gates(scored_session, rel, block, key, scale, bad):
    path = scored_session / rel
    report = json.loads(path.read_text())
    if bad == "missing":
        del report[block][key]
    else:
        report[block][key] = bad * scale if type(bad) in (int, float) else bad
    path.write_text(json.dumps(report))
    # Even absent stage-1 provenance must not bypass the recorded-gate checks.
    (scored_session / "logs" / "provenance.json").unlink()
    assert run(scored_session) == 3
    assert not (scored_session / "walk_result.json").exists()
    stopped = json.loads((scored_session / "walkday.json").read_text())
    assert stopped["stopped_at"] == "result"
    assert Path(rel).name in " ".join(stopped["reasons"])


def test_result_rejects_disagreement_between_stage_gates(scored_session):
    path = scored_session / "wallpatch" / "wallpatch.json"
    report = json.loads(path.read_text())
    report["gates"]["p95_point_to_plane_max_m"] = 0.03
    report["gates"]["revisit_separation_max_m"] = 0.03
    report["doubled_wall_rule"]["pass_needs_band_under_m"] = 0.03
    path.write_text(json.dumps(report))
    assert run(scored_session) == 3
    assert not (scored_session / "walk_result.json").exists()


def test_result_cannot_loosen_consistently_recorded_stricter_gates(scored_session):
    for rel, block, key, scale in RECORDED:
        path = scored_session / rel
        report = json.loads(path.read_text())
        report[block][key] = 0.03 * scale
        path.write_text(json.dumps(report))
    assert run(scored_session) == 3  # default 5 cm would loosen the recorded 3 cm
    assert not (scored_session / "walk_result.json").exists()
    assert run(scored_session, "--wall-gate-m", "0.03") == 0


def test_adapter_and_cli_default_to_configured_approval(scored_session, tmp_path):
    state = next(line.split("=", 1)[1].split()[0] for line in CONF.read_text().splitlines()
                 if line.startswith("WALL_GATE_STATE="))
    assert state == thresholds.WALL_GATE_STATE == "approved"
    assert inspect.signature(adapter.adapt_walkcheck).parameters["gate_state"].default == state
    assert result._parser().parse_args([str(scored_session)]).wall_gate_state == state
    wc_path = scored_session / "walkcheck.json"
    wc = json.loads(wc_path.read_text())
    walk = json.loads((scored_session / "walk_result.json").read_text())
    metadata = json.loads((scored_session / "manifest.json").read_text())
    metadata.update(recorded_at=walk["recorded_at"], physical_reference=None)
    assert adapter.adapt_walkcheck(wc, metadata, str(wc_path))["walls"]["gate_state"] == state
    meta = tmp_path / "metadata.json"
    meta.write_text(json.dumps(metadata))
    out = tmp_path / "adapted.json"
    args = ["adapt", str(wc_path), "--metadata", str(meta), "--out", str(out)]
    assert quiet(series_cli.main, args) == 0
    assert json.loads(out.read_text())["walls"]["gate_state"] == state
    assert quiet(series_cli.main, [*args, "--wall-gate-state", "proposed"]) == 0
    assert json.loads(out.read_text())["walls"]["gate_state"] == "proposed"
