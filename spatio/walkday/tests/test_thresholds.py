"""S4: one source for the wall gate, guard band and return gate, and
walkday.conf's WALL_GATE_M reaches every stage that applies it. No Docker:
the process.sh runs use test_process_origins' stubbed harness."""

import inspect
import io
import json
import math
import re
import shutil
import subprocess
import tokenize
from pathlib import Path

import pytest

from spatio import thresholds, wall_scoring
from spatio.expspec.constants import PROPOSED_THRESHOLDS
from spatio.series import adapter as series_adapter
from spatio.series import cli as series_cli
from spatio.series import validate as series_validate
from spatio.walkcheck import defaults as walkcheck_defaults
from spatio.walkcheck import evaluate as walkcheck_evaluate
from spatio.walkcheck import walls as walkcheck_walls
from spatio.walkcheck.cli import main as walkcheck_main
from spatio.walkday import result as walkday_result
from spatio.walkday.tests.test_process_origins import process_harness  # noqa: F401  (pytest fixture)
from spatio.walkday.tests.test_result import quiet
from spatio.wallpatch import cli as wallpatch_cli
from spatio.wallpatch import measure as wallpatch_measure

REPO = Path(__file__).resolve().parents[3]
FIXTURE = REPO / "spatio" / "io" / "fixtures" / "map_sharp_wall"
CONF = REPO / "spatio" / "walkday" / "walkday.conf"

# Every module that applies the wall gate, the guard band or the return gate.
GATE_FILES = (
    "spatio/wall_scoring.py",
    "spatio/walkcheck/defaults.py",
    "spatio/walkcheck/walls.py",
    "spatio/walkcheck/evaluate.py",
    "spatio/walkcheck/trajectory.py",
    "spatio/walkcheck/cli.py",
    "spatio/wallpatch/measure.py",
    "spatio/wallpatch/cli.py",
    "spatio/series/validate.py",
    "spatio/series/adapter.py",
    "spatio/series/cli.py",
    "spatio/series/report.py",
    "spatio/expspec/constants.py",
    "spatio/expspec/template.py",
    "spatio/walkday/result.py",
)
GATE_VALUES = (0.05, 5.0, 0.04, 4.0, 0.10, 10.0)
GATE_WORDS = re.compile(r"gate|guard|pass_max|return_pass|return_threshold", re.I)
_IDENT = re.compile(r"^-{0,2}[A-Za-z0-9_-]+$")


def _default(fn, name):
    return inspect.signature(fn).parameters[name].default


def _conf_value(text, key):
    match = re.search(r"^%s=(\S+)" % key, text, re.M)
    assert match, key
    return float(match.group(1))


def test_shared_values_are_unchanged():
    # Plumbing only: these are the values main shipped with.
    assert thresholds.WALL_GATE_M == 0.05
    assert thresholds.WALL_GATE_CM == 5.0
    assert thresholds.GUARD_BAND_M == 0.040
    assert thresholds.RETURN_GATE_M == 0.10


def test_every_stage_defaults_to_the_shared_gates():
    gate, guard, ret = thresholds.WALL_GATE_M, thresholds.GUARD_BAND_M, thresholds.RETURN_GATE_M
    # walkcheck
    assert walkcheck_defaults.WALL_GATE_CM == thresholds.WALL_GATE_CM == gate * 100
    assert _default(walkcheck_walls.wall_sharpness, "gate_cm") == thresholds.WALL_GATE_CM
    assert _default(walkcheck_evaluate.evaluate_session, "wall_gate_m") == gate
    assert walkcheck_defaults.RETURN_PASS_M == ret
    # wallpatch measure
    assert wallpatch_measure.P95_GATE_M == wallpatch_measure.SEPARATION_GATE_M == gate
    assert _default(wallpatch_measure.measure_map, "gate_m") == gate
    args = wallpatch_cli._parser().parse_args(["measure", "s", "--patches", "p", "--out", "o"])
    assert args.wall_gate_m == gate
    # wall scoring guard band
    assert wall_scoring.PASS_MAX_M == guard
    assert wall_scoring.PASS_MAX_LABEL == thresholds.GUARD_BAND_LABEL
    # series validate / adapter / cli
    assert series_validate.WALL_GATE_M == gate
    assert series_validate.RETURN_THRESHOLD_M == ret
    assert _default(series_adapter.adapt_walkcheck, "gate_m") == gate
    args = series_cli._parser().parse_args(["adapt", "wc.json", "--metadata", "m.json", "--out", "o.json"])
    assert args.wall_gate_m == gate
    # expspec
    assert PROPOSED_THRESHOLDS["wall_p95_gate_m"] == gate
    assert PROPOSED_THRESHOLDS["revisit_separation_gate_m"] == gate
    assert PROPOSED_THRESHOLDS["return_error_threshold_m"] == ret
    # walkday result and the shipped settings file
    assert walkday_result._parser().parse_args(["session"]).wall_gate_m == gate
    assert _conf_value(CONF.read_text(), "WALL_GATE_M") == gate, (
        "walkday.conf WALL_GATE_M differs from spatio/thresholds.py WALL_GATE_M")


def gate_literals(source):
    """(line, literal) for each gate-valued number whose scope names a gate.

    The scope is the innermost bracket group the number sits in directly (a
    call's arguments, a dict's items), or its logical line outside brackets.
    Code tokens only: comments and prose strings do not count, identifier-like
    strings ("--wall-gate-m", dict keys) do."""
    hits = []
    groups = [{"named": False, "numbers": []}]

    def close():
        group = groups.pop()
        if group["named"]:
            hits.extend(group["numbers"])

    for tok in tokenize.generate_tokens(io.StringIO(source).readline):
        top = groups[-1]
        if tok.type == tokenize.OP and tok.string in "([{":
            groups.append({"named": False, "numbers": []})
        elif tok.type == tokenize.OP and tok.string in ")]}":
            close()
        elif tok.type == tokenize.NUMBER:
            try:
                value = float(tok.string)
            except ValueError:
                value = None
            if value is not None and any(math.isclose(value, v) for v in GATE_VALUES):
                top["numbers"].append((tok.start[0], tok.string))
        elif tok.type == tokenize.NAME and GATE_WORDS.search(tok.string):
            top["named"] = True
        elif tok.type == tokenize.STRING:
            body = tok.string.strip("rbuRBU").strip("'\"")
            if _IDENT.match(body) and GATE_WORDS.search(body):
                top["named"] = True
        elif tok.type in (tokenize.NEWLINE, tokenize.ENDMARKER) and len(groups) == 1:
            close()
            groups.append({"named": False, "numbers": []})
    return sorted(hits)


def test_gate_literal_detector_catches_a_planted_copy():
    planted = (
        "P95_GATE_M = 0.05\n"
        "def f(gate_m=0.05):\n    pass\n"
        'ap.add_argument("--wall-gate-m", type=float, default=0.05)\n'
        'T = {\n    "wall_p95_gate_m": 0.05,\n}\n'
        "PASS_MAX_M = 0.040\n"
        "RETURN_THRESHOLD_M = 0.10\n"
        "WALL_GATE_CM = 5.0\n"
        "parser.add_argument(\n    \"--wall-gate-m\",\n    default=0.05,\n)\n"
    )
    assert len(gate_literals(planted)) == 8
    clean = (
        '"""The 5 cm gate is 0.05 m."""\n'
        "# gate 0.05\n"
        "MAX_ENDPOINT_SPREAD_M = 0.05\n"
        "GATE = WALL_GATE_M\n"
        'SPEC = {\n    "wall_p95_gate_m": WALL_GATE_M,\n    "windows_s": {"start": 10, "end": 10},\n}\n'
    )
    assert gate_literals(clean) == []


@pytest.mark.parametrize("rel", GATE_FILES)
def test_no_module_keeps_its_own_gate_literal(rel):
    hits = gate_literals((REPO / rel).read_text(encoding="utf-8"))
    assert hits == [], (
        f"{rel} holds its own gate number {hits}; import it from spatio.thresholds")


# Stage CLIs: --wall-gate-m reaches the recorded output, and a bad gate is refused.

def _session(tmp_path):
    session = tmp_path / "session"
    session.mkdir()
    for name in ("trajectory.tum", "map.pcd", "manifest.json"):
        shutil.copyfile(FIXTURE / name, session / name)
    return session


def test_walkcheck_records_the_gate_it_was_given(tmp_path):
    session = _session(tmp_path)
    quiet(walkcheck_main, [str(session)])
    params = json.loads((session / "walkcheck.json").read_text())["parameters"]
    assert (params["wall_gate_m"], params["wall_gate_cm"]) == (0.05, 5.0)
    quiet(walkcheck_main, [str(session), "--wall-gate-m", "0.03"])
    report = json.loads((session / "walkcheck.json").read_text())
    assert (report["parameters"]["wall_gate_m"], report["parameters"]["wall_gate_cm"]) == (0.03, 3.0)
    assert report["walls"]["proposed_gate_cm"] == 3.0


@pytest.mark.parametrize("bad", ["0", "-0.05", "nan", "inf"])
def test_walkcheck_refuses_a_bad_gate(tmp_path, bad):
    session = _session(tmp_path)
    with pytest.raises(SystemExit) as exc:
        quiet(walkcheck_main, [str(session), "--wall-gate-m", bad])
    assert exc.value.code == 2
    assert not (session / "walkcheck.json").exists()


def _suggested(tmp_path, session):
    patches = tmp_path / "patches.json"
    assert quiet(wallpatch_cli.main, ["suggest", str(session / "map.pcd"), "--n", "2",
                                      "--out", str(patches)]) == 0
    return patches


def test_wallpatch_measure_records_the_gate_it_was_given(tmp_path):
    session = _session(tmp_path)
    patches = _suggested(tmp_path, session)
    for gate in (None, 0.03):
        out = tmp_path / ("out-%s" % gate)
        extra = [] if gate is None else ["--wall-gate-m", str(gate)]
        assert quiet(wallpatch_cli.main, ["measure", str(session), "--patches", str(patches),
                                          "--out", str(out), *extra]) == 0
        report = json.loads((out / "wallpatch.json").read_text())
        want = thresholds.WALL_GATE_M if gate is None else gate
        assert report["gates"]["p95_point_to_plane_max_m"] == want
        assert report["gates"]["revisit_separation_max_m"] == want
        assert report["doubled_wall_rule"]["pass_needs_band_under_m"] == want
        assert report["gates"]["pass_max_m"] == thresholds.GUARD_BAND_M
        for patch in report["patches"]:
            if patch["status"] == "MEASURED":
                assert patch["gate_p95_pass"] == (patch["residuals"]["p95_abs_m"] <= want)


@pytest.mark.parametrize("bad", ["0", "-0.05", "nan", "inf"])
def test_wallpatch_measure_refuses_a_bad_gate(tmp_path, bad):
    session = _session(tmp_path)
    patches = _suggested(tmp_path, session)
    out = tmp_path / "out"
    assert quiet(wallpatch_cli.main, ["measure", str(session), "--patches", str(patches),
                                      "--out", str(out), "--wall-gate-m", bad]) != 0
    assert not (out / "wallpatch.json").exists()


def test_guard_band_never_exceeds_a_stricter_gate():
    # 3.5 cm is under the 4 cm guard band but over a 3 cm gate.
    assert wallpatch_measure._guard_band_reason(0.035, 0.01, 0.01) is None
    reason = wallpatch_measure._guard_band_reason(0.035, 0.01, 0.01, 0.03)
    assert reason is not None and "0.030 m pass limit" in reason


# process.sh: WALL_GATE_M from the settings file reaches every stage.

def _run(harness, settings):
    root, script, rec, spec, patches, env = harness
    doc = json.loads(patches.read_text())
    for patch in doc["patches"]:
        del patch["origin"]          # hand-selected file matching the spec
    patches.write_text(json.dumps(doc))
    args = ["bash", str(script), str(rec), "run", "--sessions-root", str(root / "sessions"),
            "--spec", str(spec), "--patches", str(patches)]
    if settings is not None:
        args += ["--settings", str(settings)]
    proc = subprocess.run(args, env=env, stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=300)
    return proc, root / "sessions" / "run"


def _recorded_gates(session):
    walkcheck = json.loads((session / "walkcheck.json").read_text())["parameters"]
    wallpatch = json.loads((session / "wallpatch" / "wallpatch.json").read_text())["gates"]
    walk = json.loads((session / "walk_result.json").read_text())
    return {
        "walkcheck": walkcheck["wall_gate_m"],
        "walkcheck_cm": walkcheck["wall_gate_cm"],
        "wallpatch_p95": wallpatch["p95_point_to_plane_max_m"],
        "wallpatch_separation": wallpatch["revisit_separation_max_m"],
        "walk_result": walk["walls"]["gate_m"],
    }


def test_settings_wall_gate_reaches_every_stage(process_harness, tmp_path):
    text = CONF.read_text()
    assert "\nWALL_GATE_M=0.05 " in text           # the edit below must really apply
    conf = tmp_path / "gate3.conf"
    conf.write_text(text.replace("\nWALL_GATE_M=0.05 ", "\nWALL_GATE_M=0.03 "))
    proc, session = _run(process_harness, conf)
    assert (session / "walk_result.json").is_file(), proc.stdout + proc.stderr
    assert _recorded_gates(session) == {
        "walkcheck": 0.03, "walkcheck_cm": 3.0, "wallpatch_p95": 0.03,
        "wallpatch_separation": 0.03, "walk_result": 0.03,
    }


def test_default_settings_keep_the_shared_gate_and_pass(process_harness):
    proc, session = _run(process_harness, None)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    gate = thresholds.WALL_GATE_M
    assert _recorded_gates(session) == {
        "walkcheck": gate, "walkcheck_cm": thresholds.WALL_GATE_CM, "wallpatch_p95": gate,
        "wallpatch_separation": gate, "walk_result": gate,
    }
    walk = json.loads((session / "walk_result.json").read_text())
    assert walk["status"] == "PASS"
    assert len(walk["wall_patches"]["verified"]) == 2
