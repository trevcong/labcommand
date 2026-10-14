"""``python -m room_memory.offline``: JSON parity with ``change-detect --evidence --json``, flags, errors."""

import pathlib
import subprocess
import sys

import numpy as np
import pytest

from room_memory import offline
from test_change_detection import CD_FIXTURES


@pytest.mark.parametrize("sensor", ["jt128", "mid360"])
def test_cli_json_matches_swift_golden(sensor, capsys):
    code = offline.main(["--evidence", "--json", str(CD_FIXTURES / f"a-{sensor}.bin"), str(CD_FIXTURES / f"b-{sensor}.bin")])
    assert code == 0
    assert capsys.readouterr().out.encode() == (CD_FIXTURES / f"{sensor}.json").read_bytes()


@pytest.mark.parametrize("sensor", ["jt128", "mid360"])
def test_bare_module_entry_point_prints_the_golden(sensor):
    result = subprocess.run(
        [sys.executable, "-m", "room_memory.offline", str(CD_FIXTURES / f"a-{sensor}.bin"),
         str(CD_FIXTURES / f"b-{sensor}.bin")],
        capture_output=True, check=True, cwd=pathlib.Path(offline.__file__).parents[1],
    )
    assert result.stdout == (CD_FIXTURES / f"{sensor}.json").read_bytes()


def test_text_block(capsys):
    code = offline.main(["--text", "--", str(CD_FIXTURES / "a-mid360.bin"), str(CD_FIXTURES / "b-mid360.bin")])
    out = capsys.readouterr().out.splitlines()
    assert code == 0
    assert out[0] == "before: a-mid360.bin  points=20064  voxels=7592  dropped=0"
    assert out[2] == "voxel_size_m=0.050  tolerance=1  min_points=1  min_voxels=6  min_extent=2  pair_moves=true"
    assert out[3] == "appeared_voxels=255  disappeared_voxels=104  unknown_voxels=1286"
    assert out[5:] == ["regions=0"]
    assert offline.main(["--text", "--min-extent", "1", "--", str(CD_FIXTURES / "a-mid360.bin"),
                         str(CD_FIXTURES / "b-mid360.bin")]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[5] == "regions=1"
    assert out[6].startswith("region 0 appeared voxels=7 volume_m3=0.000875 centroid=-0.618,1.618,0.975")


@pytest.mark.parametrize("args", [
    [], ["a.bin"], ["--bogus", "a.bin", "b.bin"], ["--tolerance", "a.bin", "b.bin"],
    ["--voxel-size", "-1", "a.bin", "b.bin"], ["--origin", "1,2", "a.bin", "b.bin"], ["--min-points", "x", "a.bin", "b.bin"],
])
def test_usage_errors(args, capsys):
    assert offline.main(args) == 1
    captured = capsys.readouterr()
    assert captured.out == "" and captured.err.startswith("python -m room_memory.offline")


def test_input_and_config_errors(tmp_path, capsys):
    assert offline.main([str(tmp_path / "a.pcd"), str(tmp_path / "b.pcd")]) == 1
    assert "expected a .bin file" in capsys.readouterr().err
    assert offline.main(["--tolerance", "9", str(CD_FIXTURES / "a-mid360.bin"), str(CD_FIXTURES / "b-mid360.bin")]) == 1
    assert "tolerance" in capsys.readouterr().err


def test_opt_out_flags_select_the_swift_defaults(capsys):
    args = [str(CD_FIXTURES / "a-mid360.bin"), str(CD_FIXTURES / "b-mid360.bin")]
    assert offline.main(["--no-evidence"] + args) == 0
    assert '"evidence":false' in capsys.readouterr().out
    assert offline.main(["--no-evidence", "--text"] + args) == 0
    assert "unknown_voxels" not in capsys.readouterr().out
    config, emit_json, _, _ = offline.parse_args(args)
    assert not config.evidence_of_absence and not emit_json


def test_flags_map_to_config():
    config, emit_json, before, after = offline.parse_args([
        "--voxel-size", "0.1", "--max-range", "15", "--min-points", "2", "--tolerance", "2", "--min-voxels", "3",
        "--min-extent", "4", "--max-move", "0.5", "--no-pair", "--evidence", "--origin", "-1,0.5,2", "--ray-stride", "8", "--unobserved",
        "--json", "x.bin", "y.bin",
    ])
    assert (before, after, emit_json) == ("x.bin", "y.bin", True)
    assert config.voxel_size_f32 == np.float32(0.1) and config.max_range == 15
    assert (config.min_points, config.tolerance, config.min_voxels, config.min_extent, config.ray_stride) == (2, 2, 3, 4, 8)
    assert config.pair_moves is False and config.evidence_of_absence and config.report_unobserved
    assert config.origin == (-1.0, 0.5, 2.0)


def test_parse_f32_rounds_the_decimal_once():
    text = "1.000000178813934326171874"
    assert int(offline.parse_f32(text).view(np.uint32)) == 0x3F800001
    assert int(np.float32(float(text)).view(np.uint32)) == 0x3F800002
    assert int(offline.parse_f32("0.05").view(np.uint32)) == 0x3D4CCCCD
    assert np.isinf(offline.parse_f32("1e39"))
