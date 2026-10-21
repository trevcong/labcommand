import json
import unittest

import pytest

from spatio.walkday.gate import evaluate, main

L, I = "/cartA/scan", "/cartA/imu"


def report(**kw):
    hdr = {"backward_jumps": 0, "resets": [], "zero_stamps": 0, "gap_count": 0, "max_gap_s": 0.1}
    topics = [
        {"name": L, "count": 600, "header": dict(hdr, **kw.get("lidar_header", {}))},
        {"name": I, "count": 15000, "header": dict(hdr, **kw.get("imu_header", {}))},
    ]
    if kw.get("drop"):
        topics = [t for t in topics if t["name"] != kw["drop"]]
    cov = {"lidar_topic": L, "imu_topic": I, "scans": 600, "evaluable_scans": kw.get("evaluable", 600),
           "covered": kw.get("covered", 600), "unknown_scans": 0}
    return {"schema": "spatio.bagtiming/1", "topics": topics, "findings": kw.get("findings", []),
            "imu_coverage": [cov]}


class GateTest(unittest.TestCase):
    def test_clean_input_is_evaluable(self):
        out = evaluate(report(), L, I)
        self.assertTrue(out["evaluable"], out["blocking"])

    def test_missing_topic_blocks(self):
        out = evaluate(report(drop=I), L, I)
        self.assertFalse(out["evaluable"])
        self.assertTrue(any(b.startswith("B1") for b in out["blocking"]))

    def test_clock_reset_error_blocks(self):
        f = [{"severity": "error", "code": "CLOCK_RESET", "text": f"{I} header: 1 resets."}]
        self.assertFalse(evaluate(report(findings=f), L, I)["evaluable"])

    def test_backward_header_jump_blocks(self):
        self.assertFalse(evaluate(report(lidar_header={"backward_jumps": 2}), L, I)["evaluable"])

    def test_zero_stamp_blocks(self):
        self.assertFalse(evaluate(report(imu_header={"zero_stamps": 1}), L, I)["evaluable"])

    def test_partial_inspection_blocks(self):
        f = [{"severity": "warn", "code": "PARTIAL_INSPECTION", "text": "Requested 30 s slice"}]
        self.assertFalse(evaluate(report(findings=f), L, I)["evaluable"])

    def test_no_checkable_scan_bounds_blocks(self):
        out = evaluate(report(evaluable=0, covered=0), L, I)
        self.assertFalse(out["evaluable"])
        self.assertTrue(any(b.startswith("B5") for b in out["blocking"]))

    def test_gap_and_partial_coverage_only_warn(self):
        f = [{"severity": "warn", "code": "IMU_COVERAGE", "text": f"{L} vs {I}: 590/600 evaluable scans covered"}]
        out = evaluate(report(lidar_header={"gap_count": 1, "max_gap_s": 1.4}, covered=590, findings=f), L, I)
        self.assertTrue(out["evaluable"])
        self.assertEqual(len(out["warnings"]), 2)

    def test_gate_uses_given_topics_not_defaults(self):
        # The same report checked against Livox default names must block.
        self.assertFalse(evaluate(report(), "/livox/lidar", "/livox/imu")["evaluable"])


@pytest.mark.parametrize("token", ["NaN", "Infinity", "-Infinity", "1e999"])
@pytest.mark.parametrize("field", ["coverage", "header"])
@pytest.mark.parametrize("earlier", [False, True])
def test_nonfinite_timing_fails_clearly_without_replacing_gate(tmp_path, capsys, token, field, earlier):
    bt = report()
    if field == "coverage":
        bt["imu_coverage"][0]["covered"] = "NONFINITE"
        location = "bagtiming.imu_coverage[0].covered"
    else:
        bt["topics"][0]["header"]["max_gap_s"] = "NONFINITE"
        location = "bagtiming.topics[0].header.max_gap_s"
    source = tmp_path / "bagtiming.json"
    source.write_text(json.dumps(bt).replace('"NONFINITE"', token))
    target = tmp_path / "input_gate.json"
    previous = b'{"evaluable": false, "earlier": true}\n'
    if earlier:
        target.write_bytes(previous)
    # The public evaluator also rejects the invalid evidence, before it can be
    # copied to imu_coverage or silently disappear into a warning string.
    with pytest.raises(ValueError, match="non-finite numeric value"):
        evaluate(json.loads(source.read_text()), L, I)
    assert main([str(source), "--lidar", L, "--imu", I, "--out", str(target)]) == 3
    error = capsys.readouterr().err
    assert "gate: non-finite numeric value at " + location in error
    assert "Traceback" not in error
    assert target.read_bytes() == previous if earlier else not target.exists()
    assert not list(tmp_path.glob("*.tmp"))


if __name__ == "__main__":
    unittest.main()
