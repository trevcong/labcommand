"""Exercise the runner's actual shell function and final status handling, without ROS."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import unittest


RUNNER = (Path(__file__).resolve().parents[1] / "run_in_container.sh").read_text()
STOP_PROC = re.search(r"^stop_proc\(\).*?^\}", RUNNER, re.M | re.S).group()


class ShutdownTests(unittest.TestCase):
    def run_shell(self, script):
        return subprocess.run(
            ["bash", "-c", "set -uo pipefail\nsay() { echo \"$*\"; }\n" +
             STOP_PROC + "\n" + script],
            text=True, capture_output=True, timeout=12)

    def test_already_exited_child_preserves_failure(self):
        result = self.run_shell(
            '(exit 42) & pid=$!\nwait "$pid" || :\nstop_proc "$pid" child 2')
        self.assertEqual(result.returncode, 42, result.stdout + result.stderr)

    def live_child(self, status, ignore=False):
        with tempfile.TemporaryDirectory() as directory:
            ready = str(Path(directory) / "ready")
            child = (
                "import signal, sys\n"
                f"signal.signal(signal.SIGINT, "
                f"{'signal.SIG_IGN' if ignore else 'lambda *_: sys.exit(' + str(status) + ')'})\n"
                f"open({ready!r}, 'w').close()\n"
                "while True: signal.pause()\n"
            )
            result = self.run_shell(
                "set -m\n" +
                shlex.quote(sys.executable) + " -c " + shlex.quote(child) +
                ' & pid=$!\ntrap \'kill -KILL "$pid" 2>/dev/null || :\' EXIT\n' +
                f"while [[ ! -f {shlex.quote(ready)} ]]; do sleep 0.01; done\n" +
                'stop_proc "$pid" child 1\nrc=$?\ntrap - EXIT\nexit "$rc"\n')
        return result

    def test_live_child_shutdown_failure_is_not_logging_success(self):
        result = self.live_child(42)
        self.assertEqual(result.returncode, 42, result.stdout + result.stderr)
        self.assertIn("exit 42", result.stdout)

    def test_clean_shutdown_succeeds(self):
        result = self.live_child(0)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_forced_shutdown_fails(self):
        result = self.live_child(0, ignore=True)
        self.assertEqual(result.returncode, 137, result.stdout + result.stderr)
        self.assertIn("SIGKILL", result.stdout)

    def test_final_runner_status_checks_shutdown_even_with_readable_outputs(self):
        # Execute the real shutdown/export tail. Stand-ins make exports succeed;
        # no ROS, containers, source trees or foreign processes are touched.
        tail = RUNNER[RUNNER.index('stop_proc "$REC" recorder 60'):]
        with tempfile.TemporaryDirectory() as directory:
            cases = [("success", {}, 0), ("FAST-LIO exited early", {"FL_RUNNING": "1"}, 1)]
            cases += [(key, {key: "42"}, 42) for key in (
                "REC_STATUS", "WATCH_STATUS", "FL_STATUS", "PLAY_RC",
                "TUM_STATUS", "MAP_STATUS")]
            for label, overrides, expected in cases:
                with self.subTest(case=label):
                    env = dict(os.environ, OUT=directory, LOGS=directory, T=directory)
                    env.update({key: "0" for key in (
                        "REC_STATUS", "WATCH_STATUS", "FL_STATUS",
                        "PLAY_RC", "TUM_STATUS", "MAP_STATUS", "FL_RUNNING")})
                    env.update(overrides)
                    for name in ("trajectory.tum", "map.pcd", "map_display.pcd"):
                        (Path(directory) / name).write_text("readable partial output\n")
                    script = """
set -uo pipefail
REC=11 WATCH=12 FL=13
say() { echo "$*"; }
stop_proc() {
  case $2 in
    recorder) return "$REC_STATUS";;
    odom_watch) return "$WATCH_STATUS";;
    fastlio) return "$FL_STATUS";;
  esac
}
kill() { return "$FL_RUNNING"; }
cp() { return 0; }
python3() {
  case $1 in
    */bag_to_tum.py) return "$TUM_STATUS";;
    */bag_to_map.py) return "$MAP_STATUS";;
    *) return 99;;
  esac
}
"""
                    result = subprocess.run(
                        ["bash", "-c", script + tail], env=env,
                        capture_output=True, text=True, timeout=5)
                    self.assertEqual(result.returncode, expected,
                                     result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
