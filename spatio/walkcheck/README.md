# Walk checker

Scores one Spatio session folder. It reads `trajectory.tum` and `map.pcd` and writes `walkcheck.json`, `walkcheck.md`, and `topdown.png` into that same folder.

The return error is the 3D gap between the mean pose of a fixed time window at the start and the mean pose of a fixed time window at the end. The end is not aligned onto the start. A result is `PASS` only when that gap is below 10 cm, the timestamps are continuous, and the path actually moves. A parked cart does not pass.

The wall number is a proposed gate (95th-percentile point-to-plane residual ≤ 5 cm, and no doubled sheet over 5 cm). The report labels it "proposed, not yet approved by Trevor". It does not change `PASS` / `FAIL` / `NOT-EVALUABLE`.

Horizontal and vertical splits assume Z is up. The 3D error does not.

## Run

From the repo root, with numpy and matplotlib available:

```bash
python3 -m venv spatio/walkcheck/.venv --system-site-packages
PYTHONPATH=. spatio/walkcheck/.venv/bin/python -m spatio.walkcheck /path/to/session
PYTHONPATH=. spatio/walkcheck/.venv/bin/python -m pytest spatio/walkcheck/tests -q
```

`--stationary-window` is seconds (default 5). `--max-gap` is the largest allowed timestamp step in seconds (default 1).

Exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 the session directory cannot be used.

`spatio/walkcheck/fixtures/sample_session/` is a synthetic 5 cm case. It is not a physical walk. Running the tool on it writes the three report files into that folder.

When `manifest.json` does not set `physical_accuracy_evaluated` to true, the report says "replay demonstrated; physical accuracy not evaluated".
