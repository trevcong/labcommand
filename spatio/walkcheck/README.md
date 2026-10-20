# Walk checker

Scores one Spatio session folder. It reads `trajectory.tum` and `map.pcd` and writes `walkcheck.json`, `walkcheck.md`, and `topdown.png` into that same folder.

The return error is the 3D gap between the mean pose of a fixed time window at the start and the mean pose of a fixed time window at the end. The end is not aligned onto the start. A result is `PASS` only when that gap is below 10 cm, both windows are parked, the timestamps are continuous, the map can be read, and the path actually moves. A parked cart does not pass. A window that is still moving does not pass either: if samples sit more than 5 cm from the window mean, or the window's own path is faster than 10 cm/s, the result is `NOT-EVALUABLE`.

`trajectory.tum` and `map.pcd` are both required. A missing or unreadable one is `NOT-EVALUABLE`. A map that opens but has no wall score does not change the walk result.

The wall number is a gate (95th-percentile point-to-plane residual ≤ 5 cm, and no doubled sheet over 5 cm). The report labels it "provisional (approved for now, revisit after real data)". The same phrase covers the 5 s window, the 1 s gap, and the must-move minimums. The 5 cm / 10 cm/s endpoint limits are a separate provisional choice and the report says they were not in that approved list. The wall gate does not change `PASS` / `FAIL` / `NOT-EVALUABLE`.

Horizontal and vertical splits assume Z is up. The 3D error does not.

## Run

From the repo root, with numpy and matplotlib available:

```bash
python3 -m venv spatio/walkcheck/.venv --system-site-packages
PYTHONPATH=. spatio/walkcheck/.venv/bin/python -m spatio.walkcheck /path/to/session
PYTHONPATH=. spatio/walkcheck/.venv/bin/python -m pytest spatio/walkcheck/tests -q
```

`--stationary-window` is seconds (default 5). `--max-gap` is the largest allowed timestamp step in seconds (default 1). `--max-endpoint-spread` is metres (default 0.05). `--max-endpoint-speed` is metres per second (default 0.10). A limit that is not a finite number, including NaN and infinity, is rejected. It does not turn a gate off.

Exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 the session directory cannot be used.

`spatio/walkcheck/fixtures/sample_session/` is a synthetic 5 cm case. It is not a physical walk. Running the tool on it writes the three report files into that folder.

When `manifest.json` does not set `physical_accuracy_evaluated` to true, the report says "replay demonstrated; physical accuracy not evaluated".
