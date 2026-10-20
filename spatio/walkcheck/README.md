# Walk checker

Scores one Spatio session folder. It reads `trajectory.tum` and `map.pcd` and writes `walkcheck.json`, `walkcheck.md`, and `topdown.png` into that same folder.

The checker detects the first and last sustained stationary holds. The return error is the 3D gap between the mean positions of the first five seconds of the first hold and the last five seconds of the last hold. The end is not aligned onto the start. A result is `PASS` only when that gap is below 10 cm, both windows are parked, the timestamps are continuous, the map can be read, and the path between the holds actually moves. A parked cart does not pass. A missing or too-short hold is `NOT-EVALUABLE`. The existing endpoint checks also remain: samples more than 5 cm from their window mean, or mean speed above 10 cm/s, make the result `NOT-EVALUABLE`.

Walk protocol: start recording on the taped mark, hold for ten seconds, walk the route, return to the mark, and hold for ten seconds. Hold detection uses consecutive trajectory steps at most 2 cm/s and 3 deg/s, with at least eight seconds and three poses, spread at most 5 cm, and rotation from the first pose at most 10 degrees. These are **PROVISIONAL**, not calibrated to a historical walk. A slow-moving candidate whose total spread exceeds the limit is rejected as a whole. Noise or a slow transition can therefore prevent detection; there is no smoothing or search for a more favorable closure.

JSON and Markdown report hold times, durations, and flags. Motion before the first hold or after the last hold beyond 20 cm or 10 degrees makes the run `NOT-EVALUABLE`. An intermediate hold at a different pose produces `possible_repositioning`: this may also be an intentional route stop, so it is a warning, not a verdict. The first hold remains the start. This detector cannot determine which pose was physically on the tape. A single full-recording stationary hold retains the stationary-trajectory `FAIL`; a lone interior hold cannot provide both endpoints.

`trajectory.tum` and `map.pcd` are both required. A missing or unreadable one is `NOT-EVALUABLE`. A map with fewer than 25 finite XYZ points is also `NOT-EVALUABLE`, with reason `too_few_map_points`. This **PROVISIONAL** input-sufficiency minimum is not calibrated to recordings and does not establish room coverage or wall quality. Set it explicitly with `--min-map-points N` (positive integer), or `evaluate_session(..., min_map_points=N)`; the report records the setting. Empty maps cannot pass any allowed setting. A map meeting this minimum but having no wall score does not change the walk result.

The wall number is a gate (95th-percentile point-to-plane residual ≤ 5 cm, and no doubled sheet over 5 cm). A stacked pair the checker can see counts, including a gap wider than 40 cm. The search goes one cell past the wider normal fit. A pair farther away than that search can still look like two sharp walls. The report labels the gate "provisional (approved for now, revisit after real data)". The same phrase covers the 5 s window, the 1 s gap, and the must-move minimums. The 5 cm / 10 cm/s endpoint limits are a separate provisional choice and the report says they were not in that approved list. The wall gate does not change `PASS` / `FAIL` / `NOT-EVALUABLE`.

Horizontal and vertical splits assume Z is up. The 3D error does not.

## Run

From the repo root, with numpy and matplotlib available:

```bash
python3 -m venv spatio/walkcheck/.venv --system-site-packages
PYTHONPATH=. spatio/walkcheck/.venv/bin/python -m spatio.walkcheck /path/to/session
PYTHONPATH=. spatio/walkcheck/.venv/bin/python -m pytest spatio/walkcheck/tests -q
```

`--stationary-window` is seconds (default 5). `--max-gap` is the largest allowed timestamp step in seconds (default 1). `--max-endpoint-spread` is metres (default 0.05). `--max-endpoint-speed` is metres per second (default 0.10). A limit that is not a finite number, including NaN and infinity, is rejected. It does not turn a gate off.

All new detection limits are CLI-overridable:

| Flag | Default | Meaning |
|---|---:|---|
| `--min-hold` | 8 | Minimum hold duration, seconds (also at least `--stationary-window`) |
| `--max-hold-speed` | 0.02 | Maximum trajectory step speed, m/s (also capped by endpoint speed limit) |
| `--max-angular-speed` | 3 | Maximum trajectory/IMU angular speed, deg/s |
| `--significant-motion` | 0.20 | Translation flag threshold, m |
| `--significant-rotation` | 10 | Rotation flag threshold and hold rotation excursion limit, degrees |
| `--max-imu-gap` | 0.10 | Maximum IMU sample gap, seconds |
| `--max-accel-deviation` | 0.50 | Maximum acceleration magnitude deviation from gravity, m/s² |
| `--gravity` | 9.80665 | Expected gravity magnitude, m/s² |

Optional IMU: supply `--imu-file /path/to/imu.csv`, or place `imu.csv` in the session. Its exact header is `t,gx,gy,gz,ax,ay,az`; subsequent rows are numeric CSV, with seconds in the **same clock as the trajectory**, angular velocity in **rad/s**, and acceleration **including gravity in m/s²**. Export and unit conversion are the caller's responsibility; this scorer does not open bags or assume sensor topics, rates, extrinsics, or clock offsets. Every trajectory step in a hold must be bracketed by continuous, quiet IMU samples. All samples between the brackets are checked, including spikes between trajectory poses. Missing coverage invalidates those candidate steps. An unreadable, malformed, non-finite or non-monotonic supplied IMU file makes the result `NOT-EVALUABLE`; it is never silently ignored. Without an IMU file, the report explicitly says detection used only trajectory speed, rotation and spread.

Exit codes: 0 PASS, 1 FAIL, 2 NOT-EVALUABLE, 3 the session directory cannot be used.

`spatio/walkcheck/fixtures/sample_session/` is a synthetic 5 cm case. It is not a physical walk. Running the tool on it writes the three report files into that folder.

When `manifest.json` does not set `physical_accuracy_evaluated` to true, the report says "replay demonstrated; physical accuracy not evaluated".
