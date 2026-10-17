# LabCommand

macOS instrument console for the **dual-lidar rig** (Livox Mid-360 + Hesai
JT128) on the DGX Spark.

**v1.0 is a 100% rehaul.** The v0.x app drove a single Mid-360 by piping
`spark-capture.sh` over SSH and parsing its output. v1.0 binds to the
Spark's **command-center API** (`~/hesai_ws/command_center.py`, :8766 over
an SSH tunnel) — the same collector that serves the web panel and the
`lidar` CLI. One collector, one truth, three views.

What carried over unchanged: the design system (DESIGN-VISION.md), the
honest-state contract (missing keys print **unavailable**, a measured 0 Hz
is a real zero, "a running container is not a running stream"), the mission
log, focus mode, and the menu-bar extra.

What's new:

- **Two sensors, four honest states each**: Dark (unplugged is a normal,
  calm state — the rig reads **Parked**, not broken), Powered (answering,
  no points — standby or driver down), Live (with measured rates), and —
  (not probed).
- **JT128 Wake / Standby** on the sensor card — stops the motor and lasers
  for heat without unplugging (PTC 0x1C via the API).
- **Clocks card**: the ptp4l software master plus per-sensor clock offsets
  measured from IMU stamps. Knows that the Mid-360 slews ~10 ms/min after
  power-up while the JT128 steps instantly.
- **Watchdog aware**: plugging a sensor back in restarts the Spark stack
  automatically; the app just watches it happen and logs the event.
- **Dual recording**: one bag with both point clouds + both IMUs
  (`<ts>_DUAL_<code>_<tag>` on the Spark, ~2 GB/min when both stream).
- **5 s polling** — an HTTP GET over the tunnel, not an SSH round-trip, so
  the panel tracks reality at web-page speed.

## Build and run

From the repository root:

```bash
./build.sh --no-install
open ./build/LabCommand.app
```

Menu bar extra: **LAB**. Capture menu: ⌘↩ start/restart rig, ⌘⇧R record,
⌘. stop, ⌘⇧D diagnose. View: ⌘⇧F focus, ⌘R refresh.

## Map

| Dest | ⌘ | Purpose |
|---|---|---|
| **Rig** | 1 | Sensor duo, transport, recording controls, event log |
| **Settings** | 2 | Host, probe interval, capture paths |

## What the buttons do

| Control | What happens |
|---|---|
| Start / Restart rig | Opens both SSH tunnels (8765 view, 8766 API), then `start`/`restart` on the API → `lidar-dual` container |
| Start recording | API `record` → `ros2 bag` of all four topics inside `lidar-dual` |
| Stop | Recording? `stop-record` (SIGINT + chown). Else `down` (60 s grace) + close tunnels |
| Wake / Standby | API `wake`/`standby` → JT128 PTC 0x1C |
| Arm at lock | API `arm` → collector fires at sustained clock lock |
| Diagnose | Shows advice for the current rig state |
| Snapshot (3D view) | Opens one socket per sensor to foxglove_bridge :8765, takes one frame each (`/livox/lidar`, `/lidar_points`; 10 s limit per sensor), unsubscribes and closes. The frames stay on screen |
| Live (3D view) | Streams while on; stops itself after 120 s or when the window is hidden, minimized or covered. The last frames stay on screen |

The 3D view opens no socket on its own: at launch it is **OFF** (the sample
cloud may stand in, badged `OFF · SAMPLE — not live data`). For testing without a
click, `--snapshot-on-launch` and `--live-on-launch` run the same paths as the
buttons.

The Spark side lives in `~/hesai_ws/` (`command_center.py`, `dual_up.sh`,
`ptp_up.sh`, `watchdog.sh`); the rig write-up is
`~/Desktop/accountable-robotics-co/DUAL-RIG.md`.

## Tests

```bash
swift run LabCommandCoreTestsRunner
swift run change-detect -- before.pcd after.pcd
swift run change-detect -- --evidence --json before.bin after.bin
```

Pins the decoder's honest rules, the gates (record needs points, not
containers), the Parked band, dual session naming, and the v0.4 design
contract (type cap, spacing tokens, no placeholder chrome).
`change-detect` diffs two clouds of the same sensor (`.pcd` or CDR `.bin`).
`--json` writes the v2 parity record, and `--evidence` carves free space from
the after cloud.

## Hosted CI

[Hosted gate](.github/workflows/ci.yml) runs on pull requests and pushes to
`main`. Ubuntu 24.04 installs [the CI requirements](spark/requirements-ci.txt),
runs pytest from `spark/`, builds the non-ROS C++ targets and runs CTest with
the same Python interpreter. It also checks committed fixtures and compares
the complete hand-golden `.hex` sets byte-for-byte. macOS 15 with Xcode 16.4
runs `swift run LabCommandCoreTestsRunner` and
`swift build -c release --product LabCommand`.

The job summary reports pytest case counts and CTest entry counts separately,
including skips and reasons. The initial pytest pass skips ROS and tests
requiring unset C++ executable paths; CTest then runs the differential and
recording-tool tests with those paths configured. The full static recordings
are not in git: `static_longrun`, `static_longrun_b`, and `static_recording`
must explicitly skip for missing datasets. Missing decoder dependencies,
missing committed fixtures, unexpected CTest skips (including nested cases),
and missing/invalid reports fail the gate. Verbose CTest logs retain inner
test counts. These checks do not establish ROS, full-recording, or live-rig
correctness, and do not launch, install, or deploy the app.

Only pip downloads are cached, keyed by the requirements file. CMake and
Swift outputs are built fresh per job. CI helper regressions can be run with:

```bash
python3 -m unittest discover -s .github/scripts -p 'test_*.py'
python3 .github/scripts/ci_checks.py fixtures "$PWD"
```
