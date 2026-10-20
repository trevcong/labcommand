# LabCommand

LabCommand is room memory for the Livox Mid-360 and Hesai JT128 on the DGX Spark, and the Mac app that mirrors it. The Spark node integrates the clouds, keeps the reference, and publishes changes. The Mac draws that state. The rig path is the command-center API on port 8766 over an SSH tunnel (`RigService`: `GET /status.json`, `POST /action`). Snapshot and Live dial Foxglove on port 8765. The room path is a WebSocket to the room-memory bridge on port 8767.

A normal launch (`LABCOMMAND_LIVE_ROOM` unset) opens the Room tab and still probes the rig. The room socket opens only for an admitted URL. The built-in address is `ws://spark-1f3e.tail8dfdc7.ts.net:8767` (`RoomMemoryEndpoint.defaultURL`, host `CaptureRecipe.tailscaleDNS`). The saved cleartext switch starts off, so that `ws://` default stays closed until Allow cleartext WebSocket is on, or the address is `wss://`. `LABCOMMAND_LIVE_ROOM=live` is the gate launch: it uses the same admission, and it skips probes and the lidar client. That flag does not select the tab. Any other value is fixture mode (`LiveRoomLaunch`). The named values are `map`, `status`, `stale`, `lost`, `restart`, `legacy`, and `unavailable`. An unknown value loads the `unavailable` fixture. Fixture mode never opens a socket. It skips probes and leaves the lidar client dark. The menu-bar extra is **LAB**. The tooltip, the accessibility label, and the menu say LabCommand.

## Tabs

Order and shortcuts follow `Dest.navCases`. The window title is `LabCommand - <tab>`.

**Room** (⌘1) is the default. It draws the voxel map, the status line, and the change list from the mirror. Event clocks are local `yyyy-MM-dd HH:mm:ss`. A stamp of `{sec: 0, nanosec: 0}` stays `time —`. A broken publisher generation shows `Spark node restarted · events from the earlier run may be missing`.

**Settings** (⌘2). The Room memory field edits the saved endpoint (`roomMemoryEndpoint`). Return saves a URL `RoomMemoryEndpoint.parse` accepts. A running socket retargets to it. A change from a saved `wss://` to an allowed `ws://` stays unsaved until Use cleartext. Under the field, the page prints `RoomMemoryEndpoint.defaultURL`. A valid `LABCOMMAND_ROOM_URL` wins at process start and is not written, except a `ws://` value leaves a saved `wss://` address in place. `LABCOMMAND_ROOM_URL` does not grant cleartext. Allow cleartext WebSocket (`roomMemoryAllowCleartextWebSocket`) defaults to off. `LABCOMMAND_ALLOW_CLEARTEXT=1` permits cleartext for this launch and does not change the switch. The Probe group is auto-refresh and the interval (15, 30, 45, 90, or 180 seconds). That interval defaults to 45 seconds (`probeIntervalSeconds`), the slow connectivity probe. Rig diagnostics stays closed until opened. Its compiled-in rows are SSH host (`spark`), Probes (`Local Process`), Foxglove (`ws://localhost:8765`), sensor IP (`192.168.1.135`), NIC (`enP7s7`), bag root (`/home/tcong/datasets/home_lab`), Mac sessions (`~/Desktop/lidar-sessions`), and the Keys group (⌘↩, ⌘⇧R, ⌘., ⌘R, ⌘⇧F, ⌘⇧D).

**Rig** (⌘3) is the lidar recorder. Sensor cards read Dark, Powered, Live, or — when a sensor has not been probed. A missing point rate and a missing clock delta are left off the card. A measured 0 Hz is a real zero (`0.0 Hz`), and a running container is not a running stream. An unpowered rig's headline on the tab is `Sensors are parked — plug in power to begin`. The status-bar menu is where the health band prints `Parked`. The Clocks card shows the measured clock state. The three desk buttons start or restart the rig, start recording, and stop. Arm at lock and Diagnose sit beside them.

The 3D view stays off until Snapshot or Live. At launch the sample cloud may stand in, badged `OFF · SAMPLE — not live data`. Snapshot takes one frame per sensor (`/livox/lidar`, `/lidar_points`) with a 10 second limit, then closes. Live stops after 120 seconds, or when the window is hidden, minimized, or covered. The last frames stay on screen. `--snapshot-on-launch` and `--live-on-launch` run those paths without a click.

The mission log shows the last five lines, stamped with `MissionLog.clock` (`yyyy-MM-dd HH:mm:ss` in the live system zone). That formatter rebuilds when the system time zone changes. That log is the recorder's. The room's change list is the other one.

The record call sends `<code>_<tag>` (`RigDesk.recordName`). `sessionFolderPreview` is called from `testDeskDualSessionNaming` in the test runner. No view calls it.

Commands are `buildMainMenu` in `LabCommandApp.swift`. View: ⌘1 Room, ⌘2 Settings, ⌘3 Rig (`Dest.navCases`), ⌘R Refresh (enabled on Rig or in focus mode, and disabled when the launch isolates the network), ⌘⇧F Focus Mode. Capture: ⌘↩ Start Rig, ⌘⇧R Start Recording, ⌘. Stop, ⌘⇧D Diagnose (switches to Rig).

The Spark chip follows the room socket while that socket is connected, and the command-center probe otherwise. The lidar chips take their state and rate from a fresh room status, and from the probe when that status is stale.

## Build

From the repository root, on macOS 14 or newer:

```bash
./build.sh --no-install
open ./build/LabCommand.app
```

`./build.sh` with no flag also replaces `~/Applications/LabCommand.app`. `--no-install` leaves that copy alone. The script builds the release binary, ad-hoc codesigns the bundle, and copies `fixtures/api`, the room hand goldens, and `fixtures/pointclouds/mid360-frame.pcd`.

The hosted Mac job is macOS 15 with Xcode 16.4. It runs `swift build -c release --product LabCommand`. It does not run `./build.sh`, and it does not install the app.

## Architecture

Three pieces. The wire rules live in the protocol doc.

**Spark room-memory node.** Production code is `spark/room_memory_cpp` (`node/room_memory_node.cpp`, `launch/room_memory_cpp.launch.py`). `spark/room_memory` is the Python reference oracle. Differential tests compare the two engines byte for byte. `RigPose` floats stay within [PLAN-RIGGUARD.md](PLAN-RIGGUARD.md) §7. The never-realigned file compare sets `rig_state` aside from that byte compare.

The node subscribes to `/livox/lidar` and `/lidar_points` with sensor-data QoS and closes a window every `window_s` (default 1 second). It publishes `/room_memory/status` and `/room_memory/activity` at 1 Hz (reliable, transient_local). Each transition goes out on volatile `/room_memory/events`. A window that has a map update publishes volatile `/room_memory/map_bricks`. It answers `RequestSync` on the sync topics and then sends `/room_memory/sync/end`.

The C++ launch file also starts foxglove_bridge on **8767**. The topic whitelist is `^/room_memory/.*`. The service whitelist is `/room_memory/sync/request` and `/room_memory/objects/name`. The node advertises the request service. It does not advertise NameObject until the wiring in [docs/known-gaps.md](docs/known-gaps.md). Client publishing stays off unless `foxglove_client_publish:=true`. Raw clouds do not leave through 8767. The Python launch file does not start this bridge. Neither node publishes `/room_memory/epoch`.

`rig_guard` defaults to false. With `rig_guard:=true` the node publishes `RigPose` on `/room_memory/rig` (reliable, transient_local, on change and every 10 seconds). `state_dir` defaults to `~/room-memory`. An empty `state_dir` value, or a directory that does not exist, stops the node; it never creates one. References are saved in `<state_dir>/<room_id>/`. With the guard, registration clouds and `rig_state` are saved there too. The event log is not.

**Protocol.** [docs/ROOM-MEMORY-PROTOCOL.md](docs/ROOM-MEMORY-PROTOCOL.md) is classic CDR. The Mac mirror follows §5 (publisher generation, `RequestSync`, `SyncEnd`). D50 (§3.14) records that the retired v0 Mac capture path is not a message and changes no wire byte. D51 (§3.15, §3.16) is `ObjectRegistry` and `NameObject`. The `.msg` files are in `ros/labcommand_msgs/`. `OperatorAction` is a `.msg` only: no codec and no node handler (§3.6, §9).

**Mac mirror.** `LiveRoomClient.init(url:)` requires a URL. The client has no `defaultURL`. The built-in address is `RoomMemoryEndpoint.defaultURL`: `ws://` plus `CaptureRecipe.tailscaleDNS` (`spark-1f3e.tail8dfdc7.ts.net`) and port 8767. `CaptureRecipe.tailscaleIP` (`100.118.187.99`) is not that default and not a fallback. `RoomMemoryEndpoint.resolve` uses a valid `LABCOMMAND_ROOM_URL` when that value would not replace a saved `wss://` with `ws://`, then a valid saved string, then the built-in URL. `AppState.start` dials only an admitted URL. `wss://` is always admitted. `ws://` needs a host `allowsCleartextHost` accepts (private, loopback, link-local, CGNAT `100.64.0.0/10`, or a `*.ts.net` name; `localhost` is not one) and cleartext permission from the saved switch or `LABCOMMAND_ALLOW_CLEARTEXT=1`. With the switch off, a fresh launch does not open the default socket. The Room screen then shows `Cleartext WebSocket is off.` A redirect from a secure URL to cleartext is refused. An IPv4 literal with a leading zero or a trailing dot is rejected before it is saved. The client subscribes to the `/room_memory/*` topics it knows — `map_bricks`, `events`, `status`, `activity`, `epoch`, and the sync topics including `sync/end` — and reduces them in `RoomMemoryMirror`. The Room page binds to that mirror ([Sources/LabCommandCore/LiveRoom/API.md](Sources/LabCommandCore/LiveRoom/API.md)). Swift can decode `RigPose`. The app does not subscribe to `/room_memory/rig`, and the Room page does not show it.

The rig path is separate from the room socket. A normal launch probes the rig (`AppState.start` → `beginRig` → `refresh`). The probe opens the command-center SSH tunnel on **8766** only (`RigService.fetchStatus` → `ensureAPITunnel`). Starting or restarting the rig calls `ensureTunnels`, which opens 8766 and the Foxglove forward on 8765 (`ssh -L` to `127.0.0.1:8765` on the Spark). The Rig tab does not open either tunnel. While auto-refresh is on, two timers run. The rig poll is every 5 seconds over 8766 (`rigPollSeconds`, an API GET, not a Settings control). The connectivity probe uses the Settings interval, which defaults to 45 seconds (`probeIntervalSeconds`). The command-center process is not in this repository.

Snapshot and Live connect straight to the Spark. `FoxgloveLiveClient` opens `ws://<host>:8765` (`FoxgloveLiveClient.run`). The hosts are the IPv4 literals from `FoxgloveProtocol.streamHosts` (`SparkSSH.hopList` with the cached LAN hop): the tailnet address `100.118.187.99`, then a cached LAN address when that cache is an IPv4 literal. SSH names and the MagicDNS name are dropped. That list has no loopback address. Rig diagnostics still shows `ws://localhost:8765` (`CaptureRecipe.viewConnectURL`). That string is the local end of the 8765 SSH forward.

## Known limits

**Each lidar stays in its own frame.** The JT128 to Mid-360 extrinsic is not measured. Each sensor keeps a reference in that sensor's frame. `map_bricks` publishes one of them: `map_sensor`, the JT128 when it is configured. Events from the other sensor stay in that sensor's frame. The nominal Mid-360 mount in `memory.py` (`MOUNT`) is a rotation prior for the rig guard, never an extrinsic. `RigTransforms.livoxToWorld` is the Rig tab's display transform for the 3D overlay. The Room map does not use it.

**Events are lost when the node restarts, until [pull request #32](https://github.com/trevcong/labcommand/pull/32).** The event log keeps the last 100 transitions in memory (`EVENT_LOG_DEPTH`, `kEventLogDepth`). `save` and `load` persist the reference bricks and, with the guard, the registration clouds and `rig_state`. They do not persist the event log. This tree has no `eventlog` source. A restart allocates a new publisher generation, and `RequestSync` on the new process cannot replay the previous run. The Room page says events from the earlier run may be missing. Pull request #32, "Durable event history on the Spark across restarts", is an open draft. Its durable log is not on this tree.

**The tracker does not publish CONFIRMED.** The transitions this tree appends are `CANDIDATE` and `EXPIRED`. The Mac can render the other states when a message carries them.

**Physical acceptance is a candidate score.** [`spark/room_memory/acceptance/`](spark/room_memory/acceptance/README.md) is an operator script, a read-only recorder, and an offline scorer. The command does not ssh, publish, or restart the node, and its tests do not dial the Spark. A passing run sets `g5_acceptance` to `full_candidate`. That is a candidate score. `confirmed_product_bar` is a separate field. This node emits `CANDIDATE`. A log with no `CONFIRMED` message leaves that field `not_evaluated`.

**A request path exists only when the bridge offers one.** This repo's C++ launch file advertises `/room_memory/sync/request`. When the running image does not, the client counts the request unsent and the event column says it cannot ask the Spark (`can't ask the Spark for them`). Protocol §9 records that the deployed image was still the pre-D39 image when that section was written. This page describes the code. It does not claim the lab image was checked.

**CI is not a live-rig proof.** The Ubuntu job skips ROS tests (`test_node.py` needs `rclpy` and `labcommand_msgs`) and `rosbags` at collection. There is no pytest config in `spark/`. `spark/room_memory/setup.cfg` sets `testpaths = tests`, and that applies only when pytest is started in `spark/room_memory`. A run from `spark/` also collects `room_memory/acceptance/tests` and the C++ `tests/differential` and `tests/recording` trees. The acceptance tests do not dial the Spark. `differential/test_differential.py` skips unless both `ROOM_MEMORY_OFFLINE` and `ROOM_MEMORY_REPLAY` are set. `recording/test_static_recording.py` skips while `ROOM_MEMORY_BAG` is unset, and at collection when `mcap` cannot be imported. Most of `recording/test_tools.py` runs in that pass; only `test_statuses_tick_every_second_independently_of_the_window` waits on `ROOM_MEMORY_BAG`. CTest runs the differential and recording-tool tests with those paths set. The workflow points `ROOM_MEMORY_RECORDING` and `ROOM_MEMORY_RECORDING_B` at paths that do not exist, and it checks that they are absent, so `static_longrun`, `static_longrun_b`, and `static_recording` skip. Left unset, those three use `~/Developer/lc-tasks/recordings/static5m` and `static5m-b`. When those recordings are present the tests run, under CTest limits of 300 s, 300 s, and 3600 s. Unexpected CTest skips, including nested ones, fail the gate. The Mac job runs the Swift test runner and a release build of the app. It does not launch or install it.

## Tests

Run the complete local non-ROS suite on macOS with one command. Set up a Python
3.12 venv first; `requirements-local.txt` includes the hosted CI pins plus the
Spatio MCAP/plotting dependencies and `rosbags`:

```bash
python3.12 -m venv spark/.venv-local < /dev/null
spark/.venv-local/bin/python -m pip install -r requirements-local.txt < /dev/null
scripts/test-all.sh < /dev/null
```

The runner executes CI helper unittests, committed-fixture checks, Spatio pytest
(including `.github/scripts`), Spark pytest, C++ configure/build/CTest, the Swift
core runner, the `change-detect` build, shared-golden comparison, and Spatio
fixture regeneration in check-only mode, in that order. The two pytest runs stay
separate because the trees contain duplicate test module names. Each test stage
prints its native test counts (CTest is verbose, including nested pytest counts).
The final summary names every stage as `PASS`, `FAIL`, or `SKIPPED`; the first
failure stops execution and returns nonzero, with later stages marked not reached.
Build stages report command success, not test counts.

The default interpreter is `spark/.venv-local/bin/python`. Override `PYTHON` for
another environment, or `SPATIO_PYTHON` for just Spatio pytest and its fixture
check. Spatio continues to support system Python 3.9; moving it to 3.12 is not
part of this runner. With Spatio's dependencies already installed for system
Python, verify that compatibility separately:

```bash
python3 -m pytest -q spatio .github/scripts < /dev/null
# Or exercise system Python for Spatio within the combined gate:
SPATIO_PYTHON=python3 scripts/test-all.sh < /dev/null
```

The full runner requires the local C++ prerequisites described below and Swift.
`ROOM_MEMORY_OPENMP=OFF scripts/test-all.sh` configures without OpenMP if libomp
is unavailable. `SKIP_CPP=1 scripts/test-all.sh` visibly reports all three C++
stages as `SKIPPED`, and is only a partial gate. The runner uses fresh absent
recording paths for CTest, so the full-recording tests skip; ROS tests and the
JT128 characterization test also skip without their external prerequisites.
It does not use Docker, connect to hardware, or launch/install the app.
An app release build, the optional Swift differential extension, and stricter
`rosbags` skip enforcement are outside this command's scope.

Mac, from the repo root:

```bash
swift run LabCommandCoreTestsRunner
swift run change-detect -- before.pcd after.pcd
swift run change-detect --evidence --json before.bin after.bin
```

`change-detect` diffs two clouds of the same sensor (`.pcd` or CDR `.bin`). It does not align them. `--json` prints the JSON record and exits. `--evidence` carves free space from the after cloud. Those flags belong before the two paths: the tool treats every argument after a `--` as a path, and `swift run` passes that `--` through.

Spark. `spark/.venv-local` is gitignored. When that interpreter exists, CMake uses it for the differential tests. There is no pytest config in `spark/` (`pytest.ini`, `pyproject.toml`, or `setup.cfg`). `spark/room_memory/setup.cfg` sets `testpaths = tests`, which applies only when pytest is started in that directory, so that invocation does not collect `acceptance/tests`. Collection from `spark/` includes `room_memory/tests`, `room_memory/acceptance/tests` (`test_runner.py`, `test_score.py`, `test_session.py`, `test_transport.py`), `room_memory_bench/tests`, and the C++ `tests/differential` and `tests/recording` trees. The acceptance tests do not dial the Spark. `test_node.py` skips at collection without `rclpy` and `labcommand_msgs`. `test_rosbags.py` skips at collection without `rosbags`. `test_differential.py` skips unless both `ROOM_MEMORY_OFFLINE` and `ROOM_MEMORY_REPLAY` are set. `test_static_recording.py` skips while `ROOM_MEMORY_BAG` is unset, and at collection when `mcap` cannot be imported. Most of `test_tools.py` runs without those binaries; only `test_statuses_tick_every_second_independently_of_the_window` waits on `ROOM_MEMORY_BAG`. The first Ubuntu pytest step uses this same `spark/` collection and adds `--durations=15`. Create the venv with Python 3.12, the version CI installs. `spark/requirements-ci.txt` pins `numpy==2.0.2`.

```bash
python3.12 -m venv spark/.venv-local
spark/.venv-local/bin/python -m pip install -r spark/requirements-ci.txt
(cd spark && .venv-local/bin/python -m pytest -q -rs)
```

CI helpers, from the repo root:

```bash
python3 -m unittest discover -s .github/scripts -p 'test_*.py'
python3 .github/scripts/ci_checks.py fixtures "$PWD"
```

Local C++ tests need CMake 3.20 or newer, a C++20 compiler, and zlib. libzstd is optional locally (the recording reader; Homebrew `zstd` on macOS); CI requires it. OpenMP defaults on (`ROOM_MEMORY_OPENMP`). Apple Clang has no OpenMP runtime, so install Homebrew `libomp` before configuring. The block below is the local run. `build/` is a local artifact. Do not commit it.

```bash
cmake -S spark/room_memory_cpp -B build/room_memory_cpp \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DROOM_MEMORY_BUILD_NODE=OFF \
  -DROOM_MEMORY_PYTHON="$PWD/spark/.venv-local/bin/python"
cmake --build build/room_memory_cpp
# Point the long recording tests at paths that are not there. Unset, they read
# ~/Developer/lc-tasks/recordings/static5m and static5m-b, and they run when
# those recordings exist (CTest limits 300 s, 300 s, and 3600 s).
ROOM_MEMORY_RECORDING="$PWD/build/absent-recordings/static5m" \
ROOM_MEMORY_RECORDING_B="$PWD/build/absent-recordings/static5m-b" \
  ctest --test-dir build/room_memory_cpp --output-on-failure
```

The hosted job adds `-DROOM_MEMORY_NATIVE=OFF`, checks that zstd was found before the build, writes JUnit reports, and rejects unexpected skips, including nested ones. It sets the same two recording variables to absent paths under the runner temp directory and checks that those paths are missing, so `static_longrun`, `static_longrun_b`, and `static_recording` skip.

## Hosted CI

The [hosted gate](.github/workflows/ci.yml) runs on pull requests and on pushes to `main`. A concurrency group per ref cancels an in-progress run on pull requests only.

Ubuntu 24.04 installs [the CI requirements](spark/requirements-ci.txt) for Python 3.12, runs `python -m pytest -q -rs --durations=15` from `spark/` (the collection in Tests, including `room_memory/acceptance/tests`), builds the non-ROS C++ targets, and runs CTest with that same interpreter. `--durations=15` prints the fifteen slowest tests. It also checks committed fixtures and compares the hand-golden `.hex` sets byte for byte. macOS 15 with Xcode 16.4 runs `swift run LabCommandCoreTestsRunner` and `swift build -c release --product LabCommand`.

The job summary counts pytest cases and CTest entries separately, including skips and reasons. The first pytest pass skips ROS and tests that need the C++ executables. CTest then runs the differential and recording-tool tests with those paths set. The workflow points `ROOM_MEMORY_RECORDING` and `ROOM_MEMORY_RECORDING_B` at absent paths, so `static_longrun`, `static_longrun_b`, and `static_recording` skip. Missing decoder dependencies, missing committed fixtures, unexpected CTest skips (including nested cases), and a missing or invalid report fail the gate. Verbose CTest logs keep the inner test counts. These checks do not establish ROS, full-recording, or live-rig correctness, and they do not launch, install, or deploy the app.

Only pip downloads are cached, keyed by the requirements file. CMake and Swift outputs are built fresh each job.

## Docs

| Path | What it is |
|---|---|
| [docs/ROOM-MEMORY-PROTOCOL.md](docs/ROOM-MEMORY-PROTOCOL.md) | Wire protocol, including D30, D34–D44, D50, and D51. §3.14: the v0 Mac capture path is not a message. §3.15–§3.16: `ObjectRegistry` and `NameObject`. §9: the Mac does not subscribe to `/room_memory/rig` |
| [Sources/LabCommandCore/LiveRoom/API.md](Sources/LabCommandCore/LiveRoom/API.md) | Mac mirror: client, mirror, screen strings |
| [docs/CHANGE-DETECTION.md](docs/CHANGE-DETECTION.md) | Occupancy diff shared by Swift, Python, and C++ |
| [spark/room_memory_cpp/PLAN.md](spark/room_memory_cpp/PLAN.md) | Production C++ node and windows. `Epoch` is not published. Its `rig_guard` note points at [PLAN-RIGGUARD.md](PLAN-RIGGUARD.md) |
| [spark/room_memory/PLAN.md](spark/room_memory/PLAN.md) | Python oracle |
| [spark/room_memory/acceptance/README.md](spark/room_memory/acceptance/README.md) | Physical acceptance: operator script, read-only recorder, offline candidate score |
| [PLAN-RIGGUARD.md](PLAN-RIGGUARD.md) | Rig-bump guard on the Spark. v1 is phases 1–2. A Mac `RigPose` display is later |
| [PLAN-PUBGEN.md](PLAN-PUBGEN.md) | Publisher generations and the mirror's restart rules |
| [design-refs/DESIGN-VISION.md](design-refs/DESIGN-VISION.md) | Visual contract the design tests pin |
| [.github/workflows/ci.yml](.github/workflows/ci.yml) | Hosted gate |
