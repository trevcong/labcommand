# LabCommand

LabCommand is room memory for the Livox Mid-360 and Hesai JT128 on the DGX Spark, and the Mac app that mirrors it. The Spark node integrates the clouds, keeps the reference, and publishes changes. The Mac draws that state.

A normal launch (`LABCOMMAND_LIVE_ROOM` unset) opens the Room tab, connects the room socket, and still probes the rig. `LABCOMMAND_LIVE_ROOM=live` is the gate launch: it skips probes and the lidar client. That flag does not select the tab. The menu-bar item is **LabCommand**.

## Tabs

Order and shortcuts follow `Dest.navCases`. The window title is `LabCommand - <tab>`.

**Room** (⌘1) is the default. It draws the voxel map, the status line, and the change list from the mirror. Event clocks are local `yyyy-MM-dd HH:mm:ss`. A stamp of `{sec: 0, nanosec: 0}` stays `time —`. A broken publisher generation shows `Spark node restarted · events from the earlier run may be missing`.

**Settings** (⌘2). Auto-refresh and the probe interval (15, 30, 45, 90, or 180 seconds; the default is 45). SSH host (`spark`), the Foxglove URL (`ws://localhost:8765`), sensor IP (`192.168.1.135`), NIC (`enP7s7`), bag root (`/home/tcong/datasets/home_lab`), and the Mac session path (`~/Desktop/lidar-sessions`) are compiled-in labels. The Keys group lists the Capture and View shortcuts. A Probes row reads Local Process.

**Rig** (⌘3) is the lidar recorder. Sensor cards read Dark, Powered, Live, or — when a sensor has not been probed. An unpowered rig reads Parked. A missing key prints `unavailable`, a measured 0 Hz is a real zero, and a running container is not a running stream. The Clocks card shows the measured clock state. The three desk buttons start or restart the rig, start recording, and stop. Arm at lock and Diagnose sit beside them.

The 3D view stays off until Snapshot or Live. At launch the sample cloud may stand in, badged `OFF · SAMPLE — not live data`. Snapshot takes one frame per sensor (`/livox/lidar`, `/lidar_points`) with a 10 second limit, then closes. Live stops after 120 seconds, or when the window is hidden, minimized, or covered. The last frames stay on screen. `--snapshot-on-launch` and `--live-on-launch` run those paths without a click.

The mission log shows the last five lines, stamped local `yyyy-MM-dd HH:mm:ss`. That log is the recorder's. The room's change list is the other one.

A recording preview is `<UTC yyyyMMdd'T'HHmmss'Z'>_DUAL_<code>_<tag>`. The record call sends `<code>_<tag>`. `RigDesk` treats the timestamp and `DUAL` as added by the :8766 API.

Menus, still wired. Capture: ⌘↩ start or restart, ⌘⇧R record, ⌘. stop, ⌘⇧D diagnose (switches to Rig). View: ⌘R refresh, enabled on Rig or in focus mode, and ⌘⇧F focus.

The Spark chip follows the room socket while that socket is connected, and the command-center probe otherwise. The lidar chips take their state and rate from a fresh room status, and from the probe when that status is stale.

## Build

From the repository root, on macOS 14 or newer:

```bash
./build.sh --no-install
open ./build/LabCommand.app
```

`./build.sh` with no flag also replaces `~/Applications/LabCommand.app`. `--no-install` leaves that copy alone. The script builds the release binary, ad-hoc codesigns the bundle, and copies `scripts/spark-capture.sh`, `fixtures/api`, the room hand goldens, and `fixtures/pointclouds/mid360-frame.pcd`.

The hosted Mac job is macOS 15 with Xcode 16.4. It runs `swift build -c release --product LabCommand`. It does not run `./build.sh`, and it does not install the app.

## Architecture

Three pieces. The wire rules live in the protocol doc.

**Spark room-memory node.** Production code is `spark/room_memory_cpp` (`node/room_memory_node.cpp`, `launch/room_memory_cpp.launch.py`). `spark/room_memory` is the Python reference oracle. Differential tests check parity under the tolerances in [PLAN-RIGGUARD.md](PLAN-RIGGUARD.md).

The node subscribes to `/livox/lidar` and `/lidar_points` with sensor-data QoS and closes a window every `window_s` (default 1 second). It publishes `/room_memory/status` and `/room_memory/activity` at 1 Hz (reliable, transient_local). Each transition goes out on volatile `/room_memory/events`. A window that has a map update publishes volatile `/room_memory/map_bricks`. It answers `RequestSync` on the sync topics and then sends `/room_memory/sync/end`.

The C++ launch file also starts foxglove_bridge on **8767**. The topic whitelist is `^/room_memory/.*`. Services are limited to `/room_memory/sync/request`. Client publishing stays off unless `foxglove_client_publish:=true`. Raw clouds do not leave through 8767. The Python launch file does not start this bridge. Neither node publishes `/room_memory/epoch`.

`rig_guard` defaults to false. With `rig_guard:=true` the node publishes `RigPose` on `/room_memory/rig` (reliable, transient_local, on change and every 10 seconds). `state_dir` defaults to `~/room-memory`. An empty `state_dir` value, or a directory that does not exist, stops the node; it never creates one. References are saved in `<state_dir>/<room_id>/`. With the guard, registration clouds and `rig_state` are saved there too. The event log is not.

**Protocol.** [docs/ROOM-MEMORY-PROTOCOL.md](docs/ROOM-MEMORY-PROTOCOL.md) is classic CDR. The Mac mirror follows §5 (publisher generation, `RequestSync`, `SyncEnd`). The `.msg` files are in `ros/labcommand_msgs/`. `OperatorAction` is a `.msg` only: no codec and no node handler (§3.6, §9).

**Mac mirror.** `LiveRoomClient`'s default URL is `ws://100.118.187.99:8767` (`CaptureRecipe.tailscaleIP`, port 8767). It subscribes to the `/room_memory/*` topics it knows — `map_bricks`, `events`, `status`, `activity`, `epoch`, and the sync topics including `sync/end` — and reduces them in `RoomMemoryMirror`. The Room page binds to that mirror ([Sources/LabCommandCore/LiveRoom/API.md](Sources/LabCommandCore/LiveRoom/API.md)). Swift can decode `RigPose`. The app does not subscribe to `/room_memory/rig`, and the Room page does not show it.

The Rig tab is a separate path. It opens two SSH tunnels: Foxglove **8765** for raw clouds, and the command-center API **8766**. While auto-refresh is on, the rig poll runs every 5 seconds. The room client is its own socket. The command-center process is not in this repository.

## Known limits

**Each lidar stays in its own frame.** The JT128 to Mid-360 extrinsic is not measured. Each sensor keeps a reference in that sensor's frame. `map_bricks` publishes one of them: `map_sensor`, the JT128 when it is configured. Events from the other sensor stay in that sensor's frame. The nominal Mid-360 mount in `memory.py` (`MOUNT`) is a rotation prior for the rig guard, never an extrinsic. `RigTransforms.livoxToWorld` is the Rig tab's display transform for the 3D overlay. The Room map does not use it.

**Events are lost when the node restarts, until [issue #32](https://github.com/trevcong/labcommand/issues/32).** The event log keeps the last 100 transitions in memory (`EVENT_LOG_DEPTH`, `kEventLogDepth`). `save` and `load` persist the reference bricks and, with the guard, the registration clouds and `rig_state`. They do not persist the event log. This tree has no `eventlog` source. A restart allocates a new publisher generation, and `RequestSync` on the new process cannot replay the previous run. The Room page says events from the earlier run may be missing. Issue #32 is open. Its durable log is not on this tree.

**The tracker does not publish CONFIRMED.** The transitions this tree appends are `CANDIDATE` and `EXPIRED`. The Mac can render the other states when a message carries them.

**A request path exists only when the bridge offers one.** This repo's C++ launch file advertises `/room_memory/sync/request`. When the running image does not, the client counts the request unsent and the event column says it cannot ask the Spark (`can't ask the Spark for them`). Protocol §9 records that the deployed image was still the pre-D39 image when that section was written. This page describes the code. It does not claim the lab image was checked.

**CI is not a live-rig proof.** The Ubuntu job skips ROS tests (`test_node.py` needs `rclpy` and `labcommand_msgs`) and `rosbags`. From `spark/`, pytest also collects the C++ differential and recording tests. `differential/test_differential.py` skips unless both `ROOM_MEMORY_OFFLINE` and `ROOM_MEMORY_REPLAY` are set. `recording/test_static_recording.py` skips while `ROOM_MEMORY_BAG` is unset, and at collection when `mcap` cannot be imported. Most of `recording/test_tools.py` runs in that pass; only `test_statuses_tick_every_second_independently_of_the_window` waits on `ROOM_MEMORY_BAG`. CTest runs the differential and recording-tool tests with those paths set. `static_longrun`, `static_longrun_b`, and `static_recording` must skip, because the full recordings are not in git. Unexpected CTest skips, including nested ones, fail the gate. The Mac job runs the Swift test runner and a release build of the app. It does not launch or install it.

## Tests

Mac, from the repo root:

```bash
swift run LabCommandCoreTestsRunner
swift run change-detect -- before.pcd after.pcd
swift run change-detect --evidence --json before.bin after.bin
```

`change-detect` diffs two clouds of the same sensor (`.pcd` or CDR `.bin`). It does not align them. `--json` prints the JSON record and exits. `--evidence` carves free space from the after cloud. Those flags belong before the two paths: the tool treats every argument after a `--` as a path, and `swift run` passes that `--` through.

Spark. `spark/.venv-local` is gitignored. When that interpreter exists, CMake uses it for the differential tests. There is no `spark/pytest.ini`. Collection from `spark/` includes `room_memory/tests`, `room_memory_bench/tests`, and the C++ `tests/differential` and `tests/recording` trees. `test_differential.py` skips unless both `ROOM_MEMORY_OFFLINE` and `ROOM_MEMORY_REPLAY` are set. `test_static_recording.py` skips while `ROOM_MEMORY_BAG` is unset, and at collection when `mcap` cannot be imported. Most of `test_tools.py` runs without those binaries; only `test_statuses_tick_every_second_independently_of_the_window` waits on `ROOM_MEMORY_BAG`. That is what the first Ubuntu pytest step does too. Create the venv with Python 3.12, the version CI installs. `spark/requirements-ci.txt` pins `numpy==2.0.2`.

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
ctest --test-dir build/room_memory_cpp --output-on-failure
```

The hosted job adds `-DROOM_MEMORY_NATIVE=OFF`, checks that zstd was found before the build, writes JUnit reports, and rejects unexpected skips, including nested ones. The three absent-dataset tests must still skip.

## Hosted CI

The [hosted gate](.github/workflows/ci.yml) runs on pull requests and on pushes to `main`. A concurrency group per ref cancels an in-progress run on pull requests only.

Ubuntu 24.04 installs [the CI requirements](spark/requirements-ci.txt) for Python 3.12, runs pytest from `spark/`, builds the non-ROS C++ targets, and runs CTest with that same interpreter. It also checks committed fixtures and compares the hand-golden `.hex` sets byte for byte. macOS 15 with Xcode 16.4 runs `swift run LabCommandCoreTestsRunner` and `swift build -c release --product LabCommand`.

The job summary counts pytest cases and CTest entries separately, including skips and reasons. The first pytest pass skips ROS and tests that need the C++ executables. CTest then runs the differential and recording-tool tests with those paths set. The full static recordings are not in git: `static_longrun`, `static_longrun_b`, and `static_recording` must skip for missing datasets. Missing decoder dependencies, missing committed fixtures, unexpected CTest skips (including nested cases), and a missing or invalid report fail the gate. Verbose CTest logs keep the inner test counts. These checks do not establish ROS, full-recording, or live-rig correctness, and they do not launch, install, or deploy the app.

Only pip downloads are cached, keyed by the requirements file. CMake and Swift outputs are built fresh each job.

## Docs

| Path | What it is |
|---|---|
| [docs/ROOM-MEMORY-PROTOCOL.md](docs/ROOM-MEMORY-PROTOCOL.md) | Wire protocol, including D30 and D34–D44. §9: the Mac does not subscribe to `/room_memory/rig` |
| [Sources/LabCommandCore/LiveRoom/API.md](Sources/LabCommandCore/LiveRoom/API.md) | Mac mirror: client, mirror, screen strings |
| [docs/CHANGE-DETECTION.md](docs/CHANGE-DETECTION.md) | Occupancy diff shared by Swift, Python, and C++ |
| [spark/room_memory_cpp/PLAN.md](spark/room_memory_cpp/PLAN.md) | Production C++ node and windows. `Epoch` is not published. Its `rig_guard` note points at [PLAN-RIGGUARD.md](PLAN-RIGGUARD.md) |
| [spark/room_memory/PLAN.md](spark/room_memory/PLAN.md) | Python oracle |
| [PLAN-RIGGUARD.md](PLAN-RIGGUARD.md) | Rig-bump guard on the Spark. v1 is phases 1–2. A Mac `RigPose` display is later |
| [PLAN-PUBGEN.md](PLAN-PUBGEN.md) | Publisher generations and the mirror's restart rules |
| [design-refs/DESIGN-VISION.md](design-refs/DESIGN-VISION.md) | Visual contract the design tests pin |
| [.github/workflows/ci.yml](.github/workflows/ci.yml) | Hosted gate |
