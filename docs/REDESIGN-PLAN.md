# LabCommand Redesign Plan

**Vision**: A Palantir Foundry/Gotham-style ops console for the dual-lidar rig, with a live 3D point-cloud viewer as the centerpiece.

---

## 1. The 3D Point-Cloud Viewer

The main canvas shows live point clouds from both sensors, overlaid with correct extrinsics. This is what makes LabCommand a command center, not a status page.

### Decision: Native Metal Renderer

**Why Metal over alternatives**:
- Both sensors produce ~1.35M pts/s combined (Mid-360 200k + JT128 1.15M)
- At ~2 GB/min, only GPU-native rendering keeps up
- Metal renders 10M+ points at 60fps with instanced geometry
- SceneKit/RealityKit add abstraction overhead and limit shader control
- Embedded Foxglove (WKWebView) has 50-100ms latency and requires the tunnel — no offline use
- Native Metal works fully offline with fixture data and recorded bags

**Data sources** (in priority order):
1. **Fixture files** — For dev/testing without Spark (primary during this phase)
2. **Recorded mcap bags** — For playback with timeline scrubbing
3. **Live WebSocket** — From foxglove_bridge :8765 over tunnel (once Spark reachable)

### Technical Design

```
Sources/LabCommand/
  PointCloudView.swift      # SwiftUI MTKView wrapper
  PointCloudRenderer.swift  # Metal pipeline: instanced points, color modes
  PointCloudDecoder.swift   # PointCloud2 binary → GPU vertex buffer
  PointCloudCamera.swift    # Orbit (default) + fly mode
  McapReader.swift          # Recorded bag playback
  WebSocketClient.swift     # Live stream (Phase B — requires Spark)
```

### Capabilities

| Feature | Implementation |
|---------|----------------|
| **Point coloring** | Intensity (grayscale), height (rainbow), sensor (Mid=cyan, JT=magenta) |
| **Camera** | Orbit: trackpad rotate/zoom/pan. Fly: WASD+mouse |
| **Dual overlay** | Static extrinsics from RigSpec (Mid-360: 11.0, 23.29, −44.12 mm). JT128 fused on Mid's IMU frame |
| **Decimation** | Every 4th point for display (~340k pts/s). Full resolution stays in bag |
| **Bag playback** | mcap files via [mcap Swift package](https://github.com/foxglove/mcap). Timeline scrubber, pause, frame-step |

### Fixtures for Testing Without Spark

```
fixtures/
  pointclouds/
    mid360-frame.pcd         # 50k points, PCD ASCII (easy to inspect/edit)
    jt128-frame.pcd          # 50k points
    dual-frame.pcd           # Both sensors, pre-transformed, 100k points
    empty.pcd                # Zero points (parked state)
    room-scan.mcap           # Short recorded bag (~5s, ~100MB)
```

PCD format for fixtures (human-readable, standard):
```
# .PCD v0.7 - Point Cloud Data
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 50000
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 50000
DATA ascii
1.234 5.678 9.012 0.85
...
```

Decoder handles both PCD (fixtures) and PointCloud2 ROS binary (live/mcap).

---

## 2. Information Architecture

One screen. No tabs. Everything an operator needs in one view.

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│ ● ● ●  Lab                    [Spark ●] [Mid-360 ●] [JT128 ●]  clocks Δ2.3ms  ⚙    │
├────────────────────────────────────┬────────────────────────────────────────────────┤
│ RIG STATUS                         │                                                │
│ ● Both sensors live                │           ┌─────────────────────────┐          │
│   ready to record                  │           │                         │          │
│                                    │           │    LIVE 3D CANVAS       │          │
│ ┌────────────┐ ┌────────────┐      │           │    (Metal point cloud)  │          │
│ │ Mid-360    │ │ JT128      │      │           │                         │          │
│ │ ● Live     │ │ ● Live     │      │           │    Mid-360: cyan        │          │
│ │ 9.8 Hz     │ │ 10.2 Hz    │      │           │    JT128: magenta       │          │
│ │ +2.1 ms    │ │ −0.2 ms    │      │           │                         │          │
│ └────────────┘ └────────────┘      │           │    orbit · intensity    │          │
│                                    │           └─────────────────────────┘          │
│ CLOCKS                             │                                                │
│ ptp master ● · Δ 2.3 ms            │ ────────────────────────────────────────────── │
│ ████████░░░░░░░░░░░░ mid slewing   │ EVENT LOG                                      │
│                                    │ 14:32:15 probe  Both sensors live              │
│ RECORDING                          │ 14:31:10 start  start rig ok                   │
│ [ Start recording ]  847 GB free   │ 14:30:55 watch  watchdog: Mid-360 restarting   │
│ or: [ Arm at lock ]                │                                                │
├────────────────────────────────────┴────────────────────────────────────────────────┤
│ [▶] ════════════════════════════●═══════════════════════════════════════════ 02:34  │
└─────────────────────────────────────────────────────────────────────────────────────┘
         ^ timeline/scrubber (for recorded bags; hidden when live)
```

### Components

| Area | Content |
|------|---------|
| **Top bar** | App title, device pips (Spark/Mid/JT), clock delta summary, settings gear |
| **Left column** | Rig status headline, sensor cards (2), clocks section, recording controls |
| **Main canvas** | Live 3D point cloud (Metal), camera controls overlay |
| **Right strip** | Event log (last 5-8 lines, scrollable) |
| **Bottom** | Timeline scrubber (bags only; hidden when live) |

### Alerts

Alerts overlay the canvas when action needed:
- **Spark offline**: Full-width amber banner, "Check Tailscale" action
- **Clock converging**: Inline in clocks section, progress bar + ETA
- **Recording**: Red pip pulse on record button, elapsed time

---

## 3. Inventory (Aggressive Cut)

**Keep only**: hardware connection, 3D view, clocks/PTP, recording, event log.

### Views (Sources/LabCommand)

| File | Verdict | Reason |
|------|---------|--------|
| `LabCommandApp.swift` | **KEEP** | Entry point, window, menu bar |
| `AppState.swift` | **KEEP** | State management — strip non-essential state |
| `CommandCenterView.swift` | **REWRITE** | Becomes single-screen ops console layout |
| `RigView.swift` | **CUT** | Replaced by unified console view |
| `DevicesView.swift` | **CUT** | Fold essential cards into left column |
| `PipelineView.swift` | **CUT** | Signal chain diagram is nice but not essential — status bar covers it |
| `MissionClockView.swift` | **CUT** | Fold clocks section inline |
| `SessionLibraryView.swift` | **CUT** | Sessions accessible via timeline/file menu, not a tab |
| `ResearchView.swift` | **CUT** | Dead wrapper |
| `TrajectoryMap.swift` | **CUT** | Nice for review but not ops — defer to Foxglove |
| `SettingsView.swift` | **KEEP** | Minimal prefs sheet (gear icon) |
| `Theme.swift` | **KEEP** | Design system |
| `Controls.swift` | **KEEP** | Button styles |
| `InstrumentChart.swift` | **CUT** | Sparklines not needed — rates shown as numbers |
| `ModuleSlot.swift` | **CUT** | Unused scaffolding |
| `ConnectivityService.swift` | **KEEP** | SSH/Tailscale probes |
| `RigService.swift` | **KEEP** | Tunnels, API |

**New files**:
| File | Purpose |
|------|---------|
| `OpsConsoleView.swift` | The single-screen layout |
| `PointCloudView.swift` | SwiftUI wrapper for Metal canvas |
| `PointCloudRenderer.swift` | Metal rendering |
| `PointCloudDecoder.swift` | PointCloud2 parsing |
| `PointCloudCamera.swift` | Orbit/fly controls |
| `WebSocketClient.swift` | foxglove_bridge consumer |
| `McapReader.swift` | Recorded bag playback |

### Core Library (Sources/LabCommandCore)

| File | Verdict | Reason |
|------|---------|--------|
| `RigModels.swift` | **KEEP** | Domain models |
| `RigHealth.swift` | **KEEP** | Health band |
| `RigGate.swift` | **KEEP** | Action gates |
| `RigDesk.swift` | **SLIM** | Keep button specs, cut verbose copy |
| `Pipeline.swift` | **CUT** | Audit checks were for the graph — not needed |
| `SessionLibrary.swift` | **CUT** | Sessions via file picker, not library view |
| `SessionTrajectory.swift` | **CUT** | Trajectory map is cut |
| `TelemetryHistory.swift` | **CUT** | No sparklines |
| `CaptureModels.swift` | **KEEP** | Recording workflow |
| `CaptureManifest.swift` | **KEEP** | Recording metadata |
| `CaptureTelemetry.swift` | **SLIM** | Keep TelemetryFormat, cut unused |
| `CaptureRecipe.swift` | **KEEP** | Constants |
| `ConnectionStatus.swift` | **KEEP** | Enums |
| `StatusAggregator.swift` | **CUT** | Was for probe list |
| `DeviceRecovery.swift` | **CUT** | Verbose recovery text — inline in alerts |
| `MissionLog.swift` | **KEEP** | Event log |
| `ProbeText.swift` | **CUT** | Merge into TelemetryFormat |
| `SparkSSH.swift` | **KEEP** | Hop logic |
| `TailscaleStatusParser.swift` | **KEEP** | Network |
| `LabMetricsLoader.swift` | **CUT** | Unused |

### Summary

| | Before | After |
|-|--------|-------|
| Views | 17 | 9 (6 kept + 3 rewritten/new renderer) |
| Core | 20 | 12 |
| **Total** | 37 | 21 + 5 new = 26 |

Cut 17 files, add 5 new (renderer stack).

---

## 4. Verification (No Spark Required)

All development and testing works without access to the Spark.

### Fixtures

```
fixtures/
  api/                           # Mock command-center responses
    both-sensors-live.json
    both-sensors-dark.json
    clock-converging.json
    clock-locked.json
    recording-active.json
    spark-offline.json
  pointclouds/                   # Static point cloud frames
    mid360-frame.pcd             # 50k points, PCD ASCII
    jt128-frame.pcd
    dual-frame.pcd               # Both sensors overlaid
    empty.pcd
    room-scan.mcap               # 5s recorded bag for playback testing
```

### Test Tiers

1. **Core tests** (`swift run LabCommandCoreTestsRunner` on Linux) — decoder, gates, health, PCD parser
2. **Renderer tests** (Mac only) — fixture → vertex buffer → render to offscreen texture → snapshot compare
3. **Design contract tests** — typography ≤15pt, spacing tokens, banned words

### Build Verification

- **Linux CI**: Core logic tests pass
- **Trevor local**: `./build.sh` + launch with `--mock both-sensors-live` + load fixture point clouds
- **Future**: Self-hosted Mac worker for automated screenshots

---

## 5. Implementation PRs

### Phase A: No Spark Required

Everything here builds and tests without Spark access.

**PR 1: Fixture infrastructure**
- `fixtures/api/*.json` — mock command-center responses
- `fixtures/pointclouds/*.pcd` — sample point cloud frames
- `fixtures/pointclouds/room-scan.mcap` — short recorded bag
- `MockRigService` with `--mock <fixture>` launch arg
- Core tests for decoder round-trips

**PR 2: Cut dead code**
- Delete 17 files per inventory
- Strip unused state from `AppState`
- Verify build + tests pass

**PR 3: PCD + PointCloud2 decoder**
- `PointCloudDecoder.swift` — parse PCD ASCII and PointCloud2 binary
- Output: `[SIMD3<Float>]` positions + `[Float]` intensities
- Unit tests against fixture files

**PR 4: Metal renderer**
- `PointCloudRenderer.swift` — instanced point shader, color modes
- `PointCloudView.swift` — SwiftUI MTKView wrapper
- Render fixture PCD to screen (static camera)
- Shader: point size, intensity/height/sensor coloring

**PR 5: Camera controls**
- `PointCloudCamera.swift` — orbit (default), fly (WASD+mouse)
- Trackpad: two-finger rotate, pinch zoom, three-finger pan
- Smooth interpolation, bounds clamping

**PR 6: Dual-sensor overlay**
- Apply static extrinsics from `RigSpec`
- Load both `mid360-frame.pcd` and `jt128-frame.pcd`
- Color: Mid-360 = cyan, JT128 = magenta

**PR 7: Mcap bag playback**
- `McapReader.swift` — parse mcap via foxglove/mcap Swift package
- Timeline scrubber below canvas
- Play/pause, frame-step, seek

**PR 8: Ops console layout**
- `OpsConsoleView.swift` — single-screen: left column + canvas + event log
- Replace tab navigation with unified view
- Point cloud canvas as main area
- Sensor cards, clocks section, recording controls in left column
- Event log in right strip

**PR 9: Recording controls**
- Start/Stop/Arm buttons (disabled when mock, wired to `RigService` actions)
- Elapsed time display, disk free
- Red pip pulse when recording

**PR 10: Polish + docs**
- Design contract tests (15pt cap, spacing tokens)
- Settings sheet (gear icon)
- README update

### Phase B: Spark Required (Trevor runs once connected)

These PRs require Spark access for testing. Trevor completes locally.

**PR 11: Live WebSocket stream**
- `WebSocketClient.swift` — connect to foxglove_bridge :8765 over tunnel
- Subscribe to `/livox/lidar` and `/lidar_points` topics
- Decimation: every 4th point for display
- Reconnect logic on tunnel drop

**PR 12: Spark-side verification**
- Verify foxglove_bridge publishes both point cloud topics
- Verify `/tf` includes sensor→base transforms (or confirm static from RigSpec suffices)
- Document any Spark-side setup in `docs/SPARK-SETUP.md`

---

## Summary

LabCommand becomes a single-screen ops console with a live 3D point-cloud viewer as centerpiece.

- **Metal renderer** handles ~1.35M pts/s, orbit/fly camera, dual-sensor overlay, bag scrubbing
- **17 files cut**, 6 new files (renderer + console)
- **10 PRs buildable without Spark**, 2 PRs require Spark connection
- 3D viewer arrives early (PRs 3-7) — Trevor sees points on screen fast, with fixture data
