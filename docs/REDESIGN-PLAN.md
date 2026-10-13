# LabCommand Redesign Plan

Palantir-style ops console for the dual-lidar rig (Livox Mid-360 + Hesai JT128 on DGX Spark).

---

## 1. Inventory

### Views (Sources/LabCommand)

| File | Purpose | Verdict | Reason |
|------|---------|---------|--------|
| `LabCommandApp.swift` | App entry, main window, menu bar extra | **KEEP** | Core shell, menu-bar status, keyboard shortcuts |
| `AppState.swift` | Central view model, rig polling, actions, timers | **KEEP** | Core state management, action dispatch |
| `CommandCenterView.swift` | Top nav bar + destination router | **KEEP** | Shell chrome, sensor/Spark chips in top bar — simplify nav |
| `RigView.swift` | Main rig page: status headline, pipeline graph, readout, takes, planner | **KEEP** | Primary operator view — refocus as dashboard |
| `DevicesView.swift` | Hardware cards, containers, dossiers, probe log | **KEEP** | Essential hardware diagnostics |
| `PipelineView.swift` | Signal chain graph (sensors → Spark → outputs), audit strip | **KEEP** | Core connectivity visualization — promote to main canvas |
| `MissionClockView.swift` | Armed take lifecycle, lock instrument, gate preflight | **KEEP** | Recording orchestration — essential for custody workflow |
| `SessionLibraryView.swift` | Takes table, filtering, compare, notes | **KEEP** | Session management — merge with ResearchView |
| `ResearchView.swift` | Wrapper for trajectory + session library | **MERGE** | Thin wrapper; fold into unified Takes page |
| `TrajectoryMap.swift` | Walk route visualization (odometry poses) | **KEEP** | Useful for reviewing sessions — keep as detail view |
| `SettingsView.swift` | Host config, probe intervals, key bindings | **KEEP** | Necessary prefs — slim it down |
| `Theme.swift` | Color palette, typography, spacing tokens | **KEEP** | Design contract implementation |
| `Controls.swift` | Button styles (Quiet, Primary, Hero, Ghost) | **KEEP** | Reusable components |
| `InstrumentChart.swift` | Sparkline, InstrumentCell, HairlineRow | **KEEP** | Core telemetry display components |
| `ModuleSlot.swift` | Generic slot/berth components | **CUT** | Unused scaffolding for empty berths — banned by contract |
| `ConnectivityService.swift` | SSH/Tailscale probes | **KEEP** | Hardware connectivity checks |
| `RigService.swift` | SSH tunnels, API calls to command center | **KEEP** | Core I/O layer |

### Core Library (Sources/LabCommandCore)

| File | Purpose | Verdict | Reason |
|------|---------|---------|--------|
| `RigModels.swift` | Sensor/clock/recording/pipeline data structures, decoder | **KEEP** | Essential domain models |
| `RigHealth.swift` | Health band evaluation (Ready/Watch/Parked/Blocked) | **KEEP** | Core status logic |
| `RigGate.swift` | Action legality checks | **KEEP** | Gate logic for UI |
| `RigDesk.swift` | Operator copy, stage states, button specs, diagnose | **KEEP** | UI copy and action logic |
| `Pipeline.swift` | Audit checks, spec dossiers, gate preflight | **KEEP** | Pipeline health assessment |
| `SessionLibrary.swift` | Session indexing, filtering, compare | **KEEP** | Session management |
| `SessionTrajectory.swift` | Trajectory data structures, loader | **KEEP** | Walk visualization |
| `TelemetryHistory.swift` | Rolling telemetry samples, trend analysis | **KEEP** | Essential for sparklines |
| `CaptureModels.swift` | Capture presets, session specs | **KEEP** | Recording workflow |
| `CaptureManifest.swift` | Capture manifest structures | **KEEP** | Recording metadata |
| `CaptureTelemetry.swift` | Telemetry formatting | **KEEP** | Display formatting |
| `CaptureRecipe.swift` | Connection constants, paths | **KEEP** | Configuration |
| `ConnectionStatus.swift` | Connection status enum | **KEEP** | Status modeling |
| `StatusAggregator.swift` | Aggregates check outcomes | **KEEP** | Status rollup |
| `DeviceRecovery.swift` | Recovery guidance | **KEEP** | Operator hints |
| `MissionLog.swift` | Event log structures | **KEEP** | Activity timeline |
| `ProbeText.swift` | Telemetry text formatting | **KEEP** | Display helpers |
| `SparkSSH.swift` | SSH hop logic | **KEEP** | Connection management |
| `TailscaleStatusParser.swift` | Tailscale JSON parsing | **KEEP** | Network status |
| `LabMetricsLoader.swift` | Metrics loading | **CUT** | Unused module metrics scaffolding |

### Summary

- **KEEP**: 28 files (core functionality)
- **CUT**: 2 files (unused scaffolding: `ModuleSlot.swift`, `LabMetricsLoader.swift`)
- **MERGE**: 1 file (`ResearchView.swift` → fold into Takes)

---

## 2. Target Information Architecture

### Top-Level Structure

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ ● ● ●  Lab — dual-lidar rig                    [Spark ●] [Mid-360 ●] [JT128 ●] │
├─────────────────────────────────────────────────────────────────────────────────┤
│  [Rig]  [Hardware]  [Takes]  [Settings]                          ⟳ Refresh     │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│                           (destination content)                                 │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Rig Page (Primary View)

The main view an operator lives on. One glance: is it connected? Is it healthy? What do I do?

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ RIG STATUS BAR                                                                  │
├─────────────────────────────────────────────────────────────────────────────────┤
│  ● Both sensors live — ready to record                    [ Start recording ]   │
│    clocks Δ 2.3 ms · 847 GB free                                               │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  SIGNAL CHAIN                                                                   │
│  ┌──────────┐     udp 56301    ┌──────────┐   /livox/lidar  ┌──────────┐       │
│  │ Mid-360  │ ──────────────▶  │          │ ──────────────▶ │lidar-dual│       │
│  │ ● Live   │     ptp 1588v2   │  Spark   │   /lidar_pts    │          │       │
│  │ 9.8 Hz   │ ◀- - - - - - - - │ enP7s7   │ ──────────────▶ │ running  │       │
│  └──────────┘                  │          │                 └──────────┘       │
│                                │ ptp GM   │                                     │
│  ┌──────────┐     udp 2368     │ Δ 2.3 ms │   both imus     ┌──────────┐       │
│  │ JT128    │ ──────────────▶  │          │ ──────────────▶ │ Foxglove │       │
│  │ ● Live   │     ptc 9347     │ load 0.4 │                 │ :8765    │       │
│  │ 10.2 Hz  │ ◀- - - - - - - - │          │                 │ tunnel   │       │
│  └──────────┘                  └──────────┘                 └──────────┘       │
│                                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────────┐ │
│  │ AUDIT: link 1000/full ● | pkt loss 0 ● | ports 3/3 ● | ptp GM ● | ...    │ │
│  └───────────────────────────────────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────────────────────┤
│ READOUT                                                                         │
│ ┌─────────────┬─────────────┬─────────────┬─────────────┐                       │
│ │ Mid points  │ JT points   │ Spark       │ Disk        │                       │
│ │ 9.8 Hz      │ 10.2 Hz     │ load 0.4    │ 847 GB free │                       │
│ │ ▁▂▃▄▅▆▇    │ ▁▂▃▄▅▆▇    │ ▁▂▃▄▅▆▇    │             │                       │
│ └─────────────┴─────────────┴─────────────┴─────────────┘                       │
├─────────────────────────────────────────────────────────────────────────────────┤
│ EVENT LOG (last 5)                                                              │
│ ┌───────────────────────────────────────────────────────────────────────────┐   │
│ │ 14:32:15  probe   Both sensors live — ready to record                     │   │
│ │ 14:31:10  start   start rig ok                                            │   │
│ │ 14:30:55  info    watchdog: Mid-360 powered, restarting stack             │   │
│ └───────────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Device Cards (Hardware Page)

Per-device status at a glance:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ HARDWARE                                                                        │
│ This Mac reaches Spark over Tailscale; both lidars live on Spark's wired port. │
├─────────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐  ┌─────────────────────┐                               │
│  │ ● connected         │  │ ● connected         │                               │
│  │ Spark               │  │ Tailscale           │                               │
│  │ ssh spark → dgx-01  │  │ 4 peers             │                               │
│  └─────────────────────┘  └─────────────────────┘                               │
│                                                                                 │
│  ┌─────────────────────┐  ┌─────────────────────┐                               │
│  │ ● Live              │  │ ● Live              │                               │
│  │ Livox Mid-360       │  │ Hesai JT128         │                               │
│  │ 9.8 Hz · imu 200 Hz │  │ 10.2 Hz · imu 199   │                               │
│  │ clock +2.1 ms       │  │ clock −0.2 ms       │                               │
│  │ 192.168.1.135       │  │ 192.168.118.201     │                               │
│  └─────────────────────┘  └─────────────────────┘                               │
│                                                                                 │
│  CONTAINERS                                                                     │
│  [lidar-dual running] [ptp-master running] [watchdog running] [center running] │
│                                                                                 │
│  CLOCKS                                                                         │
│  ptp master up · clocks Locked Δ 2.3 ms                                         │
│  Mid-360: +2.1 ms vs host · JT128: −0.2 ms vs host                              │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Recording Flow (when armed/recording)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ MISSION CLOCK                                                                   │
│ ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐     │
│ │ Armed   │ ─── │ Lock    │ ─── │ Record  │ ─── │Finalize │ ─── │ Gate    │     │
│ │ ●       │     │ ● wait  │     │ ○ pend  │     │ ○ pend  │     │ ○ pend  │     │
│ │ A_walk1 │     │ 2/3     │     │ 60s     │     │ sidecar │     │ scored  │     │
│ └─────────┘     └─────────┘     └─────────┘     └─────────┘     └─────────┘     │
│                                                                                 │
│ CLOCK OFFSET (±50 ms band)                                          [Disarm]   │
│ ┌───────────────────────────────────────────────────────────────────────────┐   │
│ │ ██████████████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ │   │
│ │ mid +47 ms                                                        jt −2 ms │   │
│ └───────────────────────────────────────────────────────────────────────────┘   │
│ slew +8.2 ms/min · in band ≈ 4 min                                              │
│                                                                                 │
│ GATE PREFLIGHT                                                                  │
│  ● clocks in band    holds    mid +47 · jt −2 · ±50 ms                          │
│  ● both streaming    holds    mid 9.8 Hz · jt 10.2 Hz                           │
│  ● loss deltas zero  holds    Δ0 mac · Δ0 InErrors · Δ0 Rcvbuf                  │
│  ○ calibration       at fin   signed calibration.json on Spark                  │
│  ○ hashes            at fin   bag · angle csv · ptp4l log                       │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Visual Direction Notes

Building on `design-refs/DESIGN-VISION.md` and the reference images:

### Reference Image Analysis

1. **01_industrial_hmi.jpg** — KAIROS-style lab health gauge with glowing vertical bar, live lidar workbench radar view, timeline strip. Dense, functional, amber/teal on black.

2. **02_flight_deck.jpg** — Lab Command flight deck: sensor cards left, main radar canvas center, pipeline stages right. Purple/cyan accent on deep black. Structured but busy.

3. **03_spatial_glass.jpg** — Connectivity mesh visualization, process fabric cards, translucent glass aesthetic. Cool blue on black. More ethereal, less utilitarian.

4. **04_brutalist_founder.jpg** — "CONNECTED 280pt SPARK" hero type, system status sidebar, radar display, Founder Mode card grid. Pink/green on black. Bold, retro-industrial.

### Design Direction for Redesign

**What to draw from**: The KAIROS (#01) and flight deck (#02) approaches — dense but structured, functional over decorative, hardware-focused status at a glance.

**What to avoid**: The glass aesthetic (#03) is too soft for hardware ops; the brutalist founder style (#04) is fun but the giant display type violates the 15pt chrome cap contract.

**Key principles from DESIGN-VISION.md**:

- **Palette**: `#0B0D10` bg, `#12161A` surfaces, `#4A8FB8` accent (cool steel, not cyan neon), status colors as 6px pips not filled billboards
- **Typography**: SF Pro, 15pt cap for destination titles, 13pt body, 11pt mono telemetry. Never display Hz/byte figures larger than 13pt
- **Status language**: Live/Idle/Blocked/— for stages; Dark/Powered/Live/— for sensors; Ready/Watch/Parked/Blocked for rig health
- **Motion**: 180ms ease-out on state changes only; breathing pip only while actually `.running`; no decorative animation
- **Banned**: Thesis copy, fake metrics, neon HUD, gradient glows, empty columns, "coming soon"

**Palantir-style ops console characteristics to incorporate**:

1. **Hierarchical density** — Most important info largest/first (rig status bar), detail available on demand (disclosure groups)
2. **One truth** — All data from the command-center API; UI invents nothing; `nil` renders as "unavailable"
3. **Calm degradation** — Parked (both sensors dark) is steel accent, not amber/red; offline is a first-class state
4. **Signal chain visibility** — The pipeline graph shows every hop, labeled with actual protocols and ports
5. **Audit strip** — Per-hop health checks with pass/watch/fail/pending/unavailable verdicts

---

## 4. Verification Approach

### Challenge

- macOS/SwiftUI app requires Xcode and real hardware for full testing
- Cloud agents run on Linux VMs with no Xcode
- Real Spark + sensors not available for automated testing

### Mock Command-Center API

Create fixture JSON files that simulate every rig state:

```
fixtures/
  api/
    both-sensors-live.json       # normal operation, ready to record
    both-sensors-dark.json       # parked rig, steel accent
    mid-live-jt-standby.json     # one sensor live, one in standby
    recording-active.json        # bag being written
    clock-converging.json        # Mid-360 slewing toward lock
    clock-locked.json            # both clocks in band
    spark-offline.json           # no SSH path
    api-down.json                # Spark reachable, API not answering
    armed-waiting.json           # armed take, waiting for lock
    armed-recording.json         # armed take, recording phase
    last-gate-passed.json        # finalize verdict: passed
    last-gate-failed.json        # finalize verdict: failed clocks
```

Each fixture includes the full `status.json` shape:
- `rates`, `offsets`, `containers`, `pipeline`, `recording`, `armed`, `last_gate`
- All fields the decoder expects, so `RigStatusDecoder.parse()` produces a real `RigSnapshot`

### Test Tiers

1. **Unit tests (LabCommandCoreTestsRunner)** — Run on Linux, test decoder, gates, health, pipeline audit against fixtures
   ```bash
   swift run LabCommandCoreTestsRunner
   ```

2. **Snapshot tests** — Compare `RigStatusDecoder.parse(fixture)` output to pinned expectations; catch decoder drift

3. **Design contract tests** — Already exist in `DesignContractTests.swift`; extend to pin:
   - No placeholder chrome
   - Type scale ≤ 15pt
   - Spacing tokens match DESIGN-VISION.md
   - Banned words ("coming soon", "not wired yet")

### Swift Builds and Screenshots

Since cloud VMs are Linux:

**Option A: Self-hosted Mac worker**
- Trevor runs a Mac Mini as a self-hosted worker with Xcode installed
- CI jobs labeled `runs-on: macos` build the app and capture screenshots
- Screenshots committed to `artifacts/screenshots/` for visual review

**Option B: Trevor's local run**
- Changes merged to branch; Trevor pulls and runs `./build.sh` locally
- Captures screenshots of each state by launching with `--mock <fixture>`
- Uploads screenshots to PR as artifacts

**Option C: SwiftUI Previews (if configured)**
- Previews can render in Xcode Cloud or local Xcode
- Export preview images as verification artifacts

### Recommended Approach

For this redesign, use **Option B** initially (Trevor's local verification) with the following workflow:

1. Agent pushes code changes to PR
2. Agent runs `swift run LabCommandCoreTestsRunner` on Linux to verify core logic
3. Trevor pulls PR, runs `./build.sh`, launches app
4. Trevor visually confirms against wireframes, captures screenshots
5. Screenshots uploaded to PR for record

Future: Set up a self-hosted Mac worker for automated screenshot capture.

---

## 5. Implementation PRs (Ordered)

Each PR is independently reviewable and leaves the app functional.

### PR 1: Fixture infrastructure
- Add `fixtures/api/*.json` for all rig states
- Add `MockRigService` that returns parsed fixtures instead of HTTP
- Add launch argument `--mock <fixture-name>` to `LabCommandApp`
- Extend `LabCommandCoreTestsRunner` with decoder round-trip tests
- **Verification**: `swift run LabCommandCoreTestsRunner` passes; app launches with `--mock both-sensors-live`

### PR 2: Remove dead code
- Delete `ModuleSlot.swift` (unused berth scaffolding)
- Delete `LabMetricsLoader.swift` (unused module metrics)
- Remove any `LabMetrics` references from `AppState`
- **Verification**: App builds; tests pass; no runtime errors

### PR 3: Merge ResearchView into Takes
- Rename `SessionLibraryView` → include trajectory card inline
- Delete `ResearchView.swift` (was thin wrapper)
- Update `CommandCenterView` switch to remove `.research` case pointing at wrapper
- **Verification**: Takes tab shows trajectory + session list

### PR 4: Simplify top bar to status chips
- Top bar: app title, minimal nav, sensor pips, Spark pip, refresh
- Remove verbose sublines; keep chips tappable for drill-down
- Sensor chip: 6px pip + label, color by state
- **Verification**: Top bar matches wireframe; chips reflect fixture state

### PR 5: Promote pipeline graph to main canvas
- Move `PipelineGraph` to top of RigView (after status headline)
- Keep audit strip attached below
- Remove redundant stage rail disclosure (pipeline graph replaces it)
- **Verification**: Signal chain visible without scrolling; audit strip shows per-hop health

### PR 6: Consolidate readout strip
- Single `HairlineRow` under pipeline: Mid Hz, JT Hz, Spark load, Disk
- Remove separate sparkline cards; inline as strip
- **Verification**: Readout strip renders 4 cells with sparklines

### PR 7: Collapse event log
- Event log as collapsible section, default collapsed
- Show last event in collapse header (one-line preview)
- Expand to see recent 8 events
- **Verification**: Log collapsed by default; expands on click

### PR 8: Streamline Hardware page
- Device cards: Spark, Tailscale, Mid-360, JT128 (4 cards)
- Container strip below
- Clocks section with offset visualization
- Remove full dossiers (move to tooltip or "Show specs" disclosure)
- **Verification**: Hardware page loads in < 2s; all cards render

### PR 9: Clean up Settings
- Remove redundant static rows
- Group: Host, Probe interval, Keyboard shortcuts
- Remove compile-time constants display (redundant)
- **Verification**: Settings page minimal; controls functional

### PR 10: Visual polish pass
- Verify all colors match DESIGN-VISION.md hex values
- Verify spacing tokens (pad=20, sectionGap=20, radius=7)
- Verify typography (titleSize=15 cap, body=13, mono=11)
- Run design contract tests
- **Verification**: `testSpacingTokensMatchContract` passes; `testNoTypeScaleExceedsChromeCap` passes

### PR 11: Documentation update
- Update README.md with new nav structure
- Add `docs/MOCK-API.md` explaining fixture usage
- Update DESIGN-VISION.md if any amendments
- **Verification**: README accurate; mock docs complete

---

## Summary

The app is already well-structured with good separation between core logic (`LabCommandCore`) and UI (`LabCommand`). The redesign focuses on:

1. **Cutting dead code** — Remove `ModuleSlot.swift`, `LabMetricsLoader.swift`
2. **Focusing the UI** — Promote pipeline graph as main visual; collapse secondary chrome
3. **Maintaining the contract** — Design-refs/DESIGN-VISION.md governs palette, typography, spacing
4. **Enabling verification** — Mock fixtures let agents test decoder logic without hardware; Swift builds verified locally or on self-hosted Mac

The 11 PRs can be implemented in order, each leaving the app functional. Trevor's local verification with `./build.sh` and screenshot capture provides the visual QA loop until a self-hosted Mac worker is available.
