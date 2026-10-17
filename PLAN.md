# Change overlay + timeline scrubber

Mac viewer for one occupancy diff. Snapshot A against snapshot B, per sensor, through the existing `ChangeDetection` API. Appeared, disappeared, and moved regions draw on the room cloud. A scrubber reveals those regions in report order. No rig contact on this path.

`ChangeDetection.swift` stays frozen. This plan does not change its math, its JSON, its defaults, or its tests.

## What the code is today

`ChangeDetection` (`Sources/LabCommandCore/ChangeDetection.swift`) is a pure occupancy diff.

- `ChangeDetection.compare(_:_:config:)` builds two `OccupancyMap`s and returns a `Report`.
- `Report.regions` is `[Region]` in v2 order: kind rank `moved`, `appeared`, `disappeared`, `unobserved`, then `voxelCount` descending, then `minKey`, then `fromMinKey`.
- A `Region` carries `kind`, `voxelCount`, `centroid`, `boundsMin` (inclusive cell corner, meters), `boundsMax` (exclusive corner, meters), `volume`, `minKey`. For `.moved` only, the disappeared partner is `fromCentroid`, `fromBoundsMin`, `fromBoundsMax`, `fromVoxelCount`, `fromMinKey`.
- `Report.appearedVoxels`, `disappearedVoxels`, and `unknownVoxels` are pre-cut counts. They are not key sets.
- `Region` does not store the occupied keys. `minKey` is one key. The drawable geometry the API actually gives is the AABB, the centroid, and, for `.moved`, the from-AABB and from-centroid.
- `Config()` is voxel 0.05 m, `tolerance` 1, `minVoxels` 6, `minExtent` 2, `pairMoves` true, `maxMove` 1.5 m, `evidenceOfAbsence` false, `origin` `(0,0,0)`, `rayStride` 1, `reportUnobserved` false.

`docs/CHANGE-DETECTION.md` measured the files `shot.sh` launches with (`~/Developer/lc-tasks/snapshots/{a,b}/{jt128,mid360}.bin`, same bytes as `Tests/Fixtures/room/snapshots/`):

- Evidence on and `Config()` cuts (`minVoxels` 6, `minExtent` 2): both sensors return `regions` empty. The goldens are `Tests/Fixtures/change-detection/{jt128,mid360}.json`.
- Evidence on, `minVoxels` 1, `minExtent` 1, pairing on: JT128 has 50 regions, Mid-360 has 272. Pre-cut voxels are JT128 46 / 14 / 854 and Mid-360 255 / 104 / 1286 (appeared / disappeared / unknown). The largest Mid-360 component is one appeared sheet of 7 voxels, 5×3×1 cells. Most regions are one voxel.

A viewer that calls `compare` with `Config()` will show a room and zero boxes on the real pair. The boxes have to come from a call-site `Config`. That is not a library change.

The room is already drawn by `PointCloudRenderer` (`MTKViewDelegate`). Points are `PointVertex` (position, intensity, sensor id), one inline Metal library, primitive `.point`. The camera is `CameraMath.fitToBounds` (FOV `PointCloudRenderer.fovY` = 45°, elevation `CameraMath.defaultElevation` = 30°, margin 0.9) inside a Z-up `lookAt`. Nothing in the app calls `setViewMatrix` from a drag. The fitted camera is the camera the overlay has to match. `DualPointCloudView` feeds that renderer. `LivePointCloudCanvas` in `CommandCenterView` is the SwiftUI slot.

Clouds are decoded by `PointCloud2CDR.decode` (CDR, 4-byte encapsulation header). `PointCloudFrame.timestamp` is the header stamp. `PointCloudFrame.sensor` is not set by the decoder. `RigTransforms.inWorld(_:sensor:)` writes `world`: JT128 is identity (`hesaiToWorld`), Mid-360 is `livoxToWorld` (translation `(0, 0, -0.110)`, yaw −86.5°, roll π). The live client already does this before packing. The sample-PCD path does not. Two sensor-frame clouds overlaid without `inWorld` are not one room. Mid-360 against JT128 without that transform is the meaningless diff the change-detection doc warns about.

`RoomMemory.ChangeEvent` is a different type (lifecycle state, ROS header, `SIMD3` boxes). The scrubber does not read it. There is no mirrored event log on this launch, only two files per sensor.

`AppState.start()` calls `refresh()`, and `ConnectivityService.runAllChecks()` runs `tailscale status`, `ssh`, and ping. `FoxgloveLiveClient` opens nothing until Snapshot or Live, but those buttons call `takeSnapshot()` / `startLive()`, which open `ws://` to the Spark. `shot.sh` waits 20 s and captures the layer-0 window of `build/LabCommand.app`. A launch that still probes will SSH during the shot.

## Architecture

One new core type family, `ChangeOverlay`, in `LabCommandCore`. Foundation and `Point3D` / `RigidTransform` / `SensorID` / `ChangeDetection` only. No simd, no Metal, no SwiftUI. The app binds.

```text
LC_SNAPSHOT_A / LC_SNAPSHOT_B
        │  app reads jt128.bin + mid360.bin (PointCloud2CDR.decode)
        ▼
per sensor, sensor frame:
  ChangeDetection.compare(before, after, config: ChangeOverlay.snapshotConfig())
        │  Report
        ▼
ChangeOverlay.timeline(sensor:report:beforeStamp:afterStamp:)
ChangeOverlay.merge  — earlier after-stamp first, then SensorID.allCases
        │  Timeline
        ▼
scrubber state (revealed count + kind flags) lives in AppState
ChangeOverlay.revealed + ChangeOverlay.geometry
        │  world-frame boxes and arrow centerlines
        ▼
PointCloudRenderer second pipeline, same MVP as the points
SwiftUI scrubber under the canvas, copy from ChangeOverlay
```

Diff in the sensor frame, then transform for display. `RigTransforms.inWorld` on the points and `RigidTransform.apply` on the box corners and centroids use the same `RigTransforms.toWorld(sensor)`. Quantizing after the Mid-360 rotation would cross voxel faces and would not be the diff the goldens describe. The CLI does not transform. The viewer does not either, until the geometry is on its way to the screen.

The cloud on screen is snapshot B, both sensors, in `world`. Snapshot A is the before argument of `compare` and is not drawn. Drawing both clouds would z-fight the unchanged room. The scrubber does not edit points. It only reveals regions. At revealed-count 0 the B cloud is on screen with no boxes. That is "before any region is drawn," not a reconstruction of cloud A.

## Viewer config

`ChangeOverlay.snapshotConfig()` returns `ChangeDetection.Config()` with three fields set:

| Field | Value | Why |
|---|---|---|
| `evidenceOfAbsence` | `true` | A disappeared cell is one an after-ray traversed. Matches the measured blip table. Origin stays `(0,0,0)` because the diff is in the sensor frame. |
| `minVoxels` | `1` | The library default 6 drops every component on this pair. |
| `minExtent` | `1` | The library default 2 drops the only component that clears a lower count, the 7-voxel Mid-360 sheet. |

`pairMoves`, `tolerance`, `voxelSize`, `maxMove`, `moveCountRatio`, `rayStride`, and `reportUnobserved` stay at `Config()` (pairing on, 0.05 m, stride 1, unobserved regions off). `reportUnobserved == false` drops unknown cells from `regions` no matter how large the component is. That is not the cluster cut. The pre-cut `unknownVoxels` count is still shown on every caption, including a playhead. Unobserved boxes are not built unless a report actually contains `.unobserved`.

`rejected` voxels are a different number. They are the pre-cut appeared and disappeared voxels that did not land in a region after `minVoxels` and `minExtent`. A moved region's `voxelCount` counts on the appeared side and `fromVoxelCount` on the disappeared side. Unknown voxels are not added into that sum. Captions print the two counts as separate clauses.

This config is a viewer choice. `ChangeDetection.Config()` and the goldens stay on the library defaults. Tests lock the three overrides and no others.

On this pair the result is on the order of 50 + 272 regions, mostly one voxel. The overlay draws what `compare` returns, including a `.moved` region the pairing heuristic emits for two similar specks. The viewer does not re-gate moves.

## Overlay geometry

`ChangeOverlay.geometry` takes the revealed events, a `RigidTransform` per `SensorID`, and the emphasized event index.

For each event, eight corners are the combinations of `boundsMin` / `boundsMax` on x, y, z, in sensor meters. Each corner goes through `transform.apply`. That is an oriented box. For Mid-360 it is not axis-aligned in world, and it is not replaced by a world AABB. A world AABB of the rotated corners would be a looser box than the voxel.

A one-voxel region has an edge of `voxelSize` (0.05 m). A larger region’s box is the component AABB, so it can contain empty cells. That is what `boundsMin` / `boundsMax` mean. The occupied key set is not recoverable from `minKey` plus the AABB, and this plan does not add keys to `Region`.

`.moved` also emits the from-box from `fromBoundsMin` / `fromBoundsMax`, and an arrow from `fromCentroid` to `centroid`, both transformed the same way. A `.moved` region missing either from-corner or `fromCentroid` contributes the appeared box only. `.appeared` and `.disappeared` emit one box and no arrow. `.unobserved` emits edges only.

Arrow head, in meters, in the plane of the segment and world up `(0,0,1)`:

- `len` is the distance from the transformed `from` to the transformed `to`. `len == 0` emits no arrow.
- Head length `h = min(0.15, len * 0.35)`.
- Side vector is `normalize(cross(dir, up))`. If that length is under `1e-4` (a vertical move), use `cross(dir, (0,1,0))` instead.
- Wings sit at `to - dir * h`, offset by `± side * (h * 0.45)`.

Core returns the centerline and the two wing points. It does not camera-face the head and it does not know the viewport.

Colors live in core as linear 8-bit RGB so the legend and the GPU share one table. Values are the `LabTheme` status colors, not a new palette:

| Kind | Hex | Role |
|---|---|---|
| `appeared` | `#3FA37A` | `LabTheme.good` |
| `disappeared` | `#C45A5A` | `LabTheme.bad` |
| `moved` | `#C4A04A` | `LabTheme.warn` |
| `unobserved` | `#5C6670` | `LabTheme.faint` |

The from-box of a move uses the moved color and is edges only. The appeared box of a move is a fill. Emphasized means the playhead event: fill alpha 0.85 instead of 0.28. Every other revealed box stays at 0.28.

## Metal for the room, SwiftUI for the scrubber

Boxes and arrows go in Metal. The scrubber goes in SwiftUI.

A SwiftUI `Canvas` over the `MTKView` cannot share the draw. `viewMatrix` and `projectionMatrix` are private on `PointCloudRenderer`. `draw(in:)` does not invalidate SwiftUI. The fit runs when the drawable size arrives, which is after the first SwiftUI update, so a 2D projection done in `updateNSView` would use the default eye `(0, -5, 3)` or a stale fit. SwiftUI shapes also have no depth against the points, so every box would paint on top of the room.

The point pipeline stays as it is. One more pair of functions in the same `shaderSource` string (`overlayVertex`, `overlayFragment`), one more `MTLRenderPipelineState` from the same `makeLibrary`. Vertex is `float3 position` plus `float3 color` plus a `float` alpha in a uniform, or alpha in the vertex. The overlay draw uses the same uniform buffer the points already bind at vertex index 1 (`modelViewProjection` only). It runs in the same command encoder, after the points:

1. Triangle list, 12 triangles per filled box, alpha blending on (`sourceAlpha` / `oneMinusSourceAlpha`). Depth test on, depth write off, so a box hides behind a wall and does not punch a hole in the cloud.
2. Line list, 12 edges per box, plus the three arrow segments (`from → to`, `to → wing`, `to → wing`). Line width on this GPU is 1. The fill is what makes a voxel read as a color.
3. Point list at the appeared centroid and, for a move, the from-centroid. `point_size` 10. The existing point shader colors by height or sensor, so these marks use the overlay pipeline, in the kind color. A 0.05 m cube at a room fit is only a few pixels (45° vertical FOV, room on the order of several meters, canvas well under the 1180×800 window). A 10 px centroid mark is what stays visible in the screenshot. The fill still carries the box. Fills and lines keep the depth test (`less`, write off). The centroid points compare depth as `always` and do not write depth, so a return inside the same voxel cannot hide the mark. Boxes and arrows still drop out behind a wall.

No instanced voxel brick renderer. A few hundred boxes are about 10⁴ triangle vertices. The point cloud is already ~1.4×10⁵ points (JT128 115,200 plus Mid-360 ~20,000). Sorting boxes by centroid depth is unnecessary at this count. Overlapping translucent boxes can composite in the wrong order. These regions are small and sparse. That artifact is accepted.

`PointCloudRenderer` gains `setOverlay(_: ChangeOverlay.Geometry)`. `DualPointCloudView` passes it from `updateNSView`. The renderer keeps the last geometry and rebuilds the three buffers only when it changes. An empty geometry draws nothing and does not affect the point path. Camera fit stays on the point bounds (`RobustBounds.percentile`, `maxFitRange` 15 m). Overlay corners do not pull the camera.

In snapshot mode the point color mode is `.sensor` (Mid-360 cyan, JT128 magenta in the existing shader). Height mode is a rainbow that includes green, red, and yellow and would swallow the kind colors. Sensor coloring leaves good / bad / warn for the overlay.

## Scrubber

The timeline is discrete. Every region of one sensor shares that sensor’s after-frame `PointCloudFrame.timestamp`. There are two stamps per sensor (A and B), not a stamp per region. The playhead is an event index.

`ChangeOverlay.Filter` holds `showAppeared`, `showDisappeared`, `showMoved`, `showUnobserved`. The first three default true. Unobserved defaults false.

`ChangeOverlay.revealed(timeline:filter:count:)` walks events in timeline order, keeps those whose kind is enabled, and returns the first `count` of that filtered list. `count` is `0...filteredCount`. `0` draws no boxes. The default `count` is `filteredCount`, so the first paint and the screenshot show every enabled region. The playhead event is the last revealed event, and it is the emphasized one. When `count` is 0 there is no playhead.

Turning a kind off rebuilds the filtered list and clamps `count` to the new length. Indices in `Timeline.events` do not renumber. Chips show the full-timeline count for that kind, not the revealed prefix.

SwiftUI, docked under the canvas inside the existing `pointCloudCanvas` `VStack` in `CommandCenterView`. The scrubber is shown only in snapshot mode, including a configuration error and a load failure.

- A `Slider` whose state comes from `ChangeOverlay.slider`. When `filteredCount > 0` the range is `0...filteredCount`, the control is enabled, and `count == 0` is a real position on that range (nothing drawn yet, the thumb at the start). When `filteredCount == 0` the control is disabled, the value is 0, and the bound range is `0...1` so SwiftUI still has a non-empty range. That disabled state covers an empty timeline and a nonempty timeline whose filter matches nothing.
- End labels are the before stamp and the after stamp, formatted by `ChangeOverlay.clock` as `HH:mm:ss` with an explicit time zone (the view passes `.current`; tests pass a fixed zone). Missing stamps read `A` and `B`.
- Three kind chips, colored with the core RGB, each `kind · count`. An unobserved chip appears only when the timeline contains one. Chip counts are the full timeline, not the revealed prefix.
- One monospaced caption from `ChangeOverlay.caption`. Counts use a singular noun at 1. The gap is `String(format:)` with the POSIX locale, two fractional digits. The word unavailable is not involved. No Hz. Unknown voxels are never described as rejected or as below the cluster cut.

Caption states, each asserted:

| State | Caption |
|---|---|
| Playhead (`count > 0`) | `JT128 · moved · 7 voxels · 1.00 m · 854 unknown voxels · 12 rejected voxels`. The gap is the sensor-frame from-centroid to centroid distance and is omitted when the kind is not `.moved` or `fromCentroid` is missing. Unknown and rejected are always on this line. |
| Empty timeline (no regions) | `0 regions · 46 appeared voxels, 14 disappeared voxels, 854 unknown voxels, 60 rejected voxels`. The three pre-cut totals plus rejected. |
| Nonempty timeline, `count == 0`, filter still matches events | `0 of 12 regions · 854 unknown voxels, 3 rejected voxels`. |
| Nonempty timeline, filter matches nothing | `0 of 12 regions · all kinds hidden · 854 unknown voxels, 3 rejected voxels`. |

Before the load returns, the caption is `ChangeOverlay.readingLine` (`Reading snapshots.`). A configuration error or a load failure replaces that with its own sentence. Those two are not scrubber states of a timeline. The slider stays in the disabled empty state until a timeline arrives.

No autoplay. A timer would make the screenshot depend on the 20 s sleep. Left and right arrow keys step `count` by one when the window is key and snapshot mode is on. The local monitor consumes the event so a focused slider does not step twice. Command, option, and control pass through. A text view keeps the keys. They are not menu shortcuts. Cmd-1 and Cmd-2 stay the destination shortcuts.

The caption, the chip labels, the empty-state sentences, `readingLine`, `localSessionLine`, and `localChip` are core strings. The views print them.

## Snapshot launch, no network

`LaunchMode.resolve` reads the environment.

| Environment | Mode |
|---|---|
| Neither `LC_SNAPSHOT_A` nor `LC_SNAPSHOT_B` is present | `.rig`. Today’s launch. Probes, timers, polls, actions, and sockets stay available. |
| Both present and both non-empty after trimming whitespace | `.snapshots(directoryA:directoryB:)`. Local load. No contact. |
| Anything else: one variable, a missing partner, or a present value that trims to empty | `.configurationError`. A local sentence names the bad side (`is not set`, `is empty`, or both). No load and no contact. This does not fall through to the rig launch. |

`ChangeOverlay.snapshotDirs` returns the two paths only for `.snapshots`. It returns nil for `.rig` and for `.configurationError`. Nil is not permission to probe. The app branches on `LaunchMode`, not on that optional.

Isolation does not stop at `start()`. `SnapshotSession` is the boundary `AppState` calls:

- `begin(rig:)` runs the rig closure only for `.rig`. Snapshot mode returns the two directories and does not call it. A configuration error returns the sentence and does not call it or the loader.
- `refresh`, `restartTimer`, `pollRig`, and `runAction` take the network work as a closure. `ContactGate.attempt` throws `ContactForbidden` before that closure when `isolates` is true, so the closure does not run. `AppState.refresh`, `AppState.restartTimer`, `AppState.pollRig`, and `AppState.runAction` (every action, including `stopView`) are that call. `restartTimer` may invalidate a local `Timer` first. It does not schedule one.
- Tunnel start is the other entry. `RigService.ensureTunnels` and `ensureAPITunnel` call `TunnelGate`, which holds a lock across the held-port check, host discovery, and spawn. `startTunnel` is the ssh spawn that factory calls. An isolated `TunnelGate` returns before discovery and before `startTunnel`. Status GET, action POST, the localhost port probe, `stopTunnels`, connectivity probes, and the Foxglove socket stay on the rig path. Snapshot mode does not reach them: those calls sit inside the closures above, and the live client is created with `isAvailable: false`.

The same flag disables the controls that would reach those methods. Disabled is not the fence. The fence is the gate. The controls are:

- The Refresh button in `CommandCenterView`, and the View menu Refresh item (`validateMenuItem` returns false).
- Settings auto-refresh and the interval picker. Both are disabled, and `autoRefreshEnabled` starts false so the row does not look armed.
- Recording, arm-at-lock, and the status-item Start / Record / Stop items. `runAction` does not consult `RigGate`, so the gate refuses even if a control were enabled. `RigGate` on an unprobed rig still disables the deck. Diagnose stays. It only reads the local snapshot.
- Live Snapshot and Live are not shown.

In snapshot mode the status item says `Local` for health and does not add `Spark offline`. The workspace Spark chip binds to `ChangeOverlay.localChip` (`Local`) in `LabTheme.dim`. The headline binds to `ChangeOverlay.localSessionLine` (`Local snapshots. No rig contact.`) after a good load, and to the configuration or load sentence when that is the result. `RigHealth` is not modified.

`LC_SCREENSHOT_MODE` is set by `shot.sh` and is not read. The two directory variables are the switch. The shot and a hand launch with the same variables behave the same. A partial pair is a configuration error on both.

Load on a detached task, publish on the main actor, so `applicationDidFinishLaunching` still orders the window front before the decode finishes. `shot.sh` sleeps 20 s. A release `compare` of these frames sits far under that. The indoor evidence bar in the change-detection doc is under 100 ms for ~89k points. These files are 115,200 and ~20,000 points per side, two sensors, release, inside the same budget with room to spare. The window title is `LabCommand - Rig`. One layer-0 window, the one `LabCommandApp` already creates at 1180×800. The process prints `localSessionLine` to stderr as soon as the isolated launch is chosen, and prints the caption when the load finishes. That line is a label in the shot log. It is not the isolation proof.

Per sensor the loader reads `<dir>/jt128.bin` and `<dir>/mid360.bin`, decodes with `PointCloud2CDR.decode`, and sets `frame.sensor`. A missing file, a `PointCloudError`, or a compare failure publishes `ChangeOverlay.loadFailure` text (`Could not load Mid-360 mid360.bin (missing file).`) into the caption and the headline, and draws no cloud. The window still opens. Sensors are tried in `SensorID.allCases` order, so a pair of missing files reports Mid-360 first. The real directories contain all four files. A load failure stays isolated: refresh, timers, polling, and actions still do not contact.

After a successful compare, `AppState` holds the two world-frame B frames and the merged `Timeline`. `DualPointCloudView` gets those frames with `liveFeed: nil` and `fitGeneration: 1`. The sample PCD is not loaded, so it cannot sit under the room.

The left column still evaluates `RigHealth` on the initial `RigSnapshot.offline(reason:)`. That headline is `Spark is unreachable` unless `blockReason` is set, and the reasons still say `No SSH path to the Spark`. In snapshot mode the workspace header binds to `ChangeOverlay.localSessionLine` instead of `RigHealth.evaluate`, and the top Spark chip binds to `ChangeOverlay.localChip` (`Local`) in `LabTheme.dim`. `RigHealth` is not modified. The chip must not say unreachable for a launch that never probed.

`localSessionLine` is `Local snapshots. No rig contact.` The canvas is the B cloud, the overlay, and the scrubber. The recording deck stays visible and disabled so this mode does not invent a second shell.

## Files

New:

| Path | Role |
|---|---|
| `Sources/LabCommandCore/ChangeOverlay.swift` | `snapshotDirs`, `snapshotConfig`, `Timeline`, `Event`, `Filter`, `revealed`, `geometry`, colors, caption, slider, clock, local-session copy |
| `Sources/LabCommandCore/SnapshotSession.swift` | `LaunchMode`, `ContactGate`, `SnapshotSession`, `SnapshotFiles` |
| `Sources/LabCommandCoreTestsRunner/ChangeOverlayTests.swift` | Groups below, including the contact regression |
| `Sources/LabCommand/ChangeScrubberView.swift` | Slider, chips, caption |

Edit:

| Path | Change |
|---|---|
| `Sources/LabCommandCoreTestsRunner/main.swift` | Call `runChangeOverlayTests()` after `runChangeDetectionTests()` |
| `Sources/LabCommand/PointCloudRenderer.swift` | Overlay pipeline, `setOverlay`, draw after the points |
| `Sources/LabCommand/PointCloudView.swift` | `DualPointCloudView` takes a `ChangeOverlay.Geometry` and forwards it |
| `Sources/LabCommand/AppState.swift` | Env switch, background load, published timeline and filter. `start`, `refresh`, `restartTimer`, `pollRig`, and `runAction` go through `SnapshotSession` |
| `Sources/LabCommand/CommandCenterView.swift` | Scrubber under the canvas. Snapshot mode hides live controls, disables Refresh and the recording deck, swaps the Spark chip and the rig headline |
| `Sources/LabCommand/LabCommandApp.swift` | Menu and status-item actions stay disabled in snapshot mode. Arrow keys step the playhead |
| `Sources/LabCommand/SettingsView.swift` | Auto-refresh and the interval picker disabled in snapshot mode |
| `Sources/LabCommand/RigService.swift` | `ensureTunnels` and `ensureAPITunnel` enter locked `TunnelGate`. `startTunnel` still spawns ssh. |
| `Sources/LabCommand/MockRigService.swift` | Passes the snapshot flag through to `RigService` |

Do not edit: `ChangeDetection.swift`, `ChangeDetect/main.swift`, `Package.swift` (both targets compile every `.swift` file in their directory), `RoomMemory*`, `RigHealth.swift`, `Info.plist`, `build.sh`.

`Package.swift` products stay `LabCommand`, `change-detect`, `LabCommandCore`, `LabCommandCoreTestsRunner`.

## Tests

`ChangeOverlayTests.swift` follows the runner style: `expect`, `passed()`, first failure exits 1. No XCTest. No network. No read of `~/Developer/lc-tasks`. The heavy snapshot goldens already live in `ChangeDetectionTests` and are not repeated.

1. **Config.** `snapshotConfig()` equals `Config()` except `evidenceOfAbsence`, `minVoxels`, and `minExtent`. `compare` on an empty-enough cloud with that config does not throw `invalidConfig`.
2. **One moved region through the viewer config.** Two single-voxel clusters in sensor frame, about 1 m apart along +x, the after voxel farther from the origin so the evidence ray traverses the before voxel. `compare` with `snapshotConfig()` returns one `.moved`. `timeline` → `revealed` at full count → `geometry` with `RigidTransform.identity` returns two boxes and one arrow. The arrow’s `from` and `to` match the transformed centroids.
3. **Order.** A report’s regions stay in the order given. `merge` places the earlier after-stamp first. Equal stamps break ties by `SensorID.allCases` (`.mid360`, then `.jt128`).
4. **Filter.** `count == 0` reveals nothing. `count == 2` reveals the first two enabled events. Clearing `showAppeared` drops appeared events and keeps the relative order of the rest. `showUnobserved` defaults false and hides an unobserved event that is in the timeline.
5. **Box corners.** Identity transform. The eight corners are exactly the eight combinations of `boundsMin` and `boundsMax`. Twelve edges connect them.
6. **Rigid transform.** One corner of a Mid-360 event transformed by `RigTransforms.toWorld(.mid360)` equals `livoxToWorld.apply` of that corner. JT128’s transform is `hesaiToWorld`, which is identity.
7. **Arrow.** Head length is `min(0.15, 0.35 * len)`. Both wings are closer to `to` than to `from`. A `.moved` region with `fromCentroid == centroid` emits no arrow. An appeared region emits no arrow. A `.moved` region with `fromBoundsMin == nil` emits one box and no arrow.
8. **Colors.** Appeared is `(63, 163, 122)`, disappeared `(196, 90, 90)`, moved `(196, 160, 74)`, unobserved `(92, 102, 112)`, each channel as `Float / 255`.
9. **Copy.** One assertion per caption state in the table above. The playhead line contains the sensor label, `moved`, the voxel count, the gap, the unknown total, and the rejected total. The empty line contains `0 regions`, the three pre-cut counts, and rejected, and does not contain `cluster`. Count zero on a nonempty timeline and the all-hidden filter each have their sentence, and each names unknown and rejected separately. `1 voxel` stays singular. `localSessionLine`, `localChip`, and `readingLine` match the sentences above. `snapshotDirs` returns both paths only when both variables are non-empty. A partial pair, including a whitespace value, is a configuration error and not nil-as-rig.
10. **Clamp and slider.** A filter that removes events clamps a stale `count` through `ChangeOverlay.clampedCount`. `filteredCount == 0` is the disabled slider (`enabled == false`, value 0, range `0...1`). `filteredCount > 0` is enabled, including when `count` is 0.
11. **No contact.** A `ContactGate` regression, not a log scrape. Factories record a call and then throw. `runProcess` and `openSocket` with `isolates == true` throw `ContactForbidden` and do not enter the factory. The same factories do run when `isolates` is false, so a deleted call fails the test too. `SnapshotSession.begin` on a partial environment does not call the rig closure or the loader, and the sentence names the missing or empty variable. `begin` on two directories does not call the rig closure. `SnapshotFiles.load` on a directory with no bins returns `Could not load Mid-360 mid360.bin (missing file).` and does not throw a contact error. After that failure, and on a ready snapshot mode, `refresh`, `restartTimer`, `pollRig`, and `runAction` (start rig, record, stop, wake, standby, arm, disarm, view, stop view) do not enter their work closures. `.rig` does enter them. The core geometry tests do not stand in for this group.

## Verification

1. `./build.sh --no-install` from the repo root. Release `swift build -c release --product LabCommand`, bundle at `build/LabCommand.app`. The script’s install path is `~/Applications/LabCommand.app`. `--no-install` skips it. That install path is never written, copied, or deleted. The build introduces no new warnings in the files this change touches. Existing warnings in untouched files stay.
2. `swift run LabCommandCoreTestsRunner` prints `PASS LabCommandCoreTestsRunner` and exits 0.
3. `~/Developer/lc-tasks/ws/shot.sh <this-worktree> /tmp/overlay-self.png`. The script runs `build/LabCommand.app` with `LC_SNAPSHOT_A` and `LC_SNAPSHOT_B` pointed at `~/Developer/lc-tasks/snapshots/{a,b}` and `LC_SCREENSHOT_MODE=1`, sleeps 20 s, captures the layer-0 window, and `kill`s that PID. The PNG should show the sensor-colored room, green / red / amber boxes and centroid marks, a move arrow where the report has a `.moved` region, and the slider at the far end with kind counts. `/tmp/lc-shot-app.log` should contain `Local snapshots. No rig contact.` because the process prints that label. A quiet log is not the isolation proof. `ConnectivityService` collects subprocess output on private pipes, so a missing `ssh` string would not show that a probe ran. The proof is test group 11.

No commit and no push.

## Risks

- **The real pair is a field of specks.** At the viewer config the documented counts are 50 and 272 regions, almost all one voxel, and pairing may label nearby dropouts as `.moved`. The screenshot will look speckled because that is the diff. The caption and the chip counts are what tell a reviewer it is the report. Hiding short moves would be a second detector. This plan does not.
- **Defaults still draw nothing.** Anyone calling `compare` with `Config()` on these files gets an empty `regions` array and a scrubber parked on `0 regions` plus the pre-cut sentence. The library stays that way on purpose. `minExtent` 2 exists to drop the one-cell sheet.
- **Evidence cost.** `snapshotConfig` casts one ray per occupied after voxel (`rayStride` 1). That is the setting the blip table was measured with. Raising the stride would change which cells count as disappeared. The 20 s shot budget covers a release compare. The load stays off the main actor so the window exists even if a debug build is slower. `shot.sh` runs the release bundle.
- **1 px lines.** Metal line width will not make a 5 cm cube obvious under a room fit. The translucent fill plus the 10 px centroid point are the visible mark. A moved arrow shorter than a few centimeters is a short segment between two point marks. The head is still in the geometry.
- **Alpha order.** Unsorted blended boxes can overlap wrong. Accepted at this region count.
- **Oriented Mid-360 boxes.** Corners are transformed as points. The box matches the transformed cloud. It will not line up with world axes. An axis-aligned world box would be the wrong shape.
- **AABB is not a voxel mask.** A multi-voxel region draws its bounds, including empty cells inside the box. Per-voxel cubes would need the key set, which `Region` does not have.
- **Disabled recording controls.** Snapshot mode leaves the deck on screen so the shell does not fork. The controls are disabled because the session is isolated, and `RigGate` would also refuse an unprobed rig. `runAction` itself does not consult `RigGate`, so the contact gate is what refuses a caller that skips the button. The headline and the Spark chip must use the local-session copy, or the shot looks like a failed SSH probe.
- **The gate is the session boundary, plus the tunnel entry.** `refresh`, a restarted probe timer, `pollRig`, and every action can SSH or open a socket after launch. Those four go through `SnapshotSession` before any of that work. Refresh lives on the workspace button, the View menu, and the status path that a timer would call. Settings can restart the timer. A partial environment is an error, not a reason to start those paths. A new call from `applicationDidFinishLaunching` has to go through `SnapshotSession.begin`. Tunnel spawn is the other entry: `TunnelGate` locks it and returns before `startTunnel` when the launch is isolated.
- **Camera does not orbit.** The overlay is checked against the fitted view only. Adding orbit is out of scope. `setViewMatrix` already exists for a later pass. The overlay uses the same MVP, so it will follow that matrix when something sets it.
- **Stale fit.** `fitGeneration` is 1 and is not bumped when the overlay arrives. The camera fits the B cloud once. Overlay bounds must not schedule another fit.
- **Process kill.** `shot.sh` kills the app PID. A child `ssh` started by `refresh` would be the failure mode this plan avoids by not starting one. Quit is that `kill`. No `Cmd-Q` and no AppleEvent.
