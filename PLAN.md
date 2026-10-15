# Live Room view (job B)

Status: plan only. Branch `live-room-ui` in `~/Developer/labcommand-liveroom-b`, based on `origin/main` (`ba767bb`). No app code in this commit.

Implementation starts only after a Codex review of this file exists as `PLAN.review.md` in the worktree root. Apply any findings to this plan before writing code.

This job draws the room. It does not open `ws://100.118.187.99:8767`, does not implement the mirror, and does not talk to the Spark. The parallel job (`~/Developer/labcommand-liveroom-a`, branch `live-room-client`) owns the client. As of this plan, that worktree has no `Sources/LabCommandCore/LiveRoom/API.md`.

## What the screen is

A third destination, **Room** (`Dest.liveRoom`, label `Room`, shortcut `3`). Rig stays `1`, Settings stays `2`. The View menu already walks `Dest.navCases`, so Room shows up there. The Rig page, LIVE/SNAPSHOT lidar canvas, and recording controls stay as they are.

The Room page is one instrument surface, same chrome as the rest of the app (`design-refs/DESIGN-VISION.md`, `LabTheme`):

- Header: title "Room" at 15pt semibold, a 6px activity pip, and a `Fixture` chip only while the golden mock is the source.
- Metal canvas: occupied voxels in the room frame, change markers on top.
- Status bar, 11pt mono, under the canvas: epoch, coverage, `events_seq`, memory state, cell size.
- Event list on the right, about 280pt, reading the same snapshot as the canvas.
- One banner line when the snapshot carries a condition that changes what may be drawn.

Nothing on this page is inferred by `ChangeDetection`. The Spark owns changes (`docs/ROOM-MEMORY-PROTOCOL.md` §6 rule 2). Markers come from `RoomMemory.ChangeEvent` values the source already accepted.

## Boundary with the client

The view reads one value, `LiveRoomSnapshot`, and never a socket.

```swift
public struct LiveRoomSnapshot: Equatable, Sendable {
    public enum Link: Equatable, Sendable {
        case unavailable
        case live
        case stale(age: TimeInterval)   // seconds, already computed by the source
        case resyncing                   // a map is held and is not current
    }

    public var link: Link
    public var roomId: String?
    public var mapEpoch: UInt64?        // epoch of the bricks being drawn
    public var voxelSize: Float?        // meters; nil hides the cell line
    public var coverage: Float?         // 0...1; nil renders "coverage —"
    public var eventsSeq: UInt64?       // nil renders "events —"
    public var memoryState: RoomMemory.MemoryState?
    public var poseResidualM: Float?    // nil renders "guard not run"
    public var silentSensors: [String]  // measured 0 Hz only
    public var eventsIncompleteSince: RoomMemory.Time?
    public var outdatedEventIds: Set<UInt64>
    public var bricks: [RoomMemory.Brick]
    public var events: [RoomMemory.ChangeEvent]
    public var fixture: Bool
}
```

`bricks` are the applied map: absent bricks are omitted, and an omitted brick is UNKNOWN. The view does not apply deltas, check `seq`, or send `RequestSync`. A resyncing or stale link is a flag on the snapshot. `events` are the source's current list, including history. The view decides what is drawn.

Pure helpers live in `Sources/LabCommandCore/LiveRoomView.swift` (one file, no `LiveRoom/` directory — that path is reserved for the client's `API.md`):

- `LiveRoomFormat` — the status, banner, and row strings below.
- `LiveRoomVoxels` — occupied centers and unknown-brick frames.
- `LiveRoomMarkers` — boxes and arrows, including the suppression rules.
- `LiveRoomFixture` — three snapshots decoded from the hand goldens.

The app target holds `LiveRoomModel: ObservableObject` and the SwiftUI/Metal views. `AppState.start()` constructs the model; `AppState.stop()` calls `stop()` on it. The model does not start in `init`.

### When `API.md` appears

At the start of implementation, look for both of these:

- `Sources/LabCommandCore/LiveRoom/API.md` on this branch
- `~/Developer/labcommand-liveroom-a/Sources/LabCommandCore/LiveRoom/API.md`

If either exists, this snapshot bends to that document: same field meaning, the client's names in a single adapter `Sources/LabCommandCore/LiveRoomViewAdapter.swift`. Expected mapping until the doc says otherwise:

| Snapshot field | Client (expected) |
|---|---|
| `mapEpoch` | mirror epoch of the applied map |
| `coverage` | `MemoryStatus.coverageKnownFraction` |
| `eventsSeq` | `MemoryStatus.eventsSeq` (`nil` is `EVENTS_SEQ_NONE`) |
| `memoryState` | `MemoryStatus.state` |
| `bricks` | applied brick list |
| `events` | applied `ChangeEvent` list |
| `link` | client connection / STALE / resync flags |
| `eventsIncompleteSince`, `outdatedEventIds` | protocol §5 event-gap flags |

Do not copy the client's sources into this branch. If the types are not on this branch yet, keep the provisional snapshot, point the adapter's comments at the `API.md` headings, and leave the default source as the fixture. Wiring the real client is a follow-up on the merged tree, still through that one adapter. The view files stay on `LiveRoomSnapshot`.

`events_seq` is not in the Phase 0 `MemoryStatus` on this branch (no field in `RoomMemoryMessages.swift`, no bytes in `memory_status_le.hex`). It is on `origin/phase3-design` as `MemoryStatus.eventsSeq: UInt64?`, wire `EVENTS_SEQ_NONE = 2⁶⁴−1` decoding to `nil`. The status bar still has the field. The golden mock leaves it `nil`. A unit test feeds `eventsSeq: 4` and `eventsSeq: nil` directly. The view must not copy `ChangeEvent.header.seq` into `eventsSeq`.

## Status bar

One line, `LabTheme.mono(11)`, color `LabTheme.dim`. Order is fixed:

`epoch <n> · coverage <p> · events <n> · <state> · cell <cm>`

| Field | Value | Text |
|---|---|---|
| epoch | `mapEpoch` nil | `epoch —` |
| epoch | present | decimal, no grouping separator (`epoch 43`) |
| coverage | nil | `coverage —` |
| coverage | finite | one decimal percent (`coverage 75.0%`, `coverage 0.0%` for a real zero) |
| events | nil | `events —` |
| events | present | decimal (`events 4`) |
| state | nil | `—` |
| state | `.building` / `.tracking` / `.rigMoved` / `.degraded` / `.paused` | `Building` / `Tracking` / `Rig moved` / `Degraded` / `Paused` |
| cell | `voxelSize` nil | the cell clause is omitted |
| cell | 0.05 | `cell 5.0 cm` |

Percent is `coverage * 100`, one digit after the point, `en_US_POSIX`. Cell size is `voxelSize * 100` in centimeters, one digit. This is the voxel edge (protocol §6 rule 10). There is no detection-distance claim from PR 3.2, so the word is `cell`.

When `link` is `.stale(age)`, a prefix comes first: `STALE · <age>s ·` with age truncated to whole seconds. The numbers after it are the last held values, and the prefix is what keeps them from reading as live. When `link` is `.resyncing`, the prefix is `resyncing ·`. When `link` is `.unavailable`, the line is `epoch — · coverage — · events — · —` and the cell clause is omitted.

## Activity pip

A 6px circle in the Room header. Color and motion:

| Condition | Pip | Motion |
|---|---|---|
| `link == .live` and state is `.tracking`, `.building`, or `.degraded` | `LabTheme.good` | opacity 1.0 ↔ 0.55 over 1.6s |
| `link` is `.stale` or `.resyncing` | `LabTheme.warn` | still |
| state is `.rigMoved` | `LabTheme.bad` | still |
| `.unavailable`, `.paused`, or state nil | `LabTheme.idle` | still |

The breathe runs only while the source says the memory is actually running. A fixture with `memoryState == nil` stays idle even if `link` is `.live`.

## Banners

At most one line, 13pt, above the canvas. First match wins:

1. `.rigMoved` — `Rig moved: changes suspended · <residual>`. Residual is `0.12 m` style (two decimals) or `guard not run` when `poseResidualM` is nil. Color `LabTheme.bad`.
2. `.resyncing` — `resyncing`. Color `LabTheme.warn`.
3. `.degraded` — `Degraded · <sensors>` where the sensors are the `silentSensors` joined by `, `, or `Degraded` when that list is empty. Color `LabTheme.warn`.
4. `.building` — `Building reference`. Color `LabTheme.dim`.
5. `.paused` — `Paused`. Color `LabTheme.dim`.
6. `eventsIncompleteSince` set — `incomplete since <time>`. Color `LabTheme.warn`.

Time format for banners and rows is `HH:mm:ss` UTC. `RoomMemory.Time` `{0, 0}` is `time —`.

## Voxel map

`RoomMapRenderer` is a new Metal renderer used only by Room. It copies the z-up camera from `PointCloudRenderer` (`up = (0,0,1)`, `CameraMath.fitToBounds`, clear `#0B0D10`). It does not share buffers with the lidar canvas. Drag orbits (yaw around z, pitch), scroll dollies, and a double-click refits. The lidar view is left alone.

Voxel index `(ix, iy, iz)` covers `[n·s, (n+1)·s)` on each axis. Its center is `(n + 0.5) · s`. Inside brick `(i, j, k)`, local `(x, y, z)` has index `(8i+x, 8j+y, 8k+z)`, using `Brick.evidence(x:y:z:)`.

| Evidence | Draw |
|---|---|
| occupied | one point at the cell center, color `#C5CDD6`, point size 4 |
| free | nothing |
| unknown | nothing at the cell |

A brick that contains any unknown voxel also draws its 12 edges in `LabTheme.faint` (`#5C6670`), line width 1, as the hatch for that brick. A brick that is entirely known (including all-free) draws no frame. Bricks the snapshot does not contain are not framed; the coverage field is what says the rest of the room is unobserved. Occupied points use a neutral grey so they do not collide with marker green, red, and amber.

The zlib snapshot golden is the numeric pin. After decode of `map_bricks_snapshot_zlib_le.hex` (`voxel_size` 0.05, room `lab`, epoch 43):

- Brick `(0, 0, 1)` is all free (`known` all `0xFF`, `occupied` empty): 0 points, 0 frame.
- Brick `(-1, 2, -3)` has occupied bit 0 of byte 0, local `(0,0,0)`, world index `(-8, 16, -24)`, center `(-0.375, 0.825, -1.225)`.
- The same brick has occupied bit 7 of byte 63, local `(7,7,7)`, world index `(-1, 23, -17)`, center `(-0.475, 1.175, -0.825)`.
- That brick also has unknown voxels, so it contributes one 12-edge frame and exactly two points.

Fit the camera to the occupied centers when there are any, otherwise to the brick frames, once per snapshot identity. A later snapshot with the same brick bytes does not move the camera.

While `link` is `.resyncing` or `.stale`, the same points draw at 35% opacity and the banner carries the reason. An `.unavailable` link draws neither points nor frames. The canvas then shows `Room memory unavailable` in `LabTheme.dim`, 13pt, centered.

## Change markers

Colors match `ChangeOverlay.color` on `origin/change-overlay` (`0f5966b`) and the theme status colors:

| Kind | RGB | Theme |
|---|---|---|
| appeared | `(63, 163, 122) / 255` | `good` `#3FA37A` |
| disappeared | `(196, 90, 90) / 255` | `bad` `#C45A5A` |
| moved | `(196, 160, 74) / 255` | `warn` `#C4A04A` |

Do not merge `origin/change-overlay`. That branch rewires the Rig page around local snapshot diffs. Copy only the draw conventions, in `LiveRoomMarkers` and `RoomMapRenderer`:

- Twelve box edges and the twelve triangles (two per face) from `ChangeOverlay.edgePairs` and the overlay renderer's `boxTriangles`.
- Fill alpha `0.28`, emphasized fill `0.85` (`ChangeOverlay.fillAlpha`, `emphasizedFillAlpha`).
- A moved arrow from `fromCentroid` to `centroid`, head in the same construction as `ChangeOverlay.makeArrow` (read that function at implementation time and keep the head size).
- The blended triangle / line pipeline from `PointCloudRenderer` on that branch: boxes depth-test and do not write depth; centroid marks draw with depth compare always.

What gets a marker:

| Event state | On the map | In the list |
|---|---|---|
| `.candidate` | dashed AABB, no fill | yes, under Events |
| `.confirmed` | solid AABB, fill 0.28 | yes, under Events |
| `.accepted`, `.reverted`, `.expired` | no | yes, under History |

Dashes are short segments along each edge (8 per edge). Metal lines are not dashed by the API. A `.moved` confirmed or candidate event also draws the arrow. A missing `fromCentroid` on a moved event draws the box only (the codec already rejects that on the wire; the view still must not crash).

`aabb` is `RoomMemory.ChangeEvent.Box` in meters, room frame, already the world box. No sensor transform.

Suppressed entirely (list still shows the rows): `memoryState` is `.building` or `.rigMoved`, or `link` is `.resyncing`. Protocol §4.1 and §5: no change overlays while building, while the rig is moved, or while the map is not current. `.degraded` and `.paused` keep the overlays; the banner says why the room is degraded or paused. `.stale` keeps the overlays at the same 35% as the voxels, with the `STALE` prefix on the status bar.

A selected row uses fill `0.85` and draws on top. Selection is by `eventId`. Clicking a history row selects it and does not draw a box.

If `mask == nil`, the row suffix is `no mask`. The mask's occupied voxels are not a second mesh in this pass; the AABB is the marker. The mask stays on the event for the row suffix and for a later pass.

## Event list

Right column, well background, two sections.

**Events.** Candidates and confirmed, newest `header.stamp` first, then higher `header.seq`. Each row:

`● appeared · candidate · 8 voxels · evidence — · α 0.05 · Mid-360 · 12:00:10 · no mask`

- Pip uses the kind color.
- Kind words: `appeared`, `disappeared`, `moved`.
- State words: `candidate`, `confirmed`, `accepted`, `reverted`, `expired`.
- `evidence` nil → `evidence —`. A real zero is `evidence 0%` (integer percent).
- `alpha` nil → `uncalibrated`. A real value is `α 0.05` (two decimals). Never `α 0` for nil.
- Sensors: `Mid-360`, `JT128`, both as `Mid-360, JT128`, empty bitmask → `sensor —`.
- An id in `outdatedEventIds` appends `· may be outdated`.

**History.** Accepted, reverted, expired. Collapsed by default. Same row shape. Hidden when empty, so the column is not an empty inspector.

The section caption above Events counts what is on the map: `1 change · 1 candidate` from the two `lab` goldens (the moved event is confirmed, the appeared event is a candidate). Zero is `0 changes` only when the list really has no confirmed events. A nil list because the link is unavailable shows `Room memory unavailable` in the column instead of `0 changes`.

Three filter chips, mono 11, on by default: Appeared, Disappeared, Moved. A chip off hides that kind in both sections and on the map. Chip tint is the kind color when on, `LabTheme.dim` when off. This is the chip pattern from `ChangeScrubberView`, fed by `RoomMemory.ChangeEvent.Kind` rather than `ChangeDetection.Kind`.

## Fixture source

`LiveRoomFixture` decodes with `RoomMemory.*.decode(cdr:)`. Hex parsing matches `RoomMemoryCDRTests.goldenBytes` (skip `#` comments). Files, all under `Tests/Fixtures/room/golden/hand/`:

| Mode | Files | What the screen shows |
|---|---|---|
| `map` | `map_bricks_snapshot_zlib_le`, `change_event_appeared_le`, `change_event_moved_mask_le` | room `lab`, epoch 43, cell 5.0 cm, two bricks, two events. Coverage, state, and events seq are `—` because no status golden shares room `lab`. `link = .live`, `fixture = true`. |
| `status` | `memory_status_le` | room `lab-a`, epoch 42, `coverage 75.0%`, `Tracking`, `events —`, no bricks, no events. Canvas text `No map in this fixture`. `link = .live`. |
| `stale` | same as `map` | `link = .stale(age: 4)`, voxels and markers at 35%, status prefix `STALE · 4s`. |
| `unavailable` | none | empty snapshot, `link = .unavailable`, `fixture = true`. |

The delta golden (`map_bricks_delta_le`, base epoch 42, removes indices 0 and 5) is not applied. The snapshot golden is already epoch 43 with two bricks, so that delta does not chain. The mock does not contain a store.

`memory_status_le` is room `lab-a` and the map and events are room `lab`. They stay in different modes. Mixing them would show epoch 42 over epoch-43 bricks.

Default mode is `map`, so the canvas is not blank on launch. A segmented control `Map / Status / Stale / Off` sits in the header only when `snapshot.fixture` is true. Labels are the mode names above (`Off` selects `unavailable`). It is a real switch over decoded bytes.

Environment `LABCOMMAND_LIVE_ROOM=map|status|stale|unavailable` picks the initial mode. Unset means `map`. Any other value means `unavailable`.

The `Fixture` chip uses `LabTheme.dim` and the help string `Hand goldens. No rig contact.` While it is showing, the Room page is not the live rig.

## Files

New:

- `Sources/LabCommandCore/LiveRoomView.swift` — snapshot, format, voxels, markers, fixture.
- `Sources/LabCommandCoreTestsRunner/LiveRoomViewTests.swift`
- `Sources/LabCommand/LiveRoomView.swift` — SwiftUI page.
- `Sources/LabCommand/RoomMapRenderer.swift` — Metal points, brick frames, boxes, arrows.
- `Sources/LabCommand/RoomMapView.swift` — `NSViewRepresentable`.

Edited:

- `Sources/LabCommand/AppState.swift` — `Dest.liveRoom`, construct and stop `LiveRoomModel`.
- `Sources/LabCommand/CommandCenterView.swift` — `case .liveRoom: LiveRoomView()`.
- `Sources/LabCommandCoreTestsRunner/main.swift` — call the new tests.

Not edited: `PointCloudRenderer.swift`, `FoxgloveLiveClient.swift`, `RoomMemoryCDR.swift`, `RoomMemoryMessages.swift`, the `.msg` files, anything under `spark/`, `~/Applications/LabCommand.app`, `~/Developer/labcommand-spark`.

`DesignContractTests` scans `Sources/LabCommand` for `LabTheme.t(16` and larger, and for `coming soon`. Room copy stays at 15pt titles and 11–13pt body. No placeholder destination: the `switch` in `CommandCenterView` gains a real `LiveRoomView`.

## Tests

`swift run LabCommandCoreTestsRunner` from the repo root. New cases:

- Status strings for nil, coverage `0.75` → `coverage 75.0%`, coverage `0` → `coverage 0.0%`, `eventsSeq` nil and `4`, stale age `4.2` → prefix `STALE · 4s`, cell `0.05` → `cell 5.0 cm`.
- `alpha` nil → `uncalibrated`; evidence nil → `evidence —`.
- Voxel samples for the decoded zlib snapshot: exactly the two centers above, one frame, the all-free brick silent.
- Marker sets: appeared candidate is dashed and unfilled; moved confirmed is filled and has an arrow; building, rig-moved, and resyncing produce zero boxes; a moved confirmed still produces a box when the state is `.degraded`.
- Kind filter off removes that kind from the marker set.
- Fixture `map` and `status` decode the goldens and do not share a `roomId`. Fixture `map` has `eventsSeq == nil`.
- `LiveRoomFormat.banner` priority: rig-moved wins over resyncing when both are set (the fixture can construct that; the client should not).

No new golden files. No Spark, no network.

## Gate, after the code exists

1. `./build.sh --no-install` clean. That writes `build/LabCommand.app` and does not copy into `~/Applications`.
2. `swift run LabCommandCoreTestsRunner` passing.
3. Launch `build/LabCommand.app/Contents/MacOS/LabCommand` (that binary, not the installed app). Open Room. Check Map (two voxels, one brick frame, dashed candidate, solid moved box with arrow, status `epoch 43 · coverage — · events — · — · cell 5.0 cm`), Status (`epoch 42 · coverage 75.0% · events — · Tracking`, no voxels), Stale (dimmed draw, `STALE · 4s`), Off (`Room memory unavailable`, idle pip). Quit that process by its PID.
4. `codex review --base origin/main` (or `codex exec review`). Fix findings until Codex is clean.
5. Commit on `live-room-ui`. Do not push to `main`. Do not open a PR.
6. Write `DONE.md` in the worktree root: what was built, the two gate results, and the Codex outcome.

## Decisions for review

1. Room is a third destination. The Rig lidar view stays the recording surface.
2. Hand goldens are not one session, so the fixture has separate Map and Status modes, and `events_seq` stays `—` on both.
3. Unknown voxels are a wireframe of any brick that still has an unknown cell, not a point per unknown cell. Free cells are undrawn. Occupied cells are grey points.
4. The change-overlay branch contributes colors, box topology, fill alphas, and the arrow. It is not merged.
5. The client socket, the mirror, and `RequestSync` are out of this job. `LiveRoomSnapshot` is the seam, replaced by an adapter when `API.md` exists.
