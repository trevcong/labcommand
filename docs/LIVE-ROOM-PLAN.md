# Live Room view (job B)

Status: plan, revised after the Codex re-review `VERDICT: REVISE` (`~/Developer/lc-tasks/liveroom/PLAN.review2.md`). The first review (`PLAN.review.md`) stays answered in the table below. Branch `live-room-ui` in `~/Developer/labcommand-liveroom-b`. No app code in this revision commit.

The first thing an implementer produces is a running app whose Room page is bound to `LiveRoomClient` and has drawn a map and a status bar from `ws://100.118.187.99:8767`. Markers, the event list, and the offline fixture follow that screenshot. They are still part of done.

## Review disposition

| Finding | Where this plan answers it |
|---|---|
| P1 client is part of completion, not a follow-up | "Client on this branch", "Who starts the socket", slice 1, gate step 3 |
| P1 `events_seq` is `mirror.status?.eventsSeq` (golden value 12) | Status bar, fixture `status` mode, tests |
| P1 freshness and overlay suppression | Banners, activity pip, markers |
| P2 event ring vs latest state, `.incomplete(since: nil)` | Event adaptation |
| P2 voxel centers | Voxel map (the two centers below) |
| P2 fit visible markers, `mapRevision` vs camera | Voxel map, camera |
| P2 reproducible screenshots, fixture really offline | Screenshots, fixture launch |
| P2 resync with no request path | Resync line |
| P3 selected candidates stay unfilled; caption; 280pt wrap | Event list |

Re-review (`PLAN.review2.md`):

| Finding | Where this plan answers it |
|---|---|
| P1 land `65122d2` before any client file is copied | "Client on this branch" (prerequisite gate) |
| P2 canvas-rect marker pixels, then read the rows off the PNG | Screenshots, visual inspection |
| P2 standalone resync line when history is complete | Resync line, rule 3 |

## Work order

Do these in order. Slice 1 is the earliest deliverable. Do not start slice 2 until `build/screenshots/live-room-live.png` exists and its sidecar shows a `.connected` room socket.

1. **Slice 0 — codec, then the client.** Cherry-pick `65122d2` and pass its gate before any client file is added. Then land the client from `live-room-client`. Do not write a second one.
2. **Slice 1 — live map and status bar.** Room destination, `LiveRoomClient` owned by `AppState`, voxels, brick frames, status bar, activity pip, banners. Launch against `ws://100.118.187.99:8767` and save the live screenshot.
3. **Slice 2 — markers and the event list.** Suppression, event adaptation, the resync line, selection, caption, wrapping.
4. **Slice 3 — fixture mode.** Bundled goldens, no socket and no SSH, four offline screenshots with the checks below.
5. **Gate.** Build, tests, both screenshot passes, Codex, commit, `DONE.md`.

## Client on this branch

The view binds to the client. It does not keep a parallel `LiveRoomSnapshot`, and it does not ship fixture-only.

The contract is `Sources/LabCommandCore/LiveRoom/API.md` on `live-room-client` (`~/Developer/labcommand-liveroom-a`). This worktree has no `MemoryStatus.eventsSeq`. `RoomMemoryMirror` reads that field. The copy list further down does not contain the codec, so copying those files onto the current tree does not compile.

Prerequisite commits, from `live-room-client`, cherry-picked onto `live-room-ui` before any client file is added. Do not start the cherry-pick at `05eaf98`.

1. `1de0454` — `API.md`, and `Package.swift` excluding it from the target.
2. `65122d2` — the codec and the golden. This commit is the one the copy must not skip.
   - `Sources/LabCommandCore/RoomMemoryMessages.swift`: `eventsSeq: UInt64?`, and `eventsSeqNone = UInt64.max` (`EVENTS_SEQ_NONE`).
   - `Sources/LabCommandCore/RoomMemoryCDR.swift`: write and read `events_seq`. The sentinel decodes to nil. Encoding nil writes the sentinel.
   - `Sources/LabCommandCoreTestsRunner/RoomMemoryCDRTests.swift`: the hand status expects `eventsSeq == 12`.
   - `Tests/Fixtures/room/golden/hand/memory_status_le.hex`: `events_seq 12` at pos 72 (`0c 00 00 00 00 00 00 00`).
   - `scripts/spark/make_room_memory_goldens.py`: the generator sets `events_seq = 12`.
   - No client sources. No `.msg` edit. The Swift struct is what the client compiles against.

Gate, still before the copy. All three are true on this branch:

- `eventsSeq` is a stored property of `RoomMemory.MemoryStatus`.
- Decoding `memory_status_le.hex` yields `eventsSeq == 12`.
- `swift run LabCommandCoreTestsRunner` passes.

A failed gate stops the slice. The client files stay uncopied.

The client is already committed on `live-room-client` as `05eaf98`, `574feb2`, and `b7ba3c1` (tip `b7ba3c1`), on top of `65122d2`. After the gate, cherry-pick that range, or copy these paths byte for byte from that tip. Do not rewrite the socket, the mirror, or the codecs. Do not edit those files to change resync behavior.

- `Sources/LabCommandCore/LiveRoom/LiveRoomClient.swift`
- `Sources/LabCommandCore/LiveRoom/LiveRoomProtocol.swift`
- `Sources/LabCommandCore/LiveRoom/RoomMemoryMirror.swift`
- `Sources/LabCommandCore/LiveRoom/API.md` (the tip's copy; it has moved past `1de0454`)
- `Sources/LabCommandCoreTestsRunner/LiveRoomTests.swift`
- `Tests/Fixtures/room/live/`
- the `live-room-probe` argv branch and the `runLiveRoomTests()` call in that worktree's `main.swift`

`574feb2` also adds `events_seq` and `EVENTS_SEQ_NONE` to `ros/labcommand_msgs/msg/MemoryStatus.msg` (the phase3-design hunk, so the ROS definition matches the codec). That hunk may arrive with the cherry-pick. Do not edit `.msg` files by hand, and do not take any other hunk from `origin/phase3-design`.

`swift run LabCommandCoreTestsRunner` stays offline: `live-room-probe` is a separate argv and is not part of this gate.

`LiveRoomClient.defaultURL` is `ws://100.118.187.99:8767`, subprotocol `foxglove.sdk.v1`. The bridge probed on 2026-10-15 advertises `connectionGraph` only. It has no `clientPublish`. The Spark still publishes snapshots on the live map topic, so the map arrives with no request path.

## Who starts the socket

`AppState` owns the client. The view never calls `start()` or `stop()`, never builds a `URLSession`, and never writes a `RequestSync`.

```swift
let room = LiveRoomClient() // default URL, publishRequests: true (the default)
// AppState.start() calls room.start() in live mode
// AppState.stop() calls room.stop()
```

`start()` connects and reconnects until `stop()`. `stop()` closes the socket. The mirror stays and goes stale inside the client (`connectionClosed`). One client per app. The lidar `FoxgloveLiveClient` is a different socket and stays on the Rig page.

`LiveRoomLaunch` in `LabCommandCore` reads `LABCOMMAND_LIVE_ROOM`:

| Value | Source | `room.start()` | `refresh`, SSH, rig poll, Tailscale | Lidar client |
|---|---|---|---|---|
| unset | live, destination stays Rig | yes | unchanged, existing probes | unchanged |
| `live` | live, destination Room | yes | skipped | `isAvailable: false` |
| `map`, `status`, `stale`, `unavailable` | fixture of that mode | client is not constructed | skipped | `isAvailable: false` |
| anything else | fixture `unavailable` | client is not constructed | skipped | `isAvailable: false` |

`--mock` only replaces `RigService`. It does not make Room offline. Fixture mode is the env value above, not `--mock`.

The gate's live screenshot uses `LABCOMMAND_LIVE_ROOM=live`, so that process opens the room socket and does not SSH, deploy, or poll the rig. A normal launch (env unset) still runs the existing probes; that launch is not the gate command.

The screen reads `room.connection` and `room.mirror`. Helpers in `Sources/LabCommandCore/LiveRoomView.swift` are pure functions of those values plus one flag, `requestPathExists`, which `AppState` computes:

`server.canPublish` is true, or `room.requestSyncSink != nil`.

This job leaves the sink nil.

## What the screen is

A third destination, **Room** (`Dest.liveRoom`, label `Room`, shortcut `3`). Rig stays `1`, Settings stays `2`. `Dest.navCases` already feeds the View menu. The Rig page, the lidar canvas, and the recording controls stay as they are.

Same chrome as the rest of the app (`design-refs/DESIGN-VISION.md`, `LabTheme`):

- Header: title "Room" at 15pt semibold, a 6px activity pip, and a `Fixture` chip only in fixture mode.
- One banner line when a condition below matches.
- Metal canvas: occupied voxels in the room frame. Change markers on top, only while the map is current.
- Status bar, 11pt mono, under the canvas.
- Event list on the right, 280pt. Slice 1 may omit the list; the column appears in slice 2. The live screenshot does not wait for it.

Nothing on this page comes from `ChangeDetection`. Markers come from `RoomMemory.ChangeEvent` values the mirror already applied.

## Status bar

One line, `LabTheme.mono(11)`, color `LabTheme.dim`. The events field is `mirror.status?.eventsSeq`, the server high-water mark copied onto `LiveRoomStatus`. It is not `mirror.eventsSeq` (that is the contiguous baseline). It is not `ChangeEvent.header.seq`. Nil, including a wire `EVENTS_SEQ_NONE`, renders `events —`. A baseline of 4 with a status high-water mark of 12 renders `events 12`.

Order, after the prefix below:

`epoch <n> · coverage <p> · events <n> · <state> · cell <cm>`

| Field | Source | Text |
|---|---|---|
| epoch | `mirror.mapEpoch`, else `status?.mapEpoch`, else none | `epoch 43` or `epoch —`. Decimal, no grouping separator |
| coverage | `status?.coverageKnownFraction` | `coverage —`, or one decimal percent (`coverage 75.0%`, `coverage 0.0%` for a real zero) |
| events | `status?.eventsSeq` | `events —` or `events 12` |
| state | `status?.state`, and only while `mirror.isStatusFresh` | `Building` / `Tracking` / `Rig moved` / `Degraded` / `Paused`. Omitted when the status is missing or not fresh |
| cell | `mirror.voxelSize` | omitted when nil. `0.05` → `cell 5.0 cm` |

Percent is `coverage * 100`, one digit, `en_US_POSIX`. Cell size is `voxelSize * 100` centimeters, one digit (protocol §6 rule 10). The word is `cell`. There is no detection-distance claim.

Prefix, first match:

- connection is not `.connected` → `offline ·`, and no state word
- `status != nil` and `!isStatusFresh` → `STALE · <age>s ·`, and no state word. Age is whole seconds of `ProcessInfo.processInfo.systemUptime - status.receivedAt` (the mirror's clock)
- otherwise no prefix

The numbers after a prefix are the last held values. The prefix is what keeps them from reading as live.

`memory_status_le.hex` after `65122d2` is room `lab-a`, epoch 42, coverage 0.75, state tracking, `events_seq` 12. Its status line is `epoch 42 · coverage 75.0% · events 12 · Tracking` with no cell clause.

## Activity pip

A 6px circle in the header. A `.connected` socket is not enough to turn it green. First match:

| Condition | Pip | Motion |
|---|---|---|
| `isStatusFresh` and state is `.rigMoved` | `LabTheme.bad` | still |
| `.connected`, `memoryTopics` not empty, `isStatusFresh`, `isMapCurrent`, state is `.tracking`, `.building`, or `.degraded` | `LabTheme.good` | opacity 1.0 ↔ 0.55 over 1.6s |
| `.waiting`, or `.connected` and `!isMapCurrent`, or a status exists and `!isStatusFresh` | `LabTheme.warn` | still |
| anything else (`.off`, `.connecting`, topics empty, `.paused`, no status) | `LabTheme.idle` | still |

`isMapCurrent` is `mapState == .current && isStatusFresh`. The green row requires it again so a fresh status over a stale or empty map stays amber.

## Banners

At most one line, 13pt, above the canvas. This is Phase 3 §5.4, on the states the client actually has. `LiveRoomConnection` has no incompatible case; do not invent one. Connection loss and a stale status come before any remembered memory state. First match:

1. `.connected` and `memoryTopics` is empty — `Room memory not running on Spark`. Color `LabTheme.warn`. The canvas is this sentence, not an empty room drawn as current.
2. `.waiting`, or `.off` after `start()` has run — `connection lost`. Color `LabTheme.bad`. The first `.connecting` of a launch uses `connecting` (`LabTheme.dim`) instead of connection lost.
3. `status != nil` and `!isStatusFresh` — `STALE · <age>s`. Color `LabTheme.warn`. Age as on the status bar. Do not show `Rig moved`, `Tracking`, or any other last state here.
4. Fresh `.rigMoved` — `Rig moved: changes suspended · <residual>`. Residual is `0.12 m` (two decimals) or `guard not run` when `poseResidualM` is nil. Color `LabTheme.bad`.
5. `mapState` is `.stale` and `bricks` is not empty — `resyncing · as of epoch N`. Color `LabTheme.warn`. The mirror does not store a map age; do not invent one.
6. `mapState` is `.empty`, or `.stale` with no bricks — `Waiting for map`. Color `LabTheme.dim`.
7. Fresh `.degraded` — `Degraded · <sensors>`, or `Degraded` when none are silent. A sensor is silent only when its Hz is a measured `0` (`mid360Hz == 0` → `Mid-360`, `jt128Hz == 0` → `JT128`). Nil Hz is unmeasured, not silent. Color `LabTheme.warn`.
8. Fresh `.paused` — `Paused`. Color `LabTheme.dim`.
9. Fresh `.building` — `Building reference`. Color `LabTheme.dim`.

No match (fresh tracking, map current): no banner.

`eventsState` is not a banner. Incomplete history is a line on the event list, specified below. `{sec: 0, nanosec: 0}` renders `time —`. Other stamps render `HH:mm:ss` UTC.

Bricks, whenever `!isMapCurrent` and the dictionary is not empty, draw at 35% opacity. That covers a stale status while `mapState` is still `.current`, and a `.stale` map. Markers do not draw in either case. When the map is current, bricks are full opacity even if the state is building or rig-moved; the marker rule is what hides the overlays.

## Voxel map

`RoomMapRenderer` is a new Metal renderer used only by Room. It copies the z-up camera from `PointCloudRenderer` (`up = (0,0,1)`, `CameraMath.fitToBounds`, clear `#0B0D10`). It does not share buffers with the lidar canvas. Drag orbits (yaw around z, pitch), scroll dollies, double-click refits. The lidar view is left alone.

The client republishes `mirror` on every status and on the 1 s tick. Re-upload voxel and brick-frame buffers only when `(mirror.mapRevision, mirror.voxelSize)` changes. `mapRevision` increments when the brick dictionary changes. Pair it with `voxelSize` because a snapshot can replace `voxelSize` without a brick change. Marker buffers upload when the marker set changes, not when the mirror value is republished unchanged.

Voxel index `(ix, iy, iz)` covers `[n·s, (n+1)·s)`. Its center is `(n + 0.5) · s`. Inside brick `(i, j, k)`, local `(x, y, z)` has index `(8i+x, 8j+y, 8k+z)`, using `Brick.evidence(x:y:z:)`.

| Evidence | Draw |
|---|---|
| occupied | one point at the cell center, color `#C5CDD6`, point size 4 |
| free | nothing |
| unknown | nothing at the cell |

A brick that contains any unknown voxel also draws its 12 edges in `LabTheme.faint` (`#5C6670`), line width 1. A brick that is entirely known, including all-free, draws no frame. Bricks the mirror does not contain are not framed. Occupied points stay grey so they do not collide with marker green, red, and amber.

The zlib snapshot golden is the numeric pin. After decode of `map_bricks_snapshot_zlib_le.hex` (`voxel_size` 0.05, room `lab`, epoch 43):

- Brick `(0, 0, 1)` is all free (`known` all `0xFF`, `occupied` empty): 0 points, 0 frame.
- Brick `(-1, 2, -3)` has occupied bit 0 of byte 0, local `(0,0,0)`, world index `(-8, 16, -24)`, center `(-0.375, 0.825, -1.175)`.
- The same brick has occupied bit 7 of byte 63, local `(7,7,7)`, world index `(-1, 23, -17)`, center `(-0.025, 1.175, -0.825)`.
- That brick also has unknown voxels, so it contributes one 12-edge frame and exactly two points.

`(index + 0.5) × 0.05` is the whole formula. `(-24 + 0.5) × 0.05 = -1.175`, and `(-1 + 0.5) × 0.05 = -0.025`.

### Camera

Content bounds are the union of:

- occupied cell centers
- corners of marker boxes that are actually drawn
- the four points of each drawn arrow (tail, tip, and both wing tips), using the head size in `ChangeOverlay.makeArrow` on `origin/change-overlay` (`0f5966b`): head `min(0.15, length × 0.35)`, wing `head × 0.45`

The golden event boxes sit near x = 1.25…2.125. The two occupied centers sit near x = −0.4…0. Fitting occupied centers alone leaves the markers outside the frame.

If that union is empty, fit the brick-frame corners. If those are empty too, leave the camera where it is.

Auto-fit on every change of those bounds until the user drags or scrolls. That first window is how a marker that arrives after the snapshot still gets into the frame. After the first drag or scroll, live updates do not fit. Double-click fits once to the current bounds and then stays frozen until the next double-click. A `mapRevision` bump is not itself a fit.

An `.off` connection with an empty mirror draws neither points nor frames. The canvas shows `Room memory unavailable`, 13pt, `LabTheme.dim`, centered. That string is the fixture Off mode and a client that was never started. It is not the copy for "bridge up, topics absent" or "no snapshot yet"; those use the banners above.

## Change markers

Colors match `ChangeOverlay.color` on `origin/change-overlay` (`0f5966b`):

| Kind | RGB | Theme |
|---|---|---|
| appeared | `(63, 163, 122) / 255` | `good` `#3FA37A` |
| disappeared | `(196, 90, 90) / 255` | `bad` `#C45A5A` |
| moved | `(196, 160, 74) / 255` | `warn` `#C4A04A` |

Do not merge `origin/change-overlay`. Copy only the draw conventions into `LiveRoomMarkers` and `RoomMapRenderer`:

- Twelve box edges and the twelve triangles (two per face) from `ChangeOverlay.edgePairs` and the overlay renderer's `boxTriangles`.
- Fill alpha `0.28`, emphasized fill `0.85`.
- A moved arrow from `fromCentroid` to `centroid`, head as in `ChangeOverlay.makeArrow`.
- Boxes depth-test and do not write depth. Centroid marks draw with depth compare always.

`aabb` is meters in the room frame. No sensor transform. A missing `fromCentroid` on a moved event draws the box only.

Draw a marker only when `mirror.isMapCurrent`, and not when the fresh state is `.building` or `.rigMoved`. `.degraded` and `.paused` keep markers while the map is current. A stale status, a stale map, a resync, and a disconnected socket keep the last bricks dimmed and draw no markers. The event list still shows the rows.

What is eligible, before that gate:

| Source | On the map | In the list |
|---|---|---|
| `openEvents` entry whose state is `.candidate` | dashed AABB, no fill | Events |
| `openEvents` entry whose state is `.confirmed` | solid AABB, fill 0.28; arrow if `.moved` and `fromCentroid` is set | Events |
| `eventsById` entry whose state is `.accepted` and (`mapEpoch == nil` or `mapEpoch < event.header.mapEpoch`), and the map is current | dashed unfilled outline, not counted | History, suffix `accepted · map updating` |
| other terminal states, and an accepted event whose `mapEpoch` the mirror has reached | no | History |

Dashes are 8 short segments along each edge. Metal lines are not dashed by the API.

The accepted outline is the Mac-side check in protocol §4.2 / Phase 3 §5.5 invariant 4. The Spark publishes the map message before ACCEPTED, and `openEvents` correctly omits terminal states, which is the steady state once `mapEpoch` has caught up. The client does not compute the "still waiting for that epoch" outline. This view does, from `eventsById` and `mapEpoch`. Do not patch `RoomMemoryMirror` for it. The outline is suppressed by the same `isMapCurrent` / building / rig-moved gate as every other marker.

A selected **confirmed** row uses fill `0.85` and draws on top. A selected **candidate** stays dashed and unfilled, and draws on top. A selected accepted-waiting outline stays unfilled. A selected history row that is not in the accepted-waiting case draws no box. Selection of an open row is `eventId`. Selection of a history row is `\(seq)-\(eventId)`.

Kind filters apply before drawing. A chip that is off removes that kind from the marker set.

## Event adaptation

The mirror keeps three collections. The view uses them for different jobs.

- `events` is the transition ring (last 1000 messages, oldest first). It is not the marker list and not the open-row list.
- `eventsById` is the latest applied transition per `event_id`.
- `openEvents` is the latest `.candidate` or `.confirmed` only, sorted by `event_id`. Current rows and current markers come from here.

History rows are the ring entries whose state is `.accepted`, `.reverted`, or `.expired`. One row per message. The row id is `\(seq)-\(event.eventId)`, not `eventId` alone, so two transitions of one change do not collapse. An older candidate that remains in the ring after a confirmed message is not a second Events row and not a second marker.

`eventsState` is an enum. Keep it as an enum all the way into the formatter. A `Time?` cannot tell `.complete` apart from `.incomplete(since: nil)`.

| `eventsState` | Event-list line | Open rows |
|---|---|---|
| `.complete` | no incomplete line | no outdated suffix |
| `.incomplete(since: nil)` | `incomplete` | `· may be outdated` |
| `.incomplete(since: {0, 0})` | `incomplete since time —` | `· may be outdated` |
| `.incomplete(since: t)` | `incomplete since HH:mm:ss` | `· may be outdated` |

There is no `outdatedEventIds` set on the client. The suffix comes only from `eventsState != .complete`.

The non-resurrection test drives the public mirror API (`connectionOpened`, `receive`, `tick`) with encoded messages, then reads the marker helper:

1. Snapshot at epoch 43, then a fresh tracking status at that epoch, so `isMapCurrent` is true.
2. One event, candidate, seq 1 → one dashed unfilled marker. The ring length is 1.
3. Same `eventId`, confirmed, seq 2 → one filled marker. The ring length is 2. Marker count stays 1.
4. Same `eventId`, accepted, seq 3, `header.mapEpoch` 44, mirror epoch still 43, status still fresh → one unfilled outline, caption does not count a change, history text contains `accepted · map updating`.
5. A later snapshot at epoch 44 → zero markers.
6. Step 3's mirror, queried again, still has one marker after the ring has both transitions. Do not feed a lower live seq to simulate this; the client treats that as a new generation.

## Event list

Right column, 280pt, well background, two sections. Slice 2.

Each row is two lines of 11pt mono, wrapping inside the column (about 256pt of text after padding). No horizontal scroller. Line limit 2 on each line.

```
● appeared · candidate · 8 voxels · 12:00:10
evidence — · α 0.05 · Mid-360 · no mask
```

- Line 1: kind-colored pip, kind, state, voxel count, `HH:mm:ss` UTC.
- Line 2, `LabTheme.dim`: evidence, alpha, sensors, then `no mask` or `accepted · map updating` or `may be outdated` when those apply.
- Kind words: `appeared`, `disappeared`, `moved`.
- State words: `candidate`, `confirmed`, `accepted`, `reverted`, `expired`.
- `evidence` nil → `evidence —`. A real zero is `evidence 0%` (integer percent).
- `alpha` nil → `uncalibrated`. A real value is `α 0.05` (two decimals). Never `α 0` for nil.
- Sensors: `Mid-360`, `JT128`, both as `Mid-360, JT128`, empty bitmask → `sensor —`.
- `mask == nil` → `no mask`. The mask's occupied voxels are not a second mesh. The AABB is the marker.

**Events.** `openEvents`, newest `header.stamp` first, then higher `header.seq`. The section is hidden when empty only if History is also empty and the connection is up; a live empty room shows the caption below rather than a blank column.

**History.** Terminal ring entries, collapsed by default. Same two-line shape. Hidden when empty.

The caption counts **drawn markers**, after the kind filter and after suppression:

`2 changes · 1 candidate on map`

- `changes` = drawn confirmed boxes
- `candidates` = drawn candidate boxes
- the accepted-waiting outline is in neither number
- suppression or a filter that hides a kind removes it from the count, and the rows stay in the list
- a current map with nothing open renders `0 changes`
- `.off` with an empty mirror renders `Room memory unavailable` in the column instead of `0 changes`

Three filter chips, mono 11, on by default: Appeared, Disappeared, Moved. Off hides that kind in both sections and on the map. Chip tint is the kind color when on, `LabTheme.dim` when off. Same chip pattern as `ChangeScrubberView`, fed by `RoomMemory.ChangeEvent.Kind`.

## Resync line

This bridge cannot carry `RequestSync` (`canPublish` is false). With `requestSyncSink == nil` the mirror never queues a request, so `activity.requestsUnsent` stays 0 even while the map is stale. The counter moves only when the mirror emits a request and the client cannot carry it (`noteRequestUnsent`).

Show `cannot request resync` when recovery is needed and no path exists.

- Recovery is needed when `mapState` is `.stale`, or `eventsState` is `.incomplete`.
- A path exists when `requestPathExists` is true (computed by `AppState`, above).
- Also show the line when `activity.requestsUnsent > 0`, whether or not the flag is set, because the client already counted a request that did not leave the machine.

When the count is non-zero the text is `cannot request resync · <n> unsent`. Otherwise it is `cannot request resync`. One placement, first match. The text appears once.

1. The visible banner is the map-stale line (`resyncing · as of epoch N`). Append ` · cannot request resync`, and ` · <n> unsent` when the count is non-zero, to that banner.
2. The event list has an incomplete-history line (`eventsState != .complete`) and rule 1 did not fire. That line gains the same suffix.
3. Standalone fallback. History is `.complete`, so the incomplete-history line is absent, and the visible banner is not the map-stale line. The text is its own line in the event column, above the sections, 11pt mono, `LabTheme.warn`. This covers a higher banner winning (connection lost, empty topics, stale status, rig moved, degraded, paused, building), the `Waiting for map` banner (a stale map with no bricks is not the map-stale line), no banner, and `requestsUnsent > 0` with a current map and complete history.

The standalone line does not replace the winning banner. It does not outrank connection loss, an empty topic list, or a stale status. The column stays on screen while the line is present, including when Events and History are both empty. Rules 1 and 2 suppress the standalone line.

The view does not set `requestSyncSink`, does not open `:8766`, and does not speak Foxglove. The relay (Phase 3 P11, the owner of the existing `RigService` tunnel) sets the sink before `start()`. Until that owner lands, the sink stays nil and this line is how the operator sees it.

## Fixture source

Fixture mode is a development source. It is not the thing this job ships as the room. `LiveRoomFixture` builds a `RoomMemoryMirror` and a `LiveRoomConnection` by decoding goldens and calling the mirror's public API. It does not construct a `LiveRoomClient`.

Hex parsing matches `RoomMemoryCDRTests.goldenBytes` (skip `#` comments). Load order for each file:

1. `Bundle.main` resource `fixtures/room/golden/hand/<name>.hex`
2. an explicit root passed by the test (`LiveRoomFixture.load(name:root:)`)
3. `Tests/Fixtures/room/golden/hand/<name>.hex` relative to that root

Unit tests pass the repo root. They do not depend on the process cwd. `build.sh` copies `Tests/Fixtures/room/golden/hand/` into `LabCommand.app/Contents/Resources/fixtures/room/golden/hand/`. Today it copies `fixtures/api` and one PCD only, which is why a bundled app cannot see the goldens.

| Mode | What the mirror holds | Screen |
|---|---|---|
| `map` | zlib snapshot (room `lab`, epoch 43) and a **synthetic** fresh tracking status for room `lab` with `coverage` nil and `eventsSeq` nil. Events: the appeared golden (candidate), the moved golden (confirmed), and one disappeared event built in code (below). Status is re-received once a second with a new `seq` so it stays inside the 3 s freshness window. | full-opacity voxels, three markers, status `epoch 43 · coverage — · events — · Tracking · cell 5.0 cm`, green pip. `events —` while event headers exist is the check that the bar is not reading `header.seq` |
| `status` | `memory_status_le` only (room `lab-a`, `events_seq` 12). No bricks. | `epoch 42 · coverage 75.0% · events 12 · Tracking`, banner `Waiting for map`, no points |
| `stale` | same bricks and events as `map`, status received at `now - 4`, connection left `.connected` with topics, no refresh timer | bricks at 35%, zero markers, banner `STALE · 4s`, status prefix `STALE · 4s ·`, no state word, amber pip |
| `unavailable` | empty mirror, connection `.off` | `Room memory unavailable`, idle pip |

The disappeared event is not a new hex golden. There is no disappeared hand golden. It is a `ChangeEvent` with room `lab`, `eventId` 11, seq 5 (after the moved golden's seq 3 and the appeared golden's seq 4), state `.confirmed`, kind `.disappeared`, box min `(3.0, 0.0, 0.0)` max `(3.5, 0.5, 0.5)`, `fromCentroid` nil, `mask` nil, `alpha` nil, `evidence` nil, sensors Mid-360. The box sits clear of the appeared box (x 1.875…2.125) and the moved box (x 1.25…1.75).

The two goldens and the synthetic status stay in room `lab`. `memory_status_le` stays in room `lab-a` and is only the `status` mode. Mixing them would show epoch 42 over epoch-43 bricks, and the mirror would count the other room as foreign.

The delta golden is not applied. The snapshot is already epoch 43. The mock does not contain a store.

The `Fixture` chip uses `LabTheme.dim` and the help string `Hand goldens. No rig contact.` A segmented control `Map / Status / Stale / Off` sits in the header only in fixture mode.

Map-mode caption from the three drawn markers: `2 changes · 1 candidate on map`.

## Screenshots

The app writes a sidecar next to each PNG when it draws Room: banner, status line, pip name (`good` / `warn` / `bad` / `idle`), marker counts, the event rows, and `canvas <x> <y> <w> <h>` in PNG pixels, origin top-left of the image. The rect is the Metal view's bounds in the captured window. It lies fully inside the PNG, with width and height each greater than 80, on every shot that draws the canvas. `unavailable` replaces the canvas with the centered sentence and omits the rect. Paths:

| Launch | PNG | What the check asserts |
|---|---|---|
| `LABCOMMAND_LIVE_ROOM=live` | `build/screenshots/live-room-live.png` | process connected: sidecar contains `ws://100.118.187.99:8767` and `.connected`. Also a status line or one of `Waiting for map`, `Room memory not running on Spark`, `connection lost`, `STALE`, `connecting`, `resyncing`. PNG width and height both > 400 |
| `=map` | `build/screenshots/live-room-map.png` | sidecar status `epoch 43 · coverage — · events — · Tracking · cell 5.0 cm`, pip `good`, rows contain `appeared`, `candidate`, `moved`, `disappeared`. Inside the `canvas` rect, inset 2 px: at least 8 pixels within ±12 per channel of `#3FA37A`, of `#C45A5A`, and of `#C4A04A`, and at least 8 within ±12 of `#C5CDD6`. Pixels outside the rect do not count. Marker edges are the full kind color; fill alpha `0.28` may fall outside ±12, so the count is edge pixels. The kind chips and the row pips are the same RGB and sit outside the canvas |
| `=status` | `build/screenshots/live-room-status.png` | sidecar contains `events 12`, `coverage 75.0%`, `Tracking`, and `Waiting for map`. Inside the canvas rect: zero pixels within ±12 of `#3FA37A`, `#C45A5A`, or `#C4A04A` |
| `=stale` | `build/screenshots/live-room-stale.png` | sidecar contains `STALE · 4s`, pip `warn`, marker count 0. Inside the canvas rect: zero pixels within ±12 of those three marker colors. Dimmed brick grey is not those colors |
| `=unavailable` | `build/screenshots/live-room-unavailable.png` | sidecar contains `Room memory unavailable`, pip `idle` |

Fixture launches are the four env values, each a separate process, each quit by PID. They do not pass `--mock` and they do not need it. Their sidecars must not contain `connected ws://100.118.187.99:8767`. The live launch is a different process. Wait up to 30 s for `.connected` before capturing. A deadline wrapper is `perl -e 'alarm N; exec @ARGV' --`, not `timeout`.

Capture the window of that PID. `screencapture` of the whole desktop is not the artifact.

### Visual inspection

A sidecar that contains the row words does not prove the rows are readable, and a whole-window color search can pass on the chips alone. After the pixel checks, open each PNG and read it. Record one line per file in the gate notes, and again in `DONE.md`: the words read off the image. Clipped, overlapping, or unreadable glyphs fail even when the sidecar is right.

- `live-room-live.png`: the status line or the banner on screen can be read. The canvas is the map or one of the banner sentences.
- `live-room-map.png`: the canvas shows grey points, one faint brick frame, a green dashed unfilled box, an amber filled box, and a red filled box, distinct from the three chips. The column shows three rows, two lines each. Kind and state read as `appeared` / `candidate`, `moved`, and `disappeared`. Timestamps are readable. Text stays inside the 280pt column and does not run under the canvas.
- `live-room-status.png`: `events 12`, `coverage 75.0%`, `Tracking`, and `Waiting for map` are readable. The canvas has no colored change box.
- `live-room-stale.png`: `STALE · 4s` is readable. Bricks are dim. No colored change box.
- `live-room-unavailable.png`: `Room memory unavailable` is readable and centered.

## Files

Slice 0, landed by cherry-pick, not authored here. `65122d2` first, and its gate, before the client files:

- `Sources/LabCommandCore/RoomMemoryMessages.swift`, `RoomMemoryCDR.swift`, `Sources/LabCommandCoreTestsRunner/RoomMemoryCDRTests.swift`, `Tests/Fixtures/room/golden/hand/memory_status_le.hex`, `scripts/spark/make_room_memory_goldens.py` — from `65122d2` only
- `Sources/LabCommandCore/LiveRoom/` (client, mirror, protocol, `API.md`) — after that gate
- `Sources/LabCommandCoreTestsRunner/LiveRoomTests.swift`
- `Tests/Fixtures/room/live/`
- `Sources/LabCommandCoreTestsRunner/main.swift` — `runLiveRoomTests()` and the probe argv, from the client worktree
- `ros/labcommand_msgs/msg/MemoryStatus.msg` — only the `events_seq` hunk inside `574feb2`, if that commit is cherry-picked

Slice 1, new:

- `Sources/LabCommandCore/LiveRoomView.swift` — format, voxels, launch parser. Markers can land in this file in slice 2.
- `Sources/LabCommand/LiveRoomView.swift` — SwiftUI page
- `Sources/LabCommand/RoomMapRenderer.swift` — Metal points and brick frames; boxes and arrows in slice 2
- `Sources/LabCommand/RoomMapView.swift`

Slice 1, edited:

- `Sources/LabCommand/AppState.swift` — `Dest.liveRoom`, construct and stop `LiveRoomClient` under `LiveRoomLaunch`
- `Sources/LabCommand/CommandCenterView.swift` — `case .liveRoom`
- `Sources/LabCommand/LabCommandApp.swift` — only if `stop()` is not already reaching `AppState.stop()` on quit. `AppState` remains the owner
- `build.sh` — copy the hand goldens into the app bundle. Still `--no-install` for the gate. Do not copy into `~/Applications`

Slice 2 and 3 add the marker and fixture sections of `LiveRoomView.swift`, `LiveRoomViewTests.swift`, and the screenshot script under `scripts/`. The test runner calls the new tests.

Not edited after slice 0: `PointCloudRenderer.swift`, `FoxgloveLiveClient.swift`, the landed `LiveRoom/*.swift` client sources, `RoomMemoryCDR.swift`, `RoomMemoryMessages.swift`, `RoomMemoryCDRTests.swift`, the hand golden, and `make_room_memory_goldens.py`. Those codec files change only by cherry-picking `65122d2`. `.msg` files are not hand-edited. Nothing under `spark/`, `~/Applications/LabCommand.app`, or `~/Developer/labcommand-spark`.

`DesignContractTests` scans `Sources/LabCommand` for `LabTheme.t(16` and larger, and for `coming soon`. Room copy stays at 15pt titles and 11–13pt body.

## Tests

`swift run LabCommandCoreTestsRunner` from the repo root, after slice 0 so the client's own tests run too. New cases:

- Decode `memory_status_le.hex`, assert `eventsSeq == 12`, `receive` it on a mirror, and assert the status line contains `events 12`. A nil `eventsSeq` renders `events —`. A mirror with baseline `eventsSeq == 4` and `status.eventsSeq == 12` renders `events 12`.
- Coverage `0.75` → `coverage 75.0%`, coverage `0` → `coverage 0.0%`, stale age `4.2` → `STALE · 4s`, cell `0.05` → `cell 5.0 cm`. The state word is absent when `isStatusFresh` is false.
- `alpha` nil → `uncalibrated`. Evidence nil → `evidence —`.
- `.complete` prints no incomplete line. `.incomplete(since: nil)` prints `incomplete` and not `incomplete since`. `.incomplete(since: a stamp)` prints `incomplete since HH:mm:ss`.
- Voxel samples for the decoded zlib snapshot: exactly the two centers above, one frame, the all-free brick silent.
- The candidate → confirmed → accepted sequence in "Event adaptation", including one marker while the ring holds two transitions, and the accepted outline until the snapshot epoch catches up.
- Building, rig-moved, a stale status, and `mapState == .stale` each produce zero boxes. A confirmed event still produces a box when the fresh state is `.degraded` and the map is current. A selected candidate is dashed and has fill alpha 0.
- Kind filter off removes that kind from the marker set and from the caption. Caption on the map fixture is `2 changes · 1 candidate on map`.
- `LiveRoomLaunch` maps the env table above. The fixture cases do not construct a client.
- Banner priority: a stale status wins over a remembered `.rigMoved`. An empty `memoryTopics` wins over `Waiting for map`. `cannot request resync` appears for a stale map with `requestPathExists == false` and a zero unsent count, and the unsent count is included when `noteRequestUnsent` has run.
- Resync placement, one line each. Map stale, path absent, unsent 0, events `.complete`, no higher banner: the warning is on the resync banner, and the standalone line is absent. The same recovery while the winning banner is `connection lost` and `eventsState` is `.complete`: the banner stays `connection lost`, and the standalone line is `cannot request resync`. The same with `.incomplete(since: nil)`: the incomplete line is suffixed, and the standalone line is absent. `requestsUnsent == 2` with a current map and `.complete` history: the standalone line is `cannot request resync · 2 unsent`. A stale map with no bricks and `.complete` history uses the standalone line, because the banner is `Waiting for map`.
- Fixture `map` and `status` do not share a `roomId`. Fixture `status` has `status.eventsSeq == 12`. Fixture `map` has `status.eventsSeq == nil`.

No new golden hex files. The default test run opens no socket and no SSH.

## Gate

1. `./build.sh --no-install` clean. That writes `build/LabCommand.app` and does not copy into `~/Applications`.
2. `swift run LabCommandCoreTestsRunner` passing, including `runLiveRoomTests()` and the cases above. Do not pass `live-room-probe`.
3. Live screenshot, from slice 1 onward: launch `build/LabCommand.app/Contents/MacOS/LabCommand` with `LABCOMMAND_LIVE_ROOM=live`. Quit that process by PID (`kill <pid>`). The PNG and sidecar are `build/screenshots/live-room-live.*`, and the sidecar shows `.connected` to `ws://100.118.187.99:8767`.
4. Fixture screenshots, four processes, the checks in the table above, the canvas-rect crop, and the visual inspection notes. Quit each by PID.
5. `codex review --base origin/main` (or `codex exec review`). Fix findings until Codex is clean.
6. Commit on `live-room-ui`. Do not push to `main`. Do not open a PR.
7. Write `DONE.md` in the worktree root: what was built, the build and test results, both screenshot paths, and the Codex outcome.

## Decisions

1. Room is a third destination. The Rig lidar view stays the recording surface.
2. The first user-visible cut is the live map and the status bar on `ws://100.118.187.99:8767`. The fixture is a later, explicit, offline mode.
3. The view binds to `LiveRoomClient.connection` and `.mirror`. `AppState` is the only caller of `start()` and `stop()`.
4. The status bar's events number is `mirror.status?.eventsSeq`. The hand status golden's value is 12.
5. Markers draw only while `isMapCurrent`, and not during building or rig-moved. Bricks that are held while the map is not current stay on screen at 35% opacity.
6. Connection, status freshness, and map freshness are separate. A connected socket is not a green pip and is not a live memory state.
7. Banner order is the Phase 3 order: topics absent, connection lost, stale status, then rig-moved, map stale, degraded, paused, building. Empty and stale-without-bricks say `Waiting for map`.
8. Open rows and their markers come from `openEvents`. History rows come from the ring and are keyed by seq and event id. `.incomplete(since: nil)` stays a distinct case.
9. An accepted event whose `mapEpoch` the mirror has not reached draws an uncounted outline until a current map catches up. The client is not modified for that.
10. Unknown voxels are a wireframe of any brick that still has an unknown cell. Free cells are undrawn. Occupied cells are grey points. Centers use `(index + 0.5) × voxelSize`.
11. `mapRevision` and `voxelSize` gate voxel uploads. The camera auto-fits only until the user orbits or zooms, and the fit includes drawn boxes and arrow points.
12. This bridge has no `clientPublish`. The view reports `cannot request resync` and does not grow a request path. The relay owner sets `requestSyncSink`. When history is complete and the banner is not the map-stale line, that report is a standalone line in the event column.
13. The change-overlay branch contributes colors, box topology, fill alphas, and the arrow. It is not merged.
14. Selected candidates stay dashed and unfilled. The caption counts drawn markers. Event rows wrap on two lines inside 280pt.
15. `65122d2` is on this branch, and its decode-and-test gate has passed, before any `LiveRoom` client file is added. Marker colors are counted inside the sidecar's canvas rect. Row readability is read off the PNG.
