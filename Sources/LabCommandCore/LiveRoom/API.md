# LiveRoom: public API (LabCommandCore)

The Mac side of the live room-memory link. A Foxglove WebSocket client for the Spark's
`room_memory` bridge, a pure mirror of what the Spark holds, and one `@MainActor`
`ObservableObject` for SwiftUI to bind to. Messages decode with `RoomMemory.*` in
`RoomMemoryCDR.swift` / `RoomMemoryMessages.swift`.

This file is the API surface only: types, inputs, outputs and what the `EventHistory` values
mean. Every rule (generations, answers, completion, legacy wire) is
`docs/ROOM-MEMORY-PROTOCOL.md` §5 (D34–D37); PLAN-PUBGEN §4 has the reducer's state machine and
invariants.

Live bridge facts (probed 2026-10-15, `ws://100.118.187.99:8767`, foxglove-bridge 3.6.0):
- Topics: `/room_memory/{map_bricks,events,status,sync/map_bricks,sync/events}`; a v2 node adds
  `/room_memory/sync/end`. No `/room_memory/epoch` yet.
- `capabilities: ["connectionGraph"]` only (re-probed 2026-10-16 on c5518eb, issue #21). **No
  `clientPublish` and no `services`**, so this deployment has no request path until the image is
  rebuilt with D39.
- Request path (protocol D39, §5.3), in order: the node's `/room_memory/sync/request` service
  (`labcommand_msgs/srv/RequestSync`) when the bridge offers `services` and advertises it; called
  with the `RequestSync` CDR as payload. Else a client channel on `/room_memory/request_sync` when the
  bridge offers `clientPublish`. Else `requestSyncSink` (for the `:8766` relay, Phase 3 P11). If none,
  requests are counted as unsent ("No request path"). The service can appear after the mirror
  opened, and go away when the node restarts: the client tells the mirror (`requestPathChanged`).
- The deployed node predates publisher generations, so today the mirror runs in legacy mode
  against it (shown, unverified, never current, no requests).

## Binding from the UI

```swift
import LabCommandCore

@StateObject var room = LiveRoomClient()            // default URL ws://100.118.187.99:8767
room.start()                                        // connect; reconnects with backoff until stop()
room.stop()                                         // close; the mirror stays, greyed (STALE)

room.connection        // LiveRoomConnection
room.mirror            // RoomMemoryMirror: everything to render
room.mirror.mapRevision // bumps on every brick change: re-upload bricks only when it changes
```

`LiveRoomClient` is `@MainActor final class: ObservableObject`. Every `@Published` property
changes on the main actor. It republishes `mirror` on each applied message and on a 1 s tick,
so time-based staleness re-renders without help from the view. It subscribes to every
`LiveRoomTopic` the bridge advertises with a matching schema, `sync/end` included (and `activity` and
`epoch` when advertised: the node publishes `activity`, and no node publishes `epoch` yet), stamps each
`Action.requestSync` with the wall clock, and sends it.

### `LiveRoomClient`

| member | type | meaning |
|---|---|---|
| `init(url: URL = LiveRoomClient.defaultURL, roomId: String? = nil, publishRequests: Bool = true)` | | `roomId` nil = adopt the first room seen; `publishRequests: false` = read-only, never advertises or sends |
| `connection` | `LiveRoomConnection` | socket state, below |
| `mirror` | `RoomMemoryMirror` | the mirrored state |
| `log` | `[String]` | last 200 diagnostic lines, newest last |
| `requestSyncSink` | `((RoomMemory.RequestSync) -> Bool)?` | another request path (the `:8766` relay); return true if sent |
| `requestPath` | `LiveRoomRequestPath` | `.bridgeService(id:)`, `.bridgeChannel`, `.relay` or `.unavailable` (shown as "no request path") |
| `start()`, `stop()` | | |

### `LiveRoomConnection` (enum)

- `.off`
- `.connecting(url: URL, attempt: Int)`
- `.connected(LiveRoomServer)`: `LiveRoomServer { name, capabilities: [String], canPublish: Bool, memoryTopics: [String], requestServiceId: UInt32? }`
  (`requestServiceId`: the bridge's id for the request service while it is advertised).
  `memoryTopics` is empty while the bridge is up but no `/room_memory/*` topic is advertised,
  which renders "Room memory not running on Spark", never an empty room.
- `.waiting(retryIn: TimeInterval, reason: String)`: between attempts.

### `LiveRoomTopic` (enum)

`.mapBricks`, `.events`, `.status`, `.activity` (`/room_memory/activity`, `labcommand_msgs/msg/Activity`),
`.epoch` (`/room_memory/epoch`, `labcommand_msgs/msg/Epoch`), `.syncMapBricks`, `.syncEvents`, `.syncEnd`
(`/room_memory/sync/end`, `labcommand_msgs/msg/SyncEnd`). `isSync` is true for the three answer topics.

### `RoomMemoryMirror` (pure `struct`, `Sendable`, `Equatable`)

It holds no socket and no clock, so tests drive it directly.

| input | returns | meaning |
|---|---|---|
| `init(roomId: String? = nil, limits: Limits = Limits(), tokenSeed: UInt64? = nil)` | | `tokenSeed` nil seeds request tokens from the system generator; tests pass a seed |
| `connectionOpened(at:canRequest:)` | `[Action]` | socket open and subscribed; `canRequest` = some request path exists |
| `connectionClosed(at:)` | | socket closed |
| `receive(topic:payload:at:)` | `[Action]` | one CDR payload (the Foxglove frame without its header) |
| `tick(at:)` | `[Action]` | clock tick: freshness, timeouts, request scheduling |
| `noteRequestUnsent()` | | the client could not carry the last request |

`Action` has one case, `.requestSync(RoomMemory.RequestSync)`: a v2 request with
`expectedGeneration` and a fresh `token`, `header.stamp` `{0, 0}` (the sender stamps it). Times
are `TimeInterval` on one monotonic clock (`ProcessInfo.systemUptime`). `Limits` bounds the event
log (1000), `eventsById` (10,000), the seq digest (10,000) and the coverage's ranges (1000);
eviction only lowers certainty.

**Identity and wire**

| property | type | meaning |
|---|---|---|
| `roomId` | `String?` | room being mirrored |
| `wire` | `Wire` | `.undetermined`, `.v2`, `.legacy`: this connection's layout |
| `connection` | `Connection` | `.offline`, `.awaitingStatus(connection:)`, `.observing(connection:generation:)` |
| `activeGeneration` | `UInt64?` | the publisher generation that renders as current; nil in legacy mode |
| `highestGeneration` | `UInt64?` | highest generation ever installed; never falls |
| `archive` | `[ArchivedGeneration]` | the newest 2 retired runs, display only: `generation` (nil = legacy run), `openEvents`, `bricks`, `voxelSize`, `mapEpoch` |
| `continuity` | `EventHistory.Continuity` | below |
| `canRequest` | `Bool` | a request path exists on this connection |
| `retrievalStatus(_ topic:)` | `RetrievalStatus` | `.idle`, `.waiting` (request out), `.receiving` (its `SyncEnd` arrived) |

**Map**

| property | type | meaning |
|---|---|---|
| `bricks` | `[RoomMemory.BrickCoordinate: RoomMemory.Brick]` | the active generation's bricks, else the newest archived run's (greyed); an absent brick is UNKNOWN, never free |
| `voxelSize` | `Float?` | meters |
| `mapEpoch` | `UInt64?` | epoch the bricks describe |
| `mapSeq` | `UInt64?` | last applied `map_bricks` seq of the active generation |
| `mapRevision` | `Int` | +1 whenever the active chain's bricks change |
| `mapState` | `MapState` | `.empty`, `.current`, or `.stale(StaleReason)` |
| `isMapCurrent` | `Bool` | `mapState == .current` **and** status fresh. Draw change overlays only when true |

`StaleReason`: `.disconnected`, `.awaitingStatus` (connected, no status of the generation on this
connection yet), `.seqGap`, `.wrongBase`, `.epochMismatch`, `.rejected`, `.newGeneration` (a newer
run has no map yet; the bricks shown are the earlier run's), `.unverified` (legacy wire). While
stale, the last good bricks stay: render them greyed as "resyncing · as of epoch N", never as current.

**Events** (active generation only)

| property | type | meaning |
|---|---|---|
| `events` | `[LiveRoomEvent]` | ring of the last 1000 event messages, oldest first |
| `eventsById` | `[UInt64: LiveRoomEvent]` | highest-seq transition received per `event_id` |
| `openEvents` | `[LiveRoomEvent]` | CANDIDATE or CONFIRMED, sorted by `event_id` |
| `archivedOpenEvents` | `[ArchivedEvent]` | open events of archived runs, `ArchivedEvent { generation: UInt64?, entry }`: "earlier run · may be outdated", never drawn as current |
| `eventHistory` | `EventHistory` | below |

`LiveRoomEvent { seq, receivedAt, fromSync, event: RoomMemory.ChangeEvent }`; `fromSync` is
diagnostics only, and `isOpen` is CANDIDATE or CONFIRMED. Only `.confirmed` renders as a change,
`.candidate` as a candidate, and terminal states never.

**`EventHistory`** (derived after every input, never stored)

| member | type | meaning |
|---|---|---|
| `generation` | `UInt64?` | the active generation; nil in legacy mode |
| `coverage` | `Coverage` | `.unknown`: no witness and nothing received, the coverage outgrew its bound, or a legacy wire. `.empty`: a v2 witness (status `EVENTS_SEQ_NONE`, or `SyncEnd` bound `SEQ_NONE`) and nothing received. `.completeThrough(n)`: every seq 0…n of the generation was received. `.missing(pending:lost:)`: seqs of [0, top] not received; `pending` may still arrive, `lost` the Spark declared unavailable |
| `freshness` | `Freshness` | `.fresh`, `.stale(age: TimeInterval?)` (nil: no status on this connection yet), `.offline` |
| `continuity` | `Continuity` | `.verified`: only this generation observed since launch. `.broken(sinceGeneration:)`: an earlier run in which something was observed was archived (nil: it was a legacy run); never heals. `.unknown`: legacy wire or nothing observed |
| `caughtUp` | `Bool` | coverage empty or complete, and a fresh status of the generation on this connection names an `events_seq` covering the newest seq received; an as-of observation |
| `contiguousThrough` | `UInt64?` | top of the run of received seqs from 0 |
| `missingCounts` | `(missing: UInt64, lost: UInt64)` | seq counts for the history line |
| `isComplete` | `Bool` | coverage is `.empty` or `.completeThrough` |

**Status**

| property | type | meaning |
|---|---|---|
| `status` | `LiveRoomStatus?` | newest `MemoryStatus` of the active generation (legacy: of the legacy run) |
| `isStatusFresh` | `Bool` | it arrived on this connection less than 3 s ago |

`LiveRoomStatus` copies the wire fields so the UI never needs the codec: `roomId`, `mapEpoch`,
`seq`, `stamp`, `state: RoomMemory.MemoryState`, `brickCount`, `openEventCount`,
`coverageKnownFraction: Float?`, `poseResidualM: Float?`, `windowS: Float?`, `mid360Hz: UInt16?`,
`jt128Hz: UInt16?`, `droppedWindows`, `eventsSeq: UInt64?`, `publisherGeneration: UInt64?`,
`receivedAt`. Nil means not measured (or, for the generation, a legacy node). Never render a nil as 0.

**Activity and Epoch** (active generation only; full state, the newest `seq` wins)

| property | type | meaning |
|---|---|---|
| `roomActivity` | `LiveRoomLatest<RoomMemory.Activity>?` | newest `Activity` (motion next to the rig, never a change) |
| `isActivityFresh` | `Bool` | it arrived on this connection less than 3 s ago |
| `epochTick` | `LiveRoomLatest<RoomMemory.Epoch>?` | newest `Epoch` timeline tick |
| `isEpochFresh` | `Bool` | it arrived on this connection less than 15 s ago |

`LiveRoomLatest { message, receivedAt, connection }`. A newer generation starts both empty, so a
restarted node's `seq` 0 applies. They never decide the connection's layout: one that arrives
before a stateful message, or in the other layout (a node from before their v2 tail sends them
legacy next to a v2 status), is dropped and counted in `activity.sideTopicLayout`.

**Activity**: `activity: LiveRoomActivity`

- `topics: [String: TopicActivity]`, with `TopicActivity { count: Int, lastAt: TimeInterval }` per topic
- `rejected`: undecodable or invalid messages, dropped whole
- `duplicates`, `gaps` (map messages that did not chain), `foreignRoom`: counts
- `generationChanges`: runs archived (a publisher restart, or a legacy/v2 switch)
- `olderGeneration`: messages below the highest generation, ignored
- `contractViolations`: a seq received twice with different payloads (the first is kept)
- `wireMismatch`: a message in the other layout than the connection's first one, dropped
- `sideTopicLayout`: `Activity`/`Epoch` dropped because the connection's layout was not decided yet or is the other one
- `requestsSent`, `requestsUnsent`: unsent means no request path carried it
- `lastRequest: RoomMemory.RequestSync?`

State to show in the banner: `status.state` (BUILDING / TRACKING / RIG_MOVED / DEGRADED / PAUSED),
but only while `isStatusFresh`. Otherwise "STALE · age".

## Screen (`LiveRoomScreen.make`)

`LiveRoomScreen` is pure: `make(mirror:connection:requestPathExists:started:now:kinds:selection:)`.
Besides the pip, banner, status line, canvas, markers, caption and event rows it carries:
- `historyLine`: "history complete through #n", "no events this run", "history incomplete: n
  missing (k lost on the Spark)" (+ " · no request path" when nothing can fetch the pending
  seqs), "history unverified (old Spark node)", or nil before any witness;
- `restartBanner`: "Spark node restarted · events from the earlier run may be missing" while
  `continuity` is broken;
- `earlierRows`: the archived open events, "earlier run · may be outdated";
- the status line's run tag, `run …1234` (`LiveRoomScreen.runTag`, the generation's last 4 digits).

`LiveRoomFixture` modes (`LABCOMMAND_LIVE_ROOM`): `map`, `status`, `stale`, `lost`, `restart`,
`legacy`, `unavailable`; the v2 ones use the hand goldens' one publisher generation
(`LiveRoomFixture.generation`).
