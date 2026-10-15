# LiveRoom: public API (LabCommandCore)

The Mac side of the live room-memory link. A Foxglove WebSocket client for the Spark's
`room_memory` bridge, a pure mirror of what the Spark holds, and one `@MainActor`
`ObservableObject` for SwiftUI to bind to. Wire contract: `docs/ROOM-MEMORY-PROTOCOL.md`.
Messages decode with `RoomMemory.*` in `RoomMemoryCDR.swift` / `RoomMemoryMessages.swift`.

Live bridge facts (probed 2026-10-15, `ws://100.118.187.99:8767`, foxglove-bridge 3.6.0):
- Topics: `/room_memory/{map_bricks,events,status,sync/map_bricks,sync/events}`. No `/room_memory/epoch` yet.
- `capabilities: ["connectionGraph"]` only. **No `clientPublish`**, so the client cannot send
  `RequestSync` over this socket. It advertises its client channel only when the server offers
  `clientPublish`. Otherwise it hands requests to `requestSyncSink` (for the `:8766` relay, Phase 3
  P11), and if that is nil it counts them as unsent. The map still arrives, because the Spark
  publishes a snapshot on the live topic on its own.
- Live `MemoryStatus` carries `events_seq` (Phase 3 P16). This branch carries that codec change
  (`d751db6`, cherry-picked from `phase3-design`).

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
so time-based staleness re-renders without help from the view.

### `LiveRoomClient`

| member | type | meaning |
|---|---|---|
| `init(url: URL = LiveRoomClient.defaultURL, roomId: String? = nil, publishRequests: Bool = true)` | | `roomId` nil = adopt the first room seen; `publishRequests: false` = read-only, never advertises or sends |
| `connection` | `LiveRoomConnection` | socket state, below |
| `mirror` | `RoomMemoryMirror` | the mirrored state |
| `log` | `[String]` | last 200 diagnostic lines, newest last |
| `requestSyncSink` | `((RoomMemory.RequestSync) -> Bool)?` | another request path (the `:8766` relay); return true if sent |
| `start()`, `stop()` | | |

### `LiveRoomConnection` (enum)

- `.off`
- `.connecting(url: URL, attempt: Int)`
- `.connected(LiveRoomServer)`: `LiveRoomServer { name, capabilities: [String], canPublish: Bool, memoryTopics: [String] }`.
  `memoryTopics` is empty while the bridge is up but no `/room_memory/*` topic is advertised,
  which renders "Room memory not running on Spark", never an empty room.
- `.waiting(retryIn: TimeInterval, reason: String)`: between attempts.

### `RoomMemoryMirror` (pure `struct`, `Sendable`, `Equatable`)

It holds no socket and no clock, so tests drive it directly. Inputs: `connectionOpened`,
`connectionClosed`, `receive(topic:payload:at:)` and `tick(at:)`. Each returns the
`[RoomMemoryMirror.Action]` the client must perform; today the only action is
`.requestSync(RoomMemory.RequestSync)`. Times are `TimeInterval` on one monotonic clock
(`ProcessInfo.systemUptime`).

**Map**

| property | type | meaning |
|---|---|---|
| `roomId` | `String?` | room being mirrored |
| `bricks` | `[RoomMemory.BrickCoordinate: RoomMemory.Brick]` | bricks by key; an absent brick is UNKNOWN, never free |
| `voxelSize` | `Float?` | meters |
| `mapEpoch` | `UInt64?` | epoch the bricks describe |
| `mapSeq` | `UInt64?` | last applied `map_bricks` seq |
| `mapRevision` | `Int` | +1 whenever `bricks` changes |
| `mapState` | `MapState` | `.empty`, `.current`, or `.stale(StaleReason)` |
| `isMapCurrent` | `Bool` | `mapState == .current` **and** status fresh. Draw change overlays only when true |

`StaleReason`: `.disconnected`, `.deltaFirst`, `.seqGap`, `.wrongBase`, `.epochMismatch`, `.rejected`.
While stale, the last good bricks stay. Render them greyed as "resyncing · as of epoch N", never as current.

**Events**

| property | type | meaning |
|---|---|---|
| `events` | `[LiveRoomEvent]` | ring of the last `RoomMemoryMirror.eventCapacity` (1000) event messages, oldest first |
| `eventsById` | `[UInt64: LiveRoomEvent]` | latest applied transition per `event_id` (at most 10,000) |
| `openEvents` | `[LiveRoomEvent]` | CANDIDATE or CONFIRMED, sorted by `event_id` |
| `eventsSeq` | `UInt64?` | events baseline: the highest seq applied contiguously |
| `eventsState` | `EventsState` | `.complete` or `.incomplete(since: RoomMemory.Time?)`, computed on every read. Complete only when every seq in `[0, max(status events_seq high-water, highest seq received)]` of this generation was received, no earlier generation was cut off, and a status on the open connection vouches for the high-water. `since` is the stamp of the last message in the gapless run from 0 (nil when seq 0 is missing). Incomplete renders "incomplete since t", with open events "may be outdated" |

`LiveRoomEvent { seq: UInt64, receivedAt: TimeInterval, fromSync: Bool, event: RoomMemory.ChangeEvent }`.
Only `.confirmed` renders as a change, `.candidate` as a candidate, and terminal states never.

**Status**

| property | type | meaning |
|---|---|---|
| `status` | `LiveRoomStatus?` | newest `MemoryStatus` |
| `isStatusFresh` | `Bool` | newest status is less than 3 s old (updated on every tick) |

`LiveRoomStatus` copies the wire fields so the UI never needs the codec: `roomId`, `mapEpoch`,
`seq`, `stamp`, `state: RoomMemory.MemoryState`, `brickCount`, `openEventCount`,
`coverageKnownFraction: Float?`, `poseResidualM: Float?`, `windowS: Float?`, `mid360Hz: UInt16?`,
`jt128Hz: UInt16?`, `droppedWindows`, `eventsSeq: UInt64?`, `receivedAt`.
Nil means not measured. Never render a nil as 0.

**Activity**: `activity: LiveRoomActivity`

- `topics: [String: TopicActivity]`, with `TopicActivity { count: Int, lastAt: TimeInterval }` per topic
- `rejected`: undecodable or invalid messages, dropped whole
- `duplicates`, `gaps`, `foreignRoom`: counts
- `requestsSent`, `requestsUnsent`: unsent means no request path
- `lastRequest: RoomMemory.RequestSync?`

State to show in the banner: `status.state` (BUILDING / TRACKING / RIG_MOVED / DEGRADED / PAUSED),
but only while `isStatusFresh`. Otherwise "STALE · age".

## Resync rules implemented (protocol §5)

- **Map.** A snapshot is always accepted. A delta applies only when `seq == mapSeq + 1` and
  `base_epoch == mapEpoch`. Removed indices refer to the base epoch's canonical (sorted) brick list.
  A gap, a wrong base, a delta first after (re)connect, or an invalid delta makes the map STALE and
  requests `RequestSync{mapBricks, since: (mapEpoch, mapSeq)}`, or `since: nil` with no mirror.
  So does **epoch change**: `status.map_epoch ≠ mapEpoch` on 2 consecutive statuses with no map
  applied between them (D20). A duplicate (`seq ≤ mapSeq` and `epoch ≤ mapEpoch`) is counted and ignored.
- **Events.** `seq == eventsSeq + 1` applies. A duplicate is counted. Any other seq is a gap: the
  list goes incomplete and requests `RequestSync{events, since: (epoch, eventsSeq)}`. So does an
  **events_seq gap**: a status `events_seq` above the baseline that the next status still finds
  unreached. An event's state changes only on a message with a higher seq than the last one applied to it.
- **Answers.** A replayed snapshot stays stale until its seq reaches the newest live seq and its
  epoch is at least the latest status epoch; the request stays open so the deltas after an older
  snapshot still apply. An events answer settles once its sync topic has been quiet for 2 s.
  A `SINCE_NONE` answer, not the first live event, settles what exists. Seqs an answer could
  not fill (the bounded fallback) are skipped: the baseline jumps to the highest replayed seq and
  the history stays incomplete. A status `events_seq` with no events held, seen again on the next
  status, marks the list incomplete even when the connect request is already outstanding, and
  triggers `SINCE_NONE` when nothing is outstanding.
- **Node restart.** A status `seq` lower than the last one, or `events_seq` below the events
  baseline on 2 consecutive statuses, starts a new generation. The events baseline is dropped
  (history incomplete) and refetched with `SINCE_NONE`; the map's live high-water mark resets.
- **Partial first history.** A first event above seq 0 with no sync coming means earlier events
  exist that the Mac never saw: the list reads incomplete.
- **Unreadable map payload.** The bricks stay, the map goes STALE (`.rejected`) and resyncs.
- **Sync topics** are read only while a request is outstanding on that topic. A request repeats
  after 5 s, then 10, 20 and 30 s, until resolved. At most one per topic per 5 s.
- **On connect**, both topics are requested once `roomId` is known.
- **Disconnect** makes the map STALE and the events incomplete.
