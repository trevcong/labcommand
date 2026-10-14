# Phase 3: live integration of the Spark `room_memory` node with the LabCommand mirror

Status: design for review, 2026-10-14. Nothing here is built.

This doc does not change the v1 wire contract. Where Phase 3 needs a change to a decided rule, the change is listed as a numbered proposal (§10) for Trevor and Codex. `docs/ROOM-MEMORY-PROTOCOL.md` stays authoritative until a proposal is accepted.

**Sources.**
- `~/Developer/lc-tasks/RESEARCH_BRIEF.md`: §2.3–§2.5, §3 Phase 3, §5. `~/Developer/RESEARCH_BRIEF.md` does not exist; only the lc-tasks copy was read.
- `docs/ROOM-MEMORY-PROTOCOL.md`, cited below as "protocol §n", with decisions D5, D9 and D19 of 2026-10-14.
- The Python reference node: `room-memory-node` branch, `spark/room_memory/`.
- The C++ port: `room-memory-cpp` branch, `spark/room_memory_cpp/PLAN.md`.
- The Mac live client: `FoxgloveProtocol.swift`, `LiveSession.swift`, `FoxgloveLiveClient.swift`.
- The measured link numbers in `lc-claude13.log`.
- External practice, cited in §12. Bridge behaviour was read from the foxglove-sdk source at tag `ros-v3.4.1`.

No Spark, rig or app was touched to write this.

## 0. Bottom line

- **A dedicated, read-only bridge for memory topics.**
  - The Mac opens one memory WebSocket to a second foxglove_bridge instance, which runs inside the room-memory container (P12).
  - That bridge serves only `/room_memory/*` and accepts no client publishing.
  - The existing `:8765` bridge, LIVE and SNAPSHOT are not touched.
- **Resync requests and operator actions go over the ssh-authenticated `:8766` API, not the bridge (P11).**
  - The bridge documents `client_topic_whitelist`, but at the deployed version and on `main` the parameter is parsed and never applied ([F2]).
  - With client publishing on, any peer that reaches the port could publish any known type on any topic, including a forged `/livox/lidar` cloud into the memory node.
- **The budget is easy in steady state and hard on reconnects.**
  - Steady state is about 0.24 KB/s down and 0.15 KB/s up, estimated with per-packet overhead (§3).
  - One reconnect costs about 110 KB.
  - The protocol's own retry rate allows about 22 KB/s during a resync storm, 4.5× the budget. Backoff on the Mac and a snapshot-answer cap on the Spark bound it (P3, P4).
- **Correctness comes from `seq` and `map_epoch`, not from QoS.**
  - DDS QoS governs only the same-host hop from node to bridge.
  - Across the tailnet the hop is one TCP stream. Messages are lost in only two places: the bridge's per-client backlog, which drops the oldest message on overflow ([F1]), and disconnects.
  - Either loss shows up as a `seq` gap or an epoch mismatch, which protocol §5 already turns into STALE and a resync.
  - `map_bricks` and `events` therefore stay volatile. A latched copy replayed by the bridge could rewind a Mac that is already current (§4.3).
- **The governing invariant: the mirror is never shown as current while it is wrong.** It may be stale, incomplete or empty, and it says so. Every test in §7 checks this under faults.
- **The Spark side is not ready for live integration yet.**
  - Both nodes (Python oracle, C++ production) publish only `status` and `events`.
  - They have no `map_bricks`, no `epoch`, no sync topics, no RIG_MOVED, and never emit CONFIRMED. Those are PR 2.3 and 2.4.
  - The rollout (§9) gates on them.

## 1. Starting point and scope

| piece | state on 2026-10-14 | Phase 3 needs |
|---|---|---|
| Wire contract (protocol §1–§8) | merged in #10; Swift codecs and hand goldens; Spark-produced goldens not yet made (protocol §9) | Spark goldens (confirm D4's `uint64` alignment) before stage S3 |
| Mac `RoomMemoryStore` (PR 1.1) | not built | the mirror's data model; §5 adds the connection layer around it |
| Python node `spark/room_memory` | reference oracle; `status` 1 Hz and `events` only; states BUILDING, TRACKING, DEGRADED | stays the oracle; not deployed |
| C++ node `spark/room_memory_cpp` | byte-identical to Python; 6.4–16.5 ms per 1 s window on an M5 Max (target < 200 ms); same two topics | production node; must add `map_bricks`, `epoch`, the sync topics and the `RequestSync` handler (PR 2.4), and CONFIRMED, moved supersession and RIG_MOVED (PR 2.3) |
| foxglove_bridge on `:8765` | built on foxglove-sdk-cpp v0.25.1 (`FoxgloveProtocol.swift`), which is bridge release 3.4.1, inferred from the changelog's SDK pin ([F3]); ROS 2 Jazzy; subprotocol `foxglove.sdk.v1`; its parameters and container are not known from the Mac | unchanged (P12) |
| Mac live client | per-sensor sockets, direct `ws://<tailnet IP>:8765`, ATS exception for the 100.x IP, backoff 1 s … 30 s | a memory socket with its own state machine (§5); advertise parsing that keeps `schemaEncoding` and `schema`, which `Channel` drops today |
| Command-center API on `:8766` | polled every 5 s over the ssh tunnel, ~0.4 KB/s measured | a `room_memory` block (§2.4) and a request relay (P11); both are changes to Trevor's `command_center.py` |

Out of scope: the change-detection algorithm (PR 2.x, `docs/CHANGE-DETECTION.md`), rendering, and the research track.

## 2. Architecture and topics

### 2.1 Data path

```
DGX Spark (ROS 2 Jazzy, --net=host)                                                Mac (LabCommand)
┌───────────────────────────────┐     DDS     ┌────────────────────┐   WireGuard, TCP   ┌──────────────────────────────┐
│ lidar-dual container          │ ──lidar───▶ │ foxglove_bridge    │ ◀═ ws:// :8765 ══▶ │ FoxgloveLiveClient           │
│  Mid-360, JT128 drivers       │             │  :8765 (unchanged) │   per-sensor socks │  SNAPSHOT / LIVE (unchanged)  │
├───────────────────────────────┤             └────────────────────┘                    ├──────────────────────────────┤
│ room-memory container         │                                                       │ MemoryClient (new, §5)        │
│  room_memory_cpp ◀── lidar    │                                                       │  → MemoryMirror (Core, pure)  │
│   │ /room_memory/* (6 topics) │ ──DDS──▶ foxglove_bridge :8767, read-only ◀═ ws:// ══▶│  → RoomMemoryStore (PR 1.1)   │
│   ▲ /room_memory/request_sync │                                                       │  → brick renderer, timeline   │
│   │ /room_memory/operator     │                                                       │                              │
│  ~/room-memory/<room_id>/     │                                                       │                              │
└───┼───────────────────────────┘                                                       │                              │
    └── command_center.py :8766 (relay, P11) ◀═══ ssh tunnel: status poll, POST requests ═══▶ RigService                │
                                                                                        └──────────────────────────────┘
```

**One socket, separate from LIVE.** The Mac's memory socket is independent of the LIVE sockets.
- A 3 MB JT128 frame never queues ahead of a 68-byte status, and stopping LIVE never drops the mirror.
- The bridge keeps an outgoing queue per client of `message_backlog_size` messages (default 1024). It drops the oldest data-plane message when that queue is full ([F1]).

**Dedicated bridge (P12).** A second foxglove_bridge runs in the room-memory container, on its own port (8767 proposed).
- That container already builds `labcommand_msgs`. A bridge can only advertise and subscribe to a type it has in its own environment ([F2]), so reusing `:8765` would mean rebuilding the lidar stack's bridge image.
- The dedicated bridge's parameters ([F1]):

  | parameter | value |
  |---|---|
  | `port` | 8767 |
  | `address` | the Spark's tailnet IP (the default `0.0.0.0` listens on every interface) |
  | `topic_whitelist` | `['^/room_memory/(map_bricks\|events\|status\|epoch\|sync/map_bricks\|sync/events)$']` |
  | `capabilities` | `[]`: no `clientPublish`, `parameters`, `services`, `connectionGraph` or `assets` |
  | `message_backlog_size` | default |

- **Benefits.**
  - LIVE is untouched.
  - No restart of the `:8765` bridge, which would interrupt LIVE.
  - The advertise burst on connect covers six channels, not the lidar topics too.
  - Rollback is stopping one container.
- **Version.** Pin 3.4.1 first, the same SDK the Mac client already talks to. An upgrade to the current 3.6.1 ([F3]) is a stage of its own, which reruns S1.

**Direct tailnet path.** The Mac dials the Spark's tailnet IP directly, through `FoxgloveProtocol.streamHosts` and `StreamHostRotation`, as LIVE does.
- WireGuard already encrypts it (§8).
- No ssh process needs supervising.
- The existing ATS exception names the IP, not a port, so port 8767 needs no `Info.plist` change.
- With `address` bound to the tailnet IP, the LAN hop in `streamHosts` does not apply to memory mode.

**The Spark is the system of record; the Mac mirror is a cache.**
- The reference map and the event log live on Spark disk (brief §2.2 step 7).
- A Mac that was closed for a week reconnects, takes one snapshot, and gets the recent event history (§4.5).
- The Mac does not persist the mirror between launches. A snapshot is 53–59 KB (protocol §7), under 0.02 s of the measured link. A disk cache would save nothing and would add a second way to show stale state.

### 2.2 Topics

The topic set is protocol §2 (D18). Phase 3 adds durability choices, the request transport (P11) and one operator topic (stage S5). The QoS column is the node-to-bridge hop. Across the tailnet the transport is TCP.

| topic | dir | message | rate | QoS (protocol §2) | Phase 3 |
|---|---|---|---|---|---|
| `/room_memory/map_bricks` | S→M | `MapBricks` | snapshot at start; deltas ≤ 0.2 Hz, only on a reference change | reliable, keep-last 10 | **volatile**, stated explicitly (§4.3) |
| `/room_memory/events` | S→M | `ChangeEvent` | per transition | reliable, keep-last 100 | **volatile** (§4.3) |
| `/room_memory/status` | S→M | `MemoryStatus` | 1 Hz | reliable, keep-last 1 | transient_local (P2) |
| `/room_memory/epoch` | S→M | `Epoch` | 0.2 Hz | reliable, keep-last 1 | transient_local (P2) |
| `/room_memory/sync/map_bricks` | S→M | `MapBricks` | answers | reliable, keep-last 10 | volatile |
| `/room_memory/sync/events` | S→M | `ChangeEvent` | answers | reliable, keep-last 200 | volatile |
| `/room_memory/request_sync` | M→S | `RequestSync` | on connect and on a gap | reliable, keep-last 10 | carried as CDR bytes over `:8766` and relayed on the Spark (P11); not on the bridge |
| `/room_memory/operator` | M→S | `OperatorAction` (new) | operator clicks | — | new message (P6), same relay (P11) |

**How the bridge sets its own subscription QoS** ([F2]).
- Reliable only if all publishers are reliable.
- Transient_local only if all publishers are transient_local.
- Depth is the publishers' total history, at least `min_qos_depth` (default 1) and at most `max_qos_depth` (default 25).

Each memory topic has one publisher, so the bridge follows the node's choices. In ROS 2 a volatile publisher and a transient_local subscriber do not communicate at all ([R1]). That is one more reason for the node to declare durability explicitly per topic.

### 2.3 Container and node

- **Production node.** `room_memory_cpp` (ROS package `room_memory_cpp`, node name `room_memory`) runs in its own container next to `lidar-dual`, with `--net=host --ipc=host`, as in its Dockerfile. The dedicated bridge runs in the same container image. The Python `room_memory` node stays the test oracle and is not deployed.
- **State directory.** `~/room-memory/<room_id>/` holds `reference-<sensor>.cdr` (a zlib `MapBricks`, written atomically) and the event log.
  - `map_epoch` and `event_id` survive a restart.
  - `event_id` is seeded from the node start time in ms << 20. It stays unique only while the Spark clock does not step back (F11).
- **Ring buffers.** The node keeps its last 10 `map_bricks` and last 100 `events` messages as published bytes, for replay (protocol §5).

### 2.4 Command-center block on `:8766` (P10)

This is the out-of-band identity and health channel. It is already polled every 5 s, and it already obeys "unavailable ≠ zero". Proposed fields:

| field | meaning | missing renders |
|---|---|---|
| `room_id` | the room the node serves | memory mode "not configured" |
| `protocol` | wire-protocol majors the node publishes, e.g. `[1]` | "protocol —"; the Mac does not subscribe |
| `node` | `room_memory_cpp` git sha and image digest; bridge version | "—" |
| `node_started_at` | node start time (Unix s); changes on every restart | "—" |
| `map_epoch`, `map_digest` | current epoch, and SHA-256 of the canonical *uncompressed* snapshot payload at that epoch (protocol §3.1) | "—" |
| `state` | `MemoryStatus.state` name | "—" |
| `publishing` | true when the node's publishers exist | false renders "node down", never "empty room" |

**`map_digest` is an end-to-end consistency check that needs no wire change.** When the mirror's epoch equals `map_epoch`, the Mac hashes its own canonical payload and compares. A mismatch is a protocol violation and forces a snapshot. It is the soak test's main oracle (§7.5).

**`node_started_at`** confirms that a drop in `seq` was a restart (§4.4) and labels diagnostics.

## 3. Bandwidth budget

**Target** (brief §2.5, protocol §7): ≤ 5 KB/s mean in memory mode, and ≤ 2 MB per (re)connect.
- The measured tailnet link is about 3.1 MB/s (`lc-claude13.log`), so 5 KB/s is 0.16 % of it.
- The budget is about being a quiet tenant on lab Wi-Fi, not about capacity.

**Accounting rule.**
- The 5 KB/s covers the memory socket, both directions, at the IP layer, including Tailscale overhead.
- The existing `:8766` poll (~0.4 KB/s measured, ssh included) is accounted separately, because it runs whether memory mode is on or not. A combined figure is reported too.
- Resync requests sent over `:8766` (P11) are a few hundred bytes each. They count against memory mode.

### 3.1 Per-message cost

Message sizes are protocol §7's exact or measured figures, for `room_id` "lab" with default zlib. Wire sizes add an **estimate** of per-packet overhead, which stage S1 must confirm by measurement:

- Foxglove message-data header: 13 B (opcode, subscription id, receive time; `FoxgloveProtocol.messageDataHeaderSize`).
- WebSocket frame header: 2 B up to 125 B of payload, 4 B up to 64 KiB.
- Inner IPv4 + TCP with timestamps: 52 B.
- WireGuard data header + tag: 32 B, plus padding of the inner packet to 16 B.
- Outer IPv4 + UDP: 28 B.
- Inner MTU assumed at 1280 B, which gives 1228 B of TCP payload per segment.
- One TCP ACK from the Mac per small message: 124 B on the wire.

| message | rate | message B | wire B (est.) | down B/s | up B/s (ACKs) |
|---|---|---|---|---|---|
| `MemoryStatus` | 1 Hz | 68 (exact) | 204 | 204 | 124 |
| `Epoch` | 0.2 Hz | 53 (exact) | 188 | 38 | 25 |
| `ChangeEvent`, no mask | per transition | 136 (exact) | 268 | — | — |
| `ChangeEvent`, 25 cm box mask | per transition | 253 (measured) | 396 | — | — |
| `MapBricks` delta, 25 cm box | on a reference change | 367 (measured) | 508 | — | — |
| `MapBricks` snapshot | connect / resync | 52,863–59,024 (measured) | 57,808–64,540 (44–49 segments) | — | — |

Small messages pay 2–3× their size in headers, which is why §3.2 counts wire bytes.

### 3.2 Scenarios

| scenario | down | up | notes |
|---|---|---|---|
| **Steady, static room** (status + epoch only) | 0.24 KB/s | 0.15 KB/s | ~0.39 KB/s total; 13× under budget. With the `:8766` poll, ~0.8 KB/s |
| **Quiet day** (60 changes/h; each 3 transitions with mask; each accepted with a 0.4 KB delta) | +0.04 KB/s | +0.01 KB/s | events and deltas are noise next to status |
| **People walking through** (≈ 2 CANDIDATE/EXPIRED transitions per second while someone moves; no mask on CANDIDATE, P5) | +0.54 KB/s | +0.25 KB/s | ~1.2 KB/s total while active. Person-sized masks on CANDIDATE (est. 0.5–1 KB each) would make it 2–3 KB/s, hence P5 |
| **One (re)connect** | ~65 KB snapshot + ≤ ~48 KB event fallback (≤ 120 messages × 396 B) + advertise | ~6 KB ACKs, < 1 KB requests | ~110–120 KB; 6 % of the 2 MB cap |
| **Re-reference or large accept** (the whole map changes) | one snapshot, 58–65 KB | — | P7: snapshot instead of a delta larger than half a snapshot |
| **Resync storm at protocol §5's limit** (one request per topic per 5 s, unanswerable by replay) | 12.9 KB/s map + 9.5 KB/s events ≈ 22 KB/s | ~0.2 KB/s | **4.5× over budget**; P3 and P4 bound it |

**Advertise burst: unmeasured.**
- On connect the bridge sends `serverInfo` and one `advertise` listing each whitelisted channel with its full message definition ([F2]).
- On the dedicated bridge that is six channels. `ChangeEvent`'s definition includes `MapBricks` and `builtin_interfaces/Time`.
- The estimate is under 10 KB. Stage S1 measures it.

**Amortized reconnect limit.** At 0.39 KB/s steady and about 115 KB per reconnect, the 5 KB/s mean holds only if reconnects average at most one per ~25 s. Hence P3's floor on the reconnect delay.

### 3.3 Spark-side shaping (PR 2.4)

- **Delta coalescing.** Deltas are capped at 0.2 Hz (protocol §2). Reference changes within one 5 s slot merge into one delta. That is legal: a delta's `map_epoch` must only exceed its `base_epoch` (D8), so epochs may skip.
- **Byte bucket on `map_bricks`.** At most 180 KB in any 60 s (3 KB/s). A delta that would break it is held and coalesced, never dropped, since dropping would break the `seq` chain. Snapshot answers count against the same bucket.
- **No mask on CANDIDATE (P5).** The protocol already allows `mask` count 0, which renders "no mask" from `aabb`. CONFIRMED and later transitions carry the mask.
- **Snapshot answers are capped (P4).** Every Mac on the shared sync topic receives every answer (protocol §5), so N Macs multiply the cost of each one.

## 4. Sync protocol in operation

Protocol §3 and §5 define the messages and the resync rules, and they are not restated here. This section is the operational layer Phase 3 adds: how a connection starts, how it recovers, and what each side rate-limits.

### 4.1 Prior art and where this design sits

| system | how a map reaches a remote client | what a late joiner gets | gap detection |
|---|---|---|---|
| octomap_server ([M1]) | the whole octree, serialized on every publish (`octomap_binary`, `octomap_full`), latched with transient_local | the latest full map | none needed; every message is complete |
| voxblox ([M2]) | `voxblox_msgs/Layer` with `ACTION_UPDATE`, `ACTION_MERGE` and `ACTION_RESET`; only updated blocks unless resetting | a reset with every block: the server "always reset[s] the remote map and send[s] all when a new subscriber subscribes", because that is "better than inconsistent map states" | none in the message |
| nvblox ([M3]) | `Mesh` / `VoxelBlockLayer` with `block_indices` and a `clear` flag, set "when the *entire* map rather than only the pieces that changed are published" | the next message with `clear` set | none in the message |
| MCAP ([B2]) | (storage, not transport) | — | each message record carries a `sequence`, an "optional counter for detecting gaps in messages" |
| **this design** | voxblox/nvblox-style brick deltas (`KIND_DELTA`) and full snapshots (`KIND_SNAPSHOT`, the counterpart of RESET or `clear`) | one snapshot on request, or a replay of the missed deltas | `seq` per topic, plus `base_epoch` → `map_epoch` chaining, plus a periodic epoch cross-check in `MemoryStatus`, plus `map_digest` (P10) |

The part this design adds is the explicit chain:
- A delta names the exact epoch it applies to, so a client can *prove* its mirror is current or know that it is not.
- That is what lets the Mac keep a greyed last-good map while resyncing, instead of resetting on every hiccup.
- The cost is the state machine in §5.

### 4.2 Recap of the rules Phase 3 relies on

- **`seq`.** +1 per message per topic per publisher, from 0. A node restart resets it.
- **`map_epoch`.** It only grows along a delta chain. A delta applies only on `base_epoch` = the mirror's epoch and `seq` = last + 1 (protocol §3.1).
- **Duplicates.** A duplicate is ignored. Anything else that does not chain makes the map STALE (protocol §5).
- **STALE.** While STALE, the Mac keeps its last good mirror greyed and labelled "resyncing", and never shows it as current. This is protocol §5's rule. It replaces the brief's older "discard local state".
- **Lost trailing delta.** `MemoryStatus.map_epoch` ≠ the mirror's epoch on 2 consecutive status messages catches it (D20).

### 4.3 Connect sequence: one mechanism, not two

**Mac.**
1. Open the socket. Read `serverInfo` and `advertise`.
2. Check the schema fingerprint of each memory channel (§4.10). On a mismatch, stop at INCOMPATIBLE.
3. Subscribe to all six S→M topics in one `subscribe` message.
   - The sync subscriptions stay up for the whole connection.
   - A subscription made only when a request goes out could miss its answer, because the Mac cannot observe when the bridge's ROS subscription has matched.
4. Send `RequestSync{TOPIC_MAP_BRICKS, SINCE_NONE}` and `RequestSync{TOPIC_EVENTS, SINCE_NONE}` through `:8766` (P11), **immediately** (P8).
   - Protocol §5 says to wait up to 5 s for the Spark's own snapshot first.
   - The node cannot see a connect through a bridge: the bridge holds one ROS subscription per channel for all clients, created by the first ([F2]). So only the first client ever triggers the node's "matched" event ([R1]).
5. The map is CURRENT when protocol §5's condition holds.

**Why `map_bricks` and `events` stay volatile.**
- For transient_local topics, the bridge caches the last N messages per publisher, with N the publisher's depth. It replays them when a later client subscribes ([F2]).
- The replay is logged to the WebSocket server's sink, not to one connection. Whether clients that are already subscribed receive it too is not settled (V2).
- If they do, a cached keyframe snapshot would reach a Mac that is already CURRENT. Protocol §5's "a snapshot is always accepted" would then rewind that Mac to an older epoch, and show it as current. That is the case the governing invariant forbids.
- A latched burst plus a request answer would also make each connect pay for the map twice.

So the request is the one connect mechanism.

**Status and epoch (P2).** `status` and `epoch` are full-state messages with depth 1. A duplicate is harmless: gaps on those topics are only counted (protocol §5), and D20's epoch check needs two consecutive disagreeing messages. They are therefore transient_local, so a new client shows the memory state on connect instead of after up to 5 s.

**No request path (P1, fallback only).** If `:8766` is down and no request can go out, a connecting Mac gets a map only when the node publishes a snapshot on the live topic.
- For that case the node publishes a live snapshot every 60 s. The cost is ~1.1 KB/s.
- The live topic stays volatile, and TCP keeps it in order, so a periodic snapshot cannot rewind anyone.
- It is off whenever the relay works (Q4).

### 4.4 `RequestSync` (D19) in operation

**Transport (P11).**
- The Mac POSTs the `RequestSync` CDR bytes, unchanged, to the command-center API through the existing ssh tunnel.
- `command_center.py` publishes them on `/room_memory/request_sync` on the Spark.
- The answer comes back on the bridge's sync topics, as protocol §5 specifies.

Compared with the bridge's `clientPublish`:
- The relay keeps the bridge read-only.
- Only holders of an authorized ssh key can reach it (§8).

**Feasibility.** Publishing a `labcommand_msgs` type needs that type in `command_center.py`'s environment. If the API runs outside the ROS environment, it forwards the bytes instead to a localhost-only endpoint in the node, which publishes them. Either way the bytes are unchanged.

`OperatorAction` (P6) uses the same relay.

**Answers** follow protocol §5:
- **Replay**, if the requested `seq` is in the ring buffer and the epoch matches.
- Otherwise a **snapshot** for maps, or the **bounded fallback** for events.

**Validation on the Spark.** A request is dropped and logged unless all of these hold:
- `room_id` is the served room.
- `map_epoch` = `since_epoch`.
- `SINCE_NONE` appears in both `since` fields or in neither.

This is symmetric with the Mac's validation (D16).

**Rate limits.**
- **Mac (P3).** At most one request per topic per 5 s (protocol §5). While consecutive requests end without CURRENT, the interval backs off 5 → 10 → 20 → 30 s.
- **Spark (P4).** At most one *snapshot* answer per topic per 15 s across all clients. A request inside the window is served in the next slot. Replays (≤ 9 deltas, or ≤ 100 events) are not capped.
- **Together** these bound a storm at 65 KB + 48 KB per 15 s, about 7.5 KB/s.
  - That is still over 5 KB/s while the storm lasts.
  - The hourly mean stays in budget unless there are more than ~40 storms an hour.
  - The soak (§7.5) reports both figures.

**Node restart (P13).** After a restart the node's `seq` starts again at 0. Two protocol §5 rules then collide:
- A snapshot "is always accepted".
- "A sync message with `seq` ≤ the last applied is ignored".

Without a tie-break, a Mac whose baseline is the old, higher `seq` could ignore every answer and stay STALE forever. P13 resolves it:
- A **snapshot** answering an outstanding request is applied whatever its `seq`, and it sets the baseline.
- An **events answer whose first message is not `since_seq` + 1** is a fallback, not a replay. It resets the events baseline before it is applied.
- A `seq` that goes *down* on live `map_bricks` or `events` is a new publisher generation. Those two topics are volatile, and TCP keeps one connection in order, so nothing else can lower their `seq`. The Mac treats it as a first connect for that topic. `node_started_at` (P10) confirms the restart in diagnostics.
- On `status` and `epoch`, a lower `seq` is only counted. Under P2 those topics are transient_local, and a bridge cache replay (V2) can deliver an older message to a Mac that is already connected.

### 4.5 Event history across a disconnect

- The Spark keeps 100 event messages. Older history reaches a reconnecting Mac only through the bounded fallback: the latest message of each open event, then the 100 kept (protocol §5).
- Older closed history is not resent. The Mac marks it "incomplete since \<stamp\>" and never invents transitions.
- A full history read is not a streaming concern. If it is wanted (Q8), it belongs on `:8766` as a paged HTTP read of the event log, outside the 5 KB/s, on demand.

### 4.6 Compression (D9)

- **Format.** Every `MapBricks` is raw DEFLATE by default (`COMPRESSION_ZLIB` = 1). That covers deltas, snapshots and event masks.
- **Both ends match.** The C++ node writes `deflateInit2(6, Z_DEFLATED, −15, …)`. A negative `windowBits` is raw deflate with "no zlib header or trailer" ([Z1]). The Mac reads it with Foundation's `.zlib`, which Apple documents as "raw DEFLATE format as described in IETF RFC 1951", equivalent to `deflateInit2(zstream,5,Z_DEFLATED,-15,8,Z_DEFAULT_STRATEGY)` ([Z2]).
- **Effect.** A 547 KB raw snapshot sends as 59 KB (protocol §7).
- **LZFSE** (2) is decoded but never produced on the Spark.
- **Zip-bomb guard.** Decoders reject a declared payload over 16 MiB before inflating (D22).

### 4.7 All-free bricks (D5)

- An absent brick is UNKNOWN. An all-free brick is sent, with `known` full and `occupied` empty. `removed_bricks_idx` means "became all-unknown" only.
- **Live consequences.**
  - A brick goes from free to UNKNOWN only by an explicit removal in a delta, never by omission.
  - A Mac that misses a delta cannot drift into drawing unknown space as free. It goes STALE instead.
- **Cost.** +5.6 KB per compressed snapshot (protocol §7).

### 4.8 Reconnect and resync scenarios

| event | what the Mac sees | recovery | wrong-current possible? |
|---|---|---|---|
| Wi-Fi blip shorter than the TCP timeout | late messages, nothing lost | none | no |
| Socket drop and reconnect; node did not restart | first live `map_bricks` may be a delta → STALE | request with `since = (epoch, seq)` → replay if ≤ 9 deltas behind, else snapshot | no |
| Dedicated bridge restarts; node keeps running | close; on return `seq` continues | as above; replay usually succeeds | no |
| Node restarts; bridge keeps running | `seq` drops on live topics | P13: first-connect handling; snapshot answer | no |
| Node restarted while the Mac was away | live `seq` lower than the baseline; the request's `since_seq` is newer than the node's latest | snapshot (protocol §5), applied under P13 | no |
| Bridge backlog overflow (> 1024 queued messages for this client) | the oldest messages are dropped → `seq` gap | replay or snapshot; overflow counted (§7.5) | no |
| Mac sleep/wake | TCP looks open; nothing arrives | status stale at 3 s; socket closed and redialled after 10 s of silence | no |
| Lost trailing delta | nothing until status disagrees | `MemoryStatus.map_epoch` ≠ mirror twice → STALE → request (D20) | no |
| Mac decode backlog | messages queue in the app | decode off the main thread, as LIVE does; freshness uses the Mac's receive time, so a backlog shows as stale | no |
| A second Mac connects while the first is CURRENT | snapshot answers on the shared sync topic | the first Mac has no request outstanding and ignores them (protocol §5) | no |

§7 tests the last column. Every row must stay "no" under fault injection.

### 4.9 Rig moved

The rig-pose guard is PR 2.3 and not built yet. Phase 3 defines what crosses the wire and what the Mac does.

**Spark.**
- TRACKING → RIG_MOVED when the ICP residual or the pose delta passes its threshold (protocol §4.1).
- Open CANDIDATEs go EXPIRED. CONFIRMED events keep their state, and no new candidates start.
- `MemoryStatus.pose_residual_m` carries the residual. NaN means "guard not run", never 0 m.

**Mac.**
- Red banner "Rig moved: changes suspended", with the residual.
- Current-window overlays are suppressed (protocol §6 rule 8).
- The mirror stays CURRENT, because the *reference* has not changed.

**Operator.** Two ways out:
1. **Put the rig back.** The guard passes, and the node returns to TRACKING with no epoch bump.
2. **Re-reference.** RIG_MOVED → BUILDING → TRACKING with a new reference. `map_epoch` bumps, and the node sends a **snapshot**, not a delta (P7).

**Coordinates.**
- In v1 the room frame is the sensor frame: the node keeps points in each sensor's frame, origin 0. So a moved rig moves the frame, and pre-move events are in the old frame.
- **P9:** re-referencing after RIG_MOVED starts a new `room_id` generation (`lab` → `lab~2`), unless the node aligns the new reference to the old one within the guard's threshold.
- A new `room_id` makes the existing rules do the right thing (protocol §1.4, §5): old messages are ignored, and the new room starts from a snapshot.
- The Mac keeps the old room's history read-only.
- Q1 decides whether this is needed.

### 4.10 Versioning

XCDR1 has no extensibility. Decoders reject 4 or more unread trailing bytes and unknown enum values (protocol §1.1, D3). So **every field or enum change is a breaking change**, and the scheme is built around that.

1. **Schema fingerprint at advertise time** (no wire cost).
   - Each advertised channel carries `schemaName` (e.g. `labcommand_msgs/msg/MapBricks`), `schemaEncoding` and the definition text ([F2]).
   - The Mac carries SHA-256 fingerprints of the exact text the pinned bridge emits for the definitions it was built against. A stage-S1 fixture records that text.
   - On a mismatch the Mac subscribes to nothing on that topic and shows "Memory protocol mismatch (Spark vX, app vY)". It never tries to decode.
   - This needs `FoxgloveProtocol.Channel` to keep `schemaEncoding` and `schema`.
2. **Major version in the topic namespace.**
   - v1 is protocol §2's bare `/room_memory/*`. A breaking v2 publishes on `/room_memory/v2/*`.
   - During a deprecation window the node publishes both majors from one core. The cost is ~0.24 KB/s more status, plus a second snapshot per connect for a Mac that subscribes to both.
   - The Mac subscribes to the highest major it supports that is both advertised and listed in `:8766`'s `protocol`.
3. **Fail closed.** An unknown enum or an extra field rejects the message (protocol §1.5). Rejections are counted. Any rejection in a soak fails the stage.
4. **Build identity.** `:8766` reports the node's git sha, image digest and bridge version. The Mac logs them with every protocol violation, so a report names the exact pair of builds.

### 4.11 Operator actions (stage S5)

Accept, revert, mark ground truth, pause, resume and re-reference change state on the Spark, and they are not in v1. P6 proposes one message for them:

```
# labcommand_msgs/OperatorAction (proposal P6)
uint8 ACTION_ACCEPT=0
uint8 ACTION_REVERT=1
uint8 ACTION_MARK_GROUND_TRUTH=2
uint8 ACTION_PAUSE=3
uint8 ACTION_RESUME=4
uint8 ACTION_REREFERENCE=5
uint64 EVENT_NONE=18446744073709551615
string room_id
uint64 map_epoch        # the epoch the operator saw; compare-and-set
uint64 seq              # this client's action counter
builtin_interfaces/Time stamp
uint8 action
uint64 event_id         # EVENT_NONE for room-level actions
uint8 expected_state    # the event state the operator saw; ACCEPT and REVERT require CONFIRMED
string action_id        # client-generated UUID; the Spark ignores a repeat
```

- **No reply message.** The answer is what the Spark already publishes: the event's next transition, a `map_epoch` bump with a delta, or a status change.
- **Compare-and-set.** An action with an out-of-date `map_epoch` or `expected_state` is ignored and logged. A double click or a retried POST cannot accept twice, and an action never applies to a map the operator did not see.
- **Audit log.** The Spark appends every action with its `action_id`, arrival time and outcome. Attribution is per action, not per user: behind an ssh local forward, the relay sees a loopback connection and cannot tell which key opened the tunnel.

## 5. Mac-side mirror state machine

Three pieces share the work:

| piece | target | role |
|---|---|---|
| `MemoryClient` | app | owns the socket and the `:8766` POSTs |
| `MemoryMirror` | LabCommandCore, pure state like `LiveSession` | owns every decision: it takes events (socket opened or closed, advertise, message bytes, POST result, clock tick) and returns actions |
| `RoomMemoryStore` (PR 1.1) | LabCommandCore | holds the bricks and events, and enforces protocol §3.1's apply rules |

Everything below is in `MemoryMirror`, so it is testable without a socket.

### 5.1 Connection

| state | entered when | leaves to |
|---|---|---|
| OFF | launch (Q6); operator turns memory mode off | CONNECTING on operator "on" |
| CONNECTING | memory mode on; after a drop, once the backoff wait ends | SUBSCRIBED when all four state topics are advertised with matching fingerprints; INCOMPATIBLE on a mismatch; UNAVAILABLE when the topics are absent for 10 s |
| UNAVAILABLE | bridge reachable, memory topics absent | SUBSCRIBED when advertised. Renders "Room memory not running on Spark", never an empty room |
| INCOMPATIBLE | fingerprint mismatch | OFF only. Renders both versions |
| SUBSCRIBED | subscriptions and requests sent (§4.3) | CONNECTING on socket close or 10 s without any message, after the backoff |

**Reconnect backoff (P3).** It reuses `StreamHostRotation`, with a 5 s floor instead of 1 s. A connection that lasted under 60 s does not reset the failure count. That enforces §3.2's limit of one reconnect per ~25 s even on a flapping link.

### 5.2 Map (inside SUBSCRIBED)

```
             snapshot                         delta chains (seq+1, base = epoch)
  EMPTY ──────────────────▶ CURRENT ◀──────────────────────────────────┐
    │                          │  ─────────────────────────────────────┘
    │ delta first after connect│ seq gap · wrong base · delta first after connect ·
    ▼                          │ status epoch ≠ mirror ×2 · map_digest mismatch · map_bricks seq goes down (P13)
  STALE(no mirror) ◀───────────┴──────────────▶ STALE(last good mirror, greyed)
    │  RequestSync since=NONE            │  RequestSync since=(epoch, seq); P3 backoff
    └──────── snapshot ─────▶ CURRENT ◀──┴── snapshot (P13), or a replay that chains to the newest live seq
```

| state | renders |
|---|---|
| EMPTY | "Waiting for map" |
| CURRENT | the map |
| STALE with a mirror | the last good map greyed: "resyncing · as of epoch N, \<age\>" |
| STALE without one | "Waiting for map" |

Change overlays are drawn only in CURRENT.

### 5.3 Events

| state | entered when | renders |
|---|---|---|
| COMPLETE | contiguous `seq` since this connection's first message or since a replay that closed a gap | normal |
| INCOMPLETE(since t) | a gap, or a fallback answer (protocol §5) | history "incomplete since t"; open events "may be outdated" until their next transition |

### 5.4 Composite rendering

The Spark's memory state (protocol §4.1) is shown only while `status` is fresh. A stale status renders "STALE · \<age\>", never the last state as live (protocol §6 rule 6).

When several conditions hold at once, the banner shows the first that applies:

1. INCOMPATIBLE
2. UNAVAILABLE
3. connection lost
4. status stale
5. RIG_MOVED
6. map STALE
7. DEGRADED
8. PAUSED
9. BUILDING

### 5.5 Invariants (each one is a test, §7)

1. The map renders as current only in CURRENT, and only while `status` is fresh.
2. In CURRENT, the mirror's bricks equal the Spark's reference at the mirror's epoch. `map_digest` checks this when `:8766` reports it.
3. No message is partly applied (protocol §6 rule 9).
4. Only CONFIRMED renders as a change (protocol §6 rule 7).
5. An event's state never goes backwards in protocol §4.2's order. A violation keeps the last legal state (protocol §4).
6. The mirror's memory is bounded:
   - one map, ≤ 16 MiB raw (D22);
   - an event list capped at 10,000 events, with older ones dropped and the history marked incomplete.

## 6. Failure modes

| id | failure | detection | on screen | recovery |
|---|---|---|---|---|
| F1 | Tailscale path falls back from direct to a DERP relay, "a last resort" ([T3]) | higher RTT; no protocol signal | none at 5 KB/s; a snapshot takes longer | none needed; soak logs the path type (`tailscale status`) |
| F2 | Tailnet down or Spark offline | socket and `:8766` fail | "connection lost"; map greyed STALE | P3 backoff |
| F3 | Dedicated bridge crashed or restarted | socket close | as F2, for seconds | §4.8 row 3 |
| F4 | `room_memory_cpp` crashed or restarted | topics unadvertised (UNAVAILABLE), then `seq` drops | "Room memory not running on Spark", then STALE → snapshot | P13; reference and epoch reload from disk |
| F5 | Bridge backlog overflow on the memory socket | `seq` gap | STALE briefly | replay or snapshot. At ~1.2 messages/s, 1024 queued messages means ~14 min of the Mac not reading, so recurrence means a stuck client, not load |
| F6 | Lost trailing delta | status epoch mismatch ×2 (D20) | STALE within 2 s | request |
| F7 | Malformed or out-of-contract message | decoder throws | diagnostics count; STALE if it was on `map_bricks` | resync. Any non-zero count in a soak fails the stage |
| F8 | Protocol major mismatch after an update | fingerprint (§4.10) | INCOMPATIBLE, both versions shown | update the app, or the node publishes both majors |
| F9 | Rig bumped | guard → RIG_MOVED | red banner, overlays suppressed | §4.9 |
| F10 | One sensor silent | `per_sensor_hz` 0 → DEGRADED | amber banner naming the sensor | Trevor restarts that driver; events continue from the other sensor (D17) |
| F11 | Spark clock steps back | possible `event_id` seed collision after a restart (§2.3) | an `event_id` reused with another `kind`: protocol violation | the node refuses to start if its clock is earlier than the newest `first_seen` in its event log (PR 2.4) |
| F12 | Spark disk full | reference save fails | not visible today | proposed: DEGRADED with a reason in the `:8766` block (Q9); the node keeps serving from memory |
| F13 | Resync storm (flapping link, many Macs) | request counter; bytes per minute | slow recovery | P3, P4 |
| F14 | `:8766` relay down | POST fails | map stays STALE; "cannot request resync" | P1 fallback snapshots; ssh tunnel restart (existing `RigService` path) |
| F15 | Large accept or re-reference | delta over half a snapshot | none | snapshot instead (P7) |
| F16 | Two Macs act on one event | compare-and-set fails for the second | the second sees the first's outcome | P6 |
| F17 | Wrong room | `room_id` ≠ `:8766` `room_id` | messages ignored (protocol §1.4); "room mismatch" diagnostic | fix the node's `room_id` parameter |

## 7. Test plan

### 7.1 Fixtures

| fixture | source | used by |
|---|---|---|
| 12 hand CDR goldens | `Tests/Fixtures/room/golden/hand/` (protocol §9) | codec tests (exist) |
| Spark CDR goldens | `scripts/spark/make_room_memory_goldens.py` with `rclpy` in the Jazzy container (Trevor runs it) | confirm D4; gate for S3 |
| Session traces | the C++ `tools/replay.cpp` (or the Python oracle) runs scripted `RoomMemory` sessions over the Phase 0 scenes: box added, removed and moved; walk-through; rig shifted. It writes every published message with topic, `seq` and bytes, as length-prefixed records under `Tests/Fixtures/room/sessions/` | `MemoryMirror` replay and fault tests |
| Bridge transcript | `serverInfo`, `advertise` and a few binary frames from the pinned 3.4.1 bridge, captured at S1 | fingerprint constants, advertise parsing, byte accounting |
| Lidar MCAP bags | `ros2 bag record` of both lidar topics on the rig, including the 12 Oct box takes (brief Q4). MCAP is the default storage on Jazzy ([B1]) | S1/S2 replay with `ros2 bag play` |

### 7.2 Mac unit and property tests (`LabCommandCoreTestsRunner`)

- **Table tests.** One per row of §4.8, and each §5 transition.
- **Fault-injection property test.**
  - Take a session trace.
  - Apply a seeded random schedule of: drop; duplicate; reorder within 10; truncate one message; reconnect with `seq` continuing; node restart (`seq` reset, same epoch); epoch jump; wrong `room_id`; fallback answers; snapshot answers meant for another Mac; a relay that fails.
  - Run 10,000 schedules, and assert §5.5's invariants 1–5 after every step.
  - Once faults stop, assert the mirror reaches CURRENT within a bounded number of requests.
  - The oracle is the trace's own state at each `seq`.
- **Budget test.** Replay a 1 h trace at real time through a client-side byte counter that applies §3.1's overhead model. The mean must be ≤ 5 KB/s, and a reconnect ≤ 2 MB.

### 7.3 Bench stack without the Spark (stage S1)

The Spark side runs in an arm64 `ros:jazzy` container on a developer machine:
- `room_memory_cpp`;
- the dedicated bridge pinned at 3.4.1;
- a stub of the `:8766` relay;
- `ros2 bag play` of recorded MCAP ([B1]).

LabCommand connects to it by IP. It is the same code path as the tailnet, minus WireGuard.

**Fault injection.**
- `tc netem` in the container shapes the link: delay 50–300 ms, loss 1–5 %, rate 64 kbit/s.
- Scripted faults: kill the bridge; kill the node; `docker pause` (a hung Spark); restart the node with its state directory; restart it without one (`map_epoch` restarts).

### 7.4 On the rig (stages S2–S6, Trevor operates the Spark)

- **Static room, 1 h.** Count false CONFIRMED events per hour (Q3).
- **Scripted session** (brief PR 3.2): 5, 10 and 25 cm objects placed, removed and moved at logged times; walk-throughs; one rig nudge.
- **Bridge kill and node kill mid-session.** STALE within 5 s, then CURRENT again with no wrong frame (brief PR 3.1).

### 7.5 Soak

24 h at S3, then 7 days at S6, with the Mac on lab Wi-Fi.

| metric | how | pass |
|---|---|---|
| mean memory-socket bytes/s, both directions | nettop per connection (the `lc-claude13.log` method, summed per connection, since nettop drops closed ones) | ≤ 5 KB/s every hour |
| bytes per (re)connect | same, windowed on connects | ≤ 2 MB; report the median |
| wrong-current | hourly `map_digest` comparison (§2.4); every mismatch logged with both build ids | 0 |
| protocol violations and rejected messages | Mac diagnostics | 0 |
| reconnects, resyncs, replays vs snapshot answers | Mac and Spark logs | reported; replays should dominate |
| time STALE | Mac log | reported; < 1 % of connected time |
| Mac resident memory | `footprint`, hourly | flat after the first hour |
| node window time | node log | p99 < 200 ms per 1 s window |
| false CONFIRMED events/hour in static periods | event log against ground-truth marks | ≤ Trevor's bar (Q3) |
| Tailscale path | `tailscale status`, hourly | reported (direct vs DERP) |

## 8. Security

**Threat model.** The Spark is reachable only on the tailnet, and, for any port bound to `0.0.0.0`, on the lab LAN. The assets are:
- the lab's 3D geometry;
- the integrity of the reference map and the event log;
- the Spark itself.

The realistic risks are another device writing to the bridge, and a malformed message crashing either end.

- **Read-only bridge.**
  - The dedicated bridge runs with `capabilities: []`.
  - Clients can subscribe; they cannot publish, call services, set parameters or fetch assets.
  - This matters because `client_topic_whitelist` is documented but not enforced at 3.4.1 or on `main` ([F2]). If `clientPublish` were on, the only control would be who can reach the port.
- **Writes go over ssh (P11).** `RequestSync` and `OperatorAction` travel over the existing ssh tunnel to `:8766`, so only holders of an authorized ssh key can reach the relay. The relay cannot tell which key opened the tunnel, so the action log attributes by `action_id`, not by user.
- **Reachability.**
  - The bridge's `address` is the Spark's tailnet IP ([F1]).
  - A Tailscale grant allows `tcp:8767` on the Spark only from Trevor's Mac or a `tag:labcommand` device. Grants are deny-by-default ([T1]).
  - Both are Trevor's to set. Both are required before S3.
- **Encryption.**
  - WireGuard covers the whole path, and `ws://` inside it matches LIVE.
  - `wss://` with `tailscale cert` would need the Mac to dial the MagicDNS name rather than the IP literal. The certificate's name would also appear in public Certificate Transparency logs ([T2]).
  - It is not proposed for v1.
- **Input validation.**
  - Both ends validate symmetrically (D16).
  - The Mac checks the 16 MiB limit before inflating (D22).
  - The relay and the node validate `RequestSync` and `OperatorAction` (`room_id`, epoch, enums), and the node rate-limits them (P4).
  - Codex's review of the decoders includes a fuzz pass.
- **State changes are safe to repeat.** `OperatorAction` is compare-and-set, and idempotent by `action_id` (§4.11).
- **Observation for Trevor; no change proposed here.**
  - The bridge's defaults are `address` `0.0.0.0` and `capabilities` `[clientPublish, parameters, parametersSubscribe, services, connectionGraph, assets]` ([F1]).
  - If the `:8765` bridge runs with defaults, any device on the lab LAN or the tailnet can publish on any topic, including the lidar topics, and can set parameters.
  - Its actual configuration is not visible from the Mac (Q5).
- **Supply chain.**
  - The node and the dedicated bridge run from images pinned by digest.
  - Versions are reported in `:8766`.
  - A bridge upgrade reruns S1 and the fingerprint fixture.

## 9. Staged rollout

Each stage has an owner, go/no-go criteria and a rollback.
- Stages S2 onward change the Spark, so Trevor performs them or authorizes an agent for that session.
- No stage changes the `:8765` bridge, SNAPSHOT or LIVE.

**Verification items at S1**, against the pinned bridge:

| id | question | why it matters |
|---|---|---|
| V1 | Does the relay path (`:8766` → `/room_memory/request_sync`) deliver the CDR bytes intact, and how long is the round trip? | P11 |
| V2 | Does the bridge's transient_local replay reach only the new client, or every client on the sink? | duplicate `status`/`epoch` messages on CURRENT Macs (harmless, but counted); confirms §4.3's reason for volatile maps |
| V3 | Message-backlog behaviour when the Mac stops reading: oldest dropped, then a `seq` gap and resync | F5 |
| V4 | Advertise and `serverInfo` bytes per connect on the dedicated bridge | §3.2 |
| V5 | On a test bridge with `clientPublish` on and a restrictive `client_topic_whitelist`, does a publish to a topic outside the whitelist arrive? | confirms the source read behind §8. P11 stands either way |

| stage | what | owner | go | no-go / rollback |
|---|---|---|---|---|
| **S0** Mac only | `RoomMemoryStore` (PR 1.1), `MemoryMirror`, session traces, §7.2 tests; memory mode behind a launch flag, off by default | Claude Code; Codex review | all §7.2 tests pass; 10,000 fault schedules with 0 invariant violations | flag stays off; nothing ships |
| **S1** bench stack | §7.3 on a dev machine; V1–V5 answered and written into this doc | Claude Code | V1–V5 answered; budget test passes with measured overheads; every §4.8 row recovers | if the relay is not acceptable (Q4), choose between `clientPublish` with grant-only control and P1-only operation, and re-review §8. If P12 is rejected, the bench uses the `:8765` bridge's image rebuilt with `labcommand_msgs` instead |
| **S2** Spark shadow | PR 2.3 and 2.4 merged; room-memory container running `room_memory_cpp` only, no bridge; memory topics recorded to MCAP | Trevor deploys | 24 h with no crash; p99 window < 200 ms; static-room false CONFIRMED ≤ Q3 bar; rig nudge → RIG_MOVED and no events; Spark goldens decode on the Mac | `docker stop` the memory container; the lidar stack is untouched |
| **S3** read-only mirror | the dedicated bridge starts in the memory container (`address`, whitelist, `capabilities: []`); Tailscale grant for 8767; P1 fallback snapshots on (no relay yet); Mac connects with the flag on | Trevor (Spark, grant), Claude Code (app) | 24 h soak (§7.5): 0 wrong-current, 0 violations, ≤ 5 KB/s every hour; bridge kill → STALE ≤ 5 s → CURRENT within one P1 period (≤ 60 s) with no wrong frame | stop the dedicated bridge (the node keeps running); app flag off. If P12 is rejected: the `:8765` bridge image is rebuilt with `labcommand_msgs` and the memory topics are added to its whitelist, a scheduled bridge restart that interrupts LIVE; rollback is the previous pinned image |
| **S4** resync requests | `:8766` relay (P11) live; P1 turned off | Trevor (`command_center.py`), Claude Code | a forced gap recovers by replay; a forced 2-minute outage recovers by one snapshot; storm test within P3/P4 bounds | disable the relay endpoint; P1 back on; the Mac behaves as at S3 |
| **S5** operator actions | P6 accepted; `OperatorAction` `.msg`, codecs, goldens; accept/revert in the app | Claude Code; Codex | accept bumps `map_epoch` on both ends and arrives as one delta; double-click is a no-op; a stale accept is refused and logged | disable the action endpoint; the app hides the buttons |
| **S6** soak and product bar | 7-day soak; PR 3.2 scripted sessions | Trevor, Claude Code | §7.5 passes all week; PR 3.2 bar met (Q2, Q3) | stay at S5 and fix; memory mode stays opt-in |

**Spark data rollback.**
- Before a re-reference, the node renames the current `reference-*.cdr` to `reference-*.<epoch>.cdr`, keeping the last 5.
- Restoring one is a rename and a node restart. The restart's `seq` reset makes every Mac resync (P13).

## 10. Proposed amendments (for Trevor and Codex)

None of these is in effect until accepted. Each would change `ROOM-MEMORY-PROTOCOL.md`, `command_center.py` or the deployment in its own PR.

| id | change | where | why |
|---|---|---|---|
| P1 | Fallback only: when no request path exists, a live `map_bricks` snapshot every 60 s | protocol §2, §5 | a connecting Mac otherwise gets no map; ~1.1 KB/s, and only while the relay is unavailable |
| P2 | `status` and `epoch` durability transient_local | protocol §2 (D18) | memory state on connect; duplicates are harmless on full-state topics |
| P3 | Mac: memory-socket reconnect floor 5 s, failure count not reset by connections under 60 s; request backoff 5 → 30 s | protocol §5 | bounds reconnect and request storms (§3.2) |
| P4 | Spark: ≤ 1 snapshot answer per topic per 15 s across clients; request inputs rate-limited | protocol §5 | the sync topics are shared, so answers multiply with clients |
| P5 | No `mask` on CANDIDATE transitions | protocol §3.2 (allowed; this makes it the rule) | walk-throughs would otherwise cost 2–3 KB/s |
| P6 | New `OperatorAction` message and `/room_memory/operator` topic (§4.11) | protocol §3.6 (new) | accept, revert, ground truth, pause, re-reference |
| P7 | A snapshot instead of a delta whose payload would exceed half a snapshot's | protocol §3.1 | a large delta costs more than a snapshot and cannot be replayed cheaply |
| P8 | The Mac requests right after subscribing, without the 5 s wait | protocol §5 | the node cannot see a later client's connect through a shared bridge subscription (§4.3) |
| P9 | Re-referencing after RIG_MOVED without alignment starts a new `room_id` generation | protocol §1.2, §4.1 | pre-move events are in a different frame (§4.9) |
| P10 | `:8766` `room_memory` block with `node_started_at` and `map_digest` (§2.4) | `command_center.py` | identity, versions and an end-to-end consistency check at no stream cost |
| P11 | `RequestSync` and `OperatorAction` carried over `:8766` and relayed on the Spark, instead of bridge `clientPublish` | protocol D19, `command_center.py` | `client_topic_whitelist` is not enforced, so the bridge stays read-only; ssh authenticates the caller |
| P12 | A dedicated read-only foxglove_bridge in the room-memory container on its own port | protocol §2 ("the existing foxglove_bridge on :8765") | the bridge needs `labcommand_msgs`; LIVE untouched; tight whitelist; one-container rollback |
| P13 | A snapshot answering an outstanding request is applied whatever its `seq`; an events fallback resets the baseline; a `seq` that goes down on live `map_bricks` or `events` is a new publisher generation (on `status`/`epoch` it is only counted) | protocol §5 | after a node restart the "always accepted" and "`seq` ≤ last applied is ignored" rules otherwise collide and leave the Mac STALE |

## 11. Open questions for Trevor

1. **Is the rig static for v1?**
   - If yes, P9 and relocalization can wait, and RIG_MOVED only needs "put it back" or "re-reference".
   - If it moves between sessions, alignment to the old reference is required, and room generations become the normal case.
2. **What is the smallest object that must be detected: 5, 10 or 25 cm?**
   - It sets `voxel_size` (5 cm today), the minimum region size and T_commit.
   - D-125: a 2.9 cm box needs 0.5–1 s of accumulation even at 2 cm voxels.
   - It is also the "minimum detectable size" the view must state (protocol §6 rule 10).
3. **How many false alarms per hour are acceptable,** counted as false CONFIRMED events per hour in a static room? The brief proposes ≤ 1/h. The answer is the go/no-go threshold at S2 and S6.
4. **May `command_center.py` relay requests and operator actions (P11)?** If not, the alternative is bridge `clientPublish`. At the deployed bridge version, the Tailscale grant would then be its only control (§8).
5. **What runs the `:8765` bridge, with which parameters?** In particular `address`, `capabilities` and `topic_whitelist`. If it uses the defaults, it accepts client publishing from the LAN and the tailnet today (§8).
6. **Should memory mode be on at launch?** LIVE is off at launch by policy. Memory mode costs ~0.4 KB/s, so it could run whenever the app is open, or be opt-in like LIVE.
7. **How many Macs?** With one, the shared sync topics stay cheap. With several, P4 is necessary rather than prudent.
8. **Is the full event history needed on the Mac,** beyond the last 100 messages? If so, it needs a paged read on `:8766` (§4.5) and a retention policy for the Spark's event log (brief R10).
9. **Should a failed reference save (disk full) put the node in DEGRADED?** That changes protocol §4.1's DEGRADED trigger.

## 12. References

All opened on 2026-10-14.

**Foxglove**
- **[F1]** foxglove_bridge README at tag `ros-v3.4.1`: parameters and defaults (`port` 8765, `address` `0.0.0.0`, default `capabilities`, `topic_whitelist`, `client_topic_whitelist`, `min_qos_depth`/`max_qos_depth`, `tls`/`certfile`/`keyfile`). Also `message_backlog_size`: "drops the oldest data-plane message on overflow and disconnects clients whose control-plane queue fills. Defaults to `1024`." <https://raw.githubusercontent.com/foxglove/foxglove-sdk/ros-v3.4.1/ros/src/foxglove_bridge/README.md>
  - The SDK server documents the same drop-oldest queue: <https://docs.rs/foxglove/latest/foxglove/struct.WebSocketServer.html>
- **[F2]** foxglove_bridge source at tag `ros-v3.4.1`, `ros/src/foxglove_bridge/src/ros2_foxglove_bridge.cpp`. <https://raw.githubusercontent.com/foxglove/foxglove-sdk/ros-v3.4.1/ros/src/foxglove_bridge/src/ros2_foxglove_bridge.cpp>
  - One ROS subscription per channel, created for the first subscriber.
  - Messages of transient_local topics are cached per publisher and replayed to later subscribers.
  - The QoS choice: reliable or transient_local only if all publishers are; depth capped by `max_qos_depth`.
  - `clientPublish` encodings `cdr`/`json` through a generic publisher, so the type must be in the bridge's environment.
  - `client_topic_whitelist` is parsed and never applied; the same holds on `main`.
- **[F3]** foxglove_bridge in foxglove-sdk, the current home of the ROS 2 bridge: <https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge>
  - Its changelog: 3.4.1 (2026-06-01) "Update Foxglove SDK version to 0.25.1"; the latest release is 3.6.1 (2026-10-13). <https://raw.githubusercontent.com/foxglove/foxglove-sdk/main/ros/src/foxglove_bridge/CHANGELOG.rst>
- The old protocol spec repository was "archived by the owner on Aug 5, 2025", and its spec is marked outdated; no current standalone spec was found in the SDK docs. <https://github.com/foxglove/ws-protocol>

**ROS 2**
- **[R1]** ROS 2 Jazzy, *Quality of Service settings*: durability ("Transient local: the publisher becomes responsible for persisting samples for 'late-joining' subscriptions"), the compatibility tables (a volatile publisher with a transient_local subscription gives no communication), and QoS and matched events. <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
  - docs.ros.org served a bot check, so the content was read from the same page's source: <https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst>
- **[B1]** rosbag2, Jazzy branch: MCAP is the default storage, and `ros2 bag play` has `--qos-profile-overrides-path` for latched topics. <https://github.com/ros2/rosbag2/tree/jazzy>
- **[B2]** MCAP specification: a message record's `sequence` is an "optional counter for detecting gaps in messages". <https://mcap.dev/spec>

**Map synchronization prior art**
- **[M1]** octomap_server (ROS 2 branch), `octomap_server/src/octomap_server.cpp`: full binary and full maps on every publish, latched with transient_local; no deltas. <https://github.com/OctoMap/octomap_mapping/tree/ros2>
  - Paper: Hornung et al., *OctoMap*, Autonomous Robots 2013. <https://doi.org/10.1007/s10514-012-9321-0>
- **[M2]** voxblox:
  - `voxblox_msgs/msg/Layer.msg`, with `ACTION_UPDATE`, `ACTION_MERGE` and `ACTION_RESET`: <https://github.com/ethz-asl/voxblox/blob/master/voxblox_msgs/msg/Layer.msg>
  - `voxblox_ros/src/tsdf_server.cc`, which resets and sends all blocks to a new subscriber: <https://github.com/ethz-asl/voxblox/blob/master/voxblox_ros/src/tsdf_server.cc>
  - Paper: Oleynikova et al., IROS 2017. <https://arxiv.org/abs/1611.03631>
- **[M3]** nvblox:
  - `nvblox_msgs/msg/Mesh.msg` and `VoxelBlockLayer.msg`, with `block_indices` and the `clear` flag: <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/tree/main/nvblox_msgs/msg>
  - Docs, including Foxglove visualization: <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html>
  - The current release targets ROS 2 Lyrical, not Jazzy (see brief §1.1).
  - Paper: Millane et al., ICRA 2024. <https://arxiv.org/abs/2311.00626>

**Tailscale**
- **[T1]** Grants: deny-by-default access rules with port lists: <https://tailscale.com/kb/1324/grants>
  - Policy syntax and `tag:` names: <https://tailscale.com/kb/1337/policy-syntax>
- **[T2]** Enabling HTTPS (`tailscale cert`): needs MagicDNS, and names appear in Certificate Transparency logs. <https://tailscale.com/kb/1153/enabling-https>
- **[T3]** DERP servers: a relay used "as a last resort", which cannot decrypt traffic. <https://tailscale.com/kb/1232/derp-servers>

**Compression**
- **[Z1]** zlib manual: `windowBits` −8 … −15 is raw deflate, with "no zlib header or trailer". <https://zlib.net/manual.html>
- **[Z2]** Apple Compression, `COMPRESSION_ZLIB`: "raw DEFLATE format as described in IETF RFC 1951", equivalent to `deflateInit2(zstream,5,Z_DEFLATED,-15,8,Z_DEFAULT_STRATEGY)`. <https://developer.apple.com/documentation/compression/compression_zlib>
