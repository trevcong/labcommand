# Phase 3: live integration of the Spark `room_memory` node with the LabCommand mirror

Status: design for review, 2026-10-14. Nothing here is built.

**Amended 2026-10-16 (protocol v2, `PLAN-PUBGEN.md`, protocol §5, D34–D37).** Restart detection is now the publisher generation carried in every stateful message, not `seq` counters or `node_started_at`. Every `RequestSync` answer ends with a `SyncEnd` manifest on `/room_memory/sync/end`, which replaces P17's `EventsFallback`. P13, P16 and P17 are revised in place below. Superseded text is marked as such, and the protocol's D23, D26 and D27 keep the old rules as history.

This doc changes the bytes of one v1 message only: P16 adds `events_seq` to `MemoryStatus`. It also adds one new message, P17's `EventsFallback` (superseded by D36's `SyncEnd`; protocol v2 also appends `publisher_generation`, D34). Where Phase 3 needs a change to a decided rule, the change is listed as a numbered proposal (§10) for Trevor and Codex. `docs/ROOM-MEMORY-PROTOCOL.md` stays authoritative. This PR writes five proposals into the protocol draft, still subject to that review:
- P2, durability: protocol §2 and D24.
- P13, the `seq` rule: protocol §5 and D23. Its restart cases are superseded by D34 and D37.
- P6, `OperatorAction`: protocol §3.6 and D25, with `ros/labcommand_msgs/msg/OperatorAction.msg`.
- P16, the events high-water mark: protocol §3.3, §5 and D26. `MemoryStatus.events_seq` itself (the `.msg`, the Swift codec, the hand golden and both nodes) is already on main and in PR #14. It stays, scoped to one publisher generation; D26's request mark is superseded by D36.
- P17, the events fallback header: protocol §2, §3.8, §5 and D27, with `ros/labcommand_msgs/msg/EventsFallback.msg`. **Superseded by D36:** `SyncEnd` and `SeqRange` (protocol §3.8) on `/room_memory/sync/end`, with codecs, goldens and both nodes; `EventsFallback.msg` is deleted.

The rest are proposals only.

**Sources.**
- `~/Developer/lc-tasks/RESEARCH_BRIEF.md`: §2.3–§2.5, §3 Phase 3, §5. `~/Developer/RESEARCH_BRIEF.md` does not exist; only the lc-tasks copy was read.
- `docs/ROOM-MEMORY-PROTOCOL.md`, cited below as "protocol §n", with decisions D5, D9 and D19 of 2026-10-14.
- The Python reference node: `spark/room_memory/` (on main since PR #11).
- The C++ port: `spark/room_memory_cpp/PLAN.md` (PR #14, branch `room-memory-cpp`, which this branch is based on).
- The Mac live client: `FoxgloveProtocol.swift`, `LiveSession.swift`, `FoxgloveLiveClient.swift`.
- The measured link numbers in `lc-claude13.log`.
- External practice, cited in §12. Bridge behaviour was read from the foxglove-sdk source at tag `ros-v3.4.1`.

No Spark, rig or app was touched to write this.

## 0. Bottom line

- **A dedicated, read-only bridge for memory topics.** This and the next point are Phase 3's proposals (P12, P11). The transport stays open in the protocol (D28, Q5) until Trevor chooses.
  - The Mac opens one memory WebSocket to a second foxglove_bridge instance, which runs inside the room-memory container (P12).
  - That bridge serves only `/room_memory/*` and accepts no client publishing.
  - The existing `:8765` bridge, LIVE and SNAPSHOT are not touched.
- **Resync requests and operator actions go over the ssh-authenticated `:8766` API, not the bridge (P11).** *Superseded for `RequestSync` by protocol D39 (issue #21):* it goes over the :8767 bridge as a call of the node's `/room_memory/sync/request` service, the only service the bridge exposes (`service_whitelist` is enforced, unlike `client_topic_whitelist`), and the bridge still accepts no client publishing. Operator actions still need P11.
  - The bridge documents `client_topic_whitelist`, but at the deployed version and on `main` the parameter is parsed and never applied ([F2]).
  - With client publishing on, any peer that reaches the port could publish any known type on any topic, including a forged `/livox/lidar` cloud into the memory node.
- **The budget is easy in steady state and hard on reconnects.**
  - Steady state is about 1.5 KB/s down and 0.3 KB/s up, estimated with per-packet overhead (§3). Two thirds of it is the 60 s keyframe (protocol D29).
  - A first connect costs about 110 KB. A reconnect that keeps its baseline costs at most about 75 KB: at most one snapshot plus the missed events, with no events fallback (§4.3).
  - The protocol's own retry rate allows about 22 KB/s during a resync storm, 4.5× the budget. Backoff on the Mac and a snapshot-answer cap on the Spark bound it (P3, P4).
  - A byte bucket on `map_bricks` caps the map's long-run rate at 3 KB/s. A snapshot up to protocol §7's 2 MB cap is delayed for a bounded time, never held indefinitely. A map that would exceed the cap is reported on `:8766`, not published (P15).
- **One room frame for both sensors (P14).** Protocol §1.2 requires one fixed room frame, and `MapBricks` carries no sensor or frame id. So the node transforms each sensor's points into the room frame before integrating, and publishes one map. Until the JT128↔Mid-360 extrinsic exists, only one configured sensor contributes geometry.
- **Correctness comes from `seq` and `map_epoch`, not from QoS.**
  - DDS QoS governs only the same-host hop from node to bridge.
  - Across the tailnet the hop is one TCP stream. Messages are lost in only two places: the bridge's per-client backlog, which drops the oldest message on overflow ([F1]), and disconnects.
  - Either loss shows up as a `seq` gap or an epoch mismatch, which protocol §5 already turns into STALE and a resync.
  - `map_bricks` and `events` therefore stay volatile. A latched copy replayed by the bridge would make each connect pay for the map twice, and the request is the one connect mechanism (§4.3). (Under D23 it would also have looked like a node restart to every connected Mac; under D37 a lower `seq` is never a restart signal.)
  - `status` and `epoch` become transient_local (protocol §2, D24), so a Mac shows the memory state on connect. Both nodes now publish `status` that way; `epoch` waits for PR 2.4 (§2.5).
- **The governing invariant: the mirror is never shown as current while it is wrong.** It may be stale, incomplete or empty, and it says so. Every test in §7 checks this under faults.
- **The Spark side is not ready for live integration yet.**
  - Both nodes (Python oracle, C++ production, PR #14) publish `map_bricks` with 60 s keyframes, `events`, `status`, `activity` and the sync topics, and answer `RequestSync`.
  - They have no `epoch`, no RIG_MOVED, and never emit CONFIRMED. Those are PR 2.3 and 2.4. (Both now allocate a publisher generation at start and end every answer with a `SyncEnd`, PLAN-PUBGEN §2–§3.)
  - They keep each sensor's points in that sensor's own frame, so they have no room frame yet (P14).
  - The rollout (§9) gates on them.

## 1. Starting point and scope

| piece | state on 2026-10-14 | Phase 3 needs |
|---|---|---|
| Wire contract (protocol §1–§8) | merged in #10; Swift codecs and hand goldens; Spark-produced goldens not yet made (protocol §9) | Spark goldens (confirm D4's `uint64` alignment) before stage S3 |
| Mac `RoomMemoryStore` (PR 1.1) | not built | the mirror's data model; §5 adds the connection layer around it |
| Python node `spark/room_memory` | reference oracle (main, PR #11; extended by PR #14): `map_bricks` (keyframes and deltas, protocol D29), `events`, `status` with `events_seq`, `activity`, the two sync topics and the `RequestSync` handler with the bounded fallback; all reliable, every topic's durability declared (`status` and `activity` transient_local, §2.5); states BUILDING, TRACKING, DEGRADED | stays the oracle; not deployed; `epoch` (§2.5). P17's manifest is done as `SyncEnd` (D36), with the publisher generation (D34) |
| C++ node `spark/room_memory_cpp` | PR #14: byte-identical to Python (differential tests); 6.4–16.5 ms per 1 s window on an M5 Max (target < 200 ms); same topics, same QoS; its launch file also starts the :8767 bridge (protocol D28, still open) | production node; must add `epoch` (PR 2.4; P17's manifest is done as `SyncEnd`, D36, with the publisher generation, D34), CONFIRMED, moved supersession and RIG_MOVED (PR 2.3), the room-frame transform (P14) and an `OperatorAction` handler (S5); the durability change (§2.5) is done |
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
│   │ /room_memory/* (8 topics) │ ──DDS──▶ foxglove_bridge :8767, read-only ◀═ ws:// ══▶│  → RoomMemoryStore (PR 1.1)   │
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
  | `topic_whitelist` | `['^/room_memory/(map_bricks\|events\|status\|activity\|epoch\|sync/map_bricks\|sync/events\|sync/end)$']` (`sync/end` replaced `sync/events_fallback`, D36) |
  | `capabilities` | `[]`: no `clientPublish`, `parameters`, `services`, `connectionGraph` or `assets` |
  | `message_backlog_size` | default |

- **Benefits.**
  - LIVE is untouched.
  - No restart of the `:8765` bridge, which would interrupt LIVE.
  - The advertise burst on connect covers eight channels, not the lidar topics too.
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

The topic set, QoS and durability are protocol §2 (D18, D24), including the operator topic (D25). Phase 3 adds the request transport (P11). The QoS column is the node-to-bridge hop. Across the tailnet the transport is TCP.

| topic | dir | message | rate | QoS (protocol §2) | Phase 3 |
|---|---|---|---|---|---|
| `/room_memory/map_bricks` | S→M | `MapBricks` | snapshot when TRACKING starts and every `keyframe_period_s` (protocol D29); deltas ≤ 0.2 Hz, only on a reference change | reliable, volatile, keep-last 10 | volatile is required (§4.3) |
| `/room_memory/events` | S→M | `ChangeEvent` | per transition | reliable, volatile, keep-last 100 | volatile is required (§4.3) |
| `/room_memory/status` | S→M | `MemoryStatus` | 1 Hz | reliable, transient_local, keep-last 1 | as specified in both nodes (§2.5) |
| `/room_memory/activity` | S→M | `Activity` (protocol §3.7) | 1 Hz, with each status | reliable, transient_local, keep-last 1 | as specified in both nodes (PR #14); the Mac mirror decodes it (protocol D38), the view does not show it yet |
| `/room_memory/epoch` | S→M | `Epoch` | 0.2 Hz | reliable, transient_local, keep-last 1 | not published yet (PR 2.4, §2.5) |
| `/room_memory/sync/map_bricks` | S→M | `MapBricks` | answers | reliable, volatile, keep-last 10 | — |
| `/room_memory/sync/events` | S→M | `ChangeEvent` | answers | reliable, volatile, keep-last 200 | — |
| `/room_memory/sync/end` | S→M | `SyncEnd` (protocol §3.8) | one per `RequestSync` answer, after its messages | reliable, volatile, keep-last 10 | P17 as revised by D36; replaces `/room_memory/sync/events_fallback` and `EventsFallback`; published by both nodes |
| `/room_memory/request_sync` | M→S | `RequestSync` | on connect and on a gap | reliable, volatile, keep-last 10 | carried as CDR bytes over `:8766` and relayed on the Spark (P11); not on the bridge |
| `/room_memory/operator` | M→S | `OperatorAction` (protocol §3.6) | operator actions | reliable, volatile, keep-last 10 | same relay (P11) |

The request transport in the last two rows is P11, a proposal. Protocol D28 and Q5 leave it open, with the data path: the dedicated bridge on :8767 or the existing one on :8765, and requests over bridge client publishing or over this relay. Trevor decides.

**How the bridge sets its own subscription QoS** ([F2]).
- Reliable only if all publishers are reliable.
- Transient_local only if all publishers are transient_local.
- Depth is the publishers' total history, at least `min_qos_depth` (default 1) and at most `max_qos_depth` (default 25).

Each memory topic has one publisher, so the bridge follows the node's choices. In ROS 2 a volatile publisher and a transient_local subscriber do not communicate at all ([R1]). That is one more reason for the node to declare durability explicitly per topic.

### 2.3 Container and node

- **Production node.** `room_memory_cpp` (ROS package `room_memory_cpp`, node name `room_memory`) runs in its own container next to `lidar-dual`, with `--net=host --ipc=host`, as in its Dockerfile. The dedicated bridge runs in the same container image. The Python `room_memory` node stays the test oracle and is not deployed.
- **State directory.** `~/room-memory/<room_id>/` holds the room frame (each sensor's `T_room←sensor`, P14), one `reference.cdr` in that frame (a zlib `MapBricks`, written atomically) and the event log.
  - This replaces today's per-sensor `reference-<sensor>.cdr`, which are in each sensor's own frame.
  - The node loads a reference only if it was saved under the same transforms.
  - A changed `T_room←frame_sensor`, or a different frame sensor, moves the room frame. It starts a new `room_id` generation, as P9 does.
  - Adding or recalibrating the other sensor's transform keeps the frame. The node rebuilds the reference (BUILDING), and the new reference bumps `map_epoch` and goes out as a snapshot (P7).
  - `map_epoch` and `event_id` survive a restart. The publisher generation does not: every start allocates a higher one in `<state_dir>/<room_id>/publisher_generation` (PLAN-PUBGEN §3, D34).
  - `event_id` is seeded from the node start time in ms << 20. It stays unique only while the Spark clock does not step back (F11).
- **Ring buffers.** The node keeps its last 10 `map_bricks` and last 100 `events` messages as published bytes, for replay (protocol §5).

### 2.4 Command-center block on `:8766` (P10)

This is the out-of-band identity and health channel. It is already polled every 5 s, and it already obeys "unavailable ≠ zero". Proposed fields:

| field | meaning | missing renders |
|---|---|---|
| `room_id` | the room the node serves | memory mode "not configured" |
| `protocol` | wire-protocol majors the node publishes, e.g. `[1]` | "protocol —"; the Mac does not subscribe |
| `node` | `room_memory_cpp` git sha and image digest; bridge version | "—" |
| `node_started_at` | node start time (Unix s); changes on every restart; diagnostics only (D34) | "—" |
| `map_epoch`, `map_digest` | current epoch, and SHA-256 of the canonical *uncompressed* snapshot payload at that epoch (protocol §3.1) | "—" |
| `state` | `MemoryStatus.state` name | "—" |
| `publishing` | true when the node's publishers exist | false renders "node down", never "empty room" |

**`map_digest` is an end-to-end consistency check that needs no wire change.** When the mirror's epoch equals `map_epoch`, the Mac hashes its own canonical payload and compares. A mismatch is a protocol violation and forces a snapshot. It is the soak test's main oracle (§7.5).

**`node_started_at`** labels diagnostics only. It was case 3 of D23's `seq` rule, which let a Mac reuse a baseline across a disconnect only while this value was unchanged. That case is superseded: the publisher generation in every stateful message tells the Mac whether the node restarted (protocol §5, D34, D37), so the Mac no longer reads `:8766` before a request.

### 2.5 Node durability change

Protocol §2 (D24) makes `status`, `activity` and `epoch` transient_local. Both nodes published `activity` that way since PR #14. **Done** on `protocol-v2-followups` for `status` and the explicit volatile topics, as planned below; `epoch` waits for PR 2.4 (no node publishes it). The rclpy late-joiner test and the C++ node need ROS 2 and were not run on the Mac.

**Topics.**

| topic | durability | depth | what a late-joining subscriber gets |
|---|---|---|---|
| `status` | transient_local (done) | 1 | the last `MemoryStatus` published, then the live stream |
| `activity` | transient_local (done) | 1 | the last `Activity` published, then the live stream |
| `epoch` | transient_local (PR 2.4) | 1 | the last `Epoch` published, then the live stream |
| `map_bricks`, `events`, the three sync topics | volatile, declared explicitly | 10, 100, 10, 200, 10 | nothing published before it subscribed; the Mac gets the map and recent events by `RequestSync` |
| `request_sync`, `operator` (node subscriptions) | volatile | 10 | nothing stored; a restarted node never sees an old request or action again |

**What a late joiner gets, in practice.**
- The late joiner is usually the dedicated bridge, when the first Mac subscribes. It takes transient_local for a topic only if all of that topic's publishers are transient_local ([F2]).
- The copy is one message. It is at most one period old while the node runs.
- If the node has stopped, the bridge unadvertises the topic, so no new Mac subscribes to a dead node's last status.
- A stray volatile publisher on `status`, for example a `ros2 topic pub` while debugging, silently makes the bridge's subscription volatile. The Mac then waits up to 1 s for `status`, which is slower but correct. A second publisher's `seq` would also break protocol §5's `seq` rule, so nothing but the node publishes on memory topics.
- A Mac that is already subscribed may get the bridge's stored copy again when another client subscribes (V2). Protocol §5's `seq` rule ignores it.

**Code changes** (PR 2.4, or a small PR before S3). They are made in the node code, not here. Each names the code it changes, not a line number.
- **Python**, `spark/room_memory/room_memory/node.py`. `DurabilityPolicy` is already imported for `activity`.
  - The `events` publisher's profile (`reliable`, depth 100), the `maps` profile and the `sync_events` profile: add `durability=DurabilityPolicy.VOLATILE`.
  - The `status` publisher: `QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)`, the profile `activity` already has.
  - PR 2.4 creates the `epoch` publisher with the same profile as `status`, and the `operator` subscription volatile.
- **C++**, `spark/room_memory_cpp/node/room_memory_node.cpp`.
  - `events_`, `map_`, `sync_map_`, `sync_events_` and the `requests_` subscription: add `.durability_volatile()` to their `rclcpp::QoS`.
  - `status_`: `rclcpp::QoS(1).reliable().transient_local()`, as `activity_` has.
  - `create_generic_publisher` takes the same `rclcpp::QoS`, so the serialized path is unchanged.
  - PR 2.4 creates the `epoch` publisher as `rclcpp::QoS(1).reliable().transient_local()`.
  - `spark/room_memory_cpp/PLAN.md` lists the publishers' QoS in prose ("It publishes the codec bytes with generic publishers"), and gets the same change.
- **Tests**, `spark/room_memory/tests/test_node.py`.
  - The existing `status` subscriber is volatile. It still receives from a transient_local publisher ([R1]), so it passes unchanged.
  - Add a late-joiner test for `status` like `test_activity_reaches_a_late_transient_local_subscriber`: publish one status, then create a transient_local subscriber, and assert that it receives exactly that message.
  - The C++ node shares these topics and QoS, so the same test runs against it in its container.

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
| `MemoryStatus` | 1 Hz | 76 (exact) | 204 | 204 | 124 |
| `Activity` | 1 Hz | 88 (exact) | 220 | 220 | 124 |
| `Epoch` | 0.2 Hz | 53 (exact) | 188 | 38 | 25 |
| `ChangeEvent`, no mask | per transition | 136 (exact) | 268 | — | — |
| `ChangeEvent`, 25 cm box mask | per transition | 253 (measured) | 396 | — | — |
| `MapBricks` delta, 25 cm box | on a reference change | 367 (measured) | 508 | — | — |
| `MapBricks` snapshot | keyframe every 60 s (protocol D29); connect / resync | 52,863–59,024 (measured) | 57,808–64,540 (44–49 segments) | 960–1,080 as keyframes | ~50 |

Small messages pay 2–3× their size in headers, which is why §3.2 counts wire bytes.

### 3.2 Scenarios

| scenario | down | up | notes |
|---|---|---|---|
| **Steady, static room** (status, activity, epoch and the 60 s keyframe) | ~1.5 KB/s | ~0.3 KB/s | ~1.8 KB/s total; ~2.8× under budget. Without the keyframe ~0.73 KB/s. With the `:8766` poll, ~2.2 KB/s |
| **Quiet day** (60 changes/h; each 3 transitions with mask; each accepted with a 0.4 KB delta) | +0.04 KB/s | +0.01 KB/s | events and deltas are noise next to status |
| **People walking through** (≈ 2 CANDIDATE/EXPIRED transitions per second while someone moves; no mask on CANDIDATE, P5) | +0.54 KB/s | +0.25 KB/s | ~1.2 KB/s total while active. Person-sized masks on CANDIDATE (est. 0.5–1 KB each) would make it 2–3 KB/s, hence P5 |
| **First connect** (no baseline: app launch, or a new room, `room_id` generation or publisher generation) | ~65 KB snapshot + ≤ ~48 KB event fallback (≤ 120 messages × 396 B) + advertise | ~6 KB ACKs, < 1 KB requests | ~110–120 KB; 6 % of the 2 MB cap |
| **Reconnect with a baseline** (§4.3) | map: a replay of the 1–9 missed deltas (~0.5 KB each), else one ~65 KB snapshot; events: the missed messages only, ≤ 100 × 396 B; + advertise | ≤ ~6 KB ACKs, < 1 KB requests | ≤ ~75 KB; nothing on the map topic when no delta was missed, only the `SyncEnd` (protocol §5, D36) |
| **Re-reference or large accept** (the whole map changes) | one snapshot, 58–65 KB | — | P7: snapshot instead of a delta larger than half a snapshot |
| **Resync storm at protocol §5's limit** (one request per topic per 5 s, unanswerable by replay) | 12.9 KB/s map + 9.5 KB/s events ≈ 22 KB/s | ~0.2 KB/s | **4.5× over budget**; P3 and P4 bound it |

**Advertise burst: unmeasured.**
- On connect the bridge sends `serverInfo` and one `advertise` listing each whitelisted channel with its full message definition ([F2]).
- On the dedicated bridge that is eight channels. `ChangeEvent`'s definition includes `MapBricks` and `builtin_interfaces/Time`.
- The estimate is under 10 KB. Stage S1 measures it.

**Amortized reconnect limit.** At 1.8 KB/s steady and about 115 KB per first connect, the 5 KB/s mean holds only if reconnects average at most one per ~36 s. A reconnect with a baseline costs at most ~75 KB, so the bound is conservative. Hence P3's floor on the reconnect delay.

**Every snapshot figure scales with the map.** The rows above use the measured 53–59 KB snapshot. Protocol §7 targets ≤ 500 KB and caps at 2 MB. The first-connect, re-reference, storm and keyframe figures grow in proportion. The `map_bricks` bucket (§3.3) keeps the map's long-run rate at 3 KB/s whatever the snapshot size, and delays messages instead.

### 3.3 Spark-side shaping (PR 2.4)

- **Delta coalescing.** Deltas are capped at 0.2 Hz (protocol §2). Reference changes within one 5 s slot merge into one delta. That is legal: a delta's `map_epoch` must only exceed its `base_epoch` (D8), so epochs may skip.
- **Byte bucket on `events` answers (P4).** One bucket covers `sync/events` and the events `SyncEnd`s on `sync/end` (which replaced `sync/events_fallback`), not live `events`, which stays under 1/min. It refills at 3 KB/s up to 48 KB, one full fallback (≤ 120 messages × 396 B). An answer is admitted whole at a balance ≥ 0 and debits its full size, so it never waits for its own size and is never split. Answers wait in arrival order. Without it, ten Macs each requesting a 99-event replay every 30 s would send about 13 KB/s on the shared sync topic (Codex, PR #13).
- **Byte bucket on `map_bricks` (P15).** One bucket covers the live topic and `sync/map_bricks`. It refills at 3 KB/s up to 180 KB.
  - **Admission.** A message may go out whenever the balance is ≥ 0 and its other limits allow (0.2 Hz for deltas, P4 for snapshot answers). It debits its full size, even if that takes the balance negative.
  - **Debt is bounded.** P15 makes protocol §7's 2 MB snapshot cap a node rule: the node never publishes a `MapBricks` larger than 2 MB on the wire. The protocol's own validity limit is D22's 16 MiB uncompressed payload, which is larger, so this rule is P15's, not the protocol's.
    - So one message debits at most 2 MB, and the balance stays above −2 MB.
    - A message never waits for its own size to fit, so a snapshot up to the cap is never held forever.
  - **A reference that would need a larger snapshot** is not published and not held silently. The node keeps serving deltas. It reports `snapshot_too_large` with the size in the `:8766` block (P10). A Mac with no mirror shows "Waiting for map: map exceeds 2 MB on the Spark", never an empty room. Whether this also makes the node DEGRADED is Q9's kind of change to protocol §4.1, left to Trevor.
  - **Why not pace one message across windows.** A `MapBricks` reaches the bridge as one ROS message, and the bridge sends it as one WebSocket frame. Splitting it would be a wire change.
  - **Worst-case wait.** Only debt from an earlier message causes a wait, about debt ÷ 3 KB/s. From a full bucket:

    | earlier message | debt it leaves | wait after it |
    |---|---|---|
    | the measured 53–59 KB snapshot | none | none: it fits the 180 KB bucket |
    | a 500 KB snapshot (protocol §7's target) | ~320 KB | ~2 min |
    | a 2 MB snapshot (the cap) | ~1.8 MB | ~10 min |

    A connecting Mac shows EMPTY or STALE for that long, which is honest but slow. If real maps approach the target, Trevor should revisit the rate rather than the rule (Q11).
  - **Held deltas.** A delta that cannot go out yet is held and coalesced with later changes, never dropped. Dropping it would break the `seq` chain, and a held delta has no `seq` yet: `seq` is assigned when a message is published.
  - **The published state.** The node keeps the reference as of the newest message published on live `map_bricks`. While a delta is held, the working reference runs ahead of it. Everything that leaves the Spark describes the published state:
    - `map_epoch` in `MemoryStatus`, `Epoch` and `ChangeEvent`;
    - `map_digest` in `:8766` (P10);
    - the content of a snapshot answer;
    - the state of an accepted event. Its ACCEPTED transition is held with the delta that carries the change, and published right after that delta, with its `map_epoch` (protocol §4.2). Until then the event is CONFIRMED on the wire and counts in `n_open_events`.

    Three failures are avoided this way.
    - If status reported the working epoch, D20 would mark every Mac STALE 2 s into each hold. Each Mac would then request, and get a "nothing missed" answer (a snapshot under D23, an empty `SyncEnd` under D36) that the bucket also gates.
    - If a snapshot answer carried the working map, the held delta, whose base is the published epoch, would no longer chain from it, and every Mac would resync again.
    - If ACCEPTED went out at once, every Mac would remove the change's overlay while its map still lacked the change. It would look CURRENT with the old geometry and no overlay for the whole hold, up to ~10 min.
  - **Live snapshots.** A live snapshot, under P7 or a keyframe (protocol D29), publishes the working reference. It supersedes any held delta, which is discarded unpublished. The ACCEPTED transitions held with that delta are published right after the snapshot.
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

- **`seq`.** Protocol §5 is the only statement of how `seq` orders messages within a publisher generation, how duplicates are ignored, and how a node restart starts a new publisher generation (D34, D37). This doc does not restate it. D23's restart cases 1–3, which this doc used to cite, are superseded.
- **`map_epoch`.** It only grows along a delta chain. A delta applies only on `base_epoch` = the mirror's epoch and `seq` = baseline + 1 (protocol §3.1).
- **Not chaining.** A gap, a wrong base or a new publisher generation makes the map STALE (protocol §5).
- **STALE.** While STALE, the Mac keeps its last good mirror greyed and labelled "resyncing", and never shows it as current. This is protocol §5's rule. It replaces the brief's older "discard local state".
- **Lost trailing delta.** `MemoryStatus.map_epoch` ≠ the mirror's epoch on 2 consecutive status messages catches it (D20).

### 4.3 Connect sequence: one mechanism, not two

**Mac.**
1. Open the socket. Read `serverInfo` and `advertise`. A mirror kept from an earlier connection has been STALE, greyed, since that socket closed (§5.2). It cannot prove it is current across a gap of unknown length.
2. Check the schema fingerprint of each memory channel (§4.10). On a mismatch, stop at INCOMPATIBLE. (Not for the publisher-generation fields: they were appended without a new major, and the decoder tells legacy from v2 by trailing bytes, §4.10 note.)
3. Subscribe to all eight S→M topics in one `subscribe` message.
   - The sync subscriptions stay up for the whole connection.
   - A subscription made only when a request goes out could miss its answer, because the Mac cannot observe when the bridge's ROS subscription has matched.
4. Send one `RequestSync` per topic through `:8766` (P11), as soon as the connection's first `MemoryStatus` names the publisher generation (P8 as revised by D37; `status` is transient_local, so that is near-immediate).
   - **`expected_generation` is that generation, and each send carries a fresh `request_token`** (D35). The Spark honours `since` only when `expected_generation` equals its own generation; otherwise it answers as for `SINCE_NONE` in its generation.
   - **`since`** is the top of what the Mac holds in that generation: for `TOPIC_MAP_BRICKS` the mirror's epoch and applied `seq`, for `TOPIC_EVENTS` the (`map_epoch`, `seq`) of the top of the contiguous received run. `SINCE_NONE` when it holds nothing there (protocol §5). The two topics are independent.
     - Nothing is persisted (§2.1), so every app launch starts with `SINCE_NONE`.
     - *Superseded:* the usable-baseline check, which read `node_started_at` from `:8766` before this step (D23's case 3). The generation in every message replaces it, and `node_started_at` (P10) is diagnostics only.
   - **What the Spark answers** (protocol §5, PLAN-PUBGEN §2.3). Events: a replay of the missed messages, or the bounded fallback when `since` is not among the 100 kept messages. Map: a replay when 1–9 deltas were missed, nothing when none were (no longer a snapshot), else a snapshot. Every answer, including an empty one, is followed by one `SyncEnd` on `/room_memory/sync/end` (D36). It names the `bound`, the `delivered` ranges and, for an events fallback, the `unavailable` ones. It travels on its own topic and can arrive before the answer's messages.
   - **When a request is done** (P3's backoff): only once the Mac has received what the `SyncEnd` for its own token names: every `delivered` event `seq`, or a map chain at or past the manifest (protocol §5, D36). No timer, request flag, quiet period or live progress completes it. A retrieval that times out (5, 10, 20, 30 s) is sent again with a fresh token. A `SyncEnd`'s `bound` and `unavailable` ranges are publisher facts, recorded from any `SyncEnd` of the generation, whoever asked; a foreign token changes no retrieval.
     - *Superseded:* the three completion rules that stood here: map done at CURRENT, events done at D26's request mark, and a fallback given up only on a matching `EventsFallback` header whose last kept `seq` had arrived on the sync topic (D27). Protocol D26 and D27 keep them as history.
   - Protocol §5 says to wait up to 5 s for the Spark's own snapshot first.
   - The node cannot see a connect through a bridge: the bridge holds one ROS subscription per channel for all clients, created by the first ([F2]). So only the first client ever triggers the node's "matched" event ([R1]).
5. The map is CURRENT when protocol §5's condition holds.

**Why `map_bricks` and `events` stay volatile.**
- For transient_local topics, the bridge caches the last N messages per publisher, with N the publisher's depth. It replays them when a later client subscribes ([F2]).
- The replay is logged to the WebSocket server's sink, not to one connection. Whether clients that are already subscribed receive it too is not settled (V2).
- If they do, a cached old message would reach a Mac that is already CURRENT on the live topic, with a `seq` below the highest it has received. Under D37 it carries the same publisher generation and is ignored as a duplicate or an older message, never a restart signal. (Under D23's case 1 it read as a new generation, and every connected Mac would have dropped to STALE and resynced whenever another client subscribed. Under the old "a snapshot is always accepted" it was worse: the cached snapshot would have rewound the Mac and shown the old map as current.)
- A latched burst plus a request answer would also make each connect pay for the map twice.

So the request is the one connect mechanism.

**Status and epoch (P2, now protocol §2 and D24).** `status` and `epoch` are full-state messages with depth 1, and they are transient_local, so a new client shows the memory state on connect instead of after up to 5 s. A copy delivered twice is harmless. A status with a `seq` at or below the one held is a duplicate, never a generation signal (protocol §5, D37), and D20's epoch check needs two consecutive disagreeing messages. §2.5 is the node change.

**Keyframes (P1, now protocol D29 in PR #14).** The node publishes a live snapshot every `keyframe_period_s` (60 s), whatever the requests do.
- A Mac whose request path fails still gets a current map within 60 s. That includes a failure the Spark cannot see, such as one Mac's ssh tunnel down while the relay serves the others. So the keyframes stay on when the relay works.
- The cost is ~1.1 KB/s with overheads at the measured snapshot size (§3.2); the `map_bricks` bucket (§3.3) covers it.
- The live topic stays volatile, and TCP keeps it in order, so a periodic snapshot cannot rewind anyone.
- A keyframe is not the connect mechanism: the request is (above). It bounds how long a Mac without a request path waits.

### 4.4 `RequestSync` (D19) in operation

**Transport (P11).**
- The Mac POSTs the `RequestSync` CDR bytes, unchanged, to the command-center API through the existing ssh tunnel.
- `command_center.py` publishes them on `/room_memory/request_sync` on the Spark.
- The answer comes back on the bridge's sync topics, as protocol §5 specifies.

Compared with the bridge's `clientPublish`:
- The relay keeps the bridge read-only.
- Only holders of an authorized ssh key can reach it (§8).

**Feasibility.** Publishing a `labcommand_msgs` type needs that type in `command_center.py`'s environment. If the API runs outside the ROS environment, it forwards the bytes instead to a localhost-only endpoint in the node, which publishes them. Either way the bytes are unchanged.

`OperatorAction` (protocol §3.6) uses the same relay.

**Answers** follow protocol §5:
- **Replay**, if `expected_generation` is the node's generation (D35), the requested `seq` is in the ring buffer and the epoch matches. Nothing, if the map `since` is already the newest.
- Otherwise a **snapshot** for maps, or the **bounded fallback** for events.
- Every answer is followed by its `SyncEnd` on `/room_memory/sync/end` (D36), which names what was delivered and what is unavailable. (It replaces P17's `EventsFallback` header, which preceded a fallback.)

**Validation on the Spark.** A request is dropped and logged unless all of these hold:
- `room_id` is the served room.
- `map_epoch` = `since_epoch`.
- `SINCE_NONE` appears in both `since` fields or in neither.

This is symmetric with the Mac's validation (D16).

**Rate limits.**
- **Mac (P3).** At most one request per topic per 5 s (protocol §5). While consecutive requests end without CURRENT, the interval backs off 5 → 10 → 20 → 30 s.
- **Spark (P4).** At most one *snapshot* answer per topic per 15 s across all clients. A request inside the window is served in the next slot. Map replays (≤ 9 deltas) are not capped by P4; the `map_bricks` bucket (§3.3) covers them. Every `events` answer, replay or fallback with its `SyncEnd`, goes through the events bucket (§3.3), shared by all clients. A request repeated with the same `since` while its answer still waits is answered once.
- **Together** these bound a storm at 65 KB + 48 KB per 15 s, about 7.5 KB/s. Once their bursts are spent, the `map_bricks` bucket and the events bucket (§3.3) hold the two shares to 3 KB/s each, so a long storm runs at about 6 KB/s, however many Macs request. For larger maps the map bucket, not P4, is the binding limit.
  - That is still over 5 KB/s while the storm lasts.
  - The hourly mean stays in budget unless there are more than ~40 storms an hour.
  - The soak (§7.5) reports both figures.

**Node restart (P13, now protocol §5, D34 and D37; D23's restart cases superseded).** After a restart the node's `seq` starts again at 0. The protocol used to say both that a snapshot "is always accepted" and that a sync message "with `seq` ≤ the last applied is ignored", and those two rules collided after a restart. Protocol §5 now scopes every `seq` to a publisher generation, and this doc only says where Phase 3 plugs into it:
- **Every start allocates a higher publisher generation** (durable, wall-clock floored, PLAN-PUBGEN §3), and every stateful message carries it (D34). The Mac's mirror is scoped to one generation: a higher one is installed on its first validated message and the old one archived, a lower one is ignored (D37). This holds the same while connected and across a disconnect, and it still tells a restart from a continuation when the new run's counters have passed the old ones.
- **Nothing is inferred from counters.** A lower `seq`, an `events_seq` of `EVENTS_SEQ_NONE`, or a changed `node_started_at` is not a restart signal. `node_started_at` (P10) is diagnostics only.
- **A fallback** is no longer recognized by a header. The answer's `SyncEnd` (D36) names its delivered and unavailable ranges, and a retrieval completes only by receipt of what the manifest names (§4.3).
- **A publisher generation is not a `room_id` generation.** P9's `lab` → `lab~2` renames the room when its frame moves; a publisher generation numbers the node processes that publish one `room_id`.
- *Superseded:* D23's cases 1 and 2 (a lower `seq` while connected, seen within about 2 s) and case 3 (`node_started_at` across a disconnect), and P17's `EventsFallback` header matched on the request's `since`.

### 4.5 Event history across a disconnect

- The Spark keeps 100 event messages. Older history reaches a reconnecting Mac only through the bounded fallback: the latest message of each open event, then the 100 kept (protocol §5).
- Older closed history is not resent. The fallback's `SyncEnd` names those `seq`s `unavailable`, and the Mac shows them as lost ("history incomplete: n missing (k lost on the Spark)", PLAN-PUBGEN §4.8). It never invents transitions.
- Across a node restart, the earlier run's history is archived, not merged: "Spark node restarted · events from the earlier run may be missing", with its open events listed as "earlier run · may be outdated" (D37).
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
| Socket drop and reconnect; node did not restart | the mirror goes STALE, greyed, on close; the first live `map_bricks` may be a delta | per-topic requests in the same publisher generation with `since = (epoch, seq)` (§4.3): map replay if 1–9 deltas behind, nothing if none were missed, else a snapshot; events replay, so the history stays complete if `since` is among the 100 kept; each answer ends with its `SyncEnd` | no |
| Dedicated bridge restarts; node keeps running | close; on return the publisher generation is unchanged and `seq` continues | as above; replay usually succeeds | no |
| Node restarts; bridge keeps running | the next message carries a higher publisher generation | the mirror installs the new publisher generation and archives the old one (D37); old-generation messages still in flight are ignored; restart notice; map STALE (the earlier run's bricks greyed) until a snapshot of the new generation; requests with the new `expected_generation` | no |
| Node restarted while the Mac was away | the first message after the reconnect carries a higher publisher generation, even if its counters passed the old ones | as the row above; nothing is read from `:8766` (`node_started_at` is diagnostics only) | no |
| Bridge backlog overflow (> 1024 queued messages for this client) | the oldest messages are dropped → `seq` gap | replay or snapshot; overflow counted (§7.5) | no |
| Mac sleep/wake | TCP looks open; nothing arrives | status stale at 3 s; socket closed and redialled after 10 s of silence | no |
| Lost trailing delta | nothing until status disagrees | `MemoryStatus.map_epoch` ≠ mirror twice → STALE → request (D20) | no |
| Mac decode backlog | messages queue in the app | decode off the main thread, as LIVE does; freshness uses the Mac's receive time, so a backlog shows as stale | no |
| A second Mac connects while the first is CURRENT | snapshot answers on the shared sync topic; possibly the bridge's stored `status`/`epoch` copy again (V2) | the answers apply to the first Mac as any message of the generation would (duplicates ignored); a foreign `SyncEnd` adds only publisher facts (`bound`, `unavailable`, D36); the `seq` rule ignores the copy (protocol §5) | no |

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

**Coordinates.** Two separate problems. P14 handles both sensors at once, and P9 handles a rig that moved.

*Both sensors at once (P14).*
- Today both nodes keep each sensor's points in that sensor's own frame, origin 0 (`spark/room_memory/PLAN.md`; brief R1).
- With both sensors enabled, that gives two frames, while protocol §1.2 has one room frame per `room_id`. `MapBricks` carries no sensor or frame id, so the same brick index from the two sensors would alias, and event geometry would not line up with the mirror. A new room generation does not help here, because both frames exist at the same moment.
- **Rule.** The room frame is fixed per `room_id`, in meters, with z up (protocol §1.2). Each sensor has a fixed rigid transform `T_room←sensor`, stored in the state directory (§2.3). The node applies it to every point and to the sensor origin used for ray casting, *before* integration. DISAPPEARED depends on free-space rays, so the origin matters as much as the points.
- **Every coordinate on the wire is in that frame:** the bricks of `MapBricks`, and `centroid`, `aabb`, `from_centroid` and the nested `mask` of `ChangeEvent`. The node publishes one reference, not one per sensor.
- **Where the frame comes from.**
  - The room frame is one sensor's frame, levelled so that z is up (a gravity or floor-plane fit), with its origin offset chosen by PR 2.2. The origin is a size decision too (D6).
  - The other sensor's transform is the JT128↔Mid-360 extrinsic composed with that.
- **Until the extrinsic exists,** only one configured `frame_sensor` contributes geometry: map bricks and events. The other sensor still reports `per_sensor_hz`, but adds nothing to the published map. This changes D17's "DEGRADED keeps tracking with the sensors left". If the frame sensor goes silent, the node is DEGRADED with no new events (F10). Which sensor is the frame sensor is Q10.
- **Rejected alternative: separately identified per-sensor maps.** They would need a sensor or frame id in `MapBricks` and `ChangeEvent`, or a topic per sensor. Either is a breaking change (§4.10), and the Mac would still need the extrinsic to draw them together. P14 needs no wire change and makes the node conform to protocol §1.2.

*A rig that moved (P9).*
- `T_room←sensor` is fixed relative to the rig, so a moved rig moves the room frame, and pre-move events are in the old frame.
- **P9:** re-referencing after RIG_MOVED starts a new `room_id` generation (`lab` → `lab~2`), unless the node aligns the new reference to the old one within the guard's threshold.
- A new `room_id` makes the existing rules do the right thing (protocol §1.4, §5): old messages are ignored, and the new room starts from a snapshot.
- The Mac keeps the old room's history read-only.
- Q1 decides whether this is needed.

### 4.10 Versioning

XCDR1 has no extensibility. Decoders reject 4 or more unread trailing bytes and unknown enum values (protocol §1.1, D3). So **every field or enum change is a breaking change**, and the scheme is built around that.

*Note (2026-10-16).* The publisher-generation change (D34–D36) did not use this scheme. Its fields are appended at the end of each message, a decoder tells the legacy layout from the v2 one by the trailing bytes, the topic names are unchanged, and a Mac facing an old node runs in legacy mode, not INCOMPATIBLE (PLAN-PUBGEN §2.1, §4.6). The client needs no schema text for it: `FoxgloveProtocol.Channel` carries none today. The scheme below still applies to any other breaking change.

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

The operator's actions change state on the Spark: acknowledge, accept, dismiss, label, mark ground truth, pause, resume and re-reference (re-baseline). P6's message for them is now in the protocol draft. It is defined once, in protocol §3.6 (D25) and `ros/labcommand_msgs/msg/OperatorAction.msg`, and is not repeated here. Topic `/room_memory/operator`, reliable, volatile, keep-last 10 (protocol §2). The Mac originates it, the relay (P11) publishes it on the Spark, and the node is its only subscriber.

What Phase 3 adds around the message:
- **No reply message.** The answer is what the Spark already publishes: the event's next transition, a status change, or for ACCEPT a delta with a `map_epoch` bump, then the ACCEPTED transition (protocol §4.2). During a bucket hold (§3.3) that pair waits together, and the event stays CONFIRMED on every Mac until the delta goes out. Acknowledge, label and ground-truth marks change nothing on the wire, so in v1 only the Mac that sent them shows them.
- **Compare-and-set.** ACCEPT and REREFERENCE with an out-of-date `map_epoch`, and ACCEPT or DISMISS with an out-of-date `expected_state`, are refused and logged. `map_epoch` is compared with the *published* epoch (P15), the one the operator can have seen. Otherwise every action during a delta hold, which can last minutes, would be refused. `expected_state` is compared with the node's own state, which includes an accept still held with its delta. So a second Mac's DISMISS of an event accepted during a hold is refused, although that Mac still shows it CONFIRMED (F16). With `action_id` dedupe, a double click or a retried POST cannot accept twice, and an action never applies to a map the operator did not see.
- **Audit log.** The Spark appends every action with its `action_id`, `operator_id`, arrival time and outcome. `operator_id` is what the app says, not an authenticated identity. Behind an ssh local forward the relay sees a loopback connection, and cannot tell which key opened the tunnel. The log records the id as claimed.

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
    │ delta first after connect│ socket closed · seq gap · wrong base · delta first after connect ·
    ▼                          │ status epoch ≠ mirror ×2 · map_digest mismatch · new publisher generation (D37)
  STALE(no mirror) ◀───────────┴──────────────▶ STALE(last good mirror, greyed)
    │  RequestSync since=NONE            │  RequestSync since=(epoch, seq) or NONE; P3 backoff
    └──────── snapshot ─────▶ CURRENT ◀──┴── snapshot (seq ≥ baseline), or a replay that chains to the newest live seq and status epoch
```

| state | renders |
|---|---|
| EMPTY | "Waiting for map" |
| CURRENT | the map |
| STALE with a mirror | the last good map greyed: "resyncing · as of epoch N, \<age\>" |
| STALE without one | "Waiting for map" |

Change overlays are drawn only in CURRENT.

**Socket closed.** A CURRENT map goes STALE with its mirror kept, and the mirror's epoch and `seq` become the next connection's `since` (§4.3).
- Without this step, `status` would be fresh the moment the socket reopens (transient_local, protocol §2). A mirror that missed deltas would then render as current for up to 2 s, until D20 caught it.
- The next map request uses `SINCE_NONE` only when the Mac holds nothing in the current publisher generation (§4.3).
  - On a `room_id` or protocol-major change, the old mirror is dropped and the map is EMPTY.
  - On a new publisher generation (D37), the earlier run's map stays greyed as the last good map until a snapshot of the new generation replaces it.

### 5.3 Events

| state | entered when | renders |
|---|---|---|
| COMPLETE | contiguous `seq` across connections, since the first message after launch, with every gap closed by a replay | normal |
| INCOMPLETE(since t) | a gap; a new generation; a fallback answer (protocol §5); or a socket close, until the reconnect's replay is done (§4.3: the baseline has reached the newest live `seq` and the request's mark, taken from `MemoryStatus.events_seq`) | history "incomplete since t"; open events "may be outdated" until their next transition |

*Superseded by D37.* The two states above, the header-driven fallback baseline (P17) and the mark-driven close (D26) are replaced by `EventHistory` (PLAN-PUBGEN §4.2, §4.4; protocol §5). Per publisher generation it reports coverage (`unknown`, `empty`, `completeThrough(n)`, or `missing` with pending and lost ranges), continuity across generations (`verified`, `broken`, `unknown` in legacy mode) and `caughtUp`. Coverage grows only by receipt, and a `seq` is lost only when a `SyncEnd` names it `unavailable` (D36).

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
4. Only CONFIRMED renders as a change (protocol §6 rule 7). An ACCEPTED event's overlay is removed only once the mirror's epoch is ≥ that event's `map_epoch` (protocol §4.2).
5. An event's state changes only on a message newer, in `seq`, than the last one applied to it, and only by a legal transition. An older message filled in by a replay goes to history only (protocol §5's per-event order). A violation keeps the last legal state (protocol §4).
6. Event coverage grows only by receipt of a message of the active publisher generation. A retrieval completes only once everything its own `SyncEnd` names has arrived, and a `seq` reads as lost only when a `SyncEnd` of that generation names it `unavailable` (protocol §5, D36, D37). (This replaces the earlier `EventsFallback` header invariant, D27.)
7. The mirror's memory is bounded:
   - one map, ≤ 16 MiB raw (D22);
   - an event list capped at 10,000 events, with older ones dropped. Dropping a display entry never changes coverage; coverage reads `unknown` only once the received-`seq` set outgrows its interval cap (PLAN-PUBGEN §4.4 invariant 11).

## 6. Failure modes

| id | failure | detection | on screen | recovery |
|---|---|---|---|---|
| F1 | Tailscale path falls back from direct to a DERP relay, "a last resort" ([T3]) | higher RTT; no protocol signal | none at 5 KB/s; a snapshot takes longer | none needed; soak logs the path type (`tailscale status`) |
| F2 | Tailnet down or Spark offline | socket and `:8766` fail | "connection lost"; map greyed STALE | P3 backoff |
| F3 | Dedicated bridge crashed or restarted | socket close | as F2, for seconds | §4.8 row 3 |
| F4 | `room_memory_cpp` crashed or restarted | topics unadvertised (UNAVAILABLE), then messages with a higher publisher generation (D34) | "Room memory not running on Spark", then the restart notice and STALE → snapshot | the mirror installs the new publisher generation (protocol §5, D37); reference and epoch reload from disk, the generation is allocated anew |
| F5 | Bridge backlog overflow on the memory socket | `seq` gap | STALE briefly | replay or snapshot. At ~1.2 messages/s, 1024 queued messages means ~14 min of the Mac not reading, so recurrence means a stuck client, not load |
| F6 | Lost trailing delta | status epoch mismatch ×2 (D20) | STALE within 2 s | request |
| F7 | Malformed or out-of-contract message | decoder throws | diagnostics count; STALE if it was on `map_bricks` | resync. Any non-zero count in a soak fails the stage |
| F8 | Protocol major mismatch after an update | fingerprint (§4.10) | INCOMPATIBLE, both versions shown | update the app, or the node publishes both majors |
| F9 | Rig bumped | guard → RIG_MOVED | red banner, overlays suppressed | §4.9 |
| F10 | One sensor silent | `per_sensor_hz` 0 → DEGRADED | amber banner naming the sensor | Trevor restarts that driver. With the extrinsic, events continue from the other sensor (D17). Before it (P14), a silent non-frame sensor changes nothing, and a silent frame sensor stops new events until it returns |
| F11 | Spark clock steps back | possible `event_id` seed collision after a restart (§2.3) | an `event_id` reused with another `kind`: protocol violation | the node refuses to start if its clock is earlier than the newest `first_seen` in its event log (PR 2.4) |
| F12 | Spark disk full | reference save fails | not visible today | proposed: DEGRADED with a reason in the `:8766` block (Q9); the node keeps serving from memory |
| F13 | Resync storm (flapping link, many Macs) | request counter; bytes per minute | slow recovery | P3, P4 |
| F14 | `:8766` relay down | POST fails | map stays STALE; "cannot request resync" | keyframes every 60 s (protocol D29); ssh tunnel restart (existing `RigService` path) |
| F15 | Large accept or re-reference | delta over half a snapshot | none | snapshot instead (P7) |
| F16 | Two Macs act on one event | compare-and-set fails for the second | the second sees the first's outcome | compare-and-set (protocol §3.6) |
| F17 | Wrong room | `room_id` ≠ `:8766` `room_id` | messages ignored (protocol §1.4); "room mismatch" diagnostic | fix the node's `room_id` parameter |

## 7. Test plan

### 7.1 Fixtures

| fixture | source | used by |
|---|---|---|
| hand CDR goldens, legacy and v2 (incl. `SyncEnd`) | `Tests/Fixtures/room/golden/hand/` (protocol §9) | codec tests (exist) |
| Spark CDR goldens | `scripts/spark/make_room_memory_goldens.py` with `rclpy` in the Jazzy container (Trevor runs it) | confirm D4; gate for S3 |
| Session traces | the C++ `tools/replay.cpp` (or the Python oracle) runs scripted `RoomMemory` sessions over the Phase 0 scenes: box added, removed and moved; walk-through; rig shifted. It writes every published message with topic, `seq` and bytes, as length-prefixed records under `Tests/Fixtures/room/sessions/` | `MemoryMirror` replay and fault tests |
| Bridge transcript | `serverInfo`, `advertise` and a few binary frames from the pinned 3.4.1 bridge, captured at S1 | fingerprint constants, advertise parsing, byte accounting |
| Lidar MCAP bags | `ros2 bag record` of both lidar topics on the rig, including the 12 Oct box takes (brief Q4). MCAP is the default storage on Jazzy ([B1]) | S1/S2 replay with `ros2 bag play` |

### 7.2 Mac unit and property tests (`LabCommandCoreTestsRunner`)

*Superseded in part.* The request-mark, fallback-header and spurious-new-generation cases below were written for D26, D27 and D23's restart cases. PLAN-PUBGEN §6 replaces them with a reference model, seeded traces and regression seeds for the generation-scoped mirror (D37). They are kept as the record of what each Codex round found.

- **Table tests.** One per row of §4.8, and each §5 transition. Also:
  - A reconnect sends each topic's saved `since`, not `SINCE_NONE`, and a replay that closes the gap leaves the events COMPLETE.
  - Disconnect at events `seq` 10, events 11–12 published, reconnect after events stop: `events_seq` is 12, so the request stays open although the baseline equals the newest live `seq`, and closes only once 11–12 are applied.
  - Codex's PR #13 case: a Mac applies 10 from a replay, the rest of that replay is lost, and it misses live 11–12. Another Mac's replay starting at 10 then arrives. The 10 is ignored, and the request stays open until 11–12 are applied.
  - A quiet room after a replay filled 11–12 from the sync topic: `status` keeps reporting 12, and no gap is flagged.
  - A fallback under steady events (2 per second): it ends once its header's `seq`s have arrived and the mark is reached, even though later events arrive live.
  - Codex's later PR #13 cases. At baseline 10, another Mac's replay starting at 12 arrives before the Mac's own replay: 11 is not given up, and the request closes only once its own replay brings 11. A fallback's first message, then live 201 and a mark of 201, arrive while the rest of the fallback is in flight: the baseline does not move until every `seq` the header names has arrived.
  - A lost header: nothing is given up, and the 5 s repeat brings a new fallback and header. A header for a `since` the Mac did not send is ignored.
  - At baseline 10, the Mac's own replay of 11 onward is still in flight on the sync topic when another Mac's header for the same `since` arrives, naming only `seq`s the Mac already has from live. 11 is not given up until the header's last kept `seq` arrives on the sync topic, by which time the replay's 11 has arrived or been lost.
  - The cached status delivered twice while a request is outstanding sets no mark, so the request does not close on it.
  - CONFIRMED at 11 lost, ACCEPTED at 12 applied after the gap: the replay's 11 enters the history, the baseline reaches 12, the event stays ACCEPTED, and no violation is logged.
  - ACCEPT while the bucket holds a delta: the event stays CONFIRMED with its overlay until the delta arrives, then ACCEPTED removes the overlay. An ACCEPTED that arrives before its delta keeps the region outlined "accepted · map updating".
- **Fault-injection property test.**
  - Take a session trace.
  - Apply a seeded random schedule of: drop; duplicate; reorder within 10; truncate one message; reconnect with `seq` continuing; node restart (`seq` reset, `map_epoch` + 1, as the node does with its state directory, protocol §5); an old snapshot answer arriving after a restart was recognized (the new snapshot must replace it, epoch first); an old `status` or live message arriving after the reset (one more, spurious, new generation that recovers); epoch jump; wrong `room_id`; fallback answers; lost, late or foreign `SyncEnd`s (formerly fallback headers); snapshot answers meant for another Mac; a relay that fails.
  - Run 10,000 schedules, and assert §5.5's invariants 1–6 after every step.
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
- **Writes go over ssh (P11).** `RequestSync` and `OperatorAction` travel over the existing ssh tunnel to `:8766`, so only holders of an authorized ssh key can reach the relay. The relay cannot tell which key opened the tunnel. So the action log keys each action by `action_id`, and records `operator_id` only as claimed by the app (protocol §3.6).
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
| V2 | Does the bridge's transient_local replay reach only the new client, or every client on the sink? | duplicate `status`/`epoch` messages on CURRENT Macs (ignored and counted under protocol §5's `seq` rule); confirms §4.3's reason for volatile maps; checks §2.5's late-joiner behaviour |
| V3 | Message-backlog behaviour when the Mac stops reading: oldest dropped, then a `seq` gap and resync | F5 |
| V4 | Advertise and `serverInfo` bytes per connect on the dedicated bridge | §3.2 |
| V5 | On a test bridge with `clientPublish` on and a restrictive `client_topic_whitelist`, does a publish to a topic outside the whitelist arrive? | confirms the source read behind §8. P11 stands either way |

| stage | what | owner | go | no-go / rollback |
|---|---|---|---|---|
| **S0** Mac only | `RoomMemoryStore` (PR 1.1), `MemoryMirror`, session traces, §7.2 tests; memory mode behind a launch flag, off by default | Claude Code; Codex review | all §7.2 tests pass; 10,000 fault schedules with 0 invariant violations | flag stays off; nothing ships |
| **S1** bench stack | §7.3 on a dev machine; V1–V5 answered and written into this doc | Claude Code | V1–V5 answered; budget test passes with measured overheads; every §4.8 row recovers | if the relay is not acceptable (Q4), choose between `clientPublish` with grant-only control and P1-only operation, and re-review §8. If P12 is rejected, the bench uses the `:8765` bridge's image rebuilt with `labcommand_msgs` instead |
| **S2** Spark shadow | PR 2.3 and 2.4 merged, including the room frame (P14) and the bucket (P15); room-memory container running `room_memory_cpp` only, no bridge; memory topics recorded to MCAP | Trevor deploys | room frame in the state directory (both transforms, or `frame_sensor` alone until the extrinsic, Q10); every recorded `MapBricks` and `ChangeEvent` in it, with z up; 24 h with no crash; p99 window < 200 ms; static-room false CONFIRMED ≤ Q3 bar; rig nudge → RIG_MOVED and no events; Spark goldens decode on the Mac | `docker stop` the memory container; the lidar stack is untouched |
| **S3** read-only mirror | the dedicated bridge starts in the memory container (`address`, whitelist, `capabilities: []`); Tailscale grant for 8767; keyframes on (protocol D29, always; no relay yet); Mac connects with the flag on | Trevor (Spark, grant), Claude Code (app) | 24 h soak (§7.5): 0 wrong-current, 0 violations, ≤ 5 KB/s every hour; bridge kill → STALE ≤ 5 s → CURRENT within one P1 period (≤ 60 s) with no wrong frame | stop the dedicated bridge (the node keeps running); app flag off. If P12 is rejected: the `:8765` bridge image is rebuilt with `labcommand_msgs` and the memory topics are added to its whitelist, a scheduled bridge restart that interrupts LIVE; rollback is the previous pinned image |
| **S4** resync requests | `:8766` relay (P11) live; keyframes stay on | Trevor (`command_center.py`), Claude Code | a forced gap recovers by replay; a forced 2-minute outage recovers by one snapshot; storm test within P3/P4 bounds | disable the relay endpoint; the Mac behaves as at S3 (keyframes only) |
| **S5** operator actions | P6 accepted (the `.msg` and protocol §3.6 are in PR #13). **Built by D46:** the codecs and goldens in Swift, Python and C++, confirm/dismiss/accept in both engines (suppressions, the held ACCEPTED), both nodes' handler and log behind `operator_actions:=true`, and the app's buttons and outcomes. Still to do: `colcon build`, and LABEL, MARK_GROUND_TRUTH, PAUSE, RESUME and REREFERENCE (refused UNSUPPORTED) | Claude Code; Codex | accept bumps `map_epoch` on both ends and arrives as one delta followed by the ACCEPTED transition; an accept during a bucket hold keeps the overlay until its delta is applied; double-click is a no-op; a stale accept is refused and logged | disable the action endpoint; the app hides the buttons |
| **S6** soak and product bar | 7-day soak; PR 3.2 scripted sessions | Trevor, Claude Code | §7.5 passes all week; PR 3.2 bar met (Q2, Q3) | stay at S5 and fix; memory mode stays opt-in |

**Spark data rollback.**
- Before a re-reference, the node renames the current `reference.cdr` (§2.3) to `reference.<epoch>.cdr`, keeping the last 5. Until the migration of §2.3, the per-sensor `reference-<sensor>.cdr` files are renamed together, to `reference-<sensor>.<epoch>.cdr`.
- Restoring one is a rename and a node restart. The room's `map_epoch` file is not rolled back, so the restored reference loads under the next epoch (protocol §5). The restart allocates a new publisher generation, so every Mac installs it and resyncs (D34, D37).

## 10. Proposed amendments (for Trevor and Codex)

None of these is in effect until accepted. Each would change `ROOM-MEMORY-PROTOCOL.md`, `command_center.py` or the deployment in its own PR. There is one exception: P2, P6, P13, P16 and P17 are written into the protocol draft by this PR, as D24, D25, D23, D26 and D27, still for review. No node code changes with them. **Since superseded in part by protocol v2 (PLAN-PUBGEN, D34–D37):** P13's restart cases, P16's request mark and P17 as a whole; see each row.

| id | change | where | why |
|---|---|---|---|
| P1 | A live `map_bricks` snapshot every 60 s, always on. **Now protocol D29, implemented in both nodes by PR #14** | protocol §2, §5, D29 | a Mac without a working request path, including one whose own tunnel failed while the relay works, otherwise gets no map; ~1.1 KB/s |
| P2 | `status` and `epoch` durability transient_local, depth 1; every other topic explicitly volatile. **Written into protocol §2 by PR #13 (D24).** Node change: §2.5 | protocol §2 (D18, D24) | memory state on connect; duplicates are harmless on full-state topics |
| P3 | Mac: memory-socket reconnect floor 5 s, failure count not reset by connections under 60 s; request backoff 5 → 30 s | protocol §5 | bounds reconnect and request storms (§3.2) |
| P4 | Spark: ≤ 1 snapshot answer per topic per 15 s across clients; `events` answers through a shared 3 KB/s bucket with a 48 KB burst, repeats of a waiting `since` answered once; request inputs rate-limited | protocol §5 | the sync topics are shared, so answers multiply with clients |
| P5 | No `mask` on CANDIDATE transitions | protocol §3.2 (allowed; this makes it the rule) | walk-throughs would otherwise cost 2–3 KB/s |
| P6 | New `OperatorAction` message and `/room_memory/operator` topic (§4.11). **Written into protocol §3.6 and `OperatorAction.msg` by PR #13 (D25).** Codecs, goldens, the node handler and the `/room_memory/operator/action` service (off by default) are D46 (S5) | protocol §2, §3.6 (D25) | acknowledge, accept, dismiss, label, ground truth, pause, resume, re-reference |
| P7 | A snapshot instead of a delta whose payload would exceed half a snapshot's | protocol §3.1 | a large delta costs more than a snapshot and cannot be replayed cheaply |
| P8 | The Mac requests right after subscribing, without the 5 s wait, with each topic's saved `since` (`SINCE_NONE` only without a usable baseline, §4.3). **Revised by D37:** right after the connection's first status names the publisher generation, with `expected_generation` and a fresh token (D35) | protocol §5 | the node cannot see a later client's connect through a shared bridge subscription (§4.3) |
| P9 | Re-referencing after RIG_MOVED without alignment starts a new `room_id` generation | protocol §1.2, §4.1 | pre-move events are in a different frame (§4.9) |
| P10 | `:8766` `room_memory` block with `node_started_at` and `map_digest` (§2.4). **`node_started_at` is diagnostics only since D34** | `command_center.py` | identity, versions and an end-to-end consistency check at no stream cost |
| P11 | `RequestSync` and `OperatorAction` carried over `:8766` and relayed on the Spark, instead of bridge `clientPublish` | protocol D19, `command_center.py` | `client_topic_whitelist` is not enforced, so the bridge stays read-only; ssh authenticates the caller |
| P12 | A dedicated read-only foxglove_bridge in the room-memory container on its own port | protocol §2, D28 (open: Q5) | the bridge needs `labcommand_msgs`; LIVE untouched; tight whitelist; one-container rollback |
| P13 | One `seq` rule with publisher generations, replacing the two rules that collided; `node_started_at` (P10) is its out-of-band restart signal. **Written into protocol §5 by PR #13 (D23), and stated only there.** **Superseded in part by D34 and D37:** restart detection is the publisher generation in every stateful message; `node_started_at` is diagnostics only, no longer case 3 | protocol §5, D23, D34, D37 | after a node restart the old "always accepted" and "`seq` ≤ last applied is ignored" rules collided and left the Mac STALE. Accepting any snapshot instead would let another Mac's older answer rewind a current Mac |
| P14 | One fixed room frame per `room_id`. Each sensor's points and ray origin are transformed by a stored `T_room←sensor` before integration, and every published coordinate is in that frame. Until the JT128↔Mid-360 extrinsic exists, only `frame_sensor` contributes geometry. A changed `T_room←frame_sensor` is a new `room_id` generation, and a changed transform for the other sensor rebuilds the reference | protocol §1.2, D17; PR 2.2, 2.4 | both sensors otherwise publish in their own frames, which `MapBricks` cannot tell apart (§4.9). Needs no wire change |
| P15 | `map_bricks` byte bucket: 3 KB/s, 180 KB burst. A message is admitted at balance ≥ 0 and debits its full size. The node never publishes a `MapBricks` over 2 MB (protocol §7's cap made a node rule), so debt stays under 2 MB. A larger map is reported on `:8766`, not published. Held deltas are coalesced. `map_epoch`, `map_digest`, snapshot answers and the `OperatorAction` epoch check use the *published* reference while a delta is held. An ACCEPTED transition is held with the delta that carries its change and published right after it (protocol §4.2) | protocol §1.4, §3.3, §5, §7, D20; PR 2.4 | a snapshot larger than the bucket would otherwise never go out (§3.3); reporting the working epoch during a hold would trip D20 and cause resyncs; an ACCEPTED sent ahead of its delta would remove the overlay from a map that does not show the change yet |
| P16 | Events high-water mark. `MemoryStatus` gains `uint64 events_seq`, the newest live events `seq`. An events request is done once the baseline reaches it, so completion never depends on where an answer ends. **Written into protocol §3.3 and §5 by PR #13 (D26).** `events_seq` stays, scoped to a publisher generation; **the request mark is superseded by D36**: a request completes by receipt of what its `SyncEnd` names | protocol §3.3, §5, D26, D36 | without a high-water mark, a Mac that missed the last events before a quiet period would close its request at once and show a wrongly COMPLETE history (§4.3). An in-band end marker (the last message sent again) fails on the shared sync topic: after a lost marker, another Mac's answer starting at the same `seq` reads as the marker (Codex, PR #13) |
| P17 | Events fallback header. A new `EventsFallback` message on a new `/room_memory/sync/events_fallback` topic precedes every events fallback. It copies the request's `since` and names the `seq`s the fallback sends. The Mac gives up an events gap only on a header for a `since` it sent, once every named `seq` has arrived and the last one has also arrived on the sync topic. A replay no longer re-sends the `since` message. **Written into protocol §2, §3.8 and §5 by PR #13 (D27), with `ros/labcommand_msgs/msg/EventsFallback.msg`**; the codecs and golden are not written yet: the Mac codec with `MemoryMirror` (S0), the node publisher with the `RequestSync` handler (PR 2.4). **Superseded by D36:** `SyncEnd` (with `SeqRange`) on `/room_memory/sync/end` follows every answer, names its delivered and unavailable ranges, and carries the request's token (D35); `EventsFallback.msg` and `/room_memory/sync/events_fallback` are deleted; codecs, goldens and both nodes are done | protocol §2, §3.8, §5, D27, D36 | inferring a fallback from an answer's first message fails on the shared sync topic: another Mac's replay arriving first, or a lost `since` message, gave up messages a replay was still bringing. Live progress up to the mark does not show a fallback has arrived, so it closed with messages in flight (Codex, PR #13) |

## 11. Open questions for Trevor

1. **Is the rig static for v1?**
   - If yes, P9 and relocalization can wait, and RIG_MOVED only needs "put it back" or "re-reference".
   - If it moves between sessions, alignment to the old reference is required, and room generations become the normal case.
2. **What is the smallest object that must be detected: 5, 10 or 25 cm?**
   - It sets `voxel_size` (5 cm today), the minimum region size and T_commit.
   - D-125: a 2.9 cm box needs 0.5–1 s of accumulation even at 2 cm voxels.
   - It is also the "minimum detectable size" the view must state (protocol §6 rule 10).
3. **How many false alarms per hour are acceptable,** counted as false CONFIRMED events per hour in a static room? The brief proposes ≤ 1/h. The answer is the go/no-go threshold at S2 and S6.
4. **May `command_center.py` relay requests and operator actions (P11)?** If not, the alternative is bridge `clientPublish`. At the deployed bridge version, the Tailscale grant would then be its only control (§8). This and P12 are protocol Q5 (D28), which keeps both transports, the dedicated bridge on :8767 and :8765 with the :8766 relay, until Trevor chooses.
5. **What runs the `:8765` bridge, with which parameters?** In particular `address`, `capabilities` and `topic_whitelist`. If it uses the defaults, it accepts client publishing from the LAN and the tailnet today (§8).
6. **Should memory mode be on at launch?** LIVE is off at launch by policy. Memory mode costs ~1.8 KB/s, so it could run whenever the app is open, or be opt-in like LIVE.
7. **How many Macs?** With one, the shared sync topics stay cheap. With several, P4 is necessary rather than prudent.
8. **Is the full event history needed on the Mac,** beyond the last 100 messages? If so, it needs a paged read on `:8766` (§4.5) and a retention policy for the Spark's event log (brief R10).
9. **Should a failed reference save (disk full) put the node in DEGRADED?** That changes protocol §4.1's DEGRADED trigger.
10. **Will the JT128↔Mid-360 extrinsic (brief R1) exist by S2? Until then, which sensor is `frame_sensor`?** That sensor alone contributes geometry (P14), so a silent frame sensor stops new events (F10). Changing the frame sensor later starts a new room generation. Adding the other sensor's transform when the extrinsic lands does not: it rebuilds the reference within the same frame (§2.3).
11. **How large will real snapshots get?** The bucket (P15) delays the next `map_bricks` message by about (snapshot − 180 KB) ÷ 3 KB/s: none at the measured 59 KB, ~2 min at 500 KB, ~10 min at 2 MB. If maps approach the 500 KB target, the bucket's rate needs revisiting against the 5 KB/s mean.

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
