# Room memory protocol, v1 (Spark `room_memory` → LabCommand Mac)

Status: draft for PR 0.1. Codex has not signed it off. Trevor decided D5, D9 and D19 on 2026-10-14 (§8); he has not answered Q1–Q5 (end of this doc). PR #13 added D23–D24 (the `seq` rule and durability), D25 (`OperatorAction`), D26 (the events high-water mark, `MemoryStatus.events_seq`) and D27 (the events fallback header, `EventsFallback`) as proposals. PR #14 (the Spark node, merged before it) added D28–D33 and §3.7 (`Activity`). `pubgen` (PLAN-PUBGEN, approved by Astra) adds protocol v2: D34 (publisher generations), D35 (`RequestSync` token and expected generation), D36 (`SyncEnd`, which supersedes D27 and D26's request mark) and D37 (the generation-scoped mirror, which supersedes D23's restart cases); its follow-up adds D38 (the generation on `Activity` and `Epoch`). Issue #21 adds D39 (`RequestSync` as a service through the :8767 bridge). `rig-guard` (PR #19, PLAN.md) adds D40–D44 and §3.9 (`RigPose`, the rig-pose guard), numbered after issue #21's. §5 is the single normative statement of the resync rules. Sources: `RESEARCH_BRIEF.md` §2.2–§2.5 (2026-10-14). The `.msg` files are in `ros/labcommand_msgs/msg/`. The Swift codecs are `Sources/LabCommandCore/RoomMemoryMessages.swift` and `RoomMemoryCDR.swift`.

The Spark does all perception and keeps the memory. The Mac mirrors the reference map, the event list and the timeline, and renders them. It never infers a change itself. Five Spark → Mac topics carry the state, three sync topics carry answers to the Mac's resync requests, and two Mac → Spark topics carry those requests and the operator's actions. How they travel between the Spark and the Mac is open (§2, D28, Q5): a dedicated foxglove_bridge on :8767 in the room-memory container, or the existing foxglove_bridge on :8765 with the requests relayed over the ssh-authenticated command-center API on :8766.

## 1. Conventions

### 1.1 Wire format

- **Encoding.** Classic CDR (XCDR1), the encoding rmw_fastrtps uses on Jazzy. foxglove_bridge passes the bytes through unchanged.
- **Encapsulation header.** 4 bytes. `00 01` is little endian and `00 00` is big endian; the 2 option bytes are ignored. The live JT128 and Mid-360 frames already carry `00 01 00 00`. Decoders reject every other header, including XCDR2 (`00 07`, `00 09`, …), as *unsupported*.
- **Alignment.** Each primitive is aligned to its own size (1, 2, 4 or 8 bytes), counted from the byte after the header. A `uint64` therefore sits on an 8-byte boundary. Nested messages add no alignment of their own.
- **Strings.** `string` is a `uint32` length that includes the NUL, then the bytes, then the NUL.
- **Arrays.** `T[]` is a `uint32` element count, then the elements. `T[N]` is N elements with no count. `T[<=1]` is a count of 0 or 1, then the elements.
- **Trailing bytes.** The rmw layer may add up to 3 bytes of padding after the message, and decoders accept that. Four or more unread bytes make the message *invalid*.
- **Encoders.** The Mac's encoders always write little endian with option bytes 0. They refuse any message the decoder would reject.
- **One exception to the byte order.** `MapBricks.payload` is a `uint8[]` whose contents are packed little endian in either CDR byte order (§3.1).
- **Protocol v2 fields and layout detection (D34).** The v2 fields sit at the end of each message, after its last legacy field, so every legacy byte offset is unchanged. A decoder decides the layout from what is left there, never from a schema:
  - at most 3 unread bytes: the **legacy** layout;
  - padding to 8, then exactly the v2 fields (8 bytes for `MemoryStatus`, `MapBricks`, `Epoch` and `Activity`, 16 for a `ChangeEvent` with a mask, 8 without, 24 for `RequestSync`), then at most 3: **v2**;
  - anything else: *invalid*. Pad bytes are not checked, as with every alignment.
  - A `ChangeEvent` with a mask: the nested `MapBricks` is not at the end, so the outer decoder decides once, after the nested payload, for both messages (16 bytes: the mask's generation, then the parent's). A legacy event has a legacy mask and a v2 event a v2 mask; nothing else can be encoded.
  - A v2 message cut inside its v2 fields can read as a valid legacy message. The Mac fixes the layout per connection (§5), so such a message is a wire mismatch there, and a map one marks the map stale like a rejected one.
  - `SyncEnd` exists only in v2.
- **Not yet confirmed.** The 8-byte alignment of `uint64` is the XCDR1 rule, but no Spark-produced bytes with a 64-bit field have been checked yet. That is what the Spark goldens (§9) confirm.

### 1.2 Frames, units, time

- Coordinates are in the **room frame**, in meters. The room frame is fixed per `room_id`, and its z axis points up.
- `voxel_size` is fixed per room. The default is 0.05 m.
- A voxel's index is `floor(coordinate / voxel_size)` on each axis. This is the rule `VoxelKey.index` uses in the Swift `ChangeDetection` module on the `change-detection` branch. PR 2.1 requires the Spark to match it exactly.
- `builtin_interfaces/Time` is `int32 sec` then `uint32 nanosec`, measured from the Unix epoch. `nanosec` must be below 10⁹, or the message is *invalid*.

### 1.3 Bricks

- **Size.** A brick holds 8 × 8 × 8 voxels: 0.4 m on a side at the default voxel size.
- **Coverage.** Brick `(i, j, k)` covers voxels `8i … 8i+7` on x, and likewise `8j … 8j+7` on y and `8k … 8k+7` on z. `int16` brick indices cover about ±13 km at 0.4 m per brick.
- **Bit order.** Voxel `(x, y, z)` inside a brick is bit `x + 8y + 64z` of that brick's 64-byte mask. Bit `n` is byte `n >> 3`, bit `n & 7`, least significant bit first. So voxel (0,0,0) is byte 0 bit 0, and voxel (7,7,7) is byte 63 bit 7.
- **Per-voxel evidence.**

  | `known` bit | `occupied` bit | evidence |
  |---|---|---|
  | 0 | 0 | UNKNOWN |
  | 1 | 0 | free |
  | 1 | 1 | occupied |
  | 0 | 1 | *invalid* |

- **An absent brick is all UNKNOWN.** A brick that is all free is still sent, with `known` full and `occupied` empty (D5, decided). Every brick that is sent has at least one known voxel.
- **Canonical order.** Bricks are strictly increasing in `(i, j, k)`, compared on `i` first, then `j`, then `k`. Duplicate or out-of-order bricks are *invalid*.

### 1.4 Common fields

Every message starts with these four fields, in the order the brief lists them:

| field | type | unit | valid | if missing or invalid |
|---|---|---|---|---|
| `room_id` | `string` | — | non-empty UTF-8, no NUL | message rejected |
| `map_epoch` | `uint64` | epoch | bumped on every reference-map change and on every node start that loads saved references; in `RequestSync`, repeats `since_epoch` | always present; a stale message renders "—" |
| `seq` | `uint64` | count | +1 per message on that topic, starts at 0 per publisher generation; `0` inside `ChangeEvent.mask`; in `RequestSync` and `OperatorAction`, +1 per request or action from that client | ordered per (generation, topic) only (§5) |
| `stamp` | `builtin_interfaces/Time` | s, ns | `nanosec` < 10⁹ | `{0, 0}` renders "time —" |

A message whose `room_id` differs from the room being viewed is ignored.

**Publisher generation (D34).** Every Spark → Mac message (`MemoryStatus`, `MapBricks` including the `ChangeEvent.mask`, `ChangeEvent`, `SyncEnd`, `Activity`, `Epoch`, and `RigPose` with `rig_guard`) ends with `uint64 publisher_generation`: the generation of the node process that published it. It is durable and increasing per room (§5), the same for every message of one process, and never `GENERATION_UNKNOWN` (0). `seq` orders messages only within one (generation, topic). A replay resends the original bytes, so it carries the original generation, which is the answering process's: the logs live in memory for one process. `map_epoch` stays a map revision, independent of the generation. "Publisher generation" is not Phase 3's `room_id` generation (P9, `lab` → `lab~2`); this doc always says "publisher generation" in full. `Activity` and `Epoch` carry it too (D38): they are transient_local and their `seq` restarts at 0 with each process, so without it a Mac holding the old run's higher `seq` would take the new run's messages for stored copies. They are full state, so the generation only scopes them; they never decide a connection's layout (§5.3). Saved reference files are `MapBricks` in the legacy layout (§5).

### 1.5 Missing values

Every field has one of three "missing" encodings:

- **float32 measurements:** NaN means *not available*. ±Inf is *invalid*. In Swift these are Optionals, so a missing value cannot be read as 0.
- **Integers that can be unmeasured or absent:** a named sentinel. v1 has five:
  - `HZ_UNAVAILABLE = 65535` in `per_sensor_hz`, where `0` is a measured zero.
  - `SINCE_NONE = 2⁶⁴ − 1` in `RequestSync`, where `0` is a real epoch or seq.
  - `EVENTS_SEQ_NONE = 2⁶⁴ − 1` in `MemoryStatus.events_seq`, where `0` is a real seq.
  - `SEQ_NONE = 2⁶⁴ − 1` in `SyncEnd.bound`: nothing published on the topic in this generation (known empty).
  - `GENERATION_UNKNOWN = 0` in `RequestSync.expected_generation`: the Mac's `since` belongs to no known generation. It is *invalid* as the `publisher_generation` of any Spark → Mac message, which is what rejects a legacy message with 8 zero bytes appended.
  - In `OperatorAction`, `EVENT_NONE = 2⁶⁴ − 1` and `STATE_NONE = 255`, the room-level target and its expected state.
- **Counters, epochs, enums, ids:** they cannot be missing inside a valid message. If the message itself is missing or stale (§5), they render "—".

Decoders reject an out-of-range enum value as *invalid*, and the whole message is dropped. A dropped message is never partly applied. The Mac counts it in a diagnostics line ("n room-memory messages rejected").

## 2. Topics

Topic names are a proposal for PR 2.4 (D18):

| topic | direction | message | rate | QoS (proposal): reliability, durability, history |
|---|---|---|---|---|
| `/room_memory/map_bricks` | Spark → Mac | `MapBricks` | snapshot when TRACKING starts and then every `keyframe_period_s` (60 s); deltas at least `delta_period_s` (5 s) apart, only when the reference changed (D29) | reliable, **volatile**, keep-last 10 |
| `/room_memory/events` | Spark → Mac | `ChangeEvent` | one per lifecycle transition, typically < 1/min | reliable, **volatile**, keep-last 100 |
| `/room_memory/status` | Spark → Mac | `MemoryStatus` | 1 Hz | reliable, **transient_local**, keep-last 1 |
| `/room_memory/activity` | Spark → Mac | `Activity` | 1 Hz, with each status (D33) | reliable, **transient_local**, keep-last 1 |
| `/room_memory/rig` | Spark → Mac | `RigPose` (`labcommand_msgs/msg/RigPose.msg`) | with `rig_guard` only: on every change of the guard's outcome or decision, and every 10 s | reliable, **transient_local**, keep-last 1 |
| `/room_memory/epoch` | Spark → Mac | `Epoch` | 0.2 Hz | reliable, **transient_local**, keep-last 1 |
| `/room_memory/request_sync` | Mac → Spark | `RequestSync` | on (re)connect and on a gap, ≤ 1 per topic per 5 s | reliable, **volatile**, keep-last 10 |
| `/room_memory/operator` | Mac → Spark | `OperatorAction` | one per operator action | reliable, **volatile**, keep-last 10 |
| `/room_memory/sync/map_bricks` | Spark → Mac | `MapBricks` | answers to `TOPIC_MAP_BRICKS` requests | reliable, **volatile**, keep-last 10 |
| `/room_memory/sync/events` | Spark → Mac | `ChangeEvent` | answers to `TOPIC_EVENTS` requests | reliable, **volatile**, keep-last 200 |
| `/room_memory/sync/end` | Spark → Mac | `SyncEnd` | one manifest per answered `RequestSync`, after the answer's messages (§5, D36) | reliable, **volatile**, keep-last 10 |

**Who publishes.** The room-memory node publishes the eight Spark → Mac topics (nine with `rig_guard`: `rig`) and is the only subscriber of the two Mac → Spark topics. The Mac originates `RequestSync` and `OperatorAction`. `RequestSync` reaches the node as the `/room_memory/sync/request` service, which the :8767 bridge exposes over the Mac's own socket (D39), or on the `request_sync` topic from a relay. How `OperatorAction` reaches it is still open (D28, Q5, below): an ssh-authenticated relay on the command-center API (`docs/phase3-live-integration.md`, P11) is the candidate. Every path carries the CDR bytes unchanged.

**Durability (D24).** Every publisher and subscriber declares durability explicitly.
- **`status`, `activity` and `epoch` are transient_local, depth 1.** A subscriber that joins late gets the last message published, then the live stream. That subscriber may be foxglove_bridge when the first Mac subscribes. All three are full-state messages, so a copy delivered twice is harmless: §5's `seq` rule ignores it.
- **`rig` (with `rig_guard`) is transient_local, depth 1** (D42), for the same reason: `RigPose` is full state, published on change and every 10 s, so a late subscriber gets the guard's current pose at once.
- **Every other topic is volatile.** A late joiner gets nothing published before it subscribed.
  - `map_bricks` and `events`: the Mac gets the map and the recent events by `RequestSync` (§5).
  - The sync topics, `sync/end` included: an answer is for the requests outstanding when it is sent, not for a later subscriber.
  - `request_sync` and `operator`: a restarted node must not receive a stored request or action again. A replayed `OperatorAction` would act twice.
- **The nodes match this.** The Python node (`spark/room_memory`) and the C++ node (`spark/room_memory_cpp`) declare every topic's durability: `status`, `activity` and (with `rig_guard`) `rig` transient_local with depth 1, the others volatile (`docs/phase3-live-integration.md` §2.5). Neither publishes `epoch` yet: its counts are of CONFIRMED events (D13), nothing reaches CONFIRMED before PR 2.3, and cumulative counts "since the room was created" also need saving across restarts. The topic's QoS above is the rule for when it is published.
- In ROS 2 a volatile subscriber still receives from a transient_local publisher. A transient_local subscriber receives nothing from a volatile one.

**Transport (D28, open: Q5).** Two options are on the table, and Trevor has not chosen. Both carry the same CDR bytes, and the node answers whatever arrives on `/room_memory/request_sync` or its `/room_memory/sync/request` service the same way, so the protocol does not depend on the choice.
- **A dedicated bridge on :8767** (implemented in PR #14's launch file, off with `foxglove_bridge:=false`). The Mac requests over the same socket, as a call of the node's request service (D39), or over a relay.
- **The existing bridge on :8765, with the requests relayed over the ssh-authenticated command-center API on :8766.** :8765 needs `labcommand_msgs` in its environment for the memory topics. `docs/phase3-live-integration.md` (P11) designs the relay.
- Phase 3's design combines the two: the :8767 bridge, kept read-only, for the Spark → Mac topics, and the :8766 relay for `RequestSync` and `OperatorAction` (`docs/phase3-live-integration.md`, P11, P12). It waits on the same choice. D39 replaces the relay for `RequestSync`: the bridge offers the node's request service and still cannot publish.

The :8767 option as implemented: the room-memory container starts its own foxglove_bridge on :8767 (launch argument `foxglove_bridge`, default on), with `topic_whitelist` `^/room_memory/.*`, so the raw clouds never leave through it, and `labcommand_msgs` in its environment, so Foxglove and the Mac get the full message definitions. Its capabilities are `connectionGraph` and `services`, with `service_whitelist` `^/room_memory/sync/request$`: a client can call the node's request service (D39) and no other service, and it cannot publish. foxglove_bridge parses `client_topic_whitelist` but does not enforce it (read in the source of 3.4.1 through 3.6.2; 3.6.0 is deployed), so turning on client publishing (`foxglove_client_publish:=true`) lets any client that reaches the port publish on any topic. The node answers whatever arrives on `/room_memory/request_sync`, so a relay that publishes the Mac's bytes there works the same way.

**Keyframes are the (re)connect snapshot (D29).** The bridge shares one ROS subscription across its clients, so the node cannot see a Mac (re)connect. The periodic keyframe on the live topic plays that role: a Mac that (re)connects holds a current map within `keyframe_period_s` without sending anything, and `RequestSync` makes it immediate where it can be sent.

## 3. Messages

The `.msg` text below is the legal ROS 2 version of §2.4. §8 (D1) lists every syntax change. Field names, types and order are as §2.4 gives them, except for `MapBricks`, whose brick arrays moved into a compressed payload (D9). §3.5 adds `RequestSync` (D19, D35), §3.6 adds `OperatorAction` (D25), §3.7 adds `Activity` (D33), §3.8 adds `SyncEnd` and `SeqRange` (D36), and §3.9 adds `RigPose` (D42). Protocol v2 appends `publisher_generation` (D34) to §3.1–§3.4 and §3.7 (D38), and §3.9. The *missing* column says how "not available" travels on the wire, and the *rendering* column says what the Mac shows then. "Stale" is the message-level rendering of §5.

### 3.1 `MapBricks`: snapshot or delta

```
uint8 KIND_SNAPSHOT=0
uint8 KIND_DELTA=1
uint8 COMPRESSION_NONE=0
uint8 COMPRESSION_ZLIB=1
uint8 COMPRESSION_LZFSE=2
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 compression
uint8 kind
uint64 base_epoch
float32 voxel_size
uint8 brick_dim
uint32 n_bricks
uint32 n_removed
uint8[] payload
uint64 publisher_generation
```

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `compression` | `uint8` | enum | `COMPRESSION_*`; `ZLIB` by default | cannot be missing | other value: *invalid*, message rejected |
| `kind` | `uint8` | enum | `KIND_SNAPSHOT`, `KIND_DELTA` | cannot be missing | other value: message rejected, map keeps its prior state |
| `base_epoch` | `uint64` | epoch | delta: the epoch it applies to, < `map_epoch`; snapshot: 0 (ignored) | cannot be missing | delta with `map_epoch` ≤ `base_epoch`: *invalid*; delta on a different epoch: resync (§5) |
| `voxel_size` | `float32` | m | finite, > 0 | NaN is *invalid* | message rejected; map "unavailable" until a valid snapshot |
| `brick_dim` | `uint8` | voxels per edge | 8 | cannot be missing | other value: *unsupported*, map "unavailable" |
| `n_bricks` | `uint32` | bricks | bricks in `payload` | cannot be missing | a payload of another size: message rejected |
| `n_removed` | `uint32` | indices | removed indices in `payload`; 0 in a snapshot | cannot be missing | as `n_bricks` |
| `payload` | `uint8[]` | bytes | `compression` applied to the packed arrays below | — | does not decompress to exactly their size: message rejected |
| `publisher_generation` | `uint64` | generation | §1.4; ≠ 0; after the payload, padded to 8. Inside `ChangeEvent.mask` it repeats the parent's | absent: the legacy layout (§1.1) | 0: *invalid* |

The payload, before compression, is exactly `134 × n_bricks + 4 × n_removed` bytes. Its arrays are packed little endian in either CDR byte order, back to back with no padding:

| part | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `brick_ijk` | `int16[3 × n_bricks]` | brick index (0.4 m at 5 cm) | canonical order | an absent brick | its volume renders UNKNOWN (hatched/transparent), never as empty space |
| `occ_bits` | `uint8[64 × n_bricks]` | bitmask | 64 B per brick, ⊆ `known_bits` | bit clear | voxel is free if known, else UNKNOWN |
| `known_bits` | `uint8[64 × n_bricks]` | bitmask | 64 B per brick, ≥ 1 bit set per brick | bit clear | voxel UNKNOWN, hatched |
| `removed_bricks_idx` | `uint32[n_removed]` | index | delta only; strictly increasing indices into the `base_epoch` brick list in canonical order | empty | a removed brick becomes UNKNOWN, never free |

**Compression (D9).**
- `COMPRESSION_ZLIB` (1), the default for every `MapBricks`, including deltas and event masks. The payload is a raw DEFLATE stream (RFC 1951): no zlib header and no Adler-32 trailer.
  - The Mac writes it with Foundation `NSData.compressed(using: .zlib)`, which is Apple's Compression framework at level 5.
  - Python reads and writes it like this:

    ```python
    raw = zlib.decompress(payload, -15)
    c = zlib.compressobj(6, zlib.DEFLATED, -15)
    payload = c.compress(raw) + c.flush()
    ```

  - Plain `zlib.compress(raw)` writes the zlib-wrapped format (`78 9c` … Adler-32), which the Mac rejects as *invalid*.
  - Plain `zlib.decompress(payload)` fails on the Mac's stream with "incorrect header check".
  - Both directions were checked on this Mac (Python 3.9 zlib and Apple, on a 547 KB payload; §9).
- `COMPRESSION_LZFSE` (2). The payload is an Apple LZFSE stream (Foundation `.lzfse`). Decoders accept it anywhere. Python's standard library has no LZFSE, so the Spark sends zlib.
- `COMPRESSION_NONE` (0). The packed arrays as they are. Use it for goldens and debugging only: a full snapshot is over the 500 KB target uncompressed (§7).
- Value 1 meant zstd in the earlier draft of this doc. zstd is dropped, because Foundation has none, and 1 now means zlib. Values ≥ 3 are *invalid*.
- **Empty payload.** With `n_bricks = n_removed = 0` the payload is empty under `NONE`. zlib and LZFSE still send a complete stream; for raw DEFLATE that is `03 00`. An empty compressed payload is *invalid*.

**Decoder checks.**
- Size limit:
  - `134 × n_bricks + 4 × n_removed` above 16 MiB (D22) is *invalid*, rejected before anything is inflated.
  - A compressed payload must be one complete stream that inflates to exactly that size. The decoder never inflates more than one byte past it.
  - An uncompressed payload must have exactly that length.
- What the Mac's inflater does not check. Apple's inflater checks neither the NLEN of a DEFLATE stored block nor for bytes after the end of the stream; Python's zlib checks both. Neither gap can make the Mac accept a payload of the wrong size, but the Spark must still send well-formed streams.
- Why there are no compressed byte goldens. DEFLATE and LZFSE output is not canonical: Apple's level 5 and Python's level 6 emit different bytes for the same input. So compressed messages are compared by decoded value, never by bytes (§9).

**Applying a message.**
- **Snapshot.** Replaces the mirror: bricks, `map_epoch`, and the applied `seq` (§5.3).
- **Delta.** Applies only when the mirror holds exactly `base_epoch` and `seq` is the next one. It removes the bricks at `removed_bricks_idx` first (indices into the `base_epoch` list), then replaces or inserts each listed brick at its coordinate. The mirror's epoch then becomes `map_epoch`. A delta that removes and re-sends the same brick is *invalid*; `RoomMemoryStore` checks that (PR 1.1).

**Wire size.**

```
4 + align8(C + 36 + P) + 8   bytes (v2)
4 + C + 36 + P               bytes (legacy)
```

- P is the payload length: `134n + 4r` uncompressed, with n bricks and r removed indices.
- C is the common-field size `align8(4 + L) + 24`, where L is the `room_id` UTF-8 length + 1. C = 32 for "lab", so a v2 message for "lab" is `4 + align8(68 + P) + 8` bytes (legacy 72 + P).
- **Saved references stay legacy.** `<state_dir>/<room_id>/reference-<sensor>.cdr` is a `MapBricks` snapshot. Both nodes write it in the legacy layout and read both layouts: a rolled-back image must still load it, and a generation in a file on disk means nothing.

### 3.2 `ChangeEvent`: one lifecycle transition

```
uint8 KIND_APPEARED=0
uint8 KIND_DISAPPEARED=1
uint8 KIND_MOVED=2
uint8 STATE_CANDIDATE=0
uint8 STATE_CONFIRMED=1
uint8 STATE_ACCEPTED=2
uint8 STATE_REVERTED=3
uint8 STATE_EXPIRED=4
uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint64 event_id
uint8 kind
uint8 state
float32[3] centroid
float32[6] aabb
float32[3] from_centroid
uint32 voxel_count
float32 volume_m3
float32 evidence
float32 alpha
uint8 sensors
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
MapBricks[<=1] mask
uint64 publisher_generation
```

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `event_id` | `uint64` | id | unique per room; repeated by every transition of one change | cannot be missing | — |
| `kind` | `uint8` | enum | `KIND_*`; fixed for the life of an `event_id` | cannot be missing | other value: message rejected |
| `state` | `uint8` | enum | `STATE_*`, legal transition (§4.2) | cannot be missing | other value: message rejected |
| `centroid` | `float32[3]` | m, room frame | finite | NaN is *invalid* | message rejected |
| `aabb` | `float32[6]` | m, room frame | `min_x, min_y, min_z, max_x, max_y, max_z`; finite; min ≤ max | NaN is *invalid* | message rejected |
| `from_centroid` | `float32[3]` | m, room frame | finite exactly when `kind = MOVED` | NaN × 3 when not MOVED | no arrow. A partial NaN, a MOVED without it, or a non-MOVED with it is *invalid* |
| `voxel_count` | `uint32` | voxels | any | cannot be missing | 0 is a real zero |
| `volume_m3` | `float32` | m³ | finite, ≥ 0; `voxel_count × voxel_size³` | NaN is *invalid* | message rejected |
| `evidence` | `float32` | fraction 0–1 | share of the region's voxels with free/occupied evidence | NaN = not computed | "evidence —", never 0 % |
| `alpha` | `float32` | probability 0–1 | calibrated false-alarm level from the research layer (RT1) | NaN = uncalibrated | "uncalibrated", never "α 0" |
| `sensors` | `uint8` | bitmask | bit 0 Mid-360, bit 1 JT128; other bits *invalid* | 0 = no attribution | "sensor —" |
| `first_seen` | `builtin_interfaces/Time` | s, ns | ≤ `last_seen` | `{0, 0}` | "first seen —" |
| `last_seen` | `builtin_interfaces/Time` | s, ns | ≥ `first_seen` | `{0, 0}` | "last seen —" |
| `mask` | `MapBricks[<=1]` | — | 0 or 1 map, `KIND_SNAPSHOT`, any `compression`. `occupied` = voxel in the region; `known` = voxel had evidence; header copies the parent's `room_id`, `map_epoch` and `stamp`, with `seq` 0, and its `publisher_generation` equals the parent's (D14) | count 0 | the region is drawn from `aabb` only, labelled "no mask" |
| `publisher_generation` | `uint64` | generation | §1.4; ≠ 0; after the mask (count or nested map), padded to 8 | absent: the legacy layout (§1.1) | 0, or not the mask's: *invalid* |

**Wire size.** Without a mask, `4 + C + 112` bytes: 148 B for "lab" (legacy `4 + C + 100`, 136 B). A mask adds its nested `MapBricks`, which starts 4-aligned, so its `map_epoch` may take 4 bytes of padding; in v2 its payload is followed by up to 7 pad bytes and the two generations. The 25 cm box event in §7 is 253 B legacy (measured) and 276 B v2 with its 2-brick mask zlib-compressed.

### 3.3 `MemoryStatus`: health, 1 Hz

```
uint8 STATE_BUILDING=0
uint8 STATE_TRACKING=1
uint8 STATE_RIG_MOVED=2
uint8 STATE_DEGRADED=3
uint8 STATE_PAUSED=4
uint16 HZ_UNAVAILABLE=65535
uint64 EVENTS_SEQ_NONE=18446744073709551615
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 state
uint32 n_bricks
uint32 n_open_events
float32 coverage_known_frac
float32 pose_residual_m
float32 window_s
uint16[2] per_sensor_hz
uint32 dropped_windows
uint64 events_seq
uint64 publisher_generation
```

`map_epoch` in §2.4's MemoryStatus is the common field and is not repeated (D2). `events_seq` is not in §2.4; D26 adds it, and D34 scopes it to the generation.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `state` | `uint8` | enum | `STATE_*` (§4.1) | cannot be missing | other value: message rejected; status stale after 3 s |
| `n_bricks` | `uint32` | bricks | bricks in the published reference map, the one `map_bricks` carries (D30) | cannot be missing | stale: "—" |
| `n_open_events` | `uint32` | events | events in CANDIDATE or CONFIRMED, as published: an accepted event counts until its ACCEPTED transition is published (§4.2) | cannot be missing | stale: "—" |
| `coverage_known_frac` | `float32` | fraction 0–1 | observed share of the room volume, defined below | NaN = room volume not yet known | "coverage —", never 0 % |
| `pose_residual_m` | `float32` | m | finite, ≥ 0; the rig-pose estimate defined below | NaN = guard has not run | "guard not run", never "0 m" |
| `window_s` | `float32` | s | finite, > 0 (0.5–1.0 expected) | NaN = not reported | "window —" |
| `per_sensor_hz` | `uint16[2]` | Hz, frames received per second, rounded | `[0]` Mid-360, `[1]` JT128 (same order as `sensors`) | `65535` = not measured | "—". `0` renders "0 Hz", a measured zero |
| `dropped_windows` | `uint32` | windows since node start | any | cannot be missing | stale: "—" |
| `events_seq` | `uint64` | count | `seq` of the newest message published on live `events` in this generation (§5). A v2 status with `EVENTS_SEQ_NONE` is the publisher's witness that this generation's event history is empty | `EVENTS_SEQ_NONE` = no event published yet in this generation | — |
| `publisher_generation` | `uint64` | generation | §1.4; ≠ 0; right after `events_seq`, already 8-aligned | absent: the legacy layout (§1.1) | 0: *invalid* |

**Definitions (v1, as the Spark computes them).**
- **`coverage_known_frac`.** Of the published reference map (D30): the number of known voxels (occupied or free) inside the axis-aligned voxel-index box of its occupied voxels, divided by the number of voxels in that box. The box stands in for the room volume until one is configured; voxels inside walls and furniture are in it and can never be observed, so the value stays well below 1 (0.17–0.18 on the JT128 recording). NaN while the reference has no occupied voxel. It is computed during BUILDING too, from the reference built so far.
- **`pose_residual_m`.** The size of the translation that best registers the latest window onto the reference, per sensor, then the largest over sensors. For each axis separately, the window's occupied voxels are shifted by −4 … +4 voxels and counted where they land on reference-occupied voxels. The best shift is refined by the parabola through that count and its two neighbours, unless it is ±4. The value is the length of the three per-axis shifts, in meters. Walls normal to an axis register that axis, and floors and ceilings do not mask a horizontal move. On the static recording it is 2.8 mm median (3.4 mm max). A real (2, 2, 0) cm move of the JT128 reads 26 mm, (5, 5, 0) cm reads 74 mm and 3 cm vertical reads 35 mm. It saturates near 0.35 m. NaN while BUILDING and when no shift overlaps at all. The memory stops learning from a window whose estimate exceeds `learn_max_offset_m` (0.02 m). With `rig_guard` on (default off), 2 windows above `pose_trigger_m` (1.5 cm) start a rig-guard check (PLAN.md): CANDIDATE promotion and learning are held while ICP decides, a confirmed move sets RIG_MOVED (§4.1) until the pose is re-aligned or the room relearned, and the guard reports on `/room_memory/rig`.

**Wire size.** `4 + C + 48` bytes: 84 B for "lab", 92 B for "lab-a" (legacy `4 + C + 40`: 76 B, 84 B).

### 3.4 `Epoch`: timeline tick, 0.2 Hz

```
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint32 n_appeared
uint32 n_disappeared
uint32 n_moved
float32 coverage_known_frac
uint8 state
uint64 publisher_generation
```

§2.4 gives Epoch as prose: "epoch id, stamp, counts of appeared/disappeared/moved, coverage_known_frac, state". The epoch id is `map_epoch` (D2). No node publishes `Epoch` yet (§2): its counts are of CONFIRMED events, and nothing reaches CONFIRMED before PR 2.3.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `n_appeared` | `uint32` | events | APPEARED events that reached CONFIRMED since the room was created; never decreases (D13) | cannot be missing | stale: "—" |
| `n_disappeared` | `uint32` | events | same, DISAPPEARED | cannot be missing | stale: "—" |
| `n_moved` | `uint32` | events | same, MOVED | cannot be missing | stale: "—" |
| `coverage_known_frac` | `float32` | fraction 0–1 | as in MemoryStatus | NaN | "coverage —" |
| `state` | `uint8` | enum | MemoryStatus `STATE_*` at this tick | cannot be missing | other value: message rejected |
| `publisher_generation` | `uint64` | generation | §1.4; ≠ 0; after `state`, 7 pad bytes to 8 (D38) | absent: the legacy layout (§1.1) | 0: *invalid* |

**Wire size.** `4 + C + 32` bytes: 68 B for "lab" (legacy `4 + C + 17`, 53 B).

### 3.5 `RequestSync`: resync request, Mac → Spark

```
uint8 TOPIC_MAP_BRICKS=0
uint8 TOPIC_EVENTS=1
uint64 SINCE_NONE=18446744073709551615
uint64 GENERATION_UNKNOWN=0
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint64 since_epoch
uint64 since_seq
uint8 topic
uint64 expected_generation
uint8[16] request_token
```

Trevor's D19 gives `RequestSync{room_id, since_epoch, since_seq, stamp}`. The common fields of §1.4 add `map_epoch` and `seq`, and `topic` says which topic's `seq` `since_seq` counts (D19).

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `map_epoch` | `uint64` | epoch | equal to `since_epoch` | cannot be missing | otherwise *invalid*; the Spark drops the request |
| `seq` | `uint64` | count | this client's request counter, +1 per request, from 0 | cannot be missing | — |
| `since_epoch` | `uint64` | epoch | `map_epoch` of the Mac's position on `topic` (§5.3): for `TOPIC_EVENTS` the message at the top of the run of received `seq`s from 0, for `TOPIC_MAP_BRICKS` the epoch the mirror holds | `SINCE_NONE` = the Mac holds no usable baseline for `topic` (§5) | `SINCE_NONE` in only one of `since_epoch`, `since_seq`: *invalid* |
| `since_seq` | `uint64` | count | `seq` of that message | `SINCE_NONE`, together with `since_epoch` | as above |
| `topic` | `uint8` | enum | `TOPIC_*` | cannot be missing | other value: *invalid* |
| `expected_generation` | `uint64` | generation | the publisher generation `since` belongs to; `GENERATION_UNKNOWN` (0) = none. The node honours `since` only when it equals its own (§5) | absent with the token: the legacy layout | — |
| `request_token` | `uint8[16]` | — | a random 128-bit token (UUID v4 bytes), new for every send, retries included; copied into the `SyncEnd` (D35). Not byte-swapped in big endian | absent with `expected_generation`: legacy | all zero in v2: *invalid*; only one of the two v2 fields: *invalid* |

`SINCE_NONE` is 2⁶⁴ − 1, not 0, because epoch 0 and seq 0 are real values. In Swift `since` is an Optional `Position`. The Spark answers by the rules in §5. A v2 node treats a legacy request (an old app) as `expected_generation` 0 with an all-zero token. A legacy node rejects a v2 request (≥ 4 trailing bytes), which is why the Mac sends no request on a legacy wire (§5).

**Wire size.** `4 + C + 48` bytes: 84 B for "lab" (legacy `4 + C + 17`, 53 B).

### 3.6 `OperatorAction`: one operator action, Mac → Spark

Proposed in PR #13 (D25). It is published on `/room_memory/operator` (§2), and the room-memory node is its only subscriber.

**Out of scope for protocol v2.** This section and `OperatorAction.msg` are a proposal. Protocol v2 (D34–D38) defines no codec, golden or handler for it, in any language, and its tests do not cover it. Codecs and goldens are Phase 3 stage S5, written together with the node's handler, for three reasons:
- **Nothing would use them.** No node subscribes to `/room_memory/operator`, and the Mac has no operator UI that sends an action.
- **No transport.** D39 carries `RequestSync` only. How an `OperatorAction` reaches the node is open (D28, Q5), and the bridge offers no `clientPublish` (§2).
- **The layout is not final.** An action is compare-and-set on `map_epoch` and `expected_state`, but not on the publisher generation. A REREFERENCE, PAUSE or RESUME sent just before a restart could act on the new process, whose `map_epoch` can equal the old one when no references were saved (§1.4). S5 decides whether the message gets an `expected_generation` (as `RequestSync` has, D35) before any bytes are frozen in a golden. The wire size below was computed by hand from §1.1, not encoded.

```
uint8 ACTION_ACKNOWLEDGE=0
uint8 ACTION_ACCEPT=1
uint8 ACTION_DISMISS=2
uint8 ACTION_LABEL=3
uint8 ACTION_MARK_GROUND_TRUTH=4
uint8 ACTION_PAUSE=5
uint8 ACTION_RESUME=6
uint8 ACTION_REREFERENCE=7
uint64 EVENT_NONE=18446744073709551615
uint8 STATE_NONE=255
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 action
uint64 event_id
uint8 expected_state
string text
string operator_id
string action_id
```

The common fields of §1.4 are the message's header. `std_msgs/Header` is not used: it would break "common fields first" (D2) and add a dependency.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `map_epoch` | `uint64` | epoch | the epoch of the map the operator was shown | cannot be missing | ACCEPT or REREFERENCE on another epoch than the node's published one: refused and logged |
| `seq` | `uint64` | count | this client's action counter, +1 per action, from 0 | cannot be missing | — |
| `stamp` | `builtin_interfaces/Time` | s, ns | when the operator acted; the time of a MARK_GROUND_TRUTH mark | cannot be missing | `{0, 0}`: *invalid* here, unlike §1.4, because a mark needs its time |
| `action` | `uint8` | enum | `ACTION_*` | cannot be missing | other value: *invalid* |
| `event_id` | `uint64` | id | the target event for ACKNOWLEDGE, ACCEPT, DISMISS and LABEL; an event or `EVENT_NONE` for MARK_GROUND_TRUTH; `EVENT_NONE` for PAUSE, RESUME and REREFERENCE | `EVENT_NONE` | the wrong one of the two for the action: *invalid*; an id the node does not know: refused and logged |
| `expected_state` | `uint8` | enum | the `ChangeEvent.STATE_*` the operator saw, when `event_id` is set; `STATE_NONE` when it is not | `STATE_NONE` | set when `event_id` is not, or the reverse: *invalid*. ACCEPT or DISMISS whose event is no longer in that state on the node, counting an accept not yet published: refused and logged |
| `text` | `string` | — | UTF-8, no NUL, ≤ 256 bytes; the label for LABEL, the description for MARK_GROUND_TRUTH, an optional note otherwise | `""` = no text | empty on LABEL or MARK_GROUND_TRUTH: *invalid* |
| `operator_id` | `string` | — | non-empty UTF-8, no NUL, ≤ 64 bytes; the name the operator gave the app | cannot be missing | empty: *invalid*. It is self-asserted, not authenticated |
| `action_id` | `string` | — | a client-generated UUID, 36 lowercase characters | cannot be missing | other format: *invalid*. A repeated `action_id` is ignored, so a retried send acts once |

| action | target | effect on the Spark | what the Mac sees next |
|---|---|---|---|
| ACKNOWLEDGE | an event, any state | the event log records that the operator saw it; no state change | nothing on the wire in v1 |
| ACCEPT | a CONFIRMED event | §4.2 "operator accept": CONFIRMED → ACCEPTED, the reference absorbs the change and `map_epoch` bumps | a delta or snapshot that carries the change, then the ACCEPTED transition with that message's `map_epoch` (§4.2) |
| DISMISS | a CONFIRMED event | §4.2 "operator revert": CONFIRMED → REVERTED; the reference is unchanged | the REVERTED transition |
| LABEL | an event | the event log stores `text` as the label of the event's region | nothing on the wire in v1 |
| MARK_GROUND_TRUTH | an event, or none | the event log stores a ground-truth mark at `stamp` with `text`, for scoring | nothing on the wire |
| PAUSE | the room | §4.1 any → PAUSED | `MemoryStatus.state` PAUSED |
| RESUME | the room | §4.1 PAUSED → the state the guard check gives | `MemoryStatus.state` |
| REREFERENCE | the room | re-baselining: §4.1 "operator re-references the room", RIG_MOVED → BUILDING; the new reference bumps `map_epoch`. §4.1 has no re-reference from other states, so there it is refused | `MemoryStatus.state` BUILDING, then a snapshot |

- **No reply message.** The outcome is what the Spark already publishes. ACKNOWLEDGE, LABEL and MARK_GROUND_TRUTH change nothing on the wire, so other Macs do not see them in v1. Showing them would need a field in `ChangeEvent`, which is a breaking change.
- **Compare-and-set.** `map_epoch` is compared with the epoch the node has published, not one it holds back. `expected_state` is compared with the node's own state of the event. That includes an accept whose ACCEPTED transition is still waiting for its map message (§4.2), so a second operator cannot dismiss an event that was already accepted. A refused action changes nothing. The node logs every action with its `action_id`, `operator_id`, arrival time and outcome.
- **Validation is symmetric (D16).** The node drops an *invalid* action and logs it. The Mac's encoder refuses to write one.

**Wire size.** 117 B for "lab" with an empty `text`, `operator_id` "trevor" and a 36-character `action_id`. Each string adds its UTF-8 length and up to 3 bytes of alignment.

### 3.7 `Activity`: transient motion next to the rig, 1 Hz

```
uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
bool active
uint8 sensors
uint8 largest_sensor
uint32 n_regions
uint32 voxel_count
float32[3] centroid
float32[6] aabb
float32 since_s
uint64 publisher_generation
```

Activity is never a change: nothing in it becomes a `ChangeEvent`, and `MemoryStatus` and `ChangeEvent` are unchanged (D33). An *activity region* is a change region of one sensor's tracking window that the window's own evidence supports and that either appeared new in that window or lost its shape (§4.2). Regions matched to an open event are not activity. `active` false means no such region, not a motionless room: still people, motion below `min_voxels` and motion inside an open event do not show.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `seq` | `uint64` | count | per-topic counter, +1 per message, from 0 per publisher generation | cannot be missing | — |
| `active` | `bool` | — | a tracking window since the previous `Activity` had activity | cannot be missing | other value than 0/1: message rejected |
| `sensors` | `uint8` | bitmask | `SENSOR_*` with activity in the newest such window; 0 when inactive | cannot be missing | undefined bit: message rejected |
| `largest_sensor` | `uint8` | enum | the `SENSOR_*` bit whose frame `centroid` and `aabb` are in (D30); one bit of `sensors`; 0 when inactive | cannot be missing | otherwise rejected |
| `n_regions` | `uint32` | count | activity regions in that window, all sensors; 0 when inactive | cannot be missing | — |
| `voxel_count` | `uint32` | voxels | their voxels (a move counts its new pose); 0 when inactive | cannot be missing | — |
| `centroid`, `aabb` | `float32[3]`, `float32[6]` | m | the largest activity region (most voxels; ties to the lower sensor bit), as a `ChangeEvent` would carry it | NaN ×3, NaN ×6 when inactive | partly NaN: rejected |
| `since_s` | `float32` | s | from the newest window with activity to `stamp`, ≥ 0 | NaN = none since the node started | negative: rejected |
| `publisher_generation` | `uint64` | generation | §1.4; ≠ 0; after `since_s`, 4 pad bytes to 8 (D38) | absent: the legacy layout (§1.1) | 0: *invalid* |

**Wire size.** `4 + C + 64` bytes: 100 B for "lab" (legacy `4 + C + 52`, 88 B).

### 3.8 `SyncEnd` and `SeqRange`: the end-of-answer manifest, Spark → Mac

Added by D36. The node publishes one `SyncEnd` on `/room_memory/sync/end` (§2) for every `RequestSync` it answers, after the answer's messages. It travels on its own topic, so it can arrive before them: receiving it proves nothing about receiving them (§5). Its `.msg` files are `ros/labcommand_msgs/msg/SyncEnd.msg` and `SeqRange.msg`. It replaces PR #13's `EventsFallback` header (D27), which no language implemented and no node published.

```
uint64 SEQ_NONE=18446744073709551615
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint64 publisher_generation
uint8[16] request_token
uint8 topic
uint64 bound
SeqRange[] delivered
SeqRange[] unavailable
```

`SeqRange` is `uint64 first`, `uint64 last`, inclusive, `first ≤ last`.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `map_epoch` | `uint64` | epoch | the node's published epoch when the answer was computed | cannot be missing | — |
| `seq` | `uint64` | count | +1 per `SyncEnd`, from 0 per generation | cannot be missing | — |
| `publisher_generation` | `uint64` | generation | the answering generation, which every message of the answer carries; ≠ 0 | cannot be missing | 0: *invalid* |
| `request_token` | `uint8[16]` | — | copied from the `RequestSync`; all zero for a legacy request | cannot be missing | — |
| `topic` | `uint8` | enum | `RequestSync.TOPIC_*`, copied | cannot be missing | other value: *invalid* |
| `bound` | `uint64` | count | the topic's newest live `seq` in this generation when the answer was computed | `SEQ_NONE` = nothing published on the topic in this generation (known empty) | — |
| `delivered` | `SeqRange[]` | seqs | the `seq`s this answer published, before this `SyncEnd`, on the topic's sync topic | `[]` | not canonical: *invalid* |
| `unavailable` | `SeqRange[]` | seqs | requested `seq`s the publisher no longer holds (events fallback only) | `[]` | not canonical, overlapping `delivered`, or on a map answer: *invalid* |

- **Validation (D16, symmetric).** Each list is canonical: ranges strictly increasing and non-adjacent (`next.first > prev.last + 1`), each `last ≤ bound`. `bound = SEQ_NONE` ⇒ both lists empty. The two lists are disjoint. `topic = TOPIC_MAP_BRICKS` ⇒ `unavailable` empty.
- **`unavailable` is permanent within a generation.** The node's log only moves forward, and an event's latest message only gets newer. A `seq` that was neither logged nor an open event's latest when the answer was computed can never be served later. So every Mac may record `unavailable` from any `SyncEnd` of the generation, whoever asked.
- **Layout.** A `SeqRange` count is a `uint32`; the first element's `uint64` does the alignment, so an empty list is followed at once by the next count.

**Wire size** for "lab", with d delivered and u unavailable ranges:

| d | u | size |
|---|---|---|
| 0 | 0 | 84 B |
| ≥ 1 | 0 | 88 + 16d B |
| 0 | ≥ 1 | 84 + 16u B |
| ≥ 1 | ≥ 1 | 92 + 16d + 16u B |

### 3.9 `RigPose`: the rig-pose guard, Spark → Mac

Added by D42. With `rig_guard` on (default off) the node checks whether the rig moved (IMU shock or tilt, `pose_residual_m` above `pose_trigger_m`, a periodic check, a start with saved state, a sensor back from silence), re-aligns each moved sensor's pose against its own reference, or relearns the room when it cannot (PLAN.md). `RigPose` on `/room_memory/rig` (§2) reports that: the outcome, what started the check, each sensor's result and absolute pose, and the last check's metrics. It is published on every change of the outcome or the decision and every 10 s. Its `.msg` is `ros/labcommand_msgs/msg/RigPose.msg`. A Mac that does not subscribe loses nothing it has today: `MemoryStatus.state` still says RIG_MOVED (§4.1).

```
uint8 OUTCOME_IN_PLACE=0
uint8 OUTCOME_CHECKING=1
uint8 OUTCOME_MOVED=2
uint8 OUTCOME_REALIGNED=3
uint8 OUTCOME_RELEARNING=4
uint8 OUTCOME_INCONCLUSIVE=5
uint8 CAUSE_IMU_SHOCK=1
uint8 CAUSE_IMU_TILT=2
uint8 CAUSE_POSE_OFFSET=4
uint8 CAUSE_PERIODIC=8
uint8 CAUSE_STARTUP=16
uint8 CAUSE_SENSOR_RESUMED=32
uint8 SENSOR_MID360=1
uint8 SENSOR_JT128=2
uint8 CHANNEL_NONE=0
uint8 CHANNEL_IN_PLACE=1
uint8 CHANNEL_MOVED=2
uint8 CHANNEL_REJECTED=3
uint8 CHANNEL_UNAVAILABLE=4
string room_id
uint64 map_epoch
uint64 seq
builtin_interfaces/Time stamp
uint8 outcome
uint8 causes
uint8 sensors
uint8[2] channel
uint64 frame_epoch
uint32 n_realigned
float32[7] mid360_pose
float32[7] jt128_pose
float32[2] delta_m
float32[2] delta_deg
float32[2] inlier_frac
float32[2] rms_m
float32[2] observe
float32 tilt_deg
bool imu_ok
builtin_interfaces/Time moved_at
uint64 publisher_generation
```

`outcome` is the guard's state: MOVED while any sensor is moved, CHECKING during a check, else the last decision. CHECKING and INCONCLUSIVE are not memory states (D44): a bump that moved nothing shows only here, as CHECKING then IN_PLACE, and `MemoryStatus` stays TRACKING. A pose maps the sensor's current frame into its reference frame, `p_ref = R p + t`; re-alignment changes it and nothing else (D40). Pairs are [0] Mid-360, [1] JT128.

| field | type | unit | valid | missing on the wire | rendering when missing / invalid |
|---|---|---|---|---|---|
| `seq` | `uint64` | count | per-topic counter, +1 per message | cannot be missing | — |
| `outcome` | `uint8` | enum | `OUTCOME_*`, 0–5 | cannot be missing | above 5: message rejected |
| `causes` | `uint8` | bitmask | `CAUSE_*` that started the current or last check; 0 before the first | cannot be missing | undefined bit: rejected |
| `sensors` | `uint8` | bitmask | `SENSOR_*` available (checked) in the decision this message reports | cannot be missing | undefined bit: rejected |
| `channel` | `uint8[2]` | enum | `CHANNEL_*` per sensor in that decision; `NONE` or `UNAVAILABLE` for a sensor outside `sensors` | cannot be missing | above 4, or another value outside `sensors`: rejected |
| `frame_epoch` | `uint64` | epoch | `map_epoch` at which the current room frame began; changes only on a relearn (D41) | cannot be missing | — |
| `n_realigned` | `uint32` | count | REALIGNED decisions since the frame began | cannot be missing | — |
| `mid360_pose`, `jt128_pose` | `float32[7]` | m, unit quaternion | `tx ty tz qx qy qz qw`; the quaternion's norm within 1e-3 of 1 | NaN ×7 = sensor not configured | partly NaN, ±Inf or another norm: rejected |
| `delta_m`, `delta_deg` | `float32[2]` | m, ° | size and angle (≤ 180) of the increment of the last applied estimate | NaN = none | negative or ±Inf: rejected |
| `inlier_frac`, `rms_m`, `observe` | `float32[2]` | 0–1, m, — | the last check's ICP result: inlier fraction, inlier RMS, λ_min of the point-to-plane information matrix over Σw | NaN = not run, or no registration cloud yet | negative, ±Inf, `inlier_frac` > 1: rejected |
| `tilt_deg` | `float32` | ° | IMU gravity against the gravity baseline of the last accepted decision, 0–180 | NaN = no IMU or no baseline | outside 0–180: rejected |
| `imu_ok` | `bool` | — | IMU samples within the last second | cannot be missing | other value than 0/1: rejected |
| `moved_at` | `Time` | — | first trigger of the current or last move | {0, 0} = none | — |
| `publisher_generation` | `uint64` | — | the publishing process's generation (§1.4, D34), the v2 tail; both nodes always send it | absent: the legacy layout (no node publishes it) | 0: rejected |

**Wire size.** `4 + C + 144` bytes: 180 B for "lab" (legacy `4 + C + 132`: 168 B). `moved_at` ends 4 bytes short of 8-alignment, so 4 pad bytes precede `publisher_generation`.

## 4. State machines

The Spark owns both machines. The Mac renders them. Event transitions are judged in `seq` order within one publisher generation: each event shows its highest-`seq` transition received, whatever the arrival order or transport (§5.3). An older one, filled in later by a replay, goes to the event's history only, and is not a violation.

### 4.1 Memory state (`MemoryStatus.state`, `Epoch.state`)

| from | to | trigger |
|---|---|---|
| node start | BUILDING | no saved reference for `room_id` |
| node start | TRACKING | saved reference loaded and the rig-pose guard passes |
| node start | RIG_MOVED | saved reference loaded and the guard fails |
| BUILDING | TRACKING | reference established (coverage threshold set in PR 2.2); `map_epoch` bumps |
| TRACKING | RIG_MOVED | guard: move evidence from a check (a fit ICP move, an IMU tilt once quiet, or a gross mismatch on every available sensor; D40). A check, or one that cannot decide, is not a state (D44) |
| DEGRADED | RIG_MOVED | the rig moved while a sensor was silent (RIG_MOVED outranks DEGRADED, D43) |
| RIG_MOVED | TRACKING | the guard has a stable, fit pose for every moved sensor: the old one (rig back in place) or a new one (re-aligned, same room frame and `map_epoch`, no map message; D40) |
| RIG_MOVED | DEGRADED | re-aligned while a sensor is still silent (D43) |
| RIG_MOVED | BUILDING | operator re-references the room; the new reference bumps `map_epoch` |
| RIG_MOVED | BUILDING | automatically, `relearn_after_s` (60 s) without an applicable alignment (`auto_relearn`, D41); never after an inconclusive check |
| BUILDING, TRACKING | DEGRADED | a sensor delivers no frames (`per_sensor_hz` 0) or windows are being dropped |
| DEGRADED | the state before DEGRADED | input restored |
| any | PAUSED | operator pause |
| PAUSED | BUILDING / TRACKING / RIG_MOVED | operator resume, re-entered through the guard check |

| state | change events | Mac rendering |
|---|---|---|
| BUILDING | none | "Building reference" with coverage; no change overlays |
| TRACKING | all transitions | normal |
| RIG_MOVED | no new CANDIDATEs; the open CANDIDATEs of each moved sensor go EXPIRED (D43); CONFIRMED events keep their state | red banner "Rig moved: changes suspended", residual shown; no change overlays from the current window |
| DEGRADED | continue, attributed by `sensors` to the sensors still delivering (D17) | amber banner naming the silent sensor |
| PAUSED | none; open events frozen | grey banner "Paused" |

### 4.2 Event lifecycle (`ChangeEvent.state`)

| from | to | trigger | Mac rendering of the new state |
|---|---|---|---|
| — | CANDIDATE | disagreement between the reference and recent layers where both have evidence that stays *settled* for `candidate_windows` (40) consecutive windows: each window's own evidence supports it (`fresh_min_voxels` 3 and `fresh_min_fraction` 0.5 of its voxels, both poses of a move), it keeps its shape (`stable_min_fraction` 0.9 of its voxels within `tolerance` of where they were when the count started, and back), and no activity region (§3.7) is within `activity_radius_m` (1.0 m). A stale window, a lost shape, nearby activity or a window without points starts the count again. Anything shorter sends nothing (D31, D33) | dashed outline, "candidate"; not counted as a change |
| CANDIDATE | CONFIRMED | persists ≥ T_commit (start at 5 s; tuned in PR 2.3) | solid overlay; counted; arrow for MOVED |
| CANDIDATE | EXPIRED | gone before T_commit (walk-through): no matching region for `expire_windows` (3) consecutive windows; superseded by a MOVED pairing; or memory left TRACKING for RIG_MOVED | removed from the overlay; history only |
| CONFIRMED | ACCEPTED | operator accept; the reference absorbs it and `map_epoch` bumps. The Spark publishes ACCEPTED only after the `map_bricks` message that carries the change, with that message's `map_epoch` | history entry. Once the mirror's epoch is ≥ the event's `map_epoch`: overlay removed, and the map shows the new state. Until then: region outlined "accepted · map updating", not counted as a change |
| CONFIRMED | REVERTED | the region returned to the reference state, or operator revert | removed; history only |

- ACCEPTED, REVERTED and EXPIRED are terminal. Only CONFIRMED renders as a change. CANDIDATE renders as a candidate, and REVERTED or EXPIRED never render as a change.
- An event's `kind` never changes. Moved pairing happens before confirmation: the paired CANDIDATEs go EXPIRED, and a new MOVED event starts at CANDIDATE (D17).
- A Mac that connects mid-session accepts any state as the first one it sees for an `event_id`. The same holds for the first message about an `event_id` after a gap in `events` (§5): the transitions in between may be lost.
- **ACCEPTED follows its map message.** Removing the overlay claims that the map now contains the change. So the Spark sends the map change first: the ACCEPTED transition is held until the delta or snapshot that carries the change is published, then published right after it. Several accepts merged into one delta, or overtaken by a snapshot, are all released after that message. Until then the event stays CONFIRMED on the wire. The two messages travel on different topics, and the map can be STALE, so the Mac also checks the epoch before it removes the overlay (table above).

## 5. Resync rule

From §2.4: *if the Mac sees a `seq` gap, or holds `map_epoch ≠ base_epoch` for a delta, it discards local state, renders "stale", and requests a `KIND_SNAPSHOT`. It never patches a map it cannot prove is current.*

`RequestSync` (§3.5) is that request. Its answer is the messages the Mac missed or a snapshot, then a `SyncEnd` (§3.8). A replayed delta is provably current, because it chains from the epoch the Mac holds and passes the same `seq` and `base_epoch` checks as a live one. So the Mac keeps its last good mirror while STALE instead of discarding it, and never shows it as current.

This section replaces the PR #13 rules (D23's restart cases 1–3, D26's request mark, D27's fallback header), which inferred restarts from counters and completeness from request flags and silence. With today's wire no client could tell "the publisher continued while I was away" from "it restarted and counted past my old counters". Protocol v2 makes identity explicit (D34), ties answers to requests (D35) and ends every answer with a manifest (D36); the Mac scopes all state to one generation (D37). PLAN-PUBGEN §4 gives the reducer's full state machine and invariants.

### 5.1 Publisher generations (D34)

- **The `seq` rule.** Within one publisher generation, each topic's `seq` starts at 0 and rises by 1 per message. Across generations, `seq`s say nothing about each other. A restart is a higher generation; nothing is ever inferred from counters.
- **Allocation.** Before it creates any publisher, a node computes `g = max(stored + 1, now_ms)`, where `stored` is `<state_dir>/<room_id>/publisher_generation` (decimal ASCII and a newline, next to `map_epoch`; 0 when absent) and `now_ms` the wall clock in ms. It writes `g` with a temp file, fsync and rename, fsyncs the directory, and only then publishes. Both nodes fsync the directory for `map_epoch` too. The file is separate from `map_epoch`, which does not bump on a start without saved references.
- **The wall-clock floor** keeps generations rising if the file is lost or the volume wiped. The residual is a lost file *and* a clock that went backwards past the old value.
- **Fail closed.** The node refuses to start, and logs why, when `state_dir` is empty or does not already exist (the allocator may create `<state_dir>/<room_id>`, never `state_dir`), the file is malformed (never silently reset), the value would overflow, the write or an fsync fails, or another live process holds `<state_dir>/<room_id>/publisher.lock` (`flock(LOCK_EX|LOCK_NB)`, held for the process lifetime: one authoritative publisher per room).
- **No in-process counter resets.** The seq counters are set only at construction; re-reference and rebuild do not reset them. A future in-process reset would need a new generation.
- **Injectable.** `RoomMemory` takes the generation as a constructor argument; only the ROS nodes call the allocator. The offline tools, the differential replay driver and the tests pass a fixed value.

### 5.2 How the Spark answers (D35, D36; both nodes, identical bytes)

Let `g` be the node's generation. `since` is honoured only when `expected_generation == g`; otherwise (0, older, unknown, or a legacy request) the request is answered as `SINCE_NONE` in `g`, and the `SyncEnd` tells the Mac the actual `g`. A request for another `room_id`, or one that does not decode, gets no answer and is logged.

**Map** (`bound` = the newest live map `seq`, or `SEQ_NONE` before the first live map message). The node keeps its last 10 map messages.

| case | `sync/map_bricks` | `SyncEnd` |
|---|---|---|
| no map yet (BUILDING) | nothing | `bound` NONE, `delivered` [] |
| `since` = (epoch, seq) of the newest logged message | nothing ("nothing missed") | `delivered` [] |
| `since` found in the log, older than `bound` | every later logged message, unchanged | `delivered` [[since + 1, bound]] |
| otherwise | one fresh snapshot of the last live state, `seq = bound` | `delivered` [[bound, bound]] |

**Events** (`bound` = `events_seq`). The node keeps its last 100 event messages and the latest message of each open event.

| case | `sync/events` | `SyncEnd` |
|---|---|---|
| `bound` NONE | nothing | `bound` NONE, both lists [] (known empty) |
| `since` found in the log | the later logged messages, unchanged (possibly none) | `delivered` [[since + 1, bound]], or [] |
| otherwise (fallback) | the latest message of each open event older than the log, then the whole log | `delivered` = those `seq`s as canonical ranges; `unavailable` = R ∖ `delivered`, with R = [since + 1, bound], or [0, bound] for `SINCE_NONE` or a mismatch |

**Publish order.** The answer's messages go out first, then the `SyncEnd`, on its own topic. `unavailable` is permanent within the generation (§3.8).

### 5.3 The Mac: one generation-scoped reducer (D37)

`RoomMemoryMirror` (`Sources/LabCommandCore/LiveRoom/`) holds one **active** generation state, the only state that renders as current, an archive of the newest 2 retired generations (display only), the highest generation ever installed, and one retrieval per topic. Validation (decode, `room_id`, the D16 rules) comes first; a rejected message changes nothing except the counters and, for a map message, the map's STALE state.

**Wire.** The layout (§1.1) of the first stateful message (`map_bricks`, `events`, `status`, a sync topic) decoded on a connection fixes it. A later message of the other layout on that connection is dropped and counted (`wireMismatch`); a map one also marks the map stale. Every reconnect re-decides. `Activity` and `Epoch` never fix it (D38): one that arrives before the layout is fixed, or in the other layout, is dropped and counted (`sideTopicLayout`), and changes nothing else. A node from before D38 publishes v2 status, map and events next to legacy `Activity`, and its transient_local `Activity` copy usually arrives first; letting it decide would run the Mac in legacy mode against a v2 node.

**Generations.**
- A v2 message of a generation above the highest installed **installs** it: the active state is archived (its tail is unknown: a retired generation has no final bound on the wire), a fresh empty state is created, and every retrieval goes idle. The message is then handled in it. Generations are durable and wall-clock floored, so a newer value always belongs to a later process; installing on the first validated message loses nothing and needs no buffer, and coverage cannot be complete or empty without a witness of that generation.
- A message below the highest generation changes nothing but a counter (`olderGeneration`), and never reactivates an archive.
- The highest generation again after a legacy connection archived it starts a fresh, empty state; the archive is never reactivated.

**Status.** A status of the active generation with `seq` above the held one replaces it, raises the events witness H (`events_seq`, or "empty" for `EVENTS_SEQ_NONE`) and, arriving on this connection, moves the connection to *observing* that generation. A `seq` not above the held one is a duplicate or a delayed copy (transient_local), never a generation signal. `status` is fresh for 3 s on the connection it arrived on.

**Events.** C is the set of `seq`s actually received, live or sync, from any Mac's answer; U the `seq`s any `SyncEnd` of the generation declared unavailable; H the highest witnessed `seq` (statuses, `SyncEnd` bounds). With top = max(H, max C) and missing = [0, top] ∖ C:
- *empty*: a v2 witness says nothing was published and C = ∅; *completeThrough(top)*: missing = ∅; *missing(pending, lost)*: pending = missing ∖ U, lost = missing ∩ U; *unknown*: no witness and C = ∅, or C outgrew its bound (then it never becomes more complete), or a legacy wire.
- A `seq` already in C with the same payload is a duplicate; with another payload a contract violation, counted, the first kept. Neither is a restart signal.
- Each event shows its highest-`seq` transition received, whatever the arrival order or transport.
- *caughtUp*: coverage is empty or complete, and a fresh status of the generation on this connection names an `events_seq` covering max C. It is an as-of observation.

**Map.** A snapshot of the active generation applies when the chain is empty or its `seq` ≥ the applied one (an older answer, possibly another Mac's, is ignored). A delta applies when `seq` = applied + 1 and `base_epoch` = the chain's epoch; at or below the applied `seq` it is a duplicate; otherwise the chain is broken (gap, wrong base). Within one generation a delta that chains after a reconnect is valid: its seqs are contiguous. The map is **current** when the chain is held in the active generation, it has applied every map `seq` the generation is known to have published (live or sync messages, `SyncEnd` bounds), no undecodable map message arrived since the last snapshot or completed retrieval, a status of the generation arrived on this connection with `map_epoch` equal to the chain's (two consecutive mismatches make it stale, D20), and the socket is open. Overlays are drawn only while it is current and the status is fresh. A newer generation with no map yet shows the earlier run's bricks greyed (`newGeneration`).

**Retrieval.** One per topic: idle, waiting(token), or receiving(manifest).
- **When to request.** Only while observing a generation on a v2 wire with a request path. Events: pending ≠ ∅, `since` = (`map_epoch`, `seq`) of the top of the run of C from 0, or `SINCE_NONE` when that run is empty or its epoch was evicted. Map: not current for a reason a request can fix (no chain unless a `SyncEnd` said there is no map yet and no later status is TRACKING or RIG_MOVED or changed `map_epoch` and the socket has not reopened since, a broken chain, a rejected message, D20 once per mismatch run); `since` = (epoch, applied `seq`), or `SINCE_NONE` on a retry. `expected_generation` is always the observed generation; every send, retries included, takes a fresh random token. At most one request per topic per 5 s.
- **Completion.** A `SyncEnd` with the outstanding token moves the retrieval to receiving; the manifest applies to its own generation (installed first if newer). It is done only once every `delivered` range is in C (events), or the chain has applied `max(delivered)` in that generation, or `delivered` is empty and the chain is at or above `bound` (map; `bound` NONE completes at once). Nothing else completes a retrieval: no timer, request flag or silence.
- **Facts.** Every `SyncEnd` of the active generation, whatever its token, raises H (events) or the known map `seq` (map) to its `bound` and adds its `unavailable` to U. A foreign or old token changes nothing else.
- **Timeouts.** 5, 10, 20, then 30 s per attempt in waiting or receiving: a fresh token if the need remains, idle otherwise. A request a client could not send is counted (`requestsUnsent`); the retrieval stays waiting for its timeout.
- **Connections.** Opening keeps the active and archived state (stale display) and kills every token; closing makes the map stale and leaves coverage untouched.

**Continuity.** *broken(since g_old)* once the mirror has archived a generation in which it observed anything; *verified* while it has observed only the active generation since launch; *unknown* on a legacy wire. A retired generation has no final bound, so the protocol never claims whole-history continuity across generations.

**Legacy wire (an old node, or a rolled-back image).** The active generation is archived and one legacy state shows bricks and events with identity unverified. The map is never current (`unverified`), coverage and continuity are unknown, caughtUp is false, and no request is sent (a legacy node rejects a v2 `RequestSync`). A legacy delta chains only on the connection whose snapshot started the chain; the map arrives with the 60 s keyframe (D29). No generation is ever synthesized from counters.

**Request path (D39).** The client prefers the bridge's `/room_memory/sync/request` service (type `labcommand_msgs/srv/RequestSync`), then a client channel when the bridge offers `clientPublish`, then a relay. The service is advertised after `serverInfo` and may come after the channels, and a node restart withdraws it and advertises it again, so the mirror's request flag changes with it on an open connection. A `serviceCallFailure` for a request counts it as unsent; the retrieval retries on its schedule.

**No request path.** Without the request service, client publishing or a relay (a node older than D39, or a bridge launched without `services`), retrieval cannot run: coverage typically reads `missing(pending: …)` after a mid-stream connect, the map becomes current from the next keyframe, and the UI says "no request path" instead of retrying.

**`status`, `activity`, `epoch`.** These are full-state messages, so a gap is not a resync, and the message is applied. All three are transient_local (§2, D24); within a generation a stored copy delivered again repeats its `seq` and is ignored, and a `seq` not above the held one is never applied. Each is scoped to its generation: a newer generation starts empty, so a restarted node's `seq` 0 applies (D38), and an older generation's message is ignored. On a legacy wire only an exact copy (`seq` and `stamp`) is a duplicate. Each is stale when its newest message on this connection is older than 3 periods: 3 s for `status` and `activity`, and 15 s for `epoch`.

## 6. Honest-state rendering rules

These extend the app's existing contract (missing ≠ zero, a running container is not a running stream) to the map.

1. **UNKNOWN ≠ free.** A voxel with no `known` bit, or inside an absent or removed brick, renders hatched/transparent ("not observed"). It is never drawn as empty space, and never counted as free.
2. **Absence of returns ≠ absence of object.** DISAPPEARED comes only from the Spark, on free-space evidence: rays now pass through a formerly occupied voxel. The Mac never derives a change from missing points.
3. **`alpha = NaN` ≠ 0.** NaN renders "uncalibrated". It is never "α = 0", which would claim a perfect false-alarm level, and never hidden as if it were zero. This follows the brief's recommendation in §5 Q11.
4. **Other NaN floats are "—", not 0.** `evidence`, `coverage_known_frac`, `pose_residual_m` and `window_s` render "—" or their named phrase ("guard not run"), never 0 % or 0 m.
5. **Rates.** `0` Hz is a measured zero (the sensor is silent). `65535` is "—" (not measured).
6. **Stale is not current.** A stale topic shows "STALE · \<age\>". Its last values are never presented as live.
7. **Only CONFIRMED is a change.** CANDIDATE is shown as a candidate. REVERTED and EXPIRED never render as changes. ACCEPTED is history. Until the mirror's epoch is ≥ the event's `map_epoch`, its region is outlined "accepted · map updating" (§4.2), so the map is not taken to contain a change it does not show yet.
8. **RIG_MOVED suppresses change overlays** from the current window. The banner gives the residual.
9. **No partial application.** A message that fails to decode, or fails the delta preconditions, changes nothing on screen except the diagnostics count and the STALE state.
10. **Minimum detectable size is stated.** The view states the smallest object size the room's `voxel_size` supports, with the figure from PR 3.2's scoring (brief R5). It does not imply that smaller changes would be seen.

## 7. Byte budget

**Budget (§2.5).** ≤ 5 KB/s mean and ≤ 2 MB per (re)connect in memory mode. A snapshot targets ≤ 500 KB, with a hard cap of 2 MB.

| stream | rate | message size ("lab", default zlib) | steady state |
|---|---|---|---|
| MemoryStatus | 1 Hz | 84 B (exact; legacy 76 B) | 0.08 KB/s |
| Activity | 1 Hz | 100 B (exact; legacy 88 B) | 0.10 KB/s |
| Epoch | 0.2 Hz | 68 B (exact; legacy 53 B), not published yet | 0.01 KB/s |
| ChangeEvent, no mask | per transition, < 1/min | 148 B (exact; legacy 136 B) | < 0.01 KB/s |
| ChangeEvent with a 25 cm box mask | per transition | 276 B (v2, from the measured 253 B legacy size) | < 0.01 KB/s |
| MapBricks delta, 25 cm box added | ≤ 0.2 Hz, on reference change | 380 B (v2, from the measured 367 B legacy size) | < 0.1 KB/s |
| MapBricks snapshot | TRACKING start, every `keyframe_period_s` (D29), and the answer to a request that cannot be replayed | 52,863–59,024 B (measured legacy, see below; v2 adds 8–15 B) | — |
| RequestSync (Mac → Spark) | on (re)connect and on a gap | 84 B (exact; legacy 53 B) | — |
| OperatorAction (Mac → Spark) | per operator action | 117 B (exact, for the §3.6 example) | — |
| SyncEnd | one per answered request | 84 B with no ranges, + 16 B per range (+ 4 B padding per non-empty list; §3.8) | — |
| RigPose (`rig_guard` only) | on change and every 10 s | 180 B (exact; legacy 168 B) | 0.02 KB/s |

**Measured on the real 5-minute static recording** `static5m` (`spark/room_memory_cpp/PLAN.md`), both lidars, node defaults, the JT128's reference as the map, before protocol v2 (v2 adds 8 B per status, 12 B per activity and up to 15 B per map message): 0.89 KB/s mean over all room-memory topics. Tracking teaches the reference nothing (`learn_windows` 0), so there are no deltas, and Trevor's motion next to the rig is activity, not events.

| stream | messages | bytes | mean |
|---|---|---|---|
| MapBricks snapshot (keyframe, 43 KB) | 5 | 217,080 | 0.73 KB/s |
| MapBricks delta | 0 | 0 | 0 |
| MemoryStatus | 299 | 22,724 | 0.08 KB/s |
| Activity (71 of them active) | 299 | 26,312 | 0.09 KB/s |
| ChangeEvent | 0 | 0 | 0 |

**Measured** by the size test in `RoomMemoryCDRTests.swift`.
- The input is a full-sphere synthetic accumulation of the furnished 8 × 8 × 3 m room: 720 × 360 rays, ±1 cm range noise, seed 1, 5 cm voxels.
- Every brick with any evidence is sent, all-free ones included (D5).
- The sizes are whole encoded messages, the payload compressed as the column says. zlib is Apple's raw DEFLATE at level 5. Python's level 6 on the Spark gives sizes that differ by a few percent.

| message | bricks | none | zlib (default) | lzfse |
|---|---|---|---|---|
| snapshot, floor at z = 0 on a brick boundary | 4,081 | 546,926 B | 59,024 B | 54,091 B |
| the same room shifted (0.2, 0.2, 0.1) m off the brick grid | 3,412 | 457,280 B | 52,863 B | 48,816 B |
| snapshot, surface bricks only (the brief's size model, not v1) | 2,211 | 296,346 B | 53,375 B | 51,194 B |
| delta: 25 cm box added | 12 | 1,680 B | 367 B | 413 B |
| APPEARED event, mask compressed as the column says | 2 (mask) | 476 B | 253 B | 293 B |

**Reading the numbers.**
- The default zlib snapshot is **53–59 KB**, 8–9× under the 500 KB target and 34× under the 2 MB cap.
- Keeping all-free bricks (D5) costs little once compressed.
  - Raw, it takes the snapshot from 2,211 surface bricks to 4,081 bricks, 296 KB to 547 KB.
  - With zlib it costs 5.6 KB (53.4 KB → 59.0 KB), because all-free bricks compress to almost nothing.
- Uncompressed, raw size depends on where the brick grid sits relative to the room (D6):
  - With the floor and two walls exactly on brick boundaries, ±1 cm noise splits their hits across two brick layers: 4,081 bricks, **547 KB, over the 500 KB target**.
  - With the room off the grid it is 3,412 bricks and 457 KB.
  - So `COMPRESSION_NONE` is for goldens and debugging only.
- A (re)connect costs one snapshot plus the latest status and epoch, about 0.06 MB with zlib. The 2 MB per-(re)connect budget leaves room for the events fallback (§5.2), at most (open events + 100) × a few hundred bytes plus its `SyncEnd`.

## 8. Decisions for review

Each item is a place where §2.4 was not legal ROS 2, was underspecified, or conflicted with the honest-state rules. Each states what this doc does and why. All need Codex review. Trevor decided D5, D9 and D19 on 2026-10-14; the parts of D19 marked below go beyond his decision and need his confirmation. D28–D33 come from the Spark implementation (Phase 2b, PR #14) and the static recording. D23–D27 are PR #13's.

- **D1. ROS 2 syntax.** §2.4's text was made legal ROS 2. Names, types and order are unchanged except in `MapBricks`, where D9 moved `compression` ahead of the brick arrays and packed those into `payload`. The syntax changes:
  - Constants go one per line (`KIND_SNAPSHOT=0, KIND_DELTA=1` became two lines).
  - Fields joined with `;` go one per line.
  - ROS 1 `time` became `builtin_interfaces/Time` (`int32 sec`, `uint32 nanosec`), for `stamp` as well.
  - `per_sensor_hz[2]` became `uint16[2] per_sensor_hz`.
  - "`MapBricks mask # optional`" became `MapBricks[<=1] mask`, since ROS 2 has no optional fields.
  - Named constants were added for values §2.4 described in comments: `COMPRESSION_*`, `SENSOR_*`, `HZ_UNAVAILABLE`. `RequestSync` (D19) adds `TOPIC_*` and `SINCE_NONE`. `OperatorAction` (D25) adds `ACTION_*`, `EVENT_NONE` and `STATE_NONE`. D26 adds `EVENTS_SEQ_NONE`.
- **D2. Common fields.**
  - They are `string room_id`, `uint64 map_epoch`, `uint64 seq` and `builtin_interfaces/Time stamp`, first in every message and in the brief's order.
  - They are written inline rather than as a shared header message; the wire bytes are identical either way.
  - MemoryStatus's own `map_epoch` is the common one, and Epoch's "epoch id" is `map_epoch`.
  - `RequestSync` carries them too: `map_epoch` repeats `since_epoch` and `seq` counts the client's requests (D19).
  - So does `OperatorAction`: `map_epoch` is the epoch the operator was shown, and `seq` counts the client's actions (D25).
  - `room_id` is a non-empty UTF-8 string. Its format, such as a slug, is not fixed.
- **D3. Enum values** follow §2.4's listing order from 0. Unknown values reject the message instead of mapping to a default.
- **D4. Classic CDR (XCDR1) with 8-byte `uint64` alignment.** This is an assumption, to be confirmed by the Spark goldens (§9). The ≤ 3 trailing bytes rule comes from rmw padding.
- **D5. Absent brick = UNKNOWN; all-free bricks are sent.** Decided by Trevor, 2026-10-14. §2.4's `removed_bricks_idx` comment says bricks "that became all-unknown/all-free".
  - Removing an all-free brick would turn free evidence into UNKNOWN on the Mac, or make absence ambiguous. Either way it breaks UNKNOWN ≠ free.
  - So removal means "became all-unknown" only, and all-free bricks are sent.
  - Cost, measured (§7): 4,081 instead of 2,211 bricks. That is 547 KB instead of 296 KB raw, but only 59.0 KB instead of 53.4 KB with the default zlib.
- **D6. Voxel and bit layout.** Voxel index `floor(coordinate / voxel_size)`; brick `(i, j, k)` covers voxels `8i … 8i+7` (likewise for j, k); bit `x + 8y + 64z`, LSB first. §2.4 did not specify these.
  - Where the room frame's origin sits relative to the floor and walls changes snapshot bytes by about 20 % (§7).
  - A floor at exactly z = 0 straddles a brick boundary. Choosing the room-frame origin (PR 2.2) is a size decision as well as a convenience.
- **D7. Canonical order.**
  - Bricks are strictly increasing in `(i, j, k)`, so that `removed_bricks_idx` (`uint32`, as specified) has a meaning: an index into the `base_epoch` brick list in that order.
  - A delta applies removals first, then upserts.
  - Brick coordinates would be sturdier than indices, at 6 B instead of 4 B each. That is left for review rather than changed.
- **D8. Brick invariants.** `occupied ⊆ known`, and every sent brick has ≥ 1 known voxel. A snapshot carries no removals. A delta's `map_epoch` exceeds its `base_epoch`, which §5's duplicate rule relies on.
- **D9. `compression` before the payload.** Decided by Trevor, 2026-10-14.
  - §2.4 had "`uint8 compression # 0 none, 1 zstd (whole payload)`" as the *last* field of `MapBricks`. A byte after the arrays cannot describe compression of the arrays before it, and typed CDR arrays leave a compressed stream nowhere to live.
  - So `compression` now comes right after the common fields. The four brick arrays are packed into one `uint8[] payload`, and `n_bricks` and `n_removed` stay outside it.
  - Those two counts fix the uncompressed size before anything is inflated. That gives the decoder an exact length check and a limit (D22).
  - The payload is packed little endian in either CDR byte order. It is an opaque byte array, the Spark (aarch64) and the Mac are both little endian, and Python packs it with `struct.pack('<…')`.
  - Supported values:
    - `NONE` (0).
    - `ZLIB` (1), the default for every `MapBricks`. It is raw DEFLATE, chosen because it is what Foundation's `.zlib` writes and Python reads with `zlib.decompress(payload, -15)`. The zlib-wrapped format would need a hand-written header and Adler-32 on the Mac.
    - `LZFSE` (2), Mac-native. The Spark has no stdlib encoder for it.
  - zstd is dropped, because Foundation has none, and its old value 1 is reused.
  - Decoding uses the Compression framework's stream API, so a stream that ends early, runs long, or never ends is caught exactly.
  - Apple's inflater is laxer than zlib in two ways (§3.1). The Mac checks the size, not the stream's framing.
  - Goldens: DEFLATE output is not canonical. So each compressed golden carries a hand-built stored block or LZFSE uncompressed block, and the tests check:
    - the encoder's bytes up to the payload;
    - that its own stream inflates to the hand payload;
    - a round trip (§9).
- **D10. Floats.** NaN = missing for `evidence`, `alpha`, `coverage_known_frac`, `pose_residual_m` and `window_s`; ±Inf is invalid. Ranges are as in §3. `from_centroid` is NaN × 3 exactly when the event is not MOVED.
- **D11. `per_sensor_hz`.** It means sensor frames received per second, rounded to whole Hz. Index 0 is Mid-360 and index 1 is JT128, matching the `sensors` bit order. `65535` = not measured.
- **D12. `sensors`.** Bit 0 is Mid-360 and bit 1 is JT128. Other bits are invalid. 0 means no attribution.
- **D13. Epoch counts are cumulative** since the room was created, so a lost tick loses nothing. The Mac diffs consecutive ticks for the timeline.
- **D14. `mask` semantics.**
  - It is a snapshot-kind `MapBricks` with `base_epoch` 0.
  - `occupied` means the voxel is in the region, and `known` means the voxel had evidence. Region voxels must be known, since a change needs evidence in both layers.
  - Its header copies the parent's `room_id`, `map_epoch` and `stamp`, with `seq` 0.
- **D15. Times.** `{0, 0}` renders "—". `nanosec` ≥ 10⁹ is invalid. `last_seen` must not be before `first_seen`.
- **D16. Validation is symmetric.** The Swift encoder refuses exactly what the decoder rejects, so the Mac can never emit a message it would drop.
- **D17. Lifecycle details §2.2 leaves open.**
  - `kind` is fixed per `event_id`.
  - Moved pairing supersedes candidates (they go EXPIRED).
  - RIG_MOVED expires open candidates (D43: those of the moved sensors).
  - DEGRADED keeps tracking with the sensors left.
  - ACCEPTED bumps `map_epoch`.
  - Accept semantics also depend on the brief's §5 Q6.
- **D18. Topic names and QoS** (§2) are proposals for PR 2.4. They include the request topic and the two sync topics of D19, the operator topic of D25, the fallback-header topic of D27, and the activity topic of D33. Durability is D24.
- **D19. `RequestSync`.** Trevor decided on 2026-10-14 that the Mac sends `RequestSync{room_id, since_epoch, since_seq, stamp}`, which the Spark answers with a snapshot or the missed deltas and events. §3.5 defines the message and §5 the answer rule. These parts go beyond his field list and need his confirmation:
  - **Common fields.** `map_epoch` and `seq` were added because §1.4 puts the four common fields first in every message. `map_epoch` repeats `since_epoch`, and both directions validate that. `seq` counts the client's requests.
  - **`topic`.** `seq` is per topic, so one `since_seq` cannot describe both `map_bricks` and `events`. The Mac sends one request per stale topic.
  - **`SINCE_NONE` = 2⁶⁴ − 1** means "holds nothing". It is not 0, because epoch 0 and seq 0 are real values (§1.5).
  - **Answers go out on separate sync topics** (`/room_memory/sync/map_bricks`, `/room_memory/sync/events`, and D36's `/room_memory/sync/end` for the manifests, which replaced D27's `sync/events_fallback`), not on the live topics. Replays keep their original `seq`.
    - On a live topic, other Macs would see `seq` go backwards, and the live topics would stop being in publish order.
    - With separate topics the live rules stay strict, and a Mac reads a sync topic only while its own request is outstanding.
  - **The answer is bounded.**
    - Maps: at most 9 replayed deltas, else one snapshot.
    - Events: replay from the last 100 messages, else the bounded fallback of §5, with its header (D27). That fallback leaves the history marked incomplete instead of reconstructing it.
  - **`RequestSync` adds to the Spark's own snapshots; it does not replace them.** The Spark still sends a snapshot on the live topic when TRACKING starts and then every `keyframe_period_s` (D29), so a Mac gets a map within that period without sending anything.
  - **Checked (issue #21, D39):** the deployed bridge (3.6.0) has no `clientPublish`, and its `client_topic_whitelist` is not enforced, so client publishing stays off. `RequestSync` goes as a whitelisted service call instead. The Spark cannot notice a (re)connect through the bridge (D29). If client publish is unavailable, the Mac cannot request. A stale map then waits for the Spark's next own snapshot, and an event gap stays open.
- **D20. Staleness.**
  - `status` is stale after 3 s and `epoch` after 15 s.
  - A missed trailing delta is caught by `MemoryStatus.map_epoch` disagreeing with the mirror twice in a row.
- **D21. `package.xml` license** says `Proprietary` because the repo has no LICENSE file. Trevor should confirm.
- **D22. Payload limit.** The uncompressed `MapBricks` payload, `134 × n_bricks + 4 × n_removed`, is at most 16 MiB (16,777,216 bytes), about 125,000 bricks or 8,000 m³ fully covered. Without a limit, a 2 MB zlib payload could declare and inflate to over 2 GB. The decoder rejects a larger declared size before inflating, and the encoder refuses to write one (D16). The measured room needs 547 KB.
- **D23. One `seq` rule, with generations** (PR #13; Phase 3's P13). **Superseded by D34 and D37:** its restart cases 1–3 and the open residual below are gone; `seq` is ordered per (generation, topic). Kept as design history.
  - **The conflict it removes.** §5 said both "a snapshot is always accepted" and "a sync message with `seq` ≤ the last applied is ignored". After a node restart, `seq` starts again at 0 and the two collide. A Mac holding the old, higher `seq` could ignore every answer and stay STALE.
  - **Why not "accept any snapshot".** The first Phase 3 draft applied a snapshot answer whatever its `seq`. On the shared sync topic, that lets an older snapshot meant for another Mac rewind a Mac that is current. So "`seq` ≤ baseline is ignored" now holds everywhere. The only exception is a snapshot at exactly the baseline, which is the "nothing missed" answer. A restart is handled as a new generation instead.
  - **How a restart is recognized.** The live `seq` of `map_bricks` or `events` goes down (this relies on those topics staying volatile, D24). Or `status` comes in low twice in a row, counting up by 1. That works in a static room, and repeated transient_local copies cannot match it, because they repeat one `seq`. Or a signal from outside the protocol.
  - **Old messages in flight.** Clearing the baselines alone lets a message the old node published, still in flight, set a new baseline above the new node's `seq`s (Codex, PR #13). Two fences were tried and dropped. One by `stamp` fails on `map_bricks` and `events`, because replays re-send messages unchanged, `stamp` included: a new-generation event published before the restart was recognized was fenced forever, and a fallback naming it never completed. One by `map_epoch` needs an epoch known to belong to the old generation, and the messages that reveal a restart already carry the new one (Codex, PR #13). §5 instead orders snapshots by epoch first, which the node raises at every restart, and lets an old message in flight cause one more, spurious, new generation.
  - **Open: a generation id.** The residual (§5) is an old `events` message on a sync topic after the reset, whose remembered `seq` can hide the new generation's message with the same `seq`. Only a generation id in every message closes it, which changes every message's bytes. That is left to Trevor.
  - **Behaviour change on reconnect.** The Mac reuses a baseline after a disconnect only when an outside signal confirms the node did not restart. Before, it always sent its saved `since`. A restarted node could then answer with a replay of new-generation messages, as if they continued the old history. Without such a signal every reconnect now sends `SINCE_NONE`. That costs one map snapshot, as a reconnect with nothing missed already did, and the bounded events fallback, with the history marked incomplete.
  - **The events baseline** is the `seq` up to which every message has been applied, not the last one applied. A replay can then fill a gap below messages applied after it. `since` names the message at the baseline.
  - **Per-event order.** A message that fills such a gap can be older than the state its event already shows: a lost CONFIRMED at 11 arrives after ACCEPTED at 12. Applying it would be CONFIRMED after ACCEPTED, which §4.2 rejects. So the Mac keeps the `seq` of the last message applied to each event's state. An older message goes to history and advances the baseline, without changing the state. A new generation clears these `seq`s with the baselines.
- **D24. Durability** (PR #13; Phase 3's P2). It needs Trevor's and Codex's review.
  - `status`, `activity` and `epoch` are transient_local with depth 1, so a Mac shows the memory state as soon as it subscribes. Without it, the Mac would wait up to 1 s for `status` and `activity` and 5 s for `epoch`.
  - Every other topic is volatile, and says so explicitly (§2).
  - **The nodes now match it** (protocol-v2 follow-up; PR #14 had done `activity` only). The code was changed, not this decision, because nothing argued for volatile `status` any more: a stored copy is safe now that `status` carries the publisher generation (D34), the mirror already treats a `seq` not above the held one as a transient_local copy (§5.3), and `activity` had used the same QoS through the same bridge since PR #14. A Mac that subscribes gets the memory state at once instead of up to 1 s later.
  - `epoch` is the exception: no node publishes it yet (§2), so it has no durability to get wrong. Its row in §2 stands for when PR 2.4 adds it.
- **D25. `OperatorAction`** (PR #13; Phase 3's P6). It needs Trevor's and Codex's review. §3.6 defines it, with `ros/labcommand_msgs/msg/OperatorAction.msg`.
  - The actions asked for (acknowledge, dismiss, re-baseline, label) and Phase 3's earlier list map onto the existing transitions. Acknowledge and label are log-only. Dismiss is §4.2's "operator revert". Re-baseline is §4.1's "operator re-references the room". Pause and resume are §4.1's operator pause and resume. Accept is §4.2's "operator accept". Mark ground truth is for PR 3.2's scoring.
  - The common fields serve as the header, not `std_msgs/Header` (D2).
  - `operator_id` is self-asserted. Neither the bridge's client-publish path nor Phase 3's ssh relay can tell which person sent an action.
  - **ACCEPTED follows its map message** (§4.2). If ACCEPTED went out first, every Mac would remove the overlay while its map still lacked the change. A delta can wait 5 s for the 0.2 Hz slot (§2), and longer behind Phase 3's byte bucket (`docs/phase3-live-integration.md` §3.3). So the node publishes ACCEPTED right after the message that carries the change. The Mac also removes the overlay only once its mirror's epoch is ≥ the event's `map_epoch`. `expected_state` is compared with the node's own state, so an accept still waiting for its map message cannot also be dismissed.
  - Open: re-referencing outside RIG_MOVED, and showing labels and acknowledgements on other Macs. Both would change §4.1 or `ChangeEvent`.
  - **Out of scope for protocol v2** (§3.6): no codec, golden or handler in any language until Phase 3 S5, which also decides whether an action is fenced to the publisher generation (an `expected_generation`, as D35 gives `RequestSync`).
- **D26. Events high-water mark** (PR #13; Phase 3's P16). **Partly superseded by D36:** `events_seq` stays (scoped to the generation by D34) as a witness of H, but the request "mark" no longer decides completion; the `SyncEnd` manifest does. §5 defines it, and §3.3 adds `uint64 events_seq` to `MemoryStatus`. That changes `MemoryStatus` bytes: 8 more per message.
  - **`events_seq` is the newest events `seq`.** Before this, a replay was complete once it reached the highest live `seq` the Mac had seen. A Mac that missed the last events before a quiet period has seen none of them on the live topic, so its request was done at once and the history stayed wrongly complete. The map already had its high-water mark in `map_epoch`.
  - **Not an in-band end marker.** An earlier draft ended every answer with its last message sent again. Sync topics are shared, and the repeat was an ordinary `ChangeEvent`. So after a lost marker, another answer starting at the same `seq` read as the marker, and the request closed before the missed events arrived (Codex, PR #13). `events_seq` is not part of any answer. A lost message only delays completion.
  - **The mark decides when a request is done, not whether a gap is given up.** That takes the fallback header (D27).
- **D27. Events fallback header** (PR #13, after Codex's review of it; Phase 3's P17). **Superseded by D36:** `EventsFallback`, its `.msg` and `/room_memory/sync/events_fallback` are deleted; no language had a codec for it and no node published it. Kept as design history: it defined `EventsFallback` and its topic, and the rule that only the header could give up a gap.
  - **Why an explicit header.** The previous draft recognized a fallback by its first message not being the `since` message, which a replay re-sent first. On the shared sync topic, another Mac's replay can arrive first, and the `since` message can be lost. Either read as a fallback, and the Mac gave up messages that its own replay was still bringing (Codex, PR #13). The `since` message is no longer re-sent.
  - **Why it names the `seq`s.** A fallback's contents are sparse: open events older than the kept messages, then the kept range. Reaching the mark on the live topic does not show they have arrived. The previous draft closed a fallback while its messages were still in flight, and ignored them afterwards (Codex, PR #13). With the list, the Mac gives up only `seq`s the Spark no longer holds, and only after receiving every one it does.
  - **Why it also waits for `kept_last_seq` on the sync topic.** Headers travel on their own topic, so another Mac's header can arrive while the Mac's own earlier answer is still in flight with a `seq` that header omits. The sync topic keeps publish order, so its copy of `kept_last_seq` arrives after that earlier answer (§3.8).
  - **Why keyed by `since`, not by client.** `RequestSync` carries no client identity, and adding one would change its bytes. The header states what the Spark holds, not who asked, so a header for the same `since` is valid for every Mac that sent it.
  - **Why not in band.** `ChangeEvent`s are re-sent unchanged, and an ordinary `ChangeEvent` cannot mark an answer: D26's lost-marker case.
  - **Why not the reply to the `:8766` POST** (Phase 3's P11). The protocol stays independent of the request transport, and bridge client publishing has no reply. A fallback can also be deferred past the request (Phase 3's P4).
  - Replays and map answers need no header. A replay gives up nothing, and a snapshot says what it is (`KIND_SNAPSHOT`).
- **D28. A dedicated, read-only bridge on :8767** (§2; a proposal: the transport is open, Q5). The memory topics need `labcommand_msgs` in the bridge's environment, which the lidar bridge on :8765 does not have, and a separate bridge leaves LIVE untouched. Read-only by default because foxglove_bridge 3.4.1 does not enforce `client_topic_whitelist`. `RequestSync` then needs `foxglove_client_publish:=true` or a relay. *Amended by D39:* the bridge also offers `services`, whitelisted to the request service, and stays unable to publish.
- **D29. Periodic keyframes.** §2.4's "snapshot on (re)connect" cannot be detected through the bridge, so the live topic carries a snapshot every `keyframe_period_s` (60 s). Changed bricks between keyframes go out as deltas at most every `delta_period_s` (5 s, the ≤ 0.2 Hz of §2). A snapshot replaces the next delta once the deltas since the last snapshot would exceed `keyframe_delta_ratio` (0.5) of its size. A keyframe of an unchanged map does not bump `map_epoch`; every delta does.
- **D30. One published map.** There is no JT128↔Mid-360 extrinsic yet (brief R1), so the references stay per sensor and `map_bricks` carries one of them, `map_sensor` (default: the JT128 when configured; its repeating pattern gives the denser map). `n_bricks` and `coverage_known_frac` describe that map. Events of the other sensor are in that sensor's own frame.
- **D31. A persistence gate before CANDIDATE (strengthened by D33).** On the static recording the node sent ~120 CANDIDATE/EXPIRED messages per minute. Most of them came from a reference built as the union of 5 windows of the Mid-360's sparse non-repetitive pattern; per-voxel window counts fix that (`spark/room_memory_cpp/PLAN.md`). As a second line, a region now has to persist `candidate_windows` windows before its CANDIDATE is sent. An open candidate expires only after `expire_windows` unmatched windows, so an intermittently seen change does not flicker. T_commit (CANDIDATE → CONFIRMED) is unchanged and still PR 2.3.
- **D32. No events in the near field (superseded by D33: removed).** The rig stands about 1.3 m from where someone works. Live and in both static recordings, something low there (z up to ~0.8 m in the sensor frames) shifts every few seconds to minutes, seen by both lidars at once, and it was 100 % of the remaining events (live 2–18 per minute). A change region with a voxel center within `near_range_m` (2.0 m) of its sensor is not reported, whichever pose of a move it is. With it, the diff's `tolerance` 2 and `min_voxels` 8 remove the far fragments of the same motion. This is a blind zone, not noise removal: a real change within 2 m of either lidar is not reported. 0.3 m cubes from 2.2 m out are still detected at the same latency (`spark/room_memory_cpp/PLAN.md`, "False change events"). *Removed:* the "something low" was Trevor working at a desk (he confirmed it), real transient motion, and a blind zone also hid real changes next to the rig. `near_range_m` is gone; `tolerance` and `min_voxels` are back at 1 and 6. D33 replaces it.
- **D33. Activity is not change.** Motion next to the rig (a person working) is reported on `/room_memory/activity` (§3.7), never as a `ChangeEvent`. A region becomes a CANDIDATE only after it has stayed settled for `candidate_windows` windows (§4.2): fresh, the same shape, and no activity within `activity_radius_m`; so a change made by a person is reported that long after they stop moving next to it. `candidate_windows` is 40 (40 s at `window_s` 1): on both static recordings, looped for 30 minutes, the longest settled run of anything Trevor left is 24 windows (the part of his BUILDING-time pose he moved away from). 0.3 m boxes 1.0–4.6 m from either lidar are CANDIDATEs 40–43 s after they appear, disappear or move. Tracking teaches the reference nothing (`learn_windows` 0): learning while he worked wrote his surroundings into the reference and produced a 45-window still residue. Limits: a person who stays perfectly still for longer than the gate after shifting is reported, and so is a reference built while someone sat in it once they leave; without the JT128↔Mid-360 extrinsic (D30) one sensor's activity cannot hold the other's regions. The Mac decodes `Activity` into its mirror (§5.3, D38); the view does not show it yet.

- **D34. Publisher generations** (`pubgen`, PLAN-PUBGEN §2–§3). Every stateful Spark → Mac message ends with `uint64 publisher_generation`, durable and increasing per room (`max(stored + 1, now_ms)`, written and fsynced before the first publish, under a per-room `flock`, fail closed), the same for every message of one process. `seq` is ordered per (generation, topic). The v2 fields are appended after each message's last legacy field, so every legacy offset and golden prefix is unchanged, and the layout is detected from the trailing bytes (§1.1): the Mac needs no schema text, and `FoxgloveProtocol.Channel` carries none. Generation 0 is reserved (`GENERATION_UNKNOWN`). Saved references stay legacy. Rollout: the Mac app first (it reads both layouts and runs an old node in legacy mode), then the room-memory container; rolling back is safe both ways (PLAN-PUBGEN §8). D38 extends the field to `Activity` and `Epoch`.
- **D35. `RequestSync` token and expected generation.** A v2 request appends `expected_generation` (the generation its `since` belongs to) and a random 16-byte token, new for every send. The node honours `since` only in its own generation, and copies the token into the `SyncEnd`. A legacy request is answered as generation 0 with an all-zero token. A legacy node rejects a v2 request, so the Mac sends none on a legacy wire.
- **D36. `SyncEnd`, the end-of-answer manifest** (supersedes D27 and D26's request mark). One per answered request, on `/room_memory/sync/end`, after the answer: the generation, the token, the topic's bound, and the `seq`s delivered and (events fallback) unavailable (§3.8). It states what was sent, never what a Mac received. Every Mac may record its bound and unavailable `seq`s; only the requester's token moves a retrieval, and only receipt of what it names completes it.
- **D37. The generation-scoped mirror** (supersedes D23's restart cases and its residual). One active generation state, installed on the first validated message of a higher generation (a deviation from Astra's table, which quarantined messages until the generation's status: installing early loses nothing, PLAN-PUBGEN §4.5), retrieval by token and manifest, and the `EventHistory` projection (coverage, freshness, continuity, caughtUp). Legacy wire: shown, unverified, never current, no requests. PLAN-PUBGEN §4.4 lists the invariants; the Swift runner checks them after every input of 2,000 seeded traces against an independent reference model, with the ten Codex findings and Astra's pair as regression seeds (PLAN-PUBGEN §6).
- **D38. `Activity` and `Epoch` carry the publisher generation** (the PR #18 follow-up to D34). Both are transient_local, and their `seq` restarts at 0 with each process. So `seq` alone cannot tell a restarted node's message from a stored copy: a Mac holding `Activity` seq 50 of the old run would drop the new run's 0, 1, … as copies until they passed 50. Like every Spark → Mac message, they now end with `uint64 publisher_generation` (padding to 8, then 8 bytes; layout from the trailing bytes, §1.1), and the Mac scopes them to the generation (§5.3). They never fix a connection's layout: a PR #18 node publishes v2 status, map and events next to legacy `Activity`, whose stored transient_local copy usually arrives first, and letting it decide would run the Mac in legacy mode against a v2 node. Rollout as D34, the Mac first: against a PR #18 node it drops the legacy `Activity` and counts it, which costs nothing it showed before. `Epoch` gets the field before any node publishes it, so it is never deployed without it.
- **D39. `RequestSync` is a service call through the :8767 bridge** (issue #21). Live on c5518eb the bridge offered `connectionGraph` only, so the Mac could not send `RequestSync` and an event gap stayed open ("history incomplete · no request path"). Client publishing cannot be turned on safely: foxglove_bridge does not enforce `client_topic_whitelist` (3.4.1 through 3.6.2), so a client could publish on any topic. The bridge does enforce `service_whitelist` (it advertises, and lets a client call, only matching services). So both nodes offer `/room_memory/sync/request`, type `labcommand_msgs/srv/RequestSync` (request: one `RequestSync request`; response: `bool accepted`, false when it did not decode or names another room), and the launch file gives the bridge `services` with `service_whitelist` `^/room_memory/sync/request$`. The whitelist must be set: its default is `.*`, which would expose every node's parameter services. The request is one nested `RequestSync`, so its CDR has the message's layout, and the Mac sends its `RequestSync` bytes as the call payload (encoding `cdr`, ws-protocol binary op 0x02). The answer still goes out on the sync topics with its `SyncEnd`: the response is an acknowledgement, not the answer, so D36 is unchanged. The `request_sync` topic stays for a relay. Rollout: the Mac first (against an older node it finds no service and reports "no request path" as before), then the room-memory image. This answers Q5 for `RequestSync` on the :8767 option; `OperatorAction` and the :8765 option stay open.
- **D40. Re-alignment keeps the room frame** (`rig-guard`, PLAN.md). Each sensor has a pose, identity when its reference is built; incoming points are transformed by it and rays start at its origin. Re-alignment changes the pose only: the reference, `map_epoch` and the map stay, so it sends no map. RIG_MOVED is entered only on move evidence; a check that cannot align without evidence is INCONCLUSIVE in `RigPose` and leaves `MemoryStatus` unchanged. `pose_residual_m` is measured on pose-corrected points.
- **D41. Automatic relearn.** RIG_MOVED → BUILDING after `relearn_after_s` without an applicable alignment, with `auto_relearn` (default on), only from RIG_MOVED. The old state is archived under `prev/`, not deleted. The new reference starts a new room frame (`frame_epoch`), and the first map message after it is a snapshot.
- **D42. `RigPose` on `/room_memory/rig`** (§3.9). Reliable, transient_local, keep-last 1, like `Activity`, so a late Mac gets the current pose at once; published on every outcome or decision change and every 10 s. It is a new message on a new topic because the old ones cannot carry it: D3 makes a Mac reject an unknown enum value (a `ChangeEvent` `KIND_RIG_MOVED` would drop every such message on deployed Macs), and §1.1 allows no new trailing fields beyond the v2 ones. A Mac that does not subscribe ignores it. The :8767 bridge's `^/room_memory/.*` whitelist already covers the topic.
- **D43. State precedence, two transitions, per-sensor expiry.** RIG_MOVED outranks DEGRADED: a silent sensor does not hide a move. §4.1 gains DEGRADED → RIG_MOVED and RIG_MOVED → DEGRADED; a Mac must accept both. RIG_MOVED expires the open CANDIDATEs of each sensor the guard finds moved; a sensor found in place keeps its candidates (amends D17).
- **D44. CHECKING and INCONCLUSIVE are not memory states.** They exist only as `RigPose.outcome`. While a check runs (at most `check_max_s`, 10 s) CANDIDATE promotion and learning are held, and `MemoryStatus.state` stays TRACKING.

## 9. Verification status

- **Hand goldens.** `Tests/Fixtures/room/golden/hand/*.hex` were written byte by byte from these rules, with every span annotated. There are 27, the same files in `spark/room_memory/tests/fixtures/hand`: the 14 legacy ones (11 little endian and 3 big endian, covering §3.1–§3.5 and two `Activity`s, active and idle with NaN geometry), and 13 protocol v2 ones (9 little endian, 4 big endian: `MemoryStatus`, `MapBricks` LE/BE, `ChangeEvent` with and without a mask, `RequestSync` LE/BE, three `SyncEnd`s, `Epoch` LE/BE and `Activity`), each v2 one repeating its legacy twin's values with the new fields appended and one publisher generation throughout. `OperatorAction` has none: it is out of scope for protocol v2 (§3.6). `scripts/check-shared-goldens.sh` requires every shared file to be identical in both fixture directories. Swift, Python and C++ decode all 27. The tests check:
  - The Swift decoder reads the 14 legacy goldens to the expected values. The big-endian `MapBricks` golden has a little-endian payload inside a big-endian message.
  - The encoder reproduces the 9 uncompressed little-endian legacy goldens exactly, and every little-endian v2 one.
  - For the zlib and LZFSE goldens, which hold a hand-built stored block and an uncompressed LZFSE block:
    - the encoder matches the bytes up to the payload length;
    - its own compressed stream, inflated through Foundation rather than the codec, equals the hand payload;
    - that stream decodes back to the value.
  - Every truncation throws, except that a v2 golden cut at its legacy length is that legacy message (with no generation), never the golden.
- **Compression interop, checked once by hand.** The test runner has no Python, so these checks are not in the suite.
  - Python 3.9 `zlib.decompress(payload, -15)` inflates Apple's `.zlib` output of a 547 KB payload. Plain `zlib.decompress(payload)` rejects it.
  - Apple inflates Python's `zlib.compressobj(6, zlib.DEFLATED, -15)` output of the same payload.
  - The packing helpers in `scripts/spark/make_room_memory_goldens.py` reproduce the payload bytes of all 5 hand goldens that have one.
- **Independent CDR cross-check: `RigPose` only.** `spark/room_memory/tests/test_rosbags.py` checks `RigPose` against `rosbags` (§3.9). The other hand goldens have only been checked against the Swift codec, which follows the same rules.
- **Spark goldens: not produced.** `scripts/spark/make_room_memory_goldens.py` serializes the 9 little-endian v2 values with `rclpy` after a `colcon build` of `labcommand_msgs` in the Jazzy container. Running it is a Spark task. The follow-up test then requires each Spark file to decode to the same value as its hand golden, which confirms D4.
- **`colcon build` of `labcommand_msgs`: not run.** There is no ROS 2 on the Mac.
- **`OperatorAction` (§3.6, D25): out of scope for protocol v2, the `.msg` only.** There is no codec in Swift, Python or C++, no hand golden and no node handler, by decision (§3.6): they are Phase 3 stage S5, together with the generation question. The 117 B wire size was computed by hand from §1.1, not encoded.
- **Protocol v2 (D34–D38): implemented** in the Python oracle, the C++ node and the Swift codec and mirror. The Python, C++ and Swift codecs decode every v2 golden and reproduce the little-endian ones. The differential tests compare Python and C++ byte for byte, `SyncEnd`s included, on every §5.2 answer case, a session across two generations, and the legacy `RequestSync` answer. The ROS node wiring (`node.py`, `room_memory_node.cpp`) needs ROS 2 and was not run on the Mac.
- **`RigPose` (§3.9, D40–D44): implemented** in the Python oracle and the C++ node, with the hand goldens `rig_pose_le.hex`, `rig_pose_v2_le.hex` and `imu_le.hex` (`sensor_msgs/Imu`, decoded for the guard) in `spark/room_memory/tests/fixtures/hand`, checked by the Python and C++ codecs; the Swift suite does not decode them. `test_rosbags.py` builds the type from `RigPose.msg` (the legacy layout without its tail) in `rosbags`. Both `RigPose` goldens decode there to their values and re-serialize byte for byte. The Python codec and `rosbags` encode the same 48 values (room ids of every length mod 8, both layouts, missing values) to the same bytes, and a rosbag2 written and read back by `rosbags` carries both goldens unchanged. It skips without `rosbags`: the gate's venv has none, so it was run by hand, 52 passed with each of `rosbags` 0.9.23 (the Mac's system Python 3.9.6) and 0.11.7 (Python 3.13), which needs the rosbag2 version passed to its `Writer`. The differential tests compare the engines with the guard on: statuses, events and maps byte for byte, `RigPose` outcomes exact and its floats within PLAN.md's v1 relaxation. Off by default, so without `rig_guard` the outputs are unchanged.
- **The Spark side of §5 is implemented** in `spark/room_memory_cpp` (the node) and in the Python oracle `spark/room_memory`. Both nodes declare D24's durability on every topic; the rclpy test for it (`test_node.py`) and the C++ node need ROS 2 and were not run on the Mac. Not implemented: the `epoch` topic (§2). D39 (issue #21) was checked in a local build of the image (colcon build and test, then `test_node.py` in the container) and end to end against its foxglove_bridge 3.6.0: the bridge advertised only `/room_memory/sync/request` of the container's 16 services, and `LiveRoomClient` called it and received the node's `SyncEnd`. The deployed Spark runs the image from before D39 until it is rebuilt, so the Mac there still has no request path (§5.3).

## 10. Open questions for Trevor (brief §5, Q1–Q4, and Q5)

Q1–Q4 are copied from the brief and left unanswered. Q5 was added by PR #14.

1. **Is the rig static for v1?** If it will move between sessions, relocalization (and place recognition) moves from optional to required.
2. **Room and targets:** room size, smallest object that must be detected (5 cm? 10 cm?), acceptable latency (seconds vs minutes), acceptable false events per hour. These set voxel size, T_commit and the PR 3.2 bar.
3. **HDC's role.** Your own record (exp119/122/123, exp113, D-33) says a superposed memory is not a cheaper map, and HDC is parked. Should the app be HDC-free with HDC kept only as an optional RT3 comparison, or do you want to reopen HDC? This brief assumes the former.
4. **Fixtures:** may the 12 Oct box takes (`*ROBOT_box_*`, `*speedybee_box_*` on the Spark) be copied to the Mac as golden fixtures? This brief did not access the Spark.
5. **Transport (D28).** A dedicated foxglove_bridge on :8767 in the room-memory container, or the existing bridge on :8765 with requests relayed over the ssh-authenticated command-center API on :8766? Both stay in this doc until you choose (§2). For `RequestSync` on :8767, D39 (issue #21) chose a whitelisted service call, not client publishing; a relay still works through `requestSyncSink` if you prefer :8766.
