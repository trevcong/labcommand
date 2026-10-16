# PLAN-PUBGEN: publisher generations and a generation-scoped room-memory mirror

Status: plan only (step 1). Trevor approved Astra's root-cause redesign (`lc-tasks/liveroom/astra-mirror.md`). This plan turns it into a wire contract, a state machine, and a file-by-file change list. Nothing below is implemented yet.

**The problem.** The Live Room mirror (`RoomMemoryMirror.swift`, PR #17) infers publisher restarts from counters. It also infers history completeness from request flags and a 2 s quiet period. Ten Codex rounds (`code-review2..10.md`) each closed one trace and opened the next. Astra's diagnosis: identity, received history and retrieval progress compete as sources of truth. With today's wire, no client can tell "the publisher continued while I was away" from "it restarted and counted past my old counters". The fix needs a protocol change:

- an ordered, durable publisher generation on every stateful message;
- request tokens;
- an explicit end-of-answer manifest.

On the Mac, one reducer scopes all state to a generation.

## 0. Where this branch stands

| | |
|---|---|
| Worktree | `~/Developer/labcommand-pubgen`, branch `pubgen` |
| Base | `origin/phase3-design` 5b7837e: PR #13 (Phase 3 design doc) rebased onto PR #14 (`room-memory-cpp` 5291df5) |
| Merged | `live-room-ui` fb9423f (PR #17, based on `main`). Merge commit b096554, clean, no conflicts |
| Baseline gate on b096554 | `build.sh --no-install` exit 0. `LabCommandCoreTestsRunner` PASS (233 groups). pytest 300 passed, 105 skipped. C++ build 0 warnings, ctest 15/15. `memory_status_le.hex` Swift == Python |

**Upstream warning.** `pubgen` currently tracks `origin/phase3-design`. A bare `git push` would push this work onto PR #13's branch. Before any push (Trevor's call, not part of this task), push to a new remote branch with `git push -u origin pubgen`.

## 1. Branch and merge strategy

**Recommendation:**
1. **Land #14 first**, as is. It is the Spark node, it has had seven Codex rounds of its own, and it speaks today's wire consistently. Nothing here needs it to change before it merges.
2. **One PR from `pubgen` to `main` then supersedes #13 and #17.**
   - #13's protocol rules D23 (restart cases 1–3), D26's request mark and D27 (`EventsFallback`) are exactly what this work replaces.
   - #17's `RoomMemoryMirror` restart and completeness logic is what this work deletes.
   - Reviewing either separately would spend review rounds on rules that will not ship.
   - Both PRs' commits are in `pubgen`'s history (the #13 commits as ancestors, #17 through merge b096554), so closing them loses nothing. Close each with a pointer to the new PR.
3. **After #14 merges,** merge `main` into `pubgen` rather than rebasing, which keeps b096554 meaningful. Then open the PR.
4. **If Trevor prefers to keep #13 separate:** land #13 after #14, merge `main` into `pubgen`, and the `pubgen` PR then supersedes only #17. The protocol edits below then show up as a diff against #13's text.

Nothing is pushed by this task.

## 2. Wire contract (protocol v2)

### 2.1 Principles

- **New fields go at the end of each existing message**, the way `events_seq` was added. Existing byte offsets, golden prefixes, and the hard-coded byte patches in `RoomMemoryCDRTests.swift` (:360–452, `PayloadFraming` :194/196) stay valid.
- **Version detection comes from the trailing bytes, not the advertised schema.** After the last legacy field, a decoder sees one of three cases:
  - ≤ 3 unread bytes: rmw padding, so the message is **legacy**.
  - Alignment padding to 8, then exactly 8 bytes (+ ≤ 3 rmw padding): **v2**.
  - Anything else: *invalid*.
  - Alignment padding is at most 7 bytes and the field is 8, so the cases cannot overlap.
  - This holds for every message below, including `MapBricks` after its variable-length payload. The client needs no schema parsing: `FoxgloveProtocol.Channel` carries no schema text today.
- **`publisher_generation` (`uint64`).**
  - It is ordered and durably increasing per room (§3), and the same for every message one node process publishes.
  - `0` is reserved: `GENERATION_UNKNOWN`, used in `RequestSync` only. It is invalid in every Spark → Mac message.
  - `map_epoch` stays a map revision. Generation and epoch are independent.
- **Replays carry the original generation.** The nodes resend the stored CDR bytes of the original publish (`Logged.bytes`). The logs live in memory for one process, so a replayed message always carries the generation of the process that answers. A generation is fixed for the life of the process.
- **Naming.** "Publisher generation" is not Phase 3's "`room_id` generation" (P9, `lab` → `lab~2`). Docs say "publisher generation" in full.

### 2.2 Byte layouts

Positions count from the byte after the 4-byte encapsulation header. They are shown for `room_id` "lab", so the common fields end at 32. "C" is the common-field size `align8(4 + L) + 24` (§3.1 of the protocol).

**`MemoryStatus`** (+8 B):

| pos ("lab") | field | note |
|---|---|---|
| 0–71 | unchanged, `state` … `events_seq` | `events_seq` @64 is the last legacy field; it ends 8-aligned |
| 72 | `uint64 publisher_generation` | no padding |
| end | 80 → **84 B** on the wire (legacy 76 B) | "lab-a" golden: generation @80, 92 B |

`events_seq` keeps its meaning, now scoped by the generation: the seq of the newest live `events` message published in this generation, or `EVENTS_SEQ_NONE` if there is none. A v2 status with `EVENTS_SEQ_NONE` is the publisher's witness that this generation's event history is empty.

**`MapBricks`** (+8 B plus 0–7 B padding):

| pos ("lab") | field |
|---|---|
| 0–67 | unchanged, through the `payload` count @64 |
| 68 … 68+P | `payload` bytes (P bytes) |
| align8(68+P) | `uint64 publisher_generation` |
| end | 4 + align8(68+P) + 8 B. Legacy: 72 + P B |

**`ChangeEvent`**, without a mask:

| pos ("lab") | field |
|---|---|
| 0–131 | unchanged; `mask` count @128 = 0 |
| 132–135 | pad |
| 136 | `uint64 publisher_generation` |
| end | **148 B** (legacy 136 B) |

**`ChangeEvent`, with a mask.**
- The nested `MapBricks` is the v2 `MapBricks`: its own `publisher_generation` follows its payload, padded to 8. The parent's `publisher_generation` comes right after it, already aligned.
- Rule (extends D14): the mask copies the parent's `room_id`, `map_epoch`, `stamp` **and `publisher_generation`**, with `seq` 0. The existing mask-header validation (Swift `RoomMemoryCDR.swift` :297–318) also requires the generations to be equal.
- The two layouts cannot be mixed. A legacy parent has a legacy mask, and a v2 parent has a v2 mask. Anything else is *invalid*.

**`RequestSync`** (Mac → Spark):

| pos ("lab") | field | note |
|---|---|---|
| 0–48 | unchanged, `topic` @48 | legacy end 49 → 53 B |
| 49–55 | pad | |
| 56 | `uint64 expected_generation` | the generation the Mac's `since` belongs to; `GENERATION_UNKNOWN` (0) = none |
| 64 | `uint8[16] request_token` | random 128-bit token (UUID v4 bytes), new for every send, retries included; all-zero is invalid in v2 |
| end | 80 → **84 B** | |

- A v2 node receiving a **legacy** `RequestSync` (from an old app) treats it as `expected_generation = 0` with an all-zero token. It answers as for any mismatch (below), and its `SyncEnd` carries the zero token.
- A legacy node receiving a v2 request rejects it (≥ 4 trailing bytes). That is why the Mac sends no request in legacy mode (§4.6).

**`SyncEnd`** (new, Spark → Mac, `/room_memory/sync/end`, reliable, volatile, keep-last 10). New `SyncEnd.msg`:

```
string room_id
uint64 map_epoch                 # node's published epoch when the answer was computed
uint64 seq                       # +1 per SyncEnd, from 0 per generation
builtin_interfaces/Time stamp
uint64 publisher_generation      # the answering generation (= the generation of every message in the answer)
uint8[16] request_token          # copied from the RequestSync; all-zero for a legacy request
uint8 topic                      # RequestSync.TOPIC_* copied
uint64 bound                     # the topic's newest live seq in this generation when the answer was computed;
                                 # SEQ_NONE (2^64-1) = nothing published on it in this generation (known empty)
SeqRange[] delivered             # seqs published in this answer, before this SyncEnd, on the topic's sync topic
SeqRange[] unavailable           # requested seqs the publisher no longer holds (events fallback only)
```

New `SeqRange.msg`: `uint64 first`, `uint64 last`, inclusive, `first ≤ last`.

| pos ("lab") | field |
|---|---|
| 32 | `publisher_generation` |
| 40–55 | `request_token` |
| 56 | `topic` |
| 57–63 | pad |
| 64 | `bound` |
| 72 | `delivered` count (u32); elements from 80, 16 B each, 8-aligned |
| … | `unavailable` count (u32), then elements, 8-aligned |

Wire sizes for "lab", with d delivered and u unavailable ranges:

| d | u | size |
|---|---|---|
| 0 | 0 | 84 B |
| ≥ 1 | 0 | 88 + 16d B |
| 0 | ≥ 1 | 84 + 16u B |
| ≥ 1 | ≥ 1 | 92 + 16d + 16u B |

`SyncEnd` validation (symmetric, D16):
- `publisher_generation ≠ 0`; `topic` a `TOPIC_*` value.
- Each list canonical: ranges strictly increasing and non-adjacent (`next.first > prev.last + 1`). The two lists are disjoint.
- Every range ≤ `bound`. `bound = SEQ_NONE` ⇒ both lists empty.
- `topic = TOPIC_MAP_BRICKS` ⇒ `unavailable` empty.

**Unchanged:** `Activity`, `Epoch` (not published), `OperatorAction`. **Deleted:** `EventsFallback.msg`, protocol §3.8, D27 and the `/room_memory/sync/events_fallback` topic. No language has a codec for it and no node publishes it, so `SyncEnd` supersedes it with nothing to migrate.

### 2.3 How the Spark answers `RequestSync` (both nodes, identical bytes)

**Since.** Let `g` be the node's generation.
- `since` is honoured only when `expected_generation == g`.
- Otherwise (0, older, or unknown) the request is answered as `SINCE_NONE` in `g`, and the `SyncEnd` tells the Mac the actual `g`.
- A request for another `room_id`, or one that does not decode, gets no answer and is logged, as today.

**Map** (`bound` = newest live map seq, or `SEQ_NONE` before the first live map message):

| case | sync/map_bricks | SyncEnd |
|---|---|---|
| no map yet (BUILDING) | nothing | `bound` NONE, `delivered` [] (today this case gets no answer and the Mac retries forever) |
| `since` = (epoch, seq) of the newest logged message, `since_seq == bound` | nothing (**new**: today a full snapshot is sent) | `delivered` [] |
| `since` found in the log, older than `bound` | replay of every later logged message, unchanged | `delivered` [[since+1, bound]] |
| otherwise | one fresh snapshot, `seq = bound` (as today) | `delivered` [[bound, bound]] |

**Events** (`bound` = `events_seq`):

| case | sync/events | SyncEnd |
|---|---|---|
| `bound` NONE | nothing | `bound` NONE, both lists [] (known empty) |
| `since` found in the 100-message log | replay of the later messages (possibly none) | `delivered` [[since+1, bound]] or [] |
| otherwise (fallback) | as today: the latest message of each open event older than the log, then the whole log | `delivered` = those seqs as canonical ranges. `unavailable` = R ∖ `delivered`, where R = [since+1, bound], or [0, bound] for `SINCE_NONE`/mismatch |

**Publish order.** The answer's messages go out first, then the `SyncEnd`. The `SyncEnd` travels on its own topic, so it can arrive before them. Receiving it proves nothing about receiving them (§4).

**`unavailable` is permanent within a generation.** The log only moves forward, and an event's latest message only gets newer. A seq that was neither logged nor an open event's latest when the answer was computed can never be served later (the argument of today's §3.8). Every Mac may therefore record `unavailable` from any `SyncEnd` of the generation, whoever asked.

## 3. Durable generation allocation (Python and C++ nodes)

- **File.** `<state_dir>/<room_id>/publisher_generation`, decimal ASCII and a newline, next to `map_epoch`. It is a separate file because `map_epoch` does not bump on a start without saved references, while the generation must bump on every start.
- **Allocation.** Before any publisher is created, the node computes `g = max(stored + 1, now_unix_ms)`:
  - `stored` is 0 when the file is absent.
  - It writes `g` with `write_atomic`, then **fsyncs the directory**, and only then starts publishing.
- **The wall-clock floor.** It keeps generations rising even if the volume is wiped or the file is lost. A restarted counter would otherwise come back low, and Macs that ignore older generations would ignore the new node until relaunched. The residual is a wiped file *and* a clock that went backwards past the old value; the protocol states it.
  - Generations are then about the start time in ms (~1.76e12), well inside `uint64`.
  - `event_id` already uses the start time this way (`memory.py` :829).
- **Fail closed.** The node refuses to start, and logs why, in any of these cases:
  - `state_dir` is empty;
  - the file is malformed (never silently reset to 0);
  - the value would overflow;
  - the write or fsync fails;
  - another live process holds `<state_dir>/<room_id>/publisher.lock` (`flock(LOCK_EX|LOCK_NB)`, held for the process lifetime: one authoritative publisher per room).
- **Directory fsync.** `write_atomic` in both nodes (Python `memory.py` :314–322, C++ `memory.cpp` :328–346) does tmp, fsync, rename, but never fsyncs the directory. Add that for both files, so `map_epoch` gains it too.
- **No in-process counter resets.** The seq counters are assigned only at construction (Python `memory.py` :822–836, C++ `memory.hpp` :395–411). Re-reference and rebuild do not reset them. This plan keeps that: a future in-process reset would need a new generation, and the protocol says so.
- **Injectable.**
  - `RoomMemory` takes `publisher_generation` as a constructor argument (Python and C++). Only the ROS nodes call the allocator.
  - The offline CLI, `tools/replay.cpp`, `differential/replay.py` and the tests pass a fixed value, so Python/C++ byte identity and fixtures stay deterministic.
  - The allocator takes the clock as a parameter for its tests.
- **Reference files stay legacy.** `reference-<sensor>.cdr` is `MapBricks` CDR. The nodes keep writing it in the **legacy layout** and read both. A v2 reference would make a rolled-back image unable to load its references (≥ 4 trailing bytes), and a generation in a file on disk means nothing.

## 4. The Mac: one generation-scoped reducer

`RoomMemoryMirror` keeps its public input methods (`connectionOpened`, `connectionClosed`, `receive(topic:payload:at:)`, `tick`, `noteRequestUnsent`) and its `Action.requestSync` shape. That leaves `LiveRoomClient` and `LiveRoomFixture` nearly untouched. Inside, three components replace every heuristic. Deleted:
- `eventsSeq` as a mutable cursor, `eventsAhead` and `eventsInitialSync`;
- `syncSettle` (2 s quiet) and `settleEventsAnswer`;
- `eventsLow`, `eventsMark`, `cutHistory`, `nodeRestarted`;
- the low-counter and NONE restart inference;
- the `needs[...]?.sentAt` gate on sync topics.

### 4.1 State

```
Mirror
├─ connection: offline | awaitingStatus(connectionID) | observing(connectionID, generation)
├─ wire: undetermined | v2 | legacy                    // per connection (§4.6)
├─ active: GenerationState?                             // the only state that renders as current
├─ archive: [ArchivedGeneration] (newest 2)             // display only, tagged with their generation
├─ retrieval[topic]: idle | waiting(token, generation, sentAt, attempt) | receiving(token, generation, manifest, sentAt, attempt)
└─ tokenRNG: SplitMix64 state (seeded at init; injectable for tests, so the reducer stays pure and Equatable)

GenerationState (generation g)
├─ status: last MemoryStatus of g (by seq), receivedAt, connectionID it arrived on
├─ map: chain = none | held(epoch, appliedSeq, bricks, voxelSize); highestLiveMapSeq; statusEpochMismatches
├─ events:
│   C        = IntervalSet<UInt64>  seqs actually received (live or sync, any Mac's answer)
│   H        = unknown | empty | through(n)   max of: status events_seq witnesses, SyncEnd bounds, max(C)
│   U        = IntervalSet<UInt64>  seqs the publisher declared unavailable (SyncEnd, any token)
│   latest   = [eventID: (seq, LiveRoomEvent)]  highest-seq transition received per event
│   digest   = [seq: (eventID, state)]  bounded; detects conflicting payloads
│   log      = ring of LiveRoomEvent (display), fromSync kept as diagnostics only
└─ limits overflowed: Bool   (C exceeded its interval cap → coverage unknown, see invariants)

Derived, never assigned:
  top        = max(H, max C)
  P          = contiguous prefix of C from 0
  missing    = [0, top] ∖ C          pending = missing ∖ U          lost = missing ∩ U
```

### 4.2 Public projection (replaces `eventsState`, `eventsSeq`)

```swift
public struct EventHistory {
    generation: UInt64?            // nil in legacy mode
    coverage: unknown | empty | completeThrough(UInt64) | missing(pending: [ClosedRange<UInt64>], lost: [ClosedRange<UInt64>])
    freshness: fresh | stale(age) | offline
    continuity: unknown | broken(sinceGeneration: UInt64) | verified
    caughtUp: Bool                 // derived, see invariants
    contiguousThrough: UInt64?     // top of P, replaces the old mutable eventsSeq
}
```

- `mapState` (`empty`, `current`, `stale(reason)`), `isMapCurrent`, `bricks`, `mapEpoch`, `mapRevision`, `status`, `isStatusFresh`, `events`, `eventsById` and `openEvents` keep their names. `eventsById`/`openEvents` are scoped to the active generation.
- `archivedOpenEvents` lists the open events of archived generations, tagged with their generation, for the "from an earlier run" section.
- New `StaleReason` cases: `newGeneration` and `unverified` (legacy).

### 4.3 Inputs and transitions

Validation (decode, `room_id`, D16 rules) comes first. A rejected message changes nothing except the counters and, for a map message, `stale(.rejected)`.

| input | condition | transition |
|---|---|---|
| `connectionOpened` | | `awaitingStatus(new id)`, `wire = undetermined`. Keep `active` and `archive` (stale display). Every retrieval → `idle`; old tokens are dead. Freshness → none |
| `connectionClosed` | | `offline`, retrieval `idle`. Map `current` → `stale(.disconnected)`. Coverage untouched |
| any v2 message (generation `m`) | no active, or `m > active.g` | **install**: archive the whole `active` (its tail is unknown: never sealed), create `GenerationState(m)`, then handle the message in it. Retrieval → `idle` |
| any v2 message | `m < active.g` | ignore, count `olderGeneration`. Never reactivates an archive |
| status of `active.g` | `seq > status.seq` | replace the status; H ← max(H, `events_seq` witness); `connection → observing(g)` if it arrived on this connection; D20 epoch-mismatch counting; then schedule retrieval |
| status of `active.g` | `seq ≤ status.seq` | duplicate or delayed copy: ignore, count. **Never** a generation signal |
| event of `active.g` (live or sync, any answer) | seq ∉ C | C ∪= {seq}; digest; `latest[eventID]` replaced only if `seq >` the held seq; append to log |
| event of `active.g` | seq ∈ C, same digest | duplicate: count |
| event of `active.g` | seq ∈ C, different digest | contract violation: count and log, keep the first. Never a restart signal |
| snapshot of `active.g` (live or sync) | chain none, or `seq ≥ appliedSeq` | replace bricks, epoch and appliedSeq |
| snapshot of `active.g` | `seq < appliedSeq` | ignore (an older answer, possibly another Mac's) |
| delta of `active.g` | `seq == appliedSeq + 1` and `base_epoch == epoch` | apply (`applying` as today); a failed apply → `stale(.rejected)` |
| delta of `active.g` | `seq ≤ appliedSeq` | duplicate |
| delta of `active.g` | otherwise | `stale(seqGap / wrongBase / deltaFirst)`; schedule map retrieval |
| live map message of `active.g` | | `highestLiveMapSeq = max(…, seq)` |
| `SyncEnd` of `active.g` | any token | H ← max(H, bound) (events); U ∪= `unavailable`. Publisher facts, true for every Mac |
| `SyncEnd` | token = current outstanding token for its topic | retrieval → `receiving(manifest)`. If the generation is newer, install first (row 3). The manifest applies to the SyncEnd's generation |
| `SyncEnd` | foreign or old token | facts only (row above); no retrieval change |
| `tick` | | freshness; retrieval timeouts (5, 10, 20, 30 s); request scheduling (≤ 1 per topic per 5 s) |
| `noteRequestUnsent` | | as today. Retrieval stays `waiting`; the next attempt follows the backoff |

**Retrieval completion.** Nothing else completes a retrieval: no timer, no request flag, no silence.
- `receiving(manifest)` becomes `idle` (done) only once:
  - **events:** every range in `manifest.delivered` ⊆ C;
  - **map:** `appliedSeq ≥ max(delivered)` in the same generation, or `delivered` is empty and `appliedSeq ≥ bound` (or `bound` NONE).
- A timeout in `waiting` or `receiving` sends a fresh token.

**When to request.** Only in `observing(g)`, only on a v2 connection, and only with a request path:
- **events:** `pending ≠ ∅`, with `since` = (map_epoch, seq) of P's top message, or `SINCE_NONE` when P is empty or its top's epoch was evicted;
- **map:** the map is not `current` for a reason a request can fix (no chain, a broken chain, D20 mismatch ×2, `appliedSeq < highestLiveMapSeq`), with `since` = (epoch, appliedSeq) or `SINCE_NONE`.
- `expected_generation = g` always.

### 4.4 Invariants (checked by the property tests after every input)

1. **Coverage only grows by receipt.** C changes only when a validated event of the active generation arrives. Disconnects, ticks, request failures, timers and outstanding requests never add to C. A generation change starts a fresh, empty C, and old C goes to the archive.
2. **No cross-generation contamination.** Every entry of `active` (C, U, `latest`, status, map chain, `highestLiveMapSeq`) came from a message carrying `active.g`.
3. **`completeThrough(n)` ⇒ every seq 0…n of `active.g` was received.** `empty` ⇒ a v2 publisher witness (a status with `EVENTS_SEQ_NONE` or a `SyncEnd` with `bound` NONE) of `active.g`, and C = ∅. `unknown` ⇒ no witness and C = ∅, or C overflowed.
4. **`caughtUp` ⇔** coverage is `empty` or `completeThrough(n)`, *and* a fresh status of `active.g` arrived on this connection whose `events_seq` covers max(C). It is an as-of observation, not a claim about the publisher's present.
5. **`lost` ranges are publisher-declared** (U). A request or a timeout never moves a seq from `pending` to `lost`.
6. **`latest[eventID]` holds the highest-seq transition received.** Arrival order and transport (live or sync) do not matter.
7. **Map `current` ⇔** chain held in `active.g`, `appliedSeq ≥ highestLiveMapSeq`, fresh status of `active.g` on this connection, its `map_epoch` == the mirror's epoch (with D20's two-status tolerance), and connected.
8. **Generations only rise.** `active.g` is non-decreasing over any input sequence, and an older-generation message changes nothing but counters.
9. **Idempotence.** Delivering any message twice changes no semantic state.
10. **Order independence.** Two delivery orders of the same set of same-generation messages produce the same C, U, H, `latest` and map chain.
11. **Eviction only lowers certainty.**
    - Limits (log 1000, `latest` 10,000, digest 10,000, C intervals 1,000) may drop display entries.
    - If C would exceed its interval cap, the generation is marked overflowed and coverage reads `unknown` from then on. It can never become more complete.
12. **Continuity.**
    - `broken(since: g_old)` once the mirror has archived a generation in which it observed anything.
    - `verified` when it has observed only `active.g` since launch.
    - `unknown` in legacy mode.
    - A retired generation has no final bound on the wire, so the redesign never claims whole-history continuity across generations (Astra §3).

### 4.5 Deviation from Astra's table, stated

Astra quarantines map/event messages of a newer generation until that generation's status arrives. This plan **installs the newer generation on its first validated message**, with H unknown until a status or `SyncEnd` of that generation witnesses a bound. Generations are durably increasing and wall-clock floored (§3), so a newer value always belongs to a later process. Installing early loses nothing and needs no buffer, and invariant 3 still holds, because coverage cannot be `complete` or `empty` without a witness. Retrieval still waits for `observing(g)`.

### 4.6 Legacy wire (old node, or a rolled-back image)

- **Mode.** The layout of the first room-memory message decoded on a connection fixes `wire`. A later message of the other layout on that connection is rejected and counted as `wireMismatch`. Every reconnect re-decides.
- **Legacy mode:**
  - The active generation is archived. The mirror holds one `legacyConnection` state with **identity unverified**.
  - Bricks and events are shown.
  - The map is never `current`: it renders `stale(.unverified)`, "unverified · Spark node predates publisher generations", with no overlays drawn as current.
  - `EventHistory.coverage = .unknown`, `continuity = .unknown`, `caughtUp = false`.
  - **No requests are sent** (a legacy node rejects a v2 `RequestSync`), and the map arrives with the 60 s keyframe (D29).
- No generation is ever synthesized from counters.

### 4.7 No request path

The deployed bridge has no `clientPublish` (`Tests/Fixtures/room/live/README.md`: `capabilities: ["connectionGraph"]`), and `requestSyncSink` is never assigned. Until Trevor picks the transport (D28/Q5):
- retrieval cannot run, so coverage typically reads `missing(pending: …)` after a mid-stream connect;
- the map becomes current from the next keyframe;
- the UI says "history incomplete · no request path" instead of retrying.

This plan does not change the transport.

### 4.8 UI (`Sources/LabCommandCore/LiveRoomView.swift`)

- **History line** (from `EventHistory`):
  - complete: "history complete through #n";
  - "no events this run";
  - incomplete: "history incomplete: n missing (k lost on the Spark)";
  - "history unverified (old Spark node)".
- **Restart banner.** "Spark node restarted · events from the earlier run may be missing" while `continuity = broken`. Archived open events are listed as "earlier run · may be outdated", never drawn as current changes.
- **Status line.** It gains a short generation tag (`run …1234`, last 4 digits). `status.eventsSeq` stays.
- `activity.historyCuts` becomes `activity.generationChanges`. New counters: `olderGeneration`, `contractViolations`, `wireMismatch`.

## 5. File-by-file changes

**ROS messages**

| file | change |
|---|---|
| `ros/labcommand_msgs/msg/MemoryStatus.msg` | append `uint64 publisher_generation`; add `GENERATION_UNKNOWN=0` comment; `events_seq` "in this generation" |
| `ros/labcommand_msgs/msg/MapBricks.msg` | append `uint64 publisher_generation` (after `payload`); mask rule comment |
| `ros/labcommand_msgs/msg/ChangeEvent.msg` | append `uint64 publisher_generation` (after `mask`) |
| `ros/labcommand_msgs/msg/RequestSync.msg` | append `uint64 expected_generation`, `uint8[16] request_token`; `GENERATION_UNKNOWN=0` |
| `ros/labcommand_msgs/msg/SyncEnd.msg`, `SeqRange.msg` | new (§2.2) |
| `ros/labcommand_msgs/msg/EventsFallback.msg` | delete |
| `ros/labcommand_msgs/CMakeLists.txt` | interface list: + SyncEnd, SeqRange; − EventsFallback |

**Python oracle (`spark/room_memory`)**

| file | change |
|---|---|
| `room_memory/messages.py` | `publisher_generation` on `MemoryStatus`/`MapBricks`/`ChangeEvent` (`None` = legacy layout; encoder writes legacy only when asked, for reference files); trailing-byte layout detection; mask generation rule; `RequestSync` v2 + legacy decode; `SyncEnd`, `SeqRange` codecs; validation symmetric with Swift |
| `room_memory/generation.py` (new) | `allocate_generation(state_dir, room_id, now_ms)`: lock, read, `max(stored+1, now_ms)`, `write_atomic` + dir fsync, fail closed |
| `room_memory/memory.py` | constructor takes `publisher_generation`; stamps every status, event and map message (masks too); `sync()` returns the answer messages **and** the `SyncEnd` (§2.3), incl. the BUILDING empty answer and the "nothing missed" empty map answer; `write_atomic` dir fsync; references saved in the legacy layout |
| `room_memory/node.py` | allocate before creating publishers; `/room_memory/sync/end` publisher (reliable, volatile, depth 10); publish `SyncEnd` after the answer |
| `room_memory/offline.py` | fixed generation argument |
| `tests/test_messages.py`, `tests/fixtures/hand/*`, `PROVENANCE.md` | v2 goldens (§6.3), legacy goldens kept for decode, hashes |
| `tests/test_memory.py` | `SyncEnd` per answer case; generation on replays; `unavailable` monotone |
| `tests/test_generation.py` (new) | allocation, wall-clock floor, malformed file, lock held, unwritable dir, overflow |
| `tests/test_node.py` | allocation happens before the first publish; `SyncEnd` topic wiring |

**C++ node (`spark/room_memory_cpp`)**

| file | change |
|---|---|
| `include/room_memory/messages.hpp`, `src/messages.cpp` | same as Python, byte for byte |
| `include/room_memory/generation.hpp`, `src/generation.cpp` (new) | allocator, same semantics |
| `include/room_memory/memory.hpp`, `src/memory.cpp` | as Python `memory.py` |
| `node/room_memory_node.cpp` | allocate first; `sync/end` publisher; publish order |
| `tools/replay.cpp`, `tools/offline*.cpp` | fixed generation; the replay driver's `sync` command takes a token and an expected generation, and prints the `SyncEnd` |
| `tests/test_messages.cpp`, `tests/test_memory.cpp`, `tests/test_generation.cpp` (new), `tests/CMakeLists.txt` | mirror the Python tests; register `generation` |
| `tests/differential/replay.py`, `test_differential.py` | the sync cases compare answer bytes **and** `SyncEnd` bytes; restart cases use two generations |
| `CMakeLists.txt` | add `src/generation.cpp` to the core library |

**Swift**

| file | change |
|---|---|
| `Sources/LabCommandCore/RoomMemoryMessages.swift` | `publisherGeneration: UInt64?` on the three messages (nil = legacy); `RequestSync.expectedGeneration`, `RequestSync.token: RequestToken?` (16 bytes); `SyncEnd`, `SeqRange` |
| `Sources/LabCommandCore/RoomMemoryCDR.swift` | trailing-byte layout detection; v2 encode/decode; mask rule; `SyncEnd` codec; header comment ("five messages") |
| `Sources/LabCommandCore/LiveRoom/SeqIntervals.swift` (new) | canonical `IntervalSet<UInt64>` (insert, union, subtract, contains-range, count cap) |
| `Sources/LabCommandCore/LiveRoom/GenerationState.swift` (new) | per-generation map + events + status, pure |
| `Sources/LabCommandCore/LiveRoom/Retrieval.swift` (new) | per-topic retrieval state, tokens, manifests, backoff |
| `Sources/LabCommandCore/LiveRoom/EventHistory.swift` (new) | public projection types |
| `Sources/LabCommandCore/LiveRoom/RoomMemoryMirror.swift` | rewritten as the coordinator (§4); public API kept where §4.2 says so |
| `Sources/LabCommandCore/LiveRoom/LiveRoomProtocol.swift` | `.syncEnd` topic (`/room_memory/sync/end`, `labcommand_msgs/msg/SyncEnd`) |
| `Sources/LabCommandCore/LiveRoom/LiveRoomClient.swift` | subscribe to `sync/end`; nothing else (it already stamps and sends `Action.requestSync`) |
| `Sources/LabCommandCore/LiveRoomView.swift` | §4.8; `LiveRoomFixture` feeds v2 goldens and a `SyncEnd` |
| `Sources/LabCommand/LiveRoomView.swift`, `RoomMapView.swift`, `RoomMapRenderer.swift`, `AppState.swift` | only if a renamed member reaches them; they pass the mirror through today |
| `Sources/LabCommandCore/LiveRoom/API.md` | rewritten to the API surface only (types, inputs, outputs, `EventHistory` meanings); every rule points at protocol §5. Removes the ~15 statements that contradict §5 today (2 s settle, SINCE_NONE settling, baseline jumps, restart heuristics, 5/10/20/30 vs 5 s) |

**Swift tests** (`Sources/LabCommandCoreTestsRunner`)

| file | change |
|---|---|
| `LiveRoomModel.swift` (new) | reference model, publisher and network simulators (§6.1) |
| `LiveRoomTraceTests.swift` (new) | seeded traces, shrinking, the regression seeds |
| `LiveRoomTests.swift` | rewritten (§6.2) |
| `LiveRoomViewTests.swift` | `testLiveRoomEventRowsAndIncomplete` (:217) and `testLiveRoomBannersAndResync` (:365) move to `EventHistory`; new rows for the restart banner, lost ranges and legacy |
| `RoomMemoryCDRTests.swift` | v2 goldens (LE + BE), `SyncEnd`, layout detection (legacy/v2/invalid trailing counts), mask-generation rule; random round trips and mutation fuzz extended to v2 and `SyncEnd`; legacy byte patches unchanged |
| `main.swift` | register the trace tests |

**Docs and scripts**

| file | change |
|---|---|
| `docs/ROOM-MEMORY-PROTOCOL.md` | the single normative statement (see below) |
| `docs/phase3-live-integration.md` | P13 and §4.3: restart detection is the publisher generation. `node_started_at` (P10) stays as diagnostics only, no longer case 3. F4 and the failure table rows. P17 → `SyncEnd`. "Publisher generation" vs P9's room generation |
| `docs/LIVE-ROOM-PLAN.md` | point the history and restart sections at §4 of this plan and protocol §5 |
| `scripts/spark/make_room_memory_goldens.py` | v2 values (generation, token) and `SyncEnd` |
| `scripts/check-shared-goldens.sh` (new) | `cmp` every file in `Tests/Fixtures/room/golden/hand/` with its `spark/room_memory/tests/fixtures/hand/` copy |
| `Tests/Fixtures/room/live/README.md` | the four captures are legacy-layout; they stay as legacy decode fixtures |

**Protocol doc edits** (`docs/ROOM-MEMORY-PROTOCOL.md`):
- §1.4 adds generation scoping.
- §1.5 adds `GENERATION_UNKNOWN` and `SEQ_NONE` in `SyncEnd`.
- §2 topics table: + `sync/end`, − `sync/events_fallback`.
- §3.1–§3.3 and §3.5 get the new fields and sizes. A new §3.8 defines `SyncEnd` and `SeqRange`.
- §5 is rewritten:
  - the `seq` rule scoped to (generation, topic);
  - no restart cases 1–3;
  - answers per §2.3;
  - completion per §4.3;
  - legacy behaviour.
- §7 sizes.
- §8: D34 publisher generation, D35 `RequestSync` token + expected generation, D36 `SyncEnd` (supersedes D27 and D26's mark), and D37 the generation-scoped mirror (supersedes D23's restart cases and its open residual). D23, D26 and D27 are each marked superseded, not deleted.
- §9 status.
- Drift fixed on this tree:
  - stray common-field lines after the §3.8 wire size (:471–474);
  - the `,,` in D27 (:802);
  - §9's claim that `EventsFallback` is pending.
- Astra's "`MemoryStatus` omits `events_seq`" applied to the `liveroom-b` checkout. On `pubgen`, §3.3 already has it (:269, :284).

## 6. Test plan

### 6.1 Independent reference model and seeded traces (Swift runner)

- **Publisher simulator** (`LiveRoomModel.swift`):
  - Generations with real restarts. Each restart starts every counter at 0, takes a higher generation, and raises `map_epoch` per §3.
  - A 100-message event log with open-event latests, a 10-message map log, and keyframes.
  - A 1 Hz status, and answers computed by the §2.3 tables. These are written independently of the production code: plain arrays and `Set`s.
- **Network simulator:**
  - Per-topic FIFO with loss, but no reordering within a topic, which is what the protocol promises.
  - Arbitrary interleaving across topics, and delays long enough for old-generation messages to arrive after a restart.
  - Duplicated status copies (transient_local), disconnects that drop in-flight data, and request failures (unsent).
  - Other Macs' requests, so foreign replays and foreign `SyncEnd`s appear on the shared sync topics.
  - Restarts while disconnected in which the new generation's counters pass the old ones before the reconnect (counter reuse).
  - Partial, empty and fallback answers, and BUILDING periods with no map.
- **Oracle.** It never reads the mirror:
  - `received[(g, seq)]` = the full event, for every event delivered to the mirror;
  - `publisherLog[g]` = what was actually emitted.
  - Expected coverage, H, `pending`/`lost`, and `latest` (max seq per event) are computed with `Set<UInt64>`. Map validity comes from replaying the publisher's map chain.
- **After every input,** compare:
  - the active generation;
  - coverage, `pending`, `lost` and `caughtUp`;
  - `latest` states;
  - map chain, epoch and `current`;
  - retrieval completion;
  - plus invariants 1–12 (§4.4).
  - With limits disabled, equality is required. With small injected limits, the eviction tests require only that every claim the mirror makes is implied by the oracle (certainty may drop, never rise).
- **Properties.**
  - Safety: no coverage without receipt, and no contamination.
  - Idempotence: re-feed a random prefix.
  - Order independence: permute same-generation deliveries and compare.
  - Liveness: after a quiet tail (connected, lossless, ticks past every backoff, request path available), the mirror converges to `completeThrough(publisher bound)` or `missing(lost: exactly the publisher's unavailable)`, retrieval is idle, and the map is current.
  - Resource limits.
- **Runs.** A seeded `SplitMix64` (existing `SyntheticRoom.swift`:102). The default gate runs 2,000 traces × ≤ 400 inputs. `LIVE_ROOM_TRACES=n` and `LIVE_ROOM_SEED=s` reproduce one trace.
- **Shrinking.** On failure, a delta-debugging shrinker removes input chunks while the failure persists. It prints the seed and the minimal trace as Swift source, ready to paste as a regression seed.

### 6.2 Regression seeds: the ten Codex findings and Astra's pair

Each is a hand trace in `LiveRoomTraceTests.swift`, run on the v2 wire and on the legacy wire.

| # | source | trace | v2 expectation | legacy expectation |
|---|---|---|---|---|
| 1 | review 2 | status `events_seq` 100, live 101 before any replay | `missing(pending [0,100])`; complete only after delivery covers it | coverage `unknown`, never complete |
| 2 | review 3a | as 1, then disconnect/reconnect, matching status | still `missing [0,100]` | `unknown` |
| 3 | review 3b | live 101 + status 101; replay delivers 0–50 only, SyncEnd says [0,101] | `missing [51,100]`; retrieval not done; retry with a new token | no request, `unknown` |
| 4 | review 4 | sync 101 first, reconnect, sync 0–50, status 101 | `missing [51,100]` | `unknown` |
| 5 | review 5 | sync seq 0 only; two statuses `events_seq` 2; request outstanding | `missing [1,2]` whatever the request state | `unknown` |
| 6 | review 6 | replay 0–2 (candidate/confirmed/accepted, one event), then live 1 | duplicate; event stays ACCEPTED; `completeThrough(2)`; no generation change | event ACCEPTED; `unknown` |
| 7 | review 7 | another Mac's replay gives confirmed@1 without 0; live candidate@0 | 0 fills C; event stays CONFIRMED; no generation change | same event state; `unknown` |
| 8 | review 8 | sync (event 2, seq 1), then live (event 1, seq 0) | both recorded; `completeThrough(1)` after status 1 | `unknown` |
| 9 | review 9 | events 0–5, disconnect, publisher restarts, new status seq passes the old one, no new events (NONE) | the new generation is installed; active coverage `empty` (witnessed); continuity `broken`; old open events listed "earlier run · may be outdated" | **restart not detectable**: `unknown`/`unknown`, never `complete`, map never current (the old assertion is replaced on purpose) |
| 10 | review 10 | restart without reconnect, startup snapshot missed; answer snapshot at seq 0 | new generation; snapshot @0 applies; map current after a fresh status | map `stale(.unverified)` |
| A | Astra | publisher continues while the Mac is away | same generation, replay fills, `completeThrough` | `unknown` |
| B | Astra | publisher restarts while away and passes the old counters | new generation, old archived, no contamination | indistinguishable from A, so `unknown` for both |

Further fixed traces:
- an old-generation snapshot/status/event arriving after the install is ignored;
- a newer-generation event before its status installs, with H `unknown`;
- a foreign `SyncEnd` gives facts only;
- a conflicting payload for one seq is counted, not treated as a restart;
- a BUILDING `SyncEnd` (bound NONE) completes;
- the "nothing missed" map answer is empty;
- a legacy-then-v2 wire switch across reconnects.

### 6.3 What happens to the existing tests

- **`LiveRoomTests.swift`, rewritten for the new reducer** (same names where the scenario survives):
  - Events :475, EventsSeqGap :540, ReplayAndFallbackEdges :581;
  - GenerationsAndPartialHistory :641–829, rewritten as v2 generation tests;
  - EpochChange :391, Reconnect :442 (now: await status, then request with `expected_generation` and `since`), GapRequestsSync :333.
- **Moved to regression seeds:**
  - OutstandingSyncHidesNothing :830 → #5;
  - DelayedLiveTransitionIsDuplicate :854 → #6;
  - OlderTransitionIsOutOfOrder :895 → #7;
  - LowerSeqFromAnotherEventIsNoRestart :1062 → #8;
  - NoneEventsSeqIsRestartEvidence :1092 → #9 (**inverted**: NONE is no longer restart evidence);
  - EveryRestartResetsMapHighWater :1170 → #10.
- **Deleted, replaced by §6.1:**
  - HistoryInvariantProperty :924 (its oracle reads `mirror.eventsById`);
  - OnlyStatusCutsHistoryProperty :1203 (it simulates a restart by resetting status while the same event plan continues).
- **Kept:**
  - `testLiveRoomGoldensDecode` :170 (legacy goldens), plus a v2 twin;
  - `testLiveRoomLiveCaptures` :228 (legacy captures → legacy mode).
- **Codec tests:**
  - **New hand goldens**, written byte by byte and annotated like today's, in both fixture directories:
    - `memory_status_v2_le.hex`;
    - `map_bricks_delta_v2_le.hex` / `_be.hex`;
    - `change_event_appeared_v2_le.hex`;
    - `change_event_moved_mask_v2_le.hex` (two generations: mask, then parent);
    - `request_sync_events_v2_le.hex` (SINCE_NONE, expected 0);
    - `request_sync_map_v2_be.hex`;
    - `sync_end_events_empty_le.hex` (bound NONE);
    - `sync_end_events_fallback_le.hex` (d = 2, u = 1);
    - `sync_end_map_replay_be.hex`.
  - The legacy goldens stay. Swift decodes them as legacy, and Python/C++ use them for the legacy reference-file path and legacy `RequestSync` decode.
- **Python/C++.** The Python and C++ suites decode/encode every v2 golden, and `check-shared-goldens.sh` compares every shared file. The differential test compares Python and C++ byte for byte on: a scripted session across two generations; every §2.3 answer case with its `SyncEnd`; and the legacy `RequestSync` answer.

## 7. Gate (every implementation step, and before the PR)

1. `scripts/check-shared-goldens.sh` — every shared hand golden identical (today's gate compares only `memory_status_le.hex`).
2. `pytest` under `spark/` — pass. Count ≥ today's 300 plus the new tests.
3. C++ `cmake` Release build — **0 warnings**; `ctest` — all pass (today 15, plus `generation`).
4. `swift run LabCommandCoreTestsRunner` — PASS, including the default 2,000-trace run with 0 failures and every regression seed.
5. `./build.sh --no-install` — exit 0.
6. Live Room screenshots (`scripts/live-room-shots.sh`), with v2 fixtures: a complete history, a restart banner, lost ranges, legacy unverified. Check them by eye.
7. Codex review rounds as Trevor directs. This plan does not run Codex.

`~/Developer/lc-tasks/gate-full.sh` runs 2–5. It gains step 1 by calling the new script instead of its single `cmp`.

## 8. Rollout (only the room-memory container is redeployed)

Trevor does these steps. This work does not touch the Spark or `~/Applications/LabCommand.app`.

1. **Prerequisite.**
   - Verify that the container runs with `-v ~/room-memory:/root/room-memory` (Dockerfile comment, `spark/room_memory_cpp/Dockerfile` :6–7) and that `state_dir` resolves inside that volume.
   - Without it, the node now refuses to start. That is intended: a generation that does not survive restarts is not durable. The wall-clock floor would still order generations, but the plan does not rely on it alone.
2. **Mac app first.**
   - Build and install the app from the merged branch.
   - Against the still-running old node it runs in legacy mode: map and events shown, "unverified", no requests.
   - `main` has no `RoomMemoryMirror`. An app built from `main` does not decode these topics, so a new node breaks nothing there.
   - An app built from `live-room-ui` decodes the legacy layout only. It would reject v2 messages (≥ 4 trailing bytes) until it is replaced, which is why the app goes first.
3. **Redeploy the room-memory container.**
   - Rebuild `labcommand/room-memory-cpp:jazzy` from this branch. The image bundles `labcommand_msgs`, so its foxglove_bridge on :8767 advertises the v2 definitions and `/room_memory/sync/end` (the whitelist `^/room_memory/.*` already covers it).
   - Restart it. The lidar bridge on :8765 and every other container stay untouched.
4. **Verify.**
   - `<state_dir>/<room_id>/publisher_generation` exists. A second restart raises it.
   - `swift run LabCommandCoreTestsRunner live-room-probe` shows v2 statuses with a generation.
   - The Live Room shows the run tag. After a container restart it shows the restart banner and an `empty` or replayed history for the new run.
   - With no request path (§4.7), the history reads incomplete until Trevor enables one. That is expected.
5. **Rollback.**
   - Redeploy the previous image. It ignores the `publisher_generation` file and still reads its references, which stay in the legacy layout (§3). The Mac drops to legacy mode.
   - Rolling forward again allocates `max(stored+1, now_ms)`, still above every earlier generation. Rollback is safe in both directions.

## 9. Order of implementation (later steps, each passing the gate)

1. Protocol doc and `.msg` files: the contract frozen first (D34–D37), API.md rewritten.
2. Hand goldens and the three codecs (Swift, Python, C++), legacy decode, layout detection.
3. Generation allocator and node stamping (Python, then C++); `SyncEnd` answers; differential tests.
4. The reference model and trace harness, written **before** the reducer and run against a stub, so the oracle is not shaped by the implementation.
5. `SeqIntervals`, `GenerationState`, `Retrieval`, and the new `RoomMemoryMirror`. Delete the heuristics; then the regression seeds and property runs.
6. UI projection, view tests, screenshots.
7. Doc sweep (phase3 doc, LIVE-ROOM-PLAN), full gate, PR text (supersedes #13 and #17).
