# Phase 0 plan: room-memory contracts and fixtures

Scope: PR 0.1, 0.2 and 0.3 from `~/Developer/lc-tasks/RESEARCH_BRIEF.md` §3 Phase 0, with the message formats taken from §2.4 and the budget from §2.5. Everything below runs on the Mac. The Spark is not contacted, nothing is installed, and nothing is committed.

## Gate (inherited from brief §3)

- `swift build` and `./build.sh --no-install` show no new warnings. The only allowed warning is the existing `AppState.swift:371` `Thread.sleep` warning.
- `swift run LabCommandCoreTestsRunner` passes. The baseline before Phase 0 is 156 groups; with Phase 0 it is 169.
- No to-do or fix-me markers and no dead code in any new file: Swift, Markdown, `.msg`, scripts.
- Doc comments on all public API.
- No edits outside the files listed here.

## Deliverables mapped to acceptance criteria

### PR 0.1: `docs/ROOM-MEMORY-PROTOCOL.md`

| brief criterion | deliverable | status |
|---|---|---|
| the §2.4 messages | every message, its legal ROS 2 `.msg` text, and a field table | done on the Mac |
| every field has a unit and a "missing" rendering | each field table has unit, valid range, the wire encoding of "missing", and the rendering when it is missing or stale | done on the Mac |
| state machines (memory state, event lifecycle) | transition tables with triggers and per-state rendering | done on the Mac |
| resync rule | stated per topic, with the `RequestSync` answer rule (D19) | done on the Mac |
| honest-state rendering rules (UNKNOWN ≠ free, `alpha = NaN` ≠ 0) | own section | done on the Mac |
| byte budget | §2.5 budget, exact wire sizes and the measured sizes from the 0.2 size test | done on the Mac |
| Trevor's decisions on D5, D9, D19 (2026-10-14) | all-free bricks kept; `compression` before a packed payload, zlib by default; `RequestSync` and sync topics | done on the Mac; the parts of D19 beyond his field list are marked for his confirmation |
| Codex review signs off | — | **deferred**: needs Codex |
| Trevor answers Q1–Q4 | Q1–Q4 listed at the end, unanswered | **deferred**: needs Trevor |

The doc also has a "Decisions for review" section. It lists each place where §2.4 had to be made legal ROS 2 syntax or was underspecified, and the choice made.

### PR 0.2: `labcommand_msgs` and the Swift CDR codecs

| brief criterion | deliverable | status |
|---|---|---|
| `labcommand_msgs` package | `ros/labcommand_msgs/` with `package.xml`, `CMakeLists.txt`, `msg/{MapBricks,ChangeEvent,MemoryStatus,Epoch,RequestSync}.msg` | written; **colcon build deferred** (no ROS 2 on the Mac) |
| Swift CDR codecs | `Sources/LabCommandCore/RoomMemoryMessages.swift` (types) and `RoomMemoryCDR.swift` (encode and decode, classic CDR, both byte orders on decode; `MapBricks` payload under none, zlib and lzfse through the Compression framework) | done on the Mac |
| round-trip tests | seeded random messages: encode, decode, compare, re-encode byte-identical | done on the Mac |
| malformed input | truncation at every byte, invariant violations, unsupported encapsulation and brick_dim, bad or wrong-size compressed streams, the 16 MiB payload limit, and a deterministic mutation fuzz | done on the Mac |
| golden CDR bytes from `ros2` on the Spark decode identically | **substitute**: 12 hand-derived goldens per the CDR spec, annotated per field, in `Tests/Fixtures/room/golden/hand/`. Decoded, and re-encoded exactly; for the 2 compressed ones, up to the payload, whose stream must inflate to the hand payload. `scripts/spark/make_room_memory_goldens.py` would serialize the same values with `rclpy` | **Spark goldens deferred**: the script is written and not run |
| size test: synthetic 8×8×3 m room snapshot ≤ 500 KB compressed, report the number | ray-cast synthetic room → `MapBricks` snapshot. Reports brick count and whole-message sizes under none, zlib (the default; raw DEFLATE, RFC 1951, Apple level 5) and lzfse. Asserts the zlib snapshot ≤ 500 KB: measured 59,024 B (4,081 bricks; 546,926 B uncompressed, 54,091 B lzfse) | done on the Mac |

### PR 0.3: fixtures under `Tests/Fixtures/room/`

| brief item | deliverable | status |
|---|---|---|
| (a) `snapshots/a`, `snapshots/b` | copied byte for byte. A test decodes all four and checks frame ids and non-empty clouds | done on the Mac |
| (b) 12 Oct box takes | — | **deferred**: on the Spark, waits on Q4 |
| (c) synthetic scenes: box added / removed / moved / person walk-through / rig shifted 5 cm | a deterministic generator, run as `swift run LabCommandCoreTestsRunner generate-room-fixtures`. It writes PointCloud2 CDR `.bin` files (the format `change-detect` reads) and a `MANIFEST.json` with ground truth. A test regenerates the scenes and compares bytes | done on the Mac |
| `change-detect` is run on each and the outputs are stored as expected files; static pair gives 0 regions | — | **deferred**: the CLI exists only on `change-detection` (06b5c83), so this branch cannot run it. The manifest names every pair and its ground truth so the follow-up only needs to run the CLI and store the outputs |

## What can't be done without the Spark (or others)

- Spark golden bytes. The script is written and not run. The follow-up adds a test that decodes `golden/spark/*.bin` and compares against the hand goldens.
- `colcon build` of `labcommand_msgs` in the Jazzy container.
- A check that the deployed foxglove_bridge accepts client publishing of `RequestSync` (D19).
- Spark-side LZFSE. Python's stdlib has none, so the Spark sends zlib (D9).
- The 12 Oct box takes (Q4).
- Expected `change-detect` outputs (needs the `change-detection` branch).
- Codex sign-off and Trevor's answers to Q1–Q4.

## Files

- `docs/PHASE0-PLAN.md`, `docs/ROOM-MEMORY-PROTOCOL.md`
- `ros/labcommand_msgs/{package.xml,CMakeLists.txt,msg/*.msg}` (five messages, including `RequestSync.msg`)
- `Sources/LabCommandCore/RoomMemoryMessages.swift`, `Sources/LabCommandCore/RoomMemoryCDR.swift`
- `Sources/LabCommandCoreTestsRunner/RoomMemoryCDRTests.swift`, `SyntheticRoom.swift` (room, scanner, evidence grid), `RoomFixtureTests.swift` (scene catalog, generator, fixture tests), and the dispatch and test calls in `main.swift`
- `Sources/LabCommandCoreTestsRunner/PointCloud2CDRTests.swift`: `CDRTestField`, `CDRTestCloud` and `cdrXYZICloud` lose `private`, so the generator reuses the existing PointCloud2 test encoder instead of duplicating it
- `Tests/Fixtures/room/` (README, snapshots, synthetic, golden/hand)
- `scripts/spark/make_room_memory_goldens.py`
