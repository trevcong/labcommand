# ChangeDetection

Pure-Swift occupancy diff for LabCommandCore. One library file, one test file registered in the existing assert runner, and a small `change-detect` executable. Foundation only. The LabCommand app, Metal renderer, and live-session code stay untouched.

## What exists

`PointCloudFrame` (`Sources/LabCommandCore/PointCloudDecoder.swift`) is the input: `positions: [Point3D]`, plus intensity, sensor, timestamp, and `frameId`. `Point3D` is three `Float`s. The core library has no simd.

Loaders the CLI will call as they stand:

- `PointCloudDecoder.loadPCD(from:)` reads a `.pcd` file (ASCII, binary, or binary-compressed).
- `PointCloud2CDR.decode(_:)` reads a CDR `sensor_msgs/PointCloud2`, including the 4-byte encapsulation header. It ignores extra fields, skips NaN/Inf, and returns a `PointCloudFrame`.

Saved frames (`~/Developer/lc-tasks/snapshots/a/`, and `b/` about 10 s later, same sensors, rig unmoved):

| File | Bytes | Frame | Points | Layout |
|---|---|---|---|---|
| `mid360.bin` | 521,881 | `livox_frame` | 20,064 | CDR little-endian, `point_step` 26, xyz+intensity float32 |
| `jt128.bin` | 2,995,397 | `hesai_lidar` | 115,200 | same layout |

Both headers are `00 01 00 00` (CDR LE). `PointCloud2CDR.decode` already accepts that layout. `snapshots/b/jt128.bin` is the same length as `a/` and is not byte-identical. `snapshots/b/mid360.bin` is 519,385 bytes (19,968 points). Same-file against itself is the empty-diff smoke check. On `a/` against `b/` with evidence on and the defaults below, both pairs print `regions=0`. The counts behind that cut are in [Static-room blips](#static-room-blips): at `minVoxels` 1 the Mid-360 still has one appeared sheet of 7 voxels, one cell thick, and `minExtent` 2 is what drops it.

`LabCommandCore` is one SPM target, one type-family per file, public `enum` namespaces (`RobustBounds`, `PointCloud2CDR`, `RigTransforms`). Tests live in `Sources/LabCommandCoreTestsRunner`, are plain `expect` functions, and are registered from `main.swift`. `swift test` / XCTest is unused (CLT installs may not have it). `Package.swift` is tools 5.9, macOS 14.

## Scope

In:

- Build a voxel occupancy map from one `PointCloudFrame`.
- Diff two maps into appeared and disappeared voxels, with a 1-voxel dilation so sensor jitter does not count as change.
- Cluster each set with 26-connectivity and drop clusters under a voxel-count threshold.
- Optionally pair an appeared cluster with a disappeared cluster into one `moved` region.
- Report kind, centroid, AABB, occupied volume, and voxel count per region.
- Tests in the custom runner, including a release timing assertion.
- `swift run change-detect <a.pcd|a.bin> <b.pcd|b.bin>`.

Out:

- SwiftUI, Metal, the command-center UI, recording, foxglove, and `LiveSession`.
- ICP, deskew, or any alignment. Both clouds are diffed in the coordinates they already have.
- Merging Mid-360 and JT128. `RigTransforms.inWorld` stays the caller's job if a later tool wants one room frame. The same rigid motion applied to both clouds does not change a same-sensor diff except at voxel boundaries, so the CLI does not transform.
- Persistence, ROS messages, and writing voxels back out.

A Mid-360 frame against a JT128 frame is a large, meaningless diff (different frames, Mid-360 is inverted, counts differ). The CLI will still run it. The meaningful call is two captures of the same sensor, rig unmoved.

## Files

| Path | Role |
|---|---|
| `Sources/LabCommandCore/ChangeDetection.swift` | Public API: keys, config, occupancy, diff, cluster, pairing |
| `Sources/LabCommandCoreTestsRunner/ChangeDetectionTests.swift` | Synthetic scenes and the timing test |
| `Sources/LabCommandCoreTestsRunner/main.swift` | One added call, `runChangeDetectionTests()`, after `runRobustBoundsTests()` |
| `Sources/ChangeDetect/main.swift` | CLI |
| `Package.swift` | Executable product `change-detect`, target `ChangeDetect`, dependency `LabCommandCore` |

`README.md` gets a short CLI line under Tests. No other docs.

## Public API

Namespace `enum ChangeDetection`, nested types, all `Sendable`. Value types that tests compare are `Equatable`. Doc comments on every public type and method, in the style of `PointCloud2CDR` and `RobustBounds` (`Parameters` / `Returns` / `Throws`).

```swift
public enum ChangeDetection {
    public struct Config: Sendable, Equatable {
        public var voxelSize: Float          // 0.05 m
        public var maxRange: Float?          // nil: no clip
        public var minPoints: Int            // 1
        public var tolerance: Int            // 1 voxel, Chebyshev
        public var minVoxels: Int            // 6
        public var minExtent: Int            // 2 voxels on every AABB axis
        public var pairMoves: Bool           // true
        public var maxMove: Float            // 1.5 m between centroids
        public var moveCountRatio: Float     // 2: larger count / smaller count
        public var evidenceOfAbsence: Bool   // false. v2: carve free space
        public var origin: Point3D           // (0, 0, 0), meters, after-frame sensor
        public var rayStride: Int            // 1. power of two, 1...64
        public var reportUnobserved: Bool    // false
        public init()
    }

    public struct OccupancyMap: Sendable, Equatable {
        public var voxelSize: Float
        public var occupied: Set<Int64>
        public var acceptedPoints: Int
        public var droppedPoints: Int
    }

    public enum Kind: String, Sendable, Equatable {
        case appeared
        case disappeared
        case moved
        case unobserved          // v2: before-occupied, neither free nor still occupied
    }

    public struct Region: Sendable, Equatable {
        public var kind: Kind
        public var voxelCount: Int
        public var centroid: Point3D
        public var boundsMin: Point3D
        public var boundsMax: Point3D
        public var volume: Float
        public var minKey: Int64          // v2: minimum packed key of this component
        /// Set only for `.moved`: the disappeared partner.
        public var fromVoxelCount: Int?
        public var fromMinKey: Int64?     // v2: partner minimum packed key
        public var fromCentroid: Point3D?
        public var fromBoundsMin: Point3D?
        public var fromBoundsMax: Point3D?
    }

    public struct Report: Sendable, Equatable {
        public var regions: [Region]
        /// Changed voxels after dilation, before the cluster-size cut.
        public var appearedVoxels: Int
        public var disappearedVoxels: Int
        public var unknownVoxels: Int    // v2: 0 when evidence is off
        public var beforeVoxels: Int
        public var afterVoxels: Int
    }

    public enum Failure: Error, Sendable, Equatable {
        case invalidConfig(String)
        case voxelSizeMismatch(Float, Float)
    }

    public static func occupancyMap(
        _ frame: PointCloudFrame,
        config: Config = Config()
    ) throws -> OccupancyMap

    public static func diff(
        _ before: OccupancyMap,
        _ after: OccupancyMap,
        config: Config
    ) throws -> Report

    /// Build both maps with `config`, then `diff`.
    public static func compare(
        _ before: PointCloudFrame,
        _ after: PointCloudFrame,
        config: Config = Config()
    ) throws -> Report

    /// Canonical v2 record. No trailing newline. `JSONEncoder` is not a substitute.
    public static func json(
        report: Report,
        before: OccupancyMap,
        after: OccupancyMap,
        config: Config
    ) -> String
}
```

`VoxelKey` is a public enum in the same file so tests can recompute a key.

```swift
public enum VoxelKey {
    /// Valid indices are `-limit ..< limit` (`limit == 1 << 20`).
    public static let limit = 1_048_576
    public static func pack(i: Int, j: Int, k: Int) -> Int64?
    public static func unpack(_ key: Int64) -> (i: Int, j: Int, k: Int)
    public static func index(_ coordinate: Float, inverseVoxelSize: Float) -> Int
}
```

`compare` is the function tests and the CLI call. `occupancyMap` and `diff` stay public so a caller can map once and diff against several later frames, and so tests can inspect `occupied`.

### Config checks

`occupancyMap` and `diff` throw `Failure.invalidConfig` when:

- `voxelSize` is non-finite or `<= 0`
- `maxRange` is non-nil and (non-finite or `<= 0`)
- `minPoints < 1`, `minVoxels < 1`, or `minExtent < 1`
- `tolerance` is outside `0...4`
- `maxMove < 0` or non-finite
- `moveCountRatio < 1` or non-finite

`diff` throws `voxelSizeMismatch` unless the two maps' `voxelSize` values are exactly equal and equal to `config.voxelSize`. Tolerance above 4 is rejected because the neighborhood is `(2t+1)³` probes per candidate voxel (7³ = 343 at t = 3, 9³ = 729 at t = 4).

### Occupancy

One pass over `frame.positions` through `withUnsafeBufferPointer`.

Skip a point, and count it in `droppedPoints`, when any coordinate is non-finite, when `maxRange` is set and `x²+y²+z² > maxRange²` (Euclidean, origin of whatever frame the points are already in), or when a quantized index is outside `VoxelKey.limit`. A skipped point never inserts.

Quantize with `inverseVoxelSize = 1 / voxelSize` once per map:

```text
q = coordinate * inverseVoxelSize
i = Int(q)                         // truncates toward zero
if q < 0 and Float(i) != q:
    i -= 1                         // floor, so (-voxelSize, 0) is index -1
```

`Int(Float)` is only reached for finite `q` with `abs(q) < Float(limit)`. `limit` is 2²⁰, which `Float` represents exactly. This keeps the cell that straddles the origin one voxel wide. `floor` from libm is off this path.

Index `(i, j, k)` packs into one `Int64`:

```text
bias = limit                       // 2^20
u, v, w = i + bias, j + bias, k + bias     // each in 0 ..< 2^21
key = Int64(bitPattern: (u << 42) | (v << 21) | w)
```

Bit 63 stays clear. At the default 0.05 m the covered half-extent is `limit * voxelSize` = 52,428.8 m. A lab cloud is inside that. Points past it increment `droppedPoints`.

`minPoints == 1` (the default, and the fast path): insert keys into `Set<Int64>` reserved to `frame.count`. Each point is at most one voxel, so the set does not rehash on a full cloud.

`minPoints > 1`: increment `Dictionary<Int64, Int32>` (reserved the same way), then insert keys whose count is at least `minPoints`. The dictionary dies at the end of the build. The stored map is always a `Set`.

The map does not keep the original points. Centroids come from voxel centers, so a later diff allocates nothing proportional to N except the changed-voxel working sets.

### Diff and dilation

Let `B` be the before set and `A` the after set.

Raw candidates: `onlyBefore = B` minus `A`, `onlyAfter = A` minus `B`. Set subtraction.

A voxel in `onlyBefore` is disappeared when no key in `A` has Chebyshev distance `<= tolerance`. Same test for appeared against `B`. `tolerance == 0` is the raw subtraction and does no probing.

For `tolerance == 1` each candidate does up to 26 extra `contains` probes (the center was already absent). Unpack the key, add `di,dj,dk ∈ {-1,0,1}`, repack, probe. Generate neighbor keys in registers. The dilation looks at the other occupancy set, so a real surface that merely slid by one cell is suppressed, and an isolated flicker next to a still-occupied surface is suppressed too.

Cluster the surviving appeared keys and the surviving disappeared keys as two separate graphs. An appeared voxel is never a neighbor of a disappeared voxel during the component walk.

### Clusters

26-connected components on the surviving key set. Copy keys to an array, walk them with an explicit stack (an array used as a stack) and a `visited: Set<Int64>`. Membership of a component does not depend on set iteration order. Geometry is accumulated from indices, not from visit order:

- `voxelCount` = component count
- centroid = mean of cell centers `((i + 0.5) * voxelSize, …)`, summed in `Double`, stored as `Float`
- `boundsMin` = `(iMin, jMin, kMin) * voxelSize` (inclusive corner of the low cell)
- `boundsMax` = `(iMax + 1, jMax + 1, kMax + 1) * voxelSize` (exclusive corner of the high cell)
- `volume` = `Float(voxelCount) * voxelSize³` (occupied volume)

Drop a component when `voxelCount < minVoxels`, or when any AABB edge covers fewer than `minExtent` voxels (`iMax - iMin + 1`, and the same for `j` and `k`). Defaults are `minVoxels` 6 and `minExtent` 2.

`minExtent` 2 is 0.10 m at the 0.05 m voxel. A component that is one cell thick fails it at any length. The static Mid-360 pair's only component above 6 voxels is that shape: 7 voxels in a 5×3×1 box. Raising `minVoxels` to 8 would hide the sheet too, and it would leave the 0.10 m shell (8 voxels, 2×2×2) with no count margin, so the count cut stays at 6. The extent cut is the one a one-cell streak cannot reach. `minExtent` 1 keeps every component that passes `minVoxels`. The 0.40 m shell in the added, removed, and moved tests is 296 voxels (49× the count cut) and 8 cells on each edge (4× the extent cut). The same generator at side 0.10 m is 8 voxels and 2 cells on each edge: it still adds, removes, and moves, two voxels above the count cut, on the extent floor.

### Moved pairing

Included, behind `pairMoves` (default true).

Occupancy has no descriptors. Two clusters of similar size, closer than a short lab motion and mutually nearest, are the same object leaving one place and showing up in another. That is the whole heuristic. It will mis-label two different objects of similar size that appear and disappear in the same gap, and it will refuse a pair when a second candidate is almost as close. Both outcomes are accepted. Unpaired clusters stay `appeared` or `disappeared`, so a wrong move is the failure to avoid, and ambiguity leaves the raw kinds in the report.

Gates, all required:

1. Distance between centroids `<= maxMove` (default 1.5 m).
2. `max(count) / min(count) <= moveCountRatio` (default 2).
3. The pair is mutual nearest neighbor under (1) and (2).
4. Ambiguity: reject when either side has another gated candidate closer than `1.5 ×` the mutual distance. Those four clusters stay unpaired.

Greedy over mutual pairs sorted by distance, then by appeared centroid `(x, y, z)`. Each region is used once.

A `moved` region describes the object once, at the appeared pose:

- `kind == .moved`
- `voxelCount`, `centroid`, `boundsMin`, `boundsMax`, `volume` are the appeared component
- `fromVoxelCount`, `fromCentroid`, `fromBoundsMin`, `fromBoundsMax` are the disappeared component

Volume is the appeared occupied volume. Summing both sides would count one object twice. The AABB is the appeared pose. The gap between poses is not part of the box; the old pose is on the `from*` fields.

`pairMoves == false` skips this and returns the filtered appeared and disappeared regions.

### Region order

Sort for output, because `Set` iteration order changes per process:

1. Kind order: `moved`, `appeared`, `disappeared`
2. `voxelCount` descending
3. Centroid `x`, then `y`, then `z`, ascending
4. For `moved`, `fromCentroid` the same way as a final tie-break

`Report.appearedVoxels` and `disappearedVoxels` count keys after dilation and before the `minVoxels` and `minExtent` cuts, so a test can see that a speck or a one-cell sheet existed and was then dropped.

### Complexity

`N` points in one cloud, `V ≤ N` occupied voxels, `C` candidate voxels in the raw symmetric difference, `t` tolerance, `K = (2t+1)³`, `R` regions after the size cut.

| Step | Time (expected) | Memory |
|---|---|---|
| Occupancy, `minPoints == 1` | `O(N)` inserts | `O(V)` for the `Set<Int64>` |
| Occupancy, `minPoints > 1` | `O(N)` increments plus `O(V)` filter | one `Dictionary<Int64, Int32>` then the set |
| Symmetric difference | `O(V)` | the two candidate sets |
| Dilation | `O(C · K)` probes, `K = 27` at the default | the surviving sets |
| 26-connected components | `O(C)` voxels, 26 probes each | visited set + stack |
| Pairing | `O(R²)` distances | negligible; `R` is tens on a lab diff |
| Sort | `O(R log R)` | |

Aligned scans make `C` the surface that actually moved, far below `N`. Map build dominates. Two disjoint clouds make `C ≈ V_before + V_after` and the dilation/cluster probes dominate.

Working memory for two ~10⁵-voxel maps is a few megabytes of Swift set storage (hash buckets plus one `Int64` per voxel). The CLI holds two frames and two maps at once. A JT128 frame is ~3 MB on disk and 115,200 `Point3D`s.

### Timing budget

Target: under 100 ms in a release build (`swift run -c release`, `-O`) on an M-series Mac, for two 80,000-point clouds, measured around `compare` only. File IO stays outside the timer.

Default `minPoints == 1` so the timed path is `Set` insert, not the dictionary. `reserveCapacity(frame.count)` on each set. Warm one untimed `compare` (allocator and hasher), then time a second call with `ContinuousClock`.

Budget inside those 100 ms, as a guide for the overlapping-room case the test builds:

- two occupancy builds, 80k inserts each: the bulk, on the order of a few tens of milliseconds
- difference plus dilation on a small changed set: a few milliseconds
- clustering that changed set: a few milliseconds

The release test asserts `< 100 ms` for that overlapping pair. A same-file run of the real JT128 snapshot (115,200 points, empty diff) is the other bar: `C = 0`, so this checks map build on the larger real frame and should also land under 100 ms. Headroom on the 80k test is what makes the 115k frame fit.

`_isDebugAssertConfiguration()` is true for the default `swift run` (`-Onone`) and false for `-O`. The debug assertion uses 3 seconds. Debug Swift on this hash-set loop is often an order of magnitude slower; 3 seconds is loose enough to be stable and tight enough that a quadratic neighbor search still fails.

If the release 80k overlapping case or the 115k same-file case exceeds 100 ms, the first fix is in the hot loop (capacity, the branchy quantizer, no per-point allocation, neighbor keys in registers). A custom open-addressed table is a later change; the shipped structure stays `Set<Int64>` / `Dictionary<Int64, Int32>`.

## CLI

Product name `change-detect`, target `ChangeDetect`, path `Sources/ChangeDetect/main.swift`. One file. `swift run change-detect -- <before> <after>`.

```text
change-detect [--voxel-size m] [--max-range m] [--min-points n]
              [--tolerance n] [--min-voxels n] [--min-extent n]
              [--max-move m] [--no-pair] <before.pcd|before.bin> <after.pcd|after.bin>
```

Two positional paths are required. Flags override `Config` and default to the library defaults. `--no-pair` sets `pairMoves` false. Unknown flags and a wrong arity print usage to stderr and exit 1.

Extension, lowercased:

- `.pcd` → `PointCloudDecoder.loadPCD(from:)`
- `.bin` → `Data(contentsOf:)` then `PointCloud2CDR.decode`

Any other extension exits 1 with a line that names the two accepted extensions. A binary PCD is a `.pcd`. A `.bin` is a CDR blob, which is what `snapshots/a/*.bin` contains.

Stdout, one stable block, seconds as milliseconds with one decimal:

```text
before: mid360.bin  points=20064  voxels=12345  dropped=0
after:  mid360.bin  points=20064  voxels=12345  dropped=0
voxel_size_m=0.050  tolerance=1  min_points=1  min_voxels=6  min_extent=2  pair_moves=true
appeared_voxels=0  disappeared_voxels=0
elapsed_ms=12.3
regions=0
```

Each region is one line:

```text
region 0 moved voxels=240 volume_m3=0.030000 centroid=1.200,0.500,0.300 bounds=[1.000,0.300,0.100 .. 1.400,0.700,0.500] from_voxels=240 from_centroid=0.200,0.500,0.300 from_bounds=[0.000,0.300,0.100 .. 0.400,0.700,0.500]
```

`appeared` and `disappeared` omit the `from_*` fields. Exit 0 after a successful compare, including zero regions. Thrown `PointCloudError` and `ChangeDetection.Failure` print to stderr and exit 1. The library call is wrapped in `ContinuousClock`; the printed `elapsed_ms` includes both map builds, the diff, clustering, and pairing.

## Tests

`ChangeDetectionTests.swift` follows `RobustBoundsTests.swift`: file-private scene builders, `expect`, a `runChangeDetectionTests()` that prints one line per group and calls `passed()`. First failure exits 1. Seeded splitmix-style generator, same constants as `roomCloud` in `RobustBoundsTests.swift` (`state = state &* 6364136223846793005 &+ 1442695040888963407`). No `Date` and no system randomness, so a run is reproducible.

Shared scene, private to the test file:

- Room shell, not a random volume fill. Floor and four walls on a 0.02 m grid, room about 4 m × 4 m × 2 m, so a 0.05 m surface cell holds several points. Grid is inset from the outer boundary by a fraction of a voxel so sample points do not sit on a cell face.
- Box shell, 0.40 m on a side, same 0.02 m spacing, six faces, center at `(1.0, 0.5, 0.30)`, clear of the walls by more than `(tolerance + 1)` cells. Hollow, because a lidar return is a surface. Face centers still average to the geometric center.
- `PointCloudFrame(positions:)` only. Intensity and sensor are unused by the diff.

Groups:

1. **Key math.** Pack/unpack round-trips for negative and positive indices, including `limit - 1` and `-limit`. `pack` returns nil outside that. A coordinate at `-0.01` with voxel size `0.05` has index `-1`. A coordinate at `0.06` has index `1` at 0.05 m and index `0` at 0.10 m. A non-finite point increments `droppedPoints` and adds no key.

2. **Voxel size.** The room at 0.10 m occupies fewer keys than the same room at 0.05 m. Both counts are stable across two builds.

3. **Range and min points.** A point at range just under 15 m is kept and a point just over 15 m is dropped when `maxRange` is 15. With `maxRange == nil` both are kept. Four points in one cell and one point in a neighbor: `minPoints == 2` keeps a single key. `voxelSize == 0` throws `invalidConfig`.

4. **Box added.** Identical room in both frames, box only in the after frame. Exactly one region, `appeared`, centroid within half a voxel of `(1.0, 0.5, 0.30)`, `voxelCount` above `minVoxels`, volume equal to `Float(voxelCount) * 0.05³`, AABB covering the box and nothing out to the walls.

5. **Box removed.** Box only in the before frame. Exactly one region, `disappeared`, same centroid and count checks.

6. **Box moved.** Box center `x = 0` in the before frame and `x = 1.0` in the after frame (20 cells at 0.05 m, so the shells do not meet even with tolerance 1). With default pairing, exactly one `moved` region: centroid near the after center, `fromCentroid` near the before center, `fromVoxelCount == voxelCount`. With `pairMoves == false`, exactly two regions, one `appeared` and one `disappeared`, and no `moved`.

7. **Noise only.** Start from one dense room+box cloud. Build the before frame by adding a seeded offset in `[-0.01, 0.01]` m on each axis and dropping every point whose seeded draw is `< 0.20`. Build the after frame with a different seed, same ranges. Default config (`tolerance == 1`, `minVoxels == 6`). Expect zero regions. ±1 cm is less than one 5 cm cell, so a point crosses at most one face; dilation absorbs it. A wall cell holds several grid points, and a neighbor cell remains occupied unless a whole neighborhood is deleted, which a 20% drop on this grid does not do. Also assert that the identical cloud compared with itself yields zero regions and `appearedVoxels == 0`.

8. **Threshold.** Room plus box, plus two isolated points that occupy two voxels far from the box. `minVoxels == 6` reports only the box. `minVoxels == 1` with `minExtent == 1` reports the box and the speck (`appearedVoxels` includes the speck in both runs). The speck is one cell thick, so the default `minExtent` would drop it even at `minVoxels` 1.

9. **Mismatch.** `diff` of a 0.05 m map and a 0.10 m map throws `voxelSizeMismatch`. `tolerance == 5` throws `invalidConfig`.

10. **Performance.** 80,000 points on the room-scale surface grid (repeat the grid with a half-cell jitter pattern if the shell is shorter than 80k, still deterministic). The after cloud is the same points plus a seeded ±1 cm shift and an 8,000-point patch relocated by 1 m (a real change, so clustering does work). The patch is a one-cell surface, so this call sets `minExtent` 1. The other knobs stay at the defaults. Time the second `compare`. Assert elapsed `< 3 s` when `_isDebugAssertConfiguration()` is true and `< 0.1 s` otherwise. Print the measured milliseconds in the passing line.

The noise-only group and the performance group use separate builders. The performance cloud is large; the behavior tests stay on the small room.

## Acceptance

- `swift build --product LabCommandCoreTestsRunner` and `swift build -c release --product LabCommandCoreTestsRunner` add no warning whose path is `ChangeDetection.swift`, `ChangeDetectionTests.swift`, or `Sources/ChangeDetect/`.
- `swift build -c release --product change-detect` is clean the same way.
- `swift run LabCommandCoreTestsRunner` passes, including the new groups.
- `swift run -c release LabCommandCoreTestsRunner` passes, and the performance group prints a time under 100 ms.
- `swift run -c release change-detect -- ~/Developer/lc-tasks/snapshots/a/jt128.bin ~/Developer/lc-tasks/snapshots/a/jt128.bin` exits 0, prints `points=115200` twice, `regions=0`, and `elapsed_ms` under 100.
- The same command on `mid360.bin` prints `points=20064` and `regions=0`.
- No `TODO`, `FIXME`, or unused private declaration in the new files. No force-unwrap in the library. No code path that builds a map and ignores it.
- Every public type and method on `ChangeDetection` and `VoxelKey` has a doc comment that states units (meters, voxels) and what `volume` measures.

Pre-existing warnings in other files are left as they are.

## Risks

- **Sparse real returns.** The noise-only guarantee is for the dense synthetic grid. A Mid-360 or JT128 surface cell often holds one return. Independent dropout then creates disappeared voxels, and dilation only saves them when a neighbor survived. With evidence on, most of those candidates become unknown rather than disappeared. What remains on the static Mid-360 pair is one-cell-thick, and `minExtent` 2 drops it. See [Static-room blips](#static-room-blips). `--tolerance 2` still absorbs a two-cell slide when that is the motion that should not count. A streak that staircases to two cells on every axis still has to clear `minVoxels` 6. On these two pairs the largest such component is 5 voxels.
- **Cell faces.** A point parked on a boundary flips cells under a sub-millimeter shift. The synthetic grids are inset from faces. Real clouds will sit on faces; tolerance 1 covers a one-cell flip when the motion is shorter than `voxelSize`. Jitter larger than the voxel (a 1 cm shift in a 5 mm grid) crosses more than one cell and shows up as change. `tolerance` has to be at least `ceil(jitter / voxelSize)`.
- **Rig motion.** If the tripod moves between snapshots, the whole room is a change. This module does not register clouds.
- **Cross-sensor files.** `livox_frame` vs `hesai_lidar` will report a room-sized diff. The CLI does not refuse it.
- **Small objects.** Anything under `minVoxels` cells, or thinner than `minExtent` cells on any axis, is dropped on purpose. At 0.05 m that is a blob under 6 voxels, or a sheet one cell (0.05 m) thick no matter how long. A 0.10 m shell still passes (8 voxels, 2×2×2). A 0.40 m shell cannot be dropped by either default. Lower `--min-voxels` and `--min-extent 1` when looking for a one-cell sheet, and expect the static-room streak back.
- **Moved false pairs.** Two similar blobs inside `maxMove` with no closer candidate become one `moved` region. The ambiguity gate (the 1.5× second-candidate rule) is what keeps a swap of two identical boxes from being labeled cleanly; that case stays unpaired and should not be asserted as `moved`.
- **Volume is a voxel count.** A one-return wall becomes a 5 cm slab. `volume` is `count * size³`, the occupied cells, not a mesh volume. A `moved` region's `volume` is the appeared side only.
- **Release vs debug.** The 100 ms bar is `-O` only. Judging the budget from `swift run` without `-c release` will fail a correct implementation.
- **Key overflow at tiny voxels.** Below about a millimeter, `VoxelKey.limit` no longer covers a multi-meter room, and those points increment `droppedPoints` silently. The CLI prints `dropped`, which is the visible signal. Default 0.05 m is far from that edge.
- **Hash-set constant factors.** A fully disjoint pair (every voxel changed, then 26 probes each) can miss 100 ms even when the overlapping test passes. The acceptance test is the overlapping 80k pair plus the identical 115k JT128 file. A future worst-case benchmark needs its own, looser number.

## v2: determinism and evidence of absence

This section is the cross-language contract for the Spark `room_memory` port. Research brief §2.2 keeps this Swift module as the reference implementation and the parity oracle. PR 2.1 matches voxel keys and occupancy-diff regions. PR 2.2 matches free-space evidence. A C++ or Python port reproduces the bytes of `ChangeDetection.json` on the same points and the same `Config`. Where this section tightens a v1 rule, v2 is the one the port implements.

v2 changes four things, and leaves the rest of the v1 pipeline standing:

- Region order becomes a total order on kind, voxel count, and packed min key.
- Move pairing states its tie outcome: equal binary32 distances stay unpaired.
- An optional evidence mode carves free space with a specified 3D DDA. It is off by default, and with it off the disappeared set is the v1 set.
- The CLI grows `--json`, the parity golden. The human text block stays, and it is not the golden.

Arithmetic is IEEE-754 round-ties-to-even. Binary32 is the point, the voxel size, the quantizer product, the DDA, the centroid distance, and the count ratio. Binary64 is only the centroid index mean below. Flush-to-zero and flush-inputs-to-zero are outside the contract; every value in this pipeline is normal at a 0.05 m lab cloud inside `VoxelKey.limit`.

### Determinism

The report and the JSON are a pure function of `Config` and the multisets of `positions`. Permuting a cloud, duplicating a point that already occupies its voxel when `minPoints == 1`, or iterating a `Set` in another order leaves the output bytes unchanged. Swift's per-process hash seed must not reach a sort, a representative point, a pair, or a region.

#### Quantization

`inverse = Float32(1) / voxelSize`, one binary32 division per compare. The dividend is the binary32 value `1` (`0x3f800000`). Reuse that `inverse` for both maps and for every ray. Do not form it in binary64 and round.

For one finite coordinate, with that `inverse`:

```text
q = coordinate * inverse          # binary32 multiply
i = truncTowardZero(q)            # the integer part, toward 0
if q < 0 and Float32(i) != q:
    i = i - 1                     # floor for the negative non-integers
```

`truncTowardZero` is defined for every finite `q` that reaches it. The occupancy guard admits `q` only when `q` is finite and `abs(q) < Float32(VoxelKey.limit)`. `VoxelKey.limit` is 2²⁰, and `Float32(limit)` is exact. Every integer with absolute value below 2²³ is exact in binary32, so `Float32(i) != q` is a true "not an integer" test inside the guard. `floor` from libm is not on this path. `-0` compares equal to `+0` and is not `< 0`, so a coordinate of `-0` has index `0`.

An exact integer `q` keeps index `q`. The boundary plane belongs to the cell that has that plane as its lower face, the cell on the positive side of the plane. `(-voxelSize, 0)` is index `-1`. `q == +limit` and `q == -limit` fail the strict `<` guard and the point is dropped.

A point is dropped, and counted in `droppedPoints`, when any coordinate is non-finite, when the guard rejects an index, or when `maxRange` is set and the binary32 value `(x*x + y*y) + z*z` is `>` `maxRange * maxRange`. The squares and the two additions are binary32, left-associated as written. A point whose squared range compares equal is kept. `minPoints` counts are exact integers over the accepted keys. A voxel count above 2³¹−1 is outside the contract. Membership at `minPoints == 1` is the set of accepted keys. Membership above 1 is the keys whose count is at least `minPoints`. Both are independent of insertion order.

#### Key packing

```text
bit 63          bits 62…42              bits 41…21              bits 20…0
0               u = i + bias            v = j + bias            w = k + bias
```

`bias = VoxelKey.limit = 2²⁰ = 1048576`. Each lane is 21 bits. `u`, `v`, and `w` lie in `0 ..< 2²¹`. The integer value is `(u << 42) | (v << 21) | w`, stored as a signed `Int64` with bit 63 clear. Numeric order of these keys is lexicographic order of `(i, j, k)`. `pack` returns no key for an index outside `-limit ..< limit`. The JSON writes the key as a decimal integer string, so the golden does not depend on host endianness.

#### Centroids and AABBs

Index sums for a component are exact integers. Accumulate them in binary64. Each index widens to binary64 exactly, and the sum of any component that fits in memory stays inside the exact integer range of binary64 (a sum of `N` indices below 2²⁰ is exact for `N < 2³³`). The sum is commutative and associative in that range, so visit order does not change it.

```text
n    = Float64(voxelCount)                 # exact for counts below 2^53
size = Float64(voxelSize)                  # exact widening of the binary32 voxel size
c    = ((sum / n) + 0.5) * size            # three binary64 ops, round-ties-to-even
centroid coordinate = Float32(c)           # one round, ties-to-even
```

The parenthesization is `((sum / n) + 0.5) * size`. A running mean of meter-valued centers is a different contract.

The AABB stays in binary32, matching the committed evaluator. `|index| < 2²⁰`, so the integer converts to binary32 exactly:

```text
boundsMin coordinate = Float32(indexMin) * voxelSize
boundsMax coordinate = Float32(indexMax + 1) * voxelSize
```

Each product is one binary32 multiply. Widening the AABB multiply to binary64 and rounding back is a different contract and will miss the golden.

`volume = ((Float32(voxelCount) * voxelSize) * voxelSize) * voxelSize`, left-associative binary32. For a moved region the volume is the appeared component only.

`Region` gains two fields. `minKey` is the minimum packed key of the component, the appeared component for `.moved`. `fromMinKey` is the disappeared partner's minimum packed key, and it is set only for `.moved`. Both are part of `Equatable`.

#### Region order

Sort with this total order. Kind rank is `moved = 0`, `appeared = 1`, `disappeared = 2`, `unobserved = 3`.

1. Kind rank, ascending.
2. `voxelCount`, descending.
3. `minKey`, ascending as a signed integer.
4. `fromMinKey`, ascending. A missing `fromMinKey` sorts before any present key.

`minKey` is unique inside one kind, because the components of one kind partition a key set and a key is used by one region. The comparator is irreflexive on distinct regions, so an unstable sort has one legal result. This replaces the v1 centroid sort. Two regions with the same kind and the same count follow min-key order even when their centroids compare the other way. The threshold test's larger cluster still comes first, because the count key stays ahead of the min key.

#### Moved-pairing tie-breaks

Gates run in this order, on the appeared and disappeared clusters only. Unobserved clusters are not candidates.

1. Binary32 centroid distance `d` is `sqrt((dx*dx + dy*dy) + dz*dz)` with `dx`, `dy`, `dz` binary32 subtractions of the stored centroid coordinates, the additions left-associated, and `sqrt` the correctly rounded binary32 square root. Require `d <= maxMove`.
2. `hi = Float32(max(count))`, `lo = Float32(min(count))`, binary32 division. Require `hi / lo <= moveCountRatio`. Counts above 2²⁴ are not exact in binary32; the comparison is still the binary32 one.
3. Unique nearest neighbor, both ways, under the binary32 `==` of those distances. When two or more gated candidates share the minimum distance, that cluster has no nearest neighbor and pairs with nobody. Do not break the tie by index, by min key, or by discovery order.
4. Ambiguity. `limit = d * Float32(1.5)`. `1.5` is exact in binary32. Reject the pair when either side has another gated candidate whose distance is `< limit`. A candidate at exactly `1.5 × d` stays inside the gate.

Each appeared cluster has at most one mutual partner, and each disappeared cluster has at most one, because of gate 3. Greedy consumption therefore admits every pair that reached it. The implementation still sorts those pairs, so a later edit cannot inherit an unspecified order: distance ascending under binary32 `<`, then appeared `minKey` ascending, then disappeared `minKey` ascending. A cluster is used once.

`pairMoves == false` skips pairing. The disappeared and appeared sets are unchanged.

### Ray-based disappeared

`Config` gains:

| Field | Default | Meaning |
|---|---|---|
| `evidenceOfAbsence` | `false` | Carve free space from the after cloud |
| `origin` | `(0, 0, 0)` | Sensor origin of the after frame, in the same coordinates as the points |
| `rayStride` | `1` | Power of two from 1 through 64. Default 1 casts every occupied after voxel |
| `reportUnobserved` | `false` | Also emit `.unobserved` regions |

`rayStride` defaults to **1**. Every occupied after voxel casts, which is the evidence a disappearance claim needs. Stride 64 keeps about one voxel in 64 (about 1.6 percent). That subsample is too sparse to treat a miss as "still there." The stride field stays in the contract for ports and for the printed long-ray benchmark below. It is not the acceptance bar, and raising it is not how a slow run gets faster. The JSON records the stride that was used, and a port matches that recorded stride.

`invalidConfig` rejects a `rayStride` outside `{1, 2, 4, 8, 16, 32, 64}`, a non-finite origin component, and, when evidence is on, an origin that fails the quantizer guard. Evidence off still validates `rayStride` and finite origin, and it does not quantize the origin.

With evidence off, `unknownVoxels` is 0 and no `.unobserved` region is produced. Appeared, disappeared, dilation, and pairing match v1 plus the v2 order and tie rules.

With evidence on, the after cloud has three states per voxel:

- **Occupied.** The after occupancy set, same rules as v1.
- **Free.** Some cast ray's traversal contains the voxel, and the voxel is not occupied. Occupied wins over free.
- **Unknown.** Everything else. Unknown is not stored as a set. It is the complement, and only before-occupied keys are classified against it.

A before-occupied voxel that the after map still occupies is unchanged. A before-occupied voxel that is free in the after map is a disappeared candidate. A before-occupied voxel that is unknown increments `unknownVoxels` and is not a disappeared candidate. Dilation then runs on the disappeared candidates only, against `after.occupied`, with the same Chebyshev `tolerance` as v1. A candidate suppressed by dilation is neither disappeared nor unknown: a neighbor explains it, and it increments neither count. `appearedVoxels` and `disappearedVoxels` stay post-dilation and pre-`minVoxels` / pre-`minExtent`. `unknownVoxels` is pre-cut the same way. Appeared candidates are the v1 set and do not consult the free set.

Tolerance does not move an unknown voxel into the disappeared set, and it does not remove it from `unknownVoxels`. A surface that slides toward the sensor leaves the old cell in shadow. That cell stays unknown. A slide away from the sensor, shorter than `tolerance` cells, is free and then suppressed. Both outcomes are required.

`.unobserved` is optional output for the unknown before-occupied keys. Cluster them with the same 26-connectivity, the same geometry, and the same `minVoxels` and `minExtent` cuts. They take kind `.unobserved`, they sit in `regions` under the v2 order, and they do not enter move pairing. With `reportUnobserved == false` the count is still reported and the regions are not built. An unobserved object is not a change: it cannot become `.moved`, and it is not a disappeared region that failed a threshold.

#### Which rays

Cast from `origin` into the after cloud only. The before cloud casts nothing.

One ray per occupied after voxel whose key passes the stride filter, ending at a real return in that voxel. Extra returns in the same voxel do not cast. At `rayStride == 1` that is one ray per occupied voxel. The representative return is the accepted point in that voxel that is minimum under the binary32 total order below, applied to `x`, then `y`, then `z`. Dropped points are not candidates. The choice is independent of input order.

```text
orderKey(bits) = (~bits) if bits has the sign bit set, else (bits | 0x80000000)
```

`bits` is the big-endian IEEE-754 binary32 pattern as a `uint32`. Smaller `orderKey` precedes. This places `-0` before `+0`. Two points that compare equal under this order are the same values and produce the same ray.

`rayStride > 1` keeps key `K` when `splitmix64(K) & (rayStride - 1) == 0`.

```text
z = uint64(bitPattern: K) + 0x9E3779B97F4A7C15     # mod 2^64
z = (z xor (z >> 30)) * 0xBF58476D1CE4E5B9
z = (z xor (z >> 27)) * 0x94D049BB133111EB
z = z xor (z >> 31)
```

Every multiply and add is mod 2⁶⁴. The shifts are logical. This is the SplitMix64 finalizer. Do not use a language hash seeded per process.

The ray endpoint is that representative point, not the cell center. The end voxel is the quantizer applied to the point, which is the occupied voxel. A port that rays every return, or that rays the cell center, will miss the golden as soon as one voxel holds two different returns.

`maxRange` does not clip a ray a second time. A dropped point occupies nothing and casts nothing. An accepted point casts the whole segment from `origin` to the point.

#### DDA

The walker is the thin Amanatides–Woo grid traversal (Amanatides and Woo, Eurographics 1987): parametric `t` in voxel-index space, and on a tie every tied axis steps in the same iteration. A ray through a grid corner does not visit the face-adjacent voxels. The diagonal test below locks that choice.

Work in `q`-space, `q = coordinate * inverse`, the same binary32 multiply as occupancy. Voxel faces are the integers of `q`. Let `qO` be the origin's `q` and `qE` the endpoint's `q`, componentwise. `dir = qE - qO` in binary32. Start indices and end indices come from the quantizer, not from a separate floor.

```text
step = 0 when the start index equals the end index, else sign(endIndex - startIndex)
```

`sign` is +1 or −1. A component whose indices match does not step, including when `dir` is a nonzero value still inside one slab. Slabs are convex, so a segment with both ends in one slab never crosses out of it.

```text
tDelta = +infinity when step == 0, else Float32(1) / abs(dir)

tMax = +infinity when step == 0
     = (Float32(startIndex + 1) - qO) / dir    when step == +1
     = (Float32(startIndex) - qO) / dir        when step == -1
```

`Float32(1)` is `0x3f800000`. `abs` is binary32. A computed `tMax < 0` is replaced with `0` before the walk. `+infinity` is the IEEE binary32 infinity. A NaN `tMax` or `tDelta` ends the ray; marks already made stay made.

```text
if start == end: mark nothing and return
cap = |di| + |dj| + |dk|          # end index minus start index, at the start
steps = 0
while steps < cap:
    if current == end: return     # do not mark the endpoint voxel
    if pack(current) fails: return
    mark current free
    tNext = min(tMaxX, tMaxY, tMaxZ)    # binary32 <, left to right
    # capture tNext first; then test all three axes against it
    if tMaxX == tNext: x += stepX; tMaxX = tMaxX + tDeltaX
    if tMaxY == tNext: y += stepY; tMaxY = tMaxY + tDeltaY
    if tMaxZ == tNext: z += stepZ; tMaxZ = tMaxZ + tDeltaZ
    steps += 1
```

`tMax + tDelta` is one binary32 add, accumulated. Do not recompute `tMax` from the plane each step. Equality is binary32 `==`. When all three `tMax` values compare equal, all three axes advance and the walk moves to the diagonal neighbor. The start voxel is marked when it differs from the end voxel, so the sensor cell is free unless some return occupies it. The endpoint voxel is never marked free by that ray. After every ray, free is the union of the marks minus the after occupied set. Ray order does not change the union.

The loop count is at most `|di| + |dj| + |dk|`. Reaching the cap without landing on the end voxel ends the ray. Both languages share that cutoff.

### Canonical JSON

`ChangeDetection.json(report:before:after:config:) -> String` is the golden encoder. It is hand-written. `JSONEncoder` is not a substitute: key order and number formatting differ. The CLI `--json` writes that string followed by a single `\n` and nothing else on stdout. Exit codes stay as in v1. The human text block is the output when `--json` is absent. Text floats stay at the v1 decimal widths and are not bit-exact. With evidence on, the text summary line gains `unknown_voxels=<n>` after `disappeared_voxels`. With evidence off that token stays absent, so a v1 grep of the summary line still matches. `.unobserved` prints as the kind word `unobserved` and omits `from_*`.

`--evidence` sets `evidenceOfAbsence`. `--origin x,y,z` sets the origin, default `0,0,0`. `--ray-stride n` and `--unobserved` set the other two fields. `--min-extent n` sets `minExtent` in voxels (default 2). Unknown flags still print usage and exit 1.

The function return has no leading or trailing whitespace. Encoding:

- UTF-8, no BOM.
- Object keys in the order below, not alphabetical.
- Separators `:` and `,` with no spaces.
- Integers as decimal JSON numbers, no leading zeros, a leading `-` when negative.
- `true`, `false`, `null`.
- Strings in RFC 8259 quotes. Every string in this schema is ASCII and needs no escapes.
- A binary32 value is eight lowercase hex digits of its big-endian bit pattern, inside a string, with no `0x` prefix. `0` is `00000000`. `-0` is `80000000`.
- A packed key is a decimal integer string, so values above 2⁵³ stay exact.
- Absent optional fields are omitted, not null. `from_*` is omitted unless `kind` is `moved`. `max_range_bits` is `null` when `maxRange` is nil.

Top-level key order: `schema`, `evidence`, `report_unobserved`, `origin_bits`, `ray_stride`, `config`, `before`, `after`, `appeared_voxels`, `disappeared_voxels`, `unknown_voxels`, `regions`.

`schema` is the string `labcommand.change-detection.v2`. `origin_bits` is `[x, y, z]`. `config` key order: `voxel_size_bits`, `max_range_bits`, `min_points`, `tolerance`, `min_voxels`, `min_extent`, `pair_moves`, `max_move_bits`, `move_count_ratio_bits`. `min_extent` is the integer voxel count. `before` and `after` key order: `points`, `accepted`, `dropped`, `voxels`, with `points = accepted + dropped`. There is no elapsed-time field and no filename. Timing stays on the human block.

Region key order: `kind`, `voxel_count`, `min_key`, `centroid_bits`, `bounds_min_bits`, `bounds_max_bits`, `volume_bits`, then for `.moved` only: `from_voxel_count`, `from_min_key`, `from_centroid_bits`, `from_bounds_min_bits`, `from_bounds_max_bits`. Bit arrays are `[x, y, z]`. `kind` is `appeared`, `disappeared`, `moved`, or `unobserved`.

Golden for the clear-line-of-sight fixture in the tests section, which is the library string the in-runner test compares. The CLI file is that string plus `\n`.

```text
{"schema":"labcommand.change-detection.v2","evidence":true,"report_unobserved":false,"origin_bits":["00000000","00000000","00000000"],"ray_stride":1,"config":{"voxel_size_bits":"3d4ccccd","max_range_bits":null,"min_points":1,"tolerance":0,"min_voxels":6,"min_extent":1,"pair_moves":true,"max_move_bits":"3fc00000","move_count_ratio_bits":"40000000"},"before":{"points":9,"accepted":9,"dropped":0,"voxels":9},"after":{"points":1,"accepted":1,"dropped":0,"voxels":1},"appeared_voxels":0,"disappeared_voxels":8,"unknown_voxels":0,"regions":[{"kind":"disappeared","voxel_count":8,"min_key":"4611776178381914112","centroid_bits":["3f99999a","3ccccccd","3ccccccd"],"bounds_min_bits":["3f800000","00000000","00000000"],"bounds_max_bits":["3fb33333","3d4ccccd","3d4ccccd"],"volume_bits":"3a83126f"}]}
```

`min_key` `4611776178381914112` unpacks to `(20, 0, 0)`. The centroid and the AABB are the v2 formulas on indices `i = 20...27`, `j = 0`, `k = 0` at voxel size `0.05` (`0x3d4ccccd`). `min_extent` is 1 in this fixture because the column is one cell thick. The library default, and the snapshot goldens, use 2.

### Performance budget

The v1 release bar stays: the overlapping 80,000-point room, evidence off, under 100 ms around `compare`.

Evidence adds one acceptance bar, the indoor room at `rayStride` 1, and one printed long-ray benchmark that does not assert.

#### Indoor room (the bar)

An 8 × 8 × 3 m room, sensor at the origin. Voxel size 0.05 m. Inclusive index box: `i, j ∈ [-80, 79]`, `k ∈ [-30, 29]` (cell coverage x, y ∈ [-4, 4) m, z ∈ [-1.5, 1.5) m). A cell is on the shell when `i` is `-80` or `79`, or `j` is `-80` or `79`, or `k` is `-30` or `29`. One return per shell cell, at the cell center:

```text
coordinate = (Float32(index) + Float32(0.5)) * voxelSize
```

One binary32 add and one binary32 multiply. `voxelSize` is the binary32 value 0.05 (`0x3d4ccccd`). The shell is 88,088 points.

Three hollow boxes. A cell of the inclusive index box is kept when any axis sits on the low or high face. No box cell is on the room shell.

| Box | i | j | k | Face cells |
|---|---|---|---|---|
| A | 20...27 | -10...-3 | -8...-1 | 296 |
| B | -30...-23 | 16...23 | -4...3 | 296 |
| C | -6...5 | -28...-17 | 4...13 | 640 |

The before cloud is the shell plus A, B, and C: 89,320 points. The after cloud is those same points with box A translated by +20 cells in `i` (1.00 m; new i is 40...47). B, C, and the shell stay. Both clouds have 89,320 points. That is the ~80k indoor sample: one return per surface voxel, a few boxes, sensor at the center.

Config: voxel size 0.05, `maxRange` nil, evidence on, `rayStride` 1, `tolerance` 1, `minVoxels` 6, `minExtent` 2, `pairMoves` true, origin `(0, 0, 0)`. Warm one untimed `compare`, time the second. Assert elapsed `< 3 s` when `_isDebugAssertConfiguration()` is true and `< 0.1 s` otherwise. The relocated box is a real change, so the timed call clusters. Print the milliseconds.

The shell alone is about 10.5×10⁶ index-Manhattan steps at stride 1. An open-addressed `Int64` probe on every visited cell of that walk measured about 111 ms in release before occupancy is added, which misses a 100 ms `compare`. A dense bit grid of the same walk measured 22 ms release and 230 ms debug. The shipped free-set is that bit grid when the ray index AABB is at most 32 MiB (256×2²⁰ cells): membership of a marked key matches the open-addressed table, and the occupied map stays `Set<Int64>`. Above 32 MiB the free-set is a private open-addressed `Int64` table of the before-only keys a ray actually visits (load factor at most 1/2, hash not seeded per process). Raising `rayStride` is not the fix.

#### Long-ray benchmark (printed, not a gate)

Enumeration of the generator below, at voxel size 0.05 m and radius 299 cells (14.95 m), is the secondary number. One ray per kept voxel. Iterations on a ray are at most the index Manhattan distance, and at most `⌊(15 / 0.05) · √3⌋ = 519`. Stride 1 on this cloud is about 80k rays times a mean Manhattan near 340, on the order of 3×10⁷ steps. Stride 64 is the sparse subsample (about 1.6 percent of voxels) and is not a disappearance claim.

| `rayStride` | Kept rays | Sum of index Manhattans |
|---|---|---|
| 16 | 5,014 | 1.68×10⁶ |
| 32 | 2,586 | 8.74×10⁵ |
| 64 | 1,275 | 4.32×10⁵ |

Generator, outside the timer. The splitmix constants are the ones already in the test runner (`state = state &* 6364136223846793005 &+ 1442695040888963407`), seed `0x15A6`, state held in `UInt64`. Draw until 80,000 distinct in-ball indices exist:

```text
i = Int(x % 599) - 299
j = Int((x >> 21) % 599) - 299
k = Int((x >> 42) % 599) - 299
keep when i*i + j*j + k*k <= 299*299 and the index triple is new
```

Each point is the cell center `(Float32(index) + 0.5) * voxelSize`, one binary32 add and one binary32 multiply, per axis. The after cloud is that list. For the first 8,000 points in generation order, add `Float32(1)` to `x` when the moved point still passes the binary32 `maxRange` test at 15 m; otherwise keep the original point. Config: voxel size 0.05, `maxRange` 15, evidence on, `tolerance` 1, `minVoxels` 6, `minExtent` 2, `pairMoves` true, origin `(0,0,0)`. Run once at `rayStride` 1 and once at `rayStride` 64. Warm one untimed `compare` per stride, time the second. Print both millisecond figures. Do not assert a limit.

### Static-room blips

Measured on the copied rig snapshots with evidence on, tolerance 1, `minVoxels` 1, `minExtent` 1, pairing on, voxel size 0.05 m, origin `(0, 0, 0)`. `minExtent` 1 turns the span gate off so every component is a region. `voxelCount` on a moved region is the appeared side.

| Sensor | Regions | Voxel-count histogram | Pre-cut appeared / disappeared / unknown |
|---|---|---|---|
| JT128 | 50 | 1×44, 2×4, 3×2 | 46 / 14 / 854 |
| Mid-360 | 272 | 1×234, 2×22, 3×10, 4×4, 5×1, 7×1 | 255 / 104 / 1286 |

The JT128 maximum is 3 voxels. One of those two 3-voxel regions is 2×2×2 cells. Everything else on that pair is one cell thick on some axis.

The Mid-360 maximum is one appeared region of 7 voxels, AABB 5×3×1 cells (0.25 m × 0.15 m × 0.05 m). That is the non-repetitive petal: a sheet one cell thick. Ten other Mid-360 regions are at least 2 cells on every axis, and the largest of those has 5 voxels, which `minVoxels` 6 already drops.

So the defaults split the two shapes. `minVoxels` 6 removes the specks, including every static component that is already two cells thick. `minExtent` 2 removes the 7-voxel sheet, and any longer sheet that stays one cell thick. Together both goldens are `regions` empty. The pre-cut counts above stay in the JSON, so the streak is visible as voxels and not as a region.

The runner's extent group builds a 7-voxel 5×3×1 sheet and expects no region at the defaults, and the same sheet back when `minExtent` is 1. It also adds, removes, and moves a 0.10 m shell (8 voxels, 2×2×2): two voxels over the count cut, and exactly the extent floor. The 0.40 m added, removed, and moved groups assert 296 voxels and a 0.40 m edge on each axis, 49× the count cut and 4× the extent cut.

### Snapshot goldens

`change-detect --evidence --json` on the rig snapshots, sensor origin `(0, 0, 0)` because those clouds are already in the sensor frame. Defaults otherwise (voxel 0.05 m, tolerance 1, `minVoxels` 6, `minExtent` 2, `rayStride` 1, pairing on). Both files have an empty `regions` array.

| Before | After | Golden |
|---|---|---|
| `snapshots/a/jt128.bin` | `snapshots/b/jt128.bin` | `Tests/Fixtures/change-detection/jt128.json` |
| `snapshots/a/mid360.bin` | `snapshots/b/mid360.bin` | `Tests/Fixtures/change-detection/mid360.json` |

The four inputs are copied beside the goldens as `a-jt128.bin`, `b-jt128.bin`, `a-mid360.bin`, and `b-mid360.bin`. A golden file is the `json` string plus one `\n`. The test decodes those copies, runs `compare`, and checks byte-identical JSON. No network.

### Tests

Points are cell centers under the formula above. Origin is `(0, 0, 0)` unless a test says otherwise. Evidence on, `rayStride` 1, `tolerance` 0, `minVoxels` 6, `minExtent` 1, voxel size 0.05, `pairMoves` true, `maxRange` nil, unless a test says otherwise. `tolerance` 0 keeps dilation from swallowing the cell next to an occluder. `minExtent` 1 keeps these one-cell columns; the tests are about evidence, and the streak gate is covered by the extent group above. The indoor timing call sets `minExtent` 2.

1. **Occluded back face.** Before centers at `i = 8...15` (front), `i = 16` (occluder), and `i = 20...27` (back), all with `j = 0`, `k = 0`. After is only the occluder. The one ray ends at the center of voxel 16, so the walk marks `i = 0...15` and stops before 16. The front is free: one `.disappeared` region, `voxelCount` 8, AABB of `i = 8...15`. The back is unknown: `unknownVoxels == 8`, and no disappeared region contains those keys. The occluder is in both maps, so `appearedVoxels == 0`. With `reportUnobserved` true, one `.unobserved` region has the AABB of `i = 20...27` and is not `.disappeared`.

2. **Clear line of sight.** Before centers at `i = 20...27` and the backstop `i = 40`, `j = 0`, `k = 0`. After is only the backstop. The ray marks `i = 0...39`. The column is free: one `.disappeared` region, `voxelCount` 8, `unknownVoxels == 0`, `appearedVoxels == 0`. This is the JSON golden above, byte for byte, `reportUnobserved` false.

3. **Diagonal tie.** Origin is the cell center of `(0, 0, 0)`, not the coordinate origin. Before holds centers of `(0,0,0)`, `(1,1,1)`, `(2,2,2)`, the face neighbor `(1,0,0)`, and the endpoint `(3,3,3)`. After is only the endpoint. `minVoxels` is 1 and `reportUnobserved` is true. The three `tMax` values start equal, so each step advances `x`, `y`, and `z` together. The diagonal cells are one `.disappeared` region of `voxelCount` 3. `(1,0,0)` is `unknownVoxels == 1` and one `.unobserved` region. The endpoint is in both maps.

4. **Determinism.** Build the clear-line-of-sight fixture, append a second copy of every point, and set `minPoints` to 1. Shuffle both `positions` arrays with Fisher–Yates driven by the same splitmix generator, seed `0xC0FFEE`. `Report` equality and `json` byte equality both hold against the unshuffled duplicated pair. Repeat with `minPoints` 2 on that duplicated pair: every voxel has two points, occupancy is unchanged, and the JSON matches the duplicated `minPoints` 1 JSON except `config.min_points`. A second shuffle of a paired scene (the v1 box moved 1 m, evidence on, stride 1) likewise matches `Report` and `json` across the permutation. The algorithm has no seed. The seeds only define the permutation.

5. **JSON golden.** The clear-line-of-sight fixture encodes to the literal in the canonical JSON section. Compare the library string, not a re-serialization, and not the human text block.

6. **Performance.** The indoor room and the `< 100 ms` release assertion above. The v1 overlapping-room assertion stays in the suite beside it. The long-ray generator prints stride 1 and stride 64 and does not assert.

7. **Snapshot goldens.** Byte-compare `json` plus `\n` against `jt128.json` and `mid360.json` on the copied `.bin` inputs.

8. **One-cell slide.** Tolerance 1, `minVoxels` 1. A surface that moves from `i = 20` to `i = 19` (toward the sensor) leaves `i = 20` unknown: `unknownVoxels == 1`, `disappearedVoxels == 0`. A surface that moves from `i = 20` to `i = 21` (away from the sensor) is free and then suppressed by dilation: `unknownVoxels == 0`, `disappearedVoxels == 0`, `appearedVoxels == 0`.

### Implementation map

Not part of the contract the port matches. This change touches these files and no others:

| Path | Change |
|---|---|
| `Sources/LabCommandCore/ChangeDetection.swift` | Config fields, evidence carve, v2 order, `minKey`, `json` |
| `Sources/LabCommandCoreTestsRunner/ChangeDetectionTests.swift` | Groups above |
| `Sources/ChangeDetect/main.swift` | Flags, `unknown_voxels` text token, `--json` |
| `README.md` | One clause for `--json` and `--evidence` on the existing CLI line |
| `Tests/Fixtures/change-detection/` | Copied `a`/`b` bins and the two JSON goldens |

No app, Metal, or live-session files. The occupied structure stays `Set<Int64>`. The free-set is the bit grid or the open-addressed table in the budget section. When evidence is on, the after map keeps one representative point per occupied voxel so `diff` can cast the specified rays. That storage is not in the JSON. The representative is the accepted point in the voxel that is minimum under the binary32 order key.
