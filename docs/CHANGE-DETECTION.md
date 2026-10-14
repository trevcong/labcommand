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

Both headers are `00 01 00 00` (CDR LE). `PointCloud2CDR.decode` already accepts that layout. `snapshots/b/jt128.bin` is the same length as `a/` and is not byte-identical. `snapshots/b/mid360.bin` is 519,385 bytes (19,968 points). Same-file against itself is the empty-diff smoke check. On `a/` against `b/` with the defaults below, the JT128 pair prints `regions=0` (46 appeared and 36 disappeared voxels, all under `minVoxels`) and the Mid-360 pair prints 4 regions of 7–22 voxels (255 appeared and 268 disappeared voxels before the cut). That is a few thin wall patches on the sparser sensor, not a room-sized diff, so the defaults stay at tolerance 1 and `minVoxels` 6. `--tolerance 2` (10 cm) clears the Mid-360 pair to `regions=0` when a two-cell slide should not count.

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
        public var pairMoves: Bool           // true
        public var maxMove: Float            // 1.5 m between centroids
        public var moveCountRatio: Float     // 2: larger count / smaller count
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
    }

    public struct Region: Sendable, Equatable {
        public var kind: Kind
        public var voxelCount: Int
        public var centroid: Point3D
        public var boundsMin: Point3D
        public var boundsMax: Point3D
        public var volume: Float
        /// Set only for `.moved`: the disappeared partner.
        public var fromVoxelCount: Int?
        public var fromCentroid: Point3D?
        public var fromBoundsMin: Point3D?
        public var fromBoundsMax: Point3D?
    }

    public struct Report: Sendable, Equatable {
        public var regions: [Region]
        /// Changed voxels after dilation, before the cluster-size cut.
        public var appearedVoxels: Int
        public var disappearedVoxels: Int
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
- `minPoints < 1` or `minVoxels < 1`
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

Drop a component with `voxelCount < minVoxels`. Default 6: a one-to-five-voxel speckle that dilation did not absorb goes away, and the synthetic box below is hundreds of voxels.

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

`Report.appearedVoxels` and `disappearedVoxels` count keys after dilation and before the `minVoxels` cut, so a test can see that a speck existed and was then dropped.

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
              [--tolerance n] [--min-voxels n] [--max-move m]
              [--no-pair] <before.pcd|before.bin> <after.pcd|after.bin>
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
voxel_size_m=0.050  tolerance=1  min_points=1  min_voxels=6  pair_moves=true
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

8. **Threshold.** Room plus box, plus two isolated points that occupy two voxels far from the box. `minVoxels == 6` reports only the box. `minVoxels == 1` reports the box and the speck (`appearedVoxels` includes the speck in both runs).

9. **Mismatch.** `diff` of a 0.05 m map and a 0.10 m map throws `voxelSizeMismatch`. `tolerance == 5` throws `invalidConfig`.

10. **Performance.** 80,000 points on the room-scale surface grid (repeat the grid with a half-cell jitter pattern if the shell is shorter than 80k, still deterministic). The after cloud is the same points plus a seeded ±1 cm shift and an 8,000-point patch relocated by 1 m (a real change, so clustering does work). Time the second `compare`. Assert elapsed `< 3 s` when `_isDebugAssertConfiguration()` is true and `< 0.1 s` otherwise. Print the measured milliseconds in the passing line.

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

- **Sparse real returns.** The noise-only guarantee is for the dense synthetic grid. A Mid-360 or JT128 surface cell often holds one return. Independent dropout then creates disappeared voxels, and dilation only saves them when a neighbor survived. `snapshots/a` against `snapshots/b` (rig unmoved, ~10 s apart) is `regions=0` on the JT128 and 4 regions on the Mid-360 at the defaults. Those leftovers are 7–22 voxel wall patches. Raising `minVoxels` to 24 would hide them and also hide any real change under 24 cells, so the default stays 6. `--tolerance 2` absorbs the patches instead, because the other scan still occupies the surface within 10 cm.
- **Cell faces.** A point parked on a boundary flips cells under a sub-millimeter shift. The synthetic grids are inset from faces. Real clouds will sit on faces; tolerance 1 covers a one-cell flip when the motion is shorter than `voxelSize`. Jitter larger than the voxel (a 1 cm shift in a 5 mm grid) crosses more than one cell and shows up as change. `tolerance` has to be at least `ceil(jitter / voxelSize)`.
- **Rig motion.** If the tripod moves between snapshots, the whole room is a change. This module does not register clouds.
- **Cross-sensor files.** `livox_frame` vs `hesai_lidar` will report a room-sized diff. The CLI does not refuse it.
- **Small objects.** Anything under `minVoxels` cells is dropped on purpose. At 0.05 m and 6 voxels, a cup-sized blob can disappear while a 0.4 m box cannot. Lower `--min-voxels` when looking for small objects, and expect more speckle.
- **Moved false pairs.** Two similar blobs inside `maxMove` with no closer candidate become one `moved` region. The ambiguity gate (the 1.5× second-candidate rule) is what keeps a swap of two identical boxes from being labeled cleanly; that case stays unpaired and should not be asserted as `moved`.
- **Volume is a voxel count.** A one-return wall becomes a 5 cm slab. `volume` is `count * size³`, the occupied cells, not a mesh volume. A `moved` region's `volume` is the appeared side only.
- **Release vs debug.** The 100 ms bar is `-O` only. Judging the budget from `swift run` without `-c release` will fail a correct implementation.
- **Key overflow at tiny voxels.** Below about a millimeter, `VoxelKey.limit` no longer covers a multi-meter room, and those points increment `droppedPoints` silently. The CLI prints `dropped`, which is the visible signal. Default 0.05 m is far from that edge.
- **Hash-set constant factors.** A fully disjoint pair (every voxel changed, then 26 probes each) can miss 100 ms even when the overlapping test passes. The acceptance test is the overlapping 80k pair plus the identical 115k JT128 file. A future worst-case benchmark needs its own, looser number.
