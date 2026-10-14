// MARK: - Voxel key

/// Packs a 3D voxel index into one `Int64` so occupancy can be a `Set`.
///
/// Valid indices are `-limit ..< limit` on each axis (`limit` is 2²⁰). At the
/// default 0.05 m voxel that covers ±52,428.8 m, which holds a lab cloud. Bit 63
/// stays clear, so numeric order of keys is lexicographic order of `(i, j, k)`.
public enum VoxelKey: Sendable {
    /// Half-extent of a packed index, in voxels. Valid indices are `-limit ..< limit`.
    public static let limit = 1_048_576

    /// Packs `(i, j, k)` into one key.
    ///
    /// - Parameters:
    ///   - i: Voxel index along x. Valid range is `-limit ..< limit`.
    ///   - j: Voxel index along y. Same range.
    ///   - k: Voxel index along z. Same range.
    /// - Returns: The packed key, or nil when any index is outside range.
    public static func pack(i: Int, j: Int, k: Int) -> Int64? {
        guard (-limit..<limit).contains(i),
              (-limit..<limit).contains(j),
              (-limit..<limit).contains(k) else {
            return nil
        }
        return packUnchecked(i: i, j: j, k: k)
    }

    /// Unpacks a key produced by `pack`. Keys from outside that function are not meaningful.
    ///
    /// - Parameter key: A packed voxel index.
    /// - Returns: The three voxel indices.
    public static func unpack(_ key: Int64) -> (i: Int, j: Int, k: Int) {
        let bits = UInt64(bitPattern: key)
        let mask: UInt64 = (1 << 21) - 1
        let w = Int(bits & mask)
        let v = Int((bits >> 21) & mask)
        let u = Int((bits >> 42) & mask)
        return (u - limit, v - limit, w - limit)
    }

    /// Quantizes one coordinate to a voxel index.
    ///
    /// `q = coordinate * inverseVoxelSize` in binary32, then truncates toward zero
    /// and steps negative non-integers down by one. The boundary plane belongs to
    /// the cell that has that plane as its lower face. `(-voxelSize, 0)` is index `-1`.
    /// `-0` is index `0`.
    ///
    /// - Parameters:
    ///   - coordinate: Position along one axis, in meters.
    ///   - inverseVoxelSize: `Float(1) / voxelSize`, in voxels per meter. One binary32 division.
    /// - Returns: The voxel index. `coordinate * inverseVoxelSize` must be finite
    ///   and inside `±limit`; occupancy checks that before calling.
    @inline(__always)
    public static func index(_ coordinate: Float, inverseVoxelSize: Float) -> Int {
        floorIndex(coordinate * inverseVoxelSize)
    }

    /// Floor of a quantizer product. `q` is already `coordinate * inverseVoxelSize`.
    @inline(__always)
    static func floorIndex(_ q: Float) -> Int {
        var i = Int(q)
        if q < 0, Float(i) != q {
            i -= 1
        }
        return i
    }

    @inline(__always)
    static func packUnchecked(i: Int, j: Int, k: Int) -> Int64 {
        let u = UInt64(i + limit)
        let v = UInt64(j + limit)
        let w = UInt64(k + limit)
        // 21-bit lanes. Bit 63 stays clear: the high lane ends at bit 62.
        return Int64(bitPattern: (u << 42) | (v << 21) | w)
    }
}

// MARK: - Change detection

/// Occupancy diff between two point clouds already in the same coordinates.
///
/// Builds a voxel set per cloud, drops changes that are only a slide of
/// `tolerance` voxels, clusters what remains, and optionally pairs a
/// disappearance with an appearance as one moved object. With evidence of
/// absence on, a before-voxel is disappeared only when an after-ray traverses
/// it. `Region.volume` is the occupied-cell volume `voxelCount * voxelSize³`
/// in cubic meters, not a mesh volume.
public enum ChangeDetection: Sendable {
    /// Knobs for occupancy, dilation, clustering, move pairing, and free-space evidence.
    /// Distances are meters. `tolerance` and `minVoxels` are voxel counts.
    public struct Config: Sendable, Equatable {
        /// Edge length of one voxel, in meters. Default 0.05.
        public var voxelSize: Float
        /// Drops points whose distance from the origin exceeds this, in meters.
        /// Nil keeps every finite in-range point.
        public var maxRange: Float?
        /// A voxel is occupied only when at least this many accepted points land in it.
        public var minPoints: Int
        /// Chebyshev radius, in voxels, used to suppress a one-cell sensor slide.
        /// `0` is a raw set difference. Values above 4 are rejected.
        public var tolerance: Int
        /// Drops a connected component smaller than this many voxels.
        public var minVoxels: Int
        /// When true, pairs an appeared cluster with a disappeared cluster into one moved region.
        public var pairMoves: Bool
        /// Largest centroid gap, in meters, that can still be one moved object.
        public var maxMove: Float
        /// Largest allowed `max(count) / min(count)` between a moved pair. Dimensionless.
        public var moveCountRatio: Float
        /// When true, disappeared voxels are the before-occupied cells an after-ray
        /// traverses. When false, disappearance is the v1 set difference. Default false.
        public var evidenceOfAbsence: Bool
        /// Sensor origin of the after frame, in meters, in the same coordinates as the points.
        /// Default `(0, 0, 0)`.
        public var origin: Point3D
        /// Power of two from 1 through 64. A kept occupied after voxel casts one ray.
        /// Default 1 casts every occupied voxel.
        public var rayStride: Int
        /// When true, and evidence is on, unknown before-occupied cells are clustered
        /// as `.unobserved` regions. The unknown count is reported either way.
        public var reportUnobserved: Bool

        /// Defaults: 0.05 m voxels, no range clip, `minPoints` 1, `tolerance` 1 voxel,
        /// `minVoxels` 6, move pairing on, `maxMove` 1.5 m, `moveCountRatio` 2,
        /// evidence off, origin at the coordinate origin, `rayStride` 1.
        public init() {
            voxelSize = 0.05
            maxRange = nil
            minPoints = 1
            tolerance = 1
            minVoxels = 6
            pairMoves = true
            maxMove = 1.5
            moveCountRatio = 2
            evidenceOfAbsence = false
            origin = Point3D(x: 0, y: 0, z: 0)
            rayStride = 1
            reportUnobserved = false
        }
    }

    /// Occupied voxels of one frame. The original cloud is not kept.
    ///
    /// When the map is built with evidence on, one representative return is
    /// retained per occupied voxel so a later `diff` can cast rays. That
    /// storage is internal and is not part of the JSON record.
    public struct OccupancyMap: Sendable, Equatable {
        /// Voxel edge length, in meters, used to build `occupied`.
        public var voxelSize: Float
        /// Packed indices of voxels that passed `minPoints`.
        public var occupied: Set<Int64>
        /// Points that were finite, inside `maxRange`, and inside the key limits.
        public var acceptedPoints: Int
        /// Points skipped for a non-finite coordinate, a range clip, or a key overflow.
        public var droppedPoints: Int
        /// Minimum accepted return in each occupied voxel, under the binary32 order
        /// key on `x`, then `y`, then `z`. Empty when evidence was off at build time.
        var endpoints: [Int64: Point3D]

        public static func == (lhs: OccupancyMap, rhs: OccupancyMap) -> Bool {
            lhs.voxelSize == rhs.voxelSize
                && lhs.occupied == rhs.occupied
                && lhs.acceptedPoints == rhs.acceptedPoints
                && lhs.droppedPoints == rhs.droppedPoints
                && lhs.endpoints == rhs.endpoints
        }
    }

    /// How a region relates the after cloud to the before cloud.
    public enum Kind: String, Sendable, Equatable {
        /// Occupied in the after cloud and not explained by the before cloud.
        case appeared
        /// Occupied in the before cloud, seen free by an after-ray (or absent, when
        /// evidence is off), and not explained by a neighbor within `tolerance`.
        case disappeared
        /// An appeared cluster paired with a disappeared cluster of similar size.
        case moved
        /// Before-occupied, not occupied after, and not traversed by an after-ray.
        /// Not a change, and not a move candidate.
        case unobserved
    }

    /// One changed object after clustering and optional move pairing.
    public struct Region: Sendable, Equatable {
        /// Appeared, disappeared, moved, or unobserved.
        public var kind: Kind
        /// Occupied voxels in the region. For `.moved`, the appeared component.
        public var voxelCount: Int
        /// Mean of the occupied cell centers, in meters. For `.moved`, the appeared pose.
        public var centroid: Point3D
        /// Inclusive corner of the lowest occupied cell, in meters.
        public var boundsMin: Point3D
        /// Exclusive corner of the highest occupied cell, in meters.
        public var boundsMax: Point3D
        /// Occupied volume in cubic meters: `((count * voxelSize) * voxelSize) * voxelSize`.
        /// A moved region's volume is the appeared side only, so one object is not counted twice.
        public var volume: Float
        /// Minimum packed key of this component. For `.moved`, the appeared component.
        public var minKey: Int64
        /// Voxel count of the disappeared partner. Set only for `.moved`.
        public var fromVoxelCount: Int?
        /// Minimum packed key of the disappeared partner. Set only for `.moved`.
        public var fromMinKey: Int64?
        /// Centroid of the disappeared partner, in meters. Set only for `.moved`.
        public var fromCentroid: Point3D?
        /// Inclusive low corner of the disappeared partner, in meters. Set only for `.moved`.
        public var fromBoundsMin: Point3D?
        /// Exclusive high corner of the disappeared partner, in meters. Set only for `.moved`.
        public var fromBoundsMax: Point3D?
    }

    /// Diff of two occupancy maps after dilation, clustering, and pairing.
    public struct Report: Sendable, Equatable {
        /// Regions in v2 order: kind (`moved`, `appeared`, `disappeared`, `unobserved`),
        /// then `voxelCount` descending, then `minKey` ascending, then `fromMinKey`.
        public var regions: [Region]
        /// Appeared voxels after dilation and before the `minVoxels` cut.
        public var appearedVoxels: Int
        /// Disappeared voxels after dilation and before the `minVoxels` cut.
        public var disappearedVoxels: Int
        /// Before-occupied voxels that evidence left unknown, before the `minVoxels` cut.
        /// Zero when evidence is off. Dilation does not move a voxel into or out of this count.
        public var unknownVoxels: Int
        /// Occupied voxels in the before map.
        public var beforeVoxels: Int
        /// Occupied voxels in the after map.
        public var afterVoxels: Int
    }

    /// A rejected config, or two maps that were not built on the same voxel size.
    public enum Failure: Error, Sendable, Equatable {
        /// A config field is out of range, or evidence is on without ray endpoints.
        case invalidConfig(String)
        /// The two maps, or a map and `config.voxelSize`, are not the same edge length in meters.
        case voxelSizeMismatch(Float, Float)
    }

    /// Builds an occupancy map from one frame.
    ///
    /// One pass over `frame.positions`. A point is dropped when any coordinate is
    /// non-finite, when its binary32 squared range exceeds `maxRange²`, or when
    /// its quantized index does not fit in `VoxelKey`. When evidence is on, the
    /// map keeps the minimum accepted return in each occupied voxel.
    ///
    /// - Parameters:
    ///   - frame: Cloud already in the coordinates to diff. Intensity and sensor are ignored.
    ///   - config: Voxel size in meters, optional range clip, `minPoints`, and evidence.
    /// - Returns: The occupied set plus accepted and dropped point counts.
    /// - Throws: `Failure.invalidConfig` when `config` is out of range.
    public static func occupancyMap(
        _ frame: PointCloudFrame,
        config: Config = Config()
    ) throws -> OccupancyMap {
        try validate(config)
        let inverse = inverseVoxelSize(config.voxelSize)
        return occupy(frame.positions, config: config, inverse: inverse)
    }

    /// Diffs two maps built with the same voxel size.
    ///
    /// A voxel that is only in one map still counts as unchanged when the other
    /// map occupies something within `tolerance` voxels (Chebyshev). With evidence
    /// on, a before-only voxel is a disappeared candidate only when an after-ray
    /// traverses it; otherwise it counts as unknown. Surviving voxels are clustered
    /// with 26-connectivity. Components smaller than `minVoxels` are dropped.
    /// When `pairMoves` is set, a mutually nearest appeared/disappeared pair of
    /// similar size becomes one `.moved` region. Equal binary32 distances do not pair.
    ///
    /// - Parameters:
    ///   - before: Occupancy of the earlier cloud.
    ///   - after: Occupancy of the later cloud. Evidence rays use its endpoints.
    ///   - config: Tolerance, cluster cut, move-pairing gates, and evidence.
    ///     `voxelSize` must match both maps.
    /// - Returns: The ordered regions and the pre-cut changed-voxel counts.
    /// - Throws: `Failure.invalidConfig`, or `Failure.voxelSizeMismatch` when the
    ///   maps and `config.voxelSize` are not exactly the same edge length.
    public static func diff(
        _ before: OccupancyMap,
        _ after: OccupancyMap,
        config: Config
    ) throws -> Report {
        try validate(config)
        guard before.voxelSize == after.voxelSize, before.voxelSize == config.voxelSize else {
            let other = before.voxelSize == after.voxelSize ? config.voxelSize : after.voxelSize
            throw Failure.voxelSizeMismatch(before.voxelSize, other)
        }
        if config.evidenceOfAbsence, after.endpoints.count != after.occupied.count {
            throw Failure.invalidConfig(
                "evidence requires the after map to be built with evidenceOfAbsence enabled"
            )
        }

        let inverse = inverseVoxelSize(config.voxelSize)
        let onlyAfter = subtracting(after.occupied, before.occupied)
        let onlyBefore = subtracting(before.occupied, after.occupied)
        let appeared = surviving(onlyAfter, against: before.occupied, tolerance: config.tolerance)

        let carved = carve(onlyBefore: onlyBefore, after: after, config: config, inverse: inverse)
        let disappeared = surviving(carved.free, against: after.occupied, tolerance: config.tolerance)

        let appearedClusters = clusters(appeared, voxelSize: config.voxelSize, minVoxels: config.minVoxels)
        let disappearedClusters = clusters(disappeared, voxelSize: config.voxelSize, minVoxels: config.minVoxels)
        let unobservedClusters: [Cluster]
        if config.evidenceOfAbsence, config.reportUnobserved {
            unobservedClusters = clusters(carved.unknown, voxelSize: config.voxelSize, minVoxels: config.minVoxels)
        } else {
            unobservedClusters = []
        }
        let regions = assemble(
            appeared: appearedClusters,
            disappeared: disappearedClusters,
            unobserved: unobservedClusters,
            config: config
        )
        return Report(
            regions: regions,
            appearedVoxels: appeared.count,
            disappearedVoxels: disappeared.count,
            unknownVoxels: carved.unknownCount,
            beforeVoxels: before.occupied.count,
            afterVoxels: after.occupied.count
        )
    }

    /// Builds both maps with `config`, then diffs them.
    ///
    /// The binary32 reciprocal `Float(1) / voxelSize` is formed once per map build
    /// and once for the rays. Each is the same division, so both maps and every
    /// ray share one reciprocal value.
    ///
    /// - Parameters:
    ///   - before: Earlier cloud, already in the coordinates to compare.
    ///   - after: Later cloud, in those same coordinates.
    ///   - config: Passed to both `occupancyMap` and `diff`.
    /// - Returns: The diff report. File IO is the caller's job.
    /// - Throws: `Failure.invalidConfig` when `config` is out of range.
    public static func compare(
        _ before: PointCloudFrame,
        _ after: PointCloudFrame,
        config: Config = Config()
    ) throws -> Report {
        try validate(config)
        let inverse = inverseVoxelSize(config.voxelSize)
        let beforeMap = occupy(before.positions, config: config, inverse: inverse)
        let afterMap = occupy(after.positions, config: config, inverse: inverse)
        return try diff(beforeMap, afterMap, config: config)
    }

    /// Canonical v2 JSON for one report.
    ///
    /// Hand-written so key order and number formatting stay fixed. Binary32 values
    /// are eight lowercase hex digits of their big-endian bit pattern. Packed keys
    /// are decimal integer strings. The result has no leading or trailing whitespace
    /// and no elapsed-time field.
    ///
    /// - Parameters:
    ///   - report: Diff to encode. Region order is kept as given.
    ///   - before: Occupancy of the earlier cloud. `points` is `accepted + dropped`.
    ///   - after: Occupancy of the later cloud, same point rule.
    ///   - config: Recorded beside the report. `maxRange` nil encodes as `null`.
    /// - Returns: One JSON object, UTF-8, schema `labcommand.change-detection.v2`.
    public static func json(
        report: Report,
        before: OccupancyMap,
        after: OccupancyMap,
        config: Config
    ) -> String {
        var text = "{\"schema\":\"labcommand.change-detection.v2\""
        text += ",\"evidence\":\(config.evidenceOfAbsence ? "true" : "false")"
        text += ",\"report_unobserved\":\(config.reportUnobserved ? "true" : "false")"
        text += ",\"origin_bits\":\(jsonPoint(config.origin))"
        text += ",\"ray_stride\":\(config.rayStride)"
        text += ",\"config\":{"
        text += "\"voxel_size_bits\":\(jsonBits(config.voxelSize))"
        if let maxRange = config.maxRange {
            text += ",\"max_range_bits\":\(jsonBits(maxRange))"
        } else {
            text += ",\"max_range_bits\":null"
        }
        text += ",\"min_points\":\(config.minPoints)"
        text += ",\"tolerance\":\(config.tolerance)"
        text += ",\"min_voxels\":\(config.minVoxels)"
        text += ",\"pair_moves\":\(config.pairMoves ? "true" : "false")"
        text += ",\"max_move_bits\":\(jsonBits(config.maxMove))"
        text += ",\"move_count_ratio_bits\":\(jsonBits(config.moveCountRatio))"
        text += "}"
        text += ",\"before\":\(jsonMap(before))"
        text += ",\"after\":\(jsonMap(after))"
        text += ",\"appeared_voxels\":\(report.appearedVoxels)"
        text += ",\"disappeared_voxels\":\(report.disappearedVoxels)"
        text += ",\"unknown_voxels\":\(report.unknownVoxels)"
        text += ",\"regions\":["
        for (index, region) in report.regions.enumerated() {
            if index > 0 { text += "," }
            text += jsonRegion(region)
        }
        text += "]}"
        return text
    }

    // MARK: Occupancy

    @inline(__always)
    private static func inverseVoxelSize(_ voxelSize: Float) -> Float {
        Float(1) / voxelSize
    }

    /// Accepts a point into a packed key, or nil when the point is dropped.
    @inline(__always)
    private static func accept(
        _ point: Point3D,
        inverse: Float,
        maxRange2: Float?
    ) -> Int64? {
        let x = point.x
        let y = point.y
        let z = point.z
        guard x.isFinite, y.isFinite, z.isFinite else { return nil }
        if let maxRange2, (x * x + y * y) + z * z > maxRange2 {
            return nil
        }
        let limit = Float(VoxelKey.limit)
        let qx = x * inverse
        let qy = y * inverse
        let qz = z * inverse
        guard qx.isFinite, qy.isFinite, qz.isFinite,
              abs(qx) < limit, abs(qy) < limit, abs(qz) < limit else {
            return nil
        }
        return VoxelKey.packUnchecked(
            i: VoxelKey.floorIndex(qx),
            j: VoxelKey.floorIndex(qy),
            k: VoxelKey.floorIndex(qz)
        )
    }

    private static func occupy(
        _ positions: [Point3D],
        config: Config,
        inverse: Float
    ) -> OccupancyMap {
        let maxRange2 = config.maxRange.map { $0 * $0 }
        let built: (occupied: Set<Int64>, dropped: Int, endpoints: [Int64: Point3D])
        if config.minPoints == 1 {
            built = occupySingle(
                positions,
                inverse: inverse,
                maxRange2: maxRange2,
                keepEndpoints: config.evidenceOfAbsence
            )
        } else {
            built = occupyThreshold(
                positions,
                inverse: inverse,
                maxRange2: maxRange2,
                minPoints: config.minPoints,
                keepEndpoints: config.evidenceOfAbsence
            )
        }
        return OccupancyMap(
            voxelSize: config.voxelSize,
            occupied: built.occupied,
            acceptedPoints: positions.count - built.dropped,
            droppedPoints: built.dropped,
            endpoints: built.endpoints
        )
    }

    private static func occupySingle(
        _ positions: [Point3D],
        inverse: Float,
        maxRange2: Float?,
        keepEndpoints: Bool
    ) -> (occupied: Set<Int64>, dropped: Int, endpoints: [Int64: Point3D]) {
        var occupied = Set<Int64>(minimumCapacity: positions.count)
        var endpoints: [Int64: Point3D] = [:]
        if keepEndpoints {
            endpoints.reserveCapacity(positions.count)
        }
        var dropped = 0
        positions.withUnsafeBufferPointer { buffer in
            var index = 0
            let count = buffer.count
            while index < count {
                let point = buffer[index]
                index += 1
                guard let key = accept(point, inverse: inverse, maxRange2: maxRange2) else {
                    dropped += 1
                    continue
                }
                if keepEndpoints {
                    if let existing = endpoints[key] {
                        if precedes(point, existing) {
                            endpoints[key] = point
                        }
                    } else {
                        endpoints[key] = point
                    }
                }
                occupied.insert(key)
            }
        }
        return (occupied, dropped, endpoints)
    }

    private struct Bin {
        var count: Int32
        var point: Point3D
    }

    private static func occupyThreshold(
        _ positions: [Point3D],
        inverse: Float,
        maxRange2: Float?,
        minPoints: Int,
        keepEndpoints: Bool
    ) -> (occupied: Set<Int64>, dropped: Int, endpoints: [Int64: Point3D]) {
        var bins = Dictionary<Int64, Bin>(minimumCapacity: positions.count)
        var dropped = 0
        positions.withUnsafeBufferPointer { buffer in
            var index = 0
            let count = buffer.count
            while index < count {
                let point = buffer[index]
                index += 1
                guard let key = accept(point, inverse: inverse, maxRange2: maxRange2) else {
                    dropped += 1
                    continue
                }
                if var bin = bins[key] {
                    bin.count = bin.count &+ 1
                    if precedes(point, bin.point) {
                        bin.point = point
                    }
                    bins[key] = bin
                } else {
                    bins[key] = Bin(count: 1, point: point)
                }
            }
        }
        var occupied = Set<Int64>(minimumCapacity: bins.count)
        var endpoints: [Int64: Point3D] = [:]
        if keepEndpoints {
            endpoints.reserveCapacity(bins.count)
        }
        for (key, bin) in bins where Int(bin.count) >= minPoints {
            occupied.insert(key)
            if keepEndpoints {
                endpoints[key] = bin.point
            }
        }
        return (occupied, dropped, endpoints)
    }

    /// Binary32 total order. Smaller key precedes. `-0` precedes `+0`.
    @inline(__always)
    private static func orderKey(_ value: Float) -> UInt32 {
        let bits = value.bitPattern
        if bits & 0x8000_0000 != 0 {
            return ~bits
        }
        return bits | 0x8000_0000
    }

    @inline(__always)
    private static func precedes(_ a: Point3D, _ b: Point3D) -> Bool {
        let ax = orderKey(a.x)
        let bx = orderKey(b.x)
        if ax != bx { return ax < bx }
        let ay = orderKey(a.y)
        let by = orderKey(b.y)
        if ay != by { return ay < by }
        return orderKey(a.z) < orderKey(b.z)
    }

    // MARK: Diff

    private static func subtracting(_ source: Set<Int64>, _ other: Set<Int64>) -> Set<Int64> {
        var result = Set<Int64>()
        result.reserveCapacity(source.count)
        for key in source where !other.contains(key) {
            result.insert(key)
        }
        return result
    }

    private static func surviving(
        _ candidates: Set<Int64>,
        against other: Set<Int64>,
        tolerance: Int
    ) -> Set<Int64> {
        if tolerance == 0 || candidates.isEmpty {
            return candidates
        }
        var kept = Set<Int64>(minimumCapacity: candidates.count)
        for key in candidates where !suppressed(key, by: other, tolerance: tolerance) {
            kept.insert(key)
        }
        return kept
    }

    /// True when `other` occupies a voxel within Chebyshev `tolerance`, excluding the center.
    /// The center is already absent: the candidate came from the set difference.
    @inline(__always)
    private static func suppressed(_ key: Int64, by other: Set<Int64>, tolerance: Int) -> Bool {
        let (i, j, k) = VoxelKey.unpack(key)
        var di = -tolerance
        while di <= tolerance {
            var dj = -tolerance
            while dj <= tolerance {
                var dk = -tolerance
                let ni = i + di
                let nj = j + dj
                while dk <= tolerance {
                    if di != 0 || dj != 0 || dk != 0 {
                        let nk = k + dk
                        if let neighbor = VoxelKey.pack(i: ni, j: nj, k: nk), other.contains(neighbor) {
                            return true
                        }
                    }
                    dk += 1
                }
                dj += 1
            }
            di += 1
        }
        return false
    }

    // MARK: Evidence

    private struct Carve {
        var free: Set<Int64>
        var unknown: Set<Int64>
        var unknownCount: Int
    }

    /// Classifies `onlyBefore` against after-rays. Evidence off returns the v1 set.
    private static func carve(
        onlyBefore: Set<Int64>,
        after: OccupancyMap,
        config: Config,
        inverse: Float
    ) -> Carve {
        if !config.evidenceOfAbsence {
            return Carve(free: onlyBefore, unknown: [], unknownCount: 0)
        }
        if onlyBefore.isEmpty {
            return Carve(free: [], unknown: [], unknownCount: 0)
        }
        let free = freeKeys(
            candidates: onlyBefore,
            endpoints: after.endpoints,
            origin: config.origin,
            inverse: inverse,
            stride: config.rayStride
        )
        var unknown = Set<Int64>()
        if config.reportUnobserved {
            unknown.reserveCapacity(onlyBefore.count - free.count)
            for key in onlyBefore where !free.contains(key) {
                unknown.insert(key)
            }
        }
        return Carve(free: free, unknown: unknown, unknownCount: onlyBefore.count - free.count)
    }

    private struct RayEnd {
        var point: Point3D
        var i: Int
        var j: Int
        var k: Int
    }

    /// SplitMix64 finalizer. The stride filter and the open-addressed table both use it.
    @inline(__always)
    private static func splitmix64(_ key: Int64) -> UInt64 {
        var z = UInt64(bitPattern: key) &+ 0x9E37_79B9_7F4A_7C15
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        z = z ^ (z >> 31)
        return z
    }

    private static func keepsRay(_ key: Int64, stride: Int) -> Bool {
        if stride == 1 { return true }
        return splitmix64(key) & UInt64(stride - 1) == 0
    }

    private static func freeKeys(
        candidates: Set<Int64>,
        endpoints: [Int64: Point3D],
        origin: Point3D,
        inverse: Float,
        stride: Int
    ) -> Set<Int64> {
        let qx = origin.x * inverse
        let qy = origin.y * inverse
        let qz = origin.z * inverse
        let originI = VoxelKey.floorIndex(qx)
        let originJ = VoxelKey.floorIndex(qy)
        let originK = VoxelKey.floorIndex(qz)

        var rays: [RayEnd] = []
        rays.reserveCapacity(endpoints.count)
        var i0 = originI
        var i1 = originI
        var j0 = originJ
        var j1 = originJ
        var k0 = originK
        var k1 = originK
        for (key, point) in endpoints where keepsRay(key, stride: stride) {
            let (i, j, k) = VoxelKey.unpack(key)
            rays.append(RayEnd(point: point, i: i, j: j, k: k))
            i0 = min(i0, i)
            i1 = max(i1, i)
            j0 = min(j0, j)
            j1 = max(j1, j)
            k0 = min(k0, k)
            k1 = max(k1, k)
        }
        if rays.isEmpty {
            return []
        }

        if var grid = FreeGrid(i0: i0, i1: i1, j0: j0, j1: j1, k0: k0, k1: k1) {
            grid.cast(
                rays: rays,
                origin: (x: qx, y: qy, z: qz),
                originIndex: (i: originI, j: originJ, k: originK),
                inverse: inverse
            )
            var free = Set<Int64>()
            free.reserveCapacity(candidates.count)
            for key in candidates {
                let (i, j, k) = VoxelKey.unpack(key)
                if grid.isMarked(i: i, j: j, k: k) {
                    free.insert(key)
                }
            }
            return free
        }

        var table = CandidateMarks(candidates)
        cast(
            rays: rays,
            origin: (x: qx, y: qy, z: qz),
            originIndex: (i: originI, j: originJ, k: originK),
            inverse: inverse,
            mark: { i, j, k in
                table.markIfCandidate(VoxelKey.packUnchecked(i: i, j: j, k: k))
            }
        )
        return table.markedKeys()
    }

    /// 32 MiB bit grid: 2²⁸ cells. The 15 m benchmark's 599³ index box fits under this.
    private struct FreeGrid {
        static let maxCells = 256 << 20

        let i0: Int
        let j0: Int
        let k0: Int
        let ni: Int
        let nj: Int
        let nk: Int
        var words: [UInt64]

        init?(i0: Int, i1: Int, j0: Int, j1: Int, k0: Int, k1: Int) {
            let ni = i1 - i0 + 1
            let nj = j1 - j0 + 1
            let nk = k1 - k0 + 1
            guard ni > 0, nj > 0, nk > 0 else { return nil }
            let (ij, overflowIJ) = ni.multipliedReportingOverflow(by: nj)
            guard !overflowIJ else { return nil }
            let (cells, overflow) = ij.multipliedReportingOverflow(by: nk)
            guard !overflow, cells > 0, cells <= Self.maxCells else { return nil }
            self.i0 = i0
            self.j0 = j0
            self.k0 = k0
            self.ni = ni
            self.nj = nj
            self.nk = nk
            words = [UInt64](repeating: 0, count: (cells + 63) / 64)
        }

        func isMarked(i: Int, j: Int, k: Int) -> Bool {
            let x = i - i0
            let y = j - j0
            let z = k - k0
            if x < 0 || y < 0 || z < 0 || x >= ni || y >= nj || z >= nk {
                return false
            }
            let idx = (x * nj + y) * nk + z
            let bit = UInt64(1) << UInt64(idx & 63)
            return words[idx >> 6] & bit != 0
        }

        mutating func cast(
            rays: [RayEnd],
            origin: (x: Float, y: Float, z: Float),
            originIndex: (i: Int, j: Int, k: Int),
            inverse: Float
        ) {
            let nj = self.nj
            let nk = self.nk
            let i0 = self.i0
            let j0 = self.j0
            let k0 = self.k0
            words.withUnsafeMutableBufferPointer { buffer in
                guard let base = buffer.baseAddress else { return }
                let qOx = origin.x
                let qOy = origin.y
                let qOz = origin.z
                let startI = originIndex.i
                let startJ = originIndex.j
                let startK = originIndex.k
                let inf = Float.infinity
                for ray in rays {
                    let qEx = ray.point.x * inverse
                    let qEy = ray.point.y * inverse
                    let qEz = ray.point.z * inverse
                    let dirX = qEx - qOx
                    let dirY = qEy - qOy
                    let dirZ = qEz - qOz
                    let endI = ray.i
                    let endJ = ray.j
                    let endK = ray.k
                    if startI == endI, startJ == endJ, startK == endK { continue }
                    let stepX = startI == endI ? 0 : (endI > startI ? 1 : -1)
                    let stepY = startJ == endJ ? 0 : (endJ > startJ ? 1 : -1)
                    let stepZ = startK == endK ? 0 : (endK > startK ? 1 : -1)
                    let tDeltaX: Float = stepX == 0 ? inf : Float(1) / abs(dirX)
                    let tDeltaY: Float = stepY == 0 ? inf : Float(1) / abs(dirY)
                    let tDeltaZ: Float = stepZ == 0 ? inf : Float(1) / abs(dirZ)
                    var tMaxX: Float = inf
                    var tMaxY: Float = inf
                    var tMaxZ: Float = inf
                    if stepX != 0 {
                        let plane = stepX > 0 ? Float(startI + 1) - qOx : Float(startI) - qOx
                        tMaxX = plane / dirX
                        if tMaxX < 0 { tMaxX = 0 }
                    }
                    if stepY != 0 {
                        let plane = stepY > 0 ? Float(startJ + 1) - qOy : Float(startJ) - qOy
                        tMaxY = plane / dirY
                        if tMaxY < 0 { tMaxY = 0 }
                    }
                    if stepZ != 0 {
                        let plane = stepZ > 0 ? Float(startK + 1) - qOz : Float(startK) - qOz
                        tMaxZ = plane / dirZ
                        if tMaxZ < 0 { tMaxZ = 0 }
                    }
                    if tMaxX.isNaN || tMaxY.isNaN || tMaxZ.isNaN
                        || tDeltaX.isNaN || tDeltaY.isNaN || tDeltaZ.isNaN {
                        continue
                    }
                    let cap = abs(endI - startI) + abs(endJ - startJ) + abs(endK - startK)
                    var x = startI
                    var y = startJ
                    var z = startK
                    var steps = 0
                    while steps < cap {
                        if x == endI, y == endJ, z == endK { break }
                        let idx = ((x - i0) * nj + (y - j0)) * nk + (z - k0)
                        base[idx >> 6] |= UInt64(1) << UInt64(idx & 63)
                        var tNext = tMaxX
                        if tMaxY < tNext { tNext = tMaxY }
                        if tMaxZ < tNext { tNext = tMaxZ }
                        if tMaxX == tNext {
                            x += stepX
                            tMaxX += tDeltaX
                        }
                        if tMaxY == tNext {
                            y += stepY
                            tMaxY += tDeltaY
                        }
                        if tMaxZ == tNext {
                            z += stepZ
                            tMaxZ += tDeltaZ
                        }
                        steps += 1
                    }
                }
            }
        }
    }

    /// Open-addressed set of candidate keys. A mark means a ray visited that key.
    /// Empty slots are `-1`. Every packed key is non-negative, so `-1` is free.
    private struct CandidateMarks {
        private var slots: [Int64]
        private var marks: [UInt8]
        private var mask: Int

        init(_ keys: Set<Int64>) {
            var capacity = 16
            let need = max(keys.count, 1) * 2
            while capacity < need {
                capacity &*= 2
            }
            slots = [Int64](repeating: -1, count: capacity)
            marks = [UInt8](repeating: 0, count: capacity)
            mask = capacity - 1
            for key in keys {
                insert(key)
            }
        }

        private func hash(_ key: Int64) -> Int {
            Int(truncatingIfNeeded: splitmix64(key)) & mask
        }

        private mutating func insert(_ key: Int64) {
            var index = hash(key)
            while slots[index] != -1 {
                if slots[index] == key { return }
                index = (index + 1) & mask
            }
            slots[index] = key
        }

        @inline(__always)
        mutating func markIfCandidate(_ key: Int64) {
            var index = hash(key)
            while true {
                let slot = slots[index]
                if slot == -1 { return }
                if slot == key {
                    marks[index] = 1
                    return
                }
                index = (index + 1) & mask
            }
        }

        func markedKeys() -> Set<Int64> {
            var free = Set<Int64>()
            for index in slots.indices where marks[index] != 0 {
                free.insert(slots[index])
            }
            return free
        }
    }

    /// Amanatides–Woo traversal. On a tie every tied axis steps. The endpoint voxel
    /// is not marked. Indices stay inside the AABB of the origin and the end, which
    /// is the bit grid, so the mark does not bounds-check.
    @inline(__always)
    private static func walk(
        origin: (x: Float, y: Float, z: Float),
        originIndex: (i: Int, j: Int, k: Int),
        end: Point3D,
        endIndex: (i: Int, j: Int, k: Int),
        inverse: Float,
        mark: (Int, Int, Int) -> Void
    ) {
        let qOx = origin.x
        let qOy = origin.y
        let qOz = origin.z
        let qEx = end.x * inverse
        let qEy = end.y * inverse
        let qEz = end.z * inverse
        let dirX = qEx - qOx
        let dirY = qEy - qOy
        let dirZ = qEz - qOz
        let startI = originIndex.i
        let startJ = originIndex.j
        let startK = originIndex.k
        let endI = endIndex.i
        let endJ = endIndex.j
        let endK = endIndex.k
        if startI == endI, startJ == endJ, startK == endK { return }

        let stepX = startI == endI ? 0 : (endI > startI ? 1 : -1)
        let stepY = startJ == endJ ? 0 : (endJ > startJ ? 1 : -1)
        let stepZ = startK == endK ? 0 : (endK > startK ? 1 : -1)
        let inf = Float.infinity

        @inline(__always)
        func tDelta(_ step: Int, _ dir: Float) -> Float {
            if step == 0 { return inf }
            return Float(1) / abs(dir)
        }
        @inline(__always)
        func tMax(_ step: Int, _ start: Int, _ qO: Float, _ dir: Float) -> Float {
            if step == 0 { return inf }
            let plane: Float
            if step > 0 {
                plane = Float(start + 1) - qO
            } else {
                plane = Float(start) - qO
            }
            var t = plane / dir
            if t < 0 { t = 0 }
            return t
        }

        var tMaxX = tMax(stepX, startI, qOx, dirX)
        var tMaxY = tMax(stepY, startJ, qOy, dirY)
        var tMaxZ = tMax(stepZ, startK, qOz, dirZ)
        let tDeltaX = tDelta(stepX, dirX)
        let tDeltaY = tDelta(stepY, dirY)
        let tDeltaZ = tDelta(stepZ, dirZ)
        if tMaxX.isNaN || tMaxY.isNaN || tMaxZ.isNaN
            || tDeltaX.isNaN || tDeltaY.isNaN || tDeltaZ.isNaN {
            return
        }

        let cap = abs(endI - startI) + abs(endJ - startJ) + abs(endK - startK)
        var x = startI
        var y = startJ
        var z = startK
        var steps = 0
        while steps < cap {
            if x == endI, y == endJ, z == endK { return }
            mark(x, y, z)
            var tNext = tMaxX
            if tMaxY < tNext { tNext = tMaxY }
            if tMaxZ < tNext { tNext = tMaxZ }
            if tMaxX == tNext {
                x += stepX
                tMaxX += tDeltaX
            }
            if tMaxY == tNext {
                y += stepY
                tMaxY += tDeltaY
            }
            if tMaxZ == tNext {
                z += stepZ
                tMaxZ += tDeltaZ
            }
            steps += 1
        }
    }

    private static func cast(
        rays: [RayEnd],
        origin: (x: Float, y: Float, z: Float),
        originIndex: (i: Int, j: Int, k: Int),
        inverse: Float,
        mark: (Int, Int, Int) -> Void
    ) {
        for ray in rays {
            walk(
                origin: origin,
                originIndex: originIndex,
                end: ray.point,
                endIndex: (ray.i, ray.j, ray.k),
                inverse: inverse,
                mark: mark
            )
        }
    }

    // MARK: Clusters

    private struct Cluster {
        var voxelCount: Int
        var centroid: Point3D
        var boundsMin: Point3D
        var boundsMax: Point3D
        var volume: Float
        var minKey: Int64
    }

    private static func clusters(_ keys: Set<Int64>, voxelSize: Float, minVoxels: Int) -> [Cluster] {
        if keys.isEmpty { return [] }
        let seeds = Array(keys)
        var visited = Set<Int64>(minimumCapacity: keys.count)
        var stack: [Int64] = []
        stack.reserveCapacity(256)
        var found: [Cluster] = []

        for seed in seeds where visited.insert(seed).inserted {
            stack.append(seed)
            var count = 0
            var sumI = 0.0
            var sumJ = 0.0
            var sumK = 0.0
            var iMin = Int.max
            var iMax = Int.min
            var jMin = Int.max
            var jMax = Int.min
            var kMin = Int.max
            var kMax = Int.min
            var minKey = seed

            while let current = stack.popLast() {
                if current < minKey { minKey = current }
                let (i, j, k) = VoxelKey.unpack(current)
                count += 1
                sumI += Double(i)
                sumJ += Double(j)
                sumK += Double(k)
                iMin = min(iMin, i)
                iMax = max(iMax, i)
                jMin = min(jMin, j)
                jMax = max(jMax, j)
                kMin = min(kMin, k)
                kMax = max(kMax, k)

                var di = -1
                while di <= 1 {
                    var dj = -1
                    let ni = i + di
                    while dj <= 1 {
                        var dk = -1
                        let nj = j + dj
                        while dk <= 1 {
                            if di != 0 || dj != 0 || dk != 0 {
                                let nk = k + dk
                                if let neighbor = VoxelKey.pack(i: ni, j: nj, k: nk),
                                   keys.contains(neighbor),
                                   visited.insert(neighbor).inserted {
                                    stack.append(neighbor)
                                }
                            }
                            dk += 1
                        }
                        dj += 1
                    }
                    di += 1
                }
            }

            guard count >= minVoxels else { continue }
            let n = Double(count)
            let size = Double(voxelSize)
            let centroid = Point3D(
                x: Float(((sumI / n) + 0.5) * size),
                y: Float(((sumJ / n) + 0.5) * size),
                z: Float(((sumK / n) + 0.5) * size)
            )
            let boundsMin = Point3D(
                x: Float(iMin) * voxelSize,
                y: Float(jMin) * voxelSize,
                z: Float(kMin) * voxelSize
            )
            let boundsMax = Point3D(
                x: Float(iMax + 1) * voxelSize,
                y: Float(jMax + 1) * voxelSize,
                z: Float(kMax + 1) * voxelSize
            )
            let countF = Float(count)
            let volume = ((countF * voxelSize) * voxelSize) * voxelSize
            found.append(Cluster(
                voxelCount: count,
                centroid: centroid,
                boundsMin: boundsMin,
                boundsMax: boundsMax,
                volume: volume,
                minKey: minKey
            ))
        }
        return found
    }

    // MARK: Pairing and order

    private struct Pair {
        var appeared: Int
        var disappeared: Int
        var distance: Float
    }

    private static func assemble(
        appeared: [Cluster],
        disappeared: [Cluster],
        unobserved: [Cluster],
        config: Config
    ) -> [Region] {
        var regions: [Region] = []
        regions.reserveCapacity(appeared.count + disappeared.count + unobserved.count)
        let pair = config.pairMoves && !appeared.isEmpty && !disappeared.isEmpty
        if !pair {
            for cluster in appeared {
                regions.append(region(cluster, kind: .appeared))
            }
            for cluster in disappeared {
                regions.append(region(cluster, kind: .disappeared))
            }
            for cluster in unobserved {
                regions.append(region(cluster, kind: .unobserved))
            }
            regions.sort(by: comesBefore)
            return regions
        }

        var usedAppeared = Set<Int>()
        var usedDisappeared = Set<Int>()
        for pair in mutualPairs(appeared: appeared, disappeared: disappeared, config: config) {
            if usedAppeared.contains(pair.appeared) || usedDisappeared.contains(pair.disappeared) {
                continue
            }
            usedAppeared.insert(pair.appeared)
            usedDisappeared.insert(pair.disappeared)
            regions.append(moved(appeared[pair.appeared], from: disappeared[pair.disappeared]))
        }
        for index in appeared.indices where !usedAppeared.contains(index) {
            regions.append(region(appeared[index], kind: .appeared))
        }
        for index in disappeared.indices where !usedDisappeared.contains(index) {
            regions.append(region(disappeared[index], kind: .disappeared))
        }
        for cluster in unobserved {
            regions.append(region(cluster, kind: .unobserved))
        }
        regions.sort(by: comesBefore)
        return regions
    }

    /// Mutual nearest neighbors that also clear the distance, count-ratio, and ambiguity gates.
    /// An equal binary32 distance is not a nearest neighbor.
    private static func mutualPairs(
        appeared: [Cluster],
        disappeared: [Cluster],
        config: Config
    ) -> [Pair] {
        var appearedCandidates = Array(repeating: [(index: Int, distance: Float)](), count: appeared.count)
        var disappearedCandidates = Array(repeating: [(index: Int, distance: Float)](), count: disappeared.count)

        for aIndex in appeared.indices {
            for dIndex in disappeared.indices {
                guard let distance = gatedDistance(appeared[aIndex], disappeared[dIndex], config: config) else {
                    continue
                }
                appearedCandidates[aIndex].append((dIndex, distance))
                disappearedCandidates[dIndex].append((aIndex, distance))
            }
        }

        var pairs: [Pair] = []
        for aIndex in appeared.indices {
            guard let nearest = uniqueNearest(appearedCandidates[aIndex]) else { continue }
            guard let back = uniqueNearest(disappearedCandidates[nearest.index]), back.index == aIndex else {
                continue
            }
            if ambiguous(appearedCandidates[aIndex], partner: nearest.index, distance: nearest.distance) {
                continue
            }
            if ambiguous(disappearedCandidates[nearest.index], partner: aIndex, distance: nearest.distance) {
                continue
            }
            pairs.append(Pair(appeared: aIndex, disappeared: nearest.index, distance: nearest.distance))
        }

        pairs.sort { lhs, rhs in
            if lhs.distance < rhs.distance { return true }
            if rhs.distance < lhs.distance { return false }
            let leftKey = appeared[lhs.appeared].minKey
            let rightKey = appeared[rhs.appeared].minKey
            if leftKey != rightKey { return leftKey < rightKey }
            return disappeared[lhs.disappeared].minKey < disappeared[rhs.disappeared].minKey
        }
        return pairs
    }

    private static func gatedDistance(_ appeared: Cluster, _ disappeared: Cluster, config: Config) -> Float? {
        let distance = centroidDistance(appeared.centroid, disappeared.centroid)
        guard distance <= config.maxMove else { return nil }
        let hi = Float(max(appeared.voxelCount, disappeared.voxelCount))
        let lo = Float(min(appeared.voxelCount, disappeared.voxelCount))
        guard hi / lo <= config.moveCountRatio else { return nil }
        return distance
    }

    /// The minimum-distance candidate, or nil when that distance is tied.
    private static func uniqueNearest(
        _ candidates: [(index: Int, distance: Float)]
    ) -> (index: Int, distance: Float)? {
        guard var best = candidates.first else { return nil }
        var unique = true
        for candidate in candidates.dropFirst() {
            if candidate.distance < best.distance {
                best = candidate
                unique = true
            } else if candidate.distance == best.distance {
                unique = false
            }
        }
        return unique ? best : nil
    }

    /// Another gated candidate sits closer than `1.5 ×` the mutual distance.
    /// A candidate at exactly `1.5 ×` stays inside the gate.
    private static func ambiguous(
        _ candidates: [(index: Int, distance: Float)],
        partner: Int,
        distance: Float
    ) -> Bool {
        let limit = distance * Float(1.5)
        for candidate in candidates where candidate.index != partner {
            if candidate.distance < limit { return true }
        }
        return false
    }

    private static func centroidDistance(_ a: Point3D, _ b: Point3D) -> Float {
        let dx = a.x - b.x
        let dy = a.y - b.y
        let dz = a.z - b.z
        return ((dx * dx + dy * dy) + dz * dz).squareRoot()
    }

    private static func region(_ cluster: Cluster, kind: Kind) -> Region {
        Region(
            kind: kind,
            voxelCount: cluster.voxelCount,
            centroid: cluster.centroid,
            boundsMin: cluster.boundsMin,
            boundsMax: cluster.boundsMax,
            volume: cluster.volume,
            minKey: cluster.minKey,
            fromVoxelCount: nil,
            fromMinKey: nil,
            fromCentroid: nil,
            fromBoundsMin: nil,
            fromBoundsMax: nil
        )
    }

    private static func moved(_ appeared: Cluster, from disappeared: Cluster) -> Region {
        Region(
            kind: .moved,
            voxelCount: appeared.voxelCount,
            centroid: appeared.centroid,
            boundsMin: appeared.boundsMin,
            boundsMax: appeared.boundsMax,
            volume: appeared.volume,
            minKey: appeared.minKey,
            fromVoxelCount: disappeared.voxelCount,
            fromMinKey: disappeared.minKey,
            fromCentroid: disappeared.centroid,
            fromBoundsMin: disappeared.boundsMin,
            fromBoundsMax: disappeared.boundsMax
        )
    }

    private static func kindRank(_ kind: Kind) -> Int {
        switch kind {
        case .moved: return 0
        case .appeared: return 1
        case .disappeared: return 2
        case .unobserved: return 3
        }
    }

    private static func comesBefore(_ a: Region, _ b: Region) -> Bool {
        let rankA = kindRank(a.kind)
        let rankB = kindRank(b.kind)
        if rankA != rankB { return rankA < rankB }
        if a.voxelCount != b.voxelCount { return a.voxelCount > b.voxelCount }
        if a.minKey != b.minKey { return a.minKey < b.minKey }
        switch (a.fromMinKey, b.fromMinKey) {
        case (nil, nil):
            return false
        case (nil, .some):
            return true
        case (.some, nil):
            return false
        case let (left?, right?):
            return left < right
        }
    }

    // MARK: JSON

    private static func jsonBits(_ value: Float) -> String {
        "\"\(String(format: "%08x", value.bitPattern))\""
    }

    private static func jsonPoint(_ point: Point3D) -> String {
        "[\(jsonBits(point.x)),\(jsonBits(point.y)),\(jsonBits(point.z))]"
    }

    private static func jsonMap(_ map: OccupancyMap) -> String {
        let points = map.acceptedPoints + map.droppedPoints
        return "{\"points\":\(points),\"accepted\":\(map.acceptedPoints),\"dropped\":\(map.droppedPoints),\"voxels\":\(map.occupied.count)}"
    }

    private static func jsonRegion(_ region: Region) -> String {
        var text = "{\"kind\":\"\(region.kind.rawValue)\""
        text += ",\"voxel_count\":\(region.voxelCount)"
        text += ",\"min_key\":\"\(region.minKey)\""
        text += ",\"centroid_bits\":\(jsonPoint(region.centroid))"
        text += ",\"bounds_min_bits\":\(jsonPoint(region.boundsMin))"
        text += ",\"bounds_max_bits\":\(jsonPoint(region.boundsMax))"
        text += ",\"volume_bits\":\(jsonBits(region.volume))"
        if region.kind == .moved,
           let fromCount = region.fromVoxelCount,
           let fromKey = region.fromMinKey,
           let fromCentroid = region.fromCentroid,
           let fromMin = region.fromBoundsMin,
           let fromMax = region.fromBoundsMax {
            text += ",\"from_voxel_count\":\(fromCount)"
            text += ",\"from_min_key\":\"\(fromKey)\""
            text += ",\"from_centroid_bits\":\(jsonPoint(fromCentroid))"
            text += ",\"from_bounds_min_bits\":\(jsonPoint(fromMin))"
            text += ",\"from_bounds_max_bits\":\(jsonPoint(fromMax))"
        }
        text += "}"
        return text
    }

    // MARK: Validate

    private static func validate(_ config: Config) throws {
        if !config.voxelSize.isFinite || config.voxelSize <= 0 {
            throw Failure.invalidConfig("voxelSize must be finite and > 0 m")
        }
        if let maxRange = config.maxRange, !maxRange.isFinite || maxRange <= 0 {
            throw Failure.invalidConfig("maxRange must be finite and > 0 m")
        }
        if config.minPoints < 1 {
            throw Failure.invalidConfig("minPoints must be >= 1")
        }
        if config.minVoxels < 1 {
            throw Failure.invalidConfig("minVoxels must be >= 1")
        }
        if !(0...4).contains(config.tolerance) {
            throw Failure.invalidConfig("tolerance must be in 0...4 voxels")
        }
        if !config.maxMove.isFinite || config.maxMove < 0 {
            throw Failure.invalidConfig("maxMove must be finite and >= 0 m")
        }
        if !config.moveCountRatio.isFinite || config.moveCountRatio < 1 {
            throw Failure.invalidConfig("moveCountRatio must be finite and >= 1")
        }
        switch config.rayStride {
        case 1, 2, 4, 8, 16, 32, 64:
            break
        default:
            throw Failure.invalidConfig("rayStride must be a power of two from 1 through 64")
        }
        let origin = config.origin
        if !origin.x.isFinite || !origin.y.isFinite || !origin.z.isFinite {
            throw Failure.invalidConfig("origin must be finite")
        }
        if config.evidenceOfAbsence {
            let inverse = inverseVoxelSize(config.voxelSize)
            let limit = Float(VoxelKey.limit)
            let qx = origin.x * inverse
            let qy = origin.y * inverse
            let qz = origin.z * inverse
            if !qx.isFinite || !qy.isFinite || !qz.isFinite
                || abs(qx) >= limit || abs(qy) >= limit || abs(qz) >= limit {
                throw Failure.invalidConfig("origin is outside the voxel grid")
            }
        }
    }
}
