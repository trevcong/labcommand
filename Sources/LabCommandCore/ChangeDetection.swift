// MARK: - Voxel key

/// Packs a 3D voxel index into one `Int64` so occupancy can be a `Set`.
///
/// Valid indices are `-limit ..< limit` on each axis (`limit` is 2²⁰). At the
/// default 0.05 m voxel that covers ±52,428.8 m, which holds a lab cloud. Bit 63
/// stays clear.
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
    /// `q = coordinate * inverseVoxelSize`, then truncates toward zero and steps
    /// negative non-integers down by one so the cell that straddles the origin
    /// stays one voxel wide. `(-voxelSize, 0)` is index `-1`.
    ///
    /// - Parameters:
    ///   - coordinate: Position along one axis, in meters.
    ///   - inverseVoxelSize: `1 / voxelSize`, in voxels per meter.
    /// - Returns: The voxel index. `coordinate * inverseVoxelSize` must be finite
    ///   and inside `±limit`; occupancy checks that before calling.
    @inline(__always)
    public static func index(_ coordinate: Float, inverseVoxelSize: Float) -> Int {
        let q = coordinate * inverseVoxelSize
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
/// disappearance with an appearance as one moved object. `Region.volume` is the
/// occupied-cell volume `voxelCount * voxelSize³` in cubic meters, not a mesh volume.
public enum ChangeDetection: Sendable {
    /// Knobs for occupancy, dilation, clustering, and move pairing.
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

        /// Defaults: 0.05 m voxels, no range clip, `minPoints` 1, `tolerance` 1 voxel,
        /// `minVoxels` 6, move pairing on, `maxMove` 1.5 m, `moveCountRatio` 2.
        public init() {
            voxelSize = 0.05
            maxRange = nil
            minPoints = 1
            tolerance = 1
            minVoxels = 6
            pairMoves = true
            maxMove = 1.5
            moveCountRatio = 2
        }
    }

    /// Occupied voxels of one frame. The original points are not kept.
    public struct OccupancyMap: Sendable, Equatable {
        /// Voxel edge length, in meters, used to build `occupied`.
        public var voxelSize: Float
        /// Packed indices of voxels that passed `minPoints`.
        public var occupied: Set<Int64>
        /// Points that were finite, inside `maxRange`, and inside the key limits.
        public var acceptedPoints: Int
        /// Points skipped for a non-finite coordinate, a range clip, or a key overflow.
        public var droppedPoints: Int
    }

    /// How a region relates the after cloud to the before cloud.
    public enum Kind: String, Sendable, Equatable {
        /// Occupied in the after cloud and not explained by the before cloud.
        case appeared
        /// Occupied in the before cloud and not explained by the after cloud.
        case disappeared
        /// An appeared cluster paired with a disappeared cluster of similar size.
        case moved
    }

    /// One changed object after clustering and optional move pairing.
    public struct Region: Sendable, Equatable {
        /// Appeared, disappeared, or moved.
        public var kind: Kind
        /// Occupied voxels in the region. For `.moved`, the appeared component.
        public var voxelCount: Int
        /// Mean of the occupied cell centers, in meters. For `.moved`, the appeared pose.
        public var centroid: Point3D
        /// Inclusive corner of the lowest occupied cell, in meters.
        public var boundsMin: Point3D
        /// Exclusive corner of the highest occupied cell, in meters.
        public var boundsMax: Point3D
        /// Occupied volume in cubic meters: `voxelCount * voxelSize³`.
        /// A moved region's volume is the appeared side only, so one object is not counted twice.
        public var volume: Float
        /// Voxel count of the disappeared partner. Set only for `.moved`.
        public var fromVoxelCount: Int?
        /// Centroid of the disappeared partner, in meters. Set only for `.moved`.
        public var fromCentroid: Point3D?
        /// Inclusive low corner of the disappeared partner, in meters. Set only for `.moved`.
        public var fromBoundsMin: Point3D?
        /// Exclusive high corner of the disappeared partner, in meters. Set only for `.moved`.
        public var fromBoundsMax: Point3D?
    }

    /// Diff of two occupancy maps after dilation, clustering, and pairing.
    public struct Report: Sendable, Equatable {
        /// Changed regions, ordered by kind, then size, then centroid.
        public var regions: [Region]
        /// Appeared voxels after dilation and before the `minVoxels` cut.
        public var appearedVoxels: Int
        /// Disappeared voxels after dilation and before the `minVoxels` cut.
        public var disappearedVoxels: Int
        /// Occupied voxels in the before map.
        public var beforeVoxels: Int
        /// Occupied voxels in the after map.
        public var afterVoxels: Int
    }

    /// A rejected config, or two maps that were not built on the same voxel size.
    public enum Failure: Error, Sendable, Equatable {
        /// `voxelSize`, `maxRange`, `minPoints`, `minVoxels`, `tolerance`, `maxMove`, or `moveCountRatio` is out of range.
        case invalidConfig(String)
        /// The two maps, or a map and `config.voxelSize`, are not the same edge length in meters.
        case voxelSizeMismatch(Float, Float)
    }

    /// Builds an occupancy map from one frame.
    ///
    /// One pass over `frame.positions`. A point is dropped when any coordinate is
    /// non-finite, when it lies outside `maxRange` of the origin, or when its
    /// quantized index does not fit in `VoxelKey`. Centroids of a later diff come
    /// from voxel centers, so the map does not keep the points.
    ///
    /// - Parameters:
    ///   - frame: Cloud already in the coordinates to diff. Intensity and sensor are ignored.
    ///   - config: Voxel size in meters, optional range clip, and `minPoints`.
    /// - Returns: The occupied set plus accepted and dropped point counts.
    /// - Throws: `Failure.invalidConfig` when `config` is out of range.
    public static func occupancyMap(
        _ frame: PointCloudFrame,
        config: Config = Config()
    ) throws -> OccupancyMap {
        try validate(config)
        let inverse = 1 / config.voxelSize
        let maxRange2 = config.maxRange.map { $0 * $0 }
        let built: (occupied: Set<Int64>, dropped: Int)
        if config.minPoints == 1 {
            built = occupySingle(frame.positions, inverse: inverse, maxRange2: maxRange2)
        } else {
            built = occupyThreshold(
                frame.positions,
                inverse: inverse,
                maxRange2: maxRange2,
                minPoints: config.minPoints
            )
        }
        return OccupancyMap(
            voxelSize: config.voxelSize,
            occupied: built.occupied,
            acceptedPoints: frame.count - built.dropped,
            droppedPoints: built.dropped
        )
    }

    /// Diffs two maps built with the same voxel size.
    ///
    /// A voxel that is only in one map still counts as unchanged when the other
    /// map occupies something within `tolerance` voxels (Chebyshev). Surviving
    /// voxels are clustered with 26-connectivity. Components smaller than
    /// `minVoxels` are dropped. When `pairMoves` is set, a mutually nearest
    /// appeared/disappeared pair of similar size becomes one `.moved` region.
    ///
    /// - Parameters:
    ///   - before: Occupancy of the earlier cloud.
    ///   - after: Occupancy of the later cloud.
    ///   - config: Tolerance, cluster cut, and move-pairing gates. `voxelSize` must match both maps.
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

        let onlyAfter = subtracting(after.occupied, before.occupied)
        let onlyBefore = subtracting(before.occupied, after.occupied)
        let appeared = surviving(onlyAfter, against: before.occupied, tolerance: config.tolerance)
        let disappeared = surviving(onlyBefore, against: after.occupied, tolerance: config.tolerance)

        let appearedClusters = clusters(appeared, voxelSize: config.voxelSize, minVoxels: config.minVoxels)
        let disappearedClusters = clusters(disappeared, voxelSize: config.voxelSize, minVoxels: config.minVoxels)
        let regions = assemble(
            appeared: appearedClusters,
            disappeared: disappearedClusters,
            config: config
        )
        return Report(
            regions: regions,
            appearedVoxels: appeared.count,
            disappearedVoxels: disappeared.count,
            beforeVoxels: before.occupied.count,
            afterVoxels: after.occupied.count
        )
    }

    /// Builds both maps with `config`, then diffs them.
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
        let beforeMap = try occupancyMap(before, config: config)
        let afterMap = try occupancyMap(after, config: config)
        return try diff(beforeMap, afterMap, config: config)
    }

    // MARK: Occupancy

    @inline(__always)
    private static func quantize(_ coordinate: Float, inverse: Float) -> Int? {
        let q = coordinate * inverse
        let limit = Float(VoxelKey.limit)
        guard q.isFinite, abs(q) < limit else { return nil }
        return VoxelKey.index(coordinate, inverseVoxelSize: inverse)
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
        if let maxRange2, x * x + y * y + z * z > maxRange2 {
            return nil
        }
        guard let i = quantize(x, inverse: inverse),
              let j = quantize(y, inverse: inverse),
              let k = quantize(z, inverse: inverse) else {
            return nil
        }
        return VoxelKey.packUnchecked(i: i, j: j, k: k)
    }

    private static func occupySingle(
        _ positions: [Point3D],
        inverse: Float,
        maxRange2: Float?
    ) -> (occupied: Set<Int64>, dropped: Int) {
        var occupied = Set<Int64>(minimumCapacity: positions.count)
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
                occupied.insert(key)
            }
        }
        return (occupied, dropped)
    }

    private static func occupyThreshold(
        _ positions: [Point3D],
        inverse: Float,
        maxRange2: Float?,
        minPoints: Int
    ) -> (occupied: Set<Int64>, dropped: Int) {
        var counts = Dictionary<Int64, Int32>(minimumCapacity: positions.count)
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
                counts[key] = (counts[key] ?? 0) &+ 1
            }
        }
        var occupied = Set<Int64>(minimumCapacity: counts.count)
        for (key, count) in counts where Int(count) >= minPoints {
            occupied.insert(key)
        }
        return (occupied, dropped)
    }

    // MARK: Diff

    private static func subtracting(_ source: Set<Int64>, _ other: Set<Int64>) -> Set<Int64> {
        var result = Set<Int64>()
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

    // MARK: Clusters

    private struct Cluster {
        var voxelCount: Int
        var centroid: Point3D
        var boundsMin: Point3D
        var boundsMax: Point3D
        var volume: Float
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

            while let current = stack.popLast() {
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
                x: Float((sumI / n + 0.5) * size),
                y: Float((sumJ / n + 0.5) * size),
                z: Float((sumK / n + 0.5) * size)
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
            let volume = Float(count) * voxelSize * voxelSize * voxelSize
            found.append(Cluster(
                voxelCount: count,
                centroid: centroid,
                boundsMin: boundsMin,
                boundsMax: boundsMax,
                volume: volume
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
        config: Config
    ) -> [Region] {
        var regions: [Region] = []
        regions.reserveCapacity(appeared.count + disappeared.count)
        if !config.pairMoves || appeared.isEmpty || disappeared.isEmpty {
            for cluster in appeared {
                regions.append(region(cluster, kind: .appeared))
            }
            for cluster in disappeared {
                regions.append(region(cluster, kind: .disappeared))
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
        regions.sort(by: comesBefore)
        return regions
    }

    /// Mutual nearest neighbors that also clear the distance, count-ratio, and ambiguity gates.
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
            if lhs.distance != rhs.distance { return lhs.distance < rhs.distance }
            let a = appeared[lhs.appeared].centroid
            let b = appeared[rhs.appeared].centroid
            if a.x != b.x { return a.x < b.x }
            if a.y != b.y { return a.y < b.y }
            return a.z < b.z
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
    private static func ambiguous(
        _ candidates: [(index: Int, distance: Float)],
        partner: Int,
        distance: Float
    ) -> Bool {
        let limit = distance * 1.5
        for candidate in candidates where candidate.index != partner {
            if candidate.distance < limit { return true }
        }
        return false
    }

    private static func centroidDistance(_ a: Point3D, _ b: Point3D) -> Float {
        let dx = a.x - b.x
        let dy = a.y - b.y
        let dz = a.z - b.z
        return (dx * dx + dy * dy + dz * dz).squareRoot()
    }

    private static func region(_ cluster: Cluster, kind: Kind) -> Region {
        Region(
            kind: kind,
            voxelCount: cluster.voxelCount,
            centroid: cluster.centroid,
            boundsMin: cluster.boundsMin,
            boundsMax: cluster.boundsMax,
            volume: cluster.volume,
            fromVoxelCount: nil,
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
            fromVoxelCount: disappeared.voxelCount,
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
        }
    }

    private static func comesBefore(_ a: Region, _ b: Region) -> Bool {
        let rankA = kindRank(a.kind)
        let rankB = kindRank(b.kind)
        if rankA != rankB { return rankA < rankB }
        if a.voxelCount != b.voxelCount { return a.voxelCount > b.voxelCount }
        if a.centroid.x != b.centroid.x { return a.centroid.x < b.centroid.x }
        if a.centroid.y != b.centroid.y { return a.centroid.y < b.centroid.y }
        if a.centroid.z != b.centroid.z { return a.centroid.z < b.centroid.z }
        let ax = a.fromCentroid?.x
        let bx = b.fromCentroid?.x
        if ax != bx { return (ax ?? -.greatestFiniteMagnitude) < (bx ?? -.greatestFiniteMagnitude) }
        let ay = a.fromCentroid?.y
        let by = b.fromCentroid?.y
        if ay != by { return (ay ?? -.greatestFiniteMagnitude) < (by ?? -.greatestFiniteMagnitude) }
        let az = a.fromCentroid?.z
        let bz = b.fromCentroid?.z
        return (az ?? -.greatestFiniteMagnitude) < (bz ?? -.greatestFiniteMagnitude)
    }

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
    }
}
