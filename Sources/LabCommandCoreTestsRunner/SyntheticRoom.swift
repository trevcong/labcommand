import Foundation
import LabCommandCore

// MARK: - Synthetic Room

// A deterministic 8 × 8 × 3 m furnished room for the room-memory fixtures (PR 0.3) and the
// snapshot size test (PR 0.2). Geometry is axis-aligned boxes in the room frame (origin at
// a floor corner, z up, meters). A seeded scanner casts rays from the sensor and returns the
// first hit with uniform range noise. Everything is a pure function of its inputs and seed.

/// An axis-aligned solid box in the room frame.
struct SyntheticBox {
    var min: SIMD3<Double>
    var max: SIMD3<Double>
    var intensity: Float

    /// A box standing on the floor, centered at `(x, y)`.
    static func onFloor(x: Double, y: Double, size: SIMD3<Double>, intensity: Float) -> SyntheticBox {
        SyntheticBox(
            min: SIMD3(x - size.x / 2, y - size.y / 2, 0),
            max: SIMD3(x + size.x / 2, y + size.y / 2, size.z),
            intensity: intensity
        )
    }

    var center: SIMD3<Double> { (min + max) / 2 }

    func contains(_ point: SIMD3<Double>, margin: Double) -> Bool {
        all(point .>= min - margin) && all(point .<= max + margin)
    }

    /// Distance along the ray to the box's entry face, or nil when the ray misses or starts inside.
    func entry(origin: SIMD3<Double>, direction: SIMD3<Double>) -> Double? {
        var near = -Double.infinity
        var far = Double.infinity
        for axis in 0..<3 {
            if direction[axis] == 0 {
                if origin[axis] < min[axis] || origin[axis] > max[axis] { return nil }
                continue
            }
            let t1 = (min[axis] - origin[axis]) / direction[axis]
            let t2 = (max[axis] - origin[axis]) / direction[axis]
            near = Swift.max(near, Swift.min(t1, t2))
            far = Swift.min(far, Swift.max(t1, t2))
        }
        return near > 0 && far >= near ? near : nil
    }
}

struct SyntheticRoom {
    static let size = SIMD3<Double>(8, 8, 3)
    static let nominalSensor = SIMD3<Double>(4.0, 4.0, 1.2)
    static let shellIntensity: Float = 0.3

    static let furniture: [SyntheticBox] = [
        SyntheticBox(min: SIMD3(0.3, 6.9, 0), max: SIMD3(1.9, 7.7, 0.75), intensity: 0.6),   // desk
        SyntheticBox(min: SIMD3(7.5, 1.0, 0), max: SIMD3(7.9, 3.0, 2.0), intensity: 0.6),    // shelf
        SyntheticBox(min: SIMD3(0.2, 0.2, 0), max: SIMD3(0.8, 1.4, 1.1), intensity: 0.6),    // cabinet
        SyntheticBox(min: SIMD3(3.0, 5.8, 0), max: SIMD3(4.6, 6.6, 0.72), intensity: 0.6),   // table
        SyntheticBox(min: SIMD3(5.0, 7.1, 0), max: SIMD3(7.0, 7.9, 0.85), intensity: 0.6),   // couch
    ]

    /// The 25 cm box, 1.8 m from the sensor (brief PR 2.3: ≥ 5 cm at 1.5–2 m).
    static let boxA = SyntheticBox.onFloor(x: 5.8, y: 4.0, size: SIMD3(repeating: 0.25), intensity: 0.9)
    /// The same box moved 1.0 m in +y (inside the 0.5–1.5 m moved band).
    static let boxB = SyntheticBox.onFloor(x: 5.8, y: 5.0, size: SIMD3(repeating: 0.25), intensity: 0.9)

    /// A 0.45 × 0.3 × 1.75 m person crossing in +x, 0.3 m per frame (1.2 m/s at 4 Hz).
    static func person(step: Int) -> SyntheticBox {
        SyntheticBox.onFloor(x: 3.1 + 0.3 * Double(step), y: 2.8, size: SIMD3(0.45, 0.3, 1.75), intensity: 0.5)
    }

    var objects: [SyntheticBox]
    var sensor: SIMD3<Double>

    init(extra: [SyntheticBox] = [], sensor: SIMD3<Double> = SyntheticRoom.nominalSensor) {
        objects = SyntheticRoom.furniture + extra
        self.sensor = sensor
    }

    /// First hit along a unit-length ray from the sensor: walls, floor, ceiling, or an object.
    func cast(_ direction: SIMD3<Double>) -> (range: Double, intensity: Float) {
        var range = Double.infinity
        for axis in 0..<3 where direction[axis] != 0 {
            let wall = direction[axis] > 0 ? SyntheticRoom.size[axis] : 0
            range = min(range, (wall - sensor[axis]) / direction[axis])
        }
        var intensity = SyntheticRoom.shellIntensity
        for object in objects {
            if let entry = object.entry(origin: sensor, direction: direction), entry < range {
                range = entry
                intensity = object.intensity
            }
        }
        return (range, intensity)
    }
}

// MARK: - Scanner

/// SplitMix64: small, fast, and identical on every platform.
struct SplitMix64 {
    private var state: UInt64

    init(seed: UInt64) {
        state = seed
    }

    mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)` with 53 bits.
    mutating func unit() -> Double {
        Double(next() >> 11) / Double(1 << 53)
    }
}

/// A repetitive azimuth × elevation grid, like a spinning lidar, with uniform range noise.
struct SyntheticScanner {
    var azimuthStepDeg: Double
    var elevationMinDeg: Double
    var elevationMaxDeg: Double
    var elevationStepDeg: Double
    var rangeNoiseM: Double

    /// The point-cloud fixtures: 288 × 81 rays, ±1 cm range noise. 1.25° is about 4.4 cm at 2 m.
    static let fixture = SyntheticScanner(
        azimuthStepDeg: 1.25, elevationMinDeg: -50, elevationMaxDeg: 50, elevationStepDeg: 1.25, rangeNoiseM: 0.01
    )
    /// An accumulated memory for the size test: 720 × 360 rays over the whole sphere.
    static let dense = SyntheticScanner(
        azimuthStepDeg: 0.5, elevationMinDeg: -89.75, elevationMaxDeg: 89.75, elevationStepDeg: 0.5, rangeNoiseM: 0.01
    )

    var azimuthCount: Int { Int((360 / azimuthStepDeg).rounded()) }
    var elevationCount: Int { Int(((elevationMaxDeg - elevationMinDeg) / elevationStepDeg).rounded()) + 1 }

    /// Hits in the room frame, elevation-major. The noise stream depends only on `seed` and ray order.
    func scan(_ room: SyntheticRoom, seed: UInt64) -> [(point: SIMD3<Double>, intensity: Float)] {
        var random = SplitMix64(seed: seed)
        var hits: [(point: SIMD3<Double>, intensity: Float)] = []
        hits.reserveCapacity(azimuthCount * elevationCount)
        for row in 0..<elevationCount {
            let elevation = (elevationMinDeg + Double(row) * elevationStepDeg) * .pi / 180
            for column in 0..<azimuthCount {
                let azimuth = Double(column) * azimuthStepDeg * .pi / 180
                let direction = SIMD3(cos(elevation) * cos(azimuth), cos(elevation) * sin(azimuth), sin(elevation))
                let hit = room.cast(direction)
                let range = hit.range + (random.unit() * 2 - 1) * rangeNoiseM
                hits.append((room.sensor + direction * range, hit.intensity))
            }
        }
        return hits
    }
}

// MARK: - Evidence Grid

/// Occupied / free / unknown voxels from ray casting, over a brick-aligned box around the room.
/// The hot loops use scalars: SIMD subscripting is slow in debug builds, where the tests run.
struct EvidenceGrid {
    static let voxelSize = 0.05
    /// Voxel index of the grid's first cell on each axis: one brick below the room.
    static let low = -RoomMemory.Brick.dim
    /// Cells per axis: the room (160 × 160 × 60 voxels) plus a brick of margin on each side.
    static let dimX = 176, dimY = 176, dimZ = 72

    private static let unknown: UInt8 = 0
    private static let free: UInt8 = 1
    private static let occupied: UInt8 = 2

    private var states = [UInt8](repeating: EvidenceGrid.unknown, count: dimX * dimY * dimZ)

    private static func index(_ coordinate: Double) -> Int {
        Int((coordinate / voxelSize).rounded(.down))
    }

    private static func cell(_ x: Int, _ y: Int, _ z: Int) -> Int? {
        let lx = x - low, ly = y - low, lz = z - low
        guard lx >= 0, ly >= 0, lz >= 0, lx < dimX, ly < dimY, lz < dimZ else { return nil }
        return (lx * dimY + ly) * dimZ + lz
    }

    /// Marks every voxel a ray passes through free (unless occupied) and its end voxel occupied.
    /// Amanatides–Woo traversal, so the result does not depend on ray order.
    mutating func integrate(from origin: SIMD3<Double>, to end: SIMD3<Double>) {
        let o = [origin.x, origin.y, origin.z]
        let d = [end.x - origin.x, end.y - origin.y, end.z - origin.z]
        var current = [EvidenceGrid.index(origin.x), EvidenceGrid.index(origin.y), EvidenceGrid.index(origin.z)]
        let target = [EvidenceGrid.index(end.x), EvidenceGrid.index(end.y), EvidenceGrid.index(end.z)]
        var step = [0, 0, 0]
        var tMax = [Double.infinity, .infinity, .infinity]
        var tDelta = [Double.infinity, .infinity, .infinity]
        for axis in 0..<3 where d[axis] != 0 {
            step[axis] = d[axis] > 0 ? 1 : -1
            let boundary = Double(current[axis] + (d[axis] > 0 ? 1 : 0)) * EvidenceGrid.voxelSize
            tMax[axis] = (boundary - o[axis]) / d[axis]
            tDelta[axis] = EvidenceGrid.voxelSize / abs(d[axis])
        }
        let limit = abs(target[0] - current[0]) + abs(target[1] - current[1]) + abs(target[2] - current[2])
        for _ in 0..<limit {
            if current == target { break }
            if let cell = EvidenceGrid.cell(current[0], current[1], current[2]), states[cell] == EvidenceGrid.unknown {
                states[cell] = EvidenceGrid.free
            }
            let axis = tMax[0] < tMax[1] ? (tMax[0] < tMax[2] ? 0 : 2) : (tMax[1] < tMax[2] ? 1 : 2)
            current[axis] += step[axis]
            tMax[axis] += tDelta[axis]
        }
        if let cell = EvidenceGrid.cell(target[0], target[1], target[2]) {
            states[cell] = EvidenceGrid.occupied
        }
    }

    /// Every brick with at least one known voxel, in canonical (i, j, k) order.
    func bricks() -> [RoomMemory.Brick] {
        let dim = RoomMemory.Brick.dim
        var bricks: [RoomMemory.Brick] = []
        for bi in 0..<EvidenceGrid.dimX / dim {
            for bj in 0..<EvidenceGrid.dimY / dim {
                for bk in 0..<EvidenceGrid.dimZ / dim {
                    var occupied = [UInt8](repeating: 0, count: RoomMemory.Brick.maskBytes)
                    var known = occupied
                    for x in 0..<dim {
                        for y in 0..<dim {
                            let row = ((bi * dim + x) * EvidenceGrid.dimY + bj * dim + y) * EvidenceGrid.dimZ + bk * dim
                            for z in 0..<dim {
                                let state = states[row + z]
                                guard state != EvidenceGrid.unknown else { continue }
                                let position = RoomMemory.Brick.bitPosition(x: x, y: y, z: z)
                                known[position.byte] |= position.mask
                                if state == EvidenceGrid.occupied { occupied[position.byte] |= position.mask }
                            }
                        }
                    }
                    guard known.contains(where: { $0 != 0 }) else { continue }
                    // low is exactly one brick, so grid brick b is room brick b - 1.
                    bricks.append(RoomMemory.Brick(
                        coordinate: RoomMemory.BrickCoordinate(
                            i: Int16(bi + EvidenceGrid.low / dim),
                            j: Int16(bj + EvidenceGrid.low / dim),
                            k: Int16(bk + EvidenceGrid.low / dim)
                        ),
                        occupied: occupied,
                        known: known
                    ))
                }
            }
        }
        return bricks
    }

    /// Integrates one scan of `room` taken by `scanner`, with the room placed at `offset` in the
    /// grid frame. Zero puts the floor and the x = 0 / y = 0 walls exactly on brick boundaries.
    static func scanned(
        _ room: SyntheticRoom,
        scanner: SyntheticScanner,
        seed: UInt64,
        offset: SIMD3<Double> = .zero
    ) -> EvidenceGrid {
        var grid = EvidenceGrid()
        for hit in scanner.scan(room, seed: seed) {
            grid.integrate(from: room.sensor + offset, to: hit.point + offset)
        }
        return grid
    }
}
