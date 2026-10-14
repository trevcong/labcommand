import Foundation
import LabCommandCore

// MARK: - Scenes

private let gridStep: Float = 0.02
private let gridPhase: Float = 0.007

private struct SplitMix {
    var state: UInt64

    mutating func unit() -> Float {
        state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
        return Float(state >> 40) / Float(1 << 24)
    }
}

/// Samples in `[low + phase, high)` on the 0.02 m lattice, off cell faces.
private func samples(from low: Float, to high: Float) -> [Float] {
    var values: [Float] = []
    var i = 0
    while true {
        let value = low + gridPhase + Float(i) * gridStep
        if value >= high { break }
        values.append(value)
        i += 1
    }
    return values
}

/// Floor and four walls, about 4 m × 4 m × 2 m. No ceiling.
private func roomShell() -> [Point3D] {
    let x0: Float = -1.5, x1: Float = 2.5
    let y0: Float = -1.5, y1: Float = 2.5
    let z0: Float = -0.7, z1: Float = 1.3
    let xs = samples(from: x0, to: x1)
    let ys = samples(from: y0, to: y1)
    let zs = samples(from: z0, to: z1)
    var points: [Point3D] = []
    points.reserveCapacity(xs.count * ys.count + 2 * (xs.count + ys.count) * zs.count)
    let floorZ = z0 + gridPhase
    for x in xs {
        for y in ys {
            points.append(Point3D(x: x, y: y, z: floorZ))
        }
    }
    let xLo = x0 + gridPhase
    let xHi = x1 - gridPhase
    for y in ys {
        for z in zs {
            points.append(Point3D(x: xLo, y: y, z: z))
            points.append(Point3D(x: xHi, y: y, z: z))
        }
    }
    let yLo = y0 + gridPhase
    let yHi = y1 - gridPhase
    for x in xs {
        for z in zs {
            points.append(Point3D(x: x, y: yLo, z: z))
            points.append(Point3D(x: x, y: yHi, z: z))
        }
    }
    return points
}

/// Hollow 0.40 m box. Face samples sit just inside the geometric face so they
/// are not on a voxel boundary.
private func boxShell(center: Point3D, side: Float = 0.40) -> [Point3D] {
    let half = side * 0.5
    let x0 = center.x - half, x1 = center.x + half
    let y0 = center.y - half, y1 = center.y + half
    let z0 = center.z - half, z1 = center.z + half
    let xs = samples(from: x0, to: x1)
    let ys = samples(from: y0, to: y1)
    let zs = samples(from: z0, to: z1)
    var points: [Point3D] = []
    let xLo = x0 + gridPhase
    let xHi = x1 - gridPhase
    for y in ys {
        for z in zs {
            points.append(Point3D(x: xLo, y: y, z: z))
            points.append(Point3D(x: xHi, y: y, z: z))
        }
    }
    let yLo = y0 + gridPhase
    let yHi = y1 - gridPhase
    for x in xs {
        for z in zs {
            points.append(Point3D(x: x, y: yLo, z: z))
            points.append(Point3D(x: x, y: yHi, z: z))
        }
    }
    let zLo = z0 + gridPhase
    let zHi = z1 - gridPhase
    for x in xs {
        for y in ys {
            points.append(Point3D(x: x, y: y, z: zLo))
            points.append(Point3D(x: x, y: y, z: zHi))
        }
    }
    return points
}

private func frame(_ points: [Point3D]) -> PointCloudFrame {
    PointCloudFrame(positions: points)
}

private func must<T>(_ body: () throws -> T, _ message: String) -> T {
    do {
        return try body()
    } catch {
        fputs("FAIL: \(message): \(error)\n", stderr)
        exit(1)
    }
}

private func near(_ value: Float, _ expected: Float, _ tolerance: Float) -> Bool {
    abs(value - expected) <= tolerance
}

private func expectBox(
    _ region: ChangeDetection.Region,
    kind: ChangeDetection.Kind,
    center: Point3D,
    label: String
) {
    expect(region.kind == kind, "\(label) kind \(region.kind)")
    expect(region.voxelCount > 6, "\(label) voxelCount \(region.voxelCount)")
    let halfVoxel: Float = 0.025
    expect(near(region.centroid.x, center.x, halfVoxel), "\(label) centroid x \(region.centroid.x)")
    expect(near(region.centroid.y, center.y, halfVoxel), "\(label) centroid y \(region.centroid.y)")
    expect(near(region.centroid.z, center.z, halfVoxel), "\(label) centroid z \(region.centroid.z)")
    let volume = Float(region.voxelCount) * 0.05 * 0.05 * 0.05
    expect(region.volume == volume, "\(label) volume \(region.volume) vs \(volume)")
    let half: Float = 0.20
    expect(region.boundsMin.x <= center.x - half + 0.001, "\(label) boundsMin.x \(region.boundsMin.x)")
    expect(region.boundsMax.x >= center.x + half - 0.001, "\(label) boundsMax.x \(region.boundsMax.x)")
    expect(region.boundsMin.y <= center.y - half + 0.001, "\(label) boundsMin.y \(region.boundsMin.y)")
    expect(region.boundsMax.y >= center.y + half - 0.001, "\(label) boundsMax.y \(region.boundsMax.y)")
    expect(region.boundsMin.z <= center.z - half + 0.001, "\(label) boundsMin.z \(region.boundsMin.z)")
    expect(region.boundsMax.z >= center.z + half - 0.001, "\(label) boundsMax.z \(region.boundsMax.z)")
    expect(region.boundsMax.x - region.boundsMin.x < 0.70, "\(label) x span reaches the walls \(region.boundsMax.x - region.boundsMin.x)")
    expect(region.boundsMax.y - region.boundsMin.y < 0.70, "\(label) y span reaches the walls")
    expect(region.boundsMax.z - region.boundsMin.z < 0.70, "\(label) z span reaches the walls")
}

// MARK: - Groups

func testChangeDetectionKeyMath() {
    let limit = VoxelKey.limit
    let samples = [
        (0, 0, 0),
        (1, -2, 3),
        (limit - 1, 0, -limit),
        (-limit, limit - 1, 4),
        (limit - 1, limit - 1, limit - 1),
        (-limit, -limit, -limit),
    ]
    for (i, j, k) in samples {
        guard let key = VoxelKey.pack(i: i, j: j, k: k) else {
            expect(false, "pack(\(i), \(j), \(k))")
            return
        }
        expect(key >= 0, "bit 63 stays clear for \(i), \(j), \(k)")
        let unpacked = VoxelKey.unpack(key)
        expect(unpacked.i == i && unpacked.j == j && unpacked.k == k,
               "round trip \(i), \(j), \(k) -> \(unpacked)")
    }
    expect(VoxelKey.pack(i: limit, j: 0, k: 0) == nil, "i == limit is outside")
    expect(VoxelKey.pack(i: -limit - 1, j: 0, k: 0) == nil, "i == -limit - 1 is outside")
    expect(VoxelKey.pack(i: 0, j: limit, k: 0) == nil, "j == limit is outside")
    expect(VoxelKey.pack(i: 0, j: 0, k: -limit - 1) == nil, "k == -limit - 1 is outside")

    let inverseFine = 1 / Float(0.05)
    let inverseCoarse = 1 / Float(0.10)
    expect(VoxelKey.index(-0.01, inverseVoxelSize: inverseFine) == -1, "(-0.05, 0) is index -1")
    expect(VoxelKey.index(0.06, inverseVoxelSize: inverseFine) == 1, "0.06 m at 0.05 m is index 1")
    expect(VoxelKey.index(0.06, inverseVoxelSize: inverseCoarse) == 0, "0.06 m at 0.10 m is index 0")

    let finite = Point3D(x: 0.06, y: -0.01, z: 0)
    let cloud = frame([
        Point3D(x: .nan, y: 0, z: 0),
        finite,
        Point3D(x: 0, y: .infinity, z: 1),
        Point3D(x: 60_000, y: 0, z: 0),
    ])
    let map = must({ try ChangeDetection.occupancyMap(cloud) }, "non-finite occupancy")
    expect(map.droppedPoints == 3, "non-finite and out-of-limit points drop, got \(map.droppedPoints)")
    expect(map.acceptedPoints == 1, "one accepted point")
    let i = VoxelKey.index(finite.x, inverseVoxelSize: inverseFine)
    let j = VoxelKey.index(finite.y, inverseVoxelSize: inverseFine)
    let k = VoxelKey.index(finite.z, inverseVoxelSize: inverseFine)
    guard let key = VoxelKey.pack(i: i, j: j, k: k) else {
        expect(false, "finite point packs")
        return
    }
    expect(map.occupied == Set([key]), "the finite point occupies the recomputed key")
}

func testChangeDetectionVoxelSize() {
    let room = frame(roomShell())
    var fine = ChangeDetection.Config()
    fine.voxelSize = 0.05
    var coarse = ChangeDetection.Config()
    coarse.voxelSize = 0.10
    let first = must({ try ChangeDetection.occupancyMap(room, config: fine) }, "fine map")
    let second = must({ try ChangeDetection.occupancyMap(room, config: fine) }, "fine map again")
    let wide = must({ try ChangeDetection.occupancyMap(room, config: coarse) }, "coarse map")
    expect(first.occupied == second.occupied, "0.05 m occupancy is stable")
    expect(wide.occupied.count < first.occupied.count,
           "0.10 m occupies fewer keys (\(wide.occupied.count) vs \(first.occupied.count))")
    let wideAgain = must({ try ChangeDetection.occupancyMap(room, config: coarse) }, "coarse map again")
    expect(wide.occupied == wideAgain.occupied, "0.10 m occupancy is stable")
}

func testChangeDetectionRangeAndMinPoints() {
    let inside = Point3D(x: 14.999, y: 0, z: 0)
    let outside = Point3D(x: 15.001, y: 0, z: 0)
    var clipped = ChangeDetection.Config()
    clipped.maxRange = 15
    let limited = must({ try ChangeDetection.occupancyMap(frame([inside, outside]), config: clipped) }, "range clip")
    expect(limited.droppedPoints == 1 && limited.acceptedPoints == 1, "just over 15 m drops, just under stays")
    expect(limited.occupied.count == 1, "the kept point occupies one voxel")
    let open = must({ try ChangeDetection.occupancyMap(frame([inside, outside])) }, "no range clip")
    expect(open.droppedPoints == 0 && open.occupied.count == 2, "nil maxRange keeps both")

    var sparse = ChangeDetection.Config()
    sparse.minPoints = 2
    let piled = frame([
        Point3D(x: 0.01, y: 0.01, z: 0.01),
        Point3D(x: 0.02, y: 0.01, z: 0.01),
        Point3D(x: 0.01, y: 0.02, z: 0.01),
        Point3D(x: 0.01, y: 0.01, z: 0.02),
        Point3D(x: 0.06, y: 0.01, z: 0.01),
    ])
    let kept = must({ try ChangeDetection.occupancyMap(piled, config: sparse) }, "minPoints")
    expect(kept.occupied.count == 1, "four points in one cell pass minPoints 2; the neighbor does not")
    expect(kept.droppedPoints == 0 && kept.acceptedPoints == 5, "minPoints filters voxels, not the point tally")

    var broken = ChangeDetection.Config()
    broken.voxelSize = 0
    do {
        _ = try ChangeDetection.occupancyMap(frame([]), config: broken)
        expect(false, "voxelSize 0 should throw")
    } catch ChangeDetection.Failure.invalidConfig {
    } catch {
        expect(false, "voxelSize 0 threw \(error)")
    }
}

func testChangeDetectionBoxAdded() {
    let room = roomShell()
    let center = Point3D(x: 1, y: 0.5, z: 0.30)
    let report = must({
        try ChangeDetection.compare(frame(room), frame(room + boxShell(center: center)))
    }, "box added")
    expect(report.regions.count == 1, "one region, got \(report.regions.count)")
    guard let region = report.regions.first else { return }
    expectBox(region, kind: .appeared, center: center, label: "added")
    expect(report.disappearedVoxels == 0, "the room cancels")
    expect(report.appearedVoxels == region.voxelCount, "the box is the whole appeared set")
    expect(region.fromCentroid == nil && region.fromVoxelCount == nil, "an appearance has no from-pose")
}

func testChangeDetectionBoxRemoved() {
    let room = roomShell()
    let center = Point3D(x: 1, y: 0.5, z: 0.30)
    let report = must({
        try ChangeDetection.compare(frame(room + boxShell(center: center)), frame(room))
    }, "box removed")
    expect(report.regions.count == 1, "one region, got \(report.regions.count)")
    guard let region = report.regions.first else { return }
    expectBox(region, kind: .disappeared, center: center, label: "removed")
    expect(report.appearedVoxels == 0, "nothing appeared")
    expect(report.disappearedVoxels == region.voxelCount, "the box is the whole disappeared set")
}

func testChangeDetectionBoxMoved() {
    let room = roomShell()
    let beforeCenter = Point3D(x: 0, y: 0.5, z: 0.30)
    let afterCenter = Point3D(x: 1, y: 0.5, z: 0.30)
    let beforeBox = boxShell(center: beforeCenter)
    let afterBox = beforeBox.map { Point3D(x: $0.x + 1, y: $0.y, z: $0.z) }
    let paired = must({
        try ChangeDetection.compare(frame(room + beforeBox), frame(room + afterBox))
    }, "box moved")
    expect(paired.regions.count == 1, "one moved region, got \(paired.regions.count)")
    guard let region = paired.regions.first else { return }
    expectBox(region, kind: .moved, center: afterCenter, label: "moved")
    expect(region.fromVoxelCount == region.voxelCount, "same shell on both sides")
    let from = region.fromCentroid ?? Point3D(x: 99, y: 99, z: 99)
    expect(near(from.x, beforeCenter.x, 0.025), "from x \(from.x)")
    expect(near(from.y, beforeCenter.y, 0.025), "from y \(from.y)")
    expect(near(from.z, beforeCenter.z, 0.025), "from z \(from.z)")
    expect(region.fromBoundsMin != nil && region.fromBoundsMax != nil, "moved keeps the old AABB")

    var unpaired = ChangeDetection.Config()
    unpaired.pairMoves = false
    let split = must({
        try ChangeDetection.compare(frame(room + beforeBox), frame(room + afterBox), config: unpaired)
    }, "box moved, unpaired")
    expect(split.regions.count == 2, "appeared plus disappeared, got \(split.regions.count)")
    expect(split.regions.contains { $0.kind == .appeared }, "an appearance remains")
    expect(split.regions.contains { $0.kind == .disappeared }, "a disappearance remains")
    expect(!split.regions.contains { $0.kind == .moved }, "pairing is off")
}

func testChangeDetectionNoiseOnly() {
    let cloud = roomShell() + boxShell(center: Point3D(x: 1, y: 0.5, z: 0.30))
    func jitter(_ points: [Point3D], seed: UInt64) -> [Point3D] {
        var rng = SplitMix(state: seed)
        var out: [Point3D] = []
        out.reserveCapacity(points.count)
        for point in points {
            let draw = rng.unit()
            let dx = (rng.unit() * 2 - 1) * 0.01
            let dy = (rng.unit() * 2 - 1) * 0.01
            let dz = (rng.unit() * 2 - 1) * 0.01
            if draw < 0.20 { continue }
            out.append(Point3D(x: point.x + dx, y: point.y + dy, z: point.z + dz))
        }
        return out
    }
    let before = frame(jitter(cloud, seed: 0x9E37_79B9_7F4A_7C15))
    let after = frame(jitter(cloud, seed: 0xD1B5_4A32_D192_ED03))
    let report = must({ try ChangeDetection.compare(before, after) }, "noise")
    expect(ChangeDetection.Config().tolerance == 1 && ChangeDetection.Config().minVoxels == 6,
           "noise uses the default tolerance and minVoxels")
    expect(report.regions.isEmpty,
           "±1 cm and 20% dropout should leave no region, got \(report.regions.count) appearedVoxels=\(report.appearedVoxels) disappearedVoxels=\(report.disappearedVoxels)")

    let same = must({ try ChangeDetection.compare(frame(cloud), frame(cloud)) }, "identical")
    expect(same.regions.isEmpty && same.appearedVoxels == 0 && same.disappearedVoxels == 0,
           "a cloud against itself is empty")
}

func testChangeDetectionThreshold() {
    let room = roomShell()
    let box = boxShell(center: Point3D(x: 1, y: 0.5, z: 0.30))
    let speck = [Point3D(x: -0.8, y: 1.8, z: 0.413), Point3D(x: -0.8, y: 1.8, z: 0.463)]
    let before = frame(room)
    let after = frame(room + box + speck)
    var tight = ChangeDetection.Config()
    tight.minVoxels = 6
    var loose = ChangeDetection.Config()
    loose.minVoxels = 1
    let dropped = must({ try ChangeDetection.compare(before, after, config: tight) }, "minVoxels 6")
    let kept = must({ try ChangeDetection.compare(before, after, config: loose) }, "minVoxels 1")
    expect(dropped.regions.count == 1 && dropped.regions[0].kind == .appeared,
           "minVoxels 6 keeps only the box, got \(dropped.regions.count)")
    expect(kept.regions.count == 2, "minVoxels 1 keeps the box and the speck, got \(kept.regions.count)")
    expect(kept.regions[0].voxelCount == dropped.regions[0].voxelCount, "the box is the larger region")
    expect(kept.regions[1].voxelCount == 2 && kept.regions[1].kind == .appeared, "the speck is two voxels")
    expect(dropped.appearedVoxels == kept.appearedVoxels, "the cut does not change the pre-threshold count")
    expect(dropped.appearedVoxels == dropped.regions[0].voxelCount + 2, "appearedVoxels includes the speck")
}

func testChangeDetectionMismatch() {
    let room = frame(roomShell())
    var fine = ChangeDetection.Config()
    fine.voxelSize = 0.05
    var coarse = ChangeDetection.Config()
    coarse.voxelSize = 0.10
    let small = must({ try ChangeDetection.occupancyMap(room, config: fine) }, "0.05 map")
    let large = must({ try ChangeDetection.occupancyMap(room, config: coarse) }, "0.10 map")
    do {
        _ = try ChangeDetection.diff(small, large, config: fine)
        expect(false, "mismatched voxel sizes should throw")
    } catch ChangeDetection.Failure.voxelSizeMismatch {
    } catch {
        expect(false, "mismatch threw \(error)")
    }

    var wild = ChangeDetection.Config()
    wild.tolerance = 5
    do {
        _ = try ChangeDetection.occupancyMap(room, config: wild)
        expect(false, "tolerance 5 should throw")
    } catch ChangeDetection.Failure.invalidConfig {
    } catch {
        expect(false, "tolerance 5 threw \(error)")
    }
}

private func seconds(_ duration: Duration) -> Double {
    let parts = duration.components
    return Double(parts.seconds) + Double(parts.attoseconds) / 1e18
}

/// Keeps both budgets live. `_isDebugAssertConfiguration()` folds to a constant,
/// and an inlined ternary then warns that the other side is dead.
@inline(never)
private func compareBudgetSeconds(debug: Bool) -> Double {
    debug ? 3.0 : 0.1
}

@inline(never)
private func compareBudgetLabel(debug: Bool) -> String {
    debug ? "debug < 3 s" : "release < 100 ms"
}

func testChangeDetectionPerformance() -> Double {
    var points = roomShell()
    if points.count < 80_000 {
        let base = points
        var copy = 1
        while points.count < 80_000 {
            let shift = Float(copy) * 0.025
            for point in base {
                points.append(Point3D(x: point.x + shift, y: point.y + shift * 0.5, z: point.z))
                if points.count == 80_000 { break }
            }
            copy += 1
        }
    } else if points.count > 80_000 {
        points.removeLast(points.count - 80_000)
    }
    expect(points.count == 80_000, "performance cloud is 80k points, got \(points.count)")

    var rng = SplitMix(state: 0xA5A5_5A5A_1234_5678)
    var shifted = points
    for index in shifted.indices {
        shifted[index].x += (rng.unit() * 2 - 1) * 0.01
        shifted[index].y += (rng.unit() * 2 - 1) * 0.01
        shifted[index].z += (rng.unit() * 2 - 1) * 0.01
    }
    for index in 0..<8_000 {
        shifted[index].x += 1
    }
    let before = frame(points)
    let after = frame(shifted)
    _ = must({ try ChangeDetection.compare(before, after) }, "warmup")
    let clock = ContinuousClock()
    let start = clock.now
    let report = must({ try ChangeDetection.compare(before, after) }, "timed compare")
    let elapsed = seconds(start.duration(to: clock.now))
    expect(report.beforeVoxels > 0 && !report.regions.isEmpty, "the relocated patch is a real change")
    let limit = compareBudgetSeconds(debug: _isDebugAssertConfiguration())
    expect(elapsed < limit, "compare took \(elapsed) s, limit \(limit) s")
    return elapsed * 1000
}

func runChangeDetectionTests() {
    testChangeDetectionKeyMath()
    print("  ChangeDetection: pack/unpack round-trips; floor quantization; non-finite points drop"); passed()
    testChangeDetectionVoxelSize()
    print("  ChangeDetection: a coarser voxel occupies fewer keys, and a rebuild matches"); passed()
    testChangeDetectionRangeAndMinPoints()
    print("  ChangeDetection: range clip, minPoints, and voxelSize 0"); passed()
    testChangeDetectionBoxAdded()
    print("  ChangeDetection: a box added to the room is one appeared region"); passed()
    testChangeDetectionBoxRemoved()
    print("  ChangeDetection: a box removed from the room is one disappeared region"); passed()
    testChangeDetectionBoxMoved()
    print("  ChangeDetection: a shifted box pairs as moved, or stays split when pairing is off"); passed()
    testChangeDetectionNoiseOnly()
    print("  ChangeDetection: ±1 cm jitter and 20% dropout leave no region; a cloud matches itself"); passed()
    testChangeDetectionThreshold()
    print("  ChangeDetection: minVoxels drops a two-voxel speck and still counts it"); passed()
    testChangeDetectionMismatch()
    print("  ChangeDetection: voxel-size mismatch and tolerance 5 are rejected"); passed()
    let milliseconds = testChangeDetectionPerformance()
    let budget = compareBudgetLabel(debug: _isDebugAssertConfiguration())
    print("  ChangeDetection: 80k compare \(String(format: "%.1f", milliseconds)) ms (\(budget))"); passed()
}
