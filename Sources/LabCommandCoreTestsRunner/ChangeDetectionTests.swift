import Foundation
import LabCommandCore

// MARK: - Scenes

private let gridStep: Float = 0.02
private let gridPhase: Float = 0.007

private struct SplitMix {
    var state: UInt64

    mutating func next() -> UInt64 {
        state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
        return state
    }

    mutating func unit() -> Float {
        Float(next() >> 40) / Float(1 << 24)
    }
}

private func shuffledPoints(_ points: [Point3D], seed: UInt64) -> [Point3D] {
    var out = points
    var rng = SplitMix(state: seed)
    var index = out.count
    while index > 1 {
        index -= 1
        let swap = Int(rng.next() % UInt64(index + 1))
        out.swapAt(index, swap)
    }
    return out
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
    // 0.40 m shell: 296 voxels is 49× minVoxels 6, and each edge is 8 cells against minExtent 2.
    expect(region.voxelCount == 296, "\(label) voxelCount \(region.voxelCount)")
    let spanX = region.boundsMax.x - region.boundsMin.x
    let spanY = region.boundsMax.y - region.boundsMin.y
    let spanZ = region.boundsMax.z - region.boundsMin.z
    expect(near(spanX, 0.40, 0.001) && near(spanY, 0.40, 0.001) && near(spanZ, 0.40, 0.001),
           "\(label) span \(spanX) \(spanY) \(spanZ)")
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
    expect(report.unknownVoxels == 0, "evidence off reports no unknown voxels")
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
    let defaults = ChangeDetection.Config()
    expect(defaults.tolerance == 1 && defaults.minVoxels == 6 && defaults.minExtent == 2,
           "noise uses the default tolerance, minVoxels, and minExtent")
    expect(defaults.rayStride == 1 && defaults.evidenceOfAbsence == false, "rayStride defaults to 1 and evidence is off")
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
    loose.minExtent = 1
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

/// A one-cell sheet is the Mid-360 static blip. A 0.10 m shell is the object that must still pass.
func testChangeDetectionExtent() {
    let sheet = [
        cellPoint(0, 0, 0), cellPoint(1, 0, 0), cellPoint(2, 0, 0), cellPoint(3, 0, 0), cellPoint(4, 0, 0),
        cellPoint(1, 1, 0), cellPoint(2, 2, 0),
    ]
    let streak = must({ try ChangeDetection.compare(frame([]), frame(sheet)) }, "streak")
    expect(streak.appearedVoxels == 7, "streak keeps its voxels before the cut, got \(streak.appearedVoxels)")
    expect(streak.regions.isEmpty, "a 5×3×1 sheet is not a change, regions \(streak.regions.count)")

    var open = ChangeDetection.Config()
    open.minExtent = 1
    let kept = must({ try ChangeDetection.compare(frame([]), frame(sheet), config: open) }, "streak open")
    expect(kept.regions.count == 1 && kept.regions[0].voxelCount == 7 && kept.regions[0].kind == .appeared,
           "minExtent 1 keeps the seven-voxel sheet")

    let center = Point3D(x: 1, y: 0.5, z: 0.30)
    let room = roomShell()
    let small = boxShell(center: center, side: 0.10)
    let added = must({ try ChangeDetection.compare(frame(room), frame(room + small)) }, "10 cm added")
    expect(added.regions.count == 1 && added.regions[0].kind == .appeared, "10 cm box appears")
    guard let appeared = added.regions.first else { return }
    expect(appeared.voxelCount == 8, "10 cm voxelCount \(appeared.voxelCount)")
    expect(near(appeared.boundsMax.x - appeared.boundsMin.x, 0.10, 0.001), "10 cm x span")
    expect(near(appeared.boundsMax.y - appeared.boundsMin.y, 0.10, 0.001), "10 cm y span")
    expect(near(appeared.boundsMax.z - appeared.boundsMin.z, 0.10, 0.001), "10 cm z span")
    expect(appeared.voxelCount >= 6 + 2, "10 cm count clears minVoxels 6")

    let removed = must({ try ChangeDetection.compare(frame(room + small), frame(room)) }, "10 cm removed")
    expect(removed.regions.count == 1 && removed.regions[0].kind == .disappeared, "10 cm box disappears")
    expect(removed.regions[0].voxelCount == 8, "10 cm removed voxelCount \(removed.regions[0].voxelCount)")

    let beforeCenter = Point3D(x: 0, y: 0.5, z: 0.30)
    let beforeBox = boxShell(center: beforeCenter, side: 0.10)
    let afterBox = beforeBox.map { Point3D(x: $0.x + 1, y: $0.y, z: $0.z) }
    let moved = must({
        try ChangeDetection.compare(frame(room + beforeBox), frame(room + afterBox))
    }, "10 cm moved")
    expect(moved.regions.count == 1 && moved.regions[0].kind == .moved, "10 cm box moves")
    expect(moved.regions[0].voxelCount == 8 && moved.regions[0].fromVoxelCount == 8, "10 cm move keeps both shells")
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

    var stride = ChangeDetection.Config()
    stride.rayStride = 3
    do {
        _ = try ChangeDetection.occupancyMap(room, config: stride)
        expect(false, "rayStride 3 should throw")
    } catch ChangeDetection.Failure.invalidConfig {
    } catch {
        expect(false, "rayStride 3 threw \(error)")
    }

    var origin = ChangeDetection.Config()
    origin.origin = Point3D(x: .nan, y: 0, z: 0)
    do {
        _ = try ChangeDetection.occupancyMap(room, config: origin)
        expect(false, "a non-finite origin should throw")
    } catch ChangeDetection.Failure.invalidConfig {
    } catch {
        expect(false, "non-finite origin threw \(error)")
    }

    var far = ChangeDetection.Config()
    far.evidenceOfAbsence = true
    far.origin = Point3D(x: 80_000, y: 0, z: 0)
    do {
        _ = try ChangeDetection.occupancyMap(room, config: far)
        expect(false, "an origin outside the grid should throw when evidence is on")
    } catch ChangeDetection.Failure.invalidConfig {
    } catch {
        expect(false, "far origin threw \(error)")
    }

    var thin = ChangeDetection.Config()
    thin.minExtent = 0
    do {
        _ = try ChangeDetection.occupancyMap(room, config: thin)
        expect(false, "minExtent 0 should throw")
    } catch ChangeDetection.Failure.invalidConfig {
    } catch {
        expect(false, "minExtent 0 threw \(error)")
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
    // The relocated patch is a room surface, one cell thick. The streak gate would
    // drop it and the timer would not cluster a real change.
    var config = ChangeDetection.Config()
    config.minExtent = 1
    _ = must({ try ChangeDetection.compare(before, after, config: config) }, "warmup")
    let clock = ContinuousClock()
    let start = clock.now
    let report = must({ try ChangeDetection.compare(before, after, config: config) }, "timed compare")
    let elapsed = seconds(start.duration(to: clock.now))
    expect(report.beforeVoxels > 0 && !report.regions.isEmpty, "the relocated patch is a real change")
    let limit = compareBudgetSeconds(debug: _isDebugAssertConfiguration())
    expect(elapsed < limit, "compare took \(elapsed) s, limit \(limit) s")
    return elapsed * 1000
}

// MARK: - Evidence

private func axisCenter(_ index: Int) -> Float {
    (Float(index) + 0.5) * 0.05
}

private func cellPoint(_ i: Int, _ j: Int, _ k: Int) -> Point3D {
    Point3D(x: axisCenter(i), y: axisCenter(j), z: axisCenter(k))
}

private func column(_ indices: [Int]) -> [Point3D] {
    indices.map { cellPoint($0, 0, 0) }
}

private func evidenceConfig(
    tolerance: Int = 0,
    minVoxels: Int = 6,
    minPoints: Int = 1,
    reportUnobserved: Bool = false,
    stride: Int = 1,
    origin: Point3D = Point3D(x: 0, y: 0, z: 0),
    minExtent: Int = 1
) -> ChangeDetection.Config {
    var config = ChangeDetection.Config()
    config.evidenceOfAbsence = true
    config.tolerance = tolerance
    config.minVoxels = minVoxels
    config.minPoints = minPoints
    config.reportUnobserved = reportUnobserved
    config.rayStride = stride
    config.origin = origin
    // Column fixtures are one cell thick. Callers that diff a real box pass 2.
    config.minExtent = minExtent
    return config
}

private func diffRecord(
    _ before: [Point3D],
    _ after: [Point3D],
    _ config: ChangeDetection.Config,
    _ label: String
) -> (report: ChangeDetection.Report, json: String) {
    let beforeMap = must({ try ChangeDetection.occupancyMap(frame(before), config: config) }, "\(label) before")
    let afterMap = must({ try ChangeDetection.occupancyMap(frame(after), config: config) }, "\(label) after")
    let report = must({ try ChangeDetection.diff(beforeMap, afterMap, config: config) }, label)
    return (report, ChangeDetection.json(report: report, before: beforeMap, after: afterMap, config: config))
}

private func expectBound(_ value: Float, _ index: Int, _ label: String, exclusive: Bool) {
    let expected = exclusive ? Float(index + 1) * 0.05 : Float(index) * 0.05
    expect(value == expected, "\(label) \(value) vs \(expected)")
}

private func clearLinePair() -> (before: [Point3D], after: [Point3D]) {
    (column(Array(20...27) + [40]), column([40]))
}

private let clearLineGolden = "{\"schema\":\"labcommand.change-detection.v2\",\"evidence\":true,\"report_unobserved\":false,\"origin_bits\":[\"00000000\",\"00000000\",\"00000000\"],\"ray_stride\":1,\"config\":{\"voxel_size_bits\":\"3d4ccccd\",\"max_range_bits\":null,\"min_points\":1,\"tolerance\":0,\"min_voxels\":6,\"min_extent\":1,\"pair_moves\":true,\"max_move_bits\":\"3fc00000\",\"move_count_ratio_bits\":\"40000000\"},\"before\":{\"points\":9,\"accepted\":9,\"dropped\":0,\"voxels\":9},\"after\":{\"points\":1,\"accepted\":1,\"dropped\":0,\"voxels\":1},\"appeared_voxels\":0,\"disappeared_voxels\":8,\"unknown_voxels\":0,\"regions\":[{\"kind\":\"disappeared\",\"voxel_count\":8,\"min_key\":\"4611776178381914112\",\"centroid_bits\":[\"3f99999a\",\"3ccccccd\",\"3ccccccd\"],\"bounds_min_bits\":[\"3f800000\",\"00000000\",\"00000000\"],\"bounds_max_bits\":[\"3fb33333\",\"3d4ccccd\",\"3d4ccccd\"],\"volume_bits\":\"3a83126f\"}]}"

func testChangeDetectionOccluded() {
    let before = column(Array(8...15) + [16] + Array(20...27))
    let after = column([16])
    let hidden = diffRecord(before, after, evidenceConfig(), "occluded")
    expect(hidden.report.appearedVoxels == 0, "occluder is in both maps, appeared \(hidden.report.appearedVoxels)")
    expect(hidden.report.unknownVoxels == 8, "back face unknown \(hidden.report.unknownVoxels)")
    expect(hidden.report.disappearedVoxels == 8, "front free \(hidden.report.disappearedVoxels)")
    expect(hidden.report.regions.count == 1, "one disappeared region, got \(hidden.report.regions.count)")
    guard let front = hidden.report.regions.first else { return }
    expect(front.kind == .disappeared && front.voxelCount == 8, "front region \(front.kind) \(front.voxelCount)")
    expectBound(front.boundsMin.x, 8, "front min x", exclusive: false)
    expectBound(front.boundsMax.x, 15, "front max x", exclusive: true)
    expect(front.boundsMin.y == 0 && front.boundsMin.z == 0, "front yz min")
    expect(front.boundsMax.y == 0.05 && front.boundsMax.z == 0.05, "front yz max")

    var show = evidenceConfig()
    show.reportUnobserved = true
    let full = diffRecord(before, after, show, "occluded unobserved").report
    let kinds = full.regions.map { "\($0.kind.rawValue):\($0.voxelCount)" }.joined(separator: ",")
    expect(full.regions.count == 2, "disappeared plus unobserved, got \(kinds)")
    expect(full.regions[0].kind == .disappeared && full.regions[0].voxelCount == 8, "front stays disappeared")
    expect(full.regions[1].kind == .unobserved && full.regions[1].voxelCount == 8, "back is unobserved, not disappeared")
    expectBound(full.regions[1].boundsMin.x, 20, "back min x", exclusive: false)
    expectBound(full.regions[1].boundsMax.x, 27, "back max x", exclusive: true)
    expect(full.unknownVoxels == 8, "reporting regions does not change the unknown count")
}

func testChangeDetectionClearLine() {
    let pair = clearLinePair()
    let record = diffRecord(pair.before, pair.after, evidenceConfig(), "clear line")
    expect(record.report.appearedVoxels == 0, "backstop is in both maps")
    expect(record.report.unknownVoxels == 0, "the column is free, unknown \(record.report.unknownVoxels)")
    expect(record.report.disappearedVoxels == 8, "eight free voxels")
    expect(record.report.regions.count == 1, "one region \(record.report.regions.count)")
    guard let region = record.report.regions.first else { return }
    expect(region.kind == .disappeared && region.voxelCount == 8, "disappeared column")
    expect(region.minKey == 4_611_776_178_381_914_112, "min key \(region.minKey)")
    expect(record.json == clearLineGolden, "clear-line JSON drifted")
}

func testChangeDetectionDiagonalTie() {
    let origin = cellPoint(0, 0, 0)
    let before = [
        cellPoint(0, 0, 0),
        cellPoint(1, 1, 1),
        cellPoint(2, 2, 2),
        cellPoint(1, 0, 0),
        cellPoint(3, 3, 3),
    ]
    let config = evidenceConfig(minVoxels: 1, reportUnobserved: true, origin: origin)
    let report = diffRecord(before, [cellPoint(3, 3, 3)], config, "diagonal").report
    let kinds = report.regions.map { "\($0.kind.rawValue):\($0.voxelCount)" }.joined(separator: ",")
    expect(report.appearedVoxels == 0, "endpoint is in both maps")
    expect(report.disappearedVoxels == 3, "diagonal cells \(report.disappearedVoxels)")
    expect(report.unknownVoxels == 1, "face neighbor \(report.unknownVoxels)")
    expect(report.regions.count == 2, "two regions \(kinds)")
    expect(report.regions[0].kind == .disappeared && report.regions[0].voxelCount == 3, "diagonal region")
    expect(report.regions[1].kind == .unobserved && report.regions[1].voxelCount == 1, "unobserved face neighbor")
    expectBound(report.regions[1].boundsMin.x, 1, "neighbor min x", exclusive: false)
    expect(report.regions[1].boundsMin.y == 0 && report.regions[1].boundsMin.z == 0, "neighbor stays off the diagonal")
}

func testChangeDetectionSlide() {
    let toward = diffRecord(column([20]), column([19]), evidenceConfig(tolerance: 1, minVoxels: 1), "slide toward").report
    expect(toward.unknownVoxels == 1, "old cell stays in shadow \(toward.unknownVoxels)")
    expect(toward.disappearedVoxels == 0, "shadow is not a disappearance")
    expect(toward.appearedVoxels == 0, "the one-cell slide toward the sensor is dilated away")

    let away = diffRecord(column([20]), column([21]), evidenceConfig(tolerance: 1, minVoxels: 1), "slide away").report
    expect(away.unknownVoxels == 0 && away.disappearedVoxels == 0 && away.appearedVoxels == 0,
           "slide away is free then suppressed, unknown \(away.unknownVoxels) disappeared \(away.disappearedVoxels) appeared \(away.appearedVoxels)")
}

func testChangeDetectionEvidenceOrder() {
    // A line on x from i=0...5, and a line on y at i=1. Same count. The x line
    // has the smaller min key and the larger centroid x.
    var after: [Point3D] = []
    for i in 0...5 { after.append(cellPoint(i, 0, 0)) }
    for j in 20...25 { after.append(cellPoint(1, j, 0)) }
    var config = ChangeDetection.Config()
    config.tolerance = 0
    config.minVoxels = 1
    config.minExtent = 1
    config.pairMoves = false
    let report = diffRecord([], after, config, "min-key order").report
    expect(report.regions.count == 2, "two same-size regions \(report.regions.count)")
    expect(report.regions[0].voxelCount == 6 && report.regions[1].voxelCount == 6, "equal counts")
    expect(report.regions[0].minKey < report.regions[1].minKey, "minKey ascending")
    expect(report.regions[0].centroid.x > report.regions[1].centroid.x,
           "centroid x would have ordered them the other way")

    config.pairMoves = true
    let tied = diffRecord(
        [cellPoint(-20, 0, 0), cellPoint(20, 0, 0)],
        [cellPoint(0, 0, 0)],
        config,
        "distance tie"
    ).report
    expect(!tied.regions.contains { $0.kind == .moved }, "equal distances stay unpaired")
    expect(tied.regions.count == 3, "one appeared and two disappeared, got \(tied.regions.count)")
}

func testChangeDetectionDeterminism() {
    let pair = clearLinePair()
    let before = pair.before + pair.before
    let after = pair.after + pair.after
    let once = evidenceConfig(minPoints: 1)
    let original = diffRecord(before, after, once, "duplicated")
    let shuffled = diffRecord(
        shuffledPoints(before, seed: 0xC0FFEE),
        shuffledPoints(after, seed: 0xC0FFEE),
        once,
        "shuffled"
    )
    expect(shuffled.report == original.report, "shuffled report matches")
    expect(shuffled.json == original.json, "shuffled JSON matches")

    var counted = once
    counted.minPoints = 2
    let doubled = diffRecord(before, after, counted, "minPoints 2")
    let expected = original.json.replacingOccurrences(of: "\"min_points\":1", with: "\"min_points\":2")
    expect(doubled.json == expected, "minPoints 2 JSON matches except min_points")
    expect(doubled.report.regions == original.report.regions, "minPoints 2 occupancy is unchanged")

    let room = roomShell()
    let beforeBox = boxShell(center: Point3D(x: 0, y: 0.5, z: 0.30))
    let afterBox = beforeBox.map { Point3D(x: $0.x + 1, y: $0.y, z: $0.z) }
    let movedConfig = evidenceConfig(tolerance: 1, minVoxels: 6, minExtent: 2)
    let movedBefore = room + beforeBox
    let movedAfter = room + afterBox
    let moved = diffRecord(movedBefore, movedAfter, movedConfig, "moved")
    let movedShuffle = diffRecord(
        shuffledPoints(movedBefore, seed: 0xC0FFEE),
        shuffledPoints(movedAfter, seed: 0xC0FFEE),
        movedConfig,
        "moved shuffle"
    )
    expect(movedShuffle.report == moved.report, "shuffled move report matches")
    expect(movedShuffle.json == moved.json, "shuffled move JSON matches")
}

/// Voxel size 1. Origin center `(0.5, 0.5, 0.5)` is cell `(0, 0, 0)`.
private func unitCenter(_ i: Int, _ j: Int, _ k: Int) -> Point3D {
    Point3D(x: Float(i) + 0.5, y: Float(j) + 0.5, z: Float(k) + 0.5)
}

private func unitEvidenceConfig(
    origin: Point3D,
    reportUnobserved: Bool = false
) -> ChangeDetection.Config {
    var config = evidenceConfig(
        tolerance: 0,
        minVoxels: 1,
        reportUnobserved: reportUnobserved,
        origin: origin,
        minExtent: 1
    )
    config.voxelSize = 1
    config.pairMoves = false
    return config
}

/// The tied walk from `(0.5, 0.5, 0.5)` to `(16, -15, 0.5)` used to leave the
/// grid at `(16, -16, 0)`. `fallback` adds a far return so the ray uses the
/// open-addressed walker instead of the bit grid.
private func boundaryEndpointReport(fallback: Bool) -> ChangeDetection.Report {
    let origin = Point3D(x: 0.5, y: 0.5, z: 0.5)
    let endpoint = Point3D(x: 16, y: -15, z: 0.5)
    var before: [Point3D] = []
    for i in 0...15 {
        before.append(unitCenter(i, -i, 0))
    }
    // Cells the unclamped tie aliases into the allocated words, plus two cells
    // past the grid where the old loop indexed off the end of `words`.
    before.append(unitCenter(15, 0, 0))
    before.append(unitCenter(16, -1, 0))
    before.append(unitCenter(16, -16, 0))
    before.append(unitCenter(21, -21, 0))
    before.append(endpoint)
    var after = [endpoint]
    if fallback {
        let far = unitCenter(700, 700, 700)
        before.append(far)
        after.append(far)
    }
    return diffRecord(before, after, unitEvidenceConfig(origin: origin), "boundary endpoint").report
}

private func expectBoundaryEndpoint(_ report: ChangeDetection.Report, _ label: String) {
    expect(report.appearedVoxels == 0, "\(label) endpoint stays occupied, appeared \(report.appearedVoxels)")
    expect(report.disappearedVoxels == 16, "\(label) diagonal has 16 cells, disappeared \(report.disappearedVoxels)")
    expect(report.unknownVoxels == 4, "\(label) alias and overshoot cells stay unknown \(report.unknownVoxels)")
    expect(report.regions.count == 1, "\(label) one disappeared region, got \(report.regions.count)")
    guard let region = report.regions.first else { return }
    expect(region.kind == .disappeared && region.voxelCount == 16, "\(label) region \(region.kind) \(region.voxelCount)")
    guard let originKey = VoxelKey.pack(i: 0, j: 0, k: 0) else {
        expect(false, "\(label) origin key")
        return
    }
    expect(region.minKey == originKey, "\(label) walk starts at the origin cell")
    expect(region.boundsMin.x == 0 && region.boundsMin.y == -15 && region.boundsMin.z == 0,
           "\(label) boundsMin \(region.boundsMin)")
    expect(region.boundsMax.x == 16 && region.boundsMax.y == 1 && region.boundsMax.z == 1,
           "\(label) boundsMax \(region.boundsMax)")
}

func testChangeDetectionBoundaryEndpoint() {
    expectBoundaryEndpoint(boundaryEndpointReport(fallback: false), "bit grid")
    expectBoundaryEndpoint(boundaryEndpointReport(fallback: true), "open-addressed")
}

private func indexEdge(_ value: Float, exclusive: Bool) -> Int {
    let truncated = Int(value)
    return exclusive ? truncated - 1 : truncated
}

private func chebyshevToBox(
    i: Int, j: Int, k: Int,
    iLo: Int, iHi: Int, jLo: Int, jHi: Int, kLo: Int, kHi: Int
) -> Int {
    func axis(_ value: Int, _ lo: Int, _ hi: Int) -> Int {
        if value < lo { return lo - value }
        if value > hi { return value - hi }
        return 0
    }
    return max(axis(i, iLo, iHi), max(axis(j, jLo, jHi), axis(k, kLo, kHi)))
}

func testChangeDetectionBoundaryOctants() {
    let origin = Point3D(x: 0.5, y: 0.5, z: 0.5)
    let inverse: Float = 1
    let originI = VoxelKey.index(origin.x, inverseVoxelSize: inverse)
    let originJ = VoxelKey.index(origin.y, inverseVoxelSize: inverse)
    let originK = VoxelKey.index(origin.z, inverseVoxelSize: inverse)
    var rng = SplitMix(state: 0xB0A1_DDA5)
    var samples = 0
    for sx in [-1, 1] {
        for sy in [-1, 1] {
            for sz in [-1, 1] {
                // Bits 0/1/2 place x/y/z on a voxel face. Every non-empty mask is a boundary endpoint.
                for mask in 1...7 {
                    let magX = Int(rng.next() % 8) + 1
                    let magY = Int(rng.next() % 8) + 1
                    let magZ = Int(rng.next() % 8) + 1
                    let endI = sx * magX
                    let endJ = sy * magY
                    let endK = sz * magZ
                    let point = Point3D(
                        x: mask & 1 != 0 ? Float(endI) : Float(endI) + 0.5,
                        y: mask & 2 != 0 ? Float(endJ) : Float(endJ) + 0.5,
                        z: mask & 4 != 0 ? Float(endK) : Float(endK) + 0.5
                    )
                    let ei = VoxelKey.index(point.x, inverseVoxelSize: inverse)
                    let ej = VoxelKey.index(point.y, inverseVoxelSize: inverse)
                    let ek = VoxelKey.index(point.z, inverseVoxelSize: inverse)
                    expect(ei == endI && ej == endJ && ek == endK,
                           "octant (\(sx),\(sy),\(sz)) mask \(mask) quantizer \(ei),\(ej),\(ek)")
                    let iLo = min(originI, ei)
                    let iHi = max(originI, ei)
                    let jLo = min(originJ, ej)
                    let jHi = max(originJ, ej)
                    let kLo = min(originK, ek)
                    let kHi = max(originK, ek)
                    var before: [Point3D] = []
                    before.reserveCapacity((iHi - iLo + 3) * (jHi - jLo + 3) * (kHi - kLo + 3))
                    for i in (iLo - 1)...(iHi + 1) {
                        for j in (jLo - 1)...(jHi + 1) {
                            for k in (kLo - 1)...(kHi + 1) {
                                before.append(unitCenter(i, j, k))
                            }
                        }
                    }
                    let report = diffRecord(
                        before,
                        [point],
                        unitEvidenceConfig(origin: origin),
                        "octant (\(sx),\(sy),\(sz)) mask \(mask)"
                    ).report
                    let span = max(abs(ei - originI), max(abs(ej - originJ), abs(ek - originK)))
                    let aabbCount = (iHi - iLo + 1) * (jHi - jLo + 1) * (kHi - kLo + 1)
                    expect(report.disappearedVoxels >= span,
                           "octant (\(sx),\(sy),\(sz)) mask \(mask) marked \(report.disappearedVoxels), long axis \(span)")
                    expect(report.disappearedVoxels <= aabbCount - 1,
                           "octant (\(sx),\(sy),\(sz)) mask \(mask) left the AABB or marked the end, count \(report.disappearedVoxels)")
                    expect(!report.regions.isEmpty, "octant (\(sx),\(sy),\(sz)) mask \(mask) produced no region")
                    var nearest = Int.max
                    for region in report.regions where region.kind == .disappeared {
                        let riLo = indexEdge(region.boundsMin.x, exclusive: false)
                        let rjLo = indexEdge(region.boundsMin.y, exclusive: false)
                        let rkLo = indexEdge(region.boundsMin.z, exclusive: false)
                        let riHi = indexEdge(region.boundsMax.x, exclusive: true)
                        let rjHi = indexEdge(region.boundsMax.y, exclusive: true)
                        let rkHi = indexEdge(region.boundsMax.z, exclusive: true)
                        expect(riLo >= iLo && riHi <= iHi && rjLo >= jLo && rjHi <= jHi && rkLo >= kLo && rkHi <= kHi,
                               "octant (\(sx),\(sy),\(sz)) mask \(mask) visited \(riLo)...\(riHi), \(rjLo)...\(rjHi), \(rkLo)...\(rkHi) outside \(iLo)...\(iHi), \(jLo)...\(jHi), \(kLo)...\(kHi)")
                        nearest = min(nearest, chebyshevToBox(
                            i: ei, j: ej, k: ek,
                            iLo: riLo, iHi: riHi, jLo: rjLo, jHi: rjHi, kLo: rkLo, kHi: rkHi
                        ))
                    }
                    expect(nearest <= 1,
                           "octant (\(sx),\(sy),\(sz)) mask \(mask) stopped \(nearest) cells short of the end")
                    samples += 1
                }
            }
        }
    }
    expect(samples == 56, "sweep covered \(samples) boundary endpoints")
}

func testChangeDetectionWideGridFallback() {
    let before = [cellPoint(10, 10, 10), cellPoint(700, 700, 700)]
    let after = [cellPoint(700, 700, 700)]
    let report = diffRecord(before, after, evidenceConfig(minVoxels: 1), "wide fallback").report
    expect(report.disappearedVoxels == 1, "the diagonal still marks (10,10,10), disappeared \(report.disappearedVoxels)")
    expect(report.unknownVoxels == 0, "the cell is not unknown")
    expect(report.appearedVoxels == 0, "the endpoint is in both maps")
    expect(report.regions.count == 1 && report.regions[0].kind == .disappeared, "one disappeared cell")
}

private func indoorPoints(moveBoxA: Bool) -> [Point3D] {
    let voxel: Float = 0.05
    func center(_ index: Int) -> Float { (Float(index) + 0.5) * voxel }
    func point(_ i: Int, _ j: Int, _ k: Int) -> Point3D {
        Point3D(x: center(i), y: center(j), z: center(k))
    }
    var points: [Point3D] = []
    points.reserveCapacity(90_000)
    for i in -80..<80 {
        for j in -80..<80 {
            for k in -30..<30 where i == -80 || i == 79 || j == -80 || j == 79 || k == -30 || k == 29 {
                points.append(point(i, j, k))
            }
        }
    }
    func addBox(i0: Int, i1: Int, j0: Int, j1: Int, k0: Int, k1: Int, shiftI: Int) {
        for i in i0...i1 {
            for j in j0...j1 {
                for k in k0...k1 where i == i0 || i == i1 || j == j0 || j == j1 || k == k0 || k == k1 {
                    points.append(point(i + shiftI, j, k))
                }
            }
        }
    }
    addBox(i0: 20, i1: 27, j0: -10, j1: -3, k0: -8, k1: -1, shiftI: moveBoxA ? 20 : 0)
    addBox(i0: -30, i1: -23, j0: 16, j1: 23, k0: -4, k1: 3, shiftI: 0)
    addBox(i0: -6, i1: 5, j0: -28, j1: -17, k0: 4, k1: 13, shiftI: 0)
    return points
}

func testChangeDetectionIndoorPerformance() -> Double {
    let beforePoints = indoorPoints(moveBoxA: false)
    let afterPoints = indoorPoints(moveBoxA: true)
    expect(beforePoints.count == 89_320 && afterPoints.count == 89_320,
           "indoor clouds are 89320 points, got \(beforePoints.count) and \(afterPoints.count)")
    let before = frame(beforePoints)
    let after = frame(afterPoints)
    let config = evidenceConfig(tolerance: 1, minVoxels: 6, minExtent: 2)
    _ = must({ try ChangeDetection.compare(before, after, config: config) }, "indoor warmup")
    let clock = ContinuousClock()
    let start = clock.now
    let report = must({ try ChangeDetection.compare(before, after, config: config) }, "indoor timed")
    let elapsed = seconds(start.duration(to: clock.now))
    let summary = report.regions.map { "\($0.kind.rawValue):\($0.voxelCount)" }.joined(separator: ",")
    expect(report.regions.contains { $0.kind == .moved }, "the relocated box is a moved region (\(summary))")
    let limit = compareBudgetSeconds(debug: _isDebugAssertConfiguration())
    expect(elapsed < limit, "indoor compare took \(elapsed) s, limit \(limit) s")
    return elapsed * 1000
}

private func longRayClouds() -> (before: [Point3D], after: [Point3D]) {
    let voxel: Float = 0.05
    var state: UInt64 = 0x15A6
    var seen = Set<Int64>(minimumCapacity: 80_000)
    var points: [Point3D] = []
    points.reserveCapacity(80_000)
    while points.count < 80_000 {
        state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
        let i = Int(state % 599) - 299
        let j = Int((state >> 21) % 599) - 299
        let k = Int((state >> 42) % 599) - 299
        if i * i + j * j + k * k > 299 * 299 { continue }
        guard let key = VoxelKey.pack(i: i, j: j, k: k), seen.insert(key).inserted else { continue }
        points.append(Point3D(
            x: (Float(i) + 0.5) * voxel,
            y: (Float(j) + 0.5) * voxel,
            z: (Float(k) + 0.5) * voxel
        ))
    }
    var after = points
    let maxRange: Float = 15
    let maxRange2 = maxRange * maxRange
    for index in 0..<8_000 {
        var moved = after[index]
        moved.x += Float(1)
        let range2 = (moved.x * moved.x + moved.y * moved.y) + moved.z * moved.z
        if range2 <= maxRange2 {
            after[index] = moved
        }
    }
    return (points, after)
}

private func timeLongRay(before: PointCloudFrame, after: PointCloudFrame, stride: Int) -> Double {
    var config = evidenceConfig(tolerance: 1, minVoxels: 6, stride: stride, minExtent: 2)
    config.maxRange = 15
    _ = must({ try ChangeDetection.compare(before, after, config: config) }, "long-ray warmup stride \(stride)")
    let clock = ContinuousClock()
    let start = clock.now
    _ = must({ try ChangeDetection.compare(before, after, config: config) }, "long-ray stride \(stride)")
    return seconds(start.duration(to: clock.now)) * 1000
}

func testChangeDetectionLongRayBenchmark() -> (stride1: Double, stride64: Double) {
    let clouds = longRayClouds()
    expect(clouds.before.count == 80_000 && clouds.after.count == 80_000, "long-ray clouds are 80k points")
    let before = frame(clouds.before)
    let after = frame(clouds.after)
    let stride1 = timeLongRay(before: before, after: after, stride: 1)
    let stride64 = timeLongRay(before: before, after: after, stride: 64)
    return (stride1, stride64)
}

private func expectSameBytes(_ got: Data, _ want: Data, _ label: String) {
    if got == want { return }
    let count = min(got.count, want.count)
    var index = 0
    while index < count && got[index] == want[index] {
        index += 1
    }
    expect(false, "\(label) JSON differs at byte \(index) (got \(got.count), file \(want.count))")
}

func testChangeDetectionSnapshotGoldens() -> (jt128: ChangeDetection.Report, mid360: ChangeDetection.Report, jt128ms: Double, mid360ms: Double) {
    func load(_ name: String) -> Data {
        let path = "Tests/Fixtures/change-detection/\(name)"
        guard let data = FileManager.default.contents(atPath: path) else {
            expect(false, "missing \(path)")
            return Data()
        }
        return data
    }
    func check(_ sensor: String) -> (ChangeDetection.Report, Double) {
        let beforeData = load("a-\(sensor).bin")
        let afterData = load("b-\(sensor).bin")
        let golden = load("\(sensor).json")
        let before = must({ try PointCloud2CDR.decode(beforeData) }, "\(sensor) before")
        let after = must({ try PointCloud2CDR.decode(afterData) }, "\(sensor) after")
        var config = ChangeDetection.Config()
        config.evidenceOfAbsence = true
        let clock = ContinuousClock()
        let start = clock.now
        let beforeMap = must({ try ChangeDetection.occupancyMap(before, config: config) }, "\(sensor) map")
        let afterMap = must({ try ChangeDetection.occupancyMap(after, config: config) }, "\(sensor) map")
        let report = must({ try ChangeDetection.diff(beforeMap, afterMap, config: config) }, "\(sensor) diff")
        let elapsed = seconds(start.duration(to: clock.now)) * 1000
        let encoded = Data((ChangeDetection.json(report: report, before: beforeMap, after: afterMap, config: config) + "\n").utf8)
        expectSameBytes(encoded, golden, sensor)
        return (report, elapsed)
    }
    let jt128 = check("jt128")
    let mid360 = check("mid360")
    return (jt128.0, mid360.0, jt128.1, mid360.1)
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
    testChangeDetectionExtent()
    print("  ChangeDetection: a one-cell sheet drops; a 10 cm shell still adds, removes, and moves"); passed()
    testChangeDetectionMismatch()
    print("  ChangeDetection: voxel-size mismatch, tolerance 5, rayStride, and origin are rejected"); passed()
    testChangeDetectionOccluded()
    print("  ChangeDetection: an occluder frees the front face and leaves the back unknown"); passed()
    testChangeDetectionClearLine()
    print("  ChangeDetection: a clear line of sight is the v2 JSON golden"); passed()
    testChangeDetectionDiagonalTie()
    print("  ChangeDetection: a corner tie steps the diagonal and leaves the face neighbor unknown"); passed()
    testChangeDetectionBoundaryEndpoint()
    print("  ChangeDetection: a tied boundary return stays in the grid and stops on the end cell"); passed()
    testChangeDetectionBoundaryOctants()
    print("  ChangeDetection: boundary endpoints in every octant stay inside the origin–end box"); passed()
    testChangeDetectionSlide()
    print("  ChangeDetection: a slide toward the sensor stays unknown; a slide away is suppressed"); passed()
    testChangeDetectionEvidenceOrder()
    print("  ChangeDetection: regions sort by min key, and equal move distances stay unpaired"); passed()
    testChangeDetectionDeterminism()
    print("  ChangeDetection: shuffled input and minPoints 2 keep the same JSON"); passed()
    testChangeDetectionWideGridFallback()
    print("  ChangeDetection: a ray outside the bit grid still marks the cell it crosses"); passed()
    let milliseconds = testChangeDetectionPerformance()
    let budget = compareBudgetLabel(debug: _isDebugAssertConfiguration())
    print("  ChangeDetection: 80k compare \(String(format: "%.1f", milliseconds)) ms (\(budget))"); passed()
    let indoor = testChangeDetectionIndoorPerformance()
    print("  ChangeDetection: indoor evidence stride 1 \(String(format: "%.1f", indoor)) ms (\(budget))"); passed()
    let longRay = testChangeDetectionLongRayBenchmark()
    print("  ChangeDetection: long-ray stride 1 \(String(format: "%.1f", longRay.stride1)) ms, stride 64 \(String(format: "%.1f", longRay.stride64)) ms (printed)"); passed()
    let snapshots = testChangeDetectionSnapshotGoldens()
    func regionLine(_ report: ChangeDetection.Report) -> String {
        "regions=\(report.regions.count) appeared=\(report.appearedVoxels) disappeared=\(report.disappearedVoxels) unknown=\(report.unknownVoxels)"
    }
    print("  ChangeDetection: jt128 golden \(regionLine(snapshots.jt128)) \(String(format: "%.1f", snapshots.jt128ms)) ms"); passed()
    print("  ChangeDetection: mid360 golden \(regionLine(snapshots.mid360)) \(String(format: "%.1f", snapshots.mid360ms)) ms"); passed()
}
