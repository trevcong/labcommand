import Foundation
import LabCommandCore

/// A deterministic room-shaped cloud: uniform pseudo-random points filling
/// [-5, 5] × [-4, 4] × [-1, 2].
private func roomCloud(count: Int) -> [Point3D] {
    var state: UInt64 = 0x9E37_79B9_7F4A_7C15
    func unit() -> Float {
        state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
        return Float(state >> 40) / Float(1 << 24)
    }
    return (0..<count).map { _ in
        Point3D(x: -5 + 10 * unit(), y: -4 + 8 * unit(), z: -1 + 3 * unit())
    }
}

private func near(_ a: Float, _ b: Float, _ tolerance: Float) -> Bool {
    abs(a - b) <= tolerance
}

func testRobustBoundsIgnoreOutliers() {
    var cloud = roomCloud(count: 20_000)
    // A handful of stray returns 40–90 m out, spread through the array.
    for i in 0..<20 {
        let far = 40 + Float(i) * 2.5
        cloud[i * 997] = Point3D(x: i.isMultiple(of: 2) ? far : -far, y: far, z: -far)
    }
    guard let bounds = RobustBounds.percentile([cloud]) else {
        return expect(false, "room with outliers has bounds")
    }
    expect(bounds.max.x < 5.01 && bounds.min.x > -5.01, "x stays in the room: \(bounds)")
    expect(bounds.max.y < 4.01 && bounds.min.y > -4.01, "y stays in the room: \(bounds)")
    expect(bounds.max.z < 2.01 && bounds.min.z > -1.01, "z stays in the room: \(bounds)")
    // 2nd–98th of a uniform spread trims ~4% of each span, no more.
    expect(bounds.max.x - bounds.min.x > 9.0, "x span keeps the room: \(bounds)")
    expect(bounds.max.y - bounds.min.y > 7.2, "y span keeps the room: \(bounds)")
}

func testRobustBoundsUniformNearMinMax() {
    let line = (0...1000).map { Point3D(x: Float($0) / 100, y: 0, z: 1) }
    guard let bounds = RobustBounds.percentile([line], maxSamples: 0) else {
        return expect(false, "line has bounds")
    }
    expect(near(bounds.min.x, 0.2, 1e-4) && near(bounds.max.x, 9.8, 1e-4), "exact 2nd/98th of 0…10: \(bounds)")
    expect(bounds.min.y == 0 && bounds.max.y == 0, "flat axis stays flat: \(bounds)")
    expect(bounds.min.z == 1 && bounds.max.z == 1, "constant axis keeps its value: \(bounds)")

    let tiny = [Point3D(x: -1, y: 0, z: 0), Point3D(x: 50, y: 2, z: 3), Point3D(x: Float.nan, y: 0, z: 0)]
    guard let small = RobustBounds.percentile([tiny]) else {
        return expect(false, "tiny cloud has bounds")
    }
    expect(small.min.x == -1 && small.max.x == 50 && small.max.z == 3, "too few points: plain min/max, NaN skipped: \(small)")
}

func testRobustBoundsSamplingAndEdges() {
    expect(RobustBounds.percentile([]) == nil, "no clouds, no bounds")
    expect(RobustBounds.percentile([[], []]) == nil, "empty clouds, no bounds")
    expect(RobustBounds.percentile([[Point3D(x: .infinity, y: 0, z: 0), Point3D(x: .nan, y: 1, z: 1)]]) == nil,
           "nothing finite, no bounds")

    // Two clouds sampled together: the bounds cover both, the sample stays capped.
    let left = roomCloud(count: 60_000)
    let right = roomCloud(count: 20_000).map { Point3D(x: $0.x + 20, y: $0.y, z: $0.z) }
    guard let both = RobustBounds.percentile([left, right], maxSamples: 2048) else {
        return expect(false, "two clouds have bounds")
    }
    expect(both.min.x < -4 && both.max.x > 24, "both clouds framed: \(both)")
    let again = RobustBounds.percentile([left, right], maxSamples: 2048)
    expect(again.map { $0.min == both.min && $0.max == both.max } == true, "same input, same bounds")
}

func testRobustBoundsSelectionMatchesSortedRanks() {
    // The line from above in a scrambled order: selection must land on the same ranks.
    var line = (0...1000).map { Point3D(x: Float($0) / 100, y: Float(1000 - $0) / 100, z: 1) }
    var state: UInt64 = 42
    for i in stride(from: line.count - 1, to: 0, by: -1) {
        state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
        line.swapAt(i, Int(state >> 33) % (i + 1))
    }
    guard let bounds = RobustBounds.percentile([line], maxSamples: 0) else {
        return expect(false, "scrambled line has bounds")
    }
    expect(near(bounds.min.x, 0.2, 1e-4) && near(bounds.max.x, 9.8, 1e-4), "exact 2nd/98th, any order: \(bounds)")
    expect(near(bounds.min.y, 0.2, 1e-4) && near(bounds.max.y, 9.8, 1e-4), "every axis selected on its own: \(bounds)")
    guard let mid = RobustBounds.percentile([line], lower: 0.255, upper: 0.5, maxSamples: 0) else {
        return expect(false, "scrambled line has mid bounds")
    }
    expect(near(mid.min.x, 2.55, 1e-4) && near(mid.max.x, 5.0, 1e-4), "between-rank interpolation and an exact rank: \(mid)")
    let ties = Array(repeating: Point3D(x: 3, y: 3, z: 3), count: 100) + [Point3D(x: 4, y: 4, z: 4)]
    let flat = RobustBounds.percentile([ties], maxSamples: 0)
    expect(flat.map { $0.min.x == 3 && $0.max.x == 3 } == true, "heavy ties select cleanly: \(String(describing: flat))")
}

func testRobustBoundsDropFarField() {
    expect(RobustBounds.maxFitRange == 15, "the camera fit keeps returns within 15 m of the rig")

    // ~10% of the points in a far wall 20–40 m out: too many for the 2% tail to trim.
    var cloud = roomCloud(count: 18_000)
    for i in 0..<2_000 {
        cloud.append(Point3D(x: 20 + Float(i % 200) / 10, y: Float(i % 7) - 3, z: 0.5))
    }
    // ~10% tall but close: height doesn't count toward the range, so these stay.
    for i in 0..<2_000 {
        cloud.append(Point3D(x: Float(i % 10) - 5, y: 0, z: 20))
    }
    guard let unfiltered = RobustBounds.percentile([cloud]) else {
        return expect(false, "room with far field has bounds")
    }
    expect(unfiltered.max.x > 15 || unfiltered.min.x < -15, "without the range cut the far band stretches the fit: \(unfiltered)")

    guard let bounds = RobustBounds.percentile([cloud], maxHorizontalRange: RobustBounds.maxFitRange) else {
        return expect(false, "in-range points have bounds")
    }
    expect(bounds.max.x < 5.01 && bounds.min.x > -5.01, "x stays in the room: \(bounds)")
    expect(bounds.max.y < 4.01 && bounds.min.y > -4.01, "y stays in the room: \(bounds)")
    expect(bounds.max.z == 20, "points 20 m up but 5 m out are kept (horizontal range only): \(bounds)")

    // Exactly at the limit is kept; just past it is not.
    let edge = [Point3D(x: 9, y: 12, z: 0), Point3D(x: 0, y: 0, z: 0), Point3D(x: 15.01, y: 0, z: 0)]
    let cut = RobustBounds.percentile([edge], maxHorizontalRange: 15)
    expect(cut.map { $0.max.y == 12 && $0.max.x == 9 } == true, "15 m on the dot stays, 15.01 m goes: \(String(describing: cut))")

    // Nothing within range: frame what there is rather than nothing.
    let far = [Point3D(x: 30, y: 0, z: 1), Point3D(x: 40, y: 5, z: 2)]
    let fallback = RobustBounds.percentile([far], maxHorizontalRange: RobustBounds.maxFitRange)
    expect(fallback.map { $0.min.x == 30 && $0.max.x == 40 } == true, "all far: the whole sample frames: \(String(describing: fallback))")
}

/// Fitting the robust bounds puts the room's limiting axis at ~90% of the view,
/// where a raw min/max fit to the same cloud shrinks it to a sliver.
func testRobustBoundsFillTheView() {
    var cloud = roomCloud(count: 20_000)
    cloud[123] = Point3D(x: 80, y: -70, z: 30)
    cloud[4567] = Point3D(x: -60, y: 90, z: -20)
    guard let bounds = RobustBounds.percentile([cloud]) else {
        return expect(false, "room has bounds")
    }

    func limitingNDC(min: Point3D, max: Point3D, of box: (min: Point3D, max: Point3D)) -> Float {
        let aspect: Float = 16.0 / 9.0
        let (eye, center, _) = CameraMath.fitToBounds(min: min, max: max, aspect: aspect, margin: 0.9)
        let mvp = CameraMath.perspective(fovY: CameraMath.defaultFovY, aspect: aspect, near: 0.1, far: 1000)
            * CameraMath.lookAt(eye: eye, center: center)
        var extent: Float = 0
        for corner in CameraMath.bboxCorners(min: box.min, max: box.max) {
            guard let ndc = mvp.transformToNDC(corner) else { continue }
            extent = Swift.max(extent, abs(ndc.x), abs(ndc.y))
        }
        return extent
    }

    let room = (min: Point3D(x: -5, y: -4, z: -1), max: Point3D(x: 5, y: 4, z: 2))
    let robust = limitingNDC(min: bounds.min, max: bounds.max, of: room)
    expect(robust > 0.85 && robust < 1.0, "room fills ~90% of the limiting axis, got \(robust)")

    let rawMin = Point3D(x: -60, y: -70, z: -20)
    let rawMax = Point3D(x: 80, y: 90, z: 30)
    let raw = limitingNDC(min: rawMin, max: rawMax, of: room)
    expect(raw < 0.3, "min/max framing shrinks the room to a sliver, got \(raw)")
}

func runRobustBoundsTests() {
    testRobustBoundsIgnoreOutliers()
    print("  RobustBounds: 2nd–98th percentiles drop far outliers, keep the room"); passed()
    testRobustBoundsUniformNearMinMax()
    print("  RobustBounds: exact percentiles on a line; tiny clouds fall back to min/max"); passed()
    testRobustBoundsSamplingAndEdges()
    print("  RobustBounds: empty / non-finite is nil; joint strided sample is deterministic"); passed()
    testRobustBoundsSelectionMatchesSortedRanks()
    print("  RobustBounds: quickselect percentiles match sorted ranks in any order"); passed()
    testRobustBoundsDropFarField()
    print("  RobustBounds: camera fit drops returns past 15 m horizontal; all-far falls back"); passed()
    testRobustBoundsFillTheView()
    print("  RobustBounds: fitted room fills ~90% of the limiting axis despite outliers"); passed()
}
