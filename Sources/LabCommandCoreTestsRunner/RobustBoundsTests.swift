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
    testRobustBoundsFillTheView()
    print("  RobustBounds: fitted room fills ~90% of the limiting axis despite outliers"); passed()
}
