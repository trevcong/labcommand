import Foundation
import LabCommandCore

/// Tests for RigSpec extrinsics and transformations.

func testMid360ExtrinsicsTransform() {
    // Mid-360 has a known offset from its IMU: (0.011, 0.02329, -0.04412) m
    // Transform: p_imu = p_lidar - offset
    // (1, 2, 3) → (1 - 0.011, 2 - 0.02329, 3 - (-0.04412)) = (0.989, 1.97671, 3.04412)
    let ex = RigSpec.mid360Extrinsics
    let point = Point3D(x: 1, y: 2, z: 3)
    let transformed = ex.transform(point)
    
    let tolerance: Float = 1e-5
    expect(abs(transformed.x - 0.989) < tolerance,
           "Mid-360 transform X: expected 0.989, got \(transformed.x)")
    expect(abs(transformed.y - 1.97671) < tolerance,
           "Mid-360 transform Y: expected 1.97671, got \(transformed.y)")
    expect(abs(transformed.z - 3.04412) < tolerance,
           "Mid-360 transform Z: expected 3.04412, got \(transformed.z)")
}

func testJT128ExtrinsicsIsPlaceholder() {
    // JT128 offset is not yet calibrated — using identity placeholder
    let ex = RigSpec.jt128Extrinsics
    
    expect(ex.isIdentity, "JT128 extrinsics should be identity placeholder")
    expect(!RigSpec.jt128IsCalibrated, "jt128IsCalibrated should be false")
}

func testIdentityExtrinsicsPreservesPoint() {
    let ex = SensorExtrinsics.identity
    
    let point = Point3D(x: 1.5, y: -2.3, z: 4.7)
    let transformed = ex.transform(point)
    
    expect(transformed.x == point.x, "Identity should preserve X")
    expect(transformed.y == point.y, "Identity should preserve Y")
    expect(transformed.z == point.z, "Identity should preserve Z")
}

func testExtrinsicsWithRotation() {
    // 90° rotation around Z axis: (1, 0, 0) -> (0, 1, 0)
    // For column-major: column 0 = [cos, sin, 0], column 1 = [-sin, cos, 0], column 2 = [0, 0, 1]
    // cos(90°) = 0, sin(90°) = 1
    let rotation90Z: [Float] = [
        0, 1, 0,   // Column 0
        -1, 0, 0,  // Column 1
        0, 0, 1    // Column 2
    ]
    
    let ex = SensorExtrinsics(rotation: rotation90Z, translation: (0.1, 0.2, 0.3))
    
    // Point (1, 0, 0) rotated 90° CCW around Z becomes (0, 1, 0), then SUBTRACT translation
    // p_imu = R * p - t = (0, 1, 0) - (0.1, 0.2, 0.3) = (-0.1, 0.8, -0.3)
    let point = Point3D(x: 1, y: 0, z: 0)
    let transformed = ex.transform(point)
    
    let tolerance: Float = 0.0001
    expect(abs(transformed.x - (-0.1)) < tolerance,
           "Rotated point X should be ~-0.1, got \(transformed.x)")
    expect(abs(transformed.y - 0.8) < tolerance,
           "Rotated point Y should be ~0.8 (1 from rotation - 0.2 translation), got \(transformed.y)")
    expect(abs(transformed.z - (-0.3)) < tolerance,
           "Rotated point Z should be ~-0.3, got \(transformed.z)")
}

func testExtrinsicsWithTranslationOnly() {
    // p_imu = p_sensor - t
    let ex = SensorExtrinsics(rotation: nil, translation: (1.0, 2.0, 3.0))
    
    let point = Point3D(x: 10, y: 20, z: 30)
    let transformed = ex.transform(point)
    
    expect(transformed.x == 9.0, "X should be 10 - 1 = 9")
    expect(transformed.y == 18.0, "Y should be 20 - 2 = 18")
    expect(transformed.z == 27.0, "Z should be 30 - 3 = 27")
}

func testTransformFramePreservesMetadata() {
    let frame = PointCloudFrame(
        positions: [Point3D(x: 1, y: 2, z: 3)],
        intensities: [0.75],
        sensor: .mid360,
        timestamp: Date(timeIntervalSince1970: 1000),
        frameId: "test_frame"
    )
    
    let transformed = RigSpec.transformToRigFrame(frame)
    
    // Metadata should be preserved
    expect(transformed.intensities == frame.intensities,
           "Intensities should be preserved after transform")
    expect(transformed.sensor == .mid360,
           "Sensor ID should be preserved after transform")
    expect(transformed.frameId == "test_frame",
           "frameId should be preserved after transform")
    expect(transformed.timestamp == frame.timestamp,
           "timestamp should be preserved after transform")
}

func testMid360FrameTransformed() {
    // Mid-360 frame should have extrinsics applied: p_imu = p_lidar - offset
    let frame = PointCloudFrame(
        positions: [Point3D(x: 1.0, y: 2.0, z: 3.0)],
        intensities: [0.9],
        sensor: .mid360,
        frameId: "mid360_frame"
    )
    
    let transformed = RigSpec.transformToRigFrame(frame)
    
    let tolerance: Float = 1e-5
    expect(abs(transformed.positions[0].x - 0.989) < tolerance,
           "Mid-360 frame X: expected 0.989, got \(transformed.positions[0].x)")
    expect(abs(transformed.positions[0].y - 1.97671) < tolerance,
           "Mid-360 frame Y: expected 1.97671, got \(transformed.positions[0].y)")
    expect(abs(transformed.positions[0].z - 3.04412) < tolerance,
           "Mid-360 frame Z: expected 3.04412, got \(transformed.positions[0].z)")
    expect(transformed.frameId == "mid360_frame", "Mid-360 frameId should be preserved")
}

func testJT128FramePassesThrough() {
    // JT128 currently uses identity placeholder — passes through until calibration
    let frame = PointCloudFrame(
        positions: [Point3D(x: 5.0, y: 3.0, z: 1.0)],
        intensities: [0.9],
        sensor: .jt128,
        frameId: "jt128_frame"
    )
    
    let transformed = RigSpec.transformToRigFrame(frame)
    
    // With identity placeholder, JT128 passes through unchanged
    expect(transformed.positions[0].x == 5.0, "JT128 point X should be unchanged (placeholder)")
    expect(transformed.positions[0].y == 3.0, "JT128 point Y should be unchanged (placeholder)")
    expect(transformed.positions[0].z == 1.0, "JT128 point Z should be unchanged (placeholder)")
    expect(transformed.frameId == "jt128_frame", "JT128 frameId should be preserved")
}

func testExtrinsicsEquatable() {
    let ex1 = SensorExtrinsics(rotation: nil, translation: (1.0, 2.0, 3.0))
    let ex2 = SensorExtrinsics(rotation: nil, translation: (1.0, 2.0, 3.0))
    let ex3 = SensorExtrinsics(rotation: nil, translation: (1.0, 2.0, 4.0))
    
    expect(ex1 == ex2, "Identical extrinsics should be equal")
    expect(ex1 != ex3, "Different extrinsics should not be equal")
    expect(SensorExtrinsics.identity == SensorExtrinsics.identity, "Identity should equal itself")
}

func testExtrinsicsLookupBySensor() {
    // Lookup should return the correct extrinsics for each sensor
    let mid360 = RigSpec.extrinsics(for: .mid360)
    let jt128 = RigSpec.extrinsics(for: .jt128)
    
    // Mid-360 should have the known offset
    let tolerance: Float = 1e-5
    expect(abs(mid360.translation.x - 0.011) < tolerance,
           "Mid-360 lookup X: expected 0.011, got \(mid360.translation.x)")
    expect(abs(mid360.translation.y - 0.02329) < tolerance,
           "Mid-360 lookup Y: expected 0.02329, got \(mid360.translation.y)")
    expect(abs(mid360.translation.z - (-0.04412)) < tolerance,
           "Mid-360 lookup Z: expected -0.04412, got \(mid360.translation.z)")
    
    // JT128 should be identity (placeholder)
    expect(jt128.isIdentity, "JT128 lookup should return identity")
}

func runRigSpecTests() {
    print("RigSpec tests:")
    
    testMid360ExtrinsicsTransform()
    print("  Mid-360: (1,2,3) → (0.989, 1.97671, 3.04412)"); passed()
    
    testJT128ExtrinsicsIsPlaceholder()
    print("  JT128: identity placeholder, isCalibrated=false"); passed()
    
    testIdentityExtrinsicsPreservesPoint()
    print("  identity: preserves points exactly"); passed()
    
    testExtrinsicsWithRotation()
    print("  rotation: R·p−t combined correctly"); passed()
    
    testExtrinsicsWithTranslationOnly()
    print("  translation: p−t works correctly"); passed()
    
    testTransformFramePreservesMetadata()
    print("  frame: preserves metadata (frameId, timestamp, sensor)"); passed()
    
    testMid360FrameTransformed()
    print("  frame: Mid-360 extrinsics applied"); passed()
    
    testJT128FramePassesThrough()
    print("  frame: JT128 passes through (placeholder)"); passed()
    
    testExtrinsicsEquatable()
    print("  Equatable: hand-written == works"); passed()
    
    testExtrinsicsLookupBySensor()
    print("  lookup: returns correct extrinsics per sensor"); passed()
}
