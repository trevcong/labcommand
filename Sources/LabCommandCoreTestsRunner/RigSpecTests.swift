import Foundation
import LabCommandCore

/// Tests for RigSpec extrinsics and transformations.

func testMid360ExtrinsicsIsReference() {
    // Mid-360 IMU is the rig origin — extrinsics should be identity
    let ex = RigSpec.mid360Extrinsics
    expect(ex.isIdentity, "Mid-360 extrinsics should be identity (reference frame)")
}

func testJT128ExtrinsicsIsPlaceholder() {
    // JT128 offset is TBD — currently identity placeholder
    let ex = RigSpec.jt128Extrinsics
    
    expect(ex.isIdentity, "JT128 extrinsics should be identity placeholder (offset TBD)")
    expect(ex.translation.x == 0 && ex.translation.y == 0 && ex.translation.z == 0,
           "JT128 translation should be zero (placeholder)")
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
    
    // Point (1, 0, 0) rotated 90° CCW around Z becomes (0, 1, 0), then add translation
    let point = Point3D(x: 1, y: 0, z: 0)
    let transformed = ex.transform(point)
    
    let tolerance: Float = 0.0001
    expect(abs(transformed.x - 0.1) < tolerance,
           "Rotated point X should be ~0.1 (translation only), got \(transformed.x)")
    expect(abs(transformed.y - 1.2) < tolerance,
           "Rotated point Y should be ~1.2 (1 from rotation + 0.2 translation), got \(transformed.y)")
    expect(abs(transformed.z - 0.3) < tolerance,
           "Rotated point Z should be ~0.3 (translation only), got \(transformed.z)")
}

func testExtrinsicsWithTranslationOnly() {
    let ex = SensorExtrinsics(rotation: nil, translation: (1.0, 2.0, 3.0))
    
    let point = Point3D(x: 10, y: 20, z: 30)
    let transformed = ex.transform(point)
    
    expect(transformed.x == 11.0, "X should be offset by translation")
    expect(transformed.y == 22.0, "Y should be offset by translation")
    expect(transformed.z == 33.0, "Z should be offset by translation")
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
    
    // With identity placeholder, positions should be unchanged
    expect(transformed.positions[0].x == 1 && transformed.positions[0].y == 2 && transformed.positions[0].z == 3,
           "Position should pass through identity transform")
    
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

func testMid360FramePassesThrough() {
    // Mid-360 is the reference frame — should pass through unchanged
    let frame = PointCloudFrame(
        positions: [Point3D(x: 5.0, y: 3.0, z: 1.0)],
        intensities: [0.9],
        sensor: .mid360,
        frameId: "mid360_frame"
    )
    
    let transformed = RigSpec.transformToRigFrame(frame)
    
    expect(transformed.positions[0].x == 5.0, "Mid-360 point X should be unchanged (reference)")
    expect(transformed.positions[0].y == 3.0, "Mid-360 point Y should be unchanged (reference)")
    expect(transformed.positions[0].z == 1.0, "Mid-360 point Z should be unchanged (reference)")
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
    let mid360 = RigSpec.extrinsics(for: .mid360)
    let jt128 = RigSpec.extrinsics(for: .jt128)
    
    expect(mid360 == RigSpec.mid360Extrinsics, "Lookup for .mid360 should return mid360Extrinsics")
    expect(jt128 == RigSpec.jt128Extrinsics, "Lookup for .jt128 should return jt128Extrinsics")
}

func runRigSpecTests() {
    print("RigSpec tests:")
    
    testMid360ExtrinsicsIsReference()
    print("  extrinsics: Mid-360 is identity (reference frame)"); passed()
    
    testJT128ExtrinsicsIsPlaceholder()
    print("  extrinsics: JT128 is identity placeholder (offset TBD)"); passed()
    
    testIdentityExtrinsicsPreservesPoint()
    print("  extrinsics: identity preserves points"); passed()
    
    testExtrinsicsWithRotation()
    print("  extrinsics: rotation + translation combined correctly"); passed()
    
    testExtrinsicsWithTranslationOnly()
    print("  extrinsics: translation-only transform works"); passed()
    
    testTransformFramePreservesMetadata()
    print("  frame transform: preserves metadata including frameId"); passed()
    
    testMid360FramePassesThrough()
    print("  frame transform: Mid-360 passes through (reference frame)"); passed()
    
    testJT128FramePassesThrough()
    print("  frame transform: JT128 passes through (placeholder)"); passed()
    
    testExtrinsicsEquatable()
    print("  Equatable: hand-written == works correctly"); passed()
    
    testExtrinsicsLookupBySensor()
    print("  extrinsics lookup: by sensor returns correct values"); passed()
}
