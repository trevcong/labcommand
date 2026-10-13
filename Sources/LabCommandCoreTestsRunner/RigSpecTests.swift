import Foundation
import LabCommandCore

/// Tests for RigSpec extrinsics and transformations.

func testMid360ExtrinsicsValues() {
    let ex = RigSpec.mid360Extrinsics
    
    // Verify translation matches spec: (11.0, 23.29, −44.12) mm
    let tolerance: Float = 0.00001
    expect(abs(ex.translation.x - 0.011) < tolerance,
           "Mid-360 X translation should be 0.011m, got \(ex.translation.x)")
    expect(abs(ex.translation.y - 0.02329) < tolerance,
           "Mid-360 Y translation should be 0.02329m, got \(ex.translation.y)")
    expect(abs(ex.translation.z - (-0.04412)) < tolerance,
           "Mid-360 Z translation should be -0.04412m, got \(ex.translation.z)")
}

func testJT128ExtrinsicsIsIdentity() {
    let ex = RigSpec.jt128Extrinsics
    
    expect(ex.isIdentity, "JT128 extrinsics should be identity transform")
    expect(ex.translation.x == 0 && ex.translation.y == 0 && ex.translation.z == 0,
           "JT128 translation should be zero")
}

func testExtrinsicsTransformPoint() {
    let ex = RigSpec.mid360Extrinsics
    
    // Transform origin point
    let origin = Point3D(x: 0, y: 0, z: 0)
    let transformed = ex.transform(origin)
    
    // Origin should map to the translation offset
    let tolerance: Float = 0.00001
    expect(abs(transformed.x - 0.011) < tolerance,
           "Transformed origin X should equal translation X")
    expect(abs(transformed.y - 0.02329) < tolerance,
           "Transformed origin Y should equal translation Y")
    expect(abs(transformed.z - (-0.04412)) < tolerance,
           "Transformed origin Z should equal translation Z")
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
    let rotation90Z: [Float] = [
        0, 1, 0,   // Column 0
        -1, 0, 0,  // Column 1
        0, 0, 1    // Column 2
    ]
    
    let ex = SensorExtrinsics(rotation: rotation90Z, translation: (0.1, 0.2, 0.3))
    
    let point = Point3D(x: 1, y: 0, z: 0)
    let transformed = ex.transform(point)
    
    let tolerance: Float = 0.0001
    expect(abs(transformed.x - 0.1) < tolerance,
           "Rotated point X should be ~0.1 (translation only), got \(transformed.x)")
    expect(abs(transformed.y - 1.2) < tolerance,
           "Rotated point Y should be ~1.2, got \(transformed.y)")
    expect(abs(transformed.z - 0.3) < tolerance,
           "Rotated point Z should be ~0.3, got \(transformed.z)")
}

func testTransformFrameAppliesExtrinsics() {
    // Create a simple Mid-360 frame
    let frame = PointCloudFrame(
        positions: [Point3D(x: 0, y: 0, z: 0), Point3D(x: 1, y: 0, z: 0)],
        intensities: [0.5, 0.8],
        sensor: .mid360
    )
    
    let transformed = RigSpec.transformToRigFrame(frame)
    
    // Check first point (origin) is at Mid-360 offset
    let tolerance: Float = 0.00001
    expect(abs(transformed.positions[0].x - 0.011) < tolerance,
           "Transformed origin X should equal Mid-360 offset")
    
    // Check second point is offset by same amount
    expect(abs(transformed.positions[1].x - 1.011) < tolerance,
           "Transformed point X should be original + offset")
    
    // Intensities should be preserved
    expect(transformed.intensities == frame.intensities,
           "Intensities should be preserved after transform")
    
    // Sensor ID should be preserved
    expect(transformed.sensor == .mid360,
           "Sensor ID should be preserved after transform")
}

func testJT128FramePassesThrough() {
    let frame = PointCloudFrame(
        positions: [Point3D(x: 5.0, y: 3.0, z: 1.0)],
        intensities: [0.9],
        sensor: .jt128
    )
    
    let transformed = RigSpec.transformToRigFrame(frame)
    
    // JT128 should pass through unchanged (identity transform)
    expect(transformed.positions[0].x == 5.0, "JT128 point X should be unchanged")
    expect(transformed.positions[0].y == 3.0, "JT128 point Y should be unchanged")
    expect(transformed.positions[0].z == 1.0, "JT128 point Z should be unchanged")
}

func testSensorColors() {
    let mid360Color = RigSpec.color(for: .mid360)
    let jt128Color = RigSpec.color(for: .jt128)
    
    // Mid-360 should be cyan (0, 1, 1)
    expect(mid360Color.r == 0.0 && mid360Color.g == 1.0 && mid360Color.b == 1.0,
           "Mid-360 color should be cyan")
    
    // JT128 should be magenta (1, 0, 1)
    expect(jt128Color.r == 1.0 && jt128Color.g == 0.0 && jt128Color.b == 1.0,
           "JT128 color should be magenta")
}

func testSensorIDForGPU() {
    expect(RigSpec.sensorIDForGPU(.mid360) == 1, "Mid-360 GPU ID should be 1")
    expect(RigSpec.sensorIDForGPU(.jt128) == 2, "JT128 GPU ID should be 2")
    expect(RigSpec.sensorIDForGPU(nil) == 0, "nil sensor GPU ID should be 0")
}

func runRigSpecTests() {
    print("RigSpec tests:")
    
    testMid360ExtrinsicsValues()
    print("  extrinsics: Mid-360 translation values match spec"); passed()
    
    testJT128ExtrinsicsIsIdentity()
    print("  extrinsics: JT128 is identity transform"); passed()
    
    testExtrinsicsTransformPoint()
    print("  extrinsics: transform applies translation correctly"); passed()
    
    testIdentityExtrinsicsPreservesPoint()
    print("  extrinsics: identity preserves points"); passed()
    
    testExtrinsicsWithRotation()
    print("  extrinsics: rotation + translation combined correctly"); passed()
    
    testTransformFrameAppliesExtrinsics()
    print("  frame transform: applies extrinsics to point cloud frame"); passed()
    
    testJT128FramePassesThrough()
    print("  frame transform: JT128 passes through unchanged"); passed()
    
    testSensorColors()
    print("  sensor colors: Mid-360=cyan, JT128=magenta"); passed()
    
    testSensorIDForGPU()
    print("  sensor IDs: GPU IDs map correctly"); passed()
}
