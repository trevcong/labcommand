import Foundation
import LabCommandCore

// Static TFs of the dual rig. Expectations are worked out independently of
// RigidTransform's closed-form matrix: elementary rotations composed by hand.

private func near(_ a: Point3D, _ b: Point3D, tolerance: Float = 1e-5) -> Bool {
    abs(a.x - b.x) <= tolerance && abs(a.y - b.y) <= tolerance && abs(a.z - b.z) <= tolerance
}

private func rotX(_ p: (Double, Double, Double), _ a: Double) -> (Double, Double, Double) {
    (p.0, p.1 * cos(a) - p.2 * sin(a), p.1 * sin(a) + p.2 * cos(a))
}

private func rotY(_ p: (Double, Double, Double), _ a: Double) -> (Double, Double, Double) {
    (p.0 * cos(a) + p.2 * sin(a), p.1, -p.0 * sin(a) + p.2 * cos(a))
}

private func rotZ(_ p: (Double, Double, Double), _ a: Double) -> (Double, Double, Double) {
    (p.0 * cos(a) - p.1 * sin(a), p.0 * sin(a) + p.1 * cos(a), p.2)
}

/// R = Rz(yaw) · Ry(pitch) · Rx(roll): roll is applied to the point first.
private func referenceYPR(_ p: Point3D, yaw: Double, pitch: Double, roll: Double) -> Point3D {
    let r = rotZ(rotY(rotX((Double(p.x), Double(p.y), Double(p.z)), roll), pitch), yaw)
    return Point3D(x: Float(r.0), y: Float(r.1), z: Float(r.2))
}

private let probePoints: [Point3D] = [
    Point3D(x: 1, y: 0, z: 0),
    Point3D(x: 0, y: 1, z: 0),
    Point3D(x: 0, y: 0, z: 1),
    Point3D(x: 1.5, y: -2.25, z: 0.75),
    Point3D(x: -3, y: 4, z: -5),
]

func testRigidTransformIdentity() {
    let identity = RigidTransform.identity
    expect(identity.isIdentity, "identity reports isIdentity")
    for p in probePoints {
        expect(identity.apply(p) == p, "identity leaves \(p) untouched")
    }
    let zero = RigidTransform(translation: Point3D(), yaw: 0, pitch: 0, roll: 0)
    expect(zero == .identity, "zero yaw/pitch/roll and no translation is exactly identity")
    expect(RigTransforms.hesaiToWorld.isIdentity, "hesai_lidar → world is identity (world is the JT128 origin)")
    expect(RigTransforms.toWorld(.jt128).isIdentity, "JT128 maps through identity")
}

func testRigidTransformSingleAxisRotations() {
    let quarter = Double.pi / 2

    // Right-handed, counter-clockwise looking down the axis.
    let yaw = RigidTransform(translation: Point3D(), yaw: quarter, pitch: 0, roll: 0)
    expect(near(yaw.apply(Point3D(x: 1, y: 0, z: 0)), Point3D(x: 0, y: 1, z: 0)), "yaw +90°: x → y")
    expect(near(yaw.apply(Point3D(x: 0, y: 1, z: 0)), Point3D(x: -1, y: 0, z: 0)), "yaw +90°: y → -x")

    let pitch = RigidTransform(translation: Point3D(), yaw: 0, pitch: quarter, roll: 0)
    expect(near(pitch.apply(Point3D(x: 1, y: 0, z: 0)), Point3D(x: 0, y: 0, z: -1)), "pitch +90°: x → -z")
    expect(near(pitch.apply(Point3D(x: 0, y: 0, z: 1)), Point3D(x: 1, y: 0, z: 0)), "pitch +90°: z → x")

    let roll = RigidTransform(translation: Point3D(), yaw: 0, pitch: 0, roll: quarter)
    expect(near(roll.apply(Point3D(x: 0, y: 1, z: 0)), Point3D(x: 0, y: 0, z: 1)), "roll +90°: y → z")
    expect(near(roll.apply(Point3D(x: 0, y: 0, z: 1)), Point3D(x: 0, y: -1, z: 0)), "roll +90°: z → -y")
}

func testRigidTransformYawPitchRollOrder() {
    // Angles chosen so every wrong multiplication order gives a different answer.
    let yaw = 0.7, pitch = -0.4, roll = 1.1
    let offset = Point3D(x: 0.3, y: -0.2, z: 0.5)
    let transform = RigidTransform(translation: offset, yaw: yaw, pitch: pitch, roll: roll)

    for p in probePoints {
        let rotated = referenceYPR(p, yaw: yaw, pitch: pitch, roll: roll)
        expect(near(transform.rotate(p), rotated, tolerance: 1e-4), "R = Rz·Ry·Rx for \(p): got \(transform.rotate(p)), want \(rotated)")
        let moved = Point3D(x: rotated.x + offset.x, y: rotated.y + offset.y, z: rotated.z + offset.z)
        expect(near(transform.apply(p), moved, tolerance: 1e-4), "rotation then translation for \(p)")
    }

    // Proper rotation: orthonormal rows, determinant +1.
    let rows = [transform.r0, transform.r1, transform.r2]
    for i in 0..<3 {
        for j in 0..<3 {
            let dot = rows[i].x * rows[j].x + rows[i].y * rows[j].y + rows[i].z * rows[j].z
            expect(abs(dot - (i == j ? 1 : 0)) < 1e-5, "R·Rᵀ = I at (\(i),\(j)): \(dot)")
        }
    }
    let (a, b, c) = (transform.r0, transform.r1, transform.r2)
    let det = a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x)
    expect(abs(det - 1) < 1e-5, "det R = +1, got \(det)")
}

func testLivoxToWorldStaticTF() {
    let tf = RigTransforms.livoxToWorld

    expect(RigTransforms.livoxTranslation == Point3D(x: 0, y: 0, z: -0.110), "livox_frame sits 110 mm below the JT128")
    expect(RigTransforms.livoxYawDegrees == -86.5, "livox yaw is -86.5°")
    expect(RigTransforms.livoxPitch == 0 && RigTransforms.livoxRoll == .pi, "livox pitch 0, roll π (mounted inverted)")
    expect(RigTransforms.toWorld(.mid360) == tf, "Mid-360 maps through livox_frame → world")
    expect(!tf.isIdentity, "livox → world is not identity")

    // cos(-86.5°) = 0.0610485, sin(-86.5°) = -0.9981348. With roll = π the
    // sensor's y and z flip before the yaw.
    let c: Float = 0.0610485
    let s: Float = -0.9981348
    expect(near(tf.rotate(Point3D(x: 1, y: 0, z: 0)), Point3D(x: c, y: s, z: 0)), "livox +x → (cos, sin, 0): \(tf.rotate(Point3D(x: 1, y: 0, z: 0)))")
    expect(near(tf.rotate(Point3D(x: 0, y: 1, z: 0)), Point3D(x: s, y: -c, z: 0)), "livox +y → (sin, -cos, 0): \(tf.rotate(Point3D(x: 0, y: 1, z: 0)))")
    expect(near(tf.rotate(Point3D(x: 0, y: 0, z: 1)), Point3D(x: 0, y: 0, z: -1)), "livox +z points down in world")

    expect(near(tf.apply(Point3D()), Point3D(x: 0, y: 0, z: -0.110)), "livox origin → (0, 0, -0.110)")
    expect(near(tf.apply(Point3D(x: 0, y: 0, z: 1)), Point3D(x: 0, y: 0, z: -1.110)), "1 m up the Mid-360 axis is 1.11 m below world origin")
    expect(
        near(tf.apply(Point3D(x: 2, y: 1, z: 0.5)), Point3D(x: 2 * c + s, y: 2 * s - c, z: -0.610)),
        "worked point (2, 1, 0.5): \(tf.apply(Point3D(x: 2, y: 1, z: 0.5)))"
    )

    for p in probePoints {
        let want = referenceYPR(p, yaw: -86.5 * .pi / 180, pitch: 0, roll: .pi)
        expect(
            near(tf.apply(p), Point3D(x: want.x, y: want.y, z: want.z - 0.110), tolerance: 1e-4),
            "livox → world matches Rz(-86.5°)·Ry(0)·Rx(π) + t for \(p)"
        )
    }
}

func testInWorldRetagsFrames() {
    let stamp = Date(timeIntervalSince1970: 1_700_000_000)
    let cloud = PointCloudFrame(
        positions: [Point3D(x: 1, y: 0, z: 0), Point3D(x: 0, y: 0, z: 2)],
        intensities: [10, 20],
        timestamp: stamp,
        frameId: "livox_frame"
    )

    let mid = RigTransforms.inWorld(cloud, sensor: .mid360)
    expect(mid.frameId == "world" && mid.sensor == .mid360, "Mid-360 cloud is relabelled world / mid360")
    expect(mid.count == 2 && mid.intensities == [10, 20] && mid.timestamp == stamp, "intensities and stamp ride along")
    expect(near(mid.positions[0], RigTransforms.livoxToWorld.apply(cloud.positions[0])), "point 0 transformed")
    expect(near(mid.positions[1], Point3D(x: 0, y: 0, z: -2.110)), "point 1: 2 m up the sensor is 2.11 m down in world")
    expect(cloud.positions[0] == Point3D(x: 1, y: 0, z: 0), "the source frame is not mutated")

    var hesai = cloud
    hesai.frameId = "hesai_lidar"
    let jt = RigTransforms.inWorld(hesai, sensor: .jt128)
    expect(jt.positions == cloud.positions, "JT128 points are already in world")
    expect(jt.frameId == "world" && jt.sensor == .jt128, "JT128 cloud is relabelled world / jt128")

    expect(RigTransforms.sensorFrame(.mid360) == "livox_frame", "Mid-360 frame id")
    expect(RigTransforms.sensorFrame(.jt128) == "hesai_lidar", "JT128 frame id")
    expect(RigTransforms.inWorld(PointCloudFrame(), sensor: .mid360).isEmpty, "empty cloud stays empty")
}

func runRigTransformsTests() {
    testRigidTransformIdentity()
    print("  RigTransforms: identity, and hesai_lidar → world is identity"); passed()
    testRigidTransformSingleAxisRotations()
    print("  RigTransforms: yaw/pitch/roll are right-handed about z/y/x"); passed()
    testRigidTransformYawPitchRollOrder()
    print("  RigTransforms: R = Rz(yaw)·Ry(pitch)·Rx(roll), orthonormal, det +1"); passed()
    testLivoxToWorldStaticTF()
    print("  RigTransforms: livox_frame → world (yaw -86.5°, roll π, z -0.110)"); passed()
    testInWorldRetagsFrames()
    print("  RigTransforms: clouds land in world with sensor, intensities and stamp intact"); passed()
}
