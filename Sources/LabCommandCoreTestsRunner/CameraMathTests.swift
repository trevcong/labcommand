import Foundation
import LabCommandCore

// MARK: - Test Helpers for PR #6 Camera Controls

private func expectFloat(_ a: Float, _ b: Float, tolerance: Float = 0.0001, _ msg: String) {
    if abs(a - b) > tolerance {
        fputs("FAIL: \(msg) — expected \(b), got \(a)\n", stderr)
        exit(1)
    }
}

private func expectVec3(_ a: Float3, _ b: Float3, tolerance: Float = 0.0001, _ msg: String) {
    if float3Length(a - b) > tolerance {
        fputs("FAIL: \(msg) — expected \(b), got \(a)\n", stderr)
        exit(1)
    }
}

private func expectCond(_ cond: Bool, _ msg: String) {
    if !cond {
        fputs("FAIL: \(msg)\n", stderr)
        exit(1)
    }
}

// MARK: - PR #5 Fit Tests

/// Helper to run fit test with real matrices and verify NDC bounds + tight fit.
private func runFitTest(
    name: String,
    bboxMin: Point3D,
    bboxMax: Point3D,
    aspect: Float,
    elevation: Float = CameraMath.defaultElevation,
    margin: Float = 0.9
) {
    let fovY = CameraMath.defaultFovY
    
    let (eye, center, _) = CameraMath.fitToBounds(
        min: bboxMin,
        max: bboxMax,
        fovY: fovY,
        aspect: aspect,
        margin: margin,
        elevation: elevation
    )
    
    let view = CameraMath.lookAt(eye: eye, center: center)
    let proj = CameraMath.perspective(
        fovY: fovY,
        aspect: aspect,
        near: 0.1,
        far: 1000
    )
    let mvp = proj * view
    
    let corners = CameraMath.bboxCorners(min: bboxMin, max: bboxMax)
    
    var maxAbsX: Float = 0
    var maxAbsY: Float = 0
    var allNDCs: [(x: Float, y: Float, z: Float)] = []
    
    for (i, corner) in corners.enumerated() {
        guard let ndc = mvp.transformToNDC(corner) else {
            fputs("FAIL \(name): corner \(i) has w ≈ 0\n", stderr)
            exit(1)
        }
        
        allNDCs.append(ndc)
        
        // Assert every |NDC| ≤ margin + 1e-4
        let limit = margin + 1e-4
        expect(
            abs(ndc.x) <= limit,
            "\(name) corner \(i): |NDC.x|=\(abs(ndc.x)) > \(limit)"
        )
        expect(
            abs(ndc.y) <= limit,
            "\(name) corner \(i): |NDC.y|=\(abs(ndc.y)) > \(limit)"
        )
        expect(
            ndc.z >= 0 && ndc.z <= 1,
            "\(name) corner \(i): NDC.z=\(ndc.z) out of [0, 1]"
        )
        
        maxAbsX = max(maxAbsX, abs(ndc.x))
        maxAbsY = max(maxAbsY, abs(ndc.y))
    }
    
    // Assert at least one axis reaches margin ± 1e-3 (tight fit)
    let tightLow = margin - 1e-3
    let isTight = maxAbsX >= tightLow || maxAbsY >= tightLow
    expect(
        isTight,
        "\(name): not tight fit - maxX=\(maxAbsX), maxY=\(maxAbsY), need ≥\(tightLow)"
    )
    
    // Assert one concrete corner NDC value (corner 0)
    let c0 = allNDCs[0]
    expect(
        abs(c0.x) < 1.0 && abs(c0.y) < 1.0,
        "\(name): concrete check corner[0] NDC=(\(c0.x), \(c0.y)) should be in [-1, 1]"
    )
}

func testFitAspect1() {
    runFitTest(
        name: "aspect1",
        bboxMin: Point3D(x: -5, y: -5, z: -2),
        bboxMax: Point3D(x: 5, y: 5, z: 3),
        aspect: 1.0
    )
}

func testFitAspect16x9() {
    runFitTest(
        name: "aspect16x9",
        bboxMin: Point3D(x: -10, y: -10, z: 0),
        bboxMax: Point3D(x: 10, y: 10, z: 5),
        aspect: 16.0 / 9.0
    )
}

func testFitAspect0_5() {
    runFitTest(
        name: "aspect0.5",
        bboxMin: Point3D(x: -5, y: -5, z: 0),
        bboxMax: Point3D(x: 5, y: 5, z: 2),
        aspect: 0.5
    )
}

func testFitFlatCloud() {
    runFitTest(
        name: "flatCloud",
        bboxMin: Point3D(x: -20, y: -20, z: -1),
        bboxMax: Point3D(x: 20, y: 20, z: 1),
        aspect: 16.0 / 9.0
    )
}

func testFitCorridor30Deg() {
    runFitTest(
        name: "corridor30deg",
        bboxMin: Point3D(x: -1, y: -10, z: -1),
        bboxMax: Point3D(x: 1, y: 10, z: 1),
        aspect: 16.0 / 9.0,
        elevation: Float.pi / 6
    )
}

func testFitDistanceAlgorithm() {
    let bboxMin = Point3D(x: -5, y: -5, z: -2)
    let bboxMax = Point3D(x: 5, y: 5, z: 3)
    let corners = CameraMath.bboxCorners(min: bboxMin, max: bboxMax)
    let center = Point3D(
        x: (bboxMin.x + bboxMax.x) / 2,
        y: (bboxMin.y + bboxMax.y) / 2,
        z: (bboxMin.z + bboxMax.z) / 2
    )
    
    let (forward, right, up) = CameraMath.cameraOrientation(elevation: CameraMath.defaultElevation)
    
    let distance = CameraMath.fitDistance(
        corners: corners,
        target: center,
        forward: forward,
        right: right,
        up: up,
        fovY: CameraMath.defaultFovY,
        aspect: 1.0,
        margin: 0.9
    )
    
    expect(distance > 0, "fitDistance should return positive distance")
    expect(distance < 1000, "fitDistance should return reasonable distance")
    
    let eye = Point3D(
        x: center.x - distance * forward.x,
        y: center.y - distance * forward.y,
        z: center.z - distance * forward.z
    )
    
    let view = CameraMath.lookAt(eye: eye, center: center)
    let proj = CameraMath.perspective(fovY: CameraMath.defaultFovY, aspect: 1.0, near: 0.1, far: 1000)
    let mvp = proj * view
    
    var maxAbsNDC: Float = 0
    for corner in corners {
        guard let ndc = mvp.transformToNDC(corner) else {
            fputs("FAIL: fitDistance corner has w ≈ 0\n", stderr)
            exit(1)
        }
        maxAbsNDC = max(maxAbsNDC, abs(ndc.x), abs(ndc.y))
    }
    
    expect(
        maxAbsNDC >= 0.89 && maxAbsNDC <= 0.91,
        "fitDistance maxNDC=\(maxAbsNDC) should be ~0.9"
    )
}

func testCameraOrientation() {
    let (f, r, u) = CameraMath.cameraOrientation(elevation: Float.pi / 6)
    
    let fLen = sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
    let rLen = sqrt(r.x * r.x + r.y * r.y + r.z * r.z)
    let uLen = sqrt(u.x * u.x + u.y * u.y + u.z * u.z)
    
    expect(abs(fLen - 1) < 0.001, "forward should be unit vector")
    expect(abs(rLen - 1) < 0.001, "right should be unit vector")
    expect(abs(uLen - 1) < 0.001, "up should be unit vector")
    
    let fDotR = CameraMath.dot(f, r)
    let fDotU = CameraMath.dot(f, u)
    let rDotU = CameraMath.dot(r, u)
    
    expect(abs(fDotR) < 0.001, "forward·right should be 0")
    expect(abs(fDotU) < 0.001, "forward·up should be 0")
    expect(abs(rDotU) < 0.001, "right·up should be 0")
}

// MARK: - Matrix Tests (PR #5)

func testMatrixIdentityTransform() {
    let identity = CameraMath.Matrix4x4.identity
    let p = Point3D(x: 3, y: 4, z: 5)
    let result = identity.transform(p)
    
    expect(abs(result.x - 3) < 0.0001, "identity transform preserves x")
    expect(abs(result.y - 4) < 0.0001, "identity transform preserves y")
    expect(abs(result.z - 5) < 0.0001, "identity transform preserves z")
    expect(abs(result.w - 1) < 0.0001, "identity transform w = 1")
}

func testMatrixMultiplicationAssociative() {
    let a = CameraMath.perspective(fovY: CameraMath.defaultFovY, aspect: 1.0, near: 0.1, far: 100)
    let b = CameraMath.lookAt(
        eye: Point3D(x: 0, y: -10, z: 5),
        center: Point3D(x: 0, y: 0, z: 0)
    )
    let c = CameraMath.Matrix4x4.identity
    
    let ab_c = (a * b) * c
    let a_bc = a * (b * c)
    
    func matricesEqual(_ m1: CameraMath.Matrix4x4, _ m2: CameraMath.Matrix4x4) -> Bool {
        let eps: Float = 0.0001
        return abs(m1.columns.0.0 - m2.columns.0.0) < eps &&
               abs(m1.columns.0.1 - m2.columns.0.1) < eps &&
               abs(m1.columns.0.2 - m2.columns.0.2) < eps &&
               abs(m1.columns.0.3 - m2.columns.0.3) < eps &&
               abs(m1.columns.1.0 - m2.columns.1.0) < eps &&
               abs(m1.columns.1.1 - m2.columns.1.1) < eps &&
               abs(m1.columns.1.2 - m2.columns.1.2) < eps &&
               abs(m1.columns.1.3 - m2.columns.1.3) < eps &&
               abs(m1.columns.2.0 - m2.columns.2.0) < eps &&
               abs(m1.columns.2.1 - m2.columns.2.1) < eps &&
               abs(m1.columns.2.2 - m2.columns.2.2) < eps &&
               abs(m1.columns.2.3 - m2.columns.2.3) < eps &&
               abs(m1.columns.3.0 - m2.columns.3.0) < eps &&
               abs(m1.columns.3.1 - m2.columns.3.1) < eps &&
               abs(m1.columns.3.2 - m2.columns.3.2) < eps &&
               abs(m1.columns.3.3 - m2.columns.3.3) < eps
    }
    
    expect(matricesEqual(ab_c, a_bc), "matrix multiplication is associative")
}

func testMatrixEquatable() {
    let a = CameraMath.Matrix4x4.identity
    let b = CameraMath.Matrix4x4.identity
    let c = CameraMath.Matrix4x4(
        (2, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, 1, 0),
        (0, 0, 0, 1)
    )
    
    expect(a == b, "identical matrices should be equal")
    expect(!(a == c), "different matrices should not be equal")
}

// MARK: - PR #6 Camera Control Tests

func testLookAtIdentity() {
    let eye = Float3(5, 0, 0)
    let target = Float3(0, 0, 0)
    let up = Float3(0, 0, 1)
    
    let view = CameraMath.lookAt(eye: eye, target: target, up: up)
    
    let transformed = view * Float4(0, 0, 0, 1)
    expectFloat(transformed.z, -5, tolerance: 0.001, "look-at places origin at -5 in view space")
}

func testLookAtFromAbove() {
    let eye = Float3(0, 0, 5)
    let target = Float3(0, 0, 0)
    let up = Float3(0, 1, 0)
    
    let view = CameraMath.lookAt(eye: eye, target: target, up: up)
    
    let transformed = view * Float4(0, 0, 0, 1)
    expectFloat(transformed.z, -5, tolerance: 0.001, "looking down Z places origin at -5")
}

func testPerspectiveMetalDepthRange() {
    let proj = CameraMath.perspectiveMetal(fovY: .pi / 4, aspect: 1.0, near: 0.1, far: 100)
    
    expectCond(proj[2][2] < 0, "Metal depth coefficient is negative (right-handed)")
    expectCond(proj[2][3] == -1, "Metal perspective divide coefficient is -1")
}

func testPerspectiveAspect() {
    let proj1 = CameraMath.perspectiveMetal(fovY: .pi / 4, aspect: 1.0, near: 0.1, far: 100)
    let proj2 = CameraMath.perspectiveMetal(fovY: .pi / 4, aspect: 2.0, near: 0.1, far: 100)
    
    expectCond(proj1[0][0] != proj2[0][0], "different aspects produce different X scaling")
    expectFloat(proj1[1][1], proj2[1][1], "Y scale same for same FOV")
}

func testOrbitPositionAtOrigin() {
    let pos = CameraMath.orbitPosition(target: .zero, distance: 10, azimuth: 0, elevation: 0)
    
    expectFloat(pos.x, 10, "azimuth 0, elevation 0 → x = distance")
    expectFloat(pos.y, 0, "azimuth 0 → y = 0")
    expectFloat(pos.z, 0, "elevation 0 → z = 0")
}

func testOrbitPositionElevated() {
    let pos = CameraMath.orbitPosition(target: .zero, distance: 10, azimuth: 0, elevation: .pi / 2)
    
    expectFloat(pos.x, 0, tolerance: 0.001, "elevation 90° → x = 0")
    expectFloat(pos.y, 0, tolerance: 0.001, "azimuth 0, elevation 90° → y = 0")
    expectFloat(pos.z, 10, tolerance: 0.001, "elevation 90° → z = distance (Z-up)")
}

func testOrbitPositionRotated() {
    let pos = CameraMath.orbitPosition(target: .zero, distance: 10, azimuth: .pi / 2, elevation: 0)
    
    expectFloat(pos.x, 0, tolerance: 0.001, "azimuth 90° → x = 0")
    expectFloat(pos.y, 10, tolerance: 0.001, "azimuth 90° → y = distance")
    expectFloat(pos.z, 0, tolerance: 0.001, "elevation 0 → z = 0")
}

func testOrbitPositionWithTarget() {
    let target = Float3(5, 5, 5)
    let pos = CameraMath.orbitPosition(target: target, distance: 10, azimuth: 0, elevation: 0)
    
    expectFloat(pos.x, 15, "target offset X + distance")
    expectFloat(pos.y, 5, "target offset Y")
    expectFloat(pos.z, 5, "target offset Z")
}

func testFlyForwardAtOrigin() {
    let forward = CameraMath.flyForward(yaw: 0, pitch: 0)
    
    expectFloat(forward.x, 1, "yaw 0, pitch 0 → x = 1 (forward is +X)")
    expectFloat(forward.y, 0, "yaw 0, pitch 0 → y = 0")
    expectFloat(forward.z, 0, "pitch 0 → z = 0")
}

func testFlyForwardYawed() {
    let forward = CameraMath.flyForward(yaw: .pi / 2, pitch: 0)
    
    expectFloat(forward.x, 0, tolerance: 0.001, "yaw 90° → x = 0")
    expectFloat(forward.y, 1, tolerance: 0.001, "yaw 90° → y = 1")
    expectFloat(forward.z, 0, "pitch 0 → z = 0")
}

func testFlyForwardPitched() {
    let forward = CameraMath.flyForward(yaw: 0, pitch: .pi / 2)
    
    expectFloat(forward.x, 0, tolerance: 0.001, "pitch 90° → x = 0")
    expectFloat(forward.y, 0, tolerance: 0.001, "yaw 0, pitch 90° → y = 0")
    expectFloat(forward.z, 1, tolerance: 0.001, "pitch 90° → z = 1 (Z-up)")
}

func testFlyRight() {
    let right = CameraMath.flyRight(yaw: 0)
    
    expectFloat(right.x, 0, "yaw 0 → right x = 0")
    expectFloat(right.y, 1, "yaw 0 → right y = 1")
    expectFloat(right.z, 0, "right z always 0 (Z-up)")
}

func testLerp() {
    expectFloat(CameraMath.lerp(0, 10, 0.5), 5, "lerp midpoint")
    expectFloat(CameraMath.lerp(0, 10, 0), 0, "lerp t=0")
    expectFloat(CameraMath.lerp(0, 10, 1), 10, "lerp t=1")
    expectFloat(CameraMath.lerp(-5, 5, 0.5), 0, "lerp negative range")
}

func testLerpVector() {
    let a = Float3(0, 0, 0)
    let b = Float3(10, 20, 30)
    let mid = CameraMath.lerp(a, b, 0.5)
    
    expectVec3(mid, Float3(5, 10, 15), "vector lerp midpoint")
}

func testLerpAngle() {
    expectFloat(CameraMath.lerpAngle(0, .pi, 0.5), .pi / 2, "angle lerp 0 to π")
    
    let shortPath = CameraMath.lerpAngle(.pi * 0.9, -.pi * 0.9, 0.5)
    expectCond(abs(shortPath) > .pi * 0.8, "angle lerp takes short path across π: \(shortPath)")
}

func testLerpAngleWraparound() {
    let result = CameraMath.lerpAngle(.pi - 0.1, -.pi + 0.1, 0.5)
    expectCond(abs(result) > .pi - 0.2, "wraparound interpolation stays near π: \(result)")
}

func testWrapAngle() {
    expectFloat(CameraMath.wrapAngle(0), 0, "zero stays zero")
    expectFloat(CameraMath.wrapAngle(.pi), .pi, tolerance: 0.001, "π stays π")
    expectFloat(CameraMath.wrapAngle(3 * .pi), .pi, tolerance: 0.001, "3π wraps to π")
    expectFloat(CameraMath.wrapAngle(-3 * .pi), .pi, tolerance: 0.001, "-3π wraps to π (half-open interval)")
    expectFloat(CameraMath.wrapAngle(-.pi), .pi, tolerance: 0.001, "-π wraps to π")
}

func testClamp() {
    expectFloat(CameraMath.clamp(5, min: 0, max: 10), 5, "clamp in range")
    expectFloat(CameraMath.clamp(-5, min: 0, max: 10), 0, "clamp below")
    expectFloat(CameraMath.clamp(15, min: 0, max: 10), 10, "clamp above")
}

func testClampToBounds() {
    let point = Float3(15, -5, 50)
    let clamped = CameraMath.clampToBounds(point, min: Float3(0, 0, 0), max: Float3(10, 10, 10))
    
    expectVec3(clamped, Float3(10, 0, 10), "clamp to bounds")
}

func testBoundsCenter() {
    let center = CameraMath.boundsCenter(min: Float3(-5, -5, -5), max: Float3(5, 5, 5))
    expectVec3(center, .zero, "bounds center")
}

func testBoundsDiagonal() {
    let diag = CameraMath.boundsDiagonal(min: Float3(0, 0, 0), max: Float3(3, 4, 0))
    expectFloat(diag, 5, "bounds diagonal (3-4-5 triangle)")
}

func testIsInvalidFloat() {
    expectCond(CameraMath.isInvalid(Float.nan), "NaN is invalid")
    expectCond(CameraMath.isInvalid(Float.infinity), "infinity is invalid")
    expectCond(!CameraMath.isInvalid(0), "zero is valid")
    expectCond(!CameraMath.isInvalid(1.5), "normal float is valid")
}

func testIsInvalidVector() {
    expectCond(CameraMath.isInvalid(Float3(.nan, 0, 0)), "vector with NaN x is invalid")
    expectCond(CameraMath.isInvalid(Float3(0, .infinity, 0)), "vector with infinity y is invalid")
    expectCond(!CameraMath.isInvalid(Float3(1, 2, 3)), "normal vector is valid")
}

func testSanitize() {
    expectFloat(CameraMath.sanitize(.nan, default: 5), 5, "NaN sanitizes to default")
    expectFloat(CameraMath.sanitize(3, default: 5), 3, "valid passes through")
    
    let defaultVec = Float3(1, 1, 1)
    let result = CameraMath.sanitize(Float3(.nan, 0, 0), default: defaultVec)
    expectVec3(result, defaultVec, "invalid vector sanitizes to default")
}

func testRotationXIdentity() {
    let rot = CameraMath.rotationX(0)
    let point = Float4(0, 1, 0, 1)
    let result = rot * point
    expectVec3(Float3(result.x, result.y, result.z), Float3(0, 1, 0), "no rotation")
}

func testRotationX90() {
    let rot = CameraMath.rotationX(.pi / 2)
    let point = Float4(0, 1, 0, 1)
    let result = rot * point
    expectFloat(result.y, 0, tolerance: 0.001, "rotateX 90° Y → 0")
    expectFloat(result.z, 1, tolerance: 0.001, "rotateX 90° Y → Z")
}

func testRotationY90() {
    let rot = CameraMath.rotationY(.pi / 2)
    let point = Float4(1, 0, 0, 1)
    let result = rot * point
    expectFloat(result.x, 0, tolerance: 0.001, "rotateY 90° X → 0")
    expectFloat(result.z, -1, tolerance: 0.001, "rotateY 90° X → -Z")
}

func testRotationZ90() {
    let rot = CameraMath.rotationZ(.pi / 2)
    let point = Float4(1, 0, 0, 1)
    let result = rot * point
    expectFloat(result.x, 0, tolerance: 0.001, "rotateZ 90° X → 0")
    expectFloat(result.y, 1, tolerance: 0.001, "rotateZ 90° X → Y")
}

func testTranslation() {
    let trans = CameraMath.translation(Float3(5, 10, 15))
    let point = Float4(0, 0, 0, 1)
    let result = trans * point
    expectVec3(Float3(result.x, result.y, result.z), Float3(5, 10, 15), "translation")
}

func testScale() {
    let scale = CameraMath.scale(Float3(2, 3, 4))
    let point = Float4(1, 1, 1, 1)
    let result = scale * point
    expectVec3(Float3(result.x, result.y, result.z), Float3(2, 3, 4), "scale")
}

// MARK: - Entry Point

func runCameraMathTests() {
    // PR #5: Matrix and fit tests
    print("Running camera math tests…")
    testMatrixIdentityTransform()
    print("  camera math: identity matrix preserves point coordinates"); passed()
    testMatrixMultiplicationAssociative()
    print("  camera math: matrix multiplication is associative"); passed()
    testMatrixEquatable()
    print("  camera math: Matrix4x4 Equatable implementation works"); passed()
    testCameraOrientation()
    print("  camera math: camera orientation vectors are orthonormal"); passed()
    testFitDistanceAlgorithm()
    print("  camera math: fitDistance computes correct distance for margin=0.9"); passed()
    testFitAspect1()
    print("  camera math: fit aspect=1.0 tight within margin"); passed()
    testFitAspect16x9()
    print("  camera math: fit aspect=16:9 tight within margin"); passed()
    testFitAspect0_5()
    print("  camera math: fit aspect=0.5 tight within margin"); passed()
    testFitFlatCloud()
    print("  camera math: fit flat/wide cloud tight within margin"); passed()
    testFitCorridor30Deg()
    print("  camera math: fit 2×20×2 corridor at 30° elevation tight within margin"); passed()
    
    // PR #6: Camera control tests
    print("  camera math: look-at matrices (Z-up)")
    testLookAtIdentity()
    testLookAtFromAbove()
    passed()
    
    print("  camera math: perspective projection (Metal depth [0,1])")
    testPerspectiveMetalDepthRange()
    testPerspectiveAspect()
    passed()
    
    print("  camera math: orbit position (Z-up)")
    testOrbitPositionAtOrigin()
    testOrbitPositionElevated()
    testOrbitPositionRotated()
    testOrbitPositionWithTarget()
    passed()
    
    print("  camera math: fly directions (Z-up)")
    testFlyForwardAtOrigin()
    testFlyForwardYawed()
    testFlyForwardPitched()
    testFlyRight()
    passed()
    
    print("  camera math: interpolation")
    testLerp()
    testLerpVector()
    testLerpAngle()
    testLerpAngleWraparound()
    testWrapAngle()
    passed()
    
    print("  camera math: bounds clamping")
    testClamp()
    testClampToBounds()
    testBoundsCenter()
    testBoundsDiagonal()
    passed()
    
    print("  camera math: NaN guards")
    testIsInvalidFloat()
    testIsInvalidVector()
    testSanitize()
    passed()
    
    print("  camera math: rotation matrices")
    testRotationXIdentity()
    testRotationX90()
    testRotationY90()
    testRotationZ90()
    passed()
    
    print("  camera math: translation and scale")
    testTranslation()
    testScale()
    passed()
}
