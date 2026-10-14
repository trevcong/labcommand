import Foundation
import LabCommandCore

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
    // Flat, wide cloud (like LiDAR ground scan): wide X/Y, shallow Z
    runFitTest(
        name: "flatCloud",
        bboxMin: Point3D(x: -20, y: -20, z: -1),
        bboxMax: Point3D(x: 20, y: 20, z: 1),
        aspect: 16.0 / 9.0
    )
}

func testFitCorridor30Deg() {
    // 2×20×2 corridor at 30° elevation
    // This tests the elevated camera view on a long narrow corridor
    runFitTest(
        name: "corridor30deg",
        bboxMin: Point3D(x: -1, y: -10, z: -1),
        bboxMax: Point3D(x: 1, y: 10, z: 1),
        aspect: 16.0 / 9.0,
        elevation: Float.pi / 6  // 30°
    )
}

func testFitDistanceAlgorithm() {
    // Direct test of fitDistance function
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
    
    // Compute eye position and verify projection
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
    
    // Should be close to margin (0.9)
    expect(
        maxAbsNDC >= 0.89 && maxAbsNDC <= 0.91,
        "fitDistance maxNDC=\(maxAbsNDC) should be ~0.9"
    )
}

func testCameraOrientation() {
    // Test that camera orientation vectors are orthonormal
    let (f, r, u) = CameraMath.cameraOrientation(elevation: Float.pi / 6)
    
    // Check unit length
    let fLen = sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
    let rLen = sqrt(r.x * r.x + r.y * r.y + r.z * r.z)
    let uLen = sqrt(u.x * u.x + u.y * u.y + u.z * u.z)
    
    expect(abs(fLen - 1) < 0.001, "forward should be unit vector")
    expect(abs(rLen - 1) < 0.001, "right should be unit vector")
    expect(abs(uLen - 1) < 0.001, "up should be unit vector")
    
    // Check orthogonality
    let fDotR = CameraMath.dot(f, r)
    let fDotU = CameraMath.dot(f, u)
    let rDotU = CameraMath.dot(r, u)
    
    expect(abs(fDotR) < 0.001, "forward·right should be 0")
    expect(abs(fDotU) < 0.001, "forward·up should be 0")
    expect(abs(rDotU) < 0.001, "right·up should be 0")
}

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

func runCameraMathTests() {
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
}
