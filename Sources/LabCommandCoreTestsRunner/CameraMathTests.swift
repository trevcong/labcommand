import Foundation
import LabCommandCore

func testFitToBoundsProjectsToNDC() {
    let bboxMin = Point3D(x: -5, y: -5, z: -2)
    let bboxMax = Point3D(x: 5, y: 5, z: 3)
    let aspect: Float = 1.0
    
    let (eye, center, _) = CameraMath.fitToBounds(
        min: bboxMin,
        max: bboxMax,
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        margin: 1.2
    )
    
    let view = CameraMath.lookAt(eye: eye, center: center)
    let proj = CameraMath.perspective(
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        near: 0.1,
        far: 1000
    )
    let mvp = proj * view
    
    let corners = [
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMax.z),
    ]
    
    for (i, corner) in corners.enumerated() {
        guard let ndc = mvp.transformToNDC(corner) else {
            fputs("FAIL: corner \(i) has w ≈ 0 (camera at geometry)\n", stderr)
            exit(1)
        }
        
        let limit: Float = 0.9
        expect(
            ndc.x >= -limit && ndc.x <= limit,
            "corner \(i) NDC.x=\(ndc.x) out of [-0.9, 0.9]"
        )
        expect(
            ndc.y >= -limit && ndc.y <= limit,
            "corner \(i) NDC.y=\(ndc.y) out of [-0.9, 0.9]"
        )
        expect(
            ndc.z >= 0 && ndc.z <= 1,
            "corner \(i) NDC.z=\(ndc.z) out of [0, 1] (Metal depth range)"
        )
    }
}

func testFitToBoundsSmallCloud() {
    let bboxMin = Point3D(x: -0.5, y: -0.5, z: -0.25)
    let bboxMax = Point3D(x: 0.5, y: 0.5, z: 0.25)
    let aspect: Float = 1.0
    
    let (eye, center, _) = CameraMath.fitToBounds(
        min: bboxMin,
        max: bboxMax,
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        margin: 1.2
    )
    
    let view = CameraMath.lookAt(eye: eye, center: center)
    let proj = CameraMath.perspective(
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        near: 0.1,
        far: 1000
    )
    let mvp = proj * view
    
    let corners = [
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMax.z),
    ]
    
    for (i, corner) in corners.enumerated() {
        guard let ndc = mvp.transformToNDC(corner) else {
            fputs("FAIL: small cloud corner \(i) has w ≈ 0\n", stderr)
            exit(1)
        }
        
        let limit: Float = 0.9
        expect(
            ndc.x >= -limit && ndc.x <= limit,
            "small cloud corner \(i) NDC.x=\(ndc.x) out of [-0.9, 0.9]"
        )
        expect(
            ndc.y >= -limit && ndc.y <= limit,
            "small cloud corner \(i) NDC.y=\(ndc.y) out of [-0.9, 0.9]"
        )
    }
}

func testFitToBoundsWideAspect() {
    let bboxMin = Point3D(x: -10, y: -10, z: 0)
    let bboxMax = Point3D(x: 10, y: 10, z: 5)
    let aspect: Float = 16.0 / 9.0
    
    let (eye, center, _) = CameraMath.fitToBounds(
        min: bboxMin,
        max: bboxMax,
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        margin: 1.2
    )
    
    let view = CameraMath.lookAt(eye: eye, center: center)
    let proj = CameraMath.perspective(
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        near: 0.1,
        far: 1000
    )
    let mvp = proj * view
    
    let corners = [
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMax.z),
    ]
    
    for (i, corner) in corners.enumerated() {
        guard let ndc = mvp.transformToNDC(corner) else {
            fputs("FAIL: wide aspect corner \(i) has w ≈ 0\n", stderr)
            exit(1)
        }
        
        let limit: Float = 0.9
        expect(
            ndc.x >= -limit && ndc.x <= limit,
            "wide aspect corner \(i) NDC.x=\(ndc.x) out of [-0.9, 0.9]"
        )
        expect(
            ndc.y >= -limit && ndc.y <= limit,
            "wide aspect corner \(i) NDC.y=\(ndc.y) out of [-0.9, 0.9]"
        )
        expect(
            ndc.z >= 0 && ndc.z <= 1,
            "wide aspect corner \(i) NDC.z=\(ndc.z) out of Metal [0, 1] range"
        )
    }
}

func testFitToBoundsNarrowAspect() {
    let bboxMin = Point3D(x: -5, y: -5, z: 0)
    let bboxMax = Point3D(x: 5, y: 5, z: 2)
    let aspect: Float = 0.5
    
    let (eye, center, _) = CameraMath.fitToBounds(
        min: bboxMin,
        max: bboxMax,
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        margin: 1.2
    )
    
    let view = CameraMath.lookAt(eye: eye, center: center)
    let proj = CameraMath.perspective(
        fovY: CameraMath.defaultFovY,
        aspect: aspect,
        near: 0.1,
        far: 1000
    )
    let mvp = proj * view
    
    let corners = [
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMin.z),
        Point3D(x: bboxMin.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMin.y, z: bboxMax.z),
        Point3D(x: bboxMin.x, y: bboxMax.y, z: bboxMax.z),
        Point3D(x: bboxMax.x, y: bboxMax.y, z: bboxMax.z),
    ]
    
    var maxAbsX: Float = 0
    var maxAbsY: Float = 0
    
    for (i, corner) in corners.enumerated() {
        guard let ndc = mvp.transformToNDC(corner) else {
            fputs("FAIL: narrow aspect corner \(i) has w ≈ 0\n", stderr)
            exit(1)
        }
        
        let limit: Float = 0.9
        expect(
            ndc.x >= -limit && ndc.x <= limit,
            "narrow aspect corner \(i) NDC.x=\(ndc.x) out of [-0.9, 0.9]"
        )
        expect(
            ndc.y >= -limit && ndc.y <= limit,
            "narrow aspect corner \(i) NDC.y=\(ndc.y) out of [-0.9, 0.9]"
        )
        expect(
            ndc.z >= 0 && ndc.z <= 1,
            "narrow aspect corner \(i) NDC.z=\(ndc.z) out of Metal [0, 1] range"
        )
        
        maxAbsX = max(maxAbsX, abs(ndc.x))
        maxAbsY = max(maxAbsY, abs(ndc.y))
    }
    
    // With aspect 0.5 (tall/narrow viewport), horizontal FOV is smaller than vertical,
    // so the x coordinates should be closer to the edges (larger absolute values)
    // because we fit to the smaller dimension
    expect(
        maxAbsX > 0.3,
        "narrow aspect: max |NDC.x|=\(maxAbsX) should be > 0.3 (bbox uses horizontal space)"
    )
    expect(
        maxAbsY > 0.3,
        "narrow aspect: max |NDC.y|=\(maxAbsY) should be > 0.3 (bbox uses vertical space)"
    )
    
    // Concrete value check: center point should project to ~(0, 0)
    let centerPoint = Point3D(
        x: (bboxMin.x + bboxMax.x) / 2,
        y: (bboxMin.y + bboxMax.y) / 2,
        z: (bboxMin.z + bboxMax.z) / 2
    )
    guard let centerNDC = mvp.transformToNDC(centerPoint) else {
        fputs("FAIL: center point has w ≈ 0\n", stderr)
        exit(1)
    }
    expect(
        abs(centerNDC.x) < 0.1,
        "narrow aspect: center NDC.x=\(centerNDC.x) should be ~0"
    )
    expect(
        abs(centerNDC.y) < 0.1,
        "narrow aspect: center NDC.y=\(centerNDC.y) should be ~0"
    )
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
    testFitToBoundsProjectsToNDC()
    print("  camera math: fitToBounds projects all 8 bbox corners within NDC [-0.9, 0.9]"); passed()
    testFitToBoundsSmallCloud()
    print("  camera math: fitToBounds works for small point clouds"); passed()
    testFitToBoundsWideAspect()
    print("  camera math: fitToBounds handles wide aspect ratios (16:9)"); passed()
    testFitToBoundsNarrowAspect()
    print("  camera math: fitToBounds handles narrow aspect ratios (0.5), concrete NDC checks"); passed()
}
