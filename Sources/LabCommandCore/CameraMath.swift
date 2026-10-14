import Foundation

// MARK: - Camera Control Vector Types (Pure Swift, Linux-portable)

/// 3D vector type for camera math.
public struct Float3: Equatable, Sendable {
    public var x: Float
    public var y: Float
    public var z: Float
    
    public static let zero = Float3(0, 0, 0)
    
    public init(_ x: Float, _ y: Float, _ z: Float) {
        self.x = x
        self.y = y
        self.z = z
    }
    
    public static func + (lhs: Float3, rhs: Float3) -> Float3 {
        Float3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z)
    }
    
    public static func - (lhs: Float3, rhs: Float3) -> Float3 {
        Float3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)
    }
    
    public static func * (lhs: Float3, rhs: Float) -> Float3 {
        Float3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs)
    }
    
    public static func * (lhs: Float, rhs: Float3) -> Float3 {
        Float3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z)
    }
    
    public static prefix func - (v: Float3) -> Float3 {
        Float3(-v.x, -v.y, -v.z)
    }
}

/// 4D vector type for homogeneous coordinates.
public struct Float4: Equatable, Sendable {
    public var x: Float
    public var y: Float
    public var z: Float
    public var w: Float
    
    public init(_ x: Float, _ y: Float, _ z: Float, _ w: Float) {
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    }
    
    public subscript(index: Int) -> Float {
        get {
            switch index {
            case 0: return x
            case 1: return y
            case 2: return z
            case 3: return w
            default: fatalError("Float4 index out of range")
            }
        }
        set {
            switch index {
            case 0: x = newValue
            case 1: y = newValue
            case 2: z = newValue
            case 3: w = newValue
            default: fatalError("Float4 index out of range")
            }
        }
    }
}

/// 4x4 matrix type for transforms (column-major).
public struct Float4x4: Equatable, Sendable {
    public var columns: (Float4, Float4, Float4, Float4)
    
    public init(_ c0: Float4, _ c1: Float4, _ c2: Float4, _ c3: Float4) {
        columns = (c0, c1, c2, c3)
    }
    
    public static func == (lhs: Float4x4, rhs: Float4x4) -> Bool {
        lhs.columns.0 == rhs.columns.0 &&
        lhs.columns.1 == rhs.columns.1 &&
        lhs.columns.2 == rhs.columns.2 &&
        lhs.columns.3 == rhs.columns.3
    }
    
    public subscript(column: Int) -> Float4 {
        get {
            switch column {
            case 0: return columns.0
            case 1: return columns.1
            case 2: return columns.2
            case 3: return columns.3
            default: fatalError("Float4x4 column index out of range")
            }
        }
        set {
            switch column {
            case 0: columns.0 = newValue
            case 1: columns.1 = newValue
            case 2: columns.2 = newValue
            case 3: columns.3 = newValue
            default: fatalError("Float4x4 column index out of range")
            }
        }
    }
    
    public static func * (lhs: Float4x4, rhs: Float4) -> Float4 {
        Float4(
            lhs[0].x * rhs.x + lhs[1].x * rhs.y + lhs[2].x * rhs.z + lhs[3].x * rhs.w,
            lhs[0].y * rhs.x + lhs[1].y * rhs.y + lhs[2].y * rhs.z + lhs[3].y * rhs.w,
            lhs[0].z * rhs.x + lhs[1].z * rhs.y + lhs[2].z * rhs.z + lhs[3].z * rhs.w,
            lhs[0].w * rhs.x + lhs[1].w * rhs.y + lhs[2].w * rhs.z + lhs[3].w * rhs.w
        )
    }
}

// MARK: - Vector Operations

public func float3Length(_ v: Float3) -> Float {
    sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
}

public func float3Normalize(_ v: Float3) -> Float3 {
    let len = float3Length(v)
    guard len > 0 else { return .zero }
    return Float3(v.x / len, v.y / len, v.z / len)
}

public func float3Cross(_ a: Float3, _ b: Float3) -> Float3 {
    Float3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    )
}

public func float3Dot(_ a: Float3, _ b: Float3) -> Float {
    a.x * b.x + a.y * b.y + a.z * b.z
}

// MARK: - Matrix Construction Helpers

public func makeFloat4x4(_ value: Float) -> Float4x4 {
    if value == 0 {
        return Float4x4(
            Float4(0, 0, 0, 0),
            Float4(0, 0, 0, 0),
            Float4(0, 0, 0, 0),
            Float4(0, 0, 0, 0)
        )
    } else {
        return identityFloat4x4()
    }
}

public func identityFloat4x4() -> Float4x4 {
    Float4x4(
        Float4(1, 0, 0, 0),
        Float4(0, 1, 0, 0),
        Float4(0, 0, 1, 0),
        Float4(0, 0, 0, 1)
    )
}

// MARK: - CameraMath

/// Camera math utilities for 3D rendering.
/// Pure Swift (no simd) for cross-platform compatibility.
/// Z-up coordinate convention. Metal depth range [0, 1].
public enum CameraMath {
    
    // MARK: - Renderer Matrix Type (PR #5)
    
    /// A 4x4 matrix stored in column-major order.
    /// Used by the renderer for view/projection matrices.
    public struct Matrix4x4: Equatable, Sendable {
        public var columns: (
            (Float, Float, Float, Float),
            (Float, Float, Float, Float),
            (Float, Float, Float, Float),
            (Float, Float, Float, Float)
        )
        
        public init(
            _ c0: (Float, Float, Float, Float),
            _ c1: (Float, Float, Float, Float),
            _ c2: (Float, Float, Float, Float),
            _ c3: (Float, Float, Float, Float)
        ) {
            self.columns = (c0, c1, c2, c3)
        }
        
        public static func == (lhs: Matrix4x4, rhs: Matrix4x4) -> Bool {
            lhs.columns.0.0 == rhs.columns.0.0 &&
            lhs.columns.0.1 == rhs.columns.0.1 &&
            lhs.columns.0.2 == rhs.columns.0.2 &&
            lhs.columns.0.3 == rhs.columns.0.3 &&
            lhs.columns.1.0 == rhs.columns.1.0 &&
            lhs.columns.1.1 == rhs.columns.1.1 &&
            lhs.columns.1.2 == rhs.columns.1.2 &&
            lhs.columns.1.3 == rhs.columns.1.3 &&
            lhs.columns.2.0 == rhs.columns.2.0 &&
            lhs.columns.2.1 == rhs.columns.2.1 &&
            lhs.columns.2.2 == rhs.columns.2.2 &&
            lhs.columns.2.3 == rhs.columns.2.3 &&
            lhs.columns.3.0 == rhs.columns.3.0 &&
            lhs.columns.3.1 == rhs.columns.3.1 &&
            lhs.columns.3.2 == rhs.columns.3.2 &&
            lhs.columns.3.3 == rhs.columns.3.3
        }
        
        public static let identity = Matrix4x4(
            (1, 0, 0, 0),
            (0, 1, 0, 0),
            (0, 0, 1, 0),
            (0, 0, 0, 1)
        )
        
        /// Transform a 3D point, returning (x, y, z, w) in clip space.
        public func transform(_ p: Point3D) -> (x: Float, y: Float, z: Float, w: Float) {
            let x = columns.0.0 * p.x + columns.1.0 * p.y + columns.2.0 * p.z + columns.3.0
            let y = columns.0.1 * p.x + columns.1.1 * p.y + columns.2.1 * p.z + columns.3.1
            let z = columns.0.2 * p.x + columns.1.2 * p.y + columns.2.2 * p.z + columns.3.2
            let w = columns.0.3 * p.x + columns.1.3 * p.y + columns.2.3 * p.z + columns.3.3
            return (x, y, z, w)
        }
        
        /// Transform a point to NDC (normalized device coordinates).
        public func transformToNDC(_ p: Point3D) -> (x: Float, y: Float, z: Float)? {
            let clip = transform(p)
            guard abs(clip.w) > 0.0001 else { return nil }
            return (clip.x / clip.w, clip.y / clip.w, clip.z / clip.w)
        }
        
        /// Matrix multiplication.
        public static func * (lhs: Matrix4x4, rhs: Matrix4x4) -> Matrix4x4 {
            var result = Matrix4x4.identity
            
            // Column 0
            result.columns.0.0 = lhs.columns.0.0 * rhs.columns.0.0 + lhs.columns.1.0 * rhs.columns.0.1 + lhs.columns.2.0 * rhs.columns.0.2 + lhs.columns.3.0 * rhs.columns.0.3
            result.columns.0.1 = lhs.columns.0.1 * rhs.columns.0.0 + lhs.columns.1.1 * rhs.columns.0.1 + lhs.columns.2.1 * rhs.columns.0.2 + lhs.columns.3.1 * rhs.columns.0.3
            result.columns.0.2 = lhs.columns.0.2 * rhs.columns.0.0 + lhs.columns.1.2 * rhs.columns.0.1 + lhs.columns.2.2 * rhs.columns.0.2 + lhs.columns.3.2 * rhs.columns.0.3
            result.columns.0.3 = lhs.columns.0.3 * rhs.columns.0.0 + lhs.columns.1.3 * rhs.columns.0.1 + lhs.columns.2.3 * rhs.columns.0.2 + lhs.columns.3.3 * rhs.columns.0.3
            
            // Column 1
            result.columns.1.0 = lhs.columns.0.0 * rhs.columns.1.0 + lhs.columns.1.0 * rhs.columns.1.1 + lhs.columns.2.0 * rhs.columns.1.2 + lhs.columns.3.0 * rhs.columns.1.3
            result.columns.1.1 = lhs.columns.0.1 * rhs.columns.1.0 + lhs.columns.1.1 * rhs.columns.1.1 + lhs.columns.2.1 * rhs.columns.1.2 + lhs.columns.3.1 * rhs.columns.1.3
            result.columns.1.2 = lhs.columns.0.2 * rhs.columns.1.0 + lhs.columns.1.2 * rhs.columns.1.1 + lhs.columns.2.2 * rhs.columns.1.2 + lhs.columns.3.2 * rhs.columns.1.3
            result.columns.1.3 = lhs.columns.0.3 * rhs.columns.1.0 + lhs.columns.1.3 * rhs.columns.1.1 + lhs.columns.2.3 * rhs.columns.1.2 + lhs.columns.3.3 * rhs.columns.1.3
            
            // Column 2
            result.columns.2.0 = lhs.columns.0.0 * rhs.columns.2.0 + lhs.columns.1.0 * rhs.columns.2.1 + lhs.columns.2.0 * rhs.columns.2.2 + lhs.columns.3.0 * rhs.columns.2.3
            result.columns.2.1 = lhs.columns.0.1 * rhs.columns.2.0 + lhs.columns.1.1 * rhs.columns.2.1 + lhs.columns.2.1 * rhs.columns.2.2 + lhs.columns.3.1 * rhs.columns.2.3
            result.columns.2.2 = lhs.columns.0.2 * rhs.columns.2.0 + lhs.columns.1.2 * rhs.columns.2.1 + lhs.columns.2.2 * rhs.columns.2.2 + lhs.columns.3.2 * rhs.columns.2.3
            result.columns.2.3 = lhs.columns.0.3 * rhs.columns.2.0 + lhs.columns.1.3 * rhs.columns.2.1 + lhs.columns.2.3 * rhs.columns.2.2 + lhs.columns.3.3 * rhs.columns.2.3
            
            // Column 3
            result.columns.3.0 = lhs.columns.0.0 * rhs.columns.3.0 + lhs.columns.1.0 * rhs.columns.3.1 + lhs.columns.2.0 * rhs.columns.3.2 + lhs.columns.3.0 * rhs.columns.3.3
            result.columns.3.1 = lhs.columns.0.1 * rhs.columns.3.0 + lhs.columns.1.1 * rhs.columns.3.1 + lhs.columns.2.1 * rhs.columns.3.2 + lhs.columns.3.1 * rhs.columns.3.3
            result.columns.3.2 = lhs.columns.0.2 * rhs.columns.3.0 + lhs.columns.1.2 * rhs.columns.3.1 + lhs.columns.2.2 * rhs.columns.3.2 + lhs.columns.3.2 * rhs.columns.3.3
            result.columns.3.3 = lhs.columns.0.3 * rhs.columns.3.0 + lhs.columns.1.3 * rhs.columns.3.1 + lhs.columns.2.3 * rhs.columns.3.2 + lhs.columns.3.3 * rhs.columns.3.3
            
            return result
        }
    }
    
    // MARK: - Constants
    
    /// Field of view used by the point cloud renderer (45 degrees).
    public static let defaultFovY: Float = Float.pi / 4
    
    /// Default elevation angle for camera (30 degrees above horizontal).
    public static let defaultElevation: Float = Float.pi / 6
    
    /// Normalize a Point3D vector.
    public static func normalize(_ v: Point3D) -> Point3D {
        let len = sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        guard len > 0.0001 else { return Point3D(x: 0, y: 0, z: 1) }
        return Point3D(x: v.x / len, y: v.y / len, z: v.z / len)
    }
    
    /// Cross product of two Point3D vectors.
    public static func cross(_ a: Point3D, _ b: Point3D) -> Point3D {
        Point3D(
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x
        )
    }
    
    /// Dot product of two Point3D vectors.
    public static func dot(_ a: Point3D, _ b: Point3D) -> Float {
        a.x * b.x + a.y * b.y + a.z * b.z
    }
    
    /// Look-at view matrix. Z-up convention.
    public static func lookAt(
        eye: Point3D,
        center: Point3D,
        up: Point3D = Point3D(x: 0, y: 0, z: 1)
    ) -> Matrix4x4 {
        // Forward (z) = normalize(eye - center)
        let fx = eye.x - center.x
        let fy = eye.y - center.y
        let fz = eye.z - center.z
        let fLen = sqrt(fx * fx + fy * fy + fz * fz)
        let zx = fx / fLen
        let zy = fy / fLen
        let zz = fz / fLen
        
        // Right (x) = normalize(cross(up, z))
        let cx = up.y * zz - up.z * zy
        let cy = up.z * zx - up.x * zz
        let cz = up.x * zy - up.y * zx
        let cLen = sqrt(cx * cx + cy * cy + cz * cz)
        let xx = cx / cLen
        let xy = cy / cLen
        let xz = cz / cLen
        
        // Up (y) = cross(z, x)
        let yx = zy * xz - zz * xy
        let yy = zz * xx - zx * xz
        let yz = zx * xy - zy * xx
        
        // Translation
        let tx = -(xx * eye.x + xy * eye.y + xz * eye.z)
        let ty = -(yx * eye.x + yy * eye.y + yz * eye.z)
        let tz = -(zx * eye.x + zy * eye.y + zz * eye.z)
        
        return Matrix4x4(
            (xx, yx, zx, 0),
            (xy, yy, zy, 0),
            (xz, yz, zz, 0),
            (tx, ty, tz, 1)
        )
    }
    
    /// Perspective projection matrix using Matrix4x4.
    /// Metal depth range [0, 1] (not OpenGL [-1, 1]).
    public static func perspective(
        fovY: Float,
        aspect: Float,
        near: Float,
        far: Float
    ) -> Matrix4x4 {
        let yScale = 1 / tan(fovY * 0.5)
        let xScale = yScale / aspect
        let zRange = far - near
        
        // Metal NDC: z in [0, 1]
        let zScale = -far / zRange
        let wzScale = -far * near / zRange
        
        return Matrix4x4(
            (xScale, 0, 0, 0),
            (0, yScale, 0, 0),
            (0, 0, zScale, -1),
            (0, 0, wzScale, 0)
        )
    }
    
    /// Computes camera distance to fit bbox corners within margin m of NDC [-1,1].
    /// - corners: 8 bbox corner points
    /// - target: bbox center
    /// - f: camera's forward unit vector (from eye toward target)
    /// - r: camera's right unit vector
    /// - u: camera's up unit vector
    /// - fovY: vertical field of view in radians
    /// - aspect: width/height
    /// - margin: NDC margin (0.9 = fill 90% of screen)
    /// - near: near plane distance
    /// Returns: distance d such that camera is at target - d*f
    public static func fitDistance(
        corners: [Point3D],
        target: Point3D,
        forward f: Point3D,
        right r: Point3D,
        up u: Point3D,
        fovY: Float,
        aspect: Float,
        margin m: Float = 0.9,
        near: Float = 0.1
    ) -> Float {
        let tanY = tan(fovY / 2)
        let tanX = tanY * aspect
        
        var dMax: Float = near + 0.001
        
        for p in corners {
            // q = p - target (corner relative to target)
            let qx = p.x - target.x
            let qy = p.y - target.y
            let qz = p.z - target.z
            
            // Project onto camera axes
            let x = qx * r.x + qy * r.y + qz * r.z  // q·r
            let y = qx * u.x + qy * u.y + qz * u.z  // q·u
            let z = qx * f.x + qy * f.y + qz * f.z  // q·f (positive = toward far side)
            
            // |NDC| ≤ m requires: d ≥ |x|/(m*tanX) - z  AND  d ≥ |y|/(m*tanY) - z
            let dForX = abs(x) / (m * tanX) - z
            let dForY = abs(y) / (m * tanY) - z
            
            dMax = Swift.max(dMax, dForX, dForY)
        }
        
        return Swift.max(dMax, near + 0.001)
    }
    
    /// Generate the 8 corners of a bounding box.
    public static func bboxCorners(min: Point3D, max: Point3D) -> [Point3D] {
        [
            Point3D(x: min.x, y: min.y, z: min.z),
            Point3D(x: max.x, y: min.y, z: min.z),
            Point3D(x: min.x, y: max.y, z: min.z),
            Point3D(x: max.x, y: max.y, z: min.z),
            Point3D(x: min.x, y: min.y, z: max.z),
            Point3D(x: max.x, y: min.y, z: max.z),
            Point3D(x: min.x, y: max.y, z: max.z),
            Point3D(x: max.x, y: max.y, z: max.z),
        ]
    }
    
    /// Compute camera orientation vectors for elevated view.
    /// Camera looks from Y-negative toward target, elevated by elevation angle.
    /// Returns (forward, right, up) unit vectors.
    public static func cameraOrientation(
        elevation: Float = defaultElevation,
        worldUp: Point3D = Point3D(x: 0, y: 0, z: 1)
    ) -> (forward: Point3D, right: Point3D, up: Point3D) {
        // Forward direction: looking from -Y toward origin, tilted down by elevation
        // At elevation=0: forward = (0, 1, 0) (looking along +Y)
        // At elevation>0: forward tilts down, so z component becomes negative
        let cosE = cos(elevation)
        let sinE = sin(elevation)
        let forward = Point3D(x: 0, y: cosE, z: -sinE)
        
        // Right is always along +X for Y-negative camera
        let right = Point3D(x: 1, y: 0, z: 0)
        
        // Up = cross(forward, right) but normalized
        // forward × right = (cosE * 0 - (-sinE) * 0, (-sinE) * 1 - 0 * 0, 0 * 0 - cosE * 1)
        //                 = (0, -sinE, -cosE)
        // But we want up to point generally toward +Z, so negate
        let up = Point3D(x: 0, y: sinE, z: cosE)
        
        return (forward, right, up)
    }
    
    /// Compute camera parameters to fit a bounding box using exact fit algorithm.
    /// Returns (eye position, center, distance).
    public static func fitToBounds(
        min: Point3D,
        max: Point3D,
        fovY: Float = defaultFovY,
        aspect: Float = 1.0,
        margin: Float = 0.9,
        elevation: Float = defaultElevation
    ) -> (eye: Point3D, center: Point3D, distance: Float) {
        let center = Point3D(
            x: (min.x + max.x) * 0.5,
            y: (min.y + max.y) * 0.5,
            z: (min.z + max.z) * 0.5
        )
        
        let corners = bboxCorners(min: min, max: max)
        let (forward, right, up) = cameraOrientation(elevation: elevation)
        
        let distance = fitDistance(
            corners: corners,
            target: center,
            forward: forward,
            right: right,
            up: up,
            fovY: fovY,
            aspect: aspect,
            margin: margin
        )
        
        // Eye = center - distance * forward
        let eye = Point3D(
            x: center.x - distance * forward.x,
            y: center.y - distance * forward.y,
            z: center.z - distance * forward.z
        )
        
        return (eye, center, distance)
    }
    
    // MARK: - Camera Control Matrix Functions (PR #6)
    
    /// Creates a view matrix using Float3. Z-up convention.
    public static func lookAt(eye: Float3, target: Float3, up: Float3 = worldUp) -> Float4x4 {
        let f = float3Normalize(target - eye)
        let s = float3Normalize(float3Cross(f, up))
        let u = float3Cross(s, f)
        
        var result = identityFloat4x4()
        result[0][0] = s.x
        result[1][0] = s.y
        result[2][0] = s.z
        result[0][1] = u.x
        result[1][1] = u.y
        result[2][1] = u.z
        result[0][2] = -f.x
        result[1][2] = -f.y
        result[2][2] = -f.z
        result[3][0] = -float3Dot(s, eye)
        result[3][1] = -float3Dot(u, eye)
        result[3][2] = float3Dot(f, eye)
        return result
    }
    
    /// Creates a perspective projection matrix for Metal (depth range [0, 1], right-handed).
    public static func perspectiveMetal(fovY: Float, aspect: Float, near: Float, far: Float) -> Float4x4 {
        let yScale = 1.0 / tan(fovY * 0.5)
        let xScale = yScale / aspect
        let zRange = far - near
        
        var result = makeFloat4x4(0)
        result[0][0] = xScale
        result[1][1] = yScale
        result[2][2] = -far / zRange
        result[2][3] = -1.0
        result[3][2] = -(far * near) / zRange
        return result
    }
    
    /// Creates a translation matrix.
    public static func translation(_ t: Float3) -> Float4x4 {
        var result = identityFloat4x4()
        result[3][0] = t.x
        result[3][1] = t.y
        result[3][2] = t.z
        return result
    }
    
    /// Creates a rotation matrix around the X axis.
    public static func rotationX(_ angle: Float) -> Float4x4 {
        let c = cos(angle)
        let s = sin(angle)
        var result = identityFloat4x4()
        result[1][1] = c
        result[1][2] = s
        result[2][1] = -s
        result[2][2] = c
        return result
    }
    
    /// Creates a rotation matrix around the Y axis.
    public static func rotationY(_ angle: Float) -> Float4x4 {
        let c = cos(angle)
        let s = sin(angle)
        var result = identityFloat4x4()
        result[0][0] = c
        result[0][2] = -s
        result[2][0] = s
        result[2][2] = c
        return result
    }
    
    /// Creates a rotation matrix around the Z axis (up axis in Z-up convention).
    public static func rotationZ(_ angle: Float) -> Float4x4 {
        let c = cos(angle)
        let s = sin(angle)
        var result = identityFloat4x4()
        result[0][0] = c
        result[0][1] = s
        result[1][0] = -s
        result[1][1] = c
        return result
    }
    
    /// Creates a scale matrix.
    public static func scale(_ s: Float3) -> Float4x4 {
        var result = identityFloat4x4()
        result[0][0] = s.x
        result[1][1] = s.y
        result[2][2] = s.z
        return result
    }
    
    // MARK: - Orbit Camera (Z-up)
    
    /// Computes the eye position for an orbit camera (Z-up convention).
    public static func orbitPosition(
        target: Float3,
        distance: Float,
        azimuth: Float,
        elevation: Float
    ) -> Float3 {
        let x = distance * cos(elevation) * cos(azimuth)
        let y = distance * cos(elevation) * sin(azimuth)
        let z = distance * sin(elevation)
        return target + Float3(x, y, z)
    }
    
    /// Computes the right vector for orbit camera (Z-up).
    public static func orbitRight(azimuth: Float) -> Float3 {
        Float3(-sin(azimuth), cos(azimuth), 0)
    }
    
    /// Computes the up vector for orbit camera relative to view (Z-up).
    public static func orbitUp(azimuth: Float, elevation: Float) -> Float3 {
        let x = -sin(elevation) * cos(azimuth)
        let y = -sin(elevation) * sin(azimuth)
        let z = cos(elevation)
        return Float3(x, y, z)
    }
    
    /// Creates a view matrix for an orbit camera (Z-up convention).
    public static func orbitViewMatrix(
        target: Float3,
        distance: Float,
        azimuth: Float,
        elevation: Float
    ) -> Float4x4 {
        let eye = orbitPosition(target: target, distance: distance, azimuth: azimuth, elevation: elevation)
        return lookAt(eye: eye, target: target, up: worldUp)
    }
    
    // MARK: - Fly Camera (Z-up)
    
    /// Computes the forward direction for a fly camera given yaw and pitch (Z-up convention).
    public static func flyForward(yaw: Float, pitch: Float) -> Float3 {
        let x = cos(pitch) * cos(yaw)
        let y = cos(pitch) * sin(yaw)
        let z = sin(pitch)
        return Float3(x, y, z)
    }
    
    /// Computes the right direction for a fly camera given yaw (Z-up).
    public static func flyRight(yaw: Float) -> Float3 {
        Float3(-sin(yaw), cos(yaw), 0)
    }
    
    /// Computes the up direction for a fly camera (always world up for level flight).
    public static func flyUp() -> Float3 {
        worldUp
    }
    
    /// Creates a view matrix for a fly camera (Z-up convention).
    public static func flyViewMatrix(position: Float3, yaw: Float, pitch: Float) -> Float4x4 {
        let forward = flyForward(yaw: yaw, pitch: pitch)
        let target = position + forward
        return lookAt(eye: position, target: target, up: worldUp)
    }
    
    // MARK: - Interpolation
    
    /// Linearly interpolates between two values.
    public static func lerp(_ a: Float, _ b: Float, _ t: Float) -> Float {
        a + (b - a) * t
    }
    
    /// Linearly interpolates between two vectors.
    public static func lerp(_ a: Float3, _ b: Float3, _ t: Float) -> Float3 {
        a + (b - a) * t
    }
    
    /// Interpolates between two angles, taking the shortest path.
    public static func lerpAngle(_ a: Float, _ b: Float, _ t: Float) -> Float {
        guard !isInvalid(a) && !isInvalid(b) && !isInvalid(t) else { return a }
        var diff = b - a
        while diff > .pi { diff -= 2 * .pi }
        while diff < -.pi { diff += 2 * .pi }
        return a + diff * t
    }
    
    /// Wraps an angle to the range (-π, π].
    public static func wrapAngle(_ angle: Float) -> Float {
        guard !isInvalid(angle) else { return 0 }
        var a = angle.truncatingRemainder(dividingBy: 2 * .pi)
        let eps: Float = 1e-5
        if a > .pi + eps { a -= 2 * .pi }
        if a < -.pi - eps { a += 2 * .pi }
        if a <= -.pi + eps && a >= -.pi - eps { a = .pi }
        return a
    }
    
    // MARK: - Utility
    
    /// Clamps a value between min and max.
    public static func clamp(_ value: Float, min minVal: Float, max maxVal: Float) -> Float {
        Swift.min(Swift.max(value, minVal), maxVal)
    }
    
    /// Clamps a point inside a bounding box.
    public static func clampToBounds(_ point: Float3, min: Float3, max: Float3) -> Float3 {
        Float3(
            clamp(point.x, min: min.x, max: max.x),
            clamp(point.y, min: min.y, max: max.y),
            clamp(point.z, min: min.z, max: max.z)
        )
    }
    
    /// Computes the center of a bounding box.
    public static func boundsCenter(min: Float3, max: Float3) -> Float3 {
        (min + max) * 0.5
    }
    
    /// Computes the diagonal length of a bounding box.
    public static func boundsDiagonal(min: Float3, max: Float3) -> Float {
        float3Length(max - min)
    }
    
    /// Returns true if the value is NaN or infinite.
    public static func isInvalid(_ value: Float) -> Bool {
        value.isNaN || value.isInfinite
    }
    
    /// Returns true if any component is NaN or infinite.
    public static func isInvalid(_ v: Float3) -> Bool {
        isInvalid(v.x) || isInvalid(v.y) || isInvalid(v.z)
    }
    
    /// Returns default if value is invalid, otherwise returns value.
    public static func sanitize(_ value: Float, default defaultValue: Float) -> Float {
        isInvalid(value) ? defaultValue : value
    }
    
    /// Returns default if any component is invalid, otherwise returns value.
    public static func sanitize(_ v: Float3, default defaultValue: Float3) -> Float3 {
        isInvalid(v) ? defaultValue : v
    }
    
    /// Returns the length (magnitude) of a vector.
    public static func length(_ v: Float3) -> Float {
        float3Length(v)
    }
}
