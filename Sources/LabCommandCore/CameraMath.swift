import Foundation

/// Camera math utilities for 3D rendering.
/// Pure Swift (no simd) for cross-platform compatibility.
/// Z-up coordinate convention. Metal depth range [0, 1].
public enum CameraMath {
    
    /// A 4x4 matrix stored in column-major order.
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
    
    /// Perspective projection matrix.
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
}
