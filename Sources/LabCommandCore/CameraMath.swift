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
    
    /// Compute camera parameters to fit a bounding box using projected extents.
    /// Returns (eye position, center, distance).
    /// Uses projected bbox fit (not bounding-sphere) for better fill on flat/wide clouds.
    /// Camera looks from Y-negative toward center (Z-up convention):
    /// - X extent → screen horizontal (width)
    /// - Z extent → screen vertical (height)
    /// - Y extent → depth (along view direction)
    public static func fitToBounds(
        min: Point3D,
        max: Point3D,
        fovY: Float = defaultFovY,
        aspect: Float = 1.0,
        margin: Float = 1.1
    ) -> (eye: Point3D, center: Point3D, distance: Float) {
        let center = Point3D(
            x: (min.x + max.x) * 0.5,
            y: (min.y + max.y) * 0.5,
            z: (min.z + max.z) * 0.5
        )
        
        // Half-extents perpendicular to view direction
        let halfWidth = (max.x - min.x) / 2   // screen horizontal
        let halfHeight = (max.z - min.z) / 2  // screen vertical
        
        // Compute horizontal FOV from vertical FOV and aspect ratio
        let fovX = 2 * atan(tan(fovY / 2) * aspect)
        
        // Distance needed for each axis to fit in view
        let distForWidth = halfWidth / tan(fovX / 2)
        let distForHeight = halfHeight / tan(fovY / 2)
        
        // Take the max (limiting axis) and apply margin for 80-90% fill
        let distance = Swift.max(Swift.max(distForWidth, distForHeight) * margin, 0.1)
        
        // Position camera looking from Y-negative toward center
        let eye = Point3D(
            x: center.x,
            y: center.y - distance,
            z: center.z + distance * 0.3
        )
        
        return (eye, center, distance)
    }
}
