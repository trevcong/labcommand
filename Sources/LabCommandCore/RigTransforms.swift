import Foundation

// MARK: - Rigid Transform

/// A rotation followed by a translation: `p_parent = R · p_child + t`.
/// Pure Swift (no simd), like `CameraMath`.
public struct RigidTransform: Sendable, Equatable {
    /// Rotation matrix rows.
    public var r0: Point3D
    public var r1: Point3D
    public var r2: Point3D
    public var translation: Point3D

    public init(r0: Point3D, r1: Point3D, r2: Point3D, translation: Point3D = Point3D()) {
        self.r0 = r0
        self.r1 = r1
        self.r2 = r2
        self.translation = translation
    }

    public static let identity = RigidTransform(
        r0: Point3D(x: 1, y: 0, z: 0),
        r1: Point3D(x: 0, y: 1, z: 0),
        r2: Point3D(x: 0, y: 0, z: 1)
    )

    /// Same convention as ROS `static_transform_publisher x y z yaw pitch roll`:
    /// `R = Rz(yaw) · Ry(pitch) · Rx(roll)`, angles in radians.
    public init(translation: Point3D, yaw: Double, pitch: Double, roll: Double) {
        let (cy, sy) = (cos(yaw), sin(yaw))
        let (cp, sp) = (cos(pitch), sin(pitch))
        let (cr, sr) = (cos(roll), sin(roll))
        self.init(
            r0: Point3D(x: Float(cy * cp), y: Float(cy * sp * sr - sy * cr), z: Float(cy * sp * cr + sy * sr)),
            r1: Point3D(x: Float(sy * cp), y: Float(sy * sp * sr + cy * cr), z: Float(sy * sp * cr - cy * sr)),
            r2: Point3D(x: Float(-sp), y: Float(cp * sr), z: Float(cp * cr)),
            translation: translation
        )
    }

    public var isIdentity: Bool { self == .identity }

    /// Rotate only — for directions.
    @inline(__always)
    public func rotate(_ p: Point3D) -> Point3D {
        Point3D(
            x: r0.x * p.x + r0.y * p.y + r0.z * p.z,
            y: r1.x * p.x + r1.y * p.y + r1.z * p.z,
            z: r2.x * p.x + r2.y * p.y + r2.z * p.z
        )
    }

    @inline(__always)
    public func apply(_ p: Point3D) -> Point3D {
        Point3D(
            x: r0.x * p.x + r0.y * p.y + r0.z * p.z + translation.x,
            y: r1.x * p.x + r1.y * p.y + r1.z * p.z + translation.y,
            z: r2.x * p.x + r2.y * p.y + r2.z * p.z + translation.z
        )
    }

    /// Move every point of `frame` into the parent frame, relabelling it `frameId`.
    public func apply(to frame: PointCloudFrame, frameId: String) -> PointCloudFrame {
        var result = frame
        result.frameId = frameId
        guard !isIdentity else { return result }
        result.positions.withUnsafeMutableBufferPointer { points in
            for i in points.indices {
                points[i] = apply(points[i])
            }
        }
        return result
    }
}

// MARK: - Rig Static TFs

/// Static TFs of the dual rig, as published on Spark's `/tf_static`.
/// `world` is the JT128 origin; the Mid-360 hangs inverted 110 mm below it.
public enum RigTransforms {
    public static let worldFrame = "world"
    public static let hesaiFrame = "hesai_lidar"
    public static let livoxFrame = "livox_frame"

    /// `static_transform_publisher 0 0 -0.110 <yaw> 0 <roll> world livox_frame`
    public static let livoxTranslation = Point3D(x: 0, y: 0, z: -0.110)
    public static let livoxYawDegrees: Double = -86.5
    public static let livoxPitch: Double = 0
    public static let livoxRoll: Double = .pi

    public static let hesaiToWorld = RigidTransform.identity

    public static let livoxToWorld = RigidTransform(
        translation: livoxTranslation,
        yaw: livoxYawDegrees * .pi / 180,
        pitch: livoxPitch,
        roll: livoxRoll
    )

    public static func sensorFrame(_ sensor: SensorID) -> String {
        switch sensor {
        case .mid360: return livoxFrame
        case .jt128: return hesaiFrame
        }
    }

    public static func toWorld(_ sensor: SensorID) -> RigidTransform {
        switch sensor {
        case .mid360: return livoxToWorld
        case .jt128: return hesaiToWorld
        }
    }

    /// A sensor-frame cloud expressed in `world`, tagged with its sensor.
    public static func inWorld(_ frame: PointCloudFrame, sensor: SensorID) -> PointCloudFrame {
        var result = toWorld(sensor).apply(to: frame, frameId: worldFrame)
        result.sensor = sensor
        return result
    }
}
