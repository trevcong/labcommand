import Foundation

// MARK: - Sensor Extrinsics

/// Rigid transformation (rotation + translation) for sensor-to-rig-frame conversion.
/// Uses a right-handed coordinate system with Z-up.
public struct SensorExtrinsics: Sendable, Equatable {
    /// Rotation matrix (3x3 as column-major flat array).
    /// Identity = no rotation.
    public let rotation: [Float]
    
    /// Translation vector (x, y, z) in meters.
    public let translation: (x: Float, y: Float, z: Float)
    
    public static func == (lhs: SensorExtrinsics, rhs: SensorExtrinsics) -> Bool {
        lhs.rotation == rhs.rotation &&
        lhs.translation.x == rhs.translation.x &&
        lhs.translation.y == rhs.translation.y &&
        lhs.translation.z == rhs.translation.z
    }
    
    /// Create extrinsics with rotation matrix and translation.
    /// - Parameters:
    ///   - rotation: 3x3 column-major rotation matrix (9 floats). Pass nil for identity.
    ///   - translation: Translation in meters (x, y, z).
    public init(rotation: [Float]? = nil, translation: (x: Float, y: Float, z: Float)) {
        self.rotation = rotation ?? [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.translation = translation
    }
    
    /// Identity transform (no rotation or translation).
    public static let identity = SensorExtrinsics(translation: (0, 0, 0))
    
    /// Transform a point from sensor frame to rig (IMU) frame.
    /// p_imu = R * p_sensor - t
    public func transform(_ point: Point3D) -> Point3D {
        // Apply rotation: R * p
        let rx = rotation[0] * point.x + rotation[3] * point.y + rotation[6] * point.z
        let ry = rotation[1] * point.x + rotation[4] * point.y + rotation[7] * point.z
        let rz = rotation[2] * point.x + rotation[5] * point.y + rotation[8] * point.z
        
        // Subtract translation: p_imu = R * p - t
        return Point3D(
            x: rx - translation.x,
            y: ry - translation.y,
            z: rz - translation.z
        )
    }
    
    /// Check if this is an identity transform.
    public var isIdentity: Bool {
        let identityRotation: [Float] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        let rotationMatch = rotation == identityRotation
        let translationMatch = translation.x == 0 && translation.y == 0 && translation.z == 0
        return rotationMatch && translationMatch
    }
}

// MARK: - Rig Specification

/// Static specifications for the dual-lidar rig.
/// Contains extrinsics, sensor metadata, and frame conventions.
///
/// **Frame Convention:**
/// - Rig frame origin: Mid-360 IMU
/// - Coordinate system: Right-handed, Z-up
/// - X: Forward (direction of travel)
/// - Y: Left
/// - Z: Up
///
/// **Transform Convention (R·p−t):**
/// To bring sensor points into the IMU frame: `p_imu = R * p_sensor - t`
/// where `t` is the sensor's optical center position relative to the IMU.
///
/// **Extrinsics Source:**
/// - Mid-360: Livox SDK calibration — optical center offset from IMU
/// - JT128: Placeholder (not yet calibrated)
public enum RigSpec {
    
    // MARK: - Sensor Extrinsics
    
    /// Mid-360 extrinsics: optical center offset from IMU.
    /// Source: Livox SDK extrinsic calibration.
    /// The Mid-360's optical center is at (11.0, 23.29, −44.12) mm from its IMU.
    /// Transform: p_imu = p_lidar - (0.011, 0.02329, -0.04412)
    public static let mid360Extrinsics = SensorExtrinsics(
        rotation: nil,
        translation: (x: 0.011, y: 0.02329, z: -0.04412)
    )
    
    /// JT128 extrinsics: placeholder (not yet calibrated).
    /// Set `isCalibrated` to false to indicate this is a placeholder.
    public static let jt128Extrinsics = SensorExtrinsics.identity
    
    /// Whether the JT128 extrinsics have been calibrated.
    /// When false, the overlay may not be spatially accurate.
    public static let jt128IsCalibrated = false
    
    /// Get extrinsics for a sensor by ID.
    public static func extrinsics(for sensor: SensorID) -> SensorExtrinsics {
        switch sensor {
        case .mid360: return mid360Extrinsics
        case .jt128: return jt128Extrinsics
        }
    }
    
    // MARK: - Point Cloud Transformation
    
    /// Transform a point cloud frame from sensor frame to rig frame.
    /// Applies the sensor's extrinsics to each point.
    public static func transformToRigFrame(_ frame: PointCloudFrame) -> PointCloudFrame {
        guard let sensor = frame.sensor else {
            return frame
        }
        
        let extrinsics = self.extrinsics(for: sensor)
        
        if extrinsics.isIdentity {
            return frame
        }
        
        let transformedPositions = frame.positions.map { extrinsics.transform($0) }
        
        return PointCloudFrame(
            positions: transformedPositions,
            intensities: frame.intensities,
            sensor: frame.sensor,
            timestamp: frame.timestamp,
            frameId: frame.frameId
        )
    }
    
    /// Transform multiple frames to rig frame.
    public static func transformToRigFrame(_ frames: [PointCloudFrame]) -> [PointCloudFrame] {
        frames.map { transformToRigFrame($0) }
    }
}
