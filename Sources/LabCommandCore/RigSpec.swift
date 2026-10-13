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
    
    /// Transform a point from sensor frame to rig frame.
    public func transform(_ point: Point3D) -> Point3D {
        // Apply rotation: R * p
        let rx = rotation[0] * point.x + rotation[3] * point.y + rotation[6] * point.z
        let ry = rotation[1] * point.x + rotation[4] * point.y + rotation[7] * point.z
        let rz = rotation[2] * point.x + rotation[5] * point.y + rotation[8] * point.z
        
        // Apply translation: R * p + t
        return Point3D(
            x: rx + translation.x,
            y: ry + translation.y,
            z: rz + translation.z
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
/// - Rig frame origin: JT128 lidar center (reference sensor)
/// - Coordinate system: Right-handed, Z-up
/// - X: Forward (direction of travel)
/// - Y: Left
/// - Z: Up
///
/// **Extrinsics Source:**
/// - Mid-360: Livox SDK calibration (IMU frame offset from optical center)
/// - JT128: Reference frame (identity transform)
public enum RigSpec {
    
    // MARK: - Sensor Extrinsics
    
    /// Mid-360 extrinsics: transform from sensor frame to rig frame.
    /// Source: Livox SDK extrinsic calibration.
    /// The Mid-360's IMU frame is offset from the optical center by (11.0, 23.29, −44.12) mm.
    /// Since JT128 is fused on Mid-360's IMU frame, this offset aligns Mid-360 points to rig frame.
    public static let mid360Extrinsics = SensorExtrinsics(
        rotation: nil,  // Identity rotation (sensors are mechanically aligned)
        translation: (
            x: 0.011,      // 11.0 mm forward
            y: 0.02329,    // 23.29 mm left
            z: -0.04412    // 44.12 mm down
        )
    )
    
    /// JT128 extrinsics: identity (reference frame).
    /// The rig frame is defined at the JT128's optical center.
    public static let jt128Extrinsics = SensorExtrinsics.identity
    
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
            timestamp: frame.timestamp
        )
    }
    
    /// Transform multiple frames to rig frame.
    public static func transformToRigFrame(_ frames: [PointCloudFrame]) -> [PointCloudFrame] {
        frames.map { transformToRigFrame($0) }
    }
    
    // MARK: - Sensor Colors (for visualization)
    
    /// RGB color for Mid-360 points in sensor color mode.
    public static let mid360Color: (r: Float, g: Float, b: Float) = (0.0, 1.0, 1.0)  // Cyan
    
    /// RGB color for JT128 points in sensor color mode.
    public static let jt128Color: (r: Float, g: Float, b: Float) = (1.0, 0.0, 1.0)   // Magenta
    
    /// Get color for a sensor ID.
    public static func color(for sensor: SensorID) -> (r: Float, g: Float, b: Float) {
        switch sensor {
        case .mid360: return mid360Color
        case .jt128: return jt128Color
        }
    }
    
    /// Sensor ID as UInt32 for GPU rendering.
    public static func sensorIDForGPU(_ sensor: SensorID?) -> UInt32 {
        guard let sensor = sensor else { return 0 }
        switch sensor {
        case .mid360: return 1
        case .jt128: return 2
        }
    }
}
