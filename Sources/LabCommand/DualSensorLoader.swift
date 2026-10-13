import Foundation
import LabCommandCore

/// Loads and prepares dual-sensor point clouds for rendering.
/// Uses PointCloudDecoder from LabCommandCore and applies RigSpec extrinsics.
public enum DualSensorLoader {
    
    /// Standard fixture paths.
    public static let mid360FixturePath = "fixtures/pointclouds/mid360-frame.pcd"
    public static let jt128FixturePath = "fixtures/pointclouds/jt128-frame.pcd"
    public static let dualFixturePath = "fixtures/pointclouds/dual-frame.pcd"
    
    // MARK: - Loading
    
    /// Load both sensor fixtures with extrinsics applied.
    /// Returns frames ready for rendering.
    public static func loadDualSensorFixtures(
        mid360Path: String = mid360FixturePath,
        jt128Path: String = jt128FixturePath
    ) -> [PointCloudFrame] {
        var result: [PointCloudFrame] = []
        
        // Load Mid-360
        if let mid360Frame = loadFrame(from: mid360Path, sensor: .mid360) {
            let transformed = RigSpec.transformToRigFrame(mid360Frame)
            result.append(transformed)
        }
        
        // Load JT128
        if let jt128Frame = loadFrame(from: jt128Path, sensor: .jt128) {
            let transformed = RigSpec.transformToRigFrame(jt128Frame)
            result.append(transformed)
        }
        
        return result
    }
    
    /// Load a single sensor fixture with extrinsics applied.
    public static func loadSensorFixture(
        path: String,
        sensor: SensorID
    ) -> PointCloudFrame? {
        guard let frame = loadFrame(from: path, sensor: sensor) else {
            return nil
        }
        
        return RigSpec.transformToRigFrame(frame)
    }
    
    // MARK: - Frame Loading
    
    private static func loadFrame(from path: String, sensor: SensorID) -> PointCloudFrame? {
        do {
            var frame = try PointCloudDecoder.loadPCD(from: path)
            frame.sensor = sensor
            return frame
        } catch {
            print("DualSensorLoader: Failed to load \(path): \(error)")
            return nil
        }
    }
    
    // MARK: - Combined Operations
    
    /// Load frames and apply extrinsics.
    public static func loadAndTransform(_ frames: [PointCloudFrame]) -> [PointCloudFrame] {
        RigSpec.transformToRigFrame(frames)
    }
}
