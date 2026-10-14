import Foundation
import LabCommandCore

/// Loads and prepares dual-sensor point clouds for rendering.
/// Uses PointCloudDecoder from LabCommandCore and applies RigSpec extrinsics.
///
/// **Transform Convention (R·p−t):**
/// Each sensor's extrinsics define `t` = the IMU origin in the lidar frame (from FAST-LIO mid360.yaml).
/// To bring points into the IMU frame: `p_imu = R * p_sensor - t`
///
/// **Resource Loading:**
/// For .app bundles, `build.sh` copies fixtures to `Contents/Resources/fixtures/pointclouds/`.
/// For development (`swift run`), relative paths from cwd are searched.
///
/// Search order:
/// 1. Bundle.main.resourcePath/fixtures/pointclouds/
/// 2. fixtures/pointclouds/, ../fixtures/pointclouds/, ../../fixtures/pointclouds/
public enum DualSensorLoader {
    
    /// Standard fixture filenames.
    public static let mid360FixtureName = "mid360-frame.pcd"
    public static let jt128FixtureName = "jt128-frame.pcd"
    
    /// Relative search paths for fixture files.
    private static let fixtureSearchPaths = [
        "fixtures/pointclouds",
        "../fixtures/pointclouds",
        "../../fixtures/pointclouds",
    ]
    
    // MARK: - Loading
    
    /// Load both sensor fixtures with extrinsics applied.
    /// Returns frames ready for rendering.
    public static func loadDualSensorFixtures() -> [PointCloudFrame] {
        var result: [PointCloudFrame] = []
        
        // Load Mid-360
        if let mid360Frame = loadFixture(named: mid360FixtureName, sensor: .mid360) {
            let transformed = RigSpec.transformToRigFrame(mid360Frame)
            result.append(transformed)
        }
        
        // Load JT128
        if let jt128Frame = loadFixture(named: jt128FixtureName, sensor: .jt128) {
            let transformed = RigSpec.transformToRigFrame(jt128Frame)
            result.append(transformed)
        }
        
        return result
    }
    
    /// Load a single sensor fixture with extrinsics applied.
    public static func loadSensorFixture(
        named filename: String,
        sensor: SensorID
    ) -> PointCloudFrame? {
        guard let frame = loadFixture(named: filename, sensor: sensor) else {
            return nil
        }
        
        return RigSpec.transformToRigFrame(frame)
    }
    
    // MARK: - Fixture Resolution
    
    /// Resolve a fixture filename to an absolute path using bundle + filesystem search.
    public static func resolveFixturePath(named filename: String) -> String? {
        let fm = FileManager.default
        
        // Try bundle resources first (for .app bundles)
        if let resourcePath = Bundle.main.resourcePath {
            let bundlePath = URL(fileURLWithPath: resourcePath)
                .appendingPathComponent("fixtures/pointclouds/\(filename)")
            if fm.fileExists(atPath: bundlePath.path) {
                return bundlePath.path
            }
        }
        
        // Try Bundle.main.path for embedded resources
        if let bundlePath = Bundle.main.path(forResource: filename.replacingOccurrences(of: ".pcd", with: ""),
                                              ofType: "pcd",
                                              inDirectory: "fixtures/pointclouds") {
            return bundlePath
        }
        
        // Try relative paths from current working directory
        let cwd = fm.currentDirectoryPath
        for searchDir in fixtureSearchPaths {
            let url = URL(fileURLWithPath: cwd)
                .appendingPathComponent(searchDir)
                .appendingPathComponent(filename)
            if fm.fileExists(atPath: url.path) {
                return url.path
            }
        }
        
        return nil
    }
    
    // MARK: - Frame Loading
    
    private static func loadFixture(named filename: String, sensor: SensorID) -> PointCloudFrame? {
        guard let path = resolveFixturePath(named: filename) else {
            print("DualSensorLoader: Could not resolve fixture: \(filename)")
            return nil
        }
        
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
    
    /// Transform existing frames to rig frame.
    public static func transformToRigFrame(_ frames: [PointCloudFrame]) -> [PointCloudFrame] {
        RigSpec.transformToRigFrame(frames)
    }
}
