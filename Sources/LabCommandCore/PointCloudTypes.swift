import Foundation

/// A raw point cloud frame from a PCD file or PointCloud2 message.
/// This is the decoded representation before GPU upload.
public struct PointCloudFrame: Sendable, Equatable {
    /// Point positions in sensor frame (x, y, z).
    public var positions: [SIMD3<Float>]
    /// Intensity values [0, 1].
    public var intensities: [Float]
    /// Source sensor for coloring (nil if unknown or single-sensor).
    public var sensor: SensorID?
    /// Timestamp of the frame (nil for static fixtures).
    public var timestamp: Date?
    
    public init(
        positions: [SIMD3<Float>] = [],
        intensities: [Float] = [],
        sensor: SensorID? = nil,
        timestamp: Date? = nil
    ) {
        self.positions = positions
        self.intensities = intensities
        self.sensor = sensor
        self.timestamp = timestamp
    }
    
    /// Number of points in the frame.
    public var count: Int { positions.count }
    
    /// Whether the frame has no points.
    public var isEmpty: Bool { positions.isEmpty }
}

/// PCD file header metadata.
public struct PCDHeader: Sendable, Equatable {
    public var version: String
    public var fields: [String]
    public var width: Int
    public var height: Int
    public var points: Int
    public var dataFormat: PCDDataFormat
    
    public init(
        version: String = "0.7",
        fields: [String] = ["x", "y", "z", "intensity"],
        width: Int = 0,
        height: Int = 1,
        points: Int = 0,
        dataFormat: PCDDataFormat = .ascii
    ) {
        self.version = version
        self.fields = fields
        self.width = width
        self.height = height
        self.points = points
        self.dataFormat = dataFormat
    }
}

/// PCD file data format.
public enum PCDDataFormat: String, Sendable, Equatable {
    case ascii
    case binary
    case binaryCompressed = "binary_compressed"
}

/// Errors that can occur when parsing point cloud data.
public enum PointCloudError: Error, Sendable, Equatable {
    case invalidHeader(String)
    case invalidData(String)
    case unsupportedFormat(String)
    case fileNotFound(String)
}
