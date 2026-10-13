import Foundation

/// A decoded point cloud with positions, intensities, and optional sensor origin.
public struct PointCloud: Sendable, Equatable {
    public var positions: [SIMD3<Float>]
    public var intensities: [Float]
    public var sensorID: String?

    public init(positions: [SIMD3<Float>] = [], intensities: [Float] = [], sensorID: String? = nil) {
        self.positions = positions
        self.intensities = intensities
        self.sensorID = sensorID
    }

    public var isEmpty: Bool { positions.isEmpty }
    public var count: Int { positions.count }
}

/// Field layout for ROS PointCloud2 binary messages.
public struct PointCloud2Layout: Sendable, Equatable {
    public var pointStep: Int
    public var xOffset: Int
    public var yOffset: Int
    public var zOffset: Int
    public var intensityOffset: Int?
    public var isBigEndian: Bool
    public var xType: FieldType
    public var yType: FieldType
    public var zType: FieldType
    public var intensityType: FieldType?

    public enum FieldType: Sendable, Equatable {
        case float32
        case float64
        case int8
        case uint8
        case int16
        case uint16
        case int32
        case uint32

        public var size: Int {
            switch self {
            case .int8, .uint8: return 1
            case .int16, .uint16: return 2
            case .float32, .int32, .uint32: return 4
            case .float64: return 8
            }
        }
    }

    public init(
        pointStep: Int,
        xOffset: Int,
        yOffset: Int,
        zOffset: Int,
        intensityOffset: Int? = nil,
        isBigEndian: Bool = false,
        xType: FieldType = .float32,
        yType: FieldType = .float32,
        zType: FieldType = .float32,
        intensityType: FieldType? = nil
    ) {
        self.pointStep = pointStep
        self.xOffset = xOffset
        self.yOffset = yOffset
        self.zOffset = zOffset
        self.intensityOffset = intensityOffset
        self.isBigEndian = isBigEndian
        self.xType = xType
        self.yType = yType
        self.zType = zType
        self.intensityType = intensityType
    }

    /// Common layout for Livox Mid-360 PointCloud2 messages (x y z intensity).
    public static let livoxMid360 = PointCloud2Layout(
        pointStep: 16,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8,
        intensityOffset: 12,
        isBigEndian: false,
        xType: .float32,
        yType: .float32,
        zType: .float32,
        intensityType: .float32
    )

    /// Common layout for Hesai JT128 PointCloud2 messages.
    public static let hesaiJT128 = PointCloud2Layout(
        pointStep: 32,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8,
        intensityOffset: 16,
        isBigEndian: false,
        xType: .float32,
        yType: .float32,
        zType: .float32,
        intensityType: .float32
    )
}

public enum PointCloudDecoderError: Error, Equatable {
    case invalidFormat(String)
    case missingHeader(String)
    case unsupportedDataFormat(String)
    case dataSizeMismatch(expected: Int, actual: Int)
    case invalidPointData(String)
}

/// Decoder for PCD (Point Cloud Data) files and ROS PointCloud2 binary messages.
public enum PointCloudDecoder {

    // MARK: - PCD ASCII Parsing

    /// Decode a PCD file (ASCII format) from raw data.
    public static func decodePCD(from data: Data) throws -> PointCloud {
        guard let content = String(data: data, encoding: .utf8) else {
            throw PointCloudDecoderError.invalidFormat("Data is not valid UTF-8")
        }
        return try decodePCD(from: content)
    }

    /// Decode a PCD file (ASCII format) from a string.
    public static func decodePCD(from content: String) throws -> PointCloud {
        let lines = content.components(separatedBy: .newlines)

        var version: String?
        var fields: [String] = []
        var _sizes: [Int] = []
        var _types: [String] = []
        var _counts: [Int] = []
        var width: Int?
        var height: Int?
        var dataFormat: String?
        var pointCount: Int?
        var headerEndIndex = 0

        for (index, line) in lines.enumerated() {
            let trimmed = line.trimmingCharacters(in: .whitespaces)

            if trimmed.isEmpty || trimmed.hasPrefix("#") {
                continue
            }

            let parts = trimmed.split(separator: " ", omittingEmptySubsequences: true)
                .map { String($0) }
            guard !parts.isEmpty else { continue }

            let key = parts[0].uppercased()
            let values = Array(parts.dropFirst())

            switch key {
            case "VERSION":
                version = values.first
            case "FIELDS":
                fields = values
            case "SIZE":
                _sizes = values.compactMap { Int($0) }
            case "TYPE":
                _types = values
            case "COUNT":
                _counts = values.compactMap { Int($0) }
            case "WIDTH":
                width = values.first.flatMap { Int($0) }
            case "HEIGHT":
                height = values.first.flatMap { Int($0) }
            case "VIEWPOINT":
                break
            case "POINTS":
                pointCount = values.first.flatMap { Int($0) }
            case "DATA":
                dataFormat = values.first?.lowercased()
                headerEndIndex = index + 1
            default:
                break
            }

            if dataFormat != nil { break }
        }

        guard version != nil else {
            throw PointCloudDecoderError.missingHeader("VERSION")
        }
        guard !fields.isEmpty else {
            throw PointCloudDecoderError.missingHeader("FIELDS")
        }
        guard let width else {
            throw PointCloudDecoderError.missingHeader("WIDTH")
        }
        guard let dataFormat else {
            throw PointCloudDecoderError.missingHeader("DATA")
        }

        let expectedPoints = pointCount ?? (width * (height ?? 1))

        if expectedPoints == 0 {
            return PointCloud(positions: [], intensities: [])
        }

        guard dataFormat == "ascii" else {
            throw PointCloudDecoderError.unsupportedDataFormat(
                "Only ASCII format is supported, got '\(dataFormat)'"
            )
        }

        let xIndex = fields.firstIndex { $0.lowercased() == "x" }
        let yIndex = fields.firstIndex { $0.lowercased() == "y" }
        let zIndex = fields.firstIndex { $0.lowercased() == "z" }
        let intensityIndex = fields.firstIndex { $0.lowercased() == "intensity" }

        guard let xIdx = xIndex, let yIdx = yIndex, let zIdx = zIndex else {
            throw PointCloudDecoderError.invalidFormat("Missing x, y, or z fields")
        }

        var positions: [SIMD3<Float>] = []
        var intensities: [Float] = []
        positions.reserveCapacity(expectedPoints)
        intensities.reserveCapacity(expectedPoints)

        let dataLines = Array(lines.dropFirst(headerEndIndex))

        for line in dataLines {
            let trimmed = line.trimmingCharacters(in: .whitespaces)
            if trimmed.isEmpty { continue }

            let values = trimmed.split(separator: " ", omittingEmptySubsequences: true)
            guard values.count > max(xIdx, yIdx, zIdx) else { continue }

            guard let x = Float(values[xIdx]),
                  let y = Float(values[yIdx]),
                  let z = Float(values[zIdx]) else {
                continue
            }

            if x.isNaN || y.isNaN || z.isNaN { continue }

            positions.append(SIMD3<Float>(x, y, z))

            if let intIdx = intensityIndex, values.count > intIdx,
               let intensity = Float(values[intIdx]), !intensity.isNaN {
                intensities.append(intensity)
            } else {
                intensities.append(1.0)
            }
        }

        return PointCloud(positions: positions, intensities: intensities)
    }

    // MARK: - PointCloud2 Binary Parsing

    /// Decode a ROS PointCloud2 binary message.
    public static func decodePointCloud2(
        from data: Data,
        fieldOffsets layout: PointCloud2Layout
    ) throws -> PointCloud {
        guard layout.pointStep > 0 else {
            throw PointCloudDecoderError.invalidFormat("pointStep must be positive")
        }

        let pointCount = data.count / layout.pointStep
        if pointCount == 0 {
            return PointCloud(positions: [], intensities: [])
        }

        var positions: [SIMD3<Float>] = []
        var intensities: [Float] = []
        positions.reserveCapacity(pointCount)
        intensities.reserveCapacity(pointCount)

        for i in 0..<pointCount {
            let baseOffset = i * layout.pointStep

            let x = try readFloat(
                from: data,
                at: baseOffset + layout.xOffset,
                type: layout.xType,
                bigEndian: layout.isBigEndian
            )
            let y = try readFloat(
                from: data,
                at: baseOffset + layout.yOffset,
                type: layout.yType,
                bigEndian: layout.isBigEndian
            )
            let z = try readFloat(
                from: data,
                at: baseOffset + layout.zOffset,
                type: layout.zType,
                bigEndian: layout.isBigEndian
            )

            if x.isNaN || y.isNaN || z.isNaN { continue }

            positions.append(SIMD3<Float>(x, y, z))

            if let intOffset = layout.intensityOffset, let intType = layout.intensityType {
                let intensity = try readFloat(
                    from: data,
                    at: baseOffset + intOffset,
                    type: intType,
                    bigEndian: layout.isBigEndian
                )
                intensities.append(intensity.isNaN ? 1.0 : intensity)
            } else {
                intensities.append(1.0)
            }
        }

        return PointCloud(positions: positions, intensities: intensities)
    }

    /// Decode a ROS PointCloud2 message with header parsing.
    /// The header format follows ROS message serialization:
    /// - uint32 height
    /// - uint32 width
    /// - uint32 num_fields
    /// - For each field: string name, uint32 offset, uint8 datatype, uint32 count
    /// - bool is_bigendian
    /// - uint32 point_step
    /// - uint32 row_step
    /// - byte[] data
    /// - bool is_dense
    public static func decodePointCloud2WithHeader(from data: Data) throws -> PointCloud {
        var offset = 0

        guard data.count >= 12 else {
            throw PointCloudDecoderError.invalidFormat("Data too short for PointCloud2 header")
        }

        let height = readUInt32LE(from: data, at: offset)
        offset += 4
        let width = readUInt32LE(from: data, at: offset)
        offset += 4

        let numFields = Int(readUInt32LE(from: data, at: offset))
        offset += 4

        var fieldMap: [String: (offset: Int, type: PointCloud2Layout.FieldType)] = [:]

        for _ in 0..<numFields {
            guard offset + 4 <= data.count else {
                throw PointCloudDecoderError.invalidFormat("Truncated field header")
            }

            let nameLen = Int(readUInt32LE(from: data, at: offset))
            offset += 4

            guard offset + nameLen + 9 <= data.count else {
                throw PointCloudDecoderError.invalidFormat("Truncated field data")
            }

            let nameData = data.subdata(in: offset..<(offset + nameLen))
            let name = String(data: nameData, encoding: .utf8)?.lowercased() ?? ""
            offset += nameLen

            let fieldOffset = Int(readUInt32LE(from: data, at: offset))
            offset += 4

            let datatype = data[offset]
            offset += 1

            _ = readUInt32LE(from: data, at: offset)
            offset += 4

            let fieldType = datatypeToFieldType(datatype)
            fieldMap[name] = (fieldOffset, fieldType)
        }

        guard offset + 13 <= data.count else {
            throw PointCloudDecoderError.invalidFormat("Truncated PointCloud2 metadata")
        }

        let isBigEndian = data[offset] != 0
        offset += 1

        let pointStep = Int(readUInt32LE(from: data, at: offset))
        offset += 4

        _ = readUInt32LE(from: data, at: offset)
        offset += 4

        let dataLen = Int(readUInt32LE(from: data, at: offset))
        offset += 4

        guard offset + dataLen <= data.count else {
            throw PointCloudDecoderError.dataSizeMismatch(
                expected: offset + dataLen,
                actual: data.count
            )
        }

        let pointData = data.subdata(in: offset..<(offset + dataLen))

        guard let xField = fieldMap["x"],
              let yField = fieldMap["y"],
              let zField = fieldMap["z"] else {
            throw PointCloudDecoderError.invalidFormat("Missing x, y, or z fields in PointCloud2")
        }

        let intensityField = fieldMap["intensity"]

        let layout = PointCloud2Layout(
            pointStep: pointStep,
            xOffset: xField.offset,
            yOffset: yField.offset,
            zOffset: zField.offset,
            intensityOffset: intensityField?.offset,
            isBigEndian: isBigEndian,
            xType: xField.type,
            yType: yField.type,
            zType: zField.type,
            intensityType: intensityField?.type
        )

        let cloud = try decodePointCloud2(from: pointData, fieldOffsets: layout)

        let expectedPoints = Int(height) * Int(width)
        if cloud.count != expectedPoints && expectedPoints > 0 {
        }

        return cloud
    }

    // MARK: - Helper Functions

    private static func readFloat(
        from data: Data,
        at offset: Int,
        type: PointCloud2Layout.FieldType,
        bigEndian: Bool
    ) throws -> Float {
        guard offset + type.size <= data.count else {
            throw PointCloudDecoderError.invalidPointData(
                "Offset \(offset) + size \(type.size) exceeds data length \(data.count)"
            )
        }

        switch type {
        case .float32:
            return readFloat32(from: data, at: offset, bigEndian: bigEndian)
        case .float64:
            return Float(readFloat64(from: data, at: offset, bigEndian: bigEndian))
        case .int8:
            return Float(Int8(bitPattern: data[offset]))
        case .uint8:
            return Float(data[offset])
        case .int16:
            return Float(readInt16(from: data, at: offset, bigEndian: bigEndian))
        case .uint16:
            return Float(readUInt16(from: data, at: offset, bigEndian: bigEndian))
        case .int32:
            return Float(readInt32(from: data, at: offset, bigEndian: bigEndian))
        case .uint32:
            return Float(readUInt32(from: data, at: offset, bigEndian: bigEndian))
        }
    }

    private static func readFloat32(from data: Data, at offset: Int, bigEndian: Bool) -> Float {
        let bytes = data.subdata(in: offset..<(offset + 4))
        var value: UInt32 = 0
        if bigEndian {
            value = UInt32(bytes[0]) << 24 | UInt32(bytes[1]) << 16 |
                    UInt32(bytes[2]) << 8 | UInt32(bytes[3])
        } else {
            value = UInt32(bytes[3]) << 24 | UInt32(bytes[2]) << 16 |
                    UInt32(bytes[1]) << 8 | UInt32(bytes[0])
        }
        return Float(bitPattern: value)
    }

    private static func readFloat64(from data: Data, at offset: Int, bigEndian: Bool) -> Double {
        let bytes = data.subdata(in: offset..<(offset + 8))
        var value: UInt64 = 0
        if bigEndian {
            for i in 0..<8 { value = value << 8 | UInt64(bytes[i]) }
        } else {
            for i in (0..<8).reversed() { value = value << 8 | UInt64(bytes[i]) }
        }
        return Double(bitPattern: value)
    }

    private static func readInt16(from data: Data, at offset: Int, bigEndian: Bool) -> Int16 {
        let u = readUInt16(from: data, at: offset, bigEndian: bigEndian)
        return Int16(bitPattern: u)
    }

    private static func readUInt16(from data: Data, at offset: Int, bigEndian: Bool) -> UInt16 {
        if bigEndian {
            return UInt16(data[offset]) << 8 | UInt16(data[offset + 1])
        } else {
            return UInt16(data[offset + 1]) << 8 | UInt16(data[offset])
        }
    }

    private static func readInt32(from data: Data, at offset: Int, bigEndian: Bool) -> Int32 {
        let u = readUInt32(from: data, at: offset, bigEndian: bigEndian)
        return Int32(bitPattern: u)
    }

    private static func readUInt32(from data: Data, at offset: Int, bigEndian: Bool) -> UInt32 {
        if bigEndian {
            return UInt32(data[offset]) << 24 | UInt32(data[offset + 1]) << 16 |
                   UInt32(data[offset + 2]) << 8 | UInt32(data[offset + 3])
        } else {
            return UInt32(data[offset + 3]) << 24 | UInt32(data[offset + 2]) << 16 |
                   UInt32(data[offset + 1]) << 8 | UInt32(data[offset])
        }
    }

    private static func readUInt32LE(from data: Data, at offset: Int) -> UInt32 {
        readUInt32(from: data, at: offset, bigEndian: false)
    }

    private static func datatypeToFieldType(_ datatype: UInt8) -> PointCloud2Layout.FieldType {
        switch datatype {
        case 1: return .int8
        case 2: return .uint8
        case 3: return .int16
        case 4: return .uint16
        case 5: return .int32
        case 6: return .uint32
        case 7: return .float32
        case 8: return .float64
        default: return .float32
        }
    }
}
