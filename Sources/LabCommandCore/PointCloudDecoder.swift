import Foundation

/// Field layout for ROS PointCloud2 binary messages.
/// Derive from message fields using `PointCloud2Layout.from(fields:pointStep:isBigEndian:)`.
public struct PointCloud2Layout: Sendable, Equatable {
    public var pointStep: Int
    public var rowStep: Int
    public var xOffset: Int
    public var yOffset: Int
    public var zOffset: Int
    public var intensityOffset: Int?
    public var isBigEndian: Bool
    public var xType: FieldType
    public var yType: FieldType
    public var zType: FieldType
    public var intensityType: FieldType?

    public enum FieldType: UInt8, Sendable, Equatable {
        case int8 = 1
        case uint8 = 2
        case int16 = 3
        case uint16 = 4
        case int32 = 5
        case uint32 = 6
        case float32 = 7
        case float64 = 8

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
        rowStep: Int = 0,
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
        self.rowStep = rowStep > 0 ? rowStep : pointStep
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

    /// A single field descriptor from a PointCloud2 message.
    public struct FieldDescriptor: Sendable, Equatable {
        public var name: String
        public var offset: Int
        public var datatype: FieldType
        public var count: Int

        public init(name: String, offset: Int, datatype: FieldType, count: Int = 1) {
            self.name = name
            self.offset = offset
            self.datatype = datatype
            self.count = count
        }
    }

    /// Derive layout from a PointCloud2 fields array.
    public static func from(
        fields: [FieldDescriptor],
        pointStep: Int,
        rowStep: Int = 0,
        isBigEndian: Bool = false
    ) throws -> PointCloud2Layout {
        var xField: FieldDescriptor?
        var yField: FieldDescriptor?
        var zField: FieldDescriptor?
        var intensityField: FieldDescriptor?

        for field in fields {
            switch field.name.lowercased() {
            case "x": xField = field
            case "y": yField = field
            case "z": zField = field
            case "intensity": intensityField = field
            default: break
            }
        }

        guard let x = xField, let y = yField, let z = zField else {
            throw PointCloudError.invalidHeader("Missing required x, y, or z fields")
        }

        return PointCloud2Layout(
            pointStep: pointStep,
            rowStep: rowStep,
            xOffset: x.offset,
            yOffset: y.offset,
            zOffset: z.offset,
            intensityOffset: intensityField?.offset,
            isBigEndian: isBigEndian,
            xType: x.datatype,
            yType: y.datatype,
            zType: z.datatype,
            intensityType: intensityField?.datatype
        )
    }
}

/// Extended PCD header with SIZE, TYPE, and COUNT metadata.
public struct PCDHeaderExtended: Sendable, Equatable {
    public var version: String
    public var fields: [String]
    public var sizes: [Int]
    public var types: [String]
    public var counts: [Int]
    public var width: Int
    public var height: Int
    public var points: Int
    public var dataFormat: PCDDataFormat

    public init(
        version: String = "0.7",
        fields: [String] = [],
        sizes: [Int] = [],
        types: [String] = [],
        counts: [Int] = [],
        width: Int = 0,
        height: Int = 1,
        points: Int = 0,
        dataFormat: PCDDataFormat = .ascii
    ) {
        self.version = version
        self.fields = fields
        self.sizes = sizes
        self.types = types
        self.counts = counts
        self.width = width
        self.height = height
        self.points = points
        self.dataFormat = dataFormat
    }

    /// Total number of values per point (sum of counts, or field count if counts empty).
    public var valuesPerPoint: Int {
        if counts.isEmpty {
            return fields.count
        }
        return counts.reduce(0, +)
    }
}

/// Decoder for PCD (Point Cloud Data) files and ROS PointCloud2 binary messages.
///
/// **Supported formats:**
/// - PCD ASCII format (VERSION 0.7)
/// - ROS1 PointCloud2 binary messages
/// - ROS2 PointCloud2 binary messages (with CDR encapsulation)
///
/// **Not yet supported:**
/// - PCD binary format
/// - PCD binary_compressed format
public enum PointCloudDecoder {

    /// Maximum points to allocate (prevents DoS via malicious headers).
    public static let maxPoints = 100_000_000

    // MARK: - PCD ASCII Parsing

    /// Result of PCD decoding including header metadata.
    public struct PCDResult: Sendable, Equatable {
        public var frame: PointCloudFrame
        public var header: PCDHeaderExtended
    }

    /// Decode a PCD file (ASCII format) from raw data.
    public static func decodePCD(from data: Data, filename: String? = nil) throws -> PointCloudFrame {
        guard let content = String(data: data, encoding: .utf8) else {
            throw PointCloudError.invalidData("Data is not valid UTF-8")
        }
        return try decodePCD(from: content, filename: filename)
    }

    /// Decode a PCD file with full header metadata.
    public static func decodePCDWithHeader(from content: String, filename: String? = nil) throws -> PCDResult {
        let lines = content.components(separatedBy: .newlines)
        let header = try parsePCDHeader(lines: lines)
        let frame = try parsePCDData(lines: lines, header: header, filename: filename)
        return PCDResult(frame: frame, header: header)
    }

    /// Decode a PCD file (ASCII format) from a string.
    public static func decodePCD(from content: String, filename: String? = nil) throws -> PointCloudFrame {
        let result = try decodePCDWithHeader(from: content, filename: filename)
        return result.frame
    }

    private static func parsePCDHeader(lines: [String]) throws -> PCDHeaderExtended {
        var version: String?
        var fields: [String] = []
        var sizes: [Int] = []
        var types: [String] = []
        var counts: [Int] = []
        var width: Int?
        var height: Int = 1
        var dataFormat: PCDDataFormat?
        var pointCount: Int?

        for line in lines {
            let trimmed = line.trimmingCharacters(in: .whitespacesAndNewlines)
            if trimmed.isEmpty || trimmed.hasPrefix("#") {
                continue
            }

            let parts = trimmed.split(whereSeparator: { $0.isWhitespace })
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
                sizes = values.compactMap { Int($0) }
            case "TYPE":
                types = values
            case "COUNT":
                counts = values.compactMap { Int($0) }
            case "WIDTH":
                if let w = values.first.flatMap({ Int($0) }), w >= 0 {
                    width = w
                } else if let w = values.first.flatMap({ Int($0) }), w < 0 {
                    throw PointCloudError.invalidHeader("WIDTH cannot be negative")
                }
            case "HEIGHT":
                if let h = values.first.flatMap({ Int($0) }), h >= 0 {
                    height = h
                } else if let h = values.first.flatMap({ Int($0) }), h < 0 {
                    throw PointCloudError.invalidHeader("HEIGHT cannot be negative")
                }
            case "VIEWPOINT":
                break
            case "POINTS":
                if let p = values.first.flatMap({ Int($0) }), p >= 0 {
                    pointCount = p
                } else if let p = values.first.flatMap({ Int($0) }), p < 0 {
                    throw PointCloudError.invalidHeader("POINTS cannot be negative")
                }
            case "DATA":
                if let format = values.first?.lowercased() {
                    dataFormat = PCDDataFormat(rawValue: format)
                }
            default:
                break
            }

            if dataFormat != nil { break }
        }

        guard version != nil else {
            throw PointCloudError.invalidHeader("Missing VERSION header")
        }
        guard !fields.isEmpty else {
            throw PointCloudError.invalidHeader("Missing FIELDS header")
        }
        guard let dataFormat else {
            throw PointCloudError.invalidHeader("Missing DATA header")
        }

        let effectiveWidth = width ?? pointCount ?? 0
        let effectivePoints = pointCount ?? (effectiveWidth * height)

        if effectivePoints < 0 {
            throw PointCloudError.invalidHeader("Computed point count is negative")
        }

        let (overflowCheck, overflow) = effectiveWidth.multipliedReportingOverflow(by: height)
        if overflow || overflowCheck < 0 {
            throw PointCloudError.invalidHeader("WIDTH * HEIGHT overflows")
        }

        return PCDHeaderExtended(
            version: version ?? "0.7",
            fields: fields,
            sizes: sizes,
            types: types,
            counts: counts,
            width: effectiveWidth,
            height: height,
            points: effectivePoints,
            dataFormat: dataFormat
        )
    }

    private static func parsePCDData(
        lines: [String],
        header: PCDHeaderExtended,
        filename: String?
    ) throws -> PointCloudFrame {
        guard header.dataFormat == .ascii else {
            throw PointCloudError.unsupportedFormat(
                "Only ASCII format is supported. Binary PCD format is not yet implemented."
            )
        }

        if header.points == 0 {
            return PointCloudFrame(sensor: sensorFromFilename(filename))
        }

        guard header.points <= maxPoints else {
            throw PointCloudError.invalidData(
                "Point count \(header.points) exceeds maximum \(maxPoints)"
            )
        }

        var headerEndIndex = 0
        for (index, line) in lines.enumerated() {
            let trimmed = line.trimmingCharacters(in: .whitespacesAndNewlines)
            if trimmed.isEmpty || trimmed.hasPrefix("#") { continue }
            let key = trimmed.split(whereSeparator: { $0.isWhitespace }).first?.uppercased()
            if key == "DATA" {
                headerEndIndex = index + 1
                break
            }
        }

        let fieldCounts = header.counts.isEmpty
            ? Array(repeating: 1, count: header.fields.count)
            : header.counts

        var valueIndices: [String: Int] = [:]
        var currentIndex = 0
        for (i, field) in header.fields.enumerated() {
            valueIndices[field.lowercased()] = currentIndex
            currentIndex += (i < fieldCounts.count) ? fieldCounts[i] : 1
        }

        let xIdx = valueIndices["x"]
        let yIdx = valueIndices["y"]
        let zIdx = valueIndices["z"]
        let intensityIdx = valueIndices["intensity"]

        guard let xIndex = xIdx, let yIndex = yIdx, let zIndex = zIdx else {
            throw PointCloudError.invalidHeader("Missing required x, y, or z fields")
        }

        var positions: [SIMD3<Float>] = []
        var intensities: [Float] = []

        let dataLines = lines.dropFirst(headerEndIndex)
        var parsedPoints = 0

        for line in dataLines {
            let trimmed = line.trimmingCharacters(in: .whitespacesAndNewlines)
            if trimmed.isEmpty { continue }

            let values = trimmed.split(whereSeparator: { $0.isWhitespace })
            let maxIndex = max(xIndex, yIndex, zIndex, intensityIdx ?? 0)
            guard values.count > maxIndex else { continue }

            guard let x = Float(values[xIndex]),
                  let y = Float(values[yIndex]),
                  let z = Float(values[zIndex]) else {
                continue
            }

            if !x.isFinite || !y.isFinite || !z.isFinite { continue }

            positions.append(SIMD3<Float>(x, y, z))

            if let intIdx = intensityIdx, values.count > intIdx,
               let intensity = Float(values[intIdx]), intensity.isFinite {
                intensities.append(intensity)
            } else {
                intensities.append(1.0)
            }

            parsedPoints += 1
        }

        if parsedPoints < header.points {
            throw PointCloudError.invalidData(
                "Truncated file: expected \(header.points) points but found \(parsedPoints)"
            )
        }

        return PointCloudFrame(
            positions: positions,
            intensities: intensities,
            sensor: sensorFromFilename(filename)
        )
    }

    private static func sensorFromFilename(_ filename: String?) -> SensorID? {
        guard let name = filename?.lowercased() else { return nil }
        if name.contains("mid360") || name.contains("mid-360") || name.contains("livox") {
            return .mid360
        }
        if name.contains("jt128") || name.contains("hesai") {
            return .jt128
        }
        return nil
    }

    // MARK: - PointCloud2 Binary Parsing

    /// Decode a ROS PointCloud2 binary message with explicit layout.
    public static func decodePointCloud2(
        from data: Data,
        layout: PointCloud2Layout,
        pointCount: Int? = nil
    ) throws -> PointCloudFrame {
        guard layout.pointStep > 0 else {
            throw PointCloudError.invalidData("pointStep must be positive")
        }

        let count = pointCount ?? (data.count / layout.pointStep)
        if count == 0 {
            return PointCloudFrame()
        }

        guard count <= maxPoints else {
            throw PointCloudError.invalidData(
                "Point count \(count) exceeds maximum \(maxPoints)"
            )
        }

        var positions: [SIMD3<Float>] = []
        var intensities: [Float] = []
        positions.reserveCapacity(count)
        intensities.reserveCapacity(count)

        try data.withUnsafeBytes { buffer in
            let baseAddress = buffer.baseAddress!

            for i in 0..<count {
                let baseOffset = i * layout.pointStep

                guard baseOffset + layout.xOffset + layout.xType.size <= data.count,
                      baseOffset + layout.yOffset + layout.yType.size <= data.count,
                      baseOffset + layout.zOffset + layout.zType.size <= data.count else {
                    break
                }

                let x = readFloatUnsafe(
                    from: baseAddress,
                    at: baseOffset + layout.xOffset,
                    type: layout.xType,
                    bigEndian: layout.isBigEndian
                )
                let y = readFloatUnsafe(
                    from: baseAddress,
                    at: baseOffset + layout.yOffset,
                    type: layout.yType,
                    bigEndian: layout.isBigEndian
                )
                let z = readFloatUnsafe(
                    from: baseAddress,
                    at: baseOffset + layout.zOffset,
                    type: layout.zType,
                    bigEndian: layout.isBigEndian
                )

                if !x.isFinite || !y.isFinite || !z.isFinite { continue }

                positions.append(SIMD3<Float>(x, y, z))

                if let intOffset = layout.intensityOffset, let intType = layout.intensityType,
                   baseOffset + intOffset + intType.size <= data.count {
                    let intensity = readFloatUnsafe(
                        from: baseAddress,
                        at: baseOffset + intOffset,
                        type: intType,
                        bigEndian: layout.isBigEndian
                    )
                    intensities.append(intensity.isFinite ? intensity : 1.0)
                } else {
                    intensities.append(1.0)
                }
            }
        }

        return PointCloudFrame(positions: positions, intensities: intensities)
    }

    /// Decode a ROS1 PointCloud2 message (fields array embedded in message).
    public static func decodeROS1PointCloud2(from data: Data) throws -> PointCloudFrame {
        guard data.count >= 12 else {
            throw PointCloudError.invalidData("Data too short for ROS1 PointCloud2 header")
        }

        var offset = 0

        let height = try readUInt32Safe(from: data, at: offset)
        offset += 4
        let width = try readUInt32Safe(from: data, at: offset)
        offset += 4

        let (totalPoints, overflow) = Int(width).multipliedReportingOverflow(by: Int(height))
        if overflow {
            throw PointCloudError.invalidHeader("width * height overflows")
        }

        let numFields = try readUInt32Safe(from: data, at: offset)
        offset += 4

        var fields: [PointCloud2Layout.FieldDescriptor] = []

        for _ in 0..<numFields {
            guard offset + 4 <= data.count else {
                throw PointCloudError.invalidData("Truncated field header")
            }

            let nameLen = Int(try readUInt32Safe(from: data, at: offset))
            offset += 4

            guard offset + nameLen + 9 <= data.count else {
                throw PointCloudError.invalidData("Truncated field data")
            }

            let name: String
            if nameLen > 0 {
                let nameData = data[data.startIndex.advanced(by: offset)..<data.startIndex.advanced(by: offset + nameLen)]
                name = String(data: nameData, encoding: .utf8)?.lowercased() ?? ""
            } else {
                name = ""
            }
            offset += nameLen

            let fieldOffset = Int(try readUInt32Safe(from: data, at: offset))
            offset += 4

            let datatype = data[data.startIndex.advanced(by: offset)]
            offset += 1

            let count = Int(try readUInt32Safe(from: data, at: offset))
            offset += 4

            guard let fieldType = PointCloud2Layout.FieldType(rawValue: datatype) else {
                throw PointCloudError.invalidData("Unknown field datatype: \(datatype)")
            }

            fields.append(PointCloud2Layout.FieldDescriptor(
                name: name,
                offset: fieldOffset,
                datatype: fieldType,
                count: count
            ))
        }

        guard offset + 10 <= data.count else {
            throw PointCloudError.invalidData("Truncated PointCloud2 metadata")
        }

        let isBigEndian = data[data.startIndex.advanced(by: offset)] != 0
        offset += 1

        let pointStep = Int(try readUInt32Safe(from: data, at: offset))
        offset += 4

        let rowStep = Int(try readUInt32Safe(from: data, at: offset))
        offset += 4

        let dataLen = Int(try readUInt32Safe(from: data, at: offset))
        offset += 4

        guard offset + dataLen <= data.count else {
            throw PointCloudError.invalidData(
                "Data section truncated: expected \(dataLen) bytes at offset \(offset), have \(data.count)"
            )
        }

        let pointData = data[data.startIndex.advanced(by: offset)..<data.startIndex.advanced(by: offset + dataLen)]

        let layout = try PointCloud2Layout.from(
            fields: fields,
            pointStep: pointStep,
            rowStep: rowStep,
            isBigEndian: isBigEndian
        )

        return try decodePointCloud2(from: Data(pointData), layout: layout, pointCount: totalPoints)
    }

    /// Decode a ROS2 PointCloud2 message (with CDR encapsulation header).
    public static func decodeROS2PointCloud2(from data: Data) throws -> PointCloudFrame {
        guard data.count >= 4 else {
            throw PointCloudError.invalidData("Data too short for ROS2 CDR header")
        }

        let encapsulation = data[data.startIndex]
        guard encapsulation == 0x00 else {
            throw PointCloudError.invalidData("Invalid CDR encapsulation byte: \(encapsulation)")
        }

        let withHeader = data[data.startIndex.advanced(by: 4)...]
        return try decodeROS1PointCloud2(from: Data(withHeader))
    }

    // MARK: - Safe Read Helpers

    private static func readUInt32Safe(from data: Data, at offset: Int) throws -> UInt32 {
        let start = data.startIndex.advanced(by: offset)
        guard start >= data.startIndex, offset + 4 <= data.count else {
            throw PointCloudError.invalidData("Read offset \(offset) out of bounds")
        }
        let b0 = UInt32(data[start])
        let b1 = UInt32(data[data.index(after: start)])
        let b2 = UInt32(data[data.index(start, offsetBy: 2)])
        let b3 = UInt32(data[data.index(start, offsetBy: 3)])
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    }

    private static func readFloatUnsafe(
        from base: UnsafeRawPointer,
        at offset: Int,
        type: PointCloud2Layout.FieldType,
        bigEndian: Bool
    ) -> Float {
        let ptr = base.advanced(by: offset)

        switch type {
        case .float32:
            var value = ptr.loadUnaligned(as: UInt32.self)
            if bigEndian { value = value.byteSwapped }
            return Float(bitPattern: value)

        case .float64:
            var value = ptr.loadUnaligned(as: UInt64.self)
            if bigEndian { value = value.byteSwapped }
            return Float(Double(bitPattern: value))

        case .int8:
            return Float(ptr.load(as: Int8.self))

        case .uint8:
            return Float(ptr.load(as: UInt8.self))

        case .int16:
            var value = ptr.loadUnaligned(as: Int16.self)
            if bigEndian { value = Int16(bitPattern: UInt16(bitPattern: value).byteSwapped) }
            return Float(value)

        case .uint16:
            var value = ptr.loadUnaligned(as: UInt16.self)
            if bigEndian { value = value.byteSwapped }
            return Float(value)

        case .int32:
            var value = ptr.loadUnaligned(as: Int32.self)
            if bigEndian { value = Int32(bitPattern: UInt32(bitPattern: value).byteSwapped) }
            return Float(value)

        case .uint32:
            var value = ptr.loadUnaligned(as: UInt32.self)
            if bigEndian { value = value.byteSwapped }
            return Float(value)
        }
    }
}

// MARK: - Convenience Extensions

extension PointCloudDecoder {
    /// Load a PCD file from disk.
    public static func loadPCD(from path: String) throws -> PointCloudFrame {
        let url = URL(fileURLWithPath: path)
        let data = try Data(contentsOf: url)
        let filename = url.lastPathComponent
        return try decodePCD(from: data, filename: filename)
    }
}
