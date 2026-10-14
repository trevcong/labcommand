import Foundation

// MARK: - PointCloud2 CDR Decoder

/// Decoder for CDR-encoded ROS 2 `sensor_msgs/PointCloud2` messages, as delivered
/// by foxglove_bridge.
///
/// Wire layout (alignment is relative to the byte after the 4-byte encapsulation header):
/// - encapsulation header: `0x00 0x01` = CDR little endian, `0x00 0x00` = CDR big endian, 2 option bytes
/// - std_msgs/Header: stamp (int32 sec, uint32 nanosec), frame_id (string)
/// - uint32 height, uint32 width
/// - PointField[] fields: string name, uint32 offset, uint8 datatype, uint32 count
/// - bool is_bigendian, uint32 point_step, uint32 row_step
/// - uint8[] data
/// - bool is_dense
///
/// Unlike `PointCloudDecoder.decodeROS2PointCloud2`, this is the live-stream path: it
/// decodes in place (slices welcome), ignores fields it doesn't need whatever their
/// datatype, tolerates trailing padding, and can decimate.
public enum PointCloud2CDR {

    /// Decode a CDR-encoded PointCloud2 message.
    ///
    /// - Parameters:
    ///   - data: Raw CDR bytes including the 4-byte encapsulation header.
    ///   - maxPoints: Optional cap on decoded points. When the cloud holds more, every
    ///     Nth point is taken (N = ceil(total / maxPoints)). Decimation runs before
    ///     NaN/Inf filtering, so the result may hold fewer than `maxPoints`. `nil` or
    ///     a non-positive value decodes every point.
    /// - Returns: A frame with `positions`, `intensities` (1.0 where the message has
    ///   no usable intensity), `frameId`, and `timestamp` set.
    /// - Throws: `PointCloudError` on truncated or malformed input.
    public static func decode(_ data: Data, maxPoints: Int? = nil) throws -> PointCloudFrame {
        try data.withUnsafeBytes { try decode(bytes: $0, maxPoints: maxPoints) }
    }

    private static func decode(bytes: UnsafeRawBufferPointer, maxPoints: Int?) throws -> PointCloudFrame {
        var reader = try CDRReader(bytes: bytes)

        let stampSec = try reader.readInt32("header.stamp.sec")
        let stampNanosec = try reader.readUInt32("header.stamp.nanosec")
        let frameId = try reader.readString("header.frame_id")
        let height = Int(try reader.readUInt32("height"))
        let width = Int(try reader.readUInt32("width"))

        var xField: RawField?
        var yField: RawField?
        var zField: RawField?
        var intensityField: RawField?

        // An absurd count runs out of bytes and throws long before it costs anything.
        let fieldCount = try reader.readUInt32("fields length")
        for index in 0..<fieldCount {
            let name = try reader.readString("fields[\(index)].name")
            let offset = Int(try reader.readUInt32("fields[\(index)].offset"))
            let datatype = try reader.readUInt8("fields[\(index)].datatype")
            _ = try reader.readUInt32("fields[\(index)].count")

            let field = RawField(offset: offset, datatype: datatype)
            switch name.lowercased() {
            case "x": xField = xField ?? field
            case "y": yField = yField ?? field
            case "z": zField = zField ?? field
            case "intensity": intensityField = intensityField ?? field
            default: break
            }
        }

        let isBigEndian = try reader.readUInt8("is_bigendian") != 0
        let pointStep = Int(try reader.readUInt32("point_step"))
        let declaredRowStep = Int(try reader.readUInt32("row_step"))
        let dataLength = Int(try reader.readUInt32("data length"))
        let dataStart = try reader.take(dataLength, "data")
        _ = try reader.readUInt8("is_dense")

        let frame = PointCloudFrame(
            timestamp: Date(timeIntervalSince1970: Double(stampSec) + Double(stampNanosec) / 1_000_000_000),
            frameId: frameId
        )

        let (totalPoints, countOverflow) = width.multipliedReportingOverflow(by: height)
        guard !countOverflow else {
            throw PointCloudError.invalidHeader("PointCloud2 CDR: width (\(width)) × height (\(height)) overflows")
        }
        if totalPoints == 0 {
            return frame
        }

        guard pointStep > 0 else {
            throw PointCloudError.invalidHeader("PointCloud2 CDR: point_step is 0 for a \(width)×\(height) cloud")
        }

        // row_step == 0 is tolerated as "rows are tightly packed".
        let (packedRowSize, rowOverflow) = pointStep.multipliedReportingOverflow(by: width)
        let rowStep = declaredRowStep > 0 ? declaredRowStep : packedRowSize
        guard !rowOverflow, packedRowSize <= rowStep else {
            throw PointCloudError.invalidHeader(
                "PointCloud2 CDR: point_step (\(pointStep)) × width (\(width)) exceeds row_step (\(rowStep))"
            )
        }
        let (requiredLength, sizeOverflow) = rowStep.multipliedReportingOverflow(by: height)
        guard !sizeOverflow, requiredLength <= dataLength else {
            throw PointCloudError.invalidData(
                "PointCloud2 CDR: data holds \(dataLength) bytes but row_step (\(rowStep)) × height (\(height)) needs more"
            )
        }

        let x = try positionReader(xField, name: "x", pointStep: pointStep, bigEndian: isBigEndian)
        let y = try positionReader(yField, name: "y", pointStep: pointStep, bigEndian: isBigEndian)
        let z = try positionReader(zField, name: "z", pointStep: pointStep, bigEndian: isBigEndian)
        let intensity = try intensityReader(intensityField, pointStep: pointStep, bigEndian: isBigEndian)

        guard let base = bytes.baseAddress else {
            return frame
        }
        let points = base + dataStart

        // The checks above bound totalPoints by the bytes actually present, and every
        // field read by point_step, so the unchecked loads below stay inside `data`.
        let stride = decimationStride(totalPoints: totalPoints, maxPoints: maxPoints)
        let rowPadding = rowStep - packedRowSize

        var positions: [Point3D] = []
        var intensities: [Float] = []
        let selected = (totalPoints - 1) / stride + 1
        positions.reserveCapacity(selected)
        intensities.reserveCapacity(selected)

        var index = 0
        while index < totalPoints {
            let rowOffset = rowPadding == 0 ? 0 : (index / width) * rowPadding
            let point = points + index * pointStep + rowOffset
            index += stride

            let px = x.read(point)
            let py = y.read(point)
            let pz = z.read(point)
            if !px.isFinite || !py.isFinite || !pz.isFinite { continue }

            positions.append(Point3D(x: px, y: py, z: pz))

            if let intensity {
                let value = intensity.read(point)
                intensities.append(value.isFinite ? value : 1.0)
            } else {
                intensities.append(1.0)
            }
        }

        var result = frame
        result.positions = positions
        result.intensities = intensities
        return result
    }

    /// Step between source points so that at most `maxPoints` are selected.
    private static func decimationStride(totalPoints: Int, maxPoints: Int?) -> Int {
        guard let maxPoints, maxPoints > 0, totalPoints > maxPoints else { return 1 }
        return (totalPoints - 1) / maxPoints + 1
    }

    private static func positionReader(
        _ field: RawField?,
        name: String,
        pointStep: Int,
        bigEndian: Bool
    ) throws -> ScalarReader {
        guard let field else {
            throw PointCloudError.invalidHeader("PointCloud2 CDR: missing required field '\(name)'")
        }
        let scalar: ScalarReader.Scalar
        switch field.datatype {
        case PointCloud2Layout.FieldType.float32.rawValue: scalar = .float32
        case PointCloud2Layout.FieldType.float64.rawValue: scalar = .float64
        default:
            throw PointCloudError.unsupportedFormat(
                "PointCloud2 CDR: field '\(name)' has datatype \(field.datatype); only FLOAT32 (7) and FLOAT64 (8) are supported"
            )
        }
        return try boundedReader(field, scalar: scalar, name: name, pointStep: pointStep, bigEndian: bigEndian)
    }

    /// Intensity is optional: a datatype we can't read means "no intensity", not a bad frame.
    private static func intensityReader(
        _ field: RawField?,
        pointStep: Int,
        bigEndian: Bool
    ) throws -> ScalarReader? {
        guard let field else { return nil }
        let scalar: ScalarReader.Scalar
        switch field.datatype {
        case PointCloud2Layout.FieldType.float32.rawValue: scalar = .float32
        case PointCloud2Layout.FieldType.float64.rawValue: scalar = .float64
        case PointCloud2Layout.FieldType.uint8.rawValue: scalar = .uint8
        case PointCloud2Layout.FieldType.uint16.rawValue: scalar = .uint16
        default: return nil
        }
        return try boundedReader(field, scalar: scalar, name: "intensity", pointStep: pointStep, bigEndian: bigEndian)
    }

    private static func boundedReader(
        _ field: RawField,
        scalar: ScalarReader.Scalar,
        name: String,
        pointStep: Int,
        bigEndian: Bool
    ) throws -> ScalarReader {
        guard field.offset <= pointStep - scalar.size else {
            throw PointCloudError.invalidHeader(
                "PointCloud2 CDR: field '\(name)' (offset \(field.offset), \(scalar.size) bytes) extends past point_step (\(pointStep))"
            )
        }
        return ScalarReader(offset: field.offset, scalar: scalar, bigEndian: bigEndian)
    }
}

// MARK: - Field Access

/// A PointField as read off the wire, before its datatype is known to be usable.
private struct RawField {
    var offset: Int
    var datatype: UInt8
}

/// Reads one scalar out of a point record as Float.
private struct ScalarReader {
    enum Scalar {
        case float32
        case float64
        case uint8
        case uint16

        var size: Int {
            switch self {
            case .uint8: return 1
            case .uint16: return 2
            case .float32: return 4
            case .float64: return 8
            }
        }
    }

    let offset: Int
    let scalar: Scalar
    let bigEndian: Bool

    @inline(__always)
    func read(_ point: UnsafeRawPointer) -> Float {
        switch scalar {
        case .float32:
            let raw = point.loadUnaligned(fromByteOffset: offset, as: UInt32.self)
            return Float(bitPattern: bigEndian ? UInt32(bigEndian: raw) : UInt32(littleEndian: raw))
        case .float64:
            let raw = point.loadUnaligned(fromByteOffset: offset, as: UInt64.self)
            return Float(Double(bitPattern: bigEndian ? UInt64(bigEndian: raw) : UInt64(littleEndian: raw)))
        case .uint8:
            return Float(point.load(fromByteOffset: offset, as: UInt8.self))
        case .uint16:
            let raw = point.loadUnaligned(fromByteOffset: offset, as: UInt16.self)
            return Float(bigEndian ? UInt16(bigEndian: raw) : UInt16(littleEndian: raw))
        }
    }
}

// MARK: - CDR Reader

/// Bounds-checked cursor over a CDR buffer. Every read either succeeds in bounds or throws.
private struct CDRReader {
    private static let encapsulationSize = 4

    private let bytes: UnsafeRawBufferPointer
    private let bigEndian: Bool
    private var offset = CDRReader.encapsulationSize

    init(bytes: UnsafeRawBufferPointer) throws {
        guard bytes.count >= CDRReader.encapsulationSize else {
            throw PointCloudError.invalidData(
                "PointCloud2 CDR: truncated encapsulation header (\(bytes.count) of \(CDRReader.encapsulationSize) bytes)"
            )
        }
        switch (bytes[0], bytes[1]) {
        case (0x00, 0x01): bigEndian = false
        case (0x00, 0x00): bigEndian = true
        default:
            throw PointCloudError.unsupportedFormat(
                String(format: "PointCloud2 CDR: unsupported encapsulation 0x%02x 0x%02x (expected CDR_LE 0x00 0x01 or CDR_BE 0x00 0x00)",
                       bytes[0], bytes[1])
            )
        }
        self.bytes = bytes
    }

    /// Consume `length` bytes and return the offset they start at.
    mutating func take(_ length: Int, _ what: @autoclosure () -> String) throws -> Int {
        guard length <= bytes.count - offset else {
            throw PointCloudError.invalidData(
                "PointCloud2 CDR: truncated \(what()) at byte \(offset) (need \(length), have \(max(0, bytes.count - offset)))"
            )
        }
        defer { offset += length }
        return offset
    }

    /// Pad to a 4-byte boundary, measured from the end of the encapsulation header.
    private mutating func align4() {
        let misalignment = (offset - CDRReader.encapsulationSize) % 4
        if misalignment != 0 {
            offset += 4 - misalignment
        }
    }

    mutating func readUInt8(_ what: @autoclosure () -> String) throws -> UInt8 {
        bytes[try take(1, what())]
    }

    mutating func readUInt32(_ what: @autoclosure () -> String) throws -> UInt32 {
        align4()
        let raw = bytes.loadUnaligned(fromByteOffset: try take(4, what()), as: UInt32.self)
        return bigEndian ? UInt32(bigEndian: raw) : UInt32(littleEndian: raw)
    }

    mutating func readInt32(_ what: @autoclosure () -> String) throws -> Int32 {
        Int32(bitPattern: try readUInt32(what()))
    }

    /// CDR string: uint32 length (including the NUL terminator) followed by the bytes.
    mutating func readString(_ what: @autoclosure () -> String) throws -> String {
        let length = Int(try readUInt32(what() + " length"))
        let start = try take(length, what())
        return String(decoding: bytes[start..<start + length].prefix(while: { $0 != 0 }), as: UTF8.self)
    }
}
