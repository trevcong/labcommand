import Foundation

/// A registered map cloud read from `map.pcd`. ASCII and binary PCD v0.7.
///
/// `PointCloudDecoder` stays ASCII-only on purpose (its tests pin that). This reader parses
/// the header as bytes, so binary data after `DATA binary` is never run through UTF-8.
/// Field offsets come from SIZE and COUNT: FAST-LIO's PointXYZINormal (x y z intensity
/// normal_x normal_y normal_z curvature, 32 bytes) and plain x y z (12 bytes) both read.
public struct TrailMapCloud: Equatable, Sendable {
    public enum Format: String, Equatable, Sendable {
        case ascii
        case binary
    }

    public var points: [Point3D]
    public var format: Format
    /// POINTS from the header.
    public var declared: Int
    /// Points with a NaN, infinite, or out-of-bounds (`TrailBounds`) coordinate. They are left out:
    /// one stray return does not refuse the whole map.
    public var dropped: Int
    /// 1 keeps every point. N keeps one in N (a map bigger than `maxKept`).
    public var stride: Int

    /// Above this many points the reader keeps one in N, evenly. The top-down view bins
    /// to a few centimetres, so a 40 M point map gains nothing from every point.
    public static let maxKept = 4_000_000
    /// A header longer than this is not a PCD header.
    static let maxHeaderBytes = 64 * 1024
    /// Same cap as `PointCloudDecoder.maxPoints`. A larger POINTS is a broken header.
    static let maxDeclared = 100_000_000

    public static func load(path: String, maxKept: Int = maxKept) throws -> TrailMapCloud {
        let data: Data
        do {
            data = try Data(contentsOf: URL(fileURLWithPath: path), options: .mappedIfSafe)
        } catch {
            throw TrailError.pcd("cannot read \(path)")
        }
        return try decode(data, maxKept: maxKept)
    }

    struct Field {
        var name: String
        var size: Int
        var type: Character
        var count: Int
        var offset: Int
        /// Index of the first value of this field on an ASCII line.
        var column: Int
    }

    struct Header {
        var fields: [Field]
        var points: Int
        var format: String
        /// Byte offset of the first data byte.
        var dataStart: Int
        var stride: Int
    }

    public static func decode(_ data: Data, maxKept: Int = maxKept) throws -> TrailMapCloud {
        let header = try parseHeader(data)
        guard let x = header.fields.first(where: { $0.name == "x" }),
              let y = header.fields.first(where: { $0.name == "y" }),
              let z = header.fields.first(where: { $0.name == "z" }) else {
            throw TrailError.pcd("FIELDS must include x, y and z")
        }
        for field in [x, y, z] where field.type != "F" || (field.size != 4 && field.size != 8) {
            throw TrailError.pcd("field \(field.name) must be TYPE F with SIZE 4 or 8")
        }
        let keep = maxKept > 0 && header.points > maxKept ? (header.points - 1) / maxKept + 1 : 1
        switch header.format {
        case "binary":
            return try decodeBinary(data, header: header, x: x, y: y, z: z, keep: keep)
        case "ascii":
            return try decodeASCII(data, header: header, x: x, y: y, z: z, keep: keep)
        case "binary_compressed":
            throw TrailError.pcd("DATA binary_compressed is not supported; save the map as binary or ascii")
        default:
            throw TrailError.pcd("unknown DATA format '\(header.format)'")
        }
    }

    static func parseHeader(_ data: Data) throws -> Header {
        var values: [String: [String]] = [:]
        var cursor = data.startIndex
        var dataStart: Int?
        var format = ""
        let limit = min(data.endIndex, data.startIndex + maxHeaderBytes)
        while cursor < limit {
            let lineEnd = data[cursor..<limit].firstIndex(of: 0x0A) ?? limit
            let bytes = data[cursor..<lineEnd]
            cursor = lineEnd < limit ? lineEnd + 1 : limit
            guard let line = String(bytes: bytes, encoding: .ascii)?
                .trimmingCharacters(in: .whitespacesAndNewlines) else {
                throw TrailError.pcd("header is not ASCII text")
            }
            if line.isEmpty || line.hasPrefix("#") { continue }
            let parts = line.split(whereSeparator: { $0.isWhitespace }).map(String.init)
            let key = parts[0].uppercased()
            values[key] = Array(parts.dropFirst())
            if key == "DATA" {
                format = parts.count > 1 ? parts[1].lowercased() : ""
                dataStart = cursor - data.startIndex
                break
            }
        }
        guard let dataStart else { throw TrailError.pcd("no DATA line in the header") }
        guard let names = values["FIELDS"], !names.isEmpty else { throw TrailError.pcd("no FIELDS line") }
        let sizes = (values["SIZE"] ?? []).compactMap(Int.init)
        let types = values["TYPE"] ?? []
        let counts = values["COUNT"].map { $0.compactMap(Int.init) } ?? Array(repeating: 1, count: names.count)
        guard sizes.count == names.count, types.count == names.count, counts.count == names.count else {
            throw TrailError.pcd("FIELDS, SIZE, TYPE and COUNT have different lengths")
        }
        var fields: [Field] = []
        var offset = 0
        var column = 0
        for i in names.indices {
            guard sizes[i] > 0, counts[i] > 0, let type = types[i].uppercased().first else {
                throw TrailError.pcd("field \(names[i]) has a bad SIZE, TYPE or COUNT")
            }
            fields.append(Field(name: names[i].lowercased(), size: sizes[i], type: type, count: counts[i],
                                offset: offset, column: column))
            // Checked: a garbage SIZE or COUNT must throw, not trap.
            let (bytes, bytesOverflow) = sizes[i].multipliedReportingOverflow(by: counts[i])
            let (nextOffset, offsetOverflow) = offset.addingReportingOverflow(bytes)
            let (nextColumn, columnOverflow) = column.addingReportingOverflow(counts[i])
            guard !bytesOverflow, !offsetOverflow, !columnOverflow else {
                throw TrailError.pcd("SIZE × COUNT overflows at field \(names[i])")
            }
            offset = nextOffset
            column = nextColumn
        }
        var points = values["POINTS"]?.first.flatMap(Int.init)
        if points == nil, let width = values["WIDTH"]?.first.flatMap(Int.init) {
            let height = values["HEIGHT"]?.first.flatMap(Int.init) ?? 1
            let (product, overflow) = width.multipliedReportingOverflow(by: height)
            guard !overflow else { throw TrailError.pcd("WIDTH × HEIGHT overflows") }
            points = product
        }
        guard let points else {
            throw TrailError.pcd("no POINTS or WIDTH line")
        }
        guard points >= 0, points <= maxDeclared else {
            throw TrailError.pcd("POINTS \(points) is out of range")
        }
        return Header(fields: fields, points: points, format: format, dataStart: dataStart, stride: offset)
    }

    private static func decodeBinary(_ data: Data, header: Header, x: Field, y: Field, z: Field, keep: Int) throws -> TrailMapCloud {
        let (need, overflow) = header.points.multipliedReportingOverflow(by: header.stride)
        guard !overflow else { throw TrailError.pcd("POINTS × point size overflows") }
        let have = data.count - header.dataStart
        guard have >= need else {
            throw TrailError.pcd("binary data is \(have) bytes; \(header.points) points × \(header.stride) bytes needs \(need)")
        }
        var points: [Point3D] = []
        points.reserveCapacity(header.points / keep + 1)
        var dropped = 0
        data.withUnsafeBytes { (raw: UnsafeRawBufferPointer) in
            let base = header.dataStart
            func read(_ field: Field, _ row: Int) -> Float {
                let at = base + row * header.stride + field.offset
                if field.size == 8 {
                    return Float(Double(bitPattern: UInt64(littleEndian: raw.loadUnaligned(fromByteOffset: at, as: UInt64.self))))
                }
                return Float(bitPattern: UInt32(littleEndian: raw.loadUnaligned(fromByteOffset: at, as: UInt32.self)))
            }
            var row = 0
            while row < header.points {
                let p = Point3D(x: read(x, row), y: read(y, row), z: read(z, row))
                if TrailBounds.contains(p) {
                    points.append(p)
                } else {
                    dropped += 1
                }
                row += keep
            }
        }
        return TrailMapCloud(points: points, format: .binary, declared: header.points, dropped: dropped, stride: keep)
    }

    private static func decodeASCII(_ data: Data, header: Header, x: Field, y: Field, z: Field, keep: Int) throws -> TrailMapCloud {
        guard let text = String(data: data[(data.startIndex + header.dataStart)...], encoding: .utf8) else {
            throw TrailError.pcd("ascii data is not UTF-8 text")
        }
        let need = max(x.column, y.column, z.column) + 1
        var points: [Point3D] = []
        points.reserveCapacity(header.points / keep + 1)
        var dropped = 0
        var row = 0
        // `isNewline`, not "\n": Swift reads "\r\n" as one Character.
        for line in text.split(whereSeparator: \.isNewline) {
            let values = line.split(whereSeparator: \.isWhitespace)
            if values.isEmpty { continue }
            defer { row += 1 }
            guard row < header.points else {
                throw TrailError.pcd("more data lines than POINTS \(header.points)")
            }
            guard values.count >= need else {
                throw TrailError.pcd("data line \(row + 1) has \(values.count) values, needs \(need)")
            }
            guard row % keep == 0 else { continue }
            guard let px = Float(values[x.column]), let py = Float(values[y.column]), let pz = Float(values[z.column]) else {
                throw TrailError.pcd("data line \(row + 1): x/y/z is not a number")
            }
            let p = Point3D(x: px, y: py, z: pz)
            if TrailBounds.contains(p) {
                points.append(p)
            } else {
                dropped += 1
            }
        }
        guard row == header.points else {
            throw TrailError.pcd("\(row) data lines; POINTS says \(header.points)")
        }
        return TrailMapCloud(points: points, format: .ascii, declared: header.points, dropped: dropped, stride: keep)
    }
}
