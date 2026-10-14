import Foundation
import LabCommandCore

// MARK: - CDR Encoder Helpers

private let cdrFloat32: UInt8 = 7
private let cdrFloat64: UInt8 = 8

private struct CDRTestField {
    var name: String
    var offset: UInt32
    var datatype: UInt8
    var count: UInt32 = 1
}

/// Minimal CDR writer: 4-byte encapsulation header, then values aligned relative to byte 4.
private struct CDRTestWriter {
    private(set) var bytes: [UInt8]
    let bigEndian: Bool

    init(bigEndian: Bool) {
        self.bigEndian = bigEndian
        bytes = [0x00, bigEndian ? 0x00 : 0x01, 0x00, 0x00]
    }

    mutating func u8(_ value: UInt8) {
        bytes.append(value)
    }

    mutating func u32(_ value: UInt32) {
        while (bytes.count - 4) % 4 != 0 { bytes.append(0) }
        withUnsafeBytes(of: bigEndian ? value.bigEndian : value.littleEndian) { bytes.append(contentsOf: $0) }
    }

    mutating func string(_ value: String) {
        let utf8 = Array(value.utf8)
        u32(UInt32(utf8.count + 1))
        bytes.append(contentsOf: utf8)
        bytes.append(0)
    }

    mutating func raw(_ value: [UInt8]) {
        bytes.append(contentsOf: value)
    }
}

/// Builds the `data` blob of a PointCloud2 message.
private struct CDRTestPoints {
    private(set) var bytes: [UInt8] = []
    var bigEndian = false

    mutating func f32(_ value: Float) { append(value.bitPattern) }
    mutating func f64(_ value: Double) { append(value.bitPattern) }
    mutating func u8(_ value: UInt8) { bytes.append(value) }
    mutating func u16(_ value: UInt16) { append(value) }
    mutating func pad(_ count: Int) { bytes.append(contentsOf: [UInt8](repeating: 0xEE, count: count)) }

    private mutating func append<T: FixedWidthInteger>(_ value: T) {
        withUnsafeBytes(of: bigEndian ? value.bigEndian : value.littleEndian) { bytes.append(contentsOf: $0) }
    }
}

private struct CDRTestCloud {
    var sec: Int32 = 0
    var nanosec: UInt32 = 0
    var frameId = "lidar"
    var height: UInt32 = 1
    var width: UInt32
    var fields: [CDRTestField]
    var dataIsBigEndian = false
    var pointStep: UInt32
    var rowStep: UInt32?
    var data: [UInt8]

    /// Encodes with no trailing padding: the message ends at the is_dense byte.
    func encoded(cdrBigEndian: Bool = false) -> Data {
        var writer = CDRTestWriter(bigEndian: cdrBigEndian)
        writer.u32(UInt32(bitPattern: sec))
        writer.u32(nanosec)
        writer.string(frameId)
        writer.u32(height)
        writer.u32(width)
        writer.u32(UInt32(fields.count))
        for field in fields {
            writer.string(field.name)
            writer.u32(field.offset)
            writer.u8(field.datatype)
            writer.u32(field.count)
        }
        writer.u8(dataIsBigEndian ? 1 : 0)
        writer.u32(pointStep)
        writer.u32(rowStep ?? pointStep * width)
        writer.u32(UInt32(data.count))
        writer.raw(data)
        writer.u8(1)
        return Data(writer.bytes)
    }
}

private let cdrXYZIFields = [
    CDRTestField(name: "x", offset: 0, datatype: cdrFloat32),
    CDRTestField(name: "y", offset: 4, datatype: cdrFloat32),
    CDRTestField(name: "z", offset: 8, datatype: cdrFloat32),
    CDRTestField(name: "intensity", offset: 12, datatype: cdrFloat32),
]

/// Unorganized float32 x/y/z/intensity cloud, 16 bytes per point.
private func cdrXYZICloud(
    _ points: [(Float, Float, Float, Float)],
    dataIsBigEndian: Bool = false
) -> CDRTestCloud {
    var data = CDRTestPoints(bigEndian: dataIsBigEndian)
    for (x, y, z, intensity) in points {
        data.f32(x); data.f32(y); data.f32(z); data.f32(intensity)
    }
    return CDRTestCloud(
        width: UInt32(points.count),
        fields: cdrXYZIFields,
        dataIsBigEndian: dataIsBigEndian,
        pointStep: 16,
        data: data.bytes
    )
}

/// Points whose x is the source index, so decimation picks are easy to read back.
private func cdrIndexedCloud(count: Int) -> CDRTestCloud {
    var points: [(Float, Float, Float, Float)] = []
    for index in 0..<count {
        let value = Float(index)
        points.append((value, value * 2, value * 3, value / 10))
    }
    return cdrXYZICloud(points)
}

private func cdrDecode(_ data: Data, maxPoints: Int? = nil, _ label: String) -> PointCloudFrame {
    do {
        return try PointCloud2CDR.decode(data, maxPoints: maxPoints)
    } catch {
        expect(false, "\(label): decode failed: \(error)")
        return PointCloudFrame()
    }
}

/// Asserts decoding throws a PointCloudError with a message; returns that message.
@discardableResult
private func cdrExpectThrows(_ data: Data, _ label: String) -> String {
    do {
        let frame = try PointCloud2CDR.decode(data)
        expect(false, "\(label): should have thrown, decoded \(frame.count) points")
        return ""
    } catch let error as PointCloudError {
        let message: String
        switch error {
        case .invalidHeader(let text), .invalidData(let text), .unsupportedFormat(let text), .fileNotFound(let text):
            message = text
        }
        expect(!message.isEmpty, "\(label): error should carry a description")
        return message
    } catch {
        expect(false, "\(label): unexpected error type \(error)")
        return ""
    }
}

// MARK: - Decoding Tests

func testCDRFloat32XYZIntensity() {
    let cloud = cdrXYZICloud([
        (1.0, 2.0, 3.0, 0.5),
        (-4.5, 5.25, 6.125, 0.75),
        (7.0, -8.0, 9.0, 255.0),
    ])
    let frame = cdrDecode(cloud.encoded(), "CDR float32")

    expect(frame.count == 3, "CDR float32: expected 3 points, got \(frame.count)")
    expect(frame.positions == [
        Point3D(x: 1.0, y: 2.0, z: 3.0),
        Point3D(x: -4.5, y: 5.25, z: 6.125),
        Point3D(x: 7.0, y: -8.0, z: 9.0),
    ], "CDR float32: positions mismatch: \(frame.positions)")
    expect(frame.intensities == [0.5, 0.75, 255.0], "CDR float32: intensities mismatch: \(frame.intensities)")
    expect(frame.sensor == nil, "CDR float32: decoder should not guess a sensor")
}

func testCDRFloat64() {
    var data = CDRTestPoints()
    data.f64(1.5); data.f64(-2.5); data.f64(3.5); data.f64(0.25)
    data.f64(100.125); data.f64(200.25); data.f64(-300.5); data.f64(42.0)
    let cloud = CDRTestCloud(
        width: 2,
        fields: [
            CDRTestField(name: "x", offset: 0, datatype: cdrFloat64),
            CDRTestField(name: "y", offset: 8, datatype: cdrFloat64),
            CDRTestField(name: "z", offset: 16, datatype: cdrFloat64),
            CDRTestField(name: "intensity", offset: 24, datatype: cdrFloat64),
        ],
        pointStep: 32,
        data: data.bytes
    )
    let frame = cdrDecode(cloud.encoded(), "CDR float64")

    expect(frame.positions == [
        Point3D(x: 1.5, y: -2.5, z: 3.5),
        Point3D(x: 100.125, y: 200.25, z: -300.5),
    ], "CDR float64: positions mismatch: \(frame.positions)")
    expect(frame.intensities == [0.25, 42.0], "CDR float64: intensities mismatch: \(frame.intensities)")
}

func testCDRIntensityIntegerTypesAndExtraFields() {
    // Livox-style record: xyz, uint8 intensity, then fields the decoder must ignore —
    // including one whose datatype it has never heard of.
    var data = CDRTestPoints()
    for (index, intensity) in [UInt8(0), 128, 255].enumerated() {
        data.f32(Float(index)); data.f32(1); data.f32(2)
        data.u8(intensity); data.u8(7); data.pad(2); data.f64(1234.5)
    }
    let uint8Cloud = CDRTestCloud(
        width: 3,
        fields: [
            CDRTestField(name: "x", offset: 0, datatype: cdrFloat32),
            CDRTestField(name: "y", offset: 4, datatype: cdrFloat32),
            CDRTestField(name: "z", offset: 8, datatype: cdrFloat32),
            CDRTestField(name: "intensity", offset: 12, datatype: 2),
            CDRTestField(name: "tag", offset: 13, datatype: 2),
            CDRTestField(name: "vendor_blob", offset: 14, datatype: 99, count: 2),
            CDRTestField(name: "timestamp", offset: 16, datatype: cdrFloat64),
        ],
        pointStep: 24,
        data: data.bytes
    )
    let uint8Frame = cdrDecode(uint8Cloud.encoded(), "CDR uint8 intensity")
    expect(uint8Frame.positions.map(\.x) == [0, 1, 2], "CDR uint8 intensity: positions mismatch")
    expect(uint8Frame.intensities == [0, 128, 255], "CDR uint8 intensity: got \(uint8Frame.intensities)")

    // uint16 intensity ahead of xyz, to prove offsets come from the fields, not field order.
    var data16 = CDRTestPoints()
    data16.u16(40_000); data16.pad(2); data16.f32(1); data16.f32(2); data16.f32(3)
    data16.u16(513); data16.pad(2); data16.f32(4); data16.f32(5); data16.f32(6)
    var uint16Cloud = CDRTestCloud(
        width: 2,
        fields: [
            CDRTestField(name: "intensity", offset: 0, datatype: 4),
            CDRTestField(name: "x", offset: 4, datatype: cdrFloat32),
            CDRTestField(name: "y", offset: 8, datatype: cdrFloat32),
            CDRTestField(name: "z", offset: 12, datatype: cdrFloat32),
        ],
        pointStep: 16,
        data: data16.bytes
    )
    let uint16Frame = cdrDecode(uint16Cloud.encoded(), "CDR uint16 intensity")
    expect(uint16Frame.positions == [Point3D(x: 1, y: 2, z: 3), Point3D(x: 4, y: 5, z: 6)],
           "CDR uint16 intensity: positions mismatch")
    expect(uint16Frame.intensities == [40_000, 513], "CDR uint16 intensity: got \(uint16Frame.intensities)")

    // An intensity datatype we can't read is "no intensity", not a dropped frame.
    uint16Cloud.fields[0].datatype = 5
    let unreadable = cdrDecode(uint16Cloud.encoded(), "CDR int32 intensity")
    expect(unreadable.count == 2 && unreadable.intensities == [1.0, 1.0],
           "CDR int32 intensity: should default to 1.0, got \(unreadable.intensities)")

    // No intensity field at all.
    uint16Cloud.fields.removeFirst()
    let absent = cdrDecode(uint16Cloud.encoded(), "CDR no intensity")
    expect(absent.count == 2 && absent.intensities == [1.0, 1.0],
           "CDR no intensity: should default to 1.0, got \(absent.intensities)")
}

func testCDRNaNAndInfSkipped() {
    let cloud = cdrXYZICloud([
        (1, 1, 1, 0.1),
        (.nan, 2, 2, 0.2),
        (3, .infinity, 3, 0.3),
        (4, 4, -.infinity, 0.4),
        (5, 5, 5, .nan),
        (6, 6, 6, 0.6),
    ])
    let frame = cdrDecode(cloud.encoded(), "CDR NaN")

    expect(frame.positions.map(\.x) == [1, 5, 6], "CDR NaN: expected points 1/5/6, got \(frame.positions)")
    // Intensities stay paired with the surviving points; a NaN intensity keeps its point.
    expect(frame.intensities == [0.1, 1.0, 0.6], "CDR NaN: intensities mismatch: \(frame.intensities)")

    // A float64 too large for Float becomes inf on narrowing and is dropped too.
    var data = CDRTestPoints()
    data.f64(1e300); data.f64(0); data.f64(0)
    data.f64(2); data.f64(0); data.f64(0)
    let wide = CDRTestCloud(
        width: 2,
        fields: [
            CDRTestField(name: "x", offset: 0, datatype: cdrFloat64),
            CDRTestField(name: "y", offset: 8, datatype: cdrFloat64),
            CDRTestField(name: "z", offset: 16, datatype: cdrFloat64),
        ],
        pointStep: 24,
        data: data.bytes
    )
    let wideFrame = cdrDecode(wide.encoded(), "CDR float64 overflow")
    expect(wideFrame.positions == [Point3D(x: 2, y: 0, z: 0)],
           "CDR float64 overflow: expected only the finite point, got \(wideFrame.positions)")
}

func testCDRDecimation() {
    let message = cdrIndexedCloud(count: 10).encoded()

    // ceil(10 / 3) = 4 → source points 0, 4, 8.
    let three = cdrDecode(message, maxPoints: 3, "CDR decimation 3")
    expect(three.positions.map(\.x) == [0, 4, 8], "CDR decimation: maxPoints 3 picked \(three.positions.map(\.x))")
    expect(three.positions[1] == Point3D(x: 4, y: 8, z: 12), "CDR decimation: picked point is intact")
    expect(three.intensities == [0, 0.4, 0.8], "CDR decimation: intensities follow their points")

    let five = cdrDecode(message, maxPoints: 5, "CDR decimation 5")
    expect(five.positions.map(\.x) == [0, 2, 4, 6, 8], "CDR decimation: maxPoints 5 picked \(five.positions.map(\.x))")

    let one = cdrDecode(message, maxPoints: 1, "CDR decimation 1")
    expect(one.positions.map(\.x) == [0], "CDR decimation: maxPoints 1 picked \(one.positions.map(\.x))")

    // At or above the cloud size, and nil / non-positive, mean "everything".
    for limit in [10, 11, 1_000_000, 0, -5] {
        let all = cdrDecode(message, maxPoints: limit, "CDR decimation \(limit)")
        expect(all.count == 10, "CDR decimation: maxPoints \(limit) should keep all 10, got \(all.count)")
    }
    expect(cdrDecode(message, "CDR decimation nil").count == 10, "CDR decimation: nil keeps all points")

    // Never exceeds the cap, whatever the ratio.
    let hundred = cdrIndexedCloud(count: 100).encoded()
    for limit in 1...100 {
        let frame = cdrDecode(hundred, maxPoints: limit, "CDR decimation 100/\(limit)")
        expect(frame.count <= limit && frame.count > limit / 2,
               "CDR decimation: 100 points capped at \(limit) gave \(frame.count)")
    }

    // Decimation selects first, then filters: a NaN at a picked index is simply lost.
    var points: [(Float, Float, Float, Float)] = []
    for index in 0..<10 {
        points.append((index == 4 ? .nan : Float(index), 0, 0, 0))
    }
    let withNaN = cdrDecode(cdrXYZICloud(points).encoded(), maxPoints: 3, "CDR decimation NaN")
    expect(withNaN.positions.map(\.x) == [0, 8], "CDR decimation: NaN at a picked index, got \(withNaN.positions.map(\.x))")
}

func testCDRFrameIdAndTimestamp() {
    // Lengths chosen so the padding after frame_id takes every value 0–3.
    for frameId in ["", "a", "ab", "abc", "abcd", "livox_frame", "hesai/jt128_lidar"] {
        var cloud = cdrXYZICloud([(1, 2, 3, 4)])
        cloud.frameId = frameId
        cloud.sec = 1_700_000_000
        cloud.nanosec = 250_000_000
        let frame = cdrDecode(cloud.encoded(), "CDR frame_id '\(frameId)'")

        expect(frame.frameId == frameId, "CDR frame_id: expected '\(frameId)', got '\(frame.frameId ?? "nil")'")
        expect(frame.positions == [Point3D(x: 1, y: 2, z: 3)],
               "CDR frame_id '\(frameId)': body misaligned after the string, got \(frame.positions)")
        guard let timestamp = frame.timestamp else {
            expect(false, "CDR timestamp: missing for frame_id '\(frameId)'")
            return
        }
        expect(abs(timestamp.timeIntervalSince1970 - 1_700_000_000.25) < 1e-6,
               "CDR timestamp: expected 1700000000.25, got \(timestamp.timeIntervalSince1970)")
    }

    // sec is int32: a pre-epoch stamp must not wrap to 2106.
    var early = cdrXYZICloud([(1, 2, 3, 4)])
    early.sec = -5
    early.nanosec = 500_000_000
    let earlyFrame = cdrDecode(early.encoded(), "CDR negative stamp")
    expect(abs((earlyFrame.timestamp?.timeIntervalSince1970 ?? 0) - (-4.5)) < 1e-6,
           "CDR timestamp: expected -4.5, got \(String(describing: earlyFrame.timestamp?.timeIntervalSince1970))")
}

func testCDRByteOrders() {
    let points: [(Float, Float, Float, Float)] = [(1.5, -2.5, 3.25, 0.5), (10, 20, 30, 0.25)]
    let expected = [Point3D(x: 1.5, y: -2.5, z: 3.25), Point3D(x: 10, y: 20, z: 30)]

    // The encapsulation byte order governs the envelope; is_bigendian governs the points.
    for cdrBigEndian in [false, true] {
        for dataBigEndian in [false, true] {
            var cloud = cdrXYZICloud(points, dataIsBigEndian: dataBigEndian)
            cloud.frameId = "be_frame"
            cloud.sec = 1_600_000_000
            let label = "CDR byte order (cdr BE=\(cdrBigEndian), data BE=\(dataBigEndian))"
            let frame = cdrDecode(cloud.encoded(cdrBigEndian: cdrBigEndian), label)

            expect(frame.positions == expected, "\(label): positions mismatch: \(frame.positions)")
            expect(frame.intensities == [0.5, 0.25], "\(label): intensities mismatch: \(frame.intensities)")
            expect(frame.frameId == "be_frame", "\(label): frame_id mismatch")
            expect(frame.timestamp == Date(timeIntervalSince1970: 1_600_000_000), "\(label): timestamp mismatch")
        }
    }

    // Big-endian float64 and uint16 point data.
    var data = CDRTestPoints(bigEndian: true)
    data.f64(1.25); data.f64(2.5); data.f64(-3.75); data.u16(4660); data.pad(6)
    let cloud = CDRTestCloud(
        width: 1,
        fields: [
            CDRTestField(name: "x", offset: 0, datatype: cdrFloat64),
            CDRTestField(name: "y", offset: 8, datatype: cdrFloat64),
            CDRTestField(name: "z", offset: 16, datatype: cdrFloat64),
            CDRTestField(name: "intensity", offset: 24, datatype: 4),
        ],
        dataIsBigEndian: true,
        pointStep: 32,
        data: data.bytes
    )
    let frame = cdrDecode(cloud.encoded(), "CDR big-endian float64")
    expect(frame.positions == [Point3D(x: 1.25, y: 2.5, z: -3.75)], "CDR big-endian float64: got \(frame.positions)")
    expect(frame.intensities == [4660], "CDR big-endian uint16: got \(frame.intensities)")
}

func testCDRRowPaddingAndTrailingBytes() {
    // Organized 2×2 cloud with 8 bytes of padding at the end of each row.
    var data = CDRTestPoints()
    for row in 0..<2 {
        for col in 0..<2 {
            data.f32(Float(row * 10 + col)); data.f32(Float(row)); data.f32(Float(col)); data.f32(0.5)
        }
        data.pad(8)
    }
    let cloud = CDRTestCloud(height: 2, width: 2, fields: cdrXYZIFields, pointStep: 16, rowStep: 40, data: data.bytes)
    let message = cloud.encoded()

    let frame = cdrDecode(message, "CDR row padding")
    expect(frame.positions.map(\.x) == [0, 1, 10, 11], "CDR row padding: got \(frame.positions.map(\.x))")
    expect(frame.positions[3] == Point3D(x: 11, y: 1, z: 1), "CDR row padding: last point mismatch")

    let decimated = cdrDecode(message, maxPoints: 2, "CDR row padding decimated")
    expect(decimated.positions.map(\.x) == [0, 10], "CDR row padding: decimated got \(decimated.positions.map(\.x))")

    // rmw pads the serialized message out to a 4-byte boundary after is_dense.
    let padded = message + Data([0, 0, 0])
    expect(cdrDecode(padded, "CDR trailing padding") == frame, "CDR trailing padding: should decode identically")

    // row_step 0 is read as tightly packed rows.
    var packed = cdrIndexedCloud(count: 4)
    packed.rowStep = 0
    expect(cdrDecode(packed.encoded(), "CDR row_step 0").count == 4, "CDR row_step 0: expected 4 points")
}

func testCDREmptyCloud() {
    var cloud = cdrXYZICloud([])
    cloud.frameId = "livox_frame"
    cloud.sec = 12
    let frame = cdrDecode(cloud.encoded(), "CDR empty")
    expect(frame.isEmpty && frame.intensities.isEmpty, "CDR empty: expected no points")
    expect(frame.frameId == "livox_frame", "CDR empty: frame_id still reported")
    expect(frame.timestamp == Date(timeIntervalSince1970: 12), "CDR empty: timestamp still reported")
}

func testCDRDataSlice() {
    // foxglove_bridge binary frames carry opcode + subscription id + timestamp before
    // the CDR payload, so the decoder gets a slice whose startIndex isn't 0.
    let message = cdrIndexedCloud(count: 5).encoded()
    let prefix = Data([0x01, 0xAA, 0xBB, 0xCC, 0xDD, 1, 2, 3, 4, 5, 6, 7, 8])
    let framed = prefix + message + Data([0xFF, 0xFF])
    let slice = framed.dropFirst(prefix.count)
    expect(slice.startIndex == prefix.count, "CDR slice: test needs a non-zero startIndex")

    let frame = cdrDecode(slice, "CDR slice")
    expect(frame == cdrDecode(message, "CDR slice baseline"), "CDR slice: should match the unsliced decode")
    expect(frame.positions.map(\.x) == [0, 1, 2, 3, 4], "CDR slice: got \(frame.positions.map(\.x))")
}

// MARK: - Malformed Input Tests

func testCDRTruncatedInput() {
    var cloud = cdrXYZICloud([(1, 2, 3, 0.5), (4, 5, 6, 0.75)])
    cloud.frameId = "livox_frame"
    let message = cloud.encoded()
    expect((try? PointCloud2CDR.decode(message))?.count == 2, "CDR truncated: baseline message must decode")

    // Cut anywhere — header, strings, field table, point data, the final is_dense byte —
    // and the decoder reports truncation instead of reading past the end.
    for length in 0..<message.count {
        let text = cdrExpectThrows(message.prefix(length), "CDR truncated at \(length)/\(message.count)")
        expect(text.contains("truncated"), "CDR truncated at \(length): message should say truncated, got '\(text)'")
    }

    let headerOnly = cdrExpectThrows(Data([0x00, 0x01, 0x00, 0x00]), "CDR truncated: header only")
    expect(headerOnly.contains("header.stamp.sec"), "CDR truncated: should name the missing value, got '\(headerOnly)'")
}

func testCDRMalformedInput() {
    let base = cdrIndexedCloud(count: 4)

    // Encapsulation other than plain CDR LE/BE.
    for kind in [[UInt8](arrayLiteral: 0x00, 0x02), [0x00, 0x03], [0x00, 0x07], [0x01, 0x01], [0xFF, 0xFF]] {
        var bytes = [UInt8](base.encoded())
        bytes[0] = kind[0]
        bytes[1] = kind[1]
        let text = cdrExpectThrows(Data(bytes), "CDR malformed: encapsulation \(kind)")
        expect(text.contains("encapsulation"), "CDR malformed: should mention encapsulation, got '\(text)'")
    }

    var missingZ = base
    missingZ.fields.remove(at: 2)
    expect(cdrExpectThrows(missingZ.encoded(), "CDR malformed: missing z").contains("'z'"),
           "CDR malformed: should name the missing field")

    var intX = base
    intX.fields[0].datatype = 5
    expect(cdrExpectThrows(intX.encoded(), "CDR malformed: int32 x").contains("'x'"),
           "CDR malformed: should name the unsupported field")

    var fieldPastStep = base
    fieldPastStep.fields[2].offset = 14
    cdrExpectThrows(fieldPastStep.encoded(), "CDR malformed: z past point_step")

    var hugeOffset = base
    hugeOffset.fields[1].offset = .max
    cdrExpectThrows(hugeOffset.encoded(), "CDR malformed: y offset UInt32.max")

    var intensityPastStep = base
    intensityPastStep.fields[3].offset = 13
    cdrExpectThrows(intensityPastStep.encoded(), "CDR malformed: intensity past point_step")

    var zeroStep = base
    zeroStep.pointStep = 0
    cdrExpectThrows(zeroStep.encoded(), "CDR malformed: point_step 0")

    var tinyStep = base
    tinyStep.pointStep = 8
    tinyStep.rowStep = 64
    cdrExpectThrows(tinyStep.encoded(), "CDR malformed: point_step smaller than fields")

    // Header claims more points than the data holds.
    var tooWide = base
    tooWide.width = 5
    tooWide.rowStep = 80
    cdrExpectThrows(tooWide.encoded(), "CDR malformed: width beyond data")

    var tooTall = base
    tooTall.height = 3
    cdrExpectThrows(tooTall.encoded(), "CDR malformed: height beyond data")

    var shortRow = base
    shortRow.rowStep = 32
    cdrExpectThrows(shortRow.encoded(), "CDR malformed: row_step below point_step × width")

    // Dimensions that overflow, or that would allocate billions of points if trusted.
    var overflowing = base
    overflowing.width = .max
    overflowing.height = .max
    overflowing.rowStep = .max
    cdrExpectThrows(overflowing.encoded(), "CDR malformed: width × height overflow")

    var enormous = base
    enormous.width = 1 << 30
    enormous.height = 4
    enormous.pointStep = 16
    enormous.rowStep = 1 << 31
    cdrExpectThrows(enormous.encoded(), "CDR malformed: 4 billion points claimed")

    // Lengths that point far past the buffer: frame_id length, fields count, data length.
    let valid = [UInt8](base.encoded())
    for (offset, what) in [(12, "frame_id length"), (32, "fields count"), (136, "data length")] {
        var bytes = valid
        bytes.replaceSubrange(offset..<offset + 4, with: [0xFF, 0xFF, 0xFF, 0xFF])
        let text = cdrExpectThrows(Data(bytes), "CDR malformed: \(what) = UInt32.max")
        expect(text.contains("truncated"), "CDR malformed: \(what) should read as truncated, got '\(text)'")
    }

    // Garbage that happens to start with a valid encapsulation header.
    cdrExpectThrows(Data([0x00, 0x01, 0x00, 0x00] + [UInt8](repeating: 0xFF, count: 64)), "CDR malformed: 0xFF fill")
    cdrExpectThrows(Data(), "CDR malformed: empty")
}

func testCDRMutatedInputNeverCrashes() {
    // Deterministic fuzz: flip bytes and cut tails of a valid message. Any outcome but a
    // crash is fine; whatever decodes must keep positions and intensities paired.
    var cloud = cdrIndexedCloud(count: 8)
    cloud.frameId = "livox_frame"
    let valid = [UInt8](cloud.encoded())

    var state: UInt64 = 0x9E37_79B9_7F4A_7C15
    func next() -> Int {
        state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
        return Int(state >> 33)
    }

    var decoded = 0
    var rejected = 0
    for round in 0..<4000 {
        var bytes = valid
        for _ in 0...(next() % 3) {
            // Bias toward the envelope, where lengths and offsets live.
            let index = round % 2 == 0 ? next() % 140 : next() % bytes.count
            bytes[index] = round % 3 == 0 ? UInt8(truncatingIfNeeded: next()) : bytes[index] ^ (1 << UInt8(next() % 8))
        }
        if round % 5 == 0 {
            bytes.removeLast(next() % bytes.count)
        }
        do {
            let frame = try PointCloud2CDR.decode(Data(bytes), maxPoints: round % 4 == 0 ? 3 : nil)
            expect(frame.positions.count == frame.intensities.count, "CDR fuzz: positions/intensities unpaired")
            expect(frame.positions.allSatisfy { $0.x.isFinite && $0.y.isFinite && $0.z.isFinite },
                   "CDR fuzz: non-finite point escaped")
            decoded += 1
        } catch is PointCloudError {
            rejected += 1
        } catch {
            expect(false, "CDR fuzz: unexpected error type \(error)")
        }
    }
    expect(decoded > 0 && rejected > 0, "CDR fuzz: expected both outcomes, got \(decoded) decoded / \(rejected) rejected")
}

// MARK: - Real Fixture Cross-Check

func testCDRRealFixture() {
    // Message 0 of room-scan.mcap: bytes this file's encoder had no hand in.
    let path = "fixtures/pointclouds/ros2-pointcloud2.bin"
    guard let data = try? Data(contentsOf: URL(fileURLWithPath: path)) else {
        expect(false, "CDR real fixture: could not read \(path)")
        return
    }

    let frame = cdrDecode(data, "CDR real fixture")
    expect(frame.count == 100, "CDR real fixture: expected 100 points, got \(frame.count)")
    expect(frame.frameId == "livox_frame", "CDR real fixture: frame_id '\(frame.frameId ?? "nil")'")
    expect(point3DEquals(frame.positions[0], Point3D(x: 0, y: 0, z: 0)), "CDR real fixture: first point")
    expect(point3DEquals(frame.positions[1], Point3D(x: 0.1, y: 0, z: 0)), "CDR real fixture: second point")
    expect(abs(frame.intensities[1] - 0.01) < 0.0001, "CDR real fixture: second intensity \(frame.intensities[1])")

    // Same answer as the offline decoder, field for field.
    do {
        let reference = try PointCloudDecoder.decodeROS2PointCloud2(from: data)
        expect(frame == reference, "CDR real fixture: differs from PointCloudDecoder.decodeROS2PointCloud2")
    } catch {
        expect(false, "CDR real fixture: reference decoder failed: \(error)")
    }

    let decimated = cdrDecode(data, maxPoints: 25, "CDR real fixture decimated")
    expect(decimated.count == 25, "CDR real fixture: maxPoints 25 gave \(decimated.count)")
    expect(decimated.positions[1] == frame.positions[4], "CDR real fixture: stride 4 should pick source point 4")
}

// MARK: - Test Runner Entry Point

func runPointCloud2CDRTests() {
    testCDRFloat32XYZIntensity()
    print("  PointCloud2 CDR: little-endian float32 x/y/z/intensity"); passed()
    testCDRFloat64()
    print("  PointCloud2 CDR: float64 x/y/z/intensity narrowed to Float"); passed()
    testCDRIntensityIntegerTypesAndExtraFields()
    print("  PointCloud2 CDR: uint8/uint16 intensity; unknown fields ignored; unreadable intensity → 1.0"); passed()
    testCDRNaNAndInfSkipped()
    print("  PointCloud2 CDR: NaN/Inf points skipped, intensities stay paired"); passed()
    testCDRDecimation()
    print("  PointCloud2 CDR: maxPoints strides the cloud and never exceeds the cap"); passed()
    testCDRFrameIdAndTimestamp()
    print("  PointCloud2 CDR: frame_id at every padding, int32 sec + nanosec timestamp"); passed()
    testCDRByteOrders()
    print("  PointCloud2 CDR: encapsulation and is_bigendian byte orders are independent"); passed()
    testCDRRowPaddingAndTrailingBytes()
    print("  PointCloud2 CDR: row_step padding honored, trailing bytes tolerated"); passed()
    testCDREmptyCloud()
    print("  PointCloud2 CDR: empty cloud keeps frame_id and timestamp"); passed()
    testCDRDataSlice()
    print("  PointCloud2 CDR: decodes a Data slice with non-zero startIndex"); passed()
    testCDRTruncatedInput()
    print("  PointCloud2 CDR safety: every truncation point throws a descriptive error"); passed()
    testCDRMalformedInput()
    print("  PointCloud2 CDR safety: bad encapsulation, layouts, and lengths rejected"); passed()
    testCDRMutatedInputNeverCrashes()
    print("  PointCloud2 CDR safety: 4000 mutated messages decode or throw, never crash"); passed()
    testCDRRealFixture()
    print("  PointCloud2 CDR: real livox fixture matches the offline ROS2 decoder"); passed()
}
