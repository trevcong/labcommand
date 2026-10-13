import Foundation
import LabCommandCore

// MARK: - PCD ASCII Tests

func testPCDBasicParsing() {
    let pcd = """
    # .PCD v0.7 - Point Cloud Data
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 3
    HEIGHT 1
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS 3
    DATA ascii
    1.0 2.0 3.0 0.5
    4.0 5.0 6.0 0.8
    7.0 8.0 9.0 1.0
    """

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 3, "PCD basic: expected 3 points, got \(frame.count)")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PCD basic: first point position mismatch")
        expect(frame.positions[1] == SIMD3<Float>(4.0, 5.0, 6.0),
               "PCD basic: second point position mismatch")
        expect(frame.positions[2] == SIMD3<Float>(7.0, 8.0, 9.0),
               "PCD basic: third point position mismatch")
        expect(abs(frame.intensities[0] - 0.5) < 0.001, "PCD basic: first intensity mismatch")
        expect(abs(frame.intensities[1] - 0.8) < 0.001, "PCD basic: second intensity mismatch")
        expect(abs(frame.intensities[2] - 1.0) < 0.001, "PCD basic: third intensity mismatch")
    } catch {
        expect(false, "PCD basic parsing failed: \(error)")
    }
}

func testPCDEmptyCloud() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 0
    HEIGHT 1
    POINTS 0
    DATA ascii
    """

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.isEmpty, "PCD empty: expected empty cloud")
        expect(frame.count == 0, "PCD empty: expected 0 points")
    } catch {
        expect(false, "PCD empty cloud parsing failed: \(error)")
    }
}

func testPCDWithoutIntensity() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH 2
    HEIGHT 1
    POINTS 2
    DATA ascii
    1.5 2.5 3.5
    4.5 5.5 6.5
    """

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 2, "PCD no intensity: expected 2 points, got \(frame.count)")
        expect(frame.positions[0] == SIMD3<Float>(1.5, 2.5, 3.5),
               "PCD no intensity: first point mismatch")
        expect(abs(frame.intensities[0] - 1.0) < 0.001,
               "PCD no intensity: default intensity should be 1.0")
    } catch {
        expect(false, "PCD without intensity failed: \(error)")
    }
}

func testPCDMissingVersion() {
    let pcd = """
    FIELDS x y z
    WIDTH 1
    HEIGHT 1
    DATA ascii
    1.0 2.0 3.0
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD missing version: should have thrown")
    } catch PointCloudError.invalidHeader {
        // Expected
    } catch {
        expect(false, "PCD missing version: unexpected error \(error)")
    }
}

func testPCDMissingFields() {
    let pcd = """
    VERSION 0.7
    WIDTH 1
    HEIGHT 1
    DATA ascii
    1.0 2.0 3.0
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD missing fields: should have thrown")
    } catch PointCloudError.invalidHeader {
        // Expected
    } catch {
        expect(false, "PCD missing fields: unexpected error \(error)")
    }
}

func testPCDMalformedPointSkipped() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 2
    HEIGHT 1
    POINTS 2
    DATA ascii
    1.0 2.0 3.0 0.5
    not valid data
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD truncated: should throw for fewer points than declared")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("Truncated"), "PCD truncated: error should mention truncation")
    } catch {
        expect(false, "PCD truncated: unexpected error \(error)")
    }
}

func testPCDWithComments() {
    let pcd = """
    # This is a comment
    VERSION 0.7
    # Another comment
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 2
    HEIGHT 1
    POINTS 2
    DATA ascii
    1.0 2.0 3.0 0.5
    4.0 5.0 6.0 0.8
    """

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 2, "PCD comments: expected 2 points")
    } catch {
        expect(false, "PCD with comments failed: \(error)")
    }
}

func testPCDNaNAndInfValues() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 4
    HEIGHT 1
    POINTS 4
    DATA ascii
    1.0 2.0 3.0 0.5
    nan nan nan 0.0
    inf -inf 1.0 0.5
    7.0 8.0 9.0 1.0
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD NaN/Inf: should throw for fewer valid points than declared")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("Truncated"), "PCD NaN/Inf: NaN/Inf lines should be skipped causing truncation")
    } catch {
        expect(false, "PCD NaN/Inf: unexpected error \(error)")
    }
}

func testPCDUnsupportedBinaryFormat() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH 1
    HEIGHT 1
    DATA binary
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD binary: should have thrown")
    } catch PointCloudError.unsupportedFormat(let msg) {
        expect(msg.contains("Binary PCD format is not yet implemented"),
               "PCD binary: error should document limitation")
    } catch {
        expect(false, "PCD binary: unexpected error \(error)")
    }
}

func testPCDFromData() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH 1
    HEIGHT 1
    POINTS 1
    DATA ascii
    1.0 2.0 3.0
    """

    guard let data = pcd.data(using: .utf8) else {
        expect(false, "PCD from data: failed to create data")
        return
    }

    do {
        let frame = try PointCloudDecoder.decodePCD(from: data)
        expect(frame.count == 1, "PCD from data: expected 1 point")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PCD from data: position mismatch")
    } catch {
        expect(false, "PCD from data failed: \(error)")
    }
}

func testPCDMultipleWhitespace() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 2
    HEIGHT 1
    POINTS 2
    DATA ascii
    1.0   2.0\t3.0    0.5
    4.0  5.0   6.0 0.8
    """

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 2, "PCD whitespace: expected 2 points, got \(frame.count)")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PCD whitespace: first point with mixed whitespace")
    } catch {
        expect(false, "PCD whitespace parsing failed: \(error)")
    }
}

func testPCDCountField() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z rgb
    SIZE 4 4 4 4
    TYPE F F F U
    COUNT 1 1 1 3
    WIDTH 1
    HEIGHT 1
    POINTS 1
    DATA ascii
    1.0 2.0 3.0 255 128 64
    """

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 1, "PCD count: expected 1 point")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PCD count: position with COUNT > 1 for rgb")
    } catch {
        expect(false, "PCD COUNT field failed: \(error)")
    }
}

func testPCDPointsWithoutWidth() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    HEIGHT 1
    POINTS 2
    DATA ascii
    1.0 2.0 3.0
    4.0 5.0 6.0
    """

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 2, "PCD no width: expected 2 points from POINTS header")
    } catch {
        expect(false, "PCD without WIDTH failed: \(error)")
    }
}

func testPCDHeaderMetadata() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 2
    HEIGHT 1
    POINTS 2
    DATA ascii
    1.0 2.0 3.0 0.5
    4.0 5.0 6.0 0.8
    """

    do {
        let result = try PointCloudDecoder.decodePCDWithHeader(from: pcd)
        expect(result.header.version == "0.7", "PCD header: version")
        expect(result.header.fields == ["x", "y", "z", "intensity"], "PCD header: fields")
        expect(result.header.sizes == [4, 4, 4, 4], "PCD header: sizes")
        expect(result.header.types == ["F", "F", "F", "F"], "PCD header: types")
        expect(result.header.counts == [1, 1, 1, 1], "PCD header: counts")
        expect(result.header.width == 2, "PCD header: width")
        expect(result.header.height == 1, "PCD header: height")
        expect(result.header.dataFormat == .ascii, "PCD header: format")
    } catch {
        expect(false, "PCD header metadata failed: \(error)")
    }
}

// MARK: - Safety Tests

func testPCDNegativeWidth() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    WIDTH -100
    HEIGHT 1
    DATA ascii
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD negative width: should have thrown")
    } catch PointCloudError.invalidHeader(let msg) {
        expect(msg.contains("negative"), "PCD negative: error mentions negative")
    } catch {
        expect(false, "PCD negative width: unexpected error \(error)")
    }
}

func testPCDNegativeHeight() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    WIDTH 100
    HEIGHT -5
    DATA ascii
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD negative height: should have thrown")
    } catch PointCloudError.invalidHeader(let msg) {
        expect(msg.contains("negative"), "PCD negative: error mentions negative")
    } catch {
        expect(false, "PCD negative height: unexpected error \(error)")
    }
}

func testPCDOverflowProtection() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    WIDTH 2147483647
    HEIGHT 2147483647
    DATA ascii
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD overflow: should have thrown for huge allocation")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("exceeds maximum"), "PCD overflow: rejects huge allocation")
    } catch PointCloudError.invalidHeader(let msg) {
        expect(msg.contains("overflow"), "PCD overflow: error mentions overflow")
    } catch {
        expect(false, "PCD overflow: unexpected error \(error)")
    }
}

func testPCDTruncatedFile() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 10
    HEIGHT 1
    POINTS 10
    DATA ascii
    1.0 2.0 3.0 0.5
    4.0 5.0 6.0 0.8
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD truncated: should throw when file has fewer points than declared")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("Truncated") || msg.contains("expected"), "PCD truncated: error mentions truncation")
    } catch {
        expect(false, "PCD truncated: unexpected error \(error)")
    }
}

// MARK: - Sensor ID from Filename Tests

func testPCDSensorFromFilename() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z
    WIDTH 1
    HEIGHT 1
    POINTS 1
    DATA ascii
    1.0 2.0 3.0
    """

    do {
        let mid = try PointCloudDecoder.decodePCD(from: pcd, filename: "mid360-frame.pcd")
        expect(mid.sensor == .mid360, "PCD sensor: mid360 from filename")

        let jt = try PointCloudDecoder.decodePCD(from: pcd, filename: "jt128-frame.pcd")
        expect(jt.sensor == .jt128, "PCD sensor: jt128 from filename")

        let unknown = try PointCloudDecoder.decodePCD(from: pcd, filename: "random.pcd")
        expect(unknown.sensor == nil, "PCD sensor: nil for unknown filename")
    } catch {
        expect(false, "PCD sensor from filename failed: \(error)")
    }
}

// MARK: - PointCloud2 Binary Tests

func testPointCloud2BasicParsing() {
    var data = Data()

    let points: [(Float, Float, Float, Float)] = [
        (1.0, 2.0, 3.0, 0.5),
        (4.0, 5.0, 6.0, 0.8),
        (7.0, 8.0, 9.0, 1.0),
    ]

    for (x, y, z, i) in points {
        data.append(contentsOf: withUnsafeBytes(of: x.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: y.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: z.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: i.bitPattern.littleEndian) { Array($0) })
    }

    let layout = PointCloud2Layout(
        pointStep: 16,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8,
        intensityOffset: 12,
        xType: .float32,
        yType: .float32,
        zType: .float32,
        intensityType: .float32
    )

    do {
        let frame = try PointCloudDecoder.decodePointCloud2(from: data, layout: layout)
        expect(frame.count == 3, "PC2 basic: expected 3 points, got \(frame.count)")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PC2 basic: first point mismatch")
        expect(abs(frame.intensities[2] - 1.0) < 0.001, "PC2 basic: third intensity mismatch")
    } catch {
        expect(false, "PC2 basic parsing failed: \(error)")
    }
}

func testPointCloud2EmptyData() {
    let data = Data()
    let layout = PointCloud2Layout(pointStep: 16, xOffset: 0, yOffset: 4, zOffset: 8)

    do {
        let frame = try PointCloudDecoder.decodePointCloud2(from: data, layout: layout)
        expect(frame.isEmpty, "PC2 empty: expected empty cloud")
    } catch {
        expect(false, "PC2 empty data failed: \(error)")
    }
}

func testPointCloud2BigEndian() {
    var data = Data()

    let x: Float = 1.0
    let y: Float = 2.0
    let z: Float = 3.0
    let i: Float = 0.5

    data.append(contentsOf: withUnsafeBytes(of: x.bitPattern.bigEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: y.bitPattern.bigEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: z.bitPattern.bigEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: i.bitPattern.bigEndian) { Array($0) })

    let layout = PointCloud2Layout(
        pointStep: 16,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8,
        intensityOffset: 12,
        isBigEndian: true,
        xType: .float32,
        yType: .float32,
        zType: .float32,
        intensityType: .float32
    )

    do {
        let frame = try PointCloudDecoder.decodePointCloud2(from: data, layout: layout)
        expect(frame.count == 1, "PC2 big endian: expected 1 point")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PC2 big endian: position mismatch")
    } catch {
        expect(false, "PC2 big endian failed: \(error)")
    }
}

func testPointCloud2LayoutFromFields() {
    let fields = [
        PointCloud2Layout.FieldDescriptor(name: "x", offset: 0, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "y", offset: 4, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "z", offset: 8, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "intensity", offset: 12, datatype: .float32),
    ]

    do {
        let layout = try PointCloud2Layout.from(fields: fields, pointStep: 16)
        expect(layout.xOffset == 0, "Layout from fields: xOffset")
        expect(layout.yOffset == 4, "Layout from fields: yOffset")
        expect(layout.zOffset == 8, "Layout from fields: zOffset")
        expect(layout.intensityOffset == 12, "Layout from fields: intensityOffset")
        expect(layout.xType == .float32, "Layout from fields: xType")
    } catch {
        expect(false, "Layout from fields failed: \(error)")
    }
}

func testPointCloud2LayoutMissingFields() {
    let fields = [
        PointCloud2Layout.FieldDescriptor(name: "x", offset: 0, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "y", offset: 4, datatype: .float32),
    ]

    do {
        _ = try PointCloud2Layout.from(fields: fields, pointStep: 12)
        expect(false, "Layout missing z: should have thrown")
    } catch PointCloudError.invalidHeader {
        // Expected
    } catch {
        expect(false, "Layout missing z: unexpected error \(error)")
    }
}

func testPointCloud2InvalidPointStep() {
    let data = Data([0, 1, 2, 3])
    let layout = PointCloud2Layout(pointStep: 0, xOffset: 0, yOffset: 4, zOffset: 8)

    do {
        _ = try PointCloudDecoder.decodePointCloud2(from: data, layout: layout)
        expect(false, "PC2 invalid step: should have thrown")
    } catch PointCloudError.invalidData {
        // Expected
    } catch {
        expect(false, "PC2 invalid step: unexpected error \(error)")
    }
}

// MARK: - ROS1 Header Tests

func testROS1PointCloud2Header() {
    var data = Data()

    func appendU32(_ value: UInt32) {
        data.append(contentsOf: withUnsafeBytes(of: value.littleEndian) { Array($0) })
    }
    func appendU8(_ value: UInt8) {
        data.append(value)
    }
    func appendString(_ str: String) {
        let bytes = Array(str.utf8)
        appendU32(UInt32(bytes.count))
        data.append(contentsOf: bytes)
    }

    appendU32(1)
    appendU32(2)

    appendU32(3)
    appendString("x")
    appendU32(0)
    appendU8(7)
    appendU32(1)

    appendString("y")
    appendU32(4)
    appendU8(7)
    appendU32(1)

    appendString("z")
    appendU32(8)
    appendU8(7)
    appendU32(1)

    appendU8(0)
    appendU32(12)
    appendU32(12)
    appendU32(24)

    let points: [(Float, Float, Float)] = [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]
    for (x, y, z) in points {
        data.append(contentsOf: withUnsafeBytes(of: x.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: y.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: z.bitPattern.littleEndian) { Array($0) })
    }

    do {
        let frame = try PointCloudDecoder.decodeROS1PointCloud2(from: data)
        expect(frame.count == 2, "ROS1 header: expected 2 points, got \(frame.count)")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "ROS1 header: first point mismatch")
        expect(frame.positions[1] == SIMD3<Float>(4.0, 5.0, 6.0),
               "ROS1 header: second point mismatch")
    } catch {
        expect(false, "ROS1 header parsing failed: \(error)")
    }
}

func testROS1UnknownDatatype() {
    var data = Data()

    func appendU32(_ value: UInt32) {
        data.append(contentsOf: withUnsafeBytes(of: value.littleEndian) { Array($0) })
    }
    func appendU8(_ value: UInt8) {
        data.append(value)
    }
    func appendString(_ str: String) {
        let bytes = Array(str.utf8)
        appendU32(UInt32(bytes.count))
        data.append(contentsOf: bytes)
    }

    appendU32(1)
    appendU32(1)

    appendU32(1)
    appendString("x")
    appendU32(0)
    appendU8(99)
    appendU32(1)

    do {
        _ = try PointCloudDecoder.decodeROS1PointCloud2(from: data)
        expect(false, "ROS1 unknown datatype: should have thrown")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("Unknown field datatype"), "ROS1 unknown datatype: error message")
    } catch {
        expect(false, "ROS1 unknown datatype: unexpected error \(error)")
    }
}

// MARK: - ROS2 Header Tests

func testROS2PointCloud2Header() {
    var inner = Data()

    func appendU32(_ value: UInt32) {
        inner.append(contentsOf: withUnsafeBytes(of: value.littleEndian) { Array($0) })
    }
    func appendU8(_ value: UInt8) {
        inner.append(value)
    }
    func appendString(_ str: String) {
        let bytes = Array(str.utf8)
        appendU32(UInt32(bytes.count))
        inner.append(contentsOf: bytes)
    }

    appendU32(1)
    appendU32(1)

    appendU32(3)
    appendString("x")
    appendU32(0)
    appendU8(7)
    appendU32(1)

    appendString("y")
    appendU32(4)
    appendU8(7)
    appendU32(1)

    appendString("z")
    appendU32(8)
    appendU8(7)
    appendU32(1)

    appendU8(0)
    appendU32(12)
    appendU32(12)
    appendU32(12)

    inner.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })
    inner.append(contentsOf: withUnsafeBytes(of: Float(2.0).bitPattern.littleEndian) { Array($0) })
    inner.append(contentsOf: withUnsafeBytes(of: Float(3.0).bitPattern.littleEndian) { Array($0) })

    var data = Data([0x00, 0x01, 0x00, 0x00])
    data.append(inner)

    do {
        let frame = try PointCloudDecoder.decodeROS2PointCloud2(from: data)
        expect(frame.count == 1, "ROS2 header: expected 1 point, got \(frame.count)")
        expect(frame.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "ROS2 header: point mismatch")
    } catch {
        expect(false, "ROS2 header parsing failed: \(error)")
    }
}

func testROS2InvalidEncapsulation() {
    let data = Data([0x01, 0x00, 0x00, 0x00])

    do {
        _ = try PointCloudDecoder.decodeROS2PointCloud2(from: data)
        expect(false, "ROS2 invalid encapsulation: should have thrown")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("encapsulation"), "ROS2 invalid encapsulation: error message")
    } catch {
        expect(false, "ROS2 invalid encapsulation: unexpected error \(error)")
    }
}

// MARK: - Fixture File Tests

func testLoadMid360Fixture() {
    let path = "fixtures/pointclouds/mid360-frame.pcd"
    guard FileManager.default.fileExists(atPath: path) else {
        print("  fixture: mid360-frame.pcd not found at \(path) — skipped")
        return
    }

    do {
        let frame = try PointCloudDecoder.loadPCD(from: path)
        expect(frame.count == 50000, "mid360 fixture: expected 50000 points, got \(frame.count)")
        expect(frame.sensor == .mid360, "mid360 fixture: sensor should be mid360")
        expect(!frame.positions.isEmpty, "mid360 fixture: has positions")
        expect(frame.positions.count == frame.intensities.count,
               "mid360 fixture: positions and intensities match")
    } catch {
        expect(false, "mid360 fixture loading failed: \(error)")
    }
}

func testLoadJT128Fixture() {
    let path = "fixtures/pointclouds/jt128-frame.pcd"
    guard FileManager.default.fileExists(atPath: path) else {
        print("  fixture: jt128-frame.pcd not found at \(path) — skipped")
        return
    }

    do {
        let frame = try PointCloudDecoder.loadPCD(from: path)
        expect(frame.count == 50000, "jt128 fixture: expected 50000 points, got \(frame.count)")
        expect(frame.sensor == .jt128, "jt128 fixture: sensor should be jt128")
    } catch {
        expect(false, "jt128 fixture loading failed: \(error)")
    }
}

func testLoadDualFrameFixture() {
    let path = "fixtures/pointclouds/dual-frame.pcd"
    guard FileManager.default.fileExists(atPath: path) else {
        print("  fixture: dual-frame.pcd not found at \(path) — skipped")
        return
    }

    do {
        let frame = try PointCloudDecoder.loadPCD(from: path)
        expect(frame.count == 100000, "dual fixture: expected 100000 points, got \(frame.count)")
    } catch {
        expect(false, "dual fixture loading failed: \(error)")
    }
}

func testLoadEmptyFixture() {
    let path = "fixtures/pointclouds/empty.pcd"
    guard FileManager.default.fileExists(atPath: path) else {
        print("  fixture: empty.pcd not found at \(path) — skipped")
        return
    }

    do {
        let frame = try PointCloudDecoder.loadPCD(from: path)
        expect(frame.isEmpty, "empty fixture: should be empty")
        expect(frame.count == 0, "empty fixture: 0 points")
    } catch {
        expect(false, "empty fixture loading failed: \(error)")
    }
}

func testLoadMissingFixture() {
    let path = "fixtures/pointclouds/nonexistent.pcd"

    do {
        _ = try PointCloudDecoder.loadPCD(from: path)
        expect(false, "missing fixture: should have thrown")
    } catch {
        // Expected - file not found
    }
}

// MARK: - PointCloudFrame Struct Tests

func testPointCloudFrameStruct() {
    var frame = PointCloudFrame()
    expect(frame.isEmpty, "PointCloudFrame empty init: isEmpty")
    expect(frame.count == 0, "PointCloudFrame empty init: count")

    frame.positions = [SIMD3<Float>(1, 2, 3)]
    frame.intensities = [0.5]
    frame.sensor = .mid360

    expect(!frame.isEmpty, "PointCloudFrame with data: not empty")
    expect(frame.count == 1, "PointCloudFrame with data: count")
    expect(frame.sensor == .mid360, "PointCloudFrame: sensor")
}

// MARK: - Large Point Cloud Test

func testPCDLargeCloud() {
    var lines = [
        "VERSION 0.7",
        "FIELDS x y z intensity",
        "SIZE 4 4 4 4",
        "TYPE F F F F",
        "COUNT 1 1 1 1",
        "WIDTH 1000",
        "HEIGHT 1",
        "POINTS 1000",
        "DATA ascii",
    ]

    for i in 0..<1000 {
        let x = Float(i) * 0.1
        let y = Float(i) * 0.2
        let z = Float(i) * 0.3
        let intensity = Float(i) / 1000.0
        lines.append("\(x) \(y) \(z) \(intensity)")
    }

    let pcd = lines.joined(separator: "\n")

    do {
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 1000, "PCD large: expected 1000 points, got \(frame.count)")
        expect(abs(frame.positions[500].x - 50.0) < 0.01, "PCD large: spot check position")
        expect(abs(frame.intensities[500] - 0.5) < 0.01, "PCD large: spot check intensity")
    } catch {
        expect(false, "PCD large cloud failed: \(error)")
    }
}

// MARK: - Test Runner Entry Points

func runPointCloudDecoderTests() {
    testPCDBasicParsing()
    print("  PCD ASCII: basic parsing with x/y/z/intensity")
    testPCDEmptyCloud()
    print("  PCD ASCII: empty cloud (0 points)")
    testPCDWithoutIntensity()
    print("  PCD ASCII: cloud without intensity field → defaults to 1.0")
    testPCDMissingVersion()
    print("  PCD ASCII: missing VERSION header → error")
    testPCDMissingFields()
    print("  PCD ASCII: missing FIELDS header → error")
    testPCDMalformedPointSkipped()
    print("  PCD ASCII: truncated file with fewer points than declared → error")
    testPCDWithComments()
    print("  PCD ASCII: comments preserved in header")
    testPCDNaNAndInfValues()
    print("  PCD ASCII: NaN/Inf values filtered out")
    testPCDUnsupportedBinaryFormat()
    print("  PCD ASCII: binary format → unsupported error with docs")
    testPCDFromData()
    print("  PCD ASCII: parse from Data (not String)")
    testPCDMultipleWhitespace()
    print("  PCD ASCII: multiple whitespace delimiters")
    testPCDCountField()
    print("  PCD ASCII: COUNT field with multiple values per field")
    testPCDPointsWithoutWidth()
    print("  PCD ASCII: POINTS header without WIDTH")
    testPCDHeaderMetadata()
    print("  PCD ASCII: SIZE/TYPE/COUNT metadata preserved")
    testPCDLargeCloud()
    print("  PCD ASCII: 1000-point cloud parsed correctly")

    testPCDNegativeWidth()
    print("  PCD safety: negative WIDTH rejected")
    testPCDNegativeHeight()
    print("  PCD safety: negative HEIGHT rejected")
    testPCDOverflowProtection()
    print("  PCD safety: WIDTH * HEIGHT overflow rejected")
    testPCDTruncatedFile()
    print("  PCD safety: truncated file detected")

    testPCDSensorFromFilename()
    print("  PCD: sensor ID derived from filename")

    testPointCloud2BasicParsing()
    print("  PointCloud2: basic binary parsing")
    testPointCloud2EmptyData()
    print("  PointCloud2: empty data → empty cloud")
    testPointCloud2BigEndian()
    print("  PointCloud2: big-endian byte order")
    testPointCloud2LayoutFromFields()
    print("  PointCloud2: layout derived from fields array")
    testPointCloud2LayoutMissingFields()
    print("  PointCloud2: missing required fields → error")
    testPointCloud2InvalidPointStep()
    print("  PointCloud2: invalid pointStep → error")

    testROS1PointCloud2Header()
    print("  ROS1: PointCloud2 header parsing")
    testROS1UnknownDatatype()
    print("  ROS1: unknown datatype → error")

    testROS2PointCloud2Header()
    print("  ROS2: PointCloud2 with CDR encapsulation")
    testROS2InvalidEncapsulation()
    print("  ROS2: invalid CDR encapsulation → error")

    testLoadMid360Fixture()
    print("  Fixture: mid360-frame.pcd loaded")
    testLoadJT128Fixture()
    print("  Fixture: jt128-frame.pcd loaded")
    testLoadDualFrameFixture()
    print("  Fixture: dual-frame.pcd loaded")
    testLoadEmptyFixture()
    print("  Fixture: empty.pcd loaded")
    testLoadMissingFixture()
    print("  Fixture: missing file → error")

    testPointCloudFrameStruct()
    print("  PointCloudFrame struct: isEmpty, count, sensor")
}
