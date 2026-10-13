import Foundation
import LabCommandCore

// MARK: - Helper for Point3D comparison

func point3DEquals(_ a: Point3D, _ b: Point3D, tolerance: Float = 0.001) -> Bool {
    return abs(a.x - b.x) < tolerance && abs(a.y - b.y) < tolerance && abs(a.z - b.z) < tolerance
}

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
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
               "PCD basic: first point position mismatch")
        expect(point3DEquals(frame.positions[1], Point3D(x: 4.0, y: 5.0, z: 6.0)),
               "PCD basic: second point position mismatch")
        expect(point3DEquals(frame.positions[2], Point3D(x: 7.0, y: 8.0, z: 9.0)),
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
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.5, y: 2.5, z: 3.5)),
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

func testPCDMalformedLineIsError() {
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
        expect(false, "PCD malformed: should throw for unparseable line")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("Malformed") || msg.contains("parse"), "PCD malformed: error should mention malformed data")
    } catch {
        expect(false, "PCD malformed: unexpected error \(error)")
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
    // NaN/Inf lines are valid but filtered — they count as data lines
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
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        // 4 data lines, 2 valid points (NaN/Inf filtered out)
        expect(frame.count == 2, "PCD NaN/Inf: expected 2 valid points after filtering, got \(frame.count)")
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
               "PCD NaN/Inf: first valid point")
        expect(point3DEquals(frame.positions[1], Point3D(x: 7.0, y: 8.0, z: 9.0)),
               "PCD NaN/Inf: second valid point")
    } catch {
        expect(false, "PCD NaN/Inf parsing failed: \(error)")
    }
}

func testPCDNaNSkippedWithCorrectCount() {
    // POINTS declares 4, we have 4 data lines, 2 are NaN/Inf → 2 valid points output
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
        let frame = try PointCloudDecoder.decodePCD(from: pcd)
        expect(frame.count == 2, "PCD NaN correct count: NaN/Inf don't count against valid points")
    } catch {
        expect(false, "PCD NaN with correct count failed: \(error)")
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
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
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
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
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
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
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

func testPCDValidateSizeTypeCounts() {
    // SIZE count doesn't match FIELDS
    let pcd1 = """
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4
    TYPE F F F
    WIDTH 1
    HEIGHT 1
    POINTS 1
    DATA ascii
    1.0 2.0 3.0
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd1)
        expect(false, "PCD SIZE mismatch: should have thrown")
    } catch PointCloudError.invalidHeader(let msg) {
        expect(msg.contains("SIZE") && msg.contains("FIELDS"), "PCD SIZE: error mentions SIZE and FIELDS")
    } catch {
        expect(false, "PCD SIZE mismatch: unexpected error \(error)")
    }

    // TYPE count doesn't match FIELDS
    let pcd2 = """
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F
    WIDTH 1
    HEIGHT 1
    POINTS 1
    DATA ascii
    1.0 2.0 3.0
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd2)
        expect(false, "PCD TYPE mismatch: should have thrown")
    } catch PointCloudError.invalidHeader(let msg) {
        expect(msg.contains("TYPE") && msg.contains("FIELDS"), "PCD TYPE: error mentions TYPE and FIELDS")
    } catch {
        expect(false, "PCD TYPE mismatch: unexpected error \(error)")
    }

    // COUNT count doesn't match FIELDS  
    let pcd3 = """
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1
    WIDTH 1
    HEIGHT 1
    POINTS 1
    DATA ascii
    1.0 2.0 3.0
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd3)
        expect(false, "PCD COUNT mismatch: should have thrown")
    } catch PointCloudError.invalidHeader(let msg) {
        expect(msg.contains("COUNT") && msg.contains("FIELDS"), "PCD COUNT: error mentions COUNT and FIELDS")
    } catch {
        expect(false, "PCD COUNT mismatch: unexpected error \(error)")
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
        expect(msg.contains("Truncated") || msg.contains("expected"),
               "PCD truncated: error mentions truncation")
    } catch {
        expect(false, "PCD truncated: unexpected error \(error)")
    }
}

func testPCDDataOverrun() {
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
    1.0 2.0 3.0
    4.0 5.0 6.0
    7.0 8.0 9.0
    """

    do {
        _ = try PointCloudDecoder.decodePCD(from: pcd)
        expect(false, "PCD overrun: should throw when file has more data than declared")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("overrun"), "PCD overrun: error mentions overrun")
    } catch {
        expect(false, "PCD overrun: unexpected error \(error)")
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
    // Realistic layout: x, y, z, intensity at typical offsets
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
        rowStep: 48,  // 3 points × 16 bytes
        width: 3,
        height: 1,
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
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
               "PC2 basic: first point mismatch")
        expect(abs(frame.intensities[2] - 1.0) < 0.001, "PC2 basic: third intensity mismatch")
    } catch {
        expect(false, "PC2 basic parsing failed: \(error)")
    }
}

func testPointCloud2EmptyData() {
    let data = Data()
    let layout = PointCloud2Layout(
        pointStep: 16,
        rowStep: 0,
        width: 0,
        height: 1,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8
    )

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
        rowStep: 16,
        width: 1,
        height: 1,
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
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
               "PC2 big endian: position mismatch")
    } catch {
        expect(false, "PC2 big endian failed: \(error)")
    }
}

func testPointCloud2LayoutFromFields() {
    // Realistic Livox Mid-360 layout
    let fields = [
        PointCloud2Layout.FieldDescriptor(name: "x", offset: 0, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "y", offset: 4, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "z", offset: 8, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "intensity", offset: 12, datatype: .float32),
    ]

    do {
        let layout = try PointCloud2Layout.from(fields: fields, pointStep: 16, rowStep: 160, width: 10, height: 1)
        expect(layout.xOffset == 0, "Layout from fields: xOffset")
        expect(layout.yOffset == 4, "Layout from fields: yOffset")
        expect(layout.zOffset == 8, "Layout from fields: zOffset")
        expect(layout.intensityOffset == 12, "Layout from fields: intensityOffset")
        expect(layout.xType == .float32, "Layout from fields: xType")
        expect(layout.hasIntensity, "Layout from fields: hasIntensity")
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

func testPointCloud2LayoutNoIntensity() {
    let fields = [
        PointCloud2Layout.FieldDescriptor(name: "x", offset: 0, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "y", offset: 4, datatype: .float32),
        PointCloud2Layout.FieldDescriptor(name: "z", offset: 8, datatype: .float32),
    ]

    do {
        let layout = try PointCloud2Layout.from(fields: fields, pointStep: 12)
        expect(!layout.hasIntensity, "Layout no intensity: hasIntensity should be false")
        expect(layout.intensityOffset == nil, "Layout no intensity: intensityOffset should be nil")
    } catch {
        expect(false, "Layout no intensity failed: \(error)")
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

func testPointCloud2PointCountMismatch() {
    var data = Data()
    // Create 3 points of 16 bytes each
    for _ in 0..<3 {
        data.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(2.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(3.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(0.5).bitPattern.littleEndian) { Array($0) })
    }

    let layout = PointCloud2Layout(
        pointStep: 16,
        rowStep: 48,  // 3 points × 16
        width: 3,
        height: 1,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8,
        intensityOffset: 12
    )

    do {
        // Pass pointCount=5 but layout says width×height=3
        _ = try PointCloudDecoder.decodePointCloud2(from: data, layout: layout, pointCount: 5)
        expect(false, "PC2 pointCount mismatch: should have thrown")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("mismatch") || msg.contains("pointCount"), "PC2 pointCount mismatch: error mentions mismatch")
    } catch {
        expect(false, "PC2 pointCount mismatch: unexpected error \(error)")
    }
}

func testPointCloud2DataSizeMismatch() {
    var data = Data()
    // Create 2 points of 16 bytes each
    for _ in 0..<2 {
        data.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(2.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(3.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(0.5).bitPattern.littleEndian) { Array($0) })
    }
    // Add extra bytes to trigger mismatch
    data.append(contentsOf: [0, 1, 2, 3])

    let layout = PointCloud2Layout(
        pointStep: 16,
        rowStep: 32,  // 2 points
        width: 2,
        height: 1,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8,
        intensityOffset: 12
    )

    do {
        _ = try PointCloudDecoder.decodePointCloud2(from: data, layout: layout)
        expect(false, "PC2 data mismatch: should have thrown")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("mismatch") || msg.contains("row_step"), "PC2 data mismatch: error mentions mismatch")
    } catch {
        expect(false, "PC2 data mismatch: unexpected error \(error)")
    }
}

func testPointCloud2RowStepAddressing() {
    // Test that row_step is used for multi-row clouds
    var data = Data()
    
    // Row 0: 2 points + padding (row_step = 48, point_step = 16, 2 points = 32 bytes, 16 bytes padding)
    data.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(2.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(3.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(0.5).bitPattern.littleEndian) { Array($0) })
    
    data.append(contentsOf: withUnsafeBytes(of: Float(4.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(5.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(6.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(0.8).bitPattern.littleEndian) { Array($0) })
    
    // Padding for row 0
    data.append(contentsOf: [UInt8](repeating: 0, count: 16))
    
    // Row 1: 2 points + padding
    data.append(contentsOf: withUnsafeBytes(of: Float(7.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(8.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(9.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })
    
    data.append(contentsOf: withUnsafeBytes(of: Float(10.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(11.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(12.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(0.9).bitPattern.littleEndian) { Array($0) })
    
    // Padding for row 1
    data.append(contentsOf: [UInt8](repeating: 0, count: 16))

    let layout = PointCloud2Layout(
        pointStep: 16,
        rowStep: 48,
        width: 2,
        height: 2,
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
        expect(frame.count == 4, "PC2 row_step: expected 4 points, got \(frame.count)")
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
               "PC2 row_step: point (0,0)")
        expect(point3DEquals(frame.positions[1], Point3D(x: 4.0, y: 5.0, z: 6.0)),
               "PC2 row_step: point (0,1)")
        expect(point3DEquals(frame.positions[2], Point3D(x: 7.0, y: 8.0, z: 9.0)),
               "PC2 row_step: point (1,0)")
        expect(point3DEquals(frame.positions[3], Point3D(x: 10.0, y: 11.0, z: 12.0)),
               "PC2 row_step: point (1,1)")
    } catch {
        expect(false, "PC2 row_step addressing failed: \(error)")
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

    // std_msgs/Header
    appendU32(42)              // seq
    appendU32(1609459200)      // stamp.sec (2021-01-01 00:00:00 UTC)
    appendU32(500_000_000)     // stamp.nsec
    appendString("lidar_frame") // frame_id

    // PointCloud2 body
    appendU32(1)  // height
    appendU32(2)  // width

    appendU32(3)  // num fields
    appendString("x")
    appendU32(0)  // offset
    appendU8(7)   // float32
    appendU32(1)  // count

    appendString("y")
    appendU32(4)
    appendU8(7)
    appendU32(1)

    appendString("z")
    appendU32(8)
    appendU8(7)
    appendU32(1)

    appendU8(0)   // is_bigendian
    appendU32(12) // point_step
    appendU32(24) // row_step
    appendU32(24) // data length

    let points: [(Float, Float, Float)] = [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]
    for (x, y, z) in points {
        data.append(contentsOf: withUnsafeBytes(of: x.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: y.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: z.bitPattern.littleEndian) { Array($0) })
    }

    appendU8(1)  // is_dense

    do {
        let frame = try PointCloudDecoder.decodeROS1PointCloud2(from: data)
        expect(frame.count == 2, "ROS1 header: expected 2 points, got \(frame.count)")
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
               "ROS1 header: first point mismatch")
        expect(point3DEquals(frame.positions[1], Point3D(x: 4.0, y: 5.0, z: 6.0)),
               "ROS1 header: second point mismatch")
        expect(frame.frameId == "lidar_frame", "ROS1 header: frame_id")
        expect(frame.timestamp != nil, "ROS1 header: timestamp parsed")
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

    // std_msgs/Header
    appendU32(0)       // seq
    appendU32(0)       // stamp.sec
    appendU32(0)       // stamp.nsec
    appendString("")   // frame_id

    appendU32(1)  // height
    appendU32(1)  // width

    appendU32(1)  // num fields
    appendString("x")
    appendU32(0)
    appendU8(99)  // unknown datatype
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

func testROS1DataSizeMismatch() {
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

    // std_msgs/Header
    appendU32(0)       // seq
    appendU32(0)       // stamp.sec
    appendU32(0)       // stamp.nsec
    appendString("")   // frame_id

    appendU32(2)  // height = 2
    appendU32(2)  // width = 2

    appendU32(3)  // num fields
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

    appendU8(0)   // is_bigendian
    appendU32(12) // point_step
    appendU32(24) // row_step = 24 (2 points × 12)
    appendU32(24) // data length = 24 (WRONG: should be 48 for 2 rows)

    // Only add data for 2 points instead of 4
    for _ in 0..<2 {
        data.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(2.0).bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: Float(3.0).bitPattern.littleEndian) { Array($0) })
    }

    appendU8(1)  // is_dense

    do {
        _ = try PointCloudDecoder.decodeROS1PointCloud2(from: data)
        expect(false, "ROS1 data mismatch: should have thrown")
    } catch PointCloudError.invalidData(let msg) {
        expect(msg.contains("mismatch") || msg.contains("row_step"), "ROS1 data mismatch: error mentions mismatch")
    } catch {
        expect(false, "ROS1 data mismatch: unexpected error \(error)")
    }
}

// MARK: - ROS2 Header Tests

func testROS2PointCloud2Header() {
    var data = Data()

    func appendU32(_ value: UInt32) {
        data.append(contentsOf: withUnsafeBytes(of: value.littleEndian) { Array($0) })
    }
    func appendI32(_ value: Int32) {
        data.append(contentsOf: withUnsafeBytes(of: value.littleEndian) { Array($0) })
    }
    func appendU8(_ value: UInt8) {
        data.append(value)
    }
    func appendStringCDR(_ str: String) {
        // CDR string: uint32 length (including NUL) + chars + NUL
        let bytes = Array(str.utf8)
        appendU32(UInt32(bytes.count + 1))
        data.append(contentsOf: bytes)
        data.append(0)  // NUL terminator
    }
    func alignTo4() {
        let rem = data.count % 4
        if rem != 0 {
            data.append(contentsOf: [UInt8](repeating: 0, count: 4 - rem))
        }
    }

    // CDR header: little-endian
    data.append(contentsOf: [0x00, 0x01, 0x00, 0x00])

    // std_msgs/Header (ROS2 format)
    appendI32(1609459200)      // stamp.sec
    appendU32(500_000_000)     // stamp.nsec
    appendStringCDR("base_link") // frame_id with NUL

    alignTo4()

    // PointCloud2 body
    appendU32(1)  // height
    appendU32(1)  // width

    appendU32(3)  // num fields
    
    appendStringCDR("x")
    alignTo4()
    appendU32(0)  // offset
    appendU8(7)   // float32
    alignTo4()
    appendU32(1)  // count

    appendStringCDR("y")
    alignTo4()
    appendU32(4)
    appendU8(7)
    alignTo4()
    appendU32(1)

    appendStringCDR("z")
    alignTo4()
    appendU32(8)
    appendU8(7)
    alignTo4()
    appendU32(1)

    appendU8(0)   // is_bigendian
    alignTo4()
    appendU32(12) // point_step
    appendU32(12) // row_step
    appendU32(12) // data length

    data.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(2.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(3.0).bitPattern.littleEndian) { Array($0) })

    appendU8(1)  // is_dense

    do {
        let frame = try PointCloudDecoder.decodeROS2PointCloud2(from: data)
        expect(frame.count == 1, "ROS2 header: expected 1 point, got \(frame.count)")
        expect(point3DEquals(frame.positions[0], Point3D(x: 1.0, y: 2.0, z: 3.0)),
               "ROS2 header: point mismatch")
        expect(frame.frameId == "base_link", "ROS2 header: frame_id stripped NUL")
        expect(frame.timestamp != nil, "ROS2 header: timestamp parsed")
    } catch {
        expect(false, "ROS2 header parsing failed: \(error)")
    }
}

func testROS2BigEndianEncapsulation() {
    var data = Data()

    func appendU32BE(_ value: UInt32) {
        data.append(contentsOf: withUnsafeBytes(of: value.bigEndian) { Array($0) })
    }
    func appendI32BE(_ value: Int32) {
        data.append(contentsOf: withUnsafeBytes(of: value.bigEndian) { Array($0) })
    }
    func appendU8(_ value: UInt8) {
        data.append(value)
    }
    func appendStringCDR(_ str: String) {
        let bytes = Array(str.utf8)
        appendU32BE(UInt32(bytes.count + 1))
        data.append(contentsOf: bytes)
        data.append(0)
    }
    func alignTo4() {
        let rem = data.count % 4
        if rem != 0 {
            data.append(contentsOf: [UInt8](repeating: 0, count: 4 - rem))
        }
    }

    // CDR header: big-endian (encapsulation kind = 0x00)
    data.append(contentsOf: [0x00, 0x00, 0x00, 0x00])

    // std_msgs/Header
    appendI32BE(1609459200)
    appendU32BE(0)
    appendStringCDR("lidar")

    alignTo4()

    appendU32BE(1)  // height
    appendU32BE(1)  // width

    appendU32BE(3)  // num fields
    
    appendStringCDR("x")
    alignTo4()
    appendU32BE(0)
    appendU8(7)
    alignTo4()
    appendU32BE(1)

    appendStringCDR("y")
    alignTo4()
    appendU32BE(4)
    appendU8(7)
    alignTo4()
    appendU32BE(1)

    appendStringCDR("z")
    alignTo4()
    appendU32BE(8)
    appendU8(7)
    alignTo4()
    appendU32BE(1)

    appendU8(1)   // is_bigendian = true
    alignTo4()
    appendU32BE(12) // point_step
    appendU32BE(12) // row_step
    appendU32BE(12) // data length

    // Point data in big-endian
    data.append(contentsOf: withUnsafeBytes(of: Float(5.0).bitPattern.bigEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(6.0).bitPattern.bigEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(7.0).bitPattern.bigEndian) { Array($0) })

    appendU8(1)  // is_dense

    do {
        let frame = try PointCloudDecoder.decodeROS2PointCloud2(from: data)
        expect(frame.count == 1, "ROS2 BE: expected 1 point, got \(frame.count)")
        expect(point3DEquals(frame.positions[0], Point3D(x: 5.0, y: 6.0, z: 7.0)),
               "ROS2 BE: point mismatch")
    } catch {
        expect(false, "ROS2 big-endian parsing failed: \(error)")
    }
}

func testROS2InvalidEncapsulation() {
    let data = Data([0x00, 0x02, 0x00, 0x00])  // Invalid encapsulation kind (0x02)

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
        passed()
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
        passed()
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
        passed()
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
        passed()
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
    
    // Empty frame bounds should have inverted min/max
    let emptyBounds = frame.bounds
    expect(emptyBounds.min.x == .greatestFiniteMagnitude, "Empty bounds min.x")
    expect(emptyBounds.max.x == -.greatestFiniteMagnitude, "Empty bounds max.x")

    frame.positions = [Point3D(x: 1, y: 2, z: 3)]
    frame.intensities = [0.5]
    frame.sensor = .mid360
    frame.frameId = "test_frame"

    expect(!frame.isEmpty, "PointCloudFrame with data: not empty")
    expect(frame.count == 1, "PointCloudFrame with data: count")
    expect(frame.sensor == .mid360, "PointCloudFrame: sensor")
    expect(frame.frameId == "test_frame", "PointCloudFrame: frameId")
    
    // Single point bounds
    let singleBounds = frame.bounds
    expect(singleBounds.min.x == 1 && singleBounds.min.y == 2 && singleBounds.min.z == 3,
           "Single point bounds min")
    expect(singleBounds.max.x == 1 && singleBounds.max.y == 2 && singleBounds.max.z == 3,
           "Single point bounds max")
    
    // Multiple points bounds
    frame.positions = [
        Point3D(x: -5, y: 0, z: 10),
        Point3D(x: 5, y: -3, z: 2),
        Point3D(x: 0, y: 7, z: -1)
    ]
    let multiBounds = frame.bounds
    expect(multiBounds.min.x == -5 && multiBounds.min.y == -3 && multiBounds.min.z == -1,
           "Multi-point bounds min: (\(multiBounds.min.x), \(multiBounds.min.y), \(multiBounds.min.z))")
    expect(multiBounds.max.x == 5 && multiBounds.max.y == 7 && multiBounds.max.z == 10,
           "Multi-point bounds max: (\(multiBounds.max.x), \(multiBounds.max.y), \(multiBounds.max.z))")
    
    // Bounds with NaN/Inf points (should be skipped)
    frame.positions = [
        Point3D(x: 1, y: 2, z: 3),
        Point3D(x: .nan, y: 0, z: 0),
        Point3D(x: 0, y: .infinity, z: 0),
        Point3D(x: 4, y: 5, z: 6)
    ]
    let filteredBounds = frame.bounds
    expect(filteredBounds.min.x == 1 && filteredBounds.min.y == 2 && filteredBounds.min.z == 3,
           "Bounds skip NaN/Inf min")
    expect(filteredBounds.max.x == 4 && filteredBounds.max.y == 5 && filteredBounds.max.z == 6,
           "Bounds skip NaN/Inf max")
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

// MARK: - Real Fixture Regression Test

func testRealROS2MessageFixture() {
    // Message 0 extracted from room-scan.mcap (1745 bytes)
    // This is a ROS2 PointCloud2 message with CDR encapsulation
    let path = "fixtures/pointclouds/ros2-pointcloud2.bin"
    
    // FAIL if fixture is missing — don't skip
    guard FileManager.default.fileExists(atPath: path) else {
        expect(false, "ROS2 real fixture: ros2-pointcloud2.bin not found at \(path)")
        return
    }

    do {
        let data = try Data(contentsOf: URL(fileURLWithPath: path))
        expect(data.count == 1745, "ROS2 real fixture: expected 1745 bytes, got \(data.count)")
        
        let frame = try PointCloudDecoder.decodeROS2PointCloud2(from: data)
        
        // Assert: 100 points
        expect(frame.count == 100, "ROS2 real fixture: expected 100 points, got \(frame.count)")
        
        // Assert: frame_id is "livox_frame"
        expect(frame.frameId == "livox_frame", 
               "ROS2 real fixture: expected frame_id='livox_frame', got '\(frame.frameId ?? "nil")'")
        
        // Assert: first point xyz (all zeros in this fixture)
        expect(frame.positions.count >= 1, "ROS2 real fixture: should have at least 1 position")
        let first = frame.positions[0]
        expect(abs(first.x) < 0.001 && abs(first.y) < 0.001 && abs(first.z) < 0.001,
               "ROS2 real fixture: first point should be (0,0,0), got (\(first.x),\(first.y),\(first.z))")
        
        // Second point should be (0.1, 0, 0)
        if frame.positions.count >= 2 {
            let second = frame.positions[1]
            expect(abs(second.x - 0.1) < 0.001 && abs(second.y) < 0.001 && abs(second.z) < 0.001,
                   "ROS2 real fixture: second point should be (0.1,0,0), got (\(second.x),\(second.y),\(second.z))")
        }
    } catch {
        expect(false, "ROS2 real fixture failed: \(error)")
    }
}

// MARK: - Test Runner Entry Points

func runPointCloudDecoderTests() {
    testPCDBasicParsing()
    print("  PCD ASCII: basic parsing with x/y/z/intensity"); passed()
    testPCDEmptyCloud()
    print("  PCD ASCII: empty cloud (0 points)"); passed()
    testPCDWithoutIntensity()
    print("  PCD ASCII: cloud without intensity field → defaults to 1.0"); passed()
    testPCDMissingVersion()
    print("  PCD ASCII: missing VERSION header → error"); passed()
    testPCDMissingFields()
    print("  PCD ASCII: missing FIELDS header → error"); passed()
    testPCDMalformedLineIsError()
    print("  PCD ASCII: malformed data line → error (not silent skip)"); passed()
    testPCDWithComments()
    print("  PCD ASCII: comments preserved in header"); passed()
    testPCDNaNAndInfValues()
    print("  PCD ASCII: NaN/Inf lines filtered but valid"); passed()
    testPCDNaNSkippedWithCorrectCount()
    print("  PCD ASCII: NaN/Inf don't count against POINTS"); passed()
    testPCDUnsupportedBinaryFormat()
    print("  PCD ASCII: binary format → unsupported error with docs"); passed()
    testPCDFromData()
    print("  PCD ASCII: parse from Data (not String)"); passed()
    testPCDMultipleWhitespace()
    print("  PCD ASCII: multiple whitespace delimiters"); passed()
    testPCDCountField()
    print("  PCD ASCII: COUNT field with multiple values per field"); passed()
    testPCDPointsWithoutWidth()
    print("  PCD ASCII: POINTS header without WIDTH"); passed()
    testPCDHeaderMetadata()
    print("  PCD ASCII: SIZE/TYPE/COUNT metadata preserved"); passed()
    testPCDValidateSizeTypeCounts()
    print("  PCD ASCII: SIZE/TYPE/COUNT lengths validated"); passed()
    testPCDLargeCloud()
    print("  PCD ASCII: 1000-point cloud parsed correctly"); passed()

    testPCDNegativeWidth()
    print("  PCD safety: negative WIDTH rejected"); passed()
    testPCDNegativeHeight()
    print("  PCD safety: negative HEIGHT rejected"); passed()
    testPCDOverflowProtection()
    print("  PCD safety: WIDTH * HEIGHT overflow rejected"); passed()
    testPCDTruncatedFile()
    print("  PCD safety: truncated file detected"); passed()
    testPCDDataOverrun()
    print("  PCD safety: data overrun detected"); passed()

    testPCDSensorFromFilename()
    print("  PCD: sensor ID derived from filename"); passed()

    testPointCloud2BasicParsing()
    print("  PointCloud2: basic binary parsing with realistic layout"); passed()
    testPointCloud2EmptyData()
    print("  PointCloud2: empty data → empty cloud"); passed()
    testPointCloud2BigEndian()
    print("  PointCloud2: big-endian byte order"); passed()
    testPointCloud2LayoutFromFields()
    print("  PointCloud2: layout derived from fields array"); passed()
    testPointCloud2LayoutMissingFields()
    print("  PointCloud2: missing required fields → error"); passed()
    testPointCloud2LayoutNoIntensity()
    print("  PointCloud2: missing intensity flagged via hasIntensity"); passed()
    testPointCloud2InvalidPointStep()
    print("  PointCloud2: invalid pointStep → error"); passed()
    testPointCloud2PointCountMismatch()
    print("  PointCloud2: pointCount != width×height → error"); passed()
    testPointCloud2DataSizeMismatch()
    print("  PointCloud2: data size != row_step × height → error"); passed()
    testPointCloud2RowStepAddressing()
    print("  PointCloud2: row_step used for multi-row addressing"); passed()

    testROS1PointCloud2Header()
    print("  ROS1: std_msgs/Header parsed before height/width"); passed()
    testROS1UnknownDatatype()
    print("  ROS1: unknown datatype → error"); passed()
    testROS1DataSizeMismatch()
    print("  ROS1: data size mismatch → error"); passed()

    testROS2PointCloud2Header()
    print("  ROS2: CDR encapsulation + Header + NUL-stripped strings"); passed()
    testROS2BigEndianEncapsulation()
    print("  ROS2: CDR big-endian encapsulation honored"); passed()
    testROS2InvalidEncapsulation()
    print("  ROS2: invalid CDR encapsulation → error"); passed()

    testLoadMid360Fixture()
    print("  Fixture: mid360-frame.pcd"); passed()
    testLoadJT128Fixture()
    print("  Fixture: jt128-frame.pcd"); passed()
    testLoadDualFrameFixture()
    print("  Fixture: dual-frame.pcd"); passed()
    testLoadEmptyFixture()
    print("  Fixture: empty.pcd"); passed()
    testLoadMissingFixture()
    print("  Fixture: missing file → error"); passed()

    testRealROS2MessageFixture()
    print("  Fixture: ROS2 PointCloud2 binary from room-scan.mcap (100 points, livox_frame)"); passed()

    testPointCloudFrameStruct()
    print("  PointCloudFrame struct: isEmpty, count, sensor, frameId, bounds"); passed()
}
