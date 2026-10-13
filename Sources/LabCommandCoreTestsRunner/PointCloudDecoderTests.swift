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
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.count == 3, "PCD basic: expected 3 points, got \(cloud.count)")
        expect(cloud.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PCD basic: first point position mismatch")
        expect(cloud.positions[1] == SIMD3<Float>(4.0, 5.0, 6.0),
               "PCD basic: second point position mismatch")
        expect(cloud.positions[2] == SIMD3<Float>(7.0, 8.0, 9.0),
               "PCD basic: third point position mismatch")
        expect(abs(cloud.intensities[0] - 0.5) < 0.001, "PCD basic: first intensity mismatch")
        expect(abs(cloud.intensities[1] - 0.8) < 0.001, "PCD basic: second intensity mismatch")
        expect(abs(cloud.intensities[2] - 1.0) < 0.001, "PCD basic: third intensity mismatch")
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
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.isEmpty, "PCD empty: expected empty cloud")
        expect(cloud.count == 0, "PCD empty: expected 0 points")
        expect(cloud.positions.isEmpty, "PCD empty: positions not empty")
        expect(cloud.intensities.isEmpty, "PCD empty: intensities not empty")
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
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.count == 2, "PCD no intensity: expected 2 points, got \(cloud.count)")
        expect(cloud.positions[0] == SIMD3<Float>(1.5, 2.5, 3.5),
               "PCD no intensity: first point mismatch")
        expect(abs(cloud.intensities[0] - 1.0) < 0.001,
               "PCD no intensity: default intensity should be 1.0")
        expect(abs(cloud.intensities[1] - 1.0) < 0.001,
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
    } catch PointCloudDecoderError.missingHeader(let field) {
        expect(field == "VERSION", "PCD missing version: expected VERSION header error")
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
    } catch PointCloudDecoderError.missingHeader(let field) {
        expect(field == "FIELDS", "PCD missing fields: expected FIELDS header error")
    } catch {
        expect(false, "PCD missing fields: unexpected error \(error)")
    }
}

func testPCDMalformedPointData() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 3
    HEIGHT 1
    POINTS 3
    DATA ascii
    1.0 2.0 3.0 0.5
    not valid data
    7.0 8.0 9.0 1.0
    """

    do {
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.count == 2, "PCD malformed: should skip invalid line, got \(cloud.count) points")
        expect(cloud.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PCD malformed: first valid point")
        expect(cloud.positions[1] == SIMD3<Float>(7.0, 8.0, 9.0),
               "PCD malformed: second valid point")
    } catch {
        expect(false, "PCD malformed data parsing failed: \(error)")
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
    # Comment in data section should still work
    1.0 2.0 3.0 0.5
    4.0 5.0 6.0 0.8
    """

    do {
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.count == 2, "PCD comments: expected 2 points")
    } catch {
        expect(false, "PCD with comments failed: \(error)")
    }
}

func testPCDNaNValues() {
    let pcd = """
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 3
    HEIGHT 1
    POINTS 3
    DATA ascii
    1.0 2.0 3.0 0.5
    nan nan nan 0.0
    7.0 8.0 9.0 1.0
    """

    do {
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.count == 2, "PCD NaN: should skip NaN points, got \(cloud.count)")
    } catch {
        expect(false, "PCD NaN handling failed: \(error)")
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
    } catch PointCloudDecoderError.unsupportedDataFormat {
        // Expected
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
        let cloud = try PointCloudDecoder.decodePCD(from: data)
        expect(cloud.count == 1, "PCD from data: expected 1 point")
        expect(cloud.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PCD from data: position mismatch")
    } catch {
        expect(false, "PCD from data failed: \(error)")
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
        isBigEndian: false,
        xType: .float32,
        yType: .float32,
        zType: .float32,
        intensityType: .float32
    )

    do {
        let cloud = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(cloud.count == 3, "PC2 basic: expected 3 points, got \(cloud.count)")
        expect(cloud.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PC2 basic: first point mismatch")
        expect(cloud.positions[1] == SIMD3<Float>(4.0, 5.0, 6.0),
               "PC2 basic: second point mismatch")
        expect(cloud.positions[2] == SIMD3<Float>(7.0, 8.0, 9.0),
               "PC2 basic: third point mismatch")
        expect(abs(cloud.intensities[0] - 0.5) < 0.001, "PC2 basic: first intensity mismatch")
        expect(abs(cloud.intensities[1] - 0.8) < 0.001, "PC2 basic: second intensity mismatch")
        expect(abs(cloud.intensities[2] - 1.0) < 0.001, "PC2 basic: third intensity mismatch")
    } catch {
        expect(false, "PC2 basic parsing failed: \(error)")
    }
}

func testPointCloud2EmptyData() {
    let data = Data()
    let layout = PointCloud2Layout.livoxMid360

    do {
        let cloud = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(cloud.isEmpty, "PC2 empty: expected empty cloud")
        expect(cloud.count == 0, "PC2 empty: expected 0 points")
    } catch {
        expect(false, "PC2 empty data failed: \(error)")
    }
}

func testPointCloud2NoIntensity() {
    var data = Data()

    let points: [(Float, Float, Float)] = [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]

    for (x, y, z) in points {
        data.append(contentsOf: withUnsafeBytes(of: x.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: y.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: z.bitPattern.littleEndian) { Array($0) })
    }

    let layout = PointCloud2Layout(
        pointStep: 12,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8,
        intensityOffset: nil,
        isBigEndian: false,
        xType: .float32,
        yType: .float32,
        zType: .float32,
        intensityType: nil
    )

    do {
        let cloud = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(cloud.count == 2, "PC2 no intensity: expected 2 points")
        expect(abs(cloud.intensities[0] - 1.0) < 0.001,
               "PC2 no intensity: default should be 1.0")
    } catch {
        expect(false, "PC2 no intensity failed: \(error)")
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
        let cloud = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(cloud.count == 1, "PC2 big endian: expected 1 point")
        expect(cloud.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PC2 big endian: position mismatch")
        expect(abs(cloud.intensities[0] - 0.5) < 0.001, "PC2 big endian: intensity mismatch")
    } catch {
        expect(false, "PC2 big endian failed: \(error)")
    }
}

func testPointCloud2WithPadding() {
    var data = Data()

    let x: Float = 1.0
    let y: Float = 2.0
    let z: Float = 3.0
    let i: Float = 0.5
    let padding: UInt64 = 0

    data.append(contentsOf: withUnsafeBytes(of: x.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: y.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: z.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: i.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: padding.littleEndian) { Array($0) })

    data.append(contentsOf: withUnsafeBytes(of: (x + 3.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: (y + 3.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: (z + 3.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(0.9).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: padding.littleEndian) { Array($0) })

    let layout = PointCloud2Layout(
        pointStep: 24,
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

    do {
        let cloud = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(cloud.count == 2, "PC2 padding: expected 2 points, got \(cloud.count)")
        expect(cloud.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PC2 padding: first point mismatch")
        expect(cloud.positions[1] == SIMD3<Float>(4.0, 5.0, 6.0),
               "PC2 padding: second point mismatch")
    } catch {
        expect(false, "PC2 with padding failed: \(error)")
    }
}

func testPointCloud2NaNSkipped() {
    var data = Data()

    let validX: Float = 1.0
    let validY: Float = 2.0
    let validZ: Float = 3.0
    let validI: Float = 0.5
    let nanValue: Float = .nan

    data.append(contentsOf: withUnsafeBytes(of: validX.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: validY.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: validZ.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: validI.bitPattern.littleEndian) { Array($0) })

    data.append(contentsOf: withUnsafeBytes(of: nanValue.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: nanValue.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: nanValue.bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: validI.bitPattern.littleEndian) { Array($0) })

    data.append(contentsOf: withUnsafeBytes(of: Float(7.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(8.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(9.0).bitPattern.littleEndian) { Array($0) })
    data.append(contentsOf: withUnsafeBytes(of: Float(1.0).bitPattern.littleEndian) { Array($0) })

    let layout = PointCloud2Layout.livoxMid360

    do {
        let cloud = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(cloud.count == 2, "PC2 NaN: should skip NaN point, got \(cloud.count)")
        expect(cloud.positions[0] == SIMD3<Float>(1.0, 2.0, 3.0),
               "PC2 NaN: first valid point")
        expect(cloud.positions[1] == SIMD3<Float>(7.0, 8.0, 9.0),
               "PC2 NaN: second valid point")
    } catch {
        expect(false, "PC2 NaN handling failed: \(error)")
    }
}

func testPointCloud2InvalidPointStep() {
    let data = Data([0, 1, 2, 3])
    let layout = PointCloud2Layout(
        pointStep: 0,
        xOffset: 0,
        yOffset: 4,
        zOffset: 8
    )

    do {
        _ = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(false, "PC2 invalid step: should have thrown")
    } catch PointCloudDecoderError.invalidFormat {
        // Expected
    } catch {
        expect(false, "PC2 invalid step: unexpected error \(error)")
    }
}

func testPointCloud2PresetLayouts() {
    expect(PointCloud2Layout.livoxMid360.pointStep == 16, "Livox preset: pointStep")
    expect(PointCloud2Layout.livoxMid360.xOffset == 0, "Livox preset: xOffset")
    expect(PointCloud2Layout.livoxMid360.yOffset == 4, "Livox preset: yOffset")
    expect(PointCloud2Layout.livoxMid360.zOffset == 8, "Livox preset: zOffset")
    expect(PointCloud2Layout.livoxMid360.intensityOffset == 12, "Livox preset: intensityOffset")

    expect(PointCloud2Layout.hesaiJT128.pointStep == 32, "Hesai preset: pointStep")
    expect(PointCloud2Layout.hesaiJT128.xOffset == 0, "Hesai preset: xOffset")
    expect(PointCloud2Layout.hesaiJT128.intensityOffset == 16, "Hesai preset: intensityOffset")
}

func testPointCloudStruct() {
    var cloud = PointCloud()
    expect(cloud.isEmpty, "PointCloud empty init: isEmpty")
    expect(cloud.count == 0, "PointCloud empty init: count")

    cloud.positions = [SIMD3<Float>(1, 2, 3)]
    cloud.intensities = [0.5]
    cloud.sensorID = "mid360"

    expect(!cloud.isEmpty, "PointCloud with data: not empty")
    expect(cloud.count == 1, "PointCloud with data: count")
    expect(cloud.sensorID == "mid360", "PointCloud: sensorID")
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
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.count == 1000, "PCD large: expected 1000 points, got \(cloud.count)")
        expect(abs(cloud.positions[500].x - 50.0) < 0.01, "PCD large: spot check position")
        expect(abs(cloud.intensities[500] - 0.5) < 0.01, "PCD large: spot check intensity")
    } catch {
        expect(false, "PCD large cloud failed: \(error)")
    }
}

func testPointCloud2LargeCloud() {
    var data = Data()

    for i in 0..<1000 {
        let x = Float(i) * 0.1
        let y = Float(i) * 0.2
        let z = Float(i) * 0.3
        let intensity = Float(i) / 1000.0

        data.append(contentsOf: withUnsafeBytes(of: x.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: y.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: z.bitPattern.littleEndian) { Array($0) })
        data.append(contentsOf: withUnsafeBytes(of: intensity.bitPattern.littleEndian) { Array($0) })
    }

    let layout = PointCloud2Layout.livoxMid360

    do {
        let cloud = try PointCloudDecoder.decodePointCloud2(from: data, fieldOffsets: layout)
        expect(cloud.count == 1000, "PC2 large: expected 1000 points, got \(cloud.count)")
        expect(abs(cloud.positions[500].x - 50.0) < 0.01, "PC2 large: spot check position")
        expect(abs(cloud.intensities[500] - 0.5) < 0.01, "PC2 large: spot check intensity")
    } catch {
        expect(false, "PC2 large cloud failed: \(error)")
    }
}

// MARK: - Spec-Matching Tests

func testPCDSpecFormat() {
    let pcd = """
    # .PCD v0.7 - Point Cloud Data
    VERSION 0.7
    FIELDS x y z intensity
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH 50000
    HEIGHT 1
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS 50000
    DATA ascii
    1.234 5.678 9.012 0.85
    """

    do {
        let cloud = try PointCloudDecoder.decodePCD(from: pcd)
        expect(cloud.count == 1, "PCD spec format: parsed spec example")
        expect(abs(cloud.positions[0].x - 1.234) < 0.001, "PCD spec format: x value")
        expect(abs(cloud.positions[0].y - 5.678) < 0.001, "PCD spec format: y value")
        expect(abs(cloud.positions[0].z - 9.012) < 0.001, "PCD spec format: z value")
        expect(abs(cloud.intensities[0] - 0.85) < 0.001, "PCD spec format: intensity value")
    } catch {
        expect(false, "PCD spec format failed: \(error)")
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
    testPCDMalformedPointData()
    print("  PCD ASCII: malformed lines skipped, valid lines kept")
    testPCDWithComments()
    print("  PCD ASCII: comments preserved in header")
    testPCDNaNValues()
    print("  PCD ASCII: NaN points skipped")
    testPCDUnsupportedBinaryFormat()
    print("  PCD ASCII: binary format → unsupported error")
    testPCDFromData()
    print("  PCD ASCII: parse from Data (not String)")
    testPCDLargeCloud()
    print("  PCD ASCII: 1000-point cloud parsed correctly")
    testPCDSpecFormat()
    print("  PCD ASCII: exact spec format from REDESIGN-PLAN.md")

    testPointCloud2BasicParsing()
    print("  PointCloud2: basic binary parsing")
    testPointCloud2EmptyData()
    print("  PointCloud2: empty data → empty cloud")
    testPointCloud2NoIntensity()
    print("  PointCloud2: no intensity → defaults to 1.0")
    testPointCloud2BigEndian()
    print("  PointCloud2: big-endian byte order")
    testPointCloud2WithPadding()
    print("  PointCloud2: stride with padding bytes")
    testPointCloud2NaNSkipped()
    print("  PointCloud2: NaN points skipped")
    testPointCloud2InvalidPointStep()
    print("  PointCloud2: invalid pointStep → error")
    testPointCloud2PresetLayouts()
    print("  PointCloud2: preset layouts for Mid-360 and JT128")
    testPointCloud2LargeCloud()
    print("  PointCloud2: 1000-point binary cloud")

    testPointCloudStruct()
    print("  PointCloud struct: isEmpty, count, sensorID")
}
