import Foundation
import LabCommandCore

// Point cloud fixture tests. Verifies PCD files exist and have valid format.
// Full decoder tests will be added in PR 3 (decoder).

private func pcdFixturesPath() -> String? {
    let fm = FileManager.default
    let searchPaths = [
        "fixtures/pointclouds",
        "../fixtures/pointclouds",
        "../../fixtures/pointclouds",
    ]
    let cwd = fm.currentDirectoryPath
    for rel in searchPaths {
        let path = URL(fileURLWithPath: cwd).appendingPathComponent(rel).path
        if fm.fileExists(atPath: path) {
            return path
        }
    }
    return nil
}

private func loadPCDFile(_ name: String) -> String? {
    guard let base = pcdFixturesPath() else { return nil }
    let path = URL(fileURLWithPath: base).appendingPathComponent(name).path
    guard let data = FileManager.default.contents(atPath: path) else { return nil }
    return String(data: data, encoding: .utf8)
}

/// Parse a PCD header and return (version, fields, width, height, points, dataFormat).
/// Returns nil if the header is invalid.
private func parsePCDHeader(_ content: String) -> (version: String, fields: [String], width: Int, height: Int, points: Int, dataFormat: String)? {
    var version = ""
    var fields: [String] = []
    var width = 0
    var height = 1
    var points = 0
    var dataFormat = ""
    var foundData = false
    
    for line in content.components(separatedBy: .newlines) {
        let trimmed = line.trimmingCharacters(in: .whitespaces)
        if trimmed.isEmpty || trimmed.hasPrefix("#") { continue }
        
        let parts = trimmed.split(separator: " ", maxSplits: 1)
        guard parts.count >= 1 else { continue }
        
        let key = String(parts[0]).uppercased()
        let value = parts.count > 1 ? String(parts[1]) : ""
        
        switch key {
        case "VERSION":
            version = value
        case "FIELDS":
            fields = value.split(separator: " ").map(String.init)
        case "WIDTH":
            width = Int(value) ?? 0
        case "HEIGHT":
            height = Int(value) ?? 1
        case "POINTS":
            points = Int(value) ?? 0
        case "DATA":
            dataFormat = value.lowercased()
            foundData = true
        default:
            break
        }
        
        if foundData { break }
    }
    
    guard !version.isEmpty, !fields.isEmpty, foundData else { return nil }
    return (version, fields, width, height, points, dataFormat)
}

func testPCDFixturesExist() {
    guard let path = pcdFixturesPath() else {
        fputs("FAIL: fixtures/pointclouds directory not found\n", stderr)
        exit(1)
    }
    
    let expectedFiles = [
        "mid360-frame.pcd",
        "jt128-frame.pcd",
        "dual-frame.pcd",
        "empty.pcd",
    ]
    
    let fm = FileManager.default
    for file in expectedFiles {
        let filePath = URL(fileURLWithPath: path).appendingPathComponent(file).path
        expect(fm.fileExists(atPath: filePath), "PCD fixture \(file) must exist")
    }
}

func testPCDMid360FrameValid() {
    guard let content = loadPCDFile("mid360-frame.pcd") else {
        fputs("FAIL: mid360-frame.pcd not found or not readable\n", stderr)
        exit(1)
    }
    
    guard let header = parsePCDHeader(content) else {
        fputs("FAIL: mid360-frame.pcd has invalid header\n", stderr)
        exit(1)
    }
    
    expect(header.version == "0.7", "mid360 PCD version is 0.7")
    expect(header.fields.contains("x") && header.fields.contains("y") && header.fields.contains("z"),
           "mid360 PCD has x y z fields")
    expect(header.fields.contains("intensity"), "mid360 PCD has intensity field")
    expect(header.points == 50000, "mid360 PCD has 50000 points, got \(header.points)")
    expect(header.dataFormat == "ascii", "mid360 PCD is ascii format")
    
    // Verify we have actual data lines
    let lines = content.components(separatedBy: .newlines)
    let dataLines = lines.dropFirst(while: { !$0.hasPrefix("DATA") }).dropFirst()
    let nonEmptyDataLines = dataLines.filter { !$0.trimmingCharacters(in: .whitespaces).isEmpty }
    expect(nonEmptyDataLines.count == 50000, "mid360 PCD has 50000 data lines, got \(nonEmptyDataLines.count)")
}

func testPCDJT128FrameValid() {
    guard let content = loadPCDFile("jt128-frame.pcd") else {
        fputs("FAIL: jt128-frame.pcd not found or not readable\n", stderr)
        exit(1)
    }
    
    guard let header = parsePCDHeader(content) else {
        fputs("FAIL: jt128-frame.pcd has invalid header\n", stderr)
        exit(1)
    }
    
    expect(header.version == "0.7", "jt128 PCD version is 0.7")
    expect(header.points == 50000, "jt128 PCD has 50000 points, got \(header.points)")
    expect(header.dataFormat == "ascii", "jt128 PCD is ascii format")
}

func testPCDDualFrameValid() {
    guard let content = loadPCDFile("dual-frame.pcd") else {
        fputs("FAIL: dual-frame.pcd not found or not readable\n", stderr)
        exit(1)
    }
    
    guard let header = parsePCDHeader(content) else {
        fputs("FAIL: dual-frame.pcd has invalid header\n", stderr)
        exit(1)
    }
    
    expect(header.points == 100000, "dual PCD has 100000 points (50k + 50k), got \(header.points)")
}

func testPCDEmptyFrameValid() {
    guard let content = loadPCDFile("empty.pcd") else {
        fputs("FAIL: empty.pcd not found or not readable\n", stderr)
        exit(1)
    }
    
    guard let header = parsePCDHeader(content) else {
        fputs("FAIL: empty.pcd has invalid header\n", stderr)
        exit(1)
    }
    
    expect(header.points == 0, "empty PCD has 0 points")
}

func testPCDDataLinesParseable() {
    guard let content = loadPCDFile("mid360-frame.pcd") else {
        fputs("FAIL: mid360-frame.pcd not found\n", stderr)
        exit(1)
    }
    
    let lines = content.components(separatedBy: .newlines)
    var inData = false
    var parsedCount = 0
    var parseErrors = 0
    
    for line in lines {
        if line.hasPrefix("DATA") {
            inData = true
            continue
        }
        if !inData { continue }
        
        let trimmed = line.trimmingCharacters(in: .whitespaces)
        if trimmed.isEmpty { continue }
        
        let parts = trimmed.split(separator: " ")
        if parts.count >= 4 {
            // Try to parse x y z intensity
            if let _ = Float(parts[0]),
               let _ = Float(parts[1]),
               let _ = Float(parts[2]),
               let _ = Float(parts[3]) {
                parsedCount += 1
            } else {
                parseErrors += 1
            }
        } else {
            parseErrors += 1
        }
        
        // Only check first 100 lines for speed
        if parsedCount + parseErrors >= 100 { break }
    }
    
    expect(parseErrors == 0, "PCD data lines are parseable as floats, errors: \(parseErrors)")
    expect(parsedCount == 100, "parsed 100 sample lines, got \(parsedCount)")
}

func runPointCloudTests() {
    print("Running point cloud fixture tests…")
    testPCDFixturesExist()
    print("  point cloud: all PCD fixture files exist")
    testPCDMid360FrameValid()
    print("  point cloud: mid360-frame.pcd has valid header, 50k points, ascii format")
    testPCDJT128FrameValid()
    print("  point cloud: jt128-frame.pcd has valid header, 50k points")
    testPCDDualFrameValid()
    print("  point cloud: dual-frame.pcd has 100k points (both sensors)")
    testPCDEmptyFrameValid()
    print("  point cloud: empty.pcd has 0 points")
    testPCDDataLinesParseable()
    print("  point cloud: PCD data lines are valid x y z intensity floats")
}
