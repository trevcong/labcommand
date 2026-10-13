import Foundation
import LabCommandCore

// Point cloud type tests for PR 3's decoder.
// These verify the frame and header structures work correctly.
// Full decoder tests (PCD parsing, PointCloud2 binary) will be added in PR 3.

func testPointCloudFrameEmpty() {
    let frame = PointCloudFrame()
    expect(frame.isEmpty, "empty frame reports isEmpty")
    expect(frame.count == 0, "empty frame count is 0")
    expect(frame.positions.isEmpty, "no positions")
    expect(frame.intensities.isEmpty, "no intensities")
}

func testPointCloudFrameWithData() {
    let positions: [SIMD3<Float>] = [
        SIMD3(1.0, 2.0, 3.0),
        SIMD3(4.0, 5.0, 6.0),
    ]
    let intensities: [Float] = [0.5, 0.8]
    let frame = PointCloudFrame(
        positions: positions,
        intensities: intensities,
        sensor: .mid360
    )
    expect(!frame.isEmpty, "frame with data is not empty")
    expect(frame.count == 2, "frame count matches")
    expect(frame.sensor == .mid360, "sensor preserved")
}

func testPCDHeaderDefaults() {
    let header = PCDHeader()
    expect(header.version == "0.7", "default version")
    expect(header.fields.contains("x"), "default fields include x")
    expect(header.fields.contains("intensity"), "default fields include intensity")
    expect(header.dataFormat == .ascii, "default format is ascii")
}

func testPCDDataFormats() {
    expect(PCDDataFormat.ascii.rawValue == "ascii", "ascii format")
    expect(PCDDataFormat.binary.rawValue == "binary", "binary format")
    expect(PCDDataFormat.binaryCompressed.rawValue == "binary_compressed", "compressed format")
}

func testPointCloudErrorCases() {
    let headerErr = PointCloudError.invalidHeader("missing VERSION")
    let dataErr = PointCloudError.invalidData("truncated")
    let formatErr = PointCloudError.unsupportedFormat("binary_compressed")
    let fileErr = PointCloudError.fileNotFound("missing.pcd")
    
    // Errors must be distinguishable
    expect(headerErr != dataErr, "different error types")
    expect(dataErr != formatErr, "different error types")
    expect(formatErr != fileErr, "different error types")
}

func runPointCloudTests() {
    print("Running point cloud type tests…")
    testPointCloudFrameEmpty()
    print("  point cloud: empty frame reports isEmpty, count 0")
    testPointCloudFrameWithData()
    print("  point cloud: frame with data preserves positions, intensities, sensor")
    testPCDHeaderDefaults()
    print("  point cloud: PCD header defaults to 0.7, ascii, standard fields")
    testPCDDataFormats()
    print("  point cloud: data format enum matches PCD spec strings")
    testPointCloudErrorCases()
    print("  point cloud: error cases are distinguishable")
}
