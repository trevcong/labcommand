import Foundation
import LabCommandCore

// MARK: - change-detect

private func emitError(_ text: String) -> Never {
    try? FileHandle.standardError.write(contentsOf: Data(text.utf8))
    exit(1)
}

private func usage() -> Never {
    emitError(
        """
        change-detect [--voxel-size m] [--max-range m] [--min-points n]
                      [--tolerance n] [--min-voxels n] [--max-move m]
                      [--no-pair] <before.pcd|before.bin> <after.pcd|after.bin>

        """
    )
}

private func fail(_ message: String) -> Never {
    emitError("change-detect: \(message)\n")
}

private func requireValue(_ args: [String], _ index: inout Int) -> String {
    guard index + 1 < args.count else { usage() }
    index += 1
    let value = args[index]
    if value.hasPrefix("-") { usage() }
    return value
}

private func parseConfig(_ args: [String]) -> (ChangeDetection.Config, String, String) {
    var config = ChangeDetection.Config()
    var paths: [String] = []
    var index = 0
    while index < args.count {
        let arg = args[index]
        switch arg {
        case "--voxel-size":
            let text = requireValue(args, &index)
            guard let value = Float(text) else { usage() }
            config.voxelSize = value
        case "--max-range":
            let text = requireValue(args, &index)
            guard let value = Float(text) else { usage() }
            config.maxRange = value
        case "--min-points":
            let text = requireValue(args, &index)
            guard let value = Int(text) else { usage() }
            config.minPoints = value
        case "--tolerance":
            let text = requireValue(args, &index)
            guard let value = Int(text) else { usage() }
            config.tolerance = value
        case "--min-voxels":
            let text = requireValue(args, &index)
            guard let value = Int(text) else { usage() }
            config.minVoxels = value
        case "--max-move":
            let text = requireValue(args, &index)
            guard let value = Float(text) else { usage() }
            config.maxMove = value
        case "--no-pair":
            config.pairMoves = false
        case "--":
            paths.append(contentsOf: args[(index + 1)...])
            index = args.count
        default:
            if arg.hasPrefix("-") { usage() }
            paths.append(arg)
        }
        index += 1
    }
    guard paths.count == 2 else { usage() }
    return (config, paths[0], paths[1])
}

private func loadCloud(_ path: String) throws -> PointCloudFrame {
    let ext = URL(fileURLWithPath: path).pathExtension.lowercased()
    switch ext {
    case "pcd":
        return try PointCloudDecoder.loadPCD(from: path)
    case "bin":
        let data = try Data(contentsOf: URL(fileURLWithPath: path))
        return try PointCloud2CDR.decode(data)
    default:
        fail("expected a .pcd or .bin file, got '\(ext.isEmpty ? path : "." + ext)'")
    }
}

private func milliseconds(_ duration: Duration) -> Double {
    let parts = duration.components
    return Double(parts.seconds) * 1000 + Double(parts.attoseconds) / 1e15
}

private func format(_ value: Float, _ digits: Int) -> String {
    String(format: "%.\(digits)f", value)
}

private func format(_ point: Point3D) -> String {
    "\(format(point.x, 3)),\(format(point.y, 3)),\(format(point.z, 3))"
}

private func format(min: Point3D, max: Point3D) -> String {
    "[\(format(min)) .. \(format(max))]"
}

private func describe(_ error: Error) -> String {
    switch error {
    case let failure as ChangeDetection.Failure:
        switch failure {
        case .invalidConfig(let reason):
            return reason
        case .voxelSizeMismatch(let before, let after):
            return "voxel size mismatch \(before) m vs \(after) m"
        }
    case let failure as PointCloudError:
        return String(describing: failure)
    default:
        return String(describing: error)
    }
}

let arguments = Array(CommandLine.arguments.dropFirst())
let (config, beforePath, afterPath) = parseConfig(arguments)

let beforeFrame: PointCloudFrame
let afterFrame: PointCloudFrame
do {
    beforeFrame = try loadCloud(beforePath)
    afterFrame = try loadCloud(afterPath)
} catch {
    fail(describe(error))
}

let beforeMap: ChangeDetection.OccupancyMap
let afterMap: ChangeDetection.OccupancyMap
let report: ChangeDetection.Report
let elapsed: Double
do {
    let clock = ContinuousClock()
    let start = clock.now
    beforeMap = try ChangeDetection.occupancyMap(beforeFrame, config: config)
    afterMap = try ChangeDetection.occupancyMap(afterFrame, config: config)
    report = try ChangeDetection.diff(beforeMap, afterMap, config: config)
    elapsed = milliseconds(start.duration(to: clock.now))
} catch {
    fail(describe(error))
}

let beforeName = URL(fileURLWithPath: beforePath).lastPathComponent
let afterName = URL(fileURLWithPath: afterPath).lastPathComponent
print("before: \(beforeName)  points=\(beforeFrame.count)  voxels=\(beforeMap.occupied.count)  dropped=\(beforeMap.droppedPoints)")
print("after:  \(afterName)  points=\(afterFrame.count)  voxels=\(afterMap.occupied.count)  dropped=\(afterMap.droppedPoints)")
print(
    "voxel_size_m=\(format(config.voxelSize, 3))  tolerance=\(config.tolerance)  min_points=\(config.minPoints)  min_voxels=\(config.minVoxels)  pair_moves=\(config.pairMoves)"
)
print("appeared_voxels=\(report.appearedVoxels)  disappeared_voxels=\(report.disappearedVoxels)")
print("elapsed_ms=\(format(Float(elapsed), 1))")
print("regions=\(report.regions.count)")
for (index, region) in report.regions.enumerated() {
    var line = "region \(index) \(region.kind.rawValue) voxels=\(region.voxelCount) volume_m3=\(format(region.volume, 6)) centroid=\(format(region.centroid)) bounds=\(format(min: region.boundsMin, max: region.boundsMax))"
    if let fromCount = region.fromVoxelCount,
       let fromCentroid = region.fromCentroid,
       let fromMin = region.fromBoundsMin,
       let fromMax = region.fromBoundsMax {
        line += " from_voxels=\(fromCount) from_centroid=\(format(fromCentroid)) from_bounds=\(format(min: fromMin, max: fromMax))"
    }
    print(line)
}
