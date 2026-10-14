// swift-tools-version: 5.9
import PackageDescription

let package = Package(
    name: "LabCommand",
    platforms: [.macOS(.v14)],
    products: [
        .executable(name: "LabCommand", targets: ["LabCommand"]),
        .executable(name: "change-detect", targets: ["ChangeDetect"]),
        .library(name: "LabCommandCore", targets: ["LabCommandCore"]),
    ],
    targets: [
        .target(
            name: "LabCommandCore",
            path: "Sources/LabCommandCore"
        ),
        .executableTarget(
            name: "LabCommand",
            dependencies: ["LabCommandCore"],
            path: "Sources/LabCommand"
        ),
        .executableTarget(
            name: "ChangeDetect",
            dependencies: ["LabCommandCore"],
            path: "Sources/ChangeDetect"
        ),
        // XCTest may be unavailable under CLT-only installs; assert-based runner.
        .executableTarget(
            name: "LabCommandCoreTestsRunner",
            dependencies: ["LabCommandCore"],
            path: "Sources/LabCommandCoreTestsRunner"
        ),
    ]
)
