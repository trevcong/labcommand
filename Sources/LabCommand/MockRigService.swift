import Foundation
import LabCommandCore

/// Loads fixture data for development and testing without Spark access.
/// Use `--mock <fixture>` launch argument to activate.
///
/// Example: `swift run LabCommand --mock both-sensors-live`
///
/// Available fixtures:
/// - both-sensors-live: Both sensors streaming, clocks locked
/// - both-sensors-dark: Sensors unpowered (parked state)
/// - clock-converging: Mid-360 slewing, not yet locked
/// - clock-locked: Clocks synchronized, ready to record
/// - recording-active: Recording in progress
/// - spark-offline: Spark unreachable
public final class MockRigService: @unchecked Sendable {
    public static let apiPort = RigService.apiPort
    public static let viewPort = RigService.viewPort
    
    private let fixtureName: String
    private let fixtureJSON: String?
    
    public init(fixture: String) {
        self.fixtureName = fixture
        self.fixtureJSON = Self.loadFixtureJSON(named: fixture)
        
        if fixtureJSON == nil {
            print("[MockRigService] Warning: fixture '\(fixture)' not found")
            print("[MockRigService] Available fixtures: both-sensors-live, both-sensors-dark, clock-converging, clock-locked, recording-active, spark-offline")
        } else {
            print("[MockRigService] Loaded fixture: \(fixture)")
        }
    }
    
    private static func loadFixtureJSON(named name: String) -> String? {
        let fm = FileManager.default
        
        // Try bundle resources first (for .app bundles)
        if let resourcePath = Bundle.main.resourcePath {
            let bundlePath = URL(fileURLWithPath: resourcePath)
                .appendingPathComponent("fixtures/api/\(name).json")
            if let data = fm.contents(atPath: bundlePath.path),
               let json = String(data: data, encoding: .utf8) {
                return json
            }
        }
        
        // Try Bundle.main.path for embedded resources
        if let bundlePath = Bundle.main.path(forResource: name, ofType: "json", inDirectory: "fixtures/api") {
            if let data = fm.contents(atPath: bundlePath),
               let json = String(data: data, encoding: .utf8) {
                return json
            }
        }
        
        // Fallback to relative paths from cwd (for swift run / development)
        let cwd = fm.currentDirectoryPath
        let searchPaths = [
            "fixtures/api/\(name).json",
            "../fixtures/api/\(name).json",
            "../../fixtures/api/\(name).json",
        ]
        
        for relativePath in searchPaths {
            let url = URL(fileURLWithPath: cwd).appendingPathComponent(relativePath)
            if let data = fm.contents(atPath: url.path),
               let json = String(data: data, encoding: .utf8) {
                return json
            }
        }
        
        return nil
    }
    
    /// Whether the view tunnel should be up for this fixture.
    /// Dark sensors typically have no active view tunnel.
    private var fixtureViewTunnelUp: Bool {
        switch fixtureName {
        case "spark-offline":
            return false
        case "both-sensors-dark":
            return false
        default:
            return true
        }
    }
    
    public var isViewTunnelRunning: Bool { fixtureViewTunnelUp }
    public var isAPITunnelRunning: Bool { fixtureName != "spark-offline" }
    public var viewPathUp: Bool { fixtureViewTunnelUp }
    
    public func ensureTunnels() -> String? {
        if fixtureName == "spark-offline" {
            return "mock: Spark not reachable"
        }
        return nil
    }
    
    public func ensureAPITunnel() -> String? {
        if fixtureName == "spark-offline" {
            return "mock: Spark not reachable"
        }
        return nil
    }
    
    public func stopTunnels() {
    }
    
    public func fetchStatus(sparkWasReachable: Bool) -> RigSnapshot {
        if fixtureName == "spark-offline" {
            return RigSnapshot.offline(reason: "mock: Spark not reachable — check Tailscale")
        }
        
        guard let json = fixtureJSON else {
            return RigSnapshot.apiDown(reason: "mock: fixture '\(fixtureName)' not loaded")
        }
        
        guard let snapshot = RigStatusDecoder.parse(json: json, viewTunnelUp: fixtureViewTunnelUp) else {
            return RigSnapshot.apiDown(reason: "mock: could not parse fixture '\(fixtureName)'")
        }
        
        return snapshot
    }
    
    public func postAction(_ name: String, arg: String? = nil,
                           extra: [String: Any] = [:]) -> (ok: Bool, message: String) {
        print("[MockRigService] Action: \(name) arg=\(arg ?? "nil") extra=\(extra)")
        return (true, "mock: \(name) ok")
    }
}

/// Parses command line arguments for mock mode.
public enum MockMode {
    /// Returns the fixture name if `--mock <fixture>` was passed, nil otherwise.
    public static func parseArgs() -> String? {
        let args = CommandLine.arguments
        
        guard let idx = args.firstIndex(of: "--mock") else {
            return nil
        }
        
        let fixtureIdx = idx + 1
        guard fixtureIdx < args.count else {
            print("[MockMode] --mock requires a fixture name")
            print("[MockMode] Example: --mock both-sensors-live")
            return nil
        }
        
        return args[fixtureIdx]
    }
    
    /// Creates the appropriate rig service based on command line arguments.
    /// Returns a `RigService` for normal operation or `MockRigService` when `--mock` is passed.
    public static func createRigService() -> RigServiceProtocol {
        if let fixture = parseArgs() {
            return MockRigService(fixture: fixture)
        }
        return RigService()
    }
}

/// Protocol for rig I/O services, allowing dependency injection of mock or real implementations.
public protocol RigServiceProtocol: Sendable {
    var isViewTunnelRunning: Bool { get }
    var isAPITunnelRunning: Bool { get }
    var viewPathUp: Bool { get }
    
    func ensureTunnels() -> String?
    func ensureAPITunnel() -> String?
    func stopTunnels()
    func fetchStatus(sparkWasReachable: Bool) -> RigSnapshot
    func postAction(_ name: String, arg: String?, extra: [String: Any]) -> (ok: Bool, message: String)
}

public extension RigServiceProtocol {
    func postAction(_ name: String) -> (ok: Bool, message: String) {
        postAction(name, arg: nil, extra: [:])
    }
    
    func postAction(_ name: String, arg: String?) -> (ok: Bool, message: String) {
        postAction(name, arg: arg, extra: [:])
    }
}

extension RigService: RigServiceProtocol {}
extension MockRigService: RigServiceProtocol {}
