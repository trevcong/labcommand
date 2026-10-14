import Foundation

/// How this process was launched. A partial snapshot pair is an error, not a rig launch.
public enum LaunchMode: Equatable, Sendable {
    case rig
    case snapshots(directoryA: String, directoryB: String)
    case configurationError(String)

    public static let variableA = "LC_SNAPSHOT_A"
    public static let variableB = "LC_SNAPSHOT_B"

    public var isolates: Bool {
        if case .rig = self { return false }
        return true
    }

    public static func resolve(_ environment: [String: String]) -> LaunchMode {
        let aRaw = environment[variableA]
        let bRaw = environment[variableB]
        if aRaw == nil, bRaw == nil { return .rig }
        let a = aRaw?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        let b = bRaw?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        if !a.isEmpty, !b.isEmpty {
            return .snapshots(directoryA: a, directoryB: b)
        }
        return .configurationError(configurationMessage(aRaw: aRaw, bRaw: bRaw))
    }

    public static func configurationMessage(aRaw: String?, bRaw: String?) -> String {
        let a = aRaw?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        let b = bRaw?.trimmingCharacters(in: .whitespacesAndNewlines) ?? ""
        if aRaw != nil, bRaw != nil, a.isEmpty, b.isEmpty {
            return "Snapshot configuration error. LC_SNAPSHOT_A and LC_SNAPSHOT_B are empty."
        }
        var parts: [String] = []
        if aRaw == nil {
            parts.append("LC_SNAPSHOT_A is not set")
        } else if a.isEmpty {
            parts.append("LC_SNAPSHOT_A is empty")
        }
        if bRaw == nil {
            parts.append("LC_SNAPSHOT_B is not set")
        } else if b.isEmpty {
            parts.append("LC_SNAPSHOT_B is empty")
        }
        return "Snapshot configuration error. \(parts.joined(separator: " and "))."
    }
}

public enum ContactKind: String, Sendable, Equatable {
    case process
    case socket
    case poll
    case action
    case timer
}

public struct ContactForbidden: Error, Equatable, Sendable {
    public var kind: ContactKind
    public var detail: String

    public init(kind: ContactKind, detail: String) {
        self.kind = kind
        self.detail = detail
    }
}

public struct ProcessSpawn: Equatable, Sendable {
    public var executable: String
    public var arguments: [String]

    public init(executable: String, arguments: [String]) {
        self.executable = executable
        self.arguments = arguments
    }
}

public struct SocketOpen: Equatable, Sendable {
    public var endpoint: String

    public init(endpoint: String) {
        self.endpoint = endpoint
    }
}

public struct ContactProcessResult: Equatable, Sendable {
    public var exitCode: Int32
    public var stdout: String
    public var stderr: String
    /// Set when the process never started. A finished process leaves this nil.
    public var spawnError: String?

    public init(exitCode: Int32, stdout: String, stderr: String, spawnError: String? = nil) {
        self.exitCode = exitCode
        self.stdout = stdout
        self.stderr = stderr
        self.spawnError = spawnError
    }
}

/// One branch in front of a process, a socket, a poll, a timer, or an action.
public enum ContactGate {
    public static func attempt<T>(
        _ kind: ContactKind,
        isolates: Bool,
        detail: String,
        _ work: () throws -> T
    ) throws -> T {
        if isolates {
            throw ContactForbidden(kind: kind, detail: detail)
        }
        return try work()
    }

    public static func runProcess(
        _ spawn: ProcessSpawn,
        isolates: Bool,
        _ factory: () throws -> ContactProcessResult
    ) throws -> ContactProcessResult {
        let detail = ([spawn.executable] + spawn.arguments).joined(separator: " ")
        return try attempt(.process, isolates: isolates, detail: detail, factory)
    }

    public static func openSocket(
        _ open: SocketOpen,
        isolates: Bool,
        _ factory: () throws -> Void
    ) throws {
        try attempt(.socket, isolates: isolates, detail: open.endpoint, factory)
    }
}

public struct TunnelSpawnOutcome: Equatable, Sendable {
    public var stayedUp: Bool
    public var error: String?

    public init(stayedUp: Bool, error: String? = nil) {
        self.stayedUp = stayedUp
        self.error = error
    }
}

/// Shared path for `ensureTunnels` and `ensureAPITunnel`.
/// An isolated call returns before discovery and before any factory.
public enum TunnelGate {
    public static let apiPort = 8766
    public static let viewPort = CaptureRecipe.foxglovePort

    public struct Factories {
        public var held: (Int) -> Bool
        public var portOpen: (Int) -> Bool
        public var hosts: () -> [String]
        public var spawn: (_ port: Int, _ command: ProcessSpawn) -> TunnelSpawnOutcome
        public var rememberHost: (String) -> Void

        public init(
            held: @escaping (Int) -> Bool,
            portOpen: @escaping (Int) -> Bool,
            hosts: @escaping () -> [String],
            spawn: @escaping (_ port: Int, _ command: ProcessSpawn) -> TunnelSpawnOutcome,
            rememberHost: @escaping (String) -> Void
        ) {
            self.held = held
            self.portOpen = portOpen
            self.hosts = hosts
            self.spawn = spawn
            self.rememberHost = rememberHost
        }
    }

    /// One check-then-spawn at a time. Two callers otherwise both see a closed port and both spawn.
    private static let lock = NSLock()

    public static func ensureTunnels(isolates: Bool, factories: Factories) -> String? {
        let apiErr = open(port: apiPort, isolates: isolates, factories: factories)
        let viewErr = open(port: viewPort, isolates: isolates, factories: factories)
        return apiErr ?? viewErr
    }

    public static func ensureAPITunnel(isolates: Bool, factories: Factories) -> String? {
        open(port: apiPort, isolates: isolates, factories: factories)
    }

    private static func open(port: Int, isolates: Bool, factories: Factories) -> String? {
        lock.lock()
        defer { lock.unlock() }
        do {
            return try ContactGate.attempt(.process, isolates: isolates, detail: "tunnel \(port)") {
                var lastError: String?
                if factories.held(port) || factories.portOpen(port) { return nil }
                for host in factories.hosts() {
                    let command = ProcessSpawn(
                        executable: "/usr/bin/ssh",
                        arguments: CaptureRecipe.viewTunnelArguments(host: host, port: port)
                    )
                    let outcome = factories.spawn(port, command)
                    if outcome.stayedUp {
                        if host.contains(".") { factories.rememberHost(host) }
                        return nil
                    }
                    lastError = outcome.error ?? "tunnel to \(host) exited"
                }
                return lastError ?? "tunnel failed"
            }
        } catch {
            return "local session"
        }
    }
}

/// The calls `AppState` makes. Snapshot mode does not enter the work closures.
public struct SnapshotSession: Equatable, Sendable {
    public let mode: LaunchMode

    public init(environment: [String: String]) {
        mode = LaunchMode.resolve(environment)
    }

    public var isolates: Bool { mode.isolates }

    public enum Begin: Equatable, Sendable {
        case rig
        case failed(String)
        case load(directoryA: String, directoryB: String)
    }

    public func begin(rig: () -> Void) -> Begin {
        switch mode {
        case .rig:
            rig()
            return .rig
        case .configurationError(let message):
            return .failed(message)
        case .snapshots(let directoryA, let directoryB):
            return .load(directoryA: directoryA, directoryB: directoryB)
        }
    }

    public func refresh(manual: Bool, _ work: () -> Void) {
        perform(.process, detail: manual ? "refresh manual" : "refresh", work)
    }

    public func restartTimer(_ work: () -> Void) {
        perform(.timer, detail: "restartTimer", work)
    }

    public func pollRig(_ work: () -> Void) {
        perform(.poll, detail: "pollRig", work)
    }

    public func runAction(_ name: String, _ work: () -> Void) {
        perform(.action, detail: name, work)
    }

    private func perform(_ kind: ContactKind, detail: String, _ work: () -> Void) {
        do {
            try ContactGate.attempt(kind, isolates: isolates, detail: detail, work)
        } catch {
            return
        }
    }
}

public struct SnapshotClouds: Sendable {
    public var timeline: ChangeOverlay.Timeline
    public var mid: PointCloudFrame?
    public var jt: PointCloudFrame?
    public var caption: String
}

public enum SnapshotLoad: Sendable {
    case ready(SnapshotClouds)
    case failure(String)

    public var caption: String {
        switch self {
        case .ready(let clouds): return clouds.caption
        case .failure(let message): return message
        }
    }
}

/// Local files only. No process and no socket.
public enum SnapshotFiles {
    public static func fileName(_ sensor: SensorID) -> String {
        switch sensor {
        case .mid360: return "mid360.bin"
        case .jt128: return "jt128.bin"
        }
    }

    public static func load(directoryA: String, directoryB: String) -> SnapshotLoad {
        var parts: [ChangeOverlay.Timeline] = []
        var mid: PointCloudFrame?
        var jt: PointCloudFrame?
        let config = ChangeOverlay.snapshotConfig()
        for sensor in SensorID.allCases {
            let name = fileName(sensor)
            do {
                let before = try decode(directory: directoryA, sensor: sensor, file: name)
                let after = try decode(directory: directoryB, sensor: sensor, file: name)
                let report = try ChangeDetection.compare(before, after, config: config)
                parts.append(ChangeOverlay.timeline(
                    sensor: sensor,
                    report: report,
                    beforeStamp: before.timestamp,
                    afterStamp: after.timestamp
                ))
                let world = RigTransforms.inWorld(after, sensor: sensor)
                switch sensor {
                case .mid360: mid = world
                case .jt128: jt = world
                }
            } catch let problem as LoadProblem {
                return .failure(problem.caption)
            } catch {
                return .failure(ChangeOverlay.loadFailure(sensor: sensor, file: name, reason: describe(error)))
            }
        }
        let timeline = ChangeOverlay.merge(parts)
        let filter = ChangeOverlay.Filter()
        let count = ChangeOverlay.filtered(timeline, filter).count
        return .ready(SnapshotClouds(
            timeline: timeline,
            mid: mid,
            jt: jt,
            caption: ChangeOverlay.caption(timeline: timeline, filter: filter, count: count)
        ))
    }

    private struct LoadProblem: Error {
        var caption: String
    }

    private static func decode(directory: String, sensor: SensorID, file: String) throws -> PointCloudFrame {
        let url = URL(fileURLWithPath: directory).appendingPathComponent(file)
        guard FileManager.default.fileExists(atPath: url.path) else {
            throw LoadProblem(caption: ChangeOverlay.loadFailure(sensor: sensor, file: file, reason: "missing file"))
        }
        let data: Data
        do {
            data = try Data(contentsOf: url)
        } catch {
            throw LoadProblem(caption: ChangeOverlay.loadFailure(sensor: sensor, file: file, reason: "missing file"))
        }
        do {
            var frame = try PointCloud2CDR.decode(data)
            frame.sensor = sensor
            return frame
        } catch {
            throw LoadProblem(caption: ChangeOverlay.loadFailure(sensor: sensor, file: file, reason: describe(error)))
        }
    }

    private static func describe(_ error: Error) -> String {
        if let error = error as? PointCloudError {
            switch error {
            case .invalidHeader(let text), .invalidData(let text), .unsupportedFormat(let text), .fileNotFound(let text):
                return text.isEmpty ? "unreadable" : text
            }
        }
        if let error = error as? ChangeDetection.Failure {
            switch error {
            case .invalidConfig(let text):
                return text
            case .voxelSizeMismatch(let left, let right):
                return "voxel size \(left) vs \(right)"
            }
        }
        return "unreadable"
    }
}
