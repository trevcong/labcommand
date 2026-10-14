import Foundation
import LabCommandCore

/// I/O for the dual rig: own the two SSH tunnels (8765 Foxglove, 8766
/// command-center API) and speak HTTP to the API. Replaces the v0.5
/// spark-capture.sh probe path — the Spark-side collector is the one truth,
/// shared with the web panel.
public final class RigService: @unchecked Sendable {
    public static let apiPort = TunnelGate.apiPort
    public static let viewPort = TunnelGate.viewPort

    private let lock = NSLock()
    private var viewTunnel: Process?
    private var apiTunnel: Process?
    private let session: URLSession
    /// Snapshot launches refuse SSH, the port probe, and the localhost API.
    private let isolates: Bool

    public init(isolates: Bool = false) {
        self.isolates = isolates
        let cfg = URLSessionConfiguration.ephemeral
        cfg.timeoutIntervalForRequest = 4
        cfg.timeoutIntervalForResource = 6
        session = URLSession(configuration: cfg)
    }

    // MARK: Tunnels

    public var isViewTunnelRunning: Bool { tunnelRunning(\.viewTunnel) }
    public var isAPITunnelRunning: Bool { tunnelRunning(\.apiTunnel) }

    private func tunnelRunning(_ key: ReferenceWritableKeyPath<RigService, Process?>) -> Bool {
        lock.lock()
        defer { lock.unlock() }
        guard let p = self[keyPath: key] else { return false }
        return p.isRunning
    }

    private func hostOrder() -> [String] {
        let ts = runProcess("/usr/bin/env", args: ["tailscale", "status", "--json"])
        return SparkSSH.hopList(
            tailscaleJSON: ts.exitCode == 0 ? ts.stdout : nil,
            lastLAN: SparkHopCache.load()
        )
    }

    /// Another process (the lidar CLI, a stray ssh) may already forward the
    /// port; that tunnel is as good as ours.
    private func portOpen(_ port: Int) -> Bool {
        do {
            return try ContactGate.attempt(.socket, isolates: isolates, detail: "127.0.0.1:\(port)") {
                self.portOpenLive(port)
            }
        } catch {
            return false
        }
    }

    private func portOpenLive(_ port: Int) -> Bool {
        var addr = sockaddr_in()
        addr.sin_family = sa_family_t(AF_INET)
        addr.sin_port = in_port_t(UInt16(port).bigEndian)
        addr.sin_addr.s_addr = inet_addr("127.0.0.1")
        let fd = socket(AF_INET, SOCK_STREAM, 0)
        guard fd >= 0 else { return false }
        defer { close(fd) }
        var tv = timeval(tv_sec: 0, tv_usec: 300_000)
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, socklen_t(MemoryLayout<timeval>.size))
        let r = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                connect(fd, $0, socklen_t(MemoryLayout<sockaddr_in>.size))
            }
        }
        return r == 0
    }

    /// Both tunnels. Returns the first error, or nil.
    public func ensureTunnels() -> String? {
        TunnelGate.ensureTunnels(isolates: isolates, factories: tunnelFactories())
    }

    public func ensureAPITunnel() -> String? {
        TunnelGate.ensureAPITunnel(isolates: isolates, factories: tunnelFactories())
    }

    private func tunnelFactories() -> TunnelGate.Factories {
        TunnelGate.Factories(
            held: { port in
                self.tunnelRunning(port == Self.apiPort ? \.apiTunnel : \.viewTunnel)
            },
            portOpen: { self.portOpen($0) },
            hosts: { self.hostOrder() },
            spawn: { port, command in
                self.launchTunnel(port: port, command: command)
            },
            rememberHost: { SparkHopCache.save($0) }
        )
    }

    private func launchTunnel(port: Int, command: ProcessSpawn) -> TunnelSpawnOutcome {
        if isolates {
            return TunnelSpawnOutcome(stayedUp: false, error: "local session")
        }
        let proc = Process()
        proc.executableURL = URL(fileURLWithPath: command.executable)
        proc.arguments = command.arguments
        proc.standardOutput = Pipe()
        proc.standardError = Pipe()
        do {
            try proc.run()
        } catch {
            return TunnelSpawnOutcome(stayedUp: false, error: error.localizedDescription)
        }
        Thread.sleep(forTimeInterval: 0.25)
        guard proc.isRunning else {
            return TunnelSpawnOutcome(stayedUp: false)
        }
        let key: ReferenceWritableKeyPath<RigService, Process?> =
            port == Self.apiPort ? \.apiTunnel : \.viewTunnel
        lock.lock()
        self[keyPath: key] = proc
        lock.unlock()
        return TunnelSpawnOutcome(stayedUp: true)
    }

    public func stopTunnels() {
        do {
            try ContactGate.attempt(.process, isolates: isolates, detail: "stop tunnels") {
                self.stopTunnelProcesses()
            }
        } catch {
            return
        }
    }

    private func stopTunnelProcesses() {
        lock.lock()
        let procs = [viewTunnel, apiTunnel]
        viewTunnel = nil
        apiTunnel = nil
        lock.unlock()
        procs.forEach { $0?.terminate() }
    }

    /// True when either our child process or any local forwarder serves 8765.
    public var viewPathUp: Bool { isViewTunnelRunning || portOpen(Self.viewPort) }

    // MARK: API

    /// GET status.json. Failure returns an honest apiDown/offline snapshot
    /// using the caller's last knowledge of Spark reachability.
    public func fetchStatus(sparkWasReachable: Bool) -> RigSnapshot {
        do {
            return try ContactGate.attempt(.poll, isolates: isolates, detail: "fetchStatus") {
                self.fetchStatusLive(sparkWasReachable: sparkWasReachable)
            }
        } catch {
            return RigSnapshot.offline(reason: "local session")
        }
    }

    private func fetchStatusLive(sparkWasReachable: Bool) -> RigSnapshot {
        _ = ensureAPITunnel()
        let viewUp = viewPathUp
        guard let url = URL(string: "http://localhost:\(Self.apiPort)/status.json") else {
            return RigSnapshot.apiDown(reason: "bad URL", viewTunnelUp: viewUp)
        }
        let sema = DispatchSemaphore(value: 0)
        var body: String?
        var errText: String?
        let task = session.dataTask(with: url) { data, _, error in
            if let error { errText = error.localizedDescription }
            else if let data { body = String(data: data, encoding: .utf8) }
            sema.signal()
        }
        task.resume()
        _ = sema.wait(timeout: .now() + 5)

        guard let body else {
            let reason = errText ?? "no response from :\(Self.apiPort)"
            return sparkWasReachable
                ? RigSnapshot.apiDown(reason: reason, viewTunnelUp: viewUp)
                : RigSnapshot.offline(reason: reason)
        }
        guard let snap = RigStatusDecoder.parse(json: body, viewTunnelUp: viewUp) else {
            return RigSnapshot.apiDown(reason: "unparseable status payload", viewTunnelUp: viewUp)
        }
        return snap
    }

    /// POST /action. Returns the API's result line, or an error description.
    /// `extra` carries action-specific fields (protocol, pose_source,
    /// duration) straight through to the collector.
    public func postAction(_ name: String, arg: String? = nil,
                           extra: [String: Any] = [:]) -> (ok: Bool, message: String) {
        do {
            return try ContactGate.attempt(.action, isolates: isolates, detail: name) {
                self.postActionLive(name, arg: arg, extra: extra)
            }
        } catch {
            return (false, "local session")
        }
    }

    private func postActionLive(_ name: String, arg: String?, extra: [String: Any]) -> (ok: Bool, message: String) {
        guard let url = URL(string: "http://localhost:\(Self.apiPort)/action") else {
            return (false, "bad URL")
        }
        var req = URLRequest(url: url)
        req.httpMethod = "POST"
        req.setValue("application/json", forHTTPHeaderField: "Content-Type")
        var payload: [String: Any] = ["name": name]
        if let arg { payload["arg"] = arg }
        for (k, v) in extra { payload[k] = v }
        req.httpBody = try? JSONSerialization.data(withJSONObject: payload)

        let sema = DispatchSemaphore(value: 0)
        var out: (Bool, String) = (false, "no response")
        let task = session.dataTask(with: req) { data, resp, error in
            defer { sema.signal() }
            if let error {
                out = (false, error.localizedDescription)
                return
            }
            let code = (resp as? HTTPURLResponse)?.statusCode ?? 0
            let text: String = {
                guard let data,
                      let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
                      let r = obj["result"] as? String else { return "http \(code)" }
                return r
            }()
            out = (code == 200, text)
        }
        task.resume()
        _ = sema.wait(timeout: .now() + 20)
        return out
    }

    private struct ProcResult {
        var exitCode: Int32
        var stdout: String
    }

    private func runProcess(_ path: String, args: [String]) -> ProcResult {
        let spawn = ProcessSpawn(executable: path, arguments: args)
        do {
            let result = try ContactGate.runProcess(spawn, isolates: isolates) {
                self.spawnProcess(spawn)
            }
            return ProcResult(exitCode: result.exitCode, stdout: result.stdout)
        } catch {
            return ProcResult(exitCode: -1, stdout: "")
        }
    }

    private func spawnProcess(_ spawn: ProcessSpawn) -> ContactProcessResult {
        let proc = Process()
        proc.executableURL = URL(fileURLWithPath: spawn.executable)
        proc.arguments = spawn.arguments
        let out = Pipe()
        proc.standardOutput = out
        proc.standardError = Pipe()
        do {
            try proc.run()
            proc.waitUntilExit()
            let stdout = String(data: out.fileHandleForReading.readDataToEndOfFile(), encoding: .utf8) ?? ""
            return ContactProcessResult(exitCode: proc.terminationStatus, stdout: stdout, stderr: "")
        } catch {
            return ContactProcessResult(exitCode: -1, stdout: "", stderr: "", spawnError: error.localizedDescription)
        }
    }
}
