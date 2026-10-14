import Foundation
import LabCommandCore

/// I/O for the dual rig: own the two SSH tunnels (8765 Foxglove, 8766
/// command-center API) and speak HTTP to the API. Replaces the v0.5
/// spark-capture.sh probe path — the Spark-side collector is the one truth,
/// shared with the web panel.
public final class RigService: @unchecked Sendable {
    public static let apiPort = 8766
    public static let viewPort = CaptureRecipe.foxglovePort

    private let lock = NSLock()
    private var viewTunnel: Process?
    private var apiTunnel: Process?
    private let session: URLSession

    public init() {
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

    @discardableResult
    private func startTunnel(port: Int, key: ReferenceWritableKeyPath<RigService, Process?>) async -> String? {
        if tunnelRunning(key) || portOpen(port) { return nil }
        var lastError: String?
        for host in hostOrder() {
            let proc = Process()
            proc.executableURL = URL(fileURLWithPath: "/usr/bin/ssh")
            proc.arguments = CaptureRecipe.viewTunnelArguments(host: host, port: port)
            proc.standardOutput = Pipe()
            proc.standardError = Pipe()
            do {
                try proc.run()
            } catch {
                lastError = error.localizedDescription
                continue
            }
            try? await Task.sleep(for: .milliseconds(250))
            if proc.isRunning {
                lock.lock()
                self[keyPath: key] = proc
                lock.unlock()
                if host.contains(".") { SparkHopCache.save(host) }
                return nil
            }
            lastError = "tunnel to \(host) exited"
        }
        return lastError ?? "tunnel failed"
    }

    /// Both tunnels. Returns the first error, or nil.
    public func ensureTunnels() async -> String? {
        let apiErr = await startTunnel(port: Self.apiPort, key: \.apiTunnel)
        let viewErr = await startTunnel(port: Self.viewPort, key: \.viewTunnel)
        return apiErr ?? viewErr
    }

    public func ensureAPITunnel() async -> String? {
        await startTunnel(port: Self.apiPort, key: \.apiTunnel)
    }

    public func stopTunnels() {
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
        let proc = Process()
        proc.executableURL = URL(fileURLWithPath: path)
        proc.arguments = args
        let out = Pipe()
        proc.standardOutput = out
        proc.standardError = Pipe()
        do {
            try proc.run()
            proc.waitUntilExit()
            let stdout = String(data: out.fileHandleForReading.readDataToEndOfFile(), encoding: .utf8) ?? ""
            return ProcResult(exitCode: proc.terminationStatus, stdout: stdout)
        } catch {
            return ProcResult(exitCode: -1, stdout: "")
        }
    }
}
