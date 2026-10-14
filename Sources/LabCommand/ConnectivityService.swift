import Foundation
import LabCommandCore

/// Thin I/O layer: real shell probes → CheckOutcome. Not mocked green.
public struct ConnectivityService: Sendable {
    public var sparkSSHHost: String
    public var sparkPingHost: String
    public var connectTimeoutSeconds: Int

    /// Snapshot launches refuse every process. The rig path spawns as before.
    public var isolates: Bool

    public init(
        sparkSSHHost: String = "spark",
        sparkPingHost: String = CaptureRecipe.tailscaleIP,
        connectTimeoutSeconds: Int = 2,
        isolates: Bool = false
    ) {
        self.sparkSSHHost = sparkSSHHost
        self.sparkPingHost = sparkPingHost
        self.connectTimeoutSeconds = connectTimeoutSeconds
        self.isolates = isolates
    }

    public func runAllChecks() -> [CheckOutcome] {
        [checkTailscale(), checkSparkSSH(), checkSparkPing()]
    }

    public func checkTailscale() -> CheckOutcome {
        let result = runProcess(
            "/usr/bin/env",
            args: ["tailscale", "status", "--json"]
        )
        if result.spawnError != nil {
            // Fallback: binary missing or not on PATH
            let which = runProcess("/usr/bin/which", args: ["tailscale"])
            if which.exitCode != 0 {
                return CheckOutcome(
                    id: "tailscale",
                    kind: .skipped,
                    detail: "tailscale CLI not found"
                )
            }
            return CheckOutcome(
                id: "tailscale",
                kind: .error,
                detail: result.spawnError ?? "tailscale failed to start"
            )
        }
        if result.exitCode == 0 {
            let peer = TailscaleStatusParser.parse(result.stdout)
            return CheckOutcome(
                id: "tailscale",
                kind: .success,
                detail: TailscaleStatusParser.describe(peer)
            )
        }
        return CheckOutcome(
            id: "tailscale",
            kind: .failure,
            detail: truncate(result.stderr.isEmpty ? "exit \(result.exitCode)" : result.stderr)
        )
    }

    public func checkSparkSSH() -> CheckOutcome {
        let ts = runProcess("/usr/bin/env", args: ["tailscale", "status", "--json"])
        let hops = SparkSSH.hopList(
            tailscaleJSON: ts.exitCode == 0 ? ts.stdout : nil,
            lastLAN: SparkHopCache.load()
        )
        var lastDetail = "ssh failed"
        let identity = CaptureRecipe.sshIdentityPath()
        for host in hops {
            let result = runProcess(
                "/usr/bin/ssh",
                args: [
                    "-o", "BatchMode=yes",
                    "-o", "ConnectTimeout=\(connectTimeoutSeconds)",
                    "-o", "StrictHostKeyChecking=accept-new",
                    "-o", "IdentitiesOnly=yes",
                    "-i", identity,
                    "-l", CaptureRecipe.sshUser,
                    host,
                    "echo LAB_OK; hostname",
                ]
            )
            if let err = result.spawnError {
                lastDetail = err
                if SparkSSH.connectionFailed(exitCode: -1, stderr: "", spawnError: err) {
                    continue
                }
                return CheckOutcome(id: "spark_ssh", kind: .error, detail: err)
            }
            if result.exitCode == 0, result.stdout.contains("LAB_OK") {
                if host.contains(".") { SparkHopCache.save(host) }
                let hostLine = result.stdout
                    .split(separator: "\n")
                    .map(String.init)
                    .filter { $0 != "LAB_OK" }
                    .first ?? host
                return CheckOutcome(
                    id: "spark_ssh",
                    kind: .success,
                    detail: "ssh \(host) → \(hostLine.trimmingCharacters(in: .whitespacesAndNewlines))"
                )
            }
            lastDetail = truncate(
                result.stderr.isEmpty
                    ? (result.stdout.isEmpty ? "ssh \(host) exit \(result.exitCode)" : result.stdout)
                    : result.stderr
            )
            if SparkSSH.connectionFailed(
                exitCode: result.exitCode,
                stderr: result.stderr,
                spawnError: result.spawnError
            ) {
                continue
            }
            return CheckOutcome(id: "spark_ssh", kind: .failure, detail: lastDetail)
        }
        return CheckOutcome(id: "spark_ssh", kind: .failure, detail: lastDetail)
    }

    public func checkSparkPing() -> CheckOutcome {
        let result = runProcess(
            "/sbin/ping",
            args: ["-c", "1", "-t", "\(connectTimeoutSeconds)", sparkPingHost]
        )
        if let err = result.spawnError {
            return CheckOutcome(id: "spark_ping", kind: .error, detail: err)
        }
        if result.exitCode == 0 {
            return CheckOutcome(
                id: "spark_ping",
                kind: .success,
                detail: "ping \(sparkPingHost) ok"
            )
        }
        return CheckOutcome(
            id: "spark_ping",
            kind: .failure,
            detail: truncate(result.stderr.isEmpty ? "ping failed (exit \(result.exitCode))" : result.stderr)
        )
    }

    // MARK: - process helper

    private struct ProcessResult {
        var exitCode: Int32
        var stdout: String
        var stderr: String
        var spawnError: String?
    }

    private func runProcess(_ launchPath: String, args: [String]) -> ProcessResult {
        let spawn = ProcessSpawn(executable: launchPath, arguments: args)
        do {
            let result = try ContactGate.runProcess(spawn, isolates: isolates) {
                self.spawnProcess(spawn)
            }
            return ProcessResult(
                exitCode: result.exitCode,
                stdout: result.stdout,
                stderr: result.stderr,
                spawnError: result.spawnError
            )
        } catch {
            return ProcessResult(exitCode: -1, stdout: "", stderr: "", spawnError: "local session")
        }
    }

    private func spawnProcess(_ spawn: ProcessSpawn) -> ContactProcessResult {
        let proc = Process()
        proc.executableURL = URL(fileURLWithPath: spawn.executable)
        proc.arguments = spawn.arguments
        let outPipe = Pipe()
        let errPipe = Pipe()
        proc.standardOutput = outPipe
        proc.standardError = errPipe
        do {
            try proc.run()
            proc.waitUntilExit()
            let outData = outPipe.fileHandleForReading.readDataToEndOfFile()
            let errData = errPipe.fileHandleForReading.readDataToEndOfFile()
            return ContactProcessResult(
                exitCode: proc.terminationStatus,
                stdout: String(data: outData, encoding: .utf8) ?? "",
                stderr: String(data: errData, encoding: .utf8) ?? ""
            )
        } catch {
            return ContactProcessResult(
                exitCode: -1,
                stdout: "",
                stderr: "",
                spawnError: error.localizedDescription
            )
        }
    }

    private func truncate(_ s: String, limit: Int = 180) -> String {
        let t = s.trimmingCharacters(in: .whitespacesAndNewlines)
        if t.count <= limit { return t }
        return String(t.prefix(limit)) + "…"
    }
}
