import Foundation

/// SSH connection vs remote-script failure. 255 + a transport error means try the next hop.
public enum SparkSSH {
    public static func connectionFailed(exitCode: Int32, stderr: String, spawnError: String?) -> Bool {
        if spawnError != nil { return true }
        guard exitCode == 255 else { return false }
        let s = stderr.lowercased()
        let needles = [
            "timed out",
            "timeout",
            "could not resolve",
            "nodename nor servname",
            "connection refused",
            "network is unreachable",
            "no route to host",
            "host is down",
            "connection reset",
            "broken pipe",
        ]
        return needles.contains { s.contains($0) } || s.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
    }

    /// Ordered SSH hops. Never lead with a stale DHCP lease.
    /// CurAddr from Tailscale (LAN) first, then the tailnet IP, then DNS, then last-known LAN.
    public static func hopList(
        tailscaleJSON: String? = nil,
        lastLAN: String? = nil
    ) -> [String] {
        var hops: [String] = []
        func add(_ raw: String?) {
            guard let raw else { return }
            var t = raw.trimmingCharacters(in: .whitespacesAndNewlines)
            if let colon = t.lastIndex(of: ":"), t[t.index(after: colon)...].allSatisfy(\.isNumber) {
                // strip :port from CurAddr like 192.168.68.51:41641
                t = String(t[..<colon])
            }
            guard !t.isEmpty, !hops.contains(t) else { return }
            hops.append(t)
        }
        if let json = tailscaleJSON {
            let peer = TailscaleStatusParser.parse(json)
            add(peer.curAddr)
            for ip in peer.ips where !ip.contains(":") {
                add(ip)
            }
        }
        add(CaptureRecipe.tailscaleIP)
        add(CaptureRecipe.tailscaleDNS)
        add(lastLAN)
        add(CaptureRecipe.tailscaleSSHHost)
        add(CaptureRecipe.sshHost)
        return hops
    }

    public static func firstReachableHost<Result>(
        hosts: [String] = CaptureRecipe.sshHostCandidates,
        tryHost: (String) -> (Result, Int32, String, String?)
    ) -> (host: String, result: Result) {
        precondition(!hosts.isEmpty, "ssh host list must not be empty")
        var last: (String, Result)?
        for host in hosts {
            let (result, code, stderr, spawn) = tryHost(host)
            last = (host, result)
            if !connectionFailed(exitCode: code, stderr: stderr, spawnError: spawn) {
                return (host, result)
            }
        }
        return last!
    }
}

/// Last working LAN hop. DHCP moves; Refresh must not stay pinned to a dead lease.
public enum SparkHopCache {
    public static func defaultURL(home: URL = FileManager.default.homeDirectoryForCurrentUser) -> URL {
        home.appendingPathComponent("Library/Application Support/LabCommand/spark-hop.txt")
    }

    public static func load(from url: URL? = nil) -> String? {
        let target = url ?? defaultURL()
        guard let text = try? String(contentsOf: target, encoding: .utf8) else { return nil }
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        return trimmed.isEmpty ? nil : trimmed
    }

    public static func save(_ host: String, to url: URL? = nil) {
        let trimmed = host.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return }
        let target = url ?? defaultURL()
        try? FileManager.default.createDirectory(
            at: target.deletingLastPathComponent(),
            withIntermediateDirectories: true
        )
        try? trimmed.write(to: target, atomically: true, encoding: .utf8)
    }
}
