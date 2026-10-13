import Foundation

/// Parsed Tailscale status for the Spark peer. CLI-up is not the same as Spark-online.
public struct TailscalePeerStatus: Sendable, Equatable {
    public var backendState: String
    public var peerListed: Bool
    public var online: Bool
    public var lastSeen: Date?
    public var hostname: String
    public var ips: [String]
    public var relay: String
    public var active: Bool
    public var curAddr: String

    public init(
        backendState: String = "",
        peerListed: Bool = false,
        online: Bool = false,
        lastSeen: Date? = nil,
        hostname: String = "",
        ips: [String] = [],
        relay: String = "",
        active: Bool = false,
        curAddr: String = ""
    ) {
        self.backendState = backendState
        self.peerListed = peerListed
        self.online = online
        self.lastSeen = lastSeen
        self.hostname = hostname
        self.ips = ips
        self.relay = relay
        self.active = active
        self.curAddr = curAddr
    }

    /// Tailscale often leaves Online=false while a direct LAN path is already up.
    public var reachable: Bool {
        if online { return true }
        if active && !curAddr.isEmpty { return true }
        return false
    }
}

/// Pure parser for `tailscale status --json`. Views bind to `describe`; they do not invent last-seen.
public enum TailscaleStatusParser {
    public static let sparkNeedles = ["spark", "100.118.187.99"]

    public static func parse(
        _ json: String,
        needles: [String] = sparkNeedles,
        now: Date = Date()
    ) -> TailscalePeerStatus {
        guard let data = json.data(using: .utf8),
              let root = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            return TailscalePeerStatus(backendState: "unparsed")
        }
        let backend = stringValue(root["BackendState"]) ?? ""
        let peers = root["Peer"] as? [String: Any] ?? [:]
        for (_, value) in peers {
            guard let peer = value as? [String: Any] else { continue }
            if matches(peer, needles: needles) {
                let lastSeen = parseDate(stringValue(peer["LastSeen"]))
                return TailscalePeerStatus(
                    backendState: backend,
                    peerListed: true,
                    online: boolValue(peer["Online"]),
                    lastSeen: lastSeen,
                    hostname: stringValue(peer["HostName"]) ?? "",
                    ips: stringArray(peer["TailscaleIPs"]),
                    relay: stringValue(peer["Relay"]) ?? "",
                    active: boolValue(peer["Active"]),
                    curAddr: stringValue(peer["CurAddr"]) ?? ""
                )
            }
        }
        return TailscalePeerStatus(backendState: backend, peerListed: false)
    }

    public static func relativeAge(from date: Date, now: Date) -> String {
        let seconds = max(0, Int(now.timeIntervalSince(date)))
        if seconds < 60 { return "just now" }
        if seconds < 3600 { return "\(seconds / 60)m ago" }
        if seconds < 36 * 3600 { return "\(seconds / 3600)h ago" }
        return "\(seconds / 86_400)d ago"
    }

    /// One-line Tailscale check detail. Distinguishes "peer listed" from "peer online".
    public static func describe(_ status: TailscalePeerStatus, now: Date = Date()) -> String {
        let backend = status.backendState.isEmpty ? "unknown" : status.backendState
        if backend.lowercased() != "running" && !backend.isEmpty {
            return "tailscale \(backend.lowercased())"
        }
        if !status.peerListed {
            return "tailscale up · spark peer not listed"
        }
        let name = status.hostname.isEmpty ? "spark" : status.hostname
        if status.reachable {
            if !status.curAddr.isEmpty {
                return "tailscale up · \(name) reachable (\(status.curAddr))"
            }
            return "tailscale up · \(name) online"
        }
        if let seen = status.lastSeen {
            return "tailscale up · \(name) offline, last seen \(relativeAge(from: seen, now: now))"
        }
        return "tailscale up · \(name) offline"
    }

    // MARK: - JSON helpers

    private static func matches(_ peer: [String: Any], needles: [String]) -> Bool {
        var hay = (stringValue(peer["HostName"]) ?? "")
            + " "
            + (stringValue(peer["DNSName"]) ?? "")
        for ip in stringArray(peer["TailscaleIPs"]) {
            hay += " " + ip
        }
        let folded = hay.lowercased()
        return needles.contains { folded.contains($0.lowercased()) }
    }

    private static func stringValue(_ any: Any?) -> String? {
        if let s = any as? String { return s }
        return nil
    }

    private static func boolValue(_ any: Any?) -> Bool {
        if let b = any as? Bool { return b }
        if let n = any as? NSNumber { return n.boolValue }
        return false
    }

    private static func stringArray(_ any: Any?) -> [String] {
        (any as? [Any])?.compactMap { $0 as? String } ?? []
    }

    private static func parseDate(_ raw: String?) -> Date? {
        guard let raw, !raw.isEmpty else { return nil }
        let fractional = ISO8601DateFormatter()
        fractional.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        if let date = fractional.date(from: raw) { return date }
        let basic = ISO8601DateFormatter()
        basic.formatOptions = [.withInternetDateTime]
        return basic.date(from: raw)
    }
}
