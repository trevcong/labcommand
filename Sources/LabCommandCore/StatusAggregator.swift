import Foundation

/// Pure mapping from probe outcomes → UI connection states.
/// Free of AppKit/SwiftUI so it unit-tests without launching the GUI.
public enum StatusAggregator {
    public static let sparkCheckIDs: Set<String> = ["spark_ssh", "spark_ping"]
    public static let tailscaleCheckIDs: Set<String> = ["tailscale"]

    /// Map a single check to a coarse connection status.
    public static func status(for outcome: CheckOutcome) -> ConnectionStatus {
        switch outcome.kind {
        case .success:
            return .connected
        case .failure:
            return .disconnected
        case .error:
            return .error
        case .skipped:
            return .unknown
        }
    }

    /// Combine several outcomes: worst-wins with error > disconnected > unknown > connected.
    public static func combine(_ statuses: [ConnectionStatus]) -> ConnectionStatus {
        guard !statuses.isEmpty else { return .unknown }
        if statuses.contains(.error) { return .error }
        if statuses.contains(.disconnected) { return .disconnected }
        if statuses.contains(.unknown) { return .unknown }
        return .connected
    }

    /// Filter checks by id set and combine their statuses (worst-wins).
    public static func status(
        from checks: [CheckOutcome],
        matching ids: Set<String>
    ) -> ConnectionStatus {
        let subset = checks.filter { ids.contains($0.id) }
        guard !subset.isEmpty else { return .unknown }
        return combine(subset.map(status(for:)))
    }

    /// Any success among matching checks wins (SSH ok is enough even if ping fails).
    public static func statusPreferSuccess(
        from checks: [CheckOutcome],
        matching ids: Set<String>
    ) -> ConnectionStatus {
        let subset = checks.filter { ids.contains($0.id) }
        guard !subset.isEmpty else { return .unknown }
        let statuses = subset.map(status(for:))
        if statuses.contains(.connected) { return .connected }
        return combine(statuses)
    }

    /// Build the snapshot the command center binds to.
    public static func snapshot(from checks: [CheckOutcome], now: Date = Date()) -> LabStatusSnapshot {
        // Spark: SSH success alone is enough for "connected" (ping is supporting).
        let spark = statusPreferSuccess(from: checks, matching: sparkCheckIDs)
        let tailscale = status(from: checks, matching: tailscaleCheckIDs)
        // Overall lab link follows Spark primarily; Tailscale alone cannot greenlight.
        let overall: ConnectionStatus
        if spark == .connected {
            overall = .connected
        } else {
            overall = spark
        }

        let summary: String
        switch overall {
        case .connected:
            summary = "Lab link healthy — Spark reachable"
        case .disconnected:
            summary = "Lab link down — Spark not reachable"
        case .error:
            summary = "Lab check error — see detail cards"
        case .unknown:
            summary = "Lab status unknown — run refresh"
        }

        return LabStatusSnapshot(
            overall: overall,
            spark: spark,
            tailscale: tailscale,
            checks: checks,
            summaryLine: summary,
            refreshedAt: now
        )
    }
}
