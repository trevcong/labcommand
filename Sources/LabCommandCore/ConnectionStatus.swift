import Foundation

/// Display-level connection state for the command center.
public enum ConnectionStatus: String, Sendable, Equatable, CaseIterable {
    case connected
    case disconnected
    case unknown
    case error

    public var displayLabel: String {
        switch self {
        case .connected: return "Connected"
        case .disconnected: return "Disconnected"
        case .unknown: return "Unknown"
        case .error: return "Error"
        }
    }

    public var accessibilityLabel: String {
        "Status \(displayLabel)"
    }
}

/// One atomic probe outcome (SSH, Tailscale, ping, …).
public struct CheckOutcome: Sendable, Equatable {
    public enum Kind: String, Sendable, Equatable {
        case success
        case failure
        case error
        case skipped
    }

    public var id: String
    public var kind: Kind
    public var detail: String
    public var checkedAt: Date

    public init(
        id: String,
        kind: Kind,
        detail: String,
        checkedAt: Date = Date()
    ) {
        self.id = id
        self.kind = kind
        self.detail = detail
        self.checkedAt = checkedAt
    }
}

/// Aggregated lab snapshot for the main UI.
public struct LabStatusSnapshot: Sendable, Equatable {
    public var overall: ConnectionStatus
    public var spark: ConnectionStatus
    public var tailscale: ConnectionStatus
    public var checks: [CheckOutcome]
    public var summaryLine: String
    public var refreshedAt: Date

    public init(
        overall: ConnectionStatus,
        spark: ConnectionStatus,
        tailscale: ConnectionStatus,
        checks: [CheckOutcome],
        summaryLine: String,
        refreshedAt: Date = Date()
    ) {
        self.overall = overall
        self.spark = spark
        self.tailscale = tailscale
        self.checks = checks
        self.summaryLine = summaryLine
        self.refreshedAt = refreshedAt
    }
}
