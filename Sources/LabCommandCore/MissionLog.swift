import Foundation

public enum MissionEventKind: String, Sendable, Equatable {
    case start
    case record
    case stop
    case probe
    case diagnose
    case note
    case focus
    case error
    case info

    public var displayLabel: String {
        rawValue
    }
}

public struct MissionEvent: Sendable, Equatable, Identifiable {
    public var id: UUID
    public var at: Date
    public var kind: MissionEventKind
    public var text: String

    public init(
        id: UUID = UUID(),
        at: Date = Date(),
        kind: MissionEventKind,
        text: String
    ) {
        self.id = id
        self.at = at
        self.kind = kind
        self.text = text
    }
}

/// Append-only operator/probe timeline. Pure so tests can drive it.
public enum MissionLog {
    public static let maxEvents = 200

    public static func append(
        _ events: [MissionEvent],
        kind: MissionEventKind,
        text: String,
        at: Date = Date(),
        id: UUID = UUID()
    ) -> [MissionEvent] {
        var next = events
        next.append(MissionEvent(id: id, at: at, kind: kind, text: text))
        if next.count > maxEvents {
            next.removeFirst(next.count - maxEvents)
        }
        return next
    }

    public static func renderLine(_ event: MissionEvent, clock: DateFormatter = MissionLog.clock) -> String {
        "\(clock.string(from: event.at))  \(event.kind.displayLabel)  \(event.text)"
    }

    public static let clock: DateFormatter = {
        let f = DateFormatter()
        f.locale = Locale(identifier: "en_US_POSIX")
        f.dateFormat = "HH:mm:ss"
        return f
    }()
}
