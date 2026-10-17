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

/// Append-only operator/probe timeline. `append` and `renderLine` are pure.
/// `clock` is the shared operator formatter.
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

    private static let clockLock = NSLock()
    private static var clockFormatter = makeClock(timeZone: .autoupdatingCurrent)

    /// Installed on the first clock read. `DateFormatter` copies the zone it is
    /// given, so a system-zone change rebuilds the formatter instead of trusting
    /// the old one to notice `autoupdatingCurrent`.
    private static let timeZoneWatch: NSObjectProtocol = NotificationCenter.default.addObserver(
        forName: .NSSystemTimeZoneDidChange,
        object: nil,
        queue: nil
    ) { _ in
        NSTimeZone.resetSystemTimeZone()
        refreshClock(timeZone: .autoupdatingCurrent)
    }

    /// The operator clock: `yyyy-MM-dd HH:mm:ss` in the live system zone.
    public static var clock: DateFormatter {
        _ = timeZoneWatch
        clockLock.lock()
        defer { clockLock.unlock() }
        return clockFormatter
    }

    /// Rebuild the shared clock. The system-zone notification passes
    /// `autoupdatingCurrent`.
    public static func refreshClock(timeZone: TimeZone = .autoupdatingCurrent) {
        _ = timeZoneWatch
        let next = makeClock(timeZone: timeZone)
        clockLock.lock()
        clockFormatter = next
        clockLock.unlock()
    }

    private static func makeClock(timeZone: TimeZone) -> DateFormatter {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = timeZone
        formatter.dateFormat = "yyyy-MM-dd HH:mm:ss"
        return formatter
    }
}
