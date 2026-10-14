import Foundation

/// What the 3D view is showing, for the badge and the `mode=` log line: nothing
/// captured (the sample PCD may stand in), one held frame per sensor, or a live
/// stream. Live and sample are never merged.
public enum LiveViewMode: String, Sendable {
    case off = "OFF"
    case snapshot = "SNAPSHOT"
    case live = "LIVE"

    public static let sampleBadge = "SAMPLE — not live data"

    /// One sensor's share of the view.
    public struct Reading: Equatable, Sendable {
        public var label: String
        public var points: Int
        public var hz: Double?

        public init(label: String, points: Int, hz: Double?) {
            self.label = label
            self.points = points
            self.hz = hz
        }
    }

    /// "OFF", or "OFF · SAMPLE — not live data" while the sample cloud is on screen.
    public static func offBadge(sampleShown: Bool) -> String {
        sampleShown ? "\(LiveViewMode.off.rawValue) · \(sampleBadge)" : LiveViewMode.off.rawValue
    }

    /// "SNAPSHOT · capturing… · Mid-360 19.9k pts" while the one-shot sockets are open.
    public static func capturingBadge(_ readings: [Reading]) -> String {
        ([LiveViewMode.snapshot.rawValue, "capturing…"] + parts(readings)).joined(separator: " · ")
    }

    /// "SNAPSHOT 10:42:13 · Mid-360 19.9k pts · JT128 no frame". Every sensor is
    /// listed, so one that timed out reads as missing rather than silently absent.
    public static func snapshotBadge(
        capturedAt: Date,
        points: [SensorID: Int],
        timeZone: TimeZone = .current
    ) -> String {
        let sensors = SensorID.allCases.map { sensor in
            points[sensor].map { "\(sensor.label) \(Self.points($0))" } ?? "\(sensor.label) no frame"
        }
        return (["\(LiveViewMode.snapshot.rawValue) \(clock(capturedAt, timeZone: timeZone))"] + sensors)
            .joined(separator: " · ")
    }

    /// "LIVE · Mid-360 19.9k pts 10.0 Hz · JT128 60k pts 10.0 Hz · 1:42 left". A rate
    /// not yet measured is left out rather than shown as a guess.
    public static func liveBadge(_ readings: [Reading], remaining: TimeInterval? = nil) -> String {
        var items = [LiveViewMode.live.rawValue]
        items += readings.isEmpty ? ["connecting…"] : parts(readings)
        if let remaining {
            items.append("\(minutesSeconds(remaining)) left")
        }
        return items.joined(separator: " · ")
    }

    /// "Mid-360 19.9k pts 10.0 Hz, JT128 60k pts" — the badge's sensors, for logs.
    public static func summary(_ readings: [Reading]) -> String {
        parts(readings).joined(separator: ", ")
    }

    /// "Mid-360 19.9k pts, JT128 no frame" — a snapshot's sensors, for logs.
    public static func snapshotSummary(_ points: [SensorID: Int]) -> String {
        SensorID.allCases.map { sensor in
            points[sensor].map { "\(sensor.label) \(Self.points($0))" } ?? "\(sensor.label) no frame"
        }.joined(separator: ", ")
    }

    private static func parts(_ readings: [Reading]) -> [String] {
        readings.map { reading in
            var part = "\(reading.label) \(points(reading.points))"
            if reading.hz != nil {
                part += " \(TelemetryFormat.hz(reading.hz))"
            }
            return part
        }
    }

    /// "850 pts", "19.9k pts", "60k pts".
    public static func points(_ count: Int) -> String {
        guard count >= 1000 else { return "\(count) pts" }
        var thousands = String(format: "%.1f", Double(count) / 1000)
        if thousands.hasSuffix(".0") {
            thousands.removeLast(2)
        }
        return "\(thousands)k pts"
    }

    /// "10:42:13", 24-hour, in `timeZone`.
    public static func clock(_ date: Date, timeZone: TimeZone = .current) -> String {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = timeZone
        formatter.dateFormat = "HH:mm:ss"
        return formatter.string(from: date)
    }

    /// "2:00", "0:07". Rounds up, so the badge never reads 0:00 while still live.
    public static func minutesSeconds(_ interval: TimeInterval) -> String {
        let seconds = max(0, Int(interval.rounded(.up)))
        return String(format: "%d:%02d", seconds / 60, seconds % 60)
    }
}

/// Message rate of one stream, from arrival times over a short sliding window.
public struct ArrivalRate: Sendable {
    public static let window: TimeInterval = 2

    private var times: [TimeInterval] = []

    public init() {}

    public mutating func note(_ time: TimeInterval) {
        times.append(time)
        times.removeAll { time - $0 > Self.window }
    }

    /// Nil until two arrivals fall inside the window, so a stream that stopped
    /// reads as unknown rather than its last rate.
    public func hz(now: TimeInterval) -> Double? {
        let recent = times.filter { now - $0 <= Self.window }
        guard let first = recent.first, let last = recent.last, recent.count >= 2, last > first else {
            return nil
        }
        return Double(recent.count - 1) / (last - first)
    }

    /// Events in the last `window` seconds, per second. Unlike `hz`, a stream
    /// slower than one event per window still reads (0.5 Hz for one), and a
    /// stopped one decays to 0 instead of vanishing.
    public func perSecond(now: TimeInterval) -> Double {
        Double(times.filter { now - $0 < Self.window && $0 <= now }.count) / Self.window
    }
}
