import Foundation

/// What the 3D view is showing: live clouds, or the sample PCD standing in. The
/// two are never merged, and the badge and log say which one is on screen.
public enum LiveViewMode: String, Sendable {
    case live = "LIVE"
    case sample = "SAMPLE"

    public static let sampleBadge = "SAMPLE — not live data"

    /// One sensor's share of the live view.
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

    /// "LIVE · Mid-360 19.9k pts 10.0 Hz · JT128 60k pts 10.0 Hz". A rate not yet
    /// measured is left out rather than shown as a guess.
    public static func liveBadge(_ readings: [Reading]) -> String {
        ([LiveViewMode.live.rawValue] + parts(readings)).joined(separator: " · ")
    }

    /// "Mid-360 19.9k pts 10.0 Hz, JT128 60k pts" — the badge's sensors, for logs.
    public static func summary(_ readings: [Reading]) -> String {
        parts(readings).joined(separator: ", ")
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
