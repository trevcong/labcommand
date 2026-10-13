import Foundation

/// One probe's worth of plottable telemetry. Every field stays optional —
/// a probe that could not measure a rate contributes a gap, never a zero.
public struct TelemetrySample: Sendable, Equatable, Codable {
    public var at: Date
    public var pointRateHz: Double?
    public var imuRateHz: Double?
    public var sparkLoad: Double?
    public var sparkMemPct: Double?
    /// True when the container was up but producing nothing.
    public var blind: Bool
    /// v1.0 dual rig: the JT128's rates. Optional so v0.x history decodes.
    public var jtPointRateHz: Double?
    public var jtImuRateHz: Double?
    /// v1.4 mission clock: host−sensor clock offsets. Optional so older
    /// history decodes; nil is a gap (probe not sent), never a zero.
    public var midOffsetMs: Double?
    public var jtOffsetMs: Double?

    public init(
        at: Date,
        pointRateHz: Double? = nil,
        imuRateHz: Double? = nil,
        sparkLoad: Double? = nil,
        sparkMemPct: Double? = nil,
        blind: Bool = false,
        jtPointRateHz: Double? = nil,
        jtImuRateHz: Double? = nil,
        midOffsetMs: Double? = nil,
        jtOffsetMs: Double? = nil
    ) {
        self.at = at
        self.pointRateHz = pointRateHz
        self.imuRateHz = imuRateHz
        self.sparkLoad = sparkLoad
        self.sparkMemPct = sparkMemPct
        self.blind = blind
        self.jtPointRateHz = jtPointRateHz
        self.jtImuRateHz = jtImuRateHz
        self.midOffsetMs = midOffsetMs
        self.jtOffsetMs = jtOffsetMs
    }
}

/// Rolling window of probe samples so the panel can draw a trend instead of a
/// single number. A number tells you what is true now; a trace tells you when
/// it stopped being true — which is the failure this app exists to catch.
public struct TelemetryHistory: Sendable, Equatable, Codable {
    public private(set) var samples: [TelemetrySample]
    public let capacity: Int

    public init(capacity: Int = 120, samples: [TelemetrySample] = []) {
        self.capacity = max(2, capacity)
        self.samples = samples.suffix(self.capacity)
    }

    public mutating func append(_ s: TelemetrySample) {
        samples.append(s)
        if samples.count > capacity {
            samples.removeFirst(samples.count - capacity)
        }
    }

    public func appending(_ s: TelemetrySample) -> TelemetryHistory {
        var c = self
        c.append(s)
        return c
    }

    public var isEmpty: Bool { samples.isEmpty }

    /// Values for one series, `nil` where that probe had no measurement.
    /// Callers must break the line at nil rather than interpolating across it.
    public func series(_ key: Series) -> [Double?] {
        samples.map { s in
            switch key {
            case .pointRate: return s.pointRateHz
            case .imuRate: return s.imuRateHz
            case .load: return s.sparkLoad
            case .mem: return s.sparkMemPct
            case .jtPointRate: return s.jtPointRateHz
            case .jtImuRate: return s.jtImuRateHz
            case .midOffset: return s.midOffsetMs
            case .jtOffset: return s.jtOffsetMs
            }
        }
    }

    public enum Series: String, Sendable, CaseIterable {
        case pointRate, imuRate, load, mem, jtPointRate, jtImuRate
        case midOffset, jtOffset

        public var label: String {
            switch self {
            case .pointRate: return "Mid points"
            case .imuRate: return "Mid IMU"
            case .load: return "Load"
            case .mem: return "Mem"
            case .jtPointRate: return "JT points"
            case .jtImuRate: return "JT IMU"
            case .midOffset: return "Mid clock"
            case .jtOffset: return "JT clock"
            }
        }
    }

    /// Plot range for a series. Returns nil when nothing was ever measured, so
    /// the view can print "unavailable" instead of drawing an empty axis.
    public func range(_ key: Series) -> (lo: Double, hi: Double)? {
        let vals = series(key).compactMap { $0 }
        guard let mn = vals.min(), let mx = vals.max() else { return nil }
        if mx - mn < 1e-9 {
            // Flat trace still deserves a band so the line sits mid-plot.
            let pad = max(abs(mx) * 0.1, 0.5)
            return (mn - pad, mx + pad)
        }
        let pad = (mx - mn) * 0.12
        return (mn - pad, mx + pad)
    }

    public var latest: TelemetrySample? { samples.last }

    /// Most recent measured value for a series, with the time it was taken.
    /// An instrument that goes blank when the link drops is less useful than
    /// one that says "9.8 Hz, four minutes ago".
    public func lastKnown(_ key: Series) -> (value: Double, at: Date)? {
        for s in samples.reversed() {
            let v: Double?
            switch key {
            case .pointRate: v = s.pointRateHz
            case .imuRate: v = s.imuRateHz
            case .load: v = s.sparkLoad
            case .mem: v = s.sparkMemPct
            case .jtPointRate: v = s.jtPointRateHz
            case .jtImuRate: v = s.jtImuRateHz
            case .midOffset: v = s.midOffsetMs
            case .jtOffset: v = s.jtOffsetMs
            }
            if let v { return (v, s.at) }
        }
        return nil
    }

    /// Short relative age, e.g. "4m ago". Nil when the sample is current.
    public static func age(_ at: Date, now: Date = Date()) -> String {
        let s = Int(max(0, now.timeIntervalSince(at)))
        if s < 5 { return "now" }
        if s < 60 { return "\(s)s ago" }
        if s < 3600 { return "\(s / 60)m ago" }
        return "\(s / 3600)h ago"
    }

    /// How long the rate has read a measured zero. This is the number that
    /// would have surfaced five hours of FAST-LIO spinning on an empty cloud.
    public func blindDuration(now: Date = Date()) -> TimeInterval? {
        guard let last = samples.last, last.blind else { return nil }
        var start = last.at
        for s in samples.reversed() {
            if s.blind { start = s.at } else { break }
        }
        return max(0, now.timeIntervalSince(start))
    }

    /// Least-squares slope of a series over the trailing window, in units/min.
    /// This is a fit over measured samples — the caller labels anything derived
    /// from it as a projection, never a promise. Nil when fewer than 3 samples
    /// were measured in the window.
    public func slewPerMin(_ key: Series, window: TimeInterval = 300,
                           now: Date = Date()) -> Double? {
        let cutoff = now.addingTimeInterval(-window)
        var pts: [(t: Double, v: Double)] = []
        for s in samples where s.at >= cutoff {
            let v: Double?
            switch key {
            case .pointRate: v = s.pointRateHz
            case .imuRate: v = s.imuRateHz
            case .load: v = s.sparkLoad
            case .mem: v = s.sparkMemPct
            case .jtPointRate: v = s.jtPointRateHz
            case .jtImuRate: v = s.jtImuRateHz
            case .midOffset: v = s.midOffsetMs
            case .jtOffset: v = s.jtOffsetMs
            }
            if let v { pts.append((s.at.timeIntervalSinceReferenceDate / 60.0, v)) }
        }
        guard pts.count >= 3 else { return nil }
        let n = Double(pts.count)
        let sumT = pts.reduce(0) { $0 + $1.t }
        let sumV = pts.reduce(0) { $0 + $1.v }
        let sumTT = pts.reduce(0) { $0 + $1.t * $1.t }
        let sumTV = pts.reduce(0) { $0 + $1.t * $1.v }
        let denom = n * sumTT - sumT * sumT
        guard abs(denom) > 1e-12 else { return nil }
        return (n * sumTV - sumT * sumV) / denom
    }

    /// Minutes until |value| enters `band`, projected from the measured slew.
    /// Nil when there is no fit, the value is already in band, or the slew is
    /// moving away from the band (an ETA there would be an invention).
    public func minutesToBand(_ key: Series, band: Double,
                              window: TimeInterval = 300,
                              now: Date = Date()) -> Double? {
        guard let (value, _) = lastKnown(key), abs(value) > band,
              let slew = slewPerMin(key, window: window, now: now),
              abs(slew) > 1e-9 else { return nil }
        // Converging only if the slope moves the value toward zero.
        guard (value > 0 && slew < 0) || (value < 0 && slew > 0) else { return nil }
        return (abs(value) - band) / abs(slew)
    }

    public static func format(duration: TimeInterval) -> String {
        let s = Int(duration)
        if s < 60 { return "\(s)s" }
        if s < 3600 { return "\(s / 60)m \(s % 60)s" }
        return "\(s / 3600)h \((s % 3600) / 60)m"
    }
}


/// Disk persistence. A console that forgets everything on launch cannot show
/// you what the rate was before you walked away from it.
public enum TelemetryStore {
    public static func defaultURL(
        fileManager: FileManager = .default
    ) -> URL {
        let base = fileManager.urls(for: .applicationSupportDirectory, in: .userDomainMask).first
            ?? fileManager.homeDirectoryForCurrentUser.appendingPathComponent("Library/Application Support")
        return base
            .appendingPathComponent("LabCommand", isDirectory: true)
            .appendingPathComponent("telemetry.json")
    }

    public static func load(from url: URL = defaultURL()) -> TelemetryHistory {
        guard let data = try? Data(contentsOf: url),
              let h = try? JSONDecoder().decode(TelemetryHistory.self, from: data) else {
            return TelemetryHistory()
        }
        return h
    }

    @discardableResult
    public static func save(_ h: TelemetryHistory, to url: URL = defaultURL()) -> Bool {
        do {
            try FileManager.default.createDirectory(
                at: url.deletingLastPathComponent(), withIntermediateDirectories: true)
            let data = try JSONEncoder().encode(h)
            try data.write(to: url, options: .atomic)
            return true
        } catch {
            return false
        }
    }
}
