import Foundation

/// Rig-level health band. `parked` is the v1.0 addition: sensors deliberately
/// unpowered with the infrastructure healthy is calm, not a failure.
public enum RigBand: String, Sendable, Equatable {
    case ready
    case watch
    case parked
    case blocked
    case unavailable

    public var label: String {
        switch self {
        case .ready: return "Ready"
        case .watch: return "Watch"
        case .parked: return "Parked"
        case .blocked: return "Blocked"
        case .unavailable: return "Unavailable"
        }
    }
}

public struct RigConfidence: Sendable, Equatable {
    public var band: RigBand
    public var headline: String
    public var reasons: [String]

    public init(band: RigBand, headline: String, reasons: [String] = []) {
        self.band = band
        self.headline = headline
        self.reasons = reasons
    }

    public var label: String { band.label }
}

public enum RigHealth {
    public static func evaluate(_ s: RigSnapshot) -> RigConfidence {
        if !s.sparkReachable {
            return RigConfidence(
                band: .blocked,
                headline: s.blockReason ?? "Spark is unreachable",
                reasons: ["No SSH path to the Spark. Tailscale on both ends is the usual fix."]
            )
        }
        if !s.apiReachable {
            return RigConfidence(
                band: .unavailable,
                headline: "Command center is not answering",
                reasons: [s.blockReason ?? "lidar-center container down, or the :8766 tunnel is closed."]
            )
        }

        var reasons: [String] = []
        if !s.watchdogRunning { reasons.append("Watchdog container is not running.") }
        if !s.ptpMasterRunning { reasons.append("PTP master is not running — sensor clocks free-run.") }

        if s.recording.active {
            var head = "Recording"
            if let session = s.recording.session { head += " \(session)" }
            if let gb = s.recording.sizeGB { head += String(format: " · %.1f GB", gb) }
            if s.mid360.state != .streaming { reasons.append("Mid-360 is not contributing points.") }
            if s.jt128.state != .streaming { reasons.append("JT128 is not contributing points.") }
            return RigConfidence(band: reasons.isEmpty ? .ready : .watch, headline: head, reasons: reasons)
        }

        if s.bothDark {
            if !s.stackRunning { reasons.append("Driver stack is stopped — the watchdog restarts it on power-up.") }
            return RigConfidence(
                band: reasons.contains(where: { $0.hasPrefix("Watchdog") }) ? .watch : .parked,
                headline: "Sensors are parked — plug in power to begin",
                reasons: reasons
            )
        }

        if !s.stackRunning {
            reasons.insert("Driver stack is stopped. Start rig brings it up.", at: 0)
            return RigConfidence(band: .watch, headline: "Sensors powered, drivers stopped", reasons: reasons)
        }

        for sensor in [s.mid360, s.jt128] where sensor.state == .powered {
            reasons.append("\(sensor.id.label): \(sensor.detail).")
        }
        for sensor in [s.mid360, s.jt128] where sensor.state == .dark {
            reasons.append("\(sensor.id.label) is unpowered.")
        }
        if s.clock.state == .converging {
            reasons.append("Clocks still converging (Mid-360 slews ~10 ms/min after power-up).")
        }

        if s.bothStreaming && reasons.isEmpty {
            return RigConfidence(band: .ready, headline: "Both sensors live — ready to record")
        }
        if s.anyStreaming {
            return RigConfidence(
                band: reasons.isEmpty ? .ready : .watch,
                headline: s.mid360.state == .streaming && s.jt128.state != .streaming
                    ? "Mid-360 live · JT128 not streaming"
                    : (s.jt128.state == .streaming && s.mid360.state != .streaming
                        ? "JT128 live · Mid-360 not streaming"
                        : "Streaming"),
                reasons: reasons
            )
        }
        return RigConfidence(band: .watch, headline: "Powered, waiting for points", reasons: reasons)
    }
}
