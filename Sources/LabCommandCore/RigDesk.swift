import Foundation

/// One stage of the dual-rig path: Mac → Spark → drivers → clocks → preview → bag.
public enum RigStage: String, Sendable, Equatable, CaseIterable {
    case link
    case stack
    case clock
    case view
    case record

    public var label: String {
        switch self {
        case .link: return "Link"
        case .stack: return "Drivers"
        case .clock: return "Clocks"
        case .view: return "View"
        case .record: return "Record"
        }
    }
}

/// Coarse per-stage verb, reusing the v0.4 vocabulary.
public enum RigStageState: String, Sendable, Equatable {
    case idle
    case running
    case blocked
    case unknown

    public var displayLabel: String {
        switch self {
        case .idle: return "Idle"
        case .running: return "Live"
        case .blocked: return "Blocked"
        case .unknown: return "—"
        }
    }
}

public enum RigAction: String, Sendable, Equatable, CaseIterable, Hashable {
    case startRig
    case startRecording
    case stop
}

public struct RigButtonSpec: Sendable, Equatable {
    public var action: RigAction
    public var title: String
    public var whatItDoes: String
    public var shortcut: String
    public var enabled: Bool
    public var disabledReason: String?
    public var isPrimary: Bool

    public init(
        action: RigAction,
        title: String,
        whatItDoes: String,
        shortcut: String,
        enabled: Bool,
        disabledReason: String?,
        isPrimary: Bool
    ) {
        self.action = action
        self.title = title
        self.whatItDoes = whatItDoes
        self.shortcut = shortcut
        self.enabled = enabled
        self.disabledReason = disabledReason
        self.isPrimary = isPrimary
    }
}

/// Operator copy + transport specs for the Rig page. Views bind; they do not
/// invent gates.
public enum RigDesk {
    public static let title = "Rig"

    public static let purpose =
        "This window runs the dual-lidar rig (Livox Mid-360 + Hesai JT128) on the DGX Spark. The Mac never talks to a lidar — it reads the Spark's command-center API over an SSH tunnel, starts and stops the shared driver stack, records dual bags, and shows the takes on this Mac."

    public static let capabilities: [String] = [
        "See both sensors honestly: Dark (unplugged is normal), Powered, or Live with rates",
        "Start the dual driver stack + preview tunnel in one press",
        "Record all four topics (both point clouds, both IMUs) into one bag",
        "Put the JT128 in standby instead of unplugging it for heat",
        "Watch PTP clock lock — Mid-360 slews for minutes after power-up",
        "Plug a sensor in and walk away: the Spark watchdog restarts the drivers",
    ]

    public static func stageMeaning(_ stage: RigStage) -> String {
        switch stage {
        case .link:
            return "Tailscale/SSH to the Spark plus the :8766 command-center tunnel. Sensors have no route from this Mac."
        case .stack:
            return "lidar-dual container on the Spark: both drivers + one foxglove bridge."
        case .clock:
            return "ptp4l software master on the Spark; both sensors are PTP slaves. Locked means bags carry one timebase."
        case .view:
            return "Foxglove tunnel at ws://localhost:8765. Display only."
        case .record:
            return "ros2 bag of /livox/lidar /livox/imu /lidar_points /lidar_imu on the Spark (~2 GB/min when both stream)."
        }
    }

    public static func stageState(_ stage: RigStage, _ s: RigSnapshot) -> RigStageState {
        switch stage {
        case .link:
            if !s.sparkReachable { return .blocked }
            return s.apiReachable ? .running : .blocked
        case .stack:
            guard s.apiReachable else { return .unknown }
            return s.stackRunning ? .running : .idle
        case .clock:
            guard s.apiReachable else { return .unknown }
            switch s.clock.state {
            case .masterDown: return .blocked
            case .locked: return .running
            case .converging, .noData: return .idle
            case .unknown: return .unknown
            }
        case .view:
            return s.viewTunnelUp ? .running : .idle
        case .record:
            guard s.apiReachable else { return .unknown }
            return s.recording.active ? .running : .idle
        }
    }

    public static func stageDetail(_ stage: RigStage, _ s: RigSnapshot) -> String {
        switch stage {
        case .link:
            if !s.sparkReachable { return s.blockReason ?? "Spark unreachable" }
            if !s.apiReachable { return s.blockReason ?? "API not answering" }
            var bits = ["spark + :8766"]
            if let load = s.load1 { bits.append("load \(TelemetryFormat.load(load))") }
            if let mem = s.memPct { bits.append("mem \(TelemetryFormat.percent(mem))") }
            return bits.joined(separator: " · ")
        case .stack:
            guard s.apiReachable else { return "not probed" }
            if s.stackRunning {
                return s.watchdogRunning ? "lidar-dual up · watchdog armed" : "lidar-dual up · watchdog DOWN"
            }
            return "lidar-dual \(s.containers["lidar-dual"] ?? "missing")"
        case .clock:
            guard s.apiReachable else { return "not probed" }
            switch s.clock.state {
            case .masterDown: return "ptp-master container down"
            case .noData: return "master up · no sensor stamps to judge"
            case .locked:
                if let d = s.clock.deltaMs { return "Δsensors \(TelemetryFormat.load(d)) ms" }
                return "offset inside \(Int(ClockSync.lockedBandMs)) ms"
            case .converging:
                let m = s.mid360.clockOffsetMs.map { TelemetryFormat.load($0) } ?? TelemetryFormat.unavailable
                let j = s.jt128.clockOffsetMs.map { TelemetryFormat.load($0) } ?? TelemetryFormat.unavailable
                return "vs host: mid \(m) ms · jt \(j) ms"
            case .unknown: return "not probed"
            }
        case .view:
            return s.viewTunnelUp ? "ws://localhost:8765" : "tunnel off"
        case .record:
            guard s.apiReachable else { return "not probed" }
            if s.recording.active {
                var bits: [String] = []
                if let session = s.recording.session { bits.append(session) }
                if let gb = s.recording.sizeGB { bits.append(String(format: "%.1f GB", gb)) }
                return bits.isEmpty ? "recording" : bits.joined(separator: " · ")
            }
            if let free = s.diskFreeGB { return "idle · \(free) GB free" }
            return "idle"
        }
    }

    public static func primaryAction(for s: RigSnapshot) -> RigAction {
        if s.recording.active { return .stop }
        if s.stackRunning && s.anyStreaming { return .startRecording }
        return .startRig
    }

    public static func buttons(for s: RigSnapshot) -> [RigButtonSpec] {
        let start = RigGate.canStartRig(s)
        let record = RigGate.canRecord(s)
        let stop = RigGate.canStopAll(s)
        let primary = primaryAction(for: s)
        return [
            RigButtonSpec(
                action: .startRig,
                title: s.stackRunning ? "Restart rig" : "Start rig",
                whatItDoes: s.stackRunning
                    ? "Restarts both drivers and reopens the preview. Use after re-plugging a sensor by hand."
                    : "Starts both drivers on the Spark and opens the preview tunnels.",
                shortcut: "⌘↩",
                enabled: start.0,
                disabledReason: start.1,
                isPrimary: primary == .startRig
            ),
            RigButtonSpec(
                action: .startRecording,
                title: "Start recording",
                whatItDoes: "Writes one bag with both point clouds and both IMUs. Walk as soon as this is live.",
                shortcut: "⌘⇧R",
                enabled: record.0,
                disabledReason: record.1,
                isPrimary: primary == .startRecording
            ),
            RigButtonSpec(
                action: .stop,
                title: s.recording.active ? "Stop recording" : "Stop rig",
                whatItDoes: s.recording.active
                    ? "Ends this take cleanly (SIGINT, then chown). The sensors stay live for the next walk."
                    : "Stops the driver stack with a 60 s grace and closes the tunnels.",
                shortcut: "⌘.",
                enabled: stop.0,
                disabledReason: stop.1,
                isPrimary: primary == .stop
            ),
        ]
    }

    public static func statusHeadline(for s: RigSnapshot) -> String {
        RigHealth.evaluate(s).headline
    }

    public static func sessionFolderPreview(_ spec: CaptureSessionSpec, now: Date = Date()) -> String {
        // Dual sessions carry the DUAL marker so the library can tell them
        // from single-sensor A/A2/B/C walks.
        let f = DateFormatter()
        f.locale = Locale(identifier: "en_US_POSIX")
        f.timeZone = TimeZone(secondsFromGMT: 0)
        f.dateFormat = "yyyyMMdd'T'HHmmss'Z'"
        return "\(f.string(from: now))_DUAL_\(spec.preset.code)_\(spec.layoutTag)"
    }

    /// The record-action argument: the API prefixes timestamp + DUAL itself.
    public static func recordName(_ spec: CaptureSessionSpec) -> String {
        "\(spec.preset.code)_\(spec.layoutTag)"
    }
}

/// One recommended action per blocked spot, replacing v0.5's CaptureDiagnose.
public struct RigAdvice: Sendable, Equatable {
    public var stage: RigStage
    public var reason: String
    public var recommendedAction: String
    public var recoveryHint: String

    public init(stage: RigStage, reason: String, recommendedAction: String, recoveryHint: String) {
        self.stage = stage
        self.reason = reason
        self.recommendedAction = recommendedAction
        self.recoveryHint = recoveryHint
    }

    public var summary: String { "\(stage.label): \(reason)" }
}

public enum RigDiagnose {
    public static func advice(for s: RigSnapshot, stage: RigStage? = nil) -> RigAdvice {
        if let stage { return adviceFor(stage, s) }
        if !s.sparkReachable { return adviceFor(.link, s) }
        if !s.apiReachable { return adviceFor(.link, s) }
        if s.bothDark { return adviceFor(.stack, s) }
        if !s.stackRunning { return adviceFor(.stack, s) }
        if !s.anyStreaming { return adviceFor(.stack, s) }
        if s.clock.state != .locked { return adviceFor(.clock, s) }
        return adviceFor(.record, s)
    }

    private static func adviceFor(_ stage: RigStage, _ s: RigSnapshot) -> RigAdvice {
        switch stage {
        case .link:
            if !s.sparkReachable {
                return RigAdvice(
                    stage: .link,
                    reason: "No SSH path to the Spark",
                    recommendedAction: "Check Tailscale on this Mac (the app auto-starts it) and on the Spark.",
                    recoveryHint: "The Spark can be powered and still unreachable: its wired port is sensors-only, so Tailscale needs Spark WiFi."
                )
            }
            return RigAdvice(
                stage: .link,
                reason: "Command-center API not answering",
                recommendedAction: "On the Spark: bash ~/hesai_ws/center_up.sh — or reopen the :8766 tunnel with Refresh.",
                recoveryHint: "The lidar-center container serves status for this app and the web panel."
            )
        case .stack:
            if s.bothDark {
                return RigAdvice(
                    stage: .stack,
                    reason: "Both sensors are dark",
                    recommendedAction: "Plug in sensor power. The Spark watchdog restarts the drivers on its own.",
                    recoveryHint: "Parking the rig unpowered is normal; use JT128 standby next time to skip the re-plug."
                )
            }
            if !s.stackRunning {
                return RigAdvice(
                    stage: .stack,
                    reason: "Driver stack is stopped",
                    recommendedAction: "Press Start rig.",
                    recoveryHint: "lidar-dual runs both drivers and the foxglove bridge in one container."
                )
            }
            return RigAdvice(
                stage: .stack,
                reason: "Drivers up but a powered sensor sends no points",
                recommendedAction: "Press Restart rig — the driver re-handshake fixes a stalled sensor join.",
                recoveryHint: "A JT128 in standby also reads Powered with no points; Wake it from the sensor card."
            )
        case .clock:
            if s.clock.state == .masterDown {
                return RigAdvice(
                    stage: .clock,
                    reason: "PTP master container is down",
                    recommendedAction: "On the Spark: bash ~/hesai_ws/ptp_up.sh (or lidar ptp from a terminal).",
                    recoveryHint: "Without the master both sensors free-run and fused clouds lose their shared timebase."
                )
            }
            return RigAdvice(
                stage: .clock,
                reason: "Clocks not locked yet",
                recommendedAction: "Wait — the Mid-360 slews ~10 ms per minute after power-up. The JT128 steps instantly.",
                recoveryHint: "Power-cycling the Mid-360 after the master is up makes it step instead of slew."
            )
        case .view:
            return RigAdvice(
                stage: .view,
                reason: "Preview tunnel is closed",
                recommendedAction: "Press Start rig (it opens the tunnel), then connect Foxglove to ws://localhost:8765.",
                recoveryHint: "3D panel: display frame `world`, enable /livox/lidar and /lidar_points."
            )
        case .record:
            return RigAdvice(
                stage: .record,
                reason: s.recording.active ? "Recording" : "Not recording",
                recommendedAction: s.recording.active
                    ? "Stop ends the take cleanly; bags land in ~/lidar-sessions on the Spark."
                    : "Press Start recording once a sensor is live.",
                recoveryHint: "Dual bags cost ~2 GB/min. Long research captures should use the pcap path instead."
            )
        }
    }
}
