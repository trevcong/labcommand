import Foundation

/// v1.0 dual rig. The Mac binds to the Spark's command-center API
/// (`command_center.py`, :8766 over the SSH tunnel) — one collector, one
/// truth, shared with the web panel. The honest-state contract from v0.4/v0.5
/// carries forward: a missing key is `nil`, a measured zero is a real zero,
/// and a dark sensor is a calm first-class state, not an error.

/// The two lidars on the rig.
public enum SensorID: String, Sendable, Equatable, CaseIterable {
    case mid360
    case jt128

    public var label: String {
        switch self {
        case .mid360: return "Mid-360"
        case .jt128: return "JT128"
        }
    }

    public var ip: String {
        switch self {
        case .mid360: return "192.168.1.135"
        case .jt128: return "192.168.118.201"
        }
    }
}

/// Power/data state of one sensor, derived from ping + measured topic rate.
public enum SensorState: String, Sendable, Equatable {
    /// No ping answer: unpowered or unplugged. Deliberate parking is normal.
    case dark
    /// Ping answers but the topic rate measured ~0: standby, driver down, or
    /// still booting. The detail line says which is knowable.
    case powered
    /// Points are arriving at a live rate.
    case streaming
    /// Not probed (Spark or the API unreachable).
    case unknown

    public var displayLabel: String {
        switch self {
        case .dark: return "Dark"
        case .powered: return "Powered"
        case .streaming: return "Live"
        case .unknown: return "—"
        }
    }
}

/// One sensor as the panel renders it. Rates are measured by the collector
/// over a 5 s window; `nil` means the key was absent, not zero.
public struct SensorReading: Sendable, Equatable {
    public var id: SensorID
    public var state: SensorState
    public var pointRateHz: Double?
    public var imuRateHz: Double?
    /// Host-vs-sensor clock offset (ms) from IMU stamps; the PTP figure.
    public var clockOffsetMs: Double?
    /// JT128 only: PTC control port answering.
    public var ptcOpen: Bool?

    public init(
        id: SensorID,
        state: SensorState = .unknown,
        pointRateHz: Double? = nil,
        imuRateHz: Double? = nil,
        clockOffsetMs: Double? = nil,
        ptcOpen: Bool? = nil
    ) {
        self.id = id
        self.state = state
        self.pointRateHz = pointRateHz
        self.imuRateHz = imuRateHz
        self.clockOffsetMs = clockOffsetMs
        self.ptcOpen = ptcOpen
    }

    public var detail: String {
        switch state {
        case .dark:
            return "unpowered — plug in and the watchdog restarts the stack"
        case .powered:
            if id == .jt128 {
                return "answering, no points — standby or driver down"
            }
            return "answering, no points — driver down or starting"
        case .streaming:
            return "\(TelemetryFormat.hz(pointRateHz)) · imu \(TelemetryFormat.hz(imuRateHz))"
        case .unknown:
            return "not probed"
        }
    }
}

/// PTP picture: the master container plus what the sensor stamps actually say.
public enum ClockSyncState: String, Sendable, Equatable {
    case masterDown
    /// Master up, no sensor data to judge against.
    case noData
    /// Offsets measured but at least one is still large (Mid-360 slews slowly).
    case converging
    case locked
    case unknown

    public var displayLabel: String {
        switch self {
        case .masterDown: return "Master down"
        case .noData: return "No data"
        case .converging: return "Converging"
        case .locked: return "Locked"
        case .unknown: return "—"
        }
    }
}

public struct ClockSync: Sendable, Equatable {
    public var masterUp: Bool
    public var state: ClockSyncState
    /// |mid − jt| in ms when both offsets were measured.
    public var deltaMs: Double?

    public init(masterUp: Bool = false, state: ClockSyncState = .unknown, deltaMs: Double? = nil) {
        self.masterUp = masterUp
        self.state = state
        self.deltaMs = deltaMs
    }

    /// Offsets within this band count as locked. Software PTP gives tens of
    /// microseconds; 50 ms is far above lock and far below the Mid-360's
    /// pre-convergence slew, so the band never flatters.
    public static let lockedBandMs = 50.0
}

public struct RecordingStatus: Sendable, Equatable {
    public var active: Bool
    public var session: String?
    public var sizeGB: Double?

    public init(active: Bool = false, session: String? = nil, sizeGB: Double? = nil) {
        self.active = active
        self.session = session
        self.sizeGB = sizeGB
    }
}

/// Per-hop pipeline probes from the collector's `pipeline` block. Every
/// field is optional: an old collector (or a failed probe) yields nil, and
/// nil renders as "unavailable" — never as a passing check.
public struct NicStatus: Sendable, Equatable {
    public var speedMbps: Int?
    public var duplex: String?
    public var carrier: Bool?
    public var mtu: Int?
    public var rmemMax: Int?

    public init(speedMbps: Int? = nil, duplex: String? = nil, carrier: Bool? = nil,
                mtu: Int? = nil, rmemMax: Int? = nil) {
        self.speedMbps = speedMbps
        self.duplex = duplex
        self.carrier = carrier
        self.mtu = mtu
        self.rmemMax = rmemMax
    }
}

/// Loss counters. Absolute values are lifetime totals (the Spark's NIC has
/// 1.03M historical `rx_mac_missed` from before the buffers were tuned);
/// only the deltas since the last poll say anything about *now*.
public struct PipelineCounters: Sendable, Equatable {
    public var udpInErrors: Int?
    public var udpRcvbufErrors: Int?
    public var rxMacMissed: Int?
    public var udpInErrorsDelta: Int?
    public var udpRcvbufErrorsDelta: Int?
    public var rxMacMissedDelta: Int?

    public init(udpInErrors: Int? = nil, udpRcvbufErrors: Int? = nil, rxMacMissed: Int? = nil,
                udpInErrorsDelta: Int? = nil, udpRcvbufErrorsDelta: Int? = nil,
                rxMacMissedDelta: Int? = nil) {
        self.udpInErrors = udpInErrors
        self.udpRcvbufErrors = udpRcvbufErrors
        self.rxMacMissed = rxMacMissed
        self.udpInErrorsDelta = udpInErrorsDelta
        self.udpRcvbufErrorsDelta = udpRcvbufErrorsDelta
        self.rxMacMissedDelta = rxMacMissedDelta
    }
}

public struct PipelineSockets: Sendable, Equatable {
    /// Driver ingest ports ("56301"/"56401" Livox, "2368" Hesai) → bound.
    /// Bound while the sensors are dark proves the drivers are ready.
    public var udpBound: [String: Bool]
    /// ESTABLISHED TCP counts on "8765" (Foxglove) / "8766" (this API).
    public var established: [String: Int]

    public init(udpBound: [String: Bool] = [:], established: [String: Int] = [:]) {
        self.udpBound = udpBound
        self.established = established
    }
}

public struct PipelineStatus: Sendable, Equatable {
    public var nic: NicStatus
    public var counters: PipelineCounters
    public var sockets: PipelineSockets
    /// "grandmaster" / "fault" from ptp4l's log, nil if no role line seen.
    public var ptpRole: String?

    public init(nic: NicStatus = NicStatus(), counters: PipelineCounters = PipelineCounters(),
                sockets: PipelineSockets = PipelineSockets(), ptpRole: String? = nil) {
        self.nic = nic
        self.counters = counters
        self.sockets = sockets
        self.ptpRole = ptpRole
    }
}

/// A server-side armed take: the collector itself waits for sustained clock
/// lock + streaming, then records and finalizes with no client in the loop.
/// Mirrors `armed` in status.json; nil when nothing is armed (or the
/// collector predates the feature — absent renders as nothing, never a state).
public struct ArmedStatus: Sendable, Equatable {
    public var name: String
    public var protocolID: String?
    public var poseSource: String?
    public var durationS: Int?
    /// Consecutive in-band+streaming samples seen; fires at 3.
    public var lockStreak: Int
    /// "waiting_for_lock" | "recording" (collector-owned vocabulary).
    public var state: String

    public init(name: String, protocolID: String? = nil, poseSource: String? = nil,
                durationS: Int? = nil, lockStreak: Int = 0,
                state: String = "waiting_for_lock") {
        self.name = name
        self.protocolID = protocolID
        self.poseSource = poseSource
        self.durationS = durationS
        self.lockStreak = lockStreak
        self.state = state
    }
}

/// Science-gate verdict of the most recent finalize, straight from the
/// collector's status.json. The verdict is the collector's — the app never
/// recomputes or embellishes it.
public struct LastGate: Sendable, Equatable {
    public var takeID: String
    public var protocolID: String?
    public var passed: Bool
    public var failedChecks: [String]
    public var stoppedUTC: String?
    public var durationS: Double?

    public init(takeID: String, protocolID: String? = nil, passed: Bool = false,
                failedChecks: [String] = [], stoppedUTC: String? = nil,
                durationS: Double? = nil) {
        self.takeID = takeID
        self.protocolID = protocolID
        self.passed = passed
        self.failedChecks = failedChecks
        self.stoppedUTC = stoppedUTC
        self.durationS = durationS
    }
}

/// The whole rig as one bindable snapshot.
public struct RigSnapshot: Sendable, Equatable {
    public var sparkReachable: Bool
    /// The :8766 API answered. False with `sparkReachable` true means the
    /// lidar-center container is down — its own diagnosis.
    public var apiReachable: Bool
    public var mid360: SensorReading
    public var jt128: SensorReading
    public var clock: ClockSync
    /// Container name → docker state ("running", "exited", …).
    public var containers: [String: String]
    public var recording: RecordingStatus
    public var diskFreeGB: Int?
    public var load1: Double?
    public var memPct: Double?
    /// Watchdog log lines, oldest first.
    public var events: [String]
    public var viewTunnelUp: Bool
    public var refreshedAt: Date
    public var blockReason: String?
    /// Per-hop probes; nil when the collector predates them or the API is down.
    public var pipeline: PipelineStatus?
    /// Armed take the collector is holding; nil when nothing is armed.
    public var armed: ArmedStatus?
    /// Gate verdict of the last finalize; nil before any take this boot.
    public var lastGate: LastGate?

    public init(
        sparkReachable: Bool = false,
        apiReachable: Bool = false,
        mid360: SensorReading = SensorReading(id: .mid360),
        jt128: SensorReading = SensorReading(id: .jt128),
        clock: ClockSync = ClockSync(),
        containers: [String: String] = [:],
        recording: RecordingStatus = RecordingStatus(),
        diskFreeGB: Int? = nil,
        load1: Double? = nil,
        memPct: Double? = nil,
        events: [String] = [],
        viewTunnelUp: Bool = false,
        refreshedAt: Date = Date(),
        blockReason: String? = nil,
        pipeline: PipelineStatus? = nil,
        armed: ArmedStatus? = nil,
        lastGate: LastGate? = nil
    ) {
        self.sparkReachable = sparkReachable
        self.apiReachable = apiReachable
        self.mid360 = mid360
        self.jt128 = jt128
        self.clock = clock
        self.containers = containers
        self.recording = recording
        self.diskFreeGB = diskFreeGB
        self.load1 = load1
        self.memPct = memPct
        self.events = events
        self.viewTunnelUp = viewTunnelUp
        self.refreshedAt = refreshedAt
        self.blockReason = blockReason
        self.pipeline = pipeline
        self.armed = armed
        self.lastGate = lastGate
    }

    public static func offline(reason: String, now: Date = Date()) -> RigSnapshot {
        RigSnapshot(sparkReachable: false, refreshedAt: now, blockReason: reason)
    }

    public static func apiDown(reason: String, viewTunnelUp: Bool = false, now: Date = Date()) -> RigSnapshot {
        RigSnapshot(
            sparkReachable: true,
            apiReachable: false,
            viewTunnelUp: viewTunnelUp,
            refreshedAt: now,
            blockReason: reason
        )
    }

    public var stackRunning: Bool { containers["lidar-dual"] == "running" }
    public var watchdogRunning: Bool { containers["lidar-watchdog"] == "running" }
    public var ptpMasterRunning: Bool { containers["ptp-master"] == "running" }

    public func sensor(_ id: SensorID) -> SensorReading {
        id == .mid360 ? mid360 : jt128
    }

    public var bothDark: Bool { mid360.state == .dark && jt128.state == .dark }
    public var anyStreaming: Bool { mid360.state == .streaming || jt128.state == .streaming }
    public var bothStreaming: Bool { mid360.state == .streaming && jt128.state == .streaming }

    public var anythingRunning: Bool {
        stackRunning || recording.active || anyStreaming || viewTunnelUp
    }
}

/// Decode `status.json` from the command-center API. Absent keys stay nil;
/// no field is ever invented. Malformed JSON returns nil so the caller can
/// keep the previous honest snapshot.
public enum RigStatusDecoder {
    /// A topic below this rate is not "streaming" — the collector's 5 s
    /// window makes real streams read 8–10 Hz.
    public static let streamingFloorHz = 1.0

    public static func parse(
        json: String,
        viewTunnelUp: Bool,
        now: Date = Date()
    ) -> RigSnapshot? {
        guard let data = json.data(using: .utf8),
              let root = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any] else {
            return nil
        }

        let rates = root["rates"] as? [String: Any] ?? [:]
        let offsets = root["offsets"] as? [String: Any] ?? [:]

        func rate(_ key: String) -> Double? { rates[key] as? Double }
        func offset(_ key: String) -> Double? { offsets[key] as? Double }

        func sensorState(ping: Bool?, pts: Double?) -> SensorState {
            guard let ping else { return .unknown }
            if !ping { return .dark }
            if let pts, pts > streamingFloorHz { return .streaming }
            return .powered
        }

        let midPing = root["mid360_ping"] as? Bool
        let jtPing = root["jt128_ping"] as? Bool

        let mid = SensorReading(
            id: .mid360,
            state: sensorState(ping: midPing, pts: rate("livox_pts")),
            pointRateHz: rate("livox_pts"),
            imuRateHz: rate("livox_imu"),
            clockOffsetMs: offset("livox_ms")
        )
        let jt = SensorReading(
            id: .jt128,
            state: sensorState(ping: jtPing, pts: rate("jt_pts")),
            pointRateHz: rate("jt_pts"),
            imuRateHz: rate("jt_imu"),
            clockOffsetMs: offset("jt_ms"),
            ptcOpen: root["jt128_ptc"] as? Bool
        )

        let containers = (root["containers"] as? [String: String]) ?? [:]

        let clock: ClockSync = {
            let masterUp = containers["ptp-master"] == "running"
            guard masterUp else { return ClockSync(masterUp: false, state: .masterDown) }
            guard let m = mid.clockOffsetMs, let j = jt.clockOffsetMs else {
                // One sensor's offset alone can still prove lock for that sensor,
                // but the rig-level figure needs both.
                if let single = mid.clockOffsetMs ?? jt.clockOffsetMs {
                    let locked = abs(single) < ClockSync.lockedBandMs
                    return ClockSync(masterUp: true, state: locked ? .locked : .converging)
                }
                return ClockSync(masterUp: true, state: .noData)
            }
            let locked = abs(m) < ClockSync.lockedBandMs && abs(j) < ClockSync.lockedBandMs
            return ClockSync(
                masterUp: true,
                state: locked ? .locked : .converging,
                deltaMs: (abs(m - j) * 100).rounded() / 100
            )
        }()

        let recording: RecordingStatus = {
            guard let r = root["recording"] as? [String: Any] else { return RecordingStatus() }
            return RecordingStatus(
                active: r["active"] as? Bool ?? false,
                session: r["session"] as? String,
                sizeGB: r["size_gb"] as? Double
            )
        }()

        let pipeline: PipelineStatus? = {
            guard let p = root["pipeline"] as? [String: Any] else { return nil }
            let nicD = p["nic"] as? [String: Any] ?? [:]
            let nic = NicStatus(
                speedMbps: nicD["speed_mbps"] as? Int,
                duplex: nicD["duplex"] as? String,
                carrier: nicD["carrier"] as? Bool,
                mtu: nicD["mtu"] as? Int,
                rmemMax: nicD["rmem_max"] as? Int
            )
            let cD = p["counters"] as? [String: Any] ?? [:]
            let counters = PipelineCounters(
                udpInErrors: cD["udp_in_errors"] as? Int,
                udpRcvbufErrors: cD["udp_rcvbuf_errors"] as? Int,
                rxMacMissed: cD["rx_mac_missed"] as? Int,
                udpInErrorsDelta: cD["udp_in_errors_delta"] as? Int,
                udpRcvbufErrorsDelta: cD["udp_rcvbuf_errors_delta"] as? Int,
                rxMacMissedDelta: cD["rx_mac_missed_delta"] as? Int
            )
            let sD = p["sockets"] as? [String: Any] ?? [:]
            let sockets = PipelineSockets(
                udpBound: sD["udp_bound"] as? [String: Bool] ?? [:],
                established: sD["established"] as? [String: Int] ?? [:]
            )
            return PipelineStatus(nic: nic, counters: counters, sockets: sockets,
                                  ptpRole: p["ptp_role"] as? String)
        }()

        let lastGate: LastGate? = {
            guard let g = root["last_gate"] as? [String: Any],
                  let id = g["take_id"] as? String else { return nil }
            return LastGate(
                takeID: id,
                protocolID: g["protocol_id"] as? String,
                passed: g["passed"] as? Bool ?? false,
                failedChecks: g["failed_checks"] as? [String] ?? [],
                stoppedUTC: g["stopped_utc"] as? String,
                durationS: g["duration_s"] as? Double
            )
        }()

        let armed: ArmedStatus? = {
            guard let a = root["armed"] as? [String: Any],
                  let nm = a["name"] as? String else { return nil }
            return ArmedStatus(
                name: nm,
                protocolID: a["protocol"] as? String,
                poseSource: a["pose_source"] as? String,
                durationS: a["duration_s"] as? Int,
                lockStreak: a["lock_streak"] as? Int ?? 0,
                state: a["state"] as? String ?? "waiting_for_lock"
            )
        }()

        return RigSnapshot(
            sparkReachable: true,
            apiReachable: true,
            mid360: mid,
            jt128: jt,
            clock: clock,
            containers: containers,
            recording: recording,
            diskFreeGB: root["disk_free_gb"] as? Int,
            load1: root["load1"] as? Double,
            memPct: root["mem_pct"] as? Double,
            events: (root["events"] as? [String]) ?? [],
            viewTunnelUp: viewTunnelUp,
            refreshedAt: now,
            pipeline: pipeline,
            armed: armed,
            lastGate: lastGate
        )
    }
}
