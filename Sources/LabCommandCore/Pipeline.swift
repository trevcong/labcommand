import Foundation

/// The pipeline audit: one check per hop of the signal chain, derived only
/// from what the collector measured. The honest-state rules apply — a probe
/// the collector didn't send is `unavailable`, never a pass. `pending` means
/// the check needs the sensors powered (parked is not a failure).
public struct AuditCheck: Sendable, Equatable, Identifiable {
    public enum Verdict: Sendable, Equatable {
        case pass
        /// Measured and wrong in a way that degrades data (loss counters
        /// climbing, low link speed). Amber, with the measured value shown.
        case watch
        /// Measured and broken (no carrier, PTP fault, container down).
        case fail
        /// Can only be judged with sensors powered; parked rigs sit here.
        case pending
        case unavailable
    }

    public var id: String
    /// Short chip label, e.g. "link".
    public var name: String
    /// The measured value, e.g. "1000/full".
    public var value: String
    public var verdict: Verdict

    public init(id: String, name: String, value: String, verdict: Verdict) {
        self.id = id
        self.name = name
        self.value = value
        self.verdict = verdict
    }
}

public enum PipelineAudit {
    public static func checks(_ s: RigSnapshot) -> [AuditCheck] {
        var out: [AuditCheck] = []
        let p = s.pipeline

        // Hop: switch → NIC. The carrier survives sensor unplugs (the switch
        // holds the link), so no-carrier is a real cable/switch failure.
        if let nic = p?.nic, let carrier = nic.carrier {
            if !carrier {
                out.append(AuditCheck(id: "link", name: "link", value: "no carrier", verdict: .fail))
            } else if let speed = nic.speedMbps, let duplex = nic.duplex {
                let good = speed >= 1000 && duplex == "full"
                out.append(AuditCheck(
                    id: "link", name: "link",
                    value: "\(speed)/\(duplex)",
                    verdict: good ? .pass : .watch
                ))
            } else {
                out.append(AuditCheck(id: "link", name: "link", value: "up", verdict: .pass))
            }
        } else {
            out.append(AuditCheck(id: "link", name: "link", value: TelemetryFormat.unavailable, verdict: .unavailable))
        }

        // Hop: NIC ingest. Lifetime totals are history; deltas are now.
        if let c = p?.counters, let macDelta = c.rxMacMissedDelta,
           let rcvDelta = c.udpRcvbufErrorsDelta, let inDelta = c.udpInErrorsDelta {
            let dropped = macDelta + rcvDelta + inDelta
            out.append(AuditCheck(
                id: "loss", name: "pkt loss",
                value: dropped == 0 ? "0" : "+\(dropped)",
                verdict: dropped == 0 ? .pass : .watch
            ))
        } else {
            out.append(AuditCheck(id: "loss", name: "pkt loss", value: TelemetryFormat.unavailable, verdict: .unavailable))
        }

        // Hop: driver sockets. Bound even while sensors are dark, so this
        // proves "plug in and points flow" before anything is powered.
        if let bound = p?.sockets.udpBound, !bound.isEmpty {
            let up = bound.values.filter { $0 }.count
            out.append(AuditCheck(
                id: "ingest", name: "driver ports",
                value: "\(up)/\(bound.count) bound",
                verdict: up == bound.count ? .pass : (s.stackRunning ? .fail : .pending)
            ))
        } else {
            out.append(AuditCheck(id: "ingest", name: "driver ports", value: TelemetryFormat.unavailable, verdict: .unavailable))
        }

        // Hop: time base.
        if !s.ptpMasterRunning {
            out.append(AuditCheck(id: "ptp", name: "ptp", value: "master down", verdict: .fail))
        } else if let role = p?.ptpRole {
            out.append(AuditCheck(
                id: "ptp", name: "ptp",
                value: role,
                verdict: role == "grandmaster" ? .pass : .fail
            ))
        } else {
            out.append(AuditCheck(id: "ptp", name: "ptp", value: "master up", verdict: .pass))
        }

        // Hop: sensor clocks vs host. Needs sensor data.
        switch s.clock.state {
        case .locked:
            let v = s.clock.deltaMs.map { "Δ \(TelemetryFormat.load($0)) ms" } ?? "locked"
            out.append(AuditCheck(id: "clocks", name: "clocks", value: v, verdict: .pass))
        case .converging:
            out.append(AuditCheck(id: "clocks", name: "clocks", value: "converging", verdict: .watch))
        case .noData:
            out.append(AuditCheck(id: "clocks", name: "clocks", value: "needs sensors", verdict: .pending))
        case .masterDown:
            out.append(AuditCheck(id: "clocks", name: "clocks", value: "master down", verdict: .fail))
        case .unknown:
            out.append(AuditCheck(id: "clocks", name: "clocks", value: TelemetryFormat.unavailable, verdict: .unavailable))
        }

        // Hop: containers.
        let wanted = ["lidar-dual", "ptp-master", "lidar-watchdog", "lidar-center"]
        let known = wanted.filter { s.containers[$0] != nil }
        if known.isEmpty {
            out.append(AuditCheck(id: "containers", name: "containers", value: TelemetryFormat.unavailable, verdict: .unavailable))
        } else {
            let running = known.filter { s.containers[$0] == "running" }.count
            out.append(AuditCheck(
                id: "containers", name: "containers",
                value: "\(running)/\(known.count)",
                verdict: running == known.count ? .pass : .fail
            ))
        }

        // Hop: end-to-end rates. The only proof the whole chain works.
        if s.bothStreaming {
            out.append(AuditCheck(id: "rates", name: "end-to-end", value: "both streaming", verdict: .pass))
        } else if s.anyStreaming {
            out.append(AuditCheck(id: "rates", name: "end-to-end", value: "one streaming", verdict: .watch))
        } else if s.bothDark {
            out.append(AuditCheck(id: "rates", name: "end-to-end", value: "parked", verdict: .pending))
        } else if s.mid360.state == .unknown && s.jt128.state == .unknown {
            out.append(AuditCheck(id: "rates", name: "end-to-end", value: TelemetryFormat.unavailable, verdict: .unavailable))
        } else {
            out.append(AuditCheck(id: "rates", name: "end-to-end", value: "powered, no points", verdict: .watch))
        }

        // Hop: storage.
        if let free = s.diskFreeGB {
            // ~2 GB/min dual: 100 GB is under an hour of recording.
            out.append(AuditCheck(
                id: "disk", name: "disk",
                value: free >= 1000 ? String(format: "%.1f T", Double(free) / 1000) : "\(free) GB",
                verdict: free > 100 ? .pass : .watch
            ))
        } else {
            out.append(AuditCheck(id: "disk", name: "disk", value: TelemetryFormat.unavailable, verdict: .unavailable))
        }

        return out
    }
}

/// Static spec sheets — the "crazy spec" layer. Sources: Livox/Hesai manuals,
/// DUAL-RIG-RESEARCH.md, and values measured on the Spark. These never mix
/// with live telemetry; a dossier line is a fact about the hardware, not a
/// reading.
public enum RigSpec {
    public struct Line: Sendable, Equatable {
        public var label: String
        public var value: String
        /// A caveat the operator must respect (safety, unpublished figure).
        public var caution: Bool

        public init(_ label: String, _ value: String, caution: Bool = false) {
            self.label = label
            self.value = value
            self.caution = caution
        }
    }

    /// Compact lines for the pipeline node cards (2–3 max).
    public static func nodeSummary(_ id: SensorID) -> [String] {
        switch id {
        case .mid360:
            return ["905 nm · 200k pts/s · 360°×[−7°,+52°]",
                    "\(id.ip) · UDP 56301/56401"]
        case .jt128:
            return ["905 nm · 1.15M pts/s · 360°×[−4°,+89°]",
                    "\(id.ip) · UDP 2368 · PTC 9347"]
        }
    }

    /// Full dossier for the Hardware page.
    public static func dossier(_ id: SensorID) -> [Line] {
        switch id {
        case .mid360:
            return [
                Line("Range", "40 m @10% · 70 m @80%"),
                Line("Rate", "200k pts/s · non-repetitive scan"),
                Line("FOV", "360° × −7°…+52°"),
                Line("Precision", "≤2 cm @10 m (1σ) · <0.15°"),
                Line("Laser", "905 nm · Class 1"),
                Line("IMU", "ICM40609 · 200 Hz · extrinsic published (11.0, 23.29, −44.12 mm)"),
                Line("Sync", "PTP 1588v2 UDP · slave · slews ~10 ms/min after power-up", caution: true),
                Line("Network", "100BASE-TX · 192.168.1.135 · UDP 56301/56401"),
                Line("Body", "265 g · 65×65×60 mm · IP67 · 9–27 V · 6.5 W"),
                Line("Care", "never point another 905 nm emitter into the aperture", caution: true),
            ]
        case .jt128:
            return [
                Line("Range", "40 m @10% · instrumented 60 m"),
                Line("Rate", "1.15M pts/s single return · 128 channels"),
                Line("FOV", "360° × −4.43°…+88.99° spatial"),
                Line("Precision", "unpublished · 4 mm range LSB", caution: true),
                Line("Laser", "905 nm · Class 1"),
                Line("IMU", "in-packet ~200 Hz · extrinsic unpublished → fuse on Mid-360's", caution: true),
                Line("Sync", "PTP 1588v2 default · steps instantly · ~60 µs jitter"),
                Line("Network", "100BASE-TX · 192.168.118.201 · UDP 2368 · TCP 9347 PTC"),
                Line("Body", "~250 g · Φ62.5×73 mm · 9–32 V · 9.5 W typ (≥30 W supply)"),
                Line("Care", "no fast power cycling — use PTC standby for heat", caution: true),
            ]
        }
    }

    public static let spark: [Line] = [
        Line("Chip", "GB10 Grace Blackwell · 20-core aarch64"),
        Line("Memory", "128 GB unified LPDDR5x"),
        Line("Storage", "3.7 T NVMe (~2 GB/min per dual take)"),
        Line("NIC", "enP7s7 · 1000BASE-T to the switch · both sensor subnets"),
        Line("Stack", "ROS 2 Jazzy in lidar-dual · foxglove :8765 · API :8766"),
        Line("Clock", "ptp4l software grandmaster · domain 0 · UDPv4 · E2E"),
        Line("Guards", "watchdog 5 s pings · restart unless-stopped · netplan pinned"),
        Line("Access", "ssh via Tailscale — flaky data plane, retry once", caution: true),
    ]
}

/// Live preview of the science gate, scored from the SAME probes the
/// collector's finalizer uses. This is advisory — the gate itself runs at
/// finalize on the Spark, and its verdict arrives as `lastGate`. A check the
/// app cannot score live says so instead of guessing.
public enum GatePreflight {
    public struct Check: Identifiable, Equatable, Sendable {
        public enum Status: Equatable, Sendable {
            /// The live probe currently satisfies the finalizer's rule.
            case holds
            /// The live probe currently violates it — the gate would fail now.
            case waiting
            /// Only the finalizer can score this (needs on-Spark state).
            case atFinalize
            /// The probe backing this check wasn't measured this poll.
            case unavailable
        }
        public var id: String
        public var name: String
        public var status: Status
        public var detail: String

        public init(id: String, name: String, status: Status, detail: String) {
            self.id = id
            self.name = name
            self.status = status
            self.detail = detail
        }
    }

    public static func checks(_ s: RigSnapshot) -> [Check] {
        var out: [Check] = []

        let mid = s.mid360.clockOffsetMs
        let jt = s.jt128.clockOffsetMs
        if let mid, let jt {
            let band = ClockSync.lockedBandMs
            let inBand = abs(mid) < band && abs(jt) < band
            out.append(Check(
                id: "clocks_in_band", name: "clocks in band",
                status: inBand ? .holds : .waiting,
                detail: String(format: "mid %+.1f · jt %+.1f · ±%.0f ms", mid, jt, band)))
        } else {
            out.append(Check(id: "clocks_in_band", name: "clocks in band",
                             status: .unavailable, detail: "no sensor stamps"))
        }

        let midHz = s.mid360.pointRateHz
        let jtHz = s.jt128.pointRateHz
        if let midHz, let jtHz {
            let flowing = midHz > RigStatusDecoder.streamingFloorHz
                && jtHz > RigStatusDecoder.streamingFloorHz
            out.append(Check(
                id: "both_streaming", name: "both streaming",
                status: flowing ? .holds : .waiting,
                detail: String(format: "mid %.1f Hz · jt %.1f Hz", midHz, jtHz)))
        } else {
            out.append(Check(id: "both_streaming", name: "both streaming",
                             status: .unavailable, detail: "rates unmeasured"))
        }

        if let c = s.pipeline?.counters,
           let mac = c.rxMacMissedDelta, let ine = c.udpInErrorsDelta,
           let buf = c.udpRcvbufErrorsDelta {
            let clean = mac == 0 && ine == 0 && buf == 0
            out.append(Check(
                id: "loss_deltas_zero", name: "loss deltas zero",
                status: clean ? .holds : .waiting,
                detail: "Δ\(mac) mac · Δ\(ine) InErrors · Δ\(buf) Rcvbuf"))
        } else {
            out.append(Check(id: "loss_deltas_zero", name: "loss deltas zero",
                             status: .unavailable, detail: "counters unmeasured"))
        }

        out.append(Check(
            id: "calibration", name: "calibration",
            status: .atFinalize,
            detail: "signed calibration.json on the Spark — scored at finalize"))
        out.append(Check(
            id: "hashes", name: "hashes",
            status: .atFinalize,
            detail: "bag · angle csv · ptp4l log — computed at finalize"))
        return out
    }
}
