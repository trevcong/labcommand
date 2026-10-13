import SwiftUI
import LabCommandCore

/// The signal chain, drawn. Sensors on the left, the Spark in the middle,
/// what comes out on the right; every wire is a real connection hop labeled
/// with its actual protocol and port. Wire color follows measured state:
/// green when data flows, steel for control paths, dashed idle when a sensor
/// is parked. The audit strip beneath re-answers "is the pipeline perfect?"
/// from the collector's per-hop probes on every poll.
struct PipelineGraph: View {
    @EnvironmentObject private var state: AppState
    var breath: Bool

    private var rig: RigSnapshot { state.rig }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(alignment: .center, spacing: 0) {
                VStack(spacing: 10) {
                    sensorNode(rig.mid360)
                    sensorNode(rig.jt128)
                }
                .frame(maxWidth: .infinity)
                ingestWires
                    .frame(width: 112)
                sparkNode
                    .frame(maxWidth: .infinity)
                topicWires
                    .frame(width: 112)
                VStack(spacing: 10) {
                    driversNode
                    viewNode
                    bagNode
                }
                .frame(maxWidth: .infinity)
            }
            .animation(.easeOut(duration: LabTheme.motion), value: rig.mid360.state)
            .animation(.easeOut(duration: LabTheme.motion), value: rig.jt128.state)
            auditStrip
        }
        .accessibilityIdentifier("pipelineGraph")
    }

    // MARK: Wires

    private func sensorFlow(_ s: SensorReading) -> (Color, Bool) {
        switch s.state {
        case .streaming: return (LabTheme.good, false)
        case .powered: return (LabTheme.warn, false)
        case .dark: return (LabTheme.idle, true)
        case .unknown: return (LabTheme.unknown, true)
        }
    }

    private var ingestWires: some View {
        let mid = sensorFlow(rig.mid360)
        let jt = sensorFlow(rig.jt128)
        let ptp = rig.ptpMasterRunning ? LabTheme.accent : LabTheme.bad
        let ptc = (rig.jt128.ptcOpen == true) ? LabTheme.accent : LabTheme.idle
        return VStack(spacing: 13) {
            Wire(label: "udp 56301/401", color: mid.0, dashed: mid.1,
                 flow: rig.mid360.state == .streaming)
            Wire(label: "ptp 1588v2", color: ptp, dashed: true, direction: .back)
            Wire(label: "udp 2368", color: jt.0, dashed: jt.1,
                 flow: rig.jt128.state == .streaming)
            Wire(label: "ptc 9347", color: ptc, dashed: true, direction: .back)
        }
        .padding(.horizontal, 6)
    }

    private var topicWires: some View {
        let midLive = (rig.mid360.pointRateHz ?? 0) > RigStatusDecoder.streamingFloorHz
        let jtLive = (rig.jt128.pointRateHz ?? 0) > RigStatusDecoder.streamingFloorHz
        let imuLive = (rig.mid360.imuRateHz ?? 0) > 1 || (rig.jt128.imuRateHz ?? 0) > 1
        return VStack(spacing: 13) {
            Wire(label: "/livox/lidar", color: midLive ? LabTheme.good : LabTheme.idle,
                 dashed: !midLive, flow: midLive)
            Wire(label: "/lidar_points", color: jtLive ? LabTheme.good : LabTheme.idle,
                 dashed: !jtLive, flow: jtLive)
            Wire(label: "both imus", color: imuLive ? LabTheme.accent : LabTheme.idle,
                 dashed: !imuLive, flow: imuLive)
        }
        .padding(.horizontal, 6)
    }

    // MARK: Nodes

    private func sensorColor(_ s: SensorState) -> Color {
        switch s {
        case .streaming: return LabTheme.good
        case .powered: return LabTheme.warn
        case .dark: return LabTheme.idle
        case .unknown: return LabTheme.unknown
        }
    }

    private func sensorNode(_ sensor: SensorReading) -> some View {
        NodeCard(
            title: sensor.id.label,
            pip: sensorColor(sensor.state),
            breathe: sensor.state == .streaming ? breath : nil,
            verb: sensor.state.displayLabel,
            verbColor: sensor.state == .streaming ? LabTheme.good : LabTheme.dim,
            hot: sensor.state == .streaming
        ) {
            if sensor.state == .streaming {
                HStack(alignment: .firstTextBaseline, spacing: 8) {
                    Text(TelemetryFormat.hz(sensor.pointRateHz))
                        .font(LabTheme.t(13, .semibold))
                        .monospacedDigit()
                        .foregroundStyle(LabTheme.text)
                        .contentTransition(.numericText())
                        .animation(.easeOut(duration: LabTheme.motion), value: sensor.pointRateHz)
                    Text("imu \(TelemetryFormat.hz(sensor.imuRateHz))")
                        .font(LabTheme.mono(11))
                        .monospacedDigit()
                        .foregroundStyle(LabTheme.faint)
                        .contentTransition(.numericText())
                        .animation(.easeOut(duration: LabTheme.motion), value: sensor.imuRateHz)
                    if let off = sensor.clockOffsetMs, abs(off) >= ClockSync.lockedBandMs {
                        Text("clock \(TelemetryFormat.load(off)) ms")
                            .font(LabTheme.mono(11))
                            .monospacedDigit()
                            .foregroundStyle(LabTheme.warn)
                            .contentTransition(.numericText())
                            .animation(.easeOut(duration: LabTheme.motion), value: off)
                    }
                }
                Sparkline(values: state.history.series(rateSeries(sensor.id)),
                          range: state.history.range(rateSeries(sensor.id)),
                          tint: LabTheme.good, zeroRule: true)
                    .frame(height: 16)
                    .accessibilityHidden(true)
            } else {
                Text(sensor.detail)
                    .font(LabTheme.t(12))
                    .foregroundStyle(LabTheme.dim)
                    .lineLimit(2)
                    .fixedSize(horizontal: false, vertical: true)
            }
            ForEach(RigSpec.nodeSummary(sensor.id), id: \.self) { line in
                Text(line)
                    .font(LabTheme.mono(10.5))
                    .foregroundStyle(LabTheme.faint)
                    .lineLimit(1)
            }
            if sensor.id == .jt128 {
                jtControls
            }
        }
        .help(sensor.id == .jt128
            ? "Hesai JT128 — up-looking dome. Standby stops the motor and lasers without unplugging."
            : "Livox Mid-360 — horizon band. Unplug to park it; the watchdog handles the return.")
        .accessibilityLabel("\(sensor.id.label) \(sensor.state.displayLabel), \(sensor.detail)")
    }

    private func rateSeries(_ id: SensorID) -> TelemetryHistory.Series {
        id == .mid360 ? .pointRate : .jtPointRate
    }

    private var jtControls: some View {
        let wake = RigGate.canWakeJT(rig)
        let standby = RigGate.canStandbyJT(rig)
        return HStack(spacing: 8) {
            if wake.0 {
                Button("Wake") { state.wakeJT() }
                    .buttonStyle(GhostButton())
                    .disabled(state.isActing)
                    .help("Spin the JT128 back up (~10 s to points).")
                    .accessibilityIdentifier("wakeJT")
            }
            if standby.0 {
                Button("Standby") { state.standbyJT() }
                    .buttonStyle(GhostButton())
                    .disabled(state.isActing)
                    .help("Stop the motor and lasers. Runs cold, stays powered, no connector wear.")
                    .accessibilityIdentifier("standbyJT")
            }
        }
    }

    private var sparkNode: some View {
        let nic = rig.pipeline?.nic
        let verb: String = {
            guard let nic, let speed = nic.speedMbps, let duplex = nic.duplex else {
                return rig.apiReachable ? "up" : "—"
            }
            return "\(speed)/\(duplex)"
        }()
        let clockState = rig.clock.state
        return NodeCard(
            title: "Spark · enP7s7",
            pip: rig.apiReachable ? LabTheme.good : LabTheme.unknown,
            breathe: nil,
            verb: verb,
            verbColor: (nic?.carrier == true) ? LabTheme.good : LabTheme.dim,
            hot: rig.anyStreaming
        ) {
            Text(".1.5 + .118.5 · mtu \(nic?.mtu.map(String.init) ?? "—") · rmem \(nic?.rmemMax.map { "\($0 / 1_048_576)M" } ?? "—")")
                .font(LabTheme.mono(10.5))
                .foregroundStyle(LabTheme.faint)
                .lineLimit(1)
            HStack(spacing: 6) {
                Text("ptp \(rig.pipeline?.ptpRole ?? (rig.ptpMasterRunning ? "master up" : "master down"))")
                    .font(LabTheme.mono(11))
                    .foregroundStyle(rig.ptpMasterRunning ? LabTheme.accent : LabTheme.bad)
                Text("· clocks \(clockState.displayLabel)\(rig.clock.deltaMs.map { " Δ\(TelemetryFormat.load($0))ms" } ?? "")")
                    .font(LabTheme.mono(11))
                    .foregroundStyle(clockState == .locked ? LabTheme.good : LabTheme.dim)
            }
            if clockState == .converging, let off = rig.mid360.clockOffsetMs {
                convergenceMeter(offsetMs: off)
                Text("Mid-360 slewing ~10 ms/min toward the ±\(Int(ClockSync.lockedBandMs)) ms band")
                    .font(LabTheme.t(11))
                    .foregroundStyle(LabTheme.warn)
                    .fixedSize(horizontal: false, vertical: true)
            } else if clockState == .converging {
                Text("Mid-360 slews ~10 ms/min — wait, or power-cycle it")
                    .font(LabTheme.t(11))
                    .foregroundStyle(LabTheme.warn)
                    .fixedSize(horizontal: false, vertical: true)
            }
            Text("load \(TelemetryFormat.load(rig.load1)) · mem \(TelemetryFormat.percent(rig.memPct)) · 20 cores · 128 GB")
                .font(LabTheme.mono(10.5))
                .monospacedDigit()
                .foregroundStyle(LabTheme.faint)
                .lineLimit(1)
        }
        .help("GB10 Grace Blackwell. NIC negotiates 1000/full to the switch; both sensors are 100BASE-TX behind it. The Spark is the PTP grandmaster.")
    }

    private var driversNode: some View {
        let dual = rig.containers["lidar-dual"]
        let bound = rig.pipeline?.sockets.udpBound
        let boundLabel: String = {
            guard let bound, !bound.isEmpty else { return TelemetryFormat.unavailable }
            return "\(bound.values.filter { $0 }.count)/\(bound.count) ports bound"
        }()
        return NodeCard(
            title: "lidar-dual",
            pip: dual == "running" ? LabTheme.good : (dual == nil ? LabTheme.unknown : LabTheme.bad),
            breathe: nil,
            verb: dual ?? "—",
            verbColor: dual == "running" ? LabTheme.good : LabTheme.dim,
            hot: rig.anyStreaming
        ) {
            Text("jazzy · 2 drivers · \(boundLabel)")
                .font(LabTheme.mono(10.5))
                .foregroundStyle(LabTheme.faint)
                .lineLimit(1)
            Text("watchdog \(rig.watchdogRunning ? "armed — replug auto-restarts" : "down")")
                .font(LabTheme.mono(10.5))
                .foregroundStyle(rig.watchdogRunning ? LabTheme.faint : LabTheme.warn)
                .lineLimit(1)
        }
        .help("One container runs both vendor drivers and the Foxglove bridge. Driver ingest ports stay bound while the sensors are parked — plug in and points flow.")
    }

    private var viewNode: some View {
        let viewers = rig.pipeline?.sockets.established["8765"]
        return NodeCard(
            title: "Foxglove :8765",
            pip: rig.viewTunnelUp ? LabTheme.accent : LabTheme.idle,
            breathe: nil,
            verb: rig.viewTunnelUp ? "tunnel" : "closed",
            verbColor: rig.viewTunnelUp ? LabTheme.accent : LabTheme.dim,
            hot: false
        ) {
            Text(viewers.map { $0 > 0 ? "\($0) viewer\($0 == 1 ? "" : "s") connected" : "no viewers yet" }
                 ?? TelemetryFormat.unavailable)
                .font(LabTheme.mono(10.5))
                .foregroundStyle(LabTheme.faint)
                .lineLimit(1)
            Text("frame world · ufw closed · tunnel only")
                .font(LabTheme.mono(10.5))
                .foregroundStyle(LabTheme.faint)
                .lineLimit(1)
        }
        .help("The 3D view. Open with `lidar view` or the View menu; the SSH tunnel is the only way in.")
    }

    private var bagNode: some View {
        let rec = rig.recording
        return NodeCard(
            title: "Bag writer",
            pip: rec.active ? LabTheme.bad : LabTheme.idle,
            breathe: rec.active ? breath : nil,
            verb: rec.active ? "rec" : "ready",
            verbColor: rec.active ? LabTheme.bad : LabTheme.dim,
            hot: rec.active
        ) {
            if rec.active {
                Text("\(rec.session ?? "take") · \(rec.sizeGB.map { String(format: "%.1f GB", $0) } ?? "sizing…")")
                    .font(LabTheme.mono(11))
                    .monospacedDigit()
                    .foregroundStyle(LabTheme.text)
                    .lineLimit(1)
            } else {
                Text("4 topics → ~2 GB/min dual · \(rig.diskFreeGB.map { "\($0) GB free" } ?? TelemetryFormat.unavailable)")
                    .font(LabTheme.mono(10.5))
                    .monospacedDigit()
                    .foregroundStyle(LabTheme.faint)
                    .lineLimit(1)
            }
        }
        .help("ros2 bag of both clouds + both IMUs, landing in ~/lidar-sessions on the Spark.")
    }

    /// The measured distance from lock, drawn: a needle riding a 0–500 ms
    /// scale toward the green band. Every position is this poll's |offset|;
    /// nothing here animates unless the measurement moved.
    private func convergenceMeter(offsetMs: Double) -> some View {
        let cap = 500.0
        let mag = min(abs(offsetMs), cap)
        return GeometryReader { geo in
            let w = geo.size.width
            ZStack(alignment: .leading) {
                Capsule().fill(LabTheme.well).frame(height: 4)
                Capsule().fill(LabTheme.good.opacity(0.3))
                    .frame(width: max(4, w * CGFloat(ClockSync.lockedBandMs / cap)), height: 4)
                Circle().fill(LabTheme.warn)
                    .frame(width: 6, height: 6)
                    .offset(x: max(0, w * CGFloat(mag / cap) - 3))
            }
            .frame(maxHeight: .infinity, alignment: .center)
        }
        .frame(height: 8)
        .animation(.easeOut(duration: 0.6), value: offsetMs)
        .accessibilityLabel("Mid-360 clock \(TelemetryFormat.load(offsetMs)) milliseconds from host, lock band \(Int(ClockSync.lockedBandMs))")
    }

    // MARK: Audit strip

    private var auditStrip: some View {
        let checks = PipelineAudit.checks(rig)
        return ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 8) {
                Text("audit")
                    .font(LabTheme.label())
                    .foregroundStyle(LabTheme.faint)
                    .textCase(.uppercase)
                    .tracking(0.8)
                ForEach(checks) { check in
                    auditChip(check)
                }
            }
            .padding(.vertical, 2)
        }
        .accessibilityIdentifier("auditStrip")
    }

    private func verdictColor(_ v: AuditCheck.Verdict) -> Color {
        switch v {
        case .pass: return LabTheme.good
        case .watch: return LabTheme.warn
        case .fail: return LabTheme.bad
        // Pending is "waiting on the operator" — calm steel, like Parked.
        case .pending: return LabTheme.accent
        case .unavailable: return LabTheme.faint
        }
    }

    private func auditChip(_ check: AuditCheck) -> some View {
        HStack(spacing: 6) {
            Text(check.name)
                .font(LabTheme.mono(10.5))
                .foregroundStyle(LabTheme.dim)
            Text(check.value)
                .font(LabTheme.mono(10.5))
                .monospacedDigit()
                .foregroundStyle(verdictColor(check.verdict))
        }
        .padding(.horizontal, 9)
        .padding(.vertical, 5)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .fill(LabTheme.well)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .stroke(verdictColor(check.verdict).opacity(0.35), lineWidth: 1)
        )
        .accessibilityLabel("\(check.name) \(check.value)")
    }
}

/// One node of the graph: title row (pip + name + verb), then whatever the
/// caller stacks beneath. `hot` nodes get a green-tinted stroke, matching the
/// mockup's "data is moving through me" look.
private struct NodeCard<Content: View>: View {
    var title: String
    var pip: Color
    var breathe: Bool?
    var verb: String
    var verbColor: Color
    var hot: Bool
    @ViewBuilder var content: () -> Content

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 7) {
                Circle()
                    .fill(pip)
                    .frame(width: LabTheme.pip, height: LabTheme.pip)
                    .opacity(breathe.map { $0 ? 1 : 0.55 } ?? 1)
                Text(title)
                    .font(LabTheme.t(13, .semibold))
                    .foregroundStyle(LabTheme.text)
                Spacer(minLength: 4)
                Text(verb)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(verbColor)
            }
            content()
        }
        .padding(12)
        .frame(maxWidth: .infinity, alignment: .topLeading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.surface)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(hot ? LabTheme.good.opacity(0.35) : LabTheme.line, lineWidth: 1)
        )
    }
}

/// A labeled connection hop. Forward wires carry data left→right; back wires
/// (PTP, PTC) run control right→left and are always dashed. A `flow` wire has
/// MEASURED data moving through it right now — its dashes travel in the data
/// direction; a wire with no measured flow never animates.
private struct Wire: View {
    enum Direction { case forward, back }
    var label: String
    var color: Color
    var dashed: Bool = false
    var direction: Direction = .forward
    var flow: Bool = false
    @State private var phase: CGFloat = 0

    var body: some View {
        VStack(spacing: 3) {
            GeometryReader { geo in
                let w = geo.size.width
                let y: CGFloat = 4
                Path { p in
                    p.move(to: CGPoint(x: direction == .forward ? 2 : 9, y: y))
                    p.addLine(to: CGPoint(x: direction == .forward ? w - 9 : w - 2, y: y))
                }
                .stroke(color, style: StrokeStyle(
                    lineWidth: 1.2,
                    dash: flow ? [5, 4] : (dashed ? [4, 4] : []),
                    dashPhase: phase))
                Path { p in
                    if direction == .forward {
                        p.move(to: CGPoint(x: w - 9, y: y - 3.5))
                        p.addLine(to: CGPoint(x: w - 2, y: y))
                        p.addLine(to: CGPoint(x: w - 9, y: y + 3.5))
                    } else {
                        p.move(to: CGPoint(x: 9, y: y - 3.5))
                        p.addLine(to: CGPoint(x: 2, y: y))
                        p.addLine(to: CGPoint(x: 9, y: y + 3.5))
                    }
                }
                .fill(color)
            }
            .frame(height: 8)
            Text(label)
                .font(LabTheme.mono(9.5))
                .foregroundStyle(LabTheme.faint)
                .lineLimit(1)
                .frame(maxWidth: .infinity)
        }
        .accessibilityHidden(true)
        .onAppear { restartFlow() }
        .onChange(of: flow) { restartFlow() }
    }

    /// Dash sum is 9, so a phase sweep of ±9 loops seamlessly. Back wires
    /// carry control toward the sensor, so their dashes travel the other way.
    private func restartFlow() {
        withAnimation(nil) { phase = 0 }
        guard flow else { return }
        withAnimation(.linear(duration: 0.7).repeatForever(autoreverses: false)) {
            phase = direction == .forward ? -9 : 9
        }
    }
}
