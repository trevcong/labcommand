import SwiftUI
import LabCommandCore

/// B1 "Mission Clock": the collector's own take lifecycle drawn as the spine
/// of the Rig page. Everything here renders from collector-reported state —
/// the armed block, the live probes, and the last finalize's gate verdict.
/// No armed take and nothing recording → the section collapses to (at most)
/// one quiet last-gate line; a rig with no history shows nothing at all.
struct MissionClockView: View {
    @EnvironmentObject private var state: AppState
    var breath: Bool

    private var rig: RigSnapshot { state.rig }

    var body: some View {
        if rig.armed != nil || rig.recording.active {
            activeRail
        } else if let gate = rig.lastGate {
            lastGateLine(gate)
        }
    }

    // MARK: Phases

    private enum Phase: Int, CaseIterable {
        case armed, lock, record, finalize, gate
        var label: String {
            switch self {
            case .armed: return "Armed"
            case .lock: return "Lock"
            case .record: return "Record"
            case .finalize: return "Finalize"
            case .gate: return "Gate"
            }
        }
    }

    private enum PhaseState { case done, active, pending }

    private func phaseState(_ p: Phase) -> PhaseState {
        let recording = rig.recording.active || rig.armed?.state == "recording"
        switch p {
        case .armed: return .done
        case .lock: return recording ? .done : .active
        case .record: return recording ? .active : .pending
        case .finalize, .gate: return .pending
        }
    }

    private func phaseDetail(_ p: Phase) -> String {
        switch p {
        case .armed:
            let proto = rig.armed?.protocolID ?? rig.lastGate?.protocolID ?? "—"
            return "\(rig.armed?.name ?? rig.recording.session ?? "take") · \(proto)"
        case .lock:
            if let streak = rig.armed?.lockStreak, rig.armed?.state == "waiting_for_lock" {
                return "streak \(streak)/3 · 3 s samples"
            }
            return "streak held"
        case .record:
            if rig.recording.active {
                let size = rig.recording.sizeGB.map { String(format: "%.1f GB", $0) } ?? "sizing…"
                return "\(size) · 4 topics"
            }
            return rig.armed?.durationS.map { "\($0) s at lock" } ?? "at lock"
        case .finalize: return "sidecar.v0 · immutable"
        case .gate: return rig.armed?.protocolID ?? "scored at finalize"
        }
    }

    private func phasePip(_ st: PhaseState) -> some View {
        Circle()
            .fill(st == .pending ? LabTheme.idle : (st == .active ? LabTheme.good : LabTheme.accent))
            .frame(width: LabTheme.pip, height: LabTheme.pip)
            .opacity(st == .active ? (breath ? 1 : 0.5) : 1)
    }

    // MARK: Active rail

    private var activeRail: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(alignment: .top, spacing: 0) {
                ForEach(Phase.allCases, id: \.rawValue) { p in
                    let st = phaseState(p)
                    VStack(alignment: .leading, spacing: 4) {
                        HStack(spacing: 6) {
                            phasePip(st)
                            Text(p.label)
                                .font(LabTheme.t(13, st == .active ? .semibold : .medium))
                                .foregroundStyle(st == .pending ? LabTheme.dim : LabTheme.text)
                        }
                        Text(phaseDetail(p))
                            .font(LabTheme.mono(10.5))
                            .monospacedDigit()
                            .foregroundStyle(st == .active ? LabTheme.faint : LabTheme.faint.opacity(0.75))
                            .lineLimit(2)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                    if p != .gate {
                        Rectangle()
                            .fill(LabTheme.line)
                            .frame(width: 24, height: 1)
                            .padding(.top, 6)
                            .padding(.horizontal, 4)
                    }
                }
                Button("Disarm") { state.disarm() }
                    .buttonStyle(GhostButton())
                    .disabled(state.isActing)
                    .help("Cancel the held take. Nothing records until re-armed.")
                    .accessibilityIdentifier("disarmTake")
            }
            if rig.armed?.state == "waiting_for_lock" {
                lockInstrument
                preflight
            }
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.surface)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .accessibilityIdentifier("missionRail")
    }

    // MARK: Lock instrument — offsets as a strip chart with the band

    private var lockInstrument: some View {
        let h = state.history
        let mid = h.series(.midOffset)
        let jt = h.series(.jtOffset)
        let midNow = rig.mid360.clockOffsetMs
        let jtNow = rig.jt128.clockOffsetMs
        return VStack(alignment: .leading, spacing: 5) {
            HStack(spacing: 10) {
                Text("clock offset · host − sensor")
                    .font(LabTheme.label())
                    .foregroundStyle(LabTheme.faint)
                    .textCase(.uppercase)
                    .tracking(0.8)
                Spacer()
                if let midNow {
                    Text(String(format: "mid %+.1f ms", midNow))
                        .font(LabTheme.mono(11))
                        .monospacedDigit()
                        .foregroundStyle(abs(midNow) < ClockSync.lockedBandMs ? LabTheme.good : LabTheme.warn)
                        .contentTransition(.numericText())
                        .animation(.easeOut(duration: LabTheme.motion), value: midNow)
                }
                if let jtNow {
                    Text(String(format: "jt %+.1f ms", jtNow))
                        .font(LabTheme.mono(11))
                        .monospacedDigit()
                        .foregroundStyle(abs(jtNow) < ClockSync.lockedBandMs ? LabTheme.good : LabTheme.warn)
                }
                Text("±\(Int(ClockSync.lockedBandMs)) ms band")
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.faint)
            }
            OffsetStripChart(mid: mid, jt: jt, band: ClockSync.lockedBandMs)
                .frame(height: 56)
            HStack(spacing: 12) {
                if let slew = h.slewPerMin(.midOffset) {
                    Text(String(format: "slew %+.1f ms/min (fit over recent samples)", slew))
                        .font(LabTheme.mono(10.5))
                        .monospacedDigit()
                        .foregroundStyle(LabTheme.dim)
                }
                if let eta = h.minutesToBand(.midOffset, band: ClockSync.lockedBandMs) {
                    Text(String(format: "in band ≈ %.0f min — projection from measured slew, not a promise", eta))
                        .font(LabTheme.mono(10.5))
                        .monospacedDigit()
                        .foregroundStyle(LabTheme.accent)
                }
            }
        }
        .accessibilityIdentifier("lockInstrument")
    }

    // MARK: Gate preflight

    private func preflightColor(_ st: GatePreflight.Check.Status) -> Color {
        switch st {
        case .holds: return LabTheme.good
        case .waiting: return LabTheme.warn
        case .atFinalize: return LabTheme.accent
        case .unavailable: return LabTheme.faint
        }
    }

    private func preflightWord(_ st: GatePreflight.Check.Status) -> String {
        switch st {
        case .holds: return "holds"
        case .waiting: return "waiting"
        case .atFinalize: return "at finalize"
        case .unavailable: return "unavailable"
        }
    }

    private var preflight: some View {
        let checks = GatePreflight.checks(rig)
        return VStack(alignment: .leading, spacing: 5) {
            Text("gate preflight · live samples of the finalizer's own probes")
                .font(LabTheme.label())
                .foregroundStyle(LabTheme.faint)
                .textCase(.uppercase)
                .tracking(0.8)
            ForEach(checks) { check in
                HStack(alignment: .firstTextBaseline, spacing: 8) {
                    Circle()
                        .fill(preflightColor(check.status))
                        .frame(width: 4, height: 4)
                    Text(check.name)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.text)
                        .frame(width: 128, alignment: .leading)
                    Text(preflightWord(check.status))
                        .font(LabTheme.mono(11))
                        .foregroundStyle(preflightColor(check.status))
                        .frame(width: 76, alignment: .leading)
                    Text(check.detail)
                        .font(LabTheme.mono(10.5))
                        .monospacedDigit()
                        .foregroundStyle(LabTheme.faint)
                        .lineLimit(1)
                }
            }
        }
        .accessibilityIdentifier("gatePreflight")
    }

    // MARK: Collapsed — last gate verdict, one quiet line

    private func lastGateLine(_ gate: LastGate) -> some View {
        HStack(spacing: 8) {
            Circle()
                .fill(gate.passed ? LabTheme.good : LabTheme.warn)
                .frame(width: 5, height: 5)
            Text("last take")
                .font(LabTheme.mono(11))
                .foregroundStyle(LabTheme.faint)
            Text(gate.takeID)
                .font(LabTheme.mono(11))
                .foregroundStyle(LabTheme.text)
                .lineLimit(1)
            Text(gate.passed
                 ? "gate passed"
                 : "gate failed: \(gate.failedChecks.joined(separator: ", "))")
                .font(LabTheme.mono(11))
                .foregroundStyle(gate.passed ? LabTheme.good : LabTheme.warn)
                .lineLimit(1)
            Spacer(minLength: 8)
            Button("Arm at lock") { state.armAtLock() }
                .buttonStyle(GhostButton())
                .disabled(state.isActing || !rig.apiReachable)
                .help("Hand the next take to the collector: it fires itself at sustained clock lock and finalizes with no app in the loop.")
                .accessibilityIdentifier("armAtLock")
        }
        .accessibilityIdentifier("lastGateLine")
    }
}

/// Two offset traces over the ±band. Gaps are gaps — a poll that measured no
/// offset breaks the line rather than inventing one. The y-scale grows to fit
/// the worst measured value so a 470 ms regression is drawn, not clipped.
private struct OffsetStripChart: View {
    let mid: [Double?]
    let jt: [Double?]
    let band: Double

    var body: some View {
        Canvas(rendersAsynchronously: false) { ctx, size in
            let all = (mid + jt).compactMap { $0 }
            guard !all.isEmpty else { return }
            let maxMag = max(band * 1.4, all.map(abs).max() ?? band)
            let span = maxMag * 2
            func y(_ v: Double) -> CGFloat { CGFloat((maxMag - v) / span) * size.height }

            let bandPath = Path(CGRect(x: 0, y: y(band), width: size.width,
                                       height: y(-band) - y(band)))
            ctx.fill(bandPath, with: .color(LabTheme.good.opacity(0.09)))
            var zero = Path()
            zero.move(to: CGPoint(x: 0, y: y(0)))
            zero.addLine(to: CGPoint(x: size.width, y: y(0)))
            ctx.stroke(zero, with: .color(LabTheme.line),
                       style: StrokeStyle(lineWidth: 1, dash: [2, 3]))

            for (vals, color) in [(jt, LabTheme.good), (mid, LabTheme.warn)] {
                guard vals.count > 1 else { continue }
                let dx = size.width / CGFloat(vals.count - 1)
                var run: [CGPoint] = []
                func flush() {
                    defer { run = [] }
                    guard run.count > 1 else { return }
                    var p = Path()
                    p.move(to: run[0])
                    for pt in run.dropFirst() { p.addLine(to: pt) }
                    ctx.stroke(p, with: .color(color),
                               style: StrokeStyle(lineWidth: 1.4, lineJoin: .round))
                }
                for (i, v) in vals.enumerated() {
                    if let v {
                        run.append(CGPoint(x: CGFloat(i) * dx, y: y(v)))
                    } else {
                        flush()
                    }
                }
                flush()
            }
        }
        .accessibilityHidden(true)
    }
}
