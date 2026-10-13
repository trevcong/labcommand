import SwiftUI
import LabCommandCore

/// Rig command, pipeline edition. The page answers three questions in order
/// and hides everything else behind a disclosure:
///   1. Where am I?   — one status line + the signal chain drawn as a graph
///   2. What do I do? — ONE contextual hero action (the gates pick it)
///   3. Where's my stuff? — the route map + recent takes, promoted
/// Copy and gates come from RigDesk/RigGate; the view invents nothing.
struct RigView: View {
    @EnvironmentObject private var state: AppState
    @State private var tick = Date()
    @State private var breath = false
    @State private var showDetails = false
    @State private var showPlanner = false

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
                deckHeader
                if !state.rig.sparkReachable || !state.rig.apiReachable || state.lastActionError != nil {
                    banner
                }
                if state.diagnosePresented, let advice = state.lastDiagnose {
                    diagnoseCard(advice)
                }
                MissionClockView(breath: breath)
                PipelineGraph(breath: breath)
                readout
                if !state.focusMode {
                    yourTakes
                    planner
                    details
                }
            }
            .padding(LabTheme.pad)
            .frame(maxWidth: 1120, alignment: .leading)
            .frame(maxWidth: .infinity, alignment: .center)
        }
        .background(LabTheme.bg)
        .onReceive(Timer.publish(every: 1, on: .main, in: .common).autoconnect()) { tick = $0 }
        .onAppear {
            withAnimation(.easeInOut(duration: 1.6).repeatForever(autoreverses: true)) {
                breath = true
            }
        }
    }

    // MARK: Header — status + one hero action

    private var deckHeader: some View {
        let health = state.health
        return HStack(alignment: .center, spacing: 20) {
            VStack(alignment: .leading, spacing: 6) {
                HStack(spacing: 9) {
                    Circle()
                        .fill(LabTheme.band(health.band))
                        .frame(width: 8, height: 8)
                        .opacity(health.band == .ready ? (breath ? 1 : 0.55) : 1)
                    Text(health.headline)
                        .font(LabTheme.t(LabTheme.titleSize, .semibold))
                        .foregroundStyle(LabTheme.text)
                        .accessibilityIdentifier("rigHeadline")
                }
                if let sub = headerSubline {
                    Text(sub)
                        .font(LabTheme.t(12))
                        .foregroundStyle(LabTheme.dim)
                        .fixedSize(horizontal: false, vertical: true)
                }
                if state.isActing {
                    HStack(spacing: 8) {
                        ProgressView().controlSize(.small)
                        Text("Talking to Spark…")
                            .font(LabTheme.t(12))
                            .foregroundStyle(LabTheme.faint)
                    }
                }
            }
            Spacer(minLength: 12)
            heroRow
        }
        .padding(.vertical, 2)
    }

    /// One quiet supporting line, not a facts table. The first blocked-ish
    /// reason wins; a healthy rig shows the clock + disk facts.
    private var headerSubline: String? {
        if let reason = state.health.reasons.first { return reason }
        var bits: [String] = []
        if state.rig.clock.state == .locked, let d = state.rig.clock.deltaMs {
            bits.append("clocks Δ \(TelemetryFormat.load(d)) ms")
        }
        if let free = state.rig.diskFreeGB { bits.append("\(free) GB free") }
        if state.rig.recording.active, let s = state.rig.recording.session {
            bits.insert(s, at: 0)
        }
        return bits.isEmpty ? nil : bits.joined(separator: " · ")
    }

    /// The gates pick ONE primary action; everything else is a quiet button.
    private var heroRow: some View {
        let specs = RigDesk.buttons(for: state.rig)
        let primary = specs.first { $0.isPrimary } ?? specs[0]
        let secondary = specs.filter { $0.action != primary.action && $0.enabled }
        let armed = primary.enabled && !state.isActing
        return VStack(alignment: .trailing, spacing: 6) {
            HStack(spacing: 8) {
                ForEach(secondary, id: \.action) { spec in
                    Button(spec.title) { fire(spec.action) }
                        .buttonStyle(QuietButton())
                        .disabled(state.isActing)
                        .help(spec.whatItDoes)
                        .keyboardShortcut(shortcut(spec.action).0, modifiers: shortcut(spec.action).1)
                        .accessibilityIdentifier(identifier(spec.action))
                }
                Button(primary.title) { fire(primary.action) }
                    .buttonStyle(heroStyle(primary, armed: armed))
                    .disabled(!armed)
                    .help(primary.disabledReason ?? primary.whatItDoes)
                    .keyboardShortcut(shortcut(primary.action).0, modifiers: shortcut(primary.action).1)
                    .accessibilityIdentifier(identifier(primary.action))
            }
            .accessibilityIdentifier("rigActions")
            Text(armed ? primary.whatItDoes : (primary.disabledReason ?? primary.whatItDoes))
                .font(LabTheme.t(11))
                .foregroundStyle(armed ? LabTheme.faint : LabTheme.warn)
                .multilineTextAlignment(.trailing)
                .fixedSize(horizontal: false, vertical: true)
                .frame(maxWidth: 380, alignment: .trailing)
            Text(state.health.label)
                .font(.system(size: 1))
                .foregroundStyle(.clear)
                .accessibilityIdentifier("sessionHealth")
                .accessibilityLabel("Rig \(state.health.label)")
        }
    }

    private func heroStyle(_ spec: RigButtonSpec, armed: Bool) -> HeroButton {
        if !armed { return HeroButton(kind: .muted) }
        switch spec.action {
        case .startRig: return HeroButton(kind: .primary)
        case .startRecording: return HeroButton(kind: .record)
        case .stop: return HeroButton(kind: .stop)
        }
    }

    private func fire(_ action: RigAction) {
        switch action {
        case .startRig: state.startRig()
        case .startRecording: state.startRecord()
        case .stop: state.stopAll()
        }
    }

    private func identifier(_ action: RigAction) -> String {
        switch action {
        case .startRig: return "startRig"
        case .startRecording: return "startRecord"
        case .stop: return "stopAll"
        }
    }

    private func shortcut(_ action: RigAction) -> (KeyEquivalent, EventModifiers) {
        switch action {
        case .startRig: return (.return, .command)
        case .startRecording: return ("r", [.command, .shift])
        case .stop: return (".", .command)
        }
    }

    // MARK: Readout — one slim strip

    private func staleFor(_ key: TelemetryHistory.Series,
                          _ fmt: (Double?) -> String) -> (text: String, age: String)? {
        guard let k = state.history.lastKnown(key) else { return nil }
        return (fmt(k.value), TelemetryHistory.age(k.at, now: tick))
    }

    private var readout: some View {
        let rig = state.rig
        let h = state.history
        return VStack(alignment: .leading, spacing: 8) {
            if let blind = h.blindDuration(now: tick) {
                Text("drivers up, no points for \(TelemetryHistory.format(duration: blind))")
                    .font(LabTheme.mono(11))
                    .monospacedDigit()
                    .foregroundStyle(LabTheme.bad)
            }
            HairlineRow {
                HairlineCell {
                    InstrumentCell(
                        label: "Mid points",
                        value: TelemetryFormat.hz(rig.mid360.pointRateHz),
                        values: h.series(.pointRate),
                        range: h.range(.pointRate),
                        tint: LabTheme.good,
                        alarm: rig.mid360.pointRateHz == 0 && rig.mid360.state != .dark,
                        zeroRule: true,
                        stale: staleFor(.pointRate, { TelemetryFormat.hz($0) })
                    )
                }
                HairlineCell {
                    InstrumentCell(
                        label: "JT points",
                        value: TelemetryFormat.hz(rig.jt128.pointRateHz),
                        values: h.series(.jtPointRate),
                        range: h.range(.jtPointRate),
                        tint: LabTheme.good,
                        alarm: rig.jt128.pointRateHz == 0 && rig.jt128.state != .dark,
                        zeroRule: true,
                        stale: staleFor(.jtPointRate, { TelemetryFormat.hz($0) })
                    )
                }
                HairlineCell {
                    if rig.recording.active {
                        InstrumentCell(
                            label: "Bag",
                            value: rig.recording.sizeGB.map { String(format: "%.1f GB", $0) } ?? "recording"
                        )
                    } else {
                        InstrumentCell(
                            label: "Spark",
                            value: "load \(TelemetryFormat.load(rig.load1)) · mem \(TelemetryFormat.percent(rig.memPct))",
                            values: h.series(.load),
                            range: h.range(.load),
                            tint: LabTheme.accent
                        )
                    }
                }
                HairlineCell {
                    if let elapsed = state.elapsedRecordLabel(now: tick) {
                        InstrumentCell(label: "Elapsed", value: elapsed)
                    } else {
                        InstrumentCell(
                            label: "Disk",
                            value: rig.diskFreeGB.map { "\($0) GB free" } ?? TelemetryFormat.unavailable
                        )
                    }
                }
            }
        }
    }

    // MARK: Your takes — the stuff

    private var yourTakes: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(alignment: .firstTextBaseline) {
                Text("Your takes")
                    .font(LabTheme.t(13, .medium))
                    .foregroundStyle(LabTheme.faint)
                Spacer()
                Button("All takes") { state.dest = .research }
                    .buttonStyle(GhostButton())
            }
            HStack(alignment: .top, spacing: 16) {
                if let track = state.currentTrack {
                    VStack(alignment: .leading, spacing: 8) {
                        if state.tracks.count > 1 {
                            trackPicker(current: track)
                        }
                        TrajectoryCard(track: track)
                            .id(track.session)
                    }
                    .frame(maxWidth: .infinity)
                    .accessibilityIdentifier("routeMap")
                }
                takeList
                    .frame(maxWidth: .infinity)
            }
        }
    }

    private func trackPicker(current: SessionTrajectory) -> some View {
        HStack(spacing: 4) {
            ForEach(state.tracks, id: \.session) { t in
                let on = t.session == current.session
                Button {
                    state.selectedTrack = t.session
                } label: {
                    Text(shortTrackName(t.session))
                        .font(LabTheme.mono(11))
                        .foregroundStyle(on ? LabTheme.text : LabTheme.dim)
                        .padding(.horizontal, 9)
                        .padding(.vertical, 5)
                        .background(
                            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                                .fill(on ? LabTheme.accentMuted : Color.clear)
                        )
                }
                .buttonStyle(.plain)
                .accessibilityAddTraits(on ? .isSelected : [])
            }
            Spacer(minLength: 0)
        }
    }

    private func shortTrackName(_ stem: String) -> String {
        let parts = stem.split(separator: "_")
        guard parts.count > 1 else { return stem }
        return parts.dropFirst().joined(separator: " ")
    }

    private var takeList: some View {
        VStack(alignment: .leading, spacing: 8) {
            if let summary = state.lastStopSummary {
                HStack(alignment: .firstTextBaseline, spacing: 8) {
                    Circle().fill(LabTheme.accent).frame(width: 5, height: 5)
                    Text(summary)
                        .font(LabTheme.t(12))
                        .foregroundStyle(LabTheme.text)
                        .textSelection(.enabled)
                        .fixedSize(horizontal: false, vertical: true)
                }
                .accessibilityIdentifier("stopReport")
            }
            if state.library.isEmpty {
                Text("Nothing on this Mac yet. Dual bags land on the Spark in ~/lidar-sessions; pull keepers to ~/Desktop/lidar-sessions.")
                    .font(LabTheme.t(12))
                    .foregroundStyle(LabTheme.faint)
                    .fixedSize(horizontal: false, vertical: true)
            } else {
                ForEach(state.library.prefix(5)) { rec in
                    HStack(spacing: 10) {
                        Circle()
                            .fill(rec.quality.hasPointCloud ? LabTheme.good : LabTheme.idle)
                            .frame(width: 5, height: 5)
                        Text(rec.filename)
                            .font(LabTheme.t(12))
                            .foregroundStyle(LabTheme.text)
                            .lineLimit(1)
                        Spacer(minLength: 8)
                        Text(rec.bytesDisplay)
                            .font(LabTheme.mono(11))
                            .foregroundStyle(LabTheme.faint)
                    }
                }
                .accessibilityIdentifier("landedSession")
            }
        }
        .padding(12)
        .frame(maxWidth: .infinity, minHeight: 84, alignment: .topLeading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.well)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
    }

    // MARK: Planner — collapsed by default

    private var planner: some View {
        VStack(alignment: .leading, spacing: 10) {
            DisclosureGroup(isExpanded: $showPlanner) {
                VStack(alignment: .leading, spacing: 10) {
                    HStack(spacing: 6) {
                        ForEach(CapturePreset.allCases) { preset in
                            presetChip(preset)
                        }
                        Spacer(minLength: 8)
                        Text(RigDesk.sessionFolderPreview(state.session))
                            .font(LabTheme.mono(11))
                            .monospacedDigit()
                            .foregroundStyle(LabTheme.faint)
                            .textSelection(.enabled)
                    }
                    Text(state.session.preset.operatorDetail)
                        .font(LabTheme.t(12))
                        .foregroundStyle(LabTheme.dim)
                        .fixedSize(horizontal: false, vertical: true)
                    TextField("Notes — what moved, doors, weather", text: $state.session.notes)
                        .textFieldStyle(.plain)
                        .font(LabTheme.t(13))
                        .foregroundStyle(LabTheme.text)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 8)
                        .background(
                            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                                .fill(LabTheme.well)
                        )
                        .overlay(
                            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                                .stroke(LabTheme.line, lineWidth: 1)
                        )
                        .accessibilityLabel("Session notes")
                }
                .padding(.top, 10)
            } label: {
                HStack(spacing: 8) {
                    Text("Plan this take")
                        .font(LabTheme.t(13, .medium))
                        .foregroundStyle(LabTheme.faint)
                    Text("\(state.session.preset.code) \(state.session.preset.operatorTitle)")
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.dim)
                }
            }
        }
    }

    private func presetChip(_ preset: CapturePreset) -> some View {
        let on = state.session.preset == preset
        return Button {
            state.session.preset = preset
        } label: {
            HStack(spacing: 6) {
                Text(preset.code)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(on ? LabTheme.accent : LabTheme.faint)
                Text(preset.operatorTitle)
                    .font(LabTheme.t(12, on ? .semibold : .regular))
                    .foregroundStyle(on ? LabTheme.text : LabTheme.dim)
            }
            .padding(.horizontal, 10)
            .padding(.vertical, 6)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .fill(on ? LabTheme.accentMuted : LabTheme.well)
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .stroke(on ? LabTheme.accent.opacity(0.55) : LabTheme.line, lineWidth: 1)
            )
        }
        .buttonStyle(.plain)
        .help(preset.operatorDetail)
        .accessibilityLabel(preset.label)
        .accessibilityAddTraits(on ? .isSelected : [])
    }

    // MARK: Details — the rail + activity, folded away

    private var details: some View {
        DisclosureGroup(isExpanded: $showDetails) {
            VStack(alignment: .leading, spacing: 14) {
                HairlineRow {
                    ForEach(RigStage.allCases, id: \.self) { stage in
                        HairlineCell { stageCell(stage) }
                    }
                }
                .accessibilityIdentifier("rigStages")
                activityLog
            }
            .padding(.top, 10)
        } label: {
            HStack(spacing: 8) {
                Text("Under the hood")
                    .font(LabTheme.t(13, .medium))
                    .foregroundStyle(LabTheme.faint)
                if let last = state.missionEvents.last {
                    Text("\(MissionLog.clock.string(from: last.at))  \(last.text)")
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.faint)
                        .lineLimit(1)
                }
            }
        }
        .animation(.easeOut(duration: LabTheme.motion), value: state.rig.stackRunning)
    }

    private func stageCell(_ stage: RigStage) -> some View {
        let st = RigDesk.stageState(stage, state.rig)
        let blocked = st == .blocked

        return VStack(alignment: .leading, spacing: 5) {
            HStack(spacing: 7) {
                Circle()
                    .fill(LabTheme.stageRig(st))
                    .frame(width: LabTheme.pip, height: LabTheme.pip)
                    .opacity(st == .running ? (breath ? 1 : 0.55) : 1)
                Text(stage.label)
                    .font(LabTheme.t(13, .semibold))
                    .foregroundStyle(blocked ? LabTheme.bad : LabTheme.text)
                Text(st.displayLabel)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(st == .running ? LabTheme.good : LabTheme.dim)
            }
            Text(RigDesk.stageDetail(stage, state.rig))
                .font(LabTheme.mono(11))
                .monospacedDigit()
                .foregroundStyle(LabTheme.faint)
                .lineLimit(2)
                .fixedSize(horizontal: false, vertical: true)
                .textSelection(.enabled)
            if blocked {
                Button("Diagnose") { state.diagnose(stage: stage) }
                    .buttonStyle(GhostButton())
                    .accessibilityIdentifier("diagnoseButton")
            }
        }
        .frame(maxWidth: .infinity, minHeight: 62, alignment: .topLeading)
        .help(RigDesk.stageMeaning(stage))
        .accessibilityLabel("\(stage.label) \(st.displayLabel), \(RigDesk.stageDetail(stage, state.rig))")
    }

    private var activityLog: some View {
        VStack(alignment: .leading, spacing: 6) {
            if state.missionEvents.isEmpty {
                Text("Nothing yet. Plug in the sensors (or press Start rig) and the timeline fills in.")
                    .font(LabTheme.t(12))
                    .foregroundStyle(LabTheme.faint)
            } else {
                ForEach(state.missionEvents.suffix(8)) { event in
                    HStack(alignment: .firstTextBaseline, spacing: 10) {
                        Text(MissionLog.clock.string(from: event.at))
                            .font(LabTheme.mono(11))
                            .foregroundStyle(LabTheme.faint)
                            .frame(width: 72, alignment: .leading)
                        Text(event.kind.displayLabel)
                            .font(LabTheme.mono(11))
                            .foregroundStyle(kindColor(event.kind))
                            .frame(width: 64, alignment: .leading)
                        Text(event.text)
                            .font(LabTheme.t(12))
                            .foregroundStyle(LabTheme.text)
                            .textSelection(.enabled)
                    }
                }
            }
        }
        .padding(12)
        .frame(maxWidth: .infinity, alignment: .topLeading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.well)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .accessibilityIdentifier("missionLog")
    }

    private func kindColor(_ kind: MissionEventKind) -> Color {
        switch kind {
        case .error: return LabTheme.bad
        case .stop: return LabTheme.warn
        case .start, .record: return LabTheme.accent
        case .diagnose: return LabTheme.warn
        default: return LabTheme.faint
        }
    }

    // MARK: Banner / diagnose

    private var banner: some View {
        let text: String = {
            if !state.rig.sparkReachable {
                return (state.rig.blockReason ?? "Spark not reachable")
                    + " — the Spark can be powered and still unreachable; Tailscale on both ends is the usual fix."
            }
            if !state.rig.apiReachable {
                return (state.rig.blockReason ?? "Command center not answering")
                    + " — lidar-center container or the :8766 tunnel. Refresh retries the tunnel."
            }
            return state.lastActionError ?? ""
        }()
        return HStack(alignment: .top, spacing: 10) {
            Circle()
                .fill(LabTheme.warn)
                .frame(width: 6, height: 6)
                .padding(.top, 6)
            Text(text)
                .font(LabTheme.t(13))
                .foregroundStyle(LabTheme.text)
                .fixedSize(horizontal: false, vertical: true)
                .textSelection(.enabled)
            Spacer(minLength: 8)
            Button("Diagnose") { state.diagnose() }
                .buttonStyle(GhostButton())
                .accessibilityIdentifier("diagnoseButton")
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.warn.opacity(0.08))
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.warn.opacity(0.28), lineWidth: 1)
        )
        .accessibilityIdentifier("rigBanner")
    }

    private func diagnoseCard(_ advice: RigAdvice) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("What to do")
                    .font(LabTheme.label())
                    .foregroundStyle(LabTheme.faint)
                    .textCase(.uppercase)
                    .tracking(0.8)
                Spacer()
                Button("Dismiss") { state.diagnosePresented = false }
                    .buttonStyle(GhostButton())
            }
            Text(advice.reason)
                .font(LabTheme.t(14, .semibold))
                .foregroundStyle(LabTheme.text)
            Text(advice.recommendedAction)
                .font(LabTheme.t(13))
                .foregroundStyle(LabTheme.text)
            Text(advice.recoveryHint)
                .font(LabTheme.t(12))
                .foregroundStyle(LabTheme.dim)
                .fixedSize(horizontal: false, vertical: true)
            HStack(spacing: 8) {
                Button("Refresh") { state.refresh(manual: true) }
                    .buttonStyle(QuietButton())
                Button("Hardware") { state.dest = .devices; state.focusMode = false }
                    .buttonStyle(QuietButton())
            }
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.well)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .accessibilityIdentifier("diagnosePanel")
    }
}
