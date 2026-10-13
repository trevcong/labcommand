import SwiftUI
import LabCommandCore

/// Operator shell: top nav + one job. Focus hides chrome.
struct CommandCenterView: View {
    @EnvironmentObject private var state: AppState

    var body: some View {
        VStack(spacing: 0) {
            if !state.focusMode {
                topBar
                Rectangle().fill(LabTheme.line).frame(height: 1)
            }
            content
        }
        .background(LabTheme.bg)
        .frame(minWidth: 1040, minHeight: 700)
        .preferredColorScheme(.dark)
        .animation(.easeOut(duration: 0.2), value: state.focusMode)
    }

    // MARK: - Top bar

    private var topBar: some View {
        HStack(spacing: 20) {
            Text("Lab")
                .font(LabTheme.t(13, .semibold))
                .foregroundStyle(LabTheme.text)
                .accessibilityLabel("Lab")
            Text("dual-lidar rig")
                .font(LabTheme.t(12))
                .foregroundStyle(LabTheme.faint)

            HStack(spacing: 4) {
                ForEach(Dest.navCases) { d in
                    navItem(d)
                }
            }

            Spacer(minLength: 12)

            statusChip(
                title: "Spark",
                ok: state.rig.sparkReachable,
                help: state.rig.sparkReachable
                    ? "Spark reachable — \(state.detail("spark_ssh"))"
                    : "Spark unreachable — \(state.detail("spark_ssh"))"
            )
            sensorChip(state.rig.mid360)
            sensorChip(state.rig.jt128)

            if state.isRefreshing || state.isActing {
                ProgressView()
                    .controlSize(.mini)
            }

            Button("Refresh") { state.refresh(manual: true) }
                .font(LabTheme.t(12, .medium))
                .buttonStyle(QuietButton())
                .disabled(state.isRefreshing || state.isActing)
                .keyboardShortcut("r", modifiers: .command)
                .accessibilityLabel("Refresh status")
        }
        .padding(.horizontal, LabTheme.pad)
        .frame(height: LabTheme.topBarH)
        .background(LabTheme.bg)
        .accessibilityIdentifier("overallStatus")
    }

    private func navItem(_ d: Dest) -> some View {
        let selected = state.dest == d
        return Button {
            state.dest = d
        } label: {
            Text(d.label)
                .font(LabTheme.t(13, selected ? .semibold : .medium))
                .foregroundStyle(selected ? LabTheme.text : LabTheme.dim)
                .padding(.horizontal, 12)
                .padding(.vertical, 6)
                .background(
                    RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                        .fill(selected ? LabTheme.accentMuted : Color.clear)
                )
                .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
        .keyboardShortcut(KeyEquivalent(Character(d.shortcutKey)), modifiers: .command)
        .accessibilityLabel(d.label)
        .accessibilityAddTraits(selected ? .isSelected : [])
    }

    private func sensorChip(_ sensor: SensorReading) -> some View {
        let color: Color = {
            switch sensor.state {
            case .streaming: return LabTheme.good
            case .powered: return LabTheme.warn
            case .dark: return LabTheme.idle
            case .unknown: return LabTheme.unknown
            }
        }()
        return HStack(spacing: 6) {
            Circle()
                .fill(color)
                .frame(width: 6, height: 6)
            Text(sensor.id.label)
                .font(LabTheme.t(12, .medium))
                .foregroundStyle(sensor.state == .streaming ? LabTheme.text : LabTheme.dim)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(
            RoundedRectangle(cornerRadius: 100, style: .continuous)
                .fill(LabTheme.well)
        )
        .overlay(
            RoundedRectangle(cornerRadius: 100, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .help("\(sensor.id.label) · \(sensor.state.displayLabel) — \(sensor.detail)")
    }

    private func statusChip(title: String, ok: Bool, help: String) -> some View {
        HStack(spacing: 6) {
            Circle()
                .fill(ok ? LabTheme.good : LabTheme.bad)
                .frame(width: 6, height: 6)
            Text(title)
                .font(LabTheme.t(12, .medium))
                .foregroundStyle(ok ? LabTheme.text : LabTheme.dim)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(
            RoundedRectangle(cornerRadius: 100, style: .continuous)
                .fill(LabTheme.well)
        )
        .overlay(
            RoundedRectangle(cornerRadius: 100, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .help(help)
    }

    // MARK: - Content

    private var content: some View {
        Group {
            switch state.dest {
            case .workspace: rigWorkspace
            case .settings: SettingsView()
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    // MARK: - Rig Workspace (minimal controls for recording)

    private var rigWorkspace: some View {
        HStack(alignment: .top, spacing: LabTheme.pad) {
            leftColumn
            Spacer()
        }
        .padding(LabTheme.pad)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
    }

    private var leftColumn: some View {
        VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
            rigStatusSection
            sensorCards
            clocksSection
            recordingSection
            eventLogSection
        }
        .frame(width: 320)
    }

    private var rigStatusSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("Rig Status")
                .font(LabTheme.t(LabTheme.titleSize, .semibold))
                .foregroundStyle(LabTheme.text)
            Text(state.health.headline)
                .font(LabTheme.t(13))
                .foregroundStyle(LabTheme.band(state.health.band))
        }
    }

    private var sensorCards: some View {
        HStack(spacing: 12) {
            sensorCard(state.rig.mid360)
            sensorCard(state.rig.jt128)
        }
    }

    private func sensorCard(_ sensor: SensorReading) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Circle()
                    .fill(LabTheme.stageRig(sensor.state == .streaming ? .running : (sensor.state == .dark ? .idle : .unknown)))
                    .frame(width: LabTheme.pip, height: LabTheme.pip)
                Text(sensor.id.label)
                    .font(LabTheme.t(12, .medium))
                    .foregroundStyle(LabTheme.text)
            }
            Text(sensor.state.displayLabel)
                .font(LabTheme.t(11))
                .foregroundStyle(LabTheme.dim)
            if let hz = sensor.pointRateHz {
                Text(TelemetryFormat.hz(hz))
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.text)
            }
        }
        .padding(12)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.surface)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
    }

    private var clocksSection: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("Clocks")
                .font(LabTheme.t(11, .medium))
                .foregroundStyle(LabTheme.faint)
                .textCase(.uppercase)
            HStack {
                Circle()
                    .fill(state.rig.clock.state == .locked ? LabTheme.good : LabTheme.warn)
                    .frame(width: LabTheme.pip, height: LabTheme.pip)
                Text(state.rig.clock.state.displayLabel)
                    .font(LabTheme.t(12))
                    .foregroundStyle(LabTheme.text)
                if let delta = state.rig.clock.deltaMs {
                    Text("Δ \(TelemetryFormat.load(delta)) ms")
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.dim)
                }
            }
        }
    }

    private var recordingSection: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text("Recording")
                .font(LabTheme.t(11, .medium))
                .foregroundStyle(LabTheme.faint)
                .textCase(.uppercase)

            HStack(spacing: 8) {
                ForEach(RigDesk.buttons(for: state.rig), id: \.action) { btn in
                    Button(btn.title) {
                        performAction(btn.action)
                    }
                    .font(LabTheme.t(12, btn.isPrimary ? .semibold : .medium))
                    .buttonStyle(PrimaryButton(emphasized: btn.isPrimary))
                    .disabled(!btn.enabled || state.isActing)
                }
            }

            if state.rig.recording.active {
                HStack {
                    Circle()
                        .fill(LabTheme.bad)
                        .frame(width: LabTheme.pip, height: LabTheme.pip)
                    Text("Recording")
                        .font(LabTheme.t(12, .medium))
                        .foregroundStyle(LabTheme.text)
                    if let elapsed = state.elapsedRecordLabel() {
                        Text(elapsed)
                            .font(LabTheme.mono(11))
                            .foregroundStyle(LabTheme.dim)
                    }
                }
            }

            if let free = state.rig.diskFreeGB {
                Text("\(free) GB free")
                    .font(LabTheme.t(11))
                    .foregroundStyle(LabTheme.faint)
            }

            HStack(spacing: 8) {
                Button("Arm at lock") { state.armAtLock() }
                    .font(LabTheme.t(11, .medium))
                    .buttonStyle(QuietButton())
                    .disabled(!RigGate.canArmAtLock(state.rig).0 || state.isActing)

                Button("Diagnose") { state.diagnose() }
                    .font(LabTheme.t(11, .medium))
                    .buttonStyle(QuietButton())
                    .keyboardShortcut("d", modifiers: [.command, .shift])
            }
        }
    }

    private var eventLogSection: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("Event Log")
                .font(LabTheme.t(11, .medium))
                .foregroundStyle(LabTheme.faint)
                .textCase(.uppercase)

            VStack(alignment: .leading, spacing: 4) {
                ForEach(Array(state.missionEvents.suffix(5).reversed())) { event in
                    HStack(alignment: .top, spacing: 6) {
                        Text(event.time)
                            .font(LabTheme.mono(10))
                            .foregroundStyle(LabTheme.faint)
                        Text(event.text)
                            .font(LabTheme.t(11))
                            .foregroundStyle(LabTheme.dim)
                            .lineLimit(2)
                    }
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
        }
    }

    private func performAction(_ action: RigAction) {
        switch action {
        case .startRig: state.startRig()
        case .startRecording: state.startRecord()
        case .stop: state.stopAll()
        }
    }
}

