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
            Text("LabCommand")
                .font(LabTheme.t(13, .semibold))
                .foregroundStyle(LabTheme.text)
                .accessibilityLabel("LabCommand")
            Text("room memory")
                .font(LabTheme.t(12))
                .foregroundStyle(LabTheme.faint)

            HStack(spacing: 4) {
                ForEach(Dest.navCases) { d in
                    navItem(d)
                }
                // Only a trail launch has a session to replay.
                if state.trail != nil {
                    navItem(.trail)
                }
            }

            Spacer(minLength: 12)

            if state.isolatesNetwork {
                localSparkChip
            } else {
                // The three chips follow the room link. The left column stays on the probe.
                statusChip(
                    title: "Spark",
                    ok: state.statusDots.sparkReachable,
                    help: state.statusDots.sparkHelp ?? (
                        state.statusDots.sparkReachable
                            ? "Spark reachable — \(state.detail("spark_ssh"))"
                            : "Spark unreachable — \(state.detail("spark_ssh"))"
                    )
                )
            }
            sensorChip(state.statusDots.mid360)
            sensorChip(state.statusDots.jt128)

            if state.isRefreshing || state.isActing {
                ProgressView()
                    .controlSize(.mini)
            }

            Button("Refresh") { state.refresh(manual: true) }
                .font(LabTheme.t(12, .medium))
                .buttonStyle(QuietButton())
                .disabled(state.isolatesNetwork || state.isRefreshing || state.isActing)
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

    private var localSparkChip: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(LabTheme.dim)
                .frame(width: 6, height: 6)
            Text(ChangeOverlay.localChip)
                .font(LabTheme.t(12, .medium))
                .foregroundStyle(LabTheme.dim)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(Capsule().fill(LabTheme.well))
        .overlay(Capsule().stroke(LabTheme.line, lineWidth: 1))
        .help(ChangeOverlay.localSessionLine)
    }

    // MARK: - Content

    private var content: some View {
        Group {
            switch state.dest {
            case .workspace: rigWorkspace
            case .settings: SettingsView()
            case .liveRoom: LiveRoomPage()
            case .trail:
                if let trail = state.trail {
                    TrailReplayView(player: trail)
                } else {
                    LiveRoomPage()
                }
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    // MARK: - Rig Workspace (minimal controls for recording)

    private var rigWorkspace: some View {
        HStack(alignment: .top, spacing: LabTheme.pad) {
            leftColumn
            pointCloudCanvas
        }
        .padding(LabTheme.pad)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
    }
    
    private var pointCloudCanvas: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("3D View")
                    .font(LabTheme.t(LabTheme.titleSize, .semibold))
                    .foregroundStyle(LabTheme.text)
                Spacer()
                if !state.isolatesNetwork {
                    LiveStreamControls(live: state.live)
                }
                Picker("Color", selection: $state.pointCloudColorMode) {
                    ForEach(PointCloudColorMode.allCases, id: \.rawValue) { mode in
                        Text(mode.label).tag(mode)
                    }
                }
                .pickerStyle(.segmented)
                .frame(width: 200)
            }

            Group {
                if state.isolatesNetwork {
                    snapshotCanvas
                } else {
                    LivePointCloudCanvas(
                        live: state.live,
                        sample: state.currentFrame,
                        colorMode: state.pointCloudColorMode
                    )
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .fill(LabTheme.surface)
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .stroke(LabTheme.line, lineWidth: 1)
            )

            if state.isolatesNetwork {
                ChangeScrubberView()
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }

    /// Snapshot B only. The camera fits those points once (`fitGeneration` 1).
    private var snapshotCanvas: some View {
        DualPointCloudView(
            midFrame: .constant(state.snapshotMid),
            jtFrame: .constant(state.snapshotJT),
            colorMode: state.pointCloudColorMode,
            pointSize: 3.0,
            fitGeneration: 1,
            overlay: state.overlayGeometry
        )
        .overlay(alignment: .topLeading) {
            if state.snapshotMid != nil || state.snapshotJT != nil {
                Text("snapshot B")
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 4)
                    .background(
                        RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                            .fill(LabTheme.bg.opacity(0.85))
                    )
                    .overlay(
                        RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                            .stroke(LabTheme.line, lineWidth: 1)
                    )
                    .padding(10)
                    .allowsHitTesting(false)
            }
        }
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
            if let line = state.localHeadline {
                Text(line)
                    .font(LabTheme.t(13))
                    .foregroundStyle(state.localFailure == nil ? LabTheme.dim : LabTheme.warn)
                    .fixedSize(horizontal: false, vertical: true)
            } else {
                Text(state.health.headline)
                    .font(LabTheme.t(13))
                    .foregroundStyle(LabTheme.band(state.health.band))
            }
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
                    .disabled(state.isolatesNetwork || !btn.enabled || state.isActing)
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
                    .disabled(state.isolatesNetwork || !RigGate.canArmAtLock(state.rig).0 || state.isActing)

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
                        Text(MissionLog.clock.string(from: event.at))
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

/// Snapshot and Live, the only ways the 3D view touches the network.
private struct LiveStreamControls: View {
    @ObservedObject var live: FoxgloveLiveClient

    var body: some View {
        HStack(spacing: 8) {
            Button("Snapshot") { live.takeSnapshot() }
                .font(LabTheme.t(11, .medium))
                .buttonStyle(QuietButton())
                .disabled(!live.isAvailable || live.isCapturing || live.isLive)
                .help("One frame per sensor, then the connection closes")
            // Follows the session, so an auto-stop flips it back off.
            Toggle("Live", isOn: Binding(
                get: { live.isLive },
                set: { $0 ? live.startLive() : live.stopLive() }
            ))
            .toggleStyle(.switch)
            .controlSize(.mini)
            .font(LabTheme.t(11, .medium))
            .foregroundStyle(LabTheme.dim)
            .disabled(!live.isAvailable || live.isCapturing)
            .help("Stream until switched off, for at most \(Int(LiveSession.liveMaxDuration / 60)) min, and only while the window is visible")
        }
    }
}

/// The 3D canvas: captured clouds (snapshot or live) once there are any, the
/// sample cloud only while OFF — never both, so a sensor that didn't deliver
/// leaves its slot empty rather than backfilled by the sample. The badge says
/// which one is on screen. The renderer pulls captured clouds itself at draw
/// time; this view only follows the mode and the badge.
private struct LivePointCloudCanvas: View {
    @ObservedObject var live: FoxgloveLiveClient
    var sample: PointCloudFrame?
    var colorMode: PointCloudColorMode

    var body: some View {
        let off = live.phase == .off
        DualPointCloudView(
            midFrame: .constant(off && sample?.sensor != .jt128 ? sample : nil),
            jtFrame: .constant(off && sample?.sensor == .jt128 ? sample : nil),
            colorMode: colorMode,
            pointSize: 3.0,
            fitGeneration: live.fitGeneration,
            liveFeed: off ? nil : live.mailbox
        )
        .overlay(alignment: .topLeading) {
            LiveModeBadge(
                isLive: live.isLive,
                text: off ? LiveViewMode.offBadge(sampleShown: sample != nil) : live.badge
            )
            .padding(10)
            // Camera drags start anywhere on the canvas, this corner included.
            .allowsHitTesting(false)
        }
    }
}

private struct LiveModeBadge: View {
    var isLive: Bool
    var text: String

    var body: some View {
        // Green only for live data; anything held or standing in is amber.
        let tint = isLive ? LabTheme.good : LabTheme.warn
        HStack(spacing: 6) {
            Circle()
                .fill(tint)
                .frame(width: LabTheme.pip, height: LabTheme.pip)
            Text(text)
                .font(LabTheme.mono(11))
                .foregroundStyle(tint)
                .lineLimit(1)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .fill(LabTheme.bg.opacity(0.85))
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .stroke(tint.opacity(0.6), lineWidth: 1)
        )
    }
}
