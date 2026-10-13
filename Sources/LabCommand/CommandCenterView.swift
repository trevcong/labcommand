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

    /// Sensor chip: green live, amber powered, quiet gray dark (parking is
    /// normal), dim unknown. Never red for a deliberate unplug.
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
            if state.focusMode {
                RigView()
            } else {
                switch state.dest {
                case .workspace: RigView()
                case .devices: DevicesView()
                case .research: ResearchView()
                case .settings: SettingsView()
                }
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
}
