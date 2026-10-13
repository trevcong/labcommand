import SwiftUI
import LabCommandCore

/// Hardware status. Cards first; probe log available for the exact SSH/Tailscale line.
struct DevicesView: View {
    @EnvironmentObject private var state: AppState
    @State private var showProbes = true

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
                VStack(alignment: .leading, spacing: 8) {
                    Text("Hardware")
                        .font(LabTheme.t(LabTheme.titleSize, .semibold))
                        .foregroundStyle(LabTheme.text)
                    Text("This Mac reaches Spark over Tailscale (`ssh spark`); both lidars live on Spark's wired port only. A gray sensor is unpowered — normal when parked. The Spark watchdog restarts the drivers when a sensor is plugged back in.")
                        .font(LabTheme.t(13))
                        .foregroundStyle(LabTheme.dim)
                        .fixedSize(horizontal: false, vertical: true)
                }

                LazyVGrid(columns: [GridItem(.flexible(), spacing: 12), GridItem(.flexible(), spacing: 12)], spacing: 12) {
                    deviceCard(
                        name: "Spark",
                        detail: state.detail("spark_ssh"),
                        status: state.snapshot.spark,
                        statusID: "sparkStatus"
                    )
                    deviceCard(
                        name: "Tailscale",
                        detail: state.detail("tailscale"),
                        status: state.snapshot.tailscale,
                        statusID: "tailscaleStatus"
                    )
                    deviceCard(
                        name: "Livox Mid-360",
                        detail: state.rig.mid360.detail,
                        status: sensorStatus(state.rig.mid360),
                        statusID: "mid360Status"
                    )
                    deviceCard(
                        name: "Hesai JT128",
                        detail: jtDetail,
                        status: sensorStatus(state.rig.jt128),
                        statusID: "jt128Status"
                    )
                }

                containerStrip

                dossiers

                if !state.rig.events.isEmpty {
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Watchdog events")
                            .font(LabTheme.label())
                            .foregroundStyle(LabTheme.faint)
                            .textCase(.uppercase)
                            .tracking(0.7)
                        VStack(alignment: .leading, spacing: 4) {
                            ForEach(Array(state.rig.events.enumerated()), id: \.offset) { _, line in
                                Text(line)
                                    .font(LabTheme.mono(11))
                                    .foregroundStyle(LabTheme.dim)
                                    .textSelection(.enabled)
                            }
                        }
                        .padding(12)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(
                            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                                .fill(LabTheme.well)
                        )
                        .overlay(
                            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                                .stroke(LabTheme.line, lineWidth: 1)
                        )
                    }
                    .accessibilityIdentifier("watchdogEvents")
                }

                HStack(spacing: 8) {
                    Circle()
                        .fill(LabTheme.good)
                        .frame(width: 6, height: 6)
                    Text("This Mac")
                        .font(LabTheme.t(13, .medium))
                        .foregroundStyle(LabTheme.text)
                        .accessibilityIdentifier("macStatus")
                    Text("local")
                        .font(LabTheme.t(12))
                        .foregroundStyle(LabTheme.dim)
                }

                Button(showProbes ? "Hide probe log" : "Show probe log") {
                    showProbes.toggle()
                }
                .buttonStyle(GhostButton())

                if showProbes, !state.snapshot.checks.isEmpty {
                    VStack(alignment: .leading, spacing: 10) {
                        ForEach(Array(state.snapshot.checks.enumerated()), id: \.offset) { _, c in
                            VStack(alignment: .leading, spacing: 4) {
                                HStack(alignment: .top, spacing: 12) {
                                    Text(c.id)
                                        .font(LabTheme.mono(12))
                                        .foregroundStyle(LabTheme.dim)
                                        .frame(width: 100, alignment: .leading)
                                    Text(c.kind.rawValue)
                                        .font(LabTheme.t(12, .medium))
                                        .foregroundStyle(LabTheme.status(StatusAggregator.status(for: c)))
                                        .frame(width: 72, alignment: .leading)
                                    Text(c.detail)
                                        .font(LabTheme.t(12))
                                        .foregroundStyle(LabTheme.text)
                                        .textSelection(.enabled)
                                        .frame(maxWidth: .infinity, alignment: .leading)
                                }
                                if let guide = DeviceRecovery.guidance(probeID: c.id, outcome: c) {
                                    Text(guide)
                                        .font(LabTheme.t(12))
                                        .foregroundStyle(LabTheme.dim)
                                        .padding(.leading, 112)
                                        .fixedSize(horizontal: false, vertical: true)
                                }
                            }
                            .padding(.vertical, 4)
                        }
                    }
                    .padding(14)
                    .background(
                        RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                            .fill(LabTheme.well)
                    )
                    .overlay(
                        RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                            .stroke(LabTheme.line, lineWidth: 1)
                    )
                }
            }
            .padding(LabTheme.pad)
            .frame(maxWidth: 840, alignment: .leading)
            .frame(maxWidth: .infinity, alignment: .center)
        }
        .background(LabTheme.bg)
        .accessibilityIdentifier("summaryLine")
    }

    /// The crazy-spec layer: full dossiers per device, straight from the
    /// vendor manuals + the dual-rig research. Static facts, never telemetry.
    private var dossiers: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("Spec dossiers")
                .font(LabTheme.label())
                .foregroundStyle(LabTheme.faint)
                .textCase(.uppercase)
                .tracking(0.7)
            HStack(alignment: .top, spacing: 12) {
                dossierCard("Livox Mid-360", role: "base sensor · dome · fusion IMU",
                            lines: RigSpec.dossier(.mid360))
                dossierCard("Hesai JT128", role: "top sensor · hyper-hemisphere · 128 ch",
                            lines: RigSpec.dossier(.jt128))
                dossierCard("DGX Spark", role: "brain · drivers · PTP grandmaster",
                            lines: RigSpec.spark)
            }
        }
        .accessibilityIdentifier("specDossiers")
    }

    private func dossierCard(_ name: String, role: String, lines: [RigSpec.Line]) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            Text(name)
                .font(LabTheme.t(13, .semibold))
                .foregroundStyle(LabTheme.text)
            Text(role)
                .font(LabTheme.mono(10.5))
                .foregroundStyle(LabTheme.accent)
            ForEach(Array(lines.enumerated()), id: \.offset) { _, line in
                HStack(alignment: .firstTextBaseline, spacing: 8) {
                    Text(line.label)
                        .font(LabTheme.label())
                        .foregroundStyle(LabTheme.faint)
                        .textCase(.uppercase)
                        .frame(width: 62, alignment: .leading)
                    Text(line.value)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(line.caution ? LabTheme.warn : LabTheme.dim)
                        .fixedSize(horizontal: false, vertical: true)
                        .textSelection(.enabled)
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
    }

    private func sensorStatus(_ sensor: SensorReading) -> ConnectionStatus {
        switch sensor.state {
        case .streaming, .powered: return .connected
        case .dark: return .disconnected
        case .unknown: return .unknown
        }
    }

    private var jtDetail: String {
        var d = state.rig.jt128.detail
        if let ptc = state.rig.jt128.ptcOpen {
            d += ptc ? " · ptc open" : " · ptc closed"
        }
        return d
    }

    /// The three Spark-side containers this rig depends on.
    private var containerStrip: some View {
        HStack(spacing: 10) {
            ForEach(["lidar-dual", "ptp-master", "lidar-watchdog", "lidar-center"], id: \.self) { name in
                let raw = name == "lidar-center"
                    ? (state.rig.apiReachable ? "running" : "unreachable")
                    : (state.rig.containers[name] ?? (state.rig.apiReachable ? "missing" : "—"))
                let ok = raw == "running"
                HStack(spacing: 6) {
                    Circle()
                        .fill(ok ? LabTheme.good : (raw == "—" ? LabTheme.unknown : LabTheme.warn))
                        .frame(width: 6, height: 6)
                    Text(name)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.dim)
                    Text(raw)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(ok ? LabTheme.good : LabTheme.faint)
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .background(
                    RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                        .fill(LabTheme.well)
                )
                .overlay(
                    RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                        .stroke(LabTheme.line, lineWidth: 1)
                )
            }
            Spacer(minLength: 0)
        }
        .accessibilityIdentifier("containerStrip")
    }

    private func deviceCard(name: String, detail: String, status: ConnectionStatus, statusID: String) -> some View {
        let probeID = name
        let guide = DeviceRecovery.guidance(probeID: probeID, status: status, detail: detail)
        return VStack(alignment: .leading, spacing: 10) {
            HStack(spacing: 8) {
                Circle()
                    .fill(LabTheme.status(status))
                    .frame(width: 8, height: 8)
                Text(status.displayLabel)
                    .font(LabTheme.t(12, .medium))
                    .foregroundStyle(LabTheme.status(status))
                    .accessibilityLabel(status.accessibilityLabel)
                    .accessibilityIdentifier(statusID)
            }
            Text(name)
                .font(LabTheme.t(13, .semibold))
                .foregroundStyle(LabTheme.text)
            Text(detail)
                .font(LabTheme.t(12))
                .foregroundStyle(LabTheme.dim)
                .textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
            if let guide, !detail.isEmpty, !guide.localizedCaseInsensitiveContains(detail.prefix(40)) {
                Text(guide)
                    .font(LabTheme.t(12))
                    .foregroundStyle(LabTheme.dim)
                    .fixedSize(horizontal: false, vertical: true)
                    .accessibilityIdentifier("\(probeID)-recovery")
            }
        }
        .padding(16)
        .frame(maxWidth: .infinity, minHeight: 132, alignment: .topLeading)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.surface)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
    }
}
