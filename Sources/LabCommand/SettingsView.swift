import SwiftUI
import LabCommandCore

/// Real preferences only. Groups with no controls are omitted.
struct SettingsView: View {
    @EnvironmentObject private var state: AppState
    @State private var endpointDraft = ""
    @State private var endpointError: String?
    @State private var endpointReady = false
    @State private var pendingDowngrade = false
    @State private var rigDiagnosticsOpen = false
    /// Viewport height of the settings scroll view. Zero until it has been laid out.
    @State private var settingsPageHeight: CGFloat = 0

    private static let cleartextToggleID = "cleartextToggle"

    var body: some View {
        ScrollViewReader { proxy in
            settingsBody
                .onAppear { loadEndpointDraft() }
                .onChange(of: state.showCleartextToggle) { _, shown in
                    if shown { revealCleartextToggle(proxy, height: settingsPageHeight) }
                }
                .background {
                    GeometryReader { geo in
                        Color.clear
                            .onAppear { noteSettingsHeight(geo.size.height, proxy: proxy) }
                            .onChange(of: geo.size.height) { _, height in
                                noteSettingsHeight(height, proxy: proxy)
                            }
                    }
                }
        }
    }

    private var settingsBody: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
                Text("Settings")
                    .font(LabTheme.t(LabTheme.titleSize, .semibold))
                    .foregroundStyle(LabTheme.text)
                Text("Room memory address for this Mac. Press Return to save it. The room reconnects if it is already running.")
                    .font(LabTheme.t(13))
                    .foregroundStyle(LabTheme.dim)
                    .fixedSize(horizontal: false, vertical: true)

                group("Room memory") {
                    TextField(RoomMemoryEndpoint.defaultURL.absoluteString, text: $endpointDraft)
                        .font(LabTheme.mono(12))
                        .textFieldStyle(.plain)
                        .foregroundStyle(LabTheme.text)
                        .accessibilityLabel("Room memory endpoint")
                        .onSubmit { commitEndpointDraft(confirmDowngrade: false) }
                        .onChange(of: endpointDraft) { _, _ in
                            if endpointReady && pendingDowngrade { pendingDowngrade = false }
                        }
                        .padding(.horizontal, 8)
                        .padding(.vertical, 6)
                        .background(
                            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                                .fill(LabTheme.well)
                        )
                        .overlay(
                            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                                .stroke(LabTheme.line, lineWidth: 1)
                        )
                        .padding(.vertical, 10)
                    if pendingDowngrade {
                        Text(RoomMemoryEndpoint.downgradePrompt)
                            .font(LabTheme.t(11))
                            .foregroundStyle(LabTheme.warn)
                            .fixedSize(horizontal: false, vertical: true)
                        HStack(spacing: 8) {
                            Button("Use cleartext") { commitEndpointDraft(confirmDowngrade: true) }
                                .buttonStyle(QuietButton())
                            Button("Keep wss://") { cancelDowngrade() }
                                .buttonStyle(QuietButton())
                        }
                        .font(LabTheme.t(12))
                        .padding(.bottom, 8)
                    }
                    if let endpointError {
                        Text(endpointError)
                            .font(LabTheme.t(11))
                            .foregroundStyle(LabTheme.bad)
                            .fixedSize(horizontal: false, vertical: true)
                            .padding(.bottom, 8)
                    }
                    if let block = state.roomEndpointBlock {
                        Text(block)
                            .font(LabTheme.t(11))
                            .foregroundStyle(LabTheme.bad)
                            .fixedSize(horizontal: false, vertical: true)
                            .padding(.bottom, 8)
                    } else if let room = state.room {
                        RoomSocketFailureLine(room: room)
                    }
                    Text("Default \(RoomMemoryEndpoint.defaultURL.absoluteString).")
                        .font(LabTheme.t(11))
                        .foregroundStyle(LabTheme.faint)
                    Rectangle().fill(LabTheme.line).frame(height: 1)
                    Toggle(isOn: Binding(
                        get: { state.allowCleartextRoomWebSocket },
                        set: { state.setAllowCleartextRoomWebSocket($0) }
                    )) {
                        Text("Allow cleartext WebSocket (ws://)")
                            .font(LabTheme.t(13))
                            .foregroundStyle(LabTheme.text)
                    }
                    .toggleStyle(.switch)
                    .controlSize(.small)
                    .padding(.vertical, 10)
                    .id(Self.cleartextToggleID)
                    .accessibilityIdentifier("allowCleartextWebSocket")
                    if state.cleartextFromEnvironment {
                        Text(RoomMemoryEndpoint.cleartextEnvironmentNote)
                            .font(LabTheme.t(11))
                            .foregroundStyle(LabTheme.warn)
                            .fixedSize(horizontal: false, vertical: true)
                            .padding(.bottom, 4)
                    }
                    Text(state.showsCleartextWarning
                         ? RoomMemoryEndpoint.cleartextWarning
                         : RoomMemoryEndpoint.cleartextOffNote)
                        .font(LabTheme.t(11))
                        .foregroundStyle(state.showsCleartextWarning ? LabTheme.warn : LabTheme.faint)
                        .fixedSize(horizontal: false, vertical: true)
                        .padding(.bottom, 10)
                }

                group("Probe") {
                    HStack {
                        Text("Auto-refresh")
                            .font(LabTheme.t(13))
                            .foregroundStyle(LabTheme.text)
                        Spacer()
                        Toggle("", isOn: Binding(
                            get: { state.autoRefreshEnabled },
                            set: { newVal in
                                guard !state.isolatesNetwork else { return }
                                state.autoRefreshEnabled = newVal
                                state.restartTimer()
                            }
                        ))
                        .labelsHidden()
                        .toggleStyle(.switch)
                        .controlSize(.small)
                        .disabled(state.isolatesNetwork)
                        .opacity(state.isolatesNetwork ? LabTheme.disabledOpacity : 1)
                    }
                    .padding(.vertical, 10)

                    Rectangle().fill(LabTheme.line).frame(height: 1)

                    // Was a static label printing the constant. restartTimer()
                    // already honored this value — the control was just missing.
                    HStack {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Interval")
                                .font(LabTheme.t(13))
                                .foregroundStyle(LabTheme.text)
                            Text(intervalNote)
                                .font(LabTheme.t(11))
                                .foregroundStyle(LabTheme.faint)
                        }
                        Spacer()
                        Picker("", selection: Binding(
                            get: { Int(state.probeIntervalSeconds) },
                            set: { newVal in
                                guard !state.isolatesNetwork else { return }
                                state.probeIntervalSeconds = Double(newVal)
                                state.restartTimer()
                            }
                        )) {
                            ForEach([15, 30, 45, 90, 180], id: \.self) { s in
                                Text("\(s)s").tag(s)
                            }
                        }
                        .labelsHidden()
                        .pickerStyle(.segmented)
                        .controlSize(.small)
                        .frame(width: 240)
                        .disabled(state.isolatesNetwork || !state.autoRefreshEnabled)
                        .opacity(state.isolatesNetwork || !state.autoRefreshEnabled ? LabTheme.disabledOpacity : 1)
                        .accessibilityLabel("Probe interval seconds")
                    }
                    .padding(.vertical, 10)
                }

                DisclosureGroup(isExpanded: $rigDiagnosticsOpen) {
                    VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
                        Text("Host and paths for the lidar recorder. Rows in mono are compiled-in constants from CaptureRecipe.")
                            .font(LabTheme.t(11))
                            .foregroundStyle(LabTheme.faint)
                            .fixedSize(horizontal: false, vertical: true)
                        group("Host") {
                            settingsRow(label: "SSH host", value: "spark")
                            settingsRow(label: "Probes", value: "Local Process")
                        }
                        group("Capture") {
                            settingsRow(label: "Foxglove", value: CaptureRecipe.viewConnectURL())
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "Sensor", value: CaptureRecipe.sensorIP)
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "NIC", value: CaptureRecipe.nic)
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "Bag root", value: CaptureRecipe.dataRoot)
                        }
                        group("Paths") {
                            settingsRow(label: "Mac sessions", value: "~/Desktop/lidar-sessions")
                        }
                        group("Keys") {
                            settingsRow(label: "Start sensor", value: "⌘↩")
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "Start recording", value: "⌘⇧R")
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "Stop", value: "⌘.")
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "Refresh", value: "⌘R")
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "Focus", value: "⌘⇧F")
                            Rectangle().fill(LabTheme.line).frame(height: 1)
                            settingsRow(label: "Diagnose", value: "⌘⇧D")
                        }
                    }
                    .padding(.top, 8)
                } label: {
                    Text("Rig diagnostics")
                        .font(LabTheme.t(13, .medium))
                        .foregroundStyle(LabTheme.text)
                }
            }
            .padding(LabTheme.pad)
            .frame(maxWidth: 520, alignment: .leading)
            .frame(maxWidth: .infinity, alignment: .leading)
        }
        .background(LabTheme.bg)
    }

    private func noteSettingsHeight(_ height: CGFloat, proxy: ScrollViewProxy) {
        if settingsPageHeight != height { settingsPageHeight = height }
        revealCleartextToggle(proxy, height: height)
    }

    /// The Room block opens this page on the switch. Showing it does not turn the switch on.
    /// `onAppear` can run before the scroll view has a height. Scrolling then does nothing,
    /// so the request stays until a pass reports a height, and is cleared on the next turn.
    private func revealCleartextToggle(_ proxy: ScrollViewProxy, height: CGFloat) {
        guard state.showCleartextToggle, height > 0 else { return }
        let app = state
        DispatchQueue.main.async {
            guard app.showCleartextToggle else { return }
            proxy.scrollTo(Self.cleartextToggleID, anchor: .center)
            app.cleartextToggleWasShown()
        }
    }

    /// The draft is the process's current endpoint, not a fresh environment resolve.
    private func loadEndpointDraft() {
        endpointDraft = state.roomEndpointURL.absoluteString
        endpointError = nil
        pendingDowngrade = false
        endpointReady = true
    }

    /// The same dial failure the Room screen shows, once the address is allowed to connect.
    private struct RoomSocketFailureLine: View {
        @ObservedObject var room: LiveRoomClient

        var body: some View {
            if let line = RoomConnectionFailure.detail(connection: room.connection, urlText: room.url.absoluteString) {
                Text(line)
                    .font(LabTheme.t(11))
                    .foregroundStyle(LabTheme.bad)
                    .fixedSize(horizontal: false, vertical: true)
                    .padding(.bottom, 8)
            }
        }
    }

    /// Return saves. A `wss://` to `ws://` change stays unsaved until "Use cleartext".
    private func commitEndpointDraft(confirmDowngrade: Bool) {
        guard endpointReady else { return }
        switch state.commitRoomEndpoint(endpointDraft, confirmCleartextDowngrade: confirmDowngrade) {
        case .saved:
            pendingDowngrade = false
            endpointError = nil
            endpointDraft = state.roomEndpointURL.absoluteString
        case .rejected(let message):
            pendingDowngrade = false
            endpointError = message
        case .needsCleartextConfirmation:
            endpointError = nil
            pendingDowngrade = true
        }
    }

    private func cancelDowngrade() {
        pendingDowngrade = false
        endpointError = nil
        endpointDraft = state.roomEndpointURL.absoluteString
    }

    /// A probe that measures point rate costs ~4 s on the Spark, so the cheap
    /// intervals are not free. Say that here rather than letting the operator
    /// discover it as lag.
    private var intervalNote: String {
        if state.isolatesNetwork { return "local snapshots — probes stay off" }
        if !state.autoRefreshEnabled { return "auto-refresh off — ⌘R to probe" }
        let s = Int(state.probeIntervalSeconds)
        if s <= 15 { return "measures rates every \(s)s — heavy while streaming" }
        return "how often Spark is probed"
    }

    private func group<Content: View>(_ title: String, @ViewBuilder content: () -> Content) -> some View {
        VStack(alignment: .leading, spacing: 10) {
            SectionLabel(text: title)
            VStack(alignment: .leading, spacing: 0) {
                content()
            }
            .padding(.horizontal, 14)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .fill(LabTheme.surface.opacity(0.55))
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .stroke(LabTheme.line, lineWidth: 1)
            )
        }
    }

    private func settingsRow(label: String, value: String) -> some View {
        HStack {
            Text(label)
                .font(LabTheme.t(13))
                .foregroundStyle(LabTheme.text)
            Spacer()
            Text(value)
                .font(LabTheme.mono(12))
                .foregroundStyle(LabTheme.dim)
                .textSelection(.enabled)
        }
        .padding(.vertical, 10)
    }
}
