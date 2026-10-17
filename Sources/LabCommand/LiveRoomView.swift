import SwiftUI
import LabCommandCore

/// Room destination. Binds to the client or a fixture. It never starts the socket.
struct LiveRoomPage: View {
    @EnvironmentObject private var state: AppState

    var body: some View {
        if let room = state.room {
            LiveRoomClientHost(room: room)
        } else {
            LiveRoomBody(
                mirror: state.roomMirror,
                connection: state.roomConnection,
                url: nil,
                started: state.roomDidStart,
                requestPathExists: state.requestPathExists,
                showsFixture: state.roomLaunch.fixtureMode != nil,
                fixtureMode: state.fixture?.mode ?? state.roomLaunch.fixtureMode,
                recorder: .once(state.roomMirror),
                desk: LiveRoomOperator(),
                operatorPath: false,
                act: nil,
                retry: nil
            )
        }
    }
}

private struct LiveRoomClientHost: View {
    @ObservedObject var room: LiveRoomClient
    @EnvironmentObject private var state: AppState

    var body: some View {
        LiveRoomBody(
            mirror: room.mirror,
            connection: room.connection,
            url: room.url.absoluteString,
            started: state.roomDidStart,
            requestPathExists: state.requestPathExists,
            showsFixture: false,
            fixtureMode: nil,
            recorder: room.timelineRecorder,
            desk: room.operatorDesk,
            operatorPath: room.operatorPath,
            act: { kind, entry in _ = room.act(kind, on: entry) },
            retry: { actionId in _ = room.retry(actionId: actionId) }
        )
    }
}

private struct CanvasPixels: Equatable {
    var x: Int
    var y: Int
    var w: Int
    var h: Int
}

private struct LiveRoomBody: View {
    @EnvironmentObject private var state: AppState
    var mirror: RoomMemoryMirror
    var connection: LiveRoomConnection
    var url: String?
    var started: Bool
    var requestPathExists: Bool
    var showsFixture: Bool
    var fixtureMode: LiveRoomFixture.Mode?
    var recorder: LiveRoomTimelineRecorder
    var desk: LiveRoomOperator
    var operatorPath: Bool
    /// Nil in fixture mode: nothing to send to.
    var act: ((LiveRoomOperator.Kind, LiveRoomEvent) -> Void)?
    var retry: ((String) -> Void)?

    @State private var enabled: Set<RoomMemory.ChangeEvent.Kind> = Set(RoomMemory.ChangeEvent.Kind.allCases)
    @State private var selection: String?
    @State private var historyOpen = false
    @State private var canvas: CanvasPixels?
    @State private var timelineWindow: TimeInterval = LiveRoomTimeline.windows[1]
    /// The timeline's right edge while it does not follow now.
    @State private var timelineEnd: Double?

    private var screen: LiveRoomScreen {
        LiveRoomScreen.make(
            mirror: mirror,
            connection: connection,
            requestPathExists: requestPathExists,
            started: started,
            now: ProcessInfo.processInfo.systemUptime,
            kinds: enabled,
            selection: selection
        )
    }

    var body: some View {
        let screen = screen
        let halo = LiveRoomTimeline.halo(selection: selection, mirror: mirror)
        VStack(alignment: .leading, spacing: 8) {
            header(screen)
            if let banner = screen.banner {
                Text(banner.text)
                    .font(LabTheme.t(13))
                    .foregroundStyle(toneColor(banner.tone))
                    .lineLimit(2)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
            if let restart = screen.restartBanner {
                Text(restart.text)
                    .font(LabTheme.t(13))
                    .foregroundStyle(toneColor(restart.tone))
                    .lineLimit(2)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
            HStack(alignment: .top, spacing: 12) {
                VStack(alignment: .leading, spacing: 8) {
                    canvasStack(screen, halo: halo)
                    Text(screen.statusLine)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.dim)
                        .lineLimit(1)
                        .frame(maxWidth: .infinity, alignment: .leading)
                    LiveRoomActionBar(mirror: mirror, selection: selection, desk: desk, operatorPath: operatorPath,
                                      locationNote: halo.note, act: act, retry: retry)
                    timelineSection
                }
                .frame(maxWidth: .infinity, maxHeight: .infinity)
                eventColumn(screen)
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .padding(LabTheme.pad)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
        .background(LabTheme.bg)
        .onAppear { writeSidecar(screen) }
        .onChange(of: sidecarText(screen)) { _, _ in writeSidecar(screen) }
        .onChange(of: selection) { _, id in
            // Picked on the timeline: show its row, even when it is in the closed history.
            if let id, screen.historyRows.contains(where: { $0.id == id }) { historyOpen = true }
        }
    }

    /// Redrawn every second so "now" moves while no message arrives.
    private var timelineSection: some View {
        TimelineView(.periodic(from: .now, by: 1)) { _ in
            LiveRoomTimelineView(
                timeline: timeline(now: ProcessInfo.processInfo.systemUptime),
                selection: $selection,
                window: $timelineWindow,
                follow: Binding(
                    get: { timelineEnd == nil },
                    set: { follow in
                        timelineEnd = follow ? nil : timeline(now: ProcessInfo.processInfo.systemUptime).end
                    }
                )
            )
        }
    }

    private func timeline(now: TimeInterval) -> LiveRoomTimeline {
        LiveRoomTimeline.make(mirror: mirror, recorder: recorder, window: timelineWindow, end: timelineEnd,
                              selection: selection, kinds: enabled, timeZone: .current, now: now)
    }

    private func header(_ screen: LiveRoomScreen) -> some View {
        HStack(spacing: 10) {
            Text("Room")
                .font(LabTheme.t(LabTheme.titleSize, .semibold))
                .foregroundStyle(LabTheme.text)
            ActivityPip(pip: screen.pip)
            if showsFixture {
                Text("Fixture")
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .overlay(
                        RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                            .stroke(LabTheme.line, lineWidth: 1)
                    )
                    .help("Hand goldens. No rig contact.")
            }
            Spacer(minLength: 8)
            if showsFixture, let fixtureMode {
                Picker("Fixture source", selection: Binding(
                    get: { fixtureMode },
                    set: { state.selectFixture($0) }
                )) {
                    Text("Map").tag(LiveRoomFixture.Mode.map)
                    Text("Status").tag(LiveRoomFixture.Mode.status)
                    Text("Stale").tag(LiveRoomFixture.Mode.stale)
                    Text("Lost").tag(LiveRoomFixture.Mode.lost)
                    Text("Restart").tag(LiveRoomFixture.Mode.restart)
                    Text("Legacy").tag(LiveRoomFixture.Mode.legacy)
                    Text("Off").tag(LiveRoomFixture.Mode.unavailable)
                }
                .pickerStyle(.segmented)
                .labelsHidden()
                .frame(width: 480)
                .font(LabTheme.t(11))
            }
        }
    }

    @ViewBuilder
    private func canvasStack(_ screen: LiveRoomScreen, halo: LiveRoomTimeline.Halo) -> some View {
        ZStack {
            if screen.canvasSentence == "Room memory unavailable" {
                LabTheme.bg
            } else {
                RoomMapView(
                    mirror: mirror,
                    markers: LiveRoomTimeline.markers(screen.markers, with: halo),
                    opacity: screen.brickOpacity,
                    drawMap: screen.canvasSentence == nil,
                    onCanvasRect: { x, y, w, h in
                        let next = CanvasPixels(x: x, y: y, w: w, h: h)
                        if canvas != next { canvas = next }
                    }
                )
            }
            if let sentence = screen.canvasSentence {
                Text(sentence)
                    .font(LabTheme.t(13))
                    .foregroundStyle(LabTheme.dim)
                    .multilineTextAlignment(.center)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .fill(LabTheme.bg)
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
        .clipShape(RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous))
    }

    private func eventColumn(_ screen: LiveRoomScreen) -> some View {
        VStack(alignment: .leading, spacing: 10) {
            kindChips
            if screen.columnIsUnavailable {
                Text(screen.caption)
                    .font(LabTheme.t(13))
                    .foregroundStyle(LabTheme.dim)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
            } else {
                Text(screen.caption)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .lineLimit(2)
                    .fixedSize(horizontal: false, vertical: true)
                if let line = screen.historyLine {
                    Text(line)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(line.hasPrefix("history incomplete") || line.hasPrefix("history unverified") ? LabTheme.warn : LabTheme.dim)
                        .lineLimit(2)
                        .fixedSize(horizontal: false, vertical: true)
                }
                if let line = screen.standaloneResync {
                    Text(line)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.warn)
                        .lineLimit(2)
                        .fixedSize(horizontal: false, vertical: true)
                }
                ScrollView {
                    VStack(alignment: .leading, spacing: 12) {
                        if !screen.openRows.isEmpty {
                            sectionLabel("Events")
                            ForEach(screen.openRows) { row in
                                rowButton(row)
                            }
                        }
                        if !screen.earlierRows.isEmpty {
                            sectionLabel("Earlier run")
                            ForEach(screen.earlierRows) { row in
                                rowButton(row)
                            }
                        }
                        if !screen.historyRows.isEmpty {
                            DisclosureGroup(isExpanded: $historyOpen) {
                                VStack(alignment: .leading, spacing: 12) {
                                    ForEach(screen.historyRows) { row in
                                        rowButton(row)
                                    }
                                }
                                .padding(.top, 6)
                            } label: {
                                sectionLabel("History")
                            }
                            .font(LabTheme.t(11))
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                }
            }
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 12)
        .frame(width: 280)
        .frame(maxHeight: .infinity, alignment: .topLeading)
        .background(LabTheme.well)
        .clipShape(RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
    }

    private var kindChips: some View {
        HStack(spacing: 6) {
            ForEach(RoomMemory.ChangeEvent.Kind.allCases, id: \.self) { kind in
                let on = enabled.contains(kind)
                Button {
                    if on { enabled.remove(kind) } else { enabled.insert(kind) }
                } label: {
                    Text(chipTitle(kind))
                        .font(LabTheme.mono(11))
                        .foregroundStyle(on ? kindColor(kind) : LabTheme.dim)
                        .lineLimit(1)
                }
                .buttonStyle(.plain)
                .help(on ? "Hide \(chipTitle(kind).lowercased())" : "Show \(chipTitle(kind).lowercased())")
            }
            Spacer(minLength: 0)
        }
    }

    private func rowButton(_ row: LiveRoomEventRow) -> some View {
        Button {
            selection = selection == row.id ? nil : row.id
        } label: {
            VStack(alignment: .leading, spacing: 2) {
                HStack(alignment: .top, spacing: 6) {
                    Circle()
                        .fill(kindColor(row.kind))
                        .frame(width: 6, height: 6)
                        .padding(.top, 4)
                    Text(row.line1)
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.text)
                        .lineLimit(2)
                        .fixedSize(horizontal: false, vertical: true)
                        .frame(maxWidth: .infinity, alignment: .leading)
                }
                Text(row.line2)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.dim)
                    .lineLimit(3)
                    .fixedSize(horizontal: false, vertical: true)
                    .frame(maxWidth: .infinity, alignment: .leading)
            }
            .padding(.vertical, 4)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(
                RoundedRectangle(cornerRadius: 4, style: .continuous)
                    .fill(selection == row.id ? LabTheme.accentMuted : Color.clear)
            )
        }
        .buttonStyle(.plain)
    }

    private func sectionLabel(_ text: String) -> some View {
        Text(text)
            .font(LabTheme.t(11, .medium))
            .foregroundStyle(LabTheme.faint)
    }

    private func sidecarText(_ screen: LiveRoomScreen) -> String {
        let rect = canvas.map { ($0.x, $0.y, $0.w, $0.h) }
        let selected = LiveRoomOperator.entry(for: selection, mirror: mirror)
        let buttons = LiveRoomOperator.buttons(for: selected?.entry, earlier: selected?.earlier ?? false, mirror: mirror,
                                               operatorPath: operatorPath && act != nil,
                                               pending: selected.flatMap { desk.latest(for: $0.entry.event.eventId) })
        let below = LiveRoomTimeline.sidecarLines(timeline: timeline(now: ProcessInfo.processInfo.systemUptime),
                                                  halo: LiveRoomTimeline.halo(selection: selection, mirror: mirror), buttons: buttons)
        return LiveRoomSidecar.render(screen: screen, connection: connection, url: url, canvas: rect)
            + below.map { $0 + "\n" }.joined()
    }

    private func writeSidecar(_ screen: LiveRoomScreen) {
        guard let path = LiveRoomSidecar.outputPath() else { return }
        let file = URL(fileURLWithPath: path)
        try? FileManager.default.createDirectory(at: file.deletingLastPathComponent(), withIntermediateDirectories: true)
        try? sidecarText(screen).write(to: file, atomically: true, encoding: .utf8)
    }

    private func toneColor(_ tone: LiveRoomTone) -> Color {
        switch tone {
        case .warn: return LabTheme.warn
        case .bad: return LabTheme.bad
        case .dim: return LabTheme.dim
        }
    }

    private func kindColor(_ kind: RoomMemory.ChangeEvent.Kind) -> Color {
        switch kind {
        case .appeared: return LabTheme.good
        case .disappeared: return LabTheme.bad
        case .moved: return LabTheme.warn
        }
    }

    private func chipTitle(_ kind: RoomMemory.ChangeEvent.Kind) -> String {
        switch kind {
        case .appeared: return "Appeared"
        case .disappeared: return "Disappeared"
        case .moved: return "Moved"
        }
    }
}

private struct ActivityPip: View {
    var pip: LiveRoomPip
    @State private var dimmed = false

    var body: some View {
        Circle()
            .fill(color)
            .frame(width: LabTheme.pip, height: LabTheme.pip)
            .opacity(pip.pulses && dimmed ? 0.55 : 1)
            .onAppear(perform: animate)
            .onChange(of: pip) { _, _ in animate() }
    }

    private var color: Color {
        switch pip {
        case .good: return LabTheme.good
        case .warn: return LabTheme.warn
        case .bad: return LabTheme.bad
        case .idle: return LabTheme.idle
        }
    }

    private func animate() {
        guard pip.pulses else {
            dimmed = false
            return
        }
        dimmed = false
        withAnimation(.easeInOut(duration: 0.8).repeatForever(autoreverses: true)) {
            dimmed = true
        }
    }
}
