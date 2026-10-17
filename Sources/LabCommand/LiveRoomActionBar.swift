import SwiftUI
import LabCommandCore

/// The selected event's actions (protocol D46): one line about it, where it is on the map, the
/// buttons its state allows, and what became of the last action on it. The Spark owns the state:
/// nothing here changes the mirror; a button only sends.
struct LiveRoomActionBar: View {
    var mirror: RoomMemoryMirror
    var selection: String?
    var desk: LiveRoomOperator
    var operatorPath: Bool
    /// Why the map shows no highlight for the selection; nil when it does, or nothing is selected.
    var locationNote: String?
    /// Nil in fixture mode: nothing to send to.
    var act: ((LiveRoomOperator.Kind, LiveRoomEvent) -> Void)?
    var retry: ((String) -> Void)?

    @State private var accepting: LiveRoomEvent?

    var body: some View {
        HStack(alignment: .top, spacing: 10) {
            if let (entry, earlier) = LiveRoomOperator.entry(for: selection, mirror: mirror) {
                VStack(alignment: .leading, spacing: 2) {
                    Text(LiveRoomOperator.summary(entry))
                        .font(LabTheme.mono(11))
                        .foregroundStyle(LabTheme.text)
                        .lineLimit(1)
                    if earlier {
                        note("from an earlier run · the Spark no longer tracks it")
                    } else if let locationNote {
                        note(locationNote)
                    }
                    if let pending = desk.latest(for: entry.event.eventId) {
                        HStack(spacing: 6) {
                            Text(LiveRoomOperator.describe(pending))
                                .font(LabTheme.mono(11))
                                .foregroundStyle(tone(pending.phase))
                                .lineLimit(2)
                            if let retry, desk.retry(actionId: pending.action.actionId, mirror: mirror) != nil {
                                Button("Retry") { retry(pending.action.actionId) }
                                    .buttonStyle(GhostButton())
                                    .help("Send the same action again; the Spark acts at most once")
                            }
                        }
                    }
                }
                Spacer(minLength: 8)
                let buttons = LiveRoomOperator.buttons(for: entry, earlier: earlier, mirror: mirror, operatorPath: operatorPath && act != nil,
                                                       pending: desk.latest(for: entry.event.eventId))
                ForEach(buttons, id: \.kind) { button in
                    Button(button.kind.title) {
                        if button.kind == .accept {
                            accepting = entry
                        } else {
                            act?(button.kind, entry)
                        }
                    }
                    .buttonStyle(QuietButton())
                    .font(LabTheme.t(11, .medium))
                    .disabled(!button.enabled)
                    .opacity(button.enabled ? 1 : LabTheme.disabledOpacity)
                    .help(button.reason ?? help(button.kind))
                }
            } else {
                note("Select an event to confirm, dismiss or accept it")
                Spacer(minLength: 0)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .confirmationDialog("Make this the new normal?", isPresented: Binding(
            get: { accepting != nil },
            set: { if !$0 { accepting = nil } }
        ), presenting: accepting) { entry in
            Button("Accept as normal") { act?(.accept, entry) }
            Button("Cancel", role: .cancel) {}
        } message: { _ in
            Text("The Spark updates its baseline for this spot. There is no undo yet.")
        }
    }

    private func note(_ text: String) -> some View {
        Text(text)
            .font(LabTheme.mono(11))
            .foregroundStyle(LabTheme.dim)
            .lineLimit(2)
    }

    private func tone(_ phase: LiveRoomOperator.Phase) -> Color {
        switch phase {
        case .refused, .unknown, .unsent: return LabTheme.warn
        case .sending, .applied, .done: return LabTheme.dim
        }
    }

    private func help(_ kind: LiveRoomOperator.Kind) -> String {
        switch kind {
        case .confirm: return "Mark this as a real change"
        case .dismiss: return "Not a change: the Spark stops reporting it until the spot is seen clear"
        case .accept: return "Make this the new normal: the Spark updates its baseline"
        }
    }
}
