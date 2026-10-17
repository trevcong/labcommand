import Foundation

/// The operator's actions on the room memory (protocol §3.6, D46): building them, following each
/// one until the Spark's own transition shows its effect, and which buttons the Room tab offers.
///
/// Pure state, like the mirror: the client feeds it sends, service responses, call failures,
/// disconnects, mirror changes and clock ticks. It never changes the mirror. The Spark owns event
/// state, so an action is finished by the transition that arrives on the wire, whatever the
/// response said or whether it came at all.
public struct LiveRoomOperator: Sendable, Equatable {
    /// No response by then: the outcome is unknown.
    public static let responseTimeout: TimeInterval = 10
    /// Applied, but no transition by then (lost on the way, or not published): the outcome is unknown.
    public static let transitionTimeout: TimeInterval = 60
    /// Actions kept for display, newest last.
    public static let capacity = 50

    public enum Kind: String, Sendable, CaseIterable {
        case confirm, dismiss, accept

        public var action: RoomMemory.OperatorAction.Action {
            switch self {
            case .confirm: return .confirm
            case .dismiss: return .dismiss
            case .accept: return .accept
            }
        }

        /// The state the Spark sends when this action took effect.
        var effect: RoomMemory.ChangeEvent.State {
            switch self {
            case .confirm: return .confirmed
            case .dismiss: return .reverted
            case .accept: return .accepted
            }
        }

        public var title: String {
            switch self {
            case .confirm: return "Confirm"
            case .dismiss: return "Dismiss"
            case .accept: return "Accept as normal"
            }
        }

        /// What the operator did, once the Spark shows it.
        var doneWord: String {
            switch self {
            case .confirm: return "confirmed by you"
            case .dismiss: return "dismissed by you"
            case .accept: return "saved as normal by you"
            }
        }
    }

    public enum Phase: Sendable, Equatable {
        /// On its way; no response yet.
        case sending(since: TimeInterval)
        /// The Spark acted; its transition has not arrived yet.
        case applied(since: TimeInterval)
        /// The Spark's transition arrived. The text says what it shows.
        case done(String)
        /// The Spark refused it; nothing changed.
        case refused(RoomMemory.OperatorOutcome)
        /// It never left this Mac.
        case unsent(String)
        /// It may or may not have acted (no answer, a lost connection, a restart). Retry repeats
        /// the same action id, which the Spark answers from memory if it already acted.
        case unknown(String)

        /// Blocks another action on the same event.
        public var inFlight: Bool {
            switch self {
            case .sending, .applied: return true
            default: return false
            }
        }
    }

    public struct Pending: Sendable, Equatable {
        public var action: RoomMemory.OperatorAction
        public var kind: Kind
        /// The event's highest transition seq this Mac held when it sent; a later one is news.
        public var seqAtSend: UInt64?
        public var phase: Phase
        public var attempts: Int

        public var eventId: UInt64 { action.eventId ?? RoomMemory.OperatorAction.eventNone }
        public var generation: UInt64 { action.expectedGeneration }
    }

    public struct Button: Sendable, Equatable {
        public var kind: Kind
        public var enabled: Bool
        /// Why it is disabled; nil when enabled.
        public var reason: String?
    }

    public private(set) var actions: [Pending] = []
    /// This client's action counter (`OperatorAction.seq`).
    public private(set) var nextSeq: UInt64 = 0

    public init() {}

    // MARK: Building and sending

    /// The action for `kind` on `entry` as the operator sees it: the mirror's map epoch, its
    /// active generation and the event's shown state. Nil when the mirror has no verified
    /// generation, or the event's state has no such action.
    public static func make(_ kind: Kind, entry: LiveRoomEvent, mirror: RoomMemoryMirror, seq: UInt64, operatorId: String,
                            stamp: RoomMemory.Time, actionId: String, text: String = "") -> RoomMemory.OperatorAction? {
        guard let generation = mirror.activeGeneration else { return nil }
        let roomId = mirror.roomId ?? entry.event.header.roomId
        let state = entry.event.state
        let allowed: Set<RoomMemory.ChangeEvent.State>
        switch kind {
        case .confirm: allowed = [.candidate]
        case .dismiss: allowed = [.candidate, .confirmed]
        case .accept: allowed = [.confirmed]
        }
        guard allowed.contains(state) else { return nil }
        return RoomMemory.OperatorAction(
            header: RoomMemory.Header(roomId: roomId, mapEpoch: mirror.mapEpoch ?? 0, seq: seq, stamp: stamp),
            action: kind.action, eventId: entry.event.eventId, expectedState: state, expectedGeneration: generation,
            text: text, operatorId: operatorId, actionId: actionId
        )
    }

    /// The next action counter; call once per new action.
    public mutating func takeSeq() -> UInt64 {
        defer { nextSeq &+= 1 }
        return nextSeq
    }

    /// `action` went out (`delivered`) or could not be sent.
    public mutating func sent(_ action: RoomMemory.OperatorAction, kind: Kind, mirror: RoomMemoryMirror, delivered: Bool,
                              at now: TimeInterval) {
        let seq = action.eventId.flatMap { mirror.eventsById[$0]?.seq }
        let phase: Phase = delivered ? .sending(since: now) : .unsent("not sent: this Spark offers no operator service")
        if let index = actions.firstIndex(where: { $0.action.actionId == action.actionId }) {
            actions[index].phase = phase
            actions[index].attempts += 1
        } else {
            actions.append(Pending(action: action, kind: kind, seqAtSend: seq, phase: phase, attempts: 1))
            if actions.count > Self.capacity { actions.removeFirst(actions.count - Self.capacity) }
        }
    }

    /// The service's response; nil when it was unreadable.
    public mutating func response(actionId: String, outcome: RoomMemory.OperatorOutcome?, at now: TimeInterval) {
        guard let index = actions.firstIndex(where: { $0.action.actionId == actionId }) else { return }
        // The transition already showed the effect: the response adds nothing.
        if case .done = actions[index].phase { return }
        guard let outcome else {
            actions[index].phase = .unknown("the Spark's answer was unreadable")
            return
        }
        if outcome == .applied {
            actions[index].phase = .applied(since: now)
        } else if (outcome == .stateMismatch || outcome == .unknownEvent), actions[index].attempts > 1 {
            // An earlier attempt may have acted and moved the event on; the Spark has forgotten its id.
            actions[index].phase = .unknown("the event has moved on since an earlier attempt; check it")
        } else {
            actions[index].phase = .refused(outcome)
        }
    }

    /// The bridge reported that the call did not complete.
    public mutating func callFailed(actionId: String, reason: String) {
        guard let index = actions.firstIndex(where: { $0.action.actionId == actionId }),
              actions[index].phase.inFlight else { return }
        actions[index].phase = .unknown("the bridge could not complete the call (\(reason))")
    }

    /// The socket closed: every action still waiting for an answer may or may not have acted.
    public mutating func connectionLost() {
        for index in actions.indices {
            if case .sending = actions[index].phase {
                actions[index].phase = .unknown("the connection closed before the Spark answered")
            }
        }
    }

    /// Finishes actions whose transition arrived, and times out the rest.
    public mutating func reconcile(_ mirror: RoomMemoryMirror, at now: TimeInterval) {
        for index in actions.indices {
            let pending = actions[index]
            switch pending.phase {
            case .done, .refused, .unsent: continue
            case .sending, .applied, .unknown: break
            }
            // A transition newer than the send, in the generation the action was made for.
            if mirror.activeGeneration == pending.generation, let latest = mirror.eventsById[pending.eventId],
               pending.seqAtSend.map({ latest.seq > $0 }) ?? true {
                let state = latest.event.state
                if state == pending.kind.effect {
                    actions[index].phase = .done(pending.kind.doneWord)
                    continue
                }
                if state != .candidate && state != .confirmed {
                    actions[index].phase = .done("the event ended: \(Self.stateWord(state))")
                    continue
                }
            }
            if let generation = mirror.activeGeneration, generation != pending.generation {
                actions[index].phase = .unknown("the Spark restarted; check the event")
                continue
            }
            switch pending.phase {
            case .sending(let since) where now - since >= Self.responseTimeout:
                actions[index].phase = .unknown("no answer from the Spark")
            case .applied(let since) where now - since >= Self.transitionTimeout:
                actions[index].phase = .unknown("the Spark acted, but its transition has not arrived; check the event")
            default:
                break
            }
        }
    }

    /// The action to send again: the same id and payload, only in the generation it was made for.
    public func retry(actionId: String, mirror: RoomMemoryMirror) -> Pending? {
        guard let pending = actions.first(where: { $0.action.actionId == actionId }),
              case .unknown = pending.phase, mirror.activeGeneration == pending.generation else { return nil }
        return pending
    }

    /// The newest action on `eventId`.
    public func latest(for eventId: UInt64) -> Pending? {
        actions.last(where: { $0.eventId == eventId })
    }

    // MARK: Buttons

    /// The buttons for the selected event: Confirm and Dismiss for a CANDIDATE, Accept as normal
    /// and Dismiss for a CONFIRMED event, none otherwise (a terminal or waiting event, an earlier
    /// run). Each disabled button says why.
    public static func buttons(for entry: LiveRoomEvent?, earlier: Bool, mirror: RoomMemoryMirror, operatorPath: Bool,
                               pending: Pending?) -> [Button] {
        guard let entry, !earlier else { return [] }
        let kinds: [Kind]
        switch entry.event.state {
        case .candidate: kinds = [.confirm, .dismiss]
        case .confirmed: kinds = [.accept, .dismiss]
        case .accepted, .reverted, .expired: kinds = []
        }
        return kinds.map { kind in
            let reason = disabledReason(kind, mirror: mirror, operatorPath: operatorPath, pending: pending)
            return Button(kind: kind, enabled: reason == nil, reason: reason)
        }
    }

    static func disabledReason(_ kind: Kind, mirror: RoomMemoryMirror, operatorPath: Bool, pending: Pending?) -> String? {
        if !operatorPath { return "this Spark can't take actions" }
        if mirror.wire != .v2 || mirror.activeGeneration == nil { return "the Spark's identity is unverified" }
        if !mirror.isStatusFresh { return "status isn't fresh" }
        if pending?.phase.inFlight == true { return "an action on this event is in flight" }
        if kind == .accept, !mirror.isMapCurrent { return "map isn't current" }
        return nil
    }

    /// The selected row's event: an open event, a terminal entry of the active generation, or an
    /// earlier run's open event (`earlier`), by the ids the event rows use.
    public static func entry(for selection: String?, mirror: RoomMemoryMirror) -> (entry: LiveRoomEvent, earlier: Bool)? {
        guard let selection else { return nil }
        if selection.hasPrefix("earlier-") {
            let found = mirror.archivedOpenEvents.first { "earlier-\($0.entry.seq)-\($0.entry.event.eventId)" == selection }
            return found.map { ($0.entry, true) }
        }
        if let open = mirror.openEvents.first(where: { String($0.event.eventId) == selection }) {
            return (open, false)
        }
        if let latest = mirror.eventsById.values.first(where: { "\($0.seq)-\($0.event.eventId)" == selection }) {
            return (latest, false)
        }
        return mirror.events.last { "\($0.seq)-\($0.event.eventId)" == selection }.map { ($0, false) }
    }

    // MARK: Words

    /// One line for the action bar.
    public static func describe(_ pending: Pending) -> String {
        let what = pending.kind.title.lowercased()
        switch pending.phase {
        case .sending: return "\(what): sending…"
        case .applied: return pending.kind == .accept ? "saved · the map is updating" : "\(what): applied · waiting for the Spark"
        case .done(let text): return text
        case .refused(let outcome): return "\(what) refused: \(refusal(outcome))"
        case .unsent(let reason): return "\(what) \(reason)"
        case .unknown(let reason): return "\(what): outcome unknown · \(reason)"
        }
    }

    public static func refusal(_ outcome: RoomMemory.OperatorOutcome) -> String {
        switch outcome {
        case .applied: return "applied"
        case .invalid: return "the Spark could not read it"
        case .wrongRoom: return "it named another room"
        case .staleGeneration: return "the Spark restarted; check the event"
        case .staleEpoch: return "the map changed since you looked; try again"
        case .unknownEvent: return "the Spark no longer tracks this event"
        case .stateMismatch: return "the event changed since you looked"
        case .notTracking: return "the Spark isn't tracking right now"
        case .unsupported: return "this Spark doesn't support it"
        case .storageFailed: return "the Spark could not save it; nothing changed"
        case .targetChanged: return "the Spark doesn't see the change as you saw it just now (gone, hidden or reshaped); look again"
        case .rigUnsettled: return "the rig may have moved; accept is held back"
        }
    }

    /// One line about the selected event: kind, state and size.
    public static func summary(_ entry: LiveRoomEvent) -> String {
        let event = entry.event
        let kind: String
        switch event.kind {
        case .appeared: kind = "Appeared"
        case .disappeared: kind = "Disappeared"
        case .moved: kind = "Moved"
        }
        let cells = event.voxelCount == 1 ? "1 cell" : "\(event.voxelCount) cells"
        return "\(kind) · \(stateWord(event.state)) · \(cells)"
    }

    /// The event rows' words, so the bar and the column never disagree.
    public static func stateWord(_ state: RoomMemory.ChangeEvent.State) -> String {
        stateName(state)
    }
}
