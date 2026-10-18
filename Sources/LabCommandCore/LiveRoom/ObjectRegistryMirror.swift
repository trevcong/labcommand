import Foundation

/// The named-object snapshot for one room (protocol §3.15).
///
/// The client feeds it `ObjectRegistry` snapshots, a settle signal for each publisher generation,
/// and the logged state of each change event. A snapshot stays held until its publisher generation
/// settles. A newer unsettled generation does not replace the one on screen. Other rooms, older
/// publisher generations, and older `snapshot_seq` values are dropped.
///
/// A receipt in the snapshot on screen covers that `(event_generation, event_id)` whether the
/// generation is the current one or an older one. A logged ACCEPTED beats the receipt. The receipt
/// beats EXPIRED and every other logged state. The row reads `accepted (recovered)` either way, named or not.
public struct ObjectRegistryMirror: Sendable, Equatable {
    /// The row text for an accept that was answered APPLIED and has no logged ACCEPTED.
    public static let recoveredLabel = "accepted (recovered)"

    /// How one accept renders against the snapshot on screen.
    public enum Accept: Sendable, Equatable {
        /// A logged ACCEPTED. It beats a receipt for the same event.
        case accepted
        /// A receipt, and the logged state is not ACCEPTED.
        case recovered
        /// The logged state. No receipt in the snapshot on screen.
        case logged(RoomMemory.ChangeEvent.State)
        /// Nothing logged, and the snapshot on screen has no receipt for this event.
        case none

        /// The text a row shows for this mark.
        public var label: String {
            switch self {
            case .accepted: return "ACCEPTED"
            case .recovered: return ObjectRegistryMirror.recoveredLabel
            case .logged(.candidate): return "CANDIDATE"
            case .logged(.confirmed): return "CONFIRMED"
            case .logged(.accepted): return "ACCEPTED"
            case .logged(.reverted): return "REVERTED"
            case .logged(.expired): return "EXPIRED"
            case .none: return ""
            }
        }
    }

    /// One `NameObject` attempt, keyed by `action_id`.
    public enum NameAttempt: Sendable, Equatable {
        /// APPLIED, or the id is in `recent_action_ids` and this attempt was not told to reconcile.
        case completed
        /// No response yet, and the id is not in the snapshot on screen.
        case pending
        /// STALE_REVISION or UNKNOWN_EVENT. Stays here after the id appears in `recent_action_ids`.
        case reconcile(RoomMemory.NameOutcome)
        /// Any other outcome. The attempt did not land.
        case rejected(RoomMemory.NameOutcome)
    }

    private struct EventKey: Hashable, Sendable {
        var generation: UInt64
        var id: UInt64
    }

    /// The room. Nil until `init` or the first kept snapshot names one.
    public private(set) var roomId: String?
    /// Best snapshot of each generation still newer than or equal to the one on screen.
    private var held: [UInt64: RoomMemory.ObjectRegistry] = [:]
    /// Publisher generations the caller has settled.
    private var settled: Set<UInt64> = []
    /// The publisher generation on screen. Nil until one of its snapshots has settled.
    private var shownGeneration: UInt64?
    /// Latest logged state of each change event.
    private var logged: [EventKey: RoomMemory.ChangeEvent.State] = [:]
    /// Latest `NameObject` outcome for each action id.
    private var responses: [String: RoomMemory.NameOutcome] = [:]

    /// - Parameter roomId: the room to mirror. Nil adopts the first snapshot's room.
    public init(roomId: String? = nil) {
        self.roomId = roomId
    }

    /// The snapshot on screen. Nil while every held generation is still unsettled.
    public var snapshot: RoomMemory.ObjectRegistry? {
        guard let shownGeneration else { return nil }
        return held[shownGeneration]
    }

    /// Keep `registry` if it is this room and newer than what is already held for its generation.
    ///
    /// A settled generation updates the screen when it is current or newer. An unsettled newer
    /// generation is held until `generationSettled` names it.
    /// - Returns: True when the snapshot on screen changed.
    @discardableResult
    public mutating func ingest(_ registry: RoomMemory.ObjectRegistry) -> Bool {
        if let roomId {
            guard registry.header.roomId == roomId else { return false }
        } else {
            roomId = registry.header.roomId
        }
        if let shownGeneration, registry.publisherGeneration < shownGeneration { return false }
        if let existing = held[registry.publisherGeneration], registry.snapshotSeq <= existing.snapshotSeq {
            return false
        }
        let before = snapshot
        held[registry.publisherGeneration] = registry
        refreshShown()
        return snapshot != before
    }

    /// The publisher generation has settled. A held snapshot of that generation can take the screen.
    public mutating func generationSettled(_ generation: UInt64) {
        settled.insert(generation)
        refreshShown()
    }

    /// Record the logged state of one change event. The latest note is the logged state.
    public mutating func note(eventGeneration: UInt64, eventId: UInt64, state: RoomMemory.ChangeEvent.State) {
        logged[EventKey(generation: eventGeneration, id: eventId)] = state
    }

    /// Record the outcome of one `NameObject` attempt. A later outcome for the same action id replaces it.
    public mutating func noteResponse(actionId: String, outcome: RoomMemory.NameOutcome) {
        responses[actionId] = outcome
    }

    /// How the snapshot on screen and the logged state render this event.
    public func accept(eventGeneration: UInt64, eventId: UInt64) -> Accept {
        let key = EventKey(generation: eventGeneration, id: eventId)
        if logged[key] == .accepted { return .accepted }
        if snapshot?.recoveredAccepts.contains(where: { $0.eventGeneration == eventGeneration && $0.eventId == eventId }) == true {
            return .recovered
        }
        if let state = logged[key] { return .logged(state) }
        return .none
    }

    /// The name of the object whose links include this event, in the snapshot on screen.
    public func objectName(eventGeneration: UInt64, eventId: UInt64) -> String? {
        guard let snapshot else { return nil }
        for object in snapshot.objects {
            if object.links.contains(where: { $0.eventGeneration == eventGeneration && $0.eventId == eventId }) {
                return object.name
            }
        }
        return nil
    }

    /// Whether this action id has completed, should reconcile, or is still open.
    public func attempt(actionId: String) -> NameAttempt {
        if let outcome = responses[actionId] {
            switch outcome {
            case .applied:
                return .completed
            case .staleRevision, .unknownEvent:
                return .reconcile(outcome)
            case .invalid, .wrongRoom, .stateMismatch, .storageFailed, .unknownObject:
                return .rejected(outcome)
            }
        }
        if snapshot?.recentActionIds.contains(actionId) == true { return .completed }
        return .pending
    }

    /// Move the screen forward to the newest settled generation that is held.
    private mutating func refreshShown() {
        guard let best = held.keys.filter({ settled.contains($0) }).max() else { return }
        if let shownGeneration, best < shownGeneration { return }
        guard shownGeneration != best else { return }
        shownGeneration = best
        held = held.filter { $0.key >= best }
    }
}
