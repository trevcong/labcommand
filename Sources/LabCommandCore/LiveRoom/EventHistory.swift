import Foundation

/// What the mirror can say about the active publisher generation's event history
/// (`docs/ROOM-MEMORY-PROTOCOL.md` §5). Derived from the mirror after every input, never stored.
public struct EventHistory: Sendable, Equatable {
    public enum Coverage: Sendable, Equatable {
        /// No publisher witness and nothing received; or the coverage outgrew its bounds; or the
        /// Spark node predates publisher generations (legacy wire).
        case unknown
        /// A v2 witness (a status with `EVENTS_SEQ_NONE`, or a `SyncEnd` with bound `SEQ_NONE`) says
        /// this generation published no event, and none was received.
        case empty
        /// Every seq `0...n` of this generation was received.
        case completeThrough(UInt64)
        /// Seqs of `[0, top]` not received. `pending` may still arrive; `lost` the publisher declared
        /// unavailable (a request or a timeout never moves a seq there).
        case missing(pending: [ClosedRange<UInt64>], lost: [ClosedRange<UInt64>])
    }

    public enum Freshness: Sendable, Equatable {
        /// A status of this generation arrived on this connection within `RoomMemoryMirror.statusFreshFor`.
        case fresh
        /// Connected, the newest status on this connection is `age` seconds old (nil: none yet).
        case stale(age: TimeInterval?)
        case offline
    }

    public enum Continuity: Sendable, Equatable {
        /// Legacy wire, or nothing observed yet.
        case unknown
        /// The mirror archived a generation in which it had observed something: events of that run
        /// may be missing. `sinceGeneration` is nil when the archived run was a legacy (unverified) one.
        /// A retired generation has no final bound on the wire, so this never heals.
        case broken(sinceGeneration: UInt64?)
        /// Everything observed since launch belongs to the active generation.
        case verified
    }

    /// The active publisher generation; nil in legacy mode or before the first message.
    public var generation: UInt64?
    public var coverage: Coverage
    public var freshness: Freshness
    public var continuity: Continuity
    /// Coverage is `empty` or `completeThrough(n)`, and a fresh status of this generation on this
    /// connection names an `events_seq` that covers the newest seq received. An as-of observation,
    /// not a claim about the publisher's present.
    public var caughtUp: Bool
    /// The top of the run of received seqs that starts at 0.
    public var contiguousThrough: UInt64?

    public init(
        generation: UInt64?,
        coverage: Coverage,
        freshness: Freshness,
        continuity: Continuity,
        caughtUp: Bool,
        contiguousThrough: UInt64?
    ) {
        self.generation = generation
        self.coverage = coverage
        self.freshness = freshness
        self.continuity = continuity
        self.caughtUp = caughtUp
        self.contiguousThrough = contiguousThrough
    }

    /// Seqs missing (pending and lost), and of those the lost ones.
    public var missingCounts: (missing: UInt64, lost: UInt64) {
        guard case .missing(let pending, let lost) = coverage else { return (0, 0) }
        let count = { (ranges: [ClosedRange<UInt64>]) in ranges.reduce(UInt64(0)) { $0 &+ ($1.upperBound - $1.lowerBound &+ 1) } }
        return (count(pending) &+ count(lost), count(lost))
    }

    /// Coverage is `empty` or `completeThrough`.
    public var isComplete: Bool {
        switch coverage {
        case .empty, .completeThrough: return true
        case .unknown, .missing: return false
        }
    }
}

/// An open event of an archived generation: "earlier run · may be outdated", never drawn as current.
public struct ArchivedEvent: Sendable, Equatable {
    /// Nil for a legacy (unverified) run.
    public var generation: UInt64?
    public var entry: LiveRoomEvent

    public init(generation: UInt64?, entry: LiveRoomEvent) {
        self.generation = generation
        self.entry = entry
    }
}
