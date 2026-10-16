import Foundation

/// The newest seq a publisher generation is witnessed to have reached on the events topic.
enum EventsWitness: Sendable, Equatable {
    /// No status or `SyncEnd` of this generation yet.
    case unknown
    /// A v2 witness says nothing was published.
    case empty
    case through(UInt64)

    var top: UInt64? {
        if case .through(let seq) = self { return seq }
        return nil
    }

    /// Witnesses only ever raise the bound: an older "empty" never lowers a seen seq.
    func merged(_ other: EventsWitness) -> EventsWitness {
        switch (self, other) {
        case (.unknown, _): return other
        case (_, .unknown): return self
        case (.through(let a), .through(let b)): return .through(Swift.max(a, b))
        case (.through, .empty): return self
        case (.empty, _): return other
        }
    }
}

/// Everything the mirror knows about one publisher generation (protocol §5), or about the one
/// legacy connection state (`generation` nil, identity unverified). Pure. Every entry came from a
/// message that carried `generation`: nothing crosses generations.
struct GenerationState: Sendable, Equatable {
    struct Chain: Sendable, Equatable {
        var epoch: UInt64
        var appliedSeq: UInt64
        var bricks: [RoomMemory.BrickCoordinate: RoomMemory.Brick]
        var voxelSize: Float
        /// Legacy only: the connection whose snapshot started the chain. A legacy delta chains
        /// only on that connection, since a restart while away is undetectable there.
        var connection: Int
    }

    /// What one seq carried, to tell a late copy from a conflicting payload.
    struct Transition: Sendable, Equatable {
        var eventId: UInt64
        var state: RoomMemory.ChangeEvent.State
        var mapEpoch: UInt64
    }

    enum EventOutcome: Equatable {
        case recorded, duplicate, conflict
    }

    enum MapOutcome: Equatable {
        case applied, duplicate, older, gap, wrongBase, noChain, applyFailed
    }

    let generation: UInt64?

    // Status
    var status: LiveRoomStatus?
    /// The connection the status arrived on.
    var statusConnection: Int?

    // Map
    var chain: Chain?
    /// The newest map seq this generation is known to have published: every map message of it
    /// received, live or sync, and every `SyncEnd` bound.
    var highestMapSeq: UInt64?
    /// Why the chain last failed to extend; shown while `appliedSeq < highestMapSeq`.
    var chainBreak: RoomMemoryMirror.StaleReason?
    /// An undecodable map message arrived: its seq is unknown, so only a snapshot or a completed
    /// map retrieval proves the chain again.
    var mapRejected = false
    /// Consecutive statuses whose `map_epoch` disagreed with the chain (D20).
    var statusEpochMismatches = 0
    /// A map retrieval completed during the current mismatch run: a request cannot fix it.
    var mismatchRetrieved = false
    /// A `SyncEnd` said the generation had published no map yet, at this `map_epoch`. Only
    /// suppresses requests while nothing newer suggests a map: a status holding a reference or on
    /// another epoch, a map seq, or a reconnect clears it.
    var mapKnownEmptyAtEpoch: UInt64?

    // Events
    /// C: the seqs actually received, live or sync, from any Mac's answer.
    var received = IntervalSet()
    /// H: the highest witnessed seq (statuses, `SyncEnd` bounds).
    var witness = EventsWitness.unknown
    /// U: seqs the publisher declared unavailable (any `SyncEnd` of this generation).
    var unavailable = IntervalSet()
    /// The highest-seq transition received per event.
    var latest: [UInt64: LiveRoomEvent] = [:]
    /// The highest seq of an entry evicted from `latest`. An event not in `latest` is not
    /// re-entered at or below it: its evicted transition may have been newer (§4.4 invariant 11).
    var latestEvictedThrough: UInt64?
    /// What each received seq carried; bounded, lowest seqs dropped first.
    var digest: [UInt64: Transition] = [:]
    /// Display ring, oldest first.
    var log: [LiveRoomEvent] = []
    /// C outgrew its interval cap: coverage is unknown from then on and never becomes more complete.
    var overflowed = false
    /// Anything at all was recorded in this state.
    var observed = false

    init(generation: UInt64?) {
        self.generation = generation
    }

    // MARK: Events

    /// The coverage of `[0, top]`, top = max(H, max C).
    var coverage: EventHistory.Coverage {
        if overflowed || generation == nil { return .unknown }
        guard let top = [witness.top, received.max].compactMap({ $0 }).max() else {
            return witness == .empty ? .empty : .unknown
        }
        let missing = received.gaps(in: 0...top)
        if missing.isEmpty { return .completeThrough(top) }
        return .missing(pending: missing.subtracting(unavailable).ranges, lost: missing.intersection(unavailable).ranges)
    }

    /// Seqs a request may still fill.
    var pending: [ClosedRange<UInt64>] {
        if case .missing(let pending, _) = coverage { return pending }
        return []
    }

    mutating func record(
        _ message: RoomMemory.ChangeEvent,
        at now: TimeInterval,
        fromSync: Bool,
        limits: RoomMemoryMirror.Limits
    ) -> EventOutcome {
        let seq = message.header.seq
        if let held = digest[seq] {
            return held.eventId == message.eventId && held.state == message.state ? .duplicate : .conflict
        }
        // Received once and since dropped from the digest: nothing to compare against.
        if received.contains(seq) { return .duplicate }
        if !overflowed {
            if received.rangeCount(inserting: seq) > limits.coverageIntervals {
                overflowed = true
            } else {
                received.insert(seq)
            }
        }
        digest[seq] = Transition(eventId: message.eventId, state: message.state, mapEpoch: message.header.mapEpoch)
        if digest.count > limits.digest, let lowest = digest.keys.min() {
            digest[lowest] = nil
        }
        let entry = LiveRoomEvent(seq: seq, receivedAt: now, fromSync: fromSync, event: message)
        log.append(entry)
        if log.count > limits.eventLog {
            log.removeFirst(log.count - limits.eventLog)
        }
        let newer = latest[message.eventId].map { $0.seq < seq } ?? latestEvictedThrough.map { $0 < seq } ?? true
        if newer {
            latest[message.eventId] = entry
            if latest.count > limits.latest {
                let terminal = latest.values.filter { !$0.isOpen }
                if let oldest = (terminal.isEmpty ? Array(latest.values) : terminal).min(by: { $0.seq < $1.seq }) {
                    latest[oldest.event.eventId] = nil
                    latestEvictedThrough = Swift.max(latestEvictedThrough ?? oldest.seq, oldest.seq)
                }
            }
        }
        observed = true
        return .recorded
    }

    /// `since` for an events request: the top of the run from 0, if its epoch is still known.
    var eventsSince: RoomMemory.RequestSync.Position? {
        guard let top = received.contiguousFromZeroThrough, let held = digest[top] else { return nil }
        return RoomMemory.RequestSync.Position(epoch: held.mapEpoch, seq: top)
    }

    var openEvents: [LiveRoomEvent] {
        latest.values.filter(\.isOpen).sorted { $0.event.eventId < $1.event.eventId }
    }

    // MARK: Map

    mutating func noteMapSeq(_ seq: UInt64) {
        highestMapSeq = Swift.max(highestMapSeq ?? seq, seq)
        mapKnownEmptyAtEpoch = nil
    }

    /// A status of this generation: holding a reference (TRACKING, RIG_MOVED) or on another
    /// epoch, a map may exist now. DEGRADED and PAUSED are entered from BUILDING too, and leaving
    /// BUILDING bumps the epoch, so they count only through the epoch.
    mutating func noteMapAvailability(_ status: RoomMemory.MemoryStatus) {
        guard let epoch = mapKnownEmptyAtEpoch else { return }
        if status.state == .tracking || status.state == .rigMoved || status.header.mapEpoch != epoch {
            mapKnownEmptyAtEpoch = nil
        }
    }

    /// Applies a snapshot or delta of this generation to the chain.
    mutating func apply(_ message: RoomMemory.MapBricks, connection: Int) -> MapOutcome {
        let seq = message.header.seq
        switch message.kind {
        case .snapshot:
            if let chain, seq < chain.appliedSeq, generation != nil {
                // An older answer, possibly another Mac's.
                return .older
            }
            chain = Chain(
                epoch: message.header.mapEpoch,
                appliedSeq: seq,
                bricks: Dictionary(uniqueKeysWithValues: message.bricks.map { ($0.coordinate, $0) }),
                voxelSize: message.voxelSize,
                connection: connection
            )
            chainBreak = nil
            mapRejected = false
            statusEpochMismatches = 0
            mismatchRetrieved = false
            observed = true
            return .applied
        case .delta:
            guard var current = chain else {
                chainBreak = .seqGap
                return .noChain
            }
            if seq <= current.appliedSeq { return .duplicate }
            if generation == nil, current.connection != connection {
                // Legacy: across a reconnect only a snapshot proves the chain.
                return .noChain
            }
            guard seq == current.appliedSeq &+ 1 else {
                chainBreak = .seqGap
                return .gap
            }
            guard message.baseEpoch == current.epoch else {
                chainBreak = .wrongBase
                return .wrongBase
            }
            guard let next = RoomMemoryMirror.applying(message, to: current.bricks, voxelSize: current.voxelSize) else {
                mapRejected = true
                return .applyFailed
            }
            current.bricks = next
            current.epoch = message.header.mapEpoch
            current.appliedSeq = seq
            chain = current
            statusEpochMismatches = 0
            mismatchRetrieved = false
            observed = true
            return .applied
        }
    }

    /// The map needs a request a `RequestSync` can answer.
    var mapNeedsRetrieval: Bool {
        guard generation != nil else { return false }
        if mapRejected { return true }
        guard let chain else {
            // Nothing held: ask, unless the publisher said it has no map yet and nothing came since.
            return !(mapKnownEmptyAtEpoch != nil && highestMapSeq == nil)
        }
        if let high = highestMapSeq, chain.appliedSeq < high { return true }
        return statusEpochMismatches >= 2 && !mismatchRetrieved
    }
}

extension LiveRoomEvent {
    /// CANDIDATE or CONFIRMED.
    public var isOpen: Bool { event.state == .candidate || event.state == .confirmed }
}
