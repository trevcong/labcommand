import Foundation

// MARK: - Values the UI reads

/// The newest `MemoryStatus`, copied out of the wire type. Nil fields were not measured; never render them as 0.
public struct LiveRoomStatus: Sendable, Equatable {
    public var roomId: String
    public var mapEpoch: UInt64
    public var seq: UInt64
    public var stamp: RoomMemory.Time
    public var state: RoomMemory.MemoryState
    public var brickCount: UInt32
    public var openEventCount: UInt32
    public var coverageKnownFraction: Float?
    public var poseResidualM: Float?
    public var windowS: Float?
    public var mid360Hz: UInt16?
    public var jt128Hz: UInt16?
    public var droppedWindows: UInt32
    /// `seq` of the newest live `events` message of this generation. Nil before the first.
    public var eventsSeq: UInt64?
    /// Nil: a legacy node.
    public var publisherGeneration: UInt64?
    /// When the Mac received it, on the mirror's clock.
    public var receivedAt: TimeInterval

    public init(_ status: RoomMemory.MemoryStatus, receivedAt: TimeInterval) {
        roomId = status.header.roomId
        mapEpoch = status.header.mapEpoch
        seq = status.header.seq
        stamp = status.header.stamp
        state = status.state
        brickCount = status.brickCount
        openEventCount = status.openEventCount
        coverageKnownFraction = status.coverageKnownFraction
        poseResidualM = status.poseResidualM
        windowS = status.windowS
        mid360Hz = status.mid360Hz
        jt128Hz = status.jt128Hz
        droppedWindows = status.droppedWindows
        eventsSeq = status.eventsSeq
        publisherGeneration = status.publisherGeneration
        self.receivedAt = receivedAt
    }
}

/// One applied `ChangeEvent` message.
public struct LiveRoomEvent: Sendable, Equatable {
    /// `header.seq` on the events topic.
    public var seq: UInt64
    public var receivedAt: TimeInterval
    /// Arrived on `/room_memory/sync/events` (a replay), not live. Diagnostics only.
    public var fromSync: Bool
    public var event: RoomMemory.ChangeEvent

    public init(seq: UInt64, receivedAt: TimeInterval, fromSync: Bool, event: RoomMemory.ChangeEvent) {
        self.seq = seq
        self.receivedAt = receivedAt
        self.fromSync = fromSync
        self.event = event
    }
}

/// What has been arriving, for diagnostics and the activity line.
public struct LiveRoomActivity: Sendable, Equatable {
    public struct TopicActivity: Sendable, Equatable {
        public var count = 0
        public var lastAt: TimeInterval = 0
    }

    /// Messages received per topic name, applied or not.
    public var topics: [String: TopicActivity] = [:]
    /// Undecodable or contract-breaking messages, dropped whole.
    public var rejected = 0
    public var duplicates = 0
    /// Map messages that did not chain (a seq gap or a wrong base).
    public var gaps = 0
    /// Generations archived after something was observed in them (a publisher restart, or a legacy/v2 switch).
    public var generationChanges = 0
    /// Messages of a generation below the highest installed, ignored.
    public var olderGeneration = 0
    /// A seq received twice with different payloads; the first is kept. Never a restart signal.
    public var contractViolations = 0
    /// A message in the other layout (legacy or v2) than the connection's first one, dropped.
    public var wireMismatch = 0
    /// Messages for another `room_id`, ignored.
    public var foreignRoom = 0
    public var requestsSent = 0
    /// Requests the mirror asked for that no request path carried.
    public var requestsUnsent = 0
    public var lastRequest: RoomMemory.RequestSync?

    mutating func note(_ topic: LiveRoomTopic, at now: TimeInterval) {
        topics[topic.rawValue, default: TopicActivity()].count += 1
        topics[topic.rawValue]?.lastAt = now
    }
}

/// A retired generation, kept for display only (its map greyed until the new one has a map, its
/// open events as "earlier run · may be outdated").
public struct ArchivedGeneration: Sendable, Equatable {
    /// Nil for a legacy (unverified) run.
    public var generation: UInt64?
    public var openEvents: [LiveRoomEvent]
    public var bricks: [RoomMemory.BrickCoordinate: RoomMemory.Brick]
    public var voxelSize: Float?
    public var mapEpoch: UInt64?
}

// MARK: - Mirror

/// The Mac's copy of the Spark's room memory, scoped to one publisher generation
/// (`docs/ROOM-MEMORY-PROTOCOL.md` §5, PLAN-PUBGEN §4).
///
/// Pure state, like `LiveSession`: the client feeds it socket events, message payloads and clock
/// ticks, and performs the actions each step returns. Times are seconds on one monotonic clock.
///
/// Three components replace every heuristic: the active `GenerationState` (the only state that
/// renders as current; a newer generation installs on its first validated message), a
/// `Retrieval` per topic (request tokens, `SyncEnd` manifests, backoff), and the `EventHistory`
/// projection, derived after every input. On a legacy wire (an old node) nothing is verified: the
/// map is never current, coverage is unknown and no request is sent.
public struct RoomMemoryMirror: Sendable, Equatable {
    /// Event messages kept in `events`.
    public static let eventCapacity = 1000
    /// Events kept in `eventsById`; the oldest terminal ones go first.
    public static let eventIdCapacity = 10_000
    /// `status` is fresh for 3 periods (protocol §5).
    public static let statusFreshFor: TimeInterval = 3
    /// Retrieval timeout by attempt: 5, 10, 20, then 30 s (Phase 3 P3).
    public static let requestRetry: [TimeInterval] = [5, 10, 20, 30]
    /// At most one request per topic per this long (protocol §2).
    public static let requestMinInterval: TimeInterval = 5
    /// Retired generations kept for display.
    public static let archiveDepth = 2

    /// Bounds on what the mirror keeps. Eviction only ever lowers certainty (protocol §5).
    public struct Limits: Sendable, Equatable {
        public var eventLog: Int
        public var latest: Int
        public var digest: Int
        /// Ranges in the received coverage; past it the generation's coverage reads unknown.
        public var coverageIntervals: Int

        public init(
            eventLog: Int = RoomMemoryMirror.eventCapacity,
            latest: Int = RoomMemoryMirror.eventIdCapacity,
            digest: Int = 10_000,
            coverageIntervals: Int = 1000
        ) {
            self.eventLog = eventLog
            self.latest = latest
            self.digest = digest
            self.coverageIntervals = coverageIntervals
        }
    }

    public enum Action: Sendable, Equatable {
        /// Send this request. `header.stamp` is `{0, 0}`; the sender stamps it.
        case requestSync(RoomMemory.RequestSync)
    }

    public enum StaleReason: String, Sendable, Equatable {
        case disconnected
        /// Connected, no status of this generation on this connection yet.
        case awaitingStatus
        case seqGap, wrongBase, epochMismatch, rejected
        /// A newer publisher generation has no map yet: the bricks shown are the earlier run's.
        case newGeneration
        /// Legacy wire: the Spark node predates publisher generations.
        case unverified
    }

    public enum MapState: Sendable, Equatable {
        /// No mirror: "Waiting for map".
        case empty
        case current
        /// The last good bricks, greyed: "resyncing · as of epoch N".
        case stale(StaleReason)
    }

    /// The layout of this connection's first room-memory message.
    public enum Wire: Sendable, Equatable {
        case undetermined, v2, legacy
    }

    public enum Connection: Sendable, Equatable {
        case offline
        /// Open; no status of the active generation on it yet.
        case awaitingStatus(connection: Int)
        case observing(connection: Int, generation: UInt64)
    }

    public enum RetrievalStatus: Sendable, Equatable {
        case idle, waiting, receiving
    }

    public let limits: Limits
    public private(set) var roomId: String?
    public private(set) var connection = Connection.offline
    public private(set) var wire = Wire.undetermined
    /// Some request path exists on this connection.
    public private(set) var canRequest = false
    /// The highest generation ever installed. Survives archiving and legacy mode.
    public private(set) var highestGeneration: UInt64?
    public private(set) var archive: [ArchivedGeneration] = []
    public private(set) var mapRevision = 0
    public private(set) var activity = LiveRoomActivity()
    public private(set) var isStatusFresh = false

    var active: GenerationState?
    private var retrieval: [RoomMemory.RequestSync.Topic: Retrieval] = [.mapBricks: Retrieval(), .events: Retrieval()]
    private var tokens: TokenRandom
    private var connectionCounter = 0
    private var requestCounter: UInt64 = 0
    private var clock: TimeInterval = 0
    /// Set once a generation in which something was observed is archived; never cleared.
    private var brokenSince: UInt64??

    /// - Parameters:
    ///   - roomId: the room to mirror; nil adopts the first room a message names.
    ///   - limits: the bounds on kept state.
    ///   - tokenSeed: seeds the request-token generator (tests). Nil seeds it from the system generator.
    public init(roomId: String? = nil, limits: Limits = Limits(), tokenSeed: UInt64? = nil) {
        self.roomId = roomId
        self.limits = limits
        var system = SystemRandomNumberGenerator()
        tokens = TokenRandom(seed: tokenSeed ?? system.next())
    }

    // MARK: Projection

    private var connectionID: Int? {
        switch connection {
        case .offline: return nil
        case .awaitingStatus(let id), .observing(let id, _): return id
        }
    }

    private var connected: Bool { connectionID != nil }

    /// The newest status of the active generation (legacy: of the legacy run).
    public var status: LiveRoomStatus? { active?.status }

    public var activeGeneration: UInt64? { active?.generation }

    /// The bricks shown: the active generation's, else the newest archived run's (greyed).
    public var bricks: [RoomMemory.BrickCoordinate: RoomMemory.Brick] {
        if let chain = active?.chain { return chain.bricks }
        return archive.first(where: { !$0.bricks.isEmpty })?.bricks ?? [:]
    }

    public var voxelSize: Float? {
        if let chain = active?.chain { return chain.voxelSize }
        return archive.first(where: { !$0.bricks.isEmpty })?.voxelSize
    }

    public var mapEpoch: UInt64? {
        if let chain = active?.chain { return chain.epoch }
        return archive.first(where: { !$0.bricks.isEmpty })?.mapEpoch
    }

    /// The seq of the last map message applied in the active generation.
    public var mapSeq: UInt64? { active?.chain?.appliedSeq }

    public var mapState: MapState {
        guard let active, let chain = active.chain else {
            guard archive.contains(where: { !$0.bricks.isEmpty }) else { return .empty }
            let legacy = active != nil && active?.generation == nil
            return .stale(legacy ? .unverified : .newGeneration)
        }
        if active.generation == nil { return .stale(.unverified) }
        guard let id = connectionID else { return .stale(.disconnected) }
        guard active.statusConnection == id else { return .stale(.awaitingStatus) }
        if active.mapRejected { return .stale(.rejected) }
        if let high = active.highestMapSeq, chain.appliedSeq < high { return .stale(active.chainBreak ?? .seqGap) }
        if active.statusEpochMismatches >= 2 { return .stale(.epochMismatch) }
        return .current
    }

    /// The map is CURRENT and the status that vouches for it is fresh. Draw change overlays only then.
    public var isMapCurrent: Bool { mapState == .current && isStatusFresh }

    /// The active generation's event messages, oldest first.
    public var events: [LiveRoomEvent] { active?.log ?? [] }

    /// The highest-seq transition received per event, active generation only.
    public var eventsById: [UInt64: LiveRoomEvent] { active?.latest ?? [:] }

    /// CANDIDATE or CONFIRMED events of the active generation, by `event_id`.
    public var openEvents: [LiveRoomEvent] { active?.openEvents ?? [] }

    /// Open events of archived generations, newest run first: "earlier run · may be outdated".
    public var archivedOpenEvents: [ArchivedEvent] {
        archive.flatMap { run in run.openEvents.map { ArchivedEvent(generation: run.generation, entry: $0) } }
    }

    public var continuity: EventHistory.Continuity {
        if wire == .legacy || active?.generation == nil && active != nil { return .unknown }
        if let since = brokenSince { return .broken(sinceGeneration: since) }
        return active == nil ? .unknown : .verified
    }

    public var eventHistory: EventHistory {
        let freshness: EventHistory.Freshness
        if !connected {
            freshness = .offline
        } else if isStatusFresh {
            freshness = .fresh
        } else if let status = active?.status, active?.statusConnection == connectionID {
            freshness = .stale(age: Swift.max(0, clock - status.receivedAt))
        } else {
            freshness = .stale(age: nil)
        }
        guard let active, active.generation != nil else {
            return EventHistory(generation: nil, coverage: .unknown, freshness: freshness, continuity: continuity,
                                caughtUp: false, contiguousThrough: nil)
        }
        let coverage = active.coverage
        var caughtUp = false
        if case .empty = coverage, isStatusFresh, active.status?.eventsSeq == nil {
            caughtUp = true
        } else if case .completeThrough = coverage, isStatusFresh, let named = active.status?.eventsSeq, let top = active.received.max {
            caughtUp = named >= top
        }
        return EventHistory(
            generation: active.generation,
            coverage: coverage,
            freshness: freshness,
            continuity: continuity,
            caughtUp: caughtUp,
            contiguousThrough: active.received.contiguousFromZeroThrough
        )
    }

    public func retrievalStatus(_ topic: RoomMemory.RequestSync.Topic) -> RetrievalStatus {
        switch retrieval[topic]?.phase ?? .idle {
        case .idle: return .idle
        case .waiting: return .waiting
        case .receiving: return .receiving
        }
    }

    // MARK: Connection

    /// The socket opened and subscribed. `canRequest` says whether any request path exists.
    /// Keeps the active generation and the archive (stale display); every old token dies.
    public mutating func connectionOpened(at now: TimeInterval, canRequest: Bool) -> [Action] {
        clock = now
        connectionCounter += 1
        connection = .awaitingStatus(connection: connectionCounter)
        wire = .undetermined
        self.canRequest = canRequest
        // A map may have been published while away: ask again rather than trust an old "no map yet".
        active?.mapKnownEmptyAtEpoch = nil
        resetRetrievals()
        refreshFreshness()
        return flushRequests()
    }

    /// The socket closed. Nothing proves the map current until a status of the generation arrives
    /// on the next connection. Coverage is untouched.
    public mutating func connectionClosed(at now: TimeInterval) {
        clock = now
        connection = .offline
        resetRetrievals()
        refreshFreshness()
    }

    /// The client could not carry a request the mirror asked for. The retrieval stays waiting;
    /// the next attempt follows the backoff.
    public mutating func noteRequestUnsent() {
        activity.requestsSent -= 1
        activity.requestsUnsent += 1
    }

    // MARK: Messages

    private enum Message {
        case map(RoomMemory.MapBricks)
        case event(RoomMemory.ChangeEvent)
        case status(RoomMemory.MemoryStatus)
        case syncEnd(RoomMemory.SyncEnd)

        var roomId: String {
            switch self {
            case .map(let m): return m.header.roomId
            case .event(let m): return m.header.roomId
            case .status(let m): return m.header.roomId
            case .syncEnd(let m): return m.header.roomId
            }
        }

        var generation: UInt64? {
            switch self {
            case .map(let m): return m.publisherGeneration
            case .event(let m): return m.publisherGeneration
            case .status(let m): return m.publisherGeneration
            case .syncEnd(let m): return m.publisherGeneration
            }
        }
    }

    private static func decode(_ topic: LiveRoomTopic, _ payload: Data) throws -> Message {
        switch topic {
        case .mapBricks, .syncMapBricks: return .map(try RoomMemory.MapBricks.decode(cdr: payload))
        case .events, .syncEvents: return .event(try RoomMemory.ChangeEvent.decode(cdr: payload))
        case .status: return .status(try RoomMemory.MemoryStatus.decode(cdr: payload))
        case .syncEnd: return .syncEnd(try RoomMemory.SyncEnd.decode(cdr: payload))
        }
    }

    /// One message's CDR payload (the Foxglove frame without its 13-byte header).
    public mutating func receive(topic: LiveRoomTopic, payload: Data, at now: TimeInterval) -> [Action] {
        clock = now
        activity.note(topic, at: now)
        guard let id = connectionID else { return [] }
        defer { refreshFreshness() }
        let message: Message
        do {
            message = try Self.decode(topic, payload)
        } catch {
            activity.rejected += 1
            if topic == .mapBricks || topic == .syncMapBricks {
                // An unreadable map update may be the next delta: nothing proves the bricks current now.
                active?.mapRejected = true
            }
            return settleAndFlush()
        }
        guard adopt(room: message.roomId) else { return [] }

        // The connection's first message fixes its layout; SyncEnd exists only in v2.
        let layout: Wire = message.generation == nil ? .legacy : .v2
        switch wire {
        case .undetermined:
            wire = layout
            if layout == .legacy { enterLegacy() }
        case let fixed where fixed != layout:
            activity.wireMismatch += 1
            if topic == .mapBricks || topic == .syncMapBricks {
                // A v2 map message cut inside its trailing fields reads as legacy: it may be the next delta.
                active?.mapRejected = true
            }
            return settleAndFlush()
        default:
            break
        }

        // Matched before any install resets the retrievals: the manifest applies to its own generation.
        var matched: Retrieval?
        if case .syncEnd(let end) = message, let current = retrieval[end.topic], current.matches(end.token) {
            matched = current
        }
        if let generation = message.generation {
            guard enterGeneration(generation) else { return settleAndFlush() }
        }
        switch message {
        case .map(let map): receiveMap(map, connection: id)
        case .event(let event): receiveEvent(event, at: now, fromSync: topic.isSync)
        case .status(let status): receiveStatus(status, at: now, connection: id)
        case .syncEnd(let end): receiveSyncEnd(end, matched: matched)
        }
        // Any validated message of the generation counts: archiving it now breaks continuity.
        active?.observed = true
        return settleAndFlush()
    }

    /// Clock tick: status freshness, retrieval timeouts and request scheduling.
    public mutating func tick(at now: TimeInterval) -> [Action] {
        clock = now
        refreshFreshness()
        return settleAndFlush()
    }

    private mutating func adopt(room: String) -> Bool {
        guard let roomId else {
            roomId = room
            return true
        }
        if room != roomId {
            activity.foreignRoom += 1
            return false
        }
        return true
    }

    private mutating func refreshFreshness() {
        guard let id = connectionID, let active, let status = active.status, active.statusConnection == id else {
            isStatusFresh = false
            return
        }
        isStatusFresh = clock - status.receivedAt < Self.statusFreshFor
    }

    // MARK: Generations

    /// Puts a v2 message's generation in place. False: the message is older than the highest
    /// generation and changes nothing but a counter.
    private mutating func enterGeneration(_ generation: UInt64) -> Bool {
        if let highest = highestGeneration, generation < highest {
            activity.olderGeneration += 1
            return false
        }
        if highestGeneration.map({ generation > $0 }) ?? true {
            // A newer generation: generations are durable and wall-clock floored, so it is a later process.
            install(generation)
        } else if active?.generation != generation {
            // The highest generation again after a legacy connection archived it: start empty,
            // never reactivate the archive.
            archiveActive()
            active = GenerationState(generation: generation)
        }
        return true
    }

    private mutating func install(_ generation: UInt64) {
        archiveActive()
        active = GenerationState(generation: generation)
        highestGeneration = generation
        resetRetrievals()
        if case .observing(let id, _) = connection {
            connection = .awaitingStatus(connection: id)
        }
    }

    /// A legacy layout on this connection: the active v2 generation retires; one legacy state
    /// (identity unverified) holds what this and later legacy connections show.
    private mutating func enterLegacy() {
        if active?.generation != nil {
            archiveActive()
        }
        if active == nil {
            active = GenerationState(generation: nil)
        }
        resetRetrievals()
    }

    private mutating func archiveActive() {
        guard let old = active else { return }
        active = nil
        if old.observed {
            brokenSince = .some(old.generation)
            activity.generationChanges += 1
        }
        guard old.observed || old.chain != nil else { return }
        archive.insert(ArchivedGeneration(
            generation: old.generation,
            openEvents: old.openEvents,
            bricks: old.chain?.bricks ?? [:],
            voxelSize: old.chain?.voxelSize,
            mapEpoch: old.chain?.epoch
        ), at: 0)
        if archive.count > Self.archiveDepth {
            archive.removeLast(archive.count - Self.archiveDepth)
        }
    }

    // MARK: Map

    private mutating func receiveMap(_ message: RoomMemory.MapBricks, connection id: Int) {
        guard var state = active else { return }
        if state.generation != nil {
            state.noteMapSeq(message.header.seq)
        }
        let before = state.chain?.bricks
        switch state.apply(message, connection: id) {
        case .applied:
            if state.chain?.bricks != before { mapRevision += 1 }
        case .duplicate:
            activity.duplicates += 1
        case .gap, .wrongBase:
            activity.gaps += 1
        case .applyFailed:
            activity.rejected += 1
        case .older, .noChain:
            break
        }
        active = state
    }

    /// The delta applied to `bricks`, or nil when it cannot apply as a whole (protocol §3.1).
    static func applying(
        _ delta: RoomMemory.MapBricks,
        to bricks: [RoomMemory.BrickCoordinate: RoomMemory.Brick],
        voxelSize: Float?
    ) -> [RoomMemory.BrickCoordinate: RoomMemory.Brick]? {
        if let voxelSize, voxelSize != delta.voxelSize { return nil }
        var next = bricks
        if !delta.removedBrickIndices.isEmpty {
            // Indices into the base epoch's brick list in canonical order.
            let order = bricks.keys.sorted()
            var removed = Set<RoomMemory.BrickCoordinate>()
            for index in delta.removedBrickIndices {
                guard Int(index) < order.count else { return nil }
                removed.insert(order[Int(index)])
            }
            for brick in delta.bricks where removed.contains(brick.coordinate) {
                return nil
            }
            for coordinate in removed {
                next[coordinate] = nil
            }
        }
        for brick in delta.bricks {
            next[brick.coordinate] = brick
        }
        return next
    }

    // MARK: Events

    private mutating func receiveEvent(_ message: RoomMemory.ChangeEvent, at now: TimeInterval, fromSync: Bool) {
        guard active != nil else { return }
        switch active!.record(message, at: now, fromSync: fromSync, limits: limits) {
        case .recorded: break
        case .duplicate: activity.duplicates += 1
        case .conflict: activity.contractViolations += 1
        }
    }

    // MARK: Status

    private mutating func receiveStatus(_ message: RoomMemory.MemoryStatus, at now: TimeInterval, connection id: Int) {
        guard var state = active else { return }
        if let held = state.status {
            // v2: a seq not above the held one is a duplicate or a delayed copy, never a generation
            // signal. Legacy: only an exact copy is known to be one.
            let duplicate = state.generation != nil
                ? message.header.seq <= held.seq
                : message.header.seq == held.seq && message.header.stamp == held.stamp
            if duplicate {
                activity.duplicates += 1
                return
            }
        }
        state.status = LiveRoomStatus(message, receivedAt: now)
        state.statusConnection = id
        state.observed = true
        if let generation = state.generation {
            state.witness = state.witness.merged(message.eventsSeq.map(EventsWitness.through) ?? .empty)
            state.noteMapAvailability(message)
            connection = .observing(connection: id, generation: generation)
        }
        // D20: a lost trailing delta shows as the status epoch disagreeing twice in a row.
        if let chain = state.chain {
            if message.header.mapEpoch != chain.epoch {
                state.statusEpochMismatches += 1
            } else {
                state.statusEpochMismatches = 0
                state.mismatchRetrieved = false
            }
        }
        active = state
    }

    // MARK: Retrieval

    /// Publisher facts from any `SyncEnd` of the generation, whoever asked; the manifest too when
    /// it answers this Mac's outstanding request.
    private mutating func receiveSyncEnd(_ end: RoomMemory.SyncEnd, matched: Retrieval?) {
        guard var state = active, state.generation == end.publisherGeneration else { return }
        switch end.topic {
        case .events:
            state.witness = state.witness.merged(end.bound.map(EventsWitness.through) ?? .empty)
            for range in end.unavailable {
                state.unavailable.formUnion(range)
            }
        case .mapBricks:
            if let bound = end.bound {
                state.noteMapSeq(bound)
            } else if state.highestMapSeq == nil {
                state.mapKnownEmptyAtEpoch = end.header.mapEpoch
            }
        }
        state.observed = true
        active = state
        if var current = matched {
            current.phase = .receiving(end)
            current.generation = end.publisherGeneration
            retrieval[end.topic] = current
        }
    }

    private mutating func resetRetrievals() {
        for topic in RoomMemory.RequestSync.Topic.allCases {
            retrieval[topic]?.reset()
        }
    }

    /// Completes the retrievals whose manifests are now held.
    private mutating func settleRetrievals() {
        for topic in RoomMemory.RequestSync.Topic.allCases {
            guard var current = retrieval[topic], case .receiving(let end) = current.phase else { continue }
            guard var state = active, state.generation == end.publisherGeneration else {
                current.reset()
                retrieval[topic] = current
                continue
            }
            let done: Bool
            switch topic {
            case .events:
                done = end.delivered.allSatisfy { state.received.contains($0) }
            case .mapBricks:
                if let last = end.delivered.last {
                    done = state.chain.map { $0.appliedSeq >= last.upperBound } ?? false
                } else if let bound = end.bound {
                    done = state.chain.map { $0.appliedSeq >= bound } ?? false
                } else {
                    done = true
                }
                if done {
                    state.mapRejected = false
                    if state.statusEpochMismatches >= 2 { state.mismatchRetrieved = true }
                    active = state
                }
            }
            if done {
                current.reset()
                retrieval[topic] = current
            }
        }
    }

    private func needs(_ topic: RoomMemory.RequestSync.Topic) -> Bool {
        guard let active, active.generation != nil else { return false }
        switch topic {
        case .mapBricks: return active.mapNeedsRetrieval
        case .events: return !active.overflowed && !active.pending.isEmpty
        }
    }

    private mutating func settleAndFlush() -> [Action] {
        settleRetrievals()
        return flushRequests()
    }

    /// Requests go out only while observing a v2 generation, with a request path.
    private mutating func flushRequests() -> [Action] {
        let now = clock
        var actions: [Action] = []
        let generation: UInt64?
        if case .observing(_, let observed) = connection, wire == .v2, canRequest, roomId != nil,
           active?.generation == observed {
            generation = observed
        } else {
            generation = nil
        }
        for topic in RoomMemory.RequestSync.Topic.allCases {
            guard var current = retrieval[topic] else { continue }
            if current.isOutstanding {
                guard current.timedOut(at: now) else { continue }
                guard let generation, needs(topic) else {
                    current.reset()
                    retrieval[topic] = current
                    continue
                }
                actions.append(.requestSync(send(topic, &current, generation: generation, attempt: current.attempt + 1, at: now)))
            } else {
                guard let generation, needs(topic), current.maySend(at: now) else { continue }
                actions.append(.requestSync(send(topic, &current, generation: generation, attempt: 1, at: now)))
            }
            retrieval[topic] = current
        }
        return actions
    }

    private mutating func send(
        _ topic: RoomMemory.RequestSync.Topic,
        _ current: inout Retrieval,
        generation: UInt64,
        attempt: Int,
        at now: TimeInterval
    ) -> RoomMemory.RequestSync {
        let since: RoomMemory.RequestSync.Position?
        switch topic {
        case .mapBricks:
            // A retry asks for a snapshot: the replay the first answer named did not complete.
            since = attempt > 1 ? nil : active?.chain.map { .init(epoch: $0.epoch, seq: $0.appliedSeq) }
        case .events:
            since = active?.eventsSince
        }
        let token = RoomMemory.RequestToken(using: &tokens)
        current.phase = .waiting
        current.token = token
        current.generation = generation
        current.sentAt = now
        current.lastSentAt = now
        current.attempt = attempt
        let request = RoomMemory.RequestSync(
            header: RoomMemory.Header(
                roomId: roomId ?? "",
                mapEpoch: since?.epoch ?? RoomMemory.RequestSync.sinceNone,
                seq: requestCounter,
                stamp: RoomMemory.Time(sec: 0, nanosec: 0)
            ),
            since: since,
            topic: topic,
            expectedGeneration: generation,
            token: token
        )
        requestCounter += 1
        activity.requestsSent += 1
        activity.lastRequest = request
        return request
    }
}
