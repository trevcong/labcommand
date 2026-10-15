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
    /// The events high-water mark: `seq` of the newest live `events` message. Nil before the first.
    public var eventsSeq: UInt64?
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
        self.receivedAt = receivedAt
    }
}

/// One applied `ChangeEvent` message.
public struct LiveRoomEvent: Sendable, Equatable {
    /// `header.seq` on the events topic.
    public var seq: UInt64
    public var receivedAt: TimeInterval
    /// Arrived on `/room_memory/sync/events` (a replay), not live.
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
    public var gaps = 0
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

// MARK: - Mirror

/// The Mac's copy of the Spark's room memory: map bricks, the event list and the newest status,
/// kept by the resync rules of `docs/ROOM-MEMORY-PROTOCOL.md` §5.
///
/// Pure state, like `LiveSession`: the client feeds it socket events, message payloads and clock
/// ticks, and performs the actions each step returns. Times are seconds on one monotonic clock.
public struct RoomMemoryMirror: Sendable, Equatable {
    /// Event messages kept in `events`.
    public static let eventCapacity = 1000
    /// Events kept in `eventsById`; the oldest terminal ones go first.
    public static let eventIdCapacity = 10_000
    /// `status` is fresh for 3 periods (protocol §5).
    public static let statusFreshFor: TimeInterval = 3
    /// Wait before repeating an unresolved request, by attempt: 5, 10, 20, then 30 s (Phase 3 P3).
    public static let requestRetry: [TimeInterval] = [5, 10, 20, 30]
    /// At most one request per topic per this long (protocol §2).
    public static let requestMinInterval: TimeInterval = 5
    /// An events answer is complete once its sync topic has been quiet this long.
    public static let syncSettle: TimeInterval = 2

    public enum Action: Sendable, Equatable {
        /// Send this request. `header.stamp` is `{0, 0}`; the sender stamps it.
        case requestSync(RoomMemory.RequestSync)
    }

    public enum StaleReason: String, Sendable, Equatable {
        case disconnected, deltaFirst, seqGap, wrongBase, epochMismatch, rejected
    }

    public enum MapState: Sendable, Equatable {
        /// No mirror: "Waiting for map".
        case empty
        case current
        /// The last good bricks, greyed: "resyncing · as of epoch N".
        case stale(StaleReason)
    }

    public enum EventsState: Sendable, Equatable {
        case complete
        /// Transitions may be missing after `since`, the stamp of the last good message.
        case incomplete(since: RoomMemory.Time?)
    }

    // Map
    public private(set) var roomId: String?
    public private(set) var bricks: [RoomMemory.BrickCoordinate: RoomMemory.Brick] = [:]
    public private(set) var voxelSize: Float?
    public private(set) var mapEpoch: UInt64?
    public private(set) var mapSeq: UInt64?
    public private(set) var mapRevision = 0
    public private(set) var mapState: MapState = .empty

    // Events
    public private(set) var events: [LiveRoomEvent] = []
    public private(set) var eventsById: [UInt64: LiveRoomEvent] = [:]
    public private(set) var eventsSeq: UInt64?
    public private(set) var eventsState: EventsState = .complete

    // Status
    public private(set) var status: LiveRoomStatus?
    public private(set) var isStatusFresh = false

    public private(set) var activity = LiveRoomActivity()

    /// The map is CURRENT and the status that vouches for it is fresh. Draw change overlays only then.
    public var isMapCurrent: Bool { mapState == .current && isStatusFresh }

    /// CANDIDATE or CONFIRMED events, by `event_id`.
    public var openEvents: [LiveRoomEvent] {
        eventsById.values
            .filter { $0.event.state == .candidate || $0.event.state == .confirmed }
            .sorted { $0.event.eventId < $1.event.eventId }
    }

    // Per connection
    private var connected = false
    private var canRequest = false
    private var sawMapThisConnection = false
    private var highestLiveMapSeq: UInt64?
    private var highestLiveEventsSeq: UInt64?
    /// Consecutive statuses whose `map_epoch` disagreed with the mirror (D20).
    private var statusEpochMismatches = 0
    /// `events_seq` of the previous status, while it was above the baseline.
    private var eventsMark: UInt64?
    /// A sync delta chained onto the mirror since it went stale: proof it can catch up.
    private var replayChained = false

    // Events bookkeeping
    /// `map_epoch` of the message at the events baseline, for `RequestSync.since`.
    private var eventsEpoch: UInt64?
    private var lastGoodEventStamp: RoomMemory.Time?
    /// Seqs above the baseline already received (seq → map_epoch); the baseline advances through them.
    private var eventsAhead: [UInt64: UInt64] = [:]
    /// The baseline skipped seqs that will never arrive, so history stays incomplete.
    private var eventsHistoryLost = false
    /// A `SINCE_NONE` events request is out: its answer, not a live event, settles what exists.
    private var eventsInitialSync = false
    /// Seqs the initial answer brought, to tell a full history from a bounded fallback.
    private var initialSyncSeqs: Set<UInt64> = []
    /// Newest message on the events sync topic, for the settle window.
    private var lastSyncEventAt: TimeInterval?
    /// Bumped when the node restarts; per-event seq order holds only within one generation.
    private var eventsGeneration = 0
    private var eventGeneration: [UInt64: Int] = [:]

    // Requests
    private struct Need: Sendable, Equatable {
        var sentAt: TimeInterval?
        var attempts = 0
    }

    private var needs: [RoomMemory.RequestSync.Topic: Need] = [:]
    private var lastSentAt: [RoomMemory.RequestSync.Topic: TimeInterval] = [:]
    private var requestCounter: UInt64 = 0

    /// - Parameter roomId: the room to mirror; nil adopts the first room a message names.
    public init(roomId: String? = nil) {
        self.roomId = roomId
    }

    // MARK: Connection

    /// The socket opened and subscribed. `canRequest` says whether any request path exists.
    public mutating func connectionOpened(at now: TimeInterval, canRequest: Bool) -> [Action] {
        connected = true
        self.canRequest = canRequest
        sawMapThisConnection = false
        highestLiveMapSeq = nil
        highestLiveEventsSeq = nil
        statusEpochMismatches = 0
        eventsMark = nil
        replayChained = false
        needs = [:]
        lastSyncEventAt = nil
        initialSyncSeqs = []
        eventsInitialSync = canRequest && eventsSeq == nil
        if canRequest {
            needs[.mapBricks] = Need()
            needs[.events] = Need()
        }
        return flushRequests(at: now)
    }

    /// The socket closed. The bricks stay, but nothing proves them current until the next connection resyncs.
    public mutating func connectionClosed(at now: TimeInterval) {
        connected = false
        needs = [:]
        if mapState == .current { markStale(.disconnected) }
        if eventsSeq != nil, eventsState == .complete {
            eventsState = .incomplete(since: lastGoodEventStamp)
        }
        refreshFreshness(at: now)
    }

    /// The client could not carry a request the mirror asked for.
    public mutating func noteRequestUnsent() {
        activity.requestsSent -= 1
        activity.requestsUnsent += 1
    }

    // MARK: Messages

    /// One message's CDR payload (the Foxglove frame without its 13-byte header).
    public mutating func receive(topic: LiveRoomTopic, payload: Data, at now: TimeInterval) -> [Action] {
        activity.note(topic, at: now)
        guard connected || !topic.isSync else { return [] }
        if topic.isSync {
            let requestTopic: RoomMemory.RequestSync.Topic = topic == .syncMapBricks ? .mapBricks : .events
            // Shared topic: answers only matter while this Mac has a request out (protocol §5).
            guard needs[requestTopic]?.sentAt != nil else { return [] }
        }
        do {
            switch topic {
            case .mapBricks, .syncMapBricks:
                let message = try RoomMemory.MapBricks.decode(cdr: payload)
                guard adopt(room: message.header.roomId) else { break }
                receiveMap(message, fromSync: topic.isSync)
            case .events, .syncEvents:
                let message = try RoomMemory.ChangeEvent.decode(cdr: payload)
                guard adopt(room: message.header.roomId) else { break }
                if topic.isSync {
                    receiveSyncEvent(message, at: now)
                } else {
                    receiveLiveEvent(message, at: now)
                }
            case .status:
                let message = try RoomMemory.MemoryStatus.decode(cdr: payload)
                guard adopt(room: message.header.roomId) else { break }
                receiveStatus(message, at: now)
            }
        } catch {
            activity.rejected += 1
        }
        refreshFreshness(at: now)
        return flushRequests(at: now)
    }

    /// Clock tick: status freshness and request repeats.
    public mutating func tick(at now: TimeInterval) -> [Action] {
        refreshFreshness(at: now)
        settleEventsAnswer(at: now)
        return flushRequests(at: now)
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

    private mutating func refreshFreshness(at now: TimeInterval) {
        isStatusFresh = connected && status.map { now - $0.receivedAt < Self.statusFreshFor } ?? false
    }

    // MARK: Map

    private mutating func receiveMap(_ message: RoomMemory.MapBricks, fromSync: Bool) {
        let seq = message.header.seq
        let firstOnConnection = !sawMapThisConnection
        if !fromSync {
            sawMapThisConnection = true
            highestLiveMapSeq = max(highestLiveMapSeq ?? seq, seq)
        }

        switch message.kind {
        case .snapshot:
            // Always accepted: resets the bricks, the epoch and the seq baseline (protocol §5).
            setBricks(Dictionary(uniqueKeysWithValues: message.bricks.map { ($0.coordinate, $0) }))
            voxelSize = message.voxelSize
            mapEpoch = message.header.mapEpoch
            mapSeq = seq
            statusEpochMismatches = 0
            if !fromSync {
                // The live topic is in order: its snapshot is the newest there is, even after a restart.
                highestLiveMapSeq = seq
            }
            if seq >= (highestLiveMapSeq ?? seq) {
                mapState = .current
                replayChained = false
                needs[.mapBricks] = nil
            } else {
                // A replayed snapshot behind the live topic: the replay's deltas must chain on before it is current.
                mapState = .stale(.seqGap)
                replayChained = true
            }

        case .delta:
            guard let epoch = mapEpoch, let last = mapSeq else {
                // Nothing to apply it to.
                needMap()
                return
            }
            if seq <= last, message.header.mapEpoch <= epoch {
                activity.duplicates += 1
                return
            }
            if fromSync, seq <= last {
                return
            }
            if !fromSync, firstOnConnection {
                markStale(.deltaFirst)
                needMap()
                return
            }
            guard seq == last &+ 1 else {
                activity.gaps += 1
                markStale(.seqGap)
                needMap()
                return
            }
            guard message.baseEpoch == epoch else {
                markStale(.wrongBase)
                needMap()
                return
            }
            guard let next = Self.applying(message, to: bricks, voxelSize: voxelSize) else {
                activity.rejected += 1
                markStale(.rejected)
                needMap()
                return
            }
            setBricks(next)
            mapEpoch = message.header.mapEpoch
            mapSeq = seq
            statusEpochMismatches = 0
            if fromSync { replayChained = true }
            reevaluateMap()
        }
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

    private mutating func setBricks(_ next: [RoomMemory.BrickCoordinate: RoomMemory.Brick]) {
        if next != bricks {
            bricks = next
            mapRevision += 1
        }
    }

    private mutating func markStale(_ reason: StaleReason) {
        guard mapEpoch != nil else { return }
        switch mapState {
        case .current:
            mapState = .stale(reason)
            replayChained = false
        case .stale(.epochMismatch) where reason != .epochMismatch:
            // A broken chain outranks a lagging status: only a resync recovers now.
            mapState = .stale(reason)
        case .stale, .empty:
            break
        }
    }

    private mutating func needMap() {
        guard canRequest, needs[.mapBricks] == nil else { return }
        needs[.mapBricks] = Need()
    }

    /// A stale map is current again once it has applied every live `seq` seen and the newest
    /// status agrees on the epoch, and it has proof it chains (protocol §5).
    private mutating func reevaluateMap() {
        guard case .stale(let reason) = mapState, let epoch = mapEpoch, let seq = mapSeq else { return }
        guard reason == .epochMismatch || replayChained else { return }
        guard seq >= (highestLiveMapSeq ?? seq) else { return }
        if let status, status.mapEpoch != epoch { return }
        mapState = .current
        needs[.mapBricks] = nil
    }

    // MARK: Events

    private mutating func receiveLiveEvent(_ message: RoomMemory.ChangeEvent, at now: TimeInterval) {
        let seq = message.header.seq
        highestLiveEventsSeq = max(highestLiveEventsSeq ?? seq, seq)

        guard let base = eventsSeq else {
            // First message after launch (or after a reset): it starts the baseline.
            record(message, at: now, fromSync: false)
            moveBaseline(to: seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
            resolveEventsIfDone()
            return
        }
        if seq == base &+ 1 {
            record(message, at: now, fromSync: false)
            moveBaseline(to: seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
            advanceThroughAhead()
            resolveEventsIfDone()
            return
        }
        if seq <= base || eventsAhead[seq] != nil {
            if let held = eventsById[message.eventId], held.event.state == message.state, held.seq >= seq {
                activity.duplicates += 1
                return
            }
        }
        activity.gaps += 1
        if eventsState == .complete {
            eventsState = .incomplete(since: lastGoodEventStamp)
        }
        if seq <= base {
            // A lower seq that is no duplicate: the node restarted. A new generation starts here.
            eventsGeneration += 1
        }
        record(message, at: now, fromSync: false)
        if seq <= base {
            highestLiveEventsSeq = seq
            eventsAhead = [:]
            eventsHistoryLost = true
            moveBaseline(to: seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
        } else if canRequest {
            // Hold the baseline so the replay can fill the gap; remember what is already here.
            holdAhead(seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
        } else {
            // Nothing can fill it: skip over.
            eventsAhead = [:]
            eventsHistoryLost = true
            moveBaseline(to: seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
        }
        needEvents()
    }

    private mutating func receiveSyncEvent(_ message: RoomMemory.ChangeEvent, at now: TimeInterval) {
        let seq = message.header.seq
        lastSyncEventAt = now
        if eventsInitialSync, initialSyncSeqs.count < Self.eventCapacity * 2 {
            initialSyncSeqs.insert(seq)
        }
        guard let base = eventsSeq else {
            record(message, at: now, fromSync: true)
            moveBaseline(to: seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
            return
        }
        if seq <= base {
            // Older than the baseline (an old open event in a fallback): it may only add what the
            // Mac never had, never roll an event back.
            record(message, at: now, fromSync: true, onlyIfNewer: true)
            return
        }
        if eventsAhead[seq] != nil { return }
        record(message, at: now, fromSync: true)
        if seq == base &+ 1 {
            moveBaseline(to: seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
            advanceThroughAhead()
        } else {
            holdAhead(seq, epoch: message.header.mapEpoch, stamp: message.header.stamp)
        }
        resolveEventsIfDone()
    }

    private mutating func moveBaseline(to seq: UInt64, epoch: UInt64, stamp: RoomMemory.Time) {
        eventsSeq = seq
        eventsEpoch = epoch
        lastGoodEventStamp = stamp
    }

    private mutating func advanceThroughAhead() {
        while let base = eventsSeq, let epoch = eventsAhead.removeValue(forKey: base &+ 1) {
            eventsSeq = base &+ 1
            eventsEpoch = epoch
            if let latest = events.last(where: { $0.seq == base &+ 1 }) {
                lastGoodEventStamp = latest.event.header.stamp
            }
        }
    }

    /// Remembers a seq received above the baseline. Past `eventCapacity` of them the baseline
    /// gives up on the gap and jumps to this one.
    private mutating func holdAhead(_ seq: UInt64, epoch: UInt64, stamp: RoomMemory.Time) {
        eventsAhead[seq] = epoch
        guard eventsAhead.count > Self.eventCapacity, seq == eventsAhead.keys.max() else { return }
        eventsAhead = [:]
        eventsHistoryLost = true
        moveBaseline(to: seq, epoch: epoch, stamp: stamp)
    }

    /// Once an events answer has gone quiet, what it could not fill is lost: a bounded fallback
    /// (protocol §5) skips to the highest seq it replayed and leaves the history incomplete.
    private mutating func settleEventsAnswer(at now: TimeInterval) {
        guard let sentAt = needs[.events]?.sentAt, let last = lastSyncEventAt, last >= sentAt,
              now - last >= Self.syncSettle else { return }
        lastSyncEventAt = nil
        if let top = eventsAhead.keys.max(), let epoch = eventsAhead[top] {
            eventsAhead = [:]
            loseEventsHistory()
            let stamp = events.last(where: { $0.seq == top })?.event.header.stamp ?? lastGoodEventStamp ?? RoomMemory.Time(sec: 0, nanosec: 0)
            moveBaseline(to: top, epoch: epoch, stamp: stamp)
        }
        if eventsInitialSync {
            eventsInitialSync = false
            // A full history runs 0, 1, 2, … with no hole; anything else is the bounded fallback.
            if let low = initialSyncSeqs.min(), let high = initialSyncSeqs.max(),
               !(low == 0 && UInt64(initialSyncSeqs.count) == high - low + 1) {
                loseEventsHistory()
            }
            initialSyncSeqs = []
        }
        resolveEventsIfDone()
    }

    private mutating func loseEventsHistory() {
        eventsHistoryLost = true
        if eventsState == .complete {
            eventsState = .incomplete(since: lastGoodEventStamp)
        }
    }

    /// Appends to the ring, and moves the event's state only on a newer seq. With `onlyIfNewer`,
    /// a message that would not move its event is dropped entirely.
    private mutating func record(
        _ message: RoomMemory.ChangeEvent,
        at now: TimeInterval,
        fromSync: Bool,
        onlyIfNewer: Bool = false
    ) {
        let entry = LiveRoomEvent(seq: message.header.seq, receivedAt: now, fromSync: fromSync, event: message)
        if onlyIfNewer, let held = eventsById[message.eventId], held.seq >= entry.seq,
           eventGeneration[message.eventId] == eventsGeneration {
            return
        }
        events.append(entry)
        if events.count > Self.eventCapacity {
            events.removeFirst(events.count - Self.eventCapacity)
        }
        if let held = eventsById[message.eventId], held.seq >= entry.seq,
           eventGeneration[message.eventId] == eventsGeneration {
            return
        }
        eventsById[message.eventId] = entry
        eventGeneration[message.eventId] = eventsGeneration
        if eventsById.count > Self.eventIdCapacity {
            let terminal = eventsById.values.filter { ![.candidate, .confirmed].contains($0.event.state) }
            if let oldest = (terminal.isEmpty ? Array(eventsById.values) : terminal).min(by: { $0.seq < $1.seq }) {
                eventsById[oldest.event.eventId] = nil
                eventGeneration[oldest.event.eventId] = nil
            }
        }
    }

    private mutating func needEvents() {
        guard canRequest, needs[.events] == nil else { return }
        needs[.events] = Need()
    }

    /// An events request is done once the baseline reaches both the newest live seq and the
    /// status high-water mark, with nothing held ahead (Phase 3 §4.3).
    private mutating func resolveEventsIfDone() {
        if eventsSeq == nil, let status, status.eventsSeq == nil {
            // Nothing held and the Spark has published no event: nothing to fetch.
            eventsInitialSync = false
            needs[.events] = nil
            return
        }
        // A live event is no proof the history before it was fetched: the initial answer settles that.
        guard let base = eventsSeq, !eventsInitialSync, eventsAhead.isEmpty else { return }
        guard base >= (highestLiveEventsSeq ?? base), base >= (status?.eventsSeq ?? base) else { return }
        needs[.events] = nil
        if !eventsHistoryLost, connected {
            eventsState = .complete
        }
    }

    // MARK: Status

    private mutating func receiveStatus(_ message: RoomMemory.MemoryStatus, at now: TimeInterval) {
        if let status, status.seq == message.header.seq, status.stamp == message.header.stamp {
            // A cached copy delivered twice.
            activity.duplicates += 1
            return
        }
        status = LiveRoomStatus(message, receivedAt: now)

        // D20: a lost trailing delta shows as the status epoch disagreeing twice in a row.
        if let epoch = mapEpoch, mapState != .empty {
            if message.header.mapEpoch != epoch {
                statusEpochMismatches += 1
                if statusEpochMismatches >= 2, mapState == .current {
                    markStale(.epochMismatch)
                    needMap()
                }
            } else {
                statusEpochMismatches = 0
                reevaluateMap()
            }
        }

        // events_seq gap: a high-water mark the baseline still has not reached one status later.
        if eventsSeq == nil, let mark = message.eventsSeq {
            // Events exist but none is held (the first was lost, or none has arrived yet).
            if eventsMark != nil, needs[.events] == nil {
                if canRequest {
                    activity.gaps += 1
                    if eventsState == .complete {
                        eventsState = .incomplete(since: lastGoodEventStamp)
                    }
                    eventsInitialSync = true
                    needEvents()
                } else if eventsState == .complete {
                    activity.gaps += 1
                    loseEventsHistory()
                }
            }
            eventsMark = mark
        }
        if let base = eventsSeq, let mark = message.eventsSeq {
            if mark > base {
                if let previous = eventsMark, base < previous, needs[.events] == nil, eventsAhead.isEmpty {
                    activity.gaps += 1
                    if eventsState == .complete {
                        eventsState = .incomplete(since: lastGoodEventStamp)
                    }
                    if canRequest {
                        needEvents()
                    } else {
                        eventsHistoryLost = true
                    }
                }
                eventsMark = mark
            } else {
                eventsMark = nil
            }
        }
        resolveEventsIfDone()
    }

    // MARK: Requests

    private mutating func flushRequests(at now: TimeInterval) -> [Action] {
        guard connected, canRequest, roomId != nil else { return [] }
        var actions: [Action] = []
        for topic in RoomMemory.RequestSync.Topic.allCases {
            guard var need = needs[topic] else { continue }
            if let sentAt = need.sentAt {
                let wait = Self.requestRetry[min(need.attempts - 1, Self.requestRetry.count - 1)]
                guard now - sentAt >= wait else { continue }
            }
            if let last = lastSentAt[topic], now - last < Self.requestMinInterval { continue }
            guard let request = makeRequest(topic) else { continue }
            need.sentAt = now
            need.attempts += 1
            needs[topic] = need
            lastSentAt[topic] = now
            activity.requestsSent += 1
            activity.lastRequest = request
            actions.append(.requestSync(request))
        }
        return actions
    }

    private mutating func makeRequest(_ topic: RoomMemory.RequestSync.Topic) -> RoomMemory.RequestSync? {
        guard let roomId else { return nil }
        let since: RoomMemory.RequestSync.Position?
        switch topic {
        case .mapBricks:
            since = mapEpoch.flatMap { epoch in mapSeq.map { .init(epoch: epoch, seq: $0) } }
        case .events:
            since = eventsEpoch.flatMap { epoch in eventsSeq.map { .init(epoch: epoch, seq: $0) } }
        }
        defer { requestCounter += 1 }
        return RoomMemory.RequestSync(
            header: RoomMemory.Header(
                roomId: roomId,
                mapEpoch: since?.epoch ?? RoomMemory.RequestSync.sinceNone,
                seq: requestCounter,
                stamp: RoomMemory.Time(sec: 0, nanosec: 0)
            ),
            since: since,
            topic: topic
        )
    }
}
