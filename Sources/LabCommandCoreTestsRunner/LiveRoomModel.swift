import Foundation
import LabCommandCore

// MARK: - Live Room reference model (PLAN-PUBGEN §6.1)
//
// A publisher simulator, a network simulator and an oracle, written from the protocol text and
// independent of `RoomMemoryMirror`: plain arrays, dictionaries and `Set<UInt64>`. The oracle
// never reads the mirror. `LiveRoomTraceTests.swift` drives seeded traces through it and compares
// after every input.

typealias RM = RoomMemory

// MARK: Message builders

/// Room-memory messages for the Live Room tests. A nil generation builds the legacy layout.
enum LRWire {
    static let room = "lab"

    static func stamp(_ seq: UInt64) -> RM.Time {
        RM.Time(sec: 1_760_400_000 + Int32(seq % 100_000), nanosec: 0)
    }

    static func cdr(_ encode: () throws -> Data, _ what: String) -> Data {
        do { return try encode() } catch {
            expect(false, "LiveRoom fixture \(what) encodes: \(error)")
            return Data()
        }
    }

    /// A brick with one known voxel at `i`; `occupied` sets that voxel occupied too.
    static func brick(_ i: Int16, occupied: Bool = false, pattern: UInt8 = 1) -> RM.Brick {
        var known = [UInt8](repeating: 0, count: RM.Brick.maskBytes)
        var occ = known
        known[0] = pattern | 1
        if occupied { occ[0] = pattern | 1 }
        return RM.Brick(coordinate: RM.BrickCoordinate(i: i, j: 0, k: 0), occupied: occ, known: known)
    }

    static func status(_ generation: UInt64?, seq: UInt64, epoch: UInt64, eventsSeq: UInt64?,
                       state: RM.MemoryState = .tracking, room: String = room) -> Data {
        let message = RM.MemoryStatus(
            header: RM.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp(seq)), state: state,
            brickCount: 0, openEventCount: 0, coverageKnownFraction: 0.5, poseResidualM: nil, windowS: 1,
            mid360Hz: 10, jt128Hz: nil, droppedWindows: 0, eventsSeq: eventsSeq, publisherGeneration: generation
        )
        return cdr(message.encodedCDR, "status g\(String(describing: generation)) s\(seq)")
    }

    /// An inactive `Activity` (nothing moving), or an active one with one JT128 region.
    static func activity(_ generation: UInt64?, seq: UInt64, epoch: UInt64 = 5, active: Bool = false, room: String = room) -> Data {
        let message = RM.Activity(
            header: RM.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp(seq)), active: active,
            sensors: active ? .jt128 : [], largestSensor: active ? .jt128 : [], regionCount: active ? 1 : 0,
            voxelCount: active ? 12 : 0, centroid: active ? SIMD3(1, 1, 0.5) : nil,
            aabb: active ? RM.ChangeEvent.Box(min: SIMD3(0.9, 0.9, 0.4), max: SIMD3(1.1, 1.1, 0.6)) : nil,
            sinceS: active ? 0 : nil, publisherGeneration: generation
        )
        return cdr(message.encodedCDR, "activity g\(String(describing: generation)) s\(seq)")
    }

    static func epochTick(_ generation: UInt64?, seq: UInt64, epoch: UInt64 = 5, room: String = room) -> Data {
        let message = RM.Epoch(
            header: RM.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp(seq)), appearedCount: 0,
            disappearedCount: 0, movedCount: 0, coverageKnownFraction: 0.5, state: .tracking, publisherGeneration: generation
        )
        return cdr(message.encodedCDR, "epoch g\(String(describing: generation)) s\(seq)")
    }

    static func snapshot(_ generation: UInt64?, epoch: UInt64, seq: UInt64, _ bricks: [RM.Brick], room: String = room) -> Data {
        let message = RM.MapBricks(header: RM.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp(seq)), kind: .snapshot,
                                   baseEpoch: 0, voxelSize: 0.05, bricks: bricks, publisherGeneration: generation)
        return cdr(message.encodedCDR, "snapshot e\(epoch) s\(seq)")
    }

    static func delta(_ generation: UInt64?, epoch: UInt64, seq: UInt64, base: UInt64, _ bricks: [RM.Brick],
                      removed: [UInt32] = []) -> Data {
        let message = RM.MapBricks(header: RM.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp(seq)), kind: .delta,
                                   baseEpoch: base, voxelSize: 0.05, bricks: bricks, removedBrickIndices: removed,
                                   publisherGeneration: generation)
        return cdr(message.encodedCDR, "delta e\(epoch) s\(seq)")
    }

    static func event(_ generation: UInt64?, seq: UInt64, id: UInt64, state: RM.ChangeEvent.State = .candidate,
                      epoch: UInt64 = 5) -> Data {
        let header = RM.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp(seq))
        let message = RM.ChangeEvent(
            header: header, eventId: id, kind: .appeared, state: state,
            centroid: SIMD3(1, 1, 1), aabb: .init(min: SIMD3(0.9, 0.9, 0.9), max: SIMD3(1.1, 1.1, 1.1)),
            fromCentroid: nil, voxelCount: 8, volumeM3: 0.001, evidence: nil, alpha: nil,
            sensors: .jt128, firstSeen: header.stamp, lastSeen: header.stamp, mask: nil, publisherGeneration: generation
        )
        return cdr(message.encodedCDR, "event \(id) s\(seq)")
    }

    static func syncEnd(_ generation: UInt64, topic: RM.RequestSync.Topic, token: RM.RequestToken, bound: UInt64?,
                        delivered: [ClosedRange<UInt64>] = [], unavailable: [ClosedRange<UInt64>] = [],
                        seq: UInt64 = 0, epoch: UInt64 = 5) -> Data {
        let message = RM.SyncEnd(header: RM.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp(seq)),
                                 publisherGeneration: generation, token: token, topic: topic, bound: bound,
                                 delivered: delivered, unavailable: unavailable)
        return cdr(message.encodedCDR, "SyncEnd \(topic)")
    }

    static func requests(_ actions: [RoomMemoryMirror.Action]) -> [RM.RequestSync] {
        actions.map { action in
            switch action {
            case .requestSync(let request): return request
            }
        }
    }

    /// The single request in `actions` for `topic`, round-tripped through the codec as the bridge would carry it.
    static func request(_ actions: [RoomMemoryMirror.Action], _ topic: RM.RequestSync.Topic, _ context: String) -> RM.RequestSync? {
        let matching = requests(actions).filter { $0.topic == topic }
        expect(matching.count <= 1, "\(context): at most one \(topic) request, got \(matching.count)")
        guard let sent = matching.first else { return nil }
        do {
            let decoded = try RM.RequestSync.decode(cdr: try sent.encodedCDR())
            expect(decoded == sent, "\(context): RequestSync round-trips through CDR")
            return decoded
        } catch {
            expect(false, "\(context): RequestSync encodes and decodes: \(error)")
            return nil
        }
    }
}

/// Canonical ranges of a set of seqs, plain loops.
func lrRanges(_ seqs: Set<UInt64>) -> [ClosedRange<UInt64>] {
    var out: [ClosedRange<UInt64>] = []
    for seq in seqs.sorted() {
        if let last = out.last, last.upperBound &+ 1 == seq {
            out[out.count - 1] = last.lowerBound...seq
        } else {
            out.append(seq...seq)
        }
    }
    return out
}

func lrSeqs(_ ranges: [ClosedRange<UInt64>]) -> Set<UInt64> {
    var out = Set<UInt64>()
    for range in ranges {
        for seq in range { out.insert(seq) }
    }
    return out
}

// MARK: Publisher simulator

/// What a delivered message carried, for the oracle (it never decodes the mirror's view).
struct LRMeta: Equatable {
    enum Kind: Equatable {
        case status(seq: UInt64, epoch: UInt64, eventsSeq: UInt64?)
        case event(seq: UInt64, id: UInt64, state: RM.ChangeEvent.State)
        case map(seq: UInt64)
        case syncEnd(topic: RM.RequestSync.Topic, token: RM.RequestToken, bound: UInt64?, unavailable: Set<UInt64>)
    }

    var generation: UInt64?
    var kind: Kind
}

struct LRMessage {
    var topic: LiveRoomTopic
    var data: Data
    var meta: LRMeta
}

/// One room-memory process and its restarts: counters from 0 per generation, a 100-message event
/// log with the latest message of each open event, a 10-message map log, keyframes, and
/// `RequestSync` answers by the tables of protocol §5 (PLAN-PUBGEN §2.3).
struct LRPublisher {
    struct Logged {
        var seq: UInt64
        var epoch: UInt64
        var message: LRMessage
    }

    /// The protocol's replay depths (§5); traces shrink them so fallbacks and lost seqs happen.
    var eventLogDepth = 100
    var mapLogDepth = 10
    var random: SplitMix64
    var legacy: Bool
    private(set) var generation: UInt64
    private(set) var mapEpoch: UInt64 = 1
    private(set) var statusSeq: UInt64 = 0
    private(set) var eventSeq: UInt64 = 0
    private(set) var mapSeq: UInt64 = 0
    private(set) var syncEndSeq: UInt64 = 0
    /// No map message yet in this generation.
    private(set) var building = true
    private(set) var bricks: [RM.BrickCoordinate: RM.Brick] = [:]
    /// The map after each map seq of each generation, and its epoch.
    private(set) var mapHistory: [UInt64: [UInt64: (epoch: UInt64, bricks: [RM.BrickCoordinate: RM.Brick])]] = [:]
    private(set) var eventLog: [Logged] = []
    private(set) var mapLog: [Logged] = []
    private(set) var openLatest: [UInt64: Logged] = [:]
    private var openState: [UInt64: RM.ChangeEvent.State] = [:]
    private var nextEventId: UInt64 = 1
    private(set) var lastStatus: LRMessage?

    init(seed: UInt64, generation: UInt64, legacy: Bool) {
        random = SplitMix64(seed: seed)
        self.generation = generation
        self.legacy = legacy
    }

    private var stamped: UInt64? { legacy ? nil : generation }

    var eventsBound: UInt64? { eventSeq == 0 ? nil : eventSeq - 1 }
    var mapBound: UInt64? { mapSeq == 0 ? nil : mapSeq - 1 }

    /// Seqs of this generation it can no longer serve: neither logged nor an open event's latest.
    var unservable: Set<UInt64> {
        guard let bound = eventsBound else { return [] }
        var held = Set(eventLog.map(\.seq))
        for entry in openLatest.values { held.insert(entry.seq) }
        var out = Set<UInt64>()
        for seq in 0...bound where !held.contains(seq) { out.insert(seq) }
        return out
    }

    /// A later process: every counter from 0, a higher generation, the next epoch, no map yet.
    mutating func restart(by step: UInt64) {
        generation += step
        mapEpoch += 1
        statusSeq = 0
        eventSeq = 0
        mapSeq = 0
        syncEndSeq = 0
        building = true
        eventLog = []
        mapLog = []
        openLatest = [:]
        openState = [:]
    }

    mutating func status() -> LRMessage {
        let seq = statusSeq
        statusSeq += 1
        let message = LRMessage(
            topic: .status,
            data: LRWire.status(stamped, seq: seq, epoch: mapEpoch, eventsSeq: eventsBound, state: building ? .building : .tracking),
            meta: LRMeta(generation: stamped, kind: .status(seq: seq, epoch: mapEpoch, eventsSeq: eventsBound))
        )
        lastStatus = message
        return message
    }

    /// One lifecycle transition: a new candidate, or the next state of an open event.
    mutating func event() -> LRMessage {
        let open = openState.keys.sorted()
        let id: UInt64
        let state: RM.ChangeEvent.State
        if open.isEmpty || random.next() % 3 == 0 {
            id = nextEventId
            nextEventId += 1
            state = .candidate
        } else {
            id = open[Int(random.next() % UInt64(open.count))]
            switch openState[id] {
            case .candidate: state = random.next() % 3 == 0 ? .expired : .confirmed
            default: state = random.next() % 2 == 0 ? .accepted : .reverted
            }
        }
        let seq = eventSeq
        eventSeq += 1
        let message = LRMessage(
            topic: .events,
            data: LRWire.event(stamped, seq: seq, id: id, state: state, epoch: mapEpoch),
            meta: LRMeta(generation: stamped, kind: .event(seq: seq, id: id, state: state))
        )
        let entry = Logged(seq: seq, epoch: mapEpoch, message: message)
        eventLog.append(entry)
        if eventLog.count > eventLogDepth { eventLog.removeFirst() }
        if state == .candidate || state == .confirmed {
            openState[id] = state
            openLatest[id] = entry
        } else {
            openState[id] = nil
            openLatest[id] = nil
        }
        return message
    }

    /// The first snapshot after a start, then a delta (new epoch) or a keyframe snapshot.
    mutating func map() -> LRMessage {
        let seq = mapSeq
        mapSeq += 1
        let data: Data
        if building || random.next() % 4 == 0 {
            if building {
                building = false
                bricks = [:]
                for i in 0..<Int16(1 + random.next() % 3) {
                    let brick = LRWire.brick(i, occupied: true)
                    bricks[brick.coordinate] = brick
                }
            }
            data = LRWire.snapshot(stamped, epoch: mapEpoch, seq: seq, bricks.keys.sorted().compactMap { bricks[$0] })
        } else {
            let base = mapEpoch
            mapEpoch += 1
            let changed = LRWire.brick(Int16(random.next() % 8) + 10, occupied: random.next() % 2 == 0,
                                       pattern: UInt8(truncatingIfNeeded: random.next()))
            // Removed indices name the base epoch's sorted brick list; a removed brick is not re-sent.
            var removed: [UInt32] = []
            let order = bricks.keys.sorted()
            if order.count > 2, random.next() % 3 == 0 {
                let index = Int(random.next() % UInt64(order.count))
                if order[index] != changed.coordinate {
                    removed = [UInt32(index)]
                    bricks[order[index]] = nil
                }
            }
            bricks[changed.coordinate] = changed
            data = LRWire.delta(stamped, epoch: mapEpoch, seq: seq, base: base, [changed], removed: removed)
        }
        mapHistory[generation, default: [:]][seq] = (mapEpoch, bricks)
        let message = LRMessage(topic: .mapBricks, data: data, meta: LRMeta(generation: stamped, kind: .map(seq: seq)))
        mapLog.append(Logged(seq: seq, epoch: mapEpoch, message: message))
        if mapLog.count > mapLogDepth { mapLog.removeFirst() }
        return message
    }

    /// The answer to one request: messages for the sync topic, then the `SyncEnd` (nil from a legacy node).
    mutating func answer(_ request: RM.RequestSync) -> [LRMessage] {
        if legacy {
            // A legacy node rejects a v2 request (≥ 4 trailing bytes); this one never answers.
            return []
        }
        let since = request.expectedGeneration == generation ? request.since : nil
        let token = request.token ?? .zero
        var out: [LRMessage] = []
        var delivered = Set<UInt64>()
        var unavailable = Set<UInt64>()
        let bound: UInt64?
        switch request.topic {
        case .mapBricks:
            bound = mapBound
            if let bound, let newest = mapLog.last, since?.epoch != newest.epoch || since?.seq != newest.seq {
                if let since, let at = mapLog.firstIndex(where: { $0.epoch == since.epoch && $0.seq == since.seq }) {
                    for entry in mapLog[(at + 1)...] {
                        out.append(sync(entry.message, .syncMapBricks))
                        delivered.insert(entry.seq)
                    }
                } else {
                    let snapshot = LRWire.snapshot(generation, epoch: mapEpoch, seq: bound, bricks.keys.sorted().compactMap { bricks[$0] })
                    out.append(LRMessage(topic: .syncMapBricks, data: snapshot, meta: LRMeta(generation: generation, kind: .map(seq: bound))))
                    delivered.insert(bound)
                }
            }
        case .events:
            bound = eventsBound
            if let bound {
                if let since, let at = eventLog.firstIndex(where: { $0.epoch == since.epoch && $0.seq == since.seq }) {
                    for entry in eventLog[(at + 1)...] {
                        out.append(sync(entry.message, .syncEvents))
                        delivered.insert(entry.seq)
                    }
                } else {
                    let oldest = eventLog.first?.seq ?? 0
                    let older = openLatest.values.filter { $0.seq < oldest }.sorted { $0.seq < $1.seq }
                    for entry in older + eventLog {
                        out.append(sync(entry.message, .syncEvents))
                        delivered.insert(entry.seq)
                    }
                    let first = since.map { $0.seq &+ 1 } ?? 0
                    if first <= bound {
                        for seq in first...bound where !delivered.contains(seq) { unavailable.insert(seq) }
                    }
                }
            }
        }
        let seq = syncEndSeq
        syncEndSeq += 1
        out.append(LRMessage(
            topic: .syncEnd,
            data: LRWire.syncEnd(generation, topic: request.topic, token: token, bound: bound, delivered: lrRanges(delivered),
                                 unavailable: lrRanges(unavailable), seq: seq, epoch: mapEpoch),
            meta: LRMeta(generation: generation, kind: .syncEnd(topic: request.topic, token: token, bound: bound, unavailable: unavailable))
        ))
        return out
    }

    private func sync(_ message: LRMessage, _ topic: LiveRoomTopic) -> LRMessage {
        LRMessage(topic: topic, data: message.data, meta: message.meta)
    }
}

// MARK: Oracle

/// The expected mirror, from what was delivered to it. Plain sets; it never reads the mirror.
struct LROracle {
    struct Run {
        var generation: UInt64?
        var received: [UInt64: (id: UInt64, state: RM.ChangeEvent.State)] = [:]
        var witnessTop: UInt64?
        var witnessEmpty = false
        var unavailable = Set<UInt64>()
        var status: (seq: UInt64, at: TimeInterval, connection: Int, eventsSeq: UInt64?, epoch: UInt64)?
        var highestMapSeq: UInt64?
        var observed = false
    }

    var connection = 0
    var connected = false
    var wire: RoomMemoryMirror.Wire = .undetermined
    var highest: UInt64?
    var run: Run?
    var brokenSince: UInt64??

    mutating func opened() {
        connection += 1
        connected = true
        wire = .undetermined
    }

    mutating func closed() {
        connected = false
    }

    private mutating func retire() {
        if let old = run, old.observed { brokenSince = .some(old.generation) }
        run = nil
    }

    /// Applies one delivered message by the protocol's rules (PLAN-PUBGEN §4.3).
    mutating func deliver(_ meta: LRMeta, at now: TimeInterval) {
        guard connected else { return }
        let layout: RoomMemoryMirror.Wire = meta.generation == nil ? .legacy : .v2
        if wire == .undetermined {
            wire = layout
            if layout == .legacy {
                if run?.generation != nil { retire() }
                if run == nil { run = Run(generation: nil) }
            }
        } else if wire != layout {
            return
        }
        if let generation = meta.generation {
            if let highest, generation < highest { return }
            if highest.map({ generation > $0 }) ?? true {
                retire()
                run = Run(generation: generation)
                highest = generation
            } else if run?.generation != generation {
                retire()
                run = Run(generation: generation)
            }
        }
        guard var current = run else { return }
        switch meta.kind {
        case .status(let seq, let epoch, let eventsSeq):
            if let held = current.status, current.generation != nil ? seq <= held.seq : seq == held.seq { break }
            current.status = (seq, now, connection, eventsSeq, epoch)
            current.observed = true
            if current.generation != nil {
                if let eventsSeq { current.witnessTop = max(current.witnessTop ?? eventsSeq, eventsSeq) } else { current.witnessEmpty = true }
            }
        case .event(let seq, let id, let state):
            if current.received[seq] == nil { current.received[seq] = (id, state) }
            current.observed = true
        case .map(let seq):
            current.highestMapSeq = max(current.highestMapSeq ?? seq, seq)
            current.observed = true
        case .syncEnd(let topic, _, let bound, let unavailable):
            current.observed = true
            if topic == .events {
                if let bound { current.witnessTop = max(current.witnessTop ?? bound, bound) } else { current.witnessEmpty = true }
                current.unavailable.formUnion(unavailable)
            } else if let bound {
                current.highestMapSeq = max(current.highestMapSeq ?? bound, bound)
            }
        }
        run = current
    }

    var coverage: EventHistory.Coverage {
        guard let run, run.generation != nil else { return .unknown }
        let seqs = Set(run.received.keys)
        guard let top = [run.witnessTop, seqs.max()].compactMap({ $0 }).max() else {
            return run.witnessEmpty ? .empty : .unknown
        }
        var missing = Set<UInt64>()
        for seq in 0...top where !seqs.contains(seq) { missing.insert(seq) }
        if missing.isEmpty { return .completeThrough(top) }
        return .missing(pending: lrRanges(missing.subtracting(run.unavailable)), lost: lrRanges(missing.intersection(run.unavailable)))
    }

    func caughtUp(at now: TimeInterval) -> Bool {
        guard connected, let run, run.generation != nil, let status = run.status, status.connection == connection,
              now - status.at < RoomMemoryMirror.statusFreshFor else { return false }
        switch coverage {
        case .empty: return status.eventsSeq == nil
        case .completeThrough: return status.eventsSeq.map { $0 >= (run.received.keys.max() ?? 0) } ?? false
        case .unknown, .missing: return false
        }
    }

    /// The highest-seq transition received per event.
    var latest: [UInt64: (seq: UInt64, state: RM.ChangeEvent.State)] {
        var out: [UInt64: (seq: UInt64, state: RM.ChangeEvent.State)] = [:]
        for (seq, value) in run?.received ?? [:] where out[value.id].map({ $0.seq < seq }) ?? true {
            out[value.id] = (seq, value.state)
        }
        return out
    }

    var continuity: EventHistory.Continuity {
        if wire == .legacy || run != nil && run?.generation == nil { return .unknown }
        if let since = brokenSince { return .broken(sinceGeneration: since) }
        return run == nil ? .unknown : .verified
    }
}
