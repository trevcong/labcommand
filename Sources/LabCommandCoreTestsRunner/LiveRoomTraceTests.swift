import Foundation
import LabCommandCore

private typealias Mirror = RoomMemoryMirror

// MARK: - Live Room seeded traces, shrinking and regression seeds (PLAN-PUBGEN §6.1, §6.2)
//
// `LIVE_ROOM_TRACES=n` raises the number of random traces (a soak run); `LIVE_ROOM_SEED=s` runs
// one. A failing trace is shrunk by delta debugging and printed as Swift source.

/// One input of a trace. Every op is valid in any state (an empty queue delivers nothing), so a
/// shrunk trace still runs.
enum TraceOp: Equatable, CustomStringConvertible {
    /// The publisher emits one live event transition, map message or status.
    case event, map, status
    /// Deliver (drop, deliver twice) the head of the n-th non-empty topic queue.
    case deliver(UInt8), drop(UInt8), duplicate(UInt8)
    /// Every queue, round robin, until empty.
    case deliverAll
    /// Advance the clock by n tenths of a second and tick.
    case tick(UInt16)
    case disconnect
    case reconnect(path: Bool)
    /// The publisher restarts with its generation raised by n + 1 (counters from 0, no map yet).
    case restart(UInt16)
    /// The publisher answers the oldest (or every) request it received.
    case answer, answerAll
    /// The next request the mirror asks for goes unsent.
    case unsent
    /// Another Mac's request, from the seed: foreign replays and SyncEnds on the shared topics.
    case foreign(UInt32)

    var description: String {
        switch self {
        case .event: return ".event"
        case .map: return ".map"
        case .status: return ".status"
        case .deliver(let n): return ".deliver(\(n))"
        case .drop(let n): return ".drop(\(n))"
        case .duplicate(let n): return ".duplicate(\(n))"
        case .deliverAll: return ".deliverAll"
        case .tick(let n): return ".tick(\(n))"
        case .disconnect: return ".disconnect"
        case .reconnect(let path): return ".reconnect(path: \(path))"
        case .restart(let n): return ".restart(\(n))"
        case .answer: return ".answer"
        case .answerAll: return ".answerAll"
        case .unsent: return ".unsent"
        case .foreign(let seed): return ".foreign(\(seed))"
        }
    }
}

struct TraceConfig {
    var seed: UInt64
    var legacy = false
    var limits = RoomMemoryMirror.Limits()
    /// Small limits: the mirror's claims need only be implied by the oracle's.
    var evicting = false
    /// Append the quiet tail and check liveness after it.
    var tail = true
}

/// The quiet tail: a fresh connection with a request path, lossless delivery, every request
/// answered, ticks past every backoff, then a final status.
let liveRoomQuietTail: [TraceOp] = {
    var ops: [TraceOp] = [.disconnect, .reconnect(path: true), .map]
    for _ in 0..<35 {
        ops += [.status, .deliverAll, .answerAll, .deliverAll, .tick(20)]
    }
    return ops + [.status, .deliverAll]
}()

private struct TraceWorld {
    static let topics: [LiveRoomTopic] = [.mapBricks, .events, .status, .syncMapBricks, .syncEvents, .syncEnd]

    var config: TraceConfig
    var mirror: Mirror
    var publisher: LRPublisher
    var oracle = LROracle()
    var queues: [LiveRoomTopic: [LRMessage]] = [:]
    var requests: [RM.RequestSync] = []
    var connected = false
    var path = false
    var unsentArmed = false
    var now: TimeInterval = 0
    var tokens = Set<RM.RequestToken>()
    var highest: UInt64?
    var foreignRandom: SplitMix64
    var deliveries = 0
    var maxLost = 0
    var mapCurrentChecks = 0
    var completions = 0

    init(_ config: TraceConfig) {
        self.config = config
        mirror = Mirror(limits: config.limits, tokenSeed: config.seed)
        publisher = LRPublisher(seed: config.seed &* 31 &+ 7, generation: 1_760_400_000_000 + config.seed % 1000, legacy: config.legacy)
        // Short logs, so replays fall back and seqs become unavailable within a trace.
        publisher.eventLogDepth = 4 + Int(config.seed % 13)
        publisher.mapLogDepth = 2 + Int(config.seed % 5)
        foreignRandom = SplitMix64(seed: config.seed ^ 0xF0F0)
    }

    var nonEmpty: [LiveRoomTopic] { Self.topics.filter { !(queues[$0]?.isEmpty ?? true) } }

    mutating func publish(_ message: LRMessage) {
        guard connected else { return }
        queues[message.topic, default: []].append(message)
    }

    mutating func pop(_ n: UInt8) -> LRMessage? {
        let topics = nonEmpty
        guard !topics.isEmpty else { return nil }
        let topic = topics[Int(n) % topics.count]
        return queues[topic]?.removeFirst()
    }

    /// The mirror's semantic state: what a duplicate delivery must not change.
    func projection() -> String {
        let latest = mirror.eventsById.keys.sorted().map { "\($0):\(mirror.eventsById[$0]!.seq):\(mirror.eventsById[$0]!.event.state)" }
        return [
            "\(String(describing: mirror.activeGeneration))", "\(mirror.eventHistory.coverage)", "\(mirror.continuity)",
            "\(mirror.mapState)", "\(String(describing: mirror.mapSeq))", "\(String(describing: mirror.mapEpoch))",
            "\(mirror.bricks.count)", "\(String(describing: mirror.status?.seq))", latest.joined(separator: ","),
        ].joined(separator: "|")
    }

    mutating func handle(_ actions: [Mirror.Action]) -> String? {
        for request in LRWire.requests(actions) {
            if mirror.wire != .v2 { return "a request on a \(mirror.wire) wire" }
            guard let token = request.token, !token.isZero, request.expectedGeneration == mirror.activeGeneration else {
                return "request without a token or with expected_generation \(String(describing: request.expectedGeneration)) ≠ active \(String(describing: mirror.activeGeneration))"
            }
            if !tokens.insert(token).inserted { return "token \(token) reused" }
            if !path { return "a request with no request path" }
            if unsentArmed {
                unsentArmed = false
                mirror.noteRequestUnsent()
            } else {
                requests.append(request)
            }
        }
        return nil
    }

    mutating func deliver(_ message: LRMessage) -> String? {
        let outstanding = RM.RequestSync.Topic.allCases.filter { mirror.retrievalStatus($0) == .receiving }
        let actions = mirror.receive(topic: message.topic, payload: message.data, at: now)
        oracle.deliver(message.meta, at: now)
        deliveries += 1
        completions += outstanding.filter { mirror.retrievalStatus($0) == .idle }.count
        if case .missing(_, let lost) = mirror.eventHistory.coverage { maxLost = max(maxLost, lrSeqs(lost).count) }
        if mirror.mapState == .current { mapCurrentChecks += 1 }
        return handle(actions) ?? check()
    }

    mutating func foreignRequest(_ seed: UInt32) -> RM.RequestSync {
        var random = SplitMix64(seed: UInt64(seed) ^ foreignRandom.next())
        let topic: RM.RequestSync.Topic = random.next() % 2 == 0 ? .events : .mapBricks
        let log = topic == .events ? publisher.eventLog : publisher.mapLog
        var since: RM.RequestSync.Position?
        switch random.next() % 3 {
        case 0: since = nil
        case 1:
            if !log.isEmpty {
                let entry = log[Int(random.next() % UInt64(log.count))]
                since = RM.RequestSync.Position(epoch: entry.epoch, seq: entry.seq)
            }
        default: since = .init(epoch: random.next() % 4, seq: random.next() % 40)
        }
        let expected: UInt64 = [publisher.generation, 0, publisher.generation - 1][Int(random.next() % 3)]
        return RM.RequestSync(
            header: RM.Header(roomId: LRWire.room, mapEpoch: since?.epoch ?? RM.RequestSync.sinceNone, seq: 0, stamp: RM.Time(sec: 0, nanosec: 0)),
            since: since, topic: topic, expectedGeneration: expected, token: RM.RequestToken(using: &random)
        )
    }

    mutating func run(_ op: TraceOp) -> String? {
        now += 0.01
        switch op {
        case .event: publish(publisher.event())
        case .map: publish(publisher.map())
        case .status: publish(publisher.status())
        case .deliver(let n):
            if let message = pop(n) { return deliver(message) }
        case .drop(let n):
            _ = pop(n)
        case .duplicate(let n):
            guard let message = pop(n) else { return nil }
            if let failure = deliver(message) { return failure }
            let before = projection()
            if let failure = deliver(message) { return failure }
            if projection() != before { return "a duplicate \(message.topic.rawValue) changed semantic state:\n  \(before)\n  \(projection())" }
        case .deliverAll:
            while !nonEmpty.isEmpty {
                for topic in nonEmpty {
                    if let message = queues[topic]?.removeFirst(), let failure = deliver(message) { return failure }
                }
            }
        case .tick(let tenths):
            now += Double(tenths) / 10
            return handle(mirror.tick(at: now)) ?? check()
        case .disconnect:
            guard connected else { return nil }
            connected = false
            queues = [:]
            mirror.connectionClosed(at: now)
            oracle.closed()
            return check()
        case .reconnect(let withPath):
            guard !connected else { return nil }
            connected = true
            path = withPath
            oracle.opened()
            if let failure = handle(mirror.connectionOpened(at: now, canRequest: withPath)) { return failure }
            // The bridge may hand a new subscriber the last status again.
            if let last = publisher.lastStatus { publish(last) }
            return check()
        case .restart(let step):
            publisher.restart(by: UInt64(step) + 1)
        case .answer:
            guard !requests.isEmpty else { return nil }
            for message in publisher.answer(requests.removeFirst()) { publish(message) }
        case .answerAll:
            while !requests.isEmpty {
                for message in publisher.answer(requests.removeFirst()) { publish(message) }
            }
        case .unsent:
            unsentArmed = true
        case .foreign(let seed):
            guard !config.legacy else { return nil }
            for message in publisher.answer(foreignRequest(seed)) { publish(message) }
        }
        return nil
    }

    /// The mirror against the oracle and the invariants (PLAN-PUBGEN §4.4), after every input.
    func check() -> String? {
        let history = mirror.eventHistory
        if let highest, (mirror.highestGeneration ?? 0) < highest { return "highestGeneration fell below \(highest)" }
        if mirror.activeGeneration != oracle.run?.generation {
            return "active generation \(String(describing: mirror.activeGeneration)), oracle \(String(describing: oracle.run?.generation))"
        }
        if let active = mirror.activeGeneration, active != mirror.highestGeneration {
            return "active \(active) is not the highest generation \(String(describing: mirror.highestGeneration))"
        }
        if config.evicting {
            // Certainty may drop, never rise.
            switch history.coverage {
            case .completeThrough(let n) where oracle.coverage != .completeThrough(n):
                return "evicting: mirror claims complete through \(n), oracle \(oracle.coverage)"
            case .empty where oracle.coverage != .empty:
                return "evicting: mirror claims empty, oracle \(oracle.coverage)"
            case .missing(_, let lost) where !lrSeqs(lost).isSubset(of: oracle.run?.unavailable ?? []):
                return "evicting: lost \(lost) not declared by the publisher"
            default: break
            }
            if history.caughtUp, !oracle.caughtUp(at: now) { return "evicting: caughtUp without the oracle" }
        } else {
            if history.coverage != oracle.coverage { return "coverage \(history.coverage), oracle \(oracle.coverage)" }
            if history.caughtUp != oracle.caughtUp(at: now) { return "caughtUp \(history.caughtUp), oracle \(oracle.caughtUp(at: now))" }
            let latest = oracle.latest
            if mirror.eventsById.count != latest.count { return "eventsById holds \(mirror.eventsById.count) events, oracle \(latest.count)" }
            for (id, expected) in latest {
                guard let held = mirror.eventsById[id], held.seq == expected.seq, held.event.state == expected.state else {
                    return "event \(id): mirror \(String(describing: mirror.eventsById[id].map { ($0.seq, $0.event.state) })), oracle \(expected)"
                }
            }
        }
        if history.continuity != oracle.continuity { return "continuity \(history.continuity), oracle \(oracle.continuity)" }
        // No contamination: everything shown as current carries the active generation.
        for entry in mirror.eventsById.values where entry.event.publisherGeneration != mirror.activeGeneration {
            return "event \(entry.event.eventId) of generation \(String(describing: entry.event.publisherGeneration)) in \(String(describing: mirror.activeGeneration))"
        }
        if let status = mirror.status, status.publisherGeneration != mirror.activeGeneration {
            return "status of generation \(String(describing: status.publisherGeneration)) in \(String(describing: mirror.activeGeneration))"
        }
        if mirror.mapState == .current {
            guard mirror.wire == .v2, let generation = mirror.activeGeneration, let seq = mirror.mapSeq,
                  let held = publisher.mapHistory[generation]?[seq] else {
                return "map current without a v2 chain the publisher sent (seq \(String(describing: mirror.mapSeq)))"
            }
            if held.bricks != mirror.bricks || held.epoch != mirror.mapEpoch {
                return "map current at seq \(seq) with bricks or epoch the publisher did not have there"
            }
            if let high = oracle.run?.highestMapSeq, seq < high { return "map current at seq \(seq) below the delivered \(high)" }
        }
        if config.legacy, mirror.activity.requestsSent != 0 { return "legacy wire sent \(mirror.activity.requestsSent) requests" }
        if case .missing(_, let lost) = history.coverage, mirror.activeGeneration == publisher.generation,
           !lrSeqs(lost).isSubset(of: publisher.unservable) {
            return "lost \(lost) includes seqs the publisher can still serve"
        }
        return nil
    }

    /// After the quiet tail: everything converged.
    func checkLiveness() -> String? {
        let history = mirror.eventHistory
        if config.legacy {
            if history.coverage != .unknown || history.continuity != .unknown || history.caughtUp { return "legacy history claimed \(history)" }
            if mirror.isMapCurrent { return "legacy map current" }
            return nil
        }
        guard mirror.activeGeneration == publisher.generation else {
            return "after the tail the active generation is \(String(describing: mirror.activeGeneration)), the publisher's \(publisher.generation)"
        }
        switch (history.coverage, publisher.eventsBound) {
        case (.empty, nil): break
        case (.completeThrough(let n), let bound?) where n == bound: break
        case (.missing(let pending, let lost), _) where pending.isEmpty && lrSeqs(lost).isSubset(of: publisher.unservable): break
        default:
            return "after the tail coverage is \(history.coverage), publisher bound \(String(describing: publisher.eventsBound))"
        }
        if history.isComplete, !history.caughtUp { return "complete after the tail but not caught up" }
        for topic in RM.RequestSync.Topic.allCases where mirror.retrievalStatus(topic) != .idle {
            return "\(topic) retrieval \(mirror.retrievalStatus(topic)) after the tail"
        }
        if !mirror.isMapCurrent { return "map \(mirror.mapState) after the tail" }
        return nil
    }
}

/// What the traces exercised, so a passing run is known to have reached the hard paths.
struct TraceCoverage {
    var deliveries = 0
    var requests = 0
    var generationChanges = 0
    var lostSeqs = 0
    var completedRetrievals = 0
    var mapCurrentChecks = 0
    var traces = 0
}

var liveRoomTraceCoverage = TraceCoverage()

/// Runs one trace; nil when every check passed.
func runLiveRoomTrace(_ config: TraceConfig, _ body: [TraceOp]) -> String? {
    var world = TraceWorld(config)
    let failure = runTraceOps(&world, config, body)
    liveRoomTraceCoverage.traces += 1
    liveRoomTraceCoverage.deliveries += world.deliveries
    liveRoomTraceCoverage.requests += world.mirror.activity.requestsSent
    liveRoomTraceCoverage.generationChanges += world.mirror.activity.generationChanges
    liveRoomTraceCoverage.lostSeqs += world.maxLost
    liveRoomTraceCoverage.mapCurrentChecks += world.mapCurrentChecks
    liveRoomTraceCoverage.completedRetrievals += world.completions
    return failure
}

private func runTraceOps(_ world: inout TraceWorld, _ config: TraceConfig, _ body: [TraceOp]) -> String? {
    if let failure = world.run(.reconnect(path: true)) { return "setup: \(failure)" }
    for (index, op) in body.enumerated() {
        if let failure = world.run(op) { return "op \(index) \(op): \(failure)" }
        if let generation = world.mirror.highestGeneration { world.highest = generation }
    }
    guard config.tail else { return nil }
    for op in liveRoomQuietTail {
        if let failure = world.run(op) { return "tail \(op): \(failure)" }
    }
    return world.checkLiveness().map { "liveness: \($0)" }
}

/// A random trace body: mostly publishing and delivery, some loss, ticks, reconnects and restarts.
func liveRoomTraceBody(seed: UInt64, length: Int) -> [TraceOp] {
    var random = SplitMix64(seed: seed)
    var ops: [TraceOp] = []
    var connected = true
    for _ in 0..<length {
        let roll = random.next() % 1000
        let n = UInt8(truncatingIfNeeded: random.next())
        switch roll {
        case 0..<140: ops.append(.event)
        case 140..<200: ops.append(.map)
        case 200..<300: ops.append(.status)
        case 300..<640: ops.append(.deliver(n))
        case 640..<680: ops.append(.drop(n))
        case 680..<710: ops.append(.duplicate(n))
        case 710..<790: ops.append(.tick(UInt16(random.next() % 40)))
        case 790..<805:
            ops.append(.disconnect)
            connected = false
        case 805..<815: ops.append(.restart(UInt16(random.next() % 500)))
        case 815..<885: ops.append(.answer)
        case 885..<895: ops.append(.unsent)
        case 895..<925: ops.append(.foreign(UInt32(truncatingIfNeeded: random.next())))
        default:
            if !connected {
                ops.append(.reconnect(path: random.next() % 5 != 0))
                connected = true
            } else {
                ops.append(.deliverAll)
            }
        }
    }
    return ops
}

/// Delta debugging: removes chunks of the body while the trace still fails. Bounded runs.
func shrinkLiveRoomTrace(_ config: TraceConfig, _ body: [TraceOp]) -> [TraceOp] {
    var current = body
    var chunks = 2
    var runs = 0
    while current.count >= 2, runs < 3000 {
        let size = max(1, current.count / chunks)
        var removedAny = false
        var start = 0
        while start < current.count, runs < 3000 {
            var candidate = current
            candidate.removeSubrange(start..<min(current.count, start + size))
            runs += 1
            if runLiveRoomTrace(config, candidate) != nil {
                current = candidate
                removedAny = true
            } else {
                start += size
            }
        }
        if !removedAny {
            if size == 1 { break }
            chunks = min(current.count, chunks * 2)
        }
    }
    return current
}

private func failTrace(_ config: TraceConfig, _ body: [TraceOp], _ failure: String) {
    let minimal = shrinkLiveRoomTrace(config, body)
    let minimalFailure = runLiveRoomTrace(config, minimal) ?? failure
    let source = "[" + minimal.map(\.description).joined(separator: ", ") + "]"
    expect(false, """
        LiveRoom trace LIVE_ROOM_SEED=\(config.seed) (legacy \(config.legacy), evicting \(config.evicting)) failed: \(failure)
        minimal trace (\(minimal.count) of \(body.count) ops) fails with: \(minimalFailure)
        regression seed: runLiveRoomTrace(TraceConfig(seed: \(config.seed), legacy: \(config.legacy)), \(source))
        """)
}

// MARK: Property runs

/// Traces in the default run: sized so the debug runner stays inside the 60 s budget.
private let defaultTraceCount = 2000

func testLiveRoomSeededTraces() -> (traces: Int, seconds: Double) {
    let environment = ProcessInfo.processInfo.environment
    let started = Date()
    if let one = environment["LIVE_ROOM_SEED"].flatMap(UInt64.init) {
        for legacy in [false, true] {
            let config = TraceConfig(seed: one, legacy: legacy)
            let body = liveRoomTraceBody(seed: one, length: 220)
            if let failure = runLiveRoomTrace(config, body) { failTrace(config, body, failure) }
        }
        return (2, Date().timeIntervalSince(started))
    }
    let count = environment["LIVE_ROOM_TRACES"].flatMap(Int.init) ?? defaultTraceCount
    for index in 0..<count {
        let seed = UInt64(index) &* 0x9E37_79B9 &+ 17
        // Every 8th trace runs on the legacy wire, every 16th with small limits.
        var config = TraceConfig(seed: seed, legacy: index % 8 == 7)
        if index % 16 == 5 {
            config.limits = Mirror.Limits(eventLog: 5, latest: 6, digest: 8, coverageIntervals: 3)
            config.evicting = true
            config.tail = false
        }
        let body = liveRoomTraceBody(seed: seed, length: 160 + Int(seed % 80))
        if let failure = runLiveRoomTrace(config, body) { failTrace(config, body, failure) }
    }
    return (count, Date().timeIntervalSince(started))
}

/// Two delivery orders of the same set of same-generation messages: same C, U, H and latest.
func testLiveRoomOrderIndependence() {
    for seed in UInt64(0)..<40 {
        var publisher = LRPublisher(seed: seed, generation: 1_760_400_000_000, legacy: false)
        var messages: [LRMessage] = [publisher.status()]
        var random = SplitMix64(seed: seed ^ 0xABCD)
        for _ in 0..<(20 + Int(random.next() % 120)) {
            switch random.next() % 10 {
            case 0..<6: messages.append(publisher.event())
            case 6, 7: messages.append(publisher.status())
            default:
                let request = RM.RequestSync(
                    header: RM.Header(roomId: LRWire.room, mapEpoch: RM.RequestSync.sinceNone, seq: 0, stamp: RM.Time(sec: 0, nanosec: 0)),
                    since: nil, topic: .events, expectedGeneration: 0, token: RM.RequestToken(using: &random))
                messages += publisher.answer(request).filter { $0.topic != .syncEvents || random.next() % 2 == 0 }
            }
        }
        var shuffled = messages
        for index in stride(from: shuffled.count - 1, to: 0, by: -1) {
            shuffled.swapAt(index, Int(random.next() % UInt64(index + 1)))
        }
        func feed(_ list: [LRMessage]) -> Mirror {
            var mirror = Mirror(tokenSeed: seed)
            _ = mirror.connectionOpened(at: 0, canRequest: false)
            for message in list { _ = mirror.receive(topic: message.topic, payload: message.data, at: 0) }
            return mirror
        }
        let a = feed(messages)
        let b = feed(shuffled)
        expect(a.eventHistory.coverage == b.eventHistory.coverage,
               "order independence seed \(seed): coverage \(a.eventHistory.coverage) vs \(b.eventHistory.coverage)")
        let latest = { (m: Mirror) in m.eventsById.mapValues { "\($0.seq):\($0.event.state)" } }
        expect(latest(a) == latest(b), "order independence seed \(seed): latest transitions differ")
    }
}

// MARK: Regression seeds: the ten Codex findings and Astra's pair, on both wires

private let g1: UInt64 = 1_760_400_000_000
private let g2: UInt64 = 1_760_400_090_000

/// A mirror on one connection. `generation` nil drives the legacy wire.
private struct Seed {
    var mirror: Mirror
    let generation: UInt64?
    var now: TimeInterval = 0
    var sent: [RM.RequestSync] = []

    init(_ generation: UInt64?, canRequest: Bool = true) {
        self.generation = generation
        mirror = Mirror(roomId: LRWire.room, tokenSeed: 99)
        sent += LRWire.requests(mirror.connectionOpened(at: 0, canRequest: canRequest))
    }

    var legacy: Bool { generation == nil }

    mutating func feed(_ topic: LiveRoomTopic, _ payload: Data, after dt: TimeInterval = 0.01) {
        now += dt
        sent += LRWire.requests(mirror.receive(topic: topic, payload: payload, at: now))
    }

    mutating func status(_ seq: UInt64, eventsSeq: UInt64?, epoch: UInt64 = 5, generation: UInt64?? = nil) {
        feed(.status, LRWire.status(generation ?? self.generation, seq: seq, epoch: epoch, eventsSeq: eventsSeq))
    }

    mutating func event(_ seq: UInt64, id: UInt64 = 1, state: RM.ChangeEvent.State = .candidate, sync: Bool = false,
                        generation: UInt64?? = nil) {
        feed(sync ? .syncEvents : .events, LRWire.event(generation ?? self.generation, seq: seq, id: id, state: state))
    }

    mutating func tick(_ dt: TimeInterval) {
        now += dt
        sent += LRWire.requests(mirror.tick(at: now))
    }

    mutating func reconnect(canRequest: Bool = true) {
        mirror.connectionClosed(at: now)
        now += 1
        sent += LRWire.requests(mirror.connectionOpened(at: now, canRequest: canRequest))
    }

    var coverage: EventHistory.Coverage { mirror.eventHistory.coverage }

    func lastRequest(_ topic: RM.RequestSync.Topic) -> RM.RequestSync? { sent.last { $0.topic == topic } }

    /// Legacy: nothing verified, nothing requested.
    func expectLegacy(_ label: String) {
        expect(coverage == .unknown && mirror.eventHistory.continuity == .unknown && !mirror.eventHistory.caughtUp,
               "\(label) legacy: coverage and continuity unknown, never caught up (\(coverage))")
        expect(sent.isEmpty && mirror.mapState != .current, "\(label) legacy: no requests, map never current")
    }
}

private func seed1(_ g: UInt64?) -> Seed {
    var s = Seed(g)
    s.status(1, eventsSeq: 100)
    s.event(101, id: 101)
    return s
}

func testLiveRoomRegressionSeeds() {
    for g in [g1, nil] as [UInt64?] {
        let wire = g == nil ? "legacy" : "v2"

        // 1 (review 2): status events_seq 100, live 101 before any replay.
        var s = seed1(g)
        if s.legacy { s.expectLegacy("#1") } else {
            expect(s.coverage == .missing(pending: [0...100], lost: []), "#1 \(wire): 0–100 pending, got \(s.coverage)")
            for seq in UInt64(0)...99 { s.event(seq, id: seq, state: .reverted, sync: true) }
            expect(!s.mirror.eventHistory.isComplete, "#1: not complete before delivery covers 100")
            s.event(100, id: 100, state: .reverted, sync: true)
            expect(s.coverage == .completeThrough(101), "#1: complete only once delivery covers it, got \(s.coverage)")
        }

        // 2 (review 3a): as 1, then disconnect/reconnect and a matching status.
        s = seed1(g)
        s.reconnect()
        s.status(2, eventsSeq: 101)
        if s.legacy { s.expectLegacy("#2") } else {
            expect(s.coverage == .missing(pending: [0...100], lost: []), "#2: still 0–100 after a reconnect, got \(s.coverage)")
        }

        // 3 (review 3b): live 101 + status 101; the replay delivers 0–50, its SyncEnd says 0–101.
        s = Seed(g)
        s.event(101, id: 101)
        s.status(1, eventsSeq: 101)
        if s.legacy { s.expectLegacy("#3") } else {
            guard let request = s.lastRequest(.events), let token = request.token else {
                expect(false, "#3: an events request with a token")
                return
            }
            expect(request.since == nil && request.expectedGeneration == g1, "#3: since NONE in g1")
            for seq in UInt64(0)...50 { s.event(seq, id: seq, state: .reverted, sync: true) }
            s.feed(.syncEnd, LRWire.syncEnd(g1, topic: .events, token: token, bound: 101, delivered: [0...101]))
            expect(s.coverage == .missing(pending: [51...100], lost: []), "#3: 51–100 still pending, got \(s.coverage)")
            expect(s.mirror.retrievalStatus(.events) == .receiving, "#3: the retrieval is not done")
            s.tick(5)
            let retry = s.lastRequest(.events)
            expect(retry?.token != nil && retry?.token != token && retry?.since == .init(epoch: 5, seq: 50),
                   "#3: retry with a new token since (5, 50), got \(String(describing: retry))")
        }

        // 4 (review 4): sync 101 first, reconnect, sync 0–50, status 101.
        s = Seed(g)
        s.event(101, id: 101, sync: true)
        s.reconnect()
        for seq in UInt64(0)...50 { s.event(seq, id: seq, state: .reverted, sync: true) }
        s.status(1, eventsSeq: 101)
        if s.legacy { s.expectLegacy("#4") } else {
            expect(s.coverage == .missing(pending: [51...100], lost: []), "#4: 51–100 pending, got \(s.coverage)")
        }

        // 5 (review 5): sync seq 0 only; two statuses events_seq 2; a request outstanding.
        s = Seed(g)
        s.event(0, sync: true)
        s.status(1, eventsSeq: 2)
        s.status(2, eventsSeq: 2)
        if s.legacy { s.expectLegacy("#5") } else {
            expect(s.coverage == .missing(pending: [1...2], lost: []), "#5: 1–2 pending, got \(s.coverage)")
            expect(s.mirror.retrievalStatus(.events) == .waiting, "#5: a request is out and hides nothing")
        }

        // 6 (review 6): replay 0–2 of one event, then the delayed live 1.
        s = Seed(g)
        s.event(0, id: 7, state: .candidate, sync: true)
        s.event(1, id: 7, state: .confirmed, sync: true)
        s.event(2, id: 7, state: .accepted, sync: true)
        let duplicates = s.mirror.activity.duplicates
        s.event(1, id: 7, state: .confirmed)
        expect(s.mirror.eventsById[7]?.event.state == .accepted, "#6 \(wire): the event stays ACCEPTED")
        expect(s.mirror.activity.duplicates == duplicates + 1 && s.mirror.activity.generationChanges == 0, "#6: a duplicate, no generation change")
        if s.legacy { s.expectLegacy("#6") } else {
            expect(s.coverage == .completeThrough(2), "#6: complete through 2, got \(s.coverage)")
        }

        // 7 (review 7): another Mac's replay gives confirmed@1 without 0; then live candidate@0.
        s = Seed(g)
        s.event(1, id: 7, state: .confirmed, sync: true)
        s.event(0, id: 7, state: .candidate)
        expect(s.mirror.eventsById[7]?.event.state == .confirmed && s.mirror.activity.generationChanges == 0,
               "#7 \(wire): the event stays CONFIRMED, no generation change")
        if s.legacy { s.expectLegacy("#7") } else {
            expect(s.coverage == .completeThrough(1), "#7: seq 0 fills the coverage, got \(s.coverage)")
        }

        // 8 (review 8): sync (event 2, seq 1), then live (event 1, seq 0).
        s = Seed(g)
        s.event(1, id: 2, sync: true)
        s.event(0, id: 1)
        s.status(1, eventsSeq: 1)
        expect(s.mirror.eventsById[1] != nil && s.mirror.eventsById[2] != nil, "#8 \(wire): both events recorded")
        if s.legacy { s.expectLegacy("#8") } else {
            expect(s.coverage == .completeThrough(1) && s.mirror.eventHistory.caughtUp, "#8: complete through 1, got \(s.coverage)")
        }

        // 9 (review 9): events 0–5, disconnect, restart, the new status seq passes the old one, no new events.
        s = Seed(g)
        for seq in UInt64(0)...5 { s.event(seq, id: seq % 3 + 1) }
        s.status(3, eventsSeq: 5)
        s.mirror.connectionClosed(at: s.now)
        s.now += 60
        s.sent += LRWire.requests(s.mirror.connectionOpened(at: s.now, canRequest: true))
        s.status(70, eventsSeq: nil, epoch: 6, generation: .some(g == nil ? nil : g2))
        if s.legacy {
            s.expectLegacy("#9 (restart not detectable)")
            expect(!s.mirror.isMapCurrent, "#9 legacy: map never current")
        } else {
            expect(s.mirror.activeGeneration == g2 && s.coverage == .empty, "#9: the new generation, known empty, got \(s.coverage)")
            expect(s.mirror.eventHistory.continuity == .broken(sinceGeneration: g1), "#9: continuity broken since g1")
            expect(!s.mirror.archivedOpenEvents.isEmpty && s.mirror.openEvents.isEmpty,
                   "#9: the earlier run's open events are archived, none current")
        }

        // 10 (review 10): restart without a reconnect, the startup snapshot missed; the answer's snapshot at seq 0.
        s = Seed(g)
        s.feed(.mapBricks, LRWire.snapshot(g, epoch: 5, seq: 10, [LRWire.brick(0), LRWire.brick(1)]))
        s.status(1, eventsSeq: nil)
        if s.legacy {
            expect(s.mirror.mapState == .stale(.unverified), "#10 legacy: map stale(unverified), got \(s.mirror.mapState)")
            s.expectLegacy("#10")
        } else {
            expect(s.mirror.isMapCurrent, "#10: current before the restart")
            s.status(0, eventsSeq: nil, epoch: 6, generation: .some(g2))
            expect(s.mirror.activeGeneration == g2 && s.mirror.mapState == .stale(.newGeneration), "#10: g2 installed, earlier map greyed")
            guard let request = s.lastRequest(.mapBricks), let token = request.token else {
                expect(false, "#10: a map request in g2")
                return
            }
            expect(request.expectedGeneration == g2 && request.since == nil, "#10: map since NONE in g2")
            s.feed(.syncMapBricks, LRWire.snapshot(g2, epoch: 6, seq: 0, [LRWire.brick(4)]))
            s.feed(.syncEnd, LRWire.syncEnd(g2, topic: .mapBricks, token: token, bound: 0, delivered: [0...0], epoch: 6))
            s.status(1, eventsSeq: nil, epoch: 6, generation: .some(g2))
            expect(s.mirror.isMapCurrent && s.mirror.mapSeq == 0 && s.mirror.bricks.count == 1, "#10: snapshot @0 applies, current after a fresh status")
            expect(s.mirror.retrievalStatus(.mapBricks) == .idle, "#10: the map retrieval completed")
        }

        // A (Astra): the publisher continues while the Mac is away. B: it restarts and passes the old counters.
        for restarted in [false, true] {
            var a = Seed(g)
            for seq in UInt64(0)...3 { a.event(seq, id: 1, state: seq == 0 ? .candidate : .confirmed) }
            a.status(10, eventsSeq: 3)
            a.reconnect()
            let next = restarted ? (g == nil ? nil : g2) : g
            a.status(30, eventsSeq: 8, generation: .some(next))
            a.event(8, id: 9, generation: .some(next))
            if a.legacy {
                // Indistinguishable: both read unknown.
                a.expectLegacy(restarted ? "B" : "A")
                continue
            }
            if restarted {
                expect(a.mirror.activeGeneration == g2 && a.mirror.eventsById[1] == nil, "B: g2, nothing of g1 current")
                expect(a.coverage == .missing(pending: [0...7], lost: []) && a.mirror.archivedOpenEvents.count == 1,
                       "B: g2's 0–7 pending, g1's open event archived, got \(a.coverage)")
            } else {
                for seq in UInt64(4)...7 { a.event(seq, id: 2, state: .reverted, sync: true) }
                expect(a.mirror.activeGeneration == g1 && a.coverage == .completeThrough(8), "A: same generation, the replay fills it")
                expect(a.mirror.eventHistory.continuity == .verified, "A: continuity verified")
            }
        }
    }
}

// MARK: Further fixed traces

func testLiveRoomFixedTraces() {
    // An old-generation snapshot, status and event after the install change nothing but a counter.
    var s = Seed(g2)
    s.status(0, eventsSeq: nil)
    s.feed(.mapBricks, LRWire.snapshot(g2, epoch: 6, seq: 0, [LRWire.brick(1)]))
    s.status(1, eventsSeq: nil, epoch: 6)
    let before = (s.mirror.mapSeq, s.coverage, s.mirror.status?.seq)
    s.feed(.mapBricks, LRWire.snapshot(g1, epoch: 9, seq: 40, [LRWire.brick(7)]))
    s.status(90, eventsSeq: 50, generation: .some(g1))
    s.event(50, generation: .some(g1))
    expect(s.mirror.activity.olderGeneration == 3 && s.mirror.activeGeneration == g2, "older generation counted, ignored")
    expect((s.mirror.mapSeq, s.coverage, s.mirror.status?.seq) == before && s.mirror.isMapCurrent, "older generation changes nothing")

    // A newer generation's event before its status installs it, with H unknown and no request yet.
    s = Seed(g1)
    s.status(0, eventsSeq: nil)
    s.sent = []
    s.event(3, id: 4, generation: .some(g2))
    expect(s.mirror.activeGeneration == g2 && s.coverage == .missing(pending: [0...2], lost: []), "early event installs g2: \(s.coverage)")
    expect(s.mirror.connection == .awaitingStatus(connection: 1) && s.sent.isEmpty, "no request before g2's status")
    s.status(0, eventsSeq: 3, generation: .some(g2))
    expect(s.lastRequest(.events)?.expectedGeneration == g2, "g2's status: the events request goes out")

    // A foreign SyncEnd gives facts only.
    s = Seed(g1)
    s.event(5, id: 1)
    s.status(0, eventsSeq: 5)
    var random = SplitMix64(seed: 5)
    s.feed(.syncEnd, LRWire.syncEnd(g1, topic: .events, token: RM.RequestToken(using: &random), bound: 7, unavailable: [0...2]))
    expect(s.coverage == .missing(pending: [3...4, 6...7], lost: [0...2]), "foreign SyncEnd: bound and unavailable count, got \(s.coverage)")
    expect(s.mirror.retrievalStatus(.events) == .waiting, "foreign SyncEnd: the retrieval is unchanged")

    // A conflicting payload for one seq is counted, never a restart.
    s = Seed(g1)
    s.event(0, id: 1, state: .candidate)
    s.event(0, id: 2, state: .confirmed, sync: true)
    expect(s.mirror.activity.contractViolations == 1 && s.mirror.eventsById[2] == nil && s.mirror.activity.generationChanges == 0,
           "conflicting payload: counted, the first kept, no generation change")

    // A BUILDING SyncEnd (bound NONE) completes the map retrieval, and nothing repeats.
    s = Seed(g1)
    s.status(0, eventsSeq: nil)
    guard let mapRequest = s.lastRequest(.mapBricks), let token = mapRequest.token else {
        expect(false, "BUILDING: a map request")
        return
    }
    s.feed(.syncEnd, LRWire.syncEnd(g1, topic: .mapBricks, token: token, bound: nil))
    expect(s.mirror.retrievalStatus(.mapBricks) == .idle, "BUILDING SyncEnd completes the retrieval")
    s.sent = []
    s.tick(60)
    expect(s.lastRequest(.mapBricks) == nil && s.mirror.mapState == .empty, "no map yet: no repeat, Waiting for map")

    // The "nothing missed" map answer is empty, and completes a D20 retrieval once.
    s = Seed(g1)
    s.feed(.mapBricks, LRWire.snapshot(g1, epoch: 5, seq: 4, [LRWire.brick(1)]))
    s.status(0, eventsSeq: nil, epoch: 5)
    s.status(1, eventsSeq: nil, epoch: 6)
    s.status(2, eventsSeq: nil, epoch: 6)
    expect(s.mirror.mapState == .stale(.epochMismatch), "D20: two mismatching statuses")
    guard let d20 = s.lastRequest(.mapBricks), let d20Token = d20.token else {
        expect(false, "D20: a map request")
        return
    }
    expect(d20.since == .init(epoch: 5, seq: 4), "D20 asks since (5, 4)")
    s.feed(.syncEnd, LRWire.syncEnd(g1, topic: .mapBricks, token: d20Token, bound: 4))
    expect(s.mirror.retrievalStatus(.mapBricks) == .idle, "nothing missed: an empty answer completes")
    s.sent = []
    s.status(3, eventsSeq: nil, epoch: 6)
    s.tick(30)
    expect(s.lastRequest(.mapBricks) == nil, "a completed retrieval does not repeat for the same mismatch")
    s.feed(.mapBricks, LRWire.delta(g1, epoch: 6, seq: 5, base: 5, [LRWire.brick(2)]))
    s.status(4, eventsSeq: nil, epoch: 6)
    expect(s.mirror.isMapCurrent, "the late delta heals it")

    // A legacy-then-v2 wire switch across reconnects.
    s = Seed(nil)
    s.feed(.mapBricks, LRWire.snapshot(nil, epoch: 5, seq: 3, [LRWire.brick(1)]))
    s.status(1, eventsSeq: nil)
    expect(s.mirror.wire == .legacy && s.mirror.mapState == .stale(.unverified), "legacy: unverified")
    s.feed(.status, LRWire.status(g1, seq: 2, epoch: 5, eventsSeq: nil))
    expect(s.mirror.activity.wireMismatch == 1, "a v2 message on a legacy connection is a wire mismatch")
    s.reconnect()
    s.status(2, eventsSeq: nil, generation: .some(g1))
    expect(s.mirror.wire == .v2 && s.mirror.activeGeneration == g1 && s.mirror.mapState == .stale(.newGeneration),
           "reconnect to a v2 node: g1 installed, the legacy map greyed")
    expect(s.mirror.eventHistory.continuity == .broken(sinceGeneration: nil), "continuity broken since the unverified run")
    expect(s.lastRequest(.mapBricks)?.expectedGeneration == g1, "the v2 connection requests the map")
    s.reconnect()
    s.feed(.status, LRWire.status(nil, seq: 9, epoch: 5, eventsSeq: nil))
    s.reconnect()
    s.status(3, eventsSeq: nil, generation: .some(g1))
    expect(s.mirror.activeGeneration == g1 && s.mirror.highestGeneration == g1 && s.coverage == .empty,
           "g1 again after a legacy connection: a fresh, empty state, never the archive")
}

func runLiveRoomTraceTests() {
    testLiveRoomRegressionSeeds()
    print("  LiveRoom seeds: the ten Codex findings and Astra's pair, v2 and legacy wire"); passed()
    testLiveRoomFixedTraces()
    print("  LiveRoom traces: older generation ignored, early install, foreign SyncEnd, conflicting payload, BUILDING and empty map answers, wire switch"); passed()
    testLiveRoomOrderIndependence()
    print("  LiveRoom property: two delivery orders give the same coverage and latest transitions"); passed()
    liveRoomTraceCoverage = TraceCoverage()
    let (traces, seconds) = testLiveRoomSeededTraces()
    let c = liveRoomTraceCoverage
    if traces >= 20 {
        expect(c.generationChanges > 0 && c.requests > 0 && c.lostSeqs > 0 && c.completedRetrievals > 0 && c.mapCurrentChecks > 0,
               "the traces reach restarts, requests, lost seqs, completed retrievals and a current map: \(c)")
    }
    print("  LiveRoom property: \(traces) seeded traces (oracle after every input, idempotence, eviction, quiet-tail liveness) in \(String(format: "%.1f", seconds)) s: \(c.deliveries) deliveries, \(c.requests) requests, \(c.completedRetrievals) completed retrievals, \(c.generationChanges) generation changes, \(c.lostSeqs) lost seqs"); passed()
}
