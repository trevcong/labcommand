import Foundation
import LabCommandCore

// MARK: - LiveRoom: Foxglove client frames, the generation-scoped RoomMemoryMirror, goldens and live captures

private typealias R = RoomMemory
private typealias Mirror = RoomMemoryMirror

private let g1: UInt64 = 1_760_400_000_000
private let g2: UInt64 = 1_760_400_090_000

private func brick(_ i: Int16, occupied: Bool = false) -> R.Brick { LRWire.brick(i, occupied: occupied) }

private func snapshot(epoch: UInt64, seq: UInt64, _ bricks: [R.Brick], room: String = "lab", g: UInt64? = g1) -> Data {
    LRWire.snapshot(g, epoch: epoch, seq: seq, bricks, room: room)
}

private func delta(epoch: UInt64, seq: UInt64, base: UInt64, _ bricks: [R.Brick], removed: [UInt32] = [], g: UInt64? = g1) -> Data {
    LRWire.delta(g, epoch: epoch, seq: seq, base: base, bricks, removed: removed)
}

private func status(epoch: UInt64, seq: UInt64, eventsSeq: UInt64? = nil, room: String = "lab", g: UInt64? = g1) -> Data {
    LRWire.status(g, seq: seq, epoch: epoch, eventsSeq: eventsSeq, room: room)
}

private func event(id: UInt64, seq: UInt64, state: R.ChangeEvent.State = .candidate, epoch: UInt64 = 5, g: UInt64? = g1) -> Data {
    LRWire.event(g, seq: seq, id: id, state: state, epoch: epoch)
}

private func request(_ actions: [Mirror.Action], _ topic: R.RequestSync.Topic, _ context: String) -> R.RequestSync? {
    LRWire.request(actions, topic, context)
}

/// A connected mirror with a current map at (epoch 5, seq 10) holding bricks 0, 1, 2 and a fresh
/// status of g1 that names no event, so neither topic needs a request.
private func currentMirror(canRequest: Bool = true, at now: TimeInterval = 100) -> Mirror {
    var mirror = Mirror(tokenSeed: 3)
    _ = mirror.connectionOpened(at: now, canRequest: canRequest)
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 5, seq: 10, [brick(0), brick(1), brick(2)]), at: now)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 1), at: now)
    expect(mirror.mapState == .current && mirror.isMapCurrent, "setup: snapshot + status makes the map current")
    expect(mirror.retrievalStatus(.mapBricks) == .idle && mirror.retrievalStatus(.events) == .idle, "setup: nothing to request")
    return mirror
}

/// The SyncEnd that answers `request` (its token copied), with the given manifest.
private func answer(_ request: R.RequestSync?, bound: UInt64?, delivered: [ClosedRange<UInt64>] = [],
                    unavailable: [ClosedRange<UInt64>] = [], g: UInt64 = g1) -> Data {
    LRWire.syncEnd(g, topic: request?.topic ?? .events, token: request?.token ?? .zero, bound: bound,
                   delivered: delivered, unavailable: unavailable)
}

// MARK: Foxglove frames

func testLiveRoomFoxgloveClientFrames() {
    let caps = FoxgloveProtocol.serverCapabilities(
        #"{"op":"serverInfo","name":"","capabilities":["connectionGraph"],"supportedEncodings":["cdr","json"]}"#
    )
    expect(caps == ["connectionGraph"], "serverInfo capabilities parse (the live :8767 bridge), got \(String(describing: caps))")
    expect(caps?.contains(FoxgloveProtocol.clientPublishCapability) == false, "live bridge offers no clientPublish")
    expect(FoxgloveProtocol.serverCapabilities(#"{"op":"advertise","channels":[]}"#) == nil, "capabilities only from serverInfo")
    expect(FoxgloveProtocol.serverCapabilities(#"{"op":"serverInfo"}"#) == [], "serverInfo without capabilities is none")

    let advertise = FoxgloveProtocol.clientAdvertiseMessage(
        channelId: 1, topic: "/room_memory/request_sync", schemaName: "labcommand_msgs/msg/RequestSync"
    )
    let object = (try? JSONSerialization.jsonObject(with: Data(advertise.utf8))) as? [String: Any]
    let channel = (object?["channels"] as? [[String: Any]])?.first
    expect(object?["op"] as? String == "advertise", "client advertise op")
    expect((channel?["id"] as? NSNumber)?.intValue == 1, "client advertise channel id")
    expect(channel?["topic"] as? String == "/room_memory/request_sync", "client advertise topic")
    expect(channel?["encoding"] as? String == "cdr", "client advertise encoding")
    expect(channel?["schemaName"] as? String == "labcommand_msgs/msg/RequestSync", "client advertise schemaName")
    expect(FoxgloveProtocol.clientUnadvertiseMessage([1, 2]) == #"{"op":"unadvertise","channelIds":[1,2]}"#, "client unadvertise")

    let frame = FoxgloveProtocol.clientMessageData(channelId: 0x0102_0304, payload: Data([0xAA, 0xBB]))
    expect(frame == Data([0x01, 0x04, 0x03, 0x02, 0x01, 0xAA, 0xBB]), "client message data: 0x01, u32 LE channel, payload")

    func advertised(_ topic: String, _ schema: String, _ encoding: String = "cdr") -> FoxgloveProtocol.Channel {
        FoxgloveProtocol.Channel(id: 9, topic: topic, encoding: encoding, schemaName: schema)
    }
    expect(LiveRoomTopic(channel: advertised("/room_memory/status", "labcommand_msgs/msg/MemoryStatus")) == .status, "status channel")
    expect(LiveRoomTopic(channel: advertised("/room_memory/sync/events", "labcommand_msgs/msg/ChangeEvent")) == .syncEvents, "sync events")
    expect(LiveRoomTopic(channel: advertised("/room_memory/sync/end", "labcommand_msgs/msg/SyncEnd")) == .syncEnd && LiveRoomTopic.syncEnd.isSync,
           "the SyncEnd topic is subscribed and is a sync topic")
    expect(LiveRoomTopic(channel: advertised("/room_memory/sync/events_fallback", "labcommand_msgs/msg/EventsFallback")) == nil,
           "the retired events_fallback topic is not subscribed")
    expect(LiveRoomTopic(channel: advertised("/room_memory/map_bricks", "labcommand_msgs/MapBricks")) == .mapBricks, "ROS 1 style schema")
    expect(LiveRoomTopic(channel: advertised("/room_memory/status", "labcommand_msgs/msg/ChangeEvent")) == nil, "wrong schema refused")
    expect(LiveRoomTopic(channel: advertised("/room_memory/status", "labcommand_msgs/msg/MemoryStatus", "json")) == nil, "json refused")
    expect(LiveRoomTopic(channel: advertised("/room_memory/request_sync", "labcommand_msgs/msg/RequestSync")) == nil,
           "the server's request_sync channel is not subscribed")
    expect(LiveRoomTopic(channel: advertised("/foxglove_bridge/sysinfo", "foxglove.SystemInfo")) == nil, "sysinfo ignored")

    let time = R.Time(date: Date(timeIntervalSince1970: 1_760_400_010.25))
    expect(time.sec == 1_760_400_010 && abs(Int(time.nanosec) - 250_000_000) < 1000, "Time(date:) splits seconds and nanoseconds")
}

// MARK: Goldens and live captures

private func hexFixture(_ name: String) -> Data {
    let path = "Tests/Fixtures/room/golden/hand/\(name).hex"
    guard let text = try? String(contentsOfFile: path, encoding: .utf8) else {
        expect(false, "\(path) not found — run tests from repo root")
        return Data()
    }
    var bytes: [UInt8] = []
    for line in text.split(separator: "\n") {
        for token in line.prefix(while: { $0 != "#" }).split(whereSeparator: { $0 == " " || $0 == "\t" }) {
            guard let byte = UInt8(token, radix: 16) else {
                expect(false, "\(path): bad hex byte '\(token)'")
                return Data()
            }
            bytes.append(byte)
        }
    }
    return Data(bytes)
}

/// Wraps a payload in the server's message-data frame and parses it back, as the client does.
private func throughFoxglove(_ payload: Data, subscription: UInt32 = 3) -> Data {
    var frame = Data([FoxgloveProtocol.messageDataOpcode])
    withUnsafeBytes(of: subscription.littleEndian) { frame.append(contentsOf: $0) }
    withUnsafeBytes(of: UInt64(42).littleEndian) { frame.append(contentsOf: $0) }
    frame.append(payload)
    let parsed = FoxgloveProtocol.parseMessageData(frame)
    expect(parsed?.subscriptionId == subscription, "message data subscription id")
    return parsed?.payload ?? Data()
}

func testLiveRoomGoldensDecode() {
    let goldens: [(String, LiveRoomTopic?)] = [
        ("map_bricks_snapshot_zlib_le", .mapBricks),
        ("map_bricks_delta_le", .mapBricks),
        ("map_bricks_delta_be", .mapBricks),
        ("map_bricks_delta_lzfse_le", .mapBricks),
        ("change_event_appeared_le", .events),
        ("change_event_moved_mask_le", .events),
        ("memory_status_le", .status),
        ("epoch_le", .epoch),
        ("epoch_be", .epoch),
        ("epoch_v2_le", .epoch),
        ("epoch_v2_be", .epoch),
        ("activity_le", .activity),
        ("activity_idle_le", .activity),
        ("activity_v2_le", .activity),
        ("request_sync_map_le", nil),
        ("request_sync_map_be", nil),
        ("request_sync_events_none_le", nil),
        ("map_bricks_delta_v2_le", .mapBricks),
        ("map_bricks_delta_v2_be", .mapBricks),
        ("change_event_appeared_v2_le", .events),
        ("change_event_moved_mask_v2_le", .events),
        ("memory_status_v2_le", .status),
        ("request_sync_events_v2_le", nil),
        ("request_sync_map_v2_be", nil),
        ("sync_end_events_empty_le", .syncEnd),
        ("sync_end_events_fallback_le", .syncEnd),
        ("sync_end_map_replay_be", .syncEnd),
    ]
    for (name, topic) in goldens {
        let payload = throughFoxglove(hexFixture(name))
        let v2 = name.contains("_v2_") || name.hasPrefix("sync_end")
        do {
            switch name {
            case let n where n.hasPrefix("map_bricks"):
                expect((try R.MapBricks.decode(cdr: payload).publisherGeneration != nil) == v2, "\(name): layout")
            case let n where n.hasPrefix("change_event"):
                expect((try R.ChangeEvent.decode(cdr: payload).publisherGeneration != nil) == v2, "\(name): layout")
            case let n where n.hasPrefix("memory_status"):
                let decoded = try R.MemoryStatus.decode(cdr: payload)
                expect(decoded.eventsSeq == 12 && (decoded.publisherGeneration == g1) == v2, "\(name): events_seq 12, generation")
            case let n where n.hasPrefix("epoch"):
                expect((try R.Epoch.decode(cdr: payload).publisherGeneration == g1) == v2, "\(name): generation")
            case let n where n.hasPrefix("activity"):
                expect((try R.Activity.decode(cdr: payload).publisherGeneration == g1) == v2, "\(name): generation")
            case let n where n.hasPrefix("sync_end"):
                expect(try R.SyncEnd.decode(cdr: payload).publisherGeneration == g1, "\(name): generation")
            default:
                expect((try R.RequestSync.decode(cdr: payload).token != nil) == v2, "\(name): layout")
            }
        } catch {
            expect(false, "golden \(name) decodes after Foxglove framing: \(error)")
        }
        // Every golden for a subscribed topic is accepted by the mirror: never counted rejected.
        if let topic {
            var mirror = Mirror(tokenSeed: 1)
            _ = mirror.connectionOpened(at: 0, canRequest: false)
            if topic == .activity || topic == .epoch {
                // These never fix the wire: a status of the same layout comes first.
                _ = mirror.receive(topic: .status, payload: status(epoch: 1, seq: 0, g: v2 ? g1 : nil), at: 0)
            }
            _ = mirror.receive(topic: topic, payload: payload, at: 0)
            expect(mirror.activity.rejected == 0 && mirror.activity.sideTopicLayout == 0, "mirror accepts golden \(name)")
            if topic == .activity { expect(mirror.roomActivity != nil && mirror.isActivityFresh, "golden \(name) is the room activity") }
            if topic == .epoch { expect(mirror.epochTick != nil && mirror.isEpochFresh, "golden \(name) is the epoch tick") }
            expect(mirror.wire == (v2 ? .v2 : .legacy), "golden \(name) fixes the wire as \(v2 ? "v2" : "legacy")")
            expect(mirror.activity.topics[topic.rawValue]?.count == 1, "activity counts golden \(name) on \(topic.rawValue)")
        }
    }

    // A delta golden alone is not a map.
    var mirror = Mirror(tokenSeed: 1)
    _ = mirror.connectionOpened(at: 0, canRequest: true)
    _ = mirror.receive(topic: .mapBricks, payload: hexFixture("map_bricks_delta_v2_le"), at: 0)
    expect(mirror.mapState == .empty && mirror.bricks.isEmpty && mirror.activeGeneration == g1, "a v2 delta with no chain installs g1, applies nothing")

    // The legacy snapshot golden shows its bricks, unverified; its v2 twin with a status is current.
    let snap = hexFixture("map_bricks_snapshot_zlib_le")
    mirror = Mirror(tokenSeed: 1)
    _ = mirror.connectionOpened(at: 0, canRequest: true)
    let legacyActions = mirror.receive(topic: .mapBricks, payload: snap, at: 0)
    let decoded = try? R.MapBricks.decode(cdr: snap)
    expect(mirror.mapState == .stale(.unverified) && legacyActions.isEmpty, "legacy snapshot: unverified, no requests")
    expect(mirror.bricks.count == decoded?.bricks.count && mirror.mapEpoch == decoded?.header.mapEpoch, "legacy snapshot bricks + epoch shown")
    expect(mirror.roomId == decoded?.header.roomId, "room adopted from the first message")
    var twin = decoded
    twin?.publisherGeneration = g1
    mirror = Mirror(tokenSeed: 1)
    _ = mirror.connectionOpened(at: 0, canRequest: false)
    if let bytes = try? twin?.encodedCDR() { _ = mirror.receive(topic: .mapBricks, payload: bytes, at: 0) }
    _ = mirror.receive(topic: .status, payload: status(epoch: 43, seq: 1), at: 0)
    expect(mirror.isMapCurrent, "the v2 twin of the snapshot golden with a g1 status is current")
}

/// Real frames from the Spark bridge at :8767 (read-only capture, 2026-10-15). They predate
/// publisher generations, so they stay legacy decode fixtures: the mirror runs in legacy mode.
func testLiveRoomLiveCaptures() {
    func load(_ name: String) -> Data? {
        try? Data(contentsOf: URL(fileURLWithPath: "Tests/Fixtures/room/live/\(name).bin"))
    }
    guard let statusBytes = load("status"), let snapBytes = load("map_snapshot") else {
        expect(false, "Tests/Fixtures/room/live/{status,map_snapshot}.bin present")
        return
    }
    do {
        let live = try R.MemoryStatus.decode(cdr: statusBytes)
        expect(live.header.roomId == "lab" && live.publisherGeneration == nil, "live status: room lab, legacy layout")
        expect(live.eventsSeq == nil, "status after a node restart: events_seq is EVENTS_SEQ_NONE (nil)")
        expect(live.coverageKnownFraction.map { (0...1).contains($0) } ?? false, "live coverage is a fraction")
        if let busy = load("status_events_seq") {
            let decoded = try R.MemoryStatus.decode(cdr: busy)
            expect(decoded.eventsSeq == 6790 && decoded.state == .tracking && decoded.brickCount == 1977,
                   "live status with events_seq 6790, TRACKING, 1977 bricks")
        } else {
            expect(false, "Tests/Fixtures/room/live/status_events_seq.bin present")
        }
        if let one = load("map_delta_1brick") {
            let d = try R.MapBricks.decode(cdr: one)
            expect(d.kind == .delta && d.baseEpoch == 2399 && d.header.mapEpoch == 2400 && d.bricks.count == 1,
                   "live 1-brick zlib delta 2399 → 2400 decodes")
        } else {
            expect(false, "Tests/Fixtures/room/live/map_delta_1brick.bin present")
        }
        let snap = try R.MapBricks.decode(cdr: snapBytes)
        expect(snap.kind == .snapshot && !snap.bricks.isEmpty && snap.compression == .zlib, "live snapshot: zlib, bricks")

        var mirror = Mirror(tokenSeed: 1)
        let opened = mirror.connectionOpened(at: 0, canRequest: true)
        var sent = opened.count
        sent += mirror.receive(topic: .status, payload: statusBytes, at: 0).count
        sent += mirror.receive(topic: .mapBricks, payload: snapBytes, at: 0.5).count
        expect(mirror.activity.rejected == 0 && mirror.wire == .legacy, "live captures are accepted as legacy")
        expect(mirror.bricks.count == snap.bricks.count && mirror.mapState == .stale(.unverified),
               "live snapshot: \(snap.bricks.count) bricks shown, unverified")
        expect(mirror.status?.eventsSeq == live.eventsSeq, "mirror status exposes events_seq")

        if let deltaBytes = load("map_delta") {
            let d = try R.MapBricks.decode(cdr: deltaBytes)
            expect(d.kind == .delta && d.baseEpoch == snap.header.mapEpoch && d.header.seq == snap.header.seq + 1,
                   "captured delta chains to the captured snapshot")
            sent += mirror.receive(topic: .mapBricks, payload: deltaBytes, at: 1).count
            expect(mirror.mapEpoch == d.header.mapEpoch && mirror.mapSeq == d.header.seq, "live delta applies on its connection")
        }
        if let eventBytes = load("events") {
            let e = try R.ChangeEvent.decode(cdr: eventBytes)
            sent += mirror.receive(topic: .events, payload: eventBytes, at: 2).count
            expect(mirror.eventsById[e.eventId]?.event == e, "live event shown")
        }
        expect(sent == 0 && mirror.eventHistory.coverage == .unknown && !mirror.isMapCurrent,
               "legacy mode: no requests, history unknown, map never current")
    } catch {
        expect(false, "live captures decode: \(error)")
    }
}

// MARK: Map

func testLiveRoomSnapshotDeltaApply() {
    var mirror = currentMirror()
    let revision = mirror.mapRevision
    expect(mirror.bricks.count == 3 && mirror.mapEpoch == 5 && mirror.mapSeq == 10, "snapshot sets bricks, epoch, seq")

    // Remove index 1 of the base list (brick 1), replace brick 2, insert brick 7.
    var actions = mirror.receive(
        topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, [brick(2, occupied: true), brick(7)], removed: [1]), at: 101
    )
    expect(actions.isEmpty, "a chaining delta needs no request")
    _ = mirror.receive(topic: .status, payload: status(epoch: 6, seq: 2), at: 101)
    expect(mirror.mapState == .current && mirror.mapEpoch == 6 && mirror.mapSeq == 11, "delta advances epoch and seq")
    expect(Set(mirror.bricks.keys.map(\.i)) == [0, 2, 7], "delta removed brick 1 and inserted brick 7")
    expect(mirror.bricks[R.BrickCoordinate(i: 2, j: 0, k: 0)]?.evidence(x: 0, y: 0, z: 0) == .occupied, "delta replaced brick 2")
    expect(mirror.mapRevision == revision + 1, "map revision bumps once per change")

    // Removed indices refer to the canonical (sorted) order of the base epoch: bricks 0, 2, 7.
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, [], removed: [0, 2]), at: 102)
    expect(Set(mirror.bricks.keys.map(\.i)) == [2], "removed [0, 2] of (0, 2, 7) leaves brick 2")

    // A duplicate (seq not newer) is ignored and counted.
    let before = mirror
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 103)
    expect(mirror.bricks == before.bricks && mirror.activity.duplicates == 1, "duplicate delta changes nothing, counted")

    // An invalid delta (index out of range) is rejected whole: map stale, bricks untouched, resync requested.
    actions = mirror.receive(topic: .mapBricks, payload: delta(epoch: 8, seq: 13, base: 7, [brick(9)], removed: [5]), at: 110)
    expect(mirror.mapState == .stale(.rejected), "out-of-range removal: stale(rejected), got \(mirror.mapState)")
    expect(mirror.bricks == before.bricks && mirror.mapEpoch == 7, "rejected delta is not partly applied")
    let resync = request(actions, .mapBricks, "rejected delta")
    expect(resync?.since == .init(epoch: 7, seq: 12) && resync?.expectedGeneration == g1, "rejected delta requests since (7, 12) in g1")

    // Removing and re-sending the same brick is invalid too.
    var other = currentMirror()
    _ = other.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, [brick(1)], removed: [1]), at: 101)
    expect(other.mapState == .stale(.rejected) && other.bricks.count == 3, "remove + re-send same brick rejected whole")

    // In one generation a delta first after a reconnect chains: seqs are contiguous per process.
    var rejoin = currentMirror()
    rejoin.connectionClosed(at: 101)
    expect(rejoin.mapState == .stale(.disconnected), "closed: stale(disconnected)")
    _ = rejoin.connectionOpened(at: 102, canRequest: true)
    _ = rejoin.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, [brick(4)]), at: 102)
    expect(rejoin.mapState == .stale(.awaitingStatus), "reconnected: awaiting a status of g1")
    _ = rejoin.receive(topic: .status, payload: status(epoch: 6, seq: 2), at: 102.5)
    expect(rejoin.isMapCurrent && rejoin.bricks.count == 4, "the chaining delta and g1's status make it current")

    // A replay (any Mac's) that chains heals a gap once it reaches the newest seq.
    other = currentMirror()
    _ = other.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 101)
    expect(other.mapState == .stale(.seqGap) && other.activity.gaps == 1, "seq 12 after 10 is a gap")
    _ = other.receive(topic: .syncMapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 102)
    _ = other.receive(topic: .syncMapBricks, payload: delta(epoch: 7, seq: 12, base: 6, [brick(4)]), at: 102.5)
    _ = other.receive(topic: .status, payload: status(epoch: 7, seq: 2), at: 103)
    expect(other.mapState == .current && other.mapEpoch == 7 && other.bricks.count == 4, "replay chained to the newest seq: current")
    // An older answer (another Mac's) never rolls the chain back.
    _ = other.receive(topic: .syncMapBricks, payload: snapshot(epoch: 5, seq: 10, [brick(0)]), at: 104)
    expect(other.mapSeq == 12 && other.bricks.count == 4, "an older snapshot is ignored")
}

func testLiveRoomGapRequestsSync() {
    var mirror = currentMirror()
    let actions = mirror.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 101)
    expect(mirror.mapState == .stale(.seqGap) && !mirror.isMapCurrent && mirror.bricks.count == 3, "seq gap: stale, last good bricks kept")
    let sent = request(actions, .mapBricks, "seq gap")
    expect(sent?.since == .init(epoch: 5, seq: 10) && sent?.header.mapEpoch == 5 && sent?.header.roomId == "lab",
           "gap request: since (mirror epoch, last seq), header repeats since_epoch and the room")
    expect(sent?.expectedGeneration == g1 && sent?.token.map { !$0.isZero } == true, "gap request carries g1 and a token")
    expect(mirror.retrievalStatus(.mapBricks) == .waiting && mirror.activity.requestsSent == 1, "waiting for the answer")

    // The answer's messages, then its SyncEnd: done once what it delivered is applied.
    _ = mirror.receive(topic: .syncEnd, payload: answer(sent, bound: 12, delivered: [11...12]), at: 101.5)
    expect(mirror.retrievalStatus(.mapBricks) == .receiving, "the SyncEnd arrived first: receiving")
    _ = mirror.receive(topic: .syncMapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 101.6)
    expect(mirror.retrievalStatus(.mapBricks) == .receiving, "11 of 11–12: still receiving")
    _ = mirror.receive(topic: .syncMapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 101.7)
    _ = mirror.receive(topic: .status, payload: status(epoch: 7, seq: 2), at: 101.8)
    expect(mirror.retrievalStatus(.mapBricks) == .idle && mirror.isMapCurrent, "delivered 11–12 applied: done, current")

    // Wrong base.
    mirror = currentMirror()
    let wrongBase = mirror.receive(topic: .mapBricks, payload: delta(epoch: 9, seq: 11, base: 8, []), at: 101)
    expect(mirror.mapState == .stale(.wrongBase), "base_epoch ≠ mirror epoch: stale(wrongBase)")
    expect(request(wrongBase, .mapBricks, "wrong base")?.since == .init(epoch: 5, seq: 10), "wrong base requests since (5, 10)")

    // Timeouts: 5, 10, 20, then 30 s, a new token each time; the retry asks for a snapshot.
    var retry = currentMirror(at: 0)
    let first = request(retry.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 10), .mapBricks, "first")
    expect(first != nil, "first request goes out at once")
    expect(request(retry.tick(at: 14), .mapBricks, "4 s") == nil, "no repeat before 5 s")
    let second = request(retry.tick(at: 15), .mapBricks, "5 s")
    expect(second != nil && second!.header.seq > first!.header.seq && second!.token != first!.token && second!.since == nil,
           "repeat after 5 s: request seq counts up, a new token, since NONE")
    expect(request(retry.tick(at: 24), .mapBricks, "9 s later") == nil, "second repeat waits 10 s")
    let third = request(retry.tick(at: 25), .mapBricks, "10 s later")
    expect(third != nil, "second repeat at 10 s")
    expect(request(retry.tick(at: 44), .mapBricks, "19 s") == nil && request(retry.tick(at: 45), .mapBricks, "20 s") != nil,
           "third repeat at 20 s")
    // A late SyncEnd of a dead token is a publisher fact only.
    _ = retry.receive(topic: .syncEnd, payload: answer(first, bound: 12, delivered: [12...12]), at: 46)
    expect(retry.retrievalStatus(.mapBricks) == .waiting, "an old token does not answer the outstanding request")
    let latest = retry.activity.lastRequest
    _ = retry.receive(topic: .syncMapBricks, payload: snapshot(epoch: 7, seq: 12, [brick(3)]), at: 46.5)
    _ = retry.receive(topic: .syncEnd, payload: answer(latest, bound: 12, delivered: [12...12]), at: 46.6)
    _ = retry.receive(topic: .status, payload: status(epoch: 7, seq: 3), at: 46.7)
    expect(retry.mapState == .current && retry.retrievalStatus(.mapBricks) == .idle, "the outstanding token's answer completes it")
    expect(request(retry.tick(at: 200), .mapBricks, "resolved") == nil, "resolved: no more map repeats")

    // No request path: no actions, the map stays stale until a live snapshot.
    var noPath = currentMirror(canRequest: false)
    expect(noPath.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 101).isEmpty, "no request path: nothing to send")
    expect(noPath.tick(at: 200).isEmpty && noPath.mapState == .stale(.seqGap), "no path: stays stale, no repeats")
    _ = noPath.receive(topic: .mapBricks, payload: snapshot(epoch: 8, seq: 14, [brick(1)]), at: 201)
    _ = noPath.receive(topic: .status, payload: status(epoch: 8, seq: 2), at: 201)
    expect(noPath.mapState == .current && noPath.bricks.count == 1, "a live snapshot recovers without a request path")

    // Unsent: the retrieval stays waiting and follows the backoff.
    var unsent = currentMirror(at: 0)
    _ = unsent.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 1)
    unsent.noteRequestUnsent()
    expect(unsent.activity.requestsUnsent == 1 && unsent.activity.requestsSent == 0 && unsent.retrievalStatus(.mapBricks) == .waiting,
           "an unsent request is counted; the retrieval waits for its timeout")
    expect(request(unsent.tick(at: 6), .mapBricks, "after unsent") != nil, "the next attempt follows the 5 s timeout")
}

func testLiveRoomEpochChange() {
    // D20: the status epoch disagreeing twice in a row makes the map stale and requests a resync.
    var mirror = currentMirror()
    var actions = mirror.receive(topic: .status, payload: status(epoch: 6, seq: 2), at: 101)
    expect(mirror.mapState == .current && request(actions, .mapBricks, "one mismatch") == nil, "one mismatching status is lag")
    actions = mirror.receive(topic: .status, payload: status(epoch: 6, seq: 3), at: 102)
    expect(mirror.mapState == .stale(.epochMismatch), "two mismatching statuses: stale(epochMismatch)")
    expect(request(actions, .mapBricks, "epoch change")?.since == .init(epoch: 5, seq: 10), "epoch change requests since (5, 10)")
    // The lost delta arrives late (seq 11 chains): current again.
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 103)
    expect(mirror.mapState == .current, "chained delta heals an epoch mismatch")

    // Status and delta in either order never trip D20.
    for statusFirst in [true, false] {
        var m = currentMirror()
        var t: TimeInterval = 101
        for step in 0..<6 {
            let epoch = UInt64(6 + step)
            let d = delta(epoch: epoch, seq: UInt64(11 + step), base: epoch - 1, [])
            let s = status(epoch: epoch, seq: UInt64(2 + step))
            if statusFirst {
                _ = m.receive(topic: .status, payload: s, at: t)
                _ = m.receive(topic: .mapBricks, payload: d, at: t + 0.5)
            } else {
                _ = m.receive(topic: .mapBricks, payload: d, at: t)
                _ = m.receive(topic: .status, payload: s, at: t + 0.5)
            }
            t += 1
        }
        expect(m.mapState == .current && m.mapEpoch == 11, "deltas racing statuses (statusFirst \(statusFirst)) stay current")
    }

    // A restart is a new generation, not a lower seq: its snapshot installs over the old chain.
    mirror = currentMirror()
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 6, seq: 0, [brick(4)], g: g2), at: 102)
    expect(mirror.activeGeneration == g2 && mirror.mapEpoch == 6 && mirror.mapSeq == 0 && mirror.mapState == .stale(.awaitingStatus),
           "g2's snapshot installs g2 with its own chain")
    _ = mirror.receive(topic: .status, payload: status(epoch: 6, seq: 0, g: g2), at: 102.5)
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 1, base: 6, [brick(5)], g: g2), at: 103)
    expect(mirror.isMapCurrent && mirror.bricks.count == 2 && mirror.mapEpoch == 7, "deltas chain in the new generation")
    // A lower seq of the same generation is never a restart.
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 1, seq: 0, [brick(9)], g: g2), at: 104)
    expect(mirror.mapSeq == 1 && mirror.activity.generationChanges == 1, "a lower-seq snapshot of g2 is an older answer")

    // Another room is ignored, never mixed in.
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 50, seq: 50, [brick(9)], room: "garage"), at: 104)
    _ = mirror.receive(topic: .status, payload: status(epoch: 50, seq: 9, room: "garage"), at: 104)
    expect(mirror.mapEpoch == 7 && mirror.roomId == "lab" && mirror.activity.foreignRoom == 2, "foreign room ignored and counted")
    var pinned = Mirror(roomId: "garage", tokenSeed: 1)
    _ = pinned.connectionOpened(at: 0, canRequest: false)
    _ = pinned.receive(topic: .mapBricks, payload: snapshot(epoch: 5, seq: 1, [brick(1)]), at: 0)
    expect(pinned.mapState == .empty, "a pinned room ignores other rooms")
}

func testLiveRoomReconnect() {
    var mirror = currentMirror()
    mirror.connectionClosed(at: 101)
    expect(mirror.mapState == .stale(.disconnected) && !mirror.isStatusFresh && mirror.connection == .offline,
           "socket close: map stale, status not fresh")
    expect(mirror.bricks.count == 3, "socket close keeps the last good bricks")

    // Reconnect: await a status of the generation, then request with its expected_generation and since.
    let opened = mirror.connectionOpened(at: 110, canRequest: true)
    expect(opened.isEmpty && mirror.connection == .awaitingStatus(connection: 2), "reconnect: nothing before a status")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 1), at: 110.2)
    expect(mirror.connection == .awaitingStatus(connection: 2), "a delayed copy (seq ≤ the held one) is no status of this connection")
    let observed = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 3), at: 110.5)
    expect(mirror.connection == .observing(connection: 2, generation: g1), "a new status: observing g1")
    let events = request(observed, .events, "reconnect")
    expect(events?.since == nil && events?.expectedGeneration == g1, "events 0–3 named, none held: since NONE, expected g1")
    expect(mirror.isMapCurrent, "a status of g1 on this connection: the chain is current again")

    // Status freshness: 3 s.
    _ = mirror.tick(at: 110.5 + 2.9)
    expect(mirror.isStatusFresh, "status fresh under 3 s")
    _ = mirror.tick(at: 110.5 + 3.1)
    expect(!mirror.isStatusFresh && !mirror.isMapCurrent, "status stale after 3 s: map not shown as current")

    // A fresh connection with no mirror: the room id waits for the first message.
    var fresh = Mirror(tokenSeed: 1)
    _ = fresh.connectionOpened(at: 0, canRequest: true)
    expect(fresh.tick(at: 1).isEmpty, "requests wait for a status")
    _ = fresh.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 2)
    let first = fresh.receive(topic: .status, payload: status(epoch: 6, seq: 0), at: 2.5)
    expect(fresh.mapState == .empty && request(first, .mapBricks, "no mirror")?.since == nil, "no chain: request since NONE")
}

// MARK: Events

func testLiveRoomEvents() {
    var mirror = currentMirror()
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 0), at: 101)
    expect(mirror.eventHistory.coverage == .completeThrough(0), "seq 0 received: complete through 0")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 1, state: .confirmed), at: 102)
    expect(mirror.eventsById[1]?.event.state == .confirmed && mirror.openEvents.count == 1, "transition applies by event_id")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 1, state: .confirmed), at: 103)
    expect(mirror.activity.duplicates == 1 && mirror.events.count == 2, "duplicate event counted, not stored")

    // A gap: 4 after 1. The newer message shows; 2–3 are pending and requested since (5, 1).
    let actions = mirror.receive(topic: .events, payload: event(id: 2, seq: 4), at: 120)
    expect(mirror.eventHistory.coverage == .missing(pending: [2...3], lost: []), "gap: 2–3 pending, got \(mirror.eventHistory.coverage)")
    expect(mirror.eventsById[2] != nil && mirror.eventHistory.contiguousThrough == 1, "the newer message shows; the run from 0 ends at 1")
    let sent = request(actions, .events, "events gap")
    expect(sent?.since == .init(epoch: 5, seq: 1) && sent?.expectedGeneration == g1, "events gap requests since (5, 1) in g1")

    // The answer: its SyncEnd, then 2 and 3 on the sync topic; done once 2–3 are held.
    _ = mirror.receive(topic: .syncEnd, payload: answer(sent, bound: 4, delivered: [2...4]), at: 121)
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 1, seq: 2, state: .accepted), at: 121)
    expect(mirror.retrievalStatus(.events) == .receiving, "2 of 2–4: receiving")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 3, seq: 3), at: 121)
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 2, seq: 4), at: 121)
    expect(mirror.eventHistory.coverage == .completeThrough(4) && mirror.retrievalStatus(.events) == .idle, "replay covers 2–4: complete, done")
    expect(mirror.eventsById[1]?.event.state == .accepted && mirror.openEvents.map(\.event.eventId) == [2, 3], "replayed states apply")
    expect(mirror.events.suffix(2).allSatisfy(\.fromSync) && mirror.events.first?.fromSync == false, "fromSync is kept as diagnostics; the sync copy of 4 was a duplicate")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 4), at: 122)
    expect(mirror.eventHistory.caughtUp, "a fresh status naming 4: caught up")
    expect(request(mirror.tick(at: 200), .events, "after replay") == nil, "nothing pending: no repeat")

    // An event's state only moves on a higher seq, whatever the order or transport.
    _ = mirror.receive(topic: .events, payload: event(id: 3, seq: 5, state: .confirmed), at: 201)
    _ = mirror.receive(topic: .events, payload: event(id: 3, seq: 7, state: .expired), at: 202)
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 3, seq: 6, state: .reverted), at: 203)
    expect(mirror.eventsById[3]?.event.state == .expired && mirror.eventHistory.coverage == .completeThrough(7),
           "a lower-seq transition does not overwrite a higher one")

    // A low live seq is a duplicate or a conflict, never a restart.
    _ = mirror.receive(topic: .events, payload: event(id: 3, seq: 0, state: .candidate), at: 210)
    expect(mirror.eventsById[3]?.event.state == .expired && mirror.activity.contractViolations == 1 && mirror.activity.generationChanges == 0,
           "seq 0 again with another payload: a contract violation, no rollback, no generation change")

    // The ring is bounded.
    var ring = currentMirror(canRequest: false)
    for seq in 0..<UInt64(Mirror.eventCapacity + 50) {
        _ = ring.receive(topic: .events, payload: event(id: seq % 7, seq: seq), at: 100)
    }
    expect(ring.events.count == Mirror.eventCapacity && ring.events.first?.seq == 50, "event ring holds the newest \(Mirror.eventCapacity)")
    expect(ring.eventHistory.coverage == .completeThrough(UInt64(Mirror.eventCapacity + 49)), "the coverage is not the ring: still complete")

    // No request path: a gap stays pending; nothing completes it.
    var noPath = currentMirror(canRequest: false)
    _ = noPath.receive(topic: .events, payload: event(id: 1, seq: 1), at: 101)
    expect(noPath.receive(topic: .events, payload: event(id: 1, seq: 5), at: 102).isEmpty, "no path: no request")
    _ = noPath.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 5), at: 103)
    expect(noPath.eventHistory.coverage == .missing(pending: [0...0, 2...4], lost: []) && !noPath.eventHistory.caughtUp,
           "missing seqs stay pending for good, got \(noPath.eventHistory.coverage)")
}

func testLiveRoomEventsSeqGap() {
    var mirror = currentMirror()
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 0), at: 100)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 0), at: 101)
    expect(mirror.eventHistory.caughtUp, "status at the newest seq: caught up")

    // A status naming 1 before event 1 arrives: 1 is pending at once (no lag tolerance needed:
    // a request is cheap and the seq will be in its answer or arrive live).
    let actions = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 1), at: 102)
    expect(mirror.eventHistory.coverage == .missing(pending: [1...1], lost: []), "status ahead: 1 pending")
    let sent = request(actions, .events, "status ahead")
    expect(sent?.since == .init(epoch: 5, seq: 0), "requests since (5, 0)")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 1), at: 102.3)
    expect(mirror.eventHistory.coverage == .completeThrough(1), "the live event fills it")
    // The outstanding retrieval completes only by its manifest, or times out with nothing needed.
    expect(mirror.retrievalStatus(.events) == .waiting, "still waiting for the SyncEnd")
    expect(request(mirror.tick(at: 108), .events, "timeout, nothing pending") == nil && mirror.retrievalStatus(.events) == .idle,
           "timed out with nothing pending: idle, no resend")

    // A duplicated status copy does not count as a second status.
    var dup = currentMirror()
    _ = dup.receive(topic: .events, payload: event(id: 1, seq: 0), at: 100)
    let ahead = status(epoch: 5, seq: 9, eventsSeq: 2)
    _ = dup.receive(topic: .status, payload: ahead, at: 101)
    _ = dup.receive(topic: .status, payload: ahead, at: 101.1)
    expect(dup.activity.duplicates == 1 && dup.status?.receivedAt == 101, "status copy is a duplicate; the first receipt counts")
}

/// The answers of protocol §5: fallbacks, unavailable seqs, empty answers.
func testLiveRoomReplayAndFallbackEdges() {
    // A replayed snapshot behind the newest seq is not current until the replay's deltas catch up.
    var map = currentMirror()
    _ = map.receive(topic: .status, payload: status(epoch: 8, seq: 2), at: 110)
    _ = map.receive(topic: .mapBricks, payload: delta(epoch: 8, seq: 13, base: 7, []), at: 110)
    expect(map.mapState == .stale(.seqGap), "live seq 13 after 10: stale")
    _ = map.receive(topic: .syncMapBricks, payload: snapshot(epoch: 6, seq: 11, [brick(1)]), at: 111)
    expect(map.mapState != .current, "replayed snapshot at seq 11 < 13 is not current")
    _ = map.receive(topic: .syncMapBricks, payload: delta(epoch: 7, seq: 12, base: 6, [brick(2)]), at: 111)
    _ = map.receive(topic: .syncMapBricks, payload: delta(epoch: 8, seq: 13, base: 7, [brick(3)]), at: 111)
    _ = map.receive(topic: .status, payload: status(epoch: 8, seq: 3), at: 111)
    expect(map.mapState == .current && map.mapSeq == 13 && map.bricks.count == 3, "the replay's deltas chain to 13: current")

    // An events fallback: the log no longer holds 1–19, an open event's latest (seq 10) is resent.
    var cold = Mirror(roomId: "lab", tokenSeed: 2)
    _ = cold.connectionOpened(at: 0, canRequest: true)
    let asked = request(cold.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 30), at: 0), .events, "cold")
    expect(asked?.since == nil, "nothing held: since NONE")
    _ = cold.receive(topic: .syncEvents, payload: event(id: 10, seq: 10, state: .confirmed), at: 1)
    for seq in UInt64(20)...30 { _ = cold.receive(topic: .syncEvents, payload: event(id: 100 + seq, seq: seq, state: .reverted), at: 1) }
    _ = cold.receive(topic: .syncEnd, payload: answer(asked, bound: 30, delivered: [10...10, 20...30], unavailable: [0...9, 11...19]), at: 1.1)
    expect(cold.eventHistory.coverage == .missing(pending: [], lost: [0...9, 11...19]), "fallback: lost exactly what the publisher declared")
    expect(cold.retrievalStatus(.events) == .idle && request(cold.tick(at: 60), .events, "after fallback") == nil,
           "the fallback completes: no endless retries")
    expect(cold.eventsById[10]?.event.state == .confirmed, "the old open event from the fallback is kept")
    _ = cold.receive(topic: .events, payload: event(id: 31, seq: 31, state: .confirmed), at: 61)
    expect(cold.eventHistory.coverage == .missing(pending: [], lost: [0...9, 11...19]) && !cold.eventHistory.caughtUp,
           "live events chain on; the lost seqs stay lost")

    // A lost seq is publisher-declared only: a timeout never moves it there.
    var waiting = Mirror(roomId: "lab", tokenSeed: 2)
    _ = waiting.connectionOpened(at: 0, canRequest: true)
    _ = waiting.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 3), at: 0)
    for second in stride(from: 5.0, through: 120, by: 5) { _ = waiting.tick(at: second) }
    expect(waiting.eventHistory.coverage == .missing(pending: [0...3], lost: []), "timeouts leave 0–3 pending, never lost")
    expect(waiting.activity.requestsSent > 3, "and keep retrying with backoff")

    // Nothing published: known empty from a SyncEnd too.
    var none = Mirror(roomId: "lab", tokenSeed: 2)
    _ = none.connectionOpened(at: 0, canRequest: true)
    _ = none.receive(topic: .syncEnd, payload: answer(nil, bound: nil), at: 0)
    expect(none.eventHistory.coverage == .empty && none.activeGeneration == g1, "a SyncEnd with bound NONE witnesses an empty history")
}

/// Generations: install on the first message, archive, older ignored, continuity.
func testLiveRoomGenerationsAndPartialHistory() {
    // Joining mid-stream with no request path: a first event above seq 0 means history is missing.
    var joined = currentMirror(canRequest: false)
    _ = joined.receive(topic: .events, payload: event(id: 1, seq: 101), at: 101)
    _ = joined.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 102)
    expect(joined.eventHistory.coverage == .missing(pending: [0...100], lost: []), "first event at seq 101: 0–100 pending")
    var fromZero = currentMirror(canRequest: false)
    _ = fromZero.receive(topic: .events, payload: event(id: 1, seq: 0), at: 101)
    expect(fromZero.eventHistory.coverage == .completeThrough(0), "first event at seq 0: complete")

    // A restart while away: g2's counters start at 0 and pass g1's; nothing of g1 contaminates g2.
    var away = currentMirror()
    for seq in UInt64(0)...4 { _ = away.receive(topic: .events, payload: event(id: 1, seq: seq, state: seq == 0 ? .candidate : .confirmed), at: 101) }
    _ = away.receive(topic: .status, payload: status(epoch: 5, seq: 50, eventsSeq: 4), at: 103)
    away.connectionClosed(at: 104)
    _ = away.connectionOpened(at: 120, canRequest: true)
    let restarted = away.receive(topic: .status, payload: status(epoch: 6, seq: 60, eventsSeq: 9, g: g2), at: 121)
    expect(away.activeGeneration == g2 && away.highestGeneration == g2 && away.activity.generationChanges == 1, "g2 installed")
    expect(away.eventHistory.coverage == .missing(pending: [0...9], lost: []) && away.eventsById.isEmpty, "g2: 0–9 pending, nothing of g1")
    expect(away.continuity == .broken(sinceGeneration: g1), "continuity broken since g1")
    expect(away.archivedOpenEvents.map(\.generation) == [g1] && away.archivedOpenEvents.first?.entry.event.eventId == 1,
           "g1's open event archived, tagged g1")
    expect(away.mapState == .stale(.newGeneration) && away.bricks.count == 3, "g1's map greyed until g2 has one")
    let restartRequest = request(restarted, .events, "restart")
    expect(restartRequest?.since == nil && restartRequest?.expectedGeneration == g2, "g2: events since NONE, expected g2")
    expect(request(restarted, .mapBricks, "restart")?.since == nil, "g2: map since NONE")

    // An old-generation message after the install is ignored, whatever its seq.
    _ = away.receive(topic: .events, payload: event(id: 1, seq: 5, state: .accepted), at: 122)
    _ = away.receive(topic: .status, payload: status(epoch: 5, seq: 99, eventsSeq: 5), at: 122)
    expect(away.activity.olderGeneration == 2 && away.activeGeneration == g2 && away.eventsById.isEmpty, "g1 messages after g2: ignored")

    // The archive keeps the newest two generations.
    _ = away.receive(topic: .status, payload: status(epoch: 7, seq: 0, g: g2 + 1), at: 130)
    _ = away.receive(topic: .status, payload: status(epoch: 8, seq: 0, g: g2 + 2), at: 131)
    expect(away.archive.count == Mirror.archiveDepth && away.archive.map(\.generation) == [g2 + 1, g2], "two newest archived runs")
    expect(away.continuity == .broken(sinceGeneration: g2 + 1), "broken since the latest archived run")

    // An unreadable map payload: bricks kept, map stale, resync requested.
    let good = delta(epoch: 6, seq: 11, base: 5, [])
    // Cut inside the generation: it reads as a legacy message, a wire mismatch on this v2 connection.
    var cut = currentMirror()
    _ = cut.receive(topic: .mapBricks, payload: good.prefix(good.count - 9), at: 110)
    expect(cut.activity.wireMismatch == 1 && cut.mapState == .stale(.rejected), "a v2 delta cut to a legacy layout: stale(rejected)")
    var bad = currentMirror()
    let truncated = bad.receive(topic: .mapBricks, payload: good.prefix(good.count - 3), at: 110)
    expect(bad.activity.rejected == 1 && bad.mapState == .stale(.rejected) && bad.bricks.count == 3, "truncated map: stale(rejected), bricks kept")
    let resync = request(truncated, .mapBricks, "truncated")
    expect(resync?.since == .init(epoch: 5, seq: 10), "truncated map requests since (5, 10)")
    _ = bad.receive(topic: .syncEnd, payload: answer(resync, bound: 10), at: 111)
    _ = bad.receive(topic: .status, payload: status(epoch: 5, seq: 2), at: 111)
    expect(bad.mapState == .current, "the manifest says nothing newer than 10 exists: current again")
}

/// Codex round 1: an old "no map yet" never suppresses a map request after newer evidence; an
/// eviction never lets an older transition reopen a closed event.
func testLiveRoomEmptyMapAndEviction() {
    func building(epoch: UInt64, seq: UInt64) -> Data { LRWire.status(g1, seq: seq, epoch: epoch, eventsSeq: nil, state: .building) }
    func tracking(epoch: UInt64, seq: UInt64) -> Data { LRWire.status(g1, seq: seq, epoch: epoch, eventsSeq: nil, state: .tracking) }
    func emptyMap(_ asked: R.RequestSync?, epoch: UInt64) -> Data {
        LRWire.syncEnd(g1, topic: .mapBricks, token: asked?.token ?? .zero, bound: nil, epoch: epoch)
    }

    // BUILDING, the SyncEnd says no map; the first snapshot is missed and the status moves on.
    var missed = Mirror(roomId: "lab", tokenSeed: 4)
    _ = missed.connectionOpened(at: 0, canRequest: true)
    let first = request(missed.receive(topic: .status, payload: building(epoch: 1, seq: 0), at: 0), .mapBricks, "building")
    expect(first?.since == nil, "no chain while BUILDING: map requested")
    _ = missed.receive(topic: .syncEnd, payload: emptyMap(first, epoch: 1), at: 0.1)
    expect(missed.retrievalStatus(.mapBricks) == .idle && request(missed.tick(at: 10), .mapBricks, "empty") == nil,
           "a SyncEnd with no map: nothing more to ask")
    expect(request(missed.receive(topic: .status, payload: building(epoch: 1, seq: 1), at: 11), .mapBricks, "still building") == nil,
           "a later BUILDING status on the same epoch keeps the suppression")
    let degraded = LRWire.status(g1, seq: 2, epoch: 1, eventsSeq: nil, state: .degraded)
    expect(request(missed.receive(topic: .status, payload: degraded, at: 15), .mapBricks, "degraded") == nil,
           "DEGRADED on the same epoch (a sensor lost while BUILDING) keeps the suppression")
    let after = request(missed.receive(topic: .status, payload: tracking(epoch: 2, seq: 3), at: 20), .mapBricks, "tracking")
    expect(after != nil, "a status past BUILDING with no snapshot held requests the map")
    var epochOnly = Mirror(roomId: "lab", tokenSeed: 4)
    _ = epochOnly.connectionOpened(at: 0, canRequest: true)
    let asked = request(epochOnly.receive(topic: .status, payload: building(epoch: 1, seq: 0), at: 0), .mapBricks, "epoch only")
    _ = epochOnly.receive(topic: .syncEnd, payload: emptyMap(asked, epoch: 1), at: 0.1)
    expect(request(epochOnly.receive(topic: .status, payload: building(epoch: 3, seq: 1), at: 20), .mapBricks, "new epoch") != nil,
           "a BUILDING status on another epoch (re-referenced while unseen) requests the map")

    // A reconnect: the map may have been published while away, so the old answer no longer holds.
    var away = Mirror(roomId: "lab", tokenSeed: 4)
    _ = away.connectionOpened(at: 0, canRequest: true)
    let before = request(away.receive(topic: .status, payload: building(epoch: 1, seq: 0), at: 0), .mapBricks, "before")
    _ = away.receive(topic: .syncEnd, payload: emptyMap(before, epoch: 1), at: 0.1)
    away.connectionClosed(at: 10)
    _ = away.connectionOpened(at: 20, canRequest: true)
    let again = request(away.receive(topic: .status, payload: building(epoch: 1, seq: 1), at: 21), .mapBricks, "reconnect")
    expect(again != nil, "after a reconnect the map is asked for again")
    _ = away.receive(topic: .syncEnd, payload: emptyMap(again, epoch: 1), at: 21.1)
    expect(request(away.tick(at: 40), .mapBricks, "reconnect answered") == nil, "and a fresh empty answer suppresses again")

    // latest holds one event: A ACCEPTED@1, B ACCEPTED@2 evicts A, then A's CANDIDATE@0 replays late.
    var evicting = Mirror(roomId: "lab", limits: Mirror.Limits(latest: 1), tokenSeed: 4)
    _ = evicting.connectionOpened(at: 0, canRequest: false)
    _ = evicting.receive(topic: .events, payload: event(id: 1, seq: 1, state: .accepted), at: 1)
    _ = evicting.receive(topic: .events, payload: event(id: 2, seq: 2, state: .accepted), at: 2)
    _ = evicting.receive(topic: .syncEvents, payload: event(id: 1, seq: 0, state: .candidate), at: 3)
    expect(evicting.openEvents.isEmpty && evicting.eventsById[1] == nil, "an evicted accepted event is not reopened by its older candidate")
    _ = evicting.receive(topic: .syncEvents, payload: event(id: 1, seq: 1, state: .accepted), at: 4)
    expect(evicting.openEvents.isEmpty, "the accepted replay is a duplicate and A stays closed")
    expect(evicting.eventHistory.coverage == .completeThrough(2), "coverage still counts the late seq 0")
    _ = evicting.receive(topic: .events, payload: event(id: 3, seq: 3, state: .candidate), at: 5)
    expect(evicting.openEvents.map(\.event.eventId) == [3], "a newer event still enters latest")
}

/// `Activity` and `Epoch`: full state per generation, applied by seq, and never the layout's decider.
func testLiveRoomActivityAndEpoch() {
    // The reported bug: after a restart the new generation's seq starts at 0 again. It applies; the
    // old generation's higher seq does not hide it.
    var mirror = currentMirror()
    _ = mirror.receive(topic: .activity, payload: LRWire.activity(g1, seq: 50, active: true), at: 100.5)
    _ = mirror.receive(topic: .epoch, payload: LRWire.epochTick(g1, seq: 9), at: 100.5)
    expect(mirror.roomActivity?.message.header.seq == 50 && mirror.roomActivity?.message.active == true, "g1 activity 50 held")
    _ = mirror.receive(topic: .activity, payload: LRWire.activity(g1, seq: 50, active: true), at: 100.6)
    _ = mirror.receive(topic: .activity, payload: LRWire.activity(g1, seq: 49), at: 100.7)
    expect(mirror.activity.duplicates == 2 && mirror.roomActivity?.message.header.seq == 50, "a repeated or older seq is a duplicate")
    _ = mirror.receive(topic: .activity, payload: LRWire.activity(g2, seq: 0), at: 101)
    expect(mirror.activeGeneration == g2 && mirror.roomActivity?.message.header.seq == 0 && mirror.roomActivity?.message.active == false,
           "g2 activity seq 0 installs g2 and applies")
    expect(mirror.epochTick == nil && !mirror.isEpochFresh, "g1's epoch tick does not cross into g2")
    _ = mirror.receive(topic: .epoch, payload: LRWire.epochTick(g2, seq: 0), at: 101)
    expect(mirror.epochTick?.message.header.seq == 0 && mirror.isEpochFresh, "g2 epoch seq 0 applies")
    _ = mirror.receive(topic: .activity, payload: LRWire.activity(g1, seq: 51), at: 101.2)
    expect(mirror.activity.olderGeneration == 1 && mirror.roomActivity?.message.publisherGeneration == g2, "a late g1 activity is ignored")
    // Freshness: 3 s for activity, 15 s for epoch, on this connection only.
    _ = mirror.tick(at: 104.1)
    expect(!mirror.isActivityFresh && mirror.isEpochFresh, "activity stale after 3 s, epoch still fresh")
    _ = mirror.tick(at: 116.1)
    expect(!mirror.isEpochFresh, "epoch stale after 15 s")
    _ = mirror.receive(topic: .epoch, payload: LRWire.epochTick(g2, seq: 1), at: 116.5)
    mirror.connectionClosed(at: 117)
    _ = mirror.connectionOpened(at: 117, canRequest: false)
    expect(mirror.epochTick != nil && !mirror.isEpochFresh, "a tick from the last connection is kept, not fresh")

    // A node from before the Activity v2 tail (PR #18): v2 status, legacy transient_local Activity
    // that arrives first. It neither fixes the wire nor archives anything.
    var rollout = Mirror(tokenSeed: 5)
    _ = rollout.connectionOpened(at: 0, canRequest: true)
    _ = rollout.receive(topic: .activity, payload: LRWire.activity(nil, seq: 7), at: 0)
    expect(rollout.wire == .undetermined && rollout.activity.sideTopicLayout == 1 && rollout.roomActivity == nil,
           "a first legacy activity is dropped and leaves the wire undecided")
    _ = rollout.receive(topic: .mapBricks, payload: snapshot(epoch: 5, seq: 10, [brick(0)]), at: 0.1)
    _ = rollout.receive(topic: .status, payload: status(epoch: 5, seq: 1), at: 0.2)
    _ = rollout.receive(topic: .activity, payload: LRWire.activity(nil, seq: 8), at: 1)
    expect(rollout.wire == .v2 && rollout.mapState == .current && rollout.archive.isEmpty && rollout.activity.wireMismatch == 0,
           "the stateful topics decide v2; the map is current and nothing was archived")
    expect(rollout.activity.sideTopicLayout == 2 && rollout.roomActivity == nil && rollout.continuity == .verified,
           "legacy activity on a v2 wire is dropped and counted, continuity untouched")

    // A v2 activity before anything else waits for the wire too; a legacy node is all legacy.
    var early = Mirror(tokenSeed: 6)
    _ = early.connectionOpened(at: 0, canRequest: false)
    _ = early.receive(topic: .activity, payload: LRWire.activity(g1, seq: 0), at: 0)
    expect(early.wire == .undetermined && early.activeGeneration == nil && early.activity.sideTopicLayout == 1,
           "a first v2 activity is dropped, installs nothing")
    var legacy = Mirror(tokenSeed: 7)
    _ = legacy.connectionOpened(at: 0, canRequest: false)
    _ = legacy.receive(topic: .status, payload: status(epoch: 5, seq: 1, g: nil), at: 0)
    _ = legacy.receive(topic: .activity, payload: LRWire.activity(nil, seq: 3), at: 0.5)
    _ = legacy.receive(topic: .activity, payload: LRWire.activity(nil, seq: 3), at: 0.6)
    _ = legacy.receive(topic: .activity, payload: LRWire.activity(nil, seq: 0), at: 1.5)
    expect(legacy.wire == .legacy && legacy.roomActivity?.message.header.seq == 0 && legacy.activity.duplicates == 1,
           "legacy wire: only an exact copy is a duplicate; a lower seq (a restart) applies")
    _ = legacy.receive(topic: .activity, payload: LRWire.activity(g1, seq: 4), at: 2)
    expect(legacy.activity.sideTopicLayout == 1 && legacy.activity.wireMismatch == 0, "v2 activity on a legacy wire: dropped, side-topic counter")

    // Undecodable activity is rejected like any message, without touching the map.
    var bad = currentMirror()
    _ = bad.receive(topic: .activity, payload: Data(LRWire.activity(g1, seq: 1).dropLast(2)), at: 100)
    expect(bad.activity.rejected == 1 && bad.mapState == .current, "a truncated activity is rejected; the map stays current")
}

func runLiveRoomTests() {
    testLiveRoomFoxgloveClientFrames()
    print("  LiveRoom: serverInfo capabilities, client advertise + message data, topic/schema match (sync/end in, events_fallback out)"); passed()
    testLiveRoomGoldensDecode()
    print("  LiveRoom: every legacy and v2 hand golden decodes through Foxglove framing; the first message fixes the wire"); passed()
    testLiveRoomLiveCaptures()
    print("  LiveRoom: real :8767 frames (legacy layout) decode and run the mirror in legacy mode: shown, unverified, no requests"); passed()
    testLiveRoomSnapshotDeltaApply()
    print("  LiveRoom: snapshot + delta apply, canonical removals, duplicates, whole-message rejection, same-generation reconnect chains"); passed()
    testLiveRoomGapRequestsSync()
    print("  LiveRoom: gap → RequestSync(since, g, token); SyncEnd manifest completes it; 5/10/20 s timeouts with new tokens; dead tokens"); passed()
    testLiveRoomEpochChange()
    print("  LiveRoom: D20 epoch mismatch ×2 → resync; deltas racing statuses stay current; a restart is a new generation, not a low seq"); passed()
    testLiveRoomReconnect()
    print("  LiveRoom: close → stale; reconnect awaits a status of the generation, then requests with expected_generation; 3 s freshness"); passed()
    testLiveRoomEvents()
    print("  LiveRoom: coverage C from receipt only, pending seqs requested since the run from 0, manifest completion, highest-seq latest, conflicts"); passed()
    testLiveRoomEventsSeqGap()
    print("  LiveRoom: status events_seq witnesses pending seqs; a timeout with nothing pending goes idle; status copies are duplicates"); passed()
    testLiveRoomReplayAndFallbackEdges()
    print("  LiveRoom: replayed snapshot behind live, events fallback lost ranges, timeouts never lose a seq, empty SyncEnd witness"); passed()
    testLiveRoomGenerationsAndPartialHistory()
    print("  LiveRoom: generations install on the first message, archive the old run, ignore older ones; unreadable map → resync"); passed()
    testLiveRoomEmptyMapAndEviction()
    print("  LiveRoom: an empty-map answer yields to a later status or reconnect; eviction never reopens a closed event"); passed()
    testLiveRoomActivityAndEpoch()
    print("  LiveRoom: Activity/Epoch per generation (a restart's seq 0 applies), seq duplicates, freshness; never decide the wire"); passed()
    runLiveRoomTraceTests()
}

/// `swift run LabCommandCoreTestsRunner live-room-probe [seconds] [ws://host:port] [request]`: connects a
/// `LiveRoomClient`, runs for a while, and prints what the mirror holds. Read-only (never advertises,
/// publishes or calls a service) unless the third argument is `request`, which lets it send
/// `RequestSync` on whatever path the bridge offers (D39). Exits non-zero when no status arrived.
@MainActor
func runLiveRoomProbe(arguments: [String]) -> Int32 {
    let seconds = arguments.first.flatMap(Double.init) ?? 15
    let url: URL
    if let raw = arguments.dropFirst().first {
        switch RoomMemoryEndpoint.parse(raw) {
        case .success(let parsed):
            url = parsed
        case .failure(let error):
            print("FAIL: room endpoint: \(error.message)")
            return 1
        }
    } else {
        url = RoomMemoryEndpoint.defaultURL
    }
    let requests = arguments.dropFirst(2).first == "request"
    let client = LiveRoomClient(url: url, publishRequests: requests)
    client.start()
    let deadline = Date().addingTimeInterval(seconds)
    while Date() < deadline {
        RunLoop.main.run(until: Date().addingTimeInterval(0.1))
    }
    let mirror = client.mirror
    let path = client.requestPath
    client.stop()
    RunLoop.main.run(until: Date().addingTimeInterval(0.2))

    for line in client.log { print("  log: \(line)") }
    print("connection: \(client.connection)")
    print("request path: \(path)")
    for (topic, activity) in mirror.activity.topics.sorted(by: { $0.key < $1.key }) {
        print("  \(topic): \(activity.count) messages")
    }
    let a = mirror.activity
    print("rejected \(a.rejected), duplicates \(a.duplicates), gaps \(a.gaps), foreign \(a.foreignRoom), older generation \(a.olderGeneration), wire mismatch \(a.wireMismatch), requests sent \(a.requestsSent) unsent \(a.requestsUnsent)")
    print("wire \(mirror.wire), generation \(mirror.activeGeneration.map(String.init) ?? "—"), continuity \(mirror.continuity)")
    print("room \(mirror.roomId ?? "—"), map \(mirror.mapState), epoch \(mirror.mapEpoch.map(String.init) ?? "—"), seq \(mirror.mapSeq.map(String.init) ?? "—"), bricks \(mirror.bricks.count), voxel \(mirror.voxelSize.map { "\($0) m" } ?? "—")")
    print("events: \(mirror.events.count) held, \(mirror.openEvents.count) open, history \(mirror.eventHistory.coverage)")
    guard let status = mirror.status else {
        print("FAIL: no MemoryStatus arrived in \(Int(seconds)) s")
        return 1
    }
    print("status: \(status.state), epoch \(status.mapEpoch), bricks \(status.brickCount), open events \(status.openEventCount), coverage \(status.coverageKnownFraction.map { String(format: "%.3f", $0) } ?? "—"), events_seq \(status.eventsSeq.map(String.init) ?? "none"), generation \(status.publisherGeneration.map(String.init) ?? "legacy"), Hz mid360 \(status.mid360Hz.map(String.init) ?? "—") jt128 \(status.jt128Hz.map(String.init) ?? "—")")
    print("PASS live-room-probe: status arrived")
    return 0
}
