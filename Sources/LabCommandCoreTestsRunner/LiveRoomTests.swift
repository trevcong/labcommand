import Foundation
import LabCommandCore

// MARK: - LiveRoom: Foxglove client frames, RoomMemoryMirror resync rules, goldens and live captures

private typealias R = RoomMemory
private typealias Mirror = RoomMemoryMirror

private let stamp = R.Time(sec: 1_760_400_010, nanosec: 0)

private func header(_ epoch: UInt64, _ seq: UInt64, room: String = "lab") -> R.Header {
    R.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: R.Time(sec: stamp.sec + Int32(seq % 1000), nanosec: 0))
}

/// A brick with one known voxel; `occupied` sets that voxel occupied too.
private func brick(_ i: Int16, _ j: Int16 = 0, _ k: Int16 = 0, occupied: Bool = false) -> R.Brick {
    var known = [UInt8](repeating: 0, count: R.Brick.maskBytes)
    var occ = known
    known[0] = 1
    if occupied { occ[0] = 1 }
    return R.Brick(coordinate: R.BrickCoordinate(i: i, j: j, k: k), occupied: occ, known: known)
}

private func cdr(_ encode: () throws -> Data, _ what: String) -> Data {
    do { return try encode() } catch {
        expect(false, "LiveRoom fixture \(what) encodes: \(error)")
        return Data()
    }
}

private func snapshot(epoch: UInt64, seq: UInt64, _ bricks: [R.Brick], room: String = "lab") -> Data {
    let message = R.MapBricks(header: header(epoch, seq, room: room), kind: .snapshot, baseEpoch: 0, voxelSize: 0.05, bricks: bricks)
    return cdr(message.encodedCDR, "snapshot e\(epoch) s\(seq)")
}

private func delta(epoch: UInt64, seq: UInt64, base: UInt64, _ bricks: [R.Brick], removed: [UInt32] = []) -> Data {
    let message = R.MapBricks(
        header: header(epoch, seq), kind: .delta, baseEpoch: base, voxelSize: 0.05, bricks: bricks, removedBrickIndices: removed
    )
    return cdr(message.encodedCDR, "delta e\(epoch) s\(seq)")
}

private func status(epoch: UInt64, seq: UInt64, eventsSeq: UInt64? = nil, room: String = "lab") -> Data {
    let message = R.MemoryStatus(
        header: header(epoch, seq, room: room), state: .tracking, brickCount: 0, openEventCount: 0,
        coverageKnownFraction: 0.5, poseResidualM: nil, windowS: 1, mid360Hz: 10, jt128Hz: nil,
        droppedWindows: 0, eventsSeq: eventsSeq
    )
    return cdr(message.encodedCDR, "status s\(seq)")
}

private func event(id: UInt64, seq: UInt64, state: R.ChangeEvent.State = .candidate, epoch: UInt64 = 5) -> Data {
    let h = header(epoch, seq)
    let message = R.ChangeEvent(
        header: h, eventId: id, kind: .appeared, state: state,
        centroid: SIMD3(1, 1, 1), aabb: .init(min: SIMD3(0.9, 0.9, 0.9), max: SIMD3(1.1, 1.1, 1.1)),
        fromCentroid: nil, voxelCount: 8, volumeM3: 0.001, evidence: nil, alpha: nil,
        sensors: .jt128, firstSeen: h.stamp, lastSeen: h.stamp, mask: nil
    )
    return cdr(message.encodedCDR, "event \(id) s\(seq)")
}

private func requests(_ actions: [Mirror.Action]) -> [R.RequestSync] {
    actions.map { action in
        switch action {
        case .requestSync(let request): return request
        }
    }
}

/// The single request in `actions` for `topic`, round-tripped through the codec as the bridge would carry it.
private func request(_ actions: [Mirror.Action], _ topic: R.RequestSync.Topic, _ context: String) -> R.RequestSync? {
    let matching = requests(actions).filter { $0.topic == topic }
    expect(matching.count <= 1, "\(context): at most one \(topic) request, got \(matching.count)")
    guard let sent = matching.first else { return nil }
    do {
        let decoded = try R.RequestSync.decode(cdr: try sent.encodedCDR())
        expect(decoded == sent, "\(context): RequestSync round-trips through CDR")
        return decoded
    } catch {
        expect(false, "\(context): RequestSync encodes and decodes: \(error)")
        return nil
    }
}

/// A connected mirror with a current map at (epoch 5, seq 10) holding bricks 0, 1, 2 and a fresh status.
private func currentMirror(canRequest: Bool = true, at now: TimeInterval = 100) -> Mirror {
    var mirror = Mirror()
    _ = mirror.connectionOpened(at: now, canRequest: canRequest)
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 5, seq: 10, [brick(0), brick(1), brick(2)]), at: now)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 1), at: now)
    expect(mirror.mapState == .current && mirror.isMapCurrent, "setup: snapshot makes the map current")
    return mirror
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
        ("epoch_le", nil),
        ("epoch_be", nil),
        ("request_sync_map_le", nil),
        ("request_sync_map_be", nil),
        ("request_sync_events_none_le", nil),
    ]
    for (name, topic) in goldens {
        let payload = throughFoxglove(hexFixture(name))
        do {
            switch name {
            case let n where n.hasPrefix("map_bricks"): _ = try R.MapBricks.decode(cdr: payload)
            case let n where n.hasPrefix("change_event"): _ = try R.ChangeEvent.decode(cdr: payload)
            case let n where n.hasPrefix("memory_status"):
                let decoded = try R.MemoryStatus.decode(cdr: payload)
                expect(decoded.eventsSeq == 12, "memory_status golden carries events_seq 12")
            case let n where n.hasPrefix("epoch"): _ = try R.Epoch.decode(cdr: payload)
            default: _ = try R.RequestSync.decode(cdr: payload)
            }
        } catch {
            expect(false, "golden \(name) decodes after Foxglove framing: \(error)")
        }
        // Every golden for a subscribed topic is accepted by the mirror: never counted rejected.
        if let topic {
            var mirror = Mirror()
            _ = mirror.connectionOpened(at: 0, canRequest: false)
            _ = mirror.receive(topic: topic, payload: payload, at: 0)
            expect(mirror.activity.rejected == 0, "mirror accepts golden \(name)")
            expect(mirror.activity.topics[topic.rawValue]?.count == 1, "activity counts golden \(name) on \(topic.rawValue)")
        }
    }

    // The delta goldens alone are not a map: a delta with no mirror leaves the map empty.
    var mirror = Mirror()
    _ = mirror.connectionOpened(at: 0, canRequest: true)
    _ = mirror.receive(topic: .mapBricks, payload: hexFixture("map_bricks_delta_le"), at: 0)
    expect(mirror.mapState == .empty && mirror.bricks.isEmpty, "a delta with no mirror applies nothing")

    // A snapshot golden makes the map current with its bricks.
    let snap = hexFixture("map_bricks_snapshot_zlib_le")
    mirror = Mirror()
    _ = mirror.connectionOpened(at: 0, canRequest: true)
    _ = mirror.receive(topic: .mapBricks, payload: snap, at: 0)
    let decoded = try? R.MapBricks.decode(cdr: snap)
    expect(mirror.mapState == .current, "golden snapshot makes the map current")
    expect(mirror.bricks.count == decoded?.bricks.count && mirror.mapEpoch == decoded?.header.mapEpoch, "golden snapshot bricks + epoch")
    expect(mirror.roomId == decoded?.header.roomId, "room adopted from the first message")
}

/// Real frames from the Spark bridge at :8767 (read-only capture, 2026-10-15).
func testLiveRoomLiveCaptures() {
    func load(_ name: String) -> Data? {
        try? Data(contentsOf: URL(fileURLWithPath: "Tests/Fixtures/room/live/\(name).bin"))
    }
    guard let statusBytes = load("status"), let snapBytes = load("map_snapshot") else {
        expect(false, "Tests/Fixtures/room/live/{status,map_snapshot}.bin present")
        return
    }
    do {
        // Both statuses are 76 bytes: main's codec without events_seq would reject 8 trailing bytes.
        let live = try R.MemoryStatus.decode(cdr: statusBytes)
        expect(live.header.roomId == "lab", "live status room is lab")
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

        var mirror = Mirror()
        _ = mirror.connectionOpened(at: 0, canRequest: false)
        _ = mirror.receive(topic: .status, payload: statusBytes, at: 0)
        _ = mirror.receive(topic: .mapBricks, payload: snapBytes, at: 0.5)
        expect(mirror.activity.rejected == 0, "live captures are accepted")
        expect(mirror.bricks.count == snap.bricks.count, "live snapshot: \(snap.bricks.count) bricks mirrored")
        expect(mirror.status?.eventsSeq == live.eventsSeq, "mirror status exposes events_seq")

        if let deltaBytes = load("map_delta") {
            let d = try R.MapBricks.decode(cdr: deltaBytes)
            expect(d.kind == .delta && d.baseEpoch == snap.header.mapEpoch && d.header.seq == snap.header.seq + 1,
                   "captured delta chains to the captured snapshot")
            _ = mirror.receive(topic: .mapBricks, payload: deltaBytes, at: 1)
            expect(mirror.mapEpoch == d.header.mapEpoch && mirror.mapSeq == d.header.seq, "live delta applies")
        }
        if let eventBytes = load("events") {
            let e = try R.ChangeEvent.decode(cdr: eventBytes)
            _ = mirror.receive(topic: .events, payload: eventBytes, at: 2)
            expect(mirror.eventsById[e.eventId]?.event == e && mirror.eventsSeq == e.header.seq, "live event applies")
        }
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
    expect(mirror.mapState == .current && mirror.mapEpoch == 6 && mirror.mapSeq == 11, "delta advances epoch and seq")
    expect(Set(mirror.bricks.keys.map(\.i)) == [0, 2, 7], "delta removed brick 1 and inserted brick 7")
    expect(mirror.bricks[R.BrickCoordinate(i: 2, j: 0, k: 0)]?.evidence(x: 0, y: 0, z: 0) == .occupied, "delta replaced brick 2")
    expect(mirror.mapRevision == revision + 1, "map revision bumps once per change")

    // Removed indices refer to the canonical (sorted) order of the base epoch: bricks 0, 2, 7.
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, [], removed: [0, 2]), at: 102)
    expect(Set(mirror.bricks.keys.map(\.i)) == [2], "removed [0, 2] of (0, 2, 7) leaves brick 2")

    // A duplicate (seq and epoch not newer) is ignored and counted.
    let before = mirror
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 103)
    expect(mirror.bricks == before.bricks && mirror.mapState == .current, "duplicate delta changes nothing")
    expect(mirror.activity.duplicates == 1, "duplicate delta counted")

    // An invalid delta (index out of range) is rejected whole: map stale, mirror untouched, resync requested.
    actions = mirror.receive(topic: .mapBricks, payload: delta(epoch: 8, seq: 13, base: 7, [brick(9)], removed: [5]), at: 110)
    expect(mirror.mapState == .stale(.rejected), "out-of-range removal: stale(rejected)")
    expect(mirror.bricks == before.bricks && mirror.mapEpoch == 7, "rejected delta is not partly applied")
    expect(request(actions, .mapBricks, "rejected delta")?.since == .init(epoch: 7, seq: 12), "rejected delta requests since (7, 12)")

    // Removing and re-sending the same brick is invalid too.
    var other = currentMirror()
    _ = other.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, [brick(1)], removed: [1]), at: 101)
    expect(other.mapState == .stale(.rejected) && other.bricks.count == 3, "remove + re-send same brick rejected whole")

    // A sync-topic delta that chains (the replay) heals it once it reaches the newest live seq and status epoch.
    other = currentMirror()
    _ = other.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 101)
    expect(other.mapState == .stale(.seqGap), "seq 12 after 10 is a gap")
    _ = other.receive(topic: .syncMapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 102)
    expect(other.mapState == .stale(.seqGap), "replay of 11 alone is behind the live seq 12")
    _ = other.receive(topic: .syncMapBricks, payload: delta(epoch: 7, seq: 12, base: 6, [brick(4)]), at: 102.5)
    _ = other.receive(topic: .status, payload: status(epoch: 7, seq: 2), at: 103)
    expect(other.mapState == .current && other.mapEpoch == 7 && other.bricks.count == 4, "replay chained to live seq + status epoch: current")
}

func testLiveRoomGapRequestsSync() {
    var mirror = currentMirror()
    // seq 12 after 10: gap.
    let actions = mirror.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 101)
    expect(mirror.mapState == .stale(.seqGap) && !mirror.isMapCurrent, "seq gap makes the map stale")
    expect(mirror.bricks.count == 3, "stale keeps the last good bricks")
    let sent = request(actions, .mapBricks, "seq gap")
    expect(sent?.since == .init(epoch: 5, seq: 10), "gap request carries since (mirror epoch, last seq)")
    expect(sent?.header.mapEpoch == 5 && sent?.header.roomId == "lab", "request header repeats since_epoch and the room")
    expect(mirror.activity.gaps == 1 && mirror.activity.requestsSent >= 1, "gap and request counted")

    // Wrong base.
    mirror = currentMirror()
    let wrongBase = mirror.receive(topic: .mapBricks, payload: delta(epoch: 9, seq: 11, base: 8, []), at: 101)
    expect(mirror.mapState == .stale(.wrongBase), "base_epoch ≠ mirror epoch: stale(wrongBase)")
    expect(request(wrongBase, .mapBricks, "wrong base")?.since == .init(epoch: 5, seq: 10), "wrong base requests since (5, 10)")

    // Sync answers only matter while a request is out: a foreign snapshot is ignored otherwise.
    mirror = currentMirror()
    var quiet = mirror
    _ = quiet.tick(at: 200) // the connect request resolves with the snapshot; nothing outstanding now
    _ = quiet.receive(topic: .syncMapBricks, payload: snapshot(epoch: 99, seq: 99, [brick(5)]), at: 201)
    expect(quiet.mapEpoch == 5, "sync snapshot ignored with no request outstanding")

    // Repeats: 5 s, then 10, 20, 30 s; never more than one per topic per 5 s.
    var retry = currentMirror(at: 0)
    let first = retry.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 10)
    expect(request(first, .mapBricks, "first") != nil, "first request goes out at once")
    expect(request(retry.tick(at: 14), .mapBricks, "4 s") == nil, "no repeat before 5 s")
    let second = request(retry.tick(at: 15), .mapBricks, "5 s")
    let firstSeq = request(first, .mapBricks, "first")?.header.seq ?? .max
    expect(second != nil && second!.header.seq > firstSeq, "repeat after 5 s, request seq counts up")
    expect(request(retry.tick(at: 24), .mapBricks, "9 s later") == nil, "second repeat waits 10 s")
    expect(request(retry.tick(at: 25), .mapBricks, "10 s later") != nil, "second repeat at 10 s")
    expect(request(retry.tick(at: 44), .mapBricks, "19 s") == nil && request(retry.tick(at: 45), .mapBricks, "20 s") != nil,
           "third repeat at 20 s")
    _ = retry.receive(topic: .syncMapBricks, payload: snapshot(epoch: 7, seq: 12, [brick(3)]), at: 46)
    expect(retry.mapState == .current, "a sync snapshot answers the request")
    expect(request(retry.tick(at: 200), .mapBricks, "resolved") == nil, "resolved: no more map repeats")

    // The events request opened on connect stays out until there is a baseline, or the status says there are no events.
    var quietRoom = Mirror()
    _ = quietRoom.connectionOpened(at: 0, canRequest: true)
    let opening = quietRoom.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 50), at: 0)
    expect(request(opening, .events, "connect")?.since == nil, "connect asks for events since NONE")
    expect(request(quietRoom.tick(at: 5), .events, "no baseline") != nil, "events request repeats while events exist and none held")
    _ = quietRoom.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: nil), at: 6)
    expect(request(quietRoom.tick(at: 100), .events, "no events") == nil, "status with no events_seq resolves the events request")

    // No request path: no actions, the map stays stale until a live snapshot.
    var noPath = currentMirror(canRequest: false)
    expect(noPath.receive(topic: .mapBricks, payload: delta(epoch: 7, seq: 12, base: 6, []), at: 101).isEmpty,
           "no request path: nothing to send")
    expect(noPath.tick(at: 200).isEmpty && noPath.mapState == .stale(.seqGap), "no path: stays stale, no repeats")
    _ = noPath.receive(topic: .mapBricks, payload: snapshot(epoch: 8, seq: 14, [brick(1)]), at: 201)
    expect(noPath.mapState == .current && noPath.bricks.count == 1, "a live snapshot recovers without a request path")
}

func testLiveRoomEpochChange() {
    // D20: the status epoch disagreeing twice in a row makes the map stale and requests a resync.
    var mirror = currentMirror()
    var actions = mirror.receive(topic: .status, payload: status(epoch: 6, seq: 2), at: 101)
    expect(mirror.mapState == .current && request(actions, .mapBricks, "one mismatch") == nil, "one mismatching status is lag")
    actions = mirror.receive(topic: .status, payload: status(epoch: 6, seq: 3), at: 102)
    expect(mirror.mapState == .stale(.epochMismatch), "two mismatching statuses: stale(epochMismatch)")
    expect(request(actions, .mapBricks, "epoch change")?.since == .init(epoch: 5, seq: 10), "epoch change requests since (5, 10)")
    // The lost delta arrives late (seq 11 chains): current again once the status agrees.
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 103)
    expect(mirror.mapState == .current, "chained delta + agreeing status heals an epoch mismatch")

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

    // Epoch reset: the node restarts; its snapshot has a lower seq and epoch and is accepted whole.
    mirror = currentMirror()
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, [brick(8)]), at: 101)
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 1, seq: 0, [brick(4)]), at: 102)
    expect(mirror.mapState == .current && mirror.mapEpoch == 1 && mirror.mapSeq == 0, "restart snapshot resets epoch and seq")
    expect(Array(mirror.bricks.keys.map(\.i)) == [4], "restart snapshot replaces the bricks")
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 2, seq: 1, base: 1, [brick(5)]), at: 103)
    expect(mirror.mapEpoch == 2 && mirror.bricks.count == 2, "deltas chain on the new generation")

    // Another room is ignored, never mixed in.
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 50, seq: 50, [brick(9)], room: "garage"), at: 104)
    _ = mirror.receive(topic: .status, payload: status(epoch: 50, seq: 9, room: "garage"), at: 104)
    expect(mirror.mapEpoch == 2 && mirror.roomId == "lab" && mirror.activity.foreignRoom == 2, "foreign room ignored and counted")
    var pinned = Mirror(roomId: "garage")
    _ = pinned.connectionOpened(at: 0, canRequest: false)
    _ = pinned.receive(topic: .mapBricks, payload: snapshot(epoch: 5, seq: 1, [brick(1)]), at: 0)
    expect(pinned.mapState == .empty, "a pinned room ignores other rooms")
}

func testLiveRoomReconnect() {
    var mirror = currentMirror()
    mirror.connectionClosed(at: 101)
    expect(mirror.mapState == .stale(.disconnected) && !mirror.isStatusFresh, "socket close: map stale, status not fresh")
    expect(mirror.bricks.count == 3, "socket close keeps the last good bricks")

    let actions = mirror.connectionOpened(at: 110, canRequest: true)
    expect(request(actions, .mapBricks, "reconnect")?.since == .init(epoch: 5, seq: 10), "reconnect requests map since its baseline")
    expect(request(actions, .events, "reconnect")?.since == nil, "reconnect requests events since NONE without a baseline")

    // A delta first after reconnect does not prove the mirror current, even one that chains.
    _ = mirror.receive(topic: .status, payload: status(epoch: 6, seq: 2), at: 110.5)
    _ = mirror.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 111)
    expect(mirror.mapState == .stale(.disconnected) && mirror.mapEpoch == 5, "delta first after reconnect: still stale")
    _ = mirror.receive(topic: .mapBricks, payload: snapshot(epoch: 6, seq: 11, [brick(0)]), at: 112)
    expect(mirror.mapState == .current && mirror.isMapCurrent, "snapshot after reconnect: current")

    // Status freshness: 3 s.
    _ = mirror.tick(at: 110.5 + 2.9)
    expect(mirror.isStatusFresh, "status fresh under 3 s")
    _ = mirror.tick(at: 110.5 + 3.1)
    expect(!mirror.isStatusFresh && !mirror.isMapCurrent, "status stale after 3 s: map not shown as current")

    // On a fresh connection with no mirror, the first delta starts nothing.
    var fresh = Mirror()
    _ = fresh.connectionOpened(at: 0, canRequest: true)
    expect(fresh.tick(at: 1).isEmpty, "requests wait for the room id")
    let first = fresh.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 2)
    expect(fresh.mapState == .empty && request(first, .mapBricks, "no mirror")?.since == nil, "no mirror: request since NONE")
}

// MARK: Events

func testLiveRoomEvents() {
    var mirror = currentMirror()
    _ = mirror.tick(at: 100) // flush connect requests
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 0), at: 101)
    expect(mirror.eventsSeq == 0 && mirror.eventsState == .complete, "first event starts the baseline")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 1, state: .confirmed), at: 102)
    expect(mirror.eventsById[1]?.event.state == .confirmed && mirror.openEvents.count == 1, "transition applies by event_id")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 1, state: .confirmed), at: 103)
    expect(mirror.activity.duplicates == 1 && mirror.events.count == 2, "duplicate event counted, not stored")

    // Gap: 44 after 41.
    let gapTime: TimeInterval = 120
    var actions = mirror.receive(topic: .events, payload: event(id: 2, seq: 4), at: gapTime)
    expect(mirror.eventsState == .incomplete(since: header(5, 1).stamp), "gap: incomplete since the last good stamp")
    expect(mirror.eventsById[2] != nil, "the newer message still shows")
    expect(mirror.eventsSeq == 1, "baseline holds so the replay can fill the gap")
    let sent = request(actions, .events, "events gap")
    expect(sent?.since == .init(epoch: 5, seq: 1), "events gap requests since (epoch, 1)")

    // Replay of 42, 43 on the sync topic closes it; the held 44 is absorbed.
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 1, seq: 2, state: .accepted), at: 121)
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 3, seq: 3), at: 121)
    expect(mirror.eventsSeq == 4 && mirror.eventsState == .complete, "replay contiguous through 4: complete")
    expect(mirror.eventsById[1]?.event.state == .accepted && mirror.openEvents.map(\.event.eventId) == [2, 3], "replayed states apply")
    expect(mirror.events.last?.fromSync == true, "replayed messages are marked fromSync")
    // The connect status named no event; a request is done only once a status agrees with the baseline.
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 4), at: 122)
    actions = mirror.tick(at: 200)
    expect(request(actions, .events, "after replay") == nil, "resolved: no repeat")

    // An event's state only moves on a newer seq: an old replayed copy goes to history only.
    _ = mirror.receive(topic: .events, payload: event(id: 3, seq: 5, state: .confirmed), at: 201)
    _ = mirror.receive(topic: .events, payload: event(id: 3, seq: 7, state: .expired), at: 202) // gap 6
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 3, seq: 6, state: .reverted), at: 203)
    expect(mirror.eventsById[3]?.event.state == .expired, "older replayed transition does not overwrite a newer one")
    expect(mirror.eventsSeq == 7 && mirror.eventsState == .complete, "gap 6 replayed: complete")

    // A live seq drop alone is out-of-order delivery, not a restart: nothing rolls back or is cut.
    _ = mirror.receive(topic: .events, payload: event(id: 3, seq: 0, state: .candidate), at: 210)
    expect(mirror.eventsSeq == 7 && mirror.eventsById[3]?.event.state == .expired, "live seq drop: no rollback, baseline stays")
    expect(mirror.eventsState == .complete && mirror.activity.historyCuts == 0, "live seq drop: no cut")

    // Node restart: the status seq drops. New generation, history incomplete for good.
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 9, eventsSeq: 7), at: 211)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 0, eventsSeq: 0), at: 212)
    _ = mirror.receive(topic: .events, payload: event(id: 3, seq: 1, state: .candidate), at: 213)
    expect(mirror.eventsSeq == 1 && mirror.eventsById[3]?.event.state == .candidate, "restart: new generation applies")
    expect(mirror.eventsState != .complete && mirror.activity.historyCuts == 1, "restart leaves the history incomplete")

    // The ring is bounded.
    var ring = currentMirror(canRequest: false)
    for seq in 0..<UInt64(Mirror.eventCapacity + 50) {
        _ = ring.receive(topic: .events, payload: event(id: seq % 7, seq: seq), at: 100)
    }
    expect(ring.events.count == Mirror.eventCapacity && ring.events.first?.seq == 50, "event ring holds the newest \(Mirror.eventCapacity)")

    // No request path: a gap skips forward and stays incomplete.
    var noPath = currentMirror(canRequest: false)
    _ = noPath.receive(topic: .events, payload: event(id: 1, seq: 1), at: 101)
    expect(noPath.receive(topic: .events, payload: event(id: 1, seq: 5), at: 102).isEmpty, "no path: no request")
    expect(noPath.eventsSeq == 5 && noPath.eventsState != .complete, "no path: baseline jumps, history incomplete")
    _ = noPath.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 5), at: 103)
    expect(noPath.eventsState != .complete, "skipped history never reads complete again")
}

func testLiveRoomEventsSeqGap() {
    var mirror = currentMirror()
    _ = mirror.tick(at: 100)
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 0), at: 100)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 0), at: 101)
    expect(mirror.eventsState == .complete, "status at the baseline: complete")

    // events_seq ahead once is a race with the event itself.
    var actions = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 1), at: 102)
    expect(request(actions, .events, "one ahead") == nil, "one status ahead is not a gap yet")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 1), at: 102.3)
    actions = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 4, eventsSeq: 1), at: 103)
    expect(request(actions, .events, "caught up") == nil && mirror.eventsState == .complete, "event arrived: no gap")

    // The last events before a quiet period were lost: the next status still finds 3 unreached.
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 5, eventsSeq: 3), at: 104)
    actions = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 6, eventsSeq: 3), at: 105)
    expect(request(actions, .events, "events_seq gap")?.since == .init(epoch: 5, seq: 1), "events_seq gap requests since (5, 1)")
    expect(mirror.eventsState == .incomplete(since: header(5, 1).stamp), "events_seq gap: incomplete")
    expect(mirror.activity.gaps == 1, "events_seq gap counted once")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 7, eventsSeq: 3), at: 106)
    expect(mirror.activity.gaps == 1, "a request already out is not a second gap")

    // The replay of 2 and 3 reaches the mark: complete, resolved.
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 4, seq: 2), at: 107)
    expect(mirror.eventsState != .complete, "2 is short of the mark 3")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 4, seq: 3, state: .confirmed), at: 107.2)
    expect(mirror.eventsSeq == 3 && mirror.eventsState == .complete, "replay reached the mark: complete")
    expect(request(mirror.tick(at: 300), .events, "resolved") == nil, "events request resolved")

    // A duplicated status copy does not count as a second status.
    var dup = currentMirror()
    _ = dup.tick(at: 100)
    _ = dup.receive(topic: .events, payload: event(id: 1, seq: 0), at: 100)
    let ahead = status(epoch: 5, seq: 9, eventsSeq: 2)
    _ = dup.receive(topic: .status, payload: ahead, at: 101)
    actions = dup.receive(topic: .status, payload: ahead, at: 101.1)
    expect(request(actions, .events, "status copy") == nil && dup.activity.duplicates == 1, "status copy is a duplicate, not a gap")
}

/// Reproductions from the Codex review of this branch.
func testLiveRoomReplayAndFallbackEdges() {
    // A replayed snapshot behind the live topic is not current until the replay's deltas catch up.
    var map = currentMirror()
    _ = map.receive(topic: .status, payload: status(epoch: 8, seq: 2), at: 110)
    _ = map.receive(topic: .mapBricks, payload: delta(epoch: 8, seq: 13, base: 7, []), at: 110)
    expect(map.mapState == .stale(.seqGap), "live seq 13 after 10: stale")
    _ = map.receive(topic: .syncMapBricks, payload: snapshot(epoch: 6, seq: 11, [brick(1)]), at: 111)
    expect(map.mapState != .current && !map.isMapCurrent, "replayed snapshot at seq 11 < live 13 is not current")
    _ = map.receive(topic: .syncMapBricks, payload: delta(epoch: 7, seq: 12, base: 6, [brick(2)]), at: 111)
    _ = map.receive(topic: .syncMapBricks, payload: delta(epoch: 8, seq: 13, base: 7, [brick(3)]), at: 111)
    expect(map.mapState == .current && map.mapSeq == 13 && map.mapEpoch == 8 && map.bricks.count == 3,
           "the replay's deltas chain on from its snapshot to the live seq: current")

    // The initial SINCE_NONE answer, not a live event, settles what exists.
    var cold = Mirror(roomId: "lab")
    _ = cold.connectionOpened(at: 0, canRequest: true)
    _ = cold.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 100), at: 0)
    _ = cold.receive(topic: .events, payload: event(id: 101, seq: 101), at: 1)
    expect(request(cold.tick(at: 1.5), .events, "live first") == nil, "no repeat yet")
    _ = cold.receive(topic: .syncEvents, payload: event(id: 10, seq: 10, state: .confirmed), at: 2)
    expect(cold.eventsById[10]?.event.state == .confirmed, "an old open event from the fallback is kept although a live event came first")
    _ = cold.tick(at: 4.5)
    expect(cold.eventsState != .complete, "a fallback with holes leaves the history incomplete")
    _ = cold.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 5)
    expect(request(cold.tick(at: 60), .events, "settled") == nil, "settled fallback resolves once the status catches up")

    // A bounded fallback answering a baseline the Spark no longer keeps: jump to the highest replayed seq.
    var fallback = currentMirror()
    _ = fallback.tick(at: 100)
    _ = fallback.receive(topic: .events, payload: event(id: 1, seq: 0), at: 101)
    let gap = fallback.receive(topic: .events, payload: event(id: 1, seq: 300), at: 110)
    expect(request(gap, .events, "deep gap")?.since == .init(epoch: 5, seq: 0), "deep gap requests since (5, 0)")
    _ = fallback.receive(topic: .syncEvents, payload: event(id: 9, seq: 20, state: .confirmed), at: 111)
    for seq in UInt64(201)...300 {
        _ = fallback.receive(topic: .syncEvents, payload: event(id: 1000 + seq, seq: seq, state: .reverted), at: 111)
    }
    expect(fallback.eventsSeq == 0, "baseline holds while the answer is arriving")
    _ = fallback.tick(at: 114)
    expect(fallback.eventsSeq == 300 && fallback.eventsState != .complete, "settled fallback: baseline jumps to 300, incomplete")
    expect(fallback.eventsById[9]?.event.state == .confirmed, "the old open event from the fallback is kept")
    _ = fallback.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 300), at: 115)
    expect(request(fallback.tick(at: 200), .events, "after fallback") == nil, "no endless retries after a fallback")
    _ = fallback.receive(topic: .events, payload: event(id: 1, seq: 301, state: .confirmed), at: 201)
    expect(fallback.eventsSeq == 301 && fallback.eventsState != .complete, "live events chain on; lost history stays flagged")

    // Events exist but none ever arrived: the high-water mark alone triggers a SINCE_NONE request.
    var lost = Mirror(roomId: "lab")
    _ = lost.connectionOpened(at: 0, canRequest: true)
    _ = lost.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: nil), at: 0)
    expect(request(lost.tick(at: 10), .events, "no events") == nil, "no events published: nothing to request")
    _ = lost.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 0), at: 11)
    let missing = lost.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 0), at: 12)
    expect(request(missing, .events, "first event lost")?.since == nil, "lost first event: request since NONE")
    expect(lost.eventsState != .complete, "lost first event: incomplete while it is fetched")
    _ = lost.receive(topic: .syncEvents, payload: event(id: 7, seq: 0), at: 13)
    _ = lost.tick(at: 16)
    expect(lost.eventsSeq == 0 && lost.eventsState == .complete, "a replay from seq 0 with no hole: complete")
}

/// Reproductions from the second Codex review: generations, partial first history, unreadable maps.
func testLiveRoomGenerationsAndPartialHistory() {
    // Joining mid-stream with no request path: a first event above seq 0 means history is missing.
    var joined = currentMirror(canRequest: false)
    _ = joined.receive(topic: .events, payload: event(id: 1, seq: 101), at: 101)
    _ = joined.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 102)
    expect(joined.eventsState != .complete, "first event at seq 101 without sync: incomplete")
    var fromZero = currentMirror(canRequest: false)
    _ = fromZero.receive(topic: .events, payload: event(id: 1, seq: 0), at: 101)
    expect(fromZero.eventsState == .complete, "first event at seq 0: complete")

    // Initial sync retries stay SINCE_NONE even after a live event set a baseline.
    var retry = Mirror(roomId: "lab")
    _ = retry.connectionOpened(at: 0, canRequest: true)
    _ = retry.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 50), at: 0)
    _ = retry.receive(topic: .events, payload: event(id: 51, seq: 51), at: 1)
    expect(request(retry.tick(at: 5), .events, "initial retry") .map { $0.since == nil } == true,
           "initial events retry keeps since NONE")

    // Node restart while disconnected: status seq resets; the events baseline starts a new generation.
    var away = currentMirror()
    _ = away.tick(at: 100)
    _ = away.receive(topic: .events, payload: event(id: 1, seq: 0), at: 101)
    _ = away.receive(topic: .events, payload: event(id: 1, seq: 1, state: .confirmed), at: 102)
    _ = away.receive(topic: .status, payload: status(epoch: 5, seq: 50, eventsSeq: 1), at: 103)
    expect(away.eventsState == .complete, "setup: events complete")
    away.connectionClosed(at: 104)
    _ = away.connectionOpened(at: 120, canRequest: true)
    let restarted = away.receive(topic: .status, payload: status(epoch: 1, seq: 0, eventsSeq: 0), at: 121)
    expect(away.eventsSeq == nil && away.eventsState != .complete, "status seq reset: events baseline dropped, incomplete")
    let restartRequest = request(restarted, .events, "restart") ?? request(away.tick(at: 130), .events, "restart retry")
    expect(restartRequest != nil && restartRequest!.since == nil, "restart: events requested since NONE")

    // events_seq below the baseline twice (status seq not reset, e.g. a long absence): new generation.
    var low = currentMirror()
    _ = low.tick(at: 100)
    for seq in UInt64(0)...5 { _ = low.receive(topic: .events, payload: event(id: 1, seq: seq), at: 101) }
    _ = low.receive(topic: .status, payload: status(epoch: 5, seq: 900, eventsSeq: 2), at: 102)
    expect(low.eventsSeq == 5, "one low mark may be lag")
    _ = low.receive(topic: .status, payload: status(epoch: 5, seq: 901, eventsSeq: 2), at: 103)
    expect(low.eventsSeq == nil && low.eventsState != .complete, "two low marks: new generation, incomplete")

    // Status seq reset: a replayed startup snapshot of the new generation is current once it agrees.
    var map = currentMirror()
    _ = map.receive(topic: .mapBricks, payload: delta(epoch: 6, seq: 11, base: 5, []), at: 101)
    _ = map.receive(topic: .status, payload: status(epoch: 6, seq: 30), at: 102)
    _ = map.receive(topic: .status, payload: status(epoch: 1, seq: 0), at: 110)
    let d20 = map.receive(topic: .status, payload: status(epoch: 1, seq: 1), at: 111)
    expect(map.mapState == .stale(.epochMismatch) && request(d20, .mapBricks, "restart D20") != nil,
           "missed startup snapshot: D20 requests the map")
    _ = map.receive(topic: .syncMapBricks, payload: snapshot(epoch: 1, seq: 0, [brick(6)]), at: 112)
    expect(map.mapState == .current && map.mapEpoch == 1 && map.bricks.count == 1, "new generation's sync snapshot at seq 0 is current")

    // Status ran ahead of the map. A replayed snapshot that is still behind that epoch must not
    // clear the request: the delta after it is the rest of the answer.
    var behind = currentMirror()
    _ = behind.receive(topic: .mapBricks, payload: snapshot(epoch: 8, seq: 10, [brick(8)]), at: 110)
    _ = behind.receive(topic: .status, payload: status(epoch: 10, seq: 2), at: 111)
    let ahead = behind.receive(topic: .status, payload: status(epoch: 10, seq: 3), at: 112)
    expect(behind.mapState == .stale(.epochMismatch) && request(ahead, .mapBricks, "status epoch") != nil,
           "status epoch 10 vs mirror 8: stale and requesting")
    _ = behind.receive(topic: .syncMapBricks, payload: snapshot(epoch: 9, seq: 11, [brick(9)]), at: 113)
    expect(behind.mapState != .current && behind.mapEpoch == 9, "replayed snapshot at epoch 9 is behind status epoch 10")
    _ = behind.receive(topic: .syncMapBricks, payload: delta(epoch: 10, seq: 12, base: 9, [brick(10)]), at: 114)
    expect(behind.mapState == .current && behind.mapEpoch == 10 && behind.mapSeq == 12 && behind.bricks.count == 2,
           "the delta after that snapshot reaches the status epoch and is applied")

    // Connect already requested events. A status that names events, with none held, is an
    // incomplete history from the first one on. The outstanding request must not hide that.
    var waiting = Mirror(roomId: "lab")
    _ = waiting.connectionOpened(at: 0, canRequest: true)
    _ = waiting.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 12), at: 0)
    expect(waiting.eventsState == .incomplete(since: nil), "events 0–12 named, none held: incomplete, got \(waiting.eventsState)")
    _ = waiting.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 12), at: 1)
    expect(waiting.eventsState != .complete, "events advertised twice and none held: incomplete while the connect request is out")
    expect(waiting.activity.gaps == 0, "the connect request is not a second gap")

    // Connect request out, status names 100, then live 101 before the replay: 0–100 are missing.
    var early = Mirror(roomId: "lab")
    _ = early.connectionOpened(at: 0, canRequest: true)
    _ = early.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 100), at: 0)
    _ = early.receive(topic: .events, payload: event(id: 101, seq: 101), at: 1)
    expect(early.eventsState == .incomplete(since: nil), "live 101 ahead of the replay: incomplete, got \(early.eventsState)")
    _ = early.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 2)
    _ = early.tick(at: 2.2)
    expect(early.eventsState != .complete, "a status matching the live baseline does not vouch for 0–100")
    for seq in UInt64(0)...100 {
        _ = early.receive(topic: .syncEvents, payload: event(id: seq, seq: seq, state: .reverted), at: 2.5)
    }
    _ = early.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 101), at: 3)
    _ = early.tick(at: 4.6)
    expect(early.eventsSeq == 101 && early.eventsState == .complete, "the replay covers 0–100: complete, got \(early.eventsState)")

    // Live 101 before the replay, then a reconnect: the baseline exists, but 0–100 still have to come.
    var rejoin = Mirror(roomId: "lab")
    _ = rejoin.connectionOpened(at: 0, canRequest: true)
    _ = rejoin.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 100), at: 0)
    _ = rejoin.receive(topic: .events, payload: event(id: 101, seq: 101), at: 1)
    rejoin.connectionClosed(at: 2)
    let reopened = rejoin.connectionOpened(at: 10, canRequest: true)
    expect(request(reopened, .events, "reconnect before replay")?.since == nil, "reconnect keeps asking for events since NONE")
    _ = rejoin.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 11)
    _ = rejoin.tick(at: 11.5)
    expect(rejoin.eventsState != .complete, "after a reconnect a status matching 101 does not vouch for 0–100")
    for seq in UInt64(0)...100 {
        _ = rejoin.receive(topic: .syncEvents, payload: event(id: seq, seq: seq, state: .reverted), at: 12)
    }
    _ = rejoin.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 101), at: 13)
    _ = rejoin.tick(at: 14.5)
    expect(rejoin.eventsState == .complete, "the replay after the reconnect covers 0–100: complete, got \(rejoin.eventsState)")
    var rejoinBlind = Mirror(roomId: "lab")
    _ = rejoinBlind.connectionOpened(at: 0, canRequest: true)
    _ = rejoinBlind.receive(topic: .events, payload: event(id: 101, seq: 101), at: 1)
    rejoinBlind.connectionClosed(at: 2)
    _ = rejoinBlind.connectionOpened(at: 10, canRequest: false)
    _ = rejoinBlind.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 11)
    expect(rejoinBlind.eventsState != .complete, "reconnect with no request path: 0–100 are lost, incomplete")

    // Live 101 before the replay, which settles after only 0–50: 51–100 are still missing.
    var partial = Mirror(roomId: "lab")
    _ = partial.connectionOpened(at: 0, canRequest: true)
    _ = partial.receive(topic: .events, payload: event(id: 101, seq: 101), at: 1)
    _ = partial.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 101), at: 1)
    for seq in UInt64(0)...50 {
        _ = partial.receive(topic: .syncEvents, payload: event(id: seq, seq: seq, state: .reverted), at: 2)
    }
    _ = partial.tick(at: 4.5)
    expect(partial.eventsState != .complete, "a gapless replay of 0–50 does not cover 0–100: incomplete, got \(partial.eventsState)")
    _ = partial.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 5)
    _ = partial.tick(at: 7.5)
    expect(partial.eventsState != .complete, "a later status matching 101 does not complete it either")

    // Sync 101 first sets the baseline, then a reconnect before the replay settles, then only
    // 0–50: 51–100 are still missing, whichever path started the baseline.
    var syncFirst = Mirror(roomId: "lab")
    _ = syncFirst.connectionOpened(at: 0, canRequest: true)
    _ = syncFirst.receive(topic: .syncEvents, payload: event(id: 101, seq: 101, state: .reverted), at: 1)
    expect(syncFirst.eventsState == .incomplete(since: nil), "sync 101 ahead of 0–100: incomplete, got \(syncFirst.eventsState)")
    syncFirst.connectionClosed(at: 2)
    _ = syncFirst.connectionOpened(at: 10, canRequest: true)
    for seq in UInt64(0)...50 {
        _ = syncFirst.receive(topic: .syncEvents, payload: event(id: seq, seq: seq, state: .reverted), at: 11)
    }
    _ = syncFirst.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 101), at: 11)
    _ = syncFirst.tick(at: 13.5)
    expect(syncFirst.eventsSeq == 101 && syncFirst.eventsState != .complete,
           "sync 101 then a reconnect and only 0–50: incomplete, got \(syncFirst.eventsState)")
    _ = syncFirst.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 14)
    _ = syncFirst.tick(at: 16.5)
    expect(syncFirst.eventsState != .complete, "a later status matching 101 does not complete it either")
    var syncFull = Mirror(roomId: "lab")
    _ = syncFull.connectionOpened(at: 0, canRequest: true)
    _ = syncFull.receive(topic: .syncEvents, payload: event(id: 101, seq: 101, state: .reverted), at: 1)
    syncFull.connectionClosed(at: 2)
    _ = syncFull.connectionOpened(at: 10, canRequest: true)
    for seq in UInt64(0)...101 {
        _ = syncFull.receive(topic: .syncEvents, payload: event(id: seq, seq: seq, state: .reverted), at: 11)
    }
    _ = syncFull.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 101), at: 11)
    _ = syncFull.tick(at: 13.5)
    expect(syncFull.eventsState == .complete, "the replay after the reconnect covers 0–101: complete, got \(syncFull.eventsState)")

    // A status opened the request (the Spark had published nothing before): the baseline the
    // answer starts still has to be covered after a reconnect.
    var statusFirst = Mirror(roomId: "lab")
    _ = statusFirst.connectionOpened(at: 0, canRequest: true)
    _ = statusFirst.receive(topic: .status, payload: status(epoch: 5, seq: 1), at: 0)
    _ = statusFirst.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 101), at: 6)
    let reasked = statusFirst.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 101), at: 7)
    expect(request(reasked, .events, "status names events")?.since == nil, "events named twice, none held: asks since NONE")
    _ = statusFirst.receive(topic: .syncEvents, payload: event(id: 101, seq: 101, state: .reverted), at: 8)
    statusFirst.connectionClosed(at: 9)
    _ = statusFirst.connectionOpened(at: 20, canRequest: true)
    for seq in UInt64(0)...50 {
        _ = statusFirst.receive(topic: .syncEvents, payload: event(id: seq, seq: seq, state: .reverted), at: 21)
    }
    _ = statusFirst.receive(topic: .status, payload: status(epoch: 5, seq: 4, eventsSeq: 101), at: 21)
    _ = statusFirst.tick(at: 23.5)
    expect(statusFirst.eventsSeq == 101 && statusFirst.eventsState != .complete,
           "status-opened request, sync 101, reconnect, only 0–50: incomplete, got \(statusFirst.eventsState)")

    // An unreadable map payload: bricks kept, map stale, resync requested.
    var bad = currentMirror()
    let good = delta(epoch: 6, seq: 11, base: 5, [])
    let truncated = bad.receive(topic: .mapBricks, payload: good.prefix(good.count - 3), at: 110)
    expect(bad.mapState == .stale(.rejected) && bad.bricks.count == 3, "truncated map: stale(rejected), bricks kept")
    expect(request(truncated, .mapBricks, "truncated")?.since == .init(epoch: 5, seq: 10), "truncated map requests since (5, 10)")
}

/// Codex P2: an outstanding events request must not hide missing seqs.
func testLiveRoomOutstandingSyncHidesNothing() {
    var mirror = Mirror(roomId: "lab")
    let opened = mirror.connectionOpened(at: 0, canRequest: true)
    expect(request(opened, .events, "P2 connect")?.since == nil, "connect asks for events since NONE")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 0, seq: 0), at: 0.5)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 2), at: 1)
    expect(mirror.eventsState == .incomplete(since: header(5, 0).stamp), "sync 0 only, status names 2: incomplete, got \(mirror.eventsState)")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 2), at: 2)
    expect(mirror.eventsState != .complete, "a second status with the request out: still incomplete")
    _ = mirror.tick(at: 3)
    expect(mirror.eventsState != .complete, "the answer settled without 1–2: incomplete, got \(mirror.eventsState)")
    let retried = mirror.tick(at: 5.5)
    expect(request(retried, .events, "P2 retry")?.since == .init(epoch: 5, seq: 0), "the retry asks since (5, 0)")
    expect(mirror.eventsState != .complete, "retrying: still incomplete")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 1, seq: 1), at: 6)
    expect(mirror.eventsState != .complete, "1 of 1–2: incomplete")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 2, seq: 2), at: 6)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 2), at: 7)
    expect(mirror.eventsState == .complete, "0–2 held, status 2: complete, got \(mirror.eventsState)")
    expect(request(mirror.tick(at: 60), .events, "P2 resolved") == nil, "events request resolved")
}

/// Codex P2: a replay overtakes the live topic. The delayed live copies of one event's transitions
/// are duplicates, not a restart: the event keeps its newest state and the history completes.
func testLiveRoomDelayedLiveTransitionIsDuplicate() {
    var mirror = Mirror(roomId: "lab")
    let opened = mirror.connectionOpened(at: 100, canRequest: true)
    expect(request(opened, .events, "delayed live connect")?.since == nil, "connect asks for events since NONE")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 1, seq: 0, state: .candidate), at: 101)
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 1, seq: 1, state: .confirmed), at: 101)
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 1, seq: 2, state: .accepted), at: 101)
    expect(mirror.eventsSeq == 2 && mirror.eventsById[1]?.event.state == .accepted, "replay 0–2 applies: accepted")
    let duplicates = mirror.activity.duplicates
    let gaps = mirror.activity.gaps

    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 1, state: .confirmed), at: 101.5)
    expect(mirror.eventsById[1]?.event.state == .accepted, "delayed live confirmed at seq 1 does not roll back, got \(String(describing: mirror.eventsById[1]?.event.state))")
    expect(mirror.eventsSeq == 2, "delayed live seq 1 keeps the baseline at 2")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 2, state: .accepted), at: 101.6)
    expect(mirror.eventsById[1]?.event.state == .accepted && mirror.eventsSeq == 2, "delayed live seq 2: still accepted at 2")
    expect(mirror.activity.duplicates == duplicates + 2 && mirror.activity.gaps == gaps,
           "both delayed copies count as duplicates, no gap: \(mirror.activity.duplicates - duplicates) dup, \(mirror.activity.gaps - gaps) gaps")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 2), at: 102)
    expect(mirror.eventsState == .complete, "0–2 held, status 2: complete, got \(mirror.eventsState)")
    _ = mirror.tick(at: 104)
    expect(request(mirror.tick(at: 200), .events, "delayed live resolved") == nil, "events request resolved")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 3, state: .expired), at: 201)
    expect(mirror.eventsSeq == 3 && mirror.eventsById[1]?.event.state == .expired && mirror.eventsState == .complete,
           "live seq 3 chains on: complete")

    // A seen seq with another transition is no restart either: only status evidence is.
    var reused = mirror
    _ = reused.receive(topic: .events, payload: event(id: 1, seq: 1, state: .reverted), at: 202)
    expect(reused.eventsSeq == 3 && reused.eventsById[1]?.event.state == .expired, "another transition at seq 1: no rollback, baseline stays")
    expect(reused.eventsState == .complete && reused.activity.historyCuts == 0, "another transition at seq 1: no cut")

    // And a status seq reset still starts a new generation.
    var reset = mirror
    _ = reset.receive(topic: .status, payload: status(epoch: 5, seq: 0, eventsSeq: 0), at: 202)
    expect(reset.eventsState != .complete && reset.eventsSeq == nil, "status seq reset: new generation, incomplete")
}

/// Codex P2: another client's replay on the shared sync topic delivers confirmed(1) without
/// candidate(0). The delayed live candidate(0) is older than the event's latest seq: out-of-order
/// delivery fills the coverage, never rolls the event back and never cuts the history.
func testLiveRoomOlderTransitionIsOutOfOrder() {
    var mirror = Mirror(roomId: "lab")
    let opened = mirror.connectionOpened(at: 100, canRequest: true)
    expect(request(opened, .events, "out-of-order connect")?.since == nil, "connect asks for events since NONE")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 1, seq: 1, state: .confirmed), at: 101)
    expect(mirror.eventsSeq == 1 && mirror.eventsById[1]?.event.state == .confirmed, "partial replay: confirmed at 1")
    let gaps = mirror.activity.gaps

    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 0, state: .candidate), at: 101.5)
    expect(mirror.eventsById[1]?.event.state == .confirmed, "delayed live candidate at seq 0 does not roll back, got \(String(describing: mirror.eventsById[1]?.event.state))")
    expect(mirror.eventsSeq == 1, "delayed live seq 0 keeps the baseline at 1")
    expect(mirror.activity.gaps == gaps, "out-of-order delivery is no gap: \(mirror.activity.gaps - gaps) gaps")
    _ = mirror.tick(at: 101 + Mirror.syncSettle + 1)
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 1), at: 104.5)
    expect(mirror.eventsState == .complete, "0–1 held, status 1: complete, got \(mirror.eventsState)")
    expect(request(mirror.tick(at: 200), .events, "out-of-order resolved") == nil, "events request resolved")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 2, state: .accepted), at: 201)
    expect(mirror.eventsSeq == 2 && mirror.eventsById[1]?.event.state == .accepted && mirror.eventsState == .complete,
           "live seq 2 chains on: complete")

    // A status seq reset still starts a new generation.
    var reset = mirror
    _ = reset.receive(topic: .status, payload: status(epoch: 5, seq: 0, eventsSeq: 0), at: 202)
    expect(reset.eventsState != .complete && reset.eventsSeq == nil, "status seq reset: new generation, incomplete")
}

/// Random interleavings of live, sync, status, reconnect and settle against one invariant: the
/// mirror reads complete only when it holds every seq in [0, max(status events_seq, highest held)].
/// Each run ends by reconnecting and replaying everything, which must then read complete.
func testLiveRoomHistoryInvariantProperty() {
    var runsCompleteMidway = 0
    for seed in UInt64(1)...250 {
        var rng = SplitMix64(seed: seed &* 0x9E37_79B9)
        func below(_ n: Int) -> Int { Int(rng.next() % UInt64(n)) }
        func chance(_ p: Double) -> Bool { rng.unit() < p }

        var mirror = Mirror(roomId: "lab")
        var now: TimeInterval = 0
        var published: UInt64 = 0
        var statusSeq: UInt64 = 0
        var statusHigh: UInt64?
        var connected = false
        var canRequest = false
        var eventsRequest: R.RequestSync?
        var lastLive: UInt64?
        var sawComplete = false
        var trace: [String] = []

        func take(_ actions: [Mirror.Action]) {
            for case .requestSync(let sent) in actions where sent.topic == .events {
                eventsRequest = sent
            }
        }
        func check(_ step: String) {
            trace.append(step)
            guard mirror.eventsState == .complete else { return }
            let held = Set(mirror.eventsById.keys)
            guard let top = [statusHigh, held.max()].compactMap({ $0 }).max() else { return }
            if published > 0 { sawComplete = true }
            let missing = (0...top).first { !held.contains($0) }
            expect(missing == nil && connected,
                   "seed \(seed): complete with seq \(missing.map(String.init) ?? "—") missing of 0–\(top), connected \(connected); trace \(trace.suffix(25).joined(separator: " "))")
        }
        func sendStatus(mark: UInt64?) {
            statusSeq += 1
            if let mark { statusHigh = max(statusHigh ?? mark, mark) }
            take(mirror.receive(topic: .status, payload: status(epoch: 5, seq: statusSeq, eventsSeq: mark), at: now))
        }
        func connect(path: Bool) {
            canRequest = path
            connected = true
            take(mirror.connectionOpened(at: now, canRequest: path))
        }
        func sync(_ seq: UInt64) {
            take(mirror.receive(topic: .syncEvents, payload: event(id: seq, seq: seq), at: now))
        }

        connect(path: chance(0.75))
        check("open")
        for _ in 0..<70 {
            now += 0.1
            switch below(11) {
            case 0, 1, 2:
                // Publish one event; it may be lost on the way, or its status may beat it here.
                let seq = published
                published += 1
                guard connected else { check("publish \(seq) unseen"); break }
                if chance(0.25) { sendStatus(mark: seq) }
                if chance(0.8) {
                    take(mirror.receive(topic: .events, payload: event(id: seq, seq: seq), at: now))
                    lastLive = seq
                    check("live \(seq)")
                } else {
                    check("lost \(seq)")
                }
            case 3:
                // The same live message twice.
                guard connected, let seq = lastLive, seq == published - 1 else { break }
                take(mirror.receive(topic: .events, payload: event(id: seq, seq: seq), at: now))
                check("dup \(seq)")
            case 4, 5:
                guard connected else { break }
                sendStatus(mark: published == 0 ? nil : published - 1)
                check("status \(published == 0 ? "none" : String(published - 1))")
            case 6:
                // Answer the latest events request: whole, cut short, with holes, or bounded.
                guard connected, canRequest, let asked = eventsRequest else { break }
                eventsRequest = nil
                let from = asked.since.map { $0.seq + 1 } ?? 0
                guard from < published else { check("answer empty"); break }
                var seqs = Array(from..<published)
                switch below(4) {
                case 0: break
                case 1: seqs = Array(seqs.prefix(below(seqs.count + 1)))
                case 2: seqs = seqs.filter { _ in chance(0.7) }
                default: seqs = Array(seqs.suffix(1 + below(seqs.count)))
                }
                for seq in seqs { sync(seq) }
                check("answer \(from)..<\(published) gave \(seqs.count)")
            case 7:
                // Another Mac's answer on the shared topic.
                guard connected, published > 0 else { break }
                let seq = UInt64(below(Int(published)))
                sync(seq)
                check("stray sync \(seq)")
            case 8:
                now += 2.5
                take(mirror.tick(at: now))
                check("settle")
            case 9:
                now += 6
                take(mirror.tick(at: now))
                check("retry")
            default:
                if connected {
                    mirror.connectionClosed(at: now)
                    connected = false
                    eventsRequest = nil
                    check("close")
                } else {
                    connect(path: chance(0.7))
                    check("open path \(canRequest)")
                }
            }
        }
        if sawComplete { runsCompleteMidway += 1 }

        // Converge: reconnect with a request path, the Spark replays everything, a status, settle.
        if connected { mirror.connectionClosed(at: now) }
        connected = false
        now += 6
        connect(path: true)
        check("final open")
        for seq in 0..<published { sync(seq) }
        sendStatus(mark: published == 0 ? nil : published - 1)
        now += 2.5
        take(mirror.tick(at: now))
        check("final settle")
        expect(mirror.eventsState == .complete,
               "seed \(seed): everything replayed but \(mirror.eventsState); trace \(trace.suffix(25).joined(separator: " "))")
    }
    expect(runsCompleteMidway >= 25, "the property is not vacuous: \(runsCompleteMidway) runs read complete midway")
}

/// Codex P2: a partial replay delivers event 2 at seq 1; the live topic then delivers event 1 at
/// seq 0, late. A lower seq from another event is out-of-order delivery, never a restart: only
/// status evidence cuts the history.
func testLiveRoomLowerSeqFromAnotherEventIsNoRestart() {
    var mirror = Mirror(roomId: "lab")
    let opened = mirror.connectionOpened(at: 100, canRequest: true)
    expect(request(opened, .events, "other event connect")?.since == nil, "connect asks for events since NONE")
    _ = mirror.receive(topic: .syncEvents, payload: event(id: 2, seq: 1, state: .candidate), at: 101)
    expect(mirror.eventsSeq == 1 && mirror.eventsById[2]?.event.state == .candidate, "partial replay: event 2 at 1")

    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 0, state: .candidate), at: 101.5)
    expect(mirror.activity.historyCuts == 0, "a lower seq from another event cuts nothing, got \(mirror.activity.historyCuts) cuts")
    expect(mirror.eventsSeq == 1 && mirror.eventsById[1]?.event.state == .candidate && mirror.eventsById[2]?.event.state == .candidate,
           "delayed live event 1 at seq 0 applies; event 2 and the baseline stay")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 1, eventsSeq: 1), at: 102)
    expect(mirror.eventsState == .complete, "0–1 held, status 1: complete, got \(mirror.eventsState)")
    _ = mirror.tick(at: 101 + Mirror.syncSettle + 1)
    expect(request(mirror.tick(at: 200), .events, "other event resolved") == nil, "events request resolved")
    _ = mirror.receive(topic: .events, payload: event(id: 1, seq: 2, state: .confirmed), at: 201)
    expect(mirror.eventsSeq == 2 && mirror.eventsById[1]?.event.state == .confirmed && mirror.eventsState == .complete,
           "live seq 2 chains on: complete")

    // A status seq reset still starts a new generation.
    var reset = mirror
    _ = reset.receive(topic: .status, payload: status(epoch: 5, seq: 0, eventsSeq: 0), at: 202)
    expect(reset.activity.historyCuts == 1 && reset.eventsState != .complete && reset.eventsSeq == nil,
           "status seq reset: one cut, new generation, incomplete")
}

/// Codex P2: events 0–5 held, then the publisher restarts while the Mac is away and publishes no
/// event. Its status seq has passed the old one, so only `events_seq` NONE tells. NONE is below any
/// baseline: twice in a row it is a restart, once may be lag. The same holds against a mark a status
/// named but no event delivered, and against an earlier status's mark.
func testLiveRoomNoneEventsSeqIsRestartEvidence() {
    var mirror = currentMirror()
    for seq in UInt64(0)...5 { _ = mirror.receive(topic: .events, payload: event(id: seq + 1, seq: seq), at: 101) }
    for seq in UInt64(0)...5 { _ = mirror.receive(topic: .syncEvents, payload: event(id: seq + 1, seq: seq), at: 101) }
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 50, eventsSeq: 5), at: 102)
    _ = mirror.tick(at: 101 + Mirror.syncSettle + 1)
    expect(mirror.eventsState == .complete && request(mirror.tick(at: 110), .events, "none setup") == nil,
           "setup: 0–5 held, complete, nothing owed")
    mirror.connectionClosed(at: 111)
    let opened = mirror.connectionOpened(at: 120, canRequest: true)
    expect(request(opened, .events, "none reconnect")?.since?.seq == 5, "reconnect asks for events since seq 5")

    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 900), at: 121)
    expect(mirror.eventsSeq == 5 && mirror.activity.historyCuts == 0, "one NONE status may be lag: baseline stays")
    // A NONE status does not agree with the baseline: the owed request is not done.
    expect(request(mirror.tick(at: 126), .events, "none retry")?.since?.seq == 5, "after one NONE the events request still retries")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 901), at: 127)
    expect(mirror.eventsSeq == nil && mirror.activity.historyCuts == 1, "two NONE statuses: new generation, one cut")
    expect(mirror.eventsState != .complete, "two NONE statuses: old open events may be outdated, got \(mirror.eventsState)")
    // The new generation published nothing: there is nothing to fetch.
    expect(request(mirror.tick(at: 140), .events, "none restart") == nil, "restart with NONE: no events to request")
    _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: 902), at: 141)
    expect(mirror.activity.historyCuts == 1 && mirror.eventsState != .complete, "further NONE statuses: no second cut, still incomplete")

    // Lag: the status was published before event 0 arrived. One NONE, then agreement: no cut.
    var lag = currentMirror(canRequest: false)
    _ = lag.receive(topic: .events, payload: event(id: 1, seq: 0), at: 101)
    _ = lag.receive(topic: .status, payload: status(epoch: 5, seq: 2), at: 102)
    _ = lag.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 0), at: 103)
    _ = lag.receive(topic: .status, payload: status(epoch: 5, seq: 4), at: 104)
    _ = lag.receive(topic: .status, payload: status(epoch: 5, seq: 5, eventsSeq: 0), at: 105)
    expect(lag.activity.historyCuts == 0 && lag.eventsSeq == 0 && lag.eventsState == .complete,
           "NONE lag statuses between agreeing ones: no cut, complete")

    // NONE then a mark below the baseline: both are low, so two in a row.
    var mixed = currentMirror(canRequest: false)
    for seq in UInt64(0)...5 { _ = mixed.receive(topic: .events, payload: event(id: 1, seq: seq), at: 101) }
    _ = mixed.receive(topic: .status, payload: status(epoch: 5, seq: 900), at: 102)
    _ = mixed.receive(topic: .status, payload: status(epoch: 5, seq: 901, eventsSeq: 1), at: 103)
    expect(mixed.eventsSeq == nil && mixed.activity.historyCuts == 1 && mixed.eventsState != .complete,
           "NONE then a low mark: new generation, incomplete")

    // Named but never held: the old generation's events are unseen for good, even once the new
    // generation reaches the old mark.
    var named = currentMirror(canRequest: false)
    _ = named.receive(topic: .status, payload: status(epoch: 5, seq: 50, eventsSeq: 5), at: 101)
    named.connectionClosed(at: 102)
    _ = named.connectionOpened(at: 120, canRequest: false)
    _ = named.receive(topic: .status, payload: status(epoch: 5, seq: 900), at: 121)
    expect(named.activity.historyCuts == 0, "one NONE after a named mark may be lag")
    _ = named.receive(topic: .status, payload: status(epoch: 5, seq: 901), at: 122)
    expect(named.activity.historyCuts == 1, "two NONE statuses after a named mark: cut")
    for seq in UInt64(0)...5 { _ = named.receive(topic: .events, payload: event(id: 1, seq: seq), at: 123) }
    _ = named.receive(topic: .status, payload: status(epoch: 5, seq: 902, eventsSeq: 5), at: 124)
    expect(named.eventsState != .complete, "new generation 0–5 after the cut stays incomplete, got \(named.eventsState)")

    // A mark below an earlier status's mark, twice, is a restart even when it is above the baseline.
    var fell = currentMirror(canRequest: false)
    for seq in UInt64(0)...3 { _ = fell.receive(topic: .events, payload: event(id: 1, seq: seq), at: 101) }
    _ = fell.receive(topic: .status, payload: status(epoch: 5, seq: 50, eventsSeq: 8), at: 102)
    _ = fell.receive(topic: .status, payload: status(epoch: 5, seq: 51, eventsSeq: 4), at: 103)
    expect(fell.activity.historyCuts == 0, "one fallen mark may be lag")
    _ = fell.receive(topic: .status, payload: status(epoch: 5, seq: 52, eventsSeq: 4), at: 104)
    expect(fell.eventsSeq == nil && fell.activity.historyCuts == 1, "marks 8 then 4, 4: new generation")

    // A cut with no baseline starts the next generation's count afresh: one fallen status after it is lag.
    var unheld = currentMirror(canRequest: false)
    _ = unheld.receive(topic: .status, payload: status(epoch: 5, seq: 2, eventsSeq: 8), at: 101)
    _ = unheld.receive(topic: .status, payload: status(epoch: 5, seq: 3, eventsSeq: 4), at: 102)
    _ = unheld.receive(topic: .status, payload: status(epoch: 5, seq: 4, eventsSeq: 4), at: 103)
    expect(unheld.activity.historyCuts == 1, "named marks 8 then 4, 4 with nothing held: cut")
    _ = unheld.receive(topic: .status, payload: status(epoch: 5, seq: 5, eventsSeq: 3), at: 104)
    expect(unheld.activity.historyCuts == 1, "one fallen mark after the cut may be lag: no second cut")
}

/// Codex P2: every status-based restart resets the map high-water mark with the events state. The
/// live map reached seq 10; the publisher restarts and its startup snapshot is missed. D20 asks for
/// the map, and the new generation's snapshot at seq 0 is current, whichever evidence told.
func testLiveRoomEveryRestartResetsMapHighWater() {
    func check(_ name: String, events: ClosedRange<UInt64>? = nil, before: [Data] = [], restart: [Data]) {
        var mirror = currentMirror()
        if let events {
            for seq in events { _ = mirror.receive(topic: .events, payload: event(id: 1, seq: seq), at: 101) }
        }
        for payload in before { _ = mirror.receive(topic: .status, payload: payload, at: 102) }
        var actions: [Mirror.Action] = []
        for (index, payload) in restart.enumerated() {
            actions += mirror.receive(topic: .status, payload: payload, at: 103 + TimeInterval(index))
        }
        expect(mirror.activity.historyCuts == 1, "\(name): restart detected, got \(mirror.activity.historyCuts) cuts")
        expect(mirror.mapState == .stale(.epochMismatch) && request(actions, .mapBricks, "\(name) D20") != nil,
               "\(name): missed startup snapshot, D20 requests the map, got \(mirror.mapState)")
        _ = mirror.receive(topic: .syncMapBricks, payload: snapshot(epoch: 1, seq: 0, [brick(6)]), at: 110)
        expect(mirror.mapState == .current && mirror.mapEpoch == 1 && mirror.mapSeq == 0,
               "\(name): the new generation's snapshot at seq 0 is current, not stale against live seq 10, got \(mirror.mapState)")
    }

    check("status seq reset", restart: [status(epoch: 1, seq: 0), status(epoch: 1, seq: 1)])
    check("events_seq below the baseline twice", events: 0...5,
          restart: [status(epoch: 1, seq: 900, eventsSeq: 2), status(epoch: 1, seq: 901, eventsSeq: 2)])
    check("NONE twice with a baseline", events: 0...5,
          restart: [status(epoch: 1, seq: 900), status(epoch: 1, seq: 901)])
    check("NONE twice after a mark named but never held", before: [status(epoch: 5, seq: 2, eventsSeq: 5)],
          restart: [status(epoch: 1, seq: 900), status(epoch: 1, seq: 901)])
    check("a mark below an earlier status's mark twice", events: 0...3, before: [status(epoch: 5, seq: 50, eventsSeq: 8)],
          restart: [status(epoch: 1, seq: 51, eventsSeq: 4), status(epoch: 1, seq: 52, eventsSeq: 4)])
}

/// Random interleavings of live, sync and duplicate deliveries over several events, with statuses
/// between them: the history is cut only on a status step that resets the status seq. No event
/// message, at any seq or from any event id, cuts it or rolls an event back.
func testLiveRoomOnlyStatusCutsHistoryProperty() {
    let lifecycle: [R.ChangeEvent.State] = [.candidate, .confirmed, .accepted]
    var lowerFromOtherEvent = 0
    var restartRuns = 0
    for seed in UInt64(1)...300 {
        var rng = SplitMix64(seed: seed &* 0xD1B5_4A32)
        func below(_ n: Int) -> Int { Int(rng.next() % UInt64(n)) }
        func chance(_ p: Double) -> Bool { rng.unit() < p }

        // What the publisher sends, one transition per seq, across a few events at once.
        var plan: [(id: UInt64, state: R.ChangeEvent.State)] = []
        var progress: [UInt64: Int] = [:]
        var nextId: UInt64 = 1
        for _ in 0..<(6 + below(14)) {
            let open = progress.filter { $0.value < lifecycle.count }.keys.sorted()
            let id: UInt64
            if open.count < 2 || chance(0.25) {
                id = nextId
                nextId += 1
            } else {
                id = open[below(open.count)]
            }
            plan.append((id, lifecycle[progress[id, default: 0]]))
            progress[id, default: 0] += 1
        }

        // Every seq arrives live at least once; some also on sync or twice, each displaced in time.
        let spread = Double(1 + below(6))
        var deliveries: [(at: Double, seq: UInt64, sync: Bool)] = []
        for seq in plan.indices {
            deliveries.append((Double(seq) + rng.unit() * spread, UInt64(seq), false))
            if chance(0.6) { deliveries.append((Double(seq) + rng.unit() * spread, UInt64(seq), true)) }
            if chance(0.15) { deliveries.append((Double(seq) + rng.unit() * spread, UInt64(seq), false)) }
        }
        deliveries.sort { $0.at < $1.at }
        let restartAt = chance(0.3) ? below(deliveries.count) : nil

        var mirror = Mirror(roomId: "lab")
        var now: TimeInterval = 0
        _ = mirror.connectionOpened(at: now, canRequest: true)
        var statusSeq: UInt64 = 0
        var restarted = false
        // The restart cut off a history that held or named an event: it never reads complete again.
        var cutHeldSomething = false
        var topDelivered: UInt64?
        var heldSeq: [UInt64: UInt64] = [:]
        var trace: [String] = []

        func sendStatus(reset: Bool) {
            statusSeq = reset ? 0 : statusSeq + 1
            let cuts = mirror.activity.historyCuts
            _ = mirror.receive(topic: .status, payload: status(epoch: 5, seq: statusSeq, eventsSeq: topDelivered), at: now)
            trace.append(reset ? "RESET" : "st\(topDelivered.map(String.init) ?? "-")")
            let expected = cuts + (reset ? 1 : 0)
            expect(mirror.activity.historyCuts == expected,
                   "seed \(seed): status \(reset ? "reset" : "step") made \(mirror.activity.historyCuts - cuts) cuts; trace \(trace.suffix(25).joined(separator: " "))")
        }

        for (index, delivery) in deliveries.enumerated() {
            now += 0.05
            if index == restartAt {
                if statusSeq == 0 { sendStatus(reset: false) }
                cutHeldSomething = topDelivered != nil
                sendStatus(reset: true)
                restarted = true
                restartRuns += 1
            } else if chance(0.3) {
                sendStatus(reset: false)
            }
            let seq = delivery.seq
            let (id, state) = plan[Int(seq)]
            if !delivery.sync, let top = topDelivered, seq < top, plan[Int(top)].id != id {
                lowerFromOtherEvent += 1
            }
            let cuts = mirror.activity.historyCuts
            _ = mirror.receive(topic: delivery.sync ? .syncEvents : .events, payload: event(id: id, seq: seq, state: state), at: now)
            trace.append("\(delivery.sync ? "sy" : "lv")\(seq)#\(id)")
            topDelivered = max(topDelivered ?? seq, seq)
            expect(mirror.activity.historyCuts == cuts,
                   "seed \(seed): an event message made \(mirror.activity.historyCuts - cuts) cuts; trace \(trace.suffix(25).joined(separator: " "))")
            guard !restarted else { continue }
            // Never rolled back: each event holds a seq at least as new as before and as its newest live one.
            for (heldId, held) in mirror.eventsById {
                let floor = max(heldSeq[heldId] ?? 0, delivery.sync || heldId != id ? 0 : seq)
                expect(held.seq >= floor && held.event.state == plan[Int(held.seq)].state,
                       "seed \(seed): event \(heldId) at seq \(held.seq) \(held.event.state), floor \(floor); trace \(trace.suffix(25).joined(separator: " "))")
                heldSeq[heldId] = held.seq
            }
        }
        now += 0.05
        sendStatus(reset: false)

        if restarted {
            expect(mirror.activity.historyCuts == 1 && (mirror.eventsState != .complete) == cutHeldSomething,
                   "seed \(seed): one status reset: \(mirror.activity.historyCuts) cuts, \(mirror.eventsState)")
        } else {
            expect(mirror.activity.historyCuts == 0 && mirror.eventsState == .complete,
                   "seed \(seed): no restart evidence: \(mirror.activity.historyCuts) cuts, \(mirror.eventsState); trace \(trace.suffix(25).joined(separator: " "))")
            for (id, _) in progress {
                let last = plan.lastIndex { $0.id == id }!
                expect(mirror.eventsById[id]?.seq == UInt64(last) && mirror.eventsById[id]?.event.state == plan[last].state,
                       "seed \(seed): event \(id) ends at its newest transition (seq \(last))")
            }
        }
    }
    expect(lowerFromOtherEvent >= 200, "the property is not vacuous: \(lowerFromOtherEvent) lower live seqs from another event")
    expect(restartRuns >= 40, "restart runs exercised: \(restartRuns)")
}

func runLiveRoomTests() {
    testLiveRoomFoxgloveClientFrames()
    print("  LiveRoom: serverInfo capabilities, client advertise + message data, topic/schema match"); passed()
    testLiveRoomGoldensDecode()
    print("  LiveRoom: every hand golden decodes through Foxglove framing; mirror accepts them"); passed()
    testLiveRoomLiveCaptures()
    print("  LiveRoom: real :8767 frames decode (status with events_seq, zlib snapshot) and mirror"); passed()
    testLiveRoomSnapshotDeltaApply()
    print("  LiveRoom: snapshot + delta apply, canonical removals, duplicates, whole-message rejection, replay heal"); passed()
    testLiveRoomGapRequestsSync()
    print("  LiveRoom: seq gap / wrong base → STALE + RequestSync(since), 5/10/20 s repeats, sync gated on a request"); passed()
    testLiveRoomEpochChange()
    print("  LiveRoom: status epoch ≠ mirror ×2 → resync; delta/status races stay current; restart snapshot resets"); passed()
    testLiveRoomReconnect()
    print("  LiveRoom: close → STALE, reconnect requests both topics, delta-first stays stale, 3 s status freshness"); passed()
    testLiveRoomEvents()
    print("  LiveRoom: events ring, duplicates, gap → incomplete + RequestSync, replay closes it, live seq drop is no restart, status reset is"); passed()
    testLiveRoomEventsSeqGap()
    print("  LiveRoom: status events_seq unreached on the next status → RequestSync(events); replay to the mark"); passed()
    testLiveRoomReplayAndFallbackEdges()
    print("  LiveRoom: replayed snapshot behind live stays stale until chained; initial + bounded fallbacks settle; lost first event"); passed()
    testLiveRoomGenerationsAndPartialHistory()
    print("  LiveRoom: partial first history, replay behind the status epoch, incomplete while a connect request is out, unreadable map → resync"); passed()
    testLiveRoomOutstandingSyncHidesNothing()
    print("  LiveRoom: an outstanding events request hides no missing seq; settle and retry stay incomplete until 1–2 arrive"); passed()
    testLiveRoomDelayedLiveTransitionIsDuplicate()
    print("  LiveRoom: a replay overtaking live delivery: delayed live transitions are duplicates, never roll back; a reused seq cuts nothing, a status reset still restarts"); passed()
    testLiveRoomOlderTransitionIsOutOfOrder()
    print("  LiveRoom: a partial replay then an older live transition: out-of-order, no rollback, no cut; status reset still restarts"); passed()
    testLiveRoomHistoryInvariantProperty()
    print("  LiveRoom: random live/sync/status/reconnect/settle runs never read complete with a seq missing; full replay converges"); passed()
    testLiveRoomLowerSeqFromAnotherEventIsNoRestart()
    print("  LiveRoom: a partial replay then a lower live seq from another event: out-of-order, no cut; complete once a status confirms"); passed()
    testLiveRoomOnlyStatusCutsHistoryProperty()
    print("  LiveRoom: random multi-event live/sync/duplicate interleavings: only a status reset cuts the history, no event rolls back"); passed()
    testLiveRoomNoneEventsSeqIsRestartEvidence()
    print("  LiveRoom: events_seq NONE (or below what the generation reached) on two statuses in a row is a restart; once is lag"); passed()
    testLiveRoomEveryRestartResetsMapHighWater()
    print("  LiveRoom: every status restart path (seq reset, low mark, NONE, fallen mark) resets the map high-water: a seq 0 snapshot is current"); passed()
}

/// `swift run LabCommandCoreTestsRunner live-room-probe [seconds] [ws://host:port]`: connects a read-only
/// `LiveRoomClient` (never advertises or publishes), runs for a while, and prints what the mirror holds.
/// Exits non-zero when no status arrived.
@MainActor
func runLiveRoomProbe(arguments: [String]) -> Int32 {
    let seconds = arguments.first.flatMap(Double.init) ?? 15
    let url = arguments.dropFirst().first.flatMap(URL.init(string:)) ?? LiveRoomClient.defaultURL
    let client = LiveRoomClient(url: url, publishRequests: false)
    client.start()
    let deadline = Date().addingTimeInterval(seconds)
    while Date() < deadline {
        RunLoop.main.run(until: Date().addingTimeInterval(0.1))
    }
    let mirror = client.mirror
    client.stop()
    RunLoop.main.run(until: Date().addingTimeInterval(0.2))

    for line in client.log { print("  log: \(line)") }
    print("connection: \(client.connection)")
    for (topic, activity) in mirror.activity.topics.sorted(by: { $0.key < $1.key }) {
        print("  \(topic): \(activity.count) messages")
    }
    let a = mirror.activity
    print("rejected \(a.rejected), duplicates \(a.duplicates), gaps \(a.gaps), foreign \(a.foreignRoom), requests sent \(a.requestsSent) unsent \(a.requestsUnsent)")
    print("room \(mirror.roomId ?? "—"), map \(mirror.mapState), epoch \(mirror.mapEpoch.map(String.init) ?? "—"), seq \(mirror.mapSeq.map(String.init) ?? "—"), bricks \(mirror.bricks.count), voxel \(mirror.voxelSize.map { "\($0) m" } ?? "—")")
    print("events: \(mirror.events.count) held, \(mirror.openEvents.count) open, baseline \(mirror.eventsSeq.map(String.init) ?? "—"), \(mirror.eventsState)")
    guard let status = mirror.status else {
        print("FAIL: no MemoryStatus arrived in \(Int(seconds)) s")
        return 1
    }
    print("status: \(status.state), epoch \(status.mapEpoch), bricks \(status.brickCount), open events \(status.openEventCount), coverage \(status.coverageKnownFraction.map { String(format: "%.3f", $0) } ?? "—"), events_seq \(status.eventsSeq.map(String.init) ?? "none"), Hz mid360 \(status.mid360Hz.map(String.init) ?? "—") jt128 \(status.jt128Hz.map(String.init) ?? "—")")
    print("PASS live-room-probe: status arrived")
    return 0
}
