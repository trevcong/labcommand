import Foundation
import LabCommandCore

// MARK: - Live Room view: status line, voxels, markers, banners, fixtures
// These cases drive RoomMemoryMirror's public API and the pure screen. They open no socket.

private func viewRepoRoot() -> URL {
    URL(fileURLWithPath: #filePath)
        .deletingLastPathComponent()
        .deletingLastPathComponent()
        .deletingLastPathComponent()
}

private func viewServer(
    topics: [String] = ["/room_memory/map_bricks", "/room_memory/events", "/room_memory/status"],
    canPublish: Bool = false
) -> LiveRoomConnection {
    .connected(LiveRoomServer(name: "view-test", capabilities: [], canPublish: canPublish, memoryTopics: topics))
}

private func viewCDR(_ encode: () throws -> Data, _ what: String) -> Data {
    do { return try encode() } catch {
        expect(false, "LiveRoom view \(what) encodes: \(error)")
        return Data()
    }
}

private func viewHeader(
    _ epoch: UInt64,
    _ seq: UInt64,
    room: String = "lab",
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1_760_400_010, nanosec: 0)
) -> RoomMemory.Header {
    RoomMemory.Header(roomId: room, mapEpoch: epoch, seq: seq, stamp: stamp)
}

private func viewBrick() -> RoomMemory.Brick {
    var known = [UInt8](repeating: 0, count: RoomMemory.Brick.maskBytes)
    var occupied = known
    known[0] = 1
    occupied[0] = 1
    return RoomMemory.Brick(
        coordinate: RoomMemory.BrickCoordinate(i: 0, j: 0, k: 0),
        occupied: occupied,
        known: known
    )
}

private func viewSnapshot(
    epoch: UInt64,
    seq: UInt64 = 1,
    room: String = "lab",
    voxel: Float = 0.05,
    bricks: [RoomMemory.Brick] = []
) -> Data {
    let message = RoomMemory.MapBricks(
        header: viewHeader(epoch, seq, room: room),
        kind: .snapshot,
        baseEpoch: 0,
        voxelSize: voxel,
        bricks: bricks
    )
    return viewCDR(message.encodedCDR, "snapshot \(epoch)")
}

private func viewStatus(
    epoch: UInt64,
    seq: UInt64,
    state: RoomMemory.MemoryState = .tracking,
    coverage: Float? = nil,
    eventsSeq: UInt64? = nil,
    room: String = "lab",
    residual: Float? = nil,
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1, nanosec: 0)
) -> Data {
    let message = RoomMemory.MemoryStatus(
        header: viewHeader(epoch, seq, room: room, stamp: stamp),
        state: state,
        brickCount: 0,
        openEventCount: 0,
        coverageKnownFraction: coverage,
        poseResidualM: residual,
        windowS: nil,
        mid360Hz: nil,
        jt128Hz: nil,
        droppedWindows: 0,
        eventsSeq: eventsSeq
    )
    return viewCDR(message.encodedCDR, "status \(seq)")
}

private func viewEvent(
    id: UInt64,
    seq: UInt64,
    state: RoomMemory.ChangeEvent.State,
    epoch: UInt64 = 43,
    kind: RoomMemory.ChangeEvent.Kind = .appeared,
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1_760_400_010, nanosec: 0),
    evidence: Float? = nil,
    alpha: Float? = nil
) -> Data {
    let message = RoomMemory.ChangeEvent(
        header: viewHeader(epoch, seq, stamp: stamp),
        eventId: id,
        kind: kind,
        state: state,
        centroid: SIMD3(1.25, 0.25, 0.25),
        aabb: RoomMemory.ChangeEvent.Box(min: SIMD3(1, 0, 0), max: SIMD3(1.5, 0.5, 0.5)),
        fromCentroid: nil,
        voxelCount: 8,
        volumeM3: 0.125,
        evidence: evidence,
        alpha: alpha,
        sensors: [],
        firstSeen: stamp,
        lastSeen: stamp,
        mask: nil
    )
    return viewCDR(message.encodedCDR, "event \(id) s\(seq)")
}

private func viewOpened(at now: TimeInterval = 100, canRequest: Bool = false) -> RoomMemoryMirror {
    var mirror = RoomMemoryMirror()
    _ = mirror.connectionOpened(at: now, canRequest: canRequest)
    return mirror
}

private func viewFeed(_ mirror: inout RoomMemoryMirror, _ topic: LiveRoomTopic, _ payload: Data, at now: TimeInterval) {
    _ = mirror.receive(topic: topic, payload: payload, at: now)
}

private func viewScreen(
    _ mirror: RoomMemoryMirror,
    connection: LiveRoomConnection? = nil,
    path: Bool = true,
    started: Bool = true,
    now: TimeInterval = 100,
    kinds: Set<RoomMemory.ChangeEvent.Kind> = Set(RoomMemory.ChangeEvent.Kind.allCases),
    selection: String? = nil
) -> LiveRoomScreen {
    LiveRoomScreen.make(
        mirror: mirror,
        connection: connection ?? viewServer(),
        requestPathExists: path,
        started: started,
        now: now,
        kinds: kinds,
        selection: selection
    )
}

private func viewNear(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> Bool {
    abs(a.x - b.x) < 1e-4 && abs(a.y - b.y) < 1e-4 && abs(a.z - b.z) < 1e-4
}

private func viewLoad(_ mode: LiveRoomFixture.Mode, now: TimeInterval) -> LiveRoomFixture? {
    do {
        return try LiveRoomFixture.load(mode: mode, root: viewRepoRoot(), now: now)
    } catch {
        expect(false, "fixture \(mode.rawValue) loads: \(error)")
        return nil
    }
}

func testLiveRoomStatusLine() {
    let bytes: Data
    do {
        bytes = try LiveRoomFixture.goldenBytes(name: "memory_status_le", root: viewRepoRoot())
    } catch {
        expect(false, "memory_status_le.hex loads: \(error)")
        return
    }
    let decoded = try? RoomMemory.MemoryStatus.decode(cdr: bytes)
    expect(decoded?.eventsSeq == 12, "memory_status_le eventsSeq is 12, got \(String(describing: decoded?.eventsSeq))")

    var golden = viewOpened()
    viewFeed(&golden, .status, bytes, at: 10)
    let goldenLine = LiveRoomScreen.statusLine(mirror: golden, connection: viewServer(), now: 10)
    expect(goldenLine.contains("events 12"), "golden status line contains events 12: \(goldenLine)")

    var bare = viewOpened()
    viewFeed(&bare, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 10)
    let nilLine = LiveRoomScreen.statusLine(mirror: bare, connection: viewServer(), now: 10)
    expect(nilLine.contains("events —"), "nil eventsSeq renders events —: \(nilLine)")

    var baseline = viewOpened(canRequest: false)
    viewFeed(&baseline, .events, viewEvent(id: 1, seq: 4, state: .candidate), at: 10)
    viewFeed(&baseline, .status, viewStatus(epoch: 43, seq: 2, eventsSeq: 12), at: 10)
    expect(baseline.eventsSeq == 4, "baseline stays at the first event seq 4, got \(String(describing: baseline.eventsSeq))")
    expect(baseline.status?.eventsSeq == 12, "status high-water is 12")
    let baseLine = LiveRoomScreen.statusLine(mirror: baseline, connection: viewServer(), now: 10)
    expect(baseLine.contains("events 12"), "bar reads status.eventsSeq 12, not the baseline: \(baseLine)")

    var cover = viewOpened()
    viewFeed(&cover, .mapBricks, viewSnapshot(epoch: 7, voxel: 0.05), at: 10)
    viewFeed(&cover, .status, viewStatus(epoch: 7, seq: 1, coverage: 0.75), at: 10)
    let coverLine = LiveRoomScreen.statusLine(mirror: cover, connection: viewServer(), now: 10)
    expect(coverLine.contains("coverage 75.0%"), "coverage 0.75 → 75.0%: \(coverLine)")
    expect(coverLine.contains("cell 5.0 cm"), "voxel 0.05 → cell 5.0 cm: \(coverLine)")
    expect(coverLine.contains("Tracking"), "fresh tracking names the state: \(coverLine)")

    var zero = viewOpened()
    viewFeed(&zero, .status, viewStatus(epoch: 7, seq: 1, coverage: 0), at: 10)
    let zeroLine = LiveRoomScreen.statusLine(mirror: zero, connection: viewServer(), now: 10)
    expect(zeroLine.contains("coverage 0.0%"), "coverage 0 → 0.0%: \(zeroLine)")

    var stale = viewOpened(at: 0)
    viewFeed(&stale, .mapBricks, viewSnapshot(epoch: 7), at: 0)
    viewFeed(&stale, .status, viewStatus(epoch: 7, seq: 1, coverage: 0.5), at: 0)
    _ = stale.tick(at: 4.2)
    expect(!stale.isStatusFresh, "4.2 s past receipt is not fresh")
    let staleLine = LiveRoomScreen.statusLine(mirror: stale, connection: viewServer(), now: 4.2)
    expect(staleLine.hasPrefix("STALE · 4s ·"), "age 4.2 renders STALE · 4s: \(staleLine)")
    expect(!staleLine.contains("Tracking"), "a stale status drops the state word: \(staleLine)")
}

func testLiveRoomEventRowsAndIncomplete() {
    var mirror = viewOpened()
    viewFeed(&mirror, .mapBricks, viewSnapshot(epoch: 43), at: 10)
    viewFeed(&mirror, .status, viewStatus(epoch: 43, seq: 1), at: 10)
    viewFeed(&mirror, .events, viewEvent(id: 9, seq: 1, state: .candidate), at: 10)
    let row = viewScreen(mirror, path: true).openRows.first
    expect(row?.line2.contains("uncalibrated") == true, "nil alpha is uncalibrated: \(row?.line2 ?? "")")
    expect(row?.line2.contains("evidence —") == true, "nil evidence is evidence —: \(row?.line2 ?? "")")

    var done = viewOpened()
    viewFeed(&done, .mapBricks, viewSnapshot(epoch: 1), at: 10)
    viewFeed(&done, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 10)
    expect(done.eventsState == .complete, "no events is a complete history")
    expect(viewScreen(done, path: true).incompleteLine == nil, "complete history prints no incomplete line")

    var gap = viewOpened(canRequest: true)
    viewFeed(&gap, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 10)
    viewFeed(&gap, .events, viewEvent(id: 3, seq: 5, state: .candidate), at: 10)
    expect(gap.eventsState == .incomplete(since: nil), "first seq after a cleared initial sync is incomplete(since: nil), got \(gap.eventsState)")
    let gapLine = viewScreen(gap, path: true).incompleteLine
    expect(gapLine == "incomplete", "nil since prints incomplete, got \(String(describing: gapLine))")

    let when = RoomMemory.Time(sec: 1_760_400_010, nanosec: 0)
    var stamped = viewOpened(canRequest: false)
    viewFeed(&stamped, .events, viewEvent(id: 4, seq: 0, state: .candidate, stamp: when), at: 10)
    viewFeed(&stamped, .events, viewEvent(id: 4, seq: 2, state: .confirmed, stamp: RoomMemory.Time(sec: 1_760_400_020, nanosec: 0)), at: 10)
    expect(stamped.eventsState == .incomplete(since: when), "no request path: since the last message before the hole, got \(stamped.eventsState)")
    let stampedLine = viewScreen(stamped, path: true).incompleteLine
    expect(stampedLine == "incomplete since 00:00:10", "stamp renders UTC, got \(String(describing: stampedLine))")

    let zero = RoomMemory.Time(sec: 0, nanosec: 0)
    var zeroTime = viewOpened(canRequest: false)
    viewFeed(&zeroTime, .events, viewEvent(id: 5, seq: 0, state: .candidate, stamp: zero), at: 10)
    viewFeed(&zeroTime, .events, viewEvent(id: 5, seq: 2, state: .confirmed), at: 10)
    let zeroLine = viewScreen(zeroTime, path: true).incompleteLine
    expect(zeroLine == "incomplete since time —", "a zero stamp is time —, got \(String(describing: zeroLine))")
}

func testLiveRoomVoxels() {
    let bytes: Data
    do {
        bytes = try LiveRoomFixture.goldenBytes(name: "map_bricks_snapshot_zlib_le", root: viewRepoRoot())
    } catch {
        expect(false, "zlib snapshot golden loads: \(error)")
        return
    }
    var mirror = viewOpened()
    viewFeed(&mirror, .mapBricks, bytes, at: 1)
    let map = LiveRoomVoxels.extract(mirror)
    expect(map.points.count == 2, "two occupied centers, got \(map.points.count)")
    expect(map.frames.count == 1, "one frame; the all-free brick is silent, got \(map.frames.count)")
    let origin = SIMD3<Float>(-0.375, 0.825, -1.175)
    let far = SIMD3<Float>(-0.025, 1.175, -0.825)
    expect(map.points.contains { viewNear($0, origin) }, "local (0,0,0) center \(origin), points \(map.points)")
    expect(map.points.contains { viewNear($0, far) }, "local (7,7,7) center \(far), points \(map.points)")
    let corner = SIMD3<Float>(-0.4, 0.8, -1.2)
    expect(map.frames.first?.contains { viewNear($0, corner) } == true, "frame includes \(corner)")
    let fitted = LiveRoomVoxels.fitPoints(map: map, markers: [])
    expect(fitted.contains { viewNear($0, origin) }, "the fit includes an occupied center")
    expect(fitted.contains { viewNear($0, corner) }, "the fit includes the brick frame, not only the cells inside it")
}

func testLiveRoomMarkerLifecycle() {
    var mirror = viewOpened(at: 50)
    viewFeed(&mirror, .mapBricks, viewSnapshot(epoch: 43, seq: 1), at: 50)
    viewFeed(&mirror, .status, viewStatus(epoch: 43, seq: 1), at: 50)
    expect(mirror.isMapCurrent, "epoch 43 snapshot plus a fresh tracking status is current")

    viewFeed(&mirror, .events, viewEvent(id: 7, seq: 1, state: .candidate, epoch: 43), at: 50)
    expect(mirror.events.count == 1, "the ring holds the candidate")
    let dashed = LiveRoomMarkers.draw(mirror: mirror)
    expect(dashed.count == 1 && dashed[0].dashed && dashed[0].fillAlpha == 0, "candidate is one dashed unfilled box")
    let selected = LiveRoomMarkers.draw(mirror: mirror, selection: "7")
    expect(selected.count == 1 && selected[0].dashed && selected[0].fillAlpha == 0, "a selected candidate stays dashed and unfilled")

    viewFeed(&mirror, .events, viewEvent(id: 7, seq: 2, state: .confirmed, epoch: 43), at: 50)
    expect(mirror.events.count == 2, "the ring keeps both transitions")
    let filled = LiveRoomMarkers.draw(mirror: mirror)
    expect(
        filled.count == 1 && filled[0].countsAsChange && filled[0].fillAlpha == LiveRoomMarkers.fillAlpha && !filled[0].dashed,
        "confirmed is one filled marker while the ring holds two"
    )
    let step3 = mirror

    viewFeed(&mirror, .events, viewEvent(id: 7, seq: 3, state: .accepted, epoch: 44), at: 50)
    let waiting = LiveRoomMarkers.draw(mirror: mirror)
    expect(
        waiting.count == 1 && waiting[0].dashed && waiting[0].fillAlpha == 0 && !waiting[0].countsAsChange,
        "accepted ahead of the map is one uncounted outline"
    )
    let waitingScreen = viewScreen(mirror, path: true, now: 50)
    expect(waitingScreen.caption == "0 changes", "the outline is not counted: \(waitingScreen.caption)")
    expect(
        waitingScreen.historyRows.contains { $0.line2.contains("accepted · map updating") },
        "history says the map is updating: \(waitingScreen.historyRows)"
    )

    viewFeed(&mirror, .mapBricks, viewSnapshot(epoch: 44, seq: 2), at: 50)
    expect(LiveRoomMarkers.draw(mirror: mirror).isEmpty, "epoch 44 catches the accepted event")
    expect(LiveRoomMarkers.draw(mirror: step3).count == 1, "the step-3 mirror copy still has its one marker")

    var confirmed = viewOpened(at: 50)
    viewFeed(&confirmed, .mapBricks, viewSnapshot(epoch: 43, seq: 1, bricks: [viewBrick()]), at: 50)
    viewFeed(&confirmed, .status, viewStatus(epoch: 43, seq: 1), at: 50)
    viewFeed(&confirmed, .events, viewEvent(id: 8, seq: 1, state: .confirmed, epoch: 43), at: 50)
    expect(LiveRoomMarkers.draw(mirror: confirmed).count == 1, "setup: one confirmed box")

    var building = confirmed
    viewFeed(&building, .status, viewStatus(epoch: 43, seq: 2, state: .building), at: 50)
    expect(LiveRoomMarkers.draw(mirror: building).isEmpty, "building draws no box")

    var moved = confirmed
    viewFeed(&moved, .status, viewStatus(epoch: 43, seq: 2, state: .rigMoved, residual: 0.12), at: 50)
    expect(LiveRoomMarkers.draw(mirror: moved).isEmpty, "rig moved draws no box")

    var aged = confirmed
    _ = aged.tick(at: 54)
    expect(LiveRoomMarkers.draw(mirror: aged).isEmpty, "a stale status draws no box")

    var lagged = confirmed
    viewFeed(&lagged, .status, viewStatus(epoch: 99, seq: 2), at: 50)
    viewFeed(&lagged, .status, viewStatus(epoch: 99, seq: 3, stamp: RoomMemory.Time(sec: 2, nanosec: 0)), at: 50)
    if case .stale = lagged.mapState {
    } else {
        expect(false, "two epoch mismatches mark the map stale, got \(lagged.mapState)")
    }
    expect(LiveRoomMarkers.draw(mirror: lagged).isEmpty, "a stale map draws no box")

    var degraded = confirmed
    viewFeed(&degraded, .status, viewStatus(epoch: 43, seq: 2, state: .degraded), at: 50)
    let kept = LiveRoomMarkers.draw(mirror: degraded)
    expect(kept.count == 1 && kept[0].countsAsChange, "degraded keeps the confirmed box")

    guard let fixture = viewLoad(.map, now: 200) else { return }
    let full = viewScreen(fixture.mirror, connection: fixture.connection, path: false, started: false, now: 200)
    expect(full.caption == "2 changes · 1 candidate on map", "map fixture caption, got \(full.caption)")
    let hidden = viewScreen(
        fixture.mirror,
        connection: fixture.connection,
        path: false,
        started: false,
        now: 200,
        kinds: [.disappeared, .moved]
    )
    expect(!hidden.markers.contains { $0.kind == .appeared }, "turning appeared off removes it from the marker set")
    expect(hidden.caption == "2 changes · 0 candidates on map", "the caption drops the filtered candidate: \(hidden.caption)")
}

func testLiveRoomBannersAndResync() {
    var rig = viewOpened(at: 0)
    viewFeed(&rig, .mapBricks, viewSnapshot(epoch: 43, bricks: [viewBrick()]), at: 0)
    viewFeed(&rig, .status, viewStatus(epoch: 43, seq: 1, state: .rigMoved, residual: 0.12), at: 0)
    _ = rig.tick(at: 4.2)
    let staleRig = viewScreen(rig, now: 4.2)
    expect(staleRig.banner?.text == "STALE · 4s", "stale status wins over a remembered rig move, got \(String(describing: staleRig.banner?.text))")

    let emptyTopics = viewScreen(RoomMemoryMirror(), connection: viewServer(topics: []), path: false, started: true, now: 1)
    expect(emptyTopics.banner?.text == "Room memory not running on Spark", "empty topics win, got \(String(describing: emptyTopics.banner?.text))")
    expect(emptyTopics.canvasSentence == "Room memory not running on Spark", "the canvas is that sentence")

    var staleMap = viewOpened(at: 20)
    viewFeed(&staleMap, .mapBricks, viewSnapshot(epoch: 43, seq: 1, bricks: [viewBrick()]), at: 20)
    viewFeed(&staleMap, .status, viewStatus(epoch: 43, seq: 1), at: 20)
    viewFeed(&staleMap, .status, viewStatus(epoch: 80, seq: 2), at: 20)
    viewFeed(&staleMap, .status, viewStatus(epoch: 80, seq: 3, stamp: RoomMemory.Time(sec: 3, nanosec: 0)), at: 20)
    expect(staleMap.eventsState == .complete, "no events leaves history complete")
    let resync = viewScreen(staleMap, path: false, now: 20)
    expect(
        resync.banner?.text == "resyncing · as of epoch 43 · cannot request resync",
        "the resync banner carries the phrase, got \(String(describing: resync.banner?.text))"
    )
    expect(resync.standaloneResync == nil, "that phrase is not also a standalone line")

    let lost = viewScreen(staleMap, connection: .waiting(retryIn: 5, reason: "down"), path: false, now: 20)
    expect(lost.banner?.text == "connection lost", "connection lost stays the banner, got \(String(describing: lost.banner?.text))")
    expect(lost.standaloneResync == "cannot request resync", "complete history stands the phrase alone, got \(String(describing: lost.standaloneResync))")

    var history = viewOpened(at: 20, canRequest: true)
    viewFeed(&history, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 20)
    viewFeed(&history, .events, viewEvent(id: 1, seq: 4, state: .candidate), at: 20)
    expect(history.eventsState == .incomplete(since: nil), "got \(history.eventsState)")
    let suffixed = viewScreen(history, connection: .waiting(retryIn: 5, reason: "down"), path: false, now: 20)
    expect(suffixed.banner?.text == "connection lost", "the banner stays connection lost")
    expect(
        suffixed.incompleteLine == "incomplete · cannot request resync",
        "the incomplete line takes the phrase, got \(String(describing: suffixed.incompleteLine))"
    )
    expect(suffixed.standaloneResync == nil, "no standalone line when the incomplete line has the phrase")

    var unsent = viewOpened(at: 30)
    viewFeed(&unsent, .mapBricks, viewSnapshot(epoch: 43, seq: 1, bricks: [viewBrick()]), at: 30)
    viewFeed(&unsent, .status, viewStatus(epoch: 43, seq: 1), at: 30)
    unsent.noteRequestUnsent()
    unsent.noteRequestUnsent()
    let unsentScreen = viewScreen(unsent, path: true, now: 30)
    expect(unsent.isMapCurrent && unsent.eventsState == .complete, "unsent count on a current, complete mirror")
    expect(unsentScreen.banner == nil, "fresh tracking has no banner, got \(String(describing: unsentScreen.banner))")
    expect(
        unsentScreen.standaloneResync == "cannot request resync · 2 unsent",
        "got \(String(describing: unsentScreen.standaloneResync))"
    )

    var empty = viewOpened(at: 40)
    viewFeed(&empty, .mapBricks, viewSnapshot(epoch: 43, seq: 1), at: 40)
    viewFeed(&empty, .status, viewStatus(epoch: 90, seq: 2), at: 40)
    viewFeed(&empty, .status, viewStatus(epoch: 90, seq: 3, stamp: RoomMemory.Time(sec: 4, nanosec: 0)), at: 40)
    let waitingMap = viewScreen(empty, path: false, now: 40)
    expect(waitingMap.banner?.text == "Waiting for map", "an empty stale map waits, got \(String(describing: waitingMap.banner?.text))")
    expect(
        waitingMap.standaloneResync == "cannot request resync",
        "Waiting for map leaves the phrase standalone, got \(String(describing: waitingMap.standaloneResync))"
    )
}

func testLiveRoomLaunchAndFixtures() {
    let unset = LiveRoomLaunch.parse([:])
    expect(unset == .live(opensRoom: false), "an unset variable stays on the live socket without opening Room")
    expect(unset.startsClient && !unset.opensRoom && !unset.skipsProbes && unset.lidarAvailable, "unset launch flags")
    expect(unset.fixtureMode == nil, "unset is not a fixture")

    let live = LiveRoomLaunch.parse(["LABCOMMAND_LIVE_ROOM": "live"])
    expect(live == .live(opensRoom: true), "live opens Room")
    expect(live.startsClient && live.opensRoom && live.skipsProbes && !live.lidarAvailable, "live skips probes and the lidar socket")

    for mode in LiveRoomFixture.Mode.allCases {
        let launch = LiveRoomLaunch.parse(["LABCOMMAND_LIVE_ROOM": mode.rawValue])
        expect(launch == .fixture(mode), "\(mode.rawValue) is that fixture")
        expect(!launch.startsClient, "\(mode.rawValue) does not construct a client")
        expect(launch.opensRoom && launch.skipsProbes && !launch.lidarAvailable, "\(mode.rawValue) launch flags")
    }
    let other = LiveRoomLaunch.parse(["LABCOMMAND_LIVE_ROOM": "nope"])
    expect(other == .fixture(.unavailable) && !other.startsClient, "an unknown value is the Off fixture")

    let now: TimeInterval = 1_000
    guard let map = viewLoad(.map, now: now),
          let status = viewLoad(.status, now: now),
          let stale = viewLoad(.stale, now: now),
          let off = viewLoad(.unavailable, now: now) else { return }

    expect(map.mirror.roomId != status.mirror.roomId, "map and status fixtures do not share a room")
    expect(status.mirror.status?.eventsSeq == 12, "status fixture eventsSeq is 12")
    expect(map.mirror.status?.eventsSeq == nil, "map fixture leaves eventsSeq nil")

    let mapScreen = viewScreen(map.mirror, connection: map.connection, path: false, started: false, now: now)
    expect(
        mapScreen.statusLine == "epoch 43 · coverage — · events — · Tracking · cell 5.0 cm",
        "map status line, got \(mapScreen.statusLine)"
    )
    expect(mapScreen.pip == .good, "map pip is good, got \(mapScreen.pip)")

    let statusScreen = viewScreen(status.mirror, connection: status.connection, path: false, started: false, now: now)
    expect(
        statusScreen.statusLine == "epoch 42 · coverage 75.0% · events 12 · Tracking",
        "status line, got \(statusScreen.statusLine)"
    )
    expect(statusScreen.banner?.text == "Waiting for map", "status banner, got \(String(describing: statusScreen.banner?.text))")

    let staleScreen = viewScreen(stale.mirror, connection: stale.connection, path: false, started: false, now: now)
    expect(staleScreen.banner?.text == "STALE · 4s", "stale banner, got \(String(describing: staleScreen.banner?.text))")
    expect(staleScreen.statusLine.hasPrefix("STALE · 4s ·"), "stale prefix, got \(staleScreen.statusLine)")
    expect(!staleScreen.statusLine.contains("Tracking"), "stale drops the state word: \(staleScreen.statusLine)")
    expect(staleScreen.markers.isEmpty, "stale draws no markers")
    expect(staleScreen.pip == .warn, "stale pip is warn, got \(staleScreen.pip)")

    let offScreen = viewScreen(off.mirror, connection: off.connection, path: false, started: false, now: now)
    expect(offScreen.canvasSentence == "Room memory unavailable", "off canvas, got \(String(describing: offScreen.canvasSentence))")
    expect(offScreen.pip == .idle, "off pip is idle")
    expect(offScreen.caption == "Room memory unavailable", "off caption, got \(offScreen.caption)")
    expect(offScreen.banner == nil, "a socket that never started is not Waiting for map")
}

func runLiveRoomViewTests() {
    testLiveRoomStatusLine()
    print("  LiveRoom view: status line reads status.eventsSeq, coverage, stale age, cell"); passed()
    testLiveRoomEventRowsAndIncomplete()
    print("  LiveRoom view: row words and incomplete-history lines"); passed()
    testLiveRoomVoxels()
    print("  LiveRoom view: zlib snapshot centers and one brick frame"); passed()
    testLiveRoomMarkerLifecycle()
    print("  LiveRoom view: candidate, confirmed, accepted outline, suppression, filter"); passed()
    testLiveRoomBannersAndResync()
    print("  LiveRoom view: banner priority and resync placement"); passed()
    testLiveRoomLaunchAndFixtures()
    print("  LiveRoom view: launch env table and offline fixtures"); passed()
}
