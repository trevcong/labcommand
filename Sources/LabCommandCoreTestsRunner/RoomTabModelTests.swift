import Foundation
import LabCommandCore
import simd

// MARK: - Room tab model
// The cases LiveRoomViewTests does not pin: golden row text, the other status words,
// pip and banner edges, marker geometry, the sidecar, and probe rates a stale room
// link must not replace. They drive RoomMemoryMirror and the pure screen. No socket.

private func roomTabRepoRoot() -> URL {
    URL(fileURLWithPath: #filePath)
        .deletingLastPathComponent()
        .deletingLastPathComponent()
        .deletingLastPathComponent()
}

private func roomTabServer(
    topics: [String] = ["/room_memory/map_bricks", "/room_memory/events", "/room_memory/status"]
) -> LiveRoomConnection {
    .connected(LiveRoomServer(name: "room-tab", capabilities: [], canPublish: false, memoryTopics: topics))
}

private func roomTabCDR(_ encode: () throws -> Data, _ what: String) -> Data {
    do { return try encode() } catch {
        expect(false, "room tab \(what) encodes: \(error)")
        return Data()
    }
}

private func roomTabHeader(
    _ epoch: UInt64,
    _ seq: UInt64,
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1, nanosec: 0)
) -> RoomMemory.Header {
    RoomMemory.Header(roomId: "lab", mapEpoch: epoch, seq: seq, stamp: stamp)
}

private func roomTabBrick() -> RoomMemory.Brick {
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

private func roomTabSnapshot(
    epoch: UInt64 = 43,
    seq: UInt64 = 1,
    voxel: Float = 0.05,
    bricks: [RoomMemory.Brick] = [roomTabBrick()],
    generation: UInt64? = LiveRoomFixture.generation
) -> Data {
    let message = RoomMemory.MapBricks(
        header: roomTabHeader(epoch, seq),
        kind: .snapshot,
        baseEpoch: 0,
        voxelSize: voxel,
        bricks: bricks,
        publisherGeneration: generation
    )
    return roomTabCDR(message.encodedCDR, "snapshot \(epoch)")
}

private func roomTabStatus(
    epoch: UInt64 = 43,
    seq: UInt64,
    state: RoomMemory.MemoryState = .tracking,
    coverage: Float? = nil,
    eventsSeq: UInt64? = nil,
    residual: Float? = nil,
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1, nanosec: 0),
    generation: UInt64? = LiveRoomFixture.generation,
    mid360Hz: UInt16? = nil,
    jt128Hz: UInt16? = nil
) -> Data {
    let message = RoomMemory.MemoryStatus(
        header: roomTabHeader(epoch, seq, stamp: stamp),
        state: state,
        brickCount: 1,
        openEventCount: 1,
        coverageKnownFraction: coverage,
        poseResidualM: residual,
        windowS: nil,
        mid360Hz: mid360Hz,
        jt128Hz: jt128Hz,
        droppedWindows: 0,
        eventsSeq: eventsSeq,
        publisherGeneration: generation
    )
    return roomTabCDR(message.encodedCDR, "status \(seq)")
}

private func roomTabEvent(
    id: UInt64,
    seq: UInt64,
    state: RoomMemory.ChangeEvent.State,
    kind: RoomMemory.ChangeEvent.Kind = .appeared,
    epoch: UInt64 = 43,
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1_760_400_010, nanosec: 0),
    centroid: SIMD3<Float> = SIMD3(1, 0.25, 0.25),
    aabb: RoomMemory.ChangeEvent.Box? = nil,
    fromCentroid: SIMD3<Float>? = nil,
    voxelCount: UInt32 = 8,
    evidence: Float? = nil,
    alpha: Float? = nil,
    sensors: RoomMemory.Sensors = [],
    generation: UInt64? = LiveRoomFixture.generation
) -> Data {
    let box = aabb ?? RoomMemory.ChangeEvent.Box(min: SIMD3(0.5, 0, 0), max: SIMD3(1.5, 0.5, 0.5))
    let message = RoomMemory.ChangeEvent(
        header: roomTabHeader(epoch, seq, stamp: stamp),
        eventId: id,
        kind: kind,
        state: state,
        centroid: centroid,
        aabb: box,
        fromCentroid: fromCentroid,
        voxelCount: voxelCount,
        volumeM3: 0.001,
        evidence: evidence,
        alpha: alpha,
        sensors: sensors,
        firstSeen: stamp,
        lastSeen: stamp,
        mask: nil,
        publisherGeneration: generation
    )
    return roomTabCDR(message.encodedCDR, "event \(id) s\(seq)")
}

private func roomTabOpened(at now: TimeInterval = 100) -> RoomMemoryMirror {
    var mirror = RoomMemoryMirror(tokenSeed: 1)
    _ = mirror.connectionOpened(at: now, canRequest: false)
    return mirror
}

private func roomTabFeed(_ mirror: inout RoomMemoryMirror, _ topic: LiveRoomTopic, _ payload: Data, at now: TimeInterval) {
    _ = mirror.receive(topic: topic, payload: payload, at: now)
}

private func roomTabScreen(
    _ mirror: RoomMemoryMirror,
    connection: LiveRoomConnection? = nil,
    path: Bool = true,
    started: Bool = true,
    now: TimeInterval = 100,
    kinds: Set<RoomMemory.ChangeEvent.Kind> = Set(RoomMemory.ChangeEvent.Kind.allCases),
    selection: String? = nil,
    timeZone: TimeZone = .gmt
) -> LiveRoomScreen {
    LiveRoomScreen.make(
        mirror: mirror,
        connection: connection ?? roomTabServer(),
        requestPathExists: path,
        started: started,
        now: now,
        kinds: kinds,
        selection: selection,
        timeZone: timeZone
    )
}

private func roomTabLoad(_ mode: LiveRoomFixture.Mode, now: TimeInterval) -> LiveRoomFixture? {
    do {
        return try LiveRoomFixture.load(mode: mode, root: roomTabRepoRoot(), now: now)
    } catch {
        expect(false, "room tab fixture \(mode.rawValue) loads: \(error)")
        return nil
    }
}

private func roomTabFloat(_ value: SIMD3<Double>) -> SIMD3<Float> {
    SIMD3(Float(value.x), Float(value.y), Float(value.z))
}

private func roomTabCorners(min lo: SIMD3<Float>, max hi: SIMD3<Float>) -> [SIMD3<Float>] {
    [
        SIMD3(lo.x, lo.y, lo.z), SIMD3(hi.x, lo.y, lo.z), SIMD3(hi.x, hi.y, lo.z), SIMD3(lo.x, hi.y, lo.z),
        SIMD3(lo.x, lo.y, hi.z), SIMD3(hi.x, lo.y, hi.z), SIMD3(hi.x, hi.y, hi.z), SIMD3(lo.x, hi.y, hi.z),
    ]
}

private func roomTabRGB(_ r: Float, _ g: Float, _ b: Float) -> SIMD3<Float> {
    SIMD3(r / 255, g / 255, b / 255)
}

/// Head length and the wing's distance off the shaft. The base is the midpoint of the two barbs.
private func roomTabArrowShape(_ arrow: LiveRoomArrow) -> (head: Float, wing: Float) {
    let base = (arrow.left + arrow.right) * 0.5
    return (simd_length(arrow.tip - base), simd_length(arrow.left - base))
}

private func roomTabLine2(
    id: UInt64,
    alpha: Float? = nil,
    sensors: RoomMemory.Sensors = [],
    voxelCount: UInt32 = 8,
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1_760_400_010, nanosec: 0)
) -> (line1: String, line2: String) {
    var mirror = roomTabOpened()
    roomTabFeed(
        &mirror, .events,
        roomTabEvent(id: id, seq: 0, state: .candidate, stamp: stamp, voxelCount: voxelCount, alpha: alpha, sensors: sensors),
        at: 100
    )
    let row = roomTabScreen(mirror).openRows.first
    return (row?.line1 ?? "", row?.line2 ?? "")
}

// MARK: - Hand-golden rows

func testRoomTabHandGoldenRows() {
    let now: TimeInterval = 500
    guard let map = roomTabLoad(.map, now: now) else { return }
    let gmt = roomTabScreen(map.mirror, connection: map.connection, path: false, started: false, now: now)
    expect(gmt.openRows.map(\.line1) == [
        "disappeared · confirmed · 8 cells · 2025-10-14 00:00:12",
        "appeared · checking · 8 cells · 2025-10-14 00:00:10",
        "moved · confirmed · 125 cells · 2025-10-14 00:00:10",
    ], "open rows, newest stamp then higher seq: \(gmt.openRows.map(\.line1))")
    expect(gmt.openRows.map(\.line2) == [
        "box not measured · false-alarm limit not calibrated · Mid-360 · box only",
        "box not measured · false-alarm limit 5% · Mid-360 · box only",
        "88% of the box was seen · false-alarm limit not calibrated · Mid-360, JT128",
    ], "open line 2: \(gmt.openRows.map(\.line2))")
    expect(gmt.openRows.allSatisfy { !$0.line2.contains("latest status unknown") },
           "the map fixture is caught up, so open rows are current")
    expect(!gmt.openRows[2].line2.contains("box only"), "the moved golden has a mask, so line 2 has no box only")
    expect(!gmt.openRows[1].line1.contains("00:00:11"), "stamp 1760400010.250 stays on second 10")

    expect(gmt.historyRows.count == 1, "the map fixture has one terminal row, the expired speck")
    expect(gmt.historyRows[0].line1 == "appeared · no longer detected · 8 cells · 2025-10-14 00:00:02",
           "history line 1: \(gmt.historyRows[0].line1)")
    expect(gmt.historyRows[0].line2 == "box not measured · false-alarm limit not calibrated · JT128 · box only",
           "history line 2 ends in the speck's sensor: \(gmt.historyRows[0].line2)")

    guard let chicago = TimeZone(identifier: "America/Chicago") else {
        expect(false, "America/Chicago is available")
        return
    }
    let zoned = roomTabScreen(map.mirror, connection: map.connection, path: false, started: false, now: now, timeZone: chicago)
    expect(zoned.openRows[1].line1 == "appeared · checking · 8 cells · 2025-10-13 19:00:10",
           "Chicago keeps the quarter-second on 19:00:10: \(zoned.openRows[1].line1)")

    let one = roomTabLine2(id: 41, voxelCount: 1)
    expect(one.line1 == "appeared · checking · 1 cell · 2025-10-14 00:00:10", "one voxel is 1 cell: \(one.line1)")
    expect(!one.line1.contains("1 cells"), "the singular does not take the plural")

    let none = roomTabLine2(id: 42, sensors: [])
    expect(none.line2.contains("sensor —"), "an empty sensor mask is sensor —: \(none.line2)")
    let jt = roomTabLine2(id: 43, sensors: .jt128)
    expect(jt.line2.contains("JT128") && !jt.line2.contains("Mid-360"), "JT128 alone: \(jt.line2)")
    let mid = roomTabLine2(id: 44, sensors: .mid360)
    expect(mid.line2.contains("Mid-360") && !mid.line2.contains("JT128"), "Mid-360 alone: \(mid.line2)")
    let both = roomTabLine2(id: 45, sensors: [.mid360, .jt128])
    expect(both.line2.contains("Mid-360, JT128"), "both sensors: \(both.line2)")

    let onePercent = roomTabLine2(id: 46, alpha: 0.01)
    expect(onePercent.line2.contains("false-alarm limit 1%") && !onePercent.line2.contains("<1%") && !onePercent.line2.contains("1.5%"),
           "alpha 0.01 is 1%, not the under-one branch: \(onePercent.line2)")
    let tenth = roomTabLine2(id: 47, alpha: 0.015)
    expect(tenth.line2.contains("false-alarm limit 1.5%"), "alpha 0.015 keeps the tenth: \(tenth.line2)")
    let under = roomTabLine2(id: 48, alpha: 0.009)
    expect(under.line2.contains("false-alarm limit <1%"), "alpha 0.009 stays under 1%: \(under.line2)")

    let same = RoomMemory.Time(sec: 1_760_400_100, nanosec: 0)
    var tied = roomTabOpened()
    roomTabFeed(&tied, .events, roomTabEvent(id: 51, seq: 1, state: .candidate, kind: .appeared, stamp: same, voxelCount: 1), at: 100)
    roomTabFeed(&tied, .events, roomTabEvent(id: 52, seq: 5, state: .confirmed, kind: .disappeared, stamp: same, voxelCount: 2), at: 100)
    expect(roomTabScreen(tied).openRows.map(\.line1) == [
        "disappeared · confirmed · 2 cells · 2025-10-14 00:01:40",
        "appeared · checking · 1 cell · 2025-10-14 00:01:40",
    ], "the same stamp orders by higher seq first")

    var later = roomTabOpened()
    let early = RoomMemory.Time(sec: 1_760_400_100, nanosec: 0)
    let after = RoomMemory.Time(sec: 1_760_400_200, nanosec: 0)
    roomTabFeed(
        &later, .events,
        roomTabEvent(id: 53, seq: 99, state: .confirmed, kind: .moved, stamp: early, centroid: SIMD3(0, 1, 0), fromCentroid: SIMD3(0, 0, 0)),
        at: 100
    )
    roomTabFeed(&later, .events, roomTabEvent(id: 54, seq: 1, state: .confirmed, kind: .disappeared, stamp: after), at: 100)
    let ordered = roomTabScreen(later).openRows.map(\.line1)
    expect(ordered.count == 2 && ordered[0].hasPrefix("disappeared") && ordered[1].hasPrefix("moved"),
           "a later stamp beats a higher seq: \(ordered)")

    let hidden = roomTabScreen(
        map.mirror, connection: map.connection, path: false, started: false, now: now,
        kinds: [.disappeared, .moved]
    )
    expect(hidden.openRows.map(\.line1) == [
        "disappeared · confirmed · 8 cells · 2025-10-14 00:00:12",
        "moved · confirmed · 125 cells · 2025-10-14 00:00:10",
    ], "the appeared chip hides that open row: \(hidden.openRows.map(\.line1))")
    expect(hidden.historyRows.isEmpty, "the expired appeared speck leaves history when appeared is filtered out")
    expect(!hidden.markers.contains { $0.kind == .appeared }, "the appeared chip hides that marker")
    expect(hidden.markers.contains { $0.kind == .disappeared } && hidden.markers.contains { $0.kind == .moved },
           "disappeared and moved stay on the map")
    expect(hidden.caption == "2 changes · 0 checking", "the caption drops the filtered checking row: \(hidden.caption)")

    var building = map.mirror
    roomTabFeed(&building, .status, roomTabStatus(seq: 2, state: .building, eventsSeq: 5), at: now)
    let built = roomTabScreen(building, connection: map.connection, path: false, started: false, now: now)
    expect(built.openRows.map(\.line1) == gmt.openRows.map(\.line1), "building keeps the open rows in the list")
    expect(built.markers.isEmpty, "building draws no markers")
    expect(built.caption == "0 changes · 0 checking", "the caption counts drawn markers: \(built.caption)")
    expect(building.openEvents.contains { $0.event.state == .confirmed }, "the confirmed event is still open while building")

    let zero = roomTabLine2(id: 55, stamp: RoomMemory.Time(sec: 0, nanosec: 0))
    expect(zero.line1.contains("time —") && !zero.line1.contains("00:00:00"), "a zero stamp is time —: \(zero.line1)")
    let almost = roomTabLine2(id: 56, stamp: RoomMemory.Time(sec: 0, nanosec: 1))
    expect(almost.line1.contains("1970-01-01 00:00:00") && !almost.line1.contains("time —"),
           "one nanosecond is the epoch clock, not the sentinel: \(almost.line1)")
}

// MARK: - Status, pip, banner, opacity, generation

func testRoomTabStatusPipBanner() {
    var held = roomTabOpened(at: 10)
    roomTabFeed(&held, .mapBricks, roomTabSnapshot(), at: 10)
    roomTabFeed(&held, .status, roomTabStatus(seq: 1, coverage: 0.5, eventsSeq: 7), at: 10)
    let offline = LiveRoomScreen.statusLine(mirror: held, connection: .off, now: 10)
    expect(offline == "offline · epoch 43 · run …0000 · coverage 50.0% · events 7 · cell 5.0 cm",
           "an offline socket keeps the held fields and drops the state word: \(offline)")

    let bare = LiveRoomScreen.statusLine(mirror: RoomMemoryMirror(), connection: roomTabServer(), now: 1)
    expect(bare == "epoch — · coverage — · events —", "no epoch anywhere: \(bare)")

    let words: [(RoomMemory.MemoryState, String)] = [
        (.building, "Building"),
        (.rigMoved, "Rig moved"),
        (.degraded, "Degraded"),
        (.paused, "Paused"),
    ]
    for (state, word) in words {
        var mirror = roomTabOpened(at: 10)
        roomTabFeed(&mirror, .status, roomTabStatus(seq: 1, state: state), at: 10)
        let line = LiveRoomScreen.statusLine(mirror: mirror, connection: roomTabServer(), now: 10)
        expect(line.contains(word), "a fresh \(state) status says \(word): \(line)")
        expect(!line.contains("STALE"), "a status read at its receipt is not stale: \(line)")
    }

    var window = roomTabOpened(at: 10)
    roomTabFeed(&window, .mapBricks, roomTabSnapshot(), at: 10)
    roomTabFeed(&window, .events, roomTabEvent(id: 60, seq: 0, state: .confirmed), at: 10)
    roomTabFeed(&window, .status, roomTabStatus(seq: 1, state: .tracking, eventsSeq: 0), at: 10)
    _ = window.tick(at: 12.9)
    let young = roomTabScreen(window, now: 12.9)
    expect(window.isStatusFresh && window.isMapCurrent, "2.9 s is inside statusFreshFor")
    expect(young.statusLine.contains("Tracking") && !young.statusLine.contains("STALE"),
           "2.9 s still shows the state word: \(young.statusLine)")
    expect(young.markers.count == 1 && young.brickOpacity == 1, "a fresh current map keeps its marker at full opacity")
    _ = window.tick(at: 13)
    let aged = roomTabScreen(window, now: 13)
    expect(!window.isStatusFresh && !window.isMapCurrent, "3 s closes the freshness window and the map leaves current")
    expect(aged.statusLine.hasPrefix("STALE · 3s ·") && !aged.statusLine.contains("Tracking"),
           "the state word drops at 3 s: \(aged.statusLine)")
    expect(aged.markers.isEmpty && aged.brickOpacity == 0.35, "a stale map hides markers and greys the bricks")

    var copy = roomTabOpened(at: 100)
    roomTabFeed(&copy, .mapBricks, roomTabSnapshot(), at: 100)
    roomTabFeed(&copy, .events, roomTabEvent(id: 61, seq: 0, state: .confirmed), at: 100)
    roomTabFeed(&copy, .status, roomTabStatus(seq: 5, state: .tracking, eventsSeq: 0), at: 100)
    let duplicates = copy.activity.duplicates
    roomTabFeed(
        &copy, .status,
        roomTabStatus(seq: 5, state: .paused, eventsSeq: 0, stamp: RoomMemory.Time(sec: 9, nanosec: 0)),
        at: 102
    )
    roomTabFeed(&copy, .status, roomTabStatus(seq: 4, state: .building, eventsSeq: 0), at: 102)
    expect(copy.activity.duplicates == duplicates + 2, "an equal seq and a lower seq are duplicates")
    expect(copy.status?.state == .tracking && copy.status?.seq == 5 && copy.status?.receivedAt == 100,
           "a duplicate does not replace the held status or its receipt time")
    _ = copy.tick(at: 102.9)
    expect(copy.isStatusFresh && copy.isMapCurrent, "the ignored copies do not extend the freshness window")
    expect(roomTabScreen(copy, now: 102.9).statusLine.contains("Tracking"), "the held word is still Tracking at 2.9 s")
    _ = copy.tick(at: 103)
    let ignored = roomTabScreen(copy, now: 103)
    expect(!copy.isStatusFresh && !copy.isMapCurrent, "the window still closes 3 s after the first receipt")
    expect(ignored.statusLine.hasPrefix("STALE · 3s ·"), "age is measured from the first receipt: \(ignored.statusLine)")
    expect(!ignored.statusLine.contains("Paused") && !ignored.statusLine.contains("Building") && !ignored.statusLine.contains("Tracking"),
           "the ignored states never reach the line: \(ignored.statusLine)")
    roomTabFeed(&copy, .status, roomTabStatus(seq: 6, state: .paused, eventsSeq: 0), at: 103)
    let restored = roomTabScreen(copy, now: 103)
    expect(copy.isStatusFresh && copy.isMapCurrent && copy.status?.receivedAt == 103, "a higher seq restores freshness")
    expect(restored.statusLine.contains("Paused") && !restored.statusLine.contains("STALE"),
           "the new seq shows its word: \(restored.statusLine)")
    expect(restored.markers.count == 1 && restored.brickOpacity == 1, "paused keeps the confirmed box on a current map")

    guard let map = roomTabLoad(.map, now: 800), let stale = roomTabLoad(.stale, now: 800) else { return }
    let mapScreen = roomTabScreen(map.mirror, connection: map.connection, path: false, started: false, now: 800)
    expect(mapScreen.brickOpacity == 1, "a current map draws bricks at full opacity")
    let staleScreen = roomTabScreen(stale.mirror, connection: stale.connection, path: false, started: false, now: 800)
    expect(staleScreen.brickOpacity == 0.35, "a stale map greys bricks to 0.35")
    expect(staleScreen.markers.isEmpty && staleScreen.pip == .warn, "a stale map draws nothing and warns")
    expect(staleScreen.statusLine == "STALE · 4s · epoch 43 · run …0000 · coverage — · events 5 · cell 5.0 cm",
           "stale fixture line: \(staleScreen.statusLine)")

    var closed = map.mirror
    let openCount = mapScreen.openRows.count
    closed.connectionClosed(at: 800)
    let lost = roomTabScreen(closed, connection: .off, path: false, started: true, now: 800)
    expect(lost.brickOpacity == 0.35 && lost.markers.isEmpty, "a closed socket greys the bricks and clears markers")
    expect(lost.openRows.count == openCount && openCount > 0, "open rows stay after the socket closes")
    expect(lost.banner == LiveRoomBanner(text: "connection lost", tone: .bad), "a started socket that is off says connection lost")
    expect(lost.statusLine == "offline · epoch 43 · run …0000 · coverage — · events 5 · cell 5.0 cm",
           "the closed socket's line keeps the held fields: \(lost.statusLine)")
    expect(lost.pip == .warn && closed.status != nil && !closed.isStatusFresh,
           "a held status that is no longer fresh warns")

    func currentRoom(
        state: RoomMemory.MemoryState,
        residual: Float? = nil,
        mid: UInt16? = nil,
        jt: UInt16? = nil
    ) -> RoomMemoryMirror {
        var mirror = roomTabOpened(at: 20)
        roomTabFeed(&mirror, .mapBricks, roomTabSnapshot(), at: 20)
        roomTabFeed(&mirror, .events, roomTabEvent(id: 70, seq: 0, state: .confirmed), at: 20)
        roomTabFeed(
            &mirror, .status,
            roomTabStatus(seq: 1, state: state, eventsSeq: 0, residual: residual, mid360Hz: mid, jt128Hz: jt),
            at: 20
        )
        return mirror
    }

    let rig = currentRoom(state: .rigMoved, residual: 0.12)
    let rigScreen = roomTabScreen(rig, now: 20)
    expect(rig.isMapCurrent && !rig.bricks.isEmpty, "rig moved still vouches for the map")
    expect(rigScreen.banner == LiveRoomBanner(text: "Rig moved: changes suspended · 0.12 m", tone: .bad),
           "residual 0.12 m: \(String(describing: rigScreen.banner))")
    expect(rigScreen.pip == .bad && rigScreen.markers.isEmpty && rigScreen.brickOpacity == 1,
           "rig moved is a bad pip, no markers, bricks still current")
    let unguarded = currentRoom(state: .rigMoved, residual: nil)
    expect(roomTabScreen(unguarded, now: 20).banner?.text == "Rig moved: changes suspended · guard not run",
           "a nil residual says the guard has not run")

    let quiet = currentRoom(state: .degraded)
    let quietScreen = roomTabScreen(quiet, now: 20)
    expect(quietScreen.pip == .good && quietScreen.markers.count == 1, "degraded keeps the confirmed box and a good pip")
    expect(quietScreen.banner == LiveRoomBanner(text: "Degraded", tone: .warn),
           "nil rates are not silent sensors: \(String(describing: quietScreen.banner))")
    expect(roomTabScreen(currentRoom(state: .degraded, mid: 0), now: 20).banner?.text == "Degraded · Mid-360",
           "only a measured zero names Mid-360")
    let jtOnly = roomTabScreen(currentRoom(state: .degraded, jt: 0), now: 20)
    expect(jtOnly.banner?.text == "Degraded · JT128" && jtOnly.banner?.text.contains("Mid-360") != true,
           "only a measured zero names JT128: \(String(describing: jtOnly.banner?.text))")
    expect(roomTabScreen(currentRoom(state: .degraded, mid: 0, jt: 0), now: 20).banner?.text == "Degraded · Mid-360, JT128",
           "both measured zeros are named")

    let paused = currentRoom(state: .paused)
    let pausedScreen = roomTabScreen(paused, now: 20)
    expect(pausedScreen.banner == LiveRoomBanner(text: "Paused", tone: .dim), "paused banner")
    expect(pausedScreen.pip == .idle && pausedScreen.markers.count == 1, "paused is idle and keeps the confirmed box")

    let building = currentRoom(state: .building)
    let buildingScreen = roomTabScreen(building, now: 20)
    expect(building.isMapCurrent && !building.bricks.isEmpty, "building can still be a current map")
    expect(buildingScreen.banner == LiveRoomBanner(text: "Building reference", tone: .dim), "building banner")
    expect(buildingScreen.pip == .good && buildingScreen.markers.isEmpty && buildingScreen.brickOpacity == 1,
           "building is a good pip, no markers, full brick opacity")

    let url = URL(string: "ws://127.0.0.1:8767")!
    let first = roomTabScreen(RoomMemoryMirror(), connection: .connecting(url: url, attempt: 1), started: true, now: 1)
    expect(first.banner == LiveRoomBanner(text: "connecting", tone: .dim) && first.pip == .idle,
           "the first attempt is a dim connecting banner and an idle pip")
    let retry = roomTabScreen(RoomMemoryMirror(), connection: .connecting(url: url, attempt: 2), started: true, now: 1)
    expect(retry.banner == LiveRoomBanner(text: "connection lost", tone: .bad),
           "a later attempt is connection lost: \(String(describing: retry.banner))")

    let never = roomTabScreen(RoomMemoryMirror(), connection: .off, started: false, now: 1)
    expect(never.banner == nil && never.pip == .idle, "a socket that was never started is idle, with no banner")
    let dropped = roomTabScreen(RoomMemoryMirror(), connection: .off, started: true, now: 1)
    expect(dropped.banner?.text == "connection lost", "an empty mirror that had started says connection lost")

    let emptyTopics = roomTabScreen(RoomMemoryMirror(), connection: roomTabServer(topics: []), started: true, now: 1)
    expect(emptyTopics.banner?.text == "Room memory not running on Spark", "empty topics banner")
    expect(emptyTopics.canvasSentence == "Room memory not running on Spark", "empty topics canvas")
    expect(emptyTopics.pip == .warn, "empty topics hit the not-current warn before the idle leftover")

    let earlier = LiveRoomFixture.generation - 1000
    var restart = roomTabOpened(at: 10)
    roomTabFeed(&restart, .mapBricks, roomTabSnapshot(generation: earlier), at: 10)
    roomTabFeed(&restart, .events, roomTabEvent(id: 71, seq: 0, state: .confirmed, generation: earlier), at: 10)
    roomTabFeed(&restart, .status, roomTabStatus(seq: 9, eventsSeq: 0, generation: earlier), at: 10)
    let before = roomTabScreen(restart, now: 10)
    expect(before.statusLine.contains("run …9000") && !before.statusLine.contains("run …0000"),
           "the earlier run tags …9000: \(before.statusLine)")
    roomTabFeed(&restart, .status, roomTabStatus(epoch: 44, seq: 0, eventsSeq: nil), at: 11)
    let after = roomTabScreen(restart, now: 11)
    expect(after.statusLine.contains("run …0000") && !after.statusLine.contains("run …9000"),
           "the run tag follows the active generation: \(after.statusLine)")
    expect(after.earlierRows.count == 1, "the earlier run's open event stays in the earlier list")

    guard let legacy = roomTabLoad(.legacy, now: 800) else { return }
    let legacyLine = roomTabScreen(legacy.mirror, connection: legacy.connection, path: false, started: false, now: 800).statusLine
    expect(legacyLine == "epoch 43 · coverage — · events — · Tracking · cell 5.0 cm",
           "a fresh legacy line has no run segment: \(legacyLine)")
}

// MARK: - Markers, arrow, sidecar

func testRoomTabMarkersArrowSidecar() {
    let from = roomTabFloat(SyntheticRoom.boxA.center)
    let to = roomTabFloat(SyntheticRoom.boxB.center)
    let box = RoomMemory.ChangeEvent.Box(
        min: roomTabFloat(SyntheticRoom.boxB.min),
        max: roomTabFloat(SyntheticRoom.boxB.max)
    )
    let corners = roomTabCorners(min: box.min, max: box.max)
    var mirror = roomTabOpened(at: 30)
    roomTabFeed(&mirror, .mapBricks, roomTabSnapshot(), at: 30)
    roomTabFeed(
        &mirror, .events,
        roomTabEvent(
            id: 81, seq: 0, state: .confirmed, kind: .moved, centroid: to, aabb: box, fromCentroid: from
        ),
        at: 30
    )
    roomTabFeed(&mirror, .status, roomTabStatus(seq: 1, eventsSeq: 0), at: 30)
    let drawn = LiveRoomMarkers.draw(mirror: mirror)
    expect(drawn.count == 1, "one confirmed moved box")
    let marker = drawn[0]
    expect(marker.corners == corners, "corners are the Float cast of box B, got \(marker.corners)")
    expect(marker.fillAlpha == 0.28 && !marker.dashed && !marker.onTop, "an unselected confirmed fill is 0.28")
    expect(marker.color == roomTabRGB(196, 160, 74), "moved is (196, 160, 74) / 255")
    expect(LiveRoomMarkers.color(.appeared) == roomTabRGB(63, 163, 122), "appeared is (63, 163, 122) / 255")
    expect(LiveRoomMarkers.color(.disappeared) == roomTabRGB(196, 90, 90), "disappeared is (196, 90, 90) / 255")
    expect(marker.color == LiveRoomMarkers.color(.moved), "the marker uses the moved color")

    guard let arrow = marker.arrow else {
        expect(false, "the 1 m move draws an arrow")
        return
    }
    expect(arrow.tail == from && arrow.tip == to, "the arrow runs from box A to box B")
    expect(abs(simd_length(arrow.tip - arrow.tail) - 1) < 1e-4, "box B is 1 m from box A")
    let capped = roomTabArrowShape(arrow)
    expect(abs(capped.head - 0.15) < 1e-4, "a 1 m shaft caps the head at 0.15 m, got \(capped.head)")
    expect(abs(capped.wing - 0.15 * 0.45) < 1e-4, "the wing is 0.45 of the head, got \(capped.wing)")

    guard let vertical = LiveRoomMarkers.arrow(from: .zero, to: SIMD3(0, 0, 1)) else {
        expect(false, "a vertical shaft still returns an arrow")
        return
    }
    let upright = roomTabArrowShape(vertical)
    expect(abs(upright.head - 0.15) < 1e-4 && abs(upright.wing - 0.15 * 0.45) < 1e-4,
           "the vertical fallback still sizes the head and wing, got \(upright)")
    expect(vertical.left.x != vertical.right.x, "the wing is perpendicular to the vertical shaft")

    guard let short = LiveRoomMarkers.arrow(from: .zero, to: SIMD3(0.2, 0, 0)) else {
        expect(false, "a 0.2 m segment returns an arrow")
        return
    }
    let small = roomTabArrowShape(short)
    expect(abs(simd_length(short.tip - short.tail) - 0.2) < 1e-4, "the short shaft is 0.2 m")
    expect(abs(small.head - 0.07) < 1e-4, "0.2 m takes the uncapped head, 0.07 m, got \(small.head)")
    expect(abs(small.wing - small.head * 0.45) < 1e-4, "the short wing is 0.45 of that head, got \(small.wing)")
    expect(LiveRoomMarkers.arrow(from: SIMD3(1, 2, 3), to: SIMD3(1, 2, 3)) == nil, "equal endpoints are not an arrow")

    let selected = LiveRoomMarkers.draw(mirror: mirror, selection: "81")
    expect(selected.count == 1 && selected[0].fillAlpha == 0.85 && selected[0].onTop && !selected[0].dashed,
           "a selected confirmed box fills at 0.85, on top, solid")
    expect(selected[0].corners == corners, "selection does not move the corners")

    var candidate = roomTabOpened(at: 30)
    roomTabFeed(&candidate, .mapBricks, roomTabSnapshot(), at: 30)
    roomTabFeed(&candidate, .events, roomTabEvent(id: 82, seq: 0, state: .candidate), at: 30)
    roomTabFeed(&candidate, .status, roomTabStatus(seq: 1, eventsSeq: 0), at: 30)
    let dashed = LiveRoomMarkers.draw(mirror: candidate, selection: "82")
    expect(dashed.count == 1 && dashed[0].dashed && dashed[0].fillAlpha == 0,
           "a selected candidate stays dashed and unfilled")

    let fitted = LiveRoomVoxels.fitPoints(map: .empty, markers: drawn)
    expect(fitted.count == corners.count + arrow.points.count, "the fit is the corners plus the four arrow points")
    for point in corners + arrow.points {
        expect(fitted.contains(point), "the fit includes \(point)")
    }

    var paused = mirror
    roomTabFeed(&paused, .status, roomTabStatus(seq: 2, state: .paused, eventsSeq: 0), at: 30)
    let kept = LiveRoomMarkers.draw(mirror: paused)
    expect(kept.count == 1 && kept[0].corners == corners && kept[0].arrow?.tail == from && kept[0].arrow?.tip == to,
           "paused keeps the synthetic box and its arrow")
    var building = mirror
    roomTabFeed(&building, .status, roomTabStatus(seq: 2, state: .building, eventsSeq: 0), at: 30)
    expect(LiveRoomMarkers.draw(mirror: building).isEmpty, "building clears the synthetic box")
    var rig = mirror
    roomTabFeed(&rig, .status, roomTabStatus(seq: 2, state: .rigMoved, eventsSeq: 0, residual: 0.12), at: 30)
    expect(LiveRoomMarkers.draw(mirror: rig).isEmpty, "rig moved clears the synthetic box")

    expect(LiveRoomSidecar.outputPath([:]) == nil, "an ordinary launch writes no sidecar")
    expect(LiveRoomSidecar.outputPath(["LABCOMMAND_ROOM_SIDECAR": "/tmp/room.txt"]) == "/tmp/room.txt",
           "an explicit sidecar path is that path")
    expect(
        LiveRoomSidecar.outputPath(["LABCOMMAND_ROOM_SIDECAR": "", "LABCOMMAND_LIVE_ROOM": "map"])
            == "build/screenshots/live-room-map.txt",
        "an empty sidecar path falls through to the launch mode"
    )
    let shots = [
        "live": "live-room-live",
        "map": "live-room-map",
        "status": "live-room-status",
        "stale": "live-room-stale",
        "lost": "live-room-lost",
        "restart": "live-room-restart",
        "legacy": "live-room-legacy",
    ]
    for (mode, name) in shots {
        expect(
            LiveRoomSidecar.outputPath(["LABCOMMAND_LIVE_ROOM": mode]) == "build/screenshots/\(name).txt",
            "\(mode) sidecar is \(name).txt"
        )
    }
    expect(
        LiveRoomSidecar.outputPath(["LABCOMMAND_LIVE_ROOM": "nope"]) == "build/screenshots/live-room-unavailable.txt",
        "any other launch value is the unavailable sidecar"
    )

    let now: TimeInterval = 900
    guard let map = roomTabLoad(.map, now: now), let off = roomTabLoad(.unavailable, now: now) else { return }
    let mapScreen = roomTabScreen(map.mirror, connection: map.connection, path: false, started: false, now: now)
    expect(mapScreen.markers.count == 3, "the map fixture draws the three open events")
    let text = LiveRoomSidecar.render(
        screen: mapScreen, connection: map.connection, url: nil, canvas: (x: 12, y: 24, w: 640, h: 480)
    )
    expect(text.contains("connection .connected"), "sidecar names the connection: \(text)")
    expect(text.contains("status epoch 43 · run …0000 · coverage — · events 5 · Tracking · cell 5.0 cm"),
           "sidecar includes the status line")
    expect(text.contains("pip good"), "sidecar names the pip")
    expect(text.contains("markers 3"), "sidecar counts the markers")
    expect(text.contains("caption 2 changes · 1 checking"), "sidecar includes the caption")
    for row in mapScreen.openRows + mapScreen.historyRows + mapScreen.earlierRows {
        expect(text.contains(row.line1), "sidecar includes \(row.line1)")
        expect(text.contains(row.line2), "sidecar includes \(row.line2)")
    }
    expect(text.contains("canvas 12 24 640 480"), "a passed rect is a canvas line")

    let offScreen = roomTabScreen(off.mirror, connection: off.connection, path: false, started: false, now: now)
    expect(offScreen.canvasSentence == "Room memory unavailable", "the off fixture replaces the map")
    let omitted = LiveRoomSidecar.render(
        screen: offScreen, connection: off.connection, url: nil, canvas: (x: 1, y: 2, w: 3, h: 4)
    )
    expect(!omitted.contains("canvas"), "Room memory unavailable omits the canvas line: \(omitted)")
}

// MARK: - Status dots the probe rates must survive

func testRoomTabStatusDotsProbeRates() {
    guard let probe = RigStatusDecoder.parse(json: requireFixture("both-sensors-live"), viewTunnelUp: true) else {
        expect(false, "both-sensors-live.json parses")
        return
    }
    expect(probe.sparkReachable, "the live probe reached the Spark")
    expect(probe.mid360.state == .streaming && probe.mid360.pointRateHz == 9.8, "Mid-360 in the fixture is 9.8 Hz")
    expect(probe.jt128.state == .streaming && probe.jt128.pointRateHz == 8.6, "JT128 in the fixture is 8.6 Hz")

    let room = LiveRoomStatus(
        RoomMemory.MemoryStatus(
            header: roomTabHeader(3, 1),
            state: .tracking,
            brickCount: 1,
            openEventCount: 0,
            coverageKnownFraction: nil,
            poseResidualM: nil,
            windowS: nil,
            mid360Hz: 10,
            jt128Hz: 10,
            droppedWindows: 0,
            eventsSeq: nil,
            publisherGeneration: LiveRoomFixture.generation
        ),
        receivedAt: 100
    )
    expect(room.mid360Hz == 10 && room.jt128Hz == 10, "the room status under test is 10 Hz on both lidars")

    let waiting = StatusDots.derive(
        rig: probe,
        connection: .waiting(retryIn: 5, reason: "down"),
        status: room,
        isStatusFresh: true
    )
    expect(waiting.sparkReachable == probe.sparkReachable && waiting.sparkHelp == nil,
           "a waiting socket leaves the Spark chip on the probe")
    expect(waiting.mid360 == probe.mid360 && waiting.jt128 == probe.jt128,
           "a waiting socket keeps 9.8 Hz and 8.6 Hz even when the room status is fresh at 10 Hz")

    let stale = StatusDots.derive(rig: probe, connection: roomTabServer(), status: room, isStatusFresh: false)
    expect(stale.sparkReachable && stale.sparkHelp == nil, "a connected socket keeps Spark reachable")
    expect(stale.mid360 == probe.mid360 && stale.jt128 == probe.jt128,
           "a stale 10 Hz does not replace the probe's 9.8 Hz and 8.6 Hz")

    var unmeasured = room
    unmeasured.mid360Hz = nil
    unmeasured.jt128Hz = nil
    let kept = StatusDots.derive(rig: probe, connection: roomTabServer(), status: unmeasured, isStatusFresh: true)
    expect(kept.mid360 == probe.mid360 && kept.jt128 == probe.jt128,
           "a nil rate is unmeasured, so the probe's 9.8 Hz and 8.6 Hz stay")
}

func runRoomTabModelTests() {
    testRoomTabHandGoldenRows()
    print("  Room tab: hand-golden rows, local clocks, kind filter, synthetic phrases"); passed()
    testRoomTabStatusPipBanner()
    print("  Room tab: status line, pip, banner, opacity, generation"); passed()
    testRoomTabMarkersArrowSidecar()
    print("  Room tab: marker corners, arrow head, sidecar text"); passed()
    testRoomTabStatusDotsProbeRates()
    print("  Room tab: a waiting socket or a stale status keeps the probe rates"); passed()
}
