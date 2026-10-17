import AppKit
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

/// The publisher generation of the view tests' messages; nil builds the legacy layout.
private let viewGeneration: UInt64? = LiveRoomFixture.generation

private func viewSnapshot(
    epoch: UInt64,
    seq: UInt64 = 1,
    room: String = "lab",
    voxel: Float = 0.05,
    bricks: [RoomMemory.Brick] = [],
    generation: UInt64? = viewGeneration
) -> Data {
    let message = RoomMemory.MapBricks(
        header: viewHeader(epoch, seq, room: room),
        kind: .snapshot,
        baseEpoch: 0,
        voxelSize: voxel,
        bricks: bricks,
        publisherGeneration: generation
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
    stamp: RoomMemory.Time = RoomMemory.Time(sec: 1, nanosec: 0),
    generation: UInt64? = viewGeneration
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
        eventsSeq: eventsSeq,
        publisherGeneration: generation
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
    alpha: Float? = nil,
    generation: UInt64? = viewGeneration
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
        mask: nil,
        publisherGeneration: generation
    )
    return viewCDR(message.encodedCDR, "event \(id) s\(seq)")
}

private func viewOpened(at now: TimeInterval = 100, canRequest: Bool = false) -> RoomMemoryMirror {
    var mirror = RoomMemoryMirror(tokenSeed: 1)
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
    selection: String? = nil,
    // Pin GMT unless a case passes a zone. The app's default is the user's zone.
    timeZone: TimeZone = .gmt
) -> LiveRoomScreen {
    LiveRoomScreen.make(
        mirror: mirror,
        connection: connection ?? viewServer(),
        requestPathExists: path,
        started: started,
        now: now,
        kinds: kinds,
        selection: selection,
        timeZone: timeZone
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
        bytes = try LiveRoomFixture.goldenBytes(name: "memory_status_v2_le", root: viewRepoRoot())
    } catch {
        expect(false, "memory_status_v2_le.hex loads: \(error)")
        return
    }
    let decoded = try? RoomMemory.MemoryStatus.decode(cdr: bytes)
    expect(decoded?.eventsSeq == 12, "memory_status_v2_le eventsSeq is 12, got \(String(describing: decoded?.eventsSeq))")

    var golden = viewOpened()
    viewFeed(&golden, .status, bytes, at: 10)
    let goldenLine = LiveRoomScreen.statusLine(mirror: golden, connection: viewServer(), now: 10)
    expect(goldenLine.contains("events 12"), "golden status line contains events 12: \(goldenLine)")
    expect(goldenLine.contains("epoch 42 · run …0000 ·"), "the status line tags the run with the generation's last 4 digits: \(goldenLine)")
    expect(LiveRoomScreen.runTag(1_760_400_001_234) == "run …1234", "run tag is the last 4 digits")
    var legacyStatus = viewOpened()
    viewFeed(&legacyStatus, .status, viewStatus(epoch: 1, seq: 1, generation: nil), at: 10)
    expect(!LiveRoomScreen.statusLine(mirror: legacyStatus, connection: viewServer(), now: 10).contains("run"),
           "a legacy status has no run tag")

    var bare = viewOpened()
    viewFeed(&bare, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 10)
    let nilLine = LiveRoomScreen.statusLine(mirror: bare, connection: viewServer(), now: 10)
    expect(nilLine.contains("events —"), "nil eventsSeq renders events —: \(nilLine)")

    var baseline = viewOpened(canRequest: false)
    viewFeed(&baseline, .events, viewEvent(id: 1, seq: 4, state: .candidate), at: 10)
    viewFeed(&baseline, .status, viewStatus(epoch: 43, seq: 2, eventsSeq: 12), at: 10)
    expect(baseline.eventHistory.contiguousThrough == nil, "seq 4 alone starts no run from 0")
    expect(baseline.eventHistory.coverage == .missing(pending: [0...3, 5...12], lost: []), "got \(baseline.eventHistory.coverage)")
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

func testLiveRoomEventRowsAndHistory() {
    var mirror = viewOpened()
    viewFeed(&mirror, .mapBricks, viewSnapshot(epoch: 43), at: 10)
    viewFeed(&mirror, .status, viewStatus(epoch: 43, seq: 1), at: 10)
    viewFeed(&mirror, .events, viewEvent(id: 9, seq: 1, state: .candidate), at: 10)
    let row = viewScreen(mirror, path: true).openRows.first
    expect(row?.line1.contains("checking") == true && row?.line1.contains("candidate") != true,
           "a candidate row says checking: \(row?.line1 ?? "")")
    expect(row?.line1.contains("8 cells") == true && row?.line1.contains("voxel") != true,
           "the row counts cells: \(row?.line1 ?? "")")
    expect(row?.line2.contains("false-alarm limit not calibrated") == true, "nil alpha names the missing limit: \(row?.line2 ?? "")")
    expect(row?.line2.contains("false-alarm limit 0") != true && row?.line2.contains("<1%") != true,
           "nil alpha is not a number: \(row?.line2 ?? "")")
    expect(row?.line2.contains("box not measured") == true && row?.line2.contains("0%") != true,
           "nil evidence is not a measured zero: \(row?.line2 ?? "")")
    expect(row?.line2.contains("latest status unknown") == true && row?.line2.contains("may be outdated") != true,
           "an open row says the latest status is unknown while not caught up: \(row?.line2 ?? "")")
    expect(row?.line2.contains("box only") == true, "a row without a shape says box only: \(row?.line2 ?? "")")

    func openLine(id: UInt64, evidence: Float? = nil, alpha: Float? = nil) -> String {
        var mirror = viewOpened()
        viewFeed(&mirror, .events, viewEvent(id: id, seq: 0, state: .candidate, evidence: evidence, alpha: alpha), at: 10)
        return viewScreen(mirror).openRows.first?.line2 ?? ""
    }
    let five = openLine(id: 30, alpha: 0.05)
    expect(five.contains("false-alarm limit 5%") && !five.contains("not calibrated"), "alpha 0.05 is a 5% limit: \(five)")
    let exactZero = openLine(id: 31, alpha: 0)
    expect(exactZero.contains("false-alarm limit 0%") && !exactZero.contains("<1%") && !exactZero.contains("not calibrated"),
           "alpha 0 is an exact zero limit: \(exactZero)")
    let tiny = openLine(id: 32, alpha: 0.004)
    expect(tiny.contains("false-alarm limit <1%") && !tiny.contains("0.00") && !tiny.contains("false-alarm limit 0%"),
           "alpha 0.004 stays a small positive limit: \(tiny)")
    let noneSeen = openLine(id: 33, evidence: 0, alpha: 0.05)
    expect(noneSeen.contains("0% of the box was seen") && !noneSeen.contains("box not measured"),
           "evidence 0 is none of the box: \(noneSeen)")
    let allSeen = openLine(id: 34, evidence: 1, alpha: 0.05)
    expect(allSeen.contains("100% of the box was seen"), "evidence 1 is the whole box: \(allSeen)")
    let mostSeen = openLine(id: 35, evidence: 0.875, alpha: 0.05)
    expect(mostSeen.contains("88% of the box was seen"), "evidence 0.875 rounds to 88%: \(mostSeen)")

    var terminal = viewOpened()
    viewFeed(&terminal, .events, viewEvent(id: 20, seq: 0, state: .reverted), at: 10)
    viewFeed(&terminal, .events, viewEvent(id: 21, seq: 1, state: .expired), at: 10)
    let terminalRows = viewScreen(terminal).historyRows.map(\.line1)
    expect(terminalRows.contains { $0.contains("change undone") && !$0.contains("reverted") && !$0.contains("put back") },
           "reverted says the change was undone: \(terminalRows)")
    expect(terminalRows.contains { $0.contains("no longer detected") && !$0.contains("expired") && !$0.contains("didn't last") },
           "expired says it is no longer detected: \(terminalRows)")

    var none = viewOpened()
    viewFeed(&none, .mapBricks, viewSnapshot(epoch: 1), at: 10)
    viewFeed(&none, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 10)
    expect(none.eventHistory.coverage == .empty && none.eventHistory.caughtUp, "a v2 status with no events is a known-empty history")
    expect(viewScreen(none, path: true, now: 10).historyLine == "no events this run", "got \(String(describing: viewScreen(none).historyLine))")

    var done = viewOpened()
    viewFeed(&done, .events, viewEvent(id: 4, seq: 0, state: .candidate), at: 10)
    viewFeed(&done, .events, viewEvent(id: 4, seq: 1, state: .confirmed), at: 10)
    viewFeed(&done, .status, viewStatus(epoch: 43, seq: 1, eventsSeq: 1), at: 10)
    expect(done.eventHistory.caughtUp, "every seq through the status's events_seq: caught up")
    let doneScreen = viewScreen(done, path: true, now: 10)
    expect(doneScreen.historyLine == "all events through event 1 are here", "got \(String(describing: doneScreen.historyLine))")
    expect(doneScreen.openRows.first.map { !$0.line2.contains("latest status unknown") } == true, "caught up: open rows are current")

    var gap = viewOpened(canRequest: true)
    viewFeed(&gap, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 10)
    viewFeed(&gap, .events, viewEvent(id: 3, seq: 5, state: .candidate), at: 10)
    expect(gap.eventHistory.coverage == .missing(pending: [0...4], lost: []), "seq 5 first: 0-4 pending, got \(gap.eventHistory.coverage)")
    expect(viewScreen(gap, path: true).historyLine == "history incomplete: 5 missing", "got \(String(describing: viewScreen(gap).historyLine))")
    expect(viewScreen(gap, path: false).historyLine == "history incomplete: 5 missing · can't ask the Spark for them",
           "a gap with no way to ask says so, got \(String(describing: viewScreen(gap, path: false).historyLine))")

    // Lost on the Spark: a fallback's manifest declares 0-2 unavailable.
    guard let lost = viewLoad(.lost, now: 200) else { return }
    let lostScreen = viewScreen(lost.mirror, connection: lost.connection, path: false, started: false, now: 200)
    expect(lostScreen.historyLine == "history incomplete: 3 missing (3 lost on the Spark)",
           "lost ranges, no ask-suffix when nothing is pending: \(String(describing: lostScreen.historyLine))")
    expect(lostScreen.historyLine?.contains("can't ask") != true, "a fully lost gap does not offer an ask")
    expect(lost.mirror.eventHistory.coverage == .missing(pending: [], lost: [0...2]), "got \(lost.mirror.eventHistory.coverage)")

    var legacy = viewOpened()
    viewFeed(&legacy, .status, viewStatus(epoch: 1, seq: 1, generation: nil), at: 10)
    viewFeed(&legacy, .events, viewEvent(id: 1, seq: 0, state: .candidate, generation: nil), at: 10)
    expect(viewScreen(legacy, path: true, now: 10).historyLine == "history unverified (old Spark node)", "legacy history line")
    expect(legacy.eventHistory.coverage == .unknown && legacy.eventHistory.continuity == .unknown, "legacy is unknown")
}

func testLiveRoomRestartBanner() {
    var mirror = viewOpened(at: 10)
    let earlier = LiveRoomFixture.generation - 1000
    viewFeed(&mirror, .mapBricks, viewSnapshot(epoch: 43, bricks: [viewBrick()], generation: earlier), at: 10)
    viewFeed(&mirror, .events, viewEvent(id: 7, seq: 0, state: .confirmed, generation: earlier), at: 10)
    viewFeed(&mirror, .status, viewStatus(epoch: 43, seq: 9, eventsSeq: 0, generation: earlier), at: 10)
    expect(viewScreen(mirror, now: 10).restartBanner == nil && mirror.continuity == .verified, "one run: no restart banner")
    expect(viewScreen(mirror, now: 10).markers.count == 1, "the confirmed event is drawn while current")

    // The node restarted: its first status counts from 0 under a higher generation.
    viewFeed(&mirror, .status, viewStatus(epoch: 44, seq: 0, eventsSeq: nil), at: 11)
    let restarted = viewScreen(mirror, now: 11)
    expect(restarted.restartBanner?.text == "Spark node restarted · events from the earlier run may be missing",
           "restart banner, got \(String(describing: restarted.restartBanner))")
    expect(mirror.continuity == .broken(sinceGeneration: earlier), "continuity broken since the earlier run")
    expect(restarted.earlierRows.count == 1 && restarted.earlierRows[0].line2.contains("earlier run · latest status unknown"),
           "the earlier run's open event is listed, got \(restarted.earlierRows)")
    expect(restarted.openRows.isEmpty && restarted.markers.isEmpty, "never drawn as a current change")
    expect(mirror.mapState == .stale(.newGeneration) && mirror.bricks.count == 1, "the earlier run's map stays, greyed")
    expect(restarted.historyLine == "no events this run", "the new run's history: \(String(describing: restarted.historyLine))")

    guard let fixture = viewLoad(.restart, now: 300) else { return }
    let screen = viewScreen(fixture.mirror, connection: fixture.connection, path: false, started: false, now: 300)
    expect(screen.restartBanner != nil && screen.earlierRows.count == 1, "restart fixture shows the banner and one earlier event")
    expect(fixture.mirror.isMapCurrent && screen.markers.isEmpty, "restart fixture: the new run's map is current, nothing drawn")

    guard let legacy = viewLoad(.legacy, now: 300) else { return }
    let legacyScreen = viewScreen(legacy.mirror, connection: legacy.connection, path: false, started: false, now: 300)
    expect(legacy.mirror.mapState == .stale(.unverified) && legacyScreen.markers.isEmpty, "legacy fixture: unverified, no overlays")
    expect(legacyScreen.historyLine == "history unverified (old Spark node)" && legacyScreen.restartBanner == nil, "legacy fixture lines")
    expect(legacyScreen.standaloneResync == nil, "legacy never asks for a resync")
    expect(legacyScreen.banner?.text == "unverified · Spark node predates publisher generations",
           "legacy banner, got \(String(describing: legacyScreen.banner?.text))")
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
        waitingScreen.historyRows.contains { $0.line2.contains("saved · map updating") },
        "history says the saved change is waiting on the map: \(waitingScreen.historyRows)"
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
    expect(full.caption == "2 changes · 1 checking", "map fixture caption, got \(full.caption)")
    let hidden = viewScreen(
        fixture.mirror,
        connection: fixture.connection,
        path: false,
        started: false,
        now: 200,
        kinds: [.disappeared, .moved]
    )
    expect(!hidden.markers.contains { $0.kind == .appeared }, "turning appeared off removes it from the marker set")
    expect(hidden.caption == "2 changes · 0 checking", "the caption drops the filtered checking row: \(hidden.caption)")
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
    expect(staleMap.eventHistory.coverage == .empty, "no events leaves the history known-empty")
    let resync = viewScreen(staleMap, path: false, now: 20)
    expect(
        resync.banner?.text == "resyncing · as of epoch 43 · can't ask the Spark to resend the map",
        "the resync banner carries the phrase, got \(String(describing: resync.banner?.text))"
    )
    expect(resync.standaloneResync == nil, "that phrase is not also a standalone line")

    let lost = viewScreen(staleMap, connection: .waiting(retryIn: 5, reason: "down"), path: false, now: 20)
    expect(lost.banner?.text == "connection lost", "connection lost stays the banner, got \(String(describing: lost.banner?.text))")
    expect(lost.standaloneResync == "can't ask the Spark to resend the map", "complete history stands the phrase alone, got \(String(describing: lost.standaloneResync))")

    var history = viewOpened(at: 20, canRequest: true)
    viewFeed(&history, .status, viewStatus(epoch: 1, seq: 1, eventsSeq: nil), at: 20)
    viewFeed(&history, .events, viewEvent(id: 1, seq: 4, state: .candidate), at: 20)
    expect(history.eventHistory.coverage == .missing(pending: [0...3], lost: []), "got \(history.eventHistory.coverage)")
    let suffixed = viewScreen(history, connection: .waiting(retryIn: 5, reason: "down"), path: false, now: 20)
    expect(suffixed.banner?.text == "connection lost", "the banner stays connection lost")
    expect(
        suffixed.historyLine == "history incomplete: 4 missing · can't ask the Spark for them",
        "the history line says it cannot ask, got \(String(describing: suffixed.historyLine))"
    )
    expect(suffixed.standaloneResync == nil, "no map to resync: no standalone line")

    var unsent = viewOpened(at: 30)
    viewFeed(&unsent, .mapBricks, viewSnapshot(epoch: 43, seq: 1, bricks: [viewBrick()]), at: 30)
    viewFeed(&unsent, .status, viewStatus(epoch: 43, seq: 1), at: 30)
    unsent.noteRequestUnsent()
    unsent.noteRequestUnsent()
    let unsentScreen = viewScreen(unsent, path: true, now: 30)
    expect(unsent.isMapCurrent && unsent.eventHistory.coverage == .empty, "unsent count on a current, complete mirror")
    expect(unsentScreen.banner == nil, "fresh tracking has no banner, got \(String(describing: unsentScreen.banner))")
    expect(
        unsentScreen.standaloneResync == "can't ask the Spark to resend the map · 2 not sent",
        "got \(String(describing: unsentScreen.standaloneResync))"
    )

    var empty = viewOpened(at: 40)
    viewFeed(&empty, .mapBricks, viewSnapshot(epoch: 43, seq: 1), at: 40)
    viewFeed(&empty, .status, viewStatus(epoch: 90, seq: 2), at: 40)
    viewFeed(&empty, .status, viewStatus(epoch: 90, seq: 3, stamp: RoomMemory.Time(sec: 4, nanosec: 0)), at: 40)
    let waitingMap = viewScreen(empty, path: false, now: 40)
    expect(waitingMap.banner?.text == "Waiting for map", "an empty stale map waits, got \(String(describing: waitingMap.banner?.text))")
    expect(
        waitingMap.standaloneResync == "can't ask the Spark to resend the map",
        "Waiting for map leaves the phrase standalone, got \(String(describing: waitingMap.standaloneResync))"
    )
}

func testLiveRoomLaunchAndFixtures() {
    let unset = LiveRoomLaunch.parse([:])
    expect(unset == .live(opensRoom: false), "an unset variable stays on the live socket and does not skip probes")
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
    expect(map.mirror.status?.eventsSeq == 5, "map fixture names events_seq 5")
    expect(map.mirror.activeGeneration == LiveRoomFixture.generation && map.mirror.wire == .v2, "fixtures are v2, one generation")

    let mapScreen = viewScreen(map.mirror, connection: map.connection, path: false, started: false, now: now)
    expect(
        mapScreen.statusLine == "epoch 43 · run …0000 · coverage — · events 5 · Tracking · cell 5.0 cm",
        "map status line, got \(mapScreen.statusLine)"
    )
    expect(mapScreen.pip == .good, "map pip is good, got \(mapScreen.pip)")
    expect(mapScreen.historyLine == "all events through event 5 are here", "map fixture history, got \(String(describing: mapScreen.historyLine))")
    expect(map.mirror.eventHistory.caughtUp, "map fixture is caught up")
    let refreshed = map.refreshed(at: now + 2)
    expect(refreshed.mirror.isStatusFresh && refreshed.mirror.status?.seq == 2, "refresh feeds the next status seq")

    let statusScreen = viewScreen(status.mirror, connection: status.connection, path: false, started: false, now: now)
    expect(
        statusScreen.statusLine == "epoch 42 · run …0000 · coverage 75.0% · events 12 · Tracking",
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

func testLiveRoomLocalClocks() {
    let october = RoomMemory.Time(sec: 1_760_400_010, nanosec: 0)
    let january = RoomMemory.Time(sec: 1_736_899_210, nanosec: 0)
    let july = RoomMemory.Time(sec: 1_752_537_610, nanosec: 0)
    guard let chicago = TimeZone(identifier: "America/Chicago") else {
        expect(false, "America/Chicago is available")
        return
    }
    let kolkata = TimeZone(secondsFromGMT: 5 * 3600 + 30 * 60)!
    let utc = TimeZone(secondsFromGMT: 0)!
    let minusFive = TimeZone(secondsFromGMT: -5 * 3600)!

    func lines(_ stamp: RoomMemory.Time, _ zone: TimeZone) -> (open: String, history: String, earlier: String) {
        var openMirror = viewOpened()
        viewFeed(&openMirror, .events, viewEvent(id: 1, seq: 0, state: .candidate, stamp: stamp), at: 10)
        let openLine = viewScreen(openMirror, timeZone: zone).openRows.first?.line1 ?? ""

        var historyMirror = viewOpened()
        viewFeed(&historyMirror, .events, viewEvent(id: 2, seq: 0, state: .accepted, stamp: stamp), at: 10)
        let historyLine = viewScreen(historyMirror, timeZone: zone).historyRows.first?.line1 ?? ""

        var earlierMirror = viewOpened(at: 10)
        let earlierGen = LiveRoomFixture.generation - 1000
        viewFeed(&earlierMirror, .mapBricks, viewSnapshot(epoch: 43, bricks: [viewBrick()], generation: earlierGen), at: 10)
        viewFeed(&earlierMirror, .events, viewEvent(id: 3, seq: 0, state: .confirmed, stamp: stamp, generation: earlierGen), at: 10)
        viewFeed(&earlierMirror, .status, viewStatus(epoch: 43, seq: 9, eventsSeq: 0, generation: earlierGen), at: 10)
        viewFeed(&earlierMirror, .status, viewStatus(epoch: 44, seq: 0, eventsSeq: nil), at: 11)
        let earlierLine = viewScreen(earlierMirror, now: 11, timeZone: zone).earlierRows.first?.line1 ?? ""
        return (openLine, historyLine, earlierLine)
    }

    func expectClock(_ stamp: RoomMemory.Time, _ zone: TimeZone, _ clock: String, _ label: String) {
        let got = lines(stamp, zone)
        for (name, line) in [("open", got.open), ("history", got.history), ("earlier", got.earlier)] {
            expect(line.contains(clock), "\(label) \(name) row is \(clock), got \(line)")
            if !clock.contains("00:00:10") {
                expect(!line.contains("00:00:10"), "\(label) \(name) row is not the UTC clock, got \(line)")
            }
        }
    }

    expectClock(october, chicago, "2025-10-13 19:00:10", "Chicago October")
    expectClock(july, chicago, "2025-07-14 19:00:10", "Chicago July")
    expectClock(january, chicago, "2025-01-14 18:00:10", "Chicago January")
    expectClock(october, kolkata, "2025-10-14 05:30:10", "UTC+5:30")
    expectClock(october, utc, "2025-10-14 00:00:10", "UTC")
    expectClock(october, minusFive, "2025-10-13 19:00:10", "fixed -5h")

    let dated = "disappeared · no longer detected · 128 cells · 2025-10-13 19:00:10"
    expect(eventColumnLines(dated, width: 252) <= 2, "a dated clock still fits line 1, lines \(eventColumnLines(dated, width: 252))")

    var zero = viewOpened()
    viewFeed(&zero, .events, viewEvent(id: 4, seq: 0, state: .candidate, stamp: RoomMemory.Time(sec: 0, nanosec: 0)), at: 10)
    for zone in [chicago, kolkata, utc] {
        let line = viewScreen(zero, timeZone: zone).openRows.first?.line1 ?? ""
        expect(line.contains("time —"), "zero stamp stays time —, got \(line)")
        expect(!line.contains("00:00:00"), "zero stamp is not midnight, got \(line)")
    }
}

private func eventColumnLines(_ text: String, width: CGFloat) -> Int {
    let font = NSFont.monospacedSystemFont(ofSize: 11, weight: .regular)
    let rect = (text as NSString).boundingRect(
        with: NSSize(width: width, height: 10_000),
        options: [.usesLineFragmentOrigin, .usesFontLeading],
        attributes: [.font: font]
    )
    let lineHeight = max(font.boundingRectForFont.height, 1)
    return max(1, Int(ceil(rect.height / lineHeight - 0.05)))
}

func testEventColumnUncertaintyFits() {
    let now: TimeInterval = 400
    guard let lost = viewLoad(.lost, now: now), let restart = viewLoad(.restart, now: now) else { return }
    let lostScreen = viewScreen(lost.mirror, connection: lost.connection, path: false, started: false, now: now)
    let restartScreen = viewScreen(restart.mirror, connection: restart.connection, path: false, started: false, now: now, timeZone: TimeZone(identifier: "America/Chicago")!)
    let history = lostScreen.historyLine ?? ""
    expect(history == "history incomplete: 3 missing (3 lost on the Spark)", "lost fixture history is the full line: \(history)")
    expect(eventColumnLines(history, width: 264) <= 2, "lost history fits the event column: \(history) lines \(eventColumnLines(history, width: 264))")

    let earlier = restartScreen.earlierRows.first
    let line2 = earlier?.line2 ?? ""
    expect(line2.contains("earlier run · latest status unknown"), "restart fixture keeps the uncertainty: \(line2)")
    expect(line2.contains("false-alarm limit not calibrated"), "restart fixture names the missing limit: \(line2)")
    expect(line2.contains("88% of the box was seen"), "restart fixture says what was seen: \(line2)")
    expect(!line2.contains("may be outdated") && !line2.contains("uncalibrated") && !line2.contains("voxel"),
           "restart fixture drops the old words: \(line2)")
    let lines = eventColumnLines(line2, width: 260)
    expect(lines <= 3, "earlier-run line 2 fits three column lines, got \(lines): \(line2)")

    let viewURL = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
        .appendingPathComponent("Sources/LabCommand/LiveRoomView.swift")
    let view = (try? String(contentsOf: viewURL, encoding: .utf8)) ?? ""
    guard let marker = view.range(of: "Text(row.line2)") else {
        expect(false, "the event row draws line 2")
        return
    }
    let slice = view[marker.upperBound...].prefix(240)
    expect(slice.contains("lineLimit(3)"), "event row line 2 allows 3 lines so the uncertainty is not clipped")
}

func runLiveRoomViewTests() {
    testLiveRoomLocalClocks()
    print("  LiveRoom view: event clocks are local yyyy-MM-dd HH:mm:ss; zero stamp stays time —"); passed()
    testLiveRoomStatusLine()
    print("  LiveRoom view: status line reads status.eventsSeq, coverage, stale age, cell"); passed()
    testLiveRoomEventRowsAndHistory()
    print("  LiveRoom view: plain row words; history lines (caught up, missing, lost on the Spark, cannot ask, unverified)"); passed()
    testLiveRoomRestartBanner()
    print("  LiveRoom view: restart banner, earlier-run rows never drawn, greyed earlier map; restart and legacy fixtures"); passed()
    testLiveRoomVoxels()
    print("  LiveRoom view: zlib snapshot centers and one brick frame"); passed()
    testLiveRoomMarkerLifecycle()
    print("  LiveRoom view: candidate, confirmed, accepted outline, suppression, filter"); passed()
    testLiveRoomBannersAndResync()
    print("  LiveRoom view: banner priority and resync placement"); passed()
    testLiveRoomLaunchAndFixtures()
    print("  LiveRoom view: launch env table and offline fixtures"); passed()
    testEventColumnUncertaintyFits()
    print("  LiveRoom view: lost and restart copy fits the event column"); passed()
}
