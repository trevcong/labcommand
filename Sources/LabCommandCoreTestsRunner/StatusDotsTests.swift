import Foundation
import LabCommandCore

/// Issue 22. The top bar used to paint `RigSnapshot` straight from the :8766 probe.
/// An offline probe is a red Spark and two grey sensors even while `/room_memory/status`
/// reports 10 Hz on the socket Live Room already holds.
func runStatusDotsTests() {
    let offline = RigSnapshot.offline(reason: "Not probed yet")
    expect(!offline.sparkReachable && offline.mid360.state == .unknown && offline.jt128.state == .unknown,
           "an unprobed rig is red Spark and grey sensors")

    let connected = roomConnected()
    let live = roomStatus(mid: 10, jt: 10, coverage: 0.177996)
    let dots = StatusDots.derive(rig: offline, connection: connected, status: live, isStatusFresh: true)
    expect(dots.sparkReachable, "a connected room socket makes the Spark chip green")
    expect(dots.sparkHelp == StatusDots.roomLinkProbeDown, "help names the room link and the down probe")
    expect(dots.mid360.state == .streaming && dots.mid360.pointRateHz == 10, "Mid-360 streams at the live 10 Hz")
    expect(dots.jt128.state == .streaming && dots.jt128.pointRateHz == 10, "JT128 streams at the live 10 Hz")
    let otherCoverage = roomStatus(mid: 10, jt: 10, coverage: 0.99)
    expect(dots == StatusDots.derive(rig: offline, connection: connected, status: otherCoverage, isStatusFresh: true),
           "coverage is not a chip input")

    let half = StatusDots.derive(
        rig: offline, connection: connected, status: roomStatus(mid: 0, jt: 10), isStatusFresh: true)
    expect(half.sparkReachable, "Spark stays up when one lidar is quiet")
    expect(half.mid360.state == .powered && half.mid360.pointRateHz == 0, "a measured zero on an unprobed sensor is powered")
    expect(half.mid360.detail == "0 Hz on the room link", "a measured zero does not use the unpowered copy")
    expect(half.jt128.state == .streaming && half.jt128.pointRateHz == 10, "the other lidar still streams")

    let unmeasured = StatusDots.derive(
        rig: offline, connection: connected, status: roomStatus(mid: nil, jt: nil), isStatusFresh: true)
    expect(unmeasured.sparkReachable, "Spark is up before either rate is measured")
    expect(unmeasured.mid360.state == .unknown && unmeasured.jt128.state == .unknown,
           "a nil rate leaves the probe sensor")

    let stale = StatusDots.derive(rig: offline, connection: connected, status: live, isStatusFresh: false)
    expect(stale.sparkReachable, "the socket, not the status age, drives the Spark chip")
    expect(stale.mid360.state == .unknown && stale.jt128.state == .unknown,
           "a stale 10 Hz must not paint a sensor green")

    let down = StatusDots.derive(rig: offline, connection: .off, status: live, isStatusFresh: true)
    expect(!down.sparkReachable && down.sparkHelp == nil, "no room socket: Spark stays the red probe")
    expect(down.mid360.state == .unknown && down.jt128.state == .unknown,
           "no room socket: sensors stay the grey probe")

    let connecting = LiveRoomConnection.connecting(url: LiveRoomClient.defaultURL, attempt: 1)
    let dialing = StatusDots.derive(rig: offline, connection: connecting, status: live, isStatusFresh: true)
    expect(!dialing.sparkReachable && dialing.mid360.state == .unknown,
           "connecting is not a live link")

    let probeLive = parseProbe("both-sensors-live")
    expect(probeLive.mid360.state == .streaming && probeLive.jt128.state == .streaming, "fixture is the streaming probe")
    let kept = StatusDots.derive(rig: probeLive, connection: .off, status: nil, isStatusFresh: false)
    expect(kept.sparkReachable && kept.sparkHelp == nil, "a good probe stands while the room socket is down")
    expect(kept.mid360 == probeLive.mid360 && kept.jt128 == probeLive.jt128,
           "a down room socket does not blank a good probe")

    let probeDark = parseProbe("both-sensors-dark")
    expect(probeDark.mid360.state == .dark && probeDark.jt128.state == .dark, "fixture is the dark probe")
    let overridden = StatusDots.derive(rig: probeDark, connection: connected, status: live, isStatusFresh: true)
    expect(overridden.mid360.state == .streaming && overridden.mid360.pointRateHz == 10, "live 10 Hz wins over a dark Mid-360")
    expect(overridden.jt128.state == .streaming && overridden.jt128.pointRateHz == 10, "live 10 Hz wins over a dark JT128")
    expect(overridden.sparkHelp == nil, "the dark fixture's probe already reached the Spark")

    let quietDark = StatusDots.derive(
        rig: probeDark, connection: connected, status: roomStatus(mid: 0, jt: 0), isStatusFresh: true)
    expect(quietDark.mid360 == probeDark.mid360 && quietDark.jt128 == probeDark.jt128,
           "a measured zero keeps a probe that already says dark")

    var powered = RigSnapshot.offline(reason: "Not probed yet")
    powered.sparkReachable = true
    powered.mid360 = SensorReading(id: .mid360, state: .powered, pointRateHz: 0)
    powered.jt128 = SensorReading(id: .jt128, state: .powered, pointRateHz: 0.4)
    let stayPowered = StatusDots.derive(
        rig: powered, connection: connected, status: roomStatus(mid: 0, jt: 0), isStatusFresh: true)
    expect(stayPowered.mid360 == powered.mid360 && stayPowered.jt128 == powered.jt128,
           "a measured zero keeps a probe that already says powered")

    let quietLive = StatusDots.derive(
        rig: probeLive, connection: connected, status: roomStatus(mid: 0, jt: 0), isStatusFresh: true)
    expect(quietLive.mid360.state == .powered && quietLive.jt128.state == .powered, "live zero wins over a streaming probe")
    expect(quietLive.mid360.pointRateHz == 0 && quietLive.jt128.pointRateHz == 0, "the streaming rate does not stick")
    expect(quietLive.mid360.detail == "0 Hz on the room link" && quietLive.jt128.detail == "0 Hz on the room link",
           "the demoted chip names the room-link zero")

    let moved = StatusDots.derive(rig: probeLive, connection: connected, status: live, isStatusFresh: true)
    expect(moved.mid360.state == .streaming && moved.mid360.pointRateHz == 10, "live rate replaces the probe's 9.8")
    expect(moved.jt128.state == .streaming && moved.jt128.pointRateHz == 10, "live rate replaces the probe's 8.6")
    expect(moved.mid360.imuRateHz == probeLive.mid360.imuRateHz, "a streaming probe keeps its imu measurement")

    let floor = StatusDots.derive(
        rig: offline, connection: connected, status: roomStatus(mid: 1, jt: 2), isStatusFresh: true)
    expect(floor.mid360.state == .powered && floor.mid360.detail == "1 Hz on the room link",
           "1 Hz is not above the streaming floor")
    expect(floor.jt128.state == .streaming && floor.jt128.pointRateHz == 2, "2 Hz is above the streaming floor")

    let hidden = """
    {"mid360_ping": false, "jt128_ping": false, "rates": {"livox_pts": 10.0, "jt_pts": 10.0}}
    """
    guard let pingDark = RigStatusDecoder.parse(json: hidden, viewTunnelUp: false) else {
        fputs("FAIL: ping-false status must parse\n", stderr)
        exit(1)
    }
    expect(pingDark.mid360.state == .dark && pingDark.jt128.state == .dark,
           "the probe still ignores a positive rate when ping is false")
    let recovered = StatusDots.derive(rig: pingDark, connection: connected, status: live, isStatusFresh: true)
    expect(recovered.mid360.state == .streaming && recovered.jt128.state == .streaming,
           "a fresh room rate wins over the probe's ping-then-rate rule")

    expectChromeReadsDerivedDots()
}

private func expectChromeReadsDerivedDots() {
    let bar = sourceText("Sources/LabCommand/CommandCenterView.swift")
    expect(bar.contains("state.statusDots.sparkReachable"), "Spark chip reads derived reachability")
    expect(bar.contains("state.statusDots.sparkHelp"), "Spark chip uses the room-link help")
    expect(bar.contains("sensorChip(state.statusDots.mid360)"), "Mid-360 chip reads the derived sensor")
    expect(bar.contains("sensorChip(state.statusDots.jt128)"), "JT128 chip reads the derived sensor")
    expect(!bar.contains("ok: state.rig.sparkReachable"), "Spark chip must not paint the probe alone")
    expect(bar.contains("sensorCard(state.rig.mid360)"), "left-column cards stay on the probe")

    let menu = sourceText("Sources/LabCommand/LabCommandApp.swift")
    expect(menu.contains("appState.statusDots"), "status menu reads the derived dots")
    expect(!menu.contains("if !rig.sparkReachable"), "status menu must not call the Spark offline from the probe alone")

    let state = sourceText("Sources/LabCommand/AppState.swift")
    expect(state.contains("StatusDots.derive"), "AppState recomputes the dots")
    expect(state.contains("room.objectWillChange"), "AppState watches the room client")
}

private func sourceText(_ relative: String) -> String {
    let path = URL(fileURLWithPath: FileManager.default.currentDirectoryPath).appendingPathComponent(relative).path
    guard let text = try? String(contentsOfFile: path, encoding: .utf8) else {
        fputs("FAIL: \(relative) not readable — run tests from the repo root\n", stderr)
        exit(1)
    }
    return text
}

private func parseProbe(_ name: String) -> RigSnapshot {
    guard let snap = RigStatusDecoder.parse(json: requireFixture(name), viewTunnelUp: true) else {
        fputs("FAIL: \(name).json must parse\n", stderr)
        exit(1)
    }
    return snap
}

private func roomConnected() -> LiveRoomConnection {
    .connected(LiveRoomServer(
        name: "spark",
        capabilities: ["connectionGraph"],
        canPublish: false,
        memoryTopics: ["/room_memory/status"]
    ))
}

/// The shape captured on the live bridge: tracking, both lidars at a measured rate.
private func roomStatus(mid: UInt16?, jt: UInt16?, coverage: Float? = 0.177996) -> LiveRoomStatus {
    LiveRoomStatus(
        RoomMemory.MemoryStatus(
            header: RoomMemory.Header(
                roomId: "lab",
                mapEpoch: 3,
                seq: 6811,
                stamp: RoomMemory.Time(sec: 1, nanosec: 0)
            ),
            state: .tracking,
            brickCount: 1851,
            openEventCount: 6,
            coverageKnownFraction: coverage,
            poseResidualM: 0.002,
            windowS: 1,
            mid360Hz: mid,
            jt128Hz: jt,
            droppedWindows: 0,
            eventsSeq: 63,
            publisherGeneration: 1_792_199_320_044
        ),
        receivedAt: 100
    )
}
