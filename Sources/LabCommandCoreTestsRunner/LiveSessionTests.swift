import Foundation
import LabCommandCore

private let all = Set(SensorID.allCases)
private let t0 = Date(timeIntervalSince1970: 1_791_974_533)

func testLiveSessionOffAtLaunch() {
    var session = LiveSession()
    expect(session.phase == .off && session.mode == .off, "launches OFF")
    expect(session.sockets.isEmpty, "no socket at launch")
    expect(session.tick(at: 1_000) == nil && session.sockets.isEmpty, "time alone opens nothing")
    expect(session.observe(points: [.mid360: 100], newest: t0, at: 1) == nil && session.points.isEmpty,
           "frames outside a session are ignored")
    expect(session.viewHidden() == nil && session.stopLive() == nil, "nothing to stop while OFF")
    expect(session.sensorEnded(.mid360) == nil, "a stray socket end means nothing while OFF")
}

func testLiveSessionSnapshotBothSensors() {
    var session = LiveSession()
    expect(session.requestSnapshot(at: 10) == .snapshotStarted, "snapshot starts")
    expect(session.mode == .snapshot && session.isCapturing && session.oneFrame, "capturing is SNAPSHOT, one frame each")
    expect(session.sockets == all, "one socket per sensor while capturing")

    expect(session.observe(points: [.mid360: 19_912], newest: t0, at: 10.4) == nil, "one sensor in, still capturing")
    expect(session.sockets == [.jt128], "the Mid-360 socket closes the moment its frame lands")
    expect(session.sensorEnded(.mid360) == nil, "its socket ending is no news")

    let later = t0.addingTimeInterval(0.7)
    let outcome = session.observe(points: [.mid360: 19_912, .jt128: 60_000], newest: later, at: 11.1)
    expect(outcome == .snapshotTaken(capturedAt: later, missing: []), "both in: taken, got \(String(describing: outcome))")
    expect(session.phase == .snapshot(capturedAt: later), "captured at the newest frame's time")
    expect(session.sockets.isEmpty && !session.oneFrame, "every socket closed after the snapshot")
    expect(session.points == [.mid360: 19_912, .jt128: 60_000], "counts held for the badge")
    expect(session.fitGeneration == 1, "camera fits once for the snapshot")

    expect(session.tick(at: 500) == nil && session.sockets.isEmpty, "a held snapshot never reopens a socket")
    expect(session.phase == .snapshot(capturedAt: later), "and stays on screen")
}

func testLiveSessionSnapshotTimeoutKeepsPartial() {
    var session = LiveSession()
    _ = session.requestSnapshot(at: 100)
    _ = session.observe(points: [.mid360: 850], newest: t0, at: 101)
    expect(session.tick(at: 100 + LiveSession.snapshotTimeout - 0.01) == nil, "JT128 still owed a frame before the timeout")
    expect(session.sockets == [.jt128], "only the owed sensor's socket is open")
    let outcome = session.tick(at: 100 + LiveSession.snapshotTimeout)
    expect(outcome == .snapshotTaken(capturedAt: t0, missing: [.jt128]), "timeout keeps what arrived, got \(String(describing: outcome))")
    expect(session.sockets.isEmpty, "the timed-out socket closes")
    expect(session.points == [.mid360: 850] && session.mode == .snapshot, "partial snapshot held")
    expect(LiveSession.snapshotTimeout == 10, "snapshot gives up after 10 s")
}

func testLiveSessionSnapshotSocketEndsWithoutFrame() {
    var session = LiveSession()
    _ = session.requestSnapshot(at: 0)
    _ = session.observe(points: [.jt128: 60_000], newest: t0, at: 1)
    let outcome = session.sensorEnded(.mid360)
    expect(outcome == .snapshotTaken(capturedAt: t0, missing: [.mid360]), "a sensor that gave up ends the capture, got \(String(describing: outcome))")
    expect(session.sockets.isEmpty, "nothing left open")
}

func testLiveSessionEmptySnapshotGoesOff() {
    var session = LiveSession()
    _ = session.requestSnapshot(at: 0)
    let outcome = session.tick(at: LiveSession.snapshotTimeout)
    expect(outcome == .snapshotEmpty, "nothing arrived, got \(String(describing: outcome))")
    expect(session.phase == .off && session.sockets.isEmpty, "back to OFF, sockets closed")
    expect(session.fitGeneration == 1, "the sample refits on the way back to OFF")
}

func testLiveSessionLiveRunsAndStops() {
    var session = LiveSession()
    expect(session.requestLive(at: 50) == .liveStarted, "live starts")
    expect(session.mode == .live && session.sockets == all && !session.oneFrame, "live streams every sensor")
    expect(session.remaining(at: 50) == 120 && session.remaining(at: 80) == 90, "120 s budget counts down")
    expect(LiveSession.liveMaxDuration == 120, "live stops itself after 120 s")

    _ = session.observe(points: [.mid360: 19_000], newest: t0, at: 51)
    expect(session.fitGeneration == 0, "fit waits for the second sensor")
    _ = session.observe(points: [.mid360: 19_100, .jt128: 60_000], newest: t0.addingTimeInterval(0.5), at: 51.5)
    expect(session.fitGeneration == 1, "both in: fit once")
    _ = session.observe(points: [.mid360: 19_200, .jt128: 60_000], newest: t0.addingTimeInterval(1), at: 52)
    _ = session.tick(at: 60)
    expect(session.fitGeneration == 1, "never refits per frame")

    // Stale removal drops a sensor that went quiet: live shows only live points.
    _ = session.observe(points: [.jt128: 60_000], newest: t0.addingTimeInterval(1), at: 56)
    expect(session.points == [.jt128: 60_000], "a quiet sensor leaves the live view")

    let outcome = session.stopLive()
    expect(outcome == .liveStopped(.requested, held: t0.addingTimeInterval(1)), "stop holds the last frames, got \(String(describing: outcome))")
    expect(session.phase == .snapshot(capturedAt: t0.addingTimeInterval(1)) && session.sockets.isEmpty, "held as a snapshot, sockets closed")
    expect(session.points == [.jt128: 60_000], "last frames stay on screen")
    expect(session.fitGeneration == 1, "stopping doesn't move the camera")
}

func testLiveSessionLiveFitSettlesWithOneSensor() {
    var session = LiveSession()
    _ = session.requestLive(at: 0)
    _ = session.observe(points: [.mid360: 19_000], newest: t0, at: 1)
    expect(session.tick(at: 1 + LiveSession.liveSettle - 0.01) == nil && session.fitGeneration == 0, "still settling")
    _ = session.tick(at: 1 + LiveSession.liveSettle)
    expect(session.fitGeneration == 1, "one sensor alone fits after the settle time")
}

func testLiveSessionAutoStopsAt120() {
    var session = LiveSession()
    _ = session.requestLive(at: 1_000)
    _ = session.observe(points: [.mid360: 19_000, .jt128: 60_000], newest: t0, at: 1_001)
    expect(session.tick(at: 1_000 + LiveSession.liveMaxDuration - 0.01) == nil, "still live just before the limit")
    expect(session.sockets == all, "sockets open until then")
    let outcome = session.tick(at: 1_000 + LiveSession.liveMaxDuration)
    expect(outcome == .liveStopped(.timeLimit, held: t0), "stops at exactly 120 s, got \(String(describing: outcome))")
    expect(session.sockets.isEmpty && session.mode == .snapshot, "sockets closed, frames held")
    expect(session.remaining(at: 2_000) == nil, "no countdown after live")
}

func testLiveSessionHiddenStopsLiveOnly() {
    var live = LiveSession()
    _ = live.requestLive(at: 0)
    let outcome = live.viewHidden()
    expect(outcome == .liveStopped(.hidden, held: nil), "hidden stops live, got \(String(describing: outcome))")
    expect(live.phase == .off && live.sockets.isEmpty, "no frames yet: OFF, sockets closed")

    var snapshot = LiveSession()
    _ = snapshot.requestSnapshot(at: 0)
    expect(snapshot.viewHidden() == nil && snapshot.isCapturing, "a snapshot in flight finishes even if hidden")
}

func testLiveSessionIgnoresWrongPhaseRequests() {
    var session = LiveSession()
    _ = session.requestSnapshot(at: 0)
    expect(session.requestSnapshot(at: 1) == nil, "no second snapshot while capturing")
    expect(session.requestLive(at: 1) == nil && session.isCapturing, "no live while capturing")
    expect(session.stopLive() == nil, "stop live means nothing while capturing")

    var live = LiveSession()
    _ = live.requestLive(at: 0)
    expect(live.requestSnapshot(at: 1) == nil && live.isLive, "no snapshot while live")
    expect(live.requestLive(at: 1) == nil && live.remaining(at: 1) == 119, "a second live request doesn't restart the clock")
}

func testLiveSessionNewSessionClearsHeldFrames() {
    var session = LiveSession()
    _ = session.requestSnapshot(at: 0)
    _ = session.observe(points: [.mid360: 1, .jt128: 2], newest: t0, at: 1)
    expect(session.fitGeneration == 1, "first snapshot fits")
    expect(session.requestSnapshot(at: 30) == .snapshotStarted, "a held snapshot can be replaced")
    expect(session.points.isEmpty && session.sockets == all, "the new capture starts clean")
    _ = session.observe(points: [.mid360: 3, .jt128: 4], newest: t0.addingTimeInterval(30), at: 31)
    expect(session.fitGeneration == 2, "each snapshot fits once")
    expect(session.requestLive(at: 40) == .liveStarted && session.points.isEmpty, "live after a snapshot starts clean")
}

func testLiveLaunchAction() {
    expect(LiveLaunchAction.parse(["LabCommand"]) == nil, "no flag, no action")
    expect(LiveLaunchAction.parse(["LabCommand", "--snapshot-on-launch"]) == .snapshot, "snapshot flag")
    expect(LiveLaunchAction.parse(["LabCommand", "--mock", "x", "--live-on-launch"]) == .live, "live flag among others")
    expect(LiveLaunchAction.parse(["LabCommand", "--live-on-launch", "--snapshot-on-launch"]) == .live, "first flag wins")
    expect(LiveLaunchAction.parse(["LabCommand", "--snapshot"]) == nil, "only the exact flag counts")
}

func runLiveSessionTests() {
    testLiveSessionOffAtLaunch()
    print("  live session: OFF at launch, no socket, outside frames ignored"); passed()
    testLiveSessionSnapshotBothSensors()
    print("  live session: snapshot closes each socket on its frame, holds counts, fits once"); passed()
    testLiveSessionSnapshotTimeoutKeepsPartial()
    print("  live session: 10 s snapshot timeout keeps what arrived"); passed()
    testLiveSessionSnapshotSocketEndsWithoutFrame()
    print("  live session: a socket that ends without a frame completes the capture"); passed()
    testLiveSessionEmptySnapshotGoesOff()
    print("  live session: empty snapshot returns to OFF"); passed()
    testLiveSessionLiveRunsAndStops()
    print("  live session: live streams, fits once, drops quiet sensors, stop holds frames"); passed()
    testLiveSessionLiveFitSettlesWithOneSensor()
    print("  live session: live fit settles on one sensor after 2 s"); passed()
    testLiveSessionAutoStopsAt120()
    print("  live session: live auto-stops at 120 s and closes sockets"); passed()
    testLiveSessionHiddenStopsLiveOnly()
    print("  live session: hidden window stops live, not a snapshot in flight"); passed()
    testLiveSessionIgnoresWrongPhaseRequests()
    print("  live session: requests in the wrong phase are ignored"); passed()
    testLiveSessionNewSessionClearsHeldFrames()
    print("  live session: a new session starts clean and fits once"); passed()
    testLiveLaunchAction()
    print("  live session: --snapshot-on-launch / --live-on-launch parse"); passed()
}
