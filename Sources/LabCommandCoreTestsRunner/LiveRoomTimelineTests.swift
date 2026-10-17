import Foundation
import LabCommandCore

// MARK: - Live Room timeline: recorder, model, halo
// Real `RoomMemoryMirror` inputs through `LRWire` and the builders below; the recorder observes
// the mirror after every input, as the client does. Times: Spark stamps from `tlBase`, Mac uptime
// from 100 on the same scale, so a status's freshness follows its stamp.

private let tlGeneration: UInt64 = LiveRoomFixture.generation
private let tlBase: Double = 1_760_400_000
/// UTC−5, never `.current`: 1_760_400_000 is 00:00:00 UTC, so 19:00:00 here.
private let tlZone = TimeZone(secondsFromGMT: -18000)!

private func tlTime(_ t: Double) -> RM.Time {
    let whole = t.rounded(.down)
    return RM.Time(sec: Int32(whole), nanosec: UInt32(((t - whole) * 1e9).rounded()))
}

/// A status at `tlBase + at` with chosen state and rates (`LRWire.status` derives its stamp from seq).
private func tlStatus(
    _ generation: UInt64? = tlGeneration,
    seq: UInt64,
    at offset: Double,
    state: RM.MemoryState = .tracking,
    mid: UInt16? = 10,
    jt: UInt16? = nil,
    epoch: UInt64 = 5,
    eventsSeq: UInt64? = nil
) -> Data {
    let message = RM.MemoryStatus(
        header: RM.Header(roomId: LRWire.room, mapEpoch: epoch, seq: seq, stamp: tlTime(tlBase + offset)), state: state,
        brickCount: 1, openEventCount: 0, coverageKnownFraction: nil, poseResidualM: nil, windowS: 1,
        mid360Hz: mid, jt128Hz: jt, droppedWindows: 0, eventsSeq: eventsSeq, publisherGeneration: generation
    )
    return LRWire.cdr(message.encodedCDR, "timeline status s\(seq)")
}

private func tlActivity(_ generation: UInt64? = tlGeneration, seq: UInt64, at offset: Double, active: Bool = false) -> Data {
    let message = RM.Activity(
        header: RM.Header(roomId: LRWire.room, mapEpoch: 5, seq: seq, stamp: tlTime(tlBase + offset)), active: active,
        sensors: active ? .jt128 : [], largestSensor: active ? .jt128 : [], regionCount: active ? 1 : 0,
        voxelCount: active ? 12 : 0, centroid: active ? SIMD3(1, 1, 0.5) : nil,
        aabb: active ? RM.ChangeEvent.Box(min: SIMD3(0.9, 0.9, 0.4), max: SIMD3(1.1, 1.1, 0.6)) : nil,
        sinceS: active ? 0 : nil, publisherGeneration: generation
    )
    return LRWire.cdr(message.encodedCDR, "timeline activity s\(seq)")
}

private func tlEvent(
    _ generation: UInt64? = tlGeneration,
    seq: UInt64,
    id: UInt64,
    state: RM.ChangeEvent.State,
    at offset: Double,
    firstSeen: Double? = nil,
    sensors: RM.Sensors = .jt128,
    epoch: UInt64 = 5
) -> Data {
    let stamp = tlTime(tlBase + offset)
    let message = RM.ChangeEvent(
        header: RM.Header(roomId: LRWire.room, mapEpoch: epoch, seq: seq, stamp: stamp), eventId: id, kind: .appeared,
        state: state, centroid: SIMD3(1, 1, 1), aabb: .init(min: SIMD3(0.9, 0.9, 0.9), max: SIMD3(1.1, 1.1, 1.1)),
        fromCentroid: nil, voxelCount: 8, volumeM3: 0.001, evidence: nil, alpha: nil, sensors: sensors,
        firstSeen: firstSeen.map { tlTime(tlBase + $0) } ?? stamp, lastSeen: stamp, mask: nil, publisherGeneration: generation
    )
    return LRWire.cdr(message.encodedCDR, "timeline event \(id) s\(seq)")
}

/// A mirror and the recorder that observes it after every input.
private struct TLRig {
    var mirror: RoomMemoryMirror
    var recorder = LiveRoomTimelineRecorder()

    init(limits: RoomMemoryMirror.Limits = RoomMemoryMirror.Limits()) {
        mirror = RoomMemoryMirror(limits: limits, tokenSeed: 1)
        _ = mirror.connectionOpened(at: 100, canRequest: false)
        recorder.observe(mirror)
    }

    /// Feeds `payload` at Mac time `100 + offset`.
    mutating func feed(_ topic: LiveRoomTopic, _ payload: Data, at offset: Double) {
        _ = mirror.receive(topic: topic, payload: payload, at: 100 + offset)
        recorder.observe(mirror)
    }

    mutating func tick(at offset: Double) {
        _ = mirror.tick(at: 100 + offset)
        recorder.observe(mirror)
    }

    func timeline(window: TimeInterval = 300, end: Double? = nil, selection: String? = nil, at offset: Double,
                  kinds: Set<RM.ChangeEvent.Kind> = Set(RM.ChangeEvent.Kind.allCases)) -> LiveRoomTimeline {
        LiveRoomTimeline.make(mirror: mirror, recorder: recorder, window: window, end: end.map { tlBase + $0 },
                              selection: selection, kinds: kinds, timeZone: tlZone, now: 100 + offset)
    }
}

private func tlInterval(_ start: Double, _ end: Double) -> LiveRoomTimelineRecorder.Interval {
    LiveRoomTimelineRecorder.Interval(start: tlBase + start, end: tlBase + end)
}

private func tlActivityState(_ timeline: LiveRoomTimeline, at offset: Double) -> LiveRoomTimeline.ActivitySegment.State? {
    let t = tlBase + offset
    return timeline.activity.first { $0.start <= t && t <= $0.end && ($0.start < t || $0.end == t) }?.state
}

// MARK: Recorder

func testTimelineRecorderLists() {
    var rig = TLRig()
    for seq in UInt64(0)...2 {
        rig.feed(.status, tlStatus(seq: seq, at: Double(seq)), at: Double(seq))
        rig.feed(.activity, tlActivity(seq: seq, at: Double(seq) + 0.5, active: seq == 1), at: Double(seq) + 0.5)
    }
    // DEGRADED, then TRACKING with a measured 0 Hz: neither is healthy.
    rig.feed(.status, tlStatus(seq: 3, at: 3, state: .degraded), at: 3)
    rig.feed(.activity, tlActivity(seq: 3, at: 3.5), at: 3.5)
    rig.feed(.status, tlStatus(seq: 4, at: 4, mid: 0, jt: 10), at: 4)
    rig.feed(.activity, tlActivity(seq: 4, at: 4.5), at: 4.5)
    rig.feed(.status, tlStatus(seq: 5, at: 5), at: 5)
    rig.feed(.activity, tlActivity(seq: 5, at: 5.5), at: 5.5)
    let r = rig.recorder
    expect(r.link == [tlInterval(0, 5.5)], "link: every accepted status and activity, one run: \(r.link)")
    expect(r.sensors == [tlInterval(0, 2), tlInterval(5, 5)], "DEGRADED and 0 Hz end sensor health: \(r.sensors)")
    expect(r.coverage == [tlInterval(0.5, 2.5), tlInterval(5.5, 5.5)],
           "coverage only while healthy, never bridging the break: \(r.coverage)")
    expect(r.activity.map(\.covered) == [true, true, true, false, false, true], "activity without healthy sensors is not coverage")
    expect(r.activity.map(\.active) == [false, true, false, false, false, false] && r.activity[1].regionCount == 1
           && r.activity[1].voxelCount == 12, "samples keep active, regions and voxels")
    expect(r.lastStamp == tlBase + 5.5 && r.lastReceivedAt == 105.5, "last stamp and its receipt for extrapolating now")

    // Unmeasured rates, BUILDING, RIG_MOVED, PAUSED: never healthy.
    for (index, state) in [RM.MemoryState.building, .rigMoved, .paused].enumerated() {
        var other = TLRig()
        other.feed(.status, tlStatus(seq: 0, at: 0, state: state), at: 0)
        other.feed(.activity, tlActivity(seq: 0, at: 0.5), at: 0.5)
        expect(other.recorder.sensors.isEmpty && other.recorder.coverage.isEmpty && other.recorder.link.count == 1,
               "state \(index) is not sensor health, and its activity not coverage")
    }
    var unmeasured = TLRig()
    unmeasured.feed(.status, tlStatus(seq: 0, at: 0, mid: nil, jt: nil), at: 0)
    expect(unmeasured.recorder.sensors.isEmpty, "no measured rate is not healthy")
    var both = TLRig()
    both.feed(.status, tlStatus(seq: 0, at: 0, mid: 10, jt: 10), at: 0)
    expect(both.recorder.sensors.count == 1, "both rates above 0 is healthy")

    // A gap over 3 s splits; a backward stamp records a jump and starts new intervals.
    rig.feed(.status, tlStatus(seq: 6, at: 10), at: 10)
    expect(rig.recorder.link == [tlInterval(0, 5.5), tlInterval(10, 10)], "a 4.5 s gap splits link: \(rig.recorder.link)")
    expect(rig.recorder.sensors.last == tlInterval(10, 10) && rig.recorder.sensors.count == 3, "and sensors")
    rig.feed(.status, tlStatus(seq: 7, at: 8), at: 11)
    let jumps = rig.recorder.clockJumps
    expect(jumps.count == 1 && jumps[0].from == tlBase + 10 && jumps[0].to == tlBase + 8, "a backward stamp is a jump: \(jumps)")
    expect(rig.recorder.link.last == tlInterval(8, 8) && rig.recorder.link.count == 3, "the jump starts a new link interval")
    expect(rig.recorder.sensors.last == tlInterval(8, 8) && rig.recorder.sensors.count == 4, "and a new sensors interval")
    rig.feed(.status, tlStatus(seq: 8, at: 9), at: 12)
    expect(rig.recorder.clockJumps.count == 1 && rig.recorder.link.last == tlInterval(8, 9), "time runs on after the jump")

    // Activity half a second behind the last status is reordering between topics, not a jump.
    rig.feed(.activity, tlActivity(seq: 6, at: 8.5), at: 12.1)
    expect(rig.recorder.clockJumps.count == 1 && rig.recorder.link.last == tlInterval(8, 9), "cross-topic order is absorbed")

    // Re-observing, or a tick that changes no message, records nothing.
    let before = rig.recorder
    rig.recorder.observe(rig.mirror)
    rig.recorder.observe(rig.mirror)
    rig.tick(at: 12.5)
    expect(rig.recorder == before, "dedupe by (generation, seq) per topic")
}

func testTimelineRecorderRetention() {
    var rig = TLRig()
    rig.feed(.status, tlStatus(seq: 0, at: 0), at: 0)
    rig.feed(.activity, tlActivity(seq: 0, at: 0.5), at: 0.5)
    expect(rig.recorder.retainedSince == tlBase, "retained since the first stamp seen")
    rig.feed(.status, tlStatus(seq: 1, at: 7300), at: 7300)
    let r = rig.recorder
    expect(r.retainedSince == tlBase + 100, "2 h back from the newest stamp: \(String(describing: r.retainedSince))")
    expect(r.link == [tlInterval(7300, 7300)] && r.sensors == [tlInterval(7300, 7300)], "older intervals dropped: \(r.link)")
    expect(r.activity.isEmpty && r.coverage.isEmpty, "older samples dropped")

    let shown = rig.timeline(window: 7200, end: 7000, at: 7300)
    let unknown = shown.bands.filter { $0.kind == .historyUnknown }
    expect(unknown.count == 1 && unknown[0].start == shown.start && unknown[0].end == tlBase + 100,
           "history left of retainedSince is unknown: \(unknown)")
    expect(unknown.first?.label == "history before 19:01 unknown", "local HH:mm label, got \(String(describing: unknown.first?.label))")
    expect(!shown.bands.contains { $0.kind == .notObserved && $0.start < tlBase + 100 }, "unknown history is not also not-observed")
}

// MARK: Model

func testTimelineSpans() {
    var rig = TLRig()
    rig.feed(.status, tlStatus(seq: 0, at: 0), at: 0)
    // Event 5: candidate (first seen 5 s earlier), confirmed, accepted.
    rig.feed(.events, tlEvent(seq: 0, id: 5, state: .candidate, at: 10, firstSeen: 5), at: 10)
    rig.feed(.events, tlEvent(seq: 1, id: 5, state: .confirmed, at: 20, firstSeen: 5), at: 20)
    rig.feed(.events, tlEvent(seq: 2, id: 5, state: .accepted, at: 30, firstSeen: 5), at: 30)
    // Event 6: still confirmed. Event 7: first received CONFIRMED, first seen at 30.
    rig.feed(.events, tlEvent(seq: 3, id: 6, state: .candidate, at: 35), at: 35)
    rig.feed(.events, tlEvent(seq: 4, id: 6, state: .confirmed, at: 38, firstSeen: 35), at: 38)
    rig.feed(.events, tlEvent(seq: 5, id: 7, state: .confirmed, at: 39, firstSeen: 30), at: 39)
    rig.feed(.status, tlStatus(seq: 1, at: 40, eventsSeq: 5), at: 40)

    let timeline = rig.timeline(at: 42)
    expect(timeline.now == tlBase + 42 && !timeline.nowIsEstimate && timeline.following, "now: the status stamp plus 2 s, fresh")
    expect(timeline.end == tlBase + 42 && timeline.start == tlBase - 258, "follow now: the window ends at now")
    expect(timeline.lanes == [.appeared, .disappeared, .moved, .activity], "four lanes")

    guard let done = timeline.items.first(where: { $0.id == "2-5" }) else {
        expect(false, "the terminal event is item 2-5: \(timeline.items.map(\.id))")
        return
    }
    expect(done.start == tlBase + 5 && done.end == tlBase + 30 && !done.open, "a span from first_seen to the terminal stamp")
    expect(done.ticks.map(\.state) == [.candidate, .confirmed, .accepted] && done.ticks.map(\.seq) == [0, 1, 2], "ticks from the ring")
    expect(done.segments == [
        LiveRoomTimeline.Segment(start: tlBase + 5, end: tlBase + 10, style: .candidate),
        LiveRoomTimeline.Segment(start: tlBase + 10, end: tlBase + 20, style: .candidate),
        LiveRoomTimeline.Segment(start: tlBase + 20, end: tlBase + 30, style: .confirmed),
    ], "candidate dashed, confirmed solid, nothing after accepted: \(done.segments)")
    expect(done.notes.isEmpty && done.lane == .appeared, "a candidate first: no lead-in")

    guard let open = timeline.items.first(where: { $0.id == "6" }) else {
        expect(false, "the open event is item 6")
        return
    }
    expect(open.open && open.end == tlBase + 42 && open.segments.last?.style == .confirmed && open.segments.last?.end == tlBase + 42,
           "an open span runs to now: \(open.segments)")

    guard let late = timeline.items.first(where: { $0.id == "7" }) else {
        expect(false, "the event first received as CONFIRMED is item 7")
        return
    }
    expect(late.segments.first == LiveRoomTimeline.Segment(start: tlBase + 30, end: tlBase + 39, style: .leadIn),
           "first_seen to the first received stamp is a dashed lead-in: \(late.segments)")
    expect(late.notes == ["earlier steps not received"], "the lead-in says so: \(late.notes)")

    let filtered = rig.timeline(at: 42, kinds: [.moved])
    expect(filtered.items.isEmpty && filtered.lanes == [.moved, .activity], "kinds filter the event lanes, not activity")

    // A fresh status vouches up to now; a stale one leaves "no data since".
    expect(!timeline.bands.contains { $0.kind == .notObserved && $0.end == timeline.end }, "fresh: no unobserved sliver at now")
    rig.tick(at: 45)
    let stale = rig.timeline(at: 45)
    expect(stale.nowIsEstimate && stale.now == tlBase + 45, "a stale status makes now an estimate")
    let trailing = stale.bands.filter { $0.kind == .notObserved && $0.end == stale.end }
    expect(trailing.count == 1 && trailing[0].start == tlBase + 40 && trailing[0].label == "no data since 19:00:40",
           "after the last stamp: not observed, labelled: \(trailing)")
    expect(stale.items.first { $0.id == "6" }?.end == tlBase + 45, "an open span still runs to the estimate")
}

func testTimelineRingEviction() {
    var rig = TLRig(limits: RoomMemoryMirror.Limits(eventLog: 3))
    rig.feed(.status, tlStatus(seq: 0, at: 0), at: 0)
    rig.feed(.events, tlEvent(seq: 0, id: 1, state: .candidate, at: 1, firstSeen: 0.5), at: 1)
    for seq in UInt64(1)...3 {
        rig.feed(.events, tlEvent(seq: seq, id: seq + 1, state: .candidate, at: Double(seq) + 1), at: Double(seq) + 1)
    }
    rig.feed(.status, tlStatus(seq: 1, at: 5, eventsSeq: 3), at: 5)
    expect(!rig.mirror.events.contains { $0.event.eventId == 1 } && rig.mirror.openEvents.contains { $0.event.eventId == 1 },
           "setup: event 1 is open and its transitions have left the ring")
    let timeline = rig.timeline(at: 6)
    guard let old = timeline.items.first(where: { $0.id == "1" }) else {
        expect(false, "an open event older than the ring still appears: \(timeline.items.map(\.id))")
        return
    }
    expect(old.ticks.map(\.seq) == [0] && old.segments.first?.style == .leadIn && old.notes == ["earlier steps not received"],
           "no ring entry: the latest message is the first received, after a lead-in: \(old.segments)")
    let unknown = timeline.bands.filter { $0.kind == .historyUnknown }
    expect(unknown.count == 1 && unknown[0].end == tlBase + 2 && unknown[0].label == "history before 19:00 unknown",
           "a full ring: history left of its oldest entry is unknown: \(unknown)")
}

func testTimelineLostBands() {
    // Two neighbours: the band runs between their stamps.
    var two = TLRig()
    two.feed(.events, tlEvent(seq: 0, id: 1, state: .candidate, at: 10), at: 10)
    two.feed(.events, tlEvent(seq: 4, id: 2, state: .candidate, at: 50), at: 50)
    two.feed(.syncEnd, LRWire.syncEnd(tlGeneration, topic: .events, token: .zero, bound: 4, unavailable: [1...3]), at: 55)
    two.feed(.status, tlStatus(seq: 0, at: 60, eventsSeq: 4), at: 60)
    expect(two.mirror.eventHistory.coverage == .missing(pending: [], lost: [1...3]), "setup: 1-3 lost, got \(two.mirror.eventHistory.coverage)")
    let twoBands = two.timeline(at: 61).bands.filter { $0.kind == .lost(3) }
    expect(twoBands.count == 1 && twoBands[0].start == tlBase + 10 && twoBands[0].end == tlBase + 50, "between neighbours: \(twoBands)")
    expect(twoBands.first?.label == "3 transitions lost", "labelled transitions, got \(String(describing: twoBands.first?.label))")

    // Only a lower neighbour: from it to now.
    var lower = TLRig()
    lower.feed(.events, tlEvent(seq: 0, id: 1, state: .candidate, at: 10), at: 10)
    lower.feed(.status, tlStatus(seq: 0, at: 20, eventsSeq: 3), at: 20)
    let lowerLine = lower.timeline(at: 21)
    let lowerBands = lowerLine.bands.filter { $0.kind == .retrieving(3) }
    expect(lowerBands.count == 1 && lowerBands[0].start == tlBase + 10 && lowerBands[0].end == lowerLine.now,
           "a lower neighbour only: to now: \(lowerBands)")
    expect(lowerBands.first?.label == "3 transitions retrieving", "pending reads retrieving")

    // Only an upper neighbour: from the window start.
    var upper = TLRig()
    upper.feed(.events, tlEvent(seq: 3, id: 1, state: .candidate, at: 50), at: 50)
    upper.feed(.status, tlStatus(seq: 0, at: 60, eventsSeq: 3), at: 60)
    let upperLine = upper.timeline(at: 61)
    let upperBands = upperLine.bands.filter { $0.kind == .retrieving(3) }
    expect(upperBands.count == 1 && upperBands[0].start == upperLine.start && upperBands[0].end == tlBase + 50,
           "an upper neighbour only: from the window start: \(upperBands)")

    // No neighbour: a badge, no band.
    var none = TLRig()
    none.feed(.status, tlStatus(seq: 0, at: 60, eventsSeq: 2), at: 60)
    let noneLine = none.timeline(at: 61)
    expect(noneLine.badges == ["3 transitions retrieving · time unknown"], "no neighbour: badge, got \(noneLine.badges)")
    expect(!noneLine.bands.contains { if case .retrieving = $0.kind { return true }; return false }, "and no band")

    // Inverted neighbours (the lower seq stamped after the higher): a clock jump, badge only.
    var inverted = TLRig()
    inverted.feed(.events, tlEvent(seq: 0, id: 1, state: .candidate, at: 50), at: 50)
    inverted.feed(.events, tlEvent(seq: 4, id: 2, state: .candidate, at: 10), at: 51)
    inverted.feed(.syncEnd, LRWire.syncEnd(tlGeneration, topic: .events, token: .zero, bound: 4, unavailable: [1...3]), at: 55)
    inverted.feed(.status, tlStatus(seq: 0, at: 60, eventsSeq: 4), at: 60)
    let invertedLine = inverted.timeline(at: 61)
    expect(invertedLine.badges == ["3 transitions lost · time unknown"], "inverted: badge only, got \(invertedLine.badges)")
    expect(!invertedLine.bands.contains { $0.kind == .lost(3) }, "inverted: no band")

    for line in [twoBands.map(\.label), lowerBands.map(\.label), noneLine.badges, invertedLine.badges].joined() {
        expect(line.contains("transition") && !line.contains("change"), "transitions, never changes: \(line)")
    }
}

func testTimelineActivityLane() {
    var rig = TLRig()
    for seq in UInt64(0)...2 {
        rig.feed(.status, tlStatus(seq: seq, at: Double(seq)), at: Double(seq))
        rig.feed(.activity, tlActivity(seq: seq, at: Double(seq) + 0.5, active: seq == 1), at: Double(seq) + 0.5)
    }
    // Nothing between 2.5 and 10: a recorder gap.
    rig.feed(.status, tlStatus(seq: 3, at: 10), at: 10)
    rig.feed(.activity, tlActivity(seq: 3, at: 10.5), at: 10.5)
    rig.feed(.activity, tlActivity(seq: 4, at: 11.5), at: 11.5)
    let timeline = rig.timeline(end: 20, at: 12)
    expect(tlActivityState(timeline, at: 1) == .active, "covered and active: active, got \(String(describing: tlActivityState(timeline, at: 1)))")
    expect(tlActivityState(timeline, at: 2) == .quiet, "covered and inactive: quiet")
    expect(tlActivityState(timeline, at: 6) == .unobserved, "a recorder gap is hatched, never quiet")
    expect(tlActivityState(timeline, at: 11) == .quiet, "covered again after the gap")
    expect(tlActivityState(timeline, at: 15) == .unobserved && tlActivityState(timeline, at: -100) == .unobserved,
           "after the last sample and before the first: unobserved")
    expect(timeline.activity.first?.start == timeline.start && timeline.activity.last?.end == timeline.end, "the lane spans the window")

    var degraded = TLRig()
    for seq in UInt64(0)...3 {
        degraded.feed(.status, tlStatus(seq: seq, at: Double(seq), state: .degraded), at: Double(seq))
        degraded.feed(.activity, tlActivity(seq: seq, at: Double(seq) + 0.5), at: Double(seq) + 0.5)
    }
    let unhealthy = degraded.timeline(at: 4)
    expect(!unhealthy.activity.contains { $0.state != .unobserved }, "status without healthy sensors: hatched, not quiet: \(unhealthy.activity)")
    expect(!unhealthy.bands.contains { $0.kind == .notObserved && $0.start >= tlBase }, "messages arrived: link is observed")
}

func testTimelineMarkersAndFlags() {
    var rig = TLRig()
    let later = tlGeneration + 1000
    rig.feed(.status, tlStatus(seq: 0, at: 0), at: 0)
    rig.feed(.events, tlEvent(seq: 0, id: 9, state: .candidate, at: 1), at: 1)
    rig.feed(.status, tlStatus(seq: 1, at: 2), at: 2)
    rig.feed(.status, tlStatus(later, seq: 0, at: 5), at: 5)
    expect(rig.recorder.restarts == [tlBase + 5], "a restart: the new generation's first stamp, got \(rig.recorder.restarts)")
    rig.feed(.status, tlStatus(later, seq: 1, at: 6), at: 6)
    expect(rig.recorder.restarts == [tlBase + 5], "once per generation")
    let timeline = rig.timeline(at: 6)
    expect(timeline.markers.contains { $0.kind == .restart && $0.stamp == tlBase + 5 && $0.label == "Spark restarted" },
           "restart marker: \(timeline.markers)")
    guard let earlier = timeline.items.first(where: { $0.id == "earlier-0-9" }) else {
        expect(false, "the earlier run's open event is an item: \(timeline.items.map(\.id))")
        return
    }
    expect(earlier.earlierRun && !earlier.open && earlier.end == tlBase + 2 && earlier.notes.contains("outcome unknown"),
           "earlier run: dimmed, ends at that run's last stamp, outcome unknown: \(earlier)")
    expect(!timeline.unverified, "v2 is verified")

    var jumped = TLRig()
    jumped.feed(.status, tlStatus(seq: 0, at: 10), at: 0)
    jumped.feed(.status, tlStatus(seq: 1, at: 4), at: 1)
    let jumpLine = jumped.timeline(at: 1)
    expect(jumpLine.markers.contains { $0.kind == .clockJump && $0.stamp == tlBase + 4 && $0.label == "Spark clock jumped back" },
           "clock-jump marker: \(jumpLine.markers)")
    expect(jumped.recorder.retainedSince == tlBase + 4, "a backward jump widens what is retained: \(String(describing: jumped.recorder.retainedSince))")
    expect(!jumpLine.bands.contains { $0.kind == .historyUnknown && $0.end > tlBase + 4 }, "what came after the jump is not unknown history")

    // One jump seen on both topics is one jump.
    var both = TLRig()
    both.feed(.status, tlStatus(seq: 0, at: 10), at: 0)
    both.feed(.activity, tlActivity(seq: 0, at: 10.5), at: 0.5)
    both.feed(.status, tlStatus(seq: 1, at: 8), at: 1)
    both.feed(.activity, tlActivity(seq: 1, at: 8.5), at: 1.5)
    expect(both.recorder.clockJumps.count == 1, "one jump on both topics records once: \(both.recorder.clockJumps)")
    expect(both.recorder.coverage == [tlInterval(10.5, 10.5), tlInterval(8.5, 8.5)], "coverage restarts after the jump: \(both.recorder.coverage)")
    expect(both.timeline(at: 1.5).markers.filter { $0.kind == .clockJump }.count == 1, "one clock-jump marker")

    var legacy = TLRig()
    legacy.feed(.status, LRWire.status(nil, seq: 0, epoch: 5, eventsSeq: nil), at: 0)
    expect(legacy.mirror.wire == .legacy && legacy.timeline(at: 0).unverified, "a legacy wire is unverified on the whole axis")
}

func testTimelineTicks() {
    expect(LiveRoomTimeline.clock(1_760_400_010, seconds: false, timeZone: tlZone) == "19:00", "1_760_400_010 at UTC−5 is 19:00")
    expect(LiveRoomTimeline.clock(1_760_400_010, seconds: true, timeZone: tlZone) == "19:00:10", "with seconds")
    expect(LiveRoomTimeline.clock(1_760_400_010, seconds: false, timeZone: TimeZone(secondsFromGMT: 0)!) == "00:00", "the zone is applied")

    let five = LiveRoomTimeline.axisTicks(start: tlBase - 150, end: tlBase + 150, timeZone: tlZone)
    expect(five.map(\.label) == ["18:58", "18:59", "19:00", "19:01", "19:02"], "5 min: a minute apart, local: \(five.map(\.label))")
    expect(five.map(\.stamp) == [-120.0, -60, 0, 60, 120].map { tlBase + $0 }, "on local minute boundaries")
    let minute = LiveRoomTimeline.axisTicks(start: tlBase - 30, end: tlBase + 30, timeZone: tlZone)
    expect(minute.map(\.label) == ["18:59:40", "18:59:50", "19:00:00", "19:00:10", "19:00:20"], "under a minute: seconds: \(minute.map(\.label))")
    // A half-hour zone aligns on its own local boundaries.
    let india = LiveRoomTimeline.axisTicks(start: tlBase, end: tlBase + 1800, timeZone: TimeZone(secondsFromGMT: 19_800)!)
    expect(india.allSatisfy { $0.label.hasSuffix("0") || $0.label.hasSuffix("5") }, "local steps in +05:30: \(india.map(\.label))")
    for window in LiveRoomTimeline.windows {
        for shift in [0.0, 7, 33, 299] {
            let ticks = LiveRoomTimeline.axisTicks(start: tlBase + shift - window, end: tlBase + shift, timeZone: tlZone)
            expect((4...8).contains(ticks.count), "\(window) s window gives 4–8 labels, got \(ticks.count)")
        }
    }

    var rig = TLRig()
    rig.feed(.status, tlStatus(seq: 0, at: 0), at: 0)
    let timeline = rig.timeline(end: 150, at: 0)
    expect(timeline.ticks.map(\.label) == five.map(\.label) && !timeline.following, "make uses the injected zone: \(timeline.ticks.map(\.label))")
    expect(abs(timeline.x(for: tlBase, width: 600) - 300) < 1e-9, "x maps the window onto the width")
}

func testTimelineHitMatchesRows() {
    var rig = TLRig()
    let later = tlGeneration + 1000
    rig.feed(.status, tlStatus(seq: 0, at: 0), at: 0)
    rig.feed(.events, tlEvent(seq: 0, id: 9, state: .candidate, at: 1), at: 1)
    rig.feed(.status, tlStatus(later, seq: 0, at: 5), at: 5)
    rig.feed(.events, tlEvent(later, seq: 0, id: 1, state: .candidate, at: 10), at: 10)
    rig.feed(.events, tlEvent(later, seq: 1, id: 2, state: .candidate, at: 20), at: 20)
    rig.feed(.events, tlEvent(later, seq: 2, id: 2, state: .expired, at: 30), at: 30)
    rig.feed(.events, tlEvent(later, seq: 3, id: 1, state: .confirmed, at: 35), at: 35)
    rig.feed(.status, tlStatus(later, seq: 1, at: 40, eventsSeq: 3), at: 40)

    let server = LiveRoomConnection.connected(LiveRoomServer(name: "timeline-test", capabilities: [], canPublish: false,
                                                            memoryTopics: ["/room_memory/events", "/room_memory/status"]))
    let screen = LiveRoomScreen.make(mirror: rig.mirror, connection: server, requestPathExists: false, started: true, now: 140)
    let rows = (screen.openRows + screen.historyRows + screen.earlierRows).map(\.id)
    expect(rows.sorted() == ["1", "2-2", "earlier-0-9"], "setup: open, history and earlier rows, got \(rows)")
    let timeline = rig.timeline(at: 42)
    expect(timeline.items.map(\.id).sorted() == rows.sorted(), "items are the rows: \(timeline.items.map(\.id))")
    let width = 1000.0
    for id in rows {
        guard let item = timeline.items.first(where: { $0.id == id }), let tick = item.ticks.last else {
            expect(false, "row \(id) has an item with a tick")
            continue
        }
        let x = timeline.x(for: tick.stamp, width: width)
        expect(timeline.hit(x: x, lane: item.lane, width: width) == id, "hit at \(id)'s tick returns its row id")
        expect(timeline.hit(x: x + 3, lane: item.lane, width: width) != nil, "within tolerance")
    }
    expect(timeline.hit(x: 5, lane: .appeared, width: width) == nil, "nothing near: nil")
    expect(timeline.hit(x: 990, lane: .disappeared, width: width) == nil && timeline.hit(x: 990, lane: .activity, width: width) == nil,
           "other lanes hold no item")
    let selected = rig.timeline(selection: "2-2", at: 42)
    expect(selected.items.filter(\.selected).map(\.id) == ["2-2"] && selected.selection == "2-2", "the selection is marked")
}

func testTimelineHalo() {
    // A current map; status reports a JT128 rate, so the JT128 is the map sensor (D30).
    var rig = TLRig()
    rig.feed(.mapBricks, LRWire.snapshot(tlGeneration, epoch: 5, seq: 0, [LRWire.brick(0, occupied: true)]), at: 0)
    rig.feed(.status, tlStatus(seq: 0, at: 0, jt: 10), at: 0)
    rig.feed(.events, tlEvent(seq: 0, id: 1, state: .confirmed, at: 1), at: 1)
    rig.feed(.events, tlEvent(seq: 1, id: 2, state: .confirmed, at: 1, sensors: .mid360), at: 1)
    rig.feed(.events, tlEvent(seq: 2, id: 3, state: .confirmed, at: 1, epoch: 6), at: 1)
    rig.feed(.events, tlEvent(seq: 3, id: 4, state: .candidate, at: 1), at: 1)
    rig.feed(.events, tlEvent(seq: 4, id: 4, state: .expired, at: 1.5), at: 1.5)
    rig.feed(.status, tlStatus(seq: 1, at: 2, jt: 10, eventsSeq: 4), at: 2)
    let mirror = rig.mirror
    expect(mirror.isMapCurrent && mirror.mapEpoch == 5, "setup: the map is current at epoch 5")

    expect(LiveRoomTimeline.halo(selection: nil, mirror: mirror) == LiveRoomTimeline.Halo.none, "no selection: no halo")
    expect(LiveRoomTimeline.halo(selection: "earlier-0-9", mirror: mirror)
           == .unavailable("from an earlier run · location unavailable on this map"), "an earlier run's event")
    expect(LiveRoomTimeline.halo(selection: "99", mirror: mirror) == .unavailable("not in this Mac's history"), "an unknown id")
    expect(LiveRoomTimeline.halo(selection: "7-4", mirror: mirror) == .unavailable("not in this Mac's history"), "a wrong seq")

    var stale = rig
    stale.tick(at: 6)
    expect(LiveRoomTimeline.halo(selection: "1", mirror: stale.mirror)
           == .unavailable("status stale · location unavailable on this map"), "a map that is not current")
    var moved = rig
    moved.feed(.status, tlStatus(seq: 2, at: 3, state: .rigMoved, jt: 10, eventsSeq: 4), at: 3)
    expect(LiveRoomTimeline.halo(selection: "1", mirror: moved.mirror)
           == .unavailable("rig moved · location unavailable on this map"), "RIG_MOVED")
    var building = rig
    building.feed(.status, tlStatus(seq: 2, at: 3, state: .building, jt: 10, eventsSeq: 4), at: 3)
    expect(LiveRoomTimeline.halo(selection: "1", mirror: building.mirror)
           == .unavailable("building reference · location unavailable on this map"), "BUILDING")

    let ownFrame = LiveRoomTimeline.Halo.unavailable("this sensor's events are in its own frame · location unavailable on this map")
    expect(LiveRoomTimeline.halo(selection: "2", mirror: mirror) == ownFrame, "a Mid-360 event on the JT128 map")
    // No JT128 rate: the Mid-360 is the map sensor, and a JT128 event is the other sensor's.
    var midMap = rig
    midMap.feed(.status, LRWire.status(tlGeneration, seq: 2, epoch: 5, eventsSeq: 4), at: 3)
    expect(midMap.mirror.isMapCurrent, "setup: still current under a Mid-360-only status")
    expect(LiveRoomTimeline.halo(selection: "1", mirror: midMap.mirror) == ownFrame, "a JT128 event on the Mid-360 map")

    expect(LiveRoomTimeline.halo(selection: "3", mirror: mirror) == .unavailable("newer than the map shown"), "an epoch past the map")

    guard case .marker(let halo) = LiveRoomTimeline.halo(selection: "1", mirror: mirror) else {
        expect(false, "a confirmed JT128 event on the current JT128 map gets a halo")
        return
    }
    expect(halo.id == "halo" && halo.dashed && halo.fillAlpha == 0 && halo.onTop && halo.arrow == nil, "halo style")
    expect(!halo.countsAsChange && !halo.countsAsCandidate, "the halo counts as neither change nor candidate")
    expect(halo.corners.count == 8 && abs(halo.corners[0].x - 0.85) < 1e-5 && abs(halo.corners[6].z - 1.15) < 1e-5,
           "the box inflated by 0.05 m: \(halo.corners)")
    expect(!RM.ChangeEvent.Kind.allCases.contains { LiveRoomMarkers.color($0) == halo.color }, "a neutral colour, not a kind's")
    guard case .marker = LiveRoomTimeline.halo(selection: "4-4", mirror: mirror) else {
        expect(false, "a terminal row id resolves too")
        return
    }

    let server = LiveRoomConnection.connected(LiveRoomServer(name: "timeline-test", capabilities: [], canPublish: false,
                                                            memoryTopics: ["/room_memory/map_bricks", "/room_memory/status"]))
    let screen = LiveRoomScreen.make(mirror: mirror, connection: server, requestPathExists: false, started: true, now: 102, selection: "1")
    let withHalo = LiveRoomTimeline.markers(screen.markers, with: .marker(halo))
    expect(withHalo.last == halo && Array(withHalo.dropLast()) == screen.markers, "the halo is drawn last, over the screen's markers")
    expect(withHalo.filter(\.countsAsChange).count == screen.markers.filter(\.countsAsChange).count
           && withHalo.filter(\.countsAsCandidate).count == screen.markers.filter(\.countsAsCandidate).count,
           "adding the halo changes no caption count (\(screen.caption))")
    expect(LiveRoomTimeline.markers(screen.markers, with: ownFrame) == screen.markers
           && LiveRoomTimeline.markers(screen.markers, with: .none) == screen.markers, "no halo: the screen's markers")
    expect(ownFrame.note == "this sensor's events are in its own frame · location unavailable on this map"
           && LiveRoomTimeline.Halo.marker(halo).note == nil && LiveRoomTimeline.Halo.none.note == nil,
           "the action bar's location note is the unavailable reason")
}

// MARK: The Room tab's wiring (review round 2)
//
// The SwiftUI view cannot run here, so these drive what it binds to: the client's recorder and
// desk over a live socket, and the fixture's one-shot recorder. Ids go timeline → column → action.

private func tlRepoRoot() -> URL {
    URL(fileURLWithPath: #filePath).deletingLastPathComponent().deletingLastPathComponent().deletingLastPathComponent()
}

@MainActor
private func tlSpin(_ seconds: TimeInterval, until done: () -> Bool) {
    let deadline = Date().addingTimeInterval(seconds)
    while Date() < deadline, !done() { RunLoop.main.run(until: Date().addingTimeInterval(0.02)) }
}

/// Every item the timeline draws is a row of the event column, resolves to its event, and `hit`
/// at its middle selects it.
private func tlCheckIdsMatchTheColumn(_ timeline: LiveRoomTimeline, _ screen: LiveRoomScreen, _ mirror: RoomMemoryMirror,
                                      _ what: String) {
    let rows = Set((screen.openRows + screen.earlierRows + screen.historyRows).map(\.id))
    for item in timeline.items {
        expect(rows.contains(item.id), "\(what): timeline item \(item.id) is a column row (\(rows.sorted()))")
        expect(LiveRoomOperator.entry(for: item.id, mirror: mirror)?.entry.event.eventId == item.eventId,
               "\(what): item \(item.id) resolves to event \(item.eventId)")
        let mid = timeline.x(for: (Swift.max(item.start, timeline.start) + Swift.min(item.end, timeline.end)) / 2, width: 1000)
        expect(timeline.hit(x: mid, lane: item.lane, width: 1000) != nil, "\(what): a click on item \(item.id) selects an item")
    }
}

@MainActor
func testRoomTimelineAndActionsOverTheBridge() {
    var publisher = LRPublisher(seed: 21, generation: 1_760_400_000_021, legacy: false)
    _ = publisher.map()
    for _ in 0..<5 { _ = publisher.event() }
    let bridge = LRFakeBridge(capabilities: ["connectionGraph", "services"], serviceType: LiveRoomClient.requestServiceType,
                              publisher: publisher)
    bridge.operatorOutcome = .applied
    bridge.start()
    let client = LiveRoomClient(url: bridge.url)
    client.start()
    defer { client.stop(); bridge.stop() }
    // The fake stamps each topic by its own seq: wait until the status clock passes the newest event.
    tlSpin(8) { client.operatorPath && client.mirror.eventHistory.coverage == .completeThrough(4)
        && (client.mirror.status?.stamp.sec ?? 0) > client.mirror.events.map(\.event.header.stamp.sec).max() ?? .max }
    expect(client.mirror.eventHistory.coverage == .completeThrough(4), "bridge: the five events arrive")

    // The client feeds the recorder from every message; nothing else does.
    let recorder = client.timelineRecorder
    expect(recorder.link.count == 1 && recorder.lastStamp == LiveRoomTimelineRecorder.seconds(client.mirror.status!.stamp),
           "bridge: the recorder follows the client's status (\(recorder.link))")
    expect(recorder.sensors.count == 1, "bridge: TRACKING with a Mid-360 rate is a healthy-sensors interval")
    let before = recorder.link[0].end
    tlSpin(2) { client.timelineRecorder.link.last?.end ?? 0 > before }
    expect(client.timelineRecorder.link[0].end > before, "bridge: later statuses extend the link interval")

    let now = ProcessInfo.processInfo.systemUptime
    let timeline = LiveRoomTimeline.make(mirror: client.mirror, recorder: client.timelineRecorder, window: LiveRoomTimeline.windows[2],
                                         timeZone: tlZone, now: now)
    let screen = LiveRoomScreen.make(mirror: client.mirror, connection: client.connection, requestPathExists: true, started: true, now: now)
    expect(!timeline.items.isEmpty && timeline.now != nil && !timeline.nowIsEstimate, "bridge: the timeline draws the events, now is live")
    tlCheckIdsMatchTheColumn(timeline, screen, client.mirror, "bridge")

    // Selected on the timeline, acted on from the action bar's buttons, sent to the node.
    guard let item = timeline.items.first(where: \.open),
          let (entry, earlier) = LiveRoomOperator.entry(for: item.id, mirror: client.mirror) else {
        expect(false, "bridge: seed 21 leaves an open event on the timeline: \(timeline.items.map { ($0.id, $0.state) })")
        return
    }
    let buttons = LiveRoomOperator.buttons(for: entry, earlier: earlier, mirror: client.mirror, operatorPath: client.operatorPath,
                                           pending: client.operatorDesk.latest(for: entry.event.eventId))
    guard let button = buttons.first(where: \.enabled) else {
        expect(false, "bridge: the open \(entry.event.state) event has an enabled action: \(buttons)")
        return
    }
    expect(client.act(button.kind, on: entry), "bridge: \(button.kind.title) goes out")
    tlSpin(5) { bridge.operatorCalls.count == 1 }
    expect(bridge.operatorCalls.first?.eventId == entry.event.eventId
           && bridge.operatorCalls.first?.expectedState == entry.event.state,
           "bridge: the node gets the action for the selected event in its shown state")
}

func testRoomTimelineInFixtureMode() {
    for mode in [LiveRoomFixture.Mode.map, .lost, .restart, .legacy] {
        guard let fixture = try? LiveRoomFixture.load(mode: mode, root: tlRepoRoot(), now: 100) else {
            expect(false, "fixture \(mode.rawValue) loads")
            continue
        }
        let recorder = LiveRoomTimelineRecorder.once(fixture.mirror)
        expect(recorder.link.count == 1 && recorder == LiveRoomTimelineRecorder.once(fixture.mirror),
               "fixture \(mode.rawValue): a one-shot recorder of the status, the same every render")
        let timeline = LiveRoomTimeline.make(mirror: fixture.mirror, recorder: recorder, window: LiveRoomTimeline.windows[0],
                                             timeZone: tlZone, now: 100)
        let screen = LiveRoomScreen.make(mirror: fixture.mirror, connection: fixture.connection, requestPathExists: false,
                                         started: true, now: 100)
        expect(!timeline.items.isEmpty, "fixture \(mode.rawValue): the timeline draws the fixture's events")
        expect(timeline.unverified == (mode == .legacy), "fixture \(mode.rawValue): only legacy is unverified")
        tlCheckIdsMatchTheColumn(timeline, screen, fixture.mirror, "fixture \(mode.rawValue)")
    }

    // The sidecar lines the Room view appends: what it drew under the map, for a launched app to check.
    guard let map = try? LiveRoomFixture.load(mode: .map, root: tlRepoRoot(), now: 100) else { return }
    let recorder = LiveRoomTimelineRecorder.once(map.mirror)
    let plain = LiveRoomTimeline.make(mirror: map.mirror, recorder: recorder, window: LiveRoomTimeline.windows[0], timeZone: tlZone, now: 100)
    let lines = LiveRoomTimeline.sidecarLines(timeline: plain, halo: .none, buttons: [])
    expect(lines.first == "timeline appeared,disappeared,moved,activity · \(plain.items.count) items" && lines.count == plain.items.count + 1,
           "sidecar: lanes and one line per item, no actions without a selection: \(lines)")
    guard let open = plain.items.first(where: \.open) else {
        expect(false, "sidecar: the map fixture has an open event")
        return
    }
    let picked = LiveRoomTimeline.make(mirror: map.mirror, recorder: recorder, window: LiveRoomTimeline.windows[0], selection: open.id,
                                       timeZone: tlZone, now: 100)
    let (entry, earlier) = LiveRoomOperator.entry(for: open.id, mirror: map.mirror)!
    let buttons = LiveRoomOperator.buttons(for: entry, earlier: earlier, mirror: map.mirror, operatorPath: false, pending: nil)
    let halo = LiveRoomTimeline.halo(selection: open.id, mirror: map.mirror)
    let selected = LiveRoomTimeline.sidecarLines(timeline: picked, halo: halo, buttons: buttons)
    expect(selected.contains { $0.hasPrefix("  \(open.id) ") && $0.hasSuffix(" open selected") },
           "sidecar: the selected item is marked: \(selected)")
    expect(selected.last == "actions " + buttons.map { "\($0.kind.rawValue) off (this Spark can't take actions)" }.joined(separator: ", ")
           && !buttons.isEmpty, "sidecar: fixture mode shows the buttons, disabled with the reason: \(selected)")
    expect(selected.contains { $0.hasPrefix("halo ") }, "sidecar: the halo's state is a line: \(selected)")
}

func runLiveRoomTimelineTests() {
    testTimelineRecorderLists()
    print("  LiveRoom timeline: recorder keeps link, sensors and coverage apart; health breaks, gaps, clock jumps, dedupe"); passed()
    testTimelineRecorderRetention()
    print("  LiveRoom timeline: 2 h retention and history before retainedSince unknown"); passed()
    testTimelineSpans()
    print("  LiveRoom timeline: spans from first_seen to terminal or now, lead-in, now estimate when stale"); passed()
    testTimelineRingEviction()
    print("  LiveRoom timeline: an open event older than the ring still appears; full ring is unknown history"); passed()
    testTimelineLostBands()
    print("  LiveRoom timeline: lost and retrieving bands by neighbour stamps; badges without or inverted"); passed()
    testTimelineActivityLane()
    print("  LiveRoom timeline: activity hatched in gaps and without healthy sensors, never quiet"); passed()
    testTimelineMarkersAndFlags()
    print("  LiveRoom timeline: restart and clock-jump markers, earlier run outcome unknown, legacy unverified"); passed()
    testTimelineTicks()
    print("  LiveRoom timeline: local tick labels at an injected zone, 4–8 per window"); passed()
    testTimelineHitMatchesRows()
    print("  LiveRoom timeline: hit returns the event column's row ids"); passed()
    testTimelineHalo()
    print("  LiveRoom timeline: halo gates (earlier, unresolved, not current, rig moved, sensor frame, epoch) and the marker"); passed()
    testRoomTimelineInFixtureMode()
    print("  LiveRoom timeline: fixture mode draws the fixture's events from a one-shot recorder; ids match the column"); passed()
    MainActor.assumeIsolated {
        testRoomTimelineAndActionsOverTheBridge()
        print("  LiveRoom timeline: over a socket the client feeds the recorder; timeline ids select, an action reaches the node"); passed()
    }
}
