import Foundation
import LabCommandCore

// MARK: - Operator actions (protocol D46): the codec, the mirror's reductions of what they cause

private typealias R = RoomMemory
private typealias Mirror = RoomMemoryMirror

private let g1: UInt64 = 1_760_400_000_000

private func operatorGolden(_ name: String) -> Data {
    let path = "Tests/Fixtures/room/golden/hand/\(name).hex"
    guard let text = try? String(contentsOfFile: path, encoding: .utf8) else {
        expect(false, "\(path) not found — run tests from repo root")
        return Data()
    }
    var bytes: [UInt8] = []
    for line in text.split(separator: "\n") {
        for token in line.prefix(while: { $0 != "#" }).split(separator: " ") {
            guard let byte = UInt8(token, radix: 16) else {
                expect(false, "\(path): bad hex byte '\(token)'")
                return Data()
            }
            bytes.append(byte)
        }
    }
    return Data(bytes)
}

private let operatorEvent: UInt64 = (g1 << 20) + 3
private let t10 = R.Time(sec: 1_760_400_010, nanosec: 250_000_000)

/// operator_action_le.hex: the §3.6 example, an accept.
private func operatorAccept() -> R.OperatorAction {
    R.OperatorAction(header: R.Header(roomId: "lab", mapEpoch: 43, seq: 0, stamp: t10), action: .accept,
                     eventId: operatorEvent, expectedState: .confirmed, expectedGeneration: g1,
                     operatorId: "trevor", actionId: "0e8a3b5c-7721-4d90-b406-5913c2ae6f01")
}

/// operator_action_text_le.hex: a dismiss with a note.
private func operatorDismiss() -> R.OperatorAction {
    R.OperatorAction(header: R.Header(roomId: "lab", mapEpoch: 43, seq: 1, stamp: R.Time(sec: 1_760_400_012, nanosec: 0)),
                     action: .dismiss, eventId: operatorEvent + 1, expectedState: .candidate, expectedGeneration: g1,
                     text: "left on purpose", operatorId: "astra", actionId: "5a1e7c3d-9b02-4f61-8e33-c4d5e6f70819")
}

/// operator_action_be.hex: a confirm, decode-only.
private func operatorConfirm() -> R.OperatorAction {
    var action = operatorAccept()
    action.header = R.Header(roomId: "lab", mapEpoch: 43, seq: 2, stamp: R.Time(sec: 1_760_400_011, nanosec: 500_000_000))
    action.action = .confirm
    action.expectedState = .candidate
    return action
}

func testOperatorActionGoldens() {
    let cases: [(String, R.OperatorAction, Bool)] = [
        ("operator_action_le", operatorAccept(), true),
        ("operator_action_text_le", operatorDismiss(), true),
        ("operator_action_be", operatorConfirm(), false),
    ]
    for (name, value, exact) in cases {
        let bytes = operatorGolden(name)
        do {
            let decoded = try R.OperatorAction.decode(cdr: bytes)
            expect(decoded == value, "golden \(name) decodes to its value")
            if exact {
                let encoded = try value.encodedCDR()
                expect(encoded == bytes, "golden \(name): the encoder reproduces the hand bytes (\(encoded.count) vs \(bytes.count))")
            }
            expect(try R.OperatorAction.decode(cdr: bytes + Data(count: 3)) == value, "golden \(name): 3 pad bytes are fine")
        } catch {
            expect(false, "golden \(name): \(error)")
        }
        for length in 0..<bytes.count {
            expect((try? R.OperatorAction.decode(cdr: bytes.prefix(length))) == nil, "golden \(name) cut at \(length) does not decode")
        }
        expect((try? R.OperatorAction.decode(cdr: bytes + Data(count: 4))) == nil, "golden \(name): 4 trailing bytes are refused")
    }
    expect(operatorGolden("operator_action_le").count == 129, "the §3.6 example is 129 bytes")
}

/// One refusal per D16 rule, the same list as the C++ and Python suites.
func testOperatorActionRefusals() {
    let changes: [(String, (inout R.OperatorAction) -> Void)] = [
        ("stamp {0, 0}", { $0.header.stamp = R.Time(sec: 0, nanosec: 0) }),
        ("event without a state", { $0.expectedState = nil }),
        ("state without an event", { $0.eventId = nil }),
        ("EVENT_NONE as an id", { $0.eventId = R.OperatorAction.eventNone }),
        ("ACCEPT of a CANDIDATE", { $0.expectedState = .candidate }),
        ("CONFIRM of a CONFIRMED", { $0.action = .confirm }),
        ("DISMISS of an EXPIRED", { $0.action = .dismiss; $0.expectedState = .expired }),
        ("room-level with an event", { $0.action = .pause }),
        ("LABEL without text", { $0.action = .label; $0.expectedState = .candidate }),
        ("generation 0", { $0.expectedGeneration = 0 }),
        ("text over 256 bytes", { $0.text = String(repeating: "x", count: 257) }),
        ("text with a NUL", { $0.text = "a\u{0}b" }),
        ("empty operator_id", { $0.operatorId = "" }),
        ("operator_id over 64 bytes", { $0.operatorId = String(repeating: "x", count: 65) }),
        ("uppercase UUID", { $0.actionId = "0E8A3B5C-7721-4D90-B406-5913C2AE6F01" }),
        ("UUID without hyphens", { $0.actionId = "0e8a3b5c77214d90b4065913c2ae6f01" }),
        ("UUID with a non-hex digit", { $0.actionId = "0e8a3b5c-7721-4d90-b406-5913c2ae6f0g" }),
        ("empty room_id", { $0.header.roomId = "" }),
    ]
    for (what, change) in changes {
        var action = operatorAccept()
        change(&action)
        expect((try? action.encodedCDR()) == nil, "OperatorAction encoder refuses: \(what)")
    }
    var limits = operatorAccept()
    limits.text = String(repeating: "x", count: 256)
    limits.operatorId = String(repeating: "y", count: 64)
    expect((try? R.OperatorAction.decode(cdr: try limits.encodedCDR())) == limits, "OperatorAction: the byte limits are inclusive")
    var room = operatorAccept()
    room.action = .rereference
    room.eventId = nil
    room.expectedState = nil
    let roomBytes = (try? room.encodedCDR()) ?? Data()
    expect([UInt8](roomBytes[(4 + 40)..<(4 + 48)]) == [UInt8](repeating: 0xFF, count: 8) && roomBytes[4 + 48] == 0xFF,
           "a room-level action carries EVENT_NONE and STATE_NONE")
    expect((try? R.OperatorAction.decode(cdr: roomBytes)) == room, "a room-level action round-trips")
    func patched(_ at: Int, _ value: [UInt8]) -> Data {
        var bytes = [UInt8](operatorGolden("operator_action_le"))
        bytes.replaceSubrange((4 + at)..<(4 + at + value.count), with: value)
        return Data(bytes)
    }
    for (what, data) in [("action 9", patched(32, [9])), ("STATE_NONE with an event", patched(48, [0xFF])),
                         ("state 7", patched(48, [7])), ("generation 0", patched(56, [UInt8](repeating: 0, count: 8))),
                         ("stamp 0", patched(24, [UInt8](repeating: 0, count: 8))), ("uppercase UUID", patched(88, [0x41]))] {
        expect((try? R.OperatorAction.decode(cdr: data)) == nil, "OperatorAction decoder rejects: \(what)")
    }
}

func testOperatorOutcomeResponses() {
    for outcome in R.OperatorOutcome.allCases {
        expect(R.OperatorOutcome.decodeResponse(Data([0, 1, 0, 0, outcome.rawValue])) == outcome, "response \(outcome) parses")
        expect(R.OperatorOutcome.decodeResponse(Data([0, 0, 0, 0, outcome.rawValue, 0, 0, 0])) == outcome, "response \(outcome) with padding parses")
    }
    expect(R.OperatorOutcome.decodeResponse(Data([0, 1, 0, 0])) == nil, "a short response is unreadable")
    expect(R.OperatorOutcome.decodeResponse(Data([0, 1, 0, 0, 12])) == nil, "an unknown outcome is unreadable")
    expect(R.OperatorOutcome.decodeResponse(Data([7, 1, 0, 0, 0])) == nil, "a bad encapsulation is unreadable")
    expect(R.OperatorOutcome.decodeResponse(Data([0, 1, 0, 0, 0, 0, 0, 0, 0])) == nil, "more than 3 pad bytes is unreadable")
}

/// A connected mirror with a current map at (epoch 5, seq 10) and a fresh status of g1.
private func currentMirror(at now: TimeInterval = 100) -> Mirror {
    var mirror = Mirror(tokenSeed: 3)
    _ = mirror.connectionOpened(at: now, canRequest: true)
    _ = mirror.receive(topic: .mapBricks, payload: LRWire.snapshot(g1, epoch: 5, seq: 10, [LRWire.brick(0), LRWire.brick(1)]), at: now)
    _ = mirror.receive(topic: .status, payload: LRWire.status(g1, seq: 1, epoch: 5, eventsSeq: nil), at: now)
    expect(mirror.isMapCurrent, "operator setup: the map is current")
    return mirror
}

/// An accept on the non-map sensor bumps map_epoch without changing the published bricks: the
/// Spark sends an empty delta. The mirror applies it and the chain stays current at the new epoch.
func testOperatorEmptyDeltaApplies() {
    var mirror = currentMirror()
    let bricks = mirror.bricks
    let revision = mirror.mapRevision
    _ = mirror.receive(topic: .mapBricks, payload: LRWire.delta(g1, epoch: 6, seq: 11, base: 5, []), at: 101)
    _ = mirror.receive(topic: .status, payload: LRWire.status(g1, seq: 2, epoch: 6, eventsSeq: nil), at: 101)
    expect(mirror.mapEpoch == 6, "empty delta: the chain moves to epoch 6, got \(String(describing: mirror.mapEpoch))")
    expect(mirror.mapSeq == 11, "empty delta: applied at seq 11")
    expect(mirror.bricks == bricks, "empty delta: the bricks are unchanged")
    expect(mirror.mapRevision == revision, "empty delta: no redraw")
    expect(mirror.isMapCurrent, "empty delta: the map stays current")
    expect(mirror.activity.rejected == 0 && mirror.activity.gaps == 0, "empty delta: neither rejected nor a gap")
}

// MARK: The mirror's reductions of what the actions cause (no new mirror state)

private func eventEntry(_ mirror: Mirror, _ id: UInt64) -> LiveRoomEvent? { mirror.eventsById[id] }

/// Confirm, dismiss and accept each reach the Mac only as the Spark's own transitions; the mirror
/// already renders them (protocol §4.2, §6 rule 7).
func testOperatorMirrorReductions() {
    var mirror = currentMirror()
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 0, id: 7, state: .candidate), at: 101)
    expect(mirror.openEvents.count == 1, "a candidate is open")
    // Confirm: CONFIRMED, the next seq; still open, now a change.
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 1, id: 7, state: .confirmed), at: 102)
    let confirmed = LiveRoomMarkers.draw(mirror: mirror)
    expect(mirror.openEvents.count == 1 && confirmed.count == 1 && confirmed[0].countsAsChange, "confirm: open and counted as a change")
    // Dismiss of a CANDIDATE (the new §4.2 row): REVERTED, history only, no marker.
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 2, id: 8, state: .candidate), at: 103)
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 3, id: 8, state: .reverted), at: 104)
    expect(!mirror.openEvents.contains { $0.event.eventId == 8 } && eventEntry(mirror, 8)?.event.state == .reverted,
           "dismiss from CANDIDATE: REVERTED leaves the open events")
    expect(!LiveRoomMarkers.draw(mirror: mirror).contains { $0.id.hasSuffix("-8") || $0.id == "8" }, "dismiss: no marker")
    // Accept: ACCEPTED at epoch 6 before the map reaches 6 waits ("map updating"), uncounted.
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 4, id: 7, state: .accepted, epoch: 6), at: 105)
    _ = mirror.receive(topic: .status, payload: LRWire.status(g1, seq: 2, epoch: 5, eventsSeq: 4), at: 105)
    let waiting = LiveRoomMarkers.draw(mirror: mirror).filter { $0.id == "4-7" }
    expect(waiting.count == 1 && !waiting[0].countsAsChange && !waiting[0].countsAsCandidate && waiting[0].dashed,
           "accept before its map: outlined, waiting, not counted")
    _ = mirror.receive(topic: .mapBricks, payload: LRWire.delta(g1, epoch: 6, seq: 11, base: 5, []), at: 106)
    _ = mirror.receive(topic: .status, payload: LRWire.status(g1, seq: 3, epoch: 6, eventsSeq: 4), at: 106)
    expect(LiveRoomMarkers.draw(mirror: mirror).isEmpty && mirror.openEvents.isEmpty, "accept after its (empty) delta: history only")
}

// MARK: The operator model

private let opNow: TimeInterval = 200

/// A current mirror with a candidate (id 7) and its row entry.
private func mirrorWithCandidate(state: R.ChangeEvent.State = .candidate) -> (Mirror, LiveRoomEvent) {
    var mirror = currentMirror(at: opNow)
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 0, id: 7, state: state), at: opNow)
    return (mirror, mirror.eventsById[7]!)
}

private func build(_ kind: LiveRoomOperator.Kind, _ entry: LiveRoomEvent, _ mirror: Mirror, _ desk: inout LiveRoomOperator,
                   id: Int = 1) -> R.OperatorAction {
    let action = LiveRoomOperator.make(kind, entry: entry, mirror: mirror, seq: desk.takeSeq(), operatorId: "trevor",
                                       stamp: t10, actionId: String(format: "00000000-0000-4000-8000-%012d", id))
    expect(action != nil, "operator: \(kind) builds")
    return action!
}

func testOperatorActionsAreBuiltFromWhatTheOperatorSees() {
    let (mirror, entry) = mirrorWithCandidate()
    var desk = LiveRoomOperator()
    let action = build(.confirm, entry, mirror, &desk)
    expect(action.header.mapEpoch == 5 && action.expectedGeneration == g1 && action.eventId == 7 && action.expectedState == .candidate,
           "operator: an action carries the chain epoch, the active generation and the shown state")
    expect(action.header.seq == 0 && desk.takeSeq() == 1, "operator: seq counts this client's actions")
    expect((try? R.OperatorAction.decode(cdr: try action.encodedCDR())) == action, "operator: a built action encodes")
    expect(LiveRoomOperator.make(.accept, entry: entry, mirror: mirror, seq: 0, operatorId: "trevor", stamp: t10,
                                 actionId: "00000000-0000-4000-8000-000000000002") == nil, "operator: no accept of a candidate")
    expect(LiveRoomOperator.make(.confirm, entry: entry, mirror: Mirror(), seq: 0, operatorId: "trevor", stamp: t10,
                                 actionId: "00000000-0000-4000-8000-000000000002") == nil, "operator: no action without a generation")
}

func testOperatorActionFinishesOnItsTransition() {
    var (mirror, entry) = mirrorWithCandidate()
    var desk = LiveRoomOperator()
    let action = build(.confirm, entry, mirror, &desk)
    desk.sent(action, kind: .confirm, mirror: mirror, delivered: true, at: opNow)
    expect(desk.latest(for: 7)?.phase == .sending(since: opNow), "operator: sending")
    desk.response(actionId: action.actionId, outcome: .applied, at: opNow + 1)
    expect(desk.latest(for: 7)?.phase == .applied(since: opNow + 1), "operator: applied, waiting for the transition")
    expect(LiveRoomOperator.buttons(for: entry, earlier: false, mirror: mirror, operatorPath: true, pending: desk.latest(for: 7))
        .allSatisfy { !$0.enabled && $0.reason == "an action on this event is in flight" }, "operator: in flight disables the buttons")
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 1, id: 7, state: .confirmed), at: opNow + 2)
    desk.reconcile(mirror, at: opNow + 2)
    expect(desk.latest(for: 7)?.phase == .done("confirmed by you"), "operator: the CONFIRMED transition finishes it")
    // The transition first, the response after: still done.
    var (other, entry2) = mirrorWithCandidate()
    var desk2 = LiveRoomOperator()
    let dismiss = build(.dismiss, entry2, other, &desk2)
    desk2.sent(dismiss, kind: .dismiss, mirror: other, delivered: true, at: opNow)
    _ = other.receive(topic: .events, payload: LRWire.event(g1, seq: 1, id: 7, state: .reverted), at: opNow + 1)
    desk2.reconcile(other, at: opNow + 1)
    desk2.response(actionId: dismiss.actionId, outcome: .applied, at: opNow + 2)
    expect(desk2.latest(for: 7)?.phase == .done("dismissed by you"), "operator: a transition before the response finishes it")
    // Another terminal state ends it honestly.
    var (third, entry3) = mirrorWithCandidate()
    var desk3 = LiveRoomOperator()
    let confirm = build(.confirm, entry3, third, &desk3)
    desk3.sent(confirm, kind: .confirm, mirror: third, delivered: true, at: opNow)
    _ = third.receive(topic: .events, payload: LRWire.event(g1, seq: 1, id: 7, state: .expired), at: opNow + 1)
    desk3.reconcile(third, at: opNow + 1)
    expect(desk3.latest(for: 7)?.phase == .done("the event ended: no longer detected"), "operator: an EXPIRED instead ends it")
}

func testOperatorUncertainOutcomes() {
    let (mirror, entry) = mirrorWithCandidate()
    // No answer: unknown after the timeout, then a retry with the same id.
    var desk = LiveRoomOperator()
    let action = build(.confirm, entry, mirror, &desk)
    desk.sent(action, kind: .confirm, mirror: mirror, delivered: true, at: opNow)
    desk.reconcile(mirror, at: opNow + LiveRoomOperator.responseTimeout - 1)
    expect(desk.latest(for: 7)?.phase.inFlight == true, "operator: waiting within the timeout")
    desk.reconcile(mirror, at: opNow + LiveRoomOperator.responseTimeout)
    expect(desk.latest(for: 7)?.phase == .unknown("no answer from the Spark"), "operator: no answer is an unknown outcome")
    expect(LiveRoomOperator.buttons(for: entry, earlier: false, mirror: mirror, operatorPath: true, pending: desk.latest(for: 7))
        .allSatisfy(\.enabled), "operator: an unknown outcome never leaves the buttons disabled")
    let again = desk.retry(actionId: action.actionId, mirror: mirror)
    expect(again?.action == action, "operator: a retry repeats the same id and payload")
    desk.sent(action, kind: .confirm, mirror: mirror, delivered: true, at: opNow + 20)
    expect(desk.latest(for: 7)?.attempts == 2 && desk.actions.count == 1, "operator: the retry is the same action")
    // The node forgot the id (its ring evicted it) and the event moved on: unknown, not refused.
    desk.response(actionId: action.actionId, outcome: .stateMismatch, at: opNow + 21)
    if case .unknown = desk.latest(for: 7)?.phase {} else { expect(false, "operator: a mismatch on a retry is an unknown outcome") }
    // A refusal on a first attempt is a refusal.
    var refused = LiveRoomOperator()
    let stale = build(.confirm, entry, mirror, &refused, id: 2)
    refused.sent(stale, kind: .confirm, mirror: mirror, delivered: true, at: opNow)
    refused.response(actionId: stale.actionId, outcome: .staleEpoch, at: opNow + 1)
    expect(refused.latest(for: 7)?.phase == .refused(.staleEpoch), "operator: refused")
    expect(LiveRoomOperator.describe(refused.latest(for: 7)!) == "confirm refused: the map changed since you looked; try again",
           "operator: a refusal reads plainly")
    // The bar speaks the event rows' words (main's plain copy): saved, checking, cells.
    var saved = refused.latest(for: 7)!
    saved.kind = .accept
    saved.phase = .applied(since: opNow)
    expect(LiveRoomOperator.describe(saved) == "saved · the map is updating", "operator: an applied accept is saved")
    expect(LiveRoomOperator.summary(entry) == "Appeared · checking · \(entry.event.voxelCount) cells"
           && LiveRoomOperator.stateWord(.expired) == "no longer detected",
           "operator: the selected line says checking and cells: \(LiveRoomOperator.summary(entry))")
    // A lost connection, a bridge failure, an unreadable answer.
    var lost = LiveRoomOperator()
    let call = build(.dismiss, entry, mirror, &lost, id: 3)
    lost.sent(call, kind: .dismiss, mirror: mirror, delivered: true, at: opNow)
    lost.connectionLost()
    expect(lost.latest(for: 7)?.phase == .unknown("the connection closed before the Spark answered"), "operator: a lost connection is unknown")
    lost.sent(call, kind: .dismiss, mirror: mirror, delivered: true, at: opNow + 1)
    lost.callFailed(actionId: call.actionId, reason: "Service does not exist")
    if case .unknown = lost.latest(for: 7)?.phase {} else { expect(false, "operator: a bridge failure is unknown") }
    lost.sent(call, kind: .dismiss, mirror: mirror, delivered: true, at: opNow + 2)
    lost.response(actionId: call.actionId, outcome: nil, at: opNow + 3)
    expect(lost.latest(for: 7)?.phase == .unknown("the Spark's answer was unreadable"), "operator: an unreadable answer is unknown")
    // Applied, but the transition never comes (lost on the way).
    var silent = LiveRoomOperator()
    let accept = build(.dismiss, entry, mirror, &silent, id: 4)
    silent.sent(accept, kind: .dismiss, mirror: mirror, delivered: true, at: opNow)
    silent.response(actionId: accept.actionId, outcome: .applied, at: opNow)
    silent.reconcile(mirror, at: opNow + LiveRoomOperator.transitionTimeout)
    if case .unknown = silent.latest(for: 7)?.phase {} else { expect(false, "operator: applied without a transition times out to unknown") }
    // Never sent.
    var unsent = LiveRoomOperator()
    let none = build(.confirm, entry, mirror, &unsent, id: 5)
    unsent.sent(none, kind: .confirm, mirror: mirror, delivered: false, at: opNow)
    if case .unsent = unsent.latest(for: 7)?.phase {} else { expect(false, "operator: an action without a service is unsent") }
}

func testOperatorAfterARestart() {
    var (mirror, entry) = mirrorWithCandidate()
    var desk = LiveRoomOperator()
    let action = build(.confirm, entry, mirror, &desk)
    desk.sent(action, kind: .confirm, mirror: mirror, delivered: true, at: opNow)
    let g2 = g1 + 90_000
    _ = mirror.receive(topic: .status, payload: LRWire.status(g2, seq: 0, epoch: 6, eventsSeq: nil), at: opNow + 1)
    desk.reconcile(mirror, at: opNow + 1)
    expect(desk.latest(for: 7)?.phase == .unknown("the Spark restarted; check the event"), "operator: a restart makes it unknown")
    expect(desk.retry(actionId: action.actionId, mirror: mirror) == nil, "operator: no retry into another generation")
}

func testOperatorButtons() {
    let (mirror, candidate) = mirrorWithCandidate()
    let kinds = { (entry: LiveRoomEvent, m: Mirror) in
        LiveRoomOperator.buttons(for: entry, earlier: false, mirror: m, operatorPath: true, pending: nil).map(\.kind)
    }
    expect(kinds(candidate, mirror) == [.confirm, .dismiss], "buttons: a candidate offers Confirm and Dismiss")
    let (confirmedMirror, confirmed) = mirrorWithCandidate(state: .confirmed)
    expect(kinds(confirmed, confirmedMirror) == [.accept, .dismiss], "buttons: a confirmed event offers Accept and Dismiss")
    let (endedMirror, ended) = mirrorWithCandidate(state: .expired)
    expect(kinds(ended, endedMirror).isEmpty, "buttons: none for a terminal event")
    expect(LiveRoomOperator.buttons(for: candidate, earlier: true, mirror: mirror, operatorPath: true, pending: nil).isEmpty,
           "buttons: none for an earlier run")
    let reasons = { (m: Mirror, path: Bool) in
        LiveRoomOperator.buttons(for: confirmed, earlier: false, mirror: m, operatorPath: path, pending: nil).map(\.reason)
    }
    expect(reasons(confirmedMirror, false) == ["this Spark can't take actions", "this Spark can't take actions"], "buttons: no service")
    var stale = confirmedMirror
    _ = stale.tick(at: opNow + 10)
    expect(reasons(stale, true) == ["status isn't fresh", "status isn't fresh"], "buttons: stale status")
    var gap = confirmedMirror
    _ = gap.receive(topic: .mapBricks, payload: LRWire.delta(g1, epoch: 6, seq: 12, base: 5, []), at: opNow)
    expect(reasons(gap, true) == ["map isn't current", nil], "buttons: Accept needs a current map, Dismiss does not")
    var legacy = Mirror(tokenSeed: 3)
    _ = legacy.connectionOpened(at: opNow, canRequest: true)
    _ = legacy.receive(topic: .status, payload: LRWire.status(nil, seq: 1, epoch: 5, eventsSeq: nil), at: opNow)
    expect(reasons(legacy, true).allSatisfy { $0 == "the Spark's identity is unverified" }, "buttons: a legacy wire")
    expect(reasons(confirmedMirror, true) == [nil, nil], "buttons: enabled")
}

func testOperatorSelectionResolvesRowIds() {
    var (mirror, _) = mirrorWithCandidate()
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 1, id: 8, state: .candidate), at: opNow)
    _ = mirror.receive(topic: .events, payload: LRWire.event(g1, seq: 2, id: 8, state: .expired), at: opNow)
    let screen = LiveRoomScreen.make(mirror: mirror, connection: .off, requestPathExists: true, started: true, now: opNow)
    for row in screen.openRows + screen.historyRows {
        let found = LiveRoomOperator.entry(for: row.id, mirror: mirror)
        expect(found != nil && found?.earlier == false, "selection: row \(row.id) resolves")
    }
    expect(LiveRoomOperator.entry(for: "7", mirror: mirror)?.entry.event.state == .candidate, "selection: an open id")
    expect(LiveRoomOperator.entry(for: "2-8", mirror: mirror)?.entry.event.state == .expired, "selection: a terminal id")
    expect(LiveRoomOperator.entry(for: "9", mirror: mirror) == nil && LiveRoomOperator.entry(for: nil, mirror: mirror) == nil,
           "selection: unknown ids resolve to nothing")
}

// MARK: The client over a fake bridge

@MainActor
private func spinUntil(_ seconds: TimeInterval, _ done: () -> Bool) {
    let deadline = Date().addingTimeInterval(seconds)
    while Date() < deadline, !done() { RunLoop.main.run(until: Date().addingTimeInterval(0.02)) }
}

@MainActor
private func operatorBridge(outcome: R.OperatorOutcome?, failure: String? = nil) -> (LRFakeBridge, LiveRoomClient) {
    var publisher = LRPublisher(seed: 4, generation: g1, legacy: false)
    _ = publisher.map()
    let bridge = LRFakeBridge(capabilities: ["connectionGraph", "services"], serviceType: LiveRoomClient.requestServiceType,
                              publisher: publisher)
    bridge.operatorOutcome = outcome
    bridge.operatorFailure = failure
    bridge.start()
    let client = LiveRoomClient(url: bridge.url)
    client.start()
    spinUntil(5) { client.mirror.activeGeneration != nil && client.requestPath != .unavailable && (outcome == nil && failure == nil || client.operatorPath) }
    return (bridge, client)
}

private func fakeEntry(_ state: R.ChangeEvent.State) -> LiveRoomEvent {
    let header = R.Header(roomId: "lab", mapEpoch: 1, seq: 0, stamp: t10)
    let event = R.ChangeEvent(header: header, eventId: 7, kind: .appeared, state: state, centroid: SIMD3(1, 1, 1),
                              aabb: .init(min: SIMD3(0.9, 0.9, 0.9), max: SIMD3(1.1, 1.1, 1.1)), fromCentroid: nil,
                              voxelCount: 8, volumeM3: 0.001, evidence: nil, alpha: nil, sensors: .jt128,
                              firstSeen: header.stamp, lastSeen: header.stamp, mask: nil, publisherGeneration: g1)
    return LiveRoomEvent(seq: 0, receivedAt: 0, fromSync: false, event: event)
}

@MainActor
func testOperatorServiceOverTheBridge() {
    let (bridge, client) = operatorBridge(outcome: .applied)
    defer { client.stop(); bridge.stop() }
    expect(client.operatorPath, "bridge: the operator service is the path")
    if case .connected(let server) = client.connection {
        expect(server.operatorServiceId == LRFakeBridge.operatorServiceId && server.requestServiceId == LRFakeBridge.serviceId,
               "bridge: both services advertised")
    }
    expect(client.act(.confirm, on: fakeEntry(.candidate)), "bridge: the action goes out")
    spinUntil(5) { client.operatorDesk.latest(for: 7)?.phase.inFlight == true && bridge.operatorCalls.count == 1
        && { if case .applied = client.operatorDesk.latest(for: 7)?.phase { return true } else { return false } }() }
    expect(bridge.operatorCalls.count == 1, "bridge: one call to the operator service")
    let sent = client.operatorDesk.latest(for: 7)?.action
    expect(bridge.operatorCalls.first == sent && sent?.expectedGeneration == g1, "bridge: the call carries the action's CDR")
    if case .applied = client.operatorDesk.latest(for: 7)?.phase {} else { expect(false, "bridge: APPLIED routes back to the action") }
    expect(client.mirror.activity.requestsUnsent == 0, "bridge: RequestSync bookkeeping untouched")
    bridge.withdrawOperatorService()
    spinUntil(5) { !client.operatorPath }
    expect(!client.operatorPath && client.requestPath == .bridgeService(id: LRFakeBridge.serviceId),
           "bridge: the operator service withdrawn, the request service stays")
    expect(!client.act(.dismiss, on: fakeEntry(.candidate)), "bridge: no service, nothing sent")
    if case .unsent = client.operatorDesk.latest(for: 7)?.phase {} else { expect(false, "bridge: the action is unsent") }
}

@MainActor
func testOperatorServiceFailureAndAbsence() {
    let (bridge, client) = operatorBridge(outcome: nil, failure: "Service does not exist")
    expect(client.act(.confirm, on: fakeEntry(.candidate)), "failure: the action goes out")
    spinUntil(5) { if case .unknown = client.operatorDesk.latest(for: 7)?.phase { return true } else { return false } }
    if case .unknown = client.operatorDesk.latest(for: 7)?.phase {} else { expect(false, "failure: a serviceCallFailure is an unknown outcome") }
    expect(client.mirror.activity.requestsUnsent == 0, "failure: not counted as an unsent RequestSync")
    client.stop()
    bridge.stop()
    let (plain, readOnly) = operatorBridge(outcome: nil)
    defer { readOnly.stop(); plain.stop() }
    expect(!readOnly.operatorPath, "absent: a Spark without the service takes no actions")
}

func runOperatorActionTests() {
    testOperatorActionGoldens()
    print("  operator: 3 OperatorAction hand goldens decode (LE and BE), the encoder reproduces the LE ones; 129 B"); passed()
    testOperatorActionRefusals()
    print("  operator: the encoder and decoder refuse every D16 rule; room-level actions carry EVENT_NONE"); passed()
    testOperatorOutcomeResponses()
    print("  operator: every service outcome parses; short or unknown responses are unreadable"); passed()
    testOperatorEmptyDeltaApplies()
    print("  operator: an empty delta (non-map-sensor accept) applies at epoch + 1 and stays current"); passed()
    testOperatorMirrorReductions()
    print("  operator: confirm, dismiss and accept reach the mirror as transitions it already renders"); passed()
    testOperatorActionsAreBuiltFromWhatTheOperatorSees()
    print("  operator: an action carries the chain epoch, the active generation and the shown state"); passed()
    testOperatorActionFinishesOnItsTransition()
    print("  operator: an action finishes on its transition, before or after the response"); passed()
    testOperatorUncertainOutcomes()
    print("  operator: timeouts, lost connections, bridge failures and evicted retries are unknown, never stuck"); passed()
    testOperatorAfterARestart()
    print("  operator: a restart makes a pending action unknown and refuses a retry"); passed()
    testOperatorButtons()
    print("  operator: the buttons per state and every disabled reason"); passed()
    testOperatorSelectionResolvesRowIds()
    print("  operator: a selected row resolves to its event"); passed()
    MainActor.assumeIsolated {
        testOperatorServiceOverTheBridge()
        print("  operator: the client calls the bridge's operator service and routes the outcome back"); passed()
        testOperatorServiceFailureAndAbsence()
        print("  operator: a serviceCallFailure is unknown; without the service the Mac takes no actions"); passed()
    }
}
