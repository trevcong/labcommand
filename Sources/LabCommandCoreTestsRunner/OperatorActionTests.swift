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

func runOperatorActionTests() {
    testOperatorActionGoldens()
    print("  operator: 3 OperatorAction hand goldens decode (LE and BE), the encoder reproduces the LE ones; 129 B"); passed()
    testOperatorActionRefusals()
    print("  operator: the encoder and decoder refuse every D16 rule; room-level actions carry EVENT_NONE"); passed()
    testOperatorOutcomeResponses()
    print("  operator: every service outcome parses; short or unknown responses are unreadable"); passed()
    testOperatorEmptyDeltaApplies()
    print("  operator: an empty delta (non-map-sensor accept) applies at epoch + 1 and stays current"); passed()
}
