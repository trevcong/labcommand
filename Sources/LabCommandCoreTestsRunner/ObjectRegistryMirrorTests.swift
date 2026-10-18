import Foundation
import LabCommandCore

// MARK: - ObjectRegistryMirror: hold until settled, receipt precedence (protocol §3.15)

private typealias Registry = RoomMemory.ObjectRegistry

private let mirrorGeneration: UInt64 = 1_760_400_000_000
private let mirrorNextGeneration: UInt64 = 1_760_400_090_000

private func mirrorHeader(room: String) -> RoomMemory.Header {
    RoomMemory.Header(roomId: room, mapEpoch: 43, seq: 4, stamp: RoomMemory.Time(sec: 1_760_400_010, nanosec: 250_000_000))
}

private func receipt(_ generation: UInt64, _ event: UInt64, kind: RoomMemory.ChangeEvent.Kind = .moved) -> RoomMemory.RecoveredAccept {
    RoomMemory.RecoveredAccept(eventGeneration: generation, eventId: event, kind: kind)
}

private func registry(
    room: String = "lab",
    generation: UInt64,
    seq: UInt64,
    receipts: [RoomMemory.RecoveredAccept] = [],
    actions: [String] = [],
    objects: [RoomMemory.RegistryObject] = []
) -> Registry {
    Registry(
        header: mirrorHeader(room: room),
        snapshotSeq: seq,
        objects: objects,
        recentActionIds: actions,
        recoveredAccepts: receipts,
        publisherGeneration: generation
    )
}

private func namedObject(generation: UInt64, event: UInt64, name: String) -> RoomMemory.RegistryObject {
    RoomMemory.RegistryObject(
        objectId: 7,
        revision: 2,
        name: name,
        baseline: .anchored,
        association: .none,
        associationGeneration: 0,
        associationEventId: 0,
        sensor: .mid360,
        frameEpoch: 40,
        centroid: SIMD3<Float>(1.5, -0.25, 0.125),
        aabb: RoomMemory.ChangeEvent.Box(
            min: SIMD3<Float>(1.25, -0.5, 0),
            max: SIMD3<Float>(1.75, 0, 0.25)
        ),
        links: [RoomMemory.ObjectLink(eventGeneration: generation, eventId: event, kind: .moved, recovered: true)]
    )
}

private func testReceiptBeatsEveryLoggedStateExceptAccepted() {
    var mirror = ObjectRegistryMirror(roomId: "lab")
    let current = receipt(mirrorGeneration, 8)
    let archived = receipt(mirrorGeneration - 1_000, 9)
    expect(!mirror.ingest(registry(generation: mirrorGeneration, seq: 3, receipts: [current, archived])), "held until settle")
    expect(mirror.snapshot == nil, "an unsettled snapshot stays off screen")
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 8) == .none, "a held receipt is not shown")

    mirror.generationSettled(mirrorGeneration)
    expect(mirror.snapshot?.publisherGeneration == mirrorGeneration, "settling shows the snapshot")
    expect(mirror.snapshot?.snapshotSeq == 3, "the held seq is the one that settles")
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 8).label == "accepted (recovered)",
           "a current-generation receipt with no ACCEPTED shows accepted (recovered)")
    expect(mirror.objectName(eventGeneration: mirrorGeneration, eventId: 8) == nil, "the phrase does not need a name")
    expect(mirror.accept(eventGeneration: mirrorGeneration - 1_000, eventId: 9).label == "accepted (recovered)",
           "a receipt naming an older generation is kept")
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 10) == .none, "a different event id does not match")

    mirror.note(eventGeneration: mirrorGeneration, eventId: 8, state: .accepted)
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 8).label == "ACCEPTED",
           "ACCEPTED plus a receipt shows ACCEPTED")

    mirror.note(eventGeneration: mirrorGeneration, eventId: 8, state: .expired)
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 8).label == "accepted (recovered)",
           "EXPIRED plus a receipt shows accepted (recovered)")

    mirror.note(eventGeneration: mirrorGeneration, eventId: 11, state: .confirmed)
    mirror.note(eventGeneration: mirrorGeneration, eventId: 12, state: .reverted)
    expect(mirror.ingest(registry(
        generation: mirrorGeneration,
        seq: 4,
        receipts: [current, receipt(mirrorGeneration, 11), receipt(mirrorGeneration, 12)]
    )), "a newer seq of a settled generation replaces the snapshot")
    expect(mirror.snapshot?.snapshotSeq == 4, "snapshot_seq 4 is on screen")
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 11).label == "accepted (recovered)",
           "CONFIRMED plus a receipt shows accepted (recovered)")
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 12).label == "accepted (recovered)",
           "REVERTED plus a receipt shows accepted (recovered)")
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 13) == .none, "an unnamed event with no receipt stays empty")

    let named = registry(
        generation: mirrorGeneration,
        seq: 5,
        receipts: [receipt(mirrorGeneration, 14, kind: .appeared)],
        objects: [namedObject(generation: mirrorGeneration, event: 14, name: "Scope cart")]
    )
    expect(mirror.ingest(named), "the named snapshot replaces seq 4")
    expect(mirror.objectName(eventGeneration: mirrorGeneration, eventId: 14) == "Scope cart", "the link carries the name")
    expect(mirror.accept(eventGeneration: mirrorGeneration, eventId: 14).label == "accepted (recovered)",
           "a named receipt still reads accepted (recovered)")
}

private func testHoldUntilSettledAndDropStaleSnapshots() {
    var mirror = ObjectRegistryMirror(roomId: "lab")
    expect(mirror.ingest(registry(generation: mirrorGeneration, seq: 1, receipts: [receipt(mirrorGeneration, 1)])) == false,
           "an unsettled snapshot stays off screen")
    mirror.generationSettled(mirrorGeneration)
    expect(mirror.snapshot?.snapshotSeq == 1, "generation 1 is on screen")

    expect(!mirror.ingest(registry(generation: mirrorGeneration, seq: 1)), "the same snapshot_seq is dropped")
    expect(!mirror.ingest(registry(generation: mirrorGeneration, seq: 0)), "an older snapshot_seq is dropped")
    expect(mirror.snapshot?.snapshotSeq == 1, "seq 1 stays")

    let pendingReceipt = receipt(mirrorNextGeneration, 2)
    expect(!mirror.ingest(registry(generation: mirrorNextGeneration, seq: 0, receipts: [pendingReceipt])),
           "a newer unsettled generation is held")
    expect(mirror.snapshot?.publisherGeneration == mirrorGeneration, "the settled snapshot stays on screen")
    expect(mirror.accept(eventGeneration: mirrorNextGeneration, eventId: 2) == .none, "the pending receipt is not current")

    expect(!mirror.ingest(registry(generation: mirrorNextGeneration, seq: 0, receipts: [receipt(mirrorNextGeneration, 99)])),
           "an older seq of the pending generation is dropped")
    expect(mirror.ingest(registry(generation: mirrorNextGeneration, seq: 2, receipts: [pendingReceipt])) == false,
           "a newer seq replaces the pending snapshot and stays off screen")

    expect(!mirror.ingest(registry(room: "other", generation: mirrorNextGeneration, seq: 9)), "another room is dropped")
    expect(mirror.snapshot?.header.roomId == "lab", "the room stays lab")

    mirror.generationSettled(mirrorNextGeneration)
    expect(mirror.snapshot?.publisherGeneration == mirrorNextGeneration, "settling the newer generation installs it")
    expect(mirror.snapshot?.snapshotSeq == 2, "the newer pending seq is the one installed")
    expect(mirror.accept(eventGeneration: mirrorNextGeneration, eventId: 2).label == "accepted (recovered)",
           "the receipt is visible once that generation settles")

    expect(!mirror.ingest(registry(generation: mirrorGeneration, seq: 50)), "an older publisher generation is dropped")
    expect(mirror.snapshot?.publisherGeneration == mirrorNextGeneration, "the newer generation stays")

    var adopting = ObjectRegistryMirror()
    expect(adopting.roomId == nil, "the room is unset until a snapshot")
    adopting.generationSettled(mirrorGeneration)
    expect(adopting.ingest(registry(room: "lab", generation: mirrorGeneration, seq: 1)), "the first room is adopted")
    expect(adopting.roomId == "lab", "the room is lab")
    expect(!adopting.ingest(registry(room: "other", generation: mirrorGeneration, seq: 2)), "a later room is dropped")
    expect(adopting.snapshot?.header.roomId == "lab", "the adopted room stays")
}

private func testNameAttemptsCompleteOrReconcile() {
    var mirror = ObjectRegistryMirror(roomId: "lab")
    mirror.generationSettled(mirrorGeneration)
    expect(mirror.ingest(registry(generation: mirrorGeneration, seq: 1, actions: ["act-1"])), "the snapshot lists act-1")
    expect(mirror.attempt(actionId: "act-1") == .completed, "an id in recent_action_ids completes")
    expect(mirror.attempt(actionId: "missing") == .pending, "an unknown id stays pending")

    mirror.noteResponse(actionId: "act-2", outcome: .applied)
    expect(mirror.attempt(actionId: "act-2") == .completed, "APPLIED completes")

    mirror.noteResponse(actionId: "act-3", outcome: .staleRevision)
    expect(mirror.attempt(actionId: "act-3") == .reconcile(.staleRevision), "STALE_REVISION reconciles")
    expect(mirror.ingest(registry(generation: mirrorGeneration, seq: 2, actions: ["act-1", "act-3"])), "act-3 is now listed")
    expect(mirror.attempt(actionId: "act-3") == .reconcile(.staleRevision),
           "STALE_REVISION stays reconcile after the id appears in recent_action_ids")

    mirror.noteResponse(actionId: "act-4", outcome: .unknownEvent)
    expect(mirror.ingest(registry(generation: mirrorGeneration, seq: 3, actions: ["act-4"])), "act-4 is listed")
    expect(mirror.attempt(actionId: "act-4") == .reconcile(.unknownEvent),
           "UNKNOWN_EVENT stays reconcile after the id appears in recent_action_ids")

    mirror.noteResponse(actionId: "act-5", outcome: .storageFailed)
    expect(mirror.attempt(actionId: "act-5") == .rejected(.storageFailed), "STORAGE_FAILED does not complete")

    MainActor.assumeIsolated {
        expect(LiveRoomClient.nameObjectService == "/room_memory/objects/name", "NameObject service path")
        expect(LiveRoomClient.nameObjectServiceType == "labcommand_msgs/srv/NameObject", "NameObject service type")
    }
}

func runObjectRegistryMirrorTests() {
    testReceiptBeatsEveryLoggedStateExceptAccepted()
    print("  ObjectRegistryMirror: a receipt reads accepted (recovered); ACCEPTED wins; EXPIRED, CONFIRMED, and REVERTED lose"); passed()
    testHoldUntilSettledAndDropStaleSnapshots()
    print("  ObjectRegistryMirror: hold until settled; drop another room, an older generation, and an older snapshot_seq"); passed()
    testNameAttemptsCompleteOrReconcile()
    print("  ObjectRegistryMirror: APPLIED and recent_action_ids complete; STALE_REVISION and UNKNOWN_EVENT reconcile"); passed()
}
