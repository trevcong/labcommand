import Foundation
import LabCommandCore

// MARK: - Hand-Derived Goldens (PR 0.2)

// Tests/Fixtures/room/golden/hand/*.hex were written byte by byte from the .msg files and the
// CDR rules, not produced by the encoder, so encoding to them is not circular. Spark goldens
// from rclpy (scripts/spark/make_room_memory_goldens.py, same values) are a follow-up.

private typealias Room = RoomMemory

private func goldenBytes(_ name: String) -> Data {
    let path = "Tests/Fixtures/room/golden/hand/\(name).hex"
    guard let text = try? String(contentsOfFile: path, encoding: .utf8) else {
        fputs("FAIL: \(path) not found — run tests from repo root\n", stderr)
        exit(1)
    }
    var bytes: [UInt8] = []
    for line in text.split(separator: "\n") {
        for token in line.prefix(while: { $0 != "#" }).split(separator: " ") {
            guard let byte = UInt8(token, radix: 16) else {
                fputs("FAIL: \(path): bad hex byte '\(token)'\n", stderr)
                exit(1)
            }
            bytes.append(byte)
        }
    }
    return Data(bytes)
}

private let t10 = Room.Time(sec: 1_760_400_010, nanosec: 250_000_000)

private func mask(_ firstByte: UInt8, last: UInt8 = 0) -> [UInt8] {
    var bytes = [UInt8](repeating: 0, count: Room.Brick.maskBytes)
    bytes[0] = firstByte
    bytes[Room.Brick.maskBytes - 1] = last
    return bytes
}

private let goldenStatus = Room.MemoryStatus(
    header: Room.Header(roomId: "lab-a", mapEpoch: 42, seq: 7, stamp: Room.Time(sec: 1_760_400_000, nanosec: 500_000_000)),
    state: .tracking,
    brickCount: 3200,
    openEventCount: 2,
    coverageKnownFraction: 0.75,
    poseResidualM: nil,
    windowS: 1.0,
    mid360Hz: 10,
    jt128Hz: nil,
    droppedWindows: 3
)

private let goldenEpoch = Room.Epoch(
    header: Room.Header(roomId: "lab", mapEpoch: 4_294_967_298, seq: 1000, stamp: Room.Time(sec: 1_760_400_005, nanosec: 0)),
    appearedCount: 5,
    disappearedCount: 1,
    movedCount: 16_909_060,
    coverageKnownFraction: nil,
    state: .rigMoved
)

private let goldenDelta = Room.MapBricks(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 12, stamp: t10),
    kind: .delta,
    baseEpoch: 42,
    voxelSize: 0.05,
    bricks: [Room.Brick(coordinate: Room.BrickCoordinate(i: -1, j: 2, k: -3), occupied: mask(0x01, last: 0x80), known: mask(0x03, last: 0x80))],
    removedBrickIndices: [0, 5]
)

private let goldenMoved = Room.ChangeEvent(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 3, stamp: t10),
    eventId: 9,
    kind: .moved,
    state: .confirmed,
    centroid: SIMD3(1.5, -0.25, 0.125),
    aabb: Room.ChangeEvent.Box(min: SIMD3(1.25, -0.5, 0), max: SIMD3(1.75, 0, 0.25)),
    fromCentroid: SIMD3(0.5, -0.25, 0.125),
    voxelCount: 125,
    volumeM3: 0.015625,
    evidence: 0.875,
    alpha: nil,
    sensors: [.mid360, .jt128],
    firstSeen: Room.Time(sec: 1_760_400_000, nanosec: 0),
    lastSeen: t10,
    mask: Room.MapBricks(
        header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 0, stamp: t10),
        kind: .snapshot,
        baseEpoch: 0,
        voxelSize: 0.05,
        bricks: [Room.Brick(coordinate: Room.BrickCoordinate(i: 3, j: -1, k: 0), occupied: mask(0x01), known: mask(0x01))]
    )
)

private let goldenAppeared = Room.ChangeEvent(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 4, stamp: t10),
    eventId: 10,
    kind: .appeared,
    state: .candidate,
    centroid: SIMD3(2, 1, 0.5),
    aabb: Room.ChangeEvent.Box(min: SIMD3(1.875, 0.875, 0.375), max: SIMD3(2.125, 1.125, 0.625)),
    fromCentroid: nil,
    voxelCount: 8,
    volumeM3: 0.001,
    evidence: nil,
    alpha: 0.05,
    sensors: .mid360,
    firstSeen: Room.Time(sec: 1_760_400_010, nanosec: 0),
    lastSeen: t10,
    mask: nil
)

/// One golden: its file, the decoder for its type, and the expected encoding of the expected value.
private struct Golden {
    var name: String
    var decodeMatches: (Data) throws -> Bool
    var encodeExpected: () throws -> Data
    /// Re-encodes whatever decodes, so the fuzz can check decode ⇒ valid.
    var reencode: (Data) throws -> Data
}

private func golden<M: Equatable>(
    _ name: String,
    _ expected: M,
    decode: @escaping (Data) throws -> M,
    encode: @escaping (M) throws -> Data
) -> Golden {
    Golden(
        name: name,
        decodeMatches: { try decode($0) == expected },
        encodeExpected: { try encode(expected) },
        reencode: { try encode(try decode($0)) }
    )
}

private let littleEndianGoldens: [Golden] = [
    golden("memory_status_le", goldenStatus, decode: Room.MemoryStatus.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("epoch_le", goldenEpoch, decode: Room.Epoch.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("map_bricks_delta_le", goldenDelta, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("change_event_moved_mask_le", goldenMoved, decode: Room.ChangeEvent.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("change_event_appeared_le", goldenAppeared, decode: Room.ChangeEvent.decode(cdr:), encode: { try $0.encodedCDR() }),
]

private let bigEndianGoldens: [Golden] = [
    golden("epoch_be", goldenEpoch, decode: Room.Epoch.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("map_bricks_delta_be", goldenDelta, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() }),
]

private enum Outcome: Equatable {
    case decoded
    case truncated
    case invalid
    case unsupported
    case otherError
}

private func outcome(_ body: () throws -> Bool) -> (Outcome, String) {
    do {
        return (try body() ? .decoded : .otherError, "decoded to a different value")
    } catch let error as RoomMemoryCDRError {
        switch error {
        case .truncated(let text): return (.truncated, text)
        case .invalid(let text): return (.invalid, text)
        case .unsupported(let text): return (.unsupported, text)
        }
    } catch {
        return (.otherError, "\(error)")
    }
}

func testRoomGoldensDecodeAndEncode() {
    for golden in littleEndianGoldens {
        let bytes = goldenBytes(golden.name)
        let (decoded, text) = outcome { try golden.decodeMatches(bytes) }
        expect(decoded == .decoded, "golden \(golden.name): \(text)")
        do {
            let encoded = try golden.encodeExpected()
            expect(encoded == bytes, "golden \(golden.name): encoder wrote \(encoded.count) bytes that differ from the hand bytes (\(bytes.count))")
        } catch {
            expect(false, "golden \(golden.name): encode threw \(error)")
        }
    }
    for golden in bigEndianGoldens {
        let (decoded, text) = outcome { try golden.decodeMatches(goldenBytes(golden.name)) }
        expect(decoded == .decoded, "golden \(golden.name): \(text)")
    }
}

func testRoomGoldenSemantics() {
    // Missing stays missing: NaN and 0xFFFF decode to nil, never to zero.
    expect(goldenStatus.poseResidualM == nil && goldenStatus.jt128Hz == nil && goldenStatus.mid360Hz == 10,
           "golden status: NaN residual and HZ_UNAVAILABLE are nil next to a measured 10 Hz")
    expect(goldenMoved.alpha == nil && goldenAppeared.alpha == 0.05, "golden events: NaN alpha is nil, 0.05 is kept")

    let brick = goldenDelta.bricks[0]
    expect(brick.evidence(x: 0, y: 0, z: 0) == .occupied, "brick bit 0 is voxel (0,0,0), occupied")
    expect(brick.evidence(x: 1, y: 0, z: 0) == .free, "known and not occupied reads free")
    expect(brick.evidence(x: 0, y: 1, z: 0) == .unknown, "no known bit reads unknown, not free")
    expect(brick.evidence(x: 7, y: 7, z: 7) == .occupied, "voxel (7,7,7) is byte 63 bit 7")
    let last = Room.Brick.bitPosition(x: 7, y: 7, z: 7)
    expect(last.byte == 63 && last.mask == 0x80, "bitPosition(7,7,7) = byte 63, mask 0x80")
    let yStep = Room.Brick.bitPosition(x: 0, y: 1, z: 0)
    expect(yStep.byte == 1 && yStep.mask == 0x01, "bitPosition(0,1,0) = bit 8")
}

func testRoomSliceAndPadding() {
    let bytes = goldenBytes("change_event_moved_mask_le")
    let padded = Data([0xAA, 0xBB, 0xCC]) + bytes + Data([0, 0, 0])
    let slice = padded[3...]
    expect(slice.startIndex == 3, "slice keeps a non-zero startIndex")
    let (sliced, text) = outcome { try Room.ChangeEvent.decode(cdr: slice) == goldenMoved }
    expect(sliced == .decoded, "a slice with 3 bytes of trailing padding decodes: \(text)")

    let (extra, _) = outcome { try Room.ChangeEvent.decode(cdr: bytes + Data([0, 0, 0, 0])) == goldenMoved }
    expect(extra == .invalid, "4 trailing bytes are not padding")
}

func testRoomTruncationAlwaysThrows() {
    for golden in littleEndianGoldens + bigEndianGoldens {
        let bytes = goldenBytes(golden.name)
        for length in 0..<bytes.count {
            let (result, text) = outcome { try golden.decodeMatches(bytes.prefix(length)) }
            expect(result == .truncated, "\(golden.name) cut to \(length) bytes: \(result) \(text)")
            expect(text.contains("CDR: truncated"), "\(golden.name) cut to \(length): message '\(text)'")
        }
    }
}

// MARK: - Malformed Input

/// `data` with bytes replaced at stream position `pos` (counted after the 4-byte header).
private func patched(_ data: Data, at pos: Int, _ replacement: [UInt8]) -> Data {
    var bytes = [UInt8](data)
    bytes.replaceSubrange((pos + 4)..<(pos + 4 + replacement.count), with: replacement)
    return Data(bytes)
}

private func patched(_ name: String, at pos: Int, _ replacement: [UInt8]) -> Data {
    patched(goldenBytes(name), at: pos, replacement)
}

private func le32(_ value: UInt32) -> [UInt8] { withUnsafeBytes(of: value.littleEndian, Array.init) }
private func f32(_ value: Float) -> [UInt8] { le32(value.bitPattern) }

func testRoomMalformedInputRejected() {
    let status = { (data: Data) in try Room.MemoryStatus.decode(cdr: data) == goldenStatus }
    let epoch = { (data: Data) in try Room.Epoch.decode(cdr: data) == goldenEpoch }
    let bricks = { (data: Data) in try Room.MapBricks.decode(cdr: data) == goldenDelta }
    let event = { (data: Data) in try Room.ChangeEvent.decode(cdr: data) == goldenMoved }
    let appeared = { (data: Data) in try Room.ChangeEvent.decode(cdr: data) == goldenAppeared }

    let delta = "map_bricks_delta_le"
    let moved = "change_event_moved_mask_le"
    let fresh = "change_event_appeared_le"
    var xcdr2 = [UInt8](goldenBytes("epoch_le"))
    xcdr2[1] = 0x07

    let cases: [(String, Outcome, () throws -> Bool)] = [
        ("XCDR2 encapsulation", .unsupported, { try epoch(Data(xcdr2)) }),
        ("empty buffer", .truncated, { try epoch(Data()) }),
        ("room_id without NUL", .invalid, { try epoch(patched("epoch_le", at: 7, [0x78])) }),
        ("room_id length 0", .invalid, { try epoch(patched("epoch_le", at: 0, le32(0))) }),
        ("room_id not UTF-8", .invalid, { try epoch(patched("epoch_le", at: 4, [0xFF])) }),
        ("room_id length past the end", .truncated, { try epoch(patched("epoch_le", at: 0, le32(0xFFFF_FFFF))) }),
        ("stamp nanosec 1e9", .invalid, { try epoch(patched("epoch_le", at: 28, le32(1_000_000_000))) }),
        ("Epoch state 5", .invalid, { try epoch(patched("epoch_le", at: 48, [5])) }),
        ("Epoch coverage -0.5", .invalid, { try epoch(patched("epoch_le", at: 44, f32(-0.5))) }),
        ("MemoryStatus state 9", .invalid, { try status(patched("memory_status_le", at: 40, [9])) }),
        ("MemoryStatus coverage 1.5", .invalid, { try status(patched("memory_status_le", at: 52, f32(1.5))) }),
        ("MemoryStatus residual -1 m", .invalid, { try status(patched("memory_status_le", at: 56, f32(-1))) }),
        ("MemoryStatus window 0 s", .invalid, { try status(patched("memory_status_le", at: 60, f32(0))) }),
        ("MemoryStatus window +Inf", .invalid, { try status(patched("memory_status_le", at: 60, f32(.infinity))) }),
        ("MapBricks kind 2", .invalid, { try bricks(patched(delta, at: 32, [2])) }),
        ("snapshot with removed bricks", .invalid, { try bricks(patched(delta, at: 32, [0])) }),
        ("voxel_size NaN", .invalid, { try bricks(patched(delta, at: 48, f32(.nan))) }),
        ("voxel_size 0", .invalid, { try bricks(patched(delta, at: 48, f32(0))) }),
        ("brick_dim 4", .unsupported, { try bricks(patched(delta, at: 52, [4])) }),
        ("brick_ijk not 3 per brick", .invalid, { try bricks(patched(delta, at: 56, le32(2))) }),
        ("brick_ijk count 2^32-1", .truncated, { try bricks(patched(delta, at: 56, le32(0xFFFF_FFFF))) }),
        ("occ_bits 63 bytes", .invalid, { try bricks(patched(delta, at: 68, le32(63))) }),
        ("occupied voxel not known", .invalid, { try bricks(patched(delta, at: 72, [0x05])) }),
        ("removed_bricks_idx not increasing", .invalid, { try bricks(patched(delta, at: 212, le32(0))) }),
        ("removed_bricks_idx count 2^32-1", .truncated, { try bricks(patched(delta, at: 204, le32(0xFFFF_FFFF))) }),
        ("compression zstd", .unsupported, { try bricks(patched(delta, at: 216, [1])) }),
        ("compression 2", .invalid, { try bricks(patched(delta, at: 216, [2])) }),
        ("ChangeEvent kind 3", .invalid, { try event(patched(moved, at: 40, [3])) }),
        ("ChangeEvent state 5", .invalid, { try event(patched(moved, at: 41, [5])) }),
        ("APPEARED with from_centroid", .invalid, { try event(patched(moved, at: 40, [0])) }),
        ("MOVED without from_centroid", .invalid, { try appeared(patched(fresh, at: 40, [2])) }),
        ("from_centroid partly NaN", .invalid, { try event(patched(moved, at: 80, f32(.nan))) }),
        ("centroid NaN", .invalid, { try event(patched(moved, at: 44, f32(.nan))) }),
        ("aabb min above max", .invalid, { try event(patched(moved, at: 56, f32(2))) }),
        ("volume -1", .invalid, { try event(patched(moved, at: 96, f32(-1))) }),
        ("evidence 1.5", .invalid, { try event(patched(moved, at: 100, f32(1.5))) }),
        ("alpha 1.5", .invalid, { try appeared(patched(fresh, at: 104, f32(1.5))) }),
        ("alpha +Inf", .invalid, { try appeared(patched(fresh, at: 104, f32(.infinity))) }),
        ("sensors bit 2", .invalid, { try appeared(patched(fresh, at: 108, [0x04])) }),
        ("last_seen before first_seen", .invalid, { try appeared(patched(fresh, at: 120, le32(1_760_400_009))) }),
        ("mask count 2", .invalid, { try event(patched(moved, at: 128, le32(2))) }),
        ("mask count 1 with no mask bytes", .truncated, { try appeared(patched(fresh, at: 128, le32(1))) }),
        ("mask of KIND_DELTA", .invalid, { try event(patched(moved, at: 168, [1])) }),
        ("mask occupied voxel not known", .invalid, { try event(patched(moved, at: 276, [0x00])) }),
        ("mask brick with no known voxel", .invalid, { try event(patched(patched(moved, at: 208, [0x00]), at: 276, [0x00])) }),
        ("mask compression zstd", .unsupported, { try event(patched(moved, at: 344, [1])) }),
        ("mask room_id not the event's", .invalid, { try event(patched(moved, at: 136, [0x78])) }),
        ("mask map_epoch not the event's", .invalid, { try event(patched(moved, at: 144, le32(44))) }),
        ("mask stamp sec not the event's", .invalid, { try event(patched(moved, at: 160, le32(1_760_400_011))) }),
        ("mask stamp nanosec not the event's", .invalid, { try event(patched(moved, at: 164, le32(0))) }),
        ("mask seq 1", .invalid, { try event(patched(moved, at: 152, le32(1))) }),
        ("mask base_epoch 1", .invalid, { try event(patched(moved, at: 176, le32(1))) }),
    ]
    for (label, want, body) in cases {
        let (got, text) = outcome(body)
        expect(got == want, "malformed '\(label)': expected \(want), got \(got) (\(text))")
        expect(!text.isEmpty, "malformed '\(label)': error carries a description")
    }
}

func testRoomEncoderEnforcesContract() {
    var unsorted = goldenDelta
    unsorted.bricks.append(unsorted.bricks[0])
    var shortMask = goldenDelta
    shortMask.bricks[0].occupied.removeLast()
    var occupiedNotKnown = goldenDelta
    occupiedNotKnown.bricks[0].occupied[5] = 0x10
    var allUnknown = goldenDelta
    allUnknown.bricks[0].occupied = mask(0)
    allUnknown.bricks[0].known = mask(0)
    var snapshotWithRemovals = goldenDelta
    snapshotWithRemovals.kind = .snapshot
    var movedWithoutFrom = goldenMoved
    movedWithoutFrom.fromCentroid = nil
    var appearedWithFrom = goldenAppeared
    appearedWithFrom.fromCentroid = SIMD3(0, 0, 0)
    var badAlpha = goldenAppeared
    badAlpha.alpha = 2
    var deltaMask = goldenMoved
    deltaMask.mask?.kind = .delta
    var maskOtherRoom = goldenMoved
    maskOtherRoom.mask?.header.roomId = "lab-b"
    var maskOtherEpoch = goldenMoved
    maskOtherEpoch.mask?.header.mapEpoch = 42
    var maskOtherStamp = goldenMoved
    maskOtherStamp.mask?.header.stamp.nanosec = 0
    var maskWithSeq = goldenMoved
    maskWithSeq.mask?.header.seq = 3
    var maskWithBase = goldenMoved
    maskWithBase.mask?.baseEpoch = 42
    var sentinelHz = goldenStatus
    sentinelHz.mid360Hz = 0xFFFF
    var emptyRoom = goldenEpoch
    emptyRoom.header.roomId = ""
    var nulRoom = goldenEpoch
    nulRoom.header.roomId = "lab\u{0}x"
    var badNanos = goldenEpoch
    badNanos.header.stamp.nanosec = 1_000_000_000

    let cases: [(String, () throws -> Data)] = [
        ("duplicate brick", { try unsorted.encodedCDR() }),
        ("63-byte mask", { try shortMask.encodedCDR() }),
        ("occupied voxel not known", { try occupiedNotKnown.encodedCDR() }),
        ("all-unknown brick", { try allUnknown.encodedCDR() }),
        ("snapshot with removals", { try snapshotWithRemovals.encodedCDR() }),
        ("MOVED without from_centroid", { try movedWithoutFrom.encodedCDR() }),
        ("APPEARED with from_centroid", { try appearedWithFrom.encodedCDR() }),
        ("alpha 2", { try badAlpha.encodedCDR() }),
        ("delta as mask", { try deltaMask.encodedCDR() }),
        ("mask room_id not the event's", { try maskOtherRoom.encodedCDR() }),
        ("mask map_epoch not the event's", { try maskOtherEpoch.encodedCDR() }),
        ("mask stamp not the event's", { try maskOtherStamp.encodedCDR() }),
        ("mask seq 3", { try maskWithSeq.encodedCDR() }),
        ("mask base_epoch 42", { try maskWithBase.encodedCDR() }),
        ("0xFFFF as a measured rate", { try sentinelHz.encodedCDR() }),
        ("empty room_id", { try emptyRoom.encodedCDR() }),
        ("NUL in room_id", { try nulRoom.encodedCDR() }),
        ("nanosec 1e9", { try badNanos.encodedCDR() }),
    ]
    for (label, body) in cases {
        do {
            _ = try body()
            expect(false, "encoder accepted \(label)")
        } catch RoomMemoryCDRError.invalid(let text) {
            expect(!text.isEmpty, "encoder '\(label)': error carries a description")
        } catch {
            expect(false, "encoder '\(label)': unexpected error \(error)")
        }
    }
}

// MARK: - Round Trips and Fuzz

private struct MessageFactory {
    var random: SplitMix64

    mutating func below(_ bound: Int) -> Int { Int(random.next() % UInt64(bound)) }
    mutating func coin() -> Bool { random.next() & 1 == 1 }
    mutating func fraction() -> Float { Float(random.unit()) }
    mutating func maybeFraction() -> Float? { coin() ? fraction() : nil }
    mutating func coordinate() -> Float { Float(random.unit() * 16 - 8) }
    mutating func vector() -> SIMD3<Float> { SIMD3(coordinate(), coordinate(), coordinate()) }
    mutating func time() -> Room.Time {
        Room.Time(sec: Int32(truncatingIfNeeded: random.next()), nanosec: UInt32(below(1_000_000_000)))
    }

    mutating func header() -> Room.Header {
        let names = ["lab", "lab-a", "bay 7", "räume", "x"]
        return Room.Header(roomId: names[below(names.count)], mapEpoch: random.next(), seq: random.next(), stamp: time())
    }

    mutating func brick(_ coordinate: Room.BrickCoordinate) -> Room.Brick {
        var occupied = [UInt8](repeating: 0, count: Room.Brick.maskBytes)
        var known = occupied
        for byte in 0..<Room.Brick.maskBytes {
            known[byte] = UInt8(truncatingIfNeeded: random.next())
            occupied[byte] = known[byte] & UInt8(truncatingIfNeeded: random.next())
        }
        known[below(Room.Brick.maskBytes)] |= 1
        return Room.Brick(coordinate: coordinate, occupied: occupied, known: known)
    }

    mutating func mapBricks(kind: Room.MapBricks.Kind) -> Room.MapBricks {
        var coordinates = Set<Room.BrickCoordinate>()
        for _ in 0..<below(12) {
            coordinates.insert(Room.BrickCoordinate(
                i: Int16(truncatingIfNeeded: random.next()),
                j: Int16(truncatingIfNeeded: random.next()),
                k: Int16(below(8))
            ))
        }
        var removed: [UInt32] = []
        if kind == .delta {
            var index: UInt32 = 0
            for _ in 0..<below(6) {
                index += UInt32(below(1000) + 1)
                removed.append(index)
            }
        }
        return Room.MapBricks(
            header: header(),
            kind: kind,
            baseEpoch: kind == .delta ? random.next() : 0,
            voxelSize: [0.02, 0.05, 0.1][below(3)],
            bricks: coordinates.sorted().map { brick($0) },
            removedBrickIndices: removed
        )
    }

    mutating func changeEvent() -> Room.ChangeEvent {
        let kind = Room.ChangeEvent.Kind.allCases[below(3)]
        let corner = vector()
        let extent = SIMD3<Float>(fraction(), fraction(), fraction())
        let first = time()
        var last = first
        last.sec = first.sec == Int32.max ? first.sec : first.sec + Int32(below(100))
        var parent = header()
        var maskMap: Room.MapBricks?
        if coin() {
            var map = mapBricks(kind: .snapshot)
            map.header = Room.Header(roomId: parent.roomId, mapEpoch: parent.mapEpoch, seq: 0, stamp: parent.stamp)
            maskMap = map
        }
        parent.seq = random.next()
        return Room.ChangeEvent(
            header: parent,
            eventId: random.next(),
            kind: kind,
            state: Room.ChangeEvent.State.allCases[below(5)],
            centroid: corner + extent / 2,
            aabb: Room.ChangeEvent.Box(min: corner, max: corner + extent),
            fromCentroid: kind == .moved ? vector() : nil,
            voxelCount: UInt32(below(100_000)),
            volumeM3: fraction(),
            evidence: maybeFraction(),
            alpha: maybeFraction(),
            sensors: Room.Sensors(rawValue: UInt8(below(4))),
            firstSeen: first,
            lastSeen: last,
            mask: maskMap
        )
    }

    mutating func memoryStatus() -> Room.MemoryStatus {
        Room.MemoryStatus(
            header: header(),
            state: Room.MemoryState.allCases[below(5)],
            brickCount: UInt32(truncatingIfNeeded: random.next()),
            openEventCount: UInt32(below(50)),
            coverageKnownFraction: maybeFraction(),
            poseResidualM: coin() ? fraction() * 0.2 : nil,
            windowS: coin() ? 0.5 + fraction() : nil,
            mid360Hz: coin() ? UInt16(below(0xFFFF)) : nil,
            jt128Hz: coin() ? UInt16(below(0xFFFF)) : nil,
            droppedWindows: UInt32(truncatingIfNeeded: random.next())
        )
    }

    mutating func epoch() -> Room.Epoch {
        Room.Epoch(
            header: header(),
            appearedCount: UInt32(truncatingIfNeeded: random.next()),
            disappearedCount: UInt32(truncatingIfNeeded: random.next()),
            movedCount: UInt32(truncatingIfNeeded: random.next()),
            coverageKnownFraction: maybeFraction(),
            state: Room.MemoryState.allCases[below(5)]
        )
    }
}

private func roundTrip<M: Equatable>(_ message: M, encode: (M) throws -> Data, decode: (Data) throws -> M, _ label: String) {
    do {
        let bytes = try encode(message)
        let decoded = try decode(bytes)
        expect(decoded == message, "\(label): decoded value differs")
        expect(try encode(decoded) == bytes, "\(label): re-encoding is not byte-identical")
    } catch {
        expect(false, "\(label): \(error)")
    }
}

func testRoomRandomRoundTrips() {
    var factory = MessageFactory(random: SplitMix64(seed: 0x00C0_FFEE))
    for index in 0..<300 {
        roundTrip(factory.mapBricks(kind: index % 2 == 0 ? .snapshot : .delta),
                  encode: { try $0.encodedCDR() }, decode: Room.MapBricks.decode(cdr:), "MapBricks round trip \(index)")
        roundTrip(factory.changeEvent(), encode: { try $0.encodedCDR() }, decode: Room.ChangeEvent.decode(cdr:), "ChangeEvent round trip \(index)")
        roundTrip(factory.memoryStatus(), encode: { try $0.encodedCDR() }, decode: Room.MemoryStatus.decode(cdr:), "MemoryStatus round trip \(index)")
        roundTrip(factory.epoch(), encode: { try $0.encodedCDR() }, decode: Room.Epoch.decode(cdr:), "Epoch round trip \(index)")
    }
}

func testRoomMutatedInputNeverCrashes() {
    // Deterministic fuzz over every golden: flip or overwrite bytes and cut tails. Anything
    // that decodes must satisfy the contract, so it must re-encode without error.
    var random = SplitMix64(seed: 0x5EED_0F_F022)
    var decoded = 0
    var rejected = 0
    for golden in littleEndianGoldens + bigEndianGoldens {
        let valid = [UInt8](goldenBytes(golden.name))
        for round in 0..<1000 {
            var bytes = valid
            for _ in 0...(random.next() % 3) {
                let index = Int(random.next() % UInt64(bytes.count))
                bytes[index] = round % 3 == 0
                    ? UInt8(truncatingIfNeeded: random.next())
                    : bytes[index] ^ (1 << UInt8(random.next() % 8))
            }
            if round % 5 == 0 {
                bytes.removeLast(Int(random.next() % UInt64(bytes.count)))
            }
            do {
                _ = try golden.reencode(Data(bytes))
                decoded += 1
            } catch is RoomMemoryCDRError {
                rejected += 1
            } catch {
                expect(false, "room fuzz \(golden.name): unexpected error type \(error)")
            }
        }
    }
    expect(decoded > 0 && rejected > 0, "room fuzz: expected both outcomes, got \(decoded) decoded / \(rejected) rejected")
}

// MARK: - Size Test

private func compressed(_ data: Data, _ algorithm: NSData.CompressionAlgorithm) -> Int {
    do {
        return try (data as NSData).compressed(using: algorithm).length
    } catch {
        expect(false, "size test: \(algorithm) compression failed: \(error)")
        return Int.max
    }
}

private func sizeLine(_ label: String, bricks: Int, _ data: Data) -> String {
    "\(label): \(bricks) bricks, raw \(data.count) B, zlib \(compressed(data, .zlib)) B, lzfse \(compressed(data, .lzfse)) B"
}

func testRoomSnapshotSizeBudget() {
    // A full-sphere accumulation (720 × 360 rays, seed 1) of the furnished 8 × 8 × 3 m room at
    // 5 cm voxels. Every brick with any evidence is sent, all-free ones included: an absent
    // brick means UNKNOWN. zlib here is Apple's raw DEFLATE (RFC 1951) at level 5.
    // With the floor at z = 0 the floor and walls straddle brick boundaries, so noisy hits fill a
    // second brick layer; the off-grid variant shifts the room by (0.2, 0.2, 0.1) m to measure that.
    let header = Room.Header(roomId: "lab", mapEpoch: 1, seq: 0, stamp: t10)
    let before = EvidenceGrid.scanned(SyntheticRoom(), scanner: .dense, seed: 1).bricks()
    let offGrid = Room.MapBricks(
        header: header, kind: .snapshot, baseEpoch: 0, voxelSize: 0.05,
        bricks: EvidenceGrid.scanned(SyntheticRoom(), scanner: .dense, seed: 1, offset: SIMD3(0.2, 0.2, 0.1)).bricks()
    )
    let after = EvidenceGrid.scanned(SyntheticRoom(extra: [SyntheticRoom.boxA]), scanner: .dense, seed: 1).bricks()

    let snapshot = Room.MapBricks(header: header, kind: .snapshot, baseEpoch: 0, voxelSize: 0.05, bricks: before)
    let surface = Room.MapBricks(header: header, kind: .snapshot, baseEpoch: 0, voxelSize: 0.05,
                                 bricks: before.filter { $0.occupied.contains { $0 != 0 } })

    // The box-added delta: bricks whose masks changed, and indices of bricks that lost all evidence.
    let beforeByCoordinate = Dictionary(uniqueKeysWithValues: before.map { ($0.coordinate, $0) })
    let afterCoordinates = Set(after.map(\.coordinate))
    let changed = after.filter { beforeByCoordinate[$0.coordinate] != $0 }
    let removed = before.indices.filter { !afterCoordinates.contains(before[$0].coordinate) }.map(UInt32.init)
    var deltaHeader = header
    deltaHeader.mapEpoch = 2
    let delta = Room.MapBricks(header: deltaHeader, kind: .delta, baseEpoch: 1, voxelSize: 0.05,
                               bricks: changed, removedBrickIndices: removed)

    // The matching APPEARED event: its mask is the newly occupied voxels.
    var maskBricks: [Room.Brick] = []
    for brick in after {
        let previous = beforeByCoordinate[brick.coordinate]
        var occupied = brick.occupied
        var known = brick.known
        for byte in 0..<Room.Brick.maskBytes {
            occupied[byte] &= ~(previous?.occupied[byte] ?? 0)
            known[byte] = occupied[byte]
        }
        if occupied.contains(where: { $0 != 0 }) {
            maskBricks.append(Room.Brick(coordinate: brick.coordinate, occupied: occupied, known: known))
        }
    }
    let box = SyntheticRoom.boxA
    let event = Room.ChangeEvent(
        header: deltaHeader,
        eventId: 1,
        kind: .appeared,
        state: .confirmed,
        centroid: SIMD3<Float>(box.center),
        aabb: Room.ChangeEvent.Box(min: SIMD3<Float>(box.min), max: SIMD3<Float>(box.max)),
        fromCentroid: nil,
        voxelCount: UInt32(maskBricks.reduce(0) { $0 + $1.occupied.reduce(0) { $0 + $1.nonzeroBitCount } }),
        volumeM3: 0.25 * 0.25 * 0.25,
        evidence: 1,
        alpha: nil,
        sensors: .all,
        firstSeen: t10,
        lastSeen: t10,
        mask: Room.MapBricks(header: Room.Header(roomId: "lab", mapEpoch: 2, seq: 0, stamp: t10),
                             kind: .snapshot, baseEpoch: 0, voxelSize: 0.05, bricks: maskBricks)
    )

    do {
        let snapshotBytes = try snapshot.encodedCDR()
        let offGridBytes = try offGrid.encodedCDR()
        let surfaceBytes = try surface.encodedCDR()
        let deltaBytes = try delta.encodedCDR()
        let eventBytes = try event.encodedCDR()
        let statusBytes = try goldenStatus.encodedCDR()
        let epochBytes = try goldenEpoch.encodedCDR()
        expect(try Room.MapBricks.decode(cdr: snapshotBytes) == snapshot, "size test: snapshot round-trips")

        let zlib = compressed(snapshotBytes, .zlib)
        let lzfse = compressed(snapshotBytes, .lzfse)
        expect(zlib <= 500_000 && lzfse <= 500_000, "size test: snapshot compressed \(zlib) / \(lzfse) B exceeds 500 KB")
        expect(compressed(offGridBytes, .zlib) <= 500_000 && compressed(offGridBytes, .lzfse) <= 500_000,
               "size test: off-grid snapshot compressed exceeds 500 KB")
        expect(snapshotBytes.count <= 2_000_000, "size test: raw snapshot \(snapshotBytes.count) B exceeds the 2 MB hard cap")
        expect(!changed.isEmpty && !maskBricks.isEmpty, "size test: box added touches bricks")

        print("  room memory size: " + sizeLine("snapshot (all observed bricks)", bricks: before.count, snapshotBytes))
        print("  room memory size: " + sizeLine("snapshot, room off the brick grid", bricks: offGrid.bricks.count, offGridBytes))
        print("  room memory size: " + sizeLine("surface bricks only (brief's model)", bricks: surface.bricks.count, surfaceBytes))
        print("  room memory size: " + sizeLine("delta for the 25 cm box (removed \(removed.count))", bricks: changed.count, deltaBytes))
        print("  room memory size: " + sizeLine("APPEARED event with mask", bricks: maskBricks.count, eventBytes))
        print("  room memory size: MemoryStatus \(statusBytes.count) B (5-char room_id), Epoch \(epochBytes.count) B (3-char room_id)")
    } catch {
        expect(false, "size test: \(error)")
    }
}

// MARK: - Test Runner Entry Point

func runRoomMemoryCDRTests() {
    testRoomGoldensDecodeAndEncode()
    print("  room memory CDR: hand-derived goldens decode (LE and BE) and the encoder reproduces them"); passed()
    testRoomGoldenSemantics()
    print("  room memory CDR: NaN/0xFFFF decode to nil; brick bit order and UNKNOWN ≠ free"); passed()
    testRoomSliceAndPadding()
    print("  room memory CDR: slices decode; ≤ 3 trailing pad bytes allowed, 4 rejected"); passed()
    testRoomTruncationAlwaysThrows()
    print("  room memory CDR safety: every truncation of every golden throws truncated"); passed()
    testRoomMalformedInputRejected()
    print("  room memory CDR safety: bad encapsulation, enums, invariants, counts, and compression rejected"); passed()
    testRoomEncoderEnforcesContract()
    print("  room memory CDR: the encoder refuses messages that break the contract"); passed()
    testRoomRandomRoundTrips()
    print("  room memory CDR: 1200 seeded random messages round-trip byte-identically"); passed()
    testRoomMutatedInputNeverCrashes()
    print("  room memory CDR safety: 7000 mutated goldens decode-and-re-encode or throw, never crash"); passed()
    testRoomSnapshotSizeBudget(); passed()
}
