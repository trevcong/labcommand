import Foundation
import LabCommandCore

// MARK: - Hand-Derived Goldens (PR 0.2)

// Tests/Fixtures/room/golden/hand/*.hex were written byte by byte from the .msg files and the
// CDR rules, not produced by the encoder, so encoding to them is not circular. Spark goldens
// from rclpy (scripts/spark/make_room_memory_goldens.py, same values) are a follow-up.
// DEFLATE and LZFSE output is not canonical, so the compressed goldens carry hand-built stored
// blocks: they pin the bytes up to the payload, and the uncompressed payload the encoder's stream
// must inflate to.

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
    droppedWindows: 3,
    eventsSeq: 12
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
    compression: .none,
    kind: .delta,
    baseEpoch: 42,
    voxelSize: 0.05,
    bricks: [Room.Brick(coordinate: Room.BrickCoordinate(i: -1, j: 2, k: -3), occupied: mask(0x01, last: 0x80), known: mask(0x03, last: 0x80))],
    removedBrickIndices: [0, 5]
)

private let goldenDeltaLZFSE: Room.MapBricks = {
    var delta = goldenDelta
    delta.compression = .lzfse
    return delta
}()

private let goldenSnapshotZlib = Room.MapBricks(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 13, stamp: t10),
    compression: .zlib,
    kind: .snapshot,
    baseEpoch: 0,
    voxelSize: 0.05,
    bricks: [
        Room.Brick(coordinate: Room.BrickCoordinate(i: -1, j: 2, k: -3), occupied: mask(0x01, last: 0x80), known: mask(0x03, last: 0x80)),
        Room.Brick(coordinate: Room.BrickCoordinate(i: 0, j: 0, k: 1), occupied: mask(0), known: [UInt8](repeating: 0xFF, count: Room.Brick.maskBytes)),
    ]
)

private let goldenRequestMap = Room.RequestSync(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 2, stamp: t10),
    since: Room.RequestSync.Position(epoch: 43, seq: 12),
    topic: .mapBricks
)

private let goldenRequestNone = Room.RequestSync(
    header: Room.Header(roomId: "lab-a", mapEpoch: Room.RequestSync.sinceNone, seq: 0,
                        stamp: Room.Time(sec: 1_760_400_000, nanosec: 500_000_000)),
    since: nil,
    topic: .events
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
        compression: .none,
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

// MARK: Protocol v2 goldens: the legacy values plus the trailing fields, one generation throughout.

private let goldenGeneration: UInt64 = 1_760_400_000_000
private let tokenA = Room.RequestToken(bytes: [0x5a, 0x1e, 0x7c, 0x3d, 0x9b, 0x02, 0x4f, 0x61, 0x8e, 0x33, 0xc4, 0xd5, 0xe6, 0xf7, 0x08, 0x19])!
private let tokenB = Room.RequestToken(bytes: [0xc1, 0x0f, 0xfe, 0xe0, 0x12, 0x34, 0x45, 0x67, 0xa9, 0x87, 0x65, 0x43, 0x21, 0x0f, 0xed, 0xcb])!
private let tokenC = Room.RequestToken(bytes: [0x0e, 0x8a, 0x3b, 0x5c, 0x77, 0x21, 0x4d, 0x90, 0xb4, 0x06, 0x59, 0x13, 0xc2, 0xae, 0x6f, 0x01])!

private func v2<M>(_ message: M, _ set: (inout M) -> Void) -> M {
    var copy = message
    set(&copy)
    return copy
}

private let goldenStatusV2 = v2(goldenStatus) { $0.publisherGeneration = goldenGeneration }
private let goldenDeltaV2 = v2(goldenDelta) { $0.publisherGeneration = goldenGeneration }
private let goldenAppearedV2 = v2(goldenAppeared) { $0.publisherGeneration = goldenGeneration }
private let goldenMovedV2 = v2(goldenMoved) {
    $0.publisherGeneration = goldenGeneration
    $0.mask?.publisherGeneration = goldenGeneration
}
private let goldenRequestNoneV2 = v2(goldenRequestNone) {
    $0.expectedGeneration = Room.generationUnknown
    $0.token = tokenA
}
private let goldenRequestMapV2 = Room.RequestSync(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 2, stamp: t10),
    since: Room.RequestSync.Position(epoch: 43, seq: 10),
    topic: .mapBricks,
    expectedGeneration: goldenGeneration,
    token: tokenC
)
private let goldenSyncEndEmpty = Room.SyncEnd(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 0, stamp: t10),
    publisherGeneration: goldenGeneration, token: .zero, topic: .events, bound: nil
)
private let goldenSyncEndFallback = Room.SyncEnd(
    header: Room.Header(roomId: "lab", mapEpoch: 43, seq: 1, stamp: t10),
    publisherGeneration: goldenGeneration, token: tokenB, topic: .events, bound: 120,
    delivered: [10...10, 21...120], unavailable: [11...20]
)
private let goldenSyncEndMap = Room.SyncEnd(
    header: Room.Header(roomId: "lab", mapEpoch: 45, seq: 2, stamp: t10),
    publisherGeneration: goldenGeneration, token: tokenC, topic: .mapBricks, bound: 13, delivered: [11...13]
)

/// Each v2 golden that extends a legacy one, and the legacy file whose length it shares up to its new fields.
private let v2LegacyTwin: [String: String] = [
    "memory_status_v2_le": "memory_status_le",
    "map_bricks_delta_v2_le": "map_bricks_delta_le",
    "map_bricks_delta_v2_be": "map_bricks_delta_be",
    "change_event_appeared_v2_le": "change_event_appeared_le",
    "change_event_moved_mask_v2_le": "change_event_moved_mask_le",
    "request_sync_events_v2_le": "request_sync_events_none_le",
    "request_sync_map_v2_be": "request_sync_map_be",
]

/// Where a compressed golden keeps its payload, in stream positions (after the 4-byte header).
private struct PayloadFraming {
    /// The `uint32` payload length; the encoder must match the golden up to here.
    var lengthPos: Int
    /// The uncompressed payload, verbatim inside the hand-built stored block.
    var verbatim: Range<Int>
    /// The algorithm that inflates the payload, used here independently of the codec.
    var algorithm: NSData.CompressionAlgorithm
}

/// One golden: its file, the decoder for its type, and the expected encoding of the expected value.
private struct Golden {
    var name: String
    var decodeMatches: (Data) throws -> Bool
    var encodeExpected: () throws -> Data
    /// Re-encodes whatever decodes, so the fuzz can check decode ⇒ valid.
    var reencode: (Data) throws -> Data
    /// Nil when the encoder must reproduce every byte.
    var framing: PayloadFraming?
}

private func golden<M: Equatable>(
    _ name: String,
    _ expected: M,
    decode: @escaping (Data) throws -> M,
    encode: @escaping (M) throws -> Data,
    framing: PayloadFraming? = nil
) -> Golden {
    Golden(
        name: name,
        decodeMatches: { try decode($0) == expected },
        encodeExpected: { try encode(expected) },
        reencode: { try encode(try decode($0)) },
        framing: framing
    )
}

private let littleEndianGoldens: [Golden] = [
    golden("memory_status_le", goldenStatus, decode: Room.MemoryStatus.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("epoch_le", goldenEpoch, decode: Room.Epoch.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("map_bricks_delta_le", goldenDelta, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("change_event_moved_mask_le", goldenMoved, decode: Room.ChangeEvent.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("change_event_appeared_le", goldenAppeared, decode: Room.ChangeEvent.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("map_bricks_snapshot_zlib_le", goldenSnapshotZlib, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() },
           framing: PayloadFraming(lengthPos: 64, verbatim: 73..<341, algorithm: .zlib)),
    golden("map_bricks_delta_lzfse_le", goldenDeltaLZFSE, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() },
           framing: PayloadFraming(lengthPos: 64, verbatim: 76..<218, algorithm: .lzfse)),
    golden("request_sync_map_le", goldenRequestMap, decode: Room.RequestSync.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("request_sync_events_none_le", goldenRequestNone, decode: Room.RequestSync.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("memory_status_v2_le", goldenStatusV2, decode: Room.MemoryStatus.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("map_bricks_delta_v2_le", goldenDeltaV2, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("change_event_appeared_v2_le", goldenAppearedV2, decode: Room.ChangeEvent.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("change_event_moved_mask_v2_le", goldenMovedV2, decode: Room.ChangeEvent.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("request_sync_events_v2_le", goldenRequestNoneV2, decode: Room.RequestSync.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("sync_end_events_empty_le", goldenSyncEndEmpty, decode: Room.SyncEnd.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("sync_end_events_fallback_le", goldenSyncEndFallback, decode: Room.SyncEnd.decode(cdr:), encode: { try $0.encodedCDR() }),
]

private let bigEndianGoldens: [Golden] = [
    golden("epoch_be", goldenEpoch, decode: Room.Epoch.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("map_bricks_delta_be", goldenDelta, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("request_sync_map_be", goldenRequestMap, decode: Room.RequestSync.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("map_bricks_delta_v2_be", goldenDeltaV2, decode: Room.MapBricks.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("request_sync_map_v2_be", goldenRequestMapV2, decode: Room.RequestSync.decode(cdr:), encode: { try $0.encodedCDR() }),
    golden("sync_end_map_replay_be", goldenSyncEndMap, decode: Room.SyncEnd.decode(cdr:), encode: { try $0.encodedCDR() }),
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
            if let framing = golden.framing {
                try checkCompressedEncoding(golden.name, encoded: encoded, hand: bytes, framing)
                expect(try golden.decodeMatches(encoded), "golden \(golden.name): the encoder's compressed bytes decode back to the value")
            } else {
                expect(encoded == bytes, "golden \(golden.name): encoder wrote \(encoded.count) bytes that differ from the hand bytes (\(bytes.count))")
            }
        } catch {
            expect(false, "golden \(golden.name): encode threw \(error)")
        }
    }
    for golden in bigEndianGoldens {
        let (decoded, text) = outcome { try golden.decodeMatches(goldenBytes(golden.name)) }
        expect(decoded == .decoded, "golden \(golden.name): \(text)")
    }
}

/// The encoder's bytes match the hand bytes up to the payload length, the length counts the rest,
/// and the encoder's own stream inflates (through Foundation, not the codec) to the hand payload.
private func checkCompressedEncoding(_ name: String, encoded: Data, hand: Data, _ framing: PayloadFraming) throws {
    let lengthAt = 4 + framing.lengthPos
    expect(encoded.prefix(lengthAt) == hand.prefix(lengthAt), "golden \(name): encoder differs from the hand bytes before the payload")
    let declared = encoded[lengthAt..<lengthAt + 4].reversed().reduce(0) { $0 << 8 | Int($1) }
    let payload = encoded.suffix(from: lengthAt + 4)
    expect(declared == payload.count, "golden \(name): payload length \(declared) is not the \(payload.count) bytes that follow")
    let inflated = try (Data(payload) as NSData).decompressed(using: framing.algorithm) as Data
    let verbatim = hand[(4 + framing.verbatim.lowerBound)..<(4 + framing.verbatim.upperBound)]
    expect(inflated == verbatim, "golden \(name): encoder payload inflates to \(inflated.count) bytes, not the hand payload")
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

    let allFree = goldenSnapshotZlib.bricks[1]
    expect(allFree.evidence(x: 3, y: 4, z: 5) == .free, "an all-free brick is sent and reads free, not unknown")
    expect(goldenRequestNone.since == nil && goldenRequestMap.since?.seq == 12, "SINCE_NONE decodes to a nil since")
    expect(Room.MapBricks(header: goldenDelta.header, kind: .snapshot, baseEpoch: 0, voxelSize: 0.05, bricks: []).compression == .zlib,
           "MapBricks compresses with zlib unless told otherwise")
}

func testRoomLayoutDetection() {
    do {
        // Legacy goldens decode with no generation, v2 goldens with one; the prefix is shared byte for byte.
        expect(try Room.MemoryStatus.decode(cdr: goldenBytes("memory_status_le")).publisherGeneration == nil, "legacy status: nil generation")
        for (name, legacy) in v2LegacyTwin where !name.hasPrefix("request_sync_map") {
            let bytes = goldenBytes(name)
            let base = goldenBytes(legacy)
            expect(bytes.prefix(base.count) == base, "\(name) starts with \(legacy) byte for byte")
        }
        // A v2 status with up to 3 bytes of rmw padding still decodes.
        expect(try Room.MemoryStatus.decode(cdr: goldenBytes("memory_status_v2_le") + Data(count: 3)) == goldenStatusV2, "v2 + 3 pad bytes")
        // MapBricks: 6 pad bytes after a 142-byte payload; ChangeEvent without a mask: 4 after the count.
        expect(goldenBytes("map_bricks_delta_v2_le").count == goldenBytes("map_bricks_delta_le").count + 6 + 8, "delta v2 = legacy + 6 pad + 8")
        expect(goldenBytes("change_event_appeared_v2_le").count == 148, "appeared v2 event is 148 bytes")
        expect(goldenBytes("change_event_moved_mask_v2_le").count == 364, "masked v2 event is 364 bytes")
        // SyncEnd: an empty list is followed by the next count at once; elements align to 8.
        expect(try goldenSyncEndEmpty.encodedCDR().count == 84, "SyncEnd d=0 u=0 is 84 bytes")
        expect(try v2(goldenSyncEndEmpty) { $0.bound = 5; $0.delivered = [0...5] }.encodedCDR().count == 88 + 16, "SyncEnd d=1 u=0")
        expect(try v2(goldenSyncEndEmpty) { $0.bound = 5; $0.unavailable = [0...5] }.encodedCDR().count == 84 + 16, "SyncEnd d=0 u=1")
        expect(try goldenSyncEndFallback.encodedCDR().count == 92 + 32 + 16, "SyncEnd d=2 u=1")
        expect(try goldenRequestMapV2.encodedCDR().count == 84, "v2 RequestSync for lab is 84 bytes")
        expect(goldenBytes("request_sync_map_v2_be").suffix(16) == Data(tokenC.bytes), "the token is never byte-swapped")
    } catch {
        expect(false, "layout detection: \(error)")
    }
}

func testRoomEmptyPayloadEveryCompression() {
    // n_bricks 0 and n_removed 0: an empty payload stays empty under NONE, and zlib and lzfse still
    // send a complete (non-empty) stream, such as raw DEFLATE 03 00.
    for compression in Room.MapBricks.Compression.allCases {
        let empty = Room.MapBricks(header: goldenDelta.header, compression: compression, kind: .snapshot, baseEpoch: 0, voxelSize: 0.05, bricks: [])
        do {
            let bytes = try empty.encodedCDR()
            let payloadLength = bytes[(4 + 64)..<(4 + 68)].reversed().reduce(0) { $0 << 8 | Int($1) }
            expect((payloadLength == 0) == (compression == .none), "empty \(compression) payload is \(payloadLength) bytes")
            expect(try Room.MapBricks.decode(cdr: bytes) == empty, "empty \(compression) snapshot round-trips")
        } catch {
            expect(false, "empty \(compression) snapshot: \(error)")
        }
    }
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
        // A v2 message cut where its legacy layout ends is that legacy message; cut inside its new
        // fields it is invalid. Neither is ever the golden.
        let legacyLength = v2LegacyTwin[golden.name].map { goldenBytes($0).count } ?? bytes.count
        for length in 0..<bytes.count {
            let (result, text) = outcome { try golden.decodeMatches(bytes.prefix(length)) }
            if length >= legacyLength {
                expect(result == .otherError || result == .invalid, "\(golden.name) cut to \(length) bytes: \(result) \(text)")
                continue
            }
            expect(result == .truncated, "\(golden.name) cut to \(length) bytes: \(result) \(text)")
            expect(text.contains("CDR: truncated"), "\(golden.name) cut to \(length): message '\(text)'")
        }
    }
    // At the legacy length exactly, each one is the legacy layout: no generation, no token.
    do {
        expect(try Room.MemoryStatus.decode(cdr: goldenBytes("memory_status_v2_le").prefix(80 + 4)) == goldenStatus,
               "v2 status cut at 80 bytes is the legacy golden")
        expect(try Room.ChangeEvent.decode(cdr: goldenBytes("change_event_moved_mask_v2_le").prefix(338 + 4)) == goldenMoved,
               "v2 masked event cut after its mask payload is the legacy golden")
        expect(try Room.RequestSync.decode(cdr: goldenBytes("request_sync_events_v2_le").prefix(57 + 4)).token == nil,
               "v2 request cut at its topic is a legacy request")
    } catch {
        expect(false, "v2 goldens cut at the legacy length decode: \(error)")
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
private func le64(_ value: UInt64) -> [UInt8] { withUnsafeBytes(of: value.littleEndian, Array.init) }

/// The zlib golden with its raw DEFLATE payload wrapped as RFC 1950 zlib (header 78 9c, Adler-32
/// trailer), which is what Python's plain `zlib.compress(raw)` emits. The protocol carries raw DEFLATE.
private func zlibWrappedSnapshot() -> Data {
    let hand = [UInt8](goldenBytes("map_bricks_snapshot_zlib_le"))
    let deflate = Array(hand[(4 + 68)...])
    let raw = deflate.dropFirst(5)
    var a: UInt32 = 1
    var b: UInt32 = 0
    for byte in raw {
        a = (a + UInt32(byte)) % 65521
        b = (b + a) % 65521
    }
    let wrapped = [0x78, 0x9c] + deflate + withUnsafeBytes(of: (b << 16 | a).bigEndian, Array.init)
    return Data(hand.prefix(4 + 64) + le32(UInt32(wrapped.count)) + wrapped)
}
private func f32(_ value: Float) -> [UInt8] { le32(value.bitPattern) }

func testRoomMalformedInputRejected() {
    let status = { (data: Data) in try Room.MemoryStatus.decode(cdr: data) == goldenStatus }
    let epoch = { (data: Data) in try Room.Epoch.decode(cdr: data) == goldenEpoch }
    let bricks = { (data: Data) in try Room.MapBricks.decode(cdr: data) == goldenDelta }
    let event = { (data: Data) in try Room.ChangeEvent.decode(cdr: data) == goldenMoved }
    let appeared = { (data: Data) in try Room.ChangeEvent.decode(cdr: data) == goldenAppeared }

    let snapshot = { (data: Data) in try Room.MapBricks.decode(cdr: data) == goldenSnapshotZlib }
    let lzfseDelta = { (data: Data) in try Room.MapBricks.decode(cdr: data) == goldenDeltaLZFSE }
    let request = { (data: Data) in try Room.RequestSync.decode(cdr: data) == goldenRequestMap }
    let requestNoneMatches = { (data: Data) in try Room.RequestSync.decode(cdr: data) == goldenRequestNone }

    let delta = "map_bricks_delta_le"
    let moved = "change_event_moved_mask_le"
    let fresh = "change_event_appeared_le"
    let zlib = "map_bricks_snapshot_zlib_le"
    let lzfse = "map_bricks_delta_lzfse_le"
    let requestMap = "request_sync_map_le"
    let requestNone = "request_sync_events_none_le"
    let none64 = [UInt8](repeating: 0xFF, count: 8)
    let zlibWrapped = zlibWrappedSnapshot()
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
        ("MapBricks compression 3", .invalid, { try bricks(patched(delta, at: 32, [3])) }),
        ("MapBricks kind 2", .invalid, { try bricks(patched(delta, at: 33, [2])) }),
        ("snapshot with removed bricks", .invalid, { try bricks(patched(delta, at: 33, [0])) }),
        ("delta map_epoch equal to base_epoch", .invalid, { try bricks(patched(delta, at: 8, le64(42))) }),
        ("voxel_size NaN", .invalid, { try bricks(patched(delta, at: 48, f32(.nan))) }),
        ("voxel_size 0", .invalid, { try bricks(patched(delta, at: 48, f32(0))) }),
        ("brick_dim 4", .unsupported, { try bricks(patched(delta, at: 52, [4])) }),
        ("n_bricks 2 with a 1-brick payload", .invalid, { try bricks(patched(delta, at: 56, le32(2))) }),
        ("n_bricks 2^32-1, past the 16 MiB payload limit", .invalid, { try bricks(patched(delta, at: 56, le32(0xFFFF_FFFF))) }),
        ("n_removed 2^32-1, past the 16 MiB payload limit", .invalid, { try bricks(patched(delta, at: 60, le32(0xFFFF_FFFF))) }),
        ("payload 141 bytes for 142", .invalid, { try bricks(patched(delta, at: 64, le32(141))) }),
        ("payload length 2^32-1", .truncated, { try bricks(patched(delta, at: 64, le32(0xFFFF_FFFF))) }),
        ("occupied voxel not known", .invalid, { try bricks(patched(delta, at: 74, [0x05])) }),
        ("removed_bricks_idx not increasing", .invalid, { try bricks(patched(delta, at: 206, le32(0))) }),
        ("COMPRESSION_ZLIB on an uncompressed payload", .invalid, { try bricks(patched(delta, at: 32, [1])) }),
        ("COMPRESSION_LZFSE on an uncompressed payload", .invalid, { try bricks(patched(delta, at: 32, [2])) }),
        ("zlib-wrapped stream (78 9c ... adler32)", .invalid, { try snapshot(zlibWrapped) }),
        ("zlib stored block of 267 bytes for 268", .invalid, { try snapshot(patched(patched(zlib, at: 69, [0x0b]), at: 71, [0xf4])) }),
        ("zlib stream cut short", .invalid, { try snapshot(Data(patched(zlib, at: 64, le32(272)).dropLast())) }),
        ("zlib stream longer than n_bricks says", .invalid, { try snapshot(patched(zlib, at: 56, le32(1))) }),
        ("empty zlib payload", .invalid, { try snapshot(Data(patched(patched(zlib, at: 56, le32(0)), at: 64, le32(0)).prefix(4 + 68))) }),
        ("lzfse stream without its end marker", .invalid, { try lzfseDelta(Data(patched(lzfse, at: 64, le32(150)).dropLast(4))) }),
        ("lzfse block count 141 for 142 bytes", .invalid, { try lzfseDelta(patched(lzfse, at: 72, le32(141))) }),
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
        ("mask of KIND_DELTA", .invalid, { try event(patched(moved, at: 169, [1])) }),
        ("mask occupied voxel not known", .invalid, { try event(patched(moved, at: 274, [0x00])) }),
        ("mask brick with no known voxel", .invalid, { try event(patched(patched(moved, at: 210, [0x00]), at: 274, [0x00])) }),
        ("mask compression 3", .invalid, { try event(patched(moved, at: 168, [3])) }),
        ("mask room_id not the event's", .invalid, { try event(patched(moved, at: 136, [0x78])) }),
        ("mask map_epoch not the event's", .invalid, { try event(patched(moved, at: 144, le32(44))) }),
        ("mask stamp sec not the event's", .invalid, { try event(patched(moved, at: 160, le32(1_760_400_011))) }),
        ("mask stamp nanosec not the event's", .invalid, { try event(patched(moved, at: 164, le32(0))) }),
        ("mask seq 1", .invalid, { try event(patched(moved, at: 152, le32(1))) }),
        ("mask base_epoch 1", .invalid, { try event(patched(moved, at: 176, le32(1))) }),
        ("RequestSync topic 2", .invalid, { try request(patched(requestMap, at: 48, [2])) }),
        ("RequestSync map_epoch not since_epoch", .invalid, { try request(patched(requestMap, at: 8, le32(44))) }),
        ("RequestSync since_seq SINCE_NONE alone", .invalid, { try request(patched(requestMap, at: 40, none64)) }),
        ("RequestSync since_epoch SINCE_NONE alone", .invalid, { try request(patched(patched(requestMap, at: 32, none64), at: 8, none64)) }),
        ("RequestSync since_epoch set, since_seq SINCE_NONE", .invalid, { try requestNoneMatches(patched(requestNone, at: 40, le64(0))) }),
        ("RequestSync map_epoch 0 with SINCE_NONE", .invalid, { try requestNoneMatches(patched(requestNone, at: 16, le64(0))) }),
    ] + v2MalformedCases()
    for (label, want, body) in cases {
        let (got, text) = outcome(body)
        expect(got == want, "malformed '\(label)': expected \(want), got \(got) (\(text))")
        expect(!text.isEmpty, "malformed '\(label)': error carries a description")
    }
}

/// Layout detection, the mask generation rule, request tokens and SyncEnd manifests.
private func v2MalformedCases() -> [(String, Outcome, () throws -> Bool)] {
    let status = { (data: Data) in try Room.MemoryStatus.decode(cdr: data) == goldenStatusV2 }
    let moved = { (data: Data) in try Room.ChangeEvent.decode(cdr: data) == goldenMovedV2 }
    let request = { (data: Data) in try Room.RequestSync.decode(cdr: data) == goldenRequestNoneV2 }
    let fallback = { (data: Data) in try Room.SyncEnd.decode(cdr: data) == goldenSyncEndFallback }
    let empty = { (data: Data) in try Room.SyncEnd.decode(cdr: data) == goldenSyncEndEmpty }
    let legacyStatus = goldenBytes("memory_status_le")
    let legacyRequest = goldenBytes("request_sync_events_none_le")
    let end = "sync_end_events_fallback_le"
    var cases: [(String, Outcome, () throws -> Bool)] = [
        ("legacy status + 8 zero bytes (generation 0)", .invalid, { try status(legacyStatus + Data(count: 8)) }),
        ("v2 status generation 0", .invalid, { try status(patched("memory_status_v2_le", at: 80, le64(0))) }),
        ("v2 status + 4 trailing bytes", .invalid, { try status(goldenBytes("memory_status_v2_le") + Data(count: 4)) }),
        ("masked event: mask generation not the parent's", .invalid,
         { try moved(patched("change_event_moved_mask_v2_le", at: 344, le64(goldenGeneration + 1))) }),
        ("masked event: only the mask's generation", .invalid,
         { try moved(goldenBytes("change_event_moved_mask_v2_le").dropLast(8)) }),
        ("RequestSync v2 all-zero token", .invalid, { try request(patched("request_sync_events_v2_le", at: 72, le64(0) + le64(0))) }),
        ("legacy RequestSync + 31 zero bytes (zero token)", .invalid, { try request(legacyRequest + Data(count: 31)) }),
        ("legacy RequestSync + 15 bytes (no token)", .invalid, { try request(legacyRequest + Data(count: 15)) }),
        ("SyncEnd generation 0", .invalid, { try empty(patched("sync_end_events_empty_le", at: 32, le64(0))) }),
        ("SyncEnd topic 2", .invalid, { try fallback(patched(end, at: 56, [2])) }),
        ("SyncEnd bound below a delivered range", .invalid, { try fallback(patched(end, at: 64, le64(119))) }),
        ("SyncEnd bound SEQ_NONE with ranges", .invalid, { try fallback(patched(end, at: 64, le64(.max))) }),
        ("SyncEnd delivered range first after last", .invalid, { try fallback(patched(end, at: 80, le64(11))) }),
        ("SyncEnd delivered ranges adjacent", .invalid, { try fallback(patched(end, at: 96, le64(11))) }),
        ("SyncEnd delivered ranges out of order", .invalid, { try fallback(patched(end, at: 80, le64(30) + le64(30))) }),
        ("SyncEnd unavailable overlaps delivered", .invalid, { try fallback(patched(end, at: 128, le64(21))) }),
        ("SyncEnd map answer with unavailable ranges", .invalid, { try fallback(patched(end, at: 56, [0])) }),
        ("SyncEnd delivered count past the end", .truncated, { try fallback(patched(end, at: 72, le32(0xFFFF_FFFF))) }),
        ("SyncEnd unavailable count 2 for 1", .truncated, { try fallback(patched(end, at: 112, le32(2))) }),
    ]
    for extra in 4..<8 {
        cases.append(("legacy status + \(extra) trailing bytes", .invalid, { try status(legacyStatus + Data(count: extra)) }))
    }
    return cases
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
    var deltaBackwards = goldenDelta
    deltaBackwards.header.mapEpoch = 41
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
    var overLimit = goldenDelta
    overLimit.removedBrickIndices = Array(0..<UInt32(4_194_271))
    var requestSentinelSeq = goldenRequestMap
    requestSentinelSeq.since?.seq = Room.RequestSync.sinceNone
    var requestEpochMismatch = goldenRequestMap
    requestEpochMismatch.header.mapEpoch = 44
    var requestNoneWithEpoch = goldenRequestNone
    requestNoneWithEpoch.header.mapEpoch = 0
    let statusGenerationZero = v2(goldenStatus) { $0.publisherGeneration = 0 }
    let deltaGenerationZero = v2(goldenDelta) { $0.publisherGeneration = 0 }
    let legacyMaskUnderV2 = v2(goldenMovedV2) { $0.mask?.publisherGeneration = nil }
    let v2MaskUnderLegacy = v2(goldenMovedV2) { $0.publisherGeneration = nil }
    let maskOtherGeneration = v2(goldenMovedV2) { $0.mask?.publisherGeneration = goldenGeneration + 1 }
    let tokenWithoutExpected = v2(goldenRequestMapV2) { $0.expectedGeneration = nil }
    let expectedWithoutToken = v2(goldenRequestMapV2) { $0.token = nil }
    let zeroToken = v2(goldenRequestMapV2) { $0.token = .zero }
    let endGenerationZero = v2(goldenSyncEndFallback) { $0.publisherGeneration = 0 }
    let endPastBound = v2(goldenSyncEndFallback) { $0.bound = 100 }
    let endNoneBound = v2(goldenSyncEndFallback) { $0.bound = nil }
    let endSentinelBound = v2(goldenSyncEndEmpty) { $0.bound = Room.SyncEnd.seqNone }
    let endAdjacent = v2(goldenSyncEndFallback) { $0.delivered = [10...20, 21...120]; $0.unavailable = [] }
    let endOverlap = v2(goldenSyncEndFallback) { $0.unavailable = [11...21] }
    let endMapUnavailable = v2(goldenSyncEndFallback) { $0.topic = .mapBricks }

    let cases: [(String, () throws -> Data)] = [
        ("duplicate brick", { try unsorted.encodedCDR() }),
        ("63-byte mask", { try shortMask.encodedCDR() }),
        ("occupied voxel not known", { try occupiedNotKnown.encodedCDR() }),
        ("all-unknown brick", { try allUnknown.encodedCDR() }),
        ("snapshot with removals", { try snapshotWithRemovals.encodedCDR() }),
        ("delta map_epoch before base_epoch", { try deltaBackwards.encodedCDR() }),
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
        ("payload 2 bytes past 16 MiB", { try overLimit.encodedCDR() }),
        ("RequestSync since_seq SINCE_NONE", { try requestSentinelSeq.encodedCDR() }),
        ("RequestSync map_epoch not since_epoch", { try requestEpochMismatch.encodedCDR() }),
        ("RequestSync map_epoch 0 with no since", { try requestNoneWithEpoch.encodedCDR() }),
        ("MemoryStatus generation 0", { try statusGenerationZero.encodedCDR() }),
        ("MapBricks generation 0", { try deltaGenerationZero.encodedCDR() }),
        ("legacy mask under a v2 event", { try legacyMaskUnderV2.encodedCDR() }),
        ("v2 mask under a legacy event", { try v2MaskUnderLegacy.encodedCDR() }),
        ("mask generation not the event's", { try maskOtherGeneration.encodedCDR() }),
        ("RequestSync token without expected_generation", { try tokenWithoutExpected.encodedCDR() }),
        ("RequestSync expected_generation without token", { try expectedWithoutToken.encodedCDR() }),
        ("RequestSync all-zero token", { try zeroToken.encodedCDR() }),
        ("SyncEnd generation 0", { try endGenerationZero.encodedCDR() }),
        ("SyncEnd range past the bound", { try endPastBound.encodedCDR() }),
        ("SyncEnd ranges with a SEQ_NONE bound", { try endNoneBound.encodedCDR() }),
        ("SyncEnd bound SEQ_NONE as a value", { try endSentinelBound.encodedCDR() }),
        ("SyncEnd adjacent ranges", { try endAdjacent.encodedCDR() }),
        ("SyncEnd delivered overlaps unavailable", { try endOverlap.encodedCDR() }),
        ("SyncEnd map answer with unavailable", { try endMapUnavailable.encodedCDR() }),
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

    /// Nil (the legacy layout) or a nonzero generation.
    mutating func generation() -> UInt64? { coin() ? nil : random.next() | 1 }

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
        var mapHeader = header()
        let baseEpoch = kind == .delta ? random.next() >> 1 : 0
        if kind == .delta {
            mapHeader.mapEpoch = baseEpoch + 1 + UInt64(below(10))
        }
        return Room.MapBricks(
            header: mapHeader,
            compression: Room.MapBricks.Compression.allCases[below(3)],
            kind: kind,
            baseEpoch: baseEpoch,
            voxelSize: [0.02, 0.05, 0.1][below(3)],
            bricks: coordinates.sorted().map { brick($0) },
            removedBrickIndices: removed,
            publisherGeneration: generation()
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
        let parentGeneration = generation()
        var maskMap: Room.MapBricks?
        if coin() {
            var map = mapBricks(kind: .snapshot)
            map.header = Room.Header(roomId: parent.roomId, mapEpoch: parent.mapEpoch, seq: 0, stamp: parent.stamp)
            map.publisherGeneration = parentGeneration
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
            mask: maskMap,
            publisherGeneration: parentGeneration
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
            droppedWindows: UInt32(truncatingIfNeeded: random.next()),
            eventsSeq: coin() ? random.next() % Room.MemoryStatus.eventsSeqNone : nil,
            publisherGeneration: generation()
        )
    }

    mutating func requestSync() -> Room.RequestSync {
        var request = Room.RequestSync(header: header(), since: nil, topic: Room.RequestSync.Topic.allCases[below(2)])
        if coin() {
            request.since = Room.RequestSync.Position(epoch: random.next() >> 1, seq: random.next() >> 1)
        }
        request.header.mapEpoch = request.since?.epoch ?? Room.RequestSync.sinceNone
        if coin() {
            request.expectedGeneration = coin() ? Room.generationUnknown : random.next()
            request.token = Room.RequestToken(using: &random)
        }
        return request
    }

    /// Canonical ranges below `bound`, as a publisher writes them.
    mutating func ranges(bound: UInt64, avoiding taken: [ClosedRange<UInt64>] = []) -> [ClosedRange<UInt64>] {
        var ranges: [ClosedRange<UInt64>] = []
        var next: UInt64 = UInt64(below(5))
        for _ in 0..<below(4) {
            let last = next + UInt64(below(20))
            guard last <= bound else { break }
            let range = next...last
            if !taken.contains(where: { $0.overlaps(range) }) { ranges.append(range) }
            next = last + 2 + UInt64(below(30))
        }
        return ranges
    }

    mutating func syncEnd() -> Room.SyncEnd {
        let topic = Room.RequestSync.Topic.allCases[below(2)]
        let bound: UInt64? = coin() ? nil : UInt64(below(400))
        let delivered = bound.map { ranges(bound: $0) } ?? []
        let unavailable = topic == .events ? bound.map { ranges(bound: $0, avoiding: delivered) } ?? [] : []
        return Room.SyncEnd(
            header: header(),
            publisherGeneration: random.next() | 1,
            token: coin() ? .zero : Room.RequestToken(using: &random),
            topic: topic,
            bound: bound,
            delivered: delivered,
            unavailable: unavailable
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

private let roundTripRounds = 300
private let roundTripTypes = 6
private let fuzzRoundsPerGolden = 1000

func testRoomRandomRoundTrips() {
    var factory = MessageFactory(random: SplitMix64(seed: 0x00C0_FFEE))
    for index in 0..<roundTripRounds {
        roundTrip(factory.mapBricks(kind: index % 2 == 0 ? .snapshot : .delta),
                  encode: { try $0.encodedCDR() }, decode: Room.MapBricks.decode(cdr:), "MapBricks round trip \(index)")
        roundTrip(factory.changeEvent(), encode: { try $0.encodedCDR() }, decode: Room.ChangeEvent.decode(cdr:), "ChangeEvent round trip \(index)")
        roundTrip(factory.memoryStatus(), encode: { try $0.encodedCDR() }, decode: Room.MemoryStatus.decode(cdr:), "MemoryStatus round trip \(index)")
        roundTrip(factory.epoch(), encode: { try $0.encodedCDR() }, decode: Room.Epoch.decode(cdr:), "Epoch round trip \(index)")
        roundTrip(factory.requestSync(), encode: { try $0.encodedCDR() }, decode: Room.RequestSync.decode(cdr:), "RequestSync round trip \(index)")
        roundTrip(factory.syncEnd(), encode: { try $0.encodedCDR() }, decode: Room.SyncEnd.decode(cdr:), "SyncEnd round trip \(index)")
    }
    // Tokens: UUID v4 bits, never zero, different on every draw.
    var random = SplitMix64(seed: 7)
    let tokens = (0..<64).map { _ in Room.RequestToken(using: &random) }
    expect(Set(tokens).count == 64 && tokens.allSatisfy { !$0.isZero && $0.bytes[6] >> 4 == 4 && $0.bytes[8] >> 6 == 2 },
           "random request tokens are distinct UUID v4 values")
}

func testRoomMutatedInputNeverCrashes() {
    // Deterministic fuzz over every golden: flip or overwrite bytes and cut tails. Anything
    // that decodes must satisfy the contract, so it must re-encode without error.
    var random = SplitMix64(seed: 0x5EED_0F_F022)
    var decoded = 0
    var rejected = 0
    for golden in littleEndianGoldens + bigEndianGoldens {
        let valid = [UInt8](goldenBytes(golden.name))
        for round in 0..<fuzzRoundsPerGolden {
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

/// Encoded message sizes under each compression, in `Compression.allCases` order.
private func encodedSizes(_ message: Room.MapBricks) throws -> [Int] {
    try Room.MapBricks.Compression.allCases.map { compression in
        var copy = message
        copy.compression = compression
        return try copy.encodedCDR().count
    }
}

private func encodedSizes(_ event: Room.ChangeEvent) throws -> [Int] {
    try Room.MapBricks.Compression.allCases.map { compression in
        var copy = event
        copy.mask?.compression = compression
        return try copy.encodedCDR().count
    }
}

private func sizeLine(_ label: String, bricks: Int, _ sizes: [Int]) -> String {
    "\(label): \(bricks) bricks, none \(sizes[0]) B, zlib \(sizes[1]) B, lzfse \(sizes[2]) B"
}

func testRoomSnapshotSizeBudget() {
    // A full-sphere accumulation (720 × 360 rays, seed 1) of the furnished 8 × 8 × 3 m room at
    // 5 cm voxels. Every brick with any evidence is sent, all-free ones included: an absent
    // brick means UNKNOWN. Sizes are whole encoded messages; zlib is Apple's raw DEFLATE at level 5.
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
        expect(snapshot.compression == .zlib, "size test: the snapshot uses the default compression, zlib")
        expect(try Room.MapBricks.decode(cdr: snapshotBytes) == snapshot, "size test: the zlib snapshot round-trips")

        let snapshotSizes = try encodedSizes(snapshot)
        let offGridSizes = try encodedSizes(offGrid)
        let zlibIndex = Room.MapBricks.Compression.allCases.firstIndex(of: .zlib)!
        expect(snapshotSizes[zlibIndex] == snapshotBytes.count, "size test: the default encoding is the zlib one")
        expect(snapshotSizes[zlibIndex] <= 500_000 && offGridSizes[zlibIndex] <= 500_000,
               "size test: zlib snapshot \(snapshotSizes[zlibIndex]) / off-grid \(offGridSizes[zlibIndex]) B exceeds the 500 KB target")
        expect(snapshotSizes.allSatisfy { $0 <= 2_000_000 } && offGridSizes.allSatisfy { $0 <= 2_000_000 },
               "size test: a snapshot exceeds the 2 MB hard cap under some compression")
        expect(!changed.isEmpty && !maskBricks.isEmpty, "size test: box added touches bricks")

        let request = try goldenRequestMap.encodedCDR()
        print("  room memory size: " + sizeLine("snapshot (all observed bricks)", bricks: before.count, snapshotSizes))
        print("  room memory size: " + sizeLine("snapshot, room off the brick grid", bricks: offGrid.bricks.count, offGridSizes))
        print("  room memory size: " + sizeLine("surface bricks only (brief's model)", bricks: surface.bricks.count, try encodedSizes(surface)))
        print("  room memory size: " + sizeLine("delta for the 25 cm box (removed \(removed.count))", bricks: changed.count, try encodedSizes(delta)))
        print("  room memory size: " + sizeLine("APPEARED event, mask compressed as shown", bricks: maskBricks.count, try encodedSizes(event)))
        print("  room memory size: MemoryStatus \(try goldenStatus.encodedCDR().count) B (5-char room_id), "
              + "Epoch \(try goldenEpoch.encodedCDR().count) B, RequestSync \(request.count) B (3-char room_id)")
    } catch {
        expect(false, "size test: \(error)")
    }
}

// MARK: - Test Runner Entry Point

func runRoomMemoryCDRTests() {
    testRoomGoldensDecodeAndEncode()
    print("  room memory CDR: \(littleEndianGoldens.count + bigEndianGoldens.count) hand-derived goldens decode (LE and BE); the encoder reproduces them (compressed: up to the payload, which inflates to the hand bytes)"); passed()
    testRoomGoldenSemantics()
    print("  room memory CDR: NaN/0xFFFF/SINCE_NONE decode to nil; brick bit order; UNKNOWN ≠ free; zlib by default"); passed()
    testRoomLayoutDetection()
    print("  room memory CDR v2: the trailing bytes decide legacy/v2/invalid; the mask decides for its event; SyncEnd lists align per element"); passed()
    testRoomEmptyPayloadEveryCompression()
    print("  room memory CDR: an empty payload round-trips under none, zlib and lzfse"); passed()
    testRoomSliceAndPadding()
    print("  room memory CDR: slices decode; ≤ 3 trailing pad bytes allowed, 4 rejected"); passed()
    testRoomTruncationAlwaysThrows()
    print("  room memory CDR safety: every truncation of every golden throws truncated; a v2 golden cut at its legacy length reads as legacy, never as itself"); passed()
    testRoomMalformedInputRejected()
    print("  room memory CDR safety: bad encapsulation, enums, invariants, counts, payload limit, and compressed streams rejected"); passed()
    testRoomEncoderEnforcesContract()
    print("  room memory CDR: the encoder refuses messages that break the contract"); passed()
    testRoomRandomRoundTrips()
    print("  room memory CDR: \(roundTripRounds * roundTripTypes) seeded random messages round-trip byte-identically"); passed()
    testRoomMutatedInputNeverCrashes()
    print("  room memory CDR safety: \(fuzzRoundsPerGolden * (littleEndianGoldens.count + bigEndianGoldens.count)) mutated goldens decode-and-re-encode or throw, never crash"); passed()
    testRoomSnapshotSizeBudget(); passed()
}
