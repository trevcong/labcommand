import Foundation

// MARK: - Room Memory CDR Codecs

/// Errors from encoding or decoding a room-memory message.
public enum RoomMemoryCDRError: Error, Sendable, Equatable {
    /// The buffer ends before a field does.
    case truncated(String)
    /// A value breaks the contract in `docs/ROOM-MEMORY-PROTOCOL.md`.
    case invalid(String)
    /// Well-formed input that v1 does not handle: encapsulation, `brick_dim`, or `compression`.
    case unsupported(String)
}

// Wire format, shared by all four messages (classic CDR, as rmw_fastrtps writes it on Jazzy):
// - 4-byte encapsulation header: `0x00 0x01` CDR little endian, `0x00 0x00` CDR big endian, 2 option bytes.
// - Every primitive is aligned to its own size (1, 2, 4 or 8), measured from the byte after the header.
// - string: uint32 length including the NUL, then the bytes and the NUL.
// - T[]: uint32 element count, then the elements. T[N]: N elements, no count.
// - Nested messages add no alignment of their own; `MapBricks[<=1]` is a count of 0 or 1, then the message.
// - Up to 3 trailing bytes are padding the rmw layer may add.
// Encoders always write little endian with option bytes 0.

extension RoomMemory.MapBricks {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate("MapBricks")
        var writer = RoomCDRWriter()
        try write(to: &writer)
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/MapBricks`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated, invalid, or unsupported input.
    public static func decode(cdr data: Data) throws -> RoomMemory.MapBricks {
        try RoomCDR.decode(data, message: "MapBricks") { reader in
            let message = try read(from: &reader, path: "")
            try message.validate("MapBricks")
            return message
        }
    }

    func write(to writer: inout RoomCDRWriter) throws {
        writer.header(header)
        writer.u8(kind.rawValue)
        writer.u64(baseEpoch)
        writer.f32(voxelSize)
        writer.u8(UInt8(RoomMemory.Brick.dim))

        writer.u32(UInt32(bricks.count * 3))
        writer.align(2)
        for brick in bricks {
            writer.u16(UInt16(bitPattern: brick.coordinate.i))
            writer.u16(UInt16(bitPattern: brick.coordinate.j))
            writer.u16(UInt16(bitPattern: brick.coordinate.k))
        }
        writer.u32(UInt32(bricks.count * RoomMemory.Brick.maskBytes))
        for brick in bricks { writer.raw(brick.occupied) }
        writer.u32(UInt32(bricks.count * RoomMemory.Brick.maskBytes))
        for brick in bricks { writer.raw(brick.known) }

        writer.u32(UInt32(removedBrickIndices.count))
        for index in removedBrickIndices { writer.u32(index) }
        writer.u8(RoomCDR.compressionNone)
    }

    /// Reads one MapBricks, top level (`path` empty) or nested (`path` such as `"mask[0]."`).
    /// The caller validates the result.
    static func read(from reader: inout RoomCDRReader, path: String) throws -> RoomMemory.MapBricks {
        let header = try reader.header(path)
        let kindValue = try reader.u8(path + "kind")
        guard let kind = Kind(rawValue: kindValue) else {
            throw RoomMemoryCDRError.invalid("MapBricks: \(path)kind \(kindValue) is not a KIND_* value")
        }
        let baseEpoch = try reader.u64(path + "base_epoch")
        let voxelSize = try reader.f32(path + "voxel_size")
        let brickDim = try reader.u8(path + "brick_dim")
        guard Int(brickDim) == RoomMemory.Brick.dim else {
            throw RoomMemoryCDRError.unsupported("MapBricks: \(path)brick_dim \(brickDim); v1 handles only \(RoomMemory.Brick.dim)")
        }

        let coordinateCount = try reader.count(elementSize: 2, path + "brick_ijk")
        guard coordinateCount % 3 == 0 else {
            throw RoomMemoryCDRError.invalid("MapBricks: \(path)brick_ijk holds \(coordinateCount) values, not 3 per brick")
        }
        let brickCount = coordinateCount / 3
        var coordinates: [RoomMemory.BrickCoordinate] = []
        coordinates.reserveCapacity(brickCount)
        for index in 0..<brickCount {
            let what = "\(path)brick_ijk[\(index)]"
            coordinates.append(RoomMemory.BrickCoordinate(
                i: Int16(bitPattern: try reader.u16(what)),
                j: Int16(bitPattern: try reader.u16(what)),
                k: Int16(bitPattern: try reader.u16(what))
            ))
        }

        let maskLength = brickCount * RoomMemory.Brick.maskBytes
        let occupiedStart = try reader.byteSequence(expectedCount: maskLength, path + "occ_bits")
        let knownStart = try reader.byteSequence(expectedCount: maskLength, path + "known_bits")
        var bricks: [RoomMemory.Brick] = []
        bricks.reserveCapacity(brickCount)
        for (index, coordinate) in coordinates.enumerated() {
            let offset = index * RoomMemory.Brick.maskBytes
            bricks.append(RoomMemory.Brick(
                coordinate: coordinate,
                occupied: reader.slice(occupiedStart + offset, RoomMemory.Brick.maskBytes),
                known: reader.slice(knownStart + offset, RoomMemory.Brick.maskBytes)
            ))
        }

        let removedCount = try reader.count(elementSize: 4, path + "removed_bricks_idx")
        var removed: [UInt32] = []
        removed.reserveCapacity(removedCount)
        for index in 0..<removedCount {
            removed.append(try reader.u32("\(path)removed_bricks_idx[\(index)]"))
        }

        let compression = try reader.u8(path + "compression")
        switch compression {
        case RoomCDR.compressionNone: break
        case RoomCDR.compressionZstd:
            throw RoomMemoryCDRError.unsupported("MapBricks: \(path)compression 1 (zstd) is reserved; v1 decodes only 0 (none)")
        default:
            throw RoomMemoryCDRError.invalid("MapBricks: \(path)compression \(compression) is not a COMPRESSION_* value")
        }

        return RoomMemory.MapBricks(
            header: header,
            kind: kind,
            baseEpoch: baseEpoch,
            voxelSize: voxelSize,
            bricks: bricks,
            removedBrickIndices: removed
        )
    }

    /// Checks the contract invariants that both the encoder and the decoder enforce.
    func validate(_ context: String) throws {
        try header.validate(context)
        guard voxelSize.isFinite, voxelSize > 0 else {
            throw RoomMemoryCDRError.invalid("\(context): voxel_size \(voxelSize) must be finite and > 0 m")
        }
        if kind == .snapshot, !removedBrickIndices.isEmpty {
            throw RoomMemoryCDRError.invalid("\(context): a snapshot carries \(removedBrickIndices.count) removed_bricks_idx")
        }
        for index in removedBrickIndices.indices.dropFirst() where removedBrickIndices[index - 1] >= removedBrickIndices[index] {
            throw RoomMemoryCDRError.invalid("\(context): removed_bricks_idx not strictly increasing at \(index)")
        }
        guard bricks.count * 3 <= Int(UInt32.max) else {
            throw RoomMemoryCDRError.invalid("\(context): \(bricks.count) bricks overflow a CDR sequence")
        }
        for (index, brick) in bricks.enumerated() {
            guard brick.occupied.count == RoomMemory.Brick.maskBytes, brick.known.count == RoomMemory.Brick.maskBytes else {
                throw RoomMemoryCDRError.invalid(
                    "\(context): brick \(index) masks are \(brick.occupied.count)/\(brick.known.count) bytes, not \(RoomMemory.Brick.maskBytes)"
                )
            }
            if index > 0, bricks[index - 1].coordinate >= brick.coordinate {
                throw RoomMemoryCDRError.invalid("\(context): brick \(index) is not after brick \(index - 1) in (i, j, k) order")
            }
            var anyKnown = false
            for byte in 0..<RoomMemory.Brick.maskBytes {
                if brick.occupied[byte] & ~brick.known[byte] != 0 {
                    throw RoomMemoryCDRError.invalid("\(context): brick \(index) marks a voxel occupied that is not known")
                }
                anyKnown = anyKnown || brick.known[byte] != 0
            }
            guard anyKnown else {
                throw RoomMemoryCDRError.invalid("\(context): brick \(index) has no known voxel; an all-unknown brick is omitted")
            }
        }
    }
}

extension RoomMemory.ChangeEvent {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u64(eventId)
        writer.u8(kind.rawValue)
        writer.u8(state.rawValue)
        writer.vector(centroid)
        writer.vector(aabb.min)
        writer.vector(aabb.max)
        writer.vector(fromCentroid ?? SIMD3(repeating: .nan))
        writer.u32(voxelCount)
        writer.f32(volumeM3)
        writer.f32(evidence ?? .nan)
        writer.f32(alpha ?? .nan)
        writer.u8(sensors.rawValue)
        writer.time(firstSeen)
        writer.time(lastSeen)
        if let mask {
            writer.u32(1)
            try mask.write(to: &writer)
        } else {
            writer.u32(0)
        }
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/ChangeEvent`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated, invalid, or unsupported input.
    public static func decode(cdr data: Data) throws -> RoomMemory.ChangeEvent {
        try RoomCDR.decode(data, message: "ChangeEvent") { reader in
            let header = try reader.header("")
            let eventId = try reader.u64("event_id")
            let kindValue = try reader.u8("kind")
            guard let kind = Kind(rawValue: kindValue) else {
                throw RoomMemoryCDRError.invalid("ChangeEvent: kind \(kindValue) is not a KIND_* value")
            }
            let stateValue = try reader.u8("state")
            guard let state = State(rawValue: stateValue) else {
                throw RoomMemoryCDRError.invalid("ChangeEvent: state \(stateValue) is not a STATE_* value")
            }
            let centroid = try reader.vector("centroid")
            let aabb = Box(min: try reader.vector("aabb[0..2]"), max: try reader.vector("aabb[3..5]"))
            let from = try reader.vector("from_centroid")
            let fromCentroid: SIMD3<Float>?
            switch (from.x.isNaN, from.y.isNaN, from.z.isNaN) {
            case (true, true, true): fromCentroid = nil
            case (false, false, false): fromCentroid = from
            default:
                throw RoomMemoryCDRError.invalid("ChangeEvent: from_centroid \(from) is partly NaN")
            }
            let voxelCount = try reader.u32("voxel_count")
            let volume = try reader.f32("volume_m3")
            let evidence = try reader.f32("evidence")
            let alpha = try reader.f32("alpha")
            let sensors = RoomMemory.Sensors(rawValue: try reader.u8("sensors"))
            let firstSeen = try reader.time("first_seen")
            let lastSeen = try reader.time("last_seen")

            let maskCount = try reader.u32("mask length")
            guard maskCount <= 1 else {
                throw RoomMemoryCDRError.invalid("ChangeEvent: mask holds \(maskCount) maps; the bound is 1")
            }
            let mask = maskCount == 1 ? try RoomMemory.MapBricks.read(from: &reader, path: "mask[0].") : nil

            let event = RoomMemory.ChangeEvent(
                header: header,
                eventId: eventId,
                kind: kind,
                state: state,
                centroid: centroid,
                aabb: aabb,
                fromCentroid: fromCentroid,
                voxelCount: voxelCount,
                volumeM3: volume,
                evidence: evidence.isNaN ? nil : evidence,
                alpha: alpha.isNaN ? nil : alpha,
                sensors: sensors,
                firstSeen: firstSeen,
                lastSeen: lastSeen,
                mask: mask
            )
            try event.validate()
            return event
        }
    }

    func validate() throws {
        let context = "ChangeEvent"
        try header.validate(context)
        try RoomCDR.requireFinite(centroid, "\(context): centroid")
        try RoomCDR.requireFinite(aabb.min, "\(context): aabb min")
        try RoomCDR.requireFinite(aabb.max, "\(context): aabb max")
        guard all(aabb.min .<= aabb.max) else {
            throw RoomMemoryCDRError.invalid("\(context): aabb min \(aabb.min) exceeds max \(aabb.max)")
        }
        switch (kind, fromCentroid) {
        case (.moved, nil):
            throw RoomMemoryCDRError.invalid("\(context): MOVED needs from_centroid")
        case (.moved, let from?):
            try RoomCDR.requireFinite(from, "\(context): from_centroid")
        case (_, .some):
            throw RoomMemoryCDRError.invalid("\(context): from_centroid is only for MOVED, not \(kind)")
        case (_, nil):
            break
        }
        guard volumeM3.isFinite, volumeM3 >= 0 else {
            throw RoomMemoryCDRError.invalid("\(context): volume_m3 \(volumeM3) must be finite and ≥ 0")
        }
        try RoomCDR.requireFraction(evidence, "\(context): evidence")
        try RoomCDR.requireFraction(alpha, "\(context): alpha")
        guard RoomMemory.Sensors.all.isSuperset(of: sensors) else {
            throw RoomMemoryCDRError.invalid("\(context): sensors 0x\(String(sensors.rawValue, radix: 16)) sets an undefined bit")
        }
        try RoomCDR.requireValid(firstSeen, "\(context): first_seen")
        try RoomCDR.requireValid(lastSeen, "\(context): last_seen")
        guard firstSeen <= lastSeen else {
            throw RoomMemoryCDRError.invalid("\(context): last_seen is before first_seen")
        }
        if let mask {
            guard mask.kind == .snapshot else {
                throw RoomMemoryCDRError.invalid("\(context): mask must be KIND_SNAPSHOT")
            }
            try mask.validate("\(context) mask[0]")
            // D14: the mask is a snapshot of the parent's map, stamped like the parent, with seq 0.
            guard mask.header.roomId == header.roomId else {
                throw RoomMemoryCDRError.invalid("\(context): mask room_id '\(mask.header.roomId)' is not the event's '\(header.roomId)'")
            }
            guard mask.header.mapEpoch == header.mapEpoch else {
                throw RoomMemoryCDRError.invalid("\(context): mask map_epoch \(mask.header.mapEpoch) is not the event's \(header.mapEpoch)")
            }
            guard mask.header.stamp == header.stamp else {
                throw RoomMemoryCDRError.invalid("\(context): mask stamp is not the event's stamp")
            }
            guard mask.header.seq == 0 else {
                throw RoomMemoryCDRError.invalid("\(context): mask seq \(mask.header.seq) must be 0")
            }
            guard mask.baseEpoch == 0 else {
                throw RoomMemoryCDRError.invalid("\(context): mask base_epoch \(mask.baseEpoch) must be 0")
            }
        }
    }
}

extension RoomMemory.MemoryStatus {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u8(state.rawValue)
        writer.u32(brickCount)
        writer.u32(openEventCount)
        writer.f32(coverageKnownFraction ?? .nan)
        writer.f32(poseResidualM ?? .nan)
        writer.f32(windowS ?? .nan)
        writer.u16(mid360Hz ?? RoomCDR.hzUnavailable)
        writer.u16(jt128Hz ?? RoomCDR.hzUnavailable)
        writer.u32(droppedWindows)
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/MemoryStatus`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated or invalid input.
    public static func decode(cdr data: Data) throws -> RoomMemory.MemoryStatus {
        try RoomCDR.decode(data, message: "MemoryStatus") { reader in
            let header = try reader.header("")
            let state = try reader.memoryState("state")
            let brickCount = try reader.u32("n_bricks")
            let openEventCount = try reader.u32("n_open_events")
            let coverage = try reader.f32("coverage_known_frac")
            let residual = try reader.f32("pose_residual_m")
            let window = try reader.f32("window_s")
            let mid360Hz = try reader.u16("per_sensor_hz[0]")
            let jt128Hz = try reader.u16("per_sensor_hz[1]")
            let status = RoomMemory.MemoryStatus(
                header: header,
                state: state,
                brickCount: brickCount,
                openEventCount: openEventCount,
                coverageKnownFraction: coverage.isNaN ? nil : coverage,
                poseResidualM: residual.isNaN ? nil : residual,
                windowS: window.isNaN ? nil : window,
                mid360Hz: mid360Hz == RoomCDR.hzUnavailable ? nil : mid360Hz,
                jt128Hz: jt128Hz == RoomCDR.hzUnavailable ? nil : jt128Hz,
                droppedWindows: try reader.u32("dropped_windows")
            )
            try status.validate()
            return status
        }
    }

    func validate() throws {
        let context = "MemoryStatus"
        try header.validate(context)
        try RoomCDR.requireFraction(coverageKnownFraction, "\(context): coverage_known_frac")
        if let poseResidualM, !(poseResidualM.isFinite && poseResidualM >= 0) {
            throw RoomMemoryCDRError.invalid("\(context): pose_residual_m \(poseResidualM) must be finite and ≥ 0")
        }
        if let windowS, !(windowS.isFinite && windowS > 0) {
            throw RoomMemoryCDRError.invalid("\(context): window_s \(windowS) must be finite and > 0")
        }
        if mid360Hz == RoomCDR.hzUnavailable || jt128Hz == RoomCDR.hzUnavailable {
            throw RoomMemoryCDRError.invalid("\(context): \(RoomCDR.hzUnavailable) Hz is the unavailable sentinel; use nil")
        }
    }
}

extension RoomMemory.Epoch {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u32(appearedCount)
        writer.u32(disappearedCount)
        writer.u32(movedCount)
        writer.f32(coverageKnownFraction ?? .nan)
        writer.u8(state.rawValue)
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/Epoch`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated or invalid input.
    public static func decode(cdr data: Data) throws -> RoomMemory.Epoch {
        try RoomCDR.decode(data, message: "Epoch") { reader in
            let header = try reader.header("")
            let appeared = try reader.u32("n_appeared")
            let disappeared = try reader.u32("n_disappeared")
            let moved = try reader.u32("n_moved")
            let coverage = try reader.f32("coverage_known_frac")
            let epoch = RoomMemory.Epoch(
                header: header,
                appearedCount: appeared,
                disappearedCount: disappeared,
                movedCount: moved,
                coverageKnownFraction: coverage.isNaN ? nil : coverage,
                state: try reader.memoryState("state")
            )
            try epoch.validate()
            return epoch
        }
    }

    func validate() throws {
        try header.validate("Epoch")
        try RoomCDR.requireFraction(coverageKnownFraction, "Epoch: coverage_known_frac")
    }
}

extension RoomMemory.Header {
    func validate(_ context: String) throws {
        guard !roomId.isEmpty else {
            throw RoomMemoryCDRError.invalid("\(context): room_id is empty")
        }
        guard !roomId.utf8.contains(0) else {
            throw RoomMemoryCDRError.invalid("\(context): room_id contains a NUL")
        }
        try RoomCDR.requireValid(stamp, "\(context): stamp")
    }
}

// MARK: - Shared Rules

enum RoomCDR {
    static let compressionNone: UInt8 = 0
    static let compressionZstd: UInt8 = 1
    static let hzUnavailable: UInt16 = 0xFFFF
    static let encapsulationSize = 4
    static let maxTrailingPadding = 3

    /// Runs `body` over the bytes, then rejects more than `maxTrailingPadding` unread bytes.
    static func decode<T>(_ data: Data, message: String, _ body: (inout RoomCDRReader) throws -> T) throws -> T {
        try data.withUnsafeBytes { bytes in
            var reader = try RoomCDRReader(bytes: bytes, message: message)
            let value = try body(&reader)
            try reader.finish()
            return value
        }
    }

    static func requireFinite(_ value: SIMD3<Float>, _ what: String) throws {
        guard value.x.isFinite, value.y.isFinite, value.z.isFinite else {
            throw RoomMemoryCDRError.invalid("\(what) \(value) is not finite")
        }
    }

    /// Nil (missing) passes; a present value must lie in `0...1`.
    static func requireFraction(_ value: Float?, _ what: String) throws {
        if let value, !(0...1).contains(value) {
            throw RoomMemoryCDRError.invalid("\(what) \(value) is outside 0...1")
        }
    }

    static func requireValid(_ time: RoomMemory.Time, _ what: String) throws {
        guard time.nanosec < 1_000_000_000 else {
            throw RoomMemoryCDRError.invalid("\(what).nanosec \(time.nanosec) is not below 1e9")
        }
    }
}

// MARK: - CDR Writer

/// Appends CDR little endian. Alignment is relative to the byte after the encapsulation header.
struct RoomCDRWriter {
    private var bytes: [UInt8] = [0x00, 0x01, 0x00, 0x00]

    var data: Data { Data(bytes) }

    mutating func align(_ size: Int) {
        while (bytes.count - RoomCDR.encapsulationSize) % size != 0 { bytes.append(0) }
    }

    mutating func u8(_ value: UInt8) { bytes.append(value) }
    mutating func u16(_ value: UInt16) { append(value) }
    mutating func u32(_ value: UInt32) { append(value) }
    mutating func u64(_ value: UInt64) { append(value) }
    mutating func f32(_ value: Float) { append(value.bitPattern) }
    mutating func raw(_ value: [UInt8]) { bytes.append(contentsOf: value) }

    mutating func vector(_ value: SIMD3<Float>) {
        f32(value.x)
        f32(value.y)
        f32(value.z)
    }

    mutating func string(_ value: String) {
        let utf8 = Array(value.utf8)
        u32(UInt32(utf8.count + 1))
        bytes.append(contentsOf: utf8)
        bytes.append(0)
    }

    mutating func time(_ value: RoomMemory.Time) {
        u32(UInt32(bitPattern: value.sec))
        u32(value.nanosec)
    }

    mutating func header(_ value: RoomMemory.Header) {
        string(value.roomId)
        u64(value.mapEpoch)
        u64(value.seq)
        time(value.stamp)
    }

    private mutating func append<T: FixedWidthInteger>(_ value: T) {
        align(MemoryLayout<T>.size)
        withUnsafeBytes(of: value.littleEndian) { bytes.append(contentsOf: $0) }
    }
}

// MARK: - CDR Reader

/// Bounds-checked cursor over a CDR buffer. Every read either succeeds in bounds or throws.
struct RoomCDRReader {
    private let bytes: UnsafeRawBufferPointer
    private let bigEndian: Bool
    private let message: String
    private var offset = RoomCDR.encapsulationSize

    init(bytes: UnsafeRawBufferPointer, message: String) throws {
        guard bytes.count >= RoomCDR.encapsulationSize else {
            throw RoomMemoryCDRError.truncated(
                "\(message) CDR: truncated encapsulation header (\(bytes.count) of \(RoomCDR.encapsulationSize) bytes)"
            )
        }
        switch (bytes[0], bytes[1]) {
        case (0x00, 0x01): bigEndian = false
        case (0x00, 0x00): bigEndian = true
        default:
            let found = String(format: "0x%02x 0x%02x", bytes[0], bytes[1])
            throw RoomMemoryCDRError.unsupported(
                "\(message) CDR: unsupported encapsulation \(found) (expected CDR_LE 0x00 0x01 or CDR_BE 0x00 0x00)"
            )
        }
        self.bytes = bytes
        self.message = message
    }

    /// Consume `length` bytes and return the offset they start at.
    mutating func take(_ length: Int, _ what: @autoclosure () -> String) throws -> Int {
        guard length <= bytes.count - offset else {
            throw RoomMemoryCDRError.truncated(
                "\(message) CDR: truncated \(what()) at byte \(offset) (need \(length), have \(max(0, bytes.count - offset)))"
            )
        }
        defer { offset += length }
        return offset
    }

    /// Copies `length` bytes starting at `start`, which a previous `take` returned.
    func slice(_ start: Int, _ length: Int) -> [UInt8] {
        Array(bytes[start..<start + length])
    }

    mutating func align(_ size: Int) {
        let misalignment = (offset - RoomCDR.encapsulationSize) % size
        if misalignment != 0 {
            offset += size - misalignment
        }
    }

    mutating func u8(_ what: @autoclosure () -> String) throws -> UInt8 {
        bytes[try take(1, what())]
    }

    mutating func u16(_ what: @autoclosure () -> String) throws -> UInt16 {
        try load(UInt16.self, what())
    }

    mutating func u32(_ what: @autoclosure () -> String) throws -> UInt32 {
        try load(UInt32.self, what())
    }

    mutating func u64(_ what: @autoclosure () -> String) throws -> UInt64 {
        try load(UInt64.self, what())
    }

    mutating func f32(_ what: @autoclosure () -> String) throws -> Float {
        Float(bitPattern: try load(UInt32.self, what()))
    }

    mutating func vector(_ what: @autoclosure () -> String) throws -> SIMD3<Float> {
        SIMD3(try f32(what() + ".x"), try f32(what() + ".y"), try f32(what() + ".z"))
    }

    mutating func time(_ what: @autoclosure () -> String) throws -> RoomMemory.Time {
        RoomMemory.Time(sec: Int32(bitPattern: try u32(what() + ".sec")), nanosec: try u32(what() + ".nanosec"))
    }

    /// CDR string: uint32 length including the NUL, then the bytes. Must be UTF-8 with no inner NUL.
    mutating func string(_ what: @autoclosure () -> String) throws -> String {
        let length = Int(try u32(what() + " length"))
        let start = try take(length, what())
        guard length >= 1, bytes[start + length - 1] == 0 else {
            throw RoomMemoryCDRError.invalid("\(message) CDR: \(what()) is not NUL-terminated")
        }
        let body = bytes[start..<start + length - 1]
        guard !body.contains(0), let text = String(bytes: body, encoding: .utf8) else {
            throw RoomMemoryCDRError.invalid("\(message) CDR: \(what()) is not NUL-free UTF-8")
        }
        return text
    }

    mutating func header(_ path: String) throws -> RoomMemory.Header {
        RoomMemory.Header(
            roomId: try string(path + "room_id"),
            mapEpoch: try u64(path + "map_epoch"),
            seq: try u64(path + "seq"),
            stamp: try time(path + "stamp")
        )
    }

    mutating func memoryState(_ what: @autoclosure () -> String) throws -> RoomMemory.MemoryState {
        let value = try u8(what())
        guard let state = RoomMemory.MemoryState(rawValue: value) else {
            throw RoomMemoryCDRError.invalid("\(message): \(what()) \(value) is not a STATE_* value")
        }
        return state
    }

    /// Reads a sequence count and checks that `count × elementSize` bytes remain, so a
    /// hostile count throws before anything is reserved.
    mutating func count(elementSize: Int, _ what: @autoclosure () -> String) throws -> Int {
        let count = Int(try u32(what() + " length"))
        align(elementSize)
        let available = max(0, bytes.count - offset)
        guard count <= available / elementSize else {
            throw RoomMemoryCDRError.truncated(
                "\(message) CDR: truncated \(what()) at byte \(offset) (\(count) elements of \(elementSize) bytes, have \(available))"
            )
        }
        return count
    }

    /// Reads a `uint8[]` whose length must be exactly `expectedCount`; returns where its bytes start.
    mutating func byteSequence(expectedCount: Int, _ what: @autoclosure () -> String) throws -> Int {
        let count = try self.count(elementSize: 1, what())
        guard count == expectedCount else {
            throw RoomMemoryCDRError.invalid("\(message) CDR: \(what()) holds \(count) bytes, expected \(expectedCount)")
        }
        return try take(count, what())
    }

    /// Rejects unread bytes beyond the padding the rmw layer may add.
    func finish() throws {
        let trailing = bytes.count - offset
        guard trailing <= RoomCDR.maxTrailingPadding else {
            throw RoomMemoryCDRError.invalid("\(message) CDR: \(trailing) bytes after the message (at most \(RoomCDR.maxTrailingPadding) of padding)")
        }
    }

    private mutating func load<T: FixedWidthInteger>(_: T.Type, _ what: @autoclosure () -> String) throws -> T {
        align(MemoryLayout<T>.size)
        let raw = bytes.loadUnaligned(fromByteOffset: try take(MemoryLayout<T>.size, what()), as: T.self)
        return bigEndian ? T(bigEndian: raw) : T(littleEndian: raw)
    }
}
