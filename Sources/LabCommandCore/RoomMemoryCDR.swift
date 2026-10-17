import Compression
import Foundation

// MARK: - Room Memory CDR Codecs

/// Errors from encoding or decoding a room-memory message.
public enum RoomMemoryCDRError: Error, Sendable, Equatable {
    /// The buffer ends before a field does.
    case truncated(String)
    /// A value breaks the contract in `docs/ROOM-MEMORY-PROTOCOL.md`.
    case invalid(String)
    /// Well-formed input that v1 does not handle: the encapsulation or `brick_dim`.
    case unsupported(String)
}

// Wire format, shared by every room-memory message (classic CDR, as rmw_fastrtps writes it on Jazzy):
// - 4-byte encapsulation header: `0x00 0x01` CDR little endian, `0x00 0x00` CDR big endian, 2 option bytes.
// - Every primitive is aligned to its own size (1, 2, 4 or 8), measured from the byte after the header.
// - string: uint32 length including the NUL, then the bytes and the NUL.
// - T[]: uint32 element count, then the elements. T[N]: N elements, no count.
// - Nested messages add no alignment of their own; `MapBricks[<=1]` is a count of 0 or 1, then the message.
// - Up to 3 trailing bytes are padding the rmw layer may add.
// - MapBricks carries its bricks as one `uint8[] payload`, packed little endian in either byte order
//   and compressed as its `compression` field says (`RoomPayload`).
// Encoders always write little endian with option bytes 0.
//
// Protocol v2 (layout detection, protocol §1.4): the new fields sit after the last legacy field.
// There, at most 3 unread bytes is the legacy layout; padding to 8, then exactly the v2 fields
// (8 bytes, 16 for a ChangeEvent with a mask, 24 for RequestSync), then at most 3, is v2; anything
// else is invalid. With a mask, the nested MapBricks decides for both messages after its payload.
// Pad bytes are not checked, as with every alignment. No schema text is needed.

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
        writer.u8(compression.rawValue)
        writer.u8(kind.rawValue)
        writer.u64(baseEpoch)
        writer.f32(voxelSize)
        writer.u8(UInt8(RoomMemory.Brick.dim))
        writer.u32(UInt32(bricks.count))
        writer.u32(UInt32(removedBrickIndices.count))
        let payload = try RoomPayload.compress(packedPayload(), compression)
        writer.u32(UInt32(payload.count))
        writer.raw(payload)
        if let publisherGeneration {
            writer.u64(publisherGeneration)
        }
    }

    /// The payload before compression, packed little endian whatever the CDR byte order:
    /// `brick_ijk` (int16 × 3 per brick), `occ_bits` (64 B per brick), `known_bits` (64 B per brick),
    /// then `removed_bricks_idx` (uint32 each).
    func packedPayload() -> [UInt8] {
        var bytes: [UInt8] = []
        bytes.reserveCapacity(RoomPayload.size(bricks: bricks.count, removed: removedBrickIndices.count))
        for brick in bricks {
            for value in [brick.coordinate.i, brick.coordinate.j, brick.coordinate.k] {
                withUnsafeBytes(of: value.littleEndian) { bytes.append(contentsOf: $0) }
            }
        }
        for brick in bricks { bytes.append(contentsOf: brick.occupied) }
        for brick in bricks { bytes.append(contentsOf: brick.known) }
        for index in removedBrickIndices {
            withUnsafeBytes(of: index.littleEndian) { bytes.append(contentsOf: $0) }
        }
        return bytes
    }

    /// Reads one MapBricks, top level (`path` empty) or nested (`path` such as `"mask[0]."`).
    /// The caller validates the result. `v2Tail` is the size of the v2 fields that end the buffer
    /// after the payload: 8 at the top level, 16 inside `ChangeEvent.mask` (its own generation,
    /// then the parent's). The layout is decided there, once, for the whole buffer.
    static func read(from reader: inout RoomCDRReader, path: String, v2Tail: Int = 8) throws -> RoomMemory.MapBricks {
        let header = try reader.header(path)
        let compressionValue = try reader.u8(path + "compression")
        guard let compression = Compression(rawValue: compressionValue) else {
            throw RoomMemoryCDRError.invalid("MapBricks: \(path)compression \(compressionValue) is not a COMPRESSION_* value")
        }
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

        let brickCount = Int(try reader.u32(path + "n_bricks"))
        let removedCount = Int(try reader.u32(path + "n_removed"))
        let rawSize = RoomPayload.size(bricks: brickCount, removed: removedCount)
        guard rawSize <= RoomPayload.maxBytes else {
            throw RoomMemoryCDRError.invalid(
                "MapBricks: \(path)n_bricks \(brickCount) and n_removed \(removedCount) need a \(rawSize)-byte payload; the limit is \(RoomPayload.maxBytes)"
            )
        }
        let payloadLength = try reader.count(elementSize: 1, path + "payload")
        let payloadStart = try reader.take(payloadLength, path + "payload")
        let raw = try RoomPayload.decompress(
            reader.slice(payloadStart, payloadLength), compression, expectedCount: rawSize, "MapBricks: \(path)payload"
        )
        let (bricks, removed) = RoomPayload.unpack(raw, brickCount: brickCount, removedCount: removedCount)
        let generation = try reader.isV2(fields: v2Tail, after: path + "payload") ? try reader.u64(path + "publisher_generation") : nil

        return RoomMemory.MapBricks(
            header: header,
            compression: compression,
            kind: kind,
            baseEpoch: baseEpoch,
            voxelSize: voxelSize,
            bricks: bricks,
            removedBrickIndices: removed,
            publisherGeneration: generation
        )
    }

    /// Checks the contract invariants that both the encoder and the decoder enforce.
    func validate(_ context: String) throws {
        try header.validate(context)
        try RoomCDR.requireGeneration(publisherGeneration, "\(context): publisher_generation")
        guard voxelSize.isFinite, voxelSize > 0 else {
            throw RoomMemoryCDRError.invalid("\(context): voxel_size \(voxelSize) must be finite and > 0 m")
        }
        if kind == .delta, header.mapEpoch <= baseEpoch {
            throw RoomMemoryCDRError.invalid("\(context): delta map_epoch \(header.mapEpoch) is not after base_epoch \(baseEpoch)")
        }
        if kind == .snapshot, !removedBrickIndices.isEmpty {
            throw RoomMemoryCDRError.invalid("\(context): a snapshot carries \(removedBrickIndices.count) removed_bricks_idx")
        }
        for index in removedBrickIndices.indices.dropFirst() where removedBrickIndices[index - 1] >= removedBrickIndices[index] {
            throw RoomMemoryCDRError.invalid("\(context): removed_bricks_idx not strictly increasing at \(index)")
        }
        let rawSize = RoomPayload.size(bricks: bricks.count, removed: removedBrickIndices.count)
        guard rawSize <= RoomPayload.maxBytes else {
            throw RoomMemoryCDRError.invalid("\(context): \(bricks.count) bricks need a \(rawSize)-byte payload; the limit is \(RoomPayload.maxBytes)")
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
        if let publisherGeneration {
            writer.u64(publisherGeneration)
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
            let mask = maskCount == 1 ? try RoomMemory.MapBricks.read(from: &reader, path: "mask[0].", v2Tail: 16) : nil
            // With a mask, the nested reader decided the layout for both messages after its payload.
            let v2 = try mask.map { $0.publisherGeneration != nil } ?? reader.isV2(fields: 8, after: "mask length")
            let generation = v2 ? try reader.u64("publisher_generation") : nil

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
                mask: mask,
                publisherGeneration: generation
            )
            try event.validate()
            return event
        }
    }

    func validate() throws {
        let context = "ChangeEvent"
        try header.validate(context)
        try RoomCDR.requireGeneration(publisherGeneration, "\(context): publisher_generation")
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
            // A legacy event has a legacy mask, a v2 event a v2 mask of the same generation.
            guard mask.publisherGeneration == publisherGeneration else {
                throw RoomMemoryCDRError.invalid(
                    "\(context): mask publisher_generation \(String(describing: mask.publisherGeneration)) is not the event's \(String(describing: publisherGeneration))"
                )
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
        writer.u64(eventsSeq ?? Self.eventsSeqNone)
        if let publisherGeneration {
            writer.u64(publisherGeneration)
        }
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
            let droppedWindows = try reader.u32("dropped_windows")
            let eventsSeq = try reader.u64("events_seq")
            let generation = try reader.isV2(fields: 8, after: "events_seq") ? try reader.u64("publisher_generation") : nil
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
                droppedWindows: droppedWindows,
                eventsSeq: eventsSeq == Self.eventsSeqNone ? nil : eventsSeq,
                publisherGeneration: generation
            )
            try status.validate()
            return status
        }
    }

    func validate() throws {
        let context = "MemoryStatus"
        try header.validate(context)
        try RoomCDR.requireGeneration(publisherGeneration, "\(context): publisher_generation")
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
        if eventsSeq == Self.eventsSeqNone {
            throw RoomMemoryCDRError.invalid("\(context): \(Self.eventsSeqNone) is the EVENTS_SEQ_NONE sentinel; use nil")
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
        if let publisherGeneration {
            writer.u64(publisherGeneration)
        }
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
            let state = try reader.memoryState("state")
            let generation = try reader.isV2(fields: 8, after: "state") ? try reader.u64("publisher_generation") : nil
            let epoch = RoomMemory.Epoch(
                header: header,
                appearedCount: appeared,
                disappearedCount: disappeared,
                movedCount: moved,
                coverageKnownFraction: coverage.isNaN ? nil : coverage,
                state: state,
                publisherGeneration: generation
            )
            try epoch.validate()
            return epoch
        }
    }

    func validate() throws {
        try header.validate("Epoch")
        try RoomCDR.requireGeneration(publisherGeneration, "Epoch: publisher_generation")
        try RoomCDR.requireFraction(coverageKnownFraction, "Epoch: coverage_known_frac")
    }
}

extension RoomMemory.Activity {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u8(active ? 1 : 0)
        writer.u8(sensors.rawValue)
        writer.u8(largestSensor.rawValue)
        writer.u32(regionCount)
        writer.u32(voxelCount)
        let missing = SIMD3<Float>(repeating: .nan)
        writer.vector(centroid ?? missing)
        writer.vector(aabb?.min ?? missing)
        writer.vector(aabb?.max ?? missing)
        writer.f32(sinceS ?? .nan)
        if let publisherGeneration {
            writer.u64(publisherGeneration)
        }
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/Activity`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated or invalid input.
    public static func decode(cdr data: Data) throws -> RoomMemory.Activity {
        try RoomCDR.decode(data, message: "Activity") { reader in
            let header = try reader.header("")
            let active = try reader.u8("active")
            guard active <= 1 else {
                throw RoomMemoryCDRError.invalid("Activity: active \(active) is not a bool")
            }
            let sensors = RoomMemory.Sensors(rawValue: try reader.u8("sensors"))
            let largestSensor = RoomMemory.Sensors(rawValue: try reader.u8("largest_sensor"))
            let regionCount = try reader.u32("n_regions")
            let voxelCount = try reader.u32("voxel_count")
            // Each triple is all NaN (missing) or finite-or-invalid; never partly NaN.
            func optional(_ value: SIMD3<Float>, _ what: String) throws -> SIMD3<Float>? {
                switch (value.x.isNaN, value.y.isNaN, value.z.isNaN) {
                case (true, true, true): return nil
                case (false, false, false): return value
                default: throw RoomMemoryCDRError.invalid("Activity: \(what) \(value) is partly NaN")
                }
            }
            let centroid = try optional(try reader.vector("centroid"), "centroid")
            let low = try optional(try reader.vector("aabb[0..2]"), "aabb min")
            let high = try optional(try reader.vector("aabb[3..5]"), "aabb max")
            let aabb: RoomMemory.ChangeEvent.Box?
            switch (low, high) {
            case (nil, nil): aabb = nil
            case let (low?, high?): aabb = .init(min: low, max: high)
            default: throw RoomMemoryCDRError.invalid("Activity: aabb is half NaN")
            }
            let since = try reader.f32("since_s")
            let generation = try reader.isV2(fields: 8, after: "since_s") ? try reader.u64("publisher_generation") : nil
            let activity = RoomMemory.Activity(
                header: header,
                active: active == 1,
                sensors: sensors,
                largestSensor: largestSensor,
                regionCount: regionCount,
                voxelCount: voxelCount,
                centroid: centroid,
                aabb: aabb,
                sinceS: since.isNaN ? nil : since,
                publisherGeneration: generation
            )
            try activity.validate()
            return activity
        }
    }

    func validate() throws {
        let context = "Activity"
        try header.validate(context)
        try RoomCDR.requireGeneration(publisherGeneration, "\(context): publisher_generation")
        guard RoomMemory.Sensors.all.isSuperset(of: sensors) else {
            throw RoomMemoryCDRError.invalid("\(context): sensors 0x\(String(sensors.rawValue, radix: 16)) sets an undefined bit")
        }
        if active {
            guard regionCount >= 1, voxelCount >= 1 else {
                throw RoomMemoryCDRError.invalid("\(context): active needs at least one region and voxel")
            }
            guard largestSensor == .mid360 || largestSensor == .jt128, sensors.contains(largestSensor) else {
                throw RoomMemoryCDRError.invalid(
                    "\(context): largest_sensor \(largestSensor.rawValue) is not one bit of sensors \(sensors.rawValue)")
            }
            guard let centroid, let aabb, sinceS != nil else {
                throw RoomMemoryCDRError.invalid("\(context): active needs the largest region's centroid and aabb, and since_s")
            }
            try RoomCDR.requireFinite(centroid, "\(context): centroid")
            try RoomCDR.requireFinite(aabb.min, "\(context): aabb min")
            try RoomCDR.requireFinite(aabb.max, "\(context): aabb max")
            guard all(aabb.min .<= aabb.max) else {
                throw RoomMemoryCDRError.invalid("\(context): aabb min \(aabb.min) exceeds max \(aabb.max)")
            }
        } else {
            guard regionCount == 0, voxelCount == 0, sensors.isEmpty, largestSensor.isEmpty else {
                throw RoomMemoryCDRError.invalid("\(context): inactive must have zero counts and sensor bits")
            }
            guard centroid == nil, aabb == nil else {
                throw RoomMemoryCDRError.invalid("\(context): inactive must have NaN geometry")
            }
        }
        if let sinceS, !(sinceS.isFinite && sinceS >= 0) {
            throw RoomMemoryCDRError.invalid("\(context): since_s \(sinceS) must be finite and ≥ 0, or NaN")
        }
    }
}

extension RoomMemory.RequestSync {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u64(since?.epoch ?? Self.sinceNone)
        writer.u64(since?.seq ?? Self.sinceNone)
        writer.u8(topic.rawValue)
        if let expectedGeneration, let token {
            writer.u64(expectedGeneration)
            writer.raw(token.bytes)
        }
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/RequestSync`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated or invalid input.
    public static func decode(cdr data: Data) throws -> RoomMemory.RequestSync {
        try RoomCDR.decode(data, message: "RequestSync") { reader in
            let header = try reader.header("")
            let sinceEpoch = try reader.u64("since_epoch")
            let sinceSeq = try reader.u64("since_seq")
            let topicValue = try reader.u8("topic")
            guard let topic = Topic(rawValue: topicValue) else {
                throw RoomMemoryCDRError.invalid("RequestSync: topic \(topicValue) is not a TOPIC_* value")
            }
            let since: Position?
            switch (sinceEpoch == sinceNone, sinceSeq == sinceNone) {
            case (true, true): since = nil
            case (false, false): since = Position(epoch: sinceEpoch, seq: sinceSeq)
            default:
                throw RoomMemoryCDRError.invalid("RequestSync: since_epoch and since_seq must both be SINCE_NONE or neither")
            }
            var expected: UInt64?
            var token: RoomMemory.RequestToken?
            if try reader.isV2(fields: 8 + RoomMemory.RequestToken.count, after: "topic") {
                expected = try reader.u64("expected_generation")
                token = RoomMemory.RequestToken(uncheckedBytes: try reader.bytes(RoomMemory.RequestToken.count, "request_token"))
            }
            let request = RoomMemory.RequestSync(header: header, since: since, topic: topic, expectedGeneration: expected, token: token)
            try request.validate()
            return request
        }
    }

    func validate() throws {
        let context = "RequestSync"
        try header.validate(context)
        if let since, since.epoch == Self.sinceNone || since.seq == Self.sinceNone {
            throw RoomMemoryCDRError.invalid("\(context): \(Self.sinceNone) is the SINCE_NONE sentinel; use a nil since")
        }
        let sinceEpoch = since?.epoch ?? Self.sinceNone
        guard header.mapEpoch == sinceEpoch else {
            throw RoomMemoryCDRError.invalid("\(context): map_epoch \(header.mapEpoch) must repeat since_epoch \(sinceEpoch)")
        }
        guard (expectedGeneration == nil) == (token == nil) else {
            throw RoomMemoryCDRError.invalid("\(context): expected_generation and request_token are both set (v2) or both absent (legacy)")
        }
        if let token, token.isZero {
            throw RoomMemoryCDRError.invalid("\(context): an all-zero request_token is invalid in v2")
        }
    }
}

extension RoomMemory.SyncEnd {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u64(publisherGeneration)
        writer.raw(token.bytes)
        writer.u8(topic.rawValue)
        writer.u64(bound ?? Self.seqNone)
        writer.ranges(delivered)
        writer.ranges(unavailable)
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/SyncEnd`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated or invalid input.
    public static func decode(cdr data: Data) throws -> RoomMemory.SyncEnd {
        try RoomCDR.decode(data, message: "SyncEnd") { reader in
            let header = try reader.header("")
            let generation = try reader.u64("publisher_generation")
            let token = RoomMemory.RequestToken(uncheckedBytes: try reader.bytes(RoomMemory.RequestToken.count, "request_token"))
            let topicValue = try reader.u8("topic")
            guard let topic = RoomMemory.RequestSync.Topic(rawValue: topicValue) else {
                throw RoomMemoryCDRError.invalid("SyncEnd: topic \(topicValue) is not a TOPIC_* value")
            }
            let bound = try reader.u64("bound")
            let end = RoomMemory.SyncEnd(
                header: header,
                publisherGeneration: generation,
                token: token,
                topic: topic,
                bound: bound == Self.seqNone ? nil : bound,
                delivered: try reader.ranges("delivered"),
                unavailable: try reader.ranges("unavailable")
            )
            try end.validate()
            return end
        }
    }

    func validate() throws {
        let context = "SyncEnd"
        try header.validate(context)
        try RoomCDR.requireGeneration(publisherGeneration, "\(context): publisher_generation")
        if bound == Self.seqNone {
            throw RoomMemoryCDRError.invalid("\(context): \(Self.seqNone) is the SEQ_NONE sentinel; use a nil bound")
        }
        try RoomCDR.requireCanonical(delivered, bound: bound, "\(context): delivered")
        try RoomCDR.requireCanonical(unavailable, bound: bound, "\(context): unavailable")
        for range in delivered where unavailable.contains(where: { $0.overlaps(range) }) {
            throw RoomMemoryCDRError.invalid("\(context): delivered and unavailable overlap at \(range)")
        }
        if topic == .mapBricks, !unavailable.isEmpty {
            throw RoomMemoryCDRError.invalid("\(context): a map answer has no unavailable ranges")
        }
    }
}

extension RoomMemory.RigPose {
    /// Encodes the message as CDR little endian, including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when the message breaks the protocol contract.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.header(header)
        writer.u8(outcome.rawValue)
        writer.u8(causes.rawValue)
        writer.u8(sensors.rawValue)
        writer.u8(channel.mid360.rawValue)
        writer.u8(channel.jt128.rawValue)
        writer.u64(frameEpoch)
        writer.u32(realignedCount)
        for pose in [mid360Pose, jt128Pose] {
            writer.vector(pose?.translation ?? SIMD3(repeating: .nan))
            let rotation = pose?.rotation ?? SIMD4(repeating: .nan)
            for index in 0..<4 { writer.f32(rotation[index]) }
        }
        for pair in [deltaM, deltaDeg, inlierFrac, rmsM, observe] {
            writer.f32(pair.mid360 ?? .nan)
            writer.f32(pair.jt128 ?? .nan)
        }
        writer.f32(tiltDeg ?? .nan)
        writer.u8(imuOK ? 1 : 0)
        writer.time(movedAt ?? RoomMemory.Time(sec: 0, nanosec: 0))
        if let publisherGeneration {
            writer.u64(publisherGeneration)
        }
        return writer.data
    }

    /// Decodes a CDR `labcommand_msgs/RigPose`, including the encapsulation header.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated or invalid input.
    public static func decode(cdr data: Data) throws -> RoomMemory.RigPose {
        try RoomCDR.decode(data, message: "RigPose") { reader in
            let header = try reader.header("")
            let outcomeValue = try reader.u8("outcome")
            guard let outcome = Outcome(rawValue: outcomeValue) else {
                throw RoomMemoryCDRError.invalid("RigPose: outcome \(outcomeValue) is not an OUTCOME_* value")
            }
            let causes = Causes(rawValue: try reader.u8("causes"))
            let sensors = RoomMemory.Sensors(rawValue: try reader.u8("sensors"))
            func channel(_ index: Int) throws -> Channel {
                let value = try reader.u8("channel[\(index)]")
                guard let channel = Channel(rawValue: value) else {
                    throw RoomMemoryCDRError.invalid("RigPose: channel[\(index)] \(value) is not a CHANNEL_* value")
                }
                return channel
            }
            let channels = RoomMemory.SensorPair(mid360: try channel(0), jt128: try channel(1))
            let frameEpoch = try reader.u64("frame_epoch")
            let realignedCount = try reader.u32("n_realigned")
            // Each pose is all NaN (not configured) or finite-or-invalid; never partly NaN.
            func pose(_ name: String) throws -> Pose? {
                let values = try (0..<7).map { try reader.f32("\(name)[\($0)]") }
                switch values.filter(\.isNaN).count {
                case 7: return nil
                case 0: return Pose(translation: SIMD3(values[0], values[1], values[2]),
                                    rotation: SIMD4(values[3], values[4], values[5], values[6]))
                default: throw RoomMemoryCDRError.invalid("RigPose: \(name) \(values) is partly NaN")
                }
            }
            let mid360Pose = try pose("mid360_pose")
            let jt128Pose = try pose("jt128_pose")
            func pair(_ name: String) throws -> RoomMemory.SensorPair<Float?> {
                let mid360 = try reader.f32("\(name)[0]")
                let jt128 = try reader.f32("\(name)[1]")
                return .init(mid360: mid360.isNaN ? nil : mid360, jt128: jt128.isNaN ? nil : jt128)
            }
            let deltaM = try pair("delta_m")
            let deltaDeg = try pair("delta_deg")
            let inlierFrac = try pair("inlier_frac")
            let rmsM = try pair("rms_m")
            let observe = try pair("observe")
            let tilt = try reader.f32("tilt_deg")
            let imuOK = try reader.u8("imu_ok")
            guard imuOK <= 1 else {
                throw RoomMemoryCDRError.invalid("RigPose: imu_ok \(imuOK) is not a bool")
            }
            let movedAt = try reader.time("moved_at")
            let generation = try reader.isV2(fields: 8, after: "moved_at") ? try reader.u64("publisher_generation") : nil
            let rig = RoomMemory.RigPose(
                header: header,
                outcome: outcome,
                causes: causes,
                sensors: sensors,
                channel: channels,
                frameEpoch: frameEpoch,
                realignedCount: realignedCount,
                mid360Pose: mid360Pose,
                jt128Pose: jt128Pose,
                deltaM: deltaM,
                deltaDeg: deltaDeg,
                inlierFrac: inlierFrac,
                rmsM: rmsM,
                observe: observe,
                tiltDeg: tilt.isNaN ? nil : tilt,
                imuOK: imuOK == 1,
                movedAt: movedAt == RoomMemory.Time(sec: 0, nanosec: 0) ? nil : movedAt,
                publisherGeneration: generation
            )
            try rig.validate()
            return rig
        }
    }

    func validate() throws {
        let context = "RigPose"
        try header.validate(context)
        try RoomCDR.requireGeneration(publisherGeneration, "\(context): publisher_generation")
        guard Causes.all.isSuperset(of: causes) else {
            throw RoomMemoryCDRError.invalid("\(context): causes 0x\(String(causes.rawValue, radix: 16)) sets an undefined bit")
        }
        guard RoomMemory.Sensors.all.isSuperset(of: sensors) else {
            throw RoomMemoryCDRError.invalid("\(context): sensors 0x\(String(sensors.rawValue, radix: 16)) sets an undefined bit")
        }
        for (index, result, bit) in [(0, channel.mid360, RoomMemory.Sensors.mid360), (1, channel.jt128, .jt128)]
        where !sensors.contains(bit) && result != .none && result != .unavailable {
            throw RoomMemoryCDRError.invalid("\(context): channel[\(index)] \(result) for a sensor outside sensors")
        }
        for (name, pose) in [("mid360_pose", mid360Pose), ("jt128_pose", jt128Pose)] {
            guard let pose else { continue }
            try RoomCDR.requireFinite(pose.translation, "\(context): \(name) translation")
            guard pose.rotation.x.isFinite, pose.rotation.y.isFinite, pose.rotation.z.isFinite, pose.rotation.w.isFinite else {
                throw RoomMemoryCDRError.invalid("\(context): \(name) rotation \(pose.rotation) is not finite")
            }
            let q = SIMD4<Double>(pose.rotation)
            let norm = (q * q).sum().squareRoot()
            guard abs(norm - 1) <= 1e-3 else {
                throw RoomMemoryCDRError.invalid("\(context): \(name) quaternion norm \(norm) is not 1")
            }
        }
        let limits: [(String, RoomMemory.SensorPair<Float?>, Float?)] = [
            ("delta_m", deltaM, nil), ("delta_deg", deltaDeg, 180), ("inlier_frac", inlierFrac, 1), ("rms_m", rmsM, nil), ("observe", observe, nil),
        ]
        for (name, pair, high) in limits {
            for value in [pair.mid360, pair.jt128] {
                if let value, !(value.isFinite && value >= 0 && value <= (high ?? .greatestFiniteMagnitude)) {
                    throw RoomMemoryCDRError.invalid("\(context): \(name) \(value) must be finite and in [0, \(high.map { "\($0)" } ?? "inf")], or NaN")
                }
            }
        }
        if let tiltDeg, !(tiltDeg.isFinite && (0...180).contains(tiltDeg)) {
            throw RoomMemoryCDRError.invalid("\(context): tilt_deg \(tiltDeg) must be finite and in [0, 180], or NaN")
        }
        if let movedAt {
            guard movedAt != RoomMemory.Time(sec: 0, nanosec: 0) else {
                throw RoomMemoryCDRError.invalid("\(context): moved_at {0, 0} is the none sentinel; use a nil moved_at")
            }
            try RoomCDR.requireValid(movedAt, "\(context): moved_at")
        }
    }
}

extension RoomMemory.ImuSample {
    /// Encodes the sample as CDR little endian with identity orientation and every covariance 0
    /// (tests and replay tools), including the encapsulation header.
    ///
    /// - Throws: `RoomMemoryCDRError.invalid` when a rate or acceleration is not finite.
    public func encodedCDR() throws -> Data {
        try validate()
        var writer = RoomCDRWriter()
        writer.time(stamp)
        writer.string(frameId)
        for value in [0, 0, 0, 1] as [Double] { writer.f64(value) }
        let zeros = [Double](repeating: 0, count: 9)
        for vector in [nil, angularVelocity, nil, linearAcceleration, nil] as [SIMD3<Double>?] {
            for value in vector.map { [$0.x, $0.y, $0.z] } ?? zeros { writer.f64(value) }
        }
        return writer.data
    }

    /// Decodes a CDR `sensor_msgs/Imu`: `std_msgs/Header`, orientation and its covariance, angular
    /// velocity and its covariance, linear acceleration and its covariance.
    ///
    /// Accepts either byte order. The input may be a slice.
    ///
    /// - Throws: `RoomMemoryCDRError` on truncated input, or non-finite rates or accelerations.
    public static func decode(cdr data: Data) throws -> RoomMemory.ImuSample {
        try RoomCDR.decode(data, message: "Imu") { reader in
            let stamp = try reader.time("header.stamp")
            let frameId = try reader.string("header.frame_id")
            for index in 0..<(4 + 9) { _ = try reader.f64("orientation[\(index)]") }
            func vector(_ name: String) throws -> SIMD3<Double> {
                let value = SIMD3(try reader.f64("\(name).x"), try reader.f64("\(name).y"), try reader.f64("\(name).z"))
                for index in 0..<9 { _ = try reader.f64("\(name)_covariance[\(index)]") }
                return value
            }
            let angular = try vector("angular_velocity")
            let linear = try vector("linear_acceleration")
            let sample = RoomMemory.ImuSample(stamp: stamp, frameId: frameId, angularVelocity: angular, linearAcceleration: linear)
            try sample.validate()
            return sample
        }
    }

    func validate() throws {
        try RoomCDR.requireValid(stamp, "Imu: stamp")
        for (name, value) in [("angular_velocity", angularVelocity), ("linear_acceleration", linearAcceleration)] {
            guard value.x.isFinite, value.y.isFinite, value.z.isFinite else {
                throw RoomMemoryCDRError.invalid("Imu: \(name) \(value) is not finite")
            }
        }
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

    /// Nil (legacy layout) passes; a present generation must not be `GENERATION_UNKNOWN`.
    static func requireGeneration(_ value: UInt64?, _ what: String) throws {
        if value == RoomMemory.generationUnknown {
            throw RoomMemoryCDRError.invalid("\(what) 0 is GENERATION_UNKNOWN, invalid in a Spark message")
        }
    }

    /// Increasing, non-adjacent ranges, each within `bound` (none when `bound` is nil).
    static func requireCanonical(_ ranges: [ClosedRange<UInt64>], bound: UInt64?, _ what: String) throws {
        for (index, range) in ranges.enumerated() {
            if index > 0, range.lowerBound <= ranges[index - 1].upperBound || range.lowerBound - ranges[index - 1].upperBound == 1 {
                throw RoomMemoryCDRError.invalid("\(what)[\(index)] is not after \(what)[\(index - 1)] with a gap (canonical ranges)")
            }
            guard let bound, range.upperBound <= bound else {
                throw RoomMemoryCDRError.invalid("\(what)[\(index)] ends at \(range.upperBound), past the bound \(bound.map(String.init) ?? "SEQ_NONE")")
            }
        }
    }

    static func requireValid(_ time: RoomMemory.Time, _ what: String) throws {
        guard time.nanosec < 1_000_000_000 else {
            throw RoomMemoryCDRError.invalid("\(what).nanosec \(time.nanosec) is not below 1e9")
        }
    }
}

// MARK: - MapBricks Payload

/// Size, compression and unpacking of the MapBricks brick payload.
enum RoomPayload {
    /// `brick_ijk` bytes per brick: three int16.
    static let coordinateBytes = 6
    /// Uncompressed payload bytes per brick: coordinates plus both masks.
    static let bytesPerBrick = coordinateBytes + 2 * RoomMemory.Brick.maskBytes
    /// Largest uncompressed payload either side accepts: 16 MiB, about 125,000 bricks.
    /// Bounds what a small compressed payload can make the decoder allocate.
    static let maxBytes = 16 << 20

    /// Uncompressed payload bytes for `bricks` bricks and `removed` removed indices.
    static func size(bricks: Int, removed: Int) -> Int {
        bricks * bytesPerBrick + removed * MemoryLayout<UInt32>.size
    }

    static func compress(_ raw: [UInt8], _ compression: RoomMemory.MapBricks.Compression) throws -> [UInt8] {
        let algorithm: NSData.CompressionAlgorithm
        switch compression {
        case .none: return raw
        case .zlib: algorithm = .zlib
        case .lzfse: algorithm = .lzfse
        }
        do {
            return [UInt8](try (Data(raw) as NSData).compressed(using: algorithm) as Data)
        } catch {
            throw RoomMemoryCDRError.invalid("MapBricks: \(compression) compression of \(raw.count) payload bytes failed: \(error)")
        }
    }

    /// Returns the uncompressed payload, which must be exactly `expectedCount` bytes. A compressed
    /// payload must be one complete stream. Bytes after the stream's end are not detected.
    static func decompress(
        _ payload: [UInt8],
        _ compression: RoomMemory.MapBricks.Compression,
        expectedCount: Int,
        _ what: String
    ) throws -> [UInt8] {
        let algorithm: compression_algorithm
        switch compression {
        case .none:
            guard payload.count == expectedCount else {
                throw RoomMemoryCDRError.invalid("\(what) holds \(payload.count) bytes, expected \(expectedCount)")
            }
            return payload
        case .zlib: algorithm = COMPRESSION_ZLIB
        case .lzfse: algorithm = COMPRESSION_LZFSE
        }
        guard !payload.isEmpty, let raw = inflate(payload, algorithm, expectedCount: expectedCount) else {
            throw RoomMemoryCDRError.invalid(
                "\(what): \(payload.count) \(compression) bytes are not one complete stream of \(expectedCount) bytes"
            )
        }
        return raw
    }

    /// Decodes into a buffer one byte larger than expected, so a longer stream is caught without
    /// decoding more than `expectedCount + 1` bytes. Nil unless the stream ends at exactly `expectedCount`.
    private static func inflate(_ payload: [UInt8], _ algorithm: compression_algorithm, expectedCount: Int) -> [UInt8]? {
        let stream = UnsafeMutablePointer<compression_stream>.allocate(capacity: 1)
        defer { stream.deallocate() }
        guard compression_stream_init(stream, COMPRESSION_STREAM_DECODE, algorithm) == COMPRESSION_STATUS_OK else {
            return nil
        }
        defer { compression_stream_destroy(stream) }

        var output = [UInt8](repeating: 0, count: expectedCount + 1)
        let status = payload.withUnsafeBufferPointer { source in
            output.withUnsafeMutableBufferPointer { destination in
                stream.pointee.src_ptr = source.baseAddress!
                stream.pointee.src_size = source.count
                stream.pointee.dst_ptr = destination.baseAddress!
                stream.pointee.dst_size = destination.count
                var status: compression_status
                var progressed: Bool
                repeat {
                    let before = (stream.pointee.src_size, stream.pointee.dst_size)
                    status = compression_stream_process(stream, Int32(bitPattern: COMPRESSION_STREAM_FINALIZE.rawValue))
                    progressed = before != (stream.pointee.src_size, stream.pointee.dst_size)
                } while status == COMPRESSION_STATUS_OK && stream.pointee.dst_size > 0 && progressed
                return status
            }
        }
        guard status == COMPRESSION_STATUS_END, output.count - stream.pointee.dst_size == expectedCount else {
            return nil
        }
        output.removeLast()
        return output
    }

    /// Splits an uncompressed payload of exactly `size(bricks:removed:)` bytes into bricks and removed indices.
    static func unpack(_ raw: [UInt8], brickCount: Int, removedCount: Int) -> ([RoomMemory.Brick], [UInt32]) {
        let maskBytes = RoomMemory.Brick.maskBytes
        let occupiedStart = brickCount * coordinateBytes
        let knownStart = occupiedStart + brickCount * maskBytes
        let removedStart = knownStart + brickCount * maskBytes
        return raw.withUnsafeBytes { bytes in
            func int16(_ offset: Int) -> Int16 {
                Int16(littleEndian: bytes.loadUnaligned(fromByteOffset: offset, as: Int16.self))
            }
            let bricks = (0..<brickCount).map { index in
                let coordinate = index * coordinateBytes
                let mask = index * maskBytes
                return RoomMemory.Brick(
                    coordinate: RoomMemory.BrickCoordinate(i: int16(coordinate), j: int16(coordinate + 2), k: int16(coordinate + 4)),
                    occupied: Array(raw[occupiedStart + mask ..< occupiedStart + mask + maskBytes]),
                    known: Array(raw[knownStart + mask ..< knownStart + mask + maskBytes])
                )
            }
            let removed = (0..<removedCount).map { index in
                UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: removedStart + index * 4, as: UInt32.self))
            }
            return (bricks, removed)
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
    mutating func f64(_ value: Double) { append(value.bitPattern) }
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

    /// `SeqRange[]`: a uint32 count, then two uint64 per range.
    mutating func ranges(_ values: [ClosedRange<UInt64>]) {
        u32(UInt32(values.count))
        for range in values {
            u64(range.lowerBound)
            u64(range.upperBound)
        }
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

    mutating func f64(_ what: @autoclosure () -> String) throws -> Double {
        Double(bitPattern: try load(UInt64.self, what()))
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

    /// `length` raw bytes (a `uint8[N]`), copied.
    mutating func bytes(_ length: Int, _ what: @autoclosure () -> String) throws -> [UInt8] {
        slice(try take(length, what()), length)
    }

    /// `SeqRange[]`. The count is checked against the bytes left before anything is reserved, and
    /// the first element's uint64 does the alignment: an empty list aligns nothing.
    mutating func ranges(_ what: @autoclosure () -> String) throws -> [ClosedRange<UInt64>] {
        let count = Int(try u32(what() + " length"))
        let available = max(0, bytes.count - offset)
        guard count <= available / 16 else {
            throw RoomMemoryCDRError.truncated("\(message) CDR: truncated \(what()) at byte \(offset) (\(count) ranges, have \(available) bytes)")
        }
        var ranges: [ClosedRange<UInt64>] = []
        ranges.reserveCapacity(count)
        for index in 0..<count {
            let first = try u64(what() + "[\(index)].first")
            let last = try u64(what() + "[\(index)].last")
            guard first <= last else {
                throw RoomMemoryCDRError.invalid("\(message): \(what())[\(index)] first \(first) is after last \(last)")
            }
            ranges.append(first...last)
        }
        return ranges
    }

    /// The layout after the last legacy field (`after`): false (legacy) when at most 3 bytes are
    /// left; true (v2) when padding to 8 and then exactly `fields` bytes, and at most 3, are left.
    /// Throws otherwise.
    mutating func isV2(fields: Int, after what: @autoclosure () -> String) throws -> Bool {
        let unread = bytes.count - offset
        if unread <= RoomCDR.maxTrailingPadding { return false }
        let misalignment = (offset - RoomCDR.encapsulationSize) % 8
        let pad = misalignment == 0 ? 0 : 8 - misalignment
        if (fields...(fields + RoomCDR.maxTrailingPadding)).contains(unread - pad) { return true }
        throw RoomMemoryCDRError.invalid(
            "\(message) CDR: \(unread) bytes after \(what()) are neither padding (at most \(RoomCDR.maxTrailingPadding)) nor the \(fields)-byte v2 fields"
        )
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
