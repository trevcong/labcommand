import Foundation

// MARK: - Room Memory Messages

/// Swift mirrors of the `labcommand_msgs` room-memory messages (`ros/labcommand_msgs/msg/`).
///
/// The contract is `docs/ROOM-MEMORY-PROTOCOL.md`. Values that can be missing on the wire
/// (a NaN float, the `0xFFFF` rate sentinel, an empty `mask`) are Optionals here, so a
/// missing value can never be read as a measured zero. `RoomMemoryCDR.swift` holds the codecs.
public enum RoomMemory {

    /// `builtin_interfaces/Time`: seconds and nanoseconds since the Unix epoch.
    public struct Time: Sendable, Hashable, Comparable {
        /// Whole seconds since 1970-01-01T00:00:00Z.
        public var sec: Int32
        /// Nanoseconds past `sec`, `0 ..< 1_000_000_000`.
        public var nanosec: UInt32

        /// Creates a time from its two wire fields.
        public init(sec: Int32, nanosec: UInt32) {
            self.sec = sec
            self.nanosec = nanosec
        }

        /// Orders by `sec`, then `nanosec`.
        public static func < (lhs: Time, rhs: Time) -> Bool {
            (lhs.sec, lhs.nanosec) < (rhs.sec, rhs.nanosec)
        }
    }

    /// The four fields every room-memory message starts with.
    public struct Header: Sendable, Equatable {
        /// Room this message belongs to.
        public var roomId: String
        /// Reference-map epoch the message describes. Bumped on every reference-map change.
        public var mapEpoch: UInt64
        /// Per-topic counter, +1 per message. `0` for a `ChangeEvent.mask`; this client's request count in a `RequestSync`.
        public var seq: UInt64
        /// When the Spark produced the message.
        public var stamp: Time

        /// Creates a header.
        public init(roomId: String, mapEpoch: UInt64, seq: UInt64, stamp: Time) {
            self.roomId = roomId
            self.mapEpoch = mapEpoch
            self.seq = seq
            self.stamp = stamp
        }
    }

    /// `MemoryStatus.STATE_*`, also carried by `Epoch.state`.
    public enum MemoryState: UInt8, Sendable, CaseIterable {
        /// No reference map yet; no change events.
        case building = 0
        /// Reference map held; change events flow.
        case tracking = 1
        /// The rig-pose guard tripped; no change events.
        case rigMoved = 2
        /// Running with reduced input (a sensor or windows missing).
        case degraded = 3
        /// Stopped by the operator.
        case paused = 4
    }

    /// The two sensors, as the `ChangeEvent.sensors` bitmask.
    public struct Sensors: OptionSet, Sendable, Hashable {
        /// The bitmask as sent on the wire.
        public let rawValue: UInt8

        /// Creates a set from its wire bitmask.
        public init(rawValue: UInt8) {
            self.rawValue = rawValue
        }

        /// Livox Mid-360, bit 0. Also index 0 of `MemoryStatus.per_sensor_hz`.
        public static let mid360 = Sensors(rawValue: 1 << 0)
        /// Hesai JT128, bit 1. Also index 1 of `MemoryStatus.per_sensor_hz`.
        public static let jt128 = Sensors(rawValue: 1 << 1)
        /// Every bit v1 defines.
        public static let all: Sensors = [.mid360, .jt128]
    }

    /// What is known about one voxel.
    public enum VoxelEvidence: Sendable, Equatable {
        /// Never observed. Not the same as free.
        case unknown
        /// Observed empty: a ray passed through it.
        case free
        /// Observed occupied: a ray ended in it.
        case occupied
    }

    /// Brick coordinates: brick `(i, j, k)` spans voxels `8i ..< 8i + 8` (and so on) in the room frame.
    public struct BrickCoordinate: Sendable, Hashable, Comparable {
        /// Brick index along x.
        public var i: Int16
        /// Brick index along y.
        public var j: Int16
        /// Brick index along z.
        public var k: Int16

        /// Creates a brick coordinate.
        public init(i: Int16, j: Int16, k: Int16) {
            self.i = i
            self.j = j
            self.k = k
        }

        /// Canonical brick order: by `i`, then `j`, then `k`.
        public static func < (lhs: BrickCoordinate, rhs: BrickCoordinate) -> Bool {
            (lhs.i, lhs.j, lhs.k) < (rhs.i, rhs.j, rhs.k)
        }
    }

    /// One 8×8×8 brick of voxel evidence.
    public struct Brick: Sendable, Equatable {
        /// Voxels along one brick edge.
        public static let dim = 8
        /// Bytes in one brick's `occupied` or `known` bitmask: 512 bits.
        public static let maskBytes = 64

        /// Where the brick sits.
        public var coordinate: BrickCoordinate
        /// `maskBytes` bytes; a set bit means the voxel is occupied. Every set bit is also set in `known`.
        public var occupied: [UInt8]
        /// `maskBytes` bytes; a set bit means the voxel was observed (occupied or free).
        public var known: [UInt8]

        /// Creates a brick from its two bitmasks. The codec checks their lengths.
        public init(coordinate: BrickCoordinate, occupied: [UInt8], known: [UInt8]) {
            self.coordinate = coordinate
            self.occupied = occupied
            self.known = known
        }

        /// Where voxel `(x, y, z)` of a brick lives in its masks: bit `x + 8y + 64z`, least significant bit first.
        ///
        /// - Parameters:
        ///   - x: Local voxel index along x, `0 ..< 8`.
        ///   - y: Local voxel index along y, `0 ..< 8`.
        ///   - z: Local voxel index along z, `0 ..< 8`.
        /// - Returns: The byte index into a mask and the bit within that byte.
        public static func bitPosition(x: Int, y: Int, z: Int) -> (byte: Int, mask: UInt8) {
            let bit = x + dim * y + dim * dim * z
            return (bit >> 3, UInt8(1) << UInt8(bit & 7))
        }

        /// The evidence for local voxel `(x, y, z)`, each `0 ..< 8`.
        public func evidence(x: Int, y: Int, z: Int) -> VoxelEvidence {
            let position = Brick.bitPosition(x: x, y: y, z: z)
            if occupied[position.byte] & position.mask != 0 { return .occupied }
            if known[position.byte] & position.mask != 0 { return .free }
            return .unknown
        }
    }

    /// `labcommand_msgs/MapBricks`: a full snapshot of the room's voxel memory, or a delta since `baseEpoch`.
    ///
    /// A brick that is absent from the map is all UNKNOWN. A brick that is all free is sent, with
    /// `known` full and `occupied` empty.
    public struct MapBricks: Sendable, Equatable {
        /// `MapBricks.KIND_*`.
        public enum Kind: UInt8, Sendable {
            /// The whole map; replaces whatever the receiver holds.
            case snapshot = 0
            /// Changes since `baseEpoch`; applies only to a map at exactly that epoch.
            case delta = 1
        }

        /// `MapBricks.COMPRESSION_*`: how the brick payload travels on the wire.
        public enum Compression: UInt8, Sendable, CaseIterable {
            /// The packed payload as is.
            case none = 0
            /// Raw DEFLATE (RFC 1951, no zlib header or checksum): Foundation `.zlib`, Python `zlib.decompress(payload, -15)`.
            case zlib = 1
            /// Apple LZFSE: Foundation `.lzfse`.
            case lzfse = 2
        }

        /// The common fields.
        public var header: Header
        /// How the encoder compresses the brick payload. A decoded message reports what the sender used.
        public var compression: Compression
        /// Snapshot or delta.
        public var kind: Kind
        /// For a delta, the epoch it applies to; below `header.mapEpoch`. `0` in a snapshot.
        public var baseEpoch: UInt64
        /// Voxel edge length in meters. Fixed per room; 0.05 by default.
        public var voxelSize: Float
        /// Bricks in strictly increasing `coordinate` order. In a delta, each one replaces the brick at its coordinate.
        public var bricks: [Brick]
        /// Delta only: indices into the `baseEpoch` brick list (canonical order) of bricks that became all UNKNOWN.
        /// Strictly increasing.
        public var removedBrickIndices: [UInt32]

        /// Creates a map message. The payload is zlib-compressed unless `compression` says otherwise.
        public init(
            header: Header,
            compression: Compression = .zlib,
            kind: Kind,
            baseEpoch: UInt64,
            voxelSize: Float,
            bricks: [Brick],
            removedBrickIndices: [UInt32] = []
        ) {
            self.header = header
            self.compression = compression
            self.kind = kind
            self.baseEpoch = baseEpoch
            self.voxelSize = voxelSize
            self.bricks = bricks
            self.removedBrickIndices = removedBrickIndices
        }
    }

    /// `labcommand_msgs/ChangeEvent`: one lifecycle transition of one change.
    public struct ChangeEvent: Sendable, Equatable {
        /// `ChangeEvent.KIND_*`.
        public enum Kind: UInt8, Sendable, CaseIterable {
            /// Occupied now where the reference has free evidence.
            case appeared = 0
            /// Free evidence now where the reference was occupied.
            case disappeared = 1
            /// A disappeared region paired with an appeared one.
            case moved = 2
        }

        /// `ChangeEvent.STATE_*`.
        public enum State: UInt8, Sendable, CaseIterable {
            /// Seen, not yet persistent for T_commit.
            case candidate = 0
            /// Persisted for T_commit; shown as a change.
            case confirmed = 1
            /// Folded into the reference map. Terminal.
            case accepted = 2
            /// Undone, by the scene or by the operator. Terminal; never shown as a change.
            case reverted = 3
            /// Dropped before confirming. Terminal; never shown as a change.
            case expired = 4
        }

        /// An axis-aligned box in the room frame, in meters.
        public struct Box: Sendable, Equatable {
            /// Minimum corner.
            public var min: SIMD3<Float>
            /// Maximum corner; each component ≥ the matching one in `min`.
            public var max: SIMD3<Float>

            /// Creates a box from its corners.
            public init(min: SIMD3<Float>, max: SIMD3<Float>) {
                self.min = min
                self.max = max
            }
        }

        /// The common fields.
        public var header: Header
        /// Unique per room; every transition of one change repeats it.
        public var eventId: UInt64
        /// Appeared, disappeared, or moved.
        public var kind: Kind
        /// Lifecycle state this message reports.
        public var state: State
        /// Region centroid, meters, room frame.
        public var centroid: SIMD3<Float>
        /// Region bounds, meters, room frame.
        public var aabb: Box
        /// Where a moved region came from, meters, room frame. Non-nil exactly when `kind == .moved`.
        public var fromCentroid: SIMD3<Float>?
        /// Voxels in the region.
        public var voxelCount: UInt32
        /// Region volume in cubic meters (`voxelCount × voxelSize³`).
        public var volumeM3: Float
        /// Fraction `0...1` of the region's voxels with evidence. Nil when not computed.
        public var evidence: Float?
        /// Calibrated false-alarm level `0...1`. Nil means uncalibrated, never zero.
        public var alpha: Float?
        /// Which sensors observed the region.
        public var sensors: Sensors
        /// When the region was first seen.
        public var firstSeen: Time
        /// When the region was last seen; not before `firstSeen`.
        public var lastSeen: Time
        /// The region's voxels as a snapshot-kind map: `occupied` = in the region. Nil when not sent.
        public var mask: MapBricks?

        /// Creates an event message.
        public init(
            header: Header,
            eventId: UInt64,
            kind: Kind,
            state: State,
            centroid: SIMD3<Float>,
            aabb: Box,
            fromCentroid: SIMD3<Float>?,
            voxelCount: UInt32,
            volumeM3: Float,
            evidence: Float?,
            alpha: Float?,
            sensors: Sensors,
            firstSeen: Time,
            lastSeen: Time,
            mask: MapBricks?
        ) {
            self.header = header
            self.eventId = eventId
            self.kind = kind
            self.state = state
            self.centroid = centroid
            self.aabb = aabb
            self.fromCentroid = fromCentroid
            self.voxelCount = voxelCount
            self.volumeM3 = volumeM3
            self.evidence = evidence
            self.alpha = alpha
            self.sensors = sensors
            self.firstSeen = firstSeen
            self.lastSeen = lastSeen
            self.mask = mask
        }
    }

    /// `labcommand_msgs/MemoryStatus`: room-memory health at 1 Hz.
    public struct MemoryStatus: Sendable, Equatable {
        /// The common fields.
        public var header: Header
        /// Memory state.
        public var state: MemoryState
        /// Bricks in the reference map.
        public var brickCount: UInt32
        /// Events in CANDIDATE or CONFIRMED.
        public var openEventCount: UInt32
        /// Fraction `0...1` of the room volume observed. Nil while the room volume is unknown.
        public var coverageKnownFraction: Float?
        /// Last rig-pose guard residual in meters. Nil when the guard has not run.
        public var poseResidualM: Float?
        /// Accumulation window in seconds. Nil when the node did not report it.
        public var windowS: Float?
        /// Mid-360 frames received per second, rounded. Nil when not measured; `0` is a measured zero.
        public var mid360Hz: UInt16?
        /// JT128 frames received per second, rounded. Nil when not measured; `0` is a measured zero.
        public var jt128Hz: UInt16?
        /// Windows dropped since the node started.
        public var droppedWindows: UInt32

        /// Creates a status message.
        public init(
            header: Header,
            state: MemoryState,
            brickCount: UInt32,
            openEventCount: UInt32,
            coverageKnownFraction: Float?,
            poseResidualM: Float?,
            windowS: Float?,
            mid360Hz: UInt16?,
            jt128Hz: UInt16?,
            droppedWindows: UInt32
        ) {
            self.header = header
            self.state = state
            self.brickCount = brickCount
            self.openEventCount = openEventCount
            self.coverageKnownFraction = coverageKnownFraction
            self.poseResidualM = poseResidualM
            self.windowS = windowS
            self.mid360Hz = mid360Hz
            self.jt128Hz = jt128Hz
            self.droppedWindows = droppedWindows
        }
    }

    /// `labcommand_msgs/Epoch`: a timeline tick at 0.2 Hz. `header.mapEpoch` is the epoch id.
    public struct Epoch: Sendable, Equatable {
        /// The common fields.
        public var header: Header
        /// APPEARED events that reached CONFIRMED since the room was created.
        public var appearedCount: UInt32
        /// DISAPPEARED events that reached CONFIRMED since the room was created.
        public var disappearedCount: UInt32
        /// MOVED events that reached CONFIRMED since the room was created.
        public var movedCount: UInt32
        /// As `MemoryStatus.coverageKnownFraction`.
        public var coverageKnownFraction: Float?
        /// Memory state at this tick.
        public var state: MemoryState

        /// Creates a timeline tick.
        public init(
            header: Header,
            appearedCount: UInt32,
            disappearedCount: UInt32,
            movedCount: UInt32,
            coverageKnownFraction: Float?,
            state: MemoryState
        ) {
            self.header = header
            self.appearedCount = appearedCount
            self.disappearedCount = disappearedCount
            self.movedCount = movedCount
            self.coverageKnownFraction = coverageKnownFraction
            self.state = state
        }
    }

    /// `labcommand_msgs/RequestSync`: the Mac asks the Spark to resend what it missed on one topic.
    ///
    /// `header.mapEpoch` repeats `since.epoch` (`RequestSync.SINCE_NONE` when `since` is nil), and
    /// `header.seq` counts this client's requests. The Spark answers on the topic's sync topic with
    /// the missed messages or a snapshot (`docs/ROOM-MEMORY-PROTOCOL.md` §5).
    public struct RequestSync: Sendable, Equatable {
        /// `RequestSync.TOPIC_*`: which topic the request is about.
        public enum Topic: UInt8, Sendable, CaseIterable {
            /// `/room_memory/map_bricks`.
            case mapBricks = 0
            /// `/room_memory/events`.
            case events = 1
        }

        /// The last message the Mac applied on the topic.
        public struct Position: Sendable, Equatable {
            /// That message's `map_epoch`. For `mapBricks` this is the epoch the mirror holds.
            public var epoch: UInt64
            /// That message's `seq`.
            public var seq: UInt64

            /// Creates a position. Neither value may be `RequestSync.sinceNone`.
            public init(epoch: UInt64, seq: UInt64) {
                self.epoch = epoch
                self.seq = seq
            }
        }

        /// `RequestSync.SINCE_NONE`: the wire value of `since_epoch` and `since_seq` when the Mac holds nothing.
        public static let sinceNone = UInt64.max

        /// The common fields.
        public var header: Header
        /// The last message applied on `topic`. Nil when the Mac holds nothing for it, which asks for everything.
        public var since: Position?
        /// The topic to resynchronize.
        public var topic: Topic

        /// Creates a request.
        public init(header: Header, since: Position?, topic: Topic) {
            self.header = header
            self.since = since
            self.topic = topic
        }
    }
}
