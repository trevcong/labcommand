import Foundation
import simd

/// How this process should open the room page. `LABCOMMAND_LIVE_ROOM` is the switch.
/// Fixture mode never constructs a `LiveRoomClient`.
public enum LiveRoomLaunch: Equatable, Sendable {
    /// A live socket. `opensRoom` is true only for the value `live` (the gate launch).
    /// That flag skips probes and the lidar client. It is not the Room tab.
    /// An unset variable stays `.live(opensRoom: false)`: probes and the lidar client stay on.
    case live(opensRoom: Bool)
    case fixture(LiveRoomFixture.Mode)

    public var startsClient: Bool {
        if case .live = self { return true }
        return false
    }

    /// The gate launch and every fixture. Not the Room tab: setting this skips probes.
    public var opensRoom: Bool {
        switch self {
        case .live(let opensRoom): return opensRoom
        case .fixture: return true
        }
    }

    /// Skip refresh, SSH, the rig poll, and Tailscale. An unset launch does not.
    public var skipsProbes: Bool {
        switch self {
        case .live(let opensRoom): return opensRoom
        case .fixture: return true
        }
    }

    /// The lidar client stays dark for the gate launch and for every fixture.
    public var lidarAvailable: Bool {
        if case .live(let opensRoom) = self { return !opensRoom }
        return false
    }

    public var fixtureMode: LiveRoomFixture.Mode? {
        if case .fixture(let mode) = self { return mode }
        return nil
    }

    public static func parse(_ environment: [String: String] = ProcessInfo.processInfo.environment) -> LiveRoomLaunch {
        guard let raw = environment["LABCOMMAND_LIVE_ROOM"] else {
            return .live(opensRoom: false)
        }
        if raw == "live" { return .live(opensRoom: true) }
        if let mode = LiveRoomFixture.Mode(rawValue: raw) { return .fixture(mode) }
        return .fixture(.unavailable)
    }
}

public enum LiveRoomFixtureError: Error, Equatable, CustomStringConvertible {
    case missing(String)

    public var description: String {
        switch self {
        case .missing(let name): return "room golden \(name).hex was not in the bundle or the fixture root"
        }
    }
}

/// Offline room. Decodes the hand goldens (v2, one publisher generation) and drives
/// `RoomMemoryMirror`. It does not open a socket. `lost`, `restart` and `legacy` show the
/// history and identity states the screenshots check: lost ranges, a restart banner with the
/// earlier run's open events, and an old Spark node.
public struct LiveRoomFixture: Equatable, Sendable {
    public enum Mode: String, Equatable, Sendable, CaseIterable {
        case map, status, stale, lost, restart, legacy, unavailable
    }

    /// The publisher generation of every v2 hand golden.
    public static let generation: UInt64 = 1_760_400_000_000
    /// The run before it in `restart` mode: an hour earlier.
    static let earlierGeneration: UInt64 = generation - 3_600_000

    public var mode: Mode
    public var mirror: RoomMemoryMirror
    public var connection: LiveRoomConnection
    private var statusSeq: UInt64
    private var statusGeneration: UInt64?
    private var statusEventsSeq: UInt64?

    public static func load(mode: Mode, root: URL?, now: TimeInterval) throws -> LiveRoomFixture {
        var fixture = LiveRoomFixture(mode: mode, mirror: RoomMemoryMirror(tokenSeed: 0), connection: connected, statusSeq: 1,
                                      statusGeneration: generation, statusEventsSeq: 5)
        switch mode {
        case .unavailable:
            fixture.connection = .off
            return fixture
        case .status:
            _ = fixture.mirror.connectionOpened(at: now, canRequest: false)
            _ = fixture.mirror.receive(topic: .status, payload: try goldenBytes(name: "memory_status_v2_le", root: root), at: now)
            return fixture
        case .map, .stale, .lost:
            let at = mode == .stale ? now - 4 : now
            _ = fixture.mirror.connectionOpened(at: at, canRequest: false)
            fixture.feed(.mapBricks, try snapshot(root: root, generation: generation), at: at)
            if mode != .lost {
                // The run's first three transitions: an earlier candidate of the moved box, and a speck that expired.
                for message in earlierTransitions(generation: generation) {
                    fixture.feed(.events, try message.encodedCDR(), at: at)
                }
            }
            fixture.feed(.events, try goldenBytes(name: "change_event_moved_mask_v2_le", root: root), at: at)
            fixture.feed(.events, try goldenBytes(name: "change_event_appeared_v2_le", root: root), at: at)
            fixture.feed(.events, try disappearedEvent(generation: generation).encodedCDR(), at: at)
            if mode == .lost {
                // Another Mac's fallback: seqs 0-2 fell out of the Spark's log before anyone fetched them.
                fixture.feed(.syncEnd, try lostSyncEnd().encodedCDR(), at: at)
            }
            fixture.feed(.status, try trackingStatus(seq: 1, stamp: RoomMemory.Time(sec: 1, nanosec: 0), eventsSeq: 5,
                                                     generation: generation).encodedCDR(), at: at)
            if mode == .stale {
                _ = fixture.mirror.tick(at: now)
            }
            return fixture
        case .restart:
            _ = fixture.mirror.connectionOpened(at: now - 1, canRequest: false)
            fixture.feed(.mapBricks, try snapshot(root: root, generation: earlierGeneration), at: now - 1)
            fixture.feed(.events, try retagged(goldenBytes(name: "change_event_moved_mask_v2_le", root: root), earlierGeneration), at: now - 1)
            fixture.feed(.status, try trackingStatus(seq: 40, stamp: RoomMemory.Time(sec: 1, nanosec: 0), eventsSeq: 3,
                                                     generation: earlierGeneration).encodedCDR(), at: now - 1)
            // The node restarted: its status counts from 0 again under a higher generation, no events yet.
            fixture.statusEventsSeq = nil
            fixture.statusSeq = 0
            fixture.feed(.mapBricks, try snapshot(root: root, generation: generation, seq: 0), at: now)
            fixture.feed(.status, try trackingStatus(seq: 0, stamp: RoomMemory.Time(sec: 2, nanosec: 0), eventsSeq: nil,
                                                     generation: generation).encodedCDR(), at: now)
            return fixture
        case .legacy:
            fixture.statusGeneration = nil
            fixture.statusEventsSeq = nil
            _ = fixture.mirror.connectionOpened(at: now, canRequest: false)
            fixture.feed(.mapBricks, try goldenBytes(name: "map_bricks_snapshot_zlib_le", root: root), at: now)
            fixture.feed(.events, try goldenBytes(name: "change_event_moved_mask_le", root: root), at: now)
            fixture.feed(.events, try goldenBytes(name: "change_event_appeared_le", root: root), at: now)
            fixture.feed(.status, try trackingStatus(seq: 1, stamp: RoomMemory.Time(sec: 1, nanosec: 0), eventsSeq: nil,
                                                     generation: nil).encodedCDR(), at: now)
            return fixture
        }
    }

    private mutating func feed(_ topic: LiveRoomTopic, _ payload: Data, at now: TimeInterval) {
        _ = mirror.receive(topic: topic, payload: payload, at: now)
    }

    /// The modes whose status the app refreshes, so they read as a live room.
    public static func keepsFresh(_ mode: Mode) -> Bool {
        [.map, .lost, .restart, .legacy].contains(mode)
    }

    /// A new status seq so the 3 s freshness window stays open (every mode with a live map).
    public func refreshed(at now: TimeInterval) -> LiveRoomFixture {
        guard Self.keepsFresh(mode) else { return self }
        var copy = self
        let seq = statusSeq &+ 1
        let stamp = RoomMemory.Time(sec: 1, nanosec: UInt32(seq % 1_000_000_000))
        guard let payload = try? Self.trackingStatus(seq: seq, stamp: stamp, eventsSeq: statusEventsSeq,
                                                     generation: statusGeneration).encodedCDR() else {
            return self
        }
        _ = copy.mirror.receive(topic: .status, payload: payload, at: now)
        copy.statusSeq = seq
        return copy
    }

    /// Hex bytes. Bundle resource, then `root/Tests/Fixtures/room/golden/hand/<name>.hex`.
    public static func goldenBytes(name: String, root: URL?) throws -> Data {
        let file = "\(name).hex"
        if let bundled = Bundle.main.url(forResource: name, withExtension: "hex", subdirectory: "fixtures/room/golden/hand"),
           let text = try? String(contentsOf: bundled, encoding: .utf8) {
            return parseHex(text)
        }
        if let root {
            let url = root.appendingPathComponent("Tests/Fixtures/room/golden/hand").appendingPathComponent(file)
            if let text = try? String(contentsOf: url, encoding: .utf8) {
                return parseHex(text)
            }
        }
        throw LiveRoomFixtureError.missing(name)
    }

    static func parseHex(_ text: String) -> Data {
        var bytes: [UInt8] = []
        for line in text.split(separator: "\n", omittingEmptySubsequences: false) {
            let code = line.prefix(while: { $0 != "#" })
            for token in code.split(separator: " ") {
                guard let byte = UInt8(token, radix: 16) else { continue }
                bytes.append(byte)
            }
        }
        return Data(bytes)
    }

    private static let connected = LiveRoomConnection.connected(
        LiveRoomServer(
            name: "fixture",
            capabilities: [],
            canPublish: false,
            memoryTopics: ["/room_memory/map_bricks", "/room_memory/events", "/room_memory/status"]
        )
    )

    /// Tracking status for room `lab` at epoch 43, coverage left unmeasured.
    static func trackingStatus(seq: UInt64, stamp: RoomMemory.Time, eventsSeq: UInt64? = nil,
                               generation: UInt64? = LiveRoomFixture.generation) -> RoomMemory.MemoryStatus {
        RoomMemory.MemoryStatus(
            header: RoomMemory.Header(roomId: "lab", mapEpoch: 43, seq: seq, stamp: stamp),
            state: .tracking,
            brickCount: 2,
            openEventCount: 3,
            coverageKnownFraction: nil,
            poseResidualM: nil,
            windowS: nil,
            mid360Hz: nil,
            jt128Hz: nil,
            droppedWindows: 0,
            eventsSeq: eventsSeq,
            publisherGeneration: generation
        )
    }

    /// The zlib snapshot golden's bricks as a v2 snapshot of `generation` (there is no v2 snapshot golden).
    static func snapshot(root: URL?, generation: UInt64, seq: UInt64? = nil) throws -> Data {
        var message = try RoomMemory.MapBricks.decode(cdr: goldenBytes(name: "map_bricks_snapshot_zlib_le", root: root))
        message.publisherGeneration = generation
        if let seq { message.header.seq = seq }
        return try message.encodedCDR()
    }

    /// A v2 event golden re-stamped with another generation (the mask follows).
    static func retagged(_ bytes: Data, _ generation: UInt64) throws -> Data {
        var event = try RoomMemory.ChangeEvent.decode(cdr: bytes)
        event.publisherGeneration = generation
        event.mask?.publisherGeneration = generation
        return try event.encodedCDR()
    }

    /// Seqs 0-2: the moved box's CANDIDATE (event 9), and a speck (event 12) that appeared and expired.
    static func earlierTransitions(generation: UInt64) -> [RoomMemory.ChangeEvent] {
        let stamp = RoomMemory.Time(sec: 1_760_400_002, nanosec: 0)
        func event(seq: UInt64, id: UInt64, kind: RoomMemory.ChangeEvent.Kind, state: RoomMemory.ChangeEvent.State,
                   centroid: SIMD3<Float>, from: SIMD3<Float>?) -> RoomMemory.ChangeEvent {
            RoomMemory.ChangeEvent(
                header: RoomMemory.Header(roomId: "lab", mapEpoch: 43, seq: seq, stamp: stamp),
                eventId: id,
                kind: kind,
                state: state,
                centroid: centroid,
                aabb: RoomMemory.ChangeEvent.Box(min: centroid - 0.125, max: centroid + 0.125),
                fromCentroid: from,
                voxelCount: 8,
                volumeM3: 0.001,
                evidence: nil,
                alpha: nil,
                sensors: .jt128,
                firstSeen: stamp,
                lastSeen: stamp,
                mask: nil,
                publisherGeneration: generation
            )
        }
        return [
            event(seq: 0, id: 9, kind: .moved, state: .candidate, centroid: SIMD3(1.5, -0.25, 0.125), from: SIMD3(0.5, -0.25, 0.125)),
            event(seq: 1, id: 12, kind: .appeared, state: .candidate, centroid: SIMD3(-1, 2, 0.25), from: nil),
            event(seq: 2, id: 12, kind: .appeared, state: .expired, centroid: SIMD3(-1, 2, 0.25), from: nil),
        ]
    }

    /// An events fallback's manifest: 3-5 sent, 0-2 no longer held.
    static func lostSyncEnd() -> RoomMemory.SyncEnd {
        RoomMemory.SyncEnd(
            header: RoomMemory.Header(roomId: "lab", mapEpoch: 43, seq: 0, stamp: RoomMemory.Time(sec: 1_760_400_012, nanosec: 0)),
            publisherGeneration: generation,
            token: RoomMemory.RequestToken(bytes: [0x3c, 0x5f, 0x10, 0x8d, 0x22, 0x91, 0x4e, 0x07, 0x9a, 0x61, 0x0b, 0xd2, 0x37, 0xe4, 0x58, 0xa6])!,
            topic: .events,
            bound: 5,
            delivered: [3...5],
            unavailable: [0...2]
        )
    }

    /// Confirmed disappeared box, clear of the two hand-golden boxes. Not a new hex file.
    static func disappearedEvent(generation: UInt64? = LiveRoomFixture.generation) -> RoomMemory.ChangeEvent {
        let stamp = RoomMemory.Time(sec: 1_760_400_012, nanosec: 0)
        return RoomMemory.ChangeEvent(
            header: RoomMemory.Header(roomId: "lab", mapEpoch: 43, seq: 5, stamp: stamp),
            eventId: 11,
            kind: .disappeared,
            state: .confirmed,
            centroid: SIMD3(3.25, 0.25, 0.25),
            aabb: RoomMemory.ChangeEvent.Box(min: SIMD3(3, 0, 0), max: SIMD3(3.5, 0.5, 0.5)),
            fromCentroid: nil,
            voxelCount: 8,
            volumeM3: 0.125,
            evidence: nil,
            alpha: nil,
            sensors: .mid360,
            firstSeen: stamp,
            lastSeen: stamp,
            mask: nil,
            publisherGeneration: generation
        )
    }
}

public enum LiveRoomPip: String, Equatable, Sendable {
    case good, warn, bad, idle

    public var pulses: Bool { self == .good }
}

public enum LiveRoomTone: String, Equatable, Sendable {
    case warn, bad, dim
}

public struct LiveRoomBanner: Equatable, Sendable {
    public var text: String
    public var tone: LiveRoomTone

    public init(text: String, tone: LiveRoomTone) {
        self.text = text
        self.tone = tone
    }
}

public struct LiveRoomEventRow: Equatable, Sendable, Identifiable {
    public var id: String
    public var kind: RoomMemory.ChangeEvent.Kind
    public var line1: String
    public var line2: String
    public var history: Bool
}

public struct LiveRoomArrow: Equatable, Sendable {
    public var tail: SIMD3<Float>
    public var tip: SIMD3<Float>
    public var left: SIMD3<Float>
    public var right: SIMD3<Float>

    public var points: [SIMD3<Float>] { [tail, tip, left, right] }
}

public struct LiveRoomMarker: Equatable, Sendable, Identifiable {
    public var id: String
    public var kind: RoomMemory.ChangeEvent.Kind
    public var corners: [SIMD3<Float>]
    public var dashed: Bool
    public var fillAlpha: Float
    public var onTop: Bool
    public var countsAsChange: Bool
    public var countsAsCandidate: Bool
    public var arrow: LiveRoomArrow?
    public var centroid: SIMD3<Float>
    public var color: SIMD3<Float>
}

/// One frame of the Room page. Pure: connection, mirror, the request-path flag, and the clock.
public struct LiveRoomScreen: Equatable, Sendable {
    public var pip: LiveRoomPip
    public var banner: LiveRoomBanner?
    /// Why the socket is not up, when that is more specific than the banner. Nil while the link is up.
    public var connectionDetail: String?
    public var statusLine: String
    /// Centered copy that replaces the voxels. Nil draws the map.
    public var canvasSentence: String?
    public var brickOpacity: Float
    public var markers: [LiveRoomMarker]
    /// "Spark node restarted · …" while continuity is broken. Shown under `banner`.
    public var restartBanner: LiveRoomBanner?
    /// The event history line from `EventHistory`; nil before any witness.
    public var historyLine: String?
    public var standaloneResync: String?
    public var caption: String
    public var openRows: [LiveRoomEventRow]
    public var historyRows: [LiveRoomEventRow]
    /// Open events of earlier runs. The row says the latest status is unknown. Never drawn on the map.
    public var earlierRows: [LiveRoomEventRow]
    /// The column prints `caption` in place of the event sections.
    public var columnIsUnavailable: Bool

    public static func make(
        mirror: RoomMemoryMirror,
        connection: LiveRoomConnection,
        requestPathExists: Bool,
        started: Bool,
        now: TimeInterval,
        kinds: Set<RoomMemory.ChangeEvent.Kind> = Set(RoomMemory.ChangeEvent.Kind.allCases),
        selection: String? = nil,
        timeZone: TimeZone = .current,
        endpointURL: String? = nil,
        endpointBlock: String? = nil
    ) -> LiveRoomScreen {
        var canvas = roomCanvasSentence(mirror: mirror, connection: connection, started: started)
        let connectionDetail = RoomConnectionFailure.line(
            connection: connection,
            urlText: endpointURL,
            blocked: endpointBlock
        )
        if let endpointBlock, !endpointBlock.isEmpty, canvas == "Room memory unavailable" {
            canvas = nil
        }
        var banner = baseBanner(mirror: mirror, connection: connection, started: started, now: now)
        let history = formatHistory(mirror: mirror, requestPathExists: requestPathExists)
        let phrase = resyncPhrase(mirror: mirror, requestPathExists: requestPathExists)
        var standalone: String?
        if let phrase {
            if banner?.text.hasPrefix("resyncing · as of epoch") == true {
                banner?.text += " · " + phrase
            } else {
                standalone = phrase
            }
        }
        let markers = LiveRoomMarkers.draw(mirror: mirror, kinds: kinds, selection: selection)
        let outdated = !mirror.eventHistory.caughtUp
        let openRows = rows(from: sortedOpen(mirror.openEvents), mirror: mirror, history: false, outdated: outdated, kinds: kinds, timeZone: timeZone)
        let historySource = Array(mirror.events.filter { isTerminal($0.event.state) }.reversed())
        let historyRows = rows(from: historySource, mirror: mirror, history: true, outdated: false, kinds: kinds, timeZone: timeZone)
        let earlierRows = rows(from: sortedOpen(mirror.archivedOpenEvents.map(\.entry)), mirror: mirror, history: false,
                               outdated: false, earlier: true, kinds: kinds, timeZone: timeZone)
        let unavailable = columnUnavailable(mirror: mirror, connection: connection)
        return LiveRoomScreen(
            pip: roomPip(mirror: mirror, connection: connection),
            banner: banner,
            connectionDetail: connectionDetail,
            statusLine: statusLine(mirror: mirror, connection: connection, now: now),
            canvasSentence: canvas,
            brickOpacity: roomBrickOpacity(mirror: mirror, drawingMap: canvas == nil),
            markers: markers,
            restartBanner: restartNotice(mirror),
            historyLine: history,
            standaloneResync: standalone,
            caption: roomCaption(mirror: mirror, markers: markers, unavailable: unavailable),
            openRows: openRows,
            historyRows: historyRows,
            earlierRows: earlierRows,
            columnIsUnavailable: unavailable
        )
    }

    public static func statusLine(mirror: RoomMemoryMirror, connection: LiveRoomConnection, now: TimeInterval) -> String {
        var parts: [String] = []
        var showState = isConnected(connection)
        if !isConnected(connection) {
            parts.append("offline")
        } else if mirror.status != nil, !mirror.isStatusFresh {
            parts.append("STALE")
            parts.append("\(ageSeconds(mirror: mirror, now: now))s")
            showState = false
        }
        let epoch = mirror.mapEpoch ?? mirror.status?.mapEpoch
        parts.append(epoch.map { "epoch \($0)" } ?? "epoch —")
        if let generation = mirror.activeGeneration {
            parts.append(runTag(generation))
        }
        if let coverage = mirror.status?.coverageKnownFraction {
            parts.append("coverage \(oneDecimal(coverage * 100))%")
        } else {
            parts.append("coverage —")
        }
        if let events = mirror.status?.eventsSeq {
            parts.append("events \(events)")
        } else {
            parts.append("events —")
        }
        if showState, mirror.isStatusFresh, let state = mirror.status?.state {
            parts.append(stateWord(state))
        }
        if let size = mirror.voxelSize {
            parts.append("cell \(oneDecimal(size * 100)) cm")
        }
        return parts.joined(separator: " · ")
    }

    /// `run …1234`: the last 4 digits of the publisher generation.
    public static func runTag(_ generation: UInt64) -> String {
        "run …" + String(String(generation).suffix(4))
    }

    static func ageSeconds(mirror: RoomMemoryMirror, now: TimeInterval) -> Int {
        guard let received = mirror.status?.receivedAt else { return 0 }
        return max(0, Int(now - received))
    }
}

public enum LiveRoomVoxels {
    public struct Map: Equatable, Sendable {
        public var points: [SIMD3<Float>]
        public var frames: [[SIMD3<Float>]]
        public var mapRevision: Int
        public var voxelSize: Float?

        public static let empty = Map(points: [], frames: [], mapRevision: 0, voxelSize: nil)
    }

    /// Occupied centers and the 8 corners of any brick that still has an unknown cell.
    public static func extract(_ mirror: RoomMemoryMirror) -> Map {
        guard let size = mirror.voxelSize else {
            return Map(points: [], frames: [], mapRevision: mirror.mapRevision, voxelSize: nil)
        }
        var points: [SIMD3<Float>] = []
        var frames: [[SIMD3<Float>]] = []
        for brick in mirror.bricks.values {
            var occupied: [SIMD3<Float>] = []
            var unknown = false
            for z in 0..<RoomMemory.Brick.dim {
                for y in 0..<RoomMemory.Brick.dim {
                    for x in 0..<RoomMemory.Brick.dim {
                        switch brick.evidence(x: x, y: y, z: z) {
                        case .occupied:
                            occupied.append(center(brick: brick.coordinate, x: x, y: y, z: z, size: size))
                        case .unknown:
                            unknown = true
                        case .free:
                            break
                        }
                    }
                }
            }
            points.append(contentsOf: occupied)
            if unknown {
                frames.append(frameCorners(brick.coordinate, size: size))
            }
        }
        return Map(points: points, frames: frames, mapRevision: mirror.mapRevision, voxelSize: size)
    }

    /// Points the camera frames: occupied centers, drawn brick frames, and drawn markers.
    /// An all-free brick still frames its corners when nothing else is occupied.
    public static func fitPoints(map: Map, markers: [LiveRoomMarker]) -> [SIMD3<Float>] {
        var points = map.points
        for frame in map.frames {
            points.append(contentsOf: frame)
        }
        for marker in markers {
            points.append(contentsOf: marker.corners)
            if let arrow = marker.arrow {
                points.append(contentsOf: arrow.points)
            }
        }
        return points
    }

    static func center(brick: RoomMemory.BrickCoordinate, x: Int, y: Int, z: Int, size: Float) -> SIMD3<Float> {
        let ix = 8 * Int(brick.i) + x
        let iy = 8 * Int(brick.j) + y
        let iz = 8 * Int(brick.k) + z
        return SIMD3((Float(ix) + 0.5) * size, (Float(iy) + 0.5) * size, (Float(iz) + 0.5) * size)
    }

    static func frameCorners(_ brick: RoomMemory.BrickCoordinate, size: Float) -> [SIMD3<Float>] {
        let origin = SIMD3(
            Float(8 * Int(brick.i)) * size,
            Float(8 * Int(brick.j)) * size,
            Float(8 * Int(brick.k)) * size
        )
        let edge = Float(RoomMemory.Brick.dim) * size
        let max = origin + SIMD3(repeating: edge)
        return boxCorners(min: origin, max: max)
    }
}

public enum LiveRoomMarkers {
    public static let fillAlpha: Float = 0.28
    public static let emphasizedFillAlpha: Float = 0.85
    public static let edgePairs: [(Int, Int)] = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    /// Two triangles per face, indexing `corners` the same way as the change overlay.
    public static let boxTriangles: [(Int, Int, Int)] = [
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 5, 1), (0, 4, 5),
        (3, 2, 6), (3, 6, 7),
        (0, 3, 7), (0, 7, 4),
        (1, 5, 6), (1, 6, 2),
    ]

    public static func color(_ kind: RoomMemory.ChangeEvent.Kind) -> SIMD3<Float> {
        switch kind {
        case .appeared: return SIMD3(Float(63) / 255, Float(163) / 255, Float(122) / 255)
        case .disappeared: return SIMD3(Float(196) / 255, Float(90) / 255, Float(90) / 255)
        case .moved: return SIMD3(Float(196) / 255, Float(160) / 255, Float(74) / 255)
        }
    }

    /// Head `min(0.15, length × 0.35)`, wing `head × 0.45`. Nil when the segment has no length.
    public static func arrow(from: SIMD3<Float>, to: SIMD3<Float>) -> LiveRoomArrow? {
        let delta = to - from
        let len = simd_length(delta)
        if len == 0 { return nil }
        let dir = delta / len
        let head = min(Float(0.15), len * 0.35)
        var side = simd_cross(dir, SIMD3<Float>(0, 0, 1))
        var sideLen = simd_length(side)
        if sideLen < 1e-4 {
            side = simd_cross(dir, SIMD3<Float>(0, 1, 0))
            sideLen = simd_length(side)
        }
        if sideLen == 0 { return nil }
        side /= sideLen
        let base = to - dir * head
        let wing = head * 0.45
        return LiveRoomArrow(tail: from, tip: to, left: base + side * wing, right: base - side * wing)
    }

    public static func draw(
        mirror: RoomMemoryMirror,
        kinds: Set<RoomMemory.ChangeEvent.Kind> = Set(RoomMemory.ChangeEvent.Kind.allCases),
        selection: String? = nil
    ) -> [LiveRoomMarker] {
        guard markersAllowed(mirror) else { return [] }
        var markers: [LiveRoomMarker] = []
        for entry in mirror.openEvents where kinds.contains(entry.event.kind) {
            let id = String(entry.event.eventId)
            markers.append(marker(entry, id: id, waiting: false, selected: selection == id))
        }
        for entry in mirror.eventsById.values where kinds.contains(entry.event.kind) && isWaiting(entry, mirror: mirror) {
            let id = "\(entry.seq)-\(entry.event.eventId)"
            markers.append(marker(entry, id: id, waiting: true, selected: selection == id))
        }
        markers.sort { lhs, rhs in
            if lhs.onTop != rhs.onTop { return !lhs.onTop }
            return lhs.id < rhs.id
        }
        return markers
    }

    static func markersAllowed(_ mirror: RoomMemoryMirror) -> Bool {
        guard mirror.isMapCurrent, let state = mirror.status?.state else { return false }
        return state != .building && state != .rigMoved
    }

    static func isWaiting(_ entry: LiveRoomEvent, mirror: RoomMemoryMirror) -> Bool {
        guard entry.event.state == .accepted else { return false }
        guard let epoch = mirror.mapEpoch else { return true }
        return epoch < entry.event.header.mapEpoch
    }
}

public enum LiveRoomSidecar {
    /// Where the app writes the text next to a capture. Nil for an ordinary launch.
    public static func outputPath(_ environment: [String: String] = ProcessInfo.processInfo.environment) -> String? {
        if let explicit = environment["LABCOMMAND_ROOM_SIDECAR"], !explicit.isEmpty {
            return explicit
        }
        guard let mode = environment["LABCOMMAND_LIVE_ROOM"] else { return nil }
        let name: String
        switch mode {
        case "live": name = "live-room-live"
        case "map": name = "live-room-map"
        case "status": name = "live-room-status"
        case "stale": name = "live-room-stale"
        case "lost": name = "live-room-lost"
        case "restart": name = "live-room-restart"
        case "legacy": name = "live-room-legacy"
        default: name = "live-room-unavailable"
        }
        return "build/screenshots/\(name).txt"
    }

    public static func render(
        screen: LiveRoomScreen,
        connection: LiveRoomConnection,
        url: String?,
        canvas: (x: Int, y: Int, w: Int, h: Int)?
    ) -> String {
        var lines: [String] = []
        lines.append("connection \(connectionWord(connection))")
        if case .connected = connection, let url, !url.isEmpty {
            lines.append("url \(url)")
        }
        if let banner = screen.banner {
            lines.append("banner \(banner.text)")
        }
        if let detail = screen.connectionDetail {
            lines.append("failure \(detail)")
        }
        if let restart = screen.restartBanner {
            lines.append("restart \(restart.text)")
        }
        lines.append("status \(screen.statusLine)")
        lines.append("pip \(screen.pip.rawValue)")
        lines.append("markers \(screen.markers.count)")
        lines.append("caption \(screen.caption)")
        if let history = screen.historyLine {
            lines.append("history \(history)")
        }
        if let standalone = screen.standaloneResync {
            lines.append("resync \(standalone)")
        }
        lines.append("rows")
        for row in screen.openRows + screen.historyRows + screen.earlierRows {
            lines.append(row.line1)
            lines.append(row.line2)
        }
        if screen.canvasSentence != "Room memory unavailable", let canvas {
            lines.append("canvas \(canvas.x) \(canvas.y) \(canvas.w) \(canvas.h)")
        }
        return lines.joined(separator: "\n") + "\n"
    }
}

// MARK: - Formatting

private let posix = Locale(identifier: "en_US_POSIX")

private func oneDecimal(_ value: Float) -> String {
    String(format: "%.1f", locale: posix, Double(value))
}

private func twoDecimals(_ value: Float) -> String {
    String(format: "%.2f", locale: posix, Double(value))
}

/// Fraction of the region's cells with evidence. Nil was not computed, which is not zero.
private func boxSeen(_ evidence: Float?) -> String {
    guard let evidence else { return "box not measured" }
    let percent = Int((Double(evidence) * 100).rounded())
    return "\(percent)% of the box was seen"
}

/// Calibrated false-alarm limit of the detector, not this event's chance of being wrong.
/// Nil, exact zero, and a positive level under 1% stay distinct.
private func falseAlarmLimit(_ alpha: Float?) -> String {
    guard let alpha else { return "false-alarm limit not calibrated" }
    if alpha == 0 { return "false-alarm limit 0%" }
    if alpha > 0, alpha < 0.01 { return "false-alarm limit <1%" }
    let percent = (Double(alpha) * 1000).rounded() / 10
    let text = percent == percent.rounded()
        ? String(Int(percent))
        : String(format: "%.1f", locale: posix, percent)
    return "false-alarm limit \(text)%"
}

private func isConnected(_ connection: LiveRoomConnection) -> Bool {
    if case .connected = connection { return true }
    return false
}

/// The line under the Room banner when the socket is not up.
public enum RoomConnectionFailure {
    /// Nil for `.off`, the first `.connecting`, and `.connected`. A `.waiting` reason is included.
    public static func detail(connection: LiveRoomConnection, urlText: String?) -> String? {
        switch connection {
        case .waiting(_, let reason):
            let why = reason.trimmingCharacters(in: .whitespacesAndNewlines)
            let tail = why.isEmpty ? "The connection failed." : why
            return "Cannot reach \(target(urlText)). \(tail)"
        case .connecting(let dial, let attempt) where attempt > 1:
            return "Reconnecting to \(target(dial.absoluteString)) (attempt \(attempt))."
        default:
            return nil
        }
    }

    /// A cleartext block wins. Otherwise the dial failure, when there is one.
    public static func line(connection: LiveRoomConnection, urlText: String?, blocked: String?) -> String? {
        if let blocked, !blocked.isEmpty { return blocked }
        return detail(connection: connection, urlText: urlText)
    }

    private static func target(_ urlText: String?) -> String {
        guard let urlText, !urlText.isEmpty else { return "the room address" }
        guard let url = URL(string: urlText), let host = url.host, !host.isEmpty else { return urlText }
        if let port = url.port { return "\(host):\(port)" }
        return host
    }
}

private func connectionWord(_ connection: LiveRoomConnection) -> String {
    switch connection {
    case .off: return ".off"
    case .connecting: return ".connecting"
    case .connected: return ".connected"
    case .waiting: return ".waiting"
    }
}

private func stateWord(_ state: RoomMemory.MemoryState) -> String {
    switch state {
    case .building: return "Building"
    case .tracking: return "Tracking"
    case .rigMoved: return "Rig moved"
    case .degraded: return "Degraded"
    case .paused: return "Paused"
    }
}

private func kindWord(_ kind: RoomMemory.ChangeEvent.Kind) -> String {
    switch kind {
    case .appeared: return "appeared"
    case .disappeared: return "disappeared"
    case .moved: return "moved"
    }
}

private func stateName(_ state: RoomMemory.ChangeEvent.State) -> String {
    switch state {
    case .candidate: return "checking"
    case .confirmed: return "confirmed"
    case .accepted: return "saved"
    case .reverted: return "change undone"
    case .expired: return "no longer detected"
    }
}

private func isTerminal(_ state: RoomMemory.ChangeEvent.State) -> Bool {
    state == .accepted || state == .reverted || state == .expired
}

/// `{sec: 0, nanosec: 0}` is `time —` in every zone. Anything else is `HH:mm:ss` there.
func liveRoomClock(_ time: RoomMemory.Time, timeZone: TimeZone = .current) -> String {
    if time.sec == 0, time.nanosec == 0 { return "time —" }
    let seconds = TimeInterval(time.sec) + TimeInterval(time.nanosec) / 1_000_000_000
    return LiveViewMode.clock(Date(timeIntervalSince1970: seconds), timeZone: timeZone)
}

private func roomPip(mirror: RoomMemoryMirror, connection: LiveRoomConnection) -> LiveRoomPip {
    if mirror.isStatusFresh, mirror.status?.state == .rigMoved { return .bad }
    if case .connected(let server) = connection,
       !server.memoryTopics.isEmpty,
       mirror.isStatusFresh,
       mirror.isMapCurrent,
       let state = mirror.status?.state,
       state == .tracking || state == .building || state == .degraded {
        return .good
    }
    if case .waiting = connection { return .warn }
    if case .connected = connection, !mirror.isMapCurrent { return .warn }
    if mirror.status != nil, !mirror.isStatusFresh { return .warn }
    return .idle
}

private func baseBanner(
    mirror: RoomMemoryMirror,
    connection: LiveRoomConnection,
    started: Bool,
    now: TimeInterval
) -> LiveRoomBanner? {
    if case .connected(let server) = connection, server.memoryTopics.isEmpty {
        return LiveRoomBanner(text: "Room memory not running on Spark", tone: .warn)
    }
    if case .connecting(_, let attempt) = connection, attempt <= 1 {
        return LiveRoomBanner(text: "connecting", tone: .dim)
    }
    if case .waiting = connection {
        return LiveRoomBanner(text: "connection lost", tone: .bad)
    }
    if case .off = connection, started {
        return LiveRoomBanner(text: "connection lost", tone: .bad)
    }
    if case .connecting = connection {
        return LiveRoomBanner(text: "connection lost", tone: .bad)
    }
    // A socket that was never started is the Off screen, not an empty room.
    if case .off = connection, !started {
        return nil
    }
    if mirror.status != nil, !mirror.isStatusFresh {
        return LiveRoomBanner(text: "STALE · \(LiveRoomScreen.ageSeconds(mirror: mirror, now: now))s", tone: .warn)
    }
    if mirror.isStatusFresh, let status = mirror.status, status.state == .rigMoved {
        let residual = status.poseResidualM.map { "\(twoDecimals($0)) m" } ?? "guard not run"
        return LiveRoomBanner(text: "Rig moved: changes suspended · \(residual)", tone: .bad)
    }
    if mirror.mapState == .stale(.unverified) {
        return LiveRoomBanner(text: "unverified · Spark node predates publisher generations", tone: .warn)
    }
    if case .stale = mirror.mapState, !mirror.bricks.isEmpty {
        let epoch = mirror.mapEpoch.map(String.init) ?? "—"
        return LiveRoomBanner(text: "resyncing · as of epoch \(epoch)", tone: .warn)
    }
    if mirror.mapState == .empty || isStale(mirror.mapState) {
        return LiveRoomBanner(text: "Waiting for map", tone: .dim)
    }
    if mirror.isStatusFresh, let status = mirror.status {
        switch status.state {
        case .degraded:
            return LiveRoomBanner(text: degradedText(status), tone: .warn)
        case .paused:
            return LiveRoomBanner(text: "Paused", tone: .dim)
        case .building:
            return LiveRoomBanner(text: "Building reference", tone: .dim)
        case .tracking, .rigMoved:
            break
        }
    }
    return nil
}

private func degradedText(_ status: LiveRoomStatus) -> String {
    var silent: [String] = []
    if status.mid360Hz == 0 { silent.append("Mid-360") }
    if status.jt128Hz == 0 { silent.append("JT128") }
    if silent.isEmpty { return "Degraded" }
    return "Degraded · " + silent.joined(separator: ", ")
}

private func isStale(_ state: RoomMemoryMirror.MapState) -> Bool {
    if case .stale = state { return true }
    return false
}

private func roomCanvasSentence(mirror: RoomMemoryMirror, connection: LiveRoomConnection, started: Bool) -> String? {
    if case .connected(let server) = connection, server.memoryTopics.isEmpty {
        return "Room memory not running on Spark"
    }
    if case .off = connection, mirror.bricks.isEmpty, mirror.status == nil, mirror.events.isEmpty, mirror.eventsById.isEmpty {
        return "Room memory unavailable"
    }
    if case .off = connection, !started, mirror.bricks.isEmpty {
        return "Room memory unavailable"
    }
    return nil
}

private func roomBrickOpacity(mirror: RoomMemoryMirror, drawingMap: Bool) -> Float {
    guard drawingMap, !mirror.bricks.isEmpty else { return 1 }
    return mirror.isMapCurrent ? 1 : 0.35
}

private func columnUnavailable(mirror: RoomMemoryMirror, connection: LiveRoomConnection) -> Bool {
    if case .off = connection, mirror.bricks.isEmpty, mirror.status == nil, mirror.events.isEmpty {
        return true
    }
    return false
}

/// The event history line (PLAN-PUBGEN §4.8). Nil while nothing witnesses the history.
private func formatHistory(mirror: RoomMemoryMirror, requestPathExists: Bool) -> String? {
    if isLegacy(mirror) { return "history unverified (old Spark node)" }
    let history = mirror.eventHistory
    switch history.coverage {
    case .unknown:
        return nil
    case .empty:
        return "no events this run"
    case .completeThrough(let top):
        return "all events through event \(top) are here"
    case .missing:
        let (missing, lost) = history.missingCounts
        var line = "history incomplete: \(missing) missing"
        if lost > 0 { line += " (\(lost) lost on the Spark)" }
        if !requestPathExists, missing > lost { line += " · can't ask the Spark for them" }
        return line
    }
}

private func isLegacy(_ mirror: RoomMemoryMirror) -> Bool {
    mirror.wire == .legacy || mirror.mapState == .stale(.unverified)
}

private func restartNotice(_ mirror: RoomMemoryMirror) -> LiveRoomBanner? {
    guard case .broken = mirror.continuity else { return nil }
    return LiveRoomBanner(text: "Spark node restarted · events from the earlier run may be missing", tone: .warn)
}

/// The map needs a resend the client cannot ask for, or a request never left the Mac.
/// The history line says that for missing events itself.
private func resyncPhrase(mirror: RoomMemoryMirror, requestPathExists: Bool) -> String? {
    let recovery = isStale(mirror.mapState) && !isLegacy(mirror)
    let unsent = mirror.activity.requestsUnsent
    guard (recovery && !requestPathExists) || unsent > 0 else { return nil }
    if unsent > 0 { return "can't ask the Spark to resend the map · \(unsent) not sent" }
    return "can't ask the Spark to resend the map"
}

private func roomCaption(mirror: RoomMemoryMirror, markers: [LiveRoomMarker], unavailable: Bool) -> String {
    if unavailable { return "Room memory unavailable" }
    let changes = markers.filter(\.countsAsChange).count
    let checking = markers.filter(\.countsAsCandidate).count
    if mirror.isMapCurrent, mirror.openEvents.isEmpty {
        return "0 changes"
    }
    return "\(countWord(changes, one: "change", many: "changes")) · \(countWord(checking, one: "checking", many: "checking"))"
}

private func countWord(_ count: Int, one: String, many: String) -> String {
    count == 1 ? "1 \(one)" : "\(count) \(many)"
}

private func sortedOpen(_ events: [LiveRoomEvent]) -> [LiveRoomEvent] {
    events.sorted { lhs, rhs in
        if lhs.event.header.stamp != rhs.event.header.stamp {
            return lhs.event.header.stamp > rhs.event.header.stamp
        }
        return lhs.event.header.seq > rhs.event.header.seq
    }
}

private func rows(
    from events: [LiveRoomEvent],
    mirror: RoomMemoryMirror,
    history: Bool,
    outdated: Bool,
    earlier: Bool = false,
    kinds: Set<RoomMemory.ChangeEvent.Kind>,
    timeZone: TimeZone
) -> [LiveRoomEventRow] {
    events.compactMap { entry in
        guard kinds.contains(entry.event.kind) else { return nil }
        let event = entry.event
        let id = earlier ? "earlier-\(entry.seq)-\(event.eventId)" : history ? "\(entry.seq)-\(event.eventId)" : String(event.eventId)
        let cells = event.voxelCount == 1 ? "1 cell" : "\(event.voxelCount) cells"
        let line1 = "\(kindWord(event.kind)) · \(stateName(event.state)) · \(cells) · \(liveRoomClock(event.header.stamp, timeZone: timeZone))"
        var bits: [String] = [boxSeen(event.evidence), falseAlarmLimit(event.alpha), sensorWords(event.sensors)]
        if event.mask == nil { bits.append("box only") }
        if history, LiveRoomMarkers.isWaiting(entry, mirror: mirror) {
            bits.append("saved · map updating")
        }
        if earlier {
            bits.append("earlier run · latest status unknown")
        } else if !history, outdated {
            bits.append("latest status unknown")
        }
        return LiveRoomEventRow(id: id, kind: event.kind, line1: line1, line2: bits.joined(separator: " · "), history: history)
    }
}

private func sensorWords(_ sensors: RoomMemory.Sensors) -> String {
    var names: [String] = []
    if sensors.contains(.mid360) { names.append("Mid-360") }
    if sensors.contains(.jt128) { names.append("JT128") }
    if names.isEmpty { return "sensor —" }
    return names.joined(separator: ", ")
}

private func boxCorners(min lo: SIMD3<Float>, max hi: SIMD3<Float>) -> [SIMD3<Float>] {
    [
        SIMD3(lo.x, lo.y, lo.z), SIMD3(hi.x, lo.y, lo.z), SIMD3(hi.x, hi.y, lo.z), SIMD3(lo.x, hi.y, lo.z),
        SIMD3(lo.x, lo.y, hi.z), SIMD3(hi.x, lo.y, hi.z), SIMD3(hi.x, hi.y, hi.z), SIMD3(lo.x, hi.y, hi.z),
    ]
}

private func marker(_ entry: LiveRoomEvent, id: String, waiting: Bool, selected: Bool) -> LiveRoomMarker {
    let event = entry.event
    let confirmed = event.state == .confirmed && !waiting
    let dashed = !confirmed
    var fill: Float = 0
    if confirmed { fill = selected ? LiveRoomMarkers.emphasizedFillAlpha : LiveRoomMarkers.fillAlpha }
    var arrow: LiveRoomArrow?
    if confirmed, event.kind == .moved, let from = event.fromCentroid {
        arrow = LiveRoomMarkers.arrow(from: from, to: event.centroid)
    }
    return LiveRoomMarker(
        id: id,
        kind: event.kind,
        corners: boxCorners(min: event.aabb.min, max: event.aabb.max),
        dashed: dashed,
        fillAlpha: fill,
        onTop: selected,
        countsAsChange: confirmed,
        countsAsCandidate: event.state == .candidate && !waiting,
        arrow: arrow,
        centroid: event.centroid,
        color: LiveRoomMarkers.color(event.kind)
    )
}
