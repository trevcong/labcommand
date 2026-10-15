import Foundation
import simd

/// How this process should open the room page. `LABCOMMAND_LIVE_ROOM` is the switch.
/// Fixture mode never constructs a `LiveRoomClient`.
public enum LiveRoomLaunch: Equatable, Sendable {
    /// A live socket. `opensRoom` is true only for the value `live` (the gate launch).
    case live(opensRoom: Bool)
    case fixture(LiveRoomFixture.Mode)

    public var startsClient: Bool {
        if case .live = self { return true }
        return false
    }

    /// Room is the destination. An unset variable stays on Rig.
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

/// Offline room. Decodes the hand goldens and drives `RoomMemoryMirror`. It does not open a socket.
public struct LiveRoomFixture: Equatable, Sendable {
    public enum Mode: String, Equatable, Sendable, CaseIterable {
        case map, status, stale, unavailable
    }

    public var mode: Mode
    public var mirror: RoomMemoryMirror
    public var connection: LiveRoomConnection
    private var statusSeq: UInt64

    public static func load(mode: Mode, root: URL?, now: TimeInterval) throws -> LiveRoomFixture {
        switch mode {
        case .unavailable:
            return LiveRoomFixture(mode: .unavailable, mirror: RoomMemoryMirror(), connection: .off, statusSeq: 0)
        case .status:
            var mirror = RoomMemoryMirror()
            _ = mirror.connectionOpened(at: now, canRequest: false)
            let bytes = try goldenBytes(name: "memory_status_le", root: root)
            _ = mirror.receive(topic: .status, payload: bytes, at: now)
            return LiveRoomFixture(mode: .status, mirror: mirror, connection: connected, statusSeq: 0)
        case .map, .stale:
            let at = mode == .stale ? now - 4 : now
            var mirror = RoomMemoryMirror()
            _ = mirror.connectionOpened(at: at, canRequest: false)
            _ = mirror.receive(topic: .mapBricks, payload: try goldenBytes(name: "map_bricks_snapshot_zlib_le", root: root), at: at)
            _ = mirror.receive(topic: .events, payload: try goldenBytes(name: "change_event_moved_mask_le", root: root), at: at)
            _ = mirror.receive(topic: .events, payload: try goldenBytes(name: "change_event_appeared_le", root: root), at: at)
            _ = mirror.receive(topic: .events, payload: try disappearedEvent().encodedCDR(), at: at)
            let seq: UInt64 = 1
            _ = mirror.receive(topic: .status, payload: try trackingStatus(seq: seq, stamp: RoomMemory.Time(sec: 1, nanosec: 0)).encodedCDR(), at: at)
            if mode == .stale {
                _ = mirror.tick(at: now)
            }
            return LiveRoomFixture(mode: mode, mirror: mirror, connection: connected, statusSeq: seq)
        }
    }

    /// Map mode only: a new status seq so the 3 s freshness window stays open.
    public func refreshed(at now: TimeInterval) -> LiveRoomFixture {
        guard mode == .map else { return self }
        var copy = self
        let seq = statusSeq &+ 1
        guard let payload = try? Self.trackingStatus(seq: seq, stamp: RoomMemory.Time(sec: 1, nanosec: UInt32(seq % 1_000_000_000))).encodedCDR() else {
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

    /// Tracking status for room `lab`, coverage and `eventsSeq` left unmeasured.
    static func trackingStatus(seq: UInt64, stamp: RoomMemory.Time) -> RoomMemory.MemoryStatus {
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
            eventsSeq: nil
        )
    }

    /// Confirmed disappeared box, clear of the two hand-golden boxes. Not a new hex file.
    static func disappearedEvent() -> RoomMemory.ChangeEvent {
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
            mask: nil
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
    public var statusLine: String
    /// Centered copy that replaces the voxels. Nil draws the map.
    public var canvasSentence: String?
    public var brickOpacity: Float
    public var markers: [LiveRoomMarker]
    public var incompleteLine: String?
    public var standaloneResync: String?
    public var caption: String
    public var openRows: [LiveRoomEventRow]
    public var historyRows: [LiveRoomEventRow]
    /// The column prints `caption` in place of the event sections.
    public var columnIsUnavailable: Bool

    public static func make(
        mirror: RoomMemoryMirror,
        connection: LiveRoomConnection,
        requestPathExists: Bool,
        started: Bool,
        now: TimeInterval,
        kinds: Set<RoomMemory.ChangeEvent.Kind> = Set(RoomMemory.ChangeEvent.Kind.allCases),
        selection: String? = nil
    ) -> LiveRoomScreen {
        let canvas = roomCanvasSentence(mirror: mirror, connection: connection, started: started)
        var banner = baseBanner(mirror: mirror, connection: connection, started: started, now: now)
        var incomplete = formatIncomplete(mirror.eventsState)
        let phrase = resyncPhrase(mirror: mirror, requestPathExists: requestPathExists)
        var standalone: String?
        if let phrase {
            if banner?.text.hasPrefix("resyncing · as of epoch") == true {
                banner?.text += " · " + phrase
            } else if incomplete != nil {
                incomplete = (incomplete ?? "") + " · " + phrase
            } else {
                standalone = phrase
            }
        }
        let markers = LiveRoomMarkers.draw(mirror: mirror, kinds: kinds, selection: selection)
        let outdated = mirror.eventsState != .complete
        let openRows = rows(from: sortedOpen(mirror.openEvents), mirror: mirror, history: false, outdated: outdated, kinds: kinds)
        let historySource = Array(mirror.events.filter { isTerminal($0.event.state) }.reversed())
        let historyRows = rows(from: historySource, mirror: mirror, history: true, outdated: false, kinds: kinds)
        let unavailable = columnUnavailable(mirror: mirror, connection: connection)
        return LiveRoomScreen(
            pip: roomPip(mirror: mirror, connection: connection),
            banner: banner,
            statusLine: statusLine(mirror: mirror, connection: connection, now: now),
            canvasSentence: canvas,
            brickOpacity: roomBrickOpacity(mirror: mirror, drawingMap: canvas == nil),
            markers: markers,
            incompleteLine: incomplete,
            standaloneResync: standalone,
            caption: roomCaption(mirror: mirror, markers: markers, unavailable: unavailable),
            openRows: openRows,
            historyRows: historyRows,
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
        lines.append("status \(screen.statusLine)")
        lines.append("pip \(screen.pip.rawValue)")
        lines.append("markers \(screen.markers.count)")
        lines.append("caption \(screen.caption)")
        if let incomplete = screen.incompleteLine {
            lines.append("incomplete \(incomplete)")
        }
        if let standalone = screen.standaloneResync {
            lines.append("resync \(standalone)")
        }
        lines.append("rows")
        for row in screen.openRows + screen.historyRows {
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

private func isConnected(_ connection: LiveRoomConnection) -> Bool {
    if case .connected = connection { return true }
    return false
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
    case .candidate: return "candidate"
    case .confirmed: return "confirmed"
    case .accepted: return "accepted"
    case .reverted: return "reverted"
    case .expired: return "expired"
    }
}

private func isTerminal(_ state: RoomMemory.ChangeEvent.State) -> Bool {
    state == .accepted || state == .reverted || state == .expired
}

/// `{sec: 0, nanosec: 0}` is `time —`. Anything else is `HH:mm:ss` UTC.
func liveRoomClock(_ time: RoomMemory.Time) -> String {
    if time.sec == 0, time.nanosec == 0 { return "time —" }
    var day = Int(time.sec) % 86_400
    if day < 0 { day += 86_400 }
    return String(format: "%02d:%02d:%02d", day / 3600, (day % 3600) / 60, day % 60)
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

private func formatIncomplete(_ state: RoomMemoryMirror.EventsState) -> String? {
    switch state {
    case .complete:
        return nil
    case .incomplete(let since):
        guard let since else { return "incomplete" }
        return "incomplete since \(liveRoomClock(since))"
    }
}

private func resyncPhrase(mirror: RoomMemoryMirror, requestPathExists: Bool) -> String? {
    let recovery = isStale(mirror.mapState) || mirror.eventsState != .complete
    let unsent = mirror.activity.requestsUnsent
    guard (recovery && !requestPathExists) || unsent > 0 else { return nil }
    if unsent > 0 { return "cannot request resync · \(unsent) unsent" }
    return "cannot request resync"
}

private func roomCaption(mirror: RoomMemoryMirror, markers: [LiveRoomMarker], unavailable: Bool) -> String {
    if unavailable { return "Room memory unavailable" }
    let changes = markers.filter(\.countsAsChange).count
    let candidates = markers.filter(\.countsAsCandidate).count
    if mirror.isMapCurrent, mirror.openEvents.isEmpty {
        return "0 changes"
    }
    return "\(countWord(changes, one: "change", many: "changes")) · \(countWord(candidates, one: "candidate", many: "candidates")) on map"
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
    kinds: Set<RoomMemory.ChangeEvent.Kind>
) -> [LiveRoomEventRow] {
    events.compactMap { entry in
        guard kinds.contains(entry.event.kind) else { return nil }
        let event = entry.event
        let id = history ? "\(entry.seq)-\(event.eventId)" : String(event.eventId)
        let voxels = event.voxelCount == 1 ? "1 voxel" : "\(event.voxelCount) voxels"
        let line1 = "\(kindWord(event.kind)) · \(stateName(event.state)) · \(voxels) · \(liveRoomClock(event.header.stamp))"
        var bits: [String] = []
        if let evidence = event.evidence {
            bits.append("evidence \(Int((Double(evidence) * 100).rounded()))%")
        } else {
            bits.append("evidence —")
        }
        if let alpha = event.alpha {
            bits.append("α \(twoDecimals(alpha))")
        } else {
            bits.append("uncalibrated")
        }
        bits.append(sensorWords(event.sensors))
        if event.mask == nil { bits.append("no mask") }
        if history, LiveRoomMarkers.isWaiting(entry, mirror: mirror) {
            bits.append("accepted · map updating")
        }
        if !history, outdated { bits.append("may be outdated") }
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
