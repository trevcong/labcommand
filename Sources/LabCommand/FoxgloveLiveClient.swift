import Foundation
import Combine
import os
import LabCommandCore

/// The latest live cloud per sensor, already in `world`.
struct LiveClouds {
    var mid: PointCloudFrame?
    var jt: PointCloudFrame?
    /// Message rate per sensor as published by the bridge, before any throttling.
    var hz: [SensorID: Double] = [:]
    /// Changes whenever either cloud does — header stamps alone can repeat.
    var revision = 0

    func frame(_ sensor: SensorID) -> PointCloudFrame? {
        switch sensor {
        case .mid360: return mid
        case .jt128: return jt
        }
    }

    var sensors: Set<SensorID> {
        Set(SensorID.allCases.filter { frame($0) != nil })
    }

    /// What the badge reports: shown points and rate, per sensor on screen.
    var readings: [LiveViewMode.Reading] {
        SensorID.allCases.compactMap { sensor in
            frame(sensor).map { LiveViewMode.Reading(label: sensor.label, points: $0.count, hz: hz[sensor]) }
        }
    }
}

/// Live lidar clouds for the 3D view, straight from foxglove_bridge on Spark
/// (`ws://<spark>:8765`, subprotocol `foxglove.sdk.v1` or `foxglove.websocket.v1`).
///
/// The socket, CDR decode, decimation and TF into `world` run off the main thread;
/// this object only publishes the latest frame per sensor, at ~10 Hz. It is its own
/// `ObservableObject` so those updates re-render the canvas, not everything that
/// watches `AppState`.
@MainActor
final class FoxgloveLiveClient: ObservableObject {
    @Published private(set) var clouds = LiveClouds()
    /// True while fresh frames are arriving. False means the caller's fallback shows.
    @Published private(set) var isLive = false
    /// Bumped once, by the first live data. The view refits the camera once per
    /// value — so never per frame, and not again when a dropped stream comes back,
    /// which would yank the camera out of the operator's hands.
    @Published private(set) var sourceGeneration = 0

    /// Mission-log lines (`[FoxgloveLive] …`): connection state, subscriptions,
    /// first-frame point counts.
    var onLog: ((String, MissionEventKind) -> Void)?
    /// No live data within `fallbackAfter`, the socket failed, or the stream stopped.
    var onFallback: (() -> Void)?
    var onLive: (() -> Void)?

    private let hosts: @Sendable () -> [String]
    private let port: Int
    private let mailbox = LiveFrameMailbox()
    private var session: URLSession?
    private var worker: Task<Void, Never>?
    private var ticker: Task<Void, Never>?
    private var startedAt: TimeInterval = 0
    private var fallbackRequested = false
    private var hasBeenLive = false
    /// Sensors the last `mode=LIVE` line named.
    private var loggedLiveSensors: Set<SensorID> = []

    /// - Parameter hosts: Candidates for `ws://<host>:<port>`, re-read before every
    ///   attempt so a hop that SSH discovers later is picked up.
    init(
        hosts: @escaping @Sendable () -> [String] = {
            FoxgloveProtocol.streamHosts(lastLAN: SparkHopCache.load())
        },
        port: Int = CaptureRecipe.foxglovePort
    ) {
        self.hosts = hosts
        self.port = port
    }

    func start() {
        guard worker == nil else { return }
        startedAt = ProcessInfo.processInfo.systemUptime
        fallbackRequested = false

        let config = URLSessionConfiguration.ephemeral
        config.waitsForConnectivity = false
        // The stream's own monitor owns timeouts; URLSession's request timeout would
        // also cut a healthy socket that is merely quiet because the sensors are dark.
        config.timeoutIntervalForRequest = 24 * 60 * 60
        let session = URLSession(configuration: config)
        self.session = session

        let sink = LiveStream.Sink(mailbox: mailbox) { [weak self] text, kind, mission in
            self?.log(text, kind: kind, mission: mission)
        }
        let hosts = self.hosts
        let port = self.port
        worker = Task.detached(priority: .userInitiated) {
            await LiveStream.run(hosts: hosts, port: port, session: session, sink: sink)
        }
        ticker = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(nanoseconds: UInt64(LiveStream.uiInterval * 1_000_000_000))
                guard let self, !Task.isCancelled else { return }
                self.tick()
            }
        }
    }

    func stop() {
        worker?.cancel()
        ticker?.cancel()
        worker = nil
        ticker = nil
        session?.invalidateAndCancel()
        session = nil
    }

    // MARK: - Publishing (~10 Hz)

    private func tick() {
        let now = ProcessInfo.processInfo.systemUptime
        let latest = mailbox.take(now: now, staleAfter: LiveStream.staleAfter)

        if isLive {
            if latest.frames.isEmpty {
                leaveLive(latest.connectionEnded ? "connection lost" : "stream stopped")
            } else if latest.revision != clouds.revision {
                show(latest)
                logLiveMode()            }
        } else if let first = latest.firstFrameAt, !latest.frames.isEmpty,
                  latest.expected.isSubset(of: latest.frames.keys) || now - first >= LiveStream.settle {
            // Going live waits a beat for the second sensor so the one camera fit
            // frames both clouds rather than whichever arrived first.
            enterLive(latest)
        } else if latest.connectionEnded {
            requestFallback("no connection")
        } else if latest.frames.isEmpty, now - startedAt >= LiveStream.fallbackAfter {
            // Frames already arriving are settling into live, not missing.
            requestFallback("no live data after \(Int(LiveStream.fallbackAfter)) s")
        }
    }

    private func show(_ latest: LiveFrameMailbox.Snapshot) {
        clouds = LiveClouds(
            mid: latest.frames[.mid360],
            jt: latest.frames[.jt128],
            hz: latest.hz,
            revision: latest.revision
        )
    }

    /// `mode=LIVE` on entry, and again when a sensor joins or leaves the view —
    /// going live can beat the second sensor by the settle time.
    private func logLiveMode() {
        let sensors = clouds.sensors
        guard sensors != loggedLiveSensors else { return }
        loggedLiveSensors = sensors
        log("mode=\(LiveViewMode.live.rawValue) — \(LiveViewMode.summary(clouds.readings))", kind: .info, mission: false)
    }

    private func enterLive(_ latest: LiveFrameMailbox.Snapshot) {
        show(latest)
        isLive = true
        fallbackRequested = false
        if !hasBeenLive {
            hasBeenLive = true
            sourceGeneration += 1
        }
        let summary = SensorID.allCases
            .compactMap { sensor in latest.frames[sensor].map { "\(sensor.label) \($0.count) pts" } }
            .joined(separator: ", ")
        log("streaming live clouds — \(summary)", kind: .info, mission: true)
        logLiveMode()
        onLive?()
    }

    private func leaveLive(_ reason: String) {
        isLive = false
        clouds = LiveClouds()
        loggedLiveSensors = []
        // Count the fallback window again from here.
        startedAt = ProcessInfo.processInfo.systemUptime
        requestFallback(reason)
    }

    private func requestFallback(_ reason: String) {
        guard !fallbackRequested else { return }
        fallbackRequested = true
        log("\(reason) — showing sample cloud", kind: .info, mission: true)
        log("mode=\(LiveViewMode.sample.rawValue)", kind: .info, mission: false)
        onFallback?()
    }

    private func log(_ text: String, kind: MissionEventKind, mission: Bool) {
        let line = "[FoxgloveLive] \(text)"
        // stderr, not print(): stdout is block-buffered off a TTY, so a killed or
        // redirected app would lose exactly the lines that explain a dead stream.
        FileHandle.standardError.write(Data((line + "\n").utf8))
        if mission {
            onLog?(line, kind)
        }
    }
}

// MARK: - Mailbox

/// Hand-off between the socket task and the main-actor ticker: the newest frame per
/// sensor, overwritten in place, so a slow UI drops frames instead of queueing them.
private final class LiveFrameMailbox: Sendable {
    struct Snapshot: Sendable {
        var frames: [SensorID: PointCloudFrame] = [:]
        /// Message rate for each sensor in `frames` that has one yet.
        var hz: [SensorID: Double] = [:]
        var revision = 0
        /// Sensors the socket is subscribed to right now.
        var expected: Set<SensorID> = []
        /// When the current run of frames began (system uptime).
        var firstFrameAt: TimeInterval?
        /// A connection attempt ended since the last `take`.
        var connectionEnded = false
    }

    private struct State: Sendable {
        var snapshot = Snapshot()
        var receivedAt: [SensorID: TimeInterval] = [:]
        var rates: [SensorID: ArrivalRate] = [:]
    }

    private let state = OSAllocatedUnfairLock(initialState: State())

    func put(_ frame: PointCloudFrame, sensor: SensorID, at now: TimeInterval) {
        state.withLock { state in
            if state.snapshot.frames.isEmpty {
                state.snapshot.firstFrameAt = now
            }
            state.snapshot.frames[sensor] = frame
            state.snapshot.revision += 1
            state.receivedAt[sensor] = now
        }
    }

    /// Every message for the sensor, including ones throttled before decoding.
    func noteArrival(_ sensor: SensorID, at now: TimeInterval) {
        state.withLock { $0.rates[sensor, default: ArrivalRate()].note(now) }
    }

    func setExpected(_ sensors: Set<SensorID>) {
        state.withLock { $0.snapshot.expected = sensors }
    }

    /// The sensor's topic went away; its last cloud must not linger on screen.
    func drop(_ sensor: SensorID) {
        state.withLock { Self.remove(sensor, from: &$0) }
    }

    func connectionEnded() {
        state.withLock { state in
            for sensor in SensorID.allCases {
                Self.remove(sensor, from: &state)
            }
            state.snapshot.expected = []
            state.snapshot.connectionEnded = true
        }
    }

    /// The current frames, minus any sensor that has gone quiet for `staleAfter`.
    func take(now: TimeInterval, staleAfter: TimeInterval) -> Snapshot {
        state.withLock { state in
            for (sensor, at) in state.receivedAt where now - at > staleAfter {
                Self.remove(sensor, from: &state)
            }
            var snapshot = state.snapshot
            snapshot.hz = [:]
            for sensor in snapshot.frames.keys {
                snapshot.hz[sensor] = state.rates[sensor]?.hz(now: now)
            }
            state.snapshot.connectionEnded = false
            return snapshot
        }
    }

    private static func remove(_ sensor: SensorID, from state: inout State) {
        state.rates[sensor] = nil
        guard state.snapshot.frames.removeValue(forKey: sensor) != nil else { return }
        state.receivedAt[sensor] = nil
        state.snapshot.revision += 1
        if state.snapshot.frames.isEmpty {
            state.snapshot.firstFrameAt = nil
        }
    }
}

// MARK: - Socket

/// The off-main half: connect, subscribe, decode, reconnect.
private enum LiveStream {
    static let maxPointsPerSensor = 60_000
    static let uiInterval: TimeInterval = 0.1
    /// Frames closer together than this are dropped before decoding. A little under
    /// the UI interval so arrival jitter doesn't halve a 10 Hz sensor.
    static let minFrameInterval: TimeInterval = 0.08
    static let fallbackAfter: TimeInterval = 3
    static let staleAfter: TimeInterval = 3
    /// How long going live waits for a subscribed sensor that hasn't sent yet. A
    /// JT128 frame is ~3 MB and over Tailscale lands up to ~0.7 s after the first
    /// Mid-360 one; the one camera fit should frame both.
    static let settle: TimeInterval = 2
    static let handshakeTimeout: TimeInterval = 5
    static let pingInterval: TimeInterval = 10
    /// URLSession's default cap is 1 MiB; one JT128 PointCloud2 is several.
    static let maxMessageBytes = 64 << 20

    struct Sink: Sendable {
        let mailbox: LiveFrameMailbox
        /// (text, kind, alsoMissionLog). Everything goes to stderr; only state changes
        /// reach the mission log, so an offline Spark can't flood it with retries.
        let log: @MainActor @Sendable (String, MissionEventKind, Bool) -> Void
    }

    private struct ConnectionEnd {
        var opened: Bool
        var reason: String
    }

    /// What the monitor and the receive loop share about one socket.
    private struct SocketHealth: Sendable {
        var opened = false
        var active = false
        var failure: String?
    }

    /// Subscriptions are per connection — rebuilt from scratch on every reconnect.
    private struct Link {
        let endpoint: String
        var opened = false
        var topics: [UInt32: String] = [:]
        var subscriptionByChannel: [UInt32: UInt32] = [:]
        var sensorBySubscription: [UInt32: SensorID] = [:]
        var nextSubscriptionId: UInt32 = 1
        var lastAccepted: [SensorID: TimeInterval] = [:]
        var reported: Set<SensorID> = []
        var undecodable: Set<SensorID> = []
        var skipped: Set<UInt32> = []

        var sensors: Set<SensorID> { Set(sensorBySubscription.values) }
    }

    static func run(
        hosts: @Sendable () -> [String],
        port: Int,
        session: URLSession,
        sink: Sink
    ) async {
        var failures = 0
        var hostIndex = 0
        var announcedDown = false

        while !Task.isCancelled {
            let candidates = hosts()
            guard !candidates.isEmpty else {
                await sink.log("no Spark host to try", .error, !announcedDown)
                announcedDown = true
                sink.mailbox.connectionEnded()
                try? await Task.sleep(nanoseconds: 30_000_000_000)
                continue
            }
            let host = candidates[hostIndex % candidates.count]
            let endpoint = "ws://\(host):\(port)"
            await sink.log("connecting \(endpoint)", .info, !announcedDown)

            let end = await connect(endpoint: endpoint, session: session, sink: sink)
            sink.mailbox.connectionEnded()
            if Task.isCancelled { break }

            if end.opened {
                failures = 0
                announcedDown = false
                await sink.log("connection lost \(endpoint) — \(end.reason)", .info, true)
            } else {
                failures += 1
                hostIndex += 1
                await sink.log("\(endpoint) unreachable — \(end.reason); retrying", .info, !announcedDown)
                announcedDown = true
            }

            let delay = FoxgloveProtocol.reconnectDelay(afterFailures: failures)
            try? await Task.sleep(nanoseconds: UInt64(delay * 1_000_000_000))
        }
    }

    private static func connect(endpoint: String, session: URLSession, sink: Sink) async -> ConnectionEnd {
        guard let url = URL(string: endpoint) else {
            return ConnectionEnd(opened: false, reason: "bad URL")
        }
        let socket = session.webSocketTask(with: url, protocols: FoxgloveProtocol.subprotocols)
        socket.maximumMessageSize = maxMessageBytes
        let health = OSAllocatedUnfairLock(initialState: SocketHealth())
        socket.resume()

        let monitor = Task { await watch(socket, health: health) }
        defer {
            monitor.cancel()
            socket.cancel(with: .goingAway, reason: nil)
        }

        // `receive()` doesn't notice task cancellation on its own.
        return await withTaskCancellationHandler {
            await receive(from: socket, endpoint: endpoint, health: health, sink: sink)
        } onCancel: {
            socket.cancel(with: .goingAway, reason: nil)
        }
    }

    /// Fails the socket when the server never greets us, or later goes silent: a
    /// half-dead link would otherwise sit in `receive()` forever and never reconnect.
    /// Pings keep the judgement fair while the sensors are dark and no data flows.
    private static func watch(_ socket: URLSessionWebSocketTask, health: OSAllocatedUnfairLock<SocketHealth>) async {
        func fail(_ reason: String) {
            health.withLock { $0.failure = reason }
            socket.cancel(with: .goingAway, reason: nil)
        }

        do {
            try await Task.sleep(nanoseconds: UInt64(handshakeTimeout * 1_000_000_000))
            guard health.withLock({ $0.opened }) else {
                return fail("no serverInfo within \(Int(handshakeTimeout)) s")
            }
            while true {
                health.withLock { $0.active = false }
                socket.sendPing { error in
                    if error == nil {
                        health.withLock { $0.active = true }
                    }
                }
                try await Task.sleep(nanoseconds: UInt64(pingInterval * 1_000_000_000))
                guard health.withLock({ $0.active }) else {
                    return fail("no data or pong for \(Int(pingInterval)) s")
                }
            }
        } catch {
            // Cancelled: the connection ended some other way.
        }
    }

    private static func receive(
        from socket: URLSessionWebSocketTask,
        endpoint: String,
        health: OSAllocatedUnfairLock<SocketHealth>,
        sink: Sink
    ) async -> ConnectionEnd {
        var link = Link(endpoint: endpoint)
        do {
            while true {
                let message = try await socket.receive()
                health.withLock {
                    $0.opened = true
                    $0.active = true
                }
                switch message {
                case .string(let text):
                    try await handle(text: text, link: &link, socket: socket, sink: sink)
                case .data(let frame):
                    if !link.opened { await markOpened(&link, serverName: "", sink: sink) }
                    await handle(binary: frame, link: &link, sink: sink)
                @unknown default:
                    break
                }
            }
        } catch {
            let reason = health.withLock { $0.failure } ?? describe(error, socket: socket)
            return ConnectionEnd(opened: link.opened, reason: reason)
        }
    }

    private static func describe(_ error: Error, socket: URLSessionWebSocketTask) -> String {
        if socket.closeCode != .invalid {
            return "server closed the socket (code \(socket.closeCode.rawValue))"
        }
        // A refused upgrade (e.g. an unsupported subprotocol) is otherwise just
        // "bad server response".
        if let http = socket.response as? HTTPURLResponse, http.statusCode != 101 {
            return "handshake refused (HTTP \(http.statusCode)) — \(error.localizedDescription)"
        }
        return error.localizedDescription
    }

    private static func markOpened(_ link: inout Link, serverName: String, sink: Sink) async {
        link.opened = true
        let server = serverName.isEmpty ? "" : " (\(serverName))"
        await sink.log("connected \(link.endpoint)\(server)", .info, true)
    }

    // MARK: Text frames

    private static func handle(
        text: String,
        link: inout Link,
        socket: URLSessionWebSocketTask,
        sink: Sink
    ) async throws {
        let message = FoxgloveProtocol.parseServerMessage(text)
        if !link.opened {
            var serverName = ""
            if case .serverInfo(let name) = message { serverName = name }
            await markOpened(&link, serverName: serverName, sink: sink)
        }

        switch message {
        case .advertise(let channels):
            try await subscribe(to: channels, link: &link, socket: socket, sink: sink)
        case .unadvertise(let channelIds):
            for channelId in channelIds {
                let topic = link.topics.removeValue(forKey: channelId)
                link.skipped.remove(channelId)
                guard let subscriptionId = link.subscriptionByChannel.removeValue(forKey: channelId),
                      let sensor = link.sensorBySubscription.removeValue(forKey: subscriptionId) else {
                    continue
                }
                // Forget the sensor entirely so a re-advertised topic starts clean.
                link.lastAccepted[sensor] = nil
                link.reported.remove(sensor)
                link.undecodable.remove(sensor)
                sink.mailbox.drop(sensor)
                sink.mailbox.setExpected(link.sensors)
                await sink.log("\(topic ?? "channel \(channelId)") unadvertised — \(sensor.label) stream ended", .info, true)
            }
        case .status(let level, let text):
            await sink.log("server status (level \(level)): \(text)", .info, false)
        case .serverInfo, .other, nil:
            break
        }
    }

    private static func subscribe(
        to channels: [FoxgloveProtocol.Channel],
        link: inout Link,
        socket: URLSessionWebSocketTask,
        sink: Sink
    ) async throws {
        var requests: [FoxgloveProtocol.Subscription] = []
        var lines: [String] = []

        for channel in channels {
            guard let sensor = FoxgloveProtocol.liveSensor(forTopic: channel.topic),
                  link.subscriptionByChannel[channel.id] == nil else {
                continue
            }
            link.topics[channel.id] = channel.topic

            var refusal: String?
            if channel.encoding != FoxgloveProtocol.cdrEncoding {
                refusal = "encoding is \"\(channel.encoding)\", need cdr"
            } else if !channel.carriesPointCloud2 {
                refusal = "schema is \(channel.schemaName), need sensor_msgs/msg/PointCloud2"
            }
            if let refusal {
                if link.skipped.insert(channel.id).inserted {
                    await sink.log("not subscribing \(channel.topic): \(refusal)", .error, true)
                }
                continue
            }
            // One channel per sensor is plenty.
            guard !link.sensors.contains(sensor) else { continue }

            let subscriptionId = link.nextSubscriptionId
            link.nextSubscriptionId += 1
            link.subscriptionByChannel[channel.id] = subscriptionId
            link.sensorBySubscription[subscriptionId] = sensor
            requests.append(FoxgloveProtocol.Subscription(id: subscriptionId, channelId: channel.id))

            let schema = channel.schemaName.isEmpty ? "schema unnamed" : channel.schemaName
            lines.append("subscribed \(channel.topic) → \(sensor.label) (\(schema), channel \(channel.id), sub \(subscriptionId))")
        }

        guard !requests.isEmpty else { return }
        try await socket.send(.string(FoxgloveProtocol.subscribeMessage(requests)))
        sink.mailbox.setExpected(link.sensors)
        for line in lines {
            await sink.log(line, .info, true)
        }
    }

    // MARK: Binary frames

    private static func handle(binary frame: Data, link: inout Link, sink: Sink) async {
        guard let message = FoxgloveProtocol.parseMessageData(frame),
              let sensor = link.sensorBySubscription[message.subscriptionId] else {
            return
        }

        // Throttle on the raw payload: a frame the UI would never show isn't decoded.
        let now = ProcessInfo.processInfo.systemUptime
        sink.mailbox.noteArrival(sensor, at: now)
        if let last = link.lastAccepted[sensor], now - last < minFrameInterval {
            return
        }
        link.lastAccepted[sensor] = now

        let decoded: PointCloudFrame
        do {
            decoded = try PointCloud2CDR.decode(message.payload, maxPoints: maxPointsPerSensor)
        } catch {
            if link.undecodable.insert(sensor).inserted {
                await sink.log("\(sensor.label) frame not decodable — \(error)", .error, true)
            }
            return
        }
        guard !decoded.isEmpty else { return }

        let world = RigTransforms.inWorld(decoded, sensor: sensor)
        sink.mailbox.put(world, sensor: sensor, at: now)

        if link.reported.insert(sensor).inserted {
            let sourceFrame = decoded.frameId.flatMap { $0.isEmpty ? nil : $0 } ?? "no frame_id"
            await sink.log(
                "\(sensor.label) first frame: \(world.count) pts shown (\(sourceFrame) → \(RigTransforms.worldFrame), \(message.payload.count) bytes on the wire)",
                .info,
                true
            )
        }
    }
}
