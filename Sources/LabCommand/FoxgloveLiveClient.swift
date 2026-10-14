import Foundation
import Combine
import os
import LabCommandCore

/// What the LIVE badge reports about the clouds on screen. The clouds themselves
/// don't pass through SwiftUI: the renderer pulls them from `LiveFrameMailbox`.
struct LiveClouds: Equatable {
    /// Shown points and delivered (rendered) rate, per sensor on screen.
    var readings: [LiveViewMode.Reading] = []
    var sensors: Set<SensorID> = []
}

/// Live lidar clouds for the 3D view, straight from foxglove_bridge on Spark
/// (`ws://<spark>:8765`, subprotocol `foxglove.sdk.v1` or `foxglove.websocket.v1`).
///
/// One socket per sensor, so a 3 MB JT128 frame never queues a Mid-360 frame
/// behind it. Each socket's receive loop only parses headers and drops the raw
/// payload into a latest-only slot; a decoder task per sensor decodes, moves the
/// cloud into `world` and packs it for the GPU. The renderer pulls the newest
/// packed cloud on every draw, so neither SwiftUI nor a polling timer sits between
/// the socket and the screen. This object only drives the LIVE/SAMPLE state and
/// the badge, and is its own `ObservableObject` so those updates re-render the
/// canvas, not everything that watches `AppState`.
@MainActor
final class FoxgloveLiveClient: ObservableObject {
    @Published private(set) var clouds = LiveClouds()
    /// True while fresh frames are arriving. False means the caller's fallback shows.
    @Published private(set) var isLive = false
    /// Bumped once, by the first live data. The view refits the camera once per
    /// value — so never per frame, and not again when a dropped stream comes back,
    /// which would yank the camera out of the operator's hands.
    @Published private(set) var sourceGeneration = 0

    /// Newest packed cloud per sensor, for the renderer to pull.
    let mailbox = LiveFrameMailbox()

    /// Mission-log lines (`[FoxgloveLive] …`): connection state, subscriptions,
    /// first-frame point counts.
    var onLog: ((String, MissionEventKind) -> Void)?
    /// No live data within `fallbackAfter`, the socket failed, or the stream stopped.
    var onFallback: (() -> Void)?
    var onLive: (() -> Void)?

    private let hosts: @Sendable () -> [String]
    private let port: Int
    private var session: URLSession?
    private var workers: [Task<Void, Never>] = []
    private var ticker: Task<Void, Never>?
    private var startedAt: TimeInterval = 0
    private var fallbackRequested = false
    private var hasBeenLive = false
    private var lastBadgeAt: TimeInterval = 0
    private var lastRateLogAt: TimeInterval = 0
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
        guard workers.isEmpty else { return }
        startedAt = ProcessInfo.processInfo.systemUptime
        lastRateLogAt = startedAt
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
        workers = SensorID.allCases.map { sensor in
            Task.detached(priority: .userInitiated) {
                await LiveStream.run(sensor: sensor, hosts: hosts, port: port, session: session, sink: sink)
            }
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
        workers.forEach { $0.cancel() }
        ticker?.cancel()
        workers = []
        ticker = nil
        session?.invalidateAndCancel()
        session = nil
    }

    // MARK: - State and badge

    private func tick() {
        let now = ProcessInfo.processInfo.systemUptime
        let latest = mailbox.take(now: now, staleAfter: LiveStream.staleAfter)

        if isLive {
            if latest.points.isEmpty {
                leaveLive(latest.connectionEnded ? "connection lost" : "stream stopped")
            } else if now - lastBadgeAt >= LiveStream.badgeInterval || Set(latest.points.keys) != clouds.sensors {
                show(latest, at: now)
                logLiveMode()
            }
        } else if let first = latest.firstFrameAt, !latest.points.isEmpty,
                  latest.expected.isSubset(of: latest.points.keys) || now - first >= LiveStream.settle {
            // Going live waits a beat for the second sensor so the one camera fit
            // frames both clouds rather than whichever arrived first.
            enterLive(latest, at: now)
        } else if latest.connectionEnded, latest.expected.isEmpty, latest.points.isEmpty {
            requestFallback("no connection")
        } else if latest.points.isEmpty, now - startedAt >= LiveStream.fallbackAfter {
            // Frames already arriving are settling into live, not missing.
            requestFallback("no live data after \(Int(LiveStream.fallbackAfter)) s")
        }

        if now - lastRateLogAt >= LiveStream.rateLogInterval {
            lastRateLogAt = now
            logRates(latest)
        }
    }

    private func show(_ latest: LiveFrameMailbox.Snapshot, at now: TimeInterval) {
        lastBadgeAt = now
        let readings = SensorID.allCases.compactMap { sensor in
            latest.points[sensor].map { LiveViewMode.Reading(label: sensor.label, points: $0, hz: latest.renderedHz[sensor]) }
        }
        let next = LiveClouds(readings: readings, sensors: Set(latest.points.keys))
        if next != clouds {
            clouds = next
        }
    }

    /// Delivered rate (frames the renderer actually drew, 2 s window) for the
    /// operator, plus where the time goes for whoever chases a low number.
    private func logRates(_ latest: LiveFrameMailbox.Snapshot) {
        guard !latest.expected.isEmpty || !latest.points.isEmpty else { return }
        func hz(_ value: Double?) -> String { String(format: "%.1f Hz", value ?? 0) }
        func ms(_ value: Double?) -> String { value.map { String(format: "%.1f ms", $0) } ?? "–" }
        let rates = SensorID.allCases.map { "\($0.label) \(hz(latest.renderedHz[$0]))" }.joined(separator: " ")
        log("rate \(rates)", kind: .info, mission: false)

        let timings = mailbox.drainTimings()
        let stages = SensorID.allCases.map { sensor in
            "\(sensor.label) in \(hz(latest.arrivalHz[sensor])) decode \(ms(timings.decodeMs[sensor]))"
        }.joined(separator: " · ")
        log("pipeline \(stages) · upload \(ms(timings.uploadMs))", kind: .info, mission: false)
    }

    /// `mode=LIVE` on entry, and again when a sensor joins or leaves the view —
    /// going live can beat the second sensor by the settle time.
    private func logLiveMode() {
        let sensors = clouds.sensors
        guard sensors != loggedLiveSensors else { return }
        loggedLiveSensors = sensors
        log("mode=\(LiveViewMode.live.rawValue) — \(LiveViewMode.summary(clouds.readings))", kind: .info, mission: false)
    }

    private func enterLive(_ latest: LiveFrameMailbox.Snapshot, at now: TimeInterval) {
        show(latest, at: now)
        isLive = true
        fallbackRequested = false
        if !hasBeenLive {
            hasBeenLive = true
            sourceGeneration += 1
        }
        let summary = SensorID.allCases
            .compactMap { sensor in latest.points[sensor].map { "\(sensor.label) \($0) pts" } }
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

/// Hand-off from the decoders to the renderer and the main-actor ticker: the newest
/// packed cloud per sensor, overwritten in place, so a slow consumer drops frames
/// instead of queueing them. Also keeps the rate meters.
final class LiveFrameMailbox: Sendable {
    struct Snapshot: Sendable {
        /// Points per sensor with a current cloud.
        var points: [SensorID: Int] = [:]
        /// Messages per second off the socket, before any frame is dropped.
        var arrivalHz: [SensorID: Double] = [:]
        /// Frames per second the renderer actually drew.
        var renderedHz: [SensorID: Double] = [:]
        /// Sensors subscribed right now.
        var expected: Set<SensorID> = []
        /// When the current run of frames began (system uptime).
        var firstFrameAt: TimeInterval?
        /// A connection attempt ended since the last `take`.
        var connectionEnded = false
    }

    struct Entry: Sendable {
        /// Changes with every new cloud for the sensor; the renderer uploads on change.
        var sequence: Int
        var cloud: PackedCloud
    }

    struct Timings: Sendable {
        var decodeMs: [SensorID: Double] = [:]
        var uploadMs: Double?
    }

    private struct Average: Sendable {
        var total: Double = 0
        var count = 0
        var mean: Double? { count > 0 ? total / Double(count) : nil }
    }

    private struct State: Sendable {
        var entries: [SensorID: Entry] = [:]
        var sequence = 0
        var expected: Set<SensorID> = []
        var firstFrameAt: TimeInterval?
        var connectionEnded = false
        var receivedAt: [SensorID: TimeInterval] = [:]
        var arrivals: [SensorID: ArrivalRate] = [:]
        var rendered: [SensorID: ArrivalRate] = [:]
        var decode: [SensorID: Average] = [:]
        var upload = Average()
    }

    private let state = OSAllocatedUnfairLock(initialState: State())

    func put(_ cloud: PackedCloud, sensor: SensorID, at now: TimeInterval, decodeMs: Double) {
        state.withLock { state in
            if state.entries.isEmpty {
                state.firstFrameAt = now
            }
            state.sequence += 1
            state.entries[sensor] = Entry(sequence: state.sequence, cloud: cloud)
            state.receivedAt[sensor] = now
            state.decode[sensor, default: Average()].total += decodeMs
            state.decode[sensor, default: Average()].count += 1
        }
    }

    /// The renderer's pull, once per sensor per draw.
    func latest(_ sensor: SensorID) -> Entry? {
        state.withLock { $0.entries[sensor] }
    }

    /// A new cloud for the sensor went into a drawn frame.
    func noteRendered(_ sensor: SensorID, at now: TimeInterval) {
        state.withLock { $0.rendered[sensor, default: ArrivalRate()].note(now) }
    }

    func noteUpload(ms: Double) {
        state.withLock {
            $0.upload.total += ms
            $0.upload.count += 1
        }
    }

    /// Every message for the sensor, including ones a newer frame overwrote.
    func noteArrival(_ sensor: SensorID, at now: TimeInterval) {
        state.withLock { $0.arrivals[sensor, default: ArrivalRate()].note(now) }
    }

    func setExpected(_ sensor: SensorID, _ expected: Bool) {
        state.withLock { state in
            if expected {
                state.expected.insert(sensor)
            } else {
                state.expected.remove(sensor)
            }
        }
    }

    /// The sensor's topic went away; its last cloud must not linger on screen.
    func drop(_ sensor: SensorID) {
        state.withLock { Self.remove(sensor, from: &$0) }
    }

    func connectionEnded(_ sensor: SensorID) {
        state.withLock { state in
            Self.remove(sensor, from: &state)
            state.expected.remove(sensor)
            state.connectionEnded = true
        }
    }

    /// The current state, minus any sensor that has gone quiet for `staleAfter`.
    func take(now: TimeInterval, staleAfter: TimeInterval) -> Snapshot {
        state.withLock { state in
            for (sensor, at) in state.receivedAt where now - at > staleAfter {
                Self.remove(sensor, from: &state)
            }
            var snapshot = Snapshot(expected: state.expected, firstFrameAt: state.firstFrameAt, connectionEnded: state.connectionEnded)
            for (sensor, entry) in state.entries {
                snapshot.points[sensor] = entry.cloud.frame.count
            }
            for (sensor, rate) in state.arrivals {
                snapshot.arrivalHz[sensor] = rate.perSecond(now: now)
            }
            for (sensor, rate) in state.rendered {
                snapshot.renderedHz[sensor] = rate.perSecond(now: now)
            }
            state.connectionEnded = false
            return snapshot
        }
    }

    /// Mean decode and upload times since the last call.
    func drainTimings() -> Timings {
        state.withLock { state in
            var timings = Timings(uploadMs: state.upload.mean)
            for (sensor, average) in state.decode {
                timings.decodeMs[sensor] = average.mean
            }
            state.decode = [:]
            state.upload = Average()
            return timings
        }
    }

    private static func remove(_ sensor: SensorID, from state: inout State) {
        state.arrivals[sensor] = nil
        state.rendered[sensor] = nil
        state.receivedAt[sensor] = nil
        guard state.entries.removeValue(forKey: sensor) != nil else { return }
        if state.entries.isEmpty {
            state.firstFrameAt = nil
        }
    }
}

// MARK: - Socket

/// The off-main half: connect, subscribe, decode, reconnect — one sensor per socket.
private enum LiveStream {
    static let maxPointsPerSensor = 60_000
    /// Live/sample state checks. Rendering doesn't wait on this.
    static let uiInterval: TimeInterval = 0.1
    static let badgeInterval: TimeInterval = 0.5
    static let rateLogInterval: TimeInterval = 2
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
        let sensor: SensorID
        let endpoint: String
        var opened = false
        var topics: [UInt32: String] = [:]
        var channelId: UInt32?
        var subscriptionId: UInt32?
        var nextSubscriptionId: UInt32 = 1
        var skipped: Set<UInt32> = []
    }

    static func run(
        sensor: SensorID,
        hosts: @Sendable () -> [String],
        port: Int,
        session: URLSession,
        sink: Sink
    ) async {
        var rotation = StreamHostRotation()
        var announcedDown = false

        while !Task.isCancelled {
            let candidates = hosts()
            guard let host = rotation.host(in: candidates) else {
                await sink.log("no Spark host to try", .error, !announcedDown)
                announcedDown = true
                sink.mailbox.connectionEnded(sensor)
                try? await Task.sleep(nanoseconds: 30_000_000_000)
                continue
            }
            let endpoint = "ws://\(host):\(port)"
            await sink.log("connecting \(endpoint) (\(sensor.label))", .info, !announcedDown)

            let end = await connect(sensor: sensor, endpoint: endpoint, session: session, sink: sink)
            sink.mailbox.connectionEnded(sensor)
            if Task.isCancelled { break }

            if end.opened {
                announcedDown = false
                await sink.log("connection lost \(endpoint) (\(sensor.label)) — \(end.reason)", .info, true)
            } else {
                await sink.log("\(endpoint) unreachable (\(sensor.label)) — \(end.reason); retrying", .info, !announcedDown)
                announcedDown = true
            }

            let delay = rotation.record(opened: end.opened, candidateCount: candidates.count)
            if delay > 0 {
                try? await Task.sleep(nanoseconds: UInt64(delay * 1_000_000_000))
            }
        }
    }

    private static func connect(sensor: SensorID, endpoint: String, session: URLSession, sink: Sink) async -> ConnectionEnd {
        guard let url = URL(string: endpoint) else {
            return ConnectionEnd(opened: false, reason: "bad URL")
        }
        let socket = session.webSocketTask(with: url, protocols: FoxgloveProtocol.subprotocols)
        socket.maximumMessageSize = maxMessageBytes
        let health = OSAllocatedUnfairLock(initialState: SocketHealth())
        socket.resume()

        // Latest-only: a payload still waiting when the next arrives is replaced,
        // so a slow decode drops stale frames and the receive loop never waits.
        let (payloads, slot) = AsyncStream.makeStream(of: Data.self, bufferingPolicy: .bufferingNewest(1))
        let decoder = Task.detached(priority: .userInitiated) {
            await decode(payloads, sensor: sensor, sink: sink)
        }
        let monitor = Task { await watch(socket, health: health) }
        defer {
            monitor.cancel()
            socket.cancel(with: .goingAway, reason: nil)
        }

        // `receive()` doesn't notice task cancellation on its own.
        let end = await withTaskCancellationHandler {
            await receive(from: socket, sensor: sensor, endpoint: endpoint, health: health, slot: slot, sink: sink)
        } onCancel: {
            socket.cancel(with: .goingAway, reason: nil)
        }
        // Let an in-flight decode land before the caller clears this sensor, so a
        // dead connection's last frame can't reappear after it.
        slot.finish()
        decoder.cancel()
        await decoder.value
        return end
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
        sensor: SensorID,
        endpoint: String,
        health: OSAllocatedUnfairLock<SocketHealth>,
        slot: AsyncStream<Data>.Continuation,
        sink: Sink
    ) async -> ConnectionEnd {
        var link = Link(sensor: sensor, endpoint: endpoint)
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
                    // Header parse only; the decoder takes it from here.
                    guard let data = FoxgloveProtocol.parseMessageData(frame),
                          let subscriptionId = link.subscriptionId,
                          data.subscriptionId == subscriptionId else {
                        continue
                    }
                    sink.mailbox.noteArrival(sensor, at: ProcessInfo.processInfo.systemUptime)
                    slot.yield(data.payload)
                @unknown default:
                    break
                }
            }
        } catch {
            let reason = health.withLock { $0.failure } ?? describe(error, socket: socket)
            return ConnectionEnd(opened: link.opened, reason: reason)
        }
    }

    /// Decode → `world` → GPU layout, one frame at a time, newest first.
    private static func decode(_ payloads: AsyncStream<Data>, sensor: SensorID, sink: Sink) async {
        var reported = false
        var undecodable = false
        for await payload in payloads {
            let started = ProcessInfo.processInfo.systemUptime
            let decoded: PointCloudFrame
            do {
                decoded = try PointCloud2CDR.decode(payload, maxPoints: maxPointsPerSensor)
            } catch {
                if !undecodable {
                    undecodable = true
                    await sink.log("\(sensor.label) frame not decodable — \(error)", .error, true)
                }
                continue
            }
            guard !decoded.isEmpty, !Task.isCancelled else { continue }

            let world = RigTransforms.inWorld(decoded, sensor: sensor)
            let cloud = PackedCloud(world)
            let now = ProcessInfo.processInfo.systemUptime
            sink.mailbox.put(cloud, sensor: sensor, at: now, decodeMs: (now - started) * 1000)

            if !reported {
                reported = true
                let sourceFrame = decoded.frameId.flatMap { $0.isEmpty ? nil : $0 } ?? "no frame_id"
                await sink.log(
                    "\(sensor.label) first frame: \(world.count) pts shown (\(sourceFrame) → \(RigTransforms.worldFrame), \(payload.count) bytes on the wire)",
                    .info,
                    true
                )
            }
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
        await sink.log("connected \(link.endpoint)\(server) for \(link.sensor.label)", .info, true)
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
                guard channelId == link.channelId else { continue }
                // Forget the subscription so a re-advertised topic starts clean.
                link.channelId = nil
                link.subscriptionId = nil
                sink.mailbox.drop(link.sensor)
                sink.mailbox.setExpected(link.sensor, false)
                await sink.log("\(topic ?? "channel \(channelId)") unadvertised — \(link.sensor.label) stream ended", .info, true)
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
        // One channel for this socket's sensor is plenty.
        guard link.channelId == nil else { return }

        for channel in channels where FoxgloveProtocol.liveSensor(forTopic: channel.topic) == link.sensor {
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

            let subscriptionId = link.nextSubscriptionId
            link.nextSubscriptionId += 1
            try await socket.send(.string(FoxgloveProtocol.subscribeMessage([
                FoxgloveProtocol.Subscription(id: subscriptionId, channelId: channel.id),
            ])))
            link.channelId = channel.id
            link.subscriptionId = subscriptionId
            sink.mailbox.setExpected(link.sensor, true)

            let schema = channel.schemaName.isEmpty ? "schema unnamed" : channel.schemaName
            await sink.log("subscribed \(channel.topic) → \(link.sensor.label) (\(schema), channel \(channel.id), sub \(subscriptionId))", .info, true)
            return
        }
    }
}
