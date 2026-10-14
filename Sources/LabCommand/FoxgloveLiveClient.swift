import Foundation
import Combine
import os
import LabCommandCore

/// Lidar clouds for the 3D view, straight from foxglove_bridge on Spark
/// (`ws://<spark>:8765`, subprotocol `foxglove.sdk.v1` or `foxglove.websocket.v1`).
///
/// Nothing is opened until the operator asks: Snapshot takes one frame per sensor
/// and closes, Live streams for at most `LiveSession.liveMaxDuration`. When to
/// open and close is `LiveSession`'s call; this object opens exactly the sockets it
/// names. One socket per sensor, so a 3 MB JT128 frame never queues a Mid-360 frame
/// behind it. Each socket's receive loop only parses headers and drops the raw
/// payload into a latest-only slot; a decoder task per sensor decodes, moves the
/// cloud into `world` and packs it for the GPU. The renderer pulls the newest
/// packed cloud on every draw, so neither SwiftUI nor a polling timer sits between
/// the socket and the screen. This object only drives the mode and the badge, and
/// is its own `ObservableObject` so those updates re-render the canvas, not
/// everything that watches `AppState`.
@MainActor
final class FoxgloveLiveClient: ObservableObject {
    @Published private(set) var phase: LiveSession.Phase = .off
    /// Badge text for every phase but OFF, whose badge depends on the sample.
    @Published private(set) var badge = ""
    /// The view refits the camera once per value — so never per frame, and never
    /// under the operator's hands mid-session.
    @Published private(set) var fitGeneration = 0

    /// False in mock mode, which is for working without Spark.
    let isAvailable: Bool
    /// Newest packed cloud per sensor, for the renderer to pull.
    let mailbox = LiveFrameMailbox()

    /// Mission-log lines (`[FoxgloveLive] …`): mode changes, connection state,
    /// subscriptions, first-frame point counts.
    var onLog: ((String, MissionEventKind) -> Void)?
    /// The first time real clouds are on screen.
    var onCaptured: (() -> Void)?

    var isLive: Bool { if case .live = phase { return true } else { return false } }
    var isCapturing: Bool { if case .snapshotting = phase { return true } else { return false } }

    private struct Worker {
        let id: Int
        let task: Task<Void, Never>
    }

    private var machine = LiveSession()
    private let hosts: @Sendable () -> [String]
    private let port: Int
    /// Snapshot launches never open a web socket, even if a caller asks.
    private let isolates: Bool
    private var session: URLSession?
    private var workers: [SensorID: Worker] = [:]
    private var nextWorkerId = 0
    private var ticker: Task<Void, Never>?
    private var latest = LiveFrameMailbox.Snapshot()
    private var lastBadgeAt: TimeInterval = 0
    private var lastRateLogAt: TimeInterval = 0
    private var captured = false
    /// Sensors the last live summary line named.
    private var loggedLiveSensors: Set<SensorID> = []

    /// - Parameter hosts: Candidates for `ws://<host>:<port>`, re-read before every
    ///   attempt so a hop that SSH discovers later is picked up.
    init(
        isAvailable: Bool = true,
        isolates: Bool = false,
        hosts: @escaping @Sendable () -> [String] = {
            FoxgloveProtocol.streamHosts(lastLAN: SparkHopCache.load())
        },
        port: Int = CaptureRecipe.foxglovePort
    ) {
        self.isAvailable = isAvailable
        self.isolates = isolates
        self.hosts = hosts
        self.port = port
        log("mode=\(LiveViewMode.off.rawValue) — no stream until Snapshot or Live", kind: .info, mission: false)
    }

    // MARK: - Requests

    func takeSnapshot() {
        guard isAvailable else { return }
        report(machine.requestSnapshot(at: Self.now))
        sync()
    }

    func startLive() {
        guard isAvailable else { return }
        report(machine.requestLive(at: Self.now))
        sync()
    }

    func stopLive() {
        endLive(.requested)
    }

    /// The window was hidden, minimized or covered, or the app hidden.
    func viewHidden() {
        endLive(.hidden)
    }

    /// App teardown: close whatever is open, whatever the phase.
    func shutdown() {
        ticker?.cancel()
        ticker = nil
        workers.values.forEach { $0.task.cancel() }
        workers = [:]
        session?.invalidateAndCancel()
        session = nil
    }

    private func endLive(_ reason: LiveSession.StopReason) {
        guard machine.isLive else { return }
        report(observeMailbox(at: Self.now))
        report(machine.stopLive(reason))
        sync()
    }

    // MARK: - Session plumbing

    private static var now: TimeInterval { ProcessInfo.processInfo.systemUptime }

    private func tick() {
        let now = Self.now
        report(observeMailbox(at: now))
        report(machine.tick(at: now))
        if machine.isLive, now - lastRateLogAt >= LiveStream.rateLogInterval {
            lastRateLogAt = now
            logRates()
        }
        sync()
    }

    private func workerEnded(_ sensor: SensorID, id: Int) {
        guard workers[sensor]?.id == id else { return }
        workers[sensor] = nil
        // A snapshot socket closes only after its frame is in the mailbox.
        report(observeMailbox(at: Self.now))
        report(machine.sensorEnded(sensor))
        sync()
    }

    private func observeMailbox(at now: TimeInterval) -> LiveSession.Outcome? {
        guard machine.isCapturing || machine.isLive else { return nil }
        // Live drops a sensor gone quiet; a snapshot's frames are meant to sit still.
        latest = mailbox.take(now: now, staleAfter: machine.isLive ? LiveStream.staleAfter : nil)
        return machine.observe(points: latest.points, newest: latest.newest, at: now)
    }

    /// Open exactly the sockets the session names, close the rest, and publish.
    private func sync() {
        let wanted = machine.sockets
        for (sensor, worker) in workers where !wanted.contains(sensor) {
            worker.task.cancel()
            workers[sensor] = nil
        }
        for sensor in SensorID.allCases where wanted.contains(sensor) && workers[sensor] == nil {
            spawn(sensor, oneFrame: machine.oneFrame)
        }
        if workers.isEmpty, let session {
            session.invalidateAndCancel()
            self.session = nil
        }

        let running = machine.isCapturing || machine.isLive
        if running, ticker == nil {
            ticker = Task { [weak self] in
                while !Task.isCancelled {
                    try? await Task.sleep(nanoseconds: UInt64(LiveStream.uiInterval * 1_000_000_000))
                    guard let self, !Task.isCancelled else { return }
                    self.tick()
                }
            }
        } else if !running {
            ticker?.cancel()
            ticker = nil
        }

        publish()
    }

    private func spawn(_ sensor: SensorID, oneFrame: Bool) {
        if isolates { return }
        let session = self.session ?? makeSession()
        self.session = session
        nextWorkerId += 1
        let id = nextWorkerId
        let sink = LiveStream.Sink(mailbox: mailbox, epoch: mailbox.epoch) { [weak self] text, kind, mission in
            self?.log(text, kind: kind, mission: mission)
        }
        let hosts = self.hosts
        let port = self.port
        let isolates = self.isolates
        let task = Task.detached(priority: .userInitiated) { [weak self] in
            await LiveStream.run(sensor: sensor, hosts: hosts, port: port, session: session, oneFrame: oneFrame, isolates: isolates, sink: sink)
            await self?.workerEnded(sensor, id: id)
        }
        workers[sensor] = Worker(id: id, task: task)
    }

    private func makeSession() -> URLSession {
        let config = URLSessionConfiguration.ephemeral
        config.waitsForConnectivity = false
        // The stream's own monitor owns timeouts; URLSession's request timeout would
        // also cut a healthy socket that is merely quiet because the sensors are dark.
        config.timeoutIntervalForRequest = 24 * 60 * 60
        return URLSession(configuration: config)
    }

    // MARK: - Mode and badge

    private func report(_ outcome: LiveSession.Outcome?) {
        guard let outcome else { return }
        switch outcome {
        case .snapshotStarted:
            mailbox.reset()
            log("snapshot: one frame per sensor (\(FoxgloveProtocol.mid360Topic), \(FoxgloveProtocol.jt128Topic)), \(Int(LiveSession.snapshotTimeout)) s limit", kind: .info, mission: true)
        case .snapshotTaken(let capturedAt, let missing):
            let held = mailbox.seal(keeping: Set(machine.points.keys))
            let line = "\(LiveViewMode.clock(capturedAt)) — \(LiveViewMode.snapshotSummary(held))"
            log("snapshot \(line)\(missing.isEmpty ? "" : " (no frame within \(Int(LiveSession.snapshotTimeout)) s)")", kind: .info, mission: true)
            log("mode=\(LiveViewMode.snapshot.rawValue) \(line)", kind: .info, mission: false)
        case .snapshotEmpty:
            mailbox.seal(keeping: [])
            log("snapshot: no frame from either sensor within \(Int(LiveSession.snapshotTimeout)) s", kind: .error, mission: true)
            log("mode=\(LiveViewMode.off.rawValue)", kind: .info, mission: false)
        case .liveStarted:
            mailbox.reset()
            loggedLiveSensors = []
            lastRateLogAt = Self.now
            log("live: streaming for up to \(Int(LiveSession.liveMaxDuration)) s", kind: .info, mission: true)
            log("mode=\(LiveViewMode.live.rawValue)", kind: .info, mission: false)
        case .liveStopped(let reason, let heldAt):
            let held = mailbox.seal(keeping: Set(machine.points.keys))
            log("live stopped (\(reason.rawValue))", kind: .info, mission: true)
            if let heldAt {
                log("mode=\(LiveViewMode.snapshot.rawValue) \(LiveViewMode.clock(heldAt)) — \(LiveViewMode.snapshotSummary(held)) (last live frames)", kind: .info, mission: false)
            } else {
                log("mode=\(LiveViewMode.off.rawValue)", kind: .info, mission: false)
            }
        }
    }

    private func publish() {
        let now = Self.now
        if phase != machine.phase {
            phase = machine.phase
            lastBadgeAt = 0
        }
        if fitGeneration != machine.fitGeneration {
            fitGeneration = machine.fitGeneration
            if machine.mode != .off, !captured {
                captured = true
                onCaptured?()
            }
        }

        let next: String
        switch machine.phase {
        case .off:
            next = ""
        case .snapshotting:
            next = LiveViewMode.capturingBadge(readings(hz: false))
        case .snapshot(let capturedAt):
            // What the renderer draws: the mailbox is sealed once a session ends.
            next = LiveViewMode.snapshotBadge(capturedAt: capturedAt, points: mailbox.take(now: now, staleAfter: nil).points)
        case .live:
            let sensors = Set(latest.points.keys)
            // Hz and the countdown move every tick; half a second is plenty.
            guard now - lastBadgeAt >= LiveStream.badgeInterval || sensors != loggedLiveSensors else { return }
            lastBadgeAt = now
            let readings = readings(hz: true)
            if sensors != loggedLiveSensors {
                loggedLiveSensors = sensors
                if !readings.isEmpty {
                    log("live: \(LiveViewMode.summary(readings))", kind: .info, mission: false)
                }
            }
            next = LiveViewMode.liveBadge(readings, remaining: machine.remaining(at: now))
        }
        if badge != next {
            badge = next
        }
    }

    private func readings(hz: Bool) -> [LiveViewMode.Reading] {
        SensorID.allCases.compactMap { sensor in
            latest.points[sensor].map { LiveViewMode.Reading(label: sensor.label, points: $0, hz: hz ? latest.renderedHz[sensor] : nil) }
        }
    }

    /// Delivered rate (frames the renderer actually drew, 2 s window) for the
    /// operator, plus where the time goes for whoever chases a low number.
    private func logRates() {
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
///
/// Each session writes under its own epoch. A decoder still finishing for a
/// session that ended can't touch the next one's frames, nor a sealed snapshot.
final class LiveFrameMailbox: Sendable {
    struct Snapshot: Sendable {
        /// Points per sensor with a current cloud.
        var points: [SensorID: Int] = [:]
        /// When the newest cloud on hand arrived (wall clock).
        var newest: Date?
        /// Messages per second off the socket, before any frame is dropped.
        var arrivalHz: [SensorID: Double] = [:]
        /// Frames per second the renderer actually drew.
        var renderedHz: [SensorID: Double] = [:]
    }

    struct Entry: Sendable {
        /// Changes with every new cloud for the sensor; the renderer uploads on change.
        var sequence: Int
        var cloud: PackedCloud
        var arrivedAt: Date
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
        var epoch = 0
        var entries: [SensorID: Entry] = [:]
        /// Never reset: the renderer skips an entry whose sequence it already drew.
        var sequence = 0
        var receivedAt: [SensorID: TimeInterval] = [:]
        var arrivals: [SensorID: ArrivalRate] = [:]
        var rendered: [SensorID: ArrivalRate] = [:]
        var decode: [SensorID: Average] = [:]
        var upload = Average()
    }

    private let state = OSAllocatedUnfairLock(initialState: State())

    /// Writers stamp their puts with this; a different epoch is ignored.
    var epoch: Int { state.withLock { $0.epoch } }

    /// Clear everything for a new session.
    func reset() {
        state.withLock { state in
            state.epoch += 1
            state.entries = [:]
            state.receivedAt = [:]
            state.arrivals = [:]
            state.rendered = [:]
            state.decode = [:]
            state.upload = Average()
        }
    }

    /// Freeze the frames on hand (only `sensors`') for a held view: later writes
    /// from the ended session are ignored. Returns what stays.
    @discardableResult
    func seal(keeping sensors: Set<SensorID>) -> [SensorID: Int] {
        state.withLock { state in
            state.epoch += 1
            for sensor in SensorID.allCases where !sensors.contains(sensor) {
                Self.remove(sensor, from: &state)
            }
            return state.entries.mapValues { $0.cloud.frame.count }
        }
    }

    func put(_ cloud: PackedCloud, sensor: SensorID, epoch: Int, at now: TimeInterval, decodeMs: Double) {
        state.withLock { state in
            guard state.epoch == epoch else { return }
            state.sequence += 1
            state.entries[sensor] = Entry(sequence: state.sequence, cloud: cloud, arrivedAt: Date())
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
    func noteArrival(_ sensor: SensorID, epoch: Int, at now: TimeInterval) {
        state.withLock { state in
            guard state.epoch == epoch else { return }
            state.arrivals[sensor, default: ArrivalRate()].note(now)
        }
    }

    /// The sensor's topic went away; its last cloud must not linger on screen.
    func drop(_ sensor: SensorID, epoch: Int) {
        state.withLock { state in
            guard state.epoch == epoch else { return }
            Self.remove(sensor, from: &state)
        }
    }

    /// The current state, minus any sensor that has gone quiet for `staleAfter`
    /// (nil keeps every frame, however old).
    func take(now: TimeInterval, staleAfter: TimeInterval?) -> Snapshot {
        state.withLock { state in
            if let staleAfter {
                for (sensor, at) in state.receivedAt where now - at > staleAfter {
                    Self.remove(sensor, from: &state)
                }
            }
            var snapshot = Snapshot()
            for (sensor, entry) in state.entries {
                snapshot.points[sensor] = entry.cloud.frame.count
                snapshot.newest = max(snapshot.newest ?? entry.arrivedAt, entry.arrivedAt)
            }
            for (sensor, rate) in state.arrivals {
                snapshot.arrivalHz[sensor] = rate.perSecond(now: now)
            }
            for (sensor, rate) in state.rendered {
                snapshot.renderedHz[sensor] = rate.perSecond(now: now)
            }
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
        state.entries[sensor] = nil
        state.arrivals[sensor] = nil
        state.rendered[sensor] = nil
        state.receivedAt[sensor] = nil
    }
}

// MARK: - Socket

/// The off-main half: connect, subscribe, decode, reconnect — one sensor per socket.
private enum LiveStream {
    static let maxPointsPerSensor = 60_000
    /// Session checks (timeouts, badge). Rendering doesn't wait on this.
    static let uiInterval: TimeInterval = 0.1
    static let badgeInterval: TimeInterval = 0.5
    static let rateLogInterval: TimeInterval = 2
    /// Live drops a sensor's cloud after this long without a new one.
    static let staleAfter: TimeInterval = 3
    static let handshakeTimeout: TimeInterval = 5
    static let pingInterval: TimeInterval = 10
    /// URLSession's default cap is 1 MiB; one JT128 PointCloud2 is several.
    static let maxMessageBytes = 64 << 20

    struct Sink: Sendable {
        let mailbox: LiveFrameMailbox
        /// The session this socket serves; the mailbox ignores other sessions' writes.
        let epoch: Int
        /// (text, kind, alsoMissionLog). Everything goes to stderr; only state changes
        /// reach the mission log, so an offline Spark can't flood it with retries.
        let log: @MainActor @Sendable (String, MissionEventKind, Bool) -> Void
    }

    private struct ConnectionEnd {
        var opened: Bool
        var reason: String
        /// The snapshot frame was handed to the decoder; this sensor is done.
        var tookFrame = false
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

    /// Until cancelled — or, with `oneFrame`, until one frame has been taken.
    static func run(
        sensor: SensorID,
        hosts: @Sendable () -> [String],
        port: Int,
        session: URLSession,
        oneFrame: Bool,
        isolates: Bool,
        sink: Sink
    ) async {
        var rotation = StreamHostRotation()
        var announcedDown = false

        while !Task.isCancelled {
            let candidates = hosts()
            guard let host = rotation.host(in: candidates) else {
                await sink.log("no Spark host to try", .error, !announcedDown)
                announcedDown = true
                try? await Task.sleep(nanoseconds: 30_000_000_000)
                continue
            }
            let endpoint = "ws://\(host):\(port)"
            await sink.log("connecting \(endpoint) (\(sensor.label))", .info, !announcedDown)

            let end = await connect(sensor: sensor, endpoint: endpoint, session: session, oneFrame: oneFrame, isolates: isolates, sink: sink)
            if end.tookFrame {
                await sink.log("closed \(endpoint) (\(sensor.label)) — \(end.reason)", .info, false)
                return
            }
            if Task.isCancelled {
                await sink.log("closed \(endpoint) (\(sensor.label))", .info, false)
                break
            }

            if end.opened {
                announcedDown = false
                await sink.log("connection lost \(endpoint) (\(sensor.label)) — \(end.reason)", .info, true)
            } else {
                await sink.log("\(endpoint) unreachable (\(sensor.label)) — \(end.reason); retrying", .info, !announcedDown)
                announcedDown = true
            }

            let delay = rotation.record(opened: end.opened, in: candidates)
            if delay > 0 {
                try? await Task.sleep(nanoseconds: UInt64(delay * 1_000_000_000))
            }
        }
    }

    private static func connect(sensor: SensorID, endpoint: String, session: URLSession, oneFrame: Bool, isolates: Bool, sink: Sink) async -> ConnectionEnd {
        guard let url = URL(string: endpoint) else {
            return ConnectionEnd(opened: false, reason: "bad URL")
        }
        let socket: URLSessionWebSocketTask
        do {
            socket = try ContactGate.attempt(.socket, isolates: isolates, detail: endpoint) {
                session.webSocketTask(with: url, protocols: FoxgloveProtocol.subprotocols)
            }
        } catch {
            return ConnectionEnd(opened: false, reason: "local session")
        }
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
            await receive(from: socket, sensor: sensor, endpoint: endpoint, oneFrame: oneFrame, health: health, slot: slot, sink: sink)
        } onCancel: {
            socket.cancel(with: .goingAway, reason: nil)
        }
        slot.finish()
        // The snapshot frame is still in the slot: let it decode. Otherwise drop
        // what's queued, and let an in-flight decode land before the caller moves
        // on, so a dead connection's last frame can't reappear after it.
        if !end.tookFrame {
            decoder.cancel()
        }
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
        oneFrame: Bool,
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
                    sink.mailbox.noteArrival(sensor, epoch: sink.epoch, at: ProcessInfo.processInfo.systemUptime)
                    if oneFrame {
                        // Stop the server sending more before the frame lands: once it
                        // does, the socket closes. The frame counts even if this fails.
                        let unsubscribed = (try? await socket.send(.string(FoxgloveProtocol.unsubscribeMessage([subscriptionId])))) != nil
                        slot.yield(data.payload)
                        return ConnectionEnd(
                            opened: true,
                            reason: unsubscribed ? "snapshot frame taken, unsubscribed" : "snapshot frame taken",
                            tookFrame: true
                        )
                    }
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
            sink.mailbox.put(cloud, sensor: sensor, epoch: sink.epoch, at: now, decodeMs: (now - started) * 1000)

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
                sink.mailbox.drop(link.sensor, epoch: sink.epoch)
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

            let schema = channel.schemaName.isEmpty ? "schema unnamed" : channel.schemaName
            await sink.log("subscribed \(channel.topic) → \(link.sensor.label) (\(schema), channel \(channel.id), sub \(subscriptionId))", .info, true)
            return
        }
    }
}
