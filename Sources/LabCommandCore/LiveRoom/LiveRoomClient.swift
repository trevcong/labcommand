import Combine
import Foundation

/// The bridge as this connection found it.
public struct LiveRoomServer: Sendable, Equatable {
    public var name: String
    public var capabilities: [String]
    /// The bridge accepts client channels and this client may publish, so `RequestSync` goes over this socket.
    public var canPublish: Bool
    /// `/room_memory/*` topics advertised and subscribed. Empty: the bridge is up but room
    /// memory is not running ("Room memory not running on Spark", never an empty room).
    public var memoryTopics: [String]

    public init(name: String, capabilities: [String], canPublish: Bool, memoryTopics: [String]) {
        self.name = name
        self.capabilities = capabilities
        self.canPublish = canPublish
        self.memoryTopics = memoryTopics
    }
}

public enum LiveRoomConnection: Sendable, Equatable {
    case off
    case connecting(url: URL, attempt: Int)
    case connected(LiveRoomServer)
    /// Between attempts.
    case waiting(retryIn: TimeInterval, reason: String)
}

/// The live room-memory link: one Foxglove WebSocket to the Spark's room-memory bridge, feeding
/// a `RoomMemoryMirror`. Bind SwiftUI to `connection` and `mirror` (see `LiveRoom/API.md`).
@MainActor
public final class LiveRoomClient: ObservableObject {
    /// The room-memory bridge on the Spark's tailnet IP (Phase 3 P12).
    public nonisolated static let defaultURL = URL(string: "ws://100.118.187.99:8767")!
    /// No `serverInfo` by then: give up on the attempt.
    public static let handshakeTimeout: TimeInterval = 5
    /// No message for this long (status is 1 Hz): the link is dead; reconnect.
    public static let silenceTimeout: TimeInterval = 10
    /// Reconnect backoff floor (Phase 3 P3).
    public static let reconnectFloor: TimeInterval = 5
    /// The client channel `RequestSync` goes out on, when the bridge allows client publishing.
    public static let requestChannelId: UInt32 = 1
    public static let requestTopic = "/room_memory/request_sync"
    public static let requestSchemaName = "labcommand_msgs/msg/RequestSync"
    /// URLSession's default cap is 1 MiB; a snapshot may reach 16 MiB raw (D22).
    static let maxMessageBytes = 64 << 20
    static let logCapacity = 200

    @Published public private(set) var connection: LiveRoomConnection = .off
    @Published public private(set) var mirror: RoomMemoryMirror
    /// Diagnostics, newest last.
    @Published public private(set) var log: [String] = []

    /// Another path for `RequestSync` (the `:8766` relay, Phase 3 P11). Return true when sent.
    /// Used when the bridge does not offer `clientPublish`. Set it before `start()`.
    public var requestSyncSink: ((RoomMemory.RequestSync) -> Bool)?

    public let url: URL
    /// False keeps the client read-only: it never advertises a channel or sends a request.
    public let publishRequests: Bool
    private let session: URLSession
    private var runTask: Task<Void, Never>?
    private var tickTask: Task<Void, Never>?
    /// Bumped by `start()`; a cancelled run that is still unwinding must not touch the new one.
    private var runGeneration = 0
    private var link: Link?

    /// Per-connection socket state.
    private struct Link {
        let socket: URLSessionWebSocketTask
        let startedAt: TimeInterval
        var lastMessageAt: TimeInterval
        var server: LiveRoomServer?
        var subscriptions: [UInt32: LiveRoomTopic] = [:]
        /// channel id → subscription id
        var channels: [UInt32: UInt32] = [:]
        var nextSubscriptionId: UInt32 = 1
        var mirrorOpened = false
        var failure: String?
    }

    public init(url: URL = LiveRoomClient.defaultURL, roomId: String? = nil, publishRequests: Bool = true) {
        self.url = url
        self.publishRequests = publishRequests
        mirror = RoomMemoryMirror(roomId: roomId)
        let config = URLSessionConfiguration.ephemeral
        config.waitsForConnectivity = false
        // The silence watchdog owns timeouts; a request timeout would cut a quiet, healthy socket.
        config.timeoutIntervalForRequest = 24 * 60 * 60
        session = URLSession(configuration: config)
    }

    deinit {
        runTask?.cancel()
        tickTask?.cancel()
    }

    /// Connects, and reconnects with backoff until `stop()`.
    public func start() {
        guard runTask == nil else { return }
        runGeneration += 1
        let generation = runGeneration
        runTask = Task { [weak self] in await self?.run(generation: generation) }
        tickTask = Task { [weak self] in
            while !Task.isCancelled {
                try? await Task.sleep(nanoseconds: 1_000_000_000)
                self?.tick()
            }
        }
    }

    /// Closes the socket. The mirror stays, greyed: nothing proves it current any more.
    public func stop() {
        runTask?.cancel()
        tickTask?.cancel()
        runTask = nil
        tickTask = nil
        if let current = link {
            // Close now, so a quick start() never meets this connection's leftovers.
            current.socket.cancel(with: .normalClosure, reason: nil)
            if current.mirrorOpened { mirror.connectionClosed(at: Self.now) }
            link = nil
        }
        connection = .off
    }

    private static var now: TimeInterval { ProcessInfo.processInfo.systemUptime }

    // MARK: Connection loop

    private func run(generation: Int) async {
        var attempt = 0
        var failures = 0
        while !Task.isCancelled, generation == runGeneration {
            attempt += 1
            connection = .connecting(url: url, attempt: attempt)
            let (opened, reason) = await connectOnce()
            if Task.isCancelled || generation != runGeneration { return }
            failures = opened ? 1 : failures + 1
            let delay = max(Self.reconnectFloor, FoxgloveProtocol.reconnectDelay(afterFailures: failures))
            note(opened ? "connection lost: \(reason); retrying in \(Int(delay)) s" : "\(url) unreachable: \(reason); retrying in \(Int(delay)) s")
            connection = .waiting(retryIn: delay, reason: reason)
            try? await Task.sleep(nanoseconds: UInt64(delay * 1_000_000_000))
        }
    }

    private func connectOnce() async -> (opened: Bool, reason: String) {
        let socket = session.webSocketTask(with: url, protocols: FoxgloveProtocol.subprotocols)
        socket.maximumMessageSize = Self.maxMessageBytes
        link = Link(socket: socket, startedAt: Self.now, lastMessageAt: Self.now)
        socket.resume()

        /// This connection is still the client's; a stopped one only unwinds.
        func isCurrent() -> Bool { link?.socket === socket }

        var reason = "closed"
        do {
            while !Task.isCancelled, isCurrent() {
                let message = try await withTaskCancellationHandler {
                    try await socket.receive()
                } onCancel: {
                    socket.cancel(with: .normalClosure, reason: nil)
                }
                guard isCurrent() else { break }
                link?.lastMessageAt = Self.now
                switch message {
                case .string(let text): handle(text: text)
                case .data(let frame): handle(frame: frame)
                @unknown default: break
                }
            }
        } catch {
            reason = (isCurrent() ? link?.failure : nil) ?? Self.describe(error, socket: socket)
        }
        socket.cancel(with: .goingAway, reason: nil)
        guard isCurrent() else { return (false, "stopped") }
        let opened = link?.server != nil
        if link?.mirrorOpened == true {
            mirror.connectionClosed(at: Self.now)
        }
        link = nil
        return (opened, reason)
    }

    private func tick() {
        let now = Self.now
        if let link {
            // A half-dead link would sit in receive() forever.
            if link.server == nil, now - link.startedAt > Self.handshakeTimeout {
                fail("no serverInfo within \(Int(Self.handshakeTimeout)) s")
            } else if link.server != nil, now - link.lastMessageAt > Self.silenceTimeout {
                fail("no message for \(Int(Self.silenceTimeout)) s")
            }
        }
        perform(mirror.tick(at: now))
    }

    private func fail(_ reason: String) {
        link?.failure = reason
        link?.socket.cancel(with: .goingAway, reason: nil)
    }

    private static func describe(_ error: Error, socket: URLSessionWebSocketTask) -> String {
        if socket.closeCode != .invalid {
            return "server closed the socket (code \(socket.closeCode.rawValue))"
        }
        if let http = socket.response as? HTTPURLResponse, http.statusCode != 101 {
            return "handshake refused (HTTP \(http.statusCode))"
        }
        return error.localizedDescription
    }

    // MARK: Frames

    private func handle(text: String) {
        if let capabilities = FoxgloveProtocol.serverCapabilities(text) {
            let name: String
            if case .serverInfo(let serverName) = FoxgloveProtocol.parseServerMessage(text) {
                name = serverName
            } else {
                name = ""
            }
            let canPublish = publishRequests && capabilities.contains(FoxgloveProtocol.clientPublishCapability)
            let server = LiveRoomServer(name: name, capabilities: capabilities, canPublish: canPublish, memoryTopics: [])
            link?.server = server
            connection = .connected(server)
            note("connected \(url) (capabilities: \(capabilities.joined(separator: ", ")))")
            if canPublish {
                send(.string(FoxgloveProtocol.clientAdvertiseMessage(
                    channelId: Self.requestChannelId,
                    topic: Self.requestTopic,
                    schemaName: Self.requestSchemaName
                )))
            } else if publishRequests {
                note(requestSyncSink == nil
                    ? "bridge has no clientPublish and no relay is set: RequestSync cannot be sent; relying on live snapshots"
                    : "bridge has no clientPublish: RequestSync goes through the relay")
            }
            return
        }
        switch FoxgloveProtocol.parseServerMessage(text) {
        case .advertise(let channels):
            subscribe(to: channels)
        case .unadvertise(let channelIds):
            guard var current = link else { return }
            var dropped: [UInt32] = []
            for channelId in channelIds {
                if let subscriptionId = current.channels.removeValue(forKey: channelId) {
                    current.subscriptions[subscriptionId] = nil
                    dropped.append(subscriptionId)
                }
            }
            link = current
            if !dropped.isEmpty {
                send(.string(FoxgloveProtocol.unsubscribeMessage(dropped)))
                publishTopics()
            }
        case .status(let level, let message):
            note("server status (level \(level)): \(message)")
        case .serverInfo, .other, nil:
            break
        }
    }

    private func subscribe(to channels: [FoxgloveProtocol.Channel]) {
        guard var current = link else { return }
        var subscriptions: [FoxgloveProtocol.Subscription] = []
        for channel in channels where current.channels[channel.id] == nil {
            guard let topic = LiveRoomTopic(channel: channel) else {
                if channel.topic.hasPrefix("/room_memory/"), LiveRoomTopic(rawValue: channel.topic) != nil {
                    note("\(channel.topic) advertised as \(channel.schemaName) (\(channel.encoding)); not subscribed")
                }
                continue
            }
            // One subscription per topic; an unadvertised one frees its topic for a new channel id.
            if current.subscriptions.values.contains(topic) { continue }
            let subscriptionId = current.nextSubscriptionId
            current.nextSubscriptionId += 1
            current.channels[channel.id] = subscriptionId
            current.subscriptions[subscriptionId] = topic
            subscriptions.append(FoxgloveProtocol.Subscription(id: subscriptionId, channelId: channel.id))
        }
        link = current
        guard !subscriptions.isEmpty else { return }
        send(.string(FoxgloveProtocol.subscribeMessage(subscriptions)))
        publishTopics()
        if link?.mirrorOpened == false {
            link?.mirrorOpened = true
            let canRequest = publishRequests && ((link?.server?.canPublish ?? false) || requestSyncSink != nil)
            perform(mirror.connectionOpened(at: Self.now, canRequest: canRequest))
        }
    }

    private func publishTopics() {
        guard let current = link, var server = current.server else { return }
        server.memoryTopics = current.subscriptions.values.map(\.rawValue).sorted()
        link?.server = server
        connection = .connected(server)
    }

    private func handle(frame: Data) {
        guard let data = FoxgloveProtocol.parseMessageData(frame),
              let topic = link?.subscriptions[data.subscriptionId] else {
            return
        }
        let rejected = mirror.activity.rejected
        perform(mirror.receive(topic: topic, payload: data.payload, at: Self.now))
        if mirror.activity.rejected > rejected {
            note("\(topic.rawValue): message rejected (\(data.payload.count) bytes)")
        }
    }

    // MARK: Requests

    private func perform(_ actions: [RoomMemoryMirror.Action]) {
        for action in actions {
            switch action {
            case .requestSync(var request):
                request.header.stamp = RoomMemory.Time(date: Date())
                let since = request.since.map { "(\($0.epoch), \($0.seq))" } ?? "NONE"
                if link?.server?.canPublish == true, let payload = try? request.encodedCDR() {
                    send(.data(FoxgloveProtocol.clientMessageData(channelId: Self.requestChannelId, payload: payload)))
                    note("RequestSync \(request.topic) since \(since) sent on the bridge")
                } else if let sink = requestSyncSink, sink(request) {
                    note("RequestSync \(request.topic) since \(since) sent via relay")
                } else {
                    mirror.noteRequestUnsent()
                    note("RequestSync \(request.topic) since \(since) not sent: no request path")
                }
            }
        }
    }

    private func send(_ message: URLSessionWebSocketTask.Message) {
        guard let socket = link?.socket else { return }
        socket.send(message) { [weak self] error in
            guard let error else { return }
            Task { @MainActor in self?.note("send failed: \(error.localizedDescription)") }
        }
    }

    private func note(_ line: String) {
        log.append(line)
        if log.count > Self.logCapacity {
            log.removeFirst(log.count - Self.logCapacity)
        }
    }
}

extension RoomMemory.Time {
    /// The time of `date`, whole seconds plus nanoseconds.
    public init(date: Date) {
        let interval = date.timeIntervalSince1970
        let seconds = interval.rounded(.down)
        self.init(sec: Int32(clamping: Int64(seconds)), nanosec: UInt32(min(999_999_999, max(0, (interval - seconds) * 1e9))))
    }
}
