import Combine
import Foundation

/// The bridge as this connection found it.
public struct LiveRoomServer: Sendable, Equatable {
    public var name: String
    public var capabilities: [String]
    /// The bridge accepts client channels and this client may publish, so `RequestSync` can go out
    /// on a client channel.
    public var canPublish: Bool
    /// `/room_memory/*` topics advertised and subscribed. Empty: the bridge is up but room
    /// memory is not running ("Room memory not running on Spark", never an empty room).
    public var memoryTopics: [String]
    /// The bridge's id for the node's `RequestSync` service (D39), while it is advertised and this
    /// client may send requests. Nil until `advertiseServices` names it, and again after a withdrawal.
    public var requestServiceId: UInt32?
    /// The bridge's id for the node's `OperatorAction` service (D46), while it is advertised. Nil
    /// for a Spark that takes no actions (the service is off by default).
    public var operatorServiceId: UInt32?

    public init(name: String, capabilities: [String], canPublish: Bool, memoryTopics: [String],
                requestServiceId: UInt32? = nil, operatorServiceId: UInt32? = nil) {
        self.name = name
        self.capabilities = capabilities
        self.canPublish = canPublish
        self.memoryTopics = memoryTopics
        self.requestServiceId = requestServiceId
        self.operatorServiceId = operatorServiceId
    }
}

/// How `RequestSync` leaves this Mac (protocol D39). The bridge's service is preferred: it is the
/// path the bridge can restrict to the one name.
public enum LiveRoomRequestPath: Sendable, Equatable {
    /// The node's `/room_memory/sync/request` service, called over the bridge socket.
    case bridgeService(id: UInt32)
    /// A client channel on `/room_memory/request_sync` (the bridge offers `clientPublish`).
    case bridgeChannel
    /// `requestSyncSink`.
    case relay
    case unavailable

    /// The path a connection to `server` has, given whether a sink is set.
    public static func of(_ server: LiveRoomServer?, hasSink: Bool) -> LiveRoomRequestPath {
        if let id = server?.requestServiceId { return .bridgeService(id: id) }
        if server?.canPublish == true { return .bridgeChannel }
        return hasSink ? .relay : .unavailable
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
    /// The node's service for `RequestSync`, the one the bridge exposes (D39). Its request is one
    /// `RequestSync`, so the call carries the message's CDR unchanged.
    public static let requestService = "/room_memory/sync/request"
    public static let requestServiceType = "labcommand_msgs/srv/RequestSync"
    /// The node's service for `OperatorAction` (D46): one action in, one outcome back. The effect
    /// is observed on the wire, as for `RequestSync`.
    public static let operatorService = "/room_memory/operator/action"
    public static let operatorServiceType = "labcommand_msgs/srv/OperatorAction"
    /// URLSession's default cap is 1 MiB; a snapshot may reach 16 MiB raw (D22).
    static let maxMessageBytes = 64 << 20
    static let logCapacity = 200

    @Published public private(set) var connection: LiveRoomConnection = .off
    /// Every change, from a message, a tick or the connection, also feeds `timelineRecorder`.
    @Published public private(set) var mirror: RoomMemoryMirror {
        didSet { timelineRecorder.observe(mirror) }
    }
    /// What this client saw of the Spark over time, for the Room tab's timeline.
    @Published public private(set) var timelineRecorder = LiveRoomTimelineRecorder()
    /// Diagnostics, newest last.
    @Published public private(set) var log: [String] = []
    /// The operator's actions and their outcomes (D46).
    @Published public private(set) var operatorDesk = LiveRoomOperator()

    /// Another path for `RequestSync` (the `:8766` relay, Phase 3 P11). Return true when sent.
    /// Used when the bridge offers neither the request service nor `clientPublish`. Set it before `start()`.
    public var requestSyncSink: ((RoomMemory.RequestSync) -> Bool)?

    /// How a `RequestSync` would leave now. `.unavailable` is "no request path" on screen.
    public var requestPath: LiveRoomRequestPath {
        guard publishRequests else { return .unavailable }
        if case .connected(let server) = connection {
            return .of(server, hasSink: requestSyncSink != nil)
        }
        return requestSyncSink != nil ? .relay : .unavailable
    }

    /// The node's operator service is advertised and this client may call it.
    public var operatorPath: Bool {
        guard publishRequests, case .connected(let server) = connection else { return false }
        return server.operatorServiceId != nil
    }

    public let url: URL
    /// False keeps the client read-only: it never advertises a channel or sends a request.
    public let publishRequests: Bool
    private let session: URLSession
    private var runTask: Task<Void, Never>?
    private var tickTask: Task<Void, Never>?
    /// Bumped by `start()`; a cancelled run that is still unwinding must not touch the new one.
    private var runGeneration = 0
    private var link: Link?

    /// What a service call asked: a `RequestSync` (described) or an `OperatorAction` (its action id).
    private enum PendingCall {
        case requestSync(String)
        case operatorAction(String)
    }

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
        /// The bridge offers `services` and this client may send requests.
        var mayCallServices = false
        var nextCallId: UInt32 = 1
        /// Service calls awaiting a response or failure.
        var pendingCalls: [UInt32: PendingCall] = [:]
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
        operatorDesk.connectionLost()
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
        operatorDesk.connectionLost()
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
        operatorDesk.reconcile(mirror, at: now)
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
            link?.mayCallServices = publishRequests && capabilities.contains(FoxgloveProtocol.servicesCapability)
            connection = .connected(server)
            note("connected \(url) (capabilities: \(capabilities.joined(separator: ", ")))")
            if canPublish {
                send(.string(FoxgloveProtocol.clientAdvertiseMessage(
                    channelId: Self.requestChannelId,
                    topic: Self.requestTopic,
                    schemaName: Self.requestSchemaName
                )))
            } else if publishRequests, link?.mayCallServices != true {
                note(requestSyncSink == nil
                    ? "bridge has no services and no clientPublish, and no relay is set: RequestSync cannot be sent; relying on live snapshots"
                    : "bridge has no services and no clientPublish: RequestSync goes through the relay")
            }
            return
        }
        if let service = FoxgloveProtocol.parseServiceMessage(text) {
            handle(service: service)
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
            perform(mirror.connectionOpened(at: Self.now, canRequest: requestPath != .unavailable))
        }
    }

    private func publishTopics() {
        guard let current = link, var server = current.server else { return }
        server.memoryTopics = current.subscriptions.values.map(\.rawValue).sorted()
        link?.server = server
        connection = .connected(server)
    }

    /// The request service coming and going (node restarts withdraw it), and call failures.
    private func handle(service message: FoxgloveProtocol.ServiceMessage) {
        guard var current = link, var server = current.server else { return }
        switch message {
        case .advertise(let services):
            guard current.mayCallServices else { return }
            var changed = false
            for service in services {
                let expected: String
                switch service.name {
                case Self.requestService: expected = Self.requestServiceType
                case Self.operatorService: expected = Self.operatorServiceType
                default: continue
                }
                guard service.type == expected else {
                    note("\(service.name) advertised as \(service.type); not used")
                    continue
                }
                if service.name == Self.requestService {
                    server.requestServiceId = service.id
                    note("RequestSync goes through the bridge's \(service.name) service")
                } else {
                    server.operatorServiceId = service.id
                    note("operator actions go through the bridge's \(service.name) service")
                }
                changed = true
            }
            guard changed else { return }
        case .unadvertise(let ids):
            var changed = false
            if let id = server.requestServiceId, ids.contains(id) {
                server.requestServiceId = nil
                note("\(Self.requestService) withdrawn")
                changed = true
            }
            if let id = server.operatorServiceId, ids.contains(id) {
                server.operatorServiceId = nil
                note("\(Self.operatorService) withdrawn")
                changed = true
            }
            guard changed else { return }
        case .failure(_, let callId, let reason):
            guard let asked = current.pendingCalls.removeValue(forKey: callId) else { return }
            link = current
            switch asked {
            case .requestSync(let what):
                // The bridge did not reach the node: the request was not carried.
                mirror.noteRequestUnsent()
                note("RequestSync \(what) failed on the bridge: \(reason)")
            case .operatorAction(let actionId):
                operatorDesk.callFailed(actionId: actionId, reason: reason)
                note("OperatorAction \(actionId) failed on the bridge: \(reason)")
            }
            return
        }
        current.server = server
        link = current
        connection = .connected(server)
        if current.mirrorOpened {
            perform(mirror.requestPathChanged(canRequest: requestPath != .unavailable, at: Self.now))
        }
    }

    private func handle(frame: Data) {
        if let response = FoxgloveProtocol.parseServiceCallResponse(frame) {
            switch link?.pendingCalls.removeValue(forKey: response.callId) {
            case .operatorAction(let actionId):
                let outcome = response.encoding == FoxgloveProtocol.cdrEncoding
                    ? RoomMemory.OperatorOutcome.decodeResponse(response.payload) : nil
                operatorDesk.response(actionId: actionId, outcome: outcome, at: Self.now)
                operatorDesk.reconcile(mirror, at: Self.now)
                note("OperatorAction \(actionId): \(outcome.map { "\($0)" } ?? "unreadable response (\(response.payload.count) bytes)")")
            case .requestSync(let asked):
                logRequestResponse(response, asked: asked)
            case nil:
                logRequestResponse(response, asked: "call \(response.callId)")
            }
            return
        }
        guard let data = FoxgloveProtocol.parseMessageData(frame),
              let topic = link?.subscriptions[data.subscriptionId] else {
            return
        }
        let rejected = mirror.activity.rejected
        perform(mirror.receive(topic: topic, payload: data.payload, at: Self.now))
        if mirror.activity.rejected > rejected {
            note("\(topic.rawValue): message rejected (\(data.payload.count) bytes)")
        }
        operatorDesk.reconcile(mirror, at: Self.now)
    }

    private func logRequestResponse(_ response: FoxgloveProtocol.ServiceCallResponse, asked: String) {
        switch Self.requestAccepted(response) {
        case true?: note("RequestSync \(asked) accepted by the node")
        case false?: note("RequestSync \(asked) refused by the node (bad request or another room)")
        case nil: note("RequestSync \(asked): unreadable response (\(response.payload.count) bytes)")
        }
    }

    // MARK: Operator actions

    /// The name an action carries: the `operatorName` default when set, else the macOS user name,
    /// cut to 64 UTF-8 bytes. Self-asserted, not authenticated.
    public nonisolated static func operatorName(_ defaults: UserDefaults = .standard) -> String {
        let chosen = defaults.string(forKey: "operatorName")?.trimmingCharacters(in: .whitespacesAndNewlines)
        var name = (chosen?.isEmpty == false ? chosen : nil) ?? NSUserName()
        while name.utf8.count > RoomMemory.OperatorAction.operatorIdMaxBytes { name.removeLast() }
        return name.isEmpty ? "operator" : name
    }

    /// Sends `kind` on `entry` as the operator sees it now. False when nothing went out (no
    /// operator service, or the action does not apply); the desk says why.
    @discardableResult
    public func act(_ kind: LiveRoomOperator.Kind, on entry: LiveRoomEvent, note text: String = "") -> Bool {
        guard let action = LiveRoomOperator.make(
            kind, entry: entry, mirror: mirror, seq: operatorDesk.takeSeq(), operatorId: Self.operatorName(),
            stamp: RoomMemory.Time(date: Date()), actionId: UUID().uuidString.lowercased(), text: text
        ) else { return false }
        return deliver(action, kind: kind)
    }

    /// Sends an action whose outcome is unknown again, with the same id and payload.
    @discardableResult
    public func retry(actionId: String) -> Bool {
        guard let pending = operatorDesk.retry(actionId: actionId, mirror: mirror) else { return false }
        return deliver(pending.action, kind: pending.kind)
    }

    private func deliver(_ action: RoomMemory.OperatorAction, kind: LiveRoomOperator.Kind) -> Bool {
        var delivered = false
        if operatorPath, case .connected(let server) = connection, let serviceId = server.operatorServiceId,
           let payload = try? action.encodedCDR(), var current = link {
            let callId = current.nextCallId
            current.nextCallId &+= 1
            current.pendingCalls[callId] = .operatorAction(action.actionId)
            link = current
            send(.data(FoxgloveProtocol.serviceCallRequest(serviceId: serviceId, callId: callId, payload: payload)))
            delivered = true
        }
        operatorDesk.sent(action, kind: kind, mirror: mirror, delivered: delivered, at: Self.now)
        note("OperatorAction \(action.action) event \(action.eventId.map(String.init) ?? "NONE") id \(action.actionId) " +
             (delivered ? "sent to \(Self.operatorService)" : "not sent: no operator service"))
        return delivered
    }

    // MARK: Requests

    private func perform(_ actions: [RoomMemoryMirror.Action]) {
        for action in actions {
            switch action {
            case .requestSync(var request):
                request.header.stamp = RoomMemory.Time(date: Date())
                let asked = "\(request.topic) since \(request.since.map { "(\($0.epoch), \($0.seq))" } ?? "NONE")"
                if let path = carry(request, asked: asked) {
                    note("RequestSync \(asked) \(path)")
                } else {
                    mirror.noteRequestUnsent()
                    note("RequestSync \(asked) not sent: no request path")
                }
            }
        }
    }

    /// Sends `request` on the current path. Returns where it went, nil when nothing carried it.
    private func carry(_ request: RoomMemory.RequestSync, asked: String) -> String? {
        switch requestPath {
        case .bridgeService(let serviceId):
            guard let payload = try? request.encodedCDR(), var current = link else { return nil }
            let callId = current.nextCallId
            current.nextCallId &+= 1
            current.pendingCalls[callId] = .requestSync(asked)
            link = current
            send(.data(FoxgloveProtocol.serviceCallRequest(serviceId: serviceId, callId: callId, payload: payload)))
            return "sent to \(Self.requestService)"
        case .bridgeChannel:
            guard let payload = try? request.encodedCDR() else { return nil }
            send(.data(FoxgloveProtocol.clientMessageData(channelId: Self.requestChannelId, payload: payload)))
            return "sent on the bridge"
        case .relay:
            return requestSyncSink?(request) == true ? "sent via relay" : nil
        case .unavailable:
            return nil
        }
    }

    /// The request service's `bool accepted`: CDR with its encapsulation header, then one byte.
    /// Nil when the response is not that.
    public nonisolated static func requestAccepted(_ response: FoxgloveProtocol.ServiceCallResponse) -> Bool? {
        let bytes = [UInt8](response.payload)
        guard response.encoding == FoxgloveProtocol.cdrEncoding, bytes.count >= 5, bytes[0] == 0,
              bytes[1] <= 1, bytes[4] <= 1 else {
            return nil
        }
        return bytes[4] == 1
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
