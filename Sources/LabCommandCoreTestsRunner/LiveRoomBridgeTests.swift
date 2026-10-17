import Foundation
import LabCommandCore
import Network

// MARK: - The request path over the bridge (issue #21, protocol D39)
//
// Live on c5518eb the :8767 bridge offered `connectionGraph` only, so `LiveRoomClient` had no way
// to send `RequestSync` and the Live Room read "history incomplete: 20 missing · no request path".
// The node now offers `/room_memory/sync/request`, and the bridge exposes it through `services`.
// `LRFakeBridge` stands in for that bridge on a loopback WebSocket and answers from `LRPublisher`.

/// A local Foxglove WebSocket server shaped like the Spark's :8767 bridge: `serverInfo`, then the
/// room-memory channels, then (when asked) `advertiseServices` — after the channels, so the client
/// has already opened its mirror when the service appears. Service calls are answered from
/// `publisher`: the answer on the sync channels, then the response.
@MainActor
final class LRFakeBridge {
    static let serviceId: UInt32 = 41
    /// The node's operator service (D46), advertised only when `operatorOutcome` or `operatorFailure` is set.
    static let operatorServiceId: UInt32 = 43
    static let channelIds: [LiveRoomTopic: UInt32] = [
        .status: 1, .events: 2, .mapBricks: 3, .syncEvents: 4, .syncMapBricks: 5, .syncEnd: 6,
    ]

    let capabilities: [String]
    var serviceType: String?
    var publisher: LRPublisher
    private(set) var port: UInt16 = 0
    /// Every service call the client made, decoded.
    private(set) var calls: [(callId: UInt32, encoding: String, request: RM.RequestSync?)] = []
    /// Every binary frame the client sent, by opcode.
    private(set) var clientOpcodes: [UInt8] = []
    /// The operator service's answer to every call; with `operatorFailure`, a `serviceCallFailure` instead.
    var operatorOutcome: RM.OperatorOutcome?
    var operatorFailure: String?
    /// Every operator call the client made, decoded.
    private(set) var operatorCalls: [RM.OperatorAction] = []
    private var listener: NWListener?
    private var connection: NWConnection?
    private var subscriptions: [UInt32: UInt32] = [:]
    private var statusTimer: Timer?

    /// `serviceType` nil: no `advertiseServices`.
    init(capabilities: [String], serviceType: String?, publisher: LRPublisher) {
        self.capabilities = capabilities
        self.serviceType = serviceType
        self.publisher = publisher
    }

    var url: URL { URL(string: "ws://127.0.0.1:\(port)")! }

    func start() {
        let websocket = NWProtocolWebSocket.Options()
        websocket.autoReplyPing = true
        websocket.setClientRequestHandler(.main) { subprotocols, _ in
            NWProtocolWebSocket.Response(status: .accept, subprotocol: subprotocols.first)
        }
        let parameters = NWParameters.tcp
        parameters.defaultProtocolStack.applicationProtocols.insert(websocket, at: 0)
        parameters.requiredInterfaceType = .loopback
        do {
            listener = try NWListener(using: parameters, on: .any)
        } catch {
            expect(false, "fake bridge listens: \(error)")
        }
        listener?.newConnectionHandler = { [weak self] connection in
            MainActor.assumeIsolated { self?.accept(connection) }
        }
        listener?.start(queue: .main)
        let deadline = Date().addingTimeInterval(5)
        while listener?.port?.rawValue ?? 0 == 0, Date() < deadline {
            RunLoop.main.run(until: Date().addingTimeInterval(0.01))
        }
        port = listener?.port?.rawValue ?? 0
        expect(port != 0, "fake bridge has a port")
    }

    func stop() {
        statusTimer?.invalidate()
        connection?.cancel()
        listener?.cancel()
    }

    private func accept(_ connection: NWConnection) {
        self.connection?.cancel()
        self.connection = connection
        subscriptions = [:]
        connection.stateUpdateHandler = { [weak self] state in
            guard case .ready = state else { return }
            MainActor.assumeIsolated { self?.opened() }
        }
        connection.start(queue: .main)
        receive(on: connection)
    }

    private func opened() {
        send(text: json(["op": "serverInfo", "name": "", "capabilities": capabilities, "supportedEncodings": ["cdr", "json"]]))
        let channels: [[String: Any]] = Self.channelIds.sorted { $0.value < $1.value }.map { topic, id in
            ["id": id, "topic": topic.rawValue, "encoding": "cdr", "schemaName": topic.schemaName,
             "schemaEncoding": "ros2msg", "schema": ""]
        }
        send(text: json(["op": "advertise", "channels": channels]))
        if let serviceType { advertiseService(type: serviceType) }
        if operatorOutcome != nil || operatorFailure != nil { advertiseOperatorService() }
        statusTimer = Timer.scheduledTimer(withTimeInterval: 0.3, repeats: true) { [weak self] _ in
            MainActor.assumeIsolated { self?.publishStatus() }
        }
    }

    func advertiseService(type: String) {
        send(text: json(["op": "advertiseServices", "services": [
            ["id": Self.serviceId, "name": LiveRoomClient.requestService, "type": type,
             "request": ["encoding": "cdr", "schemaName": type + "_Request", "schemaEncoding": "ros2msg", "schema": ""],
             "response": ["encoding": "cdr", "schemaName": type + "_Response", "schemaEncoding": "ros2msg", "schema": ""]],
            // Another node's service the client must not take for the request path.
            ["id": Self.serviceId + 1, "name": "/other/sync/request", "type": LiveRoomClient.requestServiceType],
        ]]))
    }

    func advertiseOperatorService() {
        send(text: json(["op": "advertiseServices", "services": [
            ["id": Self.operatorServiceId, "name": LiveRoomClient.operatorService, "type": LiveRoomClient.operatorServiceType],
        ]]))
    }

    func withdrawOperatorService() {
        send(text: json(["op": "unadvertiseServices", "serviceIds": [Self.operatorServiceId]]))
    }

    func withdrawService() {
        send(text: json(["op": "unadvertiseServices", "serviceIds": [Self.serviceId]]))
    }

    private func publishStatus() {
        publish(publisher.status())
    }

    private func publish(_ message: LRMessage) {
        guard let channel = Self.channelIds[message.topic], let subscription = subscriptions[channel] else { return }
        var frame = Data([FoxgloveProtocol.messageDataOpcode])
        withUnsafeBytes(of: subscription.littleEndian) { frame.append(contentsOf: $0) }
        withUnsafeBytes(of: UInt64(0).littleEndian) { frame.append(contentsOf: $0) }
        frame.append(message.data)
        send(binary: frame)
    }

    private func receive(on connection: NWConnection) {
        connection.receiveMessage { [weak self] data, context, _, error in
            MainActor.assumeIsolated {
                guard let self, error == nil, connection === self.connection else { return }
                let metadata = context?.protocolMetadata(definition: NWProtocolWebSocket.definition) as? NWProtocolWebSocket.Metadata
                if let data, let metadata {
                    switch metadata.opcode {
                    case .text: self.handle(text: String(decoding: data, as: UTF8.self))
                    case .binary: self.handle(binary: data)
                    default: break
                    }
                }
                self.receive(on: connection)
            }
        }
    }

    private func handle(text: String) {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any],
              object["op"] as? String == "subscribe" else { return }
        for entry in object["subscriptions"] as? [[String: Any]] ?? [] {
            if let id = (entry["id"] as? NSNumber)?.uint32Value, let channel = (entry["channelId"] as? NSNumber)?.uint32Value {
                subscriptions[channel] = id
            }
        }
        publishStatus()
    }

    /// A service call: opcode, service id, call id, encoding length, encoding, payload (ws-protocol v1).
    private func handle(binary frame: Data) {
        guard let opcode = frame.first else { return }
        clientOpcodes.append(opcode)
        guard opcode == FoxgloveProtocol.serviceCallRequestOpcode, frame.count >= 13 else { return }
        let bytes = [UInt8](frame)
        func word(_ at: Int) -> UInt32 {
            UInt32(bytes[at]) | UInt32(bytes[at + 1]) << 8 | UInt32(bytes[at + 2]) << 16 | UInt32(bytes[at + 3]) << 24
        }
        let serviceId = word(1), callId = word(5), length = Int(word(9))
        let encoding = String(decoding: bytes[13..<(13 + length)], as: UTF8.self)
        if serviceId == Self.operatorServiceId {
            if let action = try? RM.OperatorAction.decode(cdr: Data(bytes[(13 + length)...])) { operatorCalls.append(action) }
            if let operatorFailure {
                send(text: json(["op": "serviceCallFailure", "serviceId": serviceId, "callId": callId, "message": operatorFailure]))
                return
            }
            var response = Data([FoxgloveProtocol.serviceCallResponseOpcode])
            for value in [serviceId, callId, UInt32(3)] {
                withUnsafeBytes(of: value.littleEndian) { response.append(contentsOf: $0) }
            }
            response.append(Data("cdr".utf8))
            response.append(Data([0x00, 0x01, 0x00, 0x00, operatorOutcome?.rawValue ?? 0]))
            send(binary: response)
            return
        }
        let request = try? RM.RequestSync.decode(cdr: Data(bytes[(13 + length)...]))
        calls.append((callId, encoding, request))
        guard serviceId == Self.serviceId, let request else { return }
        for message in publisher.answer(request) { publish(message) }
        // `bool accepted = true`, CDR little endian with its encapsulation header.
        var response = Data([FoxgloveProtocol.serviceCallResponseOpcode])
        for value in [serviceId, callId, UInt32(3)] {
            withUnsafeBytes(of: value.littleEndian) { response.append(contentsOf: $0) }
        }
        response.append(Data("cdr".utf8))
        response.append(Data([0x00, 0x01, 0x00, 0x00, 0x01]))
        send(binary: response)
    }

    private func json(_ object: [String: Any]) -> String {
        String(decoding: (try? JSONSerialization.data(withJSONObject: object, options: [.sortedKeys])) ?? Data(), as: UTF8.self)
    }

    private func send(text: String) {
        send(Data(text.utf8), opcode: .text)
    }

    private func send(binary: Data) {
        send(binary, opcode: .binary)
    }

    private func send(_ data: Data, opcode: NWProtocolWebSocket.Opcode) {
        let context = NWConnection.ContentContext(identifier: "frame", metadata: [NWProtocolWebSocket.Metadata(opcode: opcode)])
        connection?.send(content: data, contentContext: context, isComplete: true, completion: .contentProcessed { _ in })
    }
}

/// A publisher that already ran: five event messages and a map the Mac never saw.
private func lrPublisherWithMissedHistory() -> LRPublisher {
    var publisher = LRPublisher(seed: 21, generation: 1_760_400_000_021, legacy: false)
    _ = publisher.map()
    for _ in 0..<5 { _ = publisher.event() }
    return publisher
}

@MainActor
private func spin(_ seconds: TimeInterval, until done: () -> Bool = { false }) {
    let deadline = Date().addingTimeInterval(seconds)
    while Date() < deadline, !done() {
        RunLoop.main.run(until: Date().addingTimeInterval(0.02))
    }
}

@MainActor
private func lrConnect(_ bridge: LRFakeBridge, publishRequests: Bool = true) -> LiveRoomClient {
    bridge.start()
    let client = LiveRoomClient(url: bridge.url, publishRequests: publishRequests)
    client.start()
    spin(5) { if case .connected(let server) = client.connection { return !server.memoryTopics.isEmpty } else { return false } }
    expect({ if case .connected = client.connection { return true } else { return false } }(), "client connects to the fake bridge")
    return client
}

private func isComplete(_ mirror: RoomMemoryMirror, through top: UInt64) -> Bool {
    mirror.eventHistory.coverage == .completeThrough(top)
}

/// The live repro: a read-only bridge leaves the gap open and says so.
@MainActor
func testLiveRoomReadOnlyBridgeHasNoRequestPath() {
    let bridge = LRFakeBridge(capabilities: ["connectionGraph"], serviceType: nil, publisher: lrPublisherWithMissedHistory())
    let client = lrConnect(bridge)
    defer { client.stop(); bridge.stop() }
    spin(1.5)
    expect(client.requestPath == .unavailable, "read-only bridge: no request path")
    expect(bridge.clientOpcodes.isEmpty, "read-only bridge: the client sends no binary frame")
    guard case .missing = client.mirror.eventHistory.coverage else {
        expect(false, "read-only bridge: history stays incomplete, got \(client.mirror.eventHistory.coverage)")
        return
    }
    expect(client.mirror.eventHistory.missingCounts.missing == 5, "read-only bridge: 5 missing")
}

/// The fix: the service appears after the channels (the mirror is already open), the client
/// switches to it, calls it with the RequestSync bytes, and the gap closes.
@MainActor
func testLiveRoomRequestServiceClosesTheGap() {
    let bridge = LRFakeBridge(capabilities: ["connectionGraph", "services"], serviceType: LiveRoomClient.requestServiceType,
                              publisher: lrPublisherWithMissedHistory())
    let client = lrConnect(bridge)
    defer { client.stop(); bridge.stop() }
    spin(5) { isComplete(client.mirror, through: 4) }
    expect(client.requestPath == .bridgeService(id: LRFakeBridge.serviceId), "services bridge: the request service is the path, got \(client.requestPath)")
    expect(isComplete(client.mirror, through: 4), "the service call closes the gap: got \(client.mirror.eventHistory.coverage)")
    expect(!bridge.calls.isEmpty && bridge.calls.allSatisfy { $0.encoding == "cdr" && $0.request?.token != nil },
           "every call is CDR and decodes as a v2 RequestSync")
    expect(bridge.calls.contains { $0.request?.topic == .events && $0.request?.expectedGeneration == bridge.publisher.generation },
           "an events request names the publisher's generation")
    expect(Set(bridge.calls.map(\.callId)).count == bridge.calls.count, "call ids are unique")
    expect(bridge.clientOpcodes.allSatisfy { $0 == FoxgloveProtocol.serviceCallRequestOpcode }, "no client-channel publish")
    expect(client.mirror.activity.requestsUnsent == 0, "nothing unsent")
    expect(client.log.contains { $0.contains("accepted by the node") }, "the response is read")

    // A node restart withdraws the service; the path goes with it, and comes back.
    bridge.withdrawService()
    spin(2) { client.requestPath == .unavailable }
    expect(client.requestPath == .unavailable && !client.mirror.canRequest, "withdrawn service: no request path")
    bridge.advertiseService(type: LiveRoomClient.requestServiceType)
    spin(2) { client.requestPath != .unavailable }
    expect(client.requestPath == .bridgeService(id: LRFakeBridge.serviceId) && client.mirror.canRequest, "re-advertised service: the path is back")
}

/// A service by that name with another type is not the request path; nor is any service for a
/// read-only client.
@MainActor
func testLiveRoomRequestServiceIsChecked() {
    let wrongType = LRFakeBridge(capabilities: ["connectionGraph", "services"], serviceType: "std_srvs/srv/Trigger",
                                 publisher: lrPublisherWithMissedHistory())
    let client = lrConnect(wrongType)
    spin(1)
    expect(client.requestPath == .unavailable && wrongType.calls.isEmpty, "another service type: not used")
    client.stop(); wrongType.stop()

    let readOnly = LRFakeBridge(capabilities: ["connectionGraph", "services"], serviceType: LiveRoomClient.requestServiceType,
                                publisher: lrPublisherWithMissedHistory())
    let quiet = lrConnect(readOnly, publishRequests: false)
    spin(1)
    expect(quiet.requestPath == .unavailable && readOnly.clientOpcodes.isEmpty, "publishRequests false: never calls the service")
    quiet.stop(); readOnly.stop()
}

func testLiveRoomServiceFrames() {
    // ws-protocol v1 snapshots from foxglove-sdk v0.28.0 (rust/foxglove/src/protocol/v1/snapshots).
    let sdkPayload = Data(#"{"key": "value"}"#.utf8)
    let request = FoxgloveProtocol.serviceCallRequest(serviceId: 10, callId: 12, encoding: "json", payload: sdkPayload)
    expect([UInt8](request) == [0x02, 0x0a, 0, 0, 0, 0x0c, 0, 0, 0, 0x04, 0, 0, 0] + Array("json".utf8) + Array(sdkPayload),
           "service call request matches the SDK's bytes")
    var responseFrame = request
    responseFrame[responseFrame.startIndex] = FoxgloveProtocol.serviceCallResponseOpcode
    let response = FoxgloveProtocol.parseServiceCallResponse(responseFrame)
    expect(response?.serviceId == 10 && response?.callId == 12 && response?.encoding == "json" && response?.payload == sdkPayload,
           "service call response parses the SDK's bytes")
    expect(FoxgloveProtocol.parseServiceCallResponse(request) == nil, "a request is not a response")
    expect(FoxgloveProtocol.parseMessageData(responseFrame) == nil, "a response is not message data")
    expect(FoxgloveProtocol.parseServiceCallResponse(Data([0x03, 10, 0, 0, 0, 12, 0, 0, 0, 9, 0, 0, 0, 0x63])) == nil,
           "a response shorter than its encoding is refused")

    let advertise = FoxgloveProtocol.parseServiceMessage("""
    {"op":"advertiseServices","services":[
      {"id":10,"name":"/s1","type":"my_type","request":{"encoding":"json","schemaName":"q","schemaEncoding":"jsonschema","schema":"{}"},
       "response":{"encoding":"json","schemaName":"r","schemaEncoding":"jsonschema","schema":"{}"}},
      {"id":30,"name":"/s3","type":"old_type","requestSchema":"request-schema","responseSchema":"response-schema"},
      {"id":40,"name":"/no-type"}]}
    """)
    expect(advertise == .advertise([.init(id: 10, name: "/s1", type: "my_type"), .init(id: 30, name: "/s3", type: "old_type")]),
           "advertiseServices: both schema forms; an entry without a type is dropped")
    expect(FoxgloveProtocol.parseServiceMessage(#"{"op":"unadvertiseServices","serviceIds":[10,30]}"#) == .unadvertise([10, 30]),
           "unadvertiseServices")
    expect(FoxgloveProtocol.parseServiceMessage(#"{"op":"serviceCallFailure","serviceId":1,"callId":1,"message":"Service does not exist"}"#)
           == .failure(serviceId: 1, callId: 1, message: "Service does not exist"), "serviceCallFailure")
    expect(FoxgloveProtocol.parseServiceMessage(#"{"op":"advertise","channels":[]}"#) == nil, "channel ops are not service ops")

    func reply(_ bytes: [UInt8], encoding: String = "cdr") -> Bool? {
        LiveRoomClient.requestAccepted(.init(serviceId: 1, callId: 1, encoding: encoding, payload: Data(bytes)))
    }
    expect(reply([0, 1, 0, 0, 1]) == true && reply([0, 1, 0, 0, 0]) == false && reply([0, 0, 0, 0, 1]) == true,
           "accepted: LE and BE CDR bools")
    expect(reply([0, 1, 0, 0]) == nil && reply([0, 1, 0, 0, 2]) == nil && reply([0, 1, 0, 0, 1], encoding: "json") == nil,
           "accepted: short, non-bool or non-CDR responses are unreadable")

    let both = LiveRoomServer(name: "", capabilities: [], canPublish: true, memoryTopics: [], requestServiceId: 7)
    expect(LiveRoomRequestPath.of(both, hasSink: true) == .bridgeService(id: 7), "the service wins over client publish and the relay")
    expect(LiveRoomRequestPath.of(LiveRoomServer(name: "", capabilities: [], canPublish: true, memoryTopics: []), hasSink: true)
           == .bridgeChannel, "client publish wins over the relay")
    expect(LiveRoomRequestPath.of(nil, hasSink: true) == .relay && LiveRoomRequestPath.of(nil, hasSink: false) == .unavailable,
           "relay, else nothing")
}

/// The mirror follows a request path that appears after it opened, and one that goes away.
func testLiveRoomRequestPathChanges() {
    let generation: UInt64 = 1_760_400_000_021
    var mirror = RoomMemoryMirror()
    expect(mirror.connectionOpened(at: 0, canRequest: false).isEmpty, "opened without a path: nothing to send")
    let early = mirror.receive(topic: .status, payload: LRWire.status(generation, seq: 0, epoch: 5, eventsSeq: 4), at: 0.1)
    expect(early.isEmpty && mirror.eventHistory.missingCounts.missing == 5, "no path: 5 missing, nothing requested")
    let opened = mirror.requestPathChanged(canRequest: true, at: 0.2)
    expect(LRWire.request(opened, .events, "path appears") != nil, "the path appears: events requested at once")
    expect(mirror.requestPathChanged(canRequest: true, at: 0.3).isEmpty, "the same path again: no new request")
    _ = mirror.requestPathChanged(canRequest: false, at: 0.4)
    expect(!mirror.canRequest && mirror.tick(at: 30).isEmpty, "the path goes away: timeouts send nothing")
}

func runLiveRoomBridgeTests() {
    testLiveRoomServiceFrames()
    print("  LiveRoom D39: service call frames match the SDK, advertise/unadvertise/failure parse, accepted reply, path priority"); passed()
    testLiveRoomRequestPathChanges()
    print("  LiveRoom D39: the mirror follows a request path that appears after open, and one that goes away"); passed()
    MainActor.assumeIsolated {
        testLiveRoomReadOnlyBridgeHasNoRequestPath()
        print("  LiveRoom #21 repro: a read-only bridge leaves 5 events missing with no request path"); passed()
        testLiveRoomRequestServiceClosesTheGap()
        print("  LiveRoom #21: the bridge's request service, advertised after the channels, closes the gap; withdrawn and back"); passed()
        testLiveRoomRequestServiceIsChecked()
        print("  LiveRoom #21: a wrong service type, or a read-only client, never calls"); passed()
    }
}
