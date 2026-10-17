import Foundation

/// The room-memory topics and the client-side Foxglove frames the live room link needs on top
/// of `FoxgloveProtocol`. Pure: the socket lives in `LiveRoomClient`.
public enum LiveRoomTopic: String, Sendable, CaseIterable {
    case mapBricks = "/room_memory/map_bricks"
    case events = "/room_memory/events"
    case status = "/room_memory/status"
    /// Transient motion next to the rig (`Activity`), 1 Hz.
    case activity = "/room_memory/activity"
    /// The timeline tick (`Epoch`), 0.2 Hz. No node publishes it yet (protocol §2).
    case epoch = "/room_memory/epoch"
    case syncMapBricks = "/room_memory/sync/map_bricks"
    case syncEvents = "/room_memory/sync/events"
    /// The end-of-answer manifest of every `RequestSync` answer (`SyncEnd`).
    case syncEnd = "/room_memory/sync/end"

    /// `schemaName` the bridge advertises for the topic (ROS 2 form).
    public var schemaName: String {
        switch self {
        case .mapBricks, .syncMapBricks: return "labcommand_msgs/msg/MapBricks"
        case .events, .syncEvents: return "labcommand_msgs/msg/ChangeEvent"
        case .status: return "labcommand_msgs/msg/MemoryStatus"
        case .activity: return "labcommand_msgs/msg/Activity"
        case .epoch: return "labcommand_msgs/msg/Epoch"
        case .syncEnd: return "labcommand_msgs/msg/SyncEnd"
        }
    }

    /// True for the topics that carry answers to `RequestSync`, any Mac's.
    public var isSync: Bool { self == .syncMapBricks || self == .syncEvents || self == .syncEnd }

    /// The advertised channel, if it is one of ours. A known topic with another schema
    /// (or a non-CDR encoding) is refused rather than misdecoded.
    public init?(channel: FoxgloveProtocol.Channel) {
        guard let topic = LiveRoomTopic(rawValue: channel.topic),
              channel.encoding == FoxgloveProtocol.cdrEncoding,
              channel.schemaName.isEmpty || channel.schemaName == topic.schemaName
                || channel.schemaName == topic.schemaName.replacingOccurrences(of: "/msg/", with: "/") else {
            return nil
        }
        self = topic
    }
}

extension FoxgloveProtocol {
    /// Lets clients advertise channels and publish on them (`RequestSync`).
    public static let clientPublishCapability = "clientPublish"

    /// `serverInfo.capabilities`; nil when `text` is not a `serverInfo`.
    public static func serverCapabilities(_ text: String) -> [String]? {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any],
              object["op"] as? String == "serverInfo" else {
            return nil
        }
        return (object["capabilities"] as? [Any] ?? []).compactMap { $0 as? String }
    }

    /// Client advertise: `{"op":"advertise","channels":[{"id":…,"topic":…,"encoding":"cdr","schemaName":…}]}`.
    public static func clientAdvertiseMessage(channelId: UInt32, topic: String, schemaName: String) -> String {
        let channel: [String: Any] = [
            "id": channelId,
            "topic": topic,
            "encoding": cdrEncoding,
            "schemaName": schemaName,
        ]
        let object: [String: Any] = ["op": "advertise", "channels": [channel]]
        let data = (try? JSONSerialization.data(withJSONObject: object, options: [.sortedKeys])) ?? Data()
        return String(decoding: data, as: UTF8.self)
    }

    /// Client unadvertise: `{"op":"unadvertise","channelIds":[…]}`.
    public static func clientUnadvertiseMessage(_ channelIds: [UInt32]) -> String {
        "{\"op\":\"unadvertise\",\"channelIds\":[\(channelIds.map(String.init).joined(separator: ","))]}"
    }

    /// Client message data: opcode `0x01`, uint32 LE client channel id, then the payload.
    public static func clientMessageData(channelId: UInt32, payload: Data) -> Data {
        var frame = Data(capacity: 5 + payload.count)
        frame.append(messageDataOpcode)
        withUnsafeBytes(of: channelId.littleEndian) { frame.append(contentsOf: $0) }
        frame.append(payload)
        return frame
    }
}

// MARK: Services (protocol D39)

extension FoxgloveProtocol {
    /// Lets clients call the services the server advertises (`RequestSync`, D39).
    public static let servicesCapability = "services"
    /// Binary opcode 0x02, client → server: uint32 LE service id, uint32 LE call id, uint32 LE
    /// encoding length, the encoding, then the request payload.
    public static let serviceCallRequestOpcode: UInt8 = 0x02
    /// Binary opcode 0x03, server → client: the same layout, with the response payload.
    public static let serviceCallResponseOpcode: UInt8 = 0x03

    /// One advertised service. `type` is the ROS service type (`labcommand_msgs/srv/RequestSync`).
    public struct Service: Sendable, Equatable {
        public var id: UInt32
        public var name: String
        public var type: String

        public init(id: UInt32, name: String, type: String) {
            self.id = id
            self.name = name
            self.type = type
        }
    }

    /// The service ops of the text frames.
    public enum ServiceMessage: Sendable, Equatable {
        case advertise([Service])
        case unadvertise([UInt32])
        /// The server could not carry a call (unknown service, node not up, bad encoding, …).
        case failure(serviceId: UInt32, callId: UInt32, message: String)
    }

    /// `advertiseServices`, `unadvertiseServices` or `serviceCallFailure`; nil for any other text.
    /// Services missing an id, name or type are dropped rather than failing the whole batch.
    public static func parseServiceMessage(_ text: String) -> ServiceMessage? {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any],
              let op = object["op"] as? String else {
            return nil
        }
        switch op {
        case "advertiseServices":
            let raw = object["services"] as? [[String: Any]] ?? []
            return .advertise(raw.compactMap { entry in
                guard let id = uint32(entry["id"]), let name = entry["name"] as? String,
                      let type = entry["type"] as? String else {
                    return nil
                }
                return Service(id: id, name: name, type: type)
            })
        case "unadvertiseServices":
            return .unadvertise((object["serviceIds"] as? [Any] ?? []).compactMap(uint32))
        case "serviceCallFailure":
            guard let serviceId = uint32(object["serviceId"]), let callId = uint32(object["callId"]) else { return nil }
            return .failure(serviceId: serviceId, callId: callId, message: object["message"] as? String ?? "")
        default:
            return nil
        }
    }

    private static func uint32(_ value: Any?) -> UInt32? {
        guard let number = value as? NSNumber else { return nil }
        return UInt32(exactly: number.doubleValue)
    }

    /// A service call: opcode `0x02`, then the ids, the encoding and the payload (CDR with its
    /// encapsulation header, as for a message).
    public static func serviceCallRequest(serviceId: UInt32, callId: UInt32, encoding: String = cdrEncoding,
                                          payload: Data) -> Data {
        let encodingBytes = Data(encoding.utf8)
        var frame = Data(capacity: 13 + encodingBytes.count + payload.count)
        frame.append(serviceCallRequestOpcode)
        for value in [serviceId, callId, UInt32(encodingBytes.count)] {
            withUnsafeBytes(of: value.littleEndian) { frame.append(contentsOf: $0) }
        }
        frame.append(encodingBytes)
        frame.append(payload)
        return frame
    }

    public struct ServiceCallResponse: Sendable, Equatable {
        public var serviceId: UInt32
        public var callId: UInt32
        public var encoding: String
        /// A slice of the frame it was parsed from.
        public var payload: Data

        public init(serviceId: UInt32, callId: UInt32, encoding: String, payload: Data) {
            self.serviceId = serviceId
            self.callId = callId
            self.encoding = encoding
            self.payload = payload
        }
    }

    /// Parse a binary frame. Nil for any opcode other than a service call response, or a frame
    /// too short for its header and encoding.
    public static func parseServiceCallResponse(_ frame: Data) -> ServiceCallResponse? {
        guard frame.count >= 13, frame.first == serviceCallResponseOpcode else { return nil }
        let (serviceId, callId, encodingLength) = frame.withUnsafeBytes { bytes in
            (
                UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: 1, as: UInt32.self)),
                UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: 5, as: UInt32.self)),
                Int(UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: 9, as: UInt32.self)))
            )
        }
        guard frame.count - 13 >= encodingLength else { return nil }
        let start = frame.startIndex + 13
        guard let encoding = String(data: frame[start..<(start + encodingLength)], encoding: .utf8) else { return nil }
        return ServiceCallResponse(serviceId: serviceId, callId: callId, encoding: encoding,
                                   payload: frame[(start + encodingLength)...])
    }
}
