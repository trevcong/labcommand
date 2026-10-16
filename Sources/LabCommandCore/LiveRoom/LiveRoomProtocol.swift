import Foundation

/// The room-memory topics and the client-side Foxglove frames the live room link needs on top
/// of `FoxgloveProtocol`. Pure: the socket lives in `LiveRoomClient`.
public enum LiveRoomTopic: String, Sendable, CaseIterable {
    case mapBricks = "/room_memory/map_bricks"
    case events = "/room_memory/events"
    case status = "/room_memory/status"
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
