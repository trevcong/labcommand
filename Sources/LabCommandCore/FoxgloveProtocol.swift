import Foundation

/// The slice of the Foxglove WebSocket protocol the live 3D view needs: channel
/// advertisements in, subscriptions out, message data in.
/// Pure parsing — the socket lives in the app's `FoxgloveLiveClient`.
public enum FoxgloveProtocol {
    /// Offered in this order on the upgrade. Spark's foxglove_bridge (built on
    /// foxglove-sdk-cpp v0.25.1, ROS jazzy) answers HTTP 400 unless `foxglove.sdk.v1`
    /// is offered; older bridges only know `foxglove.websocket.v1`. The message
    /// format the live view uses is the same for both.
    public static let subprotocols = ["foxglove.sdk.v1", "foxglove.websocket.v1"]
    public static let cdrEncoding = "cdr"

    /// Binary opcode 0x01: uint32 LE subscription id, uint64 LE receive time (ns), payload.
    public static let messageDataOpcode: UInt8 = 0x01
    public static let messageDataHeaderSize = 13

    // MARK: Server → client (text)

    public struct Channel: Sendable, Equatable {
        public var id: UInt32
        public var topic: String
        public var encoding: String
        public var schemaName: String

        public init(id: UInt32, topic: String, encoding: String, schemaName: String = "") {
            self.id = id
            self.topic = topic
            self.encoding = encoding
            self.schemaName = schemaName
        }

        /// `sensor_msgs/msg/PointCloud2` (ROS 2) or `sensor_msgs/PointCloud2` (ROS 1).
        /// A bridge that doesn't name the schema gets the benefit of the doubt.
        public var carriesPointCloud2: Bool {
            schemaName.isEmpty || schemaName.hasSuffix("/PointCloud2")
        }
    }

    public enum ServerMessage: Sendable, Equatable {
        case serverInfo(name: String)
        case advertise([Channel])
        case unadvertise([UInt32])
        case status(level: Int, message: String)
        /// A valid op the live view has no use for (parameterValues, advertiseServices, …).
        case other(op: String)
    }

    /// Parse a text frame. Nil when it isn't a JSON object with an `op`.
    /// Channels missing an id or topic are dropped rather than failing the whole batch.
    public static func parseServerMessage(_ text: String) -> ServerMessage? {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any],
              let op = object["op"] as? String else {
            return nil
        }
        switch op {
        case "serverInfo":
            return .serverInfo(name: object["name"] as? String ?? "")
        case "advertise":
            let raw = object["channels"] as? [[String: Any]] ?? []
            return .advertise(raw.compactMap { entry in
                guard let id = channelID(entry["id"]), let topic = entry["topic"] as? String else {
                    return nil
                }
                return Channel(
                    id: id,
                    topic: topic,
                    encoding: entry["encoding"] as? String ?? "",
                    schemaName: entry["schemaName"] as? String ?? ""
                )
            })
        case "unadvertise":
            let raw = object["channelIds"] as? [Any] ?? []
            return .unadvertise(raw.compactMap(channelID))
        case "status":
            return .status(
                level: (object["level"] as? NSNumber)?.intValue ?? 0,
                message: object["message"] as? String ?? ""
            )
        default:
            return .other(op: op)
        }
    }

    private static func channelID(_ value: Any?) -> UInt32? {
        guard let number = value as? NSNumber else { return nil }
        return UInt32(exactly: number.doubleValue)
    }

    // MARK: Client → server (text)

    public struct Subscription: Sendable, Equatable {
        public var id: UInt32
        public var channelId: UInt32

        public init(id: UInt32, channelId: UInt32) {
            self.id = id
            self.channelId = channelId
        }
    }

    /// `{"op":"subscribe","subscriptions":[{"id":<subId>,"channelId":<chanId>}]}`
    public static func subscribeMessage(_ subscriptions: [Subscription]) -> String {
        let entries = subscriptions
            .map { "{\"id\":\($0.id),\"channelId\":\($0.channelId)}" }
            .joined(separator: ",")
        return "{\"op\":\"subscribe\",\"subscriptions\":[\(entries)]}"
    }

    // MARK: Server → client (binary)

    public struct MessageData: Sendable, Equatable {
        public var subscriptionId: UInt32
        public var receiveTimestampNs: UInt64
        /// A slice of the frame it was parsed from — no copy.
        public var payload: Data
    }

    /// Parse a binary frame. Nil for any opcode other than message data, or a frame
    /// too short to hold the header.
    public static func parseMessageData(_ frame: Data) -> MessageData? {
        guard frame.count >= messageDataHeaderSize, frame.first == messageDataOpcode else { return nil }
        let (subscriptionId, timestamp) = frame.withUnsafeBytes { bytes in
            (
                UInt32(littleEndian: bytes.loadUnaligned(fromByteOffset: 1, as: UInt32.self)),
                UInt64(littleEndian: bytes.loadUnaligned(fromByteOffset: 5, as: UInt64.self))
            )
        }
        return MessageData(
            subscriptionId: subscriptionId,
            receiveTimestampNs: timestamp,
            payload: frame[(frame.startIndex + messageDataHeaderSize)...]
        )
    }

    // MARK: Live lidar topics

    public static let mid360Topic = "/livox/lidar"
    public static let jt128Topic = "/lidar_points"

    /// The sensor whose cloud a topic carries; nil for topics the 3D view ignores.
    public static func liveSensor(forTopic topic: String) -> SensorID? {
        switch topic {
        case mid360Topic: return .mid360
        case jt128Topic: return .jt128
        default: return nil
        }
    }

    // MARK: Endpoint

    /// Hosts to try for `ws://<host>:8765`, in `SparkSSH.hopList` order (tailnet IP
    /// first, then the last LAN hop that answered SSH). Only IPv4 literals: the ssh
    /// aliases don't resolve outside ssh, and App Transport Security refuses
    /// cleartext to DNS names under the app's `NSAllowsLocalNetworking`.
    public static func streamHosts(lastLAN: String? = nil) -> [String] {
        SparkSSH.hopList(lastLAN: lastLAN).filter(isIPv4Literal)
    }

    public static func isIPv4Literal(_ host: String) -> Bool {
        let parts = host.split(separator: ".", omittingEmptySubsequences: false)
        return parts.count == 4 && parts.allSatisfy { part in
            part.count <= 3 && part.allSatisfy { $0.isASCII && $0.isNumber } && UInt8(part) != nil
        }
    }

    /// Reconnect backoff: 1 s, 2 s, 4 s … capped at 30 s. Zero failures means the
    /// last connection worked, so retry promptly.
    public static func reconnectDelay(afterFailures failures: Int) -> TimeInterval {
        guard failures > 1 else { return 1 }
        return min(30, pow(2, Double(min(failures - 1, 10))))
    }
}

/// Which `streamHosts` entry to dial next. Every round starts at the first host
/// (the tailnet IP); a later host — the cached LAN hop, which DHCP may have moved
/// — is tried only straight after the ones before it failed, so a working tailnet
/// never waits out a dead lease's handshake timeout.
public struct StreamHostRotation: Sendable, Equatable {
    /// Position in the current round.
    public private(set) var index = 0
    /// Rounds in a row where every host failed; drives the backoff.
    public private(set) var failedRounds = 0

    public init() {}

    public func host(in candidates: [String]) -> String? {
        candidates.isEmpty ? nil : candidates[min(index, candidates.count - 1)]
    }

    /// Record how the attempt on `host(in:)` ended; returns the wait before the next one.
    public mutating func record(opened: Bool, candidateCount: Int) -> TimeInterval {
        if opened {
            // A link that worked and dropped: the same first-choice round again.
            index = 0
            failedRounds = 0
            return FoxgloveProtocol.reconnectDelay(afterFailures: 0)
        }
        index += 1
        if index < candidateCount {
            return 0
        }
        index = 0
        failedRounds += 1
        return FoxgloveProtocol.reconnectDelay(afterFailures: failedRounds)
    }
}
