import Foundation
import LabCommandCore

func testFoxgloveServerMessages() {
    let info = FoxgloveProtocol.parseServerMessage(
        #"{"op":"serverInfo","name":"foxglove_bridge","capabilities":["time"],"supportedEncodings":["cdr"]}"#
    )
    expect(info == .serverInfo(name: "foxglove_bridge"), "serverInfo carries the server name, got \(String(describing: info))")
    expect(FoxgloveProtocol.parseServerMessage(#"{"op":"serverInfo"}"#) == .serverInfo(name: ""), "serverInfo without a name")

    let advertise = FoxgloveProtocol.parseServerMessage("""
    {"op":"advertise","channels":[
      {"id":7,"topic":"/livox/lidar","encoding":"cdr","schemaName":"sensor_msgs/msg/PointCloud2","schema":"...","schemaEncoding":"ros2msg"},
      {"id":12,"topic":"/lidar_points","encoding":"cdr","schemaName":"sensor_msgs/msg/PointCloud2"},
      {"id":13,"topic":"/tf","encoding":"json"},
      {"topic":"/no_id","encoding":"cdr"},
      {"id":-1,"topic":"/negative","encoding":"cdr"},
      {"id":14,"encoding":"cdr"}
    ]}
    """)
    expect(advertise == .advertise([
        FoxgloveProtocol.Channel(id: 7, topic: "/livox/lidar", encoding: "cdr", schemaName: "sensor_msgs/msg/PointCloud2"),
        FoxgloveProtocol.Channel(id: 12, topic: "/lidar_points", encoding: "cdr", schemaName: "sensor_msgs/msg/PointCloud2"),
        FoxgloveProtocol.Channel(id: 13, topic: "/tf", encoding: "json", schemaName: ""),
    ]), "advertise keeps well-formed channels and drops ones without an id or topic, got \(String(describing: advertise))")
    expect(FoxgloveProtocol.parseServerMessage(#"{"op":"advertise"}"#) == .advertise([]), "advertise without channels is empty")

    expect(
        FoxgloveProtocol.parseServerMessage(#"{"op":"unadvertise","channelIds":[7,12]}"#) == .unadvertise([7, 12]),
        "unadvertise lists channel ids"
    )
    expect(
        FoxgloveProtocol.parseServerMessage(#"{"op":"status","level":1,"message":"slow client"}"#)
            == .status(level: 1, message: "slow client"),
        "status level + message"
    )
    expect(
        FoxgloveProtocol.parseServerMessage(#"{"op":"parameterValues","parameters":[]}"#) == .other(op: "parameterValues"),
        "unneeded ops are recognised, not errors"
    )

    for junk in ["", "not json", "[1,2,3]", #"{"channels":[]}"#, #"{"op":7}"#] {
        expect(FoxgloveProtocol.parseServerMessage(junk) == nil, "junk text frame is nil: \(junk)")
    }
}

func testFoxgloveChannelSchemaCheck() {
    func channel(_ schema: String) -> FoxgloveProtocol.Channel {
        FoxgloveProtocol.Channel(id: 1, topic: "/livox/lidar", encoding: "cdr", schemaName: schema)
    }
    expect(channel("sensor_msgs/msg/PointCloud2").carriesPointCloud2, "ROS 2 PointCloud2 schema")
    expect(channel("sensor_msgs/PointCloud2").carriesPointCloud2, "ROS 1 style PointCloud2 schema")
    expect(channel("").carriesPointCloud2, "unnamed schema gets the benefit of the doubt")
    expect(!channel("livox_ros_driver2/msg/CustomMsg").carriesPointCloud2, "Livox CustomMsg is not a PointCloud2")
    expect(!channel("sensor_msgs/msg/Imu").carriesPointCloud2, "Imu is not a PointCloud2")
}

func testFoxgloveSubscribeMessage() {
    let text = FoxgloveProtocol.subscribeMessage([
        FoxgloveProtocol.Subscription(id: 1, channelId: 7),
        FoxgloveProtocol.Subscription(id: 2, channelId: 12),
    ])
    expect(
        text == #"{"op":"subscribe","subscriptions":[{"id":1,"channelId":7},{"id":2,"channelId":12}]}"#,
        "subscribe wire text, got \(text)"
    )

    // And it must be JSON a server can read back.
    let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any]
    let subscriptions = object?["subscriptions"] as? [[String: Any]]
    expect(object?["op"] as? String == "subscribe", "op is subscribe")
    expect(subscriptions?.count == 2, "two subscriptions")
    expect(subscriptions?[1]["id"] as? Int == 2 && subscriptions?[1]["channelId"] as? Int == 12, "ids are JSON numbers")
}

private func messageDataFrame(opcode: UInt8, subscriptionId: UInt32, timestamp: UInt64, payload: [UInt8]) -> Data {
    var bytes: [UInt8] = [opcode]
    withUnsafeBytes(of: subscriptionId.littleEndian) { bytes.append(contentsOf: $0) }
    withUnsafeBytes(of: timestamp.littleEndian) { bytes.append(contentsOf: $0) }
    bytes.append(contentsOf: payload)
    return Data(bytes)
}

func testFoxgloveMessageData() {
    let payload: [UInt8] = [0x00, 0x01, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF]
    let frame = messageDataFrame(opcode: 0x01, subscriptionId: 0x0403_0201, timestamp: 1_700_000_000_123_456_789, payload: payload)
    expect(frame.count == 13 + payload.count, "header is 1 + 4 + 8 bytes")
    expect(Array(frame[1..<5]) == [0x01, 0x02, 0x03, 0x04], "subscription id is little-endian on the wire")

    let message = FoxgloveProtocol.parseMessageData(frame)
    expect(message?.subscriptionId == 0x0403_0201, "subscription id")
    expect(message?.receiveTimestampNs == 1_700_000_000_123_456_789, "receive timestamp")
    expect(message.map { Array($0.payload) } == payload, "payload starts at byte 13")

    // A frame that is itself a slice of a larger buffer: indices don't start at 0.
    let padded = Data([0xAA, 0xBB, 0xCC]) + frame + Data([0xFF])
    let slice = padded[3..<(3 + frame.count)]
    expect(slice.startIndex == 3, "slice keeps its parent's indices")
    let sliced = FoxgloveProtocol.parseMessageData(slice)
    expect(sliced?.subscriptionId == 0x0403_0201, "slice: subscription id")
    expect(sliced?.receiveTimestampNs == 1_700_000_000_123_456_789, "slice: timestamp")
    expect(sliced.map { Array($0.payload) } == payload, "slice: payload")

    let headerOnly = messageDataFrame(opcode: 0x01, subscriptionId: 9, timestamp: 0, payload: [])
    expect(FoxgloveProtocol.parseMessageData(headerOnly)?.payload.isEmpty == true, "header-only frame has an empty payload")

    for opcode: UInt8 in [0x00, 0x02, 0x03, 0x04, 0xFF] {
        let other = messageDataFrame(opcode: opcode, subscriptionId: 1, timestamp: 2, payload: payload)
        expect(FoxgloveProtocol.parseMessageData(other) == nil, "opcode \(opcode) is ignored")
    }
    for length in 0..<13 {
        expect(FoxgloveProtocol.parseMessageData(frame.prefix(length)) == nil, "\(length)-byte frame is too short")
    }
}

func testFoxgloveMessageDataFeedsCDRDecoder() {
    // The real PointCloud2 fixture, wrapped the way foxglove_bridge would send it.
    let candidates = ["fixtures/pointclouds", "../fixtures/pointclouds", "../../fixtures/pointclouds"]
    let cwd = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    let cdr = candidates.lazy
        .compactMap { FileManager.default.contents(atPath: cwd.appendingPathComponent($0).appendingPathComponent("ros2-pointcloud2.bin").path) }
        .first
    guard let cdr else {
        expect(false, "fixtures/pointclouds/ros2-pointcloud2.bin not found")
        return
    }

    let frame = messageDataFrame(opcode: 0x01, subscriptionId: 2, timestamp: 42, payload: Array(cdr))
    guard let message = FoxgloveProtocol.parseMessageData(frame) else {
        expect(false, "wrapped fixture parses as message data")
        return
    }
    expect(message.payload.startIndex == 13, "payload is a slice, not a copy")

    let direct = try? PointCloud2CDR.decode(cdr)
    let viaSocket = try? PointCloud2CDR.decode(message.payload)
    expect(direct != nil && !(direct?.isEmpty ?? true), "fixture decodes on its own")
    expect(viaSocket == direct, "the payload slice decodes to the same cloud as the bare fixture")
}

func testFoxgloveLiveTopics() {
    // foxglove-sdk-cpp bridges refuse the upgrade (HTTP 400) without sdk.v1.
    expect(FoxgloveProtocol.subprotocols.first == "foxglove.sdk.v1", "sdk.v1 offered first")
    expect(FoxgloveProtocol.subprotocols.contains("foxglove.websocket.v1"), "legacy subprotocol still offered")
    expect(FoxgloveProtocol.liveSensor(forTopic: "/livox/lidar") == .mid360, "/livox/lidar is the Mid-360")
    expect(FoxgloveProtocol.liveSensor(forTopic: "/lidar_points") == .jt128, "/lidar_points is the JT128")
    for topic in ["/livox/imu", "/cloud_registered", "/Laser_map", "livox/lidar", "/lidar_points/extra", ""] {
        expect(FoxgloveProtocol.liveSensor(forTopic: topic) == nil, "\(topic) is not a live lidar topic")
    }
}

func testFoxgloveStreamHosts() {
    let fallback = FoxgloveProtocol.streamHosts()
    expect(fallback == ["100.118.187.99"], "with nothing cached, the Tailscale IP is the only host: \(fallback)")

    let withLAN = FoxgloveProtocol.streamHosts(lastLAN: "192.168.68.51")
    expect(withLAN == ["100.118.187.99", "192.168.68.51"], "tailnet first, then the cached LAN hop: \(withLAN)")
    expect(FoxgloveProtocol.streamHosts(lastLAN: "192.168.68.51:41641") == withLAN, "a :port on the cached hop is stripped")
    expect(FoxgloveProtocol.streamHosts(lastLAN: "100.118.187.99") == fallback, "no duplicate when the cache is the tailnet IP")
    expect(FoxgloveProtocol.streamHosts(lastLAN: "spark-ts") == fallback, "ssh aliases are not socket hosts")
    expect(FoxgloveProtocol.streamHosts(lastLAN: "spark-1f3e.tail8dfdc7.ts.net") == fallback, "DNS names are left out")

    for host in ["100.118.187.99", "192.168.1.5", "0.0.0.0", "255.255.255.255"] {
        expect(FoxgloveProtocol.isIPv4Literal(host), "\(host) is an IPv4 literal")
    }
    for host in ["spark", "spark-ts", "spark.local", "1.2.3", "1.2.3.4.5", "256.1.1.1", "1.2.3.", "1..2.3", "+1.2.3.4", "fd7a::1", ""] {
        expect(!FoxgloveProtocol.isIPv4Literal(host), "\(host) is not an IPv4 literal")
    }
}

func testFoxgloveReconnectBackoff() {
    let delays = (0...8).map { FoxgloveProtocol.reconnectDelay(afterFailures: $0) }
    expect(delays == [1, 1, 2, 4, 8, 16, 30, 30, 30], "backoff doubles from 1 s to a 30 s cap: \(delays)")
    expect(FoxgloveProtocol.reconnectDelay(afterFailures: 10_000) == 30, "stays capped however long Spark is away")
    expect(FoxgloveProtocol.reconnectDelay(afterFailures: -3) == 1, "nonsense input still waits a second")
}

func runFoxgloveProtocolTests() {
    testFoxgloveServerMessages()
    print("  Foxglove: serverInfo / advertise / unadvertise / status parse; junk is nil"); passed()
    testFoxgloveChannelSchemaCheck()
    print("  Foxglove: only PointCloud2 schemas are subscribable"); passed()
    testFoxgloveSubscribeMessage()
    print("  Foxglove: subscribe wire text"); passed()
    testFoxgloveMessageData()
    print("  Foxglove: binary 0x01 header (LE sub id + timestamp), slices, other opcodes ignored"); passed()
    testFoxgloveMessageDataFeedsCDRDecoder()
    print("  Foxglove: message-data payload decodes as CDR PointCloud2"); passed()
    testFoxgloveLiveTopics()
    print("  Foxglove: /livox/lidar → Mid-360, /lidar_points → JT128"); passed()
    testFoxgloveStreamHosts()
    print("  Foxglove: stream hosts are IPv4 literals, Tailscale IP first"); passed()
    testFoxgloveReconnectBackoff()
    print("  Foxglove: reconnect backoff 1 s → 30 s"); passed()
}
