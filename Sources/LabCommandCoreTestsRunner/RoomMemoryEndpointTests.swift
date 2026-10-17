import Foundation
import LabCommandCore

// The room bridge used to be `ws://100.118.187.99:8767` inside `LiveRoomClient`. These tests pin
// the replacement: a parsed, persisted endpoint whose default is a hostname, and a retarget that
// publishes `.connecting` on the new URL before it returns.

private func isolatedDefaults() -> UserDefaults {
    let name = "RoomMemoryEndpointTests.\(UUID().uuidString)"
    let defaults = UserDefaults(suiteName: name)!
    defaults.removePersistentDomain(forName: name)
    return defaults
}

private func endpointString(_ url: URL) -> String { url.absoluteString }

@MainActor
func testRoomMemoryEndpointParsesAndResolves() {
    let builtIn = RoomMemoryEndpoint.defaultURL
    expect(builtIn.scheme == "ws", "default scheme is ws")
    expect(builtIn.host == CaptureRecipe.tailscaleDNS, "default host is the MagicDNS name, not an SSH alias")
    expect(builtIn.port == 8767, "default port is the room-memory bridge")
    expect(!endpointString(builtIn).contains("100.118.187.99"), "the tailnet IP is not the built-in endpoint")
    expect(endpointString(builtIn) == "ws://\(CaptureRecipe.tailscaleDNS):8767", "canonical default \(builtIn)")

    func accept(_ raw: String, _ canonical: String) {
        switch RoomMemoryEndpoint.parse(raw) {
        case .success(let url):
            expect(endpointString(url) == canonical, "\(raw) canonicalizes to \(canonical), got \(url)")
        case .failure(let error):
            expect(false, "\(raw) should parse, got \(error.message)")
        }
    }
    func reject(_ raw: String) {
        if case .success(let url) = RoomMemoryEndpoint.parse(raw) {
            expect(false, "\(raw) should be rejected, got \(url)")
        }
    }

    accept("ws://spark.example:8767", "ws://spark.example:8767")
    accept("wss://spark.example:8767", "wss://spark.example:8767")
    accept("  ws://spark.example:8767/  ", "ws://spark.example:8767")
    accept("WS://Spark.Example:8767/", "ws://Spark.Example:8767")
    accept("ws://192.0.2.10:8767", "ws://192.0.2.10:8767")
    reject("")
    reject("   ")
    reject("http://host:8767")
    reject("https://host:8767")
    reject("ws://host")
    reject("ws://host:0")
    reject("ws://host:65536")
    reject("ws://user:pass@host:8767")
    reject("ws://host:8767/room")
    reject("ws://host:8767?x=1")
    reject("ws://host:8767#frag")

    let empty = isolatedDefaults()
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: [:], defaults: empty)) == endpointString(builtIn),
        "empty defaults resolve to the built-in endpoint"
    )

    let saved = isolatedDefaults()
    switch RoomMemoryEndpoint.parse("ws://saved.example:8767/") {
    case .success(let url):
        RoomMemoryEndpoint.save(url, to: saved)
    case .failure(let error):
        expect(false, "saved fixture parses: \(error.message)")
    }
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: [:], defaults: saved)) == "ws://saved.example:8767",
        "a valid saved string wins over the default"
    )
    expect(saved.string(forKey: RoomMemoryEndpoint.defaultsKey) == "ws://saved.example:8767", "save writes the canonical string")

    let broken = isolatedDefaults()
    broken.set("not a url", forKey: RoomMemoryEndpoint.defaultsKey)
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: [:], defaults: broken)) == endpointString(builtIn),
        "an invalid saved string falls back to the default"
    )

    let both = isolatedDefaults()
    both.set("ws://saved.example:9", forKey: RoomMemoryEndpoint.defaultsKey)
    let env = ["LABCOMMAND_ROOM_URL": "  wss://env.example:443/  "]
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: env, defaults: both)) == "wss://env.example:443",
        "a valid environment URL wins over saved defaults"
    )
    expect(both.string(forKey: RoomMemoryEndpoint.defaultsKey) == "ws://saved.example:9", "the environment URL is not written")

    let badEnv = isolatedDefaults()
    badEnv.set("ws://saved.example:9", forKey: RoomMemoryEndpoint.defaultsKey)
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: ["LABCOMMAND_ROOM_URL": "http://nope"], defaults: badEnv))
            == "ws://saved.example:9",
        "an invalid environment value falls through to the saved string"
    )
    let badEverything = isolatedDefaults()
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: ["LABCOMMAND_ROOM_URL": ""], defaults: badEverything))
            == endpointString(builtIn),
        "an invalid environment value with nothing saved uses the default"
    )

    let round = isolatedDefaults()
    let store = RoomMemoryEndpointStore(environment: [:], defaults: round)
    expect(store.commit("ws://round.example:8767/", retargeting: nil) == nil, "commit accepts a trailing slash")
    expect(endpointString(store.current) == "ws://round.example:8767", "commit updates the current endpoint")
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: [:], defaults: round)) == "ws://round.example:8767",
        "save then resolve round-trips the canonical string"
    )
}

/// One current endpoint for the process. An invalid commit touches neither the preference nor the socket.
/// Reopening Settings reads `current`, which stays on the commit even when the process started from the environment.
@MainActor
func testRoomMemoryCommitKeepsTheCurrentEndpoint() {
    let defaults = isolatedDefaults()
    defaults.set("ws://saved.example:8767", forKey: RoomMemoryEndpoint.defaultsKey)
    let store = RoomMemoryEndpointStore(
        environment: ["LABCOMMAND_ROOM_URL": "ws://env.example:8767"],
        defaults: defaults
    )
    expect(endpointString(store.current) == "ws://env.example:8767", "init reads the environment once")
    expect(defaults.string(forKey: RoomMemoryEndpoint.defaultsKey) == "ws://saved.example:8767", "init does not write the environment")

    let client = LiveRoomClient(url: store.current)
    expect(client.connection == .off, "a client that has not started is off")
    let rejected = store.commit("ws://host", retargeting: client)
    expect(rejected != nil, "a missing port is rejected")
    expect(endpointString(store.current) == "ws://env.example:8767", "a rejected commit keeps the current endpoint")
    expect(defaults.string(forKey: RoomMemoryEndpoint.defaultsKey) == "ws://saved.example:8767", "a rejected commit does not write")
    expect(endpointString(client.url) == "ws://env.example:8767", "a rejected commit does not retarget")
    expect(client.connection == .off, "a rejected commit leaves the connection alone")

    expect(store.commit("ws://committed.example:9/", retargeting: client) == nil, "a later valid commit is saved")
    expect(endpointString(store.current) == "ws://committed.example:9", "commit replaces the current endpoint")
    expect(endpointString(client.url) == "ws://committed.example:9", "commit retargets the client")
    expect(client.connection == .off, "retarget before start does not connect")
    expect(defaults.string(forKey: RoomMemoryEndpoint.defaultsKey) == "ws://committed.example:9", "commit persists the canonical string")

    // Reopening Settings copies `current`. Resolving again would show the environment URL, which is stale.
    let reopened = endpointString(store.current)
    expect(reopened == "ws://committed.example:9", "reopen reads the committed endpoint")
    let again = RoomMemoryEndpoint.resolve(
        environment: ["LABCOMMAND_ROOM_URL": "ws://env.example:8767"],
        defaults: defaults
    )
    expect(endpointString(again) == "ws://env.example:8767", "a new process with the environment set uses the environment again")
    expect(endpointString(store.current) == "ws://committed.example:9", "this process does not re-resolve on reopen")
}

@MainActor
private func endpointSpin(_ seconds: TimeInterval, until done: () -> Bool = { false }) {
    let deadline = Date().addingTimeInterval(seconds)
    while Date() < deadline, !done() {
        RunLoop.main.run(until: Date().addingTimeInterval(0.02))
    }
}

@MainActor
private func expectDialing(_ client: LiveRoomClient, _ url: URL, _ context: String) {
    switch client.connection {
    case .off:
        expect(false, "\(context): connection is off")
    case .waiting(let retry, let reason):
        expect(false, "\(context): still waiting \(retry)s (\(reason))")
    case .connecting(let dial, let attempt):
        expect(dial.absoluteString == url.absoluteString, "\(context): connecting to \(dial), want \(url)")
        expect(attempt == 1, "\(context): a retarget starts at attempt 1, got \(attempt)")
    case .connected:
        expect(client.url.absoluteString == url.absoluteString, "\(context): connected, url \(client.url) want \(url)")
    }
}

@MainActor
private func brick(_ i: Int16, in mirror: RoomMemoryMirror) -> Bool {
    mirror.bricks.keys.contains(RoomMemory.BrickCoordinate(i: i, j: 0, k: 0))
}

@MainActor
private func legacyBridge(seed: UInt64) -> LRFakeBridge {
    LRFakeBridge(
        capabilities: ["connectionGraph"],
        serviceType: nil,
        publisher: LRPublisher(seed: seed, generation: 100 + seed, legacy: true)
    )
}

@MainActor
private func waitForBrick(_ i: Int16, on bridge: LRFakeBridge, client: LiveRoomClient) {
    endpointSpin(4) {
        if case .connected(let server) = client.connection { return !server.memoryTopics.isEmpty }
        return false
    }
    expect({ if case .connected = client.connection { return true } else { return false } }(), "connected before brick \(i)")
    bridge.sendBrick(i)
    endpointSpin(3) { brick(i, in: client.mirror) }
    expect(brick(i, in: client.mirror), "brick \(i) applied, have \(client.mirror.bricks.count)")
}

@MainActor
func testLiveRoomRetarget() {
    let idleA = URL(string: "ws://idle-a.example:8767")!
    let idleB = URL(string: "ws://idle-b.example:8767")!
    let idle = LiveRoomClient(url: idleA, roomId: "lab")
    idle.retarget(idleB)
    expect(idle.url.absoluteString == idleB.absoluteString, "retarget before start stores the URL")
    expect(idle.connection == .off, "retarget before start stays off")
    expect(idle.mirror.roomId == "lab", "retarget keeps the room id passed to init")

    let bridgeA = legacyBridge(seed: 1)
    bridgeA.start()
    let client = LiveRoomClient(url: bridgeA.url, roomId: "lab")
    client.start()
    waitForBrick(1, on: bridgeA, client: client)
    let kept = client.mirror.bricks
    client.retarget(bridgeA.url)
    expect(client.mirror.bricks == kept, "the same URL does not clear bricks")
    expect({ if case .connected = client.connection { return true } else { return false } }(), "the same URL does not drop the connection")

    let bridgeB = legacyBridge(seed: 2)
    bridgeB.start()
    client.retarget(bridgeB.url)
    expectDialing(client, bridgeB.url, "A to B")
    expect(!brick(1, in: client.mirror), "A's bricks are gone as soon as the target changes")
    waitForBrick(2, on: bridgeB, client: client)
    expect(!brick(1, in: client.mirror), "after B's handshake the mirror is B's map")
    expect(client.url.absoluteString == bridgeB.url.absoluteString, "url is B")

    client.stop()
    bridgeA.stop()
    bridgeB.stop()
}

@MainActor
func testLiveRoomRetargetDuringHandshakeBackoffAndRapidChanges() {
    let held = legacyBridge(seed: 3)
    held.holdHandshake = true
    held.start()
    let client = LiveRoomClient(url: held.url)
    client.start()
    endpointSpin(3) { held.clientCount > 0 }
    expect(held.clientCount > 0, "handshake retarget: the first bridge accepted the socket")
    expect({ if case .connecting = client.connection { return true } else { return false } }(), "handshake retarget starts in .connecting")

    let next = legacyBridge(seed: 4)
    next.start()
    client.retarget(next.url)
    expectDialing(client, next.url, "during handshake")
    waitForBrick(4, on: next, client: client)
    expect(client.url.absoluteString == next.url.absoluteString, "handshake retarget lands on the new bridge")
    held.stop()

    client.stop()
    let refused = URL(string: "ws://127.0.0.1:1")!
    let backoff = LiveRoomClient(url: refused)
    backoff.start()
    endpointSpin(8) { if case .waiting = backoff.connection { return true } else { return false } }
    guard case .waiting = backoff.connection else {
        expect(false, "backoff retarget: expected .waiting, got \(backoff.connection)")
        backoff.stop()
        next.stop()
        return
    }
    let resumed = legacyBridge(seed: 5)
    resumed.start()
    backoff.retarget(resumed.url)
    expectDialing(backoff, resumed.url, "during backoff")
    waitForBrick(5, on: resumed, client: backoff)
    next.stop()
    resumed.stop()
    backoff.stop()

    let a = legacyBridge(seed: 6)
    let b = legacyBridge(seed: 7)
    let c = legacyBridge(seed: 8)
    a.start(); b.start(); c.start()
    let rapid = LiveRoomClient(url: a.url)
    rapid.start()
    waitForBrick(6, on: a, client: rapid)
    rapid.retarget(b.url)
    rapid.retarget(c.url)
    expectDialing(rapid, c.url, "A to B to C")
    expect(rapid.url.absoluteString == c.url.absoluteString, "rapid retarget keeps C")
    a.sendBrick(16)
    b.sendBrick(17)
    waitForBrick(8, on: c, client: rapid)
    endpointSpin(1.2) { false }
    expect(brick(8, in: rapid.mirror), "C's brick stays")
    expect(!brick(6, in: rapid.mirror) && !brick(16, in: rapid.mirror) && !brick(17, in: rapid.mirror),
           "A and B cannot change C's map, have \(rapid.mirror.bricks.keys.map(\.i).sorted())")
    expect(rapid.url.absoluteString == c.url.absoluteString, "url stays C after the old bridges send")
    expect({ if case .off = rapid.connection { return false } else { return true } }(), "C is not dropped to off")
    rapid.stop()
    a.stop(); b.stop(); c.stop()
}

func testRoomMemorySettingsSourcesAndATS() {
    let root = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    func text(_ path: String) -> String {
        let url = root.appendingPathComponent(path)
        guard let body = try? String(contentsOf: url, encoding: .utf8) else {
            expect(false, "missing \(path)")
            return ""
        }
        return body
    }

    let client = text("Sources/LabCommandCore/LiveRoom/LiveRoomClient.swift")
    expect(!client.contains("100.118.187.99"), "LiveRoomClient has no tailnet IP")
    expect(!client.contains("defaultURL"), "LiveRoomClient does not own a default URL")
    expect(client.contains("func retarget"), "retarget lives on the client")

    let endpoint = text("Sources/LabCommandCore/RoomMemoryEndpoint.swift")
    expect(!endpoint.contains("100.118.187.99"), "the endpoint type has no tailnet IP")
    expect(endpoint.contains("CaptureRecipe.tailscaleDNS"), "the default host is the MagicDNS name")

    let settings = text("Sources/LabCommand/SettingsView.swift")
    expect(settings.contains("Rig diagnostics"), "recorder rows sit under Rig diagnostics")
    expect(settings.contains("Room memory"), "the endpoint group is Room memory")
    expect(settings.contains("Room memory endpoint"), "the field has its accessibility label")
    expect(settings.contains("commitRoomEndpoint"), "the field commits through AppState")
    expect(settings.contains("roomEndpointURL"), "the draft is the current endpoint")
    expect(!settings.contains("RoomMemoryEndpoint.resolve"), "appearing does not re-resolve the environment")
    expect(!settings.contains("100.118.187.99"), "Settings does not hardcode the room IP")

    let state = text("Sources/LabCommand/AppState.swift")
    expect(state.contains("LiveRoomClient(url:"), "AppState passes an explicit URL")
    expect(state.contains("RoomMemoryEndpointStore"), "one store owns the current endpoint")
    expect(state.contains("commitRoomEndpoint"), "one commit method")
    expect(state.contains("retargeting: room"), "commit retargets the live client")
    expect(!state.contains("100.118.187.99"), "AppState does not hardcode the room IP")

    let script = text("scripts/live-room-shots.sh")
    expect(script.contains("export LABCOMMAND_ROOM_URL=\"ws://100.118.187.99:8767\""), "live shots pin the grep URL")
    expect(script.contains("if [[ \"$MODE\" == \"live\" ]]; then"), "the pin is the live mode only")

    let plistURL = root.appendingPathComponent("Info.plist")
    guard let data = try? Data(contentsOf: plistURL),
          let plist = try? PropertyListSerialization.propertyList(from: data, format: nil) as? [String: Any],
          let ats = plist["NSAppTransportSecurity"] as? [String: Any] else {
        expect(false, "Info.plist ATS dictionary")
        return
    }
    expect(ats["NSAllowsArbitraryLoads"] as? Bool == true, "arbitrary loads is on")
    expect(ats["NSAllowsLocalNetworking"] == nil, "NSAllowsLocalNetworking would mask arbitrary loads")
    expect(ats["NSAllowsArbitraryLoadsForMedia"] == nil, "the media exception would mask arbitrary loads")
    expect(ats["NSAllowsArbitraryLoadsInWebContent"] == nil, "the web-content exception would mask arbitrary loads")
    let domains = ats["NSExceptionDomains"] as? [String: Any]
    expect(domains?["100.118.187.99"] != nil && domains?["192.168.68.65"] != nil, "the existing exception domains stay")
}

func runRoomMemoryEndpointTests() {
    MainActor.assumeIsolated {
        testRoomMemoryEndpointParsesAndResolves()
        print("  room endpoint: parse, canonical form, resolve order, save round-trip"); passed()
        testRoomMemoryCommitKeepsTheCurrentEndpoint()
        print("  room endpoint: invalid commit keeps the preference and the socket; reopen reads current"); passed()
        testLiveRoomRetarget()
        print("  room endpoint: retarget swaps bridges, publishes connecting, same URL keeps bricks"); passed()
        testLiveRoomRetargetDuringHandshakeBackoffAndRapidChanges()
        print("  room endpoint: retarget during handshake and backoff; A→B→C ignores the old bridges"); passed()
    }
    testRoomMemorySettingsSourcesAndATS()
    print("  room endpoint: Settings, AppState, live-shot pin, and ATS keys"); passed()
}
