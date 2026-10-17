import Foundation
import LabCommandCore
import Network

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
    expect(store.commit("wss://round.example:8767/", retargeting: nil) == .saved, "commit accepts a trailing slash")
    expect(endpointString(store.current) == "wss://round.example:8767", "commit updates the current endpoint")
    expect(
        endpointString(RoomMemoryEndpoint.resolve(environment: [:], defaults: round)) == "wss://round.example:8767",
        "save then resolve round-trips the canonical string"
    )

    let secureSaved = isolatedDefaults()
    secureSaved.set("wss://saved.example:443", forKey: RoomMemoryEndpoint.defaultsKey)
    expect(
        endpointString(RoomMemoryEndpoint.resolve(
            environment: ["LABCOMMAND_ROOM_URL": "ws://10.0.0.5:8767"],
            defaults: secureSaved
        )) == "wss://saved.example:443",
        "an environment ws:// does not replace a saved wss://"
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
    if case .rejected = rejected {
    } else {
        expect(false, "a missing port is rejected, got \(rejected)")
    }
    expect(endpointString(store.current) == "ws://env.example:8767", "a rejected commit keeps the current endpoint")
    expect(defaults.string(forKey: RoomMemoryEndpoint.defaultsKey) == "ws://saved.example:8767", "a rejected commit does not write")
    expect(endpointString(client.url) == "ws://env.example:8767", "a rejected commit does not retarget")
    expect(client.connection == .off, "a rejected commit leaves the connection alone")

    expect(store.commit("wss://committed.example:9/", retargeting: client) == .saved, "a later valid commit is saved")
    expect(endpointString(store.current) == "wss://committed.example:9", "commit replaces the current endpoint")
    expect(endpointString(client.url) == "wss://committed.example:9", "commit retargets the client")
    expect(client.connection == .off, "retarget before start does not connect")
    expect(defaults.string(forKey: RoomMemoryEndpoint.defaultsKey) == "wss://committed.example:9", "commit persists the canonical string")

    // Reopening Settings copies `current`. Resolving again would show the environment URL, which is stale.
    let reopened = endpointString(store.current)
    expect(reopened == "wss://committed.example:9", "reopen reads the committed endpoint")
    let again = RoomMemoryEndpoint.resolve(
        environment: ["LABCOMMAND_ROOM_URL": "ws://env.example:8767"],
        defaults: defaults
    )
    expect(endpointString(again) == "wss://committed.example:9", "a new process does not let an environment ws:// replace the saved wss://")
    let wsSaved = isolatedDefaults()
    wsSaved.set("ws://10.0.0.9:8767", forKey: RoomMemoryEndpoint.defaultsKey)
    expect(
        endpointString(RoomMemoryEndpoint.resolve(
            environment: ["LABCOMMAND_ROOM_URL": "ws://192.168.1.9:8767"],
            defaults: wsSaved
        )) == "ws://192.168.1.9:8767",
        "an environment ws:// still wins over a saved ws://"
    )
    expect(endpointString(store.current) == "wss://committed.example:9", "this process does not re-resolve on reopen")
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

func testCleartextHostPolicy() {
    func allow(_ host: String) {
        expect(RoomMemoryEndpoint.allowsCleartextHost(host), "\(host) may use ws://")
    }
    func deny(_ host: String) {
        expect(!RoomMemoryEndpoint.allowsCleartextHost(host), "\(host) may not use ws://")
    }

    allow("10.0.0.1")
    allow("10.255.255.255")
    allow("172.16.0.0")
    allow("172.31.255.255")
    deny("172.15.255.255")
    deny("172.32.0.1")
    allow("192.168.0.1")
    allow("192.168.68.65")
    deny("192.169.0.1")
    allow("127.0.0.1")
    allow("169.254.10.20")
    allow("100.64.0.0")
    allow("100.118.187.99")
    allow("100.127.255.255")
    deny("100.63.255.255")
    deny("100.128.0.1")
    deny("8.8.8.8")
    deny("1.2.3.4")
    deny("192.0.2.10")
    deny("2130706433")
    deny("localhost")
    deny("example.com")
    deny("notts.net")
    deny("ts.net")
    deny("ts.net.evil.com")
    deny("spark.ts.net.evil.com")
    allow("spark-1f3e.tail8dfdc7.ts.net")
    allow("Spark-1F3E.TAIL8DFDC7.TS.NET")
    allow("foo.bar.ts.net.")
    deny(".ts.net")
    deny("foo..ts.net")
    allow("::1")
    allow("[::1]")
    allow("fd00::1")
    allow("fe80::1")
    deny("2001:db8::1")
    deny("::ffff:127.0.0.1")

    let magic = RoomMemoryEndpoint.defaultURL
    switch RoomMemoryEndpoint.admit(magic, allowCleartext: false) {
    case .blocked(let message):
        expect(message.contains("Cleartext WebSocket is off"), "the default is blocked while the switch is off, got \(message)")
        expect(message.contains(CaptureRecipe.tailscaleDNS), "the block names the MagicDNS host")
        expect(!message.contains(CaptureRecipe.tailscaleIP), "the block does not substitute the tailnet IP")
    case .allowed:
        expect(false, "the default ws:// must not dial while cleartext is off")
    }
    expect(RoomMemoryEndpoint.admit(magic, allowCleartext: true) == .allowed, "the MagicDNS name is an allowed cleartext host")

    switch RoomMemoryEndpoint.parse("wss://example.com:443") {
    case .success(let secure):
        expect(RoomMemoryEndpoint.admit(secure, allowCleartext: false) == .allowed, "wss:// does not need the cleartext switch")
    case .failure(let error):
        expect(false, "wss fixture parses: \(error.message)")
    }
    switch RoomMemoryEndpoint.parse("ws://example.com:443") {
    case .success(let publicWS):
        if case .blocked(let message) = RoomMemoryEndpoint.admit(publicWS, allowCleartext: true) {
            expect(message.contains("example.com"), "a public ws:// names the host, got \(message)")
            expect(message.contains("not allowed"), "a public ws:// is refused even with the switch on")
        } else {
            expect(false, "a public ws:// must be blocked")
        }
    case .failure(let error):
        expect(false, "public ws fixture parses: \(error.message)")
    }
    switch RoomMemoryEndpoint.parse("ws://10.1.2.3:8767") {
    case .success(let lan):
        if case .blocked(let message) = RoomMemoryEndpoint.admit(lan, allowCleartext: false) {
            expect(message.contains("Cleartext WebSocket is off"), "an allowed host still needs the switch, got \(message)")
        } else {
            expect(false, "ws:// to a private host is blocked while the switch is off")
        }
        expect(RoomMemoryEndpoint.admit(lan, allowCleartext: true) == .allowed, "a private host dials once the switch is on")
    case .failure(let error):
        expect(false, "lan fixture parses: \(error.message)")
    }

    let blockedScreen = LiveRoomScreen.make(
        mirror: RoomMemoryMirror(),
        connection: .off,
        requestPathExists: false,
        started: false,
        now: 10,
        endpointBlock: RoomMemoryEndpoint.cleartextDisabledMessage(url: magic)
    )
    let detail = blockedScreen.connectionDetail ?? ""
    expect(detail.contains("Cleartext WebSocket is off"), "the Room screen shows the cleartext block, got \(detail)")
    expect(detail.contains(CaptureRecipe.tailscaleDNS), "the Room screen names the address that was not dialed")
    expect(blockedScreen.canvasSentence != "Room memory unavailable", "the block is not hidden behind the off sentence")
    expect(blockedScreen.banner == nil, "a socket that never started keeps an empty banner")
}

@MainActor
func testCleartextDowngradeRequiresConfirmation() {
    let off = isolatedDefaults()
    let secure = RoomMemoryEndpointStore(
        environment: ["LABCOMMAND_ROOM_URL": "wss://secure.example:443"],
        defaults: off
    )
    expect(endpointString(secure.current) == "wss://secure.example:443", "the secure environment URL is current")
    let bypass = secure.commit("ws://10.0.0.8:8767", retargeting: nil, confirmCleartextDowngrade: true)
    if case .rejected(let message) = bypass {
        expect(message.contains("Cleartext WebSocket is off"), "confirmation does not bypass the switch, got \(message)")
    } else {
        expect(false, "ws:// with the switch off is rejected, got \(bypass)")
    }
    expect(secure.current.scheme == "wss", "a rejected downgrade keeps wss://")
    expect(off.string(forKey: RoomMemoryEndpoint.defaultsKey) == nil, "a rejected downgrade does not write")

    let on = isolatedDefaults()
    on.set(true, forKey: RoomMemoryEndpoint.cleartextDefaultsKey)
    let store = RoomMemoryEndpointStore(
        environment: ["LABCOMMAND_ROOM_URL": "wss://secure.example:443"],
        defaults: on
    )
    expect(store.allowCleartext, "the saved switch is read at init")
    let pending = store.commit("ws://10.0.0.8:8767", retargeting: nil, confirmCleartextDowngrade: false)
    expect(pending == .needsCleartextConfirmation, "wss:// to ws:// asks before it is saved, got \(pending)")
    expect(store.current.scheme == "wss", "the ask does not change the current address")
    expect(on.string(forKey: RoomMemoryEndpoint.defaultsKey) == nil, "the ask does not write")

    let client = LiveRoomClient(url: store.current)
    expect(store.commit("ws://10.0.0.8:8767", retargeting: client, confirmCleartextDowngrade: true) == .saved,
           "a confirmed downgrade is saved")
    expect(endpointString(store.current) == "ws://10.0.0.8:8767", "the confirmed address is current")
    expect(endpointString(client.url) == "ws://10.0.0.8:8767", "the confirmed address retargets")
    expect(on.string(forKey: RoomMemoryEndpoint.defaultsKey) == "ws://10.0.0.8:8767", "the confirmed address is persisted")

    let publicRejected = store.commit("ws://example.com:443", retargeting: client, confirmCleartextDowngrade: true)
    if case .rejected(let message) = publicRejected {
        expect(message.contains("example.com") && message.contains("not allowed"), "a public host stays refused, got \(message)")
    } else {
        expect(false, "a public ws:// is rejected with the switch on, got \(publicRejected)")
    }
    expect(endpointString(store.current) == "ws://10.0.0.8:8767", "a refused host does not replace the current address")
    expect(endpointString(client.url) == "ws://10.0.0.8:8767", "a refused host does not retarget")

    let unsafeLaunch = isolatedDefaults()
    let refused = RoomMemoryEndpointStore(
        environment: ["LABCOMMAND_ROOM_URL": "ws://example.com:8767"],
        defaults: unsafeLaunch
    )
    expect(refused.current.host == "example.com", "resolve still records the environment URL")
    expect(!refused.cleartextPermitted, "a public ws:// launch is not a cleartext grant")
    if case .blocked(let message) = RoomMemoryEndpoint.admit(refused.current, allowCleartext: refused.cleartextPermitted) {
        expect(message.contains("not allowed"), "the public launch is blocked for the host, got \(message)")
    } else {
        expect(false, "a public launch must not be admitted")
    }
}

/// `LABCOMMAND_ROOM_URL` selects the address. Absent and explicit false both stay off across relaunch.
@MainActor
func testEnvironmentURLDoesNotGrantCleartext() {
    let env = ["LABCOMMAND_ROOM_URL": "ws://100.118.187.99:8767"]
    let key = RoomMemoryEndpoint.cleartextDefaultsKey

    let absent = isolatedDefaults()
    let first = RoomMemoryEndpointStore(environment: env, defaults: absent)
    expect(endpointString(first.current) == "ws://100.118.187.99:8767", "the environment URL is still the address")
    expect(!first.allowCleartext && !first.cleartextPermitted, "an absent switch is not a cleartext grant")
    expect(absent.object(forKey: key) == nil, "the launch does not write the switch")
    if case .blocked(let message) = RoomMemoryEndpoint.admit(first.current, allowCleartext: first.cleartextPermitted) {
        expect(message.contains("Cleartext WebSocket is off"), "the environment URL is not dialed, got \(message)")
    } else {
        expect(false, "an environment ws:// with no saved switch must not be admitted")
    }
    let relaunchAbsent = RoomMemoryEndpointStore(environment: env, defaults: absent)
    expect(!relaunchAbsent.allowCleartext && !relaunchAbsent.cleartextPermitted, "relaunch with an absent switch stays off")
    expect(absent.object(forKey: key) == nil, "relaunch still does not write the switch")

    let explicitOff = isolatedDefaults()
    explicitOff.set(false, forKey: key)
    let off = RoomMemoryEndpointStore(environment: env, defaults: explicitOff)
    expect(!off.allowCleartext && !off.cleartextPermitted, "an explicit off switch stays off")
    expect((explicitOff.object(forKey: key) as? Bool) == false, "the launch does not erase an explicit off")
    let relaunchOff = RoomMemoryEndpointStore(environment: env, defaults: explicitOff)
    expect(!relaunchOff.allowCleartext && !relaunchOff.cleartextPermitted, "relaunch with an explicit off switch stays off")
    expect((explicitOff.object(forKey: key) as? Bool) == false, "relaunch leaves the explicit off value in place")
    if case .blocked(let message) = RoomMemoryEndpoint.admit(relaunchOff.current, allowCleartext: relaunchOff.cleartextPermitted) {
        expect(message.contains("Cleartext WebSocket is off"), "relaunch with the environment URL still does not dial, got \(message)")
    } else {
        expect(false, "relaunch with an explicit off switch must not admit ws://")
    }

    let turned = isolatedDefaults()
    let session = RoomMemoryEndpointStore(environment: env, defaults: turned)
    session.setAllowCleartext(false)
    expect((turned.object(forKey: key) as? Bool) == false, "turning the switch off is what gets saved")
    let afterOff = RoomMemoryEndpointStore(environment: env, defaults: turned)
    expect(!afterOff.cleartextPermitted, "a later process with the same environment URL stays off")

    let savedOn = isolatedDefaults()
    savedOn.set(true, forKey: key)
    let permitted = RoomMemoryEndpointStore(environment: env, defaults: savedOn)
    expect(permitted.allowCleartext && permitted.cleartextPermitted, "a saved on switch still permits the environment URL")
    expect(RoomMemoryEndpoint.admit(permitted.current, allowCleartext: permitted.cleartextPermitted) == .allowed,
           "the environment URL dials when the saved switch is on")
}

private func redirectDecision(_ from: String, _ to: String, allowCleartext: Bool) -> RoomRedirectDecision {
    RoomMemoryEndpoint.decideRedirect(from: URL(string: from)!, to: URL(string: to)!, allowCleartext: allowCleartext)
}

func testRedirectAdmissionPolicy() {
    func refused(_ from: String, _ to: String, _ allowCleartext: Bool, _ needle: String) {
        if case .refuse(let message) = redirectDecision(from, to, allowCleartext: allowCleartext) {
            expect(message.contains(needle), "\(from) → \(to) refusal should mention \(needle), got \(message)")
        } else {
            expect(false, "\(from) → \(to) with cleartext \(allowCleartext) must be refused")
        }
    }

    refused("wss://secure.example:443", "ws://10.1.2.3:8767", true, "cleartext")
    refused("wss://secure.example:443", "ws://10.1.2.3:8767", false, "cleartext")
    refused("wss://secure.example:443", "http://10.1.2.3:8767", true, "cleartext")
    refused("https://secure.example:443", "ws://100.64.1.1:8767", true, "cleartext")
    refused("ws://10.1.2.3:8767", "ws://10.9.9.9:8767", false, "Cleartext WebSocket is off")
    refused("ws://10.1.2.3:8767", "ws://example.com:8767", true, "not allowed")
    refused("ws://10.1.2.3:8767", "ws://10.9.9.9:8767/room", true, "Refused a redirect")
    refused("wss://secure.example:443", "ws://example.com:443", true, "cleartext")

    expect(redirectDecision("ws://10.1.2.3:8767", "ws://10.9.9.9:8767", allowCleartext: true) == .follow,
           "an admitted cleartext hop is followed when the switch is on")
    expect(redirectDecision("ws://10.1.2.3:8767", "http://192.168.1.9:8767", allowCleartext: true) == .follow,
           "an http Location is judged as ws://")
    expect(redirectDecision("wss://secure.example:443", "wss://other.example:443", allowCleartext: false) == .follow,
           "wss:// to wss:// does not need the cleartext switch")
    expect(redirectDecision("wss://secure.example:443", "https://other.example:8443", allowCleartext: false) == .follow,
           "an https Location is judged as wss://")
    expect(redirectDecision("ws://10.0.0.1:8767", "wss://secure.example:443", allowCleartext: false) == .follow,
           "a cleartext to wss:// upgrade is not a downgrade")

    // URLSession keeps originalRequest on the first URL. A later hop is judged from the redirecting
    // response, and a secure hop already followed still counts when that response does not show it.
    var review = RoomRedirectChain(dialed: URL(string: "ws://10.0.0.1:8767"))
    expect(review.decide(redirecting: URL(string: "ws://10.0.0.1:8767"), to: URL(string: "wss://bridge.ts.net:8767"), allowCleartext: true) == .follow,
           "ws:// to wss:// is followed")
    if case .refuse(let dropped) = review.decide(redirecting: URL(string: "wss://bridge.ts.net:8767"), to: URL(string: "ws://10.0.0.2:8767"), allowCleartext: true) {
        expect(dropped.contains("cleartext") && dropped.contains("bridge.ts.net"),
               "ws to wss to ws names the secure hop, got \(dropped)")
        expect(!dropped.contains("Cleartext WebSocket is off"), "the refusal is the downgrade, not the switch, got \(dropped)")
    } else {
        expect(false, "ws://10.0.0.1 to wss://bridge.ts.net to ws://10.0.0.2 must be refused")
    }

    var sessionShaped = RoomRedirectChain(dialed: URL(string: "http://10.0.0.1:8767"))
    expect(sessionShaped.decide(redirecting: URL(string: "http://10.0.0.1:8767/"), to: URL(string: "https://bridge.ts.net:8767/"), allowCleartext: true) == .follow,
           "URLSession's http to https upgrade is followed")
    if case .refuse(let sessionDrop) = sessionShaped.decide(redirecting: URL(string: "https://bridge.ts.net:8767/"), to: URL(string: "ws://10.0.0.2:8767/"), allowCleartext: true) {
        expect(sessionDrop.contains("cleartext") && sessionDrop.contains("bridge.ts.net"),
               "https to ws after an upgrade is refused, got \(sessionDrop)")
    } else {
        expect(false, "a URLSession-shaped https hop must refuse the later ws://")
    }

    var stuck = RoomRedirectChain(dialed: URL(string: "http://10.0.0.1:8767"))
    expect(stuck.decide(redirecting: URL(string: "http://10.0.0.1:8767/"), to: URL(string: "https://bridge.ts.net:8767/"), allowCleartext: true) == .follow,
           "the upgrade is followed before the stuck response URL")
    if case .refuse(let stuckDrop) = stuck.decide(redirecting: URL(string: "http://10.0.0.1:8767/"), to: URL(string: "ws://10.0.0.2:8767"), allowCleartext: true) {
        expect(stuckDrop.contains("cleartext") && stuckDrop.contains("bridge.ts.net"),
               "a response URL stuck on the original cleartext hop still refuses, got \(stuckDrop)")
    } else {
        expect(false, "secure-hop history must refuse ws:// when the response URL stays the original")
    }
    expect(stuck.secureHop?.host == "bridge.ts.net", "the followed https hop stays on the chain, got \(String(describing: stuck.secureHop))")

    var blind = RoomRedirectChain(dialed: URL(string: "ws://10.0.0.1:8767"))
    expect(blind.decide(redirecting: URL(string: "ws://10.0.0.1:8767"), to: URL(string: "wss://bridge.ts.net:8767"), allowCleartext: false) == .follow,
           "an upgrade does not need the cleartext switch")
    if case .refuse = blind.decide(redirecting: nil, to: URL(string: "ws://10.0.0.2:8767"), allowCleartext: true) {
    } else {
        expect(false, "a missing redirect URL after a secure hop still refuses cleartext")
    }

    var dialedSecure = RoomRedirectChain(dialed: URL(string: "https://secure.example:443"))
    if case .refuse(let dialedDrop) = dialedSecure.decide(redirecting: nil, to: URL(string: "ws://10.1.2.3:8767"), allowCleartext: true) {
        expect(dialedDrop.contains("cleartext"), "a secure dial refuses cleartext when the response URL is missing, got \(dialedDrop)")
    } else {
        expect(false, "https dialed must refuse a cleartext redirect")
    }

    var plain = RoomRedirectChain(dialed: URL(string: "ws://10.1.2.3:8767"))
    expect(plain.decide(redirecting: URL(string: "ws://10.1.2.3:8767"), to: URL(string: "ws://10.9.9.9:8767"), allowCleartext: true) == .follow,
           "the first cleartext hop is followed")
    expect(plain.decide(redirecting: URL(string: "http://10.9.9.9:8767/"), to: URL(string: "http://192.168.1.9:8767"), allowCleartext: true) == .follow,
           "a cleartext chain is not a downgrade")
    expect(plain.secureHop == nil, "a cleartext chain records no secure hop")

    var secureChain = RoomRedirectChain(dialed: URL(string: "wss://secure.example:443"))
    expect(secureChain.decide(redirecting: URL(string: "wss://secure.example:443"), to: URL(string: "https://other.example:8443"), allowCleartext: false) == .follow,
           "wss to https is followed with cleartext off")
    expect(secureChain.decide(redirecting: URL(string: "https://other.example:8443/"), to: URL(string: "wss://third.example:443"), allowCleartext: false) == .follow,
           "a secure chain does not need the cleartext switch")
}

/// Speaks raw HTTP and answers the WebSocket handshake with one redirect.
@MainActor
private final class LRRedirector {
    let location: String
    private(set) var hits = 0
    private(set) var port: UInt16 = 0
    private var listener: NWListener?
    private var connections: [NWConnection] = []

    init(location: String) {
        self.location = location
    }

    func start() {
        let parameters = NWParameters.tcp
        parameters.requiredInterfaceType = .loopback
        do {
            listener = try NWListener(using: parameters, on: .any)
        } catch {
            expect(false, "redirector listens: \(error)")
            return
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
        expect(port != 0, "redirector has a port")
    }

    func stop() {
        connections.forEach { $0.cancel() }
        listener?.cancel()
    }

    private func accept(_ connection: NWConnection) {
        hits += 1
        connections.append(connection)
        connection.stateUpdateHandler = { [weak self] state in
            guard case .ready = state else { return }
            MainActor.assumeIsolated { self?.answer(connection) }
        }
        connection.start(queue: .main)
    }

    private func answer(_ connection: NWConnection) {
        readHeader(on: connection, into: Data())
    }

    private func readHeader(on connection: NWConnection, into buffer: Data) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self] data, _, isComplete, error in
            MainActor.assumeIsolated {
                guard let self else { return }
                var next = buffer
                if let data { next.append(data) }
                let ready = next.range(of: Data("\r\n\r\n".utf8)) != nil || next.count >= 8192 || isComplete || error != nil
                if ready {
                    self.respond(on: connection)
                } else {
                    self.readHeader(on: connection, into: next)
                }
            }
        }
    }

    private func respond(on connection: NWConnection) {
        let header = "HTTP/1.1 302 Found\r\nLocation: \(location)\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
        connection.send(content: Data(header.utf8), completion: .contentProcessed { _ in
            connection.cancel()
        })
    }
}

/// Bytes read from a fixture process. The handler is not on the main actor.
private final class LRPipeBuffer: @unchecked Sendable {
    private let lock = NSLock()
    private var text = ""

    func append(_ data: Data) {
        guard !data.isEmpty, let piece = String(data: data, encoding: .utf8) else { return }
        lock.lock()
        text += piece
        lock.unlock()
    }

    func snapshot() -> String {
        lock.lock()
        defer { lock.unlock() }
        return text
    }
}

/// One TLS hop. `URLSession` follows `https://127.0.0.1` only when the test asks the client to
/// trust the fixture certificate. The process exits when its stdin closes.
@MainActor
private final class LRSecureRedirector {
    let location: String
    private(set) var hits = 0
    private(set) var port: UInt16 = 0
    private var process: Process?
    private var input: Pipe?
    private var outputHandle: FileHandle?
    private var errorHandle: FileHandle?
    private var directory: URL?
    private var hitsURL: URL?

    init(location: String) {
        self.location = location
    }

    func start() {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent("labcommand-redirect-\(UUID().uuidString)", isDirectory: true)
        do {
            try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
            try Self.script.write(to: directory.appendingPathComponent("redirector.py"), atomically: true, encoding: .utf8)
        } catch {
            expect(false, "secure redirector directory: \(error)")
            return
        }
        self.directory = directory
        let cert = directory.appendingPathComponent("cert.pem")
        let key = directory.appendingPathComponent("key.pem")
        let hitsFile = directory.appendingPathComponent("hits")
        hitsURL = hitsFile

        let openssl = Process()
        openssl.executableURL = URL(fileURLWithPath: "/usr/bin/openssl")
        openssl.arguments = [
            "req", "-x509", "-newkey", "rsa:2048",
            "-keyout", key.path, "-out", cert.path,
            "-days", "1", "-nodes", "-subj", "/CN=127.0.0.1",
        ]
        let opensslErr = Pipe()
        openssl.standardError = opensslErr
        openssl.standardOutput = Pipe()
        openssl.standardInput = FileHandle.nullDevice
        do {
            try openssl.run()
        } catch {
            expect(false, "openssl: \(error)")
            return
        }
        openssl.waitUntilExit()
        let opensslErrorText = String(data: opensslErr.fileHandleForReading.readDataToEndOfFile(), encoding: .utf8) ?? ""
        guard openssl.terminationStatus == 0 else {
            expect(false, "openssl certificate status \(openssl.terminationStatus): \(opensslErrorText)")
            return
        }

        let proc = Process()
        proc.executableURL = URL(fileURLWithPath: "/usr/bin/python3")
        proc.arguments = [
            "-u", directory.appendingPathComponent("redirector.py").path,
            location, cert.path, key.path, hitsFile.path,
        ]
        proc.currentDirectoryURL = directory
        let input = Pipe()
        let output = Pipe()
        let errors = Pipe()
        proc.standardInput = input
        proc.standardOutput = output
        proc.standardError = errors
        let outBuffer = LRPipeBuffer()
        let errBuffer = LRPipeBuffer()
        output.fileHandleForReading.readabilityHandler = { handle in
            outBuffer.append(handle.availableData)
        }
        errors.fileHandleForReading.readabilityHandler = { handle in
            errBuffer.append(handle.availableData)
        }
        self.input = input
        self.process = proc
        self.outputHandle = output.fileHandleForReading
        self.errorHandle = errors.fileHandleForReading
        do {
            try proc.run()
        } catch {
            expect(false, "secure redirector: \(error)")
            return
        }
        let deadline = Date().addingTimeInterval(8)
        while port == 0, Date() < deadline {
            if let line = outBuffer.snapshot().split(whereSeparator: \.isNewline).first,
               let value = UInt16(line.trimmingCharacters(in: .whitespaces)), value != 0 {
                port = value
                break
            }
            if !proc.isRunning { break }
            RunLoop.main.run(until: Date().addingTimeInterval(0.02))
        }
        expect(port != 0, "secure redirector port, stdout \(outBuffer.snapshot()) stderr \(errBuffer.snapshot())")
    }

    func refreshHits() {
        guard let hitsURL else { return }
        let text = (try? String(contentsOf: hitsURL, encoding: .utf8)) ?? ""
        hits = text.split(whereSeparator: \.isNewline).filter { !$0.isEmpty }.count
    }

    func stop() {
        outputHandle?.readabilityHandler = nil
        errorHandle?.readabilityHandler = nil
        process?.terminate()
        try? input?.fileHandleForWriting.close()
        process = nil
        if let directory {
            try? FileManager.default.removeItem(at: directory)
            self.directory = nil
        }
    }

    private static let script = """
    import http.server
    import os
    import ssl
    import sys
    import threading

    def watch():
        try:
            sys.stdin.read()
        finally:
            os._exit(0)

    threading.Thread(target=watch, daemon=True).start()

    location, cert, key, hits = sys.argv[1:5]

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            with open(hits, "a", encoding="utf-8") as handle:
                handle.write("1\\n")
            self.send_response(302)
            self.send_header("Location", location)
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()

        def log_message(self, fmt, *args):
            return

    try:
        server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert, key)
        server.socket = context.wrap_socket(server.socket, server_side=True)
    except Exception as exc:
        sys.stderr.write(f"{exc}\\n")
        sys.stderr.flush()
        os._exit(1)

    sys.stdout.write(f"{server.server_address[1]}\\n")
    sys.stdout.flush()
    server.serve_forever()
    """
}

@MainActor
private func waitingReason(_ client: LiveRoomClient) -> String? {
    if case .waiting(_, let reason) = client.connection { return reason }
    return nil
}

/// The delegate is on the real handshake. Cleartext off: the target is never dialed.
/// Cleartext on, and the target an allowed host: the hop is followed and the bridge answers.
@MainActor
func testLiveRoomRedirectDoesNotBypassAdmission() {
    testRedirectAdmissionPolicy()

    let blocked = legacyBridge(seed: 11)
    blocked.start()
    let refusedHop = LRRedirector(location: blocked.url.absoluteString)
    refusedHop.start()
    let blockedClient = LiveRoomClient(url: URL(string: "ws://127.0.0.1:\(refusedHop.port)")!)
    blockedClient.start()
    endpointSpin(8) { waitingReason(blockedClient) != nil || refusedHop.hits > 0 && blocked.clientCount > 0 }
    let refusedReason = waitingReason(blockedClient) ?? ""
    expect(refusedHop.hits > 0, "the cleartext-off client reached the redirecting server, hits \(refusedHop.hits)")
    expect(blocked.clientCount == 0, "a refused redirect does not dial \(blocked.url), connections \(blocked.clientCount)")
    expect(refusedReason.contains("Refused a redirect"), "the Room failure names the redirect, got \(refusedReason)")
    expect(refusedReason.contains("Cleartext WebSocket is off"), "the refusal is the cleartext switch, got \(refusedReason)")
    blockedClient.stop()
    refusedHop.stop()
    blocked.stop()

    let allowed = legacyBridge(seed: 12)
    allowed.start()
    let followedHop = LRRedirector(location: allowed.url.absoluteString)
    followedHop.start()
    let allowedClient = LiveRoomClient(url: URL(string: "ws://127.0.0.1:\(followedHop.port)")!)
    allowedClient.setCleartextPermitted(true)
    allowedClient.start()
    waitForBrick(12, on: allowed, client: allowedClient)
    expect(followedHop.hits > 0, "the admitted client reached the redirecting server")
    expect(allowed.clientCount > 0, "an admitted redirect is followed to the bridge")
    expect(allowedClient.url.port == Int(followedHop.port),
           "the configured URL stays the redirecting server, got \(allowedClient.url)")
    allowedClient.stop()
    followedHop.stop()
    allowed.stop()

    let chained = legacyBridge(seed: 13)
    chained.start()
    let secondHop = LRRedirector(location: chained.url.absoluteString)
    secondHop.start()
    let firstHop = LRRedirector(location: "ws://127.0.0.1:\(secondHop.port)")
    firstHop.start()
    let chainedClient = LiveRoomClient(url: URL(string: "ws://127.0.0.1:\(firstHop.port)")!)
    chainedClient.setCleartextPermitted(true)
    chainedClient.start()
    waitForBrick(13, on: chained, client: chainedClient)
    expect(firstHop.hits > 0 && secondHop.hits > 0, "both hops of a cleartext chain were requested, hits \(firstHop.hits) \(secondHop.hits)")
    expect(chained.clientCount > 0, "an admitted cleartext chain reaches the bridge")
    chainedClient.stop()
    firstHop.stop()
    secondHop.stop()
    chained.stop()

    let downgraded = legacyBridge(seed: 14)
    downgraded.start()
    let secureHop = LRSecureRedirector(location: downgraded.url.absoluteString)
    secureHop.start()
    let clearHop = LRRedirector(location: "https://127.0.0.1:\(secureHop.port)/")
    clearHop.start()
    let downgradeClient = LiveRoomClient(url: URL(string: "ws://127.0.0.1:\(clearHop.port)")!)
    downgradeClient.setCleartextPermitted(true)
    downgradeClient.trustLoopbackCertificatesForTesting(true)
    downgradeClient.start()
    endpointSpin(8) {
        (waitingReason(downgradeClient)?.contains("Refused a redirect") ?? false) || downgraded.clientCount > 0
    }
    secureHop.refreshHits()
    let downgradeReason = waitingReason(downgradeClient) ?? ""
    expect(clearHop.hits > 0, "the chain reached the first redirect, hits \(clearHop.hits)")
    expect(secureHop.hits > 0, "the chain reached the secure hop, hits \(secureHop.hits)")
    expect(downgraded.clientCount == 0, "a secure hop then ws:// does not dial \(downgraded.url), connections \(downgraded.clientCount)")
    expect(downgradeReason.contains("Refused a redirect"), "the Room failure names the redirect, got \(downgradeReason)")
    expect(downgradeReason.contains("cleartext"), "the refusal is the downgrade, got \(downgradeReason)")
    expect(!downgradeReason.contains("Cleartext WebSocket is off"), "the switch is on; the refusal is the secure hop, got \(downgradeReason)")
    downgradeClient.stop()
    clearHop.stop()
    secureHop.stop()
    downgraded.stop()
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
    expect(client.contains("delegate: redirectGuard"), "the WebSocket session has a redirect delegate")
    expect(client.contains("willPerformHTTPRedirection"), "the delegate is asked before a redirect is followed")
    expect(client.contains("RoomRedirectChain"), "the delegate remembers a secure hop across the handshake")
    expect(client.contains("response.url"), "the delegate judges the redirecting response")
    expect(!client.contains("decideRedirect(from:"), "a hop is not judged by passing originalRequest to decideRedirect")

    let endpoint = text("Sources/LabCommandCore/RoomMemoryEndpoint.swift")
    expect(!endpoint.contains("100.118.187.99"), "the endpoint type has no tailnet IP")
    expect(endpoint.contains("CaptureRecipe.tailscaleDNS"), "the default host is the MagicDNS name")
    expect(endpoint.contains("func decideRedirect"), "a redirect destination is admitted")
    expect(endpoint.contains("secureHop"), "a secure hop stays on the chain")

    let settings = text("Sources/LabCommand/SettingsView.swift")
    expect(settings.contains("Rig diagnostics"), "recorder rows sit under Rig diagnostics")
    expect(settings.contains("Room memory"), "the endpoint group is Room memory")
    expect(settings.contains("Room memory endpoint"), "the field has its accessibility label")
    expect(settings.contains("commitRoomEndpoint"), "the field commits through AppState")
    expect(settings.contains("roomEndpointURL"), "the draft is the current endpoint")
    expect(!settings.contains("RoomMemoryEndpoint.resolve"), "appearing does not re-resolve the environment")
    expect(!settings.contains("100.118.187.99"), "Settings does not hardcode the room IP")
    expect(settings.contains("onSubmit"), "Return commits the address")
    expect(!settings.contains("endpointFieldFocused"), "leaving the field does not commit")
    expect(settings.contains("cleartextWarning"), "the cleartext switch shows a warning while it is on")
    expect(settings.contains("downgradePrompt"), "a wss to ws change asks before it is saved")
    expect(settings.contains("Use cleartext"), "the operator confirms the downgrade")
    expect(settings.contains("Allow cleartext WebSocket"), "cleartext is a labeled switch")

    let state = text("Sources/LabCommand/AppState.swift")
    expect(state.contains("LiveRoomClient(url:"), "AppState passes an explicit URL")
    expect(state.contains("RoomMemoryEndpointStore"), "one store owns the current endpoint")
    expect(state.contains("commitRoomEndpoint"), "one commit method")
    expect(state.contains("retargeting: room"), "commit retargets the live client")
    expect(state.contains("RoomMemoryEndpoint.admit"), "AppState admits the socket before dialing")
    expect(state.contains("setCleartextPermitted"), "AppState tells the socket the saved cleartext switch")
    expect(!state.contains("trustLoopbackCertificatesForTesting"), "AppState does not trust fixture certificates")
    expect(!settings.contains("trustLoopbackCertificatesForTesting"), "Settings does not trust fixture certificates")
    expect(state.contains("roomEndpointBlock"), "a blocked address is stored for the Room screen")
    expect(!state.contains("100.118.187.99"), "AppState does not hardcode the room IP")

    let room = text("Sources/LabCommand/LiveRoomView.swift")
    expect(room.contains("connectionDetail"), "the Room screen shows why the socket is not up")
    expect(room.contains("roomEndpointBlock"), "the Room screen receives the cleartext block")

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
    expect(ats["NSAllowsArbitraryLoads"] == nil, "global arbitrary loads is not set")
    expect(ats["NSAllowsLocalNetworking"] as? Bool == true, "local networking allows IP literals")
    expect(ats["NSAllowsArbitraryLoadsForMedia"] == nil, "the media exception stays off")
    expect(ats["NSAllowsArbitraryLoadsInWebContent"] == nil, "the web-content exception stays off")
    guard let domains = ats["NSExceptionDomains"] as? [String: Any] else {
        expect(false, "NSExceptionDomains is present")
        return
    }
    expect(domains["100.118.187.99"] == nil, "the tailnet IP exception is gone")
    expect(domains["192.168.68.65"] == nil, "the LAN IP exception is gone")
    expect(Set(domains.keys) == Set(["ts.net"]), "the only exception domain is ts.net, got \(domains.keys)")
    guard let tailnet = domains["ts.net"] as? [String: Any] else {
        expect(false, "ts.net exception is a dictionary")
        return
    }
    expect(tailnet["NSIncludesSubdomains"] as? Bool == true, "the ts.net exception covers MagicDNS names")
    expect(tailnet["NSExceptionAllowsInsecureHTTPLoads"] as? Bool == true, "the ts.net exception allows cleartext")
}

func testMagicDNSFailureIsVisibleOnTheRoomScreen() {
    let url = RoomMemoryEndpoint.defaultURL
    let reason = "A server with the specified hostname could not be found."
    let screen = LiveRoomScreen.make(
        mirror: RoomMemoryMirror(),
        connection: .waiting(retryIn: 5, reason: reason),
        requestPathExists: false,
        started: true,
        now: 10,
        endpointURL: url.absoluteString
    )
    let detail = screen.connectionDetail ?? ""
    expect(detail.contains(CaptureRecipe.tailscaleDNS), "the Room screen names the MagicDNS host, got \(detail)")
    expect(detail.contains("8767"), "the Room screen names the port, got \(detail)")
    expect(detail.contains(reason), "the Room screen includes the dial error, got \(detail)")
    expect(!detail.contains(CaptureRecipe.tailscaleIP), "a failed lookup does not substitute the tailnet IP")
    expect(screen.banner?.text == "connection lost", "the connection-lost banner stays, got \(String(describing: screen.banner?.text))")

    let first = LiveRoomScreen.make(
        mirror: RoomMemoryMirror(),
        connection: .connecting(url: url, attempt: 1),
        requestPathExists: false,
        started: true,
        now: 10,
        endpointURL: url.absoluteString
    )
    expect(first.connectionDetail == nil, "the first attempt is the connecting banner, not a failure line")
    expect(first.banner?.text == "connecting", "the first attempt says connecting")

    let retry = RoomConnectionFailure.detail(
        connection: .connecting(url: url, attempt: 2),
        urlText: url.absoluteString
    ) ?? ""
    expect(retry.contains(CaptureRecipe.tailscaleDNS), "a later attempt names the host, got \(retry)")

    let root = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    let endpoint = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommandCore/RoomMemoryEndpoint.swift"), encoding: .utf8)) ?? ""
    expect(!endpoint.contains("tailscaleIP"), "the endpoint has no tailnet IP fallback")
    let settings = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/SettingsView.swift"), encoding: .utf8)) ?? ""
    expect(settings.contains("RoomConnectionFailure"), "Settings shows the same dial failure")
    let room = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/LiveRoomView.swift"), encoding: .utf8)) ?? ""
    expect(room.contains("endpointURL"), "the Room screen passes the address into the failure line")
}

func runRoomMemoryEndpointTests() {
    MainActor.assumeIsolated {
        testRoomMemoryEndpointParsesAndResolves()
        print("  room endpoint: parse, canonical form, resolve order, save round-trip"); passed()
        testRoomMemoryCommitKeepsTheCurrentEndpoint()
        print("  room endpoint: invalid commit keeps the preference and the socket; reopen reads current"); passed()
        testCleartextDowngradeRequiresConfirmation()
        print("  room endpoint: cleartext switch, host allow-list, and wss to ws confirmation"); passed()
        testEnvironmentURLDoesNotGrantCleartext()
        print("  room endpoint: an environment ws:// does not grant cleartext across relaunch"); passed()
        testLiveRoomRetarget()
        print("  room endpoint: retarget swaps bridges, publishes connecting, same URL keeps bricks"); passed()
        testLiveRoomRetargetDuringHandshakeBackoffAndRapidChanges()
        print("  room endpoint: retarget during handshake and backoff; A→B→C ignores the old bridges"); passed()
        testLiveRoomRedirectDoesNotBypassAdmission()
        print("  room endpoint: a redirect is admitted, and a secure hop blocks a later cleartext hop"); passed()
    }
    testCleartextHostPolicy()
    print("  room endpoint: ws:// only for private, CGNAT, and .ts.net hosts"); passed()
    testMagicDNSFailureIsVisibleOnTheRoomScreen()
    print("  room endpoint: a failed MagicDNS dial is named on the Room screen, with no IP fallback"); passed()
    testRoomMemorySettingsSourcesAndATS()
    print("  room endpoint: Settings, AppState, live-shot pin, and ATS keys"); passed()
}
