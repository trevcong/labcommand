import Foundation
import LabCommandCore

// Fixture loading and decoder round-trip tests.
// Tests FAIL (not skip) when fixtures are missing — fixtures must be present for tests to pass.

private func fixturesPath() -> String? {
    let fm = FileManager.default
    let searchPaths = [
        "fixtures/api",
        "../fixtures/api",
        "../../fixtures/api",
    ]
    let cwd = fm.currentDirectoryPath
    for rel in searchPaths {
        let path = URL(fileURLWithPath: cwd).appendingPathComponent(rel).path
        if fm.fileExists(atPath: path) {
            return path
        }
    }
    return nil
}

private func loadFixture(_ name: String) -> String? {
    guard let base = fixturesPath() else { return nil }
    let path = URL(fileURLWithPath: base).appendingPathComponent("\(name).json").path
    guard let data = FileManager.default.contents(atPath: path) else { return nil }
    return String(data: data, encoding: .utf8)
}

private func requireFixture(_ name: String) -> String {
    guard let json = loadFixture(name) else {
        fputs("FAIL: fixtures/api/\(name).json not found — run tests from repo root\n", stderr)
        exit(1)
    }
    return json
}

func testFixturesDirectoryExists() {
    guard fixturesPath() != nil else {
        fputs("FAIL: fixtures/api directory not found\n", stderr)
        exit(1)
    }
}

func testFixtureBothSensorsLive() {
    let json = requireFixture("both-sensors-live")
    guard let snap = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        fputs("FAIL: both-sensors-live.json must parse\n", stderr)
        exit(1)
    }
    expect(snap.sparkReachable && snap.apiReachable, "live rig is reachable")
    expect(snap.mid360.state == .streaming, "mid360 streaming")
    expect(snap.jt128.state == .streaming, "jt128 streaming")
    expect(snap.clock.state == .locked, "clocks locked")
    expect(snap.bothStreaming, "bothStreaming aggregate")
    expect(snap.pipeline != nil, "pipeline block present")
    expect(snap.pipeline?.nic.speedMbps == 1000, "NIC speed decoded")
    expect(snap.pipeline?.ptpRole == "grandmaster", "PTP role decoded")
    expect(!snap.recording.active, "not recording")
    expect(snap.diskFreeGB == 2400, "disk free decoded")
}

func testFixtureBothSensorsDark() {
    let json = requireFixture("both-sensors-dark")
    guard let snap = RigStatusDecoder.parse(json: json, viewTunnelUp: false) else {
        fputs("FAIL: both-sensors-dark.json must parse\n", stderr)
        exit(1)
    }
    expect(snap.mid360.state == .dark, "mid360 dark (unpowered)")
    expect(snap.jt128.state == .dark, "jt128 dark (unpowered)")
    expect(snap.bothDark, "bothDark aggregate")
    let health = RigHealth.evaluate(snap)
    expect(health.band == .parked, "dark sensors + healthy infra = Parked")
}

func testFixtureClockConverging() {
    let json = requireFixture("clock-converging")
    guard let snap = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        fputs("FAIL: clock-converging.json must parse\n", stderr)
        exit(1)
    }
    expect(snap.mid360.state == .streaming, "mid360 streaming during slew")
    expect(snap.jt128.state == .streaming, "jt128 streaming during slew")
    expect(snap.clock.state == .converging, "clocks converging (Mid-360 slewing)")
    expect(abs(snap.mid360.clockOffsetMs ?? 0) > ClockSync.lockedBandMs,
           "mid offset outside lock band")
}

func testFixtureClockLocked() {
    let json = requireFixture("clock-locked")
    guard let snap = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        fputs("FAIL: clock-locked.json must parse\n", stderr)
        exit(1)
    }
    expect(snap.clock.state == .locked, "clocks locked")
    expect(abs(snap.mid360.clockOffsetMs ?? 1000) < ClockSync.lockedBandMs,
           "mid offset inside lock band")
    expect(abs(snap.jt128.clockOffsetMs ?? 1000) < ClockSync.lockedBandMs,
           "jt offset inside lock band")
}

func testFixtureRecordingActive() {
    let json = requireFixture("recording-active")
    guard let snap = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        fputs("FAIL: recording-active.json must parse\n", stderr)
        exit(1)
    }
    expect(snap.recording.active, "recording active")
    expect(snap.recording.session?.contains("DUAL") == true, "session name contains DUAL")
    expect(snap.recording.sizeGB ?? 0 > 0, "recording size tracked")
    expect(!RigGate.canRecord(snap).0, "cannot start another recording")
    expect(RigGate.canStopRecord(snap).0, "can stop recording")
}

func testFixtureSparkOffline() {
    // spark-offline.json is a documentation fixture; the real offline snapshot
    // is synthesized by the app when Spark is unreachable. Test that an offline
    // snapshot has the expected properties.
    let snap = RigSnapshot.offline(reason: "mock: Spark not reachable")
    expect(!snap.sparkReachable, "spark not reachable")
    expect(!snap.apiReachable, "api not reachable")
    expect(snap.blockReason?.contains("not reachable") == true, "block reason set")
    let health = RigHealth.evaluate(snap)
    expect(health.band == .blocked, "offline is Blocked")
}

func testDecoderRoundTripPreservesFields() {
    let json = requireFixture("both-sensors-live")
    guard let snap = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        fputs("FAIL: parse must succeed\n", stderr)
        exit(1)
    }
    expect(snap.mid360.pointRateHz != nil, "mid point rate preserved")
    expect(snap.mid360.imuRateHz != nil, "mid imu rate preserved")
    expect(snap.mid360.clockOffsetMs != nil, "mid clock offset preserved")
    expect(snap.jt128.pointRateHz != nil, "jt point rate preserved")
    expect(snap.jt128.imuRateHz != nil, "jt imu rate preserved")
    expect(snap.jt128.clockOffsetMs != nil, "jt clock offset preserved")
    expect(snap.jt128.ptcOpen != nil, "jt ptc flag preserved")
    expect(snap.diskFreeGB != nil, "disk free preserved")
    expect(snap.load1 != nil, "load1 preserved")
    expect(snap.memPct != nil, "memPct preserved")
    expect(!snap.events.isEmpty, "events preserved")
}

func testDecoderMissingFieldsStayNil() {
    let minimal = """
    {"ts": 1.0, "containers": {}}
    """
    guard let snap = RigStatusDecoder.parse(json: minimal, viewTunnelUp: false) else {
        fputs("FAIL: minimal JSON must parse\n", stderr)
        exit(1)
    }
    expect(snap.mid360.state == .unknown, "no ping key → unknown")
    expect(snap.mid360.pointRateHz == nil, "no rate → nil")
    expect(snap.mid360.clockOffsetMs == nil, "no offset → nil")
    expect(snap.diskFreeGB == nil, "no disk → nil")
    expect(snap.pipeline == nil, "no pipeline block → nil")
    expect(snap.armed == nil, "no armed block → nil")
    expect(snap.lastGate == nil, "no gate block → nil")
}

func testAllFixturesParseWithoutCrash() {
    let fixtures = [
        "both-sensors-live",
        "both-sensors-dark",
        "clock-converging",
        "clock-locked",
        "recording-active",
    ]
    for name in fixtures {
        let json = requireFixture(name)
        let snap = RigStatusDecoder.parse(json: json, viewTunnelUp: true)
        expect(snap != nil, "\(name).json parses")
    }
}

func runFixtureTests() {
    print("Running fixture tests…")
    testFixturesDirectoryExists()
    print("  fixture: fixtures/api directory exists")
    testFixtureBothSensorsLive()
    print("  fixture: both-sensors-live → streaming duo, locked clocks, pipeline")
    testFixtureBothSensorsDark()
    print("  fixture: both-sensors-dark → parked, dark sensors, healthy infra")
    testFixtureClockConverging()
    print("  fixture: clock-converging → slewing Mid-360, out-of-band offset")
    testFixtureClockLocked()
    print("  fixture: clock-locked → both offsets inside band")
    testFixtureRecordingActive()
    print("  fixture: recording-active → active session, gates enforce no-double-record")
    testFixtureSparkOffline()
    print("  fixture: spark-offline → blocked health, synthesized offline snapshot")
    testDecoderRoundTripPreservesFields()
    print("  decoder round-trip: all fields preserved from live fixture")
    testDecoderMissingFieldsStayNil()
    print("  decoder round-trip: minimal JSON → expected nils, no crashes")
    testAllFixturesParseWithoutCrash()
    print("  decoder round-trip: all fixtures parse without crash")
}
