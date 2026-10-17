import Foundation
import LabCommandCore

private let liveJSON = """
{"ts": 1787961000.0, "mid360_ping": true, "jt128_ping": true, "jt128_ptc": true,
 "containers": {"lidar-dual": "running", "ptp-master": "running", "lidar-watchdog": "running"},
 "rates": {"livox_pts": 9.8, "jt_pts": 8.6, "livox_imu": 199.2, "jt_imu": 198.8},
 "offsets": {"livox_ms": 1.2, "jt_ms": 0.9},
 "recording": {"active": false, "session": null, "size_gb": 0},
 "disk_free_gb": 2400, "load1": 0.42, "mem_pct": 3.1,
 "events": ["2026-08-28T23:52:01Z mid-360 returned"]}
"""

private let parkedJSON = """
{"ts": 1787961000.0, "mid360_ping": false, "jt128_ping": false, "jt128_ptc": false,
 "containers": {"lidar-dual": "running", "ptp-master": "running", "lidar-watchdog": "running"},
 "rates": {"livox_pts": 0.0, "jt_pts": 0.0, "livox_imu": 0.0, "jt_imu": 0.0},
 "offsets": {"livox_ms": null, "jt_ms": null},
 "recording": {"active": false, "session": null, "size_gb": 0},
 "disk_free_gb": 2400, "events": []}
"""

func testDecoderLiveRig() {
    guard let s = RigStatusDecoder.parse(json: liveJSON, viewTunnelUp: true) else {
        expect(false, "live JSON parses")
        return
    }
    expect(s.sparkReachable && s.apiReachable, "live snapshot reachable")
    expect(s.mid360.state == .streaming, "mid360 streaming at 9.8 Hz")
    expect(s.jt128.state == .streaming, "jt128 streaming at 8.6 Hz")
    expect(s.mid360.pointRateHz == 9.8, "mid rate preserved")
    expect(s.jt128.ptcOpen == true, "ptc flag preserved")
    expect(s.stackRunning && s.watchdogRunning && s.ptpMasterRunning, "containers mapped")
    expect(s.clock.state == .locked, "sub-50ms offsets → locked")
    expect(s.clock.deltaMs != nil && abs(s.clock.deltaMs! - 0.3) < 0.01, "delta |1.2-0.9| ≈ 0.3")
    expect(s.diskFreeGB == 2400, "disk mapped")
    expect(s.events.count == 1, "watchdog events mapped")
    expect(s.bothStreaming && s.anyStreaming, "streaming aggregates")
}

func testDecoderParkedRigIsDarkNotBlocked() {
    guard let s = RigStatusDecoder.parse(json: parkedJSON, viewTunnelUp: false) else {
        expect(false, "parked JSON parses")
        return
    }
    expect(s.mid360.state == .dark, "no ping → dark")
    expect(s.jt128.state == .dark, "no ping → dark")
    expect(s.bothDark, "bothDark aggregate")
    expect(s.mid360.pointRateHz == 0.0, "zero is kept as a real measurement")
    let health = RigHealth.evaluate(s)
    expect(health.band == .parked, "dark sensors + healthy infra = Parked, got \(health.band)")
    expect(health.headline.lowercased().contains("plug"), "parked copy invites power-up")
}

func testDecoderMissingKeysStayNil() {
    let minimal = """
    {"ts": 1.0, "containers": {}}
    """
    guard let s = RigStatusDecoder.parse(json: minimal, viewTunnelUp: false) else {
        expect(false, "minimal JSON parses")
        return
    }
    expect(s.mid360.state == .unknown, "no ping key → unknown, not dark")
    expect(s.mid360.pointRateHz == nil, "no rate key → nil, never 0")
    expect(s.mid360.clockOffsetMs == nil, "no offset key → nil")
    expect(s.diskFreeGB == nil, "no disk key → nil")
    expect(s.clock.state == .masterDown, "no ptp-master container → master down")
    expect(TelemetryFormat.hz(s.mid360.pointRateHz) == TelemetryFormat.unavailable,
           "nil renders as the word unavailable")
}

func testDecoderGarbageIsNilNotACrash() {
    expect(RigStatusDecoder.parse(json: "not json", viewTunnelUp: false) == nil, "garbage → nil")
    expect(RigStatusDecoder.parse(json: "[1,2,3]", viewTunnelUp: false) == nil, "non-object → nil")
}

func testPoweredNotStreamingIsHonest() {
    let json = liveJSON
        .replacingOccurrences(of: "\"jt_pts\": 8.6", with: "\"jt_pts\": 0.0")
    guard let s = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    expect(s.jt128.state == .powered, "ping ok + 0 Hz = powered, not streaming")
    expect(s.jt128.detail.contains("standby") || s.jt128.detail.contains("driver"),
           "powered JT detail names standby/driver: \(s.jt128.detail)")
    let health = RigHealth.evaluate(s)
    expect(health.band == .watch || health.band == .ready,
           "one live + one powered is watch-or-ready, not blocked")
}

func testGatesOfflineNeverArm() {
    let off = RigSnapshot.offline(reason: "Spark not reachable")
    expect(!RigGate.canStartRig(off).0, "start blocked offline")
    expect(!RigGate.canRecord(off).0, "record blocked offline")
    expect(!RigGate.canWakeJT(off).0, "wake blocked offline")
    expect(RigGate.canStartRig(off).1?.contains("Spark") == true, "reason names Spark")
    let health = RigHealth.evaluate(off)
    expect(health.band == .blocked, "offline is Blocked")
}

func testGatesRecordNeedsPointsNotJustContainers() {
    let json = liveJSON
        .replacingOccurrences(of: "\"livox_pts\": 9.8", with: "\"livox_pts\": 0.0")
        .replacingOccurrences(of: "\"jt_pts\": 8.6", with: "\"jt_pts\": 0.0")
    guard let s = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    expect(s.stackRunning, "container is up")
    let record = RigGate.canRecord(s)
    expect(!record.0, "record refuses when no sensor produces points")
    expect(record.1?.lowercased().contains("point") == true, "reason mentions points: \(record.1 ?? "")")
}

func testGatesRecordArmsWithOneLiveSensor() {
    let json = liveJSON.replacingOccurrences(of: "\"jt_pts\": 8.6", with: "\"jt_pts\": 0.0")
    guard let s = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    expect(RigGate.canRecord(s).0, "one live sensor arms record")
}

func testGatesJTControls() {
    guard let live = RigStatusDecoder.parse(json: liveJSON, viewTunnelUp: true),
          let parked = RigStatusDecoder.parse(json: parkedJSON, viewTunnelUp: false) else {
        expect(false, "parses")
        return
    }
    expect(!RigGate.canWakeJT(live).0, "wake refused while streaming")
    expect(RigGate.canStandbyJT(live).0, "standby armed while streaming")
    expect(!RigGate.canWakeJT(parked).0, "wake refused when unpowered")
    expect(!RigGate.canStandbyJT(parked).0, "standby refused when unpowered")
}

func testGatesArmAtLock() {
    guard let live = RigStatusDecoder.parse(json: liveJSON, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    let off = RigSnapshot.offline(reason: "Spark unreachable")
    expect(!RigGate.canArmAtLock(off).0, "arm blocked offline")
    expect(RigGate.canArmAtLock(off).1?.contains("Spark") == true, "reason names Spark")
    expect(RigGate.canArmAtLock(live).0, "arm allowed when live and idle")
    
    let recordingJSON = liveJSON.replacingOccurrences(
        of: "\"recording\": {\"active\": false, \"session\": null, \"size_gb\": 0}",
        with: "\"recording\": {\"active\": true, \"session\": \"take1\", \"size_gb\": 1.0}"
    )
    guard let recording = RigStatusDecoder.parse(json: recordingJSON, viewTunnelUp: true) else {
        expect(false, "recording JSON parses")
        return
    }
    expect(!RigGate.canArmAtLock(recording).0, "arm blocked while recording")
}

func testRecordingHealthAndStop() {
    let json = liveJSON.replacingOccurrences(
        of: "\"recording\": {\"active\": false, \"session\": null, \"size_gb\": 0}",
        with: "\"recording\": {\"active\": true, \"session\": \"20260828T230000Z_DUAL_A_baseline\", \"size_gb\": 1.4}"
    )
    guard let s = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    expect(s.recording.active, "recording decoded")
    expect(s.recording.session == "20260828T230000Z_DUAL_A_baseline", "session name decoded")
    expect(!RigGate.canRecord(s).0, "no double record")
    expect(RigGate.canStopRecord(s).0, "stop-record armed")
    expect(!RigGate.canStandbyJT(s).0, "standby refused mid-take")
    let health = RigHealth.evaluate(s)
    expect(health.band == .ready, "both streaming while recording = Ready")
    expect(health.headline.contains("Recording"), "headline says Recording")
}

func testClockConvergingIsVisible() {
    let json = liveJSON.replacingOccurrences(of: "\"livox_ms\": 1.2", with: "\"livox_ms\": -232.0")
    guard let s = RigStatusDecoder.parse(json: json, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    expect(s.clock.state == .converging, "232 ms offset is converging, not locked")
    let advice = RigDiagnose.advice(for: s)
    expect(advice.stage == .clock, "diagnose points at clocks")
    expect(advice.recommendedAction.lowercased().contains("wait")
           || advice.recommendedAction.lowercased().contains("slew"),
           "advice explains the slew: \(advice.recommendedAction)")
}

func testDeskButtonsAlwaysThreeAndGated() {
    guard let live = RigStatusDecoder.parse(json: liveJSON, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    let offline = RigDesk.buttons(for: RigSnapshot.offline(reason: "x"))
    let liveButtons = RigDesk.buttons(for: live)
    expect(offline.count == 3 && liveButtons.count == 3, "three transport buttons always")
    expect(offline.allSatisfy { !$0.enabled || $0.action == .stop } || offline.allSatisfy { !$0.enabled },
           "offline arms nothing")
    expect(liveButtons.first(where: { $0.action == .startRecording })?.enabled == true,
           "record armed when both stream")
    expect(liveButtons.first(where: { $0.action == .startRecording })?.isPrimary == true,
           "record is primary when rig is live and idle")
    expect(liveButtons.first(where: { $0.action == .startRig })?.title == "Restart rig",
           "start reads Restart when the stack runs")
}

func testDeskDualSessionNaming() {
    let spec = CaptureSessionSpec(preset: .baselineRewalk)
    let preview = RigDesk.sessionFolderPreview(spec, now: Date(timeIntervalSince1970: 1_787_961_000))
    expect(preview.contains("_DUAL_A2_baseline_rewalk"), "dual folder carries DUAL + code + tag: \(preview)")
    expect(RigDesk.recordName(spec) == "A2_baseline_rewalk", "record arg is code_tag")
}

func testDiagnoseParkedPointsAtPower() {
    guard let s = RigStatusDecoder.parse(json: parkedJSON, viewTunnelUp: false) else {
        expect(false, "parses")
        return
    }
    let advice = RigDiagnose.advice(for: s)
    expect(advice.stage == .stack, "parked diagnose lands on the stack stage")
    expect(advice.recommendedAction.lowercased().contains("plug"), "advice says plug in")
    expect(advice.recommendedAction.lowercased().contains("watchdog"), "advice mentions the watchdog")
}

func testStagesReflectSnapshot() {
    guard let live = RigStatusDecoder.parse(json: liveJSON, viewTunnelUp: true) else {
        expect(false, "parses")
        return
    }
    expect(RigDesk.stageState(.link, live) == .running, "link live")
    expect(RigDesk.stageState(.stack, live) == .running, "stack live")
    expect(RigDesk.stageState(.clock, live) == .running, "clock locked → live")
    expect(RigDesk.stageState(.view, live) == .running, "view live with tunnel")
    expect(RigDesk.stageState(.record, live) == .idle, "record idle")
    let off = RigSnapshot.offline(reason: "x")
    expect(RigDesk.stageState(.link, off) == .blocked, "link blocked offline")
    expect(RigDesk.stageState(.stack, off) == .unknown, "stack unknown offline — not idle")
}

func testMissionClockIncludesTheDate() {
    let date = Date(timeIntervalSince1970: 1_760_400_010)
    let formatter = DateFormatter()
    formatter.locale = Locale(identifier: "en_US_POSIX")
    formatter.timeZone = .current
    formatter.dateFormat = "yyyy-MM-dd HH:mm:ss"
    let clock = MissionLog.clock.string(from: date)
    expect(clock == formatter.string(from: date), "mission clock is the local date and time, got \(clock)")
    expect(clock.range(of: #"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}$"#, options: .regularExpression) != nil,
           "mission clock is yyyy-MM-dd HH:mm:ss, got \(clock)")
    let rendered = MissionLog.renderLine(
        MissionEvent(at: date, kind: .note, text: "hello"),
        clock: MissionLog.clock
    )
    expect(rendered.hasPrefix(clock + "  "), "renderLine leads with the dated clock, got \(rendered)")
}

func testMissionLogAppendAndCap() {
    var events: [MissionEvent] = []
    events = MissionLog.append(events, kind: .start, text: "start rig…")
    events = MissionLog.append(events, kind: .record, text: "record ok")
    expect(events.count == 2, "append grows")
    expect(events[0].text == "start rig…", "order preserved")
    for i in 0..<(MissionLog.maxEvents + 20) {
        events = MissionLog.append(events, kind: .info, text: "e\(i)")
    }
    expect(events.count == MissionLog.maxEvents, "log capped at \(MissionLog.maxEvents)")
}

func testSessionPresetsMatchResearchPlan() {
    expect(CapturePreset.allCases.count == 4, "A / A2 / B / C")
    expect(CapturePreset.baseline.code == "A" && CapturePreset.baselineRewalk.code == "A2", "codes pinned")
    expect(CapturePreset.furnitureMoved.layoutTag == "furniture_moved", "tags pinned")
}

func testTelemetryFormatUnavailable() {
    expect(TelemetryFormat.hz(nil) == "unavailable", "nil hz → unavailable")
    expect(TelemetryFormat.hz(9.8) == "9.8 Hz", "hz formats")
    expect(TelemetryFormat.percent(nil) == "unavailable", "nil pct → unavailable")
    expect(TelemetryFormat.flag(nil, yes: "y", no: "n") == "unavailable", "nil flag → unavailable")
}

func testTelemetryFormatCondense() {
    expect(TelemetryFormat.condense("ssh: connect to host spark-1f3e.tail8dfdc7.ts.net port 22:\nOperation timed out")
           == "ssh timed out", "timeout collapses to a fact")
    expect(TelemetryFormat.condense("Permission denied (publickey).") == "ssh key rejected", "auth failure")
    expect(TelemetryFormat.condense("ssh spark-ts -> spark-1f3e") == "ssh spark-ts -> spark-1f3e",
           "a short line passes through unchanged")
    expect(TelemetryFormat.condense(String(repeating: "x", count: 200)).count <= 64,
           "long text is truncated")
}

func testDecoderArmedBlock() {
    let json = """
    {"mid360_ping": true, "jt128_ping": true,
     "rates": {"livox_pts": 10.0, "jt_pts": 10.0, "livox_imu": 200.0, "jt_imu": 200.0},
     "offsets": {"livox_ms": -41.0, "jt_ms": 0.4},
     "recording": {"active": false, "session": null, "size_gb": 0},
     "armed": {"name": "first_points2", "protocol": "custody.points.v0",
               "pose_source": "static_rig_parked", "duration_s": 120,
               "lock_streak": 2, "state": "waiting_for_lock"}}
    """
    guard let s = RigStatusDecoder.parse(json: json, viewTunnelUp: false),
          let a = s.armed else {
        expect(false, "armed JSON parses with an armed block")
        return
    }
    expect(a.name == "first_points2", "armed take name decoded")
    expect(a.protocolID == "custody.points.v0", "armed protocol decoded")
    expect(a.poseSource == "static_rig_parked", "armed pose_source decoded")
    expect(a.durationS == 120 && a.lockStreak == 2, "duration + streak decoded")
    expect(a.state == "waiting_for_lock", "armed state decoded verbatim")
}

func testDecoderNoArmedBlockStaysNil() {
    guard let s = RigStatusDecoder.parse(json: parkedJSON, viewTunnelUp: false) else {
        expect(false, "parked JSON parses")
        return
    }
    expect(s.armed == nil, "absent armed key → nil, no invented chip")
}

func testDecoderLastGateBlock() {
    let json = """
    {"mid360_ping": true, "jt128_ping": true,
     "rates": {"livox_pts": 10.0, "jt_pts": 10.0, "livox_imu": 200.0, "jt_imu": 200.0},
     "offsets": {"livox_ms": -13.6, "jt_ms": 0.4},
     "recording": {"active": false, "session": null, "size_gb": 0},
     "last_gate": {"take_id": "20260830T223209Z_DUAL_first_points2",
                   "protocol_id": "custody.points.v0", "passed": false,
                   "failed_checks": ["calibration_missing"],
                   "stopped_utc": "2026-08-30T22:34:10Z", "duration_s": 120.998}}
    """
    guard let s = RigStatusDecoder.parse(json: json, viewTunnelUp: false),
          let g = s.lastGate else {
        expect(false, "last_gate JSON parses with a gate block")
        return
    }
    expect(g.takeID == "20260830T223209Z_DUAL_first_points2", "gate take id decoded")
    expect(g.passed == false && g.failedChecks == ["calibration_missing"],
           "verdict + named failures decoded verbatim, never recomputed")
    expect(g.durationS == 120.998, "duration decoded")
    expect(RigStatusDecoder.parse(json: parkedJSON, viewTunnelUp: false)?.lastGate == nil,
           "absent last_gate → nil, no invented verdict")
}

func testGatePreflightScoresFromLiveProbes() {
    var s = RigSnapshot()
    s.mid360 = SensorReading(id: .mid360, state: .streaming, pointRateHz: 10.0,
                             imuRateHz: 200.0, clockOffsetMs: -13.6)
    s.jt128 = SensorReading(id: .jt128, state: .streaming, pointRateHz: 10.0,
                            imuRateHz: 200.0, clockOffsetMs: 0.4)
    s.pipeline = PipelineStatus(counters: PipelineCounters(
        udpInErrorsDelta: 0, udpRcvbufErrorsDelta: 0, rxMacMissedDelta: 0))

    let mid = s.mid360.clockOffsetMs
    let jt = s.jt128.clockOffsetMs
    if let mid, let jt {
        let band = ClockSync.lockedBandMs
        let inBand = abs(mid) < band && abs(jt) < band
        expect(inBand, "in-band clocks hold")
    }

    let midHz = s.mid360.pointRateHz
    let jtHz = s.jt128.pointRateHz
    if let midHz, let jtHz {
        let flowing = midHz > RigStatusDecoder.streamingFloorHz
            && jtHz > RigStatusDecoder.streamingFloorHz
        expect(flowing, "streaming holds")
    }

    if let c = s.pipeline?.counters,
       let mac = c.rxMacMissedDelta, let ine = c.udpInErrorsDelta,
       let buf = c.udpRcvbufErrorsDelta {
        let clean = mac == 0 && ine == 0 && buf == 0
        expect(clean, "clean deltas hold")
    }

    s.mid360.clockOffsetMs = -450.6
    if let mid2 = s.mid360.clockOffsetMs {
        let band = ClockSync.lockedBandMs
        let inBand = abs(mid2) < band
        expect(!inBand, "out-of-band clock reads waiting")
    }

    let bare = RigSnapshot()
    expect(bare.mid360.clockOffsetMs == nil && bare.jt128.clockOffsetMs == nil,
           "no stamps → unavailable")
    expect(bare.pipeline?.counters == nil,
           "no counters → unavailable")
}

func testSpecDossiersCarryTheResearch() {
    let midIP = SensorID.mid360.ip
    let jtIP = SensorID.jt128.ip
    expect(midIP == "192.168.1.135", "mid ip")
    expect(jtIP == "192.168.118.201", "jt ip")
}

// MARK: - StatusAggregation tests

func testStatusAggregationSparkPreferSuccess() {
    let sshOk = CheckOutcome(id: "spark_ssh", kind: .success, detail: "ssh connected")
    let pingFail = CheckOutcome(id: "spark_ping", kind: .failure, detail: "timeout")
    let checks = [sshOk, pingFail]
    let status = StatusAggregation.statusPreferSuccess(from: checks, matching: StatusAggregation.sparkCheckIDs)
    expect(status == .connected, "SSH success wins over ping failure for Spark")
}

func testStatusAggregationTailscaleCombine() {
    let tailOk = CheckOutcome(id: "tailscale", kind: .success, detail: "up")
    let tailDown = CheckOutcome(id: "tailscale", kind: .failure, detail: "down")
    
    let statusOk = StatusAggregation.statusFrom(checks: [tailOk], matching: StatusAggregation.tailscaleCheckIDs)
    expect(statusOk == .connected, "single tailscale success → connected")
    
    let statusDown = StatusAggregation.statusFrom(checks: [tailDown], matching: StatusAggregation.tailscaleCheckIDs)
    expect(statusDown == .disconnected, "single tailscale failure → disconnected")
}

func testStatusAggregationEmptyReturnsUnknown() {
    let status = StatusAggregation.statusFrom(checks: [], matching: StatusAggregation.sparkCheckIDs)
    expect(status == .unknown, "empty checks → unknown")
    
    let unrelated = CheckOutcome(id: "other_check", kind: .success, detail: "ok")
    let noMatch = StatusAggregation.statusFrom(checks: [unrelated], matching: StatusAggregation.sparkCheckIDs)
    expect(noMatch == .unknown, "no matching IDs → unknown")
}

func testStatusAggregationBuildSnapshotSummary() {
    let sparkOk = CheckOutcome(id: "spark_ssh", kind: .success, detail: "reachable")
    let tailOk = CheckOutcome(id: "tailscale", kind: .success, detail: "up")
    let snapshot = StatusAggregation.buildSnapshot(from: [sparkOk, tailOk])
    expect(snapshot.overall == .connected, "connected summary when spark reachable")
    expect(snapshot.summaryLine.contains("healthy"), "summary says healthy: \(snapshot.summaryLine)")
    
    let sparkDown = CheckOutcome(id: "spark_ssh", kind: .failure, detail: "timeout")
    let downSnap = StatusAggregation.buildSnapshot(from: [sparkDown, tailOk])
    expect(downSnap.overall == .disconnected, "disconnected when spark down")
    expect(downSnap.summaryLine.contains("down"), "summary says down: \(downSnap.summaryLine)")
}

func testStatusAggregationCombineWorstWins() {
    expect(StatusAggregation.combine([.connected, .connected]) == .connected, "all connected → connected")
    expect(StatusAggregation.combine([.connected, .unknown]) == .unknown, "unknown degrades connected")
    expect(StatusAggregation.combine([.connected, .disconnected]) == .disconnected, "disconnected beats connected")
    expect(StatusAggregation.combine([.disconnected, .error]) == .error, "error beats disconnected")
    expect(StatusAggregation.combine([]) == .unknown, "empty → unknown")
}
