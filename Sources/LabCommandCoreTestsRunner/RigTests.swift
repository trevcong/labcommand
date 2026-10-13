import Foundation
import LabCommandCore

// v1.0 dual-rig core. The honest-state contract from v0.4/v0.5 is re-pinned
// against the new decoder/gates/health: missing keys never become rates, a
// measured zero is a real zero, and a parked rig is calm, not broken.

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
    // A measured 0 Hz on a dark sensor is data, not an accusation.
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
    // The v0.5 lesson, re-pinned: a running container is not a running stream.
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

// -- pipeline probes + audit (v1.2) ------------------------------------------

private let pipelineJSON = """
{"ts": 1787966000.0, "mid360_ping": false, "jt128_ping": false, "jt128_ptc": false,
 "containers": {"lidar-dual": "running", "ptp-master": "running",
                "lidar-watchdog": "running", "lidar-center": "running"},
 "pipeline": {
   "nic": {"speed_mbps": 1000, "mtu": 1500, "carrier": true, "duplex": "full",
           "rmem_max": 67108864},
   "counters": {"udp_in_errors": 0, "udp_rcvbuf_errors": 0, "rx_mac_missed": 1034144,
                "udp_in_errors_delta": 0, "udp_rcvbuf_errors_delta": 0,
                "rx_mac_missed_delta": 0},
   "sockets": {"udp_bound": {"2368": true, "56401": true, "56301": true},
               "established": {"8765": 0, "8766": 1}},
   "ptp_role": "grandmaster"},
 "rates": {"livox_pts": 0.0, "jt_pts": 0.0, "livox_imu": 0.0, "jt_imu": 0.0},
 "offsets": {"livox_ms": null, "jt_ms": null},
 "recording": {"active": false, "session": null, "size_gb": 0},
 "disk_free_gb": 2400, "load1": 0.09, "mem_pct": 2.5, "events": []}
"""

func testDecoderPipelineBlock() {
    guard let s = RigStatusDecoder.parse(json: pipelineJSON, viewTunnelUp: false),
          let p = s.pipeline else {
        expect(false, "pipeline JSON parses with a pipeline block")
        return
    }
    expect(p.nic.speedMbps == 1000 && p.nic.duplex == "full", "nic link decoded")
    expect(p.nic.carrier == true && p.nic.mtu == 1500, "carrier + mtu decoded")
    expect(p.counters.rxMacMissed == 1034144, "lifetime counter preserved verbatim")
    expect(p.counters.rxMacMissedDelta == 0, "delta decoded")
    expect(p.sockets.udpBound["56301"] == true && p.sockets.udpBound["2368"] == true,
           "driver ingest ports decoded")
    expect(p.sockets.established["8766"] == 1, "established counts decoded")
    expect(p.ptpRole == "grandmaster", "ptp role decoded")
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
    // Nothing armed (or an old collector): the block is nil and the header
    // renders no chip — absent is nothing, never a state.
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
    // Streaming, in band, clean counters: three holds + two at-finalize.
    var s = RigSnapshot()
    s.mid360 = SensorReading(id: .mid360, state: .streaming, pointRateHz: 10.0,
                             imuRateHz: 200.0, clockOffsetMs: -13.6)
    s.jt128 = SensorReading(id: .jt128, state: .streaming, pointRateHz: 10.0,
                            imuRateHz: 200.0, clockOffsetMs: 0.4)
    s.pipeline = PipelineStatus(counters: PipelineCounters(
        udpInErrorsDelta: 0, udpRcvbufErrorsDelta: 0, rxMacMissedDelta: 0))
    let checks = GatePreflight.checks(s)
    expect(checks.first { $0.id == "clocks_in_band" }?.status == .holds, "in-band clocks hold")
    expect(checks.first { $0.id == "both_streaming" }?.status == .holds, "streaming holds")
    expect(checks.first { $0.id == "loss_deltas_zero" }?.status == .holds, "clean deltas hold")
    expect(checks.first { $0.id == "calibration" }?.status == .atFinalize,
           "calibration is scored at finalize, never guessed live")

    // Out-of-band mid clock: waiting, not a fake pass.
    s.mid360.clockOffsetMs = -450.6
    expect(GatePreflight.checks(s).first { $0.id == "clocks_in_band" }?.status == .waiting,
           "out-of-band clock reads waiting")

    // No probes at all: unavailable, never pass.
    let bare = GatePreflight.checks(RigSnapshot())
    expect(bare.first { $0.id == "clocks_in_band" }?.status == .unavailable,
           "no stamps → unavailable")
    expect(bare.first { $0.id == "loss_deltas_zero" }?.status == .unavailable,
           "no counters → unavailable")
}

func testSlewFitAndETAAreMeasuredNotInvented() {
    // A clean −10 ms/min convergence toward the band.
    var h = TelemetryHistory(capacity: 60)
    let t0 = Date(timeIntervalSinceReferenceDate: 800_000_000)
    for i in 0..<10 {
        h.append(TelemetrySample(at: t0.addingTimeInterval(Double(i) * 30),
                                 midOffsetMs: -300.0 + Double(i) * 5.0))
    }
    let now = t0.addingTimeInterval(9 * 30)
    guard let slew = h.slewPerMin(.midOffset, window: 600, now: now) else {
        expect(false, "slew fit exists with 10 samples")
        return
    }
    expect(abs(slew - 10.0) < 0.5, "fit recovers the +10 ms/min slew")
    guard let eta = h.minutesToBand(.midOffset, band: 50, window: 600, now: now) else {
        expect(false, "ETA exists while converging")
        return
    }
    // Last value −255, band 50 → 205 ms at 10 ms/min ≈ 20.5 min.
    expect(abs(eta - 20.5) < 1.5, "ETA is arithmetic on the fit")

    // Diverging series: no ETA — an arrival time would be an invention.
    var away = TelemetryHistory(capacity: 60)
    for i in 0..<10 {
        away.append(TelemetrySample(at: t0.addingTimeInterval(Double(i) * 30),
                                    midOffsetMs: -100.0 - Double(i) * 5.0))
    }
    expect(away.minutesToBand(.midOffset, band: 50, window: 600, now: now) == nil,
           "diverging clock → no ETA")
    // Two samples: no fit.
    var thin = TelemetryHistory(capacity: 60)
    thin.append(TelemetrySample(at: t0, midOffsetMs: -100))
    thin.append(TelemetrySample(at: t0.addingTimeInterval(30), midOffsetMs: -99))
    expect(thin.slewPerMin(.midOffset, window: 600, now: now) == nil,
           "fewer than 3 samples → no fit")
}

func testDecoderNoPipelineBlockStaysNil() {
    // An old collector has no pipeline key: the whole block is nil, and the
    // audit must render every probe-backed check as unavailable — never pass.
    guard let s = RigStatusDecoder.parse(json: parkedJSON, viewTunnelUp: false) else {
        expect(false, "parked JSON parses")
        return
    }
    expect(s.pipeline == nil, "absent pipeline key → nil block")
    let checks = PipelineAudit.checks(s)
    let link = checks.first { $0.id == "link" }
    let loss = checks.first { $0.id == "loss" }
    expect(link?.verdict == .unavailable, "no probe → link unavailable, not pass")
    expect(loss?.verdict == .unavailable, "no probe → loss unavailable, not pass")
}

func testAuditParkedRigIsPendingNotFailing() {
    guard let s = RigStatusDecoder.parse(json: pipelineJSON, viewTunnelUp: false) else {
        expect(false, "pipeline JSON parses")
        return
    }
    let checks = PipelineAudit.checks(s)
    func check(_ id: String) -> AuditCheck? { checks.first { $0.id == id } }
    expect(check("link")?.verdict == .pass, "1000/full carrier → link pass")
    expect(check("link")?.value == "1000/full", "link chip shows the negotiated figure")
    expect(check("loss")?.verdict == .pass, "zero deltas → loss pass (history ignored)")
    expect(check("ingest")?.verdict == .pass, "3/3 bound while parked still passes")
    expect(check("ptp")?.verdict == .pass && check("ptp")?.value == "grandmaster",
           "ptp role from the log")
    expect(check("clocks")?.verdict == .pending, "no sensor stamps → pending, not fail")
    expect(check("containers")?.value == "4/4", "all four containers counted")
    expect(check("rates")?.verdict == .pending && check("rates")?.value == "parked",
           "parked end-to-end is pending — a parked rig is calm")
    expect(check("disk")?.verdict == .pass && check("disk")?.value == "2.4 T",
           "big disk renders in T")
    expect(!checks.contains { $0.verdict == .fail }, "a parked healthy rig has no failing hop")
}

func testAuditDegradationsGoAmberOrRed() {
    guard var s = RigStatusDecoder.parse(json: pipelineJSON, viewTunnelUp: false) else {
        expect(false, "pipeline JSON parses")
        return
    }
    s.pipeline?.counters.rxMacMissedDelta = 812
    s.pipeline?.nic.speedMbps = 100
    s.pipeline?.sockets.udpBound["2368"] = false
    s.pipeline?.ptpRole = "fault"
    let checks = PipelineAudit.checks(s)
    func check(_ id: String) -> AuditCheck? { checks.first { $0.id == id } }
    expect(check("loss")?.verdict == .watch && check("loss")?.value == "+812",
           "climbing loss counters go amber with the delta shown")
    expect(check("link")?.verdict == .watch, "100 Mb/s negotiates → watch")
    expect(check("ingest")?.verdict == .fail, "unbound port with stack running → fail")
    expect(check("ptp")?.verdict == .fail, "ptp fault → fail")

    var carrier = s
    carrier.pipeline?.nic.carrier = false
    expect(PipelineAudit.checks(carrier).first { $0.id == "link" }?.verdict == .fail,
           "no carrier is a real cable/switch failure")
}

func testSpecDossiersCarryTheResearch() {
    let mid = RigSpec.dossier(.mid360)
    let jt = RigSpec.dossier(.jt128)
    expect(mid.contains { $0.value.contains("200k pts/s") }, "mid rate from the manual")
    expect(mid.contains { $0.caution && $0.value.contains("905 nm") },
           "aperture-damage caution is flagged")
    expect(jt.contains { $0.value.contains("1.15M") }, "jt single-return rate")
    expect(jt.contains { $0.caution && $0.value.lowercased().contains("power cycling") },
           "jt power-cycling caution is flagged")
    expect(RigSpec.spark.contains { $0.value.contains("grandmaster") }, "spark clock role")
    expect(RigSpec.nodeSummary(.mid360).contains { $0.contains("56301") },
           "node summary carries the real ports")
    expect(RigSpec.nodeSummary(.jt128).contains { $0.contains("2368") },
           "node summary carries the real ports")
}
