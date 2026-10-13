import Foundation
import LabCommandCore

// Assert-based tests (no XCTest) — exercises the real StatusAggregator.

func expect(_ cond: Bool, _ msg: String) {
    if !cond {
        fputs("FAIL: \(msg)\n", stderr)
        exit(1)
    }
}

func testSingleCheckMapping() {
    let ok = CheckOutcome(id: "spark_ssh", kind: .success, detail: "ok")
    let bad = CheckOutcome(id: "spark_ssh", kind: .failure, detail: "timeout")
    let err = CheckOutcome(id: "spark_ssh", kind: .error, detail: "spawn failed")
    let skip = CheckOutcome(id: "spark_ssh", kind: .skipped, detail: "no binary")
    expect(StatusAggregator.status(for: ok) == .connected, "success → connected")
    expect(StatusAggregator.status(for: bad) == .disconnected, "failure → disconnected")
    expect(StatusAggregator.status(for: err) == .error, "error → error")
    expect(StatusAggregator.status(for: skip) == .unknown, "skipped → unknown")
}

func testCombineWorstWins() {
    expect(StatusAggregator.combine([.connected, .connected]) == .connected, "all ok")
    expect(StatusAggregator.combine([.connected, .disconnected]) == .disconnected, "disc wins over ok")
    expect(StatusAggregator.combine([.connected, .unknown]) == .unknown, "unknown over ok")
    expect(StatusAggregator.combine([.disconnected, .error]) == .error, "error wins")
    expect(StatusAggregator.combine([]) == .unknown, "empty → unknown")
}

func testSnapshotConnected() {
    let checks = [
        CheckOutcome(id: "spark_ssh", kind: .success, detail: "tcong@spark"),
        CheckOutcome(id: "tailscale", kind: .success, detail: "active"),
    ]
    let snap = StatusAggregator.snapshot(from: checks)
    expect(snap.spark == .connected, "spark connected")
    expect(snap.tailscale == .connected, "ts connected")
    expect(snap.overall == .connected, "overall connected")
    expect(snap.summaryLine.contains("healthy") || snap.summaryLine.contains("reachable"),
           "summary mentions healthy/reachable: \(snap.summaryLine)")
    expect(snap.checks.count == 2, "preserves checks")
}

func testSnapshotDisconnected() {
    let checks = [
        CheckOutcome(id: "spark_ssh", kind: .failure, detail: "Permission denied"),
        CheckOutcome(id: "tailscale", kind: .success, detail: "active"),
    ]
    let snap = StatusAggregator.snapshot(from: checks)
    expect(snap.spark == .disconnected, "spark disc")
    expect(snap.overall == .disconnected, "overall disc when spark down")
    expect(snap.summaryLine.lowercased().contains("down") || snap.summaryLine.lowercased().contains("not reachable"),
           "summary reflects down: \(snap.summaryLine)")
}

func testSnapshotErrorAndUnknown() {
    let errSnap = StatusAggregator.snapshot(from: [
        CheckOutcome(id: "spark_ssh", kind: .error, detail: "posix error"),
    ])
    expect(errSnap.spark == .error, "spark error")
    expect(errSnap.overall == .error, "overall error")

    let empty = StatusAggregator.snapshot(from: [])
    expect(empty.spark == .unknown, "no spark checks → unknown")
    expect(empty.tailscale == .unknown, "no ts checks → unknown")
    expect(empty.overall == .unknown, "overall unknown")
}

func testIgnoresUnrelatedChecksForSpark() {
    let checks = [
        CheckOutcome(id: "unrelated", kind: .success, detail: "x"),
        CheckOutcome(id: "spark_ping", kind: .failure, detail: "100% loss"),
    ]
    let spark = StatusAggregator.status(from: checks, matching: StatusAggregator.sparkCheckIDs)
    expect(spark == .disconnected, "only spark ids count")
}

func testSparkPreferSuccessSSHOverPingFail() {
    let checks = [
        CheckOutcome(id: "spark_ssh", kind: .success, detail: "ok"),
        CheckOutcome(id: "spark_ping", kind: .failure, detail: "loss"),
        CheckOutcome(id: "tailscale", kind: .success, detail: "up"),
    ]
    let snap = StatusAggregator.snapshot(from: checks)
    expect(snap.spark == .connected, "ssh ok → spark connected despite ping fail")
    expect(snap.overall == .connected, "overall follows spark success")
}

func testMetricsLoaderFromDisk() {
    let home = FileManager.default.homeDirectoryForCurrentUser
    let paths = LabMetricsPaths.desktopDefaults(home: home)
    let bundle = LabMetricsLoader.load(paths: paths)
    // Product structure always binds.
    expect(bundle.offerLine == "BLACKBOX Verdict · draft", "offer structure: \(bundle.offerLine ?? "nil")")
    expect(bundle.accountsLine == "0 pilots", "accounts structure")
    expect(bundle.pipelineLine?.contains("BTF-0.1") == true, "pipeline structure: \(bundle.pipelineLine ?? "nil")")
    expect(bundle.boundModuleIDs.contains("offer"), "offer bound")
    expect(bundle.boundModuleIDs.contains("pipeline"), "pipeline bound")
    expect(bundle.boundModuleIDs.contains("accounts"), "accounts bound")

    // When Desktop metrics exist, research/work slots fill.
    if FileManager.default.fileExists(atPath: paths.c1Metrics) {
        expect(bundle.experimentLine != nil, "C1 → experiment")
        expect(bundle.experimentLine?.contains("sep") == true, "experiment has sep: \(bundle.experimentLine ?? "")")
        expect(bundle.datasetLine != nil, "C1 → dataset")
        expect(bundle.boundModuleIDs.contains("experiment"), "experiment bound")
    }
    if FileManager.default.fileExists(atPath: paths.sitevectorDemo) {
        expect(bundle.memoryLine != nil, "sitevector → memory")
        expect(bundle.boundModuleIDs.contains("memory"), "memory bound")
    }
    if FileManager.default.fileExists(atPath: paths.probeReport) {
        expect(bundle.runLine != nil, "probe → run")
        expect(bundle.runLine?.contains("probe") == true, "run mentions probe")
        expect(bundle.boundModuleIDs.contains("run"), "run bound")
    }
}

func testMetricsLoaderMissingFilesStayEmpty() {
    let tmp = FileManager.default.temporaryDirectory
        .appendingPathComponent("labcommand-metrics-empty-\(UUID().uuidString)", isDirectory: true)
    try? FileManager.default.createDirectory(at: tmp, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: tmp) }

    let paths = LabMetricsPaths(
        c1Metrics: tmp.appendingPathComponent("no-c1.json").path,
        sitevectorDemo: tmp.appendingPathComponent("no-sv.json").path,
        probeReport: tmp.appendingPathComponent("no-probe.json").path,
        blackboxDir: tmp.appendingPathComponent("no-bb").path
    )
    let bundle = LabMetricsLoader.load(paths: paths)
    expect(bundle.runLine == nil, "no probe → empty run")
    expect(bundle.experimentLine == nil, "no c1 → empty experiment")
    expect(bundle.memoryLine == nil, "no sitevector → empty memory")
    expect(bundle.datasetLine == nil, "no c1 → empty dataset")
    expect(bundle.resultsLine == nil, "no metrics → empty results")
    // Product structure still binds without data files.
    expect(bundle.offerLine != nil, "offer always")
    expect(bundle.pipelineLine != nil, "pipeline always")
    expect(bundle.accountsLine != nil, "accounts always")
}

print("Running LabCommandCore tests…")
testSingleCheckMapping()
testCombineWorstWins()
testSnapshotConnected()
testSnapshotDisconnected()
testSnapshotErrorAndUnknown()
testIgnoresUnrelatedChecksForSpark()
testSparkPreferSuccessSSHOverPingFail()
testMetricsLoaderFromDisk()
testMetricsLoaderMissingFilesStayEmpty()
testDecoderLiveRig()
print("  rig decoder: live JSON → streaming duo, locked clocks, containers, events")
testDecoderParkedRigIsDarkNotBlocked()
print("  rig decoder: unpowered sensors read Parked, calm — not blocked")
testDecoderMissingKeysStayNil()
print("  rig decoder: omitted keys stay nil / unavailable; no fabricated Hz")
testDecoderGarbageIsNilNotACrash()
testPoweredNotStreamingIsHonest()
print("  rig decoder: ping + 0 Hz = Powered with a standby/driver explanation")
testGatesOfflineNeverArm()
print("  RigGate: Spark-offline arms nothing")
testGatesRecordNeedsPointsNotJustContainers()
print("  RigGate: a running container is not a running stream — record needs points")
testGatesRecordArmsWithOneLiveSensor()
testGatesJTControls()
print("  RigGate: wake/standby track JT power and recording state")
testRecordingHealthAndStop()
print("  recording: decoded session + stop armed + standby refused mid-take")
testClockConvergingIsVisible()
print("  clocks: Mid-360 slew reads Converging and diagnose explains it")
testDeskButtonsAlwaysThreeAndGated()
print("  RigDesk: three buttons, gates, Restart when running")
testDeskDualSessionNaming()
print("  RigDesk: dual takes are <ts>_DUAL_<code>_<tag>")
testDiagnoseParkedPointsAtPower()
testStagesReflectSnapshot()
print("  stages: Link/Drivers/Clocks/View/Record verbs track the snapshot")
testSessionLibraryIndexesHouseSession()
print("  session-index: house_20260808.pcd 3544568 pts xyz ptp off from files")
testSessionLibraryFilterCompareAndNotes()
testMissionLogAppendAndCap()
print("  event-log: MissionLog.append records start/stop/probe text")
testDeviceRecoveryForFailedProbes()
testTailscalePeerParserOfflineSpark()
print("  TailscaleStatusParser: listed ≠ online; Active+CurAddr is reachable")
testSparkSSHTriesNextHostAfterTimeout()
print("  SparkSSH: timeout hops to next host")
testSessionPresetsMatchResearchPlan()
testTelemetryFormatUnavailable()
testNoTypeScaleExceedsChromeCap()
print("  design contract: no chrome type above the 15pt cap")
testSpacingTokensMatchContract()
print("  design contract: spacing/radius/opacity tokens match DESIGN-VISION.md")
testNoPlaceholderScaffolding()
print("  design contract: no empty berths, dead destinations, or unwired controls")
testHistoryKeepsGapsAsGaps()
testHistoryRangeIsNilWhenNothingWasMeasured()
testFlatSeriesStillGetsABand()
print("  telemetry history: gaps stay gaps, flat series get a band, buffer is bounded")
testHistoryIsBounded()
testBlindDurationCountsTheSilentRun()
print("  telemetry history: blind clock measures the silent run")
testHistorySurvivesARestart()
print("  telemetry history: survives a relaunch; corrupt file is empty, not fatal")
testProbeTextCondenses()
print("  probe text: an ssh transcript condenses to one fact")
testTrajectoryLoadsFromRealSession()
testTrajectoryMissingSidecarIsNilNotACrash()
print("  trajectory: real /Odometry sidecars load; missing or corrupt is nil, not fatal")
testDecoderPipelineBlock()
testDecoderNoPipelineBlockStaysNil()
print("  pipeline: per-hop probes decode; an old collector renders unavailable, never pass")
testDecoderArmedBlock()
testDecoderNoArmedBlockStaysNil()
print("  armed takes: collector-held take decodes; absent is nothing, never a state")
testDecoderLastGateBlock()
testGatePreflightScoresFromLiveProbes()
testSlewFitAndETAAreMeasuredNotInvented()
print("  mission clock: gate verdict verbatim; preflight from live probes; ETA from measured slew only")
testAuditParkedRigIsPendingNotFailing()
testAuditDegradationsGoAmberOrRed()
print("  pipeline audit: parked is pending not failing; loss/link/ptp degradations surface")
testSpecDossiersCarryTheResearch()
print("  spec dossiers: manual figures + cautions pinned")
print("PASS LabCommandCoreTestsRunner (61 groups)")
