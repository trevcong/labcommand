import Foundation
import LabCommandCore

/// Tracks test pass count for computed totals (no hardcoded numbers).
var passCount = 0

func expect(_ cond: Bool, _ msg: String) {
    if !cond {
        fputs("FAIL: \(msg)\n", stderr)
        exit(1)
    }
}

/// Call after each test group passes.
func passed() {
    passCount += 1
}

if CommandLine.arguments.dropFirst().first == "live-room-probe" {
    exit(MainActor.assumeIsolated { runLiveRoomProbe(arguments: Array(CommandLine.arguments.dropFirst(2))) })
}

if CommandLine.arguments.dropFirst().first == "generate-room-fixtures" {
    generateRoomFixtures(arguments: Array(CommandLine.arguments.dropFirst(2)))
    exit(0)
}

print("Running LabCommandCore tests…")
runFixtureTests()
runPointCloudTests()
runCameraMathTests()
testDecoderLiveRig()
print("  rig decoder: live JSON → streaming duo, locked clocks, containers, events"); passed()
testDecoderParkedRigIsDarkNotBlocked()
print("  rig decoder: unpowered sensors read Parked, calm — not blocked"); passed()
testDecoderMissingKeysStayNil()
print("  rig decoder: omitted keys stay nil / unavailable; no fabricated Hz"); passed()
testDecoderGarbageIsNilNotACrash(); passed()
testPoweredNotStreamingIsHonest()
print("  rig decoder: ping + 0 Hz = Powered with a standby/driver explanation"); passed()
testGatesOfflineNeverArm()
print("  RigGate: Spark-offline arms nothing"); passed()
testGatesRecordNeedsPointsNotJustContainers()
print("  RigGate: a running container is not a running stream — record needs points"); passed()
testGatesRecordArmsWithOneLiveSensor(); passed()
testGatesJTControls()
print("  RigGate: wake/standby track JT power and recording state"); passed()
testGatesArmAtLock()
print("  RigGate: arm-at-lock needs Spark + drivers + not recording"); passed()
testRecordingHealthAndStop()
print("  recording: decoded session + stop armed + standby refused mid-take"); passed()
testClockConvergingIsVisible()
print("  clocks: Mid-360 slew reads Converging and diagnose explains it"); passed()
testDeskButtonsAlwaysThreeAndGated()
print("  RigDesk: three buttons, gates, Restart when running"); passed()
testDeskDualSessionNaming()
print("  RigDesk: dual takes are <ts>_DUAL_<code>_<tag>"); passed()
testDiagnoseParkedPointsAtPower(); passed()
testStagesReflectSnapshot()
print("  stages: Link/Drivers/Clocks/View/Record verbs track the snapshot"); passed()
testMissionLogAppendAndCap()
print("  event-log: MissionLog.append records start/stop/probe text"); passed()
testTailscalePeerParserOfflineSpark()
print("  TailscaleStatusParser: listed ≠ online; Active+CurAddr is reachable"); passed()
testSparkSSHTriesNextHostAfterTimeout()
print("  SparkSSH: timeout hops to next host"); passed()
testSessionPresetsMatchResearchPlan(); passed()
testTelemetryFormatUnavailable(); passed()
testTelemetryFormatCondense()
print("  TelemetryFormat: condense collapses ssh transcripts to facts"); passed()
testNoTypeScaleExceedsChromeCap()
print("  design contract: no chrome type above the 15pt cap"); passed()
testSpacingTokensMatchContract()
print("  design contract: spacing/radius/opacity tokens match DESIGN-VISION.md"); passed()
testNoPlaceholderScaffolding()
print("  design contract: no empty berths, dead destinations, or unwired controls"); passed()
testDecoderArmedBlock(); passed()
testDecoderNoArmedBlockStaysNil()
print("  armed takes: collector-held take decodes; absent is nothing, never a state"); passed()
testDecoderLastGateBlock(); passed()
testGatePreflightScoresFromLiveProbes()
print("  mission clock: gate verdict verbatim; preflight from live probes"); passed()
testSpecDossiersCarryTheResearch()
print("  spec dossiers: manual figures + cautions pinned"); passed()
testStatusAggregationSparkPreferSuccess()
print("  StatusAggregation: Spark prefer-success — SSH ok wins over ping failure"); passed()
testStatusAggregationTailscaleCombine()
print("  StatusAggregation: Tailscale combine — success/failure maps correctly"); passed()
testStatusAggregationEmptyReturnsUnknown()
print("  StatusAggregation: empty or unmatched checks → unknown"); passed()
testStatusAggregationBuildSnapshotSummary()
print("  StatusAggregation: buildSnapshot summary lines match overall status"); passed()
testStatusAggregationCombineWorstWins()
print("  StatusAggregation: combine worst-wins order (error > disconnected > unknown > connected)"); passed()
runPointCloudDecoderTests()
runPointCloud2CDRTests()
runRigTransformsTests()
runFoxgloveProtocolTests()
runLiveSessionTests()
runRobustBoundsTests()
runRoomMemoryCDRTests()
runRoomFixtureTests()
runChangeDetectionTests()
runChangeOverlayTests()
runLiveRoomTests()
runLiveRoomBridgeTests()
runLiveRoomViewTests()
runOperatorActionTests()
print("PASS LabCommandCoreTestsRunner (\(passCount) groups)")
