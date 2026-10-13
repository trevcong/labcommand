import Foundation
import LabCommandCore

// A number says what is true now; a trace says when it stopped being true.
// These pin the parts a chart can get quietly wrong: fabricated zeros,
// interpolation across gaps, and an unbounded buffer.

func testHistoryKeepsGapsAsGaps() {
    var h = TelemetryHistory(capacity: 10)
    h.append(TelemetrySample(at: Date(), pointRateHz: 9.8))
    h.append(TelemetrySample(at: Date(), pointRateHz: nil))   // probe could not measure
    h.append(TelemetrySample(at: Date(), pointRateHz: 9.9))

    let s = h.series(.pointRate)
    expect(s.count == 3, "one entry per probe")
    expect(s[1] == nil, "an unmeasured probe stays nil, never 0")
    expect(s.compactMap { $0 }.count == 2, "only measured samples carry values")
}

func testHistoryRangeIsNilWhenNothingWasMeasured() {
    var h = TelemetryHistory(capacity: 5)
    h.append(TelemetrySample(at: Date(), pointRateHz: nil))
    h.append(TelemetrySample(at: Date(), pointRateHz: nil))
    expect(h.range(.pointRate) == nil, "no measurement means no axis to draw")

    h.append(TelemetrySample(at: Date(), pointRateHz: 10))
    expect(h.range(.pointRate) != nil, "one measurement is enough to plot")
}

/// A flat series must still get a band, or the trace divides by zero and
/// collapses onto an edge.
func testFlatSeriesStillGetsABand() {
    var h = TelemetryHistory(capacity: 5)
    for _ in 0..<4 { h.append(TelemetrySample(at: Date(), sparkLoad: 1.0)) }
    guard let r = h.range(.load) else {
        expect(false, "flat series must still produce a range")
        return
    }
    expect(r.hi > r.lo, "flat series gets a non-zero band: \(r.lo)…\(r.hi)")
    expect(r.lo < 1.0 && r.hi > 1.0, "the flat value sits inside the band")
}

func testHistoryIsBounded() {
    var h = TelemetryHistory(capacity: 8)
    for i in 0..<200 { h.append(TelemetrySample(at: Date(), sparkLoad: Double(i))) }
    expect(h.samples.count == 8, "ring buffer holds its capacity, got \(h.samples.count)")
    expect(h.latest?.sparkLoad == 199, "newest sample survives")
    expect(h.series(.load).compactMap { $0 }.first == 192, "oldest are dropped")
}

/// The five-hour failure, as a clock: how long has the rate read a measured zero.
func testBlindDurationCountsTheSilentRun() {
    let t0 = Date()
    var h = TelemetryHistory(capacity: 20)
    h.append(TelemetrySample(at: t0, pointRateHz: 9.8, blind: false))
    h.append(TelemetrySample(at: t0.addingTimeInterval(45), pointRateHz: 0, blind: true))
    h.append(TelemetrySample(at: t0.addingTimeInterval(90), pointRateHz: 0, blind: true))

    let d = h.blindDuration(now: t0.addingTimeInterval(120))
    expect(d != nil, "a blind run has a duration")
    expect(abs((d ?? 0) - 75) < 0.01, "measured from the first blind probe: \(d ?? -1)")

    var healthy = h
    healthy.append(TelemetrySample(at: t0.addingTimeInterval(135), pointRateHz: 9.9, blind: false))
    expect(healthy.blindDuration() == nil, "a live probe clears the blind clock")

    expect(TelemetryHistory.format(duration: 45) == "45s", "seconds")
    expect(TelemetryHistory.format(duration: 135) == "2m 15s", "minutes")
    expect(TelemetryHistory.format(duration: 18_000).hasPrefix("5h"), "hours: five of them")
}


/// History must survive a relaunch, or the "last known, 4m ago" readout is
/// empty exactly when the operator most wants it.
func testHistorySurvivesARestart() {
    let tmp = FileManager.default.temporaryDirectory
        .appendingPathComponent("lc-telemetry-\(UUID().uuidString)")
        .appendingPathComponent("telemetry.json")
    defer { try? FileManager.default.removeItem(at: tmp.deletingLastPathComponent()) }

    var h = TelemetryHistory(capacity: 6)
    let t0 = Date()
    h.append(TelemetrySample(at: t0, pointRateHz: 9.8, sparkLoad: 0.4))
    h.append(TelemetrySample(at: t0.addingTimeInterval(45), pointRateHz: nil, sparkLoad: 0.5))

    expect(TelemetryStore.save(h, to: tmp), "history writes to disk")
    let back = TelemetryStore.load(from: tmp)
    expect(back.samples.count == 2, "both samples come back, got \(back.samples.count)")
    expect(back.series(.pointRate)[1] == nil, "a gap survives the round trip as a gap")
    expect(back.lastKnown(.pointRate)?.value == 9.8, "last measured rate is recoverable")
    expect(back.lastKnown(.load)?.value == 0.5, "newest load wins")

    // A missing or corrupt file is an empty history, never a crash.
    let missing = TelemetryStore.load(from: tmp.deletingLastPathComponent()
        .appendingPathComponent("nope.json"))
    expect(missing.isEmpty, "missing file yields empty history")
    try? "not json".data(using: .utf8)!.write(to: tmp)
    expect(TelemetryStore.load(from: tmp).isEmpty, "corrupt file yields empty history")
}

/// The Right Now panel carries a fact, not an ssh transcript.
func testProbeTextCondenses() {
    expect(ProbeText.condense("ssh: connect to host spark-1f3e.tail8dfdc7.ts.net port 22:\nOperation timed out")
           == "ssh timed out", "timeout collapses to a fact")
    expect(ProbeText.condense("Permission denied (publickey).") == "ssh key rejected", "auth failure")
    expect(ProbeText.condense("ssh spark-ts -> spark-1f3e") == "ssh spark-ts -> spark-1f3e",
           "a short line passes through unchanged")
    expect(ProbeText.condense(String(repeating: "x", count: 200)).count <= 64,
           "long text is truncated")
}
