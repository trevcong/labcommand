import Foundation
import LabCommandCore

private func close(_ a: Double, _ b: Double, _ tolerance: Double = 1e-9) -> Bool {
    abs(a - b) <= tolerance
}

private func expectTUMError(_ text: String, line: Int?, contains needle: String, _ label: String) {
    do {
        _ = try TrailTrajectory.parseTUM(text)
        expect(false, "\(label): parsed but should fail")
    } catch let error as TrailError {
        if let line {
            if case .tum(let got, _) = error {
                expect(got == line, "\(label): error on line \(got), want \(line)")
            } else {
                expect(false, "\(label): \(error) is not a line error")
            }
        }
        expect("\(error)".contains(needle), "\(label): '\(error)' should mention '\(needle)'")
    } catch {
        expect(false, "\(label): unexpected \(error)")
    }
}

private func expectPCDError(_ data: Data, contains needle: String, _ label: String) {
    do {
        _ = try TrailMapCloud.decode(data)
        expect(false, "\(label): decoded but should fail")
    } catch {
        expect("\(error)".contains(needle), "\(label): '\(error)' should mention '\(needle)'")
    }
}

private func tempDirectory(_ name: String) -> URL {
    let url = FileManager.default.temporaryDirectory
        .appendingPathComponent("labcommand-trail-tests-\(getpid())")
        .appendingPathComponent(name)
    try? FileManager.default.removeItem(at: url)
    try? FileManager.default.createDirectory(at: url, withIntermediateDirectories: true)
    return url
}

// MARK: - TUM

func testTrailTUMFixtureNumbers() {
    let trail = try! TrailTrajectory.parseTUM(TrailFixture.tum())
    let s = trail.summary
    expect(s.poseCount == TrailFixture.poseCount, "fixture has \(TrailFixture.poseCount) poses, got \(s.poseCount)")
    expect(close(s.duration, TrailFixture.duration, 1e-6), "duration 66 s, got \(s.duration)")
    expect(close(s.pathLength, TrailFixture.pathLength, 1e-6), "path \(TrailFixture.pathLength), got \(s.pathLength)")
    expect(close(s.startToEnd, 0.04, 1e-9), "start→end is exactly 4.0 cm, got \(s.startToEnd)")
    expect(close(s.startToEndHorizontal, 0.04, 1e-9), "horizontal 4.0 cm")
    expect(close(s.startToEndVertical, 0), "vertical 0")
    expect(close(s.largestGap, 0.1, 1e-6), "largest gap 0.1 s, got \(s.largestGap)")
    expect(close(s.meanRateHz ?? 0, 10, 1e-6), "10 Hz")
    // An epoch stamp keeps sub-millisecond steps: Float would round 1.79e9 to 128 s.
    expect(close(trail.poses[1].t - trail.poses[0].t, 0.1, 1e-6), "epoch timestamps keep 0.1 s steps")
    expect(close(trail.poses[0].t, TrailFixture.t0, 1e-6), "first stamp is the epoch t0")
}

func testTrailTUMAcceptsCommentsAndNormalises() {
    let text = """
    # header

    1.0 0 0 0 0 0 0 1.004
      # indented comment
    2.0\t1\t0\t0\t0\t0\t0.7071068\t0.7071068
    """
    let trail = try! TrailTrajectory.parseTUM(text)
    expect(trail.poses.count == 2, "comments and blank lines are skipped")
    expect(close(trail.poses[0].qw, 1, 1e-12), "a near-unit quaternion is normalised")
    expect(close(trail.poses[1].yaw, .pi / 2, 1e-6), "tab-separated yaw reads 90°")
    expect(close(trail.pathLength, 1), "path is 1 m")
    let crlf = try! TrailTrajectory.parseTUM("# c\r\n1 0 0 0 0 0 0 1\r\n2 3 4 0 0 0 0 1\r\n")
    expect(crlf.poses.count == 2 && close(crlf.pathLength, 5), "CRLF line endings read")
    expectTUMError("1 0 0 0 0 0 0 1\r\n\r\n0 0 0 0 0 0 0 1\r\n", line: 3, contains: "backward", "CRLF line numbers count")
}

func testTrailTUMRejectsBrokenLines() {
    let ok = "1.0 0 0 0 0 0 0 1\n"
    expectTUMError(ok + "0.5 1 0 0 0 0 0 1\n", line: 2, contains: "goes backward", "backward timestamp")
    expectTUMError(ok + "1.0 1 0 0 0 0 0 1\n", line: 2, contains: "repeats", "repeated timestamp")
    expectTUMError(ok + "2.0 1 0 0 0 0 1\n", line: 2, contains: "expected 8 values", "seven fields")
    expectTUMError(ok + "2.0 nan 0 0 0 0 0 1\n", line: 2, contains: "finite", "NaN position")
    expectTUMError(ok + "2.0 1 0 0 0 0 0 inf\n", line: 2, contains: "finite", "infinite quaternion")
    expectTUMError(ok + "2.0 1 0 0 0 0 0 2\n", line: 2, contains: "quaternion length", "non-unit quaternion")
    expectTUMError("# only a comment\n\n", line: nil, contains: "no poses", "no poses")
    expectTUMError("1.0 a 0 0 0 0 0 1\n", line: 1, contains: "'a'", "word in a number column")
}

// MARK: - Playback

func testTrailSampleInterpolatesAndClamps() {
    let trail = try! TrailTrajectory.parseTUM(TrailFixture.tum())
    let start = trail.sample(atElapsed: 0)!
    expect(start.index == 0 && close(start.position.x, 0) && close(start.travelled, 0), "elapsed 0 is the first pose")
    // 3 s still, then 0.2 m/s along +x: elapsed 8.05 s is x = 1.01 m.
    let mid = trail.sample(atElapsed: 8.05)!
    expect(close(mid.position.x, 1.01, 1e-6) && close(mid.position.y, 0, 1e-9), "8.05 s is (1.01, 0), got \(mid.position)")
    expect(close(mid.travelled, 1.01, 1e-6), "travelled 1.01 m at 8.05 s, got \(mid.travelled)")
    // Epoch stamps carry ~2e-7 s of rounding, so the fraction is only good to ~1e-5.
    expect(mid.index == 80 && close(mid.fraction, 0.5, 1e-4), "between pose 80 and 81, halfway")
    let past = trail.sample(atElapsed: 1_000)!
    expect(past.index == trail.poses.count - 1, "past the end clamps to the last pose")
    expect(close(past.travelled, trail.pathLength, 1e-9), "travelled at the end is the whole path")
    expect(close(past.elapsed, trail.duration, 1e-9), "elapsed clamps to the duration")
    let before = trail.sample(atElapsed: -5)!
    expect(before.index == 0 && close(before.elapsed, 0), "before the start clamps to the first pose")

    // Heading takes the short way across ±180°.
    let half = { (deg: Double) in (sin(deg * .pi / 360), cos(deg * .pi / 360)) }
    let (z0, w0) = half(170), (z1, w1) = half(-170)
    let wrap = try! TrailTrajectory.parseTUM("0 0 0 0 0 0 \(z0) \(w0)\n1 1 0 0 0 0 \(z1) \(w1)\n")
    let yaw = wrap.sample(atElapsed: 0.5)!.yaw
    expect(close(abs(yaw), .pi, 1e-6), "170° → -170° passes through 180°, got \(yaw * 180 / .pi)°")

    let single = try! TrailTrajectory.parseTUM("5 1 2 3 0 0 0 1\n")
    let one = single.sample(atElapsed: 3)!
    expect(one.index == 0 && close(one.position.z, 3) && close(single.duration, 0), "a single pose samples to itself")
}

func testTrailClock() {
    expect(TrailClock.defaultSpeed(duration: 66) == 4, "66 s plays at 4× (16.5 s)")
    expect(TrailClock.defaultSpeed(duration: 10) == 0.5, "10 s plays at 0.5×")
    expect(TrailClock.defaultSpeed(duration: 182.3) == 8, "a 3-min lap plays at 8×")
    expect(TrailClock.defaultSpeed(duration: 3_600) == 16, "an hour caps at 16×")

    var clock = TrailClock(duration: 66, speed: 2)
    expect(!clock.isPlaying && clock.elapsed(at: 100) == 0, "a new clock is paused at 0")
    clock.play(at: 100)
    expect(close(clock.elapsed(at: 105), 10), "5 s at 2× is 10 s")
    clock.setSpeed(4, at: 105)
    expect(close(clock.elapsed(at: 106), 14), "speed change keeps the playhead, then 4×")
    clock.pause(at: 106)
    expect(close(clock.elapsed(at: 200), 14), "paused holds")
    clock.seek(to: 60, at: 200)
    expect(close(clock.elapsed(at: 300), 60) && !clock.isPlaying, "seek while paused stays paused")
    clock.play(at: 300)
    expect(close(clock.secondsToEnd(at: 300) ?? -1, 1.5), "6 s left at 4× is 1.5 s")
    expect(close(clock.elapsed(at: 400), 66) && clock.atEnd(at: 400), "the playhead stops at the end")
    clock.settle(at: 400)
    expect(!clock.isPlaying && close(clock.elapsed(at: 500), 66), "settle pauses at the end")
    clock.play(at: 500)
    expect(clock.isPlaying && close(clock.elapsed(at: 500), 0), "play at the end starts over")
    clock.seek(to: 99, at: 501)
    expect(!clock.isPlaying && close(clock.elapsed(at: 600), 66), "seek past the end clamps and stops")
    var empty = TrailClock(duration: 0)
    empty.play(at: 0)
    expect(!empty.isPlaying, "a zero-length recording does not play")
}

// MARK: - PCD

func testTrailPCDBinaryAndASCIIMatch() {
    let points = TrailFixture.mapPoints()
    let binary = try! TrailMapCloud.decode(TrailFixture.binaryPCD(points))
    let ascii = try! TrailMapCloud.decode(TrailFixture.asciiPCD(points))
    expect(binary.format == .binary && ascii.format == .ascii, "formats are reported")
    expect(binary.points == points, "FAST-LIO 32-byte binary reads back every point exactly")
    expect(ascii.points == points, "ascii %.9g reads back every point exactly")
    expect(binary.declared == points.count && binary.dropped == 0 && binary.stride == 1, "declared, dropped, stride")
}

private func pcd(_ header: String, _ body: Data) -> Data {
    var data = Data(header.utf8)
    data.append(body)
    return data
}

private func floats(_ values: [Float]) -> Data {
    var data = Data()
    for v in values { withUnsafeBytes(of: v.bitPattern.littleEndian) { data.append(contentsOf: $0) } }
    return data
}

func testTrailPCDLayouts() {
    // x y z only, 12 bytes, like the historical jt128_map.pcd.
    let xyz = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n"
    let a = try! TrailMapCloud.decode(pcd(xyz, floats([1, 2, 3, 4, 5, 6])))
    expect(a.points == [Point3D(x: 1, y: 2, z: 3), Point3D(x: 4, y: 5, z: 6)], "12-byte xyz binary")

    // x is not the first field, and one field has COUNT 2: offsets come from SIZE × COUNT.
    let shuffled = "VERSION 0.7\nFIELDS intensity rgb z y x\nSIZE 4 4 4 4 4\nTYPE F U F F F\nCOUNT 1 2 1 1 1\nPOINTS 1\nDATA binary\n"
    var body = floats([9])
    body.append(Data([0, 0, 0, 0, 0, 0, 0, 0]))
    body.append(floats([3, 2, 1]))
    let b = try! TrailMapCloud.decode(pcd(shuffled, body))
    expect(b.points == [Point3D(x: 1, y: 2, z: 3)], "offsets follow FIELDS/SIZE/COUNT, got \(b.points)")
    let shuffledASCII = "VERSION 0.7\nFIELDS intensity rgb z y x\nSIZE 4 4 4 4 4\nTYPE F U F F F\nCOUNT 1 2 1 1 1\nPOINTS 1\nDATA ascii\n9 0 0 3 2 1\n"
    let c = try! TrailMapCloud.decode(Data(shuffledASCII.utf8))
    expect(c.points == [Point3D(x: 1, y: 2, z: 3)], "ascii columns follow COUNT")

    // Double coordinates.
    let f8 = "VERSION 0.7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 1\nDATA binary\n"
    var doubles = Data()
    for v in [1.5, -2.25, 0.125] { withUnsafeBytes(of: v.bitPattern.littleEndian) { doubles.append(contentsOf: $0) } }
    expect(try! TrailMapCloud.decode(pcd(f8, doubles)).points == [Point3D(x: 1.5, y: -2.25, z: 0.125)], "F8 coordinates")

    // NaN rows are dropped and counted.
    let nan = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 3\nDATA binary\n"
    let d = try! TrailMapCloud.decode(pcd(nan, floats([1, 1, 1, .nan, 0, 0, 2, 2, 2])))
    expect(d.points.count == 2 && d.dropped == 1, "one NaN row dropped")
    let nanASCII = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 2\nDATA ascii\nnan nan nan\n1 2 3\r\n"
    let e = try! TrailMapCloud.decode(Data(nanASCII.utf8))
    expect(e.points == [Point3D(x: 1, y: 2, z: 3)] && e.dropped == 1, "ascii NaN dropped, CRLF read")

    // A big map keeps one in N.
    let many = TrailFixture.mapPoints()
    let thin = try! TrailMapCloud.decode(TrailFixture.binaryPCD(many), maxKept: 1_000)
    let stride = (many.count - 1) / 1_000 + 1
    expect(thin.stride == stride && thin.points.count == (many.count + stride - 1) / stride, "decimation keeps 1 in \(stride)")
    expect(thin.points[1] == many[stride], "decimation is an even stride")
}

func testTrailPCDRejects() {
    let head = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 2\n"
    expectPCDError(pcd(head + "DATA binary\n", floats([1, 2, 3, 4, 5])), contains: "needs 24", "truncated binary")
    expectPCDError(pcd(head + "DATA binary_compressed\n", Data(repeating: 0, count: 24)), contains: "binary_compressed", "compressed")
    expectPCDError(Data(head.utf8), contains: "no DATA", "no DATA line")
    expectPCDError(Data((head + "DATA ascii\n1 2 3\n").utf8), contains: "1 data lines", "ascii short")
    expectPCDError(Data((head + "DATA ascii\n1 2 3\n4 5 6\n7 8 9\n").utf8), contains: "more data lines", "ascii long")
    expectPCDError(Data((head + "DATA ascii\n1 2\n4 5 6\n").utf8), contains: "needs 3", "ascii short line")
    expectPCDError(Data("VERSION 0.7\nFIELDS x y\nSIZE 4 4\nTYPE F F\nCOUNT 1 1\nPOINTS 0\nDATA ascii\n".utf8),
                   contains: "x, y and z", "no z")
    expectPCDError(Data("VERSION 0.7\nFIELDS x y z\nSIZE 4 4\nTYPE F F F\nPOINTS 0\nDATA ascii\n".utf8),
                   contains: "different lengths", "SIZE too short")
    expectPCDError(Data("VERSION 0.7\nFIELDS x y z\nSIZE 2 2 2\nTYPE I I I\nCOUNT 1 1 1\nPOINTS 0\nDATA ascii\n".utf8),
                   contains: "TYPE F", "integer coordinates")
    // Header arithmetic that would overflow Int is an error, not a trap.
    let fields = "VERSION 0.7\nFIELDS x y z\nTYPE F F F\n"
    expectPCDError(Data((fields + "SIZE 4 4 4\nCOUNT 2305843009213693952 1 1\nPOINTS 0\nDATA binary\n").utf8),
                   contains: "overflow", "SIZE × COUNT overflows")
    expectPCDError(Data((fields + "SIZE 1 1 1\nCOUNT 9223372036854775807 1 1\nPOINTS 0\nDATA ascii\n").utf8),
                   contains: "overflow", "point size sum overflows")
    expectPCDError(Data((fields + "SIZE 4 4 4\nCOUNT 1 1 1\nWIDTH 9223372036854775807\nHEIGHT 2\nDATA binary\n").utf8),
                   contains: "overflow", "WIDTH × HEIGHT overflows")
}

// MARK: - Bounds

func testTrailBoundsRejectAbsurdValues() {
    let tMax = TrailBounds.maxAbsTime, cMax = TrailBounds.maxAbsCoordinate
    expect(tMax == 4_102_444_800 && cMax == 10_000, "provisional bounds: year 2100 in seconds, 10 km")

    // At the limit is accepted, both signs, and everything derived from it stays usable.
    let widest = try! TrailTrajectory.parseTUM(
        "\(-tMax) \(-cMax) \(-cMax) \(-cMax) 0 0 0 1\n\(tMax) \(cMax) \(cMax) \(cMax) 0 0 0 1\n")
    let summary = widest.summary
    expect(close(summary.duration, 2 * tMax) && summary.pathLength.isFinite && summary.meanRateHz != nil,
           "the widest accepted trail has a finite duration and length")
    expect(TrailReadout.clock(summary.duration) == "2279136:00:00", "its duration formats, got \(TrailReadout.clock(summary.duration))")
    let middle = widest.sample(atElapsed: summary.duration / 2)!
    let fraction = middle.elapsed / summary.duration
    expect(fraction.isFinite && close(fraction, 0.5), "the playhead fraction stays finite, got \(fraction)")
    expect(!TrailReadout.rows(summary: summary, sample: middle).isEmpty, "readout rows build")
    // Real stamps: relative (JT128 replay) and epoch seconds (Mid-360 replay).
    _ = try! TrailTrajectory.parseTUM("912.479 3.7 3.9 0.2 0 0 0 1\n1786223931.864 28.17 18.92 2.44 0 0 0 1\n")

    // Just past the limit fails on its line.
    let ok = "1.0 0 0 0 0 0 0 1\n"
    expectTUMError(ok + "\(tMax.nextUp) 0 0 0 0 0 0 1\n", line: 2, contains: "outside", "t just past year 2100")
    expectTUMError("\((-tMax).nextDown) 0 0 0 0 0 0 1\n", line: 1, contains: "outside", "t just before -year 2100")
    expectTUMError(ok + "2 \(cMax.nextUp) 0 0 0 0 0 1\n", line: 2, contains: "x 10000.000000000002 m is outside", "x just past 10 km")
    expectTUMError(ok + "2 0 \((-cMax).nextDown) 0 0 0 0 1\n", line: 2, contains: "y -10000.000000000002 m is outside", "y just past -10 km")
    expectTUMError(ok + "2 0 0 \(cMax.nextUp) 0 0 0 1\n", line: 2, contains: "z 10000.000000000002 m is outside", "z just past 10 km")

    // The earlier crash inputs. A 1e19 s span trapped TrailReadout.clock; ±1e308 stamps made
    // the trail-colour fraction NaN; ±1e308 positions reached the raster's inf/inf.
    expectTUMError("0 0 0 0 0 0 0 1\n1e19 0 0 0 0 0 0 1\n", line: 2, contains: "timestamp 1e+19 s is outside", "1e19 s span")
    expectTUMError("-1e308 0 0 0 0 0 0 1\n1e308 0 0 0 0 0 0 1\n", line: 1, contains: "timestamp -1e+308", "±1e308 stamps")
    expectTUMError("0 -1e308 -1e308 0 0 0 0 1\n1 1e308 1e308 0 0 0 0 1\n", line: 1, contains: "x -1e+308 m", "±1e308 positions")
    expectTUMError(ok + "2 0 1e20 0 0 0 0 1\n", line: 2, contains: "y 1e+20 m", "1e20 m position")
    // Wrong units say so.
    expectTUMError(ok + "1786223931864 0 0 0 0 0 0 1\n", line: 2, contains: "seconds expected", "millisecond stamp")
    expectTUMError(ok + "1.786223931864e18 0 0 0 0 0 0 1\n", line: 2, contains: "seconds expected", "nanosecond stamp")
    // A bad line costs the whole file, never a silent skip.
    expectTUMError(ok + "2 0 0 0 0 0 0 1\n3 0 0 1e5 0 0 0 1\n4 0 0 0 0 0 0 1\n", line: 3, contains: "z 100000.0 m", "one bad line mid-file")

    // The PCD reader uses the same rule but drops and counts, as it does a NaN point.
    let far: [Float] = [1, 1, 1, 1e20, 0, 0, .greatestFiniteMagnitude, 0, 0, 0, 0, -10_001, 10_000, -10_000, 10_000]
    let header = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 5\nDATA "
    let binary = try! TrailMapCloud.decode(pcd(header + "binary\n", floats(far)))
    let kept = [Point3D(x: 1, y: 1, z: 1), Point3D(x: 10_000, y: -10_000, z: 10_000)]
    expect(binary.points == kept && binary.dropped == 3, "binary drops 1e20, Float.max and -10001, got \(binary.points)")
    let rows = "1 1 1\n1e20 0 0\n3.4028235e38 0 0\n0 0 -10001\n10000 -10000 10000\n"
    let ascii = try! TrailMapCloud.decode(Data((header + "ascii\n" + rows).utf8))
    expect(ascii.points == kept && ascii.dropped == 3, "ascii drops the same three, got \(ascii.points)")
    let line = TrailReadout.mapLine(binary, raster: TrailRaster.build(map: binary.points, trajectory: nil))
    expect(line.contains("3 NaN or out-of-range dropped"), "the map line counts them, got \(line)")
    // The bounds are approved as provisional, and every message quoting one says so.
    expect(line.contains("(±10 km bound, provisional)"), "the map line says the bound is provisional, got \(line)")
    expectTUMError(ok + "\(tMax.nextUp) 0 0 0 0 0 0 1\n", line: 2, contains: "(year 2100, provisional bound)", "time bound labelled")
    expectTUMError(ok + "2 0 1e20 0 0 0 0 1\n", line: 2, contains: "m (provisional bound)", "position bound labelled")

    // The view shows the parser's error: a bad trail opens map-only with it listed.
    let fm = FileManager.default
    let badTrail = tempDirectory("bounds-bad-trail")
    try! Data((ok + "1.786223931864e18 0 0 0 0 0 0 1\n").utf8).write(to: badTrail.appendingPathComponent("trajectory.tum"))
    try! TrailFixture.binaryPCD(TrailFixture.mapPoints()).write(to: badTrail.appendingPathComponent("map.pcd"))
    let session = try! TrailSession.load(directory: badTrail.path)
    expect(session.trajectory == nil && session.map != nil, "an out-of-bounds trail opens map-only")
    expect(session.problems.contains { $0.hasPrefix("trajectory.tum line 2: timestamp") && $0.contains("seconds expected") },
           "the bounds error is listed, got \(session.problems)")
    try! fm.removeItem(at: badTrail.appendingPathComponent("map.pcd"))
    do {
        _ = try TrailSession.load(directory: badTrail.path)
        expect(false, "an out-of-bounds trail with no map does not open")
    } catch {
        expect("\(error)".contains("trajectory.tum line 2: timestamp"), "the refusal names the line, got \(error)")
    }
    try? fm.removeItem(at: badTrail)
}

// MARK: - Raster

func testTrailRasterLayers() {
    let trail = try! TrailTrajectory.parseTUM(TrailFixture.tum())
    let points = TrailFixture.mapPoints()
    let raster = TrailRaster.build(map: points, trajectory: trail)
    expect(raster.bandFollowsCart && close(raster.band.lowerBound, -0.7) && close(raster.band.upperBound, 0.7),
           "band is cart z 0 ± 0.7 m, got \(raster.band)")
    expect(raster.cellSize >= TrailRaster.minCellSize, "cells are at least 2 cm")
    expect(raster.width <= TrailRaster.defaultMaxCells && raster.height <= TrailRaster.defaultMaxCells, "raster is capped")
    expect(raster.footprint.count == raster.width * raster.height && raster.walls.count == raster.footprint.count, "layer sizes")
    expect(raster.binned > points.count * 9 / 10, "nearly every fixture point lands in the raster")

    /// Cell indices covering a world rectangle.
    func region(_ x0: Double, _ x1: Double, _ y0: Double, _ y1: Double) -> [Int] {
        let a = raster.cell(of: TrailPoint(x: x0, y: y1, z: 0))
        let b = raster.cell(of: TrailPoint(x: x1, y: y0, z: 0))
        var out: [Int] = []
        for cy in Int(a.y)...Int(b.y) {
            for cx in Int(a.x)...Int(b.x) { out.append(cy * raster.width + cx) }
        }
        return out
    }
    // Open floor in the near-left corner, away from walls, table, cabinet, and the trail:
    // floor and ceiling points only, so footprint and no wall points.
    let open = region(-1.2, -0.8, -1.2, -0.8)
    expect(open.contains { raster.footprint[$0] > 0 }, "open floor has footprint")
    expect(open.allSatisfy { raster.walls[$0] == 0 }, "open floor has no wall-band points")
    // The near wall (y = -1.5) and the table top (z = 0.75 m above the floor) are in the band.
    let nearWall = region(0, 4, -1.53, -1.47)
    expect(nearWall.filter { raster.walls[$0] > 0 }.count > 50, "the near wall shows in the wall layer")
    let table = region(1.5, 2.5, 0.7, 1.3)
    expect(table.filter { raster.walls[$0] > 0 }.count > 30, "the table shows in the wall layer")
    // The whole trail is inside the raster.
    for pose in trail.poses {
        let c = raster.cell(of: pose.position)
        expect(c.x >= 0 && c.y >= 0 && c.x < Double(raster.width) && c.y < Double(raster.height), "trail pose inside the raster")
    }
    let rgba = raster.rgba()
    expect(rgba.count == raster.width * raster.height * 4, "RGBA size")
    var transparent = 0
    for i in 0..<(raster.width * raster.height) where raster.footprint[i] == 0 {
        expect(rgba[i * 4 + 3] == 0, "an empty cell is transparent")
        transparent += 1
    }
    expect(transparent > 0, "the fixture leaves empty cells")
    let openAlpha = open.map { rgba[$0 * 4 + 3] }.max() ?? 255
    let wallAlpha = nearWall.map { rgba[$0 * 4 + 3] }.max() ?? 0
    expect(wallAlpha > openAlpha, "wall cells are more opaque than open floor (\(wallAlpha) vs \(openAlpha))")
    for i in stride(from: 0, to: rgba.count, by: 4) {
        if rgba[i] > rgba[i + 3] || rgba[i + 1] > rgba[i + 3] || rgba[i + 2] > rgba[i + 3] {
            expect(false, "RGBA is premultiplied")
        }
    }
}

func testTrailRasterMapOnlyAndEmpty() {
    let points = TrailFixture.mapPoints()
    let mapOnly = TrailRaster.build(map: points, trajectory: nil)
    expect(!mapOnly.bandFollowsCart, "without a trail the band comes from the map")
    expect(mapOnly.band.lowerBound > TrailFixture.floorZ + 0.1 && mapOnly.band.upperBound < TrailFixture.ceilingZ - 0.1,
           "the map band leaves out floor and ceiling, got \(mapOnly.band)")
    // Far outliers do not stretch the fit.
    var noisy = points
    noisy.append(Point3D(x: 400, y: -300, z: 50))
    let fitted = TrailRaster.build(map: noisy, trajectory: nil)
    expect(fitted.worldWidth < 10 && fitted.worldHeight < 8, "one far return does not stretch the view, got \(fitted.worldWidth) × \(fitted.worldHeight)")
    // Finite returns far past Int's range fall outside the raster instead of trapping.
    var huge = points
    huge.append(contentsOf: [Point3D(x: 1e20, y: 0, z: 0), Point3D(x: 0, y: -1e20, z: 0),
                             Point3D(x: .greatestFiniteMagnitude, y: -.greatestFiniteMagnitude, z: 0)])
    let clean = TrailRaster.build(map: points, trajectory: nil)
    let hugeFit = TrailRaster.build(map: huge, trajectory: nil)
    expect(hugeFit.binned == clean.binned && hugeFit.width == clean.width && hugeFit.height == clean.height,
           "1e20 and Float.max returns are skipped, got \(hugeFit.binned) binned vs \(clean.binned)")
    // A trail whose extent overflows Double still gives a raster. `parseTUM` now refuses this
    // file (testTrailBoundsRejectAbsurdValues); `build` is public, so a trail made in code still
    // reaches it and its own guard stays.
    func far(_ t: Double, _ v: Double) -> TrailPose {
        TrailPose(t: t, position: TrailPoint(x: v, y: v, z: 0), qx: 0, qy: 0, qz: 0, qw: 1)
    }
    let wide = TrailTrajectory(poses: [far(0, -1e308), far(1, 1e308)])
    let wideFit = TrailRaster.build(map: points, trajectory: wide)
    expect(wideFit.width >= 1 && wideFit.height >= 1 && wideFit.footprint.count == wideFit.width * wideFit.height,
           "a ±1e308 trail gives a raster, got \(wideFit.width) × \(wideFit.height)")
    let nothing = TrailRaster.build(map: [], trajectory: nil)
    expect(nothing.width >= 1 && nothing.binned == 0 && close(nothing.worldWidth, 11, 0.05), "an empty map is a 10 m square plus margin")
}

// MARK: - Session, manifest, launch

func testTrailSessionLoadsFixture() {
    let session = try! TrailSession.load(directory: TrailFixture.directory)
    expect(session.trajectory?.poses.count == TrailFixture.poseCount, "fixture trajectory loads")
    expect(session.map?.format == .binary && session.map?.points.count == TrailFixture.mapPoints().count, "fixture binary map loads")
    expect(session.manifest?.runID == "synthetic-loop" && session.name == "synthetic-loop", "manifest run id")
    expect(session.problems.isEmpty, "no problems in the fixture, got \(session.problems)")
    expect(session.accuracyLabel == "replay demonstrated; physical accuracy not evaluated", "contract wording for the label")
}

func testTrailManifestLabel() {
    func label(_ json: String) -> String? {
        let manifest = try? TrailManifest.parse(Data(json.utf8))
        return TrailSession(directory: "/x", trajectory: nil, map: nil, manifest: manifest, problems: []).accuracyLabel
    }
    expect(label("{\"physical_accuracy_evaluated\": true}") == nil, "only JSON true removes the label")
    expect(label("{\"physical_accuracy_evaluated\": false}") == TrailSession.notEvaluatedLabel, "false keeps it")
    expect(label("{\"physical_accuracy_evaluated\": \"true\"}") == TrailSession.notEvaluatedLabel, "a string keeps it")
    expect(label("{\"physical_accuracy_evaluated\": 1}") == TrailSession.notEvaluatedLabel, "a number keeps it")
    expect(label("{\"physical_accuracy_evaluated\": null}") == TrailSession.notEvaluatedLabel, "null keeps it")
    expect(label("{}") == TrailSession.notEvaluatedLabel, "absent keeps it")
    expect(label("[1, 2]") == TrailSession.notEvaluatedLabel, "a malformed manifest keeps it")
    let full = try! TrailManifest.parse(TrailFixture.manifest())
    expect(full.sensor == "synthetic" && full.fastlioRevision == nil && full.createdAt == "2026-10-19T00:00:00Z", "fields and nulls")
}

func testTrailSessionPartialFolders() {
    let fm = FileManager.default
    let mapOnly = tempDirectory("map-only")
    try! TrailFixture.binaryPCD(TrailFixture.mapPoints()).write(to: mapOnly.appendingPathComponent("map.pcd"))
    let a = try! TrailSession.load(directory: mapOnly.path)
    expect(a.trajectory == nil && a.map != nil, "a map-only folder opens")
    expect(a.problems.contains("trajectory.tum missing") && a.problems.contains("manifest.json missing"), "missing files are listed")
    expect(a.accuracyLabel == TrailSession.notEvaluatedLabel, "no manifest keeps the label")

    let badManifest = tempDirectory("bad-manifest")
    try! Data(TrailFixture.tum().utf8).write(to: badManifest.appendingPathComponent("trajectory.tum"))
    try! Data("{nope".utf8).write(to: badManifest.appendingPathComponent("manifest.json"))
    try! Data("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 9\nDATA binary\n".utf8)
        .write(to: badManifest.appendingPathComponent("map.pcd"))
    let b = try! TrailSession.load(directory: badManifest.path)
    expect(b.trajectory != nil && b.map == nil, "a trail with a broken map still opens")
    expect(b.problems.contains { $0.hasPrefix("map.pcd:") } && b.problems.contains { $0.contains("manifest.json is not") },
           "broken map and manifest are listed, got \(b.problems)")

    let empty = tempDirectory("empty")
    do {
        _ = try TrailSession.load(directory: empty.path)
        expect(false, "an empty folder does not open")
    } catch {
        expect("\(error)".contains("Nothing to show"), "empty folder says why")
    }
    do {
        _ = try TrailSession.load(directory: empty.appendingPathComponent("absent").path)
        expect(false, "a missing folder does not open")
    } catch {
        expect("\(error)".contains("not found"), "missing folder says not found")
    }
    try? fm.removeItem(at: mapOnly.deletingLastPathComponent())
}

func testTrailLaunchEnvironment() {
    let trail = LaunchMode.resolve(["LABCOMMAND_TRAIL_SESSION": " /tmp/run1 "])
    expect(trail == .trail(session: "/tmp/run1"), "the trail variable selects trail mode, trimmed")
    expect(trail.isolates, "trail mode isolates the network")
    expect(SnapshotSession(environment: ["LABCOMMAND_TRAIL_SESSION": "/tmp/run1"]).begin(rig: {
        expect(false, "trail mode never begins the rig")
    }) == .trail(session: "/tmp/run1"), "begin hands over the session")
    var calls = 0
    let session = SnapshotSession(environment: ["LABCOMMAND_TRAIL_SESSION": "/tmp/run1"])
    session.refresh(manual: true) { calls += 1 }
    session.pollRig { calls += 1 }
    session.runAction("start rig") { calls += 1 }
    session.restartTimer { calls += 1 }
    expect(calls == 0, "trail mode runs no refresh, poll, action, or timer")

    if case .configurationError(let message) = LaunchMode.resolve(["LABCOMMAND_TRAIL_SESSION": "/a", "LC_SNAPSHOT_A": "/b"]) {
        expect(message.contains("not both"), "both modes is a configuration error")
    } else {
        expect(false, "trail plus a snapshot variable is a configuration error")
    }
    if case .configurationError(let message) = LaunchMode.resolve(["LABCOMMAND_TRAIL_SESSION": "  "]) {
        expect(message.contains("empty"), "an empty trail variable says empty")
    } else {
        expect(false, "an empty trail variable is a configuration error")
    }
    expect(LaunchMode.resolve([:]) == .rig, "no variables is still the rig")
    expect(ChangeOverlay.snapshotDirs(["LABCOMMAND_TRAIL_SESSION": "/a"]) == nil, "a trail launch has no snapshot pair")

    expect(TrailLaunch.startFraction(["LABCOMMAND_TRAIL_AT": "0.5"]) == 0.5, "AT 0.5")
    expect(TrailLaunch.startFraction(["LABCOMMAND_TRAIL_AT": "7"]) == 1, "AT clamps to 1")
    expect(TrailLaunch.startFraction(["LABCOMMAND_TRAIL_AT": "x"]) == nil, "AT ignores a word")
    expect(TrailLaunch.startFraction([:]) == nil, "no AT plays from the start")

    let cwd = FileManager.default.currentDirectoryPath
    expect(TrailSession.resolveDirectory("fixture", resources: nil, cwd: cwd) == cwd + "/fixtures/trail/synthetic-loop",
           "fixture resolves to the repo copy without a bundle")
    expect(TrailSession.resolveDirectory("~/x", resources: nil, cwd: cwd) == NSHomeDirectory() + "/x", "tilde expands")
    expect(TrailSession.resolveDirectory("/abs/run", resources: nil, cwd: cwd) == "/abs/run", "absolute paths pass through")
}

func testTrailReadout() {
    expect(TrailReadout.metres(0.04) == "4.0 cm", "4.0 cm")
    expect(TrailReadout.metres(11.976259) == "11.98 m", "11.98 m")
    expect(TrailReadout.clock(66) == "1:06.0", "1:06.0, got \(TrailReadout.clock(66))")
    expect(TrailReadout.clock(5.25) == "0:05.2" || TrailReadout.clock(5.25) == "0:05.3", "0:05.x")
    expect(TrailReadout.clock(3_725) == "1:02:05", "past an hour")
    expect(TrailReadout.speed(0.5) == "0.5×" && TrailReadout.speed(4) == "4×", "speed labels")
    let trail = try! TrailTrajectory.parseTUM(TrailFixture.tum())
    let rows = TrailReadout.rows(summary: trail.summary, sample: trail.sample(atElapsed: 66))
    let text = Dictionary(uniqueKeysWithValues: rows.map { ($0.label, $0.value) })
    expect(text["Path length"] == "11.98 m / 11.98 m", "path at the end, got \(text["Path length"] ?? "nil")")
    expect(text["Elapsed"] == "1:06.0 / 1:06.0", "elapsed at the end")
    expect(text["Start → end"] == "4.0 cm" && text["  horizontal"] == "4.0 cm" && text["  vertical"] == "0.0 cm", "start → end rows")
    expect(text["Poses"] == "661 · 10.0 Hz" && text["Largest gap"] == "0.10 s", "pose rows")
    expect(TrailReadout.startToEndNote.contains("not the scored return error"), "the readout says it is not the score")
}

/// The committed fixture is exactly what the generator writes.
func testTrailFixtureIsCurrent() {
    for file in try! TrailFixture.files() {
        let path = TrailFixture.directory + "/" + file.name
        guard let committed = FileManager.default.contents(atPath: path) else {
            expect(false, "\(path) missing — run: swift run LabCommandCoreTestsRunner generate-trail-fixture")
            return
        }
        expect(committed == file.data, "\(path) is stale — run: swift run LabCommandCoreTestsRunner generate-trail-fixture")
    }
}

/// The app reads the label from Core. Trail is the fourth permanent tab (spec change in E2,
/// approved: Room, Settings, Rig, Trail), and only a trail launch opens on it.
func testTrailViewContract() {
    let root = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    let view = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/TrailReplayView.swift"), encoding: .utf8)) ?? ""
    let state = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/AppState.swift"), encoding: .utf8)) ?? ""
    let chrome = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/CommandCenterView.swift"), encoding: .utf8)) ?? ""
    let player = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/TrailPlayer.swift"), encoding: .utf8)) ?? ""
    expect(!view.isEmpty && !player.isEmpty, "TrailReplayView.swift and TrailPlayer.swift exist")
    expect(view.contains("accuracyLabel"), "the view shows the session's accuracy label")
    expect(view.contains("TrailReadout.startToEndNote"), "the view says start → end is raw")
    expect(view.contains("TrailReadout.walkcheckRows") && view.contains("TrailReadout.walkcheckNote"),
           "the view shows the walkcheck verdict and how it was measured")
    expect(view.contains("TrailReadout.walkcheckRows(w, stale: session.walkcheckStale != nil, unverifiable: session.walkcheckUnverifiable)")
            && view.contains("TrailReadout.verdictTone(w.result, stale: session.walkcheckStale != nil, unverifiable: session.walkcheckUnverifiable)")
            && view.contains("TrailReadout.verdictTone(verdict, stale: entry.walkcheckStale, unverifiable: entry.walkcheckUnverifiable)"),
           "the walk check block and the sessions list both mark stale or unverifiable and take their colour from verdictTone")
    expect(!view.contains("case \"PASS\""), "the view does not pick green from the verdict string itself")
    expect(state.contains("case trail"), "Dest has a trail case")
    expect(state.contains("static var navCases: [Dest] { [.liveRoom, .settings, .workspace, .trail] }"),
           "nav order is Room, Settings, Rig, Trail")
    expect(!chrome.contains("navItem(.trail)"), "Trail is drawn once, from navCases")
    expect(state.components(separatedBy: "dest = .trail").count == 2, "one place assigns dest = .trail")
    expect(state.contains("if isTrailLaunch {\n            dest = .trail"), "only a trail launch opens on Trail")
    expect(state.contains("var dest: Dest = .liveRoom"), "a normal launch still opens on Room")
    expect(player.contains("NSOpenPanel") && player.contains("canChooseDirectories = true"), "the tab has a folder picker")
    expect(player.contains("TrailLibrary.lastSessionKey"), "the picked folder is remembered")
    expect(player.contains("TrailLibrary.rescanShouldReload(opened:") && player.contains("load(directory: reload)"),
           "rescan reloads the opened session when its verdict files change")
    // In a walk-day folder the final verdict is shown, with Core's tone, and the walkcheck is an input.
    expect(view.contains("TrailReadout.walkdayRows(w)") && view.contains("TrailReadout.walkdayNote(w, accuracyLabel: session.accuracyLabel)")
            && view.contains("verdictTint(TrailReadout.walkdayTone(w))") && view.contains("verdictTint(TrailReadout.walkdayTone(final))"),
           "the walk-day block and the sessions list take text and colour from Core")
    let listWalkday = view.range(of: "if let final = entry.walkday")
    let listWalkcheck = view.range(of: "} else if let verdict = entry.walkcheckResult")
    expect(listWalkday != nil && listWalkcheck != nil && listWalkday!.lowerBound < listWalkcheck!.lowerBound,
           "the sessions list prefers the walk-day verdict over the walkcheck")
    expect(view.contains("session.walkday == nil\n                                        ? verdictTint(TrailReadout.verdictTone(w.result"),
           "a walkcheck that is an input to a walk-day verdict is not coloured as the verdict")
    expect(view.contains("TrailReadout.walkcheckInputNote(w, walkday: walkday)"), "and says it is an input")
    expect(player.contains("TrailLibrary.scan(roots: roots)") && player.contains("TrailLibrary.defaultRoots"),
           "the tab lists the walk-day root as well as sessions")
}

// MARK: - Walkcheck and the sessions folder

/// The shape `python -m spatio.walkcheck` writes (schema spatio.walkcheck.v1), trimmed.
private func walkcheckJSON(result: String, returnError cm: Double?, manifestEvaluated: Bool = false,
                           schema: String = "spatio.walkcheck.v1") -> Data {
    let returned = cm.map { """
    {"evaluated": true, "error_3d_m": \($0 / 100), "error_3d_cm": \($0), "error_horizontal_cm": 3.0,
     "error_vertical_cm": 1.5, "heading_error_deg": 0.8, "threshold_m": 0.1, "within_threshold": \($0 < 10)}
    """ } ?? "null"
    return Data("""
    {"schema": "\(schema)", "result": "\(result)", "reasons": \(result == "PASS" ? "[]" : "[\"return_error_over_threshold\"]"),
     "detail": null,
     "parameters": {"stationary_window_s": 5.0, "return_pass_m": 0.1, "wall_gate_label": "proposed, not yet approved by Trevor"},
     "manifest": {"present": true, "physical_accuracy_evaluated": \(manifestEvaluated)},
     "return_to_start": \(returned),
     "motion": {"path_length_m": 12.022997},
     "walls": {"status": "OK", "p95_point_to_plane_cm": 5.06719, "proposed_gate_result": "FAIL",
               "proposed_gate_label": "proposed, not yet approved by Trevor"}}
    """.utf8)
}

/// A PASS with `return_to_start` built from `returned` (the object's body, raw JSON such as
/// `null`, or nil to leave the key out).
private func walkcheckPASS(returned: String?, parameters: String = #"{"return_pass_m": 0.1}"#, reasons: String = "[]") -> Data {
    var body = #"{"schema": "spatio.walkcheck.v1", "result": "PASS", "reasons": \#(reasons), "parameters": \#(parameters)"#
    if let returned {
        body += returned == "null" ? #", "return_to_start": null"# : #", "return_to_start": {\#(returned)}"#
    }
    return Data((body + "}").utf8)
}

func testTrailWalkcheckParse() {
    let fail = try! TrailWalkcheck.parse(walkcheckJSON(result: "FAIL", returnError: 12.34))
    expect(fail.result == "FAIL" && fail.reasons == ["return_error_over_threshold"], "verdict and reasons")
    expect(close(fail.error3DCm ?? 0, 12.34) && close(fail.horizontalCm ?? 0, 3) && close(fail.verticalCm ?? 0, 1.5), "return error parts")
    expect(close(fail.thresholdM ?? 0, 0.1) && close(fail.windowS ?? 0, 5) && close(fail.pathLengthM ?? 0, 12.022997), "threshold, window, path")
    expect(fail.wallGate == "FAIL" && close(fail.wallP95Cm ?? 0, 5.06719), "proposed wall gate")
    let rows = Dictionary(uniqueKeysWithValues: TrailReadout.walkcheckRows(fail, stale: false).map { ($0.label, $0.value) })
    expect(rows["Verdict"] == "FAIL", "verdict row")
    expect(rows["Return error"] == "12.3 cm · need < 10.0 cm", "return error row, got \(rows["Return error"] ?? "nil")")
    expect(rows["  horizontal"] == "3.0 cm" && rows["  vertical"] == "1.5 cm" && rows["  heading"] == "0.8°", "parts rows")
    expect(rows["Wall gate"] == "FAIL · p95 5.1 cm", "wall row, got \(rows["Wall gate"] ?? "nil")")
    let note = TrailReadout.walkcheckNote(fail, stale: nil, accuracyLabel: TrailSession.notEvaluatedLabel)
    expect(note.contains("first 5 s") && note.contains("not aligned"), "the note says how the return error was measured")
    expect(note.contains("proposed, not yet approved by Trevor"), "the note says the wall gate is proposed")
    expect(note.contains("not a measured physical walk"), "an unevaluated session says the verdict scores the replay")
    expect(!TrailReadout.walkcheckNote(fail, stale: nil, accuracyLabel: nil).contains("not a measured physical walk"),
           "an evaluated session drops that sentence")

    let unscored = try! TrailWalkcheck.parse(walkcheckJSON(result: "NOT-EVALUABLE", returnError: nil))
    expect(unscored.error3DCm == nil && unscored.horizontalCm == nil, "null return_to_start reads as not scored")
    expect(TrailReadout.walkcheckRows(unscored, stale: false).contains { $0.label == "Return error" && $0.value == "not scored" }, "not scored row")

    // Was a PASS with no scoring evidence; a PASS like that is now refused (below), so this
    // case uses FAIL to keep checking the unscored wall section and the boolean.
    let skippedWalls = try! TrailWalkcheck.parse(Data("""
    {"schema": "spatio.walkcheck.v1", "result": "FAIL", "walls": {"status": "NO_MAP", "proposed_gate_result": "FAIL"},
     "return_to_start": {"error_3d_cm": true}}
    """.utf8))
    expect(skippedWalls.wallGate == nil, "an unscored wall section shows no gate")
    expect(skippedWalls.error3DCm == nil, "a boolean is not a number")

    // A PASS carries what walkcheck always writes for one. A wall-gate FAIL does not
    // contradict it (walkcheck's wall gate never sets the verdict).
    let pass = try! TrailWalkcheck.parse(walkcheckJSON(result: "PASS", returnError: 4))
    expect(pass.result == "PASS" && close(pass.error3DCm ?? 0, 4) && pass.wallGate == "FAIL", "a scored PASS with a wall FAIL reads")
    expect(TrailReadout.verdictTone("PASS", stale: false) == .good, "a current PASS is green")
    let justUnder = walkcheckPASS(returned: #""evaluated": true, "within_threshold": true, "error_3d_cm": 9.999999, "threshold_m": 0.1"#)
    expect((try? TrailWalkcheck.parse(justUnder))?.result == "PASS", "9.999999 cm under a 0.1 m threshold passes")

    // A PASS with missing, mistyped or contradictory evidence is refused, so the session
    // lists it as a problem and nothing shows green.
    let scored = #""evaluated": true, "within_threshold": true, "threshold_m": 0.1"#
    for (data, needle, label) in [
        (walkcheckPASS(returned: nil), "return_to_start is missing", "PASS with no return_to_start"),
        (walkcheckPASS(returned: "null"), "return_to_start is missing", "PASS with return_to_start null"),
        (Data(#"{"schema": "spatio.walkcheck.v1", "result": "PASS"}"#.utf8), "reasons", "PASS with nothing else"),
        (Data(#"{"schema": "spatio.walkcheck.v1", "result": "PASS", "walls": {"status": "NO_MAP"}, "return_to_start": {"error_3d_cm": true}}"#.utf8),
         "reasons", "the old evidence-free PASS fixture"),
        (walkcheckPASS(returned: scored + #", "error_3d_cm": "4.0""#), "error_3d_cm", "error as a string"),
        (walkcheckPASS(returned: scored + #", "error_3d_cm": true"#), "error_3d_cm", "error as a boolean"),
        (walkcheckPASS(returned: scored), "error_3d_cm", "error missing"),
        (walkcheckPASS(returned: scored + #", "error_3d_cm": -1.0"#), "error_3d_cm", "negative error"),
        (walkcheckPASS(returned: #""evaluated": true, "within_threshold": true, "error_3d_cm": 4.0"#, parameters: "{}"),
         "threshold_m", "no threshold anywhere"),
        (walkcheckPASS(returned: #""evaluated": true, "within_threshold": true, "error_3d_cm": 4.0, "threshold_m": "0.1""#, parameters: "{}"),
         "threshold_m", "threshold as a string"),
        (walkcheckPASS(returned: #""evaluated": true, "within_threshold": true, "error_3d_cm": 4.0, "threshold_m": 0"#, parameters: "{}"),
         "threshold_m", "zero threshold"),
        (walkcheckPASS(returned: #""evaluated": true, "within_threshold": false, "error_3d_cm": 4.0, "threshold_m": 0.1"#),
         "within_threshold", "within_threshold false"),
        (walkcheckPASS(returned: #""evaluated": true, "within_threshold": 1, "error_3d_cm": 4.0, "threshold_m": 0.1"#),
         "within_threshold", "within_threshold 1, not true"),
        (walkcheckPASS(returned: #""within_threshold": true, "error_3d_cm": 4.0, "threshold_m": 0.1"#),
         "evaluated", "evaluated missing"),
        (walkcheckPASS(returned: scored + #", "error_3d_cm": 224.19"#), "is not below", "224 cm against 10 cm"),
        (walkcheckPASS(returned: scored + #", "error_3d_cm": 10.0"#), "is not below", "exactly at the threshold"),
        (walkcheckPASS(returned: scored + #", "error_3d_cm": 4.0"#, reasons: #"["return_error"]"#), "reasons", "PASS with a reason"),
        (walkcheckPASS(returned: scored + #", "error_3d_cm": 4.0"#, reasons: #""none""#), "reasons", "reasons not a list"),
    ] {
        do {
            _ = try TrailWalkcheck.parse(data)
            expect(false, "\(label) should not parse")
        } catch {
            expect("\(error)".contains("result PASS but") && "\(error)".contains(needle), "\(label): '\(error)' should mention '\(needle)'")
        }
    }
    // The same contradictions on a FAIL are shown as read: a FAIL never shows green.
    _ = try! TrailWalkcheck.parse(Data(#"{"schema": "spatio.walkcheck.v1", "result": "FAIL", "reasons": ["stationary_trajectory"], "return_to_start": {"within_threshold": true}}"#.utf8))
    expect(TrailReadout.verdictTone("FAIL", stale: false) == .bad && TrailReadout.verdictTone("NOT-EVALUABLE", stale: false) == .warn,
           "FAIL is red, NOT-EVALUABLE amber")

    // Walkcheck's provisional label rides on the window sentence.
    let labelled = try! TrailWalkcheck.parse(walkcheckPASS(
        returned: scored + #", "error_3d_cm": 4.0"#,
        parameters: #"{"stationary_window_s": 5.0, "provisional_label": "provisional (approved for now, revisit after real data)"}"#))
    expect(TrailReadout.walkcheckNote(labelled, stale: nil, accuracyLabel: nil)
            .contains("first 5 s to mean of the last 5 s, end not aligned onto start; window provisional (approved for now"),
           "the window is labelled provisional, got \(TrailReadout.walkcheckNote(labelled, stale: nil, accuracyLabel: nil))")
    // Without walkcheck's labels, the window and the wall gate still read provisional.
    var unlabelled = fail
    unlabelled.wallGateLabel = nil
    unlabelled.provisionalLabel = nil
    let plain = TrailReadout.walkcheckNote(unlabelled, stale: nil, accuracyLabel: nil)
    expect(plain.contains("end not aligned onto start; window provisional.") && plain.contains("Wall gate is provisional."),
           "provisional is the fallback label, got \(plain)")

    for (data, needle, label) in [
        (walkcheckJSON(result: "PASS", returnError: 4, schema: "spatio.walkcheck.v2"), "schema", "another schema"),
        (walkcheckJSON(result: "MAYBE", returnError: 4), "result", "an unknown result"),
        (Data("[1]".utf8), "not a JSON object", "not an object"),
        (Data("{nope".utf8), "not a JSON object", "malformed"),
    ] {
        do {
            _ = try TrailWalkcheck.parse(data)
            expect(false, "\(label) should not parse")
        } catch {
            expect("\(error)".contains(needle), "\(label): '\(error)' should mention '\(needle)'")
        }
    }
}

/// Requirement: only the session's own manifest `physical_accuracy_evaluated: true` clears the label.
func testTrailWalkcheckNeverClearsLabel() {
    let fm = FileManager.default
    let dir = tempDirectory("walkcheck-pass")
    let tum = dir.appendingPathComponent("trajectory.tum")
    try! Data(TrailFixture.tum().utf8).write(to: tum)
    try! Data("{\"run_id\": \"walkcheck-pass\", \"physical_accuracy_evaluated\": false}".utf8)
        .write(to: dir.appendingPathComponent("manifest.json"))
    try! walkcheckJSON(result: "PASS", returnError: 4, manifestEvaluated: true).write(to: dir.appendingPathComponent("walkcheck.json"))
    var session = try! TrailSession.load(directory: dir.path)
    expect(session.walkcheck?.result == "PASS", "the walkcheck verdict loads")
    expect(session.walkcheckStale == nil, "a walkcheck newer than its inputs is current")
    expect(session.accuracyLabel == TrailSession.notEvaluatedLabel,
           "a walkcheck PASS that copies evaluated=true does not clear the label")
    expect(session.topdownPath == nil, "no topdown.png, no picture")

    try! fm.removeItem(at: dir.appendingPathComponent("manifest.json"))
    session = try! TrailSession.load(directory: dir.path)
    expect(session.accuracyLabel == TrailSession.notEvaluatedLabel, "no manifest plus a walkcheck PASS keeps the label")

    try! Data("{\"physical_accuracy_evaluated\": true}".utf8).write(to: dir.appendingPathComponent("manifest.json"))
    try! Data([0x89, 0x50, 0x4E, 0x47]).write(to: dir.appendingPathComponent("topdown.png"))
    session = try! TrailSession.load(directory: dir.path)
    expect(session.accuracyLabel == nil, "the session's own manifest true clears it")
    expect(session.topdownPath == dir.appendingPathComponent("topdown.png").path, "topdown.png is found")

    // A walkcheck scored before the trajectory was rewritten is flagged, not hidden.
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: -3600)],
                          ofItemAtPath: dir.appendingPathComponent("walkcheck.json").path)
    session = try! TrailSession.load(directory: dir.path)
    expect(session.problems.contains { $0.contains("walkcheck.json is older than trajectory.tum") }, "stale walkcheck is listed")
    expect(session.walkcheck != nil, "a stale walkcheck still shows")
    // ...but marked stale beside the verdict, and never green.
    func expectStalePASS(_ session: TrailSession, _ label: String) {
        guard let w = session.walkcheck, let stale = session.walkcheckStale else {
            return expect(false, "\(label): walkcheck should load and be stale")
        }
        let verdict = TrailReadout.walkcheckRows(w, stale: true).first
        expect(verdict?.label == "Verdict" && verdict?.value == "PASS · stale", "\(label): the verdict row says stale")
        expect(TrailReadout.verdictTone(w.result, stale: true) == .warn, "\(label): a stale PASS is not green")
        expect(TrailReadout.walkcheckNote(w, stale: stale, accuracyLabel: nil).hasPrefix("Stale: \(stale). Re-run walkcheck"),
               "\(label): the note leads with why")
    }
    expectStalePASS(session, "trajectory newer")
    expect(TrailReadout.verdictTone("FAIL", stale: true) == .warn, "a stale FAIL is amber, not red")

    // A map rewritten after scoring makes it stale too, with the trajectory still older.
    let map = dir.appendingPathComponent("map.pcd")
    try! TrailFixture.binaryPCD(TrailFixture.mapPoints()).write(to: map)
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: -7200)], ofItemAtPath: tum.path)
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: -3600)],
                          ofItemAtPath: dir.appendingPathComponent("walkcheck.json").path)
    session = try! TrailSession.load(directory: dir.path)
    expect(session.walkcheckStale == "walkcheck.json is older than map.pcd; its verdict may be for earlier files",
           "map newer only, got \(session.walkcheckStale ?? "nil")")
    expect(session.problems.contains(session.walkcheckStale ?? "-"), "and it is listed under Files")
    expectStalePASS(session, "map newer")
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: -60)], ofItemAtPath: tum.path)
    session = try! TrailSession.load(directory: dir.path)
    expect(session.walkcheckStale?.hasPrefix("walkcheck.json is older than trajectory.tum and map.pcd") == true,
           "both newer are both named, got \(session.walkcheckStale ?? "nil")")
    try! fm.setAttributes([.modificationDate: Date()], ofItemAtPath: dir.appendingPathComponent("walkcheck.json").path)
    session = try! TrailSession.load(directory: dir.path)
    expect(session.walkcheckStale == nil, "a re-run walkcheck is current again")
    try! fm.removeItem(at: map)

    try! Data("{\"schema\": \"other\"}".utf8).write(to: dir.appendingPathComponent("walkcheck.json"))
    session = try! TrailSession.load(directory: dir.path)
    expect(session.walkcheck == nil && session.problems.contains { $0.contains("walkcheck.json: schema") },
           "an unreadable walkcheck is a listed problem, got \(session.problems)")
    expect(session.trajectory != nil, "and the session still opens")

    try! walkcheckPASS(returned: nil).write(to: dir.appendingPathComponent("walkcheck.json"))
    session = try! TrailSession.load(directory: dir.path)
    expect(session.walkcheck == nil && session.problems.contains { $0.contains("result PASS but return_to_start is missing") },
           "an evidence-free PASS is a listed problem, not a verdict, got \(session.problems)")
    try? fm.removeItem(at: dir)
}

func testTrailLibraryScanAndReopen() {
    let fm = FileManager.default
    let root = tempDirectory("sessions-root")
    func make(_ name: String, tum: Bool, map: Bool, age: TimeInterval, walkcheck: String? = nil) {
        let dir = root.appendingPathComponent(name)
        try! fm.createDirectory(at: dir, withIntermediateDirectories: true)
        let stamp = Date(timeIntervalSinceNow: -age)
        if tum {
            try! Data("1 0 0 0 0 0 0 1\n".utf8).write(to: dir.appendingPathComponent("trajectory.tum"))
            try! fm.setAttributes([.modificationDate: stamp], ofItemAtPath: dir.appendingPathComponent("trajectory.tum").path)
        }
        if map {
            try! Data("x".utf8).write(to: dir.appendingPathComponent("map.pcd"))
            try! fm.setAttributes([.modificationDate: stamp], ofItemAtPath: dir.appendingPathComponent("map.pcd").path)
        }
        if let walkcheck {
            try! walkcheckJSON(result: walkcheck, returnError: 12).write(to: dir.appendingPathComponent("walkcheck.json"))
        }
    }
    make("old-run", tum: true, map: false, age: 86_400 * 30)
    make("new-run", tum: false, map: true, age: 10)
    make("scored-run", tum: true, map: true, age: 86_400, walkcheck: "FAIL")
    make("still-starting", tum: false, map: false, age: 0)
    make(".hidden-run", tum: true, map: true, age: 0)
    try! Data("x".utf8).write(to: root.appendingPathComponent("notes.txt"))

    let entries = TrailLibrary.scan(root: root.path)
    expect(entries.map(\.name) == ["new-run", "scored-run", "old-run"],
           "sessions newest first; empty, hidden and plain files skipped, got \(entries.map(\.name))")
    expect(entries[0].hasMap && !entries[0].hasTrajectory, "map-only flags")
    expect(entries[1].walkcheckResult == "FAIL" && entries[2].walkcheckResult == nil, "the list carries the walkcheck verdict")
    expect(entries[1].directory == root.appendingPathComponent("scored-run").path, "entries open by full path")
    expect(!entries[1].walkcheckStale, "a walkcheck newer than its inputs is current in the list")
    // The list uses the same freshness rule as the opened session.
    let scoredWalkcheck = root.appendingPathComponent("scored-run/walkcheck.json").path
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: -86_400 * 2)], ofItemAtPath: scoredWalkcheck)
    let rescanned = TrailLibrary.scan(root: root.path).first { $0.name == "scored-run" }
    expect(rescanned?.walkcheckStale == true && rescanned?.walkcheckResult == "FAIL", "an older walkcheck is stale in the list")
    expect(TrailReadout.verdictText("FAIL", stale: true) == "FAIL · stale" && TrailReadout.verdictText("PASS", stale: false) == "PASS",
           "the list marks stale beside the verdict")
    try! walkcheckPASS(returned: nil).write(to: URL(fileURLWithPath: scoredWalkcheck))
    let refused = TrailLibrary.scan(root: root.path).first { $0.name == "scored-run" }
    expect(refused?.walkcheckResult == nil && refused?.walkcheckStale == false, "an evidence-free PASS shows no verdict in the list")
    expect(TrailLibrary.scan(root: root.appendingPathComponent("absent").path).isEmpty, "a missing root lists nothing")

    let saved = root.appendingPathComponent("scored-run").path
    expect(TrailLibrary.reopenTarget(saved: saved) == saved, "a remembered folder that exists reopens")
    expect(TrailLibrary.reopenTarget(saved: root.appendingPathComponent("gone").path) == nil, "a removed folder does not")
    expect(TrailLibrary.reopenTarget(saved: nil) == nil && TrailLibrary.reopenTarget(saved: "") == nil, "nothing saved, nothing reopened")
    expect(TrailLibrary.reopenTarget(saved: root.appendingPathComponent("notes.txt").path) == nil, "a file is not a session folder")
    expect(TrailLibrary.pickerStart(saved: saved, root: "/nonexistent") == root.path, "the picker starts beside the last session")
    expect(TrailLibrary.pickerStart(saved: nil, root: root.path) == root.path, "else in the sessions root")
    expect(TrailLibrary.pickerStart(saved: "/gone/run", root: "/nonexistent") == nil, "else wherever the panel likes")
    expect(TrailLibrary.defaultRoot == NSHomeDirectory() + "/Developer/spatio-data/sessions", "the shared-contract root")
    try? fm.removeItem(at: root)
}

/// Deleting trajectory.tum from a scored session must not leave a green PASS on the map.
func testTrailDeletedTrajectoryIsUnverifiable() {
    let fm = FileManager.default
    let root = tempDirectory("deleted-trajectory")
    let dir = root.appendingPathComponent("scored")
    try! fm.createDirectory(at: dir, withIntermediateDirectories: true)
    let tum = dir.appendingPathComponent("trajectory.tum")
    let map = dir.appendingPathComponent("map.pcd")
    let walk = dir.appendingPathComponent("walkcheck.json")
    try! Data(TrailFixture.tum().utf8).write(to: tum)
    try! TrailFixture.binaryPCD(TrailFixture.mapPoints()).write(to: map)
    try! walkcheckJSON(result: "PASS", returnError: 4).write(to: walk)

    let opened = try! TrailSession.load(directory: dir.path)
    expect(opened.walkcheck?.result == "PASS" && opened.walkcheckStale == nil && !opened.walkcheckUnverifiable,
           "a scored PASS with its trajectory still there is current")
    expect(TrailReadout.verdictTone("PASS", stale: false, unverifiable: false) == .good, "and it is green")
    let inputs = TrailLibrary.VerdictInputs.read(directory: dir.path)

    try! fm.removeItem(at: tum)
    expect(TrailLibrary.rescanShouldReload(opened: inputs, now: TrailLibrary.VerdictInputs.read(directory: dir.path)),
           "deleting the trajectory makes rescan reload the opened session")
    let mapOnly = try! TrailSession.load(directory: dir.path)
    expect(mapOnly.trajectory == nil && mapOnly.map != nil, "deleting the trajectory leaves a map-only session")
    expect(mapOnly.problems.contains("trajectory.tum missing"), "the missing file is listed")
    expect(mapOnly.walkcheck?.result == "PASS", "the walkcheck still parses")
    expect(mapOnly.walkcheckUnverifiable, "a missing trajectory is unverifiable")
    expect(mapOnly.walkcheckStale == "trajectory.tum is missing, so this verdict cannot be checked",
           "reason, got \(mapOnly.walkcheckStale ?? "nil")")
    expect(mapOnly.problems.contains(mapOnly.walkcheckStale ?? "-"), "and the reason is listed under Files")
    let rows = TrailReadout.walkcheckRows(mapOnly.walkcheck!, stale: true, unverifiable: true)
    expect(rows.first?.value == "PASS · unverifiable", "the opened readout says unverifiable, got \(rows.first?.value ?? "nil")")
    expect(TrailReadout.verdictTone("PASS", stale: true, unverifiable: true) == .warn, "the opened PASS is not green")
    let note = TrailReadout.walkcheckNote(mapOnly.walkcheck!, stale: mapOnly.walkcheckStale, accuracyLabel: nil, unverifiable: true)
    expect(note.hasPrefix("Unverifiable: trajectory.tum is missing, so this verdict cannot be checked. Re-run walkcheck"),
           "the note says unverifiable, got \(note)")

    let entry = TrailLibrary.scan(root: root.path).first { $0.name == "scored" }
    expect(entry?.hasTrajectory == false && entry?.hasMap == true && entry?.walkcheckStale == true, "the list row is map-only and not current")
    expect(entry?.walkcheckResult == "PASS" && entry?.walkcheckUnverifiable == true, "the list keeps the verdict and marks it unverifiable")
    expect(TrailReadout.verdictText(entry?.walkcheckResult ?? "", stale: entry?.walkcheckStale == true, unverifiable: entry?.walkcheckUnverifiable == true) == "PASS · unverifiable",
           "the list says unverifiable")
    expect(TrailReadout.verdictTone(entry?.walkcheckResult ?? "", stale: entry?.walkcheckStale == true, unverifiable: entry?.walkcheckUnverifiable == true) == .warn,
           "the list is not green either")

    // A newer map does not turn a missing trajectory back into "older than map.pcd".
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: -3600)], ofItemAtPath: walk.path)
    try! fm.setAttributes([.modificationDate: Date()], ofItemAtPath: map.path)
    let olderMap = try! TrailSession.load(directory: dir.path)
    expect(olderMap.walkcheckUnverifiable && olderMap.walkcheckStale == "trajectory.tum is missing, so this verdict cannot be checked",
           "missing trajectory wins over a newer map, got \(olderMap.walkcheckStale ?? "nil")")

    // A trajectory-only PASS stays current. The return verdict does not need a map.
    let only = tempDirectory("trajectory-only-pass")
    try! Data("1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n".utf8).write(to: only.appendingPathComponent("trajectory.tum"))
    try! walkcheckJSON(result: "PASS", returnError: 4).write(to: only.appendingPathComponent("walkcheck.json"))
    let noMap = try! TrailSession.load(directory: only.path)
    expect(noMap.trajectory != nil && noMap.map == nil, "a trajectory-only folder opens")
    expect(noMap.walkcheckStale == nil && !noMap.walkcheckUnverifiable, "a missing map does not make a PASS unverifiable")
    expect(TrailReadout.verdictTone(noMap.walkcheck?.result ?? "", stale: false, unverifiable: noMap.walkcheckUnverifiable) == .good,
           "a trajectory-only PASS stays green")
    let onlyEntry = TrailLibrary.scan(root: only.deletingLastPathComponent().path).first { $0.name == "trajectory-only-pass" }
    expect(onlyEntry?.walkcheckResult == "PASS" && onlyEntry?.walkcheckUnverifiable == false && onlyEntry?.walkcheckStale == false,
           "the list agrees, got stale=\(onlyEntry?.walkcheckStale == true) unverifiable=\(onlyEntry?.walkcheckUnverifiable == true)")

    try? fm.removeItem(at: root)
    try? fm.removeItem(at: only)
}

/// A report newer than an unusable trajectory must not look current in either display.
private func expectUnusableTrajectory(_ name: String, data: Data, reason: String, unreadable: Bool = false) {
    let fm = FileManager.default
    let root = tempDirectory("unusable-\(name)")
    let dir = root.appendingPathComponent("fresh-walk")
    try! fm.createDirectory(at: dir, withIntermediateDirectories: true)
    let tum = dir.appendingPathComponent("trajectory.tum")
    let walk = dir.appendingPathComponent("walkcheck.json")
    defer {
        try? fm.setAttributes([.posixPermissions: 0o600], ofItemAtPath: tum.path)
        try? fm.removeItem(at: root)
    }
    try! data.write(to: tum)
    try! TrailFixture.binaryPCD(TrailFixture.mapPoints()).write(to: dir.appendingPathComponent("map.pcd"))
    try! walkcheckJSON(result: "PASS", returnError: 4).write(to: walk)
    let scoredAt = Date(timeIntervalSince1970: 1_800_000_000)
    try! fm.setAttributes([.modificationDate: scoredAt], ofItemAtPath: walk.path)
    try! fm.setAttributes([.modificationDate: scoredAt.addingTimeInterval(-60)],
                          ofItemAtPath: dir.appendingPathComponent("map.pcd").path)
    if unreadable {
        try! fm.setAttributes([.posixPermissions: 0o000], ofItemAtPath: tum.path)
        expect(fm.contents(atPath: tum.path) == nil, "\(name): fixture must actually deny file reads")
    }
    let expectedReason = "trajectory.tum is \(reason), so this verdict cannot be checked"
    // Check the reported older-file hole and that unusable takes priority over merely stale.
    for offset: TimeInterval in [-30, 30] {
        try! fm.setAttributes([.modificationDate: scoredAt.addingTimeInterval(offset)], ofItemAtPath: tum.path)
        let inputs = TrailLibrary.VerdictInputs.read(directory: dir.path)
        expect(inputs.trajectory == scoredAt.addingTimeInterval(offset) && inputs.walkcheck == scoredAt,
               "\(name): fixture has the intended input/report ages")
        let session = try! TrailSession.load(directory: dir.path)
        expect(session.trajectory == nil && session.map != nil, "\(name): opens as map-only")
        expect(session.walkcheck?.result == "PASS" && session.walkcheckUnverifiable,
               "\(name): keeps the reported verdict but marks it unverifiable")
        expect(session.walkcheckStale == expectedReason && session.problems.contains(expectedReason),
               "\(name): explains the unusable trajectory under Files")
        let rows = TrailReadout.walkcheckRows(session.walkcheck!, stale: session.walkcheckStale != nil,
                                             unverifiable: session.walkcheckUnverifiable)
        expect(rows.first?.value == "PASS · unverifiable", "\(name): opened verdict is qualified")
        expect(TrailReadout.verdictTone(session.walkcheck!.result, stale: session.walkcheckStale != nil,
                                       unverifiable: session.walkcheckUnverifiable) == .warn,
               "\(name): opened verdict is amber")
        let note = TrailReadout.walkcheckNote(session.walkcheck!, stale: session.walkcheckStale,
                                              accuracyLabel: session.accuracyLabel,
                                              unverifiable: session.walkcheckUnverifiable)
        expect(note.hasPrefix("Unverifiable: \(expectedReason). Re-run walkcheck"),
               "\(name): readout explains why verification failed")
        guard let entry = TrailLibrary.scan(root: root.path).first else {
            return expect(false, "\(name): session must remain listed")
        }
        expect(entry.walkcheckResult == "PASS" && entry.walkcheckUnverifiable && entry.walkcheckStale,
               "\(name): list also marks the verdict unverifiable")
        expect(TrailReadout.verdictText(entry.walkcheckResult!, stale: entry.walkcheckStale,
                                       unverifiable: entry.walkcheckUnverifiable) == "PASS · unverifiable",
               "\(name): list verdict is qualified")
        expect(TrailReadout.verdictTone(entry.walkcheckResult!, stale: entry.walkcheckStale,
                                       unverifiable: entry.walkcheckUnverifiable) == .warn,
               "\(name): list verdict is amber")
    }

    // Repair using a separate synthetic walk, then supply a newer report.
    try! fm.setAttributes([.posixPermissions: 0o600], ofItemAtPath: tum.path)
    try! Data("10 3 2 0 0 0 0 1\n11 4 2 0 0 0 0 1\n12 3.04 2 0 0 0 0 1\n".utf8).write(to: tum)
    try! fm.setAttributes([.modificationDate: scoredAt.addingTimeInterval(60)], ofItemAtPath: tum.path)
    try! walkcheckJSON(result: "PASS", returnError: 4).write(to: walk)
    try! fm.setAttributes([.modificationDate: scoredAt.addingTimeInterval(90)], ofItemAtPath: walk.path)
    let repaired = try! TrailSession.load(directory: dir.path)
    expect(repaired.trajectory?.poses.count == 3 && repaired.walkcheckStale == nil && !repaired.walkcheckUnverifiable,
           "\(name): a usable trajectory and newer report restore current status")
    let repairedEntry = TrailLibrary.scan(root: root.path).first
    expect(repairedEntry?.walkcheckResult == "PASS" && repairedEntry?.walkcheckStale == false
           && repairedEntry?.walkcheckUnverifiable == false, "\(name): repaired list agrees")
}

func testTrailEmptyTrajectoryIsUnverifiable() {
    expectUnusableTrajectory("empty", data: Data(), reason: "empty")
    expectUnusableTrajectory("comments-only", data: Data("# no poses\n \n".utf8), reason: "empty")
}

func testTrailMalformedTrajectoryIsUnverifiable() {
    expectUnusableTrajectory("malformed", data: Data("broken TUM\n".utf8), reason: "malformed")
    expectUnusableTrajectory("broken-tail", data: Data("10 0 0 0 0 0 0 1\n11 1 0\n".utf8), reason: "malformed")
}

func testTrailUnreadableTrajectoryIsUnverifiable() {
    expectUnusableTrajectory("permissions", data: Data("10 0 0 0 0 0 0 1\n".utf8),
                             reason: "unreadable", unreadable: true)
    expectUnusableTrajectory("non-utf8", data: Data([0xff, 0xfe]), reason: "unreadable")
}

/// Rescan must not leave the opened readout on a green PASS after the files change.
func testTrailRescanReloadsOpenedVerdict() {
    let fm = FileManager.default
    let root = tempDirectory("rescan-pass")
    let dir = root.appendingPathComponent("opened")
    try! fm.createDirectory(at: dir, withIntermediateDirectories: true)
    let tum = dir.appendingPathComponent("trajectory.tum")
    let map = dir.appendingPathComponent("map.pcd")
    let walk = dir.appendingPathComponent("walkcheck.json")
    try! Data("1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n".utf8).write(to: tum)
    try! Data("x".utf8).write(to: map)
    try! walkcheckJSON(result: "PASS", returnError: 4).write(to: walk)

    let cached = try! TrailSession.load(directory: dir.path)
    expect(cached.walkcheck?.result == "PASS" && cached.walkcheckStale == nil && !cached.walkcheckUnverifiable,
           "the opened session starts as a current PASS")
    expect(TrailReadout.verdictTone("PASS", stale: false, unverifiable: false) == .good, "and the readout is green")
    let inputs = TrailLibrary.VerdictInputs.read(directory: dir.path)
    expect(!TrailLibrary.rescanShouldReload(opened: inputs, now: TrailLibrary.VerdictInputs.read(directory: dir.path)),
           "rescan leaves the opened session alone when nothing changed")
    expect(!TrailLibrary.rescanShouldReload(opened: nil, now: inputs), "rescan does not reload when no session is open")

    // Replace the trajectory. The cached value stays the old PASS; a reload must not.
    try! Data("5 0 0 0 0 0 0 1\n6 0 2 0 0 0 0 1\n".utf8).write(to: tum)
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: 30)], ofItemAtPath: tum.path)
    let replaced = TrailLibrary.VerdictInputs.read(directory: dir.path)
    expect(TrailLibrary.rescanShouldReload(opened: inputs, now: replaced),
           "replacing the trajectory makes rescan reload the opened session")
    expect(cached.walkcheck?.result == "PASS" && cached.walkcheckStale == nil, "the cached session is still the old green PASS")
    let afterTrajectory = try! TrailSession.load(directory: dir.path)
    expect(afterTrajectory.walkcheck?.result == "PASS" && afterTrajectory.walkcheckStale != nil && !afterTrajectory.walkcheckUnverifiable,
           "the reload is stale, got \(afterTrajectory.walkcheckStale ?? "nil")")
    expect(TrailReadout.verdictText("PASS", stale: true) == "PASS · stale", "it no longer reads as a plain PASS")
    expect(TrailReadout.verdictTone("PASS", stale: true, unverifiable: false) == .warn, "and it is not green")
    let listedStale = TrailLibrary.scan(root: root.path).first { $0.name == "opened" }
    expect(listedStale?.walkcheckResult == "PASS" && listedStale?.walkcheckStale == true && listedStale?.walkcheckUnverifiable == false,
           "the list updates to a stale PASS too")

    // A new FAIL report, newer than the trajectory, replaces the cached PASS.
    try! walkcheckJSON(result: "FAIL", returnError: 50).write(to: walk)
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: 60)], ofItemAtPath: walk.path)
    let failedInputs = TrailLibrary.VerdictInputs.read(directory: dir.path)
    expect(TrailLibrary.rescanShouldReload(opened: replaced, now: failedInputs),
           "writing a FAIL report makes rescan reload the opened session")
    let afterFail = try! TrailSession.load(directory: dir.path)
    expect(afterFail.walkcheck?.result == "FAIL" && afterFail.walkcheckStale == nil && !afterFail.walkcheckUnverifiable,
           "the reload reads the FAIL, got \(afterFail.walkcheck?.result ?? "nil") stale=\(afterFail.walkcheckStale ?? "nil")")
    expect(TrailReadout.verdictTone("FAIL", stale: false, unverifiable: false) == .bad, "a current FAIL is red")
    expect(TrailReadout.verdictTone(afterFail.walkcheck?.result ?? "", stale: afterFail.walkcheckStale != nil, unverifiable: afterFail.walkcheckUnverifiable) != .good,
           "the opened verdict is no longer a green PASS")
    let listedFail = TrailLibrary.scan(root: root.path).first { $0.name == "opened" }
    expect(listedFail?.walkcheckResult == "FAIL" && listedFail?.walkcheckStale == false, "the list updates to the FAIL")

    try? fm.removeItem(at: root)
}

// MARK: - Walk-day result (walk_result.json is the verdict, walkcheck.json an input)

/// The shape `python3 -m spatio.walkday.result` writes for walk_result.json, trimmed.
private func walkResultJSON(runID: String, status: String, d3: Double?, reasons: [String] = []) -> Data {
    let quoted = reasons.map { "\"\($0)\"" }.joined(separator: ", ")
    let number = { (v: Double?) in v.map { "\($0)" } ?? "null" }
    return Data("""
    {"schema": "spatio.walk_result/1", "run_id": "\(runID)", "status": "\(status)", "reasons": [\(quoted)],
     "physical_accuracy_evaluated": false, "accuracy_label": "replay demonstrated; physical accuracy not evaluated",
     "return_error": {"d3_m": \(number(d3)), "horizontal_m": \(number(d3.map { $0 * 0.8 })), "vertical_m": \(number(d3.map { $0 * 0.6 })),
                      "heading_deg": 0.8, "threshold_m": 0.1, "start_window_s": 5.0, "end_window_s": 5.0},
     "walls": {"p95_point_to_plane_m": 0.031, "doubled_wall_suspected": false, "gate_m": 0.05, "gate_state": "approved"},
     "physical_reference": null}
    """.utf8)
}

/// walkday.json (spatio.walkday.report/1), trimmed. `blockers` and `placement` are raw JSON.
private func walkdayReportJSON(runID: String, status: String, walkcheck: String = "PASS", blockers: String = "[]",
                               placement: String = "null", synthetic: Bool = false,
                               stoppedAt: String? = nil, reasons: [String] = []) -> Data {
    let quoted = reasons.map { "\"\($0)\"" }.joined(separator: ", ")
    let stopped = stoppedAt.map { #", "stopped_at": "\#($0)""# } ?? ""
    return Data("""
    {"schema": "spatio.walkday.report/1", "run_id": "\(runID)", "status": "\(status)", "reasons": [\(quoted)],
     "synthetic": \(synthetic), "walkcheck_result": "\(walkcheck)", "placement": \(placement),
     "acceptance_blockers": \(blockers), "input_gate_blockers": []\(stopped)}
    """.utf8)
}

/// A walk-day run folder: trajectory, map and a current walkcheck PASS written an hour ago,
/// then the result files now, in the order process.sh writes them.
private func makeWalkdayRun(_ root: URL, _ name: String, result: Data?, report: Data?) -> URL {
    let fm = FileManager.default
    let dir = root.appendingPathComponent(name)
    try! fm.createDirectory(at: dir.appendingPathComponent("logs"), withIntermediateDirectories: true)
    try! Data("1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n3 0.02 0 0 0 0 0 1\n".utf8).write(to: dir.appendingPathComponent("trajectory.tum"))
    try! Data("x".utf8).write(to: dir.appendingPathComponent("map.pcd"))
    try! Data("STATIONARY_WINDOW_S=5\n".utf8).write(to: dir.appendingPathComponent("logs/walkday.conf"))
    try! walkcheckJSON(result: "PASS", returnError: 4).write(to: dir.appendingPathComponent("walkcheck.json"))
    let earlier = Date(timeIntervalSinceNow: -3600)
    for file in ["trajectory.tum", "map.pcd", "walkcheck.json"] {
        try! fm.setAttributes([.modificationDate: earlier], ofItemAtPath: dir.appendingPathComponent(file).path)
    }
    if let result { try! result.write(to: dir.appendingPathComponent("walk_result.json")) }
    if let report { try! report.write(to: dir.appendingPathComponent("walkday.json")) }
    return dir
}

/// The walk-day acceptance case: walkcheck passes, the final result is NOT_EVALUABLE
/// (here, a blocked input gate), and Trail shows NOT_EVALUABLE, not PASS, in both places.
func testTrailWalkdayResultOverridesWalkcheckPASS() {
    let root = tempDirectory("walkday-not-evaluable")
    let reasons = ["input gate: B2 CLOCK_RESET: lidar stamps go backward"]
    let dir = makeWalkdayRun(root, "walk-oct21",
                             result: walkResultJSON(runID: "walk-oct21", status: "NOT_EVALUABLE", d3: nil, reasons: reasons),
                             report: walkdayReportJSON(runID: "walk-oct21", status: "NOT_EVALUABLE", reasons: reasons))

    let session = try! TrailSession.load(directory: dir.path)
    expect(session.walkcheck?.result == "PASS" && session.walkcheckStale == nil,
           "the walkcheck input is a current PASS on its own")
    guard let final = session.walkday else { expect(false, "a walk-day folder reads walk_result.json"); return }
    expect(final.status == "NOT_EVALUABLE" && final.hasResult, "the opened verdict is walk_result.json's NOT_EVALUABLE, got \(final.status)")
    expect(final.reasons == reasons, "with its reasons, got \(final.reasons)")
    expect(TrailReadout.walkdayText(final) == "NOT_EVALUABLE", "the readout shows NOT_EVALUABLE, got \(TrailReadout.walkdayText(final))")
    expect(TrailReadout.walkdayTone(final) == .warn, "and is not green")
    let rows = Dictionary(uniqueKeysWithValues: TrailReadout.walkdayRows(final).map { ($0.label, $0.value) })
    expect(rows["Verdict"] == "NOT_EVALUABLE", "verdict row, got \(rows["Verdict"] ?? "nil")")
    expect(rows["Return error"] == "withheld (not a measurement)",
           "the withheld return error is not replaced by the walkcheck's 4 cm, got \(rows["Return error"] ?? "nil")")
    expect(rows["Walk check (input)"] == "PASS", "the walkcheck PASS is shown as an input")
    let note = TrailReadout.walkdayNote(final, accuracyLabel: session.accuracyLabel)
    expect(note.contains("walkcheck.json is one of its inputs, not the verdict") && note.contains("CLOCK_RESET"),
           "the note names the source and the reason, got \(note)")
    let inputNote = TrailReadout.walkcheckInputNote(session.walkcheck!, walkday: final)
    expect(inputNote.contains("not the verdict") && inputNote.contains("for diagnosis only; it is not a valid measurement"),
           "the walkcheck block says its 4 cm is not a measurement here, got \(inputNote)")

    let entry = TrailLibrary.scan(root: root.path).first { $0.name == "walk-oct21" }
    expect(entry?.walkcheckResult == "PASS", "the list still reads the walkcheck")
    expect(entry?.walkday?.status == "NOT_EVALUABLE", "but carries the walk-day verdict, got \(entry?.walkday?.status ?? "nil")")
    expect(entry?.walkday.map(TrailReadout.walkdayText) == "NOT_EVALUABLE" && entry?.walkday.map(TrailReadout.walkdayTone) == .warn,
           "and the list shows NOT_EVALUABLE, not green")

    // The other way round: walkcheck FAIL, final NOT_EVALUABLE (as in m1-aug08-mid360-final).
    try! walkcheckJSON(result: "FAIL", returnError: 30).write(to: dir.appendingPathComponent("walkcheck.json"))
    try! FileManager.default.setAttributes([.modificationDate: Date(timeIntervalSinceNow: -3600)],
                                           ofItemAtPath: dir.appendingPathComponent("walkcheck.json").path)
    let failed = try! TrailSession.load(directory: dir.path)
    expect(failed.walkday?.status == "NOT_EVALUABLE" && failed.walkday.map(TrailReadout.walkdayTone) == .warn,
           "a walkcheck FAIL under a NOT_EVALUABLE result shows NOT_EVALUABLE")
    try? FileManager.default.removeItem(at: root)
}

/// Only a clean, current PASS with its report agreeing is green. Each kind of missing or
/// stale evidence keeps a walk_result.json PASS from showing acceptance.
func testTrailWalkdayPASSNeedsEvidence() {
    let fm = FileManager.default
    let root = tempDirectory("walkday-pass")
    func run(_ name: String, result: Data?, report: Data?) -> URL {
        makeWalkdayRun(root, name, result: result, report: report)
    }
    let pass = { (name: String) in walkResultJSON(runID: name, status: "PASS", d3: 0.04) }
    let agree = { (name: String) in walkdayReportJSON(runID: name, status: "PASS") }

    let clean = run("clean", result: pass("clean"), report: agree("clean"))
    guard let good = (try? TrailSession.load(directory: clean.path))?.walkday else {
        expect(false, "a clean PASS folder has a walk-day result"); return
    }
    expect(good.isCleanPASS && good.concerns.isEmpty, "a clean PASS has no concerns, got \(good.concerns)")
    expect(TrailReadout.walkdayText(good) == "PASS" && TrailReadout.walkdayTone(good) == .good, "and is green")
    let rows = Dictionary(uniqueKeysWithValues: TrailReadout.walkdayRows(good).map { ($0.label, $0.value) })
    expect(rows["Return error"] == "4.0 cm · need < 10.0 cm" && rows["  horizontal"] == "3.2 cm",
           "the return error is walk_result.json's, got \(rows["Return error"] ?? "nil")")
    expect(rows["Blockers"] == "none" && rows["Placement"] == "none recorded", "blockers and placement rows")
    expect(TrailLibrary.scan(root: root.path).first { $0.name == "clean" }?.walkday?.isCleanPASS == true, "the list agrees")

    func expectNotGreen(_ dir: URL, _ needle: String, _ label: String) {
        guard let w = (try? TrailSession.load(directory: dir.path))?.walkday else {
            expect(false, "\(label): no walk-day result"); return
        }
        expect(!w.isCleanPASS && TrailReadout.walkdayTone(w) != .good, "\(label): not green")
        expect(w.concerns.contains { $0.contains(needle) }, "\(label): concerns mention '\(needle)', got \(w.concerns)")
        let entry = TrailLibrary.scan(root: root.path).first { $0.directory == dir.path }
        expect(entry?.walkday != nil && entry?.walkday.map(TrailReadout.walkdayTone) != .good, "\(label): the list is not green either")
    }

    let stale = run("stale", result: pass("stale"), report: agree("stale"))
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: 60)],
                          ofItemAtPath: stale.appendingPathComponent("walkcheck.json").path)
    expectNotGreen(stale, "walk_result.json is older than walkcheck.json", "a walkcheck re-run after the result")
    expect(TrailWalkday.read(directory: stale.path).map(TrailReadout.walkdayText) == "PASS · stale", "and it reads stale")
    let newerTrajectory = run("newer-trajectory", result: pass("newer-trajectory"), report: agree("newer-trajectory"))
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: 60)],
                          ofItemAtPath: newerTrajectory.appendingPathComponent("trajectory.tum").path)
    expectNotGreen(newerTrajectory, "older than trajectory.tum", "a trajectory replaced after the result")

    expectNotGreen(run("no-report", result: pass("no-report"), report: nil), "walkday.json is missing", "no walkday.json")
    expectNotGreen(run("disagree", result: pass("disagree"), report: walkdayReportJSON(runID: "disagree", status: "FAIL")),
                   "walkday.json says FAIL", "a report that disagrees")
    expectNotGreen(run("null-blockers", result: pass("null-blockers"),
                       report: walkdayReportJSON(runID: "null-blockers", status: "PASS", blockers: "null")),
                   "acceptance blockers are not recorded", "null blockers (unknown is not none)")
    expectNotGreen(run("blocked", result: pass("blocked"),
                       report: walkdayReportJSON(runID: "blocked", status: "PASS", blockers: #"["origin is auto_suggested"]"#)),
                   "acceptance blocked: origin is auto_suggested", "a listed blocker")
    expectNotGreen(run("placement", result: pass("placement"),
                       report: walkdayReportJSON(runID: "placement", status: "PASS",
                                                 placement: #"{"within_tolerance": false, "problems": ["end_placement_residual_m not measured"], "applied": true}"#)),
                   "placement not within tolerance", "a placement outside tolerance")
    expectNotGreen(run("synthetic", result: pass("synthetic"), report: walkdayReportJSON(runID: "synthetic", status: "PASS", synthetic: true)),
                   "SYNTHETIC", "a synthetic recording")
    expectNotGreen(run("copied", result: pass("elsewhere"), report: agree("elsewhere")), "is for run elsewhere", "a result copied from another run")
    expectNotGreen(run("over", result: walkResultJSON(runID: "over", status: "PASS", d3: 0.12), report: agree("over")),
                   "is not below threshold_m", "a PASS over its threshold")
    expectNotGreen(run("unmeasured", result: walkResultJSON(runID: "unmeasured", status: "PASS", d3: nil), report: agree("unmeasured")),
                   "d3_m is not a number", "a PASS without a return error")

    let junk = run("junk", result: Data("{not json".utf8), report: agree("junk"))
    expectNotGreen(junk, "walk_result.json is not a JSON object", "malformed walk_result.json")
    expect(TrailWalkday.read(directory: junk.path)?.status == "NOT_EVALUABLE", "an unreadable result is NOT_EVALUABLE")
    let foreign = run("foreign", result: Data(#"{"schema": "spatio.walk_result/2", "run_id": "foreign", "status": "PASS"}"#.utf8),
                      report: agree("foreign"))
    expectNotGreen(foreign, "schema is not spatio.walk_result/1", "a foreign schema")
    let hyphen = run("hyphen", result: Data(#"{"schema": "spatio.walk_result/1", "run_id": "hyphen", "status": "NOT-EVALUABLE"}"#.utf8),
                     report: nil)
    expect(TrailWalkday.read(directory: hyphen.path)?.status == "NOT_EVALUABLE" && TrailWalkday.read(directory: hyphen.path)?.hasResult == false,
           "walkcheck's hyphen spelling is not a walk_result status")

    try? fm.removeItem(at: root)
}

/// A walk-day folder without walk_result.json never falls back to the walkcheck verdict.
func testTrailWalkdayWithoutResultIsNotEvaluable() {
    let fm = FileManager.default
    let root = tempDirectory("walkday-no-result")

    // Killed between the walk check and the result: stage files and a walkcheck PASS only.
    let killed = makeWalkdayRun(root, "killed", result: nil, report: nil)
    try! Data(#"{"schema": "spatio.walkday.gate/1", "evaluable": true, "blocking": []}"#.utf8)
        .write(to: killed.appendingPathComponent("logs/input_gate.json"))
    let k = try! TrailSession.load(directory: killed.path)
    expect(k.walkcheck?.result == "PASS", "the walkcheck PASS is read")
    expect(k.walkday?.status == "NOT_EVALUABLE" && k.walkday?.hasResult == false, "but the verdict is NOT_EVALUABLE")
    expect(k.walkday?.concerns == ["no walk_result.json: the run has not been scored"], "and says why, got \(k.walkday?.concerns ?? [])")
    expect(k.walkday.map(TrailReadout.walkdayTone) == .warn, "not green")
    expect(k.walkday.map(TrailReadout.walkdayRows)?.first { $0.label == "Return error" }?.value == "not scored", "no return error shown")
    expect(TrailLibrary.scan(root: root.path).first { $0.name == "killed" }?.walkday?.status == "NOT_EVALUABLE",
           "the list shows NOT_EVALUABLE too")

    // A rejected re-score: the old result moved aside, walkday.json says why.
    let retired = makeWalkdayRun(root, "retired", result: nil,
                                 report: walkdayReportJSON(runID: "retired", status: "NOT_EVALUABLE", stoppedAt: "result",
                                                           reasons: ["stopped at result: rejected, no walk_result.json: wall gate disagrees"]))
    try! walkResultJSON(runID: "retired", status: "PASS", d3: 0.04).write(to: retired.appendingPathComponent("walk_result.json.retired"))
    let r = TrailWalkday.read(directory: retired.path)
    expect(r?.status == "NOT_EVALUABLE" && r?.stoppedAt == "result", "a retired PASS is not the verdict, got \(r?.status ?? "nil")")
    expect(r?.concerns.first?.contains("retired") == true && r?.reasons.first?.contains("rejected") == true, "the reason is the report's")

    // Stopped at the input gate before replay: no trajectory or map, still listed.
    let stopped = root.appendingPathComponent("stopped-gate")
    try! fm.createDirectory(at: stopped.appendingPathComponent("logs"), withIntermediateDirectories: true)
    try! walkdayReportJSON(runID: "stopped-gate", status: "NOT_EVALUABLE", walkcheck: "", stoppedAt: "input-gate",
                           reasons: ["stopped at input-gate: B2 CLOCK_RESET"]).write(to: stopped.appendingPathComponent("walkday.json"))
    let s = TrailLibrary.scan(root: root.path).first { $0.name == "stopped-gate" }
    expect(s != nil && s?.hasTrajectory == false && s?.hasMap == false, "a stopped walk-day run is listed")
    expect(s?.walkday?.status == "NOT_EVALUABLE" && s?.walkday?.stoppedAt == "input-gate", "with its NOT_EVALUABLE")
    do {
        _ = try TrailSession.load(directory: stopped.path)
        expect(false, "a stopped run has nothing to draw")
    } catch {
        expect("\(error)".contains("Walk-day result: NOT_EVALUABLE (stopped at input-gate: B2 CLOCK_RESET)"),
               "the failure names the walk-day verdict, got \(error)")
    }

    // A replay folder under sessions/ (logs/ without walk-day files) keeps the walkcheck verdict.
    let legacy = root.appendingPathComponent("replay")
    try! fm.createDirectory(at: legacy.appendingPathComponent("logs"), withIntermediateDirectories: true)
    try! Data("x".utf8).write(to: legacy.appendingPathComponent("logs/runner.log"))
    try! Data("1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n".utf8).write(to: legacy.appendingPathComponent("trajectory.tum"))
    try! walkcheckJSON(result: "PASS", returnError: 4).write(to: legacy.appendingPathComponent("walkcheck.json"))
    expect(!TrailWalkday.isWalkdayFolder(legacy.path) && (try? TrailSession.load(directory: legacy.path))?.walkday == nil,
           "a non-walk-day folder has no walk-day verdict")
    expect(TrailLibrary.scan(root: root.path).first { $0.name == "replay" }?.walkday == nil, "and the list shows its walkcheck")
    expect(!TrailWalkday.isWalkdayFolder(TrailFixture.directory), "the synthetic-loop fixture is not a walk-day folder")

    try? fm.removeItem(at: root)
}

/// The tab lists walk-day runs and earlier replays, and reloads when the result changes.
func testTrailWalkdayRootsAndRescan() {
    let fm = FileManager.default
    expect(TrailLibrary.walkdayRoot == NSHomeDirectory() + "/Developer/spatio-data/walkday", "process.sh's default root")
    expect(TrailLibrary.defaultRoots == [TrailLibrary.walkdayRoot, TrailLibrary.defaultRoot], "walk-day runs, then sessions")

    let base = tempDirectory("walkday-roots")
    let walkday = base.appendingPathComponent("walkday")
    let sessions = base.appendingPathComponent("sessions")
    let run = makeWalkdayRun(walkday, "walk", result: walkResultJSON(runID: "walk", status: "PASS", d3: 0.04),
                             report: walkdayReportJSON(runID: "walk", status: "PASS"))
    let replay = sessions.appendingPathComponent("replay")
    try! fm.createDirectory(at: replay, withIntermediateDirectories: true)
    try! Data("1 0 0 0 0 0 0 1\n".utf8).write(to: replay.appendingPathComponent("trajectory.tum"))
    let listed = TrailLibrary.scan(roots: [walkday.path, sessions.path, base.appendingPathComponent("absent").path])
    expect(Set(listed.map(\.name)) == ["walk", "replay"], "both roots are listed, got \(listed.map(\.name))")
    expect(TrailLibrary.pickerStart(saved: nil, roots: [base.appendingPathComponent("absent").path, sessions.path]) == sessions.path,
           "the picker starts in the first root that exists")

    let opened = TrailLibrary.VerdictInputs.read(directory: run.path)
    expect(opened.walkResult != nil && opened.walkdayReport != nil, "the result files are verdict inputs")
    let result = run.appendingPathComponent("walk_result.json")
    try! walkResultJSON(runID: "walk", status: "NOT_EVALUABLE", d3: nil, reasons: ["re-scored"]).write(to: result)
    try! fm.setAttributes([.modificationDate: Date(timeIntervalSinceNow: 30)], ofItemAtPath: result.path)
    expect(TrailLibrary.rescanShouldReload(opened: opened, now: TrailLibrary.VerdictInputs.read(directory: run.path)),
           "a new walk_result.json reloads the opened session")
    let rewritten = TrailLibrary.VerdictInputs.read(directory: run.path)
    try! fm.moveItem(at: result, to: run.appendingPathComponent("walk_result.json.retired"))
    expect(TrailLibrary.rescanShouldReload(opened: rewritten, now: TrailLibrary.VerdictInputs.read(directory: run.path)),
           "a retired walk_result.json reloads it too")
    expect(TrailWalkday.read(directory: run.path)?.status == "NOT_EVALUABLE", "and the reload is NOT_EVALUABLE")

    try? fm.removeItem(at: base)
}

func runTrailTests() {
    testTrailTUMFixtureNumbers()
    print("  trail: TUM fixture — 661 poses, 66 s, 11.98 m, start→end exactly 4.0 cm, epoch stamps keep 0.1 s"); passed()
    testTrailTUMAcceptsCommentsAndNormalises()
    print("  trail: TUM comments, blank lines, tabs; near-unit quaternions normalised"); passed()
    testTrailTUMRejectsBrokenLines()
    print("  trail: TUM backward/repeated stamps, short lines, NaN, non-unit quaternions fail with the line"); passed()
    testTrailSampleInterpolatesAndClamps()
    print("  trail: playhead interpolates position, travelled and heading; clamps at both ends"); passed()
    testTrailClock()
    print("  trail: clock play/pause/seek/speed, stops at the end, replays from the start"); passed()
    testTrailPCDBinaryAndASCIIMatch()
    print("  trail: map.pcd FAST-LIO binary and ascii read the same points"); passed()
    testTrailPCDLayouts()
    print("  trail: map.pcd offsets from SIZE×COUNT, F8, NaN dropped, even 1-in-N decimation"); passed()
    testTrailPCDRejects()
    print("  trail: map.pcd truncated, compressed, missing DATA/z, line-count mismatch fail clearly"); passed()
    testTrailBoundsRejectAbsurdValues()
    print("  trail: one bounds rule — TUM |t| ≤ year 2100 s and |xyz| ≤ 10 km or the line fails; PCD drops and counts"); passed()
    testTrailRasterLayers()
    print("  trail: top-down raster — wall band at cart height, open floor faint, trail always inside"); passed()
    testTrailRasterMapOnlyAndEmpty()
    print("  trail: map-only band skips floor and ceiling; outliers do not stretch the fit"); passed()
    testTrailSessionLoadsFixture()
    print("  trail: fixture session loads with the contract label"); passed()
    testTrailManifestLabel()
    print("  trail: only physical_accuracy_evaluated JSON true removes the label"); passed()
    testTrailSessionPartialFolders()
    print("  trail: map-only and broken files open with problems listed; empty folders refuse"); passed()
    testTrailLaunchEnvironment()
    print("  trail: LABCOMMAND_TRAIL_SESSION isolates; both modes or empty is a configuration error"); passed()
    testTrailReadout()
    print("  trail: readout rows and units"); passed()
    testTrailFixtureIsCurrent()
    print("  trail: committed synthetic-loop fixture matches the generator"); passed()
    testTrailWalkcheckParse()
    print("  trail: walkcheck.json verdict, return error, proposed wall gate; null, foreign schema and junk handled"); passed()
    testTrailWalkcheckNeverClearsLabel()
    print("  trail: only the session's own manifest true clears the label; a walkcheck PASS does not"); passed()
    testTrailLibraryScanAndReopen()
    print("  trail: sessions root lists newest first, skips empty folders; last folder reopens only if present"); passed()
    testTrailDeletedTrajectoryIsUnverifiable()
    print("  trail: deleting trajectory.tum makes a PASS unverifiable in the session and the list"); passed()
    testTrailEmptyTrajectoryIsUnverifiable()
    print("  trail: empty and comments-only trajectories make PASS unverifiable in both displays"); passed()
    testTrailMalformedTrajectoryIsUnverifiable()
    print("  trail: malformed trajectories, including a broken tail, make PASS unverifiable in both displays"); passed()
    testTrailUnreadableTrajectoryIsUnverifiable()
    print("  trail: permission-denied and non-UTF8 trajectories make PASS unverifiable in both displays"); passed()
    testTrailRescanReloadsOpenedVerdict()
    print("  trail: rescan reloads the opened session when the trajectory or walkcheck changes"); passed()
    testTrailWalkdayResultOverridesWalkcheckPASS()
    print("  trail: walkcheck PASS + walk_result.json NOT_EVALUABLE shows NOT_EVALUABLE in the session and the list"); passed()
    testTrailWalkdayPASSNeedsEvidence()
    print("  trail: a walk_result PASS is green only when current, its report agrees and blockers are recorded as none"); passed()
    testTrailWalkdayWithoutResultIsNotEvaluable()
    print("  trail: a walk-day folder without walk_result.json is NOT_EVALUABLE, never the walkcheck verdict"); passed()
    testTrailWalkdayRootsAndRescan()
    print("  trail: walk-day and sessions roots both listed; rescan reloads when walk_result.json changes"); passed()
    testTrailViewContract()
    print("  trail: view shows label, raw note and walkcheck; Trail is the 4th tab (Room, Settings, Rig, Trail)"); passed()
}
