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
    // A trail whose extent overflows Double still gives a raster.
    let wide = try! TrailTrajectory.parseTUM("0 -1e308 -1e308 0 0 0 0 1\n1 1e308 1e308 0 0 0 0 1\n")
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

/// The app reads the label from Core and the trail tab stays out of the pinned nav order.
func testTrailViewContract() {
    let root = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)
    let view = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/TrailReplayView.swift"), encoding: .utf8)) ?? ""
    let state = (try? String(contentsOf: root.appendingPathComponent("Sources/LabCommand/AppState.swift"), encoding: .utf8)) ?? ""
    expect(!view.isEmpty, "TrailReplayView.swift exists")
    expect(view.contains("accuracyLabel"), "the view shows the session's accuracy label")
    expect(view.contains("TrailReadout.startToEndNote"), "the view says start → end is raw")
    expect(state.contains("case trail"), "Dest has a trail case")
    expect(state.contains("static var navCases: [Dest] { [.liveRoom, .settings, .workspace] }"), "nav order is unchanged")
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
    testTrailViewContract()
    print("  trail: view shows the label and the raw note; nav order unchanged"); passed()
}
