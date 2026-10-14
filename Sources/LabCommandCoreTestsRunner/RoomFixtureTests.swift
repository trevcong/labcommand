import Foundation
import LabCommandCore

// MARK: - Room Fixture Catalog (PR 0.3)

// Tests/Fixtures/room/synthetic is written by `swift run LabCommandCoreTestsRunner
// generate-room-fixtures`. The tests regenerate every file in memory and require the stored bytes
// to match, so the fixtures can't drift from this catalog.

enum RoomFixtures {
    static let root = "Tests/Fixtures/room"
    static let syntheticDirectory = root + "/synthetic"
    static let frameId = "synthetic_lidar"
    static let rigShift = SIMD3<Double>(0.05, 0, 0)

    struct Scene {
        var name: String
        var extra: [(name: String, box: SyntheticBox)]
        var sensor: SIMD3<Double>
        var seed: UInt64
        var sec: Int32
        var nanosec: UInt32

        var file: String { name + ".bin" }
        var room: SyntheticRoom { SyntheticRoom(extra: extra.map(\.box), sensor: sensor) }
    }

    static let personFrames = 5

    static let scenes: [Scene] = {
        let nominal = SyntheticRoom.nominalSensor
        var scenes = [
            Scene(name: "base_s1", extra: [], sensor: nominal, seed: 1, sec: 1_760_400_000, nanosec: 0),
            Scene(name: "base_s2", extra: [], sensor: nominal, seed: 2, sec: 1_760_400_010, nanosec: 0),
            Scene(name: "box_a", extra: [("box_a", SyntheticRoom.boxA)], sensor: nominal, seed: 3, sec: 1_760_400_020, nanosec: 0),
            Scene(name: "box_b", extra: [("box_b", SyntheticRoom.boxB)], sensor: nominal, seed: 4, sec: 1_760_400_030, nanosec: 0),
            // Same seed as base_s1: the 5 cm shift is the only difference.
            Scene(name: "rig_shifted", extra: [], sensor: nominal + rigShift, seed: 1, sec: 1_760_400_040, nanosec: 0),
        ]
        for step in 0..<personFrames {
            // 0.25 s per frame from 1_760_400_050; whole seconds carry into sec so nanosec stays < 1e9.
            let offsetNs = step * 250_000_000
            scenes.append(Scene(
                name: "person_\(step)",
                extra: [("person", SyntheticRoom.person(step: step))],
                sensor: nominal,
                seed: 10 + UInt64(step),
                sec: 1_760_400_050 + Int32(offsetNs / 1_000_000_000),
                nanosec: UInt32(offsetNs % 1_000_000_000)
            ))
        }
        return scenes
    }()

    /// One scene as a PointCloud2 CDR frame: x, y, z, intensity float32 in the sensor frame.
    static func encodedCloud(_ scene: Scene) -> Data {
        let points = SyntheticScanner.fixture.scan(scene.room, seed: scene.seed).map { hit in
            let local = hit.point - scene.sensor
            return (Float(local.x), Float(local.y), Float(local.z), hit.intensity)
        }
        var cloud = cdrXYZICloud(points)
        cloud.frameId = frameId
        cloud.sec = scene.sec
        cloud.nanosec = scene.nanosec
        return cloud.encoded()
    }

    /// A JSON number with at most 6 decimals, so 5.675 prints as 5.675 and not 5.6749999999999998.
    static func number(_ value: Double) -> NSDecimalNumber {
        NSDecimalNumber(string: String(format: "%.6f", value))
    }

    static func json(_ v: SIMD3<Double>) -> [NSDecimalNumber] { [v.x, v.y, v.z].map(number) }

    static func aabb(_ box: SyntheticBox, sensor: SIMD3<Double>) -> [String: Any] {
        ["room_m": json(box.min) + json(box.max), "sensor_frame_m": json(box.min - sensor) + json(box.max - sensor)]
    }

    /// Ground truth per pair; expected `change-detect` outputs are a follow-up (see README).
    static func pairs() -> [[String: Any]] {
        let nominal = SyntheticRoom.nominalSensor
        return [
            ["name": "static", "before": "base_s1.bin", "after": "base_s2.bin",
             "truth": ["kind": "none", "note": "same room, different range-noise seed"]],
            ["name": "box_added", "before": "base_s1.bin", "after": "box_a.bin",
             "truth": ["kind": "APPEARED", "aabb": aabb(SyntheticRoom.boxA, sensor: nominal)]],
            ["name": "box_removed", "before": "box_a.bin", "after": "base_s2.bin",
             "truth": ["kind": "DISAPPEARED", "aabb": aabb(SyntheticRoom.boxA, sensor: nominal)]],
            ["name": "box_moved", "before": "box_a.bin", "after": "box_b.bin",
             "truth": ["kind": "MOVED",
                       "from_aabb": aabb(SyntheticRoom.boxA, sensor: nominal),
                       "to_aabb": aabb(SyntheticRoom.boxB, sensor: nominal),
                       "move_m": number((SyntheticRoom.boxB.center - SyntheticRoom.boxA.center).y)]],
            ["name": "person_walkthrough", "before": "base_s1.bin",
             "after_sequence": (0..<personFrames).map { "person_\($0).bin" },
             "truth": ["kind": "none (transient)",
                       "note": "no CONFIRMED event: the person moves 0.3 m per 0.25 s frame",
                       "person_aabb_per_frame": (0..<personFrames).map { aabb(SyntheticRoom.person(step: $0), sensor: nominal) }]],
            ["name": "rig_shifted", "before": "base_s1.bin", "after": "rig_shifted.bin",
             "truth": ["kind": "RIG_MOVED", "shift_m": json(rigShift),
                       "note": "the whole room moves -5 cm in x in the sensor frame; no change events"]],
        ]
    }

    static func manifest() throws -> Data {
        let scanner = SyntheticScanner.fixture
        let object: [String: Any] = [
            "generator": "swift run LabCommandCoreTestsRunner generate-room-fixtures",
            "format": "sensor_msgs/PointCloud2 CDR little endian; x, y, z, intensity float32; point_step 16; frame_id \(frameId)",
            "frame": "points are in the sensor frame: room-frame hit minus that scene's sensor_m, no rotation",
            "room": [
                "size_m": json(SyntheticRoom.size),
                "furniture_room_m": SyntheticRoom.furniture.map { json($0.min) + json($0.max) },
            ],
            "scanner": [
                "azimuth_step_deg": number(scanner.azimuthStepDeg),
                "elevation_min_deg": number(scanner.elevationMinDeg),
                "elevation_max_deg": number(scanner.elevationMaxDeg),
                "elevation_step_deg": number(scanner.elevationStepDeg),
                "range_noise_m": number(scanner.rangeNoiseM),
                "rays": scanner.azimuthCount * scanner.elevationCount,
            ],
            "scenes": scenes.map { scene -> [String: Any] in
                [
                    "file": scene.file,
                    "seed": scene.seed,
                    "sensor_m": json(scene.sensor),
                    "objects": scene.extra.map(\.name),
                    "points": scanner.azimuthCount * scanner.elevationCount,
                    "stamp": ["sec": scene.sec, "nanosec": scene.nanosec],
                ]
            },
            "pairs": pairs(),
        ]
        return try JSONSerialization.data(withJSONObject: object, options: [.prettyPrinted, .sortedKeys, .withoutEscapingSlashes])
    }
}

/// `generate-room-fixtures [directory]`: writes every synthetic scene and MANIFEST.json.
func generateRoomFixtures(arguments: [String]) {
    let directory = URL(fileURLWithPath: arguments.first ?? RoomFixtures.syntheticDirectory)
    do {
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        for scene in RoomFixtures.scenes {
            let data = RoomFixtures.encodedCloud(scene)
            try data.write(to: directory.appendingPathComponent(scene.file))
            print("wrote \(scene.file) (\(data.count) bytes)")
        }
        try RoomFixtures.manifest().write(to: directory.appendingPathComponent("MANIFEST.json"))
        print("wrote MANIFEST.json")
    } catch {
        fputs("generate-room-fixtures: \(error)\n", stderr)
        exit(1)
    }
}

// MARK: - Fixture Tests

private func readFixture(_ path: String) -> Data {
    guard let data = FileManager.default.contents(atPath: path) else {
        fputs("FAIL: \(path) not found — run tests from repo root\n", stderr)
        exit(1)
    }
    return data
}

/// The header stamp of a PointCloud2 CDR fixture: int32 sec and uint32 nanosec right after the
/// 4-byte encapsulation header, in the byte order it names. PointCloud2CDR folds them into a Date,
/// which hides an out-of-range nanosec, so the tests read the raw fields.
private func expectValidStamp(_ path: String) {
    let bytes = [UInt8](readFixture(path))
    guard bytes.count >= 12, bytes[0] == 0x00, bytes[1] <= 0x01 else {
        expect(false, "room fixture \(path): no CDR header stamp")
        return
    }
    let field = { (offset: Int) -> UInt32 in
        let value = bytes[offset..<offset + 4].reversed().reduce(UInt32(0)) { $0 << 8 | UInt32($1) }
        return bytes[1] == 0x01 ? value : value.byteSwapped
    }
    let nanosec = field(8)
    expect(nanosec < 1_000_000_000, "room fixture \(path): stamp nanosec \(nanosec) is not below 1e9")
}

private func decodeCloud(_ path: String) -> PointCloudFrame {
    do {
        return try PointCloud2CDR.decode(readFixture(path))
    } catch {
        expect(false, "room fixture \(path): decode failed: \(error)")
        return PointCloudFrame()
    }
}

func testRoomSnapshotFixturesDecode() {
    // Real frames copied from lc-tasks/snapshots (~10 s apart, static room). The printed counts
    // are what PointCloud2CDR keeps after dropping non-finite points.
    var counts: [String] = []
    for take in ["a", "b"] {
        for (sensor, frameId) in [("jt128", "hesai_lidar"), ("mid360", "livox_frame")] {
            let path = "\(RoomFixtures.root)/snapshots/\(take)/\(sensor).bin"
            expectValidStamp(path)
            let frame = decodeCloud(path)
            expect(frame.frameId == frameId, "snapshot \(take)/\(sensor): frame_id '\(frame.frameId ?? "nil")'")
            expect(frame.count > 10_000, "snapshot \(take)/\(sensor): only \(frame.count) finite points")
            counts.append("\(take)/\(sensor) \(frame.count)")
        }
    }
    print("  room fixtures: snapshots decode (finite points: \(counts.joined(separator: ", ")))")
}

func testRoomSyntheticFixturesMatchGenerator() {
    for scene in RoomFixtures.scenes {
        let path = "\(RoomFixtures.syntheticDirectory)/\(scene.file)"
        expect(readFixture(path) == RoomFixtures.encodedCloud(scene),
               "synthetic \(scene.file) differs from the generator; rerun generate-room-fixtures")
        expectValidStamp(path)
        let frame = decodeCloud(path)
        expect(frame.count == SyntheticScanner.fixture.azimuthCount * SyntheticScanner.fixture.elevationCount,
               "synthetic \(scene.file): \(frame.count) points")
        expect(frame.frameId == RoomFixtures.frameId, "synthetic \(scene.file): frame_id")
    }
    do {
        expect(readFixture("\(RoomFixtures.syntheticDirectory)/MANIFEST.json") == (try RoomFixtures.manifest()),
               "synthetic MANIFEST.json differs from the generator")
    } catch {
        expect(false, "synthetic manifest: \(error)")
    }
}

/// Points of a fixture inside `box` (room frame, 2 cm margin) and above floor noise, given the
/// scene's sensor origin.
private func pointsInside(_ file: String, _ box: SyntheticBox, sensor: SIMD3<Double>) -> Int {
    decodeCloud("\(RoomFixtures.syntheticDirectory)/\(file)").positions.filter { point in
        let room = SIMD3(Double(point.x), Double(point.y), Double(point.z)) + sensor
        return room.z > 0.03 && box.contains(room, margin: 0.02)
    }.count
}

func testRoomSyntheticScenesCarryTheirChange() {
    let nominal = SyntheticRoom.nominalSensor
    let boxA = SyntheticRoom.boxA
    let boxB = SyntheticRoom.boxB
    expect(pointsInside("base_s1.bin", boxA, sensor: nominal) == 0, "base_s1 has no box A returns")
    expect(pointsInside("base_s2.bin", boxA, sensor: nominal) == 0, "base_s2 has no box A returns")
    expect(pointsInside("box_a.bin", boxA, sensor: nominal) >= 20, "box_a shows box A")
    expect(pointsInside("box_b.bin", boxA, sensor: nominal) == 0, "box_b no longer shows box A")
    expect(pointsInside("box_b.bin", boxB, sensor: nominal) >= 20, "box_b shows box B")
    for step in 0..<RoomFixtures.personFrames {
        let person = SyntheticRoom.person(step: step)
        expect(pointsInside("person_\(step).bin", person, sensor: nominal) >= 50, "person_\(step) shows the person")
        expect(pointsInside("base_s1.bin", person, sensor: nominal) == 0, "base_s1 has no person at step \(step)")
    }

    // The +x wall is 4.0 m from the nominal sensor and 3.95 m from the shifted one (±1 cm noise).
    let farWall = { (file: String) -> Float in
        decodeCloud("\(RoomFixtures.syntheticDirectory)/\(file)").positions.map(\.x).max() ?? 0
    }
    expect(abs(farWall("base_s1.bin") - 4.0) < 0.011, "base_s1: +x wall at 4.0 m")
    expect(abs(farWall("rig_shifted.bin") - 3.95) < 0.011, "rig_shifted: +x wall at 3.95 m")
}

func runRoomFixtureTests() {
    testRoomSnapshotFixturesDecode(); passed()
    testRoomSyntheticFixturesMatchGenerator()
    print("  room fixtures: synthetic scenes and MANIFEST.json match the generator byte for byte"); passed()
    testRoomSyntheticScenesCarryTheirChange()
    print("  room fixtures: box added/removed/moved, walk-through, and 5 cm rig shift are in the clouds"); passed()
}
