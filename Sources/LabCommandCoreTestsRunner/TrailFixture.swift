import Foundation
import LabCommandCore

/// The synthetic session in `fixtures/trail/synthetic-loop`. Numbers are chosen so tests can
/// assert them exactly: a 4 × 2 m loop that ends 3.2 cm / 2.4 cm off the start (4.0 cm),
/// 3 s still at each end, 10 Hz, inside a 7 × 5 m room with a table, a cabinet, and a doorway.
enum TrailFixture {
    static let directory = "fixtures/trail/synthetic-loop"
    static let t0 = 1_792_368_000.0
    static let dt = 0.1
    static let endOffset = (x: 0.032, y: 0.024)
    /// 4 + 2 + 4 + the last leg from (0, 2) to the end offset.
    static var pathLength: Double {
        10 + ((endOffset.x * endOffset.x) + (2 - endOffset.y) * (2 - endOffset.y)).squareRoot()
    }
    static let poseCount = 661
    static let duration = 66.0
    static let floorZ = -0.9
    static let ceilingZ = 1.7

    /// (x, y, steps) waypoints. A zero-length leg is the cart standing still.
    private static var legs: [(x: Double, y: Double, steps: Int)] {
        [(0, 0, 30), (4, 0, 200), (4, 2, 100), (0, 2, 200), (endOffset.x, endOffset.y, 100), (endOffset.x, endOffset.y, 30)]
    }

    struct Pose {
        var t: Double
        var x: Double
        var y: Double
        var yaw: Double
    }

    static func poses() -> [Pose] {
        var out = [Pose(t: t0, x: 0, y: 0, yaw: 0)]
        var x = 0.0, y = 0.0, yaw = 0.0, step = 0
        for leg in legs {
            let dx = leg.x - x, dy = leg.y - y
            if dx != 0 || dy != 0 { yaw = atan2(dy, dx) }
            for k in 1...leg.steps {
                step += 1
                let f = Double(k) / Double(leg.steps)
                out.append(Pose(t: t0 + Double(step) * dt, x: x + dx * f, y: y + dy * f, yaw: yaw))
            }
            x = leg.x
            y = leg.y
        }
        return out
    }

    static func tum() -> String {
        var lines = ["# synthetic loop fixture (generate-trail-fixture). t x y z qx qy qz qw"]
        for p in poses() {
            lines.append(String(format: "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f",
                                p.t, p.x, p.y, 0.0, 0.0, 0.0, sin(p.yaw / 2), cos(p.yaw / 2)))
        }
        return lines.joined(separator: "\n") + "\n"
    }

    /// Deterministic ±1 cm jitter, so walls have thickness like a real map.
    private struct Jitter {
        var state: UInt64 = 0x9E37_79B9_7F4A_7C15
        mutating func next() -> Double {
            state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
            return Double(state >> 11) / Double(1 << 53) * 0.02 - 0.01
        }
    }

    static func mapPoints() -> [Point3D] {
        var raw: [(Double, Double, Double)] = []
        func wall(from a: (Double, Double), to b: (Double, Double), z0: Double, z1: Double, skip: ((Double, Double)) -> Bool = { _ in false }) {
            let length = ((b.0 - a.0) * (b.0 - a.0) + (b.1 - a.1) * (b.1 - a.1)).squareRoot()
            let steps = Int((length / 0.05).rounded())
            for i in 0...steps {
                let f = Double(i) / Double(steps)
                let p = (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f)
                if skip(p) { continue }
                var z = z0
                while z <= z1 + 1e-9 {
                    raw.append((p.0, p.1, z))
                    z += 0.2
                }
            }
        }
        func plane(x0: Double, x1: Double, y0: Double, y1: Double, z: Double, step: Double) {
            var x = x0
            while x <= x1 + 1e-9 {
                var y = y0
                while y <= y1 + 1e-9 {
                    raw.append((x, y, z))
                    y += step
                }
                x += step
            }
        }
        // Room shell: 7 × 5 m, a 0.9 m doorway in the far wall.
        wall(from: (-1.5, -1.5), to: (5.5, -1.5), z0: floorZ, z1: ceilingZ)
        wall(from: (5.5, -1.5), to: (5.5, 3.5), z0: floorZ, z1: ceilingZ)
        wall(from: (5.5, 3.5), to: (-1.5, 3.5), z0: floorZ, z1: ceilingZ) { $0.0 > 3.6 && $0.0 < 4.5 }
        wall(from: (-1.5, 3.5), to: (-1.5, -1.5), z0: floorZ, z1: ceilingZ)
        plane(x0: -1.5, x1: 5.5, y0: -1.5, y1: 3.5, z: floorZ, step: 0.25)
        plane(x0: -1.5, x1: 5.5, y0: -1.5, y1: 3.5, z: ceilingZ, step: 0.5)
        // Table inside the loop: top at 0.75 m above the floor, four legs.
        plane(x0: 1.4, x1: 2.6, y0: 0.6, y1: 1.4, z: floorZ + 0.75, step: 0.1)
        for leg in [(1.45, 0.65), (2.55, 0.65), (1.45, 1.35), (2.55, 1.35)] {
            wall(from: leg, to: (leg.0 + 0.05, leg.1), z0: floorZ, z1: floorZ + 0.7)
        }
        // Cabinet in the near corner, 1.8 m tall.
        wall(from: (4.8, -1.5), to: (4.8, -0.9), z0: floorZ, z1: floorZ + 1.8)
        wall(from: (4.8, -0.9), to: (5.5, -0.9), z0: floorZ, z1: floorZ + 1.8)
        var jitter = Jitter()
        return raw.map { p in
            Point3D(x: Float(p.0 + jitter.next()), y: Float(p.1 + jitter.next()), z: Float(p.2 + jitter.next()))
        }
    }

    static func pcdHeader(fields: [String], count: Int, format: String) -> String {
        let ones = Array(repeating: "1", count: fields.count).joined(separator: " ")
        let fours = Array(repeating: "4", count: fields.count).joined(separator: " ")
        let fs = Array(repeating: "F", count: fields.count).joined(separator: " ")
        return """
        # .PCD v0.7 - Point Cloud Data file format
        VERSION 0.7
        FIELDS \(fields.joined(separator: " "))
        SIZE \(fours)
        TYPE \(fs)
        COUNT \(ones)
        WIDTH \(count)
        HEIGHT 1
        VIEWPOINT 0 0 0 1 0 0 0
        POINTS \(count)
        DATA \(format)

        """
    }

    /// FAST-LIO's saved layout: PointXYZINormal, 8 floats, 32 bytes per point, little-endian.
    static func binaryPCD(_ points: [Point3D]) -> Data {
        let fields = ["x", "y", "z", "intensity", "normal_x", "normal_y", "normal_z", "curvature"]
        var data = Data(pcdHeader(fields: fields, count: points.count, format: "binary").utf8)
        for (i, p) in points.enumerated() {
            for value in [p.x, p.y, p.z, Float(i % 200), 0, 0, 0, 0] {
                withUnsafeBytes(of: value.bitPattern.littleEndian) { data.append(contentsOf: $0) }
            }
        }
        return data
    }

    static func asciiPCD(_ points: [Point3D]) -> Data {
        var text = pcdHeader(fields: ["x", "y", "z", "intensity"], count: points.count, format: "ascii")
        for (i, p) in points.enumerated() {
            // %.9g round-trips a Float exactly.
            text += String(format: "%.9g %.9g %.9g %d\n", p.x, p.y, p.z, i % 200)
        }
        return Data(text.utf8)
    }

    static func manifest() throws -> Data {
        let object: [String: Any] = [
            "run_id": "synthetic-loop",
            "source_recording": "synthetic (generate-trail-fixture); not a recording",
            "sensor": "synthetic",
            "fastlio_revision": NSNull(),
            "config_file": NSNull(),
            "config_sha256": NSNull(),
            "image": NSNull(),
            "created_at": "2026-10-19T00:00:00Z",
            "notes": "Synthetic 4 x 2 m loop for tests and screenshots. Ends 4.0 cm from the start by construction.",
            "physical_accuracy_evaluated": false,
        ]
        var data = try JSONSerialization.data(withJSONObject: object, options: [.prettyPrinted, .sortedKeys, .withoutEscapingSlashes])
        data.append(0x0A)
        return data
    }

    static func files() throws -> [(name: String, data: Data)] {
        [
            (TrailSession.trajectoryFile, Data(tum().utf8)),
            (TrailSession.mapFile, binaryPCD(mapPoints())),
            (TrailSession.manifestFile, try manifest()),
        ]
    }
}

/// `generate-trail-fixture [directory]`: writes the synthetic session.
func generateTrailFixture(arguments: [String]) {
    let directory = URL(fileURLWithPath: arguments.first ?? TrailFixture.directory)
    do {
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        for file in try TrailFixture.files() {
            try file.data.write(to: directory.appendingPathComponent(file.name))
            print("wrote \(file.name) (\(file.data.count) bytes)")
        }
    } catch {
        fputs("generate-trail-fixture: \(error)\n", stderr)
        exit(1)
    }
}
