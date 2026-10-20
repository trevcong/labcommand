import Foundation

/// A position in the map frame, metres. Double so a pose read from a file keeps every digit.
public struct TrailPoint: Equatable, Sendable {
    public var x: Double
    public var y: Double
    public var z: Double

    public init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }

    public func distance(to other: TrailPoint) -> Double {
        let dx = other.x - x, dy = other.y - y, dz = other.z - z
        return (dx * dx + dy * dy + dz * dz).squareRoot()
    }

    public func horizontalDistance(to other: TrailPoint) -> Double {
        let dx = other.x - x, dy = other.y - y
        return (dx * dx + dy * dy).squareRoot()
    }
}

/// One TUM line: `t x y z qx qy qz qw`. The quaternion is stored normalised.
public struct TrailPose: Equatable, Sendable {
    /// Seconds. Double: an epoch stamp in Float would lose the milliseconds.
    public var t: Double
    public var position: TrailPoint
    public var qx: Double
    public var qy: Double
    public var qz: Double
    public var qw: Double

    public init(t: Double, position: TrailPoint, qx: Double, qy: Double, qz: Double, qw: Double) {
        self.t = t
        self.position = position
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
    }

    /// Heading about map z, radians, counter-clockwise from +x.
    public var yaw: Double {
        atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    }
}

/// What can be wrong with a session file. Each case names the file.
public enum TrailError: Error, Equatable, Sendable, CustomStringConvertible {
    case tum(line: Int, String)
    case tumEmpty
    case pcd(String)
    case session(String)

    public var description: String {
        switch self {
        case .tum(let line, let text): return "trajectory.tum line \(line): \(text)"
        case .tumEmpty: return "trajectory.tum has no poses"
        case .pcd(let text): return "map.pcd: \(text)"
        case .session(let text): return text
        }
    }
}

/// A parsed trajectory, in file order, with the running path length.
public struct TrailTrajectory: Equatable, Sendable {
    public let poses: [TrailPose]
    /// `cumulative[i]` is the 3D path length from pose 0 to pose i.
    public let cumulative: [Double]

    /// A quaternion this far from unit length is a broken line, not rounding.
    public static let quaternionTolerance = 0.01

    public init(poses: [TrailPose]) {
        self.poses = poses
        var running: [Double] = []
        running.reserveCapacity(poses.count)
        var total = 0.0
        for (i, pose) in poses.enumerated() {
            if i > 0 { total += poses[i - 1].position.distance(to: pose.position) }
            running.append(total)
        }
        cumulative = running
    }

    public var start: TrailPose? { poses.first }
    public var end: TrailPose? { poses.last }
    public var startTime: Double { poses.first?.t ?? 0 }
    public var duration: Double { (poses.last?.t ?? 0) - startTime }
    public var pathLength: Double { cumulative.last ?? 0 }

    /// Parses TUM text. Blank lines and `#` comments are skipped. Poses stay in file order:
    /// a timestamp that repeats or goes backward is an error, never a silent sort.
    public static func parseTUM(_ text: String) throws -> TrailTrajectory {
        var poses: [TrailPose] = []
        var lineNumber = 0
        // `isNewline`, not "\n": Swift reads "\r\n" as one Character.
        for raw in text.split(omittingEmptySubsequences: false, whereSeparator: \.isNewline) {
            lineNumber += 1
            let line = raw.trimmingCharacters(in: .whitespacesAndNewlines)
            if line.isEmpty || line.hasPrefix("#") { continue }
            let fields = line.split(whereSeparator: { $0 == " " || $0 == "\t" || $0 == "," })
            guard fields.count == 8 else {
                throw TrailError.tum(line: lineNumber, "expected 8 values (t x y z qx qy qz qw), got \(fields.count)")
            }
            var values: [Double] = []
            values.reserveCapacity(8)
            for field in fields {
                guard let value = Double(field), value.isFinite else {
                    throw TrailError.tum(line: lineNumber, "'\(field)' is not a finite number")
                }
                values.append(value)
            }
            let (qx, qy, qz, qw) = (values[4], values[5], values[6], values[7])
            let norm = (qx * qx + qy * qy + qz * qz + qw * qw).squareRoot()
            guard abs(norm - 1) <= quaternionTolerance else {
                throw TrailError.tum(line: lineNumber, String(format: "quaternion length %.4f is not 1", norm))
            }
            let t = values[0]
            if let previous = poses.last, t <= previous.t {
                let word = t == previous.t ? "repeats" : "goes backward from"
                throw TrailError.tum(line: lineNumber, String(format: "timestamp %.6f %@ %.6f", t, word, previous.t))
            }
            poses.append(TrailPose(
                t: t,
                position: TrailPoint(x: values[1], y: values[2], z: values[3]),
                qx: qx / norm, qy: qy / norm, qz: qz / norm, qw: qw / norm
            ))
        }
        guard !poses.isEmpty else { throw TrailError.tumEmpty }
        return TrailTrajectory(poses: poses)
    }

    /// Totals for the readout. Start-to-end is the raw first-to-last pose distance:
    /// no alignment and no stationary-window averaging. It is not the scored return error.
    public var summary: TrailSummary {
        guard let first = poses.first, let last = poses.last else {
            return TrailSummary(poseCount: 0, duration: 0, pathLength: 0, startToEnd: 0,
                                startToEndHorizontal: 0, startToEndVertical: 0, largestGap: 0)
        }
        var gap = 0.0
        for i in poses.indices.dropFirst() {
            gap = max(gap, poses[i].t - poses[i - 1].t)
        }
        return TrailSummary(
            poseCount: poses.count,
            duration: duration,
            pathLength: pathLength,
            startToEnd: first.position.distance(to: last.position),
            startToEndHorizontal: first.position.horizontalDistance(to: last.position),
            startToEndVertical: abs(last.position.z - first.position.z),
            largestGap: gap
        )
    }

    /// The cart at `elapsed` seconds after the first pose, clamped to the recording.
    /// Position is linear between the two poses around it; heading takes the short way round.
    public func sample(atElapsed elapsed: Double) -> TrailSample? {
        guard let first = poses.first else { return nil }
        let t = startTime + min(max(elapsed, 0), duration)
        // Last pose with time ≤ t.
        var lo = 0
        var hi = poses.count - 1
        while lo < hi {
            let mid = (lo + hi + 1) / 2
            if poses[mid].t <= t { lo = mid } else { hi = mid - 1 }
        }
        let a = poses[lo]
        guard lo + 1 < poses.count else {
            return TrailSample(elapsed: a.t - first.t, index: lo, fraction: 0, position: a.position,
                               yaw: a.yaw, travelled: cumulative[lo])
        }
        let b = poses[lo + 1]
        let f = (t - a.t) / (b.t - a.t)
        let p = TrailPoint(
            x: a.position.x + (b.position.x - a.position.x) * f,
            y: a.position.y + (b.position.y - a.position.y) * f,
            z: a.position.z + (b.position.z - a.position.z) * f
        )
        var dyaw = b.yaw - a.yaw
        if dyaw > .pi { dyaw -= 2 * .pi }
        if dyaw < -.pi { dyaw += 2 * .pi }
        return TrailSample(
            elapsed: t - first.t,
            index: lo,
            fraction: f,
            position: p,
            yaw: a.yaw + dyaw * f,
            travelled: cumulative[lo] + a.position.distance(to: p)
        )
    }
}

public struct TrailSummary: Equatable, Sendable {
    public var poseCount: Int
    /// Seconds from the first pose to the last.
    public var duration: Double
    /// 3D length of the polyline through every pose, metres.
    public var pathLength: Double
    /// Raw first-to-last pose distance, metres. 3D, horizontal (x/y), and vertical.
    public var startToEnd: Double
    public var startToEndHorizontal: Double
    public var startToEndVertical: Double
    /// Longest step between consecutive poses, seconds.
    public var largestGap: Double

    public var meanRateHz: Double? {
        guard poseCount > 1, duration > 0 else { return nil }
        return Double(poseCount - 1) / duration
    }
}

/// Where the cart is at one playhead.
public struct TrailSample: Equatable, Sendable {
    /// Seconds since the first pose.
    public var elapsed: Double
    /// The pose at or before the playhead. Segments `0..<index` are fully travelled.
    public var index: Int
    /// How far along segment `index → index + 1`, 0…1.
    public var fraction: Double
    public var position: TrailPoint
    public var yaw: Double
    /// Path length from the first pose to here, metres.
    public var travelled: Double
}
