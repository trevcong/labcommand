import Foundation

/// One recorded pose from the session's `/Odometry` track.
public struct TrajectoryPoint: Sendable, Equatable, Codable {
    public let x: Double
    public let y: Double
    /// Heading in degrees, 0-360.
    public let h: Double
    /// Index into `SessionTrajectory.actions`.
    public let a: Int
    /// Index into `SessionTrajectory.tasks`.
    public let t: Int
    /// Speed in m/s at this pose.
    public let s: Double

    public init(x: Double, y: Double, h: Double, a: Int, t: Int, s: Double) {
        self.x = x; self.y = y; self.h = h; self.a = a; self.t = t; self.s = s
    }
}

/// The walk itself, as recorded. Written next to the bag by
/// `export_trajectories.py`; the app never re-decodes an mcap to draw it.
public struct SessionTrajectory: Sendable, Equatable, Codable {
    public let session: String
    public let source: String
    public let frames: Int
    public let path_m: Double
    public let hz: Double
    public let actions: [String]
    public let tasks: [String]
    public let points: [TrajectoryPoint]

    public var isEmpty: Bool { points.isEmpty }

    /// Bounding box in metres. Nil for an empty track.
    public var bounds: (minX: Double, minY: Double, maxX: Double, maxY: Double)? {
        guard let f = points.first else { return nil }
        var b = (f.x, f.y, f.x, f.y)
        for p in points {
            b.0 = min(b.0, p.x); b.1 = min(b.1, p.y)
            b.2 = max(b.2, p.x); b.3 = max(b.3, p.y)
        }
        return (b.0, b.1, b.2, b.3)
    }

    public var duration: TimeInterval {
        hz > 0 ? Double(frames) / hz : 0
    }

    public func actionName(_ p: TrajectoryPoint) -> String {
        p.a >= 0 && p.a < actions.count ? actions[p.a] : "-"
    }

    public func taskName(_ p: TrajectoryPoint) -> String {
        p.t >= 0 && p.t < tasks.count ? tasks[p.t] : "-"
    }

    /// Frame counts per action, in codebook order. A symbol the walk never
    /// produced stays at zero rather than vanishing from the legend.
    public var actionCounts: [(name: String, count: Int)] {
        actions.enumerated().map { i, n in
            (n, points.reduce(0) { $0 + ($1.a == i ? 1 : 0) })
        }
    }

    public var topSpeed: Double { points.map(\.s).max() ?? 0 }
}

public enum SessionTrajectoryLoader {
    public static let fileName = "trajectory.json"

    /// Loads the sidecar from a session directory, or nil when the session has
    /// no recorded track. A missing file is a normal state, not an error.
    public static func load(
        sessionDirectory: URL,
        fileManager: FileManager = .default
    ) -> SessionTrajectory? {
        let url = sessionDirectory.appendingPathComponent(fileName)
        guard fileManager.fileExists(atPath: url.path),
              let data = try? Data(contentsOf: url),
              let t = try? JSONDecoder().decode(SessionTrajectory.self, from: data),
              !t.points.isEmpty else {
            return nil
        }
        return t
    }

    /// Every session under a root that carries a track, newest folder first.
    public static func loadAll(
        root: URL,
        fileManager: FileManager = .default
    ) -> [SessionTrajectory] {
        let names = (try? fileManager.contentsOfDirectory(atPath: root.path)) ?? []
        return names.sorted(by: >).compactMap {
            load(sessionDirectory: root.appendingPathComponent($0, isDirectory: true),
                 fileManager: fileManager)
        }
    }
}
