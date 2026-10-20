import Foundation

/// `manifest.json` from the shared session contract. Every field is optional: a missing or
/// malformed manifest still opens the session, and the accuracy label stays on.
public struct TrailManifest: Equatable, Sendable {
    public var runID: String?
    public var sourceRecording: String?
    public var sensor: String?
    public var fastlioRevision: String?
    public var configFile: String?
    public var createdAt: String?
    public var notes: String?
    /// True only when the file holds JSON `true`. Absent, null, a string, or a number is false.
    public var physicalAccuracyEvaluated: Bool

    public static func parse(_ data: Data) throws -> TrailManifest {
        guard let object = try? JSONSerialization.jsonObject(with: data), let dict = object as? [String: Any] else {
            throw TrailError.session("manifest.json is not a JSON object")
        }
        func text(_ key: String) -> String? {
            if let s = dict[key] as? String { return s.isEmpty ? nil : s }
            return nil
        }
        // NSNumber bridges both Bool and Int; only a real JSON boolean counts.
        var evaluated = false
        if let number = dict["physical_accuracy_evaluated"] as? NSNumber,
           CFGetTypeID(number) == CFBooleanGetTypeID() {
            evaluated = number.boolValue
        }
        return TrailManifest(
            runID: text("run_id"),
            sourceRecording: text("source_recording"),
            sensor: text("sensor"),
            fastlioRevision: text("fastlio_revision"),
            configFile: text("config_file"),
            createdAt: text("created_at"),
            notes: text("notes"),
            physicalAccuracyEvaluated: evaluated
        )
    }
}

/// One session folder: `trajectory.tum`, `map.pcd`, `manifest.json`. Local files only.
public struct TrailSession: Equatable, Sendable {
    public static let trajectoryFile = "trajectory.tum"
    public static let mapFile = "map.pcd"
    public static let manifestFile = "manifest.json"
    /// The shared-contract wording. Other workstreams search for this string.
    public static let notEvaluatedLabel = "replay demonstrated; physical accuracy not evaluated"
    public static let noTrajectoryLabel = "map only; no trajectory.tum in this session"

    public var directory: String
    public var trajectory: TrailTrajectory?
    public var map: TrailMapCloud?
    public var manifest: TrailManifest?
    /// Files that were missing or unreadable. Shown in the side column, never hidden.
    public var problems: [String]
    /// Optional `walkcheck.json`. Its verdict is shown; it never changes `accuracyLabel`.
    public var walkcheck: TrailWalkcheck?
    /// Optional `topdown.png` written by walkcheck.
    public var topdownPath: String?

    public init(directory: String, trajectory: TrailTrajectory?, map: TrailMapCloud?, manifest: TrailManifest?, problems: [String],
                walkcheck: TrailWalkcheck? = nil, topdownPath: String? = nil) {
        self.directory = directory
        self.trajectory = trajectory
        self.map = map
        self.manifest = manifest
        self.problems = problems
        self.walkcheck = walkcheck
        self.topdownPath = topdownPath
    }

    public var name: String {
        manifest?.runID ?? URL(fileURLWithPath: directory).lastPathComponent
    }

    /// Nil only when this session's own manifest says `physical_accuracy_evaluated: true`.
    /// A walkcheck PASS, or walkcheck's copy of the manifest, does not count.
    public var accuracyLabel: String? {
        manifest?.physicalAccuracyEvaluated == true ? nil : Self.notEvaluatedLabel
    }

    /// Reads the folder. Throws only when neither a trajectory nor a map could be read.
    public static func load(directory: String) throws -> TrailSession {
        let fm = FileManager.default
        var isDir: ObjCBool = false
        guard fm.fileExists(atPath: directory, isDirectory: &isDir), isDir.boolValue else {
            throw TrailError.session("Session folder not found: \(directory)")
        }
        let root = URL(fileURLWithPath: directory)
        var problems: [String] = []

        var trajectory: TrailTrajectory?
        let tumPath = root.appendingPathComponent(trajectoryFile).path
        if fm.fileExists(atPath: tumPath) {
            do {
                guard let data = fm.contents(atPath: tumPath), let text = String(data: data, encoding: .utf8) else {
                    throw TrailError.session("\(trajectoryFile) is not UTF-8 text")
                }
                trajectory = try TrailTrajectory.parseTUM(text)
            } catch {
                problems.append("\(error)")
            }
        } else {
            problems.append("\(trajectoryFile) missing")
        }

        var map: TrailMapCloud?
        let mapPath = root.appendingPathComponent(mapFile).path
        if fm.fileExists(atPath: mapPath) {
            do {
                map = try TrailMapCloud.load(path: mapPath)
            } catch {
                problems.append("\(error)")
            }
        } else {
            problems.append("\(mapFile) missing")
        }

        var manifest: TrailManifest?
        let manifestPath = root.appendingPathComponent(manifestFile).path
        if let data = fm.contents(atPath: manifestPath) {
            do {
                manifest = try TrailManifest.parse(data)
            } catch {
                problems.append("\(error)")
            }
        } else {
            problems.append("\(manifestFile) missing")
        }

        guard trajectory != nil || map != nil else {
            throw TrailError.session("Nothing to show in \(directory): " + problems.joined(separator: "; "))
        }

        // Walkcheck is optional: absent is not a problem, unreadable is.
        var walkcheck: TrailWalkcheck?
        let walkcheckPath = root.appendingPathComponent(TrailWalkcheck.file).path
        if let data = fm.contents(atPath: walkcheckPath) {
            do {
                walkcheck = try TrailWalkcheck.parse(data)
                func modified(_ path: String) -> Date? {
                    (try? fm.attributesOfItem(atPath: path))?[.modificationDate] as? Date
                }
                if let scored = modified(walkcheckPath), let tum = modified(tumPath), scored < tum {
                    problems.append("\(TrailWalkcheck.file) is older than \(trajectoryFile); its verdict may be for an earlier trajectory")
                }
            } catch {
                problems.append("\(error)")
            }
        }
        let topdown = root.appendingPathComponent(TrailWalkcheck.topdownFile).path
        return TrailSession(directory: directory, trajectory: trajectory, map: map, manifest: manifest, problems: problems,
                            walkcheck: walkcheck, topdownPath: fm.fileExists(atPath: topdown) ? topdown : nil)
    }

    /// The folder named by `LABCOMMAND_TRAIL_SESSION`. `fixture` is the synthetic loop:
    /// the app bundle's copy first, then the repo's (for `swift run` from the repo root).
    public static func resolveDirectory(_ raw: String, resources: URL?, cwd: String) -> String {
        guard raw == "fixture" else { return (raw as NSString).expandingTildeInPath }
        let relative = "fixtures/trail/synthetic-loop"
        if let resources {
            let bundled = resources.appendingPathComponent(relative).path
            if FileManager.default.fileExists(atPath: bundled) { return bundled }
        }
        return URL(fileURLWithPath: cwd).appendingPathComponent(relative).path
    }
}

/// How a trail launch should open, from the environment.
public enum TrailLaunch {
    public static let sessionVariable = "LABCOMMAND_TRAIL_SESSION"
    /// 0…1. Opens paused at that fraction of the recording, for screenshots.
    public static let atVariable = "LABCOMMAND_TRAIL_AT"

    /// Nil means play from the start. A value outside 0…1 is clamped; a non-number is ignored.
    public static func startFraction(_ environment: [String: String]) -> Double? {
        guard let raw = environment[atVariable]?.trimmingCharacters(in: .whitespaces),
              let value = Double(raw), value.isFinite else { return nil }
        return min(max(value, 0), 1)
    }
}

/// Playhead arithmetic, apart from any timer. `now` is a monotonic clock in seconds.
public struct TrailClock: Equatable, Sendable {
    public static let speeds: [Double] = [0.5, 1, 2, 4, 8, 16]
    /// The default speed plays the whole recording in about this long.
    public static let targetPlaySeconds = 30.0

    public var duration: Double
    public private(set) var speed: Double
    public private(set) var isPlaying = false
    /// Playhead (seconds into the recording) at `anchorNow`.
    private var anchorElapsed = 0.0
    private var anchorNow = 0.0

    public init(duration: Double, speed: Double? = nil) {
        self.duration = max(0, duration)
        self.speed = speed ?? Self.defaultSpeed(duration: duration)
    }

    /// The slowest listed speed that plays `duration` within `targetPlaySeconds`.
    public static func defaultSpeed(duration: Double) -> Double {
        speeds.first { duration / $0 <= targetPlaySeconds } ?? speeds[speeds.count - 1]
    }

    public func elapsed(at now: Double) -> Double {
        guard isPlaying else { return anchorElapsed }
        return min(duration, anchorElapsed + max(0, now - anchorNow) * speed)
    }

    public func atEnd(at now: Double) -> Bool { elapsed(at: now) >= duration }

    /// Wall-clock seconds from `now` until the playhead reaches the end. Nil when paused.
    public func secondsToEnd(at now: Double) -> Double? {
        guard isPlaying else { return nil }
        return max(0, duration - elapsed(at: now)) / speed
    }

    /// Play from here; from the start when already at the end.
    public mutating func play(at now: Double) {
        let from = atEnd(at: now) ? 0 : elapsed(at: now)
        anchorElapsed = from
        anchorNow = now
        isPlaying = duration > 0
    }

    public mutating func pause(at now: Double) {
        anchorElapsed = elapsed(at: now)
        anchorNow = now
        isPlaying = false
    }

    public mutating func toggle(at now: Double) {
        if isPlaying { pause(at: now) } else { play(at: now) }
    }

    /// Moves the playhead; keeps playing if it was.
    public mutating func seek(to elapsed: Double, at now: Double) {
        anchorElapsed = min(max(elapsed, 0), duration)
        anchorNow = now
        if anchorElapsed >= duration { isPlaying = false }
    }

    public mutating func setSpeed(_ next: Double, at now: Double) {
        guard next > 0 else { return }
        anchorElapsed = elapsed(at: now)
        anchorNow = now
        speed = next
    }

    /// Stops at the end instead of running past it.
    public mutating func settle(at now: Double) {
        if isPlaying, atEnd(at: now) {
            anchorElapsed = duration
            anchorNow = now
            isPlaying = false
        }
    }
}

/// Readout text. Numbers are measured from the files; the wording says what they are not.
public enum TrailReadout {
    public static func metres(_ value: Double) -> String {
        if value < 1 { return String(format: "%.1f cm", value * 100) }
        return String(format: "%.2f m", value)
    }

    /// `m:ss.s`, or `h:mm:ss` past an hour.
    public static func clock(_ seconds: Double) -> String {
        let s = max(0, seconds)
        if s >= 3600 {
            let whole = Int(s)
            return String(format: "%d:%02d:%02d", whole / 3600, (whole / 60) % 60, whole % 60)
        }
        let minutes = Int(s / 60)
        return String(format: "%d:%04.1f", minutes, s - Double(minutes) * 60)
    }

    public static func speed(_ value: Double) -> String {
        value < 1 ? String(format: "%.1f×", value) : String(format: "%.0f×", value)
    }

    public static let startToEndNote = "raw first-to-last pose; not the scored return error"

    public struct Row: Equatable, Sendable, Identifiable {
        public var label: String
        public var value: String
        public var id: String { label }
    }

    /// Side-column rows at one playhead.
    public static func rows(summary: TrailSummary, sample: TrailSample?) -> [Row] {
        var rows: [Row] = []
        let travelled = sample?.travelled ?? 0
        let elapsed = sample?.elapsed ?? 0
        rows.append(Row(label: "Path length", value: "\(metres(travelled)) / \(metres(summary.pathLength))"))
        rows.append(Row(label: "Elapsed", value: "\(clock(elapsed)) / \(clock(summary.duration))"))
        rows.append(Row(label: "Start → end", value: metres(summary.startToEnd)))
        rows.append(Row(label: "  horizontal", value: metres(summary.startToEndHorizontal)))
        rows.append(Row(label: "  vertical", value: metres(summary.startToEndVertical)))
        var poses = "\(summary.poseCount)"
        if let hz = summary.meanRateHz { poses += String(format: " · %.1f Hz", hz) }
        rows.append(Row(label: "Poses", value: poses))
        rows.append(Row(label: "Largest gap", value: String(format: "%.2f s", summary.largestGap)))
        return rows
    }

    /// Rows for `walkcheck.json`. Walkcheck reports centimetres; shown as it scored them.
    public static func walkcheckRows(_ w: TrailWalkcheck) -> [Row] {
        var rows = [Row(label: "Verdict", value: w.result)]
        if let d3 = w.error3DCm {
            var value = metres(d3 / 100)
            if let threshold = w.thresholdM { value += " · need < \(metres(threshold))" }
            rows.append(Row(label: "Return error", value: value))
            if let h = w.horizontalCm { rows.append(Row(label: "  horizontal", value: metres(h / 100))) }
            if let v = w.verticalCm { rows.append(Row(label: "  vertical", value: metres(v / 100))) }
            if let heading = w.headingDeg { rows.append(Row(label: "  heading", value: String(format: "%.1f°", heading))) }
        } else {
            rows.append(Row(label: "Return error", value: "not scored"))
        }
        if let path = w.pathLengthM { rows.append(Row(label: "Path (walkcheck)", value: metres(path))) }
        if let gate = w.wallGate {
            var value = gate
            if let p95 = w.wallP95Cm { value += " · p95 \(metres(p95 / 100))" }
            rows.append(Row(label: "Walls (proposed)", value: value))
        }
        return rows
    }

    /// How walkcheck measured, and what its verdict is not.
    public static func walkcheckNote(_ w: TrailWalkcheck, accuracyLabel: String?) -> String {
        var parts: [String] = []
        if let window = w.windowS {
            parts.append(String(format: "Return error: mean of the first %.0f s to mean of the last %.0f s, end not aligned onto start.", window, window))
        }
        if !w.reasons.isEmpty { parts.append("Reasons: " + w.reasons.joined(separator: ", ") + ".") }
        if let detail = w.detail, !detail.isEmpty { parts.append(detail) }
        if w.wallGate != nil { parts.append("Wall gate is \(w.wallGateLabel ?? "proposed").") }
        if accuracyLabel != nil { parts.append("This verdict scores the replay, not a measured physical walk.") }
        return parts.joined(separator: " ")
    }

    /// One line about the map layer.
    public static func mapLine(_ map: TrailMapCloud, raster: TrailRaster) -> String {
        var line = "\(map.points.count) pts \(map.format.rawValue)"
        if map.stride > 1 { line += " (1 in \(map.stride) of \(map.declared))" }
        if map.dropped > 0 { line += " · \(map.dropped) NaN dropped" }
        line += String(format: " · %.0f cm cells", raster.cellSize * 100)
        return line
    }

    public static func bandLine(_ raster: TrailRaster) -> String {
        guard raster.band.lowerBound.isFinite, raster.band.upperBound.isFinite else { return "walls: all heights" }
        let source = raster.bandFollowsCart ? "cart height ±\(String(format: "%.1f", TrailRaster.bandHalfHeight)) m" : "map z spread"
        return String(format: "walls: z %.2f…%.2f m (%@)", raster.band.lowerBound, raster.band.upperBound, source)
    }
}
