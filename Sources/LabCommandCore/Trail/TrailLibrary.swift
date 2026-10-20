import Foundation

/// `walkcheck.json` from workstream B (`python -m spatio.walkcheck`). Read-only here: the view
/// shows the verdict and the return error next to the accuracy label, never instead of it.
public struct TrailWalkcheck: Equatable, Sendable {
    public static let file = "walkcheck.json"
    public static let topdownFile = "topdown.png"
    public static let schema = "spatio.walkcheck.v1"
    public static let results = ["PASS", "FAIL", "NOT-EVALUABLE"]

    /// PASS, FAIL or NOT-EVALUABLE.
    public var result: String
    public var reasons: [String]
    public var detail: String?
    /// Nil when walkcheck did not score the return (`return_to_start` null).
    public var error3DCm: Double?
    public var horizontalCm: Double?
    public var verticalCm: Double?
    public var headingDeg: Double?
    public var thresholdM: Double?
    /// Length of each fixed start/end window, seconds.
    public var windowS: Double?
    public var pathLengthM: Double?
    /// The proposed wall gate: PASS or FAIL, or nil when walls were not scored.
    public var wallGate: String?
    public var wallP95Cm: Double?
    public var wallGateLabel: String?
    /// Walkcheck's label for its window, gap and motion thresholds.
    public var provisionalLabel: String?

    public static func parse(_ data: Data) throws -> TrailWalkcheck {
        guard let object = try? JSONSerialization.jsonObject(with: data), let dict = object as? [String: Any] else {
            throw TrailError.session("\(file) is not a JSON object")
        }
        let schemaValue = dict["schema"] as? String
        guard schemaValue == schema else {
            throw TrailError.session("\(file): schema \(schemaValue ?? "missing") is not \(schema)")
        }
        guard let result = dict["result"] as? String, results.contains(result) else {
            throw TrailError.session("\(file): result is not one of \(results.joined(separator: ", "))")
        }
        let returned = dict["return_to_start"] as? [String: Any]
        let parameters = dict["parameters"] as? [String: Any]
        let motion = dict["motion"] as? [String: Any]
        let walls = dict["walls"] as? [String: Any]
        let wallsScored = (walls?["status"] as? String) == "OK"
        let checked = TrailWalkcheck(
            result: result,
            reasons: (dict["reasons"] as? [Any])?.compactMap { $0 as? String } ?? [],
            detail: dict["detail"] as? String,
            error3DCm: number(returned?["error_3d_cm"]),
            horizontalCm: number(returned?["error_horizontal_cm"]),
            verticalCm: number(returned?["error_vertical_cm"]),
            headingDeg: number(returned?["heading_error_deg"]),
            thresholdM: number(returned?["threshold_m"]) ?? number(parameters?["return_pass_m"]),
            windowS: number(parameters?["stationary_window_s"]),
            pathLengthM: number(motion?["path_length_m"]),
            wallGate: wallsScored ? walls?["proposed_gate_result"] as? String : nil,
            wallP95Cm: wallsScored ? number(walls?["p95_point_to_plane_cm"]) : nil,
            wallGateLabel: parameters?["wall_gate_label"] as? String ?? walls?["proposed_gate_label"] as? String,
            provisionalLabel: parameters?["provisional_label"] as? String
        )
        if result == "PASS", let missing = passEvidenceProblem(checked, returned: returned, rawReasons: dict["reasons"]) {
            throw TrailError.session("\(file): result PASS but \(missing)")
        }
        return checked
    }

    /// What a PASS lacks, or nil when it carries what walkcheck always writes for one: no
    /// reasons, a scored return (`evaluated` and `within_threshold` JSON true) and a 3D error
    /// strictly under the threshold. The wall gate is not part of the verdict, so a PASS with
    /// a wall FAIL is fine. FAIL and NOT-EVALUABLE are shown as read; neither shows green.
    private static func passEvidenceProblem(_ w: TrailWalkcheck, returned: [String: Any]?, rawReasons: Any?) -> String? {
        guard (rawReasons as? [Any])?.isEmpty == true else { return "reasons is not an empty list" }
        guard let returned else { return "return_to_start is missing or null" }
        guard isTrue(returned["evaluated"]) else { return "return_to_start.evaluated is not true" }
        guard isTrue(returned["within_threshold"]) else { return "return_to_start.within_threshold is not true" }
        guard let error = w.error3DCm, error >= 0 else { return "return_to_start.error_3d_cm is not a number ≥ 0" }
        guard let threshold = w.thresholdM, threshold > 0 else { return "threshold_m is not a number > 0" }
        guard error < threshold * 100 else {
            return "error_3d_cm \(error) is not below threshold_m \(threshold) (\(threshold * 100) cm)"
        }
        return nil
    }

    /// JSON `true` only; NSNumber bridges 1 to true as well.
    private static func isTrue(_ value: Any?) -> Bool {
        guard let n = value as? NSNumber, CFGetTypeID(n) == CFBooleanGetTypeID() else { return false }
        return n.boolValue
    }

    /// A finite JSON number. A boolean is not a number here.
    private static func number(_ value: Any?) -> Double? {
        guard let n = value as? NSNumber, CFGetTypeID(n) != CFBooleanGetTypeID() else { return nil }
        let d = n.doubleValue
        return d.isFinite ? d : nil
    }

    /// Why the walkcheck in `directory` may be for earlier files, or nil when it is newer than
    /// both inputs it scored. Modification times only: a copy that keeps an old time is missed.
    public static func staleness(directory: String) -> String? {
        let fm = FileManager.default
        func modified(_ name: String) -> Date? {
            (try? fm.attributesOfItem(atPath: (directory as NSString).appendingPathComponent(name)))?[.modificationDate] as? Date
        }
        guard let scored = modified(file) else { return nil }
        let newer = [TrailSession.trajectoryFile, TrailSession.mapFile].filter { name in
            modified(name).map { scored < $0 } ?? false
        }
        guard !newer.isEmpty else { return nil }
        return "\(file) is older than \(newer.joined(separator: " and ")); its verdict may be for earlier files"
    }
}

/// One session folder found under the sessions root.
public struct TrailSessionEntry: Equatable, Sendable, Identifiable {
    public var directory: String
    public var name: String
    public var hasTrajectory: Bool
    public var hasMap: Bool
    /// The walkcheck verdict, when `walkcheck.json` reads.
    public var walkcheckResult: String?
    /// True when `walkcheck.json` is older than the trajectory or the map (`TrailWalkcheck.staleness`).
    public var walkcheckStale: Bool
    /// Newest of the trajectory and map modification times.
    public var modified: Date
    public var id: String { directory }
}

/// Where sessions live, which one to reopen, and what the root holds. Local files only.
public enum TrailLibrary {
    /// UserDefaults key for the last folder the operator opened from the Trail tab.
    public static let lastSessionKey = "trail.lastSessionDirectory"

    /// The shared-contract sessions folder.
    public static var defaultRoot: String {
        NSHomeDirectory() + "/Developer/spatio-data/sessions"
    }

    static func isDirectory(_ path: String) -> Bool {
        var isDir: ObjCBool = false
        return FileManager.default.fileExists(atPath: path, isDirectory: &isDir) && isDir.boolValue
    }

    /// The remembered folder, if it is still there.
    public static func reopenTarget(saved: String?) -> String? {
        guard let saved, !saved.isEmpty, isDirectory(saved) else { return nil }
        return saved
    }

    /// Where the folder picker starts: beside the last session, else the sessions root, else nil.
    public static func pickerStart(saved: String?, root: String) -> String? {
        if let last = reopenTarget(saved: saved) {
            return URL(fileURLWithPath: last).deletingLastPathComponent().path
        }
        return isDirectory(root) ? root : nil
    }

    /// Subfolders of `root` holding a trajectory or a map, newest first. Empty folders
    /// (a replay still starting) are left out; they have nothing to open yet.
    public static func scan(root: String) -> [TrailSessionEntry] {
        let fm = FileManager.default
        guard let names = try? fm.contentsOfDirectory(atPath: root) else { return [] }
        var entries: [TrailSessionEntry] = []
        for name in names where !name.hasPrefix(".") {
            let directory = (root as NSString).appendingPathComponent(name)
            guard isDirectory(directory) else { continue }
            let tum = (directory as NSString).appendingPathComponent(TrailSession.trajectoryFile)
            let map = (directory as NSString).appendingPathComponent(TrailSession.mapFile)
            let hasTrajectory = fm.fileExists(atPath: tum)
            let hasMap = fm.fileExists(atPath: map)
            guard hasTrajectory || hasMap else { continue }
            let modified = [tum, map]
                .compactMap { (try? fm.attributesOfItem(atPath: $0))?[.modificationDate] as? Date }
                .max() ?? .distantPast
            let walkcheck = fm.contents(atPath: (directory as NSString).appendingPathComponent(TrailWalkcheck.file))
                .flatMap { try? TrailWalkcheck.parse($0) }
            entries.append(TrailSessionEntry(
                directory: directory,
                name: name,
                hasTrajectory: hasTrajectory,
                hasMap: hasMap,
                walkcheckResult: walkcheck?.result,
                walkcheckStale: walkcheck != nil && TrailWalkcheck.staleness(directory: directory) != nil,
                modified: modified
            ))
        }
        return entries.sorted { a, b in
            a.modified != b.modified ? a.modified > b.modified : a.name < b.name
        }
    }
}
