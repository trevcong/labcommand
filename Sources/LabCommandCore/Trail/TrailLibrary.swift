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
        return TrailWalkcheck(
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
            wallGateLabel: parameters?["wall_gate_label"] as? String ?? walls?["proposed_gate_label"] as? String
        )
    }

    /// A finite JSON number. A boolean is not a number here.
    private static func number(_ value: Any?) -> Double? {
        guard let n = value as? NSNumber, CFGetTypeID(n) != CFBooleanGetTypeID() else { return nil }
        let d = n.doubleValue
        return d.isFinite ? d : nil
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
                modified: modified
            ))
        }
        return entries.sorted { a, b in
            a.modified != b.modified ? a.modified > b.modified : a.name < b.name
        }
    }
}
