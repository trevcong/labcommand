import Foundation

/// Provenance sidecar required by DATA-CAPTURE.md §3.
public enum CaptureManifest {
    public static let requiredKeys: [String] = [
        "session_id",
        "site",
        "layout_tag",
        "path_script",
        "duration_s",
        "deliberate_change",
        "operator",
        "ptp_on",
        "xfer_format",
        "record_topics",
        "forbidden_topics",
        "git_hash_driver",
        "git_hash_slam",
        "git_hash_sdk",
        "container",
        "image",
    ]

    /// Driver lock-in: CustomMsg / format 1. Never PointCloud2.
    public static let xferFormat = 1
    public static let xferFormatName = "CustomMsg"

    public static func fields(
        spec: CaptureSessionSpec,
        sessionID: String,
        ptpOn: Bool,
        driverSHA: String = "",
        slamSHA: String = "",
        sdkSHA: String = "",
        operatorName: String = ""
    ) -> [String: String] {
        [
            "session_id": sessionID,
            "site": spec.site,
            "layout_tag": spec.layoutTag,
            "path_script": spec.preset.pathScript,
            "duration_s": "\(spec.durationSeconds)",
            "deliberate_change": spec.deliberateChange,
            "operator": operatorName.isEmpty ? spec.operatorName : operatorName,
            "ptp_on": ptpOn ? "true" : "false",
            "xfer_format": "\(xferFormat)",
            "xfer_format_name": xferFormatName,
            "record_topics": CaptureRecipe.recordTopics.joined(separator: " "),
            "forbidden_topics": CaptureRecipe.forbiddenTopics.joined(separator: " "),
            "git_hash_driver": driverSHA,
            "git_hash_slam": slamSHA,
            "git_hash_sdk": sdkSHA,
            "container": CaptureRecipe.container,
            "image": CaptureRecipe.image,
        ]
    }

    public static func yaml(from fields: [String: String]) -> String {
        var lines = ["# capture provenance — DATA-CAPTURE.md §3"]
        for key in requiredKeys {
            let value = fields[key] ?? ""
            lines.append("\(key): \(yamlEscape(value))")
        }
        let extras = fields.keys.filter { !requiredKeys.contains($0) }.sorted()
        for key in extras {
            lines.append("\(key): \(yamlEscape(fields[key] ?? ""))")
        }
        return lines.joined(separator: "\n") + "\n"
    }

    public static func missingKeys(_ fields: [String: String]) -> [String] {
        // SHAs may be empty until Spark is live; the key must still be present.
        requiredKeys.filter { fields[$0] == nil }
    }

    private static func yamlEscape(_ s: String) -> String {
        if s.isEmpty { return "\"\"" }
        if s.contains(":") || s.contains("#") || s.contains("\n") || s.contains("\"") {
            let escaped = s.replacingOccurrences(of: "\"", with: "\\\"")
            return "\"\(escaped)\""
        }
        return s
    }
}

/// Reads the house-session review that already landed on this Mac.
public enum LandedSessionLoader {
    public static func defaultDirectory(home: URL = FileManager.default.homeDirectoryForCurrentUser) -> String {
        home.appendingPathComponent("Desktop/lidar-sessions", isDirectory: true).path
    }

    public static func load(
        from directory: String,
        fileManager: FileManager = .default
    ) -> LandedSession? {
        let dir = URL(fileURLWithPath: directory, isDirectory: true)
        let review = dir.appendingPathComponent("house_20260808_review.json")
        let pcd = dir.appendingPathComponent("house_20260808.pcd")
        let provenance = dir.appendingPathComponent("house_20260808_provenance.txt")

        guard fileManager.fileExists(atPath: pcd.path) || fileManager.fileExists(atPath: review.path) else {
            return nil
        }

        var points: Int?
        var fields: String?
        var bytes: Int?
        if let data = try? Data(contentsOf: review),
           let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any] {
            points = obj["points"] as? Int
            fields = obj["fields"] as? String
            bytes = obj["bytes"] as? Int
        }
        if bytes == nil, let attrs = try? fileManager.attributesOfItem(atPath: pcd.path) {
            bytes = (attrs[.size] as? NSNumber)?.intValue
        }

        var ptp: Bool?
        if let text = try? String(contentsOf: provenance, encoding: .utf8) {
            if text.contains("ptp4l running: 0") { ptp = false }
            else if text.contains("ptp4l running: 1") { ptp = true }
        }

        var bits: [String] = ["house_20260808.pcd"]
        if let points { bits.append("\(points) pts") }
        if let fields { bits.append(fields) }
        if let ptp { bits.append(ptp ? "ptp on" : "ptp off") }

        return LandedSession(
            path: pcd.path,
            points: points,
            fields: fields,
            bytes: bytes,
            ptp: ptp,
            summary: bits.joined(separator: " · ")
        )
    }
}
