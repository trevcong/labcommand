import Foundation

public struct SessionQuality: Sendable, Equatable {
    public var hasPointCloud: Bool
    public var hasProvenance: Bool
    public var hasReview: Bool
    public var hasNote: Bool
    public var pointsKnown: Bool
    public var ptpKnown: Bool

    public init(
        hasPointCloud: Bool,
        hasProvenance: Bool,
        hasReview: Bool,
        hasNote: Bool,
        pointsKnown: Bool,
        ptpKnown: Bool
    ) {
        self.hasPointCloud = hasPointCloud
        self.hasProvenance = hasProvenance
        self.hasReview = hasReview
        self.hasNote = hasNote
        self.pointsKnown = pointsKnown
        self.ptpKnown = ptpKnown
    }

    /// File-derived only. Never "science-grade".
    public var summary: String {
        var bits: [String] = []
        if hasPointCloud { bits.append("pcd") }
        if hasProvenance { bits.append("provenance") }
        if hasReview { bits.append("review") }
        if hasNote { bits.append("note") }
        return bits.isEmpty ? "no artifacts" : bits.joined(separator: " · ")
    }
}

public struct SessionRecord: Sendable, Equatable, Identifiable {
    public var id: String { path }
    public var path: String
    public var filename: String
    public var stem: String
    public var bytes: Int?
    public var points: Int?
    public var fields: String?
    public var ptp: Bool?
    public var tags: [String]
    public var notes: String
    public var artifacts: [String]
    public var quality: SessionQuality
    public var foxgloveURL: String?

    public init(
        path: String,
        filename: String,
        stem: String,
        bytes: Int? = nil,
        points: Int? = nil,
        fields: String? = nil,
        ptp: Bool? = nil,
        tags: [String] = [],
        notes: String = "",
        artifacts: [String] = [],
        quality: SessionQuality,
        foxgloveURL: String? = nil
    ) {
        self.path = path
        self.filename = filename
        self.stem = stem
        self.bytes = bytes
        self.points = points
        self.fields = fields
        self.ptp = ptp
        self.tags = tags
        self.notes = notes
        self.artifacts = artifacts
        self.quality = quality
        self.foxgloveURL = foxgloveURL
    }

    public var ptpDisplay: String {
        TelemetryFormat.flag(ptp, yes: "ptp on", no: "ptp off")
    }

    public var pointsDisplay: String {
        guard let points else { return TelemetryFormat.unavailable }
        return "\(points)"
    }

    public var bytesDisplay: String {
        TelemetryFormat.bytes(bytes)
    }
}

public struct SessionCompare: Sendable, Equatable {
    public var left: SessionRecord
    public var right: SessionRecord
    public var rows: [SessionCompareRow]

    public init(left: SessionRecord, right: SessionRecord, rows: [SessionCompareRow]) {
        self.left = left
        self.right = right
        self.rows = rows
    }
}

public struct SessionCompareRow: Sendable, Equatable, Identifiable {
    public var id: String { label }
    public var label: String
    public var left: String
    public var right: String

    public init(label: String, left: String, right: String) {
        self.label = label
        self.left = left
        self.right = right
    }
}

/// Indexes real files under `~/Desktop/lidar-sessions`. Does not invent science.
public enum SessionLibrary {
    public static func defaultDirectory(
        home: URL = FileManager.default.homeDirectoryForCurrentUser
    ) -> String {
        LandedSessionLoader.defaultDirectory(home: home)
    }

    public static func index(
        directory: String,
        fileManager: FileManager = .default,
        viewRunning: Bool = false
    ) -> [SessionRecord] {
        let dir = URL(fileURLWithPath: directory, isDirectory: true)
        guard let names = try? fileManager.contentsOfDirectory(atPath: dir.path) else {
            return []
        }
        let fox = viewRunning ? CaptureRecipe.viewConnectURL() : nil

        var stems: [String: [String]] = [:]
        for name in names {
            if name.hasPrefix(".") { continue }
            let stem = groupingStem(name)
            stems[stem, default: []].append(name)
        }

        var records: [SessionRecord] = []
        for (stem, files) in stems.sorted(by: { $0.key > $1.key }) {
            let pcd = files.first { $0.lowercased().hasSuffix(".pcd") }
            let mcap = files.first { $0.lowercased().hasSuffix(".mcap") }
            guard pcd != nil || mcap != nil || files.contains(where: { $0.hasSuffix("_review.json") }) else {
                continue
            }
            let primary = pcd ?? mcap ?? files.sorted().first!
            let primaryURL = dir.appendingPathComponent(primary)

            var bytes: Int?
            if let attrs = try? fileManager.attributesOfItem(atPath: primaryURL.path) {
                bytes = (attrs[.size] as? NSNumber)?.intValue
            }

            var points: Int?
            var fields: String?
            if let reviewName = files.first(where: { $0.hasSuffix("_review.json") }),
               let data = try? Data(contentsOf: dir.appendingPathComponent(reviewName)),
               let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any] {
                if let p = obj["points"] as? Int { points = p }
                else if let p = obj["points"] as? NSNumber { points = p.intValue }
                fields = obj["fields"] as? String
                if let b = obj["bytes"] as? Int { bytes = b }
                else if let b = obj["bytes"] as? NSNumber { bytes = b.intValue }
            }

            var ptp: Bool?
            if let provName = files.first(where: { $0.contains("provenance") }),
               let text = try? String(contentsOf: dir.appendingPathComponent(provName), encoding: .utf8) {
                if text.contains("ptp4l running: 0") { ptp = false }
                else if text.contains("ptp4l running: 1") { ptp = true }
                else if text.contains("ptp_on: false") { ptp = false }
                else if text.contains("ptp_on: true") { ptp = true }
            }

            let noteName = files.first { $0.hasSuffix(".operator-note.txt") }
                ?? "\(stem).operator-note.txt"
            let notePath = dir.appendingPathComponent(noteName)
            let notes = (try? String(contentsOf: notePath, encoding: .utf8)) ?? ""

            let quality = SessionQuality(
                hasPointCloud: pcd != nil,
                hasProvenance: files.contains(where: { $0.contains("provenance") }),
                hasReview: files.contains(where: { $0.hasSuffix("_review.json") }),
                hasNote: fileManager.fileExists(atPath: notePath.path) && !notes.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty,
                pointsKnown: points != nil,
                ptpKnown: ptp != nil
            )

            records.append(
                SessionRecord(
                    path: primaryURL.path,
                    filename: primary,
                    stem: stem,
                    bytes: bytes,
                    points: points,
                    fields: fields,
                    ptp: ptp,
                    tags: inferTags(stem: stem, files: files),
                    notes: notes.trimmingCharacters(in: .whitespacesAndNewlines),
                    artifacts: files.sorted(),
                    quality: quality,
                    foxgloveURL: fox
                )
            )
        }
        return records
    }

    public static func filter(
        _ records: [SessionRecord],
        query: String,
        tag: String? = nil
    ) -> [SessionRecord] {
        let q = query.trimmingCharacters(in: .whitespacesAndNewlines).lowercased()
        return records.filter { rec in
            if let tag, !tag.isEmpty, !rec.tags.contains(tag) {
                return false
            }
            if q.isEmpty { return true }
            let hay = [
                rec.filename,
                rec.stem,
                rec.notes,
                rec.fields ?? "",
                rec.quality.summary,
                rec.tags.joined(separator: " "),
            ].joined(separator: " ").lowercased()
            return hay.contains(q)
        }
    }

    public static func compare(_ left: SessionRecord, _ right: SessionRecord) -> SessionCompare {
        let rows = [
            SessionCompareRow(label: "file", left: left.filename, right: right.filename),
            SessionCompareRow(label: "bytes", left: left.bytesDisplay, right: right.bytesDisplay),
            SessionCompareRow(label: "points", left: left.pointsDisplay, right: right.pointsDisplay),
            SessionCompareRow(label: "fields", left: left.fields ?? TelemetryFormat.unavailable, right: right.fields ?? TelemetryFormat.unavailable),
            SessionCompareRow(label: "ptp", left: left.ptpDisplay, right: right.ptpDisplay),
            SessionCompareRow(label: "tags", left: left.tags.isEmpty ? "—" : left.tags.joined(separator: " "), right: right.tags.isEmpty ? "—" : right.tags.joined(separator: " ")),
            SessionCompareRow(label: "quality", left: left.quality.summary, right: right.quality.summary),
        ]
        return SessionCompare(left: left, right: right, rows: rows)
    }

    public static func noteURL(directory: String, stem: String) -> URL {
        URL(fileURLWithPath: directory, isDirectory: true)
            .appendingPathComponent("\(stem).operator-note.txt")
    }

    public static func attachNote(
        directory: String,
        stem: String,
        text: String,
        fileManager: FileManager = .default
    ) throws -> URL {
        let url = noteURL(directory: directory, stem: stem)
        try text.write(to: url, atomically: true, encoding: .utf8)
        return url
    }

    public static func viewTarget(viewRunning: Bool) -> String? {
        viewRunning ? CaptureRecipe.viewConnectURL() : nil
    }

    private static func groupingStem(_ name: String) -> String {
        let suffixes = [
            ".operator-note.txt",
            "_provenance.txt",
            "_review.json",
            "_review.png",
            "_core.png",
            ".pcd",
            ".mcap",
        ]
        for suffix in suffixes where name.hasSuffix(suffix) {
            return String(name.dropLast(suffix.count))
        }
        return (name as NSString).deletingPathExtension
    }

    private static func inferTags(stem: String, files: [String]) -> [String] {
        let blob = (stem + " " + files.joined(separator: " ")).lowercased()
        var tags: [String] = []
        if blob.contains("_a2_") || blob.contains("baseline_rewalk") { tags.append("A2") }
        else if blob.contains("_a_") || blob.contains("baseline") { tags.append("A") }
        if blob.contains("_b_") || blob.contains("furniture") { tags.append("B") }
        if blob.contains("_c_") || blob.contains("yaw") { tags.append("C") }
        return tags
    }
}
