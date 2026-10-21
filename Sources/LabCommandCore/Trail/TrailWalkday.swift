import Foundation

/// The final walk-day verdict for one session folder: `walk_result.json` and `walkday.json`
/// from `spatio/walkday/process.sh` (`python3 -m spatio.walkday.result`). In a walk-day
/// folder this is the verdict; `walkcheck.json` is only one of its inputs, so a walkcheck
/// PASS never stands in for a missing or non-PASS `walk_result.json`.
public struct TrailWalkday: Equatable, Sendable {
    public static let resultFile = "walk_result.json"
    public static let retiredFile = "walk_result.json.retired"
    public static let reportFile = "walkday.json"
    public static let resultSchema = "spatio.walk_result/1"
    public static let reportSchema = "spatio.walkday.report/1"
    public static let statuses = ["PASS", "FAIL", "NOT_EVALUABLE"]
    /// Files only process.sh writes. Any one makes the folder a walk-day folder.
    /// `logs/walkday.conf` is copied before anything else runs.
    public static let markers = [resultFile, retiredFile, reportFile,
                                 "logs/walkday.conf", "logs/provenance.json", "logs/input_gate.json"]

    /// PASS, FAIL or NOT_EVALUABLE, as `walk_result.json` writes it. NOT_EVALUABLE when that
    /// file is missing or cannot be read.
    public var status: String
    public var reasons: [String]
    /// True when `walk_result.json` is present and reads.
    public var hasResult: Bool
    /// `walk_result.json` `return_error`. Nil where it is null: result.py withholds the
    /// return error when the input gate blocks or the walk check could not evaluate.
    public var returnD3M: Double?
    public var returnHorizontalM: Double?
    public var returnVerticalM: Double?
    public var returnHeadingDeg: Double?
    public var returnThresholdM: Double?
    public var physicalAccuracyEvaluated: Bool
    /// From `walkday.json`. Nil when the report is absent or does not say.
    public var walkcheckResult: String?
    public var stoppedAt: String?
    public var synthetic: Bool
    public var placement: Placement?
    /// Nil when not recorded, which is not the same as none.
    public var acceptanceBlockers: [String]?
    public var inputGateBlockers: [String]?
    /// Why the verdict cannot be shown as a current acceptance. Empty for a clean result.
    public var concerns: [String]
    /// `walk_result.json` is older than the trajectory, the map or the walk check.
    public var stale: Bool
    /// The trajectory the result scored is missing or unusable.
    public var unverifiable: Bool

    public struct Placement: Equatable, Sendable {
        public var startResidualM: Double?
        public var endResidualM: Double?
        public var headingCheck: String?
        /// True only when the report holds JSON `true`.
        public var withinTolerance: Bool
        public var problems: [String]
        /// False for a dry run's placement, recorded but not applied to the manifest.
        public var applied: Bool
    }

    /// Green only for a current PASS with nothing in `concerns`.
    public var isCleanPASS: Bool { status == "PASS" && concerns.isEmpty && !stale && !unverifiable }

    /// True when `directory` holds any walk-day marker.
    public static func isWalkdayFolder(_ directory: String) -> Bool {
        markers.contains { FileManager.default.fileExists(atPath: (directory as NSString).appendingPathComponent($0)) }
    }

    /// Reads the folder's walk-day result, or nil when it is not a walk-day folder.
    public static func read(directory: String) -> TrailWalkday? {
        guard isWalkdayFolder(directory) else { return nil }
        let fm = FileManager.default
        func path(_ name: String) -> String { (directory as NSString).appendingPathComponent(name) }
        let runID = URL(fileURLWithPath: directory).lastPathComponent
        var w = TrailWalkday(status: "NOT_EVALUABLE", reasons: [], hasResult: false,
                             physicalAccuracyEvaluated: false, synthetic: false, concerns: [],
                             stale: false, unverifiable: false)

        var resultRunID: String?
        if let data = fm.contents(atPath: path(resultFile)) {
            do {
                let dict = try object(data, resultFile)
                guard dict["schema"] as? String == resultSchema else {
                    throw TrailError.session("\(resultFile): schema is not \(resultSchema)")
                }
                guard let status = dict["status"] as? String, statuses.contains(status) else {
                    throw TrailError.session("\(resultFile): status is not one of \(statuses.joined(separator: ", "))")
                }
                guard let id = dict["run_id"] as? String, !id.isEmpty else {
                    throw TrailError.session("\(resultFile): run_id is missing")
                }
                resultRunID = id
                w.hasResult = true
                w.status = status
                w.reasons = strings(dict["reasons"]) ?? []
                w.physicalAccuracyEvaluated = isTrue(dict["physical_accuracy_evaluated"])
                let returned = dict["return_error"] as? [String: Any]
                w.returnD3M = number(returned?["d3_m"])
                w.returnHorizontalM = number(returned?["horizontal_m"])
                w.returnVerticalM = number(returned?["vertical_m"])
                w.returnHeadingDeg = number(returned?["heading_deg"])
                w.returnThresholdM = number(returned?["threshold_m"])
                if id != runID {
                    w.concerns.append("\(resultFile) is for run \(id), not this folder (\(runID))")
                }
                if status == "PASS", let missing = passEvidenceProblem(w) {
                    w.concerns.append("\(resultFile): status PASS but \(missing)")
                }
            } catch {
                w.status = "NOT_EVALUABLE"
                w.concerns.append("\(error)")
            }
        } else if fm.fileExists(atPath: path(resultFile)) {
            w.concerns.append("\(resultFile) is unreadable")
        } else if fm.fileExists(atPath: path(retiredFile)) {
            w.concerns.append("no \(resultFile): the earlier result was retired and the re-score did not write a new one")
        } else {
            w.concerns.append("no \(resultFile): the run has not been scored")
        }

        if let data = fm.contents(atPath: path(reportFile)) {
            do {
                let dict = try object(data, reportFile)
                guard dict["schema"] as? String == reportSchema else {
                    throw TrailError.session("\(reportFile): schema is not \(reportSchema)")
                }
                w.walkcheckResult = dict["walkcheck_result"] as? String
                w.stoppedAt = dict["stopped_at"] as? String
                w.synthetic = isTrue(dict["synthetic"])
                w.acceptanceBlockers = strings(dict["acceptance_blockers"])
                w.inputGateBlockers = strings(dict["input_gate_blockers"])
                if let p = dict["placement"] as? [String: Any] {
                    w.placement = Placement(
                        startResidualM: number(p["start_placement_residual_m"]),
                        endResidualM: number(p["end_placement_residual_m"]),
                        headingCheck: p["heading_check"] as? String,
                        withinTolerance: isTrue(p["within_tolerance"]),
                        problems: strings(p["problems"]) ?? [],
                        applied: isTrue(p["applied"]))
                }
                let reportStatus = dict["status"] as? String
                if w.hasResult {
                    if reportStatus != w.status {
                        w.concerns.append("\(reportFile) says \(reportStatus ?? "nothing"), \(resultFile) says \(w.status)")
                    }
                    if let id = dict["run_id"] as? String, id != resultRunID {
                        w.concerns.append("\(reportFile) is for run \(id), \(resultFile) for \(resultRunID ?? "?")")
                    }
                } else {
                    // A stopped or rejected run: the report says why there is no result.
                    w.reasons = strings(dict["reasons"]) ?? []
                }
            } catch {
                w.concerns.append("\(error)")
            }
        } else if w.hasResult {
            w.concerns.append("\(reportFile) is missing, so placement and blockers are unknown")
        }

        if w.status == "PASS" {
            if w.acceptanceBlockers == nil {
                w.concerns.append("acceptance blockers are not recorded")
            } else if let blockers = w.acceptanceBlockers, !blockers.isEmpty {
                w.concerns.append("acceptance blocked: " + blockers.joined(separator: "; "))
            }
            if let gate = w.inputGateBlockers, !gate.isEmpty {
                w.concerns.append("input gate: " + gate.joined(separator: "; "))
            }
            if let p = w.placement, !p.withinTolerance {
                w.concerns.append("placement not within tolerance" + (p.problems.isEmpty ? "" : ": " + p.problems.joined(separator: "; ")))
            }
            if w.synthetic {
                w.concerns.append("SYNTHETIC recording, never physical evidence")
            }
        }

        if w.hasResult {
            let freshness = TrailWalkcheck.freshness(
                of: resultFile, against: [TrailSession.trajectoryFile, TrailSession.mapFile, TrailWalkcheck.file],
                directory: directory)
            if let reason = freshness.reason {
                w.concerns.append(reason)
                w.stale = true
                w.unverifiable = freshness.unverifiable
            }
        }
        return w
    }

    /// What a PASS lacks: a measured return error under its threshold.
    private static func passEvidenceProblem(_ w: TrailWalkday) -> String? {
        guard let d3 = w.returnD3M, d3 >= 0 else { return "return_error.d3_m is not a number ≥ 0" }
        guard let threshold = w.returnThresholdM, threshold > 0 else { return "return_error.threshold_m is not a number > 0" }
        guard d3 < threshold else { return "return_error.d3_m \(d3) is not below threshold_m \(threshold)" }
        return nil
    }

    private static func object(_ data: Data, _ name: String) throws -> [String: Any] {
        guard let object = try? JSONSerialization.jsonObject(with: data), let dict = object as? [String: Any] else {
            throw TrailError.session("\(name) is not a JSON object")
        }
        return dict
    }

    /// A JSON list of strings, or nil when the value is not a list.
    private static func strings(_ value: Any?) -> [String]? {
        (value as? [Any])?.map { ($0 as? String) ?? "\($0)" }
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
}

extension TrailReadout {
    /// The walk-day verdict as written in `walk_result.json`, marked when it is not a current result.
    public static func walkdayText(_ w: TrailWalkday) -> String {
        if w.unverifiable { return "\(w.status) · unverifiable" }
        if w.stale { return "\(w.status) · stale" }
        if w.status == "PASS" && !w.isCleanPASS { return "PASS · unconfirmed" }
        return w.status
    }

    /// Green only for a clean current PASS; a current FAIL is red; everything else is a warning.
    public static func walkdayTone(_ w: TrailWalkday) -> VerdictTone {
        if w.isCleanPASS { return .good }
        if w.status == "FAIL" && !w.stale && !w.unverifiable { return .bad }
        return .warn
    }

    /// Rows for the walk-day block. The return error is `walk_result.json`'s, in metres.
    public static func walkdayRows(_ w: TrailWalkday) -> [Row] {
        var rows = [Row(label: "Verdict", value: walkdayText(w))]
        if let d3 = w.returnD3M {
            var value = metres(d3)
            if let threshold = w.returnThresholdM { value += " · need < \(metres(threshold))" }
            rows.append(Row(label: "Return error", value: value))
            if let h = w.returnHorizontalM { rows.append(Row(label: "  horizontal", value: metres(h))) }
            if let v = w.returnVerticalM { rows.append(Row(label: "  vertical", value: metres(v))) }
            if let heading = w.returnHeadingDeg { rows.append(Row(label: "  heading", value: String(format: "%.1f°", heading))) }
        } else {
            rows.append(Row(label: "Return error", value: w.hasResult ? "withheld (not a measurement)" : "not scored"))
        }
        if let p = w.placement {
            var value = p.withinTolerance ? "within tolerance" : "NOT within tolerance"
            if !p.applied { value += " · not applied" }
            rows.append(Row(label: "Placement", value: value))
        } else {
            rows.append(Row(label: "Placement", value: "none recorded"))
        }
        let blockers: String
        switch w.acceptanceBlockers {
        case nil: blockers = "not recorded"
        case let list? where list.isEmpty: blockers = "none"
        case let list?: blockers = "\(list.count)"
        }
        rows.append(Row(label: "Blockers", value: blockers))
        if let gate = w.inputGateBlockers, !gate.isEmpty {
            rows.append(Row(label: "Input gate", value: "blocked (\(gate.count))"))
        }
        if let stopped = w.stoppedAt { rows.append(Row(label: "Stopped at", value: stopped)) }
        if let input = w.walkcheckResult { rows.append(Row(label: "Walk check (input)", value: input)) }
        return rows
    }

    /// Where the verdict comes from, what it is not, and why it is not green.
    public static func walkdayNote(_ w: TrailWalkday, accuracyLabel: String?) -> String {
        var parts = ["Final walk-day verdict (walk_result.json). walkcheck.json is one of its inputs, not the verdict."]
        if !w.concerns.isEmpty { parts.append("Not a current acceptance: " + w.concerns.joined(separator: "; ") + ".") }
        if !w.reasons.isEmpty { parts.append("Reasons: " + w.reasons.joined(separator: "; ") + ".") }
        if let blockers = w.acceptanceBlockers, !blockers.isEmpty { parts.append("Blockers: " + blockers.joined(separator: "; ") + ".") }
        if let gate = w.inputGateBlockers, !gate.isEmpty { parts.append("Input gate: " + gate.joined(separator: "; ") + ".") }
        if let p = w.placement, !p.problems.isEmpty { parts.append("Placement: " + p.problems.joined(separator: "; ") + ".") }
        if w.synthetic { parts.append("SYNTHETIC recording, not a physical walk.") }
        if accuracyLabel != nil || !w.physicalAccuracyEvaluated {
            parts.append("This verdict scores the replay, not a measured physical walk.")
        }
        return parts.joined(separator: " ")
    }
}
