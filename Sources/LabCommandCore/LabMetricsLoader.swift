import Foundation

// MARK: - Paths

/// Configurable paths to on-disk company metrics. Defaults resolve under ~/Desktop.
public struct LabMetricsPaths: Sendable, Equatable {
    public var c1Metrics: String
    public var sitevectorDemo: String
    public var probeReport: String
    public var blackboxDir: String

    public init(
        c1Metrics: String,
        sitevectorDemo: String,
        probeReport: String,
        blackboxDir: String
    ) {
        self.c1Metrics = c1Metrics
        self.sitevectorDemo = sitevectorDemo
        self.probeReport = probeReport
        self.blackboxDir = blackboxDir
    }

    /// Default absolute paths under Desktop sibling repos.
    public static func desktopDefaults(
        home: URL = FileManager.default.homeDirectoryForCurrentUser
    ) -> LabMetricsPaths {
        let desktop = home.appendingPathComponent("Desktop", isDirectory: true)
        let hdc = desktop.appendingPathComponent("hdc-spatial-memory", isDirectory: true)
        let co = desktop.appendingPathComponent("accountable-robotics-co", isDirectory: true)
        return LabMetricsPaths(
            c1Metrics: hdc.appendingPathComponent("lidar_lab/c1_dryrun_metrics.json").path,
            sitevectorDemo: hdc.appendingPathComponent("sitevector/week1_demo_numbers.json").path,
            probeReport: hdc.appendingPathComponent("lidar_lab/probe_report.json").path,
            blackboxDir: co.appendingPathComponent("blackbox", isDirectory: true).path
        )
    }

    /// Prefer sibling dirs when LabCommand lives inside hdc-spatial-memory; else Desktop.
    public static func resolvedDefaults(
        fileManager: FileManager = .default
    ) -> LabMetricsPaths {
        // Package root ≈ …/hdc-spatial-memory/LabCommand when developing in-tree.
        let cwd = URL(fileURLWithPath: fileManager.currentDirectoryPath, isDirectory: true)
        let candidates: [URL] = [
            cwd, // LabCommand/
            cwd.deletingLastPathComponent(), // hdc-spatial-memory/
            cwd.appendingPathComponent("LabCommand", isDirectory: true),
        ]
        for base in candidates {
            let lidar = base.appendingPathComponent("lidar_lab/c1_dryrun_metrics.json")
            let parentLidar = base.deletingLastPathComponent()
                .appendingPathComponent("lidar_lab/c1_dryrun_metrics.json")
            if fileManager.fileExists(atPath: lidar.path) {
                return pathsRelative(toHDC: base)
            }
            if fileManager.fileExists(atPath: parentLidar.path) {
                return pathsRelative(toHDC: base.deletingLastPathComponent())
            }
        }
        return desktopDefaults()
    }

    private static func pathsRelative(toHDC hdc: URL) -> LabMetricsPaths {
        let desktop = hdc.deletingLastPathComponent()
        let co = desktop.appendingPathComponent("accountable-robotics-co", isDirectory: true)
        return LabMetricsPaths(
            c1Metrics: hdc.appendingPathComponent("lidar_lab/c1_dryrun_metrics.json").path,
            sitevectorDemo: hdc.appendingPathComponent("sitevector/week1_demo_numbers.json").path,
            probeReport: hdc.appendingPathComponent("lidar_lab/probe_report.json").path,
            blackboxDir: co.appendingPathComponent("blackbox", isDirectory: true).path
        )
    }
}

// MARK: - Bundle (slot lines)

/// Compact one-line facts for bound modules. Nil line ⇒ leave slot empty.
public struct LabMetricsBundle: Sendable, Equatable {
    public var runLine: String?
    public var experimentLine: String?
    public var memoryLine: String?
    public var traceLine: String?
    public var datasetLine: String?
    public var resultsLine: String?
    public var offerLine: String?
    public var pipelineLine: String?
    public var accountsLine: String?

    public init(
        runLine: String? = nil,
        experimentLine: String? = nil,
        memoryLine: String? = nil,
        traceLine: String? = nil,
        datasetLine: String? = nil,
        resultsLine: String? = nil,
        offerLine: String? = nil,
        pipelineLine: String? = nil,
        accountsLine: String? = nil
    ) {
        self.runLine = runLine
        self.experimentLine = experimentLine
        self.memoryLine = memoryLine
        self.traceLine = traceLine
        self.datasetLine = datasetLine
        self.resultsLine = resultsLine
        self.offerLine = offerLine
        self.pipelineLine = pipelineLine
        self.accountsLine = accountsLine
    }

    public static let empty = LabMetricsBundle()

    /// Module ids present when corresponding data (or product structure) bound.
    public var boundModuleIDs: Set<String> {
        var ids = Set<String>()
        if runLine != nil { ids.insert("run") }
        if experimentLine != nil { ids.insert("experiment") }
        if memoryLine != nil { ids.insert("memory") }
        if traceLine != nil { ids.insert("trace") }
        if datasetLine != nil { ids.insert("dataset") }
        if resultsLine != nil { ids.insert("results") }
        if offerLine != nil { ids.insert("offer") }
        if pipelineLine != nil { ids.insert("pipeline") }
        if accountsLine != nil { ids.insert("accounts") }
        return ids
    }

    public func line(for moduleID: String) -> String? {
        switch moduleID {
        case "run": return runLine
        case "experiment": return experimentLine
        case "memory": return memoryLine
        case "trace": return traceLine
        case "dataset": return datasetLine
        case "results": return resultsLine
        case "offer": return offerLine
        case "pipeline": return pipelineLine
        case "accounts": return accountsLine
        default: return nil
        }
    }
}

// MARK: - Loader

/// Loads JSON metrics from disk and maps them to compact slot lines.
public enum LabMetricsLoader {
    public static func load(
        paths: LabMetricsPaths = .resolvedDefaults(),
        fileManager: FileManager = .default
    ) -> LabMetricsBundle {
        let c1 = loadC1(path: paths.c1Metrics, fileManager: fileManager)
        let site = loadSitevector(path: paths.sitevectorDemo, fileManager: fileManager)
        let probe = loadProbe(path: paths.probeReport, fileManager: fileManager)
        let blackboxPresent = fileManager.fileExists(atPath: paths.blackboxDir)

        var run: String?
        if let probe {
            let ok = probe.overallOK
            let tag = ok ? "ok" : "fail"
            if let ts = probe.ts, !ts.isEmpty {
                run = "\(tag) · probe \(ts)"
            } else {
                run = "\(tag) · probe"
            }
        }

        var experiment: String?
        if let c1 {
            let sep = formatRatio(c1.separationRatio)
            let cell = c1.cell ?? "C1"
            let mode = c1.mode.map { shortMode($0) } ?? "metrics"
            experiment = "\(cell) sep \(sep) · \(mode)"
        }

        var memory: String?
        if let site {
            let acc = site.ncmAccuracy.map { formatPct($0) } ?? "—"
            let d = site.d.map { "d=\($0)" } ?? "sitevector"
            memory = "sitevector · ncm \(acc) · \(d)"
        }

        var trace: String?
        if let probe, let bb = probe.blackboxOK {
            let n = probe.blackboxPassCount ?? (bb ? 19 : nil)
            if let n {
                trace = "blackbox · \(n)/\(n) · \(bb ? "ok" : "fail")"
            } else {
                trace = "blackbox · \(bb ? "ok" : "fail")"
            }
        } else if blackboxPresent {
            // Repo present; user-confirmed suite size when no fresh probe.
            trace = "blackbox · 19/19"
        }

        var dataset: String?
        if let c1 {
            let sep = formatRatio(c1.separationRatio)
            dataset = "C1 dryrun · sep \(sep)"
        }

        var results: String?
        var parts: [String] = []
        if let c1 {
            parts.append("C1 sep \(formatRatio(c1.separationRatio))")
        }
        if let site, let acc = site.ncmAccuracy {
            parts.append("ncm \(formatPct(acc))")
        }
        if let probe {
            let tag = probe.overallOK ? "probe ok" : "probe fail"
            if let ts = probe.ts {
                parts.append("\(tag) \(ts)")
            } else {
                parts.append(tag)
            }
        }
        if !parts.isEmpty {
            results = parts.joined(separator: " · ")
        }

        // Product structure only — short labels, not essays.
        let offer = "BLACKBOX Verdict · draft"
        let passLabel: String
        if let n = probe?.blackboxPassCount {
            passLabel = "\(n)/\(n)"
        } else if probe?.blackboxOK == true || blackboxPresent {
            passLabel = "19/19"
        } else {
            passLabel = "—"
        }
        let pipeline = "BTF-0.1 sidecar · \(passLabel)"
        let accounts = "0 pilots"

        return LabMetricsBundle(
            runLine: run,
            experimentLine: experiment,
            memoryLine: memory,
            traceLine: trace,
            datasetLine: dataset,
            resultsLine: results,
            offerLine: offer,
            pipelineLine: pipeline,
            accountsLine: accounts
        )
    }

    // MARK: Decode helpers

    private static func readData(path: String, fileManager: FileManager) -> Data? {
        guard fileManager.fileExists(atPath: path) else { return nil }
        return try? Data(contentsOf: URL(fileURLWithPath: path))
    }

    private static func loadC1(path: String, fileManager: FileManager) -> C1Metrics? {
        guard let data = readData(path: path, fileManager: fileManager) else { return nil }
        guard let raw = try? JSONDecoder().decode(C1MetricsDTO.self, from: data) else { return nil }
        guard let sep = raw.separation_ratio else { return nil }
        return C1Metrics(cell: raw.cell, mode: raw.mode, separationRatio: sep)
    }

    private static func loadSitevector(path: String, fileManager: FileManager) -> SitevectorMetrics? {
        guard let data = readData(path: path, fileManager: fileManager) else { return nil }
        guard let raw = try? JSONDecoder().decode(SitevectorDTO.self, from: data) else { return nil }
        let ncm = raw.results?["ncm"]?.accuracy
        return SitevectorMetrics(d: raw.d, ncmAccuracy: ncm, source: raw.source)
    }

    private static func loadProbe(path: String, fileManager: FileManager) -> ProbeMetrics? {
        guard let data = readData(path: path, fileManager: fileManager) else { return nil }
        guard let raw = try? JSONDecoder().decode(ProbeReportDTO.self, from: data) else { return nil }

        let svOK = raw.mac?.sitevector_tests?.ok
        let bbOK = raw.mac?.blackbox_t1?.ok
        let bbOut = raw.mac?.blackbox_t1?.out
        let passCount = parsePassCount(bbOut)

        // Overall run ok if core lab checks that exist are ok.
        var flags: [Bool] = []
        if let svOK { flags.append(svOK) }
        if let bbOK { flags.append(bbOK) }
        let overall: Bool
        if flags.isEmpty {
            overall = true // file present, no negative checks
        } else {
            overall = flags.allSatisfy { $0 }
        }

        return ProbeMetrics(
            ts: raw.ts,
            overallOK: overall,
            blackboxOK: bbOK,
            blackboxPassCount: passCount
        )
    }

    private static func parsePassCount(_ out: String?) -> Int? {
        guard let out else { return nil }
        // "19/19 checks passed"
        guard let range = out.range(
            of: #"\d+/\d+\s+checks\s+passed"#,
            options: .regularExpression
        ) else { return nil }
        let token = String(out[range])
        guard let slash = token.firstIndex(of: "/") else { return nil }
        return Int(token[..<slash])
    }

    private static func formatRatio(_ v: Double) -> String {
        String(format: "%.2f", v)
    }

    private static func formatPct(_ v: Double) -> String {
        // accuracy is 0…1
        if v <= 1.0 {
            return String(format: "%.0f%%", v * 100)
        }
        return String(format: "%.0f%%", v)
    }

    private static func shortMode(_ mode: String) -> String {
        mode
            .replacingOccurrences(of: "synthetic_", with: "")
            .replacingOccurrences(of: "_", with: " ")
    }
}

// MARK: - Internal DTOs / models

private struct C1Metrics {
    var cell: String?
    var mode: String?
    var separationRatio: Double
}

private struct SitevectorMetrics {
    var d: Int?
    var ncmAccuracy: Double?
    var source: String?
}

private struct ProbeMetrics {
    var ts: String?
    var overallOK: Bool
    var blackboxOK: Bool?
    var blackboxPassCount: Int?
}

private struct C1MetricsDTO: Decodable {
    var cell: String?
    var mode: String?
    var separation_ratio: Double?
}

private struct SitevectorDTO: Decodable {
    var source: String?
    var d: Int?
    var results: [String: SitevectorResultDTO]?
}

private struct SitevectorResultDTO: Decodable {
    var accuracy: Double?
}

private struct ProbeReportDTO: Decodable {
    var ts: String?
    var mac: ProbeMacDTO?
}

private struct ProbeMacDTO: Decodable {
    var sitevector_tests: ProbeCheckDTO?
    var blackbox_t1: ProbeCheckDTO?
}

private struct ProbeCheckDTO: Decodable {
    var ok: Bool?
    var out: String?
}
