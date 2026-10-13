import Foundation
import Combine
import LabCommandCore

/// App destinations. Every case is reachable from the nav — a destination that
/// renders another destination's view is scaffolding (DESIGN-VISION.md bans
/// empty inspector columns).
enum Dest: String, CaseIterable, Identifiable {
    case workspace
    case devices
    case research
    case settings

    var id: String { rawValue }

    static var navCases: [Dest] { [.workspace, .devices, .research, .settings] }

    var label: String {
        switch self {
        case .workspace: return "Rig"
        case .devices: return "Hardware"
        case .research: return "Takes"
        case .settings: return "Settings"
        }
    }

    /// Command-number digit for menu and keyboard shortcut.
    var shortcutKey: String {
        switch self {
        case .workspace: return "1"
        case .devices: return "2"
        case .research: return "3"
        case .settings: return "4"
        }
    }
}

@MainActor
final class AppState: ObservableObject {
    @Published private(set) var snapshot: LabStatusSnapshot
    @Published private(set) var isRefreshing = false
    @Published var dest: Dest = .workspace

    /// Fast rig poll (API GET over the tunnel). Cheap — not an SSH probe.
    @Published var rigPollSeconds: Double = 5
    /// Slow connectivity probe (tailscale + ssh) for the Hardware page.
    @Published var probeIntervalSeconds: Double = 45
    @Published var autoRefreshEnabled: Bool = true

    @Published private(set) var boundModules: Set<String> = []
    @Published private(set) var metrics: LabMetricsBundle = .empty

    /// The rig as last honestly measured.
    @Published private(set) var rig: RigSnapshot = RigSnapshot.offline(reason: "Not probed yet")
    @Published private(set) var history = TelemetryStore.load()
    @Published var session: CaptureSessionSpec = CaptureSessionSpec()
    @Published private(set) var missionEvents: [MissionEvent] = []
    @Published private(set) var isActing = false
    @Published private(set) var recordStartedAt: Date?
    @Published private(set) var lastActionError: String?
    @Published var focusMode = false
    @Published private(set) var library: [SessionRecord] = []
    @Published private(set) var tracks: [SessionTrajectory] = []
    @Published var selectedTrack: String?
    @Published var libraryQuery = ""
    @Published var libraryTag: String?
    @Published var compareLeft: String?
    @Published var compareRight: String?
    @Published var noteDraft = ""
    @Published var noteTargetStem: String?
    @Published private(set) var lastStopSummary: String?
    @Published var diagnosePresented = false
    @Published var diagnoseStage: RigStage?
    @Published private(set) var lastDiagnose: RigAdvice?

    private let service: ConnectivityService
    private let metricsPaths: LabMetricsPaths
    private let rigIO: RigService
    private var task: Task<Void, Never>?
    private var slowTimer: Timer?
    private var fastTimer: Timer?
    /// Watchdog lines already forwarded into the mission log.
    private var seenWatchdogLines: Set<String> = []

    init(
        service: ConnectivityService = ConnectivityService(),
        metricsPaths: LabMetricsPaths = .resolvedDefaults(),
        rigIO: RigService = RigService()
    ) {
        self.service = service
        self.metricsPaths = metricsPaths
        self.rigIO = rigIO
        self.snapshot = LabStatusSnapshot(
            overall: .unknown,
            spark: .unknown,
            tailscale: .unknown,
            checks: [],
            summaryLine: "—",
            refreshedAt: Date()
        )
    }

    var health: RigConfidence { RigHealth.evaluate(rig) }

    var filteredLibrary: [SessionRecord] {
        SessionLibrary.filter(library, query: libraryQuery, tag: libraryTag)
    }

    var compareResult: SessionCompare? {
        guard let lp = compareLeft, let rp = compareRight,
              let left = library.first(where: { $0.path == lp }),
              let right = library.first(where: { $0.path == rp }) else {
            return nil
        }
        return SessionLibrary.compare(left, right)
    }

    func start() {
        loadMetrics()
        loadLibrary()
        loadTracks()
        refresh(manual: true)
        restartTimer()
    }

    func stop() {
        slowTimer?.invalidate()
        fastTimer?.invalidate()
        slowTimer = nil
        fastTimer = nil
        task?.cancel()
    }

    func restartTimer() {
        slowTimer?.invalidate()
        fastTimer?.invalidate()
        slowTimer = nil
        fastTimer = nil
        guard autoRefreshEnabled else { return }
        slowTimer = Timer.scheduledTimer(withTimeInterval: probeIntervalSeconds, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.refresh() }
        }
        fastTimer = Timer.scheduledTimer(withTimeInterval: rigPollSeconds, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.pollRig() }
        }
    }

    func loadMetrics() {
        let bundle = LabMetricsLoader.load(paths: metricsPaths)
        metrics = bundle
        boundModules = bundle.boundModuleIDs
    }

    func refresh() { refresh(manual: false) }

    /// Full probe: connectivity checks (slow) + a rig poll.
    func refresh(manual: Bool) {
        task?.cancel()
        isRefreshing = true
        let svc = service
        let io = rigIO
        task = Task.detached(priority: .userInitiated) {
            let checks = svc.runAllChecks()
            let snap = StatusAggregator.snapshot(from: checks)
            let rig = io.fetchStatus(sparkWasReachable: snap.spark == .connected)
            await MainActor.run {
                self.snapshot = snap
                self.applyRig(rig)
                self.isRefreshing = false
                self.loadMetrics()
                self.loadLibrary()
                self.loadTracks()
                if manual {
                    self.appendLog(self.health.headline, kind: .probe)
                }
            }
        }
    }

    /// Fast path: just the API. Never blocks the UI on SSH.
    func pollRig() {
        guard !isActing else { return }
        let io = rigIO
        let sparkWasUp = snapshot.spark == .connected || rig.sparkReachable
        Task.detached(priority: .utility) {
            let rig = io.fetchStatus(sparkWasReachable: sparkWasUp)
            await MainActor.run { self.applyRig(rig) }
        }
    }

    private func applyRig(_ next: RigSnapshot) {
        let prev = rig
        rig = next
        recordTelemetry(next)
        forwardWatchdogEvents(next)
        // Observed transitions worth one honest log line each.
        if prev.recording.active && !next.recording.active {
            recordStartedAt = nil
            loadLibrary()
        }
        if !prev.bothDark && next.bothDark && prev.apiReachable && next.apiReachable {
            appendLog("sensors went dark (unplugged or power lost)", kind: .info)
        }
    }

    private func forwardWatchdogEvents(_ s: RigSnapshot) {
        for line in s.events where !seenWatchdogLines.contains(line) {
            seenWatchdogLines.insert(line)
            // Only surface lines from this app run's lifetime forward; the
            // first poll seeds the seen-set silently.
            if !missionEvents.isEmpty || seenWatchdogLines.count > s.events.count {
                appendLog("watchdog: \(line)", kind: .info)
            }
        }
    }

    private func recordTelemetry(_ s: RigSnapshot) {
        guard s.apiReachable else { return }
        history = history.appending(
            TelemetrySample(
                at: Date(),
                pointRateHz: s.mid360.pointRateHz,
                imuRateHz: s.mid360.imuRateHz,
                sparkLoad: s.load1,
                sparkMemPct: s.memPct,
                blind: s.stackRunning && !s.anyStreaming && !s.bothDark,
                jtPointRateHz: s.jt128.pointRateHz,
                jtImuRateHz: s.jt128.imuRateHz,
                midOffsetMs: s.mid360.clockOffsetMs,
                jtOffsetMs: s.jt128.clockOffsetMs
            )
        )
        TelemetryStore.save(history)
    }

    func loadTracks() {
        tracks = SessionTrajectoryLoader.loadAll(
            root: URL(fileURLWithPath: SessionLibrary.defaultDirectory(), isDirectory: true))
        if selectedTrack == nil || !tracks.contains(where: { $0.session == selectedTrack }) {
            selectedTrack = tracks.first?.session
        }
    }

    var currentTrack: SessionTrajectory? {
        tracks.first { $0.session == selectedTrack } ?? tracks.first
    }

    func loadLibrary() {
        library = SessionLibrary.index(
            directory: SessionLibrary.defaultDirectory(),
            viewRunning: rig.viewTunnelUp
        )
    }

    func toggleFocus() {
        focusMode.toggle()
        dest = .workspace
        appendLog(focusMode ? "focus on" : "focus off", kind: .focus)
    }

    func diagnose(stage: RigStage? = nil) {
        let advice = RigDiagnose.advice(for: rig, stage: stage)
        lastDiagnose = advice
        diagnoseStage = advice.stage
        diagnosePresented = true
        appendLog(advice.summary, kind: .diagnose)
    }

    func attachNote(to record: SessionRecord, text: String) {
        let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else { return }
        _ = try? SessionLibrary.attachNote(
            directory: SessionLibrary.defaultDirectory(),
            stem: record.stem,
            text: trimmed
        )
        loadLibrary()
        noteDraft = ""
        noteTargetStem = nil
        appendLog("note \(record.stem)", kind: .note)
    }

    func toggleCompare(_ path: String) {
        if compareLeft == path {
            compareLeft = compareRight
            compareRight = nil
        } else if compareRight == path {
            compareRight = nil
        } else if compareLeft == nil {
            compareLeft = path
        } else if compareRight == nil {
            compareRight = path
        } else {
            compareLeft = compareRight
            compareRight = path
        }
    }

    // MARK: Actions — all through the command-center API.

    func startRig() {
        let restarting = rig.stackRunning
        runAction(restarting ? "restart rig" : "start rig", kind: .start) { io, _ in
            if let terr = io.ensureTunnels() {
                return "tunnels: \(terr)"
            }
            let r = io.postAction(restarting ? "restart" : "start")
            if !r.ok { return r.message }
            return nil
        }
    }

    func startRecord() {
        runAction("record", kind: .record) { io, spec in
            let r = io.postAction("record", arg: RigDesk.recordName(spec))
            if !r.ok { return r.message }
            return nil
        } after: { ok in
            if ok { self.recordStartedAt = Date() }
        }
    }

    func stopAll() {
        let recording = rig.recording.active
        let sessionName = rig.recording.session
        runAction(recording ? "stop record" : "stop rig", kind: .stop) { io, _ in
            if recording {
                let r = io.postAction("stop-record")
                if !r.ok { return r.message }
                return nil
            }
            let r = io.postAction("down")
            io.stopTunnels()
            if !r.ok { return r.message }
            return nil
        } after: { _ in
            if recording {
                var line = "take ended"
                if let sessionName { line = "take ended — \(sessionName) (bag on Spark ~/lidar-sessions)" }
                if let start = self.recordStartedAt {
                    let s = Int(Date().timeIntervalSince(start))
                    line += String(format: " · %d:%02d", s / 60, s % 60)
                }
                self.lastStopSummary = line
                self.appendLog(line, kind: .stop)
            } else {
                self.lastStopSummary = "rig stopped — drivers down, tunnels closed"
            }
            self.recordStartedAt = nil
            self.loadLibrary()
        }
    }

    func wakeJT() {
        runAction("wake JT128", kind: .start) { io, _ in
            let r = io.postAction("wake")
            return r.ok ? nil : r.message
        }
    }

    func standbyJT() {
        runAction("standby JT128", kind: .stop) { io, _ in
            let r = io.postAction("standby")
            return r.ok ? nil : r.message
        }
    }

    /// Hand the take to the collector: it fires itself at sustained lock and
    /// finalizes with no app or SSH session in the loop.
    func armAtLock() {
        let name = RigDesk.recordName(session)
        runAction("arm \(name)", kind: .record) { io, _ in
            let r = io.postAction("arm", arg: name, extra: [
                "protocol": "custody.points.v0",
                "pose_source": "static_rig_parked",
                "duration": 120,
            ])
            return r.ok ? nil : r.message
        }
    }

    func disarm() {
        runAction("disarm", kind: .stop) { io, _ in
            let r = io.postAction("disarm")
            return r.ok ? nil : r.message
        }
    }

    func startView() {
        runAction("view", kind: .info) { io, _ in
            io.ensureTunnels()
        }
    }

    func stopView() {
        rigIO.stopTunnels()
        var next = rig
        next.viewTunnelUp = false
        rig = next
        appendLog("view tunnel closed")
    }

    private func runAction(
        _ name: String,
        kind: MissionEventKind = .info,
        work: @escaping @Sendable (RigService, CaptureSessionSpec) -> String?,
        after: ((Bool) -> Void)? = nil
    ) {
        guard !isActing else { return }
        isActing = true
        lastActionError = nil
        appendLog("\(name)…", kind: kind)
        let io = rigIO
        let spec = session
        let sparkWasUp = rig.sparkReachable
        Task.detached(priority: .userInitiated) {
            let err = work(io, spec)
            // Give the collector one cycle to observe what the action changed.
            Thread.sleep(forTimeInterval: 3.5)
            let rig = io.fetchStatus(sparkWasReachable: sparkWasUp)
            await MainActor.run {
                self.applyRig(rig)
                self.isActing = false
                if let err, !err.isEmpty {
                    self.lastActionError = err
                    self.appendLog("\(name) failed — \(err)", kind: .error)
                    after?(false)
                } else {
                    self.appendLog("\(name) ok", kind: kind)
                    after?(true)
                }
            }
        }
    }

    func appendLog(_ text: String, kind: MissionEventKind = .info) {
        missionEvents = MissionLog.append(missionEvents, kind: kind, text: text)
    }

    func elapsedRecordLabel(now: Date = Date()) -> String? {
        guard rig.recording.active, let start = recordStartedAt else { return nil }
        let s = Int(now.timeIntervalSince(start))
        return String(format: "%d:%02d", s / 60, s % 60)
    }

    func detail(_ id: String) -> String {
        snapshot.checks.first(where: { $0.id == id })?.detail ?? "—"
    }

    func isBound(_ moduleID: String) -> Bool {
        boundModules.contains(moduleID)
    }

    func statusLine(_ moduleID: String) -> String? {
        metrics.line(for: moduleID)
    }
}
