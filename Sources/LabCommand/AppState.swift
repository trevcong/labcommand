import Foundation
import Combine
import LabCommandCore

/// App destinations. Simplified for the redesign — workspace and settings only.
enum Dest: String, CaseIterable, Identifiable {
    case workspace
    case settings

    var id: String { rawValue }

    static var navCases: [Dest] { [.workspace, .settings] }

    var label: String {
        switch self {
        case .workspace: return "Rig"
        case .settings: return "Settings"
        }
    }

    var shortcutKey: String {
        switch self {
        case .workspace: return "1"
        case .settings: return "2"
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

    /// The rig as last honestly measured.
    @Published private(set) var rig: RigSnapshot = RigSnapshot.offline(reason: "Not probed yet")
    @Published var session: CaptureSessionSpec = CaptureSessionSpec()
    @Published private(set) var missionEvents: [MissionEvent] = []
    @Published private(set) var isActing = false
    @Published private(set) var recordStartedAt: Date?
    @Published private(set) var lastActionError: String?
    @Published var focusMode = false
    @Published private(set) var lastStopSummary: String?
    @Published var diagnosePresented = false
    @Published var diagnoseStage: RigStage?
    @Published private(set) var lastDiagnose: RigAdvice?
    
    /// Current point cloud frame for 3D canvas.
    @Published var currentFrame: PointCloudFrame?
    @Published var pointCloudColorMode: PointCloudColorMode = .height

    private let service: ConnectivityService
    private let rigIO: RigServiceProtocol
    private var task: Task<Void, Never>?
    private var slowTimer: Timer?
    private var fastTimer: Timer?
    /// Watchdog lines already forwarded into the mission log.
    private var seenWatchdogLines: Set<String> = []

    init(
        service: ConnectivityService = ConnectivityService(),
        rigIO: RigServiceProtocol = MockMode.createRigService()
    ) {
        self.service = service
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

    func start() {
        refresh(manual: true)
        restartTimer()
        loadSamplePointCloudIfNeeded()
    }
    
    /// Load a sample PCD file for the 3D canvas when Spark is offline.
    func loadSamplePointCloudIfNeeded() {
        guard currentFrame == nil else { return }
        
        let fm = FileManager.default
        let filename = "mid360-frame.pcd"
        
        // Try app bundle first
        if let resourceURL = Bundle.main.resourceURL {
            let bundlePath = resourceURL
                .appendingPathComponent("fixtures/pointclouds")
                .appendingPathComponent(filename)
            if fm.fileExists(atPath: bundlePath.path) {
                if loadPCDFile(at: bundlePath.path, name: filename) { return }
            }
        }
        
        // Fallback to relative paths from cwd (for swift run / development)
        let cwd = fm.currentDirectoryPath
        let searchPaths = [
            "fixtures/pointclouds/\(filename)",
            "../fixtures/pointclouds/\(filename)",
            "../../fixtures/pointclouds/\(filename)",
        ]
        
        for relativePath in searchPaths {
            let url = URL(fileURLWithPath: cwd).appendingPathComponent(relativePath)
            if fm.fileExists(atPath: url.path) {
                if loadPCDFile(at: url.path, name: filename) { return }
            }
        }
        
        appendLog("fixture not found: \(filename) (3D canvas will be empty)", kind: .info)
    }
    
    private func loadPCDFile(at path: String, name: String) -> Bool {
        do {
            let frame = try PointCloudDecoder.loadPCD(from: path)
            currentFrame = frame
            appendLog("loaded fixture: \(name) (\(frame.count) pts)", kind: .info)
            return true
        } catch {
            appendLog("failed to load fixture: \(error)", kind: .error)
            return false
        }
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

    func refresh() { refresh(manual: false) }

    /// Full probe: connectivity checks (slow) + a rig poll.
    func refresh(manual: Bool) {
        task?.cancel()
        isRefreshing = true
        let svc = service
        let io = rigIO
        task = Task.detached(priority: .userInitiated) {
            let checks = svc.runAllChecks()
            let snap = StatusAggregation.buildSnapshot(from: checks)
            let rig = io.fetchStatus(sparkWasReachable: snap.spark == .connected)
            await MainActor.run {
                self.snapshot = snap
                self.applyRig(rig)
                self.isRefreshing = false
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
        forwardWatchdogEvents(next)
        if prev.recording.active && !next.recording.active {
            recordStartedAt = nil
        }
        if !prev.bothDark && next.bothDark && prev.apiReachable && next.apiReachable {
            appendLog("sensors went dark (unplugged or power lost)", kind: .info)
        }
    }

    private func forwardWatchdogEvents(_ s: RigSnapshot) {
        for line in s.events where !seenWatchdogLines.contains(line) {
            seenWatchdogLines.insert(line)
            if !missionEvents.isEmpty || seenWatchdogLines.count > s.events.count {
                appendLog("watchdog: \(line)", kind: .info)
            }
        }
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
        work: @escaping @Sendable (RigServiceProtocol, CaptureSessionSpec) -> String?,
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
            try? await Task.sleep(for: .seconds(3.5))
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
}
