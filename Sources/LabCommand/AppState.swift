import Foundation
import Combine
import LabCommandCore

/// App destinations. Rig, Settings, and the live room. Trail replay is only reachable from a
/// trail launch (`LABCOMMAND_TRAIL_SESSION`), so it stays out of `navCases`.
enum Dest: String, CaseIterable, Identifiable {
    case workspace
    case settings
    case liveRoom
    case trail

    var id: String { rawValue }

    static var navCases: [Dest] { [.liveRoom, .settings, .workspace] }

    var label: String {
        switch self {
        case .workspace: return "Rig"
        case .settings: return "Settings"
        case .liveRoom: return "Room"
        case .trail: return "Trail"
        }
    }

    var windowTitle: String { "LabCommand - \(label)" }

    /// 1-based place in `navCases`, so ⌘1 is the first tab.
    var shortcutKey: String {
        let index = Self.navCases.firstIndex(of: self) ?? 0
        return String(index + 1)
    }
}

@MainActor
final class AppState: ObservableObject {
    @Published private(set) var snapshot: LabStatusSnapshot
    @Published private(set) var isRefreshing = false
    @Published var dest: Dest = .liveRoom

    /// Fast rig poll (API GET over the tunnel). Cheap — not an SSH probe.
    @Published var rigPollSeconds: Double = 5
    /// Slow connectivity probe (tailscale + ssh) for the Hardware page.
    @Published var probeIntervalSeconds: Double = 45
    @Published var autoRefreshEnabled: Bool = true

    /// The rig as last honestly measured. Recording gates and the left column read this.
    @Published private(set) var rig: RigSnapshot = RigSnapshot.offline(reason: "Not probed yet") {
        didSet { recomputeStatusDots() }
    }
    /// Top-bar Spark / Mid-360 / JT128 chips. The room link wins while it is fresh.
    @Published private(set) var statusDots = StatusDots.derive(
        rig: RigSnapshot.offline(reason: "Not probed yet"),
        connection: .off,
        status: nil,
        isStatusFresh: false
    )
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
    /// Snapshot B in world, once a local pair has loaded.
    @Published private(set) var snapshotMid: PointCloudFrame?
    @Published private(set) var snapshotJT: PointCloudFrame?
    @Published private(set) var overlayTimeline: ChangeOverlay.Timeline?
    @Published var overlayFilter = ChangeOverlay.Filter()
    @Published private(set) var overlayCount = 0
    /// Configuration error or load failure. Nil while a snapshot pair is fine.
    @Published private(set) var localFailure: String?
    /// Lidar clouds from Spark, on request only. Publishes on its own so live
    /// updates don't invalidate everything that observes AppState. Mock mode is
    /// for working without Spark — it doesn't reach for the real rig.
    let live: FoxgloveLiveClient
    /// Launch mode. Named apart from `session`, which is the capture spec.
    let launch: SnapshotSession
    /// Trajectory replay. Set only for a trail launch; its playhead publishes on its own so
    /// 60 Hz playback does not redraw everything that observes AppState.
    let trail: TrailPlayer?

    /// Live room socket. Nil in fixture mode: the view reads `fixture` instead.
    /// A snapshot launch isolates the network, so it leaves this nil too.
    let roomLaunch: LiveRoomLaunch
    /// Resolved once, from the environment and then saved defaults. Settings reads this.
    /// It does not resolve again on appear.
    private let roomEndpoint: RoomMemoryEndpointStore
    /// Saved cleartext switch. Off until the operator turns it on.
    /// `LABCOMMAND_ALLOW_CLEARTEXT=1` does not change this and is not saved.
    @Published private(set) var allowCleartextRoomWebSocket: Bool
    /// This process was started with `LABCOMMAND_ALLOW_CLEARTEXT=1`. The switch stays on the saved value.
    let cleartextFromEnvironment: Bool
    /// Set when the current address must not be dialed. The Room screen shows this string.
    @Published private(set) var roomEndpointBlock: String?
    /// The Room block asked to show the cleartext switch. Settings scrolls to it and clears this.
    @Published var showCleartextToggle = false
    let room: LiveRoomClient?
    @Published private(set) var fixture: LiveRoomFixture? {
        didSet { recomputeStatusDots() }
    }
    /// True after `start()` has opened the room socket. The Off screen is the other case.
    private(set) var roomDidStart = false
    private var fixtureTimer: Timer?

    private let service: ConnectivityService
    private let rigIO: RigServiceProtocol
    private var task: Task<Void, Never>?
    private var slowTimer: Timer?
    private var fastTimer: Timer?
    /// Watchdog lines already forwarded into the mission log.
    private var seenWatchdogLines: Set<String> = []
    private var adoptedSensorColors = false
    /// Room-client updates. Map bricks republish the client; the dots assign only when they change.
    private var roomDotsWatch: AnyCancellable?

    init(
        environment: [String: String] = ProcessInfo.processInfo.environment,
        service: ConnectivityService? = nil,
        rigIO: RigServiceProtocol? = nil
    ) {
        let launch = SnapshotSession(environment: environment)
        self.launch = launch
        if case .trail = launch.mode {
            self.trail = TrailPlayer(startFraction: TrailLaunch.startFraction(environment))
        } else {
            self.trail = nil
        }
        let roomLaunch = LiveRoomLaunch.parse(environment)
        self.roomLaunch = roomLaunch
        let endpointStore = RoomMemoryEndpointStore(environment: environment, defaults: .standard)
        self.roomEndpoint = endpointStore
        self.allowCleartextRoomWebSocket = endpointStore.allowCleartext
        self.cleartextFromEnvironment = endpointStore.cleartextFromEnvironment
        self.roomEndpointBlock = nil
        self.service = service ?? ConnectivityService()
        self.rigIO = rigIO ?? MockMode.createRigService(isolates: launch.isolates)
        // The room gate launch and every fixture leave the lidar client dark.
        self.live = FoxgloveLiveClient(
            isAvailable: !launch.isolates && roomLaunch.lidarAvailable && MockMode.parseArgs() == nil
        )
        switch roomLaunch {
        case .live:
            self.room = launch.isolates ? nil : LiveRoomClient(url: endpointStore.current)
            self.fixture = nil
        case .fixture(let mode):
            self.room = nil
            self.fixture = try? LiveRoomFixture.load(mode: mode, root: nil, now: ProcessInfo.processInfo.systemUptime)
        }
        self.room?.setCleartextPermitted(endpointStore.cleartextPermitted)
        self.snapshot = LabStatusSnapshot(
            overall: .unknown,
            spark: .unknown,
            tailscale: .unknown,
            checks: [],
            summaryLine: "—",
            refreshedAt: Date()
        )
        if launch.isolates {
            pointCloudColorMode = .sensor
            autoRefreshEnabled = false
        }
        // Room is home. A snapshot pair stays on Rig unless this launch also asked
        // for the room page. Do not open Room by setting `opensRoom`: that skips probes.
        if launch.isolates && !roomLaunch.opensRoom {
            dest = .workspace
        }
        // A trail launch opens on the replay, whatever the room variable says.
        if trail != nil {
            dest = .trail
        }
        recomputeStatusDots()
        watchRoomDots()
        refreshRoomEndpointBlock()
    }

    /// The endpoint this process is using. Settings copies it into the field on appear.
    var roomEndpointURL: URL { roomEndpoint.current }

    /// True when this process may dial cleartext. The saved switch or the environment flag.
    /// An environment URL does not turn this on.
    var showsCleartextWarning: Bool { allowCleartextRoomWebSocket || cleartextFromEnvironment }

    func setAllowCleartextRoomWebSocket(_ allow: Bool) {
        roomEndpoint.setAllowCleartext(allow)
        allowCleartextRoomWebSocket = allow
        applyRoomAdmission(startIfNeeded: true)
    }

    /// One step from the Room block to the cleartext switch. Does not turn the switch on.
    func openCleartextSettings() {
        dest = .settings
        showCleartextToggle = true
    }

    func cleartextToggleWasShown() {
        showCleartextToggle = false
    }

    /// Saves a valid endpoint and retargets the live socket. An invalid string, a disallowed
    /// cleartext host, or an unconfirmed `wss://` to `ws://` change leaves the socket alone.
    func commitRoomEndpoint(
        _ raw: String,
        confirmCleartextDowngrade: Bool = false
    ) -> RoomEndpointCommitResult {
        let result = roomEndpoint.commit(
            raw,
            retargeting: room,
            confirmCleartextDowngrade: confirmCleartextDowngrade
        )
        if case .saved = result {
            applyRoomAdmission(startIfNeeded: true)
        }
        return result
    }

    private func refreshRoomEndpointBlock() {
        guard let room else {
            roomEndpointBlock = nil
            return
        }
        switch RoomMemoryEndpoint.admit(room.url, allowCleartext: roomEndpoint.cleartextPermitted) {
        case .allowed:
            roomEndpointBlock = nil
        case .blocked(let message):
            roomEndpointBlock = message
        }
    }

    /// Dials only an admitted URL. A block stops a socket that was already open.
    /// The socket hears the switch before it dials, so a redirect is judged with the same permission.
    private func applyRoomAdmission(startIfNeeded: Bool) {
        room?.setCleartextPermitted(roomEndpoint.cleartextPermitted)
        refreshRoomEndpointBlock()
        guard let room else { return }
        if roomEndpointBlock == nil {
            if startIfNeeded, roomLaunch.startsClient, !roomDidStart {
                room.start()
                roomDidStart = true
            }
        } else if roomDidStart {
            room.stop()
            roomDidStart = false
        }
    }

    var roomMirror: RoomMemoryMirror {
        if let room { return room.mirror }
        return fixture?.mirror ?? RoomMemoryMirror()
    }

    var roomConnection: LiveRoomConnection {
        if let room { return room.connection }
        return fixture?.connection ?? .off
    }

    /// `objectWillChange` is sent before the new mirror is stored. The hop reads the
    /// value the client just published. Assigning only when the dots change keeps a
    /// map-brick update from redrawing the whole window.
    private func watchRoomDots() {
        guard let room else { return }
        roomDotsWatch = room.objectWillChange
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in
                self?.recomputeStatusDots()
            }
    }

    private func recomputeStatusDots() {
        let next = StatusDots.derive(
            rig: rig,
            connection: roomConnection,
            status: roomMirror.status,
            isStatusFresh: roomMirror.isStatusFresh
        )
        if next != statusDots {
            statusDots = next
        }
    }

    /// The bridge offers the request service or client publishing, or a relay sink is installed (D39).
    var requestPathExists: Bool {
        guard let room else { return false }
        return room.requestPath != .unavailable
    }

    func selectFixture(_ mode: LiveRoomFixture.Mode) {
        guard roomLaunch.fixtureMode != nil else { return }
        fixtureTimer?.invalidate()
        fixtureTimer = nil
        fixture = try? LiveRoomFixture.load(mode: mode, root: nil, now: ProcessInfo.processInfo.systemUptime)
        if LiveRoomFixture.keepsFresh(mode) { startFixtureClock() }
    }

    var health: RigConfidence { RigHealth.evaluate(rig) }

    var isolatesNetwork: Bool { launch.isolates }

    var localHeadline: String? {
        guard launch.isolates else { return nil }
        if let localFailure { return localFailure }
        if trail != nil { return TrailPlayer.localLine }
        if overlayTimeline == nil { return ChangeOverlay.readingLine }
        return ChangeOverlay.localSessionLine
    }

    var scrubberCaption: String {
        if let localFailure { return localFailure }
        guard let overlayTimeline else {
            return launch.isolates ? ChangeOverlay.readingLine : ""
        }
        return ChangeOverlay.caption(timeline: overlayTimeline, filter: overlayFilter, count: overlayCount)
    }

    var scrubberSlider: ChangeOverlay.SliderState {
        guard localFailure == nil, let overlayTimeline else {
            return ChangeOverlay.slider(filteredCount: 0, count: 0)
        }
        let matched = ChangeOverlay.filtered(overlayTimeline, overlayFilter).count
        return ChangeOverlay.slider(filteredCount: matched, count: overlayCount)
    }

    var scrubberChips: [ChangeOverlay.Chip] {
        guard localFailure == nil, let overlayTimeline else { return [] }
        return ChangeOverlay.chips(overlayTimeline)
    }

    var scrubberBefore: String {
        ChangeOverlay.clock(overlayTimeline?.beforeStamp, missing: "A")
    }

    var scrubberAfter: String {
        ChangeOverlay.clock(overlayTimeline?.afterStamp, missing: "B")
    }

    func overlayShows(_ kind: ChangeDetection.Kind) -> Bool {
        overlayFilter.allows(kind)
    }

    var overlayGeometry: ChangeOverlay.Geometry {
        guard localFailure == nil, let overlayTimeline else { return .empty }
        let events = ChangeOverlay.revealed(timeline: overlayTimeline, filter: overlayFilter, count: overlayCount)
        let emphasized = events.isEmpty ? nil : events.count - 1
        return ChangeOverlay.geometry(
            events: events,
            transform: RigTransforms.toWorld,
            emphasizedIndex: emphasized
        )
    }

    func setOverlayCount(_ count: Int) {
        guard let overlayTimeline, localFailure == nil else { return }
        let matched = ChangeOverlay.filtered(overlayTimeline, overlayFilter).count
        overlayCount = ChangeOverlay.clampedCount(count, filteredCount: matched)
    }

    func stepOverlay(_ delta: Int) {
        guard launch.isolates else { return }
        setOverlayCount(overlayCount + delta)
    }

    func setOverlayKind(_ kind: ChangeDetection.Kind, shown: Bool) {
        overlayFilter.set(kind, shown: shown)
        setOverlayCount(overlayCount)
    }

    func start() {
        if trail != nil {
            logLine(TrailPlayer.localLine)
        } else if launch.isolates {
            logLine(ChangeOverlay.localSessionLine)
        }
        let skipsProbes = roomLaunch.skipsProbes
        switch launch.begin(rig: { [weak self] in if !skipsProbes { self?.beginRig() } }) {
        case .rig:
            break
        case .failed(let message):
            localFailure = message
            logLine(message)
        case .load(let directoryA, let directoryB):
            Task.detached(priority: .userInitiated) { [weak self] in
                let loaded = SnapshotFiles.load(directoryA: directoryA, directoryB: directoryB)
                await self?.applySnapshot(loaded)
            }
        case .trail(let raw):
            let directory = TrailSession.resolveDirectory(
                raw,
                resources: Bundle.main.resourceURL,
                cwd: FileManager.default.currentDirectoryPath
            )
            logLine("trail session: \(directory)")
            trail?.load(directory: directory)
        }
        if roomLaunch.startsClient, room != nil {
            applyRoomAdmission(startIfNeeded: true)
        }
        if let mode = roomLaunch.fixtureMode, LiveRoomFixture.keepsFresh(mode) {
            startFixtureClock()
        }
    }

    private func beginRig() {
        refresh(manual: true)
        restartTimer()
        setUpPointClouds()
    }

    private func applySnapshot(_ loaded: SnapshotLoad) {
        switch loaded {
        case .failure(let message):
            localFailure = message
            overlayTimeline = nil
            snapshotMid = nil
            snapshotJT = nil
            overlayCount = 0
            logLine(message)
        case .ready(let clouds):
            localFailure = nil
            overlayTimeline = clouds.timeline
            snapshotMid = clouds.mid
            snapshotJT = clouds.jt
            let matched = ChangeOverlay.filtered(clouds.timeline, overlayFilter).count
            overlayCount = matched
            logLine(clouds.caption)
        }
    }

    private func logLine(_ text: String) {
        FileHandle.standardError.write(Data((text + "\n").utf8))
    }

    /// The 3D canvas opens no socket on its own: the sample PCD stands in until
    /// the operator takes a snapshot or goes live.
    private func setUpPointClouds() {
        live.onLog = { [weak self] text, kind in
            self?.appendLog(text, kind: kind)
        }
        live.onCaptured = { [weak self] in
            // Two sensors in one view read best by sensor color. Once only: after
            // that the picker is the operator's.
            guard let self, !self.adoptedSensorColors else { return }
            self.adoptedSensorColors = true
            self.pointCloudColorMode = .sensor
        }
        loadSamplePointCloud()

        // Dev/testing: the same paths as the 3D view's buttons, without a click.
        switch LiveLaunchAction.parse(CommandLine.arguments) {
        case .snapshot: live.takeSnapshot()
        case .live: live.startLive()
        case nil: break
        }
    }

    /// Load the sample PCD that stands in on the 3D canvas while nothing is captured.
    private func loadSamplePointCloud() {
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
        fixtureTimer?.invalidate()
        slowTimer = nil
        fastTimer = nil
        fixtureTimer = nil
        task?.cancel()
        room?.stop()
        live.shutdown()
    }

    private func startFixtureClock() {
        fixtureTimer?.invalidate()
        fixtureTimer = Timer.scheduledTimer(withTimeInterval: 1, repeats: true) { [weak self] _ in
            Task { @MainActor in
                guard let self else { return }
                self.fixture = self.fixture?.refreshed(at: ProcessInfo.processInfo.systemUptime)
            }
        }
    }

    func restartTimer() {
        slowTimer?.invalidate()
        fastTimer?.invalidate()
        slowTimer = nil
        fastTimer = nil
        launch.restartTimer { [weak self] in
            self?.installTimers()
        }
    }

    private func installTimers() {
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
    /// Snapshot mode returns before any of that.
    func refresh(manual: Bool) {
        launch.refresh(manual: manual) { [weak self] in
            self?.refreshOverNetwork(manual: manual)
        }
    }

    private func refreshOverNetwork(manual: Bool) {
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
    /// Snapshot mode returns before the request.
    func pollRig() {
        launch.pollRig { [weak self] in
            self?.pollOverNetwork()
        }
    }

    private func pollOverNetwork() {
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
        launch.runAction("stop view") { [weak self] in
            guard let self else { return }
            self.rigIO.stopTunnels()
            var next = self.rig
            next.viewTunnelUp = false
            self.rig = next
            self.appendLog("view tunnel closed")
        }
    }

    private func runAction(
        _ name: String,
        kind: MissionEventKind = .info,
        work: @escaping @Sendable (RigServiceProtocol, CaptureSessionSpec) -> String?,
        after: ((Bool) -> Void)? = nil
    ) {
        launch.runAction(name) { [weak self] in
            self?.runActionNow(name, kind: kind, work: work, after: after)
        }
    }

    private func runActionNow(
        _ name: String,
        kind: MissionEventKind,
        work: @escaping @Sendable (RigServiceProtocol, CaptureSessionSpec) -> String?,
        after: ((Bool) -> Void)?
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
            try? await Task.sleep(nanoseconds: 3_500_000_000)
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
