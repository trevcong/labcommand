import AppKit
import Combine
import LabCommandCore

/// One trail session and its playhead, plus the folder picker and the sessions list.
/// Loading reads local files on a background task; nothing here opens a socket or starts a process.
@MainActor
final class TrailPlayer: ObservableObject {
    static let localLine = "Local trail replay. No rig contact."

    enum Phase: Equatable {
        /// No session opened yet: the page shows the picker and the sessions list.
        case empty
        case loading(String)
        case failed(String)
        case ready
    }

    /// The drawn trail: poses at least `minStep` apart, plus the last pose.
    struct Vertex {
        var index: Int
        var point: TrailPoint
        /// 0…1 through the recording, for the trail colour.
        var fraction: Double
    }

    @Published private(set) var phase: Phase = .empty
    @Published private(set) var session: TrailSession?
    /// Session folders under `root`, newest first.
    @Published private(set) var entries: [TrailSessionEntry] = []
    /// The folder being shown or loaded.
    @Published private(set) var directory: String?
    let root = TrailLibrary.defaultRoot
    @Published private(set) var raster: TrailRaster?
    @Published private(set) var mapImage: CGImage?
    @Published private(set) var mapLine: String?
    @Published private(set) var bandLine: String?
    @Published private(set) var clock = TrailClock(duration: 0)
    private(set) var vertices: [Vertex] = []

    /// Applies to the first load only (the launch's own folder).
    private var startFraction: Double?
    private let restoresLastSession: Bool
    private var restored = false
    /// Bumped per load, so a slow earlier read cannot replace a later pick.
    private var loadToken = 0
    /// Verdict-file times for the session on screen. Nil until that load finishes, and cleared
    /// when the next one starts, so a rescan during a load does not start another.
    private var openedVerdictInputs: TrailLibrary.VerdictInputs?
    private var endTimer: Timer?
    private let defaults: UserDefaults

    /// Metres between drawn trail vertices. A 10 Hz trail at walking pace is ~5 cm per pose,
    /// so this only thins a stationary cart's repeated poses.
    static let minStep = 0.005

    init(startFraction: Double?, restoresLastSession: Bool, defaults: UserDefaults = .standard) {
        self.startFraction = startFraction
        self.restoresLastSession = restoresLastSession
        self.defaults = defaults
    }

    /// Monotonic seconds, the clock's time base.
    nonisolated static var now: Double { ProcessInfo.processInfo.systemUptime }

    var trajectory: TrailTrajectory? { session?.trajectory }

    var summary: TrailSummary? { session?.trajectory?.summary }

    func elapsed(at now: Double = TrailPlayer.now) -> Double { clock.elapsed(at: now) }

    func sample(at now: Double = TrailPlayer.now) -> TrailSample? {
        trajectory?.sample(atElapsed: clock.elapsed(at: now))
    }

    func load(directory: String) {
        loadToken += 1
        let token = loadToken
        endTimer?.invalidate()
        endTimer = nil
        openedVerdictInputs = nil
        // Drop the previous session so nothing (keys, readout) acts on it while this one reads.
        session = nil
        raster = nil
        mapImage = nil
        mapLine = nil
        bandLine = nil
        vertices = []
        clock = TrailClock(duration: 0)
        self.directory = directory
        phase = .loading(directory)
        Task.detached(priority: .userInitiated) { [weak self] in
            let result = Self.read(directory: directory)
            await self?.apply(result, token: token)
        }
    }

    // MARK: Picking a session

    /// The Trail tab appeared: list the root, and in a normal launch reopen the last folder once.
    func appeared() {
        rescan()
        guard restoresLastSession, !restored, phase == .empty else { return }
        restored = true
        if let last = TrailLibrary.reopenTarget(saved: defaults.string(forKey: TrailLibrary.lastSessionKey)) {
            load(directory: last)
        }
    }

    /// Re-lists the sessions root. Replays write new folders while the app is open.
    /// Also reloads the session on screen when its trajectory, map or walkcheck has changed
    /// since it was read, so a cached green PASS cannot outlive those files.
    func rescan() {
        let root = root
        let open = directory
        let opened = openedVerdictInputs
        Task.detached(priority: .utility) { [weak self] in
            let found = TrailLibrary.scan(root: root)
            let reload: String?
            if let open {
                let now = TrailLibrary.VerdictInputs.read(directory: open)
                reload = TrailLibrary.rescanShouldReload(opened: opened, now: now) ? open : nil
            } else {
                reload = nil
            }
            await self?.applyRescan(found, reload: reload)
        }
    }

    private func applyRescan(_ found: [TrailSessionEntry], reload: String?) {
        setEntries(found)
        // A pick made while the scan ran has a different directory. Leave it alone.
        guard let reload, reload == directory else { return }
        load(directory: reload)
    }

    private func setEntries(_ found: [TrailSessionEntry]) {
        if found != entries { entries = found }
    }

    /// Opens a folder the operator chose and remembers it for the next launch.
    func open(directory: String) {
        defaults.set(directory, forKey: TrailLibrary.lastSessionKey)
        load(directory: directory)
    }

    /// The synthetic loop. Not remembered: it is a demo, not a session.
    func openFixture() {
        load(directory: TrailSession.resolveDirectory(
            "fixture", resources: Bundle.main.resourceURL, cwd: FileManager.default.currentDirectoryPath))
    }

    /// NSOpenPanel for a session folder. Starts beside the last session, else in the sessions root.
    func chooseFolder() {
        let panel = NSOpenPanel()
        panel.title = "Open a session folder"
        panel.message = "A folder with trajectory.tum, map.pcd and manifest.json."
        panel.prompt = "Open"
        panel.canChooseDirectories = true
        panel.canChooseFiles = false
        panel.allowsMultipleSelection = false
        panel.canCreateDirectories = false
        if let start = TrailLibrary.pickerStart(saved: defaults.string(forKey: TrailLibrary.lastSessionKey), root: root) {
            panel.directoryURL = URL(fileURLWithPath: start, isDirectory: true)
        }
        guard panel.runModal() == .OK, let url = panel.url else { return }
        open(directory: url.path)
        rescan()
    }

    private struct Loaded: Sendable {
        var session: TrailSession
        /// Times taken before the session was read, so a later change still counts as new.
        var verdictInputs: TrailLibrary.VerdictInputs
        var raster: TrailRaster
        var rgba: [UInt8]
        var mapLine: String?
        var bandLine: String
    }

    nonisolated private static func read(directory: String) -> Result<Loaded, TrailError> {
        do {
            let verdictInputs = TrailLibrary.VerdictInputs.read(directory: directory)
            var session = try TrailSession.load(directory: directory)
            let raster = TrailRaster.build(map: session.map?.points ?? [], trajectory: session.trajectory)
            let mapLine = session.map.map { TrailReadout.mapLine($0, raster: raster) }
            // The raster holds everything the view draws. Let the points go.
            session.map?.points = []
            return .success(Loaded(session: session, verdictInputs: verdictInputs, raster: raster, rgba: raster.rgba(),
                                   mapLine: mapLine, bandLine: TrailReadout.bandLine(raster)))
        } catch let error as TrailError {
            return .failure(error)
        } catch {
            return .failure(.session("\(error)"))
        }
    }

    private func apply(_ result: Result<Loaded, TrailError>, token: Int) {
        guard token == loadToken else { return }
        switch result {
        case .failure(let error):
            phase = .failed("\(error)")
            FileHandle.standardError.write(Data("trail: \(error)\n".utf8))
        case .success(let loaded):
            session = loaded.session
            openedVerdictInputs = loaded.verdictInputs
            raster = loaded.raster
            mapImage = Self.image(loaded.raster, rgba: loaded.rgba)
            mapLine = loaded.mapLine
            bandLine = loaded.bandLine
            vertices = Self.vertices(loaded.session.trajectory)
            clock = TrailClock(duration: loaded.session.trajectory?.duration ?? 0)
            phase = .ready
            let now = Self.now
            if let startFraction {
                clock.seek(to: startFraction * clock.duration, at: now)
                self.startFraction = nil
            } else {
                clock.play(at: now)
            }
            scheduleEnd()
            for problem in loaded.session.problems {
                FileHandle.standardError.write(Data("trail: \(problem)\n".utf8))
            }
        }
    }

    // MARK: Transport

    func toggle() {
        clock.toggle(at: Self.now)
        scheduleEnd()
    }

    func seek(to elapsed: Double) {
        clock.seek(to: elapsed, at: Self.now)
        scheduleEnd()
    }

    /// Arrow keys: one second back or forward. Pauses so the step is visible.
    func step(_ seconds: Double) {
        let now = Self.now
        clock.pause(at: now)
        clock.seek(to: clock.elapsed(at: now) + seconds, at: now)
        scheduleEnd()
    }

    func rewind() {
        seek(to: 0)
    }

    func setSpeed(_ speed: Double) {
        clock.setSpeed(speed, at: Self.now)
        scheduleEnd()
    }

    /// Pauses exactly at the end so the timeline stops redrawing.
    private func scheduleEnd() {
        endTimer?.invalidate()
        endTimer = nil
        guard let wait = clock.secondsToEnd(at: Self.now) else { return }
        endTimer = Timer.scheduledTimer(withTimeInterval: wait + 0.01, repeats: false) { [weak self] _ in
            MainActor.assumeIsolated {
                guard let self else { return }
                self.clock.settle(at: Self.now)
            }
        }
    }

    // MARK: Drawing inputs

    private static func vertices(_ trajectory: TrailTrajectory?) -> [Vertex] {
        guard let trajectory, let first = trajectory.poses.first else { return [] }
        let span = max(trajectory.duration, 1e-9)
        var out = [Vertex(index: 0, point: first.position, fraction: 0)]
        for (i, pose) in trajectory.poses.enumerated().dropFirst() {
            let last = i == trajectory.poses.count - 1
            guard last || out[out.count - 1].point.distance(to: pose.position) >= minStep else { continue }
            out.append(Vertex(index: i, point: pose.position, fraction: (pose.t - first.t) / span))
        }
        return out
    }

    private static func image(_ raster: TrailRaster, rgba: [UInt8]) -> CGImage? {
        guard raster.width > 0, raster.height > 0, rgba.count == raster.width * raster.height * 4,
              let provider = CGDataProvider(data: Data(rgba) as CFData),
              let space = CGColorSpace(name: CGColorSpace.sRGB) else { return nil }
        return CGImage(
            width: raster.width,
            height: raster.height,
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: raster.width * 4,
            space: space,
            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
            provider: provider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        )
    }
}
