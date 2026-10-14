import Foundation

/// When the 3D view may touch the Wi-Fi: never on its own. Nothing is open at
/// launch; a snapshot opens one socket per sensor, takes one frame each and closes
/// them; live streams until the operator stops it, `liveMaxDuration` passes, or
/// the window can't be seen. Whatever was captured stays on screen afterwards.
///
/// Pure state: the app feeds it requests, mailbox observations and clock ticks,
/// and opens exactly the sockets `sockets` names.
public struct LiveSession: Equatable, Sendable {
    /// A live session ends itself after this long.
    public static let liveMaxDuration: TimeInterval = 120
    /// A snapshot gives up on a sensor that hasn't sent a frame by then.
    public static let snapshotTimeout: TimeInterval = 10
    /// The live camera fit waits this long after the first frame for the other
    /// sensor, so the one fit frames both clouds. A JT128 frame is ~3 MB and can
    /// land well after the first Mid-360 one.
    public static let liveSettle: TimeInterval = 2

    public enum Phase: Equatable, Sendable {
        case off
        case snapshotting(startedAt: TimeInterval)
        /// Frames held on screen, from a snapshot or a stopped live session.
        case snapshot(capturedAt: Date)
        case live(startedAt: TimeInterval)
    }

    public enum StopReason: String, Equatable, Sendable {
        case requested = "stopped"
        case timeLimit = "time limit"
        case hidden = "window hidden"
    }

    /// What a step did, for the log. Nil from a step means nothing changed phase.
    public enum Outcome: Equatable, Sendable {
        case snapshotStarted
        /// Some frames arrived; `missing` timed out or ended without one.
        case snapshotTaken(capturedAt: Date, missing: Set<SensorID>)
        case snapshotEmpty
        case liveStarted
        /// Live ended; frames held as a snapshot unless `held` is nil.
        case liveStopped(StopReason, held: Date?)
    }

    public private(set) var phase: Phase = .off
    /// Points per sensor in the frames on screen.
    public private(set) var points: [SensorID: Int] = [:]
    /// Snapshot sensors still owed a frame — their sockets stay open.
    public private(set) var pending: Set<SensorID> = []
    /// Bumped once per captured snapshot, once per live session (when its frames
    /// settle), and on returning to OFF. The view refits the camera once per value,
    /// never per frame.
    public private(set) var fitGeneration = 0

    private var newestFrameAt: Date?
    private var firstLiveFrameAt: TimeInterval?
    private var liveFitted = false

    public init() {}

    public var mode: LiveViewMode {
        switch phase {
        case .off: return .off
        case .snapshotting, .snapshot: return .snapshot
        case .live: return .live
        }
    }

    public var isCapturing: Bool {
        if case .snapshotting = phase { return true }
        return false
    }

    public var isLive: Bool {
        if case .live = phase { return true }
        return false
    }

    /// Sensors that should have an open socket right now. Empty unless capturing
    /// or live.
    public var sockets: Set<SensorID> {
        switch phase {
        case .snapshotting: return pending
        case .live: return Set(SensorID.allCases)
        case .off, .snapshot: return []
        }
    }

    /// True while the open sockets should each take one frame and close.
    public var oneFrame: Bool { isCapturing }

    /// Seconds of live left, nil when not live.
    public func remaining(at now: TimeInterval) -> TimeInterval? {
        guard case .live(let startedAt) = phase else { return nil }
        return max(0, Self.liveMaxDuration - (now - startedAt))
    }

    // MARK: - Requests

    /// Ignored while a snapshot or live session is already running.
    public mutating func requestSnapshot(at now: TimeInterval) -> Outcome? {
        guard !isCapturing, !isLive else { return nil }
        begin(.snapshotting(startedAt: now))
        pending = Set(SensorID.allCases)
        return .snapshotStarted
    }

    /// Ignored while a snapshot or live session is already running.
    public mutating func requestLive(at now: TimeInterval) -> Outcome? {
        guard !isCapturing, !isLive else { return nil }
        begin(.live(startedAt: now))
        return .liveStarted
    }

    public mutating func stopLive(_ reason: StopReason = .requested) -> Outcome? {
        guard isLive else { return nil }
        if points.isEmpty {
            enterOff()
            return .liveStopped(reason, held: nil)
        }
        // The camera stays where the operator left it.
        let held = newestFrameAt ?? Date()
        phase = .snapshot(capturedAt: held)
        return .liveStopped(reason, held: held)
    }

    /// The window was hidden, minimized or covered, or the app hidden. Live stops;
    /// a snapshot in flight is at most `snapshotTimeout` from done and finishes.
    public mutating func viewHidden() -> Outcome? {
        stopLive(.hidden)
    }

    // MARK: - Observations

    /// The frames on screen now (per-sensor point counts) and when the newest of
    /// them arrived. The app clears the frames when a session begins, so anything
    /// here belongs to the current one.
    public mutating func observe(points current: [SensorID: Int], newest: Date?, at now: TimeInterval) -> Outcome? {
        switch phase {
        case .snapshotting:
            for (sensor, count) in current where pending.contains(sensor) {
                points[sensor] = count
                pending.remove(sensor)
            }
            noteNewest(newest)
            return pending.isEmpty ? finishSnapshot() : nil
        case .live:
            // Live drops a sensor that went quiet, so it can't pose as live.
            points = current
            noteNewest(newest)
            if !current.isEmpty, firstLiveFrameAt == nil {
                firstLiveFrameAt = now
            }
            settleLiveFit(at: now)
            return nil
        case .off, .snapshot:
            return nil
        }
    }

    /// The sensor's socket closed on its own: after its snapshot frame, or giving up.
    public mutating func sensorEnded(_ sensor: SensorID) -> Outcome? {
        guard isCapturing, pending.remove(sensor) != nil else { return nil }
        return pending.isEmpty ? finishSnapshot() : nil
    }

    public mutating func tick(at now: TimeInterval) -> Outcome? {
        switch phase {
        case .snapshotting(let startedAt) where now - startedAt >= Self.snapshotTimeout:
            pending = []
            return finishSnapshot()
        case .live(let startedAt):
            if now - startedAt >= Self.liveMaxDuration {
                return stopLive(.timeLimit)
            }
            settleLiveFit(at: now)
            return nil
        default:
            return nil
        }
    }

    // MARK: - Transitions

    private mutating func begin(_ next: Phase) {
        phase = next
        points = [:]
        pending = []
        newestFrameAt = nil
        firstLiveFrameAt = nil
        liveFitted = false
    }

    private mutating func finishSnapshot() -> Outcome {
        guard !points.isEmpty else {
            enterOff()
            return .snapshotEmpty
        }
        let capturedAt = newestFrameAt ?? Date()
        phase = .snapshot(capturedAt: capturedAt)
        fitGeneration += 1
        let missing = Set(SensorID.allCases).subtracting(points.keys)
        return .snapshotTaken(capturedAt: capturedAt, missing: missing)
    }

    private mutating func enterOff() {
        begin(.off)
        // Whatever stands in (the sample) frames itself afresh.
        fitGeneration += 1
    }

    private mutating func settleLiveFit(at now: TimeInterval) {
        guard !liveFitted, let first = firstLiveFrameAt, !points.isEmpty else { return }
        guard Set(points.keys) == Set(SensorID.allCases) || now - first >= Self.liveSettle else { return }
        liveFitted = true
        fitGeneration += 1
    }

    private mutating func noteNewest(_ newest: Date?) {
        guard let newest else { return }
        if newestFrameAt.map({ newest > $0 }) ?? true {
            newestFrameAt = newest
        }
    }
}

/// Dev/testing switches that run the same paths as the 3D view's buttons.
public enum LiveLaunchAction: String, Equatable, Sendable {
    case snapshot = "--snapshot-on-launch"
    case live = "--live-on-launch"

    /// The first of the two flags present; nil when neither is.
    public static func parse(_ arguments: [String]) -> LiveLaunchAction? {
        arguments.lazy.compactMap(LiveLaunchAction.init(rawValue:)).first
    }
}
