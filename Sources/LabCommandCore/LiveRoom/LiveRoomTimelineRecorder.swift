import Foundation

/// What the Mac saw of the Spark over the last two hours, for the Room tab's timeline.
///
/// A pure value. The client calls `observe(_:)` after every mirror input (a message, a tick, a
/// connection change), and so does the fixture. It reads the mirror's newest `status` and
/// `roomActivity` and records them on Spark time only: `header.stamp` as Unix seconds. Mac uptime
/// (`receivedAt`) is kept only so the model can extrapolate "now".
///
/// Three interval lists stay separate and are never merged into one another:
/// - `link`: any status or `Activity` accepted by the mirror. Messages arrived, nothing more.
/// - `sensors`: a TRACKING status whose measured rates are all above 0 (and at least one is
///   measured). DEGRADED, BUILDING, RIG_MOVED and PAUSED do not count.
/// - `coverage`: an `Activity` received while `sensors` is healthy. Only then does an inactive
///   `Activity` mean quiet (protocol §3.7, §6).
public struct LiveRoomTimelineRecorder: Sendable, Equatable {
    /// A sample this far past the end of an interval still extends it: the status staleness rule.
    public static let gapAfter: TimeInterval = RoomMemoryMirror.statusFreshFor
    /// Spark time kept, back from the newest stamp.
    public static let retention: TimeInterval = 2 * 3600
    /// Activity samples kept at most: 2 h at 1 Hz, with room for jitter.
    public static let sampleCapacity = 10_000
    /// A `link` stamp this far behind the last one is reordering between the two topics, not a jump.
    static let reorderTolerance: TimeInterval = 1

    /// Spark seconds, `start <= end`.
    public struct Interval: Sendable, Equatable {
        public var start: Double
        public var end: Double

        public init(start: Double, end: Double) {
            self.start = start
            self.end = end
        }
    }

    public struct ActivitySample: Sendable, Equatable {
        public var stamp: Double
        public var active: Bool
        public var regionCount: UInt32
        public var voxelCount: UInt32
        /// Received while the sensors were healthy: an inactive sample then means quiet.
        public var covered: Bool
    }

    /// The Spark clock went back: `to` is earlier than the previous stamp of the same topic.
    public struct ClockJump: Sendable, Equatable {
        public var from: Double
        public var to: Double
    }

    /// The first and last stamps seen in one publisher generation (nil: the legacy run).
    public struct GenerationSpan: Sendable, Equatable {
        public var generation: UInt64?
        public var first: Double
        public var last: Double
    }

    public private(set) var link: [Interval] = []
    public private(set) var sensors: [Interval] = []
    public private(set) var coverage: [Interval] = []
    public private(set) var activity: [ActivitySample] = []
    /// The first stamp of each new active generation, after the first one seen.
    public private(set) var restarts: [Double] = []
    public private(set) var clockJumps: [ClockJump] = []
    public private(set) var generations: [GenerationSpan] = []
    /// The left edge of what is retained: the first stamp seen, or the retention cutoff once data
    /// has been dropped. Nil before anything was recorded.
    public private(set) var retainedSince: Double?
    /// The newest stamp observed (status or activity) and when the Mac received it.
    public private(set) var lastStamp: Double?
    public private(set) var lastReceivedAt: TimeInterval?

    /// The last health verdict: false after a status that was not healthy, until a healthy one.
    public private(set) var sensorsHealthy = false

    private struct Key: Sendable, Equatable {
        var generation: UInt64?
        var seq: UInt64
        var stamp: RoomMemory.Time
    }

    private var lastStatusKey: Key?
    private var lastActivityKey: Key?
    private var lastStatusStamp: Double?
    private var lastActivityStamp: Double?
    private var highestGeneration: UInt64?
    /// The highest cutoff `prune` applied: nothing left of it is held.
    private var prunedThrough: Double?

    public init() {}

    /// Unix seconds of a wire stamp.
    public static func seconds(_ time: RoomMemory.Time) -> Double {
        Double(time.sec) + Double(time.nanosec) / 1e9
    }

    /// Records the mirror's newest status and activity if they are new. Re-observing the same
    /// mirror changes nothing: each topic is keyed by (generation, seq, stamp).
    public mutating func observe(_ mirror: RoomMemoryMirror) {
        if let status = mirror.status {
            let key = Key(generation: status.publisherGeneration, seq: status.seq, stamp: status.stamp)
            if key != lastStatusKey {
                lastStatusKey = key
                recordStatus(status)
            }
        }
        if let latest = mirror.roomActivity {
            let header = latest.message.header
            let key = Key(generation: latest.message.publisherGeneration, seq: header.seq, stamp: header.stamp)
            if key != lastActivityKey {
                lastActivityKey = key
                recordActivity(latest)
            }
        }
    }

    // MARK: Samples

    private mutating func recordStatus(_ status: LiveRoomStatus) {
        let t = Self.seconds(status.stamp)
        let jumped = noteJump(t, previous: lastStatusStamp)
        lastStatusStamp = t
        if jumped { lastActivityStamp = Self.lowered(lastActivityStamp, by: t) }
        noteGeneration(status.publisherGeneration, at: t)
        addLink(t)
        let measured = [status.mid360Hz, status.jt128Hz].compactMap { $0 }
        let healthy = status.state == .tracking && !measured.isEmpty && measured.allSatisfy { $0 > 0 }
        if healthy {
            // A health break or a backward stamp closed the last interval: start a new one.
            if !sensorsHealthy || jumped {
                sensors.append(Interval(start: t, end: t))
            } else {
                Self.extend(&sensors, with: t)
            }
        }
        sensorsHealthy = healthy
        noteLast(t, receivedAt: status.receivedAt)
        prune()
    }

    private mutating func recordActivity(_ latest: LiveRoomLatest<RoomMemory.Activity>) {
        let message = latest.message
        let t = Self.seconds(message.header.stamp)
        let jumped = noteJump(t, previous: lastActivityStamp)
        lastActivityStamp = t
        if jumped { lastStatusStamp = Self.lowered(lastStatusStamp, by: t) }
        noteGeneration(message.publisherGeneration, at: t)
        addLink(t)
        let covered = isCovered(t)
        if covered {
            // Coverage never bridges a health break: it extends only inside one sensors interval.
            if jumped || (coverage.last?.end ?? -.infinity) < (sensors.last?.start ?? .infinity) {
                coverage.append(Interval(start: t, end: t))
            } else {
                Self.extend(&coverage, with: t)
            }
        }
        activity.append(ActivitySample(
            stamp: t,
            active: message.active,
            regionCount: message.regionCount,
            voxelCount: message.voxelCount,
            covered: covered
        ))
        if activity.count > Self.sampleCapacity {
            activity.removeFirst(activity.count - Self.sampleCapacity)
            // The cap dropped samples: what is left of the oldest kept one is not retained.
            if let oldest = activity.first?.stamp {
                retainedSince = Swift.max(retainedSince ?? oldest, oldest)
                prunedThrough = Swift.max(prunedThrough ?? oldest, oldest)
            }
        }
        noteLast(t, receivedAt: latest.receivedAt)
        prune()
    }

    /// Covered when the sensors are healthy now and `t` lies in the current healthy interval or
    /// within `gapAfter` past its end. An `Activity` stamped just after the last healthy status is
    /// covered; one after a status that was not healthy is not.
    private func isCovered(_ t: Double) -> Bool {
        guard sensorsHealthy, let current = sensors.last else { return false }
        return t >= current.start && t - current.end <= Self.gapAfter
    }

    /// One jump moves both topics back: the other topic's baseline follows, less the reorder
    /// tolerance, so its first stamp after the jump is not a second jump.
    private static func lowered(_ baseline: Double?, by t: Double) -> Double? {
        baseline.map { Swift.min($0, t - reorderTolerance) }
    }

    /// True when `t` is behind the topic's previous stamp: a clock jump, recorded once per occurrence.
    private mutating func noteJump(_ t: Double, previous: Double?) -> Bool {
        guard let last = previous, t < last else { return false }
        clockJumps.append(ClockJump(from: last, to: t))
        return true
    }

    private mutating func noteGeneration(_ generation: UInt64?, at t: Double) {
        if let generation {
            if let highest = highestGeneration, generation > highest {
                restarts.append(t)
            }
            if highestGeneration.map({ generation > $0 }) ?? true {
                highestGeneration = generation
            }
        }
        if let index = generations.lastIndex(where: { $0.generation == generation }) {
            generations[index].last = t
        } else {
            generations.append(GenerationSpan(generation: generation, first: t, last: t))
        }
    }

    /// `link` takes both topics, which run at 1 Hz with different phases: a stamp slightly behind
    /// the last interval's end is absorbed. Further back starts a new interval (the jump itself is
    /// recorded per topic).
    private mutating func addLink(_ t: Double) {
        guard let last = link.last else {
            link.append(Interval(start: t, end: t))
            return
        }
        if t >= last.start && t <= last.end {
            return
        }
        if t < last.start && last.start - t <= Self.reorderTolerance {
            link[link.count - 1].start = t
            return
        }
        if t < last.end && last.end - t <= Self.reorderTolerance {
            return
        }
        Self.extend(&link, with: t)
    }

    /// Extends the last interval when `0 <= t - end <= gapAfter`, else starts a new one.
    private static func extend(_ list: inout [Interval], with t: Double) {
        if let last = list.last, t - last.end >= 0, t - last.end <= gapAfter {
            list[list.count - 1].end = t
        } else {
            list.append(Interval(start: t, end: t))
        }
    }

    private mutating func noteLast(_ t: Double, receivedAt: TimeInterval) {
        lastStamp = t
        lastReceivedAt = receivedAt
        // A stamp left of the retained edge (a backward clock jump) widens it, down to what was pruned.
        if retainedSince.map({ t < $0 && t >= (prunedThrough ?? -.infinity) }) ?? true {
            retainedSince = t
        }
    }

    // MARK: Retention

    /// Drops what lies before the newest stamp minus `retention`.
    private mutating func prune() {
        guard let newest = lastStamp else { return }
        let cutoff = newest - Self.retention
        guard let since = retainedSince, since < cutoff else { return }
        retainedSince = cutoff
        prunedThrough = Swift.max(prunedThrough ?? cutoff, cutoff)
        Self.clip(&link, from: cutoff)
        Self.clip(&sensors, from: cutoff)
        Self.clip(&coverage, from: cutoff)
        activity.removeAll { $0.stamp < cutoff }
        restarts.removeAll { $0 < cutoff }
        clockJumps.removeAll { Swift.max($0.from, $0.to) < cutoff }
        generations.removeAll { $0.last < cutoff }
    }

    private static func clip(_ list: inout [Interval], from cutoff: Double) {
        list = list.compactMap { interval in
            guard interval.end >= cutoff else { return nil }
            return Interval(start: Swift.max(interval.start, cutoff), end: interval.end)
        }
    }
}
