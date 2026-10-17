import Foundation
import simd

/// The Room tab's live timeline. Pure: the mirror, the recorder, the window, the clock.
///
/// Everything on the axis is Spark time (`header.stamp`, Unix seconds). Mac uptime enters once,
/// to extrapolate "now" from the last status. The honest-state rules (protocol §6) hold on the
/// axis too: what was not observed is hatched, never drawn as quiet; a stale status makes "now"
/// an estimate; lost history is a band or a badge, never a gap that reads as a calm room.
public struct LiveRoomTimeline: Sendable, Equatable {
    /// The window presets: 5 min, 30 min, 2 h.
    public static let windows: [TimeInterval] = [5 * 60, 30 * 60, 2 * 3600]
    /// Axis steps, smallest first. The first that puts 4–8 labels inside the window wins.
    public static let tickSteps: [TimeInterval] = [10, 30, 60, 120, 300, 600, 900, 1800, 3600]

    public enum Lane: String, Sendable, Equatable, CaseIterable {
        case appeared, disappeared, moved, activity

        public var title: String {
            switch self {
            case .appeared: return "Appeared"
            case .disappeared: return "Disappeared"
            case .moved: return "Moved"
            case .activity: return "Activity"
            }
        }

        public static func of(_ kind: RoomMemory.ChangeEvent.Kind) -> Lane {
            switch kind {
            case .appeared: return .appeared
            case .disappeared: return .disappeared
            case .moved: return .moved
            }
        }
    }

    /// One received transition of an event.
    public struct Tick: Sendable, Equatable {
        public var stamp: Double
        public var state: RoomMemory.ChangeEvent.State
        public var seq: UInt64
    }

    /// A stretch of an item. `leadIn` and `candidate` draw dashed, `confirmed` solid.
    public struct Segment: Sendable, Equatable {
        public enum Style: Sendable, Equatable {
            /// From `first_seen` to the first transition received: "earlier steps not received".
            case leadIn
            case candidate
            case confirmed
        }

        public var start: Double
        public var end: Double
        public var style: Style

        public init(start: Double, end: Double, style: Style) {
            self.start = start
            self.end = end
            self.style = style
        }
    }

    /// One event on its kind's lane.
    public struct Item: Sendable, Equatable, Identifiable {
        /// The selection id, the same as the event column's row id.
        public var id: String
        public var eventId: UInt64
        public var kind: RoomMemory.ChangeEvent.Kind
        public var lane: Lane
        /// The newest state received.
        public var state: RoomMemory.ChangeEvent.State
        public var start: Double
        public var end: Double
        /// Still CANDIDATE or CONFIRMED in the active run: the span runs to now.
        public var open: Bool
        /// An open event of a retired run: dimmed, ends at that run's last stamp.
        public var earlierRun: Bool
        public var ticks: [Tick]
        public var segments: [Segment]
        /// "earlier steps not received", "outcome unknown".
        public var notes: [String]
        /// One line for hover.
        public var detail: String
        public var selected: Bool
    }

    public struct ActivitySegment: Sendable, Equatable {
        public enum State: Sendable, Equatable {
            case active
            case quiet
            /// Hatched: no `Activity` received while the sensors were healthy. Never quiet.
            case unobserved
        }

        public var start: Double
        public var end: Double
        public var state: State
    }

    /// A band across every lane.
    public struct Band: Sendable, Equatable {
        public enum Kind: Sendable, Equatable {
            /// No status or `Activity` arrived.
            case notObserved
            /// Before what this Mac retained, or before the oldest transition the ring holds.
            case historyUnknown
            /// Transitions the Spark declared unavailable.
            case lost(UInt64)
            /// Transitions not received that may still arrive.
            case retrieving(UInt64)
        }

        public var kind: Kind
        public var start: Double
        public var end: Double
        public var label: String
    }

    public struct Marker: Sendable, Equatable {
        public enum Kind: Sendable, Equatable {
            case restart, clockJump
        }

        public var kind: Kind
        public var stamp: Double
        public var label: String
        public var detail: String
    }

    public struct AxisTick: Sendable, Equatable {
        public var stamp: Double
        public var label: String
    }

    /// Window edges, Spark seconds.
    public var start: Double
    public var end: Double
    public var window: TimeInterval
    /// The window's right edge is now.
    public var following: Bool
    /// Spark now: the last status stamp plus the Mac time since it arrived. Nil before anything.
    public var now: Double?
    /// The status is not fresh: `now` is extrapolated past the last stamp.
    public var nowIsEstimate: Bool
    /// Legacy wire: nothing on the axis is verified.
    public var unverified: Bool
    public var lanes: [Lane]
    public var items: [Item]
    public var activity: [ActivitySegment]
    public var bands: [Band]
    public var markers: [Marker]
    /// History the axis cannot place: "n transitions lost · time unknown".
    public var badges: [String]
    public var ticks: [AxisTick]
    public var selection: String?

    // MARK: Make

    /// - Parameters:
    ///   - window: the span shown, one of `windows` or any positive length.
    ///   - end: the right edge in Spark seconds; nil follows now.
    ///   - selection: the selected row id, as the event column uses it.
    ///   - kinds: the event lanes shown; the activity lane always is.
    ///   - timeZone: the zone the axis labels are in. The app passes `.current`.
    ///   - now: Mac uptime, on the mirror's clock.
    public static func make(
        mirror: RoomMemoryMirror,
        recorder: LiveRoomTimelineRecorder,
        window: TimeInterval,
        end: Double? = nil,
        selection: String? = nil,
        kinds: Set<RoomMemory.ChangeEvent.Kind> = Set(RoomMemory.ChangeEvent.Kind.allCases),
        timeZone: TimeZone,
        now: TimeInterval
    ) -> LiveRoomTimeline {
        let span = Swift.max(window, 1)
        let fresh = mirror.isStatusFresh
        let lastStamp = recorder.lastStamp ?? mirror.status.map { seconds($0.stamp) }
        var nowStamp: Double?
        if let status = mirror.status {
            nowStamp = seconds(status.stamp) + Swift.max(0, now - status.receivedAt)
        } else if let last = recorder.lastStamp, let at = recorder.lastReceivedAt {
            nowStamp = last + Swift.max(0, now - at)
        }
        let right = end ?? nowStamp ?? lastStamp ?? 0
        let left = right - span
        let context = Context(left: left, right: right, now: nowStamp ?? right, timeZone: timeZone, seconds: span <= 30 * 60)

        var badges: [String] = []
        var bands = observedBands(recorder: recorder, fresh: fresh, lastStamp: lastStamp, nowStamp: nowStamp, context)
        bands += historyBands(mirror: mirror, recorder: recorder, context)
        bands += missingBands(mirror: mirror, badges: &badges, context)

        var items = currentItems(mirror: mirror, kinds: kinds, context)
        items += earlierItems(mirror: mirror, recorder: recorder, kinds: kinds, context)
        items = items.filter { $0.end >= left && $0.start <= right }
        for index in items.indices {
            items[index].selected = items[index].id == selection
        }
        items.sort { lhs, rhs in
            if lhs.start != rhs.start { return lhs.start < rhs.start }
            return lhs.id < rhs.id
        }

        var markers: [Marker] = []
        for stamp in recorder.restarts where stamp >= left && stamp <= right {
            markers.append(Marker(kind: .restart, stamp: stamp, label: "Spark restarted",
                                  detail: "Spark restarted · \(context.clock(stamp, seconds: true))"))
        }
        for jump in recorder.clockJumps where jump.to >= left && jump.to <= right {
            markers.append(Marker(kind: .clockJump, stamp: jump.to, label: "Spark clock jumped back",
                                  detail: "Spark clock jumped back · \(context.clock(jump.from, seconds: true)) → \(context.clock(jump.to, seconds: true))"))
        }
        markers.sort { $0.stamp < $1.stamp }

        return LiveRoomTimeline(
            start: left,
            end: right,
            window: span,
            following: end == nil,
            now: nowStamp,
            nowIsEstimate: !fresh,
            unverified: mirror.wire == .legacy,
            lanes: Lane.allCases.filter { lane in lane == .activity || kinds.contains { Lane.of($0) == lane } },
            items: items,
            activity: activitySegments(recorder: recorder, context),
            bands: bands,
            markers: markers,
            badges: badges,
            ticks: axisTicks(start: left, end: right, timeZone: timeZone),
            selection: selection
        )
    }

    /// Window edges and formatting shared by the builders: one formatter per format per `make`.
    private struct Context {
        var left: Double
        var right: Double
        var now: Double
        var timeZone: TimeZone
        /// Labels carry seconds: the window is 30 min or less.
        var seconds: Bool
        private let minutes: DateFormatter
        private let withSeconds: DateFormatter

        init(left: Double, right: Double, now: Double, timeZone: TimeZone, seconds: Bool) {
            self.left = left
            self.right = right
            self.now = now
            self.timeZone = timeZone
            self.seconds = seconds
            minutes = LiveRoomTimeline.formatter(seconds: false, timeZone: timeZone)
            withSeconds = LiveRoomTimeline.formatter(seconds: true, timeZone: timeZone)
        }

        func clock(_ stamp: Double, seconds: Bool? = nil) -> String {
            let formatter = seconds ?? self.seconds ? withSeconds : minutes
            return formatter.string(from: Date(timeIntervalSince1970: stamp))
        }

        /// The part of `[start, end]` inside the window, or nil.
        func clip(_ start: Double, _ end: Double) -> (Double, Double)? {
            let lo = Swift.max(start, left)
            let hi = Swift.min(end, right)
            return lo <= hi ? (lo, hi) : nil
        }
    }

    // MARK: Bands

    /// The complement of `link` in the window, right of `retainedSince`. A fresh status vouches up
    /// to now; a stale one leaves the part after the last stamp unobserved: "no data since …".
    private static func observedBands(
        recorder: LiveRoomTimelineRecorder,
        fresh: Bool,
        lastStamp: Double?,
        nowStamp: Double?,
        _ context: Context
    ) -> [Band] {
        var link = recorder.link.map { ($0.start, $0.end) }
        if fresh, let lastStamp, let nowStamp, nowStamp > lastStamp {
            link.append((lastStamp, nowStamp))
        }
        let from = Swift.max(context.left, recorder.retainedSince ?? context.left)
        guard from < context.right else { return [] }
        return complement(of: link, in: from, context.right).map { gap in
            var label = "not observed"
            if !fresh, let lastStamp, gap.0 >= lastStamp - 1e-6 {
                label = "no data since \(context.clock(lastStamp, seconds: true))"
            }
            return Band(kind: .notObserved, start: gap.0, end: gap.1, label: label)
        }
    }

    /// Left of `retainedSince`, and left of the oldest ring entry once the ring is full.
    private static func historyBands(mirror: RoomMemoryMirror, recorder: LiveRoomTimelineRecorder, _ context: Context) -> [Band] {
        var edge = recorder.retainedSince
        let ring = mirror.events
        if ring.count >= mirror.limits.eventLog, let oldest = ring.map({ seconds($0.event.header.stamp) }).min() {
            edge = Swift.max(edge ?? oldest, oldest)
        }
        guard let edge, edge > context.left else { return [] }
        let hi = Swift.min(edge, context.right)
        return [Band(kind: .historyUnknown, start: context.left, end: hi,
                     label: "history before \(context.clock(edge, seconds: false)) unknown")]
    }

    /// One band per missing seq range, between the stamps of its nearest received neighbours by
    /// seq. Without neighbours, or with inverted ones (a clock jump), a badge instead.
    private static func missingBands(mirror: RoomMemoryMirror, badges: inout [String], _ context: Context) -> [Band] {
        guard case .missing(let pending, let lost) = mirror.eventHistory.coverage else { return [] }
        // The ring is in arrival order (a replay arrives late with low seqs): search it by seq.
        let ring = mirror.events.sorted { $0.seq < $1.seq }
        /// The index of the first entry with seq above `seq`.
        func above(_ seq: UInt64) -> Int {
            var lo = 0
            var hi = ring.count
            while lo < hi {
                let mid = (lo + hi) / 2
                if ring[mid].seq <= seq { lo = mid + 1 } else { hi = mid }
            }
            return lo
        }
        var bands: [Band] = []
        func place(_ range: ClosedRange<UInt64>, lostOnSpark: Bool) {
            let count = range.upperBound - range.lowerBound + 1
            let noun = count == 1 ? "1 transition" : "\(count) transitions"
            let label = lostOnSpark ? "\(noun) lost" : "\(noun) retrieving"
            let below = above(range.lowerBound) - 1
            let after = above(range.upperBound)
            let lower = below >= 0 && ring[below].seq < range.lowerBound ? seconds(ring[below].event.header.stamp) : nil
            let upper = after < ring.count ? seconds(ring[after].event.header.stamp) : nil
            let span: (Double, Double)?
            switch (lower, upper) {
            case let (lo?, hi?): span = lo <= hi ? (lo, hi) : nil
            case let (lo?, nil): span = (lo, Swift.max(lo, context.now))
            case let (nil, hi?): span = (Swift.min(context.left, hi), hi)
            case (nil, nil): span = nil
            }
            guard let span else {
                badges.append("\(label) · time unknown")
                return
            }
            guard let clipped = context.clip(span.0, span.1) else { return }
            bands.append(Band(kind: lostOnSpark ? .lost(count) : .retrieving(count), start: clipped.0, end: clipped.1, label: label))
        }
        for range in lost { place(range, lostOnSpark: true) }
        for range in pending { place(range, lostOnSpark: false) }
        return bands
    }

    /// The parts of `[from, to]` no interval covers. Intervals may overlap or be out of order.
    private static func complement(of intervals: [(Double, Double)], in from: Double, _ to: Double) -> [(Double, Double)] {
        var gaps: [(Double, Double)] = []
        var cursor = from
        for (lo, hi) in intervals.sorted(by: { $0.0 < $1.0 }) where hi >= from && lo <= to {
            if lo > cursor { gaps.append((cursor, Swift.min(lo, to))) }
            cursor = Swift.max(cursor, hi)
            if cursor >= to { break }
        }
        if cursor < to { gaps.append((cursor, to)) }
        return gaps
    }

    // MARK: Items

    /// The active run's events: `eventsById` and `openEvents`, ticks from the ring.
    private static func currentItems(mirror: RoomMemoryMirror, kinds: Set<RoomMemory.ChangeEvent.Kind>, _ context: Context) -> [Item] {
        var identity = mirror.eventsById
        for entry in mirror.openEvents where identity[entry.event.eventId].map({ $0.seq < entry.seq }) ?? true {
            identity[entry.event.eventId] = entry
        }
        let ring = Dictionary(grouping: mirror.events, by: { $0.event.eventId })
        var items: [Item] = []
        for (eventId, latest) in identity where kinds.contains(latest.event.kind) {
            let received = ring[eventId] ?? []
            var bySeq: [UInt64: LiveRoomEvent] = [:]
            for entry in received { bySeq[entry.seq] = entry }
            bySeq[latest.seq] = latest
            let transitions = bySeq.values.sorted { $0.seq < $1.seq }
            let terminal = isTerminal(latest.event.state)
            let id = terminal ? "\(latest.seq)-\(eventId)" : String(eventId)
            let leadIn = received.isEmpty || transitions.first?.event.state != .candidate
            let lastStamp = seconds(latest.event.header.stamp)
            let end = terminal ? lastStamp : Swift.max(lastStamp, context.now)
            if let item = item(id: id, latest: latest, transitions: transitions, leadIn: leadIn, end: end, open: !terminal,
                               earlierRun: false, context) {
                items.append(item)
            }
        }
        return items
    }

    /// Open events of retired runs: dimmed, ending at that run's last stamp, outcome unknown.
    private static func earlierItems(
        mirror: RoomMemoryMirror,
        recorder: LiveRoomTimelineRecorder,
        kinds: Set<RoomMemory.ChangeEvent.Kind>,
        _ context: Context
    ) -> [Item] {
        mirror.archivedOpenEvents.compactMap { archived in
            let entry = archived.entry
            guard kinds.contains(entry.event.kind) else { return nil }
            let own = Swift.max(seconds(entry.event.header.stamp), seconds(entry.event.lastSeen))
            let runEnd = recorder.generations.last(where: { $0.generation == archived.generation })?.last ?? own
            return item(id: "earlier-\(entry.seq)-\(entry.event.eventId)", latest: entry, transitions: [entry],
                        leadIn: true, end: Swift.max(runEnd, seconds(entry.event.header.stamp)),
                        open: false, earlierRun: true, context)
        }
    }

    private static func item(
        id: String,
        latest: LiveRoomEvent,
        transitions: [LiveRoomEvent],
        leadIn: Bool,
        end: Double,
        open: Bool,
        earlierRun: Bool,
        _ context: Context
    ) -> Item? {
        let ticks = transitions.map { Tick(stamp: seconds($0.event.header.stamp), state: $0.event.state, seq: $0.seq) }
        let firstStamp = ticks.first?.stamp ?? seconds(latest.event.header.stamp)
        // `{0, 0}` is a missing first_seen (§3.2): the span starts at the first transition received.
        let firstSeen = isMissing(latest.event.firstSeen) ? firstStamp : Swift.min(seconds(latest.event.firstSeen), firstStamp)
        var segments: [Segment] = []
        if firstSeen < firstStamp {
            let style: Segment.Style = leadIn ? .leadIn : self.style(ticks.first?.state ?? .candidate) ?? .candidate
            segments.append(Segment(start: firstSeen, end: firstStamp, style: style))
        }
        for (index, tick) in ticks.enumerated() {
            guard let style = self.style(tick.state) else { continue }
            let next = index + 1 < ticks.count ? ticks[index + 1].stamp : end
            segments.append(Segment(start: tick.stamp, end: Swift.max(tick.stamp, next), style: style))
        }
        // Out of the window: skip before any formatting (`eventsById` may hold 10 000 entries).
        guard Swift.max(firstSeen, end) >= context.left, firstSeen <= context.right else { return nil }
        var notes: [String] = []
        if leadIn, firstSeen < firstStamp || transitions.first?.event.state != .candidate {
            notes.append("earlier steps not received")
        }
        if earlierRun { notes.append("outcome unknown") }
        let event = latest.event
        var detail = "\(kindWord(event.kind)) · \(stateWord(event.state)) · \(context.clock(firstSeen, seconds: true))"
        detail += open ? " → now" : "–\(context.clock(end, seconds: true))"
        if earlierRun { detail += " · earlier run" }
        if !notes.isEmpty { detail += " · " + notes.joined(separator: " · ") }
        return Item(
            id: id,
            eventId: event.eventId,
            kind: event.kind,
            lane: Lane.of(event.kind),
            state: event.state,
            start: firstSeen,
            end: Swift.max(firstSeen, end),
            open: open,
            earlierRun: earlierRun,
            ticks: ticks,
            segments: segments,
            notes: notes,
            detail: detail,
            selected: false
        )
    }

    /// Candidate dashed, confirmed solid; a terminal state draws nothing after it.
    private static func style(_ state: RoomMemory.ChangeEvent.State) -> Segment.Style? {
        switch state {
        case .candidate: return .candidate
        case .confirmed: return .confirmed
        case .accepted, .reverted, .expired: return nil
        }
    }

    // MARK: Activity

    /// Covered samples paint backward, `(previous, t]`, since `active` describes the windows since
    /// the previous `Activity` (§3.7); the first sample of a coverage interval is a point. Every
    /// other part of the window is unobserved.
    private static func activitySegments(recorder: LiveRoomTimelineRecorder, _ context: Context) -> [ActivitySegment] {
        let intervals = recorder.coverage.sorted { $0.start < $1.start }
        /// The coverage interval holding `t`, by index into `intervals`.
        func interval(of t: Double) -> Int? {
            var lo = 0
            var hi = intervals.count
            while lo < hi {
                let mid = (lo + hi) / 2
                if intervals[mid].start <= t { lo = mid + 1 } else { hi = mid }
            }
            let index = lo - 1
            return index >= 0 && t <= intervals[index].end ? index : nil
        }
        var observed: [ActivitySegment] = []
        var previous: (stamp: Double, interval: Int)?
        for sample in recorder.activity.filter(\.covered).sorted(by: { $0.stamp < $1.stamp }) {
            guard let index = interval(of: sample.stamp) else { continue }
            let from = previous.flatMap { $0.interval == index ? $0.stamp : nil } ?? sample.stamp
            if let clipped = context.clip(from, sample.stamp) {
                observed.append(ActivitySegment(start: clipped.0, end: clipped.1, state: sample.active ? .active : .quiet))
            }
            previous = (sample.stamp, index)
        }
        let gaps = complement(of: observed.map { ($0.start, $0.end) }, in: context.left, context.right)
        var all = observed + gaps.map { ActivitySegment(start: $0.0, end: $0.1, state: .unobserved) }
        all.sort { lhs, rhs in
            if lhs.start != rhs.start { return lhs.start < rhs.start }
            return lhs.end < rhs.end
        }
        // Adjacent segments of one state read as one.
        var merged: [ActivitySegment] = []
        for segment in all {
            if let last = merged.last, last.state == segment.state, segment.start <= last.end {
                merged[merged.count - 1].end = Swift.max(last.end, segment.end)
            } else {
                merged.append(segment)
            }
        }
        return merged
    }

    // MARK: Axis

    /// Labels strictly inside the window at the first step giving 4–8 of them, aligned in local time.
    public static func axisTicks(start: Double, end: Double, timeZone: TimeZone) -> [AxisTick] {
        guard end > start else { return [] }
        let offset = Double(timeZone.secondsFromGMT(for: Date(timeIntervalSince1970: start)))
        func stamps(_ step: TimeInterval) -> [Double] {
            var out: [Double] = []
            var local = (((start + offset) / step).rounded(.down) + 1) * step
            while local - offset < end, out.count <= 8 {
                if local - offset > start { out.append(local - offset) }
                local += step
            }
            return out
        }
        let step = tickSteps.first { (4...8).contains(stamps($0).count) }
            ?? tickSteps.first { stamps($0).count <= 8 }
            ?? tickSteps[tickSteps.count - 1]
        let format = formatter(seconds: step < 60, timeZone: timeZone)
        return stamps(step).map { AxisTick(stamp: $0, label: format.string(from: Date(timeIntervalSince1970: $0))) }
    }

    /// `HH:mm` or `HH:mm:ss` in `timeZone`, `en_US_POSIX`.
    public static func clock(_ stamp: Double, seconds: Bool, timeZone: TimeZone) -> String {
        formatter(seconds: seconds, timeZone: timeZone).string(from: Date(timeIntervalSince1970: stamp))
    }

    private static func formatter(seconds: Bool, timeZone: TimeZone) -> DateFormatter {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = timeZone
        formatter.dateFormat = seconds ? "HH:mm:ss" : "HH:mm"
        return formatter
    }

    // MARK: Geometry

    /// The x of `stamp` on a canvas `width` wide.
    public func x(for stamp: Double, width: Double) -> Double {
        guard end > start else { return 0 }
        return (stamp - start) / (end - start) * width
    }

    /// The selection id of the item in `lane` nearest `x`, within `tolerance` points. Spans that
    /// contain `x` tie on distance 0; the one with a transition nearest `x` wins.
    public func hit(x: Double, lane: Lane, width: Double, tolerance: Double = 4) -> String? {
        var best: (distance: Double, tick: Double, id: String)?
        for item in items where item.lane == lane {
            let x0 = self.x(for: item.start, width: width)
            let x1 = self.x(for: item.end, width: width)
            let distance = x < x0 ? x0 - x : x > x1 ? x - x1 : 0
            guard distance <= tolerance else { continue }
            let marks = item.ticks.map(\.stamp) + [item.start, item.end]
            let tick = marks.map { abs(self.x(for: $0, width: width) - x) }.min() ?? .infinity
            let candidate = (distance, tick, item.id)
            if let current = best, (current.distance, current.tick, current.id) <= candidate { continue }
            best = candidate
        }
        return best?.id
    }

    // MARK: Map halo

    /// A highlight on the map for the selected event, or why there is none.
    public enum Halo: Equatable, Sendable {
        case none
        case marker(LiveRoomMarker)
        case unavailable(String)
    }

    /// Margin the halo adds on each side of the event's box, in meters.
    public static let haloInflation: Float = 0.05
    /// Neutral light grey: not a kind colour, not a change.
    public static let haloColor = SIMD3<Float>(0.82, 0.84, 0.87)

    /// The map highlight for `selection`, only when its coordinates are valid on the map shown.
    public static func halo(selection: String?, mirror: RoomMemoryMirror) -> Halo {
        guard let selection else { return .none }
        if selection.hasPrefix("earlier-") {
            return .unavailable("from an earlier run · location unavailable on this map")
        }
        guard let entry = resolve(selection, mirror: mirror) else {
            return .unavailable("not in this Mac's history")
        }
        guard LiveRoomMarkers.markersAllowed(mirror) else {
            return .unavailable("\(mapUnavailableReason(mirror)) · location unavailable on this map")
        }
        let event = entry.event
        guard let sensor = mapSensor(mirror), event.sensors == sensor else {
            return .unavailable("this sensor's events are in its own frame · location unavailable on this map")
        }
        if let epoch = mirror.mapEpoch, event.header.mapEpoch > epoch {
            return .unavailable("newer than the map shown")
        }
        let margin = SIMD3<Float>(repeating: haloInflation)
        return .marker(LiveRoomMarker(
            id: "halo",
            kind: event.kind,
            corners: corners(min: event.aabb.min - margin, max: event.aabb.max + margin),
            dashed: true,
            fillAlpha: 0,
            onTop: true,
            countsAsChange: false,
            countsAsCandidate: false,
            arrow: nil,
            centroid: event.centroid,
            color: haloColor
        ))
    }

    /// The sensor whose frame `map_bricks` is in. This follows D30's default: the JT128 when the
    /// status reports a JT128 rate, else the Mid-360. A non-default `map_sensor` would need a
    /// status field naming it.
    static func mapSensor(_ mirror: RoomMemoryMirror) -> RoomMemory.Sensors? {
        guard let status = mirror.status else { return nil }
        return status.jt128Hz != nil ? .jt128 : .mid360
    }

    /// The active run's entry a row id names: `eventId` (open) or `seq-eventId` (terminal).
    private static func resolve(_ id: String, mirror: RoomMemoryMirror) -> LiveRoomEvent? {
        let parts = id.split(separator: "-")
        if parts.count == 1, let eventId = UInt64(parts[0]) {
            return mirror.openEvents.first { $0.event.eventId == eventId }
                ?? mirror.eventsById[eventId].flatMap { isTerminal($0.event.state) ? nil : $0 }
        }
        guard parts.count == 2, let seq = UInt64(parts[0]), let eventId = UInt64(parts[1]) else { return nil }
        if let latest = mirror.eventsById[eventId], latest.seq == seq { return latest }
        return mirror.events.first { $0.seq == seq && $0.event.eventId == eventId }
    }

    private static func mapUnavailableReason(_ mirror: RoomMemoryMirror) -> String {
        if !mirror.isStatusFresh { return "status stale" }
        if mirror.mapState != .current { return "map not current" }
        switch mirror.status?.state {
        case .building?: return "building reference"
        case .rigMoved?: return "rig moved"
        default: return "map not current"
        }
    }

    /// The 8 corners in the order `LiveRoomMarkers.edgePairs` and `boxTriangles` index.
    private static func corners(min lo: SIMD3<Float>, max hi: SIMD3<Float>) -> [SIMD3<Float>] {
        [
            SIMD3(lo.x, lo.y, lo.z), SIMD3(hi.x, lo.y, lo.z), SIMD3(hi.x, hi.y, lo.z), SIMD3(lo.x, hi.y, lo.z),
            SIMD3(lo.x, lo.y, hi.z), SIMD3(hi.x, lo.y, hi.z), SIMD3(hi.x, hi.y, hi.z), SIMD3(lo.x, hi.y, hi.z),
        ]
    }

    // MARK: Words

    static func seconds(_ time: RoomMemory.Time) -> Double {
        LiveRoomTimelineRecorder.seconds(time)
    }

    private static func isMissing(_ time: RoomMemory.Time) -> Bool {
        time.sec == 0 && time.nanosec == 0
    }

    private static func isTerminal(_ state: RoomMemory.ChangeEvent.State) -> Bool {
        state == .accepted || state == .reverted || state == .expired
    }

    private static func kindWord(_ kind: RoomMemory.ChangeEvent.Kind) -> String {
        switch kind {
        case .appeared: return "appeared"
        case .disappeared: return "disappeared"
        case .moved: return "moved"
        }
    }

    private static func stateWord(_ state: RoomMemory.ChangeEvent.State) -> String {
        switch state {
        case .candidate: return "candidate"
        case .confirmed: return "confirmed"
        case .accepted: return "accepted"
        case .reverted: return "reverted"
        case .expired: return "expired"
        }
    }
}
