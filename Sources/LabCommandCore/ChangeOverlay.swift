import Foundation

/// One occupancy diff drawn on the room, plus the scrubber that reveals it.
///
/// The diff stays in the sensor frame. `geometry` is what moves corners and
/// centroids into the frame the cloud is drawn in. `ChangeDetection` is not
/// modified here.
public enum ChangeOverlay: Sendable {
    public static let localSessionLine = "Local snapshots. No rig contact."
    public static let localChip = "Local"
    public static let readingLine = "Reading snapshots."
    public static let fillAlpha: Float = 0.28
    public static let emphasizedFillAlpha: Float = 0.85

    /// Twelve edges of a box, indexing `Box.corners`.
    public static let edgePairs: [(Int, Int)] = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]

    public struct RGB: Equatable, Sendable {
        public var r: Float
        public var g: Float
        public var b: Float
    }

    /// Viewer config. Library defaults stay on `ChangeDetection.Config()`.
    public static func snapshotConfig() -> ChangeDetection.Config {
        var config = ChangeDetection.Config()
        config.evidenceOfAbsence = true
        config.minVoxels = 1
        config.minExtent = 1
        return config
    }

    /// Both snapshot directories, or nil when the pair is not ready.
    /// Nil includes a partial configuration. That is not a rig launch.
    public static func snapshotDirs(_ environment: [String: String]) -> (a: String, b: String)? {
        if case let .snapshots(a, b) = LaunchMode.resolve(environment) {
            return (a, b)
        }
        return nil
    }

    public static func color(_ kind: ChangeDetection.Kind) -> RGB {
        switch kind {
        case .appeared:
            return RGB(r: Float(63) / 255, g: Float(163) / 255, b: Float(122) / 255)
        case .disappeared:
            return RGB(r: Float(196) / 255, g: Float(90) / 255, b: Float(90) / 255)
        case .moved:
            return RGB(r: Float(196) / 255, g: Float(160) / 255, b: Float(74) / 255)
        case .unobserved:
            return RGB(r: Float(92) / 255, g: Float(102) / 255, b: Float(112) / 255)
        }
    }

    public static func loadFailure(sensor: SensorID, file: String, reason: String) -> String {
        "Could not load \(sensor.label) \(file) (\(reason))."
    }

    /// `HH:mm:ss` in `timeZone`. A missing stamp uses `missing` (`A` or `B`).
    public static func clock(_ date: Date?, missing: String, timeZone: TimeZone = .current) -> String {
        guard let date else { return missing }
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.dateFormat = "HH:mm:ss"
        formatter.timeZone = timeZone
        return formatter.string(from: date)
    }

    public static func distance(_ a: Point3D, _ b: Point3D) -> Float {
        let dx = a.x - b.x
        let dy = a.y - b.y
        let dz = a.z - b.z
        return (dx * dx + dy * dy + dz * dz).squareRoot()
    }

    /// Appeared and disappeared voxels that the cluster cut dropped.
    /// Unknown voxels are not part of this count.
    public static func rejectedVoxels(_ report: ChangeDetection.Report) -> Int {
        accounted(regions: report.regions, appeared: report.appearedVoxels, disappeared: report.disappearedVoxels)
    }

    // MARK: - Timeline

    /// Fields the scrubber needs from one region. The app fills this from a report.
    public struct Draft: Equatable, Sendable {
        public var kind: ChangeDetection.Kind
        public var voxelCount: Int
        public var centroid: Point3D
        public var boundsMin: Point3D
        public var boundsMax: Point3D
        public var fromVoxelCount: Int?
        public var fromCentroid: Point3D?
        public var fromBoundsMin: Point3D?
        public var fromBoundsMax: Point3D?

        public init(
            kind: ChangeDetection.Kind,
            voxelCount: Int,
            centroid: Point3D,
            boundsMin: Point3D,
            boundsMax: Point3D,
            fromVoxelCount: Int? = nil,
            fromCentroid: Point3D? = nil,
            fromBoundsMin: Point3D? = nil,
            fromBoundsMax: Point3D? = nil
        ) {
            self.kind = kind
            self.voxelCount = voxelCount
            self.centroid = centroid
            self.boundsMin = boundsMin
            self.boundsMax = boundsMax
            self.fromVoxelCount = fromVoxelCount
            self.fromCentroid = fromCentroid
            self.fromBoundsMin = fromBoundsMin
            self.fromBoundsMax = fromBoundsMax
        }
    }

    public struct Event: Equatable, Sendable {
        public var sensor: SensorID
        public var region: ChangeDetection.Region
        public var beforeStamp: Date?
        public var afterStamp: Date?
    }

    public struct Timeline: Equatable, Sendable {
        public var sensor: SensorID
        public var events: [Event]
        public var beforeStamp: Date?
        public var afterStamp: Date?
        public var appearedVoxels: Int
        public var disappearedVoxels: Int
        public var unknownVoxels: Int
        public var rejectedVoxels: Int
    }

    public struct Filter: Equatable, Sendable {
        public var showAppeared: Bool
        public var showDisappeared: Bool
        public var showMoved: Bool
        public var showUnobserved: Bool

        public init(
            showAppeared: Bool = true,
            showDisappeared: Bool = true,
            showMoved: Bool = true,
            showUnobserved: Bool = false
        ) {
            self.showAppeared = showAppeared
            self.showDisappeared = showDisappeared
            self.showMoved = showMoved
            self.showUnobserved = showUnobserved
        }

        public func allows(_ kind: ChangeDetection.Kind) -> Bool {
            switch kind {
            case .appeared: return showAppeared
            case .disappeared: return showDisappeared
            case .moved: return showMoved
            case .unobserved: return showUnobserved
            }
        }

        public mutating func set(_ kind: ChangeDetection.Kind, shown: Bool) {
            switch kind {
            case .appeared: showAppeared = shown
            case .disappeared: showDisappeared = shown
            case .moved: showMoved = shown
            case .unobserved: showUnobserved = shown
            }
        }
    }

    public static func timeline(
        sensor: SensorID,
        report: ChangeDetection.Report,
        beforeStamp: Date?,
        afterStamp: Date?
    ) -> Timeline {
        timeline(
            sensor: sensor,
            regions: report.regions.map(draft),
            appearedVoxels: report.appearedVoxels,
            disappearedVoxels: report.disappearedVoxels,
            unknownVoxels: report.unknownVoxels,
            beforeStamp: beforeStamp,
            afterStamp: afterStamp
        )
    }

    public static func timeline(
        sensor: SensorID,
        regions: [Draft],
        appearedVoxels: Int,
        disappearedVoxels: Int,
        unknownVoxels: Int,
        beforeStamp: Date?,
        afterStamp: Date?
    ) -> Timeline {
        let built = regions.map { region($0) }
        let events = built.map {
            Event(sensor: sensor, region: $0, beforeStamp: beforeStamp, afterStamp: afterStamp)
        }
        return Timeline(
            sensor: sensor,
            events: events,
            beforeStamp: beforeStamp,
            afterStamp: afterStamp,
            appearedVoxels: appearedVoxels,
            disappearedVoxels: disappearedVoxels,
            unknownVoxels: unknownVoxels,
            rejectedVoxels: accounted(regions: built, appeared: appearedVoxels, disappeared: disappearedVoxels)
        )
    }

    /// Earlier after-stamp first. Equal stamps follow `SensorID.allCases`.
    public static func merge(_ parts: [Timeline]) -> Timeline {
        let ordered = parts.sorted { lhs, rhs in
            switch (lhs.afterStamp, rhs.afterStamp) {
            case let (left?, right?):
                if left != right { return left < right }
                return sensorRank(lhs.sensor) < sensorRank(rhs.sensor)
            case (nil, .some):
                return false
            case (.some, nil):
                return true
            case (nil, nil):
                return sensorRank(lhs.sensor) < sensorRank(rhs.sensor)
            }
        }
        var events: [Event] = []
        var appeared = 0
        var disappeared = 0
        var unknown = 0
        var rejected = 0
        var before: Date?
        var after: Date?
        for part in ordered {
            events.append(contentsOf: part.events)
            appeared += part.appearedVoxels
            disappeared += part.disappearedVoxels
            unknown += part.unknownVoxels
            rejected += part.rejectedVoxels
            if let stamp = part.beforeStamp {
                before = before.map { min($0, stamp) } ?? stamp
            }
            if let stamp = part.afterStamp {
                after = after.map { max($0, stamp) } ?? stamp
            }
        }
        return Timeline(
            sensor: ordered.first?.sensor ?? .mid360,
            events: events,
            beforeStamp: before,
            afterStamp: after,
            appearedVoxels: appeared,
            disappearedVoxels: disappeared,
            unknownVoxels: unknown,
            rejectedVoxels: rejected
        )
    }

    public static func filtered(_ timeline: Timeline, _ filter: Filter) -> [Event] {
        timeline.events.filter { filter.allows($0.region.kind) }
    }

    public static func clampedCount(_ count: Int, filteredCount: Int) -> Int {
        guard filteredCount > 0 else { return 0 }
        return min(max(0, count), filteredCount)
    }

    public static func revealed(timeline: Timeline, filter: Filter, count: Int) -> [Event] {
        let events = filtered(timeline, filter)
        let shown = clampedCount(count, filteredCount: events.count)
        return Array(events.prefix(shown))
    }

    // MARK: - Slider and copy

    public struct SliderState: Equatable, Sendable {
        public var enabled: Bool
        public var value: Int
        public var lower: Int
        public var upper: Int
    }

    /// `filteredCount == 0` is disabled, value 0, range `0...1`.
    /// A positive count stays enabled at `count == 0`.
    public static func slider(filteredCount: Int, count: Int) -> SliderState {
        if filteredCount <= 0 {
            return SliderState(enabled: false, value: 0, lower: 0, upper: 1)
        }
        return SliderState(
            enabled: true,
            value: clampedCount(count, filteredCount: filteredCount),
            lower: 0,
            upper: filteredCount
        )
    }

    public struct Chip: Equatable, Sendable, Identifiable {
        public var kind: ChangeDetection.Kind
        public var count: Int
        public var title: String
        public var red: Float
        public var green: Float
        public var blue: Float
        public var id: String { kind.rawValue }
    }

    public static func chips(_ timeline: Timeline) -> [Chip] {
        var counts: [ChangeDetection.Kind: Int] = [:]
        for event in timeline.events {
            counts[event.region.kind, default: 0] += 1
        }
        var kinds: [ChangeDetection.Kind] = [.appeared, .disappeared, .moved]
        if counts[.unobserved, default: 0] > 0 {
            kinds.append(.unobserved)
        }
        return kinds.map { kind in
            let count = counts[kind, default: 0]
            let rgb = color(kind)
            return Chip(
                kind: kind,
                count: count,
                title: "\(kind.rawValue) · \(count)",
                red: rgb.r,
                green: rgb.g,
                blue: rgb.b
            )
        }
    }

    public static func caption(timeline: Timeline, filter: Filter, count: Int) -> String {
        let matched = filtered(timeline, filter)
        let shown = clampedCount(count, filteredCount: matched.count)
        let unknown = counted(timeline.unknownVoxels, "unknown")
        let rejected = counted(timeline.rejectedVoxels, "rejected")
        if timeline.events.isEmpty {
            return "0 regions · \(counted(timeline.appearedVoxels, "appeared")), \(counted(timeline.disappearedVoxels, "disappeared")), \(unknown), \(rejected)"
        }
        if matched.isEmpty {
            return "0 of \(timeline.events.count) regions · all kinds hidden · \(unknown), \(rejected)"
        }
        if shown == 0 {
            return "0 of \(matched.count) regions · \(unknown), \(rejected)"
        }
        let event = matched[shown - 1]
        let region = event.region
        var parts = [event.sensor.label, region.kind.rawValue, plainVoxels(region.voxelCount)]
        if region.kind == .moved, let from = region.fromCentroid {
            parts.append(meters(distance(from, region.centroid)))
        }
        parts.append(unknown)
        parts.append(rejected)
        return parts.joined(separator: " · ")
    }

    public static func meters(_ value: Float) -> String {
        String(format: "%.2f m", locale: Locale(identifier: "en_US_POSIX"), value)
    }

    // MARK: - Geometry

    public struct Segment: Equatable, Sendable {
        public var a: Point3D
        public var b: Point3D
    }

    public struct Box: Equatable, Sendable {
        public var corners: [Point3D]
        public var edges: [Segment]
        public var kind: ChangeDetection.Kind
        public var filled: Bool
        public var emphasized: Bool
        public var color: RGB
        public var fillAlpha: Float
        public var centroid: Point3D
        public var markCentroid: Bool
    }

    public struct Arrow: Equatable, Sendable {
        public var from: Point3D
        public var to: Point3D
        public var left: Point3D
        public var right: Point3D
    }

    public struct Geometry: Equatable, Sendable {
        public var boxes: [Box]
        public var arrows: [Arrow]

        public static let empty = Geometry(boxes: [], arrows: [])
    }

    public static func geometry(
        events: [Event],
        transform: (SensorID) -> RigidTransform,
        emphasizedIndex: Int?
    ) -> Geometry {
        var boxes: [Box] = []
        var arrows: [Arrow] = []
        for (index, event) in events.enumerated() {
            let emphasized = emphasizedIndex == index
            let pose = transform(event.sensor)
            let region = event.region
            let markTo = region.kind != .unobserved
            boxes.append(makeBox(
                min: region.boundsMin,
                max: region.boundsMax,
                transform: pose,
                kind: region.kind,
                filled: markTo,
                emphasized: emphasized,
                centroid: region.centroid,
                mark: markTo
            ))
            guard region.kind == .moved,
                  let fromMin = region.fromBoundsMin,
                  let fromMax = region.fromBoundsMax,
                  let fromCentroid = region.fromCentroid else {
                continue
            }
            boxes.append(makeBox(
                min: fromMin,
                max: fromMax,
                transform: pose,
                kind: .moved,
                filled: false,
                emphasized: emphasized,
                centroid: fromCentroid,
                mark: true
            ))
            if let arrow = makeArrow(from: pose.apply(fromCentroid), to: pose.apply(region.centroid)) {
                arrows.append(arrow)
            }
        }
        return Geometry(boxes: boxes, arrows: arrows)
    }

    // MARK: - Private

    private static func sensorRank(_ sensor: SensorID) -> Int {
        SensorID.allCases.firstIndex(of: sensor) ?? 0
    }

    private static func draft(_ region: ChangeDetection.Region) -> Draft {
        Draft(
            kind: region.kind,
            voxelCount: region.voxelCount,
            centroid: region.centroid,
            boundsMin: region.boundsMin,
            boundsMax: region.boundsMax,
            fromVoxelCount: region.fromVoxelCount,
            fromCentroid: region.fromCentroid,
            fromBoundsMin: region.fromBoundsMin,
            fromBoundsMax: region.fromBoundsMax
        )
    }

    private static func region(_ draft: Draft) -> ChangeDetection.Region {
        ChangeDetection.Region(
            kind: draft.kind,
            voxelCount: draft.voxelCount,
            centroid: draft.centroid,
            boundsMin: draft.boundsMin,
            boundsMax: draft.boundsMax,
            volume: 0,
            minKey: 0,
            fromVoxelCount: draft.fromVoxelCount,
            fromMinKey: nil,
            fromCentroid: draft.fromCentroid,
            fromBoundsMin: draft.fromBoundsMin,
            fromBoundsMax: draft.fromBoundsMax
        )
    }

    private static func accounted(regions: [ChangeDetection.Region], appeared: Int, disappeared: Int) -> Int {
        var appearedKept = 0
        var disappearedKept = 0
        for region in regions {
            switch region.kind {
            case .appeared:
                appearedKept += region.voxelCount
            case .disappeared:
                disappearedKept += region.voxelCount
            case .moved:
                appearedKept += region.voxelCount
                disappearedKept += region.fromVoxelCount ?? 0
            case .unobserved:
                break
            }
        }
        return max(0, appeared - appearedKept) + max(0, disappeared - disappearedKept)
    }

    private static func counted(_ count: Int, _ adjective: String) -> String {
        let noun = count == 1 ? "voxel" : "voxels"
        return "\(count) \(adjective) \(noun)"
    }

    private static func plainVoxels(_ count: Int) -> String {
        count == 1 ? "1 voxel" : "\(count) voxels"
    }

    private static func corners(min lo: Point3D, max hi: Point3D) -> [Point3D] {
        [
            Point3D(x: lo.x, y: lo.y, z: lo.z),
            Point3D(x: hi.x, y: lo.y, z: lo.z),
            Point3D(x: hi.x, y: hi.y, z: lo.z),
            Point3D(x: lo.x, y: hi.y, z: lo.z),
            Point3D(x: lo.x, y: lo.y, z: hi.z),
            Point3D(x: hi.x, y: lo.y, z: hi.z),
            Point3D(x: hi.x, y: hi.y, z: hi.z),
            Point3D(x: lo.x, y: hi.y, z: hi.z),
        ]
    }

    private static func makeBox(
        min lo: Point3D,
        max hi: Point3D,
        transform: RigidTransform,
        kind: ChangeDetection.Kind,
        filled: Bool,
        emphasized: Bool,
        centroid: Point3D,
        mark: Bool
    ) -> Box {
        let placed = corners(min: lo, max: hi).map(transform.apply)
        let edges = edgePairs.map { pair in
            Segment(a: placed[pair.0], b: placed[pair.1])
        }
        let alpha: Float = filled ? (emphasized ? emphasizedFillAlpha : fillAlpha) : 0
        return Box(
            corners: placed,
            edges: edges,
            kind: kind,
            filled: filled,
            emphasized: emphasized,
            color: color(kind),
            fillAlpha: alpha,
            centroid: transform.apply(centroid),
            markCentroid: mark
        )
    }

    private static func makeArrow(from: Point3D, to: Point3D) -> Arrow? {
        let delta = Point3D(x: to.x - from.x, y: to.y - from.y, z: to.z - from.z)
        let len = length(delta)
        if len == 0 { return nil }
        let dir = Point3D(x: delta.x / len, y: delta.y / len, z: delta.z / len)
        let head = min(Float(0.15), len * 0.35)
        var side = cross(dir, Point3D(x: 0, y: 0, z: 1))
        var sideLen = length(side)
        if sideLen < 1e-4 {
            side = cross(dir, Point3D(x: 0, y: 1, z: 0))
            sideLen = length(side)
        }
        if sideLen == 0 { return nil }
        side = Point3D(x: side.x / sideLen, y: side.y / sideLen, z: side.z / sideLen)
        let base = Point3D(
            x: to.x - dir.x * head,
            y: to.y - dir.y * head,
            z: to.z - dir.z * head
        )
        let wing = head * 0.45
        return Arrow(
            from: from,
            to: to,
            left: Point3D(x: base.x + side.x * wing, y: base.y + side.y * wing, z: base.z + side.z * wing),
            right: Point3D(x: base.x - side.x * wing, y: base.y - side.y * wing, z: base.z - side.z * wing)
        )
    }

    private static func cross(_ a: Point3D, _ b: Point3D) -> Point3D {
        Point3D(
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x
        )
    }

    private static func length(_ point: Point3D) -> Float {
        (point.x * point.x + point.y * point.y + point.z * point.z).squareRoot()
    }
}
