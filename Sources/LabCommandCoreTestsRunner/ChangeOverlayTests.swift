import Foundation
import LabCommandCore

private func near(_ a: Float, _ b: Float, _ msg: String) {
    expect(abs(a - b) < 1e-4, "\(msg): \(a) vs \(b)")
}

private func draft(
    _ kind: ChangeDetection.Kind,
    voxels: Int = 1,
    centroid: Point3D = Point3D(x: 1, y: 2, z: 3),
    boundsMin: Point3D = Point3D(x: 1, y: 2, z: 3),
    boundsMax: Point3D = Point3D(x: 1.05, y: 2.05, z: 3.05),
    fromCentroid: Point3D? = nil,
    fromBoundsMin: Point3D? = nil,
    fromBoundsMax: Point3D? = nil,
    fromVoxelCount: Int? = nil
) -> ChangeOverlay.Draft {
    ChangeOverlay.Draft(
        kind: kind,
        voxelCount: voxels,
        centroid: centroid,
        boundsMin: boundsMin,
        boundsMax: boundsMax,
        fromVoxelCount: fromVoxelCount,
        fromCentroid: fromCentroid,
        fromBoundsMin: fromBoundsMin,
        fromBoundsMax: fromBoundsMax
    )
}

private func oneTimeline(
    sensor: SensorID = .jt128,
    regions: [ChangeOverlay.Draft],
    appeared: Int = 0,
    disappeared: Int = 0,
    unknown: Int = 0,
    before: Date? = nil,
    after: Date? = nil
) -> ChangeOverlay.Timeline {
    ChangeOverlay.timeline(
        sensor: sensor,
        regions: regions,
        appearedVoxels: appeared,
        disappearedVoxels: disappeared,
        unknownVoxels: unknown,
        beforeStamp: before,
        afterStamp: after
    )
}

func testChangeOverlayConfig() {
    let config = ChangeOverlay.snapshotConfig()
    var expected = ChangeDetection.Config()
    expected.evidenceOfAbsence = true
    expected.minVoxels = 1
    expected.minExtent = 1
    expect(config == expected, "snapshot config changes evidence, minVoxels, and minExtent only")
    do {
        let report = try ChangeDetection.compare(PointCloudFrame(), PointCloudFrame(), config: config)
        expect(report.regions.isEmpty, "empty clouds stay empty")
    } catch {
        expect(false, "snapshot config was rejected: \(error)")
    }
}

func testChangeOverlayMovedThroughViewerConfig() {
    let before = PointCloudFrame(positions: [Point3D(x: 1.025, y: 0.025, z: 0.025)])
    let after = PointCloudFrame(positions: [Point3D(x: 2.025, y: 0.025, z: 0.025)])
    let report: ChangeDetection.Report
    do {
        report = try ChangeDetection.compare(before, after, config: ChangeOverlay.snapshotConfig())
    } catch {
        expect(false, "compare threw: \(error)")
        return
    }
    expect(report.regions.count == 1, "one region, got \(report.regions.count)")
    expect(report.regions.first?.kind == .moved, "the pair is a move")
    let timeline = ChangeOverlay.timeline(
        sensor: .jt128,
        report: report,
        beforeStamp: nil,
        afterStamp: nil
    )
    let revealed = ChangeOverlay.revealed(timeline: timeline, filter: ChangeOverlay.Filter(), count: timeline.events.count)
    expect(revealed.count == 1, "full count reveals the move")
    let geometry = ChangeOverlay.geometry(events: revealed, transform: { _ in .identity }, emphasizedIndex: 0)
    expect(geometry.boxes.count == 2, "a move draws the from box and the to box")
    expect(geometry.arrows.count == 1, "a separated move draws one arrow")
    guard let region = revealed.first?.region, let from = region.fromCentroid, let arrow = geometry.arrows.first else {
        expect(false, "move is missing a from centroid or an arrow")
        return
    }
    expect(arrow.from == from, "arrow from is the from centroid")
    expect(arrow.to == region.centroid, "arrow to is the appeared centroid")
    expect(geometry.boxes[0].filled && geometry.boxes[0].fillAlpha == ChangeOverlay.emphasizedFillAlpha, "playhead fill is emphasized")
    expect(!geometry.boxes[1].filled, "the from box is edges only")
}

func testChangeOverlayOrderAndFilter() {
    let stamp = Date(timeIntervalSince1970: 1_700_000_000)
    let earlier = stamp
    let later = stamp.addingTimeInterval(10)
    let mid = oneTimeline(
        sensor: .mid360,
        regions: [draft(.moved, voxels: 2), draft(.disappeared)],
        after: earlier
    )
    let jt = oneTimeline(sensor: .jt128, regions: [draft(.appeared), draft(.unobserved)], after: later)
    let merged = ChangeOverlay.merge([jt, mid])
    expect(merged.events.map(\.sensor) == [.mid360, .mid360, .jt128, .jt128], "earlier after-stamp comes first")
    expect(merged.events.map(\.region.kind) == [.moved, .disappeared, .appeared, .unobserved], "report order is kept inside a sensor")

    let tiedMid = oneTimeline(sensor: .mid360, regions: [draft(.appeared)], after: stamp)
    let tiedJT = oneTimeline(sensor: .jt128, regions: [draft(.disappeared)], after: stamp)
    let tied = ChangeOverlay.merge([tiedJT, tiedMid])
    expect(tied.events.map(\.sensor) == [.mid360, .jt128], "equal stamps follow SensorID.allCases")

    let filterTimeline = oneTimeline(
        sensor: .jt128,
        regions: [draft(.appeared), draft(.moved), draft(.disappeared), draft(.unobserved)]
    )
    let filter = ChangeOverlay.Filter()
    expect(!filter.showUnobserved, "unobserved starts hidden")
    expect(ChangeOverlay.revealed(timeline: filterTimeline, filter: filter, count: 0).isEmpty, "count 0 draws nothing")
    let two = ChangeOverlay.revealed(timeline: filterTimeline, filter: filter, count: 2)
    expect(two.map(\.region.kind) == [.appeared, .moved], "count 2 is the first two enabled events")
    var hiddenAppeared = filter
    hiddenAppeared.showAppeared = false
    let rest = ChangeOverlay.revealed(timeline: filterTimeline, filter: hiddenAppeared, count: 10)
    expect(rest.map(\.region.kind) == [.moved, .disappeared], "dropping appeared keeps the other enabled kinds in order")
    expect(!rest.contains { $0.region.kind == .unobserved }, "default filter hides unobserved")
}

func testChangeOverlayBoxesArrowsColors() {
    let lo = Point3D(x: 1, y: 2, z: 3)
    let hi = Point3D(x: 1.4, y: 2.2, z: 3.1)
    let boxTimeline = oneTimeline(
        regions: [draft(.appeared, centroid: Point3D(x: 1.2, y: 2.1, z: 3.05), boundsMin: lo, boundsMax: hi)]
    )
    let identity = ChangeOverlay.geometry(
        events: boxTimeline.events,
        transform: { _ in .identity },
        emphasizedIndex: nil
    )
    guard let box = identity.boxes.first else {
        expect(false, "identity box missing")
        return
    }
    let expected = [
        Point3D(x: lo.x, y: lo.y, z: lo.z),
        Point3D(x: hi.x, y: lo.y, z: lo.z),
        Point3D(x: hi.x, y: hi.y, z: lo.z),
        Point3D(x: lo.x, y: hi.y, z: lo.z),
        Point3D(x: lo.x, y: lo.y, z: hi.z),
        Point3D(x: hi.x, y: lo.y, z: hi.z),
        Point3D(x: hi.x, y: hi.y, z: hi.z),
        Point3D(x: lo.x, y: hi.y, z: hi.z),
    ]
    expect(box.corners == expected, "eight corners are the bound combinations")
    expect(box.edges.count == ChangeOverlay.edgePairs.count, "twelve edges")
    for (edge, pair) in zip(box.edges, ChangeOverlay.edgePairs) {
        expect(edge.a == box.corners[pair.0] && edge.b == box.corners[pair.1], "edge \(pair) connects those corners")
    }
    expect(box.fillAlpha == ChangeOverlay.fillAlpha, "a quiet box uses the low fill")

    let mid = ChangeOverlay.geometry(
        events: oneTimeline(sensor: .mid360, regions: [draft(.appeared, boundsMin: lo, boundsMax: hi)]).events,
        transform: RigTransforms.toWorld,
        emphasizedIndex: nil
    )
    expect(mid.boxes.first?.corners.first == RigTransforms.livoxToWorld.apply(lo), "Mid-360 corner uses livoxToWorld")
    expect(RigTransforms.toWorld(.jt128) == .identity, "JT128 world transform is identity")
    let jt = ChangeOverlay.geometry(
        events: oneTimeline(sensor: .jt128, regions: [draft(.appeared, boundsMin: lo, boundsMax: hi)]).events,
        transform: RigTransforms.toWorld,
        emphasizedIndex: nil
    )
    expect(jt.boxes.first?.corners.first == lo, "JT128 corner stays in sensor meters")

    let from = Point3D(x: 0, y: 0, z: 0)
    let to = Point3D(x: 1, y: 0, z: 0)
    let moved = oneTimeline(regions: [draft(
        .moved,
        voxels: 4,
        centroid: to,
        boundsMin: Point3D(x: 0.9, y: -0.05, z: -0.05),
        boundsMax: Point3D(x: 1.1, y: 0.05, z: 0.05),
        fromCentroid: from,
        fromBoundsMin: Point3D(x: -0.05, y: -0.05, z: -0.05),
        fromBoundsMax: Point3D(x: 0.05, y: 0.05, z: 0.05),
        fromVoxelCount: 4
    )])
    let arrowGeometry = ChangeOverlay.geometry(events: moved.events, transform: { _ in .identity }, emphasizedIndex: 0)
    guard let arrow = arrowGeometry.arrows.first else {
        expect(false, "horizontal move has no arrow")
        return
    }
    let len = ChangeOverlay.distance(arrow.from, arrow.to)
    let wingMid = Point3D(
        x: (arrow.left.x + arrow.right.x) * 0.5,
        y: (arrow.left.y + arrow.right.y) * 0.5,
        z: (arrow.left.z + arrow.right.z) * 0.5
    )
    near(ChangeOverlay.distance(arrow.to, wingMid), min(Float(0.15), len * 0.35), "head length")
    expect(ChangeOverlay.distance(arrow.left, arrow.to) < ChangeOverlay.distance(arrow.left, arrow.from), "left wing sits by the tip")
    expect(ChangeOverlay.distance(arrow.right, arrow.to) < ChangeOverlay.distance(arrow.right, arrow.from), "right wing sits by the tip")

    let vertical = oneTimeline(regions: [draft(
        .moved,
        centroid: Point3D(x: 0, y: 0, z: 1),
        fromCentroid: Point3D(x: 0, y: 0, z: 0),
        fromBoundsMin: Point3D(x: 0, y: 0, z: 0),
        fromBoundsMax: Point3D(x: 0.05, y: 0.05, z: 0.05),
        fromVoxelCount: 1
    )])
    expect(
        ChangeOverlay.geometry(events: vertical.events, transform: { _ in .identity }, emphasizedIndex: nil).arrows.count == 1,
        "a vertical move still has an arrow"
    )

    let same = oneTimeline(regions: [draft(
        .moved,
        centroid: Point3D(x: 1, y: 1, z: 1),
        fromCentroid: Point3D(x: 1, y: 1, z: 1),
        fromBoundsMin: Point3D(x: 0, y: 0, z: 0),
        fromBoundsMax: Point3D(x: 0.05, y: 0.05, z: 0.05),
        fromVoxelCount: 1
    )])
    let sameGeometry = ChangeOverlay.geometry(events: same.events, transform: { _ in .identity }, emphasizedIndex: nil)
    expect(sameGeometry.boxes.count == 2 && sameGeometry.arrows.isEmpty, "a zero-length move has two boxes and no arrow")

    let appeared = ChangeOverlay.geometry(
        events: oneTimeline(regions: [draft(.appeared)]).events,
        transform: { _ in .identity },
        emphasizedIndex: nil
    )
    expect(appeared.boxes.count == 1 && appeared.arrows.isEmpty, "an appearance has no arrow")

    let missingFrom = oneTimeline(regions: [draft(
        .moved,
        fromCentroid: Point3D(x: 0, y: 0, z: 0),
        fromBoundsMin: nil,
        fromBoundsMax: Point3D(x: 0.05, y: 0.05, z: 0.05),
        fromVoxelCount: 1
    )])
    let missingGeometry = ChangeOverlay.geometry(events: missingFrom.events, transform: { _ in .identity }, emphasizedIndex: nil)
    expect(missingGeometry.boxes.count == 1 && missingGeometry.arrows.isEmpty, "a move without from bounds is one box")

    let appearedColor = ChangeOverlay.color(.appeared)
    expect(appearedColor.r == Float(63) / 255 && appearedColor.g == Float(163) / 255 && appearedColor.b == Float(122) / 255, "appeared green")
    let disappearedColor = ChangeOverlay.color(.disappeared)
    expect(disappearedColor.r == Float(196) / 255 && disappearedColor.g == Float(90) / 255 && disappearedColor.b == Float(90) / 255, "disappeared red")
    let movedColor = ChangeOverlay.color(.moved)
    expect(movedColor.r == Float(196) / 255 && movedColor.g == Float(160) / 255 && movedColor.b == Float(74) / 255, "moved amber")
    let unknownColor = ChangeOverlay.color(.unobserved)
    expect(unknownColor.r == Float(92) / 255 && unknownColor.g == Float(102) / 255 && unknownColor.b == Float(112) / 255, "unobserved faint")
}

func testChangeOverlayCaptions() {
    let moved = oneTimeline(
        sensor: .jt128,
        regions: [draft(
            .moved,
            voxels: 7,
            centroid: Point3D(x: 1, y: 0, z: 0),
            fromCentroid: Point3D(x: 0, y: 0, z: 0),
            fromBoundsMin: Point3D(x: 0, y: 0, z: 0),
            fromBoundsMax: Point3D(x: 0.05, y: 0.05, z: 0.05),
            fromVoxelCount: 4
        )],
        appeared: 19,
        disappeared: 4,
        unknown: 854
    )
    let playhead = ChangeOverlay.caption(timeline: moved, filter: ChangeOverlay.Filter(), count: 1)
    expect(
        playhead == "JT128 · moved · 7 voxels · 1.00 m · 854 unknown voxels · 12 rejected voxels",
        "playhead caption was \(playhead)"
    )
    expect(!playhead.contains("cluster"), "unknown voxels are not the cluster cut")

    let singular = oneTimeline(regions: [draft(.appeared, voxels: 1)], appeared: 1, unknown: 1)
    let one = ChangeOverlay.caption(timeline: singular, filter: ChangeOverlay.Filter(), count: 1)
    expect(one.contains("1 voxel"), "one voxel stays singular: \(one)")
    expect(one.contains("1 unknown voxel"), "one unknown voxel stays singular: \(one)")
    expect(!one.contains(" m"), "an appearance has no gap")

    let empty = oneTimeline(regions: [], appeared: 46, disappeared: 14, unknown: 854)
    let emptyLine = ChangeOverlay.caption(timeline: empty, filter: ChangeOverlay.Filter(), count: 0)
    expect(
        emptyLine == "0 regions · 46 appeared voxels, 14 disappeared voxels, 854 unknown voxels, 60 rejected voxels",
        "empty caption was \(emptyLine)"
    )
    expect(!emptyLine.contains("cluster") && !emptyLine.contains("below"), "empty caption does not call unknown a cut")
    expect(emptyLine.contains("854 unknown voxels") && emptyLine.contains("60 rejected voxels"), "unknown and rejected stay distinct")

    let parked = oneTimeline(
        regions: [draft(.appeared), draft(.moved), draft(.disappeared)],
        // 5 appeared pre-cut voxels, 2 kept (the appearance and the move). The
        // disappeared voxel has no pre-cut disappeared count, so rejected is 3.
        appeared: 5,
        unknown: 854
    )
    let parkedLine = ChangeOverlay.caption(timeline: parked, filter: ChangeOverlay.Filter(), count: 0)
    expect(
        parkedLine == "0 of 3 regions · 854 unknown voxels, 3 rejected voxels",
        "count zero caption was \(parkedLine)"
    )
    var hidden = ChangeOverlay.Filter()
    hidden.showAppeared = false
    hidden.showDisappeared = false
    hidden.showMoved = false
    hidden.showUnobserved = false
    let hiddenLine = ChangeOverlay.caption(timeline: parked, filter: hidden, count: 4)
    expect(
        hiddenLine == "0 of 3 regions · all kinds hidden · 854 unknown voxels, 3 rejected voxels",
        "hidden caption was \(hiddenLine)"
    )

    let off = ChangeOverlay.slider(filteredCount: 0, count: 5)
    expect(!off.enabled && off.value == 0 && off.lower == 0 && off.upper == 1, "empty slider is disabled on 0...1")
    let onZero = ChangeOverlay.slider(filteredCount: 3, count: 0)
    expect(onZero.enabled && onZero.value == 0 && onZero.upper == 3, "a nonempty timeline can sit at count zero")
    expect(ChangeOverlay.clampedCount(10, filteredCount: 2) == 2, "stale count clamps to the filtered length")
    expect(ChangeOverlay.clampedCount(-3, filteredCount: 2) == 0, "negative count clamps to zero")
    expect(ChangeOverlay.clampedCount(1, filteredCount: 0) == 0, "an empty filter clamps to zero")

    var calendar = Calendar(identifier: .gregorian)
    calendar.timeZone = TimeZone(secondsFromGMT: 0)!
    let date = calendar.date(from: DateComponents(year: 2026, month: 10, day: 14, hour: 15, minute: 4, second: 5))
    expect(ChangeOverlay.clock(date, missing: "A", timeZone: TimeZone(secondsFromGMT: 0)!) == "15:04:05", "clock is HH:mm:ss")
    expect(ChangeOverlay.clock(nil, missing: "A") == "A" && ChangeOverlay.clock(nil, missing: "B") == "B", "missing stamps read A and B")

    expect(ChangeOverlay.localSessionLine == "Local snapshots. No rig contact.", "local session line")
    expect(ChangeOverlay.localChip == "Local", "local chip")
    expect(ChangeOverlay.readingLine == "Reading snapshots.", "reading line")
    expect(ChangeOverlay.snapshotDirs([:]) == nil, "no variables is not a snapshot pair")
    expect(ChangeOverlay.snapshotDirs(["LC_SNAPSHOT_A": "/a"]) == nil, "one variable is not a pair")
    expect(ChangeOverlay.snapshotDirs(["LC_SNAPSHOT_A": " ", "LC_SNAPSHOT_B": "/b"]) == nil, "whitespace is not a path")
    let dirs = ChangeOverlay.snapshotDirs(["LC_SNAPSHOT_A": "/a", "LC_SNAPSHOT_B": " /b "])
    expect(dirs?.a == "/a" && dirs?.b == "/b", "both paths trim and return")

    let chips = ChangeOverlay.chips(oneTimeline(regions: [draft(.appeared), draft(.unobserved)]))
    expect(chips.map(\.kind) == [.appeared, .disappeared, .moved, .unobserved], "unobserved chip appears only when one exists")
    expect(chips.first?.title == "appeared · 1", "chip title is kind and full count")
    let noUnknownChip = ChangeOverlay.chips(oneTimeline(regions: [draft(.moved)]))
    expect(!noUnknownChip.contains { $0.kind == .unobserved }, "no unobserved chip when the timeline has none")

    var left = oneTimeline(regions: [draft(.appeared)])
    var right = left
    left.events[0].region.volume = 0
    left.events[0].region.minKey = 0
    right.events[0].region.volume = 1.5
    right.events[0].region.minKey = 99
    right.events[0].region.fromMinKey = 7
    expect(left.events[0] == right.events[0], "placeholder volume and minKey are not event identity")
    right.events[0].region.voxelCount += 1
    expect(left.events[0] != right.events[0], "voxel count still participates in equality")
}

private final class TunnelFactoryLog {
    var held: [Int] = []
    var probes: [Int] = []
    var discoveries = 0
    var spawns: [(port: Int, command: ProcessSpawn)] = []
    var remembered: [String] = []
    var invocations: Int { held.count + probes.count + discoveries + spawns.count + remembered.count }
}

private func tunnelFactories(
    log: TunnelFactoryLog,
    hosts: [String],
    portIsOpen: Bool = false,
    stayUp: Bool = true
) -> TunnelGate.Factories {
    TunnelGate.Factories(
        held: { port in
            log.held.append(port)
            return false
        },
        portOpen: { port in
            log.probes.append(port)
            return portIsOpen
        },
        hosts: {
            log.discoveries += 1
            return hosts
        },
        spawn: { port, command in
            log.spawns.append((port, command))
            return TunnelSpawnOutcome(stayedUp: stayUp)
        },
        rememberHost: { log.remembered.append($0) }
    )
}

/// Both public tunnel methods, through the injected factories. An isolated call
/// returns before any factory. Overlapping rig calls share the gate lock.
func testTunnelGateBoundary() {
    let fallbacks = SparkSSH.hopList(tailscaleJSON: nil, lastLAN: nil)
    expect(fallbacks.count >= 2, "blocked tailscale discovery still leaves fallback hosts: \(fallbacks)")
    let first = fallbacks[0]
    expect(first.contains("."), "first fallback hop is dotted: \(first)")

    let isolated = TunnelFactoryLog()
    let apiRefusal = TunnelGate.ensureAPITunnel(
        isolates: true,
        factories: tunnelFactories(log: isolated, hosts: fallbacks, portIsOpen: false, stayUp: true)
    )
    let bothRefusal = TunnelGate.ensureTunnels(
        isolates: true,
        factories: tunnelFactories(log: isolated, hosts: fallbacks, portIsOpen: false, stayUp: true)
    )
    expect(
        apiRefusal == "local session" && bothRefusal == "local session",
        "isolated tunnels refuse before spawn: api=\(apiRefusal ?? "nil") both=\(bothRefusal ?? "nil")"
    )
    expect(
        isolated.invocations == 0,
        "isolated ensureAPITunnel and ensureTunnels invoked \(isolated.invocations) factories"
    )

    let apiLog = TunnelFactoryLog()
    let apiResult = TunnelGate.ensureAPITunnel(
        isolates: false,
        factories: tunnelFactories(log: apiLog, hosts: fallbacks)
    )
    expect(apiResult == nil, "a stayed-up API tunnel is success, got \(apiResult ?? "nil")")
    expect(apiLog.held == [TunnelGate.apiPort], "API tunnel checks the local process: \(apiLog.held)")
    expect(apiLog.probes == [TunnelGate.apiPort], "API tunnel probes \(TunnelGate.apiPort): \(apiLog.probes)")
    expect(apiLog.discoveries == 1, "API tunnel discovers hosts once, got \(apiLog.discoveries)")
    expect(apiLog.spawns.count == 1, "API tunnel must exercise the spawn factory, got \(apiLog.spawns.count)")
    let apiCommand = ProcessSpawn(
        executable: "/usr/bin/ssh",
        arguments: CaptureRecipe.viewTunnelArguments(host: first, port: TunnelGate.apiPort)
    )
    expect(apiLog.spawns.first?.port == TunnelGate.apiPort, "API spawn port")
    expect(apiLog.spawns.first?.command == apiCommand, "API spawn is the ssh forward: \(String(describing: apiLog.spawns.first?.command))")
    expect(apiLog.remembered == [first], "a stayed-up dotted hop is remembered: \(apiLog.remembered)")

    let bothLog = TunnelFactoryLog()
    let bothResult = TunnelGate.ensureTunnels(
        isolates: false,
        factories: tunnelFactories(log: bothLog, hosts: fallbacks)
    )
    expect(bothResult == nil, "both tunnels stay up, got \(bothResult ?? "nil")")
    expect(bothLog.probes == [TunnelGate.apiPort, TunnelGate.viewPort], "ensureTunnels probes API then view: \(bothLog.probes)")
    expect(bothLog.discoveries == 2, "ensureTunnels discovers once per port, got \(bothLog.discoveries)")
    expect(
        bothLog.spawns.map(\.port) == [TunnelGate.apiPort, TunnelGate.viewPort],
        "ensureTunnels spawns both forwards: \(bothLog.spawns.map(\.port))"
    )
    expect(bothLog.spawns.allSatisfy { $0.command.executable == "/usr/bin/ssh" }, "both spawns are ssh")
    expect(bothLog.remembered == [first, first], "each stayed-up hop is remembered: \(bothLog.remembered)")

    let openLog = TunnelFactoryLog()
    let openResult = TunnelGate.ensureAPITunnel(
        isolates: false,
        factories: tunnelFactories(log: openLog, hosts: fallbacks, portIsOpen: true)
    )
    expect(openResult == nil, "an already-open port is success")
    expect(openLog.probes == [TunnelGate.apiPort], "an open port is still probed: \(openLog.probes)")
    expect(openLog.discoveries == 0 && openLog.spawns.isEmpty, "an open port does not discover or spawn")

    let overlap = SpawnOverlap()
    let racing = TunnelGate.Factories(
        held: { _ in false },
        portOpen: { _ in false },
        hosts: { ["10.0.0.8"] },
        spawn: { _, _ in
            overlap.enter()
            Thread.sleep(forTimeInterval: 0.05)
            overlap.leave()
            return TunnelSpawnOutcome(stayedUp: true)
        },
        rememberHost: { _ in }
    )
    let group = DispatchGroup()
    for _ in 0..<2 {
        group.enter()
        DispatchQueue.global(qos: .userInitiated).async {
            _ = TunnelGate.ensureAPITunnel(isolates: false, factories: racing)
            group.leave()
        }
    }
    let finished = group.wait(timeout: .now() + 2)
    expect(finished == .success, "overlapping tunnel calls finished")
    expect(overlap.maxInside == 1, "the gate runs one spawn at a time, saw \(overlap.maxInside)")
    expect(overlap.spawns == 2, "a closed port still lets each caller spawn, saw \(overlap.spawns)")
}

/// Counts how many spawn factories are inside the gate at once.
private final class SpawnOverlap: @unchecked Sendable {
    private let lock = NSLock()
    private var inside = 0
    private(set) var maxInside = 0
    private(set) var spawns = 0

    func enter() {
        lock.lock()
        inside += 1
        if inside > maxInside { maxInside = inside }
        lock.unlock()
    }

    func leave() {
        lock.lock()
        inside -= 1
        spawns += 1
        lock.unlock()
    }
}

func testSnapshotIsolation() {
    var spawned = false
    do {
        _ = try ContactGate.runProcess(
            ProcessSpawn(executable: "/usr/bin/ssh", arguments: ["spark"]),
            isolates: true
        ) {
            spawned = true
            return ContactProcessResult(exitCode: 0, stdout: "LAB_OK", stderr: "")
        }
        expect(false, "an isolated process factory must throw")
    } catch let error as ContactForbidden {
        expect(error.kind == .process, "process refusal kind")
        expect(error.detail.contains("/usr/bin/ssh"), "process refusal names ssh")
    } catch {
        expect(false, "unexpected process error \(error)")
    }
    expect(!spawned, "ssh factory ran while isolated")

    var opened = false
    do {
        try ContactGate.openSocket(SocketOpen(endpoint: "ws://10.0.0.8:8765"), isolates: true) {
            opened = true
        }
        expect(false, "an isolated socket factory must throw")
    } catch let error as ContactForbidden {
        expect(error.kind == .socket, "socket refusal kind")
        expect(error.detail == "ws://10.0.0.8:8765", "socket refusal names the endpoint")
    } catch {
        expect(false, "unexpected socket error \(error)")
    }
    expect(!opened, "socket factory ran while isolated")

    var rigSpawned = false
    let rigResult = try? ContactGate.runProcess(
        ProcessSpawn(executable: "/bin/echo", arguments: ["hi"]),
        isolates: false
    ) {
        rigSpawned = true
        return ContactProcessResult(exitCode: 0, stdout: "hi", stderr: "")
    }
    expect(rigSpawned && rigResult?.stdout == "hi", "rig mode enters the process factory")
    var rigOpened = false
    try? ContactGate.openSocket(SocketOpen(endpoint: "ws://127.0.0.1:9"), isolates: false) {
        rigOpened = true
    }
    expect(rigOpened, "rig mode enters the socket factory")

    func failedMessage(_ environment: [String: String]) -> String? {
        let session = SnapshotSession(environment: environment)
        var rigged = false
        let begun = session.begin { rigged = true }
        expect(!rigged, "partial config must not start the rig: \(environment)")
        if case .load = begun {
            expect(false, "partial config must not load: \(environment)")
        }
        if case .failed(let message) = begun { return message }
        return nil
    }
    expect(
        failedMessage(["LC_SNAPSHOT_A": "/tmp/a-only"]) == "Snapshot configuration error. LC_SNAPSHOT_B is not set.",
        "missing B"
    )
    expect(
        failedMessage(["LC_SNAPSHOT_B": "/tmp/b-only"]) == "Snapshot configuration error. LC_SNAPSHOT_A is not set.",
        "missing A"
    )
    expect(
        failedMessage(["LC_SNAPSHOT_A": "  ", "LC_SNAPSHOT_B": "/tmp/b"]) == "Snapshot configuration error. LC_SNAPSHOT_A is empty.",
        "blank A"
    )
    expect(
        failedMessage(["LC_SNAPSHOT_A": "", "LC_SNAPSHOT_B": ""]) == "Snapshot configuration error. LC_SNAPSHOT_A and LC_SNAPSHOT_B are empty.",
        "both empty"
    )
    expect(
        failedMessage(["LC_SNAPSHOT_A": " ", "LC_SNAPSHOT_B": ""]) == "Snapshot configuration error. LC_SNAPSHOT_A and LC_SNAPSHOT_B are empty.",
        "whitespace and empty"
    )
    expect(
        failedMessage(["LC_SNAPSHOT_A": " "]) == "Snapshot configuration error. LC_SNAPSHOT_A is empty and LC_SNAPSHOT_B is not set.",
        "empty A and missing B"
    )

    let rig = SnapshotSession(environment: [:])
    var started = false
    let rigBegin = rig.begin { started = true }
    expect(started, "rig launch runs the rig closure")
    if case .rig = rigBegin {} else { expect(false, "unset environment is the rig") }

    let missingA = "/lc-overlay-missing-\(UUID().uuidString)/a"
    let missingB = "/lc-overlay-missing-\(UUID().uuidString)/b"
    let missing = SnapshotFiles.load(directoryA: missingA, directoryB: missingB)
    if case .failure(let message) = missing {
        expect(message == "Could not load Mid-360 mid360.bin (missing file).", "missing file caption was \(message)")
    } else {
        expect(false, "missing bins should fail the load")
    }

    guard let root = overlayScratchDirectory() else {
        expect(false, "no writable directory for the decode-failure fixture")
        return
    }
    defer { try? FileManager.default.removeItem(at: root) }
    let directoryA = root.appendingPathComponent("a").path
    let directoryB = root.appendingPathComponent("b").path

    let session = SnapshotSession(environment: ["LC_SNAPSHOT_A": directoryA, "LC_SNAPSHOT_B": directoryB])
    var contacted = false
    let begun = session.begin { contacted = true }
    if case .load(let a, let b) = begun {
        expect(a == directoryA && b == directoryB, "begin returns the snapshot directories")
    } else {
        expect(false, "a full pair should ask for a load")
    }
    expect(!contacted, "begin on snapshots must not start the rig")
    session.refresh(manual: true) { contacted = true }
    session.refresh(manual: false) { contacted = true }
    session.restartTimer { contacted = true }
    session.pollRig { contacted = true }
    for name in ["start rig", "record", "stop", "wake JT128", "standby JT128", "arm", "disarm", "view", "stop view"] {
        session.runAction(name) { contacted = true }
    }
    expect(!contacted, "snapshot mode entered a contact closure")

    let junk = Data("nope".utf8)
    do {
        try junk.write(to: URL(fileURLWithPath: directoryA).appendingPathComponent("mid360.bin"))
        try junk.write(to: URL(fileURLWithPath: directoryB).appendingPathComponent("mid360.bin"))
    } catch {
        expect(false, "could not write the decode-failure fixture: \(error)")
        return
    }
    let garbage = SnapshotFiles.load(directoryA: directoryA, directoryB: directoryB)
    if case .failure(let message) = garbage {
        expect(message.hasPrefix("Could not load Mid-360 mid360.bin ("), "decode failure caption was \(message)")
        expect(!message.contains("unavailable"), "load failure does not say unavailable")
    } else {
        expect(false, "garbage bytes should fail the load")
    }
    session.refresh(manual: true) { contacted = true }
    session.restartTimer { contacted = true }
    session.pollRig { contacted = true }
    session.runAction("start rig") { contacted = true }
    expect(!contacted, "a failed load still isolates contact")

    var ran: [String] = []
    rig.refresh(manual: true) { ran.append("refresh") }
    rig.restartTimer { ran.append("timer") }
    rig.pollRig { ran.append("poll") }
    rig.runAction("start rig") { ran.append("action") }
    expect(ran == ["refresh", "timer", "poll", "action"], "rig mode enters refresh, timer, poll, and action")
}

func runChangeOverlayTests() {
    testChangeOverlayConfig()
    print("  change overlay: viewer config is evidence on, minVoxels 1, minExtent 1"); passed()
    testChangeOverlayMovedThroughViewerConfig()
    print("  change overlay: one moved region, two boxes, arrow on the centroids"); passed()
    testChangeOverlayOrderAndFilter()
    print("  change overlay: merge order and kind filter"); passed()
    testChangeOverlayBoxesArrowsColors()
    print("  change overlay: corners, rig transform, arrow head, colors"); passed()
    testChangeOverlayCaptions()
    print("  change overlay: captions, disabled slider, clocks, snapshot dirs"); passed()
    testTunnelGateBoundary()
    print("  change overlay: isolated tunnel methods invoke zero factories; rig mode spawns"); passed()
    testSnapshotIsolation()
    print("  change overlay: snapshot mode does not enter process, socket, timer, poll, or action"); passed()
}

/// `temporaryDirectory` follows DARWIN_USER_TEMP_DIR, which a read-only sandbox rejects.
private func overlayScratchDirectory() -> URL? {
    let candidates = [
        FileManager.default.temporaryDirectory,
        URL(fileURLWithPath: "/tmp", isDirectory: true),
        URL(fileURLWithPath: FileManager.default.currentDirectoryPath, isDirectory: true)
            .appendingPathComponent("build", isDirectory: true),
    ]
    for base in candidates {
        let root = base.appendingPathComponent("lc-overlay-\(UUID().uuidString)", isDirectory: true)
        let a = root.appendingPathComponent("a", isDirectory: true)
        let b = root.appendingPathComponent("b", isDirectory: true)
        if (try? FileManager.default.createDirectory(at: a, withIntermediateDirectories: true)) != nil,
           (try? FileManager.default.createDirectory(at: b, withIntermediateDirectories: true)) != nil {
            return root
        }
        try? FileManager.default.removeItem(at: root)
    }
    return nil
}
