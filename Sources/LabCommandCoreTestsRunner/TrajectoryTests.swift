import Foundation
import LabCommandCore

// The route drawn on the deck is the session's own /Odometry track. These pin
// the loader against the real sidecars written next to the bags.

func testTrajectoryLoadsFromRealSession() {
    let root = URL(fileURLWithPath: SessionLibrary.defaultDirectory(), isDirectory: true)
    let tracks = SessionTrajectoryLoader.loadAll(root: root)
    guard let t = tracks.first else {
        // No sidecar on this machine is a legitimate state; nothing to assert.
        print("  (no trajectory sidecars present - skipped)")
        return
    }
    expect(!t.isEmpty, "a loaded track has poses")
    expect(t.points.count == t.frames, "frames matches the point count")
    expect(t.source == "/Odometry", "poses come from odometry, got \(t.source)")
    expect(t.path_m > 0, "walk has length")
    expect(t.bounds != nil, "track has a bounding box")

    let names = Set(t.actions)
    expect(names.contains("straight"), "action codebook carries straight")

    let total = t.actionCounts.reduce(0) { $0 + $1.count }
    expect(total == t.frames, "every pose is in the legend: \(total)/\(t.frames)")
    expect(t.actionCounts.count == t.actions.count,
           "legend keeps unused symbols so coverage stays visible")
}

func testTrajectoryMissingSidecarIsNilNotACrash() {
    let tmp = FileManager.default.temporaryDirectory
        .appendingPathComponent("lc-track-\(UUID().uuidString)", isDirectory: true)
    try? FileManager.default.createDirectory(at: tmp, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: tmp) }

    expect(SessionTrajectoryLoader.load(sessionDirectory: tmp) == nil,
           "a session with no sidecar loads as nil")

    let bad = tmp.appendingPathComponent(SessionTrajectoryLoader.fileName)
    try? "{ not json".data(using: .utf8)!.write(to: bad)
    expect(SessionTrajectoryLoader.load(sessionDirectory: tmp) == nil,
           "a corrupt sidecar loads as nil, never a crash")

    let empty = #"{"session":"x","source":"/Odometry","frames":0,"path_m":0,"hz":5,"actions":[],"tasks":[],"points":[]}"#
    try? empty.data(using: .utf8)!.write(to: bad)
    expect(SessionTrajectoryLoader.load(sessionDirectory: tmp) == nil,
           "an empty track is not worth drawing")
}
