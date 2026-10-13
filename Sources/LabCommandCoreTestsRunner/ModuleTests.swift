import Foundation
import LabCommandCore

// Tests for the modules the v1.0 rehaul kept: session library, device
// recovery, tailscale parsing, ssh hop discovery. Rewritten against the same
// behavior the deleted v0.5 suite pinned.

func testSessionLibraryIndexesHouseSession() {
    let dir = SessionLibrary.defaultDirectory()
    guard FileManager.default.fileExists(atPath: dir) else {
        print("  session-index: ~/Desktop/lidar-sessions absent on this Mac — skipped")
        return
    }
    let records = SessionLibrary.index(directory: dir)
    // Index must be stable and derived from files; nothing invented.
    for rec in records {
        expect(!rec.stem.isEmpty, "record has a stem")
        expect(!rec.filename.isEmpty, "record has a filename")
    }
}

func testSessionLibraryFilterCompareAndNotes() {
    let quality = SessionQuality(
        hasPointCloud: true, hasProvenance: false, hasReview: false,
        hasNote: false, pointsKnown: true, ptpKnown: true
    )
    let a = SessionRecord(
        path: "/tmp/a", filename: "a.mcap", stem: "20260828T000000Z_A_baseline",
        bytes: 100, points: 10, fields: "xyz", ptp: false,
        tags: ["A"], notes: "first walk", artifacts: [], quality: quality
    )
    let b = SessionRecord(
        path: "/tmp/b", filename: "b.mcap", stem: "20260828T010000Z_B_furniture_moved",
        bytes: 200, points: 20, fields: "xyz", ptp: true,
        tags: ["B"], notes: "moved a chair", artifacts: [], quality: quality
    )
    expect(SessionLibrary.filter([a, b], query: "chair").map(\.stem) == [b.stem], "query hits notes")
    expect(SessionLibrary.filter([a, b], query: "", tag: "A").map(\.stem) == [a.stem], "tag filters")
    expect(SessionLibrary.filter([a, b], query: "").count == 2, "empty query keeps all")
    let cmp = SessionLibrary.compare(a, b)
    expect(!cmp.rows.isEmpty, "compare produces rows")
}

func testDeviceRecoveryForFailedProbes() {
    let spark = DeviceRecovery.guidance(
        probeID: "spark_ssh", status: .disconnected, detail: "timeout")
    expect(spark != nil, "spark failure gets guidance")
    expect(spark?.lowercased().contains("tailscale") == true, "spark guidance names tailscale")
    let ok = DeviceRecovery.guidance(probeID: "spark_ssh", status: .connected, detail: "ok")
    expect(ok == nil, "connected needs no recovery copy")
}

func testTailscalePeerParserOfflineSpark() {
    let offline = """
    {"BackendState": "Running", "Peer": {"k": {"HostName": "spark-1f3e",
     "TailscaleIPs": ["100.118.187.99"], "Online": false,
     "LastSeen": "2026-08-27T01:00:00Z", "Active": false, "CurAddr": ""}}}
    """
    let peer = TailscaleStatusParser.parse(offline)
    expect(peer.peerListed, "spark peer is listed")
    expect(peer.online == false, "listed is not online")
    let active = offline
        .replacingOccurrences(of: "\"Online\": false", with: "\"Online\": true")
        .replacingOccurrences(of: "\"Active\": false", with: "\"Active\": true")
        .replacingOccurrences(of: "\"CurAddr\": \"\"", with: "\"CurAddr\": \"192.168.68.51:41641\"")
    let up = TailscaleStatusParser.parse(active)
    expect(up.online == true && up.active == true, "active peer parses")
    expect(up.curAddr.contains("192.168.68.51"), "CurAddr preserved")
    let garbage = TailscaleStatusParser.parse("not json")
    expect(garbage.backendState == "unparsed", "garbage → unparsed, not a crash")
}

func testSparkSSHTriesNextHostAfterTimeout() {
    var attempts: [String] = []
    let picked = SparkSSH.firstReachableHost(hosts: ["dead-host", "live-host"]) { host in
        attempts.append(host)
        if host == "dead-host" {
            return ("x", 255, "ssh: connect to host dead-host: Operation timed out", nil)
        }
        return ("ok", 0, "", nil)
    }
    expect(attempts == ["dead-host", "live-host"], "tries hosts in order")
    expect(picked.host == "live-host", "settles on the host that answered")
    expect(picked.result == "ok", "returns that host's result")

    // CurAddr with a port strips to a bare IP and leads the hop list.
    let json = """
    {"BackendState": "Running", "Peer": {"k": {"HostName": "spark-1f3e",
     "TailscaleIPs": ["100.118.187.99"], "Online": true, "Active": true,
     "CurAddr": "192.168.68.51:41641"}}}
    """
    let hops = SparkSSH.hopList(tailscaleJSON: json, lastLAN: nil)
    expect(hops.first == "192.168.68.51", "CurAddr leads, port stripped: \(hops)")
    expect(hops.contains("spark"), "plain spark alias stays as fallback")
}
