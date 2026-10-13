import Foundation
import LabCommandCore

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

    let json = """
    {"BackendState": "Running", "Peer": {"k": {"HostName": "spark-1f3e",
     "TailscaleIPs": ["100.118.187.99"], "Online": true, "Active": true,
     "CurAddr": "192.168.68.51:41641"}}}
    """
    let hops = SparkSSH.hopList(tailscaleJSON: json, lastLAN: nil)
    expect(hops.first == "192.168.68.51", "CurAddr leads, port stripped: \(hops)")
    expect(hops.contains("spark"), "plain spark alias stays as fallback")
}
