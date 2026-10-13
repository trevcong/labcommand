import Foundation

/// Action legality over a RigSnapshot. Spark-offline must never look ready,
/// and a parked rig (both sensors dark) blocks Record but not Start.
public enum RigGate {
    public static func canStartRig(_ s: RigSnapshot) -> (Bool, String?) {
        if !s.sparkReachable { return (false, "Spark not reachable") }
        if !s.apiReachable { return (false, s.blockReason ?? "Command center not answering") }
        if s.recording.active { return (false, "Recording — stop the take first") }
        return (true, nil)
    }

    public static func canRecord(_ s: RigSnapshot) -> (Bool, String?) {
        if !s.sparkReachable { return (false, "Spark not reachable") }
        if !s.apiReachable { return (false, s.blockReason ?? "Command center not answering") }
        if s.recording.active { return (false, "Already recording") }
        if !s.stackRunning { return (false, "Drivers are not running — Start rig first") }
        if s.bothDark { return (false, "Both sensors are dark — plug in power") }
        if !s.anyStreaming { return (false, "No sensor is producing points yet") }
        return (true, nil)
    }

    public static func canStopRecord(_ s: RigSnapshot) -> (Bool, String?) {
        if !s.apiReachable { return (false, "Command center not answering") }
        if !s.recording.active { return (false, "Not recording") }
        return (true, nil)
    }

    /// Stop is two-stage like v0.5: first stop ends the take, second parks
    /// the stack. Tunnel close is always allowed locally.
    public static func canStopAll(_ s: RigSnapshot) -> (Bool, String?) {
        if s.anythingRunning { return (true, nil) }
        return (false, "Nothing is running")
    }

    public static func canWakeJT(_ s: RigSnapshot) -> (Bool, String?) {
        if !s.apiReachable { return (false, "Command center not answering") }
        if s.jt128.state == .dark { return (false, "JT128 is unpowered") }
        if s.jt128.state == .streaming { return (false, "JT128 is already live") }
        return (true, nil)
    }

    public static func canStandbyJT(_ s: RigSnapshot) -> (Bool, String?) {
        if !s.apiReachable { return (false, "Command center not answering") }
        if s.jt128.state == .dark { return (false, "JT128 is unpowered") }
        if s.recording.active { return (false, "Recording — stop the take first") }
        return (true, nil)
    }

    public static func canArmAtLock(_ s: RigSnapshot) -> (Bool, String?) {
        if !s.sparkReachable { return (false, "Spark not reachable") }
        if !s.apiReachable { return (false, s.blockReason ?? "Command center not answering") }
        if s.recording.active { return (false, "Already recording") }
        if !s.stackRunning { return (false, "Drivers are not running — Start rig first") }
        return (true, nil)
    }
}
