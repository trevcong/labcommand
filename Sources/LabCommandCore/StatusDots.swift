import Foundation

/// Spark, Mid-360, and JT128 chips on the top bar.
///
/// The command-center probe (`RigSnapshot`, SSH to :8766) and the room socket
/// (:8767) are different measurements. The chips follow the room socket when it
/// is actually up: Spark turns on `.connected`, and each lidar follows a fresh
/// `per_sensor_hz`. A stale status, or a socket that is not `.connected`, leaves
/// the sensors on the probe, so a dead link cannot keep a dot green.
/// Recording gates and `RigHealth` stay on the probe. This value does not replace `rig`.
public struct StatusDots: Sendable, Equatable {
    /// Spark-chip help when the room socket is up and the probe still says the Spark is down.
    public static let roomLinkProbeDown = "Room link connected, command-center probe down"

    public var sparkReachable: Bool
    /// Nil: the chip keeps the command-center SSH detail. Set only for `roomLinkProbeDown`.
    public var sparkHelp: String?
    public var mid360: SensorReading
    public var jt128: SensorReading

    public init(sparkReachable: Bool, sparkHelp: String?, mid360: SensorReading, jt128: SensorReading) {
        self.sparkReachable = sparkReachable
        self.sparkHelp = sparkHelp
        self.mid360 = mid360
        self.jt128 = jt128
    }

    /// `coverageKnownFraction` is not an input. The caller passes `mirror.isStatusFresh`
    /// (3 s); this function does not read a clock.
    public static func derive(
        rig: RigSnapshot,
        connection: LiveRoomConnection,
        status: LiveRoomStatus?,
        isStatusFresh: Bool
    ) -> StatusDots {
        let roomUp = isRoomConnected(connection)
        // Freshness gates the rates only. The socket itself is enough for the Spark chip.
        let trustRates = roomUp && isStatusFresh
        return StatusDots(
            sparkReachable: roomUp || rig.sparkReachable,
            sparkHelp: (roomUp && !rig.sparkReachable) ? roomLinkProbeDown : nil,
            mid360: sensor(rig.mid360, hz: status?.mid360Hz, trustRates: trustRates),
            jt128: sensor(rig.jt128, hz: status?.jt128Hz, trustRates: trustRates)
        )
    }

    private static func isRoomConnected(_ connection: LiveRoomConnection) -> Bool {
        if case .connected = connection { return true }
        return false
    }

    /// A trusted rate above `streamingFloorHz` is streaming. Zero (and a measured 1,
    /// which is not above the floor) is not. Nil means the node did not measure that sensor.
    private static func sensor(_ probe: SensorReading, hz: UInt16?, trustRates: Bool) -> SensorReading {
        guard trustRates, let hz else { return probe }
        if Double(hz) > RigStatusDecoder.streamingFloorHz {
            // The probe already classified this sensor as streaming, so its imu and
            // clock are real measurements. Only the point rate moves to the room link.
            if probe.state == .streaming {
                var live = probe
                live.pointRateHz = Double(hz)
                live.detailOverride = nil
                return live
            }
            return SensorReading(id: probe.id, state: .streaming, pointRateHz: Double(hz))
        }
        // A probe that already measured dark or powered is more specific than "0 Hz".
        if probe.state == .dark || probe.state == .powered {
            return probe
        }
        var quiet = SensorReading(id: probe.id, state: .powered, pointRateHz: Double(hz))
        quiet.detailOverride = "\(hz) Hz on the room link"
        return quiet
    }
}
