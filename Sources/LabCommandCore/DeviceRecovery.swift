import Foundation

/// Recovery copy for a failed Devices probe. Honest, no invented next hop.
public enum DeviceRecovery {
    public static func guidance(
        probeID: String,
        status: ConnectionStatus,
        detail: String
    ) -> String? {
        switch status {
        case .connected:
            return nil
        case .unknown:
            return unknownGuidance(probeID: probeID, detail: detail)
        case .disconnected, .error:
            return failedGuidance(probeID: probeID, detail: detail)
        }
    }

    public static func guidance(
        probeID: String,
        outcome: CheckOutcome
    ) -> String? {
        guidance(probeID: probeID, status: StatusAggregator.status(for: outcome), detail: outcome.detail)
    }

    private static func failedGuidance(probeID: String, detail: String) -> String {
        switch probeID {
        case "spark_ssh", "Spark":
            return "Open Tailscale on this Mac, then `ssh spark`. A powered Spark is not enough — on its desktop join WiFi and run `tailscale up`. Ethernet is Livox-only. Start sensor stays off until SSH works."
        case "spark_ping", "Ping":
            return "ICMP can fail while SSH works. Prefer the Spark SSH row. Detail: \(detail)"
        case "tailscale", "Tailscale":
            return "Run `tailscale up --accept-routes` on this Mac, then Refresh. Spark is reached over Tailscale, not LAN. Detail: \(detail)"
        case "Mid-360", "lidar", "link":
            return "The Mac cannot see the Livox. On Spark, check \(CaptureRecipe.nic) carrier and ping \(CaptureRecipe.sensorIP). Detail: \(detail)"
        case "Stream", "stream":
            return "Start the sensor after Livox shows linked. Stream runs on Spark, not on this Mac."
        case "Mac":
            return detail
        default:
            return "Refresh after fixing: \(detail)"
        }
    }

    private static func unknownGuidance(probeID: String, detail: String) -> String {
        switch probeID {
        case "spark_ssh", "Spark":
            return "Spark not probed yet. Refresh on Devices. Detail: \(detail)"
        case "tailscale", "Tailscale":
            return "Tailscale CLI missing or not probed. Detail: \(detail)"
        default:
            return "Not probed. Refresh. Detail: \(detail)"
        }
    }
}
