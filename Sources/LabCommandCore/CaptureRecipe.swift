import Foundation

/// Command recipes taken from MID360-BRINGUP.md + DATA-CAPTURE.md.
/// The panel executes these; tests pin the strings so we cannot drift into
/// the serial 2D recorder or PointCloud2 bags.
public enum CaptureRecipe {
    public static let sshHost = "spark"
    public static let sshUser = "tcong"
    public static let sshIdentityFileName = "id_ed25519_spark"
    /// Last-known lab WiFi. Discovery may replace this; do not pin `Host spark` to it.
    public static let lanSSHAddress = "192.168.68.51"
    public static let tailscaleSSHHost = "spark-ts"
    public static let tailscaleIP = "100.118.187.99"
    public static let tailscaleDNS = "spark-1f3e.tail8dfdc7.ts.net"
    public static let sshHostCandidates = ["spark-ts", "spark"]
    public static let sensorIP = "192.168.1.135"
    public static let hostIP = "192.168.1.5"
    public static let nic = "enP7s7"
    public static let container = "livox-slam"
    public static let image = "livox:jazzy"
    public static let workspace = "/home/tcong/livox_ws"
    public static let dataRoot = "/home/tcong/datasets/home_lab"
    public static let foxglovePort = 8765
    public static let dockerStopGraceSeconds = 60
    public static let maxBagBytes: Int = 2_147_483_648

    /// Topics that are replayable into FAST-LIO2. `/Laser_map` is forbidden.
    public static let recordTopics: [String] = [
        "/livox/lidar",
        "/livox/imu",
        "/tf",
        "/tf_static",
        "/Odometry",
        "/path",
        "/cloud_registered",
    ]

    public static let forbiddenTopics: [String] = ["/Laser_map"]

    /// Docker run that starts driver + FAST-LIO2 + foxglove_bridge (bring-up §3).
    public static func streamStartCommand() -> String {
        """
        docker rm -f livox-slam livox-ros >/dev/null 2>&1 || true
        docker run -d --name \(container) --net=host \
          -v \(workspace):/ws \
          -v \(dataRoot):/data \
          -w /ws \
          \(image) bash -c \
          "source /opt/ros/jazzy/setup.sh && source /ws/install/setup.sh && ros2 launch /ws/slam_launch.py"
        """
        .trimmingCharacters(in: .whitespacesAndNewlines)
    }

    /// Clean stop so FAST-LIO can flush PCD (`docker stop -t 60`).
    public static func streamStopCommand() -> String {
        "docker stop -t \(dockerStopGraceSeconds) \(container)"
    }

    /// Local Foxglove tunnel. Mac never talks to the LiDAR.
    public static func viewTunnelArguments(
        host: String = sshHost,
        port: Int = foxglovePort,
        identityFile: String = CaptureRecipe.sshIdentityPath()
    ) -> [String] {
        [
            "-N",
            "-L", "\(port):127.0.0.1:\(port)",
            "-o", "ExitOnForwardFailure=yes",
            "-o", "ServerAliveInterval=30",
            "-o", "BatchMode=yes",
            "-o", "IdentitiesOnly=yes",
            "-i", identityFile,
            "-l", sshUser,
            host,
        ]
    }

    public static func viewConnectURL(port: Int = foxglovePort) -> String {
        "ws://localhost:\(port)"
    }

    /// ros2 bag record line used inside the slam container. Format lock-in is
    /// the driver's `xfer_format=1` / CustomMsg (copied into the sidecar),
    /// not a bag flag. Storage is mcap. No `/Laser_map`.
    public static func recordCommand(
        outputDir: String,
        durationSeconds: Int = 0
    ) -> String {
        var parts: [String] = [
            "ros2", "bag", "record",
            "-s", "mcap",
            "-o", outputDir,
            "--max-bag-size", "\(maxBagBytes)",
        ]
        if durationSeconds > 0 {
            parts += ["-d", "\(durationSeconds)"]
        }
        parts += recordTopics
        return parts.joined(separator: " ")
    }

    public static func containsForbiddenTopic(_ command: String) -> Bool {
        forbiddenTopics.contains { command.contains($0) }
    }

    public static func sshIdentityPath(home: URL = FileManager.default.homeDirectoryForCurrentUser) -> String {
        home.appendingPathComponent(".ssh/\(sshIdentityFileName)").path
    }
}
