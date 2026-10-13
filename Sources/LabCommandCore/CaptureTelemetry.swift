import Foundation

/// Optional live figures from `spark-capture.sh status`.
/// Every field is `nil` until that JSON key is present. The UI must print
/// `TelemetryFormat.unavailable` for nil — never `0 Hz`.
public struct CaptureTelemetry: Sendable, Equatable {
    public var pointRateHz: Double?
    public var imuRateHz: Double?
    public var bagBytes: Int?
    public var bagGrowthBps: Double?
    public var writeHealth: String?
    public var driverStatus: String?
    public var fastLioStatus: String?
    public var sparkLoad: Double?
    public var sparkMemPct: Double?
    public var sensorPing: Bool?

    public init(
        pointRateHz: Double? = nil,
        imuRateHz: Double? = nil,
        bagBytes: Int? = nil,
        bagGrowthBps: Double? = nil,
        writeHealth: String? = nil,
        driverStatus: String? = nil,
        fastLioStatus: String? = nil,
        sparkLoad: Double? = nil,
        sparkMemPct: Double? = nil,
        sensorPing: Bool? = nil
    ) {
        self.pointRateHz = pointRateHz
        self.imuRateHz = imuRateHz
        self.bagBytes = bagBytes
        self.bagGrowthBps = bagGrowthBps
        self.writeHealth = writeHealth
        self.driverStatus = driverStatus
        self.fastLioStatus = fastLioStatus
        self.sparkLoad = sparkLoad
        self.sparkMemPct = sparkMemPct
        self.sensorPing = sensorPing
    }

    public static let empty = CaptureTelemetry()

    public static let slotCount = 10

    public var availableCount: Int {
        [
            pointRateHz != nil,
            imuRateHz != nil,
            bagBytes != nil,
            bagGrowthBps != nil,
            writeHealth != nil,
            driverStatus != nil,
            fastLioStatus != nil,
            sparkLoad != nil,
            sparkMemPct != nil,
            sensorPing != nil,
        ].filter { $0 }.count
    }

    public var missingCount: Int { Self.slotCount - availableCount }

    public var hasAnyRate: Bool {
        pointRateHz != nil || imuRateHz != nil || bagGrowthBps != nil
    }
}

/// Display helpers. Nil always becomes the word "unavailable".
public enum TelemetryFormat {
    public static let unavailable = "unavailable"

    public static func hz(_ value: Double?) -> String {
        guard let value else { return unavailable }
        return String(format: "%.1f Hz", value)
    }

    public static func bytes(_ value: Int?) -> String {
        guard let value else { return unavailable }
        if value >= 1_073_741_824 {
            return String(format: "%.2f GB", Double(value) / 1_073_741_824)
        }
        if value >= 1_048_576 {
            return String(format: "%.1f MB", Double(value) / 1_048_576)
        }
        if value >= 1024 {
            return String(format: "%.0f KB", Double(value) / 1024)
        }
        return "\(value) B"
    }

    public static func bps(_ value: Double?) -> String {
        guard let value else { return unavailable }
        if value >= 1_048_576 {
            return String(format: "%.1f MB/s", value / 1_048_576)
        }
        if value >= 1024 {
            return String(format: "%.1f KB/s", value / 1024)
        }
        return String(format: "%.0f B/s", value)
    }

    public static func load(_ value: Double?) -> String {
        guard let value else { return unavailable }
        return String(format: "%.2f", value)
    }

    public static func percent(_ value: Double?) -> String {
        guard let value else { return unavailable }
        return String(format: "%.0f%%", value)
    }

    public static func text(_ value: String?) -> String {
        guard let value, !value.isEmpty else { return unavailable }
        return value
    }

    public static func flag(_ value: Bool?, yes: String, no: String) -> String {
        guard let value else { return unavailable }
        return value ? yes : no
    }
}
