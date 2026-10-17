import Foundation

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

    /// One-line renderings of probe output. The Right Now panel carries a fact;
    /// the full transcript stays in the probe log on Hardware.
    /// Merged from deleted ProbeText module.
    public static func condense(_ s: String, limit: Int = 64) -> String {
        let one = s.split(whereSeparator: \.isNewline)
            .map { $0.trimmingCharacters(in: .whitespaces) }
            .joined(separator: " ")
        let l = one.lowercased()
        if l.contains("operation timed out") { return "ssh timed out" }
        if l.contains("connection refused") { return "ssh refused" }
        if l.contains("permission denied") { return "ssh key rejected" }
        if l.contains("could not resolve") { return "host not resolving" }
        if one.count <= limit { return one }
        return String(one.prefix(limit - 1)) + "\u{2026}"
    }
}
