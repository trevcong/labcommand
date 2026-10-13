import Foundation

/// One-line renderings of probe output. The Right Now panel carries a fact;
/// the full transcript stays in the probe log on Hardware.
public enum ProbeText {
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
