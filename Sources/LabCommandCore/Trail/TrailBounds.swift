import Foundation

/// The one rule for absurd values in session files. `parseTUM` rejects a pose line that breaks
/// it; the PCD reader drops and counts a point that breaks it, as it does a NaN point.
///
/// Provisional bounds, approved as provisional by Trevor's delegate (revisit after real data);
/// every message that quotes one says so:
/// - time: |t| ≤ 4,102,444,800 s, the Unix time of 2100-01-01. Relative stamps (0, 912 s) and
///   epoch stamps (1.79e9 s) both pass; millisecond or nanosecond stamps (1.7e12, 1.7e18) fail,
///   which is right because the contract says seconds.
/// - position: |x|, |y|, |z| ≤ 10 km. The map frame starts at the first pose, and a room or
///   building map spans tens of metres.
///
/// Every number derived from values inside these bounds (durations, path lengths, cell
/// indices, playhead fractions) stays far inside Int and Double, so the readout and view can
/// convert them without their own checks.
public enum TrailBounds {
    public static let maxAbsTime = 4_102_444_800.0
    public static let maxAbsCoordinate = 10_000.0

    /// Why `t` is not a usable timestamp, or nil when it is.
    public static func timeProblem(_ t: Double) -> String? {
        guard t.isFinite else { return "timestamp \(t) is not a finite number" }
        guard abs(t) <= maxAbsTime else {
            return "timestamp \(t) s is outside ±\(Int(maxAbsTime)) s (year 2100, provisional bound); seconds expected, not ms or ns"
        }
        return nil
    }

    /// Why `value` is not a usable map-frame coordinate, or nil when it is.
    public static func coordinateProblem(_ axis: String, _ value: Double) -> String? {
        guard value.isFinite else { return "\(axis) \(value) is not a finite number" }
        guard abs(value) <= maxAbsCoordinate else {
            return "\(axis) \(value) m is outside ±\(Int(maxAbsCoordinate)) m (provisional bound)"
        }
        return nil
    }

    public static func contains(_ p: Point3D) -> Bool {
        p.x.isFinite && p.y.isFinite && p.z.isFinite
            && abs(Double(p.x)) <= maxAbsCoordinate
            && abs(Double(p.y)) <= maxAbsCoordinate
            && abs(Double(p.z)) <= maxAbsCoordinate
    }
}
