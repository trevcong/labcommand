import Foundation

/// Camera-fit bounds that ignore stray returns. A lidar cloud always holds a few
/// points far past the room (reflections, a door left open, sensor noise), and a
/// raw min/max fit frames those instead of the room. Per-axis percentiles of a
/// strided sample keep the room filling the view and cost the same on any cloud.
public enum RobustBounds {
    public static let defaultLower: Float = 0.02
    public static let defaultUpper: Float = 0.98
    public static let defaultMaxSamples = 4096
    /// Below this many finite points a percentile says little; use plain min/max.
    public static let minSamplesForPercentiles = 32

    /// Per-axis `lower`…`upper` percentiles over every cloud together.
    ///
    /// - Parameters:
    ///   - clouds: Point arrays (e.g. one per sensor), sampled with one shared
    ///     stride so each contributes in proportion to its size.
    ///   - lower: Fraction in [0, 1] for the low edge, e.g. 0.02.
    ///   - upper: Fraction in [0, 1] for the high edge, e.g. 0.98.
    ///   - maxSamples: Cap on points examined; sampling is a fixed stride, so the
    ///     same cloud always gives the same bounds.
    /// - Returns: nil when no point is finite.
    public static func percentile(
        _ clouds: [[Point3D]],
        lower: Float = defaultLower,
        upper: Float = defaultUpper,
        maxSamples: Int = defaultMaxSamples
    ) -> (min: Point3D, max: Point3D)? {
        let total = clouds.reduce(0) { $0 + $1.count }
        guard total > 0 else { return nil }
        let stride = maxSamples > 0 && total > maxSamples ? (total - 1) / maxSamples + 1 : 1

        var xs: [Float] = []
        var ys: [Float] = []
        var zs: [Float] = []
        let capacity = (total - 1) / stride + 1
        xs.reserveCapacity(capacity)
        ys.reserveCapacity(capacity)
        zs.reserveCapacity(capacity)

        // One running index across clouds so the stride doesn't restart per cloud.
        var next = 0
        var base = 0
        for cloud in clouds {
            while next < base + cloud.count {
                let p = cloud[next - base]
                next += stride
                guard p.x.isFinite, p.y.isFinite, p.z.isFinite else { continue }
                xs.append(p.x)
                ys.append(p.y)
                zs.append(p.z)
            }
            base += cloud.count
        }
        guard !xs.isEmpty else { return nil }

        let lo = Swift.min(Swift.max(Swift.min(lower, upper), 0), 1)
        let hi = Swift.min(Swift.max(Swift.max(lower, upper), 0), 1)
        let usePercentiles = xs.count >= minSamplesForPercentiles
        func range(_ values: inout [Float]) -> (Float, Float) {
            values.sort()
            guard usePercentiles else { return (values[0], values[values.count - 1]) }
            return (quantile(values, lo), quantile(values, hi))
        }
        let (x0, x1) = range(&xs)
        let (y0, y1) = range(&ys)
        let (z0, z1) = range(&zs)
        return (Point3D(x: x0, y: y0, z: z0), Point3D(x: x1, y: y1, z: z1))
    }

    /// Linear interpolation between closest ranks of an ascending array.
    static func quantile(_ sorted: [Float], _ q: Float) -> Float {
        let position = q * Float(sorted.count - 1)
        let below = Int(position.rounded(.down))
        let above = Swift.min(below + 1, sorted.count - 1)
        let t = position - Float(below)
        return sorted[below] + (sorted[above] - sorted[below]) * t
    }
}
