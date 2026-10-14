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
    /// Horizontal range (m, √(x² + y²) from the rig origin) past which returns are
    /// left out of the camera fit: the room is inside it, the far field isn't.
    public static let maxFitRange: Float = 15

    /// Per-axis `lower`…`upper` percentiles over every cloud together.
    ///
    /// - Parameters:
    ///   - clouds: Point arrays (e.g. one per sensor), sampled with one shared
    ///     stride so each contributes in proportion to its size.
    ///   - lower: Fraction in [0, 1] for the low edge, e.g. 0.02.
    ///   - upper: Fraction in [0, 1] for the high edge, e.g. 0.98.
    ///   - maxSamples: Cap on points examined; sampling is a fixed stride, so the
    ///     same cloud always gives the same bounds.
    ///   - maxHorizontalRange: Sampled points farther than this from the origin in
    ///     x/y are dropped first, e.g. `maxFitRange`; if that drops every point the
    ///     whole sample is used. Height doesn't count toward the range.
    /// - Returns: nil when no point is finite.
    public static func percentile(
        _ clouds: [[Point3D]],
        lower: Float = defaultLower,
        upper: Float = defaultUpper,
        maxSamples: Int = defaultMaxSamples,
        maxHorizontalRange: Float = .infinity
    ) -> (min: Point3D, max: Point3D)? {
        let total = clouds.reduce(0) { $0 + $1.count }
        guard total > 0 else { return nil }
        let stride = maxSamples > 0 && total > maxSamples ? (total - 1) / maxSamples + 1 : 1

        var sample: [Point3D] = []
        sample.reserveCapacity((total - 1) / stride + 1)

        // One running index across clouds so the stride doesn't restart per cloud.
        var next = 0
        var base = 0
        for cloud in clouds {
            while next < base + cloud.count {
                let p = cloud[next - base]
                next += stride
                guard p.x.isFinite, p.y.isFinite, p.z.isFinite else { continue }
                sample.append(p)
            }
            base += cloud.count
        }
        guard !sample.isEmpty else { return nil }

        if maxHorizontalRange < .infinity {
            let limit = maxHorizontalRange * maxHorizontalRange
            let inRange = sample.filter { $0.x * $0.x + $0.y * $0.y <= limit }
            if !inRange.isEmpty { sample = inRange }
        }

        let lo = Swift.min(Swift.max(Swift.min(lower, upper), 0), 1)
        let hi = Swift.min(Swift.max(Swift.max(lower, upper), 0), 1)
        let usePercentiles = sample.count >= minSamplesForPercentiles
        func range(_ axis: KeyPath<Point3D, Float>) -> (Float, Float) {
            var values = sample.map { $0[keyPath: axis] }
            guard usePercentiles else { return (values.min()!, values.max()!) }
            return (quantile(&values, lo), quantile(&values, hi))
        }
        let (x0, x1) = range(\.x)
        let (y0, y1) = range(\.y)
        let (z0, z1) = range(\.z)
        return (Point3D(x: x0, y: y0, z: z0), Point3D(x: x1, y: y1, z: z1))
    }

    /// The `q` quantile, linearly interpolated between closest ranks, found by
    /// selection instead of a full sort. Reorders `values`.
    static func quantile(_ values: inout [Float], _ q: Float) -> Float {
        let position = q * Float(values.count - 1)
        let below = Int(position.rounded(.down))
        let t = position - Float(below)
        let low = select(&values, below)
        guard t > 0, below + 1 < values.count else { return low }
        // Everything past `below` is ≥ `low`; the smallest of it is the next rank.
        let high = values[(below + 1)...].min()!
        return low + (high - low) * t
    }

    /// Hoare quickselect: puts the `k`th smallest value at `values[k]`, smaller or
    /// equal values before it and larger or equal after. O(n) on average.
    static func select(_ values: inout [Float], _ k: Int) -> Float {
        var lo = 0
        var hi = values.count - 1
        while lo < hi {
            let pivot = values[(lo + hi) / 2]
            var i = lo
            var j = hi
            while i <= j {
                while values[i] < pivot { i += 1 }
                while values[j] > pivot { j -= 1 }
                if i <= j {
                    values.swapAt(i, j)
                    i += 1
                    j -= 1
                }
            }
            if k <= j {
                hi = j
            } else if k >= i {
                lo = i
            } else {
                break
            }
        }
        return values[k]
    }
}
