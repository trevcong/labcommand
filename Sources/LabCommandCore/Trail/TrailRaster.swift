import Foundation

/// The map seen from above, binned into square cells. This is the view's downsampling:
/// any number of points becomes at most `maxCells` × `maxCells` counts.
///
/// Two layers. `footprint` counts every point, so the floor and ceiling show the room's
/// extent faintly. `walls` counts only points in a height band around the cart, so walls
/// and furniture stand out and a ceiling does not cover the floor plan. A doubled wall
/// shows as two parallel lines in this layer.
public struct TrailRaster: Equatable, Sendable {
    public var width: Int
    public var height: Int
    /// Metres per cell.
    public var cellSize: Double
    /// World x of the left edge and world y of the top edge. Row 0 is the top (largest y).
    public var minX: Double
    public var maxY: Double
    public var footprint: [UInt32]
    public var walls: [UInt32]
    /// Map z kept in `walls`, metres.
    public var band: ClosedRange<Double>
    /// True when the band follows the cart's height; false when it comes from the map's own z spread.
    public var bandFollowsCart: Bool
    /// Points that landed inside the raster.
    public var binned: Int

    public static let defaultMaxCells = 1024
    public static let minCellSize = 0.02
    /// Margin around the room and the trail, metres.
    public static let margin = 0.5
    /// Wall band half-height around the cart's median z, metres.
    public static let bandHalfHeight = 0.7

    public var worldWidth: Double { Double(width) * cellSize }
    public var worldHeight: Double { Double(height) * cellSize }

    /// Cell coordinates of a world point, continuous: (0, 0) is the top-left corner of the raster.
    public func cell(of p: TrailPoint) -> (x: Double, y: Double) {
        ((p.x - minX) / cellSize, (maxY - p.y) / cellSize)
    }

    public static func build(
        map: [Point3D],
        trajectory: TrailTrajectory?,
        maxCells: Int = defaultMaxCells
    ) -> TrailRaster {
        let poses = trajectory?.poses ?? []
        // Map extent ignores stray returns; the whole trail is always inside.
        var lo = (x: Double.infinity, y: Double.infinity)
        var hi = (x: -Double.infinity, y: -Double.infinity)
        if let bounds = RobustBounds.percentile([map], lower: 0.01, upper: 0.99, maxSamples: 20_000) {
            lo = (Double(bounds.min.x), Double(bounds.min.y))
            hi = (Double(bounds.max.x), Double(bounds.max.y))
        }
        for pose in poses {
            lo = (min(lo.x, pose.position.x), min(lo.y, pose.position.y))
            hi = (max(hi.x, pose.position.x), max(hi.y, pose.position.y))
        }
        if !lo.x.isFinite {
            lo = (-5, -5)
            hi = (5, 5)
        }
        lo = (lo.x - margin, lo.y - margin)
        hi = (hi.x + margin, hi.y + margin)

        let cells = max(maxCells, 1)
        let cell = max(minCellSize, max(hi.x - lo.x, hi.y - lo.y) / Double(cells))
        // A finite trail can still span more than Double holds (±1e308): the ratio is then
        // inf / inf. Take the cap rather than convert NaN to Int.
        func count(_ span: Double) -> Int {
            let n = (span / cell).rounded(.up)
            guard n.isFinite else { return cells }
            return max(1, min(cells, Int(min(n, Double(cells)))))
        }
        let width = count(hi.x - lo.x)
        let height = count(hi.y - lo.y)

        let (band, followsCart) = heightBand(map: map, poses: poses)
        var footprint = [UInt32](repeating: 0, count: width * height)
        var walls = [UInt32](repeating: 0, count: width * height)
        var binned = 0
        for p in map {
            // Bounds-check in Double first: a finite return at 1e20 m is past Int.max as a cell
            // index. The comparisons also fail for NaN, so only in-range values reach Int().
            let fx = ((Double(p.x) - lo.x) / cell).rounded(.down)
            let fy = ((hi.y - Double(p.y)) / cell).rounded(.down)
            guard fx >= 0, fx < Double(width), fy >= 0, fy < Double(height) else { continue }
            let i = Int(fy) * width + Int(fx)
            footprint[i] &+= 1
            if band.contains(Double(p.z)) { walls[i] &+= 1 }
            binned += 1
        }
        return TrailRaster(width: width, height: height, cellSize: cell, minX: lo.x, maxY: hi.y,
                           footprint: footprint, walls: walls, band: band, bandFollowsCart: followsCart,
                           binned: binned)
    }

    /// Cart median z ± `bandHalfHeight` when there is a trail. Without one, the middle 60%
    /// of the map's 2nd–98th percentile height, which drops the floor and ceiling of a room.
    static func heightBand(map: [Point3D], poses: [TrailPose]) -> (ClosedRange<Double>, Bool) {
        if !poses.isEmpty {
            let zs = poses.map(\.position.z).sorted()
            let median = zs[zs.count / 2]
            return ((median - bandHalfHeight)...(median + bandHalfHeight), true)
        }
        guard let bounds = RobustBounds.percentile([map], maxSamples: 20_000) else {
            return ((-Double.infinity)...Double.infinity, false)
        }
        let z0 = Double(bounds.min.z), z1 = Double(bounds.max.z)
        let span = z1 - z0
        return ((z0 + 0.2 * span)...(z1 - 0.2 * span), false)
    }

    /// Premultiplied RGBA8, row 0 at the top. Empty cells are transparent.
    /// Footprint is a faint steel wash; walls go from dim to near-white with density.
    public func rgba() -> [UInt8] {
        let footRef = Self.reference(footprint)
        let wallRef = Self.reference(walls)
        var out = [UInt8](repeating: 0, count: width * height * 4)
        for i in 0..<(width * height) {
            var r = 0.0, g = 0.0, b = 0.0, a = 0.0
            if footprint[i] > 0 {
                let v = Self.shade(footprint[i], footRef)
                let alpha = 0.10 + 0.30 * v
                (r, g, b, a) = (0.290 * alpha, 0.561 * alpha, 0.722 * alpha, alpha)
            }
            if walls[i] > 0 {
                let v = Self.shade(walls[i], wallRef)
                let alpha = 0.40 + 0.60 * v
                let wr = 0.545 + (0.902 - 0.545) * v
                let wg = 0.584 + (0.918 - 0.584) * v
                let wb = 0.631 + (0.937 - 0.631) * v
                // Walls over footprint, premultiplied "over".
                r = wr * alpha + r * (1 - alpha)
                g = wg * alpha + g * (1 - alpha)
                b = wb * alpha + b * (1 - alpha)
                a = alpha + a * (1 - alpha)
            }
            out[i * 4] = UInt8((r * 255).rounded())
            out[i * 4 + 1] = UInt8((g * 255).rounded())
            out[i * 4 + 2] = UInt8((b * 255).rounded())
            out[i * 4 + 3] = UInt8((a * 255).rounded())
        }
        return out
    }

    /// 95th percentile of the non-empty cells, so one dense cell does not wash out the rest.
    static func reference(_ counts: [UInt32]) -> UInt32 {
        var filled = counts.filter { $0 > 0 }
        guard !filled.isEmpty else { return 1 }
        filled.sort()
        return max(1, filled[min(filled.count - 1, Int(Double(filled.count - 1) * 0.95))])
    }

    /// 0…1 on a log scale against `reference`.
    static func shade(_ count: UInt32, _ reference: UInt32) -> Double {
        min(1, log1p(Double(count)) / log1p(Double(reference)))
    }
}
