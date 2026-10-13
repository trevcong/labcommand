import SwiftUI
import LabCommandCore

/// The walk, drawn. Route as a continuous track with every recorded pose on it,
/// coloured by what the robot was doing there. Geometry is the session's own
/// `/Odometry` — nothing is smoothed, resampled, or invented.
struct TrajectoryMap: View {
    let track: SessionTrajectory
    /// Pose the cursor is on, if any.
    var highlight: Int? = nil
    var colorByAction: Bool = true
    var showHeading: Bool = true

    private func tint(_ p: TrajectoryPoint) -> Color {
        guard colorByAction else { return LabTheme.accent }
        switch track.actionName(p) {
        case "hold": return LabTheme.warn
        case "turn_left", "turn_right": return LabTheme.good
        default: return LabTheme.accent
        }
    }

    var body: some View {
        Canvas(rendersAsynchronously: false) { ctx, size in
            guard let b = track.bounds, !track.points.isEmpty else { return }
            let pad: CGFloat = 18
            let w = max(b.maxX - b.minX, 0.001)
            let h = max(b.maxY - b.minY, 0.001)
            // One scale for both axes so the room is not stretched.
            let scale = min((size.width - 2 * pad) / w, (size.height - 2 * pad) / h)
            let offX = (size.width - CGFloat(w) * scale) / 2
            let offY = (size.height - CGFloat(h) * scale) / 2

            func pt(_ p: TrajectoryPoint) -> CGPoint {
                CGPoint(x: offX + CGFloat(p.x - b.minX) * scale,
                        // flip Y: metres go up, screen goes down
                        y: size.height - (offY + CGFloat(p.y - b.minY) * scale))
            }

            // route
            var route = Path()
            route.move(to: pt(track.points[0]))
            for p in track.points.dropFirst() { route.addLine(to: pt(p)) }
            ctx.stroke(route, with: .color(LabTheme.lineStrong),
                       style: StrokeStyle(lineWidth: 1, lineCap: .round, lineJoin: .round))

            // every recorded pose
            for (i, p) in track.points.enumerated() {
                let c = pt(p)
                let isHi = highlight == i
                let r: CGFloat = isHi ? 4.5 : (p.s < 0.05 ? 2.4 : 1.8)
                ctx.fill(Path(ellipseIn: CGRect(x: c.x - r, y: c.y - r,
                                                width: r * 2, height: r * 2)),
                         with: .color(tint(p).opacity(isHi ? 1 : 0.62)))
            }

            // start / end
            if let f = track.points.first, let l = track.points.last {
                let s = pt(f), e = pt(l)
                ctx.stroke(Path(ellipseIn: CGRect(x: s.x - 6, y: s.y - 6, width: 12, height: 12)),
                           with: .color(LabTheme.text), lineWidth: 1.4)
                ctx.fill(Path(ellipseIn: CGRect(x: e.x - 3.5, y: e.y - 3.5, width: 7, height: 7)),
                         with: .color(LabTheme.text))
            }

            // heading at the highlighted pose
            if showHeading, let i = highlight, i < track.points.count {
                let p = track.points[i]
                let c = pt(p)
                let rad = p.h * .pi / 180
                var arrow = Path()
                arrow.move(to: c)
                arrow.addLine(to: CGPoint(x: c.x + cos(rad) * 22, y: c.y - sin(rad) * 22))
                ctx.stroke(arrow, with: .color(LabTheme.text),
                           style: StrokeStyle(lineWidth: 1.5, lineCap: .round))
            }
        }
        .drawingGroup()
        .accessibilityLabel("Walk route, \(track.frames) poses over \(String(format: "%.1f", track.path_m)) metres")
    }
}

/// Map plus the facts that make it readable: length, duration, action mix.
struct TrajectoryCard: View {
    let track: SessionTrajectory
    @State private var cursor: Double = 0

    private var idx: Int {
        max(0, min(track.points.count - 1, Int(cursor)))
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(alignment: .firstTextBaseline, spacing: 10) {
                Text("Route")
                    .font(LabTheme.t(13, .medium))
                    .foregroundStyle(LabTheme.faint)
                Text(track.session)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.faint)
                    .lineLimit(1)
                    .truncationMode(.middle)
                Spacer(minLength: 8)
                Text("\(track.frames) poses \u{00b7} \(String(format: "%.1f", track.path_m)) m \u{00b7} \(Int(track.duration))s")
                    .font(LabTheme.mono(11))
                    .monospacedDigit()
                    .foregroundStyle(LabTheme.faint)
            }

            TrajectoryMap(track: track, highlight: idx)
                .frame(height: 220)
                .background(
                    RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                        .fill(LabTheme.well)
                )
                .overlay(
                    RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                        .stroke(LabTheme.line, lineWidth: 1)
                )

            if track.points.count > 1 {
                Slider(value: $cursor, in: 0...Double(track.points.count - 1))
                    .controlSize(.small)
                    .tint(LabTheme.accent)
                    .accessibilityLabel("Scrub the walk")
            }

            let p = track.points[idx]
            HStack(spacing: 22) {
                mapFact("Frame", "\(idx)")
                mapFact("Action", track.actionName(p))
                mapFact("Task", track.taskName(p))
                mapFact("Heading", String(format: "%.0f\u{00b0}", p.h))
                mapFact("Speed", String(format: "%.2f m/s", p.s))
                Spacer(minLength: 0)
            }

            // Legend doubles as codebook coverage: a symbol the walk never
            // produced shows zero rather than disappearing.
            HStack(spacing: 14) {
                ForEach(track.actionCounts, id: \.name) { item in
                    HStack(spacing: 5) {
                        Circle()
                            .fill(legendColor(item.name))
                            .frame(width: 5, height: 5)
                            .opacity(item.count == 0 ? 0.35 : 1)
                        Text("\(item.name) \(item.count)")
                            .font(LabTheme.mono(10))
                            .monospacedDigit()
                            .foregroundStyle(item.count == 0 ? LabTheme.faint : LabTheme.dim)
                    }
                }
                Spacer(minLength: 0)
                Text("from \(track.source)")
                    .font(LabTheme.mono(10))
                    .foregroundStyle(LabTheme.faint)
            }
        }
    }

    private func legendColor(_ name: String) -> Color {
        switch name {
        case "hold": return LabTheme.warn
        case "turn_left", "turn_right": return LabTheme.good
        default: return LabTheme.accent
        }
    }

    private func mapFact(_ k: String, _ v: String) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(k)
                .font(LabTheme.label())
                .foregroundStyle(LabTheme.faint)
                .textCase(.uppercase)
                .tracking(1)
            Text(v)
                .font(LabTheme.mono(12))
                .monospacedDigit()
                .foregroundStyle(LabTheme.text)
        }
    }
}
