import SwiftUI
import LabCommandCore

/// The Room tab's live timeline: event lanes, the activity lane, bands, markers and the axis.
/// It only draws `LiveRoomTimeline`; selection goes through `hit` so ids match the event column.
struct LiveRoomTimelineView: View {
    var timeline: LiveRoomTimeline
    @Binding var selection: String?
    @Binding var window: TimeInterval
    @Binding var follow: Bool

    @State private var hoverText: String?

    private static let canvasHeight: CGFloat = 120
    private static let labelWidth: CGFloat = 76
    private static let axisHeight: CGFloat = 16
    private static let barHeight: CGFloat = 6

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            controls
            Canvas { context, size in
                draw(&context, size: size)
            }
            .frame(height: Self.canvasHeight)
            .frame(maxWidth: .infinity)
            .contentShape(Rectangle())
            .background(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .fill(LabTheme.well)
            )
            .overlay(
                RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                    .stroke(LabTheme.line, lineWidth: 1)
            )
            .clipShape(RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous))
            .overlay(GeometryReader { geometry in
                Color.clear
                    .contentShape(Rectangle())
                    .onTapGesture(coordinateSpace: .local) { location in
                        tap(at: location, size: geometry.size)
                    }
                    .onContinuousHover(coordinateSpace: .local) { phase in
                        switch phase {
                        case .active(let location):
                            let text = describe(at: location, size: geometry.size)
                            if text != hoverText { hoverText = text }
                        case .ended:
                            hoverText = nil
                        }
                    }
            })
            .help(hoverText ?? "")
            Text(hoverText ?? " ")
                .font(LabTheme.mono(11))
                .foregroundStyle(LabTheme.dim)
                .lineLimit(1)
                .frame(maxWidth: .infinity, alignment: .leading)
        }
    }

    // MARK: Controls

    private var controls: some View {
        HStack(spacing: 10) {
            Text("Timeline")
                .font(LabTheme.t(11, .medium))
                .foregroundStyle(LabTheme.faint)
            Picker("Window", selection: $window) {
                ForEach(LiveRoomTimeline.windows, id: \.self) { span in
                    Text(windowTitle(span)).tag(span)
                }
            }
            .pickerStyle(.segmented)
            .labelsHidden()
            .frame(width: 200)
            .font(LabTheme.t(11))
            Toggle("Follow now", isOn: $follow)
                .toggleStyle(.checkbox)
                .font(LabTheme.t(11))
                .foregroundStyle(LabTheme.dim)
            if timeline.unverified {
                Text("unverified · old Spark node")
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.warn)
                    .lineLimit(1)
            }
            if timeline.nowIsEstimate, timeline.now != nil {
                Text("now estimated")
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.warn)
                    .lineLimit(1)
            }
            ForEach(timeline.badges, id: \.self) { badge in
                Text(badge)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(LabTheme.bad)
                    .lineLimit(1)
            }
            Spacer(minLength: 0)
        }
    }

    private func windowTitle(_ span: TimeInterval) -> String {
        span >= 3600 ? "\(Int(span / 3600)) h" : "\(Int(span / 60)) min"
    }

    // MARK: Layout

    private struct Layout {
        var plot: CGRect
        var lanes: [LiveRoomTimeline.Lane]
        var laneHeight: CGFloat

        func row(_ lane: LiveRoomTimeline.Lane) -> CGRect? {
            guard let index = lanes.firstIndex(of: lane) else { return nil }
            return CGRect(x: plot.minX, y: plot.minY + CGFloat(index) * laneHeight, width: plot.width, height: laneHeight)
        }

        func lane(at y: CGFloat) -> LiveRoomTimeline.Lane? {
            guard y >= plot.minY, y < plot.maxY, laneHeight > 0 else { return nil }
            let index = Int((y - plot.minY) / laneHeight)
            return lanes.indices.contains(index) ? lanes[index] : nil
        }
    }

    private func layout(_ size: CGSize) -> Layout {
        let plot = CGRect(
            x: Self.labelWidth,
            y: 4,
            width: max(1, size.width - Self.labelWidth - 8),
            height: max(1, size.height - Self.axisHeight - 4)
        )
        let count = max(1, timeline.lanes.count)
        return Layout(plot: plot, lanes: timeline.lanes, laneHeight: plot.height / CGFloat(count))
    }

    private func x(_ stamp: Double, _ layout: Layout) -> CGFloat {
        layout.plot.minX + CGFloat(timeline.x(for: stamp, width: Double(layout.plot.width)))
    }

    // MARK: Input

    private func tap(at location: CGPoint, size: CGSize) {
        let layout = layout(size)
        guard let lane = layout.lane(at: location.y), lane != .activity else { return }
        let hit = timeline.hit(x: Double(location.x - layout.plot.minX), lane: lane, width: Double(layout.plot.width))
        if let hit {
            selection = selection == hit ? nil : hit
        } else {
            selection = nil
        }
    }

    private func describe(at location: CGPoint, size: CGSize) -> String? {
        let layout = layout(size)
        guard location.x >= layout.plot.minX, location.x <= layout.plot.maxX else { return nil }
        let px = Double(location.x - layout.plot.minX)
        let width = Double(layout.plot.width)
        if let lane = layout.lane(at: location.y), lane != .activity,
           let id = timeline.hit(x: px, lane: lane, width: width),
           let item = timeline.items.first(where: { $0.id == id }) {
            return item.detail
        }
        for marker in timeline.markers where abs(timeline.x(for: marker.stamp, width: width) - px) <= 4 {
            return marker.detail
        }
        if layout.lane(at: location.y) == .activity {
            for segment in timeline.activity where segment.start <= stamp(px, width) && stamp(px, width) <= segment.end {
                switch segment.state {
                case .active: return "activity next to the rig"
                case .quiet: return "no activity seen"
                case .unobserved: return "not observed · sensors not healthy or no Activity"
                }
            }
        }
        let t = stamp(px, width)
        let labels = timeline.bands.filter { $0.start <= t && t <= $0.end }.map(\.label)
        return labels.isEmpty ? nil : labels.joined(separator: " · ")
    }

    private func stamp(_ px: Double, _ width: Double) -> Double {
        guard width > 0 else { return timeline.start }
        return timeline.start + px / width * (timeline.end - timeline.start)
    }

    // MARK: Drawing

    private func draw(_ context: inout GraphicsContext, size: CGSize) {
        let layout = layout(size)
        drawLanes(&context, layout)
        drawBands(&context, layout)
        drawActivity(&context, layout)
        drawItems(&context, layout)
        drawMarkers(&context, layout)
        drawNow(&context, layout)
        drawAxis(&context, layout, size: size)
    }

    private func drawLanes(_ context: inout GraphicsContext, _ layout: Layout) {
        for lane in layout.lanes {
            guard let row = layout.row(lane) else { continue }
            var divider = Path()
            divider.move(to: CGPoint(x: 0, y: row.maxY))
            divider.addLine(to: CGPoint(x: layout.plot.maxX, y: row.maxY))
            context.stroke(divider, with: .color(LabTheme.line), lineWidth: 0.5)
            context.draw(
                Text(lane.title).font(LabTheme.t(10.5, .medium)).foregroundColor(laneColor(lane)),
                at: CGPoint(x: 8, y: row.midY),
                anchor: .leading
            )
        }
        for tick in timeline.ticks {
            var grid = Path()
            let gx = x(tick.stamp, layout)
            grid.move(to: CGPoint(x: gx, y: layout.plot.minY))
            grid.addLine(to: CGPoint(x: gx, y: layout.plot.maxY))
            context.stroke(grid, with: .color(LabTheme.line.opacity(0.6)), lineWidth: 0.5)
        }
    }

    private func drawBands(_ context: inout GraphicsContext, _ layout: Layout) {
        for band in timeline.bands {
            let x0 = x(band.start, layout)
            let x1 = max(x0 + 1, x(band.end, layout))
            let rect = CGRect(x: x0, y: layout.plot.minY, width: x1 - x0, height: layout.plot.height)
            switch band.kind {
            case .notObserved:
                hatch(&context, rect, color: LabTheme.unknown, spacing: 6)
            case .historyUnknown:
                context.fill(Path(rect), with: .color(LabTheme.bg.opacity(0.5)))
                hatch(&context, rect, color: LabTheme.idle, spacing: 9)
            case .lost:
                context.fill(Path(rect), with: .color(LabTheme.bad.opacity(0.16)))
                edge(&context, rect, color: LabTheme.bad)
            case .retrieving:
                context.fill(Path(rect), with: .color(LabTheme.warn.opacity(0.12)))
                edge(&context, rect, color: LabTheme.warn)
            }
            if rect.width > 60 {
                var inner = context
                inner.clip(to: Path(rect))
                inner.draw(
                    Text(band.label).font(LabTheme.mono(9.5)).foregroundColor(bandTextColor(band.kind)),
                    at: CGPoint(x: rect.minX + 4, y: layout.plot.minY + 2),
                    anchor: .topLeading
                )
            }
        }
    }

    private func drawActivity(_ context: inout GraphicsContext, _ layout: Layout) {
        guard let row = layout.row(.activity) else { return }
        let lane = row.insetBy(dx: 0, dy: row.height * 0.3)
        for segment in timeline.activity {
            let x0 = x(segment.start, layout)
            let x1 = max(x0 + 1, x(segment.end, layout))
            let rect = CGRect(x: x0, y: lane.minY, width: x1 - x0, height: lane.height)
            switch segment.state {
            case .active:
                context.fill(Path(rect), with: .color(LabTheme.accent))
            case .quiet:
                context.fill(Path(CGRect(x: x0, y: lane.midY - 1, width: x1 - x0, height: 2)), with: .color(LabTheme.lineStrong))
            case .unobserved:
                hatch(&context, rect, color: LabTheme.unknown, spacing: 4)
            }
        }
    }

    private func drawItems(_ context: inout GraphicsContext, _ layout: Layout) {
        for item in timeline.items {
            guard let row = layout.row(item.lane) else { continue }
            let color = kindColor(item.kind).opacity(item.earlierRun ? 0.4 : 1)
            let y = row.midY
            if item.selected {
                let x0 = x(item.start, layout) - 3
                let x1 = x(item.end, layout) + 3
                let rect = CGRect(x: x0, y: y - Self.barHeight, width: max(6, x1 - x0), height: Self.barHeight * 2)
                let shape = Path(roundedRect: rect, cornerRadius: 3)
                context.fill(shape, with: .color(LabTheme.accentMuted))
                context.stroke(shape, with: .color(LabTheme.text.opacity(0.8)), lineWidth: 1)
            }
            for segment in item.segments {
                let x0 = x(segment.start, layout)
                let x1 = max(x0 + 1.5, x(segment.end, layout))
                switch segment.style {
                case .confirmed:
                    let rect = CGRect(x: x0, y: y - Self.barHeight / 2, width: x1 - x0, height: Self.barHeight)
                    context.fill(Path(roundedRect: rect, cornerRadius: 1.5), with: .color(color))
                case .candidate, .leadIn:
                    var line = Path()
                    line.move(to: CGPoint(x: x0, y: y))
                    line.addLine(to: CGPoint(x: x1, y: y))
                    let dash: [CGFloat] = segment.style == .leadIn ? [2, 3] : [4, 3]
                    let tone = segment.style == .leadIn ? color.opacity(0.55) : color
                    context.stroke(line, with: .color(tone), style: StrokeStyle(lineWidth: 2, dash: dash))
                }
            }
            for tick in item.ticks {
                var mark = Path()
                let tx = x(tick.stamp, layout)
                mark.move(to: CGPoint(x: tx, y: y - Self.barHeight))
                mark.addLine(to: CGPoint(x: tx, y: y + Self.barHeight))
                context.stroke(mark, with: .color(color), lineWidth: 1)
            }
        }
    }

    private func drawMarkers(_ context: inout GraphicsContext, _ layout: Layout) {
        for marker in timeline.markers {
            let mx = x(marker.stamp, layout)
            var line = Path()
            line.move(to: CGPoint(x: mx, y: layout.plot.minY))
            line.addLine(to: CGPoint(x: mx, y: layout.plot.maxY))
            let color = marker.kind == .restart ? LabTheme.warn : LabTheme.bad
            context.stroke(line, with: .color(color), style: StrokeStyle(lineWidth: 1, dash: [3, 2]))
            context.draw(
                Text(marker.label).font(LabTheme.mono(9.5)).foregroundColor(color),
                at: CGPoint(x: mx + 3, y: layout.plot.maxY - 2),
                anchor: .bottomLeading
            )
        }
    }

    private func drawNow(_ context: inout GraphicsContext, _ layout: Layout) {
        guard let now = timeline.now, now >= timeline.start, now <= timeline.end else { return }
        let nx = min(x(now, layout), layout.plot.maxX - 0.5)
        var line = Path()
        line.move(to: CGPoint(x: nx, y: layout.plot.minY))
        line.addLine(to: CGPoint(x: nx, y: layout.plot.maxY))
        let style = timeline.nowIsEstimate ? StrokeStyle(lineWidth: 1, dash: [2, 2]) : StrokeStyle(lineWidth: 1)
        context.stroke(line, with: .color(timeline.nowIsEstimate ? LabTheme.warn : LabTheme.text.opacity(0.7)), style: style)
    }

    private func drawAxis(_ context: inout GraphicsContext, _ layout: Layout, size: CGSize) {
        let y = size.height - Self.axisHeight / 2
        for tick in timeline.ticks {
            context.draw(
                Text(tick.label).font(LabTheme.mono(9.5)).foregroundColor(LabTheme.dim),
                at: CGPoint(x: x(tick.stamp, layout), y: y),
                anchor: .center
            )
        }
    }

    // MARK: Primitives

    /// Diagonal lines clipped to `rect`: not observed, never drawn as empty.
    private func hatch(_ context: inout GraphicsContext, _ rect: CGRect, color: Color, spacing: CGFloat) {
        guard rect.width > 0, rect.height > 0 else { return }
        var inner = context
        inner.clip(to: Path(rect))
        var lines = Path()
        var offset = rect.minX - rect.height
        // Anchor to a fixed phase so stripes do not crawl as the window follows now.
        offset -= offset.truncatingRemainder(dividingBy: spacing)
        while offset < rect.maxX {
            lines.move(to: CGPoint(x: offset, y: rect.maxY))
            lines.addLine(to: CGPoint(x: offset + rect.height, y: rect.minY))
            offset += spacing
        }
        inner.stroke(lines, with: .color(color), lineWidth: 1)
    }

    private func edge(_ context: inout GraphicsContext, _ rect: CGRect, color: Color) {
        var lines = Path()
        lines.move(to: CGPoint(x: rect.minX, y: rect.minY))
        lines.addLine(to: CGPoint(x: rect.minX, y: rect.maxY))
        lines.move(to: CGPoint(x: rect.maxX, y: rect.minY))
        lines.addLine(to: CGPoint(x: rect.maxX, y: rect.maxY))
        context.stroke(lines, with: .color(color.opacity(0.7)), lineWidth: 1)
    }

    private func bandTextColor(_ kind: LiveRoomTimeline.Band.Kind) -> Color {
        switch kind {
        case .notObserved, .historyUnknown: return LabTheme.dim
        case .lost: return LabTheme.bad
        case .retrieving: return LabTheme.warn
        }
    }

    private func laneColor(_ lane: LiveRoomTimeline.Lane) -> Color {
        switch lane {
        case .appeared: return LabTheme.good
        case .disappeared: return LabTheme.bad
        case .moved: return LabTheme.warn
        case .activity: return LabTheme.dim
        }
    }

    private func kindColor(_ kind: RoomMemory.ChangeEvent.Kind) -> Color {
        switch kind {
        case .appeared: return LabTheme.good
        case .disappeared: return LabTheme.bad
        case .moved: return LabTheme.warn
        }
    }
}
