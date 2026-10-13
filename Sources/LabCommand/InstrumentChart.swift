import SwiftUI
import LabCommandCore

/// Trace for one telemetry series. Gaps are gaps: an unmeasured probe breaks
/// the line rather than interpolating a value the Spark never sent.
struct Sparkline: View {
    let values: [Double?]
    let range: (lo: Double, hi: Double)?
    var tint: Color = LabTheme.accent
    var showFill: Bool = true
    /// Draws a hard rule at zero when the series can legitimately reach it.
    var zeroRule: Bool = false

    var body: some View {
        Canvas(rendersAsynchronously: false) { ctx, size in
            guard let range, values.count > 1 else { return }
            let span = max(range.hi - range.lo, 1e-9)
            let dx = size.width / CGFloat(values.count - 1)
            func point(_ i: Int, _ v: Double) -> CGPoint {
                let y = 1 - (v - range.lo) / span
                return CGPoint(x: CGFloat(i) * dx,
                               y: max(0, min(1, y)) * size.height)
            }

            if zeroRule, range.lo <= 0, range.hi >= 0 {
                let y = (1 - (0 - range.lo) / span) * size.height
                var r = Path()
                r.move(to: CGPoint(x: 0, y: y))
                r.addLine(to: CGPoint(x: size.width, y: y))
                ctx.stroke(r, with: .color(LabTheme.line),
                           style: StrokeStyle(lineWidth: 1, dash: [2, 3]))
            }

            // Contiguous runs only — a nil ends the current run.
            var run: [(Int, Double)] = []
            func flush() {
                defer { run = [] }
                guard run.count > 1 else {
                    // A lone sample still deserves a mark.
                    if let (i, v) = run.first {
                        let p = point(i, v)
                        ctx.fill(Path(ellipseIn: CGRect(x: p.x - 1.5, y: p.y - 1.5,
                                                        width: 3, height: 3)),
                                 with: .color(tint))
                    }
                    return
                }
                var line = Path()
                line.move(to: point(run[0].0, run[0].1))
                for (i, v) in run.dropFirst() { line.addLine(to: point(i, v)) }

                if showFill {
                    var area = line
                    area.addLine(to: CGPoint(x: point(run.last!.0, 0).x, y: size.height))
                    area.addLine(to: CGPoint(x: point(run[0].0, 0).x, y: size.height))
                    area.closeSubpath()
                    ctx.fill(area, with: .linearGradient(
                        Gradient(colors: [tint.opacity(0.22), tint.opacity(0.02)]),
                        startPoint: .zero,
                        endPoint: CGPoint(x: 0, y: size.height)))
                }
                ctx.stroke(line, with: .color(tint),
                           style: StrokeStyle(lineWidth: 1.5, lineJoin: .round))
            }

            for (i, v) in values.enumerated() {
                if let v { run.append((i, v)) } else { flush() }
            }
            flush()

            // Emphasised endpoint — the reading you are actually standing on.
            if let lastIdx = values.lastIndex(where: { $0 != nil }),
               let v = values[lastIdx] {
                let p = point(lastIdx, v)
                ctx.fill(Path(ellipseIn: CGRect(x: p.x - 4, y: p.y - 4, width: 8, height: 8)),
                         with: .color(tint.opacity(0.22)))
                ctx.fill(Path(ellipseIn: CGRect(x: p.x - 2, y: p.y - 2, width: 4, height: 4)),
                         with: .color(tint))
            }
        }
        .drawingGroup()
    }
}

/// Label + value + trace. The unit of the readout deck.
struct InstrumentCell: View {
    let label: String
    let value: String
    var values: [Double?] = []
    var range: (lo: Double, hi: Double)? = nil
    var tint: Color = LabTheme.accent
    var alarm: Bool = false
    var zeroRule: Bool = false
    /// Shown when the current probe has no value but an earlier one did.
    var stale: (text: String, age: String)? = nil

    private var showsStale: Bool {
        value == TelemetryFormat.unavailable && stale != nil
    }

    private var tone: Color {
        if alarm { return LabTheme.bad }
        if value == TelemetryFormat.unavailable { return LabTheme.faint }
        return LabTheme.text
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            Text(label)
                .font(LabTheme.label())
                .foregroundStyle(LabTheme.faint)
                .textCase(.uppercase)
                .tracking(1)
            // A dropped link should not blank the panel: keep the last real
            // reading visible, marked with its age so it is never mistaken for live.
            if showsStale, let stale {
                HStack(spacing: 6) {
                    Text(stale.text)
                        .font(LabTheme.mono(12))
                        .monospacedDigit()
                        .foregroundStyle(LabTheme.dim)
                        .strikethrough(false)
                    Text(stale.age)
                        .font(LabTheme.mono(10))
                        .foregroundStyle(LabTheme.faint)
                }
            } else {
                Text(value)
                    .font(LabTheme.mono(12))
                    .monospacedDigit()
                    .foregroundStyle(tone)
                    .textSelection(.enabled)
            }
            if range != nil, values.contains(where: { $0 != nil }) {
                Sparkline(values: values, range: range,
                          tint: alarm ? LabTheme.bad : tint, zeroRule: zeroRule)
                    .frame(height: 26)
                    .opacity(showsStale ? 0.45 : 1)
            } else {
                // Keep the row height stable whether or not a trace exists.
                Rectangle()
                    .fill(LabTheme.line.opacity(0.5))
                    .frame(height: 1)
                    .frame(maxHeight: 26, alignment: .center)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .accessibilityElement(children: .combine)
        .accessibilityLabel(showsStale && stale != nil
            ? "\(label) \(stale!.text), \(stale!.age)"
            : "\(label) \(value)")
    }
}

/// Cells separated by true hairlines rather than gaps — the dense instrument
/// grid the web console uses (1px background showing through a tight grid).
struct HairlineRow<Content: View>: View {
    var padding: CGFloat = 12
    @ViewBuilder var content: () -> Content

    var body: some View {
        HStack(spacing: 1) {
            content()
        }
        .padding(1)
        .background(LabTheme.line)
        .clipShape(RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous))
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                .stroke(LabTheme.line, lineWidth: 1)
        )
    }
}

/// A cell that paints its own surface so the hairline shows between siblings.
struct HairlineCell<Content: View>: View {
    var padding: CGFloat = 12
    @ViewBuilder var content: () -> Content

    var body: some View {
        content()
            .padding(padding)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(LabTheme.surface)
    }
}
