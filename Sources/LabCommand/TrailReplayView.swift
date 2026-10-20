import SwiftUI
import LabCommandCore

/// Trajectory replay: the session map from above, the cart's path drawing itself over time,
/// and a readout of what the files measure. Local files only.
struct TrailReplayView: View {
    @ObservedObject var player: TrailPlayer

    @State private var zoom: CGFloat = 1
    @State private var pinch: CGFloat = 1
    @State private var pan: CGSize = .zero
    @State private var drag: CGSize = .zero
    @State private var showsTopdown = false

    var body: some View {
        Group {
            switch player.phase {
            case .empty:
                message(nil, tint: LabTheme.dim)
            case .loading(let directory):
                message("Reading \(directory)…", tint: LabTheme.dim)
            case .failed(let text):
                message(text, tint: LabTheme.warn)
            case .ready:
                TimelineView(.animation(minimumInterval: 1.0 / 60, paused: !player.clock.isPlaying)) { _ in
                    page(now: TrailPlayer.now)
                }
            }
        }
        .onAppear { player.appeared() }
        // A new session opens fitted.
        .onChange(of: player.directory) { _, _ in
            zoom = 1
            pan = .zero
            showsTopdown = false
        }
        .sheet(isPresented: $showsTopdown) { topdownSheet }
    }

    /// No session yet, a load in progress, or a failure: the picker and the sessions list.
    private func message(_ text: String?, tint: Color) -> some View {
        VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
            VStack(alignment: .leading, spacing: 8) {
                Text("Trail replay")
                    .font(LabTheme.t(LabTheme.titleSize, .semibold))
                    .foregroundStyle(LabTheme.text)
                Text(text ?? "Open a session folder (trajectory.tum, map.pcd, manifest.json). Local files only; no rig contact.")
                    .font(LabTheme.t(13))
                    .foregroundStyle(tint)
                    .textSelection(.enabled)
                    .fixedSize(horizontal: false, vertical: true)
            }
            sessionsSection(limit: nil)
                .frame(maxWidth: 520, alignment: .leading)
        }
        .padding(LabTheme.pad)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
    }

    // MARK: - Sessions

    private func sessionsSection(limit: Int?) -> some View {
        let shown = limit.map { Array(player.entries.prefix($0)) } ?? player.entries
        return VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 6) {
                SectionLabel(text: "Sessions")
                Spacer(minLength: 4)
                Button("Rescan") { player.rescan() }
                Button("Fixture") { player.openFixture() }
                Button("Open folder…") { player.chooseFolder() }
            }
            .font(LabTheme.t(11, .medium))
            .buttonStyle(QuietButton())
            if shown.isEmpty {
                Text("No sessions with trajectory.tum or map.pcd in \(abbreviated(player.root)).")
                    .font(LabTheme.t(11))
                    .foregroundStyle(LabTheme.faint)
                    .fixedSize(horizontal: false, vertical: true)
            } else {
                Text(abbreviated(player.root))
                    .font(LabTheme.mono(10))
                    .foregroundStyle(LabTheme.faint)
                    .lineLimit(1)
                    .truncationMode(.middle)
                ForEach(shown) { entry in
                    sessionRow(entry)
                }
                if shown.count < player.entries.count {
                    Text("+\(player.entries.count - shown.count) more · Open folder… to pick another")
                        .font(LabTheme.t(10))
                        .foregroundStyle(LabTheme.faint)
                }
            }
        }
    }

    private func sessionRow(_ entry: TrailSessionEntry) -> some View {
        let current = player.directory == entry.directory
        let files = [entry.hasTrajectory ? "trail" : nil, entry.hasMap ? "map" : nil].compactMap { $0 }.joined(separator: "+")
        return Button {
            player.open(directory: entry.directory)
        } label: {
            HStack(spacing: 6) {
                Text(entry.name)
                    .font(LabTheme.mono(11))
                    .foregroundStyle(current ? LabTheme.text : LabTheme.dim)
                    .lineLimit(1)
                    .truncationMode(.middle)
                Spacer(minLength: 4)
                if let verdict = entry.walkcheckResult {
                    Text(verdict)
                        .font(LabTheme.mono(10))
                        .foregroundStyle(verdictTint(verdict))
                }
                Text(files)
                    .font(LabTheme.mono(10))
                    .foregroundStyle(LabTheme.faint)
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 4)
            .background(
                RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                    .fill(current ? LabTheme.accentMuted : LabTheme.well)
            )
            .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
        .accessibilityLabel("Open session \(entry.name)")
    }

    private func verdictTint(_ verdict: String) -> Color {
        switch verdict {
        case "PASS": return LabTheme.good
        case "FAIL": return LabTheme.bad
        default: return LabTheme.warn
        }
    }

    private func abbreviated(_ path: String) -> String {
        (path as NSString).abbreviatingWithTildeInPath
    }

    // MARK: - Walk check

    private func walkcheckSection(_ w: TrailWalkcheck, session: TrailSession) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                SectionLabel(text: "Walk check")
                Spacer(minLength: 4)
                if session.topdownPath != nil {
                    Button("topdown.png") { showsTopdown = true }
                        .font(LabTheme.t(11, .medium))
                        .buttonStyle(QuietButton())
                }
            }
            InstrumentWell {
                VStack(alignment: .leading, spacing: 5) {
                    ForEach(TrailReadout.walkcheckRows(w)) { row in
                        if row.label == "Verdict" {
                            HStack {
                                Text("Verdict").font(LabTheme.t(12)).foregroundStyle(LabTheme.dim)
                                Spacer(minLength: 8)
                                Text(row.value)
                                    .font(LabTheme.mono(12))
                                    .fontWeight(.semibold)
                                    .foregroundStyle(verdictTint(row.value))
                            }
                        } else {
                            readoutRow(row)
                        }
                    }
                    Text(TrailReadout.walkcheckNote(w, accuracyLabel: session.accuracyLabel))
                        .font(LabTheme.t(10))
                        .foregroundStyle(LabTheme.faint)
                        .fixedSize(horizontal: false, vertical: true)
                        .padding(.top, 2)
                }
            }
        }
    }

    @ViewBuilder
    private var topdownSheet: some View {
        VStack(alignment: .trailing, spacing: 10) {
            if let path = player.session?.topdownPath, let image = NSImage(contentsOfFile: path) {
                Image(nsImage: image)
                    .resizable()
                    .interpolation(.high)
                    .aspectRatio(contentMode: .fit)
                    .frame(minWidth: 520, idealWidth: 820, minHeight: 420, idealHeight: 640)
                Text(path)
                    .font(LabTheme.mono(10))
                    .foregroundStyle(LabTheme.faint)
                    .textSelection(.enabled)
            } else {
                Text("topdown.png could not be read.")
                    .foregroundStyle(LabTheme.warn)
                    .frame(width: 320, height: 120)
            }
            Button("Close") { showsTopdown = false }
                .keyboardShortcut(.cancelAction)
        }
        .padding(LabTheme.pad)
        .background(LabTheme.bg)
    }

    private func page(now: Double) -> some View {
        let sample = player.sample(at: now)
        return HStack(alignment: .top, spacing: LabTheme.pad) {
            sideColumn(sample: sample)
                .frame(width: 300)
            VStack(alignment: .leading, spacing: 10) {
                canvas(sample: sample)
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .background(
                        RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                            .fill(LabTheme.surface)
                    )
                    .clipShape(RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous))
                    .overlay(
                        RoundedRectangle(cornerRadius: LabTheme.radius, style: .continuous)
                            .stroke(LabTheme.line, lineWidth: 1)
                    )
                transport(now: now)
            }
        }
        .padding(LabTheme.pad)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
    }

    // MARK: - Side column

    private func sideColumn(sample: TrailSample?) -> some View {
        let session = player.session
        // Scrolls when the walk check, files and sessions do not all fit beside the map.
        return ScrollView(.vertical, showsIndicators: false) {
        VStack(alignment: .leading, spacing: LabTheme.sectionGap) {
            VStack(alignment: .leading, spacing: 6) {
                Text("Trail replay")
                    .font(LabTheme.t(LabTheme.titleSize, .semibold))
                    .foregroundStyle(LabTheme.text)
                Text(session?.name ?? "—")
                    .font(LabTheme.mono(12))
                    .foregroundStyle(LabTheme.text)
                    .lineLimit(1)
                    .truncationMode(.middle)
                if let line = sessionLine(session) {
                    Text(line)
                        .font(LabTheme.t(11))
                        .foregroundStyle(LabTheme.dim)
                        .lineLimit(2)
                }
                if let label = session?.accuracyLabel {
                    accuracyBadge(label)
                        .padding(.top, 2)
                }
            }

            if let session, let walkcheck = session.walkcheck {
                walkcheckSection(walkcheck, session: session)
            }

            VStack(alignment: .leading, spacing: 8) {
                SectionLabel(text: "Readout")
                InstrumentWell {
                    if let summary = player.summary {
                        VStack(alignment: .leading, spacing: 5) {
                            ForEach(TrailReadout.rows(summary: summary, sample: sample)) { row in
                                readoutRow(row)
                            }
                            Text("Start → end: \(TrailReadout.startToEndNote).")
                                .font(LabTheme.t(10))
                                .foregroundStyle(LabTheme.faint)
                                .fixedSize(horizontal: false, vertical: true)
                                .padding(.top, 2)
                        }
                    } else {
                        Text(TrailSession.noTrajectoryLabel)
                            .font(LabTheme.t(12))
                            .foregroundStyle(LabTheme.warn)
                    }
                }
            }

            VStack(alignment: .leading, spacing: 8) {
                SectionLabel(text: "Map")
                VStack(alignment: .leading, spacing: 4) {
                    Text(player.mapLine ?? "no map.pcd")
                    if player.mapLine != nil, let band = player.bandLine {
                        Text(band)
                    }
                }
                .font(LabTheme.mono(10))
                .foregroundStyle(LabTheme.dim)
                .fixedSize(horizontal: false, vertical: true)
                legend
            }

            if let problems = session?.problems, !problems.isEmpty {
                VStack(alignment: .leading, spacing: 6) {
                    SectionLabel(text: "Files")
                    ForEach(problems, id: \.self) { problem in
                        Text(problem)
                            .font(LabTheme.t(11))
                            .foregroundStyle(LabTheme.warn)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }
            }

            sessionsSection(limit: 6)

            if let directory = session?.directory {
                Text(directory)
                    .font(LabTheme.mono(10))
                    .foregroundStyle(LabTheme.faint)
                    .lineLimit(2)
                    .truncationMode(.middle)
                    .textSelection(.enabled)
            }
        }
        .frame(maxWidth: .infinity, alignment: .topLeading)
        }
        .frame(maxHeight: .infinity, alignment: .topLeading)
    }

    private func sessionLine(_ session: TrailSession?) -> String? {
        guard let manifest = session?.manifest else { return nil }
        let parts = [manifest.sensor, manifest.createdAt].compactMap { $0 }
        var line = parts.joined(separator: " · ")
        if let notes = manifest.notes {
            line += line.isEmpty ? notes : "\n\(notes)"
        }
        return line.isEmpty ? nil : line
    }

    private func readoutRow(_ row: TrailReadout.Row) -> some View {
        let sub = row.label.hasPrefix(" ")
        return HStack(alignment: .firstTextBaseline) {
            Text(row.label.trimmingCharacters(in: .whitespaces))
                .font(LabTheme.t(sub ? 11 : 12))
                .foregroundStyle(sub ? LabTheme.faint : LabTheme.dim)
                .padding(.leading, sub ? 10 : 0)
            Spacer(minLength: 8)
            Text(row.value)
                .font(LabTheme.mono(sub ? 11 : 12))
                .foregroundStyle(sub ? LabTheme.dim : LabTheme.text)
                .monospacedDigit()
        }
    }

    private func accuracyBadge(_ label: String) -> some View {
        HStack(spacing: 6) {
            Circle()
                .fill(LabTheme.warn)
                .frame(width: LabTheme.pip, height: LabTheme.pip)
            Text(label)
                .font(LabTheme.t(11, .medium))
                .foregroundStyle(LabTheme.warn)
                .fixedSize(horizontal: false, vertical: true)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 5)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .fill(LabTheme.warn.opacity(0.08))
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .stroke(LabTheme.warn.opacity(0.45), lineWidth: 1)
        )
        .accessibilityLabel(label)
    }

    private var legend: some View {
        VStack(alignment: .leading, spacing: 5) {
            HStack(spacing: 8) {
                LinearGradient(colors: TrailPalette.stops.map(\.color), startPoint: .leading, endPoint: .trailing)
                    .frame(width: 44, height: 4)
                    .clipShape(Capsule())
                legendText("trail, first pose → last")
            }
            HStack(spacing: 8) {
                Circle().stroke(LabTheme.good, lineWidth: 2).frame(width: 10, height: 10).frame(width: 44)
                legendText("start")
                Rectangle().stroke(LabTheme.warn, lineWidth: 2).frame(width: 9, height: 9).padding(.leading, 6)
                legendText("end")
            }
            HStack(spacing: 8) {
                RoundedRectangle(cornerRadius: 2).fill(LabTheme.text.opacity(0.85)).frame(width: 44, height: 6)
                legendText("walls and furniture (cart height)")
            }
            HStack(spacing: 8) {
                RoundedRectangle(cornerRadius: 2).fill(LabTheme.accent.opacity(0.35)).frame(width: 44, height: 6)
                legendText("all map points (floor, ceiling)")
            }
        }
    }

    private func legendText(_ text: String) -> some View {
        Text(text)
            .font(LabTheme.t(11))
            .foregroundStyle(LabTheme.dim)
    }

    // MARK: - Canvas

    private func canvas(sample: TrailSample?) -> some View {
        GeometryReader { geo in
            let frame = viewFrame(size: geo.size)
            Canvas { context, _ in
                if let frame {
                    TrailDrawing(player: player, frame: frame, sample: sample).draw(&context)
                }
            }
            .overlay(alignment: .topTrailing) {
                if let summary = player.summary, let sample {
                    Text("\(TrailReadout.clock(sample.elapsed)) / \(TrailReadout.clock(summary.duration))")
                        .font(LabTheme.mono(13))
                        .monospacedDigit()
                        .foregroundStyle(LabTheme.text)
                        .padding(.horizontal, 8)
                        .padding(.vertical, 4)
                        .background(
                            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                                .fill(LabTheme.bg.opacity(0.85))
                        )
                        .padding(10)
                        .allowsHitTesting(false)
                }
            }
            .overlay(alignment: .topLeading) {
                if let label = player.session?.accuracyLabel {
                    canvasBadge(label, tint: LabTheme.warn)
                        .padding(10)
                        .allowsHitTesting(false)
                }
            }
            .overlay(alignment: .bottomTrailing) {
                HStack(spacing: 6) {
                    Text("map frame · x → · y ↑")
                        .font(LabTheme.mono(10))
                        .foregroundStyle(LabTheme.faint)
                        .padding(.trailing, 4)
                    Button("−") { zoom = max(0.25, zoom / 1.5) }
                    Button("+") { zoom = min(40, zoom * 1.5) }
                    Button("Fit") {
                        zoom = 1
                        pan = .zero
                    }
                }
                .font(LabTheme.t(11, .medium))
                .buttonStyle(QuietButton())
                .padding(10)
            }
            .contentShape(Rectangle())
            .gesture(
                DragGesture(minimumDistance: 2)
                    .onChanged { drag = $0.translation }
                    .onEnded { value in
                        pan.width += value.translation.width
                        pan.height += value.translation.height
                        drag = .zero
                    }
            )
            .simultaneousGesture(
                MagnificationGesture()
                    .onChanged { pinch = $0 }
                    .onEnded { value in
                        zoom = min(40, max(0.25, zoom * value))
                        pinch = 1
                    }
            )
            .onTapGesture(count: 2) {
                zoom = 1
                pan = .zero
            }
        }
    }

    private func canvasBadge(_ text: String, tint: Color) -> some View {
        HStack(spacing: 6) {
            Circle()
                .fill(tint)
                .frame(width: LabTheme.pip, height: LabTheme.pip)
            Text(text)
                .font(LabTheme.mono(11))
                .foregroundStyle(tint)
                .lineLimit(1)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .fill(LabTheme.bg.opacity(0.85))
        )
        .overlay(
            RoundedRectangle(cornerRadius: LabTheme.controlRadius, style: .continuous)
                .stroke(tint.opacity(0.6), lineWidth: 1)
        )
    }

    /// World → view for this canvas size, zoom and pan. Nil before a raster exists.
    private func viewFrame(size: CGSize) -> TrailViewFrame? {
        guard let raster = player.raster, size.width > 1, size.height > 1 else { return nil }
        let inset: CGFloat = 28
        let fit = min((size.width - 2 * inset) / CGFloat(raster.worldWidth),
                      (size.height - 2 * inset) / CGFloat(raster.worldHeight))
        let scale = max(fit, 1e-3) * zoom * pinch
        let center = CGPoint(x: size.width / 2 + pan.width + drag.width,
                             y: size.height / 2 + pan.height + drag.height)
        return TrailViewFrame(
            size: size,
            scale: scale,
            center: center,
            worldCenter: (raster.minX + raster.worldWidth / 2, raster.maxY - raster.worldHeight / 2)
        )
    }

    // MARK: - Transport

    private func transport(now: Double) -> some View {
        let clock = player.clock
        let elapsed = clock.elapsed(at: now)
        let hasTrail = player.trajectory != nil && clock.duration > 0
        let playTitle = clock.isPlaying ? "Pause" : (clock.atEnd(at: now) && hasTrail ? "Replay" : "Play")
        return HStack(spacing: 10) {
            Button(playTitle) { player.toggle() }
                .buttonStyle(PrimaryButton(emphasized: true))
                .frame(width: 96)
                .disabled(!hasTrail)
                .opacity(hasTrail ? 1 : LabTheme.disabledOpacity)
                .accessibilityLabel(playTitle)
            Button("Start") { player.rewind() }
                .font(LabTheme.t(12, .medium))
                .buttonStyle(QuietButton())
                .disabled(!hasTrail)
                .opacity(hasTrail ? 1 : LabTheme.disabledOpacity)
            Text(TrailReadout.clock(elapsed))
                .font(LabTheme.mono(11))
                .monospacedDigit()
                .foregroundStyle(LabTheme.dim)
                .frame(width: 56, alignment: .trailing)
            Slider(
                value: Binding(
                    get: { player.elapsed() },
                    set: { player.seek(to: $0) }
                ),
                in: 0...max(clock.duration, 0.001)
            )
            .disabled(!hasTrail)
            .opacity(hasTrail ? 1 : LabTheme.disabledOpacity)
            .accessibilityLabel("Trail playhead")
            Text(TrailReadout.clock(clock.duration))
                .font(LabTheme.mono(11))
                .monospacedDigit()
                .foregroundStyle(LabTheme.dim)
                .frame(width: 56, alignment: .leading)
            Picker("Speed", selection: Binding(
                get: { clock.speed },
                set: { player.setSpeed($0) }
            )) {
                ForEach(TrailClock.speeds, id: \.self) { speed in
                    Text(TrailReadout.speed(speed)).tag(speed)
                }
            }
            .pickerStyle(.segmented)
            .labelsHidden()
            .frame(width: 250)
            .disabled(!hasTrail)
            .opacity(hasTrail ? 1 : LabTheme.disabledOpacity)
            .accessibilityLabel("Playback speed")
        }
        .frame(height: 40)
    }
}

/// Trail colour from first pose to last: steel, green, amber. Theme colours only.
enum TrailPalette {
    struct Stop {
        var r: Double
        var g: Double
        var b: Double
        var color: Color { Color(red: r, green: g, blue: b) }
    }

    static let stops = [
        Stop(r: 0.290, g: 0.561, b: 0.722),
        Stop(r: 0.247, g: 0.639, b: 0.478),
        Stop(r: 0.769, g: 0.627, b: 0.290),
    ]

    static func color(_ f: Double) -> Color {
        let t = min(max(f, 0), 1) * Double(stops.count - 1)
        let i = min(Int(t), stops.count - 2)
        let u = t - Double(i)
        let a = stops[i], b = stops[i + 1]
        return Color(red: a.r + (b.r - a.r) * u, green: a.g + (b.g - a.g) * u, blue: a.b + (b.b - a.b) * u)
    }
}

struct TrailViewFrame {
    var size: CGSize
    /// Points per metre.
    var scale: CGFloat
    var center: CGPoint
    var worldCenter: (x: Double, y: Double)

    func point(_ p: TrailPoint) -> CGPoint {
        CGPoint(x: center.x + CGFloat(p.x - worldCenter.x) * scale,
                y: center.y - CGFloat(p.y - worldCenter.y) * scale)
    }

    func point(x: Double, y: Double) -> CGPoint { point(TrailPoint(x: x, y: y, z: 0)) }
}

/// Everything drawn on the canvas, back to front.
@MainActor
struct TrailDrawing {
    var player: TrailPlayer
    var frame: TrailViewFrame
    var sample: TrailSample?

    /// Trail colour bands. Enough that the gradient reads smooth, few enough to stroke per frame.
    static let bands = 48

    func draw(_ context: inout GraphicsContext) {
        guard let raster = player.raster else { return }
        let topLeft = frame.point(x: raster.minX, y: raster.maxY)
        let mapRect = CGRect(x: topLeft.x, y: topLeft.y,
                             width: CGFloat(raster.worldWidth) * frame.scale,
                             height: CGFloat(raster.worldHeight) * frame.scale)
        let step = gridStep()
        drawGrid(&context, raster: raster, rect: mapRect, step: step)
        if let image = player.mapImage {
            context.draw(Image(decorative: image, scale: 1).interpolation(.low), in: mapRect)
        }
        drawTrail(&context)
        drawMarkers(&context)
        drawCart(&context)
        drawScaleBar(&context, step: step)
    }

    /// The grid spacing whose cells are at least 48 points wide.
    private func gridStep() -> Double {
        for step in [0.25, 0.5, 1, 2, 5, 10, 20, 50] where CGFloat(step) * frame.scale >= 48 {
            return step
        }
        return 100
    }

    private func drawGrid(_ context: inout GraphicsContext, raster: TrailRaster, rect: CGRect, step: Double) {
        var grid = Path()
        let x0 = (raster.minX / step).rounded(.up) * step
        let x1 = raster.minX + raster.worldWidth
        var x = x0
        while x <= x1 {
            let p = frame.point(x: x, y: 0)
            grid.move(to: CGPoint(x: p.x, y: rect.minY))
            grid.addLine(to: CGPoint(x: p.x, y: rect.maxY))
            x += step
        }
        let y1 = raster.maxY
        var y = ((raster.maxY - raster.worldHeight) / step).rounded(.up) * step
        while y <= y1 {
            let p = frame.point(x: 0, y: y)
            grid.move(to: CGPoint(x: rect.minX, y: p.y))
            grid.addLine(to: CGPoint(x: rect.maxX, y: p.y))
            y += step
        }
        context.stroke(grid, with: .color(LabTheme.line.opacity(0.7)), lineWidth: 0.5)
    }

    private func drawTrail(_ context: inout GraphicsContext) {
        let vertices = player.vertices
        guard vertices.count > 1 else { return }

        // The whole route, faint, so the shape is visible before the cart gets there.
        var ghost = Path()
        ghost.addLines(vertices.map { frame.point($0.point) })
        context.stroke(ghost, with: .color(LabTheme.dim.opacity(0.35)),
                       style: StrokeStyle(lineWidth: 1.2, lineCap: .round, lineJoin: .round, dash: [3, 4]))

        guard let sample else { return }
        let elapsedFraction = player.clock.duration > 0 ? sample.elapsed / player.clock.duration : 1
        var travelled: [(point: CGPoint, fraction: Double)] = []
        for v in vertices where v.index <= sample.index {
            travelled.append((frame.point(v.point), v.fraction))
        }
        travelled.append((frame.point(sample.position), elapsedFraction))
        guard travelled.count > 1 else { return }

        var glow = Path()
        glow.addLines(travelled.map(\.point))
        context.stroke(glow, with: .color(LabTheme.accent.opacity(0.16)),
                       style: StrokeStyle(lineWidth: 8, lineCap: .round, lineJoin: .round))

        // One path per colour band, each segment in the band of its start.
        var bands = [Path](repeating: Path(), count: Self.bands)
        for i in 0..<(travelled.count - 1) {
            let band = min(Self.bands - 1, Int(travelled[i].fraction * Double(Self.bands)))
            bands[band].move(to: travelled[i].point)
            bands[band].addLine(to: travelled[i + 1].point)
        }
        for (i, path) in bands.enumerated() where !path.isEmpty {
            let color = TrailPalette.color((Double(i) + 0.5) / Double(Self.bands))
            context.stroke(path, with: .color(color), style: StrokeStyle(lineWidth: 2.6, lineCap: .round, lineJoin: .round))
        }
    }

    private func drawMarkers(_ context: inout GraphicsContext) {
        guard let trajectory = player.trajectory, let first = trajectory.start, let last = trajectory.end else { return }
        let start = frame.point(first.position)
        let end = frame.point(last.position)
        let reached = player.clock.duration <= 0 || (sample?.elapsed ?? 0) >= player.clock.duration - 1e-6

        if reached {
            // Start → end, the raw closure, drawn only once the cart is there.
            var link = Path()
            link.move(to: start)
            link.addLine(to: end)
            context.stroke(link, with: .color(LabTheme.text.opacity(0.7)), style: StrokeStyle(lineWidth: 1, dash: [2, 2]))
            let distance = first.position.distance(to: last.position)
            // On a plate, so the label stays readable over dense map points.
            let label = context.resolve(Text("start → end \(TrailReadout.metres(distance)) raw")
                .font(LabTheme.mono(11))
                .foregroundStyle(LabTheme.text))
            let size = label.measure(in: CGSize(width: 400, height: 40))
            let origin = CGPoint(x: max(start.x, end.x) + 16, y: min(start.y, end.y) - 18 - size.height / 2)
            let plate = CGRect(origin: origin, size: size).insetBy(dx: -5, dy: -3)
            context.fill(Path(roundedRect: plate, cornerRadius: 4), with: .color(LabTheme.bg.opacity(0.85)))
            context.draw(label, at: origin, anchor: .topLeading)
        }

        let ring = CGRect(x: start.x - 7, y: start.y - 7, width: 14, height: 14)
        context.fill(Path(ellipseIn: ring), with: .color(LabTheme.good.opacity(0.18)))
        context.stroke(Path(ellipseIn: ring), with: .color(LabTheme.good), lineWidth: 2)
        context.draw(Text("start").font(LabTheme.mono(10)).foregroundStyle(LabTheme.good),
                     at: CGPoint(x: start.x - 11, y: start.y + 2), anchor: .trailing)

        let square = CGRect(x: end.x - 5.5, y: end.y - 5.5, width: 11, height: 11)
        context.stroke(Path(square), with: .color(LabTheme.warn.opacity(reached ? 1 : 0.45)), lineWidth: 2)
        context.draw(Text("end").font(LabTheme.mono(10)).foregroundStyle(LabTheme.warn.opacity(reached ? 1 : 0.6)),
                     at: CGPoint(x: end.x - 11, y: end.y + 14), anchor: .trailing)
    }

    private func drawCart(_ context: inout GraphicsContext) {
        guard let sample else { return }
        let c = frame.point(sample.position)
        context.fill(Path(ellipseIn: CGRect(x: c.x - 13, y: c.y - 13, width: 26, height: 26)),
                     with: .color(LabTheme.text.opacity(0.08)))
        // Screen y points down, so a counter-clockwise map yaw is a negative screen angle.
        let yaw = -sample.yaw
        func at(_ forward: CGFloat, _ side: CGFloat) -> CGPoint {
            CGPoint(x: c.x + forward * CGFloat(cos(yaw)) - side * CGFloat(sin(yaw)),
                    y: c.y + forward * CGFloat(sin(yaw)) + side * CGFloat(cos(yaw)))
        }
        var arrow = Path()
        arrow.move(to: at(11, 0))
        arrow.addLine(to: at(-6, 7))
        arrow.addLine(to: at(-3, 0))
        arrow.addLine(to: at(-6, -7))
        arrow.closeSubpath()
        context.fill(arrow, with: .color(LabTheme.text))
        context.stroke(arrow, with: .color(LabTheme.bg), lineWidth: 1)
    }

    private func drawScaleBar(_ context: inout GraphicsContext, step: Double) {
        let length = CGFloat(step) * frame.scale
        let origin = CGPoint(x: 18, y: frame.size.height - 22)
        var bar = Path()
        bar.move(to: CGPoint(x: origin.x, y: origin.y - 4))
        bar.addLine(to: origin)
        bar.addLine(to: CGPoint(x: origin.x + length, y: origin.y))
        bar.addLine(to: CGPoint(x: origin.x + length, y: origin.y - 4))
        context.stroke(bar, with: .color(LabTheme.dim), lineWidth: 1.2)
        let label = step < 1 ? String(format: "%.0f cm", step * 100) : String(format: "%.0f m", step)
        context.draw(Text(label).font(LabTheme.mono(10)).foregroundStyle(LabTheme.dim),
                     at: CGPoint(x: origin.x + length + 6, y: origin.y - 2), anchor: .leading)
    }
}
