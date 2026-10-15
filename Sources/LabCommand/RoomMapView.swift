import AppKit
import MetalKit
import SwiftUI
import LabCommandCore

/// The Room canvas. Drag orbits around z, scroll dollies, double-click fits once.
final class RoomMapCanvas: MTKView {
    var onCanvasRect: ((Int, Int, Int, Int) -> Void)?
    weak var map: RoomMapRenderer?
    private var drag: NSPoint?
    private var lastRect: ReportedRect?

    private struct ReportedRect: Equatable {
        var x: Int
        var y: Int
        var w: Int
        var h: Int
    }

    override var acceptsFirstResponder: Bool { true }

    override func viewDidMoveToWindow() {
        super.viewDidMoveToWindow()
        window?.makeFirstResponder(self)
        reportRect()
    }

    override func layout() {
        super.layout()
        reportRect()
    }

    override func setFrameSize(_ newSize: NSSize) {
        super.setFrameSize(newSize)
        reportRect()
    }

    override func mouseDown(with event: NSEvent) {
        window?.makeFirstResponder(self)
        if event.clickCount >= 2 {
            map?.refit()
            return
        }
        drag = convert(event.locationInWindow, from: nil)
    }

    override func mouseDragged(with event: NSEvent) {
        let point = convert(event.locationInWindow, from: nil)
        let last = drag ?? point
        map?.orbit(dx: Float(point.x - last.x), dy: Float(point.y - last.y))
        drag = point
    }

    override func scrollWheel(with event: NSEvent) {
        let scale: CGFloat = event.hasPreciseScrollingDeltas ? 1 : 8
        map?.dolly(Float(event.scrollingDeltaY * scale))
    }

    /// Metal view bounds in the captured window, PNG pixels, origin top-left.
    /// `screencapture -l -o` omits the shadow, so the image is the window frame.
    func reportRect() {
        guard let window, bounds.width > 1, bounds.height > 1 else { return }
        let scale = window.backingScaleFactor
        guard scale > 0 else { return }
        let onScreen = window.convertToScreen(convert(bounds, to: nil))
        let frame = window.frame
        let rect = ReportedRect(
            x: Int(((onScreen.minX - frame.minX) * scale).rounded()),
            y: Int(((frame.maxY - onScreen.maxY) * scale).rounded()),
            w: Int((onScreen.width * scale).rounded()),
            h: Int((onScreen.height * scale).rounded())
        )
        guard rect != lastRect else { return }
        lastRect = rect
        onCanvasRect?(rect.x, rect.y, rect.w, rect.h)
    }
}

struct RoomMapView: NSViewRepresentable {
    var mirror: RoomMemoryMirror
    var markers: [LiveRoomMarker]
    var opacity: Float
    var drawMap: Bool
    var onCanvasRect: (Int, Int, Int, Int) -> Void

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeNSView(context: Context) -> NSView {
        let renderer = RoomMapRenderer()
        context.coordinator.renderer = renderer
        guard let device = renderer.device else {
            let placeholder = NSView()
            placeholder.wantsLayer = true
            placeholder.layer?.backgroundColor = NSColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1).cgColor
            return placeholder
        }
        let view = RoomMapCanvas(frame: .zero, device: device)
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float
        view.clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1)
        view.preferredFramesPerSecond = 60
        view.enableSetNeedsDisplay = false
        view.isPaused = false
        view.delegate = renderer
        view.map = renderer
        view.onCanvasRect = onCanvasRect
        context.coordinator.canvas = view
        return view
    }

    func updateNSView(_ nsView: NSView, context: Context) {
        context.coordinator.canvas?.onCanvasRect = onCanvasRect
        context.coordinator.renderer?.update(mirror: mirror, markers: markers, opacity: opacity, drawMap: drawMap)
        context.coordinator.canvas?.reportRect()
    }

    final class Coordinator {
        var renderer: RoomMapRenderer?
        var canvas: RoomMapCanvas?
    }
}
