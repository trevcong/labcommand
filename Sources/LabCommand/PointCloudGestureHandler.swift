import SwiftUI
import AppKit

/// SwiftUI view modifier that handles camera gestures for the point cloud viewer.
/// Captures trackpad gestures (rotate, zoom, pan) and keyboard input (WASD, F to toggle mode).
struct PointCloudGestureHandler: ViewModifier {
    @ObservedObject var camera: PointCloudCamera
    
    @State private var lastMagnification: CGFloat = 1.0
    @State private var lastRotation: CGFloat = 0.0
    @State private var isPanning = false
    
    func body(content: Content) -> some View {
        content
            .gesture(dragGesture)
            .gesture(magnifyGesture)
            .onAppear {
                NSEvent.addLocalMonitorForEvents(matching: .keyDown) { event in
                    handleKeyDown(event)
                    return event
                }
                NSEvent.addLocalMonitorForEvents(matching: .keyUp) { event in
                    handleKeyUp(event)
                    return event
                }
                NSEvent.addLocalMonitorForEvents(matching: .flagsChanged) { event in
                    handleFlagsChanged(event)
                    return event
                }
            }
    }
    
    private var dragGesture: some Gesture {
        DragGesture(minimumDistance: 1)
            .onChanged { value in
                let delta = CGSize(
                    width: value.translation.width,
                    height: value.translation.height
                )
                
                if NSEvent.modifierFlags.contains(.option) || isPanning {
                    camera.handlePan(delta)
                } else if camera.mode == .fly {
                    camera.handleMouseDelta(delta)
                } else {
                    camera.handleRotation(delta)
                }
            }
            .onEnded { _ in
                isPanning = false
            }
    }
    
    private var magnifyGesture: some Gesture {
        MagnifyGesture()
            .onChanged { value in
                let scale = value.magnification / lastMagnification
                camera.handlePinch(scale)
                lastMagnification = value.magnification
            }
            .onEnded { _ in
                lastMagnification = 1.0
            }
    }
    
    private func handleKeyDown(_ event: NSEvent) {
        guard !event.isARepeat else { return }
        
        if event.charactersIgnoringModifiers?.lowercased() == "f" {
            camera.toggleMode()
            return
        }
        
        guard camera.mode == .fly else { return }
        
        switch event.charactersIgnoringModifiers?.lowercased() {
        case "w": camera.setMoveForward(1)
        case "s": camera.setMoveForward(-1)
        case "a": camera.setMoveRight(-1)
        case "d": camera.setMoveRight(1)
        case " ": camera.setMoveUp(1)
        default: break
        }
    }
    
    private func handleKeyUp(_ event: NSEvent) {
        guard camera.mode == .fly else { return }
        
        switch event.charactersIgnoringModifiers?.lowercased() {
        case "w", "s": camera.setMoveForward(0)
        case "a", "d": camera.setMoveRight(0)
        case " ": camera.setMoveUp(0)
        default: break
        }
    }
    
    private func handleFlagsChanged(_ event: NSEvent) {
        guard camera.mode == .fly else { return }
        
        if event.modifierFlags.contains(.shift) {
            camera.setMoveUp(-1)
        } else {
            if camera.mode == .fly {
                camera.setMoveUp(0)
            }
        }
    }
}

extension View {
    func pointCloudGestures(camera: PointCloudCamera) -> some View {
        modifier(PointCloudGestureHandler(camera: camera))
    }
}

/// A view that captures mouse movement for fly mode camera control.
/// Use this when you want continuous mouse tracking (mouse capture).
struct MouseTrackingView: NSViewRepresentable {
    let camera: PointCloudCamera
    let isActive: Bool
    
    func makeNSView(context: Context) -> MouseTrackingNSView {
        let view = MouseTrackingNSView()
        view.camera = camera
        view.isTrackingActive = isActive
        return view
    }
    
    func updateNSView(_ nsView: MouseTrackingNSView, context: Context) {
        nsView.camera = camera
        nsView.isTrackingActive = isActive
    }
}

class MouseTrackingNSView: NSView {
    weak var camera: PointCloudCamera?
    var isTrackingActive = false
    private var trackingArea: NSTrackingArea?
    
    override func updateTrackingAreas() {
        super.updateTrackingAreas()
        
        if let existing = trackingArea {
            removeTrackingArea(existing)
        }
        
        let options: NSTrackingArea.Options = [
            .mouseMoved,
            .activeInKeyWindow,
            .inVisibleRect
        ]
        
        trackingArea = NSTrackingArea(
            rect: bounds,
            options: options,
            owner: self,
            userInfo: nil
        )
        addTrackingArea(trackingArea!)
    }
    
    override var acceptsFirstResponder: Bool { true }
    
    override func mouseMoved(with event: NSEvent) {
        guard isTrackingActive else { return }
        Task { @MainActor in
            camera?.handleMouseDelta(CGSize(width: event.deltaX, height: event.deltaY))
        }
    }
    
    override func mouseDragged(with event: NSEvent) {
        guard isTrackingActive else { return }
        Task { @MainActor in
            camera?.handleMouseDelta(CGSize(width: event.deltaX, height: event.deltaY))
        }
    }
}

/// Scroll wheel handler for zoom in orbit mode.
struct ScrollWheelHandler: NSViewRepresentable {
    let camera: PointCloudCamera
    
    func makeNSView(context: Context) -> ScrollWheelNSView {
        let view = ScrollWheelNSView()
        view.camera = camera
        return view
    }
    
    func updateNSView(_ nsView: ScrollWheelNSView, context: Context) {
        nsView.camera = camera
    }
}

class ScrollWheelNSView: NSView {
    weak var camera: PointCloudCamera?
    
    override var acceptsFirstResponder: Bool { true }
    
    override func scrollWheel(with event: NSEvent) {
        Task { @MainActor in
            camera?.handleScroll(CGSize(width: event.scrollingDeltaX, height: event.scrollingDeltaY))
        }
    }
}
