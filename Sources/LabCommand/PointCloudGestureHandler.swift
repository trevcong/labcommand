import SwiftUI
import AppKit

/// SwiftUI view modifier that handles camera gestures for the point cloud viewer.
/// Captures trackpad gestures (rotate, zoom, pan) and keyboard input (WASD, F to toggle mode).
@MainActor
struct PointCloudGestureHandler: ViewModifier {
    @ObservedObject var camera: PointCloudCamera
    
    @State private var lastDragLocation: CGPoint?
    @State private var lastMagnification: CGFloat = 1.0
    @State private var keyDownMonitor: Any?
    @State private var keyUpMonitor: Any?
    @State private var flagsMonitor: Any?
    @State private var displayLink: CVDisplayLink?
    @State private var lastFrameTime: CFTimeInterval = 0
    @State private var isShiftDown: Bool = false
    
    func body(content: Content) -> some View {
        content
            .gesture(rotateGesture)
            .gesture(magnifyGesture)
            .background(ScrollWheelCaptureView(camera: camera))
            .onAppear {
                installEventMonitors()
                startDisplayLink()
            }
            .onDisappear {
                removeEventMonitors()
                stopDisplayLink()
            }
    }
    
    /// Two-finger drag: rotate in orbit mode, mouse look in fly mode
    private var rotateGesture: some Gesture {
        DragGesture(minimumDistance: 1)
            .onChanged { value in
                let currentLocation = value.location
                let delta: CGSize
                
                if let last = lastDragLocation {
                    delta = CGSize(
                        width: currentLocation.x - last.x,
                        height: currentLocation.y - last.y
                    )
                } else {
                    delta = .zero
                }
                
                lastDragLocation = currentLocation
                
                // Option+drag = pan, otherwise rotate
                if NSEvent.modifierFlags.contains(.option) {
                    camera.handlePan(delta)
                } else if camera.mode == .fly {
                    camera.handleMouseDelta(delta)
                } else {
                    camera.handleRotation(delta)
                }
            }
            .onEnded { _ in
                lastDragLocation = nil
            }
    }
    
    /// Pinch: zoom in orbit mode
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
    
    // MARK: - Display Link for update(deltaTime:)
    
    private func startDisplayLink() {
        var link: CVDisplayLink?
        CVDisplayLinkCreateWithActiveCGDisplays(&link)
        guard let displayLink = link else { return }
        
        let displayLinkCallback: CVDisplayLinkOutputCallback = { _, inNow, _, _, _, userInfo in
            let now = inNow.pointee
            let timeNow = Double(now.videoTime) / Double(now.videoTimeScale)
            
            DispatchQueue.main.async {
                guard let camera = userInfo?.assumingMemoryBound(to: PointCloudCamera.self).pointee else { return }
                // Use a fixed timestep for stability
                camera.update(deltaTime: 1.0 / 60.0)
            }
            return kCVReturnSuccess
        }
        
        // Note: We can't easily pass camera to the callback, so use a simpler approach
        CVDisplayLinkSetOutputCallback(displayLink, displayLinkCallback, nil)
        CVDisplayLinkStart(displayLink)
        self.displayLink = displayLink
        
        // Use a Timer instead for simpler MainActor-compatible updates
        stopDisplayLink()
        startUpdateTimer()
    }
    
    private func stopDisplayLink() {
        if let link = displayLink {
            CVDisplayLinkStop(link)
            displayLink = nil
        }
    }
    
    @State private var updateTimer: Timer?
    
    private func startUpdateTimer() {
        updateTimer = Timer.scheduledTimer(withTimeInterval: 1.0/60.0, repeats: true) { _ in
            Task { @MainActor in
                camera.update(deltaTime: 1.0 / 60.0)
            }
        }
    }
    
    private func stopUpdateTimer() {
        updateTimer?.invalidate()
        updateTimer = nil
    }
    
    // MARK: - Event Monitors
    
    private func installEventMonitors() {
        keyDownMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyDown) { event in
            handleKeyDown(event)
            return event
        }
        
        keyUpMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyUp) { event in
            handleKeyUp(event)
            return event
        }
        
        flagsMonitor = NSEvent.addLocalMonitorForEvents(matching: .flagsChanged) { event in
            handleFlagsChanged(event)
            return event
        }
    }
    
    private func removeEventMonitors() {
        if let monitor = keyDownMonitor {
            NSEvent.removeMonitor(monitor)
            keyDownMonitor = nil
        }
        if let monitor = keyUpMonitor {
            NSEvent.removeMonitor(monitor)
            keyUpMonitor = nil
        }
        if let monitor = flagsMonitor {
            NSEvent.removeMonitor(monitor)
            flagsMonitor = nil
        }
        
        stopDisplayLink()
        stopUpdateTimer()
        camera.stopMovement()
    }
    
    private func handleKeyDown(_ event: NSEvent) {
        // Ignore if a text field has focus
        if let responder = NSApp.keyWindow?.firstResponder,
           responder is NSTextView || responder is NSTextField {
            return
        }
        
        // Ignore keys with command or control modifiers
        let ignoredModifiers: NSEvent.ModifierFlags = [.command, .control]
        if !event.modifierFlags.intersection(ignoredModifiers).isEmpty {
            return
        }
        
        guard !event.isARepeat else { return }
        
        // F toggles camera mode (no modifiers)
        if event.charactersIgnoringModifiers?.lowercased() == "f" &&
           event.modifierFlags.intersection([.shift, .option]).isEmpty {
            camera.toggleMode()
            return
        }
        
        // R resets camera to fit bounds (no modifiers)
        if event.charactersIgnoringModifiers?.lowercased() == "r" &&
           event.modifierFlags.intersection([.shift, .option]).isEmpty {
            camera.resetToFitBounds()
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
        // Ignore if a text field has focus
        if let responder = NSApp.keyWindow?.firstResponder,
           responder is NSTextView || responder is NSTextField {
            return
        }
        
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
        
        let shiftNowDown = event.modifierFlags.contains(.shift)
        
        if shiftNowDown && !isShiftDown {
            // Shift pressed - move down
            camera.setMoveUp(-1)
        } else if !shiftNowDown && isShiftDown {
            // Shift released - stop descending
            camera.setMoveUp(0)
        }
        
        isShiftDown = shiftNowDown
    }
}

extension View {
    func pointCloudGestures(camera: PointCloudCamera) -> some View {
        modifier(PointCloudGestureHandler(camera: camera))
    }
}

/// NSView that captures scroll wheel events for zoom.
/// Uses .background placement to avoid blocking drag gestures.
struct ScrollWheelCaptureView: NSViewRepresentable {
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

final class ScrollWheelNSView: NSView {
    weak var camera: PointCloudCamera?
    
    override var acceptsFirstResponder: Bool { true }
    
    override func scrollWheel(with event: NSEvent) {
        Task { @MainActor in
            camera?.handleScroll(CGSize(width: event.scrollingDeltaX, height: event.scrollingDeltaY))
        }
    }
}

/// A view that captures continuous mouse movement for fly mode camera control.
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

final class MouseTrackingNSView: NSView {
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
