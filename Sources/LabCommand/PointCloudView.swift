import SwiftUI
import MetalKit
import LabCommandCore

/// SwiftUI wrapper for Metal-based point cloud rendering with camera controls.
/// Displays point clouds from PCD fixtures or live streams.
public struct PointCloudView: NSViewRepresentable {
    
    /// The point cloud to render. Nil displays an empty canvas.
    @Binding public var frame: PointCloudFrame?
    
    /// Color mode for visualization.
    public var colorMode: PointCloudColorMode
    
    /// Point size in pixels.
    public var pointSize: Float
    
    public init(
        frame: Binding<PointCloudFrame?>,
        colorMode: PointCloudColorMode = .intensity,
        pointSize: Float = 4.0
    ) {
        self._frame = frame
        self.colorMode = colorMode
        self.pointSize = pointSize
    }
    
    public func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    public func makeNSView(context: Context) -> NSView {
        let renderer = PointCloudRenderer()
        let camera = PointCloudCamera()
        context.coordinator.renderer = renderer
        context.coordinator.camera = camera
        
        guard let device = renderer.device else {
            let placeholder = NSView()
            placeholder.wantsLayer = true
            placeholder.layer?.backgroundColor = NSColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1).cgColor
            return placeholder
        }
        
        let mtkView = PointCloudMTKView(frame: .zero, device: device)
        mtkView.camera = camera
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0)
        mtkView.preferredFramesPerSecond = 60
        mtkView.enableSetNeedsDisplay = false
        mtkView.isPaused = false
        mtkView.delegate = renderer
        
        context.coordinator.mtkView = mtkView
        
        // Start camera update timer
        context.coordinator.startUpdateTimer(camera: camera, renderer: renderer)
        
        return mtkView
    }
    
    public func updateNSView(_ nsView: NSView, context: Context) {
        guard let renderer = context.coordinator.renderer else { return }
        
        renderer.colorMode = colorMode
        renderer.pointSize = pointSize
        
        let currentTimestamp = frame?.timestamp
        if context.coordinator.lastTimestamp != currentTimestamp || context.coordinator.lastCount != frame?.count {
            if let f = frame {
                renderer.load(f)
                // Update camera bounds from point cloud
                if let camera = context.coordinator.camera {
                    let bounds = f.bounds
                    if bounds.min.x < bounds.max.x {
                        camera.setBounds(
                            min: Float3(bounds.min.x, bounds.min.y, bounds.min.z),
                            max: Float3(bounds.max.x, bounds.max.y, bounds.max.z)
                        )
                        // Reset to fit on first load
                        if !context.coordinator.hasInitializedCamera {
                            camera.resetToFitBounds()
                            context.coordinator.hasInitializedCamera = true
                        }
                    }
                }
            } else {
                renderer.load(PointCloudFrame())
            }
            context.coordinator.lastTimestamp = currentTimestamp
            context.coordinator.lastCount = frame?.count ?? 0
        }
    }
    
    public class Coordinator: NSObject {
        var renderer: PointCloudRenderer?
        var camera: PointCloudCamera?
        var mtkView: PointCloudMTKView?
        var lastTimestamp: Date?
        var lastCount: Int = 0
        var updateTimer: Timer?
        var hasInitializedCamera: Bool = false
        
        func startUpdateTimer(camera: PointCloudCamera, renderer: PointCloudRenderer) {
            updateTimer = Timer.scheduledTimer(withTimeInterval: 1.0/60.0, repeats: true) { [weak self] _ in
                guard let self = self else { return }
                Task { @MainActor in
                    camera.update(deltaTime: 1.0 / 60.0)
                    renderer.setViewMatrix(camera.viewMatrix)
                }
            }
        }
        
        deinit {
            updateTimer?.invalidate()
        }
    }
}

/// Custom MTKView that handles scroll wheel for zoom and keyboard for camera.
final class PointCloudMTKView: MTKView {
    weak var camera: PointCloudCamera?
    private var keyDownMonitor: Any?
    private var keyUpMonitor: Any?
    private var flagsMonitor: Any?
    private var isShiftDown: Bool = false
    private var lastDragLocation: CGPoint?
    
    override init(frame frameRect: CGRect, device: MTLDevice?) {
        super.init(frame: frameRect, device: device)
        setupEventMonitors()
    }
    
    required init(coder: NSCoder) {
        super.init(coder: coder)
        setupEventMonitors()
    }
    
    deinit {
        removeEventMonitors()
    }
    
    private func setupEventMonitors() {
        keyDownMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyDown) { [weak self] event in
            self?.handleKeyDown(event)
            return event
        }
        
        keyUpMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyUp) { [weak self] event in
            self?.handleKeyUp(event)
            return event
        }
        
        flagsMonitor = NSEvent.addLocalMonitorForEvents(matching: .flagsChanged) { [weak self] event in
            self?.handleFlagsChanged(event)
            return event
        }
    }
    
    private func removeEventMonitors() {
        if let monitor = keyDownMonitor {
            NSEvent.removeMonitor(monitor)
        }
        if let monitor = keyUpMonitor {
            NSEvent.removeMonitor(monitor)
        }
        if let monitor = flagsMonitor {
            NSEvent.removeMonitor(monitor)
        }
        camera?.stopMovement()
    }
    
    override var acceptsFirstResponder: Bool { true }
    
    // MARK: - Scroll Wheel (Zoom)
    
    override func scrollWheel(with event: NSEvent) {
        Task { @MainActor in
            camera?.handleScroll(CGSize(width: event.scrollingDeltaX, height: event.scrollingDeltaY))
        }
    }
    
    // MARK: - Mouse Drag (Rotate/Pan)
    
    override func mouseDown(with event: NSEvent) {
        lastDragLocation = convert(event.locationInWindow, from: nil)
    }
    
    override func mouseDragged(with event: NSEvent) {
        let currentLocation = convert(event.locationInWindow, from: nil)
        let delta: CGSize
        
        if let last = lastDragLocation {
            delta = CGSize(
                width: currentLocation.x - last.x,
                height: -(currentLocation.y - last.y)  // Flip Y for natural feel
            )
        } else {
            delta = .zero
        }
        
        lastDragLocation = currentLocation
        
        Task { @MainActor in
            guard let camera = camera else { return }
            
            if NSEvent.modifierFlags.contains(.option) {
                camera.handlePan(delta)
            } else if camera.mode == .fly {
                camera.handleMouseDelta(delta)
            } else {
                camera.handleRotation(delta)
            }
        }
    }
    
    override func mouseUp(with event: NSEvent) {
        lastDragLocation = nil
    }
    
    // MARK: - Magnify (Pinch Zoom)
    
    override func magnify(with event: NSEvent) {
        Task { @MainActor in
            camera?.handlePinch(1.0 + CGFloat(event.magnification))
        }
    }
    
    // MARK: - Keyboard
    
    private func handleKeyDown(_ event: NSEvent) {
        guard window?.firstResponder === self || window?.firstResponder is MTKView else { return }
        
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
        
        Task { @MainActor in
            guard let camera = camera else { return }
            
            // F toggles camera mode
            if event.charactersIgnoringModifiers?.lowercased() == "f" &&
               event.modifierFlags.intersection([.shift, .option]).isEmpty {
                camera.toggleMode()
                return
            }
            
            // R resets camera to fit bounds
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
    }
    
    private func handleKeyUp(_ event: NSEvent) {
        if let responder = NSApp.keyWindow?.firstResponder,
           responder is NSTextView || responder is NSTextField {
            return
        }
        
        Task { @MainActor in
            guard let camera = camera, camera.mode == .fly else { return }
            
            switch event.charactersIgnoringModifiers?.lowercased() {
            case "w", "s": camera.setMoveForward(0)
            case "a", "d": camera.setMoveRight(0)
            case " ": camera.setMoveUp(0)
            default: break
            }
        }
    }
    
    private func handleFlagsChanged(_ event: NSEvent) {
        Task { @MainActor in
            guard let camera = camera, camera.mode == .fly else { return }
            
            let shiftNowDown = event.modifierFlags.contains(.shift)
            
            if shiftNowDown && !isShiftDown {
                camera.setMoveUp(-1)
            } else if !shiftNowDown && isShiftDown {
                camera.setMoveUp(0)
            }
            
            isShiftDown = shiftNowDown
        }
    }
}

/// Multiple point clouds overlaid with sensor coloring.
public struct DualPointCloudView: NSViewRepresentable {
    
    /// Mid-360 point cloud (sensor ID 0, cyan).
    @Binding public var midFrame: PointCloudFrame?
    
    /// JT128 point cloud (sensor ID 1, magenta).
    @Binding public var jtFrame: PointCloudFrame?
    
    /// Color mode for visualization.
    public var colorMode: PointCloudColorMode
    
    /// Point size in pixels.
    public var pointSize: Float
    
    public init(
        midFrame: Binding<PointCloudFrame?>,
        jtFrame: Binding<PointCloudFrame?>,
        colorMode: PointCloudColorMode = .sensor,
        pointSize: Float = 4.0
    ) {
        self._midFrame = midFrame
        self._jtFrame = jtFrame
        self.colorMode = colorMode
        self.pointSize = pointSize
    }
    
    public func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    public func makeNSView(context: Context) -> NSView {
        let renderer = PointCloudRenderer()
        let camera = PointCloudCamera()
        context.coordinator.renderer = renderer
        context.coordinator.camera = camera
        
        guard let device = renderer.device else {
            let placeholder = NSView()
            placeholder.wantsLayer = true
            placeholder.layer?.backgroundColor = NSColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1).cgColor
            return placeholder
        }
        
        let mtkView = PointCloudMTKView(frame: .zero, device: device)
        mtkView.camera = camera
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0)
        mtkView.preferredFramesPerSecond = 60
        mtkView.enableSetNeedsDisplay = false
        mtkView.isPaused = false
        mtkView.delegate = renderer
        
        context.coordinator.mtkView = mtkView
        context.coordinator.startUpdateTimer(camera: camera, renderer: renderer)
        
        return mtkView
    }
    
    public func updateNSView(_ nsView: NSView, context: Context) {
        guard let renderer = context.coordinator.renderer else { return }
        
        renderer.colorMode = colorMode
        renderer.pointSize = pointSize
        
        let midTs = midFrame?.timestamp
        let jtTs = jtFrame?.timestamp
        let midChanged = context.coordinator.lastMidTimestamp != midTs || context.coordinator.lastMidCount != midFrame?.count
        let jtChanged = context.coordinator.lastJtTimestamp != jtTs || context.coordinator.lastJtCount != jtFrame?.count
        
        if midChanged || jtChanged {
            var frames: [PointCloudFrame] = []
            if let mid = midFrame {
                var f = mid
                f.sensor = .mid360
                frames.append(f)
            }
            if let jt = jtFrame {
                var f = jt
                f.sensor = .jt128
                frames.append(f)
            }
            renderer.load(frames: frames)
            
            // Update camera bounds from combined frames
            if let camera = context.coordinator.camera {
                var minP = Point3D(x: .greatestFiniteMagnitude, y: .greatestFiniteMagnitude, z: .greatestFiniteMagnitude)
                var maxP = Point3D(x: -.greatestFiniteMagnitude, y: -.greatestFiniteMagnitude, z: -.greatestFiniteMagnitude)
                for f in frames {
                    let b = f.bounds
                    minP.x = min(minP.x, b.min.x)
                    minP.y = min(minP.y, b.min.y)
                    minP.z = min(minP.z, b.min.z)
                    maxP.x = max(maxP.x, b.max.x)
                    maxP.y = max(maxP.y, b.max.y)
                    maxP.z = max(maxP.z, b.max.z)
                }
                if minP.x < maxP.x {
                    camera.setBounds(
                        min: Float3(minP.x, minP.y, minP.z),
                        max: Float3(maxP.x, maxP.y, maxP.z)
                    )
                    // Reset to fit on first load
                    if !context.coordinator.hasInitializedCamera {
                        camera.resetToFitBounds()
                        context.coordinator.hasInitializedCamera = true
                    }
                }
            }
            
            context.coordinator.lastMidTimestamp = midTs
            context.coordinator.lastJtTimestamp = jtTs
            context.coordinator.lastMidCount = midFrame?.count ?? 0
            context.coordinator.lastJtCount = jtFrame?.count ?? 0
        }
    }
    
    public class Coordinator: NSObject {
        var renderer: PointCloudRenderer?
        var camera: PointCloudCamera?
        var mtkView: PointCloudMTKView?
        var lastMidTimestamp: Date?
        var lastJtTimestamp: Date?
        var lastMidCount: Int = 0
        var lastJtCount: Int = 0
        var updateTimer: Timer?
        var hasInitializedCamera: Bool = false
        
        func startUpdateTimer(camera: PointCloudCamera, renderer: PointCloudRenderer) {
            updateTimer = Timer.scheduledTimer(withTimeInterval: 1.0/60.0, repeats: true) { [weak self] _ in
                guard let self = self else { return }
                Task { @MainActor in
                    camera.update(deltaTime: 1.0 / 60.0)
                    renderer.setViewMatrix(camera.viewMatrix)
                }
            }
        }
        
        deinit {
            updateTimer?.invalidate()
        }
    }
}

// MARK: - Preview Helpers

#if DEBUG
func makeTestPointCloudFrame(pointCount: Int = 10000, spread: Float = 5.0) -> PointCloudFrame {
    var positions: [Point3D] = []
    var intensities: [Float] = []
    
    for _ in 0..<pointCount {
        let theta = Float.random(in: 0..<Float.pi * 2)
        let phi = Float.random(in: 0..<Float.pi)
        let r = Float.random(in: 0.5..<spread)
        
        let x = r * sin(phi) * cos(theta)
        let y = r * sin(phi) * sin(theta)
        let z = r * cos(phi)
        
        positions.append(Point3D(x: x, y: y, z: z))
        intensities.append(Float.random(in: 0..<1))
    }
    
    return PointCloudFrame(positions: positions, intensities: intensities, timestamp: Date())
}

struct PointCloudView_Previews: PreviewProvider {
    static var previews: some View {
        PointCloudPreviewWrapper()
            .frame(width: 800, height: 600)
    }
}

private struct PointCloudPreviewWrapper: View {
    @State private var frame: PointCloudFrame? = makeTestPointCloudFrame()
    @State private var colorMode: PointCloudColorMode = .height
    
    var body: some View {
        VStack(spacing: 0) {
            PointCloudView(
                frame: $frame,
                colorMode: colorMode,
                pointSize: 4.0
            )
            
            HStack {
                ForEach(PointCloudColorMode.allCases, id: \.rawValue) { mode in
                    Button(mode.label) {
                        colorMode = mode
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(colorMode == mode ? .blue : .gray)
                }
            }
            .padding()
            .background(Color(red: 0.043, green: 0.051, blue: 0.063))
        }
    }
}
#endif
