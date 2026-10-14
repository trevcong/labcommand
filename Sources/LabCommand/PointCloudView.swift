import SwiftUI
import MetalKit
import LabCommandCore

/// SwiftUI wrapper for Metal-based point cloud rendering.
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
        context.coordinator.renderer = renderer
        
        guard let device = renderer.device else {
            let placeholder = NSView()
            placeholder.wantsLayer = true
            placeholder.layer?.backgroundColor = NSColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1).cgColor
            return placeholder
        }
        
        let mtkView = MTKView(frame: .zero, device: device)
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0)
        mtkView.preferredFramesPerSecond = 60
        mtkView.enableSetNeedsDisplay = false
        mtkView.isPaused = false
        mtkView.delegate = renderer
        
        context.coordinator.mtkView = mtkView
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
            } else {
                renderer.load(PointCloudFrame())
            }
            context.coordinator.lastTimestamp = currentTimestamp
            context.coordinator.lastCount = frame?.count ?? 0
        }
    }
    
    public class Coordinator: NSObject {
        var renderer: PointCloudRenderer?
        var mtkView: MTKView?
        var lastTimestamp: Date?
        var lastCount: Int = 0
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
    
    /// Bump to force a reload when the frames changed but their stamps and counts
    /// might not have (live streams can repeat a header stamp).
    public var revision: Int
    
    /// Nil refits the camera on every load. Otherwise the camera is fitted once per
    /// distinct value, on the first load that has points — so a live stream can
    /// keep replacing the points without moving the camera.
    public var fitGeneration: Int?
    
    public init(
        midFrame: Binding<PointCloudFrame?>,
        jtFrame: Binding<PointCloudFrame?>,
        colorMode: PointCloudColorMode = .sensor,
        pointSize: Float = 4.0,
        revision: Int = 0,
        fitGeneration: Int? = nil
    ) {
        self._midFrame = midFrame
        self._jtFrame = jtFrame
        self.colorMode = colorMode
        self.pointSize = pointSize
        self.revision = revision
        self.fitGeneration = fitGeneration
    }
    
    public func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    public func makeNSView(context: Context) -> NSView {
        let renderer = PointCloudRenderer()
        context.coordinator.renderer = renderer
        
        guard let device = renderer.device else {
            let placeholder = NSView()
            placeholder.wantsLayer = true
            placeholder.layer?.backgroundColor = NSColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1).cgColor
            return placeholder
        }
        
        let mtkView = MTKView(frame: .zero, device: device)
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0)
        mtkView.preferredFramesPerSecond = 60
        mtkView.enableSetNeedsDisplay = false
        mtkView.isPaused = false
        mtkView.delegate = renderer
        
        context.coordinator.mtkView = mtkView
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
        
        let revisionChanged = context.coordinator.lastRevision != revision
        let fitPending = fitGeneration != nil && context.coordinator.lastFitGeneration != fitGeneration
        
        if midChanged || jtChanged || revisionChanged || fitPending {
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
            var fitCamera = true
            if let fitGeneration {
                fitCamera = fitPending && frames.contains { !$0.isEmpty }
                if fitCamera {
                    context.coordinator.lastFitGeneration = fitGeneration
                    let points = frames.reduce(0) { $0 + $1.count }
                    FileHandle.standardError.write(Data("[PointCloudView] camera fit to \(points) pts (fit generation \(fitGeneration))\n".utf8))
                }
            }
            renderer.load(frames: frames, fitCamera: fitCamera)
            context.coordinator.lastRevision = revision
            context.coordinator.lastMidTimestamp = midTs
            context.coordinator.lastJtTimestamp = jtTs
            context.coordinator.lastMidCount = midFrame?.count ?? 0
            context.coordinator.lastJtCount = jtFrame?.count ?? 0
        }
    }
    
    public class Coordinator: NSObject {
        var renderer: PointCloudRenderer?
        var mtkView: MTKView?
        var lastMidTimestamp: Date?
        var lastJtTimestamp: Date?
        var lastMidCount: Int = 0
        var lastJtCount: Int = 0
        var lastRevision: Int = 0
        var lastFitGeneration: Int?
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
