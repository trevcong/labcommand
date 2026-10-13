import SwiftUI
import MetalKit

/// SwiftUI wrapper for Metal-based point cloud rendering.
/// Displays point clouds from PCD fixtures or live streams.
public struct PointCloudView: NSViewRepresentable {
    
    /// The point cloud to render. Nil displays an empty canvas.
    @Binding public var pointCloud: PointCloud?
    
    /// Color mode for visualization.
    public var colorMode: PointCloudColorMode
    
    /// Point size in pixels.
    public var pointSize: Float
    
    public init(
        pointCloud: Binding<PointCloud?>,
        colorMode: PointCloudColorMode = .intensity,
        pointSize: Float = 4.0
    ) {
        self._pointCloud = pointCloud
        self.colorMode = colorMode
        self.pointSize = pointSize
    }
    
    public func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    public func makeNSView(context: Context) -> MTKView {
        guard let device = MTLCreateSystemDefaultDevice() else {
            fatalError("Metal is not supported on this device")
        }
        
        let mtkView = MTKView(frame: .zero, device: device)
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0)
        mtkView.preferredFramesPerSecond = 60
        mtkView.enableSetNeedsDisplay = false
        mtkView.isPaused = false
        
        let renderer = PointCloudRenderer()
        mtkView.delegate = renderer
        context.coordinator.renderer = renderer
        context.coordinator.mtkView = mtkView
        
        return mtkView
    }
    
    public func updateNSView(_ nsView: MTKView, context: Context) {
        guard let renderer = context.coordinator.renderer else { return }
        
        renderer.colorMode = colorMode
        renderer.pointSize = pointSize
        
        // Load new point cloud if changed
        let currentID = pointCloud?.id
        if context.coordinator.lastPointCloudID != currentID {
            if let cloud = pointCloud {
                renderer.load(cloud)
            } else {
                renderer.load(points: [], intensities: [], sensorID: nil)
            }
            context.coordinator.lastPointCloudID = currentID
        }
    }
    
    public class Coordinator: NSObject {
        var renderer: PointCloudRenderer?
        var mtkView: MTKView?
        var lastPointCloudID: UUID?
    }
}

/// Multiple point clouds overlaid with sensor coloring.
public struct DualPointCloudView: NSViewRepresentable {
    
    /// Mid-360 point cloud (sensor ID 0, cyan).
    @Binding public var midCloud: PointCloud?
    
    /// JT128 point cloud (sensor ID 1, magenta).
    @Binding public var jtCloud: PointCloud?
    
    /// Color mode for visualization.
    public var colorMode: PointCloudColorMode
    
    /// Point size in pixels.
    public var pointSize: Float
    
    public init(
        midCloud: Binding<PointCloud?>,
        jtCloud: Binding<PointCloud?>,
        colorMode: PointCloudColorMode = .sensor,
        pointSize: Float = 4.0
    ) {
        self._midCloud = midCloud
        self._jtCloud = jtCloud
        self.colorMode = colorMode
        self.pointSize = pointSize
    }
    
    public func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    public func makeNSView(context: Context) -> MTKView {
        guard let device = MTLCreateSystemDefaultDevice() else {
            fatalError("Metal is not supported on this device")
        }
        
        let mtkView = MTKView(frame: .zero, device: device)
        mtkView.colorPixelFormat = .bgra8Unorm
        mtkView.depthStencilPixelFormat = .depth32Float
        mtkView.clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0)
        mtkView.preferredFramesPerSecond = 60
        mtkView.enableSetNeedsDisplay = false
        mtkView.isPaused = false
        
        let renderer = PointCloudRenderer()
        mtkView.delegate = renderer
        context.coordinator.renderer = renderer
        context.coordinator.mtkView = mtkView
        
        return mtkView
    }
    
    public func updateNSView(_ nsView: MTKView, context: Context) {
        guard let renderer = context.coordinator.renderer else { return }
        
        renderer.colorMode = colorMode
        renderer.pointSize = pointSize
        
        // Check if either cloud changed
        let midID = midCloud?.id
        let jtID = jtCloud?.id
        let midChanged = context.coordinator.lastMidCloudID != midID
        let jtChanged = context.coordinator.lastJtCloudID != jtID
        
        if midChanged || jtChanged {
            var clouds: [(cloud: PointCloud, sensorID: UInt32)] = []
            if let mid = midCloud {
                clouds.append((mid, 0))  // Mid-360 = sensor 0
            }
            if let jt = jtCloud {
                clouds.append((jt, 1))   // JT128 = sensor 1
            }
            renderer.load(clouds: clouds)
            context.coordinator.lastMidCloudID = midID
            context.coordinator.lastJtCloudID = jtID
        }
    }
    
    public class Coordinator: NSObject {
        var renderer: PointCloudRenderer?
        var mtkView: MTKView?
        var lastMidCloudID: UUID?
        var lastJtCloudID: UUID?
    }
}

// MARK: - Preview Helpers

#if DEBUG
/// Generate a synthetic point cloud for previews and testing.
func makeTestPointCloud(pointCount: Int = 10000, spread: Float = 5.0) -> PointCloud {
    var positions: [SIMD3<Float>] = []
    var intensities: [Float] = []
    
    for _ in 0..<pointCount {
        let theta = Float.random(in: 0..<Float.pi * 2)
        let phi = Float.random(in: 0..<Float.pi)
        let r = Float.random(in: 0.5..<spread)
        
        let x = r * sin(phi) * cos(theta)
        let y = r * sin(phi) * sin(theta)
        let z = r * cos(phi)
        
        positions.append(SIMD3<Float>(x, y, z))
        intensities.append(Float.random(in: 0..<1))
    }
    
    return PointCloud(positions: positions, intensities: intensities)
}

struct PointCloudView_Previews: PreviewProvider {
    static var previews: some View {
        PointCloudPreviewWrapper()
            .frame(width: 800, height: 600)
    }
}

private struct PointCloudPreviewWrapper: View {
    @State private var cloud: PointCloud? = makeTestPointCloud()
    @State private var colorMode: PointCloudColorMode = .height
    
    var body: some View {
        VStack(spacing: 0) {
            PointCloudView(
                pointCloud: $cloud,
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
