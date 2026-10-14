import Foundation
import MetalKit
import simd
import LabCommandCore

/// Color modes for point cloud visualization.
public enum PointCloudColorMode: Int, CaseIterable, Sendable {
    case intensity = 0
    case height = 1
    case sensor = 2
    
    public var label: String {
        switch self {
        case .intensity: return "Intensity"
        case .height: return "Height"
        case .sensor: return "Sensor"
        }
    }
}

/// One point as the shader reads it: 32 bytes (`SIMD3<Float>` strides 16).
struct PointVertex: Sendable {
    var position: SIMD3<Float>
    var intensity: Float
    var sensorID: UInt32
    var _padding: UInt32 = 0

    /// 0 = Mid-360, 1 = JT128, 2 = unknown (neutral color).
    static func sensorIndex(_ sensor: SensorID?) -> UInt32 {
        switch sensor {
        case .mid360: return 0
        case .jt128: return 1
        case nil: return 2
        }
    }
}

/// A cloud already in the GPU's vertex layout, packed off the main thread so a
/// draw only has to copy bytes. Keeps its frame for camera fits and counts.
struct PackedCloud: Sendable {
    let frame: PointCloudFrame
    let vertices: [PointVertex]
    let zMin: Float
    let zMax: Float

    /// Intensity is normalized per frame to [0, 1]; a cloud without a usable
    /// intensity spread reads 1.
    init(_ frame: PointCloudFrame) {
        let count = frame.count
        let sensorIndex = PointVertex.sensorIndex(frame.sensor)
        var zMin = Float.greatestFiniteMagnitude
        var zMax = -Float.greatestFiniteMagnitude

        let vertices = frame.positions.withUnsafeBufferPointer { positions in
            frame.intensities.withUnsafeBufferPointer { intensities in
                var minI = Float.greatestFiniteMagnitude
                var maxI = -Float.greatestFiniteMagnitude
                for value in intensities {
                    minI = min(minI, value)
                    maxI = max(maxI, value)
                }
                let range = maxI - minI
                let scale: Float = range > 0.001 ? 1 / range : 0
                let withIntensity = min(intensities.count, count)

                return [PointVertex](unsafeUninitializedCapacity: count) { out, initialized in
                    for i in 0..<count {
                        let p = positions[i]
                        let intensity: Float = i < withIntensity && scale > 0 ? (intensities[i] - minI) * scale : 1
                        out[i] = PointVertex(position: SIMD3(p.x, p.y, p.z), intensity: intensity, sensorID: sensorIndex)
                        zMin = min(zMin, p.z)
                        zMax = max(zMax, p.z)
                    }
                    initialized = count
                }
            }
        }

        self.frame = frame
        self.vertices = vertices
        self.zMin = count > 0 ? zMin : 0
        self.zMax = count > 0 ? zMax : 0
    }
}

/// Metal renderer for point cloud visualization.
/// Renders millions of points at 60fps using instanced geometry.
/// Z-up coordinate system. Metal depth range [0, 1].
@MainActor
public final class PointCloudRenderer: NSObject, MTKViewDelegate {
    
    // MARK: - Metal State
    
    public let device: MTLDevice?
    private let commandQueue: MTLCommandQueue?
    private var pipelineState: MTLRenderPipelineState?
    private var depthState: MTLDepthStencilState?
    
    /// True when Metal initialized successfully.
    public var isReady: Bool { pipelineState != nil }
    
    // MARK: - Buffers
    
    private var vertexBuffer: MTLBuffer?
    private var uniformBuffers: [MTLBuffer] = []
    private var uniformBufferIndex: Int = 0
    private static let uniformBufferCount = 3
    private let bufferSemaphore = DispatchSemaphore(value: uniformBufferCount)
    private var pointCount: Int = 0
    
    // MARK: - Uniforms
    
    private struct Uniforms {
        var modelViewProjection: simd_float4x4
        var pointSize: Float
        var colorMode: UInt32
        var heightMin: Float
        var heightMax: Float
    }
    
    // MARK: - Configuration
    
    public var colorMode: PointCloudColorMode = .intensity
    public var pointSize: Float = 4.0
    
    /// Field of view in radians (45 degrees).
    public static let fovY: Float = Float.pi / 4
    
    // MARK: - Camera
    
    private var viewMatrix: simd_float4x4
    private var projectionMatrix: simd_float4x4
    private var aspect: Float = 16.0 / 9.0
    private var heightMin: Float = 0
    private var heightMax: Float = 1
    
    /// Current bounding box of loaded points.
    private var boundingBox: (min: SIMD3<Float>, max: SIMD3<Float>)?
    
    /// True once we've received a real nonzero drawable size from MTKView.
    /// Starts false; the default aspect (16:9) is just a placeholder.
    private var hasValidDrawableSize: Bool = false
    
    /// Tracks whether we've done the initial fit with valid drawable size.
    /// Prevents re-fitting on every resize while ensuring first fit uses real aspect.
    private var hasFittedInitialBounds: Bool = false
    
    /// Set when bounds are loaded but drawable size isn't known yet.
    private var needsInitialFit: Bool = false
    
    // MARK: - Live Layers

    /// While set, every draw pulls the newest cloud per sensor from here instead
    /// of drawing what `load` gave it.
    var liveFeed: LiveFrameMailbox? {
        didSet {
            guard liveFeed !== oldValue else { return }
            liveLayers = [:]
        }
    }

    /// One sensor's live cloud on the GPU. A ring of buffers so an upload never
    /// writes one that a frame still in flight reads: at most two other frames are
    /// in flight once a draw holds the semaphore, and they use the current and
    /// previous slots, so the next slot is free.
    private struct LiveLayer {
        var ring: [MTLBuffer] = []
        var capacity = 0
        var slot = 0
        var count = 0
        var sequence = -1
        var zMin: Float = 0
        var zMax: Float = 0
    }
    private var liveLayers: [SensorID: LiveLayer] = [:]

    // MARK: - Init
    
    public override init() {
        self.device = MTLCreateSystemDefaultDevice()
        self.commandQueue = device?.makeCommandQueue()
        
        // Default camera: Z-up, look at origin from Y-negative
        let eye = SIMD3<Float>(0, -5, 3)
        let center = SIMD3<Float>(0, 0, 0)
        let up = SIMD3<Float>(0, 0, 1)
        self.viewMatrix = Self.lookAt(eye: eye, center: center, up: up)
        
        // Initialize projection with default 16:9 aspect
        self.projectionMatrix = Self.perspective(fovY: Self.fovY, aspect: 16.0 / 9.0, near: 0.01, far: 1000)
        
        super.init()
        
        guard device != nil else { return }
        
        buildPipeline()
        buildDepthState()
        buildUniformBuffers()
    }
    
    // MARK: - Pipeline Setup
    
    private func buildPipeline() {
        guard let device = device else { return }
        
        let library: MTLLibrary?
        do {
            library = try device.makeLibrary(source: Self.shaderSource, options: nil)
        } catch {
            print("PointCloudRenderer: Shader compile error: \(error)")
            return
        }
        
        guard let lib = library,
              let vertexFunction = lib.makeFunction(name: "pointVertex"),
              let fragmentFunction = lib.makeFunction(name: "pointFragment") else {
            print("PointCloudRenderer: Failed to find shader functions")
            return
        }
        
        let vertexDescriptor = MTLVertexDescriptor()
        
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        
        vertexDescriptor.attributes[1].format = .float
        vertexDescriptor.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        vertexDescriptor.attributes[1].bufferIndex = 0
        
        vertexDescriptor.attributes[2].format = .uint
        vertexDescriptor.attributes[2].offset = MemoryLayout<SIMD3<Float>>.stride + MemoryLayout<Float>.stride
        vertexDescriptor.attributes[2].bufferIndex = 0
        
        vertexDescriptor.layouts[0].stride = MemoryLayout<PointVertex>.stride
        vertexDescriptor.layouts[0].stepFunction = .perVertex
        
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.label = "PointCloudPipeline"
        pipelineDescriptor.vertexFunction = vertexFunction
        pipelineDescriptor.fragmentFunction = fragmentFunction
        pipelineDescriptor.vertexDescriptor = vertexDescriptor
        pipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        pipelineDescriptor.colorAttachments[0].isBlendingEnabled = false
        pipelineDescriptor.depthAttachmentPixelFormat = .depth32Float
        
        do {
            pipelineState = try device.makeRenderPipelineState(descriptor: pipelineDescriptor)
        } catch {
            print("PointCloudRenderer: Failed to create pipeline state: \(error)")
        }
    }
    
    private func buildDepthState() {
        guard let device = device else { return }
        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .less
        depthDescriptor.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: depthDescriptor)
    }
    
    private func buildUniformBuffers() {
        guard let device = device else { return }
        uniformBuffers = (0..<Self.uniformBufferCount).compactMap { _ in
            device.makeBuffer(length: MemoryLayout<Uniforms>.stride, options: .storageModeShared)
        }
    }
    
    // MARK: - Data Loading
    
    /// Load a PointCloudFrame from LabCommandCore.
    public func load(_ frame: PointCloudFrame) {
        load(frames: [frame])
    }
    
    /// Load multiple frames with sensor coloring.
    ///
    /// - Parameter fitCamera: Refit the camera to the new bounds. Pass false for
    ///   updates that should change the points under a camera that stays put; a fit
    ///   still waiting on the first drawable size is left pending either way.
    public func load(frames: [PointCloudFrame], fitCamera: Bool = true) {
        let packed = frames.filter { !$0.isEmpty }.map(PackedCloud.init)
        let total = packed.reduce(0) { $0 + $1.vertices.count }
        guard let device = device, total > 0 else {
            vertexBuffer = nil
            pointCount = 0
            boundingBox = nil
            return
        }
        
        guard let buffer = device.makeBuffer(length: total * MemoryLayout<PointVertex>.stride, options: .storageModeShared) else {
            return
        }
        var offset = 0
        for cloud in packed {
            cloud.vertices.withUnsafeBytes { bytes in
                (buffer.contents() + offset).copyMemory(from: bytes.baseAddress!, byteCount: bytes.count)
                offset += bytes.count
            }
        }
        vertexBuffer = buffer
        pointCount = total
        heightMin = packed.map(\.zMin).min() ?? 0
        heightMax = max(packed.map(\.zMax).max() ?? 1, heightMin + 0.001)
        boundingBox = Self.fitBounds(packed.map(\.frame.positions))
        
        // Schedule fit-to-bounds for when drawable size is known
        if fitCamera {
            needsInitialFit = true
            hasFittedInitialBounds = false
        }
        fitToBoundsIfReady()
    }
    
    /// Fit the camera to the live clouds now in `liveFeed`. False when there are none.
    @discardableResult
    public func fitToLive() -> Bool {
        guard let feed = liveFeed else { return false }
        let clouds = SensorID.allCases.compactMap { feed.latest($0)?.cloud.frame.positions }
        guard let bounds = Self.fitBounds(clouds) else { return false }
        // Extremes of the same bounded sample, for the log: cheap on the main thread.
        if let raw = RobustBounds.percentile(clouds, lower: 0, upper: 1) {
            func box(_ lo: Point3D, _ hi: Point3D) -> String {
                String(format: "x %.1f…%.1f y %.1f…%.1f z %.1f…%.1f m", lo.x, hi.x, lo.y, hi.y, lo.z, hi.z)
            }
            let fit = box(Point3D(x: bounds.min.x, y: bounds.min.y, z: bounds.min.z), Point3D(x: bounds.max.x, y: bounds.max.y, z: bounds.max.z))
            FileHandle.standardError.write(Data("[PointCloudView] live fit box \(fit) (raw min/max \(box(raw.min, raw.max)))\n".utf8))
        }
        boundingBox = bounds
        needsInitialFit = true
        hasFittedInitialBounds = false
        fitToBoundsIfReady()
        return true
    }
    
    /// The box the camera frames: 2nd–98th percentile per axis of the points within
    /// `RobustBounds.maxFitRange` of the rig, so the far field and a few stray
    /// returns don't shrink the room to a corner of the view.
    private static func fitBounds(_ clouds: [[Point3D]]) -> (min: SIMD3<Float>, max: SIMD3<Float>)? {
        guard let bounds = RobustBounds.percentile(clouds, maxHorizontalRange: RobustBounds.maxFitRange) else { return nil }
        return (
            SIMD3(bounds.min.x, bounds.min.y, bounds.min.z),
            SIMD3(bounds.max.x, bounds.max.y, bounds.max.z)
        )
    }
    
    /// Copy any newer live cloud into its layer's next ring slot. Runs inside
    /// `draw`, after the semaphore, so it never writes a buffer the GPU reads.
    private func refreshLiveLayers(from feed: LiveFrameMailbox) {
        guard let device = device else { return }
        let now = ProcessInfo.processInfo.systemUptime
        var uploaded = false
        for sensor in SensorID.allCases {
            var layer = liveLayers[sensor] ?? LiveLayer()
            guard let entry = feed.latest(sensor) else {
                layer.count = 0
                liveLayers[sensor] = layer
                continue
            }
            guard entry.sequence != layer.sequence else { continue }
            
            let vertices = entry.cloud.vertices
            if vertices.count > layer.capacity {
                // Headroom so frame-to-frame jitter in point count doesn't reallocate.
                let capacity = vertices.count + vertices.count / 4
                let ring = (0..<Self.uniformBufferCount).compactMap { _ in
                    device.makeBuffer(length: capacity * MemoryLayout<PointVertex>.stride, options: .storageModeShared)
                }
                guard ring.count == Self.uniformBufferCount else { continue }
                layer.ring = ring
                layer.capacity = capacity
            }
            let slot = (layer.slot + 1) % layer.ring.count
            vertices.withUnsafeBytes { bytes in
                layer.ring[slot].contents().copyMemory(from: bytes.baseAddress!, byteCount: bytes.count)
            }
            layer.slot = slot
            layer.count = vertices.count
            layer.sequence = entry.sequence
            layer.zMin = entry.cloud.zMin
            layer.zMax = entry.cloud.zMax
            liveLayers[sensor] = layer
            feed.noteRendered(sensor, at: now)
            uploaded = true
        }
        if uploaded {
            feed.noteUpload(ms: (ProcessInfo.processInfo.systemUptime - now) * 1000)
        }
    }
    
    // MARK: - Camera Control
    
    public func setViewMatrix(_ matrix: simd_float4x4) {
        viewMatrix = matrix
    }
    
    public func resetCamera() {
        if fitToLive() { return }
        hasFittedInitialBounds = false
        needsInitialFit = true
        fitToBoundsIfReady()
    }
    
    /// Fit camera if we have both bounds and a valid drawable size.
    /// Called from loadFrame, drawable size change, and resetCamera.
    private func fitToBoundsIfReady() {
        guard needsInitialFit,
              hasValidDrawableSize,
              boundingBox != nil else {
            return
        }
        fitToBounds(logOnce: !hasFittedInitialBounds)
        hasFittedInitialBounds = true
        needsInitialFit = false
    }
    
    /// Fit camera to show all loaded points using exact fit algorithm.
    /// Uses the real elevated camera orientation to compute distance.
    public func fitToBounds(logOnce: Bool = false) {
        guard let bb = boundingBox else {
            // No bounds, use default view
            let eye = SIMD3<Float>(0, -5, 3)
            let center = SIMD3<Float>(0, 0, 0)
            let up = SIMD3<Float>(0, 0, 1)
            viewMatrix = Self.lookAt(eye: eye, center: center, up: up)
            return
        }
        
        // Convert SIMD bounds to Point3D for CameraMath
        let bbMin = Point3D(x: bb.min.x, y: bb.min.y, z: bb.min.z)
        let bbMax = Point3D(x: bb.max.x, y: bb.max.y, z: bb.max.z)
        
        // Use CameraMath's exact fit algorithm
        let (eyeP, centerP, distance) = CameraMath.fitToBounds(
            min: bbMin,
            max: bbMax,
            fovY: Self.fovY,
            aspect: aspect,
            margin: 0.9,
            elevation: CameraMath.defaultElevation
        )
        
        // Convert back to SIMD
        let eye = SIMD3<Float>(eyeP.x, eyeP.y, eyeP.z)
        let target = SIMD3<Float>(centerP.x, centerP.y, centerP.z)
        let up = SIMD3<Float>(0, 0, 1)
        viewMatrix = Self.lookAt(eye: eye, center: target, up: up)
        
        if logOnce {
            print("[fitToBounds] target=\(target), distance=\(distance), aspect=\(aspect)")
        }
    }
    
    // MARK: - MTKViewDelegate
    
    public func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        guard size.width > 0, size.height > 0 else { return }
        hasValidDrawableSize = true
        aspect = Float(size.width / size.height)
        projectionMatrix = Self.perspective(fovY: Self.fovY, aspect: aspect, near: 0.01, far: 1000)
        
        // On first valid drawable size with loaded bounds, fit camera
        fitToBoundsIfReady()
    }
    
    public func draw(in view: MTKView) {
        guard let pipelineState = pipelineState,
              let commandQueue = commandQueue,
              !uniformBuffers.isEmpty,
              liveFeed != nil || (vertexBuffer != nil && pointCount > 0),
              let drawable = view.currentDrawable,
              let renderPassDescriptor = view.currentRenderPassDescriptor else {
            return
        }

        // Wait for an available buffer slot
        _ = bufferSemaphore.wait(timeout: .distantFuture)

        var draws: [(buffer: MTLBuffer, count: Int)] = []
        if let feed = liveFeed {
            refreshLiveLayers(from: feed)
            let layers = SensorID.allCases.compactMap { liveLayers[$0] }.filter { $0.count > 0 }
            draws = layers.map { ($0.ring[$0.slot], $0.count) }
            if let low = layers.map(\.zMin).min(), let high = layers.map(\.zMax).max() {
                heightMin = low
                heightMax = max(high, low + 0.001)
            }
        } else if let vertexBuffer {
            draws = [(vertexBuffer, pointCount)]
        }

        uniformBufferIndex = (uniformBufferIndex + 1) % Self.uniformBufferCount
        let uniformBuffer = uniformBuffers[uniformBufferIndex]
        
        // Write uniforms every frame to the current buffer
        updateUniforms(to: uniformBuffer)
        
        guard let commandBuffer = commandQueue.makeCommandBuffer() else {
            bufferSemaphore.signal()
            return
        }
        
        // Signal semaphore when GPU is done with this buffer
        commandBuffer.addCompletedHandler { [weak self] _ in
            self?.bufferSemaphore.signal()
        }
        
        renderPassDescriptor.colorAttachments[0].clearColor = MTLClearColor(
            red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0
        )
        renderPassDescriptor.colorAttachments[0].loadAction = .clear
        renderPassDescriptor.colorAttachments[0].storeAction = .store
        
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else {
            bufferSemaphore.signal()
            return
        }
        
        encoder.setRenderPipelineState(pipelineState)
        encoder.setDepthStencilState(depthState)
        encoder.setVertexBuffer(uniformBuffer, offset: 0, index: 1)
        for draw in draws {
            encoder.setVertexBuffer(draw.buffer, offset: 0, index: 0)
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: draw.count)
        }
        encoder.endEncoding()
        
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }
    
    private func updateUniforms(to buffer: MTLBuffer) {
        let mvp = projectionMatrix * viewMatrix
        var uniforms = Uniforms(
            modelViewProjection: mvp,
            pointSize: pointSize,
            colorMode: UInt32(colorMode.rawValue),
            heightMin: heightMin,
            heightMax: heightMax
        )
        memcpy(buffer.contents(), &uniforms, MemoryLayout<Uniforms>.stride)
    }
    
    // MARK: - Matrix Helpers (Z-up, Metal depth [0,1])
    
    private static func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
        let z = normalize(eye - center)
        let x = normalize(cross(up, z))
        let y = cross(z, x)
        
        let t = SIMD3<Float>(-dot(x, eye), -dot(y, eye), -dot(z, eye))
        
        return simd_float4x4(
            SIMD4<Float>(x.x, y.x, z.x, 0),
            SIMD4<Float>(x.y, y.y, z.y, 0),
            SIMD4<Float>(x.z, y.z, z.z, 0),
            SIMD4<Float>(t.x, t.y, t.z, 1)
        )
    }
    
    /// Perspective projection with Metal depth range [0, 1].
    private static func perspective(fovY: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let yScale = 1 / tan(fovY * 0.5)
        let xScale = yScale / aspect
        let zRange = far - near
        
        // Metal NDC: depth in [0, 1]
        let zScale = -far / zRange
        let wzScale = -far * near / zRange
        
        return simd_float4x4(
            SIMD4<Float>(xScale, 0, 0, 0),
            SIMD4<Float>(0, yScale, 0, 0),
            SIMD4<Float>(0, 0, zScale, -1),
            SIMD4<Float>(0, 0, wzScale, 0)
        )
    }
    
    // MARK: - Inline Shader Source
    
    private static let shaderSource = """
    #include <metal_stdlib>
    using namespace metal;
    
    struct PointVertex {
        float3 position [[attribute(0)]];
        float intensity [[attribute(1)]];
        uint sensorID [[attribute(2)]];
    };
    
    struct VertexOut {
        float4 position [[position]];
        float pointSize [[point_size]];
        float4 color;
    };
    
    struct Uniforms {
        float4x4 modelViewProjection;
        float pointSize;
        uint colorMode;
        float heightMin;
        float heightMax;
    };
    
    float3 rainbowGradient(float t) {
        t = saturate(t);
        float3 c;
        if (t < 0.25) {
            c = mix(float3(0.0, 0.0, 1.0), float3(0.0, 1.0, 1.0), t * 4.0);
        } else if (t < 0.5) {
            c = mix(float3(0.0, 1.0, 1.0), float3(0.0, 1.0, 0.0), (t - 0.25) * 4.0);
        } else if (t < 0.75) {
            c = mix(float3(0.0, 1.0, 0.0), float3(1.0, 1.0, 0.0), (t - 0.5) * 4.0);
        } else {
            c = mix(float3(1.0, 1.0, 0.0), float3(1.0, 0.0, 0.0), (t - 0.75) * 4.0);
        }
        return c;
    }
    
    vertex VertexOut pointVertex(
        PointVertex in [[stage_in]],
        constant Uniforms& uniforms [[buffer(1)]]
    ) {
        VertexOut out;
        out.position = uniforms.modelViewProjection * float4(in.position, 1.0);
        out.pointSize = uniforms.pointSize;
        
        float3 color;
        switch (uniforms.colorMode) {
            case 0: {
                float i = saturate(in.intensity);
                color = float3(i, i, i);
                break;
            }
            case 1: {
                float heightRange = uniforms.heightMax - uniforms.heightMin;
                float normalizedHeight = (heightRange > 0.001)
                    ? (in.position.z - uniforms.heightMin) / heightRange
                    : 0.5;
                color = rainbowGradient(normalizedHeight);
                break;
            }
            case 2: {
                if (in.sensorID == 0) {
                    color = float3(0.0, 0.85, 0.95);  // Mid-360: cyan
                } else if (in.sensorID == 1) {
                    color = float3(0.9, 0.25, 0.8);   // JT128: magenta
                } else {
                    color = float3(0.7, 0.7, 0.7);   // Unknown: gray
                }
                break;
            }
            default:
                color = float3(1.0, 1.0, 1.0);
        }
        
        out.color = float4(color, 1.0);
        return out;
    }
    
    fragment float4 pointFragment(
        VertexOut in [[stage_in]],
        float2 pointCoord [[point_coord]]
    ) {
        float2 centered = pointCoord - 0.5;
        float dist = length(centered);
        
        if (dist > 0.5) {
            discard_fragment();
        }
        
        return in.color;
    }
    """
}
