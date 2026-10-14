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
    
    // MARK: - Vertex Layout
    
    private struct PointVertex {
        var position: SIMD3<Float>
        var intensity: Float
        var sensorID: UInt32
        var _padding: UInt32 = 0
    }
    
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
        guard let device = device, !frame.isEmpty else {
            vertexBuffer = nil
            pointCount = 0
            boundingBox = nil
            return
        }
        
        // Sensor index: 0=Mid-360, 1=JT128, 2=unknown/nil (neutral color)
        let sensorIndex: UInt32 = {
            switch frame.sensor {
            case .mid360: return 0
            case .jt128: return 1
            case nil: return 2
            }
        }()
        
        // Find intensity range for per-frame normalization
        var minI: Float = .greatestFiniteMagnitude
        var maxI: Float = -.greatestFiniteMagnitude
        for i in frame.intensities {
            minI = min(minI, i)
            maxI = max(maxI, i)
        }
        let intensityRange = maxI - minI
        
        var bbMin = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
        var bbMax = SIMD3<Float>(repeating: -.greatestFiniteMagnitude)
        
        var vertices: [PointVertex] = []
        vertices.reserveCapacity(frame.count)
        
        for i in 0..<frame.count {
            let p = frame.positions[i]
            var intensity: Float = 1.0
            if i < frame.intensities.count {
                let raw = frame.intensities[i]
                intensity = intensityRange > 0.001 ? (raw - minI) / intensityRange : 1.0
            }
            
            let pos = SIMD3<Float>(p.x, p.y, p.z)
            vertices.append(PointVertex(
                position: pos,
                intensity: intensity,
                sensorID: sensorIndex
            ))
            
            bbMin = min(bbMin, pos)
            bbMax = max(bbMax, pos)
        }
        
        boundingBox = (bbMin, bbMax)
        heightMin = bbMin.z
        heightMax = max(bbMax.z, bbMin.z + 0.001)
        
        vertexBuffer = device.makeBuffer(
            bytes: vertices,
            length: vertices.count * MemoryLayout<PointVertex>.stride,
            options: .storageModeShared
        )
        pointCount = frame.count
        
        // Schedule fit-to-bounds for when drawable size is known
        needsInitialFit = true
        hasFittedInitialBounds = false
        fitToBoundsIfReady()
    }
    
    /// Load multiple frames with sensor coloring.
    public func load(frames: [PointCloudFrame]) {
        guard let device = device else {
            vertexBuffer = nil
            pointCount = 0
            boundingBox = nil
            return
        }
        
        var allVertices: [PointVertex] = []
        var bbMin = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
        var bbMax = SIMD3<Float>(repeating: -.greatestFiniteMagnitude)
        
        for frame in frames {
            let sensorIndex: UInt32 = {
                switch frame.sensor {
                case .mid360: return 0
                case .jt128: return 1
                case nil: return 2
                }
            }()
            
            // Find intensity range for per-frame normalization
            var minI: Float = .greatestFiniteMagnitude
            var maxI: Float = -.greatestFiniteMagnitude
            for i in frame.intensities {
                minI = min(minI, i)
                maxI = max(maxI, i)
            }
            let intensityRange = maxI - minI
            
            for i in 0..<frame.count {
                let p = frame.positions[i]
                var intensity: Float = 1.0
                if i < frame.intensities.count {
                    let raw = frame.intensities[i]
                    intensity = intensityRange > 0.001 ? (raw - minI) / intensityRange : 1.0
                }
                
                let pos = SIMD3<Float>(p.x, p.y, p.z)
                allVertices.append(PointVertex(
                    position: pos,
                    intensity: intensity,
                    sensorID: sensorIndex
                ))
                
                bbMin = min(bbMin, pos)
                bbMax = max(bbMax, pos)
            }
        }
        
        guard !allVertices.isEmpty else {
            vertexBuffer = nil
            pointCount = 0
            boundingBox = nil
            return
        }
        
        boundingBox = (bbMin, bbMax)
        heightMin = bbMin.z
        heightMax = max(bbMax.z, bbMin.z + 0.001)
        
        vertexBuffer = device.makeBuffer(
            bytes: allVertices,
            length: allVertices.count * MemoryLayout<PointVertex>.stride,
            options: .storageModeShared
        )
        pointCount = allVertices.count
        
        // Schedule fit-to-bounds for when drawable size is known
        needsInitialFit = true
        hasFittedInitialBounds = false
        fitToBoundsIfReady()
    }
    
    // MARK: - Camera Control
    
    public func setViewMatrix(_ matrix: simd_float4x4) {
        viewMatrix = matrix
    }
    
    public func resetCamera() {
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
              let drawable = view.currentDrawable,
              let renderPassDescriptor = view.currentRenderPassDescriptor,
              let vertexBuffer = vertexBuffer,
              pointCount > 0,
              !uniformBuffers.isEmpty else {
            return
        }
        
        // Wait for an available buffer slot
        _ = bufferSemaphore.wait(timeout: .distantFuture)
        
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
        encoder.setVertexBuffer(vertexBuffer, offset: 0, index: 0)
        encoder.setVertexBuffer(uniformBuffer, offset: 0, index: 1)
        encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: pointCount)
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
