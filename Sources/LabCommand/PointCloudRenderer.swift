import Foundation
import MetalKit
import simd

/// A point cloud loaded from PCD or PointCloud2 data.
/// Expected to be provided by PointCloudDecoder from LabCommandCore.
public struct PointCloud: Equatable, Identifiable {
    public let id: UUID
    public var positions: [SIMD3<Float>]
    public var intensities: [Float]
    public var sensorID: UInt32?
    
    public init(positions: [SIMD3<Float>], intensities: [Float], sensorID: UInt32? = nil) {
        self.id = UUID()
        self.positions = positions
        self.intensities = intensities
        self.sensorID = sensorID
    }
    
    public var pointCount: Int { positions.count }
    public var isEmpty: Bool { positions.isEmpty }
    
    public static func == (lhs: PointCloud, rhs: PointCloud) -> Bool {
        lhs.id == rhs.id
    }
}

/// Color modes for point cloud visualization.
public enum PointCloudColorMode: Int, CaseIterable, Sendable {
    case intensity = 0  // Grayscale based on intensity
    case height = 1     // Rainbow gradient based on Z
    case sensor = 2     // Mid-360 = cyan, JT128 = magenta
    
    public var label: String {
        switch self {
        case .intensity: return "Intensity"
        case .height: return "Height"
        case .sensor: return "Sensor"
        }
    }
}

/// Metal renderer for point cloud visualization.
/// Renders up to millions of points at 60fps using instanced geometry.
@MainActor
public final class PointCloudRenderer: NSObject, MTKViewDelegate {
    
    // MARK: - Metal State
    
    private let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private var pipelineState: MTLRenderPipelineState?
    private var depthState: MTLDepthStencilState?
    
    // MARK: - Buffers
    
    private var vertexBuffer: MTLBuffer?
    private var uniformBuffer: MTLBuffer?
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
    
    public var colorMode: PointCloudColorMode = .intensity {
        didSet { needsUniformUpdate = true }
    }
    
    public var pointSize: Float = 4.0 {
        didSet { needsUniformUpdate = true }
    }
    
    // MARK: - Camera (static for this PR)
    
    private var viewMatrix: simd_float4x4
    private var projectionMatrix: simd_float4x4 = matrix_identity_float4x4
    private var heightMin: Float = 0
    private var heightMax: Float = 1
    private var needsUniformUpdate = true
    
    // MARK: - Vertex Layout
    
    private struct PointVertex {
        var position: SIMD3<Float>
        var intensity: Float
        var sensorID: UInt32
        var _padding: UInt32 = 0  // Align to 32 bytes
    }
    
    // MARK: - Init
    
    public override init() {
        guard let device = MTLCreateSystemDefaultDevice() else {
            fatalError("Metal is not supported on this device")
        }
        self.device = device
        
        guard let queue = device.makeCommandQueue() else {
            fatalError("Failed to create Metal command queue")
        }
        self.commandQueue = queue
        
        // Default camera: look at origin from a reasonable distance
        let eye = SIMD3<Float>(0, -5, 3)
        let center = SIMD3<Float>(0, 0, 0)
        let up = SIMD3<Float>(0, 0, 1)
        self.viewMatrix = Self.lookAt(eye: eye, center: center, up: up)
        
        super.init()
        
        buildPipeline()
        buildDepthState()
        buildUniformBuffer()
    }
    
    // MARK: - Pipeline Setup
    
    private func buildPipeline() {
        guard let library = device.makeDefaultLibrary() else {
            print("PointCloudRenderer: Failed to create default Metal library")
            return
        }
        
        guard let vertexFunction = library.makeFunction(name: "pointVertex"),
              let fragmentFunction = library.makeFunction(name: "pointFragment") else {
            print("PointCloudRenderer: Failed to find shader functions")
            return
        }
        
        let vertexDescriptor = MTLVertexDescriptor()
        
        // Position (float3)
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        
        // Intensity (float)
        vertexDescriptor.attributes[1].format = .float
        vertexDescriptor.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        vertexDescriptor.attributes[1].bufferIndex = 0
        
        // SensorID (uint)
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
        pipelineDescriptor.colorAttachments[0].isBlendingEnabled = true
        pipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        pipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        pipelineDescriptor.colorAttachments[0].sourceAlphaBlendFactor = .one
        pipelineDescriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        pipelineDescriptor.depthAttachmentPixelFormat = .depth32Float
        
        do {
            pipelineState = try device.makeRenderPipelineState(descriptor: pipelineDescriptor)
        } catch {
            print("PointCloudRenderer: Failed to create pipeline state: \(error)")
        }
    }
    
    private func buildDepthState() {
        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .less
        depthDescriptor.isDepthWriteEnabled = true
        depthState = device.makeDepthStencilState(descriptor: depthDescriptor)
    }
    
    private func buildUniformBuffer() {
        uniformBuffer = device.makeBuffer(
            length: MemoryLayout<Uniforms>.stride,
            options: .storageModeShared
        )
    }
    
    // MARK: - Data Loading
    
    /// Load point cloud data into GPU buffers.
    public func load(points: [SIMD3<Float>], intensities: [Float], sensorID: UInt32?) {
        guard !points.isEmpty else {
            vertexBuffer = nil
            pointCount = 0
            return
        }
        
        let sid = sensorID ?? 0
        
        // Calculate height bounds for rainbow coloring
        var minZ: Float = .greatestFiniteMagnitude
        var maxZ: Float = -.greatestFiniteMagnitude
        for p in points {
            minZ = min(minZ, p.z)
            maxZ = max(maxZ, p.z)
        }
        heightMin = minZ
        heightMax = maxZ
        
        // Build vertex data
        var vertices: [PointVertex] = []
        vertices.reserveCapacity(points.count)
        
        for i in 0..<points.count {
            let intensity = i < intensities.count ? intensities[i] : 0.5
            vertices.append(PointVertex(
                position: points[i],
                intensity: intensity,
                sensorID: sid
            ))
        }
        
        vertexBuffer = device.makeBuffer(
            bytes: vertices,
            length: vertices.count * MemoryLayout<PointVertex>.stride,
            options: .storageModeShared
        )
        pointCount = points.count
        needsUniformUpdate = true
    }
    
    /// Load a PointCloud struct directly.
    public func load(_ cloud: PointCloud) {
        load(points: cloud.positions, intensities: cloud.intensities, sensorID: cloud.sensorID)
    }
    
    /// Load multiple point clouds with different sensor IDs.
    public func load(clouds: [(cloud: PointCloud, sensorID: UInt32)]) {
        var allVertices: [PointVertex] = []
        var minZ: Float = .greatestFiniteMagnitude
        var maxZ: Float = -.greatestFiniteMagnitude
        
        for (cloud, sid) in clouds {
            for i in 0..<cloud.positions.count {
                let p = cloud.positions[i]
                let intensity = i < cloud.intensities.count ? cloud.intensities[i] : 0.5
                allVertices.append(PointVertex(
                    position: p,
                    intensity: intensity,
                    sensorID: sid
                ))
                minZ = min(minZ, p.z)
                maxZ = max(maxZ, p.z)
            }
        }
        
        guard !allVertices.isEmpty else {
            vertexBuffer = nil
            pointCount = 0
            return
        }
        
        heightMin = minZ
        heightMax = maxZ
        
        vertexBuffer = device.makeBuffer(
            bytes: allVertices,
            length: allVertices.count * MemoryLayout<PointVertex>.stride,
            options: .storageModeShared
        )
        pointCount = allVertices.count
        needsUniformUpdate = true
    }
    
    // MARK: - Camera Control
    
    /// Set the view matrix for camera positioning.
    public func setViewMatrix(_ matrix: simd_float4x4) {
        viewMatrix = matrix
        needsUniformUpdate = true
    }
    
    /// Reset camera to default position.
    public func resetCamera() {
        let eye = SIMD3<Float>(0, -5, 3)
        let center = SIMD3<Float>(0, 0, 0)
        let up = SIMD3<Float>(0, 0, 1)
        viewMatrix = Self.lookAt(eye: eye, center: center, up: up)
        needsUniformUpdate = true
    }
    
    // MARK: - MTKViewDelegate
    
    public func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        let aspect = Float(size.width / size.height)
        projectionMatrix = Self.perspective(fovY: Float.pi / 4, aspect: aspect, near: 0.01, far: 1000)
        needsUniformUpdate = true
    }
    
    public func draw(in view: MTKView) {
        guard let pipelineState = pipelineState,
              let drawable = view.currentDrawable,
              let renderPassDescriptor = view.currentRenderPassDescriptor,
              let commandBuffer = commandQueue.makeCommandBuffer(),
              let vertexBuffer = vertexBuffer,
              pointCount > 0 else {
            return
        }
        
        // Update uniforms if needed
        if needsUniformUpdate {
            updateUniforms()
            needsUniformUpdate = false
        }
        
        renderPassDescriptor.colorAttachments[0].clearColor = MTLClearColor(
            red: 0.043, green: 0.051, blue: 0.063, alpha: 1.0
        )
        renderPassDescriptor.colorAttachments[0].loadAction = .clear
        renderPassDescriptor.colorAttachments[0].storeAction = .store
        
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else {
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
    
    private func updateUniforms() {
        guard let uniformBuffer = uniformBuffer else { return }
        
        let mvp = projectionMatrix * viewMatrix
        var uniforms = Uniforms(
            modelViewProjection: mvp,
            pointSize: pointSize,
            colorMode: UInt32(colorMode.rawValue),
            heightMin: heightMin,
            heightMax: heightMax
        )
        
        memcpy(uniformBuffer.contents(), &uniforms, MemoryLayout<Uniforms>.stride)
    }
    
    // MARK: - Matrix Helpers
    
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
    
    private static func perspective(fovY: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let yScale = 1 / tan(fovY * 0.5)
        let xScale = yScale / aspect
        let zRange = far - near
        let zScale = -(far + near) / zRange
        let wzScale = -2 * far * near / zRange
        
        return simd_float4x4(
            SIMD4<Float>(xScale, 0, 0, 0),
            SIMD4<Float>(0, yScale, 0, 0),
            SIMD4<Float>(0, 0, zScale, -1),
            SIMD4<Float>(0, 0, wzScale, 0)
        )
    }
}
