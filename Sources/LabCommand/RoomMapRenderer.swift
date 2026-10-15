import Foundation
import MetalKit
import simd
import LabCommandCore

/// Metal voxels for the Room page. Z-up, the same fit as the lidar view, and no shared buffers.
@MainActor
final class RoomMapRenderer: NSObject, MTKViewDelegate {
    let device: MTLDevice?
    private let commandQueue: MTLCommandQueue?
    private var pipeline: MTLRenderPipelineState?
    private var depthWrite: MTLDepthStencilState?
    private var depthRead: MTLDepthStencilState?
    private var depthAlways: MTLDepthStencilState?
    private var uniformBuffers: [MTLBuffer] = []
    private var uniformIndex = 0
    private let frameSlots = 3
    private let slotGate: DispatchSemaphore

    private var pointBuffer: MTLBuffer?
    private var pointCount = 0
    private var frameBuffer: MTLBuffer?
    private var frameCount = 0
    private var fillBuffer: MTLBuffer?
    private var fillCount = 0
    private var edgeBuffer: MTLBuffer?
    private var edgeCount = 0
    private var topFillBuffer: MTLBuffer?
    private var topFillCount = 0
    private var topEdgeBuffer: MTLBuffer?
    private var topEdgeCount = 0
    private var markBuffer: MTLBuffer?
    private var markCount = 0

    private var viewMatrix = matrix_identity_float4x4
    private var projection = matrix_identity_float4x4
    private var aspect: Float = 16.0 / 9.0
    private var hasSize = false
    private var eye = SIMD3<Float>(0, -5, 2)
    private var target = SIMD3<Float>(0, 0, 0)
    private var userMoved = false
    private var fittedKey = ""
    private var content: [SIMD3<Float>] = []
    private var uploaded = false
    private var uploadedRevision = 0
    private var uploadedVoxel: Float?
    private var uploadedMarkers: [LiveRoomMarker] = []
    private var uploadedOpacity: Float = -1

    private struct Uniforms {
        var modelViewProjection: simd_float4x4
    }

    /// Tight enough for the shader and aligned so float4 starts at 16.
    private struct Vertex {
        var x: Float
        var y: Float
        var z: Float
        var pad: Float
        var r: Float
        var g: Float
        var b: Float
        var a: Float
        var size: Float
        var pad2: Float
    }

    override init() {
        let device = MTLCreateSystemDefaultDevice()
        self.device = device
        self.commandQueue = device?.makeCommandQueue()
        self.slotGate = DispatchSemaphore(value: 3)
        super.init()
        guard let device else { return }
        buildPipeline(device)
        uniformBuffers = (0..<frameSlots).compactMap { _ in
            device.makeBuffer(length: MemoryLayout<Uniforms>.stride, options: .storageModeShared)
        }
        let eye = SIMD3<Float>(0, -5, 2)
        viewMatrix = Self.lookAt(eye: eye, center: .zero, up: SIMD3(0, 0, 1))
        projection = Self.perspective(fovY: CameraMath.defaultFovY, aspect: aspect, near: 0.1, far: 1000)
    }

    func update(mirror: RoomMemoryMirror, markers: [LiveRoomMarker], opacity: Float, drawMap: Bool) {
        guard drawMap else {
            clearGeometry()
            uploaded = true
            uploadedRevision = mirror.mapRevision
            uploadedVoxel = mirror.voxelSize
            uploadedMarkers = []
            uploadedOpacity = opacity
            content = []
            return
        }
        let revisionChanged = !uploaded || mirror.mapRevision != uploadedRevision || mirror.voxelSize != uploadedVoxel
        let markersChanged = markers != uploadedMarkers
        let opacityChanged = opacity != uploadedOpacity
        guard revisionChanged || markersChanged || opacityChanged else { return }
        let map = LiveRoomVoxels.extract(mirror)
        if revisionChanged || opacityChanged {
            uploadMap(map, opacity: opacity)
        }
        if markersChanged || opacityChanged || !uploaded {
            uploadMarkers(markers)
        }
        uploaded = true
        uploadedRevision = mirror.mapRevision
        uploadedVoxel = mirror.voxelSize
        uploadedMarkers = markers
        uploadedOpacity = opacity
        content = LiveRoomVoxels.fitPoints(map: map, markers: markers)
        noteContent()
    }

    func orbit(dx: Float, dy: Float) {
        userMoved = true
        let offset = eye - target
        var radius = simd_length(offset)
        if radius < 0.05 { radius = 0.05 }
        var yaw = atan2(offset.x, -offset.y)
        var pitch = asin(max(-1, min(1, offset.z / radius)))
        yaw -= dx * 0.005
        pitch = min(1.2, max(-0.15, pitch + dy * 0.005))
        let cp = cos(pitch)
        let sp = sin(pitch)
        eye = target + radius * SIMD3(sin(yaw) * cp, -cos(yaw) * cp, sp)
        viewMatrix = Self.lookAt(eye: eye, center: target, up: SIMD3(0, 0, 1))
    }

    func dolly(_ deltaY: Float) {
        userMoved = true
        let offset = eye - target
        let length = simd_length(offset)
        let radius = max(Float(0.2), length * exp(-deltaY * 0.01))
        let dir = length > 0.001 ? offset / length : SIMD3<Float>(0, -1, 0.4)
        eye = target + dir * radius
        viewMatrix = Self.lookAt(eye: eye, center: target, up: SIMD3(0, 0, 1))
    }

    /// One fit, then the camera stays until the next double-click.
    func refit() {
        userMoved = true
        applyFit()
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        guard size.width > 1, size.height > 1 else { return }
        hasSize = true
        aspect = Float(size.width / size.height)
        projection = Self.perspective(fovY: CameraMath.defaultFovY, aspect: aspect, near: 0.1, far: 1000)
        if !userMoved { applyFit() }
    }

    func draw(in view: MTKView) {
        guard let pipeline, let commandQueue, !uniformBuffers.isEmpty,
              let drawable = view.currentDrawable,
              let pass = view.currentRenderPassDescriptor else { return }
        _ = slotGate.wait(timeout: .distantFuture)
        uniformIndex = (uniformIndex + 1) % uniformBuffers.count
        let uniforms = uniformBuffers[uniformIndex]
        var block = Uniforms(modelViewProjection: projection * viewMatrix)
        memcpy(uniforms.contents(), &block, MemoryLayout<Uniforms>.stride)
        guard let buffer = commandQueue.makeCommandBuffer() else {
            slotGate.signal()
            return
        }
        let gate = slotGate
        buffer.addCompletedHandler { _ in gate.signal() }
        pass.colorAttachments[0].clearColor = MTLClearColor(red: 0.043, green: 0.051, blue: 0.063, alpha: 1)
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].storeAction = .store
        guard let encoder = buffer.makeRenderCommandEncoder(descriptor: pass) else {
            slotGate.signal()
            return
        }
        encoder.setRenderPipelineState(pipeline)
        encoder.setVertexBuffer(uniforms, offset: 0, index: 1)
        encoder.setDepthStencilState(depthWrite)
        drawLines(encoder, frameBuffer, frameCount)
        drawPoints(encoder, pointBuffer, pointCount)
        encoder.setDepthStencilState(depthRead)
        drawTriangles(encoder, fillBuffer, fillCount)
        drawLines(encoder, edgeBuffer, edgeCount)
        encoder.setDepthStencilState(depthAlways)
        drawTriangles(encoder, topFillBuffer, topFillCount)
        drawLines(encoder, topEdgeBuffer, topEdgeCount)
        drawPoints(encoder, markBuffer, markCount)
        encoder.endEncoding()
        buffer.present(drawable)
        buffer.commit()
    }

    // MARK: Upload

    private func clearGeometry() {
        pointBuffer = nil
        pointCount = 0
        frameBuffer = nil
        frameCount = 0
        fillBuffer = nil
        fillCount = 0
        edgeBuffer = nil
        edgeCount = 0
        topFillBuffer = nil
        topFillCount = 0
        topEdgeBuffer = nil
        topEdgeCount = 0
        markBuffer = nil
        markCount = 0
    }

    private func uploadMap(_ map: LiveRoomVoxels.Map, opacity: Float) {
        let grey = SIMD3<Float>(Float(197) / 255, Float(205) / 255, Float(214) / 255)
        let frame = SIMD3<Float>(Float(92) / 255, Float(102) / 255, Float(112) / 255)
        var points: [Vertex] = []
        points.reserveCapacity(map.points.count)
        for point in map.points {
            points.append(vertex(point, grey, opacity, 4))
        }
        var lines: [Vertex] = []
        for corners in map.frames {
            guard corners.count == 8 else { continue }
            for pair in LiveRoomMarkers.edgePairs {
                lines.append(vertex(corners[pair.0], frame, opacity, 1))
                lines.append(vertex(corners[pair.1], frame, opacity, 1))
            }
        }
        pointBuffer = buffer(points)
        pointCount = points.count
        frameBuffer = buffer(lines)
        frameCount = lines.count
    }

    private func uploadMarkers(_ markers: [LiveRoomMarker]) {
        var fills: [Vertex] = []
        var edges: [Vertex] = []
        var topFills: [Vertex] = []
        var topEdges: [Vertex] = []
        var marks: [Vertex] = []
        for marker in markers {
            let rgba = marker.color
            var face: [Vertex] = []
            if marker.fillAlpha > 0, marker.corners.count == 8 {
                for tri in LiveRoomMarkers.boxTriangles {
                    face.append(vertex(marker.corners[tri.0], rgba, marker.fillAlpha, 1))
                    face.append(vertex(marker.corners[tri.1], rgba, marker.fillAlpha, 1))
                    face.append(vertex(marker.corners[tri.2], rgba, marker.fillAlpha, 1))
                }
            }
            var lines: [Vertex] = []
            if marker.corners.count == 8 {
                for pair in LiveRoomMarkers.edgePairs {
                    let a = marker.corners[pair.0]
                    let b = marker.corners[pair.1]
                    if marker.dashed {
                        lines.append(contentsOf: dashes(a, b, rgba))
                    } else {
                        lines.append(vertex(a, rgba, 1, 1))
                        lines.append(vertex(b, rgba, 1, 1))
                    }
                }
            }
            if let arrow = marker.arrow {
                for segment in [(arrow.tail, arrow.tip), (arrow.tip, arrow.left), (arrow.tip, arrow.right)] {
                    lines.append(vertex(segment.0, rgba, 1, 1))
                    lines.append(vertex(segment.1, rgba, 1, 1))
                }
            }
            marks.append(vertex(marker.centroid, rgba, 1, 6))
            if marker.onTop {
                topFills.append(contentsOf: face)
                topEdges.append(contentsOf: lines)
            } else {
                fills.append(contentsOf: face)
                edges.append(contentsOf: lines)
            }
        }
        fillBuffer = buffer(fills)
        fillCount = fills.count
        edgeBuffer = buffer(edges)
        edgeCount = edges.count
        topFillBuffer = buffer(topFills)
        topFillCount = topFills.count
        topEdgeBuffer = buffer(topEdges)
        topEdgeCount = topEdges.count
        markBuffer = buffer(marks)
        markCount = marks.count
    }

    private func dashes(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ color: SIMD3<Float>) -> [Vertex] {
        var out: [Vertex] = []
        let slots = 15
        for index in stride(from: 0, to: slots, by: 2) {
            let t0 = Float(index) / Float(slots)
            let t1 = Float(index + 1) / Float(slots)
            out.append(vertex(simd_mix(a, b, SIMD3(repeating: t0)), color, 1, 1))
            out.append(vertex(simd_mix(a, b, SIMD3(repeating: t1)), color, 1, 1))
        }
        return out
    }

    private func vertex(_ position: SIMD3<Float>, _ color: SIMD3<Float>, _ alpha: Float, _ size: Float) -> Vertex {
        Vertex(
            x: position.x, y: position.y, z: position.z, pad: 0,
            r: color.x, g: color.y, b: color.z, a: alpha,
            size: size, pad2: 0
        )
    }

    private func buffer(_ vertices: [Vertex]) -> MTLBuffer? {
        guard let device, !vertices.isEmpty else { return nil }
        guard let buffer = device.makeBuffer(length: vertices.count * MemoryLayout<Vertex>.stride, options: .storageModeShared) else {
            return nil
        }
        vertices.withUnsafeBytes { raw in
            buffer.contents().copyMemory(from: raw.baseAddress!, byteCount: raw.count)
        }
        return buffer
    }

    private func drawLines(_ encoder: MTLRenderCommandEncoder, _ buffer: MTLBuffer?, _ count: Int) {
        guard let buffer, count > 0 else { return }
        encoder.setVertexBuffer(buffer, offset: 0, index: 0)
        encoder.drawPrimitives(type: .line, vertexStart: 0, vertexCount: count)
    }

    private func drawPoints(_ encoder: MTLRenderCommandEncoder, _ buffer: MTLBuffer?, _ count: Int) {
        guard let buffer, count > 0 else { return }
        encoder.setVertexBuffer(buffer, offset: 0, index: 0)
        encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: count)
    }

    private func drawTriangles(_ encoder: MTLRenderCommandEncoder, _ buffer: MTLBuffer?, _ count: Int) {
        guard let buffer, count > 0 else { return }
        encoder.setVertexBuffer(buffer, offset: 0, index: 0)
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: count)
    }

    // MARK: Camera

    private func noteContent() {
        let key = boundsKey(content)
        guard !key.isEmpty else { return }
        if !userMoved, key != fittedKey { applyFit() }
    }

    private func applyFit() {
        guard hasSize, let box = bounds(of: content) else { return }
        let (eyeP, centerP, _) = CameraMath.fitToBounds(
            min: Point3D(x: box.min.x, y: box.min.y, z: box.min.z),
            max: Point3D(x: box.max.x, y: box.max.y, z: box.max.z),
            fovY: CameraMath.defaultFovY,
            aspect: aspect,
            margin: 0.9,
            elevation: CameraMath.defaultElevation,
            near: 0.1
        )
        eye = SIMD3(eyeP.x, eyeP.y, eyeP.z)
        target = SIMD3(centerP.x, centerP.y, centerP.z)
        viewMatrix = Self.lookAt(eye: eye, center: target, up: SIMD3(0, 0, 1))
        fittedKey = boundsKey(content)
    }

    private func bounds(of points: [SIMD3<Float>]) -> (min: SIMD3<Float>, max: SIMD3<Float>)? {
        guard var lo = points.first, var hi = points.first else { return nil }
        for point in points.dropFirst() {
            lo = simd_min(lo, point)
            hi = simd_max(hi, point)
        }
        if lo.x == hi.x { lo.x -= 0.05; hi.x += 0.05 }
        if lo.y == hi.y { lo.y -= 0.05; hi.y += 0.05 }
        if lo.z == hi.z { lo.z -= 0.05; hi.z += 0.05 }
        return (lo, hi)
    }

    private func boundsKey(_ points: [SIMD3<Float>]) -> String {
        guard let box = bounds(of: points) else { return "" }
        func q(_ value: Float) -> Int { Int((value * 1000).rounded()) }
        return "\(q(box.min.x)) \(q(box.min.y)) \(q(box.min.z)) \(q(box.max.x)) \(q(box.max.y)) \(q(box.max.z))"
    }

    private static func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
        let z = simd_normalize(eye - center)
        let x = simd_normalize(simd_cross(up, z))
        let y = simd_cross(z, x)
        let t = SIMD3<Float>(-simd_dot(x, eye), -simd_dot(y, eye), -simd_dot(z, eye))
        return simd_float4x4(
            SIMD4(x.x, y.x, z.x, 0),
            SIMD4(x.y, y.y, z.y, 0),
            SIMD4(x.z, y.z, z.z, 0),
            SIMD4(t.x, t.y, t.z, 1)
        )
    }

    private static func perspective(fovY: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
        let yScale = 1 / tan(fovY * 0.5)
        let xScale = yScale / aspect
        let zRange = far - near
        return simd_float4x4(
            SIMD4(xScale, 0, 0, 0),
            SIMD4(0, yScale, 0, 0),
            SIMD4(0, 0, -far / zRange, -1),
            SIMD4(0, 0, -far * near / zRange, 0)
        )
    }

    private func buildPipeline(_ device: MTLDevice) {
        let library: MTLLibrary
        do {
            library = try device.makeLibrary(source: Self.shader, options: nil)
        } catch {
            print("RoomMapRenderer: \(error)")
            return
        }
        guard let vertex = library.makeFunction(name: "roomVertex"),
              let fragment = library.makeFunction(name: "roomFragment") else { return }
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = 0
        descriptor.attributes[0].bufferIndex = 0
        descriptor.attributes[1].format = .float4
        descriptor.attributes[1].offset = 16
        descriptor.attributes[1].bufferIndex = 0
        descriptor.attributes[2].format = .float
        descriptor.attributes[2].offset = 32
        descriptor.attributes[2].bufferIndex = 0
        descriptor.layouts[0].stride = MemoryLayout<Vertex>.stride
        descriptor.layouts[0].stepFunction = .perVertex
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.vertexFunction = vertex
        pipelineDescriptor.fragmentFunction = fragment
        pipelineDescriptor.vertexDescriptor = descriptor
        pipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        pipelineDescriptor.colorAttachments[0].isBlendingEnabled = true
        pipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        pipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        pipelineDescriptor.depthAttachmentPixelFormat = .depth32Float
        pipeline = try? device.makeRenderPipelineState(descriptor: pipelineDescriptor)
        depthWrite = depth(device, compare: .less, write: true)
        depthRead = depth(device, compare: .less, write: false)
        depthAlways = depth(device, compare: .always, write: false)
    }

    private func depth(_ device: MTLDevice, compare: MTLCompareFunction, write: Bool) -> MTLDepthStencilState? {
        let descriptor = MTLDepthStencilDescriptor()
        descriptor.depthCompareFunction = compare
        descriptor.isDepthWriteEnabled = write
        return device.makeDepthStencilState(descriptor: descriptor)
    }

    private static let shader = """
    #include <metal_stdlib>
    using namespace metal;

    struct VertexIn {
        float3 position [[attribute(0)]];
        float4 color [[attribute(1)]];
        float size [[attribute(2)]];
    };

    struct VertexOut {
        float4 position [[position]];
        float size [[point_size]];
        float4 color;
    };

    struct Uniforms {
        float4x4 modelViewProjection;
    };

    vertex VertexOut roomVertex(VertexIn in [[stage_in]], constant Uniforms& uniforms [[buffer(1)]]) {
        VertexOut out;
        out.position = uniforms.modelViewProjection * float4(in.position, 1.0);
        out.size = in.size;
        out.color = in.color;
        return out;
    }

    fragment float4 roomFragment(VertexOut in [[stage_in]]) {
        return in.color;
    }
    """
}
