import Foundation
import simd
import SwiftUI
import Combine
import LabCommandCore

/// Camera controller for point cloud visualization with orbit and fly modes.
/// Uses Z-up coordinate convention and integrates with PointCloudRenderer.
@MainActor
final class PointCloudCamera: ObservableObject {
    enum Mode: String, CaseIterable, Sendable {
        case orbit
        case fly
    }
    
    // MARK: - Published State
    
    @Published private(set) var mode: Mode = .orbit
    @Published private(set) var viewMatrix: simd_float4x4 = matrix_identity_float4x4
    @Published private(set) var projectionMatrix: simd_float4x4 = matrix_identity_float4x4
    
    // MARK: - Orbit Mode State
    
    /// Distance from target in orbit mode
    private(set) var distance: Float = 10.0
    /// Horizontal angle (radians, rotation around Z axis)
    private(set) var azimuth: Float = 0.0
    /// Vertical angle (radians from XY plane)
    private(set) var elevation: Float = Float.pi / 6
    /// Point the camera orbits around
    private(set) var target: Float3 = .zero
    
    // MARK: - Fly Mode State
    
    /// Camera position in fly mode
    private(set) var position: Float3 = Float3(0, -10, 5)
    /// Horizontal rotation (radians, rotation around Z axis)
    private(set) var yaw: Float = Float.pi / 2
    /// Vertical rotation (radians from XY plane)
    private(set) var pitch: Float = 0.0
    
    // MARK: - Movement State (for smooth interpolation)
    
    private var moveForward: Float = 0
    private var moveRight: Float = 0
    private var moveUp: Float = 0
    
    // MARK: - Configuration
    
    var fieldOfView: Float = 60.0
    var aspectRatio: Float = 16.0 / 9.0
    var nearPlane: Float = 0.1
    var farPlane: Float = 1000.0
    
    var orbitRotateSensitivity: Float = 0.005
    var orbitZoomSensitivity: Float = 0.02
    var orbitPanSensitivity: Float = 0.02
    
    var flyMoveSensitivity: Float = 10.0
    var flyLookSensitivity: Float = 0.003
    
    var interpolationSpeed: Float = 8.0
    
    var minDistance: Float = 0.5
    var maxDistance: Float = 500.0
    var minElevation: Float = -Float.pi / 2 + 0.01
    var maxElevation: Float = Float.pi / 2 - 0.01
    var maxPitch: Float = Float.pi / 2 - 0.01
    
    /// Bounds for orbit target clamping (fly mode is unclamped)
    var boundingBox: (min: Float3, max: Float3)? = nil
    
    // MARK: - Interpolation Targets
    
    private var targetDistance: Float = 10.0
    private var targetAzimuth: Float = 0.0
    private var targetElevation: Float = Float.pi / 6
    private var targetOrbitCenter: Float3 = .zero
    
    private var targetPosition: Float3 = Float3(0, -10, 5)
    private var targetYaw: Float = Float.pi / 2
    private var targetPitch: Float = 0.0
    
    // MARK: - Initialization
    
    init() {
        targetDistance = distance
        targetAzimuth = azimuth
        targetElevation = elevation
        targetOrbitCenter = target
        targetPosition = position
        targetYaw = yaw
        targetPitch = pitch
        updateMatrices()
    }
    
    // MARK: - Mode Switching
    
    func setMode(_ newMode: Mode) {
        guard newMode != mode else { return }
        
        if newMode == .fly {
            let orbitPos = CameraMath.orbitPosition(
                target: target,
                distance: distance,
                azimuth: azimuth,
                elevation: elevation
            )
            position = orbitPos
            targetPosition = orbitPos
            
            yaw = CameraMath.wrapAngle(azimuth + Float.pi)
            targetYaw = yaw
            pitch = elevation
            targetPitch = pitch
        } else {
            let forward = CameraMath.flyForward(yaw: yaw, pitch: pitch)
            target = position + forward * distance
            targetOrbitCenter = target
            
            azimuth = CameraMath.wrapAngle(yaw - Float.pi)
            targetAzimuth = azimuth
            elevation = CameraMath.clamp(pitch, min: minElevation, max: maxElevation)
            targetElevation = elevation
        }
        
        mode = newMode
        updateMatrices()
    }
    
    func toggleMode() {
        setMode(mode == .orbit ? .fly : .orbit)
    }
    
    // MARK: - Orbit Mode Gestures
    
    func handleScroll(_ delta: CGSize) {
        guard mode == .orbit else { return }
        let zoomDelta = Float(delta.height) * orbitZoomSensitivity
        targetDistance = CameraMath.clamp(targetDistance - zoomDelta, min: minDistance, max: maxDistance)
    }
    
    func handleRotation(_ delta: CGSize) {
        guard mode == .orbit else { return }
        targetAzimuth = CameraMath.wrapAngle(targetAzimuth - Float(delta.width) * orbitRotateSensitivity)
        targetElevation += Float(delta.height) * orbitRotateSensitivity
        targetElevation = CameraMath.clamp(targetElevation, min: minElevation, max: maxElevation)
    }
    
    func handlePan(_ delta: CGSize) {
        guard mode == .orbit else { return }
        
        let right = CameraMath.orbitRight(azimuth: azimuth)
        let up = CameraMath.orbitUp(azimuth: azimuth, elevation: elevation)
        
        let panX = Float(delta.width) * orbitPanSensitivity * (distance / 10.0)
        let panY = Float(delta.height) * orbitPanSensitivity * (distance / 10.0)
        
        var newTarget = targetOrbitCenter - right * panX + up * panY
        
        // Clamp orbit target to bounds if set
        if let bounds = boundingBox {
            newTarget = CameraMath.clampToBounds(newTarget, min: bounds.min, max: bounds.max)
        }
        
        targetOrbitCenter = CameraMath.sanitize(newTarget, default: targetOrbitCenter)
    }
    
    func handlePinch(_ scale: CGFloat) {
        guard mode == .orbit else { return }
        let newDistance = targetDistance / Float(scale)
        targetDistance = CameraMath.clamp(newDistance, min: minDistance, max: maxDistance)
    }
    
    // MARK: - Fly Mode Controls
    
    func setMoveForward(_ value: Float) {
        guard mode == .fly else { return }
        moveForward = CameraMath.clamp(value, min: -1, max: 1)
    }
    
    func setMoveRight(_ value: Float) {
        guard mode == .fly else { return }
        moveRight = CameraMath.clamp(value, min: -1, max: 1)
    }
    
    func setMoveUp(_ value: Float) {
        guard mode == .fly else { return }
        moveUp = CameraMath.clamp(value, min: -1, max: 1)
    }
    
    func stopMovement() {
        moveForward = 0
        moveRight = 0
        moveUp = 0
    }
    
    func handleMouseDelta(_ delta: CGSize) {
        guard mode == .fly else { return }
        targetYaw = CameraMath.wrapAngle(targetYaw + Float(delta.width) * flyLookSensitivity)
        targetPitch -= Float(delta.height) * flyLookSensitivity
        targetPitch = CameraMath.clamp(targetPitch, min: -maxPitch, max: maxPitch)
    }
    
    // MARK: - Update Loop
    
    func update(deltaTime: Float) {
        guard !CameraMath.isInvalid(deltaTime) && deltaTime > 0 else { return }
        
        let t = min(1.0, deltaTime * interpolationSpeed)
        
        switch mode {
        case .orbit:
            distance = CameraMath.lerp(distance, targetDistance, t)
            azimuth = CameraMath.lerpAngle(azimuth, targetAzimuth, t)
            elevation = CameraMath.lerp(elevation, targetElevation, t)
            target = CameraMath.lerp(target, targetOrbitCenter, t)
            
        case .fly:
            yaw = CameraMath.lerpAngle(yaw, targetYaw, t)
            pitch = CameraMath.lerp(pitch, targetPitch, t)
            
            // Fly mode: no bounds clamping - let the user explore freely
            if moveForward != 0 || moveRight != 0 || moveUp != 0 {
                let forward = CameraMath.flyForward(yaw: yaw, pitch: pitch)
                let right = CameraMath.flyRight(yaw: yaw)
                let up = CameraMath.flyUp()
                
                let velocity = (forward * moveForward + right * moveRight + up * moveUp)
                    * flyMoveSensitivity * deltaTime
                
                targetPosition = CameraMath.sanitize(targetPosition + velocity, default: targetPosition)
            }
            
            position = CameraMath.lerp(position, targetPosition, t)
        }
        
        // Sanitize all values
        distance = CameraMath.sanitize(distance, default: 10.0)
        azimuth = CameraMath.wrapAngle(azimuth)
        
        updateMatrices()
    }
    
    // MARK: - Matrix Computation
    
    private func updateMatrices() {
        let coreMat: Float4x4
        
        switch mode {
        case .orbit:
            coreMat = CameraMath.orbitViewMatrix(
                target: target,
                distance: distance,
                azimuth: azimuth,
                elevation: elevation
            )
        case .fly:
            coreMat = CameraMath.flyViewMatrix(position: position, yaw: yaw, pitch: pitch)
        }
        
        viewMatrix = toSimd(coreMat)
        projectionMatrix = toSimd(CameraMath.perspectiveMetal(
            fovY: fieldOfView * .pi / 180.0,
            aspect: aspectRatio,
            near: nearPlane,
            far: farPlane
        ))
    }
    
    /// Convert CameraMath Float4x4 to simd_float4x4 for Metal.
    private func toSimd(_ m: Float4x4) -> simd_float4x4 {
        simd_float4x4(
            SIMD4<Float>(m[0].x, m[0].y, m[0].z, m[0].w),
            SIMD4<Float>(m[1].x, m[1].y, m[1].z, m[1].w),
            SIMD4<Float>(m[2].x, m[2].y, m[2].z, m[2].w),
            SIMD4<Float>(m[3].x, m[3].y, m[3].z, m[3].w)
        )
    }
    
    // MARK: - Computed Properties
    
    var forward: Float3 {
        CameraMath.flyForward(yaw: yaw, pitch: pitch)
    }
    
    var right: Float3 {
        CameraMath.flyRight(yaw: yaw)
    }
    
    var up: Float3 {
        CameraMath.flyUp()
    }
    
    // MARK: - Public Configuration
    
    func setBounds(min: Float3, max: Float3) {
        boundingBox = (min: min, max: max)
    }
    
    func clearBounds() {
        boundingBox = nil
    }
    
    func resetToFitBounds() {
        guard let bounds = boundingBox else { return }
        let center = CameraMath.boundsCenter(min: bounds.min, max: bounds.max)
        guard !CameraMath.isInvalid(center) else { return }
        
        let size = bounds.max - bounds.min
        let maxExtent = max(size.x, max(size.y, size.z))
        guard !CameraMath.isInvalid(maxExtent) && maxExtent > 0 else { return }
        
        if mode == .orbit {
            targetOrbitCenter = center
            targetDistance = CameraMath.clamp(maxExtent * 1.5, min: minDistance, max: maxDistance)
            targetAzimuth = 0
            targetElevation = CameraMath.clamp(Float.pi / 6, min: minElevation, max: maxElevation)
        } else {
            targetPosition = center + Float3(0, -maxExtent * 1.5, maxExtent * 0.5)
            targetYaw = Float.pi / 2
            targetPitch = CameraMath.clamp(Float.pi / 6, min: -maxPitch, max: maxPitch)
        }
    }
    
    func setAspectRatio(_ aspect: Float) {
        guard !CameraMath.isInvalid(aspect) && aspect > 0 else { return }
        aspectRatio = aspect
        updateMatrices()
    }
}
