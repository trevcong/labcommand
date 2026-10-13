import Foundation
import simd
import SwiftUI
import Combine

/// Camera controller for point cloud visualization with orbit and fly modes.
/// Orbit mode: trackpad gestures for rotate/zoom/pan around a target point.
/// Fly mode: WASD + mouse for first-person flight.
@MainActor
final class PointCloudCamera: ObservableObject {
    enum Mode: String, CaseIterable {
        case orbit
        case fly
    }
    
    // MARK: - Published State
    
    @Published private(set) var mode: Mode = .orbit
    @Published private(set) var viewMatrix: simd_float4x4 = matrix_identity_float4x4
    @Published private(set) var projectionMatrix: simd_float4x4 = matrix_identity_float4x4
    
    // MARK: - Orbit Mode State
    
    /// Distance from target in orbit mode
    var distance: Float = 10.0 {
        didSet { distance = clamp(distance, min: minDistance, max: maxDistance) }
    }
    /// Horizontal angle (radians) around target
    var azimuth: Float = 0.0
    /// Vertical angle (radians) from horizon
    var elevation: Float = 0.0 {
        didSet { elevation = clamp(elevation, min: minElevation, max: maxElevation) }
    }
    /// Point the camera orbits around
    var target: SIMD3<Float> = .zero
    
    // MARK: - Fly Mode State
    
    /// Camera position in fly mode
    var position: SIMD3<Float> = SIMD3<Float>(0, 0, 10)
    /// Horizontal rotation (radians)
    var yaw: Float = 0.0
    /// Vertical rotation (radians)
    var pitch: Float = 0.0 {
        didSet { pitch = clamp(pitch, min: -maxPitch, max: maxPitch) }
    }
    
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
    var minElevation: Float = -.pi / 2 + 0.01
    var maxElevation: Float = .pi / 2 - 0.01
    var maxPitch: Float = .pi / 2 - 0.01
    
    /// Bounds for the viewable area (prevents losing the point cloud)
    var boundingBox: (min: SIMD3<Float>, max: SIMD3<Float>)? = nil
    
    // MARK: - Interpolation Targets
    
    private var targetDistance: Float = 10.0
    private var targetAzimuth: Float = 0.0
    private var targetElevation: Float = 0.0
    private var targetOrbitCenter: SIMD3<Float> = .zero
    
    private var targetPosition: SIMD3<Float> = SIMD3<Float>(0, 0, 10)
    private var targetYaw: Float = 0.0
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
            let orbitPosition = computeOrbitPosition()
            position = orbitPosition
            targetPosition = orbitPosition
            
            yaw = azimuth + .pi
            targetYaw = yaw
            pitch = -elevation
            targetPitch = pitch
        } else {
            target = position + forward * distance
            targetOrbitCenter = target
            
            azimuth = yaw - .pi
            targetAzimuth = azimuth
            elevation = -pitch
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
        targetDistance = clamp(targetDistance - zoomDelta, min: minDistance, max: maxDistance)
    }
    
    func handleRotation(_ delta: CGSize) {
        guard mode == .orbit else { return }
        targetAzimuth -= Float(delta.width) * orbitRotateSensitivity
        targetElevation += Float(delta.height) * orbitRotateSensitivity
        targetElevation = clamp(targetElevation, min: minElevation, max: maxElevation)
    }
    
    func handlePan(_ delta: CGSize) {
        guard mode == .orbit else { return }
        
        let right = computeOrbitRight()
        let up = computeOrbitUp()
        
        let panX = Float(delta.width) * orbitPanSensitivity * (distance / 10.0)
        let panY = Float(delta.height) * orbitPanSensitivity * (distance / 10.0)
        
        var newTarget = targetOrbitCenter - right * panX + up * panY
        newTarget = clampToBounds(newTarget)
        targetOrbitCenter = newTarget
    }
    
    func handlePinch(_ scale: CGFloat) {
        guard mode == .orbit else { return }
        let newDistance = targetDistance / Float(scale)
        targetDistance = clamp(newDistance, min: minDistance, max: maxDistance)
    }
    
    // MARK: - Fly Mode Controls
    
    func handleKeyDown(_ key: KeyEquivalent) {
        guard mode == .fly else { return }
        switch key {
        case "w", "W": moveForward = 1
        case "s", "S": moveForward = -1
        case "a", "A": moveRight = -1
        case "d", "D": moveRight = 1
        case " ": moveUp = 1
        case KeyEquivalent(Character(UnicodeScalar(16))): moveUp = -1 // Shift detection via modifier
        default: break
        }
    }
    
    func handleKeyUp(_ key: KeyEquivalent) {
        guard mode == .fly else { return }
        switch key {
        case "w", "W": if moveForward > 0 { moveForward = 0 }
        case "s", "S": if moveForward < 0 { moveForward = 0 }
        case "a", "A": if moveRight < 0 { moveRight = 0 }
        case "d", "D": if moveRight > 0 { moveRight = 0 }
        case " ": if moveUp > 0 { moveUp = 0 }
        case KeyEquivalent(Character(UnicodeScalar(16))): if moveUp < 0 { moveUp = 0 }
        default: break
        }
    }
    
    func setMoveForward(_ value: Float) {
        guard mode == .fly else { return }
        moveForward = clamp(value, min: -1, max: 1)
    }
    
    func setMoveRight(_ value: Float) {
        guard mode == .fly else { return }
        moveRight = clamp(value, min: -1, max: 1)
    }
    
    func setMoveUp(_ value: Float) {
        guard mode == .fly else { return }
        moveUp = clamp(value, min: -1, max: 1)
    }
    
    func stopMovement() {
        moveForward = 0
        moveRight = 0
        moveUp = 0
    }
    
    func handleMouseDelta(_ delta: CGSize) {
        guard mode == .fly else { return }
        targetYaw += Float(delta.width) * flyLookSensitivity
        targetPitch -= Float(delta.height) * flyLookSensitivity
        targetPitch = clamp(targetPitch, min: -maxPitch, max: maxPitch)
    }
    
    // MARK: - Update Loop
    
    func update(deltaTime: Float) {
        let t = min(1.0, deltaTime * interpolationSpeed)
        
        switch mode {
        case .orbit:
            distance = lerp(distance, targetDistance, t)
            azimuth = lerpAngle(azimuth, targetAzimuth, t)
            elevation = lerp(elevation, targetElevation, t)
            target = lerp(target, targetOrbitCenter, t)
            
        case .fly:
            yaw = lerpAngle(yaw, targetYaw, t)
            pitch = lerp(pitch, targetPitch, t)
            
            if moveForward != 0 || moveRight != 0 || moveUp != 0 {
                let velocity = (forward * moveForward + right * moveRight + up * moveUp)
                    * flyMoveSensitivity * deltaTime
                targetPosition = clampToBounds(targetPosition + velocity)
            }
            
            position = lerp(position, targetPosition, t)
        }
        
        updateMatrices()
    }
    
    // MARK: - Matrix Computation
    
    private func updateMatrices() {
        switch mode {
        case .orbit:
            viewMatrix = computeOrbitViewMatrix()
        case .fly:
            viewMatrix = computeFlyViewMatrix()
        }
        projectionMatrix = computeProjectionMatrix()
    }
    
    private func computeOrbitViewMatrix() -> simd_float4x4 {
        let eye = computeOrbitPosition()
        return lookAt(eye: eye, target: target, up: SIMD3<Float>(0, 1, 0))
    }
    
    private func computeFlyViewMatrix() -> simd_float4x4 {
        let target = position + forward
        return lookAt(eye: position, target: target, up: SIMD3<Float>(0, 1, 0))
    }
    
    private func computeProjectionMatrix() -> simd_float4x4 {
        perspective(fovY: fieldOfView * .pi / 180.0, aspect: aspectRatio, near: nearPlane, far: farPlane)
    }
    
    // MARK: - Position & Direction Helpers
    
    private func computeOrbitPosition() -> SIMD3<Float> {
        let x = distance * cos(elevation) * sin(azimuth)
        let y = distance * sin(elevation)
        let z = distance * cos(elevation) * cos(azimuth)
        return target + SIMD3<Float>(x, y, z)
    }
    
    private func computeOrbitRight() -> SIMD3<Float> {
        let x = cos(azimuth)
        let z = -sin(azimuth)
        return SIMD3<Float>(x, 0, z)
    }
    
    private func computeOrbitUp() -> SIMD3<Float> {
        let x = -sin(elevation) * sin(azimuth)
        let y = cos(elevation)
        let z = -sin(elevation) * cos(azimuth)
        return SIMD3<Float>(x, y, z)
    }
    
    var forward: SIMD3<Float> {
        let x = cos(pitch) * sin(yaw)
        let y = sin(pitch)
        let z = cos(pitch) * cos(yaw)
        return SIMD3<Float>(x, y, z)
    }
    
    var right: SIMD3<Float> {
        let x = cos(yaw)
        let z = -sin(yaw)
        return SIMD3<Float>(x, 0, z)
    }
    
    var up: SIMD3<Float> {
        simd_cross(right, forward)
    }
    
    // MARK: - Bounds Clamping
    
    private func clampToBounds(_ point: SIMD3<Float>) -> SIMD3<Float> {
        guard let bounds = boundingBox else { return point }
        return SIMD3<Float>(
            clamp(point.x, min: bounds.min.x, max: bounds.max.x),
            clamp(point.y, min: bounds.min.y, max: bounds.max.y),
            clamp(point.z, min: bounds.min.z, max: bounds.max.z)
        )
    }
    
    // MARK: - Public Configuration
    
    func setBounds(min: SIMD3<Float>, max: SIMD3<Float>) {
        boundingBox = (min: min, max: max)
    }
    
    func clearBounds() {
        boundingBox = nil
    }
    
    func resetToFitBounds() {
        guard let bounds = boundingBox else { return }
        let center = (bounds.min + bounds.max) * 0.5
        let size = bounds.max - bounds.min
        let maxExtent = max(size.x, max(size.y, size.z))
        
        if mode == .orbit {
            targetOrbitCenter = center
            targetDistance = maxExtent * 1.5
            targetAzimuth = 0
            targetElevation = .pi / 6
        } else {
            targetPosition = center + SIMD3<Float>(0, maxExtent * 0.5, maxExtent * 1.5)
            targetYaw = 0
            targetPitch = -.pi / 6
        }
    }
    
    func setAspectRatio(_ aspect: Float) {
        aspectRatio = aspect
        updateMatrices()
    }
}

// MARK: - Math Utilities

private func clamp(_ value: Float, min minVal: Float, max maxVal: Float) -> Float {
    Swift.min(Swift.max(value, minVal), maxVal)
}

private func lerp(_ a: Float, _ b: Float, _ t: Float) -> Float {
    a + (b - a) * t
}

private func lerp(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ t: Float) -> SIMD3<Float> {
    a + (b - a) * t
}

private func lerpAngle(_ a: Float, _ b: Float, _ t: Float) -> Float {
    var diff = b - a
    while diff > .pi { diff -= 2 * .pi }
    while diff < -.pi { diff += 2 * .pi }
    return a + diff * t
}

private func lookAt(eye: SIMD3<Float>, target: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
    let f = simd_normalize(target - eye)
    let s = simd_normalize(simd_cross(f, up))
    let u = simd_cross(s, f)
    
    var result = matrix_identity_float4x4
    result[0][0] = s.x
    result[1][0] = s.y
    result[2][0] = s.z
    result[0][1] = u.x
    result[1][1] = u.y
    result[2][1] = u.z
    result[0][2] = -f.x
    result[1][2] = -f.y
    result[2][2] = -f.z
    result[3][0] = -simd_dot(s, eye)
    result[3][1] = -simd_dot(u, eye)
    result[3][2] = simd_dot(f, eye)
    return result
}

private func perspective(fovY: Float, aspect: Float, near: Float, far: Float) -> simd_float4x4 {
    let yScale = 1.0 / tan(fovY * 0.5)
    let xScale = yScale / aspect
    let zRange = far - near
    
    var result = simd_float4x4(0)
    result[0][0] = xScale
    result[1][1] = yScale
    result[2][2] = -(far + near) / zRange
    result[2][3] = -1.0
    result[3][2] = -2.0 * far * near / zRange
    return result
}
