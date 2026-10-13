import SwiftUI
import AppKit

/// SwiftUI view modifier that handles camera gestures for the point cloud viewer.
/// Note: Primary gesture handling is now in PointCloudMTKView. This modifier is
/// kept for compatibility with SwiftUI-only views that don't use the MTKView directly.
@MainActor
struct PointCloudGestureHandler: ViewModifier {
    @ObservedObject var camera: PointCloudCamera
    
    @State private var lastDragLocation: CGPoint?
    @State private var lastMagnification: CGFloat = 1.0
    
    func body(content: Content) -> some View {
        content
            .gesture(rotateGesture)
            .gesture(magnifyGesture)
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
}

extension View {
    func pointCloudGestures(camera: PointCloudCamera) -> some View {
        modifier(PointCloudGestureHandler(camera: camera))
    }
}
