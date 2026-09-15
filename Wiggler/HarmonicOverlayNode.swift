import SceneKit
import UIKit

/// Composites a camera-aligned image of any resolution below the world-space axis and ray.
///
/// The quad is a child of the point of view, spanning the viewport half a metre in front of the camera, so it
/// moves rigidly with the camera: a screen-filling quad placed in world space a millimetre from the lens would
/// be thrown across the screen by ARKit's sub-millimetre pose jitter.
final class HarmonicOverlayNode: SCNNode {
    private let material = SCNMaterial()
    private let triangles = SCNGeometryElement(indices: [UInt16(0), 1, 2, 2, 1, 3], primitiveType: .triangles)
    private let corners = [CGPoint(x: 0, y: 0), CGPoint(x: 1, y: 0), CGPoint(x: 0, y: 1), CGPoint(x: 1, y: 1)]
    private let distance: Float = 0.5
    private var image: CGImage?
    private var quad: (projection: SCNMatrix4, viewport: CGSize, display: CGAffineTransform)?

    override init() {
        super.init()
        renderingOrder = -100
        material.lightingModel = .constant
        material.isDoubleSided = true
        material.readsFromDepthBuffer = false
        material.writesToDepthBuffer = false
        material.diffuse.wrapS = .clamp
        material.diffuse.wrapT = .clamp
        material.diffuse.magnificationFilter = .linear
        isHidden = true
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    /// Render thread. The display transform maps top-left-origin image coordinates to view coordinates.
    func update(
        image: CGImage?, displayTransform: CGAffineTransform, viewportSize: CGSize,
        renderer: SCNSceneRenderer
    ) {
        guard let image, let camera = renderer.pointOfView, viewportSize.width > 0, viewportSize.height > 0
        else {
            isHidden = true
            self.image = nil
            material.diffuse.contents = nil
            return
        }
        if parent !== camera {
            removeFromParentNode()
            camera.addChildNode(self)
        }
        isHidden = false
        if self.image !== image {
            self.image = image
            material.diffuse.contents = image
        }
        let projection = camera.camera?.projectionTransform ?? SCNMatrix4Identity
        if let quad, SCNMatrix4EqualToMatrix4(quad.projection, projection), quad.viewport == viewportSize,
            quad.display == displayTransform
        {
            return
        }
        quad = (projection, viewportSize, displayTransform)
        // Viewport corners along their rays, `distance` in front of the camera, in the camera's own frame —
        // unprojected and converted within the same frame, so the camera's later motion carries the quad along.
        let vertices = corners.map { p -> SCNVector3 in
            let x = Float(p.x * viewportSize.width), y = Float(p.y * viewportSize.height)
            let near = camera.convertPosition(renderer.unprojectPoint(SCNVector3(x, y, 0)), from: nil)
            let far = camera.convertPosition(renderer.unprojectPoint(SCNVector3(x, y, 1)), from: nil)
            let s = (-distance - near.z) / (far.z - near.z)
            return SCNVector3(near.x + s * (far.x - near.x), near.y + s * (far.y - near.y), -distance)
        }
        // SceneKit's image texture coordinates, like UIKit's view coordinates, start at the top left.
        let inverse = displayTransform.inverted()
        let uv = corners.map { $0.applying(inverse) }
        geometry = SCNGeometry(
            sources: [
                SCNGeometrySource(vertices: vertices),
                SCNGeometrySource(textureCoordinates: uv),
            ], elements: [triangles])
        geometry?.materials = [material]
    }
}

extension CGImage {
    /// Premultiplied RGBA8 bytes → image (the bytes are copied).
    static func rgba8(width: Int, height: Int, bytes: [UInt8]) -> CGImage? {
        guard bytes.count == 4 * width * height,
            let provider = CGDataProvider(data: Data(bytes) as CFData)
        else { return nil }
        return CGImage(
            width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: 4 * width,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(
                rawValue: CGImageAlphaInfo.premultipliedLast.rawValue | CGBitmapInfo.byteOrder32Big.rawValue),
            provider: provider, decode: nil, shouldInterpolate: true, intent: .defaultIntent)
    }
}
