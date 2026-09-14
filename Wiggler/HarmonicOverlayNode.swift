import SceneKit
import UIKit

/// Composites a camera-aligned image of any resolution below the world-space axis and ray.
final class HarmonicOverlayNode: SCNNode {
    private let material = SCNMaterial()
    private let triangles = SCNGeometryElement(indices: [UInt16(0), 1, 2, 2, 1, 3], primitiveType: .triangles)
    private let corners = [CGPoint(x: 0, y: 0), CGPoint(x: 1, y: 0), CGPoint(x: 0, y: 1), CGPoint(x: 1, y: 1)]
    private var image: CGImage?

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
        guard let image, renderer.pointOfView != nil, viewportSize.width > 0, viewportSize.height > 0 else {
            isHidden = true
            self.image = nil
            material.diffuse.contents = nil
            return
        }
        isHidden = false
        if self.image !== image {
            self.image = image
            material.diffuse.contents = image
        }
        let vertices = corners.map { p in
            renderer.unprojectPoint(
                SCNVector3(
                    Float(p.x * viewportSize.width),
                    Float(p.y * viewportSize.height), 0.01))
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
