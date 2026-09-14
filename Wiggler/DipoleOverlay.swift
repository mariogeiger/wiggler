import SwiftUI
import UIKit
import WigglerCore

/// Draws an engine-sized RGBA image over the camera view, through the same display transform ARKit uses for the
/// camera image, so every overlay pixel sits on the camera pixel it was computed from.
final class ImageOverlayUIView: UIView {
    private let imageLayer = CALayer()
    private var image: CGImage?
    private var transform2D = CGAffineTransform.identity

    override init(frame: CGRect) {
        super.init(frame: frame)
        isUserInteractionEnabled = false
        imageLayer.anchorPoint = .zero
        imageLayer.position = .zero
        imageLayer.magnificationFilter = .linear
        layer.addSublayer(imageLayer)
    }

    required init?(coder: NSCoder) { fatalError() }

    /// `transform` maps normalised image coordinates to normalised view coordinates.
    func set(image: CGImage?, transform: CGAffineTransform) {
        self.image = image
        transform2D = transform
        apply()
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        apply()
    }

    private func apply() {
        CATransaction.begin()
        CATransaction.setDisableActions(true)
        imageLayer.contents = image
        if let img = image {
            let w = CGFloat(img.width), h = CGFloat(img.height)
            imageLayer.bounds = CGRect(x: 0, y: 0, width: w, height: h)
            // layer pixels → normalised image → normalised view → view points
            let m = CGAffineTransform(scaleX: 1 / w, y: 1 / h)
                .concatenating(transform2D)
                .concatenating(CGAffineTransform(scaleX: bounds.width, y: bounds.height))
            imageLayer.setAffineTransform(m)
        }
        CATransaction.commit()
    }
}

struct DipoleOverlay: UIViewRepresentable {
    let controller: ARSessionController
    let image: CGImage?

    func makeUIView(context: Context) -> ImageOverlayUIView { ImageOverlayUIView(frame: .zero) }

    func updateUIView(_ view: ImageOverlayUIView, context: Context) {
        view.set(image: image, transform: controller.displayTransform())
    }
}

extension CGImage {
    /// Premultiplied RGBA8 bytes → image (the bytes are copied).
    static func rgba8(width: Int, height: Int, bytes: [UInt8]) -> CGImage? {
        guard bytes.count == 4 * width * height,
              let provider = CGDataProvider(data: Data(bytes) as CFData) else { return nil }
        return CGImage(width: width, height: height, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: 4 * width,
                       space: CGColorSpaceCreateDeviceRGB(),
                       bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue | CGBitmapInfo.byteOrder32Big.rawValue),
                       provider: provider, decode: nil, shouldInterpolate: true, intent: .defaultIntent)
    }
}

extension DipoleMap.Display {
    var label: String {
        switch self {
        case .current: return "now"
        case .fixed: return "θ=0"
        case .magnitude: return "|c₁|"
        case .phase: return "∠c₁"
        }
    }
}
