/// Scalar camera measurement fitted against the rotation angle. Tracking always uses luma.
public enum HarmonicSignal: String, CaseIterable, Sendable {
    case luma
    case chromaRed
    case chromaBlue
    case depth

    public init(savedValue: String?) {
        self = savedValue.flatMap(Self.init(rawValue:)) ?? .luma
    }

    public var label: String {
        switch self {
        case .luma: return "Luma / brightness"
        case .chromaRed: return "Red chroma (Cr)"
        case .chromaBlue: return "Blue chroma (Cb)"
        case .depth: return "LiDAR depth"
        }
    }

    public var shortLabel: String {
        switch self {
        case .luma: return "Y"
        case .chromaRed: return "Cr"
        case .chromaBlue: return "Cb"
        case .depth: return "Depth"
        }
    }

    /// Signal magnitude for a fully opaque overlay: 20/255 for color, 2 cm for metric depth.
    public var fullScale: Float { self == .depth ? 0.02 : 20 / 255 }

    public func image(in frame: FrameInput) -> GrayImage? {
        switch self {
        case .luma: return frame.image
        case .chromaRed: return frame.chromaRed
        case .chromaBlue: return frame.chromaBlue
        case .depth:
            guard let map = frame.depth else { return nil }
            let pixels = map.depth.enumerated().map { i, value in
                value.isFinite && value > 0.05 && (map.confidence?[i] ?? 1) >= 1 ? value : Float.nan
            }
            return GrayImage(width: map.width, height: map.height, pixels: pixels)
        }
    }
}
