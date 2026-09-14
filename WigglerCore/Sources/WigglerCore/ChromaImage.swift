/// Byte position in NV12's interleaved Cb,Cr plane.
public enum ChromaComponent: Int {
    case blue = 0
    case red = 1
}

extension GrayImage {
    /// Decode one signed chroma component on its native grid, with neutral at byte 128.
    public init(
        width: Int, height: Int, cbcr8: UnsafePointer<UInt8>, bytesPerRow: Int,
        component: ChromaComponent, videoRange: Bool
    ) {
        precondition(width > 0 && height > 0 && bytesPerRow >= 2 * width)
        var pixels = [Float](repeating: 0, count: width * height)
        let scale: Float = 1 / (videoRange ? 224 : 255)
        for y in 0..<height {
            for x in 0..<width {
                let source = y * bytesPerRow + 2 * x + component.rawValue
                pixels[y * width + x] = (Float(cbcr8[source]) - 128) * scale
            }
        }
        self.init(width: width, height: height, pixels: pixels)
    }
}
