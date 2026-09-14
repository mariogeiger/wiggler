import Foundation

/// Row-major scalar image. Tracking uses luma in [0, 1]; harmonic inputs may be signed, metric or NaN (missing).
public struct GrayImage {
    public let width: Int
    public let height: Int
    public var pixels: [Float]

    public init(width: Int, height: Int, pixels: [Float]) {
        precondition(pixels.count == width * height)
        self.width = width
        self.height = height
        self.pixels = pixels
    }

    public init(width: Int, height: Int, fill: Float = 0) {
        self.width = width
        self.height = height
        self.pixels = [Float](repeating: fill, count: width * height)
    }

    /// Build from 8-bit luma with a row stride (bytes per row may exceed width).
    public init(width: Int, height: Int, luma8: UnsafePointer<UInt8>, bytesPerRow: Int) {
        self.width = width
        self.height = height
        var px = [Float](repeating: 0, count: width * height)
        px.withUnsafeMutableBufferPointer { dst in
            let scale: Float = 1.0 / 255.0
            for y in 0..<height {
                let src = luma8 + y * bytesPerRow
                let row = y * width
                for x in 0..<width {
                    dst[row + x] = Float(src[x]) * scale
                }
            }
        }
        self.pixels = px
    }

    @inline(__always) public func at(_ x: Int, _ y: Int) -> Float { pixels[y * width + x] }

    /// Half-resolution image with a 3x3 binomial ([1 2 1]⊗[1 2 1]) anti-alias filter centered on the even pixels.
    public func downsampled() -> GrayImage {
        let w2 = width / 2, h2 = height / 2
        var out = [Float](repeating: 0, count: w2 * h2)
        let w = width, h = height
        pixels.withUnsafeBufferPointer { src in
            out.withUnsafeMutableBufferPointer { dst in
                for y in 0..<h2 {
                    let sy = 2 * y
                    let ym = max(sy - 1, 0), yp = min(sy + 1, h - 1)
                    for x in 0..<w2 {
                        let sx = 2 * x
                        let xm = max(sx - 1, 0), xp = min(sx + 1, w - 1)
                        let r0 = src[ym * w + xm] + 2 * src[ym * w + sx] + src[ym * w + xp]
                        let r1 = src[sy * w + xm] + 2 * src[sy * w + sx] + src[sy * w + xp]
                        let r2 = src[yp * w + xm] + 2 * src[yp * w + sx] + src[yp * w + xp]
                        dst[y * w2 + x] = (r0 + 2 * r1 + r2) * (1.0 / 16.0)
                    }
                }
            }
        }
        return GrayImage(width: w2, height: h2, pixels: out)
    }

    /// Bilinear sample with clamping to the image domain.
    @inline(__always) public func bilinear(_ x: Float, _ y: Float) -> Float {
        let cx = min(max(x, 0), Float(width - 1) - 0.001)
        let cy = min(max(y, 0), Float(height - 1) - 0.001)
        let x0 = Int(cx), y0 = Int(cy)
        let fx = cx - Float(x0), fy = cy - Float(y0)
        let x1 = min(x0 + 1, width - 1), y1 = min(y0 + 1, height - 1)
        let a = pixels[y0 * width + x0], b = pixels[y0 * width + x1]
        let c = pixels[y1 * width + x0], d = pixels[y1 * width + x1]
        return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy
    }

    /// Area-average resample of an axis-aligned square region into a small `side` x `side` patch.
    /// Used for the appearance descriptors. Pixels outside the image count as 0.5 (gray).
    public func patch(centerX: Float, centerY: Float, halfSize: Float, side: Int) -> [Float] {
        var out = [Float](repeating: 0, count: side * side)
        let cell = 2 * halfSize / Float(side)
        let sub = max(1, min(4, Int(cell.rounded(.up))))  // supersampling per cell
        let inv = 1.0 / Float(sub * sub)
        for j in 0..<side {
            for i in 0..<side {
                var s: Float = 0
                for sj in 0..<sub {
                    for si in 0..<sub {
                        let x = centerX - halfSize + (Float(i) + (Float(si) + 0.5) / Float(sub)) * cell
                        let y = centerY - halfSize + (Float(j) + (Float(sj) + 0.5) / Float(sub)) * cell
                        if x < 0 || y < 0 || x > Float(width - 1) || y > Float(height - 1) {
                            s += 0.5
                        } else {
                            s += bilinear(x, y)
                        }
                    }
                }
                out[j * side + i] = s * inv
            }
        }
        return out
    }
}

/// Image pyramid (level 0 = full resolution).
public final class Pyramid {
    public let levels: [GrayImage]
    public init(image: GrayImage, levelCount: Int) {
        var l = [image]
        for _ in 1..<max(levelCount, 1) {
            let last = l[l.count - 1]
            if last.width < 16 || last.height < 16 { break }
            l.append(last.downsampled())
        }
        levels = l
    }
    public var count: Int { levels.count }
}
