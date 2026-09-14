import Foundation

/// Shi–Tomasi ("good features to track") corner detector restricted to a disk region.
public struct CornerDetector {
    /// Radius of the structure-tensor summation window.
    public var windowRadius: Int = 2
    /// Minimum distance between returned corners (and to `exclude` points), in pixels.
    public var minDistance: Float = 8
    public var border: Int = 8

    public init() {}

    /// - Parameters:
    ///   - image: level-0 image.
    ///   - center, radius: disk of interest (pixels).
    ///   - exclude: existing points to keep away from.
    ///   - maxCount: maximum number of corners returned.
    public func detect(
        in image: GrayImage, centerX: Float, centerY: Float, radius: Float,
        exclude: [(Float, Float)], maxCount: Int
    ) -> [(Float, Float)] {
        let w = image.width, h = image.height
        let x0 = max(border, Int(centerX - radius) - 1)
        let x1 = min(w - border - 1, Int(centerX + radius) + 1)
        let y0 = max(border, Int(centerY - radius) - 1)
        let y1 = min(h - border - 1, Int(centerY + radius) + 1)
        if x1 <= x0 || y1 <= y0 { return [] }
        let rw = x1 - x0 + 1, rh = y1 - y0 + 1
        let r = windowRadius
        let r2 = radius * radius

        // Gradients over the region (with a margin of `r` + 1).
        let gx0 = max(1, x0 - r), gy0 = max(1, y0 - r)
        let gx1 = min(w - 2, x1 + r), gy1 = min(h - 2, y1 + r)
        let gw = gx1 - gx0 + 1, gh = gy1 - gy0 + 1
        var ix = [Float](repeating: 0, count: gw * gh)
        var iy = [Float](repeating: 0, count: gw * gh)
        image.pixels.withUnsafeBufferPointer { p in
            for y in gy0...gy1 {
                for x in gx0...gx1 {
                    let i = (y - gy0) * gw + (x - gx0)
                    ix[i] = 0.5 * (p[y * w + x + 1] - p[y * w + x - 1])
                    iy[i] = 0.5 * (p[(y + 1) * w + x] - p[(y - 1) * w + x])
                }
            }
        }
        var score = [Float](repeating: 0, count: rw * rh)
        for y in y0...y1 {
            let dy0 = Float(y) - centerY
            for x in x0...x1 {
                let dx0 = Float(x) - centerX
                if dx0 * dx0 + dy0 * dy0 > r2 { continue }
                var gxx: Float = 0, gxy: Float = 0, gyy: Float = 0
                for dy in -r...r {
                    let yy = y + dy
                    if yy < gy0 || yy > gy1 { continue }
                    for dx in -r...r {
                        let xx = x + dx
                        if xx < gx0 || xx > gx1 { continue }
                        let i = (yy - gy0) * gw + (xx - gx0)
                        let a = ix[i], b = iy[i]
                        gxx += a * a
                        gxy += a * b
                        gyy += b * b
                    }
                }
                let tr = gxx + gyy, det = gxx * gyy - gxy * gxy
                score[(y - y0) * rw + (x - x0)] = 0.5 * (tr - max(tr * tr - 4 * det, 0).squareRoot())
            }
        }
        // Non-maximum suppression (3x3) + threshold relative to the best score.
        var best: Float = 0
        for s in score where s > best { best = s }
        if best <= 0 { return [] }
        let threshold = best * 0.01
        var cands: [(Float, Int, Int)] = []
        for y in 1..<(rh - 1) {
            for x in 1..<(rw - 1) {
                let s = score[y * rw + x]
                if s < threshold { continue }
                var isMax = true
                for dy in -1...1 where isMax {
                    for dx in -1...1 where !(dx == 0 && dy == 0) {
                        if score[(y + dy) * rw + (x + dx)] > s {
                            isMax = false
                            break
                        }
                    }
                }
                if isMax { cands.append((s, x + x0, y + y0)) }
            }
        }
        cands.sort { $0.0 > $1.0 }
        // Greedy min-distance selection using a coarse grid.
        let cell = max(minDistance, 1)
        var occupied: [Int: [(Float, Float)]] = [:]
        func key(_ x: Float, _ y: Float) -> Int { Int(x / cell) &* 73856093 ^ Int(y / cell) &* 19349663 }
        func farEnough(_ x: Float, _ y: Float) -> Bool {
            let cx = Int(x / cell), cy = Int(y / cell)
            for j in -1...1 {
                for i in -1...1 {
                    let k = (cx + i) &* 73856093 ^ (cy + j) &* 19349663
                    if let pts = occupied[k] {
                        for (ox, oy) in pts where (ox - x) * (ox - x) + (oy - y) * (oy - y) < minDistance * minDistance
                        {
                            return false
                        }
                    }
                }
            }
            return true
        }
        func insert(_ x: Float, _ y: Float) { occupied[key(x, y), default: []].append((x, y)) }
        for (ex, ey) in exclude { insert(ex, ey) }
        var out: [(Float, Float)] = []
        for (_, x, y) in cands {
            let fx = Float(x), fy = Float(y)
            if farEnough(fx, fy) {
                out.append((fx, fy))
                insert(fx, fy)
                if out.count >= maxCount { break }
            }
        }
        return out
    }
}
