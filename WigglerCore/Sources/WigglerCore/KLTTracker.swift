import Foundation

/// Result of tracking one point between two frames.
public struct KLTResult {
    public var x: Float
    public var y: Float
    public var ok: Bool
    /// Mean absolute intensity difference under the fitted level-zero patch warp.
    public var residual: Float
}

/// Pyramidal inverse-compositional Lucas–Kanade tracker with an affine patch warp.
public struct KLTTracker {
    public var halfWindow: Int = 4
    public var maxIterations: Int = 12
    public var epsilon: Float = 0.03
    /// Minimum mean translation-tensor eigenvalue for a trackable patch.
    public var minEigenvalue: Float = 1e-4
    public init() {}

    /// Maximum forward/backward return distance in level-zero pixels.
    public var maxCycleError: Float = 1

    public func track(prev: Pyramid, cur: Pyramid, x: Float, y: Float) -> KLTResult {
        precondition(maxCycleError >= 0)
        var forward = trackDirection(prev: prev, cur: cur, x: x, y: y)
        guard forward.ok else { return forward }
        let backward = trackDirection(prev: cur, cur: prev, x: forward.x, y: forward.y)
        let dx = backward.x - x, dy = backward.y - y
        forward.ok = backward.ok && dx * dx + dy * dy <= maxCycleError * maxCycleError
        return forward
    }

    private struct AffineWarp {
        var gx = 0.0, gy = 0.0
        var a00 = 1.0, a01 = 0.0, a10 = 0.0, a11 = 1.0
    }

    private func trackDirection(prev: Pyramid, cur: Pyramid, x: Float, y: Float) -> KLTResult {
        let levelCount = min(prev.count, cur.count)
        let half = halfWindow, n = (2 * half + 1) * (2 * half + 1)
        let failure = KLTResult(x: x, y: y, ok: false, residual: .infinity)
        guard x.isFinite, y.isFinite, half > 0 else { return failure }
        let topScale = Double(1 << (levelCount - 1))
        var px = Double(x) / topScale, py = Double(y) / topScale
        var warp = AffineWarp()
        var template = [Double](repeating: 0, count: n)
        var jacobian = [Double](repeating: 0, count: n * 6)
        for level in stride(from: levelCount - 1, through: 0, by: -1) {
            if let refined = refinePatch(
                prev: prev.levels[level], cur: cur.levels[level], x: px, y: py,
                warp: warp, template: &template, jacobian: &jacobian)
            {
                warp = refined
            } else if level == 0 {
                return failure
            }
            if level > 0 {
                px *= 2
                py *= 2
                warp.gx *= 2
                warp.gy *= 2
            }
        }
        let gx = warp.gx, gy = warp.gy
        let a00 = warp.a00, a01 = warp.a01, a10 = warp.a10, a11 = warp.a11
        let nx = px + gx, ny = py + gy, c = cur.levels[0], p = prev.levels[0]
        let mx = (abs(a00) + abs(a01)) * Double(half) + 1
        let my = (abs(a10) + abs(a11)) * Double(half) + 1
        guard nx.isFinite, ny.isFinite, mx.isFinite, my.isFinite, nx >= mx, ny >= my, nx <= Double(c.width - 1) - mx,
            ny <= Double(c.height - 1) - my
        else { return failure }
        var residual = 0.0
        for dy in -half...half {
            for dx in -half...half {
                residual += abs(
                    Double(
                        c.bilinear(
                            Float(nx + a00 * Double(dx) + a01 * Double(dy)),
                            Float(ny + a10 * Double(dx) + a11 * Double(dy))))
                        - Double(p.bilinear(x + Float(dx), y + Float(dy))))
            }
        }
        return KLTResult(x: Float(nx), y: Float(ny), ok: true, residual: Float(residual / Double(n)))
    }

    /// Fit one scale without changing the input warp when its patch cannot be measured.
    private func refinePatch(
        prev p: GrayImage, cur c: GrayImage, x px: Double, y py: Double,
        warp: AffineWarp, template: inout [Double], jacobian: inout [Double]
    ) -> AffineWarp? {
        let half = halfWindow
        let margin = Double(half + 2)
        guard px >= margin, py >= margin, px <= Double(p.width - 1) - margin,
            py <= Double(p.height - 1) - margin
        else { return nil }
        // The hot loops run on raw buffers: bounds checks on five arrays per sample cost more than the arithmetic.
        return p.pixels.withUnsafeBufferPointer { prevPixels in
            c.pixels.withUnsafeBufferPointer { curPixels in
                template.withUnsafeMutableBufferPointer { template in
                    jacobian.withUnsafeMutableBufferPointer { jacobian in
                        refinePatch(
                            prev: Sampler(pixels: prevPixels.baseAddress!, width: p.width, height: p.height),
                            cur: Sampler(pixels: curPixels.baseAddress!, width: c.width, height: c.height),
                            x: px, y: py, warp: warp, template: template.baseAddress!, jacobian: jacobian.baseAddress!)
                    }
                }
            }
        }
    }

    /// `GrayImage.bilinear` on a raw buffer, same arithmetic.
    private struct Sampler {
        let pixels: UnsafePointer<Float>
        let width: Int, height: Int
        @inline(__always) func bilinear(_ x: Float, _ y: Float) -> Float {
            let cx = min(max(x, 0), Float(width - 1) - 0.001)
            let cy = min(max(y, 0), Float(height - 1) - 0.001)
            let x0 = Int(cx), y0 = Int(cy)
            let fx = cx - Float(x0), fy = cy - Float(y0)
            let x1 = min(x0 + 1, width - 1), y1 = min(y0 + 1, height - 1)
            let a = pixels[y0 * width + x0], b = pixels[y0 * width + x1]
            let c = pixels[y1 * width + x0], d = pixels[y1 * width + x1]
            return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy
        }
    }

    private func refinePatch(
        prev p: Sampler, cur c: Sampler, x px: Double, y py: Double, warp: AffineWarp,
        template: UnsafeMutablePointer<Double>, jacobian: UnsafeMutablePointer<Double>
    ) -> AffineWarp? {
        let half = halfWindow, n = (2 * half + 1) * (2 * half + 1)
        var gx = warp.gx, gy = warp.gy
        var a00 = warp.a00, a01 = warp.a01, a10 = warp.a10, a11 = warp.a11
        // Scratch: the 6×6 normal matrix, its Cholesky factor and the update vector.
        var scratch = [Double](repeating: 0, count: 36 + 36 + 6)
        return scratch.withUnsafeMutableBufferPointer { scratch -> AffineWarp? in
            let h = scratch.baseAddress!, l = h + 36, b = h + 72
            var i = 0
            for dy in -half...half {
                for dx in -half...half {
                    let sx = Float(px + Double(dx)), sy = Float(py + Double(dy))
                    template[i] = Double(p.bilinear(sx, sy))
                    let ix = 0.5 * Double(p.bilinear(sx + 1, sy) - p.bilinear(sx - 1, sy))
                    let iy = 0.5 * Double(p.bilinear(sx, sy + 1) - p.bilinear(sx, sy - 1))
                    let u = Double(dx) / Double(half), v = Double(dy) / Double(half)
                    let base = i * 6
                    jacobian[base] = ix
                    jacobian[base + 1] = iy
                    jacobian[base + 2] = ix * u
                    jacobian[base + 3] = ix * v
                    jacobian[base + 4] = iy * u
                    jacobian[base + 5] = iy * v
                    for row in 0..<6 {
                        for col in 0...row { h[row * 6 + col] += jacobian[base + row] * jacobian[base + col] }
                    }
                    i += 1
                }
            }
            let tr = h[0] + h[7], det = h[0] * h[7] - h[6] * h[6]
            let eig = 0.5 * (tr - sqrt(max(0, tr * tr - 4 * det)))
            guard eig / Double(n) >= Double(minEigenvalue), det >= 1e-12 else { return nil }
            var pivotFloor = 0.0
            for k in 0..<6 { pivotFloor = max(pivotFloor, h[k * 6 + k]) }
            pivotFloor *= 1e-10
            for row in 0..<6 {
                for col in 0...row {
                    var value = h[row * 6 + col]
                    for k in 0..<col { value -= l[row * 6 + k] * l[col * 6 + k] }
                    if row == col {
                        guard value.isFinite, value > pivotFloor else { return nil }
                        l[row * 6 + col] = sqrt(value)
                    } else {
                        l[row * 6 + col] = value / l[col * 6 + col]
                    }
                }
            }
            for _ in 0..<maxIterations {
                let cx = px + gx, cy = py + gy
                let mx = (abs(a00) + abs(a01)) * Double(half) + 1
                let my = (abs(a10) + abs(a11)) * Double(half) + 1
                guard cx.isFinite, cy.isFinite, mx.isFinite, my.isFinite,
                    cx >= mx, cy >= my, cx <= Double(c.width - 1) - mx, cy <= Double(c.height - 1) - my
                else { return nil }
                for row in 0..<6 { b[row] = 0 }
                i = 0
                for dy in -half...half {
                    for dx in -half...half {
                        let sx = cx + a00 * Double(dx) + a01 * Double(dy)
                        let sy = cy + a10 * Double(dx) + a11 * Double(dy)
                        let d = Double(c.bilinear(Float(sx), Float(sy))) - template[i]
                        let base = i * 6
                        for row in 0..<6 { b[row] += jacobian[base + row] * d }
                        i += 1
                    }
                }
                for row in 0..<6 {
                    for col in 0..<row { b[row] -= l[row * 6 + col] * b[col] }
                    b[row] /= l[row * 6 + row]
                }
                for row in stride(from: 5, through: 0, by: -1) {
                    for col in (row + 1)..<6 { b[row] -= l[col * 6 + row] * b[col] }
                    b[row] /= l[row * 6 + row]
                }
                for parameter in 2..<6 { b[parameter] /= Double(half) }
                let determinant = (1 + b[2]) * (1 + b[5]) - b[3] * b[4]
                guard determinant.isFinite, abs(determinant) > 1e-9 else { return nil }
                let n00 = (a00 * (1 + b[5]) - a01 * b[4]) / determinant
                let n01 = (-a00 * b[3] + a01 * (1 + b[2])) / determinant
                let n10 = (a10 * (1 + b[5]) - a11 * b[4]) / determinant
                let n11 = (-a10 * b[3] + a11 * (1 + b[2])) / determinant
                let ux = n00 * b[0] + n01 * b[1], uy = n10 * b[0] + n11 * b[1]
                gx -= ux
                gy -= uy
                let shapeStep =
                    max(abs(n00 - a00) + abs(n01 - a01), abs(n10 - a10) + abs(n11 - a11)) * Double(half)
                a00 = n00
                a01 = n01
                a10 = n10
                a11 = n11
                if hypot(ux, uy) + shapeStep < Double(epsilon) { break }
            }
            return AffineWarp(gx: gx, gy: gy, a00: a00, a01: a01, a10: a10, a11: a11)
        }
    }
}
