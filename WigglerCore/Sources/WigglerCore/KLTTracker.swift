import Foundation

/// Result of tracking one point between two frames.
public struct KLTResult {
    public var x: Float
    public var y: Float
    public var ok: Bool
    /// Mean absolute intensity difference between the template and the tracked patch (level 0).
    public var residual: Float
}

/// Pyramidal Lucas–Kanade sparse tracker (translation model, inverse-free iterative form).
public struct KLTTracker {
    public var halfWindow: Int = 4
    public var maxIterations: Int = 12
    public var epsilon: Float = 0.03
    /// Minimum (mean) eigenvalue of the structure tensor to accept a patch as trackable.
    public var minEigenvalue: Float = 1e-4

    public init() {}

    public func track(prev: Pyramid, cur: Pyramid, x: Float, y: Float) -> KLTResult {
        let levelCount = min(prev.count, cur.count)
        let half = halfWindow
        let side = 2 * half + 1
        let n = side * side
        let topScale = Float(1 << (levelCount - 1))
        var px = x / topScale, py = y / topScale  // position in current level (prev image)
        var gx: Float = 0, gy: Float = 0  // displacement estimate in current level

        var template = [Float](repeating: 0, count: n)
        var ix = [Float](repeating: 0, count: n)
        var iy = [Float](repeating: 0, count: n)

        for lvl in stride(from: levelCount - 1, through: 0, by: -1) {
            let P = prev.levels[lvl]
            let C = cur.levels[lvl]
            // The template must fit inside the previous image.
            let margin = Float(half + 2)
            if px < margin || py < margin || px > Float(P.width - 1) - margin || py > Float(P.height - 1) - margin {
                if lvl == 0 { return KLTResult(x: x, y: y, ok: false, residual: .infinity) }
                // Too close to the border at this coarse level: skip it and refine at the next finer level.
                px *= 2
                py *= 2
                gx *= 2
                gy *= 2
                continue
            }
            var gxx: Float = 0, gxy: Float = 0, gyy: Float = 0
            var i = 0
            for dy in -half...half {
                for dx in -half...half {
                    let sx = px + Float(dx), sy = py + Float(dy)
                    template[i] = P.bilinear(sx, sy)
                    let gxv = 0.5 * (P.bilinear(sx + 1, sy) - P.bilinear(sx - 1, sy))
                    let gyv = 0.5 * (P.bilinear(sx, sy + 1) - P.bilinear(sx, sy - 1))
                    ix[i] = gxv
                    iy[i] = gyv
                    gxx += gxv * gxv
                    gxy += gxv * gyv
                    gyy += gyv * gyv
                    i += 1
                }
            }
            let det = gxx * gyy - gxy * gxy
            let tr = gxx + gyy
            let eigMin = 0.5 * (tr - max(tr * tr - 4 * det, 0).squareRoot())
            if eigMin / Float(n) < minEigenvalue || det < 1e-12 {
                return KLTResult(x: x, y: y, ok: false, residual: .infinity)
            }
            let invDet = 1 / det
            for _ in 0..<maxIterations {
                var bx: Float = 0, by: Float = 0
                var k = 0
                let cx = px + gx, cy = py + gy
                if cx < 1 || cy < 1 || cx > Float(C.width - 2) || cy > Float(C.height - 2) {
                    return KLTResult(x: x, y: y, ok: false, residual: .infinity)
                }
                for dy in -half...half {
                    for dx in -half...half {
                        let d = C.bilinear(cx + Float(dx), cy + Float(dy)) - template[k]
                        bx += ix[k] * d
                        by += iy[k] * d
                        k += 1
                    }
                }
                let ux = -(gyy * bx - gxy * by) * invDet
                let uy = -(-gxy * bx + gxx * by) * invDet
                gx += ux
                gy += uy
                if ux * ux + uy * uy < epsilon * epsilon { break }
            }
            if lvl > 0 {
                px *= 2
                py *= 2
                gx *= 2
                gy *= 2
            }
        }
        let nx = px + gx, ny = py + gy
        let C0 = cur.levels[0], P0 = prev.levels[0]
        let m = Float(half + 1)
        if nx < m || ny < m || nx > Float(C0.width - 1) - m || ny > Float(C0.height - 1) - m {
            return KLTResult(x: nx, y: ny, ok: false, residual: .infinity)
        }
        var res: Float = 0
        for dy in -half...half {
            for dx in -half...half {
                res += abs(C0.bilinear(nx + Float(dx), ny + Float(dy)) - P0.bilinear(x + Float(dx), y + Float(dy)))
            }
        }
        return KLTResult(x: nx, y: ny, ok: true, residual: res / Float(n))
    }
}
