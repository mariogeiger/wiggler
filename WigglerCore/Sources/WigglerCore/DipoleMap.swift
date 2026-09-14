import Foundation

/// Per-pixel lock-in detection of the l = 1 (dipole) component of the luma as a function of the rotation angle.
///
/// Every pixel's luma is fitted by weighted least squares on the basis {1, θ−θ_now, cos θ, sin θ} over the recent
/// history. The trend term absorbs exposure and lighting ramps, which would otherwise leak into l = 1. The window
/// is an exponential in *angle* — a frame weighs |Δθ| and the past decays as e^{−|Δθ|/Θ} — so a stopped object
/// neither adds to nor forgets its map, and the coefficient is a genuine Fourier coefficient however irregular the
/// rotation. The normal matrix depends only on the angle samples and is shared by all pixels; each pixel keeps its
/// four right-hand sides, so a frame costs a few multiply-adds per pixel and no frame is ever stored.
///
/// For a pixel on the object, I(θ) = L(φ − θ) gives c₁ = L̂₁ e^{iφ}: the phase is the pixel's azimuth around the
/// axis (the axis is the vortex of the phase field) and the magnitude is the dipole of the texture at that height.
public struct DipoleMap {
    public let width: Int
    public let height: Int
    /// e-folding length of the window, radians.
    public let window: Double

    // Shared moments Σ w φφᵀ, φ = (1, τ, cos θ, sin θ), τ = θ − θ_now.
    private var s0 = 0.0, sT = 0.0, sTT = 0.0, sC = 0.0, sS = 0.0, sTC = 0.0, sTS = 0.0, sCC = 0.0, sCS = 0.0, sSS = 0.0
    /// Per-pixel right-hand sides Σ w I φ, four planes of width × height: [Σ w I | Σ w I τ | Σ w I cos θ | Σ w I sin θ].
    private var acc: [Float]
    private var thetaPrev: Double?

    public init(width: Int, height: Int, windowTurns: Double = 3) {
        self.width = width
        self.height = height
        self.window = windowTurns * 2 * .pi
        self.acc = [Float](repeating: 0, count: 4 * width * height)
    }

    public mutating func reset() {
        s0 = 0; sT = 0; sTT = 0; sC = 0; sS = 0; sTC = 0; sTS = 0; sCC = 0; sCS = 0; sSS = 0
        for i in 0..<acc.count { acc[i] = 0 }
        thetaPrev = nil
    }

    /// Total weight in the window as a fraction of its e-folding length (→ 1 after many turns).
    public var fill: Double { s0 / window }
    /// Progress towards one full turn of accumulated angle, 0…1. Below 1 the coefficient would be extrapolated
    /// from an arc, not measured; above, the speed no longer matters.
    public var turnProgress: Double { min(1, s0 / (window * (1 - exp(-2 * .pi / window)))) }
    public var hasFullTurn: Bool { turnProgress >= 1 }

    /// Add a frame observed at angle θ. A jump of more than a quarter turn means the angle reference changed:
    /// the map is reset and `false` returned.
    @discardableResult
    public mutating func add(image: GrayImage, theta: Double) -> Bool {
        precondition(image.width == width && image.height == height)
        guard let prev = thetaPrev else { thetaPrev = theta; return true }
        let dth = theta - prev
        thetaPrev = theta
        if abs(dth) > .pi / 2 {
            reset()
            thetaPrev = theta
            return false
        }
        let w = abs(dth)
        let lam = exp(-w / window)
        let c = cos(theta), s = sin(theta)
        // Shift τ of every past sample by −Δθ (old values on the right), then decay and add the new one (τ = 0).
        (sT, sTT, sTC, sTS) = (lam * (sT - dth * s0), lam * (sTT - 2 * dth * sT + dth * dth * s0),
                               lam * (sTC - dth * sC), lam * (sTS - dth * sS))
        s0 = lam * s0 + w
        sC = lam * sC + w * c; sS = lam * sS + w * s
        sCC = lam * sCC + w * c * c; sCS = lam * sCS + w * c * s; sSS = lam * sSS + w * s * s

        let n = width * height
        let lamF = Float(lam), dthF = Float(dth), wF = Float(w), wc = Float(w * c), ws = Float(w * s)
        image.pixels.withUnsafeBufferPointer { img in
            acc.withUnsafeMutableBufferPointer { a in
                let r0 = a.baseAddress!, rT = r0 + n, rC = r0 + 2 * n, rS = r0 + 3 * n
                for i in 0..<n {
                    let v = img[i]
                    rT[i] = lamF * (rT[i] - dthF * r0[i])
                    r0[i] = lamF * r0[i] + wF * v
                    rC[i] = lamF * rC[i] + wc * v
                    rS[i] = lamF * rS[i] + ws * v
                }
            }
        }
        return true
    }

    /// Rows of the inverse normal matrix that give the cos θ and sin θ coefficients; nil while singular.
    private func solveRows() -> (a: [Double], b: [Double])? {
        var m: [[Double]] = [[s0, sT, sC, sS, 1, 0, 0, 0],
                             [sT, sTT, sTC, sTS, 0, 1, 0, 0],
                             [sC, sTC, sCC, sCS, 0, 0, 1, 0],
                             [sS, sTS, sCS, sSS, 0, 0, 0, 1]]
        let tiny = 1e-12 * (s0 + sTT + sCC + sSS)
        for col in 0..<4 {
            var pivot = col
            for r in col + 1..<4 where abs(m[r][col]) > abs(m[pivot][col]) { pivot = r }
            if abs(m[pivot][col]) <= tiny { return nil }
            m.swapAt(col, pivot)
            let inv = 1 / m[col][col]
            for j in 0..<8 { m[col][j] *= inv }
            for r in 0..<4 where r != col {
                let f = m[r][col]
                if f != 0 { for j in 0..<8 { m[r][j] -= f * m[col][j] } }
            }
        }
        return (Array(m[2][4..<8]), Array(m[3][4..<8]))
    }

    /// The dipole (a, b) — coefficients of cos θ and sin θ, luma units — at every pixel; nil while undetermined.
    public func solve() -> (a: [Float], b: [Float])? {
        guard hasFullTurn, let rows = solveRows() else { return nil }
        let n = width * height
        var a = [Float](repeating: 0, count: n), b = [Float](repeating: 0, count: n)
        let ga = rows.a.map(Float.init), gb = rows.b.map(Float.init)
        acc.withUnsafeBufferPointer { r in
            let r0 = r.baseAddress!, rT = r0 + n, rC = r0 + 2 * n, rS = r0 + 3 * n
            for i in 0..<n {
                a[i] = ga[0] * r0[i] + ga[1] * rT[i] + ga[2] * rC[i] + ga[3] * rS[i]
                b[i] = gb[0] * r0[i] + gb[1] * rT[i] + gb[2] * rC[i] + gb[3] * rS[i]
            }
        }
        return (a, b)
    }

    /// The real quantity shown for the complex c₁ = a − i b.
    public enum Display: Int, CaseIterable {
        /// a cos θ + b sin θ: the l = 1 part of the current image, turning with the object. Blue → 0 → red.
        case current
        /// a: the l = 1 image at θ = 0, a frozen two-lobe pattern whose zero line passes through the axis. Blue → 0 → red.
        case fixed
        /// |c₁|: transparent → red.
        case magnitude
        /// arg c₁ as a hue, opacity by |c₁|. The pixel's azimuth around the axis.
        case phase
    }

    /// Premultiplied RGBA8 overlay, width × height, fully opaque at |value| = `fullScale` (luma units);
    /// nil while the map is undetermined.
    public func render(_ display: Display, theta: Double, fullScale: Float) -> [UInt8]? {
        guard let sol = solve() else { return nil }
        let a = sol.a, b = sol.b
        let n = width * height
        var out = [UInt8](repeating: 0, count: 4 * n)
        let inv = 1 / fullScale
        let ct = Float(cos(theta)), st = Float(sin(theta))
        @inline(__always) func byte(_ x: Float) -> UInt8 { UInt8(min(255, max(0, x * 255 + 0.5))) }
        out.withUnsafeMutableBufferPointer { o in
            for i in 0..<n {
                let p = 4 * i
                switch display {
                case .current, .fixed:
                    let v = (display == .current ? a[i] * ct + b[i] * st : a[i]) * inv
                    let alpha = min(1, abs(v))
                    let k = byte(alpha)
                    if v > 0 { o[p] = k } else { o[p + 2] = k }
                    o[p + 3] = k
                case .magnitude:
                    let k = byte(min(1, (a[i] * a[i] + b[i] * b[i]).squareRoot() * inv))
                    o[p] = k; o[p + 3] = k
                case .phase:
                    let m = (a[i] * a[i] + b[i] * b[i]).squareRoot()
                    let alpha = min(1, m * inv)
                    if m > 0 {
                        // Hue from (cos φ, sin φ) = (a, b)/|c₁| directly: the three phases 0, ±2π/3 of a cosine.
                        let cp = a[i] / m, sp = b[i] / m
                        let h: Float = 0.8660254
                        o[p] = byte(alpha * (0.5 + 0.5 * cp))
                        o[p + 1] = byte(alpha * (0.5 + 0.5 * (-0.5 * cp + h * sp)))
                        o[p + 2] = byte(alpha * (0.5 + 0.5 * (-0.5 * cp - h * sp)))
                    }
                    o[p + 3] = byte(alpha)
                }
            }
        }
        return out
    }
}
