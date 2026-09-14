import Foundation

/// Per-pixel lock-in detection of chosen Fourier harmonics l of the luma as a function of the rotation angle.
///
/// Every pixel's luma is fitted by weighted least squares on the basis {1, θ−θ_now, cos lθ, sin lθ for l in
/// `orders`} over the recent history. The trend term absorbs exposure and lighting ramps, which would otherwise leak
/// into the low harmonics; fitting all orders jointly removes the cross-talk an irregular, windowed sampling puts
/// between them. The window is an exponential in *angle* — a frame weighs |Δθ| and the past decays as e^{−|Δθ|/Θ}
/// — so a stopped object neither adds to nor forgets its map, and the coefficients are genuine Fourier coefficients
/// however irregular the rotation. The normal matrix depends only on the angle samples and is shared by all pixels;
/// each pixel keeps its right-hand sides, one plane per basis term, so a frame costs one multiply-add per term per
/// pixel and no frame is ever stored.
///
/// For a pixel on the object, I(θ) = L(φ − θ) gives c_l = L̂_l e^{ilφ}: the phase is l times the pixel's azimuth
/// around the axis and the magnitude the l-th harmonic of the texture at that height.
public struct HarmonicMap {
    public let width: Int
    public let height: Int
    /// e-folding length of the window, radians.
    public let window: Double
    /// Harmonics fitted, each ≥ 1 and distinct.
    public let orders: [Int]
    /// Basis size: 1, τ, then (cos, sin) per order.
    private let terms: Int

    /// Shared moments N = Σ w φφᵀ, φ = (1, τ, cos l₁θ, sin l₁θ, …), τ = θ − θ_now; row-major terms × terms.
    private var moments: [Double]
    /// Per-pixel right-hand sides Σ w I φ: `terms` planes of width × height, in basis order.
    private var acc: [Float]
    private var thetaPrev: Double?

    public init(width: Int, height: Int, orders: [Int] = [1], windowTurns: Double = 3) {
        precondition(!orders.isEmpty && orders.allSatisfy { $0 >= 1 } && Set(orders).count == orders.count)
        self.width = width
        self.height = height
        self.window = windowTurns * 2 * .pi
        self.orders = orders
        self.terms = 2 + 2 * orders.count
        self.moments = [Double](repeating: 0, count: terms * terms)
        self.acc = [Float](repeating: 0, count: terms * width * height)
    }

    public mutating func reset() {
        for i in 0..<moments.count { moments[i] = 0 }
        for i in 0..<acc.count { acc[i] = 0 }
        thetaPrev = nil
    }

    /// Total weight in the window as a fraction of its e-folding length (→ 1 after many turns).
    public var fill: Double { moments[0] / window }
    /// Progress towards one full turn of accumulated angle, 0…1. Below 1 the coefficients would be extrapolated
    /// from an arc, not measured; above, the speed no longer matters.
    public var turnProgress: Double { min(1, moments[0] / (window * (1 - exp(-2 * .pi / window)))) }
    public var hasFullTurn: Bool { turnProgress >= 1 }

    /// The basis evaluated at θ with τ = 0.
    private func basis(theta: Double) -> [Double] {
        var phi = [Double](repeating: 0, count: terms)
        phi[0] = 1
        for (k, l) in orders.enumerated() {
            phi[2 + 2 * k] = cos(Double(l) * theta)
            phi[3 + 2 * k] = sin(Double(l) * theta)
        }
        return phi
    }

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
        let phi = basis(theta: theta)
        let t = terms
        // Every past sample's τ shifts by −Δθ: φ ← Aφ with A = I − Δθ e_τ e_1ᵀ, so N ← A N Aᵀ (row τ, then
        // column τ, the Δθ² term falling out of the second step) and r ← A r. Then decay and add the new sample.
        for j in 0..<t { moments[t + j] -= dth * moments[j] }
        for i in 0..<t { moments[i * t + 1] -= dth * moments[i * t] }
        for i in 0..<t {
            for j in 0..<t { moments[i * t + j] = lam * moments[i * t + j] + w * phi[i] * phi[j] }
        }

        let n = width * height
        let lamF = Float(lam), dthF = Float(dth)
        image.pixels.withUnsafeBufferPointer { img in
            acc.withUnsafeMutableBufferPointer { a in
                let r0 = a.baseAddress!, rT = r0 + n
                for i in 0..<n { rT[i] = lamF * (rT[i] - dthF * r0[i]) }
                for k in 0..<t where k != 1 {
                    let r = r0 + k * n, wk = Float(w * phi[k])
                    for i in 0..<n { r[i] = lamF * r[i] + wk * img[i] }
                }
            }
        }
        return true
    }

    /// The inverse normal matrix; nil while singular.
    private func inverseMoments() -> [[Double]]? {
        let t = terms
        var m: [[Double]] = (0..<t).map { i in
            let row = Array(moments[(i * t)..<((i + 1) * t)])
            let identity: [Double] = (0..<t).map { i == $0 ? 1 : 0 }
            return row + identity
        }
        let tiny = 1e-12 * (0..<t).reduce(0) { $0 + moments[$1 * t + $1] }
        for col in 0..<t {
            var pivot = col
            for r in col + 1..<t where abs(m[r][col]) > abs(m[pivot][col]) { pivot = r }
            if abs(m[pivot][col]) <= tiny { return nil }
            m.swapAt(col, pivot)
            let inv = 1 / m[col][col]
            for j in 0..<2 * t { m[col][j] *= inv }
            for r in 0..<t where r != col {
                let f = m[r][col]
                if f != 0 { for j in 0..<2 * t { m[r][j] -= f * m[col][j] } }
            }
        }
        return m.map { Array($0[t..<2 * t]) }
    }

    /// Rows of the inverse normal matrix giving the cos lθ and sin lθ coefficients; nil while undetermined.
    private func rows(order l: Int) -> (a: [Double], b: [Double])? {
        guard hasFullTurn, let k = orders.firstIndex(of: l), let inv = inverseMoments() else { return nil }
        return (inv[2 + 2 * k], inv[3 + 2 * k])
    }

    /// gᵀ r at every pixel, g in basis order.
    private func project(_ g: [Double]) -> [Float] {
        let n = width * height
        var out = [Float](repeating: 0, count: n)
        acc.withUnsafeBufferPointer { r in
            out.withUnsafeMutableBufferPointer { o in
                for k in 0..<terms {
                    let rk = r.baseAddress! + k * n, gk = Float(g[k])
                    for i in 0..<n { o[i] += gk * rk[i] }
                }
            }
        }
        return out
    }

    /// The coefficients (a, b) of cos lθ and sin lθ, luma units, at every pixel — c_l = a − i b; nil while
    /// undetermined.
    public func coefficients(order l: Int) -> (a: [Float], b: [Float])? {
        guard let g = rows(order: l) else { return nil }
        return (project(g.a), project(g.b))
    }

    /// Premultiplied RGBA8 overlay of the l-th harmonic of the current image, a cos lθ + b sin lθ, which turns with
    /// the object: blue → 0 → red, fully opaque at |value| = `fullScale` (luma units). Nil while undetermined.
    public func render(order l: Int, theta: Double, fullScale: Float) -> [UInt8]? {
        guard let g = rows(order: l) else { return nil }
        let c = cos(Double(l) * theta), s = sin(Double(l) * theta)
        let v = project((0..<terms).map { c * g.a[$0] + s * g.b[$0] })
        let n = width * height
        var out = [UInt8](repeating: 0, count: 4 * n)
        let inv = 255 / fullScale
        out.withUnsafeMutableBufferPointer { o in
            for i in 0..<n {
                let p = 4 * i
                let k = UInt8(min(255, abs(v[i]) * inv + 0.5))
                if v[i] > 0 { o[p] = k } else { o[p + 2] = k }
                o[p + 3] = k
            }
        }
        return out
    }
}
