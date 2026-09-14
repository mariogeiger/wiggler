import Foundation

/// Per-pixel lock-in detection of chosen Fourier harmonics l of a scalar signal as a function of the rotation angle.
///
/// Every pixel's signal is fitted by weighted least squares on the basis {1, θ−θ_now, cos lθ, sin lθ for l in
/// `orders`} over the recent history. The trend term absorbs exposure and lighting ramps, which would otherwise leak
/// into the low harmonics; fitting all orders jointly removes the cross-talk an irregular, windowed sampling puts
/// between them. The window is an exponential in *angle* — a frame weighs |Δθ| and the past decays as e^{−|Δθ|/Θ}
/// — so a stopped object neither adds to nor forgets its map, and the coefficients are genuine Fourier coefficients
/// however irregular the rotation. Complete observations share one normal matrix. Non-finite samples are ignored,
/// with separate normal matrices for each pixel's valid observations. No frame is ever stored.
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

    private var moments: HarmonicMoments
    /// Per-pixel right-hand sides Σ w I φ: `terms` planes of width × height, in basis order.
    private var acc: [Float]
    private var thetaPrev: Double?

    public init(width: Int, height: Int, orders: [Int] = [1], windowTurns: Double = 3) {
        precondition(width > 0 && height > 0 && windowTurns > 0 && windowTurns.isFinite)
        precondition(!orders.isEmpty && orders.allSatisfy { $0 >= 1 } && Set(orders).count == orders.count)
        self.width = width
        self.height = height
        self.window = windowTurns * 2 * .pi
        self.orders = orders
        self.terms = 2 + 2 * orders.count
        self.moments = HarmonicMoments(terms: terms, pixelCount: width * height)
        self.acc = [Float](repeating: 0, count: terms * width * height)
    }

    public mutating func reset() {
        moments.reset()
        for i in 0..<acc.count { acc[i] = 0 }
        thetaPrev = nil
    }

    /// Total weight in the window as a fraction of its e-folding length (→ 1 after many turns).
    public var fill: Double { moments.maximumWeight / window }
    /// Progress towards one full turn of accumulated angle, 0…1. Below 1 the coefficients would be extrapolated
    /// from an arc, not measured; above, the speed no longer matters.
    public var turnProgress: Double { min(1, moments.maximumWeight / fullTurnWeight) }
    public var hasFullTurn: Bool { turnProgress >= 1 }
    private var fullTurnWeight: Double { window * (1 - exp(-2 * .pi / window)) }

    /// Accumulate only a stable measurement. Invalidation discards both the fit and the angle reference.
    public mutating func update(image: GrayImage, output: EngineOutput) {
        guard output.axisStable && output.state == .locked else {
            if thetaPrev != nil { reset() }
            return
        }
        add(image: image, theta: output.theta)
    }

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
        guard theta.isFinite else {
            reset()
            return false
        }
        guard let prev = thetaPrev else {
            thetaPrev = theta
            return true
        }
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
        let valid = image.pixels.allSatisfy(\.isFinite) ? nil : image.pixels.map(\.isFinite)
        moments.add(basis: phi, delta: dth, decay: lam, weight: w, valid: valid)

        let n = width * height
        let lamF = Float(lam), dthF = Float(dth)
        image.pixels.withUnsafeBufferPointer { img in
            acc.withUnsafeMutableBufferPointer { a in
                let r0 = a.baseAddress!, rT = r0 + n
                for i in 0..<n { rT[i] = lamF * (rT[i] - dthF * r0[i]) }
                for k in 0..<t where k != 1 {
                    let r = r0 + k * n, wk = Float(w * phi[k])
                    for i in 0..<n { r[i] = lamF * r[i] + (img[i].isFinite ? wk * img[i] : 0) }
                }
            }
        }
        return true
    }

    private func weights(order: Int, cosine: Double, sine: Double) -> [Double]? {
        guard hasFullTurn, let k = orders.firstIndex(of: order) else { return nil }
        var weights = [Double](repeating: 0, count: terms)
        weights[2 + 2 * k] = cosine
        weights[3 + 2 * k] = sine
        return weights
    }

    /// The coefficients (a, b) of cos lθ and sin lθ, signal units, at every pixel — c_l = a − i b; nil while
    /// undetermined.
    public func coefficients(order l: Int) -> (a: [Float], b: [Float])? {
        guard let a = weights(order: l, cosine: 1, sine: 0), let b = weights(order: l, cosine: 0, sine: 1) else {
            return nil
        }
        return (
            moments.project(rhs: acc, weights: a, minimumWeight: fullTurnWeight),
            moments.project(rhs: acc, weights: b, minimumWeight: fullTurnWeight)
        )
    }

    /// Premultiplied RGBA8 overlay of the l-th harmonic of the current image, a cos lθ + b sin lθ, which turns with
    /// the object: blue → 0 → red, fully opaque at |value| = `fullScale` (signal units). Nil while undetermined.
    public func render(order l: Int, theta: Double, fullScale: Float) -> [UInt8]? {
        guard fullScale > 0, fullScale.isFinite,
            let weights = weights(order: l, cosine: cos(Double(l) * theta), sine: sin(Double(l) * theta))
        else { return nil }
        let v = moments.project(rhs: acc, weights: weights, minimumWeight: fullTurnWeight)
        let n = width * height
        var out = [UInt8](repeating: 0, count: 4 * n)
        let inv = 255 / fullScale
        out.withUnsafeMutableBufferPointer { o in
            for i in 0..<n where v[i].isFinite {
                let p = 4 * i
                let k = UInt8(min(255, abs(v[i]) * inv + 0.5))
                if v[i] > 0 { o[p] = k } else { o[p + 2] = k }
                o[p + 3] = k
            }
        }
        return out
    }
}
