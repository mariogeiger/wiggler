import Foundation

/// Fuses the two angle measurements the engine produces.
///
/// * The **geometric increment** (azimuth of the tracked 3D points around the axis) is the relative measurement.
///   It is precise from one frame to the next but has no absolute reference, and it drifts as tracks are renewed.
///   It plays the role of a gyroscope.
/// * The **appearance match** (current patch against the library of patches recorded per angle) is the absolute
///   measurement. It does not drift, but it is ambiguous for a periodic object and plainly wrong when the library
///   no longer describes the scene. It plays the role of a compass.
///
/// The fusion is the classic complementary filter around three rules:
///
/// 1. **The state is never teleported.** A correction becomes a bounded rate, so the displayed angle is always
///    continuous — and the rpm read-out, which comes from the geometric increment alone, is never polluted by it.
/// 2. **An honest uncertainty.** `sigma` grows at the rate the angle can actually run away: as a random walk
///    through track turnover while the geometry measures, and *linearly* at the rotation speed itself while it
///    does not, because an object nobody is watching keeps turning the same way.
/// 3. **Gate and validate.** A match is used only inside a 3-sigma innovation gate (sigma of the *innovation*:
///    state and measurement noise together), and only if no clearly better match sits outside it. After a healthy
///    run the gate is about 15°, so the 120° lobe of a hexagonal box cannot be followed; after a real loss it opens
///    to ±180°, so the absolute angle can be re-acquired.
///
/// When a strong match keeps contradicting a healthy geometry, it is the library that is wrong (the object was
/// displaced, the light changed): `staleSeconds` builds up and the engine rebuilds it instead of fighting it.
public struct AngleFusion {
    /// Uncertainty of the integrated angle (radians, 1 sigma).
    private(set) public var sigma: Double
    /// Correction not yet applied, spread over time by the rate limit.
    private(set) public var pending: Double = 0
    /// How long a strong appearance match has been contradicting a healthy geometry.
    private(set) public var staleSeconds: Double = 0
    /// The last match was sharp and consistent: the library may be refreshed with the current patch.
    private(set) public var lastMatchTrusted = false
    /// Last known rotation rate (rad/s), held while the geometry is blind.
    private var omegaHeld: Double = 0

    /// Maximum correction rate (rad/s): a 150° re-acquisition takes about 1.7 s, continuously.
    public var maxCorrectionRate = 90 * Double.pi / 180
    /// Fraction of the rotation rate that leaks into the integrated angle through track turnover.
    public var turnoverDrift = 0.02
    /// Drift floor (rad/s) even at rest.
    public var driftFloor = 0.3 * Double.pi / 180
    /// Extra uncertainty rate (rad/s) while nothing measures the angle, on top of the rotation rate itself.
    public var blindMargin = 30 * Double.pi / 180

    public init(sigma: Double = 3 * Double.pi / 180) { self.sigma = sigma }

    public mutating func reset(sigma: Double = 3 * Double.pi / 180) {
        self.sigma = sigma
        pending = 0
        staleSeconds = 0
        lastMatchTrusted = false
        omegaHeld = 0
    }

    public enum Outcome: String {
        case noMeasurement      // nothing to compare against
        case gated              // a match exists but is outside the gate
        case ambiguous          // a clearly better match sits outside the gate: do not trust this measurement
        case updated            // the state was corrected
    }

    public struct Candidate {
        public var theta: Double
        public var score: Double
        public init(theta: Double, score: Double) { self.theta = theta; self.score = score }
    }

    /// - Parameters:
    ///   - theta: current integrated angle.
    ///   - omega: geometric rotation rate (rad/s); ignored while `healthy` is false (the last value is held).
    ///   - healthy: the geometry is really measuring this object (enough consistent points, and the independent
    ///     image-plane rotation agrees with the geometric increment).
    ///   - candidates: appearance matches, any order.
    /// - Returns: the correction to apply to theta this frame (always bounded), and what happened.
    public mutating func update(theta: Double, omega: Double, healthy: Bool,
                                candidates: [Candidate], dt: Double) -> (correction: Double, outcome: Outcome) {
        if healthy { omegaHeld = omega }
        let w = abs(omegaHeld)
        if healthy {
            let rate = turnoverDrift * w + driftFloor
            sigma = (sigma * sigma + rate * dt * rate * dt).squareRoot()
        } else {
            sigma += (1.2 * w + blindMargin) * dt
        }
        sigma = min(sigma, .pi)

        var outcome = Outcome.noMeasurement
        lastMatchTrusted = false
        if !candidates.isEmpty {
            // The gate is on the innovation covariance S = P + R, not on P alone: the measurement's own noise
            // (about one bin) is part of what makes an innovation ordinary. Gating on P would, once the state is
            // confident, reject normal measurements and then mistake them for a stale library.
            func gate(_ c: Candidate) -> Double { min(3 * (sigma * sigma + Self.sigmaM(c) * Self.sigmaM(c)).squareRoot(), .pi) }
            let reference = theta + pending
            var bestInside: Candidate?
            var bestOverall: Candidate?
            for c in candidates {
                if bestOverall == nil || c.score > bestOverall!.score { bestOverall = c }
                if c.score > 0.55 && abs(wrapAngle(c.theta - reference)) < gate(c) {
                    if bestInside == nil || c.score > bestInside!.score { bestInside = c }
                }
            }
            if let inside = bestInside, let overall = bestOverall, overall.score <= inside.score + 0.05 {
                let innovation = wrapAngle(inside.theta - reference)
                let sigmaM = Self.sigmaM(inside)
                let k = sigma * sigma / (sigma * sigma + sigmaM * sigmaM)
                pending += k * innovation
                sigma = ((1 - k) * sigma * sigma).squareRoot()
                staleSeconds = max(0, staleSeconds - 2 * dt)
                lastMatchTrusted = inside.score > 0.6 && abs(innovation) < 10 * Double.pi / 180
                outcome = .updated
            } else {
                // The measurement contradicts the state. Never follow it — and never let it creep in through a
                // marginal candidate. Only a *strong* disagreement means the library itself is out of date.
                if healthy, let overall = bestOverall, overall.score > 0.7,
                   abs(wrapAngle(overall.theta - reference)) > gate(overall) {
                    staleSeconds += dt
                }
                outcome = bestInside == nil ? .gated : .ambiguous
            }
        }
        let step = max(-maxCorrectionRate * dt, min(maxCorrectionRate * dt, pending))
        pending -= step
        return (step, outcome)
    }

    public mutating func clearStale() { staleSeconds = 0 }

    /// Measurement noise of an appearance match: a sharp match is worth about one bin, a blurry one several.
    private static func sigmaM(_ c: Candidate) -> Double { (5 * Double.pi / 180) / max(c.score, 0.3) }
}

/// Robust 2D rotation of a set of correspondences (Procrustes + IRLS).
///
/// This is deliberately computed from the *image* positions only: no depth, no axis. It therefore fails in
/// different situations than the 3D azimuth, which is exactly what makes it useful as a cross-check — if the two
/// disagree, something has taken over the tracked points (a hand, a reflection) and the geometric angle must not
/// be trusted, however many "inliers" it reports.
public func similarityRotation(from a: [(Float, Float)], to b: [(Float, Float)]) -> (angle: Double, inliers: Int)? {
    let n = a.count
    guard n >= 8, b.count == n else { return nil }
    var w = [Double](repeating: 1, count: n)
    var angle = 0.0
    var inliers = 0
    for _ in 0..<3 {
        var sw = 0.0, ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0
        for i in 0..<n {
            sw += w[i]
            ax += w[i] * Double(a[i].0); ay += w[i] * Double(a[i].1)
            bx += w[i] * Double(b[i].0); by += w[i] * Double(b[i].1)
        }
        if sw < 1e-9 { return nil }
        ax /= sw; ay /= sw; bx /= sw; by /= sw
        var num = 0.0, den = 0.0, norm = 0.0
        for i in 0..<n {
            let px = Double(a[i].0) - ax, py = Double(a[i].1) - ay
            let qx = Double(b[i].0) - bx, qy = Double(b[i].1) - by
            num += w[i] * (px * qy - py * qx)
            den += w[i] * (px * qx + py * qy)
            norm += w[i] * (px * px + py * py)
        }
        if norm < 1e-9 { return nil }
        angle = atan2(num, den)
        let scale = (num * num + den * den).squareRoot() / norm
        let c = cos(angle) * scale, s = sin(angle) * scale
        var residuals = [Double](repeating: 0, count: n)
        for i in 0..<n {
            let px = Double(a[i].0) - ax, py = Double(a[i].1) - ay
            let qx = Double(b[i].0) - bx, qy = Double(b[i].1) - by
            let ex = c * px - s * py - qx
            let ey = s * px + c * py - qy
            residuals[i] = (ex * ex + ey * ey).squareRoot()
        }
        let scaleR = 1.5 * median(residuals) + 1e-6
        inliers = 0
        for i in 0..<n {
            let r = residuals[i] / scaleR
            w[i] = 1 / (1 + r * r)
            if residuals[i] < 2 * scaleR { inliers += 1 }
        }
    }
    return (angle, inliers)
}
