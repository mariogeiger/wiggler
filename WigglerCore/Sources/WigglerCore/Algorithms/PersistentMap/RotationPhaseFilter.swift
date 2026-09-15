import Foundation

/// Carries the body's rotation angle and rate between measurements: the angle advances at the current rate and
/// its uncertainty grows with a white-noise angular-acceleration process, so a frame that measures nothing still
/// leaves a usable prediction and a widening interval. Measurements arrive with their own information (radians⁻²).
///
/// `sigma` is conditional: it describes the spread this model implies if the axis and the map are right and the
/// pixel errors of the landmarks are independent. It does not cover a biased axis or depth, correlations between
/// landmarks or between frames, or an angle that another angle explains equally well.
struct RotationPhaseFilter {
    /// Continuous (unwrapped) angle, radians.
    private(set) var theta = 0.0
    /// Rate, radians per second.
    private(set) var omega = 0.0
    /// Covariance of (theta, omega), symmetric.
    private var p00: Double
    private var p01: Double
    private var p11: Double
    /// Square root of the angular-acceleration noise density of the process, radians per second to the power three
    /// halves: the covariance it adds over `dt` is this squared times [[dt³/3, dt²/2], [dt²/2, dt]].
    var accelerationNoiseDensity = 30 * Double.pi / 180

    init(thetaSigma: Double = 5 * .pi / 180, omegaSigma: Double = 30 * .pi / 180) {
        p00 = thetaSigma * thetaSigma
        p01 = 0
        p11 = omegaSigma * omegaSigma
    }

    /// 1-sigma spread of the angle under this model, radians. Conditional: see the type's own note.
    var sigma: Double { max(p00, 0).squareRoot() }

    mutating func reset(theta: Double, thetaSigma: Double = 5 * .pi / 180, omegaSigma: Double = 30 * .pi / 180) {
        self.theta = theta
        omega = 0
        p00 = thetaSigma * thetaSigma
        p01 = 0
        p11 = omegaSigma * omegaSigma
    }

    /// Advance to the next frame and return the predicted angle.
    mutating func predict(dt: Double) -> Double {
        theta += omega * dt
        // [[1, dt], [0, 1]] P [[1, dt], [0, 1]]ᵀ + acceleration process noise.
        let a = accelerationNoiseDensity * accelerationNoiseDensity
        let next00 = p00 + 2 * dt * p01 + dt * dt * p11 + a * dt * dt * dt / 3
        let next01 = p01 + dt * p11 + a * dt * dt / 2
        let next11 = p11 + a * dt
        p00 = next00
        p01 = next01
        p11 = next11
        return theta
    }

    /// Fold in a measured angle whose `information` is 1/variance in radians⁻². The prediction must already be
    /// this frame's, so the innovation is measured against `theta`.
    mutating func correct(theta measured: Double, information: Double) {
        let variance = 1 / max(information, 1e-12)
        let denominator = p00 + variance
        guard denominator > 0 else { return }
        let gain0 = p00 / denominator, gain1 = p01 / denominator
        let innovation = measured - theta
        theta += gain0 * innovation
        omega += gain1 * innovation
        // P ← (I − K H) P with H = [1, 0]; the result stays symmetric because P01 = P10.
        p00 -= gain0 * p00
        p11 -= gain1 * p01
        p01 -= gain0 * p01
    }

    /// Set the rate directly, keeping its uncertainty wide: used once, when the sense of rotation is first known.
    mutating func seed(omega value: Double, sigma: Double) {
        omega = value
        p11 = sigma * sigma
        p01 = 0
    }
}
