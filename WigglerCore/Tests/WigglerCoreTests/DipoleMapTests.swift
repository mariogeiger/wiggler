import XCTest
@testable import WigglerCore

final class DipoleMapTests: XCTestCase {
    /// I(p, θ) = m + A cos(θ − φ) + trend·θ, irregular steps in both directions: the model lies exactly in the
    /// fit's span, so the recursive estimator must return (A cos φ, A sin φ) up to float accumulation error.
    func testRecoversDipoleUnderIrregularRotationAndTrend() {
        let w = 8, h = 6, n = w * h
        var map = DipoleMap(width: w, height: h, windowTurns: 3)
        let phi = (0..<n).map { Double($0) / Double(n) * 2 * .pi }
        let amp = (0..<n).map { 0.05 + 0.1 * Double($0 % 5) / 4 }
        let mean = 0.5, trend = 0.02
        var rng = LCG(seed: 7)
        var theta = 0.3
        for k in 0..<600 {
            // 5° ± 4° per frame, with a reversal in the middle.
            theta += (k < 300 ? 1 : -1) * (5 + 4 * rng.uniform(-1, 1)) * .pi / 180
            let px = (0..<n).map { Float(mean + amp[$0] * cos(theta - phi[$0]) + trend * theta) }
            XCTAssertTrue(map.add(image: GrayImage(width: w, height: h, pixels: px), theta: theta))
        }
        XCTAssertTrue(map.hasFullTurn)
        let sol = map.solve()!
        for i in 0..<n {
            XCTAssertEqual(Double(sol.a[i]), amp[i] * cos(phi[i]), accuracy: 2e-4)
            XCTAssertEqual(Double(sol.b[i]), amp[i] * sin(phi[i]), accuracy: 2e-4)
        }
        // Render: sign and opacity of the "current" and "fixed" images, hue of the phase image.
        let cur = map.render(.current, theta: theta, fullScale: 0.1)!
        let fixed = map.render(.fixed, theta: theta, fullScale: 0.1)!
        for i in 0..<n {
            let vCur = amp[i] * cos(theta - phi[i]), vFix = amp[i] * cos(phi[i])
            XCTAssertEqual(Int(cur[4 * i + 3]), Int(min(1, abs(vCur) / 0.1) * 255 + 0.5), accuracy: 2)
            XCTAssertEqual(cur[4 * i] > 0, vCur > 0.001); XCTAssertEqual(cur[4 * i + 2] > 0, vCur < -0.001)
            XCTAssertEqual(fixed[4 * i] > 0, vFix > 0.001); XCTAssertEqual(fixed[4 * i + 2] > 0, vFix < -0.001)
        }
        let mag = map.render(.magnitude, theta: 0, fullScale: 0.1)!
        XCTAssertEqual(Int(mag[3]), Int(min(1, amp[0] / 0.1) * 255 + 0.5), accuracy: 2)
        let phase = map.render(.phase, theta: 0, fullScale: 0.05)!
        XCTAssertEqual(phase[3], 255)                       // amp[0] = 0.05 → opaque
        XCTAssertEqual(Int(phase[0]), 255, accuracy: 2)     // φ = 0 → pure red
    }

    func testJumpResetsAndFullTurnGates() {
        var map = DipoleMap(width: 2, height: 1)
        let img = GrayImage(width: 2, height: 1, fill: 0.5)
        var theta = 0.0
        for _ in 0..<30 { theta += 0.1; map.add(image: img, theta: theta) }
        XCTAssertFalse(map.hasFullTurn)
        XCTAssertNil(map.solve())
        for _ in 0..<40 { theta += 0.1; map.add(image: img, theta: theta) }
        XCTAssertTrue(map.hasFullTurn)
        XCTAssertNotNil(map.solve())
        XCTAssertFalse(map.add(image: img, theta: theta + 2))
        XCTAssertEqual(map.fill, 0)
        XCTAssertTrue(map.add(image: img, theta: theta + 2.1))
    }
}
