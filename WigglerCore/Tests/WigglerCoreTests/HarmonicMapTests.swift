import XCTest
@testable import WigglerCore

final class HarmonicMapTests: XCTestCase {
    /// I(p, θ) = m + Σ_l A_l cos(lθ − φ_l) + trend·θ over l = 1, 2, 3, irregular steps in both directions: the
    /// model lies exactly in the fit's span, so the recursive estimator must return (A_l cos φ_l, A_l sin φ_l) for
    /// every order up to float accumulation error.
    func testRecoversHarmonicsUnderIrregularRotationAndTrend() {
        let w = 8, h = 6, n = w * h
        let orders = [1, 2, 3]
        var map = HarmonicMap(width: w, height: h, orders: orders, windowTurns: 3)
        // Phase and amplitude per (pixel, order).
        func phi(_ i: Int, _ l: Int) -> Double { (Double(i) / Double(n) + 0.17 * Double(l)) * 2 * .pi }
        func amp(_ i: Int, _ l: Int) -> Double { (0.05 + 0.1 * Double(i % 5) / 4) / Double(l) }
        let mean = 0.5, trend = 0.02
        var rng = LCG(seed: 7)
        var theta = 0.3
        for k in 0..<600 {
            // 5° ± 4° per frame, with a reversal in the middle.
            theta += (k < 300 ? 1 : -1) * (5 + 4 * rng.uniform(-1, 1)) * .pi / 180
            let px = (0..<n).map { i in
                Float(mean + trend * theta + orders.reduce(0) { $0 + amp(i, $1) * cos(Double($1) * theta - phi(i, $1)) })
            }
            XCTAssertTrue(map.add(image: GrayImage(width: w, height: h, pixels: px), theta: theta))
        }
        XCTAssertTrue(map.hasFullTurn)
        for l in orders {
            let sol = map.coefficients(order: l)!
            for i in 0..<n {
                XCTAssertEqual(Double(sol.a[i]), amp(i, l) * cos(phi(i, l)), accuracy: 2e-4, "l=\(l)")
                XCTAssertEqual(Double(sol.b[i]), amp(i, l) * sin(phi(i, l)), accuracy: 2e-4, "l=\(l)")
            }
            // Render: sign and opacity of the l-th harmonic of the current image.
            let img = map.render(order: l, theta: theta, fullScale: 0.1)!
            for i in 0..<n {
                let v = amp(i, l) * cos(Double(l) * theta - phi(i, l))
                XCTAssertEqual(Int(img[4 * i + 3]), Int(min(1, abs(v) / 0.1) * 255 + 0.5), accuracy: 2)
                if abs(v) > 0.002 { XCTAssertEqual(img[4 * i] > 0, v > 0); XCTAssertEqual(img[4 * i + 2] > 0, v < 0) }
            }
        }
        XCTAssertNil(map.coefficients(order: 4))
    }

    func testJumpResetsAndFullTurnGates() {
        var map = HarmonicMap(width: 2, height: 1)
        let img = GrayImage(width: 2, height: 1, fill: 0.5)
        var theta = 0.0
        for _ in 0..<30 { theta += 0.1; map.add(image: img, theta: theta) }
        XCTAssertFalse(map.hasFullTurn)
        XCTAssertNil(map.coefficients(order: 1))
        for _ in 0..<40 { theta += 0.1; map.add(image: img, theta: theta) }
        XCTAssertTrue(map.hasFullTurn)
        XCTAssertNotNil(map.coefficients(order: 1))
        XCTAssertFalse(map.add(image: img, theta: theta + 2))
        XCTAssertEqual(map.fill, 0)
        XCTAssertTrue(map.add(image: img, theta: theta + 2.1))
    }
}
