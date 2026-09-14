import XCTest
@testable import WigglerCore

final class MotionLocatorTests: XCTestCase {
    /// A static scene must not produce a marker; once the disc turns, the marker lands on it within a second or so.
    func testAutoMarkerLandsOnTheTurningDisc() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 3)
        let engine = RotationEngine()
        var theta = 0.0
        var placedAt: Int?
        var marker: Marker?
        for f in 0..<240 {
            let rpm = f < 60 ? 0.0 : 20
            theta += rpm / 60 * 2 * .pi / 60
            var input = scene.render(theta: theta, depthNoise: 0.004, rng: &rng)
            input.timestamp = Double(f) / 60
            let out = engine.process(input)
            if f < 60 { XCTAssertNil(out.marker, "marker placed on a static scene at frame \(f)") }
            if placedAt == nil, let m = out.marker { placedAt = f; marker = m }
        }
        guard let at = placedAt, let m = marker else { return XCTFail("marker never placed") }
        XCTAssertLessThan(at, 60 + 90, "took \(at - 60) frames after motion started")
        let (cx, cy) = scene.projectedCenter()
        let dist = ((m.x - cx) * (m.x - cx) + (m.y - cy) * (m.y - cy)).squareRoot()
        XCTAssertLessThan(dist, scene.projectedRadius() / 2, "marker \(dist) px from the disc centre")
        XCTAssertEqual(m.radius, Float(0.35 * 360), accuracy: 1e-3)
    }

    func testModeFindsTheDenseCluster() {
        var rng = LCG(seed: 5)
        var pts: [(Float, Float)] = []
        for _ in 0..<40 { pts.append((Float(200 + rng.uniform(-30, 30)), Float(150 + rng.uniform(-30, 30)))) }
        for _ in 0..<15 { pts.append((Float(rng.uniform(0, 480)), Float(rng.uniform(0, 360)))) }
        let m = MotionLocator.mode(of: pts, radius: 60, minCount: 12)!
        XCTAssertEqual(m.x, 200, accuracy: 12)
        XCTAssertEqual(m.y, 150, accuracy: 12)
        XCTAssertNil(MotionLocator.mode(of: Array(pts[40...]), radius: 60, minCount: 12))
    }
}
