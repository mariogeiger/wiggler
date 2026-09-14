import XCTest
@testable import WigglerCore

final class AxisStabilityTests: XCTestCase {
    func testOnlyFreshConsistentEstimatesConfirmStability() {
        var stability = AxisStability()
        for _ in 0..<10 { stability.observe(frame: 10, consistent: true) }
        XCTAssertFalse(stability.isStable(required: 3))
        stability.observe(frame: 20, consistent: true)
        stability.observe(frame: 30, consistent: true)
        XCTAssertTrue(stability.isStable(required: 3))
        stability.observe(frame: 40, consistent: false)
        XCTAssertFalse(stability.isStable(required: 3))
        stability.observe(frame: 50, consistent: true)
        stability.invalidate()
        for _ in 0..<10 { stability.observe(frame: 50, consistent: true) }
        XCTAssertEqual(stability.confirmations, 0, "invalid evidence cannot be replayed")
        for frame in 60...62 { stability.observe(frame: frame, consistent: true) }
        XCTAssertTrue(stability.isStable(required: 3))
    }

    func testAxisComparisonIgnoresSignAndPositionAlongTheLine() {
        let axis = Axis(origin: .zero, direction: V3(0, 1, 0))
        XCTAssertTrue(AxisStability.agrees(Axis(origin: V3(0, 2, 0), direction: V3(0, -1, 0)),
                                          with: axis, objectRadius: 0.1))
        XCTAssertFalse(AxisStability.agrees(Axis(origin: V3(0.1, 0, 0), direction: axis.direction),
                                           with: axis, objectRadius: 0.1))
        XCTAssertFalse(AxisStability.agrees(Axis(origin: .zero, direction: V3(1, 1, 0)),
                                           with: axis, objectRadius: 0.1))
    }

    func testNewBodyChordsRejectAnOldAxis() {
        let old = Axis(origin: V3(-0.13, 0, -0.9), direction: V3(0, 0, 1))
        let new = Axis(origin: V3(0.13, 0, -0.9), direction: V3(0, 0, 1))
        let chords = (0..<60).map { i -> ChordConstraint in
            let theta = Double(i) * 2 * .pi / 60
            let a = new.origin + V3(cos(theta), sin(theta), 0) * 0.1
            let b = new.origin + V3(cos(theta + 0.4), sin(theta + 0.4), 0) * 0.1
            return ChordConstraint(midpoint: (a + b) * 0.5, chord: b - a, frame: 100)
        }
        XCTAssertTrue(AxisStability.rejects(chords, axis: old, tolerance: 0.03))
        XCTAssertFalse(AxisStability.rejects(chords, axis: new, tolerance: 0.03))
        var stability = AxisStability()
        for frame in 1...3 { stability.observe(frame: frame, consistent: true) }
        XCTAssertTrue(stability.isStable(required: 3))
        XCTAssertFalse(stability.observe(chords: chords, axis: old, tolerance: 0.01, required: 3))
        XCTAssertFalse(stability.isStable(required: 3), "first suspect batch must turn the axis red")
        XCTAssertFalse(stability.observe(chords: [], axis: old, tolerance: 0.01, required: 3))
        XCTAssertFalse(stability.observe(chords: chords, axis: old, tolerance: 0.01, required: 3))
        XCTAssertTrue(stability.observe(chords: chords, axis: old, tolerance: 0.01, required: 3))
        stability.reset()
        XCTAssertFalse(stability.observe(chords: chords, axis: old, tolerance: 0.01, required: 3))
        XCTAssertFalse(stability.observe(chords: chords, axis: new, tolerance: 0.01, required: 3))
        XCTAssertFalse(stability.observe(chords: chords, axis: old, tolerance: 0.01, required: 3))
    }

    func testHarmonicsDiscardUnstableFramesAndRebuildFromZero() {
        var map = HarmonicMap(width: 1, height: 1)
        var out = EngineOutput()
        out.state = .locked
        out.axisStable = true
        for f in 0...150 {
            out.theta = Double(f) * 0.05
            map.update(image: GrayImage(width: 1, height: 1, fill: Float(0.5 + 0.2 * cos(out.theta))), output: out)
        }
        XCTAssertTrue(map.hasFullTurn)
        out.axisStable = false
        map.update(image: GrayImage(width: 1, height: 1, fill: 1), output: out)
        XCTAssertEqual(map.turnProgress, 0)
        XCTAssertNil(map.coefficients(order: 1))
        out.axisStable = true
        for f in 0...150 {
            out.theta = Double(f) * 0.05
            map.update(image: GrayImage(width: 1, height: 1, fill: Float(0.5 + 0.1 * sin(out.theta))), output: out)
            if f < 125 { XCTAssertFalse(map.hasFullTurn) }
        }
        guard let (a, b) = map.coefficients(order: 1) else { return XCTFail("map did not rebuild") }
        XCTAssertEqual(a[0], 0, accuracy: 0.0001)
        XCTAssertEqual(b[0], 0.1, accuracy: 0.0001)
        out.state = .lost
        map.update(image: GrayImage(width: 1, height: 1, fill: 0), output: out)
        XCTAssertEqual(map.turnProgress, 0)
    }
}
