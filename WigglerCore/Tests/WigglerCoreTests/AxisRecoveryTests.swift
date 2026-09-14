import XCTest
@testable import WigglerCore

final class AxisRecoveryTests: XCTestCase {
    func testNoisyRotationCanBuildAHarmonicMap() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 71)
        let engine = RotationEngine()
        var map = HarmonicMap(width: 1, height: 1)
        var ready = false
        for f in 0..<650 {
            var input = scene.render(theta: Double(f) * 0.04, depthNoise: 0.004, rng: &rng)
            input.timestamp = Double(f) / 60
            let out = engine.process(input)
            map.update(image: GrayImage(width: 1, height: 1, fill: 0.5), output: out)
            ready = ready || map.hasFullTurn
        }
        XCTAssertTrue(ready, "depth noise prevented a stable full turn")
    }

    func testCameraMotionInvalidatesAxisAndRequiresFreshGeometry() {
        let scene = SyntheticScene(normal: V3(0, 0, 1))
        var rng = LCG(seed: 53)
        let engine = RotationEngine()
        var map = HarmonicMap(width: 1, height: 1)
        var previous = EngineOutput()
        for f in 0..<1100 {
            var input = scene.render(theta: Double(f) * 0.04, depthNoise: 0, rng: &rng)
            input.timestamp = Double(f) / 60
            if f >= 500 { input.cameraToWorld.translation = V3(0.03, 0, 0) }
            if f >= 530 && f < 580 { input.depth = nil }
            let out = engine.process(input)
            if f == 500 {
                XCTAssertTrue(previous.axisStable)
                XCTAssertTrue(map.hasFullTurn)
                XCTAssertFalse(out.axisStable)
                XCTAssertNotNil(out.axis, "last axis must remain visible in red")
                XCTAssertEqual(out.constraintCount, 0)
            }
            map.update(image: GrayImage(width: 1, height: 1, fill: 0.5), output: out)
            if f >= 500 && f < 580 {
                XCTAssertFalse(out.axisStable)
                XCTAssertEqual(map.turnProgress, 0)
            }
            previous = out
        }
        XCTAssertTrue(previous.axisStable, "axis did not recover from camera movement")
        XCTAssertTrue(map.hasFullTurn, "map did not rebuild after camera movement")
    }

    func testMissingPoseAndTimestampGapInvalidateLockedAxis() {
        for missingPose in [true, false] {
            let scene = SyntheticScene(normal: V3(0, 0, 1))
            var rng = LCG(seed: 61)
            let engine = RotationEngine()
            var previous = EngineOutput()
            for f in 0..<700 {
                var input = scene.render(theta: Double(f) * 0.04, depthNoise: 0, rng: &rng)
                input.timestamp = Double(f) / 60 + (!missingPose && f >= 300 ? 1 : 0)
                input.poseValid = !missingPose || f != 300
                let out = engine.process(input)
                if f == 300 {
                    XCTAssertTrue(previous.axisStable)
                    XCTAssertFalse(out.axisStable)
                    XCTAssertNotNil(out.axis)
                    XCTAssertEqual(out.constraintCount, 0)
                }
                previous = out
            }
            XCTAssertTrue(previous.axisStable)
        }
    }

    func testDisplacedObjectRecalibratesWithoutCameraMotion() {
        let engine = RotationEngine()
        var rng = LCG(seed: 67)
        var previous = EngineOutput()
        var invalidated = false
        for f in 0..<800 {
            let shift = 0.02 * min(1, max(0, Double(f - 300) / 30))
            let scene = SyntheticScene(center: V3(0.05 + shift, -0.05, -0.75), normal: V3(0, 0, 1))
            var input = scene.render(theta: Double(f) * 0.04, depthNoise: 0, rng: &rng)
            input.timestamp = Double(f) / 60
            let out = engine.process(input)
            if f == 300 { XCTAssertTrue(previous.axisStable) }
            if f > 300 && f < 360 && !out.axisStable {
                invalidated = true
                XCTAssertNotNil(out.axis)
            }
            previous = out
        }
        XCTAssertTrue(invalidated, "object displacement left the axis green")
        XCTAssertTrue(previous.axisStable)
        guard let axis = previous.axis else { return XCTFail("missing recovered axis") }
        XCTAssertEqual(axis.origin.x, 0.07, accuracy: 0.005)
    }

    func testDepthBearingBodyHandoffWithinOneRegionDiscardsOldAxis() {
        var left = SyntheticScene(center: V3(-0.13, 0, -0.9), normal: V3(0, 0, 1), radius: 0.11)
        var right = SyntheticScene(seed: 17, center: V3(0.13, 0, -0.9), normal: V3(0, 0, 1), radius: 0.11)
        left.background = []; right.background = []
        var rng = LCG(seed: 59)
        let engine = RotationEngine()
        var before = EngineOutput(), after = EngineOutput()
        var invalidated = false
        for f in 0..<850 {
            var input = left.render(theta: Double(min(f, 349)) * 0.04, depthNoise: 0, rng: &rng)
            let other = right.render(theta: Double(max(0, f - 350)) * 0.04, depthNoise: 0, rng: &rng)
            input.image = GrayImage(width: input.image.width, height: input.image.height,
                                    pixels: zip(input.image.pixels, other.image.pixels).map { min(1, max(0, $0 + $1 - 0.5)) })
            input.depth!.depth = zip(input.depth!.depth, other.depth!.depth).map { min($0, $1) }
            input.timestamp = Double(f) / 60
            let out = engine.process(input)
            if f == 349 { before = out }
            if f > 350 && !out.axisStable && out.constraintCount < before.constraintCount { invalidated = true }
            if f == 849 { after = out }
        }
        XCTAssertTrue(before.axisStable, "first body did not stabilize")
        XCTAssertTrue(invalidated, "new moving support reused the old constraints")
        XCTAssertEqual(before.marker?.x, after.marker?.x, "handoff must be inside the same ROI")
        XCTAssertTrue(after.axisStable, "second body did not stabilize")
        guard let a = before.axis, let b = after.axis else { return XCTFail("missing axis") }
        XCTAssertEqual(a.origin.x, -0.13, accuracy: 0.02)
        XCTAssertEqual(b.origin.x, 0.13, accuracy: 0.02)
    }
}
