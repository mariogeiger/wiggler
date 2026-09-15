import XCTest

@testable import WigglerCore

final class MotionSelectionTests: XCTestCase {
    func testReplacesStaticPointsWithAnotherMovingRegionWithoutDepth() {
        let (before, after) = trackAlternatingDiscs(separation: 0.24, roiRadiusFraction: 0.2)
        guard let first = before.marker, let last = after.marker else { return XCTFail("missing automatic marker") }
        XCTAssertLessThan(first.x, 200)
        XCTAssertGreaterThan(last.x, 280)
        XCTAssertGreaterThan(before.trackCount, 12)
        XCTAssertGreaterThan(after.trackCount, 12)
        XCTAssertTrue(after.tracks.allSatisfy { $0.x > 240 }, "static left-hand points were not replaced")
    }

    func testStaticPointsAreRetiredEvenWithSpareBudgetInTheSameRegion() {
        let (before, after) = trackAlternatingDiscs(separation: 0.13, roiRadiusFraction: 0.35)
        XCTAssertNotNil(before.marker)
        XCTAssertEqual(before.marker?.x, after.marker?.x, "region should not need a new calibration")
        XCTAssertGreaterThan(before.trackCount, 12)
        XCTAssertGreaterThan(after.trackCount, 12)
        XCTAssertLessThan(after.trackCount, EngineConfig().targetTrackCount)
        XCTAssertTrue(after.tracks.allSatisfy { $0.x > 240 }, "static points survived despite measured motion nearby")
    }

    private func trackAlternatingDiscs(separation: Double, roiRadiusFraction: Double) -> (EngineOutput, EngineOutput) {
        var left = SyntheticScene(center: V3(-separation, 0, -0.9), normal: V3(0, 0, 1), radius: 0.11)
        var right = SyntheticScene(seed: 17, center: V3(separation, 0, -0.9), normal: V3(0, 0, 1), radius: 0.11)
        left.background = []
        right.background = []
        var rng = LCG(seed: 31)
        var config = EngineConfig()
        config.roiRadiusFraction = roiRadiusFraction
        let engine = RotationEngine(config: config)
        var leftTheta = 0.0, rightTheta = 0.0
        var before = EngineOutput(), after = EngineOutput()
        for f in 0..<360 {
            if f < 150 { leftTheta += 0.04 } else { rightTheta += 0.04 }
            var input = left.render(theta: leftTheta, depthNoise: 0, rng: &rng)
            let other = right.render(theta: rightTheta, depthNoise: 0, rng: &rng)
            // Both images have the same pixel axes. Subtract the common 0.5 background once.
            input.image = GrayImage(
                width: input.image.width, height: input.image.height,
                pixels: zip(input.image.pixels, other.image.pixels).map { min(1, max(0, $0 + $1 - 0.5)) })
            input.depth = nil
            input.timestamp = Double(f) / 60
            let output = engine.process(input)
            XCTAssertNil(output.axis)
            XCTAssertLessThanOrEqual(output.trackCount, config.targetTrackCount)
            if f == 149 { before = output }
            if f == 359 { after = output }
        }
        return (before, after)
    }

    func testPausePreservesPointsAndBudgetReductionIsImmediate() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 33)
        let engine = RotationEngine()
        var theta = 0.0
        var paused: [(Float, Float)] = []
        var markerAtPause: Marker?
        for f in 0..<330 {
            if f < 150 || f >= 300 { theta += 0.035 }
            if f == 180 { engine.config.targetTrackCount = 20 }
            var input = scene.render(theta: theta, depthNoise: 0, rng: &rng)
            input.depth = nil
            input.timestamp = Double(f) / 60
            let output = engine.process(input)
            if f >= 180 { XCTAssertLessThanOrEqual(output.trackCount, 20) }
            if f == 240 {
                paused = output.tracks.map { ($0.x, $0.y) }
                markerAtPause = output.marker
                XCTAssertGreaterThan(paused.count, 12)
            }
            if f == 299 {
                XCTAssertEqual(output.trackCount, paused.count)
                XCTAssertEqual(output.marker?.x, markerAtPause?.x)
                XCTAssertEqual(output.marker?.y, markerAtPause?.y)
                for (point, reference) in zip(output.tracks, paused) {
                    XCTAssertEqual(point.x, reference.0, accuracy: 0.1)
                    XCTAssertEqual(point.y, reference.1, accuracy: 0.1)
                }
            }
        }
    }

    /// A locked body that stops and resumes slowly: its rim passes the speed threshold long before its inner
    /// points do. Tracks the angle tracker vouches for must survive, or the depth history is lost for nothing.
    func testSlowResumeKeepsRotationConsistentTracks() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 37)
        let engine = RotationEngine()
        var theta = 0.0
        var countAtPause = 0
        var minimumWhileResuming = Int.max
        for f in 0..<620 {
            if f < 400 { theta += 0.04 } else if f >= 490 { theta += 0.0017 }  // rim ≈ 8 px/s, median point ≈ 6
            var input = scene.render(theta: theta, depthNoise: 0, rng: &rng)
            input.timestamp = Double(f) / 60
            let output = engine.process(input)
            if f == 399 { XCTAssertTrue(output.axisStable, "did not lock before the pause") }
            if f == 489 {
                countAtPause = output.trackCount
                XCTAssertGreaterThan(countAtPause, 50)
            }
            if f >= 490 {
                XCTAssertEqual(output.state, .locked)
                minimumWhileResuming = min(minimumWhileResuming, output.trackCount)
            }
        }
        XCTAssertGreaterThanOrEqual(minimumWhileResuming, countAtPause - 5, "tracks were dropped for being slow")
    }

    func testMovingCameraAndMissingPoseDoNotActivatePoints() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 35)
        let engine = RotationEngine()
        for f in 0..<210 {
            var input = scene.render(theta: Double(f) * 0.04, depthNoise: 0, rng: &rng)
            input.timestamp = Double(f) / 60
            input.poseValid = f < 120
            input.cameraToWorld = RigidTransform(rotation: .identity, translation: V3(Double(f) * 0.01, 0, 0))
            let output = engine.process(input)
            XCTAssertNil(output.marker, "camera motion or missing pose activated the scene at frame \(f)")
            XCTAssertEqual(output.trackCount, 0)
        }
    }

    func testPoseJitterAndTimestampGapRecovery() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 41)
        var locator = MotionLocator()
        var previous: Pyramid?
        var time = 0.0
        var acquiredBeforeGap = false, acquiredAfterGap = false
        for f in 0..<180 {
            let input = scene.render(theta: Double(f) * 0.04, depthNoise: 0, rng: &rng)
            let pyramid = Pyramid(image: input.image, levelCount: 4)
            let dt = f == 90 ? 1.0 : 1.0 / 60
            time += dt
            let a = 0.0007 * sin(Double(f) * 1.3)
            let rotation = M3([cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1])
            let pose = RigidTransform(rotation: rotation, translation: V3(0.0001 * sin(Double(f) * 1.7), 0, 0))
            let centre = locator.add(
                image: input.image, prev: previous, cur: pyramid, pose: pose,
                dt: dt, time: time, radius: 126, klt: KLTTracker(), corners: CornerDetector())
            if centre != nil {
                if f < 90 { acquiredBeforeGap = true } else { acquiredAfterGap = true }
            }
            if f == 90 {
                XCTAssertFalse(locator.motionValid)
                XCTAssertNil(centre)
            }
            previous = pyramid
        }
        XCTAssertTrue(acquiredBeforeGap)
        XCTAssertTrue(acquiredAfterGap)
    }

    func testSearchBudgetStaysBoundedAndStaticProbesAreReplaced() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 37)
        let image = scene.render(theta: 0, depthNoise: 0, rng: &rng).image
        let pyramid = Pyramid(image: image, levelCount: 4)
        var locator = MotionLocator()
        locator.maxPoints = 60
        locator.detectEvery = 1
        var firstIDs = Set<Int>()
        for f in 0..<180 {
            let centre = locator.add(
                image: image, prev: f == 0 ? nil : pyramid, cur: pyramid, pose: .identity,
                dt: 1.0 / 60, time: Double(f) / 60, radius: 100,
                klt: KLTTracker(), corners: CornerDetector())
            XCTAssertNil(centre)
            XCTAssertLessThanOrEqual(locator.points.count, locator.maxPoints)
            if f == 30 { firstIDs = Set(locator.points.map { $0.id }) }
        }
        XCTAssertFalse(firstIDs.isEmpty)
        XCTAssertTrue(firstIDs.isDisjoint(with: Set(locator.points.map { $0.id })))
        XCTAssertFalse(locator.points.isEmpty)
    }
}
