import XCTest

@testable import WigglerCore

final class RotationConsensusTests: XCTestCase {
    private let intrinsics = CameraIntrinsics(fx: 420, fy: 420, cx: 240, cy: 180)
    private let axis = Axis(origin: V3(0, 0, -1), direction: V3(0.2, 0.4, 1))

    private func point(_ index: Int, theta: Double = 0, radius: Double = 0.16) -> V3 {
        let phi = Double(index) * 0.71 + theta
        return axis.origin + axis.e1 * (radius * cos(phi)) + axis.e2 * (radius * sin(phi))
    }

    private func match(id: Int, before: V3, after: V3, pose: RigidTransform = .identity) -> RotationImageMatch {
        let p = pose.inverse.apply(after)
        return RotationImageMatch(
            id: id, point: before, x: intrinsics.cx - intrinsics.fx * p.x / p.z,
            y: intrinsics.cy + intrinsics.fy * p.y / p.z)
    }

    private func matches(delta: Double, theta: Double = 0, pose: RigidTransform = .identity) -> [RotationImageMatch] {
        let moving = (0..<12).map {
            match(id: $0, before: point($0, theta: theta), after: point($0, theta: theta + delta), pose: pose)
        }
        let background = (100..<220).map { id in
            let p = point(id, radius: 0.5)
            return match(id: id, before: p, after: p, pose: pose)
        }
        return moving + background
    }

    @discardableResult
    private func update(
        _ consensus: inout RotationConsensus, _ matches: [RotationImageMatch], pose: RigidTransform = .identity
    )
        -> RotationConsensus.Decision
    {
        consensus.update(
            matches, axis: axis, cameraToWorld: pose, intrinsics: intrinsics,
            retaining: Set(matches.map { $0.id }))
    }

    func testRotatingMinorityWinsAgainstDistantStaticMajorityInEitherDirection() {
        for delta in [-0.12, 0.12] {
            var consensus = RotationConsensus()
            let decision = update(&consensus, matches(delta: delta))
            XCTAssertEqual(decision.delta ?? 0, delta, accuracy: 1e-6)
            XCTAssertEqual(decision.support, 12)
            XCTAssertEqual(consensus.rotatingIDs, Set(0..<12))
            XCTAssertEqual(decision.backgroundIDs, Set(100..<220))
        }
    }

    func testCameraMotionIsCompensatedForBothModels() {
        let pose = RigidTransform(
            rotation: M3([cos(0.1), -sin(0.1), 0, sin(0.1), cos(0.1), 0, 0, 0, 1]),
            translation: V3(0.025, -0.01, 0.02))
        var consensus = RotationConsensus()
        let decision = update(&consensus, matches(delta: 0.07, pose: pose), pose: pose)
        XCTAssertEqual(decision.delta ?? 0, 0.07, accuracy: 1e-6)
        XCTAssertEqual(consensus.rotatingIDs, Set(0..<12))
        var staticOnly = RotationConsensus()
        XCTAssertNil(update(&staticOnly, matches(delta: 0, pose: pose), pose: pose).delta)
        XCTAssertTrue(staticOnly.rotatingIDs.isEmpty)
    }

    func testPauseSlowResumeReverseAndNewStaticTracksPreserveMembership() {
        var consensus = RotationConsensus()
        update(&consensus, matches(delta: 0.1))
        for delta in Array(repeating: 0.0, count: 50) + Array(repeating: 0.0005, count: 50) + [-0.08] {
            var input = matches(delta: delta)
            for id in 300..<330 {
                input.append(match(id: id, before: point(id), after: point(id)))
            }
            update(&consensus, input)
            XCTAssertEqual(consensus.rotatingIDs, Set(0..<12))
        }
    }

    func testStationaryOnlyAndInsufficientSupportDoNotInventMembers() {
        var consensus = RotationConsensus()
        XCTAssertNil(update(&consensus, matches(delta: 0)).delta)
        XCTAssertTrue(consensus.rotatingIDs.isEmpty)
        let sparse = Array(matches(delta: 0.1).prefix(3)) + Array(matches(delta: 0).dropFirst(12))
        XCTAssertNil(update(&consensus, sparse).delta)
        XCTAssertTrue(consensus.rotatingIDs.isEmpty)
    }

    func testInvalidPixelsDepthAndOnAxisPointsCannotVote() {
        var consensus = RotationConsensus()
        var input = matches(delta: 0.1)
        input.append(RotationImageMatch(id: 400, point: V3(0, 0, 1), x: 0, y: 0))
        input.append(RotationImageMatch(id: 401, point: point(0), x: .nan, y: 0))
        input.append(match(id: 402, before: axis.origin, after: axis.origin))
        update(&consensus, input)
        XCTAssertEqual(consensus.rotatingIDs, Set(0..<12))
    }

    func testSubpixelNoiseAtRestIsAmbiguousAndDoesNotAdmitBackground() {
        var unknown = RotationConsensus()
        var known = RotationConsensus()
        update(&known, matches(delta: 0.1))
        var rng = LCG(seed: 7)
        for _ in 0..<60 {
            var input = matches(delta: 0)
            for i in input.indices {
                input[i].x += rng.uniform(-0.8, 0.8)
                input[i].y += rng.uniform(-0.8, 0.8)
            }
            XCTAssertNil(update(&unknown, input).delta)
            XCTAssertTrue(unknown.rotatingIDs.isEmpty)
            XCTAssertNil(update(&known, input).delta)
            XCTAssertEqual(known.rotatingIDs, Set(0..<12))
        }
    }

    func testMovingConsensusToleratesPixelAndPreviousDepthNoise() {
        var consensus = RotationConsensus()
        var input = matches(delta: 0.08)
        var rng = LCG(seed: 83)
        for i in input.indices {
            input[i].point = input[i].point * rng.uniform(0.99, 1.01)
            input[i].x += rng.uniform(-0.2, 0.2)
            input[i].y += rng.uniform(-0.2, 0.2)
        }
        let decision = update(&consensus, input)
        XCTAssertEqual(decision.delta ?? 0, 0.08, accuracy: 0.006)
        XCTAssertEqual(consensus.rotatingIDs, Set(0..<12))
    }

    func testNoConsensusRejectsImpossibleMatchesInsteadOfTreatingThemAsAPause() {
        var consensus = RotationConsensus()
        update(&consensus, matches(delta: 0.1))
        var input = matches(delta: 0)
        for i in input.indices {
            input[i].x += 300
            input[i].y -= 300
        }
        XCTAssertNil(update(&consensus, input).delta)
        XCTAssertTrue(consensus.rotatingIDs.isEmpty)
    }

    func testMissingEvidenceKeepsMembershipButRetiredIdentitiesDisappear() {
        var consensus = RotationConsensus()
        update(&consensus, matches(delta: 0.1))
        let result = consensus.update(
            [], axis: axis, cameraToWorld: .identity, intrinsics: intrinsics, retaining: Set(0..<6))
        XCTAssertNil(result.delta)
        XCTAssertEqual(consensus.rotatingIDs, Set(0..<6))
    }

    func testUnevenRadiiSpeedSweepPreservesAnchoredAngleAcrossThePixelNoiseFloor() {
        var consensus = RotationConsensus()
        var tracker = AngleTracker()
        var theta = 0.0
        var lostMembership = false
        let ramp = (0...80).map { Double($0) * 0.001 }
        let speeds = [0.1] + ramp + ramp.reversed() + ramp.map { -$0 }
        for (frame, delta) in speeds.enumerated() {
            let pairs =
                (0..<12).map { id in
                    let radius = id < 3 ? 0.28 : 0.06 + Double(id) * 0.008
                    return match(
                        id: id, before: point(id, theta: theta, radius: radius),
                        after: point(id, theta: theta + delta, radius: radius))
                } + Array(matches(delta: 0).dropFirst(12))
            update(&consensus, pairs)
            theta += delta
            lostMembership = lostMembership || consensus.rotatingIDs.count < 12
            tracker.removeAllExcept(ids: consensus.rotatingIDs)
            let observations = pairs.filter { consensus.rotatingIDs.contains($0.id) }.map { pair in
                let radius = pair.id < 3 ? 0.28 : 0.06 + Double(pair.id) * 0.008
                let (phi, r, _) = axis.cylindrical(point(pair.id, theta: theta, radius: radius))
                return AngleObservation(id: pair.id, phi: phi, radius: r)
            }
            let result = tracker.update(observations)
            if frame > 0 { XCTAssertTrue(result.ok) }
            XCTAssertEqual(tracker.theta, theta - 0.1, accuracy: 1e-8)
        }
        XCTAssertTrue(lostMembership)
        XCTAssertEqual(consensus.rotatingIDs, Set(0..<12))
    }

    func testThreeTurnsWithBackgroundTakeoverAndAnchoredAngle() {
        var consensus = RotationConsensus()
        var tracker = AngleTracker()
        var theta = 0.0
        var measured = 0
        var frozenTheta = 0.0
        for frame in 0..<240 {
            let delta = frame < 10 || (80..<110).contains(frame) ? 0 : 0.1
            var pairs = matches(delta: delta, theta: theta)
            if frame == 140 { frozenTheta = theta }
            if frame >= 140 {
                for id in 0..<4 {
                    let p = point(id, theta: frozenTheta)
                    pairs[id] = match(id: id, before: p, after: p)
                }
            }
            update(&consensus, pairs)
            theta += delta
            if frame >= 140 { XCTAssertTrue(consensus.rotatingIDs.isDisjoint(with: Set(0..<4))) }
            tracker.removeAllExcept(ids: consensus.rotatingIDs)
            let observations = pairs.filter { consensus.rotatingIDs.contains($0.id) }.map { pair in
                let actualTheta = frame >= 140 && pair.id < 4 ? frozenTheta : theta
                let (phi, radius, _) = axis.cylindrical(point(pair.id, theta: actualTheta))
                return AngleObservation(id: pair.id, phi: phi, radius: radius)
            }
            let result = tracker.update(observations)
            if result.ok { measured += 1 }
            if frame % 10 == 0 { tracker.rebase(observations) }
        }
        XCTAssertGreaterThan(measured, 200)
        XCTAssertEqual(tracker.theta, theta - 0.1, accuracy: 1e-8)
        XCTAssertGreaterThan(theta, 6 * .pi)
    }
}
