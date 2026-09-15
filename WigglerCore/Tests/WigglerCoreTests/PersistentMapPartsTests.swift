import XCTest

@testable import WigglerCore

/// Controls on the parts the persistent map is built from, where an engine run would only show their sum.
final class PersistentMapPartsTests: XCTestCase {
    private let intrinsics = CameraIntrinsics(fx: 420, fy: 420, cx: 240, cy: 180)

    private func view(eye: V3 = .zero) -> PinholeProjection {
        PinholeProjection(
            cameraToWorld: RigidTransform(rotation: .identity, translation: eye), intrinsics: intrinsics,
            width: 480, height: 360)
    }

    /// The patch grid is the exact image of the plane, not a tangent approximation of it: every sample must land
    /// where projecting that plane point directly puts it, at any tilt.
    func testPatchGridIsTheExactImageOfThePlane() {
        let sampler = SurfacePatchSampler(side: 9, halfSize: 0.018)
        let view = view()
        let center = V3(0.05, -0.03, -0.4)
        for tilt in [0.0, 0.4, 0.9, 1.2] {
            let u = V3(cos(tilt), 0, sin(tilt)) * sampler.halfSize
            let v = V3(0, cos(tilt), sin(tilt) * 0.5) * sampler.halfSize
            guard let warp = sampler.warp(center: center, u: u, v: v, view: view) else {
                return XCTFail("no grid at tilt \(tilt)")
            }
            var worst = 0.0
            for j in 0..<9 {
                for i in 0..<9 {
                    let a = 2 * Double(i) / 8 - 1, b = 2 * Double(j) / 8 - 1
                    let direct = view.project(center + u * a + v * b)!
                    let grid = (
                        x: (warp.nx + warp.nxu * Double(i) + warp.nxv * Double(j))
                            / (warp.d + warp.du * Double(i) + warp.dv * Double(j)) + warp.cx,
                        y: (warp.ny + warp.nyu * Double(i) + warp.nyv * Double(j))
                            / (warp.d + warp.du * Double(i) + warp.dv * Double(j)) + warp.cy
                    )
                    worst = max(worst, max(abs(direct.x - grid.x), abs(direct.y - grid.y)))
                }
            }
            XCTAssertLessThan(worst, 1e-9, "grid departs from the projection at tilt \(tilt)")
        }
    }

    /// Whatever way the patch is turned, it is sampled when its whole quadrilateral is inside the image and refused
    /// when any corner is not: the bounds are those of the shape, not of its first sample.
    func testPatchBoundsFollowTheTurnedQuadrilateral() {
        let sampler = SurfacePatchSampler(side: 9, halfSize: 0.03)
        let view = view()
        let image = GrayImage(width: 480, height: 360, fill: 0.5)
        var buffer = [Float](repeating: 0, count: sampler.count)
        var inside = 0, outside = 0
        for step in 0..<24 {
            let spin = Double(step) * .pi / 12
            let u = V3(cos(spin), sin(spin), 0) * sampler.halfSize
            let v = V3(-sin(spin), cos(spin), 0) * sampler.halfSize
            for x in [0.02, 0.1, 0.28] {
                let center = V3(x, -0.13, -0.4)
                guard let warp = sampler.warp(center: center, u: u, v: v, view: view) else { continue }
                var corners: [(Double, Double)] = []
                for (i, j) in [(0, 0), (8, 0), (8, 8), (0, 8)] {
                    let d = warp.d + warp.du * Double(i) + warp.dv * Double(j)
                    corners.append(
                        (
                            (warp.nx + warp.nxu * Double(i) + warp.nxv * Double(j)) / d + warp.cx,
                            (warp.ny + warp.nyu * Double(i) + warp.nyv * Double(j)) / d + warp.cy
                        ))
                }
                let fits = corners.allSatisfy { $0.0 >= 0 && $0.1 >= 0 && $0.0 <= 479 && $0.1 <= 359 }
                // A flat image has no contrast, so a fitting patch is refused for that reason and no other.
                XCTAssertEqual(sampler.fill(&buffer, from: image, warp: warp), false)
                let textured = GrayImage(
                    width: 480, height: 360,
                    pixels: (0..<480 * 360).map { Float(($0 % 480 + $0 / 480) % 7) / 7 })
                XCTAssertEqual(sampler.fill(&buffer, from: textured, warp: warp), fits, "corners \(corners)")
                if fits { inside += 1 } else { outside += 1 }
            }
        }
        XCTAssertGreaterThan(inside, 20, "no orientation was accepted")
        XCTAssertGreaterThan(outside, 5, "no orientation was refused")
    }

    /// A rival angle is another explanation, not a shoulder of the accepted one, and on a full circle the seam is
    /// not a rival at all. An edge of a truncated search is a mode that search did not resolve.
    func testRivalsAreOtherModesAndNotShouldersOrTheSeam() {
        let broad = [1.0, 3.0, 6.0, 9.0, 10.0, 9.0, 6.0, 3.0, 1.0]
        XCTAssertEqual(RigidRotationMap.rivalRatio(broad, best: 4, periodic: false, separation: 3), 0, accuracy: 0)
        let twoModes = [1.0, 3.0, 6.0, 9.0, 10.0, 9.0, 6.0, 3.0, 8.0]
        XCTAssertEqual(
            RigidRotationMap.rivalRatio(twoModes, best: 4, periodic: false, separation: 3), 0.8, accuracy: 1e-12)
        let seam = [10.0, 9.0, 6.0, 3.0, 1.0, 3.0, 6.0, 9.0]
        XCTAssertEqual(RigidRotationMap.rivalRatio(seam, best: 0, periodic: true, separation: 3), 0, accuracy: 0)
        let rising = [10.0, 8.0, 3.0, 1.0, 2.0, 8.0, 9.0]
        XCTAssertEqual(
            RigidRotationMap.rivalRatio(rising, best: 0, periodic: false, separation: 3), 0.9, accuracy: 1e-12)
    }

    /// Taking a point's positions back by the body's angle must land them on one place. A tangential slide keeps a
    /// fixed radius and height and still fails this, which is why the invariants of a circle are not enough.
    func testRigidResidualSeparatesASlideFromTheBodysRotation() {
        let axis = Axis(origin: V3(0, 0, -0.5), direction: V3(0, 1, 0))
        let rate = 1.0
        func trace(_ move: (Double) -> V3) -> (
            spread: Double, worldSpread: Double, travel: Double, span: Double, samples: Int
        )? {
            let points = TrackedWorldPoints()
            for step in 0..<12 {
                let time = Double(step) / 30
                let world = move(time)
                let projected = view().project(world)!
                let depth = DepthMap(
                    width: 8, height: 6, depth: [Float](repeating: Float(projected.range), count: 48),
                    confidence: nil)
                points.observe(
                    [
                        MotionLocator.Point(
                            id: 1, x: Float(projected.x), y: Float(projected.y), previousX: 0, previousY: 0)
                    ],
                    depth: depth, view: view(), frame: step, time: time, minConfidence: 0)
            }
            return points.rigidResidual(id: 1, axis: axis, angleAt: { $0 * rate })
        }
        let radius = 0.1
        guard
            let rigid = trace({ time in
                axis.origin + axis.e1 * (radius * cos(rate * time)) + axis.e2 * (radius * sin(rate * time))
            }),
            let sliding = trace({ time in axis.origin + axis.e1 * radius + axis.e2 * (0.01 * time) })
        else { return XCTFail("no residual") }
        XCTAssertLessThan(rigid.spread, 0.0005, "the body's own point was called a stranger")
        XCTAssertLessThan(rigid.spread, rigid.worldSpread, "the body's rotation should explain its own point best")
        XCTAssertGreaterThan(sliding.spread, 0.004, "a tangential slide passed as the body's rotation")
        XCTAssertGreaterThan(sliding.spread, sliding.worldSpread, "standing still explains a slide better")
        print(
            "residual: rigid \(rigid.spread) vs world \(rigid.worldSpread) m,"
                + " sliding \(sliding.spread) vs world \(sliding.worldSpread) m over \(sliding.span) s")
    }

    /// A slide too short to leave any absolute tolerance behind: over a tenth of a second at this radius it sits well
    /// inside the tolerance a real sensor demands, and only the comparison of the two models rejects it.
    func testAShortSlideIsRejectedByComparisonNotByTolerance() {
        let axis = Axis(origin: V3(0, 0, -0.5), direction: V3(0, 1, 0))
        let points = TrackedWorldPoints()
        for step in 0..<7 {
            let time = Double(step) / 60
            let world = axis.origin + axis.e1 * 0.1 + axis.e2 * (0.1 * time)
            let projected = view().project(world)!
            let depth = DepthMap(
                width: 8, height: 6, depth: [Float](repeating: Float(projected.range), count: 48), confidence: nil)
            points.observe(
                [MotionLocator.Point(id: 1, x: Float(projected.x), y: Float(projected.y), previousX: 0, previousY: 0)],
                depth: depth, view: view(), frame: step, time: time, minConfidence: 0)
        }
        guard let fit = points.rigidResidual(id: 1, axis: axis, angleAt: { $0 * 3 }) else {
            return XCTFail("no residual")
        }
        XCTAssertLessThan(fit.spread, 0.012, "this slide is meant to sit inside the absolute tolerance")
        XCTAssertGreaterThan(fit.spread, fit.worldSpread, "the rotation must not explain this slide best")
        print("short slide: canonical \(fit.spread) m, world \(fit.worldSpread) m, travel \(fit.travel * 180 / .pi)°")
    }

    /// Chords of one turning body among chords of something else that merely moves: the consensus must keep the
    /// body's and leave the strangers out, which a reweighted fit alone cannot do at this contamination.
    func testConsensusKeepsTheChordsOfTheTurningBody() {
        let axis = Axis(origin: V3(0, 0, -0.6), direction: V3(0, 1, 0))
        var rng = LCG(seed: 5)
        var chords: [ChordConstraint] = []
        var belongs: [Bool] = []
        for index in 0..<300 {
            let radius = rng.uniform(0.05, 0.15)
            let phi = rng.uniform(0, 2 * .pi)
            let height = rng.uniform(-0.05, 0.05)
            let start =
                axis.origin + axis.direction * height + axis.e1 * (radius * cos(phi))
                + axis.e2 * (radius * sin(phi))
            let turned = axis.origin + rotatedAbout(axis.direction, by: 0.3, start - axis.origin)
            chords.append(
                ChordConstraint(midpoint: (start + turned) * 0.5, chord: turned - start, frame: index))
            belongs.append(true)
        }
        for index in 0..<200 {
            let start = V3(rng.uniform(-0.3, 0.3), rng.uniform(-0.2, 0.2), rng.uniform(-0.8, -0.4))
            let slide = V3(0.05, 0.01, 0)
            chords.append(ChordConstraint(midpoint: start + slide * 0.5, chord: slide, frame: 300 + index))
            belongs.append(false)
        }
        let selected = Set(ChordConsensus().select(chords).map { $0.frame })
        let kept = selected.filter { $0 < 300 }.count
        let strangers = selected.filter { $0 >= 300 }.count
        XCTAssertGreaterThan(kept, 240, "kept only \(kept) of the body's 300 chords")
        XCTAssertLessThan(strangers, 40, "kept \(strangers) of the 200 strangers")
        print("consensus: kept \(kept)/300 of the body, \(strangers)/200 strangers")
    }

    /// Deterministic contrast everywhere, so a patch taken anywhere in the image has something to describe.
    private func texturedImage() -> GrayImage {
        var pixels = [Float](repeating: 0, count: 480 * 360)
        for y in 0..<360 {
            for x in 0..<480 {
                pixels[y * 480 + x] = Float(0.5 + 0.35 * sin(0.7 * Double(x)) * sin(1.1 * Double(y)))
            }
        }
        return GrayImage(width: 480, height: 360, pixels: pixels)
    }

    /// A body of landmarks made the way the engine makes them: from tracked points, each one carrying the identity
    /// of the track it was born on.
    private func mappedBody(axis: Axis, view: PinholeProjection, range: Double) -> (
        map: RigidRotationMap, image: GrayImage, depth: DepthMap
    ) {
        let map = RigidRotationMap(
            axis: axis, sampler: SurfacePatchSampler(side: 9, halfSize: 0.018))
        let image = texturedImage()
        let depth = DepthMap(
            width: 96, height: 72, depth: [Float](repeating: Float(range), count: 96 * 72), confidence: nil)
        var observations: [TrackedWorldPoints.Observation] = []
        var id = 1
        for column in [140.0, 165, 190, 290, 315, 340] {
            for row in [110.0, 145, 180, 215, 250] {
                observations.append(
                    TrackedWorldPoints.Observation(
                        id: id, x: column, y: row, world: view.unproject(x: column, y: row, range: range),
                        range: range, moving: true, displacement: 20))
                id += 1
            }
        }
        _ = map.insert(
            observations, theta: 0, image: image, depth: depth, view: view, frame: 0, minConfidence: 0, limit: 100,
            occupied: [])
        return (map, image, depth)
    }

    /// Where the tracks say the landmarks are when the body has turned by `theta`, one observation per landmark
    /// under the identity it was born with.
    private func tracks(of map: RigidRotationMap, at theta: Double, view: PinholeProjection)
        -> [TrackedWorldPoints.Observation]
    {
        map.landmarks.indices.compactMap { index in
            guard let pixel = map.pixel(of: index, at: theta, view: view) else { return nil }
            return TrackedWorldPoints.Observation(
                id: map.landmarks[index].trackID, x: pixel.x, y: pixel.y, world: .zero, range: 0, moving: true,
                displacement: 0)
        }
    }

    /// The tracks decide the angle. The angle carried into a frame chooses only which turn the answer belongs to,
    /// so a prediction badly overtaken by the body must not drag the measurement towards itself.
    func testTheTracksMeasureTheAngleAndThePredictionOnlyChoosesTheTurn() {
        let axis = Axis(origin: V3(0, 0, -0.4), direction: V3(0, 1, 0))
        let view = view()
        let body = mappedBody(axis: axis, view: view, range: 0.4)
        XCTAssertGreaterThan(body.map.count, 20, "the map was not built from the tracks")
        let truth = 0.0
        let observations = tracks(of: body.map, at: truth, view: view)
        for predicted in [truth + 0.7, truth - 0.7, truth + 2 * Double.pi + 0.7] {
            let fit = body.map.measureByTrack(
                observations: observations, predicted: predicted, image: body.image, view: view, depth: nil,
                minConfidence: 0)
            guard let measured = fit.measurement else {
                return XCTFail("no fit from \(fit.bound) bound tracks at predicted \(predicted)")
            }
            let turns = ((predicted - truth) / (2 * Double.pi)).rounded()
            XCTAssertEqual(
                measured.theta, truth + turns * 2 * Double.pi, accuracy: 0.002,
                "the fit followed the prediction \(predicted)")
        }
    }

    /// Half a turn away from the angle carried in, the two representatives of the answer lie either side of the
    /// seam. Here the search's own grid node lies on the near side of it and the angle the refinement reaches lies
    /// on the far side, so settling the turn before the refinement would leave the answer a whole turn out. It is
    /// settled after, on the angle actually reached, and comes back as the representative nearest the angle carried
    /// in.
    func testAnAngleMeasuredAcrossTheSeamComesBackOnTheNearestTurn() {
        let axis = Axis(origin: V3(0, 0, -0.4), direction: V3(0, 1, 0))
        let view = view()
        let body = mappedBody(axis: axis, view: view, range: 0.4)
        let step = 2 * Double.pi / (2 * Double.pi / body.map.coarseStep).rounded()
        let truth = 0.019
        let predicted = truth + Double.pi + 0.002
        let node = (truth / step).rounded() * step
        XCTAssertGreaterThan(
            abs(truth - predicted), Double.pi, "the fixture does not put the refined angle past the seam")
        XCTAssertLessThan(
            abs(RigidRotationMap.lifted(node, near: predicted) - predicted), Double.pi,
            "the fixture does not start the refinement inside the seam")
        let observations = tracks(of: body.map, at: truth, view: view)
        guard
            let measured = body.map.measureByTrack(
                observations: observations, predicted: predicted, image: body.image, view: view, depth: nil,
                minConfidence: 0
            ).measurement
        else { return XCTFail("no fit across the seam") }
        XCTAssertEqual(
            measured.theta, truth + 2 * Double.pi, accuracy: 0.003, "the answer came back on the far turn")
        XCTAssertLessThanOrEqual(
            abs(measured.theta - predicted), Double.pi, "the answer is more than half a turn from the prediction")
    }

    /// One track carries one material point. However far a track's own landmark has moved away from it, that track
    /// may not make a second landmark, or the same pixel would enter the fit twice and lend it support twice.
    func testOneTrackNeverMakesASecondLandmark() {
        let axis = Axis(origin: V3(0, 0, -0.4), direction: V3(0, 1, 0))
        let view = view()
        let body = mappedBody(axis: axis, view: view, range: 0.4)
        let before = body.map.count
        let carried = body.map.landmarks[0].birthTrackID
        let elsewhere = TrackedWorldPoints.Observation(
            id: carried, x: 380, y: 300, world: view.unproject(x: 380, y: 300, range: 0.4), range: 0.4,
            moving: true, displacement: 20)
        let grown = body.map.insert(
            [elsewhere], theta: 0, image: body.image, depth: body.depth, view: view, frame: 1, minConfidence: 0,
            limit: 10, occupied: [])
        XCTAssertEqual(grown.added, 0, "a track already carrying a landmark made another one")
        XCTAssertEqual(body.map.count, before, "the map grew on one track twice")
        XCTAssertEqual(
            body.map.landmarks.filter { $0.birthTrackID == carried }.count, 1, "one track made two landmarks")
        // And still not after its claim has been let go: a released track is a track without a landmark to carry,
        // not a track free to anchor the same material again at today's angle.
        let observations = tracks(of: body.map, at: 0, view: view)
        guard
            let measured = body.map.measureByTrack(
                observations: observations, predicted: 0, image: body.image, view: view, depth: nil,
                minConfidence: 0
            ).measurement
        else { return XCTFail("no fit to release against") }
        _ = body.map.releaseTracksThatDisagree(with: [], measurement: measured)
        XCTAssertEqual(body.map.landmarks.filter { $0.trackID != 0 }.count, 0, "a track survived every track ending")
        let again = body.map.insert(
            [elsewhere], theta: 0, image: body.image, depth: body.depth, view: view, frame: 2, minConfidence: 0,
            limit: 10, occupied: [])
        XCTAssertEqual(again.added, 0, "a released track anchored the same material again")
        XCTAssertEqual(body.map.count, before, "the map grew on a released track")
    }

    /// A claim of identity has to keep agreeing with the rigid body. A track whose pixel contradicts where the map
    /// puts its landmark is let go, as is one that has simply ended — and letting go is all that happens: no free
    /// landmark is bound to a nearby corner, since proximity is not proof of being the same material point.
    func testATrackIsLetGoWhenItEndsOrContradictsTheBody() {
        let axis = Axis(origin: V3(0, 0, -0.4), direction: V3(0, 1, 0))
        let view = view()
        let body = mappedBody(axis: axis, view: view, range: 0.4)
        let truth = 0.0
        var observations = tracks(of: body.map, at: truth, view: view)
        let victim = observations[3].id
        observations[3].x += 40
        let fit = body.map.measureByTrack(
            observations: observations, predicted: truth, image: body.image, view: view, depth: nil,
            minConfidence: 0)
        guard let measured = fit.measurement else { return XCTFail("no fit from \(fit.bound) bound tracks") }
        XCTAssertEqual(measured.theta, truth, accuracy: 0.002, "one wandering track moved the angle")
        let release = body.map.releaseTracksThatDisagree(with: observations, measurement: measured)
        XCTAssertEqual(release.bound, fit.bound - 1, "the contradicting track was kept")
        XCTAssertEqual(release.contradicted, 1, "\(release.contradicted) tracks reported as contradicting")
        XCTAssertEqual(release.ended, 0, "a live track was reported as ended")
        XCTAssertEqual(
            body.map.landmarks.filter { $0.trackID == victim }.count, 0, "the contradicting track still claims one")
        let after = body.map.releaseTracksThatDisagree(with: [], measurement: measured)
        XCTAssertEqual(after.bound, 0, "tracks that have ended still claim landmarks")
        XCTAssertEqual(after.ended, fit.bound - 1, "\(after.ended) tracks reported as ended")
        XCTAssertEqual(body.map.count, body.map.landmarks.count, "letting a track go removed a landmark")
    }

    /// A landmark earns its place by how often it is seen per frame of its existence, so a map full of landmarks
    /// nobody has seen for a long time still makes room for a surface just discovered.
    func testRetentionPrefersASeenSurfaceOverAForgottenOne() {
        let map = RigidRotationMap(
            axis: Axis(origin: .zero, direction: V3(0, 1, 0)), sampler: SurfacePatchSampler(side: 9, halfSize: 0.018))
        func landmark(id: Int, observations: Int, firstFrame: Int, lastSeen: Int) -> RigidRotationMap.Landmark {
            RigidRotationMap.Landmark(
                id: id, canonical: V3(0.1, 0, 0), tangentU: V3(0, 0.018, 0), tangentV: V3(0, 0, 0.018),
                descriptor: [], birthTrackID: 0, trackID: 0, firstFrame: firstFrame, lastSeenFrame: lastSeen,
                observations: observations,
                radius: 0.1, height: 0)
        }
        let stale = landmark(id: 1, observations: 2, firstFrame: 0, lastSeen: 20)
        let fresh = landmark(id: 2, observations: 1, firstFrame: 500, lastSeen: 500)
        let faithful = landmark(id: 3, observations: 120, firstFrame: 0, lastSeen: 498)
        XCTAssertGreaterThan(map.retention(fresh, frame: 500), map.retention(stale, frame: 500))
        XCTAssertGreaterThan(map.retention(faithful, frame: 500), map.retention(stale, frame: 500))
    }
}
