import XCTest

@testable import WigglerCore

final class PointTracksTests: XCTestCase {
    private let marker = Marker(x: 32, y: 32, radius: 20)
    private let axis = Axis(origin: .zero, direction: V3(0, 1, 0))

    private func input(depth: Float? = 1, confidence: UInt8 = 2) -> FrameInput {
        var rng = LCG(seed: 51)
        let image = GrayImage(width: 64, height: 64, pixels: (0..<4096).map { _ in Float(rng.next()) })
        let map = depth.map {
            DepthMap(
                width: 64, height: 64, depth: [Float](repeating: $0, count: 4096),
                confidence: [UInt8](repeating: confidence, count: 4096))
        }
        return FrameInput(
            image: image, intrinsics: CameraIntrinsics(fx: 64, fy: 64, cx: 32, cy: 32),
            cameraToWorld: .identity, poseValid: true, depth: map, timestamp: 0)
    }

    private func selectedPoints(firstID: Int = 1) -> [MotionLocator.Point] {
        let points = CornerDetector().detect(
            in: input().image, centerX: marker.x, centerY: marker.y,
            radius: marker.radius, exclude: [], maxCount: 4)
        return points.enumerated().map { i, p in
            MotionLocator.Point(id: firstID + i, x: p.0, y: p.1, previousX: p.0, previousY: p.1)
        }
    }

    private func populatedTracks() -> PointTracks {
        let tracks = PointTracks()
        tracks.update(selectedPoints())
        XCTAssertEqual(tracks.count, 4)
        return tracks
    }

    private func sample(
        _ tracks: PointTracks, frame: Int, depth: Float? = 1, confidence: UInt8 = 2,
        x: Double = 0, historyLength: Int = 90
    ) {
        var pose = RigidTransform.identity
        pose.translation = V3(x, 0, 0)
        tracks.sampleDepth(
            input(depth: depth, confidence: confidence), pose: pose, frame: frame,
            minConfidence: 1, maxJumpFraction: 0.08, historyLength: historyLength)
    }

    func testDepthDropoutConfidenceAndPersistentJump() {
        let tracks = populatedTracks()
        sample(tracks, frame: 0)
        let original = tracks.project(around: axis)
        XCTAssertEqual(original.count, 4)
        sample(tracks, frame: 1, depth: nil)
        XCTAssertTrue(tracks.observations(around: axis).isEmpty)
        sample(tracks, frame: 2, confidence: 0)
        XCTAssertTrue(tracks.observations(around: axis).isEmpty)
        for frame in 3...5 {
            sample(tracks, frame: frame, depth: 2)
            XCTAssertTrue(tracks.observations(around: axis).isEmpty)
        }
        sample(tracks, frame: 6, depth: 2)
        let jumped = tracks.project(around: axis)
        XCTAssertEqual(jumped.map { $0.id }, original.map { $0.id })
        for (a, b) in zip(original, jumped) { XCTAssertEqual(b.radius, 2 * a.radius, accuracy: 1e-12) }
    }

    func testChordStrideAndCalibrationReset() {
        let tracks = populatedTracks()
        sample(tracks, frame: 1)
        sample(tracks, frame: 4, x: 0.1)
        let chords = tracks.chordConstraints(frame: 4, minLength: 0.02, maxFrames: 45, stride: 3)
        XCTAssertEqual(chords.count, 4)
        for c in chords { XCTAssertEqual(c.chord.x, 0.1, accuracy: 1e-12) }
        XCTAssertTrue(tracks.chordConstraints(frame: 4, minLength: 0.02, maxFrames: 45, stride: 3).isEmpty)
        sample(tracks, frame: 5, x: 0.2)
        XCTAssertTrue(tracks.chordConstraints(frame: 5, minLength: 0.02, maxFrames: 45, stride: 3).isEmpty)
        tracks.resetChordCadence()
        XCTAssertEqual(tracks.chordConstraints(frame: 5, minLength: 0.02, maxFrames: 45, stride: 3).count, 4)
        XCTAssertEqual(tracks.observations(around: axis).count, 4)
    }

    func testSampleHistoryAndChordAgeAreBounded() {
        let tracks = populatedTracks()
        for frame in 1...3 { sample(tracks, frame: frame, x: Double(frame) * 0.1, historyLength: 2) }
        XCTAssertTrue(tracks.chordConstraints(frame: 3, minLength: 0.02, maxFrames: 0, stride: 1).isEmpty)
        let chords = tracks.chordConstraints(frame: 3, minLength: 0.02, maxFrames: 45, stride: 1)
        XCTAssertEqual(chords.count, 4)
        for c in chords { XCTAssertEqual(c.chord.x, 0.1, accuracy: 1e-12) }
    }

    func testRebasedObservationsDoNotReplaceMeasuredExtent() {
        let tracks = populatedTracks()
        sample(tracks, frame: 1)
        let measured = tracks.project(around: axis)
        let extent = tracks.extent { _ in true }
        let revised = tracks.observations(around: Axis(origin: V3(2, 3, 0), direction: V3(0, 1, 0)))
        XCTAssertEqual(revised.count, measured.count)
        XCTAssertNotEqual(revised.map { $0.radius }, measured.map { $0.radius })
        XCTAssertEqual(tracks.extent { _ in true }.radii, extent.radii)
        XCTAssertEqual(tracks.extent { _ in true }.heights, extent.heights)
        XCTAssertTrue(tracks.extent { _ in false }.radii.isEmpty)
        sample(tracks, frame: 2, depth: nil)
        XCTAssertTrue(tracks.extent { _ in true }.radii.isEmpty)
        XCTAssertEqual(tracks.objectRadius, percentile(extent.radii, 0.8))
    }

    func testSelectedCorrespondencesAndIdentityAfterReset() {
        let tracks = populatedTracks()
        sample(tracks, frame: 0)
        let ids = tracks.ids
        for _ in 0..<3 {
            let pairs = tracks.update(selectedPoints())
            XCTAssertEqual(pairs.before.count, 4)
            XCTAssertEqual(pairs.after.count, 4)
            for (a, b) in zip(pairs.before, pairs.after) {
                XCTAssertEqual(a.0, b.0, accuracy: 1e-5)
                XCTAssertEqual(a.1, b.1, accuracy: 1e-5)
            }
        }
        XCTAssertEqual(tracks.ids, ids)
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in true }.allSatisfy { $0.status == .good })
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in false }.allSatisfy { $0.status == .inconsistent })
        sample(tracks, frame: 1, depth: nil)
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in true }.allSatisfy { $0.status == .noDepth })
        tracks.removeAll()
        XCTAssertEqual(tracks.count, 0)
        tracks.update(selectedPoints(firstID: 10))
        sample(tracks, frame: 2)
        XCTAssertTrue(tracks.observations(around: axis).allSatisfy { $0.id >= 10 })
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in true }.allSatisfy { $0.status == .young })
    }

    func testDeselectedPointsLoseTheirDepthHistory() {
        let tracks = populatedTracks()
        let selected = selectedPoints()
        sample(tracks, frame: 1)
        sample(tracks, frame: 4, x: 0.1)
        tracks.update(Array(selected.prefix(2)))
        XCTAssertEqual(tracks.count, 2)
        XCTAssertEqual(tracks.observations(around: axis).map { $0.id }, Array(selected.prefix(2)).map { $0.id })
        tracks.update(selected)
        sample(tracks, frame: 5, x: 0.2)
        let chords = tracks.chordConstraints(frame: 5, minLength: 0.02, maxFrames: 45, stride: 3)
        XCTAssertEqual(chords.count, 2, "reselected points must start a new depth history")
        for c in chords { XCTAssertEqual(c.chord.x, 0.2, accuracy: 1e-12) }
    }

    func testEmptySelectionClearsPointsAndCorrespondences() {
        let tracks = populatedTracks()
        sample(tracks, frame: 1)
        let pairs = tracks.update([])
        XCTAssertEqual(tracks.count, 0)
        XCTAssertTrue(tracks.ids.isEmpty)
        XCTAssertTrue(pairs.before.isEmpty && pairs.after.isEmpty)
        XCTAssertTrue(tracks.observations(around: axis).isEmpty)
        XCTAssertTrue(tracks.debug(hasAxis: false) { _ in true }.isEmpty)
    }
}
