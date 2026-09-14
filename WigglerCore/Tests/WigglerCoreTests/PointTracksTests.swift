import XCTest
@testable import WigglerCore

final class PointTracksTests: XCTestCase {
    private let marker = Marker(x: 32, y: 32, radius: 20)
    private let axis = Axis(origin: .zero, direction: V3(0, 1, 0))

    private func input(depth: Float? = 1, confidence: UInt8 = 2) -> FrameInput {
        var rng = LCG(seed: 51)
        let image = GrayImage(width: 64, height: 64, pixels: (0..<4096).map { _ in Float(rng.next()) })
        let map = depth.map { DepthMap(width: 64, height: 64, depth: [Float](repeating: $0, count: 4096),
                                      confidence: [UInt8](repeating: confidence, count: 4096)) }
        return FrameInput(image: image, intrinsics: CameraIntrinsics(fx: 64, fy: 64, cx: 32, cy: 32),
                          cameraToWorld: .identity, poseValid: true, depth: map, timestamp: 0)
    }

    private func populatedTracks() -> PointTracks {
        let tracks = PointTracks()
        tracks.replenish(in: input().image, marker: marker, detector: CornerDetector(), targetCount: 4)
        XCTAssertEqual(tracks.count, 4)
        return tracks
    }

    private func sample(_ tracks: PointTracks, frame: Int, depth: Float? = 1, confidence: UInt8 = 2,
                        x: Double = 0, historyLength: Int = 90) {
        var pose = RigidTransform.identity
        pose.translation = V3(x, 0, 0)
        tracks.sampleDepth(input(depth: depth, confidence: confidence), pose: pose, frame: frame,
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
        tracks.resetMotionHistory()
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

    func testTrackingCorrespondencesAndMonotonicIDsAfterReset() {
        let tracks = populatedTracks()
        sample(tracks, frame: 0)
        let ids = tracks.observations(around: axis).map { $0.id }
        let pyramid = Pyramid(image: input().image, levelCount: 1)
        for _ in 0..<3 {
            let pairs = tracks.track(from: pyramid, to: pyramid, marker: marker,
                                     tracker: KLTTracker(), maxResidual: 0.12)
            XCTAssertEqual(pairs.before.count, 4)
            XCTAssertEqual(pairs.after.count, 4)
            for (a, b) in zip(pairs.before, pairs.after) {
                XCTAssertEqual(a.0, b.0, accuracy: 1e-5)
                XCTAssertEqual(a.1, b.1, accuracy: 1e-5)
            }
        }
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in true }.allSatisfy { $0.status == .good })
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in false }.allSatisfy { $0.status == .inconsistent })
        sample(tracks, frame: 1, depth: nil)
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in true }.allSatisfy { $0.status == .noDepth })
        tracks.removeAll()
        XCTAssertEqual(tracks.count, 0)
        tracks.replenish(in: input().image, marker: marker, detector: CornerDetector(), targetCount: 4)
        sample(tracks, frame: 2)
        XCTAssertTrue(tracks.observations(around: axis).allSatisfy { $0.id > ids.max()! })
        XCTAssertTrue(tracks.debug(hasAxis: true) { _ in true }.allSatisfy { $0.status == .young })
        let pairs = tracks.track(from: nil, to: pyramid, marker: marker, tracker: KLTTracker(), maxResidual: 0.12)
        XCTAssertEqual(tracks.count, 0)
        XCTAssertTrue(pairs.before.isEmpty && pairs.after.isEmpty)
    }

    func testStaticPointsAreRemovedOnlyDuringMeasuredRotation() {
        for rotating in [false, true] {
            let tracks = populatedTracks()
            for frame in 0...60 {
                sample(tracks, frame: frame)
                let removed = tracks.removeStatic(theta: rotating ? Double(frame) * 0.01 : 0,
                                                   frame: frame, minLength: 0.02)
                XCTAssertEqual(removed.count, rotating && frame == 60 ? 4 : 0)
            }
            XCTAssertEqual(tracks.count, rotating ? 0 : 4)
        }
    }

    func testMotionHistoryResetAndMissingAnglesPreventStaticRejection() {
        let tracks = populatedTracks()
        for frame in 0..<60 {
            sample(tracks, frame: frame)
            XCTAssertTrue(tracks.removeStatic(theta: Double(frame) * 0.01, frame: frame, minLength: 0.02).isEmpty)
        }
        tracks.resetMotionHistory()
        sample(tracks, frame: 60)
        XCTAssertTrue(tracks.removeStatic(theta: 1, frame: 60, minLength: 0.02).isEmpty)
        for frame in 61...120 {
            sample(tracks, frame: frame)
            XCTAssertTrue(tracks.removeStatic(theta: nil, frame: frame, minLength: 0.02).isEmpty)
        }
        XCTAssertEqual(tracks.count, 4)
    }
}
