import XCTest

@testable import WigglerCore

final class PointDiagnosticsTests: XCTestCase {
    func testPointIdentityHistoryAndDepthDecisions() throws {
        let recorder = try diagnosticsRecorder()
        let tracks = PointTracks()
        let plain = PointTracks()
        var input = blankFrame(
            depth: DepthMap(width: 5, height: 5, depth: Array(repeating: 1, count: 25), confidence: nil))
        for frame in 1...5 {
            let point = MotionLocator.Point(
                id: 71, x: Float(10 + frame), y: 12, previousX: Float(9 + frame), previousY: 12)
            tracks.update([point], diagnostics: recorder)
            plain.update([point])
            if frame > 1 { input.depth?.depth = Array(repeating: 1.2, count: 25) }
            tracks.sampleDepth(
                input, pose: .identity, frame: frame, minConfidence: 1, maxJumpFraction: 0.08,
                historyLength: 90, diagnostics: recorder)
            plain.sampleDepth(
                input, pose: .identity, frame: frame, minConfidence: 1, maxJumpFraction: 0.08, historyLength: 90)
            let observed = try XCTUnwrap(recorder.value.depth.last)
            XCTAssertEqual(observed.id, 71)
            XCTAssertEqual(observed.neighborhood?.validCount, 9)
            XCTAssertEqual(observed.neighborhood?.pixels.count, 9)
            XCTAssertEqual(observed.outcome, frame == 1 || frame == 5 ? "accepted" : "depthJumpRejected")
            XCTAssertEqual(tracks.diagnosticState().first?.lastDepth, plain.diagnosticState().first?.lastDepth)
        }
        XCTAssertEqual(recorder.value.trackUpdates.first?.addedIDs, [71])
        XCTAssertTrue(
            recorder.value.trackUpdates.dropFirst().allSatisfy { $0.addedIDs.isEmpty && $0.removedIDs.isEmpty })
        XCTAssertEqual(tracks.diagnosticState().first?.age, 4)
        XCTAssertEqual(tracks.diagnosticHistories().first?.samples.map { $0.frame }, [1, 5])
        let chords = tracks.chordConstraints(frame: 5, minLength: 0.02, maxFrames: 45, stride: 3, diagnostics: recorder)
        let expected = plain.chordConstraints(frame: 5, minLength: 0.02, maxFrames: 45, stride: 3)
        XCTAssertEqual(chords.count, expected.count)
        XCTAssertEqual(chords.first?.chord, expected.first?.chord)
        XCTAssertEqual(recorder.value.freshChords.first?.trackID, 71)
        XCTAssertEqual(recorder.value.freshChords.first?.start.frame, 1)
        XCTAssertEqual(recorder.value.freshChords.first?.end.frame, 5)
        tracks.update([], diagnostics: recorder)
        XCTAssertEqual(recorder.value.trackUpdates.last?.removedIDs, [71])
    }

    func testDepthNonFinitesRoundTripWithCallerStrategy() throws {
        let recorder = try diagnosticsRecorder()
        let tracks = PointTracks()
        tracks.update([MotionLocator.Point(id: 9, x: 12, y: 12, previousX: 12, previousY: 12)])
        let depths: [Float] = [.nan, .infinity, -.infinity, 1, 1, 1, 1, 1, 1]
        let depth = DepthMap(width: 3, height: 3, depth: depths, confidence: [2, 2, 2, 0, 2, 2, 2, 2, 2])
        tracks.sampleDepth(
            blankFrame(depth: depth), pose: .identity, frame: 1, minConfidence: 1,
            maxJumpFraction: 0.08, historyLength: 90, diagnostics: recorder)
        let observation = try XCTUnwrap(recorder.value.depth.last)
        XCTAssertEqual(observation.outcome, "accepted")
        XCTAssertEqual(observation.neighborhood?.validCount, 5)
        XCTAssertTrue(observation.neighborhood?.pixels[0].depth.isNaN == true)
        XCTAssertThrowsError(try JSONEncoder().encode(recorder.value))
        let encoder = JSONEncoder()
        encoder.nonConformingFloatEncodingStrategy = .convertToString(
            positiveInfinity: "Infinity", negativeInfinity: "-Infinity", nan: "NaN")
        let decoder = JSONDecoder()
        decoder.nonConformingFloatDecodingStrategy = .convertFromString(
            positiveInfinity: "Infinity", negativeInfinity: "-Infinity", nan: "NaN")
        let data = try encoder.encode(recorder.value)
        let decoded = try decoder.decode(EngineDiagnostics.self, from: data)
        let pixels = try XCTUnwrap(decoded.depth.last?.neighborhood?.pixels)
        XCTAssertTrue(pixels[0].depth.isNaN)
        XCTAssertEqual(pixels[1].depth, .infinity)
        XCTAssertEqual(pixels[2].depth, -.infinity)
        XCTAssertFalse(pixels[3].accepted)
    }

    func testDepthNeighborhoodRejectionEvidenceMatchesSampling() throws {
        let cases: [([Float], String)] = [
            ([1, 1, 1, 1, 1, 1, 1, 1, 2], "depthEdge"),
            ([0, 0, 0, 0, 0, 0, 1, 1, 1], "insufficientValidSamples"),
        ]
        for (values, outcome) in cases {
            let depth = DepthMap(width: 3, height: 3, depth: values, confidence: nil)
            var evidence: EngineDiagnostics.DepthNeighborhood?
            let result = depth.sample(u: 0.5, v: 0.5, minConfidence: 1) { evidence = $0 }
            XCTAssertEqual(result, depth.sample(u: 0.5, v: 0.5, minConfidence: 1))
            XCTAssertNil(result)
            XCTAssertEqual(evidence?.outcome, outcome)
        }
    }

    func testFreshChordContradictionsExplainInvalidationAndRestart() throws {
        let recorder = try diagnosticsRecorder()
        let axis = Axis(origin: .zero, direction: V3(0, 1, 0))
        let chords = (0..<8).map { _ in ChordConstraint(midpoint: V3(0.1, 0, 0), chord: V3(0, 0.1, 0), frame: 10) }
        var observed = AxisStability()
        var plain = AxisStability()
        for frame in 1...3 {
            observed.observe(frame: frame, consistent: true)
            plain.observe(frame: frame, consistent: true)
        }
        for batch in 1...3 {
            let restart = observed.observe(
                chords: chords, axis: axis, tolerance: 0.01, required: 3, diagnostics: recorder)
            XCTAssertEqual(restart, plain.observe(chords: chords, axis: axis, tolerance: 0.01, required: 3))
            let evidence = try XCTUnwrap(recorder.value.chordBatch)
            XCTAssertEqual(evidence.confirmationsBefore, batch == 1 ? 3 : 0)
            XCTAssertEqual(evidence.confirmationsAfter, 0)
            XCTAssertEqual(evidence.inconsistentBefore, batch - 1)
            XCTAssertEqual(evidence.inconsistentAfter, batch)
            XCTAssertEqual(evidence.restartRequired, batch == 3)
            XCTAssertEqual(evidence.badCount, 8)
            XCTAssertTrue(evidence.rejects)
            XCTAssertEqual(evidence.residuals[0].residualMeters, 0.1)
            XCTAssertEqual(observed.isStable(required: 3), plain.isStable(required: 3))
        }
        XCTAssertFalse(observed.observe(chords: [], axis: axis, tolerance: 0.01, required: 3, diagnostics: recorder))
        XCTAssertEqual(recorder.value.chordBatch?.restartRequired, false)
        XCTAssertEqual(recorder.value.chordBatch?.inconsistentAfter, 3)
    }
}
