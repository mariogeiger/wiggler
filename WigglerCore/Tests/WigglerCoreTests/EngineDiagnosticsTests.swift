import XCTest

@testable import WigglerCore

final class EngineDiagnosticsTests: XCTestCase {
    func testDefaultAPIAndCodableConfig() throws {
        let engine = RotationEngine()
        XCTAssertNil(engine.process(blankFrame()).diagnostics)
        XCTAssertNil(EngineOutput().diagnostics)
        var config = EngineConfig()
        config.chordStride = 7
        config.maxResidual = 0.2
        config.minDepthConfidence = 2
        let encoder = JSONEncoder()
        encoder.outputFormatting = .sortedKeys
        let data = try encoder.encode(config)
        let decoded = try JSONDecoder().decode(EngineConfig.self, from: data)
        XCTAssertEqual(try encoder.encode(decoded), data)
        let json = try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: Any])
        XCTAssertEqual(json.count, Mirror(reflecting: config).children.count)
        XCTAssertEqual(json["chordStride"] as? Int, 7)
    }

    func testCaptureDoesNotChangeOutputsOrResetWarmTracking() throws {
        let scene = SyntheticScene()
        var rng = LCG(seed: 3)
        let plain = RotationEngine()
        let captured = RotationEngine()
        var sawWarmHistory = false
        var sawFreshChords = false
        var sawEstimate = false
        var sawCameraReset = false
        var sawUnhealthyGeometry = false
        var previous: EngineDiagnostics?
        var highestAllocatedID = 0
        for frame in 0..<310 {
            var input = scene.render(theta: Double(frame) * 20 / 60 * 2 * .pi / 60, depthNoise: 0.004, rng: &rng)
            input.timestamp = Double(frame) / 60
            if frame >= 290 { input.cameraToWorld.translation.x = 0.1 }
            if frame == 300 { input.poseValid = false }
            let capture = frame >= 100 && frame != 180
            let expected = plain.process(input)
            if frame == 200 { captured.beginDiagnosticCapture() }
            let actual = captured.process(input, captureDiagnostics: capture)
            assertAlgorithmEqual(expected, actual)
            XCTAssertNil(expected.diagnostics)
            guard capture else {
                XCTAssertNil(actual.diagnostics)
                previous = nil
                continue
            }
            let diagnostics = try XCTUnwrap(actual.diagnostics)
            XCTAssertEqual(diagnostics.frameIndex, frame + 1)
            XCTAssertEqual(diagnostics.after.state, actual.state)
            XCTAssertEqual(diagnostics.after.theta, actual.theta)
            if let previous, frame != 200 {
                XCTAssertNil(diagnostics.initialContext)
                XCTAssertEqual(diagnostics.before.theta, previous.after.theta)
                XCTAssertEqual(diagnostics.before.confirmations, previous.after.confirmations)
                XCTAssertEqual(diagnostics.before.tracks.map { $0.id }, previous.after.tracks.map { $0.id })
            } else {
                let initial = try XCTUnwrap(diagnostics.initialContext)
                XCTAssertEqual(initial.previousFrameIndex, frame)
                XCTAssertEqual(initial.previousTimestamp, Double(frame - 1) / 60)
                XCTAssertEqual(initial.locator.frame, frame)
                highestAllocatedID = max(highestAllocatedID, initial.locator.nextID - 1)
                if !initial.trackHistories.isEmpty {
                    sawWarmHistory = true
                    XCTAssertTrue(initial.trackHistories.contains { !$0.samples.isEmpty })
                    XCTAssertTrue(initial.trackHistories.flatMap { $0.samples }.allSatisfy { $0.frame <= frame })
                }
            }
            for event in diagnostics.pointEvents where event.action == "added" {
                XCTAssertGreaterThan(event.id, highestAllocatedID, "locator IDs must never be reused")
                highestAllocatedID = event.id
            }
            if let geometry = diagnostics.geometry, let update = geometry.update {
                XCTAssertEqual(geometry.candidates.count, update.candidateCount)
                XCTAssertEqual(geometry.candidates.filter { $0.inlier }.count, update.inlierCount)
                XCTAssertTrue(geometry.candidates.allSatisfy { $0.inlier == (abs($0.residual) < geometry.angleGate) })
            }
            for chord in diagnostics.freshChords {
                sawFreshChords = true
                XCTAssertEqual(chord.end.frame, diagnostics.frameIndex)
                XCTAssertEqual(chord.constraint.chord, chord.end.point - chord.start.point)
                XCTAssertTrue(diagnostics.selectedTrackIDs.contains(chord.trackID))
            }
            sawEstimate = sawEstimate || diagnostics.axisEstimation?.estimate != nil
            sawCameraReset =
                sawCameraReset
                || diagnostics.events.contains {
                    $0.action == "resetMeasurement" && $0.cause == "cameraMotionVeto"
                }
            sawUnhealthyGeometry = sawUnhealthyGeometry || diagnostics.geometry?.healthy == false
            if frame == 300 {
                XCTAssertTrue(diagnostics.cameraMotion?.vetoes.contains("invalidPose") == true)
            }
            if frame == 290 {
                let camera = try XCTUnwrap(diagnostics.cameraMotion)
                XCTAssertTrue(camera.vetoes.contains("cameraTranslation"))
                XCTAssertTrue(
                    diagnostics.trackUpdates.contains { $0.reason == "measurementReset" && !$0.removedIDs.isEmpty })
                XCTAssertGreaterThan(try XCTUnwrap(camera.translationMeters), try XCTUnwrap(camera.translationLimit))
            }
            if frame == 100 || frame == 290 {
                let data = try JSONEncoder().encode(diagnostics)
                let decoded = try JSONDecoder().decode(EngineDiagnostics.self, from: data)
                XCTAssertEqual(decoded.frameIndex, diagnostics.frameIndex)
                XCTAssertEqual(decoded.before.tracks.map { $0.id }, diagnostics.before.tracks.map { $0.id })
            }
            previous = diagnostics
        }
        XCTAssertTrue(sawWarmHistory)
        XCTAssertTrue(sawFreshChords)
        XCTAssertTrue(sawEstimate)
        XCTAssertTrue(sawCameraReset)
        XCTAssertTrue(sawUnhealthyGeometry)
    }

    func testColdIdleAndInvalidFrameIntervalDiagnostics() throws {
        let engine = RotationEngine()
        let first = try XCTUnwrap(engine.process(blankFrame(), captureDiagnostics: true).diagnostics)
        XCTAssertEqual(first.before.state, .idle)
        XCTAssertEqual(first.after.state, .idle)
        XCTAssertEqual(first.initialContext?.previousFrameIndex, 0)
        XCTAssertTrue(first.cameraMotion?.vetoes.contains("invalidFrameInterval") == true)
        XCTAssertTrue(first.cameraMotion?.vetoes.contains("insufficientPoseHistory") == true)
        let second = try XCTUnwrap(engine.process(blankFrame(time: 1), captureDiagnostics: true).diagnostics)
        XCTAssertNil(second.initialContext)
        XCTAssertEqual(second.frameDT, 1)
        XCTAssertEqual(second.integrationDT, 1.0 / 60.0)
        XCTAssertTrue(second.cameraMotion?.vetoes.contains("invalidFrameInterval") == true)
    }

    func testBackToBackCaptureSessionsIncludeContextWithoutReset() throws {
        let engine = RotationEngine()
        let plain = RotationEngine()
        for frame in 0..<5 {
            let input = blankFrame(time: Double(frame) / 60)
            let output = engine.process(input, captureDiagnostics: true)
            assertAlgorithmEqual(plain.process(input), output)
            if frame > 0 { XCTAssertNil(output.diagnostics?.initialContext) }
        }
        engine.beginDiagnosticCapture()
        let input = blankFrame(time: 5.0 / 60.0)
        let output = engine.process(input, captureDiagnostics: true)
        assertAlgorithmEqual(plain.process(input), output)
        let diagnostics = try XCTUnwrap(output.diagnostics)
        XCTAssertEqual(diagnostics.frameIndex, 6)
        XCTAssertEqual(diagnostics.initialContext?.previousFrameIndex, 5)
        XCTAssertEqual(diagnostics.initialContext?.locator.frame, 5)
        XCTAssertTrue(diagnostics.events.isEmpty)
        XCTAssertNil(engine.process(blankFrame(time: 0.1), captureDiagnostics: true).diagnostics?.initialContext)
    }

    func testCameraRotationVetoRecordsMeasuredAngleAndLimit() throws {
        let engine = RotationEngine()
        var output = EngineOutput()
        for frame in 0...5 {
            var input = blankFrame(time: Double(frame) * 0.1)
            if frame == 5 {
                let a = 0.05
                input.cameraToWorld.rotation = M3([cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1])
            }
            output = engine.process(input, captureDiagnostics: true)
        }
        let camera = try XCTUnwrap(output.diagnostics?.cameraMotion)
        XCTAssertFalse(camera.motionValid)
        XCTAssertEqual(camera.vetoes, ["cameraRotation"])
        XCTAssertEqual(try XCTUnwrap(camera.rotationRadians), 0.05, accuracy: 1e-12)
        XCTAssertEqual(try XCTUnwrap(camera.rotationLimit), camera.maxRotationRate * 0.5)
        XCTAssertEqual(camera.translationMeters, 0)
    }

    private func assertAlgorithmEqual(
        _ a: EngineOutput, _ b: EngineOutput, file: StaticString = #filePath, line: UInt = #line
    ) {
        // Wall-clock processing time is not an algorithm result.
        XCTAssertEqual(a.state, b.state, file: file, line: line)
        XCTAssertEqual(a.marker?.x, b.marker?.x, file: file, line: line)
        XCTAssertEqual(a.marker?.y, b.marker?.y, file: file, line: line)
        XCTAssertEqual(a.marker?.radius, b.marker?.radius, file: file, line: line)
        XCTAssertEqual(a.axis, b.axis, file: file, line: line)
        XCTAssertEqual(a.theta, b.theta, file: file, line: line)
        XCTAssertEqual(a.angleDegrees, b.angleDegrees, file: file, line: line)
        XCTAssertEqual(a.angleConfidence, b.angleConfidence, file: file, line: line)
        XCTAssertEqual(a.axisStable, b.axisStable, file: file, line: line)
        XCTAssertEqual(a.axisQuality, b.axisQuality, file: file, line: line)
        XCTAssertEqual(a.rpm, b.rpm, file: file, line: line)
        XCTAssertEqual(a.periodDegrees, b.periodDegrees, file: file, line: line)
        XCTAssertEqual(a.relocalizerFill, b.relocalizerFill, file: file, line: line)
        XCTAssertEqual(a.turnCoverageDegrees, b.turnCoverageDegrees, file: file, line: line)
        XCTAssertEqual(a.objectRadius, b.objectRadius, file: file, line: line)
        XCTAssertEqual(a.heightMin, b.heightMin, file: file, line: line)
        XCTAssertEqual(a.heightMax, b.heightMax, file: file, line: line)
        XCTAssertEqual(a.tracks.count, b.tracks.count, file: file, line: line)
        for (a, b) in zip(a.tracks, b.tracks) {
            XCTAssertEqual(a.x, b.x, file: file, line: line)
            XCTAssertEqual(a.y, b.y, file: file, line: line)
            XCTAssertEqual(a.status, b.status, file: file, line: line)
        }
        XCTAssertEqual(a.trackCount, b.trackCount, file: file, line: line)
        XCTAssertEqual(a.inlierCount, b.inlierCount, file: file, line: line)
        XCTAssertEqual(a.constraintCount, b.constraintCount, file: file, line: line)
        XCTAssertEqual(a.message, b.message, file: file, line: line)
        XCTAssertEqual(a.angleDispersionDegrees, b.angleDispersionDegrees, file: file, line: line)
        XCTAssertEqual(a.relocalizerAnalysed, b.relocalizerAnalysed, file: file, line: line)
        XCTAssertEqual(a.lastRelocalizationAge, b.lastRelocalizationAge, file: file, line: line)
        XCTAssertEqual(a.angleUncertaintyDegrees, b.angleUncertaintyDegrees, file: file, line: line)
    }
}

func blankFrame(time: Double = 0, depth: DepthMap? = nil) -> FrameInput {
    FrameInput(
        image: GrayImage(width: 24, height: 24, fill: 0.5),
        intrinsics: CameraIntrinsics(fx: 24, fy: 24, cx: 12, cy: 12),
        cameraToWorld: .identity, poseValid: true, depth: depth, timestamp: time)
}

func diagnosticsRecorder() throws -> EngineDiagnosticsRecorder {
    try EngineDiagnosticsRecorder(
        XCTUnwrap(RotationEngine().process(blankFrame(), captureDiagnostics: true).diagnostics))
}
