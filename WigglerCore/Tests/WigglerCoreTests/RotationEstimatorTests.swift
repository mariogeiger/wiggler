import XCTest

@testable import WigglerCore

final class RotationEstimatorTests: XCTestCase {
    func testDefaultUsesSeventyFiveTracksAndExplicitReplayConfigIsPreserved() {
        let estimator = RotationEstimator()
        XCTAssertEqual(estimator.algorithm, .trackedGeometry)
        XCTAssertEqual(estimator.config.targetTrackCount, 75)
        XCTAssertEqual(estimator.revision, 0)
        var config = EngineConfig()
        config.targetTrackCount = 123
        let replay = RotationEstimator(revision: 7, config: config)
        XCTAssertEqual(replay.revision, 7)
        XCTAssertEqual(replay.config.targetTrackCount, 123)
        replay.config.targetTrackCount = 95
        XCTAssertEqual(replay.config.targetTrackCount, 95)
    }

    func testPersistentDefaultsAndExplicitReplayConfiguration() {
        let estimator = RotationEstimator(algorithm: .persistentMap)
        XCTAssertEqual(estimator.config.targetTrackCount, 200)
        XCTAssertEqual(estimator.config.explorationPoints, 0)
        var recorded = EngineConfig()
        recorded.targetTrackCount = 75
        recorded.explorationPoints = 80
        let replay = RotationEstimator(algorithm: .persistentMap, revision: 4, config: recorded)
        XCTAssertEqual(replay.config.targetTrackCount, 75)
        XCTAssertEqual(replay.config.explorationPoints, 80)
        replay.select(.persistentMap, revision: 5, config: recorded)
        XCTAssertEqual(replay.config.targetTrackCount, 75)
        replay.select(.persistentMap, revision: 6)
        XCTAssertEqual(replay.config.targetTrackCount, 200)
        XCTAssertEqual(replay.config.explorationPoints, 0)
    }

    func testSegmentConstructsLegacyWithRecordedConfiguration() throws {
        var config = EngineConfig.synthetic
        config.descriptorSide = 9
        config.keyframeBins = 17
        config.axisConstraintCapacity = 80
        let estimator = RotationEstimator()
        estimator.select(.trackedGeometry, revision: 3, config: config)
        let direct = RotationEngine(config: config)
        let scene = SyntheticScene()
        var rng = LCG(seed: 47)
        let encoder = JSONEncoder()
        encoder.outputFormatting = .sortedKeys
        encoder.nonConformingFloatEncodingStrategy = .convertToString(
            positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
        for index in 0..<130 {
            var input = scene.render(theta: Double(index) * 0.035, depthNoise: 0.002, rng: &rng)
            input.timestamp = Double(index) / 20
            let actual = estimator.process(input, captureDiagnostics: true)
            let expected = direct.process(input, captureDiagnostics: true)
            var journal = try XCTUnwrap(actual.diagnostics)
            let offset = actual.axisGeneration - expected.axisGeneration
            journal.before.axisGeneration -= offset
            journal.after.axisGeneration -= offset
            for event in journal.events.indices { journal.events[event].state.axisGeneration -= offset }
            XCTAssertEqual(try encoder.encode(journal), try encoder.encode(expected.diagnostics))
        }
    }

    func testStableIdentifiersRoundTripAndUnknownAlgorithmFails() throws {
        let encoder = JSONEncoder(), decoder = JSONDecoder()
        for algorithm in RotationAlgorithm.allCases {
            XCTAssertEqual(try decoder.decode(RotationAlgorithm.self, from: encoder.encode(algorithm)), algorithm)
        }
        XCTAssertThrowsError(try decoder.decode(RotationAlgorithm.self, from: Data("\"unknown\"".utf8)))
    }

    func testLegacyFacadeIsExactlyTheSameEstimator() throws {
        let direct = RotationEngine(config: .synthetic)
        let facade = RotationEstimator(config: .synthetic)
        let scene = SyntheticScene()
        var rng = LCG(seed: 19)
        let encoder = JSONEncoder()
        encoder.outputFormatting = .sortedKeys
        encoder.nonConformingFloatEncodingStrategy = .convertToString(
            positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
        for index in 0..<130 {
            var input = scene.render(theta: Double(index) * 0.035, depthNoise: 0.002, rng: &rng)
            input.timestamp = Double(index) / 20
            let expected = direct.process(input, captureDiagnostics: true)
            let actual = facade.process(input, captureDiagnostics: true)
            XCTAssertEqual(actual.theta, expected.theta)
            XCTAssertEqual(actual.axisGeneration, expected.axisGeneration)
            XCTAssertEqual(actual.state, expected.state)
            XCTAssertEqual(actual.angleMeasured, expected.angleMeasured)
            XCTAssertEqual(actual.axisStable, expected.axisStable)
            XCTAssertEqual(actual.rpm, expected.rpm)
            XCTAssertEqual(try encoder.encode(actual.diagnostics), try encoder.encode(expected.diagnostics))
        }
    }

    func testSwitchAndRepeatedRevisionReplaceAllState() throws {
        let estimator = RotationEstimator()
        _ = estimator.process(blankFrame(), captureDiagnostics: true)
        estimator.select(.trackedGeometry, revision: 0)
        XCTAssertEqual(estimator.process(blankFrame(time: 1), captureDiagnostics: true).diagnostics?.frameIndex, 2)
        var previous = 0
        for (algorithm, revision) in [
            (RotationAlgorithm.persistentMap, 1), (.trackedGeometry, 2), (.trackedGeometry, 4),
        ] {
            estimator.select(algorithm, revision: revision)
            let output = estimator.process(blankFrame(time: Double(revision)), captureDiagnostics: true)
            XCTAssertEqual(estimator.algorithm, algorithm)
            XCTAssertEqual(estimator.revision, revision)
            XCTAssertGreaterThan(output.axisGeneration, previous)
            XCTAssertFalse(output.angleMeasured)
            XCTAssertEqual(output.theta, 0)
            if algorithm == .trackedGeometry {
                XCTAssertEqual(output.diagnostics?.frameIndex, 1)
                XCTAssertEqual(output.diagnostics?.after.axisGeneration, output.axisGeneration)
                XCTAssertEqual(estimator.config.targetTrackCount, 75)
            }
            previous = output.axisGeneration
        }
    }

    func testReturningToWarmAlgorithmMatchesAFreshStream() throws {
        let estimator = RotationEstimator(config: .synthetic)
        let scene = SyntheticScene()
        var rng = LCG(seed: 23)
        for index in 0..<110 {
            var input = scene.render(theta: Double(index) * 0.04, depthNoise: 0.002, rng: &rng)
            input.timestamp = Double(index) / 20
            _ = estimator.process(input)
        }
        estimator.select(.persistentMap, revision: 1)
        _ = estimator.process(blankFrame(time: 6))
        estimator.select(.trackedGeometry, revision: 2, config: .synthetic)
        let fresh = RotationEngine(config: .synthetic)
        let encoder = JSONEncoder()
        encoder.outputFormatting = .sortedKeys
        encoder.nonConformingFloatEncodingStrategy = .convertToString(
            positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
        var generationOffset: Int?
        for index in 0..<50 {
            var input = scene.render(theta: -Double(index) * 0.04, depthNoise: 0.002, rng: &rng)
            input.timestamp = 7 + Double(index) / 20
            let actual = estimator.process(input, captureDiagnostics: true)
            let expected = fresh.process(input, captureDiagnostics: true)
            XCTAssertEqual(actual.theta, expected.theta)
            XCTAssertEqual(actual.state, expected.state)
            XCTAssertEqual(actual.angleMeasured, expected.angleMeasured)
            let offset = actual.axisGeneration - expected.axisGeneration
            if let generationOffset { XCTAssertEqual(offset, generationOffset) }
            generationOffset = offset
            var journal = try XCTUnwrap(actual.diagnostics)
            XCTAssertEqual(journal.after.axisGeneration, actual.axisGeneration)
            journal.before.axisGeneration -= offset
            journal.after.axisGeneration -= offset
            for event in journal.events.indices { journal.events[event].state.axisGeneration -= offset }
            XCTAssertEqual(try encoder.encode(journal), try encoder.encode(expected.diagnostics))
        }
    }

    func testGenerationInvalidatesHarmonicHistoryEvenWhenLocalGenerationRepeats() {
        let estimator = RotationEstimator()
        var output = estimator.process(blankFrame())
        output.state = .locked
        output.angleMeasured = true
        var fit = HarmonicFit()
        fit.update(frame: blankFrame(), output: output)
        output.theta = 1
        fit.update(frame: blankFrame(time: 1), output: output)
        XCTAssertGreaterThan(fit.turnProgress, 0)
        estimator.select(.trackedGeometry, revision: 2)
        output = estimator.process(blankFrame(time: 2))
        output.state = .locked
        output.angleMeasured = true
        fit.update(frame: blankFrame(time: 2), output: output)
        XCTAssertEqual(fit.turnProgress, 0)
    }
}
