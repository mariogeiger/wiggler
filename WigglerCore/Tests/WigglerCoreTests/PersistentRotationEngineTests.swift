import XCTest

@testable import WigglerCore

final class PersistentRotationEngineTests: XCTestCase {
    /// The engine is given nothing but frames: no marker, no axis, no hint of where the body is.
    func testFindsAxisAndAngleOfATurningDiskUnaided() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine()
        let run = drivePersistentMap(scene: scene, engine: engine, frames: 300, seed: 11) { Double($0) * 0.05 }
        guard let bootstrap = run.bootstrapFrame else {
            return XCTFail("no axis was ever fitted; last journal \(run.journals[299])")
        }
        XCTAssertLessThan(run.times[bootstrap], 3.5, "the axis took too long to appear")
        let axis = run.outputs[bootstrap].axis!
        XCTAssertGreaterThan(axis.direction.dot(scene.normal), cos(6 * .pi / 180), "axis direction")
        let offset = axis.origin - scene.center
        let lateral = offset - axis.direction * offset.dot(axis.direction)
        XCTAssertLessThan(lateral.length, 0.02, "axis line distance")

        let errors = run.errors(from: bootstrap)
        XCTAssertGreaterThan(
            Double(errors.count) / Double(300 - bootstrap), 0.9, "measured \(errors.count) frames only")
        let worst = errors.map { abs($0.unwrapped) }.max() ?? .pi
        XCTAssertLessThan(worst * 180 / .pi, 4, "unwrapped phase drifted by \(worst * 180 / .pi)°")
        XCTAssertEqual(run.generations(from: bootstrap), [run.outputs[bootstrap].axisGeneration], "generation")
        let last = run.outputs[299]
        XCTAssertEqual(last.rpm, 0.05 * 60 * 60 / (2 * .pi), accuracy: 3, "rate")
        XCTAssertGreaterThan(last.turnCoverageDegrees, 300)
        XCTAssertTrue(last.message.hasPrefix("Experimental"), last.message)
        XCTAssertNil(last.diagnostics)
        print(
            "unaided: bootstrap \(bootstrap), measured \(errors.count)/\(300 - bootstrap),"
                + " worst \(worst * 180 / .pi)°, landmarks \(last.trackCount),"
                + " ms median \(run.millisMedian) max \(run.millisMax)")
    }

    /// Half a second behind a featureless occluder: the map must survive it, refuse to measure through it, and
    /// count the turn that happened while it could not see.
    func testHoldsThroughAShortBlackoutAndKeepsTheTurnCount() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        let run = drivePersistentMap(scene: scene, engine: engine, frames: 330, seed: 23, blackout: 220..<250) {
            Double($0) * 0.05
        }
        guard let bootstrap = run.bootstrapFrame, bootstrap < 200 else {
            return XCTFail("no axis before the blackout; journal \(run.journals[219])")
        }
        for frame in 220..<250 {
            XCTAssertFalse(run.outputs[frame].angleMeasured, "measured an angle through the occluder at \(frame)")
            XCTAssertGreaterThan(run.outputs[frame].trackCount, 0, "the map was thrown away at \(frame)")
        }
        guard let resumed = (250..<330).first(where: { run.outputs[$0].angleMeasured }) else {
            return XCTFail("never measured again; journal \(run.journals[329])")
        }
        XCTAssertLessThan(resumed - 250, 60, "took \(resumed - 250) frames to measure again")
        // Half a second is short enough that the prediction still holds: the same reference must survive it, and
        // the turn that happened behind the occluder must be counted. Realigning the truth here would hide exactly
        // the error this test is for.
        XCTAssertEqual(
            run.outputs[329].axisGeneration, run.outputs[bootstrap].axisGeneration, "took a new reference")
        XCTAssertFalse(run.journals[249].referenceLost, "gave up the prediction over half a second")
        let worst = run.errors(from: bootstrap).filter { $0.frame >= resumed }.map { abs($0.unwrapped) }.max() ?? .pi
        XCTAssertLessThan(worst * 180 / .pi, 5, "the turn through the blackout was miscounted")
        print(
            "blackout: resumed at \(resumed) (+\(resumed - 250) frames), worst after \(worst * 180 / .pi)°,"
                + " generations \(run.generations(from: bootstrap)),"
                + " returned \(run.journals.map { $0.returnedCount }.reduce(0, +))")
    }

    /// Three seconds hidden: the prediction is worth nothing by then. The map must still be there, and whatever the
    /// engine says afterwards must be honest about the turns it could not count.
    func testKeepsTheMapThroughALongBlackoutAndRecoversExplicitly() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        let run = drivePersistentMap(scene: scene, engine: engine, frames: 480, seed: 31, blackout: 200..<380) {
            Double($0) * 0.05
        }
        guard let bootstrap = run.bootstrapFrame, bootstrap < 190 else {
            return XCTFail("no axis before the blackout; journal \(run.journals[199])")
        }
        for frame in 200..<380 {
            XCTAssertFalse(run.outputs[frame].angleMeasured, "measured through the occluder at \(frame)")
            XCTAssertGreaterThan(run.outputs[frame].trackCount, 0, "the map was thrown away at \(frame)")
        }
        XCTAssertTrue(run.journals[370].referenceLost, "the reference should be declared lost by then")
        guard let resumed = (380..<480).first(where: { run.outputs[$0].angleMeasured }) else {
            return XCTFail("never measured again; journal \(run.journals[479])")
        }
        // Three seconds unseen: the engine may not pretend to know how many turns passed. It must say so by taking a
        // new reference, and only then may the angle be compared with the scene again from that reference on.
        XCTAssertTrue(run.journals[resumed].relocked, "resumed without taking a new reference")
        XCTAssertGreaterThan(
            run.outputs[resumed].axisGeneration, run.outputs[bootstrap].axisGeneration, "generation")
        let relocked = run.errors(from: resumed).map { abs($0.unwrapped) }.max() ?? .pi
        XCTAssertLessThan(relocked * 180 / .pi, 5, "the angle after recovery does not follow the scene")
        print(
            "long blackout: resumed at \(resumed), relocked \(run.journals[resumed].relocked),"
                + " generation \(run.outputs[bootstrap].axisGeneration) → \(run.outputs[479].axisGeneration),"
                + " error after \(relocked * 180 / .pi)°, ms max \(run.millisMax)")
    }

    /// A body that turns, stops, and turns back. Integrating increments drifts here; fitting a fixed map should not.
    /// The angle comes from the tracks that carry the landmarks, so an estimate the body has just overtaken cannot
    /// drag the measurement after itself. A speed that changes from one frame to the next puts the truth far outside
    /// anything the previous frame's uncertainty would have justified searching, and it must still be measured.
    func testASuddenChangeOfSpeedIsMeasuredAndNotFollowed() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        let change = 200
        let run = drivePersistentMap(scene: scene, engine: engine, frames: 340, seed: 23) { frame in
            frame < change ? Double(frame) * 0.02 : Double(change) * 0.02 + Double(frame - change) * 0.09
        }
        guard let bootstrap = run.bootstrapFrame, bootstrap < change - 20 else {
            return XCTFail("no axis before the change of speed; journal \(run.journals[339])")
        }
        let errors = run.errors(from: change - 5)
        let worst = errors.map { abs($0.unwrapped) }.max() ?? .pi
        XCTAssertGreaterThan(errors.count, 100, "only \(errors.count) frames measured across the change")
        XCTAssertLessThan(worst * 180 / .pi, 6, "the angle lagged the change by \(worst * 180 / .pi)°")
        XCTAssertEqual(run.generations(from: bootstrap), [run.outputs[bootstrap].axisGeneration], "generation")
        let byTrack = (change..<340).filter { run.journals[$0].association == .track }.count
        XCTAssertGreaterThan(byTrack, 50, "the tracks carried the angle on only \(byTrack) frames")
        print(
            "speed change: \(errors.count) measured, worst \(worst * 180 / .pi)°, \(byTrack) frames by track,"
                + " bound \(run.journals[339].boundCount) of \(run.journals[339].landmarkCount)")
    }

    func testFollowsAPauseAndAReversal() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        let run = drivePersistentMap(scene: scene, engine: engine, frames: 420, seed: 41) { frame in
            if frame < 180 { return Double(frame) * 0.05 }
            if frame < 260 { return 180 * 0.05 }
            return 180 * 0.05 - Double(frame - 260) * 0.04
        }
        guard let bootstrap = run.bootstrapFrame else {
            return XCTFail("no axis was ever fitted; journal \(run.journals[419])")
        }
        let errors = run.errors(from: bootstrap)
        let worst = errors.map { abs($0.unwrapped) }.max() ?? .pi
        XCTAssertLessThan(worst * 180 / .pi, 6, "pause and reversal drifted by \(worst * 180 / .pi)°")
        // A body that holds still stops displacing its points, so the tracks that carry the landmarks have to be
        // asked for by name or they expire. They are, so the pause costs no identities and no turns: one reference
        // holds across the pause and the reversal.
        XCTAssertEqual(run.generations(from: bootstrap), [run.outputs[bootstrap].axisGeneration], "generation")
        XCTAssertLessThan(run.outputs[419].theta, run.outputs[300].theta, "the reversal was not followed")
        print("pause/reverse: measured \(errors.count), worst \(worst * 180 / .pi)°")
    }

    /// Something else moves across the frame while the body turns. It must not be adopted as part of the body.
    func testDoesNotAdoptAnIndependentlyMovingCloud() {
        let base = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        var rng = LCG(seed: 53)
        var outputs: [EngineOutput] = []
        for frame in 0..<300 {
            var drifting = base
            let shift = Double(frame) * 0.004
            drifting.background = base.background.map { (V3($0.0.x + shift, $0.0.y, $0.0.z), $0.1) }
            var input = drifting.render(theta: Double(frame) * 0.05, depthNoise: 0.002, rng: &rng)
            input.timestamp = Double(frame) / 60
            outputs.append(engine.process(input))
        }
        guard let bootstrap = outputs.firstIndex(where: { $0.axis != nil }) else {
            return XCTFail("no axis was fitted beside a drifting cloud")
        }
        let axis = outputs[bootstrap].axis!
        XCTAssertGreaterThan(axis.direction.dot(base.normal), cos(8 * .pi / 180), "axis direction")
        for output in outputs[bootstrap...] where output.angleMeasured {
            XCTAssertLessThan(output.objectRadius, 0.25, "the map grew beyond the body")
        }
        let measured = outputs[bootstrap...].reduce(0) { $0 + ($1.angleMeasured ? 1 : 0) }
        XCTAssertGreaterThan(
            Double(measured) / Double(300 - bootstrap), 0.7, "measured \(measured) frames beside the cloud")
        print("distractor: bootstrap \(bootstrap), measured \(measured), radius \(outputs[299].objectRadius)")
    }

    /// Nothing turns. The engine must not invent an axis for a still scene.
    func testFitsNoAxisWhenNothingTurns() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        let run = drivePersistentMap(scene: scene, engine: engine, frames: 200, seed: 67) { _ in 0.4 }
        XCTAssertNil(run.bootstrapFrame, "an axis was fitted for a still scene")
        XCTAssertEqual(run.measuredCount, 0)
        XCTAssertEqual(run.outputs[199].state, .idle)
    }

    /// Frames without a pose can be tracked but not understood: nothing may be measured or learnt from them.
    func testMeasuresAndLearnsNothingWithoutAPose() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        var rng = LCG(seed: 71)
        var outputs: [EngineOutput] = []
        var journals: [PersistentMapDiagnostics] = []
        for frame in 0..<330 {
            var input = scene.render(theta: Double(frame) * 0.05, depthNoise: 0.002, rng: &rng)
            input.timestamp = Double(frame) / 60
            if (220..<260).contains(frame) { input.poseValid = false }
            outputs.append(engine.process(input))
            journals.append(engine.lastDiagnostics!)
        }
        guard let bootstrap = outputs.firstIndex(where: { $0.axis != nil }), bootstrap < 210 else {
            return XCTFail("no axis before the pose was lost")
        }
        let before = outputs[219].trackCount
        for frame in 220..<260 {
            XCTAssertFalse(outputs[frame].angleMeasured, "measured without a pose at \(frame)")
            XCTAssertFalse(journals[frame].poseValid)
            XCTAssertEqual(outputs[frame].trackCount, before, "the map changed without a pose at \(frame)")
        }
        XCTAssertTrue(outputs[260...].contains { $0.angleMeasured }, "never recovered after the pose returned")
    }

    /// A gap in the timestamps is a real gap: the interval must widen by the time that passed, not by one frame, and
    /// what the engine says afterwards must come from a search that covered that widened interval.
    func testATimestampGapWidensTheAngleInterval() {
        let scene = SyntheticScene()
        let engine = PersistentRotationEngine(config: .persistentSynthetic)
        var rng = LCG(seed: 83)
        var outputs: [EngineOutput] = []
        var journals: [PersistentMapDiagnostics] = []
        for frame in 0..<300 {
            var input = scene.render(theta: Double(frame) * 0.05, depthNoise: 0.002, rng: &rng)
            input.timestamp = Double(frame) / 60 + (frame >= 220 ? 3 : 0)
            outputs.append(engine.process(input))
            journals.append(engine.lastDiagnostics!)
        }
        XCTAssertNotNil(outputs.firstIndex(where: { $0.axis != nil }), "no axis")
        XCTAssertTrue(journals[220].discontinuity, "the gap was not seen as one")
        // Three seconds of unseen rotation leaves no usable prediction: the frame may only measure the angle by
        // searching the whole turn, and then it is a new reference, not the old angle continued.
        XCTAssertTrue(journals[220].referenceLost, "a three-second gap left the prediction in force")
        XCTAssertEqual(journals[220].searchWindow, .pi, accuracy: 1e-9, "the whole turn was not searched")
        if outputs[220].angleMeasured {
            XCTAssertTrue(journals[220].relocked, "measured across the gap without taking a new reference")
            XCTAssertGreaterThan(
                outputs[220].axisGeneration, outputs[219].axisGeneration, "a new reference kept the old generation")
        }
        XCTAssertTrue(outputs[221...].contains { $0.angleMeasured }, "never measured again after the gap")
        print(
            "gap: sigma \(outputs[219].angleUncertaintyDegrees)° → \(outputs[220].angleUncertaintyDegrees)°,"
                + " relocked \(journals[220].relocked), generation \(outputs[220].axisGeneration)")
    }
}
