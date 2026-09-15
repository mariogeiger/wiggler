import XCTest

@testable import WigglerCore

/// Drives the persistent-map engine over a synthetic run and keeps what it said beside what the scene did.
struct PersistentMapRun {
    var outputs: [EngineOutput] = []
    var journals: [PersistentMapDiagnostics] = []
    var trueTheta: [Double] = []
    var times: [Double] = []

    /// First frame with an axis: the engine's angle starts from zero there.
    var bootstrapFrame: Int? { outputs.firstIndex { $0.axis != nil } }

    /// Angle error against the scene, radians, for every frame from `from` on that measured the angle. `unwrapped`
    /// counts the turns: a lost or invented turn shows there and nowhere else. `wrapped` ignores them, which is
    /// what a dial shows. Both are kept because they answer different questions.
    func errors(from frame: Int) -> [(frame: Int, unwrapped: Double, wrapped: Double)] {
        let offset = trueTheta[frame] - outputs[frame].theta
        return outputs.indices.filter { $0 >= frame && outputs[$0].angleMeasured }
            .map { index in
                let error = outputs[index].theta + offset - trueTheta[index]
                return (index, error, wrapAngle(error))
            }
    }

    /// Generations seen from `frame` on: more than one means the engine explicitly gave up the old reference.
    func generations(from frame: Int) -> Set<Int> {
        Set(outputs.indices.filter { $0 >= frame }.map { outputs[$0].axisGeneration })
    }

    var measuredCount: Int { outputs.reduce(0) { $0 + ($1.angleMeasured ? 1 : 0) } }
    var millisMedian: Double { median(outputs.map { $0.processingMillis }) }
    var millisMax: Double { outputs.map { $0.processingMillis }.max() ?? 0 }
}

extension EngineConfig {
    /// A smaller point budget for the auxiliary synthetic runs: the disk offers few corners and the per-point KLT
    /// dominates the run time. The accuracy run uses the engine's own default.
    static let persistentSynthetic: EngineConfig = {
        var config = EngineConfig()
        config.targetTrackCount = 80
        config.explorationPoints = 0
        return config
    }()
}

/// A body hidden behind a flat, featureless occluder: no texture to track and depth that belongs to something
/// else. The engine must hold its angle rather than measure one.
func blackedOut(_ input: FrameInput) -> FrameInput {
    var out = input
    out.image = GrayImage(width: input.image.width, height: input.image.height, fill: 0.5)
    if var depth = input.depth {
        depth.depth = [Float](repeating: 0.35, count: depth.depth.count)
        out.depth = depth
    }
    return out
}

/// Renders `frames` frames of `scene` at 60 Hz with the scene angle `theta(frame)` and feeds them to `engine`.
func drivePersistentMap(
    scene: SyntheticScene, engine: PersistentRotationEngine, frames: Int, seed: UInt64,
    depthNoise: Double = 0.002, blackout: Range<Int> = 0..<0, theta: (Int) -> Double
) -> PersistentMapRun {
    var rng = LCG(seed: seed)
    var run = PersistentMapRun()
    for frame in 0..<frames {
        let angle = theta(frame)
        var input = scene.render(theta: angle, depthNoise: depthNoise, rng: &rng)
        input.timestamp = Double(frame) / 60
        if blackout.contains(frame) { input = blackedOut(input) }
        run.outputs.append(engine.process(input, captureDiagnostics: true))
        run.journals.append(engine.lastDiagnostics ?? PersistentMapDiagnostics())
        run.trueTheta.append(angle)
        run.times.append(input.timestamp)
    }
    return run
}
