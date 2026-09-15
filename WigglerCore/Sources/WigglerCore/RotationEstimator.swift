/// A complete axis-and-angle estimator. Image and geometry primitives may be shared; state is not.
public protocol RotationAlgorithmEngine: AnyObject {
    var config: EngineConfig { get set }
    func process(_ input: FrameInput, captureDiagnostics: Bool) -> EngineOutput
    func beginDiagnosticCapture()
}

extension RotationEngine: RotationAlgorithmEngine {}
extension PersistentRotationEngine: RotationAlgorithmEngine {}

/// Stable identifiers stored in recordings. Missing identifiers in older recordings mean tracked geometry.
public enum RotationAlgorithm: String, Codable, CaseIterable {
    case trackedGeometry
    case persistentMap

    public var shortLabel: String {
        switch self {
        case .trackedGeometry: return "Tracks"
        case .persistentMap: return "Map (exp.)"
        }
    }

    public var label: String {
        switch self {
        case .trackedGeometry: return "Tracked geometry"
        case .persistentMap: return "Persistent rigid map (experimental)"
        }
    }

    fileprivate func makeEngine(config: EngineConfig?) -> any RotationAlgorithmEngine {
        switch self {
        case .trackedGeometry:
            var defaults = EngineConfig()
            defaults.targetTrackCount = 75
            return RotationEngine(config: config ?? defaults)
        case .persistentMap:
            return PersistentRotationEngine(config: config)
        }
    }
}

/// Owns exactly one algorithm and gives each new angle reference a distinct generation.
public final class RotationEstimator {
    public private(set) var algorithm: RotationAlgorithm
    public private(set) var revision: Int
    private var engine: any RotationAlgorithmEngine
    private var generationOffset = 0
    private var lastGeneration = 0

    /// Explicit configurations support historical replay; the app uses each algorithm's fixed defaults.
    public init(algorithm: RotationAlgorithm = .trackedGeometry, revision: Int = 0, config: EngineConfig? = nil) {
        self.algorithm = algorithm
        self.revision = revision
        engine = algorithm.makeEngine(config: config)
    }

    public var config: EngineConfig {
        get { engine.config }
        set { engine.config = newValue }
    }

    /// A changed revision also resets the same algorithm, including a switch away and back between frames.
    public func select(_ algorithm: RotationAlgorithm, revision: Int, config: EngineConfig? = nil) {
        guard self.algorithm != algorithm || self.revision != revision else { return }
        self.algorithm = algorithm
        self.revision = revision
        engine = algorithm.makeEngine(config: config)
        generationOffset = lastGeneration + 1
        lastGeneration = generationOffset
    }

    public func beginDiagnosticCapture() {
        engine.beginDiagnosticCapture()
    }

    public func process(_ input: FrameInput, captureDiagnostics: Bool = false) -> EngineOutput {
        var output = engine.process(input, captureDiagnostics: captureDiagnostics)
        if captureDiagnostics, let persistent = engine as? PersistentRotationEngine {
            output.persistentDiagnostics = persistent.lastDiagnostics
        }
        if var diagnostics = output.diagnostics {
            diagnostics.before.axisGeneration += generationOffset
            diagnostics.after.axisGeneration += generationOffset
            for index in diagnostics.events.indices {
                diagnostics.events[index].state.axisGeneration += generationOffset
            }
            output.diagnostics = diagnostics
        }
        output.axisGeneration += generationOffset
        lastGeneration = output.axisGeneration
        return output
    }
}
