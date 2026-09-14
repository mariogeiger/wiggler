import Foundation

/// Per-frame observations in engine pixels, world meters, radians, and seconds unless named otherwise.
/// Synthesized Codable leaves non-finite float handling to the caller's encoder/decoder strategies.
/// Optional fields are absent when that stage did not run. This is not an exact warm replay checkpoint.
public struct EngineDiagnostics: Codable {
    public var schemaVersion = 1
    public var frameIndex: Int
    public var timestamp: Double
    public var config: EngineConfig
    public var before: DecisionState
    public var after: DecisionState
    /// Present on the first captured frame, including when capture resumes after an uncaptured frame.
    public var initialContext: InitialContext?
    public var frameDT = 0.0
    public var integrationDT = 0.0
    public var poseUsed: RigidTransform?
    public var cameraMotion: CameraMotion?
    public var locator: LocatorState?
    public var pointEvents: [PointEvent] = []
    public var selectedTrackIDs: [Int] = []
    public var trackUpdates: [TrackUpdate] = []
    public var depth: [DepthObservation] = []
    public var depthSamplingSkipped: String?
    public var chordMinLengthMeters: Double?
    public var freshChords: [FreshChord] = []
    public var chordBatch: ChordBatch?
    public var axisEstimation: AxisEstimation?
    public var geometry: GeometryDecision?
    public var relocalization: Relocalization?
    /// In execution order. State is observed immediately before the named action.
    public var events: [Event] = []

    public struct DecisionState: Codable {
        public var state: EngineState
        public var axis: Axis?
        public var displayedAxis: Axis?
        public var marker: [Float]?
        public var confirmations: Int
        public var newestEvidenceFrame: Int
        public var inconsistentBatches: Int
        public var inconsistentAxisCount: Int
        public var calibrationStartFrame: Int
        public var theta: Double
        public var lastDelta: Double
        public var thetaMin: Double
        public var thetaMax: Double
        public var lastAngleOkFrame: Int
        public var lastRelocFrame: Int
        public var lastAngleOk: Bool
        public var inlierCount: Int
        public var dispersion: Double
        public var imageScale: Double?
        public var disagreeStreak: Int
        public var rpmFiltered: Double
        public var lostSince: Double?
        public var constraintCount: Int
        public var oldestConstraintFrame: Int
        public var newestConstraintFrame: Int
        public var lastEstimate: Estimate?
        public var tracks: [Track]
        public var angleTracks: [AngleTrack]
        public var fusionSigma: Double
        public var fusionPending: Double
        public var fusionStaleSeconds: Double
        public var fusionLastMatchTrusted: Bool
        public var relocalizerFill: Double
        public var relocalizerPeriod: Double
        public var relocalizerAnalysed: Bool
    }

    public struct InitialContext: Codable {
        public var previousFrameIndex: Int
        public var previousTimestamp: Double?
        public var lastPose: RigidTransform
        public var previousImageWidth: Int?
        public var previousImageHeight: Int?
        public var trackHistories: [TrackHistory]
        /// Older estimator chords lack source IDs; midpoint ± chord/2 supplies equivalent endpoints.
        public var constraints: [ChordConstraint]
        public var locator: LocatorState
    }

    public struct Sample: Codable {
        public var frame: Int
        public var point: V3
    }

    public struct TrackHistory: Codable {
        public var id: Int
        public var samples: [Sample]
    }

    public struct Track: Codable {
        public var id: Int
        public var x: Float
        public var y: Float
        /// Measurement-track age; zero at selection, increments only while this identity survives.
        public var age: Int
        public var hasDepthThisFrame: Bool
        public var lastDepth: Double
        public var depthRejects: Int
        public var lastChordFrame: Int
        public var sampleCount: Int
        public var oldestSampleFrame: Int?
        public var newestSampleFrame: Int?
        public var radius: Double
        public var height: Double
    }

    public struct AngleTrack: Codable {
        public var id: Int
        public var offset: Double
        public var badCount: Int
        public var age: Int
    }

    public struct TrackUpdate: Codable {
        public var reason = "selection"
        public var selectedIDs: [Int]
        public var addedIDs: [Int]
        public var removedIDs: [Int]
    }

    public struct PointEvent: Codable {
        public var id: Int
        public var action: String
        public var reason: String
        public var kltOK: Bool?
        /// Mean absolute patch residual, absent when no finite comparison was computed.
        public var residual: Float?
        public var x: Float
        public var y: Float

        init(id: Int, action: String, reason: String, kltOK: Bool? = nil, residual: Float? = nil, x: Float, y: Float) {
            self.id = id
            self.action = action
            self.reason = reason
            self.kltOK = kltOK
            self.residual = residual.flatMap { $0.isFinite ? $0 : nil }
            self.x = x
            self.y = y
        }
    }

    public struct LocatorPoint: Codable {
        public var id: Int
        public var x: Float
        public var y: Float
        public var previousX: Float
        public var previousY: Float
        public var speed: Float
        public var duration: Double
        public var stillFor: Double
    }

    public struct TimedPose: Codable {
        public var time: Double
        public var pose: RigidTransform
    }

    public struct LocatorState: Codable {
        public var frame: Int
        public var nextID: Int
        public var tile: Int
        public var candidate: [Float]?
        public var candidateSince: Double
        public var movingCount: Int
        public var points: [LocatorPoint]
        public var poses: [TimedPose]
        public var maxPoints: Int
        public var detectEvery: Int
        public var speedTau: Double
        public var minimumMovingDuration = 0.1
        public var retentionStillSeconds = 0.5
        public var probeStillExpirySeconds = 0.75
        public var probeLifetimeSeconds = 1.5
        public var selectionRadiusScale: Float = 1.3
        public var movingSpeed: Float
        public var minMoving: Int
        public var settleSeconds: Double
        public var maxResidual: Float
        public var border: Float
    }

    public struct CameraMotion: Codable {
        public var poseValid: Bool
        public var dt: Double
        public var windowSeconds: Double
        public var maxRotationRate: Double
        public var maxTranslationSpeed: Double
        public var span: Double?
        public var rotationRadians: Double?
        public var translationMeters: Double?
        public var rotationLimit: Double?
        public var translationLimit: Double?
        public var motionValid = false
        public var vetoes: [String] = []
    }

    public struct DepthPixel: Codable {
        public var x: Int
        public var y: Int
        public var depth: Double
        public var confidence: UInt8?
        public var accepted: Bool
    }

    public struct DepthNeighborhood: Codable {
        public var pixels: [DepthPixel] = []
        public var minimumValidCount = 4
        public var minimumDepthMeters = 0.05
        public var edgeFraction = 0.15
        public var validCount = 0
        public var median: Double?
        public var spread: Double?
        public var edgeLimit: Double?
        public var outcome = "outOfBounds"
    }

    public struct DepthObservation: Codable {
        public var id: Int
        public var x: Float
        public var y: Float
        public var previousDepth: Double
        public var jumpAcceptanceRejectCount = 4
        public var rejectsBefore: Int
        public var minConfidence: UInt8
        public var maxJumpFraction: Double
        public var neighborhood: DepthNeighborhood?
        public var sampledDepth: Double?
        public var jumpLimit: Double?
        public var rejectsAfter: Int
        public var worldPoint: V3?
        public var outcome = "noDepthMap"
    }

    public struct FreshChord: Codable {
        public var trackID: Int
        public var start: Sample
        public var end: Sample
        public var constraint: ChordConstraint
    }

    public struct ChordResidual: Codable {
        public var trackID: Int?
        public var tiltMeters: Double
        public var offsetMeters: Double
        public var residualMeters: Double
        public var exceedsTolerance: Bool
    }

    public struct ChordBatch: Codable {
        public var axis: Axis
        public var toleranceMeters: Double
        public var minimumCount = 8
        public var requiredBatches: Int
        public var count: Int
        public var badCount: Int
        public var rejects: Bool
        public var residuals: [ChordResidual]
        public var confirmationsBefore: Int
        public var confirmationsAfter: Int
        public var inconsistentBefore: Int
        public var inconsistentAfter: Int
        public var restartRequired: Bool
    }

    public struct Estimate: Codable {
        public var axis: Axis
        public var eigenvalues: [Double]
        public var inlierRatio: Double
        public var constraintCount: Int
        public var planarity: Double
        public var coverage: Double
        public var isWellConditioned: Bool
        public var minimumConstraintCount = 150
        public var minimumInlierRatio = 0.45
        public var maximumPlanarity = 0.2
        public var minimumCoverage = 0.25

        init(_ estimate: AxisEstimate) {
            axis = estimate.axis
            eigenvalues = estimate.eigenvalues
            inlierRatio = estimate.inlierRatio
            constraintCount = estimate.constraintCount
            planarity = estimate.planarity
            coverage = estimate.coverage
            isWellConditioned = estimate.isWellConditioned
        }
    }

    public struct AxisEstimation: Codable {
        public var evidenceFrame: Int
        public var estimate: Estimate?
        public var huberMeters: Double
        public var iterations: Int
        public var objectRadius: Double
        public var consistent: Bool?
        public var angleBetween: Double?
        public var lineDistance: Double?
        public var consistencyAngleLimit = 12 * Double.pi / 180
        public var consistencyDistanceLimit: Double
        public var farAngleLimit = 25 * Double.pi / 180
        public var farDistanceLimit: Double
        public var action = "unavailableOrIllConditioned"
    }

    public struct GeometryDecision: Codable {
        public var observations: [AngleObservation] = []
        public var candidates: [AngleCandidate] = []
        public var radiusCap: Double?
        public var predictedTheta: Double?
        public var thetaBeforeHardGate: Double?
        public var angleTracksBefore: [AngleTrack] = []
        public var angleTracksAfter: [AngleTrack] = []
        public var update: AngleUpdate?
        public var similarityAngle: Double?
        public var similarityInliers: Int?
        public var similarityScale: Double?
        public var predictedImageAngle: Double?
        public var crossCheckLimit: Double?
        public var agrees = true
        public var disagreeStreak = 0
        public var similarityMinimumInliers = 8
        public var scaleLearningMinimumDelta = Double.pi / 180
        public var crossCheckBaseLimit = 3 * Double.pi / 180
        public var crossCheckRelativeLimit = 0.35
        public var disagreeStreakLimit = 3
        public var minimumRadiusMeters = 0.005
        public var minHealthyInliers: Int
        public var healthy = false
        public var angleGate: Double
        public var angleSigma: Double
        public var angleMinInliers: Int
        public var angleMaxBadFrames: Int
    }

    public struct AngleCandidate: Codable {
        public var id: Int
        public var value: Double
        public var weight: Double
        /// Residual and membership in the final hard-gated pass, before its mean correction.
        public var residual: Double
        public var inlier: Bool
    }

    public struct Relocalization: Codable {
        public var candidates: [AngleFusion.Candidate]
        public var correction: Double
        public var outcome: AngleFusion.Outcome
    }

    public struct Event: Codable {
        public var action: String
        public var cause: String
        public var state: DecisionState
    }
}

/// Allocated only while a frame is being captured. Shared by the engine's existing stages.
final class EngineDiagnosticsRecorder {
    var value: EngineDiagnostics
    init(_ value: EngineDiagnostics) { self.value = value }
}

final class DepthObservationRecorder {
    var value: EngineDiagnostics.DepthObservation
    init(_ value: EngineDiagnostics.DepthObservation) { self.value = value }
}
