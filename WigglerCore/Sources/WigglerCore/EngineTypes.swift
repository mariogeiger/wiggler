import Foundation

// MARK: - Inputs

public struct CameraIntrinsics {
    /// Focal lengths and principal point in the pixel units of the image handed to the engine.
    public var fx: Double, fy: Double, cx: Double, cy: Double
    public init(fx: Double, fy: Double, cx: Double, cy: Double) {
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
    }
    /// Rescale intrinsics given for a `fromWidth` x `fromHeight` image to a `toWidth` x `toHeight` one (same field of view).
    public func scaled(fromWidth: Double, fromHeight: Double, toWidth: Double, toHeight: Double) -> CameraIntrinsics {
        let sx = toWidth / fromWidth, sy = toHeight / fromHeight
        return CameraIntrinsics(fx: fx * sx, fy: fy * sy, cx: cx * sx, cy: cy * sy)
    }
}

/// Metric depth aligned with the camera image (same field of view, any resolution). Depth = distance along the optical axis.
public struct DepthMap {
    public var width: Int
    public var height: Int
    public var depth: [Float]
    /// Optional per-pixel confidence (ARKit: 0 low, 1 medium, 2 high).
    public var confidence: [UInt8]?
    public init(width: Int, height: Int, depth: [Float], confidence: [UInt8]?) {
        self.width = width
        self.height = height
        self.depth = depth
        self.confidence = confidence
    }

    /// Robust depth at normalised image coordinates (u, v ∈ [0,1]): median of the confident 3x3 neighbourhood,
    /// rejected when the neighbourhood straddles a depth edge.
    public func sample(u: Double, v: Double, minConfidence: UInt8) -> Double? {
        sample(u: u, v: v, minConfidence: minConfidence, observe: nil)
    }

    func sample(
        u: Double, v: Double, minConfidence: UInt8,
        observe: ((EngineDiagnostics.DepthNeighborhood) -> Void)?
    ) -> Double? {
        var evidence = observe.map { _ in EngineDiagnostics.DepthNeighborhood() }
        defer { if let evidence { observe?(evidence) } }
        let x = Int((u * Double(width)).rounded(.down)), y = Int((v * Double(height)).rounded(.down))
        if x < 0 || y < 0 || x >= width || y >= height { return nil }
        var vals: [Double] = []
        vals.reserveCapacity(9)
        for dy in -1...1 {
            let yy = y + dy
            if yy < 0 || yy >= height { continue }
            for dx in -1...1 {
                let xx = x + dx
                if xx < 0 || xx >= width { continue }
                let i = yy * width + xx
                evidence?.pixels.append(
                    .init(
                        x: xx, y: yy, depth: Double(depth[i]), confidence: confidence?[i],
                        accepted: (confidence?[i] ?? minConfidence) >= minConfidence
                            && Double(depth[i]).isFinite && Double(depth[i]) > 0.05))
                if let c = confidence, c[i] < minConfidence { continue }
                let d = Double(depth[i])
                if d.isFinite && d > 0.05 { vals.append(d) }
            }
        }
        evidence?.validCount = vals.count
        if vals.count < 4 {
            evidence?.outcome = "insufficientValidSamples"
            return nil
        }
        vals.sort()
        let med = vals[vals.count / 2]
        // Depth edge: the neighbourhood spans more than 15 % of the depth.
        evidence?.median = med
        evidence?.spread = vals[vals.count - 1] - vals[0]
        evidence?.edgeLimit = 0.15 * med
        if vals[vals.count - 1] - vals[0] > 0.15 * med {
            evidence?.outcome = "depthEdge"
            return nil
        }
        evidence?.outcome = "accepted"
        return med
    }
}

public struct FrameInput {
    /// Luma image used for feature tracking.
    public var image: GrayImage
    public var chromaRed: GrayImage?
    public var chromaBlue: GrayImage?
    public var intrinsics: CameraIntrinsics
    /// Camera-to-world rigid transform (ARKit camera convention: x right, y up, z backward).
    public var cameraToWorld: RigidTransform
    public var poseValid: Bool
    public var depth: DepthMap?
    public var timestamp: Double
    public init(
        image: GrayImage, intrinsics: CameraIntrinsics, cameraToWorld: RigidTransform, poseValid: Bool,
        depth: DepthMap?, timestamp: Double, chromaRed: GrayImage? = nil, chromaBlue: GrayImage? = nil
    ) {
        self.image = image
        self.chromaRed = chromaRed
        self.chromaBlue = chromaBlue
        self.intrinsics = intrinsics
        self.cameraToWorld = cameraToWorld
        self.poseValid = poseValid
        self.depth = depth
        self.timestamp = timestamp
    }
}

// MARK: - Outputs

public enum EngineState: String, Codable {
    case idle  // no marker placed
    case calibrating  // gathering a full turn to find the axis
    case locked  // axis known, angle tracked
    case lost  // axis known but the angle cannot currently be measured
}

public enum TrackStatus { case young, good, inconsistent, noDepth }

public struct TrackDebug {
    public var x: Float
    public var y: Float
    public var status: TrackStatus
}

public struct Marker {
    public var x: Float
    public var y: Float
    /// Region-of-interest radius, pixels.
    public var radius: Float
}

public struct EngineOutput {
    public var state: EngineState = .idle
    /// Where the object is tracked (engine pixels); nil while idle.
    public var marker: Marker?
    public var axis: Axis?
    /// Continuous (unwrapped) angle, radians.
    public var theta: Double = 0
    /// Angle in degrees in [0, 360).
    public var angleDegrees: Double = 0
    public var angleConfidence: Double = 0
    /// The axis is confirmed by fresh, consistent estimates and not contradicted by recent chords. Says nothing
    /// about the current angle measurement: see `angleConfidence` and `angleMeasured`.
    public var axisStable = false
    /// The angle was measured from tracked points this frame (otherwise it is held).
    public var angleMeasured = false
    /// Counts replacements of the axis or of the angle reference: anything accumulated under an earlier
    /// generation (a harmonic map) no longer refers to the current geometry.
    public var axisGeneration = 0
    public var axisQuality: Double = 0
    public var rpm: Double = 0
    /// Appearance period in degrees (360 asymmetric, 180 two-fold, … ; 0 = rotationally symmetric / unknown).
    public var periodDegrees: Double = 360
    public var relocalizerFill: Double = 0
    public var turnCoverageDegrees: Double = 0
    public var objectRadius: Double = 0
    public var heightMin: Double = 0
    public var heightMax: Double = 0
    public var tracks: [TrackDebug] = []
    public var trackCount: Int = 0
    public var inlierCount: Int = 0
    public var constraintCount: Int = 0
    public var message: String = ""
    /// Decision observations only; not a serializable engine checkpoint.
    public var diagnostics: EngineDiagnostics? = nil
    public var processingMillis: Double = 0
    public var angleDispersionDegrees: Double = 0
    public var relocalizerAnalysed = false
    public var lastRelocalizationAge: Int = -1
    /// 1-sigma uncertainty of the absolute angle (degrees): small while everything agrees, large after a loss.
    public var angleUncertaintyDegrees: Double = 0
    public init() {}
}

// MARK: - Configuration

public struct EngineConfig: Codable {
    public var targetTrackCount = 160
    /// Corners tracked beyond the measurement tracks, spread over the whole image, to notice a moving body
    /// anywhere; each costs one KLT solve per frame.
    public var explorationPoints = 80
    public var pyramidLevels = 4
    public var maxResidual: Float = 0.12
    public var minDepthConfidence: UInt8 = 1
    public var chordMinMeters = 0.02
    public var chordMaxFrames = 45
    public var chordStride = 3
    public var axisUpdateInterval = 10
    public var constraintWindowFrames = 900
    public var sampleHistory = 90
    /// Consecutive contradicting axis estimates (one per `axisUpdateInterval`) before the axis is replaced.
    public var lockedDriftFrames = 3
    public var axisConstraintCapacity = 24000
    /// The axis is also estimated from the chords of the last `recentWindowFrames` alone. When that estimate is
    /// well conditioned and lies persistently farther than max(`axisMovedMeters`, 0.2 × object radius) or 12°
    /// from the current axis, the object was moved: the old chords are dropped and the recent axis adopted.
    public var recentWindowFrames = 45
    public var axisMovedMeters = 0.02
    /// A track whose depth jumps by more than this fraction between consecutive samples is on a depth edge
    /// (LiDAR bleeding from the background): the sample is skipped.
    public var maxDepthJumpFraction = 0.08
    /// Points below which the geometric angle is not considered a measurement of the object. With a dozen
    /// agreeing points the angle's dispersion is still a few degrees; fewer survive an acceleration burst.
    public var minHealthyInliers = 12
    /// The library is rebuilt when a strong appearance match has contradicted a healthy geometry for this long.
    public var staleLibrarySeconds = 2.0
    public var relocRefreshAlpha: Float = 0.05
    public var descriptorSide = 32
    public var keyframeBins = 36
    public var lostAfterFrames = 45
    /// Radius of the region of interest around the marker, as a fraction of the image height.
    public var roiRadiusFraction = 0.35
    /// Search for another moving region after the angle has been lost for this long.
    public var reacquisitionDelaySeconds = 5.0
    public init() {}
}
