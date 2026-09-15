import Foundation

/// What the persistent-map engine decided on one frame, and on what evidence. This is a journal for replay and
/// review, not a checkpoint: it cannot restore the map.
public struct PersistentMapDiagnostics: Codable {
    /// What the axis fit was asked and what it answered.
    public struct AxisFitNote: Codable {
        public var reason: String
        public var chords: Int
        public var inlierRatio: Double
        public var planarity: Double
        public var coverage: Double
        public var origin: V3?
        public var direction: V3?
        public var seededRate: Double?
        public init(
            reason: String, chords: Int, inlierRatio: Double, planarity: Double, coverage: Double,
            origin: V3? = nil, direction: V3? = nil, seededRate: Double? = nil
        ) {
            self.reason = reason
            self.chords = chords
            self.inlierRatio = inlierRatio
            self.planarity = planarity
            self.coverage = coverage
            self.origin = origin
            self.direction = direction
            self.seededRate = seededRate
        }
    }

    /// What established which pixel carries which landmark this frame.
    public enum Association: String, Codable {
        /// Live image tracks: identity from the image sequence, so the whole circle could be scored.
        case track
        /// The patches themselves, scored over the whole circle because they alone cannot say who is who.
        case appearance
    }

    public var frame = 0
    public var time = 0.0
    /// searching, bootstrapping, mapping or holding.
    public var phase = "searching"
    /// Angle carried into this frame, radians.
    public var predictedTheta = 0.0
    /// Angle this frame measured, radians; nil when the frame measured nothing.
    public var measuredTheta: Double?
    /// Angle the filter holds after this frame, radians; nil when the frame measured nothing.
    public var filteredTheta: Double?
    /// What said which pixel is which landmark; nil until the map exists.
    public var association: Association?
    /// Landmarks carried by a live track at the end of this frame.
    public var boundCount = 0
    /// Landmarks that lost their track this frame because the track itself ended.
    public var trackEndedCount = 0
    /// Landmarks whose track was let go because the pixel it pointed at no longer looked like the landmark or
    /// disagreed with where the body puts it.
    public var trackContradictedCount = 0
    public var omega = 0.0
    /// 1-sigma uncertainty of the angle after this frame, radians.
    public var sigma = 0.0
    /// Half-width of the angle search this frame, radians.
    public var searchWindow = 0.0
    /// Score of the best rival angle relative to the accepted one; near 1 means the frame was ambiguous.
    public var ambiguity = 0.0
    /// Whether this frame had a pose to place anything in the world; without one nothing is measured or learnt.
    public var poseValid = true
    /// Time did not advance, or the gap was too long to bridge: the retained positions were dropped.
    public var discontinuity = false
    /// The frame's timestamp did not follow the last one, so the frame was dropped whole.
    public var timeRegressed = false
    /// The prediction became worth less than the widest ordinary search, so the angle reference was lost and only
    /// a search of the whole circle could offer a new one.
    public var referenceLost = false
    /// This frame took a new reference from the whole circle; how many turns went by is unknown.
    public var relocked = false
    public var landmarkCount = 0
    public var visibleCount = 0
    public var supportCount = 0
    /// Landmarks that agreed after at least ten frames unseen.
    public var returnedCount = 0
    public var createdCount = 0
    public var evictedCount = 0
    /// Moving points refused as landmarks because depth could not describe their surface plane.
    public var planeRejectedCount = 0
    /// Reprojection error of the supporting landmarks, pixels.
    public var reprojectionRMS = 0.0
    public var trackedPointCount = 0
    public var movingPointCount = 0
    public var chordCount = 0
    public var axisFit: AxisFitNote?
    /// Landmark identities, recorded only when the frame was captured.
    public var observedIDs: [Int] = []
    public var returnedIDs: [Int] = []
    public var millis = 0.0
    public init() {}
}
