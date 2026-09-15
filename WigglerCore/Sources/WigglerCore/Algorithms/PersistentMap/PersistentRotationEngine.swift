import Foundation

/// Experimental engine: measures the rotation axis and the rotation angle from a map of the body's own landmarks,
/// held in the body's frame. Chords of the points that turn about a common axis give the axis; from then on one
/// scalar angle explains where every landmark is seen, so a landmark that disappears and comes back is evidence
/// again instead of being replaced, and the angle is fitted, never integrated.
///
/// It acquires the body by itself: no marker, no recorded axis, full-image features, one set of parameters.
/// Nothing about it is settled. Each landmark's appearance is transported assuming its surface element is planar,
/// which depth measures but only approximately; a body that looks the same from several angles can be read at the
/// wrong angle; and the reported uncertainty is the one this model implies, not a bound on the truth. The outputs
/// say when the engine is only holding a prediction, and when it has lost the angle reference altogether.
public final class PersistentRotationEngine {
    /// Image points this engine tracks. Its own fixed target: acquisition rests on each point's own displacement,
    /// so it needs no separate pool of exploration points.
    public static let pointTarget = 200

    public var config: EngineConfig
    public private(set) var lastDiagnostics: PersistentMapDiagnostics?

    /// Image layer, shared with the tracked-geometry engine.
    private var klt = KLTTracker()
    private var corners = CornerDetector()
    private var locator = MotionLocator()
    private var previousPyramid: Pyramid?
    private let points = TrackedWorldPoints()

    /// Geometry layer.
    private var chords: [ChordConstraint] = []
    private let consensus = ChordConsensus()
    private var map: RigidRotationMap?
    private var filter = RotationPhaseFilter()
    private let sampler = SurfacePatchSampler(side: 9, halfSize: 0.018)

    private var frameIndex = 0
    private var lastTimestamp: Double?
    private var lastPose = RigidTransform.identity
    private var poseEverValid = false
    private var state: EngineState = .idle
    private var axisGeneration = 0
    private var retainedIDs: Set<Int> = []

    /// The tracks this engine still needs kept alive: the ones whose displacement is still being gathered towards a
    /// chord, and every one that carries a landmark. A body that stops turning stops displacing its points, and
    /// unprotected probes then expire — which would cost the map the very identities that make its angle absolute,
    /// for no reason but that the body held still.
    private var neededIDs: Set<Int> {
        var ids = retainedIDs
        guard let map else { return ids }
        for landmark in map.landmarks where landmark.trackID != 0 { ids.insert(landmark.trackID) }
        return ids
    }
    private var bootstrapStart: Double?
    /// The last axis that its chords determined: two successive fits must agree before one is adopted.
    private var previousCandidate: Axis?
    /// Recent angles with their times, so a point's whole history can be taken back to the body's frame.
    private var thetaTrace: [(time: Double, theta: Double)] = []
    private var lastEstimate: AxisEstimate?
    private var lastMeasurement: RigidRotationMap.Measurement?
    private var lastMeasuredFrame = -1000
    private var lastMeasuredTime: Double?
    private var lastReturnFrame = -1000
    private var thetaMin = 0.0
    private var thetaMax = 0.0
    private var lastSupport = 0
    private var lastDispersion = 0.0
    private var marker: Marker?

    /// Seconds of chords before the axis is first fitted: long enough for a body turning a few degrees per frame to
    /// sweep chords in every direction around its axis.
    private let bootstrapSeconds = 2.0
    /// Fewest chords a fit may rest on. It is the robust fit's own floor, not a budget for a particular body: how
    /// many chords there are says nothing about observability, so the rest of the judgement is geometric.
    private let minChords = 30
    private let seedLimit = 240
    private let insertLimit = 24
    private let landmarkCapacity = 600
    /// Widest half-window of an ordinary angle search, radians. Beyond this the prediction is worth so little that
    /// only a search of the whole circle can say anything, and that is a new reference, not a continuation.
    private let maxSearchWindow = 60 * Double.pi / 180
    /// A frame whose second-best angle scores this fraction of the best one says nothing trustworthy. Reclaiming a
    /// lost reference from the whole circle must clear a stricter bar, since it has no prediction behind it.
    private let ambiguityLimit = 0.85
    private let relockAmbiguityLimit = 0.6
    /// A gap longer than this leaves nothing tracked to compare with, so the point histories are dropped.
    private let timeGapLimit = 1.0
    /// Frames unseen after which a landmark counts as having come back rather than simply been tracked.
    private let gapFrames = 10
    private let trackDebugLimit = 500
    /// How far a candidate's positions may disagree about its place on the body, metres: two depth samples of a real
    /// sensor already differ by about this much.
    private let rigidSpreadMeters = 0.012
    /// Angles kept for testing a candidate against the body's own rotation, seconds: the point histories are no
    /// longer than this either.
    private let thetaTraceSeconds = 1.2
    /// Angles kept at most, whatever the frame rate, so the trace is bounded like the point histories are.
    private let thetaTraceLimit = 64

    public init(config: EngineConfig? = nil) {
        if let config {
            self.config = config
        } else {
            var own = EngineConfig()
            own.targetTrackCount = Self.pointTarget
            own.explorationPoints = 0
            self.config = own
        }
        klt.halfWindow = 4
        corners.minDistance = 8
    }

    /// Drop the previous frame's journal, so a fresh capture cannot read a stale one as its own.
    public func beginDiagnosticCapture() {
        lastDiagnostics = nil
    }

    // MARK: Main entry point

    public func process(_ input: FrameInput, captureDiagnostics: Bool = false) -> EngineOutput {
        let started = Date()
        frameIndex += 1
        var journal = PersistentMapDiagnostics()
        journal.frame = frameIndex
        journal.time = input.timestamp

        // A frame whose timestamp does not follow the last one is not part of the stream: it is dropped whole,
        // without advancing the clock, because letting an older image correct the current angle would be measuring
        // the past as if it were now. A forward gap too long for anything tracked to bridge keeps the frame but
        // drops the retained positions, and the real elapsed time still goes to the filter, so the interval widens
        // by as much time as actually passed.
        let elapsed = lastTimestamp.map { input.timestamp - $0 }
        let regressed = (elapsed ?? 1) <= 0
        let gap = (elapsed ?? 0) > timeGapLimit
        let dt = max(0, elapsed ?? 0)
        if regressed || gap {
            points.removeAll()
            locator.reset()
            previousPyramid = nil
        }
        journal.discontinuity = regressed || gap
        journal.timeRegressed = regressed

        let pyramid = Pyramid(image: input.image, levelCount: config.pyramidLevels)
        if input.poseValid {
            lastPose = input.cameraToWorld
            poseEverValid = true
        }
        // Without a pose of this frame nothing can be placed in the world: the frame may track pixels, but it may
        // not measure geometry, learn a landmark, or claim the body moved.
        let poseUsable = input.poseValid && poseEverValid && !regressed
        journal.poseValid = poseUsable
        let view = PinholeProjection(
            cameraToWorld: lastPose, intrinsics: input.intrinsics, width: input.image.width,
            height: input.image.height)
        defer {
            previousPyramid = regressed ? nil : pyramid
            if !regressed { lastTimestamp = input.timestamp }
        }

        // 1. Corners tracked over the whole image. The points that carry the body are protected from the rolling
        //    search; the rest expire, so the search keeps looking elsewhere.
        locator.maxPoints = max(1, config.targetTrackCount)
        locator.maxResidual = config.maxResidual
        // This engine has no region of interest to refill, so the rolling search is its only source of corners on
        // the body: it visits the image more often than the tracked-geometry engine needs to.
        locator.detectEvery = 2
        _ = locator.add(
            image: input.image, prev: previousPyramid, cur: pyramid,
            pose: input.poseValid ? input.cameraToWorld : nil, dt: dt, time: input.timestamp,
            radius: Float(config.roiRadiusFraction) * Float(input.image.height), klt: klt, corners: corners,
            retaining: neededIDs)

        // 2. Where those points are in the world, and which of them move against the static scene.
        points.chordStride = config.chordStride
        points.maxDepthJumpFraction = config.maxDepthJumpFraction
        if poseUsable {
            points.observe(
                locator.points, depth: input.depth, view: view, frame: frameIndex, time: input.timestamp,
                minConfidence: config.minDepthConfidence)
        } else {
            points.skipFrame()
        }
        retainedIDs = points.driftingIDs
        journal.trackedPointCount = points.observations.count
        journal.movingPointCount = points.movingCount

        // 3. The axis, then the angle against the map.
        let predicted = filter.predict(dt: dt)
        if let map {
            if poseUsable {
                measure(
                    map, input: input, view: view, predicted: predicted, journal: &journal, capture: captureDiagnostics)
            } else {
                lastMeasurement = nil
            }
            state = frameIndex - lastMeasuredFrame > config.lostAfterFrames ? .lost : .locked
            // Absence is not proof that the body left, and a bounded map costs nothing to keep. Only a body that
            // demonstrably turns about another axis replaces it.
            if state == .lost, poseUsable {
                collectChords(journal: &journal)
                replaceMapIfAnotherAxisTurns(input: input, journal: &journal)
            }
        } else if poseUsable {
            bootstrap(input: input, view: view, journal: &journal)
        } else {
            state = bootstrapStart == nil ? .idle : .calibrating
        }

        var out = EngineOutput()
        out.state = state
        out.marker = marker
        out.axis = map?.axis
        out.theta = filter.theta
        out.angleDegrees = positiveAngle(filter.theta) * 180 / .pi
        out.angleMeasured = lastMeasuredFrame == frameIndex
        out.axisGeneration = axisGeneration
        out.axisStable = map != nil && state == .locked
        out.rpm = filter.omega * 60 / (2 * .pi)
        out.trackCount = map?.count ?? points.observations.count
        out.inlierCount = out.angleMeasured ? lastSupport : 0
        out.constraintCount = chords.count
        out.turnCoverageDegrees = (thetaMax - thetaMin) * 180 / .pi
        out.angleUncertaintyDegrees = filter.sigma * 180 / .pi
        out.angleDispersionDegrees = lastDispersion * 180 / .pi
        out.lastRelocalizationAge = lastReturnFrame < 0 ? -1 : frameIndex - lastReturnFrame
        if let estimate = lastEstimate {
            let quality =
                min(1, estimate.inlierRatio / 0.7) * min(1, estimate.coverage / 0.6)
                * min(1, (0.3 - min(estimate.planarity, 0.3)) / 0.25)
            out.axisQuality = max(0, min(1, quality))
        }
        if let map {
            let extent = map.extent()
            out.objectRadius = extent.radius
            out.heightMin = extent.heightMin
            out.heightMax = extent.heightMax
        }
        if out.angleMeasured {
            var confidence = min(1, Double(lastSupport) / 15)
            confidence *= max(0.2, 1 - lastDispersion / (25 * Double.pi / 180))
            confidence *= exp(-filter.sigma / (20 * Double.pi / 180))
            out.angleConfidence = max(0, min(1, confidence))
        }
        out.tracks = trackDebug()
        out.message = message(out)
        out.processingMillis = Date().timeIntervalSince(started) * 1000
        journal.phase = phaseName()
        journal.landmarkCount = map?.count ?? 0
        journal.omega = filter.omega
        journal.sigma = filter.sigma
        journal.millis = out.processingMillis
        lastDiagnostics = journal
        return out
    }

    // MARK: The axis

    /// Accumulate chords of the moving points and fit the axis as soon as they determine one. The body's own
    /// displacement decides what is moving, so this needs neither a marker nor a sustained crowd of fast points.
    private func bootstrap(input: FrameInput, view: PinholeProjection, journal: inout PersistentMapDiagnostics) {
        if bootstrapStart == nil, points.movingCount >= 6 { bootstrapStart = input.timestamp }
        collectChords(journal: &journal)
        state = bootstrapStart == nil ? .idle : .calibrating
        marker = movingMarker(view: view)
        guard let start = bootstrapStart, input.timestamp - start >= bootstrapSeconds,
            frameIndex % max(1, config.axisUpdateInterval) == 0
        else { return }
        guard let estimate = fitAxis() else {
            journal.axisFit = .init(
                reason: "notEnoughChords", chords: chords.count, inlierRatio: 0, planarity: 1, coverage: 0)
            return
        }
        let determined = determinesAxis(estimate)
        let confirmed =
            determined && previousCandidate.map { AxisStability.agrees(estimate.axis, with: $0, within: 0.02) } ?? false
        journal.axisFit = .init(
            reason: confirmed ? "adopted" : (determined ? "awaitingConfirmation" : "notDetermined"),
            chords: estimate.constraintCount, inlierRatio: estimate.inlierRatio, planarity: estimate.planarity,
            coverage: estimate.coverage, origin: estimate.axis.origin, direction: estimate.axis.direction)
        if determined { previousCandidate = estimate.axis }
        guard confirmed else { return }

        // An axis is a line; the sign of its direction is a choice. Take the sign that makes the body turn the
        // positive way, so the angle and the rate read the same for every body.
        // Only the points that turn about this axis may say which way it turns; anything else that moves would vote
        // on a rotation it has nothing to do with. Their invariants do not depend on the sign, so the same points
        // seed the map after the sign is settled.
        let candidates = circularCandidates(axis: estimate.axis)
        let rate = points.rotationRate(
            origin: estimate.axis.origin, direction: estimate.axis.direction,
            among: Set(candidates.map { $0.id }))
        let axis = rate < 0 ? Axis(origin: estimate.axis.origin, direction: -estimate.axis.direction) : estimate.axis
        journal.axisFit?.seededRate = rate
        let candidate = RigidRotationMap(axis: axis, sampler: sampler)
        candidate.capacity = landmarkCapacity
        candidate.coarseStepLimit = Int((Double.pi / candidate.coarseStep).rounded(.up))
        let seeded = candidate.insert(
            candidates, theta: 0, image: input.image, depth: input.depth, view: view,
            frame: frameIndex, minConfidence: config.minDepthConfidence, limit: seedLimit, occupied: [])
        journal.createdCount = seeded.added
        journal.planeRejectedCount = seeded.planeRejected
        // The map is worth adopting exactly when it could measure the angle: fewer landmarks than the fit needs
        // redundancy for would claim a body without being able to follow it. It is not a budget for a body of a
        // particular size — the map grows from here as the body turns new surfaces towards the camera.
        guard seeded.added >= candidate.minSupport else {
            journal.axisFit?.reason = "tooFewLandmarks"
            return
        }
        map = candidate
        lastEstimate = estimate
        axisGeneration += 1
        filter.reset(theta: 0)
        filter.seed(omega: abs(rate), sigma: max(0.5, abs(rate)))
        thetaMin = 0
        thetaMax = 0
        lastMeasuredFrame = frameIndex
        lastMeasuredTime = input.timestamp
        lastMeasurement = nil
        thetaTrace.removeAll()
        recordAngle(time: input.timestamp, theta: 0)
        state = .locked
    }

    /// Whether the chords determine the axis, judged by their own geometry rather than by how many of them there
    /// are: they must lie in planes perpendicular to one line, point in enough different directions around it, and
    /// mostly agree with the fit. Counting chords would not say this — a thousand chords of one narrow arc still
    /// leave the axis undetermined, and a well-spread hundred already fix it.
    private func determinesAxis(_ estimate: AxisEstimate) -> Bool {
        estimate.constraintCount >= minChords && estimate.inlierRatio >= 0.6 && estimate.planarity < 0.15
            && estimate.coverage > 0.3
    }

    /// Chords of the moving points, kept for as long as the configured window: the evidence an axis is fitted from,
    /// whether none is known yet or the known one is being contradicted.
    private func collectChords(journal: inout PersistentMapDiagnostics) {
        chords.append(contentsOf: points.chordConstraints(minLength: config.chordMinMeters, frame: frameIndex))
        let oldest = frameIndex - config.constraintWindowFrames
        if let first = chords.firstIndex(where: { $0.frame >= oldest }) {
            if first > 0 { chords.removeFirst(first) }
        } else {
            chords.removeAll(keepingCapacity: true)
        }
        if chords.count > config.axisConstraintCapacity {
            chords.removeFirst(chords.count - config.axisConstraintCapacity)
        }
        journal.chordCount = chords.count
    }

    /// The axis the chords agree on: the largest set of them one axis explains, then a reweighted fit of that set.
    /// Anything else that moved in front of the camera contributes chords that no single axis explains, and they are
    /// left out before the fit rather than being averaged into it.
    private func fitAxis(newerThan frame: Int = -1) -> AxisEstimate? {
        let window = frame < 0 ? chords : chords.filter { $0.frame > frame }
        guard window.count >= consensus.minimumChords else { return nil }
        var estimator = AxisEstimator()
        estimator.capacity = config.axisConstraintCapacity
        for chord in consensus.select(window) { estimator.add(chord) }
        return estimator.estimate()
    }

    /// While no angle can be measured, the recent chords are still watched. A well-determined axis that disagrees
    /// with the map's own is a body that is not the mapped one: that, and not mere absence, replaces the map. The
    /// recent chords are kept so the next axis can be fitted at once.
    private func replaceMapIfAnotherAxisTurns(input: FrameInput, journal: inout PersistentMapDiagnostics) {
        guard let map, frameIndex % max(1, config.axisUpdateInterval) == 0,
            let recent = fitAxis(newerThan: frameIndex - config.recentWindowFrames), determinesAxis(recent)
        else { return }
        let radius = map.extent().radius
        let apart =
            !AxisStability.agrees(recent.axis, with: map.axis, within: max(0.05, 0.5 * radius))
            || acos(min(1, abs(recent.axis.direction.dot(map.axis.direction)))) > 25 * Double.pi / 180
        journal.axisFit = .init(
            reason: apart ? "anotherBodyTurns" : "sameAxisWhileUnseen", chords: recent.constraintCount,
            inlierRatio: recent.inlierRatio, planarity: recent.planarity, coverage: recent.coverage,
            origin: recent.axis.origin, direction: recent.axis.direction)
        guard apart else { return }
        self.map = nil
        lastMeasurement = nil
        lastEstimate = nil
        lastMeasuredTime = nil
        marker = nil
        lastSupport = 0
        lastDispersion = 0
        bootstrapStart = input.timestamp - bootstrapSeconds
        previousCandidate = nil
        thetaTrace.removeAll()
        state = .calibrating
    }

    /// Moving points that could belong to a body turning about this axis, before any angle is known: they must keep
    /// a fixed distance to the axis and a fixed height along it while travelling around it. Those are the invariants
    /// of a circle about the axis — necessary, and all that is available until the map measures the angle.
    private func circularCandidates(axis: Axis) -> [TrackedWorldPoints.Observation] {
        points.observations.filter { observation in
            guard observation.moving, let fit = points.axisConsistency(id: observation.id, axis: axis),
                fit.samples >= 3
            else { return false }
            return fit.radiusDrift < max(0.008, 0.1 * fit.radius) && fit.heightDrift < 0.012
                && abs(fit.travel) > 3 * Double.pi / 180
        }
    }

    /// Moving points the body's own rotation explains better than anything else: taking each retained position back
    /// by the angle the body had at that moment must land them on one place, and must do so more tightly than
    /// leaving them where they are in the world. The comparison is what carries the argument — a tangential slide
    /// can sit inside any absolute tolerance over a short baseline, but it is then explained better by standing
    /// still than by the body's rotation, and only a longer baseline separates the two outright.
    private func mappedCandidates(_ map: RigidRotationMap) -> [TrackedWorldPoints.Observation] {
        points.observations.filter { observation in
            guard observation.moving,
                let fit = points.rigidResidual(id: observation.id, axis: map.axis, angleAt: angle(at:)),
                fit.span >= 0.1, fit.samples >= 3
            else { return false }
            let radius = map.axis.cylindrical(observation.world).radius
            return fit.spread < max(rigidSpreadMeters, 0.06 * radius) && fit.spread < fit.worldSpread
                && abs(fit.travel) > 3 * Double.pi / 180
        }
    }

    /// The body's angle at a past moment, interpolated from the measured angles; nil outside what they cover.
    private func angle(at time: Double) -> Double? {
        guard let first = thetaTrace.first, let last = thetaTrace.last, time >= first.time - 1e-6,
            time <= last.time + 1e-6
        else { return nil }
        guard let index = thetaTrace.firstIndex(where: { $0.time >= time }) else { return last.theta }
        if index == 0 { return thetaTrace[0].theta }
        let before = thetaTrace[index - 1], after = thetaTrace[index]
        let span = after.time - before.time
        guard span > 1e-9 else { return after.theta }
        return before.theta + (after.theta - before.theta) * (time - before.time) / span
    }

    private func recordAngle(time: Double, theta: Double) {
        thetaTrace.append((time, theta))
        thetaTrace.removeAll { time - $0.time > thetaTraceSeconds }
        if thetaTrace.count > thetaTraceLimit {
            thetaTrace = thetaTrace.enumerated().filter { $0.offset % 2 == 0 || $0.offset == thetaTrace.count - 1 }
                .map { $0.element }
        }
    }

    // MARK: The angle

    private func measure(
        _ map: RigidRotationMap, input: FrameInput, view: PinholeProjection, predicted: Double,
        journal: inout PersistentMapDiagnostics, capture: Bool
    ) {
        // Identity first, from the tracks that are still alive: they say which pixel carries which landmark without
        // consulting the estimate, so the whole circle can be scored and what wins wins against every other angle
        // rather than against the shoulders of a window. Only when too few landmarks are still carried do the
        // patches have to say who is who.
        //
        // Either way an angle is measured modulo a full turn, so the same reference test applies to both: once the
        // prediction is worth less than the widest ordinary search, which turn the answer belongs to is not
        // observed, and what the frame offers is a new reference rather than the old angle continued.
        journal.predictedTheta = predicted
        let referenceLost = 3 * filter.sigma + map.coarseStep > maxSearchWindow

        journal.referenceLost = referenceLost
        let ambiguous = referenceLost ? relockAmbiguityLimit : ambiguityLimit
        let needed = referenceLost ? 2 * map.minSupport : map.minSupport
        var accepted: RigidRotationMap.Measurement?
        let track = map.measureByTrack(
            observations: points.observations, predicted: predicted, image: input.image, view: view,
            depth: input.depth, minConfidence: config.minDepthConfidence)
        journal.boundCount = track.bound
        if track.bound >= map.minSupport {
            // The tracks are the better evidence and they have already been scored against the whole circle. A
            // rival they could not tell apart is a fact about this frame, not a reason to ask a narrower question.
            journal.association = .track
            journal.searchWindow = track.measurement?.searchWindow ?? .pi
            journal.visibleCount = track.measurement?.visible ?? 0
            journal.ambiguity = track.measurement?.ambiguity ?? 0
            if let measured = track.measurement, measured.ambiguity < ambiguous, measured.support >= needed {
                accepted = measured
            }
        } else {
            // Too few landmarks are carried for the tracks to say who is who, so the patches have to. Then the
            // whole circle is scored however small the uncertainty is: a peak inside a narrow window is no evidence
            // that no other angle explains the frame as well, and it is exactly the material ambiguity of a patch
            // that a prior would hide. The prediction may choose the turn; it may not choose the angle.
            journal.association = .appearance
            let appearance = map.measure(
                image: input.image, depth: input.depth, view: view, predicted: predicted, window: .pi,
                minConfidence: config.minDepthConfidence)
            journal.searchWindow = appearance?.searchWindow ?? .pi
            journal.visibleCount = appearance?.visible ?? 0
            journal.ambiguity = appearance?.ambiguity ?? 0
            if let appearance, appearance.ambiguity < ambiguous, appearance.support >= needed {
                accepted = appearance
            }
        }
        guard let measurement = accepted else {
            lastMeasurement = nil
            return
        }
        if referenceLost {
            filter.reset(theta: measurement.theta, thetaSigma: 2 * Double.pi / 180, omegaSigma: 1)
            axisGeneration += 1
            thetaMin = measurement.theta
            thetaMax = measurement.theta
            journal.relocked = true
        } else {
            filter.correct(theta: measurement.theta, information: measurement.information)
        }
        let theta = filter.theta
        if journal.relocked { thetaTrace.removeAll() }
        recordAngle(time: input.timestamp, theta: theta)
        let seen = map.markSeen(measurement, frame: frameIndex, gapFrames: gapFrames, recordIdentities: capture)
        if seen.returned > 0 { lastReturnFrame = frameIndex }
        let grown = map.insert(
            mappedCandidates(map), theta: theta, image: input.image, depth: input.depth, view: view,
            frame: frameIndex, minConfidence: config.minDepthConfidence, limit: insertLimit,
            occupied: measurement.occupied)
        let binding = map.releaseTracksThatDisagree(with: points.observations, measurement: measurement)
        journal.boundCount = binding.bound
        journal.trackEndedCount = binding.ended
        journal.trackContradictedCount = binding.contradicted
        journal.filteredTheta = theta
        journal.measuredTheta = measurement.theta
        journal.supportCount = measurement.support
        journal.reprojectionRMS = measurement.reprojectionRMS
        journal.returnedCount = seen.returned
        journal.returnedIDs = seen.returnedIDs
        journal.observedIDs = seen.observedIDs
        journal.createdCount = grown.added
        journal.planeRejectedCount = grown.planeRejected
        journal.evictedCount = map.trim(frame: frameIndex)
        lastMeasurement = measurement
        lastMeasuredFrame = frameIndex
        lastMeasuredTime = input.timestamp
        lastSupport = measurement.support
        lastDispersion = measurement.dispersion
        thetaMin = min(thetaMin, theta)
        thetaMax = max(thetaMax, theta)
        marker = Marker(
            x: Float(measurement.centerX), y: Float(measurement.centerY),
            radius: Float(max(24, min(view.height / 2, 2 * measurement.spreadPixels))))
    }

    // MARK: Reporting

    private func phaseName() -> String {
        if map == nil { return bootstrapStart == nil ? "searching" : "bootstrapping" }
        return lastMeasuredFrame == frameIndex ? "mapping" : "holding"
    }

    /// Where the moving points are while no axis is known yet, so the display has something honest to show.
    private func movingMarker(view: PinholeProjection) -> Marker? {
        let moving = points.observations.filter { $0.moving }
        guard moving.count >= 6 else { return nil }
        let x = moving.reduce(0) { $0 + $1.x } / Double(moving.count)
        let y = moving.reduce(0) { $0 + $1.y } / Double(moving.count)
        var spread = 0.0
        for point in moving { spread += (point.x - x) * (point.x - x) + (point.y - y) * (point.y - y) }
        spread = (spread / Double(moving.count)).squareRoot()
        return Marker(x: Float(x), y: Float(y), radius: Float(max(24, min(view.height / 2, 2 * spread))))
    }

    /// The landmarks that agreed, those seen and contradicting, and the moving points not yet in the map. The
    /// pixels come from the measurement itself, so trimming the map cannot shift them.
    private func trackDebug() -> [TrackDebug] {
        var out: [TrackDebug] = []
        if let measurement = lastMeasurement {
            for pixel in measurement.supportingPixels {
                out.append(TrackDebug(x: Float(pixel.x), y: Float(pixel.y), status: .good))
            }
            for pixel in measurement.contradictingPixels where out.count < trackDebugLimit {
                out.append(TrackDebug(x: Float(pixel.x), y: Float(pixel.y), status: .inconsistent))
            }
        }
        for observation in points.observations where observation.moving && out.count < trackDebugLimit {
            out.append(TrackDebug(x: Float(observation.x), y: Float(observation.y), status: .young))
        }
        return out
    }

    private func message(_ out: EngineOutput) -> String {
        switch out.state {
        case .idle:
            return "Experimental persistent map: looking for a body that turns…"
        case .calibrating:
            return String(format: "Experimental persistent map: finding the axis, %d chords", out.constraintCount)
        case .locked where out.angleMeasured:
            return String(
                format: "Experimental persistent map: %d landmarks, %d agree, ±%.1f° (model)", out.trackCount,
                out.inlierCount, out.angleUncertaintyDegrees)
        case .locked:
            return String(
                format: "Experimental persistent map: holding the angle, ±%.1f° (model)",
                out.angleUncertaintyDegrees)
        case .lost:
            return lastDiagnostics?.referenceLost == true
                ? "Experimental persistent map: angle reference lost, searching the whole turn"
                : String(
                    format: "Experimental persistent map: body not seen, ±%.1f° (model)",
                    out.angleUncertaintyDegrees)
        }
    }
}
