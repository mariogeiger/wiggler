import Foundation

/// Orchestrates tracking, axis estimation, angle integration and relocalisation. Not thread-safe: call `process`
/// from one queue.
public final class RotationEngine {
    public var config: EngineConfig

    private var diagnostics: EngineDiagnosticsRecorder?
    private var capturedPreviousFrame = false
    private let tracks = PointTracks()
    private var prevPyramid: Pyramid?
    private var frameIndex = 0
    private var klt = KLTTracker()
    private var corners = CornerDetector()
    private var estimator = AxisEstimator()
    private var angle = AngleTracker()
    private var reloc: Relocalizer
    private var state: EngineState = .idle

    private var marker: (x: Float, y: Float)?
    private var locator = MotionLocator()
    private var lostSince: Double?
    private var axis: Axis?
    private var displayedAxis: Axis?
    private var stability = AxisStability()
    private var lastEstimate: AxisEstimate?
    private var inconsistentAxisCount = 0
    private var calibrationStartFrame = 0
    private var thetaMin = 0.0, thetaMax = 0.0
    private var lastAngleOkFrame = -1000
    private var lastRelocFrame = -1000
    private var fusion = AngleFusion()
    /// Ratio between the image-plane rotation and the 3D azimuth increment, learned online (it depends only on
    /// how the axis is tilted with respect to the camera, so it is constant for a fixed phone).
    private var imageScale: Double?
    private var disagreeStreak = 0
    private var lastPose = RigidTransform.identity
    private var lastTimestamp: Double?
    private var rpmFiltered = 0.0
    private var lastInlierCount = 0
    private var lastDispersion = 0.0
    private var lastAngleOk = false
    private var axisGeneration = 0

    public init(config: EngineConfig = EngineConfig()) {
        self.config = config
        self.reloc = Relocalizer(side: config.descriptorSide, binCount: config.keyframeBins)
        klt.halfWindow = 4
        corners.minDistance = 8
        estimator.capacity = config.axisConstraintCapacity
    }

    // MARK: Marker

    private func beginTracking(x: Float, y: Float) {
        let previousAxis = axis ?? displayedAxis
        marker = (x, y)
        recordEvent(action: "resetMeasurement", cause: "movingRegionSelected")
        resetMeasurement()
        displayedAxis = previousAxis
        state = .calibrating
    }

    private func clearTracking() {
        recordEvent(action: "resetMeasurement", cause: "reacquisitionTimeout")
        marker = nil
        resetMeasurement()
        state = .idle
    }

    private func resetMeasurement() {
        axisGeneration += 1
        tracks.removeAll(diagnostics: diagnostics)
        displayedAxis = nil
        estimator.removeAll()
        angle.reset()
        reloc.reset()
        axis = nil
        stability.reset()
        lastEstimate = nil
        inconsistentAxisCount = 0
        calibrationStartFrame = frameIndex
        thetaMin = 0
        thetaMax = 0
        lastAngleOkFrame = -1000
        lastRelocFrame = -1000
        fusion.reset()
        imageScale = nil
        disagreeStreak = 0
        rpmFiltered = 0
        lastInlierCount = 0
        lastAngleOk = false
        lostSince = nil
    }

    private func restartCalibration(cause: String) {
        recordEvent(action: "resetMeasurement", cause: cause)
        let previousAxis = axis ?? displayedAxis
        resetMeasurement()
        displayedAxis = previousAxis
        state = .calibrating
    }

    /// Include warm observations on the next captured frame, even when no uncaptured frame separates sessions.
    /// Call on the same queue as `process`. This changes diagnostics bookkeeping only.
    public func beginDiagnosticCapture() {
        capturedPreviousFrame = false
    }

    // MARK: Main entry point

    public func process(_ input: FrameInput, captureDiagnostics: Bool = false) -> EngineOutput {
        let t0 = Date()
        beginDiagnostics(input, capture: captureDiagnostics)
        defer {
            diagnostics = nil
            locator.diagnostics = nil
        }
        frameIndex += 1
        var out = EngineOutput()
        let pyramid = Pyramid(image: input.image, levelCount: config.pyramidLevels)
        if input.poseValid { lastPose = input.cameraToWorld }
        let pose = lastPose
        defer {
            prevPyramid = pyramid
            lastTimestamp = input.timestamp
        }
        let frameDT = lastTimestamp.map { input.timestamp - $0 } ?? 0
        let dt = frameDT > 0 && frameDT < 0.5 ? frameDT : 1.0 / 60.0

        diagnostics?.value.frameDT = frameDT
        diagnostics?.value.integrationDT = dt
        diagnostics?.value.poseUsed = pose

        let roiRadius = Float(config.roiRadiusFraction) * Float(input.image.height)
        let retainedIDs = tracks.ids
        locator.maxPoints = config.targetTrackCount + config.explorationPoints
        locator.maxResidual = config.maxResidual
        // Tracks die in bursts when the object accelerates; below half the target, refill the region every frame.
        let refill = tracks.count < config.targetTrackCount / 2 ? marker.map { ($0.x, $0.y, roiRadius) } : nil
        let hit = locator.add(
            image: input.image, prev: prevPyramid, cur: pyramid,
            pose: input.poseValid ? input.cameraToWorld : nil, dt: frameDT, time: input.timestamp,
            radius: roiRadius, klt: klt, corners: corners, retaining: retainedIDs, refill: refill)
        diagnostics?.value.locator = locator.diagnosticState()
        if let hit {
            if let m = marker {
                let nearbyMoving = locator.points.filter {
                    let dx = $0.x - m.x, dy = $0.y - m.y
                    return $0.speed > locator.movingSpeed && dx * dx + dy * dy < roiRadius * roiRadius
                }.count
                let dx = hit.x - m.x, dy = hit.y - m.y
                if nearbyMoving < locator.minMoving && dx * dx + dy * dy > roiRadius * roiRadius / 4 {
                    beginTracking(x: hit.x, y: hit.y)
                }
            } else {
                beginTracking(x: hit.x, y: hit.y)
            }
        }
        guard let marker = marker.map({ Marker(x: $0.x, y: $0.y, radius: roiRadius) }) else {
            out.state = .idle
            diagnostics?.value.depthSamplingSkipped = "idle"
            out.message = "Looking for a moving object… \(locator.movingCount) moving points"
            out.processingMillis = Date().timeIntervalSince(t0) * 1000
            return finishDiagnostics(out)
        }

        // 1. Reuse the motion search's correspondences; each corner is tracked only once per frame. While the
        //    body turns, its rim moves at ω·R even when its inner points are still: nearby motion is its own.
        let rimSpeed = abs(angle.lastDelta) / dt * Double(tracks.imageRadius(aroundX: marker.x, y: marker.y))
        let bodyMoving = axis != nil && rimSpeed > Double(locator.movingSpeed) / 2
        let selected = locator.selected(
            around: marker, retaining: tracks.ids, bodyMoving: bodyMoving, limit: config.targetTrackCount)
        diagnostics?.value.selectedTrackIDs = selected.map { $0.id }
        if !locator.motionValid {
            restartCalibration(cause: "cameraMotionVeto")
            diagnostics?.value.depthSamplingSkipped = "cameraMotionVeto"
        }
        let (before, after) = tracks.update(selected, diagnostics: diagnostics)
        angle.removeAllExcept(ids: tracks.ids)

        // 2. Depth → 3D samples in world coordinates.
        if locator.motionValid {
            tracks.sampleDepth(
                input, pose: pose, frame: frameIndex, minConfidence: config.minDepthConfidence,
                maxJumpFraction: config.maxDepthJumpFraction, historyLength: config.sampleHistory,
                diagnostics: diagnostics)
        }

        // 3. Chord constraints for the axis.
        let objectRadius = currentObjectRadius()
        let dmin = axis == nil ? config.chordMinMeters : min(0.05, max(0.01, 0.1 * objectRadius))
        diagnostics?.value.chordMinLengthMeters = dmin
        let chords = tracks.chordConstraints(
            frame: frameIndex, minLength: dmin,
            maxFrames: config.chordMaxFrames, stride: config.chordStride, diagnostics: diagnostics)
        for chord in chords { estimator.add(chord) }
        estimator.prune(before: frameIndex - config.constraintWindowFrames)

        // 4. Angle.
        diagnostics?.value.geometry = .init(
            minHealthyInliers: config.minHealthyInliers, angleGate: angle.gate, angleSigma: angle.sigma,
            angleMinInliers: angle.minInliers, angleMaxBadFrames: angle.maxBadFrames)
        var angleOk = false
        if let ax = axis {
            let observations = tracks.project(around: ax)
            diagnostics?.value.geometry?.observations = observations
            let up = angle.update(observations, diagnostics: diagnostics)
            diagnostics?.value.geometry?.update = up
            angleOk = up.ok
            lastInlierCount = up.inlierCount
            lastDispersion = up.dispersion
            if up.ok {
                lastAngleOkFrame = frameIndex
                thetaMin = min(thetaMin, up.theta)
                thetaMax = max(thetaMax, up.theta)
                let rpmInst = up.delta / dt * 60 / (2 * .pi)
                rpmFiltered += 0.15 * (rpmInst - rpmFiltered)
            } else {
                rpmFiltered *= 0.9
            }
        }
        lastAngleOk = angleOk

        // 4b. Cross-check: the image-plane rotation of the same correspondences uses neither depth nor the axis,
        //     so it fails in different situations than the 3D azimuth. When the two disagree, something else has
        //     taken over the tracked points (a hand, a reflection) and the geometric angle must not be trusted,
        //     however many "inliers" it reports.
        var agrees = true
        let similarity = similarityRotation(from: before, to: after)
        diagnostics?.value.geometry?.similarityAngle = similarity?.angle
        diagnostics?.value.geometry?.similarityInliers = similarity?.inliers
        if let sim = similarity, sim.inliers >= 8 {
            if angleOk, abs(angle.lastDelta) > Double.pi / 180 {
                let ratio = sim.angle / angle.lastDelta
                imageScale = imageScale.map { 0.9 * $0 + 0.1 * ratio } ?? ratio
            }
            if let scale = imageScale {
                let predicted = scale * angle.lastDelta
                agrees = abs(sim.angle - predicted) < 3 * Double.pi / 180 + 0.35 * abs(predicted)
                diagnostics?.value.geometry?.predictedImageAngle = predicted
                diagnostics?.value.geometry?.crossCheckLimit = 3 * Double.pi / 180 + 0.35 * abs(predicted)
            }
        }
        disagreeStreak = agrees ? 0 : disagreeStreak + 1
        let geometryHealthy = angleOk && lastInlierCount >= config.minHealthyInliers && disagreeStreak < 3
        diagnostics?.value.geometry?.similarityScale = imageScale
        diagnostics?.value.geometry?.agrees = agrees
        diagnostics?.value.geometry?.disagreeStreak = disagreeStreak
        diagnostics?.value.geometry?.healthy = geometryHealthy

        // 5. Axis refinement — deliberately *after* the angle update. Re-anchoring the per-track offsets first
        //    would make every candidate agree with the current angle, silently throwing away one frame of motion
        //    every `axisUpdateInterval` frames (a systematic ~7 % under-estimation of the rotation).
        if frameIndex % max(1, config.axisUpdateInterval) == 0,
            stability.hasNewEvidence(frame: estimator.newestFrame)
        {
            let evidenceFrame = estimator.newestFrame
            let estimate = estimator.estimate()
            diagnostics?.value.axisEstimation = .init(
                evidenceFrame: evidenceFrame, estimate: estimate.map { .init($0) },
                huberMeters: estimator.huberMeters, iterations: estimator.iterations, objectRadius: objectRadius,
                consistencyDistanceLimit: max(0.05, 0.5 * objectRadius),
                farDistanceLimit: max(0.15, 3 * objectRadius))
            if let est = estimate, est.isWellConditioned {
                lastEstimate = est
                let consistent =
                    axis.map { AxisStability.agrees(est.axis, with: $0, within: consistencyDistance(objectRadius)) }
                    ?? false
                diagnostics?.value.axisEstimation?.consistent = consistent
                updateAxis(with: est, objectRadius: objectRadius)
                stability.observe(frame: evidenceFrame, consistent: consistent)
            } else {
                recordEvent(
                    action: "invalidateStability",
                    cause: estimate == nil ? "axisEstimateUnavailable" : "axisEstimateIllConditioned")
                stability.invalidate()
            }
            // Has the object moved? Only the recent chords can say so before the old ones are outvoted.
            if let ax = axis {
                let since = frameIndex - config.recentWindowFrames
                let recent = estimator.estimate(newerThan: since)
                let distance = max(config.axisMovedMeters, 0.2 * objectRadius)
                let moved = stability.observe(
                    recent: recent, axis: ax, within: distance, required: config.lockedDriftFrames)
                diagnostics?.value.recentAxis = .init(
                    sinceFrame: since, estimate: recent.map { .init($0) }, distanceLimit: distance,
                    agrees: recent.map { AxisStability.agrees($0.axis, with: ax, within: distance) },
                    contradictions: stability.contradictions, moved: moved)
                if moved, let recent {
                    let far =
                        !AxisStability.agrees(recent.axis, with: ax, within: max(0.15, 3 * objectRadius))
                        || acos(min(1, abs(recent.axis.direction.dot(ax.direction)))) > 25 * .pi / 180
                    if far {
                        restartCalibration(cause: "recentAxisFar")
                    } else {
                        adopt(recent.axis, since: since, cause: "recentAxisMoved")
                    }
                }
            }
        }

        // 6. State transitions and relocalisation.
        switch state {
        case .calibrating:
            if axis != nil, let est = lastEstimate, est.isWellConditioned, thetaMax - thetaMin >= 2 * .pi {
                recordEvent(action: "resetAppearanceAndFusion", cause: "fullTurnLocked")
                state = .locked
                reloc.reset()
                fusion.reset()
            }
        case .locked, .lost:
            if frameIndex - lastAngleOkFrame > config.lostAfterFrames {
                state = .lost
            } else if state == .lost && angleOk {
                state = .locked
            }
            if state == .lost {
                if lostSince == nil { lostSince = input.timestamp }
                if input.timestamp - lostSince! > config.reacquisitionDelaySeconds {
                    // The object is gone: look for a moving one again.
                    clearTracking()
                    out.state = .idle
                    out.processingMillis = Date().timeIntervalSince(t0) * 1000
                    return finishDiagnostics(out)
                }
            } else {
                lostSince = nil
            }
            relocalise(
                image: input.image, marker: marker, angleOk: angleOk,
                healthy: geometryHealthy, omega: angle.lastDelta / dt, dt: dt)
        case .idle:
            break
        }

        // 8. Output.
        out.state = state
        out.marker = marker
        out.axis = axis ?? displayedAxis
        // A statement about the axis alone: confirmed by fresh estimates and not contradicted by recent chords.
        // How well the angle is measured right now is the ray's business (`angleConfidence`), not the axis's.
        out.axisStable = axis != nil && state == .locked && stability.isStable(required: config.lockedDriftFrames)
        out.angleMeasured = angleOk
        out.axisGeneration = axisGeneration
        out.theta = angle.theta
        out.angleDegrees = positiveAngle(angle.theta) * 180 / .pi
        out.rpm = rpmFiltered
        out.trackCount = tracks.count
        out.inlierCount = lastInlierCount
        out.constraintCount = estimator.constraints.count
        out.turnCoverageDegrees = (thetaMax - thetaMin) * 180 / .pi
        out.relocalizerFill = reloc.fillRatio
        out.periodDegrees = reloc.analysed ? reloc.period * 180 / .pi : 360
        if let est = lastEstimate {
            let q =
                min(1, est.inlierRatio / 0.7) * min(1, est.coverage / 0.6)
                * min(1, (0.3 - min(est.planarity, 0.3)) / 0.25)
            out.axisQuality = max(0, min(1, q))
        }
        let (radii, heights) = tracks.extent { angle.isConsistent(id: $0) }
        out.objectRadius = radii.isEmpty ? objectRadius : percentile(radii, 0.8)
        if !heights.isEmpty {
            out.heightMin = percentile(heights, 0.05)
            out.heightMax = percentile(heights, 0.95)
        }
        if state == .locked {
            // Confidence is now a direct reading of the filter: how many points agree, how tightly, and how
            // uncertain the absolute angle is after gating the appearance measurements.
            var conf = min(1, Double(lastInlierCount) / 15.0)
            conf *= max(0.2, 1 - lastDispersion / (25 * Double.pi / 180))
            conf *= exp(-fusion.sigma / (20 * Double.pi / 180))
            if reloc.isRotationallySymmetric { conf *= 0.4 }
            out.angleConfidence = angleOk ? max(0, min(1, conf)) : 0
        }
        out.tracks = tracks.debug(hasAxis: axis != nil) { angle.isConsistent(id: $0) }
        out.angleDispersionDegrees = lastDispersion * 180 / .pi
        out.relocalizerAnalysed = reloc.analysed
        out.lastRelocalizationAge = lastRelocFrame < 0 ? -1 : frameIndex - lastRelocFrame
        out.angleUncertaintyDegrees = fusion.sigma * 180 / .pi
        out.message = statusMessage(out)
        out.processingMillis = Date().timeIntervalSince(t0) * 1000
        return finishDiagnostics(out)
    }

    // MARK: Helpers

    private func currentObjectRadius() -> Double {
        guard axis != nil else { return 0.1 }
        return tracks.objectRadius
    }

    /// How far a full-window estimate may sit from the axis it refines (it lags a moved object).
    private func consistencyDistance(_ objectRadius: Double) -> Double { max(0.05, 0.5 * objectRadius) }

    /// The object was displaced a little (chair rolled, axis re-estimated more precisely): take the new axis,
    /// keep the angle continuous, and forget the chords of the old one. Everything built on the old geometry
    /// — appearance library, fusion, harmonic maps — starts over.
    private func adopt(_ newAxis: Axis, since frame: Int, cause: String) {
        guard let old = axis else { return }
        recordEvent(action: "adoptAxis", cause: cause)
        estimator.prune(before: frame + 1)
        blend(into: old, target: newAxis.alignedSign(with: old.direction), alpha: 1)
        stability.reset()
        inconsistentAxisCount = 0
        axisGeneration += 1
        reloc.reset()
        fusion.reset()
    }

    private func updateAxis(with est: AxisEstimate, objectRadius: Double) {
        var newAxis = est.axis
        if let old = axis {
            newAxis = newAxis.alignedSign(with: old.direction)
            let angleBetween = acos(max(-1, min(1, newAxis.direction.dot(old.direction))))
            // Distance between the two axis lines, measured near the object.
            let d = newAxis.origin - old.origin
            let perp = d - old.direction * d.dot(old.direction)
            let lineDistance = perp.length
            let consistent = AxisStability.agrees(newAxis, with: old, within: consistencyDistance(objectRadius))
            diagnostics?.value.axisEstimation?.angleBetween = angleBetween
            diagnostics?.value.axisEstimation?.lineDistance = lineDistance
            if state == .calibrating {
                // Follow the estimate closely while calibrating.
                diagnostics?.value.axisEstimation?.action = "calibrationBlend"
                blend(into: old, target: newAxis, alpha: 0.5)
                inconsistentAxisCount = 0
            } else if consistent {
                diagnostics?.value.axisEstimation?.action = "consistentBlend"
                blend(into: old, target: newAxis, alpha: 0.15)
                inconsistentAxisCount = 0
            } else {
                diagnostics?.value.axisEstimation?.action = "inconsistentEstimateHeld"
                inconsistentAxisCount += 1
                if inconsistentAxisCount >= config.lockedDriftFrames {
                    inconsistentAxisCount = 0
                    let farOff = angleBetween > 25 * .pi / 180 || lineDistance > max(0.15, 3 * objectRadius)
                    if farOff {
                        // The object was replaced / moved a lot: start over.
                        diagnostics?.value.axisEstimation?.action = "farAxisRestart"
                        restartCalibration(cause: "farAxisEstimate")
                    } else {
                        diagnostics?.value.axisEstimation?.action = "driftAdopted"
                        adopt(newAxis, since: frameIndex - config.recentWindowFrames, cause: "fullAxisDrift")
                    }
                }
            }
        } else {
            // First estimate: orient the axis so that it points roughly "up" in world space (y up in ARKit).
            diagnostics?.value.axisEstimation?.action = "firstAxis"
            axis = newAxis.alignedSign(with: V3(0, 1, 0))
            rebaseAngle()
        }
    }

    private func blend(into old: Axis, target: Axis, alpha: Double) {
        let dir = (old.direction + (target.direction - old.direction) * alpha).normalized
        // Move the origin toward the target line, but keep it at the old height along the axis.
        let d = target.origin - old.origin
        let perp = d - old.direction * d.dot(old.direction)
        let origin = old.origin + perp * alpha
        // Keep the in-plane basis continuous: re-project the old e1 onto the new plane.
        var ax = Axis(origin: origin, direction: dir)
        let e1 = (old.e1 - dir * old.e1.dot(dir)).normalized
        if e1.length > 0.5 {
            ax.e1 = e1
            ax.e2 = dir.cross(e1)
        }
        axis = ax
        rebaseAngle()
    }

    private func rebaseAngle() {
        guard let ax = axis else { return }
        recordEvent(action: "rebaseAngle", cause: "axisUpdated")
        angle.rebase(tracks.observations(around: ax))
    }

    /// Fuse the appearance measurement into the integrated angle. Returns nothing: everything it does is either
    /// a bounded correction of `angle`, or bookkeeping on the library.
    private func relocalise(
        image: GrayImage, marker: Marker,
        angleOk: Bool, healthy: Bool, omega: Double, dt: Double
    ) {
        let patch = image.patch(
            centerX: marker.x, centerY: marker.y, halfSize: marker.radius, side: config.descriptorSide)
        // Fill the library while the geometry is clean. A partially filled library is already useful.
        if angleOk && lastDispersion < 6 * Double.pi / 180 && lastInlierCount >= 8 && !reloc.isComplete {
            reloc.record(patch: patch, theta: angle.theta)
        }
        guard reloc.isUsable else { return }

        let candidates = reloc.candidates(patch: patch, nearTheta: angle.theta)
        let (correction, outcome) = fusion.update(
            theta: angle.theta, omega: omega, healthy: healthy,
            candidates: candidates, dt: dt)
        diagnostics?.value.relocalization = .init(candidates: candidates, correction: correction, outcome: outcome)
        if correction != 0 { angle.shift(by: correction) }
        if outcome == .updated { lastRelocFrame = frameIndex }
        // Keep the library in step with the current appearance, but only on a match we actually believe.
        if healthy && lastDispersion < 6 * Double.pi / 180 && fusion.lastMatchTrusted {
            reloc.refresh(patch: patch, theta: angle.theta, alpha: config.relocRefreshAlpha)
        }
        // A strong match that keeps contradicting a healthy geometry means the library, not the geometry, is
        // wrong: the object was displaced or the light changed. Rebuild it instead of fighting it.
        if fusion.staleSeconds > config.staleLibrarySeconds {
            recordEvent(action: "resetAppearance", cause: "staleLibrary")
            reloc.reset()
            fusion.clearStale()
        }
    }

    private func statusMessage(_ out: EngineOutput) -> String {
        switch out.state {
        case .idle: return "Looking for a moving object…"
        case .calibrating:
            if out.trackCount < 10 { return "Not enough texture / depth around the marker" }
            if out.constraintCount < 150 { return "Rotate the object…" }
            if axis == nil { return "Finding the axis… (\(out.constraintCount) chords)" }
            return String(format: "Provisional axis — full turn: %.0f°/360°", min(out.turnCoverageDegrees, 360))
        case .locked:
            if !reloc.isComplete {
                return String(format: "Axis locked — learning appearance %.0f %%", reloc.fillRatio * 100)
            }
            if reloc.isRotationallySymmetric { return "Rotationally symmetric object: relative angle only" }
            if reloc.period < 2 * .pi - 1e-6 {
                return String(format: "Tracking — appearance period %.0f°", reloc.period * 180 / .pi)
            }
            return "Tracking"
        case .lost: return "Tracking lost — clear the view or rotate the object"
        }
    }
}

extension RotationEngine {
    private func beginDiagnostics(_ input: FrameInput, capture: Bool) {
        defer { capturedPreviousFrame = capture }
        guard capture else { return }
        let before = diagnosticState()
        var value = EngineDiagnostics(
            frameIndex: frameIndex + 1, timestamp: input.timestamp, config: config, before: before, after: before)
        if !capturedPreviousFrame {
            value.initialContext = .init(
                previousFrameIndex: frameIndex, previousTimestamp: lastTimestamp, lastPose: lastPose,
                previousImageWidth: prevPyramid?.levels.first?.width,
                previousImageHeight: prevPyramid?.levels.first?.height,
                trackHistories: tracks.diagnosticHistories(), constraints: estimator.constraints,
                locator: locator.diagnosticState())
        }
        diagnostics = EngineDiagnosticsRecorder(value)
        locator.diagnostics = diagnostics
    }

    private func finishDiagnostics(_ output: EngineOutput) -> EngineOutput {
        guard let diagnostics else { return output }
        diagnostics.value.after = diagnosticState()
        var output = output
        output.diagnostics = diagnostics.value
        return output
    }

    private func recordEvent(action: String, cause: String) {
        diagnostics?.value.events.append(.init(action: action, cause: cause, state: diagnosticState()))
    }

    private func diagnosticState() -> EngineDiagnostics.DecisionState {
        .init(
            state: state, axis: axis, displayedAxis: displayedAxis, marker: marker.map { [$0.x, $0.y] },
            confirmations: stability.confirmations, newestEvidenceFrame: stability.newestFrame,
            contradictions: stability.contradictions, inconsistentAxisCount: inconsistentAxisCount,
            axisGeneration: axisGeneration,
            calibrationStartFrame: calibrationStartFrame, theta: angle.theta, lastDelta: angle.lastDelta,
            thetaMin: thetaMin, thetaMax: thetaMax, lastAngleOkFrame: lastAngleOkFrame, lastRelocFrame: lastRelocFrame,
            lastAngleOk: lastAngleOk, inlierCount: lastInlierCount, dispersion: lastDispersion,
            imageScale: imageScale, disagreeStreak: disagreeStreak, rpmFiltered: rpmFiltered, lostSince: lostSince,
            constraintCount: estimator.constraints.count, oldestConstraintFrame: estimator.oldestFrame,
            newestConstraintFrame: estimator.newestFrame, lastEstimate: lastEstimate.map { .init($0) },
            tracks: tracks.diagnosticState(), angleTracks: angle.diagnosticTracks(),
            fusionSigma: fusion.sigma, fusionPending: fusion.pending, fusionStaleSeconds: fusion.staleSeconds,
            fusionLastMatchTrusted: fusion.lastMatchTrusted, relocalizerFill: reloc.fillRatio,
            relocalizerPeriod: reloc.period, relocalizerAnalysed: reloc.analysed)
    }
}
