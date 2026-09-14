import Foundation

// MARK: - Inputs

public struct CameraIntrinsics {
    /// Focal lengths and principal point in the pixel units of the image handed to the engine.
    public var fx: Double, fy: Double, cx: Double, cy: Double
    public init(fx: Double, fy: Double, cx: Double, cy: Double) { self.fx = fx; self.fy = fy; self.cx = cx; self.cy = cy }
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
        self.width = width; self.height = height; self.depth = depth; self.confidence = confidence
    }

    /// Robust depth at normalised image coordinates (u, v ∈ [0,1]): median of the confident 3x3 neighbourhood,
    /// rejected when the neighbourhood straddles a depth edge.
    public func sample(u: Double, v: Double, minConfidence: UInt8) -> Double? {
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
                if let c = confidence, c[i] < minConfidence { continue }
                let d = Double(depth[i])
                if d.isFinite && d > 0.05 { vals.append(d) }
            }
        }
        if vals.count < 4 { return nil }
        vals.sort()
        let med = vals[vals.count / 2]
        // Depth edge: the neighbourhood spans more than 15 % of the depth.
        if vals[vals.count - 1] - vals[0] > 0.15 * med { return nil }
        return med
    }
}

public struct FrameInput {
    public var image: GrayImage
    public var intrinsics: CameraIntrinsics
    /// Camera-to-world rigid transform (ARKit camera convention: x right, y up, z backward).
    public var cameraToWorld: RigidTransform
    public var poseValid: Bool
    public var depth: DepthMap?
    public var timestamp: Double
    public init(image: GrayImage, intrinsics: CameraIntrinsics, cameraToWorld: RigidTransform, poseValid: Bool,
                depth: DepthMap?, timestamp: Double) {
        self.image = image; self.intrinsics = intrinsics; self.cameraToWorld = cameraToWorld
        self.poseValid = poseValid; self.depth = depth; self.timestamp = timestamp
    }
}

// MARK: - Outputs

public enum EngineState: String {
    case idle          // no marker placed
    case calibrating   // gathering a full turn to find the axis
    case locked        // axis known, angle tracked
    case lost          // axis known but the angle cannot currently be measured
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
    public var processingMillis: Double = 0
    public var angleDispersionDegrees: Double = 0
    public var relocalizerAnalysed = false
    public var lastRelocalizationAge: Int = -1
    /// 1-sigma uncertainty of the absolute angle (degrees): small while everything agrees, large after a loss.
    public var angleUncertaintyDegrees: Double = 0
    public init() {}
}

// MARK: - Configuration

public struct EngineConfig {
    public var targetTrackCount = 160
    public var pyramidLevels = 4
    public var maxResidual: Float = 0.12
    public var minDepthConfidence: UInt8 = 1
    public var chordMinMeters = 0.02
    public var chordMaxFrames = 45
    public var chordStride = 3
    public var axisUpdateInterval = 10
    public var constraintWindowFrames = 900
    public var sampleHistory = 90
    public var lockedDriftFrames = 3      // consecutive inconsistent axis estimates before adopting the new axis
    public var axisConstraintCapacity = 24000
    /// A track whose depth jumps by more than this fraction between consecutive samples is on a depth edge
    /// (LiDAR bleeding from the background): the sample is skipped.
    public var maxDepthJumpFraction = 0.08
    /// Points below which the geometric angle is not considered a measurement of the object.
    public var minHealthyInliers = 20
    /// The library is rebuilt when a strong appearance match has contradicted a healthy geometry for this long.
    public var staleLibrarySeconds = 2.0
    public var relocRefreshAlpha: Float = 0.05
    public var descriptorSide = 32
    public var keyframeBins = 36
    public var lostAfterFrames = 45
    /// Radius of the region of interest around the marker, as a fraction of the image height.
    public var roiRadiusFraction = 0.35
    /// Place the marker on the moving textured region by itself (see `MotionLocator`), and look for it again
    /// after the angle has been lost for `autoMarkerLostSeconds`.
    public var autoMarker = true
    public var autoMarkerLostSeconds = 5.0
    public init() {}
}

// MARK: - Engine

/// Orchestrates tracking, axis estimation, angle integration and relocalisation. Not thread-safe: call `process`
/// from one queue.
public final class RotationEngine {
    public var config: EngineConfig

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

    public init(config: EngineConfig = EngineConfig()) {
        self.config = config
        self.reloc = Relocalizer(side: config.descriptorSide, binCount: config.keyframeBins)
        klt.halfWindow = 4
        corners.minDistance = 8
        estimator.capacity = config.axisConstraintCapacity
    }

    // MARK: Marker

    /// Place the marker (engine image pixels); the region of interest is `config.roiRadiusFraction` of the image height.
    public func setMarker(x: Float, y: Float) {
        marker = (x, y)
        resetAll()
        state = .calibrating
    }

    public func clearMarker() {
        marker = nil
        resetAll()
        state = .idle
    }

    private func resetAll() {
        tracks.removeAll()
        estimator.removeAll()
        angle.reset()
        reloc.reset()
        axis = nil
        lastEstimate = nil
        inconsistentAxisCount = 0
        calibrationStartFrame = frameIndex
        thetaMin = 0; thetaMax = 0
        lastAngleOkFrame = -1000
        lastRelocFrame = -1000
        fusion.reset()
        imageScale = nil
        disagreeStreak = 0
        rpmFiltered = 0
        lastInlierCount = 0
        lastAngleOk = false
        lostSince = nil
        locator.reset()
    }

    private func restartCalibration() {
        estimator.removeAll()
        angle.reset()
        reloc.reset()
        axis = nil
        lastEstimate = nil
        inconsistentAxisCount = 0
        calibrationStartFrame = frameIndex
        thetaMin = 0; thetaMax = 0
        fusion.reset()
        tracks.resetMotionHistory()
        state = .calibrating
    }

    // MARK: Main entry point

    public func process(_ input: FrameInput) -> EngineOutput {
        let t0 = Date()
        frameIndex += 1
        var out = EngineOutput()
        let pyramid = Pyramid(image: input.image, levelCount: config.pyramidLevels)
        if input.poseValid { lastPose = input.cameraToWorld }
        let pose = lastPose
        defer {
            prevPyramid = pyramid
            lastTimestamp = input.timestamp
        }
        let dt: Double = {
            guard let lt = lastTimestamp else { return 1.0 / 60.0 }
            let d = input.timestamp - lt
            return d > 0 && d < 0.5 ? d : 1.0 / 60.0
        }()

        let roiRadius = Float(config.roiRadiusFraction) * Float(input.image.height)
        if marker == nil, config.autoMarker,
           let hit = locator.add(image: input.image, prev: prevPyramid, cur: pyramid,
                                 pose: input.poseValid ? input.cameraToWorld : nil, dt: dt, time: input.timestamp,
                                 radius: roiRadius, klt: klt, corners: corners) {
            setMarker(x: hit.x, y: hit.y)
        }
        guard let marker = marker.map({ Marker(x: $0.x, y: $0.y, radius: roiRadius) }) else {
            out.state = .idle
            out.message = config.autoMarker ? "Looking for a moving object… \(locator.movingCount) moving points" : "Tap the object to track"
            out.processingMillis = Date().timeIntervalSince(t0) * 1000
            return out
        }

        // 1. Track existing points. The correspondences are kept: they also give the image-plane rotation,
        //    which cross-checks the 3D geometry later in this frame.
        let (before, after) = tracks.track(from: prevPyramid, to: pyramid, marker: marker,
                                           tracker: klt, maxResidual: config.maxResidual)

        // 2. Depth → 3D samples in world coordinates.
        tracks.sampleDepth(input, pose: pose, frame: frameIndex, minConfidence: config.minDepthConfidence,
                           maxJumpFraction: config.maxDepthJumpFraction, historyLength: config.sampleHistory)

        // 3. Chord constraints for the axis.
        let objectRadius = currentObjectRadius()
        let dmin = axis == nil ? config.chordMinMeters : min(0.05, max(0.01, 0.1 * objectRadius))
        for chord in tracks.chordConstraints(frame: frameIndex, minLength: dmin,
                                             maxFrames: config.chordMaxFrames, stride: config.chordStride) {
            estimator.add(chord)
        }
        estimator.prune(before: frameIndex - config.constraintWindowFrames)

        // 4. Angle.
        var angleOk = false
        if let ax = axis {
            let up = angle.update(tracks.project(around: ax))
            angleOk = up.ok
            lastInlierCount = up.inlierCount
            lastDispersion = up.dispersion
            if up.ok {
                lastAngleOkFrame = frameIndex
                thetaMin = min(thetaMin, up.theta); thetaMax = max(thetaMax, up.theta)
                let rpmInst = up.delta / dt * 60 / (2 * .pi)
                rpmFiltered += 0.15 * (rpmInst - rpmFiltered)
            } else {
                rpmFiltered *= 0.9
            }
            let removed = tracks.removeStatic(theta: up.ok ? up.theta : nil, frame: frameIndex, minLength: dmin)
            if !removed.isEmpty { angle.remove(ids: removed) }
        }
        lastAngleOk = angleOk

        // 4b. Cross-check: the image-plane rotation of the same correspondences uses neither depth nor the axis,
        //     so it fails in different situations than the 3D azimuth. When the two disagree, something else has
        //     taken over the tracked points (a hand, a reflection) and the geometric angle must not be trusted,
        //     however many "inliers" it reports.
        var agrees = true
        if let sim = similarityRotation(from: before, to: after), sim.inliers >= 8 {
            if angleOk, abs(angle.lastDelta) > Double.pi / 180 {
                let ratio = sim.angle / angle.lastDelta
                imageScale = imageScale.map { 0.9 * $0 + 0.1 * ratio } ?? ratio
            }
            if let scale = imageScale {
                let predicted = scale * angle.lastDelta
                agrees = abs(sim.angle - predicted) < 3 * Double.pi / 180 + 0.35 * abs(predicted)
            }
        }
        disagreeStreak = agrees ? 0 : disagreeStreak + 1
        let geometryHealthy = angleOk && lastInlierCount >= config.minHealthyInliers && disagreeStreak < 3

        // 5. Axis refinement — deliberately *after* the angle update. Re-anchoring the per-track offsets first
        //    would make every candidate agree with the current angle, silently throwing away one frame of motion
        //    every `axisUpdateInterval` frames (a systematic ~7 % under-estimation of the rotation).
        if frameIndex % config.axisUpdateInterval == 0, let est = estimator.estimate() {
            lastEstimate = est
            if est.isWellConditioned {
                updateAxis(with: est, objectRadius: objectRadius)
            }
        }

        // 6. State transitions and relocalisation.
        switch state {
        case .calibrating:
            if axis != nil, let est = lastEstimate, est.isWellConditioned, thetaMax - thetaMin >= 2 * .pi {
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
                if config.autoMarker, input.timestamp - lostSince! > config.autoMarkerLostSeconds {
                    // The object is gone: look for a moving one again.
                    clearMarker()
                    out.state = .idle
                    out.processingMillis = Date().timeIntervalSince(t0) * 1000
                    return out
                }
            } else {
                lostSince = nil
            }
            relocalise(image: input.image, marker: marker, angleOk: angleOk,
                       healthy: geometryHealthy, omega: angle.lastDelta / dt, dt: dt)
        case .idle:
            break
        }

        // 7. Replenish tracks.
        if tracks.count < config.targetTrackCount && (frameIndex % 5 == 0 || tracks.count < config.targetTrackCount / 3) {
            tracks.replenish(in: input.image, marker: marker, detector: corners, targetCount: config.targetTrackCount)
        }

        // 8. Output.
        out.state = state
        out.marker = marker
        out.axis = axis
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
            let q = min(1, est.inlierRatio / 0.7) * min(1, est.coverage / 0.6) * min(1, (0.3 - min(est.planarity, 0.3)) / 0.25)
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
        return out
    }

    // MARK: Helpers

    private func currentObjectRadius() -> Double {
        guard axis != nil else { return 0.1 }
        return tracks.objectRadius
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
            let tolDist = max(0.05, 0.5 * objectRadius)
            let consistent = angleBetween < 12 * .pi / 180 && lineDistance < tolDist
            if state == .calibrating {
                // Follow the estimate closely while calibrating.
                blend(into: old, target: newAxis, alpha: 0.5)
                inconsistentAxisCount = 0
            } else if consistent {
                blend(into: old, target: newAxis, alpha: 0.15)
                inconsistentAxisCount = 0
            } else {
                inconsistentAxisCount += 1
                if inconsistentAxisCount >= config.lockedDriftFrames {
                    inconsistentAxisCount = 0
                    let farOff = angleBetween > 25 * .pi / 180 || lineDistance > max(0.15, 3 * objectRadius)
                    if farOff {
                        // The object was replaced / moved a lot: start over.
                        restartCalibration()
                        axis = newAxis
                        rebaseAngle()
                    } else {
                        // Semi-static drift (chair rolled a little, axis re-estimated more precisely): adopt the new
                        // axis while keeping the angle continuous; the appearance library is rebuilt.
                        blend(into: old, target: newAxis, alpha: 0.7)
                        reloc.reset()
                        fusion.reset()
                    }
                }
            }
        } else {
            // First estimate: orient the axis so that it points roughly "up" in world space (y up in ARKit).
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
        angle.rebase(tracks.observations(around: ax))
    }

    /// Fuse the appearance measurement into the integrated angle. Returns nothing: everything it does is either
    /// a bounded correction of `angle`, or bookkeeping on the library.
    private func relocalise(image: GrayImage, marker: Marker,
                            angleOk: Bool, healthy: Bool, omega: Double, dt: Double) {
        let patch = image.patch(centerX: marker.x, centerY: marker.y, halfSize: marker.radius, side: config.descriptorSide)
        // Fill the library while the geometry is clean. A partially filled library is already useful.
        if angleOk && lastDispersion < 6 * Double.pi / 180 && lastInlierCount >= 8 && !reloc.isComplete {
            reloc.record(patch: patch, theta: angle.theta)
        }
        guard reloc.isUsable else { return }

        let candidates = reloc.candidates(patch: patch, nearTheta: angle.theta)
        let (correction, outcome) = fusion.update(theta: angle.theta, omega: omega, healthy: healthy,
                                                  candidates: candidates, dt: dt)
        if correction != 0 { angle.shift(by: correction) }
        if outcome == .updated { lastRelocFrame = frameIndex }
        // Keep the library in step with the current appearance, but only on a match we actually believe.
        if healthy && lastDispersion < 6 * Double.pi / 180 && fusion.lastMatchTrusted {
            reloc.refresh(patch: patch, theta: angle.theta, alpha: config.relocRefreshAlpha)
        }
        // A strong match that keeps contradicting a healthy geometry means the library, not the geometry, is
        // wrong: the object was displaced or the light changed. Rebuild it instead of fighting it.
        if fusion.staleSeconds > config.staleLibrarySeconds {
            reloc.reset()
            fusion.clearStale()
        }
    }

    private func statusMessage(_ out: EngineOutput) -> String {
        switch out.state {
        case .idle: return "Tap the object to track"
        case .calibrating:
            if out.trackCount < 10 { return "Not enough texture / depth around the marker" }
            if out.constraintCount < 150 { return "Rotate the object…" }
            if axis == nil { return "Finding the axis… (\(out.constraintCount) chords)" }
            return String(format: "Provisional axis — full turn: %.0f°/360°", min(out.turnCoverageDegrees, 360))
        case .locked:
            if !reloc.isComplete { return String(format: "Axis locked — learning appearance %.0f %%", reloc.fillRatio * 100) }
            if reloc.isRotationallySymmetric { return "Rotationally symmetric object: relative angle only" }
            if reloc.period < 2 * .pi - 1e-6 { return String(format: "Tracking — appearance period %.0f°", reloc.period * 180 / .pi) }
            return "Tracking"
        case .lost: return "Tracking lost — clear the view or rotate the object"
        }
    }
}
