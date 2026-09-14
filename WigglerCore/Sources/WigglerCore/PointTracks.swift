import Foundation

/// Owns the lifetime of tracked image points and their depth, chord and motion histories.
final class PointTracks {
    private final class Track {
        let id: Int
        var x: Float
        var y: Float
        var samples: [(frame: Int, p: V3)] = []
        var age = 0
        var hasDepthThisFrame = false
        var lastChordFrame = -1000
        var radius = 0.0
        var height = 0.0
        var lastDepth = 0.0
        var depthRejects = 0
        init(id: Int, x: Float, y: Float) { self.id = id; self.x = x; self.y = y }
    }

    private var tracks: [Track] = []
    private var nextId = 1
    private var thetaHistory: [Double?] = []

    var count: Int { tracks.count }

    func removeAll() {
        tracks.removeAll()
        thetaHistory.removeAll()
    }

    func resetMotionHistory() {
        thetaHistory.removeAll()
        for t in tracks { t.lastChordFrame = -1000 }
    }

    func track(from prev: Pyramid?, to pyramid: Pyramid, marker: Marker,
                        tracker: KLTTracker, maxResidual: Float) -> (before: [(Float, Float)], after: [(Float, Float)]) {
        var before: [(Float, Float)] = []
        var after: [(Float, Float)] = []
        if let prev = prev, prev.levels[0].width == pyramid.levels[0].width {
            var survivors: [Track] = []
            survivors.reserveCapacity(tracks.count)
            before.reserveCapacity(tracks.count)
            after.reserveCapacity(tracks.count)
            let roi2 = marker.radius * marker.radius * 1.3 * 1.3
            for t in tracks {
                let r = tracker.track(prev: prev, cur: pyramid, x: t.x, y: t.y)
                if !r.ok || r.residual > maxResidual { continue }
                let dx = r.x - marker.x, dy = r.y - marker.y
                if dx * dx + dy * dy > roi2 { continue }
                before.append((t.x, t.y))
                after.append((r.x, r.y))
                t.x = r.x; t.y = r.y; t.age += 1
                survivors.append(t)
            }
            tracks = survivors
        } else {
            tracks.removeAll()
        }
        return (before, after)
    }

    func sampleDepth(_ input: FrameInput, pose: RigidTransform, frame: Int,
                     minConfidence: UInt8, maxJumpFraction: Double, historyLength: Int) {
        let w = Double(input.image.width), h = Double(input.image.height)
        let K = input.intrinsics
        for t in tracks {
            t.hasDepthThisFrame = false
            guard let depth = input.depth,
                  let z = depth.sample(u: Double(t.x) / w, v: Double(t.y) / h, minConfidence: minConfidence)
            else { continue }
            if t.lastDepth > 0 && abs(z - t.lastDepth) > maxJumpFraction * t.lastDepth {
                t.depthRejects += 1
                if t.depthRejects < 4 { continue }
            }
            t.depthRejects = 0
            t.lastDepth = z
            let xc = (Double(t.x) - K.cx) / K.fx * z
            let yc = (Double(t.y) - K.cy) / K.fy * z
            let cam = V3(xc, -yc, -z)
            let world = pose.apply(cam)
            if !world.isFinite { continue }
            t.samples.append((frame, world))
            if t.samples.count > historyLength { t.samples.removeFirst(t.samples.count - historyLength) }
            t.hasDepthThisFrame = true
        }
    }

    func chordConstraints(frame: Int, minLength: Double, maxFrames: Int, stride: Int) -> [ChordConstraint] {
        var constraints: [ChordConstraint] = []
        for t in tracks where t.hasDepthThisFrame && frame - t.lastChordFrame >= stride {
            guard let last = t.samples.last else { continue }
            for s in t.samples {
                if frame - s.frame > maxFrames { continue }
                if s.frame >= last.frame { break }
                let d = last.p - s.p
                if d.length >= minLength {
                    constraints.append(ChordConstraint(midpoint: (last.p + s.p) * 0.5, chord: d, frame: frame))
                    t.lastChordFrame = frame
                    break
                }
            }
        }
        return constraints
    }

    /// Measure the angle observations and retain the object's extent in this frame's axis coordinates.
    func project(around axis: Axis) -> [AngleObservation] {
        var observations: [AngleObservation] = []
        observations.reserveCapacity(tracks.count)
        for t in tracks where t.hasDepthThisFrame {
            let (phi, r, height) = axis.cylindrical(t.samples[t.samples.count - 1].p)
            t.radius = r; t.height = height
            if r > 0.005 { observations.append(AngleObservation(id: t.id, phi: phi, radius: r)) }
        }
        return observations
    }

    /// Observe a revised axis without replacing the extent measured before that revision.
    func observations(around axis: Axis) -> [AngleObservation] {
        var observations: [AngleObservation] = []
        for t in tracks where t.hasDepthThisFrame {
            let (phi, r, _) = axis.cylindrical(t.samples[t.samples.count - 1].p)
            if r > 0.005 { observations.append(AngleObservation(id: t.id, phi: phi, radius: r)) }
        }
        return observations
    }

    func removeStatic(theta: Double?, frame: Int, minLength: Double) -> [Int] {
        thetaHistory.append(theta)
        if thetaHistory.count > 120 { thetaHistory.removeFirst(thetaHistory.count - 120) }
        let span = 60
        guard thetaHistory.count > span, let now = thetaHistory[thetaHistory.count - 1],
              let before = thetaHistory[thetaHistory.count - 1 - span] else { return [] }
        if abs(now - before) < 20 * .pi / 180 { return [] }
        var removed: [Int] = []
        tracks.removeAll { t in
            guard let last = t.samples.last, let first = t.samples.first(where: { frame - $0.frame <= span }),
                  last.frame - first.frame >= span - 5 else { return false }
            var maxD = 0.0
            for s in t.samples where s.frame >= first.frame {
                maxD = max(maxD, (s.p - last.p).length)
            }
            if maxD < minLength { removed.append(t.id); return true }
            return false
        }
        return removed
    }

    func replenish(in image: GrayImage, marker: Marker, detector: CornerDetector, targetCount: Int) {
        guard tracks.count < targetCount else { return }
        let existing = tracks.map { ($0.x, $0.y) }
        let fresh = detector.detect(in: image, centerX: marker.x, centerY: marker.y, radius: marker.radius,
                                    exclude: existing, maxCount: targetCount - tracks.count)
        for (x, y) in fresh {
            tracks.append(Track(id: nextId, x: x, y: y))
            nextId += 1
        }
    }

    var objectRadius: Double {
        let radii = tracks.filter { $0.radius > 0 }.map { $0.radius }
        return radii.isEmpty ? 0.1 : percentile(radii, 0.8)
    }

    func extent(isConsistent: (Int) -> Bool) -> (radii: [Double], heights: [Double]) {
        let visible = tracks.filter { $0.hasDepthThisFrame && isConsistent($0.id) }
        return (visible.map { $0.radius }, visible.map { $0.height })
    }

    func debug(hasAxis: Bool, isConsistent: (Int) -> Bool) -> [TrackDebug] {
        tracks.map { t in
            let status: TrackStatus
            if t.age < 3 { status = .young }
            else if !t.hasDepthThisFrame { status = .noDepth }
            else if hasAxis && !isConsistent(t.id) { status = .inconsistent }
            else { status = .good }
            return TrackDebug(x: t.x, y: t.y, status: status)
        }
    }
}
