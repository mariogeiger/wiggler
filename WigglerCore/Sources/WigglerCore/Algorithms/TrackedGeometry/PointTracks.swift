import Foundation

/// Retains depth, chord, and extent histories for the selected image-point identities.
final class PointTracks {
    private final class Track {
        let id: Int
        var x: Float
        var y: Float
        var samples: [(frame: Int, p: V3)] = []
        var previousSample: (frame: Int, p: V3)?
        var age = 0
        var hasDepthThisFrame = false
        var lastChordFrame = -1000
        var radius = 0.0
        var height = 0.0
        var lastDepth = 0.0
        var depthRejects = 0
        init(id: Int, x: Float, y: Float) {
            self.id = id
            self.x = x
            self.y = y
        }
    }

    private var tracks: [Track] = []

    var count: Int { tracks.count }
    var ids: Set<Int> { Set(tracks.map { $0.id }) }

    func removeAll(diagnostics: EngineDiagnosticsRecorder? = nil) {
        diagnostics?.value.trackUpdates.append(
            .init(
                reason: "measurementReset", selectedIDs: [], addedIDs: [], removedIDs: tracks.map { $0.id }))
        tracks.removeAll()
    }

    func resetChordCadence() {
        for t in tracks { t.lastChordFrame = -1000 }
    }

    /// Adopt selected image correspondences while preserving the depth history of surviving identities.
    @discardableResult
    func update(_ selected: [MotionLocator.Point], diagnostics: EngineDiagnosticsRecorder? = nil)
        -> (before: [(Float, Float)], after: [(Float, Float)])
    {
        if let diagnostics {
            let selectedIDs = Set(selected.map { $0.id })
            diagnostics.value.trackUpdates.append(
                .init(
                    selectedIDs: selected.map { $0.id }, addedIDs: selectedIDs.subtracting(ids).sorted(),
                    removedIDs: ids.subtracting(selectedIDs).sorted()))
        }
        let existing = Dictionary(uniqueKeysWithValues: tracks.map { ($0.id, $0) })
        var before: [(Float, Float)] = []
        var after: [(Float, Float)] = []
        tracks = selected.map { p in
            let t: Track
            if let old = existing[p.id] {
                t = old
                before.append((p.previousX, p.previousY))
                after.append((p.x, p.y))
                t.age += 1
            } else {
                t = Track(id: p.id, x: p.x, y: p.y)
            }
            t.x = p.x
            t.y = p.y
            return t
        }
        return (before, after)
    }

    func sampleDepth(
        _ input: FrameInput, pose: RigidTransform, frame: Int,
        minConfidence: UInt8, maxJumpFraction: Double, historyLength: Int,
        diagnostics: EngineDiagnosticsRecorder? = nil
    ) {
        let w = Double(input.image.width), h = Double(input.image.height)
        let K = input.intrinsics
        for t in tracks {
            let evidence = diagnostics.map { _ in
                DepthObservationRecorder(
                    .init(
                        id: t.id, x: t.x, y: t.y, previousDepth: t.lastDepth, rejectsBefore: t.depthRejects,
                        minConfidence: minConfidence, maxJumpFraction: maxJumpFraction, rejectsAfter: t.depthRejects))
            }
            defer {
                if var observed = evidence?.value {
                    observed.rejectsAfter = t.depthRejects
                    diagnostics?.value.depth.append(observed)
                }
            }
            t.previousSample = t.samples.last
            t.hasDepthThisFrame = false
            guard let depth = input.depth else { continue }
            let observe: ((EngineDiagnostics.DepthNeighborhood) -> Void)? =
                diagnostics == nil
                ? nil
                : {
                    evidence?.value.neighborhood = $0
                    evidence?.value.outcome = $0.outcome
                }
            guard
                let z = depth.sample(
                    u: Double(t.x) / w, v: Double(t.y) / h, minConfidence: minConfidence, observe: observe)
            else { continue }
            evidence?.value.sampledDepth = z
            evidence?.value.jumpLimit = maxJumpFraction * t.lastDepth
            if t.lastDepth > 0 && abs(z - t.lastDepth) > maxJumpFraction * t.lastDepth {
                t.depthRejects += 1
                if t.depthRejects < 4 {
                    evidence?.value.outcome = "depthJumpRejected"
                    continue
                }
            }
            t.depthRejects = 0
            t.lastDepth = z
            let xc = (Double(t.x) - K.cx) / K.fx * z
            let yc = (Double(t.y) - K.cy) / K.fy * z
            let cam = V3(xc, -yc, -z)
            let world = pose.apply(cam)
            evidence?.value.worldPoint = world
            if !world.isFinite {
                evidence?.value.outcome = "nonFiniteWorldPoint"
                continue
            }
            evidence?.value.outcome = "accepted"
            t.samples.append((frame, world))
            if t.samples.count > historyLength { t.samples.removeFirst(t.samples.count - historyLength) }
            t.hasDepthThisFrame = true
        }
    }

    func chordConstraints(
        frame: Int, minLength: Double, maxFrames: Int, stride: Int, diagnostics: EngineDiagnosticsRecorder? = nil
    ) -> [ChordConstraint] {
        var constraints: [ChordConstraint] = []
        for t in tracks where t.hasDepthThisFrame && frame - t.lastChordFrame >= stride {
            guard let last = t.samples.last else { continue }
            for s in t.samples {
                if frame - s.frame > maxFrames { continue }
                if s.frame >= last.frame { break }
                let d = last.p - s.p
                if d.length >= minLength {
                    constraints.append(ChordConstraint(midpoint: (last.p + s.p) * 0.5, chord: d, frame: frame))
                    diagnostics?.value.freshChords.append(
                        .init(
                            trackID: t.id, start: .init(frame: s.frame, point: s.p),
                            end: .init(frame: last.frame, point: last.p), constraint: constraints[constraints.count - 1]
                        ))
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
            t.radius = r
            t.height = height
            if r > 0.005 { observations.append(AngleObservation(id: t.id, phi: phi, radius: r)) }
        }
        return observations
    }

    /// Compare consecutive image positions using only the previous frame's depth.
    func rotationMatches(frame: Int) -> [RotationImageMatch] {
        tracks.compactMap { track in
            guard let previous = track.previousSample, previous.frame == frame - 1 else {
                return nil
            }
            return RotationImageMatch(id: track.id, point: previous.p, x: Double(track.x), y: Double(track.y))
        }
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

    /// Image extent of the tracked body around a point: the 90th percentile distance of its tracks (pixels).
    func imageRadius(aroundX cx: Float, y cy: Float, including include: (Int) -> Bool) -> Float {
        let d = tracks.filter { include($0.id) }.map { Double(hypot($0.x - cx, $0.y - cy)) }
        return d.isEmpty ? 0 : Float(percentile(d, 0.9))
    }

    func objectRadius(including include: (Int) -> Bool) -> Double {
        let radii = tracks.filter { $0.radius > 0 && include($0.id) }.map { $0.radius }
        return radii.isEmpty ? 0.1 : percentile(radii, 0.8)
    }

    func extent(isConsistent: (Int) -> Bool) -> (radii: [Double], heights: [Double]) {
        let visible = tracks.filter { $0.hasDepthThisFrame && isConsistent($0.id) }
        return (visible.map { $0.radius }, visible.map { $0.height })
    }

    func debug(hasAxis: Bool, isConsistent: (Int) -> Bool) -> [TrackDebug] {
        tracks.map { t in
            let status: TrackStatus
            if t.age < 3 {
                status = .young
            } else if !t.hasDepthThisFrame {
                status = .noDepth
            } else if hasAxis && !isConsistent(t.id) {
                status = .inconsistent
            } else {
                status = .good
            }
            return TrackDebug(x: t.x, y: t.y, status: status)
        }
    }
}

extension PointTracks {
    func diagnosticState() -> [EngineDiagnostics.Track] {
        tracks.map {
            .init(
                id: $0.id, x: $0.x, y: $0.y, age: $0.age, hasDepthThisFrame: $0.hasDepthThisFrame,
                lastDepth: $0.lastDepth, depthRejects: $0.depthRejects, lastChordFrame: $0.lastChordFrame,
                sampleCount: $0.samples.count, oldestSampleFrame: $0.samples.first?.frame,
                newestSampleFrame: $0.samples.last?.frame, radius: $0.radius, height: $0.height)
        }
    }

    func diagnosticHistories() -> [EngineDiagnostics.TrackHistory] {
        tracks.map { .init(id: $0.id, samples: $0.samples.map { .init(frame: $0.frame, point: $0.p) }) }
    }
}
