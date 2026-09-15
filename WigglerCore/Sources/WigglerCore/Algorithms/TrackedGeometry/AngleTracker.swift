import Foundation

public struct AngleObservation: Codable {
    public var id: Int
    /// Azimuth of the tracked point around the axis (radians).
    public var phi: Double
    /// Distance of the point to the axis (meters) — sets the reliability of `phi`.
    public var radius: Double
    public init(id: Int, phi: Double, radius: Double) {
        self.id = id
        self.phi = phi
        self.radius = radius
    }
}

public struct AngleUpdate: Codable {
    public var theta: Double
    public var delta: Double
    public var inlierCount: Int
    public var candidateCount: Int
    /// Weighted RMS of the inlier deviations (radians).
    public var dispersion: Double
    public var ok: Bool
}

/// Estimates the global rotation angle θ(t) of a rigid object from the azimuths of tracked points.
/// Each track i has an unknown, constant offset o_i with φ_i(t) = θ(t) + o_i; offsets are learned when a track
/// first appears and θ is the robust weighted mean of φ_i − o_i. Because every live track "remembers" θ from its
/// birth, drift only accumulates through track turnover, not per frame.
public struct AngleTracker {
    private struct State {
        var offset: Double
        var badCount: Int
        var age: Int
    }
    private var states: [Int: State] = [:]

    /// Continuous (unwrapped) angle in radians.
    private(set) public var theta: Double = 0
    private(set) public var lastDelta: Double = 0

    public var sigma: Double = 5 * .pi / 180  // Cauchy scale for the robust mean
    public var gate: Double = 15 * .pi / 180  // beyond this a track is counted as inconsistent
    public var maxBadFrames: Int = 3
    public var minInliers: Int = 4

    public init() {}

    public mutating func reset(theta: Double = 0) {
        states.removeAll()
        self.theta = theta
        lastDelta = 0
    }

    public mutating func update(_ observations: [AngleObservation]) -> AngleUpdate {
        update(observations, diagnostics: nil)
    }

    mutating func update(_ observations: [AngleObservation], diagnostics: EngineDiagnosticsRecorder?) -> AngleUpdate {
        diagnostics?.value.geometry?.angleTracksBefore = diagnosticTracks()
        defer { diagnostics?.value.geometry?.angleTracksAfter = diagnosticTracks() }
        var cand: [(id: Int, value: Double, w: Double)] = []
        cand.reserveCapacity(observations.count)
        var radii: [Double] = []
        for o in observations where states[o.id] != nil {
            radii.append(o.radius)
        }
        let rcap = radii.isEmpty ? 1 : 1.5 * median(radii)
        diagnostics?.value.geometry?.radiusCap = rcap
        for o in observations {
            guard let s = states[o.id] else { continue }
            let r = min(o.radius, rcap)
            cand.append((o.id, wrapAngle(o.phi - s.offset), r * r))
        }
        var pred = theta + lastDelta
        diagnostics?.value.geometry?.predictedTheta = pred
        var inliers = 0
        var dispersion = 0.0
        var ok = false
        if !cand.isEmpty {
            for _ in 0..<4 {
                var num = 0.0, den = 0.0
                for c in cand {
                    let dev = wrapAngle(c.value - pred)
                    let w = c.w / (1 + (dev / sigma) * (dev / sigma))
                    num += w * dev
                    den += w
                }
                if den > 0 { pred += num / den }
            }
            diagnostics?.value.geometry?.thetaBeforeHardGate = pred
            // Final hard-gated pass.
            var num = 0.0, den = 0.0, sq = 0.0
            for c in cand {
                let dev = wrapAngle(c.value - pred)
                diagnostics?.value.geometry?.candidates.append(
                    .init(
                        id: c.id, value: c.value, weight: c.w, residual: dev, inlier: abs(dev) < gate))
                if abs(dev) < gate {
                    num += c.w * dev
                    den += c.w
                    sq += c.w * dev * dev
                    inliers += 1
                }
            }
            if den > 0 && inliers >= minInliers {
                pred += num / den
                dispersion = (sq / den).squareRoot()
                ok = true
            }
        }
        if ok {
            let delta = wrapAngle(pred - theta)
            theta += delta
            lastDelta = delta
            // Bookkeeping: consistency of every track with the new estimate.
            for c in cand {
                let dev = wrapAngle(c.value - theta)
                if abs(dev) < gate {
                    states[c.id]?.badCount = 0
                    states[c.id]?.age += 1
                } else {
                    states[c.id]?.badCount += 1
                    if let s = states[c.id], s.badCount >= maxBadFrames {
                        states[c.id] = nil  // will be re-anchored below with the current θ
                    }
                }
            }
        } else {
            lastDelta *= 0.5
        }
        // Anchor new (or re-anchored) tracks.
        for o in observations where states[o.id] == nil {
            states[o.id] = State(offset: wrapAngle(o.phi - theta), badCount: 0, age: 0)
        }
        return AngleUpdate(
            theta: theta, delta: lastDelta, inlierCount: inliers, candidateCount: cand.count,
            dispersion: dispersion, ok: ok)
    }

    /// The axis moved: re-anchor every track so θ stays continuous.
    public mutating func rebase(_ observations: [AngleObservation]) {
        for o in observations {
            let age = states[o.id]?.age ?? 0
            states[o.id] = State(offset: wrapAngle(o.phi - theta), badCount: 0, age: age)
        }
    }

    /// Absolute correction (e.g. from appearance relocalisation): θ ← θ + delta, tracks follow.
    public mutating func shift(by delta: Double) {
        theta += delta
        for (id, s) in states {
            states[id] = State(offset: wrapAngle(s.offset - delta), badCount: s.badCount, age: s.age)
        }
    }

    public mutating func remove(ids: [Int]) { for id in ids { states[id] = nil } }
    public mutating func removeAllExcept(ids: Set<Int>) { states = states.filter { ids.contains($0.key) } }

    public func isConsistent(id: Int) -> Bool { (states[id]?.badCount ?? 1) == 0 }
    public var trackedCount: Int { states.count }
}

extension AngleTracker {
    func diagnosticTracks() -> [EngineDiagnostics.AngleTrack] {
        states.keys.sorted().map { id in
            let state = states[id]!
            return .init(id: id, offset: state.offset, badCount: state.badCount, age: state.age)
        }
    }
}
