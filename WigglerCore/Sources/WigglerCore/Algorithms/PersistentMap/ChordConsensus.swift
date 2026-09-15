import Foundation

/// The largest set of chords one axis explains. Chords of a body turning about an axis agree on it; chords of
/// anything else that moves do not, and a reweighted fit alone cannot separate them once the strangers are many,
/// since it starts from the mean of everything. Trying small subsets finds the agreement first and lets the fit
/// refine it afterwards.
struct ChordConsensus {
    /// How far a chord may sit from the axis it is said to belong to, metres: two depth samples of a real sensor
    /// already differ by about this much.
    var inlierMeters = 0.015
    var trials = 512
    /// Chords scored per trial. Scoring every chord in every trial buys precision that the final fit provides
    /// anyway; a sample of this size ranks the candidates just as well.
    var sampleLimit = 400
    var subsetSize = 6
    /// Below this many chords a subset search has nothing to add over the fit itself.
    var minimumChords = 30

    /// The chords that the best-supported candidate axis explains, in their original order.
    func select(_ chords: [ChordConstraint], seed: UInt64 = 9) -> [ChordConstraint] {
        guard chords.count >= minimumChords else { return chords }
        var units: [V3] = [], midpoints: [V3] = [], lengths: [Double] = []
        units.reserveCapacity(chords.count)
        for chord in chords {
            let length = chord.chord.length
            guard length > 1e-6 else { continue }
            units.append(chord.chord / length)
            midpoints.append(chord.midpoint)
            lengths.append(length)
        }
        guard units.count >= minimumChords else { return chords }
        var state = seed &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
        func nextIndex(_ bound: Int) -> Int {
            state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
            return Int((state >> 33) % UInt64(bound))
        }
        let sample =
            units.count <= sampleLimit
            ? Array(units.indices) : (0..<sampleLimit).map { _ in nextIndex(units.count) }
        var best: (direction: V3, center: V3, support: Int)?
        for _ in 0..<trials {
            let picks = (0..<subsetSize).map { _ in nextIndex(units.count) }
            guard let candidate = fit(picks, units: units, midpoints: midpoints) else { continue }
            var support = 0
            for index in sample
            where residual(
                index, candidate, units: units, midpoints: midpoints,
                lengths: lengths) < inlierMeters
            {
                support += 1
            }
            if support > (best?.support ?? 0) { best = (candidate.direction, candidate.center, support) }
        }
        guard let best else { return chords }
        let candidate = (direction: best.direction, center: best.center)
        var selected: [ChordConstraint] = []
        selected.reserveCapacity(units.count)
        for index in units.indices
        where residual(index, candidate, units: units, midpoints: midpoints, lengths: lengths) < inlierMeters {
            selected.append(chords[index])
        }
        return selected.count >= minimumChords ? selected : chords
    }

    /// Axis of one subset: the direction least aligned with its chords, and the point their perpendicular bisectors
    /// meet. Nil when the subset determines neither.
    private func fit(_ picks: [Int], units: [V3], midpoints: [V3]) -> (direction: V3, center: V3)? {
        var scatter = M3.zero
        var mean = V3.zero
        for index in picks {
            scatter = scatter + M3.outer(units[index], units[index])
            mean += midpoints[index]
        }
        mean = mean / Double(picks.count)
        let direction = scatter.symmetricEigen().vectors[0].normalized
        var rhs = direction * direction.dot(mean)
        for index in picks { rhs += units[index] * units[index].dot(midpoints[index]) }
        guard let center = (scatter + M3.outer(direction, direction)).solve(rhs), center.isFinite else {
            return nil
        }
        return (direction, center)
    }

    /// How far one chord is from being a chord of that axis: its component along the axis, and how far its midpoint
    /// misses the plane through the axis perpendicular to it.
    private func residual(
        _ index: Int, _ candidate: (direction: V3, center: V3), units: [V3], midpoints: [V3], lengths: [Double]
    ) -> Double {
        let along = units[index].dot(candidate.direction) * lengths[index]
        let across = (midpoints[index] - candidate.center).dot(units[index])
        return (along * along + across * across).squareRoot()
    }
}
