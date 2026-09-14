import Foundation

/// Appearance-based absolute angle: a library of small normalised patches of the region of interest,
/// one per angular bin, recorded during one full turn once the axis is locked.
/// Matching a live patch against the library gives an absolute angle (modulo the object's appearance period),
/// used to cancel the slow drift of the geometric integration and to recover after occlusions.
public struct Relocalizer {
    public let side: Int
    public let binCount: Int
    /// Raw (area-averaged luma) patches per angular bin.
    private var raw: [[Float]?]
    /// Mean of the raw patches — the static background/illumination that every bin shares.
    private var meanPatch: [Float] = []
    /// Centred + normalised descriptors used for matching.
    private var library: [[Float]] = []
    private var filled = 0

    /// Smallest appearance period found in the library (radians); 2π for an asymmetric object.
    private(set) public var period: Double = 2 * .pi
    /// Self-similarity score of the library at its period (1 = perfectly periodic). Low values ⇒ unreliable period.
    private(set) public var periodScore: Double = 0
    /// True when the object looks the same from (almost) every angle ⇒ absolute angle unobservable.
    private(set) public var isRotationallySymmetric = false
    private(set) public var analysed = false

    public var binWidth: Double { 2 * .pi / Double(binCount) }
    public var isComplete: Bool { filled >= binCount }
    public var fillRatio: Double { Double(filled) / Double(binCount) }

    public init(side: Int = 32, binCount: Int = 36) {
        self.side = side
        self.binCount = binCount
        self.raw = [[Float]?](repeating: nil, count: binCount)
    }

    public mutating func reset() {
        raw = [[Float]?](repeating: nil, count: binCount)
        library = []
        meanPatch = []
        filled = 0
        period = 2 * .pi
        periodScore = 0
        isRotationallySymmetric = false
        analysed = false
    }

    /// Zero-mean, unit-norm descriptor.
    public static func normalise(_ p: [Float]) -> [Float] {
        let n = Float(p.count)
        var mean: Float = 0
        for v in p { mean += v }
        mean /= n
        var out = p.map { $0 - mean }
        var ss: Float = 0
        for v in out { ss += v * v }
        let inv = ss > 1e-12 ? 1 / ss.squareRoot() : 0
        for i in 0..<out.count { out[i] *= inv }
        return out
    }

    @inline(__always) static func ncc(_ a: [Float], _ b: [Float]) -> Double {
        var s: Float = 0
        for i in 0..<a.count { s += a[i] * b[i] }
        return Double(s)
    }

    /// Record a live raw patch at angle θ (radians, any range).
    public mutating func record(patch: [Float], theta: Double) {
        let bin = Int((positiveAngle(theta) / binWidth).rounded()) % binCount
        if raw[bin] == nil { filled += 1 }
        raw[bin] = patch
    }

    private func centred(_ patch: [Float]) -> [Float] {
        var out = patch
        for i in 0..<out.count { out[i] -= meanPatch[i] }
        return Relocalizer.normalise(out)
    }

    /// Determine the appearance period once the library is complete.
    public mutating func analyse() {
        guard isComplete, let first = raw[0] else { return }
        analysed = true
        meanPatch = [Float](repeating: 0, count: first.count)
        for p in raw { for (i, v) in p!.enumerated() { meanPatch[i] += v } }
        for i in 0..<meanPatch.count { meanPatch[i] /= Float(binCount) }
        library = raw.map { centred($0!) }
        // Autocorrelation over bin shifts.
        var scores = [Double](repeating: 0, count: binCount)
        for shift in 1..<binCount {
            var s = 0.0
            for k in 0..<binCount {
                s += Relocalizer.ncc(library[k], library[(k + shift) % binCount])
            }
            scores[shift] = s / Double(binCount)
        }
        // Similarity between neighbouring bins tells us whether the appearance carries an angular signal at all
        // (the shared static background was removed by `meanPatch`, so a round pot gives only noise here).
        let neighbour = scores[1]
        if neighbour < 0.35 {
            isRotationallySymmetric = true
            period = 0
            periodScore = neighbour
            return
        }
        isRotationallySymmetric = false
        // The smallest shift (a divisor of the turn) whose similarity rivals the neighbour similarity is the period.
        period = 2 * .pi
        periodScore = 1
        for shift in 2...(binCount / 2) where binCount % shift == 0 {
            let s = scores[shift]
            if s > 0.5 && s >= neighbour - 0.1 {
                period = Double(shift) * binWidth
                periodScore = s
                break
            }
        }
    }

    public struct Match {
        /// Absolute angle candidate closest to `nearTheta` (radians, unwrapped relative to nearTheta).
        public var theta: Double
        public var score: Double
        /// Score of the best bin that is not within ±2 bins of any accepted candidate.
        public var distinctiveness: Double
        public var confident: Bool
    }

    /// Match a live descriptor. `nearTheta` disambiguates periodic objects (choose the candidate closest to it).
    public func match(patch: [Float], nearTheta: Double) -> Match? {
        guard analysed, library.count == binCount else { return nil }
        let descriptor = centred(patch)
        var scores = [Double](repeating: -2, count: binCount)
        var best = -2.0
        for k in 0..<binCount {
            let s = Relocalizer.ncc(descriptor, library[k])
            scores[k] = s
            if s > best { best = s }
        }
        if best < 0.55 { return nil }
        // Candidate bins: local maxima with score close to the best.
        var candidates: [Int] = []
        for k in 0..<binCount {
            let s = scores[k]
            if s >= best - 0.06 && s >= scores[(k + 1) % binCount] && s >= scores[(k + binCount - 1) % binCount] {
                candidates.append(k)
            }
        }
        if candidates.isEmpty { return nil }
        // Choose the candidate closest to the current estimate.
        var chosen = candidates[0]
        var bestDist = Double.infinity
        for k in candidates {
            let d = abs(wrapAngle(Double(k) * binWidth - nearTheta))
            if d < bestDist { bestDist = d; chosen = k }
        }
        // Sub-bin refinement by parabolic interpolation on the three neighbouring scores.
        let sm = scores[(chosen + binCount - 1) % binCount], s0 = scores[chosen], sp = scores[(chosen + 1) % binCount]
        let denom = sm - 2 * s0 + sp
        var frac = 0.0
        if denom < -1e-9 { frac = max(-0.5, min(0.5, 0.5 * (sm - sp) / denom)) }
        let absolute = (Double(chosen) + frac) * binWidth
        // Distinctiveness: best score outside ±2 bins of all candidates (period-aware ambiguity is expected and fine).
        var runnerUp = -2.0
        for k in 0..<binCount {
            var near = false
            for c in candidates {
                let d = min((k - c + binCount) % binCount, (c - k + binCount) % binCount)
                if d <= 2 { near = true; break }
            }
            if !near && scores[k] > runnerUp { runnerUp = scores[k] }
        }
        let distinct = best - runnerUp
        let thetaUnwrapped = nearTheta + wrapAngle(absolute - nearTheta)
        let confident = best >= 0.6 && distinct >= 0.1 && !isRotationallySymmetric
        return Match(theta: thetaUnwrapped, score: best, distinctiveness: distinct, confident: confident)
    }
}
