import Foundation

/// Appearance-based absolute angle: a library of small normalised patches of the region of interest, one per
/// angular bin, filled as the object turns.
///
/// The library is usable as soon as a handful of bins exist — waiting for a full turn would leave the absolute
/// reference unavailable most of the time. Matching only ever considers the bins that have been filled.
public struct Relocalizer {
    public let side: Int
    public let binCount: Int
    /// Raw (area-averaged luma) patches per angular bin; nil where nothing has been recorded yet.
    private var raw: [[Float]?]
    /// Mean of the filled patches — the static background/illumination every bin shares.
    private var meanPatch: [Float] = []
    /// Centred + normalised descriptors of the filled bins, and the bin index of each.
    private var library: [[Float]] = []
    private var index: [Int] = []
    private(set) public var filled = 0

    /// Smallest appearance period found once the library is complete (radians); 2π for an asymmetric object,
    /// 0 when the object looks the same from every angle.
    private(set) public var period: Double = 2 * .pi
    private(set) public var periodScore: Double = 0
    private(set) public var isRotationallySymmetric = false
    private(set) public var analysed = false

    public var binWidth: Double { 2 * .pi / Double(binCount) }
    /// Enough bins to be worth matching against.
    public var isUsable: Bool { filled >= 6 }
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
        index = []
        meanPatch = []
        filled = 0
        period = 2 * .pi
        periodScore = 0
        isRotationallySymmetric = false
        analysed = false
    }

    /// Zero-mean, unit-norm descriptor.
    public static func normalise(_ p: [Float]) -> [Float] {
        var mean: Float = 0
        for v in p { mean += v }
        mean /= Float(p.count)
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

    @inline(__always) private func bin(for theta: Double) -> Int {
        Int((positiveAngle(theta) / binWidth).rounded()) % binCount
    }

    /// Record a live patch at angle θ.
    public mutating func record(patch: [Float], theta: Double) {
        let b = bin(for: theta)
        if raw[b] == nil { filled += 1 }
        raw[b] = patch
        rebuild()
    }

    /// Blend a live patch into its bin: the library follows slow changes (light, small displacements) while the
    /// older content still anchors it against drift.
    public mutating func refresh(patch: [Float], theta: Double, alpha: Float) {
        let b = bin(for: theta)
        if var old = raw[b], old.count == patch.count {
            for i in 0..<old.count { old[i] += alpha * (patch[i] - old[i]) }
            raw[b] = old
        } else {
            raw[b] = patch
            filled += 1
        }
        rebuild()
    }

    private mutating func rebuild() {
        index = (0..<binCount).filter { raw[$0] != nil }
        guard index.count >= 6, let first = raw[index[0]] else {
            library = []
            analysed = false
            return
        }
        var mean = [Float](repeating: 0, count: first.count)
        for k in index {
            let p = raw[k]!
            for i in 0..<mean.count { mean[i] += p[i] }
        }
        for i in 0..<mean.count { mean[i] /= Float(index.count) }
        meanPatch = mean
        library = index.map { centred(raw[$0]!) }
        analysed = true
        if isComplete { analysePeriod() }
    }

    private func centred(_ patch: [Float]) -> [Float] {
        var out = patch
        for i in 0..<out.count { out[i] -= meanPatch[i] }
        return Relocalizer.normalise(out)
    }

    /// Appearance period, once every bin exists.
    private mutating func analysePeriod() {
        // Rotational symmetry = the appearance does not depend on θ: the variance of the patches *across bins*
        // (around the mean patch) is a negligible fraction of their total variance (around a flat grey).
        // Adjacent-bin correlation cannot tell this apart from fine texture, which also decorrelates within a bin.
        var between: Float = 0, total: Float = 0, grey: Float = 0
        for v in meanPatch { grey += v }
        grey /= Float(max(meanPatch.count, 1))
        for k in 0..<binCount {
            let p = raw[k]!
            for i in 0..<p.count {
                between += (p[i] - meanPatch[i]) * (p[i] - meanPatch[i])
                total += (p[i] - grey) * (p[i] - grey)
            }
        }
        let thetaDependence = Double(between / max(total, 1e-12))
        if thetaDependence < 0.1 {
            isRotationallySymmetric = true
            period = 0
            periodScore = thetaDependence
            return
        }
        var scores = [Double](repeating: 0, count: binCount)
        for shift in 1..<binCount {
            var s = 0.0
            for k in 0..<binCount { s += Relocalizer.ncc(library[k], library[(k + shift) % binCount]) }
            scores[shift] = s / Double(binCount)
        }
        let neighbour = scores[1]
        isRotationallySymmetric = false
        period = 2 * .pi
        periodScore = 1
        for shift in 2...(binCount / 2) where binCount % shift == 0 {
            if scores[shift] > 0.5 && scores[shift] >= neighbour - 0.1 {
                period = Double(shift) * binWidth
                periodScore = scores[shift]
                break
            }
        }
    }

    /// All plausible absolute angles for a live patch, unwrapped around `nearTheta`.
    /// The fusion decides which one (if any) to believe — that is not this type's job.
    public func candidates(patch: [Float], nearTheta: Double) -> [AngleFusion.Candidate] {
        guard analysed, !library.isEmpty else { return [] }
        let d = centred(patch)
        var score = [Double](repeating: -2, count: binCount)
        for (i, k) in index.enumerated() { score[k] = Relocalizer.ncc(d, library[i]) }
        var out: [AngleFusion.Candidate] = []
        for k in index {
            let prev = score[(k + binCount - 1) % binCount]
            let next = score[(k + 1) % binCount]
            guard score[k] > 0.5, score[k] >= prev, score[k] >= next else { continue }
            var frac = 0.0
            if prev > -1, next > -1 {
                let den = prev - 2 * score[k] + next
                if den < -1e-9 { frac = max(-0.5, min(0.5, 0.5 * (prev - next) / den)) }
            }
            let absolute = (Double(k) + frac) * binWidth
            out.append(AngleFusion.Candidate(theta: nearTheta + wrapAngle(absolute - nearTheta), score: score[k]))
        }
        return out
    }
}
