import Foundation

/// A rotation axis in world coordinates, with the object's extent for display.
public struct Axis: Equatable, Codable {
    /// A point on the axis (chosen near the object's centroid height).
    public var origin: V3
    /// Unit direction.
    public var direction: V3
    /// Orthonormal basis of the plane perpendicular to the axis (e1, e2, direction) is right-handed,
    /// so angles measured with (e1, e2) increase for a rotation that is counter-clockwise when seen from +direction.
    public var e1: V3
    public var e2: V3

    public init(origin: V3, direction: V3) {
        self.origin = origin
        self.direction = direction.normalized
        self.e1 = self.direction.anyOrthogonal()
        self.e2 = self.direction.cross(self.e1)
    }

    /// Azimuth of a point around the axis, and its distance to the axis and height along it.
    @inline(__always) public func cylindrical(_ p: V3) -> (phi: Double, radius: Double, height: Double) {
        let d = p - origin
        let a = d.dot(e1), b = d.dot(e2)
        return (atan2(b, a), (a * a + b * b).squareRoot(), d.dot(direction))
    }

    /// Keep the sign of `direction` consistent with a reference (axes are lines; the sign is arbitrary).
    public func alignedSign(with reference: V3) -> Axis {
        direction.dot(reference) < 0 ? Axis(origin: origin, direction: -direction) : self
    }
}

/// One "chord" observation: a material point moved from a to b; the chord (b - a) is perpendicular to the
/// axis direction, and the segment from the axis to the chord midpoint is perpendicular to the chord.
public struct ChordConstraint: Codable {
    public var midpoint: V3
    public var chord: V3
    public var frame: Int
    public init(midpoint: V3, chord: V3, frame: Int) {
        self.midpoint = midpoint
        self.chord = chord
        self.frame = frame
    }
}

public struct AxisEstimate {
    public var axis: Axis
    /// Eigenvalues (ascending) of the weighted chord scatter matrix: [λ0, λ1, λ2].
    /// λ0/λ1 small ⇒ chords are coplanar; λ1/λ2 close to 1 ⇒ good angular coverage.
    public var eigenvalues: [Double]
    public var inlierRatio: Double
    public var constraintCount: Int
    public var planarity: Double { eigenvalues[1] > 0 ? eigenvalues[0] / eigenvalues[1] : 1 }
    public var coverage: Double { eigenvalues[2] > 0 ? eigenvalues[1] / eigenvalues[2] : 0 }
    public var isWellConditioned: Bool {
        constraintCount >= 150 && inlierRatio >= 0.45 && planarity < 0.2 && coverage > 0.25
    }
}

/// Robust (IRLS / Huber) least-squares estimation of the rotation axis from chord constraints.
public struct AxisEstimator {
    public var capacity: Int = 6000
    public var huberMeters: Double = 0.03
    public var iterations: Int = 8
    /// Constraints in chronological order.
    private(set) public var constraints: [ChordConstraint] = []

    public init() {}

    public mutating func add(_ c: ChordConstraint) {
        constraints.append(c)
        if constraints.count > capacity + capacity / 4 {
            constraints.removeFirst(constraints.count - capacity)
        }
    }

    public mutating func removeAll() { constraints.removeAll(keepingCapacity: true) }

    /// Drop constraints older than `frame`.
    public mutating func prune(before frame: Int) {
        if let first = constraints.firstIndex(where: { $0.frame >= frame }) {
            if first > 0 { constraints.removeFirst(first) }
        } else {
            constraints.removeAll(keepingCapacity: true)
        }
    }

    public var newestFrame: Int { constraints.last?.frame ?? -1 }
    public var oldestFrame: Int { constraints.first?.frame ?? -1 }

    /// Estimate from all constraints, or from those newer than `frame` only.
    public func estimate(huber: Double? = nil, newerThan frame: Int = -1) -> AxisEstimate? {
        let cs = frame < 0 ? constraints : Array(constraints.drop(while: { $0.frame <= frame }))
        let n = cs.count
        if n < 30 { return nil }
        let hub = huber ?? huberMeters
        var weights = [Double](repeating: 1, count: n)
        let lengths = cs.map { $0.chord.length }
        let units = zip(cs, lengths).map { $0.0.chord / max($0.1, 1e-9) }
        var axis: Axis?
        var evals = [0.0, 0.0, 0.0]
        var inlierRatio = 0.0
        var meanMid = V3.zero
        for c in cs { meanMid += c.midpoint }
        meanMid = meanMid / Double(n)

        for _ in 0..<iterations {
            // Weighted scatter of the chords: M = Σ w L² u uᵀ.  Axis direction = eigenvector of the smallest eigenvalue.
            var m = M3.zero
            var totalW = 0.0
            for i in 0..<n {
                let w = weights[i] * lengths[i] * lengths[i]
                if w <= 0 { continue }
                let u = units[i]
                m.m[0] += w * u.x * u.x
                m.m[1] += w * u.x * u.y
                m.m[2] += w * u.x * u.z
                m.m[4] += w * u.y * u.y
                m.m[5] += w * u.y * u.z
                m.m[8] += w * u.z * u.z
                totalW += w
            }
            if totalW <= 0 { return nil }
            m.m[3] = m.m[1]
            m.m[6] = m.m[2]
            m.m[7] = m.m[5]
            let (values, vectors) = m.symmetricEigen()
            evals = values.map { $0 / totalW }
            let dir = vectors[0].normalized
            // Center: minimise Σ w ((mid - c)·u)²  with c constrained to the plane through meanMid ⊥ dir
            // (the component along the axis is not observable): (M + d dᵀ) c = Σ w u (u·mid) + d (d·meanMid).
            var rhs = V3.zero
            for i in 0..<n {
                let w = weights[i] * lengths[i] * lengths[i]
                if w <= 0 { continue }
                rhs += units[i] * (w * units[i].dot(cs[i].midpoint))
            }
            rhs += dir * dir.dot(meanMid)
            let a = m + M3.outer(dir, dir)
            guard let center = a.solve(rhs), center.isFinite else { return nil }
            let ax = Axis(origin: center, direction: dir)
            axis = ax
            // Residuals (meters): out-of-plane chord component and violation of the mid-chord perpendicularity.
            var inliers = 0
            for i in 0..<n {
                let r1 = abs(units[i].dot(dir)) * lengths[i]
                let r2 = abs((cs[i].midpoint - center).dot(units[i]))
                let r = (r1 * r1 + r2 * r2).squareRoot()
                if r < hub {
                    weights[i] = 1
                    inliers += 1
                } else {
                    weights[i] = hub / r
                }
            }
            inlierRatio = Double(inliers) / Double(n)
        }
        guard let ax = axis else { return nil }
        return AxisEstimate(axis: ax, eigenvalues: evals, inlierRatio: inlierRatio, constraintCount: n)
    }
}
