import Foundation

/// Minimal 3-vector (Double) — no simd so the core stays portable/testable everywhere.
public struct V3: Equatable, CustomStringConvertible {
    public var x: Double
    public var y: Double
    public var z: Double

    public init(_ x: Double, _ y: Double, _ z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }
    public static let zero = V3(0, 0, 0)

    @inline(__always) public static func + (a: V3, b: V3) -> V3 { V3(a.x + b.x, a.y + b.y, a.z + b.z) }
    @inline(__always) public static func - (a: V3, b: V3) -> V3 { V3(a.x - b.x, a.y - b.y, a.z - b.z) }
    @inline(__always) public static prefix func - (a: V3) -> V3 { V3(-a.x, -a.y, -a.z) }
    @inline(__always) public static func * (a: V3, s: Double) -> V3 { V3(a.x * s, a.y * s, a.z * s) }
    @inline(__always) public static func * (s: Double, a: V3) -> V3 { V3(a.x * s, a.y * s, a.z * s) }
    @inline(__always) public static func / (a: V3, s: Double) -> V3 { V3(a.x / s, a.y / s, a.z / s) }
    @inline(__always) public static func += (a: inout V3, b: V3) { a = a + b }
    @inline(__always) public static func -= (a: inout V3, b: V3) { a = a - b }

    @inline(__always) public func dot(_ b: V3) -> Double { x * b.x + y * b.y + z * b.z }
    @inline(__always) public func cross(_ b: V3) -> V3 {
        V3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x)
    }
    @inline(__always) public var lengthSquared: Double { dot(self) }
    @inline(__always) public var length: Double { lengthSquared.squareRoot() }
    public var normalized: V3 {
        let l = length
        return l > 1e-300 ? self / l : V3(0, 0, 1)
    }
    public var isFinite: Bool { x.isFinite && y.isFinite && z.isFinite }

    /// Any unit vector orthogonal to self (self assumed unit).
    public func anyOrthogonal() -> V3 {
        let helper = abs(z) < 0.9 ? V3(0, 0, 1) : V3(1, 0, 0)
        return cross(helper).normalized
    }
    public var description: String { String(format: "(%.4f, %.4f, %.4f)", x, y, z) }
}

/// 3x3 matrix, row-major storage.
public struct M3: Equatable {
    public var m: [Double]  // 9 entries, row-major
    public init(_ m: [Double]) {
        precondition(m.count == 9)
        self.m = m
    }
    public static let zero = M3([Double](repeating: 0, count: 9))
    public static let identity = M3([1, 0, 0, 0, 1, 0, 0, 0, 1])

    @inline(__always) public subscript(r: Int, c: Int) -> Double {
        get { m[r * 3 + c] }
        set { m[r * 3 + c] = newValue }
    }
    public static func outer(_ a: V3, _ b: V3) -> M3 {
        M3([
            a.x * b.x, a.x * b.y, a.x * b.z,
            a.y * b.x, a.y * b.y, a.y * b.z,
            a.z * b.x, a.z * b.y, a.z * b.z,
        ])
    }
    public static func + (a: M3, b: M3) -> M3 { M3(zip(a.m, b.m).map(+)) }
    public static func * (a: M3, s: Double) -> M3 { M3(a.m.map { $0 * s }) }
    public static func * (a: M3, v: V3) -> V3 {
        V3(
            a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z,
            a.m[3] * v.x + a.m[4] * v.y + a.m[5] * v.z,
            a.m[6] * v.x + a.m[7] * v.y + a.m[8] * v.z)
    }
    public static func * (a: M3, b: M3) -> M3 {
        var r = M3.zero
        for i in 0..<3 {
            for j in 0..<3 {
                var s = 0.0
                for k in 0..<3 { s += a[i, k] * b[k, j] }
                r[i, j] = s
            }
        }
        return r
    }
    public var transposed: M3 { M3([m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]]) }
    public var determinant: Double {
        m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6])
    }

    /// Solve A x = b (Cramer's rule). Returns nil when singular.
    public func solve(_ b: V3) -> V3? {
        let det = determinant
        if abs(det) < 1e-18 { return nil }
        func replaced(_ col: Int) -> M3 {
            var r = self
            r[0, col] = b.x
            r[1, col] = b.y
            r[2, col] = b.z
            return r
        }
        return V3(replaced(0).determinant / det, replaced(1).determinant / det, replaced(2).determinant / det)
    }

    /// Eigen-decomposition of a symmetric matrix by cyclic Jacobi rotations.
    /// Returns eigenvalues in ascending order and the matching unit eigenvectors.
    public func symmetricEigen() -> (values: [Double], vectors: [V3]) {
        var a = self
        var v = M3.identity
        for _ in 0..<50 {
            // off-diagonal norm
            let off = a[0, 1] * a[0, 1] + a[0, 2] * a[0, 2] + a[1, 2] * a[1, 2]
            if off < 1e-24 { break }
            for p in 0..<2 {
                for q in (p + 1)..<3 {
                    let apq = a[p, q]
                    if abs(apq) < 1e-300 { continue }
                    let app = a[p, p], aqq = a[q, q]
                    let theta = (aqq - app) / (2 * apq)
                    let t = (theta >= 0 ? 1.0 : -1.0) / (abs(theta) + (theta * theta + 1).squareRoot())
                    let c = 1 / (t * t + 1).squareRoot()
                    let s = t * c
                    // A' = J^T A J
                    for k in 0..<3 {
                        let akp = a[k, p], akq = a[k, q]
                        a[k, p] = c * akp - s * akq
                        a[k, q] = s * akp + c * akq
                    }
                    for k in 0..<3 {
                        let apk = a[p, k], aqk = a[q, k]
                        a[p, k] = c * apk - s * aqk
                        a[q, k] = s * apk + c * aqk
                    }
                    for k in 0..<3 {
                        let vkp = v[k, p], vkq = v[k, q]
                        v[k, p] = c * vkp - s * vkq
                        v[k, q] = s * vkp + c * vkq
                    }
                }
            }
        }
        var pairs = (0..<3).map { i in (a[i, i], V3(v[0, i], v[1, i], v[2, i])) }
        pairs.sort { $0.0 < $1.0 }
        return (pairs.map { $0.0 }, pairs.map { $0.1 })
    }
}

/// Rigid transform p' = R p + t.
public struct RigidTransform: Equatable {
    public var rotation: M3
    public var translation: V3
    public init(rotation: M3, translation: V3) {
        self.rotation = rotation
        self.translation = translation
    }
    public static let identity = RigidTransform(rotation: .identity, translation: .zero)

    /// Build from a column-major 4x4 matrix (as ARKit's simd_float4x4 laid out column by column).
    public init(columnMajor4x4 m: [Double]) {
        precondition(m.count == 16)
        rotation = M3([
            m[0], m[4], m[8],
            m[1], m[5], m[9],
            m[2], m[6], m[10],
        ])
        translation = V3(m[12], m[13], m[14])
    }
    @inline(__always) public func apply(_ p: V3) -> V3 { rotation * p + translation }
    @inline(__always) public func applyDirection(_ d: V3) -> V3 { rotation * d }
    public var inverse: RigidTransform {
        let rt = rotation.transposed
        return RigidTransform(rotation: rt, translation: -(rt * translation))
    }
}

@inline(__always) public func wrapAngle(_ a: Double) -> Double {
    var r = a.truncatingRemainder(dividingBy: 2 * .pi)
    if r > .pi { r -= 2 * .pi } else if r < -.pi { r += 2 * .pi }
    return r
}

/// Angle in [0, 2π).
@inline(__always) public func positiveAngle(_ a: Double) -> Double {
    var r = a.truncatingRemainder(dividingBy: 2 * .pi)
    if r < 0 { r += 2 * .pi }
    return r
}

/// Median of a non-empty array (copy + sort; fine for the sizes we use).
public func median(_ values: [Double]) -> Double {
    guard !values.isEmpty else { return 0 }
    let s = values.sorted()
    let n = s.count
    return n % 2 == 1 ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2])
}

public func percentile(_ values: [Double], _ p: Double) -> Double {
    guard !values.isEmpty else { return 0 }
    let s = values.sorted()
    let idx = max(0, min(s.count - 1, Int((Double(s.count - 1) * p).rounded())))
    return s[idx]
}
