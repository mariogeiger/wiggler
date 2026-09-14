import Foundation

/// Normal equations shared by pixels with complete observations, or per pixel when observations are missing.
/// Symmetric matrices use lower-triangle planes: plane r(r+1)/2+c contains N[r,c] for c ≤ r.
struct HarmonicMoments {
    let terms: Int
    let pixelCount: Int
    private var spatialCount = 1
    private var moments: [Double]
    private(set) var maximumWeight = 0.0

    init(terms: Int, pixelCount: Int) {
        self.terms = terms
        self.pixelCount = pixelCount
        moments = [Double](repeating: 0, count: terms * (terms + 1) / 2)
    }

    mutating func reset() {
        spatialCount = 1
        moments = [Double](repeating: 0, count: terms * (terms + 1) / 2)
        maximumWeight = 0
    }

    mutating func add(basis: [Double], delta: Double, decay: Double, weight: Double, valid: [Bool]?) {
        if valid != nil && spatialCount == 1 && pixelCount > 1 {
            moments = moments.flatMap { [Double](repeating: $0, count: pixelCount) }
            spatialCount = pixelCount
        }
        let n = spatialCount
        // Recenter τ = θ_sample − θ_now before decaying and adding the new observation at τ = 0.
        for i in 0..<n {
            moments[2 * n + i] += -2 * delta * moments[n + i] + delta * delta * moments[i]
        }
        for j in 0..<terms where j != 1 {
            let row = max(1, j), col = min(1, j)
            let target = (row * (row + 1) / 2 + col) * n
            let source = j * (j + 1) / 2 * n
            for i in 0..<n { moments[target + i] -= delta * moments[source + i] }
        }
        for row in 0..<terms {
            for col in 0...row {
                let start = (row * (row + 1) / 2 + col) * n
                let increment = weight * basis[row] * basis[col]
                for i in 0..<n {
                    moments[start + i] = decay * moments[start + i] + (valid?[i] ?? true ? increment : 0)
                }
            }
        }
        maximumWeight = moments.prefix(n).max() ?? 0
    }

    /// hᵀN⁻¹r at each pixel. Missing angular coverage or a singular fit produces NaN, never a zero sample.
    func project(rhs: [Float], weights: [Double], minimumWeight: Double) -> [Float] {
        var output = [Float](repeating: .nan, count: pixelCount)
        var factor = [Double](repeating: 0, count: terms * (terms + 1) / 2)
        var solution = weights
        if spatialCount == 1 {
            guard moments[0] >= minimumWeight, solve(pixel: 0, rhs: &solution, factor: &factor) else { return output }
            output = [Float](repeating: 0, count: pixelCount)
            for k in 0..<terms {
                let coefficient = Float(solution[k]), start = k * pixelCount
                for i in 0..<pixelCount { output[i] += coefficient * rhs[start + i] }
            }
        } else {
            for i in 0..<pixelCount where moments[i] >= minimumWeight {
                for k in 0..<terms { solution[k] = Double(rhs[k * pixelCount + i]) }
                guard solve(pixel: i, rhs: &solution, factor: &factor) else { continue }
                var value = 0.0
                for k in 0..<terms { value += weights[k] * solution[k] }
                output[i] = Float(value)
            }
        }
        return output
    }

    /// Cholesky factorization and two triangular solves; scratch buffers are reused across pixels.
    private func solve(pixel: Int, rhs: inout [Double], factor: inout [Double]) -> Bool {
        for k in factor.indices { factor[k] = moments[k * spatialCount + pixel] }
        var trace = 0.0
        for row in 0..<terms { trace += factor[row * (row + 1) / 2 + row] }
        let tolerance = 1e-10 * trace
        for row in 0..<terms {
            let r = row * (row + 1) / 2
            for col in 0...row {
                let c = col * (col + 1) / 2
                var value = factor[r + col]
                for k in 0..<col { value -= factor[r + k] * factor[c + k] }
                if row == col {
                    guard value > tolerance else { return false }
                    factor[r + col] = sqrt(value)
                } else {
                    factor[r + col] = value / factor[c + col]
                }
            }
            var value = rhs[row]
            for col in 0..<row { value -= factor[r + col] * rhs[col] }
            rhs[row] = value / factor[r + row]
        }
        for row in (0..<terms).reversed() {
            var value = rhs[row]
            for col in (row + 1)..<terms { value -= factor[col * (col + 1) / 2 + row] * rhs[col] }
            rhs[row] = value / factor[row * (row + 1) / 2 + row]
        }
        return true
    }
}
