import Foundation

/// A previous world point and its tracked pixel in the current camera.
struct RotationImageMatch {
    var id: Int
    var point: V3
    var x: Double
    var y: Double
}

/// Separates stationary points from a rigid rotation about a known axis in image space.
/// One-point hypotheses are enumerated deterministically; static points cannot outvote a rotating minority.
struct RotationConsensus {
    private(set) var rotatingIDs: Set<Int> = []
    var pixelTolerance = 1.5
    var minSupport = 4

    struct Decision {
        var delta: Double?
        var support: Int = 0
        var backgroundIDs: Set<Int> = []
    }

    mutating func update(
        _ matches: [RotationImageMatch], axis: Axis, cameraToWorld: RigidTransform,
        intrinsics: CameraIntrinsics, retaining ids: Set<Int>
    ) -> Decision {
        rotatingIDs.formIntersection(ids)
        let worldToCamera = cameraToWorld.inverse
        let curves = matches.compactMap { match -> ProjectedOrbit? in
            let offset = match.point - axis.origin
            let radial = offset - axis.direction * offset.dot(axis.direction)
            let curve = ProjectedOrbit(
                id: match.id,
                center: worldToCamera.apply(match.point - radial),
                cosine: worldToCamera.applyDirection(radial),
                sine: worldToCamera.applyDirection(axis.direction.cross(radial)),
                x: match.x, y: match.y, intrinsics: intrinsics)
            guard curve.error(at: 0).isFinite else {
                rotatingIDs.remove(match.id)
                return nil
            }
            return curve
        }
        let toleranceSquared = pixelTolerance * pixelTolerance
        let staticErrors = curves.map { $0.error(at: 0) }
        func support(_ delta: Double) -> (score: Double, count: Int) {
            var score = 0.0, count = 0
            let c = cos(delta), s = sin(delta)
            for (curve, staticError) in zip(curves, staticErrors) {
                let error = curve.error(cosine: c, sine: s)
                if error < toleranceSquared && staticError > toleranceSquared {
                    score += 1 - error / toleranceSquared
                    count += 1
                }
            }
            return (score, count)
        }
        var bestDelta = 0.0, bestScore = 0.0, bestCount = 0
        for curve in curves {
            guard let delta = curve.fit(startingAt: 0) else { continue }
            let candidate = support(delta)
            if candidate.count >= minSupport && candidate.score > bestScore {
                bestDelta = delta
                bestScore = candidate.score
                bestCount = candidate.count
            }
        }
        guard bestCount >= minSupport else {
            // With no common rotation, only a plausible stationary observation can measure a held phase.
            for (curve, staticError) in zip(curves, staticErrors) where staticError > toleranceSquared {
                rotatingIDs.remove(curve.id)
            }
            return Decision()
        }
        let c = cos(bestDelta), s = sin(bestDelta)
        var decision = Decision(delta: bestDelta, support: bestCount)
        for (curve, staticError) in zip(curves, staticErrors) {
            let error = curve.error(cosine: c, sine: s)
            if error < toleranceSquared && staticError > toleranceSquared {
                rotatingIDs.insert(curve.id)
            } else if error >= toleranceSquared {
                rotatingIDs.remove(curve.id)
                if staticError < toleranceSquared { decision.backgroundIDs.insert(curve.id) }
            }
            // Indistinguishable models retain the previous membership, including at rest.
        }
        return decision
    }
}

private struct ProjectedOrbit {
    var id: Int
    var center: V3
    var cosine: V3
    var sine: V3
    var x: Double
    var y: Double
    var intrinsics: CameraIntrinsics

    func error(at angle: Double) -> Double { error(cosine: cos(angle), sine: sin(angle)) }

    func error(cosine c: Double, sine s: Double) -> Double {
        let p = center + cosine * c + sine * s
        guard p.isFinite, p.z < -0.01 else { return .infinity }
        let dx = -intrinsics.fx * p.x / p.z + intrinsics.cx - x
        let dy = intrinsics.fy * p.y / p.z + intrinsics.cy - y
        return dx * dx + dy * dy
    }

    func fit(startingAt angle: Double) -> Double? {
        var angle = angle
        for _ in 0..<4 {
            let c = cos(angle), s = sin(angle)
            let p = center + cosine * c + sine * s
            let v = sine * c - cosine * s
            guard p.isFinite, p.z < -0.01 else { return nil }
            let dx = x - (-intrinsics.fx * p.x / p.z + intrinsics.cx)
            let dy = y - (intrinsics.fy * p.y / p.z + intrinsics.cy)
            let jx = -intrinsics.fx * (v.x * p.z - p.x * v.z) / (p.z * p.z)
            let jy = intrinsics.fy * (v.y * p.z - p.y * v.z) / (p.z * p.z)
            let norm = jx * jx + jy * jy
            guard norm > 1e-12 else { return nil }
            let step = (dx * jx + dy * jy) / norm
            guard step.isFinite else { return nil }
            angle = wrapAngle(angle + min(.pi / 4, max(-.pi / 4, step)))
        }
        return angle
    }
}
