import Foundation

/// Samples a small planar element of a body's surface. The element's two in-plane directions are carried by the
/// body, so one material patch yields the same samples whatever angle the body has turned through and wherever
/// the camera stands — as far as the element really is planar, opaque and evenly lit, which is what depth and
/// photometry limit, not the geometry here. The plane is measured from depth, never assumed from the rotation
/// axis, and it is projected exactly: the image of a plane is a homography, so every sample sits where the
/// pinhole camera really puts it.
struct SurfacePatchSampler {
    /// Samples per side of the square patch.
    let side: Int
    /// Half-extent of the patch on the body, in metres.
    let halfSize: Double

    var count: Int { side * side }

    /// The exact projection of one surface element, walked by addition: sample (i, j) lands at
    /// ((nx + i·nxu + j·nxv) / (d + i·du + j·dv) + cx, …). The bounds are those of the projected quadrilateral.
    struct Warp {
        var nx: Double, ny: Double, d: Double
        var nxu: Double, nyu: Double, du: Double
        var nxv: Double, nyv: Double, dv: Double
        var cx: Double, cy: Double
        var minX: Double, maxX: Double, minY: Double, maxY: Double
        var centerX: Double, centerY: Double
    }

    /// Sampling grid for a surface element at `center` spanned by the half-extent vectors `u` and `v`. Nil when
    /// the element faces away, does not lie wholly in front of the camera, or covers too little of the image to
    /// describe.
    func warp(center: V3, u: V3, v: V3, view: PinholeProjection) -> Warp? {
        guard side > 1, u.cross(v).dot(view.eye - center) > 0, let projected = view.project(center) else {
            return nil
        }
        let steps = Double(side - 1)
        let origin = view.homogeneous(center - u - v)
        let alongU = view.homogeneous(direction: u * (2 / steps))
        let alongV = view.homogeneous(direction: v * (2 / steps))
        var minX = Double.infinity, maxX = -Double.infinity
        var minY = Double.infinity, maxY = -Double.infinity
        var corners: [(x: Double, y: Double)] = []
        for (i, j) in [(0.0, 0.0), (steps, 0.0), (steps, steps), (0.0, steps)] {
            let d = origin.d + alongU.d * i + alongV.d * j
            guard d > 0.05 else { return nil }
            let x = (origin.nx + alongU.nx * i + alongV.nx * j) / d + view.intrinsics.cx
            let y = (origin.ny + alongU.ny * i + alongV.ny * j) / d + view.intrinsics.cy
            corners.append((x, y))
            minX = min(minX, x)
            maxX = max(maxX, x)
            minY = min(minY, y)
            maxY = max(maxY, y)
        }
        // The projected quadrilateral must give each sample at least a tenth of a pixel of area, or the samples
        // describe the interpolation between pixels rather than the body.
        var area = 0.0
        for index in corners.indices {
            let a = corners[index], b = corners[(index + 1) % corners.count]
            area += a.x * b.y - b.x * a.y
        }
        guard abs(area) / 2 > 0.1 * steps * steps else { return nil }
        return Warp(
            nx: origin.nx, ny: origin.ny, d: origin.d, nxu: alongU.nx, nyu: alongU.ny, du: alongU.d,
            nxv: alongV.nx, nyv: alongV.ny, dv: alongV.d, cx: view.intrinsics.cx, cy: view.intrinsics.cy,
            minX: minX, maxX: maxX, minY: minY, maxY: maxY, centerX: projected.x, centerY: projected.y)
    }

    /// Writes the zero-mean unit-norm samples of `image` under `warp`, shifted by (`offsetX`, `offsetY`) pixels,
    /// into `buffer`. False when the patch would leave the image or carries no contrast.
    func fill(
        _ buffer: inout [Float], from image: GrayImage, warp: Warp, offsetX: Double = 0, offsetY: Double = 0
    ) -> Bool {
        guard warp.minX + offsetX >= 0, warp.minY + offsetY >= 0, warp.maxX + offsetX <= Double(image.width - 1),
            warp.maxY + offsetY <= Double(image.height - 1)
        else { return false }
        let shiftX = warp.cx + offsetX, shiftY = warp.cy + offsetY
        return buffer.withUnsafeMutableBufferPointer { out in
            var sum: Float = 0
            var index = 0
            for j in 0..<side {
                let row = Double(j)
                var nx = warp.nx + warp.nxv * row
                var ny = warp.ny + warp.nyv * row
                var d = warp.d + warp.dv * row
                for _ in 0..<side {
                    let inverse = 1 / d
                    let value = image.bilinear(Float(nx * inverse + shiftX), Float(ny * inverse + shiftY))
                    out[index] = value
                    sum += value
                    index += 1
                    nx += warp.nxu
                    ny += warp.nyu
                    d += warp.du
                }
            }
            let mean = sum / Float(out.count)
            var square: Float = 0
            for index in 0..<out.count {
                out[index] -= mean
                square += out[index] * out[index]
            }
            guard square > 1e-9 else { return false }
            let inverse = 1 / square.squareRoot()
            for index in 0..<out.count { out[index] *= inverse }
            return true
        }
    }

    /// Correlation of two normalised sample sets, in [-1, 1].
    @inline(__always) static func correlation(_ a: [Float], _ b: [Float]) -> Double {
        var sum: Float = 0
        a.withUnsafeBufferPointer { p in
            b.withUnsafeBufferPointer { q in
                for i in 0..<min(p.count, q.count) { sum += p[i] * q[i] }
            }
        }
        return Double(sum)
    }

    /// The surface element around a pixel, measured from depth: two orthogonal in-plane vectors of length
    /// `halfSize`, facing the camera along the ray that actually sees it. Nil when depth cannot describe a plane
    /// there — an assumed plane would
    /// transport the wrong intensities and bias every later comparison, so no landmark is worth making.
    func surfaceTangents(
        x: Double, y: Double, world: V3, range: Double, depth: DepthMap?, view: PinholeProjection,
        minConfidence: UInt8
    ) -> (u: V3, v: V3)? {
        guard let depth else { return nil }
        let spacing = max(2.0, halfSize * view.intrinsics.fx / max(range, 0.05))
        var points: [V3] = []
        points.reserveCapacity(9)
        for j in -1...1 {
            for i in -1...1 {
                let px = x + Double(i) * spacing, py = y + Double(j) * spacing
                guard let z = depth.sample(u: px / view.width, v: py / view.height, minConfidence: minConfidence)
                else { continue }
                points.append(view.unproject(x: px, y: py, range: z))
            }
        }
        guard points.count >= 6 else { return nil }
        var centroid = V3.zero
        for p in points { centroid += p }
        centroid = centroid / Double(points.count)
        var scatter = M3.zero
        for p in points {
            let d = p - centroid
            scatter = scatter + M3.outer(d, d)
        }
        let (values, vectors) = scatter.symmetricEigen()
        // Conditioning: the samples must span the plane (not a line) and lie close to it, and the plane must
        // face the camera enough for its foreshortening to stay invertible.
        guard values[2] > 1e-12, values[1] > 0.05 * values[2] else { return nil }
        let residual = (max(values[0], 0) / Double(points.count)).squareRoot()
        guard residual < 0.3 * halfSize else { return nil }
        var normal = vectors[0].normalized
        if normal.dot(view.eye - world) < 0 { normal = -normal }
        guard normal.dot((view.eye - world).normalized) > 0.17 else { return nil }
        let reference = abs(normal.dot(view.right)) < 0.9 ? view.right : view.up
        let u = (reference - normal * normal.dot(reference)).normalized
        let v = normal.cross(u)
        return (u * halfSize, v * halfSize)
    }
}
