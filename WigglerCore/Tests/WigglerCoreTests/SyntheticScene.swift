import Foundation
@testable import WigglerCore

/// Deterministic pseudo-random generator so tests are reproducible.
struct LCG {
    var state: UInt64
    init(seed: UInt64) { state = seed &* 6364136223846793005 &+ 1442695040888963407 }
    mutating func next() -> Double {
        state = state &* 6364136223846793005 &+ 1442695040888963407
        return Double(state >> 11) / Double(1 << 53)
    }
    mutating func uniform(_ a: Double, _ b: Double) -> Double { a + (b - a) * next() }
    mutating func gaussian(_ sigma: Double) -> Double {
        let u1 = max(next(), 1e-12), u2 = next()
        return sigma * (-2 * log(u1)).squareRoot() * cos(2 * .pi * u2)
    }
}

/// A textured disk (blobs) rotating about its own normal, seen by a pinhole camera at the origin looking down -z.
/// Produces images and matching depth maps, like ARKit would.
struct SyntheticScene {
    let width = 480, height = 360
    let depthWidth = 128, depthHeight = 96
    let intrinsics = CameraIntrinsics(fx: 420, fy: 420, cx: 240, cy: 180)
    let center: V3
    let normal: V3
    let e1: V3, e2: V3
    let radius: Double
    var blobs: [(r: Double, a: Double, h: Double, intensity: Double)] = []   // disk points in cylindrical coords
    var background: [(V3, Double)] = []
    var rng: LCG

    init(seed: UInt64 = 7, center: V3 = V3(0.05, -0.05, -0.75), normal: V3 = V3(0.25, 0.55, 0.8), radius: Double = 0.15) {
        self.center = center
        self.normal = normal.normalized
        self.e1 = self.normal.anyOrthogonal()
        self.e2 = self.normal.cross(e1)
        self.radius = radius
        rng = LCG(seed: seed)
        for _ in 0..<500 {
            let r = radius * rng.next().squareRoot()
            let a = rng.uniform(0, 2 * .pi)
            let h = rng.uniform(-0.01, 0.01) // slight relief
            blobs.append((r, a, h, rng.uniform(-0.45, 0.45)))
        }
        for _ in 0..<300 {
            background.append((V3(rng.uniform(-1.5, 1.5), rng.uniform(-1.2, 1.2), -2.2), rng.uniform(-0.3, 0.3)))
        }
    }

    func point(_ b: (r: Double, a: Double, h: Double, intensity: Double), theta: Double) -> V3 {
        center + normal * b.h + e1 * (b.r * cos(b.a + theta)) + e2 * (b.r * sin(b.a + theta))
    }

    func project(_ p: V3) -> (Double, Double)? {
        let z = -p.z
        if z <= 0.05 { return nil }
        return (intrinsics.fx * p.x / z + intrinsics.cx, intrinsics.cy - intrinsics.fy * p.y / z)
    }

    func projectedCenter() -> (Float, Float) {
        let p = project(center)!
        return (Float(p.0), Float(p.1))
    }
    func projectedRadius() -> Float { Float(intrinsics.fx * radius / -center.z) }

    func render(theta: Double, depthNoise: Double, rng: inout LCG) -> FrameInput {
        var img = GrayImage(width: width, height: height, fill: 0.5)
        func splat(_ px: Double, _ py: Double, _ intensity: Double, sigma: Double) {
            if px < 5 || py < 5 || px > Double(width - 6) || py > Double(height - 6) { return }
            let x0 = Int(px) - 4, y0 = Int(py) - 4
            for y in max(0, y0)...min(height - 1, y0 + 8) {
                for x in max(0, x0)...min(width - 1, x0 + 8) {
                    let dx = Double(x) - px, dy = Double(y) - py
                    img.pixels[y * width + x] += Float(intensity * exp(-(dx * dx + dy * dy) / (2 * sigma * sigma)))
                }
            }
        }
        // Occlusion is ignored: the disk is in front of the background in all our tests.
        for (p, i) in background { if let q = project(p) { splat(q.0, q.1, i, sigma: 1.5) } }
        for b in blobs {
            if let q = project(point(b, theta: theta)) { splat(q.0, q.1, b.intensity, sigma: 1.6) }
        }
        for i in 0..<img.pixels.count { img.pixels[i] = min(1, max(0, img.pixels[i])) }

        // Depth map: ray/plane intersection with the disk, else the background plane.
        var depth = [Float](repeating: 0, count: depthWidth * depthHeight)
        var conf = [UInt8](repeating: 2, count: depthWidth * depthHeight)
        let sx = Double(width) / Double(depthWidth), sy = Double(height) / Double(depthHeight)
        for j in 0..<depthHeight {
            for i in 0..<depthWidth {
                let u = (Double(i) + 0.5) * sx, v = (Double(j) + 0.5) * sy
                let dir = V3((u - intrinsics.cx) / intrinsics.fx, -(v - intrinsics.cy) / intrinsics.fy, -1)
                // plane: (p - center)·normal = 0, p = t dir
                let denom = dir.dot(normal)
                var z = 2.2
                if abs(denom) > 1e-9 {
                    let t = center.dot(normal) / denom
                    let p = dir * t
                    if t > 0 && (p - center).length <= radius { z = -p.z }
                }
                depth[j * depthWidth + i] = Float(z + rng.gaussian(depthNoise))
            }
        }
        // A few low-confidence holes.
        for _ in 0..<40 {
            let k = Int(rng.uniform(0, Double(depth.count - 1)))
            conf[k] = 0
        }
        let dm = DepthMap(width: depthWidth, height: depthHeight, depth: depth, confidence: conf)
        return FrameInput(image: img, intrinsics: intrinsics, cameraToWorld: .identity, poseValid: true, depth: dm, timestamp: 0)
    }
}
