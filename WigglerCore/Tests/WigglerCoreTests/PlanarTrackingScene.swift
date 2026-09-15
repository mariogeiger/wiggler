import Foundation

@testable import WigglerCore

struct PlanarTextureRandom {
    var state: UInt64
    mutating func uniform(_ lo: Double, _ hi: Double) -> Double {
        state = state &* 6364136223846793005 &+ 1442695040888963407
        return lo + (hi - lo) * Double(state >> 11) / Double(UInt64(1) << 53)
    }
}

struct PlanarImageWarp {
    var angle: Double
    var squash: Double = 1
    var perspective: Double = 0
    var tx: Double = 0
    var ty: Double = 0
    func project(_ u: Double, _ v: Double) -> (Double, Double) {
        let c = cos(angle), s = sin(angle)
        let x = c * u - s * v, y = s * u + c * v
        let d = 1 + perspective * y
        return (160 + tx + x / d, 120 + ty + squash * y / d)
    }
    func unproject(_ x: Double, _ y: Double) -> (Double, Double) {
        let px = x - 160 - tx, py = y - 120 - ty
        let ry = py / (squash - perspective * py)
        let rx = px * (1 + perspective * ry)
        let c = cos(angle), s = sin(angle)
        return (c * rx + s * ry, -s * rx + c * ry)
    }
}

/// Textures are immutable and take a while to synthesise (thousands of Gaussian splats): one per (seed, kind).
private var planarTextures: [String: GrayImage] = [:]
private let planarTextureLock = NSLock()

func makePlanarTexture(seed: UInt64, repeated: Bool) -> GrayImage {
    let key = "\(seed)/\(repeated)"
    planarTextureLock.lock()
    defer { planarTextureLock.unlock() }
    if let cached = planarTextures[key] { return cached }
    let texture = synthesisePlanarTexture(seed: seed, repeated: repeated)
    planarTextures[key] = texture
    return texture
}

private func synthesisePlanarTexture(seed: UInt64, repeated: Bool) -> GrayImage {
    let side = 640
    var image = GrayImage(width: side, height: side, fill: 0.5)
    var random = PlanarTextureRandom(state: seed)
    if repeated {
        for y in 0..<side {
            for x in 0..<side {
                image.pixels[y * side + x] += Float(0.19 * cos(Double(x) * .pi / 12) * cos(Double(y) * .pi / 12))
            }
        }
    }
    for _ in 0..<(repeated ? 900 : 8500) {
        let x = random.uniform(20, Double(side - 20)), y = random.uniform(20, Double(side - 20))
        let sigma = random.uniform(1.2, 5.5)
        let amplitude = random.uniform(-0.25, 0.25)
        let r = Int(3 * sigma)
        for iy in Int(y) - r...Int(y) + r {
            for ix in Int(x) - r...Int(x) + r {
                let dx = Double(ix) - x, dy = Double(iy) - y
                image.pixels[iy * side + ix] += Float(amplitude * exp(-(dx * dx + dy * dy) / (2 * sigma * sigma)))
            }
        }
    }
    image.pixels = image.pixels.map { min(1, max(0, $0)) }
    return image
}

func renderPlanarTexture(_ texture: GrayImage, warp: PlanarImageWarp, occluded: Bool) -> GrayImage {
    var image = GrayImage(width: 320, height: 240)
    for y in 0..<240 {
        for x in 0..<320 {
            var value: Float = 0
            for dy in [-0.25, 0.25] {
                for dx in [-0.25, 0.25] {
                    let (u, v) = warp.unproject(Double(x) + dx, Double(y) + dy)
                    value += texture.bilinear(Float(u + 320), Float(v + 320)) * 0.25
                }
            }
            image.pixels[y * 320 + x] = occluded && x >= 160 && x < 210 && y >= 80 && y < 160 ? 0.5 : value
        }
    }
    return image
}

func planarPatchIsVisible(x: Double, y: Double, occluded: Bool) -> Bool {
    let radius = 6.0
    guard x >= radius, y >= radius, x <= 319 - radius, y <= 239 - radius else { return false }
    return !occluded || x + radius < 160 || x - radius >= 210 || y + radius < 80 || y - radius >= 160
}

func planarPatchHasTextureSupport(x: Double, y: Double, warp: PlanarImageWarp, texture: GrayImage) -> Bool {
    // This footprint includes the coarsest template and pyramid filtering support.
    for dy in [-64.0, 64.0] {
        for dx in [-64.0, 64.0] {
            let denominator = warp.squash - warp.perspective * (y + dy - 120 - warp.ty)
            guard denominator > 0 else { return false }
            let (u, v) = warp.unproject(x + dx, y + dy)
            guard u.isFinite, v.isFinite, u + 320 >= 0, v + 320 >= 0,
                u + 320 < Double(texture.width - 1), v + 320 < Double(texture.height - 1)
            else { return false }
        }
    }
    return true
}
