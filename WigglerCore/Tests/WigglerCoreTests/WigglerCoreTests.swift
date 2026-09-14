import XCTest
@testable import WigglerCore

final class GeometryTests: XCTestCase {
    func testSymmetricEigen() {
        var rng = LCG(seed: 1)
        for _ in 0..<20 {
            let b = M3((0..<9).map { _ in rng.gaussian(1) })
            let a = b * b.transposed
            let (values, vectors) = a.symmetricEigen()
            XCTAssertLessThanOrEqual(values[0], values[1])
            XCTAssertLessThanOrEqual(values[1], values[2])
            for i in 0..<3 {
                let av = a * vectors[i]
                let lv = vectors[i] * values[i]
                XCTAssertLessThan((av - lv).length, 1e-8 * max(1, abs(values[i])))
                XCTAssertEqual(vectors[i].length, 1, accuracy: 1e-9)
            }
        }
    }

    func testSolve() {
        let a = M3([4, 1, 2, 1, 3, 0, 2, 0, 5])
        let x = V3(1, -2, 3)
        let b = a * x
        let s = a.solve(b)!
        XCTAssertLessThan((s - x).length, 1e-9)
    }

    func testRigidTransformInverse() {
        let m: [Double] = [0, 1, 0, 0,  -1, 0, 0, 0,  0, 0, 1, 0,  1, 2, 3, 1]  // column-major: 90° about z + translation
        let t = RigidTransform(columnMajor4x4: m)
        let p = V3(0.3, -0.7, 2)
        let q = t.inverse.apply(t.apply(p))
        XCTAssertLessThan((q - p).length, 1e-12)
        XCTAssertLessThan((t.apply(.zero) - V3(1, 2, 3)).length, 1e-12)
    }
}

final class KLTTests: XCTestCase {
    /// Track corners between a textured image and a translated + rotated copy of it.
    func testTrackingAccuracy() {
        let w = 240, h = 180
        var rng = LCG(seed: 3)
        var base = [Float](repeating: 0.5, count: (w + 40) * (h + 40))
        for _ in 0..<800 {
            let cx = rng.uniform(4, Double(w + 36)), cy = rng.uniform(4, Double(h + 36))
            let i = rng.uniform(-0.4, 0.4)
            for y in Int(cy) - 4...Int(cy) + 4 { for x in Int(cx) - 4...Int(cx) + 4 {
                let dx = Double(x) - cx, dy = Double(y) - cy
                base[y * (w + 40) + x] += Float(i * exp(-(dx * dx + dy * dy) / 5))
            } }
        }
        func warp(tx: Double, ty: Double, ang: Double) -> GrayImage {
            var out = GrayImage(width: w, height: h)
            let c = cos(ang), s = sin(ang)
            for y in 0..<h { for x in 0..<w {
                let dx = Double(x) - tx - Double(w) / 2, dy = Double(y) - ty - Double(h) / 2
                let sx = c * dx + s * dy + Double(w) / 2 + 20, sy = -s * dx + c * dy + Double(h) / 2 + 20
                let x0 = Int(sx), y0 = Int(sy)
                let fx = Float(sx - Double(x0)), fy = Float(sy - Double(y0))
                let W = w + 40
                let a = base[y0 * W + x0], b = base[y0 * W + x0 + 1], cc = base[(y0 + 1) * W + x0], d = base[(y0 + 1) * W + x0 + 1]
                out.pixels[y * w + x] = (a * (1 - fx) + b * fx) * (1 - fy) + (cc * (1 - fx) + d * fx) * fy
            } }
            return out
        }
        let (tx, ty, ang) = (6.3, -4.7, 3.0 * Double.pi / 180)
        let imgA = warp(tx: 0, ty: 0, ang: 0), imgB = warp(tx: tx, ty: ty, ang: ang)
        let pa = Pyramid(image: imgA, levelCount: 4), pb = Pyramid(image: imgB, levelCount: 4)
        let det = CornerDetector()
        let pts = det.detect(in: imgA, centerX: Float(w) / 2, centerY: Float(h) / 2, radius: 70, exclude: [], maxCount: 60)
        XCTAssertGreaterThan(pts.count, 30)
        let klt = KLTTracker()
        var errors: [Double] = []
        let c = cos(ang), s = sin(ang)
        for (x, y) in pts {
            let r = klt.track(prev: pa, cur: pb, x: x, y: y)
            guard r.ok, r.residual < 0.12 else { continue }
            let ex = c * (Double(x) - Double(w) / 2) - s * (Double(y) - Double(h) / 2) + Double(w) / 2 + tx
            let ey = s * (Double(x) - Double(w) / 2) + c * (Double(y) - Double(h) / 2) + Double(h) / 2 + ty
            errors.append(((Double(r.x) - ex) * (Double(r.x) - ex) + (Double(r.y) - ey) * (Double(r.y) - ey)).squareRoot())
        }
        XCTAssertGreaterThan(errors.count, 25)
        XCTAssertLessThan(median(errors), 0.3)
    }
}

final class AxisAndAngleTests: XCTestCase {
    func testAxisFromNoisyChords() {
        var rng = LCG(seed: 11)
        let nTrue = V3(0.2, 0.9, -0.3).normalized
        let cTrue = V3(0.1, -0.2, 0.8)
        let axTrue = Axis(origin: cTrue, direction: nTrue)
        var pts: [(r: Double, a: Double, h: Double)] = []
        for _ in 0..<120 { pts.append((rng.uniform(0.03, 0.2), rng.uniform(0, 2 * .pi), rng.uniform(-0.1, 0.1))) }
        func pos(_ p: (r: Double, a: Double, h: Double), _ th: Double) -> V3 {
            axTrue.origin + axTrue.direction * p.h + axTrue.e1 * (p.r * cos(p.a + th)) + axTrue.e2 * (p.r * sin(p.a + th))
        }
        var est = AxisEstimator()
        var theta = 0.0
        var history: [[V3]] = []
        for t in 0..<240 {
            theta += (2 * .pi * 100 / 60 / 60) * pow(sin(Double(t) / 80), 2)
            var frame: [V3] = []
            for p in pts { frame.append(pos(p, theta) + V3(rng.gaussian(0.006), rng.gaussian(0.006), rng.gaussian(0.006))) }
            // outliers: random walkers ("hands")
            for k in 0..<10 { frame.append(V3(0.3 + Double(k) * 0.01 + rng.gaussian(0.02), rng.gaussian(0.02), 0.8 + rng.gaussian(0.02))) }
            history.append(frame)
            if t > 0 {
                for i in 0..<frame.count {
                    for k in 1...min(45, t) {
                        let d = frame[i] - history[t - k][i]
                        if d.length >= 0.03 {
                            est.add(ChordConstraint(midpoint: (frame[i] + history[t - k][i]) * 0.5, chord: d, frame: t))
                            break
                        }
                    }
                }
            }
        }
        let e = est.estimate()!
        XCTAssertTrue(e.isWellConditioned, "planarity \(e.planarity) coverage \(e.coverage) inliers \(e.inlierRatio)")
        let dirErr = acos(abs(e.axis.direction.dot(nTrue))) * 180 / .pi
        XCTAssertLessThan(dirErr, 1.5)
        let d = e.axis.origin - cTrue
        let perp = (d - nTrue * d.dot(nTrue)).length
        XCTAssertLessThan(perp, 0.006)
    }

    func testAngleTrackerFollowsRotationWithOutliers() {
        var rng = LCG(seed: 5)
        let ax = Axis(origin: V3(0, 0, 0), direction: V3(0, 0, 1))
        var offsets: [Double] = [], radii: [Double] = []
        for _ in 0..<80 { offsets.append(rng.uniform(0, 2 * .pi)); radii.append(rng.uniform(0.03, 0.2)) }
        var tracker = AngleTracker()
        var theta = 0.0
        var maxErr = 0.0
        var hands: [Double] = (0..<8).map { _ in rng.uniform(0, 2 * .pi) }
        for t in 0..<400 {
            theta += (2 * .pi * 100 / 60 / 60) * pow(sin(Double(t) / 60), 2)   // up to 100 rpm at 60 fps
            var obs: [AngleObservation] = []
            for i in 0..<80 {
                let noise = rng.gaussian(0.006) / radii[i]  // ~6 mm position noise
                obs.append(AngleObservation(id: i, phi: wrapAngle(offsets[i] + theta + noise), radius: radii[i]))
            }
            for k in 0..<8 {
                hands[k] += rng.gaussian(0.05)
                obs.append(AngleObservation(id: 1000 + k, phi: wrapAngle(hands[k]), radius: 0.4))
            }
            // static background points
            for k in 0..<15 { obs.append(AngleObservation(id: 2000 + k, phi: Double(k) * 0.4, radius: 0.5)) }
            let up = tracker.update(obs)
            if t > 5 {
                XCTAssertTrue(up.ok, "frame \(t)")
                maxErr = max(maxErr, abs(wrapAngle(up.theta - theta)))
            }
        }
        XCTAssertLessThan(maxErr * 180 / .pi, 5)
        _ = ax
    }

    func testRelocalizerPeriodDetection() {
        var rng = LCG(seed: 9)
        var reloc = Relocalizer(side: 8, binCount: 36)
        // A pattern with two-fold symmetry: patch(θ) = f(2θ) + static background.
        let bg = (0..<64).map { _ in Float(rng.uniform(0, 1)) }
        func patch(_ theta: Double) -> [Float] {
            var p = bg
            for i in 0..<64 {
                p[i] += 0.3 * Float(sin(2 * theta + Double(i) * 0.7) + cos(4 * theta + Double(i) * 1.3))
            }
            return p
        }
        for b in 0..<36 { reloc.record(patch: patch(Double(b) * 10 * .pi / 180), theta: Double(b) * 10 * .pi / 180) }
        XCTAssertTrue(reloc.isComplete)
        reloc.analyse()
        XCTAssertEqual(reloc.period * 180 / .pi, 180, accuracy: 1e-6)
        XCTAssertFalse(reloc.isRotationallySymmetric)
        let m = reloc.match(patch: patch(47 * .pi / 180), nearTheta: 40 * .pi / 180)!
        XCTAssertEqual(m.theta * 180 / .pi, 47, accuracy: 4)
        XCTAssertTrue(m.confident)
        // Same appearance half a turn later: the candidate nearest to the current estimate wins.
        let m2 = reloc.match(patch: patch(47 * .pi / 180), nearTheta: 230 * .pi / 180)!
        XCTAssertEqual(m2.theta * 180 / .pi, 227, accuracy: 4)
    }
}

final class EngineEndToEndTests: XCTestCase {
    func testLocksAxisAndTracksAngle() {
        let scene = SyntheticScene()
        var rng = LCG(seed: 21)
        var config = EngineConfig()
        config.constraintWindowFrames = 600
        let engine = RotationEngine(config: config)
        let (mx, my) = scene.projectedCenter()
        engine.setMarker(x: mx, y: my, radius: scene.projectedRadius() * 1.15)
        var theta = 0.0
        var lockedAt: Int?
        var thetaAtLock = 0.0
        var engineThetaAtLock = 0.0
        var maxErr = 0.0
        var last = EngineOutput()
        for f in 0..<900 {
            // 0 → ~40 rpm, with a pause and a reversal to exercise variable speed.
            let rpm: Double
            switch f {
            case 0..<300: rpm = 40 * Double(f) / 300
            case 300..<420: rpm = 25
            case 420..<480: rpm = 0
            default: rpm = -30
            }
            theta += rpm / 60 * 2 * .pi / 60
            var input = scene.render(theta: theta, depthNoise: 0.004, rng: &rng)
            input.timestamp = Double(f) / 60
            last = engine.process(input)
            if last.state == .locked && lockedAt == nil {
                lockedAt = f
                thetaAtLock = theta
                engineThetaAtLock = last.theta
            }
            if let l = lockedAt, f > l + 5, last.angleConfidence > 0 {
                let err = abs(wrapAngle((last.theta - engineThetaAtLock) - (theta - thetaAtLock)))
                maxErr = max(maxErr, err)
            }
        }
        XCTAssertNotNil(lockedAt, "engine never locked: \(last.message) tracks=\(last.trackCount) constraints=\(last.constraintCount)")
        guard let axis = last.axis else { return XCTFail("no axis") }
        let dirErr = acos(abs(axis.direction.dot(scene.normal))) * 180 / .pi
        XCTAssertLessThan(dirErr, 3, "axis direction error \(dirErr)°")
        let d = axis.origin - scene.center
        let perp = (d - scene.normal * d.dot(scene.normal)).length
        XCTAssertLessThan(perp, 0.01, "axis offset \(perp) m")
        XCTAssertLessThan(maxErr * 180 / .pi, 6, "max angle error \(maxErr * 180 / .pi)°")
        XCTAssertGreaterThan(last.angleConfidence, 0.3)
        XCTAssertEqual(last.periodDegrees, 360, accuracy: 1e-6)
    }
}
