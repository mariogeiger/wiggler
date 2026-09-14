import XCTest

@testable import WigglerCore

final class KLTPyramidRefinementTests: XCTestCase {
    func testFineTextureSurvivesUninformativeCoarseLevels() {
        let first = quarterPeriodTexture(dx: 0, dy: 0)
        let previous = Pyramid(image: first, levelCount: 4)
        for level in previous.levels.dropFirst() {
            XCTAssertEqual(level.at(level.width / 2, level.height / 2), 0.5)
            XCTAssertEqual(level.at(level.width / 2 + 1, level.height / 2), 0.5)
            XCTAssertEqual(level.at(level.width / 2, level.height / 2 + 1), 0.5)
        }
        for shift in [0.0, 0.25, 0.75, -0.25, -0.75] {
            let second = quarterPeriodTexture(dx: shift, dy: -shift / 2)
            let current = Pyramid(image: second, levelCount: 4)
            for (x, y): (Float, Float) in [(32, 32), (40, 48), (64, 64), (80, 72), (96, 96)] {
                let fine = KLTTracker().track(
                    prev: Pyramid(image: first, levelCount: 1), cur: Pyramid(image: second, levelCount: 1), x: x, y: y)
                let multiscale = KLTTracker().track(prev: previous, cur: current, x: x, y: y)
                XCTAssertTrue(fine.ok)
                XCTAssertTrue(multiscale.ok, "shift \(shift), point \(x),\(y)")
                XCTAssertEqual(multiscale.x, fine.x)
                XCTAssertEqual(multiscale.y, fine.y)
                XCTAssertEqual(multiscale.residual, fine.residual)
                XCTAssertLessThan(hypot(Double(multiscale.x - x) - shift, Double(multiscale.y - y) + shift / 2), 0.1)
            }
        }
    }

    func testFailedCoarseUpdatesCannotMoveAnUnchangedFinePatch() {
        var accepted = 0
        for seed in 1...50 {
            let first = patchWithChangingSurroundings(seed: seed, changed: false)
            let second = patchWithChangingSurroundings(seed: seed, changed: true)
            let fine = KLTTracker().track(
                prev: Pyramid(image: first, levelCount: 1), cur: Pyramid(image: second, levelCount: 1), x: 64, y: 64)
            XCTAssertTrue(fine.ok)
            XCTAssertEqual(fine.x, 64)
            XCTAssertEqual(fine.y, 64)
            let multiscale = KLTTracker().track(
                prev: Pyramid(image: first, levelCount: 4), cur: Pyramid(image: second, levelCount: 4), x: 64, y: 64)
            if [2, 4, 9].contains(seed) { XCTAssertTrue(multiscale.ok, "seed \(seed)") }
            if multiscale.ok {
                accepted += 1
                XCTAssertLessThan(hypot(multiscale.x - 64, multiscale.y - 64), 0.1, "seed \(seed)")
            }
        }
        XCTAssertGreaterThanOrEqual(accepted, 3)
    }

    func testFinestAffineRankFailureRemainsFatal() {
        var image = GrayImage(width: 128, height: 128)
        for y in 0..<128 {
            for x in 0..<128 {
                let u = Float(x - 64), v = Float(y - 64)
                image.pixels[y * 128 + x] = min(1, 0.2 + 0.001 * (u * u + v * v))
            }
        }
        var tracker = KLTTracker()
        tracker.minEigenvalue = 1e-8
        for levels in [1, 4] {
            let pyramid = Pyramid(image: image, levelCount: levels)
            XCTAssertFalse(tracker.track(prev: pyramid, cur: pyramid, x: 64, y: 64).ok)
        }
    }

    func testLargeTranslationStillUsesCoarseRefinement() {
        var fineCorrect = 0, multiscaleCorrect = 0
        for seed: UInt64 in [3, 11, 29] {
            let texture = makePlanarTexture(seed: seed, repeated: false)
            let first = renderPlanarTexture(texture, warp: PlanarImageWarp(angle: 0), occluded: false)
            let second = renderPlanarTexture(texture, warp: PlanarImageWarp(angle: 0, tx: 12, ty: -8), occluded: false)
            let finePrevious = Pyramid(image: first, levelCount: 1), fineCurrent = Pyramid(image: second, levelCount: 1)
            let previous = Pyramid(image: first, levelCount: 4), current = Pyramid(image: second, levelCount: 4)
            let points = CornerDetector().detect(
                in: first, centerX: 160, centerY: 120, radius: 56, exclude: [], maxCount: 30)
            for (x, y) in points {
                let fine = KLTTracker().track(prev: finePrevious, cur: fineCurrent, x: x, y: y)
                if fine.ok, hypot(fine.x - x - 12, fine.y - y + 8) < 0.1 { fineCorrect += 1 }
                let multiscale = KLTTracker().track(prev: previous, cur: current, x: x, y: y)
                if multiscale.ok {
                    XCTAssertLessThan(hypot(multiscale.x - x - 12, multiscale.y - y + 8), 0.1)
                    multiscaleCorrect += 1
                }
            }
        }
        XCTAssertGreaterThan(multiscaleCorrect, fineCorrect)
    }

    private func quarterPeriodTexture(dx: Double, dy: Double) -> GrayImage {
        var image = GrayImage(width: 128, height: 128)
        for y in 0..<128 {
            for x in 0..<128 {
                image.pixels[y * 128 + x] = Float(
                    0.5 + (sin(.pi * (Double(x) - dx) / 2) + sin(.pi * (Double(y) - dy) / 2)) / 8)
            }
        }
        return image
    }

    private func patchWithChangingSurroundings(seed: Int, changed: Bool) -> GrayImage {
        var image = GrayImage(width: 128, height: 128)
        for y in 0..<128 {
            for x in 0..<128 {
                let changedPixel = changed && (abs(x - 64) > 12 || abs(y - 64) > 12)
                let phase = Double(seed) * (changedPixel ? 1.37 : 0.31)
                let u = Double(x), v = Double(y)
                let value =
                    0.5 + 0.18 * sin(0.08 * u + phase) + 0.18 * cos(0.085 * v - phase)
                    + 0.10 * sin(0.4 * u + 0.3 * v + phase) + 0.10 * cos(0.27 * u - 0.31 * v - phase)
                image.pixels[y * 128 + x] = Float(min(1, max(0, value)))
            }
        }
        return image
    }
}
