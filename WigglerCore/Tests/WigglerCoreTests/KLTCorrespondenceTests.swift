import XCTest

@testable import WigglerCore

final class KLTCorrespondenceTests: XCTestCase {
    func testRetainedIDsFollowMaterialPointsThroughRotation() {
        assertCorrespondence(degrees: 3)
    }

    func testRetainedIDsFollowMaterialPointsThroughProjectiveRotation() {
        assertCorrespondence(degrees: 2, squash: 0.65, perspective: 0.0015)
    }

    func testTranslationKeepsSupportAndSubpixelAccuracy() {
        assertCorrespondence(degrees: 0)
    }

    func testRepeatedTextureKeepsOnlyAccurateMaterialCorrespondences() {
        assertCorrespondence(degrees: 3, repeated: true)
    }

    func testOccludedTracksCannotSurviveAsDifferentMaterialPoints() {
        assertCorrespondence(degrees: 3, occlusion: true)
    }

    func testAffineRankIsRequiredBeyondTranslationCornerStrength() {
        var image = GrayImage(width: 64, height: 64)
        for y in 0..<64 {
            for x in 0..<64 {
                let u = Float(x - 32), v = Float(y - 32)
                image.pixels[y * 64 + x] = min(1, 0.2 + 0.001 * (u * u + v * v))
            }
        }
        let pyramid = Pyramid(image: image, levelCount: 1)
        var tracker = KLTTracker()
        tracker.minEigenvalue = 1e-8
        XCTAssertFalse(tracker.track(prev: pyramid, cur: pyramid, x: 32, y: 32).ok)
    }

    func testWarpedPatchMustFitNotJustItsCenter() {
        let texture = makePlanarTexture(seed: 3, repeated: false)
        let previous = Pyramid(
            image: renderPlanarTexture(texture, warp: PlanarImageWarp(angle: 0), occluded: false), levelCount: 1)
        let current = Pyramid(
            image: renderPlanarTexture(texture, warp: PlanarImageWarp(angle: 0, squash: 3, ty: -216), occluded: false),
            levelCount: 1)
        XCTAssertFalse(KLTTracker().track(prev: previous, cur: current, x: 72, y: 228).ok)
    }

    private struct MaterialTrack {
        var x: Float
        var y: Float
        var u: Double
        var v: Double
    }

    private func assertCorrespondence(
        degrees: Double, squash: Double = 1, perspective: Double = 0,
        repeated: Bool = false, occlusion: Bool = false,
        file: StaticString = #filePath, line: UInt = #line
    ) {
        let tracker = KLTTracker()
        var endpointErrors: [Int: [Double]] = [:]
        var retainedCounts: [Int: Int] = [:]
        var visibleErrors: [Double] = []
        var occludedComparisons = 0
        var estimatedPatchInvalidComparisons = 0
        var initialCount = 0
        for seed: UInt64 in [3, 11, 29] {
            let scoredBeforeSeed = visibleErrors.count
            var seedEndpointCounts: [Int: Int] = [:]
            let texture = makePlanarTexture(seed: seed, repeated: repeated)
            func warp(_ frame: Int) -> PlanarImageWarp {
                PlanarImageWarp(
                    angle: Double(frame) * degrees * .pi / 180, squash: squash, perspective: perspective,
                    tx: degrees == 0 ? 8 * sin(Double(frame) * 0.075) : 0,
                    ty: degrees == 0 ? 5 * sin(Double(frame) * 0.06) : 0)
            }
            let first = renderPlanarTexture(texture, warp: warp(0), occluded: false)
            let points = CornerDetector().detect(
                in: first, centerX: 160, centerY: 120, radius: 72, exclude: [], maxCount: 75)
            initialCount += points.count
            var tracks = points.map { x, y in
                let (u, v) = warp(0).unproject(Double(x), Double(y))
                return MaterialTrack(x: x, y: y, u: u, v: v)
            }
            var previous = Pyramid(image: first, levelCount: 4)
            for frame in 1...120 {
                let current = Pyramid(
                    image: renderPlanarTexture(
                        texture, warp: warp(frame), occluded: occlusion && (15...25).contains(frame)),
                    levelCount: 4)
                tracks = tracks.compactMap { point in
                    let result = tracker.track(prev: previous, cur: current, x: point.x, y: point.y)
                    let (previousX, previousY) = warp(frame - 1).project(point.u, point.v)
                    let (x, y) = warp(frame).project(point.u, point.v)
                    let previousCovered = occlusion && (15...25).contains(frame - 1)
                    let currentCovered = occlusion && (15...25).contains(frame)
                    let previousVisible = planarPatchIsVisible(x: previousX, y: previousY, occluded: previousCovered)
                    let currentVisible = planarPatchIsVisible(x: x, y: y, occluded: currentCovered)
                    if !previousVisible || !currentVisible { occludedComparisons += 1 }
                    guard result.ok, result.residual <= 0.12 else { return nil }
                    let supported =
                        planarPatchHasTextureSupport(
                            x: previousX, y: previousY, warp: warp(frame - 1), texture: texture)
                        && planarPatchHasTextureSupport(x: x, y: y, warp: warp(frame), texture: texture)
                    if previousVisible && currentVisible && supported {
                        let estimatedPatchValid =
                            planarPatchIsVisible(x: Double(point.x), y: Double(point.y), occluded: previousCovered)
                            && planarPatchIsVisible(x: Double(result.x), y: Double(result.y), occluded: currentCovered)
                            && planarPatchHasTextureSupport(
                                x: Double(point.x), y: Double(point.y), warp: warp(frame - 1), texture: texture)
                            && planarPatchHasTextureSupport(
                                x: Double(result.x), y: Double(result.y), warp: warp(frame), texture: texture)
                        if !estimatedPatchValid { estimatedPatchInvalidComparisons += 1 }
                        let error = hypot(Double(result.x) - x, Double(result.y) - y)
                        visibleErrors.append(error)
                        if frame == 45 || frame == 120 {
                            endpointErrors[frame, default: []].append(error)
                            seedEndpointCounts[frame, default: 0] += 1
                        }
                    }
                    return MaterialTrack(x: result.x, y: result.y, u: point.u, v: point.v)
                }
                if frame == 45 || frame == 120 {
                    retainedCounts[frame, default: 0] += tracks.count
                    XCTAssertGreaterThanOrEqual(
                        tracks.count, repeated || occlusion ? 20 : 70,
                        "seed \(seed), frame \(frame)", file: file, line: line)
                }
                previous = current
            }
            for frame in [45, 120] {
                XCTAssertGreaterThan(
                    seedEndpointCounts[frame] ?? 0, 0, "seed \(seed), frame \(frame)", file: file, line: line)
            }
            XCTAssertGreaterThan(
                visibleErrors.count - scoredBeforeSeed, points.count * 30,
                "seed \(seed)", file: file, line: line)
        }
        XCTAssertEqual(initialCount, 225, file: file, line: line)
        XCTAssertGreaterThan(visibleErrors.count, initialCount * 30, file: file, line: line)
        XCTAssertLessThan(
            visibleErrors.max() ?? .infinity, 1.5,
            "all truth-visible frames; \(estimatedPatchInvalidComparisons) accepted comparisons with invalid estimated patches",
            file: file, line: line)
        if occlusion { XCTAssertGreaterThan(occludedComparisons, 0, file: file, line: line) }
        for frame in [45, 120] {
            let errors = (endpointErrors[frame] ?? []).sorted()
            let minimumRetained = repeated || occlusion ? 75 : 210
            XCTAssertGreaterThanOrEqual(
                retainedCounts[frame] ?? 0, minimumRetained, "frame \(frame)", file: file, line: line)
            guard !errors.isEmpty else {
                XCTFail("no truth-visible comparisons at frame \(frame)", file: file, line: line)
                continue
            }
            XCTAssertLessThan(
                errors[Int(0.95 * Double(errors.count - 1))], 0.75, "frame \(frame)", file: file, line: line)
            XCTAssertLessThan(errors.last!, 1.5, "frame \(frame)", file: file, line: line)
        }
    }
}
