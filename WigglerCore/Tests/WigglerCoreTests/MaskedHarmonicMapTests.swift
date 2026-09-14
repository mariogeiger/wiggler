import XCTest

@testable import WigglerCore

final class MaskedHarmonicMapTests: XCTestCase {
    func testMissingSamplesDoNotBecomeZerosOrPolluteOtherPixels() {
        var map = HarmonicMap(width: 3, height: 2, orders: [1, 2, 3])
        var theta = 0.0
        func value(pixel: Int, theta: Double) -> Float {
            var value = 1.2 + 0.003 * theta
            for order in 1...3 {
                let amplitude = 0.02 * Double(pixel + 1) / Double(order)
                value += amplitude * cos(Double(order) * theta - 0.3 * Double(pixel))
            }
            return Float(value)
        }
        for frame in 0..<1000 {
            theta += frame < 600 ? 0.043 : -0.051
            var pixels = (0..<6).map { value(pixel: $0, theta: theta) }
            if frame % 5 == 0 { pixels[1] = .nan }
            if frame > 150 && frame % 2 == 0 { pixels[2] = .nan }
            pixels[3] = .nan
            if frame < 950 { pixels[4] = .nan }
            if frame % 4 == 0 { pixels[5] = .infinity }
            map.add(image: GrayImage(width: 3, height: 2, pixels: pixels), theta: theta)
        }
        XCTAssertTrue(map.hasFullTurn)
        for order in 1...3 {
            guard let coefficients = map.coefficients(order: order) else { return XCTFail("missing coefficients") }
            for pixel in [0, 1, 2, 5] {
                let amplitude = 0.02 * Double(pixel + 1) / Double(order)
                XCTAssertEqual(Double(coefficients.a[pixel]), amplitude * cos(0.3 * Double(pixel)), accuracy: 0.0005)
                XCTAssertEqual(Double(coefficients.b[pixel]), amplitude * sin(0.3 * Double(pixel)), accuracy: 0.0005)
            }
            for pixel in [3, 4] {
                XCTAssertTrue(coefficients.a[pixel].isNaN)
                XCTAssertTrue(coefficients.b[pixel].isNaN)
            }
            let rgba = map.render(order: order, theta: theta, fullScale: 0.02)!
            XCTAssertEqual(rgba.count, 24)
            XCTAssertEqual(rgba[4 * 3 + 3], 0)
            XCTAssertEqual(rgba[4 * 4 + 3], 0)
        }
    }

    func testDenseHistorySurvivesFirstMissingSample() {
        var map = HarmonicMap(width: 2, height: 1)
        for frame in 0...200 {
            let theta = Double(frame) * 0.05
            let value = Float(0.7 + 0.1 * cos(theta))
            map.add(image: GrayImage(width: 2, height: 1, pixels: [value, value]), theta: theta)
        }
        XCTAssertTrue(map.hasFullTurn)
        map.add(image: GrayImage(width: 2, height: 1, pixels: [.nan, 0.7 + 0.1 * cos(10.05)]), theta: 10.05)
        guard let coefficients = map.coefficients(order: 1) else { return XCTFail("promotion lost the fit") }
        for pixel in 0..<2 {
            XCTAssertEqual(coefficients.a[pixel], 0.1, accuracy: 0.0001)
            XCTAssertEqual(coefficients.b[pixel], 0, accuracy: 0.0001)
        }
        map.reset()
        XCTAssertEqual(map.turnProgress, 0)
        XCTAssertNil(map.render(order: 1, theta: 0, fullScale: 0.1))
    }

    func testAllMissingAndRankDeficientFitsStayTransparent() {
        var map = HarmonicMap(width: 1, height: 1)
        for frame in 0..<300 {
            map.add(image: GrayImage(width: 1, height: 1, fill: .nan), theta: Double(frame) * 0.05)
        }
        XCTAssertEqual(map.fill, 0)
        XCTAssertFalse(map.hasFullTurn)
        XCTAssertNil(map.coefficients(order: 1))

        var moments = HarmonicMoments(terms: 4, pixelCount: 1)
        for _ in 0..<20 { moments.add(basis: [1, 0, 1, 0], delta: 0, decay: 1, weight: 1, valid: nil) }
        XCTAssertTrue(moments.project(rhs: [10, 0, 10, 0], weights: [0, 0, 1, 0], minimumWeight: 6)[0].isNaN)
    }
}
