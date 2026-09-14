import XCTest

@testable import WigglerCore

final class HarmonicSignalTests: XCTestCase {
    private func frame(width: Int = 4, height: Int = 3, depth: DepthMap? = nil) -> FrameInput {
        FrameInput(
            image: GrayImage(width: width, height: height, fill: 0.5),
            intrinsics: CameraIntrinsics(fx: 1, fy: 1, cx: 0, cy: 0), cameraToWorld: .identity,
            poseValid: true, depth: depth, timestamp: 0,
            chromaRed: GrayImage(width: width, height: height, fill: 0.1),
            chromaBlue: GrayImage(width: width, height: height, fill: -0.2))
    }

    private func stable(theta: Double) -> EngineOutput {
        var output = EngineOutput()
        output.state = .locked
        output.axisStable = true
        output.theta = theta
        return output
    }

    func testFourSignalsAndSavedSelection() {
        XCTAssertEqual(HarmonicSignal.allCases, [.luma, .chromaRed, .chromaBlue, .depth])
        XCTAssertEqual(HarmonicFit().signal, .luma)
        XCTAssertEqual(HarmonicSignal(savedValue: nil), .luma)
        XCTAssertEqual(HarmonicSignal(savedValue: "unknown"), .luma)
        for signal in HarmonicSignal.allCases {
            XCTAssertEqual(HarmonicSignal(savedValue: signal.rawValue), signal)
        }
        let input = frame()
        XCTAssertEqual(HarmonicSignal.luma.image(in: input)?.pixels.first, 0.5)
        XCTAssertEqual(HarmonicSignal.chromaRed.image(in: input)?.pixels.first, 0.1)
        XCTAssertEqual(HarmonicSignal.chromaBlue.image(in: input)?.pixels.first, -0.2)
        XCTAssertNil(HarmonicSignal.depth.image(in: input))
        XCTAssertEqual(HarmonicSignal.depth.fullScale, 0.02)
    }

    func testChromaOrderNeutralValueRangeAndRowStride() {
        let bytes: [UInt8] = [128, 128, 240, 16, 99, 98, 16, 240, 128, 128, 97, 96]
        for videoRange in [false, true] {
            for component in [ChromaComponent.blue, .red] {
                let image = bytes.withUnsafeBufferPointer {
                    GrayImage(
                        width: 2, height: 2, cbcr8: $0.baseAddress!, bytesPerRow: 6,
                        component: component, videoRange: videoRange)
                }
                let amplitude: Float = 112 / (videoRange ? 224 : 255) * (component == .blue ? 1 : -1)
                XCTAssertEqual(image.width, 2)
                XCTAssertEqual(image.height, 2)
                for (actual, expected) in zip(image.pixels, [0, amplitude, -amplitude, 0]) {
                    XCTAssertEqual(actual, expected, accuracy: 1e-7)
                }
            }
        }
    }

    func testNativeDepthGridAndInvalidConfidenceMask() {
        var depth = DepthMap(
            width: 4, height: 2, depth: [.nan, .infinity, 0, -1, 0.05, 1.2, 1.3, 1.4],
            confidence: [2, 2, 2, 2, 2, 0, 1, 2])
        let image = HarmonicSignal.depth.image(in: frame(depth: depth))!
        XCTAssertEqual(image.width, 4)
        XCTAssertEqual(image.height, 2)
        XCTAssertTrue(image.pixels.prefix(6).allSatisfy(\.isNaN))
        XCTAssertEqual(Array(image.pixels.suffix(2)), [1.3, 1.4])
        depth.confidence = nil
        let withoutConfidence = HarmonicSignal.depth.image(in: frame(depth: depth))!
        XCTAssertTrue(withoutConfidence.pixels.prefix(5).allSatisfy(\.isNaN))
        XCTAssertEqual(withoutConfidence.pixels[5], 1.2)
    }

    func testSignalAndGridChangesRequireNewHistory() {
        var fit = HarmonicFit()
        for i in 0...160 { fit.update(frame: frame(), output: stable(theta: Double(i) * 0.05)) }
        XCTAssertEqual(fit.turnProgress, 1)
        fit.select(.luma)
        XCTAssertEqual(fit.turnProgress, 1)
        fit.select(.chromaRed)
        XCTAssertNil(fit.map)
        XCTAssertEqual(fit.turnProgress, 0)
        fit.update(frame: frame(), output: stable(theta: 8.05))
        XCTAssertEqual(fit.turnProgress, 0)
        fit.update(frame: frame(), output: stable(theta: 8.1))
        XCTAssertGreaterThan(fit.turnProgress, 0)
        fit.update(frame: frame(width: 2, height: 2), output: stable(theta: 8.15))
        XCTAssertEqual(fit.map?.width, 2)
        XCTAssertEqual(fit.map?.height, 2)
        XCTAssertEqual(fit.turnProgress, 0)
        fit.select(.luma)
        XCTAssertNil(fit.map)
        XCTAssertEqual(fit.turnProgress, 0)
    }

    func testMissingInputDecaysFitButUnstableAxisDiscardsIt() {
        var fit = HarmonicFit(signal: .chromaBlue)
        fit.update(frame: frame(), output: stable(theta: 0))
        fit.update(frame: frame(), output: stable(theta: 0.1))
        XCTAssertGreaterThan(fit.turnProgress, 0)
        let progress = fit.turnProgress
        var missing = frame()
        missing.chromaBlue = nil
        fit.update(frame: missing, output: stable(theta: 0.2))
        XCTAssertNotNil(fit.map)
        XCTAssertEqual(fit.turnProgress, progress * exp(-0.1 / fit.map!.window), accuracy: 1e-12)
        fit.update(frame: nil, output: stable(theta: 0.25))
        XCTAssertEqual(fit.turnProgress, progress * exp(-0.15 / fit.map!.window), accuracy: 1e-12)
        fit.update(frame: frame(), output: stable(theta: 0.3))
        XCTAssertGreaterThan(fit.turnProgress, progress)
        var uncertain = stable(theta: 0.4)
        uncertain.axisStable = false
        fit.update(frame: nil, output: uncertain)
        XCTAssertNil(fit.map)
        fit.select(.depth)
        fit.update(frame: frame(), output: stable(theta: 0.5))
        XCTAssertNil(fit.map)
    }

    func testMetricDepthHarmonicWithMissingMeasurements() {
        var fit = HarmonicFit(signal: .depth)
        for i in 0..<600 {
            let theta = Double(i) * 0.05
            let value = Float(1.2 + 0.01 * cos(theta) - 0.02 * sin(theta))
            let depth = DepthMap(width: 2, height: 1, depth: [value, value], confidence: [i % 3 == 0 ? 0 : 2, 0])
            fit.update(frame: frame(depth: i % 11 == 0 ? nil : depth), output: stable(theta: theta))
        }
        XCTAssertEqual(fit.map?.width, 2)
        XCTAssertEqual(fit.map?.height, 1)
        let coefficients = fit.map?.coefficients(order: 1)
        XCTAssertNotNil(coefficients)
        XCTAssertEqual(coefficients!.a[0], 0.01, accuracy: 0.0001)
        XCTAssertEqual(coefficients!.b[0], -0.02, accuracy: 0.0001)
        XCTAssertTrue(coefficients!.a[1].isNaN)
        XCTAssertTrue(coefficients!.b[1].isNaN)
        fit.update(frame: frame(), output: stable(theta: 30))
        XCTAssertEqual(fit.turnProgress, 1)
        XCTAssertEqual(fit.map!.coefficients(order: 1)!.a[0], 0.01, accuracy: 0.0001)
        XCTAssertEqual(fit.map!.coefficients(order: 1)!.b[0], -0.02, accuracy: 0.0001)
    }
}
