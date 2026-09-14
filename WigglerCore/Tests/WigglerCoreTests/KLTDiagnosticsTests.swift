import XCTest

@testable import WigglerCore

final class KLTDiagnosticsTests: XCTestCase {
    func testUncomputedResidualIsAbsentInRejectedTrackRoundTrip() throws {
        let pyramid = Pyramid(image: GrayImage(width: 80, height: 80, fill: 0.5), levelCount: 4)
        let result = KLTTracker().track(prev: pyramid, cur: pyramid, x: 40, y: 40)
        XCTAssertFalse(result.ok)
        XCTAssertFalse(result.residual.isFinite)
        let event = EngineDiagnostics.PointEvent(
            id: 7, action: "removed", reason: "kltFailed", kltOK: result.ok,
            residual: result.residual, x: result.x, y: result.y)
        XCTAssertNil(event.residual)
        let data = try JSONEncoder().encode(event)
        let decoded = try JSONDecoder().decode(EngineDiagnostics.PointEvent.self, from: data)
        XCTAssertEqual(decoded.id, 7)
        XCTAssertEqual(decoded.kltOK, false)
        XCTAssertEqual(decoded.reason, "kltFailed")
        XCTAssertNil(decoded.residual)
    }
}
