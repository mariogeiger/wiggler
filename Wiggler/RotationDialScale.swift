import Foundation

/// Clock-face coordinates: zero at noon, positive values clockwise.
enum RotationDialScale {
    case angle
    case rpm

    func needleDirection(for value: Double) -> CGPoint? {
        guard value.isFinite else { return nil }
        let radians: Double
        switch self {
        case .angle:
            radians = value.truncatingRemainder(dividingBy: 360) * (.pi / 180)
        case .rpm:
            // Saturate at six o'clock rather than wrap a high speed back to zero.
            radians = min(100, max(-100, value)) * (.pi / 100)
        }
        return CGPoint(x: sin(radians), y: -cos(radians))
    }
}
