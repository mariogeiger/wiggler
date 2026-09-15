import SwiftUI

struct RotationDial: View {
    let scale: RotationDialScale
    let value: Double?

    private var tint: Color { scale == .angle ? .orange : .cyan }

    private var accessibleReading: String {
        guard let value, value.isFinite else { return "Unavailable" }
        return scale == .angle
            ? String(format: "%.0f degrees", -value)
            : String(format: "%+.0f revolutions per minute", -value)
    }

    var body: some View {
        let direction = scale.needleDirection(for: value)
        Canvas { context, size in
            let center = CGPoint(x: size.width / 2, y: size.height / 2)
            let radius = min(size.width, size.height) / 2 - 1
            let face = Path(
                ellipseIn: CGRect(
                    x: center.x - radius, y: center.y - radius,
                    width: 2 * radius, height: 2 * radius))
            context.fill(face, with: .color(.black.opacity(0.4)))
            context.stroke(face, with: .color(.white.opacity(0.3)), lineWidth: 1)

            if scale == .rpm {
                var zero = Path()
                zero.move(to: CGPoint(x: center.x, y: center.y - radius))
                zero.addLine(to: CGPoint(x: center.x, y: center.y - radius + 4))
                context.stroke(
                    zero, with: .color(.white.opacity(0.9)), style: StrokeStyle(lineWidth: 2, lineCap: .round))
            }

            guard let direction else { return }
            var needle = Path()
            needle.move(to: center)
            needle.addLine(
                to: CGPoint(
                    x: center.x + direction.x * (radius - 5),
                    y: center.y + direction.y * (radius - 5)))
            context.stroke(needle, with: .color(tint), style: StrokeStyle(lineWidth: 2, lineCap: .round))
            context.fill(
                Path(ellipseIn: CGRect(x: center.x - 2, y: center.y - 2, width: 4, height: 4)),
                with: .color(tint))
        }
        .frame(width: 34, height: 34)
        .accessibilityElement(children: .ignore)
        .accessibilityLabel(scale == .angle ? "Angle" : "Rotation speed")
        .accessibilityValue(accessibleReading)
        .accessibilityHint(
            scale == .angle
                ? "Zero at the top; one full turn is 360 degrees."
                : "Zero at the top; positive clockwise, negative counterclockwise. The needle stops at 100 rpm at the bottom."
        )
        .allowsHitTesting(false)
    }
}
