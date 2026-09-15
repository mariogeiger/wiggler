import Foundation

private var checks = 0

private func check(_ scale: RotationDialScale, _ value: Double, x: Double, y: Double) {
    guard let direction = scale.needleDirection(for: value) else {
        fatalError("Missing needle for \(scale), \(value)")
    }
    precondition(abs(direction.x - x) < 1e-12, "Wrong x for \(scale), \(value): \(direction)")
    precondition(abs(direction.y - y) < 1e-12, "Wrong y for \(scale), \(value): \(direction)")
    checks += 1
}

for turn in -3...3 {
    let offset = Double(turn) * 360
    check(.angle, offset, x: 0, y: -1)
    check(.angle, offset + 90, x: -1, y: 0)
    check(.angle, offset + 180, x: 0, y: 1)
    check(.angle, offset + 270, x: 1, y: 0)
}
check(.rpm, 0, x: 0, y: -1)
check(.rpm, 50, x: -1, y: 0)
check(.rpm, -50, x: 1, y: 0)
check(.rpm, 100, x: 0, y: 1)
check(.rpm, -100, x: 0, y: 1)
for speed in [101.0, 200, 1_000, Double.greatestFiniteMagnitude] {
    check(.rpm, speed, x: 0, y: 1)
    check(.rpm, -speed, x: 0, y: 1)
}

for scale in [RotationDialScale.angle, .rpm] {
    precondition(scale.needleDirection(for: nil) == nil, "Unavailable reading must not appear as zero")
    checks += 1
    for value in [Double.nan, .infinity, -.infinity] {
        precondition(scale.needleDirection(for: value) == nil, "Non-finite reading must not appear as zero")
        checks += 1
    }
    for value in stride(from: -100.0, through: 100, by: 0.25) {
        let positive = scale.needleDirection(for: value)!
        let negative = scale.needleDirection(for: -value)!
        precondition(abs(positive.x * positive.x + positive.y * positive.y - 1) < 1e-12)
        precondition(abs(positive.x + negative.x) < 1e-12)
        precondition(abs(positive.y - negative.y) < 1e-12)
        checks += 1
    }
}
let beforeWrap = RotationDialScale.angle.needleDirection(for: 359.9)!
let afterWrap = RotationDialScale.angle.needleDirection(for: 0.1)!
precondition(hypot(beforeWrap.x - afterWrap.x, beforeWrap.y - afterWrap.y) < 0.004)
print("Passed \(checks + 1) dial checks")
