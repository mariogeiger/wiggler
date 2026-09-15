import Foundation

/// Tracked image points with the world position depth gives them, and which of them belong to a body that moves
/// against the static scene. A point is moving when its own earlier world position, reprojected into the current
/// frame, misses where the point is seen now: the test compares a point with itself over as long a baseline as it
/// has, so it also notices slow rotation, and it asks nothing of the camera beyond a usable pose.
final class TrackedWorldPoints {
    struct Observation {
        var id: Int
        var x: Double
        var y: Double
        var world: V3
        var range: Double
        var moving: Bool
        /// Pixels between where the point is seen and where its earliest retained world position reprojects.
        var displacement: Double
    }

    private final class History {
        var samples: [(time: Double, point: V3)] = []
        var lastDepth = 0.0
        var lastChordFrame = -1000
    }

    private var histories: [Int: History] = [:]
    private(set) var observations: [Observation] = []

    /// How long a point's world positions are kept, seconds: the longest baseline the moving test may use.
    var historySeconds = 1.0
    /// Positions kept per point whatever the frame rate, so memory is bounded by the point count alone. The oldest
    /// and the newest are always kept, and a full window is thinned to every other sample.
    var maxSamples = 64
    var movingPixels = 3.0
    /// A depth that jumps by more than this fraction between frames comes from another surface: the point's
    /// baseline restarts rather than fabricating a chord across a depth edge.
    var maxDepthJumpFraction = 0.08
    /// Frames between two chords of the same point, so one point cannot flood the axis estimate.
    var chordStride = 3

    var movingCount: Int { observations.reduce(0) { $0 + ($1.moving ? 1 : 0) } }
    var movingIDs: Set<Int> { Set(observations.filter { $0.moving }.map { $0.id }) }
    /// Points already separating from where their earlier position reprojects, moving or not yet. A body turning
    /// slowly needs a longer baseline than a point's ordinary lifetime allows, so this is who is worth keeping while
    /// the evidence accumulates. A point of the static scene stays at zero and is not in it.
    var driftingIDs: Set<Int> {
        Set(observations.filter { $0.displacement > movingPixels / 3 }.map { $0.id })
    }

    func removeAll() {
        histories.removeAll()
        observations.removeAll()
    }

    /// No usable pose this frame: nothing can be placed in the world, so the frame yields no observations. The
    /// positions taken while the pose was good stay, and stay valid, because each was placed with the pose of its
    /// own moment.
    func skipFrame() {
        observations.removeAll(keepingCapacity: true)
    }

    /// Sample depth for the tracked points, retain their recent world positions, and mark the moving ones.
    func observe(
        _ points: [MotionLocator.Point], depth: DepthMap?, view: PinholeProjection, frame: Int, time: Double,
        minConfidence: UInt8
    ) {
        observations.removeAll(keepingCapacity: true)
        var live = Set<Int>()
        live.reserveCapacity(points.count)
        for point in points {
            live.insert(point.id)
            let history = histories[point.id] ?? History()
            histories[point.id] = history
            let x = Double(point.x), y = Double(point.y)
            guard let depth,
                let z = depth.sample(u: x / view.width, v: y / view.height, minConfidence: minConfidence)
            else { continue }
            if history.lastDepth > 0, abs(z - history.lastDepth) > maxDepthJumpFraction * history.lastDepth {
                history.lastDepth = z
                history.samples.removeAll()
                continue
            }
            history.lastDepth = z
            let world = view.unproject(x: x, y: y, range: z)
            guard world.isFinite else { continue }
            if let last = history.samples.last, time < last.time { history.samples.removeAll() }
            history.samples.removeAll { time - $0.time > historySeconds }
            if history.samples.count >= maxSamples {
                var thinned = history.samples.enumerated().filter { $0.offset % 2 == 0 }.map { $0.element }
                if let last = history.samples.last, thinned.last?.time != last.time { thinned.append(last) }
                history.samples = thinned
            }
            var displacement = 0.0
            if let first = history.samples.first, let projected = view.project(first.point) {
                let dx = projected.x - x, dy = projected.y - y
                displacement = (dx * dx + dy * dy).squareRoot()
            }
            history.samples.append((time, world))
            observations.append(
                Observation(
                    id: point.id, x: x, y: y, world: world, range: z, moving: displacement > movingPixels,
                    displacement: displacement))
        }
        histories = histories.filter { live.contains($0.key) }
    }

    /// Chords of the moving points: for each, the oldest retained position far enough from the newest one. A
    /// chord is perpendicular to the axis and its perpendicular bisector meets the axis, whatever angle the body
    /// turned through between the two positions.
    func chordConstraints(minLength: Double, frame: Int) -> [ChordConstraint] {
        var constraints: [ChordConstraint] = []
        for observation in observations where observation.moving {
            guard let history = histories[observation.id], frame - history.lastChordFrame >= chordStride,
                let last = history.samples.last
            else { continue }
            for sample in history.samples {
                let chord = last.point - sample.point
                if chord.length >= minLength {
                    constraints.append(
                        ChordConstraint(midpoint: (last.point + sample.point) * 0.5, chord: chord, frame: frame))
                    history.lastChordFrame = frame
                    break
                }
            }
        }
        return constraints
    }

    /// How well a point's retained positions fit a rigid rotation about an axis: how much its distance to the axis
    /// and its height along it drift, and how far it travelled around it. A part of the turning body keeps both
    /// fixed while it travels; a translating hand, or a point of another body, does not.
    func axisConsistency(id: Int, axis: Axis) -> (
        radiusDrift: Double, heightDrift: Double, travel: Double, radius: Double, samples: Int
    )? {
        guard let history = histories[id], history.samples.count >= 3 else { return nil }
        var radii: [Double] = [], heights: [Double] = []
        var travel = 0.0
        var previousPhi: Double?
        for sample in history.samples {
            let (phi, radius, height) = axis.cylindrical(sample.point)
            radii.append(radius)
            heights.append(height)
            if let previous = previousPhi { travel += wrapAngle(phi - previous) }
            previousPhi = phi
        }
        let count = Double(radii.count)
        let meanRadius = radii.reduce(0, +) / count, meanHeight = heights.reduce(0, +) / count
        let radiusDrift = (radii.reduce(0) { $0 + ($1 - meanRadius) * ($1 - meanRadius) } / count).squareRoot()
        let heightDrift = (heights.reduce(0) { $0 + ($1 - meanHeight) * ($1 - meanHeight) } / count).squareRoot()
        return (radiusDrift, heightDrift, travel, meanRadius, radii.count)
    }

    /// The root-mean-square distance by which a point's retained positions disagree about where it sits on the body,
    /// in metres, and how far it
    /// travelled about the axis meanwhile. Each position is taken back by the angle the body had at that moment, and
    /// a rigid point of the body maps them all to one place: that is the whole rigid condition, so a tangential
    /// slide, a different rate, or another body all fail it. Nil until the angle is known over enough of the
    /// point's own history.
    func rigidResidual(id: Int, axis: Axis, angleAt: (Double) -> Double?) -> (
        spread: Double, worldSpread: Double, travel: Double, span: Double, samples: Int
    )? {
        guard let history = histories[id] else { return nil }
        var canonical: [V3] = []
        var world: [V3] = []
        var travel = 0.0
        var previousPhi: Double?
        var first: Double?
        var last = 0.0
        for sample in history.samples {
            guard let theta = angleAt(sample.time) else { continue }
            canonical.append(rotatedAbout(axis.direction, by: -theta, sample.point - axis.origin))
            world.append(sample.point)
            let phi = axis.cylindrical(sample.point).phi
            if let previous = previousPhi { travel += wrapAngle(phi - previous) }
            previousPhi = phi
            if first == nil { first = sample.time }
            last = sample.time
        }
        guard canonical.count >= 3, let first else { return nil }
        return (
            Self.deviation(canonical), Self.deviation(world), travel, last - first, canonical.count
        )
    }

    /// Root-mean-square distance of points from their own mean.
    private static func deviation(_ points: [V3]) -> Double {
        guard !points.isEmpty else { return 0 }
        var mean = V3.zero
        for point in points { mean += point }
        mean = mean / Double(points.count)
        var sum = 0.0
        for point in points { sum += (point - mean).lengthSquared }
        return (sum / Double(points.count)).squareRoot()
    }

    /// Median signed rate at which the named points travel about a candidate axis, radians per second: which way
    /// the body turns and how fast, measured over each point's own baseline. Beyond half a turn per baseline the
    /// rate wraps, so it names the sense reliably and the magnitude only for a body slower than that.
    func rotationRate(origin: V3, direction: V3, among ids: Set<Int>) -> Double {
        var rates: [Double] = []
        for observation in observations where ids.contains(observation.id) {
            guard let history = histories[observation.id], let first = history.samples.first,
                let last = history.samples.last, last.time - first.time > 1e-3
            else { continue }
            let a = first.point - origin, b = last.point - origin
            let ap = a - direction * a.dot(direction), bp = b - direction * b.dot(direction)
            guard ap.length > 0.005, bp.length > 0.005 else { continue }
            rates.append(atan2(ap.cross(bp).dot(direction), ap.dot(bp)) / (last.time - first.time))
        }
        return rates.isEmpty ? 0 : median(rates)
    }
}
