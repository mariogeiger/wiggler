import Foundation

/// Finds the object to track without a tap: the densest region, the size of the region of interest, of trackable
/// corners that move.
///
/// Texture and motion are exactly what the engine needs, so the criterion is the engine's own — corners KLT can
/// follow, whose speed stays above the jitter of a static scene. Raw image change would not do: a dark chair back
/// sweeping across a bright room changes far more pixels than the small textured box turning on it, while its
/// interior carries no corner at all. The region is the mode of the moving corners under a flat kernel of the
/// region's radius (mean shift from the densest coarse cell), adopted once it has stayed put for a while.
/// While the camera itself moves every corner moves: those frames are discarded.
public struct MotionLocator {
    public var maxPoints = 200
    public var detectEvery = 5
    /// Time constant of a point's speed estimate (s).
    public var speedTau = 0.5
    /// Below this speed a point is static (px/s); KLT jitter on a static scene is a few px/s.
    public var movingSpeed: Float = 6
    /// Moving points needed in the region.
    public var minMoving = 12
    /// The region must stay within half its radius for this long before it is adopted (s).
    public var settleSeconds = 0.7
    /// Camera motion above these rates (rad/s, m/s) makes the frame worthless. Measured over `cameraWindow`
    /// seconds, not between consecutive frames: ARKit's pose jitter (~0.04°, ~0.1 mm per frame) is white noise,
    /// and read as a per-frame rate at 30 fps it alone would exceed these thresholds.
    public var maxCameraRotationRate = 1.0 * Double.pi / 180
    public var maxCameraSpeed = 0.005
    public var cameraWindow = 0.5
    public var border: Float = 8

    private struct Point { var x: Float; var y: Float; var speed: Float }
    private var points: [Point] = []
    private var candidate: (x: Float, y: Float)?
    private var candidateSince = 0.0
    private var frame = 0
    private var poses: [(time: Double, pose: RigidTransform)] = []
    /// Points currently moving (for diagnostics).
    private(set) public var movingCount = 0

    public init() {}

    public mutating func reset() {
        points = []
        candidate = nil
        frame = 0
        poses = []
        movingCount = 0
    }

    /// Feed one frame. Returns the centre of the moving region (pixels) once it is established.
    public mutating func add(image: GrayImage, prev: Pyramid?, cur: Pyramid, pose: RigidTransform?, dt: Double,
                             time: Double, radius: Float, klt: KLTTracker, corners: CornerDetector) -> (x: Float, y: Float)? {
        frame += 1
        let w = Float(image.width), h = Float(image.height)
        var cameraMoved = false
        if let p = pose {
            poses.append((time, p))
            while poses.count > 1, poses[1].time <= time - cameraWindow { poses.removeFirst() }
            let old = poses[0]
            let span = time - old.time
            if span >= cameraWindow / 2 {
                let dR = old.pose.rotation.transposed * p.rotation
                let cosA = max(-1, min(1, (dR.m[0] + dR.m[4] + dR.m[8] - 1) / 2))
                cameraMoved = acos(cosA) > maxCameraRotationRate * span
                    || (p.translation - old.pose.translation).length > maxCameraSpeed * span
            }
        }

        // 1. Track, keeping a smoothed speed per point.
        if let prev = prev, !points.isEmpty, dt > 0 {
            let a = Float(min(1, dt / speedTau))
            var kept: [Point] = []
            kept.reserveCapacity(points.count)
            for var pt in points {
                let r = klt.track(prev: prev, cur: cur, x: pt.x, y: pt.y)
                guard r.ok, r.x > border, r.y > border, r.x < w - border, r.y < h - border else { continue }
                let dx = r.x - pt.x, dy = r.y - pt.y
                let v = (dx * dx + dy * dy).squareRoot() / Float(dt)
                pt.speed = cameraMoved ? 0 : pt.speed + a * (v - pt.speed)
                pt.x = r.x; pt.y = r.y
                kept.append(pt)
            }
            points = kept
        }
        if cameraMoved { candidate = nil }

        // 2. Replenish corners over the whole image.
        if frame % detectEvery == 1 && points.count < maxPoints {
            let diagonal = (w * w + h * h).squareRoot()
            let fresh = corners.detect(in: image, centerX: w / 2, centerY: h / 2, radius: diagonal / 2,
                                       exclude: points.map { ($0.x, $0.y) }, maxCount: maxPoints - points.count)
            for (x, y) in fresh { points.append(Point(x: x, y: y, speed: 0)) }
        }

        // 3. Mode of the moving points under a flat kernel of the region's radius.
        let moving = points.filter { $0.speed > movingSpeed }
        movingCount = moving.count
        guard moving.count >= minMoving, let centre = Self.mode(of: moving.map { ($0.x, $0.y) }, radius: radius, minCount: minMoving) else {
            candidate = nil
            return nil
        }
        if let c = candidate, (c.x - centre.x) * (c.x - centre.x) + (c.y - centre.y) * (c.y - centre.y) < radius * radius / 4 {
            return time - candidateSince >= settleSeconds ? centre : nil
        }
        candidate = centre
        candidateSince = time
        return nil
    }

    /// Densest coarse cell, then mean shift with a flat kernel; nil when the densest disc is too sparse.
    static func mode(of pts: [(Float, Float)], radius: Float, minCount: Int) -> (x: Float, y: Float)? {
        let cell = radius / 2
        var counts: [Int: Int] = [:]
        for (x, y) in pts { counts[Int(x / cell) &* 4096 &+ Int(y / cell), default: 0] += 1 }
        guard let best = counts.max(by: { $0.value < $1.value })?.key else { return nil }
        var cx: Float = 0, cy: Float = 0, n: Float = 0
        for (x, y) in pts where Int(x / cell) &* 4096 &+ Int(y / cell) == best { cx += x; cy += y; n += 1 }
        cx /= n; cy /= n
        var inside = 0
        for _ in 0..<4 {
            var sx: Float = 0, sy: Float = 0
            inside = 0
            for (x, y) in pts where (x - cx) * (x - cx) + (y - cy) * (y - cy) < radius * radius { sx += x; sy += y; inside += 1 }
            if inside == 0 { return nil }
            cx = sx / Float(inside); cy = sy / Float(inside)
        }
        return inside >= minCount ? (cx, cy) : nil
    }
}
