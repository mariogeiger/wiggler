import Foundation

/// Tracks image corners once for both motion-based selection and the rotation measurement.
/// A rolling, spatially distributed search keeps exploring even when the current points are static.
public struct MotionLocator {
    public var maxPoints = 240
    public var detectEvery = 5
    public var speedTau = 0.25
    public var movingSpeed: Float = 6
    public var minMoving = 12
    public var settleSeconds = 0.7
    public var maxCameraRotationRate = 1.0 * Double.pi / 180
    public var maxCameraSpeed = 0.005
    public var cameraWindow = 0.5
    public var border: Float = 8
    public var maxResidual: Float = 0.12

    struct Point {
        let id: Int
        var x: Float, y: Float
        var previousX: Float, previousY: Float
        var speed: Float = 0
        var duration = 0.0
        var stillFor = 0.0
    }

    var diagnostics: EngineDiagnosticsRecorder?
    private(set) var points: [Point] = []
    private(set) var motionValid = false
    private var nextID = 1
    private var candidate: (x: Float, y: Float)?
    private var candidateSince = 0.0
    private var frame = 0
    private var tile = 0
    private var poses: [(time: Double, pose: RigidTransform)] = []
    private(set) public var movingCount = 0

    public init() {}

    public mutating func reset() {
        points = []
        candidate = nil
        frame = 0
        tile = 0
        poses = []
        movingCount = 0
        motionValid = false
    }

    /// Returns an established moving region. `retaining` protects measurement tracks during a pause;
    /// unselected static probes expire so background texture cannot exhaust the search budget.
    public mutating func add(
        image: GrayImage, prev: Pyramid?, cur: Pyramid, pose: RigidTransform?, dt: Double,
        time: Double, radius: Float, klt: KLTTracker, corners: CornerDetector,
        retaining: Set<Int> = []
    ) -> (x: Float, y: Float)? {
        frame += 1
        let w = Float(image.width), h = Float(image.height)
        diagnostics?.value.cameraMotion = .init(
            poseValid: pose != nil, dt: dt, windowSeconds: cameraWindow,
            maxRotationRate: maxCameraRotationRate, maxTranslationSpeed: maxCameraSpeed)
        if pose == nil { diagnostics?.value.cameraMotion?.vetoes.append("invalidPose") }
        if !(dt > 0 && dt < 0.5) { diagnostics?.value.cameraMotion?.vetoes.append("invalidFrameInterval") }
        motionValid = pose != nil && dt > 0 && dt < 0.5
        if dt <= 0 || dt >= 0.5 { poses.removeAll() }
        if let p = pose {
            poses.append((time, p))
            while poses.count > 1, poses[1].time <= time - cameraWindow { poses.removeFirst() }
            let old = poses[0]
            let span = time - old.time
            diagnostics?.value.cameraMotion?.span = span
            if span >= cameraWindow / 2 {
                let dR = old.pose.rotation.transposed * p.rotation
                let cosA = max(-1, min(1, (dR.m[0] + dR.m[4] + dR.m[8] - 1) / 2))
                if let diagnostics {
                    let rotation = acos(cosA)
                    let translation = (p.translation - old.pose.translation).length
                    diagnostics.value.cameraMotion?.rotationRadians = rotation
                    diagnostics.value.cameraMotion?.translationMeters = translation
                    diagnostics.value.cameraMotion?.rotationLimit = maxCameraRotationRate * span
                    diagnostics.value.cameraMotion?.translationLimit = maxCameraSpeed * span
                    if !(rotation <= maxCameraRotationRate * span) {
                        diagnostics.value.cameraMotion?.vetoes.append("cameraRotation")
                    }
                    if !(translation <= maxCameraSpeed * span) {
                        diagnostics.value.cameraMotion?.vetoes.append("cameraTranslation")
                    }
                }
                motionValid =
                    motionValid && acos(cosA) <= maxCameraRotationRate * span
                    && (p.translation - old.pose.translation).length <= maxCameraSpeed * span
            } else {
                diagnostics?.value.cameraMotion?.vetoes.append("insufficientPoseHistory")
                motionValid = false
            }
        } else {
            poses.removeAll()
        }

        diagnostics?.value.cameraMotion?.motionValid = motionValid

        if let prev, prev.levels[0].width == image.width, prev.levels[0].height == image.height {
            let a = Float(1 - exp(-max(0, dt) / speedTau))
            points = points.compactMap { pt in
                var pt = pt
                let r = klt.track(prev: prev, cur: cur, x: pt.x, y: pt.y)
                guard r.ok, r.residual <= maxResidual, r.x > border, r.y > border,
                    r.x < w - border, r.y < h - border
                else {
                    diagnostics?.value.pointEvents.append(
                        .init(
                            id: pt.id, action: "removed",
                            reason: !r.ok
                                ? "kltFailed" : (!(r.residual <= maxResidual) ? "kltResidual" : "imageBorder"),
                            kltOK: r.ok, residual: r.residual, x: r.x, y: r.y))
                    return nil
                }
                diagnostics?.value.pointEvents.append(
                    .init(
                        id: pt.id, action: "tracked", reason: "kltAccepted", kltOK: r.ok,
                        residual: r.residual, x: r.x, y: r.y))
                let dx = r.x - pt.x, dy = r.y - pt.y
                pt.previousX = pt.x
                pt.previousY = pt.y
                pt.x = r.x
                pt.y = r.y
                pt.duration += max(0, dt)
                if motionValid {
                    let v = (dx * dx + dy * dy).squareRoot() / Float(dt)
                    pt.speed += a * (v - pt.speed)
                    pt.stillFor = pt.speed <= movingSpeed ? pt.stillFor + dt : 0
                } else {
                    pt.speed = 0
                    pt.stillFor = 0
                }
                return pt
            }
        } else {
            if let diagnostics {
                for point in points {
                    diagnostics.value.pointEvents.append(
                        .init(
                            id: point.id, action: "removed", reason: "missingOrResizedPreviousImage", x: point.x,
                            y: point.y))
                }
            }
            points.removeAll()
            candidate = nil
        }

        if points.count > maxPoints {
            points.sort {
                if retaining.contains($0.id) != retaining.contains($1.id) { return retaining.contains($0.id) }
                return $0.speed != $1.speed ? $0.speed > $1.speed : $0.id < $1.id
            }
            if let diagnostics {
                for point in points.dropFirst(max(0, maxPoints)) {
                    diagnostics.value.pointEvents.append(
                        .init(
                            id: point.id, action: "removed", reason: "pointCapacity", x: point.x, y: point.y))
                }
            }
            points = Array(points.prefix(max(0, maxPoints)))
        }
        if (frame - 1) % max(1, detectEvery) == 0 || points.isEmpty {
            if let diagnostics {
                for point in points
                where !retaining.contains(point.id) && (point.stillFor >= 0.75 || point.duration >= 1.5) {
                    diagnostics.value.pointEvents.append(
                        .init(
                            id: point.id, action: "removed", reason: "unselectedProbeExpired", x: point.x, y: point.y))
                }
            }
            points.removeAll { !retaining.contains($0.id) && ($0.stillFor >= 0.75 || $0.duration >= 1.5) }
            replenish(image: image, corners: corners)
        }

        let moving = points.filter { $0.speed > movingSpeed && $0.duration >= 0.1 }
        movingCount = motionValid ? moving.count : 0
        guard motionValid, moving.count >= minMoving,
            let centre = Self.mode(of: moving.map { ($0.x, $0.y) }, radius: radius, minCount: minMoving)
        else {
            candidate = nil
            return nil
        }
        if let c = candidate,
            (c.x - centre.x) * (c.x - centre.x) + (c.y - centre.y) * (c.y - centre.y) < radius * radius / 4
        {
            return time - candidateSince >= settleSeconds ? centre : nil
        }
        candidate = centre
        candidateSince = time
        return nil
    }

    /// Existing moving tracks keep their history. Vacancies and static tracks go to the fastest measured
    /// corners in the region. When everything stops, the existing tracks remain; no static corner is activated.
    func selected(around marker: Marker, retaining: Set<Int>, limit: Int) -> [Point] {
        let r2 = marker.radius * marker.radius * 1.3 * 1.3
        let nearby = points.filter {
            let dx = $0.x - marker.x, dy = $0.y - marker.y
            return dx * dx + dy * dy <= r2
        }
        func moving(_ p: Point) -> Bool { motionValid && p.speed > movingSpeed && p.duration >= 0.1 }
        func rank(_ p: Point) -> Int {
            if retaining.contains(p.id) && (!motionValid || p.stillFor < 0.5) { return 2 }
            return moving(p) ? 1 : 0
        }
        let hasMotion = nearby.filter { moving($0) }.count >= minMoving
        return Array(
            nearby.filter {
                moving($0) || (retaining.contains($0.id) && (!hasMotion || $0.stillFor < 0.5))
            }.sorted {
                let a = rank($0), b = rank($1)
                if a != b { return a > b }
                if $0.speed != $1.speed { return $0.speed > $1.speed }
                return $0.id < $1.id
            }.prefix(max(0, limit)))
    }

    private mutating func replenish(image: GrayImage, corners: CornerDetector) {
        let columns = 4, rows = 3
        let tw = Float(image.width) / Float(columns), th = Float(image.height) / Float(rows)
        let radius = (tw * tw + th * th).squareRoot() / 2
        let quota = max(12, maxPoints / (columns * rows))
        for _ in 0..<3 {
            let x = (Float(tile % columns) + 0.5) * tw
            let y = (Float(tile / columns) + 0.5) * th
            tile = (tile + 1) % (columns * rows)
            let count = min(quota, maxPoints - points.count)
            guard count > 0 else { continue }
            let fresh = corners.detect(
                in: image, centerX: x, centerY: y, radius: radius,
                exclude: points.map { ($0.x, $0.y) }, maxCount: count)
            for (x, y) in fresh {
                points.append(Point(id: nextID, x: x, y: y, previousX: x, previousY: y))
                diagnostics?.value.pointEvents.append(
                    .init(
                        id: nextID, action: "added", reason: "cornerDetected", x: x, y: y))
                nextID += 1
            }
        }
    }

    /// Densest coarse cell, then mean shift with a flat kernel; nil when the densest disc is too sparse.
    static func mode(of pts: [(Float, Float)], radius: Float, minCount: Int) -> (x: Float, y: Float)? {
        let cell = radius / 2
        var counts: [Int: Int] = [:]
        for (x, y) in pts { counts[Int(x / cell) &* 4096 &+ Int(y / cell), default: 0] += 1 }
        guard let best = counts.keys.sorted().max(by: { counts[$0]! < counts[$1]! }) else { return nil }
        var cx: Float = 0, cy: Float = 0, n: Float = 0
        for (x, y) in pts where Int(x / cell) &* 4096 &+ Int(y / cell) == best {
            cx += x
            cy += y
            n += 1
        }
        cx /= n
        cy /= n
        var inside = 0
        for _ in 0..<4 {
            var sx: Float = 0, sy: Float = 0
            inside = 0
            for (x, y) in pts where (x - cx) * (x - cx) + (y - cy) * (y - cy) < radius * radius {
                sx += x
                sy += y
                inside += 1
            }
            if inside == 0 { return nil }
            cx = sx / Float(inside)
            cy = sy / Float(inside)
        }
        return inside >= minCount ? (cx, cy) : nil
    }
}

extension MotionLocator {
    func diagnosticState() -> EngineDiagnostics.LocatorState {
        .init(
            frame: frame, nextID: nextID, tile: tile, candidate: candidate.map { [$0.x, $0.y] },
            candidateSince: candidateSince, movingCount: movingCount,
            points: points.map {
                .init(
                    id: $0.id, x: $0.x, y: $0.y, previousX: $0.previousX, previousY: $0.previousY,
                    speed: $0.speed, duration: $0.duration, stillFor: $0.stillFor)
            },
            poses: poses.map { .init(time: $0.time, pose: $0.pose) },
            maxPoints: maxPoints, detectEvery: detectEvery, speedTau: speedTau, movingSpeed: movingSpeed,
            minMoving: minMoving, settleSeconds: settleSeconds, maxResidual: maxResidual, border: border)
    }
}
