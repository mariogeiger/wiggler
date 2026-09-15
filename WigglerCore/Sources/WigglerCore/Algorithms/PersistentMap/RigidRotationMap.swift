import Foundation

/// Rodrigues rotation of `point` about the unit vector `direction` by `angle`.
@inline(__always) func rotatedAbout(_ direction: V3, by angle: Double, _ point: V3) -> V3 {
    let c = cos(angle), s = sin(angle)
    return point * c + direction.cross(point) * s + direction * (direction.dot(point) * (1 - c))
}

/// The landmarks of one rigid body, held in the body's own frame, and the single angle that explains where they
/// are seen. A landmark keeps for good the position and the surface patch it had when it was measured, so the
/// angle is fitted against a fixed map instead of being integrated frame by frame, and a landmark that was hidden
/// for a while is found again by the same fit. Nothing here is ever rewritten from a filtered angle: a map that
/// re-anchored itself on its own predictions would be adjacent-frame integration under another name.
final class RigidRotationMap {
    struct Landmark {
        let id: Int
        /// Position in the body frame: the world position rotated back by the angle at which it was measured.
        let canonical: V3
        /// The surface element's in-plane half-extent vectors, in the body frame, measured from depth.
        let tangentU: V3
        let tangentV: V3
        /// Zero-mean unit-norm samples of the patch, as first seen. Immutable.
        let descriptor: [Float]
        /// The tracked image point this landmark was born on. It never changes and it is never reused: one track
        /// gets one landmark, so material already in the map cannot be learnt a second time at today's angle.
        let birthTrackID: Int
        /// The tracked image point still believed to carry this material point, zero when none does. The geometry
        /// above never changes; a claim about which pixel path carries it can only be dropped.
        var trackID: Int
        let firstFrame: Int
        var lastSeenFrame: Int
        var observations: Int
        /// Distance to the axis and height along it, both fixed with the body.
        let radius: Double
        let height: Double
    }

    /// What one frame says about the angle.
    struct Measurement {
        var theta: Double
        var support: Int
        var visible: Int
        var reprojectionRMS: Double
        /// Robust angular scatter of the supporting landmarks, radians.
        var dispersion: Double
        /// Information the supporting landmarks carry about the angle, radians⁻².
        var information: Double
        var searchWindow: Double
        /// Score of the best rival mode of the coarse search, relative to the accepted one: 0 when the angle was
        /// not searched, near 1 when another angle explains the frame nearly as well.
        var ambiguity: Double
        /// Indices of the landmarks that agree, and of those seen unoccluded yet contradicting. Valid until the
        /// map is trimmed, which renumbers it; `supportingPixels` and `contradictingPixels` outlive that.
        var supporting: [Int]
        var contradicting: [Int]
        var supportingPixels: [(x: Double, y: Double)]
        var contradictingPixels: [(x: Double, y: Double)]
        /// Pixels held by landmarks seen unoccluded this frame: where a new landmark would only be a duplicate.
        var occupied: [(x: Double, y: Double)]
        /// Where the supporting landmarks sit in the image, and how far they spread.
        var centerX: Double
        var centerY: Double
        var spreadPixels: Double
    }

    let axis: Axis
    let sampler: SurfacePatchSampler
    private(set) var landmarks: [Landmark] = []
    private var scratch: [Float]
    private var nextID = 1

    /// Most landmarks kept. The map is trimmed to this by dropping the least observed and least recently seen, so
    /// both memory and the cost of a frame stay bounded however long the session runs.
    var capacity = 600
    /// Correlation a patch must reach to be accepted as the same material point.
    var acceptCorrelation = 0.5
    /// Correlation from which a patch contributes to the coarse search over the angle.
    var searchCorrelation = 0.3
    /// Reprojection error a supporting landmark may have, pixels.
    var gatePixels = 3.0
    /// Spacing of the 3x3 appearance search around the predicted pixel, pixels.
    var searchStepPixels = 1.5
    /// Spacing of the coarse search over the angle, radians.
    var coarseStep = 1.5 * Double.pi / 180
    /// Landmarks used by the coarse search: the most observed ones, so a wide search stays affordable.
    var coarseLandmarkLimit = 80
    /// Steps to either side the coarse search may take.
    var coarseStepLimit = 48
    /// Fewest landmarks that may carry a measurement. The angle is one unknown and each landmark gives two pixel
    /// equations, so this is the redundancy the truncated-quadratic fit needs to reject outliers — not a budget for
    /// a body of a particular size.
    var minSupport = 8
    /// Standard deviation assumed for one landmark's pixel position.
    var pixelSigma = 1.5
    /// Fraction by which the depth at a landmark's pixel may differ from its predicted range before the landmark
    /// counts as hidden by something else.
    var depthTolerance = 0.12
    /// Pixels a new landmark must keep from every landmark seen unoccluded this frame.
    var separationPixels = 8.0
    /// Coarse steps a rival mode must stand away from the accepted angle to be another explanation of the frame.
    var rivalSeparation = 3

    init(axis: Axis, sampler: SurfacePatchSampler) {
        self.axis = axis
        self.sampler = sampler
        scratch = [Float](repeating: 0, count: sampler.count)
    }

    var count: Int { landmarks.count }

    @inline(__always) func world(_ landmark: Landmark, at theta: Double) -> V3 {
        axis.origin + rotatedAbout(axis.direction, by: theta, landmark.canonical)
    }

    /// Where a landmark is predicted to be seen at a given angle.
    func pixel(of index: Int, at theta: Double, view: PinholeProjection) -> (x: Double, y: Double)? {
        guard let projected = view.project(world(landmarks[index], at: theta)) else { return nil }
        return (projected.x, projected.y)
    }

    /// Pixel velocity of a landmark for one radian of rotation.
    @inline(__always) private func angleDifferential(at point: V3, view: PinholeProjection) -> (
        dx: Double, dy: Double
    ) {
        view.differential(at: point, along: axis.direction.cross(point - axis.origin))
    }

    /// Sampling grid of a landmark's patch at a given angle: the surface element turned with the body.
    @inline(__always) private func warp(_ landmark: Landmark, at theta: Double, view: PinholeProjection)
        -> SurfacePatchSampler.Warp?
    {
        sampler.warp(
            center: world(landmark, at: theta), u: rotatedAbout(axis.direction, by: theta, landmark.tangentU),
            v: rotatedAbout(axis.direction, by: theta, landmark.tangentV), view: view)
    }

    /// Where the map puts a landmark in this frame, when the camera should see it there at all: nil when it falls
    /// outside the image and nil when the depth at that pixel belongs to something nearer, since a landmark hidden
    /// behind another surface says nothing about the angle.
    func predictedPixel(
        of index: Int, at theta: Double, view: PinholeProjection, depth: DepthMap?, minConfidence: UInt8
    ) -> (x: Double, y: Double, range: Double)? {
        guard let projected = view.project(world(landmarks[index], at: theta)) else { return nil }
        if let depth,
            let z = depth.sample(
                u: projected.x / view.width, v: projected.y / view.height, minConfidence: minConfidence),
            abs(z - projected.range) > depthTolerance * projected.range
        {
            return nil
        }
        return (projected.x, projected.y, projected.range)
    }

    /// The angle that best explains this frame, searched over `predicted` ± `window` and then fitted to the pixels
    /// the supporting landmarks are actually seen at. Nil when too few landmarks agree.
    func measure(
        image: GrayImage, depth: DepthMap?, view: PinholeProjection, predicted: Double, window: Double,
        minConfidence: UInt8
    ) -> Measurement? {
        // The coarse search asks each candidate angle how many landmarks it explains, the soft sum of correlations
        // breaking ties. Only a genuine local maximum counts as a rival explanation: the shoulders of one broad peak
        // are the same peak, and a search of the whole circle wraps, so its seam is not a rival either.
        let fullCircle = window >= Double.pi
        let steps = min(coarseStepLimit, max(0, Int((window / coarseStep).rounded(.up))))
        var coarse = predicted
        var ambiguity = 0.0
        if steps > 0 {
            let order = landmarks.indices.sorted {
                let a = landmarks[$0], b = landmarks[$1]
                if a.observations != b.observations { return a.observations > b.observations }
                return a.lastSeenFrame > b.lastSeenFrame
            }
            let subset = Array(order.prefix(coarseLandmarkLimit))
            guard !subset.isEmpty else { return nil }
            let candidates = fullCircle ? Int((2 * Double.pi / coarseStep).rounded()) : 2 * steps + 1
            // A search of the whole circle answers modulo a turn, so its candidates straddle the angle carried in:
            // the answer is then the representative nearest that angle. Offsets that only ran forward would report a
            // body that has fallen behind the prediction as one that has run a whole turn ahead of it.
            let centre = fullCircle ? candidates / 2 : steps
            var scores = [Double](repeating: 0, count: candidates)
            for candidate in 0..<candidates {
                let theta = predicted + Double(candidate - centre) * coarseStep
                var explained = 0
                var soft = 0.0
                for index in subset {
                    guard let grid = warp(landmarks[index], at: theta, view: view),
                        sampler.fill(&scratch, from: image, warp: grid)
                    else { continue }
                    let value = SurfacePatchSampler.correlation(scratch, landmarks[index].descriptor)
                    if value >= acceptCorrelation { explained += 1 }
                    soft += max(0, value - searchCorrelation)
                }
                // Lexicographic in (landmarks explained, soft agreement): the tie-break is below one by
                // construction, since each term is at most 1 - searchCorrelation.
                scores[candidate] = Double(explained) + soft / Double(subset.count)
            }
            var best = 0
            for candidate in scores.indices where scores[candidate] > scores[best] { best = candidate }
            guard scores[best] >= 1 else { return nil }
            coarse = predicted + Double(best - centre) * coarseStep
            ambiguity = Self.rivalRatio(scores, best: best, periodic: fullCircle, separation: rivalSeparation)
        }

        // Where each landmark is actually seen: the appearance peak within a few pixels of what the map predicts.
        // A landmark whose pixel shows depth other than its own range is hidden behind something and says nothing
        // about the angle — and does not hold that pixel against a new landmark either.
        var observed: [(index: Int, x: Double, y: Double)] = []
        var contradicting: [Int] = []
        var occupied: [(x: Double, y: Double)] = []
        for index in landmarks.indices {
            let landmark = landmarks[index]
            guard
                let projected = predictedPixel(
                    of: index, at: coarse, view: view, depth: depth, minConfidence: minConfidence),
                let grid = warp(landmark, at: coarse, view: view)
            else { continue }
            var scores = [Double](repeating: -2, count: 9)
            for j in -1...1 {
                for i in -1...1 {
                    let dx = Double(i) * searchStepPixels, dy = Double(j) * searchStepPixels
                    guard sampler.fill(&scratch, from: image, warp: grid, offsetX: dx, offsetY: dy) else {
                        continue
                    }
                    scores[(j + 1) * 3 + i + 1] = SurfacePatchSampler.correlation(scratch, landmark.descriptor)
                }
            }
            var peak = 0
            for cell in scores.indices where scores[cell] > scores[peak] { peak = cell }
            guard scores[peak] > -2 else { continue }
            occupied.append((projected.x, projected.y))
            guard scores[peak] >= acceptCorrelation else {
                contradicting.append(index)
                continue
            }
            let column = peak % 3, row = peak / 3
            let dx =
                (Double(column - 1) + Self.parabolic(scores, at: peak, stride: 1, edge: column))
                * searchStepPixels
            let dy = (Double(row - 1) + Self.parabolic(scores, at: peak, stride: 3, edge: row)) * searchStepPixels
            observed.append((index, projected.x + dx, projected.y + dy))
        }
        let visible = occupied.count
        guard observed.count >= minSupport else { return nil }
        return fitted(
            observed, contradicting: contradicting, occupied: occupied, visible: visible, start: coarse,
            near: predicted, searchWindow: Double(steps) * coarseStep, ambiguity: ambiguity, view: view)
    }

    /// The angle that puts the map on the pixels its landmarks were seen at, whatever established that they are the
    /// same material points. Iteratively reweighted least squares in the one unknown, then the measurement that
    /// angle earns: its supporters, its residual, and the information it carries about the angle.
    /// The representative of `theta` nearest `reference`. An angle is measured modulo a turn; only continuity says
    /// which turn it belongs to, so the choice is made once and by that rule alone.
    @inline(__always) static func lifted(_ theta: Double, near reference: Double) -> Double {
        theta + 2 * Double.pi * ((reference - theta) / (2 * Double.pi)).rounded()
    }

    func fitted(
        _ observed: [(index: Int, x: Double, y: Double)], contradicting: [Int],
        occupied: [(x: Double, y: Double)], visible: Int, start: Double, near reference: Double,
        searchWindow: Double, ambiguity: Double, view: PinholeProjection
    ) -> Measurement? {
        var contradicting = contradicting
        var theta = start
        // The refinement answers the same question the search did, so it minimises the same bounded cost:
        // r²/(g² + r²) summed over the associations, whose reweighting factor is (1 + r²/g²)⁻². A different loss
        // here could improve itself while walking out of the mode the search chose. Nor may a step leave that mode:
        // the search located it to within its own spacing, so a larger step means the linearisation does not hold
        // there, and a fit that ends outside the mode it was refining is refused rather than believed. The turn is
        // settled afterwards, on the angle the refinement actually reached: settling it before would let a step
        // across the seam leave the answer on the neighbouring turn.
        for _ in 0..<5 {
            var numerator = 0.0, denominator = 0.0
            for item in observed {
                let point = world(landmarks[item.index], at: theta)
                guard let projected = view.project(point) else { continue }
                let jacobian = angleDifferential(at: point, view: view)
                let rx = item.x - projected.x, ry = item.y - projected.y
                let scaled = 1 + (rx * rx + ry * ry) / (gatePixels * gatePixels)
                let weight = 1 / (scaled * scaled)
                numerator += weight * (jacobian.dx * rx + jacobian.dy * ry)
                denominator += weight * (jacobian.dx * jacobian.dx + jacobian.dy * jacobian.dy)
            }
            guard denominator > 1e-12 else { break }
            theta += max(-coarseStep, min(coarseStep, numerator / denominator))
        }
        guard abs(theta - start) <= coarseStep else { return nil }

        var supporting: [Int] = []
        var supportingPixels: [(x: Double, y: Double)] = []
        var squares = 0.0, information = 0.0, angular = 0.0
        var sumX = 0.0, sumY = 0.0, sumX2 = 0.0, sumY2 = 0.0
        for item in observed {
            let point = world(landmarks[item.index], at: theta)
            guard let projected = view.project(point) else { continue }
            let jacobian = angleDifferential(at: point, view: view)
            let rx = item.x - projected.x, ry = item.y - projected.y
            let error = (rx * rx + ry * ry).squareRoot()
            let gradient = (jacobian.dx * jacobian.dx + jacobian.dy * jacobian.dy).squareRoot()
            guard error < gatePixels else {
                contradicting.append(item.index)
                continue
            }
            supporting.append(item.index)
            supportingPixels.append((item.x, item.y))
            squares += error * error
            information += gradient * gradient / (pixelSigma * pixelSigma)
            if gradient > 1e-9 {
                let along = (jacobian.dx * rx + jacobian.dy * ry) / (gradient * gradient)
                angular += along * along
            }
            sumX += item.x
            sumY += item.y
            sumX2 += item.x * item.x
            sumY2 += item.y * item.y
        }
        guard supporting.count >= minSupport else { return nil }
        let n = Double(supporting.count)
        let centerX = sumX / n, centerY = sumY / n
        let spread = (max(0, sumX2 / n - centerX * centerX) + max(0, sumY2 / n - centerY * centerY)).squareRoot()
        return Measurement(
            theta: Self.lifted(theta, near: reference), support: supporting.count, visible: visible,
            reprojectionRMS: (squares / n).squareRoot(), dispersion: (angular / n).squareRoot(),
            information: information, searchWindow: searchWindow, ambiguity: ambiguity,
            supporting: supporting, contradicting: contradicting, supportingPixels: supportingPixels,
            contradictingPixels: contradicting.compactMap { index in
                view.project(world(landmarks[index], at: theta)).map { ($0.x, $0.y) }
            }, occupied: occupied, centerX: centerX, centerY: centerY, spreadPixels: spread)
    }

    /// The angle measured from the landmarks whose image tracks are still alive. Identity comes from the tracks —
    /// the image sequence says which pixel carries which material point — so the angle is not searched for near
    /// where the estimate already expects it. The whole circle is scored, every time, and what wins wins against
    /// every other angle rather than against the shoulders of a window. `predicted` therefore chooses nothing but
    /// the turn the answer belongs to: an angle is measured modulo a full turn, and only continuity says how many
    /// turns have gone by.
    func measureByTrack(
        observations: [TrackedWorldPoints.Observation], predicted: Double, image: GrayImage,
        view: PinholeProjection, depth: DepthMap?, minConfidence: UInt8
    ) -> (bound: Int, measurement: Measurement?) {
        var pixelOfTrack: [Int: (x: Double, y: Double)] = [:]
        pixelOfTrack.reserveCapacity(observations.count)
        for observation in observations where observation.id != 0 {
            pixelOfTrack[observation.id] = (observation.x, observation.y)
        }
        var bound: [(index: Int, x: Double, y: Double)] = []
        for index in landmarks.indices {
            guard landmarks[index].trackID != 0, let pixel = pixelOfTrack[landmarks[index].trackID] else {
                continue
            }
            bound.append((index, pixel.x, pixel.y))
        }
        guard bound.count >= minSupport else { return (bound.count, nil) }

        // How well an angle explains the tracks: each one contributes near one while it lands within a gate of
        // where the map says it should be, and falls away smoothly beyond it. The sum is bounded, so no single
        // track can carry the maximum, and it is smooth, so the coarse step does not have to straddle the peak.
        let candidates = max(3, Int((2 * Double.pi / coarseStep).rounded()))
        let step = 2 * Double.pi / Double(candidates)
        var scores = [Double](repeating: 0, count: candidates)
        for candidate in 0..<candidates {
            let theta = Double(candidate) * step
            var score = 0.0
            for item in bound {
                guard let projected = view.project(world(landmarks[item.index], at: theta)) else { continue }
                let rx = item.x - projected.x, ry = item.y - projected.y
                let square = (rx * rx + ry * ry) / (gatePixels * gatePixels)
                score += 1 / (1 + square)
            }
            scores[candidate] = score
        }
        var best = 0
        for candidate in scores.indices where scores[candidate] > scores[best] { best = candidate }
        let ambiguity = Self.rivalRatio(scores, best: best, periodic: true, separation: rivalSeparation)
        let start = Self.lifted(Double(best) * step, near: predicted)

        // Which of them the camera should be seeing at that angle, and where the map then says every landmark is: a
        // point already spoken for is not free for a new landmark, whether or not it is tracked. A track is believed
        // only while the pixel it claims still looks like the landmark it carries — the patch is read exactly where
        // the track says, never searched for, so this can strike a claim out but can never move the angle. A frame
        // with nothing to see, or a track that has slid onto something else, therefore stops being evidence instead
        // of becoming false evidence.
        var observed: [(index: Int, x: Double, y: Double)] = []
        var contradicting: [Int] = []
        var occupied: [(x: Double, y: Double)] = []
        var pixelOfLandmark = [Int](repeating: -1, count: landmarks.count)
        for item in bound.indices { pixelOfLandmark[bound[item].index] = item }
        for index in landmarks.indices {
            guard
                let projected = predictedPixel(
                    of: index, at: start, view: view, depth: depth, minConfidence: minConfidence)
            else { continue }
            occupied.append((projected.x, projected.y))
            let item = pixelOfLandmark[index]
            guard item >= 0 else { continue }
            guard let grid = warp(landmarks[index], at: start, view: view),
                sampler.fill(
                    &scratch, from: image, warp: grid, offsetX: bound[item].x - projected.x,
                    offsetY: bound[item].y - projected.y),
                SurfacePatchSampler.correlation(scratch, landmarks[index].descriptor) >= acceptCorrelation
            else {
                contradicting.append(index)
                continue
            }
            observed.append(bound[item])
        }
        guard observed.count >= minSupport else { return (bound.count, nil) }
        return (
            bound.count,
            fitted(
                observed, contradicting: contradicting, occupied: occupied, visible: occupied.count, start: start,
                near: predicted, searchWindow: .pi, ambiguity: ambiguity, view: view)
        )
    }

    /// Let go of the tracks that no longer speak for their landmarks: the ones that have ended, and the ones whose
    /// pixel contradicts where the rigid body puts the landmark. Only claims of identity are dropped here; nothing
    /// is claimed. A landmark keeps its canonical position and its patch, so it is the same material point it
    /// always was, merely uncarried — and being uncarried, it stops voting on the angle rather than voting wrongly.
    /// Nothing binds a free landmark to a new track: proximity to where the map expects it, even with a patch that
    /// correlates, is not proof that a given corner is that material point, and a wrong binding would be believed
    /// as evidence.
    func releaseTracksThatDisagree(with observations: [TrackedWorldPoints.Observation], measurement: Measurement)
        -> (bound: Int, ended: Int, contradicted: Int)
    {
        var live = Set<Int>()
        live.reserveCapacity(observations.count)
        for observation in observations where observation.id != 0 { live.insert(observation.id) }
        var contradicted = 0
        for index in measurement.contradicting where landmarks[index].trackID != 0 {
            landmarks[index].trackID = 0
            contradicted += 1
        }
        var bound = 0
        var ended = 0
        for index in landmarks.indices where landmarks[index].trackID != 0 {
            if live.contains(landmarks[index].trackID) {
                bound += 1
            } else {
                landmarks[index].trackID = 0
                ended += 1
            }
        }
        return (bound, ended, contradicted)
    }

    /// The best rival mode's score relative to the accepted one. A rival is a local maximum of the score far enough
    /// from the accepted angle to be another explanation rather than a shoulder of the same peak; when the whole
    /// circle was searched, distance is measured around it.
    static func rivalRatio(_ scores: [Double], best: Int, periodic: Bool, separation: Int) -> Double {
        let count = scores.count
        guard count > 2, scores[best] > 0 else { return 0 }
        var rival = 0.0
        for index in 0..<count {
            let straight = abs(index - best)
            if (periodic ? min(straight, count - straight) : straight) <= separation { continue }
            // A missing neighbour cannot vouch for a candidate: the edge of a truncated search may be a mode still
            // rising outside it, which is exactly an explanation this frame did not resolve.
            let previous = periodic ? scores[(index - 1 + count) % count] : (index > 0 ? scores[index - 1] : -1)
            let next = periodic ? scores[(index + 1) % count] : (index + 1 < count ? scores[index + 1] : -1)
            guard scores[index] >= previous, scores[index] >= next else { continue }
            rival = max(rival, scores[index])
        }
        return rival / scores[best]
    }

    /// Sub-pixel offset of a correlation peak from its two neighbours; zero at the edge of the search grid.
    private static func parabolic(_ scores: [Double], at peak: Int, stride: Int, edge: Int) -> Double {
        guard edge == 1, scores[peak - stride] > -2, scores[peak + stride] > -2 else { return 0 }
        let denominator = scores[peak - stride] - 2 * scores[peak] + scores[peak + stride]
        guard denominator < -1e-12 else { return 0 }
        return max(-0.5, min(0.5, 0.5 * (scores[peak - stride] - scores[peak + stride]) / denominator))
    }

    /// Re-date the landmarks that agreed, and report those that came back after being unseen for a while. Their
    /// positions and patches are left exactly as they were measured.
    func markSeen(_ measurement: Measurement, frame: Int, gapFrames: Int, recordIdentities: Bool) -> (
        returned: Int, returnedIDs: [Int], observedIDs: [Int]
    ) {
        var returned = 0
        var returnedIDs: [Int] = []
        var observedIDs: [Int] = []
        for index in measurement.supporting {
            if frame - landmarks[index].lastSeenFrame > gapFrames {
                returned += 1
                if recordIdentities { returnedIDs.append(landmarks[index].id) }
            }
            if recordIdentities { observedIDs.append(landmarks[index].id) }
            landmarks[index].lastSeenFrame = frame
            landmarks[index].observations += 1
        }
        return (returned, returnedIDs, observedIDs)
    }

    /// Take in candidate points the map does not explain yet, as landmarks of the body at the angle just measured.
    /// Whether a point belongs to the body is the caller's judgement; here a point is refused when the map already
    /// predicts one nearby, when it lies on the axis, or when depth cannot describe its surface plane, since its
    /// patch could then not be transported to another angle.
    func insert(
        _ observations: [TrackedWorldPoints.Observation], theta: Double, image: GrayImage, depth: DepthMap?,
        view: PinholeProjection, frame: Int, minConfidence: UInt8, limit: Int,
        occupied: [(x: Double, y: Double)]
    ) -> (added: Int, planeRejected: Int) {
        var taken = occupied
        var carried = Set<Int>()
        for landmark in landmarks where landmark.birthTrackID != 0 { carried.insert(landmark.birthTrackID) }
        var added = 0
        var planeRejected = 0
        let separation = separationPixels * separationPixels
        for observation in observations {
            if added >= limit { break }
            // One track carries one material point. A track already carrying a landmark may not make a second one,
            // however far its old projection has drifted or however hidden it has become, or the same pixel would
            // enter the fit twice and lend it support and information it does not have.
            if observation.id != 0, carried.contains(observation.id) { continue }

            if taken.contains(where: {
                ($0.x - observation.x) * ($0.x - observation.x) + ($0.y - observation.y) * ($0.y - observation.y)
                    < separation
            }) {
                continue
            }
            let (_, radius, height) = axis.cylindrical(observation.world)
            guard radius > 0.01, observation.world.isFinite else { continue }
            guard
                let tangents = sampler.surfaceTangents(
                    x: observation.x, y: observation.y, world: observation.world, range: observation.range,
                    depth: depth, view: view, minConfidence: minConfidence)
            else {
                planeRejected += 1
                continue
            }
            guard
                let grid = sampler.warp(
                    center: observation.world, u: tangents.u, v: tangents.v, view: view),
                sampler.fill(&scratch, from: image, warp: grid)
            else { continue }
            landmarks.append(
                Landmark(
                    id: nextID, canonical: rotatedAbout(axis.direction, by: -theta, observation.world - axis.origin),
                    tangentU: rotatedAbout(axis.direction, by: -theta, tangents.u),
                    tangentV: rotatedAbout(axis.direction, by: -theta, tangents.v), descriptor: scratch,
                    birthTrackID: observation.id, trackID: observation.id,
                    firstFrame: frame, lastSeenFrame: frame, observations: 1, radius: radius, height: height))
            nextID += 1
            taken.append((observation.x, observation.y))
            if observation.id != 0 { carried.insert(observation.id) }
            added += 1
        }
        return (added, planeRejected)
    }

    /// Keep the map within capacity. A landmark earns its place by how often it has been seen per frame of its
    /// existence: a landmark of a surface that keeps coming back scores well, one seen twice long ago scores
    /// nothing, and a landmark admitted this frame scores highest, so a full map can still learn a new surface.
    func retention(_ landmark: Landmark, frame: Int) -> Double {
        Double(landmark.observations) / Double(max(1, frame - landmark.firstFrame + 1))
    }

    func trim(frame: Int) -> Int {
        guard landmarks.count > capacity else { return 0 }
        let excess = landmarks.count - capacity
        let order = landmarks.indices.sorted {
            let a = retention(landmarks[$0], frame: frame), b = retention(landmarks[$1], frame: frame)
            if a != b { return a < b }
            return landmarks[$0].lastSeenFrame < landmarks[$1].lastSeenFrame
        }
        let dropped = Set(order.prefix(excess))
        landmarks = landmarks.enumerated().filter { !dropped.contains($0.offset) }.map { $0.element }
        return excess
    }

    /// The body's extent in axis coordinates, from the landmarks themselves.
    func extent() -> (radius: Double, heightMin: Double, heightMax: Double) {
        guard !landmarks.isEmpty else { return (0, 0, 0) }
        return (
            percentile(landmarks.map { $0.radius }, 0.8), percentile(landmarks.map { $0.height }, 0.05),
            percentile(landmarks.map { $0.height }, 0.95)
        )
    }
}
