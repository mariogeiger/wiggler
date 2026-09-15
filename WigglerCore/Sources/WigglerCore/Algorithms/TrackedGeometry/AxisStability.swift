import Foundation

/// Confirms an axis with fresh estimates, and detects a moved object by an estimate from recent chords alone,
/// before old observations can outvote them.
///
/// Single chords carry the depth noise of two LiDAR samples (≈ 1 cm at arm's length), so no per-chord
/// tolerance can tell a moved axis from noise. An estimate over the recent window averages that noise away;
/// it is compared with the current axis only when it is well conditioned, and a contradiction must persist
/// over several evaluations, so an occluder passing in front of the object cannot trigger it.
struct AxisStability {
    private(set) var confirmations = 0
    private(set) var newestFrame = -1
    /// Consecutive recent-window estimates contradicting the axis.
    private(set) var contradictions = 0

    mutating func reset() {
        confirmations = 0
        newestFrame = -1
        contradictions = 0
    }

    func hasNewEvidence(frame: Int) -> Bool { frame > newestFrame }

    mutating func observe(frame: Int, consistent: Bool) {
        guard hasNewEvidence(frame: frame) else { return }
        newestFrame = frame
        confirmations = consistent ? confirmations + 1 : 0
    }

    mutating func invalidate() { confirmations = 0 }

    func isStable(required: Int) -> Bool { contradictions == 0 && confirmations >= max(1, required) }

    /// A contradicting recent estimate hides the map immediately; `required` of them in a row mean the object
    /// was moved. An ill-conditioned recent window gives no verdict and keeps the count.
    mutating func observe(recent: AxisEstimate?, axis: Axis, within distance: Double, required: Int) -> Bool {
        guard let recent, recent.isWellConditioned else { return false }
        if Self.agrees(recent.axis, with: axis, within: distance) {
            contradictions = 0
            return false
        }
        contradictions += 1
        invalidate()
        return contradictions >= max(1, required)
    }

    /// Same line up to sign and position along it: directions within 12°, lines within `distance`.
    static func agrees(_ candidate: Axis, with reference: Axis, within distance: Double) -> Bool {
        let cosine = abs(candidate.direction.dot(reference.direction))
        let d = candidate.origin - reference.origin
        let separation = (d - reference.direction * d.dot(reference.direction)).length
        return cosine > cos(12 * .pi / 180) && separation < distance
    }
}
