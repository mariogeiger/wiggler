import Foundation

/// Confirms an axis with fresh estimates and checks new chords before old observations can outvote them.
struct AxisStability {
    private(set) var confirmations = 0
    private(set) var newestFrame = -1
    private(set) var inconsistentBatches = 0

    mutating func reset() {
        confirmations = 0
        newestFrame = -1
        inconsistentBatches = 0
    }

    func hasNewEvidence(frame: Int) -> Bool { frame > newestFrame }

    mutating func observe(frame: Int, consistent: Bool) {
        guard hasNewEvidence(frame: frame) else { return }
        newestFrame = frame
        confirmations = consistent ? confirmations + 1 : 0
    }

    mutating func invalidate() { confirmations = 0 }

    func isStable(required: Int) -> Bool { inconsistentBatches == 0 && confirmations >= max(1, required) }

    /// A suspect batch hides the map immediately; persistent contradictions restart calibration.
    mutating func observe(
        chords: [ChordConstraint], axis: Axis, tolerance: Double, required: Int,
        diagnostics: EngineDiagnosticsRecorder? = nil
    ) -> Bool {
        let confirmationsBefore = confirmations
        let inconsistentBefore = inconsistentBatches
        defer {
            diagnostics?.observeChordBatch(
                chords, axis: axis, tolerance: tolerance, required: required,
                confirmationsBefore: confirmationsBefore, confirmationsAfter: confirmations,
                inconsistentBefore: inconsistentBefore, inconsistentAfter: inconsistentBatches)
        }
        guard chords.count >= 8 else { return false }
        if Self.rejects(chords, axis: axis, tolerance: tolerance) {
            invalidate()
            inconsistentBatches += 1
        } else {
            inconsistentBatches = 0
        }
        return inconsistentBatches >= max(1, required)
    }

    static func agrees(_ candidate: Axis, with reference: Axis, objectRadius: Double) -> Bool {
        let cosine = abs(candidate.direction.dot(reference.direction))
        let d = candidate.origin - reference.origin
        let distance = (d - reference.direction * d.dot(reference.direction)).length
        return cosine > cos(12 * .pi / 180) && distance < max(0.05, 0.5 * objectRadius)
    }

    /// Both terms vanish for a circular trajectory: d·axis = 0 and (midpoint−origin)·d = 0.
    static func rejects(_ chords: [ChordConstraint], axis: Axis, tolerance: Double) -> Bool {
        guard chords.count >= 8 else { return false }
        let bad = chords.filter { c in
            let length = c.chord.length
            guard length > 0 else { return false }
            let u = c.chord / length
            let tilt = c.chord.dot(axis.direction)
            let offset = (c.midpoint - axis.origin).dot(u)
            return hypot(tilt, offset) > tolerance
        }.count
        return bad * 2 > chords.count
    }
}
