import Foundation

extension EngineDiagnosticsRecorder {
    func observeChordBatch(
        _ chords: [ChordConstraint], axis: Axis, tolerance: Double, required: Int,
        confirmationsBefore: Int, confirmationsAfter: Int, inconsistentBefore: Int, inconsistentAfter: Int
    ) {
        let residuals: [EngineDiagnostics.ChordResidual] = chords.enumerated().map { index, chord in
            let length = chord.chord.length
            let tilt = chord.chord.dot(axis.direction)
            let offset = length > 0 ? (chord.midpoint - axis.origin).dot(chord.chord / length) : 0
            let residual = hypot(tilt, offset)
            let id = index < value.freshChords.count ? value.freshChords[index].trackID : nil
            return .init(
                trackID: id, tiltMeters: tilt, offsetMeters: offset, residualMeters: residual,
                exceedsTolerance: length > 0 && residual > tolerance)
        }
        let badCount = residuals.filter { $0.exceedsTolerance }.count
        value.chordBatch = .init(
            axis: axis, toleranceMeters: tolerance, requiredBatches: max(1, required), count: chords.count,
            badCount: badCount, rejects: chords.count >= 8 && badCount * 2 > chords.count, residuals: residuals,
            confirmationsBefore: confirmationsBefore, confirmationsAfter: confirmationsAfter,
            inconsistentBefore: inconsistentBefore, inconsistentAfter: inconsistentAfter,
            restartRequired: chords.count >= 8 && inconsistentAfter >= max(1, required))
    }
}
