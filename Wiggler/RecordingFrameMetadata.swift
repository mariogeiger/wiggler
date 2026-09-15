import Foundation
import WigglerCore

/// Per-frame UI selection and the generation of the converted input.
struct RecordingSettings: Encodable {
    var harmonicSignal = "luma"
    var harmonicOrder: Int?
    var harmonicRevision = 0
    var inputRevision = 0
    var convertedSignal = "luma"
    var harmonicInputAccepted = true
    var harmonicTurnProgress = 0.0
    var cameraTrackingState = "unknown"

    private enum CodingKeys: String, CodingKey {
        case harmonicSignal, harmonicOrder, harmonicRevision, inputRevision, convertedSignal
        case harmonicInputAccepted, harmonicTurnProgress, cameraTrackingState
    }

    func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(harmonicSignal, forKey: .harmonicSignal)
        try container.encode(harmonicOrder, forKey: .harmonicOrder)
        try container.encode(harmonicRevision, forKey: .harmonicRevision)
        try container.encode(inputRevision, forKey: .inputRevision)
        try container.encode(convertedSignal, forKey: .convertedSignal)
        try container.encode(harmonicInputAccepted, forKey: .harmonicInputAccepted)
        try container.encode(harmonicTurnProgress, forKey: .harmonicTurnProgress)
        try container.encode(cameraTrackingState, forKey: .cameraTrackingState)
    }
}

/// The v2 metadata schema, encoded without an untyped JSON round trip.
struct RecordingFrameMetadata: Encodable {
    let input: FrameInput
    let output: EngineOutput
    let config: EngineConfig
    let settings: RecordingSettings
    let sequence: Int
    let droppedFrames: Int
    let conversionFailures: Int
    let processedFps: Double
    /// Writer time of the previous frame (ms) and frames still queued when this one was appended.
    var recorderMillis = 0.0
    var pendingFrames = 0

    private struct Track: Encodable {
        let x: Float
        let y: Float
        let status: String
    }

    private enum CodingKeys: String, CodingKey {
        case t
        case width
        case height
        case fx
        case fy
        case cx
        case cy
        case rotation
        case translation
        case poseValid
        case depthWidth
        case depthHeight
        case chromaRedWidth
        case chromaRedHeight
        case chromaBlueWidth
        case chromaBlueHeight
        case state
        case theta
        case angleDegrees
        case angleConfidence
        case axisStable
        case axisGeneration
        case angleMeasured
        case axisQuality
        case rpm
        case trackCount
        case inlierCount
        case constraintCount
        case processingMillis
        case objectRadius
        case heightMin
        case heightMax
        case angleUncertaintyDegrees
        case message
        case tracks
        case roiRadius
        case dispersionDeg
        case relocAnalysed
        case relocFill
        case relocAge
        case periodDeg
        case turnDeg
        case sequence
        case config
        case settings
        case droppedFrames
        case conversionFailures
        case processedFps
        case recorderMillis
        case pendingFrames
        case diagnostics
        case marker
        case axisOrigin
        case axisDirection
    }

    func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        let t = input.cameraToWorld.translation
        try container.encode(input.timestamp, forKey: .t)
        try container.encode(input.image.width, forKey: .width)
        try container.encode(input.image.height, forKey: .height)
        try container.encode(input.intrinsics.fx, forKey: .fx)
        try container.encode(input.intrinsics.fy, forKey: .fy)
        try container.encode(input.intrinsics.cx, forKey: .cx)
        try container.encode(input.intrinsics.cy, forKey: .cy)
        try container.encode(input.cameraToWorld.rotation.m, forKey: .rotation)
        try container.encode([t.x, t.y, t.z], forKey: .translation)
        try container.encode(input.poseValid, forKey: .poseValid)
        try container.encode(input.depth?.width ?? 0, forKey: .depthWidth)
        try container.encode(input.depth?.height ?? 0, forKey: .depthHeight)
        try container.encode(input.chromaRed?.width ?? 0, forKey: .chromaRedWidth)
        try container.encode(input.chromaRed?.height ?? 0, forKey: .chromaRedHeight)
        try container.encode(input.chromaBlue?.width ?? 0, forKey: .chromaBlueWidth)
        try container.encode(input.chromaBlue?.height ?? 0, forKey: .chromaBlueHeight)
        try container.encode(output.state.rawValue, forKey: .state)
        try container.encode(output.theta, forKey: .theta)
        try container.encode(output.angleDegrees, forKey: .angleDegrees)
        try container.encode(output.angleConfidence, forKey: .angleConfidence)
        try container.encode(output.axisStable, forKey: .axisStable)
        try container.encode(output.axisGeneration, forKey: .axisGeneration)
        try container.encode(output.angleMeasured, forKey: .angleMeasured)
        try container.encode(output.axisQuality, forKey: .axisQuality)
        try container.encode(output.rpm, forKey: .rpm)
        try container.encode(output.trackCount, forKey: .trackCount)
        try container.encode(output.inlierCount, forKey: .inlierCount)
        try container.encode(output.constraintCount, forKey: .constraintCount)
        try container.encode(output.processingMillis, forKey: .processingMillis)
        try container.encode(output.objectRadius, forKey: .objectRadius)
        try container.encode(output.heightMin, forKey: .heightMin)
        try container.encode(output.heightMax, forKey: .heightMax)
        try container.encode(output.angleUncertaintyDegrees, forKey: .angleUncertaintyDegrees)
        try container.encode(output.message, forKey: .message)
        try container.encode(
            output.tracks.map { Track(x: $0.x, y: $0.y, status: String(describing: $0.status)) }, forKey: .tracks)
        try container.encode(output.marker?.radius ?? 0, forKey: .roiRadius)
        try container.encode(output.angleDispersionDegrees, forKey: .dispersionDeg)
        try container.encode(output.relocalizerAnalysed, forKey: .relocAnalysed)
        try container.encode(output.relocalizerFill, forKey: .relocFill)
        try container.encode(output.lastRelocalizationAge, forKey: .relocAge)
        try container.encode(output.periodDegrees, forKey: .periodDeg)
        try container.encode(output.turnCoverageDegrees, forKey: .turnDeg)
        try container.encode(sequence, forKey: .sequence)
        try container.encode(config, forKey: .config)
        try container.encode(settings, forKey: .settings)
        try container.encode(droppedFrames, forKey: .droppedFrames)
        try container.encode(conversionFailures, forKey: .conversionFailures)
        try container.encode(processedFps, forKey: .processedFps)
        try container.encode(recorderMillis, forKey: .recorderMillis)
        try container.encode(pendingFrames, forKey: .pendingFrames)
        try container.encodeIfPresent(output.diagnostics, forKey: .diagnostics)
        if let marker = output.marker {
            try container.encode([marker.x, marker.y], forKey: .marker)
        }
        if let axis = output.axis {
            try container.encode([axis.origin.x, axis.origin.y, axis.origin.z], forKey: .axisOrigin)
            try container.encode([axis.direction.x, axis.direction.y, axis.direction.z], forKey: .axisDirection)
        }
    }
}
