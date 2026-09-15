import Foundation

@testable import WigglerCore

func checkMetadataEncoding(input: FrameInput, diagnosticOutput: EngineOutput) throws {
    var output = diagnosticOutput
    output.state = .locked
    output.marker = Marker(x: 12.5, y: 8.25, radius: 16)
    output.axis = Axis(origin: V3(1, 2, 3), direction: V3(0, 1, 0))
    output.theta = 1.25
    output.angleDegrees = 90
    output.angleConfidence = 0.75
    output.axisStable = true
    output.axisQuality = 0.875
    output.rpm = .infinity
    output.periodDegrees = 180
    output.relocalizerFill = 0.5
    output.turnCoverageDegrees = 450
    output.objectRadius = 0.125
    output.heightMin = -0.25
    output.heightMax = 0.5
    output.tracks = [TrackDebug(x: 10.5, y: 11.25, status: .good)]
    output.trackCount = 75
    output.inlierCount = 65
    output.constraintCount = 900
    output.message = "quotes \" newline\n slash / Unicode é"
    output.processingMillis = 2.5
    output.angleDispersionDegrees = 0.125
    output.relocalizerAnalysed = true
    output.lastRelocalizationAge = 17
    output.angleUncertaintyDegrees = .nan
    output.diagnostics!.before.fusionSigma = -.infinity
    output.diagnostics!.initialContext!.trackHistories = (0..<75).map { id in
        EngineDiagnostics.TrackHistory(
            id: id, samples: (0..<90).map { EngineDiagnostics.Sample(frame: $0, point: V3(Double($0), 1, 2)) })
    }
    let settings = RecordingSettings(
        algorithm: .persistentMap, algorithmRevision: 7, harmonicSignal: "depth", harmonicOrder: nil, harmonicRevision: 5, inputRevision: 4,
        convertedSignal: "luma", harmonicInputAccepted: false, harmonicTurnProgress: .nan,
        cameraTrackingState: "limited")
    let config = EngineConfig()
    let encoder = JSONEncoder()
    encoder.nonConformingFloatEncodingStrategy = .convertToString(
        positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
    let bytes = try encoder.encode(
        RecordingFrameMetadata(
            input: input, output: output, config: config, settings: settings, sequence: 7,
            droppedFrames: 9, conversionFailures: 2, processedFps: 59.5))
    let actual = try JSONSerialization.jsonObject(with: bytes) as! [String: Any]
    let t = input.cameraToWorld.translation
    let expected: [String: Any] = [
        "t": input.timestamp, "width": input.image.width, "height": input.image.height,
        "fx": input.intrinsics.fx, "fy": input.intrinsics.fy, "cx": input.intrinsics.cx, "cy": input.intrinsics.cy,
        "rotation": input.cameraToWorld.rotation.m, "translation": [t.x, t.y, t.z], "poseValid": input.poseValid,
        "depthWidth": input.depth?.width ?? 0, "depthHeight": input.depth?.height ?? 0,
        "chromaRedWidth": input.chromaRed?.width ?? 0, "chromaRedHeight": input.chromaRed?.height ?? 0,
        "chromaBlueWidth": input.chromaBlue?.width ?? 0, "chromaBlueHeight": input.chromaBlue?.height ?? 0,
        "state": "locked", "theta": 1.25, "angleDegrees": 90, "angleConfidence": 0.75,
        "axisGeneration": 0, "angleMeasured": false,
        "axisStable": true, "axisQuality": 0.875, "rpm": "+Infinity", "periodDeg": 180,
        "relocFill": 0.5, "turnDeg": 450, "objectRadius": 0.125, "heightMin": -0.25, "heightMax": 0.5,
        "tracks": [["x": 10.5, "y": 11.25, "status": "good"]], "trackCount": 75,
        "inlierCount": 65, "constraintCount": 900, "message": output.message,
        "processingMillis": 2.5, "dispersionDeg": 0.125, "relocAnalysed": true, "relocAge": 17,
        "angleUncertaintyDegrees": "NaN", "marker": [12.5, 8.25], "roiRadius": 16,
        "axisOrigin": [1, 2, 3], "axisDirection": [0, 1, 0], "sequence": 7,
        "recorderMillis": 0, "pendingFrames": 0,
        "droppedFrames": 9, "conversionFailures": 2, "processedFps": 59.5,
        "config": try JSONSerialization.jsonObject(with: encoder.encode(config)),
        "diagnostics": try JSONSerialization.jsonObject(with: encoder.encode(output.diagnostics)),
        "settings": [
            "algorithm": "persistentMap", "algorithmRevision": 7,
            "harmonicSignal": "depth", "harmonicOrder": NSNull(), "harmonicRevision": 5, "inputRevision": 4,
            "convertedSignal": "luma", "harmonicInputAccepted": false, "harmonicTurnProgress": "NaN",
            "cameraTrackingState": "limited", "deliveryMillis": 0, "conversionMillis": 0,
        ],
    ]
    try check(Set(actual.keys) == Set(expected.keys), "metadata keys changed")
    for (key, value) in expected {
        let actualJSON = try JSONSerialization.data(withJSONObject: [key: actual[key]!], options: [.sortedKeys])
        let expectedJSON = try JSONSerialization.data(withJSONObject: [key: value], options: [.sortedKeys])
        try check(actualJSON == expectedJSON, "metadata value changed at \(key)")
    }
    let decoder = JSONDecoder()
    decoder.nonConformingFloatDecodingStrategy = .convertFromString(
        positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
    let diagnostics = try decoder.decode(
        EngineDiagnostics.self, from: JSONSerialization.data(withJSONObject: actual["diagnostics"]!))
    try check(diagnostics.initialContext!.trackHistories.count == 75, "warm histories lost")
    try check(diagnostics.initialContext!.trackHistories[74].samples.count == 90, "history samples lost")
    print("PASS: complete typed metadata schema, \(bytes.count)-byte warm history, nested nonfinite values")
}
