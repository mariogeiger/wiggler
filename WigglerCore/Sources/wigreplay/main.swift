import CZlib
import Foundation
import WigglerCore

// wigreplay <recording.wig> [--diagnostics] [--algorithm trackedGeometry|persistentMap]
// wigreplay <recording.wig> --harmonic <order> <rgba.bin>
//
// Normal replay starts a cold estimator and follows every recorded algorithm selection. `--algorithm` instead
// runs each recorded algorithm segment through one chosen estimator, resetting it at every recorded boundary.
// Cold replay cannot restore the app's prior estimator history, so its output can differ throughout a recording.
//
// `--harmonic` does not run an estimator. It drives HarmonicFit from the recorded outputs and luma, resetting
// the fit at every recorded algorithm boundary. It appends each rendered premultiplied RGBA8 overlay to the
// given file.

struct Failure: Error, CustomStringConvertible {
    let description: String
    init(_ description: String) { self.description = description }
}

private let usage = """
    usage: wigreplay <recording.wig> [--diagnostics] [--algorithm trackedGeometry|persistentMap]
           wigreplay <recording.wig> --harmonic <order> <rgba.bin>
    """

private struct ReplayOptions {
    var recordingPath: String
    var fullDiagnostics = false
    var harmonic: (order: Int, path: String)?
    var algorithmOverride: RotationAlgorithm?

    static func parse(_ arguments: [String]) throws -> ReplayOptions {
        var recordingPath: String?
        var fullDiagnostics = false
        var harmonic: (order: Int, path: String)?
        var algorithmOverride: RotationAlgorithm?
        var index = 0
        while index < arguments.count {
            switch arguments[index] {
            case "--diagnostics":
                guard !fullDiagnostics else { throw Failure("--diagnostics specified more than once") }
                fullDiagnostics = true
                index += 1
            case "--algorithm":
                guard algorithmOverride == nil else { throw Failure("--algorithm specified more than once") }
                guard index + 1 < arguments.count else { throw Failure("--algorithm needs a value") }
                let rawValue = arguments[index + 1]
                guard let algorithm = RotationAlgorithm(rawValue: rawValue) else {
                    throw Failure("unknown algorithm '\(rawValue)'")
                }
                algorithmOverride = algorithm
                index += 2
            case "--harmonic":
                guard harmonic == nil else { throw Failure("--harmonic specified more than once") }
                guard index + 2 < arguments.count, let order = Int(arguments[index + 1]), (1...3).contains(order)
                else { throw Failure("--harmonic needs an order from 1 through 3 and an output path") }
                harmonic = (order, arguments[index + 2])
                index += 3
            default:
                let argument = arguments[index]
                guard !argument.hasPrefix("-") else { throw Failure("unknown option '\(argument)'") }
                guard recordingPath == nil else { throw Failure("more than one recording path specified") }
                recordingPath = argument
                index += 1
            }
        }
        guard let recordingPath else { throw Failure("recording path is required") }
        guard harmonic == nil || !fullDiagnostics else {
            throw Failure("--diagnostics cannot be combined with --harmonic")
        }
        guard harmonic == nil || algorithmOverride == nil else {
            throw Failure("--algorithm cannot be combined with --harmonic; harmonic replay uses recorded outputs")
        }
        return ReplayOptions(
            recordingPath: recordingPath, fullDiagnostics: fullDiagnostics, harmonic: harmonic,
            algorithmOverride: algorithmOverride)
    }
}

private struct AlgorithmSelection: Equatable {
    var algorithm: RotationAlgorithm
    var revision: Int
}

/// Length-prefixed chunks, as written by SessionRecorder.
final class ChunkReader {
    private let handle: FileHandle

    init(path: String) throws {
        guard let handle = FileHandle(forReadingAtPath: path) else { throw Failure("cannot open \(path)") }
        self.handle = handle
    }

    func next() throws -> Data? {
        let prefix = handle.readData(ofLength: 4)
        if prefix.isEmpty { return nil }
        guard prefix.count == 4 else { throw Failure("truncated chunk length") }
        let length = prefix.withUnsafeBytes { Int($0.loadUnaligned(as: UInt32.self).littleEndian) }
        let data = handle.readData(ofLength: length)
        guard data.count == length else { throw Failure("truncated chunk") }
        return data
    }

    /// Codec byte 0 = raw, 1 = raw deflate; empty = absent plane.
    static func decode(_ data: Data) throws -> Data {
        guard let tag = data.first else { return data }
        if tag == 0 { return data.dropFirst() }
        guard tag == 1 else { throw Failure("unknown chunk codec \(tag)") }
        var stream = z_stream()
        guard inflateInit2_(&stream, -MAX_WBITS, ZLIB_VERSION, Int32(MemoryLayout<z_stream>.size)) == Z_OK else {
            throw Failure("inflateInit failed")
        }
        defer { inflateEnd(&stream) }
        var output = Data()
        var buffer = [UInt8](repeating: 0, count: 1 << 20)
        try data.dropFirst().withUnsafeBytes { (source: UnsafeRawBufferPointer) in
            stream.next_in = UnsafeMutablePointer(mutating: source.bindMemory(to: Bytef.self).baseAddress!)
            stream.avail_in = uInt(source.count)
            while true {
                let result = buffer.withUnsafeMutableBufferPointer { destination -> Int32 in
                    stream.next_out = destination.baseAddress
                    stream.avail_out = uInt(destination.count)
                    return inflate(&stream, Z_NO_FLUSH)
                }
                output.append(contentsOf: buffer.prefix(buffer.count - Int(stream.avail_out)))
                if result == Z_STREAM_END { return }
                guard result == Z_OK else { throw Failure("inflate failed (\(result))") }
            }
        }
        return output
    }
}

/// The recorded fields needed to rebuild a FrameInput.
private struct FrameMeta: Decodable {
    struct Settings: Decodable {
        var algorithm: String?
        var algorithmRevision: Int?
    }

    var t: Double
    var width: Int, height: Int
    var fx: Double, fy: Double, cx: Double, cy: Double
    var rotation: [Double], translation: [Double]
    var poseValid: Bool
    var depthWidth: Int, depthHeight: Int
    var chromaRedWidth: Int, chromaRedHeight: Int, chromaBlueWidth: Int, chromaBlueHeight: Int
    var state: String
    var theta: Double
    var angleMeasured: Bool?
    var axisGeneration: Int?
    var settings: Settings?

    func algorithmSelection() throws -> AlgorithmSelection {
        guard let rawValue = settings?.algorithm else {
            return AlgorithmSelection(
                algorithm: .trackedGeometry, revision: settings?.algorithmRevision ?? 0)
        }
        guard let algorithm = RotationAlgorithm(rawValue: rawValue) else {
            throw Failure("unknown recorded algorithm '\(rawValue)' at t=\(t)")
        }
        return AlgorithmSelection(algorithm: algorithm, revision: settings?.algorithmRevision ?? 0)
    }
}

/// The recorded configuration, when it still matches the estimator's; otherwise defaults are used.
private struct ConfigMeta: Decodable {
    var config: EngineConfig
}

private struct DiagnosticsEnvelope<Diagnostics: Encodable>: Encodable {
    var algorithm: RotationAlgorithm
    var algorithmRevision: Int
    var diagnostics: Diagnostics?
}

private func floats(_ data: Data) -> [Float] {
    data.withUnsafeBytes { raw in
        (0..<(raw.count / 4)).map {
            Float(bitPattern: raw.loadUnaligned(fromByteOffset: 4 * $0, as: UInt32.self).littleEndian)
        }
    }
}

private func plane(_ data: Data, width: Int, height: Int) -> GrayImage? {
    guard width > 0, height > 0 else { return nil }
    let pixels = floats(data)
    guard pixels.count == width * height else { return nil }
    return GrayImage(width: width, height: height, pixels: pixels)
}

private func writeError(_ message: String) {
    FileHandle.standardError.write(Data(message.utf8))
}

private func run(_ options: ReplayOptions) throws {
    let reader = try ChunkReader(path: options.recordingPath)
    let decoder = JSONDecoder()
    decoder.nonConformingFloatDecodingStrategy = .convertFromString(
        positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
    let encoder = JSONEncoder()
    encoder.nonConformingFloatEncodingStrategy = .convertToString(
        positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
    guard let headerData = try reader.next(),
        let header = try JSONSerialization.jsonObject(with: headerData) as? [String: Any],
        header["version"] as? Int == 2
    else { throw Failure("not a v2 recording") }

    var harmonicOutput: FileHandle?
    if let harmonic = options.harmonic {
        guard FileManager.default.createFile(atPath: harmonic.path, contents: nil),
            let output = FileHandle(forWritingAtPath: harmonic.path)
        else { throw Failure("cannot create \(harmonic.path)") }
        harmonicOutput = output
    }
    defer { try? harmonicOutput?.close() }

    var estimator: RotationEstimator?
    var harmonicFit = HarmonicFit(signal: .luma, orders: [1, 2, 3])
    var previousRecordedSelection: AlgorithmSelection?
    var overrideRevision = 0
    var configWarned = false

    while let metadata = try reader.next() {
        let metadataJSON = try ChunkReader.decode(metadata)
        let meta = try decoder.decode(FrameMeta.self, from: metadataJSON)
        let recordedSelection = try meta.algorithmSelection()
        let selectionChanged = previousRecordedSelection.map { $0 != recordedSelection } ?? false
        if selectionChanged { overrideRevision += 1 }
        let effectiveSelection = AlgorithmSelection(
            algorithm: options.algorithmOverride ?? recordedSelection.algorithm,
            revision: options.algorithmOverride == nil ? recordedSelection.revision : overrideRevision)

        if options.harmonic != nil, selectionChanged {
            harmonicFit = HarmonicFit(signal: .luma, orders: [1, 2, 3])
        }
        previousRecordedSelection = recordedSelection

        let recordedConfig: EngineConfig?
        if options.harmonic == nil, options.algorithmOverride == nil {
            if let recorded = try? decoder.decode(ConfigMeta.self, from: metadataJSON) {
                recordedConfig = recorded.config
            } else {
                recordedConfig = nil
                if !configWarned {
                    configWarned = true
                    writeError(
                        "recorded EngineConfig is from another version; using algorithm defaults for affected segments\n"
                    )
                }
            }
        } else {
            recordedConfig = nil
        }

        let activeEstimator: RotationEstimator?
        if options.harmonic == nil {
            if let estimator {
                estimator.select(
                    effectiveSelection.algorithm, revision: effectiveSelection.revision,
                    config: selectionChanged ? recordedConfig : nil)
                activeEstimator = estimator
            } else {
                let created = RotationEstimator(
                    algorithm: effectiveSelection.algorithm, revision: effectiveSelection.revision,
                    config: recordedConfig)
                estimator = created
                activeEstimator = created
            }
            if let recordedConfig { activeEstimator?.config = recordedConfig }
        } else {
            activeEstimator = nil
        }

        guard let luma = try reader.next().map(ChunkReader.decode),
            let depthData = try reader.next().map(ChunkReader.decode),
            let confidence = try reader.next().map(ChunkReader.decode),
            let chromaRed = try reader.next().map(ChunkReader.decode),
            let chromaBlue = try reader.next().map(ChunkReader.decode)
        else { throw Failure("truncated frame") }
        guard luma.count == meta.width * meta.height else { throw Failure("luma size mismatch at t=\(meta.t)") }
        let image = luma.withUnsafeBytes {
            GrayImage(
                width: meta.width, height: meta.height, luma8: $0.bindMemory(to: UInt8.self).baseAddress!,
                bytesPerRow: meta.width)
        }
        var depth: DepthMap?
        if meta.depthWidth > 0 {
            let values = floats(depthData)
            guard values.count == meta.depthWidth * meta.depthHeight else {
                throw Failure("depth size mismatch at t=\(meta.t)")
            }
            depth = DepthMap(
                width: meta.depthWidth, height: meta.depthHeight, depth: values,
                confidence: confidence.isEmpty ? nil : [UInt8](confidence))
        }
        guard meta.rotation.count == 9, meta.translation.count == 3 else {
            throw Failure("invalid camera transform at t=\(meta.t)")
        }
        let input = FrameInput(
            image: image, intrinsics: CameraIntrinsics(fx: meta.fx, fy: meta.fy, cx: meta.cx, cy: meta.cy),
            cameraToWorld: RigidTransform(
                rotation: M3(meta.rotation),
                translation: V3(meta.translation[0], meta.translation[1], meta.translation[2])),
            poseValid: meta.poseValid, depth: depth, timestamp: meta.t,
            chromaRed: plane(chromaRed, width: meta.chromaRedWidth, height: meta.chromaRedHeight),
            chromaBlue: plane(chromaBlue, width: meta.chromaBlueWidth, height: meta.chromaBlueHeight))

        if let harmonic = options.harmonic {
            var recorded = EngineOutput()
            recorded.state = EngineState(rawValue: meta.state) ?? .idle
            recorded.theta = meta.theta
            recorded.angleMeasured = meta.angleMeasured ?? true
            recorded.axisGeneration = meta.axisGeneration ?? 0
            harmonicFit.update(frame: input, output: recorded)
            if let map = harmonicFit.map,
                let rgba = map.render(order: harmonic.order, theta: meta.theta, fullScale: 20 / 255)
            {
                harmonicOutput?.write(Data(rgba))
                print("\(meta.t) rendered")
            } else {
                print("\(meta.t) no map (progress \(harmonicFit.turnProgress))")
            }
            continue
        }

        guard let activeEstimator else { throw Failure("internal error: replay estimator was not created") }
        let output = activeEstimator.process(input, captureDiagnostics: true)
        if options.fullDiagnostics {
            if effectiveSelection.algorithm == .trackedGeometry {
                guard let diagnostics = output.diagnostics else {
                    throw Failure("tracked geometry diagnostics missing at t=\(meta.t)")
                }
                let encoded = try encoder.encode(diagnostics)
                guard var journal = try JSONSerialization.jsonObject(with: encoded) as? [String: Any] else {
                    throw Failure("cannot encode tracked geometry diagnostics at t=\(meta.t)")
                }
                journal["algorithm"] = effectiveSelection.algorithm.rawValue
                journal["algorithmRevision"] = effectiveSelection.revision
                let json = try JSONSerialization.data(withJSONObject: journal, options: [.sortedKeys])
                print(String(decoding: json, as: UTF8.self))
            } else {
                let envelope = DiagnosticsEnvelope(
                    algorithm: effectiveSelection.algorithm, algorithmRevision: effectiveSelection.revision,
                    diagnostics: output.persistentDiagnostics)
                print(String(decoding: try encoder.encode(envelope), as: UTF8.self))
            }
            continue
        }

        var line: [String: Any] = [
            "t": meta.t,
            "algorithm": effectiveSelection.algorithm.rawValue,
            "algorithmRevision": effectiveSelection.revision,
            "recordedAlgorithm": recordedSelection.algorithm.rawValue,
            "recordedAlgorithmRevision": recordedSelection.revision,
            "state": output.state.rawValue,
            "recordedState": meta.state,
            "theta": output.theta,
            "rpm": output.rpm,
            "axisStable": output.axisStable,
            "axisGeneration": output.axisGeneration,
            "angleMeasured": output.angleMeasured,
            "trackCount": output.trackCount,
            "inlierCount": output.inlierCount,
            "constraintCount": output.constraintCount,
            "processingMillis": output.processingMillis,
        ]
        if let axis = output.axis {
            line["axisOrigin"] = [axis.origin.x, axis.origin.y, axis.origin.z]
            line["axisDirection"] = [axis.direction.x, axis.direction.y, axis.direction.z]
        }
        if let diagnostics = output.diagnostics {
            line["events"] = diagnostics.events.map { ["action": $0.action, "cause": $0.cause] }
            if let estimation = diagnostics.axisEstimation {
                line["axisEstimation"] = [
                    "action": estimation.action,
                    "consistent": estimation.consistent as Any,
                    "count": estimation.estimate?.constraintCount as Any,
                    "coverage": estimation.estimate?.coverage as Any,
                    "inlierRatio": estimation.estimate?.inlierRatio as Any,
                ]
            }
            if let recentAxis = diagnostics.recentAxis {
                var recent: [String: Any] = [
                    "contradictions": recentAxis.contradictions,
                    "moved": recentAxis.moved,
                    "agrees": recentAxis.agrees as Any,
                ]
                if let estimate = recentAxis.estimate {
                    recent["count"] = estimate.constraintCount
                    recent["coverage"] = estimate.coverage
                    recent["wellConditioned"] = estimate.isWellConditioned
                    recent["origin"] = [estimate.axis.origin.x, estimate.axis.origin.y, estimate.axis.origin.z]
                }
                line["recentAxis"] = recent
            }
            if let geometry = diagnostics.geometry {
                line["rotationDelta"] = geometry.rotationDelta
                line["rotationSupport"] = geometry.rotationSupport
                line["backgroundCount"] = geometry.backgroundIDs?.count
                line["geometryHealthy"] = geometry.healthy
                line["disagreeStreak"] = geometry.disagreeStreak
            }
        }
        if let diagnostics = output.persistentDiagnostics {
            let encoded = try encoder.encode(diagnostics)
            line["persistentMapDiagnostics"] = try JSONSerialization.jsonObject(with: encoded)
        }
        let json = try JSONSerialization.data(withJSONObject: line.mapValues(finite), options: [.sortedKeys])
        print(String(decoding: json, as: UTF8.self))
    }
}

/// JSONSerialization refuses NaN and infinities; the recorder's convention is their string names.
private func finite(_ value: Any) -> Any {
    if let number = value as? Double, !number.isFinite {
        return number.isNaN ? "NaN" : (number > 0 ? "+Infinity" : "-Infinity")
    }
    if let values = value as? [Any] { return values.map(finite) }
    if let object = value as? [String: Any] { return object.mapValues(finite) }
    return value
}

private let options: ReplayOptions
do {
    options = try ReplayOptions.parse(Array(CommandLine.arguments.dropFirst()))
} catch {
    writeError("wigreplay: \(error)\n")
    writeError(usage + "\n")
    exit(2)
}

do {
    try run(options)
} catch {
    writeError("wigreplay: \(error)\n")
    exit(1)
}
