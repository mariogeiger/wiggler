import CZlib
import Foundation
import WigglerCore

// wigreplay <recording.wig> [--diagnostics] [--harmonic <order> <rgba.bin>]
//
// Feeds every recorded frame to a fresh RotationEngine and prints one JSON line per frame: the engine's
// outputs plus the decisions the app does not record (events, axis estimation, recent-axis test). With
// `--diagnostics` the complete EngineDiagnostics of every frame is printed instead. The engine starts cold,
// so the first seconds differ from the app's warm run; everything after the first calibration is exact.
//
// `--harmonic` instead drives the app's HarmonicFit with the recorded luma and the *recorded* outputs (state,
// θ, angleMeasured, axisGeneration), exactly as the app fed it, and appends every rendered overlay
// (premultiplied RGBA8, width × height × 4 bytes, one per frame with a map) to the given file.

struct Failure: Error, CustomStringConvertible {
    let description: String
    init(_ d: String) { description = d }
}

/// Length-prefixed chunks, as written by SessionRecorder.
final class ChunkReader {
    private let handle: FileHandle
    init(path: String) throws {
        guard let h = FileHandle(forReadingAtPath: path) else { throw Failure("cannot open \(path)") }
        handle = h
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
        var out = Data()
        var buffer = [UInt8](repeating: 0, count: 1 << 20)
        try data.dropFirst().withUnsafeBytes { (source: UnsafeRawBufferPointer) in
            stream.next_in = UnsafeMutablePointer(mutating: source.bindMemory(to: Bytef.self).baseAddress!)
            stream.avail_in = uInt(source.count)
            while true {
                let result = buffer.withUnsafeMutableBufferPointer { dst -> Int32 in
                    stream.next_out = dst.baseAddress
                    stream.avail_out = uInt(dst.count)
                    return inflate(&stream, Z_NO_FLUSH)
                }
                out.append(contentsOf: buffer.prefix(buffer.count - Int(stream.avail_out)))
                if result == Z_STREAM_END { return }
                guard result == Z_OK else { throw Failure("inflate failed (\(result))") }
            }
        }
        return out
    }
}

/// The recorded fields needed to rebuild a FrameInput.
struct FrameMeta: Decodable {
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
}

/// The recorded configuration, when it still matches the engine's; otherwise the defaults are used.
struct ConfigMeta: Decodable {
    var config: EngineConfig
}

func floats(_ data: Data) -> [Float] {
    data.withUnsafeBytes { raw in
        (0..<(raw.count / 4)).map {
            Float(bitPattern: raw.loadUnaligned(fromByteOffset: 4 * $0, as: UInt32.self).littleEndian)
        }
    }
}

func plane(_ data: Data, width: Int, height: Int) -> GrayImage? {
    guard width > 0, height > 0 else { return nil }
    let p = floats(data)
    guard p.count == width * height else { return nil }
    return GrayImage(width: width, height: height, pixels: p)
}

let arguments = CommandLine.arguments
guard arguments.count >= 2 else {
    FileHandle.standardError.write("usage: wigreplay <recording.wig> [--diagnostics]\n".data(using: .utf8)!)
    exit(2)
}
let full = arguments.contains("--diagnostics")
var harmonicOrder: Int?
var harmonicOutput: FileHandle?
if let i = arguments.firstIndex(of: "--harmonic"), i + 2 < arguments.count, let order = Int(arguments[i + 1]) {
    harmonicOrder = order
    FileManager.default.createFile(atPath: arguments[i + 2], contents: nil)
    harmonicOutput = FileHandle(forWritingAtPath: arguments[i + 2])
}
var fit = HarmonicFit(signal: .luma, orders: [1, 2, 3])
let reader = try ChunkReader(path: arguments[1])
let decoder = JSONDecoder()
decoder.nonConformingFloatDecodingStrategy = .convertFromString(
    positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
let encoder = JSONEncoder()
encoder.nonConformingFloatEncodingStrategy = .convertToString(
    positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
guard let headerData = try reader.next(),
    let header = try JSONSerialization.jsonObject(with: headerData) as? [String: Any], header["version"] as? Int == 2
else { throw Failure("not a v2 recording") }

let engine = RotationEngine()
var configWarned = false
var line = [String: Any]()
while let metaData = try reader.next() {
    let metaJSON = try ChunkReader.decode(metaData)
    let meta = try decoder.decode(FrameMeta.self, from: metaJSON)
    if let recorded = try? decoder.decode(ConfigMeta.self, from: metaJSON) {
        engine.config = recorded.config
    } else if !configWarned {
        configWarned = true
        FileHandle.standardError.write(
            "recorded EngineConfig is from another version; replaying with defaults\n".data(using: .utf8)!)
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
        guard values.count == meta.depthWidth * meta.depthHeight else { throw Failure("depth size mismatch") }
        depth = DepthMap(
            width: meta.depthWidth, height: meta.depthHeight, depth: values,
            confidence: confidence.isEmpty ? nil : [UInt8](confidence))
    }
    let input = FrameInput(
        image: image, intrinsics: CameraIntrinsics(fx: meta.fx, fy: meta.fy, cx: meta.cx, cy: meta.cy),
        cameraToWorld: RigidTransform(
            rotation: M3(meta.rotation), translation: V3(meta.translation[0], meta.translation[1], meta.translation[2])),
        poseValid: meta.poseValid, depth: depth, timestamp: meta.t,
        chromaRed: plane(chromaRed, width: meta.chromaRedWidth, height: meta.chromaRedHeight),
        chromaBlue: plane(chromaBlue, width: meta.chromaBlueWidth, height: meta.chromaBlueHeight))
    if let order = harmonicOrder {
        var recorded = EngineOutput()
        recorded.state = EngineState(rawValue: meta.state) ?? .idle
        recorded.theta = meta.theta
        recorded.angleMeasured = meta.angleMeasured ?? true
        recorded.axisGeneration = meta.axisGeneration ?? 0
        fit.update(frame: input, output: recorded)
        if let map = fit.map, let rgba = map.render(order: order, theta: meta.theta, fullScale: 20 / 255) {
            harmonicOutput?.write(Data(rgba))
            print("\(meta.t) rendered")
        } else {
            print("\(meta.t) no map (progress \(fit.turnProgress))")
        }
        continue
    }
    let out = engine.process(input, captureDiagnostics: true)
    let d = out.diagnostics!
    if full {
        print(String(decoding: try encoder.encode(d), as: UTF8.self))
        continue
    }
    line = [
        "t": meta.t, "state": out.state.rawValue, "recordedState": meta.state,
        "theta": out.theta, "rpm": out.rpm, "axisStable": out.axisStable, "axisGeneration": out.axisGeneration,
        "angleMeasured": out.angleMeasured, "trackCount": out.trackCount, "inlierCount": out.inlierCount,
        "constraintCount": out.constraintCount, "processingMillis": out.processingMillis,
        "events": d.events.map { ["action": $0.action, "cause": $0.cause] },
    ]
    if let axis = out.axis {
        line["axisOrigin"] = [axis.origin.x, axis.origin.y, axis.origin.z]
        line["axisDirection"] = [axis.direction.x, axis.direction.y, axis.direction.z]
    }
    if let e = d.axisEstimation {
        line["axisEstimation"] = [
            "action": e.action, "consistent": e.consistent as Any, "count": e.estimate?.constraintCount as Any,
            "coverage": e.estimate?.coverage as Any, "inlierRatio": e.estimate?.inlierRatio as Any,
        ]
    }
    if let r = d.recentAxis {
        var recent: [String: Any] = ["contradictions": r.contradictions, "moved": r.moved, "agrees": r.agrees as Any]
        if let e = r.estimate {
            recent["count"] = e.constraintCount
            recent["coverage"] = e.coverage
            recent["wellConditioned"] = e.isWellConditioned
            recent["origin"] = [e.axis.origin.x, e.axis.origin.y, e.axis.origin.z]
        }
        line["recentAxis"] = recent
    }
    if let g = d.geometry {
        line["geometryHealthy"] = g.healthy
        line["disagreeStreak"] = g.disagreeStreak
    }
    let json = try JSONSerialization.data(withJSONObject: line.mapValues(finite), options: [.sortedKeys])
    print(String(decoding: json, as: UTF8.self))
}

/// JSONSerialization refuses NaN and infinities; the recorder's convention is their string names.
func finite(_ value: Any) -> Any {
    if let x = value as? Double, !x.isFinite { return x.isNaN ? "NaN" : (x > 0 ? "+Infinity" : "-Infinity") }
    if let a = value as? [Any] { return a.map(finite) }
    if let o = value as? [String: Any] { return o.mapValues(finite) }
    return value
}
