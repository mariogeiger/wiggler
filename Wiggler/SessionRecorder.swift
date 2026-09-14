import Foundation
import WigglerCore

/// Records sampled inputs the engine sees (downscaled luma, LiDAR depth + confidence, intrinsics, pose) plus the
/// engine's outputs, so a session can be replayed and studied offline.
///
/// File format (`.wig`), all integers little-endian:
///   header: UInt32 length + JSON  {"version":1,"width":480,"height":360,...}
///   frames: UInt32 len + JSON meta | UInt32 len + deflate(luma UInt8[w*h]) | UInt32 len + deflate(depth Float32[dw*dh])
///           | UInt32 len + deflate(confidence UInt8[dw*dh])   (len 0 when absent)
final class SessionRecorder {
    private let queue = DispatchQueue(label: "ch.mariogeiger.wiggler.recorder", qos: .utility)
    private var handle: FileHandle?
    private(set) var url: URL?
    private var startTime: Double?
    private var written: Int64 = 0
    private var frames = 0

    var isRecording: Bool { handle != nil }

    struct Status {
        var recording: Bool
        var seconds: Double
        var megabytes: Double
        var frames: Int
        var fileName: String
    }

    func status(now: Double) -> Status {
        Status(
            recording: isRecording, seconds: startTime.map { now - $0 } ?? 0,
            megabytes: Double(written) / 1_048_576, frames: frames, fileName: url?.lastPathComponent ?? "")
    }

    static var documentsDirectory: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
    }

    func start() {
        stop()
        url = nil
        written = 0
        frames = 0
        startTime = nil
        let f = DateFormatter()
        f.dateFormat = "yyyyMMdd-HHmmss"
        let sessionName = "wiggler-" + f.string(from: Date())
        let u = Self.documentsDirectory.appendingPathComponent("\(sessionName).wig")
        guard FileManager.default.createFile(atPath: u.path, contents: nil),
            let h = try? FileHandle(forWritingTo: u)
        else { return }
        handle = h
        url = u
        let header: [String: Any] = [
            "version": 1, "width": FrameConverter.engineWidth, "height": FrameConverter.engineHeight,
            "session": sessionName,
        ]
        writeChunk(try! JSONSerialization.data(withJSONObject: header))
    }

    func stop() {
        queue.sync {
            handle?.closeFile()
            handle = nil
        }
    }

    /// Called on the frame queue; copies what it needs and returns immediately.
    func append(
        input: FrameInput, luma8: [UInt8], output: EngineOutput, marker: CGPoint?, roiRadius: Float,
        droppedFrames: Int = 0, processedFps: Double = 0
    ) {
        guard handle != nil else { return }
        if startTime == nil { startTime = input.timestamp }
        let pose = input.cameraToWorld
        let r = pose.rotation.m, t = pose.translation
        var meta: [String: Any] = [
            "t": input.timestamp,
            "fx": input.intrinsics.fx, "fy": input.intrinsics.fy, "cx": input.intrinsics.cx, "cy": input.intrinsics.cy,
            "rotation": r, "translation": [t.x, t.y, t.z], "poseValid": input.poseValid,
            "depthWidth": input.depth?.width ?? 0, "depthHeight": input.depth?.height ?? 0,
            "state": output.state.rawValue, "theta": output.theta, "angleConfidence": output.angleConfidence,
            "axisQuality": output.axisQuality, "rpm": output.rpm, "trackCount": output.trackCount,
            "inlierCount": output.inlierCount, "processingMillis": output.processingMillis,
            "roiRadius": roiRadius,
            "dispersionDeg": output.angleDispersionDegrees, "relocAnalysed": output.relocalizerAnalysed,
            "relocFill": output.relocalizerFill, "relocAge": output.lastRelocalizationAge,
            "periodDeg": output.periodDegrees, "turnDeg": output.turnCoverageDegrees,
            "objectRadius": output.objectRadius, "droppedFrames": droppedFrames, "processedFps": processedFps,
        ]
        if let m = marker { meta["marker"] = [m.x, m.y] }
        if let a = output.axis {
            meta["axisOrigin"] = [a.origin.x, a.origin.y, a.origin.z]
            meta["axisDirection"] = [a.direction.x, a.direction.y, a.direction.z]
        }
        let metaData = (try? JSONSerialization.data(withJSONObject: meta)) ?? Data()
        let lumaData = Data(luma8)
        let depthData = input.depth.map { d in d.depth.withUnsafeBufferPointer { Data(buffer: $0) } } ?? Data()
        let confData = input.depth?.confidence.map { Data($0) } ?? Data()
        queue.async { [self] in
            guard handle != nil else { return }
            writeChunk(metaData)
            writeChunk(Self.compress(lumaData))
            writeChunk(Self.compress(depthData))
            writeChunk(Self.compress(confData))
            frames += 1
        }
    }

    private static func compress(_ d: Data) -> Data {
        if d.isEmpty { return d }
        // Raw deflate stream (no zlib header): Python reads it with zlib.decompress(data, -15).
        return (try? (d as NSData).compressed(using: .zlib) as Data) ?? d
    }

    private func writeChunk(_ d: Data) {
        guard let h = handle else { return }
        var len = UInt32(d.count).littleEndian
        h.write(Data(bytes: &len, count: 4))
        h.write(d)
        written += Int64(4 + d.count)
    }

    static func recordings() -> [URL] {
        (try? FileManager.default.contentsOfDirectory(at: documentsDirectory, includingPropertiesForKeys: nil))?
            .filter { $0.pathExtension == "wig" }.sorted { $0.lastPathComponent < $1.lastPathComponent } ?? []
    }
}
