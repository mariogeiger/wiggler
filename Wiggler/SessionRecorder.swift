import Foundation
import WigglerCore

#if canImport(CryptoKit)
    import CryptoKit
#endif

/// Streams every processed input and its decision evidence to one file. Public calls belong on the engine queue.
/// Version 2: length-prefixed JSON header, then metadata and five image chunks per frame.
/// Each nonempty frame chunk starts with a codec byte (0 raw, 1 raw deflate). Numbers in planes are little-endian.
final class SessionRecorder {
    private let queue = DispatchQueue(label: "ch.mariogeiger.wiggler.recorder", qos: .utility)
    private let slots = DispatchSemaphore(value: 8)
    private let lock = NSLock()
    private let directory: URL
    /// Set before starting capture. Called on the writer queue on the first failure.
    var onFailure: ((String) -> Void)?
    // Writer queue only.
    private var handle: FileHandle?
    // Engine queue only.
    private(set) var url: URL?
    private var startTime: Double?
    private var sequence = 0
    // Protected by lock.
    private var snapshot = Status()

    struct Status {
        var recording = false
        var seconds = 0.0
        var megabytes = 0.0
        var frames = 0
        var fileName = ""
        var error: String?
    }

    init(directory: URL = SessionRecorder.documentsDirectory) { self.directory = directory }

    var isRecording: Bool { status(now: 0).recording }

    func status(now: Double) -> Status {
        lock.lock()
        defer { lock.unlock() }
        var result = snapshot
        result.seconds = startTime.map { max(0, now - $0) } ?? 0
        return result
    }

    static var documentsDirectory: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
    }

    static func buildIdentity() -> [String: Any] {
        let bundle = Bundle.main
        var result: [String: Any] = [
            "bundleIdentifier": bundle.bundleIdentifier ?? "unknown",
            "version": bundle.object(forInfoDictionaryKey: "CFBundleShortVersionString") ?? "unknown",
            "build": bundle.object(forInfoDictionaryKey: "CFBundleVersion") ?? "unknown",
            "os": ProcessInfo.processInfo.operatingSystemVersionString,
            "sourceRevision": "not embedded",
        ]
        #if canImport(CryptoKit)
            if let executable = bundle.executableURL,
                let data = try? Data(contentsOf: executable, options: .mappedIfSafe)
            {
                result["executableSHA256"] = SHA256.hash(data: data).map { String(format: "%02x", $0) }.joined()
            }
        #endif
        return result
    }

    func start(width: Int, height: Int, context: [String: Any]) {
        stop()
        url = nil
        sequence = 0
        startTime = nil
        lock.lock()
        snapshot = Status()
        lock.unlock()
        let f = DateFormatter()
        f.dateFormat = "yyyyMMdd-HHmmss"
        let sessionName = "wiggler-" + f.string(from: Date()) + "-" + UUID().uuidString
        let destination = directory.appendingPathComponent("\(sessionName).wig")
        let header: [String: Any] = [
            "version": 2, "width": width, "height": height, "session": sessionName,
            "createdAt": ISO8601DateFormatter().string(from: Date()),
            "chunks": ["metadata", "luma", "depth", "confidence", "chromaRed", "chromaBlue"],
            "chunkCodec": "byte tag: 0=raw, 1=raw-deflate; empty=absent",
            "compression": "automatic per chunk, stored raw only if compression fails or gives no size gain",
            "planeTypes": [
                "luma": "uint8", "depth": "float32-le", "confidence": "uint8",
                "chromaRed": "float32-le", "chromaBlue": "float32-le",
            ],
            "capture": "every processed frame; ARKit frames skipped while busy are counted",
            "initialState": "warm; first diagnostics.before describes the existing engine, not a restorable snapshot",
            "replayLimitations":
                "No pre-recording images, KLT pyramid, or appearance/harmonic library snapshot",
            "nonFiniteNumbers": "NaN, +Infinity, -Infinity strings in JSON; IEEE 754 in float planes",
            "maximumPendingFrames": 8, "backpressure": "wait; never discard a processed frame",
            "build": Self.buildIdentity(), "context": context,
        ]
        queue.sync {
            do {
                guard FileManager.default.createFile(atPath: destination.path, contents: nil) else {
                    throw CocoaError(.fileWriteUnknown)
                }
                handle = try FileHandle(forWritingTo: destination)
                url = destination
                try writeChunk(Self.jsonData(header))
                lock.lock()
                snapshot.recording = true
                snapshot.fileName = destination.lastPathComponent
                lock.unlock()
            } catch { fail(error) }
        }
    }

    func stop() {
        queue.sync {
            do { try handle?.close() } catch { fail(error) }
            handle = nil
            lock.lock()
            snapshot.recording = false
            lock.unlock()
        }
    }

    /// The eight-frame bound applies backpressure instead of growing memory or silently losing evidence.
    func append(
        input: FrameInput, luma8: [UInt8], output: EngineOutput, config: EngineConfig,
        settings: RecordingSettings, droppedFrames: Int, conversionFailures: Int, processedFps: Double
    ) {
        guard isRecording else { return }
        if startTime == nil { startTime = input.timestamp }
        let index = sequence
        sequence += 1
        slots.wait()
        queue.async { [self] in
            defer { slots.signal() }
            guard handle != nil else { return }
            do {
                let meta = RecordingFrameMetadata(
                    input: input, output: output, config: config, settings: settings, sequence: index,
                    droppedFrames: droppedFrames, conversionFailures: conversionFailures, processedFps: processedFps)
                let encoder = JSONEncoder()
                encoder.nonConformingFloatEncodingStrategy = .convertToString(
                    positiveInfinity: "+Infinity", negativeInfinity: "-Infinity", nan: "NaN")
                try Self.validatePlanes(input: input, luma8: luma8)
                try writeChunk(RecordingChunkCodec.encode(encoder.encode(meta)))
                try writeChunk(RecordingChunkCodec.encode(Data(luma8)))
                try writeChunk(RecordingChunkCodec.encode(Self.floatData(input.depth?.depth ?? [])))
                try writeChunk(RecordingChunkCodec.encode(Data(input.depth?.confidence ?? [])))
                try writeChunk(RecordingChunkCodec.encode(Self.floatData(input.chromaRed?.pixels ?? [])))
                try writeChunk(RecordingChunkCodec.encode(Self.floatData(input.chromaBlue?.pixels ?? [])))
                lock.lock()
                snapshot.frames += 1
                lock.unlock()
            } catch { fail(error) }
        }
    }

    private static func jsonData(_ value: [String: Any]) throws -> Data {
        try JSONSerialization.data(withJSONObject: finiteJSON(value), options: [.sortedKeys])
    }

    private static func finiteJSON(_ value: Any) -> Any {
        if let array = value as? [Any] { return array.map(finiteJSON) }
        if let object = value as? [String: Any] { return object.mapValues(finiteJSON) }
        if let number = value as? NSNumber, !number.doubleValue.isFinite {
            let x = number.doubleValue
            return x.isNaN ? "NaN" : (x > 0 ? "+Infinity" : "-Infinity")
        }
        return value
    }

    private static func validatePlanes(input: FrameInput, luma8: [UInt8]) throws {
        guard input.image.width > 0, input.image.height > 0,
            luma8.count == input.image.width * input.image.height
        else { throw CocoaError(.coderInvalidValue) }
        if let depth = input.depth {
            guard depth.width > 0, depth.height > 0, depth.depth.count == depth.width * depth.height,
                depth.confidence == nil || depth.confidence?.count == depth.depth.count
            else { throw CocoaError(.coderInvalidValue) }
        }
        for image in [input.chromaRed, input.chromaBlue].compactMap({ $0 }) {
            guard image.width > 0, image.height > 0, image.pixels.count == image.width * image.height else {
                throw CocoaError(.coderInvalidValue)
            }
        }
    }

    private static func floatData(_ values: [Float]) -> Data {
        values.map { $0.bitPattern.littleEndian }.withUnsafeBufferPointer { Data(buffer: $0) }
    }

    private func writeChunk(_ data: Data) throws {
        guard let handle, let count = UInt32(exactly: data.count) else { throw CocoaError(.fileWriteUnknown) }
        var length = count.littleEndian
        try handle.write(contentsOf: Data(bytes: &length, count: 4))
        try handle.write(contentsOf: data)
        lock.lock()
        snapshot.megabytes += Double(4 + data.count) / 1_048_576
        lock.unlock()
    }

    private func fail(_ error: Error) {
        try? handle?.close()
        handle = nil
        lock.lock()
        snapshot.recording = false
        let firstFailure = snapshot.error == nil
        if firstFailure { snapshot.error = error.localizedDescription }
        lock.unlock()
        if firstFailure { onFailure?(error.localizedDescription) }
    }

    static func recordings() -> [URL] {
        (try? FileManager.default.contentsOfDirectory(at: documentsDirectory, includingPropertiesForKeys: nil))?
            .filter { $0.pathExtension == "wig" }.sorted { $0.lastPathComponent < $1.lastPathComponent } ?? []
    }
}
