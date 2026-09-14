import Foundation

#if os(Linux)
    import Glibc
#else
    import Darwin
#endif

/// Recompresses completed v2 recordings without changing their header, layout, or decoded bytes.
enum RecordingFileCompression {
    private static let maximumChunkBytes = 256 * 1024 * 1024

    enum Failure: LocalizedError {
        case invalidRecording
        case verificationFailed

        var errorDescription: String? {
            switch self {
            case .invalidRecording: "Unsupported, truncated, or malformed recording."
            case .verificationFailed: "Compressed recording did not preserve the original bytes."
            }
        }
    }

    /// The original stays in place until a complete, smaller file has been written and synced.
    static func recompress(at url: URL) throws {
        let input = try FileHandle(forReadingFrom: url)
        defer { try? input.close() }
        guard let header = try readChunk(input),
            let fields = try JSONSerialization.jsonObject(with: header) as? [String: Any],
            let version = fields["version"] as? Int
        else { throw Failure.invalidRecording }
        if version == 1 { return }
        guard version == 2,
            fields["chunks"] as? [String] == ["metadata", "luma", "depth", "confidence", "chromaRed", "chromaBlue"]
        else { throw Failure.invalidRecording }

        let temporary = url.deletingLastPathComponent().appendingPathComponent(".\(UUID().uuidString).wig.tmp")
        defer { try? FileManager.default.removeItem(at: temporary) }
        guard FileManager.default.createFile(atPath: temporary.path, contents: nil) else {
            throw CocoaError(.fileWriteUnknown)
        }
        let output = try FileHandle(forWritingTo: temporary)
        defer { try? output.close() }
        try writeChunk(header, to: output)
        var chunkCount = 0
        var savedBytes = 0
        while true {
            let saved: Int?
            #if canImport(ObjectiveC)
                saved = try autoreleasepool {
                    try recompressNextChunk(from: input, to: output, index: chunkCount)
                }
            #else
                saved = try recompressNextChunk(from: input, to: output, index: chunkCount)
            #endif
            guard let saved else { break }
            savedBytes += saved
            chunkCount += 1
        }
        guard chunkCount % 6 == 0 else { throw Failure.invalidRecording }
        try output.synchronize()
        try output.close()
        try input.close()
        guard savedBytes > 0 else { return }
        guard rename(temporary.path, url.path) == 0 else {
            throw POSIXError(POSIXErrorCode(rawValue: errno) ?? .EIO)
        }
    }

    private static func recompressNextChunk(from input: FileHandle, to output: FileHandle, index: Int) throws -> Int? {
        guard let chunk = try readChunk(input) else { return nil }
        if index % 6 == 0 && chunk.isEmpty { throw Failure.invalidRecording }
        let decoded = try RecordingChunkCodec.decode(chunk, maximumBytes: maximumChunkBytes)
        guard chunk.isEmpty || !decoded.isEmpty else { throw Failure.invalidRecording }
        let compressed = RecordingChunkCodec.encode(decoded, level: 9)
        guard compressed.count < chunk.count else {
            try writeChunk(chunk, to: output)
            return 0
        }
        guard try RecordingChunkCodec.decode(compressed, maximumBytes: maximumChunkBytes) == decoded else {
            throw Failure.verificationFailed
        }
        try writeChunk(compressed, to: output)
        return chunk.count - compressed.count
    }

    private static func readChunk(_ input: FileHandle) throws -> Data? {
        let prefix = try input.read(upToCount: 4) ?? Data()
        if prefix.isEmpty { return nil }
        guard prefix.count == 4 else { throw Failure.invalidRecording }
        let size = prefix.enumerated().reduce(0) { $0 | (Int($1.element) << (8 * $1.offset)) }
        guard size <= maximumChunkBytes else { throw Failure.invalidRecording }
        let data = try input.read(upToCount: size) ?? Data()
        guard data.count == size else { throw Failure.invalidRecording }
        return data
    }

    private static func writeChunk(_ data: Data, to output: FileHandle) throws {
        var size = UInt32(data.count).littleEndian
        try withUnsafeBytes(of: &size) { try output.write(contentsOf: $0) }
        try output.write(contentsOf: data)
    }
}
