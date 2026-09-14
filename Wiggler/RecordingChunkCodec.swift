import Foundation

#if os(Linux)
    import CZlib
#else
    import zlib
#endif

/// A raw-deflate chunk with a one-byte codec tag; empty data denotes an absent plane.
enum RecordingChunkCodec {
    enum Failure: LocalizedError {
        case invalidChunk

        var errorDescription: String? { "Invalid or oversized recording chunk." }
    }

    static func encode(_ data: Data, level: Int32 = Z_BEST_SPEED) -> Data {
        guard !data.isEmpty else { return data }
        var size = compressBound(uLong(data.count))
        var output = Data(count: Int(size))
        let result = output.withUnsafeMutableBytes { destination in
            data.withUnsafeBytes { source in
                compress2(
                    destination.bindMemory(to: Bytef.self).baseAddress, &size,
                    source.bindMemory(to: Bytef.self).baseAddress, uLong(data.count), level)
            }
        }
        // compress2 adds a two-byte zlib header and a four-byte Adler-32 checksum around raw deflate.
        if result == Z_OK, size >= 6, Int(size) - 6 < data.count {
            return Data([1]) + output.subdata(in: 2..<(Int(size) - 4))
        }
        return Data([0]) + data
    }

    static func decode(_ data: Data, maximumBytes: Int) throws -> Data {
        guard let tag = data.first else { return data }
        if tag == 0 {
            guard data.count - 1 <= maximumBytes else { throw Failure.invalidChunk }
            return Data(data.dropFirst())
        }
        guard tag == 1 else { throw Failure.invalidChunk }
        var stream = z_stream()
        guard inflateInit2_(&stream, -MAX_WBITS, ZLIB_VERSION, Int32(MemoryLayout<z_stream>.size)) == Z_OK else {
            throw Failure.invalidChunk
        }
        defer { inflateEnd(&stream) }
        return try data.withUnsafeBytes { source in
            stream.next_in = UnsafeMutablePointer(mutating: source.bindMemory(to: Bytef.self).baseAddress! + 1)
            stream.avail_in = uInt(data.count - 1)
            var decoded = Data()
            var buffer = [UInt8](repeating: 0, count: 65_536)
            while true {
                let result = buffer.withUnsafeMutableBufferPointer { destination in
                    stream.next_out = destination.baseAddress
                    stream.avail_out = uInt(destination.count)
                    return inflate(&stream, Z_NO_FLUSH)
                }
                let count = buffer.count - Int(stream.avail_out)
                guard count <= maximumBytes - decoded.count else {
                    throw Failure.invalidChunk
                }
                decoded.append(contentsOf: buffer.prefix(count))
                if result == Z_STREAM_END {
                    guard stream.avail_in == 0 else { throw Failure.invalidChunk }
                    return decoded
                }
                guard result == Z_OK else { throw Failure.invalidChunk }
            }
        }
    }
}
