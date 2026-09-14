import Foundation

#if os(Linux)
    import CZlib
#else
    import zlib
#endif

/// A raw-deflate chunk with a one-byte codec tag; empty data denotes an absent plane.
enum RecordingChunkCodec {
    static func encode(_ data: Data) -> Data {
        guard !data.isEmpty else { return data }
        var size = compressBound(uLong(data.count))
        var output = Data(count: Int(size))
        let result = output.withUnsafeMutableBytes { destination in
            data.withUnsafeBytes { source in
                compress2(
                    destination.bindMemory(to: Bytef.self).baseAddress, &size,
                    source.bindMemory(to: Bytef.self).baseAddress, uLong(data.count), Z_BEST_SPEED)
            }
        }
        // compress2 adds a two-byte zlib header and a four-byte Adler-32 checksum around raw deflate.
        if result == Z_OK, size >= 6, Int(size) - 6 < data.count {
            return Data([1]) + output.subdata(in: 2..<(Int(size) - 4))
        }
        return Data([0]) + data
    }
}
