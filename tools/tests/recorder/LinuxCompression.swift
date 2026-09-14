#if os(Linux)
    import CZlib
    import Foundation

    enum NSDataCompression { case zlib }
    enum CompressionUnavailable: Error { case unavailable }
    extension NSData {
        /// Exercises the raw fallback or a zlib-produced raw deflate stream; not Apple's compression implementation.
        func compressed(using: NSDataCompression) throws -> Data {
            guard ProcessInfo.processInfo.environment["WIG_TEST_COMPRESSION"] == "deflate" else {
                throw CompressionUnavailable.unavailable
            }
            var size = compressBound(uLong(length))
            var output = Data(count: Int(size))
            let result = output.withUnsafeMutableBytes {
                compress2(
                    $0.bindMemory(to: Bytef.self).baseAddress, &size, bytes.assumingMemoryBound(to: Bytef.self),
                    uLong(length), Z_DEFAULT_COMPRESSION)
            }
            guard result == Z_OK else { throw CompressionUnavailable.unavailable }
            // compress2 wraps deflate with a two-byte zlib header and a four-byte Adler-32 checksum.
            return output.subdata(in: 2..<(Int(size) - 4))
        }
    }
#endif
