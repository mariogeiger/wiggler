import Foundation

func testFileCompression(in root: URL) throws {
    let directory = root.appendingPathComponent("compression-tests")
    try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: directory) }
    let url = directory.appendingPathComponent("recording.wig")
    func chunk(_ bytes: Data) -> Data {
        var size = UInt32(bytes.count).littleEndian
        return withUnsafeBytes(of: &size) { Data($0) } + bytes
    }
    let header = Data(
        #"{"version":2,"width":2,"height":2,"chunks":["metadata","luma","depth","confidence","chromaRed","chromaBlue"]}"#.utf8)
    let meta = Data(#"{"sequence":0,"unknownFutureField":["NaN","+Infinity","-Infinity",1.2345678901234567]}"#.utf8)
    // Distinct IEEE 754 bit patterns: signed zero, infinities, and NaN payloads must survive untouched.
    let bits: [UInt32] = [0, 0x8000_0000, 0x7f80_0000, 0xff80_0000, 0x7fc0_1234, 0x7fa0_5678, 0xffff_ffff]
    var floats = Data()
    for _ in 0..<4096 {
        for bit in bits {
            var little = bit.littleEndian
            floats.append(withUnsafeBytes(of: &little) { Data($0) })
        }
    }
    let planes = [meta, Data([0, 1, 254, 255]), floats, Data(), floats, Data()]
    let original = chunk(header) + planes.reduce(Data()) { $0 + chunk($1.isEmpty ? Data() : Data([0]) + $1) }
    try original.write(to: url)
    try RecordingFileCompression.recompress(at: url)
    let compressed = try Data(contentsOf: url)
    try check(compressed.count < original.count / 10, "export failed to compress raw chunks")
    var offset = 0
    func nextChunk() -> Data {
        let prefix = compressed[offset..<(offset + 4)]
        let size = prefix.enumerated().reduce(0) { $0 | (Int($1.element) << (8 * $1.offset)) }
        offset += 4
        defer { offset += size }
        return compressed.subdata(in: offset..<(offset + size))
    }
    try check(nextChunk() == header, "header bytes changed")
    for plane in planes {
        try check(try RecordingChunkCodec.decode(nextChunk(), maximumBytes: 256 * 1024 * 1024) == plane, "plane bytes changed")
    }
    try check(offset == compressed.count, "export gained trailing bytes")
    try RecordingFileCompression.recompress(at: url)
    try check(try Data(contentsOf: url) == compressed, "repeated export changed or grew the file")

    let legacy = chunk(Data(#"{"version":1,"width":2,"height":2}"#.utf8)) + Data([42, 0, 1, 2])
    try legacy.write(to: url)
    try RecordingFileCompression.recompress(at: url)
    try check(try Data(contentsOf: url) == legacy, "legacy recording was changed")

    let fast = RecordingChunkCodec.encode(floats)
    let dense = RecordingChunkCodec.encode(floats, level: 9)
    try check(dense.count < fast.count, "export level did not improve recording compression")
    try check(try RecordingChunkCodec.decode(fast, maximumBytes: floats.count) == floats, "level-1 decode failed")
    let chunks = planes.map { $0.isEmpty ? Data() : Data([0]) + $0 }
    let validPrefix = chunk(header) + chunk(chunks[0])
    let invalidFiles = [
        Data(), Data([1, 2, 3]), chunk(Data("not json".utf8)),
        chunk(Data(#"{"version":3}"#.utf8)), chunk(Data(#"{"version":2,"chunks":[]}"#.utf8)),
        chunk(header) + Data([1]), validPrefix, original.dropLast(), original + Data([1]),
        validPrefix + chunk(Data([9, 0])), validPrefix + chunk(Data([1, 255])),
        validPrefix + chunk(Data([0])), validPrefix + chunk(Data([1, 3, 0])),
        validPrefix + chunk(dense + Data([0])), validPrefix + chunk(Data(dense.dropLast())),
        chunk(header) + chunk(Data()), validPrefix + Data([1, 0, 0, 16]),
    ]
    for invalid in invalidFiles {
        try invalid.write(to: url)
        var failed = false
        do { try RecordingFileCompression.recompress(at: url) } catch { failed = true }
        try check(failed, "malformed recording accepted")
        try check(try Data(contentsOf: url) == invalid, "failed compression overwrote the original")
        try check(try FileManager.default.contentsOfDirectory(atPath: directory.path) == ["recording.wig"], "temporary file leaked")
    }
    for encoded in [dense, Data([0]) + floats] {
        var failed = false
        do { _ = try RecordingChunkCodec.decode(encoded, maximumBytes: floats.count - 1) } catch { failed = true }
        try check(failed, "decode exceeded output bound")
    }
    try original.write(to: url)
    try FileManager.default.setAttributes([.posixPermissions: 0o500], ofItemAtPath: directory.path)
    defer { try? FileManager.default.setAttributes([.posixPermissions: 0o700], ofItemAtPath: directory.path) }
    var writeFailed = false
    do { try RecordingFileCompression.recompress(at: url) } catch { writeFailed = true }
    try check(writeFailed, "unwritable export directory was accepted")
    try check(try Data(contentsOf: url) == original, "write failure changed the original")
    print("PASS: lossless export, exact header/IEEE bytes, raw/deflate/empty chunks, legacy passthrough, malformed/write rollback, bounds and idempotence")
}
