import Foundation
import WigglerCore

enum TestFailure: Error { case failed(String) }
func check(_ condition: Bool, _ message: String) throws {
    if !condition { throw TestFailure.failed(message) }
}

let directory = URL(fileURLWithPath: CommandLine.arguments[1], isDirectory: true)
try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
let recorder = SessionRecorder(directory: directory)
let engine = RotationEngine()
let intrinsics = CameraIntrinsics(fx: 40, fy: 40, cx: 24, cy: 18)
let smallPixels = (0..<(48 * 36)).map { UInt8($0 % 251) }
let image = smallPixels.withUnsafeBufferPointer {
    GrayImage(width: 48, height: 36, luma8: $0.baseAddress!, bytesPerRow: 48)
}
func frame(_ index: Int) -> FrameInput {
    var input = FrameInput(
        image: image, intrinsics: intrinsics, cameraToWorld: .identity, poseValid: true,
        depth: nil, timestamp: Double(index) / 20)
    if index % 3 != 0 {
        input.depth = DepthMap(width: 2, height: 1, depth: [1, .nan], confidence: index % 3 == 1 ? [1, 2] : nil)
    }
    if index % 3 == 1 {
        input.chromaRed = GrayImage(width: 1, height: 2, pixels: [-0.25, 0.5])
    } else if index % 3 == 2 {
        input.chromaBlue = GrayImage(width: 3, height: 1, pixels: [0.25, -0.125, 0])
    }
    return input
}
for index in 0..<5 { _ = engine.process(frame(index)) }
recorder.start(width: 48, height: 36, context: ["test": "warm recording"])
try check(recorder.isRecording, "start failed")
let firstURL = recorder.url!
for index in 5..<29 {
    let input = frame(index)
    engine.config.targetTrackCount = index < 17 ? 60 : 65
    var output = engine.process(input, captureDiagnostics: true)
    output.rpm = index == 5 ? .infinity : Double(index)
    recorder.append(
        input: input, luma8: smallPixels, output: output, config: engine.config,
        settings: [
            "harmonicSignal": index % 3 == 1 ? "chromaRed" : (index % 3 == 2 ? "chromaBlue" : "luma"),
            "harmonicOrder": index < 17 ? NSNull() as Any : 2 as Any,
        ],
        droppedFrames: index, conversionFailures: 2, processedFps: 20)
}
// Starting again must drain the old session and never reuse its path, even within one second.
recorder.start(width: 480, height: 360, context: ["test": "single large file"])
let largeURL = recorder.url!
try check(largeURL != firstURL, "start reused a file path")
let largeCount = 1600
for index in 0..<largeCount {
    let pixels = [UInt8](repeating: UInt8(index % 251), count: 480 * 360)
    let largeImage = pixels.withUnsafeBufferPointer {
        GrayImage(width: 480, height: 360, luma8: $0.baseAddress!, bytesPerRow: 480)
    }
    let input = FrameInput(
        image: largeImage, intrinsics: intrinsics, cameraToWorld: .identity, poseValid: false,
        depth: nil, timestamp: Double(index) / 30)
    recorder.append(
        input: input, luma8: pixels, output: EngineOutput(), config: EngineConfig(), settings: [:],
        droppedFrames: 0, conversionFailures: 0, processedFps: 30)
}
recorder.stop()
let status = recorder.status(now: Double(largeCount - 1) / 30)
try check(status.error == nil && !status.recording && status.frames == largeCount, "stop failed to drain")
let size = try FileManager.default.attributesOfItem(atPath: largeURL.path)[.size] as! NSNumber
#if os(Linux)
    if ProcessInfo.processInfo.environment["WIG_TEST_COMPRESSION"] == "deflate" {
        try check(size.intValue < 5 * 1024 * 1024, "compressible session did not shrink")
    } else {
        try check(size.intValue > 250 * 1024 * 1024, "raw fallback did not cross old split threshold")
    }
#endif
try check(status.megabytes == size.doubleValue / 1_048_576, "size counter is wrong")
try check(status.seconds == Double(largeCount - 1) / 30, "duration is wrong")
recorder.stop()
try check(recorder.url == largeURL, "repeat stop discarded URL")
try check(
    try FileManager.default.contentsOfDirectory(atPath: directory.path).filter { $0.hasSuffix(".wig") }.count == 2,
    "session was split")
let invalid = SessionRecorder(directory: directory.appendingPathComponent("missing/child"))
invalid.start(width: 48, height: 36, context: [:])
try check(!invalid.isRecording && invalid.status(now: 0).error != nil, "open failure was hidden")
let failureReported = DispatchSemaphore(value: 0)
recorder.onFailure = { _ in failureReported.signal() }
recorder.start(width: 48, height: 36, context: [:])
let failedURL = recorder.url!
recorder.append(
    input: frame(0), luma8: [], output: EngineOutput(), config: EngineConfig(), settings: [:],
    droppedFrames: 0, conversionFailures: 0, processedFps: 20)
try check(failureReported.wait(timeout: .now() + 5) == .success, "failure was not reported without another frame")
recorder.stop()
try check(recorder.status(now: 0).error != nil && recorder.status(now: 0).frames == 0, "append failure was hidden")
recorder.start(width: 48, height: 36, context: [:])
try check(recorder.isRecording && recorder.status(now: 0).error == nil, "could not recover after error")
recorder.stop()
let manifest: [String: Any] = [
    "warm": firstURL.lastPathComponent, "large": largeURL.lastPathComponent,
    "failed": failedURL.lastPathComponent, "largeFrames": largeCount, "largeBytes": size,
]
try JSONSerialization.data(withJSONObject: manifest, options: [.prettyPrinted, .sortedKeys])
    .write(to: directory.appendingPathComponent("manifest.json"))
print("PASS: warm capture, settings/planes, drain/restart, one \(size)-byte file, error reporting and recovery")
