import AVFoundation
import simd
import CoreVideo
import Accelerate
import WigglerCore

/// Lightweight capture path (no ARKit): the LiDAR depth camera through AVFoundation. Video frames (60 Hz when the
/// format allows it) are synchronised with depth frames; the latest depth map is reused when depth lags behind video.
/// Camera pose is the identity (the phone is fixed), so "world" = camera coordinates (ARKit convention: x right, y up, z back).
final class AVCaptureSource: NSObject, AVCaptureDataOutputSynchronizerDelegate {
    let session = AVCaptureSession()
    private let queue = DispatchQueue(label: "ch.mariogeiger.wiggler.avcapture", qos: .userInteractive)
    private let videoOutput = AVCaptureVideoDataOutput()
    private let depthOutput = AVCaptureDepthDataOutput()
    private var synchronizer: AVCaptureDataOutputSynchronizer?
    private let converter = FrameConverter()
    private var lastDepth: DepthMap?
    private var lastDepthTime: Double = 0
    private var intrinsics: CameraIntrinsics?
    private(set) var formatDescription = ""

    /// Called on the capture queue with the converted frame and the 8-bit luma copy (for the recorder).
    var onFrame: ((FrameInput, [UInt8], Bool) -> Void)?

    func configure() -> Bool {
        guard let device = AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) else { return false }
        session.beginConfiguration()
        session.sessionPreset = .inputPriority
        guard let input = try? AVCaptureDeviceInput(device: device), session.canAddInput(input) else {
            session.commitConfiguration(); return false
        }
        session.addInput(input)

        // Pick the 4:3 format with depth support and the highest frame rate (60 Hz on iPhone 15 Pro), moderate resolution.
        var best: AVCaptureDevice.Format?
        var bestScore = -1.0
        for f in device.formats {
            guard !f.supportedDepthDataFormats.isEmpty else { continue }
            let dims = CMVideoFormatDescriptionGetDimensions(f.formatDescription)
            guard abs(Double(dims.width) / Double(dims.height) - 4.0 / 3.0) < 0.05, dims.width <= 2000 else { continue }
            let fps = f.videoSupportedFrameRateRanges.map { $0.maxFrameRate }.max() ?? 0
            let score = fps * 10 + (dims.width >= 1440 ? 1 : 0)
            if score > bestScore { bestScore = score; best = f }
        }
        guard let format = best else { session.commitConfiguration(); return false }
        let depthFormat = format.supportedDepthDataFormats
            .filter { CMFormatDescriptionGetMediaSubType($0.formatDescription) == kCVPixelFormatType_DepthFloat32 }
            .max { a, b in
                CMVideoFormatDescriptionGetDimensions(a.formatDescription).width < CMVideoFormatDescriptionGetDimensions(b.formatDescription).width
            } ?? format.supportedDepthDataFormats.last!
        let fps = format.videoSupportedFrameRateRanges.map { $0.maxFrameRate }.max() ?? 30
        do {
            try device.lockForConfiguration()
            device.activeFormat = format
            device.activeDepthDataFormat = depthFormat
            let duration = CMTime(value: 1, timescale: CMTimeScale(fps.rounded()))
            device.activeVideoMinFrameDuration = duration
            device.activeVideoMaxFrameDuration = duration
            if device.isFocusModeSupported(.continuousAutoFocus) { device.focusMode = .continuousAutoFocus }
            device.unlockForConfiguration()
        } catch {
            session.commitConfiguration(); return false
        }
        let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
        let ddims = CMVideoFormatDescriptionGetDimensions(depthFormat.formatDescription)
        formatDescription = "AV \(dims.width)x\(dims.height)@\(Int(fps)) depth \(ddims.width)x\(ddims.height)"

        videoOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange]
        videoOutput.alwaysDiscardsLateVideoFrames = true
        guard session.canAddOutput(videoOutput) else { session.commitConfiguration(); return false }
        session.addOutput(videoOutput)
        if let c = videoOutput.connection(with: .video), c.isCameraIntrinsicMatrixDeliverySupported {
            c.isCameraIntrinsicMatrixDeliveryEnabled = true
        }
        depthOutput.isFilteringEnabled = true
        depthOutput.alwaysDiscardsLateDepthData = true
        guard session.canAddOutput(depthOutput) else { session.commitConfiguration(); return false }
        session.addOutput(depthOutput)
        session.commitConfiguration()

        let sync = AVCaptureDataOutputSynchronizer(dataOutputs: [videoOutput, depthOutput])
        sync.setDelegate(self, queue: queue)
        synchronizer = sync
        return true
    }

    func start() { queue.async { if !self.session.isRunning { self.session.startRunning() } } }
    func stop() { queue.async { if self.session.isRunning { self.session.stopRunning() } } }

    // MARK: AVCaptureDataOutputSynchronizerDelegate

    func dataOutputSynchronizer(_ synchronizer: AVCaptureDataOutputSynchronizer,
                                didOutput collection: AVCaptureSynchronizedDataCollection) {
        guard let videoData = collection.synchronizedData(for: videoOutput) as? AVCaptureSynchronizedSampleBufferData,
              !videoData.sampleBufferWasDropped,
              let pixelBuffer = CMSampleBufferGetImageBuffer(videoData.sampleBuffer) else { return }
        let timestamp = CMTimeGetSeconds(videoData.timestamp)

        if let depthData = collection.synchronizedData(for: depthOutput) as? AVCaptureSynchronizedDepthData,
           !depthData.depthDataWasDropped {
            var d = depthData.depthData
            if d.depthDataType != kCVPixelFormatType_DepthFloat32 {
                d = d.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
            }
            if let map = Self.depthMap(from: d) {
                lastDepth = map
                lastDepthTime = timestamp
            }
            if intrinsics == nil, let cal = d.cameraCalibrationData {
                let k = cal.intrinsicMatrix
                let ref = cal.intrinsicMatrixReferenceDimensions
                intrinsics = CameraIntrinsics(fx: Double(k[0][0]), fy: Double(k[1][1]), cx: Double(k[2][0]), cy: Double(k[2][1]))
                    .scaled(fromWidth: Double(ref.width), fromHeight: Double(ref.height),
                            toWidth: Double(FrameConverter.engineWidth), toHeight: Double(FrameConverter.engineHeight))
            }
        }
        if intrinsics == nil,
           let att = CMGetAttachment(videoData.sampleBuffer, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, attachmentModeOut: nil),
           let data = att as? Data, data.count >= MemoryLayout<matrix_float3x3>.size {
            let k = data.withUnsafeBytes { $0.loadUnaligned(as: matrix_float3x3.self) }
            let w = Double(CVPixelBufferGetWidth(pixelBuffer)), h = Double(CVPixelBufferGetHeight(pixelBuffer))
            intrinsics = CameraIntrinsics(fx: Double(k[0][0]), fy: Double(k[1][1]), cx: Double(k[2][0]), cy: Double(k[2][1]))
                .scaled(fromWidth: w, fromHeight: h, toWidth: Double(FrameConverter.engineWidth), toHeight: Double(FrameConverter.engineHeight))
        }
        guard let K = intrinsics, let image = converter.luma(from: pixelBuffer) else { return }
        // Depth older than ~50 ms would attach stale geometry to fast-moving points: skip it.
        let depth = (timestamp - lastDepthTime) < 0.05 ? lastDepth : nil
        let input = FrameInput(image: image, intrinsics: K, cameraToWorld: .identity, poseValid: true,
                               depth: depth, timestamp: timestamp)
        onFrame?(input, converter.lastLuma8, depth != nil)
    }

    private static func depthMap(from depth: AVDepthData) -> DepthMap? {
        let buffer = depth.depthDataMap
        guard CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_DepthFloat32 else { return nil }
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        let w = CVPixelBufferGetWidth(buffer), h = CVPixelBufferGetHeight(buffer)
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { return nil }
        let stride = CVPixelBufferGetBytesPerRow(buffer)
        var values = [Float](repeating: 0, count: w * h)
        for y in 0..<h {
            let row = base.advanced(by: y * stride).assumingMemoryBound(to: Float.self)
            for x in 0..<w { values[y * w + x] = row[x] }
        }
        // AVFoundation exposes no per-pixel confidence map; treat everything as medium confidence.
        return DepthMap(width: w, height: h, depth: values, confidence: nil)
    }
}
