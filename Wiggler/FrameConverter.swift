import ARKit
import Accelerate
import WigglerCore

/// Converts ARKit frames (YCbCr image + LiDAR depth + camera pose) into the engine's input format.
final class FrameConverter {
    /// Engine image size (landscape, same aspect as the 4:3 capture).
    static let engineWidth = 480
    static let engineHeight = 360

    private var scaled = [UInt8](repeating: 0, count: engineWidth * engineHeight)
    private var tempBuffer: UnsafeMutableRawPointer?
    private var tempBufferSize = 0

    deinit { tempBuffer?.deallocate() }

    func convert(_ frame: ARFrame) -> FrameInput? {
        guard let image = luma(from: frame.capturedImage) else { return nil }
        let res = frame.camera.imageResolution
        let k = frame.camera.intrinsics
        let intrinsics = CameraIntrinsics(fx: Double(k[0][0]), fy: Double(k[1][1]), cx: Double(k[2][0]), cy: Double(k[2][1]))
            .scaled(fromWidth: Double(res.width), fromHeight: Double(res.height),
                    toWidth: Double(Self.engineWidth), toHeight: Double(Self.engineHeight))
        let m = frame.camera.transform
        let pose = RigidTransform(columnMajor4x4: [
            Double(m.columns.0.x), Double(m.columns.0.y), Double(m.columns.0.z), Double(m.columns.0.w),
            Double(m.columns.1.x), Double(m.columns.1.y), Double(m.columns.1.z), Double(m.columns.1.w),
            Double(m.columns.2.x), Double(m.columns.2.y), Double(m.columns.2.z), Double(m.columns.2.w),
            Double(m.columns.3.x), Double(m.columns.3.y), Double(m.columns.3.z), Double(m.columns.3.w),
        ])
        let poseValid: Bool
        switch frame.camera.trackingState {
        case .normal: poseValid = true
        case .limited(let reason):
            // With a fixed phone ARKit often reports "insufficient features"/"excessive motion" while the object
            // spins; the pose is still usable. Only "initializing"/"relocalizing" are rejected.
            switch reason {
            case .initializing, .relocalizing: poseValid = false
            default: poseValid = true
            }
        case .notAvailable: poseValid = false
        }
        let depth = frame.sceneDepth.flatMap { depthMap(from: $0) }
        return FrameInput(image: image, intrinsics: intrinsics, cameraToWorld: pose, poseValid: poseValid,
                          depth: depth, timestamp: frame.timestamp)
    }

    /// Downscale the luma plane with vImage and convert to a float image.
    private func luma(from pixelBuffer: CVPixelBuffer) -> GrayImage? {
        let format = CVPixelBufferGetPixelFormatType(pixelBuffer)
        guard format == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange || format == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange,
              CVPixelBufferGetPlaneCount(pixelBuffer) >= 1 else { return nil }
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0) else { return nil }
        let w = CVPixelBufferGetWidthOfPlane(pixelBuffer, 0)
        let h = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0)
        let stride = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        var src = vImage_Buffer(data: base, height: vImagePixelCount(h), width: vImagePixelCount(w), rowBytes: stride)
        var result: GrayImage?
        scaled.withUnsafeMutableBytes { dstBytes in
            var dst = vImage_Buffer(data: dstBytes.baseAddress, height: vImagePixelCount(Self.engineHeight),
                                    width: vImagePixelCount(Self.engineWidth), rowBytes: Self.engineWidth)
            let needed = vImageScale_Planar8(&src, &dst, nil, vImage_Flags(kvImageGetTempBufferSize))
            if needed > tempBufferSize {
                tempBuffer?.deallocate()
                tempBuffer = UnsafeMutableRawPointer.allocate(byteCount: needed, alignment: 16)
                tempBufferSize = needed
            }
            let err = vImageScale_Planar8(&src, &dst, tempBuffer, vImage_Flags(kvImageNoFlags))
            if err == kvImageNoError, let p = dstBytes.baseAddress?.assumingMemoryBound(to: UInt8.self) {
                result = GrayImage(width: Self.engineWidth, height: Self.engineHeight, luma8: p, bytesPerRow: Self.engineWidth)
            }
        }
        return result
    }

    private func depthMap(from depth: ARDepthData) -> DepthMap? {
        let buffer = depth.depthMap
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
        var confidence: [UInt8]?
        if let cbuf = depth.confidenceMap {
            CVPixelBufferLockBaseAddress(cbuf, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(cbuf, .readOnly) }
            if CVPixelBufferGetWidth(cbuf) == w, CVPixelBufferGetHeight(cbuf) == h, let cbase = CVPixelBufferGetBaseAddress(cbuf) {
                let cstride = CVPixelBufferGetBytesPerRow(cbuf)
                var c = [UInt8](repeating: 0, count: w * h)
                for y in 0..<h {
                    let row = cbase.advanced(by: y * cstride).assumingMemoryBound(to: UInt8.self)
                    for x in 0..<w { c[y * w + x] = row[x] }
                }
                confidence = c
            }
        }
        return DepthMap(width: w, height: h, depth: values, confidence: confidence)
    }
}
