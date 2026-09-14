import ARKit
import AVFoundation
import SceneKit
import SwiftUI
import Combine
import WigglerCore

/// Owns the capture (ARKit or plain AVFoundation), feeds every frame to the rotation engine, and exposes what the
/// views need: engine output, marker, image→view mapping and 3D→view projection.
final class ARSessionController: NSObject, ObservableObject, ARSessionDelegate, ARSCNViewDelegate {
    // MARK: Published state

    /// Latest engine output for the overlays (published at ~30 Hz on the main thread).
    @Published private(set) var output = EngineOutput()
    /// Marker in engine image coordinates (pixels of the 480x360 landscape image), nil when not placed.
    @Published private(set) var markerImagePoint: CGPoint?
    @Published var showPoints = true
    @Published var sessionMessage = ""
    @Published private(set) var recorderStatus = SessionRecorder.Status(recording: false, seconds: 0, megabytes: 0, frames: 0, fileName: "")
    @Published private(set) var recordingCount = SessionRecorder.recordings().count
    /// Processed frame rate, dropped frames and camera format (debug line).
    @Published private(set) var statsLine = ""
    /// Set when a recording stops: the view presents the export sheet for it.
    @Published var pendingShare: ShareItem?
    /// ARKit (world tracking + SceneKit) or the lighter AVFoundation path (fixed phone, 2D overlay).
    @Published var useARKit = false {
        didSet { if useARKit != oldValue && started { restart() } }
    }

    /// Region-of-interest radius (feature search area around the marker) as a fraction of the engine image height.
    var roiFraction: Double = 0.35
    /// Number of tracked points the engine aims for (adjustable from the UI).
    @Published private(set) var targetTrackCount = 160

    func adjustTrackCount(by delta: Int) {
        targetTrackCount = max(20, min(400, targetTrackCount + delta))
        let n = targetTrackCount
        engineQueue.async { [engine] in engine.config.targetTrackCount = n }
    }

    // MARK: Views backing

    let sceneView = ARSCNView(frame: .zero)
    let previewLayer = AVCaptureVideoPreviewLayer()

    // MARK: Private

    private let engine = RotationEngine()
    private let converter = FrameConverter()
    private let recorder = SessionRecorder()
    private var avSource: AVCaptureSource?
    private var started = false
    private var recordEveryNth = 2
    private var frameCounter = 0
    /// ARKit delivers frames here; conversion is quick and the ARFrame is released immediately.
    private let frameQueue = DispatchQueue(label: "ch.mariogeiger.wiggler.frames", qos: .userInteractive)
    /// The engine runs here; frames arriving while it is busy are dropped (never queued) so the capture is never starved.
    private let engineQueue = DispatchQueue(label: "ch.mariogeiger.wiggler.engine", qos: .userInteractive)
    private var engineBusy = false
    private var droppedFrames = 0
    private var processedFps = 0.0
    private var lastProcessedTimestamp: Double?
    private var formatDescription = ""
    private let overlay = AxisOverlayNode()
    private let lock = NSLock()
    private var latestOutput = EngineOutput()
    private var latestDisplayTransform = CGAffineTransform.identity
    private var latestIntrinsics: CameraIntrinsics?
    private var latestWorldToCamera = RigidTransform.identity
    private var markerImagePointForFrameQueue: CGPoint?
    private var roiRadiusForFrameQueue: Float = 0
    private var viewportSize = CGSize(width: 1, height: 1)
    private var lastPublish = Date.distantPast

    override init() {
        super.init()
        sceneView.delegate = self
        sceneView.session.delegate = self
        sceneView.session.delegateQueue = frameQueue
        sceneView.automaticallyUpdatesLighting = true
        sceneView.rendersContinuously = true
        sceneView.preferredFramesPerSecond = 60
        sceneView.scene.rootNode.addChildNode(overlay)
        overlay.isHidden = true
        previewLayer.videoGravity = .resizeAspectFill
    }

    // MARK: Lifecycle

    func start() {
        started = true
        if useARKit { startARKit() } else { startAV() }
    }

    func pause() {
        sceneView.session.pause()
        avSource?.stop()
    }

    private func restart() {
        pause()
        clearMarker()
        droppedFrames = 0
        processedFps = 0
        lastProcessedTimestamp = nil
        start()
    }

    private func startARKit() {
        guard ARWorldTrackingConfiguration.isSupported else {
            sessionMessage = "ARKit non disponible"
            return
        }
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        } else {
            sessionMessage = "Pas de LiDAR : profondeur indisponible"
        }
        config.planeDetection = []
        config.isAutoFocusEnabled = true
        config.environmentTexturing = .none
        let formats = ARWorldTrackingConfiguration.supportedVideoFormats
        if let f = formats.first(where: { $0.framesPerSecond >= 60 && $0.imageResolution.width <= 2000 })
            ?? formats.first(where: { $0.framesPerSecond >= 60 }) {
            config.videoFormat = f
        }
        let vf = config.videoFormat
        formatDescription = "ARKit \(Int(vf.imageResolution.width))x\(Int(vf.imageResolution.height))@\(vf.framesPerSecond)"
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    private func startAV() {
        let src = AVCaptureSource()
        guard src.configure() else {
            sessionMessage = "Caméra LiDAR (AVFoundation) indisponible — bascule sur ARKit"
            useARKit = true
            return
        }
        formatDescription = src.formatDescription
        previewLayer.session = src.session
        previewLayer.connection?.videoOrientation = .portrait
        src.onFrame = { [weak self] input, luma8, hasDepth in
            self?.enqueue(input: input, luma8: luma8, hasDepth: hasDepth, displayTransform: nil)
        }
        avSource = src
        src.start()
    }

    // MARK: Marker handling (main thread)

    /// Called with a touch location in view coordinates.
    func placeMarker(viewPoint: CGPoint) {
        guard let n = normalizedImagePoint(fromView: viewPoint) else { return }
        let p = CGPoint(x: n.x * CGFloat(FrameConverter.engineWidth), y: n.y * CGFloat(FrameConverter.engineHeight))
        markerImagePoint = p
        applyMarker()
    }

    func toggleRecording() {
        if recorder.isRecording {
            let url = recorder.url
            recorder.stop()
            recordingCount = SessionRecorder.recordings().count
            if let u = url { pendingShare = ShareItem(url: u) }
        } else {
            recorder.start()
        }
        recorderStatus = recorder.status(now: 0)
    }

    func deleteRecordings() {
        for u in SessionRecorder.recordings() { try? FileManager.default.removeItem(at: u) }
        recordingCount = SessionRecorder.recordings().count
    }

    func clearMarker() {
        markerImagePoint = nil
        lock.lock()
        markerImagePointForFrameQueue = nil
        lock.unlock()
        engineQueue.async { [engine] in engine.clearMarker() }
    }

    private func applyMarker() {
        guard let p = markerImagePoint else { return }
        let radius = Float(roiFraction * Double(FrameConverter.engineHeight))
        lock.lock()
        markerImagePointForFrameQueue = p
        roiRadiusForFrameQueue = radius
        lock.unlock()
        engineQueue.async { [engine] in
            engine.setMarker(x: Float(p.x), y: Float(p.y), radius: radius)
        }
    }

    // MARK: Coordinate mapping (main thread)

    /// Normalised image coordinates (0…1, landscape sensor frame, origin top-left) → view coordinates.
    func viewPoint(normalizedX: CGFloat, normalizedY: CGFloat) -> CGPoint {
        if useARKit {
            lock.lock()
            let t = latestDisplayTransform
            let size = viewportSize
            lock.unlock()
            let n = CGPoint(x: normalizedX, y: normalizedY).applying(t)
            return CGPoint(x: n.x * size.width, y: n.y * size.height)
        } else {
            return previewLayer.layerPointConverted(fromCaptureDevicePoint: CGPoint(x: normalizedX, y: normalizedY))
        }
    }

    /// Engine image pixel → view coordinates.
    func viewPoint(imageX: CGFloat, imageY: CGFloat) -> CGPoint {
        viewPoint(normalizedX: imageX / CGFloat(FrameConverter.engineWidth), normalizedY: imageY / CGFloat(FrameConverter.engineHeight))
    }

    private func normalizedImagePoint(fromView p: CGPoint) -> CGPoint? {
        if useARKit {
            lock.lock()
            let t = latestDisplayTransform
            let size = viewportSize
            lock.unlock()
            guard size.width > 1, size.height > 1 else { return nil }
            return CGPoint(x: p.x / size.width, y: p.y / size.height).applying(t.inverted())
        } else {
            return previewLayer.captureDevicePointConverted(fromLayerPoint: p)
        }
    }

    /// Project a world point into view coordinates (nil when behind the camera). Used by the 2D axis overlay.
    func projectToView(_ world: V3) -> CGPoint? {
        lock.lock()
        let K = latestIntrinsics
        let w2c = latestWorldToCamera
        lock.unlock()
        guard let k = K else { return nil }
        let c = w2c.apply(world)          // camera frame: x right, y up, z back
        let z = -c.z
        guard z > 0.05 else { return nil }
        let u = k.fx * c.x / z + k.cx
        let v = k.cy - k.fy * c.y / z
        return viewPoint(imageX: CGFloat(u), imageY: CGFloat(v))
    }

    func updateViewportSize(_ size: CGSize) {
        lock.lock()
        viewportSize = size
        lock.unlock()
        previewLayer.frame = CGRect(origin: .zero, size: size)
    }

    // MARK: Frame handling

    private func enqueue(input: FrameInput, luma8: [UInt8], hasDepth: Bool, displayTransform: CGAffineTransform?) {
        lock.lock()
        let busy = engineBusy
        if !busy { engineBusy = true }
        lock.unlock()
        if busy {
            droppedFrames += 1
            return
        }
        engineQueue.async { [self] in
            if let lt = lastProcessedTimestamp, input.timestamp > lt {
                let fps = 1.0 / (input.timestamp - lt)
                processedFps += 0.1 * (fps - processedFps)
            }
            lastProcessedTimestamp = input.timestamp
            var out = engine.process(input)
            if !hasDepth && !out.message.isEmpty && out.state != .idle {
                out.message = "Profondeur LiDAR absente — " + out.message
            }
            lock.lock()
            latestOutput = out
            if let t = displayTransform { latestDisplayTransform = t }
            latestIntrinsics = input.intrinsics
            latestWorldToCamera = input.cameraToWorld.inverse
            let marker = markerImagePointForFrameQueue
            let roi = roiRadiusForFrameQueue
            engineBusy = false
            lock.unlock()
            frameCounter += 1
            if recorder.isRecording && frameCounter % recordEveryNth == 0 {
                recorder.append(input: input, luma8: luma8, output: out, marker: marker, roiRadius: roi,
                                droppedFrames: droppedFrames, processedFps: processedFps)
            }
            let stats = String(format: "%.0f fps · %d sautées · %@", processedFps, droppedFrames, formatDescription)
            let now = Date()
            if now.timeIntervalSince(lastPublish) > 1.0 / 30.0 {
                lastPublish = now
                let status = recorder.status(now: input.timestamp)
                DispatchQueue.main.async { [weak self] in
                    self?.output = out
                    self?.recorderStatus = status
                    self?.statsLine = stats
                }
            }
        }
    }

    // MARK: ARSessionDelegate (frame queue)

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard useARKit else { return }
        lock.lock()
        let size = viewportSize
        lock.unlock()
        guard let input = converter.convert(frame) else { return }
        let luma8 = converter.lastLuma8
        let transform = frame.displayTransform(for: .portrait, viewportSize: size)
        let hasDepth = frame.sceneDepth != nil
        // Nothing below touches `frame` any more.
        enqueue(input: input, luma8: luma8, hasDepth: hasDepth, displayTransform: transform)
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async { self.sessionMessage = "Erreur AR : \(error.localizedDescription)" }
    }

    func sessionWasInterrupted(_ session: ARSession) {
        DispatchQueue.main.async { self.sessionMessage = "Session AR interrompue" }
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        DispatchQueue.main.async { self.sessionMessage = "" }
    }

    // MARK: ARSCNViewDelegate (render thread)

    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        lock.lock()
        let out = latestOutput
        lock.unlock()
        overlay.update(with: out)
    }
}
