import ARKit
import SceneKit
import SwiftUI
import Combine
import WigglerCore

/// Owns the ARKit session, feeds every frame to the rotation engine and drives the SceneKit overlay.
final class ARSessionController: NSObject, ObservableObject, ARSessionDelegate, ARSCNViewDelegate {
    // MARK: Published state (main thread)

    /// Latest engine output for the overlays (published at ~30 Hz).
    @Published private(set) var output = EngineOutput()
    /// Marker in engine image coordinates (pixels of the 480x360 landscape image), nil when not placed.
    @Published private(set) var markerImagePoint: CGPoint?
    /// Number of tracked points the engine aims for.
    @Published private(set) var targetTrackCount = 160
    @Published private(set) var recorderStatus = SessionRecorder.Status(recording: false, seconds: 0, megabytes: 0, frames: 0, fileName: "")
    /// Set when a recording stops: the view presents the export sheet for it.
    @Published var pendingShare: ShareItem?

    let sceneView = ARSCNView(frame: .zero)

    // MARK: Private

    /// Feature search radius around the marker, as a fraction of the engine image height.
    private let roiFraction = 0.35
    private let engine = RotationEngine()
    private let converter = FrameConverter()
    private let recorder = SessionRecorder()
    private let recordEveryNth = 2
    private var frameCounter = 0
    /// ARKit delivers frames here; conversion is quick and the ARFrame is released immediately.
    private let frameQueue = DispatchQueue(label: "ch.mariogeiger.wiggler.frames", qos: .userInteractive)
    /// The engine runs here; frames arriving while it is busy are dropped (never queued) so ARKit is never starved.
    private let engineQueue = DispatchQueue(label: "ch.mariogeiger.wiggler.engine", qos: .userInteractive)
    private var engineBusy = false
    private var droppedFrames = 0
    private var processedFps = 0.0
    private var lastProcessedTimestamp: Double?
    private let overlay = AxisOverlayNode()
    private let lock = NSLock()
    private var latestOutput = EngineOutput()
    private var latestDisplayTransform = CGAffineTransform.identity
    private var markerForEngine: CGPoint?
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
    }

    // MARK: Lifecycle

    func start() {
        guard ARWorldTrackingConfiguration.isSupported else { return }
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        config.planeDetection = []
        config.isAutoFocusEnabled = true
        config.environmentTexturing = .none
        // Prefer the 60 fps 4:3 format of the wide camera.
        let formats = ARWorldTrackingConfiguration.supportedVideoFormats
        if let f = formats.first(where: { $0.framesPerSecond >= 60 && $0.imageResolution.width <= 2000 })
            ?? formats.first(where: { $0.framesPerSecond >= 60 }) {
            config.videoFormat = f
        }
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func pause() { sceneView.session.pause() }

    // MARK: Controls (main thread)

    /// Called with a touch location in view coordinates.
    func placeMarker(viewPoint: CGPoint) {
        lock.lock()
        let t = latestDisplayTransform
        let size = viewportSize
        lock.unlock()
        guard size.width > 1, size.height > 1 else { return }
        let n = CGPoint(x: viewPoint.x / size.width, y: viewPoint.y / size.height).applying(t.inverted())
        let p = CGPoint(x: n.x * CGFloat(FrameConverter.engineWidth), y: n.y * CGFloat(FrameConverter.engineHeight))
        markerImagePoint = p
        lock.lock()
        markerForEngine = p
        lock.unlock()
        let radius = Float(roiFraction * Double(FrameConverter.engineHeight))
        engineQueue.async { [engine] in engine.setMarker(x: Float(p.x), y: Float(p.y), radius: radius) }
    }

    func adjustTrackCount(by delta: Int) {
        targetTrackCount = max(20, min(400, targetTrackCount + delta))
        let n = targetTrackCount
        engineQueue.async { [engine] in engine.config.targetTrackCount = n }
    }

    func toggleRecording() {
        if recorder.isRecording {
            recorder.stop()
            let urls = recorder.sessionURLs
            if !urls.isEmpty { pendingShare = ShareItem(urls: urls) }
        } else {
            recorder.start()
        }
        recorderStatus = recorder.status(now: 0)
    }

    /// Engine image pixel → view coordinates (for the overlays).
    func viewPoint(imageX: CGFloat, imageY: CGFloat) -> CGPoint {
        lock.lock()
        let t = latestDisplayTransform
        let size = viewportSize
        lock.unlock()
        let n = CGPoint(x: imageX / CGFloat(FrameConverter.engineWidth), y: imageY / CGFloat(FrameConverter.engineHeight)).applying(t)
        return CGPoint(x: n.x * size.width, y: n.y * size.height)
    }

    func updateViewportSize(_ size: CGSize) {
        lock.lock()
        viewportSize = size
        lock.unlock()
    }

    // MARK: ARSessionDelegate (frame queue)

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        lock.lock()
        let busy = engineBusy
        if !busy { engineBusy = true }
        let size = viewportSize
        lock.unlock()
        if busy {
            droppedFrames += 1
            return
        }
        guard let input = converter.convert(frame) else {
            lock.lock(); engineBusy = false; lock.unlock()
            return
        }
        let luma8 = converter.lastLuma8
        let transform = frame.displayTransform(for: .portrait, viewportSize: size)
        // Nothing below touches `frame` any more.
        engineQueue.async { [self] in
            if let lt = lastProcessedTimestamp, input.timestamp > lt {
                processedFps += 0.1 * (1.0 / (input.timestamp - lt) - processedFps)
            }
            lastProcessedTimestamp = input.timestamp
            let out = engine.process(input)
            lock.lock()
            latestOutput = out
            latestDisplayTransform = transform
            let marker = markerForEngine
            engineBusy = false
            lock.unlock()
            frameCounter += 1
            if recorder.isRecording && frameCounter % recordEveryNth == 0 {
                recorder.append(input: input, luma8: luma8, output: out, marker: marker,
                                roiRadius: Float(roiFraction * Double(FrameConverter.engineHeight)),
                                droppedFrames: droppedFrames, processedFps: processedFps)
            }
            let now = Date()
            if now.timeIntervalSince(lastPublish) > 1.0 / 30.0 {
                lastPublish = now
                let status = recorder.status(now: input.timestamp)
                DispatchQueue.main.async { [weak self] in
                    self?.output = out
                    self?.recorderStatus = status
                }
            }
        }
    }

    // MARK: ARSCNViewDelegate (render thread)

    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        lock.lock()
        let out = latestOutput
        lock.unlock()
        overlay.update(with: out)
    }
}
