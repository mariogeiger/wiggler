import ARKit
import Combine
import SceneKit
import SwiftUI
import WigglerCore

/// Owns the ARKit session, feeds every frame to the rotation engine and drives the SceneKit overlay.
final class ARSessionController: NSObject, ObservableObject, ARSessionDelegate, ARSCNViewDelegate {
    // MARK: Published state (main thread)

    /// Latest engine output for the overlays (published at ~30 Hz).
    @Published private(set) var output = EngineOutput()
    /// Number of tracked points the engine aims for. Persisted across launches.
    @Published private(set) var targetTrackCount = EngineConfig().targetTrackCount {
        didSet { UserDefaults.standard.set(targetTrackCount, forKey: Self.trackCountKey) }
    }
    private static let trackCountKey = "targetTrackCount"
    private static let trackCountRange = 5...400
    /// Harmonics of the luma in the rotation angle that the map fits and the user can display.
    static let harmonicOrders = [1, 2, 3]
    /// Which harmonic of the current image is drawn over the camera image; nil = off. Persisted.
    @Published private(set) var harmonicOrder: Int? {
        didSet { UserDefaults.standard.set(harmonicOrder ?? 0, forKey: Self.harmonicOrderKey) }
    }
    private static let harmonicOrderKey = "harmonicOrder"
    /// Fraction of the first full turn the harmonic map has accumulated (it is drawn from 1).
    @Published private(set) var harmonicProgress = 0.0
    @Published private(set) var recorderStatus = SessionRecorder.Status(
        recording: false, seconds: 0, megabytes: 0, frames: 0, fileName: "")
    /// Set when a recording stops: the view presents the export sheet for it.
    @Published var pendingShare: ShareItem?

    let sceneView = ARSCNView(frame: .zero)

    // MARK: Private

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
    private let harmonicOverlay = HarmonicOverlayNode()
    private let lock = NSLock()
    private var latestOutput = EngineOutput()
    private var latestHarmonicImage: CGImage?
    private var latestDisplayTransform = CGAffineTransform.identity
    private var viewportSize = CGSize(width: 1, height: 1)
    private var lastPublish = Date.distantPast
    // Engine queue. Only stable measurements contribute to the map, even when its display is off.
    private var harmonics = HarmonicMap(
        width: FrameConverter.engineWidth, height: FrameConverter.engineHeight,
        orders: ARSessionController.harmonicOrders)
    private var harmonicOrderEngine: Int?
    private var renderedHarmonicImage: CGImage?
    private var lastHarmonicRender = Date.distantPast
    /// Fully opaque at this luma modulation (20 of 255 levels).
    private let harmonicFullScale: Float = 20 / 255

    override init() {
        super.init()
        sceneView.delegate = self
        sceneView.session.delegate = self
        sceneView.session.delegateQueue = frameQueue
        sceneView.automaticallyUpdatesLighting = true
        sceneView.rendersContinuously = true
        sceneView.preferredFramesPerSecond = 60
        sceneView.scene.rootNode.addChildNode(harmonicOverlay)
        sceneView.scene.rootNode.addChildNode(overlay)
        overlay.isHidden = true
        let saved = UserDefaults.standard.integer(forKey: Self.trackCountKey)
        if saved != 0 { targetTrackCount = Self.clampTrackCount(saved) }
        engine.config.targetTrackCount = targetTrackCount
        let savedOrder = UserDefaults.standard.integer(forKey: Self.harmonicOrderKey)
        harmonicOrder = Self.harmonicOrders.contains(savedOrder) ? savedOrder : nil
        harmonicOrderEngine = harmonicOrder
    }

    private static func clampTrackCount(_ n: Int) -> Int {
        max(trackCountRange.lowerBound, min(trackCountRange.upperBound, n))
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
            ?? formats.first(where: { $0.framesPerSecond >= 60 })
        {
            config.videoFormat = f
        }
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        // A measurement session is hands-off by nature: the phone must not lock while the camera is running.
        UIApplication.shared.isIdleTimerDisabled = true
    }

    func pause() {
        sceneView.session.pause()
        UIApplication.shared.isIdleTimerDisabled = false
    }

    // MARK: Controls (main thread)

    func adjustTrackCount(by delta: Int) {
        targetTrackCount = Self.clampTrackCount(targetTrackCount + delta)
        let n = targetTrackCount
        engineQueue.async { [engine] in engine.config.targetTrackCount = n }
    }

    func setHarmonicOrder(_ l: Int?) {
        harmonicOrder = l
        engineQueue.async { [self] in harmonicOrderEngine = l }
    }

    func toggleRecording() {
        if recorder.isRecording {
            recorder.stop()
            if let url = recorder.url { pendingShare = ShareItem(urls: [url]) }
        } else {
            recorder.start()
        }
        recorderStatus = recorder.status(now: 0)
    }

    /// Normalised engine image → normalised view coordinates.
    func displayTransform() -> CGAffineTransform {
        lock.lock()
        defer { lock.unlock() }
        return latestDisplayTransform
    }

    /// Engine image pixel → view coordinates (for the overlays).
    func viewPoint(imageX: CGFloat, imageY: CGFloat) -> CGPoint {
        lock.lock()
        let t = latestDisplayTransform
        let size = viewportSize
        lock.unlock()
        let n = CGPoint(
            x: imageX / CGFloat(FrameConverter.engineWidth), y: imageY / CGFloat(FrameConverter.engineHeight)
        ).applying(t)
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
            lock.lock()
            engineBusy = false
            lock.unlock()
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
            updateHarmonics(input: input, output: out)
            lock.lock()
            latestOutput = out
            latestHarmonicImage = renderedHarmonicImage
            latestDisplayTransform = transform
            engineBusy = false
            lock.unlock()
            frameCounter += 1
            if recorder.isRecording && frameCounter % recordEveryNth == 0 {
                recorder.append(
                    input: input, luma8: luma8, output: out,
                    marker: out.marker.map { CGPoint(x: CGFloat($0.x), y: CGFloat($0.y)) },
                    roiRadius: out.marker?.radius ?? 0,
                    droppedFrames: droppedFrames, processedFps: processedFps)
            }
            let now = Date()
            if now.timeIntervalSince(lastPublish) > 1.0 / 30.0 {
                lastPublish = now
                let status = recorder.status(now: input.timestamp)
                let progress = harmonics.turnProgress
                DispatchQueue.main.async { [weak self] in
                    self?.output = out
                    self?.harmonicProgress = progress
                    self?.recorderStatus = status
                }
            }
        }
    }

    /// Engine queue. Feeds the harmonic map and publishes its rendering at ≤ 15 Hz while it is meaningful.
    private func updateHarmonics(input: FrameInput, output out: EngineOutput) {
        harmonics.update(image: input.image, output: out)
        let now = Date()
        let show = harmonicOrderEngine != nil && out.axisStable && harmonics.hasFullTurn
        if show {
            guard now.timeIntervalSince(lastHarmonicRender) > 1.0 / 15.0, let l = harmonicOrderEngine,
                let rgba = harmonics.render(order: l, theta: out.theta, fullScale: harmonicFullScale)
            else { return }
            lastHarmonicRender = now
            renderedHarmonicImage = CGImage.rgba8(width: harmonics.width, height: harmonics.height, bytes: rgba)
        } else {
            renderedHarmonicImage = nil
        }
    }

    // MARK: ARSCNViewDelegate (render thread)

    func renderer(_ renderer: SCNSceneRenderer, willRenderScene scene: SCNScene, atTime time: TimeInterval) {
        lock.lock()
        let out = latestOutput
        let image = latestHarmonicImage
        let transform = latestDisplayTransform
        let size = viewportSize
        lock.unlock()
        harmonicOverlay.update(image: image, displayTransform: transform, viewportSize: size, renderer: renderer)
        overlay.update(with: out, hideRay: image != nil)
    }
}
