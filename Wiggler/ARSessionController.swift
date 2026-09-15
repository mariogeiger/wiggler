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
    @Published private(set) var algorithm = RotationAlgorithm.trackedGeometry {
        didSet { UserDefaults.standard.set(algorithm.rawValue, forKey: Self.algorithmKey) }
    }
    private static let algorithmKey = "rotationAlgorithm"
    /// Main-thread selection generation, read by the frame and engine queues under the snapshot lock.
    private var algorithmRevision = 0
    /// Harmonics in the rotation angle that the map fits and the user can display.
    static let harmonicOrders = [1, 2, 3]
    /// Which harmonic of the current image is drawn over the camera image; nil = off. Persisted.
    @Published private(set) var harmonicOrder: Int? {
        didSet { UserDefaults.standard.set(harmonicOrder ?? 0, forKey: Self.harmonicOrderKey) }
    }
    private static let harmonicOrderKey = "harmonicOrder"
    @Published private(set) var harmonicSignal = HarmonicSignal.luma {
        didSet { UserDefaults.standard.set(harmonicSignal.rawValue, forKey: Self.harmonicSignalKey) }
    }
    private static let harmonicSignalKey = "harmonicSignal"
    /// Main-thread selection generation, also read under the snapshot lock by the engine queue.
    private var harmonicRevision = 0
    /// Fraction of the first full turn the harmonic map has accumulated (it is drawn from 1).
    @Published private(set) var harmonicProgress = 0.0
    @Published private(set) var recorderStatus = SessionRecorder.Status(
        recording: false, seconds: 0, megabytes: 0, frames: 0, fileName: "")
    /// Set when a recording stops: the view presents the export sheet for it.
    @Published var pendingShare: ShareItem?
    @Published var recordingError: String?
    @Published private(set) var preparingRecordingShare = false

    let sceneView = ARSCNView(frame: .zero)

    // MARK: Private

    private let engine = RotationEstimator()
    private let converter = FrameConverter()
    private let recorder = SessionRecorder()
    /// ARKit delivers frames here; conversion is quick and the ARFrame is released immediately.
    private let frameQueue = DispatchQueue(label: "ch.mariogeiger.wiggler.frames", qos: .userInteractive)
    /// The engine runs here; frames arriving while it is busy are dropped (never queued) so ARKit is never starved.
    private let engineQueue = DispatchQueue(label: "ch.mariogeiger.wiggler.engine", qos: .userInteractive)
    private var engineBusy = false
    private var videoFormat = "unknown"
    /// ARKit's delivery cadence and our conversion cost, measured on the frame queue for the recording.
    private var lastDeliveredTimestamp: Double?
    private var deliveryMillis = 0.0
    private var conversionMillis = 0.0
    private var droppedFrames = 0
    private var conversionFailures = 0
    private var processedFps = 0.0
    private var lastProcessedTimestamp: Double?
    private let overlay = AxisOverlayNode()
    private let harmonicOverlay = HarmonicOverlayNode()
    private let lock = NSLock()
    private var latestOutput = EngineOutput()
    private var latestHarmonicImage: CGImage?
    private var harmonicSignalSnapshot = HarmonicSignal.luma
    private var latestDisplayTransform = CGAffineTransform.identity
    private var viewportSize = CGSize(width: 1, height: 1)
    private var lastPublish = Date.distantPast
    // Engine queue. Only stable measurements contribute to the map, even when its display is off.
    private var harmonics = HarmonicFit(orders: ARSessionController.harmonicOrders)
    private var harmonicRevisionEngine = 0
    private var harmonicOrderEngine: Int?
    private var renderedHarmonicImage: CGImage?
    private var lastHarmonicRender = Date.distantPast

    override init() {
        super.init()
        recorder.onFailure = { [weak self] error in self?.reportRecordingFailure(error) }
        sceneView.delegate = self
        sceneView.session.delegate = self
        sceneView.session.delegateQueue = frameQueue
        sceneView.automaticallyUpdatesLighting = true
        sceneView.rendersContinuously = true
        sceneView.preferredFramesPerSecond = 60
        sceneView.scene.rootNode.addChildNode(overlay)  // the harmonic overlay attaches itself to the camera
        overlay.isHidden = true
        algorithm =
            UserDefaults.standard.string(forKey: Self.algorithmKey)
            .flatMap(RotationAlgorithm.init(rawValue:)) ?? .trackedGeometry
        engine.select(algorithm, revision: algorithmRevision)
        let savedOrder = UserDefaults.standard.integer(forKey: Self.harmonicOrderKey)
        harmonicOrder = Self.harmonicOrders.contains(savedOrder) ? savedOrder : nil
        harmonicOrderEngine = harmonicOrder
        harmonicSignal = HarmonicSignal(savedValue: UserDefaults.standard.string(forKey: Self.harmonicSignalKey))
        harmonics.select(harmonicSignal)
        harmonicSignalSnapshot = harmonicSignal
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
        videoFormat =
            "\(Int(config.videoFormat.imageResolution.width))x\(Int(config.videoFormat.imageResolution.height))@\(config.videoFormat.framesPerSecond)"
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        // A measurement session is hands-off by nature: the phone must not lock while the camera is running.
        UIApplication.shared.isIdleTimerDisabled = true
    }

    func pause() {
        sceneView.session.pause()
        UIApplication.shared.isIdleTimerDisabled = false
    }

    // MARK: Controls (main thread)

    func setAlgorithm(_ algorithm: RotationAlgorithm) {
        guard self.algorithm != algorithm else { return }
        self.algorithm = algorithm
        output = EngineOutput()
        harmonicProgress = 0
        lock.lock()
        algorithmRevision += 1
        let revision = algorithmRevision
        latestOutput = EngineOutput()
        latestHarmonicImage = nil
        engineQueue.async { [self] in
            engine.select(algorithm, revision: revision)
            harmonics = HarmonicFit(signal: harmonics.signal, orders: Self.harmonicOrders)
            renderedHarmonicImage = nil
            lastHarmonicRender = .distantPast
        }
        lock.unlock()
    }

    func setHarmonicOrder(_ l: Int?) {
        harmonicOrder = l
        enqueueHarmonicSelection()
    }

    func setHarmonicSignal(_ signal: HarmonicSignal) {
        guard harmonicSignal != signal else { return }
        harmonicSignal = signal
        harmonicProgress = 0
        enqueueHarmonicSelection()
    }

    /// Clear the render snapshot now; old in-flight frames cannot republish a prior selection.
    private func enqueueHarmonicSelection() {
        let signal = harmonicSignal, order = harmonicOrder
        lock.lock()
        harmonicRevision += 1
        harmonicSignalSnapshot = signal
        let revision = harmonicRevision
        latestHarmonicImage = nil
        lock.unlock()
        engineQueue.async { [self] in
            harmonics.select(signal)
            harmonicOrderEngine = order
            harmonicRevisionEngine = revision
            renderedHarmonicImage = nil
            lastHarmonicRender = .distantPast
        }
    }

    func toggleRecording() {
        guard !preparingRecordingShare else { return }
        preparingRecordingShare = true
        let deviceModel = UIDevice.current.model
        engineQueue.async { [self] in
            if recorder.isRecording {
                recorder.stop()
                finishRecording()
                return
            }
            recorder.start(
                width: FrameConverter.engineWidth, height: FrameConverter.engineHeight,
                context: [
                    "deviceModel": deviceModel,
                    "videoFormat": videoFormat,
                    "harmonicOrders": Self.harmonicOrders,
                    "algorithm": engine.algorithm.rawValue,
                    "algorithmRevision": engine.revision,
                    "posePolicy": "normal and limited(excessiveMotion/insufficientFeatures) accepted",
                ])
            let status = recorder.status(now: lastProcessedTimestamp ?? 0)
            DispatchQueue.main.async { [weak self] in
                self?.recorderStatus = status
                self?.recordingError = status.error
                self?.preparingRecordingShare = false
            }
        }
    }

    /// Engine queue, after the recorder has closed its file (by the button or by its byte limit): share it as is.
    private func finishRecording() {
        let status = recorder.status(now: lastProcessedTimestamp ?? 0)
        let shareURL = status.error == nil ? recorder.url : nil
        DispatchQueue.main.async { [weak self] in
            self?.recorderStatus = status
            self?.recordingError = status.error
            self?.preparingRecordingShare = false
            if let shareURL { self?.pendingShare = ShareItem(urls: [shareURL]) }
        }
    }

    private func reportRecordingFailure(_ error: String) {
        engineQueue.async { [weak self] in
            guard let self else { return }
            let status = recorder.status(now: lastProcessedTimestamp ?? 0)
            DispatchQueue.main.async { [weak self] in
                self?.recorderStatus = status
                self?.recordingError = error
            }
        }
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
        deliveryMillis = lastDeliveredTimestamp.map { (frame.timestamp - $0) * 1000 } ?? 0
        lastDeliveredTimestamp = frame.timestamp
        lock.lock()
        let busy = engineBusy
        if !busy { engineBusy = true }
        let signal = harmonicSignalSnapshot
        let inputRevision = harmonicRevision
        let inputAlgorithmRevision = algorithmRevision
        let size = viewportSize
        lock.unlock()
        if busy {
            droppedFrames += 1
            return
        }
        let conversionStart = Date()
        guard let input = converter.convert(frame, harmonicSignal: signal) else {
            conversionFailures += 1
            lock.lock()
            engineBusy = false
            lock.unlock()
            return
        }
        conversionMillis = Date().timeIntervalSince(conversionStart) * 1000
        let luma8 = converter.lastLuma8
        let dropped = droppedFrames, failedConversions = conversionFailures
        let delivery = deliveryMillis, conversion = conversionMillis
        let cameraTrackingState = String(describing: frame.camera.trackingState)
        let transform = frame.displayTransform(for: .portrait, viewportSize: size)
        // Nothing below touches `frame` any more.
        engineQueue.async { [self] in
            guard inputAlgorithmRevision == engine.revision else {
                lock.lock()
                engineBusy = false
                lock.unlock()
                return
            }
            if let lt = lastProcessedTimestamp, input.timestamp > lt {
                processedFps += 0.1 * (1.0 / (input.timestamp - lt) - processedFps)
            }
            lastProcessedTimestamp = input.timestamp
            let recording = recorder.isRecording
            let out = engine.process(input)
            updateHarmonics(input: inputRevision == harmonicRevisionEngine ? input : nil, output: out)
            lock.lock()
            if engine.revision == algorithmRevision {
                latestOutput = out
                if harmonicRevisionEngine == harmonicRevision {
                    latestHarmonicImage = renderedHarmonicImage
                }
                latestDisplayTransform = transform
            }
            engineBusy = false
            lock.unlock()
            if recording {
                let appended = recorder.append(
                    input: input, luma8: luma8, output: out, config: engine.config,
                    settings: RecordingSettings(
                        algorithm: engine.algorithm, algorithmRevision: engine.revision,
                        harmonicSignal: harmonics.signal.rawValue, harmonicOrder: harmonicOrderEngine,
                        harmonicRevision: harmonicRevisionEngine, inputRevision: inputRevision,
                        convertedSignal: signal.rawValue,
                        harmonicInputAccepted: inputRevision == harmonicRevisionEngine,
                        harmonicTurnProgress: harmonics.turnProgress, cameraTrackingState: cameraTrackingState,
                        deliveryMillis: delivery, conversionMillis: conversion),
                    droppedFrames: dropped, conversionFailures: failedConversions, processedFps: processedFps)
                if !appended { finishRecording() }
            }
            let now = Date()
            if now.timeIntervalSince(lastPublish) > 1.0 / 30.0 {
                lastPublish = now
                let status = recorder.status(now: input.timestamp)
                let progress = harmonics.turnProgress
                let revision = harmonicRevisionEngine
                let outputAlgorithmRevision = engine.revision
                DispatchQueue.main.async { [weak self] in
                    guard let self else { return }
                    if algorithmRevision == outputAlgorithmRevision {
                        output = out
                        if harmonicRevision == revision { harmonicProgress = progress }
                    }
                    recorderStatus = status
                }
            }
        }
    }

    /// Engine queue. Feeds the harmonic map and publishes its rendering at ≤ 15 Hz while it is meaningful.
    private func updateHarmonics(input: FrameInput?, output out: EngineOutput) {
        harmonics.update(frame: input, output: out)
        guard let map = harmonics.map, map.hasFullTurn, let order = harmonicOrderEngine else {
            renderedHarmonicImage = nil
            return
        }
        let now = Date()
        guard now.timeIntervalSince(lastHarmonicRender) > 1.0 / 15.0 else { return }
        lastHarmonicRender = now
        if let rgba = map.render(order: order, theta: out.theta, fullScale: harmonics.signal.fullScale) {
            renderedHarmonicImage = CGImage.rgba8(width: map.width, height: map.height, bytes: rgba)
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
