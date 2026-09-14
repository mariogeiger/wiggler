import ARKit
import SceneKit
import SwiftUI
import Combine
import WigglerCore

/// Owns the AR session, feeds every frame to the rotation engine and drives the SceneKit overlay.
final class ARSessionController: NSObject, ObservableObject, ARSessionDelegate, ARSCNViewDelegate {
    let sceneView = ARSCNView(frame: .zero)

    /// Latest engine output for the HUD (published at ~30 Hz on the main thread).
    @Published private(set) var output = EngineOutput()
    /// Marker in engine image coordinates (pixels of the 480x360 landscape image), nil when not placed.
    @Published private(set) var markerImagePoint: CGPoint?
    /// Region-of-interest radius as a fraction of the engine image height.
    @Published var roiFraction: Double = 0.25 { didSet { applyMarker() } }
    @Published var showPoints = true
    @Published var sessionMessage = ""

    private let engine = RotationEngine()
    private let converter = FrameConverter()
    private let frameQueue = DispatchQueue(label: "ch.mariogeiger.wiggler.frames", qos: .userInteractive)
    private let overlay = AxisOverlayNode()
    private let lock = NSLock()
    private var latestOutput = EngineOutput()
    private var latestDisplayTransform = CGAffineTransform.identity
    private var viewportSize = CGSize(width: 1, height: 1)
    private var lastPublish = Date.distantPast

    override init() {
        super.init()
        sceneView.delegate = self
        sceneView.session.delegate = self
        sceneView.session.delegateQueue = frameQueue
        sceneView.automaticallyUpdatesLighting = true
        sceneView.rendersContinuously = true
        sceneView.scene.rootNode.addChildNode(overlay)
        overlay.isHidden = true
    }

    func start() {
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
        // Prefer the 60 fps 4:3 format of the wide camera.
        let formats = ARWorldTrackingConfiguration.supportedVideoFormats
        if let f = formats.first(where: { $0.framesPerSecond >= 60 && $0.imageResolution.width <= 2000 })
            ?? formats.first(where: { $0.framesPerSecond >= 60 }) {
            config.videoFormat = f
        }
        sceneView.session.run(config, options: [.resetTracking, .removeExistingAnchors])
    }

    func pause() { sceneView.session.pause() }

    // MARK: Marker handling (main thread)

    /// Called with a touch location in view coordinates.
    func placeMarker(viewPoint: CGPoint) {
        lock.lock()
        let t = latestDisplayTransform
        let size = viewportSize
        lock.unlock()
        guard size.width > 1, size.height > 1 else { return }
        let normalizedView = CGPoint(x: viewPoint.x / size.width, y: viewPoint.y / size.height)
        let normalizedImage = normalizedView.applying(t.inverted())
        let p = CGPoint(x: normalizedImage.x * CGFloat(FrameConverter.engineWidth),
                        y: normalizedImage.y * CGFloat(FrameConverter.engineHeight))
        markerImagePoint = p
        applyMarker()
    }

    func clearMarker() {
        markerImagePoint = nil
        frameQueue.async { [engine] in engine.clearMarker() }
    }

    private func applyMarker() {
        guard let p = markerImagePoint else { return }
        let radius = Float(roiFraction * Double(FrameConverter.engineHeight))
        frameQueue.async { [engine] in
            engine.setMarker(x: Float(p.x), y: Float(p.y), radius: radius)
        }
    }

    /// Engine image pixel → view coordinates (for the debug overlay).
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
        guard let input = converter.convert(frame) else { return }
        lock.lock()
        let size = viewportSize
        lock.unlock()
        let transform = frame.displayTransform(for: .portrait, viewportSize: size)
        var out = engine.process(input)
        if frame.sceneDepth == nil && !out.message.isEmpty && out.state != .idle {
            out.message = "Profondeur LiDAR absente — " + out.message
        }
        lock.lock()
        latestOutput = out
        latestDisplayTransform = transform
        lock.unlock()
        let now = Date()
        if now.timeIntervalSince(lastPublish) > 1.0 / 30.0 {
            lastPublish = now
            DispatchQueue.main.async { [weak self] in self?.output = out }
        }
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
