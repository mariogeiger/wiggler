import SwiftUI
import ARKit
import AVFoundation
import WigglerCore

struct ARViewContainer: UIViewRepresentable {
    let controller: ARSessionController

    func makeUIView(context: Context) -> ARSCNView {
        let v = controller.sceneView
        v.backgroundColor = .black
        return v
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}

/// Plain camera preview for the AVFoundation path.
final class PreviewView: UIView {
    var previewLayer: AVCaptureVideoPreviewLayer? {
        didSet {
            oldValue?.removeFromSuperlayer()
            if let l = previewLayer { layer.addSublayer(l) }
            setNeedsLayout()
        }
    }
    override func layoutSubviews() {
        super.layoutSubviews()
        previewLayer?.frame = bounds
    }
}

struct PreviewContainer: UIViewRepresentable {
    let controller: ARSessionController

    func makeUIView(context: Context) -> PreviewView {
        let v = PreviewView()
        v.backgroundColor = .black
        v.previewLayer = controller.previewLayer
        return v
    }

    func updateUIView(_ uiView: PreviewView, context: Context) {}
}

struct ContentView: View {
    @StateObject private var controller = ARSessionController()

    var body: some View {
        GeometryReader { geo in
            ZStack {
                if controller.useARKit {
                    ARViewContainer(controller: controller)
                } else {
                    PreviewContainer(controller: controller)
                    axisOverlay2D
                        .allowsHitTesting(false)
                }

                pointsOverlay
                    .allowsHitTesting(false)

                markerOverlay
                    .allowsHitTesting(false)

                // Touch layer: tap or drag to (re)place the marker.
                Color.clear
                    .contentShape(Rectangle())
                    .gesture(
                        DragGesture(minimumDistance: 0, coordinateSpace: .local)
                            .onEnded { v in controller.placeMarker(viewPoint: v.location) }
                    )

                VStack {
                    HStack(alignment: .top) {
                        rpmText
                        Spacer()
                        angleText
                    }
                    .padding(.horizontal, 16)
                    .padding(.top, 56)
                    Spacer()
                    HStack {
                        arkitSwitch
                        Spacer()
                        recordButton
                    }
                    .padding(.horizontal, 20)
                    .padding(.bottom, 40)
                }
            }
            .onAppear { controller.updateViewportSize(geo.size) }
            .onChange(of: geo.size) { _, s in controller.updateViewportSize(s) }
        }
        .ignoresSafeArea()
        .onAppear { controller.start() }
        .onDisappear { controller.pause() }
    }

    // MARK: Overlays

    private var pointsOverlay: some View {
        Canvas { ctx, _ in
            guard controller.showPoints else { return }
            for t in controller.output.tracks {
                let p = controller.viewPoint(imageX: CGFloat(t.x), imageY: CGFloat(t.y))
                let color: Color
                switch t.status {
                case .young: color = .white.opacity(0.5)
                case .good: color = .green
                case .inconsistent: color = .red
                case .noDepth: color = .yellow
                }
                let r: CGFloat = 2.5
                ctx.fill(Path(ellipseIn: CGRect(x: p.x - r, y: p.y - r, width: 2 * r, height: 2 * r)), with: .color(color))
            }
        }
    }

    private var markerOverlay: some View {
        Canvas { ctx, _ in
            guard let m = controller.markerImagePoint else { return }
            let c = controller.viewPoint(imageX: m.x, imageY: m.y)
            var cross = Path()
            cross.move(to: CGPoint(x: c.x - 12, y: c.y)); cross.addLine(to: CGPoint(x: c.x + 12, y: c.y))
            cross.move(to: CGPoint(x: c.x, y: c.y - 12)); cross.addLine(to: CGPoint(x: c.x, y: c.y + 12))
            ctx.stroke(cross, with: .color(.cyan), lineWidth: 2)
        }
    }

    /// Axis line + rotating half-plane, projected with the camera intrinsics (AVFoundation path).
    private var axisOverlay2D: some View {
        Canvas { ctx, _ in
            let o = controller.output
            guard let axis = o.axis, o.state != .idle else { return }
            let span = 4.0
            let n = 48
            // Axis line, clipped to the part in front of the camera by sampling.
            var line = Path()
            var pen = false
            for i in 0...n {
                let s = -span + 2 * span * Double(i) / Double(n)
                if let p = controller.projectToView(axis.origin + axis.direction * s) {
                    if pen { line.addLine(to: p) } else { line.move(to: p); pen = true }
                } else {
                    pen = false
                }
            }
            ctx.stroke(line, with: .color(.cyan.opacity(0.4 + 0.6 * o.axisQuality)), lineWidth: 2)

            guard o.state == .locked, o.angleConfidence > 0.15 else { return }
            let radius = max(0.03, o.objectRadius) * 1.35
            let radial = (axis.e1 * cos(o.theta) + axis.e2 * sin(o.theta)) * radius
            // Half-plane between the axis and its parallel at `radial`, filled piecewise.
            var quad = Path()
            var prevA: CGPoint?
            var prevB: CGPoint?
            for i in 0...n {
                let s = -span + 2 * span * Double(i) / Double(n)
                let a = controller.projectToView(axis.origin + axis.direction * s)
                let b = controller.projectToView(axis.origin + axis.direction * s + radial)
                if let a = a, let b = b, let pa = prevA, let pb = prevB {
                    quad.move(to: pa); quad.addLine(to: pb); quad.addLine(to: b); quad.addLine(to: a); quad.closeSubpath()
                }
                prevA = a
                prevB = b
            }
            ctx.fill(quad, with: .color(.orange.opacity(0.35 + 0.5 * o.angleConfidence)))
        }
    }

    // MARK: Discreet readouts

    private var rpmText: some View {
        let o = controller.output
        return Text(o.state == .locked ? String(format: "%+.0f rpm", o.rpm) : "")
            .font(.caption.monospacedDigit())
            .foregroundStyle(.white.opacity(0.8))
    }

    private var angleText: some View {
        let o = controller.output
        return Text(o.state == .locked && o.angleConfidence > 0 ? String(format: "%.0f°", o.angleDegrees) : "")
            .font(.caption.monospacedDigit())
            .foregroundStyle(.white.opacity(0.8))
    }

    private var arkitSwitch: some View {
        Toggle(isOn: $controller.useARKit) {
            Text("ARKit").font(.caption).foregroundStyle(.white.opacity(0.8))
        }
        .toggleStyle(.switch)
        .tint(.cyan)
        .scaleEffect(0.75, anchor: .leading)
        .fixedSize()
    }

    // MARK: Recording (small, discreet)

    private var recordButton: some View {
        Button {
            controller.toggleRecording()
        } label: {
            let st = controller.recorderStatus
            HStack(spacing: 6) {
                Circle().fill(st.recording ? Color.red : Color.white.opacity(0.7)).frame(width: 14, height: 14)
                if st.recording {
                    Text(String(format: "%.0f s", st.seconds)).font(.caption.monospacedDigit()).foregroundStyle(.white)
                }
            }
            .padding(8)
            .background(.black.opacity(0.35), in: Capsule())
        }
        .buttonStyle(.plain)
        .sheet(item: $controller.pendingShare) { item in
            // Export prompt opened automatically when a recording stops.
            ShareSheet(items: [item.url])
        }
    }
}

struct ShareItem: Identifiable {
    let url: URL
    var id: String { url.path }
}

struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: items, applicationActivities: nil)
    }
    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}
