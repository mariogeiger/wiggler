import SwiftUI
import ARKit
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

struct ContentView: View {
    @StateObject private var controller = ARSessionController()
    @State private var dragPoint: CGPoint?

    var body: some View {
        GeometryReader { geo in
            ZStack {
                ARViewContainer(controller: controller)
                    .ignoresSafeArea()
                    .onAppear { controller.updateViewportSize(geo.size) }
                    .onChange(of: geo.size) { _, s in controller.updateViewportSize(s) }

                pointsOverlay
                    .allowsHitTesting(false)

                markerOverlay
                    .allowsHitTesting(false)

                // Touch layer: tap or drag to (re)place the marker.
                Color.clear
                    .contentShape(Rectangle())
                    .gesture(
                        DragGesture(minimumDistance: 0, coordinateSpace: .local)
                            .onChanged { v in dragPoint = v.location }
                            .onEnded { v in
                                dragPoint = nil
                                controller.placeMarker(viewPoint: v.location)
                            }
                    )

                VStack {
                    hud
                    Spacer()
                    controls
                }
                .padding()
            }
            .ignoresSafeArea()
        }
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
            var center: CGPoint?
            if let d = dragPoint {
                center = d
            } else if let m = controller.markerImagePoint {
                center = controller.viewPoint(imageX: m.x, imageY: m.y)
            }
            guard let c = center else { return }
            var circle = Path()
            circle.move(to: CGPoint(x: c.x - 12, y: c.y)); circle.addLine(to: CGPoint(x: c.x + 12, y: c.y))
            circle.move(to: CGPoint(x: c.x, y: c.y - 12)); circle.addLine(to: CGPoint(x: c.x, y: c.y + 12))
            ctx.stroke(circle, with: .color(.cyan), lineWidth: 2)
        }
    }

    // MARK: HUD

    private var hud: some View {
        let o = controller.output
        return VStack(alignment: .leading, spacing: 6) {
            HStack(alignment: .firstTextBaseline) {
                Text(o.state == .locked && o.angleConfidence > 0 ? String(format: "%.0f°", o.angleDegrees) : "—")
                    .font(.system(size: 54, weight: .bold, design: .rounded).monospacedDigit())
                    .foregroundStyle(o.angleConfidence > 0.5 ? .orange : .orange.opacity(0.5))
                VStack(alignment: .leading, spacing: 2) {
                    Text(String(format: "%+.0f rpm", o.rpm)).font(.title3.monospacedDigit())
                    Text(periodText(o)).font(.caption)
                }
                Spacer()
                stateBadge(o.state)
            }
            Text(controller.sessionMessage.isEmpty ? o.message : controller.sessionMessage)
                .font(.subheadline)
                .foregroundStyle(.white.opacity(0.9))
            HStack(spacing: 12) {
                gauge("axe", o.axisQuality, .cyan)
                gauge("angle", o.angleConfidence, .orange)
                gauge("aspect", o.relocalizerFill, .green)
            }
            Text(String(format: "%d pts · %d ok · %d cordes · %.1f ms · disp %.0f°", o.trackCount, o.inlierCount, o.constraintCount, o.processingMillis, o.angleDispersionDegrees))
                .font(.caption2.monospacedDigit())
                .foregroundStyle(.white.opacity(0.6))
            Text(controller.statsLine)
                .font(.caption2.monospacedDigit())
                .foregroundStyle(.white.opacity(0.6))
        }
        .padding(12)
        .background(.black.opacity(0.45), in: RoundedRectangle(cornerRadius: 14))
        .padding(.top, 40)
    }

    private func periodText(_ o: EngineOutput) -> String {
        if o.state != .locked { return "" }
        if o.periodDegrees <= 0 { return "symétrique : angle relatif" }
        if o.periodDegrees < 359 { return String(format: "aspect périodique : %.0f°", o.periodDegrees) }
        return "aspect unique sur 360°"
    }

    private func stateBadge(_ s: EngineState) -> some View {
        let (label, color): (String, Color) = {
            switch s {
            case .idle: return ("prêt", .gray)
            case .calibrating: return ("calibration", .yellow)
            case .locked: return ("verrouillé", .green)
            case .lost: return ("perdu", .red)
            }
        }()
        return Text(label)
            .font(.caption.bold())
            .padding(.horizontal, 8).padding(.vertical, 4)
            .background(color.opacity(0.3), in: Capsule())
            .overlay(Capsule().stroke(color, lineWidth: 1))
    }

    private func gauge(_ name: String, _ value: Double, _ color: Color) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(name).font(.caption2).foregroundStyle(.white.opacity(0.7))
            GeometryReader { g in
                ZStack(alignment: .leading) {
                    Capsule().fill(.white.opacity(0.15))
                    Capsule().fill(color).frame(width: g.size.width * max(0, min(1, value)))
                }
            }
            .frame(height: 6)
        }
    }

    // MARK: Controls

    private var controls: some View {
        VStack(spacing: 10) {
            HStack {
                Toggle("points", isOn: $controller.showPoints).toggleStyle(.button)
                Button {
                    controller.toggleRecording()
                } label: {
                    let st = controller.recorderStatus
                    if st.recording {
                        Label(String(format: "Stop  %.0f s · %.0f Mo", st.seconds, st.megabytes), systemImage: "stop.circle.fill")
                            .monospacedDigit()
                    } else {
                        Label("Enregistrer" + (controller.recordingCount > 0 ? " (\(controller.recordingCount))" : ""), systemImage: "record.circle")
                    }
                }
                .buttonStyle(.borderedProminent)
                .tint(controller.recorderStatus.recording ? .red : .accentColor)
                if controller.recordingCount > 0 && !controller.recorderStatus.recording {
                    ShareLink(items: SessionRecorder.recordings()) {
                        Image(systemName: "square.and.arrow.up")
                    }
                    .buttonStyle(.bordered)
                    Button(role: .destructive) { controller.deleteRecordings() } label: {
                        Image(systemName: "trash")
                    }
                    .buttonStyle(.bordered)
                }
                Spacer()
                Button(role: .destructive) { controller.clearMarker() } label: {
                    Label("Réinitialiser", systemImage: "arrow.counterclockwise")
                }
                .buttonStyle(.bordered)
            }
        }
        .padding(12)
        .background(.black.opacity(0.45), in: RoundedRectangle(cornerRadius: 14))
        .padding(.bottom, 20)
    }
}
