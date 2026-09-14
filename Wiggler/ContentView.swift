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

    var body: some View {
        GeometryReader { geo in
            ZStack {
                ARViewContainer(controller: controller)

                pointsOverlay
                    .allowsHitTesting(false)
                markerOverlay
                    .allowsHitTesting(false)

                // A tap places the marker; the controls above keep their own taps.
                Color.clear
                    .contentShape(Rectangle())
                    .onTapGesture(coordinateSpace: .local) { p in controller.placeMarker(viewPoint: p) }

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
                        pointCountControl
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

    // MARK: Readouts and controls

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

    private var pointCountControl: some View {
        HStack(spacing: 10) {
            Button { controller.adjustTrackCount(by: -20) } label: {
                Image(systemName: "minus").frame(width: 36, height: 36).contentShape(Rectangle())
            }
            Text("\(controller.targetTrackCount) pts").font(.caption.monospacedDigit()).foregroundStyle(.white.opacity(0.8))
            Button { controller.adjustTrackCount(by: 20) } label: {
                Image(systemName: "plus").frame(width: 36, height: 36).contentShape(Rectangle())
            }
        }
        .buttonStyle(.plain)
        .foregroundStyle(.white.opacity(0.8))
        .padding(.horizontal, 6)
        .background(.black.opacity(0.35), in: Capsule())
    }

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
            // Export prompt opened automatically when a recording stops (all parts of the session).
            ShareSheet(items: item.urls)
        }
    }
}

struct ShareItem: Identifiable {
    let urls: [URL]
    var id: String { urls.map(\.path).joined(separator: "|") }
}

struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: items, applicationActivities: nil)
    }
    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}
