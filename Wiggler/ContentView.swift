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
                    .onAppear { controller.updateViewportSize(geo.size) }
                    .onChange(of: geo.size) { _, s in controller.updateViewportSize(s) }

                pointsOverlay
                    .allowsHitTesting(false)

                // Touch layer: tap or drag to (re)place the marker.
                Color.clear
                    .contentShape(Rectangle())
                    .gesture(
                        DragGesture(minimumDistance: 0, coordinateSpace: .local)
                            .onEnded { v in controller.placeMarker(viewPoint: v.location) }
                    )

                VStack {
                    Spacer()
                    HStack {
                        Spacer()
                        recordButton
                    }
                    .padding(.trailing, 20)
                    .padding(.bottom, 40)
                }
            }
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

    // MARK: Recording (small, discreet)

    private var recordButton: some View {
        HStack(spacing: 10) {
            if controller.recordingCount > 0 && !controller.recorderStatus.recording {
                ShareLink(items: SessionRecorder.recordings()) {
                    Image(systemName: "square.and.arrow.up").font(.title3)
                }
                .buttonStyle(.plain)
                .foregroundStyle(.white.opacity(0.7))
            }
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
        }
    }
}
