import ARKit
import SwiftUI
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

                // Everything lives at the top so the hand turning the object never covers a control.
                VStack(spacing: 10) {
                    HStack(alignment: .top) {
                        rpmText
                        Spacer()
                        angleText
                    }
                    .padding(.horizontal, 16)
                    .padding(.top, 56)
                    HStack {
                        pointCountControl
                        Spacer()
                        recordButton
                    }
                    .padding(.horizontal, 20)
                    HStack {
                        harmonicControl
                        Spacer()
                    }
                    .padding(.horizontal, 20)
                    Spacer()
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
                ctx.fill(
                    Path(ellipseIn: CGRect(x: p.x - r, y: p.y - r, width: 2 * r, height: 2 * r)), with: .color(color))
            }
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
            Button {
                controller.adjustTrackCount(by: -20)
            } label: {
                Image(systemName: "minus").frame(width: 36, height: 36).contentShape(Rectangle())
            }
            Text("\(controller.targetTrackCount) pts").font(.caption.monospacedDigit()).foregroundStyle(
                .white.opacity(0.8))
            Button {
                controller.adjustTrackCount(by: 20)
            } label: {
                Image(systemName: "plus").frame(width: 36, height: 36).contentShape(Rectangle())
            }
        }
        .buttonStyle(.plain)
        .foregroundStyle(.white.opacity(0.8))
        .padding(.horizontal, 6)
        .background(.black.opacity(0.35), in: Capsule())
    }

    /// Which harmonic of the current image is drawn (see `HarmonicMap`).
    private var harmonicControl: some View {
        HStack(spacing: 2) {
            harmonicButton(nil, "off")
            ForEach(ARSessionController.harmonicOrders, id: \.self) { harmonicButton($0, "l=\($0)") }
            if controller.harmonicOrder != nil, controller.output.state == .locked, controller.harmonicProgress < 1 {
                // First turn not yet accumulated: the overlay appears at 100 %.
                Text("\(Int(controller.harmonicProgress * 100)) %")
                    .font(.caption2.monospacedDigit())
                    .foregroundStyle(.white.opacity(0.6))
                    .padding(.horizontal, 6)
            }
        }
        .padding(3)
        .background(.black.opacity(0.35), in: Capsule())
    }

    private func harmonicButton(_ l: Int?, _ label: String) -> some View {
        let selected = controller.harmonicOrder == l
        return Button {
            controller.setHarmonicOrder(l)
        } label: {
            Text(label)
                .font(.caption2.monospaced())
                .foregroundStyle(selected ? .black : .white.opacity(0.8))
                .padding(.horizontal, 8).padding(.vertical, 6)
                .background(selected ? Color.white.opacity(0.85) : .clear, in: Capsule())
                .contentShape(Capsule())
        }
        .buttonStyle(.plain)
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
