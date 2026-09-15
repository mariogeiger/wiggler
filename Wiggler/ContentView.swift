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
        GeometryReader { safeArea in
            GeometryReader { viewport in
                ZStack(alignment: .top) {
                    ARViewContainer(controller: controller)

                    pointsOverlay
                        .allowsHitTesting(false)

                    controls
                        .padding(.top, safeArea.safeAreaInsets.top + 4)
                }
                .overlay(alignment: .topTrailing) {
                    measurementDials
                        .padding(.top, safeArea.safeAreaInsets.top + 44)
                        .padding(.trailing, 8)
                }
                .onAppear { controller.updateViewportSize(viewport.size) }
                .onChange(of: viewport.size) { _, size in controller.updateViewportSize(size) }
            }
            .ignoresSafeArea()
        }
        .onAppear { controller.start() }
        .onDisappear { controller.pause() }
    }

    private var controls: some View {
        FittingHorizontalPadding(maximum: 8) {
            Grid(alignment: .leading, horizontalSpacing: 0, verticalSpacing: 8) {
                GridRow {
                    harmonicControl.fixedSize()
                    recordButton
                        .fixedSize()
                        .frame(maxWidth: .infinity, alignment: .trailing)
                        .gridColumnAlignment(.trailing)
                }
                GridRow {
                    harmonicSignalControl.fixedSize()
                }
                GridRow {
                    algorithmControl.fixedSize()
                }
            }
        }
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

    private var measurementDials: some View {
        let output = controller.output
        let hasMeasurement = output.state == .locked && output.angleConfidence > 0
        return HStack(spacing: 8) {
            RotationDial(scale: .angle, value: hasMeasurement ? output.angleDegrees : nil)
            RotationDial(scale: .rpm, value: hasMeasurement ? output.rpm : nil)
        }
        .fixedSize()
    }

    private var algorithmControl: some View {
        HStack(spacing: 2) {
            ForEach(RotationAlgorithm.allCases, id: \.self) { algorithm in
                selectionButton(algorithm.shortLabel, selected: controller.algorithm == algorithm) {
                    controller.setAlgorithm(algorithm)
                }
                .accessibilityLabel(algorithm.label)
            }
        }
        .padding(3)
        .background(.black.opacity(0.35), in: Capsule())
    }

    private var harmonicSignalControl: some View {
        HStack(spacing: 2) {
            ForEach(HarmonicSignal.allCases, id: \.self) { signal in
                let available = signal != .depth || ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
                selectionButton(signal.shortLabel, selected: controller.harmonicSignal == signal) {
                    controller.setHarmonicSignal(signal)
                }
                .disabled(!available)
                .opacity(available ? 1 : 0.4)
                .accessibilityLabel(signal.label)
            }
        }
        .padding(3)
        .background(.black.opacity(0.35), in: Capsule())
    }

    /// Which harmonic of the current image is drawn (see `HarmonicMap`).
    private var harmonicControl: some View {
        HStack(spacing: 2) {
            harmonicButton(nil, "off")
            ForEach(ARSessionController.harmonicOrders, id: \.self) { harmonicButton($0, "l=\($0)") }
        }
        .padding(3)
        .background(.black.opacity(0.35), in: Capsule())
    }

    private func harmonicButton(_ l: Int?, _ label: String) -> some View {
        selectionButton(label, selected: controller.harmonicOrder == l) {
            controller.setHarmonicOrder(l)
        }
    }

    private func selectionButton(_ label: String, selected: Bool, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Text(label)
                .font(.caption2.monospaced())
                .foregroundStyle(selected ? .black : .white.opacity(0.8))
                .padding(.horizontal, 8).padding(.vertical, 6)
                .background(selected ? Color.white.opacity(0.85) : .clear, in: Capsule())
                .contentShape(Capsule())
        }
        .buttonStyle(.plain)
        .accessibilityAddTraits(selected ? .isSelected : [])
    }

    private var recordButton: some View {
        Button {
            controller.toggleRecording()
        } label: {
            let st = controller.recorderStatus
            HStack(spacing: 6) {
                Circle().fill(st.recording ? Color.red : Color.white.opacity(0.7)).frame(width: 14, height: 14)
                if controller.preparingRecordingShare {
                    ProgressView().tint(.white)
                    Text("Preparing…").font(.caption).foregroundStyle(.white)
                } else if st.recording {
                    Text(String(format: "%.0f s", st.seconds)).font(.caption.monospacedDigit()).foregroundStyle(.white)
                }
            }
            .padding(8)
            .background(.black.opacity(0.35), in: Capsule())
        }
        .buttonStyle(.plain)
        .disabled(controller.preparingRecordingShare)
        .sheet(item: $controller.pendingShare) { item in
            // Export prompt opened automatically when a recording stops.
            ShareSheet(items: item.urls)
        }
        .alert(
            "Recording failed",
            isPresented: Binding(
                get: { controller.recordingError != nil },
                set: { if !$0 { controller.recordingError = nil } }
            )
        ) {
            Button("OK", role: .cancel) { controller.recordingError = nil }
        } message: {
            Text(controller.recordingError ?? "")
        }
    }
}

/// Spend spare width on outer margins, never by shrinking the controls inside them.
private struct FittingHorizontalPadding: Layout {
    let maximum: CGFloat

    private func inset(width: CGFloat?, content: LayoutSubview) -> CGFloat {
        guard let width else { return maximum }
        let minimumWidth = content.sizeThatFits(.unspecified).width
        return min(maximum, max(0, (width - minimumWidth) / 2))
    }

    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        precondition(subviews.count == 1)
        let padding = inset(width: proposal.width, content: subviews[0])
        let size = subviews[0].sizeThatFits(
            ProposedViewSize(width: proposal.width.map { max(0, $0 - 2 * padding) }, height: proposal.height))
        return CGSize(width: size.width + 2 * padding, height: size.height)
    }

    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        precondition(subviews.count == 1)
        let padding = inset(width: bounds.width, content: subviews[0])
        subviews[0].place(
            at: CGPoint(x: bounds.minX + padding, y: bounds.minY), anchor: .topLeading,
            proposal: ProposedViewSize(width: max(0, bounds.width - 2 * padding), height: bounds.height))
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
