import SceneKit
import UIKit
import WigglerCore

/// SceneKit node group drawn at the estimated rotation axis: a thin axis line and a "ray" (half-plane) that turns
/// with the object. Both run ±2 m along the axis, i.e. "to infinity" on screen.
final class AxisOverlayNode: SCNNode {
    private let axisLine = SCNNode()
    private let ray = SCNNode()
    private let rayPivot = SCNNode()
    private let span: Float = 4
    private var lastRadius = -1.0

    override init() {
        super.init()
        let lineMat = SCNMaterial()
        lineMat.diffuse.contents = UIColor.cyan
        lineMat.emission.contents = UIColor.cyan
        lineMat.lightingModel = .constant
        axisLine.geometry = SCNCylinder(radius: 0.002, height: CGFloat(span))
        axisLine.geometry?.materials = [lineMat]

        let rayMat = SCNMaterial()
        rayMat.diffuse.contents = UIColor.orange.withAlphaComponent(0.85)
        rayMat.emission.contents = UIColor.orange
        rayMat.lightingModel = .constant
        rayMat.isDoubleSided = true
        rayMat.writesToDepthBuffer = false
        // A thin half-plane: unit box scaled to (radius, span, ~0), shifted so it starts at the axis.
        ray.geometry = SCNBox(width: 1, height: 1, length: 0.0015, chamferRadius: 0)
        ray.geometry?.materials = [rayMat]
        rayPivot.addChildNode(ray)

        addChildNode(axisLine)
        addChildNode(rayPivot)
        renderingOrder = 10
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    /// Called on the SceneKit render thread. `hidden` withdraws the whole node (another overlay has the screen).
    func update(with out: EngineOutput, hidden: Bool) {
        guard !hidden, let axis = out.axis, out.state != .idle else {
            isHidden = true
            return
        }
        isHidden = false
        // Local frame: x = e1, y = axis direction, z = -e2 (right-handed). A rotation of +θ about +y takes x toward
        // -z = e2, which is exactly how the engine measures its azimuth φ (from e1 toward e2).
        let x = axis.e1, y = axis.direction, z = -axis.e2, o = axis.origin
        transform = SCNMatrix4(
            m11: Float(x.x), m12: Float(x.y), m13: Float(x.z), m14: 0,
            m21: Float(y.x), m22: Float(y.y), m23: Float(y.z), m24: 0,
            m31: Float(z.x), m32: Float(z.y), m33: Float(z.z), m34: 0,
            m41: Float(o.x), m42: Float(o.y), m43: Float(o.z), m44: 1)

        let radius = max(0.03, out.objectRadius) * 1.35
        if abs(radius - lastRadius) > 0.002 {
            lastRadius = radius
            ray.scale = SCNVector3(Float(radius), span, 1)
            ray.position = SCNVector3(Float(0.5 * radius), 0, 0)
        }
        let visible = out.state == .locked && out.angleConfidence > 0.15
        rayPivot.isHidden = !visible
        rayPivot.eulerAngles = SCNVector3(0, Float(out.theta), 0)
        ray.geometry?.firstMaterial?.diffuse.contents = UIColor.orange.withAlphaComponent(CGFloat(0.35 + 0.6 * out.angleConfidence))
        axisLine.geometry?.firstMaterial?.diffuse.contents = UIColor.cyan.withAlphaComponent(CGFloat(0.4 + 0.6 * out.axisQuality))
    }
}
