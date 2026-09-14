import SceneKit
import UIKit
import WigglerCore

/// SceneKit node group drawn at the estimated rotation axis:
/// a thin axis line, a translucent cylinder hugging the object, and a "ray" (half-plane) that turns with it.
final class AxisOverlayNode: SCNNode {
    private let axisLine = SCNNode()
    private let cylinder = SCNNode()
    private let baseRing = SCNNode()
    private let ray = SCNNode()
    private let rayPivot = SCNNode()
    private var lastGeometryKey = ""

    override init() {
        super.init()
        let lineMat = SCNMaterial()
        lineMat.diffuse.contents = UIColor.cyan
        lineMat.emission.contents = UIColor.cyan
        lineMat.lightingModel = .constant
        axisLine.geometry = SCNCylinder(radius: 0.002, height: 1)
        axisLine.geometry?.materials = [lineMat]

        let cylMat = SCNMaterial()
        cylMat.diffuse.contents = UIColor.white.withAlphaComponent(0.18)
        cylMat.lightingModel = .constant
        cylMat.isDoubleSided = true
        cylMat.writesToDepthBuffer = false
        cylinder.geometry = SCNCylinder(radius: 0.1, height: 0.1)
        cylinder.geometry?.materials = [cylMat]

        let ringMat = SCNMaterial()
        ringMat.diffuse.contents = UIColor.cyan.withAlphaComponent(0.8)
        ringMat.emission.contents = UIColor.cyan
        ringMat.lightingModel = .constant
        baseRing.geometry = SCNTorus(ringRadius: 0.1, pipeRadius: 0.002)
        baseRing.geometry?.materials = [ringMat]

        let rayMat = SCNMaterial()
        rayMat.diffuse.contents = UIColor.orange.withAlphaComponent(0.85)
        rayMat.emission.contents = UIColor.orange
        rayMat.lightingModel = .constant
        rayMat.isDoubleSided = true
        rayMat.writesToDepthBuffer = false
        // A thin half-plane: length along +x, extent along y; pivot at the axis.
        ray.geometry = SCNBox(width: 1, height: 1, length: 0.0015, chamferRadius: 0)
        ray.geometry?.materials = [rayMat]
        ray.position = SCNVector3(0.5, 0, 0)  // box centred → shift so it starts at the axis
        rayPivot.addChildNode(ray)

        addChildNode(axisLine)
        addChildNode(cylinder)
        addChildNode(rayPivot)
        renderingOrder = 10
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

    /// Called on the SceneKit render thread.
    func update(with out: EngineOutput) {
        guard let axis = out.axis, out.state != .idle else {
            isHidden = true
            return
        }
        isHidden = false
        // Local frame: x = e1, y = axis direction, z = -e2 (right-handed). A rotation of +θ about +y takes x toward -z = e2,
        // which is exactly how the engine measures its azimuth φ (from e1 toward e2).
        let x = axis.e1, y = axis.direction, z = -axis.e2
        let o = axis.origin
        transform = SCNMatrix4(
            m11: Float(x.x), m12: Float(x.y), m13: Float(x.z), m14: 0,
            m21: Float(y.x), m22: Float(y.y), m23: Float(y.z), m24: 0,
            m31: Float(z.x), m32: Float(z.y), m33: Float(z.z), m34: 0,
            m41: Float(o.x), m42: Float(o.y), m43: Float(o.z), m44: 1)

        let radius = max(0.03, out.objectRadius)
        var hMin = out.heightMin, hMax = out.heightMax
        if hMax - hMin < 0.02 { hMin -= 0.01; hMax += 0.01 }
        let pad = 0.15 * (hMax - hMin) + 0.01
        hMin -= pad; hMax += pad
        let key = String(format: "%.3f/%.3f/%.3f", radius, hMin, hMax)
        if key != lastGeometryKey {
            lastGeometryKey = key
            (cylinder.geometry as? SCNCylinder)?.radius = CGFloat(radius)
            (cylinder.geometry as? SCNCylinder)?.height = CGFloat(hMax - hMin)
            cylinder.position = SCNVector3(0, Float(0.5 * (hMin + hMax)), 0)
            (baseRing.geometry as? SCNTorus)?.ringRadius = CGFloat(radius)
            baseRing.position = SCNVector3(0, Float(hMin), 0)
            let lineLength = max(0.6, 3 * (hMax - hMin))
            (axisLine.geometry as? SCNCylinder)?.height = CGFloat(lineLength)
            axisLine.position = SCNVector3(0, Float(0.5 * (hMin + hMax)), 0)
            ray.scale = SCNVector3(Float(radius * 1.35), Float(hMax - hMin), 1)
            ray.position = SCNVector3(Float(0.5 * radius * 1.35), Float(0.5 * (hMin + hMax)), 0)
        }
        // The ray turns with the object; hide it when the angle is not trustworthy.
        let visible = out.state == .locked && out.angleConfidence > 0.15
        rayPivot.isHidden = !visible
        rayPivot.eulerAngles = SCNVector3(0, Float(out.theta), 0)
        let alpha = CGFloat(0.35 + 0.6 * out.angleConfidence)
        ray.geometry?.firstMaterial?.diffuse.contents = UIColor.orange.withAlphaComponent(alpha)
        let axisAlpha = CGFloat(0.4 + 0.6 * out.axisQuality)
        axisLine.geometry?.firstMaterial?.diffuse.contents = UIColor.cyan.withAlphaComponent(axisAlpha)
    }
}
