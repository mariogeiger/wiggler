import Foundation

/// Projects world points into the pixels of one frame, projects world velocities into pixel velocities, and
/// turns pixels with a known range back into world points. ARKit camera convention: x right, y up, z backward,
/// so a point in front of the camera has a negative camera z and its range is that z negated.
struct PinholeProjection {
    let intrinsics: CameraIntrinsics
    let width: Double
    let height: Double
    /// Where the camera stands, in world coordinates.
    let eye: V3
    /// Camera axes in world coordinates: right, up, and the direction the camera looks along.
    let right: V3
    let up: V3
    let forward: V3
    private let cameraToWorld: RigidTransform
    private let worldToCamera: RigidTransform

    init(cameraToWorld: RigidTransform, intrinsics: CameraIntrinsics, width: Int, height: Int) {
        self.intrinsics = intrinsics
        self.width = Double(width)
        self.height = Double(height)
        self.cameraToWorld = cameraToWorld
        worldToCamera = cameraToWorld.inverse
        eye = cameraToWorld.translation
        right = cameraToWorld.applyDirection(V3(1, 0, 0)).normalized
        up = cameraToWorld.applyDirection(V3(0, 1, 0)).normalized
        forward = cameraToWorld.applyDirection(V3(0, 0, -1)).normalized
    }

    /// Pixel coordinates and range of a world point; nil when it is behind the camera.
    @inline(__always) func project(_ point: V3) -> (x: Double, y: Double, range: Double)? {
        let camera = worldToCamera.apply(point)
        let range = -camera.z
        guard range > 0.05 else { return nil }
        return (
            intrinsics.fx * camera.x / range + intrinsics.cx,
            intrinsics.cy - intrinsics.fy * camera.y / range,
            range
        )
    }

    /// World point seen at a pixel at a given range.
    @inline(__always) func unproject(x: Double, y: Double, range: Double) -> V3 {
        cameraToWorld.apply(
            V3((x - intrinsics.cx) / intrinsics.fx * range, -(y - intrinsics.cy) / intrinsics.fy * range, -range))
    }

    /// Numerator and denominator of the projection of a world point: x = nx/d + cx and y = ny/d + cy. Both are
    /// linear in the point, so the projection of a plane can be walked by addition and stays exact.
    @inline(__always) func homogeneous(_ point: V3) -> (nx: Double, ny: Double, d: Double) {
        let camera = worldToCamera.apply(point)
        return (intrinsics.fx * camera.x, -intrinsics.fy * camera.y, -camera.z)
    }

    /// The same quantities for a world direction, which is what one step across a plane adds.
    @inline(__always) func homogeneous(direction: V3) -> (nx: Double, ny: Double, d: Double) {
        let camera = worldToCamera.applyDirection(direction)
        return (intrinsics.fx * camera.x, -intrinsics.fy * camera.y, -camera.z)
    }

    /// Pixel velocity of a world point moving at `velocity` (pixels per unit of `velocity`).
    @inline(__always) func differential(at point: V3, along velocity: V3) -> (dx: Double, dy: Double) {
        let camera = worldToCamera.apply(point)
        let motion = worldToCamera.applyDirection(velocity)
        let range = -camera.z
        guard range > 0.05 else { return (0, 0) }
        let inverse = 1 / range
        let dRange = -motion.z
        return (
            intrinsics.fx * (motion.x * inverse - camera.x * dRange * inverse * inverse),
            -intrinsics.fy * (motion.y * inverse - camera.y * dRange * inverse * inverse)
        )
    }
}
