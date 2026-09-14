# Wiggler

An iPhone app (15 Pro, iOS 17+) that measures the rotation of an object in real time from a stationary phone:
a pottery wheel, an office chair, or anything rotating around a fixed axis. It infers the axis, then the object's
angle (0–360°) around that axis, and displays an augmented-reality cylinder along the axis and a ray
(half-plane) that rotates with the object.

No assumptions about the object, distance, or viewpoint. There is just one assumption: between two estimates,
the object is approximately rigid and its axis approximately fixed ("semi-static"). All sensors are used
continuously: camera image, LiDAR depth, and ARKit pose.

## Build

```bash
brew install xcodegen
cd wiggler
xcodegen generate          # generates Wiggler.xcodeproj
open Wiggler.xcodeproj     # select your signing team and your iPhone as the target
```

The algorithmic core's tests run directly on a Mac, without an iPhone:

```bash
cd WigglerCore
swift test
```

They check the algebra (eigenvalues, transforms), point tracking, axis estimation, angle tracking,
appearance-period detection, and an end-to-end synthetic scene (a textured disk rotating at variable speed,
with a depth map and noise).

A hand-written `Wiggler.xcodeproj` is also provided (XcodeGen is not required). The app's Record button writes
`.wig` files (480×360 image, depth, pose, engine outputs) that can be shared through AirDrop and replayed
offline with `tools/replay.py`. Fusion strategies can be compared with `tools/fusion.py`, which can also inject
an occlusion or a stale library to test those two key behaviors.

## Usage

1. Mount the phone in portrait orientation with the object in view.
2. Tap the object on screen: a cross appears, and points are detected in a large disk around it.
   Tapping elsewhere, or dragging, moves the marker. Each move restarts the algorithm from scratch, so you
   can experiment with marker placement and debug tracking.
3. Rotate the object. In the calibrating state, the app collects 3D chords until it finds a well-conditioned
   axis, then waits for a full turn (0→360° counter). In the locked state, the axis is displayed as a cyan line,
   a translucent cylinder, and a ring at the base; the orange ray rotates with the object.
4. During the first turn after locking, the app learns the object's appearance every 10° (appearance gauge).
   It then continuously aligns the absolute angle with this library and detects the appearance period:
   360° for an arbitrary object, 180°, 120°… or rotational symmetry. For a body of revolution, only the relative
   angle is available, and the displayed angle drifts slowly.

Point colors: green = tracking consistent with rotation, red = inconsistent (hands, background),
yellow = no depth, white = new.

## How it works

The algorithms live in `WigglerCore` (pure Swift, without Apple platform-framework dependencies, so they can
be tested on a Mac). The app (`Wiggler/`) converts ARKit frames, calls the engine, and renders the output.

### 1. Point tracking (image)

The luma image is downscaled to 480×360. Shi–Tomasi corners are detected in the disk around the marker and
tracked between frames with pyramidal Lucas–Kanade (4 levels, 9×9 window). At 100 rpm and 60 fps, an edge point
moves about fifteen pixels, which the pyramid can handle. Tracks are removed when their photometric residual
rises (hand occlusion), when they leave the region, or when they prove to be static background points.
The pool of tracks is continuously replenished (~160 points).

### 2. 3D points (LiDAR + pose)

Each tracked point is unprojected using LiDAR depth (3×3 median, depth-edge rejection) and the camera
intrinsics, then transformed into world coordinates using the ARKit pose. This compensates for small phone
movements. The result is a set of metric 3D trajectories, without prior knowledge of scale or distance.

### 3. Axis: chord constraints

A rigid point rotating around an axis (c, n) traces a circle in a plane perpendicular to n. For two positions
a and b on the same track, the chord d = b − a satisfies these exact identities, regardless of arc size:

* d · n = 0 (the chord lies in the circle's plane);
* (m − c) · d = 0, where m = (a+b)/2 (the perpendicular bisector of a chord passes through the center).

The engine accumulates these chords in a sliding 15 s window: one per track every 3 frames, with a minimum
length adapted to the noise (2 cm initially, then 10% of the object radius) and a time span ≤ 0.75 s. It solves:

* n = the eigenvector of the smallest eigenvalue of Σ w d dᵀ;
* c = the linear least-squares solution of Σ w ((m − c)·d̂)², constrained to the plane perpendicular to n
  through the centroid.

IRLS with Huber weights (3 cm) rejects hands, background, and depth outliers. Quality is assessed from the
eigenvalues (planarity λ₀/λ₁, angular coverage λ₁/λ₂) and the inlier ratio. Python validation measured 0.25°
direction error and 0.7 mm position error with 6 mm noise and 20% outliers.

### 4. Angle: per-track offsets (the relative measurement)

For each track i, the azimuth φᵢ(t) around the axis is θ(t) + oᵢ. The offset oᵢ is anchored when the track is
created. θ(t) is the robust circular mean of φᵢ − oᵢ (Cauchy weighting followed by a 15° gate, with weights
proportional to min(r, r_cap)²), predicted from the previous velocity to handle 10°/frame. Tracks that remain
inconsistent for three consecutive frames are re-anchored. Since each track remembers θ from its creation,
drift comes only from track turnover, not from every frame (about 2° RMS in simulation at 100 rpm with 6 mm noise).

The axis is refined **after** the angle measurement, never before. Re-anchoring offsets first makes every
candidate agree with the current θ and silently discards one increment every `axisUpdateInterval` frames.
The measured effect was 7% frozen frames, meaning about 7% of the rotation was consistently lost.

### 4b. Independent cross-check

The same KLT correspondences also provide a 2D image-plane rotation (Procrustes + IRLS, `similarityRotation`).
It uses neither depth nor the axis, so it fails in different situations from the 3D azimuth. The ratio between
them is learned online; it depends only on the axis tilt relative to the camera, which is constant for a fixed
phone. If the two disagree for three consecutive frames, something else has taken over the tracked points.
The geometry is then no longer considered healthy, regardless of its reported inlier count.

### 5. Appearance and fusion (the absolute measurement)

A 32×32 patch of the region of interest is stored in an angular bin every 10°. The library is usable as soon as
6 bins are filled. Waiting for a full turn would leave it unavailable most of the time; measured availability
rose from 8% to 94% of the locked time. The mean of the filled bins (the static background) is subtracted before
normalization.

Fusion (`AngleFusion`) uses a classic complementary filter. The geometric increment acts as a gyroscope,
and appearance acts as a compass:

1. **No jumps.** A correction becomes a bounded rate (90°/s), keeping the displayed angle continuous.
   The rpm reading comes only from the geometric increment and is never contaminated by realignment.
2. **Honest uncertainty.** σ grows as a random walk through track turnover while geometry provides a
   measurement, and **linearly with rotation speed** when it does not: an unobserved object keeps rotating in
   the same direction. This is why a 0.3 s occlusion at 100 rpm opens the gate to 180°, while 30 s of healthy
   tracking keeps it within a few degrees.
3. **Gating and validation.** A match is used only inside a 3σ innovation gate, and only if no clearly better
   match lies outside it. The 120° lobe of a hexagonal box therefore cannot be followed after healthy tracking,
   while genuine reacquisition at 150° remains possible immediately after a loss.

When a **strong** match contradicts **healthy** geometry for more than 2 s, the library is considered stale
(the object moved or the lighting changed). It is rebuilt rather than allowed to fight the geometry.

Measurements on `wiggler-20260914-160109` (90 s, hand manipulations): 11 discontinuities of up to 177° before,
**0 after**, and the residual against independent image rotation decreased from 39 to 19 °/s.

### 6. Rendering

SceneKit (`ARSCNView`) uses a local frame (e₁, n, −e₂) placed on the world-space axis. A rotation of θ around
its local y axis corresponds exactly to the azimuth measured by the engine, so the orange ray simply uses
`eulerAngles.y = θ`. The cylinder's radius is the 80th percentile of distances to the axis; its height spans
the 5th–95th percentiles of consistent tracks.

## Useful settings (`EngineConfig`)

| Parameter | Default | Purpose |
|---|---|---|
| `targetTrackCount` | 160 | number of tracked points |
| `maxResidual` | 0.12 | photometric rejection threshold (0–1) |
| `chordMinMeters` | 0.02 | minimum chord length before an axis is known |
| `chordMaxFrames` | 45 | maximum chord time span in frames |
| `constraintWindowFrames` | 900 | sliding chord window (15 s) |
| `lockedDriftFrames` | 3 | inconsistent axis estimates before adopting a new axis |
| `minHealthyInliers` | 20 | minimum point count for healthy geometry |
| `staleLibrarySeconds` | 2.0 | disagreement duration before rebuilding the appearance library |
| `keyframeBins` | 36 | patches per turn (10°) |

## Known limits / ideas

* A large moving object can confuse ARKit when it fills the image. If the pose becomes incorrect, position
  the phone so that a stationary background is visible at the edges, or force the identity pose.
* A perfectly smooth body of revolution provides neither texture nor asymmetry: there is nothing to track.
  By design, the app makes no prediction in this case.
* LiDAR is noisy at edges (handled by median rejection) and below about 20 cm. Between 30 cm and 2 m it works well.

## Experiment log (September 14, 2026, office chair + box)

* **Performance:** the package took 60–400 ms/frame without `-O`; forcing `-O` in `Package.swift` reduced this to 2–5 ms.
* **Box at the seat edge:** failed because the smoothed 256×192 LiDAR assigned the depth of the floor behind it
  to the box's points. Per-track depth-jump rejection (> 8%) was added. A practical rule is to aim inside the
  depth silhouette.
* **Unwanted recalibrations:** the re-estimated axis moved by 3–8 cm as the chair rolled. Tolerances were widened,
  and the new axis is now adopted gradually instead of restarting calibration.
* **Hand-held object without a fixed axis:** never locked, with planarity about 0.4, as intended.
* **Recording `wiggler-20260914-154441`:** locked after one turn (10 s), appearance library complete at 14 s,
  confidence 0.8–0.99, dispersion 1–3°. Hand occlusions and direction reversals were handled successfully.
* ARKit delivers about 20 fps after a few minutes because of thermal limits; the engine is not the bottleneck.

Tools: `tools/wigreader.py` reads `.wig` files; `tools/replay.py` replays the full pipeline with OpenCV and produces
PNG reports; `tools/diag_tracks.py` fits circles to individual tracks.
