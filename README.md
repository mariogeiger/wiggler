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
one `.wig` file per session (480×360 image, depth, pose, engine outputs), streamed to disk without splitting.
It can be shared through AirDrop and replayed offline with `tools/replay.py`. Fusion strategies can be compared
with `tools/fusion.py`, which can also inject an occlusion or a stale library to test those two key behaviors.

## Swift lint and formatting

On Linux or macOS, run these commands from the repository root with Docker installed and running:

```bash
./tools/swift-quality format  # Rewrite Swift files with swift format.
./tools/swift-quality check   # Check formatting and SwiftLint rules without changing files.
```

The runner pins Swift 6.2.4 and SwiftLint 0.65.1 container images by digest. The first run downloads them;
no local Swift installation is needed. Each container is limited to 1 GiB of memory, no extra swap, and two
CPUs. Files retain the invoking user's ownership. Checks cover the app, package manifest, core sources,
and tests, but not build output. Run `check` after `format`: SwiftLint findings may need a manual fix.

`swift format` owns layout (four spaces, 120 columns), import ordering, and semicolon removal. Its `rules`
map is an allowlist; omitted rules are off. SwiftLint checks seven suspicious patterns, not naming or
complexity limits; mathematical symbols and short variable names are allowed.
These checks do not replace `swift test` or an iOS build in Xcode.

To enable the optional read-only Git hook, install [pre-commit](https://pre-commit.com/) and run:

```bash
pre-commit install
pre-commit run --all-files
```

## Usage

1. Mount the phone in portrait orientation with the object in view.
2. Move the object: its region and tracking points are selected automatically, without tapping the screen.
   The engine also explores the rest of the image; points that become stationary are replaced by moving
   points. When everything stops, existing tracks are preserved.
   If another region becomes active while the first stops, calibration restarts on the new region.
3. Rotate the object. In the calibrating state, the app collects 3D chords until it finds a well-conditioned
   axis, then waits for a full turn. The axis is green when fresh, consistent estimates have confirmed it and
   no recent chords contradict it; it is red while calibrating or while a contradiction stands. Its colour says
   nothing about the current frame: the orange ray carries the angle measurement — it rotates with the object
   when the angle is reliable and fades or hides (occluder, few points) otherwise. The axis is drawn above the
   harmonic map; only the ray is hidden in that mode.
   Moving the phone or losing its pose restarts calibration; the last axis stays visible in red meanwhile.
   A displaced object is recognised when the axis estimated from the last 1.5 s of chords alone persistently
   contradicts the current one (single chords carry ~1 cm of LiDAR noise, so no per-chord test can): the old
   chords are dropped and the new axis adopted. The harmonic map belongs to one axis generation — it is
   discarded when the axis is replaced, and only paused while the angle is held (occluder, tracking lost).
4. During the first turn after locking, the app learns the object's appearance every 10° (appearance gauge).
   It then continuously aligns the absolute angle with this library and detects the appearance period:
   360° for an arbitrary object, 180°, 120°… or rotational symmetry. For a body of revolution, only the relative
   angle is available, and the displayed angle drifts slowly.

Point colors: green = tracking consistent with rotation, red = inconsistent (hands, background),
yellow = no depth, white = new.

Controls sit in two columns just below the top safe area (below the Dynamic Island on supported iPhones).
The left column has harmonic-order buttons, harmonic-input buttons, then point-count buttons on three rows.
The right column has Record, with elapsed seconds while recording. Below it, two small dials show the angle
(orange, one turn, zero at noon) and speed (cyan, with a white noon mark at zero). Both displays reverse the
engine's sign; engine measurements and recordings are unchanged. Displayed positive rpm moves clockwise and
negative rpm counterclockwise; ±100 rpm is six o'clock. Speed saturates there rather than wrapping back to zero.
The two faces stay visible at fixed top-right positions, independently of control sizes and tracking state;
only their needles disappear when measurements are unavailable. No text or percentage appears beside them.
VoiceOver retains the signed numeric readings and reports unavailable measurements. Controls keep their
intrinsic sizes; outer margins give way when they need the width. The camera and AR overlays remain full-screen.

`./tools/test-dials` checks the dial scales, app Swift syntax, and checked-in Xcode source membership on Linux
with Docker and `memcap`. It does not replace an iOS build or a visual check on the phone.

The harmonic input buttons, below the harmonic-order row, select **Y** (luma / brightness, the default),
**Cr** (red chroma, red relative to luminance), **Cb** (blue chroma, blue relative to luminance),
or **Depth** (LiDAR depth in meters).
The selection is saved. Tracking always uses luma. Choose **off**, **l=1**, **l=2**, or **l=3** in the row above to
hide the map or display a harmonic. Switching input clears the map and requires a new full turn.
The overlay shows the signed reconstruction `a cos(lθ) + b sin(lθ)`: red is positive, blue is negative,
and opacity is its absolute value, reaching full opacity at 20/255 for color or 2 cm for depth.
It is not a hue/phase map.

Chroma comes from the camera's Cb,Cr plane, centered at byte 128 and normalized by 255 (full range) or
224 (video range), then scaled to the tracking grid. Depth keeps its native pixel grid and the same
normalized camera field of view. Non-finite depths, depths at or below 5 cm, and low-confidence samples
are missing observations, not zeros; absent confidence accepts otherwise valid depths. Each pixel needs
enough valid angular coverage and a nonsingular fit, or stays transparent. Entire missing frames decay
the existing fit without adding samples; a changed grid clears it. Depth is disabled on devices without scene-depth support. Recordings include the selected chroma plane
when it was converted for a processed frame.

## Recordings

Record writes one streamed `.wig` file per session, without restarting the running engine. Version 2 saves
**every processed frame**, not every ARKit frame: busy-frame drops and conversion failures are cumulative
per-frame counters. Each frame includes the inputs (luma, depth, confidence, converted chroma, pose,
intrinsics), the full active `EngineConfig`, harmonic signal/order and selection generations, the map's turn
progress, camera tracking state, and the engine's outputs (including `axisStable`, `axisGeneration`,
`angleMeasured`, tracked points with status). Recording stops by itself at 100 MiB and opens the share sheet.

The app records no decision journal: serialising it cost more than the engine itself and throttled the engine
to ~10 fps, so what was recorded was not what runs. The inputs are sufficient — the engine is deterministic —
and `wigreplay` recomputes the journal offline:

```bash
cd WigglerCore && swift run -c release wigreplay recording.wig > decisions.jsonl   # --diagnostics for everything
```

One JSON line per frame: outputs, events (resets, axis adoption), the full-window axis estimate, the
recent-window test, geometry health. The engine starts cold, so the first calibration differs from the
app's warm run; afterwards the replay is exact. This is **not a restorable engine snapshot**: earlier images,
the KLT pyramid, and appearance/harmonic libraries are absent. `tools/replay.py` is a research pipeline,
not an exact Swift replay.

The writer documents its own cost in every frame: `recorderMillis` (time spent writing the previous frame,
on its own queue) and `pendingFrames` (frames still queued when this one was appended — nonzero means the
writer was slower than the engine at that moment; at eight the engine waits).

All chunk lengths are UInt32 little-endian. A JSON header identifies format, app version/build, OS and an
executable SHA-256 when available; a source revision is explicitly marked as not embedded. Each frame has
six chunks: JSON metadata, luma UInt8, depth Float32, confidence UInt8, Cr Float32, Cb Float32. Metadata and
images use low-latency deflate while streaming; Stop opens the share sheet on the file as written, with no
further pass. Every nonempty frame chunk starts with a codec byte: raw (0) or raw deflate (1), followed by
the payload.
Raw storage is used only when compression fails or would not reduce size. Absent image planes have empty chunks. Float
planes are little-endian and row-major; signed chroma preserves the converted input exactly. Dimensions
are per frame. JSON nonfinite numbers use `NaN`, `+Infinity`, `-Infinity` strings; float planes retain IEEE 754.
The writer holds at most eight pending frames and waits rather than dropping processed inputs; the recorded
timestamps, drop counters and `pendingFrames` expose any slowdown.
Write failures are shown, not silently replaced with empty metadata.

`tools/wigreader.py` reads both v1 and v2 and rejects malformed or truncated chunks. For long recordings,
use `iter_frames(path)` instead of `read(path)`, which loads all frames. Frame metadata includes `config`,
`settings` and `diagnostics`; optional signed planes are `frame.chroma_red` and `frame.chroma_blue`.

```bash
PYTHONDONTWRITEBYTECODE=1 uv run --no-project --with numpy python -m unittest discover -s tools/tests
./tools/test-recording      # Linux + Docker + memcap: writer, Python readback, app syntax
```

The Swift suite runs synthetic scenes through the whole engine; the test target is built with `-O` and the scenes
use a 40-track budget. `cd WigglerCore && swift test --parallel` finishes in a few seconds on an Apple-silicon Mac
(about 19 s sequentially).

## How it works

The algorithms live in `WigglerCore` (pure Swift, without Apple platform-framework dependencies, so they can
be tested on a Mac). The app (`Wiggler/`) converts ARKit frames, calls the engine, and renders the output.

### 1. Point tracking (image)

The luma image is downscaled to 480×360. `MotionLocator` searches small regions across the image for
Shi–Tomasi corners. Pyramidal Lucas–Kanade (4 levels, 9×9 window) measures their displacement: at 100 rpm and
60 fps, an edge point moves about fifteen pixels, which the pyramid can handle. Each point is tracked only
once per frame; the same correspondences drive region selection and rotation measurement. `PointTracks`
retains the depth, chord, and extent history of selected identities.

The initial region is a stable cluster of moving corners. Camera movement and invalid ARKit poses suspend
selection. New points are activated by measured speed, not just contrast, even without depth and before the
axis is estimated. Existing moving tracks keep their history; stationary tracks give way when there is
reliable motion nearby. Stopping the object therefore does not empty the tracker. Exploratory points expire
and are redistributed so a highly textured background cannot exhaust the search budget. The default budget
is 160 measurement tracks plus 80 exploratory points. When the measurement tracks fall below half the target
(an acceleration burst kills them faster than the rolling search replaces them), fresh corners are detected in
the region every frame, beside the surviving tracks. The search continues after acquisition, including
during calibration.

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
| `minHealthyInliers` | 12 | minimum point count for healthy geometry |
| `staleLibrarySeconds` | 2.0 | disagreement duration before rebuilding the appearance library |
| `keyframeBins` | 36 | patches per turn (10°) |
| `reacquisitionDelaySeconds` | 5.0 | delay before searching again after losing the angle |

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
* **Axis motion:** fresh chords are checked before the long observation window can hide a moved object.
  A majority above 1 cm of circular-trajectory error marks the axis uncertain; three consecutive suspect
  batches restart calibration. Fresh, consistent estimates are needed before the axis turns green again.
* **Hand-held object without a fixed axis:** never locked, with planarity about 0.4, as intended.
* **Recording `wiggler-20260914-154441`:** locked after one turn (10 s), appearance library complete at 14 s,
  confidence 0.8–0.99, dispersion 1–3°. Hand occlusions and direction reversals were handled successfully.
* ARKit delivers about 20 fps after a few minutes because of thermal limits; the engine is not the bottleneck.

Tools: `tools/wigreader.py` reads `.wig` files; `tools/replay.py` replays the full pipeline with OpenCV and produces
PNG reports; `tools/diag_tracks.py` fits circles to individual tracks.
