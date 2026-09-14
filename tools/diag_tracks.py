"""Diagnostic: 3D trajectories of long-lived tracks, per-track circle fits."""
import sys, os, numpy as np, cv2
sys.path.insert(0, os.path.dirname(__file__))
import wigreader, replay as R

path = sys.argv[1]; t_start = float(sys.argv[2]) if len(sys.argv) > 2 else 0; t_end = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
h, frames = wigreader.read(path)
t0 = frames[0].t
frames = [f for f in frames if t_start <= f.t - t0 <= t_end]
W, H = h["width"], h["height"]
tracks, nid, prev = {}, 1, None
lk = dict(winSize=(9, 9), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 12, 0.03))
hist = {}
for fi, f in enumerate(frames):
    m = f.meta; g = f.luma; mk = m.get("marker")
    if mk is None: prev = g; continue
    mx, my = mk; roi = 0.3 * H
    K = (m["fx"], m["fy"], m["cx"], m["cy"]); Rm = np.array(m["rotation"]).reshape(3, 3); t = np.array(m["translation"])
    if prev is not None and tracks:
        ids = list(tracks); pts = np.array([tracks[i] for i in ids], np.float32).reshape(-1, 1, 2)
        nxt, st, err = cv2.calcOpticalFlowPyrLK(prev, g, pts, None, **lk)
        for k, i in enumerate(ids):
            x, y = nxt[k, 0]
            if not st[k, 0] or err[k, 0] > 30 or np.hypot(x - mx, y - my) > 1.3 * roi: del tracks[i]
            else: tracks[i] = (float(x), float(y))
    for i, (x, y) in tracks.items():
        z = R.sample_depth(f.depth, f.confidence, x / W, y / H)
        if z is not None:
            hist.setdefault(i, []).append((fi, f.t - t0, R.unproject(x, y, z, K, Rm, t), z))
    if len(tracks) < 120 and fi % 5 == 0:
        mask = np.zeros_like(g); cv2.circle(mask, (int(mx), int(my)), int(roi), 255, -1)
        for (x, y) in tracks.values(): cv2.circle(mask, (int(x), int(y)), 8, 0, -1)
        c = cv2.goodFeaturesToTrack(g, 120 - len(tracks), 0.01, 8, mask=mask, blockSize=5)
        if c is not None:
            for p in c.reshape(-1, 2): tracks[nid] = (float(p[0]), float(p[1])); nid += 1
    prev = g

long = sorted(hist.items(), key=lambda kv: -len(kv[1]))[:12]
print("tracks with depth:", len(hist), " longest:", [len(v) for _, v in long])
for i, v in long:
    P = np.array([s[2] for s in v]); z = np.array([s[3] for s in v]); tt = np.array([s[1] for s in v])
    c = P.mean(0); U, S, Vt = np.linalg.svd(P - c); n = Vt[2]
    # circle fit in plane
    e1 = Vt[0]; e2 = Vt[1]; x = (P - c) @ e1; y = (P - c) @ e2
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]; b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None); cx, cy = sol[0], sol[1]; r = np.sqrt(sol[2] + cx * cx + cy * cy)
    res = np.sqrt((x - cx) ** 2 + (y - cy) ** 2) - r
    print("track %4d n=%3d t=%.1f-%.1f  depth %.3f..%.3f (std %.3f)  plane sv=%s normal=%s  radius=%.3f  circle rms=%.4f  out-of-plane rms=%.4f" % (
        i, len(v), tt[0], tt[-1], z.min(), z.max(), z.std(), np.round(S / np.sqrt(len(v)), 4), np.round(n, 2), r, res.std(), ((P - c) @ n).std()))
