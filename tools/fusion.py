"""Offline test bench for the angle fusion: relative (geometry) + absolute (appearance) + independent check.

Runs the full pipeline on a .wig recording with pluggable fusion strategies, so a change can be measured
before it is ported to Swift.

Strategies:
  "current"  : integrate geometry, appearance pulls with gain 0.15, teleport on persistent large disagreement
  "fused"    : complementary filter — geometry is the "gyro", appearance is the "compass":
               * the state is never teleported; corrections are rate-limited (deg/s)
               * an uncertainty sigma grows while the geometry is not measuring and shrinks when it is,
                 and the appearance measurement is accepted only inside a 3-sigma innovation gate
               * an independent image-plane rotation estimate validates the geometric increment

Metrics: discontinuities of the reported angle, and agreement with the independent image rotation.
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import wigreader  # noqa: E402
import replay as R  # noqa: E402

BINS = 36
SIDE = 32


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# --------------------------------------------------------------------------- appearance library
class Relocalizer:
    def __init__(self, bins=BINS, side=SIDE):
        self.bins, self.side = bins, side
        self.raw = [None] * bins
        self.mean = None
        self.lib = None
        self.index = []
        self.filled = 0

    @property
    def bin_width(self):
        return 2 * np.pi / self.bins

    @property
    def complete(self):
        return self.filled >= self.bins

    @staticmethod
    def patch(gray, cx, cy, half, side):
        x0, x1 = int(cx - half), int(cx + half)
        y0, y1 = int(cy - half), int(cy + half)
        h, w = gray.shape
        pad = np.full((y1 - y0, x1 - x0), 127, np.uint8)
        sx0, sy0 = max(0, x0), max(0, y0)
        sx1, sy1 = min(w, x1), min(h, y1)
        if sx1 > sx0 and sy1 > sy0:
            pad[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0] = gray[sy0:sy1, sx0:sx1]
        return cv2.resize(pad, (side, side), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0

    @property
    def usable(self):
        return self.filled >= 6

    def record(self, p, theta):
        b = int(round((theta % (2 * np.pi)) / self.bin_width)) % self.bins
        if self.raw[b] is None:
            self.filled += 1
        self.raw[b] = p
        self.analyse()

    def analyse(self):
        idx = [k for k in range(self.bins) if self.raw[k] is not None]
        if len(idx) < 6:
            self.lib, self.index = None, []
            return
        self.mean = np.mean([self.raw[k] for k in idx], axis=0)
        self.index = idx
        self.lib = np.stack([self._centred(self.raw[k]) for k in idx])

    def _centred(self, p):
        d = (p - self.mean).ravel()
        n = np.linalg.norm(d)
        return d / n if n > 1e-6 else d

    def refresh(self, p, theta, alpha):
        if self.lib is None:
            return
        b = int(round((theta % (2 * np.pi)) / self.bin_width)) % self.bins
        if self.raw[b] is None:
            self.raw[b] = p
            self.filled += 1
        else:
            self.raw[b] = self.raw[b] + alpha * (p - self.raw[b])
        self.analyse()

    def match(self, p, near):
        """Returns list of (theta_absolute_unwrapped_near, score) for all local maxima, best first."""
        if self.lib is None:
            return []
        sc = self.lib @ self._centred(p)
        full = np.full(self.bins, -2.0)
        full[self.index] = sc
        out = []
        for k in self.index:
            if full[k] >= full[(k + 1) % self.bins] and full[k] >= full[k - 1] and full[k] > 0.5:
                sm, s0, sp = full[k - 1], full[k], full[(k + 1) % self.bins]
                den = sm - 2 * s0 + sp
                frac = np.clip(0.5 * (sm - sp) / den, -0.5, 0.5) if (den < -1e-9 and sm > -1 and sp > -1) else 0.0
                a = (k + frac) * self.bin_width
                out.append((near + wrap(a - near), float(s0)))
        out.sort(key=lambda x: -x[1])
        return out


# --------------------------------------------------------------------------- independent image rotation
class ImageRotation:
    """2D similarity between consecutive frames (RANSAC on LK tracks). Uses no depth at all, so it is a genuinely
    independent estimate of the rotation rate as seen in the image plane."""

    def __init__(self, target=120, roi=110):
        self.prev = None
        self.pts = None
        self.target = target
        self.roi = roi
        self.lk = dict(winSize=(11, 11), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01))

    def update(self, gray, cx, cy):
        d = None
        if self.prev is not None and self.pts is not None and len(self.pts) >= 20:
            nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev, gray, self.pts, None, **self.lk)
            back, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev, nxt, None, **self.lk)
            ok = (st[:, 0] == 1) & (st2[:, 0] == 1) & (np.linalg.norm((back - self.pts).reshape(-1, 2), axis=1) < 0.5)
            if ok.sum() >= 12:
                M, inl = cv2.estimateAffinePartial2D(self.pts[ok], nxt[ok], method=cv2.RANSAC, ransacReprojThreshold=1.5)
                if M is not None and inl is not None and inl.sum() >= 10:
                    d = np.arctan2(M[1, 0], M[0, 0])
            self.pts = nxt[ok] if ok.sum() >= 12 else None
        if self.pts is None or len(self.pts) < self.target * 0.6:
            mask = np.zeros_like(gray)
            cv2.circle(mask, (int(cx), int(cy)), self.roi, 255, -1)
            c = cv2.goodFeaturesToTrack(gray, self.target, 0.01, 6, mask=mask, blockSize=5)
            self.pts = c.astype(np.float32) if c is not None else None
        self.prev = gray
        return d


# --------------------------------------------------------------------------- fusion strategies
class CurrentFusion:
    """What the app does today."""
    name = "current"

    def __init__(self):
        self.jump = None

    def update(self, state, cands, healthy, dt):
        th = state["theta"]
        if not cands:
            self.jump = None
            return th, "none"
        m, score = cands[0]
        if score < 0.6:
            self.jump = None
            return th, "weak"
        err = wrap(m - th)
        if abs(err) < np.radians(20):
            self.jump = None
            return th + 0.15 * err, "pull"
        if state["healthy_streak"] < 120:
            if self.jump is not None and abs(wrap(self.jump[0] - m)) < np.radians(10):
                self.jump = (m, self.jump[1] + 1)
                if self.jump[1] >= 6:
                    self.jump = None
                    return th + err, "TELEPORT"
            else:
                self.jump = (m, 1)
        return th, "held"


class FusedFilter:
    """Complementary filter. The geometric increment is the "gyro", the appearance match is the "compass".

    Three ideas, all standard:

    1. The state is never teleported. A correction is a bounded *rate*, so the displayed angle stays continuous
       and the rpm read-out (which comes from the geometric increment) is untouched.

    2. sigma is the honest uncertainty of the integrated angle, and it grows at the rate the angle can actually
       run away: while the geometry measures, only through track turnover (a small fraction of |omega|); while it
       does not, at |omega| itself, because the object keeps turning unseen. That is why a 0.3 s occlusion at
       100 rpm opens the gate to 180 deg while 30 s of healthy tracking keeps it at a few degrees.

    3. The appearance measurement is accepted only inside a 3-sigma innovation gate, which makes the periodic
       lobe of a hexagonal box (120 deg away) unacceptable after a healthy run, and a real 150 deg recovery
       acceptable right after a loss. Among the candidates inside the gate the best score wins, which resolves the
       periodic ambiguity the way a compass with a known approximate heading does.
    """
    name = "fused"

    def __init__(self, max_rate_deg_s=90.0, sigma0=np.radians(3)):
        self.max_rate = np.radians(max_rate_deg_s)
        self.sigma = sigma0
        self.pending = 0.0
        self.turnover_drift = 0.02      # fraction of |omega| leaking into the integrated angle per unit time
        self.floor = np.radians(0.3)    # deg/s of drift even at rest
        self.stale = 0.0                # seconds a healthy geometry has been contradicted by a strong match
        self.trusted = False            # the last match was sharp and consistent: safe to refresh the library

    def update(self, state, cands, healthy, dt):
        omega = abs(state["omega"])                       # rad/s, from the geometric increment
        if healthy:
            # zero-mean turnover noise: a random walk
            self.sigma = np.hypot(self.sigma, (self.turnover_drift * omega + self.floor) * dt)
        else:
            # nothing is measuring the rotation: the object keeps turning one way, so the error grows linearly
            self.sigma += (1.2 * omega + np.radians(30)) * dt
        self.sigma = min(self.sigma, np.pi)

        th = state["theta"]
        tag = "none"
        self.trusted = False
        if cands:
            gate = min(3 * self.sigma, np.pi)
            ref = th + self.pending
            inside = [(m, sc) for m, sc in cands if abs(wrap(m - ref)) < gate and sc > 0.55]
            best_all = max(cands, key=lambda x: x[1])
            best_in = max(inside, key=lambda x: x[1]) if inside else None
            # Validation: if the strongest match sits outside the gate and is clearly better than anything inside,
            # the measurement contradicts the state. Never follow it (it is the periodic lobe of the object, or a
            # library that no longer describes the scene) and never let it creep in through a marginal candidate.
            if best_in is None or best_all[1] > best_in[1] + 0.05:
                # Only a *strong* match that lands far from the state means the library is wrong about the scene;
                # a weak one just means the object is not clearly visible right now.
                if state["healthy"] and best_all[1] > 0.7 and abs(wrap(best_all[0] - ref)) > gate:
                    self.stale += dt
                tag = "gated" if best_in is None else "ambiguous"
            else:
                m, sc = best_in
                innov = wrap(m - ref)
                sigma_m = np.radians(5) / max(sc, 0.3)     # a sharp match is worth about one bin
                k = self.sigma ** 2 / (self.sigma ** 2 + sigma_m ** 2)
                self.pending += k * innov
                self.sigma = np.sqrt((1 - k) * self.sigma ** 2)
                self.stale = max(0.0, self.stale - 2 * dt)
                self.trusted = sc > 0.6 and abs(innov) < np.radians(10)
                tag = "update"
        step = float(np.clip(self.pending, -self.max_rate * dt, self.max_rate * dt))
        self.pending -= step
        return th + step, tag


def similarity_rotation(p0, p1):
    """Robust 2D rotation of the point cloud between two frames (Procrustes + IRLS). Independent of depth and of
    the axis: a cross-check on the geometric increment."""
    if len(p0) < 8:
        return None, 0
    w = np.ones(len(p0))
    ang = 0.0
    for _ in range(4):
        c0 = (p0 * w[:, None]).sum(0) / w.sum()
        c1 = (p1 * w[:, None]).sum(0) / w.sum()
        a = p0 - c0
        b = p1 - c1
        num = (w * (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])).sum()
        den = (w * (a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])).sum()
        ang = np.arctan2(num, den)
        ca, sa = np.cos(ang), np.sin(ang)
        scale = np.hypot(num, den) / max((w * (a ** 2).sum(1)).sum(), 1e-9)
        pred = np.c_[scale * (ca * a[:, 0] - sa * a[:, 1]), scale * (sa * a[:, 0] + ca * a[:, 1])]
        res = np.linalg.norm(pred - b, axis=1)
        s = 1.5 * np.median(res) + 1e-6
        w = 1.0 / (1.0 + (res / s) ** 2)
    return ang, int((res < 2 * s).sum())


# --------------------------------------------------------------------------- main loop
def run(path, strategy, inject=None, verbose=False):
    """inject: ("bad_library", t) rotates the library by 120 deg at time t (object displaced / periodic lobe wins)
               ("lost", t) blinds the geometry for 1 s and re-anchors it 150 deg off (occlusion recovery)"""
    header, frames = wigreader.read(path)
    W, H = header["width"], header["height"]
    tracks, nid, prev_gray = {}, 1, None
    mids, chords, cframes = [], [], []
    axis, est, state_name = None, None, "idle"
    angle = R.AngleTracker()
    reloc = Relocalizer()
    fusion = {"current": CurrentFusion, "fused": FusedFilter}[strategy]()
    imgrot = ImageRotation()
    lk = dict(winSize=(9, 9), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 12, 0.03))
    theta_min = theta_max = 0.0
    healthy_streak = 0
    scale_est = None
    omega_known = 0.0
    rebuilds = 0
    marker_prev = None
    log = []
    img_theta = 0.0

    for fi, f in enumerate(frames):
        m = f.meta
        gray = f.luma
        marker = m.get("marker")
        if marker is None:
            prev_gray = gray
            continue
        roi = float(m.get("roiRadius") or 0.35 * H)
        K = (m["fx"], m["fy"], m["cx"], m["cy"])
        Rm = np.array(m["rotation"]).reshape(3, 3)
        tv = np.array(m["translation"])
        mx, my = marker
        if marker_prev is None or np.hypot(mx - marker_prev[0], my - marker_prev[1]) > 1:
            tracks.clear(); mids.clear(); chords.clear(); cframes.clear()
            axis = est = None; angle.reset(); theta_min = theta_max = 0.0; state_name = "calibrating"
            marker_prev = marker

        now = f.t - frames[0].t
        blind = inject is not None and inject[0] == "lost" and inject[1] <= now < inject[1] + 1.0
        if inject is not None and inject[0] == "bad_library" and abs(now - inject[1]) < 0.06 and reloc.lib is not None:
            k = int(round(np.radians(120) / reloc.bin_width))
            reloc.raw = reloc.raw[k:] + reloc.raw[:k]
            reloc.lib = np.roll(reloc.lib, k, axis=0)

        d_img = imgrot.update(gray, mx, my)
        if d_img is not None:
            img_theta += d_img

        # --- track (keep the correspondences: they also give the image-plane rotation)
        pair0, pair1 = [], []
        if prev_gray is not None and tracks:
            ids = list(tracks)
            pts = np.array([[tracks[i]["x"], tracks[i]["y"]] for i in ids], np.float32).reshape(-1, 1, 2)
            nxt, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, pts, None, **lk)
            for k, i in enumerate(ids):
                x, y = nxt[k, 0]
                if not st[k, 0] or err[k, 0] > R.MAX_RESIDUAL * 255 or np.hypot(x - mx, y - my) > 1.3 * roi \
                        or x < 5 or y < 5 or x > W - 6 or y > H - 6:
                    del tracks[i]
                else:
                    pair0.append((tracks[i]["x"], tracks[i]["y"]))
                    pair1.append((float(x), float(y)))
                    tracks[i]["x"], tracks[i]["y"] = float(x), float(y)
                    tracks[i]["age"] += 1
        else:
            tracks.clear()

        # --- depth
        for i, tr in tracks.items():
            tr["has"] = False
            if f.depth is None:
                continue
            z = R.sample_depth(f.depth, f.confidence, tr["x"] / W, tr["y"] / H)
            if z is None:
                continue
            if tr["lastz"] > 0 and abs(z - tr["lastz"]) > 0.08 * tr["lastz"]:
                tr["rej"] += 1
                if tr["rej"] < 4:
                    continue
            tr["rej"] = 0
            tr["lastz"] = z
            p = R.unproject(tr["x"], tr["y"], z, K, Rm, tv)
            tr["samples"].append((fi, p))
            tr["samples"] = tr["samples"][-90:]
            tr["has"] = True

        # --- chords / axis (same as replay.py)
        obj_r = 0.1
        if axis is not None:
            rs = [t["radius"] for t in tracks.values() if t.get("radius", 0) > 0]
            obj_r = np.percentile(rs, 80) if rs else 0.1
        dmin = R.CHORD_MIN if axis is None else min(0.05, max(0.01, 0.1 * obj_r))
        for i, tr in tracks.items():
            if not tr["has"] or fi - tr["last_chord"] < R.CHORD_STRIDE:
                continue
            last = tr["samples"][-1]
            for s in tr["samples"]:
                if fi - s[0] > R.CHORD_MAX_FRAMES or s[0] >= last[0]:
                    continue
                dd = last[1] - s[1]
                if np.linalg.norm(dd) >= dmin:
                    mids.append(0.5 * (last[1] + s[1])); chords.append(dd); cframes.append(fi)
                    tr["last_chord"] = fi
                    break
        while cframes and cframes[0] < fi - R.WINDOW_FRAMES:
            mids.pop(0); chords.pop(0); cframes.pop(0)

        # (axis refinement happens after the angle update: re-anchoring first would destroy one increment)
        # --- geometric angle (the "gyro")
        ok, inl, disp = False, 0, 0.0
        if axis is not None:
            obs = []
            for i, tr in tracks.items():
                if tr["has"]:
                    phi, r, h = axis.cylindrical(tr["samples"][-1][1])
                    tr["radius"], tr["height"] = r, h
                    if r > 0.005:
                        obs.append((i, phi, r))
            if blind:
                obs = []
            ok, inl, disp = angle.update(obs)
            if inject is not None and inject[0] == "lost" and abs(now - (inject[1] + 1.0)) < 0.06:
                angle.shift(np.radians(150))   # wrong re-anchor after the occlusion
            if ok:
                theta_min = min(theta_min, angle.theta); theta_max = max(theta_max, angle.theta)
        if state_name == "calibrating" and axis is not None and est and est["good"] and theta_max - theta_min >= 2 * np.pi:
            state_name = "locked"

        if fi % R.AXIS_UPDATE_INTERVAL == 0 and len(mids) >= 30:
            e = R.estimate_axis(np.array(mids), np.array(chords))
            if e is not None:
                est = e
                if e["good"]:
                    new = R.Axis(e["origin"], e["direction"])
                    if axis is None:
                        if new.direction @ np.array([0, 1.0, 0]) < 0:
                            new = R.Axis(new.origin, -new.direction)
                        axis = new
                    else:
                        if new.direction @ axis.direction < 0:
                            new = R.Axis(new.origin, -new.direction)
                        alpha = 0.5 if state_name == "calibrating" else 0.15
                        dv = new.origin - axis.origin
                        perp = dv - axis.direction * (dv @ axis.direction)
                        axis = R.Axis(axis.origin + alpha * perp,
                                      R.unit(axis.direction + alpha * (new.direction - axis.direction)), axis.e1)
                    obs = []
                    for i, tr in tracks.items():
                        if tr["has"]:
                            phi, r, _ = axis.cylindrical(tr["samples"][-1][1])
                            if r > 0.005:
                                obs.append((i, phi, r))
                    angle.rebase(obs)


        # --- independent cross-check: image-plane rotation of the same correspondences (no depth, no axis)
        dt = 1.0 / 20 if fi == 0 else max(1e-3, frames[fi].t - frames[fi - 1].t)
        d_sim, n_sim = similarity_rotation(np.array(pair0), np.array(pair1)) if len(pair0) >= 8 else (None, 0)
        if d_sim is not None and ok and abs(angle.delta) > np.radians(1):
            ratio = d_sim / angle.delta
            scale_est = 0.9 * scale_est + 0.1 * ratio if scale_est is not None else ratio
        agree = True
        if d_sim is not None and scale_est is not None and n_sim >= 8:
            pred = scale_est * angle.delta
            agree = abs(d_sim - pred) < np.radians(3) + 0.35 * abs(pred)

        # --- appearance (the "compass") + fusion
        tag = ""
        theta_before = angle.theta
        # While the geometry is blind, `delta` decays to zero, but the object does not stop: hold the last
        # measured rate so the uncertainty keeps growing at the rate the angle can actually run away.
        if ok:
            omega_known = angle.delta / dt
        omega = omega_known
        healthy = ok and inl >= 20 and agree
        healthy_streak = healthy_streak + 1 if healthy else 0
        if state_name == "locked":
            p = Relocalizer.patch(gray, mx, my, roi, SIDE)
            if ok and disp < np.radians(6) and inl >= 8 and not reloc.complete:
                reloc.record(p, angle.theta)
            if not reloc.usable:
                pass
            else:
                # A hand over the object hides the appearance as well as the geometry: no candidates, but the
                # uncertainty must still be propagated (that is the whole point of carrying sigma).
                cands = [] if blind else reloc.match(p, angle.theta)
                best = cands[0][1] if cands else 0.0
                # The geometry is trusted only when several independent signals agree that the object is still
                # the thing being measured: the robust mean converged on enough points, the region still looks
                # like the object (appearance score), and the image-plane rotation matches the 3D increment.
                healthy = healthy and best > 0.45
                healthy_streak = healthy_streak if healthy else 0
                new_theta, tag = fusion.update(
                    {"theta": angle.theta, "healthy_streak": healthy_streak, "omega": omega, "healthy": healthy}, cands, healthy, dt)
                shift = new_theta - angle.theta
                if shift != 0:
                    angle.shift(shift)
                if healthy and disp < np.radians(6) and getattr(fusion, "trusted", True):
                    reloc.refresh(p, angle.theta, 0.05)
                # The library keeps contradicting a healthy geometry: the object's appearance has changed for good
                # (moved on its support, relit). Rebuild it rather than fighting it.
                if getattr(fusion, "stale", 0.0) > 2.0:
                    reloc.__init__()
                    fusion.stale = 0.0
                    rebuilds += 1

        # --- replenish
        if len(tracks) < R.TARGET_TRACKS and (fi % 5 == 0 or len(tracks) < R.TARGET_TRACKS // 3):
            mask = np.zeros_like(gray)
            cv2.circle(mask, (int(mx), int(my)), int(roi), 255, -1)
            for tr in tracks.values():
                cv2.circle(mask, (int(tr["x"]), int(tr["y"])), 8, 0, -1)
            c = cv2.goodFeaturesToTrack(gray, R.TARGET_TRACKS - len(tracks), 0.01, 8, mask=mask, blockSize=5)
            if c is not None:
                for q in c.reshape(-1, 2):
                    tracks[nid] = dict(x=float(q[0]), y=float(q[1]), samples=[], last_chord=-1000, age=0,
                                       has=False, lastz=0.0, rej=0)
                    nid += 1

        log.append(dict(t=f.t - frames[0].t, state=state_name, theta=angle.theta, geo=theta_before,
                        delta=angle.delta, inl=inl, disp=disp, tag=tag, img=img_theta,
                        sigma=getattr(fusion, "sigma", 0.0), app=m["theta"], app_state=m["state"],
                        agree=agree, healthy=healthy, sim=d_sim if d_sim is not None else np.nan,
                        rebuilds=rebuilds, libok=reloc.usable))
        prev_gray = gray
    return log


def report(log, label):
    L = [r for r in log if r["state"] == "locked"]
    if len(L) < 10:
        print(f"{label}: not locked"); return
    th = np.degrees([r["theta"] for r in L])
    t = np.array([r["t"] for r in L])
    d = np.abs(np.diff(th))
    # expected motion per frame from the geometric increment
    exp = np.abs(np.degrees([r["delta"] for r in L[1:]]))
    excess = d - exp
    jumps = np.where(excess > 10)[0]
    img = np.degrees([r["img"] for r in L])
    # relative consistency with the independent image rotation over 1 s windows
    n = max(1, int(1.0 / np.median(np.diff(t))))
    dg = th[n:] - th[:-n]
    di = img[n:] - img[:-n]
    scale = np.polyfit(di, dg, 1)[0] if np.std(di) > 1 else np.nan
    resid = dg - scale * di
    print("%-8s locked %5.1f s | discontinuities %2d (max %.0f deg) | vs image rot: scale %.2f, rms %.1f deg/s"
          " | library rebuilds %d"
          % (label, t[-1] - t[0], len(jumps), excess[jumps].max() if len(jumps) else 0, scale, np.std(resid),
             L[-1].get("rebuilds", 0)))
    if len(jumps):
        print("         at t =", [round(t[i + 1], 1) for i in jumps[:12]])


def injection_report(log, label, kind, t0):
    L = [r for r in log if r["state"] == "locked"]
    t = np.array([r["t"] for r in L]); th = np.degrees([r["theta"] for r in L]); geo = np.degrees([r["geo"] for r in L])
    if len(L) < 10 or t[-1] < t0 + 3:
        print("%-8s %s: not enough locked data after t0" % (label, kind)); return
    # correction applied by the fusion, relative to the pure geometric integration, since t0
    i0 = int(np.argmin(np.abs(t - t0)))
    corr = np.cumsum(th - geo)               # geo is theta before the fusion step of that frame
    applied = corr - corr[i0]
    d = np.abs(np.diff(th)) - np.abs(np.degrees([r["delta"] for r in L[1:]]))
    jumps = int((d > 10).sum())
    end = applied[min(len(applied) - 1, int(np.argmin(np.abs(t - (t0 + 6)))))]
    print("%-8s %-12s correction applied after 6 s: %+7.1f deg | discontinuities %d" % (label, kind, end, jumps))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--strategies", default="current,fused")
    ap.add_argument("--inject-at", type=float, default=None)
    a = ap.parse_args()
    for p in a.paths:
        print("===", os.path.basename(p))
        for s in a.strategies.split(","):
            report(run(p, s), s)
            if a.inject_at is not None:
                injection_report(run(p, s, ("bad_library", a.inject_at)), s, "bad library", a.inject_at)
                injection_report(run(p, s, ("lost", a.inject_at)), s, "occlusion", a.inject_at + 1.0)
