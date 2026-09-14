"""Offline replay of the Wiggler pipeline on a .wig recording (research harness).

Mirrors RotationEngine.swift step by step, but uses OpenCV for the image part (goodFeaturesToTrack + pyramidal LK)
so it runs fast in Python. The 3D / axis / angle parts are direct numpy ports of WigglerCore.

    python3 tools/replay.py recordings/xxx.wig [--out recordings/xxx-report] [--skip N]
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import wigreader  # noqa: E402

# ----------------------------------------------------------------------------- parameters (mirror EngineConfig)
TARGET_TRACKS = 160
MAX_RESIDUAL = 0.12
CHORD_MIN = 0.02
CHORD_MAX_FRAMES = 45
CHORD_STRIDE = 3
AXIS_UPDATE_INTERVAL = 10
WINDOW_FRAMES = 900
HUBER = 0.03
MIN_DEPTH_CONF = 1


def unit(v):
    return v / max(np.linalg.norm(v), 1e-12)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ----------------------------------------------------------------------------- depth sampling (DepthMap.sample)
def sample_depth(depth, conf, u, v, min_conf=MIN_DEPTH_CONF):
    dh, dw = depth.shape
    x, y = int(u * dw), int(v * dh)
    if x < 0 or y < 0 or x >= dw or y >= dh:
        return None
    ys, xs = slice(max(0, y - 1), min(dh, y + 2)), slice(max(0, x - 1), min(dw, x + 2))
    d = depth[ys, xs].ravel()
    c = conf[ys, xs].ravel() if conf is not None else np.full_like(d, 2, dtype=np.uint8)
    ok = (c >= min_conf) & np.isfinite(d) & (d > 0.05)
    vals = np.sort(d[ok])
    if vals.size < 4:
        return None
    med = vals[vals.size // 2]
    if vals[-1] - vals[0] > 0.15 * med:
        return None
    return float(med)


def unproject(x, y, z, K, R, t):
    fx, fy, cx, cy = K
    cam = np.array([(x - cx) / fx * z, -(y - cy) / fy * z, -z])
    return R @ cam + t


# ----------------------------------------------------------------------------- axis estimator (AxisEstimator.swift)
def estimate_axis(mids, chords, iters=8, huber=HUBER):
    n = len(mids)
    if n < 30:
        return None
    L = np.linalg.norm(chords, axis=1)
    u = chords / L[:, None]
    w = np.ones(n)
    mean_mid = mids.mean(0)
    for _ in range(iters):
        W = w * L * L
        M = (u * W[:, None]).T @ u
        evals, evecs = np.linalg.eigh(M)
        d = evecs[:, 0]
        A = M + np.outer(d, d)
        b = (u * (W * np.einsum("ij,ij->i", u, mids))[:, None]).sum(0) + d * (d @ mean_mid)
        c = np.linalg.solve(A, b)
        r1 = np.abs(u @ d) * L
        r2 = np.abs(np.einsum("ij,ij->i", mids - c, u))
        r = np.sqrt(r1 ** 2 + r2 ** 2)
        w = np.where(r < huber, 1.0, huber / r)
        inl = (r < huber).mean()
    ev = evals / W.sum()
    return dict(origin=c, direction=d, evals=ev, planarity=ev[0] / max(ev[1], 1e-12), coverage=ev[1] / max(ev[2], 1e-12),
                inlier=inl, count=n,
                good=(n >= 150 and inl >= 0.45 and ev[0] / max(ev[1], 1e-12) < 0.2 and ev[1] / max(ev[2], 1e-12) > 0.25))


class Axis:
    def __init__(self, origin, direction, e1=None):
        self.origin = np.asarray(origin, float)
        self.direction = unit(np.asarray(direction, float))
        if e1 is None:
            helper = np.array([0, 0, 1.0]) if abs(self.direction[2]) < 0.9 else np.array([1.0, 0, 0])
            e1 = np.cross(self.direction, helper)
        self.e1 = unit(e1 - self.direction * (e1 @ self.direction))
        self.e2 = np.cross(self.direction, self.e1)

    def cylindrical(self, p):
        d = p - self.origin
        a, b = d @ self.e1, d @ self.e2
        return np.arctan2(b, a), np.hypot(a, b), d @ self.direction


# ----------------------------------------------------------------------------- angle tracker (AngleTracker.swift)
class AngleTracker:
    SIG, GATE = np.radians(5), np.radians(15)

    def __init__(self):
        self.offset, self.bad = {}, {}
        self.theta, self.delta = 0.0, 0.0

    def reset(self):
        self.__init__()

    def update(self, obs):  # obs: list of (id, phi, radius)
        cand = [(i, wrap(phi - self.offset[i]), r) for i, phi, r in obs if i in self.offset]
        ok, inl, disp = False, 0, 0.0
        if cand:
            ids = np.array([c[0] for c in cand]); val = np.array([c[1] for c in cand]); rad = np.array([c[2] for c in cand])
            rcap = 1.5 * np.median(rad)
            base = np.minimum(rad, rcap) ** 2
            pred = self.theta + self.delta
            for _ in range(4):
                dev = wrap(val - pred)
                wt = base / (1 + (dev / self.SIG) ** 2)
                pred += (wt * dev).sum() / wt.sum()
            dev = wrap(val - pred)
            gate = np.abs(dev) < self.GATE
            inl = int(gate.sum())
            if inl >= 4 and base[gate].sum() > 0:
                pred += (base[gate] * dev[gate]).sum() / base[gate].sum()
                dev = wrap(val - pred)
                disp = float(np.sqrt((base[gate] * dev[gate] ** 2).sum() / base[gate].sum()))
                ok = True
            if ok:
                d = wrap(pred - self.theta)
                self.theta += d
                self.delta = d
                for i, dv in zip(ids, wrap(val - self.theta)):
                    if abs(dv) < self.GATE:
                        self.bad[i] = 0
                    else:
                        self.bad[i] = self.bad.get(i, 0) + 1
                        if self.bad[i] >= 3:
                            del self.offset[i]
            else:
                self.delta *= 0.5
        for i, phi, r in obs:
            if i not in self.offset:
                self.offset[i] = wrap(phi - self.theta); self.bad[i] = 0
        return ok, inl, disp

    def rebase(self, obs):
        for i, phi, r in obs:
            self.offset[i] = wrap(phi - self.theta); self.bad[i] = 0

    def consistent(self, i):
        return self.bad.get(i, 1) == 0


# ----------------------------------------------------------------------------- replay
def replay(path, out_dir, skip=0, verbose=True):
    header, frames = wigreader.read(path)
    frames = frames[skip:]
    os.makedirs(out_dir, exist_ok=True)
    W, H = header["width"], header["height"]
    log = []
    tracks = {}  # id -> dict(x, y, samples=[(frame, p)], last_chord, age)
    next_id = 1
    prev_gray = None
    mids, chords, cframes = [], [], []
    axis, est, state = None, None, "idle"
    angle = AngleTracker()
    theta_min = theta_max = 0.0
    theta_hist = []
    lk = dict(winSize=(9, 9), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 12, 0.03))
    marker_prev = None
    snapshots = []

    for fi, f in enumerate(frames):
        m = f.meta
        gray = f.luma
        marker = m.get("marker")
        roi = float(m.get("roiRadius") or 0.25 * H)
        K = (m["fx"], m["fy"], m["cx"], m["cy"])
        R = np.array(m["rotation"]).reshape(3, 3)
        t = np.array(m["translation"])
        if marker is None:
            prev_gray = gray
            log.append(dict(t=m["t"], state="idle"))
            continue
        if marker_prev is None or np.hypot(marker[0] - marker_prev[0], marker[1] - marker_prev[1]) > 1:
            tracks.clear(); mids.clear(); chords.clear(); cframes.clear(); axis = est = None; angle.reset()
            theta_min = theta_max = 0.0; theta_hist = []; state = "calibrating"
            marker_prev = marker
        mx, my = marker

        # 1. track
        if prev_gray is not None and tracks:
            ids = list(tracks.keys())
            pts = np.array([[tracks[i]["x"], tracks[i]["y"]] for i in ids], np.float32).reshape(-1, 1, 2)
            nxt, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, pts, None, **lk)
            for k, i in enumerate(ids):
                x, y = nxt[k, 0]
                if not st[k, 0] or err[k, 0] > MAX_RESIDUAL * 255 or np.hypot(x - mx, y - my) > 1.3 * roi \
                        or x < 5 or y < 5 or x > W - 6 or y > H - 6:
                    del tracks[i]
                else:
                    tracks[i]["x"], tracks[i]["y"] = float(x), float(y); tracks[i]["age"] += 1
        else:
            tracks.clear()

        # 2. depth -> 3D
        for i, tr in tracks.items():
            tr["has"] = False
            if f.depth is None:
                continue
            z = sample_depth(f.depth, f.confidence, tr["x"] / W, tr["y"] / H)
            if z is None:
                continue
            p = unproject(tr["x"], tr["y"], z, K, R, t)
            tr["samples"].append((fi, p)); tr["samples"] = tr["samples"][-90:]; tr["has"] = True

        # 3. chords
        obj_r = 0.1
        if axis is not None:
            rs = [tr["radius"] for tr in tracks.values() if tr.get("radius", 0) > 0]
            obj_r = np.percentile(rs, 80) if rs else 0.1
        dmin = CHORD_MIN if axis is None else min(0.05, max(0.01, 0.1 * obj_r))
        for i, tr in tracks.items():
            if not tr["has"] or fi - tr["last_chord"] < CHORD_STRIDE:
                continue
            last = tr["samples"][-1]
            for s in tr["samples"]:
                if fi - s[0] > CHORD_MAX_FRAMES:
                    continue
                if s[0] >= last[0]:
                    break
                d = last[1] - s[1]
                if np.linalg.norm(d) >= dmin:
                    mids.append(0.5 * (last[1] + s[1])); chords.append(d); cframes.append(fi); tr["last_chord"] = fi
                    break
        while cframes and cframes[0] < fi - WINDOW_FRAMES:
            mids.pop(0); chords.pop(0); cframes.pop(0)

        # 4. axis
        if fi % AXIS_UPDATE_INTERVAL == 0 and len(mids) >= 30:
            e = estimate_axis(np.array(mids), np.array(chords))
            if e is not None:
                est = e
                if e["good"]:
                    new = Axis(e["origin"], e["direction"])
                    if axis is None:
                        if new.direction @ np.array([0, 1, 0]) < 0:
                            new = Axis(new.origin, -new.direction)
                        axis = new
                    else:
                        if new.direction @ axis.direction < 0:
                            new = Axis(new.origin, -new.direction)
                        alpha = 0.5 if state == "calibrating" else 0.15
                        dvec = new.origin - axis.origin
                        perp = dvec - axis.direction * (dvec @ axis.direction)
                        ang = np.degrees(np.arccos(np.clip(new.direction @ axis.direction, -1, 1)))
                        if state == "calibrating" or (ang < 8 and np.linalg.norm(perp) < max(0.02, 0.25 * obj_r)):
                            axis = Axis(axis.origin + alpha * perp, unit(axis.direction + alpha * (new.direction - axis.direction)), axis.e1)
                    obs = []
                    for i, tr in tracks.items():
                        if tr["has"]:
                            phi, r, h = axis.cylindrical(tr["samples"][-1][1])
                            if r > 0.005:
                                obs.append((i, phi, r))
                    angle.rebase(obs)

        # 5. angle
        ok, inl, disp = False, 0, 0.0
        if axis is not None:
            obs = []
            for i, tr in tracks.items():
                if tr["has"]:
                    phi, r, h = axis.cylindrical(tr["samples"][-1][1])
                    tr["radius"], tr["height"] = r, h
                    if r > 0.005:
                        obs.append((i, phi, r))
            ok, inl, disp = angle.update(obs)
            if ok:
                theta_min, theta_max = min(theta_min, angle.theta), max(theta_max, angle.theta)
            theta_hist.append(angle.theta if ok else None)
            # static track removal
            if len(theta_hist) > 60 and theta_hist[-1] is not None and theta_hist[-61] is not None \
                    and abs(theta_hist[-1] - theta_hist[-61]) > np.radians(20):
                for i in list(tracks):
                    tr = tracks[i]
                    ss = [s for s in tr["samples"] if fi - s[0] <= 60]
                    if len(ss) >= 55 and max(np.linalg.norm(s[1] - ss[-1][1]) for s in ss) < dmin:
                        del tracks[i]
        if state == "calibrating" and axis is not None and est and est["good"] and theta_max - theta_min >= 2 * np.pi:
            state = "locked"

        # 7. replenish
        if len(tracks) < TARGET_TRACKS and (fi % 5 == 0 or len(tracks) < TARGET_TRACKS // 3):
            mask = np.zeros_like(gray)
            cv2.circle(mask, (int(mx), int(my)), int(roi), 255, -1)
            for tr in tracks.values():
                cv2.circle(mask, (int(tr["x"]), int(tr["y"])), 8, 0, -1)
            corners = cv2.goodFeaturesToTrack(gray, TARGET_TRACKS - len(tracks), 0.01, 8, mask=mask, blockSize=5)
            if corners is not None:
                for c in corners.reshape(-1, 2):
                    tracks[next_id] = dict(x=float(c[0]), y=float(c[1]), samples=[], last_chord=-1000, age=0, has=False)
                    next_id += 1

        rec = dict(t=m["t"], state=state, theta=angle.theta, ok=ok, inliers=inl, dispersion=disp, tracks=len(tracks),
                   with_depth=sum(1 for tr in tracks.values() if tr["has"]), constraints=len(mids),
                   axis_planarity=est["planarity"] if est else None, axis_coverage=est["coverage"] if est else None,
                   axis_inlier=est["inlier"] if est else None, turn=np.degrees(theta_max - theta_min),
                   app_theta=m.get("theta"), app_state=m.get("state"), app_conf=m.get("angleConfidence"),
                   axis_origin=axis.origin.tolist() if axis else None, axis_dir=axis.direction.tolist() if axis else None)
        log.append(rec)
        if verbose and fi % 15 == 0:
            print("f%4d t=%6.2f %-11s theta=%7.1f ok=%d inl=%3d disp=%4.1f tracks=%3d(%3d) chords=%5d plan=%s cov=%s inl=%s turn=%.0f app=%s/%.0f" % (
                fi, m["t"] - frames[0].t, state, np.degrees(angle.theta), ok, inl, np.degrees(disp), len(tracks), rec["with_depth"],
                len(mids), "%.2f" % est["planarity"] if est else "-", "%.2f" % est["coverage"] if est else "-",
                "%.2f" % est["inlier"] if est else "-", rec["turn"], m.get("state"), np.degrees(m.get("theta", 0))))
        if fi % 30 == 0:
            vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            for i, tr in tracks.items():
                col = (0, 255, 0) if (axis is not None and tr["has"] and angle.consistent(i)) else ((0, 0, 255) if tr["has"] else (0, 255, 255))
                cv2.circle(vis, (int(tr["x"]), int(tr["y"])), 2, col, -1)
            cv2.circle(vis, (int(mx), int(my)), int(roi), (255, 200, 0), 1)
            if axis is not None:
                # project the axis segment through the marker depth
                for sgn in (-1, 1):
                    pass
            cv2.putText(vis, "%s th=%.0f inl=%d" % (state, np.degrees(angle.theta), inl), (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            snapshots.append(vis)
            cv2.imwrite(os.path.join(out_dir, "frame%04d.png" % fi), vis)
        prev_gray = gray

    with open(os.path.join(out_dir, "log.json"), "w") as fh:
        json.dump(log, fh)
    return header, frames, log


def plot(log, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    L = [r for r in log if "theta" in r]
    if not L:
        return
    t0 = L[0]["t"]
    t = np.array([r["t"] - t0 for r in L])
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(t, np.degrees([r["theta"] for r in L]), label="replay θ (unwrapped)")
    ax[0].plot(t, np.degrees([r["app_theta"] or 0 for r in L]), label="app θ", alpha=0.6)
    ax[0].set_ylabel("deg"); ax[0].legend()
    ax[1].plot(t, [r["inliers"] for r in L], label="angle inliers")
    ax[1].plot(t, [r["tracks"] for r in L], label="tracks")
    ax[1].plot(t, [r["with_depth"] for r in L], label="with depth"); ax[1].legend()
    ax[2].plot(t, [r["axis_planarity"] or 0 for r in L], label="planarity λ0/λ1 (low=good)")
    ax[2].plot(t, [r["axis_coverage"] or 0 for r in L], label="coverage λ1/λ2 (high=good)")
    ax[2].plot(t, [r["axis_inlier"] or 0 for r in L], label="axis inlier ratio"); ax[2].legend()
    ax[3].plot(t, [r["constraints"] for r in L], label="chords"); ax[3].legend(); ax[3].set_xlabel("s")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "report.png"), dpi=110)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--out", default=None)
    ap.add_argument("--skip", type=int, default=0)
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.path)[0] + "-report"
    header, frames, log = replay(a.path, out, a.skip)
    plot(log, out)
    print("report in", out)
