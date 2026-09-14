"""Reader for Wiggler .wig recordings (see SessionRecorder.swift for the format)."""
import json
import struct
import zlib
import numpy as np


def _chunks(path):
    with open(path, "rb") as f:
        while True:
            hdr = f.read(4)
            if len(hdr) < 4:
                return
            (n,) = struct.unpack("<I", hdr)
            data = f.read(n)
            if len(data) < n:
                return
            yield data


def _inflate(b):
    if not b:
        return b""
    try:
        return zlib.decompress(b, -15)
    except zlib.error:
        try:
            return zlib.decompress(b)
        except zlib.error:
            return b  # stored raw (compression failed on device)


class Frame:
    __slots__ = ("meta", "luma", "depth", "confidence")

    def __init__(self, meta, luma, depth, confidence):
        self.meta = meta
        self.luma = luma
        self.depth = depth
        self.confidence = confidence

    @property
    def t(self):
        return self.meta["t"]


def read(path, max_frames=None):
    """Returns (header, [Frame]). luma: uint8 HxW; depth: float32 dh x dw (or None); confidence: uint8 (or None)."""
    it = _chunks(path)
    header = json.loads(next(it))
    w, h = header["width"], header["height"]
    frames = []
    while True:
        try:
            meta = json.loads(next(it))
            luma = np.frombuffer(_inflate(next(it)), dtype=np.uint8)
            depth = np.frombuffer(_inflate(next(it)), dtype=np.float32)
            conf = np.frombuffer(_inflate(next(it)), dtype=np.uint8)
        except StopIteration:
            break
        luma = luma.reshape(h, w) if luma.size == w * h else None
        dw, dh = meta.get("depthWidth", 0), meta.get("depthHeight", 0)
        depth = depth.reshape(dh, dw) if dw and depth.size == dw * dh else None
        conf = conf.reshape(dh, dw) if dw and conf.size == dw * dh else None
        frames.append(Frame(meta, luma, depth, conf))
        if max_frames and len(frames) >= max_frames:
            break
    return header, frames


if __name__ == "__main__":
    import sys
    header, frames = read(sys.argv[1])
    print("header", header)
    print("frames", len(frames))
    if frames:
        t0 = frames[0].t
        dts = np.diff([f.t for f in frames])
        print("duration %.1f s, median dt %.1f ms, max dt %.1f ms" % (frames[-1].t - t0, np.median(dts) * 1000, dts.max() * 1000))
        m = frames[0].meta
        print("intrinsics", m["fx"], m["fy"], m["cx"], m["cy"], "depth", m["depthWidth"], m["depthHeight"])
        states = {}
        for f in frames:
            states[f.meta["state"]] = states.get(f.meta["state"], 0) + 1
        print("states", states)
        for f in frames[:: max(1, len(frames) // 25)]:
            mm = f.meta
            print("t=%6.2f %-11s theta=%7.1f conf=%.2f axisQ=%.2f rpm=%6.1f tracks=%3d inl=%3d ms=%5.1f marker=%s" % (
                f.t - t0, mm["state"], np.degrees(mm["theta"]), mm["angleConfidence"], mm["axisQuality"], mm["rpm"],
                mm["trackCount"], mm["inlierCount"], mm["processingMillis"], mm.get("marker")))
