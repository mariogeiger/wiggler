"""Read .wig v1/v2 recordings. iter_frames streams without retaining the session."""
import itertools
import json
import struct
import zlib

import numpy as np


class RecordingError(ValueError):
    """An unsupported, truncated, or malformed recording."""


def _chunk(stream, *, eof_ok=False):
    prefix = stream.read(4)
    if not prefix and eof_ok:
        return None
    if len(prefix) != 4:
        raise RecordingError("Truncated chunk length")
    (length,) = struct.unpack("<I", prefix)
    if length > 256 * 1024 * 1024:
        raise RecordingError("Chunk exceeds 256 MiB")
    data = stream.read(length)
    if len(data) != length:
        raise RecordingError("Truncated chunk payload")
    return data


def _json(data):
    try:
        value = json.loads(data)
    except (ValueError, UnicodeDecodeError) as error:
        raise RecordingError("Invalid JSON chunk") from error
    if not isinstance(value, dict):
        raise RecordingError("Expected a JSON object")
    return value


def _header(stream):
    header = _json(_chunk(stream))
    if header.get("version") not in (1, 2):
        raise RecordingError(f"Unsupported .wig version: {header.get('version')}")
    if header["version"] == 2 and header.get("chunks") != [
        "metadata", "luma", "depth", "confidence", "chromaRed", "chromaBlue"
    ]:
        raise RecordingError("Unsupported v2 chunk layout")
    return header


def _decompress(data, window, max_bytes):
    decoder = zlib.decompressobj(window)
    result = decoder.decompress(data, max_bytes + 1)
    if len(result) > max_bytes or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise RecordingError("Oversized, truncated, or trailing deflate data")
    return result


def _inflate(data, version, max_bytes=256 * 1024 * 1024):
    if not data:
        return b""
    if version == 2:
        if data[0] == 0:
            return data[1:]
        if data[0] != 1:
            raise RecordingError(f"Unknown chunk codec: {data[0]}")
        try:
            return _decompress(data[1:], -15, max_bytes)
        except zlib.error as error:
            raise RecordingError("Invalid deflate chunk") from error
    for window in (-15, 15):
        try:
            return _decompress(data, window, max_bytes)
        except (zlib.error, RecordingError):
            pass
    return data  # v1 has an untagged raw fallback


def _byte_count(dtype, width, height, name):
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise RecordingError(f"Invalid {name} dimensions")
    count = width * height * np.dtype(dtype).itemsize
    if count > 256 * 1024 * 1024:
        raise RecordingError(f"{name} exceeds 256 MiB")
    return count


def _plane(data, dtype, width, height, name):
    dtype = np.dtype(dtype)
    if len(data) != _byte_count(dtype, width, height, name):
        raise RecordingError(f"Wrong byte count for {name} ({width}x{height})")
    # Storage axes are (image row, image column), with columns contiguous.
    return np.frombuffer(data, dtype=dtype).reshape(height, width)


class Frame:
    __slots__ = ("meta", "luma", "depth", "confidence", "chroma_red", "chroma_blue")

    def __init__(self, meta, luma, depth, confidence, chroma_red=None, chroma_blue=None):
        self.meta = meta
        self.luma = luma
        self.depth = depth
        self.confidence = confidence
        self.chroma_red = chroma_red
        self.chroma_blue = chroma_blue

    @property
    def t(self):
        return self.meta["t"]


def _frames(stream, header):
    version = header["version"]
    sequence = 0
    while (data := _chunk(stream, eof_ok=True)) is not None:
        meta = _json(_inflate(data, version) if version == 2 else data)
        if version == 2 and meta.get("sequence") != sequence:
            raise RecordingError(f"Expected frame sequence {sequence}")
        sequence += 1
        def plane(dtype, width, height, name, *, optional=False):
            payload = _chunk(stream)
            if not payload and optional:
                return None
            size = _byte_count(dtype, width, height, name)
            return _plane(_inflate(payload, version, size), dtype, width, height, name)

        width, height = meta.get("width", header["width"]), meta.get("height", header["height"])
        dw, dh = meta.get("depthWidth", 0), meta.get("depthHeight", 0)
        luma = plane("u1", width, height, "luma")
        depth = plane("<f4", dw, dh, "depth", optional=True)
        confidence = plane("u1", dw, dh, "confidence", optional=True)
        chroma = [None, None]
        if version == 2:
            for index, name in enumerate(("chromaRed", "chromaBlue")):
                chroma[index] = plane(
                    "<f4", meta.get(name + "Width", 0), meta.get(name + "Height", 0), name, optional=True,
                )
        yield Frame(meta, luma, depth, confidence, *chroma)


def iter_frames(path):
    """Yield frames with exact plane shapes. Truncation and malformed payloads raise RecordingError."""
    with open(path, "rb") as stream:
        yield from _frames(stream, _header(stream))


def read(path, max_frames=None):
    """Return (header, frames). Use iter_frames for large files. Chroma planes are signed float32.

    All diagnostics/settings are in Frame.meta. JSON nonfinite values remain strings.
    Both versions expose uint8 luma/confidence and little-endian float32 depth; absent planes are None.
    """
    if max_frames is not None and max_frames < 0:
        raise ValueError("max_frames must be nonnegative")
    with open(path, "rb") as stream:
        header = _header(stream)
        return header, list(itertools.islice(_frames(stream, header), max_frames))


if __name__ == "__main__":
    import sys

    header, _ = read(sys.argv[1], max_frames=0)
    print("header", header)
    count, first, last = 0, None, None
    states = {}
    for frame in iter_frames(sys.argv[1]):
        count += 1
        if first is None:
            first = frame.t
        last = frame.t
        state = frame.meta.get("state", "unknown")
        states[state] = states.get(state, 0) + 1
    print("frames", count, "duration", 0 if first is None else last - first, "states", states)
