"""Format compatibility and strict readback tests; no Apple SDK required."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import zlib

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wigreader


CHUNKS = ["metadata", "luma", "depth", "confidence", "chromaRed", "chromaBlue"]


def chunk(data):
    return struct.pack("<I", len(data)) + data


def encoded(data, version, codec):
    if not data:
        return data
    payload = data if codec == "raw" else zlib.compress(data, wbits=-15 if codec == "deflate" else 15)
    return (bytes([0 if codec == "raw" else 1]) if version == 2 else b"") + payload


def fixture(version, codec="raw"):
    header = {"version": version, "width": 3, "height": 2, "chunks": CHUNKS}
    result = chunk(json.dumps(header).encode())
    for index in range(3):
        meta = {
            "sequence": index, "t": index / 20, "state": "calibrating", "depthWidth": 2, "depthHeight": 1,
            "chromaRedWidth": 1, "chromaRedHeight": 2, "chromaBlueWidth": 3, "chromaBlueHeight": 1,
            "config": {"targetTrackCount": 60 + index},
            "diagnostics": {"frameIndex": 100 + index, "events": ["cameraMotion"]},
            "settings": {"harmonicSignal": "chromaRed" if index == 1 else "luma"},
        }
        planes = [bytes([index] * 6), np.array([1, np.nan], dtype="<f4").tobytes(), bytes([1, 2])]
        if index == 0:
            planes[1:] = [b"", b""]
        if index == 2:
            planes[2] = b""
        if version == 2:
            planes += [np.array([-0.25, 0.5], dtype="<f4").tobytes() if index == 1 else b"",
                       np.array([0.25, -0.125, 0], dtype="<f4").tobytes() if index == 2 else b""]
        metadata = json.dumps(meta).encode()
        if version == 2:
            metadata = encoded(metadata, version, codec)
        result += chunk(metadata) + b"".join(chunk(encoded(p, version, codec)) for p in planes)
    return result


class ReaderTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "session.wig"

    def read_bytes(self, data):
        self.path.write_bytes(data)
        return wigreader.read(self.path)

    def test_versions_codecs_planes_settings_and_streaming(self):
        for version in (1, 2):
            for codec in ("raw", "deflate", "zlib") if version == 1 else ("raw", "deflate"):
                with self.subTest(version=version, codec=codec):
                    header, frames = self.read_bytes(fixture(version, codec))
                    self.assertEqual(header["version"], version)
                    self.assertEqual(len(frames), 3)
                    self.assertEqual([f.t for f in wigreader.iter_frames(self.path)], [f.t for f in frames])
                    self.assertIsNone(frames[0].depth)
                    self.assertIsNone(frames[0].confidence)
                    self.assertIsNone(frames[2].confidence)
                    for index, frame in enumerate(frames):
                        np.testing.assert_array_equal(frame.luma, np.full((2, 3), index, dtype="u1"))
                        self.assertEqual(frame.meta["config"]["targetTrackCount"], 60 + index)
                    self.assertEqual(frames[1].depth.shape, (1, 2))
                    self.assertTrue(np.isnan(frames[1].depth[0, 1]))
                    self.assertEqual(frames[1].depth.dtype, np.dtype("<f4"))
                    if version == 2:
                        np.testing.assert_array_equal(frames[1].chroma_red, [[-0.25], [0.5]])
                        np.testing.assert_array_equal(frames[2].chroma_blue, [[0.25, -0.125, 0]])
                    else:
                        self.assertTrue(all(f.chroma_red is None and f.chroma_blue is None for f in frames))
                    self.assertEqual(wigreader.read(self.path, max_frames=0)[1], [])
                    self.assertEqual(len(wigreader.read(self.path, max_frames=1)[1]), 1)

    def test_truncation_is_not_silent(self):
        for version in (1, 2):
            data = fixture(version)
            for cut in (1, 2, 3, 5, 9):
                with self.subTest(version=version, cut=cut):
                    with self.assertRaises(wigreader.RecordingError):
                        self.read_bytes(data[:-cut])
        for data in (b"", b"\x01", chunk(b"{bad")):
            with self.assertRaises(wigreader.RecordingError):
                self.read_bytes(data)

    def test_invalid_sequence_shape_version_and_codec(self):
        for old, new in ((b'"sequence": 1', b'"sequence": 9'), (b'"width": 3', b'"width": 4'),
                         (b'"version": 2', b'"version": 9')):
            with self.subTest(old=old):
                with self.assertRaises(wigreader.RecordingError):
                    self.read_bytes(fixture(2).replace(old, new, 1))
        with self.assertRaises(wigreader.RecordingError):
            wigreader._inflate(b"\x02bad", 2)
        with self.assertRaises(wigreader.RecordingError):
            wigreader._inflate(b"\x01bad", 2)
        with self.assertRaises(wigreader.RecordingError):
            wigreader._inflate(b"\x01" + zlib.compress(b"plane", wbits=-15) + b"trailing", 2)
        with self.assertRaises(wigreader.RecordingError):
            wigreader._inflate(b"\x01" + zlib.compress(bytes(1000), wbits=-15), 2, max_bytes=6)


if __name__ == "__main__":
    unittest.main()
