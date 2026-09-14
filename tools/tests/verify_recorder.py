"""Check files written by the real SessionRecorder harness, not a Python format facsimile."""
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wigreader

root = Path(sys.argv[1])
manifest = json.loads((root / "manifest.json").read_text())
header, frames = wigreader.read(root / manifest["warm"])
assert header["version"] == 2 and "not a restorable snapshot" in header["initialState"]
assert header["build"]["sourceRevision"] == "not embedded"
assert len(frames) == 24
for index, frame in enumerate(frames, 5):
    meta = frame.meta
    assert meta["sequence"] == index - 5
    assert meta["diagnostics"]["frameIndex"] == index + 1
    assert meta["config"]["targetTrackCount"] == (60 if index < 17 else 65)
    assert meta["settings"]["harmonicOrder"] == (None if index < 17 else 2)
    assert meta["droppedFrames"] == index and meta["conversionFailures"] == 2
    assert meta["t"] == index / 20
    assert meta["axisStable"] is False
    np.testing.assert_array_equal(frame.luma, (np.arange(48 * 36) % 251).astype("u1").reshape(36, 48))
    if index % 3 == 0:
        assert frame.depth is None and frame.confidence is None
    else:
        np.testing.assert_array_equal(frame.depth, [[1, np.nan]])
        if index % 3 == 1:
            np.testing.assert_array_equal(frame.confidence, [[1, 2]])
        else:
            assert frame.confidence is None
    if index % 3 == 1:
        np.testing.assert_array_equal(frame.chroma_red, [[-0.25], [0.5]])
        assert frame.chroma_blue is None
    elif index % 3 == 2:
        np.testing.assert_array_equal(frame.chroma_blue, [[0.25, -0.125, 0]])
        assert frame.chroma_red is None
    else:
        assert frame.chroma_red is None and frame.chroma_blue is None
assert frames[0].meta["rpm"] == "+Infinity"
assert "initialContext" in frames[0].meta["diagnostics"]
assert "initialContext" not in frames[1].meta["diagnostics"]
count = 0
for index, frame in enumerate(wigreader.iter_frames(root / manifest["large"])):
    count += 1
    assert frame.meta["sequence"] == index and frame.t == index / 30
    assert frame.luma.shape == (360, 480) and np.all(frame.luma == index % 251)
    assert frame.depth is None and frame.chroma_red is None and frame.chroma_blue is None
assert count == manifest["largeFrames"]
assert (root / manifest["large"]).stat().st_size == manifest["largeBytes"]
assert wigreader.read(root / manifest["failed"])[1] == []
print(f"PASS: actual recorder readback, 24 diagnostic frames and {count} streamed large-session frames")
