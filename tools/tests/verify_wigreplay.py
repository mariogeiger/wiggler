#!/usr/bin/env python3
"""Exercise wigreplay's algorithm routing with a minimal v2 sensor recording."""

import json
from pathlib import Path
import re
import struct
import subprocess
import sys


def write_chunk(stream, payload):
    stream.write(struct.pack("<I", len(payload)))
    stream.write(payload)


def historical_config():
    return {
        "targetTrackCount": 23,
        "explorationPoints": 7,
        "pyramidLevels": 4,
        "maxResidual": 0.12,
        "minDepthConfidence": 1,
        "chordMinMeters": 0.02,
        "chordMaxFrames": 45,
        "chordStride": 3,
        "axisUpdateInterval": 10,
        "constraintWindowFrames": 900,
        "sampleHistory": 90,
        "lockedDriftFrames": 3,
        "axisConstraintCapacity": 321,
        "recentWindowFrames": 45,
        "axisMovedMeters": 0.02,
        "maxDepthJumpFraction": 0.08,
        "minHealthyInliers": 12,
        "staleLibrarySeconds": 2.0,
        "relocRefreshAlpha": 0.05,
        "descriptorSide": 18,
        "keyframeBins": 17,
        "lostAfterFrames": 45,
        "roiRadiusFraction": 0.35,
        "reacquisitionDelaySeconds": 5.0,
    }


def frame_metadata(index, algorithm, revision, alternate_live_output=False):
    width, height = 96, 72
    metadata = {
        "t": index / 10,
        "width": width,
        "height": height,
        "fx": 80.0,
        "fy": 80.0,
        "cx": width / 2,
        "cy": height / 2,
        "rotation": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "translation": [0.0, 0.0, 0.0],
        "poseValid": True,
        "depthWidth": 0,
        "depthHeight": 0,
        "chromaRedWidth": 0,
        "chromaRedHeight": 0,
        "chromaBlueWidth": 0,
        "chromaBlueHeight": 0,
        "state": "lost" if alternate_live_output else "locked",
        "theta": 1000.0 - index if alternate_live_output else index / 10,
        "angleMeasured": not alternate_live_output,
        "axisGeneration": 900 + index if alternate_live_output else 0,
        "rpm": -123.0 if alternate_live_output else 456.0,
        "marker": {
            "x": -100.0 if alternate_live_output else 100.0,
            "y": 500.0 if alternate_live_output else 50.0,
            "radius": 999.0 if alternate_live_output else 10.0,
        },
        "axisOrigin": [9.0, 8.0, 7.0] if alternate_live_output else [1.0, 2.0, 3.0],
        "axisDirection": [-1.0, 0.0, 0.0] if alternate_live_output else [0.0, 1.0, 0.0],
        "config": historical_config(),
    }
    if algorithm is not None:
        metadata["settings"] = {
            "algorithm": algorithm,
            "algorithmRevision": revision,
        }
    return metadata


def write_recording(path, selections, alternate_live_output=False):
    width, height = 96, 72
    pixels = bytes((x + y) % 256 for y in range(height) for x in range(width))
    with path.open("wb") as stream:
        write_chunk(stream, json.dumps({"version": 2}).encode())
        for index, (algorithm, revision) in enumerate(selections):
            metadata = json.dumps(
                frame_metadata(index, algorithm, revision, alternate_live_output)
            ).encode()
            write_chunk(stream, b"\0" + metadata)
            write_chunk(stream, b"\0" + pixels)
            for _ in range(4):
                write_chunk(stream, b"")


def run(binary, *arguments):
    return subprocess.run(
        [str(binary), *map(str, arguments)],
        check=False,
        text=True,
        capture_output=True,
    )


def json_lines(output):
    return [json.loads(line) for line in output.splitlines()]


def recalculated_output(line):
    ignored = {
        "t",
        "processingMillis",
        "recordedState",
        "recordedAlgorithm",
        "recordedAlgorithmRevision",
    }
    output = {key: value for key, value in line.items() if key not in ignored}
    if "persistentMapDiagnostics" in output:
        output["persistentMapDiagnostics"] = {
            key: value for key, value in output["persistentMapDiagnostics"].items() if key != "millis"
        }
    return output


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: verify_wigreplay.py <wigreplay> <temporary-directory>")
    binary = Path(sys.argv[1])
    temporary = Path(sys.argv[2])
    recording = temporary / "algorithm-switches.wig"
    selections = [
        (None, 0),
        ("trackedGeometry", 0),
        ("persistentMap", 1),
        ("persistentMap", 1),
        ("trackedGeometry", 3),
        ("trackedGeometry", 3),
    ]
    write_recording(recording, selections)

    replay = run(binary, recording)
    require(replay.returncode == 0, replay.stderr)
    lines = json_lines(replay.stdout)
    require(
        [(line["algorithm"], line["algorithmRevision"]) for line in lines]
        == [
            ("trackedGeometry", 0),
            ("trackedGeometry", 0),
            ("persistentMap", 1),
            ("persistentMap", 1),
            ("trackedGeometry", 3),
            ("trackedGeometry", 3),
        ],
        "recorded routing was not reproduced",
    )

    alternate_recording = temporary / "different-live-measurements.wig"
    write_recording(alternate_recording, selections, alternate_live_output=True)
    alternate_replay = run(binary, alternate_recording)
    require(alternate_replay.returncode == 0, alternate_replay.stderr)
    alternate_lines = json_lines(alternate_replay.stdout)
    require(
        [recalculated_output(line) for line in alternate_lines]
        == [recalculated_output(line) for line in lines],
        "recorded live measurements leaked into recalculated estimator output",
    )

    overridden = run(binary, recording, "--algorithm", "persistentMap")
    require(overridden.returncode == 0, overridden.stderr)
    override_lines = json_lines(overridden.stdout)
    require(
        [(line["algorithm"], line["algorithmRevision"]) for line in override_lines]
        == [("persistentMap", revision) for revision in [0, 0, 1, 1, 2, 2]],
        "override did not reset at every recorded segment boundary",
    )

    diagnostics = run(binary, recording, "--diagnostics")
    require(diagnostics.returncode == 0, diagnostics.stderr)
    diagnostic_lines = json_lines(diagnostics.stdout)
    require(
        [line["algorithm"] for line in diagnostic_lines]
        == ["trackedGeometry", "trackedGeometry", "persistentMap", "persistentMap", "trackedGeometry", "trackedGeometry"],
        "diagnostic output lost algorithm provenance",
    )

    require(
        diagnostic_lines[0]["config"]["targetTrackCount"] == 23
        and diagnostic_lines[0]["config"]["axisConstraintCapacity"] == 321
        and diagnostic_lines[0]["config"]["descriptorSide"] == 18
        and diagnostic_lines[0]["config"]["keyframeBins"] == 17,
        "historical constructor configuration was not present in the replay journal",
    )

    rgba = temporary / "harmonic.rgba"
    harmonic = run(binary, recording, "--harmonic", "1", rgba)
    require(harmonic.returncode == 0, harmonic.stderr)
    progress = [float(match.group(1)) for match in re.finditer(r"progress ([0-9.eE+-]+)", harmonic.stdout)]
    require(len(progress) == 6, "harmonic replay did not report every frame")
    require(progress[1] > 0 and progress[2] == 0 and progress[3] > 0 and progress[4] == 0 and progress[5] > 0,
            "harmonic fit was not reset at algorithm boundaries")

    conflict = run(binary, recording, "--algorithm", "persistentMap", "--harmonic", "1", rgba)
    require(conflict.returncode == 2, "algorithm override was accepted with recorded-output harmonic replay")

    unknown = temporary / "unknown-algorithm.wig"
    write_recording(unknown, [("futureEngine", 0)])
    rejected = run(binary, unknown)
    require(rejected.returncode == 1 and "unknown recorded algorithm" in rejected.stderr,
            "unknown recorded algorithm did not fail clearly")

    print("wigreplay algorithm routing: ok")


if __name__ == "__main__":
    main()
