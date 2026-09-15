"""Check the checked-in target graph without requiring Xcode."""
from collections import Counter
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / "Wiggler.xcodeproj/project.pbxproj"
OBJECT = re.compile(r"^\s*([A-F0-9]{24})(?: /\* [^\n]*? \*/)? = \{(.*?)\};", re.M | re.S)
REFERENCE = re.compile(r"\b[A-F0-9]{24}\b")


class XcodeProjectTests(unittest.TestCase):
    def setUp(self):
        self.source = PROJECT.read_text()
        self.entries = OBJECT.findall(self.source)
        self.objects = dict(self.entries)

    def test_object_ids_are_unique(self):
        duplicates = [key for key, count in Counter(key for key, _ in self.entries).items() if count > 1]
        self.assertEqual(duplicates, [], "Xcode objects overwrite one another")

    def test_references_resolve(self):
        self.assertEqual(set(REFERENCE.findall(self.source)) - self.objects.keys(), set())

    def target_files(self, phase):
        targets = [body for body in self.objects.values() if "isa = PBXNativeTarget;" in body]
        self.assertEqual(len(targets), 1)
        phase_ids = REFERENCE.findall(re.search(r"buildPhases = \((.*?)\);", targets[0], re.S)[1])
        phases = [self.objects[key] for key in phase_ids if f"isa = {phase};" in self.objects[key]]
        self.assertEqual(len(phases), 1)
        build_ids = REFERENCE.findall(re.search(r"files = \((.*?)\);", phases[0], re.S)[1])
        paths = []
        for key in build_ids:
            file_ref = re.search(r"fileRef = ([A-F0-9]{24})", self.objects[key])
            if file_ref:
                paths.append(re.search(r"\bpath = ([^;]+);", self.objects[file_ref[1]])[1].strip('"'))
        return paths

    def test_every_app_source_is_compiled_once(self):
        actual = self.target_files("PBXSourcesBuildPhase")
        expected = [path.name for path in (ROOT / "Wiggler").glob("*.swift")]
        self.assertCountEqual(actual, expected)

    def test_zlib_is_linked_once(self):
        paths = self.target_files("PBXFrameworksBuildPhase")
        self.assertEqual(paths.count("usr/lib/libz.tbd"), 1)


if __name__ == "__main__":
    unittest.main()
