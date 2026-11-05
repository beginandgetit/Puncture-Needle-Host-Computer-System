"""STL loader tests: binary, ASCII, normalisation and simplification."""

from __future__ import annotations

import os
import struct
import tempfile
import unittest

from wt901.gui.stl import StlMesh, load_stl

CUBE_TRIS = [
    ((0, 0, 0), (1, 0, 0), (0, 1, 0)),
    ((1, 0, 0), (1, 1, 0), (0, 1, 0)),
    ((0, 0, 1), (1, 0, 1), (0, 1, 1)),
    ((1, 0, 1), (1, 1, 1), (0, 1, 1)),
]


def binary_stl(tris) -> bytes:
    out = bytearray(b"\0" * 80)
    out += struct.pack("<I", len(tris))
    for a, b, c in tris:
        out += struct.pack("<3f", 0.0, 0.0, 1.0)
        for v in (a, b, c):
            out += struct.pack("<3f", *v)
        out += struct.pack("<H", 0)
    return bytes(out)


def ascii_stl(tris) -> str:
    lines = ["solid cube"]
    for a, b, c in tris:
        lines.append("  facet normal 0 0 1")
        lines.append("    outer loop")
        for v in (a, b, c):
            lines.append(f"      vertex {v[0]} {v[1]} {v[2]}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid cube")
    return "\n".join(lines)


class TestStlLoader(unittest.TestCase):
    def setUp(self) -> None:
        self._dir = tempfile.mkdtemp(prefix="stl_")

    def _write(self, name: str, data: bytes) -> str:
        path = os.path.join(self._dir, name)
        with open(path, "wb") as fh:
            fh.write(data)
        return path

    def test_binary(self) -> None:
        path = self._write("cube.stl", binary_stl(CUBE_TRIS))
        mesh = load_stl(path)
        self.assertEqual(mesh.triangle_count, 4)
        self.assertEqual(len(mesh.vertices), 8)
        self.assertEqual(mesh.size(), (1.0, 1.0, 1.0))
        self.assertEqual(mesh.center(), (0.5, 0.5, 0.5))

    def test_ascii(self) -> None:
        path = self._write("cube_ascii.stl", ascii_stl(CUBE_TRIS).encode("utf-8"))
        mesh = load_stl(path)
        self.assertEqual(mesh.triangle_count, 4)
        self.assertEqual(len(mesh.vertices), 8)

    def test_centered_moves_origin_into_model(self) -> None:
        mesh = load_stl(self._write("cube.stl", binary_stl(CUBE_TRIS)))
        moved = mesh.centered()
        cx, cy, cz = moved.center()
        self.assertAlmostEqual(cx, 0.0, places=6)
        self.assertAlmostEqual(cy, 0.0, places=6)
        self.assertAlmostEqual(cz, 0.0, places=6)
        # the origin must now be inside the bounding box
        lo, hi = moved.bounds()
        self.assertLessEqual(lo[0], 0.0)
        self.assertGreaterEqual(hi[0], 0.0)

    def test_centered_scales_to_target(self) -> None:
        mesh = load_stl(self._write("cube.stl", binary_stl(CUBE_TRIS)))
        scaled = mesh.centered(target_max_mm=10.0)
        self.assertAlmostEqual(max(scaled.size()), 10.0, places=5)

    def test_top_at_origin(self) -> None:
        mesh = load_stl(self._write("cube.stl", binary_stl(CUBE_TRIS)))
        moved = mesh.with_top_at_origin()
        lo, hi = moved.bounds()
        self.assertAlmostEqual(hi[2], 0.0, places=6)
        self.assertAlmostEqual((lo[0] + hi[0]) / 2, 0.0, places=6)

    def test_simplified(self) -> None:
        many = CUBE_TRIS * 50
        mesh = load_stl(self._write("many.stl", binary_stl(many)))
        self.assertEqual(mesh.triangle_count, 200)
        small = mesh.simplified(20)
        self.assertLessEqual(small.triangle_count, 20)

    def test_invalid_file(self) -> None:
        path = self._write("bad.stl", b"not an stl at all")
        with self.assertRaises(ValueError):
            load_stl(path)

    def test_transformed_does_not_mutate(self) -> None:
        mesh = StlMesh("m", [(0, 0, 0), (1, 0, 0), (0, 1, 0)], [(0, 1, 2)])
        moved = mesh.transformed(offset=(1, 0, 0))
        self.assertEqual(mesh.vertices[0], (0, 0, 0))
        self.assertEqual(moved.vertices[0], (1, 0, 0))


if __name__ == "__main__":
    unittest.main()
