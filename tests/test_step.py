"""Model import tests: STL dispatch, STEP backend detection and parsing.

The STEP file in ``tests/assets`` is a small public sample from the cascadio
project (MIT) used only for local verification.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

from wt901.gui.model_io import StepSupportMissing, load_model, load_step

ASSET = Path(__file__).parent / "assets" / "material.stp"


def _cascadio_available() -> bool:
    try:
        import cascadio  # noqa: F401
    except ImportError:
        return False
    return True


class TestDispatch(unittest.TestCase):
    def setUp(self) -> None:
        self._dir = tempfile.mkdtemp(prefix="model_io_")

    def test_unknown_suffix(self) -> None:
        path = Path(self._dir) / "thing.obj"
        path.write_text("v 0 0 0\n")
        with self.assertRaises(ValueError):
            load_model(path)

    def test_stl_dispatch(self) -> None:
        # a minimal binary STL: one triangle
        import struct

        path = Path(self._dir) / "tri.stl"
        data = b"\0" * 80 + struct.pack("<I", 1)
        data += struct.pack("<3f", 0.0, 0.0, 1.0)
        for v in ((0, 0, 0), (1, 0, 0), (0, 1, 0)):
            data += struct.pack("<3f", *v)
        data += struct.pack("<H", 0)
        path.write_bytes(data)

        mesh = load_model(path)
        self.assertEqual(mesh.triangle_count, 1)


class TestStepParsing(unittest.TestCase):
    def test_obj_parser_handles_quad_faces(self) -> None:
        from wt901.gui.model_io import _parse_obj

        text = "\n".join(
            [
                "v 0 0 0",
                "v 1 0 0",
                "v 1 1 0",
                "v 0 1 0",
                "f 1/1/1 2/1/1 3/1/1 4/1/1",
            ]
        )
        mesh = _parse_obj(text, "quad.obj")
        self.assertEqual(len(mesh.vertices), 4)
        self.assertEqual(mesh.triangle_count, 2)

    def test_obj_parser_negative_indices(self) -> None:
        from wt901.gui.model_io import _parse_obj

        text = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n"
        mesh = _parse_obj(text, "neg.obj")
        self.assertEqual(mesh.triangles, [(0, 1, 2)])


@unittest.skipUnless(ASSET.exists(), "sample STEP asset missing")
class TestStepBackend(unittest.TestCase):
    def test_missing_backend_message(self) -> None:
        if _cascadio_available():
            self.skipTest("cascadio installed; the missing-backend path is not reachable")
        with self.assertRaises(StepSupportMissing):
            load_step(ASSET)

    @unittest.skipUnless(_cascadio_available(), "cascadio not installed")
    def test_converts_real_step(self) -> None:
        mesh = load_step(ASSET)
        self.assertGreater(mesh.triangle_count, 0)
        self.assertGreater(len(mesh.vertices), 0)
        size = mesh.size()
        # sample part is roughly 60-70 mm across
        self.assertGreater(max(size), 50.0)
        self.assertLess(max(size), 80.0)


if __name__ == "__main__":
    unittest.main()
