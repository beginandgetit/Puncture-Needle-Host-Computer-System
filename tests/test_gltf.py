"""glTF/GLB import tests (skipped when trimesh is not installed)."""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

from wt901.gui.model_io import GltfSupportMissing, load_gltf, load_model


def _trimesh_available() -> bool:
    try:
        import trimesh  # noqa: F401
    except ImportError:
        return False
    return True


@unittest.skipUnless(_trimesh_available(), "trimesh not installed")
class TestGltf(unittest.TestCase):
    def setUp(self) -> None:
        import trimesh

        self._dir = tempfile.mkdtemp(prefix="gltf_")
        self._trimesh = trimesh

    def _export(self, ext: str, extents=(1.0, 1.0, 1.0)) -> Path:
        mesh = self._trimesh.creation.box(extents=extents)
        path = Path(self._dir) / f"box.{ext}"
        mesh.export(str(path))
        return path

    def test_glb(self) -> None:
        path = self._export("glb")
        mesh = load_gltf(path)
        self.assertEqual(mesh.triangle_count, 12)
        self.assertEqual(len(mesh.vertices), 8)
        # 1 unit is assumed to be metres -> 1000 mm
        self.assertAlmostEqual(max(mesh.size()), 1000.0, delta=1.0)

    def test_gltf(self) -> None:
        path = self._export("gltf")
        mesh = load_gltf(path)
        self.assertEqual(mesh.triangle_count, 12)

    def test_millimetre_model_is_not_rescaled(self) -> None:
        path = self._export("glb", extents=(150.0, 20.0, 5.0))
        mesh = load_gltf(path)
        self.assertAlmostEqual(max(mesh.size()), 150.0, delta=0.5)

    def test_dispatch_through_load_model(self) -> None:
        path = self._export("glb")
        mesh = load_model(path)
        self.assertGreater(mesh.triangle_count, 0)


@unittest.skipIf(_trimesh_available(), "trimesh installed; missing-backend path unreachable")
class TestGltfMissingBackend(unittest.TestCase):
    def test_message(self) -> None:
        with self.assertRaises(GltfSupportMissing):
            load_gltf(Path("does-not-exist.glb"))


if __name__ == "__main__":
    unittest.main()
