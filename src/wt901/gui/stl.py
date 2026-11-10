"""Minimal STL loader for the 3D view (binary and ASCII).

No third-party dependency: binary STL is a fixed 50-byte record layout and the
ASCII form is a simple line format.  Meshes are normalised so the model's
bounding-box centre sits at the scene origin -- that is what "坐标原点在模型里"
needs: the origin ends up inside the imported object instead of floating beside
it.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class StlMesh:
    name: str
    vertices: List[Vec3]
    triangles: List[Tuple[int, int, int]]

    # ------------------------------------------------------------------ info
    def bounds(self) -> Tuple[Vec3, Vec3]:
        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        zs = [v[2] for v in self.vertices]
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    @property
    def triangle_count(self) -> int:
        return len(self.triangles)

    def size(self) -> Vec3:
        lo, hi = self.bounds()
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    def center(self) -> Vec3:
        lo, hi = self.bounds()
        return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)

    # ------------------------------------------------------------- transform
    def transformed(self, *, offset: Vec3 = (0.0, 0.0, 0.0), scale: float = 1.0) -> "StlMesh":
        verts = [
            (v[0] * scale + offset[0], v[1] * scale + offset[1], v[2] * scale + offset[2])
            for v in self.vertices
        ]
        return StlMesh(self.name, verts, list(self.triangles))

    def centered(self, *, scale: float = 1.0, target_max_mm: float | None = None) -> "StlMesh":
        """Scale to a target size (optional) and move the bbox centre to origin."""

        size = self.size()
        longest = max(size) or 1.0
        if target_max_mm and longest > 0:
            scale *= target_max_mm / longest
        c = self.center()
        return self.transformed(offset=(-c[0] * scale, -c[1] * scale, -c[2] * scale), scale=scale)

    def with_top_at_origin(self) -> "StlMesh":
        """Put the model's top-centre at the origin (used for a base/stand)."""

        lo, hi = self.bounds()
        cx = (lo[0] + hi[0]) / 2
        cy = (lo[1] + hi[1]) / 2
        return self.transformed(offset=(-cx, -cy, -hi[2]))

    def simplified(self, max_triangles: int) -> "StlMesh":
        """Uniformly subsample triangles so the painter stays responsive."""

        if self.triangle_count <= max_triangles:
            return self
        step = max(1, math.ceil(self.triangle_count / max_triangles))
        tris = self.triangles[::step]
        used = sorted({i for tri in tris for i in tri})
        remap = {old: new for new, old in enumerate(used)}
        verts = [self.vertices[i] for i in used]
        return StlMesh(self.name, verts, [(remap[a], remap[b], remap[c]) for a, b, c in tris])


def _binary(path: Path, data: bytes) -> StlMesh:
    count = struct.unpack_from("<I", data, 80)[0]
    verts: List[Vec3] = []
    tris: List[Tuple[int, int, int]] = []
    offset = 84
    index: dict[Vec3, int] = {}
    for _ in range(count):
        if offset + 50 > len(data):
            break
        values = struct.unpack_from("<12f", data, offset)
        tri = []
        for k in range(3):
            v = (values[3 + k * 3], values[4 + k * 3], values[5 + k * 3])
            idx = index.get(v)
            if idx is None:
                idx = len(verts)
                index[v] = idx
                verts.append(v)
            tri.append(idx)
        tris.append((tri[0], tri[1], tri[2]))
        offset += 50
    return StlMesh(path.name, verts, tris)


def _ascii(path: Path, text: str) -> StlMesh:
    verts: List[Vec3] = []
    tris: List[Tuple[int, int, int]] = []
    index: dict[Vec3, int] = {}
    tri: List[int] = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            v = (float(parts[1]), float(parts[2]), float(parts[3]))
            idx = index.get(v)
            if idx is None:
                idx = len(verts)
                index[v] = idx
                verts.append(v)
            tri.append(idx)
        elif len(parts) >= 1 and parts[0].lower() == "endfacet":
            if len(tri) == 3:
                tris.append((tri[0], tri[1], tri[2]))
            tri = []
    return StlMesh(path.name, verts, tris)


def load_stl(path: str | Path) -> StlMesh:
    p = Path(path)
    data = p.read_bytes()
    # Binary STL: 80-byte header + uint32 count, and the size must match exactly.
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            mesh = _binary(p, data)
            if mesh.triangles:
                return mesh
    text = data.decode("utf-8", errors="ignore")
    if "facet" in text.lower():
        mesh = _ascii(p, text)
        if mesh.triangles:
            return mesh
    raise ValueError(f"无法识别的 STL 文件：{p.name}（既不是二进制也不是 ASCII）")
