"""Generate a rectangular base model (STL, optionally GLB).

    python tools/make_box.py --size 120 80 10 --out examples/base_box.stl
    python tools/make_box.py --size 200 60 8 --glb      # also writes a .glb

By default the box is centred in X/Y and its **top face sits at z = 0**, which is
exactly where the app expects a base ("导入基座" puts the model's top at the
origin).  Use --bottom-at-zero for a box standing on the ground plane instead.

All dimensions are millimetres, matching the CAD assembly.
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path
from typing import List, Tuple

Tri = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]


def box_triangles(sx: float, sy: float, sz: float, *, top_at_zero: bool = True) -> List[Tri]:
    hx, hy = sx / 2.0, sy / 2.0
    z_top = 0.0 if top_at_zero else sz
    z_bot = z_top - sz

    v = [
        (-hx, -hy, z_bot), (hx, -hy, z_bot), (hx, hy, z_bot), (-hx, hy, z_bot),
        (-hx, -hy, z_top), (hx, -hy, z_top), (hx, hy, z_top), (-hx, hy, z_top),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),      # bottom
        (4, 6, 5), (4, 7, 6),      # top
        (0, 4, 5), (0, 5, 1),      # -Y
        (1, 5, 6), (1, 6, 2),      # +X
        (2, 6, 7), (2, 7, 3),      # +Y
        (3, 7, 4), (3, 4, 0),      # -X
    ]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def write_stl(path: Path, tris: List[Tri]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as fh:
        fh.write(b"wt901 base box".ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            ux, uy, uz = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            vx, vy, vz = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            fh.write(struct.pack("<3f", nx / norm, ny / norm, nz / norm))
            for p in (a, b, c):
                fh.write(struct.pack("<3f", *p))
            fh.write(struct.pack("<H", 0))


def write_glb(path: Path, tris: List[Tri], name: str) -> bool:
    try:
        import trimesh  # optional
    except ImportError:
        return False

    vertices = [p for tri in tris for p in tri]
    faces = [(i * 3, i * 3 + 1, i * 3 + 2) for i in range(len(tris))]
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.export(str(path))
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", nargs=3, type=float, metavar=("X", "Y", "Z"), default=[120.0, 80.0, 10.0])
    ap.add_argument("--out", default="examples/base_box.stl")
    ap.add_argument("--glb", action="store_true", help="also write a .glb next to the STL")
    ap.add_argument("--bottom-at-zero", action="store_true", help="stand on z=0 instead of top at z=0")
    args = ap.parse_args()

    sx, sy, sz = args.size
    if min(sx, sy, sz) <= 0:
        raise SystemExit("尺寸必须为正数")

    tris = box_triangles(sx, sy, sz, top_at_zero=not args.bottom_at_zero)
    out = Path(args.out)
    write_stl(out, tris)
    print(f"STL: {out.resolve()}  ({sx:g} x {sy:g} x {sz:g} mm, {len(tris)} 面)")

    if args.glb:
        glb = out.with_suffix(".glb")
        if write_glb(glb, tris, out.stem):
            print(f"GLB: {glb.resolve()}")
        else:
            print("GLB 跳过：未安装 trimesh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
