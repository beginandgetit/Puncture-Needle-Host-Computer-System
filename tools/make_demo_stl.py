"""Generate two small demo STL files (needle + base) for the GUI.

    python tools/make_demo_stl.py --out examples

Binary STL, millimetres.  Useful for trying the「导入模型 / 导入基座」buttons
without needing a real CAD export.
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path
from typing import List, Tuple

Tri = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]


def cylinder(tris: List[Tri], radius: float, z0: float, z1: float, segments: int = 32) -> None:
    for i in range(segments):
        a0 = 2 * math.pi * i / segments
        a1 = 2 * math.pi * (i + 1) / segments
        p0 = (radius * math.cos(a0), radius * math.sin(a0))
        p1 = (radius * math.cos(a1), radius * math.sin(a1))
        tris.append(((p0[0], p0[1], z0), (p1[0], p1[1], z0), (p1[0], p1[1], z1)))
        tris.append(((p0[0], p0[1], z0), (p1[0], p1[1], z1), (p0[0], p0[1], z1)))


def cone(tris: List[Tri], radius: float, z0: float, z1: float, segments: int = 32) -> None:
    for i in range(segments):
        a0 = 2 * math.pi * i / segments
        a1 = 2 * math.pi * (i + 1) / segments
        p0 = (radius * math.cos(a0), radius * math.sin(a0))
        p1 = (radius * math.cos(a1), radius * math.sin(a1))
        tris.append(((p0[0], p0[1], z0), (p1[0], p1[1], z0), (0.0, 0.0, z1)))


def disc(tris: List[Tri], radius: float, z: float, segments: int = 32, flip: bool = False) -> None:
    for i in range(segments):
        a0 = 2 * math.pi * i / segments
        a1 = 2 * math.pi * (i + 1) / segments
        p0 = (radius * math.cos(a0), radius * math.sin(a0), z)
        p1 = (radius * math.cos(a1), radius * math.sin(a1), z)
        if flip:
            tris.append((p1, p0, (0.0, 0.0, z)))
        else:
            tris.append((p0, p1, (0.0, 0.0, z)))


def box(tris: List[Tri], sx: float, sy: float, sz: float, z_top: float) -> None:
    x, y = sx / 2, sy / 2
    z0, z1 = z_top - sz, z_top
    v = [
        (-x, -y, z0), (x, -y, z0), (x, y, z0), (-x, y, z0),
        (-x, -y, z1), (x, -y, z1), (x, y, z1), (-x, y, z1),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3),      # bottom
        (4, 6, 5), (4, 7, 6),      # top
        (0, 4, 5), (0, 5, 1),      # front
        (1, 5, 6), (1, 6, 2),      # right
        (2, 6, 7), (2, 7, 3),      # back
        (3, 7, 4), (3, 4, 0),      # left
    ]
    for a, b, c in faces:
        tris.append((v[a], v[b], v[c]))


def write_stl(path: Path, tris: List[Tri]) -> None:
    with path.open("wb") as fh:
        fh.write(b"wt901 demo".ljust(80, b"\0"))
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="examples")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    needle: List[Tri] = []
    cylinder(needle, 1.2, -140.0, 0.0, segments=36)
    cone(needle, 1.2, -150.0, -140.0, segments=36)
    disc(needle, 1.2, 0.0, segments=36, flip=True)
    disc(needle, 1.2, -140.0, segments=36)
    write_stl(out / "needle_demo.stl", needle)

    base: List[Tri] = []
    box(base, 90.0, 90.0, 8.0, z_top=0.0)
    write_stl(out / "base_demo.stl", base)

    print(f"needle: {len(needle)} tris -> {out / 'needle_demo.stl'}")
    print(f"base:   {len(base)} tris -> {out / 'base_demo.stl'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
