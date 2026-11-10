"""Decimate a triangle mesh (STL/OBJ/PLY/GLB) to a target face count.

Anatomical scans often have 100k-500k triangles, which is too heavy for the
QPainter renderer.  This wraps trimesh's quadric decimation:

    python tools/decimate_stl.py big_skull.stl --out examples/skull_demo.stl --faces 8000

Requires ``trimesh`` and ``fast_simplification``::

    python -m pip install trimesh fast_simplification
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", help="input mesh (stl/obj/ply/glb)")
    ap.add_argument("--out", help="output file (default: <src>_decimated.stl)")
    ap.add_argument("--faces", type=int, default=8000, help="target triangle count")
    args = ap.parse_args()

    try:
        import trimesh
    except ImportError:
        raise SystemExit("需要 trimesh：python -m pip install trimesh fast_simplification")

    src = Path(args.src)
    out = Path(args.out) if args.out else src.with_name(src.stem + "_decimated.stl")

    mesh = trimesh.load(str(src), force="mesh")
    if mesh is None or len(mesh.faces) == 0:
        raise SystemExit(f"没有读到三角面：{src}")

    before = len(mesh.faces)
    try:
        slim = mesh.simplify_quadric_decimation(face_count=args.faces)
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(
            f"简化失败（{exc}）。请安装 fast_simplification：python -m pip install fast_simplification"
        ) from exc

    slim.export(str(out))
    size = slim.bounds[1] - slim.bounds[0]
    print(f"{before} -> {len(slim.faces)} faces")
    print(f"size: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm")
    print(f"written: {out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
