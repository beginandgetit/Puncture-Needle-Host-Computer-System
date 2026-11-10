"""Model import: STL natively, STEP through an optional CAD kernel.

STL already stores triangles, so it is parsed directly (``stl.py``).  STEP is a
B-rep format (surfaces, not triangles) and needs a CAD kernel to tessellate it.
Rather than shipping a 100 MB dependency with the SDK, the STEP path is
*optional*: if ``cascadio`` (a small OpenCASCADE-based converter) is installed,
STEP is converted to OBJ and parsed here; otherwise the caller gets a clear,
actionable error instead of a crash.

Install once with::

    python -m pip install cascadio
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import List, Tuple

from .stl import StlMesh, load_stl

SUPPORTED_SUFFIXES = (".stl", ".stp", ".step", ".gltf", ".glb")

STEP_INSTALL_HINT = (
    "导入 STEP/STP 需要 CAD 内核扩展，请先安装：\n"
    "    python -m pip install cascadio\n"
    "或者在 CAD 软件里导出为 STL 后再导入（无需任何额外依赖）。"
)

GLTF_INSTALL_HINT = (
    "导入 glTF/GLB 需要 trimesh 扩展，请先安装：\n"
    "    python -m pip install trimesh\n"
    "或者在导出时选择 STL / STEP 格式。"
)


class BackendMissing(RuntimeError):
    """A model format needs an optional backend that is not installed."""


class StepSupportMissing(BackendMissing):
    """Raised when a STEP file is given but the CAD backend is unavailable."""


class GltfSupportMissing(BackendMissing):
    """Raised when a glTF/GLB file is given but trimesh is unavailable."""


def load_model(path: str | Path) -> StlMesh:
    """Load an STL / STEP / glTF model, dispatching on the file suffix."""

    p = Path(path)
    suffix = p.suffix.lower()
    if suffix == ".stl":
        return load_stl(p)
    if suffix in (".stp", ".step"):
        return load_step(p)
    if suffix in (".gltf", ".glb"):
        return load_gltf(p)
    raise ValueError(
        f"不支持的模型格式 {suffix or '(无扩展名)'}，支持：{', '.join(SUPPORTED_SUFFIXES)}"
    )


def load_gltf(path: str | Path, *, scale_meters_below: float = 1.0) -> StlMesh:
    """Load glTF/GLB through trimesh and normalise the unit to millimetres.

    glTF has no unit field: the convention is metres, but exporters often emit
    millimetres.  If the whole model is smaller than ``scale_meters_below`` it is
    assumed to be in metres and scaled by 1000; otherwise it is used as-is.
    """

    p = Path(path)
    try:
        import trimesh  # type: ignore
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise GltfSupportMissing(GLTF_INSTALL_HINT) from exc

    try:
        loaded = trimesh.load(str(p), force="scene")
    except Exception as exc:  # noqa: BLE001 - parser errors are opaque
        raise ValueError(f"glTF 解析失败：{p.name}（{exc}）") from exc

    mesh = None
    if hasattr(loaded, "to_geometry"):
        try:
            mesh = loaded.to_geometry()
        except Exception:  # noqa: BLE001 - scene may hold non-mesh geometry
            mesh = None
    if mesh is None and hasattr(loaded, "dump"):
        try:
            mesh = loaded.dump(concatenate=True)
        except Exception:  # noqa: BLE001
            mesh = None
    if mesh is None:
        mesh = loaded

    faces = getattr(mesh, "faces", None)
    vertices = getattr(mesh, "vertices", None)
    if faces is None or vertices is None or len(faces) == 0:
        raise ValueError(f"glTF 中没有可用的三角面：{p.name}")

    result = StlMesh(
        p.name,
        [(float(v[0]), float(v[1]), float(v[2])) for v in vertices],
        [(int(f[0]), int(f[1]), int(f[2])) for f in faces],
    )
    if max(result.size()) <= scale_meters_below:
        result = result.transformed(scale=1000.0)
    return result


def load_step(path: str | Path) -> StlMesh:
    p = Path(path)
    try:
        import cascadio  # type: ignore
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise StepSupportMissing(STEP_INSTALL_HINT) from exc

    with tempfile.TemporaryDirectory(prefix="wt901_step_") as tmp:
        obj_path = Path(tmp) / "model.obj"
        try:
            cascadio.step_to_obj(str(p), str(obj_path))
        except Exception as exc:  # noqa: BLE001 - kernel errors are opaque
            raise ValueError(f"STEP 解析失败：{p.name}（{exc}）") from exc

        if not obj_path.exists() or obj_path.stat().st_size == 0:
            raise ValueError(f"STEP 未生成任何几何：{p.name}")

        text = obj_path.read_text(encoding="utf-8", errors="ignore")

    mesh = _parse_obj(text, p.name)
    if not mesh.triangles:
        raise ValueError(f"STEP 中没有可用的三角面：{p.name}")
    return mesh


def _parse_obj(text: str, name: str) -> StlMesh:
    vertices: List[Tuple[float, float, float]] = []
    triangles: List[Tuple[int, int, int]] = []

    for line in text.splitlines():
        if line.startswith("v "):
            parts = line.split()
            if len(parts) >= 4:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif line.startswith("f "):
            indices: List[int] = []
            for token in line.split()[1:]:
                raw = token.split("/")[0]
                if not raw:
                    continue
                idx = int(raw)
                indices.append(idx - 1 if idx > 0 else len(vertices) + idx)
            # fan triangulation for quads/ngons
            for k in range(1, len(indices) - 1):
                triangles.append((indices[0], indices[k], indices[k + 1]))

    return StlMesh(name, vertices, triangles)
