"""装配标定参数（rig）：传感器 ↔ 针 ↔ CAD 模型之间的固定关系。

这些数值来自 `总装.stp` 与 `总装_坐标系数据_*.xlsx` 的实测提取，集中在这里，
改装配/换传感器只改这个 JSON，不动代码。

    config/rig.json

字段含义
--------
``needle_axis_sensor``
    针轴在**传感器本体系**中的方向（单位向量，hub → tip）。传感器手册轴：
    X 东、Y 北、Z 天。装配里针指向 +Y，故默认 (0,1,0)。若安装转了 90°，
    只改这里；也可以用界面「标定针轴（竖直）」自动反解。
``model_axis``
    针轴在**模型文件坐标系**中的方向（导入的 STL/STEP 自己的坐标）。
``model_right``
    模型坐标系里「手柄/PCB 的右方向」。显示时它被放到屏幕 +X，用来固定
    reference 姿态下模型的滚转角；它不影响针轴的相对运动。
``hub_mm``
    针座（旋转中心）在模型坐标系中的位置，mm。
``needle_length_mm``
    针长，用于针尖推算与显示长度参考。
``cad_mode``
    True 时导入的模型按真实 CAD 坐标摆放（不居中、不缩放），并以 ``pivot_mode`` 指定的点
    为旋转中心。
``pivot_mode``
    旋转中心：``"hub"`` = 针座（默认），``"tip"`` = 针尖。
    针尖坐标 = ``hub_mm + model_axis * needle_length_mm``。
``invert_quaternion``
    True 表示设备四元数是「世界 → 传感器」。正常维特输出是「传感器 → 世界」，
    默认 False。实物上表现为：针往右倾、界面往左倾时，把它打开。
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

from .mapping import canonical_placement
from .pose import Quaternion, Vec3, v_add, v_normalize, v_scale, v_sub

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "config" / "rig.json"

PIVOT_HUB = "hub"
PIVOT_TIP = "tip"
PIVOT_MODES = (PIVOT_HUB, PIVOT_TIP)


@dataclass(frozen=True)
class Rig:
    # 传感器本体系中针轴的方向（装配实测：针指向 +Y；安装同向时即 +Y）
    needle_axis_sensor: Vec3 = (0.0, 1.0, 0.0)
    # 模型文件中针轴的方向（总装装配系同上；露出的针指向 +Y）
    model_axis: Vec3 = (0.0, 1.0, 0.0)
    # 模型坐标系中手柄的「右」方向，用于固定 reference 姿态的滚转角
    model_right: Vec3 = (1.0, 0.0, 0.0)
    # 针从手柄伸出的位置（旋转中心），mm
    hub_mm: Vec3 = (0.0, 18.38, 9.56)
    # 露出针长（mm）：194.73 − 18.38
    needle_length_mm: float = 176.35
    # 导入模型是否按 CAD 真实坐标摆放
    cad_mode: bool = True
    # 旋转中心：hub = 针座，tip = 针尖（界面可切换）
    pivot_mode: str = PIVOT_HUB
    # 设备欧拉角顺序（维特默认 ZYX）
    euler_order: str = "ZYX"
    # 设备四元数是否为「世界 → 传感器」
    invert_quaternion: bool = False

    # ------------------------------------------------------------------ io
    def as_dict(self) -> dict:
        data = asdict(self)
        data["needle_axis_sensor"] = list(self.needle_axis_sensor)
        data["model_axis"] = list(self.model_axis)
        data["model_right"] = list(self.model_right)
        data["hub_mm"] = list(self.hub_mm)
        return data

    def save(self, path: str | Path = DEFAULT_PATH) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.as_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return p

    @classmethod
    def load(cls, path: str | Path = DEFAULT_PATH) -> "Rig":
        p = Path(path)
        if not p.exists():
            return cls()
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return cls()
        return cls(
            needle_axis_sensor=tuple(float(v) for v in data.get("needle_axis_sensor", cls.needle_axis_sensor)),
            model_axis=tuple(float(v) for v in data.get("model_axis", cls.model_axis)),
            model_right=tuple(float(v) for v in data.get("model_right", cls.model_right)),
            hub_mm=tuple(float(v) for v in data.get("hub_mm", cls.hub_mm)),
            needle_length_mm=float(data.get("needle_length_mm", cls.needle_length_mm)),
            cad_mode=bool(data.get("cad_mode", cls.cad_mode)),
            pivot_mode=(
                str(data.get("pivot_mode", cls.pivot_mode))
                if str(data.get("pivot_mode", cls.pivot_mode)) in PIVOT_MODES
                else PIVOT_HUB
            ),
            euler_order=str(data.get("euler_order", cls.euler_order)),
            invert_quaternion=bool(data.get("invert_quaternion", cls.invert_quaternion)),
        )

    # ------------------------------------------------------------- geometry
    @property
    def tip_mm(self) -> Vec3:
        """Needle tip in model coordinates: hub + axis * length."""

        return v_add(self.hub_mm, v_scale(v_normalize(self.model_axis), self.needle_length_mm))

    def pivot_mm(self, mode: Optional[str] = None) -> Vec3:
        """Rotation centre in model coordinates for ``mode`` (hub / tip)."""

        selected = self.pivot_mode if mode is None else mode
        return self.tip_mm if selected == PIVOT_TIP else self.hub_mm

    def model_placement(self) -> Quaternion:
        """Fixed model -> display placement at the reference pose."""

        return canonical_placement(self.model_axis, self.model_right)

    def model_alignment(self, display_axis: Vec3) -> Quaternion:
        """Deprecated: shortest-arc axis alignment.

        Kept for callers that only need the axis.  The renderer uses
        :meth:`model_placement` plus the tracked relative rotation so the roll
        about the needle is preserved.
        """

        return Quaternion.from_two_vectors(self.model_axis, display_axis)

    def align_model_point(self, v: Vec3, display_axis: Vec3) -> Vec3:
        """Map a model-space point into the display frame (hub → origin)."""

        q = self.model_alignment(display_axis)
        return q.rotate(v_sub(v, self.hub_mm))


def align_model_point(v: Vec3, model_axis: Vec3, hub: Vec3, display_axis: Vec3) -> Vec3:
    """Pure helper used by the renderer/tests (shortest-arc, axis only)."""

    q = Quaternion.from_two_vectors(model_axis, display_axis)
    return q.rotate(v_sub(v, hub))


def tip_position_display(rig: Rig, display_axis: Vec3) -> Vec3:
    """Where the needle tip ends up in the display frame (hub pivot)."""

    return align_model_point(rig.tip_mm, rig.model_axis, rig.hub_mm, display_axis)